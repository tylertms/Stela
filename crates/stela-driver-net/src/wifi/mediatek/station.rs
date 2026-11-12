// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
//! An attached MT7925 as the connection keeper's radio.
use super::{
    bootstrap::client::{
        self, BssHandle, Event, LinkControlError, LinkEvent, ManagedLink, Phase, PolicyError,
        management::Session,
    },
    mcu::{client::Profile, device::Context},
    regd,
    registers::{BarIo, MmioBar},
    ring::{MmioQueue, QueueIo},
};
use crate::{
    Clock, Link, LinkError, Mac,
    bus::dma::memory::PageSource,
    wifi::{
        Band, Channel,
        connection::keeper::{self, Advertisement, Keeper, Peer, Radio, Stage},
        network::Network,
        regulatory::database::Country,
        scan::{self, result::Bss},
        security::{KeyIdentity, PacketNumber, ReplaySeed},
        station::{
            Join, NativeBssObservation, Station as Association, data::Ingress, external,
            multi_link::Affiliation, wire::six_ghz::Selection,
        },
    },
};
use alloc::{boxed::Box, format, sync::Arc, vec::Vec};
use stela_driver_base::entropy::Entropy;

/// The AP's WCID, the first `mt76_wcid_alloc` gives the sole station.
const PEER_WCID: u16 = 1;

/// An attached MT7925 kept connected to one network.
pub type Wifi<C> = Station<MmioBar, MmioQueue, C, &'static dyn PageSource>;

/// A managed MT7925 link kept connected to one network.
pub type Station<I, Q, C, S> = Keeper<Mt7925<I, Q, C, S>>;

/// A managed MT7925 link and its station interface.
pub struct Mt7925<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> {
    link: ManagedLink<I, Q, C, S>,
    interface: Context,
    country: Country,
}

impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Station<I, Q, C, S> {
    /// Keep `link`, whose station interface is `interface`, connected to
    /// `network` in `country`; `entropy` gives each handshake's SNonce.
    pub fn new(
        link: ManagedLink<I, Q, C, S>,
        interface: Context,
        country: Country,
        network: Network,
        entropy: Arc<dyn Entropy>,
        clock: Clock,
    ) -> Self {
        Keeper::keep(
            Mt7925 {
                link,
                interface,
                country,
            },
            network,
            entropy,
            clock,
        )
    }
}

#[cfg(test)]
impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Mt7925<I, Q, C, S> {
    /// One pass with no keeper behind it, for what the link says after it.
    pub(crate) fn link_mut(&mut self) -> &mut ManagedLink<I, Q, C, S> {
        &mut self.link
    }
}

impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Link for Mt7925<I, Q, C, S> {
    fn mac(&self) -> Mac {
        self.link.mac()
    }
    fn carrier(&mut self) -> bool {
        self.link.carrier()
    }
    fn signals(&self) -> bool {
        self.link.signals()
    }
    fn due_us(&self) -> Option<u64> {
        self.link.due_us()
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.link.transmit(frame)
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.link.receive()
    }
}

impl<I: BarIo, Q: QueueIo, C: 'static, S: PageSource + 'static> Radio for Mt7925<I, Q, C, S> {
    type Handle = BssHandle;
    type Profile = Profile;
    type Management = Arc<Session>;
    type Error = LinkControlError;

    const NAME: &'static str = "MT7925";
    /// mt76 leaves `max_listen_interval` at mac80211's default.
    const LISTEN_INTERVAL: u16 = 5;

    fn deferred(error: &LinkControlError) -> bool {
        matches!(
            error,
            LinkControlError::Backpressure
                | LinkControlError::Policy(PolicyError::Client(client::Error::NeedsDrain))
        )
    }

    fn service(&mut self) -> Result<(), LinkControlError> {
        self.link.service().map(|_| ())
    }

    fn take_event(&mut self) -> Option<keeper::Event> {
        while let Some(notice) = self.link.take_notice() {
            let LinkEvent::Controller(event) = notice.event else {
                continue;
            };
            return Some(match event {
                Event::Ingress(Ingress::ControlPort { ethernet, .. }) => {
                    keeper::Event::Eapol(ethernet)
                }
                Event::ExternalAuthentication(received) => keeper::Event::Sae(Box::new(received)),
                Event::ExternalAuthenticationEnded {
                    failure: Some(failure),
                    ..
                } => keeper::Event::SaeEnded(format!("{failure:?}")),
                Event::Unsupported(operation) => keeper::Event::Unsupported(operation),
                Event::ConnectionLost(_) => keeper::Event::ConnectionLost,
                Event::MichaelMicFailure { group, .. } => {
                    keeper::Event::MichaelFailure { pairwise: !group }
                }
                _ => continue,
            });
        }
        None
    }

    fn stage(&self) -> Stage {
        match self.link.phase() {
            Phase::Idle => Stage::Idle,
            Phase::ScanCompleted => Stage::Scanned,
            Phase::AwaitingModernAdmission => Stage::Admitted,
            Phase::NeedsDrain => Stage::NeedsDrain,
            _ => Stage::Busy,
        }
    }

    fn station(&self) -> &Association {
        self.link.station()
    }

    fn channels(&self, band: Band) -> Vec<(Channel, u32)> {
        self.link.channels(band)
    }

    fn probes(&self, channel: Channel) -> bool {
        self.link.probes(channel)
    }

    fn supported_channels(&self, band: Band) -> &'static [u8] {
        regd::channels(band)
    }

    fn regulated_power_dbm(&self, frequency_mhz: u16) -> i32 {
        self.country.channel(frequency_mhz).max_reg_dbm
    }

    fn advertisement(&self, band: Band) -> Result<Advertisement, LinkControlError> {
        let local = self.link.local_advertisement(band)?;
        Ok(Advertisement {
            capabilities: local.capabilities().clone(),
            elements: local.elements().to_vec(),
            he_source_maps: *local.he_source_maps(),
            // Single-link: an AP MLD is joined as a legacy station on one
            // link, with no Multi-Link element, so the firmware keeps a
            // non-MLD BSS (group 0xff) and a single-link STA_REC, as mt7925
            // does for a vif that is not an MLD. The two-link path stays in
            // the client until it is proven on hardware.
            multi_link: Some(false),
        })
    }

    fn profile(&self, peer: Peer) -> Result<Profile, LinkControlError> {
        let local = self.link.local_advertisement(peer.channel.band)?;
        Ok(Profile {
            interface: self.interface,
            peer: peer.bssid,
            wcid: PEER_WCID,
            channel: peer.channel,
            aid: 0,
            basic_rates: peer.basic_rates,
            supported_rates: peer.supported_rates,
            beacon_interval: peer.beacon_interval,
            dtim_period: peer.dtim_period,
            slot_time: peer.slot_time,
            antenna_mask: local.antenna_mask(),
            // The client takes the cipher from the first installed key.
            cipher: 0,
        })
    }

    fn begin_scan(
        &mut self,
        request: scan::Request,
        budget_us: u64,
    ) -> Result<(), LinkControlError> {
        self.link.begin_scan(request, budget_us).map(|_| ())
    }

    unsafe fn retire_scan(&mut self) -> Result<(), LinkControlError> {
        // SAFETY: the keeper establishes the native client's scan drain.
        unsafe { self.link.retire_scan_after_drain() }.map(|_| ())
    }

    fn six_ghz_scan(
        &self,
        request: &scan::Request,
        options: scan::six_ghz::Options,
    ) -> Result<Option<scan::Request>, LinkControlError> {
        self.link.six_ghz_scan(request, options)
    }

    fn bss(&self) -> Vec<(BssHandle, &Bss)> {
        self.link.bss().collect()
    }

    fn observation(
        &self,
        handle: BssHandle,
    ) -> Result<Arc<NativeBssObservation>, LinkControlError> {
        self.link.observation(handle)
    }

    fn join(
        &mut self,
        handle: BssHandle,
        join: Join,
        profile: Profile,
    ) -> Result<(), LinkControlError> {
        self.link.join(handle, join, profile)
    }

    fn begin_external_authentication(
        &mut self,
        handle: BssHandle,
        profile: Profile,
        multi_link: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, LinkControlError> {
        self.link
            .begin_external_authentication(handle, profile, multi_link, budget_us)
    }

    fn begin_six_ghz_external_authentication(
        &mut self,
        handle: BssHandle,
        selection: Selection,
        profile: Profile,
        _: Option<Affiliation>,
        budget_us: u64,
    ) -> Result<Arc<external::Session>, LinkControlError> {
        self.link
            .begin_six_ghz_external_authentication(handle, selection, profile, budget_us)
    }

    fn transmit_external_authentication(
        &mut self,
        session: &external::Session,
        frame: external::Frame<'_>,
    ) -> Result<(), LinkControlError> {
        self.link
            .transmit_external_authentication(session, frame)
            .map(|_| ())
    }

    fn associate_external(
        &mut self,
        session: &external::Session,
        handle: BssHandle,
        join: Join,
    ) -> Result<(), LinkControlError> {
        self.link.associate_external(session, handle, join)
    }

    fn configure(&mut self) -> Result<(), LinkControlError> {
        self.link.configure()
    }

    fn send_control(
        &mut self,
        ethernet: &[u8],
        key: Option<KeyIdentity>,
    ) -> Result<(), LinkControlError> {
        self.link.send_control(ethernet, key).map(|_| ())
    }

    fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<KeyIdentity, LinkControlError> {
        self.link.install_key(pairwise, index, material, receive)
    }

    fn install_link_key(
        &mut self,
        link_id: u8,
        index: u8,
        material: &[u8],
        receive: ReplaySeed,
    ) -> Result<(), LinkControlError> {
        self.link
            .install_link_key(link_id, index, material, receive)
            .map(|_| ())
    }

    unsafe fn install_link_management_key(
        &mut self,
        link_id: u8,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<bool, LinkControlError> {
        // SAFETY: inherited from the keeper's authenticated key and drain boundary.
        unsafe {
            self.link
                .install_link_management_key(link_id, index, material, receive)
        }?;
        Ok(true)
    }

    unsafe fn install_management_key(
        &mut self,
        epoch: u64,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<Session>, LinkControlError> {
        // SAFETY: forwarded from the keeper.
        unsafe {
            self.link
                .install_management_key(epoch, index, material, receive)
        }
    }

    unsafe fn rotate_management_key(
        &mut self,
        session: &Arc<Session>,
        index: u8,
        material: &[u8],
        receive: PacketNumber,
    ) -> Result<Arc<Session>, LinkControlError> {
        // SAFETY: forwarded from the keeper.
        unsafe {
            self.link
                .rotate_management_key(session, index, material, receive)
        }
    }

    fn authorize_port(&mut self, epoch: u64) -> Result<(), LinkControlError> {
        self.link.authorize_port(epoch)
    }

    fn deauthenticate(&mut self, reason: u16) -> Result<(), LinkControlError> {
        self.link.deauthenticate(reason)
    }

    unsafe fn retire_peer(&mut self) -> Result<(), LinkControlError> {
        // SAFETY: forwarded from the keeper.
        unsafe { self.link.retire_peer_after_drain() }
    }
}
