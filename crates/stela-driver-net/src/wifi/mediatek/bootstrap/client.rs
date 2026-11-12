// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Single-interface MT7925 client ownership. Plain STA_REC installation binds
//! raw-header format to the peer; generic WMWA facts never do.
use super::{Controller, Phase as ControllerPhase};

mod action;
mod association;
mod ba;
mod edca;
mod external;
mod polling;
mod teardown;
pub use edca::AppliedEdca;
mod cqm;
mod csa;
mod departure;
pub use csa::ChannelSwitchStep;
mod local;
pub mod management;
mod modern;
mod multi_link;
mod packet;
mod pmf;
mod six_ghz;
mod txs;
use crate::wifi::station::operations::ChannelDefinition;
pub use txs::TxStatus;
pub use wifi::station::NativeBssObservation as PmfBssObservation;

/// One MSDU to transmit: its Ethernet frame, transmit class and QoS TID.
#[derive(Clone, Copy)]
pub struct Msdu<'a> {
    pub ethernet: &'a [u8],
    pub class: TxClass,
    pub tid: Option<u8>,
}

/// The OS's PMF join request and the BSS observation it selected.
pub struct PmfJoin<'a> {
    pub join: Join,
    pub observed: &'a Arc<PmfBssObservation>,
}

/// A join request, the scanned BSS it names when known, and the profile
/// selected for it.
struct JoinTarget {
    join: Join,
    beacon: Option<scan::result::Bss>,
    profile: Profile,
}
mod policy;
pub use policy::{
    AttachError, BssHandle, Environment, LinkAttachError, LinkControlError, LinkEvent, LinkNotice,
    LinkProgress, ManagedClient, ManagedLink, PolicyError,
};
mod radio;
mod receive;
use crate::{
    bus::dma::memory::PageSource,
    wifi::{
        self,
        frame::{Header, Mpdu},
        mediatek::{
            identity::Chip,
            mailbox::{
                self,
                device::Interface,
                key::{HardwareKey, Removal},
            },
            mcu::{
                client::{self as wire, Profile},
                device::Context,
                key::Context as KeyContext,
                runtime::Command,
            },
            registers::BarIo,
            ring::{
                QueueIo,
                data::{self, Receipt, Released},
            },
            rx, txwi,
        },
        scan,
        security::{
            Cipher, Integrity, KeyIdentity, KeyScope, PacketNumber, ReplaySeed, SecurityStatus,
            management::ManagementCipher,
        },
        station::{
            self, Action, Completion, Join, Station, Ticket,
            data::{Ingress, Protection, SequenceOwner, TxClass},
        },
    },
};
use alloc::{boxed::Box, collections::VecDeque, sync::Arc, vec::Vec};
pub use local::{Advertisement, LocalError, MloObservations};

#[derive(Debug)]
pub enum Error {
    WrongPhase,
    WrongDevice,
    WrongProfile,
    Local(LocalError),
    Busy,
    NoMemory,
    NeedsDrain,
    NoKey,
    Unsupported,
    GrantTimeout,
    ModernAdmissionRequired,
    Regulatory(wifi::regulatory::Error),
    Bootstrap(super::Error),
    Codec(wire::Error),
    Station(station::Error),
    Data(station::data::Error),
    Scan(scan::Error),
    Transport(crate::wifi::mediatek::download::TransportFailure),
    Firmware(u32),
    Key(mailbox::key::Error),
    Ring(data::Error),
    Rx(rx::Error),
    Frame(wifi::frame::Error),
    Aggregate(wifi::security::aggregate::Error),
    IncompleteParent,
    BlockAck(station::ba::Error),
    Reorder(crate::wifi::mediatek::reorder::Error),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MT7925 client: {self:?}")
    }
}
impl core::error::Error for Error {}
/// `ieee80211_drop_unencrypted_mgmt`'s verdict on a PMF association's robust
/// management frame.
enum Unencrypted {
    Accept,
    Drop,
    /// `cfg80211_rx_unprot_mlme_mgmt`, then drop.
    Report,
    /// An unprotected departure before the handshake's keys.
    Depart,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Phase {
    Idle,
    Scanning,
    ScanCompleted,
    Joining,
    AwaitingModernAdmission,
    Associated,
    Removing,
    NeedsDrain,
    Quarantined,
    ControllerDrain,
    Stopped,
}
/// Per-packet kept device/peer/key generation, not a copied current
/// epoch lookup. DataRing owns it through descriptor + TX-free + unmap.
pub struct PacketOwner<C: 'static> {
    resource: Arc<C>,
    association: u64,
    key: Option<KeyIdentity>,
    context: Option<Arc<station::AssociationContext>>,
    tid: Option<u8>,
    /// Sent at the minimum rate, as control-port frames are.
    control_port: bool,
    /// The TXWI packet ID; one of at least 3 asked for TX status.
    pid: u8,
    tx_ba: Option<Arc<station::ba::tx::Session>>,
    authentication: Option<Arc<station::external::Session>>,
    management: Option<Arc<management::Session>>,
}
#[cfg(test)]
impl PacketOwner<()> {
    pub(crate) fn for_tests(association: u64, pid: u8) -> Self {
        Self {
            resource: Arc::new(()),
            association,
            key: None,
            context: None,
            tid: None,
            control_port: false,
            pid,
            tx_ba: None,
            authentication: None,
            management: None,
        }
    }
}
impl<C> PacketOwner<C> {
    pub fn association_epoch(&self) -> u64 {
        self.association
    }
    pub const fn tid(&self) -> Option<u8> {
        self.tid
    }
    pub const fn control_port(&self) -> bool {
        self.control_port
    }
    pub fn authentication_session(&self) -> Option<&Arc<station::external::Session>> {
        self.authentication.as_ref()
    }
    pub fn management_session(&self) -> Option<&Arc<management::Session>> {
        self.management.as_ref()
    }
    pub fn key(&self) -> Option<KeyIdentity> {
        self.key
    }
    pub fn resource(&self) -> &C {
        &self.resource
    }
    pub fn association_context(&self) -> Option<&station::AssociationContext> {
        self.context.as_deref()
    }
}
struct Owned<I: BarIo, Q: QueueIo, C: 'static> {
    controller: Controller<I, Q, C>,
    interface: Interface<Arc<C>>,
    station: Station,
    /// Each scan observation with the time it was seen, kept for
    /// [`policy::BSS_EXPIRE_US`] as cfg80211 keeps its BSS table.
    pmf_observations: Vec<(u64, Arc<PmfBssObservation>)>,
    fragments: station::fragments::Fragments,
    txs: txs::State,
    pmf_selected: Option<Arc<PmfBssObservation>>,
    pmf_authentication: Option<wifi::regulatory::PmfAuthenticationBinding>,
    scanner: scan::Scanner,
    phase: Phase,
    profile: Option<Profile>,
    ciphers: Option<(Cipher, Cipher)>,
    /// The PMF selection's group management suite.
    management_cipher: Option<ManagementCipher>,
    peer_generation: u64,
    reserved_wcid: Option<u16>,
    format_generation: Option<u64>,
    ptk: Option<HardwareKey<Arc<C>>>,
    gtk: Option<HardwareKey<Arc<C>>>,
    ptk_removed: Option<Removal>,
    gtk_removed: Option<Removal>,
    remove_ticket: Option<Ticket>,
    interface_removed: Option<mailbox::device::Deactivated>,
    scan_ticket: Option<scan::Ticket>,
    scan_sequence: u8,
    replies: Vec<mailbox::UnifiedReply>,
    transport_facts: Vec<mailbox::init::Fact>,
    asynchronous: Vec<crate::wifi::mediatek::ring::rx::Frame>,
    pending_rx: VecDeque<receive::ReceivedFrame>,
    /// Events of a pass that a later frame failed, for the next pass.
    carried_events: Vec<Event>,
    /// The last look at the receive rings found them empty.
    rings_empty: bool,
    receive_groups: receive::Groups<C>,
    receive_ba: [Option<ba::Receive<C>>; 8],
    transmit_ba: [Option<ba::Transmit<C>>; 8],
    ba_actions: VecDeque<station::ba::Action>,
    /// OS action registrations and the replies mac80211 would send.
    action: action::State,
    /// A TX block-ack agreement without A-MSDU cleared `wcid->amsdu`.
    amsdu_off: bool,
    ba_management: Option<Arc<management::Session>>,
    ba_management_pn: Option<PacketNumber>,
    roc_token: u8,
    roc_deadline: Option<u64>,
    modern: Option<modern::Configuration>,
    management: Option<Arc<management::Session>>,
    edca: Option<edca::Observed<C>>,
    /// A slot time the AP's beacon changed, not yet sent.
    slot_time: Option<u16>,
    /// The OS's RSSI monitor and the AP's averaged beacon signal.
    cqm: cqm::Cqm,
    /// The AP's announced channel switch.
    csa: csa::State,
    /// The firmware holds the BSS in dynamic power save.
    power_save: bool,
    /// The channel the firmware operates when a beacon narrowed it.
    channel: Option<ChannelDefinition>,
    /// A channel a beacon changed, not yet sent; `None` inside when the
    /// change leaves the associated channel, which mac80211 disconnects for.
    channel_change: Option<Option<ChannelDefinition>>,
    authentication_deadline: Option<u64>,
    authentication_end: Option<(Arc<station::external::Session>, Option<station::Failure>)>,
    /// The latest authentication or association frame, whose TX status
    /// paces the exchange (`ifmgd->status_received`).
    mlme_receipt: Option<Receipt>,
    /// The bound 6 GHz transport until its peer retires after drain.
    six_ghz: Option<six_ghz::Bound>,
    /// The interface's own registration while an AP MLD association's
    /// setup link holds its slots.
    single_link: Option<Context>,
    /// An AP MLD association's other links.
    links: Vec<multi_link::Link<C>>,
    /// Deadline of the AP's pending Multi-Link reconfiguration.
    link_removal: Option<u64>,
    /// A reconfiguration reached its deadline or a TID mapping barred a link.
    link_taken: bool,
}
/// On uncertain/live Drop keep the entire controller, station, real key
/// material history and resources. No new owner may reset their PN.
pub struct Client<I: BarIo, Q: QueueIo, C: 'static> {
    owned: Option<Box<Owned<I, Q, C>>>,
}
impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// # Safety
    /// C is the exclusive MT7925 controller/radio/OS-policy guard. It
    /// keeps the complete initial interface + sole peer WCID namespace, actual
    /// board/antenna/MAC resources and nonrevoked source/OS regulatory permission.
    /// Every public publication is serialized with reset, key retirement and
    /// restrictive policy change. Revocation must stop/cancel/drain queued radio
    /// work, not change preflight booleans. Controller and Interface are
    /// from this same requester, after completed bootstrap/source policy.
    /// No external raw mailbox/radio/WTBL or delayed previous peer/scan user
    /// exists; all DMA/IRQ/coherence contracts remain those of Controller.
    /// No temporal key used by any earlier native owner may be reinstalled
    /// with reset transmit counters. Preserve this Client/key history through
    /// reconnect/retry; do not create a replacement owner to bypass it. A real
    /// new OS key exchange must supply fresh TX material for a new owner.
    /// Hardware stop/drain alone never proves old material was unused.
    pub unsafe fn new(
        mut controller: Controller<I, Q, C>,
        interface: Interface<Arc<C>>,
    ) -> Result<Self, Error> {
        if controller.phase() != ControllerPhase::StartFacts
            || controller.state().identity.chip != Chip::Mt7925
        {
            return Err(Error::WrongDevice);
        }
        let m = controller.state().mailbox.as_ref().unwrap();
        let context = interface.registered_context().ok_or(Error::WrongPhase)?;
        if !interface.belongs_to(m)
            || !Arc::ptr_eq(interface.resource(), &controller.state().resource)
        {
            return Err(Error::WrongDevice);
        }
        let chip = controller.state().identity.chip;
        let station = Station::new(context.address).map_err(Error::Station)?;
        let scanner = scan::Scanner::new(context.address).map_err(Error::Scan)?;
        // The attach is complete: the station's commands run from its polls.
        controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .enter_runtime();
        Ok(Self {
            owned: Some(Box::new(Owned {
                controller,
                interface,
                station,
                pmf_observations: Vec::new(),
                fragments: Default::default(),
                txs: txs::State::of(chip),
                pmf_selected: None,
                pmf_authentication: None,
                scanner,
                phase: Phase::Idle,
                profile: None,
                ciphers: None,
                management_cipher: None,
                peer_generation: 0,
                reserved_wcid: None,
                format_generation: None,
                ptk: None,
                gtk: None,
                ptk_removed: None,
                gtk_removed: None,
                remove_ticket: None,
                interface_removed: None,
                scan_ticket: None,
                scan_sequence: 0,
                replies: Vec::new(),
                transport_facts: Vec::new(),
                asynchronous: Vec::new(),
                pending_rx: VecDeque::new(),
                carried_events: Vec::new(),
                rings_empty: true,
                receive_groups: receive::Groups::new(),
                receive_ba: core::array::from_fn(|_| None),
                transmit_ba: core::array::from_fn(|_| None),
                ba_actions: VecDeque::new(),
                action: action::State::default(),
                amsdu_off: false,
                ba_management: None,
                ba_management_pn: None,
                roc_token: 0,
                roc_deadline: None,
                modern: None,
                management: None,
                edca: None,
                slot_time: None,
                cqm: cqm::Cqm::default(),
                csa: csa::State::default(),
                power_save: false,
                channel: None,
                channel_change: None,
                authentication_deadline: None,
                authentication_end: None,
                mlme_receipt: None,
                six_ghz: None,
                single_link: None,
                links: Vec::new(),
                link_removal: None,
                link_taken: false,
            })),
        })
    }
    /// The station command layouts of the controller's chip.
    fn codec(s: &Owned<I, Q, C>) -> wire::Codec {
        wire::Codec::of(s.controller.state().identity.chip)
    }
    fn state(&self) -> &Owned<I, Q, C> {
        self.owned.as_deref().unwrap()
    }
    fn state_mut(&mut self) -> &mut Owned<I, Q, C> {
        self.owned.as_deref_mut().unwrap()
    }
    pub fn phase(&self) -> Phase {
        self.state().phase
    }
    pub fn station(&self) -> &Station {
        &self.state().station
    }
    pub fn replies(&self) -> &[mailbox::UnifiedReply] {
        &self.state().replies
    }
    pub fn transport_facts(&self) -> &[mailbox::init::Fact] {
        &self.state().transport_facts
    }
    fn command(s: &mut Owned<I, Q, C>, command: Command, body: &[u8]) -> Result<(), Error> {
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        s.phase = Phase::Quarantined;
        Self::command_parts(&mut s.controller, &mut s.replies, command, body)
    }
    fn command_parts(
        controller: &mut Controller<I, Q, C>,
        replies: &mut Vec<mailbox::UnifiedReply>,
        command: Command,
        body: &[u8],
    ) -> Result<(), Error> {
        // Reserve before mutation so reply bytes stay owned.
        replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let m = controller.state_mut().mailbox.as_mut().unwrap();
        let result = m.unified(command, body);
        match result {
            Ok(reply) => {
                let status = reply.status;
                let complete = reply.dma_released;
                replies.push(reply);
                if status != 0 {
                    return Err(Error::Firmware(status));
                }
                if !complete {
                    return Err(Error::NeedsDrain);
                }
                Ok(())
            }
            Err(e) => {
                if let Some(r) = m.interrupted_unified_reply() {
                    replies.push(r.clone());
                }
                Err(Error::Transport(e))
            }
        }
    }
    /// Send one station command from an owned state, as [`Self::command`].
    fn send(s: &mut Owned<I, Q, C>, outgoing: &wire::Outgoing) -> Result<(), Error> {
        s.phase = Phase::Quarantined;
        Self::deliver(
            &mut s.controller,
            &mut s.replies,
            &mut s.transport_facts,
            outgoing,
        )
    }
    /// Send one station command: a UNI command whose status result is kept
    /// in `replies`, or a mailbox request whose transport fact is kept in
    /// `facts`.
    fn deliver(
        controller: &mut Controller<I, Q, C>,
        replies: &mut Vec<mailbox::UnifiedReply>,
        facts: &mut Vec<mailbox::init::Fact>,
        outgoing: &wire::Outgoing,
    ) -> Result<(), Error> {
        match outgoing {
            wire::Outgoing::Uni(command, body) => {
                Self::command_parts(controller, replies, *command, body)
            }
            wire::Outgoing::Mailbox(request) => {
                facts.try_reserve(1).map_err(|_| Error::NoMemory)?;
                let fact = controller
                    .state_mut()
                    .mailbox
                    .as_mut()
                    .unwrap()
                    .initial(request)
                    .map_err(Error::Transport)?;
                facts.push(fact);
                Ok(())
            }
        }
    }
    fn action(
        s: &mut Owned<I, Q, C>,
        action: Action,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        Self::action_guarded(s, action, now, source, policy, None)
    }
    fn action_guarded(
        s: &mut Owned<I, Q, C>,
        action: Action,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
        guard: Option<&mut wifi::regulatory::PmfAuthenticationGuard<'_>>,
    ) -> Result<(), Error> {
        match action {
            Action::Transmit {
                ticket,
                channel,
                bytes,
            } => {
                if channel != s.profile.ok_or(Error::WrongProfile)?.channel
                    || !policy.allow_management_tx(channel)
                {
                    return Err(Error::WrongProfile);
                }
                if s.station.requires_protected_management() {
                    Self::qualify_pmf_authentication(
                        s,
                        guard.ok_or(Error::ModernAdmissionRequired)?,
                        Some((ticket, &bytes)),
                    )?;
                }
                Self::reserve_channel(s)?;
                if s.roc_deadline
                    .is_none_or(|d| s.controller.state().control.clock.now_us() >= d)
                {
                    return Err(Error::GrantTimeout);
                }
                s.mlme_receipt = Some(Self::submit(s, &bytes, None, true, true, source)?);
                s.station.submitted(ticket).map_err(Error::Station)?;
            }
            Action::Configure {
                ticket,
                bssid,
                channel,
                request: _,
                response,
                context,
                links,
            } => {
                if let Some(context) = context {
                    return Self::prepare_modern(s, ticket, context, links);
                }
                // Only an EHT association has other links.
                if !links.is_empty() {
                    return Err(Error::WrongProfile);
                }
                let clock = s.controller.state().control.clock;
                let began = clock.now_us();
                let mut p = s.profile.ok_or(Error::WrongProfile)?;
                if p.peer != bssid || p.channel != channel || response.aid == 0 {
                    return Err(Error::WrongProfile);
                }
                // Response provides AID; source/OS observation
                // supplies beacon/DTIM/basic bitmap. Never invent association0.
                p.aid = response.aid;
                let (supported, _) = rates(&response.elements, channel.band)?;
                if p.supported_rates & !supported != 0 {
                    return Err(Error::WrongProfile);
                }
                let codec = Self::codec(s);
                let bss = codec.bss(p, None, true).map_err(Error::Codec)?;
                let peer = codec.peer(p, None, true, true).map_err(Error::Codec)?;
                let broadcast = codec.broadcast(p, None).map_err(Error::Codec)?;
                for body in &bss {
                    Self::command(s, Command::BssInfo, body)?;
                }
                s.controller
                    .state_mut()
                    .control
                    .clear_peer_admission(p.wcid)
                    .map_err(|e| Error::Bootstrap(super::Error::Control(e)))?;
                Self::command(s, Command::StationRecord, &peer)?;
                Self::command(s, Command::StationRecord, &broadcast)?;
                let beacon_filter = codec.beacon_filter(p).map_err(Error::Codec)?;
                Self::send(s, &beacon_filter)?;
                s.transport_facts
                    .try_reserve(1)
                    .map_err(|_| Error::NoMemory)?;
                let filter = codec.filter(p, true).map_err(Error::Codec)?;
                let fact = s
                    .controller
                    .state_mut()
                    .mailbox
                    .as_mut()
                    .unwrap()
                    .initial(&filter)
                    .map_err(Error::Transport)?;
                s.transport_facts.push(fact);
                Self::command(s, Command::BssInfo, &p.awake().map_err(Error::Codec)?)?;
                s.profile = Some(p);
                s.format_generation = Some(s.peer_generation);
                Self::abort_channel(s)?;
                s.station
                    .complete_configuration(
                        ticket,
                        Completion::Accepted,
                        clock
                            .now_us()
                            .checked_sub(began)
                            .and_then(|elapsed| now.checked_add(elapsed))
                            .ok_or(Error::WrongProfile)?,
                    )
                    .map_err(Error::Station)?;
                s.phase = Phase::Associated;
            }
            Action::Remove { ticket, failure } => {
                Self::authentication_ended(s, failure);
                Self::remove(s, ticket)?;
            }
        }
        Ok(())
    }
}
pub enum Event {
    ProtectedManagement(management::Received),
    /// `cfg80211_rx_unprot_mlme_mgmt`: a PMF association dropped this
    /// unprotected deauthentication or disassociation; the OS may check the
    /// AP with an SA Query.
    UnprotectedDeparture(crate::wifi::mediatek::ring::rx::Frame),
    ExternalAuthentication(station::external::Received),
    ExternalAuthenticationEnded {
        session: Arc<station::external::Session>,
        failure: Option<station::Failure>,
    },
    ConnectionLost(crate::wifi::mediatek::ring::rx::Frame),
    /// `cfg80211_michael_mic_failure`: a TKIP frame from `transmitter` failed
    /// its Michael MIC under the group or pairwise key `key_index`.
    MichaelMicFailure {
        transmitter: crate::Mac,
        group: bool,
        key_index: Option<u8>,
    },
    /// The firmware's TX status for a frame that asked for it.
    TxStatus(TxStatus),
    /// `ieee80211_cqm_rssi_notify`: the signal crossed the monitor's
    /// threshold, upward when `high`.
    Rssi {
        rssi: i32,
        high: bool,
    },
    /// `cfg80211_ch_switch_started_notify`: the AP announced a switch to
    /// `definition` in `count` beacons, asking for silence when `block_tx`.
    ChannelSwitchStarted {
        definition: ChannelDefinition,
        count: u8,
        block_tx: bool,
    },
    /// `cfg80211_ch_switch_notify`: the association now operates there.
    ChannelSwitched(ChannelDefinition),
    Bss(scan::result::Bss),
    Ingress(Ingress),
    Raw(crate::wifi::mediatek::ring::rx::Frame),
    /// An operation the association needs that the driver does not perform,
    /// reported once; the driver continues with its documented degraded
    /// response.
    Unsupported(&'static str),
}
impl<I: BarIo, Q: QueueIo, C: 'static> Drop for Client<I, Q, C> {
    fn drop(&mut self) {
        if let Some(s) = self.owned.take()
            && s.phase != Phase::Stopped
        {
            core::mem::forget(s)
        }
    }
}
/// `mt7925_mcu_rssi_monitor_event` over an event payload: each INFO TLV
/// after the four-byte header reports the first TLV's RSSI.
fn rssi_events(payload: &[u8], threshold: i32) -> Vec<Event> {
    let mut out = Vec::new();
    let tlvs = payload.get(4..).unwrap_or_default();
    let Some(rssi) = tlvs
        .get(4..8)
        .map(|b| i32::from_le_bytes(b.try_into().unwrap()))
    else {
        return out;
    };
    let mut at = 0;
    while let Some(tlv) = tlvs.get(at..at + 4) {
        let len = usize::from(u16::from_le_bytes([tlv[2], tlv[3]]));
        if len == 0 || len > tlvs.len() - at {
            break;
        }
        if tlv[..2] == [0, 0] {
            out.push(Event::Rssi {
                rssi,
                high: rssi > threshold,
            });
        }
        at += len;
    }
    out
}
fn rates(elements: &[u8], band: wifi::Band) -> Result<(u16, u16), Error> {
    let table: &[u8] = if band == wifi::Band::Ghz2 {
        &[2, 4, 11, 22, 12, 18, 24, 36, 48, 72, 96, 108]
    } else {
        &[12, 18, 24, 36, 48, 72, 96, 108]
    };
    let mut rest = elements;
    let (mut supported, mut basic) = (0, 0);
    while !rest.is_empty() {
        let n = usize::from(*rest.get(1).ok_or(Error::WrongProfile)?);
        let v = rest.get(2..2 + n).ok_or(Error::WrongProfile)?;
        if matches!(rest[0], 1 | 50) {
            for rate in v {
                if let Some(i) = table.iter().position(|r| *r == rate & 127) {
                    supported |= 1 << i;
                    if rate & 128 != 0 {
                        basic |= 1 << i
                    }
                }
            }
        }
        rest = &rest[2 + n..];
    }
    if supported == 0 {
        return Err(Error::WrongProfile);
    }
    Ok((supported, basic))
}

#[cfg(test)]
pub(crate) mod tests;
