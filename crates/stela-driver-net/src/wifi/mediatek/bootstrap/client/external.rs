// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Native peer and ROC programming follows the existing source-checked client;
// external SAE transport follows Linux mac80211, documented in NOTICE.
//! Preassociation SAE management transport for an OS supplicant. There
//! is no boolean completion API or transition to associated.

use super::*;
use station::{external::Session, multi_link::Affiliation};

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// Claim the same real preassociation peer namespace used by ordinary join.
    /// The selected OS BSS supplies the SSID/profile. Credential computation is
    /// external; this operation neither installs keys nor marks an association.
    /// Cancellation/timeout uses the ordinary remove + independent drain fence.
    /// With an AP MLD, `multi_link` names the setup link the exchange runs
    /// on under its own address.
    pub fn begin_external_authentication(
        &mut self,
        mut profile: Profile,
        ssid: Vec<u8>,
        multi_link: Option<Affiliation>,
        now: u64,
        budget: u64,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<Arc<Session>, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Idle || s.authentication_end.is_some() {
            return Err(Error::Busy);
        }
        if Some(profile.interface) != s.interface.registered_context()
            || profile.aid != 0
            || profile.cipher != 0
            || s.reserved_wcid.is_some_and(|wcid| wcid != profile.wcid)
        {
            return Err(Error::WrongProfile);
        }
        profile.validate().map_err(Error::Codec)?;
        let actual = s
            .controller
            .state()
            .policy
            .as_ref()
            .ok_or(Error::WrongProfile)?;
        if !actual.initial_channel_present(profile.channel.band, profile.channel.number as u8)
            || actual
                .capabilities()
                .observed()
                .phy()
                .and_then(|p| p.antenna_mask().ok())
                != Some(profile.antenna_mask)
        {
            return Err(Error::WrongProfile);
        }
        let generation = s.peer_generation.checked_add(1).ok_or(Error::Busy)?;
        let deadline = s
            .controller
            .state()
            .control
            .clock
            .now_us()
            .checked_add(budget)
            .ok_or(Error::Station(station::Error::Overflow))?;
        let link = multi_link
            .as_ref()
            .map(|affiliation| Self::setup_link_context(s, affiliation))
            .transpose()?;
        if let Some(context) = link {
            profile.interface = context;
        }
        // Every fallible body is prepared before Station or hardware mutation.
        let codec = Self::codec(s);
        let bss = match &multi_link {
            // Only the MT7925 advertises multi-link operation.
            Some(affiliation) => profile
                .link_bss_request(false, &Self::mld_link(affiliation, profile))
                .map(|body| vec![body]),
            None => codec.bss(profile, None, false),
        }
        .map_err(Error::Codec)?;
        let peer = codec
            .peer(profile, None, true, false)
            .map_err(Error::Codec)?;
        let broadcast = codec.broadcast(profile, None).map_err(Error::Codec)?;
        let session = s
            .station
            .begin_external_authentication(
                station::external::Peer {
                    bssid: profile.peer,
                    channel: profile.channel,
                    ssid,
                    multi_link,
                },
                now,
                budget,
                policy,
            )
            .map_err(Error::Station)?;
        s.authentication_deadline = Some(deadline);
        s.profile = Some(profile);
        s.ciphers = None;
        s.management_cipher = None;
        s.reserved_wcid = Some(profile.wcid);
        s.peer_generation = generation;
        s.format_generation = None;
        s.phase = Phase::Quarantined;
        if let Some(link) = link {
            Self::bind_setup_link(s, link)?;
        }
        s.controller
            .state_mut()
            .control
            .clear_peer_admission(profile.wcid)
            .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
        for body in &bss {
            Self::command(s, Command::BssInfo, body)?;
        }
        Self::command(s, Command::StationRecord, &peer)?;
        s.format_generation = Some(generation);
        Self::command(s, Command::StationRecord, &broadcast)?;
        s.phase = Phase::Joining;
        Ok(session)
    }

    /// Publish one exact OS-generated SAE transaction on the current
    /// ROC grant. Receipt proves queue ownership, never an air ACK or successful
    /// authentication. PacketOwner keeps this session and controller
    /// until descriptor consumption + TX-free + unmap all complete.
    pub fn transmit_external_authentication(
        &mut self,
        session: &Session,
        frame: station::external::Frame<'_>,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<Receipt, Error> {
        let s = self.state_mut();
        // Stale OS replies must fail before any radio/MMIO/publication work.
        s.station
            .validate_external_authentication(session)
            .map_err(Error::Station)?;
        if s.phase != Phase::Joining || s.six_ghz.is_some() {
            return Err(Error::WrongPhase);
        }
        if !policy.allow_management_tx(session.channel()) {
            return Err(Error::Station(station::Error::Regulatory));
        }
        let queue = s.controller.state().data_tx.as_ref().unwrap();
        if queue.outstanding_tokens() != 0 || queue.pending_descriptors() != 0 {
            return Err(Error::Busy);
        }
        let clock = s.controller.state().control.clock;
        if s.authentication_deadline
            .is_none_or(|deadline| clock.now_us() >= deadline)
        {
            return Err(Error::Station(station::Error::Timeout));
        }
        let began = clock.now_us();
        // Validate all OS wire input and the frontend deadline before ROC work.
        let before = s.station.external_deadline();
        let bytes = s
            .station
            .external_authentication_transmit(session, frame, now, policy)
            .map_err(Error::Station)?;
        Self::follow_external_deadline(s, before);
        Self::reserve_channel(s)?;
        let after = clock
            .now_us()
            .checked_sub(began)
            .and_then(|elapsed| now.checked_add(elapsed))
            .ok_or(Error::WrongProfile)?;
        if s.station
            .external_deadline()
            .is_none_or(|deadline| after >= deadline)
        {
            return Err(Error::Station(station::Error::Timeout));
        }
        // Recheck restrictive policy at the publication boundary.
        if !policy.allow_management_tx(session.channel()) {
            return Err(Error::Station(station::Error::Regulatory));
        }
        let receipt = Self::submit(s, &bytes, None, true, true, source)?;
        s.mlme_receipt = Some(receipt);
        Ok(receipt)
    }

    /// `ieee80211_auth` sending the OS's last SAE frame again once its wait
    /// passed, on a freshly reserved channel as `drv_mgd_prepare_tx` reserves
    /// one. A 6 GHz exchange retransmits under its own guards.
    pub(super) fn retransmit_external(
        s: &mut Owned<I, Q, C>,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        let queue = s.controller.state().data_tx.as_ref().unwrap();
        if s.six_ghz.is_some()
            || queue.outstanding_tokens() != 0
            || queue.pending_descriptors() != 0
        {
            return Ok(());
        }
        let Some(bytes) = s
            .station
            .external_retransmission(now, policy)
            .map_err(Error::Station)?
        else {
            return Ok(());
        };
        Self::reserve_channel(s)?;
        s.mlme_receipt = Some(Self::submit(s, &bytes, None, true, true, source)?);
        Ok(())
    }

    /// Move the native deadline as far as the station's moved: an
    /// authenticated exchange waits for association, and a retry status
    /// waits for the OS's next commit.
    pub(super) fn follow_external_deadline(s: &mut Owned<I, Q, C>, before: Option<u64>) {
        if let (Some(before), Some(after), Some(native)) = (
            before,
            s.station.external_deadline(),
            s.authentication_deadline.as_mut(),
        ) && after > before
        {
            *native = native.saturating_add(after - before);
        }
    }

    pub(super) fn authentication_ended(s: &mut Owned<I, Q, C>, failure: Option<station::Failure>) {
        if let Some(session) = s.station.external_authentication() {
            s.authentication_end = Some((session.clone(), failure));
        }
    }

    pub fn cancel_external_authentication(&mut self, session: &Session) -> Result<(), Error> {
        self.state()
            .station
            .validate_external_authentication(session)
            .map_err(Error::Station)?;
        self.disconnect()
    }
}

#[cfg(test)]
mod tests;
