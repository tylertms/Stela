// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
//! Explicit original native scan observation and held pre-association policy.
use super::*;
pub(super) const MAX_OBSERVATIONS: usize = 256;

use wifi::regulatory::{HardwarePolicy, OsPolicy, Owner, PmfAuthenticationGuard};

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// At most 256 native observations, each kept until it expires; a
    /// newer one of the same BSS replaces it.
    pub fn pmf_bss_observations(&self) -> impl Iterator<Item = &Arc<PmfBssObservation>> {
        self.state()
            .pmf_observations
            .iter()
            .map(|(_, observed)| observed)
    }
    /// Explicit PMF association using an observation and real OS-selected
    /// pre-association policy, before any peer/ROC/auth/assoc native publication.
    /// Canonical preparation is CPU-only; initial refusal withdraws only that
    /// exact never-submitted request. Partial native work stays quarantined.
    ///
    /// # Safety
    /// Both providers belong to this real kept controller/requester.
    /// Their held leases check the full pre-association native envelope and
    /// serialize revocation through peer/ROC/DMA/TX/deferred users.
    /// C must keep those policy/board/radio resources through partial
    /// programming, uncertain commands and every pending/deferred native user.
    /// The OS selects these exact RSNE/BSS bytes; no parser/scan result
    /// authenticates the peer or substitutes for that OS selection service.
    pub unsafe fn join_modern_pmf<H: HardwarePolicy, O: OsPolicy>(
        &mut self,
        authority: &mut Owner<H, O>,
        target: PmfJoin<'_>,
        profile: Profile,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        let PmfJoin { join, observed } = target;
        if join.request.pmf().is_none()
            || !self
                .state()
                .pmf_observations
                .iter()
                .any(|(_, old)| Arc::ptr_eq(old, observed))
        {
            return Err(Error::WrongProfile);
        }
        self.validate_modern_join(&join, profile)?;
        let pmf = join.request.pmf().ok_or(Error::WrongProfile)?;
        if observed.bss().kind != scan::result::Kind::Beacon
            || observed.bss().capabilities & 0x10 == 0
        {
            return Err(Error::WrongProfile);
        }
        pmf.validate_peer(&observed.bss().elements, &[])
            .map_err(|e| Error::Station(station::Error::Wire(e)))?;
        let observed = observed.clone();
        // SAFETY: same exclusive owner and kept provider leases.
        unsafe {
            authority.with_pmf_authentication(self, |client, guard| {
                let beacon = observed.bss.clone();
                client.state_mut().pmf_selected = Some(observed);
                client.join_inner_guarded(
                    JoinTarget {
                        join,
                        beacon: Some(beacon),
                        profile,
                    },
                    now,
                    source,
                    policy,
                    Some(guard),
                )
            })
        }
        .map_err(Error::Regulatory)?
    }
    /// Service the same pending auth/assoc Ticket under freshly acquired
    /// original policy/native/revision guards. Each retry and association TX
    /// requalifies its newly generated Ticket and exact canonical bytes.
    /// A revoked binding never silently inherits a later policy revision.
    ///
    /// # Safety
    /// The same requester/selection, retention and real revocation/drain
    /// contract as join_modern_pmf remains in force through every native user.
    pub unsafe fn poll_pmf_authentication<H: HardwarePolicy, O: OsPolicy>(
        &mut self,
        authority: &mut Owner<H, O>,
        now: u64,
        source: &dyn PageSource,
        policy: &mut (impl station::RegulatoryPolicy + scan::Policy),
    ) -> Result<Vec<Event>, Error> {
        if self.phase() != Phase::Joining || !self.station().requires_protected_management() {
            return Err(Error::WrongPhase);
        }
        // SAFETY: exclusive same native owner; guard borrows both leases.
        unsafe {
            authority.with_pmf_authentication(self, |client, guard| {
                Self::qualify_pmf_authentication(client.state_mut(), guard, None)?;
                let events = client.poll_inner(now, source, policy, Some(guard), None)?;
                let s = client.state_mut();
                if s.phase == Phase::Joining
                    && let Some(action) = s.station.poll(now, policy).map_err(Error::Station)?
                {
                    Self::action_guarded(s, action, now, source, policy, Some(guard))?;
                }
                Ok(events)
            })
        }
        .map_err(Error::Regulatory)?
    }
    /// `ieee80211_mgd_assoc` once the OS's SAE exchange authenticated the
    /// station: associate with the OS's SAE request through the exchange's
    /// native peer, claiming the keys a join claims, under freshly acquired
    /// PMF guards. A refused first request goes back to the exchange.
    ///
    /// # Safety
    /// As `join_modern_pmf`.
    pub unsafe fn associate_external_pmf<H: HardwarePolicy, O: OsPolicy>(
        &mut self,
        authority: &mut Owner<H, O>,
        session: &station::external::Session,
        target: PmfJoin<'_>,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        let PmfJoin { join, observed } = target;
        let s = self.state();
        s.station
            .validate_external_authentication(session)
            .map_err(Error::Station)?;
        if s.phase != Phase::Joining || s.six_ghz.is_some() {
            return Err(Error::WrongPhase);
        }
        let profile = s.profile.ok_or(Error::WrongProfile)?;
        if profile.peer != join.bssid
            || profile.channel != join.channel
            || !s
                .pmf_observations
                .iter()
                .any(|(_, old)| Arc::ptr_eq(old, observed))
        {
            return Err(Error::WrongProfile);
        }
        self.validate_modern_join(&join, profile)?;
        let pmf = join.request.pmf().ok_or(Error::WrongProfile)?;
        if observed.bss().kind != scan::result::Kind::Beacon
            || observed.bss().capabilities & 0x10 == 0
        {
            return Err(Error::WrongProfile);
        }
        pmf.validate_peer(&observed.bss().elements, &[])
            .map_err(|e| Error::Station(station::Error::Wire(e)))?;
        let observed = observed.clone();
        // SAFETY: same exclusive owner and kept provider leases.
        unsafe {
            authority.with_pmf_authentication(self, |client, guard| {
                let s = client.state_mut();
                let ciphers = join.request.key_ciphers();
                let management_cipher = join.request.pmf().map(|pmf| pmf.management());
                let action = s
                    .station
                    .associate_external_pmf(session, join, observed.clone(), now, policy)
                    .map_err(Error::Station)?;
                let Action::Transmit {
                    ticket, ref bytes, ..
                } = action
                else {
                    return Err(Error::WrongPhase);
                };
                s.pmf_selected = Some(observed);
                if let Err(error) =
                    Self::qualify_pmf_authentication(s, guard, Some((ticket, bytes)))
                {
                    s.station
                        .withdraw_unsubmitted_pmf(ticket)
                        .map_err(Error::Station)?;
                    s.pmf_selected = None;
                    s.pmf_authentication = None;
                    return Err(error);
                }
                s.ciphers = ciphers;
                s.management_cipher = management_cipher;
                Self::claim_keys(s, profile)?;
                let result = Self::action_guarded(s, action, now, source, policy, Some(guard));
                if result.is_err() {
                    s.phase = Phase::Quarantined;
                }
                result
            })
        }
        .map_err(Error::Regulatory)?
    }

    pub(super) fn qualify_pmf_authentication(
        s: &mut Owned<I, Q, C>,
        guard: &mut PmfAuthenticationGuard<'_>,
        transmit: Option<(Ticket, &[u8])>,
    ) -> Result<(), Error> {
        let observed = s.pmf_selected.as_ref().ok_or(Error::WrongProfile)?;
        let binding = guard
            .qualify(
                &s.station,
                observed,
                s.pmf_authentication.as_ref(),
                transmit,
            )
            .map_err(Error::Regulatory)?;
        s.pmf_authentication = Some(binding);
        Ok(())
    }
}
