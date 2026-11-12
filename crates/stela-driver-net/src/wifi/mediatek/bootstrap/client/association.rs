// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Joining a BSS: station setup, key slots and installation, and controlled-
//! port authorization.

use super::*;

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// Selected scan/beacon profile supplies timing/antenna/rate facts;
    /// no PHY capability, channel or OS credential is guessed. The legacy
    /// request/response and this profile must describe the same real BSS.
    pub fn join(
        &mut self,
        join: Join,
        profile: Profile,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        self.join_inner(join, None, profile, now, source, policy)
    }

    pub(super) fn join_inner(
        &mut self,
        join: Join,
        beacon: Option<scan::result::Bss>,
        profile: Profile,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        if join.request.pmf().is_some() {
            return Err(Error::ModernAdmissionRequired);
        }
        self.join_inner_guarded(
            JoinTarget {
                join,
                beacon,
                profile,
            },
            now,
            source,
            policy,
            None,
        )
    }

    pub(super) fn join_inner_guarded(
        &mut self,
        target: JoinTarget,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
        mut guard: Option<&mut wifi::regulatory::PmfAuthenticationGuard<'_>>,
    ) -> Result<(), Error> {
        let JoinTarget {
            join,
            beacon,
            profile,
        } = target;
        let s = self.state_mut();
        if s.phase != Phase::Idle || s.authentication_end.is_some() {
            return Err(Error::Busy);
        }
        if Some(profile.interface) != s.interface.registered_context()
            || profile.peer != join.bssid
            || profile.channel != join.channel
            || profile.aid != 0
            || profile.cipher != 0
        {
            return Err(Error::WrongProfile);
        }
        if beacon.is_none()
            && join
                .request
                .key_ciphers()
                .is_some_and(|pair| pair != (Cipher::Ccmp128, Cipher::Ccmp128))
        {
            return Err(Error::Unsupported);
        }
        profile.validate().map_err(Error::Codec)?;
        let source_policy = s
            .controller
            .state()
            .policy
            .as_ref()
            .ok_or(Error::WrongProfile)?;
        if !source_policy
            .initial_channel_present(profile.channel.band, profile.channel.number as u8)
            || source_policy
                .capabilities()
                .observed()
                .phy()
                .and_then(|p| p.antenna_mask().ok())
                != Some(profile.antenna_mask)
        {
            return Err(Error::WrongProfile);
        }
        let (local_rates, _) = rates(join.request.elements(), profile.channel.band)?;
        if profile.supported_rates & !local_rates != 0 {
            return Err(Error::WrongProfile);
        }
        let mut ies = join.request.elements();
        while !ies.is_empty() {
            let len = usize::from(ies[1]);
            if beacon.is_none() && ies[0] == 221 && ies[2..2 + len].starts_with(&[0, 0x50, 0xf2, 2])
            {
                return Err(Error::Unsupported);
            }
            ies = &ies[2 + len..];
        }

        if s.reserved_wcid.is_some_and(|wcid| wcid != profile.wcid) {
            return Err(Error::WrongProfile);
        }
        let generation = s.peer_generation.checked_add(1).ok_or(Error::Busy)?;
        // Prepare all source bodies before beginning physical mutation.
        let codec = Self::codec(s);
        let bss = codec.bss(profile, None, false).map_err(Error::Codec)?;
        let peer = codec
            .peer(profile, None, true, false)
            .map_err(Error::Codec)?;
        let broadcast = codec.broadcast(profile, None).map_err(Error::Codec)?;
        let ciphers = join.request.key_ciphers();
        let management_cipher = join.request.pmf().map(|pmf| pmf.management());
        let action = match beacon {
            Some(_) if join.request.pmf().is_some() => s.station.begin_modern_pmf(
                join,
                s.pmf_selected.as_ref().ok_or(Error::WrongProfile)?.clone(),
                now,
                policy,
            ),
            Some(beacon) => s.station.begin_modern(join, beacon, now, policy),
            None => s.station.begin(join, now, policy),
        }
        .map_err(Error::Station)?;
        if s.station.requires_protected_management() {
            let Action::Transmit {
                ticket, ref bytes, ..
            } = action
            else {
                return Err(Error::WrongPhase);
            };
            let outcome = match guard.as_deref_mut() {
                Some(guard) => Self::qualify_pmf_authentication(s, guard, Some((ticket, bytes))),
                None => Err(Error::ModernAdmissionRequired),
            };
            if let Err(error) = outcome {
                // No native mutation has happened: withdraw only this
                // unsubmitted canonical generation, keeping epochs monotonic.
                s.station
                    .withdraw_unsubmitted_pmf(ticket)
                    .map_err(Error::Station)?;
                s.pmf_selected = None;
                s.pmf_authentication = None;
                return Err(error);
            }
        }
        // source peer-add clears admission counters before BSS(false), then
        // plain STA_REC(NONE). Neither creates an authorized OS port.
        s.profile = Some(profile);
        s.ciphers = ciphers;
        s.management_cipher = management_cipher;
        s.reserved_wcid = Some(profile.wcid);
        s.peer_generation = generation;
        s.format_generation = None;
        s.phase = Phase::Quarantined;
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
        Self::claim_keys(s, profile)?;
        s.phase = Phase::Joining;
        let result = Self::action_guarded(s, action, now, source, policy, guard);
        if result.is_err() && s.station.requires_protected_management() {
            s.phase = Phase::Quarantined;
        }
        result
    }

    /// Claim, or rebind from the last association, the PTK and GTK slots of
    /// the profile's peer.
    pub(super) fn claim_keys(s: &mut Owned<I, Q, C>, profile: Profile) -> Result<(), Error> {
        let key_context = KeyContext {
            bss_index: profile.interface.bss_index,
            omac_index: profile.interface.omac_index,
            station_disabled: false,
        };
        // An AP MLD's PTK is the MLDs'.
        let peer = s.station.pairwise_peer().ok_or(Error::WrongPhase)?;
        let m = s.controller.state().mailbox.as_ref().unwrap();
        let resource = s.controller.state().resource.clone();
        for (owner, slot, scope) in [
            (&mut s.ptk, profile.wcid, KeyScope::Pairwise(peer)),
            (
                &mut s.gtk,
                profile.interface.broadcast_wcid,
                KeyScope::Group(profile.peer),
            ),
        ] {
            if let Some(key) = owner {
                // SAFETY: the key is idle between associations and keeps its slot.
                unsafe { key.rebind(key_context, scope) }.map_err(Error::Key)?;
            } else {
                *owner = Some(
                    // SAFETY: the slot is this association's WCID on the client's own mailbox.
                    unsafe { HardwareKey::claim(m, slot, key_context, scope, resource.clone()) }
                        .map_err(Error::Key)?,
                );
            }
        }
        Ok(())
    }

    pub fn poll_station(
        &mut self,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        if self.station().requires_protected_management() {
            return Err(Error::ModernAdmissionRequired);
        }
        let s = self.state_mut();
        if !matches!(s.phase, Phase::Joining | Phase::Associated) {
            return Err(Error::WrongPhase);
        }
        let now = if s.station.phase() == station::Phase::ExternalAuthenticating
            && s.authentication_deadline
                .is_some_and(|deadline| s.controller.state().control.clock.now_us() >= deadline)
        {
            now.max(s.station.external_deadline().ok_or(Error::WrongPhase)?)
        } else {
            now
        };
        if let Some(a) = s.station.poll(now, policy).map_err(Error::Station)? {
            Self::action(s, a, now, source, policy)?;
        }
        if s.phase == Phase::Joining && s.station.phase() == station::Phase::ExternalAuthenticating
        {
            Self::retransmit_external(s, now, source, policy)?;
        }
        Ok(())
    }

    /// Source BSS cipher update precedes first key programming. Frontend
    /// supplies authenticated key/RSC after its real EAPOL exchange; this driver
    /// does not substitute an OS supplicant or treat EAPOL classification as MIC.
    pub fn install_key(
        &mut self,
        pairwise: bool,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        if s.modern.is_some() {
            return Err(Error::ModernAdmissionRequired);
        }
        let mut p = s.profile.ok_or(Error::WrongProfile)?;
        if p.cipher == 0 {
            p.cipher = 4;
            for body in Self::bss_request(s, p, true)? {
                Self::command(s, Command::BssInfo, &body)?;
            }
            s.profile = Some(p);
        }
        let key = if pairwise {
            s.ptk.as_mut()
        } else {
            s.gtk.as_mut()
        }
        .ok_or(Error::NoKey)?;
        s.phase = Phase::Quarantined;
        let mailbox = s.controller.state_mut().mailbox.as_mut().unwrap();
        let outcome = key.install(mailbox, index, Cipher::Ccmp128, material, seed);
        if !key.is_quarantined() {
            s.phase = Phase::Associated;
        }
        outcome.map_err(Error::Key)
    }

    /// Explicit real OS controlled-port activation against both installed
    /// original slots. No firmware PORT_SECURE wire value grants this permission.
    pub fn authorize_port(&mut self, epoch: u64) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        s.station
            .authorize_port(epoch, &Self::link_keys(&s.ptk, &s.gtk, &s.links)?)
            .map_err(Error::Station)
    }
}
