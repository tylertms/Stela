// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2023 MediaTek Inc.
//! Current-association modern command ownership. A prepared context has no
//! permission to enable firmware rate selection or autonomous radio responses.
use super::*;
use crate::wifi::station::AssociationContext;

pub(super) struct Configuration {
    context: Arc<AssociationContext>,
    pub(super) wire: wire::modern::Configuration,
    commands: Vec<wire::Outgoing>,
    filter: crate::wifi::mediatek::mcu::init::Request,
    completed: usize,
    pub(super) prepared_edca: [station::phy::AccessCategory; 4],
    pub(super) applied_edca: Option<super::edca::AppliedEdca>,
}
impl Configuration {
    pub(super) fn context(&self) -> &Arc<AssociationContext> {
        &self.context
    }
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// Program the original canonical key owner and source BSS cipher under
    /// the same complete modern hardware/OS permission. Material/RSC come from
    /// the OS authentication service; this is no supplicant substitute.
    ///
    /// # Safety
    /// The exact-controller, policy retention and revocation requirements of
    /// `configure_modern` apply to both BSS and key command publication.
    pub unsafe fn install_key_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        pairwise: bool,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        // SAFETY: covered by this function's own safety contract.
        unsafe { self.modern_key(authority, admission, pairwise, index, material, seed) }
    }

    unsafe fn modern_key<H: wifi::regulatory::HardwarePolicy, O: wifi::regulatory::OsPolicy>(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        pairwise: bool,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        s.station
            .validate_association_context(&modern.context)
            .map_err(Error::Station)?;
        let mut profile = s.profile.ok_or(Error::WrongProfile)?;
        let ciphers = s.ciphers.ok_or(Error::NoKey)?;
        let cipher = if pairwise { ciphers.0 } else { ciphers.1 };
        let bss = if profile.cipher == 0 {
            profile.cipher = cipher_code(cipher);
            Some(
                Self::codec(s)
                    .bss(profile, Some(&modern.wire), true)
                    .map_err(Error::Codec)?,
            )
        } else {
            None
        };
        let key = if pairwise {
            s.ptk.as_mut()
        } else {
            s.gtk.as_mut()
        }
        .ok_or(Error::NoKey)?;
        let phase = &mut s.phase;
        let current_profile = &mut s.profile;
        let controller = &mut s.controller;
        let replies = &mut s.replies;
        // mt7925_set_key: the MLDs' PTK goes to every link's AP WCID.
        let links: &mut [multi_link::Link<C>] = if pairwise { &mut s.links } else { &mut [] };
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            authority.publish_association(
                admission,
                &s.station,
                wifi::regulatory::AssociationUse::Management,
                |_, _| {
                    *phase = Phase::Quarantined;
                    if let Some(bodies) = &bss {
                        for body in bodies {
                            Self::command_parts(controller, replies, Command::BssInfo, body)?;
                        }
                        *current_profile = Some(profile);
                    }
                    let mailbox = controller.state_mut().mailbox.as_mut().unwrap();
                    let result = key.install(mailbox, index, cipher, material, seed);
                    if !key.is_quarantined() {
                        *phase = Phase::Associated;
                    }
                    let identity = result.map_err(Error::Key)?;
                    for link in links.iter_mut() {
                        *phase = Phase::Quarantined;
                        let key = multi_link::LinkKey {
                            pairwise: true,
                            cipher,
                            index,
                            material,
                            seed,
                        };
                        Self::install_link_key(controller, replies, link, &key)?;
                        *phase = Phase::Associated;
                    }
                    Ok(identity)
                },
            )
        }
        .map_err(Error::Regulatory)?
    }

    /// Install the GTK of the AP of the association's link `link_id`, its
    /// MLO GTK KDE's, under the same permission as `install_key_modern`.
    ///
    /// # Safety
    /// As `install_key_modern`.
    pub unsafe fn install_link_key_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        link_id: u8,
        index: u8,
        material: &[u8],
        seed: ReplaySeed,
    ) -> Result<KeyIdentity, Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        s.station
            .validate_association_context(&modern.context)
            .map_err(Error::Station)?;
        let cipher = s.ciphers.ok_or(Error::NoKey)?.1;
        let link = s
            .links
            .iter_mut()
            .find(|link| link.link_id == link_id)
            .ok_or(Error::WrongProfile)?;
        let phase = &mut s.phase;
        let controller = &mut s.controller;
        let replies = &mut s.replies;
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            authority.publish_association(
                admission,
                &s.station,
                wifi::regulatory::AssociationUse::Management,
                |_, _| {
                    *phase = Phase::Quarantined;
                    let key = multi_link::LinkKey {
                        pairwise: false,
                        cipher,
                        index,
                        material,
                        seed,
                    };
                    let identity = Self::install_link_key(controller, replies, link, &key)?;
                    *phase = Phase::Associated;
                    Ok(identity)
                },
            )
        }
        .map_err(Error::Regulatory)?
    }

    /// Begin source-backed single-link modern negotiation. Modern
    /// configuration remains pending until context-bound hardware/OS
    /// admission covers its complete firmware/autonomous transmission set.
    pub fn join_modern(
        &mut self,
        join: Join,
        beacon: scan::result::Bss,
        profile: Profile,
        now: u64,
        source: &dyn PageSource,
        policy: &mut impl station::RegulatoryPolicy,
    ) -> Result<(), Error> {
        self.validate_modern_join(&join, profile)?;
        self.join_inner(join, Some(beacon), profile, now, source, policy)
    }

    pub(super) fn validate_modern_join(&self, join: &Join, profile: Profile) -> Result<(), Error> {
        let requested = join.request.modern().ok_or(Error::WrongProfile)?;
        let actual = self
            .local_advertisement(join.channel.band)
            .map_err(|_| Error::WrongProfile)?;
        actual
            .validate_capabilities(requested.capabilities())
            .map_err(|_| Error::WrongProfile)?;
        if profile.antenna_mask != actual.antenna_mask()
            || matches!(
                requested.width(),
                station::phy::Width::Mhz80P80 | station::phy::Width::Mhz320
            )
        {
            return Err(Error::WrongProfile);
        }
        Ok(())
    }

    /// Private shared context kept by this native owner. Inspection
    /// is neither modern publication permission nor a firmware completion.
    pub fn modern_context(&self) -> Option<&Arc<AssociationContext>> {
        self.state().modern.as_ref().map(|m| &m.context)
    }

    pub fn pending_modern_requests(&self) -> usize {
        if self.phase() == Phase::AwaitingModernAdmission {
            self.state().modern.as_ref().map_or(0, |m| m.commands.len())
        } else {
            0
        }
    }
    /// Derive the complete restricted command before replacing the pending
    /// command. Allocation/validation failure leaves the old owner intact;
    /// this helper performs no native publication or association completion.
    pub(super) fn restrict_prepared_edca(
        &mut self,
        effective: &[station::phy::AccessCategory; 4],
    ) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::AwaitingModernAdmission {
            return Err(Error::WrongPhase);
        }
        let m = s.modern.as_mut().ok_or(Error::WrongPhase)?;
        if m.completed != 0 {
            return Err(Error::WrongPhase);
        }
        let p = s.profile.ok_or(Error::WrongProfile)?;
        // The restriction is checked against the negotiated set first.
        m.wire
            .edca_restricted_request(p, effective)
            .map_err(Error::Codec)?;
        let n = m.wire.negotiated();
        let outgoing = wire::Codec::of(s.controller.state().identity.chip)
            .edca(p, &n.edca(), effective, n.wmm.is_some())
            .map_err(Error::Codec)?;
        let pending = m
            .commands
            .iter_mut()
            .find(|outgoing| outgoing.is_edca())
            .ok_or(Error::WrongPhase)?;
        *pending = outgoing;
        m.prepared_edca = *effective;
        Ok(())
    }
    pub fn completed_modern_requests(&self) -> usize {
        self.state().modern.as_ref().map_or(0, |m| m.completed)
    }

    /// Execute this same private current association's complete source command
    /// chain under the canonical hardware/OS guards, then acknowledge it to
    /// Station only after real matching firmware and DMA completions.
    ///
    /// # Safety
    /// `authority` belongs to this exact controller/requester and C,
    /// with current reset/firmware/policy epochs and applied
    /// per-rate power. Its acquired guards serialize this native path; C keeps
    /// their underlying policy/board/radio/queue/key resources alive through
    /// partial/uncertain programming and all armed autonomous/deferred users.
    /// Revocation must enforce/stop/cancel/drain those real users, not
    /// change an admission boolean. A different device's valid permission or a
    /// copied source plan cannot check this native attachment.
    pub unsafe fn configure_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        now: u64,
    ) -> Result<(), Error> {
        use wifi::regulatory::AssociationUse;
        let s = self.state_mut();
        if s.phase != Phase::AwaitingModernAdmission {
            return Err(Error::WrongPhase);
        }
        let p = s.profile.ok_or(Error::WrongProfile)?;
        let m = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        s.station
            .validate_association_context(&m.context)
            .map_err(Error::Station)?;
        if m.completed != 0 {
            return Err(Error::WrongPhase);
        }
        s.replies
            .try_reserve(m.commands.len())
            .map_err(|_| Error::NoMemory)?;
        s.transport_facts
            .try_reserve(1 + m.commands.len())
            .map_err(|_| Error::NoMemory)?;
        let clock = s.controller.state().control.clock;
        let began = clock.now_us();
        s.phase = Phase::Quarantined;
        let mut started = false;
        let outcome = {
            // Keep Station immutably borrowed with the canonical guards while
            // mutating only its disjoint native controller and raw journals.
            let station = &s.station;
            let mut roc = radio::Roc {
                controller: &mut s.controller,
                transport_facts: &mut s.transport_facts,
                asynchronous: &mut s.asynchronous,
                phase: &mut s.phase,
                roc_token: &mut s.roc_token,
                roc_deadline: &mut s.roc_deadline,
                profile: &mut s.profile,
            };
            let replies = &mut s.replies;
            let format = &mut s.format_generation;
            let links = &mut s.links;
            let m = s.modern.as_mut().unwrap();
            // SAFETY: covered by this function's own safety contract.
            unsafe {
                authority.publish_association(
                    admission,
                    station,
                    AssociationUse::Configure,
                    |_, _| {
                        started = true;
                        *format = None;
                        for (index, outgoing) in m.commands.iter().enumerate() {
                            if index == 1 {
                                roc.controller
                                    .state_mut()
                                    .control
                                    .clear_peer_admission(p.wcid)
                                    .map_err(|e| {
                                        Error::Bootstrap(super::super::Error::Control(e))
                                    })?;
                            }
                            // Source filter transport precedes the final awake update;
                            // preserve its one-way/raw fact rather than inventing ACK.
                            if index + 1 == m.commands.len() {
                                let fact = roc
                                    .controller
                                    .state_mut()
                                    .mailbox
                                    .as_mut()
                                    .unwrap()
                                    .initial(&m.filter)
                                    .map_err(Error::Transport)?;
                                roc.transport_facts.push(fact);
                            }
                            Self::deliver(roc.controller, replies, roc.transport_facts, outgoing)?;
                            m.completed += 1;
                        }
                        for link in links.iter_mut() {
                            Self::activate_link(&mut roc, replies, p, link)?;
                        }
                        Ok::<(), Error>(())
                    },
                )
            }
        };
        match outcome {
            Err(error) => {
                if !started {
                    s.phase = Phase::AwaitingModernAdmission;
                }
                return Err(Error::Regulatory(error));
            }
            Ok(result) => result?,
        }
        // The plain source STA_REC command/ACK, not WMWA or an opaque generic
        // response, establishes this peer's raw-header generation.
        s.format_generation = Some(s.peer_generation);
        Self::abort_channel(s)?;
        let ticket = s.modern.as_ref().unwrap().context.ticket();
        let completed_now = clock
            .now_us()
            .checked_sub(began)
            .and_then(|elapsed| now.checked_add(elapsed))
            .ok_or(Error::WrongProfile)?;
        s.station
            .complete_configuration(ticket, Completion::Accepted, completed_now)
            .map_err(Error::Station)?;
        // NIC/local issuer and source link negotiation bound the receive
        // window; no session exists until an ADDBA exchange and real BA ACK.
        let n = s.modern.as_ref().unwrap().context.negotiated();
        let window = if n.eht.is_some() {
            1024
        } else if n.he.is_some() {
            256
        } else {
            64
        };
        if n.aggregates() {
            s.station
                .configure_rx_block_ack(window, true)
                .map_err(Error::BlockAck)?;
            s.station
                .configure_tx_block_ack(window)
                .map_err(Error::BlockAck)?;
        }
        let m = s.modern.as_mut().unwrap();
        m.applied_edca = Some(super::edca::AppliedEdca {
            parameter_set_count: m
                .context
                .negotiated()
                .wmm
                .as_ref()
                .map_or(0, |wmm| wmm.parameter_set_count),
            access_categories: m.prepared_edca,
        });
        s.phase = Phase::Associated;
        Ok(())
    }

    pub(super) fn bss_request(
        s: &Owned<I, Q, C>,
        p: Profile,
        enable: bool,
    ) -> Result<Vec<Vec<u8>>, Error> {
        Self::codec(s)
            .bss(p, s.modern.as_ref().map(|m| &m.wire), enable)
            .map_err(Error::Codec)
    }

    pub(super) fn prepare_modern(
        s: &mut Owned<I, Q, C>,
        ticket: Ticket,
        context: Arc<AssociationContext>,
        links: Vec<Arc<AssociationContext>>,
    ) -> Result<(), Error> {
        s.station
            .validate_association_context(&context)
            .map_err(Error::Station)?;
        let p = s.profile.ok_or(Error::WrongProfile)?;
        if context.ticket() != ticket
            || context.bssid() != p.peer
            || context.channel() != p.channel
            || s.modern.is_some()
        {
            return Err(Error::WrongProfile);
        }
        // Keep Station's same private context even if subsequent preparation
        // fails. Existing programmed peer/controller remain owned/quarantined.
        s.phase = Phase::Quarantined;
        let actual = local::current(s, p.channel.band).map_err(|_| Error::WrongProfile)?;
        actual
            .validate_capabilities(context.negotiated().local())
            .map_err(|_| Error::WrongProfile)?;
        let p = associated(p, &context);
        let mut wire = wire::modern::Configuration::new(
            p,
            context.negotiated().clone(),
            operation(&context, p.channel.band)?,
            *actual.he_source_maps(),
        )
        .map_err(Error::Codec)?;
        // The setup link runs where its authentication bound the interface.
        if let Some(affiliation) = context.affiliation() {
            if p.interface.address != affiliation.local {
                return Err(Error::WrongProfile);
            }
            wire = wire.with_link(Self::mld_link(affiliation, p));
        }
        let codec = Self::codec(s);
        let bss = codec.bss(p, Some(&wire), true).map_err(Error::Codec)?;
        let mut commands = Vec::new();
        commands
            .try_reserve_exact(6 + bss.len())
            .map_err(|_| Error::NoMemory)?;
        commands.extend(
            bss.into_iter()
                .map(|body| wire::Outgoing::Uni(Command::BssInfo, body)),
        );
        commands.push(wire::Outgoing::Uni(
            Command::StationRecord,
            codec
                .peer(p, Some(&wire), true, true)
                .map_err(Error::Codec)?,
        ));
        commands.push(wire::Outgoing::Uni(
            Command::StationRecord,
            codec.broadcast(p, Some(&wire)).map_err(Error::Codec)?,
        ));
        // edca_request checks the negotiated set against the profile.
        wire.edca_request(p).map_err(Error::Codec)?;
        let n = context.negotiated();
        commands.push(
            codec
                .edca(p, &n.edca(), &n.edca(), n.wmm.is_some())
                .map_err(Error::Codec)?,
        );
        if n.eht.is_some() {
            commands.push(wire::Outgoing::Uni(
                Command::BssInfo,
                wire.puncture_request(p).map_err(Error::Codec)?,
            ));
        }
        commands.push(codec.beacon_filter(p).map_err(Error::Codec)?);
        commands.push(wire::Outgoing::Uni(
            Command::BssInfo,
            p.awake().map_err(Error::Codec)?,
        ));
        Self::prepare_links(s, links, p, &wire)?;
        s.profile = Some(p);
        let filter = codec.filter(p, true).map_err(Error::Codec)?;
        let prepared_edca = context.negotiated().edca();
        s.modern = Some(Configuration {
            context,
            wire,
            commands,
            filter,
            completed: 0,
            prepared_edca,
            applied_edca: None,
        });
        // Preparation never acknowledges or publishes modern BSS/peer effects.
        // configure_modern consumes the canonical context-bound admission.
        s.phase = Phase::AwaitingModernAdmission;
        Ok(())
    }
}

/// The firmware cipher code of `cipher`.
pub(super) fn cipher_code(cipher: Cipher) -> u8 {
    match cipher {
        Cipher::Wep40 => 1,
        Cipher::Tkip => 2,
        Cipher::Ccmp128 => 4,
        Cipher::Ccmp256 => 10,
        Cipher::Gcmp128 => 11,
        Cipher::Gcmp256 => 12,
        Cipher::Wep104 => 5,
    }
}

/// `p` once `context` associated it: the AID, rates, beacon timing and slot
/// time the association gives the BSS.
pub(super) fn associated(mut p: Profile, context: &AssociationContext) -> Profile {
    p.aid = context.response().aid;
    p.basic_rates = context.negotiated().legacy.basic;
    p.supported_rates = context.negotiated().legacy.supported;
    p.beacon_interval = context.beacon().beacon_interval;
    p.dtim_period = context.dtim_period();
    // Source use_short_slot is always true outside2GHz and follows the
    // AP capability on2GHz; no caller timing guess enters Configure.
    p.slot_time =
        if p.channel.band != wifi::Band::Ghz2 || context.response().capabilities & (1 << 10) != 0 {
            9
        } else {
            20
        };
    p
}

/// The BSS operation `context` runs on `band`, as the firmware takes it.
pub(super) fn operation(
    context: &AssociationContext,
    band: wifi::Band,
) -> Result<wire::modern::Operation, Error> {
    let definition = context.definition();
    let (mbssid_indicator, mbssid_index) = context
        .beacon()
        .profile
        .map_or((0, 0), |profile| (profile.indicator, profile.index));
    Ok(wire::modern::Operation {
        center_channel: center_channel(band, definition.center1_mhz)?,
        second_center_channel: definition
            .center2_mhz
            .map(|mhz| center_channel(band, mhz))
            .transpose()?
            .unwrap_or(0),
        punctured: definition.punctured,
        he: context
            .operations()
            .he
            .as_ref()
            .map(|h| wire::modern::HeOperation {
                packet_extension: h.packet_extension(),
                rts_threshold: h.rts_threshold(),
                color_enabled: h.color_enabled(),
                color: h.color(),
            }),
        mbssid_indicator,
        mbssid_index,
    })
}

pub(super) fn center_channel(band: wifi::Band, mhz: u32) -> Result<u8, Error> {
    if band == wifi::Band::Ghz2 && mhz == 2484 {
        return Ok(14);
    }
    let base = match band {
        wifi::Band::Ghz2 => 2407,
        wifi::Band::Ghz5 => 5000,
        wifi::Band::Ghz6 => 5950,
    };
    let offset = mhz
        .checked_sub(base)
        .filter(|n| n % 5 == 0)
        .ok_or(Error::WrongProfile)?;
    u8::try_from(offset / 5)
        .ok()
        .filter(|n| *n != 0)
        .ok_or(Error::WrongProfile)
}
#[cfg(test)]
pub(super) mod tests;
