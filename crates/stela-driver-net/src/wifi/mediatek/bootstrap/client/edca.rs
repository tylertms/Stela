// SPDX-License-Identifier: BSD-3-Clause-Clear AND GPL-2.0-only
// Copyright (C) 2020-2023 MediaTek Inc.
// Count comparison and queue updates follow pinned Linux mac80211/mlme.c.
//! Associated WMM observations from this owner's completed native RX, followed
//! by an independently guarded real EDCA command. Negotiated identity is immutable.
use super::*;
use station::phy::{AccessCategory, Role, Wmm};

const OBSERVATION_LIFETIME_US: u64 = 30_000_000;

/// Last complete EDCA ACK plus descriptor/mapping retirement. This is
/// local queue configuration, not new association or authentication evidence.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AppliedEdca {
    pub parameter_set_count: u8,
    pub access_categories: [AccessCategory; 4],
}

pub(super) struct Observed<C: 'static> {
    resource: Arc<C>,
    context: Arc<station::AssociationContext>,
    profile: Profile,
    association: u64,
    peer_generation: u64,
    ptk: Option<KeyIdentity>,
    gtk: Option<KeyIdentity>,
    seen: u64,
    count: u8,
    parameters: [AccessCategory; 4],
    // Keep the normalized completed DMA observation, never caller bytes.
    _frame: Mpdu,
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub fn applied_edca(&self) -> Option<&AppliedEdca> {
        self.state().modern.as_ref()?.applied_edca.as_ref()
    }
    pub fn pending_edca(&self) -> bool {
        Self::pending_edca_observation(self.state()).is_some()
    }

    fn pending_edca_observation(s: &Owned<I, Q, C>) -> Option<&Observed<C>> {
        s.edca
            .as_ref()
            .or_else(|| s.links.iter().find_map(|link| link.edca.as_ref()))
    }

    pub(super) fn pending_edca_channel(&self) -> Option<wifi::Channel> {
        Self::pending_edca_observation(self.state()).map(|observed| observed.profile.channel)
    }

    pub(super) fn observe_edca(s: &mut Owned<I, Q, C>, frame: Mpdu) -> Result<(), Error> {
        let p = s.profile.ok_or(Error::WrongProfile)?;
        let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        s.station
            .validate_association_context(modern.context())
            .map_err(Error::Station)?;
        let h = frame.header();
        let subtype = h.control & 0x00fc;
        let channel = (
            frame.radio.band,
            frame.radio.channel,
            frame.radio.frequency_mhz,
        );
        // A frame received before a channel switch is no longer the AP's.
        if Self::switched_from(s, channel) {
            return Ok(());
        }
        let joined = modern.context().clone();
        let sent_by = |address: crate::Mac| {
            h.transmitter == Some(address)
                && frame.bytes().get(16..22) == Some(address.0.as_slice())
        };
        if !(sent_by(p.peer) || sent_by(joined.beacon().transmitter))
            || (
                frame.radio.band,
                frame.radio.channel,
                frame.radio.frequency_mhz,
            ) != (p.channel.band, p.channel.number, p.channel.frequency_mhz)
            || modern.context().bssid() != p.peer
            || s.station.operating_definition().map(|d| d.control) != Some(p.channel)
        {
            return Err(Error::WrongProfile);
        }
        // Contradictory current-peer evidence cannot leave an older pending
        // relaxation eligible. This drops only an unsubmitted CPU observation.
        s.edca = None;
        if s.phase != Phase::Associated
            || frame.fcs_error
            || frame.monitor_only
            || h.protected()
            || h.fragmented()
            || h.control & 0x0300 != 0
            || h.length != 24
            || frame.aggregation != wifi::Aggregation::default()
            || !matches!(subtype, 0x0080 | 0x0050)
            || (subtype == 0x0080 && h.receiver != crate::Mac::BROADCAST)
            || (subtype == 0x0050 && h.receiver != p.interface.address)
        {
            return Err(Error::WrongProfile);
        }
        let sent = frame.bytes().get(36..).ok_or(Error::WrongProfile)?;
        // A nontransmitted BSS reads its share of its transmitter's frames,
        // and an EMA AP's without that share say what they all inherit.
        let own = scan::multiple_bssid::heard(joined.beacon(), sent);
        let capabilities = match &own {
            Some(own) => Some(own.capabilities),
            None if joined.beacon().profile.is_some() => None,
            None => Some(u16::from_le_bytes([frame.bytes()[34], frame.bytes()[35]])),
        };
        let elements = own.as_ref().map_or(sent, |own| own.elements.as_slice());
        // mac80211 processes a beacon's channel switch elements first.
        if subtype == 0x0080 {
            Self::process_channel_switch(s, elements, csa::Source::Beacon, None);
            Self::follow_links(s, &joined, elements);
        }
        let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
        // mac80211 `ieee80211_handle_bss_capability`: on 2.4 GHz the slot
        // follows the AP's Short Slot Time capability; elsewhere it is 9 us.
        if let (wifi::Band::Ghz2, Some(capabilities)) = (p.channel.band, capabilities) {
            let slot = if capabilities & (1 << 10) != 0 { 9 } else { 20 };
            s.slot_time = (slot != p.slot_time).then_some(slot);
        }
        if let (Some(signal), Some(association)) =
            (frame.radio.signal_dbm, s.station.association_epoch())
        {
            s.cqm.beacon(association, i32::from(signal));
        }
        // mac80211 `ieee80211_config_bw`: the AP's operation narrowed to the
        // association, applied when it differs from the current channel.
        let context = modern.context().clone();
        let operating = s.station.operating_definition().ok_or(Error::WrongPhase)?;
        let current = s.channel.unwrap_or(operating);
        s.channel_change = match context.beacon_definition(elements, operating.control) {
            Ok(definition) if definition == current => None,
            Ok(definition) if definition.within(&operating) => Some(Some(definition)),
            _ => Some(None),
        };
        let Some((count, parameters)) = parameters(
            elements,
            modern.context().negotiated(),
            modern.applied_edca.as_ref().ok_or(Error::WrongPhase)?,
        )?
        else {
            return Ok(());
        };
        s.edca = Some(Observed {
            resource: s.controller.state().resource.clone(),
            context: modern.context().clone(),
            profile: p,
            association: s.station.association_epoch().ok_or(Error::WrongPhase)?,
            peer_generation: s.peer_generation,
            ptk: s.ptk.as_ref().and_then(|key| key.slot().installed()),
            gtk: s.gtk.as_ref().and_then(|key| key.slot().installed()),
            seen: s.controller.state().control.clock.now_us(),
            count,
            parameters,
            _frame: frame,
        });
        Ok(())
    }

    pub(super) fn observe_link_beacon(s: &mut Owned<I, Q, C>, frame: Mpdu) -> Result<(), Error> {
        let link = s
            .links
            .iter_mut()
            .find(|link| {
                (link.profile.channel.band, link.profile.channel.number)
                    == (frame.radio.band, frame.radio.channel)
            })
            .ok_or(Error::WrongProfile)?;
        link.edca = None;
        let h = frame.header();
        let p = link.profile;
        let subtype = h.control & 0x00fc;
        if frame.radio.frequency_mhz != p.channel.frequency_mhz
            || h.transmitter != Some(p.peer)
            || frame.bytes().get(16..22) != Some(p.peer.0.as_slice())
            || frame.fcs_error
            || frame.monitor_only
            || h.protected()
            || h.fragmented()
            || h.control & 0x0300 != 0
            || h.length != 24
            || frame.aggregation != wifi::Aggregation::default()
            || !matches!(subtype, 0x0080 | 0x0050)
            || (subtype == 0x0080 && h.receiver != crate::Mac::BROADCAST)
            || (subtype == 0x0050 && h.receiver != p.interface.address)
        {
            return Err(Error::WrongProfile);
        }
        if !s
            .station
            .link_contexts()
            .iter()
            .any(|context| Arc::ptr_eq(context, &link.context))
        {
            return Err(Error::Station(station::Error::Stale));
        }
        let elements = frame.bytes().get(36..).ok_or(Error::WrongProfile)?;
        let operating = link.context.definition();
        link.channel_change = match link.context.beacon_definition(elements, operating.control) {
            Ok(definition) if definition == link.channel => None,
            Ok(definition) if definition.within(&operating) => Some(Some(definition)),
            _ => Some(None),
        };
        let Some((count, parameters)) = parameters(
            elements,
            link.context.negotiated(),
            link.applied_edca.as_ref().ok_or(Error::WrongPhase)?,
        )?
        else {
            return Ok(());
        };
        link.edca = Some(Observed {
            resource: s.controller.state().resource.clone(),
            context: link.context.clone(),
            profile: p,
            association: s.station.association_epoch().ok_or(Error::WrongPhase)?,
            peer_generation: s.peer_generation,
            ptk: link.ptk.slot().installed(),
            gtk: link.gtk.slot().installed(),
            seen: s.controller.state().control.clock.now_us(),
            count,
            parameters,
            _frame: frame,
        });
        Ok(())
    }

    /// Whether a beacon changed the channel: `Some(false)` when the change
    /// leaves the associated channel.
    pub fn pending_channel(&self) -> Option<bool> {
        let s = self.state();
        s.channel_change
            .or_else(|| s.links.iter().find_map(|link| link.channel_change))
            .map(|change| change.is_some())
    }

    /// mac80211 `ieee80211_link_change_chanreq` to `mt7925_change_chanctx`:
    /// sends the channel a beacon changed through `mt7925_mcu_set_chctx`.
    ///
    /// # Safety
    /// As `update_slot_time_modern`.
    pub unsafe fn update_channel_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        Self::command_ready(s)?;
        let codec = Self::codec(s);
        let (pending, current, profile) = if s.channel_change.is_some() {
            (
                &mut s.channel_change,
                None,
                s.profile.ok_or(Error::WrongPhase)?,
            )
        } else {
            let link = s
                .links
                .iter_mut()
                .find(|link| link.channel_change.is_some())
                .ok_or(Error::WrongPhase)?;
            (
                &mut link.channel_change,
                Some(&mut link.channel),
                link.profile,
            )
        };
        let Some(Some(definition)) = *pending else {
            return Err(Error::WrongPhase);
        };
        let body = codec.channel(profile, &definition).map_err(Error::Codec)?;
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
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
                    Self::command_parts(controller, replies, Command::BssInfo, &body)?;
                    *phase = Phase::Associated;
                    Ok(())
                },
            )
        }
        .map_err(Error::Regulatory)??;
        *pending = None;
        match current {
            Some(channel) => *channel = definition,
            None => s.channel = Some(definition),
        }
        Ok(())
    }

    /// Whether the firmware holds the BSS in dynamic power save.
    pub fn power_save(&self) -> bool {
        self.state().power_save
    }

    /// Linux `mt7925_mcu_uni_bss_ps` for BSS_CHANGED_PS: the firmware saves
    /// power between beacons while `enable`, waking for traffic itself.
    ///
    /// # Safety
    /// As `update_slot_time_modern`.
    pub unsafe fn set_power_save_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        enable: bool,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        Self::command_ready(s)?;
        let profile = s.profile.ok_or(Error::WrongPhase)?;
        let body = profile.power_save_request(enable).map_err(Error::Codec)?;
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
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
                    Self::command_parts(controller, replies, Command::BssInfo, &body)?;
                    *phase = Phase::Associated;
                    Ok(())
                },
            )
        }
        .map_err(Error::Regulatory)??;
        s.power_save = enable;
        Ok(())
    }

    /// Linux `mt7925_mcu_set_rssimonitor` for BSS_CHANGED_CQM: firmware RSSI
    /// events past `threshold` dBm with `hysteresis`, or none for `None`.
    /// Firmware without RSSI_NOTIFY_EVT_EN, for which Linux leaves
    /// IEEE80211_VIF_SUPPORTS_CQM_RSSI clear, is watched from beacons.
    ///
    /// # Safety
    /// As `update_slot_time_modern`.
    pub unsafe fn set_rssi_monitor_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        monitor: Option<(i32, u32)>,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        let flags = s
            .controller
            .state()
            .policy
            .as_ref()
            .and_then(|policy| policy.capabilities().observed().chip_flags());
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        // Without firmware RSSI events mac80211 watches beacons instead.
        if flags.unwrap_or(0) & 2 == 0 {
            s.cqm.set(monitor, false);
            return Ok(());
        }
        Self::command_ready(s)?;
        let profile = s.profile.ok_or(Error::WrongPhase)?;
        let (threshold, hysteresis) = monitor.unwrap_or_default();
        let hysteresis = hysteresis as i32;
        let outgoing = Self::codec(s)
            .rssi_monitor(
                profile,
                monitor.is_some(),
                threshold.wrapping_add(hysteresis) as i8,
                threshold.wrapping_sub(hysteresis) as i8,
            )
            .map_err(Error::Codec)?;
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let phase = &mut s.phase;
        let controller = &mut s.controller;
        let replies = &mut s.replies;
        let facts = &mut s.transport_facts;
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            authority.publish_association(
                admission,
                &s.station,
                wifi::regulatory::AssociationUse::Management,
                |_, _| {
                    *phase = Phase::Quarantined;
                    Self::deliver(controller, replies, facts, &outgoing)?;
                    *phase = Phase::Associated;
                    Ok(())
                },
            )
        }
        .map_err(Error::Regulatory)??;
        s.cqm.set(monitor, true);
        Ok(())
    }

    pub fn pending_slot_time(&self) -> bool {
        self.state().slot_time.is_some()
    }

    /// Linux `mt7925_link_info_changed` on BSS_CHANGED_ERP_SLOT: sends the
    /// slot time a beacon changed through `mt7925_mcu_set_timing`.
    ///
    /// # Safety
    /// The exact-controller, applied-policy and revocation contracts of
    /// `update_edca_modern` apply.
    pub unsafe fn update_slot_time_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        let slot = s.slot_time.ok_or(Error::WrongPhase)?;
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        Self::command_ready(s)?;
        let mut profile = s.profile.ok_or(Error::WrongPhase)?;
        profile.slot_time = slot;
        let Some(body) = Self::codec(s).timing(profile).map_err(Error::Codec)? else {
            // connac2 keeps the slot time in the MAC's timing registers
            // (`mt792x_mac_set_timeing`), which this driver does not
            // program: report the change and keep the current timing.
            s.slot_time = None;
            s.csa.ready.try_reserve(1).map_err(|_| Error::NoMemory)?;
            s.csa.ready.push(Event::Unsupported(
                "a 2.4 GHz slot time change on MT7921 or MT7922: the MAC keeps its timing",
            ));
            return Ok(());
        };
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let phase = &mut s.phase;
        let current = &mut s.profile;
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
                    Self::command_parts(controller, replies, Command::BssInfo, &body)?;
                    *current = Some(profile);
                    *phase = Phase::Associated;
                    Ok(())
                },
            )
        }
        .map_err(Error::Regulatory)??;
        s.slot_time = None;
        Ok(())
    }

    /// No received event waits ahead of a command's reply.
    pub(super) fn command_ready(s: &Owned<I, Q, C>) -> Result<(), Error> {
        let controller = s.controller.state();
        if !s.pending_rx.is_empty() || !controller.mailbox.as_ref().unwrap().receive_idle() {
            return Err(Error::NeedsDrain);
        }
        Ok(())
    }

    fn validate_edca(s: &Owned<I, Q, C>, observed: &Observed<C>) -> Result<(), Error> {
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        let (profile, context, ptk, gtk) = if let Some(link) = s
            .links
            .iter()
            .find(|link| Arc::ptr_eq(&link.context, &observed.context))
        {
            if !s
                .station
                .link_contexts()
                .iter()
                .any(|context| Arc::ptr_eq(context, &link.context))
            {
                return Err(Error::Station(station::Error::Stale));
            }
            (
                link.profile,
                &link.context,
                Some(&link.ptk),
                Some(&link.gtk),
            )
        } else {
            let modern = s.modern.as_ref().ok_or(Error::WrongPhase)?;
            s.station
                .validate_association_context(modern.context())
                .map_err(Error::Station)?;
            (
                s.profile.ok_or(Error::WrongProfile)?,
                modern.context(),
                s.ptk.as_ref(),
                s.gtk.as_ref(),
            )
        };
        if !Arc::ptr_eq(context, &observed.context)
            || !Arc::ptr_eq(&observed.resource, &s.controller.state().resource)
            || profile != observed.profile
            || s.station.association_epoch() != Some(observed.association)
            || s.peer_generation != observed.peer_generation
            || ptk.and_then(|key| key.slot().installed()) != observed.ptk
            || gtk.and_then(|key| key.slot().installed()) != observed.gtk
        {
            return Err(Error::Station(station::Error::Stale));
        }
        if s.controller
            .state()
            .control
            .clock
            .now_us()
            .checked_sub(observed.seen)
            .is_none_or(|age| age > OBSERVATION_LIFETIME_US)
        {
            return Err(Error::GrantTimeout);
        }
        Ok(())
    }

    /// Install the latest completed current-peer WMM observation.
    /// `limit` is the current immutable control-channel regulatory WMM rule;
    /// clamping and request encoding happen inside publication guards.
    ///
    /// # Safety
    /// The exact-controller, applied-policy, revocation and kept autonomous
    /// user contracts of `configure_modern` apply. `limit` must belong to that
    /// same current OS country/control-channel rule, not an invented relaxation.
    /// As Linux `mt7925_mcu_set_tx` does, the command goes beside live
    /// traffic and block-ack sessions; only received events queued ahead of
    /// its reply must drain first. External publication/reset/IRQ
    /// paths serialize through this owner and permission; no raw user may publish new work during the checked drain and
    /// command. This function preserves keys/context and grants no new
    /// PHY, ACM, U-APSD, PMF or 6 GHz admission.
    pub unsafe fn update_edca_modern<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        limit: Option<&wifi::regulatory::database::Wmm>,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        let observed = Self::pending_edca_observation(s).ok_or(Error::WrongPhase)?;
        Self::validate_edca(s, observed)?;
        Self::command_ready(s)?;
        s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let clock = s.controller.state().control.clock;
        let codec = Self::codec(s);
        let (pending, applied) = if s.edca.is_some() {
            (
                &mut s.edca,
                &mut s.modern.as_mut().ok_or(Error::WrongPhase)?.applied_edca,
            )
        } else {
            let link = s
                .links
                .iter_mut()
                .find(|link| link.edca.is_some())
                .ok_or(Error::WrongPhase)?;
            (&mut link.edca, &mut link.applied_edca)
        };
        let observed = pending.as_ref().unwrap();
        let phase = &mut s.phase;
        let controller = &mut s.controller;
        let replies = &mut s.replies;
        let facts = &mut s.transport_facts;
        // SAFETY: covered by this function's exact-controller/policy contract.
        let result = unsafe {
            authority.publish_edca(
                admission,
                &s.station,
                wifi::regulatory::EdcaUpdate {
                    context: &observed.context,
                    parameter_set_count: observed.count,
                    observed: &observed.parameters,
                    limit,
                },
                |operation| {
                    if clock
                        .now_us()
                        .checked_sub(observed.seen)
                        .is_none_or(|age| age > OBSERVATION_LIFETIME_US)
                    {
                        return Err(Error::GrantTimeout);
                    }
                    let effective = *operation.effective();
                    // The AP's EDCA parameters come in its WMM element: QoS.
                    let outgoing = codec
                        .edca(observed.profile, operation.observed(), &effective, true)
                        .map_err(Error::Codec)?;
                    *phase = Phase::Quarantined;
                    Self::deliver(controller, replies, facts, &outgoing)?;
                    if clock
                        .now_us()
                        .checked_sub(observed.seen)
                        .is_none_or(|age| age > OBSERVATION_LIFETIME_US)
                    {
                        return Err(Error::GrantTimeout);
                    }
                    Ok(AppliedEdca {
                        parameter_set_count: observed.count,
                        access_categories: effective,
                    })
                },
            )
        }
        .map_err(Error::Regulatory)??;
        *applied = Some(result);
        *pending = None;
        s.phase = Phase::Associated;
        Ok(())
    }
}

fn parameters(
    mut elements: &[u8],
    negotiated: &station::phy::Negotiated,
    applied: &AppliedEdca,
) -> Result<Option<(u8, [AccessCategory; 4])>, Error> {
    if elements.len() > 2300 {
        return Err(Error::WrongProfile);
    }
    let mut observed = None;
    while !elements.is_empty() {
        let n = usize::from(*elements.get(1).ok_or(Error::WrongProfile)?);
        let data = elements.get(2..2 + n).ok_or(Error::WrongProfile)?;
        // MU-EDCA requires its own current count and firmware semantics.
        if elements[0] == 255 && data.first() == Some(&38) {
            return Err(Error::Unsupported);
        }
        if elements[0] == 221 && data.get(..4) == Some(&[0, 0x50, 0xf2, 2]) {
            if observed.is_some() {
                return Err(Error::WrongProfile);
            }
            observed = Some(Wmm::parse(data, Role::AccessPoint).map_err(|_| Error::WrongProfile)?);
        }
        elements = &elements[2 + n..];
    }
    let Some(observed) = observed else {
        return Ok(None);
    };
    let parameters = observed.parameters.ok_or(Error::Unsupported)?;
    negotiated.wmm.as_ref().ok_or(Error::Unsupported)?;
    let original_ap = negotiated.peer().wmm().ok_or(Error::Unsupported)?;
    // U-APSD continues to use the immutable association. Admission
    // control is host-side: data TIDs follow the applied flags, as
    // `ieee80211_sta_wmm_params` updates `wmm_acm`.
    if observed.qos_info & 0x80 != original_ap.qos_info & 0x80
        || parameters.iter().any(|ac| ac.ecw_min == 0)
    {
        return Err(Error::Unsupported);
    }
    let count = observed.qos_info & 15;
    // Pinned source uses equality, not ordering:15 ->0 is a real update.
    if count == applied.parameter_set_count {
        return Ok(None);
    }
    Ok(Some((count, parameters)))
}
