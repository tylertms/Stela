// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Polling the DMA queues, dispatching what they carry and dropping unprotected
//! robust frames.

use super::*;

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    /// Poll the DMA queues and dispatch synchronously. There is
    /// no public "assign current epoch" function for copied/deferred RX bytes.
    pub fn poll(
        &mut self,
        now: u64,
        source: &dyn PageSource,
        policy: &mut (impl station::RegulatoryPolicy + scan::Policy),
    ) -> Result<Vec<Event>, Error> {
        if self.station().requires_protected_management()
            && matches!(
                self.phase(),
                Phase::Joining | Phase::AwaitingModernAdmission | Phase::Associated
            )
        {
            return Err(Error::ModernAdmissionRequired);
        }
        self.poll_inner(now, source, policy, None, None)
    }

    /// Whether the last pass left the receive rings nothing to give, as the
    /// end of a NAPI poll does.
    pub(super) fn rings_empty(&self) -> bool {
        let s = self.state();
        s.rings_empty && s.pending_rx.is_empty()
    }

    /// One pass over the rings. The events of the frames taken before a
    /// frame that fails stay for the next pass, as mac80211 has already
    /// handed them on when it drops a frame, so a refused frame never takes
    /// the good ones before it with it.
    pub(super) fn poll_inner(
        &mut self,
        now: u64,
        source: &dyn PageSource,
        policy: &mut (impl station::RegulatoryPolicy + scan::Policy),
        guard: Option<&mut wifi::regulatory::PmfAuthenticationGuard<'_>>,
        six: Option<&mut wifi::regulatory::SixGhzAuthenticationGuard<'_>>,
    ) -> Result<Vec<Event>, Error> {
        let mut events = core::mem::take(&mut self.state_mut().carried_events);
        match self.poll_frames(now, source, policy, guard, six, &mut events) {
            Ok(()) => Ok(events),
            Err(error) => {
                self.state_mut().carried_events = events;
                Err(error)
            }
        }
    }

    fn poll_frames(
        &mut self,
        now: u64,
        source: &dyn PageSource,
        policy: &mut (impl station::RegulatoryPolicy + scan::Policy),
        mut guard: Option<&mut wifi::regulatory::PmfAuthenticationGuard<'_>>,
        mut six: Option<&mut wifi::regulatory::SixGhzAuthenticationGuard<'_>>,
        events: &mut Vec<Event>,
    ) -> Result<(), Error> {
        // A live 6 GHz transport is serviced only under its held providers;
        // cleanup after removal needs none.
        if self.state().six_ghz.is_some() && six.is_none() && self.phase() == Phase::Joining {
            return Err(Error::WrongPhase);
        }
        if self.station().phase() == station::Phase::ExternalAuthenticating {
            self.poll_station(now, source, policy)?;
        }
        if self.phase() == Phase::Scanning {
            let s = self.state_mut();
            let ticket = s.scan_ticket.ok_or(Error::WrongPhase)?;
            if s.scanner.poll(ticket, now).map_err(Error::Scan)? {
                self.cancel_scan()?;
            }
        }
        let s = self.state_mut();
        if !matches!(
            s.phase,
            Phase::Scanning
                | Phase::ScanCompleted
                | Phase::Joining
                | Phase::AwaitingModernAdmission
                | Phase::Associated
                | Phase::Removing
                | Phase::NeedsDrain
        ) {
            return Err(Error::WrongPhase);
        }
        if s.pending_rx.is_empty() {
            let raw = s.controller.poll_events().map_err(Error::Bootstrap)?;
            s.rings_empty = raw.is_empty();
            s.pending_rx
                .extend(raw.into_iter().map(|frame| receive::ReceivedFrame {
                    queue: receive::Queue::Event,
                    frame,
                }));
            if let Some(f) = s.controller.poll_data().map_err(Error::Bootstrap)? {
                s.rings_empty = false;
                s.pending_rx.push_back(receive::ReceivedFrame {
                    queue: receive::Queue::Data,
                    frame: f,
                });
            }
        }
        // TX-free stamps each release on the controller's clock, as mt76
        // times status out in jiffies.
        s.txs.expire(s.controller.state().control.clock.now_us());
        events.extend(s.txs.ready.drain(..).map(Event::TxStatus));
        events.append(&mut s.cqm.ready);
        while let Some(received) = s.pending_rx.pop_front() {
            let receive::ReceivedFrame { queue, frame: f } = received;
            // mt76_dma_rx_process frees a frame the hardware or the ring
            // dropped and goes on with the next; a split A-MSDU it belonged
            // to cannot complete.
            if f.drop {
                s.receive_groups.discard(queue);
                continue;
            }
            let kind = f
                .bytes
                .get(..4)
                .map(|b| u32::from_le_bytes(b.try_into().unwrap()) >> 27);
            if kind == Some(6) {
                let q = s.controller.state_mut().data_tx.as_mut().unwrap();
                let epoch = q.epoch();
                // SAFETY: this TX-free event came from the current data ring's own RX path.
                unsafe { q.apply_tx_free(epoch, &f.bytes) }.map_err(Error::Ring)?;
                continue;
            }
            if kind == Some(0) {
                if let Some(p) = s.profile {
                    let reports = s
                        .txs
                        .receive(&f.bytes, &Self::peer_wcids(s, p), p.channel.band);
                    events.extend(reports.into_iter().map(Event::TxStatus));
                }
                continue;
            }
            if kind.is_some_and(|k| k != 2 && k != 7) {
                s.asynchronous.push(f);
                continue;
            }
            let chip = s.controller.state().identity.chip;
            if let Ok(event) = crate::wifi::mediatek::mcu::Event::parse(chip, &f.bytes) {
                // mt7921_mcu_scan_event ends the scan on either CE scan-done
                // event; the MT7925's UNI event carries a done TLV.
                let scan_done = event.is_unsolicited()
                    && s.phase == Phase::Scanning
                    && if chip == Chip::Mt7925 {
                        event.event_id == 14
                            && wire::scan_done(event.payload).map_err(Error::Codec)?
                    } else {
                        wire::connac2::scan_done(event.event_id)
                    };
                if scan_done {
                    s.scanner
                        .completed(s.scan_ticket.unwrap(), scan::Outcome::Completed)
                        .map_err(Error::Scan)?;
                    s.phase = Phase::ScanCompleted;
                }
                // mt7925_mcu_rssi_monitor_event: each INFO TLV reports
                // the first TLV's RSSI against the threshold.
                if chip == Chip::Mt7925
                    && event.event_id == 0x41
                    && event.is_unsolicited()
                    && s.phase == Phase::Associated
                    && let Some(monitor) = s.cqm.monitor.filter(|m| m.firmware)
                {
                    events.extend(rssi_events(event.payload, monitor.threshold));
                    continue;
                }
                // mt7921_mcu_rssi_monitor_event: the RSSI of each BSS.
                if chip != Chip::Mt7925
                    && event.event_id == wire::connac2::RSSI_NOTIFY_EVENT
                    && event.is_unsolicited()
                    && s.phase == Phase::Associated
                    && let Some(monitor) = s.cqm.monitor.filter(|m| m.firmware)
                {
                    let bss = s.profile.unwrap().interface.bss_index;
                    if let Some(rssi) = wire::connac2::rssi(event.payload, bss) {
                        events.push(Event::Rssi {
                            rssi,
                            high: rssi > monitor.threshold,
                        });
                    }
                    continue;
                }
                // mt7921_mcu_tx_done_event: one TXS record by event.
                if chip != Chip::Mt7925
                    && event.event_id == wire::connac2::TX_DONE_EVENT
                    && event.is_unsolicited()
                    && let Some(p) = s.profile
                    && let Some(packet) = wire::connac2::tx_done_status(event.payload)
                {
                    let reports = s
                        .txs
                        .receive(&packet, &Self::peer_wcids(s, p), p.channel.band);
                    events.extend(reports.into_iter().map(Event::TxStatus));
                    continue;
                }
                // mt7921_mcu_connection_loss_event takes the CE beacon-loss
                // event's bss_idx; the MT7925's UNI event leads its TLV with it.
                let (lost, length) = if chip == Chip::Mt7925 {
                    (12, 12)
                } else {
                    (wire::connac2::BEACON_LOSS_EVENT, 4)
                };
                if event.event_id == lost
                    && event.is_unsolicited()
                    && s.phase == Phase::Associated
                    && event.payload.len() >= length
                    && event.payload[0] == s.profile.unwrap().interface.bss_index
                {
                    let action = s.station.disconnect().map_err(Error::Station)?;
                    Self::action_guarded(s, action, now, source, policy, guard.as_deref_mut())?;
                    events.push(Event::ConnectionLost(f));
                } else {
                    s.asynchronous.push(f);
                }
                continue;
            }
            if s.phase == Phase::ScanCompleted {
                s.asynchronous.push(f);
                continue;
            }
            // Cancelled external sessions keep old RX until the independent
            // drain fence; do not reinterpret it as a new association.
            if s.station.external_authentication().is_some()
                && matches!(s.phase, Phase::Removing | Phase::NeedsDrain)
            {
                s.asynchronous.push(f);
                continue;
            }
            let modern_data = s.modern.is_some() && s.phase == Phase::Associated;
            let frame = match Self::normalized(s, &f.bytes, modern_data, policy) {
                Ok(frame) => frame,
                Err(error) => {
                    // An invalid native receive cannot leave an older pending
                    // queue relaxation eligible after uncertain current input.
                    if modern_data {
                        s.edca = None;
                        for link in &mut s.links {
                            link.edca = None;
                        }
                    }
                    return Err(error);
                }
            };
            // ieee80211_rx_h_michael_mic_verify: with the MIC stripped, a
            // Michael failure the hardware reports goes to the OS, as
            // cfg80211_michael_mic_failure, and the frame is dropped.
            if modern_data && frame.security.tkip_mic_error && frame.header().data_present() {
                let header = frame.header();
                events.push(Event::MichaelMicFailure {
                    transmitter: header.transmitter.ok_or(Error::WrongProfile)?,
                    group: header.receiver.is_multicast(),
                    key_index: frame.security.key.map(|key| key.index()),
                });
                continue;
            }
            if modern_data
                && matches!(frame.header().control & 0x00fc, 0x0080 | 0x0050)
                && Self::other_link(s, &frame.radio).is_some()
            {
                Self::beacon_of_other_link(s, frame, events)?;
                events.push(Event::Raw(f));
                continue;
            }
            if s.phase == Phase::Scanning {
                let bss = s
                    .scanner
                    .observe(s.scan_ticket.unwrap(), &frame, policy)
                    .map_err(Error::Scan)?;
                let nontransmitted = scan::multiple_bssid::nontransmitted(&bss);
                for bss in core::iter::once(bss).chain(nontransmitted) {
                    s.pmf_observations.retain(|(_, old)| {
                        old.bss.bssid != bss.bssid || old.bss.channel != bss.channel
                    });
                    s.pmf_observations
                        .try_reserve(1)
                        .map_err(|_| Error::NoMemory)?;
                    if s.pmf_observations.len() == pmf::MAX_OBSERVATIONS {
                        s.pmf_observations.remove(0);
                    }
                    s.pmf_observations
                        .push((now, Arc::new(PmfBssObservation { bss: bss.clone() })));
                    events.push(Event::Bss(bss));
                }
                continue;
            }
            if s.station.phase() == station::Phase::ExternalAuthenticating {
                let session = s
                    .station
                    .external_authentication()
                    .ok_or(Error::WrongPhase)?
                    .clone();
                if s.authentication_deadline
                    .is_some_and(|deadline| s.controller.state().control.clock.now_us() >= deadline)
                {
                    let deadline = s.station.external_deadline().ok_or(Error::WrongPhase)?;
                    if let Some(action) = s
                        .station
                        .poll(now.max(deadline), policy)
                        .map_err(Error::Station)?
                    {
                        Self::action_guarded(s, action, now, source, policy, guard.as_deref_mut())?;
                    }
                    s.asynchronous.push(f);
                    continue;
                }
                if let Some(guard) = six.as_deref_mut() {
                    Self::qualify_six(s, guard, six_ghz::Use::Receive(&frame), now)?;
                }
                let before = s.station.external_deadline();
                let received = s
                    .station
                    .receive_external_authentication(&session, frame, now)
                    .map_err(Error::Station)?;
                Self::follow_external_deadline(s, before);
                if let Some(received) = received {
                    events.push(Event::ExternalAuthentication(received));
                }
                continue;
            }
            if modern_data && matches!(frame.header().control & 0x00fc, 0x0080 | 0x0050) {
                Self::observe_edca(s, frame)?;
                events.push(Event::Raw(f));
                continue;
            }
            if (s.management.is_some() || s.station.requires_protected_management())
                && wifi::security::management::robust(&frame)
            {
                let pmf = s.station.requires_protected_management();
                if pmf {
                    match Self::unencrypted_management(s, &frame) {
                        Unencrypted::Accept => {}
                        Unencrypted::Drop => continue,
                        Unencrypted::Report => {
                            events.push(Event::UnprotectedDeparture(f));
                            continue;
                        }
                        Unencrypted::Depart => {
                            let epoch = s.station.association_epoch().ok_or(Error::WrongPhase)?;
                            let body = frame.bytes().get(24..).unwrap_or(&[]);
                            let action = s
                                .station
                                .receive_protected_departure(epoch, &frame, body)
                                .map_err(Error::Station)?;
                            Self::action_guarded(
                                s,
                                action,
                                now,
                                source,
                                policy,
                                guard.as_deref_mut(),
                            )?;
                            continue;
                        }
                    }
                }
                let received = Self::receive_management(s, frame, &f.bytes)?;
                // ieee80211_rx_mgmt_deauth: the AP's authenticated departure
                // ends the association; the OS sees the frame too.
                if pmf && matches!(received.frame().header().control & 0x00fc, 0x00a0 | 0x00c0) {
                    let epoch = s.station.association_epoch().ok_or(Error::WrongPhase)?;
                    let action = s
                        .station
                        .receive_protected_departure(epoch, received.frame(), received.body())
                        .map_err(Error::Station)?;
                    events.push(Event::ProtectedManagement(received));
                    Self::action_guarded(s, action, now, source, policy, guard.as_deref_mut())?;
                    continue;
                }
                if Self::channel_switch_action(s, received.frame(), received.body()) {
                    continue;
                }
                // ieee80211_rx_h_action on the decrypted frame: block ack and
                // what the OS takes reach it; the rest the station answers.
                if pmf
                    && received.frame().header().control & 0x00fc == 0x00d0
                    && matches!(
                        Self::receive_protected_action(s, received.frame(), received.body())?,
                        action::Receive::Done
                    )
                {
                    continue;
                }
                events.push(Event::ProtectedManagement(received));
                continue;
            }
            let epoch = s.station.association_epoch().ok_or(Error::WrongPhase)?;
            if modern_data
                && Self::channel_switch_action(s, &frame, frame.bytes().get(24..).unwrap_or(&[]))
            {
                continue;
            }
            if modern_data && frame.header().control & 0x00fc == 0x00d0 {
                match Self::receive_action(s, &frame)? {
                    action::Receive::BlockAck => {}
                    action::Receive::Os => {
                        events.push(Event::Raw(f));
                        continue;
                    }
                    action::Receive::Done => continue,
                }
            }
            if modern_data
                && (frame.header().control & 0x00fc == 0x00d0
                    || frame.header().control & 0x00fc == 0x0084)
            {
                let action = s
                    .station
                    .receive_block_ack(epoch, &frame, now)
                    .map_err(Error::BlockAck)?;
                events.extend(
                    Self::queue_block_ack(s, action, now)?
                        .into_iter()
                        .map(Event::Ingress),
                );
                continue;
            }
            if frame.header().control & 12 == 0 {
                // The 6 GHz association response arrives under the held
                // transport's providers, as the exchange's frames did.
                if s.phase == Phase::Joining
                    && let Some(guard) = six.as_deref_mut()
                {
                    Self::qualify_six(s, guard, six_ghz::Use::Receive(&frame), now)?;
                }
                let action = s
                    .station
                    .receive(epoch, &frame, now, policy)
                    .map_err(Error::Station)?;
                Self::action_guarded(s, action, now, source, policy, guard.as_deref_mut())?;
            } else if s.phase == Phase::Associated {
                Self::validate_pmf_data(s)?;
                if frame.header().fragmented() {
                    // ieee80211_rx_h_defragment: the station has the MSDU with
                    // the last fragment, and refuses what it replays.
                    let (ptk, gtk) =
                        Self::receiving(&mut s.ptk, &mut s.gtk, &mut s.links, &frame.radio)?;
                    let slot = frame.security.key.map(|_| ptk.slot_mut());
                    if let Some(whole) = s.fragments.add(&frame, slot, now)
                        && let Ok(ingress) = s.station.receive_msdu_refs(
                            epoch,
                            &whole,
                            &mut [ptk.slot_mut(), gtk.slot_mut()],
                        )
                    {
                        events.push(Event::Ingress(ingress));
                    }
                    continue;
                }
                if modern_data
                    && (frame.header().protected()
                        || frame.header().amsdu
                        || frame.aggregation != wifi::Aggregation::default())
                {
                    events.extend(
                        Self::receive_parent(s, queue, frame, now)?
                            .into_iter()
                            .map(Event::Ingress),
                    );
                    continue;
                }
                let (ptk, gtk) =
                    Self::receiving(&mut s.ptk, &mut s.gtk, &mut s.links, &frame.radio)?;
                let ingress = s
                    .station
                    .receive_msdu_refs(epoch, &frame, &mut [ptk.slot_mut(), gtk.slot_mut()])
                    .map_err(Error::Data)?;
                events.push(Event::Ingress(ingress));
            }
        }
        Self::poll_link_removal(s);
        if core::mem::take(&mut s.link_taken) {
            events.push(Event::Unsupported(
                "an AP MLD taking a link from an association: it ends",
            ));
        }
        events.extend(Self::poll_reorder(s, now)?.into_iter().map(Event::Ingress));
        events.append(&mut s.csa.ready);
        events.extend(
            core::mem::take(&mut s.asynchronous)
                .into_iter()
                .map(Event::Raw),
        );
        if let Some((session, failure)) = s.authentication_end.take() {
            events.push(Event::ExternalAuthenticationEnded { session, failure });
        }
        for event in events.iter() {
            if let Event::TxStatus(status) = event
                && Some(status.receipt) == s.mlme_receipt
            {
                s.mlme_receipt = None;
                s.station
                    .management_status(status.acked, now)
                    .map_err(Error::Station)?;
            }
        }
        Ok(())
    }

    /// `ieee80211_drop_unencrypted_mgmt` for a robust frame on a PMF
    /// association. A frame protected with keys the association does not
    /// hold yet is dropped, as one `ieee80211_rx_h_decrypt` cannot decrypt;
    /// the PTK and IGTK are installed together, so one protected unicast
    /// frame between them is dropped too.
    pub(super) fn unencrypted_management(s: &Owned<I, Q, C>, frame: &Mpdu) -> Unencrypted {
        let h = frame.header();
        let departure = matches!(h.control & 0x00fc, 0x00a0 | 0x00c0);
        let keyed = s
            .ptk
            .as_ref()
            .is_some_and(|key| key.slot().installed().is_some());
        if !h.receiver.is_multicast() {
            return match (h.protected(), keyed, departure) {
                (true, true, _) if s.management.is_some() => Unencrypted::Accept,
                (true, ..) => Unencrypted::Drop,
                // Permit unprotected departures during the 4-way handshake.
                (false, false, true) => Unencrypted::Depart,
                (false, true, true) => Unencrypted::Report,
                (false, _, false) => Unencrypted::Drop,
            };
        }
        // BIP leaves the Protected field clear; its MMIE carries the key.
        let bytes = frame.bytes();
        let length = s
            .management_cipher
            .unwrap_or(ManagementCipher::Cmac128)
            .mmie_length();
        let mmie = bytes.len() >= 24 + length
            && bytes[bytes.len() - length..][..2] == [76, (length - 2) as u8];
        match (mmie, departure) {
            (false, true) => Unencrypted::Report,
            (false, false) => Unencrypted::Drop,
            (true, _) if s.management.is_some() => Unencrypted::Accept,
            (true, _) => Unencrypted::Drop,
        }
    }
}
