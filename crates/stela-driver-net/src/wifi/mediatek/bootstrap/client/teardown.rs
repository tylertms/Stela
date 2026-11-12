// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Disconnecting, removing peers after their queues drain, and stopping the
//! interface.

use super::*;

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub(super) fn remove(s: &mut Owned<I, Q, C>, ticket: Ticket) -> Result<(), Error> {
        let p = s.profile.ok_or(Error::WrongProfile)?;
        s.remove_ticket = Some(ticket);
        s.link_removal = None;
        s.link_taken = false;
        s.phase = Phase::Removing;
        s.format_generation = None;
        // Canonical disconnect has already invalidated these sessions. This
        // serialized owner cancels its real timers/callbacks and terminally
        // discards buffered CPU parents before firmware peer removal.
        for native in s.receive_ba.iter_mut().flatten() {
            native.stop();
        }
        // Stop new publication first, then await TX-free, descriptor
        // consumption and mapping release before destroying firmware keys/peer.
        // A local queue-empty/index observation alone is insufficient.
        let q = s.controller.state().data_tx.as_ref().unwrap();
        if q.outstanding_tokens() != 0 || q.pending_descriptors() != 0 {
            return Ok(());
        }
        Self::abort_channel(s)?;
        let six = s.six_ghz.as_ref().map(|six| six.codec().clone());
        // Explicit key removal precedes peer destruction. Keep their canonical
        // histories and ACKs until independent old-user drain completes.
        for (key, receipt) in [
            (&mut s.ptk, &mut s.ptk_removed),
            (&mut s.gtk, &mut s.gtk_removed),
        ] {
            if let Some(key) = key
                && key.slot().installed().is_some()
            {
                *receipt = Some(
                    key.remove(s.controller.state_mut().mailbox.as_mut().unwrap())
                        .map_err(Error::Key)?,
                );
            }
        }
        Self::command(
            s,
            Command::BssInfo,
            &match &six {
                Some(codec) => codec.disable_beacon_pm(),
                None => p.disable_beacon_pm(),
            }
            .map_err(Error::Codec)?,
        )?;
        s.transport_facts
            .try_reserve(1)
            .map_err(|_| Error::NoMemory)?;
        let fact = s
            .controller
            .state_mut()
            .mailbox
            .as_mut()
            .unwrap()
            .initial(
                &match &six {
                    Some(codec) => codec.filter_request(false),
                    None => p.filter_request(false),
                }
                .map_err(Error::Codec)?,
            )
            .map_err(Error::Transport)?;
        s.transport_facts.push(fact);
        Self::command(
            s,
            Command::StationRecord,
            &match &six {
                Some(codec) => codec.peer_request(false),
                None => p.peer_request(false, false),
            }
            .map_err(Error::Codec)?,
        )?;
        s.controller
            .state_mut()
            .control
            .clear_peer_admission(p.wcid)
            .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
        let bss = match &six {
            Some(codec) => codec.bss_request().map_err(Error::Codec)?,
            None => Self::bss_request(s, p, false)?,
        };
        Self::command(s, Command::BssInfo, &bss)?;
        Self::remove_links(s)?;
        s.phase = Phase::NeedsDrain;
        Ok(())
    }

    pub fn disconnect(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        if !matches!(
            s.phase,
            Phase::Joining | Phase::AwaitingModernAdmission | Phase::Associated
        ) {
            return Err(Error::WrongPhase);
        }
        let a = s.station.disconnect().map_err(Error::Station)?;
        if let Action::Remove { ticket, failure } = a {
            Self::authentication_ended(s, failure);
            Self::remove(s, ticket)
        } else {
            Err(Error::WrongPhase)
        }
    }

    /// # Safety
    /// Peer/key removal responses above are complete AND all old RX/TX,
    /// radio, IRQ/deferred users and late notifications are independently
    /// drained. Empty TX tokens alone do not establish RX/firmware quiescence.
    /// The same reserved physical peer/BCMC slots stay owned across reconnect.
    pub unsafe fn retire_peer_after_drain(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase == Phase::Removing {
            return Err(Error::NeedsDrain);
        }
        if s.phase != Phase::NeedsDrain {
            return Err(Error::WrongPhase);
        }
        let q = s.controller.state().data_tx.as_ref().unwrap();
        if q.outstanding_tokens() != 0
            || q.pending_descriptors() != 0
            || !s.pending_rx.is_empty()
            || !s.controller.state().data_assembly.is_idle()
            || !s
                .controller
                .state()
                .mailbox
                .as_ref()
                .unwrap()
                .receive_idle()
        {
            return Err(Error::NeedsDrain);
        }
        for (key, receipt) in [
            (&mut s.ptk, &mut s.ptk_removed),
            (&mut s.gtk, &mut s.gtk_removed),
        ] {
            if let Some(r) = receipt.take() {
                // SAFETY: covered by this function's own safety contract.
                unsafe { key.as_mut().unwrap().retire_after_drain(r) }.map_err(Error::Key)?;
            }
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe { Self::retire_links(s) }?;
        s.station
            .complete_removal(
                s.remove_ticket.take().ok_or(Error::WrongPhase)?,
                Completion::Accepted,
            )
            .map_err(Error::Station)?;
        s.phase = Phase::Idle;
        // The inherited hardware/RX/work drain contract is now complete;
        // incomplete host-only parents can release their old owners.
        s.receive_groups.clear();
        s.ciphers = None;
        s.management_cipher = None;
        s.receive_ba = core::array::from_fn(|_| None);
        s.transmit_ba = core::array::from_fn(|_| None);
        s.ba_actions.clear();
        s.action.replies.clear();
        s.amsdu_off = false;
        s.ba_management = None;
        s.ba_management_pn = None;
        s.profile = None;
        s.modern = None;
        s.edca = None;
        s.slot_time = None;
        s.channel = None;
        s.channel_change = None;
        s.csa = csa::State::default();
        s.fragments.clear();
        s.power_save = false;
        s.management = None;
        s.pmf_selected = None;
        s.pmf_authentication = None;
        s.authentication_deadline = None;
        s.six_ghz = None;
        Self::unbind_links(s)
    }

    /// Interface disable ACKs then WFDMA disable/idle observation. This
    /// never claims the independent firmware/CPU/IOMMU fence from idle alone.
    pub fn stop(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Idle {
            return Err(Error::Busy);
        }
        s.phase = Phase::Quarantined;
        s.interface_removed = Some(
            s.interface
                .deactivate(s.controller.state_mut().mailbox.as_mut().unwrap())
                .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?,
        );
        // The same stop as the bring-up's, run to completion: one 100 ms
        // poll at most.
        crate::wifi::stepped::block_on(s.controller.state_mut().control.stop_dma(false))
            .map_err(|e| Error::Bootstrap(super::super::Error::Control(e)))?;
        s.phase = Phase::ControllerDrain;
        Ok(())
    }

    /// Release everything the client holds once its hardware was reset, as
    /// `mt7925e_mac_reset` frees its tokens and queues: the controller's rings
    /// and mappings, and the interfaces and keys the reset firmware forgot,
    /// in whatever phase the failure left them.
    ///
    /// # Safety
    /// The WLAN subsystem was reset (WFSYS) with its WFDMA stopped and every
    /// ring base reprogrammed by a later owner, so neither the old firmware
    /// nor its DMA reaches this client's memory; no interrupt or deferred user
    /// of this client remains.
    pub unsafe fn release_after_reset(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        s.phase = Phase::Quarantined;
        // SAFETY: the reset is this function's contract.
        unsafe { s.controller.retire_after_quiesce() }.map_err(Error::Bootstrap)?;
        for link in &mut s.links {
            // SAFETY: as above: the reset firmware holds none of them.
            unsafe {
                link.ptk.retire_after_reset();
                link.gtk.retire_after_reset();
                link.interface.retire_after_reset();
            }
        }
        for key in [&mut s.ptk, &mut s.gtk].into_iter().flatten() {
            // SAFETY: as above.
            unsafe { key.retire_after_reset() };
        }
        // SAFETY: as above.
        unsafe { s.interface.retire_after_reset() };
        s.phase = Phase::Stopped;
        Ok(())
    }

    /// # Safety
    /// Whole-controller radio/firmware/DMA/IOTLB/IRQ/work quiescence
    /// meets Controller::retire_after_quiesce. This is an independent platform
    /// fence, not a command ACK or a local stop-request boolean.
    pub unsafe fn stop_after_quiesce(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::ControllerDrain {
            return Err(Error::WrongPhase);
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe { s.controller.retire_after_quiesce() }.map_err(Error::Bootstrap)?;
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            s.interface
                .retire_after_drain(s.interface_removed.take().ok_or(Error::WrongPhase)?)
        }
        .map_err(|e| Error::Bootstrap(super::super::Error::Interface(e)))?;
        s.phase = Phase::Stopped;
        Ok(())
    }
}
