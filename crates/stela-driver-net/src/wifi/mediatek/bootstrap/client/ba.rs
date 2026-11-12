// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2018 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2023 MediaTek Inc.
//! Recipient BA command/queue ownership; canonical Station owns the air
//! negotiation. No copied session, raw AMPDU or command encoder enables a BA.
use super::*;
use crate::wifi::mediatek::reorder::Reorder;
use station::ba::{Action as BaAction, RxSession};
mod transmit;
pub(super) use transmit::Transmit;

pub(super) struct Receive<C: 'static> {
    session: Arc<RxSession>,
    queue: Reorder<receive::Parent<C>>,
    deadline: Option<u64>,
}
impl<C> Receive<C> {
    pub(super) fn stop(&mut self) {
        self.deadline = None;
        self.queue.stop();
    }
    fn arm(&mut self, now: u64) {
        if !self.queue.needs_timer() {
            self.deadline = None;
        } else if self.deadline.is_none() {
            self.deadline = Some(now.saturating_add(u64::from(self.queue.timeout_ticks())));
        }
    }
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub fn pending_block_ack(&self) -> bool {
        !self.state().ba_actions.is_empty()
    }
    /// Earliest canonical TX negotiation/idle timeout. The serialized deferred
    /// owner must call poll_block_ack_timers at this deadline and continue while
    /// another TID is due. This does not install a platform timer/scheduler.
    pub fn next_transmit_deadline(&self) -> Option<u64> {
        self.state().station.next_tx_block_ack_deadline()
    }

    /// Apply only restrictive canonical timeout/cancellation, keeping its
    /// native owner. Firmware disable and any resulting protected local DELBA
    /// still require drain/ACK and separately guarded publication.
    pub fn poll_block_ack_timers(&mut self, now: u64) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        s.ba_actions.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let action = s.station.poll_tx_block_ack(now).map_err(Error::BlockAck)?;
        Self::queue_tx_ba(s, action, now)
    }

    /// Serialized deferred owner must call `poll` no later than this absolute
    /// microsecond deadline while active. Source 32-bit ticks use1MHz here;
    /// successive polls/RX must never skip its signed half-range. This is a
    /// requested timer, not a claim that the platform scheduler is installed.
    pub fn next_receive_deadline(&self) -> Option<u64> {
        self.state()
            .receive_ba
            .iter()
            .flatten()
            .filter_map(|r| r.deadline)
            .min()
    }

    pub(super) fn queue_block_ack(
        s: &mut Owned<I, Q, C>,
        action: BaAction,
        now: u64,
    ) -> Result<Vec<Ingress>, Error> {
        match action {
            BaAction::Tx(action) => {
                Self::queue_tx_ba(s, action, now)?;
                Ok(Vec::new())
            }
            BaAction::None => {
                if let Err(error) = Self::reconcile_tx_ba(s, now)
                    && !matches!(error, Error::NeedsDrain)
                {
                    return Err(error);
                }
                // DELBA can cancel a canonical installation before this native
                // owner ever publishes its queued start. Retire only that
                // proven-unused action; never turn a late ACK into success.
                while let Some(BaAction::Program {
                    ticket,
                    session,
                    enable: true,
                }) = s.ba_actions.front()
                {
                    if s.receive_ba[usize::from(session.tid())].is_some()
                        || s.station
                            .validate_rx_block_ack_program(*ticket, session, true)
                            .is_ok()
                    {
                        break;
                    }
                    let ticket = *ticket;
                    s.ba_actions.pop_front();
                    // SAFETY: this queued start was never published, so nothing used it.
                    let next = unsafe {
                        s.station
                            .complete_rx_block_ack(ticket, Completion::RejectedAndIdle)
                    }
                    .map_err(Error::BlockAck)?;
                    if !matches!(next, BaAction::None) {
                        s.ba_actions.push_front(next);
                        break;
                    }
                }
                Ok(Vec::new())
            }
            BaAction::Bar {
                session,
                start_sequence,
            } => {
                s.station
                    .validate_rx_block_ack(&session)
                    .map_err(Error::BlockAck)?;
                let native = s.receive_ba[usize::from(session.tid())]
                    .as_mut()
                    .ok_or(Error::WrongPhase)?;
                if !Arc::ptr_eq(&native.session, &session) {
                    return Err(Error::WrongPhase);
                }
                let mut released = Vec::new();
                released
                    .try_reserve_exact(usize::from(session.window()))
                    .map_err(|_| Error::NoMemory)?;
                native
                    .queue
                    .bar(start_sequence, |p| released.push(p))
                    .map_err(Error::Reorder)?;
                native.arm(now);
                Self::deliver_parents(s, released)
            }
            action => {
                s.ba_actions.try_reserve(1).map_err(|_| Error::NoMemory)?;
                if let BaAction::Program {
                    session,
                    enable: false,
                    ..
                } = &action
                {
                    // Canonical invalidation cancels an unsubmitted start or
                    // stale positive air response before a peer teardown can
                    // be blocked behind it. No active session is resurrected.
                    s.ba_actions.retain(|old| match old {
                        BaAction::Program { session: old, .. }
                        | BaAction::Transmit {
                            session: Some(old), ..
                        } => old.tid() != session.tid(),
                        _ => true,
                    });
                }
                s.ba_actions.push_back(action);
                Ok(Vec::new())
            }
        }
    }

    /// Execute the pending canonical BA action under the same whole-profile
    /// management permission as the current modern association. A successful
    /// Program queues its exact air response for another real publication;
    /// command ACK and packet enqueue are never reported as an air ACK.
    ///
    /// # Safety
    /// The same exact-controller/kept-policy/armed-autonomous-user contract
    /// as `configure_modern` applies through this source BA command and
    /// descriptor publication. This serialized owner has no concurrent RX,
    /// BAR, timer, key retirement or teardown callback. Platform timer servicing
    /// must obey `next_receive_deadline`; stop cancels those users.
    pub unsafe fn progress_block_ack<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        &mut self,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        source: &dyn PageSource,
        now: u64,
    ) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        Self::validate_ba_management(s)?;
        let reconciled = Self::reconcile_tx_ba(s, now)?;
        if reconciled && s.ba_actions.is_empty() {
            return Ok(());
        }
        if matches!(s.ba_actions.front(), Some(BaAction::Tx(_))) {
            // SAFETY: covered by this function's own safety contract.
            return unsafe { Self::progress_tx_ba(s, authority, admission, source, now) };
        }
        if matches!(
            s.ba_actions.front(),
            Some(BaAction::Program { enable: false, .. })
        ) {
            return Self::stop_ba_action(s);
        }
        let action = s.ba_actions.front().ok_or(Error::WrongPhase)?;
        match action {
            BaAction::Program {
                ticket,
                session,
                enable: true,
            } => {
                s.station
                    .validate_rx_block_ack_program(*ticket, session, true)
                    .map_err(Error::BlockAck)?;
                let index = usize::from(session.tid());
                if s.receive_ba[index].is_some() {
                    return Err(Error::Busy);
                }
                let queue = Reorder::new(
                    session.tid(),
                    session.start_sequence(),
                    session.window(),
                    1_000_000,
                )
                .map_err(Error::Reorder)?;
                let bodies = Self::on_every_link(s, |codec, p| {
                    codec.ba(
                        p,
                        session.tid(),
                        session.start_sequence(),
                        session.window(),
                        session.firmware_amsdu(),
                        true,
                        false,
                    )
                })?;
                s.replies.try_reserve(1).map_err(|_| Error::NoMemory)?;
                let (ticket, session) = (*ticket, session.clone());
                let station = &s.station;
                let controller = &mut s.controller;
                let replies = &mut s.replies;
                let mut started = false;
                // SAFETY: covered by this function's own safety contract.
                let outcome = unsafe {
                    authority.publish_association(
                        admission,
                        station,
                        wifi::regulatory::AssociationUse::Management,
                        |_, _| {
                            started = true;
                            bodies.iter().try_for_each(|body| {
                                Self::command_parts(
                                    controller,
                                    replies,
                                    Command::StationRecord,
                                    body,
                                )
                            })
                        },
                    )
                };
                if !started {
                    return outcome.map_err(Error::Regulatory).and_then(|v| v);
                }
                s.ba_actions.pop_front();
                s.receive_ba[index] = Some(Receive {
                    session: session.clone(),
                    queue,
                    deadline: None,
                });
                match outcome.map_err(Error::Regulatory).and_then(|v| v) {
                    Ok(()) => {
                        // SAFETY: covered by this function's own safety contract.
                        let action = unsafe {
                            s.station
                                .complete_rx_block_ack(ticket, Completion::Accepted)
                        }
                        .map_err(Error::BlockAck)?;
                        Self::queue_block_ack(s, action, now)?;
                        Ok(())
                    }
                    Err(error) => {
                        // SAFETY: covered by this function's own safety contract.
                        let _ = unsafe {
                            s.station
                                .complete_rx_block_ack(ticket, Completion::Uncertain)
                        };
                        s.phase = Phase::Quarantined;
                        Err(error)
                    }
                }
            }
            BaAction::Transmit {
                context,
                bytes,
                session,
                ..
            } => {
                s.station
                    .validate_association_context(context)
                    .map_err(Error::Station)?;
                if let Some(session) = session {
                    s.station
                        .validate_rx_block_ack(session)
                        .map_err(Error::BlockAck)?;
                }
                let submission = Self::prepare_ba_submission(s, bytes)?;
                let bytes = submission.bytes;
                let prepared = (submission.config, submission.owner, submission.grant);
                let mut started = false;
                // SAFETY: covered by this function's own safety contract.
                let result = unsafe {
                    authority.publish_association(
                        admission,
                        &s.station,
                        wifi::regulatory::AssociationUse::Management,
                        |_, _| {
                            started = true;
                            Self::publish_submission(&mut s.controller, source, &bytes, prepared)
                        },
                    )
                }
                .map_err(Error::Regulatory)?;
                let uncertain = s.controller.state().data_tx.as_ref().unwrap().is_poisoned();
                if started && (result.is_ok() || uncertain) {
                    s.ba_actions.pop_front();
                }
                if uncertain {
                    s.phase = Phase::Quarantined;
                }
                result.map(|_| ())
            }
            _ => Err(Error::WrongPhase),
        }
    }

    /// Restriction/disconnect-compatible BA disable. No new transmit
    /// permission is needed to stop; a resulting DELBA/replacement action stays
    /// pending until separately admitted. This does not fence requester DMA.
    pub fn progress_block_ack_stop(&mut self) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated {
            return Err(Error::WrongPhase);
        }
        let now = s.controller.state().control.clock.now_us();
        let reconciled = Self::reconcile_tx_ba(s, now)?;
        if reconciled && s.ba_actions.is_empty() {
            return Ok(());
        }
        if matches!(s.ba_actions.front(), Some(BaAction::Tx(_))) {
            Self::stop_tx_ba_action(s, now)
        } else {
            Self::stop_ba_action(s)
        }
    }
    fn stop_ba_action(s: &mut Owned<I, Q, C>) -> Result<(), Error> {
        let Some(BaAction::Program {
            ticket,
            session,
            enable: false,
        }) = s.ba_actions.front()
        else {
            return Err(Error::WrongPhase);
        };
        let (ticket, session) = (*ticket, session.clone());
        s.station
            .validate_rx_block_ack_program(ticket, &session, false)
            .map_err(Error::BlockAck)?;
        let index = usize::from(session.tid());
        let Some(native) = s.receive_ba[index].as_mut() else {
            // This exact private start never reached publication:
            // successful or uncertain command publication always keeps a
            // native owner, and uncertainty terminally quarantines Client.
            // There is no hardware/callback/session queue to disable here.
            s.ba_actions.pop_front();
            // SAFETY: the start was never published, so there are no users to retire.
            let action = unsafe {
                s.station
                    .complete_rx_block_ack(ticket, Completion::Accepted)
            }
            .map_err(Error::BlockAck)?;
            let now = s.controller.state().control.clock.now_us();
            Self::queue_block_ack(s, action, now)?;
            return Ok(());
        };
        if !Arc::ptr_eq(&native.session, &session) {
            return Err(Error::WrongPhase);
        }
        if s.ba_management.as_ref().is_some_and(|session| {
            s.controller
                .state()
                .data_tx
                .as_ref()
                .unwrap()
                .has_pending_context(|packet| {
                    packet
                        .management
                        .as_ref()
                        .is_some_and(|original| Arc::ptr_eq(original, session))
                })
        }) {
            // A published protected positive response may still be in real TX.
            // Keep this original native agreement until descriptor consumption,
            // firmware TX-free and successful unmap retire that Session's users.
            // Canonical invalidation already denies every new agreement use.
            return Err(Error::NeedsDrain);
        }
        native.stop(); // terminal software queue plus serialized timer/callback cancellation
        let bodies = Self::on_every_link(s, |codec, p| {
            codec.ba(
                p,
                session.tid(),
                session.start_sequence(),
                session.window(),
                session.firmware_amsdu(),
                false,
                false,
            )
        })?;
        if let Err(error) = bodies
            .iter()
            .try_for_each(|body| Self::command(s, Command::StationRecord, body))
        {
            // SAFETY: an uncertain publication keeps every owner, as the contract requires.
            let _ = unsafe {
                s.station
                    .complete_rx_block_ack(ticket, Completion::Uncertain)
            };
            return Err(error);
        }
        // Already-owned batches keep their observed session until discarded;
        // later completed RX is classified under the current agreement,
        // matching source SSN/PN semantics without inventing a wire generation.
        s.receive_groups.discard_tid(session.tid());
        s.pending_rx.retain(|f| {
            rx::decode(Chip::Mt7925, &f.frame.bytes)
                .ok()
                .and_then(|d| Header::parse(d.payload).ok())
                .is_none_or(|h| h.tid != Some(session.tid()))
        });
        s.receive_ba[index] = None;
        s.ba_actions.pop_front();
        // SAFETY: firmware acknowledged the stop and this TID's queued RX was discarded.
        let action = unsafe {
            s.station
                .complete_rx_block_ack(ticket, Completion::Accepted)
        }
        .map_err(Error::BlockAck)?;
        let now = s.controller.state().control.clock.now_us();
        Self::queue_block_ack(s, action, now)?;
        s.phase = Phase::Associated;
        Ok(())
    }

    pub(super) fn order_parent(
        s: &mut Owned<I, Q, C>,
        parent: receive::Parent<C>,
        now: u64,
    ) -> Result<Vec<Ingress>, Error> {
        let h = parent.head().header();
        let eligible = h.tid.is_some()
            && !h.receiver.is_multicast()
            && parent.head().bytes()[24] & 0x60 != 0x20; // source NOACK equality
        let session = h
            .tid
            .and_then(|tid| s.station.active_rx_block_ack(tid))
            .cloned();
        if !eligible || session.is_none() {
            return Self::deliver_parent(s, parent);
        }
        let session = session.unwrap();
        s.station
            .validate_rx_block_ack(&session)
            .map_err(Error::BlockAck)?;
        let native = s.receive_ba[usize::from(session.tid())]
            .as_mut()
            .ok_or(Error::WrongPhase)?;
        if !Arc::ptr_eq(&native.session, &session)
            || !Arc::ptr_eq(parent.context(), session.context())
        {
            return Err(Error::WrongPhase);
        }
        let sequence = h.sequence.ok_or(Error::WrongProfile)?;
        let mut released = Vec::new();
        released
            .try_reserve_exact(usize::from(session.window()) + 1)
            .map_err(|_| Error::NoMemory)?;
        native
            .queue
            .push(sequence, now as u32, parent, |p| released.push(p))
            .map_err(|e| Error::Reorder(e.error))?;
        native.arm(now);
        Self::deliver_parents(s, released)
    }
    pub(super) fn poll_reorder(s: &mut Owned<I, Q, C>, now: u64) -> Result<Vec<Ingress>, Error> {
        let mut released = Vec::new();
        let total = s
            .receive_ba
            .iter()
            .flatten()
            .map(|r| r.queue.buffered())
            .sum();
        released
            .try_reserve_exact(total)
            .map_err(|_| Error::NoMemory)?;
        for native in s.receive_ba.iter_mut().flatten() {
            if s.station.validate_rx_block_ack(&native.session).is_err() {
                native.stop();
                continue;
            }
            if native.deadline.is_some_and(|deadline| now >= deadline) {
                native.deadline = None;
                native
                    .queue
                    .poll(now as u32, |p| released.push(p))
                    .map_err(Error::Reorder)?;
                native.arm(now);
            }
        }
        Self::deliver_parents(s, released)
    }
    fn deliver_parents(
        s: &mut Owned<I, Q, C>,
        parents: Vec<receive::Parent<C>>,
    ) -> Result<Vec<Ingress>, Error> {
        let mut out = Vec::new();
        for parent in parents {
            let ingress = Self::deliver_parent(s, parent)?;
            out.try_reserve(ingress.len())
                .map_err(|_| Error::NoMemory)?;
            out.extend(ingress);
        }
        Ok(out)
    }
}
