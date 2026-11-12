// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2023 MediaTek Inc.
//! Native source TX_START/TX_OPERATIONAL/TX_STOP owners. Canonical Station owns
//! the management exchange/counters; firmware owns air sequence/PN.
use super::*;
use station::ba::tx::{Action, Agreement, Session};
#[cfg(test)]
mod tests;

pub(crate) struct Transmit<C: 'static> {
    _resource: Arc<C>,
    session: Arc<Session>,
    blocked: bool,
    agreement: Option<Agreement>,
}

impl<I: BarIo, Q: QueueIo, C: 'static> Client<I, Q, C> {
    pub(in crate::wifi::mediatek::bootstrap::client) fn tx_ba_blocks_data(
        s: &Owned<I, Q, C>,
        tid: u8,
    ) -> bool {
        s.transmit_ba
            .get(usize::from(tid))
            .and_then(Option::as_ref)
            .is_some_and(|native| native.blocked)
    }

    /// Request the same source TID's aggregation exchange. New borrowed
    /// Ethernet submissions return Busy while blocked; their caller keeps
    /// those bytes. Already-published packets stay in the original DataRing.
    pub fn begin_tx_block_ack(&mut self, tid: u8, timeout_tu: u16, now: u64) -> Result<(), Error> {
        let s = self.state_mut();
        if s.phase != Phase::Associated || s.modern.is_none() {
            return Err(Error::WrongPhase);
        }
        // Management RX stays an OS observation until explicitly consumed.
        // Legacy initiation cannot select negotiated protected BA policy or
        // bypass the separate OS-selected management entry point.
        if s.management.is_some() || s.station.requires_protected_management() {
            return Err(Error::Unsupported);
        }
        if s.transmit_ba
            .get(usize::from(tid))
            .is_some_and(Option::is_some)
        {
            return Err(Error::Busy);
        }
        s.ba_actions.try_reserve(1).map_err(|_| Error::NoMemory)?;
        let action = s
            .station
            .begin_tx_block_ack(tid, timeout_tu, now)
            .map_err(Error::BlockAck)?;
        Self::queue_tx_ba(s, action, now)
    }

    pub(in crate::wifi::mediatek::bootstrap::client) fn queue_tx_ba(
        s: &mut Owned<I, Q, C>,
        action: Action,
        now: u64,
    ) -> Result<(), Error> {
        match action {
            Action::None => match Self::reconcile_tx_ba(s, now) {
                Ok(_) | Err(Error::NeedsDrain) => Ok(()),
                Err(error) => Err(error),
            },
            Action::Operational { session, agreement } => {
                let native = s.transmit_ba[usize::from(session.tid())]
                    .as_mut()
                    .ok_or(Error::WrongPhase)?;
                if !Arc::ptr_eq(&native.session, &session) {
                    return Err(Error::WrongPhase);
                }
                s.station
                    .validate_tx_block_ack(&session)
                    .map_err(Error::BlockAck)?;
                native.agreement = Some(agreement);
                native.blocked = false;
                Ok(())
            }
            Action::ResumeLegacy { session } => {
                let native = s.transmit_ba[usize::from(session.tid())]
                    .as_ref()
                    .ok_or(Error::WrongPhase)?;
                if !Arc::ptr_eq(&native.session, &session) {
                    return Err(Error::WrongPhase);
                }
                if Self::tx_ba_pending(s, &session) {
                    // A local DELBA was published; canonical state is
                    // retired, but its real packet still owns this native TID.
                    // Preserve the successor until TX-free and unmap.
                    s.ba_actions.try_reserve(1).map_err(|_| Error::NoMemory)?;
                    s.ba_actions
                        .push_front(BaAction::Tx(Action::ResumeLegacy { session }));
                    return Ok(());
                }
                s.transmit_ba[usize::from(session.tid())] = None;
                Ok(())
            }
            action => {
                s.ba_actions.try_reserve(1).map_err(|_| Error::NoMemory)?;
                match &action {
                    Action::Prepare { session, .. } => {
                        let index = usize::from(session.tid());
                        if s.transmit_ba[index].is_some() {
                            return Err(Error::Busy);
                        }
                        s.transmit_ba[index] = Some(Transmit {
                            _resource: s.controller.state().resource.clone(),
                            session: session.clone(),
                            blocked: true,
                            agreement: None,
                        });
                    }
                    Action::Program { session, .. } => {
                        let native = s.transmit_ba[usize::from(session.tid())]
                            .as_mut()
                            .ok_or(Error::WrongPhase)?;
                        if !Arc::ptr_eq(&native.session, session) {
                            return Err(Error::WrongPhase);
                        }
                        native.blocked = true;
                    }
                    _ => {}
                }
                s.ba_actions.push_back(BaAction::Tx(action));
                Ok(())
            }
        }
    }

    fn tx_ba_pending(s: &Owned<I, Q, C>, session: &Arc<Session>) -> bool {
        s.controller
            .state()
            .data_tx
            .as_ref()
            .unwrap()
            .has_pending_context(|p| {
                p.tid == Some(session.tid())
                    || p.tx_ba
                        .as_ref()
                        .is_some_and(|old| Arc::ptr_eq(old, session))
            })
    }

    /// The front action is still privately queued: successful publication
    /// removes it immediately, and uncertain publication quarantines Client.
    /// Canonical cancellation alone never proves this native fact or idle TX.
    pub(super) fn reconcile_tx_ba(s: &mut Owned<I, Q, C>, now: u64) -> Result<bool, Error> {
        let Some(BaAction::Tx(action)) = s.ba_actions.front() else {
            return Ok(false);
        };
        if let Action::ResumeLegacy { session } = action {
            let native = s.transmit_ba[usize::from(session.tid())]
                .as_ref()
                .ok_or(Error::WrongPhase)?;
            if !Arc::ptr_eq(&native.session, session) || !native.blocked {
                return Err(Error::WrongPhase);
            }
            if Self::tx_ba_pending(s, session) {
                return Err(Error::NeedsDrain);
            }
            // The original canonical record already retired at DELBA
            // publication. Only this private native successor and the
            // real completed packet lifecycle authorize releasing its TID.
            let index = usize::from(session.tid());
            s.transmit_ba[index] = None;
            s.ba_actions.pop_front();
            return Ok(true);
        }
        if s.station
            .validate_tx_block_ack_cancellation(action)
            .is_err()
        {
            return Ok(false);
        }
        let (ticket, session) = match action {
            Action::Prepare { ticket, session }
            | Action::Transmit {
                ticket, session, ..
            }
            | Action::Program {
                ticket, session, ..
            } => (*ticket, session.clone()),
            _ => return Err(Error::WrongPhase),
        };
        let native = s.transmit_ba[usize::from(session.tid())]
            .as_ref()
            .ok_or(Error::WrongPhase)?;
        if !Arc::ptr_eq(&native.session, &session) || !native.blocked {
            return Err(Error::WrongPhase);
        }
        if Self::tx_ba_pending(s, &session) {
            return Err(Error::NeedsDrain);
        }
        // SAFETY: the exact canceled action remains unpublished at the private
        // front; real descriptor consumption, TX-free and unmap have retired
        // this TID's preceding packets, and the same native owner stays held.
        let next = unsafe {
            match action {
                Action::Prepare { .. } => s.station.complete_tx_block_ack_prepare(
                    ticket,
                    Completion::RejectedAndIdle,
                    None,
                    now,
                ),
                Action::Transmit { .. } => s.station.complete_tx_block_ack_transmit(
                    ticket,
                    Completion::RejectedAndIdle,
                    now,
                ),
                Action::Program { .. } => s
                    .station
                    .complete_tx_block_ack_program(ticket, Completion::RejectedAndIdle),
                _ => unreachable!(),
            }
        }
        .map_err(Error::BlockAck)?;
        // Keep the successor in the already allocated queue slot. In particular
        // an allocation failure must not lose a required disable/resume owner.
        *s.ba_actions.front_mut().unwrap() = BaAction::Tx(next);
        if let Some(BaAction::Tx(Action::ResumeLegacy { session })) = s.ba_actions.front() {
            let session = session.clone();
            Self::queue_tx_ba(s, Action::ResumeLegacy { session }, now)?;
            s.ba_actions.pop_front();
        }
        Ok(true)
    }

    pub(super) unsafe fn progress_tx_ba<
        H: wifi::regulatory::HardwarePolicy,
        O: wifi::regulatory::OsPolicy,
    >(
        s: &mut Owned<I, Q, C>,
        authority: &mut wifi::regulatory::Owner<H, O>,
        admission: wifi::regulatory::AssociationAdmission,
        source: &dyn PageSource,
        now: u64,
    ) -> Result<(), Error> {
        let Some(BaAction::Tx(action)) = s.ba_actions.front() else {
            return Err(Error::WrongPhase);
        };
        match action {
            Action::Prepare { ticket, session } => {
                let (ticket, session) = (*ticket, session.clone());
                s.station
                    .validate_tx_block_ack_prepare(ticket, &session)
                    .map_err(Error::BlockAck)?;
                if s.controller
                    .state()
                    .data_tx
                    .as_ref()
                    .unwrap()
                    .has_pending_context(|p| p.tid == Some(session.tid()))
                {
                    return Err(Error::NeedsDrain);
                }
                // Serialized TID block plus descriptor/TX-free/unmap
                // retirement precede this real canonical software snapshot.
                // It is never a hardware observation or an invented zero.
                let next = s
                    .station
                    .next_qos_sequence(session.context(), session.tid())
                    .map_err(Error::Station)?;
                // SAFETY: covered by this function's own safety contract.
                let action = unsafe {
                    s.station.complete_tx_block_ack_prepare(
                        ticket,
                        Completion::Accepted,
                        Some(next),
                        now,
                    )
                }
                .map_err(Error::BlockAck)?;
                s.ba_actions.pop_front();
                Self::queue_tx_ba(s, action, now)
            }
            Action::Transmit {
                ticket,
                session,
                bytes,
            } => {
                let (ticket, session) = (*ticket, session.clone());
                s.station
                    .validate_tx_block_ack_transmit(ticket, &session, bytes)
                    .map_err(Error::BlockAck)?;
                let mut submission = Self::prepare_ba_submission(s, bytes)?;
                submission.owner.tx_ba = Some(session.clone());
                let bytes = submission.bytes;
                let prepared = (submission.config, submission.owner, submission.grant);
                // SAFETY: covered by this function's own safety contract.
                let result = unsafe {
                    authority.publish_association(
                        admission,
                        &s.station,
                        wifi::regulatory::AssociationUse::Management,
                        |_, _| {
                            Self::publish_submission(&mut s.controller, source, &bytes, prepared)
                        },
                    )
                }
                .map_err(Error::Regulatory)?;
                match result {
                    Ok(_) => {
                        s.ba_actions.pop_front();
                        // SAFETY: covered by this function's own safety contract.
                        let action = unsafe {
                            s.station.complete_tx_block_ack_transmit(
                                ticket,
                                Completion::Accepted,
                                now,
                            )
                        }
                        .map_err(Error::BlockAck)?;
                        Self::queue_tx_ba(s, action, now)
                    }
                    Err(error) => {
                        if s.controller.state().data_tx.as_ref().unwrap().is_poisoned() {
                            // SAFETY: covered by this function's own safety contract.
                            let _ = unsafe {
                                s.station.complete_tx_block_ack_transmit(
                                    ticket,
                                    Completion::Uncertain,
                                    now,
                                )
                            };
                            s.phase = Phase::Quarantined;
                        }
                        Err(error)
                    }
                }
            }
            Action::Program {
                ticket,
                session,
                agreement,
            } => {
                if agreement.is_none() {
                    return Self::stop_tx_ba_action(s, now);
                }
                let (ticket, session, agreement) = (*ticket, session.clone(), *agreement);
                s.station
                    .validate_tx_block_ack_program(ticket, &session, agreement)
                    .map_err(Error::BlockAck)?;
                let agreement = agreement.unwrap();
                // mt7925_mcu_uni_tx_ba: an agreement without A-MSDU turns
                // hardware A-MSDU off for the station.
                if !agreement.amsdu_in_ampdu() {
                    s.amsdu_off = true;
                }
                // Generic TX_OPERATIONAL's source params.ssn is0; the request
                // software snapshot is a separate management input, not this.
                let bodies = Self::on_every_link(s, |codec, p| {
                    codec.ba(
                        p,
                        session.tid(),
                        0,
                        agreement.window(),
                        agreement.amsdu_in_ampdu(),
                        true,
                        true,
                    )
                })?;
                let mut started = false;
                // SAFETY: covered by this function's own safety contract.
                let result = unsafe {
                    authority.publish_association(
                        admission,
                        &s.station,
                        wifi::regulatory::AssociationUse::Management,
                        |_, _| {
                            started = true;
                            bodies.iter().try_for_each(|body| {
                                Self::command_parts(
                                    &mut s.controller,
                                    &mut s.replies,
                                    Command::StationRecord,
                                    body,
                                )
                            })
                        },
                    )
                }
                .map_err(Error::Regulatory)?;
                match result {
                    Ok(()) => {
                        s.ba_actions.pop_front();
                        // SAFETY: covered by this function's own safety contract.
                        let action = unsafe {
                            s.station
                                .complete_tx_block_ack_program(ticket, Completion::Accepted)
                        }
                        .map_err(Error::BlockAck)?;
                        Self::queue_tx_ba(s, action, now)
                    }
                    Err(error) => {
                        if started {
                            // SAFETY: covered by this function's own safety contract.
                            let _ = unsafe {
                                s.station
                                    .complete_tx_block_ack_program(ticket, Completion::Uncertain)
                            };
                            s.phase = Phase::Quarantined;
                        }
                        Err(error)
                    }
                }
            }
            _ => Err(Error::WrongPhase),
        }
    }

    pub(super) fn stop_tx_ba_action(s: &mut Owned<I, Q, C>, now: u64) -> Result<(), Error> {
        let Some(BaAction::Tx(Action::Program {
            ticket,
            session,
            agreement: None,
        })) = s.ba_actions.front()
        else {
            return Err(Error::WrongPhase);
        };
        let (ticket, session) = (*ticket, session.clone());
        s.station
            .validate_tx_block_ack_program(ticket, &session, None)
            .map_err(Error::BlockAck)?;
        if Self::tx_ba_pending(s, &session) {
            return Err(Error::NeedsDrain);
        }
        let bodies = Self::on_every_link(s, |codec, p| {
            codec.ba(p, session.tid(), 0, 1, false, false, true)
        })?;
        if let Err(error) = bodies
            .iter()
            .try_for_each(|body| Self::command(s, Command::StationRecord, body))
        {
            // SAFETY: an uncertain publication keeps every owner, as the contract requires.
            let _ = unsafe {
                s.station
                    .complete_tx_block_ack_program(ticket, Completion::Uncertain)
            };
            s.phase = Phase::Quarantined;
            return Err(error);
        }
        s.ba_actions.pop_front();
        // SAFETY: firmware acknowledged the disable and no TX for this TID is pending.
        let action = unsafe {
            s.station
                .complete_tx_block_ack_program(ticket, Completion::Accepted)
        }
        .map_err(Error::BlockAck)?;
        Self::queue_tx_ba(s, action, now)?;
        s.phase = Phase::Associated;
        Ok(())
    }
}
