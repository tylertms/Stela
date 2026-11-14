//! Publish the exact owned General worker under fresh creator permission.
use super::*;
impl<const N: usize> OwnedGeneralEscrow<'_, N> {
    /// Commit one native callback to this excluded worker/job slot. The caller
    /// obtains fresh permission on the same creator after all resource waits.
    /// Every fallible check precedes publication; an error returns this exact
    /// escrow and accepts no callback. No callback executes inline.
    ///
    /// This is native publication only: it supplies neither Linux resources,
    /// PID visibility, child startup nor a terminal/stack-retirement receipt.
    /// A foreign task owner must coordinate its nonblocking publication under
    /// the same actual CPU exclusion before allowing the creator to switch.
    ///
    /// # Safety
    /// As Queue::execute for callback/context/code lifetime and geometry. Keep
    /// them before calling the publisher, including unwind in the outer CPU
    /// restore tail after publication but before a ticket can return. The
    /// worker's original Frame must be valid; a cold preparation must have
    /// explicitly reconstructed it before transferring its escrow here.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal returns the exact affine worker/slot owner without an allocation or an implicit uncertain cancellation"
    )]
    pub unsafe fn commit(
        mut self,
        entered: &dyn EnteredProcess,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<AssignedGeneral, (Self, WorkError)> {
        // SAFETY: the caller keeps this callback and its complete context graph.
        match unsafe { self.publish(entered, callback, context, None, None) } {
            Ok(assigned) => Ok(assigned),
            Err(error) => Err((self, error)),
        }
    }
    /// Internal pinned publication keeps the worker excluded after callback return.
    /// # Safety
    /// As commit; the caller already owns the corresponding retained queue/pin
    /// count and keeps it until exact successor release, including unwind.
    #[expect(
        clippy::result_large_err,
        reason = "The exact unpublished escrow is returned on refusal"
    )]
    pub(super) unsafe fn commit_pinned(
        mut self,
        entered: &dyn EnteredProcess,
        epoch: u64,
        dormant: Option<scheduling::Snapshot>,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<AssignedGeneral, (Self, WorkError)> {
        // SAFETY: the pinned owner keeps the complete publication graph/count.
        match unsafe { self.publish(entered, callback, context, Some(epoch), dormant) } {
            Ok(assigned) => Ok(assigned),
            Err(error) => Err((self, error)),
        }
    }
    pub(super) fn reserve_child(
        &mut self,
        entered: &dyn EnteredProcess,
        rank: scheduling::Snapshot,
    ) -> Result<(), WorkError> {
        self.current(entered)?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            self.current(entered)?;
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if self.runtime.retired.get()
                || state.closed
                || state.poisoned
                || self.queue.abandoned.load(Ordering::Acquire)
            {
                return Err(WorkError::Closed);
            }
            self.validate(state)?;
            if let Some((sequence, reserved_rank)) = self.child_rank {
                if reserved_rank != rank {
                    return Err(WorkError::InvalidTicket);
                }
                state
                    .scheduling
                    .reserved_child(self.context, sequence, rank)?;
                return Ok(());
            }

            let sequence = state.next;
            let next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
            state
                .scheduling
                .reserve_child(self.context, sequence, rank)?;
            self.child_rank = Some((sequence, rank));
            state.next = next;
            Ok(())
        })();
        // SAFETY: all queue/task borrows ended before restoring the enclosing flags.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }

    unsafe fn publish(
        &mut self,
        entered: &dyn EnteredProcess,
        callback: Callback,
        context: *mut c_void,
        epoch: Option<u64>,
        dormant: Option<scheduling::Snapshot>,
    ) -> Result<AssignedGeneral, WorkError> {
        self.current(entered)?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            self.current(entered)?;
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if self.runtime.retired.get()
                || state.closed
                || state.poisoned
                || self.queue.abandoned.load(Ordering::Acquire)
            {
                return Err(WorkError::Closed);
            }
            self.validate(state)?;
            // A pinned publication consumes its exact newly reconstructed
            // epoch. Plain native work cannot bypass an open physical epoch.
            // SAFETY: private owned worker/nonce checked at IF0, no callback.
            let task = unsafe { &*self.runtime.task(self.worker) };
            match epoch {
                Some(epoch)
                    if epoch != 0
                        && task.stack_epoch == epoch
                        && task.stack_retired_epoch != epoch => {}
                None if task.stack_epoch == task.stack_retired_epoch => {}
                _ => return Err(WorkError::corrupt()),
            }
            let (sequence, next) = if let Some((sequence, rank)) = self.child_rank {
                if dormant != Some(rank) || epoch.is_none() {
                    return Err(WorkError::InvalidTicket);
                }
                let slot = state
                    .scheduling
                    .reserved_child(self.context, sequence, rank)?;
                state.scheduling.publish_reserved_child(slot);
                self.child_rank = None;
                (sequence, state.next)
            } else {
                if dormant.is_some() {
                    return Err(WorkError::InvalidTicket);
                }
                let sequence = state.next;
                let next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
                state.scheduling.insert(self.context, sequence, false)?;
                (sequence, next)
            };
            let job = Job {
                callback,
                context: context as usize,
                lane: Lane::General,
                sequence,
                running: true,
                requirement: Some(self.requirement),
            };
            // No fallible operation or callback follows this ownership transfer.
            state.next = next;
            state.jobs[self.slot] = Some(job);
            state.reservations[self.slot] = 0;
            // SAFETY: matching worker/nonce and original creator were validated
            // under this IF0 queue guard. No task borrow survives restoration.
            unsafe {
                let task = &mut *self.runtime.task(self.worker);
                task.stack_pin_nonce = if epoch.is_some() { self.nonce } else { 0 };
                task.job = Some((self.slot, job));
                task.flags = flags;
                task.state = if dormant.is_some() {
                    TaskState::Dormant
                } else {
                    TaskState::Ready
                };
                task.signal.pending.store(false, Ordering::Relaxed);
                task.signal.sequence.store(sequence, Ordering::Release);
                (*self.runtime.task(self.creator_worker)).owned_escrows -= 1;
            }
            self.runtime
                .owned_escrows
                .set(self.runtime.owned_escrows.get() - 1);
            self.live = false;
            Ok(AssignedGeneral {
                ticket: Ticket {
                    queue: self.queue.id,
                    sequence,
                },
                context: self.context,
            })
        })();
        // SAFETY: exact enclosing flags; all queue/task borrows have ended.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
}
