//! Borrow-free ownership of a failed short-reservation transfer.
use super::*;

#[derive(Clone, Copy)]
enum Stage {
    Reserved,
    Escrow,
}
struct TransferOwner<'runtime, const N: usize> {
    escrow: Option<OwnedGeneralEscrow<'runtime, N>>,
    stage: Stage,
}
/// An unpublished reservation whose ownership transfer failed. The same native
/// exclusion remains owned, without retaining the borrowed process permission.
/// Only an exact fresh-scope cancellation can prove native rollback completed.
/// Dropping/forgetting this value cannot authorize source resource cleanup.
#[must_use = "cancel under fresh same-creator permission; Drop retains resources"]
pub struct OwnedTransferFailure<'runtime, const N: usize> {
    pub error: WorkError,
    owner: TransferOwner<'runtime, N>,
}
/// Affine observation that this exact unpublished native reservation was
/// released. It proves no callback was accepted by this transfer; source
/// reverse-unwind, descriptor, DMA and task lifetime proofs remain separate.
#[must_use = "combine with the matching source request's independent no-exposure proof"]
pub struct CancelledReservation {
    queue: u64,
    context: ContextId,
    nonce: u64,
    _local: PhantomData<*mut ()>,
}
impl CancelledReservation {
    pub fn queue_id(&self) -> u64 {
        self.queue
    }
    pub fn context_id(&self) -> ContextId {
        self.context
    }
}
impl<const N: usize> core::fmt::Debug for OwnedTransferFailure<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("OwnedTransferFailure")
            .field("error", &self.error)
            .field("reservation", &self.owner.escrow)
            .finish_non_exhaustive()
    }
}
impl core::fmt::Debug for CancelledReservation {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("CancelledReservation")
            .field("queue", &self.queue)
            .field("context", &self.context)
            .field("nonce", &self.nonce)
            .finish()
    }
}
impl<const N: usize> core::fmt::Display for OwnedTransferFailure<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "native reservation ownership transfer: {:?}", self.error)
    }
}
impl<const N: usize> core::error::Error for OwnedTransferFailure<'_, N> {}
impl<'runtime, const N: usize> GeneralReservation<'runtime, '_, N> {
    /// Transfer native exclusion without leaking a process-scope borrow. An
    /// error owns the same reservation, not merely its status. Its counters and
    /// private stage are captured before the first fallible permission check.
    /// This accepts no callback and grants no C task/current/PID/stack access.
    #[expect(
        clippy::result_large_err,
        reason = "failure keeps exact native exclusion without allocation"
    )]
    pub fn into_owned(
        mut self,
    ) -> Result<OwnedGeneralEscrow<'runtime, N>, OwnedTransferFailure<'runtime, N>> {
        let entered = self.entered;
        let escrow = OwnedGeneralEscrow {
            runtime: self.runtime,
            queue: self.queue.clone(),
            creator: self.creator,
            creator_worker: self.creator_worker,
            worker: self.worker,
            context: self.context,
            slot: self.slot,
            nonce: self.nonce,
            requirement: self.requirement,
            child_rank: None,
            live: true,
            _local: PhantomData,
        };
        let mut owner = TransferOwner {
            escrow: Some(escrow),
            stage: Stage::Reserved,
        };
        // The private owned stage now keeps the same queue/slot/creator. The
        // short-lived destructor must never attempt a second ambiguous rollback.
        self.live = false;
        match owner.convert(entered) {
            Ok(()) => Ok(owner.escrow.take().expect("converted owned reservation")),
            Err(error) => Err(OwnedTransferFailure { error, owner }),
        }
    }
}
impl<const N: usize> TransferOwner<'_, N> {
    fn convert(&mut self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        let stage = &mut self.stage;
        let owner = self.escrow.as_mut().expect("kept transfer owner");
        owner.current(entered)?;
        let flags = owner.queue.cpu.save_disable()?;
        let result = (|| {
            owner.current(entered)?;
            let mut guard = owner.queue.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned || owner.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            Self::validate_reserved(owner, state)?;
            let total = owner
                .runtime
                .owned_escrows
                .get()
                .checked_add(1)
                .ok_or(WorkError::Exhausted)?;
            // SAFETY: fresh matching creator and IF0 exclude every peer; no
            // callback or native switch spans this counter read or mutation.
            let count = unsafe { (*owner.runtime.task(owner.creator_worker)).owned_escrows }
                .checked_add(1)
                .ok_or(WorkError::Exhausted)?;
            // All fallible checks precede the counter/state transfer. Its stage
            // changes before either nested or outer external flag restoration.
            // SAFETY: exact reserved worker/nonce and creator were validated.
            unsafe {
                (*owner.runtime.task(owner.worker)).state = TaskState::Escrow;
                (*owner.runtime.task(owner.creator_worker)).owned_escrows = count;
            }
            owner.runtime.owned_escrows.set(total);
            owner
                .runtime
                .pending_reservations
                .set(owner.runtime.pending_reservations.get() - 1);
            *stage = Stage::Escrow;
            Ok(())
        })();
        // SAFETY: exact enclosing save; no Task or queue borrow remains.
        unsafe { owner.queue.cpu.restore(flags) };
        result
    }
    fn validate_reserved(
        owner: &OwnedGeneralEscrow<'_, N>,
        state: &State<N>,
    ) -> Result<(), WorkError> {
        if owner.slot >= N
            || state.reservations[owner.slot] != owner.nonce
            || state.jobs[owner.slot].is_some()
            || owner.runtime.pending_reservations.get() == 0
        {
            return Err(WorkError::corrupt());
        }
        // SAFETY: caller checked fresh same-creator scope and IF0. Immutable
        // constructor identities/geometry and the private worker remain kept.
        let task = unsafe { &*owner.runtime.task(owner.worker) };
        if task.id != owner.context
            || task.lane != Lane::General
            || task.state != TaskState::Reserved
            || task.job.is_some()
            || task.binding.is_some()
            || task.stack_pin_nonce != 0
            || task.draining
            || task.signal.sequence.load(Ordering::Acquire) != 0
            || !stack_requirement::compatible(
                Some(owner.requirement),
                stack_requirement::Geometry::of(&task.stack),
            )
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    fn rollback_reserved(
        owner: &mut OwnedGeneralEscrow<'_, N>,
        entered: &dyn EnteredProcess,
    ) -> Result<(), WorkError> {
        owner.current(entered)?;
        let flags = owner.queue.cpu.save_disable()?;
        let result = (|| {
            owner.current(entered)?;
            let mut guard = owner.queue.lock()?;
            let state = guard.state();
            if state.poisoned || owner.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            Self::validate_reserved(owner, state)?;
            // SAFETY: exact unpublished excluded worker, no C pin/binding/job;
            // preserve its real immutable geometry and entry history.
            let shape = unsafe {
                let task = &*owner.runtime.task(owner.worker);
                ReleasedStackGeometry::of(&task.stack, !task.ever_entered)
            };
            state.resources.reservation_released(shape)?;
            state.reservations[owner.slot] = 0;
            // SAFETY: the fallible release note succeeded before this bounded
            // IF0 transition. No foreign producer owns the unbound worker.
            unsafe {
                (*owner.runtime.task(owner.worker)).state = TaskState::Idle;
            }
            owner
                .runtime
                .pending_reservations
                .set(owner.runtime.pending_reservations.get() - 1);
            owner.live = false;
            Ok(())
        })();
        // SAFETY: original flags after all Task/queue borrows ended. Unwind
        // after completed cancellation keeps live=false, never rolls back twice.
        unsafe { owner.queue.cpu.restore(flags) };
        result
    }
}
impl<const N: usize> OwnedTransferFailure<'_, N> {
    /// Roll back only this retained unpublished reservation under fresh matching
    /// creator permission. A closed queue permits cleanup; uncertain state does
    /// not. No borrowed permission survives in a returned failure or receipt.
    #[expect(
        clippy::result_large_err,
        reason = "refusal returns exact owning transfer failure"
    )]
    pub fn cancel(
        mut self,
        entered: &dyn EnteredProcess,
    ) -> Result<CancelledReservation, (Self, WorkError)> {
        let owner = self.owner.escrow.as_mut().expect("kept transfer failure");
        let result = match self.owner.stage {
            Stage::Reserved => TransferOwner::rollback_reserved(owner, entered),
            Stage::Escrow => owner.rollback(entered),
        };
        match result {
            Ok(()) => Ok(CancelledReservation {
                queue: owner.queue.id,
                context: owner.context,
                nonce: owner.nonce,
                _local: PhantomData,
            }),
            Err(error) => Err((self, error)),
        }
    }
}
