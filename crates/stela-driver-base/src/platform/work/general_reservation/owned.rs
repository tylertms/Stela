//! Owned exclusion across creator suspension; no stack loan or child publication.
use super::*;
mod commit;
mod prepared;
mod transfer;
pub use prepared::{
    DormantGeneralStack, GeneralStackPreparation, OwnedPinnedGeneral,
    PinnedGeneralStackPreparation, PublishedGeneralStack, StackPinError,
};
pub use transfer::{CancelledReservation, OwnedTransferFailure};

/// Keeps a compatible idle worker and job slot without a process-scope
/// borrow. The lifetime pins the Runtime and its StackBacking owners;
/// this is not an Arc<Runtime>, stack pointer grant, ticket or Linux task.
///
/// Explicit cancel requires fresh permission for the same creator job.
/// Drop cannot obtain that permission and therefore closes and quarantines the
/// captured queue atomically, without accessing the affine Runtime. Forgetting
/// the value keeps its markers/counts and prevents creator return/retirement.
#[must_use = "cancel with fresh entered authority; Drop retains and closes admission"]
pub struct OwnedGeneralEscrow<'runtime, const N: usize> {
    runtime: Pin<&'runtime Runtime<N>>,
    queue: Arc<Queue<N>>,
    creator: Creator,
    creator_worker: usize,
    worker: usize,
    context: ContextId,
    slot: usize,
    nonce: u64,
    requirement: StackRequirement,
    child_rank: Option<(u64, scheduling::Snapshot)>,
    live: bool,
    _local: PhantomData<*mut ()>,
}
impl<const N: usize> OwnedGeneralEscrow<'_, N> {
    fn current(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        entered.check()?;
        if !self.creator.matches(self.runtime.current_context()?)
            || self.runtime.current.get() != self.creator_worker
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    fn validate(&self, state: &State<N>) -> Result<(), WorkError> {
        self.validate_state(state, TaskState::Escrow)
    }
    fn validate_state(&self, state: &State<N>, expected: TaskState) -> Result<(), WorkError> {
        if self.slot >= N
            || state.reservations[self.slot] != self.nonce
            || state.jobs[self.slot].is_some()
            || self.runtime.owned_escrows.get() == 0
        {
            return Err(WorkError::corrupt());
        }
        // SAFETY: caller proved fresh permission and exact creator at IF0;
        // both constructor-captured workers stay held and stopped here.
        let (task, creator) = unsafe {
            (
                &*self.runtime.task(self.worker),
                &*self.runtime.task(self.creator_worker),
            )
        };
        if task.id != self.context
            || task.lane != Lane::General
            || task.state != expected
            || task.job.is_some()
            || task.binding.is_some()
            || task.stack_pin_nonce != 0
            || task.draining
            || task.signal.sequence.load(Ordering::Acquire) != 0
            || creator.owned_escrows == 0
            || !stack_requirement::compatible(
                Some(self.requirement),
                stack_requirement::Geometry::of(&task.stack),
            )
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    fn rollback(&mut self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        self.current(entered)?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            self.current(entered)?;
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if state.poisoned || self.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            self.validate(state)?;
            // SAFETY: validate checked this same unpublished reserved worker,
            // no binding/C pin/job owns it; original geometry/history are kept.
            let shape = unsafe {
                let task = &*self.runtime.task(self.worker);
                ReleasedStackGeometry::of(&task.stack, !task.ever_entered)
            };
            let reserved_child = self
                .child_rank
                .map(|(sequence, rank)| {
                    state
                        .scheduling
                        .reserved_child(self.context, sequence, rank)
                })
                .transpose()?;
            state.resources.reservation_released(shape)?;
            if let Some(slot) = reserved_child {
                state.scheduling.cancel_reserved_child(slot);
                self.child_rank = None;
            }
            state.reservations[self.slot] = 0;
            // SAFETY: exact private worker and creator ownership checked above;
            // no published callback/stack loan exists and peers remain stopped.
            unsafe {
                (*self.runtime.task(self.worker)).state = TaskState::Idle;
                (*self.runtime.task(self.creator_worker)).owned_escrows -= 1;
            }
            self.runtime
                .owned_escrows
                .set(self.runtime.owned_escrows.get() - 1);
            self.live = false;
            Ok(())
        })();
        // SAFETY: exact cancel entry flags; all affine borrows ended before restore.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
    /// Release only this unpublished exclusion under fresh permission for
    /// its original creator job. A closed queue still allows exact cleanup;
    /// stale, foreign or uncertain ownership instead keeps the whole executor.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal returns the exact affine reservation without allocating during cancellation"
    )]
    pub fn cancel(mut self, entered: &dyn EnteredProcess) -> Result<(), (Self, WorkError)> {
        match self.rollback(entered) {
            Ok(()) => Ok(()),
            Err(error) => Err((self, error)),
        }
    }
    fn abandon(&mut self) {
        // Only the independently atomic queue view is touched. Never
        // inspect Runtime/stack metadata without fresh entered permission.
        self.queue.abandoned.store(true, Ordering::Release);
        quarantine();
        self.live = false;
    }
}
impl<const N: usize> Drop for OwnedGeneralEscrow<'_, N> {
    fn drop(&mut self) {
        if self.live {
            self.abandon();
        }
    }
}

impl<const N: usize> core::fmt::Debug for OwnedGeneralEscrow<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Captured immutable identity only; never touches affine Runtime state.
        f.debug_struct("OwnedGeneralEscrow")
            .field("context", &self.context)
            .field("nonce", &self.nonce)
            .finish_non_exhaustive()
    }
}
