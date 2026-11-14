//! Original TASK_NEW reservation: accepted native work is not runnable yet.
use super::*;

/// Exact accepted native job/physical pin held nonrunnable until its original
/// source owner publishes the C child and explicitly activates it. Abandoning
/// this owner retains the worker, scheduler slot/order credit, code and backing.
#[must_use = "activate only after source publication; Drop permanently retains"]
pub struct DormantGeneralStack<'runtime, const N: usize> {
    published: PublishedGeneralStack<'runtime, N>,
}
impl<const N: usize> core::fmt::Debug for DormantGeneralStack<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("DormantGeneralStack")
            .field("epoch", &self.published.epoch)
            .finish_non_exhaustive()
    }
}
impl<'runtime, const N: usize> OwnedPinnedGeneral<'runtime, N> {
    /// Keep scheduling and sequence capacity while the source copies the child.
    /// Neither the scheduler nor the queue can run the child during this phase.
    pub fn reserve_child_rank(
        &mut self,
        entered: &dyn EnteredProcess,
        rank: scheduling::Snapshot,
    ) -> Result<(), WorkError> {
        self.check(entered)?;
        self.escrow.reserve_child(entered, rank)
    }

    /// Reserve the final native job and scheduling/order capacity before the
    /// original nofail C publication point. This executes no child callback.
    /// Success cannot be treated as an unpublished/canceled fork refusal.
    ///
    /// # Safety
    /// All commit callback/context/root/pin lifetimes apply. Original source
    /// resource copying has reached its true nofail publication boundary. Keep
    /// that exact graph on uncertain C/native publication or outer restore
    /// unwind; no ordinary original unwind may reclaim an accepted job.
    #[expect(clippy::result_large_err, reason = "exact retained affine owner")]
    pub unsafe fn publish_dormant(
        mut self,
        entered: &dyn EnteredProcess,
        rank: scheduling::Snapshot,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<DormantGeneralStack<'runtime, N>, (Self, WorkError)> {
        if let Err(error) = self.reserve_child_rank(entered, rank) {
            return Err((self, error));
        }
        // SAFETY: caller owns the genuine source nofail transition and complete
        // callback graph; private mode only changes real scheduler eligibility.
        unsafe { self.publish_mode(entered, callback, context, Some(rank)) }
            .map(|published| DormantGeneralStack { published })
    }
}
impl<'runtime, const N: usize> DormantGeneralStack<'runtime, N> {
    /// Assignment only: neither runnable child nor original C publication.
    pub fn assigned(&self) -> AssignedGeneral {
        self.published.assigned
    }

    /// Activate the already-reserved job from original wake_up_new_task. No
    /// worker/job/order capacity is allocated here. Scope/identity failure
    /// retains the exact dormant graph; it cannot authorize original cleanup.
    ///
    /// # Safety
    /// The actual original child resource/PID/tasklist publication completed
    /// for this exact job and original source reached its first runnable wake.
    /// Keep all task/C/stack/code/data roots independently through final joins.
    pub unsafe fn activate(
        self,
        entered: &dyn EnteredProcess,
    ) -> Result<PublishedGeneralStack<'runtime, N>, (Self, WorkError)> {
        if let Err(error) = self.activate_inner(entered) {
            return Err((self, error));
        }
        Ok(self.published)
    }
    fn activate_inner(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        let p = &self.published;
        entered.check()?;
        p.queue.process()?;
        let flags = p.queue.cpu.save_disable()?;
        let result = (|| {
            entered.check()?;
            let mut guard = p.queue.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned || p.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            // SAFETY: exact affine source job, actual CPU/IF0 and private nonce;
            // all constructor-owned task/pin roots stay kept across this leaf.
            let task = unsafe { &mut *p.runtime.task(p.worker) };
            if task.state != TaskState::Dormant
                || task.id != p.assigned.context
                || task.stack_pin_nonce != p.nonce
                || task.stack_epoch != p.epoch
                || task
                    .job
                    .is_none_or(|(_, job)| job.sequence != p.assigned.ticket.sequence)
                || task.signal.sequence.load(Ordering::Acquire) != p.assigned.ticket.sequence
            {
                return Err(WorkError::corrupt());
            }
            state
                .scheduling
                .activate(task.id, p.assigned.ticket.sequence)?;
            // Every fallible check precedes the actual runnable transition.
            task.state = TaskState::Ready;
            Ok(())
        })();
        // SAFETY: enclosing flags after all task/queue borrows ended. An unwind
        // here retains pin/owner and an already-source-published runnable child.
        unsafe { p.queue.cpu.restore(flags) };
        result
    }
}
