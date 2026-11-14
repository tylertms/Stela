//! Affine worker/job reservation under a borrowed entered process guard.
use super::*;
mod owned;
mod refusal;
pub use owned::{
    CancelledReservation, DormantGeneralStack, GeneralStackPreparation, OwnedGeneralEscrow,
    OwnedPinnedGeneral, OwnedTransferFailure, PinnedGeneralStackPreparation, PublishedGeneralStack,
    StackPinError,
};
pub use refusal::{GeneralBlocker, GeneralRefusal};
/// Borrowed platform process-entry permission, independent of task IDs.
///
/// # Safety
/// The view borrows an entered process guard for its entire lifetime.
/// check validates CPU/root/process generation before mutable service
/// access; neither RootIdentity nor a caller integer substitutes for that guard.
/// It is nonblocking/nonallocating and valid with local IRQs saved/disabled:
/// no AML, queue callback, allocator, RestoreRoot reacquisition or wait. There
/// is no permissive default. A borrowed permission must not escape across Guest.
pub unsafe trait EnteredProcess {
    fn check(&self) -> Result<(), WorkError>;
}
#[derive(Clone, Copy)]
struct Creator {
    queue: u64,
    context: u64,
    sequence: u64,
}
impl Creator {
    fn of(view: TaskContext) -> Self {
        Self {
            queue: view.queue_id,
            context: view.context_id,
            sequence: view.sequence,
        }
    }
    fn matches(self, view: TaskContext) -> bool {
        self.queue == view.queue_id
            && self.context == view.context_id
            && self.sequence == view.sequence
    }
}
/// Receipt for assigned deferred work, not child startup or C current.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AssignedGeneral {
    ticket: Ticket,
    context: ContextId,
}
impl AssignedGeneral {
    pub fn ticket(self) -> Ticket {
        self.ticket
    }
    pub fn context_id(self) -> ContextId {
        self.context
    }
}
/// Non-Send, non-Clone short-lived reservation. Drop rolls back only under its
/// matching entered scope; uncertain ownership keeps the whole owner.
#[must_use = "drop/cancel rolls back; commit publishes only actual owned deferred work"]
pub struct GeneralReservation<'runtime, 'entered, const N: usize> {
    runtime: Pin<&'runtime Runtime<N>>,
    queue: Arc<Queue<N>>,
    entered: &'entered dyn EnteredProcess,
    creator: Creator,
    creator_worker: usize,
    worker: usize,
    context: ContextId,
    slot: usize,
    nonce: u64,
    requirement: StackRequirement,
    live: bool,
    _local: PhantomData<*mut ()>,
}
#[derive(Clone, Copy)]
enum Eligibility {
    AnyIdle,
    Unstarted,
}
impl Eligibility {
    fn permits(self, task: &Task) -> bool {
        match self {
            Self::AnyIdle => true,
            Self::Unstarted => !task.ever_entered,
        }
    }
}
impl<const N: usize> Runtime<N> {
    /// Constructor capacity for simultaneous General jobs with this geometry,
    /// bounded by both owned stacks and their permitted admission slots.
    pub fn general_capacity(&self, requirement: StackRequirement) -> Result<usize, WorkError> {
        self.queue.process()?;
        let slots = self
            .layout
            .admission(Lane::General, N)?
            .into_iter()
            .flatten()
            .count();
        // SAFETY: immutable constructor geometry, on the runtime process CPU.
        Ok(unsafe { &*self.tasks.get() }
            .iter()
            .filter(|task| {
                task.lane == Lane::General
                    && stack_requirement::compatible(
                        Some(requirement),
                        stack_requirement::Geometry::of(&task.stack),
                    )
            })
            .count()
            .min(slots))
    }
    /// Assign work to a real idle compatible General continuation immediately.
    /// Kept bindings are reusable only when their exact retained objects appear
    /// in `reusable_bindings`; unrelated foreign continuation state is excluded.
    /// Unlike queue admission, success guarantees a stack even for a task that
    /// may remain parked indefinitely. This works on the process CPU from its
    /// dispatcher or a running task, and never invokes the callback inline.
    ///
    /// # Safety
    /// As for [`Queue::execute`]. The body fits `requirement` and correctly
    /// enters/reuses any supplied binding before accessing its foreign state.
    pub unsafe fn execute_idle_general(
        self: Pin<&Self>,
        requirement: StackRequirement,
        callback: Callback,
        context: *mut c_void,
        reusable_bindings: &[&dyn SwitchBinding],
    ) -> Result<Ticket, GeneralRefusal> {
        // SAFETY: forwards the caller's callback, geometry and binding contract.
        unsafe {
            self.execute_idle_general_with_headroom(
                requirement,
                callback,
                context,
                reusable_bindings,
                0,
            )
        }
    }
    /// As `execute_idle_general`, while leaving at least `minimum_idle`
    /// compatible reusable idle continuations and free admission slots for
    /// other declared producers.
    ///
    /// # Safety
    /// The callback, geometry and binding contract is `execute_idle_general`'s.
    pub unsafe fn execute_idle_general_with_headroom(
        self: Pin<&Self>,
        requirement: StackRequirement,
        callback: Callback,
        context: *mut c_void,
        reusable_bindings: &[&dyn SwitchBinding],
        minimum_idle: usize,
    ) -> Result<Ticket, GeneralRefusal> {
        self.queue.process()?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if self.retired.get()
                || state.closed
                || state.poisoned
                || self.queue.abandoned.load(Ordering::Acquire)
            {
                return Err(WorkError::Closed.into());
            }
            let mut free_slots = state
                .layout
                .admission(Lane::General, N)?
                .into_iter()
                .flatten()
                .filter(|slot| state.jobs[*slot].is_none() && state.reservations[*slot] == 0);
            let slot = free_slots.next().ok_or_else(GeneralRefusal::queue_credit)?;
            if free_slots.count() < minimum_idle {
                return Err(GeneralRefusal::queue_credit());
            }
            // SAFETY: process CPU at IF0; every other continuation is stopped.
            let count = unsafe { (*self.tasks.get()).len() };
            let mut idle = (0..count).filter(|index| {
                // SAFETY: immutable constructor-owned entry, bounded above.
                let task = unsafe { &*self.task(*index) };
                task.lane == Lane::General
                    && task.state == TaskState::Idle
                    && task.job.is_none()
                    && !task.draining
                    && task.signal.sequence.load(Ordering::Acquire) == 0
                    && task.binding.as_ref().is_none_or(|binding| {
                        reusable_bindings
                            .iter()
                            .any(|allowed| core::ptr::addr_eq(Arc::as_ref(binding), *allowed))
                    })
                    && stack_requirement::compatible(
                        Some(requirement),
                        stack_requirement::Geometry::of(&task.stack),
                    )
            });
            let worker = idle.next().ok_or_else(|| {
                // Immutable actual constructor geometry distinguishes permanent
                // placement refusal from temporarily unavailable reusable stacks.
                let supported = (0..count).any(|index| {
                    // SAFETY: same immutable bounded constructor-owned entry.
                    let task = unsafe { &*self.task(index) };
                    task.lane == Lane::General
                        && stack_requirement::compatible(
                            Some(requirement),
                            stack_requirement::Geometry::of(&task.stack),
                        )
                });
                if supported {
                    GeneralRefusal::compatible_stack()
                } else {
                    GeneralRefusal::placement(WorkError::Full)
                }
            })?;
            if idle.count() < minimum_idle {
                return Err(GeneralRefusal::compatible_stack());
            }
            let sequence = state.next;
            let next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
            // SAFETY: exact idle entry selected above while all frames are stopped.
            let task = unsafe { &mut *self.task(worker) };
            state.scheduling.insert(task.id, sequence, false)?;
            let job = Job {
                callback,
                context: context as usize,
                lane: Lane::General,
                sequence,
                running: true,
                requirement: Some(requirement),
            };
            state.next = next;
            state.jobs[slot] = Some(job);
            task.job = Some((slot, job));
            task.flags = flags;
            task.state = TaskState::Ready;
            task.signal.pending.store(false, Ordering::Relaxed);
            task.signal.sequence.store(sequence, Ordering::Release);
            Ok(Ticket {
                queue: self.queue.id,
                sequence,
            })
        })();
        // SAFETY: all borrows end before restoring exactly the incoming flags.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
    /// Reserve one idle, unbound compatible General worker and one
    /// ordinary job slot. This publishes no callback, ticket, C task or current.
    /// All mutations follow real scope/current checks. Incoming IF is restored
    /// before returning for bounded nonblocking metadata setup; no permit parks.
    pub fn reserve_general<'runtime, 'entered>(
        self: Pin<&'runtime Self>,
        entered: &'entered dyn EnteredProcess,
        requirement: StackRequirement,
    ) -> Result<GeneralReservation<'runtime, 'entered, N>, GeneralRefusal> {
        self.reserve_eligible(entered, requirement, Eligibility::AnyIdle)
    }
    /// Cold-only admission. A released previously-entered stack remains
    /// ineligible. Canceling an unpublished never-entered reservation may
    /// restore this separate eligibility without changing entry history.
    pub fn reserve_unstarted_general<'runtime, 'entered>(
        self: Pin<&'runtime Self>,
        entered: &'entered dyn EnteredProcess,
        requirement: StackRequirement,
    ) -> Result<GeneralReservation<'runtime, 'entered, N>, GeneralRefusal> {
        self.reserve_eligible(entered, requirement, Eligibility::Unstarted)
    }
    fn reserve_eligible<'runtime, 'entered>(
        self: Pin<&'runtime Self>,
        entered: &'entered dyn EnteredProcess,
        requirement: StackRequirement,
        eligibility: Eligibility,
    ) -> Result<GeneralReservation<'runtime, 'entered, N>, GeneralRefusal> {
        entered.check()?;
        self.current_context()?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            entered.check()?;
            let creator = Creator::of(self.current_context()?);
            let creator_worker = self.current.get();
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned || self.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed.into());
            }
            if state.stack_profile.is_none()
                || !state.stack_catalog.supports(Lane::General, requirement)
            {
                return Err(GeneralRefusal::placement(WorkError::Unavailable));
            }
            let slot = state
                .layout
                .admission(Lane::General, N)?
                .into_iter()
                .flatten()
                .find(|slot| state.jobs[*slot].is_none() && state.reservations[*slot] == 0)
                .ok_or_else(GeneralRefusal::queue_credit)?;
            // SAFETY: current CPU and IF0 own this immutable Vec;
            // no foreign hook/callback, allocation or switch spans these reads.
            let count = unsafe { (*self.tasks.get()).len() };
            let worker = (0..count)
                .find(|index| {
                    // SAFETY: bounded task entry, stopped while creator runs at IF0.
                    let task = unsafe { &*self.task(*index) };
                    task.lane == Lane::General
                        && task.state == TaskState::Idle
                        && eligibility.permits(task)
                        && task.job.is_none()
                        && task.binding.is_none()
                        && !task.draining
                        && task.signal.sequence.load(Ordering::Acquire) == 0
                        && stack_requirement::compatible(
                            Some(requirement),
                            stack_requirement::Geometry::of(&task.stack),
                        )
                })
                .ok_or_else(|| match eligibility {
                    Eligibility::AnyIdle => GeneralRefusal::compatible_stack(),
                    Eligibility::Unstarted => GeneralRefusal::unstarted_stack(),
                })?;
            let nonce = state.next_reservation;
            if nonce == 0 {
                return Err(WorkError::corrupt().into());
            }
            let next = nonce.checked_add(1).ok_or(WorkError::Exhausted)?;
            let pending = self
                .pending_reservations
                .get()
                .checked_add(1)
                .ok_or(WorkError::Exhausted)?;
            // SAFETY: exact idle unbound worker selected above, exclusively stopped at IF0.
            let task = unsafe { &mut *self.task(worker) };
            state.next_reservation = next;
            state.reservations[slot] = nonce;
            task.state = TaskState::Reserved;
            self.pending_reservations.set(pending);
            Ok(GeneralReservation {
                runtime: self,
                queue: self.queue.clone(),
                entered,
                creator,
                creator_worker,
                worker,
                context: task.id,
                slot,
                nonce,
                requirement,
                live: true,
                _local: PhantomData,
            })
        })();
        // SAFETY: enclosing save_disable flags; all queue/task borrows ended.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
}
impl<const N: usize> GeneralReservation<'_, '_, N> {
    fn current(&self) -> Result<(), WorkError> {
        self.entered.check()?;
        if !self.creator.matches(self.runtime.current_context()?)
            || self.runtime.current.get() != self.creator_worker
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    /// # Safety
    /// The provider already keeps callback/code/context and every reachable
    /// resource through completion or quarantine, as for Queue::execute. The
    /// body fits the requested extent and must not unwind. Metadata
    /// setup was nonblocking; no borrowed process permission enters the job.
    /// Error admits no callback and grants no wrong-scope cleanup permission.
    pub unsafe fn commit(
        mut self,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<AssignedGeneral, WorkError> {
        self.current()?;
        let flags = self.runtime.queue.cpu.save_disable()?;
        let result = (|| {
            self.current()?;
            let mut guard = self.runtime.queue.lock()?;
            let state = guard.state();
            if state.closed
                || state.poisoned
                || self.runtime.queue.abandoned.load(Ordering::Acquire)
            {
                return Err(WorkError::Closed);
            }
            self.validate(state)?;
            let sequence = state.next;
            let next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
            state.scheduling.insert(self.context, sequence, false)?;
            let job = Job {
                callback,
                context: context as usize,
                lane: Lane::General,
                sequence,
                running: true,
                requirement: Some(self.requirement),
            };
            // No fallible operation follows this ownership publication.
            state.next = next;
            state.jobs[self.slot] = Some(job);
            state.reservations[self.slot] = 0;
            // SAFETY: exact matching private reservation was checked at IF0 above;
            // no alias/hook/switch crosses the assignment of this owned child frame.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            task.job = Some((self.slot, job));
            task.flags = flags;
            task.state = TaskState::Ready;
            task.signal.pending.store(false, Ordering::Relaxed);
            task.signal.sequence.store(sequence, Ordering::Release);
            self.runtime
                .pending_reservations
                .set(self.runtime.pending_reservations.get() - 1);
            self.live = false;
            Ok(AssignedGeneral {
                ticket: Ticket {
                    queue: self.runtime.queue.id,
                    sequence,
                },
                context: self.context,
            })
        })();
        // SAFETY: all queue/task borrows ended; restore exact commit entry flags.
        unsafe { self.runtime.queue.cpu.restore(flags) };
        result
    }
    fn validate(&self, state: &State<N>) -> Result<(), WorkError> {
        if self.slot >= N
            || state.reservations[self.slot] != self.nonce
            || state.jobs[self.slot].is_some()
            || self.runtime.pending_reservations.get() == 0
        {
            return Err(WorkError::corrupt());
        }
        // SAFETY: caller checked real current permission at IF0 and kept the
        // unchanged task vector; worker is private constructor-captured.
        let task = unsafe { &*self.runtime.task(self.worker) };
        if task.id != self.context
            || task.lane != Lane::General
            || task.state != TaskState::Reserved
            || task.job.is_some()
            || task.binding.is_some()
            || task.draining
            || task.signal.sequence.load(Ordering::Acquire) != 0
            || !stack_requirement::compatible(
                Some(self.requirement),
                stack_requirement::Geometry::of(&task.stack),
            )
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    fn rollback(&mut self) -> Result<(), WorkError> {
        self.current()?;
        let flags = self.runtime.queue.cpu.save_disable()?;
        let result = (|| {
            self.current()?;
            let mut guard = self.runtime.queue.lock()?;
            let state = guard.state();
            self.validate(state)?;
            // SAFETY: validate checked this same unpublished reserved worker,
            // no binding/C pin/job owns it; original geometry/history are kept.
            let shape = unsafe {
                let task = &*self.runtime.task(self.worker);
                ReleasedStackGeometry::of(&task.stack, !task.ever_entered)
            };
            state.resources.reservation_released(shape)?;
            state.reservations[self.slot] = 0;
            // SAFETY: reserved worker ownership validated above. No callback
            // or task metadata was published, and no binding/producer owns it.
            unsafe { (*self.runtime.task(self.worker)).state = TaskState::Idle };
            self.runtime
                .pending_reservations
                .set(self.runtime.pending_reservations.get() - 1);
            self.live = false;
            Ok(())
        })();
        // SAFETY: no metadata borrow survives restoration of exact entry flags.
        unsafe { self.runtime.queue.cpu.restore(flags) };
        result
    }
    fn abandon(&mut self) {
        // No affine Runtime/Task/queue state is read or freed under a stale
        // permission. The atomic view closes admission; reserved state and every
        // real owner remain held, including this whole pinned executor.
        self.queue.abandoned.store(true, Ordering::Release);
        quarantine();
        self.live = false;
    }
    pub fn cancel(mut self) -> Result<(), WorkError> {
        let result = self.rollback();
        if result.is_err() {
            self.abandon();
        }
        result
    }
}
impl<const N: usize> Drop for GeneralReservation<'_, '_, N> {
    fn drop(&mut self) {
        if self.live && self.rollback().is_err() {
            self.abandon();
        }
    }
}
