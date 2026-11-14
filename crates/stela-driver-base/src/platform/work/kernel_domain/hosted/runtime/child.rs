//! Exact hosted worker, job and scheduler custody across source publication.
use super::*;
use crate::platform::work::{
    Callback, Job, State, kernel_domain::HostedTicket, scheduling::Snapshot,
};

#[derive(Clone, Copy, PartialEq, Eq)]
struct Creator {
    context: u64,
    sequence: u64,
}

impl Creator {
    fn current<const N: usize>(runtime: &HostedRuntime<N>) -> Result<Self, WorkError> {
        let view = runtime.current_context()?;
        Ok(Self {
            context: view.context_id,
            sequence: view.sequence,
        })
    }

    fn check<const N: usize>(self, runtime: &HostedRuntime<N>) -> Result<(), WorkError> {
        if self != Self::current(runtime)? {
            return Err(WorkError::InvalidTicket);
        }
        Ok(())
    }
}

/// One excluded model stack and queue slot from the actual hosted creator.
/// It retains their reservation on abandonment.
#[must_use = "publish or explicitly cancel this reserved worker"]
pub struct HostedChildReservation<'runtime, const N: usize> {
    runtime: Pin<&'runtime HostedRuntime<N>>,
    creator: Creator,
    worker: usize,
    slot: usize,
    nonce: u64,
    rank: Option<(u64, Snapshot)>,
}

/// An accepted, initially dormant hosted job. The exact stack stays
/// excluded through callback return and explicit source release.
#[must_use = "activate the original child and release after its source joins"]
pub struct HostedChild<'runtime, const N: usize> {
    runtime: Pin<&'runtime HostedRuntime<N>>,
    creator: Creator,
    worker: usize,
    nonce: u64,
    ticket: HostedTicket,
}

impl<const N: usize> HostedRuntime<N> {
    /// Retain the constructor-owned model mappings eligible for child use.
    /// This is an immutable inventory, separate from each stack reservation.
    pub fn child_stack_inventory(
        &self,
        required: StackRequirement,
    ) -> Result<Vec<HostedPages>, WorkError> {
        self.dispatcher_successor()?;
        let mut stacks = Vec::new();
        stacks
            .try_reserve_exact(self.count)
            .map_err(|_| WorkError::Full)?;
        // SAFETY: every continuation is stopped on the original dispatcher.
        let tasks = unsafe { &*self.tasks.get() };
        for task in tasks {
            if task.lane == Lane::General && required.accepts(task.stack.geometry) {
                stacks.push(task.stack._pages.clone());
            }
        }
        Ok(stacks)
    }

    /// Exclude one stopped General worker and one job slot. The exact source
    /// stack can then be prepared while the creator continues or suspends.
    pub fn reserve_child_stack(
        self: Pin<&Self>,
        required: StackRequirement,
    ) -> Result<HostedChildReservation<'_, N>, WorkError> {
        let creator = Creator::current(self.get_ref())?;
        let mut guard = self.queue.lock()?;
        let state = guard.state();
        if state.closed || state.poisoned {
            return Err(WorkError::Closed);
        }
        if !state.scheduling.enabled {
            return Err(WorkError::Unavailable);
        }
        let worker = (0..self.count)
            .find(|&index| {
                // SAFETY: the active creator is the sole OS thread accessing
                // the ledger and no callback runs under this observation.
                let task = unsafe { &*self.task(index) };
                task.lane == Lane::General
                    && task.state == TaskState::Idle
                    && task.binding.is_none()
                    && required.accepts(task.stack.geometry)
            })
            .ok_or(WorkError::Full)?;
        let slot = state
            .layout
            .admission(Lane::General, N)?
            .into_iter()
            .flatten()
            .find(|&slot| state.jobs[slot].is_none() && state.reservations[slot] == 0)
            .ok_or(WorkError::Full)?;
        let nonce = state.next_reservation;
        state.next_reservation = nonce.checked_add(1).ok_or(WorkError::Exhausted)?;
        state.reservations[slot] = nonce;
        // SAFETY: this exact idle worker has no running or retained binding.
        let task = unsafe { &mut *self.task(worker) };
        task.state = TaskState::Reserved;
        task.stack_pin_nonce = nonce;
        Ok(HostedChildReservation {
            runtime: self,
            creator,
            worker,
            slot,
            nonce,
            rank: None,
        })
    }

    fn reset_child_frame(&self, worker: usize, task: &mut Task) {
        task.frame = context::Frame::new(self.initial_controls);
        // SAFETY: callers own the stopped excluded stack and have joined its
        // previous frame users before rewriting the bootstrap words.
        unsafe {
            task.frame.start(
                task.stack.bounds.end,
                ptr::from_ref(self).addr(),
                worker,
                entry::<N>,
            );
        }
    }

    /// End the current hosted child's source frame and return physically to
    /// the original dispatcher. The stack remains kept for source retirement.
    ///
    /// # Safety
    /// This exact original child reached its nonreturning source exit.
    /// No Rust resource or borrowed permit on the abandoned frame may require
    /// cleanup, and all original context and binding owners remain retained.
    pub unsafe fn terminate_child(self: Pin<&Self>) -> ! {
        let view = require(self.current_context());
        let index = self.current.get();
        {
            // SAFETY: actual current, checked above. End the binding borrow
            // before committing the return and switching away.
            let task = unsafe { &*self.task(index) };
            if task.stack_pin_nonce == 0 {
                std::process::abort();
            }
            let binding = task
                .binding
                .as_ref()
                .unwrap_or_else(|| std::process::abort());
            // SAFETY: the caller supplies the original child's terminal scope.
            require(unsafe { binding.terminal_prepare(&view) });
        }
        self.finish_current(index, true);
        self.depart(index);
        std::process::abort()
    }

    pub(super) fn observe_child_return(&self, index: usize) {
        {
            // SAFETY: the actual dispatcher has physically returned from
            // the exact selected stack. No mutable task loan spans the hook.
            let task = unsafe { &*self.task(index) };
            if task.state == TaskState::Terminating {
                let binding = task
                    .binding
                    .as_ref()
                    .unwrap_or_else(|| std::process::abort());
                let mut view = self.context_view(task, task.frame.saved_stack_pointer());
                view.sequence = task.completed_sequence;
                // SAFETY: the exact terminal source frame is stopped. The
                // binding retains its original record across this callback.
                require(unsafe { binding.terminal_successor(&view) });
            } else if task.state != TaskState::Retained {
                return;
            }
        }
        // SAFETY: the terminal hook returned without any retained borrow.
        let task = unsafe { &mut *self.task(index) };
        task.returned_sequence = task.completed_sequence;
        task.state = TaskState::Retained;
    }
}

impl<'runtime, const N: usize> HostedChildReservation<'runtime, N> {
    fn check(&self, task: &Task, state: &State<N>) -> Result<(), WorkError> {
        if state.reservations[self.slot] != self.nonce
            || state.jobs[self.slot].is_some()
            || task.state != TaskState::Reserved
            || task.stack_pin_nonce != self.nonce
        {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }

    /// The exact stopped mmap backing. Model frames are local to this
    /// runtime's host execution owner.
    pub fn stack(&self) -> HostedPages {
        // SAFETY: the reservation keeps this immutable mmap owner. The
        // returned clone borrows no task metadata across a possible switch.
        unsafe { (*self.runtime.task(self.worker)).stack._pages.clone() }
    }

    /// Authenticate this reservation's original creator on its live stack.
    pub fn check_creator(&self) -> Result<(), WorkError> {
        self.creator.check(self.runtime.get_ref())
    }

    /// Keep the source rank and an activation order credit before publication.
    pub fn reserve_rank(&mut self, rank: Snapshot) -> Result<(), WorkError> {
        self.check_creator()?;
        let mut guard = self.runtime.queue.lock()?;
        let state = guard.state();
        // SAFETY: the creator owns this stopped worker and no switch occurs.
        let task = unsafe { &*self.runtime.task(self.worker) };
        self.check(task, state)?;
        if state.closed || state.poisoned {
            return Err(WorkError::Closed);
        }
        if let Some((sequence, reserved)) = self.rank {
            if reserved != rank {
                return Err(WorkError::InvalidTicket);
            }
            state.scheduling.reserved_child(task.id, sequence, rank)?;
            return Ok(());
        }
        let sequence = state.next;
        let next = sequence.checked_add(1).ok_or(WorkError::Exhausted)?;
        state.scheduling.reserve_child(task.id, sequence, rank)?;
        state.next = next;
        self.rank = Some((sequence, rank));
        Ok(())
    }

    /// Publish a prepared callback to the exact nonrunnable assignment.
    ///
    /// # Safety
    /// Source stack preparation has ended. Keep callback, context, image and
    /// source resources alive through activation and source release, including
    /// abandonment. This accepts no inline execution or ordinary cancellation.
    pub unsafe fn publish(
        self,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<HostedChild<'runtime, N>, (Self, WorkError)> {
        let result = (|| {
            self.check_creator()?;
            let mut guard = self.runtime.queue.lock()?;
            let state = guard.state();
            // SAFETY: exact stopped stack, exclusively excluded by this owner.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            self.check(task, state)?;
            if state.closed || state.poisoned {
                return Err(WorkError::Closed);
            }
            let (sequence, rank) = self.rank.ok_or(WorkError::Unavailable)?;
            let row = state.scheduling.reserved_child(task.id, sequence, rank)?;
            let job = Job {
                callback,
                context: context.addr(),
                lane: Lane::General,
                sequence,
                running: true,
                requirement: None,
            };
            // Source stack preparation may have cleared the old frame.
            self.runtime.reset_child_frame(self.worker, task);
            state.scheduling.publish_reserved_child(row);
            state.jobs[self.slot] = Some(job);
            state.reservations[self.slot] = 0;
            task.assign((self.slot, job), ());
            task.state = TaskState::Dormant;
            Ok(HostedChild {
                runtime: self.runtime,
                creator: self.creator,
                worker: self.worker,
                nonce: self.nonce,
                ticket: HostedTicket {
                    queue: self.runtime.queue.id,
                    sequence,
                },
            })
        })();
        result.map_err(|error| (self, error))
    }

    /// # Safety
    /// All unpublished source users of the stack have joined. Cancellation
    /// only applies before this exact reservation has accepted a callback.
    pub unsafe fn cancel(self) -> Result<(), (Self, WorkError)> {
        let result = (|| {
            self.check_creator()?;
            let mut guard = self.runtime.queue.lock()?;
            let state = guard.state();
            // SAFETY: the reservation owns this stopped stack exclusively.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            self.check(task, state)?;
            if let Some((sequence, rank)) = self.rank {
                let row = state.scheduling.reserved_child(task.id, sequence, rank)?;
                state.scheduling.cancel_reserved_child(row);
            }
            self.runtime.reset_child_frame(self.worker, task);
            state.reservations[self.slot] = 0;
            task.stack_pin_nonce = 0;
            task.state = TaskState::Idle;
            Ok(())
        })();
        result.map_err(|error| (self, error))
    }
}

impl<const N: usize> HostedChild<'_, N> {
    pub fn ticket(&self) -> HostedTicket {
        self.ticket
    }

    /// # Safety
    /// The original source has published this exact child and reached its
    /// first runnable wake. The caller keeps all published source owners.
    pub unsafe fn activate(&mut self) -> Result<(), WorkError> {
        self.creator.check(self.runtime.get_ref())?;
        let mut guard = self.runtime.queue.lock()?;
        let state = guard.state();
        if state.closed || state.poisoned {
            return Err(WorkError::Closed);
        }
        // SAFETY: the affine child keeps this nonrunnable worker.
        let task = unsafe { &mut *self.runtime.task(self.worker) };
        if task.state != TaskState::Dormant
            || task.stack_pin_nonce != self.nonce
            || task
                .job
                .is_none_or(|(_, job)| job.sequence != self.ticket.sequence)
        {
            return Err(WorkError::InvalidTicket);
        }
        state.scheduling.activate(task.id, self.ticket.sequence)?;
        task.state = TaskState::Ready;
        Ok(())
    }

    /// Observe the exact child on its physical dispatcher successor.
    pub fn returned(&self) -> Result<bool, WorkError> {
        self.runtime.dispatcher_successor()?;
        self.returned_stopped()
    }

    /// Observe a physical return from another live process on the same runtime.
    pub fn returned_to_peer(&self) -> Result<bool, WorkError> {
        self.runtime.current_context()?;
        if self.worker == self.runtime.current.get() {
            return Err(WorkError::InvalidTicket);
        }
        self.returned_stopped()
    }

    fn returned_stopped(&self) -> Result<bool, WorkError> {
        // SAFETY: the dispatcher or a different live task owns this runtime.
        let task = unsafe { &*self.runtime.task(self.worker) };
        if task.stack_pin_nonce != self.nonce {
            return Err(WorkError::InvalidTicket);
        }
        Ok(task.state == TaskState::Retained
            && task.completed_sequence == self.ticket.sequence
            && task.returned_sequence == self.ticket.sequence)
    }

    /// # Safety
    /// The child has physically returned and all original C stack users
    /// and the exact switch binding have independently joined.
    pub unsafe fn release(self) -> Result<(), (Self, WorkError)> {
        let result = (|| {
            if !self.returned()? {
                return Err(WorkError::Busy);
            }
            // SAFETY: actual dispatcher and the exact retained worker after
            // physical return. No callback or external state is read here.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            if task.binding.is_some() {
                return Err(WorkError::Busy);
            }
            self.runtime.reset_child_frame(self.worker, task);
            task.stack_pin_nonce = 0;
            task.state = TaskState::Idle;
            Ok(())
        })();
        result.map_err(|error| (self, error))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::work::{
        Kind, LeaveReason, kernel_domain::HostedExecution, scheduling::BasePolicy,
    };
    use core::cell::RefCell;

    struct Call<'a> {
        runtime: Pin<&'a HostedRuntime<5>>,
        required: StackRequirement,
        child: RefCell<Option<HostedChild<'a, 5>>>,
        stack_base: Cell<usize>,
        visits: Cell<usize>,
        wake: RefCell<Option<HostedWake>>,
    }

    struct TerminalBinding(Cell<u8>);

    // SAFETY: this fixture owns one real hosted scope. Its hooks record the
    // terminal transition and verify it at the actual detach.
    unsafe impl HostedSwitchBinding for TerminalBinding {
        unsafe fn attach(&self, _: &TaskContext) -> Result<(), WorkError> {
            Ok(())
        }
        unsafe fn still_blocked(&self, _: &TaskContext) -> Result<bool, WorkError> {
            Err(WorkError::Unavailable)
        }
        unsafe fn leave(&self, _: &TaskContext, _: LeaveReason) -> Result<(), WorkError> {
            Err(WorkError::Unavailable)
        }
        unsafe fn arrive(&self, _: &TaskContext, _: bool) -> Result<(), WorkError> {
            Err(WorkError::Unavailable)
        }
        unsafe fn complete(&self, _: &TaskContext) -> Result<(), WorkError> {
            Err(WorkError::Unavailable)
        }
        unsafe fn terminal_prepare(&self, _: &TaskContext) -> Result<(), WorkError> {
            assert_eq!(self.0.replace(1), 0);
            Ok(())
        }
        unsafe fn terminal_successor(&self, view: &TaskContext) -> Result<(), WorkError> {
            assert_ne!(view.sequence, 0);
            assert!((view.stack_start..view.stack_end).contains(&view.stack_pointer));
            assert!(!(view.stack_start..view.stack_end).contains(&context::stack_pointer()));
            assert_eq!(self.0.replace(2), 1);
            Ok(())
        }
        unsafe fn detach(&self, _: &TaskContext) -> Result<(), WorkError> {
            assert_eq!(self.0.get(), 2);
            Ok(())
        }
    }

    unsafe extern "C" fn child_body(raw: *mut c_void) {
        // SAFETY: the fixture keeps this context through source release.
        let call = unsafe { &*raw.cast::<Call<'_>>() };
        let view = call.runtime.current_context().unwrap();
        assert_eq!(view.stack_start, call.stack_base.get());
        assert!(view.stack_pointer < view.stack_end);
        call.visits.set(call.visits.get() + 1);
        *call.wake.borrow_mut() = Some(
            call.runtime
                .bind_current(Rc::new(TerminalBinding(Cell::new(0))))
                .unwrap(),
        );
        // SAFETY: this exact child has no pending destructors or scope loan
        // on the frame it leaves behind.
        unsafe { call.runtime.terminate_child() }
    }

    unsafe extern "C" fn creator(raw: *mut c_void) {
        // SAFETY: the fixture keeps this actual creator and child context.
        let call = unsafe { &*raw.cast::<Call<'_>>() };
        let runtime = call.runtime;
        let base = BasePolicy::Fifo(10);
        let rank = Snapshot {
            base,
            effective: base.priority().unwrap(),
        };
        let mut canceled = runtime.reserve_child_stack(call.required).unwrap();
        let original = canceled.stack();
        canceled.reserve_rank(rank).unwrap();
        assert!(matches!(
            runtime.reserve_child_stack(call.required),
            Err(WorkError::Full)
        ));
        // SAFETY: this test has no remaining unpublished source users.
        unsafe { canceled.cancel() }.unwrap_or_else(|(_, error)| panic!("{error:?}"));

        let mut reserved = runtime.reserve_child_stack(call.required).unwrap();
        let stack = reserved.stack();
        assert!(stack.same_allocation(&original));
        let address = stack.cpu_address(0, stack.bytes()).unwrap();
        call.stack_base.set(address.as_ptr().addr());
        // SAFETY: this exact stopped and reserved stack has no prior users.
        // Overwrite the old entry frame as an original stack initializer can.
        unsafe { ptr::write_bytes(address.as_ptr(), 0, stack.bytes()) };
        reserved.reserve_rank(rank).unwrap();
        // SAFETY: exact source preparation ended and the fixture stays kept.
        let child = unsafe { reserved.publish(child_body, raw) }
            .unwrap_or_else(|(_, error)| panic!("{error:?}"));
        assert_eq!(child.returned_to_peer(), Ok(false));
        *call.child.borrow_mut() = Some(child);
        runtime.yield_now().unwrap();
        assert_eq!(call.visits.get(), 0);
        // SAFETY: this bounded model completes its source publication here
        // on the exact creator and retains the callback through its return.
        unsafe { call.child.borrow_mut().as_mut().unwrap().activate() }.unwrap();
    }

    unsafe extern "C" fn observe_return(raw: *mut c_void) {
        // SAFETY: the fixture keeps the same context while this peer runs.
        let call = unsafe { &*raw.cast::<Call<'_>>() };
        let child = call.child.borrow();
        assert_eq!(child.as_ref().unwrap().returned_to_peer(), Ok(true));
    }

    #[test]
    fn reserved_child_stack_is_dormant_until_activation_and_kept_until_source_release() {
        let queue = HostedQueue::new(HostedExecution::new(), Arc::new(())).unwrap();
        let required = StackRequirement::exact(16 * 1024, 16 * 1024, PAGE).unwrap();
        // SAFETY: all fixture callbacks and their actual stacks stay retained
        // until the original dispatcher observes completion and releases them.
        let executor =
            unsafe { HostedExecutor::new_with_stacks(queue.clone(), required, 2) }.unwrap();
        let runtime = executor.runtime();
        runtime.enable_ranked_scheduling().unwrap();
        let call = Call {
            runtime,
            required,
            child: RefCell::new(None),
            stack_base: Cell::new(0),
            visits: Cell::new(0),
            wake: RefCell::new(None),
        };
        // SAFETY: this context stays live through both actual returns.
        let creator = unsafe {
            queue.execute(
                Kind::General,
                creator,
                ptr::from_ref(&call).cast_mut().cast(),
            )
        }
        .unwrap();
        assert_eq!(runtime.step(), Ok(true));
        let ticket = call.child.borrow().as_ref().unwrap().ticket();
        assert_eq!(queue.completed(ticket), Ok(false));
        assert_eq!(call.child.borrow().as_ref().unwrap().returned(), Ok(false));
        assert_eq!(runtime.step(), Ok(true));
        assert_eq!(queue.completed(creator), Ok(true));
        assert_eq!(call.visits.get(), 0);
        assert_eq!(runtime.step(), Ok(true));
        assert_eq!(call.visits.get(), 1);
        assert_eq!(queue.completed(ticket), Ok(true));
        // SAFETY: the same context survives this separate hosted assignment.
        let peer = unsafe {
            queue.execute(
                Kind::General,
                observe_return,
                ptr::from_ref(&call).cast_mut().cast(),
            )
        }
        .unwrap();
        assert_eq!(runtime.step(), Ok(true));
        assert_eq!(queue.completed(peer), Ok(true));
        let child = call.child.take().unwrap();
        assert_eq!(child.returned(), Ok(true));
        queue.close().unwrap();
        assert_eq!(runtime.retire(), Err(WorkError::Busy));
        let wake = call.wake.take().unwrap();
        assert_eq!(wake.wake(), Err(WorkError::Closed));
        // SAFETY: the source body physically returned and no user of its
        // binding or stack is still active.
        drop(unsafe { runtime.detach_binding(&wake) }.unwrap());
        assert_eq!(runtime.retire(), Err(WorkError::Busy));
        // SAFETY: both callbacks and every source user joined on this dispatcher.
        unsafe { child.release() }.unwrap_or_else(|(_, error)| panic!("{error:?}"));
        runtime.retire().unwrap();
        // SAFETY: both callback owners returned and all producers stopped.
        unsafe { queue.retire() }.unwrap();
    }
}
