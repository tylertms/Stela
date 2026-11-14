//! Hosted continuation hooks use actual host stack/current observations.
use super::*;
use crate::platform::work::{
    ContextId, InactiveObservation, LeaveReason, ParkOutcome, WakeSignal, scheduling::Snapshot,
};

/// A retained foreign context attached to one hosted stack.
///
/// # Safety
/// Keep all reachable source/code/storage owners through quiescent detach.
/// Hooks are bounded, cannot unwind, allocate, dispatch, suspend or release
/// owners. They may check current metadata and publish a same-thread wake. Attach, leave and detach refusals are transactional.
/// Leave commits only a source-prepared suspension. Arrive restores foreign
/// current on the resumed stack. Complete requires the source body to have
/// exited already. Context views are borrowed for one call.
pub unsafe trait HostedSwitchBinding {
    /// # Safety
    /// Actual current stack, checked original host thread, no queue guard.
    unsafe fn attach(&self, context: &TaskContext) -> Result<(), WorkError>;
    /// # Safety
    /// Same stack after original prepare. Observe the condition only.
    unsafe fn still_blocked(&self, context: &TaskContext) -> Result<bool, WorkError>;
    /// # Safety
    /// Same stack after original prepare, with all source owners retained.
    unsafe fn leave(&self, context: &TaskContext, reason: LeaveReason) -> Result<(), WorkError>;
    /// # Safety
    /// Actual resumed stack before foreign code continues.
    unsafe fn arrive(&self, context: &TaskContext, new_job: bool) -> Result<(), WorkError>;
    /// # Safety
    /// Actual finishing stack after the foreign scope exited.
    unsafe fn complete(&self, context: &TaskContext) -> Result<(), WorkError>;
    /// # Safety
    /// Exact owned child stack at the original nonreturning source exit.
    /// Consume its live scope without abandoning any independently owned loan.
    unsafe fn terminal_prepare(&self, _context: &TaskContext) -> Result<(), WorkError> {
        Err(WorkError::Unavailable)
    }
    /// # Safety
    /// The original dispatcher has physically returned from the exact
    /// terminal child. Record only this bounded source transition.
    unsafe fn terminal_successor(&self, _context: &TaskContext) -> Result<(), WorkError> {
        Err(WorkError::Unavailable)
    }
    /// # Safety
    /// Actual taskless dispatcher, with this stack idle and source users joined.
    unsafe fn detach(&self, context: &TaskContext) -> Result<(), WorkError>;
}

/// A wake for one accepted hosted job. Foreign threads publish through the
/// hosted event transport. This handle checks the actual owning thread.
#[derive(Clone)]
pub struct HostedWake {
    execution: Arc<super::super::HostedExecution>,
    signal: Arc<WakeSignal>,
    queue: u64,
    task: ContextId,
    sequence: u64,
}

impl HostedWake {
    pub fn sequence(&self) -> u64 {
        self.sequence
    }

    pub fn wake(&self) -> Result<(), WorkError> {
        self.execution.check_current()?;
        if self.signal.sequence.load(Ordering::Acquire) != self.sequence {
            return Err(WorkError::Closed);
        }
        self.signal.pending.store(true, Ordering::Release);
        Ok(())
    }
}

#[derive(Clone, Copy)]
pub(super) struct InactiveWait {
    target: usize,
    target_sequence: u64,
    waiter_sequence: u64,
    notified: bool,
}

impl<const N: usize> HostedRuntime<N> {
    fn inactive_target(&self, target: &HostedWake) -> Result<usize, WorkError> {
        if self.queue.id != target.queue {
            return Err(WorkError::InvalidTicket);
        }
        (0..self.count)
            .find(|&index| {
                // SAFETY: the checked current continuation owns all runtime metadata.
                Arc::ptr_eq(&unsafe { &*self.task(index) }.signal, &target.signal)
            })
            .ok_or(WorkError::InvalidTicket)
    }

    /// Observe a different assignment on this cooperative runtime, or arm
    /// the current assignment for its next physical return to the dispatcher.
    /// A stopped task with a pending wake cannot satisfy the observation.
    pub fn observe_inactive(
        &self,
        target: &HostedWake,
        force_wait: bool,
    ) -> Result<InactiveObservation, WorkError> {
        let waiter = self.current_context()?;
        let index = self.current.get();
        let target_index = self.inactive_target(target)?;
        if target_index == index {
            return Err(WorkError::InvalidTicket);
        }
        self.check_target(target)?;
        let slot = &self.inactive[index];
        if slot.get().is_some() {
            return Err(WorkError::Busy);
        }
        // SAFETY: same checked host thread. The other stack can only move
        // after this continuation physically returns to the dispatcher.
        let task = unsafe { &*self.task(target_index) };
        if !force_wait
            && task.state == TaskState::Blocked
            && !target.signal.pending.load(Ordering::Acquire)
            && !self.inactive_ready(target_index, target.sequence)
        {
            return Ok(InactiveObservation::Inactive);
        }
        slot.set(Some(InactiveWait {
            target: target_index,
            target_sequence: target.sequence,
            waiter_sequence: waiter.sequence,
            notified: false,
        }));
        Ok(InactiveObservation::Subscribed)
    }

    /// Close only the current assignment's exact inactive wait. A completed
    /// target may have cleared its live sequence after notifying this wait.
    pub fn cancel_inactive(&self, target: &HostedWake) -> Result<(), WorkError> {
        let waiter = self.current_context()?;
        let index = self.current.get();
        let target_index = self.inactive_target(target)?;
        let slot = &self.inactive[index];
        let wait = slot.get().ok_or(WorkError::InvalidTicket)?;
        if wait.waiter_sequence != waiter.sequence
            || wait.target != target_index
            || wait.target_sequence != target.sequence
        {
            return Err(WorkError::InvalidTicket);
        }
        slot.set(None);
        Ok(())
    }

    pub(super) fn inactive_ready(&self, index: usize, sequence: u64) -> bool {
        self.inactive[index]
            .get()
            .is_some_and(|wait| wait.waiter_sequence == sequence && wait.notified)
    }

    pub(super) fn notify_inactive(&self, target: usize, sequence: u64) {
        for slot in &self.inactive {
            if let Some(mut wait) = slot.get()
                && wait.target == target
                && wait.target_sequence == sequence
            {
                wait.notified = true;
                slot.set(Some(wait));
            }
        }
    }

    pub fn bind_current(
        self: Pin<&Self>,
        binding: Rc<dyn HostedSwitchBinding>,
    ) -> Result<HostedWake, WorkError> {
        let context = self.current_context()?;
        let index = self.current.get();
        // SAFETY: checked current index on the original thread.
        if unsafe { (*self.task(index)).binding.is_some() } {
            return Err(WorkError::Busy);
        }
        // SAFETY: this exact live stack is executing with no queue guard.
        unsafe { binding.attach(&context)? };
        // SAFETY: the successful hook returned without a switch or a borrow.
        let task = unsafe { &mut *self.task(index) };
        task.binding = Some(binding);
        self.current_wake()
    }

    pub fn current_wake(&self) -> Result<HostedWake, WorkError> {
        let context = self.current_context()?;
        // SAFETY: current_context verified this exact executing stack.
        let task = unsafe { &*self.task(self.current.get()) };
        Ok(HostedWake {
            execution: Arc::clone(&self.queue.execution),
            signal: task.signal.clone(),
            queue: self.queue.id,
            task: task.id,
            sequence: context.sequence,
        })
    }

    /// Commit an original source rank change on this runtime's active caller.
    ///
    /// # Safety
    /// The source holds the target task and its serialized priority mutation,
    /// and commits the matching original fields before releasing that lease.
    pub unsafe fn update_rank(
        &self,
        target: &HostedWake,
        before: Snapshot,
        after: Snapshot,
        head: bool,
    ) -> Result<(), WorkError> {
        self.current_context()?;
        self.check_target(target)?;
        let mut guard = self.queue.lock()?;
        let board = &mut guard.state().scheduling;
        if !board.enabled {
            return Err(WorkError::Unavailable);
        }
        let now = self.thread_cpu_ns()?;
        board.update(target.task, target.sequence, before, after, head, Some(now))
    }

    /// Observe time charged to this exact live assignment. An inactive
    /// assignment keeps its last charge until it runs again. Completion
    /// closes the handle even if its physical stack is reused later.
    pub fn execution_ns(&self, target: &HostedWake) -> Result<u64, WorkError> {
        self.check()?;
        if DISPATCH.get().active.is_null() {
            self.dispatcher_successor()?;
        } else {
            self.current_context()?;
        }
        self.check_target(target)?;
        let mut guard = self.queue.lock()?;
        let board = &mut guard.state().scheduling;
        if !board.enabled {
            return Err(WorkError::Unavailable);
        }
        let (total, _, _) = board.execution(target.task, target.sequence, self.thread_cpu_ns()?)?;
        Ok(total)
    }

    /// Commit the actual current assignment's CPU-time charge. An outgoing
    /// source callback can publish this sample before the scheduler parks it.
    pub fn account_current_execution(&self) -> Result<u64, WorkError> {
        let view = self.current_context()?;
        let mut guard = self.queue.lock()?;
        let board = &mut guard.state().scheduling;
        if !board.enabled {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: current_context authenticated this live stack. The CPU-time
        // query and board update cannot switch or invoke foreign callbacks.
        let task = unsafe { &*self.task(self.current.get()) };
        let now = self.thread_cpu_ns()?;
        board.account(now)?;
        let (total, _, _) = board.execution(task.id, view.sequence, now)?;
        Ok(total)
    }

    fn check_target(&self, target: &HostedWake) -> Result<(), WorkError> {
        if self.queue.id != target.queue {
            return Err(WorkError::InvalidTicket);
        }
        if target.signal.sequence.load(Ordering::Acquire) != target.sequence {
            return Err(WorkError::Closed);
        }
        Ok(())
    }

    /// # Safety
    /// Actual foreign task/IRQ/wait references have joined. The handle names
    /// the latest completed assignment of the exact stack to detach, and
    /// its binding has not been replaced.
    pub unsafe fn detach_binding(
        self: Pin<&Self>,
        wake: &HostedWake,
    ) -> Result<Rc<dyn HostedSwitchBinding>, WorkError> {
        self.dispatcher_successor()?;
        for index in 0..self.count {
            // SAFETY: actual original dispatcher, all continuations stopped.
            let task = unsafe { &*self.task(index) };
            if !Arc::ptr_eq(&task.signal, &wake.signal) {
                continue;
            }
            if !matches!(task.state, TaskState::Idle | TaskState::Retained) {
                return Err(WorkError::Busy);
            }
            if task.completed_sequence != wake.sequence {
                return Err(WorkError::InvalidTicket);
            }
            let binding = task.binding.as_ref().ok_or(WorkError::Unavailable)?;
            let view = self.context_view(task, task.frame.saved_stack_pointer());
            // SAFETY: actual stopped stack and caller-supplied source joins.
            unsafe { binding.detach(&view)? };
            // SAFETY: all shared observations and the hook borrow have ended.
            return unsafe { (*self.task(index)).binding.take() }
                .ok_or_else(|| WorkError::corrupt());
        }
        Err(WorkError::InvalidTicket)
    }

    pub fn yield_now(self: Pin<&Self>) -> Result<(), WorkError> {
        self.suspend(LeaveReason::Yield).map(|_| ())
    }

    pub fn reschedule(self: Pin<&Self>) -> Result<(), WorkError> {
        self.suspend(LeaveReason::Reschedule).map(|_| ())
    }

    pub fn service_current(&self) -> Result<bool, WorkError> {
        self.current_context()?;
        // SAFETY: the checked current task remains live on this stack.
        Ok(unsafe { (*self.task(self.current.get())).lane == Lane::Service })
    }

    /// Park after preparing the original source wait. A concurrent cooperative
    /// wake or a cleared predicate leaves the current source scope intact.
    pub fn park_if_blocked(self: Pin<&Self>) -> Result<ParkOutcome, WorkError> {
        self.suspend(LeaveReason::Blocked)
    }

    fn suspend(self: Pin<&Self>, reason: LeaveReason) -> Result<ParkOutcome, WorkError> {
        let view = self.current_context()?;
        let index = self.current.get();
        {
            // SAFETY: checked current stack. No task borrow spans the switch.
            let task = unsafe { &*self.task(index) };
            if reason == LeaveReason::Blocked {
                let binding = task.binding.as_ref().ok_or(WorkError::Unavailable)?;
                // SAFETY: source-prepared condition on the same checked stack.
                if !unsafe { binding.still_blocked(&view)? }
                    || self.inactive_ready(index, view.sequence)
                    || task.signal.pending.swap(false, Ordering::AcqRel)
                {
                    return Ok(ParkOutcome::Skipped);
                }
            }
            {
                let mut guard = self.queue.lock()?;
                let board = &mut guard.state().scheduling;
                board.can_suspend(task.id, view.sequence, reason)?;
                if board.enabled {
                    board.account(self.thread_cpu_ns()?)?;
                }
            }
            if let Some(binding) = &task.binding {
                // SAFETY: pure scheduler preflight completed and its guard
                // ended. The exact source-prepared stack is still current.
                unsafe { binding.leave(&view, reason)? };
            }
            let result = self.queue.lock().and_then(|mut guard| {
                guard
                    .state()
                    .scheduling
                    .suspend(task.id, view.sequence, reason)
            });
            require(result);
            // SAFETY: the shared task and binding loans have ended. This is
            // the actual outgoing stack before control returns to dispatcher.
            unsafe {
                (*self.task(index)).state = if reason == LeaveReason::Blocked {
                    TaskState::Blocked
                } else {
                    TaskState::Runnable
                };
            }
        }
        self.depart(index);
        self.arrive(index, false);
        Ok(ParkOutcome::Resumed)
    }

    pub(super) fn arrive(&self, index: usize, new_job: bool) {
        // SAFETY: only this exact task has resumed on the original thread.
        let task = unsafe { &*self.task(index) };
        if let Some(binding) = &task.binding {
            let view = require(self.current_context());
            // SAFETY: actual resumed stack and callback-scoped view.
            require(unsafe { binding.arrive(&view, new_job) });
        }
    }

    pub(super) fn complete_binding(&self, index: usize) {
        // SAFETY: the job returned on its exact current stack.
        let task = unsafe { &*self.task(index) };
        if let Some(binding) = &task.binding {
            let view = require(self.current_context());
            // SAFETY: completion requires the source already exited, verified
            // by the hook. No task or queue transition is committed yet.
            require(unsafe { binding.complete(&view) });
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::platform::work::{Kind, kernel_domain::HostedExecution};
    use core::cell::RefCell;

    #[derive(Clone, Copy, Debug, PartialEq)]
    enum State {
        New,
        Entered,
        Parked,
        Exited,
        Completed,
        Detached,
    }

    struct Binding(Cell<State>);
    impl Binding {
        fn transition(&self, previous: State, next: State) -> Result<(), WorkError> {
            if self.0.get() != previous {
                return Err(WorkError::CorruptContext);
            }
            self.0.set(next);
            Ok(())
        }
    }

    // SAFETY: fixture hooks only observe and commit bounded in-place metadata.
    // The owning test keeps the original binding until quiescent detach.
    unsafe impl HostedSwitchBinding for Binding {
        unsafe fn attach(&self, _: &TaskContext) -> Result<(), WorkError> {
            self.transition(State::New, State::Entered)
        }
        unsafe fn still_blocked(&self, _: &TaskContext) -> Result<bool, WorkError> {
            Ok(self.0.get() == State::Entered)
        }
        unsafe fn leave(&self, _: &TaskContext, reason: LeaveReason) -> Result<(), WorkError> {
            assert_eq!(reason, LeaveReason::Blocked);
            self.transition(State::Entered, State::Parked)
        }
        unsafe fn arrive(&self, _: &TaskContext, new_job: bool) -> Result<(), WorkError> {
            let previous = if new_job {
                State::Completed
            } else {
                State::Parked
            };
            self.transition(previous, State::Entered)
        }
        unsafe fn complete(&self, _: &TaskContext) -> Result<(), WorkError> {
            self.transition(State::Exited, State::Completed)
        }
        unsafe fn detach(&self, _: &TaskContext) -> Result<(), WorkError> {
            self.transition(State::Completed, State::Detached)
        }
    }

    struct Wait<'a> {
        runtime: Pin<&'a HostedRuntime<4>>,
        binding: Rc<Binding>,
        wake: RefCell<Option<HostedWake>>,
    }
    unsafe extern "C" fn wait(context: *mut c_void) {
        // SAFETY: test keeps this context and its runtime through the return.
        let wait = unsafe { &*context.cast::<Wait<'_>>() };
        let wake = wait.runtime.bind_current(wait.binding.clone()).unwrap();
        // A publication before the final park check must preserve current.
        wake.wake().unwrap();
        assert_eq!(
            wait.runtime.park_if_blocked().unwrap(),
            ParkOutcome::Skipped
        );
        assert_eq!(wait.binding.0.get(), State::Entered);
        *wait.wake.borrow_mut() = Some(wake);
        assert_eq!(
            wait.runtime.park_if_blocked().unwrap(),
            ParkOutcome::Resumed
        );
        wait.binding
            .transition(State::Entered, State::Exited)
            .unwrap();
    }

    unsafe extern "C" fn reuse(context: *mut c_void) {
        // SAFETY: the test keeps this context through the exact return.
        let wait = unsafe { &*context.cast::<Wait<'_>>() };
        assert_eq!(wait.binding.0.get(), State::Entered);
        *wait.wake.borrow_mut() = Some(wait.runtime.current_wake().unwrap());
        wait.binding
            .transition(State::Entered, State::Exited)
            .unwrap();
    }

    #[test]
    fn hosted_park_preserves_foreign_scope_until_real_wake_and_detach() {
        let queue = HostedQueue::new(HostedExecution::new(), Arc::new(())).unwrap();
        // SAFETY: bounded callback on the original thread, no shadow stacks.
        let executor = unsafe { HostedExecutor::new(queue.clone(), 16) }.unwrap();
        let runtime = executor.runtime();
        let context = Wait {
            runtime,
            binding: Rc::new(Binding(Cell::new(State::New))),
            wake: RefCell::new(None),
        };
        // SAFETY: context stays alive until callback return, and the static
        // callback is bounded by the executor's stack.
        let ticket = unsafe {
            queue.execute(
                Kind::General,
                wait,
                ptr::from_ref(&context).cast_mut().cast(),
            )
        }
        .unwrap();
        assert!(runtime.step().unwrap());
        assert_eq!(context.binding.0.get(), State::Parked);
        assert!(!runtime.step().unwrap());
        assert!(!queue.completed(ticket).unwrap());
        let wake = context.wake.borrow_mut().take().unwrap();
        wake.wake().unwrap();
        assert!(runtime.step().unwrap());
        assert_eq!(context.binding.0.get(), State::Completed);
        assert!(queue.completed(ticket).unwrap());
        assert_eq!(wake.wake(), Err(WorkError::Closed));
        // SAFETY: same retained source and actual stack through its new return.
        let reused = unsafe {
            queue.execute(
                Kind::General,
                reuse,
                ptr::from_ref(&context).cast_mut().cast(),
            )
        }
        .unwrap();
        assert!(runtime.step().unwrap());
        assert!(queue.completed(reused).unwrap());
        // SAFETY: no source user remains. An old completion still must not
        // detach the newer assignment of the same physical stack.
        let detached = unsafe { runtime.detach_binding(&wake) };
        assert!(matches!(detached, Err(WorkError::InvalidTicket)));
        assert_eq!(context.binding.0.get(), State::Completed);
        let wake = context.wake.borrow_mut().take().unwrap();
        queue.close().unwrap();
        assert_eq!(runtime.retire(), Err(WorkError::Busy));
        // SAFETY: the sole callback completed and has no remaining users.
        drop(unsafe { runtime.detach_binding(&wake) }.unwrap());
        assert_eq!(context.binding.0.get(), State::Detached);
        runtime.retire().unwrap();
        // SAFETY: the only producer stopped and the callback joined.
        unsafe { queue.retire() }.unwrap();
    }

    struct WaitTest<'a> {
        runtime: Pin<&'a HostedRuntime<4>>,
        target: Cell<Option<HostedWake>>,
        waiter: Cell<Option<HostedWake>>,
    }

    unsafe extern "C" fn target(raw: *mut c_void) {
        // SAFETY: the dispatcher keeps this fixture through both callbacks.
        let test = unsafe { &*raw.cast::<WaitTest<'_>>() };
        let runtime = test.runtime;
        test.target.set(Some(runtime.current_wake().unwrap()));
        runtime.yield_now().unwrap();
        let binding = Rc::new(Binding(Cell::new(State::New)));
        runtime.bind_current(binding.clone()).unwrap();
        // Post a separate ordinary wake immediately before the dispatcher
        // records the target's return for the inactive subscription.
        let waiter = test.waiter.take().unwrap();
        waiter.wake().unwrap();
        test.waiter.set(Some(waiter));
        assert_eq!(runtime.park_if_blocked(), Ok(ParkOutcome::Resumed));
        binding.transition(State::Entered, State::Exited).unwrap();
    }

    unsafe extern "C" fn waiter(raw: *mut c_void) {
        // SAFETY: the dispatcher keeps this fixture through both callbacks.
        let test = unsafe { &*raw.cast::<WaitTest<'_>>() };
        let runtime = test.runtime;
        let binding = Rc::new(Binding(Cell::new(State::New)));
        test.waiter
            .set(Some(runtime.bind_current(binding.clone()).unwrap()));
        let target = test.target.take().unwrap();
        assert_eq!(
            runtime.observe_inactive(&target, false),
            Ok(InactiveObservation::Subscribed)
        );
        assert_eq!(runtime.park_if_blocked(), Ok(ParkOutcome::Resumed));
        runtime.cancel_inactive(&target).unwrap();
        assert_eq!(
            runtime.observe_inactive(&target, false),
            Ok(InactiveObservation::Inactive)
        );
        assert_eq!(runtime.park_if_blocked(), Ok(ParkOutcome::Skipped));
        target.wake().unwrap();
        assert_eq!(
            runtime.observe_inactive(&target, false),
            Ok(InactiveObservation::Subscribed)
        );
        assert_eq!(runtime.park_if_blocked(), Ok(ParkOutcome::Resumed));
        runtime.cancel_inactive(&target).unwrap();
        assert_eq!(
            runtime.observe_inactive(&target, false),
            Err(WorkError::Closed)
        );
        test.target.set(Some(target));
        binding.transition(State::Entered, State::Exited).unwrap();
    }

    #[test]
    fn inactive_notification_preserves_an_independent_wake() {
        let queue = HostedQueue::new(HostedExecution::new(), Arc::new(())).unwrap();
        let geometry = StackRequirement::exact(16 * 1024, 16 * 1024, PAGE).unwrap();
        // SAFETY: all calls and bound contexts remain kept through detach.
        let executor =
            unsafe { HostedExecutor::new_with_stacks(queue.clone(), geometry, 2) }.unwrap();
        let runtime = executor.runtime();
        let test = WaitTest {
            runtime,
            target: Cell::new(None),
            waiter: Cell::new(None),
        };
        let raw = ptr::from_ref(&test).cast_mut().cast();
        // SAFETY: same live fixture, two separate hosted callbacks.
        let target = unsafe { queue.execute(Kind::General, target, raw) }.unwrap();
        // SAFETY: the dispatcher retains the context for this second callback.
        let waiter = unsafe { queue.execute(Kind::General, waiter, raw) }.unwrap();
        for _ in 0..6 {
            assert_eq!(runtime.step(), Ok(true));
        }
        assert_eq!(queue.completed(target), Ok(true));
        assert_eq!(queue.completed(waiter), Ok(true));
        assert_eq!(runtime.step(), Ok(false));
        // SAFETY: both exact callbacks have physically returned.
        drop(unsafe { runtime.detach_binding(&test.target.take().unwrap()) }.unwrap());
        // SAFETY: the source wait and this exact callback have both joined.
        drop(unsafe { runtime.detach_binding(&test.waiter.take().unwrap()) }.unwrap());
        queue.close().unwrap();
        runtime.retire().unwrap();
        // SAFETY: both bindings and all actual callbacks have joined.
        unsafe { queue.retire() }.unwrap();
    }
}
