//! Original C task ABI on the actual hosted continuation's guarded stack.
use super::*;
use alloc::{boxed::Box, rc::Rc};
use core::cell::{Cell, RefCell};
use stela_driver_base::platform::work::{
    InactiveObservation,
    kernel_domain::{DomainRef, HostedRuntime, HostedSwitchBinding, HostedWake},
};
mod fork;
pub(crate) use fork::service_progress;
pub use fork::step_hosted;
mod initial;
pub use initial::{initialize_hosted_resources, start_hosted_threads};
mod stack;
pub use stack::{HostedStackReservation, reserve_hosted_child_stack};

/// The original source record attached to one checked hosted continuation.
pub struct HostedTaskBinding<const N: usize> {
    task: NonNull<CTask>,
    runtime: NonNull<HostedRuntime<N>>,
    wake: RefCell<Option<HostedWake>>,
    detached: Cell<bool>,
    released: bool,
    kind: Kind,
    identity: Cell<Option<TaskContext>>,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Kind {
    Bare,
    Initial,
    Child,
}

// C also pins the target record and task reference through this exact wait.
struct InactiveOwner {
    target: HostedWake,
    subscribed: bool,
}

impl<const N: usize> HostedTaskBinding<N> {
    const OPS: TaskOps = TaskOps {
        is_current: Self::is_current,
        wake_target: Self::wake_target,
        service_reserved: Self::service_reserved,
        rank: Self::rank,
        retain_inactive: Some(Self::retain_inactive),
    };
    const SCOPE_OPS: ScopeOps = ScopeOps {
        park: Self::park,
        yield_now: Self::yield_now,
        reschedule: Self::reschedule,
        reschedule_requested: Self::reschedule_requested,
        terminate: Self::terminate,
        observe_inactive: Some(Self::observe_inactive),
        end_inactive: Some(Self::end_inactive),
        account_current: Some(Self::account_current),
    };

    /// # Safety
    /// This is the sole !SMP C archive owner thread for the fixture. The
    /// canonical heap and actual pinned executor/container stay alive through
    /// every callback, detach and final release, including abandoned failures.
    pub unsafe fn new_bare(runtime: Pin<&HostedRuntime<N>>) -> Result<Rc<Self>, WorkError> {
        DomainRef::borrow_hosted(runtime)?.dispatcher_successor()?;
        // SAFETY: caller supplies the original heap and serialized dispatcher.
        let task = NonNull::new(unsafe { stela_linux_task_allocate() }).ok_or(WorkError::Full)?;
        Ok(Self::new_record(task, runtime, Kind::Bare))
    }

    fn new_record(task: NonNull<CTask>, runtime: Pin<&HostedRuntime<N>>, kind: Kind) -> Rc<Self> {
        Rc::new(Self {
            task,
            runtime: NonNull::from(runtime.get_ref()),
            wake: RefCell::new(None),
            detached: Cell::new(false),
            released: false,
            kind,
            identity: Cell::new(None),
        })
    }

    fn runtime(&self) -> Pin<&HostedRuntime<N>> {
        // SAFETY: the constructor retains the exact pinned executor lifetime.
        unsafe { Pin::new_unchecked(self.runtime.as_ref()) }
    }

    pub fn bind_current(self: &Rc<Self>) -> Result<HostedWake, WorkError> {
        if self.detached.get() || self.wake.borrow().is_some() {
            return Err(WorkError::Busy);
        }
        self.runtime().bind_current(self.clone())
    }

    /// Observe the hosted scheduler's charge for this binding's live
    /// assignment. This observation lends no source task-data writer.
    pub fn measured_execution_ns(&self) -> Result<u64, WorkError> {
        if self.detached.get() {
            return Err(WorkError::Closed);
        }
        let wake = self.wake.borrow();
        let target = wake.as_ref().ok_or(WorkError::Unavailable)?;
        self.runtime().execution_ns(target)
    }

    /// # Safety
    /// Body and context stay valid through every real suspension. The source
    /// body cannot unwind, retire the executor or retain the scoped capability.
    pub unsafe fn run_scoped(
        &self,
        body: unsafe extern "C" fn(*mut c_void),
        context: *mut c_void,
    ) -> Result<(), WorkError> {
        let runtime = self.runtime();
        let actual = runtime.current_wake()?;
        if self.wake.borrow().as_ref().map(HostedWake::sequence) != Some(actual.sequence()) {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: exact active assignment on the same runtime, scoped body.
        unsafe { run_scope(self.task, &Self::SCOPE_OPS, runtime, body, context) }
    }

    /// # Safety
    /// Original producer/waiter/lease users have actually joined. This must
    /// run on the sole dispatcher, after successful detach if ever attached.
    pub unsafe fn release_quiescent(self: Rc<Self>) -> Result<(), Rc<Self>> {
        if self.kind != Kind::Bare
            || DomainRef::borrow_hosted(self.runtime())
                .and_then(|domain| domain.dispatcher_successor())
                .is_err()
            || (self.wake.borrow().is_some() && !self.detached.get())
        {
            return Err(self);
        }
        let mut binding = Rc::try_unwrap(self)?;
        // SAFETY: exact unique bare record plus caller's original joins.
        unsafe { stela_linux_task_release(binding.task.as_ptr()) };
        binding.released = true;
        Ok(())
    }

    unsafe extern "C" fn is_current(binding: *mut c_void, view: *const TaskContext) -> bool {
        // SAFETY: the C stack record borrows the retained original Rc.
        let this = unsafe { &*binding.cast::<Self>() };
        let Ok(actual) = DomainRef::borrow_hosted(this.runtime()).and_then(|d| d.current_context())
        else {
            return false;
        };
        if view.is_null()
            || this.wake.borrow().as_ref().map(HostedWake::sequence) != Some(actual.sequence)
        {
            return false;
        }
        // SAFETY: actual checked host thread and RSP before source observation.
        let view = unsafe { &*view };
        actual.queue_id == view.queue_id
            && actual.context_id == view.context_id
            && actual.sequence == view.sequence
    }

    unsafe extern "C" fn wake_target(binding: *mut c_void, expected: u64) -> i32 {
        // SAFETY: C retains this exact binding until producer joins.
        let this = unsafe { &*binding.cast::<Self>() };
        let wake = this.wake.borrow();
        let Some(wake) = wake.as_ref().filter(|wake| wake.sequence() == expected) else {
            return -116;
        };
        match wake.wake() {
            Ok(()) => 0,
            Err(WorkError::Closed) => -116,
            Err(_) => -1,
        }
    }

    unsafe extern "C" fn retain_inactive(
        binding: *mut c_void,
        sequence: u64,
        out: *mut *mut c_void,
    ) -> i32 {
        // SAFETY: the original wait retains the target record and borrows
        // its exact installed binding while handing over an output slot.
        let Some(binding) = (unsafe { binding.cast::<Self>().as_ref() }) else {
            return -22;
        };
        // SAFETY: the original source lends one writable opaque pointer.
        let Some(out) = (unsafe { out.as_mut() }) else {
            return -22;
        };
        let wake = binding.wake.borrow();
        let Some(target) = wake.as_ref().filter(|wake| wake.sequence() == sequence) else {
            return -116;
        };
        if binding.detached.get() || sequence == 0 {
            return -116;
        }
        *out = Box::into_raw(Box::new(InactiveOwner {
            target: target.clone(),
            subscribed: false,
        }))
        .cast();
        0
    }

    unsafe extern "C" fn observe_inactive(
        scope: *mut c_void,
        opaque: *mut c_void,
        force_wait: bool,
    ) -> i32 {
        // SAFETY: the original wait lends its own scope and retained target.
        let (runtime, owner) = unsafe {
            (
                &*scope.cast::<Pin<&HostedRuntime<N>>>(),
                &mut *opaque.cast::<InactiveOwner>(),
            )
        };
        match runtime.observe_inactive(&owner.target, force_wait) {
            Ok(InactiveObservation::Inactive) => 1,
            Ok(InactiveObservation::Subscribed) => {
                owner.subscribed = true;
                0
            }
            Err(error) => scope_status(Err(error)),
        }
    }

    unsafe extern "C" fn end_inactive(scope: *mut c_void, opaque: *mut c_void) -> i32 {
        // SAFETY: only the original wait owns this shell and its live scope.
        let (runtime, owner) = unsafe {
            (
                &*scope.cast::<Pin<&HostedRuntime<N>>>(),
                &*opaque.cast::<InactiveOwner>(),
            )
        };
        if owner.subscribed
            && let Err(error) = runtime.cancel_inactive(&owner.target)
        {
            return scope_status(Err(error));
        }
        // SAFETY: the exact registration is gone and C transfers this owner once.
        drop(unsafe { Box::from_raw(opaque.cast::<InactiveOwner>()) });
        0
    }

    unsafe extern "C" fn service_reserved(binding: *mut c_void) -> bool {
        // SAFETY: C retains the same binding and checks its assignment.
        unsafe { &*binding.cast::<Self>() }.kind == Kind::Initial
    }

    unsafe extern "C" fn rank(
        binding: *mut c_void,
        expected: u64,
        before: *const Rank,
        after: *const Rank,
        head: bool,
    ) -> i32 {
        // SAFETY: the original caller retains the target binding and holds
        // its priority mutation lease on this runtime's owning thread.
        let this = unsafe { &*binding.cast::<Self>() };
        let runtime = this.runtime();
        let wake = this.wake.borrow();
        let Some(target) = wake.as_ref().filter(|wake| wake.sequence() == expected) else {
            return -116;
        };
        if before.is_null() || after.is_null() || this.detached.get() {
            return -116;
        }
        let result = (|| {
            // SAFETY: original C lends two initialized scalar records for
            // this bounded priority update and retains the target lease.
            let (before, after) = unsafe { ((&*before).snapshot()?, (&*after).snapshot()?) };
            // SAFETY: the original priority mutation stays serialized until
            // C commits these same fields. The update cannot switch stacks.
            unsafe { runtime.update_rank(target, before, after, head) }
        })();
        match result {
            Ok(()) => 0,
            Err(WorkError::Closed) => -116,
            Err(WorkError::Unavailable) => -95,
            Err(_) => -1,
        }
    }

    unsafe extern "C" fn park(scope: *mut c_void, service_idle: bool) -> i32 {
        // SAFETY: run_scope retains this exact pinned borrow through resume.
        let runtime = *unsafe { &*scope.cast::<Pin<&HostedRuntime<N>>>() };
        let result = if service_idle {
            runtime.park_service_if_idle()
        } else {
            runtime.park_if_blocked()
        };
        match result {
            Ok(ParkOutcome::Skipped) => 0,
            Ok(ParkOutcome::Resumed) => 1,
            Err(WorkError::Busy) => -16,
            Err(_) => -22,
        }
    }

    unsafe extern "C" fn yield_now(scope: *mut c_void) -> i32 {
        // SAFETY: same live stack scope as park.
        scope_status(unsafe { &*scope.cast::<Pin<&HostedRuntime<N>>>() }.yield_now())
    }

    unsafe extern "C" fn reschedule(scope: *mut c_void) -> i32 {
        // SAFETY: same live stack scope as park.
        scope_status(unsafe { &*scope.cast::<Pin<&HostedRuntime<N>>>() }.reschedule())
    }

    unsafe extern "C" fn reschedule_requested(scope: *mut c_void) -> bool {
        // SAFETY: same live stack scope as park.
        let runtime = unsafe { &*scope.cast::<Pin<&HostedRuntime<N>>>() };
        runtime.reschedule_requested().unwrap_or_else(|_| {
            // SAFETY: an invalid current query must stop before the C caller proceeds.
            unsafe { core::arch::asm!("ud2", options(noreturn)) }
        })
    }

    unsafe extern "C" fn account_current(scope: *mut c_void, total_ns: *mut u64) -> i32 {
        // SAFETY: C supplies the exact pinned current scope and one writable
        // result. The hosted scheduler authenticates and charges that stack.
        let runtime = unsafe { &*scope.cast::<Pin<&HostedRuntime<N>>>() };
        match runtime.account_current_execution() {
            Ok(sample) => {
                // SAFETY: the caller lends the result for this bounded call.
                unsafe { total_ns.write(sample) };
                0
            }
            Err(WorkError::Unavailable) => -95,
            Err(_) => -1,
        }
    }

    unsafe extern "C" fn terminate(scope: *mut c_void) -> ! {
        // SAFETY: C entered from original do_task_dead and keeps the exact
        // child scope. The runtime independently checks the live child stack.
        let runtime = unsafe { *scope.cast::<Pin<&HostedRuntime<N>>>() };
        // SAFETY: the nonreturning C body owns no returning Rust guards.
        unsafe { runtime.terminate_child() }
    }
}

fn scope_status(result: Result<(), WorkError>) -> i32 {
    match result {
        Ok(()) => 0,
        Err(WorkError::Busy) => -16,
        Err(_) => -22,
    }
}

// SAFETY: the original hosted thread owns all task state. The hooks call the
// same bounded C ABI as native, with checked hosted current and retained roots.
unsafe impl<const N: usize> HostedSwitchBinding for HostedTaskBinding<N> {
    unsafe fn attach(&self, context: &TaskContext) -> Result<(), WorkError> {
        if self.detached.get() || self.wake.borrow().is_some() {
            return Err(WorkError::Busy);
        }
        let runtime = self.runtime();
        if runtime.service_current()? != (self.kind == Kind::Initial) {
            return Err(WorkError::Unavailable);
        }
        let wake = runtime.current_wake()?;
        if wake.sequence() != context.sequence {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: checked live stack, immutable ops and retained Rc receiver.
        status(unsafe {
            stela_linux_task_attach(
                self.task.as_ptr(),
                context,
                &Self::OPS,
                core::ptr::from_ref(self).cast_mut().cast(),
            )
        })?;
        self.identity.set(Some(*context));
        *self.wake.borrow_mut() = Some(wake);
        Ok(())
    }

    unsafe fn still_blocked(&self, context: &TaskContext) -> Result<bool, WorkError> {
        // SAFETY: actual current and original source-prepared wait.
        match unsafe { stela_linux_task_still_blocked(self.task.as_ptr(), context) } {
            0 => Ok(false),
            1 => Ok(true),
            -16 => Err(WorkError::Busy),
            _ => Err(WorkError::corrupt()),
        }
    }

    unsafe fn leave(&self, context: &TaskContext, reason: LeaveReason) -> Result<(), WorkError> {
        // SAFETY: exact outgoing stack and source-prepared suspension.
        status(unsafe { stela_linux_task_leave(self.task.as_ptr(), context, reason as u32) })
    }

    unsafe fn arrive(&self, context: &TaskContext, new_job: bool) -> Result<(), WorkError> {
        if new_job {
            if self.kind != Kind::Bare {
                return Err(WorkError::corrupt());
            }
            // Old signal/OS execution owners remain kept by the same runtime.
            *self.wake.borrow_mut() = Some(self.runtime().current_wake()?);
        }
        // SAFETY: actual destination stack with its exact current assignment.
        status(unsafe { stela_linux_task_arrive(self.task.as_ptr(), context, new_job) })
    }

    unsafe fn complete(&self, context: &TaskContext) -> Result<(), WorkError> {
        // SAFETY: actual finishing stack; C checks that the source scope ended.
        status(unsafe { stela_linux_task_complete(self.task.as_ptr(), context) })
    }

    unsafe fn terminal_prepare(&self, context: &TaskContext) -> Result<(), WorkError> {
        if self.kind != Kind::Child {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: actual current, source terminal path and kept child record.
        status(unsafe { stela_linux_task_terminal_prepare(self.task.as_ptr(), context) })
    }

    unsafe fn terminal_successor(&self, context: &TaskContext) -> Result<(), WorkError> {
        if self.kind != Kind::Child {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: runtime returned to its dispatcher from this exact child.
        status(unsafe { stela_linux_task_terminal_successor(self.task.as_ptr(), context) })
    }

    unsafe fn detach(&self, context: &TaskContext) -> Result<(), WorkError> {
        // SAFETY: the actual dispatcher and independently joined users.
        status(unsafe { stela_linux_task_detach(self.task.as_ptr(), context) })?;
        self.detached.set(true);
        Ok(())
    }
}

impl<const N: usize> Drop for HostedTaskBinding<N> {
    fn drop(&mut self) {
        if !self.released {
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
    }
}
