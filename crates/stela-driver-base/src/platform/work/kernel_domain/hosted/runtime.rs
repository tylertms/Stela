//! Linux userspace continuations with distinct guarded mmap stacks.
use super::{HostedDeadlines, HostedPages, HostedQueue, WorkError, memory::PAGE, std};
use crate::platform::work::{
    Continuation, Lane, Layout, NONE, StackRequirement, TaskContext, TaskState, context,
    stack_requirement::Geometry,
};
use alloc::{boxed::Box, rc::Rc, sync::Arc, vec::Vec};
use core::{
    cell::{Cell, UnsafeCell},
    ffi::c_void,
    marker::{PhantomData, PhantomPinned},
    mem::ManuallyDrop,
    ops::Range,
    pin::Pin,
    ptr,
    sync::atomic::Ordering,
};

mod binding;
pub use binding::{HostedSwitchBinding, HostedWake};
mod child;
pub use child::{HostedChild, HostedChildReservation};
mod service;
use binding::InactiveWait;
pub use service::HostedServiceCallback;
use service::Service;

struct Stack {
    _pages: HostedPages,
    bounds: Range<usize>,
    geometry: Geometry,
    guard: Range<usize>,
}
type Task = Continuation<Stack, (), Rc<dyn HostedSwitchBinding>>;

// All hosted executors on one OS thread share the actual dispatcher boundary.
// Keeping this in each Runtime separately would let a running callback borrow
// another Runtime's apparent dispatcher or even enter a nested source stack.
#[derive(Clone, Copy)]
struct Dispatch {
    active: *const (),
    epoch: u64,
}
std::thread_local! {
    static DISPATCH: Cell<Dispatch> = const {
        Cell::new(Dispatch { active: ptr::null(), epoch: 0 })
    };
}

pub struct HostedExecutor<const N: usize> {
    runtime: ManuallyDrop<Pin<Box<HostedRuntime<N>>>>,
}
pub struct HostedRuntime<const N: usize> {
    queue: Arc<HostedQueue<N>>,
    tasks: UnsafeCell<Vec<Task>>,
    inactive: Box<[Cell<Option<InactiveWait>>]>,
    count: usize,
    service: Option<Service>,
    initial_controls: context::Controls,
    dispatcher: UnsafeCell<context::Frame>,
    current: Cell<usize>,
    cursor: Cell<usize>,
    retired: Cell<bool>,
    _local: PhantomData<*mut ()>,
    _pinned: PhantomPinned,
}

impl<const N: usize> HostedExecutor<N> {
    fn default_stack(pages: usize) -> Result<StackRequirement, WorkError> {
        let bytes = pages.checked_mul(PAGE).ok_or(WorkError::InvalidStack)?;
        StackRequirement::exact(bytes, PAGE, PAGE)
    }

    /// Allocate exclusive guarded stacks for General and Notify callbacks.
    ///
    /// # Safety
    /// Callbacks must fit within the supplied stack and must not unwind or
    /// jump across the switch boundary. The original thread must keep shadow
    /// stacks disabled while this executor can run, including in callbacks.
    pub unsafe fn new(queue: Arc<HostedQueue<N>>, stack_pages: usize) -> Result<Self, WorkError> {
        Self::build(queue, Self::default_stack(stack_pages)?, 1, None)
    }

    /// Allocate guarded General continuations and one reserved Notify stack.
    /// Each actual mmap backing meets the caller's source stack requirement.
    ///
    /// # Safety
    /// Same lifetime, unwind and thread requirements as `new`.
    pub unsafe fn new_with_stacks(
        queue: Arc<HostedQueue<N>>,
        stack: StackRequirement,
        general_stacks: usize,
    ) -> Result<Self, WorkError> {
        Self::build(queue, stack, general_stacks, None)
    }

    /// Add one guarded service stack and reserve one queue slot for its work.
    ///
    /// # Safety
    /// The requirements of `new` apply to all callbacks. Keep the service
    /// callback, context and reachable storage/code alive through actual
    /// retirement or retained abandonment. The callback must deliver its
    /// captured reasons and return the actual remaining timer deadline.
    pub unsafe fn new_with_service(
        queue: Arc<HostedQueue<N>>,
        stack_pages: usize,
        deadlines: HostedDeadlines,
        callback: HostedServiceCallback,
        context: *mut c_void,
    ) -> Result<Self, WorkError> {
        // SAFETY: the caller supplies the same service and callback lifetimes.
        unsafe {
            Self::new_with_service_stacks(
                queue,
                Self::default_stack(stack_pages)?,
                1,
                deadlines,
                callback,
                context,
            )
        }
    }

    /// Add the original reserved service lane to an explicit stack layout.
    ///
    /// # Safety
    /// Same lifetime, unwind and thread requirements as `new_with_service`.
    pub unsafe fn new_with_service_stacks(
        queue: Arc<HostedQueue<N>>,
        stack: StackRequirement,
        general_stacks: usize,
        deadlines: HostedDeadlines,
        callback: HostedServiceCallback,
        context: *mut c_void,
    ) -> Result<Self, WorkError> {
        Self::build(
            queue,
            stack,
            general_stacks,
            Some(Service::bounded(deadlines, callback, context)),
        )
    }

    /// Reserve a service stack for one persistent hosted assignment. Its
    /// source starts it explicitly and consumes each owned deadline batch.
    ///
    /// # Safety
    /// Same stack, thread and unwind requirements as `new`.
    pub unsafe fn new_with_persistent_service_stacks(
        queue: Arc<HostedQueue<N>>,
        stack: StackRequirement,
        general_stacks: usize,
        deadlines: HostedDeadlines,
    ) -> Result<Self, WorkError> {
        Self::build(
            queue,
            stack,
            general_stacks,
            Some(Service::persistent(deadlines)),
        )
    }

    fn build(
        queue: Arc<HostedQueue<N>>,
        stack: StackRequirement,
        general_stacks: usize,
        service: Option<Service>,
    ) -> Result<Self, WorkError> {
        queue.execution.check_current()?;
        let layout = if let Some(service) = &service {
            if !service.same_execution(&queue.execution) {
                return Err(WorkError::InvalidTicket);
            }
            Layout::HostedService
        } else {
            Layout::Ordinary
        };
        let count = general_stacks
            .checked_add(1 + usize::from(service.is_some()))
            .filter(|count| general_stacks != 0 && *count <= layout.stack_limit())
            .ok_or(WorkError::InvalidStack)?;
        layout.validate(count, N)?;
        // This switch backend uses RET with a replacement RSP. Linux reports
        // the actual shadow-stack state of this thread through arch_prctl.
        let mut features = 0u64;
        // SAFETY: Linux x86-64 arch_prctl(ARCH_SHSTK_STATUS, &features).
        let result = unsafe {
            let result: i64;
            core::arch::asm!(
                "syscall", inlateout("rax") 158i64 => result,
                in("rdi") 0x5005usize, in("rsi") &mut features,
                lateout("rcx") _, lateout("r11") _, options(nostack),
            );
            result
        };
        if result != 0 || features != 0 {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: reads the current thread's user-accessible FP controls.
        let controls = unsafe { context::read_controls() };
        let task = |lane| {
            let pages = HostedPages::allocate(
                &queue.execution,
                stack.usable_bytes() / PAGE,
                stack.alignment().max(PAGE),
                stack.minimum_guard_bytes() / PAGE,
            )
            .map_err(|_| WorkError::InvalidStack)?;
            let start = pages
                .cpu_address(0, 1)
                .map_err(|_| WorkError::InvalidStack)?
                .as_ptr()
                .addr();
            let bounds = start
                ..start
                    .checked_add(pages.bytes())
                    .ok_or(WorkError::InvalidStack)?;
            let guard = pages.guard();
            let geometry = Geometry::from_ranges(&bounds, &guard);
            Task::new(
                lane,
                Stack {
                    _pages: pages,
                    bounds,
                    geometry,
                    guard,
                },
                controls,
                (),
            )
        };
        let mut tasks = Vec::new();
        tasks
            .try_reserve_exact(count)
            .map_err(|_| WorkError::Full)?;
        for _ in 0..general_stacks {
            tasks.push(task(Lane::General)?);
        }
        tasks.push(task(Lane::Notify)?);
        if service.is_some() {
            tasks.push(task(Lane::Service)?);
        }
        let mut inactive = Vec::new();
        inactive
            .try_reserve_exact(count)
            .map_err(|_| WorkError::Full)?;
        inactive.resize_with(count, || Cell::new(None));
        let runtime = Box::pin(HostedRuntime {
            queue,
            tasks: UnsafeCell::new(tasks),
            inactive: inactive.into_boxed_slice(),
            count,
            service,
            initial_controls: controls,
            dispatcher: UnsafeCell::new(context::Frame::new(controls)),
            current: Cell::new(NONE),
            cursor: Cell::new(0),
            retired: Cell::new(false),
            _local: PhantomData,
            _pinned: PhantomPinned,
        });
        {
            let mut guard = runtime.queue.lock()?;
            let state = guard.state();
            if state.closed {
                return Err(WorkError::Closed);
            }
            if layout == Layout::HostedService && state.jobs[N - 1].is_some() {
                return Err(WorkError::Busy);
            }
            runtime
                .queue
                .attached
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .map_err(|_| WorkError::Busy)?;
            state.layout = layout;
        }
        let address = ptr::from_ref(runtime.as_ref().get_ref()).addr();
        for index in 0..runtime.count {
            // SAFETY: this pinned runtime has no live frames yet and its
            // producer cannot execute tasks during construction.
            let task = unsafe { &mut *runtime.task(index) };
            // SAFETY: mmap provides aligned, exclusive writable stack tops.
            unsafe {
                task.frame
                    .start(task.stack.bounds.end, address, index, entry::<N>)
            };
        }
        Ok(Self {
            runtime: ManuallyDrop::new(runtime),
        })
    }

    pub fn runtime(&self) -> Pin<&HostedRuntime<N>> {
        self.runtime.as_ref()
    }
}

impl<const N: usize> Drop for HostedExecutor<N> {
    fn drop(&mut self) {
        if self.runtime.retired.get() {
            // SAFETY: dispatcher retirement excludes any live callback frame.
            unsafe { ManuallyDrop::drop(&mut self.runtime) };
        }
    }
}

impl<const N: usize> HostedRuntime<N> {
    /// Original OS-thread owner of this runtime's modeled memory namespace.
    pub fn execution(&self) -> &Arc<super::HostedExecution> {
        &self.queue.execution
    }

    fn task(&self, index: usize) -> *mut Task {
        // SAFETY: internal indices always name one initialized task in the
        // immutable, retained allocation.
        unsafe { (&mut *self.tasks.get()).as_mut_ptr().add(index) }
    }

    pub(in crate::platform::work::kernel_domain) fn check(&self) -> Result<(), WorkError> {
        self.queue.execution.check_current()?;
        if self.retired.get() {
            return Err(WorkError::Closed);
        }
        Ok(())
    }

    pub(in crate::platform::work::kernel_domain) fn current_context(
        &self,
    ) -> Result<crate::platform::work::TaskContext, WorkError> {
        self.check()?;
        let index = self.current.get();
        if index == NONE {
            return Err(WorkError::Unavailable);
        }
        if DISPATCH.get().active != ptr::from_ref(self).cast() {
            return Err(WorkError::corrupt());
        }
        // SAFETY: the checked original thread is executing this task. No
        // mutable task borrow spans a call to this observation.
        let task = unsafe { &*self.task(index) };
        let stack_pointer = context::stack_pointer();
        if task.state != TaskState::Running || !task.stack.bounds.contains(&stack_pointer) {
            return Err(WorkError::corrupt());
        }
        Ok(self.context_view(task, stack_pointer))
    }

    fn context_view(&self, task: &Task, stack_pointer: usize) -> TaskContext {
        TaskContext {
            queue_id: self.queue.id,
            context_id: task.id.get(),
            sequence: task.job.map_or(0, |(_, job)| job.sequence),
            stack_start: task.stack.bounds.start,
            stack_end: task.stack.bounds.end,
            guard_start: task.stack.guard.start,
            guard_end: task.stack.guard.end,
            stack_pointer,
        }
    }

    pub(in crate::platform::work::kernel_domain) fn segment_epoch(&self) -> Result<u64, WorkError> {
        self.current_context()?;
        Ok(DISPATCH.get().epoch)
    }

    pub(in crate::platform::work::kernel_domain) fn dispatcher_successor(
        &self,
    ) -> Result<(), WorkError> {
        self.dispatcher_epoch().map(|_| ())
    }

    pub(in crate::platform::work::kernel_domain) fn dispatcher_epoch(
        &self,
    ) -> Result<u64, WorkError> {
        self.check()?;
        let dispatch = DISPATCH.get();
        if !dispatch.active.is_null() || self.current.get() != NONE {
            return Err(WorkError::Busy);
        }
        Ok(dispatch.epoch)
    }

    /// Use the shared cooperative NORMAL/FIFO policy for future assignments.
    /// The default executor keeps round-robin ordering.
    pub fn enable_ranked_scheduling(&self) -> Result<(), WorkError> {
        self.dispatcher_successor()?;
        let mut guard = self.queue.lock()?;
        let state = guard.state();
        if state.closed {
            return Err(WorkError::Closed);
        }
        if state.jobs.iter().flatten().any(|job| job.running) {
            return Err(WorkError::Busy);
        }
        self.thread_cpu_ns()?;
        state.scheduling.enabled = true;
        Ok(())
    }

    pub fn step(self: Pin<&Self>) -> Result<bool, WorkError> {
        let epoch = self
            .dispatcher_epoch()
            .map_err(|error| match error {
                WorkError::Busy => WorkError::WouldDeadlock,
                other => other,
            })?
            .checked_add(1)
            .ok_or(WorkError::Exhausted)?;
        self.schedule_service()?;
        let Some(index) = self.select_ready()? else {
            return Ok(false);
        };
        let sequence = {
            // SAFETY: the original dispatcher selected this stopped task.
            let task = unsafe { &mut *self.task(index) };
            let sequence = task.job.unwrap_or_else(|| std::process::abort()).1.sequence;
            let mut guard = self.queue.lock()?;
            let board = &mut guard.state().scheduling;
            if board.enabled {
                let now = self.thread_cpu_ns()?;
                require(board.running(task.id, sequence, now));
            }
            task.state = TaskState::Running;
            sequence
        };
        DISPATCH.set(Dispatch {
            active: ptr::from_ref(self.get_ref()).cast(),
            epoch,
        });
        self.current.set(index);
        self.cursor.set((index + 1) % self.count);
        // SAFETY: both frames are pinned and owned by this thread. All
        // task/state borrows and queue guards ended before the switch.
        unsafe {
            context::switch(
                self.dispatcher.get(),
                ptr::addr_of!((*self.task(index)).frame),
            )
        };
        if self.current.get() != NONE {
            std::process::abort();
        }
        DISPATCH.set(Dispatch {
            active: ptr::null(),
            epoch,
        });
        self.observe_child_return(index);
        self.notify_inactive(index, sequence);
        Ok(true)
    }

    fn select_ready(&self) -> Result<Option<usize>, WorkError> {
        let mut guard = self.queue.lock()?;
        let state = guard.state();
        let mut selected = None;
        for offset in 0..self.count {
            let index = (self.cursor.get() + offset) % self.count;
            // SAFETY: the original dispatcher has stopped every continuation.
            let task = unsafe { &mut *self.task(index) };
            if matches!(
                task.state,
                TaskState::Reserved | TaskState::Dormant | TaskState::Retained
            ) {
                continue;
            }
            if task.state == TaskState::Idle {
                let Some(job) = state.select_job(task.lane, task.stack.geometry, false)? else {
                    continue;
                };
                require(state.scheduling.insert(
                    task.id,
                    job.1.sequence,
                    task.lane == Lane::Service,
                ));
                task.assign(job, ());
            }
            let sequence = task.job.unwrap_or_else(|| std::process::abort()).1.sequence;
            if task.state == TaskState::Blocked {
                if !self.inactive_ready(index, sequence)
                    && !task.signal.pending.swap(false, Ordering::AcqRel)
                {
                    continue;
                }
                require(state.scheduling.wake(task.id, sequence));
                task.state = TaskState::Runnable;
            }
            // A due service callback can advance deadlines while ordinary
            // jobs hold FIFO ranks. Legacy mode keeps the first ready task.
            if !state.scheduling.enabled || task.lane == Lane::Service {
                return Ok(Some(index));
            }
            let rank_key = (task.id, sequence);
            if let Some((_, prior)) = selected
                && !state.scheduling.precedes(rank_key, prior)?
            {
                continue;
            }
            selected = Some((index, rank_key));
        }
        Ok(selected.map(|(index, _)| index))
    }

    /// Linux thread CPU time for work accounting. Deadline delivery keeps
    /// the original wall-monotonic clock and advances even while we sleep.
    fn thread_cpu_ns(&self) -> Result<u64, WorkError> {
        self.queue.execution.check_current()?;
        const SYS_CLOCK_GETTIME: i64 = 228;
        const CLOCK_THREAD_CPUTIME_ID: usize = 3;
        #[repr(C)]
        struct Timespec {
            seconds: i64,
            nanoseconds: i64,
        }
        let mut stamp = core::mem::MaybeUninit::<Timespec>::uninit();
        // SAFETY: Linux x86-64 clock_gettime writes the two fields on success.
        // The output stays live for the entire syscall.
        let result = unsafe {
            let result: i64;
            core::arch::asm!(
                "syscall", inlateout("rax") SYS_CLOCK_GETTIME => result,
                in("rdi") CLOCK_THREAD_CPUTIME_ID, in("rsi") stamp.as_mut_ptr(),
                lateout("rcx") _, lateout("r11") _, options(nostack),
            );
            result
        };
        if result != 0 {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: the successful kernel call initialized the full Timespec.
        let stamp = unsafe { stamp.assume_init() };
        let seconds = u64::try_from(stamp.seconds).map_err(|_| WorkError::corrupt())?;
        let nanos = u64::try_from(stamp.nanoseconds)
            .ok()
            .filter(|nanos| *nanos < 1_000_000_000)
            .ok_or_else(|| WorkError::corrupt())?;
        seconds
            .checked_mul(1_000_000_000)
            .and_then(|seconds| seconds.checked_add(nanos))
            .ok_or(WorkError::Exhausted)
    }

    fn depart(&self, index: usize) {
        self.current.set(NONE);
        // SAFETY: called only from this live task with its dispatcher saved,
        // on its owning thread after ending all exclusive task/state borrows.
        unsafe {
            context::switch(
                ptr::addr_of_mut!((*self.task(index)).frame),
                self.dispatcher.get(),
            )
        };
    }

    fn finish_current(&self, index: usize, nonreturning: bool) {
        if self.inactive[index].get().is_some() {
            std::process::abort(); // an active source wait owner still keeps this assignment
        }
        // SAFETY: only the original running task can finish its own slot.
        let task = unsafe { &mut *self.task(index) };
        let (slot, job) = task.job.unwrap_or_else(|| std::process::abort());
        let mut guard = require(self.queue.lock());
        let state = guard.state();
        let child = task.stack_pin_nonce != 0;
        if state.scheduling.enabled {
            let now = require(self.thread_cpu_ns());
            require(state.scheduling.account(now));
            if child {
                require(state.scheduling.remove_terminal(task.id, job.sequence));
            } else {
                require(state.scheduling.remove(task.id, job.sequence));
            }
        }
        task.signal.sequence.store(0, Ordering::Release);
        require(state.finish_job(slot, job.sequence, None));
        task.completed_sequence = job.sequence;
        task.job = None;
        task.state = if nonreturning {
            TaskState::Terminating
        } else if child {
            TaskState::Retained
        } else {
            TaskState::Idle
        };
    }

    /// Stop dispatch after the caller closes the queue and drains callbacks.
    pub fn retire(self: Pin<&Self>) -> Result<(), WorkError> {
        self.queue.execution.check_current()?;
        if self.retired.get() {
            return Ok(());
        }
        self.dispatcher_successor()?;
        for index in 0..self.count {
            // SAFETY: all task stacks are stopped on the original dispatcher.
            let task = unsafe { &*self.task(index) };
            if task.state != TaskState::Idle || task.binding.is_some() {
                return Err(WorkError::Busy);
            }
        }
        let mut guard = self.queue.lock()?;
        let state = guard.state();
        if !state.closed
            || state.jobs.iter().any(Option::is_some)
            || self
                .service
                .as_ref()
                .is_some_and(|service| !service.retired.get())
        {
            return Err(WorkError::Busy);
        }
        self.retired.set(true);
        self.queue.attached.store(false, Ordering::Release);
        Ok(())
    }
}

fn require<T>(result: Result<T, WorkError>) -> T {
    result.unwrap_or_else(|_| std::process::abort())
}

unsafe extern "sysv64" fn entry<const N: usize>(address: usize, index: usize) -> ! {
    // SAFETY: only the trampoline in this executor's own task stack enters,
    // with the original pinned runtime kept by HostedExecutor.
    let runtime = unsafe { &*(address as *const HostedRuntime<N>) };
    loop {
        // SAFETY: immutable assignment for the exact running task. No borrow
        // of its state survives the callback or any cooperative yield.
        let job = unsafe { (*runtime.task(index)).job }.unwrap_or_else(|| std::process::abort());
        runtime.arrive(index, true);
        // SAFETY: queue admission retains the original callback/context graph.
        unsafe { (job.1.callback)(job.1.context as *mut c_void) };
        runtime.complete_binding(index);
        runtime.finish_current(index, false);
        runtime.depart(index);
    }
}

#[cfg(test)]
mod tests {
    use super::super::HostedExecution;
    use super::*;
    use crate::platform::work::Kind;
    use crate::platform::work::kernel_domain::DomainRef;

    struct Call<'a> {
        runtime: Pin<&'a HostedRuntime<4>>,
        slot: usize,
        visits: &'a Cell<u32>,
    }
    unsafe extern "C" fn run(raw: *mut c_void) {
        // SAFETY: the test retains this call and runtime through dispatch.
        let call = unsafe { &*raw.cast::<Call<'_>>() };
        assert_eq!(call.runtime.current.get(), call.slot);
        let domain = DomainRef::borrow_hosted(call.runtime).unwrap();
        let segment = domain.hosted_segment().unwrap();
        assert!(segment.matches(&domain));
        assert_eq!(segment.check(), Ok(()));
        assert_eq!(domain.dispatcher_successor(), Err(WorkError::Busy));
        // SAFETY: only this task runs, and the borrow ends before yielding.
        let current = domain.current_context().unwrap();
        assert!((current.stack_start..current.stack_end).contains(&context::stack_pointer()));
        assert_eq!(current.stack_end - current.stack_start, 16 * 1024);
        assert_eq!(current.stack_start % (16 * 1024), 0);
        assert_eq!(current.guard_end - current.guard_start, 2 * PAGE);
        call.visits
            .set(call.visits.get() * 10 + call.slot as u32 + 1);
        if call.slot == 0 {
            call.runtime.yield_now().unwrap();
            assert_eq!(segment.check(), Err(WorkError::InvalidTicket));
            assert_eq!(domain.hosted_segment().unwrap().check(), Ok(()));
            call.visits.set(call.visits.get() * 10 + 4);
        }
    }

    #[test]
    fn another_executor_cannot_dispatch_or_borrow_the_dispatcher_from_a_callback() {
        unsafe extern "C" fn refuse_nested(raw: *mut c_void) {
            // SAFETY: the original dispatcher keeps the second runtime alive
            // until this exact callback returns.
            let other = unsafe { *raw.cast::<Pin<&HostedRuntime<2>>>() };
            let domain = DomainRef::borrow_hosted(other).unwrap();
            assert_eq!(domain.dispatcher_successor(), Err(WorkError::Busy));
            assert!(matches!(
                domain.hosted_dispatcher_segment(),
                Err(WorkError::Busy)
            ));
            assert_eq!(other.step(), Err(WorkError::WouldDeadlock));
            assert_eq!(other.retire(), Err(WorkError::Busy));
        }
        unsafe extern "C" fn ordinary(_: *mut c_void) {}

        let first_queue = HostedQueue::<2>::new(HostedExecution::new(), Arc::new(())).unwrap();
        let second_queue = HostedQueue::<2>::new(HostedExecution::new(), Arc::new(())).unwrap();
        // SAFETY: each callback is bounded, with no shadow-stack changes.
        let first = unsafe { HostedExecutor::new(first_queue.clone(), 4) }.unwrap();
        // SAFETY: same conditions, with separate actual continuation stacks.
        let second = unsafe { HostedExecutor::new(second_queue.clone(), 4) }.unwrap();
        let first_runtime = first.runtime();
        let second_runtime = second.runtime();
        let first_domain = DomainRef::borrow_hosted(first_runtime).unwrap();
        let second_domain = DomainRef::borrow_hosted(second_runtime).unwrap();
        let segment = second_domain.hosted_dispatcher_segment().unwrap();
        assert!(segment.matches(&second_domain));
        assert!(!segment.matches(&first_domain));
        assert_eq!(segment.check(), Ok(()));

        // SAFETY: static callback, no context.
        let pending =
            unsafe { second_queue.execute(Kind::General, ordinary, ptr::null_mut()) }.unwrap();
        // SAFETY: second_runtime and the local reference outlive this callback.
        unsafe {
            first_queue.execute(
                Kind::General,
                refuse_nested,
                ptr::from_ref(&second_runtime).cast_mut().cast(),
            )
        }
        .unwrap();
        assert!(first_runtime.step().unwrap());
        assert!(!second_queue.completed(pending).unwrap());
        assert_eq!(segment.check(), Err(WorkError::InvalidTicket));
        assert_eq!(
            second_domain.hosted_dispatcher_segment().unwrap().check(),
            Ok(())
        );
        assert!(second_runtime.step().unwrap());
        assert!(second_queue.completed(pending).unwrap());

        for (queue, runtime) in [(first_queue, first_runtime), (second_queue, second_runtime)] {
            queue.close().unwrap();
            runtime.retire().unwrap();
            // SAFETY: both callbacks and all producers joined.
            unsafe { queue.retire() }.unwrap();
        }
    }

    #[test]
    fn callbacks_resume_on_their_owned_stacks_and_retire_after_return() {
        let queue = HostedQueue::new(HostedExecution::new(), Arc::new(())).unwrap();
        let stack = StackRequirement::exact(16 * 1024, 16 * 1024, 2 * PAGE).unwrap();
        // SAFETY: bounded callbacks with no shadow-stack changes or unwinds.
        let executor = unsafe { HostedExecutor::new_with_stacks(queue.clone(), stack, 2) }.unwrap();
        let runtime = executor.runtime();
        let domain = DomainRef::borrow_hosted(runtime).unwrap();
        assert_eq!(domain.dispatcher_successor(), Ok(()));
        assert!(matches!(
            domain.hosted_segment(),
            Err(WorkError::Unavailable)
        ));
        let visits = Cell::new(0);
        let first = Call {
            runtime,
            slot: 0,
            visits: &visits,
        };
        let second = Call {
            runtime,
            slot: 1,
            visits: &visits,
        };
        let notify = Call {
            runtime,
            slot: 2,
            visits: &visits,
        };
        // SAFETY: call contexts stay on the dispatcher's live stack
        // until their tickets complete, and their contents remain valid.
        let general_ticket =
            unsafe { queue.execute(Kind::General, run, ptr::from_ref(&first).cast_mut().cast()) }
                .unwrap();
        // SAFETY: same lifetime as the first General call.
        let second_ticket =
            unsafe { queue.execute(Kind::General, run, ptr::from_ref(&second).cast_mut().cast()) }
                .unwrap();
        // SAFETY: same lifetime as the General calls above.
        let notify_ticket =
            unsafe { queue.execute(Kind::Notify, run, ptr::from_ref(&notify).cast_mut().cast()) }
                .unwrap();
        queue.close().unwrap();
        assert!(runtime.step().unwrap());
        assert_eq!(visits.get(), 1);
        assert!(!queue.completed(general_ticket).unwrap());
        assert_eq!(runtime.retire(), Err(WorkError::Busy));
        assert!(runtime.step().unwrap());
        assert!(queue.completed(second_ticket).unwrap());
        assert!(runtime.step().unwrap());
        assert!(queue.completed(notify_ticket).unwrap());
        assert!(runtime.step().unwrap());
        assert!(queue.completed(general_ticket).unwrap());
        assert_eq!(visits.get(), 1234);
        assert!(!runtime.step().unwrap());
        runtime.retire().unwrap();
        assert_eq!(domain.check(), Err(WorkError::Closed));
        // SAFETY: the only producer closed and every callback returned.
        unsafe { queue.retire() }.unwrap();
    }
}
