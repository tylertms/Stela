//! Hosted source fixtures only. Real mmap stacks/guards and Runtime
//! continuations; the CPU/IRQ model grants no native admission. All C tree
//! assertions are held. This plain Rust child is not whole-C ASan/UBSan.
extern crate std;
use super::*;
use crate::owner::RawOwner;
use alloc::{boxed::Box, vec::Vec};
use core::{ops::Range, ptr, sync::atomic::AtomicPtr};
use std::{sync::Mutex, thread};
use stela_driver_base::platform::image::NativeReadOnlyImage;
use stela_driver_base::platform::work::service;
use stela_driver_base::platform::work::{Cpu, Executor, Kind, Queue, Stack, StackBacking};

#[global_allocator]
static HEAP: crate::test_heap::TestHeap = crate::test_heap::TestHeap;
unsafe extern "C" {
    fn stela_linux_task_test_taskless();
    fn stela_linux_percpu_fixture(context: *mut c_void);
    fn stela_linux_percpu_taskless_fixture();
    fn stela_linux_task_test_unprepared_begin();
    fn stela_linux_task_test_unprepared_end();
    fn stela_linux_task_test_foreground(context: *mut c_void);
    fn stela_linux_task_test_worker(context: *mut c_void);
    fn stela_linux_task_test_schedule_worker(context: *mut c_void);
    fn stela_linux_task_test_pending_schedule(context: *mut c_void);
    fn stela_linux_task_test_service_wait(context: *mut c_void);
    fn stela_linux_task_test_service_cancel(context: *mut c_void);
    fn stela_linux_task_test_clock_service(context: *mut c_void);
    fn stela_linux_task_test_runnable_boundary(context: *mut c_void);
    fn stela_linux_task_test_closed_stack();
    fn stela_linux_atomic_callback_prepared_fixture(task: *mut CTask, context: *const TaskContext);
    fn mmap(
        address: *mut c_void,
        bytes: usize,
        protection: i32,
        flags: i32,
        fd: i32,
        offset: i64,
    ) -> *mut c_void;
    fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
    fn munmap(address: *mut c_void, bytes: usize) -> i32;
}
struct MappedStack {
    base: usize,
    bytes: usize,
    usable: Range<usize>,
    dropped: Arc<AtomicU64>,
}
// SAFETY: real exclusive OS mapping, PROT_NONE guard, kept through every
// switched frame. Hosted test only; this is no native root/IRQ/provider grant.
unsafe impl StackBacking for MappedStack {
    fn bounds(&self) -> Range<usize> {
        self.usable.clone()
    }
    fn guard(&self) -> Range<usize> {
        self.usable.start - 4096..self.usable.start
    }
}
impl Drop for MappedStack {
    fn drop(&mut self) {
        // SAFETY: Runtime retired all frames/producers and detached C task users.
        assert_eq!(unsafe { munmap(self.base as *mut c_void, self.bytes) }, 0);
        self.dropped.fetch_add(1, Ordering::Relaxed);
    }
}
fn stacks(dropped: &Arc<AtomicU64>, usable: usize) -> Vec<Stack> {
    stacks_count(dropped, usable, 4)
}
fn stacks_count(dropped: &Arc<AtomicU64>, usable: usize, count: usize) -> Vec<Stack> {
    (0..count)
        .map(|_| {
            #[cfg(target_os = "macos")]
            let anonymous = 0x1000;
            #[cfg(not(target_os = "macos"))]
            let anonymous = 0x20;
            // The compiled Linux stack contract requires THREAD_ALIGN=16K,
            // independently of the OS mmap page alignment.
            let bytes = usable + 4096 + 16 * 1024;
            // SAFETY: OS-owned anonymous RW memory; no preexisting frames.
            let base = unsafe { mmap(ptr::null_mut(), bytes, 3, 2 | anonymous, -1, 0) };
            assert_ne!(base as usize, usize::MAX);
            let start = (base as usize + 4096 + 16 * 1024 - 1) & !(16 * 1024 - 1);
            // SAFETY: the page preceding aligned usable storage lies within
            // this larger owned mapping and becomes its real stack guard.
            assert_eq!(
                // SAFETY: this checked page lies in the larger owned mapping above.
                unsafe { mprotect((start - 4096) as *mut c_void, 4096, 0) },
                0
            );
            Stack::new(Box::new(MappedStack {
                base: base as usize,
                bytes,
                usable: start..start + usable,
                dropped: dropped.clone(),
            }))
            .unwrap()
        })
        .collect()
}
struct CpuModel {
    thread: thread::ThreadId,
    flags: AtomicU64,
    irq: AtomicBool,
    time: AtomicU64,
}
impl CpuModel {
    fn new() -> Arc<Self> {
        Arc::new(Self {
            thread: thread::current().id(),
            flags: AtomicU64::new(0x202),
            irq: AtomicBool::new(false),
            time: AtomicU64::new(0),
        })
    }
}
// SAFETY: explicit hosted same-thread exclusion/IRQ model. No CPL0 CLI/STI or
// real native producer is claimed. Runtime still uses switched stacks.
unsafe impl Cpu for CpuModel {
    fn check_cpu(&self) -> Result<(), WorkError> {
        if self.thread == thread::current().id() {
            Ok(())
        } else {
            Err(WorkError::WrongCpu)
        }
    }
    fn in_interrupt(&self) -> bool {
        self.irq.load(Ordering::Acquire)
    }
    fn save_disable(&self) -> Result<u64, WorkError> {
        self.check_cpu()?;
        Ok(self.flags.swap(2, Ordering::AcqRel))
    }
    unsafe fn restore(&self, flags: u64) {
        self.flags.store(flags, Ordering::Release);
    }
    fn ticks(&self) -> u64 {
        self.time.fetch_add(1, Ordering::Relaxed)
    }
    fn fatal(&self, error: WorkError) -> ! {
        std::eprintln!(
            "fatal work error {error:?}
{}",
            std::backtrace::Backtrace::force_capture()
        );
        std::process::abort()
    }
}
struct Fixture {
    cpu: Arc<CpuModel>,
    runtime: AtomicPtr<Runtime<16>>,
    bindings: [Mutex<Option<Arc<TaskBinding<16>>>>; 3],
    service_target: Mutex<Option<TargetTask<16>>>,
    scope_target: Mutex<Option<TargetTask<16>>>,
    service_domain: Mutex<Option<Arc<service::Domain<16>>>>,
    ticks: AtomicU64,
    tick_goal: AtomicU64,
    boundary_mode: AtomicU64,
    boundary_ready: AtomicBool,
    clock_reads: Arc<AtomicU64>,
    wake: Mutex<Option<WakeHandle<16>>>,
    late: Mutex<Option<WakeHandle<16>>>,
    trees: AtomicU64,
    refused: AtomicBool,
}
impl Fixture {
    fn new(cpu: Arc<CpuModel>) -> Arc<Self> {
        Arc::new(Self {
            cpu,
            runtime: AtomicPtr::new(ptr::null_mut()),
            bindings: core::array::from_fn(|_| Mutex::new(None)),
            service_target: Mutex::new(None),
            scope_target: Mutex::new(None),
            service_domain: Mutex::new(None),
            ticks: AtomicU64::new(0),
            tick_goal: AtomicU64::new(2),
            boundary_mode: AtomicU64::new(0),
            boundary_ready: AtomicBool::new(false),
            clock_reads: Arc::new(AtomicU64::new(0)),
            wake: Mutex::new(None),
            late: Mutex::new(None),
            trees: AtomicU64::new(0),
            refused: AtomicBool::new(false),
        })
    }
    unsafe fn runtime(&self) -> Pin<&Runtime<16>> {
        self.cpu.check_cpu().unwrap();
        // SAFETY: harness owns pinned executor through all callbacks/retirement;
        // Queue keeps this fixture. This private process-only helper cannot
        // escape into an IRQ waker; production run_scoped keeps a real borrow.
        unsafe { Pin::new_unchecked(&*self.runtime.load(Ordering::Acquire)) }
    }
    fn binding(&self, index: usize) -> Arc<TaskBinding<16>> {
        if let Some(binding) = self.bindings[index].lock().unwrap().clone() {
            return binding;
        }
        // SAFETY: hosted checked process thread before Linux entry.
        let binding = unsafe { TaskBinding::new() }.unwrap();
        // SAFETY: the harness keeps the pinned executor through every callback and
        // retirement.
        binding.bind_current(unsafe { self.runtime() }).unwrap();
        *self.bindings[index].lock().unwrap() = Some(binding.clone());
        binding
    }
}
unsafe fn fixture<'a>(context: *mut c_void) -> &'a Fixture {
    // SAFETY: Queue owner keeps this private fixture through callback.
    unsafe { &*context.cast::<Fixture>() }
}
unsafe extern "C" fn worker(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    let binding = f.binding(1);
    // SAFETY: shared Runtime/fixture live through complete scoped C body.
    unsafe { binding.run_scoped(f.runtime(), stela_linux_task_test_worker, context) }.unwrap();
    let wake = f.wake.lock().unwrap().take().unwrap();
    wake.wake().unwrap();
    // SAFETY: the same real worker now exercises original
    // schedule's distinct process preempt0 sleep/wake boundary.
    unsafe { binding.run_scoped(f.runtime(), stela_linux_task_test_schedule_worker, context) }
        .unwrap();
}
unsafe extern "C" fn unprepared(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    // SAFETY: the C test hook only checks task state on this thread's entered continuation.
    unsafe { stela_linux_task_test_unprepared_begin() };
    let before = runtime.current_context().unwrap();
    // Direct generic yield cannot silently suspend an entered C lease. A
    // refusal keeps the SAME task, native flags and service owner.
    assert_eq!(runtime.yield_now(), Err(WorkError::Busy));
    assert_eq!(
        runtime.current_context().unwrap().context_id,
        before.context_id
    );
    assert_eq!(cpu_flags(f), 0x202);
    // SAFETY: the C test hook only checks task state on this thread's entered continuation.
    unsafe { stela_linux_task_test_unprepared_end() };
}
fn cpu_flags(f: &Fixture) -> u64 {
    f.cpu.flags.load(Ordering::Acquire)
}
unsafe extern "C" fn foreground(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    let binding = f.binding(0);
    let prepared_view = runtime.current_context().unwrap();
    // SAFETY: this real attached current has no Linux owner/scope yet. The
    // C fixture keeps its own scope and checks before-effect refusal.
    unsafe { stela_linux_atomic_callback_prepared_fixture(binding.task.as_ptr(), &prepared_view) };
    let wake = binding.current_wake().unwrap();
    *f.late.lock().unwrap() = Some(wake.clone());
    *f.wake.lock().unwrap() = Some(wake);
    // Worker executes at the foreground's first scheduling checkpoint, after
    // it released its prepared owner lease. It wakes this exact old job.
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { runtime.queue().execute(Kind::General, worker, context) }.unwrap();
    // Refuse an unprepared entered lease without leaving or changing identity.
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // through every suspension.
    unsafe { binding.run_scoped(runtime, unprepared, context) }.unwrap();
    let view = runtime.current_context().unwrap();
    // Additional C fixture checks all entered IRQ/BH/RCU/preempt refusals.
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // through every suspension.
    unsafe { binding.run_scoped(runtime, stela_linux_task_test_foreground, context) }.unwrap();
    assert_eq!(
        runtime.current_context().unwrap().context_id,
        view.context_id
    );
    binding.current_wake().unwrap().wake().unwrap();
    // SAFETY: this job and scope keep the pending generic wake;
    // the C fixture checks schedule's state restoration without a switch.
    unsafe { binding.run_scoped(runtime, stela_linux_task_test_pending_schedule, context) }
        .unwrap();
}
unsafe extern "C" fn trees_body(_: *mut c_void) {
    // SAFETY: reused foreground job, old worker metadata kept.
    unsafe { stela_linux_task_test_closed_stack() };
    // SAFETY: same real attached task and RunScoped body; this callable
    // fixture enters/exits its own owner and keeps no callback/context.
    unsafe { stela_linux_percpu_fixture(ptr::null_mut()) };
}
unsafe extern "C" fn trees(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    let binding = f.binding(0);
    // SAFETY: scope keeps shared Runtime and all owner/model backing.
    unsafe { binding.run_scoped(f.runtime(), trees_body, context) }.unwrap();
    f.trees.fetch_add(1, Ordering::Relaxed);
}
unsafe extern "C" fn reject_large_stack(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: process context on the runtime CPU with the hosted heap initialized.
    let binding = unsafe { TaskBinding::<16>::new() }.unwrap();
    assert_eq!(
        // SAFETY: the harness keeps the pinned executor through every callback and
        // retirement.
        binding.bind_current(unsafe { f.runtime() }),
        Err(WorkError::CorruptContext)
    );
    // Rejected initial attach installed no C task or binding; unique metadata
    // rollback occurs here in process context, never IF=0 hooks.
    // SAFETY: the runtime detached this binding and no job or waker still uses it.
    assert!(unsafe { binding.release_quiescent() }.is_ok());
    f.refused.store(true, Ordering::Release);
}

/// Invoked in its own current unit-test executable, so C globals and
/// permanent canonical registry cannot race parallel parent unit fixtures.
pub(crate) fn run_trees_child() {
    assert_eq!(
        super::stack_requirement(),
        stela_driver_base::platform::work::StackRequirement::exact(16 * 1024, 16 * 1024, 4096)
    );
    let executable = std::env::current_exe().unwrap();
    let output = std::process::Command::new(executable)
        .args([
            "--exact",
            "task::tests::trees_child",
            "--ignored",
            "--test-threads=1",
            "--nocapture",
            "--format=pretty",
            "--color=never",
        ])
        .output()
        .unwrap();
    let stdout = std::string::String::from_utf8_lossy(&output.stdout);
    let stderr = std::string::String::from_utf8_lossy(&output.stderr);
    // A missing/cfg-excluded exact selector exits successfully with zero
    // tests. Require named execution AND exactly one passing test.
    let named = stdout
        .lines()
        .any(|line| line == "test task::tests::trees_child ... ok");
    let one_passed = stdout
        .lines()
        .any(|line| line.starts_with("test result: ok. 1 passed; 0 failed; 0 ignored;"));
    assert!(
        output.status.success() && named && one_passed,
        "task-aware tree child did not pass exactly once ({}):\n{}{}",
        output.status,
        stdout,
        stderr
    );
}
#[test]
#[ignore = "isolation: child"]
fn trees_child() {
    // SAFETY: isolated single test child; same canonical GlobalAlloc owns this
    // real raw span and ordinary System storage until exact-layout free.
    assert!(crate::test_heap::register());
    // SAFETY: the C test hook only checks task state on this thread's entered continuation.
    unsafe { stela_linux_task_test_taskless() };
    // SAFETY: taskless dispatcher, before any TaskBinding attach;
    // the observer requires a trap before backing allocation is attempted.
    unsafe { stela_linux_percpu_taskless_fixture() };
    let dropped = Arc::new(AtomicU64::new(0));
    // A larger generic Runtime stack is real, but not this Linux THREAD_SIZE.
    {
        let cpu = CpuModel::new();
        let f = Fixture::new(cpu.clone());
        // SAFETY: the fixture is the queue's owner, and only this test thread produces
        // work.
        let queue = unsafe { Queue::<16>::new(cpu, f.clone()) }.unwrap();
        let executor = Executor::new_hosted_for_test(queue, stacks(&dropped, 32 * 1024)).unwrap();
        let runtime = executor.runtime();
        f.runtime.store(
            runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
            Ordering::Release,
        );
        // SAFETY: the callbacks are static, never unwind, and take the fixture the queue
        // keeps until completion.
        unsafe {
            runtime.call_foreground(reject_large_stack, Arc::as_ptr(&f) as *mut c_void, 10000)
        }
        .unwrap();
        assert!(f.refused.load(Ordering::Acquire));
        runtime.queue().close().unwrap();
        // SAFETY: the queue is closed and this test thread is the only producer; pending work
        // is a checked refusal.
        unsafe { runtime.retire() }.unwrap();
    }
    assert_eq!(dropped.load(Ordering::Relaxed), 4);
    let cpu = CpuModel::new();
    let f = Fixture::new(cpu.clone());
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
    let executor = Executor::new_hosted_for_test(queue, stacks(&dropped, 16 * 1024)).unwrap();
    let runtime = executor.runtime();
    f.runtime.store(
        runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
        Ordering::Release,
    );
    let context = Arc::as_ptr(&f) as *mut c_void;
    assert_eq!(runtime.current_context(), Err(WorkError::Unavailable));
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { runtime.call_foreground(foreground, context, 10000) }.unwrap();
    assert_eq!(
        f.late.lock().unwrap().as_ref().unwrap().wake(),
        Err(WorkError::Closed)
    );
    let first = f.bindings[0]
        .lock()
        .unwrap()
        .as_ref()
        .unwrap()
        .context_id()
        .unwrap();
    // Reuse the SAME continuation/C task with a new existing job ticket.
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { runtime.call_foreground(trees, context, 10000) }.unwrap();
    assert_eq!(f.trees.load(Ordering::Relaxed), 1);
    assert_eq!(
        f.bindings[0]
            .lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .context_id()
            .unwrap(),
        first
    );
    assert_eq!(
        f.late.lock().unwrap().as_ref().unwrap().wake(),
        Err(WorkError::Closed)
    );
    assert!(!crate::test_heap::span_occupied());
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    runtime.queue().close().unwrap();
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    assert_eq!(unsafe { runtime.retire() }, Err(WorkError::Pending));
    for slot in &f.bindings {
        let Some(binding) = slot.lock().unwrap().take() else {
            continue;
        };
        // SAFETY: both callbacks completed, no C waiter/foreign producer;
        // no registered IRQ producer or C task reference escapes this child.
        let retained = unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
        assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
        drop(retained); // after flags restoration; break the binding/Queue cycle
        // SAFETY: the runtime detached this binding and no job or waker still uses it.
        assert!(unsafe { binding.release_quiescent() }.is_ok());
    }
    f.late.lock().unwrap().take();
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { runtime.retire() }.unwrap();
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 8);
    assert_eq!(quarantined(), 0);
    // SAFETY: the C test hook only checks task state on this thread's entered continuation.
    unsafe { stela_linux_task_test_taskless() };
    service_roundtrip();
    clock_service_roundtrip(0);
    clock_service_roundtrip(1);
    clock_service_roundtrip(2);
    clock_service_roundtrip(3);
    // SAFETY: this child borrows no constant Linux names, which classifying
    // no address as immutable data requires.
    assert!(unsafe { crate::image::register(alloc::sync::Arc::new(NoImage)) }.is_ok());
    stack_catalog_capacity();
}

struct NoImage;

// SAFETY: no address is classified as immutable data, and none is read.
unsafe impl NativeReadOnlyImage for NoImage {
    fn contains(&self, _address: usize) -> bool {
        false
    }
}

unsafe extern "C" fn service_wait(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    assert_eq!(runtime.service_current(), Ok(true));
    let binding = f.binding(2);
    *f.service_target.lock().unwrap() = Some(binding.target_current().unwrap());
    // SAFETY: reserved continuation and kept fixture/code roots.
    unsafe { binding.run_scoped(runtime, stela_linux_task_test_service_wait, context) }.unwrap();
}
unsafe extern "C" fn service_cancel_and_wake(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    let target = f.service_target.lock().unwrap().as_ref().unwrap().clone();
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    assert_eq!(
        // SAFETY: callback from the entered same-CPU foreground C owner.
        unsafe { target.signal(runtime, WaitSignal::Interrupt) },
        Err(WorkError::Unavailable)
    );
    assert_eq!(
        // SAFETY: this is the entered same-CPU continuation inside `run_scoped`, with the
        // service roots kept.
        unsafe { target.signal(runtime, WaitSignal::Fatal) },
        Err(WorkError::Unavailable)
    );
    target.wake.wake().unwrap();
}
#[repr(C)]
struct ServiceCall {
    context: *mut c_void,
    cancel_and_wake: unsafe extern "C" fn(*mut c_void),
}
unsafe extern "C" fn service_caller(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    assert_eq!(runtime.service_current(), Ok(false));
    let binding = f.binding(0);
    let mut call = ServiceCall {
        context,
        cancel_and_wake: service_cancel_and_wake,
    };
    // SAFETY: stack-local call and real scope remain live through C return.
    unsafe {
        binding.run_scoped(
            runtime,
            stela_linux_task_test_service_cancel,
            (&mut call as *mut ServiceCall).cast(),
        )
    }
    .unwrap();
}
fn service_roundtrip() {
    let cpu = CpuModel::new();
    let f = Fixture::new(cpu.clone());
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
    let dropped = Arc::new(AtomicU64::new(0));
    let executor =
        Executor::new_hosted_service_for_test(queue, stacks_count(&dropped, 16 * 1024, 5)).unwrap();
    let runtime = executor.runtime();
    f.runtime.store(
        runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
        Ordering::Release,
    );
    let context = Arc::as_ptr(&f) as *mut c_void;
    // SAFETY: fixture roots keep both callbacks through join below.
    let service = unsafe { runtime.start_service(service_wait, context) }.unwrap();
    assert!(runtime.step().unwrap());
    assert!(!runtime.queue().completed(service).unwrap());
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { runtime.call_foreground(service_caller, context, 10000) }.unwrap();
    runtime.drain(10000).unwrap();
    assert!(runtime.queue().completed(service).unwrap());
    assert_eq!(
        // SAFETY: the queue owner keeps the fixture, and this service binds its task before
        // any admission.
        unsafe { runtime.start_service(service_wait, context) },
        Err(WorkError::Busy)
    );
    let target = f.service_target.lock().unwrap().take().unwrap();
    assert_eq!(target.wake.wake(), Err(WorkError::Closed));
    drop(target);
    runtime.queue().close().unwrap();
    for slot in &f.bindings {
        let Some(binding) = slot.lock().unwrap().take() else {
            continue;
        };
        // SAFETY: all real service/caller frames completed; no C/IRQ producer.
        let retained = unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
        drop(retained);
        // SAFETY: the runtime detached this binding and no job or waker still uses it.
        assert!(unsafe { binding.release_quiescent() }.is_ok());
    }
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { runtime.retire() }.unwrap();
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 5);
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    assert_eq!(quarantined(), 0);
}

struct ServiceClock {
    cpu: Arc<dyn Cpu>,
    reads: Arc<AtomicU64>,
    start: std::time::Instant,
}
// SAFETY: hosted monotonic clock; no native IRQ attestation.
unsafe impl service::Clock for ServiceClock {
    fn cpu(&self) -> &Arc<dyn Cpu> {
        &self.cpu
    }
    fn now_ns(&self) -> Result<u64, WorkError> {
        self.cpu.check_cpu()?;
        self.reads.fetch_add(1, Ordering::Relaxed);
        self.start
            .elapsed()
            .as_nanos()
            .try_into()
            .map_err(|_| WorkError::Exhausted)
    }
}
struct ServiceIdle {
    clock: Arc<dyn service::Clock>,
    owner: thread::Thread,
    waits: AtomicU64,
}
// SAFETY: hosted park/unpark event token; no native ISR is admitted.
// Unpark is held across check-to-park and never runs a callback inline.
unsafe impl service::Idle for ServiceIdle {
    fn clock(&self) -> &Arc<dyn service::Clock> {
        &self.clock
    }
    fn notify(&self) -> Result<(), WorkError> {
        self.owner.unpark();
        Ok(())
    }
    fn wait_until_ns(&self, deadline: Option<u64>) -> Result<(), WorkError> {
        self.clock.cpu().check_cpu()?;
        self.waits.fetch_add(1, Ordering::Relaxed);
        if let Some(deadline) = deadline {
            let now = self.clock.now_ns()?;
            if now < deadline {
                thread::park_timeout(std::time::Duration::from_nanos(deadline - now));
            }
        } else {
            thread::park();
        }
        Ok(())
    }
}
#[repr(C)]
struct ClockServiceCall {
    context: *mut c_void,
    boundary: unsafe extern "C" fn(*mut c_void) -> bool,
}
unsafe extern "C" fn clock_boundary(context: *mut c_void) -> bool {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    assert_eq!(runtime.service_current(), Ok(true));
    let domain = f.service_domain.lock().unwrap().as_ref().unwrap().clone();
    if let Some(batch) = domain.begin(runtime).unwrap() {
        assert!(batch.tick() && !batch.raised() && !batch.timer());
        if f.boundary_mode.load(Ordering::Relaxed) == 0 || f.boundary_ready.load(Ordering::Acquire)
        {
            f.ticks.fetch_add(1, Ordering::Relaxed);
        }
        if f.boundary_mode.load(Ordering::Relaxed) == 2 {
            let target = f.scope_target.lock().unwrap().as_ref().unwrap().clone();
            assert_eq!(
                // SAFETY: same-CPU entered service; target has returned its
                // body and is in its private scope-end switch.
                unsafe { target.signal(runtime, WaitSignal::Interrupt) },
                Err(WorkError::Busy)
            );
            assert_eq!(
                // SAFETY: this is the entered same-CPU continuation inside `run_scoped`, with
                // the service roots kept.
                unsafe { target.signal(runtime, WaitSignal::Fatal) },
                Err(WorkError::Busy)
            );
            target.wake.wake().unwrap();
        }
        // SAFETY: reserved task delivered this fixture's counter
        // consumer. No original IRQ-work/timer storage is admitted by it.
        unsafe { domain.finish(runtime, batch, None) }.unwrap();
    }
    f.ticks.load(Ordering::Relaxed) >= f.tick_goal.load(Ordering::Relaxed)
}
unsafe extern "C" fn clock_service(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    let binding = f.binding(2);
    let domain = f.service_domain.lock().unwrap().as_ref().unwrap().clone();
    domain.bind_current(runtime).unwrap();
    let mut call = ClockServiceCall {
        context,
        boundary: clock_boundary,
    };
    // SAFETY: real kept C task/stack and stack-local callback descriptor.
    unsafe {
        binding.run_scoped(
            runtime,
            stela_linux_task_test_clock_service,
            (&mut call as *mut ClockServiceCall).cast(),
        )
    }
    .unwrap();
    domain.stop_producers().unwrap();
}
#[repr(C)]
struct RunnableBoundaryCall {
    context: *mut c_void,
    mode: u32,
    make_due: unsafe extern "C" fn(*mut c_void),
    reads: unsafe extern "C" fn(*mut c_void) -> u64,
    ticks: unsafe extern "C" fn(*mut c_void) -> u64,
}
unsafe extern "C" fn boundary_make_due(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    f.boundary_ready.store(true, Ordering::Release);
    // Real elapsed hosted time, with the Runtime task still Running.
    // This invokes no Runtime park/yield, fake tick or service callback.
    thread::sleep(std::time::Duration::from_millis(2));
}
unsafe extern "C" fn boundary_reads(context: *mut c_void) -> u64 {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    unsafe { fixture(context) }
        .clock_reads
        .load(Ordering::Relaxed)
}
unsafe extern "C" fn boundary_ticks(context: *mut c_void) -> u64 {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    unsafe { fixture(context) }.ticks.load(Ordering::Relaxed)
}
unsafe extern "C" fn runnable_boundary(context: *mut c_void) {
    // SAFETY: the queue owner keeps the fixture behind `context` through this callback.
    let f = unsafe { fixture(context) };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    let binding = f.binding(0);
    if f.boundary_mode.load(Ordering::Relaxed) == 2 {
        *f.scope_target.lock().unwrap() = Some(binding.target_current().unwrap());
    }
    let mut call = RunnableBoundaryCall {
        context,
        mode: f.boundary_mode.load(Ordering::Relaxed) as u32,
        make_due: boundary_make_due,
        reads: boundary_reads,
        ticks: boundary_ticks,
    };
    // SAFETY: same-CPU C scope keeps descriptor and providers.
    unsafe {
        binding.run_scoped(
            runtime,
            stela_linux_task_test_runnable_boundary,
            (&mut call as *mut RunnableBoundaryCall).cast(),
        )
    }
    .unwrap();
    assert_eq!(
        f.ticks.load(Ordering::Relaxed),
        1,
        "cond_resched/scope-end must deliver before this runnable task returns"
    );
    f.scope_target.lock().unwrap().take();
}
fn clock_service_roundtrip(mode: u64) {
    let cpu = CpuModel::new();
    let f = Fixture::new(cpu.clone());
    f.boundary_mode.store(mode, Ordering::Relaxed);
    f.tick_goal
        .store(if mode == 0 { 2 } else { 1 }, Ordering::Relaxed);
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
    let dropped = Arc::new(AtomicU64::new(0));
    let executor =
        Executor::new_hosted_service_for_test(queue, stacks_count(&dropped, 16 * 1024, 5)).unwrap();
    let runtime = executor.runtime();
    f.runtime.store(
        runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
        Ordering::Release,
    );
    let clock: Arc<dyn service::Clock> = Arc::new(ServiceClock {
        cpu: cpu.clone(),
        reads: f.clock_reads.clone(),
        start: std::time::Instant::now(),
    });
    let idle = Arc::new(ServiceIdle {
        clock: clock.clone(),
        owner: thread::current(),
        waits: AtomicU64::new(0),
    });
    let domain = service::Domain::new(clock, idle.clone()).unwrap();
    runtime.attach_service_domain(domain.clone()).unwrap();
    *f.service_domain.lock().unwrap() = Some(domain.clone());
    let context = Arc::as_ptr(&f) as *mut c_void;
    // SAFETY: fixture and registered domain keep all code/providers.
    let ticket = unsafe { runtime.start_service(clock_service, context) }.unwrap();
    if mode == 0 {
        let mut attempts = 0;
        while !runtime.queue().completed(ticket).unwrap() {
            assert!(
                attempts < 32,
                "real service did not complete after bounded event waits"
            );
            runtime.step_or_idle().unwrap();
            attempts += 1;
        }
        assert!(idle.waits.load(Ordering::Relaxed) > 0);
        assert_eq!(
            cpu.time.load(Ordering::Relaxed),
            0,
            "Cpu::ticks is not nanoseconds"
        );
    } else {
        assert!(runtime.step().unwrap()); // service setup/block
        // SAFETY: rooted caller; it must remain Runtime-runnable.
        unsafe { runtime.call_foreground(runnable_boundary, context, 10000) }.unwrap();
        assert!(runtime.queue().completed(ticket).unwrap());
        assert_eq!(idle.waits.load(Ordering::Relaxed), 0);
    }
    assert_eq!(
        f.ticks.load(Ordering::Relaxed),
        f.tick_goal.load(Ordering::Relaxed)
    );
    domain.stop_producers().unwrap();
    // SAFETY: service job completed, no original C timer/work/storage
    // was admitted; the fixture consumer has no outstanding callback/user.
    unsafe { domain.retire() }.unwrap();
    for slot in &f.bindings {
        let Some(binding) = slot.lock().unwrap().take() else {
            continue;
        };
        // SAFETY: service and caller jobs/consumers are now joined.
        let retained = unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
        drop(retained);
        // SAFETY: the runtime detached this binding and no job or waker still uses it.
        assert!(unsafe { binding.release_quiescent() }.is_ok());
    }
    runtime.queue().close().unwrap();
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { runtime.retire() }.unwrap();
    f.service_domain.lock().unwrap().take();
    drop(domain);
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 5);
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    assert_eq!(quarantined(), 0);
}

/// The C catalog's capacity, which one executor's stacks fill.
const CATALOG: usize = stela_driver_base::platform::work::MAX_CONTINUATIONS;
/// Jobs for a full catalog's executor: its stacks and the protected GPE job.
const CATALOG_JOBS: usize = 2 * CATALOG;
use stela_driver_base::platform::work::ROLE_STACKS;
struct CatalogFixture {
    cpu: Arc<CpuModel>,
    runtime: AtomicPtr<Runtime<CATALOG_JOBS>>,
    bindings: Mutex<Vec<Arc<TaskBinding<CATALOG_JOBS>>>>,
    refused: AtomicBool,
}
impl CatalogFixture {
    fn new(cpu: Arc<CpuModel>) -> Arc<Self> {
        Arc::new(Self {
            cpu,
            runtime: AtomicPtr::new(ptr::null_mut()),
            bindings: Mutex::new(Vec::new()),
            refused: AtomicBool::new(false),
        })
    }
    unsafe fn runtime(&self) -> Pin<&Runtime<CATALOG_JOBS>> {
        self.cpu.check_cpu().unwrap();
        // SAFETY: fixture owner keeps this pinned runtime through
        // all admitted callbacks, detach and process-context metadata free.
        unsafe { Pin::new_unchecked(&*self.runtime.load(Ordering::Acquire)) }
    }
    fn publish(&self, runtime: Pin<&Runtime<CATALOG_JOBS>>) {
        self.runtime.store(
            runtime.get_ref() as *const Runtime<CATALOG_JOBS> as *mut Runtime<CATALOG_JOBS>,
            Ordering::Release,
        );
    }
    fn detach(&self, runtime: Pin<&Runtime<CATALOG_JOBS>>) {
        let bindings = core::mem::take(&mut *self.bindings.lock().unwrap());
        for binding in bindings {
            // SAFETY: all real jobs completed, no source/storage/foreign refs.
            let retained =
                unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
            drop(retained);
            // SAFETY: the runtime detached this binding and no job or waker still uses it.
            assert!(unsafe { binding.release_quiescent() }.is_ok());
        }
    }
}
unsafe extern "C" fn catalog_bind(context: *mut c_void) {
    // SAFETY: rooted fixture owns this pointer through completion.
    let f = unsafe { &*context.cast::<CatalogFixture>() };
    // SAFETY: process context on the runtime CPU with the hosted heap initialized.
    let binding = unsafe { TaskBinding::<CATALOG_JOBS>::new() }.unwrap();
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    binding.bind_current(unsafe { f.runtime() }).unwrap();
    f.bindings.lock().unwrap().push(binding);
}
unsafe extern "C" fn catalog_refused(context: *mut c_void) {
    // SAFETY: same rooted fixture/stack, with a full original catalog.
    let f = unsafe { &*context.cast::<CatalogFixture>() };
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    let before = runtime.current_context().unwrap();
    // SAFETY: process context on the runtime CPU with the hosted heap initialized.
    let binding = unsafe { TaskBinding::<CATALOG_JOBS>::new() }.unwrap();
    assert_eq!(binding.bind_current(runtime), Err(WorkError::Full));
    assert_eq!(runtime.current_context(), Ok(before));
    // SAFETY: admission failed before original metadata/publication effects.
    assert!(unsafe { binding.release_quiescent() }.is_ok());
    f.refused.store(true, Ordering::Release);
}
fn stack_catalog_capacity() {
    let cpu = CpuModel::new();
    let dropped = Arc::new(AtomicU64::new(0));
    let first = CatalogFixture::new(cpu.clone());
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<CATALOG_JOBS>::new(cpu.clone(), first.clone()) }.unwrap();
    let executor = Executor::new_hosted_policy_for_test(
        queue,
        stacks_count(&dropped, 16 * 1024, CATALOG),
        stela_driver_base::platform::work::ReservationPolicy::NativeFirmwareProtected {
            gpe: stela_driver_base::platform::work::GpeQueueBudget::QUALIFICATION,
        },
    )
    .unwrap();
    let runtime = executor.runtime();
    first.publish(runtime);
    let context = Arc::as_ptr(&first) as *mut c_void;
    // SAFETY: each role/worker callback borrows first, kept by queue through detach.
    unsafe { runtime.start_service(catalog_bind, context) }.unwrap();
    // SAFETY: independent control worker, same immutable fixture owner.
    unsafe { runtime.start_firmware(catalog_bind, context) }.unwrap();
    // SAFETY: serial GPE has one mapped worker; its callback owner survives join.
    unsafe { runtime.queue().execute_gpe(catalog_bind, context) }.unwrap();
    for _ in 0..2 {
        // SAFETY: each accepted Notify permit owns a distinct stack.
        unsafe { runtime.queue().execute(Kind::Notify, catalog_bind, context) }.unwrap();
        // SAFETY: each accepted Portal permit owns a distinct stack.
        unsafe { runtime.queue().execute_portal(catalog_bind, context) }.unwrap();
    }
    for _ in 0..CATALOG - ROLE_STACKS {
        // SAFETY: each General callback keeps first and binds one mapped continuation.
        unsafe {
            runtime
                .queue()
                .execute(Kind::General, catalog_bind, context)
        }
        .unwrap();
    }
    // SAFETY: Foreground's mapped stack is distinct from all its peers above.
    unsafe { runtime.call_foreground(catalog_bind, context, 10000) }.unwrap();
    runtime.drain(10000).unwrap();
    let mut ids: Vec<_> = first
        .bindings
        .lock()
        .unwrap()
        .iter()
        .map(|binding| binding.context_id().unwrap().get())
        .collect();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), CATALOG);
    let second = CatalogFixture::new(cpu.clone());
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<CATALOG_JOBS>::new(cpu.clone(), second.clone()) }.unwrap();
    let refused = Executor::new_hosted_for_test(queue, stacks(&dropped, 16 * 1024)).unwrap();
    let rejected = refused.runtime();
    second.publish(rejected);
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe {
        rejected.call_foreground(catalog_refused, Arc::as_ptr(&second) as *mut c_void, 10000)
    }
    .unwrap();
    assert!(second.refused.load(Ordering::Acquire));
    rejected.queue().close().unwrap();
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { rejected.retire() }.unwrap();
    drop(refused);
    runtime.queue().close().unwrap();
    first.detach(runtime);
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { runtime.retire() }.unwrap();
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), CATALOG as u64 + 4);
    // Every retired slot is available to fresh mappings again.
    let third = CatalogFixture::new(cpu.clone());
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<CATALOG_JOBS>::new(cpu.clone(), third.clone()) }.unwrap();
    let reused = Executor::new_hosted_for_test(queue, stacks(&dropped, 16 * 1024)).unwrap();
    let available = reused.runtime();
    third.publish(available);
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { available.call_foreground(catalog_bind, Arc::as_ptr(&third) as *mut c_void, 10000) }
        .unwrap();
    assert_eq!(third.bindings.lock().unwrap().len(), 1);
    available.queue().close().unwrap();
    third.detach(available);
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { available.retire() }.unwrap();
    drop(reused);
    assert_eq!(dropped.load(Ordering::Relaxed), CATALOG as u64 + 8);
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    assert_eq!(quarantined(), 0);
}

#[repr(C)]
struct MutexCall {
    scenario: u32,
    role: u32,
    context: *mut c_void,
    checkpoint: unsafe extern "C" fn(*mut c_void, u32),
}
unsafe extern "C" {
    fn stela_linux_mutex_test_run(context: *mut c_void);
    fn stela_linux_rt_mutex_test_run(context: *mut c_void);
}
struct MutexRole {
    fixture: alloc::sync::Weak<MutexFixture>,
    scenario: u32,
    role: usize,
}
struct MutexFixture {
    cpu: Arc<CpuModel>,
    runtime: AtomicPtr<Runtime<16>>,
    bindings: [Mutex<Option<Arc<TaskBinding<16>>>>; 5],
    targets: [Mutex<Option<TargetTask<16>>>; 5],
    jobs: [Box<MutexRole>; 5],
    completed: AtomicU64,
    signals: AtomicU64,
    stale: AtomicU64,
    self_signals: AtomicU64,
}
impl MutexFixture {
    fn new(cpu: Arc<CpuModel>, scenario: u32) -> Arc<Self> {
        Arc::new_cyclic(|weak| Self {
            cpu,
            runtime: AtomicPtr::new(ptr::null_mut()),
            bindings: core::array::from_fn(|_| Mutex::new(None)),
            targets: core::array::from_fn(|_| Mutex::new(None)),
            // All stable callback storage is allocated before publication;
            // neither original wake_q nor PREPARE callbacks allocate it.
            jobs: core::array::from_fn(|role| {
                Box::new(MutexRole {
                    fixture: weak.clone(),
                    scenario,
                    role,
                })
            }),
            completed: AtomicU64::new(0),
            signals: AtomicU64::new(0),
            stale: AtomicU64::new(0),
            self_signals: AtomicU64::new(0),
        })
    }
    unsafe fn runtime(&self) -> Pin<&Runtime<16>> {
        self.cpu.check_cpu().unwrap();
        // SAFETY: this private hosted fixture's executor owns the
        // exact pinned Runtime until all callbacks end and bindings detach.
        unsafe { Pin::new_unchecked(&*self.runtime.load(Ordering::Acquire)) }
    }
    fn binding(&self, index: usize) -> Arc<TaskBinding<16>> {
        if let Some(binding) = self.bindings[index].lock().unwrap().clone() {
            return binding;
        }
        // SAFETY: real process stack/CPU, outside entered C/IF0 hooks.
        let binding = unsafe { TaskBinding::new() }.unwrap();
        // SAFETY: the harness keeps the pinned executor through every callback and
        // retirement.
        binding.bind_current(unsafe { self.runtime() }).unwrap();
        *self.bindings[index].lock().unwrap() = Some(binding.clone());
        binding
    }
    fn context(&self, role: usize) -> *mut c_void {
        &*self.jobs[role] as *const MutexRole as *mut c_void
    }
    fn target(&self, role: usize) -> TargetTask<16> {
        // This is also used in an original RCU process-retirement callback.
        // Refuse contention instead of blocking at SOFTIRQ_OFFSET; cloning
        // only increments kept owners, with nonfinal drops afterward.
        self.targets[role]
            .try_lock()
            .unwrap()
            .as_ref()
            .unwrap()
            .clone()
    }
    fn enqueue(&self, role: usize) {
        let lane = if role == 1 || role == 4 {
            Kind::Notify
        } else {
            Kind::General
        };
        // SAFETY: fixture owns stable job storage until every real callback
        // completes. Enqueue is bounded, with no callback or heap allocation.
        unsafe {
            self.runtime()
                .queue()
                .execute(lane, mutex_role, self.context(role))
        }
        .unwrap();
    }
}
unsafe fn mutex_job<'a>(context: *mut c_void) -> &'a MutexRole {
    // SAFETY: stable boxed role kept by the Queue fixture owner.
    unsafe { &*context.cast::<MutexRole>() }
}
unsafe extern "C" fn mutex_checkpoint(context: *mut c_void, event: u32) {
    // SAFETY: the queue owner keeps this boxed role through the callback.
    let job = unsafe { mutex_job(context) };
    let f = job.fixture.upgrade().unwrap();
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    match event {
        1 => {
            f.enqueue(1);
            if job.scenario == 0 {
                f.enqueue(2);
                f.enqueue(3);
            } else if job.scenario == 11 {
                f.enqueue(2);
            }
        }
        9 => {
            assert_eq!(job.scenario, 12);
            f.enqueue(2);
            f.enqueue(3);
        }
        2 | 3 => {
            let target = f.target(1);
            let kind = if event == 3 {
                WaitSignal::Fatal
            } else {
                WaitSignal::Interrupt
            };
            // SAFETY: called by the real same-CPU entered C owner.
            unsafe { target.signal(runtime, kind) }.unwrap();
            f.signals.fetch_add(1, Ordering::Relaxed);
        }
        5 => {
            let old = f.target(1);
            assert_eq!(
                // SAFETY: this is the entered same-CPU continuation inside `run_scoped`, with
                // the service roots kept.
                unsafe { old.signal(runtime, WaitSignal::Fatal) },
                Err(WorkError::Closed)
            );
            f.stale.fetch_add(1, Ordering::Relaxed);
        }
        6 => f.enqueue(4),
        7 | 8 => {
            let target = f.target(0);
            let kind = if event == 7 {
                WaitSignal::Interrupt
            } else {
                WaitSignal::Fatal
            };
            // SAFETY: entered current posts to its own expected job.
            unsafe { target.signal(runtime, kind) }.unwrap();
            f.self_signals.fetch_add(1, Ordering::Relaxed);
        }
        10 => {
            let target = f.target(1);
            let wake = (*target.wake).clone();
            drop(target);
            // Explicit hosted IRQ model: only the generic atomic
            // signal view is reachable here, never a C task/Runtime pointer.
            f.cpu.irq.store(true, Ordering::Release);
            wake.wake().unwrap();
            wake.wake().unwrap(); // coalesced repeated wake before resume
            f.cpu.irq.store(false, Ordering::Release);
            drop(wake);
        }
        _ => panic!("unexpected mutex checkpoint"),
    }
}
unsafe extern "C" fn mutex_role(context: *mut c_void) {
    // SAFETY: the queue owner keeps this boxed role through the callback.
    let job = unsafe { mutex_job(context) };
    let f = job.fixture.upgrade().unwrap();
    // SAFETY: the harness keeps the pinned executor through every callback and retirement.
    let runtime = unsafe { f.runtime() };
    let index = if job.role == 4 { 1 } else { job.role };
    let binding = f.binding(index);
    let target = binding.target_current().unwrap();
    if job.role == 4 {
        let old = f.target(1);
        assert_eq!(old.context_id(), target.context_id());
        assert_ne!(old.sequence(), target.sequence());
    }
    *f.targets[job.role].try_lock().unwrap() = Some(target);
    let mut call = MutexCall {
        scenario: job.scenario,
        role: job.role as u32,
        context,
        checkpoint: mutex_checkpoint,
    };
    // SAFETY: only shared kept Runtime/owner roots span suspension;
    // call is an immovable stack local until this C body returns.
    unsafe {
        binding.run_scoped(
            runtime,
            if job.scenario >= 11 {
                stela_linux_rt_mutex_test_run
            } else {
                stela_linux_mutex_test_run
            },
            ptr::from_mut(&mut call).cast(),
        )
    }
    .unwrap();
    f.completed.fetch_add(1, Ordering::Relaxed);
}

fn exact_mutex_child(selector: &str) {
    let output = std::process::Command::new(std::env::current_exe().unwrap())
        .args([
            "--exact",
            selector,
            "--ignored",
            "--test-threads=1",
            "--nocapture",
            "--format=pretty",
            "--color=never",
        ])
        .output()
        .unwrap();
    let stdout = std::string::String::from_utf8_lossy(&output.stdout);
    let stderr = std::string::String::from_utf8_lossy(&output.stderr);
    let expected = std::format!("test {selector} ... ok");
    let named = stdout.lines().any(|line| line == expected);
    let one = stdout
        .lines()
        .any(|line| line.starts_with("test result: ok. 1 passed; 0 failed; 0 ignored;"));
    assert!(
        output.status.success() && named && one,
        "mutex child did not execute/pass exactly once ({}):\n{}{}",
        output.status,
        stdout,
        stderr
    );
}
pub(crate) fn run_mutex_child() {
    exact_mutex_child("task::tests::mutex_child");
    exact_mutex_child("task::tests::mutex_quarantine_child");
    exact_mutex_child("task::tests::rt_mutex::rt_mutex_child");
}

mod rt_mutex;

#[test]
#[ignore = "isolation: child"]
fn mutex_child() {
    // SAFETY: isolated canonical allocator registry, same real GlobalAlloc
    // owns ordinary and raw-span allocations and their exact-layout frees.
    assert!(crate::test_heap::register());
    assert_eq!(core::mem::size_of::<MutexCall>(), 24);
    let dropped = Arc::new(AtomicU64::new(0));
    // Scenario 5 is retired: nothing runs during a waiter's PREPARE that
    // could post it a signal after its condition check.
    for (round, scenario) in [0, 1, 2, 3, 4, 6, 7, 8, 9, 10].into_iter().enumerate() {
        let cpu = CpuModel::new();
        let f = MutexFixture::new(cpu.clone(), scenario);
        // SAFETY: the fixture is the queue's owner, and only this test thread produces
        // work.
        let queue = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
        let executor = Executor::new_hosted_for_test(queue, stacks(&dropped, 16 * 1024)).unwrap();
        let runtime = executor.runtime();
        f.runtime.store(
            runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
            Ordering::Release,
        );
        // SAFETY: guarded stacks and kept fixture/job roots, no
        // C owner/ref survives completion, no native admission is asserted.
        unsafe { runtime.call_foreground(mutex_role, f.context(0), 100000) }.unwrap();
        assert_eq!(
            f.completed.load(Ordering::Relaxed),
            if scenario == 0 { 5 } else { 2 }
        );
        assert_eq!(
            f.stale.load(Ordering::Relaxed),
            if scenario == 0 { 2 } else { 0 }
        );
        let expected_signals = match scenario {
            1 | 3 | 4 | 6 | 9 => 1,
            2 => 2,
            _ => 0,
        };
        assert_eq!(f.signals.load(Ordering::Relaxed), expected_signals);
        assert_eq!(
            f.self_signals.load(Ordering::Relaxed),
            if scenario < 7 || scenario == 10 { 2 } else { 0 }
        );
        // All expected-job snapshots are now inert even though metadata is
        // still held. Taking them in process context breaks Arc cycles.
        for slot in &f.targets {
            if let Some(target) = slot.lock().unwrap().take() {
                assert_eq!(target.wake.wake(), Err(WorkError::Closed));
                drop(target);
            }
        }
        assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
        runtime.queue().close().unwrap();
        // SAFETY: the queue is closed and this test thread is the only producer; pending work
        // is a checked refusal.
        assert_eq!(unsafe { runtime.retire() }, Err(WorkError::Pending));
        for slot in &f.bindings {
            if let Some(binding) = slot.lock().unwrap().take() {
                let retained =
                        // SAFETY: the queue is closed and no producer, task or waker still uses
                        // this binding.
                        unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
                assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
                drop(retained);
                // SAFETY: jobs completed; C refs, waiters, signal
                // snapshots and all producer borrows ended independently.
                assert!(unsafe { binding.release_quiescent() }.is_ok());
            }
        }
        // SAFETY: the queue is closed and this test thread is the only producer; pending work
        // is a checked refusal.
        unsafe { runtime.retire() }.unwrap();
        drop(executor);
        assert_eq!(dropped.load(Ordering::Relaxed), (round as u64 + 1) * 4);
        assert_eq!(quarantined(), 0);
        assert!(!crate::test_heap::span_occupied());
        // SAFETY: the C test hook only checks task state on this thread's entered continuation.
        unsafe { stela_linux_task_test_taskless() };
    }
}

unsafe extern "C" fn quarantine_targets(context: *mut c_void) {
    // No Linux entry is needed for capture/drop mechanics. Runtime still
    // provides current/C task identity on its real guarded stack.
    // SAFETY: the queue owner keeps this boxed role through the callback.
    let job = unsafe { mutex_job(context) };
    let f = job.fixture.upgrade().unwrap();
    let binding = f.binding(0);
    let baseline = Arc::strong_count(&binding);
    let target = binding.target_current().unwrap();
    let foreign = target.clone();
    thread::spawn(move || drop(foreign)).join().unwrap();
    assert_eq!(quarantined(), 1);
    assert_eq!(Arc::strong_count(&binding), baseline + 2);
    let irq = target.clone();
    f.cpu.irq.store(true, Ordering::Release);
    drop(irq);
    f.cpu.irq.store(false, Ordering::Release);
    assert_eq!(quarantined(), 2);
    assert_eq!(Arc::strong_count(&binding), baseline + 3);
    drop(target);
    assert_eq!(Arc::strong_count(&binding), baseline + 2);
}
#[test]
#[ignore = "isolation: child"]
fn mutex_quarantine_child() {
    let dropped = Arc::new(AtomicU64::new(0));
    let cpu = CpuModel::new();
    let f = MutexFixture::new(cpu.clone(), 0);
    // SAFETY: the fixture is the queue's owner, and only this test thread produces
    // work.
    let queue = unsafe { Queue::<16>::new(cpu, f.clone()) }.unwrap();
    let executor = Executor::new_hosted_for_test(queue, stacks(&dropped, 16 * 1024)).unwrap();
    let runtime = executor.runtime();
    f.runtime.store(
        runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
        Ordering::Release,
    );
    // SAFETY: the callbacks are static, never unwind, and take the fixture the queue keeps
    // until completion.
    unsafe { runtime.call_foreground(quarantine_targets, f.context(0), 100000) }.unwrap();
    runtime.queue().close().unwrap();
    let binding = f.bindings[0].lock().unwrap().take().unwrap();
    // SAFETY: the queue is closed and no producer, task or waker still uses this binding.
    let retained = unsafe { runtime.detach_binding(binding.context_id().unwrap()) }.unwrap();
    drop(retained);
    // The two improper drops kept real metadata owners. A
    // claimed unique release must refuse; no final C free is smuggled into
    // the foreign thread/IRQ model or inferred from task usage alone.
    // SAFETY: the runtime detached this binding and no job or waker still uses it.
    let unreleased = unsafe { binding.release_quiescent() }.err().unwrap();
    assert_eq!(Arc::strong_count(&unreleased), 3);
    drop(unreleased); // nonfinal process-context drop; two kept owners remain
    // SAFETY: the queue is closed and this test thread is the only producer; pending work is a
    // checked refusal.
    unsafe { runtime.retire() }.unwrap();
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 4);
    assert_eq!(quarantined(), 2);
}

/// Run a non-returning terminal C fixture on an actual hosted Runtime stack and
/// TaskBinding. Reuses the same guarded stacks and checked CPU as the task tests.
/// # Safety
/// Keep owner/body/context and all source/dev_id products live on the caller's
/// suspended frame. The body must terminate the isolated subprocess, never
/// unwind, return, escape its scoped task loan or install a native IRQ producer.
pub(crate) unsafe fn terminal_foreground(
    owner: *const RawOwner,
    body: unsafe extern "C" fn(*mut c_void),
    context: *mut c_void,
) -> ! {
    struct Call {
        fixture: Arc<Fixture>,
        owner: *const RawOwner,
        body: unsafe extern "C" fn(*mut c_void),
        context: *mut c_void,
    }
    unsafe extern "C" fn entered(context: *mut c_void) {
        // SAFETY: the outer helper retains this exact stack-local Call and its
        // actual owner/source roots throughout this non-returning task scope.
        let call = unsafe { &*context.cast::<Call>() };
        // SAFETY: run_scoped already installed this Runtime's real task/current;
        // the original caller retains the SAME owner through fatal reporting.
        let _owner = unsafe { crate::owner::Entered::new(call.owner) };
        // SAFETY: caller's retained body/context and non-return/no-unwind contract.
        unsafe { (call.body)(call.context) };
        std::process::exit(98); // a fixture return is never a successful trap
    }
    unsafe extern "C" fn foreground(context: *mut c_void) {
        // SAFETY: helper retains Call and Queue's actual fixture through this job.
        let call = unsafe { &*context.cast::<Call>() };
        let binding = call.fixture.binding(0);
        // SAFETY: pinned executor/guarded stack, original fixture and body live
        // through this actual scoped continuation; no owner is entered outside it.
        unsafe { binding.run_scoped(call.fixture.runtime(), entered, context) }.unwrap();
    }
    let cpu = CpuModel::new();
    let fixture = Fixture::new(cpu.clone());
    // SAFETY: this isolated thread owns the Queue and keeps its original fixture.
    let queue = unsafe { Queue::<16>::new(cpu, fixture.clone()) }.unwrap();
    let dropped = Arc::new(AtomicU64::new(0));
    let executor =
        Executor::new_hosted_for_test(queue, stacks_count(&dropped, 16 * 1024, 4)).unwrap();
    let runtime = executor.runtime();
    fixture.runtime.store(
        runtime.get_ref() as *const Runtime<16> as *mut Runtime<16>,
        Ordering::Release,
    );
    let mut call = Call {
        fixture,
        owner,
        body,
        context,
    };
    // SAFETY: the caller retains every borrowed source/owner/dev_id through the
    // actual switched foreground stack. The bound body terminates this process.
    unsafe { runtime.call_foreground(foreground, (&raw mut call).cast(), 10000) }.unwrap();
    std::process::exit(98);
}
