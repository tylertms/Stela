//! Hosted context switching on guarded POSIX mappings.
use super::*;
use core::arch::asm;
use std::sync::Mutex;
unsafe extern "C" {
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
    dropped: Arc<AtomicU64>,
}
// SAFETY: exclusive private OS mmap, guard has PROT_NONE, and the
// owning runtime keeps this exact mapping through every continuation.
unsafe impl StackBacking for MappedStack {
    fn bounds(&self) -> Range<usize> {
        self.base + 4096..self.base + self.bytes
    }
    fn guard(&self) -> Range<usize> {
        self.base..self.base + 4096
    }
}
impl Drop for MappedStack {
    fn drop(&mut self) {
        // SAFETY: base and bytes are this object's successful mmap extent. A rejected
        // constructor has not started a continuation; started stacks drop only after
        // retirement. Quarantined continuations keep this owner and mapping kept.
        assert_eq!(unsafe { munmap(self.base as *mut c_void, self.bytes) }, 0);
        self.dropped.fetch_add(1, Ordering::Relaxed);
    }
}
fn stacks(dropped: Arc<AtomicU64>) -> Vec<Stack> {
    stacks_count(dropped, 4)
}
fn stacks_count(dropped: Arc<AtomicU64>, count: usize) -> Vec<Stack> {
    (0..count)
        .map(|_| {
            #[cfg(target_os = "macos")]
            let anonymous = 0x1000;
            #[cfg(not(target_os = "macos"))]
            let anonymous = 0x20;
            let bytes = 128 * 1024 + 4096;
            // SAFETY: This requests a new private anonymous RW mapping with no supplied
            // address or file descriptor. The result is checked before it is accessed.
            let base = unsafe { mmap(core::ptr::null_mut(), bytes, 3, 2 | anonymous, -1, 0) };
            assert_ne!(base as usize, usize::MAX);
            // SAFETY: mmap succeeded above and returned a page-aligned mapping larger than
            // 4096 bytes. This protects its first page before the stack is published.
            assert_eq!(unsafe { mprotect(base, 4096, 0) }, 0);
            Stack::new(Box::new(MappedStack {
                base: base as usize,
                bytes,
                dropped: dropped.clone(),
            }))
            .unwrap()
        })
        .collect()
}
mod idle_general;
mod ranked;

struct Fixture {
    cpu: Arc<CpuModel>,
    baseline: context::Controls,
    runtime: AtomicU64,
    events: Mutex<Vec<(u64, u64, usize)>>,
    done: AtomicBool,
    parked: Mutex<Vec<(Arc<firmware::Parked>, WakeHandle<16>)>>,
}
impl Fixture {
    /// # Safety
    /// setup must have published the live pinned Executor runtime, and that
    /// Executor must remain held while this reference is used.
    unsafe fn runtime(&self) -> Pin<&Runtime<16>> {
        // SAFETY: setup wrote this pointer from the already pinned Runtime before
        // callback admission. The Executor keeps that allocation through every use here.
        unsafe {
            Pin::new_unchecked(&*(self.runtime.load(Ordering::Acquire) as *const Runtime<16>))
        }
    }
    fn event(&self, value: u64) {
        // SAFETY: setup published this pinned Runtime pointer before callback admission.
        // The fixture is used while its Executor is live, including all suspended callbacks.
        let runtime = unsafe { self.runtime() };
        let id = runtime.current_id().unwrap();
        assert_eq!(runtime.queue.current_id(), Ok(id));
        self.events
            .lock()
            .unwrap()
            .push((value, id.get(), context::stack_pointer()));
    }
}
/// # Safety
/// p must be the queue-kept Arc pointer for Fixture; the returned borrow
/// must end before that owner can be released.
unsafe fn fixture<'a>(p: *mut c_void) -> &'a Fixture {
    // SAFETY: The caller supplies the aligned Arc::as_ptr allocation of this fixture
    // type. Its queue keeps the Arc while callbacks use this shared reference.
    unsafe { &*p.cast::<Fixture>() }
}
unsafe extern "C" fn notify_one(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    f.event(20);
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    unsafe { f.runtime() }.yield_now().unwrap();
    f.event(21);
}
unsafe extern "C" fn notify_two(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    unsafe { fixture(p) }.event(22);
}
unsafe extern "C" fn general_one(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    f.event(30);
    // Model interrupt-entry metadata while hardware RSP still
    // belongs to a real task stack. IF/stack membership alone must
    // never authorize a thread ID or context switch from an ISR.
    f.cpu.irq.store(true, Ordering::Release);
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let r = unsafe { f.runtime() };
    assert_eq!(r.current_id(), Err(WorkError::InterruptContext));
    assert_eq!(r.queue.current_id(), Err(WorkError::InterruptContext));
    assert_eq!(r.yield_now(), Err(WorkError::InterruptContext));
    f.cpu.irq.store(false, Ordering::Release);
    // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
    // snapshots their control registers into local storage; it performs no privileged access.
    let original = unsafe { context::read_controls() };
    let mxcsr = (original.mxcsr & !(3 << 13)) | 2 << 13;
    let fcw = (original.fcw & !(3 << 10)) | 2 << 10;
    // SAFETY: mxcsr and fcw come from this thread's control values with only their
    // rounding-mode fields changed. Both operands remain live, aligned local storage.
    unsafe {
        asm!("ldmxcsr [{}]",in(reg)&mxcsr,options(nostack));
        asm!("fldcw [{}]",in(reg)&fcw,options(nostack));
    }
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    unsafe { f.runtime() }.yield_now().unwrap();
    // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
    // snapshots their control registers into local storage; it performs no privileged access.
    let resumed = unsafe { context::read_controls() };
    assert_eq!(resumed.mxcsr, mxcsr);
    assert_eq!(resumed.fcw, fcw);
    // SAFETY: mxcsr and fcw come from this thread's control values with only their
    // rounding-mode fields changed. Both operands remain live, aligned local storage.
    unsafe {
        asm!("ldmxcsr [{}]",in(reg)&original.mxcsr,options(nostack));
        asm!("fldcw [{}]",in(reg)&original.fcw,options(nostack));
    }
    f.event(31);
}
unsafe extern "C" fn general_two(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
    // snapshots their control registers into local storage; it performs no privileged access.
    let controls = unsafe { context::read_controls() };
    assert_eq!(controls.mxcsr, f.baseline.mxcsr);
    assert_eq!(controls.fcw, f.baseline.fcw);
    f.event(40);
}
unsafe extern "C" fn foreground(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    f.event(10);
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let r = unsafe { f.runtime() };
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::Notify, notify_one, p) }.unwrap();
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::Notify, notify_two, p) }.unwrap();
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::General, general_one, p) }.unwrap();
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::General, general_two, p) }.unwrap();
    assert_eq!(
        f.events.lock().unwrap().len(),
        1,
        "execute must not run inline"
    );
    r.drain(10000).unwrap();
    f.event(11);
}
fn setup() -> (Executor<16>, Arc<Fixture>, Arc<CpuModel>, Arc<AtomicU64>) {
    setup_with_service(false)
}
fn setup_with_service(
    reserve_service: bool,
) -> (Executor<16>, Arc<Fixture>, Arc<CpuModel>, Arc<AtomicU64>) {
    setup_with_reservations(reserve_service, false)
}
fn setup_with_reservations(
    reserve_service: bool,
    reserve_firmware: bool,
) -> (Executor<16>, Arc<Fixture>, Arc<CpuModel>, Arc<AtomicU64>) {
    let cpu = CpuModel::new();
    let f = Arc::new(Fixture {
        cpu: cpu.clone(),
        // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
        // snapshots their control registers into local storage; it performs no privileged access.
        baseline: unsafe { context::read_controls() },
        runtime: AtomicU64::new(0),
        events: Mutex::new(Vec::new()),
        done: AtomicBool::new(false),
        parked: Mutex::new(Vec::new()),
    });
    // SAFETY: The queue keeps the fixture Arc and same-thread CpuModel. setup stores
    // the pinned runtime pointer before admitting any callback that can read it.
    let q = unsafe { Queue::<16>::new(cpu.clone(), f.clone()) }.unwrap();
    let dropped = Arc::new(AtomicU64::new(0));
    // Hosted context switching uses guarded mappings and unprivileged FP controls.
    let runtime = Runtime::with_reserved_controls(
        q,
        stacks_count(
            dropped.clone(),
            4 + usize::from(reserve_service) + usize::from(reserve_firmware),
        ),
        // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
        // snapshots their control registers into local storage; it performs no privileged access.
        unsafe { context::read_controls() },
        reserve_service,
        reserve_firmware,
    )
    .unwrap();
    f.runtime.store(
        runtime.as_ref().get_ref() as *const _ as u64,
        Ordering::Release,
    );
    (
        Executor {
            runtime: ManuallyDrop::new(runtime),
        },
        f,
        cpu,
        dropped,
    )
}
unsafe extern "C" fn reserved_service(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let runtime = unsafe { f.runtime() };
    assert_eq!(runtime.service_current(), Ok(true));
    f.event(50);
    runtime.yield_now().unwrap();
    assert_eq!(runtime.service_current(), Ok(true));
    f.event(51);
}
#[test]
fn reserved_service_stack() {
    let (ordinary, _, _, ordinary_dropped) = setup();
    let runtime = ordinary.runtime();
    assert_eq!(
        // SAFETY: noop is static and ignores the null context. This unreserved Runtime
        // rejects service admission before keeping or invoking the callback.
        unsafe { runtime.start_service(noop, core::ptr::null_mut()) },
        Err(WorkError::Unavailable)
    );
    runtime.queue().close().unwrap();
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { runtime.retire() }.unwrap();
    drop(ordinary);
    assert_eq!(ordinary_dropped.load(Ordering::Relaxed), 4);

    let (executor, f, cpu, dropped) = setup_with_service(true);
    let runtime = executor.runtime();
    let context = Arc::as_ptr(&f) as *mut c_void;
    // SAFETY: The service callback receives the queue-kept fixture Arc pointer.
    // The pinned executor and its dedicated service stack survive until service join.
    let service = unsafe { runtime.start_service(reserved_service, context) }.unwrap();
    assert!(f.events.lock().unwrap().is_empty());
    // SAFETY: The context is the queue-kept fixture Arc pointer for this callback.
    // The Executor keeps its pinned runtime and guarded stacks through join or quarantine.
    unsafe { runtime.call_foreground(foreground, context, 10000) }.unwrap();
    assert!(runtime.queue().completed(service).unwrap());
    assert_eq!(
        // SAFETY: The service callback receives the queue-kept fixture Arc pointer.
        // The pinned executor and its dedicated service stack survive until service join.
        unsafe { runtime.start_service(reserved_service, context) },
        Err(WorkError::Busy)
    );
    let events = f.events.lock().unwrap();
    let service_id = events.iter().find(|event| event.0 == 50).unwrap().1;
    assert_eq!(
        events.iter().find(|event| event.0 == 51).unwrap().1,
        service_id
    );
    let mut ids: Vec<_> = events.iter().map(|event| event.1).collect();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(
        ids.len(),
        5,
        "Foreground/Notify/service/two General identities"
    );
    drop(events);
    runtime.queue().close().unwrap();
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { runtime.retire() }.unwrap();
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 5);
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
}
#[test]
fn continuation_stacks_and_control_state() {
    let (executor, f, cpu, dropped) = setup();
    let r = executor.runtime();
    assert_eq!(r.current_id(), Err(WorkError::Unavailable));
    // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
    // snapshots their control registers into local storage; it performs no privileged access.
    let original = unsafe { context::read_controls() };
    // SAFETY: The context is the queue-kept fixture Arc pointer for this callback.
    // The Executor keeps its pinned runtime and guarded stacks through join or quarantine.
    unsafe { r.call_foreground(foreground, Arc::as_ptr(&f) as *mut c_void, 10000) }.unwrap();
    // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
    // snapshots their control registers into local storage; it performs no privileged access.
    let after = unsafe { context::read_controls() };
    assert_eq!(original.mxcsr, after.mxcsr);
    assert_eq!(original.fcw, after.fcw);
    let events = f.events.lock().unwrap();
    let pos = |n| events.iter().position(|e| e.0 == n).unwrap();
    assert!(pos(20) < pos(21) && pos(21) < pos(22));
    assert!(pos(30) < pos(40) && pos(40) < pos(31));
    assert_eq!(events[pos(10)].1, events[pos(11)].1);
    assert_eq!(events[pos(20)].1, events[pos(22)].1);
    let mut ids: Vec<_> = events.iter().map(|e| e.1).collect();
    ids.sort_unstable();
    ids.dedup();
    assert_eq!(ids.len(), 4);
    for (_, _, rsp) in &*events {
        assert!(
            // SAFETY: The dispatcher owns this pinned Runtime and all jobs have joined.
            // No continuation can mutate the task vector while this shared inspection runs.
            unsafe { &*r.tasks.get() }
                .iter()
                .any(|t| t.stack.bounds.contains(rsp))
        );
    }
    assert_eq!(cpu.flags.load(Ordering::Acquire), 0x202);
    assert_eq!(r.queue.current_id(), Err(WorkError::Unavailable));
    drop(events);
    let view = r.queue.clone();
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { r.retire() }.unwrap();
    assert_eq!(view.current_id(), Err(WorkError::Closed));
    drop(executor);
    assert_eq!(dropped.load(Ordering::Relaxed), 4);
}
#[test]
fn executor_binding_is_exclusive() {
    let (executor, _, _, _) = setup();
    let view = executor.runtime().queue.clone();
    let dropped = Arc::new(AtomicU64::new(0));
    for retired in [false, true] {
        if retired {
            // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
            // Completed jobs/bindings permit retirement; pending continuations keep their owners.
            unsafe { executor.runtime().retire() }.unwrap();
        }
        // SAFETY: The hosted x86-64 process has x87/SSE enabled. read_controls only
        // snapshots their control registers into local storage; it performs no privileged access.
        let result = Runtime::with_controls(view.clone(), stacks(dropped.clone()), unsafe {
            context::read_controls()
        });
        assert!(matches!(result, Err(WorkError::Busy)));
        assert_eq!(
            view.current_id(),
            Err(if retired {
                WorkError::Closed
            } else {
                WorkError::Unavailable
            })
        );
    }
    assert_eq!(dropped.load(Ordering::Relaxed), 8);
}

unsafe extern "C" fn serial_drain(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let r = unsafe { f.runtime() };
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::Notify, notify_two, p) }.unwrap();
    assert_eq!(r.drain(10000), Err(WorkError::WouldDeadlock));
    f.event(50);
}
unsafe extern "C" fn cyclic_drain(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let r = unsafe { f.runtime() };
    let result = r.drain(10000);
    assert!(matches!(result, Ok(()) | Err(WorkError::WouldDeadlock)));
    f.event(if result.is_err() { 61 } else { 60 });
}
#[test]
fn drain_cycles_do_not_deadlock() {
    let (executor, f, _, _) = setup();
    let r = executor.runtime();
    let context = Arc::as_ptr(&f) as *mut c_void;
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::Notify, serial_drain, context) }.unwrap();
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::General, cyclic_drain, context) }.unwrap();
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::General, cyclic_drain, context) }.unwrap();
    r.drain(20000).unwrap();
    let events = f.events.lock().unwrap();
    assert!(events.iter().any(|e| e.0 == 50));
    assert!(events.iter().any(|e| e.0 == 61));
    assert!(events.iter().any(|e| e.0 == 60));
    drop(events);
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { r.retire() }.unwrap();
}
unsafe extern "C" fn bounded_work(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    // SAFETY: setup published this pinned Runtime pointer before callback admission.
    // The fixture is used while its Executor is live, including all suspended callbacks.
    let r = unsafe { f.runtime() };
    f.event(70);
    if r.reschedule_requested().unwrap() {
        r.reschedule().unwrap();
    }
    f.event(71);
    assert!(!r.reschedule_requested().unwrap());
    f.event(72);
}
#[test]
fn a_step_bound_ends_the_step_at_a_reschedule_point() {
    let (executor, f, _, _) = setup();
    let r = executor.runtime();
    let context = Arc::as_ptr(&f) as *mut c_void;
    let seen = |f: &Fixture| {
        f.events
            .lock()
            .unwrap()
            .iter()
            .map(|e| e.0)
            .collect::<Vec<_>>()
    };
    // SAFETY: The callback and context identify this queue's fixture Arc.
    // The owning Executor keeps its pinned runtime/stacks alive while the job is suspended.
    unsafe { r.queue.execute(Kind::General, bounded_work, context) }.unwrap();
    r.bound_steps(Some(|| true));
    assert!(r.step().unwrap());
    assert_eq!(
        seen(&f),
        [70],
        "the bound ends the step at its reschedule point"
    );
    r.bound_steps(None);
    while r.step().unwrap() {}
    assert_eq!(
        seen(&f),
        [70, 71, 72],
        "unbounded, the continuation runs to its end"
    );
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { r.retire() }.unwrap();
}
unsafe extern "C" fn suspended(p: *mut c_void) {
    // SAFETY: p is the matching fixture Arc pointer submitted to this callback.
    // The queue keeps that owner across every suspension and callback return.
    let f = unsafe { fixture(p) };
    while !f.done.load(Ordering::Acquire) {
        // SAFETY: setup published this pinned Runtime pointer before callback admission.
        // The fixture is used while its Executor is live, including all suspended callbacks.
        unsafe { f.runtime() }.yield_now().unwrap();
    }
}
#[test]
fn timeout_quarantines_executor() {
    let (executor, f, _, dropped) = setup();
    let raw = executor.runtime().get_ref() as *const Runtime<16>;
    let view = executor.runtime().queue.clone();
    assert_eq!(
        // SAFETY: The context is the queue-kept fixture Arc pointer for this callback.
        // The Executor keeps its pinned runtime and guarded stacks through join or quarantine.
        unsafe {
            executor
                .runtime()
                .call_foreground(suspended, Arc::as_ptr(&f) as *mut c_void, 20)
        },
        Err(WorkError::TimedOut)
    );
    assert_eq!(
        // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
        // Completed jobs/bindings permit retirement; pending continuations keep their owners.
        unsafe { executor.runtime().retire() },
        Err(WorkError::Pending)
    );
    let weak = Arc::downgrade(&f);
    drop(executor);
    drop(f);
    assert_eq!(dropped.load(Ordering::Relaxed), 0);
    assert!(weak.upgrade().is_some());
    assert_eq!(view.current_id(), Err(WorkError::Closed));
    assert_eq!(view.active.id.load(Ordering::Acquire), 0);
    // SAFETY: raw came from this pinned Executor. Its timed-out Drop keeps the
    // entire ManuallyDrop allocation and stacks quarantined, so the pointer stays live.
    let retained = unsafe { Pin::new_unchecked(&*raw) };
    assert_eq!(retained.step(), Err(WorkError::Closed));
}
unsafe extern "C" fn counted(_: *mut c_void) {}
#[test]
fn step_counters_split_selection_body_and_empty_steps() {
    let (executor, _, _, _) = setup();
    let r = executor.runtime();
    let counts = &super::super::step_counts::STEP_COUNTS;
    let load = |counter: &AtomicU64| counter.load(Ordering::Relaxed);
    let before = [
        load(&counts.select),
        load(&counts.body),
        load(&counts.empty_steps),
        load(&counts.scans),
        load(&counts.locks),
    ];
    // SAFETY: counted is static and ignores its null context; the Executor
    // keeps its pinned runtime and stacks through the job's completion.
    unsafe {
        r.queue
            .execute(Kind::General, counted, core::ptr::null_mut())
    }
    .unwrap();
    assert!(r.step().unwrap());
    assert!(!r.step().unwrap());
    let after = [
        load(&counts.select),
        load(&counts.body),
        load(&counts.empty_steps),
        load(&counts.scans),
        load(&counts.locks),
    ];
    // Other tests share the process-wide counters, so only growth is exact.
    for (before, after) in before.iter().zip(after) {
        assert!(after > *before);
    }
    // SAFETY: This test has no asynchronous producers and calls on the dispatcher.
    // Completed jobs/bindings permit retirement; pending continuations keep their owners.
    unsafe { r.retire() }.unwrap();
}
// Switched-stack fixtures for the generic binding contract.
// This is a hosted state/IRQ model, not a native Linux/C task producer.
mod bound;

mod firmware;

#[cfg(feature = "linux-kernel")]
mod service_geometry;
