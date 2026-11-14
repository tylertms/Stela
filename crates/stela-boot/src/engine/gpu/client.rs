//! NVK's process services (`compat/libc/stela/host.h`), from whichever PE runs
//! it.
//!
//! Linux's files belong to the environment on the boot CPU, so a PE posts its
//! call, ends the boot PE's slice and spins: an ioctl costs the caller a
//! round trip, not a core. Each call runs on one of a few standing Linux
//! tasks, since an ioctl may sleep on the GPU. On the boot CPU itself there is
//! no one to wait for, so the caller runs the environment until its call is done.
//! Each wait is bounded ([`posted_call`]): a call no task took in time is
//! withdrawn and answered `ETIMEDOUT`, and one a task performs too long stops
//! the boot, naming it, as a processor held out of its guest for long makes
//! XNU panic.
//!
//! A thread Mesa starts is a Linux task too, as Linux's own kthreads are, so it
//! makes its calls in place and sleeps in Linux. It runs Mesa on a stack of its
//! own and goes back to the task's, which is what Linux knows, for those. Everything
//! that waits in the C library comes through `stela_libc_wait`: a PE services
//! shootdowns and spins, the boot CPU also runs the environment, and a task
//! thread sleeps.
use super::{
    BOOT_CPU, Installed, PAGE, installed,
    stack::{self, Stack},
};
use crate::{engine::host, output};
use alloc::{
    alloc::{Layout, alloc, dealloc},
    boxed::Box,
    collections::VecDeque,
    rc::Rc,
    vec::Vec,
};
use core::{
    cell::{Cell, RefCell},
    ffi::{CStr, c_char, c_int, c_long, c_uint, c_ulong, c_void},
    hint,
    pin::Pin,
    ptr::{self, NonNull},
    slice, str,
    sync::atomic::{
        AtomicBool, AtomicI64, AtomicPtr, AtomicU32, AtomicU64, AtomicUsize, Ordering, fence,
    },
};
use stela_boot_core::page_window::Cache;
use stela_boot_core::posted_call::{self, Claim};
use stela_drivers::platform::work::{WorkError, service::Idle};
use stela_linux::{
    client,
    task::runtime::{ManagedJob, Outcome, Pending, SubmitError},
};

fn nanoseconds(ticks: u64, tsc_hz: u64) -> u64 {
    (u128::from(ticks) * 1_000_000_000 / u128::from(tsc_hz)) as u64
}

/// Mesa's `CLOCK_MONOTONIC`: Linux's own clock, carried to Mesa's cores by
/// the TSC from one reading on the boot CPU, where Linux's clock reads.
/// The absolute deadlines Mesa gives DRM waits then name the instant Linux
/// waits for; a TSC count from reset would place each one later by however
/// long the boot ran before Linux's clock started.
#[derive(Clone, Copy)]
pub(super) struct LibcClock {
    tsc: u64,
    ns: u64,
}

impl LibcClock {
    /// Pairs Linux's clock with the TSC read around it.
    pub(super) fn anchor(
        linux_ns: impl FnOnce() -> Result<u64, WorkError>,
    ) -> Result<Self, WorkError> {
        let before = host::rdtsc();
        let ns = linux_ns()?;
        let after = host::rdtsc();
        Ok(Self {
            tsc: before + after.saturating_sub(before) / 2,
            ns,
        })
    }

    fn now_ns(self, tsc: u64, tsc_hz: u64) -> u64 {
        self.ns
            .saturating_add(nanoseconds(tsc.saturating_sub(self.tsc), tsc_hz))
    }
}

const ENODEV: i64 = -19;
const EFAULT: i64 = -14;
const ENOMEM: i64 = -12;
const EAGAIN: i64 = -11;
const ETIMEDOUT: i64 = -110;

/// `struct stela_libc_drm_device`.
#[repr(C)]
#[derive(Clone, Copy, Debug)]
pub(super) struct DrmDevice {
    pub segment: u16,
    pub vendor: u16,
    pub device: u16,
    pub subsystem_vendor: u16,
    pub subsystem_device: u16,
    pub bus: u8,
    pub slot: u8,
    pub function: u8,
    pub revision: u8,
}

/// `struct drm_nouveau_getparam`.
#[repr(C)]
struct GetParam {
    parameter: u64,
    value: u64,
}

/// `DRM_IOCTL_NOUVEAU_GETPARAM`.
const GET_PARAM: u32 = 0xc010_6440;
const CHIPSET_ID: u64 = 11;
/// The first render node's minor.
pub(super) const RENDER_MINOR: u32 = 128;

#[derive(Clone, Copy)]
enum Operation {
    Open(u32),
    Close(c_int),
    Duplicate(c_int),
    Device(c_int),
    Control {
        fd: c_int,
        request: u32,
        argument: *mut c_void,
    },
    Map {
        fd: c_int,
        offset: u64,
        size: usize,
    },
    Unmap(u64),
    Fault {
        address: u64,
        write: bool,
    },
    Thread(*mut Thread),
    Wake,
    Export(*const client::Run, usize),
    /// Stop the GPUs as a power-off does.
    Shutdown,
}

/// What Mesa's threads run on: their calls go to Linux, off this stack. Linux's
/// default.
const THREAD_STACK_BYTES: usize = 8 << 20;
/// What stays free of a task's stack below `run_thread`'s frame, which a
/// thread's Linux calls start past.
const TASK_STACK_MARGIN: u64 = 256;

/// A thread for its task to run.
struct Thread {
    body: unsafe extern "C" fn(*mut c_void),
    argument: *mut c_void,
    core: c_uint,
}

/// A running thread of Mesa's.
#[derive(Clone, Copy)]
pub(super) struct Running {
    top: u64,
    core: c_uint,
    /// Where its Linux calls run on its task's stack.
    task: u64,
}

/// `STELA_LIBC_PROCESSORS` and `STELA_LIBC_THREADS`: a thread's core is past
/// the processors'.
const PROCESSORS: usize = 256;
const THREADS: usize = 16;
/// The word each thread is asleep on in `stela_libc_wait`. Every unlock wakes
/// its word, so one nobody sleeps on must cost no call into Linux.
static SLEEPING: [AtomicPtr<c_uint>; THREADS] =
    [const { AtomicPtr::new(ptr::null_mut()) }; THREADS];

fn run_thread(gpu: &Installed, thread: &Thread, stack: &Stack) {
    let running = Running {
        top: stack.top(),
        core: thread.core,
        task: (stack::pointer() - TASK_STACK_MARGIN) & !15,
    };
    gpu.mesa_threads
        .borrow_mut()
        .insert(stack.bottom(), running);
    // SAFETY: the thread's own stack, and the C library's thread.
    unsafe { stack::on_stack(running.top, &mut || (thread.body)(thread.argument)) };
    gpu.mesa_threads.borrow_mut().remove(&stack.bottom());
}

/// The Mesa thread running here, by its stack.
fn current_thread(gpu: &Installed) -> Option<Running> {
    if host::processor() != BOOT_CPU.load(Ordering::Relaxed) {
        return None;
    }
    let pointer = stack::pointer();
    let threads = gpu.mesa_threads.borrow();
    let (_, thread) = threads.range(..=pointer).next_back()?;
    (pointer < thread.top).then_some(*thread)
}

/// A posted call, which its caller keeps until `done`, unless it withdrew
/// it ([`Claim`]): a withdrawn call is boxed, and the task that meets it
/// frees it ([`release_withdrawn`]).
struct Request {
    operation: Operation,
    next: Cell<*const Request>,
    result: AtomicI64,
    done: AtomicU32,
    wake_task: bool,
    /// The TSC at posting, for the report of calls that wait.
    posted: u64,
    /// Whether a task took it, or its poster withdrew it.
    claim: Claim,
}

impl Request {
    fn new(operation: Operation) -> Self {
        Self {
            operation,
            next: Cell::new(ptr::null()),
            result: AtomicI64::new(0),
            done: AtomicU32::new(0),
            wake_task: false,
            posted: host::rdtsc(),
            claim: Claim::new(),
        }
    }

    /// Whether its poster may withdraw it once its wait passes
    /// [`posted_call::WITHDRAW_MS`]: a call whose absence its caller answers
    /// as a failure. A thread, a wake and the stop are waited for.
    fn withdrawable(&self) -> bool {
        !matches!(
            self.operation,
            Operation::Thread(_) | Operation::Wake | Operation::Shutdown
        )
    }

    fn complete(&self, result: i64) -> bool {
        let wake_task = self.wake_task;
        self.result.store(result, Ordering::Relaxed);
        self.done.store(1, Ordering::Release);
        wake_task
    }

    fn refuse(&self, result: i64) -> bool {
        if let Operation::Thread(thread) = self.operation {
            // SAFETY: no original child executed the request or took its box.
            drop(unsafe { Box::from_raw(thread) });
        }
        self.complete(result)
    }
}

/// The posted calls, newest first.
static REQUESTS: AtomicPtr<Request> = AtomicPtr::new(ptr::null_mut());
/// The mappings clients hold. A fault with none is no client's, so it reports
/// on the spot instead of waiting for the boot PE, which may be waiting for it.
static MAPPINGS: AtomicUsize = AtomicUsize::new(0);

fn post(request: &Request) {
    let this = ptr::from_ref(request).cast_mut();
    let mut head = REQUESTS.load(Ordering::Relaxed);
    loop {
        request.next.set(head);
        match REQUESTS.compare_exchange_weak(head, this, Ordering::Release, Ordering::Relaxed) {
            Ok(_) => return,
            Err(current) => head = current,
        }
    }
}

/// Stops the GPUs' drivers within `seconds`, as a reset is safest for a card
/// whose GSP has unloaded, and says whether they did. A GSP that does not answer
/// its unload stays running. On the boot CPU the deadline is seen only between
/// Linux's sleeps, so nouveau bounds its own unload wait in time
/// (`r535_gsp_unload_wait`, 4 s); its poll-counted RPC wait spun for 80-97 s.
pub(super) fn shutdown(seconds: u64) -> bool {
    // A wait that gives up leaves this with the Linux task.
    unload(
        Box::leak(Box::new(Request::new(Operation::Shutdown))),
        seconds,
    )
}

/// [`shutdown`] for a fatal stop, which allocates nothing: the stop never
/// returns, so its stack keeps the request for a wait that gives up.
pub(super) fn shutdown_for_fatal(seconds: u64) -> bool {
    unload(&Request::new(Operation::Shutdown), seconds)
}

fn unload(request: &Request, seconds: u64) -> bool {
    let Some(gpu) = installed() else {
        return true;
    };
    let deadline = host::rdtsc().saturating_add(seconds.saturating_mul(gpu.tsc_hz));
    wait(gpu, request, deadline);
    request.done.load(Ordering::Acquire) != 0
}

/// A dma-buf of `runs` and the host's mapping of them, for the GPU's service.
pub(super) fn export(runs: &[client::Run]) -> Option<(c_int, NonNull<u8>)> {
    let address = map_runs(runs)?;
    let fd = submit(Operation::Export(runs.as_ptr(), runs.len()));
    let Ok(fd) = c_int::try_from(fd).and_then(|fd| u32::try_from(fd).map(|_| fd)) else {
        crate::output::arguments(format_args!("GPU export of {} runs: {fd}\n", runs.len()));
        let bytes = runs.iter().map(|run| run.bytes).sum();
        unmap_export(address.as_ptr() as u64, bytes);
        return None;
    };
    Some((fd, address))
}

/// The host's mapping of `runs`, in one piece.
fn map_runs(runs: &[client::Run]) -> Option<NonNull<u8>> {
    let gpu = installed()?;
    let pages: u64 = runs.iter().map(|run| run.bytes / PAGE).sum();
    let address = gpu.window.reserve(pages as usize)?;
    let mut next = address;
    for run in runs {
        for page in 0..run.bytes / PAGE {
            if !gpu
                .window
                .install(next, run.physical + page * PAGE, Cache::WriteBack)
            {
                unmap_export(address, pages * PAGE);
                return None;
            }
            next += PAGE;
        }
    }
    NonNull::new(address as *mut u8)
}

/// Ends the host's mapping of an export.
pub(super) fn unmap_export(address: u64, bytes: u64) {
    let (Some(gpu), pages) = (installed(), (bytes / PAGE) as usize) else {
        return;
    };
    // A processor that kept the translation keeps the addresses too.
    if gpu.window.remove(address, pages) {
        gpu.window.release(address, pages);
    }
}

/// Closes a descriptor Linux gave out.
pub(super) fn close_descriptor(fd: c_int) {
    let result = submit(Operation::Close(fd));
    if result < 0 {
        // This void exporter/display cleanup contract cannot return the error
        // or prove that its caller will keep the original physical owner.
        // Stop before releasing that owner's stack, export mapping or memory.
        output::fatal(format_args!(
            "GPU descriptor {fd} cleanup refused ({result}); retaining its owners"
        ));
    }
}

/// Run `operation` on a Linux task.
///
/// # Safety
/// Entered in the owner, on a task; `operation`'s pointers are its caller's.
unsafe fn perform(gpu: &Installed, operation: Operation) -> i64 {
    // SAFETY: the function's own contract for every arm.
    unsafe {
        match operation {
            Operation::Open(minor) => stela_drivers::gpu::drm::open(minor).into(),
            Operation::Close(fd) => client::close(fd).into(),
            Operation::Duplicate(fd) => client::duplicate(fd).into(),
            Operation::Device(fd) => client::device(fd),
            Operation::Control {
                fd,
                request,
                argument,
            } => client::control(fd, request, argument),
            Operation::Map { fd, offset, size } => {
                let pages = size / PAGE as usize;
                let Some(address) = gpu.window.reserve(pages) else {
                    return ENOMEM;
                };
                match client::map(fd, address, size, offset) {
                    Ok(mapping) => {
                        gpu.mappings.borrow_mut().insert(address, (mapping, pages));
                        MAPPINGS.fetch_add(1, Ordering::Relaxed);
                        address as i64
                    }
                    Err(error) => {
                        gpu.window.release(address, pages);
                        error.into()
                    }
                }
            }
            Operation::Unmap(address) => {
                let Some((mapping, pages)) = gpu.mappings.borrow_mut().remove(&address) else {
                    return EFAULT;
                };
                client::unmap(mapping);
                MAPPINGS.fetch_sub(1, Ordering::Relaxed);
                // A processor that kept the translation keeps the addresses too.
                if gpu.window.remove(address, pages) {
                    gpu.window.release(address, pages);
                }
                0
            }
            Operation::Fault { address, write } => {
                let mappings = gpu.mappings.borrow();
                let found = mappings.range(..=address).next_back();
                let Some((&start, &(mapping, pages))) = found else {
                    return EFAULT;
                };
                if address >= start + pages as u64 * PAGE || !client::fault(mapping, address, write)
                {
                    return EFAULT;
                }
                0
            }
            Operation::Thread(_) => unreachable!("a thread runs in its original managed child"),
            Operation::Export(runs, count) => {
                client::export(slice::from_raw_parts(runs, count)).into()
            }
            Operation::Wake => {
                client::wake();
                0
            }
            Operation::Shutdown => {
                let gpu = installed().expect("requests are posted to a running environment");
                // A completion waiter's file is a nouveau client: freed after
                // the stop began, its RM objects outlive GSP's unload.
                if !super::waiter::wait_gone() {
                    output::arguments(format_args!(
                        "GPU completion waiter still open at the stop\n"
                    ));
                }
                gpu.start
                    ._attached
                    .iter()
                    .for_each(|attached| attached.shutdown());
                0
            }
        }
    }
}

/// How long a worker waits for a call before it leaves, while another stays.
const IDLE_SECONDS: u64 = 10;

/// The word idle workers sleep on, bumped as calls are queued.
static ARRIVALS: AtomicU32 = AtomicU32::new(0);

/// The Linux tasks that serve posted calls, and Mesa's threads. On the boot CPU.
///
/// A task costs Linux's kthreadd round trip and one of its few stacks, so
/// workers stay and take calls from one queue: a call waits for a worker
/// instead of failing for one. The pool grows while each worker is busy, as
/// one may sleep on the GPU, and shrinks back to one worker when idle.
#[derive(Default)]
pub(super) struct Service {
    children: RefCell<Vec<Child>>,
    /// Calls no worker has taken yet, oldest first.
    queue: RefCell<VecDeque<NonNull<Request>>>,
    /// Workers accepted and not yet left.
    workers: Cell<usize>,
    /// Workers accepted and not yet running.
    starting: Cell<usize>,
    /// Workers waiting for a call.
    idle: Cell<usize>,
    /// Whether a task waiting in Linux must look at its word again.
    wake_task: Cell<bool>,
}

impl Service {
    fn wake(&self, wake: bool) {
        self.wake_task.set(self.wake_task.get() || wake);
    }

    /// Answers every queued call with `result`, as no worker is left to.
    fn refuse_queued(&self, result: i64) {
        let queued = core::mem::take(&mut *self.queue.borrow_mut());
        for request in queued {
            // SAFETY: a queued call's poster keeps it until it is done or
            // withdrawn, and then the call is this task's to free.
            if unsafe { release_withdrawn(request) } {
                continue;
            }
            // SAFETY: as above; taken, so its poster waits for it.
            self.wake(unsafe { request.as_ref() }.refuse(result));
        }
    }
}

/// An accepted original child: a worker, or a Mesa thread whose request waits
/// for it to start.
struct Child {
    job: Pending,
    ran: Rc<Cell<bool>>,
    thread: Option<NonNull<Request>>,
}

struct Worker {
    ran: Rc<Cell<bool>>,
}

// SAFETY: a worker takes each call off the queue before performing it, and
// the call's poster keeps it until done. The job holds nothing else.
unsafe impl ManagedJob for Worker {
    fn run(self: Pin<&mut Self>) {
        self.ran.set(true);
        let gpu = installed().expect("workers have a running environment");
        let service = &gpu.service;
        service.starting.set(service.starting.get() - 1);
        let idle_ticks = IDLE_SECONDS.saturating_mul(gpu.tsc_hz);
        let mut idle_since = None;
        loop {
            // Read before the queue, so a call queued after it ends the wait.
            let seen = ARRIVALS.load(Ordering::Acquire);
            let next = service.queue.borrow_mut().pop_front();
            if let Some(request) = next {
                idle_since = None;
                // SAFETY: a queued call's poster keeps it until it is done
                // or withdrawn, and then the call is this worker's to free.
                unsafe { serve(gpu, request) };
                continue;
            }
            let now = host::rdtsc();
            let since = *idle_since.get_or_insert(now);
            if now.wrapping_sub(since) >= idle_ticks && service.workers.get() > 1 {
                service.workers.set(service.workers.get() - 1);
                return;
            }
            service.idle.set(service.idle.get() + 1);
            // SAFETY: an original task, sleeping on a static word.
            unsafe { client::wait(ARRIVALS.as_ptr(), seen, IDLE_SECONDS * 1_000_000_000) };
            service.idle.set(service.idle.get() - 1);
        }
    }
}

/// The calls workers are performing: each slot's request code plus one
/// (zero when free) and when it started.
static SERVING: [(AtomicU32, AtomicU64); 32] =
    [const { (AtomicU32::new(0), AtomicU64::new(0)) }; 32];

/// Frees `request` when its poster withdrew it, which no task then performs;
/// else takes it to perform. Whether it was freed.
///
/// # Safety
/// `request` is a live posted call this task met; a withdrawn one is boxed
/// and no one else frees it.
unsafe fn release_withdrawn(request: NonNull<Request>) -> bool {
    // SAFETY: the caller's.
    if unsafe { request.as_ref() }.claim.take() {
        return false;
    }
    // SAFETY: withdrawn calls are boxed by `submit`, and the poster left it.
    drop(unsafe { Box::from_raw(request.as_ptr()) });
    true
}

/// Performs `request` on this task and answers its poster, unless its poster
/// withdrew it.
///
/// # Safety
/// `request` is a queued call this worker took off the queue.
unsafe fn serve(gpu: &Installed, request: NonNull<Request>) {
    // SAFETY: the caller's.
    if unsafe { release_withdrawn(request) } {
        return;
    }
    // SAFETY: taken, so its poster keeps it until it is done.
    let request = unsafe { request.as_ref() };
    let code = request_code(&request.operation);
    let started = host::rdtsc();
    let slot = SERVING.iter().find(|slot| {
        slot.0
            .compare_exchange(
                0,
                code.wrapping_add(1),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_ok()
    });
    if let Some(slot) = slot {
        slot.1.store(started, Ordering::Relaxed);
    }
    let shot = crate::native_mmio::shootdown::count();
    // SAFETY: an original task entered in the canonical Linux owner. The
    // poster keeps the operation's inputs until it is done.
    let result = unsafe { perform(gpu, request.operation) };
    if let Some(slot) = slot {
        slot.0.store(0, Ordering::Relaxed);
    }
    count(&QUEUED, code, started.wrapping_sub(request.posted));
    let took = host::rdtsc().wrapping_sub(started);
    count(&SERVED, code, took);
    // A report between the two reads cleared the count.
    let now_shot = crate::native_mmio::shootdown::count();
    let during = now_shot.checked_sub(shot).unwrap_or(now_shot);
    note_slowest(code, took, during);
    if request.complete(result) {
        // SAFETY: the same source task owns the Linux wait queue.
        unsafe { client::wake() };
    }
}

struct ThreadJob {
    request: NonNull<Request>,
    ran: Rc<Cell<bool>>,
    thread: Option<(Box<Thread>, Stack)>,
}

// SAFETY: the poster keeps the request until its completion, after which
// the job does not touch it. The thread's storage stays with the job.
unsafe impl ManagedJob for ThreadJob {
    fn run(mut self: Pin<&mut Self>) {
        self.ran.set(true);
        let gpu = installed().expect("threads have a running environment");
        // SAFETY: the request stays live until it is completed below.
        let request = unsafe { self.request.as_ref() };
        let Operation::Thread(thread) = request.operation else {
            unreachable!("a thread job carries a thread request")
        };
        // SAFETY: this original child is the sole consumer of the box.
        let thread = unsafe { Box::from_raw(thread) };
        let result = match Stack::map(gpu.window, THREAD_STACK_BYTES) {
            Some(stack) => {
                self.thread = Some((thread, stack));
                0
            }
            None => EAGAIN,
        };
        if request.complete(result) {
            // SAFETY: the same source task owns the Linux wait queue.
            unsafe { client::wake() };
        }
        if let Some((thread, stack)) = &self.thread {
            run_thread(gpu, thread, stack);
        }
    }
}

fn join_children(gpu: &Installed) {
    let service = &gpu.service;
    let mut children = service.children.take();
    let mut index = 0;
    while index < children.len() {
        if !children[index].job.status().joined {
            index += 1;
            continue;
        }
        let Child { job, ran, thread } = children.swap_remove(index);
        // SAFETY: same taskless dispatcher, after all active C/native frames
        // have returned, with no source or metadata loan.
        match unsafe { job.try_join() } {
            Ok(joined) => {
                let outcome = joined.outcome();
                // A task that ran returns; one that did not was never created.
                let expected = if ran.get() {
                    outcome == Outcome::CallbackReturned
                } else {
                    matches!(outcome, Outcome::CreationFailed(_) | Outcome::Skipped(_))
                };
                if !expected {
                    output::fatal(format_args!("a Linux GPU task ended early: {outcome:?}"));
                }
                if ran.get() {
                    continue;
                }
                match thread {
                    // SAFETY: the poster still waits and no callback took the box.
                    Some(request) => service.wake(unsafe { request.as_ref() }.refuse(EAGAIN)),
                    None => {
                        service.starting.set(service.starting.get() - 1);
                        service.workers.set(service.workers.get() - 1);
                        if service.workers.get() == 0 {
                            service.refuse_queued(ENOMEM);
                        }
                    }
                }
            }
            Err(job) => {
                children.push(Child { job, ran, thread });
                break;
            }
        }
    }
    service.children.replace(children);
}

/// Starts a worker for each queued call beyond those idle or starting. With
/// none to be had, the calls wait for a busy worker, or with no worker at all
/// fail as out of memory.
fn grow(gpu: &Installed) -> Result<(), WorkError> {
    let service = &gpu.service;
    while service.queue.borrow().len() > service.idle.get() + service.starting.get() {
        let ran = Rc::new(Cell::new(false));
        let worker = Box::pin(Worker { ran: ran.clone() });
        // SAFETY: the exact dispatcher and installed owner retain the
        // original child, which takes only queued calls their posters keep.
        match unsafe { gpu.threads.submit(worker) } {
            Ok(job) => {
                service.workers.set(service.workers.get() + 1);
                service.starting.set(service.starting.get() + 1);
                service.children.borrow_mut().push(Child {
                    job,
                    ran,
                    thread: None,
                });
            }
            Err(refusal) => match refusal.error {
                SubmitError::Capacity => {
                    if service.workers.get() == 0 {
                        service.refuse_queued(ENOMEM);
                    }
                    break;
                }
                SubmitError::Context(error) => return Err(error),
            },
        }
    }
    Ok(())
}

/// Starts Mesa's thread on a task of its own, or refuses it for want of one.
fn start_thread(gpu: &Installed, request: &Request) -> Result<(), WorkError> {
    let ran = Rc::new(Cell::new(false));
    let job = Box::pin(ThreadJob {
        request: NonNull::from(request),
        ran: ran.clone(),
        thread: None,
    });
    // SAFETY: the exact dispatcher and installed owner retain the original
    // child, and the poster keeps the request until it is completed.
    match unsafe { gpu.threads.submit(job) } {
        Ok(job) => gpu.service.children.borrow_mut().push(Child {
            job,
            ran,
            thread: Some(NonNull::from(request)),
        }),
        Err(refusal) => match refusal.error {
            SubmitError::Capacity => gpu.service.wake(request.refuse(EAGAIN)),
            SubmitError::Context(error) => return Err(error),
        },
    }
    Ok(())
}

unsafe extern "C" fn wake_waiters(_: *mut c_void) {
    // SAFETY: a reserved service leaf is already entered in the exact owner.
    unsafe { client::wake() };
}

/// An ioctl by its request code; every other request as zero.
fn request_code(operation: &Operation) -> u32 {
    match *operation {
        Operation::Control { request, .. } => request,
        _ => 0,
    }
}

/// Request counts and ticks by request code plus one, in slots that fill in
/// order and stay claimed, as the gate's call counts do.
type Counts = [(AtomicU32, AtomicU64, AtomicU64); 64];

fn count(counts: &Counts, code: u32, ticks: u64) {
    let key = code.wrapping_add(1);
    if let Some(slot) = counts.iter().find(|slot| {
        match slot
            .0
            .compare_exchange(0, key, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => true,
            Err(claimed) => claimed == key,
        }
    }) {
        slot.1.fetch_add(1, Ordering::Relaxed);
        slot.2.fetch_add(ticks, Ordering::Relaxed);
    }
}

/// Each code's count and ticks since the last call, most ticks first.
fn take(counts: &Counts) -> Vec<(u32, u64, u64)> {
    let mut taken: Vec<_> = counts
        .iter()
        .filter_map(|slot| {
            let count = slot.1.swap(0, Ordering::Relaxed);
            (count != 0).then(|| {
                (
                    slot.0.load(Ordering::Relaxed).wrapping_sub(1),
                    count,
                    slot.2.swap(0, Ordering::Relaxed),
                )
            })
        })
        .collect();
    taken.sort_unstable_by_key(|&(_, _, ticks)| core::cmp::Reverse(ticks));
    taken
}

/// A wake leaf uses the service task's existing owner/process scope. Its
/// requester keeps the stack record live until the release acknowledgement.
unsafe extern "C" fn serve_wake(context: *mut c_void) {
    // SAFETY: start deferred this Wake request and its poster waits for done.
    let request = unsafe { &*context.cast::<Request>() };
    // SAFETY: the service owns a real entered process continuation; waking
    // waiters is bounded and cannot sleep or enter another owner.
    unsafe { client::wake() };
    request.complete(0);
}

/// Queues posted calls for the workers, oldest first, and starts Mesa's
/// threads. Wake leaves use the reserved service.
pub(super) fn start(gpu: &Installed) -> Result<(), WorkError> {
    CHILDREN_STALE.store(false, Ordering::Relaxed);
    join_children(gpu);
    let service = &gpu.service;
    if WAKE_ASKED.swap(false, Ordering::AcqRel) {
        service.wake(true);
    }
    let mut request = REQUESTS
        .swap(ptr::null_mut(), Ordering::Acquire)
        .cast_const();
    let mut deferred = false;
    let mut queued = false;
    // The posted list is newest first, so each goes ahead of the one before.
    let oldest = service.queue.borrow().len();
    // SAFETY: captured requests stay live until their completion is published.
    while let Some(current) = unsafe { request.as_ref() } {
        let next = current.next.get();
        match current.operation {
            Operation::Wake => {
                // SAFETY: the poster holds the record until this reserved leaf runs.
                match unsafe {
                    gpu.owner
                        .environment()
                        .defer_service(serve_wake, request.cast_mut().cast())
                } {
                    Ok(()) => {}
                    Err(WorkError::Full) => {
                        post(current);
                        deferred = true;
                    }
                    Err(error) => return Err(error),
                }
            }
            Operation::Thread(_) => start_thread(gpu, current)?,
            // SAFETY: the posted call met here, which nothing queued yet;
            // `request` is its non-null pointer.
            _ if current.claim.withdrawn()
                && unsafe { release_withdrawn(NonNull::new_unchecked(request.cast_mut())) } => {}
            _ => {
                count(
                    &TAKEN,
                    request_code(&current.operation),
                    host::rdtsc().wrapping_sub(current.posted),
                );
                service
                    .queue
                    .borrow_mut()
                    .insert(oldest, NonNull::from(current));
                queued = true;
            }
        }
        request = next;
    }
    if queued {
        ARRIVALS.fetch_add(1, Ordering::Release);
        service.wake(service.idle.get() != 0);
    }
    grow(gpu)?;
    if service.wake_task.get() {
        // SAFETY: this wake needs no caller storage and cannot sleep.
        match unsafe {
            gpu.owner
                .environment()
                .defer_service(wake_waiters, ptr::null_mut())
        } {
            Ok(()) => service.wake_task.set(false),
            Err(WorkError::Full) => deferred = true,
            Err(error) => return Err(error),
        }
    }
    if deferred {
        gpu.events.notify()?;
    }
    Ok(())
}

fn submit(operation: Operation) -> i64 {
    let Some(gpu) = installed() else {
        return {
            if let Operation::Thread(thread) = operation {
                // SAFETY: the thread was never accepted.
                drop(unsafe { Box::from_raw(thread) });
            }
            ENODEV
        };
    };
    if let Some(thread) = current_thread(gpu) {
        let mut result = 0;
        let mut call = || {
            if matches!(operation, Operation::Thread(_)) {
                let mut request = Request::new(operation);
                request.wake_task = true;
                post(&request);
                if let Err(error) = gpu.events.notify() {
                    output::fatal(format_args!(
                        "the host's Linux environment failed: {error:?}"
                    ));
                }
                // SAFETY: Mesa is back on its original source task stack.
                // The completion word stays live through the Linux wait.
                unsafe { client::wait(request.done.as_ptr(), 0, u64::MAX) };
                result = request.result.load(Ordering::Acquire);
            } else {
                // SAFETY: back on the original source task's Linux stack.
                result = unsafe { perform(gpu, operation) };
            }
        };
        // SAFETY: the task runs nothing else below its thread's frame.
        unsafe { stack::on_stack(thread.task, &mut call) };
        return result;
    }
    let code = request_code(&operation);
    // Boxed, so that a withdrawn call outlives this frame with the task
    // that meets it.
    let request = NonNull::from(Box::leak(Box::new(Request::new(operation))));
    // SAFETY: this frame owns the box until it frees it or withdraws the call.
    let posted = unsafe { request.as_ref() }.posted;
    // SAFETY: as above.
    let Some(result) = posted_wait(gpu, unsafe { request.as_ref() }, code) else {
        return ETIMEDOUT;
    };
    // SAFETY: done, so no task touches it again; this frame's box.
    drop(unsafe { Box::from_raw(request.as_ptr()) });
    let waited = host::rdtsc().wrapping_sub(posted);
    if let Some(mine) = MINE.get(host::processor() as usize) {
        mine.0.fetch_add(1, Ordering::Relaxed);
        mine.1.fetch_add(waited, Ordering::Relaxed);
    }
    count(&POSTED, code, waited);
    result
}

/// Calls withdrawn so far, for the report of the first and of each power of
/// two after it.
static WITHDRAWN: AtomicU64 = AtomicU64::new(0);

/// Posts `request` and waits for its result, within the bounds of
/// [`posted_call`]: None when it was withdrawn, no task having taken it by
/// [`posted_call::WITHDRAW_MS`]; it then stays with the task that meets it.
/// A call a task performs past that is reported and waited for, and one
/// still running at [`posted_call::STOP_MS`] stops the boot, naming it.
fn posted_wait(gpu: &Installed, request: &Request, code: u32) -> Option<i64> {
    let millis = (gpu.tsc_hz / 1000).max(1);
    let posted = request.posted;
    let ms = || host::rdtsc().wrapping_sub(posted) / millis;
    let done = || request.done.load(Ordering::Acquire) != 0;
    let withdraw_at = posted.saturating_add(posted_call::WITHDRAW_MS * millis);
    wait(gpu, request, withdraw_at);
    if !done() {
        if request.withdrawable() && request.claim.withdraw() {
            let count = WITHDRAWN.fetch_add(1, Ordering::Relaxed) + 1;
            if count.is_power_of_two() {
                output::arguments(format_args!(
                    "GPU call {code:#x} withdrawn after {} ms on PE {}: no task took it; \
                     answered ETIMEDOUT ({count} withdrawn)\n",
                    ms(),
                    host::processor()
                ));
                // The pool's state is the boot CPU's to read.
                if host::processor() == BOOT_CPU.load(Ordering::Relaxed)
                    && let Some(line) = pool_report(gpu.tsc_hz)
                {
                    output::arguments(format_args!("{line}\n"));
                }
            }
            return None;
        }
        output::arguments(format_args!(
            "GPU call {code:#x} still running after {} ms on PE {}, which waits for it\n",
            ms(),
            host::processor()
        ));
        spin(
            gpu,
            request,
            posted.saturating_add(posted_call::STOP_MS * millis),
        );
        if !done() {
            output::fatal(format_args!(
                "a GPU call ({code:#x}) still ran after {} ms",
                posted_call::STOP_MS
            ));
        }
    }
    Some(request.result.load(Ordering::Acquire))
}

/// Requests posted to the service task since [`take_posted`] by request
/// code: how many, and the TSC ticks their posters waited for them.
static POSTED: Counts = [const { (AtomicU32::new(0), AtomicU64::new(0), AtomicU64::new(0)) }; 64];

/// Posted requests the service task performed since [`take_served`] by
/// request code: how many, and the TSC ticks performing them took, which the
/// boot CPU spends unless the request sleeps.
static SERVED: Counts = [const { (AtomicU32::new(0), AtomicU64::new(0), AtomicU64::new(0)) }; 64];

/// Posted requests the service task performed since [`pool_report`] by
/// request code: the TSC ticks of the longest one, and the host's mapping
/// shootdowns while any of them ran, which a call that frees Linux mappings
/// (`iounmap`, `vfree`) or sleeps while another task does causes.
static SLOWEST: Counts = [const { (AtomicU32::new(0), AtomicU64::new(0), AtomicU64::new(0)) }; 64];

fn note_slowest(code: u32, ticks: u64, shootdowns: u64) {
    let key = code.wrapping_add(1);
    if let Some(slot) = SLOWEST.iter().find(|slot| {
        match slot
            .0
            .compare_exchange(0, key, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => true,
            Err(claimed) => claimed == key,
        }
    }) {
        slot.1.fetch_max(ticks, Ordering::Relaxed);
        slot.2.fetch_add(shootdowns, Ordering::Relaxed);
    }
}

/// `GPU SERVED LONGEST request:longest_us/shootdowns`: [`SLOWEST`] since the
/// last call, longest first, cleared.
fn slowest_line(tsc_hz: u64) -> Option<alloc::string::String> {
    use core::fmt::Write;
    const SHOWN: usize = 8;
    let mut rows: Vec<_> = SLOWEST
        .iter()
        .filter_map(|slot| {
            let longest = slot.1.swap(0, Ordering::Relaxed);
            let shootdowns = slot.2.swap(0, Ordering::Relaxed);
            (longest != 0).then(|| {
                (
                    slot.0.load(Ordering::Relaxed).wrapping_sub(1),
                    longest,
                    shootdowns,
                )
            })
        })
        .collect();
    if rows.is_empty() {
        return None;
    }
    rows.sort_unstable_by_key(|&(_, longest, _)| core::cmp::Reverse(longest));
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let mut line = alloc::string::String::from("GPU SERVED LONGEST request:longest_us/shootdowns");
    for &(request, longest, shootdowns) in rows.iter().take(SHOWN) {
        let _ = write!(line, " {request:#x}:{}/{shootdowns}", micros(longest));
    }
    Some(line)
}

/// The request codes of up to four calls workers are performing now, each
/// plus one, zero for none.
pub(super) fn serving() -> [u32; 4] {
    let mut codes = [0; 4];
    let busy = SERVING
        .iter()
        .map(|slot| slot.0.load(Ordering::Relaxed))
        .filter(|&code| code != 0);
    for (into, code) in codes.iter_mut().zip(busy) {
        *into = code;
    }
    codes
}

/// Posted requests a worker started since [`take_queued`] by request code:
/// how many, and the TSC ticks from posting until a worker took each one,
/// which is the poster's wait that is not the call's own work.
static QUEUED: Counts = [const { (AtomicU32::new(0), AtomicU64::new(0), AtomicU64::new(0)) }; 64];

/// Posted requests the boot CPU took for its workers since [`take_taken`]
/// by request code: how many, and the TSC ticks from posting until then.
static TAKEN: Counts = [const { (AtomicU32::new(0), AtomicU64::new(0), AtomicU64::new(0)) }; 64];

/// Requests each processor posted since it last called [`take_mine`]: how
/// many, and the TSC ticks it waited for them.
static MINE: [(AtomicU64, AtomicU64); PROCESSORS] =
    [const { (AtomicU64::new(0), AtomicU64::new(0)) }; PROCESSORS];

/// How many requests this processor posted since it last asked, and the
/// ticks it waited for them: a gate call's share of the boot CPU's work.
pub(super) fn take_mine() -> (u64, u64) {
    MINE.get(host::processor() as usize).map_or((0, 0), |mine| {
        (
            mine.0.swap(0, Ordering::Relaxed),
            mine.1.swap(0, Ordering::Relaxed),
        )
    })
}

/// The requests posted since the last call: each request code with its
/// count and the ticks they waited, most waited first.
pub(super) fn take_posted() -> Vec<(u32, u64, u64)> {
    take(&POSTED)
}

/// The requests the service task performed since the last call: each
/// request code with its count and the ticks performing them took.
pub(super) fn take_served() -> Vec<(u32, u64, u64)> {
    take(&SERVED)
}

/// The requests the boot CPU took since the last call: each request code
/// with its count and the ticks from posting until it took them.
pub(super) fn take_taken() -> Vec<(u32, u64, u64)> {
    take(&TAKEN)
}

/// The requests a worker started since the last call: each request code
/// with its count and the ticks from posting until a worker took them.
pub(super) fn take_queued() -> Vec<(u32, u64, u64)> {
    take(&QUEUED)
}

/// The display's presents since [`take_presents`]: how many, the TSC ticks
/// they took, the longest, how many ran on the boot CPU, and the ticks of
/// those that began on the boot CPU with a posted call waiting for it.
static PRESENTS: [AtomicU64; 5] = [const { AtomicU64::new(0) }; 5];

/// Runs the display present `present`, counting its time and whether it held
/// the boot CPU while a posted call waited for it.
pub(in crate::engine) fn timed_present<T>(present: impl FnOnce() -> T) -> T {
    let blocking = on_boot() && !REQUESTS.load(Ordering::Relaxed).is_null();
    let start = host::rdtsc();
    let result = present();
    count_present(host::rdtsc().wrapping_sub(start), blocking);
    result
}

fn on_boot() -> bool {
    host::processor() == BOOT_CPU.load(Ordering::Relaxed)
}

/// Counts a display present this processor made in `ticks`, in pieces
/// between which a posted call could run.
pub(in crate::engine) fn present_took(ticks: u64) {
    count_present(ticks, false);
}

fn count_present(ticks: u64, blocking: bool) {
    let boot = on_boot();
    let [count, total, longest, on_boot, blocked] = &PRESENTS;
    count.fetch_add(1, Ordering::Relaxed);
    total.fetch_add(ticks, Ordering::Relaxed);
    longest.fetch_max(ticks, Ordering::Relaxed);
    on_boot.fetch_add(u64::from(boot), Ordering::Relaxed);
    if blocking {
        blocked.fetch_add(ticks, Ordering::Relaxed);
    }
}

/// Whether a posted call waits for the boot CPU to take it.
pub(super) fn posted_waiting() -> bool {
    !REQUESTS.load(Ordering::Relaxed).is_null()
}

/// Whether Linux ran steps since [`start`] last looked at the children: an
/// entry other than the pump's ([`wait`], startup) leaves them unlooked.
static CHILDREN_STALE: AtomicBool = AtomicBool::new(true);

/// Whether a pump's first pass retires tasks and runs [`start`]: a call is
/// posted, a PE asked for a wake, a task waits for a wake, calls wait for workers, or steps ran
/// since the last look, which may have ended a child. Without any, both
/// would find nothing.
pub(super) fn start_due() -> bool {
    posted_waiting()
        || WAKE_ASKED.load(Ordering::Relaxed)
        || CHILDREN_STALE.load(Ordering::Relaxed)
        || installed().is_some_and(|gpu| {
            let service = &gpu.service;
            service.wake_task.get()
                || service.queue.borrow().len() > service.idle.get() + service.starting.get()
        })
}

/// [`PRESENTS`] since the last call, cleared.
pub(super) fn take_presents() -> [u64; 5] {
    PRESENTS
        .each_ref()
        .map(|value| value.swap(0, Ordering::Relaxed))
}

/// The worker pool's state when a call waits or runs long, then each
/// request's longest call since the last report ([`slowest_line`]), one line
/// each. On the boot CPU, between pumps.
pub(super) fn pool_report(tsc_hz: u64) -> Option<alloc::string::String> {
    let workers = workers_line(tsc_hz);
    let slowest = slowest_line(tsc_hz);
    match (workers, slowest) {
        (Some(mut workers), Some(slowest)) => {
            workers.push('\n');
            workers.push_str(&slowest);
            Some(workers)
        }
        (workers, slowest) => workers.or(slowest),
    }
}

/// The worker pool's state when a call waits or runs long: its workers,
/// idle and starting ones, queued calls with the oldest one's wait, and each
/// call still running with how long it has.
fn workers_line(tsc_hz: u64) -> Option<alloc::string::String> {
    use core::fmt::Write;
    let gpu = installed()?;
    let service = &gpu.service;
    let now = host::rdtsc();
    let ms = |since: u64| now.wrapping_sub(since) / (tsc_hz / 1000).max(1);
    let queue = service.queue.borrow();
    // SAFETY: a queued call's poster keeps it until it is done.
    let oldest = queue
        .front()
        .map_or(0, |request| ms(unsafe { request.as_ref() }.posted));
    let mut serving = alloc::string::String::new();
    for slot in &SERVING {
        let code = slot.0.load(Ordering::Relaxed);
        let took = ms(slot.1.load(Ordering::Relaxed));
        if code != 0 && took >= 1000 {
            let _ = write!(serving, " {:#x}:{took}ms", code.wrapping_sub(1));
        }
    }
    if queue.is_empty() && serving.is_empty() {
        return None;
    }
    let mut line = alloc::string::String::new();
    let _ = write!(
        line,
        "GPU WORKERS workers={} idle={} starting={} queued={} oldest_ms={oldest} pending={} \
         timer_failure={:?} serving:{serving}",
        service.workers.get(),
        service.idle.get(),
        service.starting.get(),
        queue.len(),
        gpu.events.pending.load(Ordering::Relaxed),
        stela_linux::service::timer_delivery_failure(),
    );
    Some(line)
}

/// Posts `request` to the service and waits for it, until the host counter
/// reaches `deadline`.
fn wait(gpu: &Installed, request: &Request, deadline: u64) {
    post(request);
    if let Err(error) = gpu.events.notify() {
        output::fatal(format_args!(
            "the host's Linux environment failed: {error:?}"
        ));
    }
    spin(gpu, request, deadline);
}

/// Waits for the posted `request`, until the host counter reaches
/// `deadline`: a PE spins, the boot CPU runs the environment.
fn spin(gpu: &Installed, request: &Request, deadline: u64) {
    let waiting = || request.done.load(Ordering::Acquire) == 0 && host::rdtsc() < deadline;
    if host::processor() != BOOT_CPU.load(Ordering::Relaxed) {
        while waiting() {
            // The boot CPU's work may be retiring a mapping, which waits for this.
            crate::native_mmio::shootdown::service();
            hint::spin_loop();
        }
    } else {
        let ran = gpu.work.dispatch_linux(|runtime| {
            while waiting() {
                gpu.advance_children();
                start(gpu)?;
                if waiting() {
                    runtime.step_or_idle()?;
                    CHILDREN_STALE.store(true, Ordering::Relaxed);
                }
            }
            Ok(())
        });
        if let Err(error) = ran {
            output::fatal(format_args!("the host's Linux environment failed: {error}"));
        }
    }
}

/// A file of the DRM node at `minor`.
pub(super) fn open_node(minor: u32) -> Option<c_int> {
    let fd = submit(Operation::Open(minor)) as c_int;
    (fd >= 0).then_some(fd)
}

/// The ioctl `request` on `fd`: zero or a negative errno.
pub(super) fn control(fd: c_int, request: u32, argument: *mut c_void) -> i64 {
    submit(Operation::Control {
        fd,
        request,
        argument,
    })
}

/// Maps `size` bytes of `fd` at `offset`.
pub(super) fn map(fd: c_int, offset: u64, size: usize) -> Option<NonNull<u8>> {
    let result = submit(Operation::Map { fd, offset, size });
    match stela_boot_core::page_window::mapped_address(result) {
        Ok(address) => NonNull::new(address as *mut u8),
        Err(error) => {
            output::arguments(format_args!(
                "GPU map of {size} bytes at offset {offset:#x} refused: {error}\n"
            ));
            None
        }
    }
}

/// Removes the mapping [`map`] gave at `address`.
pub(super) fn unmap(address: NonNull<u8>) {
    let result = submit(Operation::Unmap(address.as_ptr() as u64));
    if result != 0 {
        output::arguments(format_args!("GPU unmap of {address:p} refused: {result}\n"));
    }
}

/// `NOUVEAU_GETPARAM_CHIPSET_ID` of the bound GPU at `index`.
pub(super) fn chipset(index: u32) -> Option<u16> {
    let fd = submit(Operation::Open(RENDER_MINOR + index)) as c_int;
    if fd < 0 {
        return None;
    }
    let mut parameter = GetParam {
        parameter: CHIPSET_ID,
        value: 0,
    };
    let result = submit(Operation::Control {
        fd,
        request: GET_PARAM,
        argument: (&raw mut parameter).cast(),
    });
    close_descriptor(fd);
    (result == 0).then_some(parameter.value as u16)
}

/// `NOUVEAU_GETPARAM_FB_SIZE`: the VRAM nouveau makes available.
const FB_SIZE: u64 = 8;

/// The VRAM bytes of the bound GPU at `index`, for the guest's `StelaHost`.
pub(in crate::engine) fn vram_bytes(index: u32) -> Option<u64> {
    let fd = submit(Operation::Open(RENDER_MINOR + index)) as c_int;
    if fd < 0 {
        return None;
    }
    let mut parameter = GetParam {
        parameter: FB_SIZE,
        value: 0,
    };
    let result = submit(Operation::Control {
        fd,
        request: GET_PARAM,
        argument: (&raw mut parameter).cast(),
    });
    close_descriptor(fd);
    (result == 0 && parameter.value != 0).then_some(parameter.value)
}

/// Serve a fault on a mapping that moved, as Linux unmaps a buffer it evicts.
/// False when `address` is not one a client mapped.
pub(in crate::engine) fn serve_fault(address: u64, write: bool) -> bool {
    installed().is_some_and(|gpu| {
        gpu.window.contains(address)
            && MAPPINGS.load(Ordering::Relaxed) != 0
            && submit(Operation::Fault { address, write }) == 0
    })
}

fn text(pointer: *const c_char) -> &'static str {
    // SAFETY: the C library's message, a literal.
    unsafe { CStr::from_ptr(pointer) }.to_str().unwrap_or("?")
}

/// The heap bytes the C library (Mesa, NAK, LLVM) holds, for `HEAP OWNERS`.
pub(in crate::engine) static LIBC_HEAP: stela_boot_core::heap::tally::Tally =
    stela_boot_core::heap::tally::Tally::new();

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_allocate(size: usize, alignment: usize) -> *mut u8 {
    match Layout::from_size_align(size, alignment) {
        Ok(layout) => {
            // SAFETY: the C library never asks for zero bytes: a block has a header.
            let pointer = unsafe { alloc(layout) };
            if !pointer.is_null() {
                LIBC_HEAP.allocated(size);
            }
            pointer
        }
        Err(_) => ptr::null_mut(),
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_release(pointer: *mut u8, size: usize, alignment: usize) {
    LIBC_HEAP.released(size);
    // SAFETY: the C library gives back what it got with this size and alignment.
    unsafe { dealloc(pointer, Layout::from_size_align_unchecked(size, alignment)) };
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_log(pointer: *const u8, size: usize) {
    // SAFETY: the C library's live text.
    let bytes = unsafe { slice::from_raw_parts(pointer, size) };
    if let Ok(line) = str::from_utf8(bytes) {
        output::arguments(format_args!("{line}"));
    }
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_fatal(message: *const c_char) -> ! {
    output::fatal(text(message))
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_memory_bytes() -> u64 {
    installed().map_or(0, |gpu| gpu.heap_bytes)
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_available_memory_bytes() -> u64 {
    installed().map_or(0, |gpu| {
        let usage = crate::ALLOCATOR.usage();
        let free = usage.total.saturating_sub(usage.live);
        gpu.heap_bytes.min(free as u64)
    })
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_now_ns() -> u64 {
    installed().map_or(0, |gpu| gpu.libc_clock.now_ns(host::rdtsc(), gpu.tsc_hz))
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_core() -> c_uint {
    installed()
        .and_then(current_thread)
        .map_or_else(host::processor, |thread| thread.core)
}

/// The core a thread of the platform's own runs as: past the C library's
/// threads, which never takes it.
const PLATFORM_THREAD_CORE: c_uint = (PROCESSORS + THREADS) as c_uint;

/// Runs `body` on a Linux task of its own, as Mesa's threads run: its Linux
/// calls run in place and sleep in Linux. Zero, or a negative errno when no
/// task could be had.
pub(super) fn spawn(body: unsafe extern "C" fn(*mut c_void), argument: *mut c_void) -> c_int {
    // SAFETY: the body takes the argument it is given, as a C thread's does.
    unsafe { stela_libc_thread_start(body, argument, PLATFORM_THREAD_CORE) }
}

/// The C library's monotonic clock, which absolute DRM deadlines name.
pub(super) fn now_ns() -> u64 {
    stela_libc_now_ns()
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_thread_start(
    body: unsafe extern "C" fn(*mut c_void),
    argument: *mut c_void,
    core: c_uint,
) -> c_int {
    let thread = Box::into_raw(Box::new(Thread {
        body,
        argument,
        core,
    }));
    submit(Operation::Thread(thread)) as c_int
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_wait(word: *const c_uint, value: c_uint, deadline: u64) {
    let Some(gpu) = installed() else {
        return hint::spin_loop();
    };
    let Some(thread) = current_thread(gpu) else {
        return super::gate::relax();
    };
    let sleeping = &SLEEPING[thread.core as usize - PROCESSORS];
    sleeping.store(word.cast_mut(), Ordering::SeqCst);
    let left = match deadline {
        u64::MAX => u64::MAX,
        _ => deadline.saturating_sub(gpu.libc_clock.now_ns(host::rdtsc(), gpu.tsc_hz)),
    };
    let mut sleep = || {
        // SAFETY: entered, on a task, and the C library's word outlives its wait.
        unsafe { client::wait(word, value, left) };
    };
    // SAFETY: the task runs nothing else below its thread's frame.
    unsafe { stack::on_stack(thread.task, &mut sleep) };
    sleeping.store(ptr::null_mut(), Ordering::SeqCst);
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_wake(word: *const c_uint) {
    // Against a sleeper's mark before it reads the word.
    fence(Ordering::SeqCst);
    let asleep = |slot: &AtomicPtr<c_uint>| slot.load(Ordering::SeqCst).cast_const() == word;
    if !SLEEPING.iter().any(asleep) {
        return;
    }
    // A guest PE goes on at once: the boot CPU's next pump wakes the
    // sleepers, which look at their words again. A sleeper that has not
    // slept yet finds its word changed and does not.
    if let Some(gpu) = installed()
        && host::processor() != BOOT_CPU.load(Ordering::Relaxed)
    {
        WAKE_ASKED.store(true, Ordering::Release);
        if let Err(error) = gpu.events.notify() {
            output::fatal(format_args!(
                "the host's Linux environment failed: {error:?}"
            ));
        }
        return;
    }
    submit(Operation::Wake);
}

/// A guest PE asked to wake the C library's sleepers ([`stela_libc_wake`]):
/// the next [`start`] does.
static WAKE_ASKED: AtomicBool = AtomicBool::new(false);

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_open(minor: c_uint) -> c_int {
    submit(Operation::Open(minor)) as c_int
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_close(fd: c_int) -> c_int {
    submit(Operation::Close(fd)) as c_int
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_dup(fd: c_int) -> c_int {
    submit(Operation::Duplicate(fd)) as c_int
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_rdev(fd: c_int) -> c_long {
    submit(Operation::Device(fd)) as c_long
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_ioctl(fd: c_int, request: c_ulong, argument: *mut c_void) -> c_int {
    submit(Operation::Control {
        fd,
        request: request as u32,
        argument,
    }) as c_int
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_mmap(fd: c_int, offset: u64, size: usize) -> *mut c_void {
    submit(Operation::Map { fd, offset, size }) as *mut c_void
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_munmap(address: *mut c_void, _size: usize) -> c_int {
    let mapped = installed().is_some_and(|gpu| gpu.window.contains(address as u64));
    if !mapped {
        return 0;
    }
    let result = submit(Operation::Unmap(address as u64));
    if result < 0 { result as c_int } else { 1 }
}

#[unsafe(no_mangle)]
extern "C" fn stela_libc_drm_devices() -> c_uint {
    installed().map_or(0, |gpu| gpu.start.bound.len() as c_uint)
}

#[unsafe(no_mangle)]
unsafe extern "C" fn stela_libc_drm_device(index: c_uint, device: *mut DrmDevice) {
    if let Some(found) = installed().and_then(|gpu| gpu.start.bound.get(index as usize)) {
        // SAFETY: the C library's record.
        unsafe { device.write(found.drm) };
    }
}
