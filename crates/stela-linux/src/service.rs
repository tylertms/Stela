//! The one canonical original Initial service on the work [`Runtime`].
//! Original workqueue.c/kthread.c own workers and managed source threads through
//! task::runtime's SAME native broker; there is no Environment worker pool.
//! Timer delivery and native IRQ notification are separate owned boundaries;
//! this service does not manufacture the selected original periodic tick.
//!
//! An interrupt only records its original epoch ([`IrqEvent::raise`]); the handlers
//! run on the service task, which sleeps until a line is raised or the next
//! timer is due (`compat/kernel/service.c`).
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
mod hosted;
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
pub use hosted::run_hosted_service;
pub(crate) mod initial;
mod timer;
/// First exact terminal timer-service failure, retained for diagnostics.
/// Observation only; it grants no recovery, batch or completion authority.
pub fn timer_delivery_failure() -> Option<WorkError> {
    timer::failure()
}
/// Run the original Linux timer phases in a mounted hosted service callback.
/// The returned value is the actual remaining source deadline.
///
/// # Safety
/// The caller retains this exact canonical source owner and its active
/// Initial/Managed binding for this service runtime. The original callbacks
/// are bounded and cannot sleep, reenter delivery, or retain the opaque loan.
/// An error refuses delivery before any callback runs.
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
pub unsafe fn run_hosted_timers<const N: usize>(
    owner: &RawOwner,
    runtime: Pin<&stela_driver_base::platform::work::kernel_domain::HostedRuntime<N>>,
    batch: &stela_driver_base::platform::work::kernel_domain::HostedBatch,
) -> Result<Option<u64>, WorkError> {
    let delivery = timer::TimerDelivery::hosted(owner, runtime, batch)?;
    // SAFETY: constructor checked the live service batch and actual source
    // current, and the caller keeps that binding and its callback graph.
    Ok(unsafe { timer::run(&delivery) })
}

use super::{owner::RawOwner, task::TaskBinding};
use alloc::{boxed::Box, sync::Arc};
use core::{
    cell::UnsafeCell,
    ffi::{c_int, c_void},
    marker::PhantomPinned,
    pin::Pin,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU8, AtomicUsize, Ordering},
};
use stela_driver_base::platform::work::{
    Callback, Runtime, Ticket, WorkError,
    service::{Batch, Domain, RaisedWork},
};

/// `struct stela_linux_service_call`.
#[repr(C)]
struct Call {
    owner: *const RawOwner,
    context: *mut c_void,
    begin: unsafe extern "C" fn(*mut c_void) -> bool,
    next_irq: unsafe extern "C" fn(*mut c_void) -> c_int,
    complete_irq: unsafe extern "C" fn(*mut c_void),
    drain: unsafe extern "C" fn(*mut c_void),
    finish: unsafe extern "C" fn(*mut c_void),
    done: unsafe extern "C" fn(*mut c_void) -> bool,
}

unsafe extern "C" {
    fn stela_linux_service_loop(call: *mut c_void);
    fn stela_linux_srcu_owner_bound(owner: *const RawOwner) -> bool;
}

/// Interrupt lines an owner may give Linux.
pub const LINES: u32 = 256;

/// Interrupts whose original handlers the service ran, for host reports.
static SERVICED: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// How many raised interrupts the service has run Linux's handlers for.
pub fn serviced_irqs() -> u64 {
    SERVICED.load(Ordering::Relaxed)
}

const IRQ_OPEN: u8 = 0;
const IRQ_CLOSING: u8 = 1;
const IRQ_JOINED: u8 = 2;
struct IrqEpoch {
    line: u32,
    state: AtomicU8,
    pending: AtomicBool,
    active: AtomicUsize,
}
/// One original service notification epoch. Native callbacks retain this exact
/// endpoint instead of rediscovering the current epoch from a numeric line.
#[derive(Clone)]
pub struct IrqEvent {
    epoch: Arc<IrqEpoch>,
    raised: RaisedWork,
}
impl IrqEvent {
    pub fn raise(&self) -> Result<(), WorkError> {
        if self.epoch.state.load(Ordering::Acquire) != IRQ_OPEN {
            return Err(WorkError::Closed);
        }
        self.epoch.pending.store(true, Ordering::Release);
        if self.epoch.state.load(Ordering::Acquire) != IRQ_OPEN {
            return Err(WorkError::Closed);
        }
        self.raised.raise()
    }
    /// Prevent new old-epoch service entries. This alone proves no hardware,
    /// callback, thread or DMA retirement; those owners must join separately.
    pub fn begin_close(&self) {
        let _ = self.epoch.state.compare_exchange(
            IRQ_OPEN,
            IRQ_CLOSING,
            Ordering::AcqRel,
            Ordering::Acquire,
        );
    }
}

const DEFERRED_CAPACITY: usize = 32;
#[derive(Clone, Copy)]
struct Deferred {
    body: Callback,
    context: *mut c_void,
}
struct DeferredQueue {
    entries: [Option<Deferred>; DEFERRED_CAPACITY],
    head: usize,
    len: usize,
}

pub struct Environment<const N: usize> {
    owner: *const RawOwner,
    runtime: NonNull<Runtime<N>>,
    domain: Arc<Domain<N>>,
    raised: RaisedWork,
    lines: UnsafeCell<[Option<Arc<IrqEpoch>>; LINES as usize]>,
    active_irq: UnsafeCell<Option<Arc<IrqEpoch>>>,
    /// The batch the service task is delivering.
    batch: UnsafeCell<Option<Batch>>,
    stopping: AtomicBool,
    service_started: AtomicBool,
    deferred: UnsafeCell<DeferredQueue>,
    /// Exact accepted permanent service assignment; managed children are
    /// owned by the ONE original task broker, never a parallel Environment pool.
    service_ticket: UnsafeCell<Option<Ticket>>,
    /// Exact original Initial startup graph, retained before native publication.
    initial: UnsafeCell<Option<initial::Initial<N>>>,
    _pin: PhantomPinned,
}

impl<const N: usize> Environment<N> {
    /// Attach to `runtime`, whose domain's clock must be the owner's `now_ns`.
    ///
    /// # Safety
    /// On the runtime's CPU, in its dispatcher. `owner` is entered by the
    /// tasks here, and it and `runtime` outlive them; the result stays alive
    /// until [`Self::retire`]. Other producers explicitly requesting Linux-size
    /// General stacks must honor this environment's two-per-pool headroom;
    /// default ordinary work in the native mixed profile uses different stacks.
    pub unsafe fn new(
        owner: *const RawOwner,
        runtime: Pin<&Runtime<N>>,
        domain: Arc<Domain<N>>,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        // Original workqueue.c owns its workers/rescuers and actual failures.
        // No compatibility-pool headroom promises progress. Validate this
        // exact immutable Service stack BEFORE global environment publication.
        if !runtime.service_supports(super::task::stack_requirement()?)? {
            return Err(WorkError::InvalidStack);
        }
        unsafe extern "C" {
            fn stela_linux_kernel_environment(owner: *const RawOwner) -> i32;
        }
        // SAFETY: caller owns the original resident RawOwner; this only records
        // cold bootstrap and rejects a second canonical Environment before effects.
        if unsafe { stela_linux_kernel_environment(owner) } != 0 {
            return Err(WorkError::Unavailable);
        }
        runtime.attach_service_domain(domain.clone())?;
        Ok(Box::pin(Self {
            owner,
            runtime: NonNull::from(runtime.get_ref()),
            raised: domain.raised_work()?,
            domain,
            lines: UnsafeCell::new(core::array::from_fn(|_| None)),
            active_irq: UnsafeCell::new(None),
            batch: UnsafeCell::new(None),
            stopping: AtomicBool::new(false),
            service_started: AtomicBool::new(false),
            deferred: UnsafeCell::new(DeferredQueue {
                entries: [None; DEFERRED_CAPACITY],
                head: 0,
                len: 0,
            }),
            service_ticket: UnsafeCell::new(None),
            initial: UnsafeCell::new(None),
            _pin: PhantomPinned,
        }))
    }

    pub(crate) fn matches_runtime(self: Pin<&Self>, runtime: Pin<&Runtime<N>>) -> bool {
        core::ptr::eq(
            self.runtime.as_ptr(),
            core::ptr::from_ref(runtime.get_ref()),
        )
    }

    fn runtime(&self) -> Pin<&Runtime<N>> {
        // SAFETY: `new`'s caller keeps the pinned runtime.
        unsafe { Pin::new_unchecked(self.runtime.as_ref()) }
    }

    fn initial(&self) -> Option<&initial::Initial<N>> {
        // SAFETY: only the runtime's CPU can access this Environment. Start
        // writes the initially empty slot once, before any native publication.
        // Once installed, the Initial stays in the pinned slot until drop.
        unsafe { &*self.initial.get() }.as_ref()
    }

    /// One-shot actual Initial service start. The private producer already owns
    /// the real original binding, root/code/data lease and bootstrap callback.
    /// Unknown native publication retains the whole installed startup graph;
    /// neither a returned error nor abandonment permits another Initial.
    pub(crate) unsafe fn start(
        self: Pin<&Self>,
        startup: initial::Initial<N>,
    ) -> Result<(), WorkError> {
        self.preflight_initial()?;
        // SAFETY: same dispatcher CPU, no switch or foreign callback during
        // this mutation; store every actual owner BEFORE native publication.
        unsafe { *self.initial.get() = Some(startup) };
        let context = core::ptr::from_ref(self.get_ref()).cast_mut().cast();
        // SAFETY: canonical mount keeps self and its exact Initial graph even
        // on failure/unwind. No error here is interpreted as a no-pub receipt.
        let ticket = unsafe { self.runtime().start_service(service::<N>, context) }?;
        // SAFETY: the dispatcher owns the one-shot ticket slot, and the
        // service callback does not access it.
        unsafe { *self.service_ticket.get() = Some(ticket) };
        self.service_started.store(true, Ordering::Release);
        Ok(())
    }

    pub(crate) fn preflight_initial(&self) -> Result<(), WorkError> {
        if !self
            .runtime()
            .service_supports(super::task::stack_requirement()?)?
        {
            return Err(WorkError::InvalidStack);
        }
        if self.initial().is_some() || self.service_started.load(Ordering::Acquire) {
            return Err(WorkError::Busy);
        }
        Ok(())
    }

    pub(crate) fn thread_wake(&self) -> RaisedWork {
        self.raised.clone()
    }

    /// Exact retained Initial bootstrap failure, for the original CPU's
    /// dispatcher diagnostics only. It grants no retry or resource readiness.
    pub fn initial_failure(&self) -> Option<super::task::ResourceBootstrapError> {
        self.initial().and_then(initial::Initial::failure)
    }
    /// Observe this exact service assignment only. This says nothing about
    /// original managed children, WQ pools, source/RCU teardown or kernel join.
    /// A genuine permanent Initial returning is a retained contract failure.
    pub fn finished(&self) -> Result<bool, WorkError> {
        // SAFETY: only the runtime's CPU can access the ticket. The start
        // path installs it once, and service callbacks never modify it.
        let ticket = unsafe { *self.service_ticket.get() };
        let Some(ticket) = ticket else {
            return Ok(true);
        };
        if !self.runtime().queue().completed(ticket)? {
            return Ok(false);
        }
        if self.initial().is_some() {
            return Err(WorkError::corrupt());
        }
        Ok(true)
    }

    /// Queue a bounded nonblocking process callback on the retained service
    /// continuation. It consumes no General task or workqueue pool credit and
    /// never executes inline. Full leaves ownership with the caller for retry.
    ///
    /// # Safety
    /// Called on the runtime process CPU (dispatcher or task). Keep callback,
    /// context and reachable resources until delivery or environment retirement.
    /// The callback runs already entered in this environment's owner, in normal
    /// process context. It must return promptly without sleeping, switching,
    /// entering/exiting an owner, or escaping the callback scope. No allocation
    /// or borrowing of the queue spans callback execution. An error accepts none.
    pub unsafe fn defer_service(
        self: Pin<&Self>,
        body: Callback,
        context: *mut c_void,
    ) -> Result<(), WorkError> {
        // Checks the real process CPU before accessing affine queue metadata.
        self.runtime()
            .general_capacity(super::task::stack_requirement()?)?;
        if self.stopping.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        if !self.service_started.load(Ordering::Acquire) {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: same process CPU; no switch/callback while this borrow exists.
        let pending = unsafe { &mut *self.deferred.get() };
        if pending.len == DEFERRED_CAPACITY {
            return Err(WorkError::Full);
        }
        let tail = (pending.head + pending.len) % DEFERRED_CAPACITY;
        pending.entries[tail] = Some(Deferred { body, context });
        pending.len += 1;
        // RaisedWork performs bounded publication/notification, never inline
        // delivery or switching. On refusal the just-appended entry is ours.
        if let Err(error) = self.raised.raise() {
            pending.entries[tail] = None;
            pending.len -= 1;
            return Err(error);
        }
        Ok(())
    }

    /// Create one notification epoch for an original inactive IRQ action.
    /// # Safety
    /// Called on the original process CPU; the actual source/entry owner is
    /// exclusive. A prior epoch was joined, not merely masked or root-fenced.
    pub unsafe fn irq_epoch(&self, line: u32) -> Result<IrqEvent, WorkError> {
        self.runtime()
            .general_capacity(super::task::stack_requirement()?)?;
        if line >= LINES || self.stopping.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        // SAFETY: checked original process CPU, no yield/callback in this borrow.
        let slots = unsafe { &mut *self.lines.get() };
        if slots[line as usize]
            .as_ref()
            .is_some_and(|old| old.state.load(Ordering::Acquire) != IRQ_JOINED)
        {
            return Err(WorkError::Busy);
        }
        let epoch = Arc::new(IrqEpoch {
            line,
            state: AtomicU8::new(IRQ_OPEN),
            pending: AtomicBool::new(false),
            active: AtomicUsize::new(0),
        });
        slots[line as usize] = Some(epoch.clone());
        Ok(IrqEvent {
            epoch,
            raised: self.raised.clone(),
        })
    }
    /// Join/cancel old queued primary service only after real native root/CPU
    /// retirement and C action-thread joining. No entered permit spans a wait.
    /// # Safety
    /// This is the original source/event and its producers cannot resume;
    /// actual source mask/action-close and CPU/inflight/EOI join already completed.
    /// C primary callbacks cannot sleep; an active callback here would mean
    /// reentrant free_irq and is refused instead of pretending to wait itself.
    pub unsafe fn join_irq(&self, event: &IrqEvent) -> Result<(), WorkError> {
        self.runtime()
            .general_capacity(super::task::stack_requirement()?)?;
        // SAFETY: original process CPU, no switch while reading this table.
        let slots = unsafe { &*self.lines.get() };
        let epoch = slots
            .get(event.epoch.line as usize)
            .and_then(Option::as_ref)
            .ok_or(WorkError::InvalidTicket)?;
        if !Arc::ptr_eq(epoch, &event.epoch) {
            return Err(WorkError::InvalidTicket);
        }
        let state = epoch.state.load(Ordering::Acquire);
        if state != IRQ_CLOSING && state != IRQ_JOINED {
            return Err(WorkError::Pending);
        }
        if epoch.active.load(Ordering::Acquire) != 0 {
            return Err(WorkError::WouldDeadlock);
        }
        epoch.pending.store(false, Ordering::Release);
        epoch.state.store(IRQ_JOINED, Ordering::Release);
        Ok(())
    }
    /// Notify this SAME retained service of a genuine source timer queue
    /// mutation or softirq post. Bounded existing IRQ-safe signal only; no C/current lookup,
    /// process lock, timer polling or callback-presence readiness.
    pub fn timer_changed(&self) -> Result<(), WorkError> {
        self.raised.raise()
    }

    /// Ask the service task to finish.
    pub fn stop(&self) -> Result<(), WorkError> {
        // SAFETY: read-only identity lookup, valid even in the dispatcher; the
        // pinned owner remains retained by this Environment's lifetime contract.
        if unsafe { stela_linux_srcu_owner_bound(self.owner) } {
            return Err(WorkError::Busy);
        }
        // The genuine original Initial is a permanent mount, with live
        // original resource/task/RCU roots. No joined kernel teardown exists.
        if self.initial().is_some() {
            return Err(WorkError::Busy);
        }
        self.stopping.store(true, Ordering::Release);
        self.raised.raise()
    }

    /// Release what the tasks held.
    ///
    /// # Safety
    /// [`Self::stop`] was called and [`Self::finished`] answered true, with
    /// nothing left to raise a line. On the runtime's CPU, in its dispatcher.
    pub unsafe fn retire(self: Pin<&Self>) -> Result<(), WorkError> {
        // SAFETY: same retained-owner lookup; no full-kernel SRCU teardown exists.
        if unsafe { stela_linux_srcu_owner_bound(self.owner) } {
            return Err(WorkError::Busy);
        }
        // SAFETY: the caller proves the process CPU and completed service.
        if unsafe { (*self.deferred.get()).len } != 0 {
            return Err(WorkError::Pending);
        }
        if self.initial().is_some() {
            return Err(WorkError::Busy); // actual permanent original roots remain
        }
        // SAFETY: only a genuinely unstarted/closed descriptor can reach this
        // partial retirement; original managed task teardown is never inferred.
        unsafe { self.domain.retire() }?;
        Ok(())
    }
}

/// # Safety
/// `context` is the environment a task here was started with.
unsafe fn environment<'a, const N: usize>(context: *mut c_void) -> &'a Environment<N> {
    // SAFETY: the caller's contract; `new`'s caller keeps it through the task.
    unsafe { &*context.cast() }
}

// A failure on these paths means the work runtime's bookkeeping no longer
// matches its tasks. They run below C frames, so they stop rather than unwind.

unsafe extern "C" fn begin<const N: usize>(context: *mut c_void) -> bool {
    // SAFETY: the service call's context.
    let this = unsafe { environment::<N>(context) };
    // C already entered this exact canonical owner on the real Initial.
    // No queue/row/EnteredProcess loan spans original resource helpers, clone,
    // completion waits or their actual physical switches.
    let initial = this.initial().expect("retained original Initial");
    initial.bootstrap();
    initial.service();
    let batch = this
        .domain
        .begin(this.runtime())
        .unwrap_or_else(|error| timer::fatal(error));
    let due = batch.is_some();
    // SAFETY: only the service task touches the batch.
    unsafe { *this.batch.get() = batch };
    due
}

unsafe extern "C" fn next_irq<const N: usize>(context: *mut c_void) -> c_int {
    // SAFETY: the service call's context.
    let this = unsafe { environment::<N>(context) };
    // SAFETY: this one original service task owns the table cursor/active slot.
    let active = unsafe { &mut *this.active_irq.get() };
    assert!(
        active.is_none(),
        "previous IRQ service body did not complete"
    );
    // SAFETY: same service CPU; process table mutations cannot interleave here.
    for epoch in unsafe { &*this.lines.get() }.iter().flatten() {
        if epoch.state.load(Ordering::Acquire) != IRQ_OPEN
            || !epoch.pending.swap(false, Ordering::AcqRel)
        {
            continue;
        }
        epoch.active.fetch_add(1, Ordering::AcqRel);
        *active = Some(epoch.clone());
        return epoch.line as c_int;
    }
    -1
}

unsafe extern "C" fn complete_irq<const N: usize>(context: *mut c_void) {
    // SAFETY: this original service call remains entered through primary return.
    let this = unsafe { environment::<N>(context) };
    // SAFETY: only the same service task owns this active epoch reference.
    let epoch = unsafe { &mut *this.active_irq.get() }
        .take()
        .expect("active original IRQ service body");
    SERVICED.fetch_add(1, Ordering::Relaxed);
    assert_eq!(epoch.active.fetch_sub(1, Ordering::AcqRel), 1);
}

unsafe extern "C" fn drain<const N: usize>(context: *mut c_void) {
    // SAFETY: the service call retains this environment through the callback.
    let this = unsafe { environment::<N>(context) };
    // Bound one batch even if a callback posts more leaves. Requeues retain a
    // raised epoch for the next original service pass.
    for _ in 0..DEFERRED_CAPACITY {
        let next = {
            // SAFETY: only this process CPU mutates this affine queue; the
            // borrow ends before invoking any foreign callback.
            let pending = unsafe { &mut *this.deferred.get() };
            if pending.len == 0 {
                None
            } else {
                let head = pending.head;
                pending.head = (head + 1) % DEFERRED_CAPACITY;
                pending.len -= 1;
                pending.entries[head].take()
            }
        };
        let Some(next) = next else {
            break;
        };
        // SAFETY: defer_service retained the nonblocking leaf/code/context.
        // C owns the entered process callback guard, never an IRQ scope.
        unsafe { (next.body)(next.context) };
    }
}

unsafe extern "C" fn finish<const N: usize>(context: *mut c_void) {
    // SAFETY: original actual service call's retained Environment context.
    let this = unsafe { environment::<N>(context) };
    // SAFETY: only this service task accesses the batch slot. Take its value
    // before C expiry delivery so the cell borrow ends before the callback.
    let batch = unsafe { &mut *this.batch.get() }
        .take()
        .expect("actual batch");
    let batch = core::mem::ManuallyDrop::new(batch);
    let deadline = {
        let delivery =
            timer::TimerDelivery::new(this, &batch).unwrap_or_else(|error| timer::fatal(error));
        // SAFETY: actual same owned batch/current and bounded token loan;
        // C cannot escape, sleep, recursively pump or retain this stack pointer.
        unsafe { timer::run(&delivery) }
    };
    // SAFETY: genuine same batch's primary/expiry callbacks completed and
    // scoped loan ended; actual queue minima came from those original bodies.
    unsafe {
        this.domain.finish(
            this.runtime(),
            core::mem::ManuallyDrop::into_inner(batch),
            deadline,
        )
    }
    .unwrap_or_else(|error| timer::fatal(error));
}

unsafe extern "C" fn done<const N: usize>(context: *mut c_void) -> bool {
    // SAFETY: the service call's context.
    let this = unsafe { environment::<N>(context) };
    this.stopping.load(Ordering::Acquire)
}

unsafe extern "C" fn service<const N: usize>(context: *mut c_void) {
    // SAFETY: `start`'s context.
    let this = unsafe { environment::<N>(context) };
    let runtime = this.runtime();
    let initial = this.initial().expect("retained original Initial");
    let binding = initial.binding();
    binding
        .bind_current(runtime)
        .unwrap_or_else(|error| timer::fatal(error));
    this.domain
        .bind_current(runtime)
        .unwrap_or_else(|error| timer::fatal(error));
    let mut call = Call {
        owner: this.owner,
        context,
        begin: begin::<N>,
        next_irq: next_irq::<N>,
        complete_irq: complete_irq::<N>,
        drain: drain::<N>,
        finish: finish::<N>,
        done: done::<N>,
    };
    let call = (&raw mut call).cast();
    // SAFETY: the loop leaves the owner before it returns, and `call` outlives it.
    unsafe { binding.run_scoped(runtime, stela_linux_service_loop, call) }
        .unwrap_or_else(|error| timer::fatal(error));
    // This actual original Initial owns permanent kernel roots. There is no
    // genuine shutdown/reaper contract allowing its callback to complete.
    // Keep binding/domain/source/code mounted if the C loop ever returns.
    timer::fatal(WorkError::corrupt());
}
