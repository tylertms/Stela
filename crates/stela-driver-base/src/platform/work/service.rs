//! Clock/event boundary and reasons for a cooperative kernel service.
//!
//! This is timing and lifecycle machinery, not a Timer continuation or Linux
//! IRQ-work admission. The execution domain must bind its reserved task
//! and run the original service bodies before acknowledging a captured batch.
//! No clock is inferred from Cpu::ticks and no callback is invoked here.
use super::{Cpu, QUARANTINED, Runtime, WakeHandle, WorkError};
use alloc::sync::Arc;
use core::{
    cell::UnsafeCell,
    mem::ManuallyDrop,
    pin::Pin,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};

pub const IRQ_WORK_TICK_NS: u64 = 1_000_000;
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum IdleOutcome {
    Ready,
    Waited,
}
const CLOSED: u64 = 1 << 63;
pub(super) mod reason;
use reason::ReasonCore;
pub(super) mod deadline;
use deadline::{Capture, DeadlineCore};

/// Owned nanosecond clock over the exact CPU/root permission.
///
/// # Safety
/// Readings are monotonic nanoseconds on this CPU, including during its
/// idle/event wait. `now_ns` is bounded, nonblocking and invokes no callback or
/// switch. The CPU, clock source and code remain valid through final release.
pub unsafe trait Clock: Send + Sync {
    fn cpu(&self) -> &Arc<dyn Cpu>;
    fn now_ns(&self) -> Result<u64, WorkError>;
}

/// Event/idle provider using that same clock and physical CPU.
///
/// # Safety
/// `notify` publishes a real event without allocation, blocking, switching or
/// inline service delivery, including from a checked IRQ/foreign producer.
/// `wait_until_ns` atomically checks/consumes those events before sleeping, so a
/// notify racing its entry cannot be lost. It uses the exact clock's absolute
/// nanosecond deadline (None means event-only), may return spuriously, preserves
/// incoming native flags and reports provider failures. It never enables IRQs
/// to manufacture progress. Keep all native clock/idle/event roots.
pub unsafe trait Idle: Send + Sync {
    fn clock(&self) -> &Arc<dyn Clock>;
    fn notify(&self) -> Result<(), WorkError>;
    fn wait_until_ns(&self, deadline: Option<u64>) -> Result<(), WorkError>;
}

struct Signal {
    reason: ReasonCore,
    idle: Arc<dyn Idle>,
}

/// Immutable generic reason publication; no Runtime, C task or owner pointer.
/// Closed handles remain safe and cannot reopen a later service generation.
pub struct RaisedWork {
    signal: ManuallyDrop<Arc<Signal>>,
}
impl Clone for RaisedWork {
    fn clone(&self) -> Self {
        Self {
            signal: self.signal.clone(),
        }
    }
}
impl RaisedWork {
    pub fn raise(&self) -> Result<(), WorkError> {
        self.signal.reason.publish()?;
        // A failure keeps the published reason; it never invents delivery.
        self.signal.idle.notify()
    }
}
impl Drop for RaisedWork {
    fn drop(&mut self) {
        let cpu = self.signal.idle.clock().cpu();
        if cpu.check_cpu().is_ok() && !cpu.in_interrupt() {
            // SAFETY: checked process release; a stale handle owns only these
            // generic provider roots, never retired C metadata.
            unsafe { ManuallyDrop::drop(&mut self.signal) };
        } else {
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// One nonforgeable capture. Acknowledgement retires only its observed epoch,
/// so a requeue during real callback delivery remains pending for another batch.
pub struct Batch {
    capture: Capture,
}
impl Batch {
    pub fn raised(&self) -> bool {
        self.capture.raised
    }
    pub fn tick(&self) -> bool {
        self.capture.tick
    }
    pub fn timer(&self) -> bool {
        self.capture.timer
    }
}

/// Same-CPU serialized timing state shared by timer and IRQ-work consumers.
/// Default Runtime construction does not create or enable this service.
pub struct Deadlines {
    signal: ManuallyDrop<Arc<Signal>>,
    core: DeadlineCore,
}
impl Deadlines {
    pub fn new(clock: Arc<dyn Clock>, idle: Arc<dyn Idle>) -> Result<Self, WorkError> {
        if !Arc::ptr_eq(&clock, idle.clock()) {
            return Err(WorkError::InvalidTicket);
        }
        clock.cpu().check_cpu()?;
        if clock.cpu().in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        let now = clock.now_ns()?;
        let core = DeadlineCore::new(now)?;
        Ok(Self {
            signal: ManuallyDrop::new(Arc::new(Signal {
                reason: ReasonCore::new(),
                idle,
            })),
            core,
        })
    }
    fn process(&self) -> Result<(), WorkError> {
        let cpu = self.signal.idle.clock().cpu();
        cpu.check_cpu()?;
        if cpu.in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        if self.core.retired() {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    fn now(&mut self) -> Result<u64, WorkError> {
        self.process()?;
        let now = self.signal.idle.clock().now_ns()?;
        self.core.observe(now)?;
        Ok(now)
    }
    pub fn raised_work(&self) -> Result<RaisedWork, WorkError> {
        self.process()?;
        if self.signal.reason.is_closed() {
            return Err(WorkError::Closed);
        }
        Ok(RaisedWork {
            signal: self.signal.clone(),
        })
    }
    /// Establish the real 1-ms periodic deadline before IRQ-work admission.
    pub fn start_irq_ticks(&mut self) -> Result<(), WorkError> {
        self.process()?;
        if !self.core.can_start_ticks() || self.signal.reason.is_closed() {
            return Err(WorkError::Busy);
        }
        let now = self.now()?;
        self.core.start_ticks(now)
    }
    /// The owner copies the original timerqueue's earliest deadline.
    pub fn set_timer(&mut self, deadline: Option<u64>) -> Result<(), WorkError> {
        self.process()?;
        if self.signal.reason.is_closed() {
            return Err(WorkError::Closed);
        }
        self.core.set_timer(deadline);
        Ok(())
    }
    pub fn next_deadline(&self) -> Result<Option<u64>, WorkError> {
        self.process()?;
        Ok(self.core.next_deadline())
    }
    /// No callback, switch, fake tick count or global jiffies effect.
    pub fn due(&mut self) -> Result<bool, WorkError> {
        let now = self.now()?;
        Ok(self.core.due(now, self.signal.reason.observed_epoch()))
    }
    pub fn begin(&mut self) -> Result<Option<Batch>, WorkError> {
        self.process()?;
        if self.core.running() {
            return Err(WorkError::Busy);
        }
        let now = self.now()?;
        // Capture BEFORE task/service inspection. A later raise is not
        // cleared by finish and remains eligible even if its notify was early.
        let epoch = self.signal.reason.observed_epoch();
        Ok(self
            .core
            .begin(now, epoch)?
            .map(|capture| Batch { capture }))
    }
    /// # Safety
    /// The kept service task delivered the captured original work
    /// and expiry batches without recursive pumping. `next_timer` is the real
    /// timerqueue minimum after that delivery. Callback/storage/provider leases
    /// remain held. This API cannot prove delivery from numerical reasons.
    pub unsafe fn finish(
        &mut self,
        batch: Batch,
        next_timer: Option<u64>,
    ) -> Result<(), WorkError> {
        self.finish_batch(batch, next_timer)
    }
    fn validate_batch(&self, batch: &Batch) -> Result<(), WorkError> {
        self.process()?;
        self.core.validate(&batch.capture)
    }
    fn finish_batch(&mut self, batch: Batch, next_timer: Option<u64>) -> Result<(), WorkError> {
        self.validate_batch(&batch)?;
        let now = self.now()?;
        self.core.finish(batch.capture, now, next_timer)
    }
    /// # Safety
    /// Call only on the real taskless domain dispatcher, with no entered Linux
    /// owner/critical state or mutable task borrow. The provider owns
    /// native idle/event permission; this method never installs that permission.
    pub unsafe fn idle(&mut self) -> Result<IdleOutcome, WorkError> {
        self.process()?;
        if self.signal.reason.is_closed() {
            return Err(WorkError::Closed);
        }
        if self.core.running() {
            return Err(WorkError::Busy);
        }
        // Ordinary arrival between dispatcher inspection and idle is progress,
        // not a failed provider. Keep it distinct from Busy/errors.
        if self.due()? {
            return Ok(IdleOutcome::Ready);
        }
        self.signal.idle.wait_until_ns(self.next_deadline()?)?;
        Ok(IdleOutcome::Waited)
    }
    pub fn stop_producers(&mut self) -> Result<(), WorkError> {
        self.process()?;
        self.signal.reason.close();
        Ok(())
    }
    /// # Safety
    /// Original producers, work/storage, waiter/RCU users and callbacks
    /// are stopped/drained. The reserved Timer task is joined before releasing
    /// its separate runtime/domain ownership. Numerical emptiness is no proof.
    pub unsafe fn retire(&mut self) -> Result<(), WorkError> {
        self.process()?;
        let epoch = self.signal.reason.load();
        self.core.retire(epoch)
    }
}
impl Drop for Deadlines {
    fn drop(&mut self) {
        let cpu = self.signal.idle.clock().cpu();
        if self.core.retired() && cpu.check_cpu().is_ok() && !cpu.in_interrupt() {
            // SAFETY: explicit quiescent retirement preceded process release.
            unsafe { ManuallyDrop::drop(&mut self.signal) };
        } else {
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

/// Keeps one clock/event domain and the reserved task's immutable
/// wake. Dispatcher inspection publishes that wake; it never executes a batch.
/// Linux task/storage admission and original callback delivery remain separate.
pub struct Domain<const N: usize> {
    clock: ManuallyDrop<Arc<dyn Clock>>,
    deadlines: ManuallyDrop<UnsafeCell<Deadlines>>,
    wake: ManuallyDrop<UnsafeCell<Option<WakeHandle<N>>>>,
    queue: AtomicU64,
    bound: AtomicBool,
    stopping: AtomicBool,
    retired: AtomicBool,
}
// SAFETY: every mutable access first validates the physical CPU and
// process context. Providers invoke no callbacks or switches. IRQ views carry
// only the separate atomic reason signal, never a Domain/Runtime/C task pointer.
unsafe impl<const N: usize> Send for Domain<N> {}
// SAFETY: same serialized CPU rule; no mutable borrow crosses delivery.
unsafe impl<const N: usize> Sync for Domain<N> {}
impl<const N: usize> Domain<N> {
    pub(super) fn scheduling_clock(&self) -> Arc<dyn Clock> {
        Arc::clone(&self.clock)
    }
    pub fn new(clock: Arc<dyn Clock>, idle: Arc<dyn Idle>) -> Result<Arc<Self>, WorkError> {
        let mut deadlines = Deadlines::new(clock.clone(), idle)?;
        deadlines.start_irq_ticks()?;
        Ok(Self::over(clock, deadlines))
    }
    /// A domain that wakes only for raised work and its timer, with no
    /// periodic tick: for an owner whose IRQ work raises its own service, or
    /// that has none. A CPU that runs guest code has no tick to spare.
    pub fn tickless(clock: Arc<dyn Clock>, idle: Arc<dyn Idle>) -> Result<Arc<Self>, WorkError> {
        let deadlines = Deadlines::new(clock.clone(), idle)?;
        Ok(Self::over(clock, deadlines))
    }
    fn over(clock: Arc<dyn Clock>, deadlines: Deadlines) -> Arc<Self> {
        Arc::new(Self {
            clock: ManuallyDrop::new(clock),
            deadlines: ManuallyDrop::new(UnsafeCell::new(deadlines)),
            wake: ManuallyDrop::new(UnsafeCell::new(None)),
            queue: AtomicU64::new(0),
            bound: AtomicBool::new(false),
            stopping: AtomicBool::new(false),
            retired: AtomicBool::new(false),
        })
    }
    fn process(&self) -> Result<(), WorkError> {
        self.clock.cpu().check_cpu()?;
        if self.clock.cpu().in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        if self.retired.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    pub(super) fn register(&self, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        self.process()?;
        if !Arc::ptr_eq(self.clock.cpu(), &runtime.queue.cpu) {
            return Err(WorkError::WrongCpu);
        }
        self.queue
            .compare_exchange(0, runtime.queue.id, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| WorkError::Busy)?;
        Ok(())
    }
    /// Bind after task attachment, on the reserved continuation.
    /// Original IRQ-work/RT callers may be admitted only after this succeeds.
    pub fn bind_current(self: &Arc<Self>, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        self.process()?;
        if self.queue.load(Ordering::Acquire) != runtime.queue.id
            || !runtime
                .domain()
                .is_some_and(|domain| Arc::ptr_eq(self, domain))
            || !runtime.service_current()?
        {
            return Err(WorkError::InvalidTicket);
        }
        if self.bound.load(Ordering::Acquire) {
            return Err(WorkError::Busy);
        }
        let wake = runtime.wake_handle()?;
        // SAFETY: checked service task; single publication, no callback.
        unsafe { *self.wake.get() = Some(wake) };
        self.bound.store(true, Ordering::Release);
        Ok(())
    }
    fn current(&self, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        self.process()?;
        if !self.bound.load(Ordering::Acquire)
            || self.queue.load(Ordering::Acquire) != runtime.queue.id
            || !runtime.service_current()?
        {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: acquired once-only initialization on the same checked CPU.
        let wake = unsafe { (&*self.wake.get()).as_ref() }.ok_or_else(|| WorkError::corrupt())?;
        if wake.renew_current()?.sequence() != wake.sequence() {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    pub fn raised_work(&self) -> Result<RaisedWork, WorkError> {
        self.process()?;
        // SAFETY: process serialization; no mutable deadline borrow is live.
        unsafe { &*self.deadlines.get() }.raised_work()
    }
    pub fn set_timer(&self, deadline: Option<u64>) -> Result<(), WorkError> {
        self.process()?;
        // SAFETY: same checked CPU; provider call cannot reenter or switch.
        unsafe { &mut *self.deadlines.get() }.set_timer(deadline)
    }
    /// When the service is next due with nothing raised.
    pub fn next_deadline(&self) -> Result<Option<u64>, WorkError> {
        self.process()?;
        // SAFETY: as in `set_timer`.
        unsafe { &*self.deadlines.get() }.next_deadline()
    }
    pub fn begin(&self, runtime: Pin<&Runtime<N>>) -> Result<Option<Batch>, WorkError> {
        self.current(runtime)?;
        // SAFETY: service; no borrow survives delivery/return.
        unsafe { &mut *self.deadlines.get() }.begin()
    }
    /// Validate an actual outstanding owned Batch from this SAME current
    /// reserved service/domain. This borrows only metadata, holds no queue/
    /// deadline lock across delivery, and grants no native IRQ/data access.
    pub fn validate_batch(
        &self,
        runtime: Pin<&Runtime<N>>,
        batch: &Batch,
    ) -> Result<(), WorkError> {
        self.current(runtime)?;
        // SAFETY: same serialized service; this shared immutable observation
        // ends before any foreign callback or source timer body is invoked.
        unsafe { &*self.deadlines.get() }.validate_batch(batch)
    }
    /// # Safety
    /// The original work/expiry consumers completed this captured batch on the
    /// service stack; next_timer is the original timerqueue min.
    pub unsafe fn finish(
        &self,
        runtime: Pin<&Runtime<N>>,
        batch: Batch,
        next_timer: Option<u64>,
    ) -> Result<(), WorkError> {
        self.current(runtime)?;
        // SAFETY: caller supplies delivery/storage proof; borrow ends now.
        unsafe { (&mut *self.deadlines.get()).finish(batch, next_timer) }
    }
    pub(super) fn inspect_and_wake(&self) -> Result<bool, WorkError> {
        self.process()?;
        if !self.bound.load(Ordering::Acquire) {
            return Ok(true);
        } // initial real service setup
        // SAFETY: acquired publication; immutable handle, no task/C pointer.
        let wake = unsafe { (&*self.wake.get()).as_ref() }.ok_or_else(|| WorkError::corrupt())?;
        if self.stopping.load(Ordering::Acquire) {
            if wake.closed()? {
                return Ok(false);
            }
            wake.wake()?;
            return Ok(true);
        }
        // SAFETY: pure reason/clock inspection, no C callback/inline switch.
        if unsafe { &mut *self.deadlines.get() }.due()? {
            wake.wake()?;
            return Ok(true);
        }
        Ok(false)
    }
    pub(super) fn idle(&self) -> Result<IdleOutcome, WorkError> {
        self.process()?;
        if !self.bound.load(Ordering::Acquire) {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: Runtime calls only on its taskless dispatcher.
        unsafe { (&mut *self.deadlines.get()).idle() }
    }
    pub fn stop_producers(&self) -> Result<(), WorkError> {
        self.process()?;
        // SAFETY: serialized process owner closes atomic producers first.
        unsafe { &mut *self.deadlines.get() }.stop_producers()?;
        self.stopping.store(true, Ordering::Release);
        if self.bound.load(Ordering::Acquire) {
            // SAFETY: acquired initialization; no final owner release or C read.
            let wake =
                unsafe { (&*self.wake.get()).as_ref() }.ok_or_else(|| WorkError::corrupt())?;
            if !wake.closed()? {
                wake.wake()?;
            }
        }
        Ok(())
    }
    pub fn stopping(&self) -> bool {
        self.stopping.load(Ordering::Acquire)
    }
    pub(super) fn retired(&self) -> bool {
        self.retired.load(Ordering::Acquire)
    }
    /// # Safety
    /// Original producers/storage/waiter/RCU users are truly drained. The
    /// reserved job completed and its separate task/binding lifetime is joined.
    pub unsafe fn retire(&self) -> Result<(), WorkError> {
        self.process()?;
        if !self.bound.load(Ordering::Acquire) {
            return Err(WorkError::Busy);
        }
        // SAFETY: immutable job handle; closed check invokes no provider.
        let wake = unsafe { (&*self.wake.get()).as_ref() }.ok_or_else(|| WorkError::corrupt())?;
        if !wake.closed()? {
            return Err(WorkError::Busy);
        }
        // SAFETY: caller supplies original storage/drain proof and join.
        unsafe { (&mut *self.deadlines.get()).retire()? };
        self.retired.store(true, Ordering::Release);
        // Break the explicit domain -> wake -> Queue/provider ownership cycle
        // only after job closure and successful original-state drain.
        // SAFETY: checked process retirement; no callback/borrow is live.
        let wake = unsafe { (&mut *self.wake.get()).take() };
        drop(wake);
        Ok(())
    }
}
impl<const N: usize> Drop for Domain<N> {
    fn drop(&mut self) {
        if self.retired.load(Ordering::Acquire)
            && self.clock.cpu().check_cpu().is_ok()
            && !self.clock.cpu().in_interrupt()
        {
            // SAFETY: explicit join/drain and checked process release.
            unsafe {
                ManuallyDrop::drop(&mut self.wake);
                ManuallyDrop::drop(&mut self.deadlines);
                ManuallyDrop::drop(&mut self.clock);
            }
        } else {
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(test)]
mod tests {
    extern crate std;
    use super::*;
    use core::sync::atomic::{AtomicBool, AtomicUsize};
    use std::{
        thread::{self, Thread, ThreadId},
        time::{Duration, Instant},
    };

    struct ModelCpu(ThreadId);
    // SAFETY: hosted same-thread model only.
    unsafe impl Cpu for ModelCpu {
        fn check_cpu(&self) -> Result<(), WorkError> {
            if thread::current().id() == self.0 {
                Ok(())
            } else {
                Err(WorkError::WrongCpu)
            }
        }
        fn in_interrupt(&self) -> bool {
            false
        }
        fn save_disable(&self) -> Result<u64, WorkError> {
            self.check_cpu()?;
            Ok(0x202)
        }
        unsafe fn restore(&self, flags: u64) {
            assert_eq!(flags, 0x202);
        }
        fn ticks(&self) -> u64 {
            panic!("service nanoseconds cannot use Cpu::ticks")
        }
        fn fatal(&self, error: WorkError) -> ! {
            panic!("{error:?}")
        }
    }
    struct ManualClock {
        cpu: Arc<dyn Cpu>,
        now: AtomicU64,
    }
    // SAFETY: explicit arithmetic input model; reads are bounded, same-thread,
    // with no callback or claim of a native clockevent producer.
    unsafe impl Clock for ManualClock {
        fn cpu(&self) -> &Arc<dyn Cpu> {
            &self.cpu
        }
        fn now_ns(&self) -> Result<u64, WorkError> {
            self.cpu.check_cpu()?;
            Ok(self.now.load(Ordering::Acquire))
        }
    }
    struct RealClock {
        cpu: Arc<dyn Cpu>,
        start: Instant,
    }
    // SAFETY: hosted monotonic elapsed time; provider/code kept.
    unsafe impl Clock for RealClock {
        fn cpu(&self) -> &Arc<dyn Cpu> {
            &self.cpu
        }
        fn now_ns(&self) -> Result<u64, WorkError> {
            self.cpu.check_cpu()?;
            self.start
                .elapsed()
                .as_nanos()
                .try_into()
                .map_err(|_| WorkError::Exhausted)
        }
    }
    struct Events {
        clock: Arc<dyn Clock>,
        pending: AtomicBool,
        owner: Thread,
        waits: AtomicUsize,
    }
    impl Events {
        fn new(clock: Arc<dyn Clock>) -> Arc<Self> {
            Arc::new(Self {
                clock,
                pending: AtomicBool::new(false),
                owner: thread::current(),
                waits: AtomicUsize::new(0),
            })
        }
    }
    // SAFETY: hosted park/unpark token, preserving an unpark
    // before park without a notifier mutex. This model admits no native ISR;
    // notification invokes no C callback or inline switch.
    unsafe impl Idle for Events {
        fn clock(&self) -> &Arc<dyn Clock> {
            &self.clock
        }
        fn notify(&self) -> Result<(), WorkError> {
            self.pending.store(true, Ordering::Release);
            self.owner.unpark();
            Ok(())
        }
        fn wait_until_ns(&self, deadline: Option<u64>) -> Result<(), WorkError> {
            self.clock.cpu().check_cpu()?;
            self.waits.fetch_add(1, Ordering::Relaxed);
            while !self.pending.swap(false, Ordering::AcqRel) {
                if let Some(deadline) = deadline {
                    let now = self.clock.now_ns()?;
                    if deadline <= now {
                        break;
                    }
                    thread::park_timeout(Duration::from_nanos(deadline - now));
                } else {
                    thread::park();
                }
            }
            Ok(())
        }
    }
    fn manual() -> (Arc<ManualClock>, Arc<Events>, Deadlines) {
        let cpu: Arc<dyn Cpu> = Arc::new(ModelCpu(thread::current().id()));
        let clock = Arc::new(ManualClock {
            cpu,
            now: AtomicU64::new(0),
        });
        let events = Events::new(clock.clone());
        let deadlines = Deadlines::new(clock.clone(), events.clone()).unwrap();
        (clock, events, deadlines)
    }
    fn retire(deadlines: &mut Deadlines) {
        deadlines.stop_producers().unwrap();
        // SAFETY: component fixture admits no original C storage, Timer job or
        // external producer. Captured timing state is finished and closed.
        unsafe { deadlines.retire() }.unwrap();
    }
    #[test]
    fn raised_requeue_survives_capture_and_late_tick_keeps_future_phase() {
        let (clock, _, mut deadlines) = manual();
        deadlines.start_irq_ticks().unwrap();
        let raised = deadlines.raised_work().unwrap();
        raised.raise().unwrap();
        let first = deadlines.begin().unwrap().unwrap();
        assert!(first.raised() && !first.tick() && !first.timer());
        raised.raise().unwrap();
        // Test the arithmetic component, not an invented C delivery callback.
        deadlines.finish_batch(first, None).unwrap();
        assert!(deadlines.due().unwrap());
        let requeued = deadlines.begin().unwrap().unwrap();
        deadlines.finish_batch(requeued, None).unwrap();
        assert!(!deadlines.due().unwrap());
        clock.now.store(3_500_000, Ordering::Release);
        let late = deadlines.begin().unwrap().unwrap();
        assert!(late.tick() && !late.raised());
        deadlines.finish_batch(late, None).unwrap();
        assert_eq!(deadlines.next_deadline(), Ok(Some(4_000_000)));
        assert!(!deadlines.due().unwrap());
        retire(&mut deadlines);
        assert_eq!(raised.raise(), Err(WorkError::Closed));
    }
    #[test]
    fn pending_work_refuses_idle_and_retirement() {
        let (_, events, mut deadlines) = manual();
        deadlines.start_irq_ticks().unwrap();
        deadlines.set_timer(Some(17)).unwrap();
        assert_eq!(deadlines.next_deadline(), Ok(Some(17)));
        let raised = deadlines.raised_work().unwrap();
        raised.raise().unwrap();
        // SAFETY: modeled dispatcher, no task or foreign mutable borrow.
        assert_eq!(unsafe { deadlines.idle() }, Ok(IdleOutcome::Ready));
        assert_eq!(events.waits.load(Ordering::Relaxed), 0);
        deadlines.stop_producers().unwrap();
        // SAFETY: no original service; incomplete numerical state.
        assert_eq!(unsafe { deadlines.retire() }, Err(WorkError::Busy));
        let batch = deadlines.begin().unwrap().unwrap();
        deadlines.finish_batch(batch, None).unwrap();
        retire(&mut deadlines);
    }
    #[test]
    fn real_elapsed_idle_deadline_requires_no_sleeping_driver_or_fake_tick() {
        let cpu: Arc<dyn Cpu> = Arc::new(ModelCpu(thread::current().id()));
        let clock: Arc<dyn Clock> = Arc::new(RealClock {
            cpu,
            start: Instant::now(),
        });
        let events = Events::new(clock.clone());
        let mut deadlines = Deadlines::new(clock, events.clone()).unwrap();
        deadlines.start_irq_ticks().unwrap();
        // SAFETY: modeled dispatcher owns this timing component and no C task.
        unsafe { deadlines.idle() }.unwrap();
        assert_eq!(events.waits.load(Ordering::Relaxed), 1);
        assert!(deadlines.due().unwrap());
        let batch = deadlines.begin().unwrap().unwrap();
        assert!(batch.tick());
        deadlines.finish_batch(batch, None).unwrap();
        retire(&mut deadlines);
    }
    #[test]
    fn deadline_or_raise_between_inspection_and_idle_requests_redispatch() {
        let (clock, events, mut deadlines) = manual();
        deadlines.start_irq_ticks().unwrap();
        assert!(!deadlines.due().unwrap());
        clock.now.store(IRQ_WORK_TICK_NS, Ordering::Release);
        // SAFETY: modeled taskless dispatcher; the exact pre-idle arrival is
        // deterministic and must not call the idle provider or report failure.
        assert_eq!(unsafe { deadlines.idle() }, Ok(IdleOutcome::Ready));
        let batch = deadlines.begin().unwrap().unwrap();
        deadlines.finish_batch(batch, None).unwrap();
        assert!(!deadlines.due().unwrap());
        let raised = deadlines.raised_work().unwrap();
        raised.raise().unwrap();
        // SAFETY: same controlled inspection-to-idle gap for a real publication.
        assert_eq!(unsafe { deadlines.idle() }, Ok(IdleOutcome::Ready));
        assert_eq!(events.waits.load(Ordering::Relaxed), 0);
        let batch = deadlines.begin().unwrap().unwrap();
        deadlines.finish_batch(batch, None).unwrap();
        retire(&mut deadlines);
    }
}
