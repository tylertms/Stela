//! One physical root's fixed-clock producer, multiplexed through the engine's
//! existing local APIC timer/vector. No Linux callback enters from this ISR.
//!
//! General time is the invariant TSC at the boot's calibrated rate, else an
//! exclusively owned HPET64 source read with coherent32 sampling. No source
//! grants NTP, suspend or cross-CPU synchronization.
use crate::{
    apic::{self, Apic},
    engine::{host, interrupts},
};
use alloc::{boxed::Box, sync::Arc};
use core::{
    arch::asm,
    cell::Cell,
    marker::PhantomData,
    mem::ManuallyDrop,
    ptr,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU64, Ordering},
};
use stela_drivers::platform::work::{
    Cpu, EnteredProcess, WorkError,
    service::{Clock, Idle},
};
mod counter;
pub(crate) mod hpet;
mod pm;
mod source;
pub(crate) mod tsc;
pub(crate) use counter::Counter as HpetCounter;
pub(crate) use source::Prepared;
const IF: u64 = 1 << 9;
const TICK_NS: u64 = 1_000_000;
const NONE: u64 = u64::MAX;

// Exactly one registered timer consumer. Publication and removal occur on its
// owning processor with IF off; the hardware ISR never takes an Arc reference.
static ACTIVE: AtomicPtr<State> = AtomicPtr::new(ptr::null_mut());
// Other native host processors use the same engine handler. They must reject
// BEFORE loading/dereferencing the owning CPU's registration pointer: checking
// State.cpu afterward would race real BSP removal and permit a use-after-free.
// The original HostOwner owns a distinct permanently pinned GS record on every
// physical CPU. Validate that live architectural identity BEFORE ACTIVE, on
// every timer arm and interrupt: unlike CPUID it never exits to a hypervisor,
// and unlike a legacy APIC ID no remote CPU can collide with it.
static ACTIVE_GS: AtomicU64 = AtomicU64::new(0);
/// Counts the native clock's takeovers and hand-backs of the local APIC
/// timer, after which its LVT may not be what the engine's host last wrote
/// (a hand-back with no guest deadline leaves it masked): the host keeps
/// its LVT write only while this is unchanged (`bare_host::arm_timer`).
pub(crate) static TIMER_HANDOVERS: AtomicU64 = AtomicU64::new(0);
/// Counts clock samples (`RootCpu::sample_ticks`), for the `GPU LINUX`
/// report's `clocks=`.
pub(crate) static SAMPLES: AtomicU64 = AtomicU64::new(0);

fn processor_id() -> u32 {
    let highest = core::arch::x86_64::__cpuid(0).eax;
    for leaf in [0x1f, 0xb] {
        if highest >= leaf {
            let topology = core::arch::x86_64::__cpuid_count(leaf, 0);
            if topology.ebx != 0 {
                return topology.edx;
            }
        }
    }
    core::arch::x86_64::__cpuid(1).ebx >> 24
}

fn gs_base() -> u64 {
    // SAFETY: read-only architectural CPL0 GS context, not device MMIO.
    unsafe { crate::cpu::msr::GS_BASE.read() }
}

fn registration_cpu() -> bool {
    let expected_gs = ACTIVE_GS.load(Ordering::Acquire);
    expected_gs != 0 && gs_base() == expected_gs
}

fn flags_disable() -> u64 {
    let flags;
    // SAFETY: boot-owned CPL0 entry; memory clobber encloses the critical scope.
    unsafe {
        asm!("pushfq", "pop {}", "cli", out(reg) flags);
    }
    flags
}
unsafe fn restore(flags: u64) {
    // SAFETY: enclosing IRQ-save on this same physical processor.
    unsafe {
        asm!("push {}", "popfq", in(reg) flags);
    }
}
fn root() -> u64 {
    let root;
    // SAFETY: read-only privileged root observation.
    unsafe {
        asm!("mov {}, cr3", out(reg) root, options(nomem, nostack, preserves_flags));
    }
    root
}

/// Immutable CPU view. The !Send HostOwner remains on its processor;
/// its root lease refuses owner destruction while any view still exists.
pub(crate) struct RootCpu {
    identity: Arc<host::RootIdentity>,
    base: u64,
    registers: u64,
    gs: u64,
    deadline_control: tsc::DeadlineControl,
    source: Prepared,
    previous: AtomicU64,
    elapsed: AtomicU64,
    closed: AtomicBool,
    health: source::Health,
}
impl RootCpu {
    fn controller(&self) -> Apic {
        Apic::new(self.base, self.registers)
    }
    fn affinity(&self) -> Result<(), WorkError> {
        if self.closed.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        // The kept immutable Owner identity checks the CPU/GS/
        // descriptors and original root catalog before device access.
        self.identity.check_irq_cpu()
    }
    /// Caller holds local IRQ exclusion. A source fault does not
    /// revoke the time-free CPU identity needed to drain/release resources.
    fn sample_ticks(&self) -> Result<u64, WorkError> {
        SAMPLES.fetch_add(1, Ordering::Relaxed);
        self.health.check()?;
        let result = (|| {
            // No CPUID here: it is slow on every CPU and exits under a
            // hypervisor. The caller's `affinity` already proved this is the
            // owning processor (its unique GS record and descriptors), whose
            // vendor and family cannot change; ARAT matters only to the
            // timer, and `rearm` checks it before every arm.
            // SAFETY: original owner and CPU remain held. Source checks
            // real control/power eligibility around the coherent read.
            let mut raw = unsafe { self.source.read()? };
            let previous = self.previous.load(Ordering::Relaxed);
            if raw == previous {
                raw = self.source.progress_after(previous)?;
            }
            let elapsed = self
                .elapsed
                .load(Ordering::Relaxed)
                .checked_add(self.source.delta(previous, raw)?)
                .ok_or(WorkError::Exhausted)?;
            self.previous.store(raw, Ordering::Relaxed);
            self.elapsed.store(elapsed, Ordering::Release);
            Ok(elapsed)
        })();
        result.map_err(|error| self.health.fail(error))
    }
    fn sample(&self) -> Result<u64, WorkError> {
        self.source
            .ns(self.sample_ticks()?)
            .map_err(|error| self.health.fail(error))
    }
    fn now(&self) -> Result<u64, WorkError> {
        self.check_cpu()?;
        let flags = flags_disable();
        let result = self.sample();
        // SAFETY: enclosing save, no switch/callback or migration.
        unsafe {
            restore(flags);
        }
        result
    }
}
// SAFETY: immutable root/CPU witnesses are checked before entry; all mutable
// clock publication is atomic and serialized by same-CPU IRQ exclusion.
// The lease prevents HostOwner destruction, and foreign CPU entry is refused.
unsafe impl Cpu for RootCpu {
    fn check_cpu(&self) -> Result<(), WorkError> {
        self.affinity()?;
        self.identity.check_clock_root()
    }
    fn in_interrupt(&self) -> bool {
        self.affinity().is_err() || host::root_irq_active()
    }
    fn save_disable(&self) -> Result<u64, WorkError> {
        self.check_cpu()?;
        Ok(flags_disable())
    }
    unsafe fn restore(&self, flags: u64) {
        if self.check_cpu().is_err() {
            self.fatal(WorkError::WrongCpu);
        }
        // SAFETY: the Cpu contract requires this same CPU's enclosing save.
        unsafe {
            restore(flags);
        }
    }
    fn ticks(&self) -> u64 {
        self.now().unwrap_or_else(|error| self.fatal(error))
    }
    fn fatal(&self, error: WorkError) -> ! {
        crate::output::fatal(format_args!("native clock/root contract failed: {error:?}"))
    }
}

/// Continuous-source identity kept with the original physical CPU owner.
/// Cloning keeps ownership; it neither migrates sampling nor rebases time.
#[derive(Clone)]
pub(crate) struct CounterEpoch {
    cpu: Arc<RootCpu>,
}
/// A sealed pairing of the existing installed CPU, clock and idle producer.
/// Its private fields cannot pair an unrelated Cpu/Idle with a counter epoch.
#[derive(Clone)]
pub(crate) struct ClockServices {
    state: Arc<State>,
    clock: Arc<dyn Clock>,
    idle: Arc<dyn Idle>,
}
impl ClockServices {
    pub(crate) fn check_identity(
        &self,
        identity: &Arc<host::RootIdentity>,
    ) -> Result<(), WorkError> {
        self.state.process()?;
        if !Arc::ptr_eq(identity, &self.state.cpu.identity) {
            return Err(WorkError::WrongCpu);
        }
        self.state.cpu.health.check()?;
        self.state
            .cpu
            .source
            .continuous_period()
            .map_err(|error| self.state.cpu.health.fail(error))?;
        Ok(())
    }
    pub(crate) fn cpu(&self) -> Arc<dyn Cpu> {
        self.state.cpu_view.clone()
    }
    pub(crate) fn clock(&self) -> Arc<dyn Clock> {
        self.clock.clone()
    }
    pub(crate) fn idle(&self) -> Arc<dyn Idle> {
        self.idle.clone()
    }
    /// Publish a software wake from the retained service. The service sends
    /// its own doorbell after this store when another CPU may be waiting.
    pub(crate) fn notify_dispatch(&self) {
        self.state.pending.store(true, Ordering::Release);
    }

    pub(crate) fn epoch(&self) -> Result<CounterEpoch, WorkError> {
        self.check_identity(&self.state.cpu.identity)?;
        Ok(CounterEpoch {
            cpu: self.state.cpu.clone(),
        })
    }
}
impl CounterEpoch {
    /// Nominal monotonic nanoseconds for transport deadlines.
    pub(crate) fn now_ns(&self) -> Result<u64, WorkError> {
        let ticks = self.sample_ticks()?;
        self.cpu
            .source
            .ns(ticks)
            .map_err(|error| self.cpu.health.fail(error))
    }
    fn sample_ticks(&self) -> Result<u64, WorkError> {
        self.cpu.health.check()?;
        self.cpu
            .source
            .continuous_period()
            .map_err(|error| self.cpu.health.fail(error))?;
        let flags = flags_disable();
        // One CPU and root check with IF clear covers the sample: from it to
        // the restore nothing runs on this processor but the counter read,
        // no task switch, callback or root change, and an NMI returns to the
        // same root. Checking again in the restore and after it (two more
        // page walks per Linux clock read) proved nothing more.
        let result = self.cpu.check_cpu().and_then(|()| self.cpu.sample_ticks());
        // Restore depends only on the real time-free CPU identity. A failed
        // source must not prevent resources from being drained on that CPU.
        // SAFETY: the same CPU's enclosing IRQ save above; no switch/callback.
        unsafe {
            restore(flags);
        }
        let ticks = result?;
        self.cpu.health.check()?;
        Ok(ticks)
    }
}

/// A physical minimum on the same retained epoch as the scheduler clock.
pub(crate) struct MinimumDeadline {
    epoch: CounterEpoch,
    start: u64,
    target: u64,
    minimum_ns: u64,
    nominal_ns: i64,
}

impl CounterEpoch {
    fn minimum_target(&self, start: u64, minimum_ns: u64) -> Result<u64, WorkError> {
        let ticks = self.cpu.source.ticks_for_minimum_ns(minimum_ns)?;
        start.checked_add(ticks).ok_or(WorkError::Exhausted)
    }

    /// Atomic physical wait on this CPU's continuously qualified source.
    pub(crate) fn minimum_delay_us(&self, minimum_us: u64) -> Result<(), WorkError> {
        let minimum_ns = minimum_us.checked_mul(1000).ok_or(WorkError::Exhausted)?;
        let start = self.sample_ticks()?;
        let target = self.minimum_target(start, minimum_ns)?;
        let mut now = start;
        while now < target {
            core::hint::spin_loop();
            now = self.sample_ticks()?;
        }
        Ok(())
    }

    pub(crate) fn minimum_deadline(&self, minimum_ns: u64) -> Result<MinimumDeadline, WorkError> {
        let start = self.sample_ticks()?;
        let target = self.minimum_target(start, minimum_ns)?;
        // The nominal conversion floors. A subnanosecond HPET tick can
        // share that nanosecond with an earlier tick, so wake one ns later.
        let nominal_ns = self.cpu.source.ns(target)?;
        let nominal_ns = i64::try_from(nominal_ns)
            .ok()
            .and_then(|ns| ns.checked_add(1))
            .filter(|ns| *ns != i64::MAX)
            .ok_or(WorkError::Exhausted)?;
        Ok(MinimumDeadline {
            epoch: self.clone(),
            start,
            target,
            minimum_ns,
            nominal_ns,
        })
    }
}

// SAFETY: construction is private to the installed HPET epoch. Every call
// revalidates that retained source and CPU and releases its sampling scope
// before returning. The hint uses the nominal conversion of that same epoch.
unsafe impl stela_linux::owner::MinimumDeadline for MinimumDeadline {
    fn ready(&self) -> Result<bool, WorkError> {
        Ok(self.epoch.sample_ticks()? >= self.target)
    }

    fn wake_ns(&self) -> Result<i64, WorkError> {
        self.epoch.sample_ticks()?;
        Ok(self.nominal_ns)
    }

    fn remaining_quanta_ceil(&self, quantum_us: u64) -> Result<u64, WorkError> {
        let quantum_ns = quantum_us
            .checked_mul(1000)
            .filter(|ns| *ns != 0)
            .ok_or(WorkError::Exhausted)?;
        let now = self.epoch.sample_ticks()?;
        if now >= self.target {
            return Ok(0);
        }
        let elapsed = now
            .checked_sub(self.start)
            .ok_or_else(|| WorkError::corrupt())?;
        let lower_ns = self.epoch.cpu.source.elapsed_bounds(elapsed)?.lower_ns;
        let remaining = self
            .minimum_ns
            .checked_sub(lower_ns)
            .filter(|ns| *ns != 0)
            .ok_or_else(|| WorkError::corrupt())?;
        // A fresh delay for this remainder reapplies the HPET edge bound.
        // The elapsed lower bound never credits more ticks than actually ran.
        Ok(remaining.div_ceil(quantum_ns))
    }
}

impl stela_drivers::bus::pci::power::Interval for MinimumDeadline {
    fn elapsed(&self) -> Result<bool, stela_drivers::bus::pci::power::Error> {
        use stela_drivers::bus::pci::power::Error;
        if self.epoch.cpu.in_interrupt() {
            return Err(Error::Clock);
        }
        stela_linux::owner::MinimumDeadline::ready(self).map_err(|_| Error::Clock)
    }
}

// SAFETY: the epoch retains and checks the actual HPET/CPU/root on every
// sample, derives the target from the outward lower bound, and does not wait
// in PCI interrupt context.
unsafe impl stela_drivers::bus::pci::power::Timer for CounterEpoch {
    fn check(&self) -> Result<(), stela_drivers::bus::pci::power::Error> {
        use stela_drivers::bus::pci::power::Error;
        if self.cpu.in_interrupt() {
            return Err(Error::Clock);
        }
        let start = self.sample_ticks().map_err(|_| Error::Clock)?;
        // The longest PCI power readiness interval is 60 seconds.
        self.minimum_target(start, 60_000_000_000)
            .map(|_| ())
            .map_err(|_| Error::Clock)
    }

    fn wait_us(&self, microseconds: u64) -> Result<(), stela_drivers::bus::pci::power::Error> {
        use stela_drivers::bus::pci::power::Error;
        if self.cpu.in_interrupt() {
            return Err(Error::Clock);
        }
        self.minimum_delay_us(microseconds)
            .map_err(|_| Error::Clock)
    }

    fn begin_interval(
        &self,
        microseconds: u64,
    ) -> Result<
        Option<Box<dyn stela_drivers::bus::pci::power::Interval>>,
        stela_drivers::bus::pci::power::Error,
    > {
        use stela_drivers::bus::pci::power::Error;
        if self.cpu.in_interrupt() {
            return Err(Error::Clock);
        }
        let ns = microseconds.checked_mul(1_000).ok_or(Error::Clock)?;
        let interval = self.minimum_deadline(ns).map_err(|_| Error::Clock)?;
        Ok(Some(Box::new(interval)))
    }
}

struct State {
    cpu: Arc<RootCpu>,
    apic_hz: u64,
    cpu_view: Arc<dyn Cpu>,
    pending: AtomicBool,
    executing: AtomicBool,
    execution_generation: AtomicU64,
    guest: AtomicU64,
    idle: AtomicU64,
    heartbeat: AtomicU64,
}
// SAFETY: pointer fields never leave the owning CPU, every effect checks
// affinity first, and process mutation excludes the same real ISR with CLI.
unsafe impl Send for State {}
// SAFETY: same CPU and IRQ exclusion; no callback accesses mutable State.
unsafe impl Sync for State {}
impl State {
    fn process(&self) -> Result<(), WorkError> {
        self.cpu.check_cpu()?;
        if self.cpu.in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        if !ptr::eq(ACTIVE.load(Ordering::Acquire), self) {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    /// Wait on the installed mux under the caller's local interrupt save.
    /// The caller owns the explicit root/IRQ execution boundary for HLT.
    unsafe fn wait_until(&self, deadline: Option<u64>) -> Result<(), WorkError> {
        let now = self.cpu.sample()?;
        if self.pending.swap(false, Ordering::AcqRel)
            || deadline.is_some_and(|deadline| deadline <= now)
        {
            return Ok(());
        }
        self.idle.store(deadline.unwrap_or(NONE), Ordering::Relaxed);
        self.rearm(now)?;
        let budget = || {
            let ticks = self.cpu.controller().read(apic::CURRENT);
            Some(u64::from(ticks) * 1_000_000 / self.apic_hz)
        };
        // SAFETY: caller supplies the live IDT/LAPIC and the explicit wait scope.
        unsafe { crate::native_idle::wait(budget) };
        self.idle.store(NONE, Ordering::Relaxed);
        self.pending.swap(false, Ordering::AcqRel);
        self.rearm(self.cpu.sample()?)
    }

    fn rearm(&self, now: u64) -> Result<(), WorkError> {
        self.cpu.health.check()?;
        self.cpu
            .deadline_control
            .check()
            .map_err(|error| self.cpu.health.fail(error))?;
        let next = self
            .heartbeat
            .load(Ordering::Relaxed)
            .min(self.guest.load(Ordering::Relaxed))
            .min(self.idle.load(Ordering::Relaxed));
        let delta = next.saturating_sub(now).max(1);
        let ticks = (u128::from(delta) * u128::from(self.apic_hz))
            .div_ceil(u128::from(pm::NS))
            .clamp(1, u128::from(u32::MAX)) as u32;
        let apic = self.cpu.controller();
        if apic.read(apic::DIVIDE) & 0b1011 != 0b1011 {
            return Err(self.cpu.health.fail(WorkError::corrupt()));
        }
        apic.write(apic::LVT_TIMER, u32::from(interrupts::TIMER_VECTOR));
        apic.write(apic::INITIAL, ticks);
        Ok(())
    }
}
struct NativeClock(Arc<State>);
// SAFETY: time comes from the hardware counter. Its coherent
// reads and elapsed accumulator are serialized by this physical CPU's IRQ save.
unsafe impl Clock for NativeClock {
    fn cpu(&self) -> &Arc<dyn Cpu> {
        &self.0.cpu_view
    }
    fn now_ns(&self) -> Result<u64, WorkError> {
        self.0.cpu.now()
    }
}
struct NativeIdle {
    state: Arc<State>,
    clock: Arc<dyn Clock>,
}
// SAFETY: all notifications are same physical CPU and kept GS/IDT roots;
// pending is published before sleeping's CLI/check/STI;HLT sequence. An incoming
// IF=0 refuses waiting, so the provider never creates permission for progress.
unsafe impl Idle for NativeIdle {
    fn clock(&self) -> &Arc<dyn Clock> {
        &self.clock
    }
    fn notify(&self) -> Result<(), WorkError> {
        self.state.cpu.affinity()?;
        self.state.pending.store(true, Ordering::Release);
        Ok(())
    }
    fn wait_until_ns(&self, deadline: Option<u64>) -> Result<(), WorkError> {
        self.state.process()?;
        let flags = flags_disable();
        let result = (|| {
            if flags & IF == 0 {
                return Err(WorkError::WouldDeadlock);
            }
            // SAFETY: incoming IF was set and process validation owns the
            // exact installed dispatcher CPU and root for this wait.
            unsafe { self.state.wait_until(deadline) }
        })();
        // SAFETY: exact flags from the same physical dispatcher entry.
        unsafe {
            restore(flags);
        }
        result
    }
}

/// Own the native registration until the service has stopped and joined.
/// Forgetting this guard pins the real root rather than freeing live ISR state.
pub(crate) struct Installed {
    state: ManuallyDrop<Arc<State>>,
    pub(crate) clock: ManuallyDrop<Arc<dyn Clock>>,
    pub(crate) idle: ManuallyDrop<Arc<dyn Idle>>,
    _local: PhantomData<*mut ()>,
}

/// Explicit main-owned permission for physical IRQ delivery during driver
/// execution. Generic Cpu, Runtime and Idle code cannot create this grant.
pub(crate) struct DriverExecution<'a> {
    installed: &'a Installed,
    flags: u64,
    root: u64,
    generation: u64,
    process_borrowed: Cell<bool>,
    _local: PhantomData<*mut ()>,
}
struct DriverProcess<'a, 'installed> {
    execution: &'a DriverExecution<'installed>,
    saved: u64,
}

// SAFETY: the borrow is subordinate to the actual DriverExecution and never
// outlives its interrupt-excluded segment. It validates its physical CPU,
// active generation and exact entry CR3 before each metadata operation.
unsafe impl EnteredProcess for DriverProcess<'_, '_> {
    fn check(&self) -> Result<(), WorkError> {
        self.execution.check()?;
        if !self.execution.process_borrowed.get() || self.execution.root != root() {
            return Err(WorkError::corrupt());
        }
        let flags: u64;
        // SAFETY: read-only flag observation on the admitted physical CPU.
        unsafe { asm!("pushfq", "pop {}", out(reg) flags, options(preserves_flags)) };
        if flags & IF != 0 {
            return Err(WorkError::Busy);
        }
        Ok(())
    }
}

impl Drop for DriverProcess<'_, '_> {
    fn drop(&mut self) {
        if let Err(error) = self.check() {
            self.execution.installed.state.cpu.fatal(error);
        }
        self.execution.process_borrowed.set(false);
        // SAFETY: the same admitted CPU/root retains this enclosing IRQ save.
        unsafe { restore(self.saved) };
    }
}

impl DriverExecution<'_> {
    pub(crate) fn check(&self) -> Result<(), WorkError> {
        self.installed.state.process()?;
        if !self.installed.state.executing.load(Ordering::Acquire)
            || self
                .installed
                .state
                .execution_generation
                .load(Ordering::Acquire)
                != self.generation
        {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    /// Lend one process metadata operation the currently entered root.
    /// # Safety
    /// The body must be bounded and synchronous. It cannot switch, park,
    /// restore interrupts, change roots or retain the borrowed permission.
    pub(crate) unsafe fn with_process<R>(
        &self,
        body: impl FnOnce(&dyn EnteredProcess) -> R,
    ) -> Result<R, WorkError> {
        self.check()?;
        if root() != self.root {
            return Err(WorkError::corrupt());
        }
        if self.process_borrowed.get() {
            return Err(WorkError::Busy);
        }
        let saved = flags_disable();
        self.process_borrowed.set(true);
        let process = DriverProcess {
            execution: self,
            saved,
        };
        process.check()?;
        Ok(body(&process))
    }
}
impl Drop for DriverExecution<'_> {
    fn drop(&mut self) {
        if self.check().is_err() {
            self.installed.state.cpu.fatal(WorkError::corrupt());
        }
        if self.process_borrowed.get() {
            self.installed.state.cpu.fatal(WorkError::Busy);
        }
        flags_disable();
        self.installed
            .state
            .executing
            .store(false, Ordering::Release);
        // SAFETY: same-CPU owner scope; no C frame/IRQ remains active.
        unsafe {
            restore(self.flags);
        }
    }
}
impl Installed {
    /// # Safety
    /// Post-EBS owning CPU, using the production resource owner and immutable
    /// HostOwner identity. HPET source owns its UC mapping/controller and the
    /// same identity. No stop/reset/counter-write/rate/decode/suspend
    /// discontinuity, migration or root replacement occurs. Existing all-root
    /// IDT/GS/xstate/clock code mappings remain held. The hardware timer is
    /// quiescent and its sole owner hands it to this mux. The ARAT-checked
    /// LAPIC timer's measured rate remains stable through every HLT. Every old
    /// absence/publication capability must independently close before RAM
    /// adoption; this clock never keeps or recreates one of those grants.
    pub(crate) unsafe fn attach_native(
        identity: Arc<host::RootIdentity>,
        source: Prepared,
    ) -> Result<Self, WorkError> {
        identity.check_dispatch()?;
        source.admit(&identity)?;
        source.continuous_period()?;
        // SAFETY: caller keeps the physical controller/root/deadline
        // owner; source admission below checks the same identity.
        unsafe { Self::attach_identity(identity, source) }
    }
    /// Source and local timer handoff for `attach_native`.
    unsafe fn attach_identity(
        identity: Arc<host::RootIdentity>,
        source: Prepared,
    ) -> Result<Self, WorkError> {
        identity.check_dispatch()?;
        source.admit(&identity)?;
        let deadline_control = tsc::DeadlineControl::capture()?;
        // ARAT is the local timer's invariant-rate/idle prerequisite;
        // the existing guest timer or successful periodic delivery supplies no
        // such proof. Refuse before CLI, calibration, ownership or publication.
        // Keep a source owner outside the IRQ-off closure so failed admission
        // cannot perform its final Mapping/root owner release before restore.
        let deferred_source = source.clone();
        let flags = flags_disable();
        let result = (|| {
            if !ACTIVE.load(Ordering::Acquire).is_null() {
                return Err(WorkError::Busy);
            }
            identity.check_dispatch()?;
            let base = apic::base_msr();
            let registers = crate::layout::physical(base & apic::BASE_ADDRESS);
            let apic = Apic::new(base, registers);
            if apic.id() == u32::MAX || processor_id() != apic.id() {
                return Err(WorkError::WrongCpu);
            }
            let timer_bit = 1u32 << (interrupts::TIMER_VECTOR % 32);
            let timer_group = u32::from(interrupts::TIMER_VECTOR / 32) * 16;
            if apic.read(apic::INITIAL) != 0
                || apic.read(apic::CURRENT) != 0
                || apic.read(0x200 + timer_group) & timer_bit != 0
                || apic.read(0x100 + timer_group) & timer_bit != 0
                || apic.read(apic::LVT_TIMER) & 255 != u32::from(interrupts::TIMER_VECTOR)
            {
                return Err(WorkError::Busy);
            }
            // SAFETY: descriptor/root ownership from owner.
            let observed = unsafe { interrupts::observe() };
            if host::root_irq_active() {
                return Err(WorkError::InterruptContext);
            }
            // SAFETY: exact canonical source/root ownership kept;
            // HPET validates the original CPU/root before coherent MMIO reads.
            let previous = unsafe { source.read()? };
            let cpu = Arc::new(RootCpu {
                identity,
                base,
                registers,
                gs: observed.gs,
                deadline_control,
                source,
                previous: AtomicU64::new(previous),
                elapsed: AtomicU64::new(0),
                closed: AtomicBool::new(false),
                health: source::Health::new(),
            });
            let apic_hz = calibrate(&cpu)?;
            let now = cpu.sample()?;
            let state = Arc::new(State {
                cpu_view: cpu.clone(),
                cpu,
                apic_hz,
                pending: AtomicBool::new(false),
                executing: AtomicBool::new(false),
                execution_generation: AtomicU64::new(0),
                guest: AtomicU64::new(NONE),
                idle: AtomicU64::new(NONE),
                heartbeat: AtomicU64::new(now.checked_add(TICK_NS).ok_or(WorkError::Exhausted)?),
            });
            // Complete fallible hardware validation before publishing an IRQ
            // registration. A changed controller must leave no orphaned Arc.
            state.rearm(now)?;
            // The explicit raw strong reference is registration ownership;
            // removal is same-CPU IRQ-excluded and precedes from_raw release.
            ACTIVE_GS.store(state.cpu.gs, Ordering::Release);
            ACTIVE.store(Arc::into_raw(state.clone()).cast_mut(), Ordering::Release);
            TIMER_HANDOVERS.fetch_add(1, Ordering::Release);
            let clock: Arc<dyn Clock> = Arc::new(NativeClock(state.clone()));
            let idle: Arc<dyn Idle> = Arc::new(NativeIdle {
                state: state.clone(),
                clock: clock.clone(),
            });
            Ok(Self {
                state: ManuallyDrop::new(state),
                clock: ManuallyDrop::new(clock),
                idle: ManuallyDrop::new(idle),
                _local: PhantomData,
            })
        })();
        // SAFETY: enclosing owner-local save, no switch occurred.
        unsafe {
            restore(flags);
        }
        drop(deferred_source);
        result
    }
    /// Only the real installed native source can mint a continuous epoch.
    pub(crate) fn epoch(&self) -> Result<CounterEpoch, WorkError> {
        self.state.process()?;
        self.state.cpu.health.check()?;
        self.state
            .cpu
            .source
            .continuous_period()
            .map_err(|error| self.state.cpu.health.fail(error))?;
        self.state.cpu.now()?;
        Ok(CounterEpoch {
            cpu: self.state.cpu.clone(),
        })
    }

    pub(crate) fn services(&self) -> Result<ClockServices, WorkError> {
        self.epoch()?;
        Ok(ClockServices {
            state: Arc::clone(&self.state),
            clock: Arc::clone(&self.clock),
            idle: Arc::clone(&self.idle),
        })
    }
    /// # Safety
    /// Called only by the main/root dispatcher, which grants interrupt
    /// delivery through the existing IDT/IST/xstate/LAPIC ownership.
    /// All ISR-reachable publications/code/storage are installed and kept;
    /// no mutable Host State borrow or native/foreign critical scope survives
    /// this entry. All driver continuations and foreign frames return/join
    /// before dropping the grant. Preserve the source's checked sample bound.
    pub(crate) unsafe fn enter_driver_execution(&self) -> Result<DriverExecution<'_>, WorkError> {
        self.state.process()?;
        let flags = flags_disable();
        if self.state.executing.load(Ordering::Acquire) {
            // SAFETY: `flags` came from `flags_disable` on this CPU above.
            unsafe {
                restore(flags);
            }
            return Err(WorkError::Busy);
        }
        let generation = match self
            .state
            .execution_generation
            .load(Ordering::Relaxed)
            .checked_add(1)
        {
            Some(generation) => generation,
            None => {
                // SAFETY: `flags` came from `flags_disable` on this CPU above.
                unsafe {
                    restore(flags);
                }
                return Err(WorkError::Exhausted);
            }
        };
        self.state
            .execution_generation
            .store(generation, Ordering::Release);
        self.state.executing.store(true, Ordering::Release);
        let root = root();
        // SAFETY: this is the explicit main-owned execution permission, never
        // an implicit attempt by Idle/Runtime to manufacture interrupt progress.
        unsafe {
            asm!("sti", options(nostack));
        }
        Ok(DriverExecution {
            installed: self,
            flags,
            root,
            generation,
            process_borrowed: Cell::new(false),
            _local: PhantomData,
        })
    }
}

fn calibrate(cpu: &RootCpu) -> Result<u64, WorkError> {
    let apic = cpu.controller();
    let saved_lvt = apic.read(apic::LVT_TIMER);
    let saved_divide = apic.read(apic::DIVIDE);
    apic.write(
        apic::LVT_TIMER,
        apic::LVT_MASKED | u32::from(interrupts::TIMER_VECTOR),
    );
    apic.write(apic::DIVIDE, 0b1011); // Existing engine divider: one.
    apic.write(apic::INITIAL, u32::MAX);
    let result = (|| {
        let begin_ns = cpu.sample()?;
        let begin_apic = apic.read(apic::CURRENT);
        for _ in 0..10_000_000 {
            let now = cpu.sample()?;
            if now - begin_ns >= 10_000_000 {
                let end_apic = apic.read(apic::CURRENT);
                if end_apic == 0 || end_apic >= begin_apic {
                    return Err(WorkError::Unavailable);
                }
                let hz = u128::from(begin_apic - end_apic) * u128::from(pm::NS)
                    / u128::from(now - begin_ns);
                return u64::try_from(hz)
                    .ok()
                    .filter(|hz| *hz != 0)
                    .ok_or(WorkError::Unavailable);
            }
            core::hint::spin_loop();
        }
        Err(WorkError::TimedOut)
    })();
    apic.write(apic::INITIAL, 0);
    apic.write(apic::LVT_TIMER, saved_lvt);
    if result.is_err() {
        apic.write(apic::DIVIDE, saved_divide);
    }
    result
}

/// Route a host wait through the installed LAPIC owner, when present.
pub(crate) fn host_sleep(micros: u64) -> bool {
    let flags = flags_disable();
    let handled = host_sleep_disabled(micros);
    // SAFETY: restore the enclosing same-CPU save after returning from idle.
    unsafe { restore(flags) };
    handled
}

fn host_sleep_disabled(micros: u64) -> bool {
    if !registration_cpu() {
        return false;
    }
    let pointer = ACTIVE.load(Ordering::Acquire);
    if pointer.is_null() {
        return false;
    }
    // SAFETY: same-CPU IRQ exclusion keeps the active registration's Arc.
    let state = unsafe { &*pointer };
    if state.process().is_ok() {
        let result = (|| {
            let now = state.cpu.sample()?;
            let delta = micros.checked_mul(1000).ok_or(WorkError::Exhausted)?;
            let deadline = now.checked_add(delta).ok_or(WorkError::Exhausted)?;
            // SAFETY: same installed host root/CPU and existing host wait
            // boundary, with local IRQ exclusion already held.
            unsafe { state.wait_until(Some(deadline)) }
        })();
        if let Err(error) = result {
            state.cpu.fatal(error);
        }
    }
    true
}

/// Existing host owner routes its guest timer count here when registered.
/// Saves/restores local IRQ state itself, including callers already at IF=0.
/// No independent vector namespace is allocated.
pub(crate) fn arm_guest(ticks: Option<u32>) -> bool {
    let flags = flags_disable();
    let result = arm_guest_disabled(ticks);
    // SAFETY: same-CPU save; no callback or switch occurred in the mux.
    unsafe {
        restore(flags);
    }
    result
}
fn arm_guest_disabled(ticks: Option<u32>) -> bool {
    if !registration_cpu() {
        return false;
    }
    let pointer = ACTIVE.load(Ordering::Acquire);
    if pointer.is_null() {
        return false;
    }
    // SAFETY: IRQ-excluded pointer owned by the real same-CPU registration.
    let state = unsafe { &*pointer };
    if state.cpu.affinity().is_err() {
        return false;
    }
    let result = (|| {
        let now = state.cpu.sample()?;
        let deadline = match ticks {
            Some(ticks) => now
                .checked_add(
                    u64::try_from(
                        (u128::from(ticks) * u128::from(pm::NS))
                            .div_ceil(u128::from(state.apic_hz)),
                    )
                    .map_err(|_| WorkError::Exhausted)?,
                )
                .ok_or(WorkError::Exhausted)?,
            None => NONE,
        };
        state.guest.store(deadline, Ordering::Relaxed);
        state.rearm(now)
    })();
    if let Err(error) = result {
        state.cpu.fatal(error);
    }
    true
}

/// Fast path called by the existing timer-vector entry BEFORE its EOI.
/// It samples elapsed time, advances the heartbeat, and rearms the shared
/// minimum. It enters no Linux owner or original callback.
pub(crate) fn interrupt() {
    if !registration_cpu() {
        return;
    }
    let pointer = ACTIVE.load(Ordering::Acquire);
    if pointer.is_null() {
        return;
    }
    // SAFETY: same-CPU IRQ entry/removal exclusion keeps registration's Arc.
    let state = unsafe { &*pointer };
    if state.cpu.affinity().is_err() {
        return;
    } // Other physical CPUs own their timers.
    let result = (|| {
        let now = state.cpu.sample()?;
        let heartbeat = state.heartbeat.load(Ordering::Relaxed);
        if now >= heartbeat {
            let phases = (now - heartbeat) / TICK_NS + 1;
            state.heartbeat.store(
                heartbeat
                    .checked_add(phases.checked_mul(TICK_NS).ok_or(WorkError::Exhausted)?)
                    .ok_or(WorkError::Exhausted)?,
                Ordering::Relaxed,
            );
        }
        if state.guest.load(Ordering::Relaxed) <= now {
            state.guest.store(NONE, Ordering::Relaxed);
        }
        if state.idle.load(Ordering::Relaxed) <= now {
            state.idle.store(NONE, Ordering::Relaxed);
            state.pending.store(true, Ordering::Release);
        }
        state.rearm(now)
    })();
    if let Err(error) = result {
        state.cpu.fatal(error);
    }
}
