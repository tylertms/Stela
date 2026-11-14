//! The host's Linux environment and the NVIDIA GPUs nouveau drives in it,
//! which back the guest's Metal device.
//!
//! It lives on the boot CPU and takes time from the boot PE as Linux steals it
//! from an interrupted thread. A GPU's MSI targets that CPU, which takes it
//! between slices or when the PE waits, and only records its line ([`Fired`]);
//! the run loop then runs what is ready ([`pump`]) and the guest goes on.
//! Like the host's service, it waits for a boot PE that holds no guest lock,
//! waiting for an interrupt or at EL0, but only for a bounded time
//! ([`pump_due`]), so other PEs do not spin on a lock while Linux runs and
//! GPU completions are not starved. Nothing reserves a core, and a Linux
//! environment with nothing to do costs the boot PE one comparison a slice.
#[cfg(feature = "nvk")]
pub(super) mod client;
#[cfg(feature = "nvk")]
pub(super) mod display;
#[cfg(feature = "nvk")]
mod file;
#[cfg(feature = "nvk")]
pub(super) mod gate;
/// `xtask guest`'s objects of the StelaGPU kext.
#[cfg(feature = "nvk")]
mod kext {
    include!(concat!(
        env!("CARGO_MANIFEST_DIR"),
        "/../../target/guest/StelaGPU.kext.rs"
    ));
}
#[cfg(feature = "nvk")]
mod stack;
#[cfg(feature = "nvk")]
mod waiter;

use super::{
    host,
    kernel_work::{self, KernelPages, KernelWork, SLOTS},
};
use crate::{handoff::RtcReading, native_irq, native_memory, native_mmio::Window, output};
use alloc::{
    borrow::ToOwned, boxed::Box, collections::BTreeMap, collections::BTreeSet, format, rc::Rc,
    string::String, sync::Arc, vec::Vec,
};
use core::{
    alloc::Layout,
    cell::{Cell, RefCell},
    ffi::{CStr, c_void},
    mem::ManuallyDrop,
    ops::Range,
    pin::Pin,
    ptr::{self, addr_of},
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, AtomicUsize, Ordering},
};
use stela_boot_core::page_window::{self, Cache};
#[cfg(feature = "nvk")]
use stela_drivers::gpu::metal::nvidia::PcieLink;
use stela_drivers::{
    bus::{
        irq::{Disposition, Handler, IrqError, IrqRegistration, Route, Source},
        mmio::owned::{self, CacheAttribute, Mapping, Protection},
        pci::{
            Bar, ConfigError, ConfigFailure, ConfigSpace, Function,
            owned::{ConfigRetention, OwnedConfig},
        },
    },
    firmware::{Catalog, ErrorKind as FirmwareError, Provider},
    gpu,
    platform::work::{
        WorkError,
        service::{Clock, Domain, Idle},
    },
    probe::{NativeServices, ServiceError},
};
use stela_engine::machine::engine::Hub;
use stela_linux::{
    catalogue::{FirmwareTarget, FunctionCustody, PciAuthority, PciFirmware},
    function::{self, Attached, Msi, Placement, Region},
    kernel::KernelOwner,
    owner::{Errno, Platform, RealtimeOrigin, RealtimeRead},
    random::Seeded,
    service::Environment as LinuxEnvironment,
    task::runtime::{ManagedJob, Outcome, TaskRuntime},
    threads::{KernelThreadPlatform, TaskDataBacking},
};

type Environment = LinuxEnvironment<SLOTS>;
type LinuxKernel = KernelOwner<Host, SLOTS>;
type LinuxThreads = TaskRuntime<Host, KernelPages, SLOTS>;

const PAGE: u64 = page_window::PAGE;
const EIO: Errno = -5;
const EFAULT: Errno = -14;
const ERANGE: Errno = -34;
const ENODEV: Errno = -19;
const EINVAL: Errno = -22;
const ENOMEM: Errno = -12;
const EOPNOTSUPP: Errno = -95;

const MSI_CAPABILITY: u8 = 5;
#[cfg(feature = "nvk")]
const PCIE_CAPABILITY: u8 = 0x10;
/// `PCI_EXP_LNKSTA`, in the PCIe capability.
#[cfg(feature = "nvk")]
const LINK_STATUS: u16 = 0x12;
#[cfg(feature = "nvk")]
const REVISION: u16 = 0x08;
#[cfg(feature = "nvk")]
const SUBSYSTEM_ID: u16 = 0x2c;
/// Linux's `IORESOURCE_*` bits of a memory BAR.
const RESOURCE_MEMORY: u64 = 0x200;
const RESOURCE_PREFETCHABLE: u64 = 0x2000;
const RESOURCE_WIDE: u64 = 0x10_0000;
/// Every DMA mask nouveau sets on the GPUs Stela drives holds 47 bits: the
/// Turing-and-later MMUs' `dma_bits` (`nvkm/subdev/mmu/tu102.c` 47,
/// `gh100.c` 52), which `nvkm_device_pci_new` sets as the mask. A mapping
/// past it is refused (`function::Mappings`), not truncated.
const DMA_LIMIT: u64 = (1 << 47) - 1;
/// How long the GPUs may take to boot their firmware before the guest starts.
const START_SECONDS: u64 = 60;
/// The longest idle wait while they do, which the timeout above is checked at.
const WAIT_MICROS: u64 = 10_000;
/// How often a long wait in the start says where it is ([`wait_for_child`]).
const WAIT_REPORT_SECONDS: u64 = 2;
/// The longest one pump runs Linux between the boot PE's slices. What is
/// still runnable stays pending for the next, so a busy driver shares the
/// boot CPU with the guest and the host's devices instead of holding it.
const PUMP_NS: u64 = 2_000_000;

/// Who runs PE work on the boot CPU, once the PEs run.
static HUB: AtomicPtr<Hub> = AtomicPtr::new(ptr::null_mut());
static BOOT_PE: AtomicUsize = AtomicUsize::new(0);
/// The processor the environment runs on.
#[cfg(feature = "nvk")]
static BOOT_CPU: AtomicU32 = AtomicU32::new(0);

/// End the boot PE's slices for Linux, through `hub`.
pub(super) fn set_hub(hub: Arc<Hub>, pe: usize) {
    BOOT_PE.store(pe, Ordering::Relaxed);
    HUB.store(Arc::into_raw(hub).cast_mut(), Ordering::Release);
}

/// What wakes the service: a raised line or a call.
struct Events {
    clock: crate::native_clock::ClockServices,
    idle: Arc<dyn Idle>,
    pending: AtomicBool,
}

// SAFETY: the native idle owner consumes the software event under its own
// IRQ exclusion. Publish that event before ringing the boot PE, and use the
// paired installed clock for each absolute wait.
unsafe impl Idle for Events {
    fn clock(&self) -> &Arc<dyn Clock> {
        self.idle.clock()
    }
    fn notify(&self) -> Result<(), WorkError> {
        let raised = !self.pending.swap(true, Ordering::AcqRel);
        if raised {
            PENDING_SINCE.store(host::rdtsc(), Ordering::Release);
        }
        self.clock.notify_dispatch();
        // Only the first event since the last pump ends the boot PE's slice:
        // later ones find it already called, and the pump takes them all.
        // SAFETY: a published hub is leaked, so it lives for the boot.
        if raised && let Some(hub) = unsafe { HUB.load(Ordering::Acquire).as_ref() } {
            hub.call_host(BOOT_PE.load(Ordering::Relaxed));
        }
        Ok(())
    }
    fn wait_until_ns(&self, deadline: Option<u64>) -> Result<(), WorkError> {
        let limit = self
            .idle
            .clock()
            .now_ns()?
            .checked_add(WAIT_MICROS * 1000)
            .ok_or(WorkError::Exhausted)?;
        self.idle
            .wait_until_ns(Some(deadline.map_or(limit, |deadline| deadline.min(limit))))
    }
}

unsafe fn allocate_zeroed(layout: Layout) -> *mut u8 {
    // SAFETY: the registration's contract, which this allocator's one attempt
    // at its lock meets; the C side frees through the global allocator.
    unsafe { crate::ALLOCATOR.try_allocate_zeroed(layout) }
}

/// A function's configuration space, which its MSI source shares with Linux.
/// The two take turns on the boot CPU, and neither calls out while it holds it.
#[derive(Clone)]
struct SharedConfig(Rc<RefCell<Function<OwnedConfig>>>);

impl ConfigSpace for SharedConfig {
    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        self.0.borrow_mut().config().read32(offset)
    }
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError> {
        self.0.borrow_mut().config().write32(offset, value)
    }
    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        self.0.borrow_mut().config().read16(offset)
    }
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        self.0.borrow_mut().config().read8(offset)
    }
    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
        self.0.borrow_mut().config().write16(offset, value)
    }
    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError> {
        self.0.borrow_mut().config().write8(offset, value)
    }
}

/// The MSI of one function, routed to the boot CPU.
struct MsiSource {
    config: SharedConfig,
    msi: Msi,
    /// The function the route was attached for, which only it may program.
    requester: stela_drivers::bus::pci::requester::RequesterId,
    programmed: bool,
    event: stela_linux::service::IrqEvent,
    environment: *const Environment,
    line: u32,
}

// SAFETY: the function was stopped when claimed, with no route of its own.
// Nothing here frees: config/capability stay with the registration. Terminal
// engine/payload/Linux-work joining is unavailable, so a programmed route's
// close refuses and retains owners even if nouveau masked its local source.
unsafe impl Source for MsiSource {
    fn mask(&mut self) -> Result<(), IrqError> {
        let command = self.config.read16(4)?;
        if command == u16::MAX {
            return Err(IrqError::FailedRead);
        }
        self.config.write16(4, command | 1 << 10)?;
        self.msi.disable(&mut self.config)?;
        let actual = self.config.read16(4)?;
        let control = self.msi.control(&self.config)?;
        if actual == u16::MAX || control == u16::MAX {
            return Err(IrqError::FailedRead);
        }
        if actual & (1 << 10) == 0 || control & 1 != 0 {
            return Err(IrqError::Mask);
        }
        Ok(())
    }
    fn quiesce(&mut self) -> Result<(), IrqError> {
        if self.programmed {
            // GPU engine/payload and Linux service work have no independent
            // joined terminal acknowledgement here. Retain the old route.
            return Err(IrqError::Unsupported);
        }
        self.mask()
    }
    fn route_kind(&self) -> stela_drivers::bus::irq::RouteKind {
        stela_drivers::bus::irq::RouteKind::Msi
    }
    fn requester(&self) -> Option<&stela_drivers::bus::pci::requester::RequesterId> {
        Some(&self.requester)
    }
    unsafe fn program(
        &mut self,
        route: &stela_drivers::bus::irq::AllocatedRoute<'_>,
    ) -> Result<(), IrqError> {
        let Route::Msi(message) = route.programming() else {
            return Err(IrqError::Route);
        };
        let data = u16::try_from(message.data).map_err(|_| IrqError::Route)?;
        self.programmed = true;
        self.msi
            .program(&mut self.config, message.address, data)
            .map_err(IrqError::from)
    }
    unsafe fn unmask(&mut self) -> Result<(), IrqError> {
        self.msi.enable(&mut self.config)?;
        let control = self.msi.control(&self.config)?;
        if control == u16::MAX {
            return Err(IrqError::FailedRead);
        }
        if control & 1 == 0 {
            return Err(IrqError::Mask);
        }
        Ok(())
    }
    unsafe fn synchronize(&mut self) -> Result<(), IrqError> {
        unsafe extern "C" {
            fn stela_linux_irq_actions_joined(irq: u32) -> bool;
        }
        // SAFETY: native callback runs below the exact entered Linux owner;
        // C free_irq retains its closing request and joined work until return.
        if !unsafe { stela_linux_irq_actions_joined(self.line) } {
            return Err(IrqError::Drain);
        }
        self.event.begin_close();
        // SAFETY: Registry already closed this exact action and drained native
        // CPU/inflight/EOI; the original pinned environment/event are retained.
        unsafe { (&*self.environment).join_irq(&self.event) }.map_err(|_| IrqError::Drain)
    }
}

/// The GPU interrupts taken, which the host beat reports.
static FIRED: AtomicU64 = AtomicU64::new(0);

/// The GPU interrupts taken since boot.
pub(super) fn interrupts() -> u64 {
    FIRED.load(Ordering::Relaxed)
}

/// Records an interrupt for the service task.
struct Fired {
    event: stela_linux::service::IrqEvent,
}

impl Handler for Fired {
    fn handle(&self) -> Disposition {
        FIRED.fetch_add(1, Ordering::Relaxed);
        match self.event.raise() {
            Ok(()) => Disposition::Handled,
            Err(_) => Disposition::NotMine,
        }
    }
}

/// A function as Linux's PCI core reaches it.
struct Device {
    config: SharedConfig,
    msi: Option<Msi>,
    custody: Rc<FunctionCustody>,
    _views: ManuallyDrop<GpuViews>,
    _resources: ManuallyDrop<Vec<owned::Resource>>,
}

impl function::Host for Device {
    type Config = SharedConfig;
    fn config(&mut self) -> &mut SharedConfig {
        &mut self.config
    }
    fn enable_msi(&mut self) -> Result<(), function::MsiError> {
        // The line's route programs it, when nouveau requests its interrupt.
        self.msi
            .map(|_| ())
            .ok_or(function::MsiError::Native(EOPNOTSUPP))
    }
    fn disable_msi(&mut self) -> Result<(), function::MsiError> {
        if let Some(msi) = self.msi {
            msi.disable(&mut self.config)?;
        }
        Ok(())
    }
}

/// An interrupt line Linux knows a function by, its index in `Host::lines`.
struct Line {
    config: SharedConfig,
    msi: Option<Msi>,
    custody: Rc<FunctionCustody>,
    registration: Option<Box<dyn IrqRegistration>>,
    retired: Option<stela_drivers::bus::irq::RetiredSource<dyn Source>>,
}

/// What Linux code gets from Stela.
struct Host {
    work: &'static KernelWork,
    task_data: Arc<dyn TaskDataBacking>,
    tsc_hz: u64,
    clock: crate::native_clock::CounterEpoch,
    realtime: Option<RealtimeOrigin>,
    window: &'static Window,
    image: Arc<kernel_work::ReadOnlyImage>,
    random: RefCell<Seeded>,
    environment: Cell<*const Environment>,
    /// The memory BARs Stela claimed, which are all Linux may map.
    bars: Vec<Range<u64>>,
    /// Device mappings' sizes in pages, by address.
    devices: RefCell<BTreeMap<u64, usize>>,
    lines: RefCell<Vec<Line>>,
    /// The unsupported operations already reported.
    reported: RefCell<BTreeSet<String>>,
}

impl Host {
    fn environment(&self) -> Pin<&Environment> {
        // SAFETY: set before any task runs, to a pinned box kept for the boot.
        unsafe { Pin::new_unchecked(&*self.environment.get()) }
    }

    fn map(&self, pages: usize, frame: impl Fn(u64) -> u64, cache: Cache) -> Option<*mut c_void> {
        let address = self.window.reserve(pages)?;
        for page in 0..pages as u64 {
            if !self
                .window
                .install(address + page * PAGE, frame(page), cache)
            {
                self.unmap(address, pages);
                return None;
            }
        }
        Some(address as *mut c_void)
    }

    fn unmap(&self, address: u64, pages: usize) {
        // A processor that kept the translation keeps the addresses too.
        if self.window.remove(address, pages) {
            self.window.release(address, pages);
        }
    }

    fn report(&self, line: core::fmt::Arguments<'_>) {
        output::arguments(format_args!("LINUX {line}\n"));
    }
}

// SAFETY: work owns the original queue, code and root. Its live dispatcher
// scope authenticates the same CPU and loader root for each bounded loan.
// task_data separately pins the original writable image and heap generation.
unsafe impl KernelThreadPlatform for Host {
    fn task_data_backing(&self) -> Result<Arc<dyn TaskDataBacking>, WorkError> {
        Ok(self.task_data.clone())
    }

    unsafe fn with_process<R>(
        &self,
        body: impl FnOnce(&dyn stela_drivers::platform::work::EnteredProcess) -> R,
    ) -> Result<R, WorkError> {
        // SAFETY: the caller supplies the bounded, non-switching metadata leaf.
        unsafe { self.work.with_linux_process(body) }
    }
}

impl Platform for Host {
    fn calibrated_tsc_hz(&self) -> Option<core::num::NonZeroU64> {
        core::num::NonZeroU64::new(self.tsc_hz)
    }

    fn now_ns(&self) -> i64 {
        let sample = self.clock.now_ns().and_then(|now| {
            i64::try_from(now)
                .ok()
                .filter(|&now| now != i64::MAX)
                .ok_or(WorkError::Exhausted)
        });
        sample.unwrap_or_else(|error| output::fatal(format_args!("Linux clock failed: {error}")))
    }

    fn boottime_ns(&self) -> Result<u64, WorkError> {
        // The retained native counter epoch starts before Linux starts, and
        // its admission excludes suspend. Both Linux clocks share this epoch.
        self.clock.now_ns()
    }

    fn realtime_at(&self, ns: i64, read: RealtimeRead) -> Option<(i64, u32)> {
        self.realtime.as_ref()?.read(ns, read)
    }

    fn minimum_delay_us(&self, minimum: u64) -> Result<(), WorkError> {
        self.clock.minimum_delay_us(minimum)
    }

    fn minimum_deadline_us(
        &self,
        minimum: u64,
    ) -> Result<Box<dyn stela_linux::owner::MinimumDeadline>, WorkError> {
        let nanoseconds = minimum.checked_mul(1000).ok_or(WorkError::Exhausted)?;
        Ok(Box::new(self.clock.minimum_deadline(nanoseconds)?))
    }

    fn log(&self, level: i32, message: &CStr) {
        let message = message.to_str().unwrap_or("(not UTF-8)").trim_end();
        self.report(format_args!("<{level}> {message}"));
    }

    fn warn(&self, file: &CStr, line: i32, condition: Option<&CStr>) {
        let file = file.to_str().unwrap_or("(not UTF-8)");
        let condition = condition.and_then(|text| text.to_str().ok()).unwrap_or("");
        self.report(format_args!("WARN {file}:{line} {condition}"));
    }

    fn unsupported(&self, operation: &CStr) {
        let operation = operation.to_str().unwrap_or("(not UTF-8)");
        if self.reported.borrow_mut().insert(operation.to_owned()) {
            self.report(format_args!("unsupported: {operation}"));
        }
    }

    fn random_bytes(&self, bytes: &mut [u8]) {
        self.random.borrow_mut().fill(bytes);
    }

    fn mix_random(&self, bytes: &[u8]) -> bool {
        self.random.borrow_mut().mix(host::rdtsc(), bytes);
        true
    }

    fn timer_changed(&self) -> Result<(), WorkError> {
        // SAFETY: after publication this points to the original pinned
        // environment, kept for the boot. A prepublication call is refused.
        let environment =
            unsafe { self.environment.get().as_ref() }.ok_or(WorkError::Unavailable)?;
        environment.timer_changed()
    }

    fn shutdown(&self, force: bool) {
        super::service::request_shutdown(force);
    }

    fn map_device(&self, physical: u64, size: usize, write_combining: bool) -> Option<*mut c_void> {
        let end = physical.checked_add(size as u64)?;
        if !self
            .bars
            .iter()
            .any(|bar| bar.start <= physical && end <= bar.end)
        {
            return None;
        }
        let pages = size / PAGE as usize;
        let cache = if write_combining {
            Cache::WriteCombining
        } else {
            Cache::Uncached
        };
        let address = self.map(pages, |page| physical + page * PAGE, cache)?;
        self.devices.borrow_mut().insert(address as u64, pages);
        Some(address)
    }

    unsafe fn unmap_device(&self, address: *mut c_void) {
        let pages = self.devices.borrow_mut().remove(&(address as u64));
        if let Some(pages) = pages {
            unmapped(Unmapped::Device);
            // Device memory never becomes RAM, and Linux drops the mapping
            // with its last use: its shootdown waits for others'. Page
            // tables nouveau frees cost one iounmap each.
            self.window.remove_lazily(address as u64, pages);
        }
    }

    fn map_pages(&self, physical: &[u64]) -> Option<*mut c_void> {
        self.map(
            physical.len(),
            |page| physical[page as usize],
            Cache::WriteBack,
        )
    }

    unsafe fn unmap_pages(&self, address: *mut c_void, count: usize) {
        unmapped(Unmapped::Pages);
        self.unmap(address as u64, count);
    }

    fn irq_enable(&self, irq: u32) -> Result<(), stela_linux::owner::IrqEnableError> {
        use stela_linux::owner::IrqEnableError;
        // Refuse a foreign native execution context before classifying any
        // owner return; no errno itself proves a clean unpublished attempt.
        let controller = native_irq::service().map_err(|_| IrqEnableError::retained(ENODEV))?;
        let mut lines = self.lines.borrow_mut();
        let Some(line) = lines.get_mut(irq as usize) else {
            // SAFETY: checked original process, before any route/event/source.
            return Err(unsafe { IrqEnableError::unexposed(EINVAL) });
        };
        if line.retired.is_some() {
            return Err(IrqEnableError::retained(EOPNOTSUPP));
        }
        if let Some(registration) = &mut line.registration {
            return registration
                .unmask()
                .map_err(|_| IrqEnableError::retained(EIO));
        }
        let Some(msi) = line.msi else {
            // SAFETY: no native attempt or service epoch was issued.
            return Err(unsafe { IrqEnableError::unexposed(EOPNOTSUPP) });
        };
        // The attachment's own route: its root decides how messages arrive.
        // SAFETY: a short borrow on the original process; nothing reenters.
        let route = match unsafe {
            line.custody
                .with_native(|attachment| attachment.irq_route())
        } {
            Ok(Ok(route)) => route,
            // SAFETY: no route, event or source was issued.
            _ => return Err(unsafe { IrqEnableError::unexposed(EOPNOTSUPP) }),
        };
        // SAFETY: actual original entered process; any preceding epoch joined.
        let event = unsafe { self.environment().irq_epoch(irq) }
            .map_err(|_| IrqEnableError::retained(EIO))?;
        let source = Box::new(MsiSource {
            config: line.config.clone(),
            msi,
            requester: line.custody.requester().clone(),
            programmed: false,
            event: event.clone(),
            environment: self.environment.get(),
            line: irq,
        });
        let handler = Arc::new(Fired {
            event: event.clone(),
        });
        match controller.request(stela_drivers::bus::irq::Request {
            source,
            handler,
            routing: stela_drivers::bus::irq::RouteRequest::Msi(route),
        }) {
            Ok(registration) => {
                line.registration = Some(registration);
                Ok(())
            }
            Err(refusal) => {
                let errno = match refusal.error {
                    stela_drivers::bus::irq::IrqError::Exhausted => ENOMEM,
                    stela_drivers::bus::irq::IrqError::Unavailable => ENODEV,
                    stela_drivers::bus::irq::IrqError::Unsupported
                    | stela_drivers::bus::irq::IrqError::Delivery(
                        stela_drivers::bus::irq::posted_msi::Error::RootUnavailable
                        | stela_drivers::bus::irq::posted_msi::Error::Unsupported,
                    ) => EOPNOTSUPP,
                    _ => EIO,
                };
                if errno == EOPNOTSUPP {
                    self.unsupported(c"interrupt notification admission");
                } else {
                    self.report(format_args!("interrupt {irq} refused: {refusal:?}"));
                }
                let Some(untouched) = refusal.returned else {
                    return Err(IrqEnableError::retained(EIO));
                };
                // SAFETY: actual checked prepublication return; extraction
                // repeats the original process/context/provider checks.
                let (source, handler, routing) =
                    unsafe { untouched.into_parts() }.map_err(|_| IrqEnableError::retained(EIO))?;
                // Retain actual inputs before any fallible cancellation/join.
                let mut source = core::mem::ManuallyDrop::new(source);
                let mut handler = core::mem::ManuallyDrop::new(handler);
                let stela_drivers::bus::irq::RouteRequest::Msi(route) = routing else {
                    return Err(IrqEnableError::retained(EIO));
                };
                // SAFETY: a short borrow on the original process; the grant
                // was never published, and a failed cancel keeps it retained.
                let cancelled = unsafe {
                    line.custody.with_native(|attachment| {
                        // A failed cancel hands the grant back; keep it.
                        attachment
                            .cancel_unpublished_irq(route)
                            .map_err(core::mem::forget)
                            .is_ok()
                    })
                };
                if cancelled != Ok(true) {
                    return Err(IrqEnableError::retained(EIO));
                }
                event.begin_close();
                // SAFETY: RequestError returned actual untouched owners before
                // Slot/root/source publication, so this new event has no native
                // callback producer. Join still checks exact epoch/activity.
                unsafe { self.environment().join_irq(&event) }
                    .map_err(|_| IrqEnableError::retained(EIO))?;
                // SAFETY: same original process, unused route canceled and exact
                // new service epoch actually joined; no C action reachability.
                unsafe {
                    core::mem::ManuallyDrop::drop(&mut source);
                    core::mem::ManuallyDrop::drop(&mut handler);
                }
                // SAFETY: all original return/cancel/join facts above, not errno.
                Err(unsafe { IrqEnableError::unexposed(errno) })
            }
        }
    }

    fn irq_disable(&self, irq: u32) -> Result<(), Errno> {
        let mut lines = self.lines.borrow_mut();
        let line = lines.get_mut(irq as usize).ok_or(EINVAL)?;
        match &mut line.registration {
            Some(registration) => registration.mask().map_err(|_| EIO),
            None => Ok(()),
        }
    }
    fn irq_retire(&self, irq: u32) -> Result<(), Errno> {
        let mut lines = self.lines.borrow_mut();
        let line = lines.get_mut(irq as usize).ok_or(EINVAL)?;
        if line.retired.is_some() {
            return Err(EOPNOTSUPP);
        }
        let Some(registration) = line.registration.as_mut() else {
            return Ok(());
        };
        match registration.retire() {
            Err(error) => {
                self.report(format_args!("interrupt {irq} action retained: {error:?}"));
                Err(EOPNOTSUPP)
            }
            Ok(retired) => {
                // GPU has no original endpoint/entry delivery factory or
                // custody transfer yet. Do not return completed free_irq from
                // a future partial source implementation; keep typed owners.
                line.retired = Some(retired);
                Err(EOPNOTSUPP)
            }
        }
    }

    unsafe fn copy_from_user(&self, to: *mut u8, from: *const u8, size: usize) -> usize {
        // SAFETY: clients are in-process, so their addresses are ordinary
        // valid memory by the caller's contract.
        unsafe { ptr::copy_nonoverlapping(from, to, size) };
        0
    }

    unsafe fn copy_to_user(&self, to: *mut u8, from: *const u8, size: usize) -> usize {
        // SAFETY: as in `copy_from_user`.
        unsafe { ptr::copy_nonoverlapping(from, to, size) };
        0
    }

    unsafe fn copy_from_kernel_nofault(
        &self,
        to: *mut u8,
        from: *const u8,
        size: usize,
    ) -> Result<(), Errno> {
        if native_irq::current_context() != Ok(None) {
            return Err(EFAULT);
        }
        let heap = native_memory::SOURCE.registration();
        let in_heap = heap.generation != 0
            && from
                .addr()
                .checked_sub(heap.cpu)
                .is_some_and(|offset| offset <= heap.bytes && size <= heap.bytes - offset);
        if !in_heap && !self.image.contains_range(from.addr(), size) {
            return Err(ERANGE);
        }
        if from.addr().abs_diff(to.addr()) < size {
            return Err(EINVAL);
        }
        // SAFETY: the retained heap or immutable image owns the full source
        // range. The engine host IDT is active on this CPU, and the C caller
        // owns the disjoint writable destination.
        if unsafe { stela_boot_core::nofault_read::read(to, from, size) } == 0 {
            Ok(())
        } else {
            Err(EFAULT)
        }
    }

    fn available_page_frames(&self, start: usize, pages: usize) -> Result<usize, Errno> {
        if native_irq::current_context() != Ok(None) {
            return Err(EFAULT);
        }
        let heap = native_memory::SOURCE.registration();
        let bytes = pages
            .checked_mul(stela_boot_core::native_memory::PAGE)
            .ok_or(ERANGE)?;
        let end = start.checked_add(bytes).ok_or(ERANGE)?;
        if heap.generation == 0 || start < heap.cpu || end - heap.cpu > heap.bytes {
            return Err(ERANGE);
        }
        Ok(crate::ALLOCATOR.available_page_frames_in(start..end))
    }

    fn map_user(&self, space: *mut c_void, address: u64, pfn: u64, prot: u64) -> Result<(), Errno> {
        if !space.is_null() {
            return Err(EOPNOTSUPP);
        }
        let entry = page_window::client_leaf(pfn, prot).ok_or(EINVAL)?;
        if self.window.install_client(address, entry) {
            Ok(())
        } else {
            Err(ENOMEM)
        }
    }

    fn unmap_user(&self, space: *mut c_void, address: u64, size: usize) -> Result<(), Errno> {
        if !space.is_null() {
            return Err(EOPNOTSUPP);
        }
        if self.window.remove(address, size / PAGE as usize) {
            Ok(())
        } else {
            Err(EIO)
        }
    }
}

/// The original pre-reset register observation and configuration epoch.
struct GpuViews {
    config: ConfigRetention,
    pci_words: [u32; 8],
    registers: Mapping,
    boot0: u32,
}

fn gpu_pci_words(function: &mut Function<OwnedConfig>) -> Result<[u32; 8], ConfigError> {
    // Identity and class/revision, then all six restored BAR registers.
    let mut words = [0; 8];
    for (word, offset) in words
        .iter_mut()
        .zip([0, 8, 0x10, 0x14, 0x18, 0x1c, 0x20, 0x24])
    {
        *word = function.config().read32(offset)?;
    }
    Ok(words)
}

// SAFETY: the exclusive original preparation owns this exact configuration
// and register BAR. Any original GOP writers were joined before BAR sizing,
// and its terminal admission remains closed. This checks the same retained
// epoch and reads a harmless UC register on this function after the CPU fence,
// completing its posted writes before Cold is allowed to reset it.
unsafe impl gpu::cold::Views<OwnedConfig> for GpuViews {
    fn seal(&mut self, function: &mut Function<OwnedConfig>) -> Result<(), gpu::cold::ViewError> {
        self.check(function)
    }
}

impl GpuViews {
    fn check(&self, function: &mut Function<OwnedConfig>) -> Result<(), gpu::cold::ViewError> {
        use gpu::cold::ViewError;
        if !function
            .config()
            .matches_retention(&self.config)
            .map_err(|_| ViewError::Context)?
        {
            return Err(ViewError::ForeignSource);
        }
        if function
            .config()
            .read16(4)
            .map_err(|_| ViewError::RegisterReadback)?
            & 2
            == 0
            || gpu_pci_words(function).map_err(|_| ViewError::RegisterReadback)? != self.pci_words
            || self.registers.read32(0) != Ok(self.boot0)
        {
            return Err(ViewError::RegisterReadback);
        }
        Ok(())
    }
}

fn select_firmware(boot0: u32, catalog: Arc<Catalog>) -> Result<PciFirmware, ServiceError> {
    use stela_loader::host_firmware::{bundle, recipes, requirements::Inventory};
    let chip = gpu::nouveau_chip::Chip::from_boot0(boot0).ok_or(ServiceError::Refused(
        "the GPU's PMC chipset is unsupported",
    ))?;
    let name = chip.name.to_ascii_lowercase();
    let mut inventory = Inventory::default();
    recipes::require_gsp(&mut inventory, &name)
        .map_err(|_| ServiceError::Refused("the GPU has no pinned GSP recipe"))?;
    let mut names = Vec::new();
    let request = inventory
        .requests()
        .first()
        .ok_or(ServiceError::Refused("the GPU has no pinned GSP recipe"))?;
    let target = FirmwareTarget::new(
        request.target.driver.clone(),
        request.target.hardware.clone(),
        request.target.revision.clone(),
        request.target.board.clone(),
    )
    .map_err(|_| ServiceError::Refused("the GPU firmware target is invalid"))?;
    let plan = recipes::Plan::select(&inventory)
        .map_err(|_| ServiceError::Refused("the GPU firmware recipe cannot be selected"))?;
    // SAFETY: the reference count is never given back, so the catalog
    // stays allocated for every image borrowed from it for the boot.
    let stored: &'static Catalog = unsafe { &*Arc::into_raw(catalog.clone()) };
    let mut bindings = Vec::new();
    // nouveau needs every file of the chip's recipe: one missing fails its
    // start later with a generic error.
    let mut missing = Vec::new();
    for selection in plan.selections() {
        let request = &selection.request;
        // An exact binding, else the bundle's file by its pinned digest.
        let found = match stored.load(request.view()) {
            Err(error) if error.kind == FirmwareError::Missing => {
                let name = bundle::name(selection.recipe.image.sha256);
                match stored.load(stela_drivers::firmware::Requirement {
                    name: &name,
                    target: bundle::TARGET,
                    minimum_bytes: request.minimum_bytes,
                }) {
                    Err(stored_error) if stored_error.kind == FirmwareError::Missing => Err(error),
                    other => other,
                }
            }
            other => other,
        };
        if let Ok(image) = found {
            bindings.push((image, request));
        }
        match found {
            Ok(image) => {
                selection
                    .recipe
                    .image
                    .verify_digest(image.bytes().len(), image.sha256())
                    .map_err(|_| {
                        ServiceError::Refused(
                            "the GPU firmware digest does not match its pinned recipe",
                        )
                    })?;
            }
            Err(error) if error.kind == FirmwareError::Missing => {
                missing.push(request.name.clone());
            }
            Err(error) => {
                output::arguments(format_args!(
                    "GPU {name}: firmware {} refused: {error:?}\n",
                    request.name
                ));
                return Err(ServiceError::Refused(
                    "the GPU firmware does not match its exact catalogue target",
                ));
            }
        }
        names.push(request.name.clone());
    }
    if !missing.is_empty() {
        output::arguments(format_args!(
            "GPU {name}: the USB's firmware bundle lacks {} of this chip's {} files ({}); \
             `stela repair` completes it\n",
            missing.len(),
            names.len(),
            missing.join(", ")
        ));
        return Err(ServiceError::Refused(NO_FIRMWARE));
    }
    // Each file under the exact name and target nouveau asks for.
    let catalog = Catalog::rebind_all(bindings.iter().map(|(image, request)| {
        let view = request.view();
        (*image, view.name, view.target)
    }))
    .map_err(|_| ServiceError::Refused("the GPU firmware cannot be bound to its requests"))?;
    PciFirmware::new(target, Arc::new(catalog), names.into_boxed_slice())
        .map_err(|_| ServiceError::Refused("the GPU firmware image names are invalid"))
}

/// A function claimed from probe, before Linux sees it.
pub(super) struct Claimed {
    device: Device,
    firmware: PciFirmware,
    placement: Placement,
    bars: Vec<Range<u64>>,
    name: String,
    #[cfg(feature = "nvk")]
    drm: client::DrmDevice,
    #[cfg(feature = "nvk")]
    link: Option<PcieLink>,
}

fn claim(
    services: &dyn NativeServices,
    adapter: &gpu::Adapter,
    line: u32,
    gop: crate::pci_io::ScreenOwner,
    catalog: Arc<Catalog>,
) -> Result<Claimed, ServiceError> {
    use crate::pci_io::ScreenOwner;
    let address = adapter.address;
    let timer = services.minimum_delay()?;
    let mut requester = services.prepare_requester(address)?;
    let bus_reset = requester.bus_reset_available();
    let function = requester.function_mut();
    if (function.vendor, function.device, function.class) != (0x10de, adapter.device, adapter.class)
    {
        return Err(ServiceError::Refused("the GPU's original identity changed"));
    }
    let translated = services.dma_translated(address);
    gpu::cold::check_candidate(function, timer.as_ref(), translated, bus_reset)
        .map_err(|error| cold_refusal(address, error))?;
    let owns_screen = match gop {
        ScreenOwner::Function(source) if source.address == address => {
            source.bar.words().try_for_each(|(offset, expected)| {
                let observed = function
                    .config()
                    .read32(offset)
                    .map_err(ServiceError::Config)?;
                if observed != expected {
                    return Err(ServiceError::Config(ConfigError::before_effect(
                        ConfigFailure::Readback {
                            offset,
                            width: 4,
                            expected,
                            observed,
                        },
                    )));
                }
                Ok(())
            })?;
            true
        }
        // Any GPU may show the screen, so none starts beside its writer.
        ScreenOwner::Unknown => true,
        ScreenOwner::Function(_) | ScreenOwner::NoScreen => false,
    };
    // BAR sizing below temporarily turns off memory decode.
    if owns_screen && !crate::console::retire_firmware() {
        return Err(ServiceError::Refused(
            "a firmware framebuffer writer is still active",
        ));
    }
    let mut placement = Placement {
        bus: address.bus,
        devfn: address.device << 3 | address.function,
        irq: line,
        dma_limit: DMA_LIMIT,
        ..Placement::default()
    };
    let (mut bars, mut resources) = (Vec::new(), ManuallyDrop::new(Vec::new()));
    let function_bars = function.bars().map_err(ServiceError::Config)?;
    if let Err(why) = gpu::cold::register_bar(function_bars[0], PAGE) {
        output::arguments(format_args!(
            "GPU {address} register BAR refused: {why:?} BAR0={:?}\n",
            function_bars[0]
        ));
        return Err(ServiceError::Refused(
            "the GPU's BAR 0 is not an assigned register window",
        ));
    }
    for (region, bar) in placement.regions.iter_mut().zip(function_bars) {
        let Some(
            bar @ Bar::Memory {
                address: start,
                size,
                prefetchable,
                wide,
            },
        ) = bar
        else {
            continue;
        };
        let resource = services.bar_resource(address, bar)?;
        let physical = resource.physical_span();
        if physical.start() != start || physical.bytes() != size {
            return Err(ServiceError::Refused(
                "the GPU BAR is translated by the host bridge",
            ));
        }
        resources.push(resource);
        bars.push(start..start + size);
        *region = Region {
            start,
            size,
            flags: RESOURCE_MEMORY
                | if prefetchable {
                    RESOURCE_PREFETCHABLE
                } else {
                    0
                }
                | if wide { RESOURCE_WIDE } else { 0 },
        };
    }
    // Each memory BAR lies in a host aperture: decode it, as Linux
    // `pci_enable_device` does before the register reads below.
    let decode = gpu::cold::enable_decode(function, timer.as_ref()).map_err(|error| {
        output::arguments(format_args!(
            "GPU {address} memory decoding refused: {error:?}\n"
        ));
        ServiceError::Refused("the GPU's memory decoding did not turn on")
    })?;
    if decode.power_state != 0 || decode.memory_was_off {
        output::arguments(format_args!(
            "GPU {address}: firmware left it in D{} with memory decoding {}; brought to D0 with decoding on, as Linux pci_enable_device does\n",
            decode.power_state,
            if decode.memory_was_off { "off" } else { "on" }
        ));
    }
    let config = function
        .config()
        .retain_epoch()
        .map_err(ServiceError::Mapping)?;
    let pci_words = gpu_pci_words(function).map_err(ServiceError::Config)?;
    let registers = services
        .authority()?
        .map(
            resources[0].clone(),
            0,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )
        .map_err(ServiceError::Mapping)?;
    let endian = registers.read32(4).map_err(ServiceError::Mapping)?;
    if endian == u32::MAX || endian == 0x01000001 {
        return Err(ServiceError::Refused(
            "the GPU register byte order is unavailable",
        ));
    }
    let boot0 = registers.read32(0).map_err(ServiceError::Mapping)?;
    let firmware = select_firmware(boot0, catalog)?;
    if let Some(chip) = gpu::nouveau_chip::Chip::from_boot0(boot0) {
        crate::host_inventory::text(alloc::format!("GPU{line}Chip"), chip.name);
    }
    crate::host_inventory::text(
        alloc::format!("GPU{line}PCIID"),
        alloc::format!("10de:{:04x}", adapter.device),
    );
    let mut cold = gpu::cold::Cold::new(gpu::cold::Inputs {
        requester,
        views: GpuViews {
            config,
            pci_words,
            registers,
            boot0,
        },
        timer,
    });
    if translated {
        // SAFETY: the mount's active remapper translates this requester, and
        // its private domain is the one the cold attempt binds.
        cold = unsafe { cold.translated() };
    }
    let ready = cold
        .quiesce_and_bind()
        .map_err(|failure| cold_refusal(address, failure.error))?;
    match ready.reset() {
        None => output::arguments(format_args!(
            "GPU {address}: no function-local reset; started as firmware left it, DMA translated\n"
        )),
        Some(gpu::cold::Reset::SecondaryBus) => output::arguments(format_args!(
            "GPU {address}: no function-local reset; reset through its bridge's secondary bus\n"
        )),
        Some(gpu::cold::Reset::Function(_)) => {}
    }
    // SAFETY: both original owners transfer into this retained function
    // lifetime before Linux probe. Only the temporary UC observation is retired.
    let (bound, views) = unsafe { ready.into_parts() };
    let attachment = ManuallyDrop::new(bound.attachment);
    let mut function = ManuallyDrop::new(bound.requester.into_function());
    let mut views = ManuallyDrop::new(views);
    views
        .check(&mut function)
        .map_err(|error| cold_refusal(address, gpu::cold::Error::Views(error)))?;
    if let Some(chip) =
        gpu::nouveau_chip::Chip::from_boot0(boot0).filter(|chip| chip.waits_for_firmware_boot())
    {
        wait_for_firmware_boot(services, &resources[0], chip)?;
    }
    // SAFETY: this scoped BAR observation was the sole user of the UC mapping.
    unsafe { views.registers.retire() }.map_err(ServiceError::Mapping)?;
    let capabilities = function
        .capabilities()
        .map_err(ServiceError::PciCapability)?;
    let msi = capabilities
        .iter()
        .find(|(_, id)| *id == MSI_CAPABILITY)
        .map(|(offset, _)| Msi::at(u16::from(*offset)));
    #[cfg(feature = "nvk")]
    let drm = {
        let config = function.config();
        let subsystem = config.read32(SUBSYSTEM_ID).map_err(ServiceError::Config)?;
        let revision = config.read8(REVISION).map_err(ServiceError::Config)?;
        client::DrmDevice {
            segment: address.segment,
            vendor: function.vendor,
            device: function.device,
            subsystem_vendor: subsystem as u16,
            subsystem_device: (subsystem >> 16) as u16,
            bus: address.bus,
            slot: address.device,
            function: address.function,
            revision,
        }
    };
    #[cfg(feature = "nvk")]
    let link = capabilities
        .iter()
        .find(|(_, id)| *id == PCIE_CAPABILITY)
        .map(|(offset, _)| {
            function
                .config()
                .read16(u16::from(*offset) + LINK_STATUS)
                .map(PcieLink::from_link_status)
        })
        .transpose()
        .map_err(ServiceError::Config)?;
    Ok(Claimed {
        device: Device {
            config: SharedConfig(Rc::new(RefCell::new(ManuallyDrop::into_inner(function)))),
            msi,
            custody: FunctionCustody::new(ManuallyDrop::into_inner(attachment)),
            _views: views,
            _resources: resources,
        },
        firmware,
        placement,
        bars,
        name: format!("{address}"),
        #[cfg(feature = "nvk")]
        drm,
        #[cfg(feature = "nvk")]
        link,
    })
}

/// Read one BAR0 register through a scoped read-only page of `registers`,
/// or `None` when it cannot be mapped or read.
fn read_register(
    services: &dyn NativeServices,
    registers: &owned::Resource,
    offset: usize,
) -> Option<u32> {
    let page = offset & !(PAGE as usize - 1);
    let mut window = services
        .authority()
        .ok()?
        .map(
            registers.clone(),
            page,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )
        .ok()?;
    let value = window.read32(offset - page).ok();
    // SAFETY: the read above was this scoped mapping's only use.
    unsafe { window.retire() }.ok()?;
    value
}

/// After the claim's function-level reset a Turing-to-Ada GPU's own firmware
/// runs devinit again. nouveau reads the VBIOS before it waits for that
/// (`firmware_boot`), so wait here first, by nouveau's bound, and print what
/// the start of the old boot flow depends on: the firmware boot, WPR2 (which
/// FWSEC-FRTS sets) and the VBIOS window the PRAMIN shadow reads.
/// `GPU AD107: firmware boot completed after N polls: gfw=… plm=… wpr2=…-…
/// display_fuse=… vbios_window=…`, or `did not complete` and a refusal.
fn wait_for_firmware_boot(
    services: &dyn NativeServices,
    registers: &owned::Resource,
    chip: &gpu::nouveau_chip::Chip,
) -> Result<(), ServiceError> {
    use gpu::nouveau_chip::firmware_boot::{self, Error as BootError};
    let page = firmware_boot::PRIV_LEVEL_MASK & !(PAGE as usize - 1);
    let mut window = services
        .authority()?
        .map(
            registers.clone(),
            page,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )
        .map_err(ServiceError::Mapping)?;
    let timer = services.minimum_delay()?;
    let result = firmware_boot::wait(
        |offset| {
            window
                .read32(offset - page)
                .map_err(|_| "a register read failed")
        },
        |micros| timer.wait_us(micros).map_err(|_| "the timer failed"),
    );
    // SAFETY: the polls above were this scoped mapping's only use.
    unsafe { window.retire() }.map_err(ServiceError::Mapping)?;
    let read = |offset| read_register(services, registers, offset);
    let hex =
        |value: Option<u32>| value.map_or_else(|| String::from("unread"), |v| format!("{v:#010x}"));
    // nouveau's PRAMIN shadow reads the window only with the display fused
    // on (`shadowramin.c` `pramin_init`: 0x820c04 from GA100, else 0x021c04).
    let fuse = read(match chip.architecture {
        gpu::nouveau_chip::Architecture::Turing => 0x02_1c04,
        _ => 0x82_0c04,
    });
    let window = fuse
        .filter(|fuse| fuse & 1 == 0)
        .and_then(|_| read(0x62_5f04));
    let state = format!(
        "wpr2={}-{} display_fuse={} vbios_window={}",
        hex(read(0x1f_a824)),
        hex(read(0x1f_a828)),
        hex(fuse),
        hex(window),
    );
    match result {
        Ok(reading) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot completed after {} polls: gfw={:#010x} plm={:#010x} {state}\n",
                chip.name, reading.polls, reading.progress, reading.priv_level_mask
            ));
            Ok(())
        }
        Err(BootError::Timeout(reading)) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot did not complete after its reset in {} polls of {} us: \
                 gfw={:#010x} (expected low byte {:#04x}) plm={:#010x} (expected bit 0) {state}\n",
                chip.name,
                reading.polls,
                firmware_boot::POLL_MICROS,
                reading.progress,
                firmware_boot::COMPLETED,
                reading.priv_level_mask
            ));
            Err(ServiceError::Refused(
                "the GPU's firmware boot did not complete after its reset",
            ))
        }
        Err(BootError::Access(error)) => {
            output::arguments(format_args!(
                "GPU {}: firmware boot wait stopped: {error}\n",
                chip.name
            ));
            Err(ServiceError::Refused(
                "the GPU's firmware boot registers could not be read",
            ))
        }
    }
}

fn cold_refusal(
    address: stela_drivers::bus::pci::Address,
    error: gpu::cold::Error,
) -> ServiceError {
    output::arguments(format_args!("GPU {address} cold preparation: {error:?}\n"));
    ServiceError::Refused("the GPU's original cold handoff was refused")
}

/// Why a GPU was refused when the USB lacks any of its chip's firmware files.
const NO_FIRMWARE: &str = "no pinned firmware for the observed GPU chipset was loaded";

/// An NVIDIA adapter [`claim_all`] did not claim, for the boot screen
/// (`stela_boot_core::gpu_status`).
pub(super) struct Refused {
    pub(super) address: String,
    pub(super) device: u16,
    pub(super) reason: stela_boot_core::gpu_status::Reason,
}

/// Claim every NVIDIA adapter, each reported if it cannot be, and the ones
/// refused. In a foreground continuation, with probe finished.
pub(super) fn claim_all(
    services: &dyn NativeServices,
    adapters: &[gpu::Adapter],
    gop: Result<crate::pci_io::ScreenOwner, String>,
    catalog: Option<&Arc<Catalog>>,
) -> (Vec<Claimed>, Vec<Refused>) {
    use stela_boot_core::gpu_status::Reason;
    let refused_as = |adapter: &gpu::Adapter, reason| Refused {
        address: format!("{}", adapter.address),
        device: adapter.device,
        reason,
    };
    for adapter in adapters
        .iter()
        .filter(|adapter| adapter.vendor != gpu::Vendor::Nvidia)
    {
        output::arguments(format_args!(
            "GPU {} vendor={:?} device={:04x} unused: {}\n",
            adapter.address,
            adapter.vendor,
            adapter.device,
            adapter.unavailable_reason()
        ));
    }
    let nvidia = adapters
        .iter()
        .filter(|adapter| adapter.vendor == gpu::Vendor::Nvidia);
    if !cfg!(feature = "nvk") {
        let mut refused = Vec::new();
        for adapter in nvidia {
            output::arguments(format_args!(
                "GPU {} left with firmware: build the image with `cargo xtask boot build`\n",
                adapter.address
            ));
            refused.push(refused_as(adapter, Reason::Other));
        }
        return (Vec::new(), refused);
    }
    let mut claimed = Vec::new();
    let mut refused = Vec::new();
    let source = match gop {
        Ok(source) => source,
        Err(why) => {
            for adapter in nvidia {
                output::arguments(format_args!("GPU {} refused: {why}\n", adapter.address));
                refused.push(refused_as(adapter, Reason::Other));
            }
            return (claimed, refused);
        }
    };
    for adapter in nvidia {
        let result = catalog
            .ok_or(ServiceError::Refused("no approved GPU firmware catalogue"))
            .and_then(|catalog| {
                claim(
                    services,
                    adapter,
                    claimed.len() as u32,
                    source,
                    catalog.clone(),
                )
            });
        match result {
            Ok(function) => claimed.push(function),
            Err(error) => {
                output::arguments(format_args!("GPU {} refused: {error:?}\n", adapter.address));
                let reason = match error {
                    ServiceError::Refused(NO_FIRMWARE) => Reason::MissingFirmware,
                    _ => Reason::Other,
                };
                refused.push(refused_as(adapter, reason));
            }
        }
    }
    (claimed, refused)
}

/// A GPU nouveau bound.
#[cfg(feature = "nvk")]
struct Bound {
    name: String,
    drm: client::DrmDevice,
    link: Option<PcieLink>,
}

/// The original managed child runs the driver's initcalls and probe.
struct Start {
    kernel: &'static LinuxKernel,
    claimed: ManuallyDrop<Vec<Claimed>>,
    completed: Rc<Cell<Option<ManuallyDrop<Started>>>>,
}

struct Started {
    _attached: Vec<Attached<Device>>,
    #[cfg(feature = "nvk")]
    bound: Vec<Bound>,
    result: Result<(), Errno>,
}

// SAFETY: the original managed runtime enters the canonical owner on the
// child before run. The job retains the actual devices until it publishes
// the result, whose attachments remain owned for the boot.
unsafe impl ManagedJob for Start {
    fn run(mut self: Pin<&mut Self>) {
        let claimed = core::mem::take(&mut *self.claimed);
        self.completed
            .set(Some(ManuallyDrop::new(start(self.kernel, claimed))));
    }
}

fn start(kernel: &LinuxKernel, claimed: Vec<Claimed>) -> Started {
    #[cfg(feature = "nvk")]
    stela_boot_core::heap::set_peak_hook(gate::peak_passed);
    let mut job = Started {
        _attached: Vec::new(),
        #[cfg(feature = "nvk")]
        bound: Vec::new(),
        result: Ok(()),
    };
    // SAFETY: entered, on a task.
    job.result = unsafe { gpu::nouveau::start() };
    if job.result.is_err() {
        // No source-specific device return was acknowledged.
        core::mem::forget(claimed);
        return job;
    }
    for claimed in claimed {
        let authority = {
            let mut function = claimed.device.config.0.borrow_mut();
            // SAFETY: the firmware keys came from the real chipset read on the
            // exact register BAR and were rechecked after its consumed reset.
            // Device and catalogue share the original native attachment.
            unsafe {
                PciAuthority::new(
                    kernel,
                    claimed.device.custody.clone(),
                    function.config(),
                    claimed.firmware,
                )
            }
        };
        let authority = match authority {
            Ok(authority) => authority,
            Err(error) => {
                output::arguments(format_args!(
                    "GPU {} catalogue refused: {error}\n",
                    claimed.name
                ));
                core::mem::forget(claimed.device);
                continue;
            }
        };
        // SAFETY: entered on the same canonical kernel task. The device and
        // enrolled catalogue retain the exact original configuration/attachment.
        let attached = unsafe {
            Attached::new_catalogued(kernel, claimed.device, claimed.placement, authority)
        };
        match attached {
            Ok(attached) => {
                // SAFETY: entered.
                if unsafe { attached.bound() } {
                    #[cfg(feature = "nvk")]
                    job.bound.push(Bound {
                        name: claimed.name,
                        drm: claimed.drm,
                        link: claimed.link,
                    });
                } else {
                    output::arguments(format_args!(
                        "GPU {}: nouveau did not bind it\n",
                        claimed.name
                    ));
                }
                job._attached.push(attached);
            }
            Err(error) => output::arguments(format_args!(
                "GPU {} presentation refused: {:?}\n",
                claimed.name, error.error
            )),
        }
    }
    job
}

/// The running environment.
#[cfg_attr(
    not(feature = "nvk"),
    allow(
        dead_code,
        reason = "held for the boot; only Mesa's client reads these"
    )
)]
struct Installed {
    work: &'static KernelWork,
    time: Arc<dyn Clock>,
    events: Arc<Events>,
    domain: Arc<Domain<SLOTS>>,
    /// When the service is next due, as a TSC value: the clock reads only
    /// in the Linux environment's root, and the boot loop asks outside it.
    next: AtomicU64,
    #[cfg(feature = "nvk")]
    window: &'static Window,
    tsc_hz: u64,
    #[cfg(feature = "nvk")]
    libc_clock: client::LibcClock,
    #[cfg(feature = "nvk")]
    heap_bytes: u64,
    /// Clients' mappings with their sizes in pages, by address. For tasks.
    #[cfg(feature = "nvk")]
    mappings: RefCell<BTreeMap<u64, (stela_linux::client::Mapping, usize)>>,
    /// Mesa's running threads, by the bottom of their stacks. On the boot CPU.
    #[cfg(feature = "nvk")]
    mesa_threads: RefCell<BTreeMap<u64, client::Running>>,
    #[cfg(feature = "nvk")]
    service: client::Service,
    owner: &'static Arc<LinuxKernel>,
    threads: &'static LinuxThreads,
    start: ManuallyDrop<Started>,
}

impl Installed {
    fn advance_children(&self) {
        // SAFETY: callers are on the same dispatcher after physical return,
        // outside native/source stacks and metadata loans.
        if let Err(error) = unsafe { self.threads.dispatcher_step() } {
            output::fatal(format_args!(
                "Linux original task retirement failed: {error}"
            ));
        }
    }
}

static INSTALLED: AtomicPtr<Installed> = AtomicPtr::new(ptr::null_mut());

fn installed() -> Option<&'static Installed> {
    // SAFETY: a published environment is leaked, so it lives for the boot.
    unsafe { INSTALLED.load(Ordering::Acquire).as_ref() }
}

/// What [`open`] needs beyond the claimed functions.
pub(super) struct Machine {
    pub work: &'static KernelWork,
    pub window: &'static Window,
    pub random: Seeded,
    pub tsc_hz: u64,
    pub wall: Option<RtcReading>,
    /// The host heap capacity reported to the linked graphics runtime.
    #[cfg(feature = "nvk")]
    pub heap_bytes: u64,
}

fn environment_error(error: WorkError) -> kernel_work::Error {
    kernel_work::Error::Work(error)
}

/// Why this boot gives macOS no Metal device, if it does not. WindowServer
/// aborts without one and watchdogd
/// then panics the system, so a boot of macOS stops here instead.
pub(super) fn no_metal_device() -> Option<&'static str> {
    #[cfg(feature = "nvk")]
    return gate::shared()
        .is_empty()
        .then_some("no GPU started; the lines above say why");
    #[cfg(not(feature = "nvk"))]
    Some("this stela.efi was built without GPU drivers (the `nvk` feature)")
}

/// The kexts a boot that gives the GPUs to nouveau links into its system.
pub(super) fn kexts() -> Vec<stela_loader::aux_kc::Kext<'static>> {
    #[cfg(feature = "nvk")]
    return alloc::vec![stela_loader::aux_kc::Kext {
        info: kext::INFO,
        objects: kext::OBJECTS.to_vec(),
        bundle_path: "/Library/Extensions/StelaGPU.kext".into(),
        trusted_code: kext::TRUSTED_CODE.to_vec(),
    }];
    #[cfg(not(feature = "nvk"))]
    Vec::new()
}

/// Takes the display for the GPUs [`open`] bound, in the monitor's own mode
/// or else the firmware's `screen` size, and opens their Metal services. `firmware` names the PCI
/// function that showed the firmware's picture, `None` when there was a
/// picture whose function is unknown.
pub(super) fn serve(
    #[cfg_attr(not(feature = "nvk"), allow(unused_variables))] screen: Option<(usize, usize)>,
    #[cfg_attr(not(feature = "nvk"), allow(unused_variables))] firmware: Option<&str>,
) {
    #[cfg(feature = "nvk")]
    {
        let shown = match (screen, firmware) {
            (None, _) => display::Shown::Nothing,
            (Some(_), None) => display::Shown::Unknown,
            (Some(_), Some(function)) => display::Shown::By(function),
        };
        display::take(screen.unwrap_or((0, 0)), shown);
        gate::open_service();
    }
}

/// Bring Linux up over `claimed`, and return once nouveau has bound them or
/// failed, as the guest should not start with a half-started GPU beneath it.
pub(super) fn open(machine: Machine, claimed: Vec<Claimed>) -> Result<(), Error> {
    // Keep the exact original claims on every partial startup refusal.
    let claimed = ManuallyDrop::new(claimed);
    let Machine {
        work,
        window,
        random,
        tsc_hz,
        wall,
        #[cfg(feature = "nvk")]
        heap_bytes,
    } = machine;
    let image = work.image()?;
    let task_data = work.task_data()?;
    #[cfg(feature = "nvk")]
    let claiming = !claimed.is_empty();
    let lines = claimed
        .iter()
        .map(|claimed| Line {
            config: claimed.device.config.clone(),
            msi: claimed.device.msi,
            custody: claimed.device.custody.clone(),
            registration: None,
            retired: None,
        })
        .collect();
    let bars = claimed
        .iter()
        .flat_map(|claimed| claimed.bars.iter().cloned())
        .collect();
    let clock = work.service_clock().map_err(environment_error)?;
    let epoch = clock.epoch().map_err(environment_error)?;
    #[cfg(feature = "nvk")]
    let libc_clock = client::LibcClock::anchor(|| epoch.now_ns()).map_err(environment_error)?;
    let time = clock.clock();
    let events = Arc::new(Events {
        clock: clock.clone(),
        idle: clock.idle(),
        pending: AtomicBool::new(false),
    });
    // Linux's service runs for raised work and its earliest timer only:
    // nothing here consumes a periodic tick, and one would pump the boot
    // CPU every millisecond while the GPU idles.
    let domain = Domain::tickless(time.clone(), events.clone()).map_err(environment_error)?;
    let realtime = wall.and_then(|reading| {
        if tsc_hz == 0
            || !crate::cpu::INVARIANT_TSC.present()
            || reading.resolution == 0
            || reading.tsc_before > reading.tsc_after
        {
            return None;
        }
        // Move the firmware wall sample to a fresh observation in the
        // installed monotonic epoch, using the boot CPU's invariant TSC
        // only to carry UTC forward from the firmware read.
        let monotonic_ns = i64::try_from(epoch.now_ns().ok()?).ok()?;
        let elapsed = host::rdtsc().checked_sub(reading.tsc_after)?;
        let wall_ns = u128::from(elapsed) * 1_000_000_000 / u128::from(tsc_hz)
            + u128::from(reading.utc.nanosecond);
        let seconds = reading
            .utc
            .seconds
            .checked_add(u64::try_from(wall_ns / 1_000_000_000).ok()?)?;
        RealtimeOrigin::new(
            monotonic_ns,
            i64::try_from(seconds).ok()?,
            (wall_ns % 1_000_000_000) as u32,
        )
    });
    let platform = Host {
        work,
        task_data,
        tsc_hz,
        clock: epoch,
        realtime,
        window,
        image: image.clone(),
        random: RefCell::new(random),
        environment: Cell::new(ptr::null()),
        bars,
        devices: RefCell::new(BTreeMap::new()),
        lines: RefCell::new(lines),
        reported: RefCell::new(BTreeSet::new()),
    };
    // SAFETY: boot's original code and allocator remain installed permanently.
    unsafe {
        stela_linux::image::register(image).map_err(|_| kernel_work::Error::Inventory)?;
        stela_linux::heap::register(allocate_zeroed).map_err(|_| kernel_work::Error::Inventory)?;
    }
    #[cfg(feature = "nvk")]
    BOOT_CPU.store(host::processor(), Ordering::Relaxed);
    let (owner, threads, job) = work.dispatch_linux(|runtime| {
        Ok(mount_and_start(
            work,
            runtime,
            platform,
            domain.clone(),
            claimed,
            time.as_ref(),
        ))
    })??;
    let result = job.result.map_err(Error::Init);
    #[cfg(feature = "nvk")]
    let result = result.and_then(|()| {
        if claiming && job.bound.is_empty() {
            Err(Error::NoDevices)
        } else {
            Ok(())
        }
    });
    let installed = Box::new(Installed {
        work,
        time,
        events,
        domain,
        // Due at once: the first pump finds Linux's own deadline.
        next: AtomicU64::new(0),
        #[cfg(feature = "nvk")]
        window,
        tsc_hz,
        #[cfg(feature = "nvk")]
        libc_clock,
        #[cfg(feature = "nvk")]
        heap_bytes,
        #[cfg(feature = "nvk")]
        mappings: RefCell::new(BTreeMap::new()),
        #[cfg(feature = "nvk")]
        mesa_threads: RefCell::new(BTreeMap::new()),
        #[cfg(feature = "nvk")]
        service: client::Service::default(),
        owner,
        threads,
        start: job,
    });
    INSTALLED.store(Box::into_raw(installed), Ordering::Release);
    result
}

/// TSC ticks [`open`] took to start Linux and for nouveau's probe.
static START_TICKS: [AtomicU64; 2] = [const { AtomicU64::new(0) }; 2];

/// The ticks the last [`open`] took starting Linux and probing the GPUs.
pub(super) fn start_times() -> (u64, u64) {
    (
        START_TICKS[0].load(Ordering::Relaxed),
        START_TICKS[1].load(Ordering::Relaxed),
    )
}

type MountedStartup = (
    &'static Arc<LinuxKernel>,
    &'static LinuxThreads,
    ManuallyDrop<Started>,
);

fn mount_and_start(
    work: &'static KernelWork,
    runtime: Pin<&'static stela_drivers::platform::work::Runtime<SLOTS>>,
    platform: Host,
    domain: Arc<Domain<SLOTS>>,
    claimed: ManuallyDrop<Vec<Claimed>>,
    time: &dyn Clock,
) -> Result<MountedStartup, Error> {
    let deadline = time
        .now_ns()
        .and_then(|now| {
            now.checked_add(START_SECONDS * 1_000_000_000)
                .ok_or(WorkError::Exhausted)
        })
        .map_err(environment_error)?;
    let started = host::rdtsc();
    // SAFETY: the enclosing dispatch owns this exact root/CPU/clock and
    // queue. The original heap, executable backing, platform and catalogue
    // stay retained permanently, including any partial kernel publication.
    let kernel =
        unsafe { KernelOwner::new(platform, runtime, domain.clone(), work.linux_geometry()) }
            .map_err(|failure| Error::Kernel(failure.error))?;
    #[expect(
        clippy::arc_with_non_send_sync,
        reason = "the canonical broker retains this CPU-local kernel through Arc"
    )]
    let owner = &*Box::leak(Box::new(Arc::new(kernel)));
    owner
        .platform()
        .environment
        .set(addr_of!(*owner.environment()));
    // SAFETY: this same native dispatcher owns the already claimed kernel
    // and no task has been published. The catalogue must retain this owner.
    let pages = unsafe { work.register_linux_pages(owner.as_ref()) }?;
    // SAFETY: the permanent, same-owner runtime, catalogue and service
    // domain were installed above. No original child has started yet.
    let threads = unsafe { TaskRuntime::install(owner.clone(), runtime, pages, SLOTS) }
        .map_err(Error::Threads)?;
    let threads = &*Box::leak(Box::new(threads));
    // Reserve the error envelope before Initial can publish native state.
    let initial_failure = Box::new_uninit();
    // SAFETY: this exact source, root and broker remain kept on refusal. Each
    // runtime step runs inside the same live native execution segment.
    unsafe { Pin::new(owner.as_ref()).start_initial(work.linux_root(), threads) }
        .map_err(|error| Error::Initial(Box::write(initial_failure, error)))?;

    // SAFETY: submit via this actual original task broker. The service
    // initializes the original resources before dispatching accepted jobs.
    let srcu = unsafe { stela_linux::srcu::Bootstrap::start(owner.clone(), threads) }
        .map_err(|failure| Error::SrcuStart(failure.error))?;
    wait_for_child(
        runtime,
        threads,
        time,
        deadline,
        "Linux's first tasks",
        || matches!(srcu.status(), stela_linux::srcu::Status::Pending(status) if status.joined),
    )?;
    // SAFETY: the taskless dispatcher observed the original joined job.
    // Only this consuming join can acknowledge public SRCU admission.
    let ready = unsafe { srcu.join() }.map_err(|failure| Error::Srcu(failure.error))?;
    let probing = host::rdtsc();
    START_TICKS[0].store(probing.wrapping_sub(started), Ordering::Relaxed);
    let completed = Rc::new(Cell::new(None));
    let job = Box::pin(Start {
        kernel: owner.as_ref(),
        claimed,
        completed: completed.clone(),
    });
    // SAFETY: the ready original task broker keeps the callback and all
    // original owners until its genuine managed child has joined.
    let pending = unsafe { threads.submit(job) }.map_err(|refusal| Error::Submit(refusal.error))?;
    wait_for_child(runtime, threads, time, deadline, "the GPU's probe", || {
        pending.status().joined
    })?;
    // SAFETY: physical return on the same dispatcher with the joined status.
    let joined =
        unsafe { pending.try_join() }.map_err(|_| environment_error(WorkError::Pending))?;
    START_TICKS[1].store(host::rdtsc().wrapping_sub(probing), Ordering::Relaxed);
    if joined.outcome() != Outcome::CallbackReturned {
        return Err(Error::Child(joined.outcome()));
    }
    let job = completed
        .take()
        .ok_or_else(|| environment_error(WorkError::corrupt()))?;
    drop(joined.into_job());
    drop(ready);
    Ok((owner, threads, job))
}

/// Wait for `stage` of the start, which Linux's tasks run, until `finished`
/// or `deadline` on `time`. A long wait says every
/// [`WAIT_REPORT_SECONDS`] how long it has waited by the counter and by
/// Linux's clock, and how many service steps it took, and stores the USB's
/// log: a start that never returns then shows whether this loop still
/// runs, whether Linux's clock moves, and which stage it waits for. The
/// counter ends the wait after [`START_SECONDS`] even when the clock
/// stands still.
fn wait_for_child(
    runtime: Pin<&stela_drivers::platform::work::Runtime<SLOTS>>,
    threads: &LinuxThreads,
    time: &dyn Clock,
    deadline: u64,
    stage: &str,
    finished: impl Fn() -> bool,
) -> Result<(), Error> {
    let began = host::rdtsc();
    let began_ns = time.now_ns().map_err(environment_error)?;
    let mut steps = 0u64;
    let mut reports = 0u64;
    loop {
        // SAFETY: runtime is on its dispatcher after real physical return.
        unsafe { threads.dispatcher_step() }.map_err(Error::Threads)?;
        if finished() {
            return Ok(());
        }
        let now = time.now_ns().map_err(environment_error)?;
        let waited_ms = output::millis_since(began);
        // The counter bounds the wait too, when Linux's clock stands still.
        if now >= deadline || waited_ms >= START_SECONDS * 1000 {
            return Err(Error::Timeout);
        }
        if waited_ms / 1000 / WAIT_REPORT_SECONDS > reports {
            reports = waited_ms / 1000 / WAIT_REPORT_SECONDS;
            output::arguments(format_args!(
                "GPU START waiting for {stage}: {waited_ms} ms by the counter, {} ms by \
                 Linux's clock, {steps} steps\n",
                now.saturating_sub(began_ns) / 1_000_000,
            ));
            super::storage::store_usb_log_in_gpu_start();
        }
        runtime.step_or_idle().map_err(environment_error)?;
        steps += 1;
    }
}

/// Startup failures distinguish the work environment from driver admission.
#[derive(Debug)]
pub(super) enum Error {
    Environment(kernel_work::Error),
    Kernel(stela_linux::kernel::Error),
    Threads(stela_linux::task::runtime::RuntimeError),
    Initial(Box<stela_linux::kernel::InitialStartError>),
    SrcuStart(stela_linux::srcu::StartError),
    Srcu(stela_linux::srcu::Error),
    Submit(stela_linux::task::runtime::SubmitError),
    Child(Outcome),
    Init(Errno),
    Timeout,
    #[cfg(feature = "nvk")]
    NoDevices,
}

impl From<kernel_work::Error> for Error {
    fn from(error: kernel_work::Error) -> Self {
        Self::Environment(error)
    }
}

impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Environment(error) => write!(f, "Linux environment: {error}"),
            Self::Kernel(error) => write!(f, "{error}"),
            Self::Threads(error) => write!(f, "{error}"),
            Self::Initial(error) => write!(f, "{error}"),
            Self::SrcuStart(error) => write!(f, "{error}"),
            Self::Srcu(error) => write!(f, "{error}"),
            Self::Submit(error) => write!(f, "GPU startup job refused: {error:?}"),
            Self::Child(outcome) => write!(f, "GPU startup child ended: {outcome:?}"),
            Self::Init(error) => write!(f, "Linux initialization failed: errno {}", -error),
            Self::Timeout => {
                f.write_str("GPU startup exceeded its deadline; native owners remain held")
            }
            #[cfg(feature = "nvk")]
            Self::NoDevices => f.write_str("nouveau bound no NVIDIA GPUs"),
        }
    }
}

impl Installed {
    /// The TSC ticks in `ns` nanoseconds.
    fn tsc_after(&self, ns: u64) -> u64 {
        let ticks = u128::from(ns) * u128::from(self.tsc_hz) / 1_000_000_000;
        ticks.min(u128::from(u64::MAX)) as u64
    }
}

/// When the environment's pending events were raised, as a TSC value: the
/// age the boot PE's run loop bounds ([`pump_due`]).
static PENDING_SINCE: AtomicU64 = AtomicU64::new(0);

/// How long the environment's work may wait while the boot PE runs guest
/// code at EL1 or EL2 with interrupts unmasked: it may hold a lock that
/// disables only preemption, which other PEs spin on while it pumps.
const DEFER_UNMASKED_NS: u64 = 1_000_000;
/// The longest the work waits whatever the boot PE runs, interrupts masked
/// included, where XNU holds its spinlocks: GPU completions and the calls
/// other PEs posted are never later than this.
const DEFER_LONGEST_NS: u64 = 4_000_000;
/// How often a slice of a boot PE that holds work back ends to look again.
const RECHECK_NS: u64 = 250_000;

/// What the boot PE runs at a slice's end, which decides how long the
/// environment's ready work waits for it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub(super) enum Calm {
    /// Waiting for an interrupt or at EL0: it holds no guest lock.
    Quiet,
    /// EL1 or EL2 with IRQs unmasked.
    Unmasked,
    /// IRQs masked, or a guarded level: likely inside a spinlock.
    Masked,
}

/// Why pumps ran, and what they took, since the last [`pump_report`].
struct PumpCounts {
    /// Pumps that ran the environment, by the boot PE's [`Calm`].
    quiet: AtomicU64,
    unmasked: AtomicU64,
    forced: AtomicU64,
    /// Pumps from a wait for the service's turn ([`gate::relax`]).
    relaxed: AtomicU64,
    /// Pumps for a posted call from the host service ([`pump_posted`]).
    posted: AtomicU64,
    /// Pumps for raised events, and for Linux's deadline alone.
    by_event: AtomicU64,
    by_deadline: AtomicU64,
    /// Pumps whose first step found nothing to run.
    idle: AtomicU64,
    /// Slice ends that held ready work back for a calmer boot PE.
    deferred: AtomicU64,
    /// Ticks pumping, and the longest pump.
    ticks: AtomicU64,
    longest: AtomicU64,
    /// Ticks from work being ready until a pump took it, and the longest.
    delay: AtomicU64,
    delay_longest: AtomicU64,
    /// The longest single step, which no budget ends: one Linux task run
    /// until it sleeps. With the host's mapping shootdowns during it and the
    /// posted calls (request code plus one) workers were performing as it
    /// began.
    step_longest: AtomicU64,
    step_shootdowns: AtomicU64,
    step_serving: [AtomicU32; 4],
    /// Linux steps the pumps ran.
    steps: AtomicU64,
    /// Ticks the pumps spent by part ([`PumpPart`]), and the longest one
    /// part took in a pump: what a long pump spent its time in.
    parts: [AtomicU64; PumpPart::COUNT],
    part_longest: [AtomicU64; PumpPart::COUNT],
}

/// The parts of a pump, timed for [`pump_report`].
#[derive(Clone, Copy)]
enum PumpPart {
    /// Entering the environment: the host drivers' lock, the root switch
    /// and the identity checks, until the first step.
    Enter,
    /// Retiring tasks and taking posted calls ([`client::start`]).
    Children,
    /// Linux's steps.
    Steps,
    /// Linux's next deadline, and leaving the environment.
    Leave,
    /// Looking for completed work ([`look_due`]).
    Look,
}

impl PumpPart {
    const COUNT: usize = 5;
    const NAMES: [&str; Self::COUNT] = ["enter", "children", "steps", "leave", "look"];

    fn add(self, ticks: u64) {
        PUMPS.parts[self as usize].fetch_add(ticks, Ordering::Relaxed);
        PUMPS.part_longest[self as usize].fetch_max(ticks, Ordering::Relaxed);
    }
}

/// Host page-window unmappings by their source since [`pump_report`]: Linux
/// device memory (`iounmap`), Linux pages (`vfree`, `vunmap`), the C
/// library's file mappings (`munmap`, fixed remaps) and task stacks. Each
/// costs one shootdown of every other processor.
static UNMAPS: [AtomicU64; 4] = [const { AtomicU64::new(0) }; 4];

/// The [`UNMAPS`] sources.
#[derive(Clone, Copy)]
pub(super) enum Unmapped {
    Device,
    Pages,
    #[cfg_attr(not(feature = "nvk"), allow(dead_code))]
    Files,
    #[cfg_attr(not(feature = "nvk"), allow(dead_code))]
    Stacks,
}

/// Counts one page-window unmapping from `source`.
pub(super) fn unmapped(source: Unmapped) {
    UNMAPS[source as usize].fetch_add(1, Ordering::Relaxed);
}

static PUMPS: PumpCounts = PumpCounts {
    quiet: AtomicU64::new(0),
    unmasked: AtomicU64::new(0),
    forced: AtomicU64::new(0),
    relaxed: AtomicU64::new(0),
    posted: AtomicU64::new(0),
    by_event: AtomicU64::new(0),
    by_deadline: AtomicU64::new(0),
    idle: AtomicU64::new(0),
    deferred: AtomicU64::new(0),
    ticks: AtomicU64::new(0),
    longest: AtomicU64::new(0),
    delay: AtomicU64::new(0),
    delay_longest: AtomicU64::new(0),
    step_longest: AtomicU64::new(0),
    step_shootdowns: AtomicU64::new(0),
    step_serving: [const { AtomicU32::new(0) }; 4],
    steps: AtomicU64::new(0),
    parts: [const { AtomicU64::new(0) }; PumpPart::COUNT],
    part_longest: [const { AtomicU64::new(0) }; PumpPart::COUNT],
};

/// Ticks the boot CPU spent in posted pumps ([`pump_posted`]).
#[cfg(feature = "nvk")]
static POSTED_TICKS: AtomicU64 = AtomicU64::new(0);

/// The ticks this processor spent in posted pumps so far: the boot CPU's
/// total, and nothing on another processor, which runs none.
pub(super) fn posted_ticks_here() -> u64 {
    #[cfg(feature = "nvk")]
    if host::processor() == BOOT_CPU.load(Ordering::Relaxed) {
        return POSTED_TICKS.load(Ordering::Relaxed);
    }
    0
}

/// The shootdowns counted so far, for a difference across a step.
fn shootdowns() -> u64 {
    #[cfg(feature = "nvk")]
    return crate::native_mmio::shootdown::count();
    #[cfg(not(feature = "nvk"))]
    0
}

/// Notes a step of `ticks` that began with `serving` and overlapped
/// `shot` shootdowns, when it is the longest since the last report.
fn note_step(ticks: u64, shot: u64, serving: [u32; 4]) {
    if PUMPS.step_longest.fetch_max(ticks, Ordering::Relaxed) < ticks {
        PUMPS.step_shootdowns.store(shot, Ordering::Relaxed);
        for (into, code) in PUMPS.step_serving.iter().zip(serving) {
            into.store(code, Ordering::Relaxed);
        }
    }
}

/// Since when the environment has had work ready, as a TSC value: events
/// raised or its deadline passed.
fn ready_since(gpu: &Installed, now: u64) -> Option<u64> {
    let raised = gpu.events.pending.load(Ordering::Acquire).then(|| {
        // An event raised as a pump cleared its time ages from here.
        match PENDING_SINCE.compare_exchange(0, now, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => now,
            Err(since) => since.min(now),
        }
    });
    let next = gpu.next.load(Ordering::Relaxed);
    let due = (now >= next).then_some(next);
    match (raised, due) {
        (Some(raised), Some(due)) => Some(raised.min(due)),
        (raised, due) => raised.or(due),
    }
}

/// Opens the boot CPU's interrupt window, which a GPU's MSI needs: compiled
/// code runs with interrupts off and a busy PE never waits. Taking one only
/// records it, so this is cheap whatever the guest holds.
pub(super) fn interrupt_window() {
    if installed().is_some() {
        native_irq::interrupt_window();
    }
}

/// Whether the run loop pumps now, the boot PE being `calm`: at once when it
/// holds no guest lock, and otherwise only once the ready work has waited
/// [`DEFER_UNMASKED_NS`] with interrupts unmasked or [`DEFER_LONGEST_NS`]
/// whatever it runs, so completions and posted calls are never starved.
pub(super) fn pump_due(calm: Calm) -> bool {
    let Some(gpu) = installed() else {
        return false;
    };
    let now = host::rdtsc();
    let Some(since) = ready_since(gpu, now) else {
        return false;
    };
    let waited = now.saturating_sub(since);
    let due = match calm {
        Calm::Quiet => true,
        Calm::Unmasked => waited >= gpu.tsc_after(DEFER_UNMASKED_NS),
        Calm::Masked => false,
    } || waited >= gpu.tsc_after(DEFER_LONGEST_NS);
    if !due {
        PUMPS.deferred.fetch_add(1, Ordering::Relaxed);
        return false;
    }
    let counter = match calm {
        _ if waited >= gpu.tsc_after(DEFER_LONGEST_NS) => &PUMPS.forced,
        Calm::Quiet => &PUMPS.quiet,
        Calm::Unmasked | Calm::Masked => &PUMPS.unmasked,
    };
    counter.fetch_add(1, Ordering::Relaxed);
    true
}

/// Run what is ready in the environment, on the boot CPU: between the boot
/// PE's slices once [`pump_due`], or while it waits for the service's turn.
pub(super) fn pump() {
    let Some(gpu) = installed() else {
        return;
    };
    native_irq::interrupt_window();
    let now = host::rdtsc();
    let Some(since) = ready_since(gpu, now) else {
        return;
    };
    // Cleared before the flag: an event raised between the two stores its
    // own time once the flag is clear.
    PENDING_SINCE.store(0, Ordering::Release);
    let raised = gpu.events.pending.swap(false, Ordering::AcqRel);
    let started = host::rdtsc();
    let delay = started.saturating_sub(since);
    PUMPS.delay.fetch_add(delay, Ordering::Relaxed);
    PUMPS.delay_longest.fetch_max(delay, Ordering::Relaxed);
    let cause = if raised {
        &PUMPS.by_event
    } else {
        &PUMPS.by_deadline
    };
    cause.fetch_add(1, Ordering::Relaxed);
    let budget = gpu.tsc_after(PUMP_NS);
    let mut steps = 0u32;
    let mut part_at = started;
    let mut lap = |part: PumpPart| {
        let now = host::rdtsc();
        part.add(now.wrapping_sub(part_at));
        part_at = now;
    };
    let ran = gpu.work.dispatch_linux(|runtime| {
        lap(PumpPart::Enter);
        let mut first = true;
        loop {
            // What is raised from here on this pass takes: the step runs
            // the tasks it woke, and `client::start` the calls posted. Only
            // what comes after stays pending for the next pump, which does
            // not then run for nothing.
            if !first {
                PENDING_SINCE.store(0, Ordering::Release);
                gpu.events.pending.store(false, Ordering::Release);
            }
            // Tasks retire and calls arrive only through steps and posts:
            // a pump's first pass looks only if something may have.
            #[cfg(feature = "nvk")]
            if !first || client::start_due() {
                gpu.advance_children();
                client::start(gpu)?;
            }
            #[cfg(not(feature = "nvk"))]
            gpu.advance_children();
            first = false;
            lap(PumpPart::Children);
            #[cfg(feature = "nvk")]
            let serving = client::serving();
            #[cfg(not(feature = "nvk"))]
            let serving = [0; 4];
            let (step_started, shot) = (host::rdtsc(), shootdowns());
            let stepped = runtime.step()?;
            lap(PumpPart::Steps);
            if !stepped {
                break;
            }
            let now_shot = shootdowns();
            note_step(
                host::rdtsc().wrapping_sub(step_started),
                now_shot.checked_sub(shot).unwrap_or(now_shot),
                serving,
            );
            steps += 1;
            // A core spinning for the host drivers' lock this pump holds
            // (a disk or input call) takes it after this step; the rest
            // stays pending for the next pump.
            if super::service::shutdown_pending()
                || host::rdtsc().wrapping_sub(started) >= budget
                || super::storage::host_drivers_wanted()
            {
                if !gpu.events.pending.swap(true, Ordering::AcqRel) {
                    PENDING_SINCE.store(host::rdtsc(), Ordering::Release);
                }
                break;
            }
        }
        let next = match gpu.domain.next_deadline()? {
            Some(next) => {
                let left = next.saturating_sub(gpu.time.now_ns()?);
                host::rdtsc().saturating_add(gpu.tsc_after(left))
            }
            None => u64::MAX,
        };
        gpu.next.store(next, Ordering::Relaxed);
        Ok(())
    });
    if let Err(error) = ran {
        output::fatal(format_args!("the host's Linux environment failed: {error}"));
    }
    lap(PumpPart::Leave);
    if steps == 0 {
        PUMPS.idle.fetch_add(1, Ordering::Relaxed);
    }
    PUMPS.steps.fetch_add(u64::from(steps), Ordering::Relaxed);
    #[cfg(feature = "nvk")]
    match look_due(gpu) {
        Look::Device => {
            gate::interrupted();
            gate::watch_completions();
        }
        Look::Waiters => gate::watch_completions(),
        Look::No => {}
    }
    lap(PumpPart::Look);
    // The pipelines the device added go to the store's file, queued a
    // piece at a time.
    #[cfg(feature = "nvk")]
    super::pipelines::step();
    let took = host::rdtsc().wrapping_sub(started);
    PUMPS.ticks.fetch_add(took, Ordering::Relaxed);
    PUMPS.longest.fetch_max(took, Ordering::Relaxed);
}

/// A pump for a posted call, from host work on the boot CPU that can run
/// for milliseconds between the run loop's pumps (the host service's device
/// polls): an AP waiting for a DRM call waits for the boot CPU, not for that
/// work. Nothing runs elsewhere, or with no call posted.
pub(super) fn pump_posted() {
    #[cfg(feature = "nvk")]
    if client::posted_waiting() && host::processor() == BOOT_CPU.load(Ordering::Relaxed) {
        PUMPS.posted.fetch_add(1, Ordering::Relaxed);
        let started = host::rdtsc();
        pump();
        POSTED_TICKS.fetch_add(host::rdtsc().wrapping_sub(started), Ordering::Relaxed);
    }
}

/// A pump from a wait for the service's turn on the boot CPU, which runs no
/// guest code meanwhile.
#[cfg(feature = "nvk")]
fn pump_waiting() {
    if installed().is_some_and(|gpu| ready_since(gpu, host::rdtsc()).is_some()) {
        PUMPS.relaxed.fetch_add(1, Ordering::Relaxed);
    }
    pump();
}

/// `GPU PUMP runs=… quiet=… unmasked=… forced=… relaxed=… posted=… by_event=…
/// by_deadline=… idle=… deferred=… mean_us=… longest_us=… delay_mean_us=…
/// delay_longest_us=… next_us=… next_timer=…`: the pumps since the last
/// report, by what let each run and why it was due, what they took, how long
/// ready work waited for one, and Linux's next deadline with the function
/// of its first timer-list timer, as an offset in this image. Then
/// `step_longest_us=… step_shootdowns=… step_serving=…`: the longest single
/// Linux step, the shootdowns during it and the posted calls being performed
/// as it began (`-` for none), and `unmaps=device/pages/files/stacks`
/// ([`UNMAPS`]), and `total_us=enter/children/steps/leave/look:…` the
/// pumps' time by [`PumpPart`], then `part_longest_us=…` the longest each
/// part took in one pump. Then `\nGPU LINUX timers=… works=… sleepers=… timeouts=…
/// steps=…` ([`linux_report`]).
pub(super) fn pump_report(tsc_hz: u64) -> Option<String> {
    let gpu = installed()?;
    let take = |counter: &AtomicU64| counter.swap(0, Ordering::Relaxed);
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let (by_event, by_deadline) = (take(&PUMPS.by_event), take(&PUMPS.by_deadline));
    let runs = by_event + by_deadline;
    let (ticks, delay) = (take(&PUMPS.ticks), take(&PUMPS.delay));
    let next = gpu.next.load(Ordering::Relaxed);
    let next_us = match next {
        u64::MAX => -1,
        next => micros(next.saturating_sub(host::rdtsc())) as i128,
    };
    let mut serving = String::new();
    for code in &PUMPS.step_serving {
        let code = code.swap(0, Ordering::Relaxed);
        if code != 0 {
            if !serving.is_empty() {
                serving.push('+');
            }
            serving.push_str(&format!("{:#x}", code.wrapping_sub(1)));
        }
    }
    if serving.is_empty() {
        serving.push('-');
    }
    let [device, pages, files, stacks] = UNMAPS.each_ref().map(take);
    #[cfg(feature = "nvk")]
    let nvkm = nvkm_report();
    #[cfg(not(feature = "nvk"))]
    let nvkm = String::new();
    let pumps = format!(
        "GPU PUMP runs={runs} quiet={} unmasked={} forced={} relaxed={} posted={} by_event={by_event} \
         by_deadline={by_deadline} idle={} deferred={} mean_us={} longest_us={} \
         delay_mean_us={} delay_longest_us={} next_us={next_us} next_timer={} \
         step_longest_us={} step_shootdowns={} step_serving={serving} \
         unmaps={device}/{pages}/{files}/{stacks}",
        take(&PUMPS.quiet),
        take(&PUMPS.unmasked),
        take(&PUMPS.forced),
        take(&PUMPS.relaxed),
        take(&PUMPS.posted),
        take(&PUMPS.idle),
        take(&PUMPS.deferred),
        micros(ticks.checked_div(runs).unwrap_or(0)),
        micros(take(&PUMPS.longest)),
        micros(delay.checked_div(runs).unwrap_or(0)),
        micros(take(&PUMPS.delay_longest)),
        next_timer(),
        micros(take(&PUMPS.step_longest)),
        take(&PUMPS.step_shootdowns),
    );
    // Each part's total over the pumps since the last report.
    let parts: Vec<String> = PUMPS
        .parts
        .iter()
        .map(|part| format!("{}", micros(take(part))))
        .collect();
    let longest: Vec<String> = PUMPS
        .part_longest
        .iter()
        .map(|part| format!("{}", micros(take(part))))
        .collect();
    let pumps = format!(
        "{pumps} total_us={names}/{} part_longest_us={names}/{}",
        parts.join("/"),
        longest.join("/"),
        names = PumpPart::NAMES.join("/"),
    );
    let linux = linux_report(take(&PUMPS.steps), tsc_hz);
    Some(pumps + &linux + &nvkm)
}

/// `compat/stela/counts.h`: `struct stela_linux_count_kind`.
#[repr(C)]
struct LinuxCountKind {
    /// Per slot the function and its count.
    slots: [[AtomicUsize; 2]; 32],
    other: AtomicUsize,
}

/// `compat/stela/counts.h`: `struct stela_linux_counts`.
#[repr(C)]
struct LinuxCounts {
    timers: LinuxCountKind,
    works: LinuxCountKind,
    sleepers: LinuxCountKind,
    timeouts: AtomicUsize,
}

/// `\nGPU LINUX timers=+0x…:N,… works=… sleepers=… timeouts=N steps=N`:
/// since the last report, the timer callbacks (timer-list and
/// high-resolution) and work items that ran in the environment, most first,
/// by function as an offset in this image (`other:N` for the rest), the tasks
/// a timeout woke by the work item or thread function they ran (`-` for
/// neither) and in all, and the Linux steps the pumps ran. Then
/// `step_us=select/body/after empty_us=N empties=N scans=N locks=N clocks=N`:
/// the work dispatchers' time (all runtimes) in steps that ran a task (entry to the switch, the
/// task's run, return to exit), its time and count of steps that found
/// nothing, its job-slot scans for idle tasks, its queue locks, and the
/// native clock's samples.
fn linux_report(steps: u64, tsc_hz: u64) -> String {
    /// Functions listed per kind.
    const TOP: usize = 4;
    unsafe extern "C" {
        /// `compat/kernel/counts.c`, which updates it only atomically.
        static stela_linux_counts: LinuxCounts;
        static __ImageBase: u8;
    }
    // SAFETY: counters the environment updates only atomically.
    let counts = unsafe { &stela_linux_counts };
    let base = &raw const __ImageBase as usize;
    let list = |kind: &LinuxCountKind| {
        let mut taken: Vec<(usize, usize)> = kind
            .slots
            .iter()
            .filter_map(|[function, count]| {
                let count = count.swap(0, Ordering::Relaxed);
                (count != 0).then(|| (function.load(Ordering::Relaxed), count))
            })
            .collect();
        taken.sort_unstable_by_key(|&(_, count)| core::cmp::Reverse(count));
        let other = kind.other.swap(0, Ordering::Relaxed)
            + taken
                .iter()
                .skip(TOP)
                .map(|&(_, count)| count)
                .sum::<usize>();
        let mut line = String::new();
        for &(function, count) in taken.iter().take(TOP) {
            if !line.is_empty() {
                line.push(',');
            }
            match function {
                // The counters' mark for no function.
                1 => line.push_str(&format!("-:{count}")),
                function => line.push_str(&format!("+{:#x}:{count}", function.wrapping_sub(base))),
            }
        }
        if other != 0 {
            if !line.is_empty() {
                line.push(',');
            }
            line.push_str(&format!("other:{other}"));
        }
        if line.is_empty() {
            line.push('-');
        }
        line
    };
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let step = stela_drivers::platform::work::step_counts::take();
    format!(
        "\nGPU LINUX timers={} works={} sleepers={} timeouts={} steps={steps} \
         step_us={}/{}/{} empty_us={} empties={} scans={} locks={} clocks={}",
        list(&counts.timers),
        list(&counts.works),
        list(&counts.sleepers),
        counts.timeouts.swap(0, Ordering::Relaxed),
        micros(step.select),
        micros(step.body),
        micros(step.after),
        micros(step.empty),
        step.empty_steps,
        step.scans,
        step.locks,
        crate::native_clock::SAMPLES.swap(0, Ordering::Relaxed),
    )
}

/// `\nGPU NVKM kind:count/total_us/longest_us …`: where nouveau's page-table
/// and instance-memory paths spent their time since the last report
/// (`include/nvkm/core/counts.h`), for kinds that ran, then
/// `\nGPU NVKM SIZES kind:count/total_kib/largest_kib …` for its page-table
/// references by page size and its sparse references. Empty when none ran.
#[cfg(feature = "nvk")]
fn nvkm_report() -> String {
    /// `enum nvkm_count`'s kinds, in order.
    const KINDS: [&str; 12] = [
        "flush",
        "pt_new",
        "pt_cached",
        "pt_free",
        "kmap",
        "evict",
        "kmap_slow",
        "rpc",
        "get_4k",
        "get_64k",
        "get_2m",
        "sparse",
    ];
    /// The first kind that counts bytes, not nanoseconds.
    const SIZES: usize = 8;
    unsafe extern "C" {
        /// `nvkm/subdev/mmu/base.c`: per kind a count, nanoseconds (or
        /// bytes) and the longest (or largest), which nvkm updates
        /// atomically.
        static nvkm_counts: [[AtomicU64; 3]; 12];
    }
    let mut times = String::new();
    let mut sizes = String::new();
    // SAFETY: nvkm's static counters, which it only updates atomically.
    let counts = unsafe { &nvkm_counts };
    for (index, (kind, [count, total, longest])) in KINDS.iter().zip(counts).enumerate() {
        let count = count.swap(0, Ordering::Relaxed);
        let total = total.swap(0, Ordering::Relaxed);
        let longest = longest.swap(0, Ordering::Relaxed);
        if count == 0 {
            continue;
        }
        let (line, head, unit) = if index < SIZES {
            (
                &mut times,
                "\nGPU NVKM kind:count/total_us/longest_us",
                1000,
            )
        } else {
            (
                &mut sizes,
                "\nGPU NVKM SIZES kind:count/total_kib/largest_kib",
                1024,
            )
        };
        if line.is_empty() {
            line.push_str(head);
        }
        line.push_str(&format!(
            " {kind}:{count}/{}/{}",
            total / unit,
            longest / unit
        ));
    }
    times + &sizes
}

/// The function of the timer-list timer due first when Linux last looked,
/// as an offset in this image, or `-` for none.
fn next_timer() -> String {
    unsafe extern "C" {
        /// `compat/kernel/timer.c`: the first pending timer's function.
        static stela_linux_timer_next_function: AtomicUsize;
        static __ImageBase: u8;
    }
    // SAFETY: a word the environment stores whenever it reads its queue.
    let function = unsafe { stela_linux_timer_next_function.load(Ordering::Relaxed) };
    if function == 0 {
        return String::from("-");
    }
    let base = &raw const __ImageBase as usize;
    format!("+{:#x}", function.wrapping_sub(base))
}

#[cfg(feature = "nvk")]
/// The GPU interrupts taken when the Metal services last looked for
/// completed work.
static LOOKED: AtomicU64 = AtomicU64::new(0);
#[cfg(feature = "nvk")]
/// A service a call held asked to look again.
static LOOK: AtomicBool = AtomicBool::new(false);
#[cfg(feature = "nvk")]
/// When a pump next looks though nothing asked, as a TSC value.
static NEXT_LOOK: AtomicU64 = AtomicU64::new(0);

#[cfg(feature = "nvk")]
/// How long a pump goes without looking when the GPU has not interrupted: a
/// fence Linux signals outside its interrupt, as a submission or a job's
/// timeout may, is seen within this.
const LOOK_FALLBACK_NS: u64 = 10_000_000;

#[cfg(feature = "nvk")]
/// What a pump looks at after its steps ([`look_due`]).
enum Look {
    /// Each GPU's completion, which enters the driver, and the devices'
    /// new completion timelines.
    Device,
    /// Only the devices' new completion timelines, which need no driver.
    Waiters,
    No,
}

#[cfg(feature = "nvk")]
/// What this pump looks at: each look asks the driver for each GPU's
/// completion, which only an interrupt or a waiter's point makes likely to
/// have moved. Most pumps run for a call a PE posted or a Linux timer. The
/// fallback look every [`LOOK_FALLBACK_NS`] is for a device no waiter
/// watches: while one waits on the only device's timeline, every fence it
/// signals reaches the waiter, which asks for the look itself, so while
/// every device's waiter waits the fallback only starts the waiters of new
/// timelines.
fn look_due(gpu: &Installed) -> Look {
    let fired = FIRED.load(Ordering::Acquire);
    let interrupted = LOOKED.swap(fired, Ordering::Relaxed) != fired;
    let asked = LOOK.swap(false, Ordering::AcqRel);
    let now = host::rdtsc();
    let fallback = now >= NEXT_LOOK.load(Ordering::Relaxed);
    if interrupted || asked || fallback {
        NEXT_LOOK.store(
            now.saturating_add(gpu.tsc_after(LOOK_FALLBACK_NS)),
            Ordering::Relaxed,
        );
    }
    if interrupted || asked {
        Look::Device
    } else if !fallback {
        Look::No
    } else if waiter::watching(gate::shared().len()) {
        Look::Waiters
    } else {
        Look::Device
    }
}

#[cfg(feature = "nvk")]
/// Pumps the service again soon, from any processor, and has that pump look
/// for completed work.
fn look_again() {
    LOOK.store(true, Ordering::Release);
    if let Some(gpu) = installed() {
        // This raises a flag and calls the boot PE, so it cannot fail.
        let _ = gpu.events.notify();
    }
}

/// `ticks` of the host counter, cut short to end at Linux's next deadline,
/// or, while work is ready but held back for a calmer boot PE, when the
/// run loop next looks whether to pump ([`pump_due`]).
pub(super) fn slice(ticks: u64) -> u64 {
    let Some(gpu) = installed() else {
        return ticks;
    };
    let now = host::rdtsc();
    let end = match ready_since(gpu, now) {
        Some(since) => {
            let unmasked = since.saturating_add(gpu.tsc_after(DEFER_UNMASKED_NS));
            let longest = since.saturating_add(gpu.tsc_after(DEFER_LONGEST_NS));
            // Every recheck, so that a boot PE that reaches EL0 or waits
            // for an interrupt (calm, `pump_due`) pumps then rather than at
            // the unmasked deadline: a submission or completion waits a
            // quarter of a millisecond for a calm moment, not the whole one.
            let recheck = now.saturating_add(gpu.tsc_after(RECHECK_NS));
            if now < unmasked {
                recheck.min(unmasked)
            } else {
                recheck.min(longest)
            }
        }
        None => gpu.next.load(Ordering::Relaxed),
    };
    let left = end.saturating_sub(now);
    let left = u128::from(left) * u128::from(host::COUNTER_HZ) / u128::from(gpu.tsc_hz.max(1));
    ticks.min(left.min(u128::from(u64::MAX)) as u64).max(1)
}
