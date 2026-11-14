//! The host firmware's AML, run by the pinned ACPICA
//! (`stela_drivers::platform::acpi`, whose `Services` table is the provider
//! contract) through the engine's own OSL services: its heap, the TSC, port I/O, PCI
//! configuration space and the loader root's mappings.
//!
//! The boot keeps its EC binding before namespace initialization. When the
//! captured SCI resources check, a native interrupt masks the owned line
//! and the BSP later dispatches ACPICA and its FIFO work under RestoreRoot.
//! Power-device notifications coalesce into refresh work on that same BSP.
//!
//! Every ACPICA call runs on the boot processor under the loader root
//! ([`super::storage::RestoreRoot`]), with interrupts off and the host drivers' lock held.
//! There RAM, ACPI tables and NVS included, is reached through the physmap,
//! write-back; any other address, device registers, through an identity
//! mapping the provider adds to the loader root uncached on first use. When
//! the native PCI source takes configuration custody, AML has scoped scalar
//! reads through that source, for its PCI_Config fields and for its
//! SystemMemory fields that land in an ECAM window, which ACPICA reaches
//! through [`read_memory`]. Otherwise the firmware provider uses PCI
//! configuration mechanism #1 and the MCFG's ECAM windows.
pub(super) mod sci;
use super::{
    host,
    kernel_work::KernelWork,
    power::{in8, in16, in32, out8, out16, out32},
};
use crate::{
    layout, native_mmio, output,
    tables::{self, Frames, NX, PAGE, PRESENT, UNCACHED, WRITABLE},
};
use alloc::{boxed::Box, collections::VecDeque, string::String, vec::Vec};
use core::{
    alloc::Layout,
    arch::asm,
    cell::{Cell, RefCell},
    ffi::{c_char, c_void},
    ptr::{self, NonNull},
    sync::atomic::{AtomicBool, AtomicU32, Ordering},
};
use stela_boot_core::ecam::memory as ecam_memory;
use stela_drivers::{
    bus::pci::{Address, EcamWindow},
    platform::acpi::{
        Interrupt, Mode, PciId, SERVICES_ABI_BYTES, SERVICES_ABI_VERSION, Services, Session,
        Status, Value, Work,
    },
    usb::typec::ucsi::acpi::attachment as ucsi,
};
use stela_engine::machine::mmu::placement::Placement;

// ACPICA's statuses (`acexcep.h`).
const AE_OK: Status = 0;
const AE_ERROR: Status = 0x0001;
const AE_NOT_EXIST: Status = 0x0006;
const AE_SUPPORT: Status = 0x000f;
const AE_LIMIT: Status = 0x0010;
const AE_TIME: Status = 0x0011;
const AE_BAD_PARAMETER: Status = 0x1001;
/// `ACPI_WAIT_FOREVER`, a semaphore wait with no timeout.
const WAIT_FOREVER: u16 = 0xffff;
/// `ACPI_SIGNAL_FATAL`: AML's Fatal operator.
const SIGNAL_FATAL: u32 = 0;
/// RFLAGS.IF.
const INTERRUPTS: u64 = 1 << 9;
/// Connector changes one poll reads from a UCSI manager, bounding a port that
/// keeps changing; the rest stay pending for the next poll.
const UCSI_SERVICE_BUDGET: usize = 16;
const PCI_ADDRESS: u16 = 0xcf8;
const PCI_DATA: u16 = 0xcfc;
/// The header [`allocate`] keeps before each allocation: its layout's size.
const HEADER: usize = 16;

/// What the provider reports once, as unsupported.
#[derive(Clone, Copy)]
enum Gap {
    /// A wait no other processor or interrupt could ever end.
    Wait = 1,
    /// Configuration space beyond mechanism #1 that no ECAM window decodes.
    Config = 2,
    /// An address beyond what the loader root can map by identity.
    Address = 4,
    /// Access that would bypass the native PCI configuration owner.
    PciCustody = 8,
    /// An AML SystemMemory read in ECAM the native PCI owner refused.
    EcamRead = 16,
    /// An AML SystemMemory write in ECAM, under native PCI custody.
    EcamWrite = 32,
}

struct Provider {
    /// The one BSP executor; Session callbacks never infer a task from IF.
    executor: &'static KernelWork,
    /// The loader root, which takes the identity mappings of device pages.
    root: u64,
    ecam: Vec<EcamWindow>,
    pci: Option<&'static native_mmio::Source>,
    tsc_hz: u64,
    /// `acpi_os_execute`'s work, run in order by [`drain_work`].
    work: RefCell<VecDeque<(Work, usize)>>,
    queued: AtomicBool,
    /// Quiesced: work is refused.
    closed: Cell<bool>,
    reported: Cell<u32>,
    prepared_sci: RefCell<Option<sci::Prepared>>,
    sci: RefCell<Option<sci::Armed>>,
    polling: Cell<bool>,
    needs_rearm: Cell<bool>,
}

impl Provider {
    fn report(&self, gap: Gap, message: core::fmt::Arguments<'_>) {
        let bit = gap as u32;
        if self.reported.get() & bit == 0 {
            self.reported.set(self.reported.get() | bit);
            output::arguments(format_args!("UNSUPPORTED ACPI {message}\n"));
        }
    }

    /// `bytes` at `physical` under the loader root: through the physmap
    /// where it maps every page (RAM), else by identity, mapping the pages
    /// not yet mapped uncached. No ECAM page is lent under native custody:
    /// its scalar accesses go through [`Self::configuration_owner`].
    fn reach(&self, physical: u64, bytes: u64) -> Option<*mut u8> {
        let end = physical.checked_add(bytes.max(1))?;
        if self.configuration_owner(physical, bytes).is_some() {
            self.report(
                Gap::PciCustody,
                format_args!("native PCI configuration lends no mapping of ECAM at {physical:#x}"),
            );
            return None;
        }
        let pages = (physical & !(PAGE - 1)..end).step_by(PAGE as usize);
        if pages
            .clone()
            .all(|page| self.mapped(layout::physical(page)))
        {
            return Some(layout::physical(physical) as *mut u8);
        }
        let limit = match layout::placement() {
            Placement::FiveLevel => 1 << 52,
            Placement::FourLevel { .. } => 1 << 47,
        };
        if end > limit {
            self.report(
                Gap::Address,
                format_args!("firmware address {physical:#x} lies beyond the identity map"),
            );
            return None;
        }
        for page in pages {
            if !self.mapped(page) {
                let rights = PRESENT | WRITABLE | NX | UNCACHED;
                tables::map_page(&mut HeapFrames, self.root, page, page, rights).ok()?;
            }
        }
        Some(physical as *mut u8)
    }

    /// The native PCI owner, when `bytes` at `physical` touch an ECAM window
    /// under its custody.
    fn configuration_owner(
        &self,
        physical: u64,
        bytes: u64,
    ) -> Option<&'static native_mmio::Source> {
        self.pci
            .filter(|_| ecam_memory::touches(&self.ecam, physical, bytes))
    }

    /// AML's read of `bytes` at `physical` in an ECAM window, as the
    /// configuration reads of the functions it decodes to, which the native
    /// owner serves as it serves PCI_Config fields; `None`, reported once,
    /// when a read is refused.
    fn read_configuration(
        &self,
        pci: &native_mmio::Source,
        physical: u64,
        bytes: u64,
    ) -> Option<u64> {
        let read = ecam_memory::read(&self.ecam, physical, bytes, |address, offset, width| {
            pci.read_firmware_config(address, offset, width)
        });
        match read {
            Ok(value) => Some(value),
            Err(ecam_memory::Refusal::Config {
                address,
                offset,
                error,
            }) => {
                self.report(
                    Gap::EcamRead,
                    format_args!(
                        "native PCI read of {address} register {offset:#x} through ECAM at \
                         {physical:#x} refused: {error:?}"
                    ),
                );
                None
            }
            Err(ecam_memory::Refusal::Outside { physical: outside }) => {
                self.report(
                    Gap::EcamRead,
                    format_args!(
                        "AML read of {bytes} bytes at {physical:#x} leaves ECAM at {outside:#x}"
                    ),
                );
                None
            }
            Err(ecam_memory::Refusal::Width) => {
                self.report(
                    Gap::EcamRead,
                    format_args!("AML read of {bytes} bytes at {physical:#x} in ECAM"),
                );
                None
            }
        }
    }

    /// Whether the loader root maps `address`.
    fn mapped(&self, address: u64) -> bool {
        match tables::entry(&mut HeapFrames, self.root, address, 0, false) {
            // Only a 2 MiB page stops the walk early.
            Err(_) => true,
            // SAFETY: a live leaf slot of the loader root.
            Ok(slot) => slot.is_some_and(|slot| unsafe { slot.read_volatile() } & PRESENT != 0),
        }
    }

    /// The ECAM address of `register` in `id`'s configuration space.
    fn ecam(&self, id: &PciId, register: u32) -> Option<u64> {
        let bus = u8::try_from(id.bus).ok()?;
        let window = self.ecam.iter().find(|window| {
            window.segment == id.segment && (window.start_bus..=window.end_bus).contains(&bus)
        })?;
        (id.device < 32 && id.function < 8 && register < 4096).then(|| {
            window.base
                + (u64::from(bus) << 20
                    | u64::from(id.device) << 15
                    | u64::from(id.function) << 12
                    | u64::from(register))
        })
    }

    fn permits_port(&self, port: u16, width: u32) -> bool {
        let first = u32::from(port);
        if self.pci.is_some()
            && first < u32::from(PCI_DATA) + 4
            && first + width / 8 > u32::from(PCI_ADDRESS)
        {
            self.report(
                Gap::PciCustody,
                format_args!("native PCI configuration owns the legacy configuration ports"),
            );
            return false;
        }
        true
    }

    fn dispatch_sci(&self, work: bool) -> bool {
        if self.polling.replace(true) {
            return false;
        }
        let armed = self.sci.borrow_mut().take();
        let mut handled = false;
        if let Some(armed) = &armed {
            match armed.dispatch() {
                Ok(dispatched) => {
                    handled = dispatched;
                    if dispatched {
                        self.needs_rearm.set(true);
                    }
                }
                Err(error) => output::arguments(format_args!("HOST ACPI SCI failed: {error:?}\n")),
            }
        }
        // ACPICA's ISR has acknowledged fixed events and disabled deferred
        // GPE contributors. Rearm the parent before executing AML: a worker
        // may wait for the FACS global-lock release fixed event on this SCI.
        if self.needs_rearm.get()
            && let Some(armed) = &armed
        {
            // SAFETY: the native IRQ returned/EOIed and the C ISR completed;
            // each deferred contributor owns its own disabled source until done.
            match unsafe { armed.rearm() } {
                Ok(()) => self.needs_rearm.set(false),
                Err(error) => {
                    output::arguments(format_args!("HOST ACPI SCI rearm failed: {error:?}\n"))
                }
            }
        }
        *self.sci.borrow_mut() = armed;
        self.polling.set(false);
        if work {
            // SAFETY: process queue owned by this provider, with no route or
            // queue borrow held across a worker's OSL semaphore waits.
            if unsafe { drain_work(core::ptr::from_ref(self).cast_mut().cast()) } != AE_OK {
                output::fatal("ACPI deferred work completion failed");
            }
        }
        handled
    }
    fn milliseconds(&self, milliseconds: u64) {
        let start = host::rdtsc();
        let ticks = self.tsc_hz / 1000 * milliseconds;
        while host::rdtsc().wrapping_sub(start) < ticks {
            core::hint::spin_loop();
        }
    }
}

/// Page-table frames from the engine heap, which lies in the physmap.
struct HeapFrames;

impl Frames for HeapFrames {
    fn frame(&mut self) -> Option<u64> {
        let layout = Layout::from_size_align(PAGE as usize, PAGE as usize).ok()?;
        // SAFETY: a nonzero layout; the frame stays the loader root's.
        let page = unsafe { alloc::alloc::alloc_zeroed(layout) };
        (!page.is_null()).then(|| page as u64 - layout::physical(0))
    }

    fn table(&self, physical: u64) -> *mut u64 {
        layout::physical(physical) as *mut u64
    }
}

/// # Safety
/// `context` is the session's live [`Provider`].
unsafe fn provider<'a>(context: *mut c_void) -> &'a Provider {
    // SAFETY: the caller's contract.
    unsafe { &*context.cast::<Provider>() }
}

unsafe extern "C" fn allocate(_: *mut c_void, bytes: u64) -> *mut c_void {
    let Some(layout) = usize::try_from(bytes)
        .ok()
        .and_then(|bytes| bytes.checked_add(HEADER))
        .and_then(|total| Layout::from_size_align(total, HEADER).ok())
    else {
        return ptr::null_mut();
    };
    // SAFETY: a nonzero layout.
    let raw = unsafe { alloc::alloc::alloc(layout) };
    if raw.is_null() {
        return ptr::null_mut();
    }
    // SAFETY: the header lies in this allocation, which is aligned for it.
    unsafe {
        raw.cast::<usize>().write(layout.size());
        raw.add(HEADER).cast()
    }
}

unsafe extern "C" fn free(_: *mut c_void, allocation: *mut c_void) {
    // SAFETY: `allocate` returned `allocation`, its header just before it.
    unsafe {
        let raw = allocation.cast::<u8>().sub(HEADER);
        let size = raw.cast::<usize>().read();
        alloc::alloc::dealloc(raw, Layout::from_size_align_unchecked(size, HEADER));
    }
}

unsafe extern "C" fn map(context: *mut c_void, physical: u64, bytes: u64) -> *mut c_void {
    // SAFETY: ACPICA passes the session's provider.
    unsafe { provider(context) }
        .reach(physical, bytes)
        .map_or(ptr::null_mut(), |pointer| pointer.cast())
}

/// The lease ends; the mapping stays the loader root's, as the identity
/// map's other pages do, for the next lease of the same registers.
unsafe extern "C" fn unmap(_: *mut c_void, _: *mut c_void, _: u64) {}

unsafe extern "C" fn create_lock(_: *mut c_void, out: *mut *mut c_void) -> Status {
    // SAFETY: ACPICA's writable output.
    unsafe { out.write(Box::into_raw(Box::new(AtomicBool::new(false))).cast()) };
    AE_OK
}

unsafe extern "C" fn delete_lock(_: *mut c_void, lock: *mut c_void) {
    // SAFETY: `create_lock` made `lock`, which ACPICA deletes once.
    drop(unsafe { Box::from_raw(lock.cast::<AtomicBool>()) });
}

unsafe extern "C" fn acquire_lock(_: *mut c_void, lock: *mut c_void) -> u64 {
    let flags: u64;
    // SAFETY: CPL0; `release_lock` restores IF.
    unsafe { asm!("pushfq", "pop {}", "cli", out(reg) flags, options(nomem)) };
    // SAFETY: a lock `create_lock` made.
    let lock = unsafe { &*lock.cast::<AtomicBool>() };
    while lock
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
    flags
}

unsafe extern "C" fn release_lock(_: *mut c_void, lock: *mut c_void, flags: u64) {
    // SAFETY: a lock `create_lock` made.
    unsafe { &*lock.cast::<AtomicBool>() }.store(false, Ordering::Release);
    if flags & INTERRUPTS != 0 {
        // SAFETY: `acquire_lock` cleared IF, which was set.
        unsafe { asm!("sti", options(nomem, nostack)) };
    }
}

struct Semaphore {
    units: AtomicU32,
    max: u32,
}

unsafe extern "C" fn create_semaphore(
    _: *mut c_void,
    max: u32,
    initial: u32,
    out: *mut *mut c_void,
) -> Status {
    if max == 0 || initial > max {
        return AE_BAD_PARAMETER;
    }
    let semaphore = Semaphore {
        units: AtomicU32::new(initial),
        max,
    };
    // SAFETY: ACPICA's writable output.
    unsafe { out.write(Box::into_raw(Box::new(semaphore)).cast()) };
    AE_OK
}

unsafe extern "C" fn delete_semaphore(_: *mut c_void, semaphore: *mut c_void) -> Status {
    // SAFETY: `create_semaphore` made it, and ACPICA deletes it once.
    drop(unsafe { Box::from_raw(semaphore.cast::<Semaphore>()) });
    AE_OK
}

unsafe extern "C" fn wait_semaphore(
    context: *mut c_void,
    semaphore: *mut c_void,
    units: u32,
    timeout: u16,
) -> Status {
    // SAFETY: ACPICA passes the session's provider and a live semaphore.
    let (provider, semaphore) = unsafe { (provider(context), &*semaphore.cast::<Semaphore>()) };
    if units == 0 || units > semaphore.max {
        return AE_BAD_PARAMETER;
    }
    let take = || {
        semaphore
            .units
            .try_update(Ordering::AcqRel, Ordering::Acquire, |held| {
                held.checked_sub(units)
            })
            .is_ok()
    };
    if take() {
        return AE_OK;
    }
    if timeout == WAIT_FOREVER && provider.sci.borrow().is_none() {
        provider.report(
            Gap::Wait,
            format_args!("semaphore wait has no interrupt progress source"),
        );
        return AE_TIME;
    }
    let start = host::rdtsc();
    let ticks = provider.tsc_hz / 1000 * u64::from(timeout);
    loop {
        if take() {
            return AE_OK;
        }
        if timeout != WAIT_FOREVER && host::rdtsc().wrapping_sub(start) >= ticks {
            break;
        }
        if let Ok(platform) = crate::native_irq::NativePlatform::current() {
            // SAFETY: current BSP and provider root are held through this
            // wait; only the small native IRQ handlers run in this window.
            unsafe { stela_drivers::bus::irq::Platform::interrupt_window(&platform) };
            provider.dispatch_sci(false);
        }
        core::hint::spin_loop();
    }
    AE_TIME
}

unsafe extern "C" fn signal_semaphore(
    _: *mut c_void,
    semaphore: *mut c_void,
    units: u32,
) -> Status {
    // SAFETY: a live semaphore `create_semaphore` made.
    let semaphore = unsafe { &*semaphore.cast::<Semaphore>() };
    match semaphore
        .units
        .try_update(Ordering::AcqRel, Ordering::Acquire, |held| {
            held.checked_add(units)
                .filter(|&next| next <= semaphore.max)
        }) {
        Ok(_) => AE_OK,
        Err(_) => AE_LIMIT,
    }
}

unsafe extern "C" fn install_interrupt(
    context: *mut c_void,
    irq: u32,
    handler: Interrupt,
    argument: *mut c_void,
) -> Status {
    // SAFETY: this live provider is held by the active ACPICA session.
    let provider = unsafe { provider(context) };
    let Some(prepared) = provider.prepared_sci.borrow_mut().take() else {
        return AE_SUPPORT;
    };
    if prepared.irq() != irq {
        return AE_BAD_PARAMETER;
    }
    // SAFETY: ACPICA's permanent SCI handler/context and exclusive boot route.
    match unsafe { prepared.install(handler, argument) } {
        Ok(armed) => {
            *provider.sci.borrow_mut() = Some(armed);
            AE_OK
        }
        Err(error) => {
            output::arguments(format_args!("HOST ACPI SCI install failed: {error:?}\n"));
            AE_ERROR
        }
    }
}
unsafe extern "C" fn remove_interrupt(context: *mut c_void, _: u32, _: Interrupt) -> Status {
    // SAFETY: the session's provider; process context at termination.
    let provider = unsafe { provider(context) };
    let mut slot = provider.sci.borrow_mut();
    match slot.as_mut() {
        None => AE_NOT_EXIST,
        Some(armed) => match armed.retire() {
            Ok(()) => {
                slot.take();
                AE_OK
            }
            Err(_) => AE_ERROR,
        },
    }
}

/// ACPICA mutex ownership uses the continuation identity.
unsafe extern "C" fn thread_id(context: *mut c_void) -> u64 {
    // SAFETY: this Session keeps its provider and real affine work owner.
    let provider = unsafe { provider(context) };
    provider
        .executor
        .current_id()
        .unwrap_or_else(|_| output::fatal("ACPICA called outside its owned native continuation"))
}

unsafe extern "C" fn execute(
    context: *mut c_void,
    _: u32,
    work: Work,
    argument: *mut c_void,
) -> Status {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    if provider.closed.get() {
        return AE_NOT_EXIST;
    }
    provider
        .work
        .borrow_mut()
        .push_back((work, argument as usize));
    provider.queued.store(true, Ordering::Release);
    AE_OK
}

unsafe extern "C" fn drain_work(context: *mut c_void) -> Status {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    loop {
        let next = provider.work.borrow_mut().pop_front();
        let Some((work, argument)) = next else {
            provider.queued.store(false, Ordering::Release);
            return AE_OK;
        };
        // SAFETY: ACPICA queued this work with its argument.
        unsafe { work(argument as *mut c_void) };
    }
}

unsafe extern "C" fn sleep(context: *mut c_void, milliseconds: u64) {
    // SAFETY: ACPICA passes the session's provider.
    unsafe { provider(context) }.milliseconds(milliseconds);
}

unsafe extern "C" fn stall(context: *mut c_void, microseconds: u32) {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    let start = host::rdtsc();
    let ticks = provider.tsc_hz / 1_000_000 * u64::from(microseconds);
    while host::rdtsc().wrapping_sub(start) < ticks {
        core::hint::spin_loop();
    }
}

unsafe extern "C" fn read_port(
    context: *mut c_void,
    port: u64,
    value: *mut u32,
    width: u32,
) -> Status {
    let Ok(port) = u16::try_from(port) else {
        return AE_BAD_PARAMETER;
    };
    if !matches!(width, 8 | 16 | 32) {
        return AE_BAD_PARAMETER;
    }
    // SAFETY: ACPICA passes the session's provider.
    if !unsafe { provider(context) }.permits_port(port, width) {
        return AE_SUPPORT;
    }
    // SAFETY: the firmware's AML or FADT names the port; ACPICA screens it
    // as Linux's does; the output is writable.
    unsafe {
        value.write(match width {
            8 => in8(port).into(),
            16 => in16(port).into(),
            32 => in32(port),
            _ => unreachable!("port width was checked"),
        })
    };
    AE_OK
}

unsafe extern "C" fn write_port(context: *mut c_void, port: u64, value: u32, width: u32) -> Status {
    let Ok(port) = u16::try_from(port) else {
        return AE_BAD_PARAMETER;
    };
    if !matches!(width, 8 | 16 | 32) {
        return AE_BAD_PARAMETER;
    }
    // SAFETY: ACPICA passes the session's provider.
    if !unsafe { provider(context) }.permits_port(port, width) {
        return AE_SUPPORT;
    }
    // SAFETY: as in `read_port`.
    unsafe {
        match width {
            8 => out8(port, value as u8),
            16 => out16(port, value as u16),
            32 => out32(port, value),
            _ => unreachable!("port width was checked"),
        }
    }
    AE_OK
}

/// # Safety
/// `pointer` reaches `width` bits the firmware names.
unsafe fn read(pointer: *mut u8, width: u32) -> Option<u64> {
    // SAFETY: the caller's contract.
    unsafe {
        Some(match width {
            8 => pointer.read_volatile().into(),
            16 => pointer.cast::<u16>().read_volatile().into(),
            32 => pointer.cast::<u32>().read_volatile().into(),
            64 => pointer.cast::<u64>().read_volatile(),
            _ => return None,
        })
    }
}

/// # Safety
/// As [`read`].
unsafe fn write(pointer: *mut u8, value: u64, width: u32) -> bool {
    // SAFETY: the caller's contract.
    unsafe {
        match width {
            8 => pointer.write_volatile(value as u8),
            16 => pointer.cast::<u16>().write_volatile(value as u16),
            32 => pointer.cast::<u32>().write_volatile(value as u32),
            64 => pointer.cast::<u64>().write_volatile(value),
            _ => return false,
        }
    }
    true
}

/// A hardware register or an AML SystemMemory field: the bridge routes
/// every SystemMemory region access here.
unsafe extern "C" fn read_memory(
    context: *mut c_void,
    physical: u64,
    value: *mut u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    if !matches!(width, 8 | 16 | 32 | 64) {
        return AE_BAD_PARAMETER;
    }
    let bytes = u64::from(width / 8);
    let found = if let Some(pci) = provider.configuration_owner(physical, bytes) {
        provider.read_configuration(pci, physical, bytes)
    } else {
        let Some(pointer) = provider.reach(physical, bytes) else {
            return AE_ERROR;
        };
        // SAFETY: the firmware's register, now mapped; the width was checked.
        unsafe { read(pointer, width) }
    };
    let Some(found) = found else {
        return AE_SUPPORT;
    };
    // SAFETY: ACPICA's writable output.
    unsafe { value.write(found) };
    AE_OK
}

/// As [`read_memory`]. Native PCI custody excludes AML configuration
/// writes, through ECAM as through PCI_Config fields ([`write_pci`]).
unsafe extern "C" fn write_memory(
    context: *mut c_void,
    physical: u64,
    value: u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    if provider
        .configuration_owner(physical, u64::from(width / 8))
        .is_some()
    {
        provider.report(
            Gap::EcamWrite,
            format_args!("native PCI configuration excludes AML writes to ECAM at {physical:#x}"),
        );
        return AE_SUPPORT;
    }
    let Some(pointer) = provider.reach(physical, u64::from(width / 8)) else {
        return AE_ERROR;
    };
    // SAFETY: the firmware's register, now mapped.
    match unsafe { write(pointer, value, width) } {
        true => AE_OK,
        false => AE_BAD_PARAMETER,
    }
}

/// Configuration mechanism #1's address, as Linux's `PCI_CONF1_ADDRESS`,
/// for segment 0's first 256 registers.
fn mechanism_one(id: &PciId, register: u32) -> Option<u32> {
    (id.segment == 0 && id.bus < 256 && id.device < 32 && id.function < 8 && register < 256).then(
        || {
            0x8000_0000
                | u32::from(id.bus) << 16
                | u32::from(id.device) << 11
                | u32::from(id.function) << 8
                | register & 0xfc
        },
    )
}

unsafe extern "C" fn read_pci(
    context: *mut c_void,
    id: *mut PciId,
    register: u32,
    value: *mut u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes the session's provider and a live identity.
    let (provider, id) = unsafe { (provider(context), &*id) };
    if !matches!(width, 8 | 16 | 32) {
        return AE_ERROR;
    }
    let read = if let Some(pci) = provider.pci {
        let (Ok(bus), Ok(device), Ok(function), Ok(offset)) = (
            u8::try_from(id.bus),
            u8::try_from(id.device),
            u8::try_from(id.function),
            u16::try_from(register),
        ) else {
            return AE_BAD_PARAMETER;
        };
        let address = Address {
            segment: id.segment,
            bus,
            device,
            function,
        };
        match pci.read_firmware_config(address, offset, (width / 8) as u16) {
            Ok(value) => u64::from(value),
            Err(error) => {
                provider.report(
                    Gap::Config,
                    format_args!(
                        "native PCI read of {address} register {offset:#x} refused: {error:?}"
                    ),
                );
                return AE_SUPPORT;
            }
        }
    } else if let Some(address) = mechanism_one(id, register) {
        let port = PCI_DATA + (register & 3) as u16;
        // SAFETY: mechanism #1's ports, under the host drivers' lock.
        unsafe {
            out32(PCI_ADDRESS, address);
            match width {
                8 => u64::from(in8(port)),
                16 => u64::from(in16(port & !1)),
                _ => u64::from(in32(PCI_DATA)),
            }
        }
    } else {
        let Some(pointer) = provider
            .ecam(id, register)
            .and_then(|address| provider.reach(address, u64::from(width / 8)))
        else {
            provider.report(
                Gap::Config,
                format_args!(
                    "configuration space {id:?} register {register:#x} has no ECAM window"
                ),
            );
            return AE_SUPPORT;
        };
        // SAFETY: an ECAM register, mapped uncached.
        unsafe { read(pointer, width) }.unwrap_or_default()
    };
    // SAFETY: ACPICA's writable output.
    unsafe { value.write(read) };
    AE_OK
}

unsafe extern "C" fn write_pci(
    context: *mut c_void,
    id: *mut PciId,
    register: u32,
    value: u64,
    width: u32,
) -> Status {
    // SAFETY: ACPICA passes the session's provider and a live identity.
    let (provider, id) = unsafe { (provider(context), &*id) };
    if !matches!(width, 8 | 16 | 32) {
        return AE_ERROR;
    }
    if provider.pci.is_some() {
        provider.report(
            Gap::PciCustody,
            format_args!("native PCI configuration excludes AML writes"),
        );
        return AE_SUPPORT;
    }
    if let Some(address) = mechanism_one(id, register) {
        let port = PCI_DATA + (register & 3) as u16;
        // SAFETY: mechanism #1's ports, under the host drivers' lock.
        unsafe {
            out32(PCI_ADDRESS, address);
            match width {
                8 => out8(port, value as u8),
                16 => out16(port & !1, value as u16),
                _ => out32(PCI_DATA, value as u32),
            }
        }
        return AE_OK;
    }
    let Some(pointer) = provider
        .ecam(id, register)
        .and_then(|address| provider.reach(address, u64::from(width / 8)))
    else {
        provider.report(
            Gap::Config,
            format_args!("configuration space {id:?} register {register:#x} has no ECAM window"),
        );
        return AE_SUPPORT;
    };
    // SAFETY: an ECAM register, mapped uncached.
    unsafe { write(pointer, value, width) };
    AE_OK
}

/// 100 ns units, from the TSC.
unsafe extern "C" fn timer(context: *mut c_void) -> u64 {
    // SAFETY: ACPICA passes the session's provider.
    let hz = unsafe { provider(context) }.tsc_hz;
    (u128::from(host::rdtsc()) * 10_000_000 / u128::from(hz)) as u64
}

/// As Linux's `acpi_os_signal`: AML's Fatal is reported, and execution
/// goes on; a breakpoint is ignored.
unsafe extern "C" fn signal(_: *mut c_void, kind: u32, info: *mut c_void) -> Status {
    if kind == SIGNAL_FATAL && !info.is_null() {
        // SAFETY: ACPICA's `acpi_signal_fatal_info`: type, code, argument.
        let [kind, code, argument] = unsafe { info.cast::<[u32; 3]>().read() };
        output::arguments(format_args!(
            "ACPI firmware AML Fatal type={kind:#x} code={code:#x} argument={argument:#x}\n"
        ));
    }
    AE_OK
}

unsafe extern "C" fn log(_: *mut c_void, text: *const c_char, bytes: u64) {
    // SAFETY: ACPICA's formatted message of `bytes` bytes.
    let text = unsafe { core::slice::from_raw_parts(text.cast::<u8>(), bytes as usize) };
    output::arguments(format_args!("{}", String::from_utf8_lossy(text)));
}

unsafe extern "C" fn flush_cache(_: *mut c_void) {
    // SAFETY: CPL0; writes back and invalidates the caches.
    unsafe { asm!("wbinvd", options(nostack, preserves_flags)) };
}

/// No SCI or region callback runs beside AML: closing admission and running
/// the queued work quiesces the session.
unsafe extern "C" fn quiesce(context: *mut c_void) -> Status {
    // SAFETY: ACPICA passes the session's provider.
    let provider = unsafe { provider(context) };
    provider.closed.set(true);
    if let Some(armed) = provider.sci.borrow_mut().as_mut()
        && armed.retire().is_err()
    {
        return AE_ERROR;
    }
    // SAFETY: as above.
    unsafe { drain_work(context) }
}

/// Immediate eligibility only: this checks the real current continuation and
/// reserved firmware-role contract before C removes any callback. The current
/// legacy constructor refuses; it does not acquire permission from an empty FIFO.
unsafe extern "C" fn preflight_wait_events(context: *mut c_void) -> Status {
    // SAFETY: Session keeps this provider and its affine executor.
    match unsafe { provider(context) }.executor.check_firmware_wait() {
        Ok(()) => AE_OK,
        Err(_) => AE_SUPPORT,
    }
}

unsafe extern "C" fn destroy(context: *mut c_void) {
    // SAFETY: the session's provider, which `HostAcpi::start` leaked to it.
    drop(unsafe { Box::from_raw(context.cast::<Provider>()) });
}

unsafe extern "C" fn fatal(_: *mut c_void, status: Status) -> ! {
    output::fatal(format_args!(
        "ACPI callback completion failed: ACPICA status {status:#x}"
    ))
}

/// The firmware's ACPI namespace, for the host's power transitions.
/// One namespace serves the whole program: [`HostAcpi::start`] leaks it, and
/// the host drivers and power transitions borrow it for `'static`.
pub struct HostAcpi {
    session: Session,
    provider: *const Provider,
    ec: RefCell<Option<stela_drivers::platform::ec::binding::Binding<'static>>>,
    installed_ec: RefCell<Option<stela_drivers::platform::ec::capability::Installed<'static>>>,
    power: RefCell<stela_drivers::probe::Power>,
    notification: RefCell<Option<stela_drivers::platform::acpi::Notification<'static>>>,
    ec_pending: Cell<bool>,
    hardware: AtomicBool,
    ec_failed: AtomicBool,
    deferred_failed: AtomicBool,
}
static EC_EVENT: AtomicBool = AtomicBool::new(false);
static POWER_CHANGED: AtomicBool = AtomicBool::new(false);
static POWER_BUTTON: AtomicBool = AtomicBool::new(false);

impl HostAcpi {
    /// Load the tables at `rsdp` and their namespace; `None`, reported,
    /// when ACPICA refuses them. `root` is the loader root. The namespace
    /// lives until the host powers off, so it is leaked here, once.
    pub(super) fn start(
        rsdp: u64,
        root: u64,
        mut ecam: Vec<EcamWindow>,
        pci: Option<&'static native_mmio::Source>,
        tsc_hz: u64,
        prepared_sci: Option<sci::Prepared>,
        executor: Option<&'static KernelWork>,
    ) -> Option<&'static Self> {
        let Some(executor) = executor else {
            output::arguments(format_args!(
                "ACPI firmware AML unavailable: native continuation owner is unavailable\n"
            ));
            return None;
        };
        let pci = if let Some(source) = pci {
            match source.configuration_windows() {
                Ok(windows) if windows.is_empty() => None,
                Ok(windows) => {
                    ecam = windows;
                    Some(source)
                }
                Err(error) => {
                    output::arguments(format_args!(
                        "ACPI firmware AML unavailable: native PCI configuration inventory refused ({error:?})\n"
                    ));
                    return None;
                }
            }
        } else {
            None
        };
        match executor.call_owned(5_000_000, move || {
            Self::start_entered(rsdp, root, ecam, pci, tsc_hz, prepared_sci, executor)
        }) {
            Ok(acpi) => acpi,
            Err(error) => {
                output::arguments(format_args!(
                    "ACPI firmware AML admission failed: {error}\n"
                ));
                None
            }
        }
    }
    /// Called only by the owned foreground callback under its kept
    /// loader-root/host-driver guard. It must never acquire that guard again.
    fn start_entered(
        rsdp: u64,
        root: u64,
        ecam: Vec<EcamWindow>,
        pci: Option<&'static native_mmio::Source>,
        tsc_hz: u64,
        prepared_sci: Option<sci::Prepared>,
        executor: &'static KernelWork,
    ) -> Option<&'static Self> {
        if rsdp == 0 {
            output::arguments(format_args!("ACPI firmware AML unavailable: no RSDP\n"));
            return None;
        }
        // An SCI route alone does not check ACPICA hardware mode. The draft
        // provider still lacks real role-specific continuations/thread IDs and
        // exact resource-owned map/unmap leases. Keep namespace-only operation
        // until that complete provider is admitted by the work owner.
        let capabilities = 0;
        // Controller containment can prepare the route while ACPICA stays
        // in namespace mode. No hardware callback may take it in that mode.
        let prepared_sci = if capabilities == 0 {
            None
        } else {
            prepared_sci
        };
        let provider = Box::new(Provider {
            executor,
            root,
            ecam,
            pci,
            tsc_hz,
            work: RefCell::new(VecDeque::new()),
            queued: AtomicBool::new(false),
            closed: Cell::new(false),
            reported: Cell::new(0),
            prepared_sci: RefCell::new(prepared_sci),
            sci: RefCell::new(None),
            polling: Cell::new(false),
            needs_rearm: Cell::new(false),
        });
        let provider = Box::into_raw(provider);
        let services = Services {
            abi_version: SERVICES_ABI_VERSION,
            abi_bytes: SERVICES_ABI_BYTES,
            context: provider.cast(),
            root: rsdp,
            capabilities,
            allocate,
            free,
            map,
            unmap,
            create_lock,
            delete_lock,
            acquire_lock,
            release_lock,
            create_semaphore,
            delete_semaphore,
            wait_semaphore,
            signal_semaphore,
            install_interrupt,
            remove_interrupt,
            thread_id,
            execute,
            drain_work,
            sleep,
            stall,
            read_port,
            write_port,
            read_memory,
            write_memory,
            read_pci,
            write_pci,
            timer,
            signal,
            log,
            flush_cache,
            quiesce,
            destroy,
            fatal,
            preflight_wait_events,
        };
        // SAFETY: every callback keeps the OSL contract above, the provider
        // is the session's, and only this processor runs ACPICA.
        let session = match unsafe { Session::start(services, Mode::InterpreterOnly) } {
            Ok(session) => session,
            Err(error) => {
                output::arguments(format_args!(
                    "ACPI firmware AML refused: ACPICA status {:#x}\n",
                    error.0
                ));
                return None;
            }
        };
        let s5 = session
            .handle("\\_S5")
            .and_then(|s5| s5.evaluate(None, &[]));
        match s5 {
            Ok(Some(Value::Package(values))) => {
                let values: Vec<_> = values
                    .iter()
                    .map(|value| match value {
                        Value::Integer(value) => alloc::format!("{value:#x}"),
                        _ => String::from("?"),
                    })
                    .collect();
                output::arguments(format_args!(
                    "ACPI firmware AML loaded: \\_S5 {}\n",
                    values.join(",")
                ));
            }
            Ok(_) => output::arguments(format_args!(
                "ACPI firmware AML loaded: \\_S5 is not a package\n"
            )),
            Err(error) => output::arguments(format_args!(
                "ACPI firmware AML loaded: no \\_S5 (ACPICA status {:#x})\n",
                error.0
            )),
        }
        let host = Box::leak(Box::new(Self {
            session,
            provider,
            ec: RefCell::new(None),
            installed_ec: RefCell::new(None),
            power: RefCell::new(Default::default()),
            notification: RefCell::new(None),
            ec_pending: Cell::new(false),
            hardware: AtomicBool::new(false),
            ec_failed: AtomicBool::new(false),
            deferred_failed: AtomicBool::new(false),
        }));
        if capabilities != 0 {
            // SAFETY: the boot keeps the provider's exclusive EC resources
            // before namespace initialization and through this permanent owner.
            let installation =
                unsafe { stela_drivers::probe::install_embedded_controller(&host.session) };
            host.ec_failed
                .store(!installation.failures.is_empty(), Ordering::Release);
            for (location, error) in installation.failures {
                output::arguments(format_args!("HOST ACPI EC {location}: {error}\n"));
            }
            *host.ec.borrow_mut() = installation.binding;
            match host.session.enable_hardware() {
                Ok(()) => {
                    host.hardware.store(true, Ordering::Release);
                    if let Some(binding) = host.ec.borrow().as_ref() {
                        match stela_drivers::platform::ec::capability::Installed::from_binding(
                            binding,
                        ) {
                            Ok(installed) => *host.installed_ec.borrow_mut() = Some(installed),
                            Err(error) => output::arguments(format_args!(
                                "HOST ACPI EC activation refused: {error:?}\n"
                            )),
                        }
                        if let Some(gpe) = binding.device().gpe
                            && let Err(error) = host.session.flag_gpe(gpe, true, &EC_EVENT)
                        {
                            output::arguments(format_args!(
                                "HOST ACPI EC GPE refused: {error:?}\n"
                            ));
                        }
                    }
                    if let Err(error) = host.session.flag_fixed_event(
                        stela_drivers::platform::acpi::EVENT_POWER_BUTTON,
                        &POWER_BUTTON,
                    ) {
                        output::arguments(format_args!(
                            "HOST ACPI power button refused: {error:?}\n"
                        ));
                    }
                    match host
                        .session
                        .notify_all(|_, _| POWER_CHANGED.store(true, Ordering::Release))
                    {
                        Ok(notification) => *host.notification.borrow_mut() = Some(notification),
                        Err(error) => {
                            output::arguments(format_args!("HOST ACPI notify refused: {error:?}\n"))
                        }
                    }
                    if let Err(error) = host.session.update_all_gpes() {
                        output::arguments(format_args!(
                            "HOST ACPI GPE update refused: {error:?}\n"
                        ));
                    }
                    output::arguments(format_args!(
                        "HOST ACPI hardware active: routed SCI and deferred events\n"
                    ));
                }
                Err(error) => output::arguments(format_args!(
                    "UNSUPPORTED ACPI hardware activation: {error:?}\n"
                )),
            }
        } else {
            output::arguments(format_args!(
                "UNSUPPORTED ACPI hardware activation: native firmware services are not qualified\n"
            ));
        }
        Some(host)
    }

    pub(super) fn embedded_controller(
        &self,
    ) -> Result<
        Option<stela_drivers::platform::ec::capability::Installed<'static>>,
        stela_drivers::probe::ServiceError,
    > {
        if let Some(installed) = self.installed_ec.borrow().clone() {
            return Ok(Some(installed));
        }
        if self.ec.borrow().is_some()
            || self.ec_failed.load(Ordering::Acquire)
            || !self.hardware_active()
        {
            return Err(stela_drivers::probe::ServiceError::Unavailable);
        }
        Ok(None)
    }
    /// `bytes` of firmware RAM at `physical` through the mapping AML's
    /// SystemMemory regions reach it by, kept for the program as theirs are.
    pub(super) fn firmware_memory(&self, physical: u64, bytes: u64) -> Option<NonNull<u8>> {
        // SAFETY: this permanent provider is held by the leaked Session.
        let provider = unsafe { &*self.provider };
        provider.reach(physical, bytes).and_then(NonNull::new)
    }
    pub(super) fn retain_power(&self, power: stela_drivers::probe::Power) {
        *self.power.borrow_mut() = power;
    }
    pub(super) fn hardware_active(&self) -> bool {
        self.hardware.load(Ordering::Acquire)
    }
    pub(super) fn take_power_button(&self) -> bool {
        POWER_BUTTON.swap(false, Ordering::AcqRel)
    }

    pub(super) fn poll(&'static self) {
        // SAFETY: this permanent provider is held by the leaked Session.
        let provider = unsafe { &*self.provider };
        // These atomic observations do not borrow AML/provider mutable state
        // while an uncertain continuation might remain held in quarantine.
        if self.deferred_failed.load(Ordering::Acquire)
            || (!self.hardware_active()
                && !provider.queued.load(Ordering::Acquire)
                && !POWER_CHANGED.load(Ordering::Acquire)
                && !EC_EVENT.load(Ordering::Acquire))
        {
            return;
        }
        if let Err(error) = provider
            .executor
            .call_owned(5_000_000, move || self.poll_entered())
            && !self.deferred_failed.swap(true, Ordering::AcqRel)
        {
            output::arguments(format_args!(
                "UNSUPPORTED ACPI deferred work admission failed: {error}\n"
            ));
        }
    }
    fn poll_entered(&self) {
        // SAFETY: the work dispatcher owns the loader root and driver
        // exclusion across this continuation; the Session keeps the provider.
        let provider = unsafe { &*self.provider };
        provider.dispatch_sci(true);
        if EC_EVENT.swap(false, Ordering::AcqRel) || self.ec_pending.get() {
            let bindings = self.ec.borrow();
            if let Some(binding) = bindings.as_ref() {
                self.ec_pending.set(true);
                for _ in 0..32 {
                    match binding.poll_query() {
                        Ok(Some(event)) => {
                            if let Err(error) = event.dispatch() {
                                output::arguments(format_args!(
                                    "HOST ACPI EC query failed: {error:?}\n"
                                ));
                            }
                        }
                        Ok(None) => {
                            if let Some(number) = binding.device().gpe {
                                let gpe = stela_drivers::platform::acpi::Gpe {
                                    block: None,
                                    number,
                                };
                                if let Err(error) = self.session.enable_gpe(gpe) {
                                    output::arguments(format_args!(
                                        "HOST ACPI EC GPE resume failed: {error:?}\n"
                                    ));
                                    return;
                                }
                            }
                            self.ec_pending.set(false);
                            break;
                        }
                        Err(error) => {
                            output::arguments(format_args!(
                                "HOST ACPI EC transaction failed: {error:?}\n"
                            ));
                            return;
                        }
                    }
                }
            }
        }
        if POWER_CHANGED.swap(false, Ordering::AcqRel) {
            let mut power = self.power.borrow_mut();
            for battery in &mut power.batteries {
                if let Err(error) = battery.refresh() {
                    output::arguments(format_args!(
                        "HOST ACPI battery refresh failed: {error:?}\n"
                    ));
                }
            }
            if let Some(ac) = &mut power.ac
                && let Err(error) = ac.refresh()
            {
                output::arguments(format_args!("HOST ACPI AC refresh failed: {error:?}\n"));
            }
            if let Some(lid) = &mut power.lid
                && let Err(error) = lid.refresh()
            {
                output::arguments(format_args!("HOST ACPI lid refresh failed: {error:?}\n"));
            }
        }
        self.service_ucsi();
        // SAFETY: run Notify/_Q work queued by device methods on the BSP.
        // A budgeted EC continuation owns only its disabled GPE, allowing
        // unrelated fixed events and GPEs to keep making SCI progress.
        if unsafe { drain_work(self.provider.cast_mut().cast()) } != AE_OK {
            output::fatal("ACPI deferred work completion failed");
        }
    }

    /// Linux `ucsi_acpi_notify`'s work: each running UCSI manager whose
    /// Notify arrived reads and acknowledges its connector changes. A manager
    /// that faults is reported once and left stopped.
    fn service_ucsi(&self) {
        let mut power = self.power.borrow_mut();
        for (index, ucsi) in power.ucsi.iter_mut().enumerate() {
            if ucsi.state() != ucsi::State::Running || !ucsi.needs_service() {
                continue;
            }
            match ucsi.service(UCSI_SERVICE_BUDGET) {
                Ok(report) => {
                    for change in &report.changes {
                        output::arguments(format_args!(
                            "HOST USB-C {index} connector {}: {}\n",
                            change.number.get(),
                            change.status
                        ));
                    }
                    if let Some(error) = report.error {
                        output::arguments(format_args!(
                            "HOST USB-C {index} connector change failed: {error:?}\n"
                        ));
                    }
                }
                Err(error) => output::arguments(format_args!(
                    "HOST USB-C {index} manager stopped: {error:?}\n"
                )),
            }
        }
    }

    /// The firmware's processor idle declaration, once the processors are
    /// told `capabilities`; `None`, reported, when AML refuses it.
    pub(super) fn processor_idle(
        &'static self,
        capabilities: u32,
    ) -> Option<stela_drivers::platform::acpi::processor::Declaration> {
        // SAFETY: this permanent provider is held by the leaked Session.
        let provider = unsafe { &*self.provider };
        match provider
            .executor
            .call_owned(5_000_000, move || self.session.processor_idle(capabilities))
        {
            Ok(Ok(declaration)) => Some(declaration),
            Ok(Err(error)) => {
                output::arguments(format_args!(
                    "UNSUPPORTED ACPI processor idle declaration: ACPICA status {:#x}\n",
                    error.0
                ));
                None
            }
            Err(error) => {
                output::arguments(format_args!(
                    "ACPI processor idle continuation unavailable: {error}\n"
                ));
                None
            }
        }
    }

    /// The loaded namespace. Its calls run in the loader root
    /// ([`super::storage::RestoreRoot`]), as the host drivers do.
    pub fn session(&'static self) -> &'static Session {
        &self.session
    }

    /// Linux's `acpi_power_off_prepare`: S5's SLP_TYPa and SLP_TYPb as the
    /// firmware's `\_S5` evaluates, after `\_PTS(5)` and `\_SST`; `None`
    /// without S5. A failed `\_PTS` is reported, and S5 follows all the same.
    pub fn prepare_soft_off(&'static self) -> Option<(u8, u8)> {
        // SAFETY: this permanent provider is held by the leaked Session.
        let provider = unsafe { &*self.provider };
        match provider
            .executor
            .call_owned(5_000_000, move || self.prepare_soft_off_entered())
        {
            Ok(types) => types,
            Err(error) => {
                output::arguments(format_args!(
                    "POWER firmware continuation unavailable: {error}\n"
                ));
                None
            }
        }
    }
    fn prepare_soft_off_entered(&self) -> Option<(u8, u8)> {
        let types = match self.session.sleep_types(5) {
            Ok(types) => types,
            Err(error) => {
                output::arguments(format_args!(
                    "POWER the firmware's AML has no S5: ACPICA status {:#x}\n",
                    error.0
                ));
                return None;
            }
        };
        match self.session.prepare_sleep(5) {
            Ok(()) => output::arguments(format_args!(
                "POWER the firmware prepared S5: SLP_TYPa={:#x} SLP_TYPb={:#x}\n",
                types.0, types.1
            )),
            Err(error) => output::arguments(format_args!(
                "POWER \\_PTS(5) failed: ACPICA status {:#x}\n",
                error.0
            )),
        }
        Some(types)
    }
}
