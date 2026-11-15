//! The boot processor's device interrupts: physical MSI and owned I/O APIC
//! line routes through the engine's IDT vectors 0x50..=0x5f and its local
//! APIC. [`start`] attaches the registry once the host is installed; the
//! host drivers request routes through [`service`], and the engine takes
//! their interrupts in its wait windows. Interrupt remapping is off.
use crate::apic::{self, Apic};
use alloc::{sync::Arc, vec::Vec};
use core::{
    arch::asm,
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering},
};
use stela_drivers::bus::irq::{
    FIRST_VECTOR, IrqController, IrqError, Platform, Registry, Target, Trigger, VECTOR_COUNT,
};

use stela_boot_core::irq_context as context;
pub(crate) mod direct;
pub use stela_boot_core::ioapic;
pub mod madt;

static CONTEXT: context::Context = context::Context::new();
static READY: AtomicBool = AtomicBool::new(false);
static BASE: AtomicU64 = AtomicU64::new(0);
static REGISTERS: AtomicU64 = AtomicU64::new(0);
static BSP: AtomicU32 = AtomicU32::new(0);
static TIMEOUT: AtomicU64 = AtomicU64::new(0);
static MOUNTED_CPU: AtomicPtr<crate::engine::host::RootIdentity> =
    AtomicPtr::new(core::ptr::null_mut());

fn mounted_cpu() -> Option<&'static crate::engine::host::RootIdentity> {
    let identity = MOUNTED_CPU.load(Ordering::Acquire);
    // SAFETY: mounted start permanently retains this original Arc for the
    // static registry and every action that may remain in quarantine.
    unsafe { identity.as_ref() }
}

/// Check this CPU against the mounted identity.
fn check_irq_cpu() -> Result<(), IrqError> {
    mounted_cpu().map_or(Ok(()), |identity| {
        identity.check_irq_cpu().map_err(|_| IrqError::WrongCpu)
    })
}

/// # Safety
/// The slice retains every original controller under exclusive ownership.
/// The caller validated its complete MADT inventory, never routes MADT NMI sources
/// and keeps old destinations/contributors through independent retirement.
pub(crate) unsafe fn plan_firmware_lines<R: ioapic::Registers>(
    units: &[(madt::Unit, Arc<ioapic::Unit<R, NativePlatform>>)],
) -> Result<Vec<ioapic::FirmwareRoutes<R, NativePlatform>>, IrqError> {
    let mut routes = Vec::new();
    routes
        .try_reserve_exact(units.len())
        .map_err(|_| IrqError::Exhausted)?;
    for (descriptor, unit) in units {
        // SAFETY: complete original controller handoff, NMI sources unrouted,
        // and direct delivery, as required by the caller.
        routes.push(unsafe { unit.firmware_routes() }.map_err(|error| {
            crate::output::arguments(format_args!(
                "HOST IOAPIC plan refused id={} base={:#x} gsi={}: {error:?}\n",
                descriptor.id, descriptor.base, descriptor.gsi_base
            ));
            error
        })?);
    }
    Ok(routes)
}

/// # Safety
/// Same original complete controller ownership and source exclusions used
/// for planning. The caller retains all plans across a refusal or unwind.
pub(crate) unsafe fn contain_firmware_lines<R: ioapic::Registers>(
    routes: &[ioapic::FirmwareRoutes<R, NativePlatform>],
) -> Result<(), IrqError> {
    for (index, plan) in routes.iter().enumerate() {
        // SAFETY: forwarded original controller and source exclusions.
        unsafe { plan.contain() }.map_err(|error| {
            crate::output::arguments(format_args!(
                "HOST IOAPIC contain refused controller={index}: {error:?}\n"
            ));
            error
        })?;
    }
    for (index, plan) in routes.iter().enumerate() {
        plan.verify_contained().map_err(|error| {
            crate::output::arguments(format_args!(
                "HOST IOAPIC verify refused controller={index}: {error:?}\n"
            ));
            error
        })?;
    }
    Ok(())
}

pub struct NativePlatform(());
impl NativePlatform {
    pub(crate) fn current() -> Result<Self, IrqError> {
        let platform = Self(());
        platform.check_cpu()?;
        let spurious = controller().read(apic::SPURIOUS);
        if spurious == u32::MAX {
            return Err(IrqError::FailedRead);
        }
        if spurious & (1 << 12) != 0 {
            return Err(IrqError::Unsupported);
        }
        Ok(platform)
    }
}
pub static CONTROLLER: Registry<NativePlatform> = Registry::new(NativePlatform(()));

pub fn service() -> Result<&'static dyn IrqController, IrqError> {
    NativePlatform(()).check_cpu()?;
    Ok(&CONTROLLER)
}

/// Attach the registry to the installed boot processor's local APIC, which
/// `registers` maps in every root, with a two-second drain bound.
///
/// # Safety
/// The boot processor after `host::install`, with interrupts off; its IDT
/// routes 0x50..=0x5f to [`dispatch`] and nothing else owns those vectors.
pub(crate) unsafe fn start(
    registers: u64,
    tsc_hz: u64,
    identity: Arc<crate::engine::host::RootIdentity>,
) -> Result<(), IrqError> {
    identity.check_current().map_err(|_| IrqError::WrongCpu)?;
    let timeout = tsc_hz.checked_mul(2).ok_or(IrqError::Unsupported)?;
    // SAFETY: forwarded from the caller; the original APIC page is mapped in
    // every root. Interrupts remain disabled through identity publication.
    unsafe { initialize(apic::base_msr(), registers, timeout) }?;
    MOUNTED_CPU.store(Arc::into_raw(identity).cast_mut(), Ordering::Release);
    Ok(())
}

/// Whether this processor is running a device interrupt's handler.
pub(crate) fn in_device_dispatch() -> bool {
    matches!(current_context(), Ok(Some(_)))
}

fn controller() -> Apic {
    Apic::new(
        BASE.load(Ordering::Relaxed),
        REGISTERS.load(Ordering::Relaxed),
    )
}
fn id(apic: &Apic) -> Result<u32, IrqError> {
    let raw = apic.read(0x20);
    if raw == u32::MAX {
        return Err(IrqError::FailedRead);
    }
    Ok(if apic.is_x2apic() { raw } else { raw >> 24 })
}

fn in_service_vector(apic: &Apic) -> Result<Option<u8>, IrqError> {
    for bank in (0..8u32).rev() {
        let bits = apic.read(0x100 + bank * 16);
        if bits == u32::MAX {
            return Err(IrqError::FailedRead);
        }
        if bits != 0 {
            return Ok(Some((bank * 32 + 31 - bits.leading_zeros()) as u8));
        }
    }
    Ok(None)
}

fn pending_bank(apic: &Apic, bank: u32) -> Result<u32, IrqError> {
    let irr = apic.read(0x200 + bank * 16);
    let isr = apic.read(0x100 + bank * 16);
    if irr == u32::MAX || isr == u32::MAX {
        return Err(IrqError::FailedRead);
    }
    Ok(irr | isr)
}

/// Attach the existing, already enabled APIC and installed device-vector IDT.
///
/// # Safety
/// One BSP only, with interrupts disabled, no migration/SMM/other owner of the
/// reserved vectors or APIC, and the existing IDT's 0x50..0x5f stubs installed.
/// The xAPIC UC mapping is root-stable for the permanent registry lifetime
/// (including quarantine); x2APIC mode uses the existing APIC MSR adapter.
/// Interrupt remapping is off and physical MSI is checked by the platform.
pub unsafe fn initialize(base: u64, registers: u64, timeout_ticks: u64) -> Result<(), IrqError> {
    if READY.load(Ordering::Acquire) {
        return Err(IrqError::Unavailable);
    }
    if base & (1 << 11) == 0
        || base != apic::base_msr()
        || timeout_ticks == 0
        || timeout_ticks > u64::MAX / 2
    {
        return Err(IrqError::Unsupported);
    }
    let apic = Apic::new(base, registers);
    let bsp = id(&apic)?;
    let priority = apic.read(0x80);
    if priority == u32::MAX {
        return Err(IrqError::FailedRead);
    }
    if priority & 255 != 0 {
        return Err(IrqError::Unsupported);
    }
    // 0xff is the legacy physical broadcast destination, not a unicast CPU.
    if bsp >= 255 {
        return Err(IrqError::Unsupported);
    }
    // No stale firmware delivery may become the first interrupt of a
    // newly installed handler. Inspect the entire registry vector block.
    const FIRST_BIT: u8 = FIRST_VECTOR % 32;
    const _: () = assert!(VECTOR_COUNT < 32 && FIRST_BIT as usize + VECTOR_COUNT <= 32);
    const MASK: u32 = ((1u32 << VECTOR_COUNT) - 1) << FIRST_BIT;
    if pending_bank(&apic, u32::from(FIRST_VECTOR / 32))? & MASK != 0 {
        return Err(IrqError::Drain);
    }
    BASE.store(base, Ordering::Relaxed);
    REGISTERS.store(registers, Ordering::Relaxed);
    BSP.store(bsp, Ordering::Relaxed);
    TIMEOUT.store(timeout_ticks, Ordering::Relaxed);
    READY.store(true, Ordering::Release);
    Ok(())
}

/// Take the device interrupts pending on this CPU, whose code runs with them
/// off. The caller holds no lock a handler takes.
pub(crate) fn interrupt_window() {
    if NativePlatform(()).check_cpu().is_ok() {
        // SAFETY: on the owning CPU, with the caller's promise above.
        unsafe { NativePlatform(()).interrupt_window() };
    }
}

pub fn dispatch(vector: u8) -> bool {
    // SAFETY: Existing CPU entry calls this with local IRQs off. It does not supply
    // an interrupted-SP witness, so no such frame proof may be claimed.
    unsafe { dispatch_entry(vector, None) }
}

/// Device-vector dispatch with the unchanged IDT's hardware frame.
/// # Safety
/// Called only by the existing CPU interrupt handler with IRQs off;
/// `interrupted_rsp` is Frame.rsp from that entry, not the ISR's current RSP.
#[cfg(feature = "diagnostics")]
pub(crate) unsafe fn dispatch_frame(vector: u8, interrupted_rsp: usize) -> bool {
    // SAFETY: the caller passes the interrupted RSP from its own CPU entry frame.
    unsafe { dispatch_entry(vector, Some(interrupted_rsp)) }
}
unsafe fn dispatch_entry(vector: u8, interrupted_rsp: Option<usize>) -> bool {
    if !READY.load(Ordering::Acquire)
        || vector < FIRST_VECTOR
        || usize::from(vector - FIRST_VECTOR) >= VECTOR_COUNT
    {
        return false;
    }
    let cpu = NativePlatform(()).check_cpu();
    if cpu.is_err() {
        return false;
    }
    // SAFETY: the existing ISR entry and owning BSP were checked above;
    // IRQs stay disabled through this balanced callback/EOI scope.
    let _context = unsafe { CONTEXT.enter(cpu, interrupted_rsp) }
        .unwrap_or_else(|_| crate::output::fatal("native IRQ context depth exhausted"));
    CONTROLLER.dispatch(vector)
}
pub(crate) fn current_context() -> Result<Option<context::Snapshot>, IrqError> {
    CONTEXT.current(NativePlatform(()).check_cpu())
}

// SAFETY: initialization checks the existing APIC/IDT on one BSP,
// preserves its mapping forever, refuses migration and uses real IRQ-save,
// APIC pending/EOI and physical MSI operations. No CPU/IDT is allocated here.
unsafe impl Platform for NativePlatform {
    fn check_cpu(&self) -> Result<(), IrqError> {
        if !READY.load(Ordering::Acquire) {
            return Err(IrqError::Unavailable);
        }
        check_irq_cpu()?;
        if id(&controller())? != BSP.load(Ordering::Relaxed) {
            return Err(IrqError::WrongCpu);
        }
        Ok(())
    }
    fn check_process(&self) -> Result<(), IrqError> {
        self.check_cpu()?;
        if current_context()?.is_some() {
            return Err(IrqError::WrongCpu);
        }
        Ok(())
    }
    fn save_and_disable(&self) -> u64 {
        let saved;
        // SAFETY: native privileged CPU; memory clobber orders the critical
        // region, and exact IF state is restored on this same BSP.
        unsafe {
            asm!("pushfq", "pop {}", "cli", out(reg) saved);
        }
        saved
    }
    unsafe fn restore(&self, saved: u64) {
        if saved & (1 << 9) != 0 {
            // SAFETY: immediately enclosing saved state from the same BSP.
            unsafe {
                asm!("sti", options(nostack));
            }
        }
    }
    fn target(&self, vector: u8) -> Result<Target, IrqError> {
        self.check_cpu()?;
        // Physical destination, fixed delivery, edge trigger, one vector.
        Ok(Target {
            destination: BSP.load(Ordering::Relaxed) as u8,
            vector,
        })
    }
    fn pending(&self, vector: u8) -> Result<bool, IrqError> {
        self.check_cpu()?;
        let pending = pending_bank(&controller(), u32::from(vector / 32))?;
        Ok(pending & (1u32 << (vector % 32)) != 0)
    }
    fn eoi(&self, vector: u8) -> Result<Trigger, IrqError> {
        self.check_cpu()?;
        let apic = controller();
        // EOI removes the highest-priority ISR bit, not a caller-selected
        // vector. A process/debug dispatch or nested lower-vector call must
        // never acknowledge somebody else's still-active interrupt.
        if in_service_vector(&apic)? != Some(vector) {
            return Err(IrqError::Drain);
        }
        let tmr = apic.read(0x180 + u32::from(vector / 32) * 16);
        let spurious = apic.read(apic::SPURIOUS);
        // Read TMR before local EOI. Even a changed broadcast policy cannot
        // justify leaving this actual in-service interrupt unacknowledged.
        apic.eoi();
        if tmr == u32::MAX || spurious == u32::MAX {
            return Err(IrqError::FailedRead);
        }
        if spurious & (1 << 12) != 0 {
            return Err(IrqError::Unsupported);
        }
        Ok(if tmr & (1u32 << (vector % 32)) == 0 {
            Trigger::Edge
        } else {
            Trigger::Level
        })
    }
    fn ticks(&self) -> u64 {
        apic::rdtsc()
    }
    fn timeout_ticks(&self) -> u64 {
        TIMEOUT.load(Ordering::Relaxed)
    }
    unsafe fn interrupt_window(&self) {
        // SAFETY: caller holds no resource/registry lock. The NOP consumes
        // STI's interrupt shadow; pending IRQs run before the following CLI.
        unsafe {
            asm!("sti", "nop", "cli", options(nostack));
        }
    }
}
