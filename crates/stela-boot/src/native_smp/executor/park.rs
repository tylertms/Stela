//! The processors that run no PE, started once into an idle loop of their
//! own (`stela_boot_core::parking`): CR4.MCE from the startup trampoline,
//! the engine's descriptors, whose #MC and NMI gates stop the boot with the
//! fatal report, interrupts masked, and the deepest MWAIT its CPUID offers
//! or HLT. A parked processor does no host service, joins no shootdown and
//! takes no interrupt: nothing but NMI, SMI, INIT or a machine check ends
//! its wait.
//!
//! The parking fails safe. A processor that does not reach its loop within a
//! second, or faults on the way, may still read the startup page, so no
//! other processor starts from it: it and the processors after it stay in
//! wait-for-SIPI as before, the boot goes on and the log line says so.
use super::super::{bootstrap, firmware, memory_types::Image, topology};
use crate::{
    apic::{self, Apic},
    engine::interrupts,
    handoff::Handoff,
    layout,
};
use alloc::{boxed::Box, vec};
use core::{
    arch::{
        asm,
        x86_64::{__cpuid, CpuidResult},
    },
    cell::UnsafeCell,
    sync::atomic::{AtomicU8, AtomicU32, Ordering},
};
use stela_boot_core::parking::{Census, Idle, IdleCpuid, Left, UNPUBLISHED};
use stela_engine::machine::mmu::placement::Placement;

/// A parked processor's stack: its descriptor installation runs on it, then
/// the idle loop. Exceptions run on the descriptors' own stacks.
const STACK_BYTES: usize = 32 * 1024;

const STARTING: u8 = 0;
const PARKED: u8 = 1;

/// What one parked processor takes at entry. Leaked with its tables and
/// stack, which an NMI or machine check still enters for the whole boot.
struct Parking {
    tables: UnsafeCell<Option<&'static mut interrupts::Tables>>,
    /// The boot processor runs x2APIC, which this one then takes too.
    x2apic: bool,
    status: AtomicU8,
    /// Its wait (`Idle::encode`).
    idle: AtomicU32,
}

// SAFETY: the boot processor fills `tables` and `x2apic` before the SIPI;
// only the parked processor takes `tables`, once. The rest are atomics.
unsafe impl Sync for Parking {}

/// The line MONITOR arms. Nothing writes it.
#[repr(align(64))]
struct Line([u8; 64]);
static LINE: Line = Line([0; 64]);

/// Any exception on a parked processor stops the boot: it runs nothing a
/// fault could be resumed into.
extern "sysv64" fn interrupt(frame: &mut interrupts::Frame, _: &interrupts::SavedState<'_>) {
    interrupts::fatal(frame)
}

/// This processor's CPUID facts for its wait.
fn cpuid() -> IdleCpuid {
    let identity = __cpuid(0);
    let leaf1 = __cpuid(1);
    let leaf5 = match identity.eax >= 5 {
        true => __cpuid(5),
        false => CpuidResult {
            eax: 0,
            ebx: 0,
            ecx: 0,
            edx: 0,
        },
    };
    IdleCpuid {
        // "GenuineIntel".
        intel: (identity.ebx, identity.edx, identity.ecx)
            == (0x756e_6547, 0x4965_6e69, 0x6c65_746e),
        max_leaf: identity.eax,
        leaf1_ecx: leaf1.ecx,
        leaf1_edx: leaf1.edx,
        leaf5_ecx: leaf5.ecx,
        leaf5_edx: leaf5.edx,
    }
}

unsafe extern "sysv64" fn enter(argument: u64) -> ! {
    // SAFETY: the boot processor passed this processor one leaked Parking.
    let parking = unsafe { &*(argument as *const Parking) };
    // SAFETY: this processor alone takes its tables, once.
    let tables = unsafe {
        (&mut *parking.tables.get())
            .take()
            .expect("parked processor's descriptors")
    };
    // SAFETY: interrupts are masked since the trampoline; the tables are
    // this processor's and leaked, and nothing used the trampoline's own.
    unsafe { interrupts::install_on(tables, interrupt) };
    if parking.x2apic {
        // SAFETY: CPL0 with interrupts masked; this processor makes no APIC
        // handle.
        unsafe { apic::prefer_x2apic() };
    }
    let idle = stela_boot_core::parking::idle(cpuid());
    parking.idle.store(idle.encode(), Ordering::Relaxed);
    parking.status.store(PARKED, Ordering::Release);
    loop {
        match idle {
            Idle::Mwait { hint } => {
                // SAFETY: MONITOR arms a write-back line of the image that
                // nothing writes. MWAIT without interrupt break (ECX 0) and
                // with interrupts masked returns only for NMI, SMI, INIT or
                // a machine check, whose gates are installed.
                unsafe {
                    asm!("monitor", in("rax") LINE.0.as_ptr(), in("ecx") 0, in("edx") 0,
                        options(nostack));
                    asm!("mwait", in("eax") hint, in("ecx") 0, options(nostack));
                }
            }
            Idle::Halt => {
                // SAFETY: with interrupts masked HLT returns only for NMI,
                // SMI, INIT or a machine check, whose gates are installed.
                unsafe { asm!("hlt", options(nomem, nostack)) }
            }
        }
    }
}

/// Start every enabled processor that runs no PE into [`enter`], one at a
/// time from the startup page, and log the census
/// (`stela_boot_core::parking::Census`). Nothing is started after a PE's
/// processor failed (`startup_failed`), on a boot without a startup page,
/// or after a processor that did not park.
///
/// # Safety
/// As `executor::start`, after every PE's processor became ready: no
/// processor reads the startup page. `memory_types` and `pat` are the boot
/// processor's checked memory types.
pub(super) unsafe fn park(
    handoff: &Handoff,
    capture: &firmware::Capture,
    sender: &Apic,
    memory_types: &Image,
    pat: u64,
    startup_failed: bool,
) {
    let cores = handoff.native_resources.cores as usize;
    let processors = &capture.processors[..capture.processor_count as usize];
    let Ok(unused) = topology::enabled_unselected(processors, &capture.selected[..cores]) else {
        return;
    };
    let mut census = Census {
        unused: unused.clone().count(),
        ..Census::default()
    };
    census.left = match (startup_failed, handoff.native_resources.sipi) {
        (true, _) => Some(Left::StartupFailed),
        (false, 0) => Some(Left::NoStartupPage),
        _ => None,
    };
    if census.left.is_none() {
        for index in unused {
            let processor = processors[index];
            let Some(destination) = u32::try_from(processor.apic_id)
                .ok()
                .filter(|&id| sender.validate_init(id, handoff.tsc_hz).is_ok())
            else {
                census.unreachable += 1;
                continue;
            };
            // SAFETY: the caller's; the previous processor started from
            // the page parked, so none reads it.
            match unsafe {
                park_one(
                    handoff,
                    capture,
                    sender,
                    memory_types,
                    pat,
                    index,
                    destination,
                )
            } {
                Some(idle) => census.parked(idle),
                None => {
                    census.left = Some(Left::ParkFailed {
                        apic: processor.apic_id,
                    });
                    break;
                }
            }
        }
    }
    crate::output::arguments(format_args!("{census}\n"));
}

/// Start processor `index`, APIC ID `destination`, into [`enter`] and wait
/// a second at most for its wait; `None` when it did not park.
///
/// # Safety
/// As [`park`], and no processor reads the startup page.
unsafe fn park_one(
    handoff: &Handoff,
    capture: &firmware::Capture,
    sender: &Apic,
    memory_types: &Image,
    pat: u64,
    index: usize,
    destination: u32,
) -> Option<Idle> {
    let processor = capture.processors[index];
    // The trampoline checks an AMD processor's signature: its own where
    // the firmware's snapshot has one, else the boot processor's.
    let slot = capture.selected[..capture.core_count as usize]
        .iter()
        .position(|&selected| selected as usize == index)
        .unwrap_or(0);
    let signature = capture.snapshots[slot].get(1, 0)?.eax;
    let stack = Box::leak(vec![0u8; STACK_BYTES].into_boxed_slice());
    let stack_top = (stack.as_mut_ptr() as u64 + STACK_BYTES as u64) & !15;
    let parking: &'static Parking = Box::leak(Box::new(Parking {
        tables: UnsafeCell::new(Some(Box::leak(Box::new(interrupts::Tables::new())))),
        x2apic: handoff.apic_base & apic::X2APIC_MODE != 0,
        status: AtomicU8::new(STARTING),
        idle: AtomicU32::new(UNPUBLISHED),
    }));
    let physical = handoff.native_resources.sipi;
    crate::output::arguments(format_args!(
        "NATIVE PARK apic={destination:#x} firmware_cpu={}\n",
        processor.processor_number,
    ));
    // A reset during the startup leaves this line as the last.
    crate::output::flush_log();
    // SAFETY: the caller's: the page is owned and no processor reads it; the
    // stack, argument and tables are leaked heap in the shared physmap,
    // which `handoff.root` maps.
    let vector = unsafe {
        bootstrap::prepare(
            layout::physical(physical) as *mut u8,
            physical,
            bootstrap::Entry {
                processor_signature: signature,
                root: handoff.root,
                five_level: matches!(handoff.placement, Placement::FiveLevel),
                pat,
                memory_types,
                stack_top,
                argument: parking as *const Parking as u64,
                entry: enter,
            },
        )
    }
    .ok()?;
    // SAFETY: a validated destination, and complete startup resources that
    // stay for the whole boot.
    unsafe { sender.startup(destination, vector, handoff.tsc_hz) }.ok()?;
    let since = apic::rdtsc();
    loop {
        // SAFETY: the startup page stays this processor's until it parks.
        match unsafe { bootstrap::failure(layout::physical(physical) as *const u8) } {
            Ok(None) => {}
            _ => return None,
        }
        if parking.status.load(Ordering::Acquire) == PARKED {
            return Idle::decode(parking.idle.load(Ordering::Relaxed));
        }
        if apic::rdtsc().wrapping_sub(since) >= handoff.tsc_hz {
            return None;
        }
        core::hint::spin_loop();
    }
}
