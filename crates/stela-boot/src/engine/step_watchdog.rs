//! The Linux task step watchdog (`stela_boot_core::step_watchdog`) on the
//! boot processor, while it is the only processor running: the GPU's start
//! and every other Linux or foreground work segment before the other host
//! processors start, and from the other processors once they run. A core cycle counter, armed for each segment
//! ([`enter`]) and put back once a second at most by every step that
//! returns (`step_counts::WATCH`), overflows only when nothing returned for
//! the whole limit; the local APIC delivers that overflow as an NMI, whose
//! terminal handler stops the boot with the stuck step and the instruction
//! pointer it interrupted ([`nmi_reason`], `interrupts::fatal`), saved and
//! shown like every stop.
//!
//! In the normal path a step costs three relaxed stores and one compare
//! (`step_counts`), and a second of steps one counter write; a segment
//! costs a few model-specific register writes at its ends, before the other
//! processors start only. Once they run ([`retire`]) the counter stays off,
//! as `--profile` may then use it, and segments cost one load. The other
//! processors then watch the boot processor's steps ([`look`]): one relaxed
//! load between two of their guest slices while no step runs, and while
//! one does a clock read and a compare; a step they see run for the whole
//! limit gets the boot processor an NMI, the same terminal handler's stop.
//! Without another processor (every PE on the boot processor) nothing
//! watches then.
use super::host::{self, pmu};
use crate::output;
use core::sync::atomic::{AtomicBool, AtomicPtr, AtomicU8, AtomicU32, AtomicU64, Ordering};
use stela_boot_core::step_watchdog::{self as decide, Look, Observation, Sighting};
use stela_drivers::platform::work::step_counts::{RUNNING, WATCH};

/// Which counter overflows into the NMI.
const NONE: u8 = 0;
/// Intel architectural fixed counter 1, unhalted core cycles.
const INTEL_FIXED: u8 = 1;
/// AMD core counter 0, PMCx076 cycles not halted.
const AMD_CORE: u8 = 2;

static COUNTER: AtomicU8 = AtomicU8::new(NONE);
/// AMD performance monitoring version 2: a global enable gates counter 0.
static AMD_GLOBAL: AtomicBool = AtomicBool::new(false);
static START: AtomicU64 = AtomicU64::new(0);
static TSC_HZ: AtomicU64 = AtomicU64::new(0);
/// The processor whose local APIC delivers the NMI.
static PROCESSOR: AtomicU32 = AtomicU32::new(u32::MAX);
/// Segments entered and not yet left.
static DEPTH: AtomicU32 = AtomicU32::new(0);
static ARMED: AtomicBool = AtomicBool::new(false);
static ARMED_AT: AtomicU64 = AtomicU64::new(0);
/// The local APIC's performance counter entry before [`install`].
static SAVED_LVT: AtomicU32 = AtomicU32::new(0);

/// The boot processor's APIC ID, which the other processors' NMI goes to
/// once they run ([`retire`]); `u32::MAX` before.
static TARGET: AtomicU32 = AtomicU32::new(u32::MAX);
/// The step a watcher last saw (its start, 0 for none), when one first saw
/// it, by the shared count, and one look at a time.
static SEEN_BEGAN: AtomicU64 = AtomicU64::new(0);
static SEEN_AT: AtomicU64 = AtomicU64::new(0);
static LOOKING: AtomicBool = AtomicBool::new(false);
/// The start of the step a watcher sent the NMI for; 0 while none was.
static SENT: AtomicU64 = AtomicU64::new(0);

/// What the boot is doing, for the stop's reason.
static STAGE: AtomicPtr<&'static str> = AtomicPtr::new(core::ptr::null_mut());
static DRIVER_WORK: &str = "Linux driver work";

const LVT_PERFORMANCE: u32 = 0x340;
const LVT_NMI: u32 = 4 << 8;

const INTEL_FIXED_CTR1: u32 = 0x30a;
const INTEL_FIXED_CTR_CTRL: u32 = 0x38d;
const INTEL_GLOBAL_CTRL: u32 = 0x38f;
const INTEL_GLOBAL_OVF_CTRL: u32 = 0x390;
/// Fixed counter 1's control field: ring 0, and an interrupt on overflow.
const INTEL_FIXED1_FIELD: u64 = 0xf << 4;
const INTEL_FIXED1_OS_PMI: u64 = 0x9 << 4;
const INTEL_FIXED1_GLOBAL: u64 = 1 << 33;

const AMD_CTL0: u32 = 0xc001_0000;
const AMD_CTR0: u32 = 0xc001_0004;
const AMD_GLOBAL_CTL: u32 = 0xc000_0301;
const AMD_GLOBAL_STATUS_CLR: u32 = 0xc000_0302;
const AMD_CYCLES: u64 = 0x76;
const AMD_OS: u64 = 1 << 17;
const AMD_INT: u64 = 1 << 20;
const AMD_ENABLE: u64 = 1 << 22;

fn apic() -> crate::apic::Apic {
    let base = crate::apic::base_msr();
    crate::apic::Apic::new(
        base,
        crate::layout::physical(base & crate::apic::BASE_ADDRESS),
    )
}

/// The counter's select (AMD) without and with its overflow interrupt.
fn amd_select(interrupt: bool) -> u64 {
    AMD_CYCLES | AMD_OS | AMD_ENABLE | if interrupt { AMD_INT } else { 0 }
}

/// Find a free cycle counter this processor implements and counts with,
/// and its width, or why there is none.
fn probe() -> Result<(u8, u32), &'static str> {
    use core::arch::x86_64::{__cpuid, __cpuid_count};
    let vendor = __cpuid(0);
    let vendor = (vendor.ebx, vendor.edx, vendor.ecx);
    if vendor == (0x756e_6547, 0x4965_6e69, 0x6c65_746e) {
        if __cpuid(0).eax < 0xa {
            return Err("no Intel architectural performance monitoring");
        }
        let leaf = __cpuid_count(0xa, 0);
        let version = leaf.eax & 0xff;
        let fixed = leaf.edx & 0x1f;
        let width = (leaf.edx >> 5) & 0xff;
        if version < 2 || fixed < 2 {
            return Err("no Intel fixed core cycle counter");
        }
        let control = pmu::read(INTEL_FIXED_CTR_CTRL).ok_or("fixed counter control refused")?;
        if control & INTEL_FIXED1_FIELD != 0 {
            return Err("the fixed core cycle counter is in use");
        }
        Ok((INTEL_FIXED, width))
    } else if vendor == (0x6874_7541, 0x6974_6e65, 0x444d_4163) {
        let select = pmu::read(AMD_CTL0).ok_or("core counter 0 refused")?;
        if select & AMD_ENABLE != 0 {
            return Err("core counter 0 is in use");
        }
        let extended = __cpuid(0x8000_0000).eax;
        AMD_GLOBAL.store(
            extended >= 0x8000_0022 && __cpuid(0x8000_0022).eax & 1 != 0,
            Ordering::Relaxed,
        );
        Ok((AMD_CORE, 48))
    } else {
        Err("no known core cycle counter")
    }
}

/// Count, without an interrupt, from `start`.
fn count(counter: u8, start: u64) -> bool {
    match counter {
        INTEL_FIXED => {
            let Some(control) = pmu::read(INTEL_FIXED_CTR_CTRL) else {
                return false;
            };
            let Some(global) = pmu::read(INTEL_GLOBAL_CTRL) else {
                return false;
            };
            pmu::write(INTEL_FIXED_CTR1, start)
                && pmu::write(INTEL_GLOBAL_OVF_CTRL, INTEL_FIXED1_GLOBAL)
                && pmu::write(
                    INTEL_FIXED_CTR_CTRL,
                    control & !INTEL_FIXED1_FIELD | 0x1 << 4,
                )
                && pmu::write(INTEL_GLOBAL_CTRL, global | INTEL_FIXED1_GLOBAL)
        }
        AMD_CORE => {
            pmu::write(AMD_CTL0, 0)
                && pmu::write(AMD_CTR0, start)
                && (!AMD_GLOBAL.load(Ordering::Relaxed)
                    || pmu::read(AMD_GLOBAL_CTL)
                        .is_some_and(|global| pmu::write(AMD_GLOBAL_CTL, global | 1)))
                && pmu::write(AMD_CTL0, amd_select(false))
        }
        _ => false,
    }
}

fn counter_value(counter: u8) -> Option<u64> {
    pmu::read(match counter {
        INTEL_FIXED => INTEL_FIXED_CTR1,
        _ => AMD_CTR0,
    })
}

/// Stop the counter. Its interrupt goes with it.
fn stop(counter: u8) {
    match counter {
        INTEL_FIXED => {
            if let Some(global) = pmu::read(INTEL_GLOBAL_CTRL) {
                pmu::write(INTEL_GLOBAL_CTRL, global & !INTEL_FIXED1_GLOBAL);
            }
            if let Some(control) = pmu::read(INTEL_FIXED_CTR_CTRL) {
                pmu::write(INTEL_FIXED_CTR_CTRL, control & !INTEL_FIXED1_FIELD);
            }
        }
        AMD_CORE => {
            pmu::write(AMD_CTL0, 0);
        }
        _ => {}
    }
}

/// Make the counter, which counts, interrupt on overflow.
fn interrupt(counter: u8) -> bool {
    match counter {
        INTEL_FIXED => pmu::read(INTEL_FIXED_CTR_CTRL).is_some_and(|control| {
            pmu::write(
                INTEL_FIXED_CTR_CTRL,
                control & !INTEL_FIXED1_FIELD | INTEL_FIXED1_OS_PMI,
            )
        }),
        AMD_CORE => {
            (!AMD_GLOBAL.load(Ordering::Relaxed) || pmu::write(AMD_GLOBAL_STATUS_CLR, 1))
                && pmu::write(AMD_CTL0, amd_select(true))
        }
        _ => false,
    }
}

/// Find the counter on the boot processor, check that it counts, and route
/// its overflow to an NMI. Says once whether the watchdog runs.
///
/// # Safety
/// The boot processor in the engine's root, before the other processors
/// start, with nothing else using its performance counters or its local
/// APIC's performance counter entry.
pub(super) unsafe fn install(tsc_hz: u64) {
    TSC_HZ.store(tsc_hz, Ordering::Relaxed);
    let found = probe().and_then(|(counter, width)| {
        let start = decide::start_value(tsc_hz, width).ok_or("the limit does not fit its width")?;
        // A counter a hypervisor accepts but never advances raises nothing.
        if !count(counter, start) {
            stop(counter);
            return Err("the counter refused its setup");
        }
        let began = crate::apic::rdtsc();
        while crate::apic::rdtsc().wrapping_sub(began) < 100_000 {
            core::hint::spin_loop();
        }
        // It counted up from the whole start value: a counter that kept
        // only the low 32 bits of the write would overflow within a second.
        let counted =
            counter_value(counter).is_some_and(|value| value > start && value - start < 1 << 40);
        stop(counter);
        if !counted {
            return Err("the counter does not count from a full-width start");
        }
        Ok((counter, start))
    });
    match found {
        Ok((counter, start)) => {
            let apic = apic();
            SAVED_LVT.store(apic.read(LVT_PERFORMANCE), Ordering::Relaxed);
            apic.write(LVT_PERFORMANCE, LVT_NMI);
            START.store(start, Ordering::Relaxed);
            PROCESSOR.store(super::host::local::processor(), Ordering::Relaxed);
            COUNTER.store(counter, Ordering::Release);
            output::arguments(format_args!(
                "STEP WATCHDOG on: a Linux step that does not return within {} s stops the boot \
                 ({})\n",
                decide::LIMIT_SECONDS,
                if counter == INTEL_FIXED {
                    "Intel fixed counter 1"
                } else {
                    "AMD core counter 0"
                }
            ));
        }
        Err(why) => output::arguments(format_args!(
            "STEP WATCHDOG unavailable: {why}; a Linux step that never returns leaves no stop \
             record\n"
        )),
    }
}

/// The other processors start, on the boot processor: the counter goes off
/// for good, and the APIC entry back to what it was. From now on the other
/// processors watch its steps ([`look`]), with or without a counter.
pub(super) fn retire() {
    let counter = COUNTER.swap(NONE, Ordering::AcqRel);
    if counter != NONE {
        WATCH.disarm();
        ARMED.store(false, Ordering::Relaxed);
        stop(counter);
        apic().write(
            LVT_PERFORMANCE,
            SAVED_LVT.load(Ordering::Relaxed) | crate::apic::LVT_MASKED,
        );
    }
    // Only after `install`: without Linux's work there is nothing to watch.
    if TSC_HZ.load(Ordering::Relaxed) == 0 {
        return;
    }
    PROCESSOR.store(host::local::processor(), Ordering::Relaxed);
    TARGET.store(apic().id(), Ordering::Release);
    output::arguments(format_args!(
        "STEP WATCHDOG the other processors watch the boot processor's Linux steps: one that \
         does not return within {} s stops the boot\n",
        decide::LIMIT_SECONDS
    ));
}

/// Look at the boot processor's step, from another processor between two
/// of its guest slices: send the NMI once a step has run for the whole
/// limit by the looks' own count. Never to a boot processor that runs no
/// step, as one running guest code or parked does not.
pub(super) fn look() {
    let began = RUNNING.began.load(Ordering::Relaxed);
    if began == 0 {
        return;
    }
    let target = TARGET.load(Ordering::Acquire);
    if target == u32::MAX || LOOKING.swap(true, Ordering::Acquire) {
        return;
    }
    let last = match SEEN_BEGAN.load(Ordering::Relaxed) {
        0 => None,
        began => Some(Sighting {
            began,
            seen_at: SEEN_AT.load(Ordering::Relaxed),
        }),
    };
    match decide::look(last, began, host::fresh_counter(), host::COUNTER_HZ) {
        Look::New(sighting) => {
            SEEN_AT.store(sighting.seen_at, Ordering::Relaxed);
            SEEN_BEGAN.store(sighting.began, Ordering::Relaxed);
        }
        Look::Overdue { .. } => {
            // One NMI, for the first step found stuck: its handler stops
            // the boot. Nothing printed here: the stuck processor may hold
            // the output.
            if SENT
                .compare_exchange(0, began, Ordering::AcqRel, Ordering::Relaxed)
                .is_ok()
            {
                host::nmi(target);
            }
        }
        Look::Idle | Look::Running => {}
    }
    LOOKING.store(false, Ordering::Release);
}

/// Name what the boot is doing for a stop inside the segments that follow.
pub(super) fn stage(stage: &'static &'static str) {
    STAGE.store(core::ptr::from_ref(stage).cast_mut(), Ordering::Relaxed);
}

/// What the boot is doing is no longer named.
pub(super) fn clear_stage() {
    STAGE.store(core::ptr::null_mut(), Ordering::Relaxed);
}

/// Put the counter back: a step returned at `now`.
fn rearm(now: u64) {
    let counter = COUNTER.load(Ordering::Relaxed);
    if !ARMED.load(Ordering::Relaxed) || counter == NONE {
        WATCH.disarm();
        return;
    }
    let written = pmu::write(
        if counter == INTEL_FIXED {
            INTEL_FIXED_CTR1
        } else {
            AMD_CTR0
        },
        START.load(Ordering::Relaxed),
    );
    if written {
        ARMED_AT.store(now, Ordering::Relaxed);
    }
    WATCH.due_at(decide::next_rearm(now, TSC_HZ.load(Ordering::Relaxed)));
}

/// Progress a long operation reports: the counter goes back as after a
/// returned step, once a second at most.
pub(super) fn progress() {
    if ARMED.load(Ordering::Relaxed) {
        WATCH.returned(crate::apic::rdtsc());
    }
}

/// A Linux or foreground work segment begins on this processor; the
/// counter arms for the outermost.
pub(super) fn enter() {
    let counter = COUNTER.load(Ordering::Acquire);
    if counter == NONE
        || PROCESSOR.load(Ordering::Relaxed) != super::host::local::processor()
        || DEPTH.fetch_add(1, Ordering::Relaxed) != 0
    {
        return;
    }
    if count(counter, START.load(Ordering::Relaxed)) && interrupt(counter) {
        let now = crate::apic::rdtsc();
        ARMED_AT.store(now, Ordering::Relaxed);
        ARMED.store(true, Ordering::Relaxed);
        WATCH.arm(
            rearm,
            decide::next_rearm(now, TSC_HZ.load(Ordering::Relaxed)),
        );
    } else {
        stop(counter);
    }
}

/// The segment [`enter`] began ends; the outermost disarms the counter.
pub(super) fn leave() {
    let counter = COUNTER.load(Ordering::Acquire);
    if counter == NONE || PROCESSOR.load(Ordering::Relaxed) != super::host::local::processor() {
        return;
    }
    if DEPTH
        .try_update(Ordering::Relaxed, Ordering::Relaxed, |depth| {
            depth.checked_sub(1)
        })
        .is_ok_and(|depth| depth == 1)
    {
        WATCH.disarm();
        ARMED.store(false, Ordering::Relaxed);
        stop(counter);
    }
}

/// The stop's reason when an NMI on this processor is the watchdog's.
pub(super) fn nmi_reason() -> Option<alloc::string::String> {
    if PROCESSOR.load(Ordering::Relaxed) != super::host::local::processor() {
        return None;
    }
    let observation = Observation {
        armed: ARMED.load(Ordering::Relaxed),
        armed_at: ARMED_AT.load(Ordering::Relaxed),
        step_began: RUNNING.began.load(Ordering::Relaxed),
        task: RUNNING.task.load(Ordering::Relaxed),
        sequence: RUNNING.sequence.load(Ordering::Relaxed),
        now: crate::apic::rdtsc(),
        tsc_hz: TSC_HZ.load(Ordering::Relaxed),
    };
    let stage = STAGE.load(Ordering::Relaxed);
    // SAFETY: `stage` stores only references to statics.
    let stage = unsafe { stage.as_ref() }.copied().unwrap_or(DRIVER_WORK);
    if let Some(caught) = decide::caught(SENT.load(Ordering::Acquire), &observation) {
        return Some(decide::caught_message(stage, &caught));
    }
    let stuck = decide::attribute(&observation)?;
    Some(decide::message(stage, &stuck))
}
