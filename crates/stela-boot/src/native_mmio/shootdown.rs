//! The MMIO aperture's TLB shootdown: a retirement on one processor clears
//! the leaf entries, publishes a generation with the range it retired
//! ([`aperture::Processors::request`]), flushes itself, and sends
//! [`SHOOTDOWN_VECTOR`] to every other processor that joined. Each flushes
//! what the requests since its last flush retired ([`Processors::plan`]: their
//! pages in every PCID when they are few, else every translation) and
//! acknowledges the generation, in the vector's handler when it runs with
//! the vector open, or at its next safe point with interrupts off: each
//! native return, each engine wait and each turn of its executor's mailbox
//! loop ([`service`]). Native epochs end at their deadlines, which bounds how
//! long a processor runs without a safe point. A compile job, which runs
//! whole for up to seconds, runs with the vector open ([`job`]).
//!
//! [`SHOOTDOWN_VECTOR`]: crate::engine::interrupts::SHOOTDOWN_VECTOR
//! [`aperture::Processors::request`]: stela_boot_core::aperture::Processors::request
//! [`Processors::plan`]: stela_boot_core::aperture::Processors::plan
use super::PROCESSORS;
use crate::engine::interrupts::{NATIVE_WAKE_VECTOR, SHOOTDOWN_VECTOR, TIMER_VECTOR};
use core::{
    arch::{asm, x86_64::__cpuid_count},
    ops::Range,
    sync::atomic::{AtomicU8, AtomicU64, Ordering},
};
use stela_boot_core::aperture::{Doing, RANGE_PAGES};

/// How long a retirement waits before it reports the processors it waits
/// for.
const REPORT_MS: u64 = 100;
/// How long a retirement waits for every acknowledgement. A processor goes
/// this long without a safe point only when it is stuck: every engine wait
/// and engine or driver lock spin is one, a native epoch ends at its
/// deadline, and a compile job takes the vector.
const WAIT_MS: u64 = 30_000;
const CR4_PGE: u64 = 1 << 7;
const CR4_PCIDE: u64 = 1 << 17;
#[cfg(feature = "nvk")]
const RFLAGS_IF: u64 = 1 << 9;
const PAGE: u64 = 4096;
/// CPUID.(EAX=7,ECX=0):EBX.INVPCID.
const INVPCID: u32 = 1 << 10;
/// INVPCID type 0: one address in one context, global entries excepted.
const ONE_ADDRESS: u64 = 0;
/// INVPCID type 2: every context's translations, global ones included
/// (Intel SDM Vol. 2A, INVPCID).
const ALL_CONTEXTS: u64 = 2;
/// The task priority a compile job runs at: it blocks every class up to the
/// host devices' and the timer's, so only the doorbell and the shootdown
/// reach it (Intel SDM Vol. 3A 11.8.3.1; CR8 holds TPR bits 7:4).
const JOB_PRIORITY: u64 = (SHOOTDOWN_VECTOR as u64 >> 4) - 1;
const _: () = assert!(NATIVE_WAKE_VECTOR >> 4 == SHOOTDOWN_VECTOR >> 4);
// Host device interrupts use 0x50..=0x5f.
const _: () = assert!((TIMER_VECTOR >> 4) as u64 <= JOB_PRIORITY && 0x5f >> 4 <= JOB_PRIORITY);

/// Whether INVPCID is available: 0 unknown, 1 absent, 2 present.
static INVPCID_PRESENT: AtomicU8 = AtomicU8::new(0);

pub(crate) fn invpcid() -> bool {
    match INVPCID_PRESENT.load(Ordering::Relaxed) {
        0 => {
            let present = __cpuid_count(7, 0).ebx & INVPCID != 0;
            INVPCID_PRESENT.store(1 + u8::from(present), Ordering::Relaxed);
            present
        }
        known => known == 2,
    }
}

fn cr4() -> u64 {
    let cr4: u64;
    // SAFETY: CPL0 observation.
    unsafe { asm!("mov {}, cr4", out(reg) cr4, options(nomem, nostack, preserves_flags)) };
    cr4
}

/// Drop every aperture translation from this processor's TLB and paging
/// structure caches, in every PCID.
fn flush() {
    let cr4 = cr4();
    if cr4 & CR4_PCIDE == 0 {
        // Without PCIDs a CR3 write drops every non-global translation, and
        // the aperture's leaves are not global.
        // SAFETY: rewrites the live root.
        unsafe {
            asm!("mov {0}, cr3", "mov cr3, {0}", out(reg) _, options(nostack, preserves_flags))
        };
    } else if invpcid() {
        let descriptor = [0u64; 2];
        // SAFETY: a valid all-contexts INVPCID; the descriptor is ignored.
        unsafe {
            asm!("invpcid {}, [{}]", in(reg) ALL_CONTEXTS, in(reg) &descriptor,
                options(nostack, preserves_flags))
        };
    } else {
        // Any change to CR4.PGE drops every PCID's translations (Intel SDM
        // Vol. 3A 4.10.4.1, AMD APM Vol. 2 5.5.3).
        let saved: u64;
        // SAFETY: interrupts stay off across the toggle; CR4 is restored.
        unsafe {
            asm!("pushfq", "pop {}", "cli", out(reg) saved);
            asm!("mov cr4, {}", in(reg) cr4 ^ CR4_PGE, options(nostack, preserves_flags));
            asm!("mov cr4, {}", in(reg) cr4, options(nostack, preserves_flags));
            asm!("push {}", "popfq", in(reg) saved);
        }
    }
}

/// Drop the pages of `ranges` from this processor's TLB in every PCID a
/// root loads in (`host::CONTEXTS`); everything when that takes a CR4
/// toggle. The leaves are not global, and no paging-structure entry above
/// them changes, as the window and the aperture keep their tables.
fn flush_ranges(ranges: impl Iterator<Item = Range<u64>>) {
    let tagged = cr4() & CR4_PCIDE != 0;
    if tagged && !invpcid() {
        return flush();
    }
    for range in ranges {
        let mut page = range.start & !(PAGE - 1);
        while page < range.end {
            if tagged {
                for context in 0..crate::engine::host::CONTEXTS {
                    let descriptor = [context, page];
                    // SAFETY: a valid single-address INVPCID; TLB
                    // maintenance only.
                    unsafe {
                        asm!("invpcid {}, [{}]", in(reg) ONE_ADDRESS, in(reg) &descriptor,
                            options(nostack, preserves_flags))
                    };
                }
            } else {
                // SAFETY: TLB maintenance only; without PCIDs it covers
                // every root.
                unsafe { asm!("invlpg [{}]", in(reg) page, options(nostack, preserves_flags)) };
            }
            page += PAGE;
        }
    }
}

/// Flush and acknowledge the latest request if this processor joined and
/// has not flushed it yet. Cheap when nothing is requested: two loads.
pub(crate) fn service() {
    let requested = PROCESSORS.requested();
    if requested == 0 {
        return;
    }
    let (slot, flushed) = crate::engine::host::shootdown();
    let from = flushed.load(Ordering::Relaxed);
    if from >= requested {
        return;
    }
    let Some(slot) = slot else {
        return;
    };
    match PROCESSORS.plan(from, requested) {
        Some(ranges) => flush_ranges(ranges.iter()),
        None => flush(),
    }
    // A waiter may have IF set. An interrupt between the flush and this
    // publication flushed at least as far, and must not let its newer local
    // acknowledgement regress.
    flushed.fetch_max(requested, Ordering::Relaxed);
    PROCESSORS.acknowledge(slot, requested);
}

/// Note what this processor does until the returned guard drops.
fn doing(doing: Doing) -> Option<Restore> {
    let slot = crate::engine::host::shootdown().0?;
    let previous = PROCESSORS.set_doing(slot, doing, crate::engine::host::rdtsc());
    Some(Restore(slot, previous))
}

struct Restore(usize, Doing);

impl Drop for Restore {
    fn drop(&mut self) {
        PROCESSORS.set_doing(self.0, self.1, crate::engine::host::rdtsc());
    }
}

/// Runs a compile job with this processor's shootdown vector open, so a
/// retirement elsewhere is acknowledged within an interrupt's latency rather
/// than after the job: with interrupts already on it runs as it is; else
/// with the task priority raised to [`JOB_PRIORITY`], so that only the
/// doorbell and the shootdown, whose handlers take no lock, reach it.
///
/// # Safety
/// The caller holds nothing those two handlers take, and the job leaves
/// RFLAGS.IF and CR8 as it found them.
#[cfg(feature = "nvk")]
pub(crate) unsafe fn job<R>(body: impl FnOnce() -> R) -> R {
    let flags: u64;
    // SAFETY: reads RFLAGS through the stack, which the push and pop balance.
    unsafe { asm!("pushfq", "pop {}", out(reg) flags, options(preserves_flags)) };
    if flags & RFLAGS_IF != 0 {
        let _doing = doing(Doing::Job);
        return body();
    }
    let saved: u64;
    // SAFETY: CPL0 read of the task priority.
    unsafe { asm!("mov {}, cr8", out(reg) saved, options(nomem, nostack, preserves_flags)) };
    if saved > JOB_PRIORITY {
        let _doing = doing(Doing::MaskedJob);
        return body();
    }
    let _doing = doing(Doing::Job);
    // SAFETY: the priority blocks every vector but the doorbell's and the
    // shootdown's, whose handlers run on their own stack (IST) and take no
    // lock; the caller's contract covers the rest. Both are restored.
    unsafe {
        asm!("mov cr8, {}", "sti", in(reg) JOB_PRIORITY, options(nomem, nostack));
    }
    let result = body();
    // SAFETY: back to how the caller ran; an interrupt the priority held
    // stays pending until interrupts open again.
    unsafe {
        asm!("cli", "mov cr8, {}", in(reg) saved, options(nomem, nostack));
    }
    result
}

/// After the caller cleared the leaf entries of `retired` (all of them, or
/// `None` when they are not one range) and flushed its own TLB: shoot them
/// down on every other joined processor. Whether each acknowledged within
/// [`WAIT_MS`]; until then a processor may still translate through them. The
/// wait is itself a safe point, so processors retiring at once acknowledge
/// each other.
pub(super) fn retire(tsc_hz: u64, retired: Option<Range<u64>>) -> bool {
    if PROCESSORS.alone() {
        // The caller's own flush covers only its current PCID.
        match retired {
            Some(range) if range.end - range.start <= RANGE_PAGES * PAGE => {
                flush_ranges(core::iter::once(range));
            }
            _ => flush(),
        }
        return true;
    }
    let (me, _) = crate::engine::host::shootdown();
    if me.is_none() {
        return false;
    }
    let base = crate::apic::base_msr();
    let sender = crate::apic::Apic::new(
        base,
        crate::layout::physical(base & crate::apic::BASE_ADDRESS),
    );
    if retired
        .as_ref()
        .is_some_and(|range| range.end - range.start <= RANGE_PAGES * PAGE)
    {
        RANGED.fetch_add(1, Ordering::Relaxed);
    }
    let generation = PROCESSORS.request(retired);
    // This processor's other PCIDs, and anything others requested.
    service();
    // Counted from the first doorbell: under a hypervisor each ICR access
    // exits, so sending to every processor is a cost of its own.
    let start = crate::engine::host::rdtsc();
    let _counted = Counted(start);
    let _doing = doing(Doing::Retiring);
    for (slot, apic) in PROCESSORS.sharers() {
        if Some(slot) == me || PROCESSORS.acknowledged(slot) >= generation {
            continue;
        }
        // A processor halted or in a job with the vector open takes it and
        // sees the request stored above; one with interrupts off takes it
        // at its next safe point, whether or not this delivery completes.
        // SAFETY: every joined processor's IDT routes the vector to its
        // shootdown handler. No handler this processor may take while it
        // retires (the doorbell's, the shootdown's) writes the ICR.
        let _ = unsafe { sender.notify(apic, SHOOTDOWN_VECTOR, tsc_hz) };
    }
    let (report, limit) = (tsc_hz / 1000 * REPORT_MS, tsc_hz / 1000 * WAIT_MS);
    let mut reported = false;
    // A peer may be publishing a retirement at the same time and cannot
    // finish until this processor acknowledges its generation.
    while !PROCESSORS.flushed(generation, me) {
        service();
        let now = crate::engine::host::rdtsc();
        let waited = now.wrapping_sub(start);
        if waited >= limit {
            report_unacknowledged(generation, me, now, tsc_hz, "the mapping stays reserved");
            return false;
        }
        if !reported && waited >= report {
            report_unacknowledged(generation, me, now, tsc_hz, "still waiting");
            reported = true;
        }
        core::hint::spin_loop();
    }
    true
}

/// Shootdowns this processor's peers had to acknowledge since [`take_waits`],
/// and the TSC ticks their retirements waited for them.
static WAITS: [AtomicU64; 2] = [const { AtomicU64::new(0) }; 2];
/// Of those, the ones whose range peers could flush page by page.
static RANGED: AtomicU64 = AtomicU64::new(0);

/// Counts one retirement's wait when it ends, however it ends.
struct Counted(u64);

impl Drop for Counted {
    fn drop(&mut self) {
        WAITS[0].fetch_add(1, Ordering::Relaxed);
        WAITS[1].fetch_add(
            crate::engine::host::rdtsc().wrapping_sub(self.0),
            Ordering::Relaxed,
        );
    }
}

/// The shootdowns since the last call, the ticks they waited and how many
/// of them retired a range peers flushed page by page, for the GPU gate's
/// report.
#[cfg(feature = "nvk")]
pub(crate) fn take_waits() -> (u64, u64, u64) {
    (
        WAITS[0].swap(0, Ordering::Relaxed),
        WAITS[1].swap(0, Ordering::Relaxed),
        RANGED.swap(0, Ordering::Relaxed),
    )
}

/// The shootdowns since [`take_waits`] last cleared them, left counting: a
/// caller takes the difference across its own work.
#[cfg(feature = "nvk")]
pub(crate) fn count() -> u64 {
    WAITS[0].load(Ordering::Relaxed)
}

/// Names each processor that has not acknowledged `generation`, what it
/// does and for how long, and how many requests it is behind.
fn report_unacknowledged(generation: u64, me: Option<usize>, now: u64, tsc_hz: u64, outcome: &str) {
    for (slot, apic) in PROCESSORS.sharers() {
        let acknowledged = PROCESSORS.acknowledged(slot);
        if Some(slot) != me && acknowledged < generation {
            // A processor never marked has been masked since it joined.
            let (doing, since) = PROCESSORS.doing(slot);
            let held_ms = (since != 0).then(|| now.saturating_sub(since) / (tsc_hz / 1000).max(1));
            crate::output::arguments(format_args!(
                "NATIVE MMIO shootdown {generation} unacknowledged by APIC {apic:#x} \
                 doing={}/{}ms behind={}; {outcome}\n",
                doing.name(),
                Since(held_ms),
                generation - acknowledged,
            ));
        }
    }
}

/// Milliseconds, or `-` when not known.
struct Since(Option<u64>);

impl core::fmt::Display for Since {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            Some(ms) => write!(formatter, "{ms}"),
            None => formatter.write_str("-"),
        }
    }
}
