//! The system counter the PEs observe.
//!
//! **Parallel PEs** each run on a host core of their own and read one
//! [`SharedClock`], as the cores of a Mac read one system counter (Linux
//! KVM likewise keeps one counter offset for a whole arm64 VM, so its
//! vCPUs agree). Its count is the host's invariant counter, scaled to the
//! guest's frequency, plus one offset: guest time follows wall time on any
//! host, whatever any PE's speed, and every PE reads the same monotonic
//! count (Arm ARM D12.1.2) with no exit and no shared write.
//!
//! Earlier the count was paced by retired instructions: it advanced only as
//! far as the slowest running PE had worked at 250 MIPS, and stopped while a
//! PE in the kernel did host work. Guest time then ran at 0.07-0.45 of wall
//! time whenever one PE was slow (an AMX loop at 10 us an instruction, the
//! boot PE's GPU and display work). Its reasons, and how each is kept:
//!
//! - XNU times a lock holder from another core: a mutex interlock panics
//!   after 12,582,912 counts (`lock_mtx.c` 326, about 0.5 s at 24 MHz), a
//!   ticket spinlock after 3,000,000 (`lock_ticket.c`, 125 ms). A running
//!   PE, however slow, leaves a critical section in microseconds of host
//!   time; only a PE the host keeps from running can overrun them, for any
//!   reason: host work between slices or inside one (a compile, a disk
//!   step, a lock or another PE it waits for). So a PE whose guest has
//!   work and that ran no block for [`STALL_MILLIS`] of guest time holds
//!   the count there until it runs again, the rest becoming lag: a stopped
//!   holder looks at most that long stopped. Each PE publishes its count
//!   every few milliseconds of progress, in a slot of its own, and looks at
//!   the others' then (`Clock::mark`); one that waits looks at each pass.
//!   Only a PE that fell behind makes them write the shared state, so with
//!   no stall native reads share it unmodified. A PE that knows it stops
//!   for host work in the kernel (`Clock::leave`) sets its own limit.
//!   Normal host work takes a few milliseconds and never reaches it.
//! - A PE whose guest waits holds nothing, unless guest work comes for it
//!   while it is out of its run loop serving the host (`Machine::serve_host`):
//!   an interrupt or fast IPI another PE sends it, or its own timer coming
//!   due. Another core's XNU times that PE's answer (a cross-call's
//!   acknowledgement, a timer call queued on its CPU), so from that moment
//!   (the wake, or the timer's deadline) it counts as a PE with guest work
//!   that ran nothing: the count holds a stall limit later if it is still
//!   out (`SharedClock::pending`), and its host work ends at its next part
//!   (`GuestWatch`). As with every hold, the held time is lag the slew
//!   repays, so guest time comes back to wall time.
//! - A slow PE got more timer interrupts per unit of its work: the EL0
//!   release experiment counted 34 guest seconds
//!   where a paced count counted 16, with every PE in timer interrupts. A
//!   comparator signals once however late it is, and XNU expires every due
//!   timer call in one interrupt, so no flood follows a long stop; what can
//!   starve a slow PE is a stream of new deadlines. So after a timer
//!   interrupt a busy PE's timers wait until it has had the time to retire
//!   [`TIMER_WORK`] instructions at its rate, at most [`TIMER_DELAY_HZ`]⁻¹
//!   (`Clock::timer_taken`): a late tick, as on a slow core, for the PE that
//!   is slow, instead of a slow clock for all of them.
//! - Timeouts measured on a PE's own work (watchdogs, launchd's and XPC's)
//!   now see a slow PE as a slow core. That is the host's speed showing, as
//!   it does to a user, not a broken guarantee.
//!
//! **Serialized PEs** take turns on one host core. A time slice accounts
//! for at most [`TICKS_PER_BLOCK`] counts for each block it runs, shared
//! among the PEs that could run with it: over a round of slices every
//! runnable PE then gets a block done for each `TICKS_PER_BLOCK` counts, as
//! on a slow but concurrent machine. A slice that takes more host time is
//! charged as lag when it ends, and the counter holds at the last value
//! read until the adjusted host count catches up, so it never runs
//! backwards. While every PE waits nothing has to progress, and the counter
//! follows the host's. With idle skip (`Machine::set_idle_skip`), a wait of
//! every PE ends at once: the counter leaps to the next deadline, as if the
//! host had waited that long, so the guest's sleeps cost no host time. A
//! serialized PE that does not run holds no core, so its lock holders are
//! the reason this count stays paced.
//!
//! The architecture requires a constant counter frequency, not a match
//! with wall time. A serialized count's time of day drifts behind by the
//! accumulated lag, as on a machine whose counter was paused, and ahead by
//! any idle leaps. A shared count repays the lag its holds left: once no
//! hold is in force it runs [`SLEW_PARTS`]⁻¹ (1%) faster than the host's,
//! in steps of at most [`SLEW_STEP_PARTS`]⁻¹ of the stall limit, until it
//! is back on the host's count ([`SharedClock::look`]). A step only raises
//! the offset, so every reader's count stays monotonic, and waits and
//! native deadlines count the slew in, so no timer is late for it.

use alloc::boxed::Box;
use alloc::sync::Arc;
use core::sync::atomic::{AtomicU64, Ordering, fence};
use stela_compiler::x86_64::register_read::{Count, CountState};

/// The most counts one block may account for: one count at 24 MHz is
/// 42 ns, a PE at least ~1/12 the speed of an M2 core. XNU drivers time
/// work against fixed timeouts (IOUSBDeviceFamily's function registration,
/// IOKit start waits), so a slower apparent PE fails them although nothing
/// is wrong. The longest real critical section XNU guards with a timeout
/// runs far below 3,000,000 blocks, the ticket spinlock bound at this rate.
pub(super) const TICKS_PER_BLOCK: u64 = 1;

/// How long a PE with guest work may look stopped before the count waits
/// for it. XNU's shortest timeout another core can trip on a stopped one
/// is the ticket spinlock's 125 ms (`ml-timeout-ticket-lock`, 3,000,000
/// counts in 26B123); the mutex interlock's is 0.5 s. A waiter may have
/// started timing shortly before the holder stopped, and the holder needs
/// time to finish once it runs, so half of that leaves 75 ms of margin.
/// It is far above the host's normal work between two slices (a GPU
/// pump's 2 ms budget) and a PE's longest normal gap between progress
/// marks, a few milliseconds.
pub(super) const STALL_MILLIS: u64 = 50;

/// The instructions a running PE retires between two progress marks
/// ([`Clock::mark_due`]): a counter read each, every 250 blocks or so, so
/// that even a PE at 50 us a block marks every 13 ms.
const MARK_INSTRUCTIONS: u64 = 1024;

/// In parts of the stall limit: the longest a running PE goes without
/// publishing its count, and the age of a published count at which the
/// others set its limit ahead of time.
const PUBLISH_PARTS: u64 = 16;
const SUSPECT_PARTS: u64 = 4;

/// A slot's count while its PE's guest waits, or before it ever ran.
const IDLE: u64 = u64::MAX;

/// Set in a published count while the PE stops for host work in the
/// kernel ([`Clock::leave`]): its limit holds whatever its age.
const STOPPED: u64 = 1 << 63;

/// The longest gaps reported, in host counts, keep the phase in their low
/// bits ([`Clock::stall_phase`]).
const PHASE_BITS: u32 = 3;

/// The phase of a gap spent outside the step loop's phases (host work
/// between slices), and of one measured without step timing.
pub(super) const STALL_HOST: u8 = super::step_timing::Phase::COUNT as u8;
pub(super) const STALL_UNKNOWN: u8 = (1 << PHASE_BITS) - 1;

/// Gaps shorter than this many host counts are not reported, so that the
/// slots see a write only for gaps worth naming.
const REPORT_PARTS: u64 = 50;

/// The instructions a busy PE gets to retire between two timer interrupts:
/// XNU's timer interrupt path, which expires the due timer calls and
/// programs the next deadline, retires some tens of thousands, so timer
/// work stays near a tenth of a busy PE that is at least this fast for
/// each [`TIMER_DELAY_HZ`]⁻¹.
pub(super) const TIMER_WORK: u64 = 1 << 18;

/// The latest a busy PE's timer interrupt comes: one in this many parts of
/// a second after the previous one, about a scheduler quantum.
pub(super) const TIMER_DELAY_HZ: u64 = 100;

/// No ceiling, or no stop.
const UNBOUNDED: u64 = u64::MAX;

/// The slew repays one count of lag for each this many host counts: the
/// count runs 1% fast until its lag is repaid, so a lag of L takes 100 L
/// of wall time.
pub(super) const SLEW_PARTS: u64 = 100;

/// In parts of the stall limit: the least host time between two slew
/// steps (3.1 ms at 50 ms, a step of at most 31 us), so that the state
/// native reads share sees at most a few hundred writes a second while
/// lag is owed.
const SLEW_STEP_PARTS: u64 = 16;

/// The host's counter, read fresh at each call.
pub(super) trait Counter: FnMut() -> u64 {}
impl<F: FnMut() -> u64> Counter for F {}

/// The count every parallel PE reads: the host's counter plus one offset,
/// held below a ceiling only while a PE with guest work has run nothing
/// for [`STALL_MILLIS`].
#[derive(Debug)]
pub struct SharedClock {
    /// Written only to seed it and when a PE falls behind or catches up:
    /// native reads share it unmodified in their caches.
    state: CountState,
    /// Each PE's own: written by it alone, read by the others now and then.
    slots: Box<[Slot]>,
    /// The counts a PE may look stopped before it holds the count, and the
    /// latest a busy PE's timer interrupt comes.
    stall: AtomicU64,
    timer_delay: AtomicU64,
    /// Host counts the count lost to stalls past the limit since the last
    /// [`Self::take_stalled`].
    stalled: AtomicU64,
    /// The offset as seeded: the count with no lag. `offset` stays at or
    /// below it, and the difference is the lag the slew repays. Written
    /// only under the sequence, as is `slewed`.
    target: AtomicU64,
    /// The host count the slew's next step counts from: its last step, or
    /// the last [`Self::settle`].
    slewed: AtomicU64,
    /// Counts the slew repaid since the last [`Self::take_repaid`].
    repaid: AtomicU64,
    /// The count before which no running PE can look stalled
    /// ([`Self::look`]), on a line of its own.
    next_scan: Summary,
    /// Slot loads, for the host tests.
    #[cfg(test)]
    loads: AtomicU64,
}

/// A word on a cache line of its own.
#[derive(Debug, Default)]
#[repr(align(64))]
struct Summary(AtomicU64);

/// What one read of a [`SharedClock`] saw: its offset, its lag, the host
/// count its slew steps from, and the host's counter then.
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct Seen {
    offset: u64,
    lag: u64,
    slewed: u64,
    host: u64,
}

impl Seen {
    /// The highest the offset can be at host count `host`: a slew adds at
    /// most one count for each [`SLEW_PARTS`] host counts since `slewed`,
    /// and never more than the lag. A ceiling only lowers the count.
    fn offset_by(&self, host: u64) -> u64 {
        let ahead = host.saturating_sub(self.slewed) / SLEW_PARTS;
        self.offset.wrapping_add(self.lag.min(ahead))
    }

    /// The first host count at which the count reaches `count`, as soon as
    /// a slew that steps at most every `step` host counts can bring it: a
    /// stale answer, or one a slower slew misses, only wakes early.
    fn host_deadline(&self, count: u64, step: u64) -> u64 {
        let plain = count.saturating_add_signed((self.offset as i64).wrapping_neg());
        let first = self.slewed.saturating_add(step);
        if self.lag == 0 || plain <= first {
            return plain;
        }
        // The least `d` with `d + d / SLEW_PARTS` at least `plain - slewed`.
        let distance = plain - self.slewed;
        let full = self.slewed + distance - distance / (SLEW_PARTS + 1);
        full.max(plain.saturating_sub(self.lag)).max(first)
    }
}

/// One PE's part of a [`SharedClock`], on a cache line of its own.
#[derive(Debug)]
#[repr(align(64))]
struct Slot {
    /// The count when it last ran a block, [`IDLE`] while its guest waits;
    /// with [`STOPPED`] while it stops in the kernel for host work.
    count: AtomicU64,
    /// Its longest gap between two blocks with guest work since the last
    /// [`SharedClock::take_stalls`], in host counts, with the phase that
    /// took most of it in the low [`PHASE_BITS`].
    longest: AtomicU64,
}

impl SharedClock {
    /// A clock for `pes` PEs following the host's count.
    pub fn new(pes: usize) -> Self {
        let state = CountState::default();
        state.ceiling.store(UNBOUNDED, Ordering::Relaxed);
        Self {
            state,
            slots: (0..pes)
                .map(|_| Slot {
                    count: AtomicU64::new(IDLE),
                    longest: AtomicU64::new(0),
                })
                .collect(),
            stall: AtomicU64::new(0),
            timer_delay: AtomicU64::new(0),
            stalled: AtomicU64::new(0),
            target: AtomicU64::new(0),
            slewed: AtomicU64::new(0),
            repaid: AtomicU64::new(0),
            next_scan: Summary::default(),
            #[cfg(test)]
            loads: AtomicU64::new(0),
        }
    }

    /// PE `pe`'s published count.
    fn slot(&self, pe: usize) -> u64 {
        #[cfg(test)]
        self.loads.fetch_add(1, Ordering::Relaxed);
        self.slots[pe].count.load(Ordering::Relaxed)
    }

    /// Each PE's longest gap between two blocks with guest work since the
    /// last call, in host counts, and the phase that took most of it
    /// ([`STALL_HOST`], [`STALL_UNKNOWN`] or a step timing phase).
    pub fn take_stalls(&self) -> alloc::vec::Vec<(u64, u8)> {
        self.slots
            .iter()
            .map(|slot| {
                let longest = slot.longest.swap(0, Ordering::Relaxed);
                (
                    longest >> PHASE_BITS,
                    (longest & ((1 << PHASE_BITS) - 1)) as u8,
                )
            })
            .collect()
    }

    /// The count a PE that ran a block at count `count` holds it at, if it
    /// looks stalled at `now`: it stopped in the kernel, or published no
    /// count for a part of the limit.
    fn limit_of(&self, count: u64, now: u64) -> Option<u64> {
        if count == IDLE {
            return None;
        }
        let stall = self.stall.load(Ordering::Relaxed);
        let stopped = count & STOPPED != 0;
        let count = count & !STOPPED;
        (stopped || now.saturating_sub(count) >= stall / SUSPECT_PARTS)
            .then(|| count.saturating_add(stall))
    }

    /// The lowest limit of the PEs that look stalled at count `now`, and
    /// the lowest count at which one that does not yet could.
    fn scan(&self, now: u64) -> (u64, u64) {
        let suspect = self.stall.load(Ordering::Relaxed) / SUSPECT_PARTS;
        // A PE that runs again later publishes a count past `now`.
        let mut next = now.saturating_add(suspect);
        let mut limit = UNBOUNDED;
        for pe in 0..self.slots.len() {
            let count = self.slot(pe);
            match self.limit_of(count, now) {
                Some(of) => limit = limit.min(of),
                None if count != IDLE => next = next.min(count.saturating_add(suspect)),
                None => {}
            }
        }
        (limit, next)
    }

    /// The lowest limit of the PEs that look stalled at count `now`.
    fn limit(&self, now: u64) -> u64 {
        self.scan(now).0
    }

    /// Look at the PEs: set the limit of one that fell behind, or lift a
    /// limit nobody holds any more; else repay lag by a slew step when one
    /// is due. Without either, as nearly always, it writes nothing. Read
    /// outside the sequence: a stale answer settles once more or waits for
    /// the next look.
    ///
    /// With no ceiling set, a look before the count reaches the summary
    /// `next_scan` reads no slot: no PE that runs can look stalled before
    /// its published count is a part of the limit old, and a PE that
    /// publishes later, running again, publishes a later count. A stop
    /// settles at once ([`Self::stop`]). So the slots are read about once a
    /// part of the limit in all, by the one look that claims the scan,
    /// whatever the number of PEs, and publishing writes only its own slot.
    fn look(&self, counter: &mut impl Counter) {
        let (count, seen) = self.read(counter);
        let ceiling = self.state.ceiling.load(Ordering::Relaxed);
        if ceiling == UNBOUNDED {
            let next = self.next_scan.0.load(Ordering::Relaxed);
            if count < next {
                self.slew(&seen, counter);
                return;
            }
            // Claim the scan, so that one look among the PEs reads the
            // slots; the others go on as before it.
            let claim = count.saturating_add(self.stall.load(Ordering::Relaxed) / SUSPECT_PARTS);
            if self
                .next_scan
                .0
                .compare_exchange(next, claim, Ordering::Relaxed, Ordering::Relaxed)
                .is_err()
            {
                return;
            }
        }
        let (limit, next) = self.scan(count);
        self.next_scan.0.fetch_min(next, Ordering::Relaxed);
        let held = ceiling != UNBOUNDED && count >= ceiling;
        // A limit set ahead and still right, or a count held at a limit
        // still owed: the held part becomes lag only when it is lifted.
        if limit == ceiling || held && limit <= ceiling {
            if ceiling == UNBOUNDED {
                self.slew(&seen, counter);
            }
            return;
        }
        self.settle(counter);
    }

    /// Repay lag by one step, if lag is owed, no ceiling is set and a step's
    /// time has passed since the last one ([`SLEW_PARTS`]): the offset rises
    /// by a count for each [`SLEW_PARTS`] host counts since, and never past
    /// the lag. `seen` is a read's, checked first without a write.
    fn slew(&self, seen: &Seen, counter: &mut impl Counter) {
        let step = self.slew_step();
        if seen.lag == 0 || seen.host.saturating_sub(seen.slewed) < step {
            return;
        }
        let sequence = self.lock();
        let host = sampled(counter);
        let offset = self.state.offset.load(Ordering::Relaxed);
        let lag = self.target.load(Ordering::Relaxed).wrapping_sub(offset);
        let slewed = self.slewed.load(Ordering::Relaxed);
        let since = host.saturating_sub(slewed);
        if self.state.ceiling.load(Ordering::Relaxed) == UNBOUNDED && lag != 0 && since >= step {
            let repay = lag.min(since / SLEW_PARTS);
            self.state
                .offset
                .store(offset.wrapping_add(repay), Ordering::Relaxed);
            // The remainder counts toward the next step; repaid in full,
            // the next lag's slew starts from its settle.
            self.slewed
                .store(slewed + repay * SLEW_PARTS, Ordering::Relaxed);
            self.repaid.fetch_add(repay, Ordering::Relaxed);
        }
        self.unlock(sequence);
    }

    /// The least host time between two slew steps, at least one count's.
    fn slew_step(&self) -> u64 {
        (self.stall.load(Ordering::Relaxed) / SLEW_STEP_PARTS).max(SLEW_PARTS)
    }

    /// Set the count's ceiling to the PEs' lowest limit, the count a limit
    /// held becoming lag, so that it continues from where it was held.
    fn settle(&self, counter: &mut impl Counter) {
        let sequence = self.lock();
        let host = sampled(counter);
        let offset = self.state.offset.load(Ordering::Relaxed);
        let ceiling = self.state.ceiling.load(Ordering::Relaxed);
        let free = host.wrapping_add(offset);
        let count = free.min(ceiling);
        if free > count {
            self.stalled.fetch_add(free - count, Ordering::Relaxed);
            self.state
                .offset
                .store(count.wrapping_sub(host), Ordering::Relaxed);
        }
        // No slew runs while a ceiling is set: the next step counts from
        // here.
        self.slewed.store(host, Ordering::Relaxed);
        let limit = self.limit(count);
        let ceiling = if limit == UNBOUNDED {
            UNBOUNDED
        } else {
            limit.max(count)
        };
        self.state.ceiling.store(ceiling, Ordering::Relaxed);
        self.unlock(sequence);
    }

    /// PE `pe` publishes `count`, the count now or a mark.
    fn publish(&self, pe: usize, count: u64) {
        self.slots[pe].count.store(count, Ordering::Relaxed);
    }

    /// Guest work waits for PE `pe` from `count` on while it is out of its
    /// run loop serving the host: a wake that another core sends it, or its
    /// own timer's deadline (`Machine::serve_host`). Its slot takes `count`
    /// if it showed later or idle, so that the count holds a stall limit
    /// past `count` until the PE looks again ([`Clock::idle`],
    /// [`Clock::busy`], [`Clock::look`]): a waiting PE that does not answer
    /// then looks stalled like a running one. A PE stopped in the kernel
    /// ([`STOPPED`]) already holds the count and keeps its mark. Any core
    /// may call this; only the wake path and the start of host service do.
    pub(super) fn pending(&self, pe: usize, count: u64) {
        let slot = &self.slots[pe].count;
        let mut seen = slot.load(Ordering::Relaxed);
        while (seen == IDLE || seen & STOPPED == 0) && count < seen {
            match slot.compare_exchange_weak(seen, count, Ordering::Relaxed, Ordering::Relaxed) {
                Ok(_) => break,
                Err(now) => seen = now,
            }
        }
    }

    /// PE `pe` reports a gap of `ticks` host counts that `phase` took most
    /// of, kept if it is the longest since the last report.
    fn report(&self, pe: usize, ticks: u64, phase: u8) {
        if ticks < self.stall.load(Ordering::Relaxed) / REPORT_PARTS {
            return;
        }
        let packed = ticks << PHASE_BITS | u64::from(phase);
        let longest = &self.slots[pe].longest;
        if packed > longest.load(Ordering::Relaxed) {
            longest.fetch_max(packed, Ordering::Relaxed);
        }
    }

    /// Continue from a count of `offset` past the host's, counting at
    /// `frequency` Hz.
    fn seed(&self, offset: u64, frequency: u64) {
        let sequence = self.lock();
        self.state.offset.store(offset, Ordering::Relaxed);
        self.target.store(offset, Ordering::Relaxed);
        self.slewed.store(0, Ordering::Relaxed);
        self.next_scan.0.store(0, Ordering::Relaxed);
        self.stall
            .store(frequency * STALL_MILLIS / 1000, Ordering::Relaxed);
        self.timer_delay
            .store(frequency / TIMER_DELAY_HZ, Ordering::Relaxed);
        self.unlock(sequence);
    }

    /// The count and what it was read from in one publication, with the
    /// host's counter sampled while that state holds.
    fn read(&self, counter: &mut impl Counter) -> (u64, Seen) {
        loop {
            let sequence = self.state.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                core::hint::spin_loop();
                continue;
            }
            let offset = self.state.offset.load(Ordering::Relaxed);
            let ceiling = self.state.ceiling.load(Ordering::Relaxed);
            let target = self.target.load(Ordering::Relaxed);
            let slewed = self.slewed.load(Ordering::Relaxed);
            let host = sampled(counter);
            fence(Ordering::Acquire);
            if self.state.sequence.load(Ordering::Relaxed) == sequence {
                let seen = Seen {
                    offset,
                    lag: target.wrapping_sub(offset),
                    slewed,
                    host,
                };
                return (host.wrapping_add(offset).min(ceiling), seen);
            }
        }
    }

    /// The offset, lag and slew start from one publication, without the
    /// host's counter.
    fn snapshot(&self) -> Seen {
        loop {
            let sequence = self.state.sequence.load(Ordering::Acquire);
            if sequence & 1 != 0 {
                core::hint::spin_loop();
                continue;
            }
            let offset = self.state.offset.load(Ordering::Relaxed);
            let target = self.target.load(Ordering::Relaxed);
            let slewed = self.slewed.load(Ordering::Relaxed);
            fence(Ordering::Acquire);
            if self.state.sequence.load(Ordering::Relaxed) == sequence {
                return Seen {
                    offset,
                    lag: target.wrapping_sub(offset),
                    slewed,
                    host: 0,
                };
            }
        }
    }

    /// The count now.
    pub fn count(&self, mut counter: impl Counter) -> u64 {
        self.read(&mut counter).0
    }

    /// Host counts the count lost to stalls past the limit since the last
    /// call.
    pub fn take_stalled(&self) -> u64 {
        self.stalled.swap(0, Ordering::Relaxed)
    }

    /// The counts in a millisecond, once seeded.
    pub(super) fn counts_per_milli(&self) -> u64 {
        self.stall.load(Ordering::Relaxed) / STALL_MILLIS
    }

    /// Host counts the count lags the host's by, owed to the slew.
    pub fn lag(&self) -> u64 {
        self.snapshot().lag
    }

    /// Counts the slew repaid since the last call.
    pub fn take_repaid(&self) -> u64 {
        self.repaid.swap(0, Ordering::Relaxed)
    }

    /// The host count at which the count reaches `count` if no stop holds
    /// it meanwhile, as early as the slew can bring it
    /// ([`Seen::host_deadline`]).
    fn host_deadline(&self, count: u64) -> u64 {
        self.snapshot().host_deadline(count, self.slew_step())
    }

    /// When a PE waiting at host counter `host` for the count to reach
    /// `count` looks again: as it does, or, while a stalled PE holds the
    /// count below it, a stall limit later. Read outside the sequence: a stale
    /// answer only wakes early.
    fn wake_at(&self, count: u64, host: u64) -> u64 {
        if count > self.state.ceiling.load(Ordering::Acquire) {
            return host.saturating_add(self.stall.load(Ordering::Relaxed).max(1));
        }
        self.host_deadline(count)
    }

    /// PE `pe` stops in the kernel for host work: the count goes on at
    /// most a stall limit past now until it runs again.
    fn stop(&self, pe: usize, counter: &mut impl Counter) {
        let (count, _) = self.read(counter);
        self.publish(pe, count | STOPPED);
        self.settle(counter);
    }

    /// Make the sequence odd, waiting for another writer to finish; the even
    /// sequence it had. The following host sample is ordered after it.
    fn lock(&self) -> u64 {
        loop {
            let sequence = self.state.sequence.load(Ordering::Relaxed);
            if sequence & 1 == 0
                && self
                    .state
                    .sequence
                    .compare_exchange_weak(
                        sequence,
                        sequence + 1,
                        Ordering::Acquire,
                        Ordering::Relaxed,
                    )
                    .is_ok()
            {
                fence(Ordering::SeqCst);
                return sequence;
            }
            core::hint::spin_loop();
        }
    }

    fn unlock(&self, sequence: u64) {
        self.state.sequence.store(sequence + 2, Ordering::Release);
    }
}

/// The host's counter, sampled after every earlier load and before every
/// later one (`lfence`, Intel SDM Vol. 2B RDTSC), so that a seqlock read
/// brackets it.
fn sampled(counter: &mut impl Counter) -> u64 {
    lfence();
    let host = counter();
    lfence();
    host
}

fn lfence() {
    // SAFETY: `lfence` is SSE2, which every x86-64 processor implements.
    #[cfg(target_arch = "x86_64")]
    unsafe {
        core::arch::x86_64::_mm_lfence()
    };
}

/// This PE's place in a [`SharedClock`].
#[derive(Debug)]
struct Attachment {
    clock: Arc<SharedClock>,
    pe: usize,
    /// Whether it left guest code for host work, and whether its guest
    /// waits, so has no work the host keeps from it.
    stopped: bool,
    idle: bool,
    /// What it last read: the count stays at or below the host's counter
    /// past [`Seen::offset_by`].
    seen: Seen,
    timers: TimerGate,
    progress: Progress,
}

/// A running PE's last progress mark ([`Clock::mark`]).
#[derive(Debug, Default, Clone, Copy)]
struct Progress {
    /// The host counter and retired instructions at the mark.
    host: u64,
    retired: u64,
    /// The profile clock and the step loop's phase totals then, under
    /// step timing.
    phases: Option<Phases>,
    /// The count it last published.
    published: u64,
    /// The block count at which it next marks.
    due: u64,
}

/// The profile clock and the step loop's phase totals at one moment.
pub(super) type Phases = (u64, [u64; super::step_timing::Phase::COUNT]);

/// When a parallel PE's timers may next interrupt it ([`TIMER_WORK`]).
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
struct TimerGate {
    /// The count and retired instructions at the last timer interrupt.
    taken: Option<(u64, u64)>,
    /// The count before which its timers do not signal.
    until: u64,
    /// Whether it waited since: it had time to spare.
    waited: bool,
}

#[derive(Debug)]
pub(super) struct Clock {
    /// Host counts the guest is behind.
    lag: u64,
    /// Counts the guest leapt ahead over idle waits.
    lead: u64,
    /// The serialized count, `lead` less `lag` and no ceiling, where native
    /// count reads find it while no [`SharedClock`] is attached.
    own: CountState,
    /// This reader's last count, which no later read precedes, and its
    /// TSC scale (`register_read::count_thunk`).
    count: Count,
    /// The running slice, when a serialized PE runs: its host start, start
    /// block count and the PEs sharing the round.
    slice: Option<Slice>,
    shared: Option<Attachment>,
}

impl Default for Clock {
    fn default() -> Self {
        let own = CountState::default();
        own.ceiling.store(UNBOUNDED, Ordering::Relaxed);
        Self {
            lag: 0,
            lead: 0,
            own,
            count: Count::default(),
            slice: None,
            shared: None,
        }
    }
}

#[derive(Debug, Clone, Copy)]
struct Slice {
    host: u64,
    lag: u64,
    blocks: u64,
    share: u64,
}

impl Clock {
    /// Read `clock` as PE `pe` from now on, from the count this clock last
    /// read.
    pub(super) fn attach(&mut self, clock: Arc<SharedClock>, pe: usize) {
        let seen = clock.snapshot();
        self.shared = Some(Attachment {
            clock,
            pe,
            stopped: false,
            // It runs: a PE attaches as it enters parallel.
            idle: false,
            seen,
            timers: TimerGate::default(),
            progress: Progress::default(),
        });
        self.publish_state();
    }

    /// Hand this serialized clock's count, as at the host counter `host`, on
    /// to `clock`, which continues it at `frequency` Hz.
    pub(super) fn seed(&mut self, clock: &SharedClock, host: u64, frequency: u64) {
        let count = self.read_at(host);
        clock.seed(count.wrapping_sub(host), frequency);
    }

    /// The count now.
    pub(super) fn read(&mut self, mut counter: impl Counter) -> u64 {
        let count = match &mut self.shared {
            Some(attachment) => {
                let (count, seen) = attachment.clock.read(&mut counter);
                attachment.seen = seen;
                count
            }
            None => return self.read_at(counter()),
        };
        self.count.last = count.max(self.count.last);
        self.count.last
    }

    /// Whether the count is below `count` for certain, answered without
    /// reading the shared state: an exit's question, whose usual answer is
    /// that no deadline is near.
    pub(super) fn below(&self, count: u64, mut counter: impl Counter) -> bool {
        self.shared.as_ref().is_some_and(|attachment| {
            let host = counter();
            host.wrapping_add(attachment.seen.offset_by(host)) < count
        })
    }

    /// The serialized count at host counter value `host`.
    fn read_at(&mut self, host: u64) -> u64 {
        self.count.last = host
            .saturating_add(self.lead)
            .saturating_sub(self.lag)
            .max(self.count.last);
        self.count.last
    }

    /// The host counter value at which the count reaches `count`.
    pub(super) fn host_deadline(&self, count: u64) -> u64 {
        match &self.shared {
            Some(attachment) => attachment.clock.host_deadline(count),
            None => count.saturating_add(self.lag).saturating_sub(self.lead),
        }
    }

    /// The host counter value at which a PE waiting at host counter `host`
    /// for the count to reach `count` looks at the count again.
    pub(super) fn wake_at(&self, count: u64, host: u64) -> u64 {
        match &self.shared {
            Some(attachment) => attachment.clock.wake_at(count, host),
            None => self.host_deadline(count),
        }
    }

    /// `counts` of host time passed with no PE running and nothing in the
    /// guest's world happening: the serialized count does not show it.
    /// Attached, the count is the host's.
    pub(super) fn exclude(&mut self, counts: u64) {
        if self.shared.is_none() {
            self.lag += counts;
            self.sync();
        }
    }

    /// While every PE waits at host counter value `host`, the count leaps to
    /// `deadline` if it is still before it. Parallel PEs do not leap
    /// (`Machine::set_idle_skip`).
    pub(super) fn skip_to(&mut self, host: u64, deadline: u64) {
        debug_assert!(self.shared.is_none(), "an attached clock never leaps");
        let count = self.read_at(host);
        if deadline > count {
            // Advance the underlying count, not only the monotonic clamp:
            // an earlier slice may still hold `count` above host minus lag.
            let adjusted = host.saturating_add(self.lead).saturating_sub(self.lag);
            self.lead += deadline - adjusted;
            self.count.last = deadline;
            self.sync();
        }
    }

    /// A slice starts at block count `blocks`, with `share` PEs (at least
    /// one) able to run in the round. Attached, the count is the host's.
    pub(super) fn start_slice(&mut self, mut counter: impl Counter, blocks: u64, share: u64) {
        if self.shared.is_some() {
            return;
        }
        let host = counter();
        self.read_at(host);
        self.slice = Some(Slice {
            host,
            lag: self.lag,
            blocks,
            share: share.max(1),
        });
    }

    /// The running slice ends at block count `blocks`. Serialized, elapsed
    /// host counts beyond its blocks' bound become lag; explicitly excluded
    /// host work was already charged and is not charged twice, and the
    /// monotonic count, which an earlier read can still hold, does not
    /// measure this slice's elapsed time.
    pub(super) fn end_slice(&mut self, mut counter: impl Counter, blocks: u64) {
        if self.shared.is_some() {
            return;
        }
        let host = counter();
        self.read_at(host);
        let Some(slice) = self.slice.take() else {
            return;
        };
        let allowed = blocks
            .saturating_sub(slice.blocks)
            .saturating_mul(TICKS_PER_BLOCK)
            / slice.share;
        let elapsed = host
            .saturating_sub(slice.host)
            .saturating_sub(self.lag.saturating_sub(slice.lag));
        self.lag += elapsed.saturating_sub(allowed);
        self.sync();
    }

    /// The running PE waits or is paused: attached, it has time to spare,
    /// so its timers signal again as soon as they are due.
    pub(super) fn wait(&mut self, _counter: impl Counter) {
        if let Some(attachment) = &mut self.shared {
            attachment.timers.until = 0;
            attachment.timers.waited = true;
        }
    }

    /// The running PE's guest waits (WFI or WFE) or its PE is off, with
    /// `retired` instructions retired: until [`Self::busy`] the host keeps
    /// no guest work from it, so it neither holds the count nor reports a
    /// stall. A guest wait is the guest's choice; were it a stall, a wait
    /// for a count the stall holds would never end.
    pub(super) fn idle(&mut self, mut counter: impl Counter, retired: u64, phases: Option<Phases>) {
        if self
            .shared
            .as_ref()
            .is_some_and(|attachment| !attachment.idle)
        {
            self.mark_ending(&mut counter, retired, phases, true);
        }
        if let Some(attachment) = &mut self.shared {
            attachment.idle = true;
            let clock = &attachment.clock;
            clock.publish(attachment.pe, IDLE);
            if clock.state.ceiling.load(Ordering::Relaxed) != UNBOUNDED {
                clock.look(&mut counter);
            }
        }
    }

    /// The running PE has guest work again, from host counter now.
    pub(super) fn busy(&mut self, mut counter: impl Counter, retired: u64, phases: Option<Phases>) {
        let Some(attachment) = &mut self.shared else {
            return;
        };
        if !core::mem::take(&mut attachment.idle) {
            return;
        }
        let (count, _) = attachment.clock.read(&mut counter);
        attachment.clock.publish(attachment.pe, count);
        attachment.progress = Progress {
            host: counter(),
            retired,
            phases,
            published: count,
            due: retired.saturating_add(MARK_INSTRUCTIONS),
        };
    }

    /// Whether the running PE, at `retired` instructions, is due to mark
    /// its progress: a compare, for every step.
    #[inline]
    pub(super) fn mark_due(&self, retired: u64) -> bool {
        self.shared
            .as_ref()
            .is_some_and(|attachment| retired >= attachment.progress.due)
    }

    /// The running PE marks its progress at `retired` instructions, with
    /// the step loop's `phases` under step timing: it reports the gap since
    /// its last mark, publishes its count every part of the stall limit,
    /// and then looks at the others ([`SharedClock::look`]).
    pub(super) fn mark(&mut self, counter: impl Counter, retired: u64, phases: Option<Phases>) {
        self.mark_ending(counter, retired, phases, false);
    }

    /// [`Self::mark`], with the gap reported even without progress when
    /// the PE's guest work `ends` (its guest waits).
    fn mark_ending(
        &mut self,
        mut counter: impl Counter,
        retired: u64,
        phases: Option<Phases>,
        ends: bool,
    ) {
        let Some(attachment) = &mut self.shared else {
            return;
        };
        let clock = &attachment.clock;
        let host = counter();
        let progress = &mut attachment.progress;
        progress.due = retired.saturating_add(MARK_INSTRUCTIONS);
        if attachment.idle {
            return;
        }
        let ran = retired != progress.retired;
        if ran || ends {
            // Its first mark since it attached starts the first gap.
            if progress.host != 0 {
                let gap = host.saturating_sub(progress.host);
                clock.report(
                    attachment.pe,
                    gap,
                    Self::stall_phase(progress.phases, phases),
                );
            }
            progress.host = host;
            progress.retired = retired;
            progress.phases = phases;
        }
        // While a limit is set, every mark publishes and looks, so that
        // the PE that set it lifts it as soon as it runs.
        let stall = clock.stall.load(Ordering::Relaxed);
        let ceiling = clock.state.ceiling.load(Ordering::Relaxed);
        let (count, _) = clock.read(&mut counter);
        if ceiling == UNBOUNDED && count.saturating_sub(progress.published) < stall / PUBLISH_PARTS
        {
            return;
        }
        if ran {
            clock.publish(attachment.pe, count);
            progress.published = count;
        }
        clock.look(&mut counter);
    }

    /// The running PE, waiting, looks at the others once a pass: the PEs
    /// that run may all be stopped. It is back in its loop, so a wake mark
    /// left in its own slot from its host service ([`SharedClock::pending`])
    /// goes: the pass sees what the wake brought.
    pub(super) fn look(&self, mut counter: impl Counter) {
        if let Some(attachment) = &self.shared {
            let slot = &attachment.clock.slots[attachment.pe].count;
            if attachment.idle && slot.load(Ordering::Relaxed) != IDLE {
                slot.store(IDLE, Ordering::Relaxed);
            }
            attachment.clock.look(&mut counter);
        }
    }

    /// The running PE leaves its run loop to serve the host, its own timers
    /// next due at `deadline`: the count waits for it a stall limit past
    /// that deadline if it is still out then ([`SharedClock::pending`]).
    pub(super) fn serving(&self, deadline: Option<u64>) {
        if let (Some(attachment), Some(deadline)) = (&self.shared, deadline) {
            attachment.clock.pending(attachment.pe, deadline);
        }
    }

    /// The phase that took most of a gap from `then` to `now`: one of the
    /// step loop's, or the host's work outside them.
    fn stall_phase(then: Option<Phases>, now: Option<Phases>) -> u8 {
        let (Some((then_ticks, then)), Some((now_ticks, now))) = (then, now) else {
            return STALL_UNKNOWN;
        };
        let mut spent = [0u64; super::step_timing::Phase::COUNT + 1];
        for (phase, (now, then)) in now.iter().zip(then).enumerate() {
            spent[phase] = now.wrapping_sub(then);
        }
        let phases = spent
            .iter()
            .fold(0u64, |sum, ticks| sum.saturating_add(*ticks));
        spent[STALL_HOST as usize] = now_ticks.wrapping_sub(then_ticks).saturating_sub(phases);
        (0..spent.len())
            .max_by_key(|&phase| spent[phase])
            .map_or(STALL_UNKNOWN, |phase| phase as u8)
    }

    /// The running PE leaves guest code for host work, `at_el0` or in the
    /// kernel. Attached, with guest work and in the kernel, it may hold a
    /// lock another PE times, so the count waits for it once it has been
    /// stopped [`STALL_MILLIS`]; at EL0 it holds no such lock, nor while
    /// its guest waits for an interrupt. Whether it left, for
    /// [`Self::resume`] once the host's work returns.
    pub(super) fn leave(&mut self, mut counter: impl Counter, _work: u64, at_el0: bool) -> bool {
        match &mut self.shared {
            Some(attachment) if !attachment.idle && !attachment.stopped => {
                attachment.stopped = true;
                if at_el0 {
                    attachment.clock.publish(attachment.pe, IDLE);
                } else {
                    attachment.clock.stop(attachment.pe, &mut counter);
                }
                true
            }
            _ => false,
        }
    }

    /// The PE that [`Self::leave`]s returns to guest code. A count its stop
    /// held becomes lag, so it continues from where it was held.
    pub(super) fn resume(&mut self, mut counter: impl Counter, _work: u64) {
        if let Some(attachment) = &mut self.shared
            && core::mem::take(&mut attachment.stopped)
        {
            let (count, _) = attachment.clock.read(&mut counter);
            attachment.clock.publish(attachment.pe, count);
            attachment.progress.published = count;
            attachment.clock.look(&mut counter);
        }
    }

    /// The earliest count at which the running PE's timers may signal, of
    /// its timers' own `deadline`: attached, no earlier than its share of
    /// work allows ([`TIMER_WORK`]).
    pub(super) fn timer_due(&self, deadline: Option<u64>) -> Option<u64> {
        match &self.shared {
            Some(attachment) => deadline.map(|deadline| deadline.max(attachment.timers.until)),
            None => deadline,
        }
    }

    /// The running PE took a timer interrupt at `count` with `retired`
    /// instructions retired. Attached and busy since the last one, its
    /// timers next signal once it has had the time to retire
    /// [`TIMER_WORK`] more at the rate it ran since, and no later than the
    /// timer delay; one that waited meanwhile had time to spare.
    pub(super) fn timer_taken(&mut self, count: u64, retired: u64) {
        let Some(attachment) = &mut self.shared else {
            return;
        };
        let gate = &mut attachment.timers;
        let delay = attachment.clock.timer_delay.load(Ordering::Relaxed);
        let spacing = match gate.taken {
            Some((then, before)) if !gate.waited => {
                let elapsed = u128::from(count.saturating_sub(then));
                let ran = u128::from(retired.saturating_sub(before).max(1));
                (elapsed * u128::from(TIMER_WORK) / ran).min(u128::from(delay)) as u64
            }
            _ => 0,
        };
        *gate = TimerGate {
            taken: Some((count, retired)),
            until: count.saturating_add(spacing),
            waited: false,
        };
    }

    /// The last count read.
    pub(super) fn last(&self) -> u64 {
        self.count.last
    }

    /// Native count reads at the host counter `((tsc - base) * per_tsc) >>
    /// 64` ([`super::Host::tsc_counter`]): where they find this reader.
    pub(super) fn native(&mut self, (base, per_tsc): (u64, u64)) {
        self.count.base = base;
        self.count.per_tsc = per_tsc;
        self.publish_state();
    }

    /// Whether native count reads find the host counter's scale here
    /// ([`Self::native`]): without it they read a count that stands still.
    pub(super) fn reads_natively(&self) -> bool {
        self.count.per_tsc != 0
    }

    /// Show native count reads the state this clock follows.
    fn publish_state(&mut self) {
        self.count.state = match &self.shared {
            Some(attachment) => &attachment.clock.state as *const CountState as u64,
            None => &self.own as *const CountState as u64,
        };
    }

    /// The reader native count reads use ([`Anchor::Count`]).
    pub(super) fn reader(&self) -> u64 {
        &self.count as *const Count as u64
    }

    fn sync(&mut self) {
        self.own
            .offset
            .store(self.lead.wrapping_sub(self.lag), Ordering::Relaxed);
    }
}

#[cfg(test)]
mod tests;
