//! [`Service`] for every PE at once, as the call gate's [`GateService`].
//!
//! The device is one Vulkan device with one state, so calls take turns under
//! a lock. A PE waiting its turn runs the platform's `relax`, since the call
//! it waits for may need that PE: on the CPU that runs the kernel, waiting
//! without running it would never end. Each call runs through the platform's
//! `run`, as a compile needs more stack than a PE keeps; the lock makes one
//! such stack enough for a device.
//!
//! The gate asks about completed work under the board's lock, where a Vulkan
//! call would hold up every PE, so [`GateService::signaled`] only reads a
//! flag. The platform reports each GPU interrupt ([`SharedService::interrupted`]);
//! whoever next holds the service looks at the device for what completed.
//!
//! A `Poll` that finds another call holding the service is answered with the
//! newest serial a holder saw completed, rather than waiting out a commit or
//! a compile: the completion interrupt that wakes a frontend's poll is raised
//! only once a holder has seen that serial, so the answer covers it. Only a
//! connection whose poll the service itself answered is answered this way,
//! and never once the device is lost.
//!
//! The platform's waiter on the device's completion timeline may publish a
//! completed serial itself ([`SharedService::completed_through`]), without a
//! turn, when every host-visible memory type is coherent: completion then
//! needs no invalidation before the CPU reads what the GPU wrote. The next
//! holder looks at the device before its call, so what it answers agrees.
//!
//! No compilation runs in a turn. A pipeline creation or commit that needs
//! one no cache holds (AIR to NIR, or a pipeline's creation, which runs NIR
//! to machine code) fails without writing to its caller and leaves a job
//! ([`crate::metal::deferred`]); the service answers the call
//! [`Status::Pending`] and the frontend calls again after a moment, its
//! processor back in the guest meanwhile. A commit's replay that meets a
//! missing variant discards its command buffer, as nothing it did before
//! stays, and once replay changes an indirect command buffer it compiles in
//! its turn ([`Compiler::stop_deferring`]). Until its job has run, a call
//! made again is answered `Pending` without a turn. A new texture's kernel
//! memory, which Linux can take many frames to make, defers the same way:
//! the job allocates it, and the call's next run takes it.
//!
//! Driver work that needs no service state runs after the call's turn, so
//! that its host kernel calls, which wait for the boot processor's Linux,
//! hold no other connection's call: an import's export and the driver's
//! import of it, and frees of evicted kept imports
//! ([`crate::metal::vulkan::backend::Outside`]). It runs one call at a time
//! on a stack of its own ([`SharedService::set_outside_stack`]) under the
//! outside lock. Lock order: the turn and the outside lock are never held
//! together (a call drops its turn before it takes the outside lock, and
//! takes the turn again only after releasing it); the compile lock is taken
//! inside a turn or by a job, never with the outside lock; the recent-import
//! hashes' and jobs' spin locks are leaves. What stays under the turn: the
//! connections' object tables, the device's resource and queue state, the
//! Vulkan queue and command pools, and the backend's import bookkeeping,
//! which an outside import joins in a second, short turn. The Linux layer is
//! unchanged: posted calls go on its lock-free list from any processor, and
//! only the boot processor runs Linux.
//!
//! The platform runs the jobs ([`SharedService::run_job`]) where it allows
//! long host work, one at a time on a stack of their own
//! ([`SharedService::set_compile_stack`]) under the compile lock, which AIR
//! compilations in a turn also take. What a job made is adopted at the next
//! call's turn, which its call's next run then finds. Only owned inputs
//! cross: the libraries' bytes, the driver's NIR options, copies of NIR and
//! pipeline caches, which are internally synchronized; the device's queue
//! and command pools stay the turn's.
use super::{
    Service, StagedLibrary,
    wire::{self, GateCall, NewLibraryRequest, Operation, Span, Status},
};
use crate::metal::{
    compiler::{self, AirCompiler, CompileLock},
    deferred::{Done, Job, JobLock, Owner},
    shader::Compiler,
    timing::CallPhases,
    vulkan::{
        MemoryProperties,
        backend::{Backend, Outside},
    },
};
use alloc::{boxed::Box, collections::VecDeque, sync::Arc, vec::Vec};
use core::{
    any::Any,
    cell::UnsafeCell,
    mem::size_of,
    num::NonZeroU64,
    ops::{Deref, DerefMut},
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, AtomicUsize, Ordering},
};
use stela_contracts::gate::{BundleReport, CallerFault, CallerMemory, GateService, MemoryRange};
use stela_crypto::{Sha256, digest::Digest};

pub struct SharedService<B: Backend, C: Compiler> {
    service: UnsafeCell<Service<B, C>>,
    locked: AtomicBool,
    /// The GPU interrupted since the device was last asked.
    stale: AtomicBool,
    /// Completed work the gate has not taken.
    completed: AtomicBool,
    /// The newest serial a holder saw completed, or POLL_UNAVAILABLE.
    reached: AtomicU64,
    /// Connections whose `Poll` the service answered, zero for a free slot:
    /// those a busy service answers from `reached`.
    polling: [AtomicU64; POLLING],
    /// Submissions not yet seen complete, with when each was submitted;
    /// only a turn's holder touches it.
    pending: UnsafeCell<Pending>,
    /// Since [`SharedService::take_latency`]: submissions seen complete,
    /// the ticks from submission until a holder saw each complete, and the
    /// longest of those.
    latency: [AtomicU64; 3],
    /// The completion timelines handed to the platform, one per device: a
    /// waiter publishes for the newest only.
    timeline: AtomicU64,
    /// The newest serial a holder saw submitted, past which nothing is
    /// published.
    submitted: AtomicU64,
    /// The platform's waiter's newest reading of the completion timeline,
    /// published or not, with the low 16 bits of its generation above bit
    /// 48: a look takes it instead of asking the driver.
    waited: AtomicU64,
    /// Whether a waiter may publish completions without a turn: no
    /// host-visible memory needs an invalidation.
    publishable: AtomicBool,
    /// When a serial was published without a turn, by serial modulo
    /// [`PENDING`]: the serial and the tick.
    early: [(AtomicU64, AtomicU64); PENDING],
    /// The stack jobs run on, zero while the platform runs none, and the
    /// lock they run under.
    compile_stack: AtomicU64,
    /// The longest a frontend call waits for the turn before it is answered
    /// `Pending`, in ticks; 0 waits for as long as the holder holds it.
    wait_bound: AtomicU64,
    job_lock: JobLock,
    jobs: Spin<Jobs>,
    /// `NewLibrary` calls' bytes read so far, before their turns.
    staging: Spin<Vec<Staging>>,
    /// How long a `NewLibrary` call reads its library at one run, before
    /// its turn, in ticks; 0 reads one piece.
    library_budget: AtomicU64,
    /// The bytes it reads and digests at once ([`LIBRARY_PIECE_BYTES`]).
    library_piece: AtomicUsize,
    /// The guest RAM the latest imports named, by a hash of their ranges.
    imported: Spin<RecentImports>,
    /// The stack driver work outside the turn runs on, zero for none (it
    /// then stays in the turn), and the lock its user holds.
    outside_stack: AtomicU64,
    outside_locked: AtomicBool,
    /// The backend's outside work as the last turn saw it, for the
    /// platform's upkeep ([`SharedService::upkeep`]), which takes no turn;
    /// whether the platform was asked for one; and the ticks each part of
    /// the platform's look may take ([`SharedService::set_look_budget`]).
    outside_seen: Spin<Option<Arc<dyn Outside>>>,
    /// The device's completion marker as the last turn saw it, null for
    /// none or a lost device, and the processors reading it now: a turn
    /// that clears it waits for them, so no read outlives the device.
    marker: AtomicPtr<AtomicU32>,
    marker_readers: AtomicU32,
    upkeep_asked: AtomicBool,
    look_budget: AtomicU64,
    /// Refused submissions whose command buffers' connections were not yet
    /// told: raised before the refused submission's signals are made, so
    /// while it is not zero no poll is answered from the published serial,
    /// which cannot say which command buffer failed.
    refusals: Arc<AtomicU32>,
    host: ServiceHost,
}

/// Deferred compilations and the calls that wait for them.
#[derive(Default)]
struct Jobs {
    /// Jobs not yet run, each with when it was queued.
    queued: VecDeque<(Job, u64)>,
    running: Option<[u8; 32]>,
    /// Jobs that ran, for the next turn to adopt.
    done: Vec<Done>,
    /// Calls answered `Pending`, by the key they wait for.
    parked: VecDeque<Parked>,
    report: JobReport,
}

/// A call answered `Pending`: the same call made again is answered so
/// until the job of `key` has run.
#[derive(Clone, Copy, PartialEq, Eq)]
struct Parked {
    connection: u64,
    record: u64,
    operation: u32,
    request: u64,
    key: [u8; 32],
}

/// More calls than wait at once; the oldest is forgotten, and asks again.
const PARKED: usize = 128;

/// Jobs run since the last [`SharedService::take_job_report`], in the
/// platform's ticks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct JobReport {
    pub ran: u64,
    pub ticks: u64,
    pub longest: u64,
    /// From queueing to running, the longest.
    pub longest_queued: u64,
    /// Calls answered `Pending`, with a turn and without one.
    pub deferred: u64,
    pub repeated: u64,
}

/// The operation [`Served`] names for the kext's import of wired memory,
/// which no frontend operation is: past every protocol operation's code.
pub const IMPORT_OPERATION: u32 = 0xff01;
/// The operation [`Served`] names for an import of the same guest RAM as one
/// of the [`RECENT_IMPORTS`] before it, such as a surface's pages each time a
/// texture is made of them: what keeping an import for its pages would save.
pub const IMPORT_AGAIN_OPERATION: u32 = 0xff02;

/// How many imports [`IMPORT_AGAIN_OPERATION`] looks back over.
const RECENT_IMPORTS: usize = 64;

/// The latest imports' ranges, each as an FNV-1a hash of its addresses and
/// lengths, oldest overwritten first.
struct RecentImports {
    hashes: [u64; RECENT_IMPORTS],
    next: usize,
}

impl Default for RecentImports {
    fn default() -> Self {
        Self {
            hashes: [0; RECENT_IMPORTS],
            next: 0,
        }
    }
}

impl RecentImports {
    /// Notes an import of `ranges`: whether one of the latest named the
    /// same.
    fn again(&mut self, ranges: &[MemoryRange]) -> bool {
        let hash = ranges
            .iter()
            .flat_map(|range| [range.address, range.bytes])
            .flat_map(u64::to_le_bytes)
            .fold(0xcbf2_9ce4_8422_2325_u64, |hash, byte| {
                (hash ^ u64::from(byte)).wrapping_mul(0x100_0000_01b3)
            });
        let again = self.hashes.contains(&hash);
        self.hashes[self.next] = hash;
        self.next = (self.next + 1) % RECENT_IMPORTS;
        again
    }
}

/// The bytes of a library a `NewLibrary` call reads and digests at once,
/// before its turn; a run reads pieces until its budget
/// ([`SharedService::set_library_budget`]) is spent, at least one.
pub(super) const LIBRARY_PIECE_BYTES: usize = 1 << 20;

/// Libraries read in part at once; past this the oldest is dropped, and its
/// call, made again, starts over.
const STAGED: usize = 4;

/// A `NewLibrary` call's library read so far ([`SharedService::stage_library`]).
///
/// A library is first read only to digest it, a piece at a time through one
/// piece buffer, keeping no byte: the device shares the bytes of a library
/// it already holds by their digest, and every process loads the same
/// system libraries of hundreds of megabytes. Only when the turn finds the
/// device holds no library of that digest is the library read again into
/// `kept`, digested again as it is copied, so the kept bytes are what the
/// digest names even when the caller changed them between the reads.
struct Staging {
    connection: u64,
    record: u64,
    span: Span,
    /// The bytes read and digested so far.
    read: usize,
    digest: Sha256,
    /// The buffer each piece is read into while only digesting.
    piece: Vec<u8>,
    /// The bytes, once the turn asked for them.
    kept: Option<Vec<u8>>,
}

/// What reading the next piece of a call's library found.
enum Staged {
    /// The call reads no library, or reading it failed without a fault the
    /// frontend can repair: its turn serves it as it is.
    Not,
    /// More bytes remain: the call is answered `Pending`.
    Pending,
    /// Part of the span is not mapped: the call is answered `Unmapped`,
    /// and the frontend maps it and calls again, keeping what was read.
    Unmapped,
    /// Every byte was read: the turn takes them.
    Ready,
}

/// A lock for short bookkeeping, which spins.
struct Spin<T> {
    held: AtomicBool,
    value: UnsafeCell<T>,
}

impl<T> Spin<T> {
    fn new(value: T) -> Self {
        Self {
            held: AtomicBool::new(false),
            value: UnsafeCell::new(value),
        }
    }

    fn with<R>(&self, body: impl FnOnce(&mut T) -> R) -> R {
        while self
            .held
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            core::hint::spin_loop();
        }
        // SAFETY: the flag gives one caller at a time the value.
        let result = body(unsafe { &mut *self.value.get() });
        self.held.store(false, Ordering::Release);
        result
    }
}

/// The newest submissions, as a ring of serial and submission tick.
struct Pending {
    submitted: u64,
    ring: [(u64, u64); PENDING],
    head: usize,
    len: usize,
}

/// More submissions than are in flight at once; older ones go uncounted.
const PENDING: usize = 64;

/// More connections than a device's gates open.
const POLLING: usize = 64;
/// A lost device or this serial must use the normal poll path.
const POLL_UNAVAILABLE: u64 = u64::MAX;
/// Where [`SharedService::waited`] keeps the generation: serials stay below.
const WAITED_GENERATION: u64 = 1 << 48;

/// The waiter's newest reading of the current timeline, if any.
fn waited_value(waited: &AtomicU64, timeline: &AtomicU64) -> Option<u64> {
    let waited = waited.load(Ordering::Acquire);
    let timeline = timeline.load(Ordering::Acquire);
    (waited >> 48 == timeline & 0xffff && waited & (WAITED_GENERATION - 1) != 0)
        .then_some(waited & (WAITED_GENERATION - 1))
}

/// What the platform does for the shared service.
#[derive(Clone, Copy)]
pub struct ServiceHost {
    /// Waits a moment for the turn's holder, in a wait with no bound: it may
    /// do the platform's work the holder waits for, however long that takes.
    pub relax: fn(),
    /// Waits a moment for the turn's holder, in a wait the wait bound ends
    /// ([`SharedService::set_wait_bound`]): it does nothing that can outlast
    /// the bound, and answers false when the wait should end at once, as the
    /// holder waits for work only this processor does once it stops waiting.
    pub relax_within: fn() -> bool,
    /// What `run` takes first: this service's stack.
    pub stack: u64,
    /// Calls its argument on a stack a compile fits on. Only a turn's holder
    /// comes here.
    pub run: fn(u64, &mut dyn FnMut()),
    /// Told why the service refused a guest's call.
    pub report: fn(&str),
    /// Told what a guest's call made, such as a render pipeline's stages and
    /// vertex fetch.
    pub note: fn(&str),
    /// The platform's tick counter, which times [`Served`].
    pub ticks: fn() -> u64,
    /// Told of each guest call the gate served.
    pub served: fn(Served),
    /// Asks the platform to call [`SharedService::interrupted`] again soon,
    /// outside this turn: a turn held the service when the GPU interrupted.
    pub wake: fn(),
    /// Told what the kext found of the Metal plugin, which the platform's
    /// boot decides on: the plugin is the boot's.
    pub bundle: fn(BundleReport),
}

/// A guest call the gate served, in the platform's ticks: the PE that rang
/// for it runs no guest code from the doorbell until the call returns.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Served {
    /// The call's operation code, zero when its record was out of reach.
    pub operation: u32,
    pub status: Status,
    /// Waiting for the turn another call held.
    pub waited: u64,
    /// Holding the turn, the look at completed work after the call included.
    pub held: u64,
    /// The serials of the last command buffer submitted and the last seen
    /// complete after the call, when it held the service.
    pub progress: Option<(u64, u64)>,
    /// Where the call's time went, for the operations timed by phase.
    pub phases: Option<CallPhases>,
    /// Compiling outside the lock for the call, which its other calls'
    /// turns were served meanwhile, and how many compilations that was:
    /// now always zero and one per call answered `Pending`, as the platform
    /// runs the jobs ([`SharedService::take_job_report`]).
    pub compiled: u64,
    pub deferrals: u32,
    /// The longest one of the call's waits for a turn.
    pub longest_wait: u64,
    /// Answered `Pending` once its wait for the turn passed the bound
    /// ([`SharedService::set_wait_bound`]).
    pub gave_up: bool,
}

// SAFETY: the lock gives one PE at a time the service, which `Send` lets move
// between them; jobs are `Send` and their list is behind its own lock.
unsafe impl<B: Backend + Send, C: Compiler + Send> Sync for SharedService<B, C> {}
// SAFETY: every field is, given the bounds.
unsafe impl<B: Backend + Send, C: Compiler + Send> Send for SharedService<B, C> {}

/// Pieces of outside work one upkeep does at most ([`SharedService::upkeep`]).
pub(super) const UPKEEP_PIECES: usize = 16;

/// A turn at the service.
/// The service held, and whether a look in this turn found work completed.
pub(super) struct Turn<'a, B: Backend, C: Compiler>(&'a SharedService<B, C>, bool);

impl<B: Backend, C: Compiler> Deref for Turn<'_, B, C> {
    type Target = Service<B, C>;

    fn deref(&self) -> &Self::Target {
        // SAFETY: this turn holds the lock.
        unsafe { &*self.0.service.get() }
    }
}

impl<B: Backend, C: Compiler> DerefMut for Turn<'_, B, C> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: this turn holds the lock.
        unsafe { &mut *self.0.service.get() }
    }
}

impl<B: Backend, C: Compiler> Turn<'_, B, C> {
    /// Asks the device what completed for the interrupt that raised `stale`.
    /// The turn's drop publishes what it found.
    fn look(&mut self) {
        self.0.stale.store(false, Ordering::SeqCst);
        let mut completed = false;
        // The waiter read the timeline as it woke, in Linux: its value spares
        // the driver query, a host kernel call that waits for the boot
        // processor. A value that shows nothing new still asks the driver,
        // which notices a loss.
        let waited = waited_value(&self.0.waited, &self.0.timeline);
        // The marker may show more, with no Linux work behind it.
        let waited = match (waited, self.0.marker_value()) {
            (Some(waited), Some(marked)) => Some(waited.max(marked)),
            (waited, marked) => waited.or(marked),
        };
        // The platform's processor waits for its look: what completed work
        // released is destroyed within the budget, and the rest is left for
        // the next look, which the drop asks for, or a call.
        let budget = self.0.look_budget.load(Ordering::Acquire);
        self.device().bound_destruction(budget);
        // Polling enters Vulkan too. An interrupt can take this path without
        // a guest call's stack switch around it.
        (self.0.host.run)(self.0.host.stack, &mut || {
            completed = match waited {
                Some(value) => (**self).signaled_reached(value),
                None => (**self).signaled(),
            }
        });
        let device = self.device();
        device.unbound_destruction();
        if device.destruction_left() {
            self.0.stale.store(true, Ordering::SeqCst);
        }
        self.1 |= completed;
    }
}

impl<B: Backend, C: Compiler> Turn<'_, B, C> {
    /// Looks at the device when a waiter published past what it last saw,
    /// so the call's answers agree with the completion the guest was told.
    ///
    /// The published serial is the waiter's reading of the timeline (or a
    /// turn's, which never passes it), and nothing is held while one is
    /// published, so the device takes it as the timeline's value instead of
    /// asking the driver: on a guest PE that query is a host kernel call the
    /// boot CPU serves while the PE spins.
    fn catch_up(&mut self) {
        let shown = self.0.reached.load(Ordering::Acquire);
        let device = self.device();
        if shown != POLL_UNAVAILABLE && !device.is_lost() && shown > device.completed_serial() {
            // An interrupt's look stays the platform's (`stale`): it may have
            // seen past what was published.
            let mut completed = false;
            (self.0.host.run)(self.0.host.stack, &mut || {
                completed = (**self).signaled_reached(shown);
            });
            self.1 |= completed;
        }
    }

    /// Notes when submissions through `submitted` were made and how long
    /// those through `reached` took to be seen complete.
    fn time_submissions(&self, submitted: u64, reached: u64) {
        // SAFETY: this turn holds the lock, which owns `pending`.
        let pending = unsafe { &mut *self.0.pending.get() };
        let due = pending.len != 0 && pending.ring[pending.head].0 <= reached;
        if pending.submitted >= submitted && !due {
            return;
        }
        let now = (self.0.host.ticks)();
        while pending.submitted < submitted {
            pending.submitted += 1;
            let at = (pending.head + pending.len) % PENDING;
            pending.ring[at] = (pending.submitted, now);
            if pending.len == PENDING {
                pending.head = (pending.head + 1) % PENDING;
            } else {
                pending.len += 1;
            }
        }
        while pending.len != 0 && pending.ring[pending.head].0 <= reached {
            let (serial, at) = pending.ring[pending.head];
            // Seen when a waiter published it, if one did first.
            let (published, when) = &self.0.early[serial as usize % PENDING];
            let seen = if published.load(Ordering::Acquire) == serial {
                when.load(Ordering::Relaxed).min(now)
            } else {
                now
            };
            let took = seen.wrapping_sub(at);
            pending.head = (pending.head + 1) % PENDING;
            pending.len -= 1;
            self.0.latency[0].fetch_add(1, Ordering::Relaxed);
            self.0.latency[1].fetch_add(took, Ordering::Relaxed);
            self.0.latency[2].fetch_max(took, Ordering::Relaxed);
        }
    }
}

impl<B: Backend, C: Compiler> Drop for Turn<'_, B, C> {
    fn drop(&mut self) {
        // A lost device answers without entering the driver.
        let mut completed = self.1;
        if self.device().is_lost() {
            (self.0.host.run)(self.0.host.stack, &mut || completed |= (**self).signaled());
        }
        let refusing = self.0.refusals.load(Ordering::Acquire) != 0;
        let device = self.device();
        let submitted = device.submitted_serial();
        let serial = device.completed_serial();
        let word = device.completion_word();
        // While some connection's work is held or completes on the late
        // timeline, completion differs by connection: the service answers
        // each poll, and no waiter publishes the timeline's value.
        let lost = device.is_lost();
        let reached = if lost || device.holds() || refusing {
            POLL_UNAVAILABLE
        } else {
            serial
        };
        self.time_submissions(submitted, if lost { reached } else { serial });
        self.0.submitted.store(submitted, Ordering::Release);
        // The marker of a live device, for readers outside the turn; a lost
        // device's is cleared before anything can replace the device.
        match word {
            Some(word) => self.0.marker.store(word.as_ptr(), Ordering::SeqCst),
            None => self.0.forget_marker(),
        }
        // Publish one snapshot before making the completion observable. A
        // waiter may have published past what this holder saw; a lost device
        // answers no poll, and its replacement starts again.
        let shown = self.0.reached.load(Ordering::Acquire);
        if reached == POLL_UNAVAILABLE || shown == POLL_UNAVAILABLE || shown > submitted {
            self.0.reached.store(reached, Ordering::Release);
        } else {
            self.0.reached.fetch_max(reached, Ordering::AcqRel);
        }
        if completed || self.retiring() {
            self.0.completed.store(true, Ordering::Release);
        }
        self.0.locked.store(false, Ordering::SeqCst);
        // An interrupt this turn kept from looking is the platform's to look
        // at again: a guest call's processor would wait for the boot
        // processor to answer the query on its behalf.
        if self.0.stale.load(Ordering::SeqCst) {
            (self.0.host.wake)();
        }
    }
}

impl<B: Backend, C: Compiler> SharedService<B, C> {
    /// The newest serial the device's completion marker shows, read from
    /// the host's view of the word the GPU writes after each submission
    /// ([`crate::metal::device::Device::completion_word`]): no driver call,
    /// no interrupt, no Linux work, from any processor. Its 32 bits are
    /// widened against the newest serial submitted.
    pub(super) fn marker_value(&self) -> Option<u64> {
        self.marker_readers.fetch_add(1, Ordering::SeqCst);
        let word = self.marker.load(Ordering::SeqCst);
        // SAFETY: a turn clears the pointer and waits for its readers before
        // the device that owns the word can go (`forget_marker`).
        let low = (!word.is_null()).then(|| unsafe { (*word).load(Ordering::Acquire) });
        self.marker_readers.fetch_sub(1, Ordering::SeqCst);
        let submitted = self.submitted.load(Ordering::Acquire);
        let value = (submitted & !0xffff_ffff) | u64::from(low?);
        Some(if value > submitted {
            value.checked_sub(1 << 32)?
        } else {
            value
        })
    }

    /// Clears the marker for readers and waits for those reading it.
    fn forget_marker(&self) {
        if self
            .marker
            .swap(core::ptr::null_mut(), Ordering::SeqCst)
            .is_null()
        {
            return;
        }
        while self.marker_readers.load(Ordering::SeqCst) != 0 {
            core::hint::spin_loop();
        }
    }
}

impl<B: Backend, C: Compiler + 'static> SharedService<B, C> {
    pub fn new(mut service: Service<B, C>, host: ServiceHost) -> Self {
        service.device().set_clock(host.ticks);
        // Completion needs no invalidation when every host-visible memory
        // type is coherent.
        let publishable = service
            .device()
            .backend()
            .physical_device()
            .memory
            .types
            .iter()
            .all(|kind| {
                !kind.properties.contains(MemoryProperties::HOST_VISIBLE)
                    || kind.properties.contains(MemoryProperties::HOST_COHERENT)
            });
        // The AIR compiler keeps what it reflects and compiles for the boot,
        // and defers its compilations once the platform runs jobs.
        let compile_lock = air(service.device().compiler_mut()).map(|compiler| {
            keep_compilations(compiler);
            compiler.compile_lock(host.relax)
        });
        // Refused submissions not yet answered, which keep polls from the
        // published serial.
        let refusals = Arc::new(AtomicU32::new(0));
        service.device().count_failures(&refusals);
        // A device reopened after a loss keeps the compilations too, under
        // the lock the jobs hold, and counts its own refusals from none.
        let lock = compile_lock.clone();
        let relax = host.relax;
        let counted = refusals.clone();
        service.prepare_reopened(Box::new(move |device| {
            device.count_failures(&counted);
            if let Some(lock) = &lock
                && let Some(compiler) = air(device.compiler_mut())
            {
                keep_compilations(compiler);
                compiler.share_compile_lock(lock.clone(), relax);
            }
        }));
        Self {
            service: UnsafeCell::new(service),
            locked: AtomicBool::new(false),
            stale: AtomicBool::new(false),
            completed: AtomicBool::new(false),
            reached: AtomicU64::new(0),
            polling: [const { AtomicU64::new(0) }; POLLING],
            pending: UnsafeCell::new(Pending {
                submitted: 0,
                ring: [(0, 0); PENDING],
                head: 0,
                len: 0,
            }),
            latency: [const { AtomicU64::new(0) }; 3],
            timeline: AtomicU64::new(0),
            submitted: AtomicU64::new(0),
            waited: AtomicU64::new(0),
            publishable: AtomicBool::new(publishable),
            early: [const { (AtomicU64::new(0), AtomicU64::new(0)) }; PENDING],
            compile_stack: AtomicU64::new(0),
            wait_bound: AtomicU64::new(0),
            job_lock: JobLock {
                lock: compile_lock.unwrap_or_default(),
                relax: host.relax,
            },
            jobs: Spin::new(Jobs::default()),
            staging: Spin::new(Vec::new()),
            library_budget: AtomicU64::new(0),
            library_piece: AtomicUsize::new(LIBRARY_PIECE_BYTES),
            imported: Spin::new(RecentImports::default()),
            outside_stack: AtomicU64::new(0),
            outside_locked: AtomicBool::new(false),
            outside_seen: Spin::new(None),
            marker: AtomicPtr::new(core::ptr::null_mut()),
            marker_readers: AtomicU32::new(0),
            upkeep_asked: AtomicBool::new(false),
            look_budget: AtomicU64::new(0),
            refusals,
            host,
        }
    }

    /// Defers the compilations no cache holds to jobs, which the platform
    /// runs on `stack`, a stack as deep as the service's own, by
    /// [`Self::run_job`].
    pub fn set_compile_stack(&self, stack: u64) {
        self.compile_stack.store(stack, Ordering::Release);
    }

    /// Bounds a frontend call's wait for the turn to `ticks`: a call that
    /// waited so long ran nothing, so it is answered `Pending` and the
    /// frontend makes it again with its processor back in the guest, which
    /// takes no interrupt while it waits here.
    pub fn set_wait_bound(&self, ticks: u64) {
        self.wait_bound.store(ticks, Ordering::Release);
    }

    /// Runs imports' driver work outside the turn on `stack`, a stack as
    /// deep as the service's own ([`crate::metal::vulkan::backend::Outside`]).
    pub fn set_outside_stack(&self, stack: u64) {
        self.outside_stack.store(stack, Ordering::Release);
    }

    /// Runs `body` on the outside stack under the outside lock, holding no
    /// turn. Waiting for the lock relaxes, as a wait for the turn does.
    fn outside<R>(&self, stack: u64, body: impl FnOnce() -> R) -> R {
        while self
            .outside_locked
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            (self.host.relax)();
        }
        let (mut body, mut result) = (Some(body), None);
        (self.host.run)(stack, &mut || {
            result = body.take().map(|body| body());
        });
        self.outside_locked.store(false, Ordering::Release);
        result.expect("the platform runs what it is given")
    }

    /// Runs the frees a frontend call's turn let go of, unless another
    /// caller holds the outside lock: a frontend call waits for nothing
    /// but its bounded turn, and the frees wait for the next call (the
    /// backend frees in its turn past its queue's bound).
    fn try_outside_frees(&self, outside: &dyn Outside, stack: u64) {
        if self
            .outside_locked
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            return;
        }
        (self.host.run)(stack, &mut || {
            outside.run_frees();
        });
        self.outside_locked.store(false, Ordering::Release);
    }

    /// The backend's outside work and stack, for after this turn; tells
    /// the backend a service runs it, and keeps it for the platform's
    /// upkeep.
    fn after_turn(&self, turn: &mut Turn<'_, B, C>) -> Option<(Arc<dyn Outside>, u64)> {
        let stack = self.outside_stack.load(Ordering::Acquire);
        if stack == 0 {
            return None;
        }
        let outside = turn.device().outside()?;
        outside.enable();
        self.outside_seen.with(|seen| {
            let same = seen
                .as_ref()
                .is_some_and(|seen| core::ptr::addr_eq(Arc::as_ptr(seen), Arc::as_ptr(&outside)));
            if !same {
                *seen = Some(outside.clone());
            }
        });
        Some((outside, stack))
    }

    /// Asks the platform for an upkeep ([`Self::upkeep`]) when the outside
    /// work has some and none was asked for yet.
    fn ask_upkeep(&self, outside: &dyn Outside) {
        if outside.upkeep_due() && !self.upkeep_asked.swap(true, Ordering::AcqRel) {
            (self.host.wake)();
        }
    }

    /// Bounds what the platform's look ([`Self::interrupted`]) does on its
    /// processor, whose guest waits meanwhile, to about `ticks` per part:
    /// the destruction of released objects its retirement frees (the rest
    /// waits for the next look or call) and its upkeep ([`Self::upkeep`]).
    /// Zero bounds the upkeep by its count of pieces alone and leaves the
    /// destruction unbounded.
    pub fn set_look_budget(&self, ticks: u64) {
        self.look_budget.store(ticks, Ordering::Release);
    }

    /// The backend's work after its turns ([`Outside::upkeep`]): frees,
    /// destruction of released semaphores and semaphores made ahead, which
    /// the platform runs where a host kernel call costs least, on the
    /// outside stack and holding no turn. Bounded by [`UPKEEP_PIECES`] and
    /// the budget; what is left asks for the next. Nothing when another
    /// caller holds the outside lock: it asks again once it leaves.
    /// Answers the pieces done.
    pub fn upkeep(&self) -> usize {
        let stack = self.outside_stack.load(Ordering::Acquire);
        let Some(outside) = self.outside_seen.with(|seen| seen.clone()) else {
            return 0;
        };
        if stack == 0 || !outside.upkeep_due() {
            self.upkeep_asked.store(false, Ordering::Release);
            return 0;
        }
        let locked =
            self.outside_locked
                .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed);
        self.upkeep_asked.store(false, Ordering::Release);
        if locked.is_err() {
            return 0;
        }
        let (budget, began) = (
            self.look_budget.load(Ordering::Acquire),
            (self.host.ticks)(),
        );
        let mut done = 0;
        let mut pieces = 0;
        (self.host.run)(stack, &mut || {
            done = outside.upkeep(&mut || {
                pieces += 1;
                pieces <= UPKEEP_PIECES
                    && (budget == 0 || (self.host.ticks)().wrapping_sub(began) < budget)
            });
        });
        self.outside_locked.store(false, Ordering::Release);
        // A piece that cannot be done (a semaphore the driver refuses) asks
        // for no upkeep after it: the turns then make their own.
        if done != 0 {
            self.ask_upkeep(&*outside);
        }
        done
    }

    /// Runs the oldest deferred job, when one has waited `older_than` ticks
    /// and no other runs: the platform calls it where long host work is
    /// allowed, between guest slices and never inside a guest's call, as a
    /// job runs whole, the compilers having no point to stop at. Whether
    /// one ran.
    pub fn run_job(&self, older_than: u64) -> bool {
        let stack = self.compile_stack.load(Ordering::Acquire);
        if stack == 0 {
            return false;
        }
        let Some(held) = self.job_lock.lock.try_hold() else {
            return false;
        };
        let now = (self.host.ticks)();
        let taken = self.jobs.with(|jobs| {
            let &(_, queued) = jobs.queued.front()?;
            if now.wrapping_sub(queued) < older_than {
                return None;
            }
            let (job, queued) = jobs.queued.pop_front()?;
            jobs.running = Some(job.key);
            Some((job, queued))
        });
        let Some((job, queued)) = taken else {
            return false;
        };
        let mut job = Some(job);
        let mut done = None;
        (self.host.run)(stack, &mut || {
            done = job.take().map(|job| job.run(&held));
        });
        drop(held);
        let ran = (self.host.ticks)().wrapping_sub(now);
        self.jobs.with(|jobs| {
            jobs.running = None;
            jobs.done.extend(done);
            let report = &mut jobs.report;
            report.ran += 1;
            report.ticks += ran;
            report.longest = report.longest.max(ran);
            report.longest_queued = report.longest_queued.max(now.wrapping_sub(queued));
        });
        true
    }

    /// Whether a deferred job waits to run.
    pub fn jobs_waiting(&self) -> bool {
        self.jobs.with(|jobs| !jobs.queued.is_empty())
    }

    /// What jobs did since the last call.
    pub fn take_job_report(&self) -> JobReport {
        self.jobs.with(|jobs| core::mem::take(&mut jobs.report))
    }

    /// Hands what jobs made to their owners, in a turn.
    fn adopt_jobs(&self, turn: &mut Turn<'_, B, C>) {
        let done = self.jobs.with(|jobs| core::mem::take(&mut jobs.done));
        for done in done {
            match done.owner {
                Owner::Compiler => turn.device().compiler_mut().adopt_job(done),
                Owner::Backend => turn.device().backend().adopt_job(done),
            }
        }
    }

    /// Answers the call at `record` `Pending` without a turn when it was
    /// answered so before and its job has not run. The job never runs here:
    /// a guest's call stays short however long its job waits, and the job
    /// runs where [`Self::run_job`] is called.
    fn still_pending(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
    ) -> bool {
        let Some(call) = read_call(record, caller) else {
            return false;
        };
        let pending = self.jobs.with(|jobs| {
            let at = jobs.parked.iter().position(|parked| {
                (
                    parked.connection,
                    parked.record,
                    parked.operation,
                    parked.request,
                ) == (
                    connection.get(),
                    record,
                    call.operation,
                    call.request.address,
                )
            })?;
            let key = jobs.parked[at].key;
            let waits =
                jobs.running == Some(key) || jobs.queued.iter().any(|(job, _)| job.key == key);
            if waits {
                jobs.report.repeated += 1;
            } else {
                jobs.parked.remove(at);
            }
            Some(waits)
        });
        if pending != Some(true) {
            return false;
        }
        answer_pending(record, call, caller).is_ok()
    }

    /// Bounds how long a `NewLibrary` call reads and digests its library at
    /// one run to about `ticks`, before its turn and holding none: a library
    /// of hundreds of megabytes is read over many runs answered `Pending`,
    /// rather than in one exit holding every other call's turn.
    pub fn set_library_budget(&self, ticks: u64) {
        self.library_budget.store(ticks, Ordering::Release);
    }

    /// Reads libraries `bytes` at a time, so small ones take several runs.
    #[cfg(test)]
    pub(crate) fn set_library_piece(&self, bytes: usize) {
        self.library_piece.store(bytes.max(1), Ordering::Release);
    }

    /// Reads the next piece of the library of the `NewLibrary` call at
    /// `record`, before its turn and outside every lock, and digests it.
    fn stage_library(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
    ) -> Staged {
        let Some(call) = read_call(record, caller) else {
            return Staged::Not;
        };
        let mut raw = [0; size_of::<NewLibraryRequest>()];
        if call.operation != Operation::NewLibrary.raw()
            || call.request.length != raw.len() as u64
            || caller.read(call.request.address, &mut raw).is_err()
        {
            return Staged::Not;
        }
        let Ok(NewLibraryRequest { data: span }) = wire::read(&raw) else {
            return Staged::Not;
        };
        let Ok(length) = usize::try_from(span.length) else {
            return Staged::Not;
        };
        let at = (connection.get(), record, span);
        let kept = self.staging.with(|staging| {
            let found = staging
                .iter()
                .position(|staged| (staged.connection, staged.record, staged.span) == at)?;
            Some(staging.remove(found))
        });
        let piece = self.library_piece.load(Ordering::Acquire);
        let mut staged = match kept {
            Some(staged) => staged,
            None => {
                let mut buffer = Vec::new();
                // Without room the turn reports the failure.
                if buffer.try_reserve_exact(piece.min(length)).is_err() {
                    return Staged::Not;
                }
                Staging {
                    connection: connection.get(),
                    record,
                    span,
                    read: 0,
                    digest: Sha256::new(),
                    piece: buffer,
                    kept: None,
                }
            }
        };
        // The second read keeps every byte: the one allocation of the
        // library's size, made only for a library the device does not hold.
        if let Some(kept) = &mut staged.kept
            && kept.capacity() < length
            && kept.try_reserve_exact(length).is_err()
        {
            return Staged::Not;
        }
        let budget = self.library_budget.load(Ordering::Acquire);
        let began = (self.host.ticks)();
        let found = loop {
            let start = staged.read;
            if start == length {
                break Staged::Ready;
            }
            let end = start.saturating_add(piece).min(length);
            let target = match &mut staged.kept {
                Some(kept) => {
                    kept.resize(end, 0);
                    &mut kept[start..end]
                }
                None => {
                    staged.piece.resize(end - start, 0);
                    &mut staged.piece[..end - start]
                }
            };
            if let Err(error) = caller.read(span.address + start as u64, target) {
                if let Some(kept) = &mut staged.kept {
                    kept.truncate(start);
                }
                match error {
                    CallerFault::Denied { .. } => break Staged::Unmapped,
                    _ => return Staged::Not,
                }
            }
            staged.digest.update(&*target);
            staged.read = end;
            if end < length && (self.host.ticks)().wrapping_sub(began) >= budget {
                break Staged::Pending;
            }
        };
        self.staging.with(|staging| {
            if staging.len() == STAGED {
                staging.remove(0);
            }
            staging.push(staged);
        });
        found
    }

    /// The library the call at `record` read whole before this turn.
    fn staged_library(&self, connection: NonZeroU64, record: u64) -> Option<StagedLibrary> {
        let staged = self.staging.with(|staging| {
            let found = staging.iter().position(|staged| {
                (staged.connection, staged.record) == (connection.get(), record)
                    && staged.read as u64 == staged.span.length
            })?;
            Some(staging.remove(found))
        })?;
        Some(StagedLibrary {
            span: staged.span,
            bytes: staged.kept,
            digest: staged.digest.finalize().into(),
        })
    }

    /// Reads the library of the call at `record` again, keeping its bytes,
    /// once its turn found the device holds no library of its digest.
    fn keep_library(&self, connection: NonZeroU64, record: u64, span: Span) {
        let staged = Staging {
            connection: connection.get(),
            record,
            span,
            read: 0,
            digest: Sha256::new(),
            piece: Vec::new(),
            kept: Some(Vec::new()),
        };
        self.staging.with(|staging| {
            if staging.len() == STAGED {
                staging.remove(0);
            }
            staging.push(staged);
        });
    }

    /// Queues each of `jobs` unless one of its key is queued, running or
    /// done, and parks the call at `record` on the first.
    fn park(&self, connection: NonZeroU64, record: u64, call: GateCall, jobs: Vec<Job>) {
        let key = jobs[0].key;
        let parked = Parked {
            connection: connection.get(),
            record,
            operation: call.operation,
            request: call.request.address,
            key,
        };
        let now = (self.host.ticks)();
        let new = jobs;
        let unwanted = self.jobs.with(|jobs| {
            jobs.report.deferred += 1;
            jobs.parked.retain(|other| {
                (other.connection, other.record) != (parked.connection, parked.record)
            });
            if jobs.parked.len() == PARKED {
                jobs.parked.pop_front();
            }
            jobs.parked.push_back(parked);
            let mut unwanted = Vec::new();
            for job in new {
                let known = jobs.running == Some(job.key)
                    || jobs.queued.iter().any(|(queued, _)| queued.key == job.key)
                    || jobs.done.iter().any(|done| done.key == job.key);
                if known {
                    unwanted.push(job);
                } else {
                    jobs.queued.push_back((job, now));
                }
            }
            unwanted
        });
        // Dropped outside the lock: a job frees what it holds.
        drop(unwanted);
    }

    /// The compiler's counts, unless a call holds the service now.
    pub fn compiler_report(&self) -> Option<alloc::string::String> {
        let mut turn = self.try_turn()?;
        air(turn.device().compiler_mut()).map(|compiler| compiler.report())
    }

    /// A descriptor of the device's completion timeline, once per device,
    /// unless a call holds the service now, with the timeline's generation
    /// for [`Self::completed_through`]: the platform waits on it to look
    /// for completed work as soon as some completes.
    pub fn take_completion(&self) -> Option<(i32, u64)> {
        let fd = self
            .try_turn()
            .and_then(|mut turn| turn.device().take_completion())?;
        Some((fd, self.timeline.fetch_add(1, Ordering::AcqRel) + 1))
    }

    /// Offers the device's completion timeline again
    /// ([`Self::take_completion`]), for a waiter started in place of one that
    /// left by itself ([`super::waiters`]): whether one is offered. False
    /// while a call holds the service, or once the device is lost.
    pub fn rearm_completion(&self) -> bool {
        self.try_turn()
            .is_some_and(|mut turn| turn.device().rearm_completion())
    }

    /// Whether work was submitted that is not yet published complete: a
    /// waiter whose wait times out then has the platform look at the device,
    /// which notices a loss the stalled timeline never signals. Not once
    /// polls are unavailable: the loss is already known.
    pub fn awaits_completion(&self) -> bool {
        let reached = self.reached.load(Ordering::Acquire);
        reached != POLL_UNAVAILABLE && self.submitted.load(Ordering::Acquire) > reached
    }

    /// The platform's waiter saw the completion timeline of generation
    /// `timeline` reach `value`, every serial through it complete. Published
    /// at once when a waiter may publish, so a call holding the service
    /// delays no frame; the platform still looks at the device
    /// ([`Self::interrupted`]) once a turn is free.
    pub fn completed_through(&self, timeline: u64, value: u64) {
        if timeline == self.timeline.load(Ordering::Acquire) && value < WAITED_GENERATION {
            self.waited
                .fetch_max((timeline & 0xffff) << 48 | value, Ordering::AcqRel);
        }
        if timeline == self.timeline.load(Ordering::Acquire) {
            self.publish(value);
        }
        // The look itself is the platform's, outside the waiter's context
        // (`interrupted`): it enters the driver.
        self.stale.store(true, Ordering::SeqCst);
    }

    /// Publishes every serial through `value` complete, when a waiter or
    /// the marker may publish: the guest is told at once, whatever holds
    /// the service.
    fn publish(&self, value: u64) {
        if !self.publishable.load(Ordering::Acquire) || self.refusing() {
            return;
        }
        let value = value.min(self.submitted.load(Ordering::Acquire));
        let mut shown = self.reached.load(Ordering::Acquire);
        while shown != POLL_UNAVAILABLE && shown < value {
            match self.reached.compare_exchange_weak(
                shown,
                value,
                Ordering::AcqRel,
                Ordering::Acquire,
            ) {
                Ok(_) => {
                    let now = (self.host.ticks)();
                    // The newest PENDING serials are those timed.
                    for serial in shown.max(value.saturating_sub(PENDING as u64)) + 1..=value {
                        let (published, when) = &self.early[serial as usize % PENDING];
                        when.store(now, Ordering::Relaxed);
                        published.store(serial, Ordering::Release);
                    }
                    self.completed.store(true, Ordering::Release);
                    break;
                }
                Err(now) => shown = now,
            }
        }
    }

    /// Publishes what the completion marker shows past what is published,
    /// while work is outstanding.
    fn publish_marker(&self) {
        if self.awaits_completion()
            && let Some(value) = self.marker_value()
        {
            self.publish(value);
        }
    }

    /// Submissions seen complete since the last call, the ticks from their
    /// submission until a holder saw them complete, and the longest of those.
    pub fn take_latency(&self) -> (u64, u64, u64) {
        (
            self.latency[0].swap(0, Ordering::Relaxed),
            self.latency[1].swap(0, Ordering::Relaxed),
            self.latency[2].swap(0, Ordering::Relaxed),
        )
    }

    /// Whether a refused submission's command buffer is not yet answered.
    fn refusing(&self) -> bool {
        self.refusals.load(Ordering::Acquire) != 0
    }

    fn polls(&self, connection: NonZeroU64) -> bool {
        self.polling
            .iter()
            .any(|slot| slot.load(Ordering::Acquire) == connection.get())
    }

    /// Answers `connection`'s `Poll` at `record` from `reached`, or leaves
    /// the call to the service: any other operation or record, a connection
    /// the service never answered a poll for, a lost device, held work, or
    /// a span out of the caller's reach.
    pub(super) fn poll_published(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
    ) -> bool {
        let mut raw = [0; size_of::<GateCall>()];
        if caller.read(record, &mut raw).is_err() {
            return false;
        }
        let mut call: GateCall = wire::read(&raw).expect("a record of its own size");
        let (request_bytes, response_bytes) = Operation::Poll.sizes();
        if call.operation != Operation::Poll.raw()
            || call.request.length != request_bytes as u64
            || call.response.length != response_bytes as u64
        {
            return false;
        }
        // What the GPU finished, as it wrote the marker.
        self.publish_marker();
        let reached = self.reached.load(Ordering::Acquire);
        // Check membership after acquiring the serial. A close that
        // precedes its publication has already removed that connection.
        // A refusal counted before its signals showed keeps this serial
        // from answering for the command buffer that failed.
        if reached == POLL_UNAVAILABLE || !self.polls(connection) || self.refusing() {
            return false;
        }
        let value = wire::PollResponse {
            value: reached,
            refused: 0,
        };
        let mut response = [0; size_of::<wire::PollResponse>()];
        wire::write(&value, &mut response).expect("a record of its own size");
        if caller.write(call.response.address, &response).is_err() {
            return false;
        }
        call.status = Status::Ok.raw();
        call.reason_length = 0;
        wire::write(&call, &mut raw).expect("a record of its own size");
        caller.write(record, &raw).is_ok()
    }

    /// Whether the call at `record` creates a pipeline or a texture or
    /// commits commands, whose compilations and new textures' kernel memory
    /// may be made outside the lock: nothing it does before it defers stays.
    /// A commit's replay compiles the variants its draws and dispatches
    /// need, and its command buffer is discarded when it defers one; once
    /// replay changes an indirect command buffer, it compiles in its turn
    /// ([`Compiler::stop_deferring`]).
    fn defers(&self, record: u64, caller: &mut dyn CallerMemory) -> bool {
        let mut raw = [0; size_of::<GateCall>()];
        if caller.read(record, &mut raw).is_err() {
            return false;
        }
        let call: GateCall = wire::read(&raw).expect("a record of its own size");
        [
            Operation::NewSourceLibrary,
            Operation::NewStitchedLibrary,
            Operation::NewFunction,
            Operation::FunctionBindings,
            Operation::ArgumentLayout,
            Operation::NewRenderPipeline,
            Operation::NewMeshPipeline,
            Operation::NewComputePipeline,
            Operation::NewTilePipeline,
            Operation::NewTexture,
            Operation::Commit,
        ]
        .iter()
        .any(|operation| operation.raw() == call.operation)
    }

    pub(super) fn try_turn(&self) -> Option<Turn<'_, B, C>> {
        let mut turn = (!self.locked.swap(true, Ordering::SeqCst)).then(|| Turn(self, false))?;
        // A lost device may be replaced in this turn, its marker with it.
        if !self.marker.load(Ordering::Relaxed).is_null() && turn.device().is_lost() {
            self.forget_marker();
        }
        Some(turn)
    }

    fn turn(&self) -> Turn<'_, B, C> {
        loop {
            if let Some(turn) = self.try_turn() {
                return turn;
            }
            (self.host.relax)();
        }
    }

    /// The turn, unless the wait bound ([`Self::set_wait_bound`]) passes
    /// first, counted from `rang`, or the platform ends the wait
    /// ([`ServiceHost::relax_within`]). Without a bound it waits as
    /// [`Self::turn`] does.
    fn turn_within(&self, rang: u64) -> Option<Turn<'_, B, C>> {
        let bound = self.wait_bound.load(Ordering::Acquire);
        loop {
            if let Some(turn) = self.try_turn() {
                return Some(turn);
            }
            if bound == 0 {
                (self.host.relax)();
                continue;
            }
            if (self.host.ticks)().wrapping_sub(rang) >= bound || !(self.host.relax_within)() {
                return None;
            }
        }
    }

    /// Runs `call` on the service in a turn of its own.
    fn call<R>(&self, call: impl FnOnce(&mut Service<B, C>) -> R) -> R {
        let mut turn = self.turn();
        turn.catch_up();
        let (mut call, mut result) = (Some(call), None);
        (self.host.run)(self.host.stack, &mut || {
            if let Some(call) = call.take() {
                result = Some(call(&mut turn));
            }
        });
        result.expect("the platform runs what it is given")
    }

    /// What the service holds, unless a call holds it now: a report never
    /// waits for the service.
    pub fn memory(&self) -> Option<super::MemoryReport> {
        self.try_turn().map(|turn| turn.memory())
    }

    /// Where retired command buffers' time went since the last call, unless
    /// a call holds the service now; it is then counted for the next.
    pub fn gpu_times(&self) -> Option<crate::metal::device::gpu_times::GpuTimes> {
        self.try_turn()
            .map(|mut turn| turn.device().take_gpu_times())
    }

    /// `--profile`'s GPU time per pass since the last call, unless a call
    /// holds the service now or passes are not profiled
    /// ([`set_pass_profile`](crate::metal::device::pass_times::set_pass_profile)).
    pub fn pass_times(&self) -> Option<crate::metal::device::pass_times::PassTimes> {
        self.try_turn()
            .and_then(|mut turn| turn.device().take_pass_times())
    }

    /// Whether the device has GPU timestamps, without which passes are not
    /// timed; `None` while a call holds the service.
    pub fn has_timestamps(&self) -> Option<bool> {
        self.try_turn()
            .map(|mut turn| turn.device().capabilities().timestamp_frequency != 0)
    }

    /// The GPU interrupted: look for what completed now or, while a call
    /// holds the service, once it leaves.
    pub fn interrupted(&self) {
        self.stale.store(true, Ordering::SeqCst);
        if let Some(mut turn) = self.try_turn() {
            turn.look();
        }
        // The platform looks where a host kernel call costs least, which is
        // where the work after the turns costs least too.
        if self.upkeep_asked.load(Ordering::Acquire) {
            self.upkeep();
        }
    }

    /// Closes admission while holding the same turn as guest calls, so a
    /// call either precedes closure or observes it.
    pub fn begin_quiesce(&self) {
        self.call(|service| service.begin_quiesce());
    }

    /// Advances the native owner's GPU stop protocol on the service stack.
    /// An idle answer does not disconnect the gate or invalidate DMA maps.
    pub fn quiesce_step(&self) -> crate::metal::device::lifecycle::QuiesceStatus {
        self.call(|service| service.quiesce_step())
    }
}

impl<B: Backend + Send, C: Compiler + Send + 'static> GateService for SharedService<B, C> {
    fn connect(&self) -> Result<NonZeroU64, &'static str> {
        self.call(|service| service.connect()).map_err(|fault| {
            (self.host.report)(&alloc::format!("{fault} in a connection"));
            fault.reason
        })
    }

    fn disconnect(&self, connection: NonZeroU64) {
        self.jobs.with(|jobs| {
            jobs.parked
                .retain(|parked| parked.connection != connection.get());
        });
        self.staging
            .with(|staging| staging.retain(|staged| staged.connection != connection.get()));
        // The gate closes only connections it opened and holds.
        let closed = self.call(|service| {
            for slot in &self.polling {
                let _ =
                    slot.compare_exchange(connection.get(), 0, Ordering::AcqRel, Ordering::Acquire);
            }
            service.disconnect(connection)
        });
        debug_assert!(closed.is_ok());
    }

    fn serve(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
    ) -> Result<(), CallerFault> {
        let rang = (self.host.ticks)();
        let served_at_once = |operation: u32, status: Status| {
            let left = (self.host.ticks)();
            (self.host.served)(Served {
                operation,
                status,
                waited: 0,
                held: left.wrapping_sub(rang),
                progress: None,
                phases: None,
                compiled: 0,
                deferrals: u32::from(status == Status::Pending),
                longest_wait: 0,
                gave_up: false,
            });
        };
        if self.still_pending(connection, record, caller) {
            let operation = read_call(record, caller).map_or(0, |call| call.operation);
            served_at_once(operation, Status::Pending);
            return Ok(());
        }
        // A poll answers from the completion a turn or the waiter published,
        // whether or not a call holds the service: a turn would ask the
        // device again, a timeline query the host's Linux environment
        // serves while this PE spins, and every completion is published
        // before the kext wakes a frontend to poll.
        if self.poll_published(connection, record, caller) {
            served_at_once(Operation::Poll.raw(), Status::Ok);
            return Ok(());
        }
        // A library's bytes are read a piece at each run, before any turn.
        let staged = match self.stage_library(connection, record, caller) {
            Staged::Not => false,
            Staged::Ready => true,
            staged @ (Staged::Pending | Staged::Unmapped) => {
                let status = match staged {
                    Staged::Pending => Status::Pending,
                    _ => Status::Unmapped,
                };
                answer(record, read_record(record, caller)?, status, caller)?;
                served_at_once(Operation::NewLibrary.raw(), status);
                return Ok(());
            }
        };
        let Some(mut turn) = self.turn_within(rang) else {
            // Nothing of the call ran: it is made again once the frontend
            // has had its processor back.
            let call = read_record(record, caller)?;
            answer_pending(record, call, caller)?;
            let left = (self.host.ticks)();
            (self.host.served)(Served {
                operation: call.operation,
                status: Status::Pending,
                waited: left.wrapping_sub(rang),
                held: 0,
                progress: None,
                phases: None,
                compiled: 0,
                deferrals: 0,
                longest_wait: left.wrapping_sub(rang),
                gave_up: true,
            });
            return Ok(());
        };
        let entered = (self.host.ticks)();
        let waited = entered.wrapping_sub(rang);
        let deferring =
            self.compile_stack.load(Ordering::Acquire) != 0 && self.defers(record, caller);
        turn.catch_up();
        let job_lock = deferring.then_some(&self.job_lock);
        let mut answer = None;
        self.job_lock.lock.set_deferred(false);
        (self.host.run)(self.host.stack, &mut || {
            self.adopt_jobs(&mut turn);
            turn.device().compiler_mut().defer_compiles(deferring);
            turn.device().backend().defer_pipelines(job_lock);
            let mut deferrable = Deferrable {
                caller: &mut *caller,
                lock: job_lock.map(|jobs| &*jobs.lock),
            };
            if staged {
                turn.staged_library = self.staged_library(connection, record);
            }
            let served = turn.serve(connection, record, &mut deferrable);
            turn.staged_library = None;
            if let Some(span) = turn.library_wanted.take() {
                self.keep_library(connection, record, span);
            }
            let device = turn.device();
            // A call that went on past a deferral may have left both.
            let jobs: Vec<Job> = [
                device.compiler_mut().take_job(),
                device.backend().take_job(),
            ]
            .into_iter()
            .flatten()
            .collect();
            device.compiler_mut().defer_compiles(false);
            device.backend().defer_pipelines(None);
            answer = Some((
                served,
                core::mem::take(&mut turn.refused),
                turn.noted.take(),
                turn.served,
                jobs,
            ));
        });
        let (mut served, mut refused, noted, (mut operation, mut status), jobs) =
            answer.expect("the platform runs what it is given");
        if !jobs.is_empty() {
            // The call left nothing but the compilation: it is answered
            // `Pending`, and made again once the job has run.
            let deferral = alloc::format!("{}", super::wire::Fault::from(compiler::DEFERRED));
            refused.retain(|reason| !reason.contains(&deferral));
            // The run's own writes were dropped once it deferred, so a
            // record that takes no `Pending` is untouched: the caller learns
            // why from the gate's fault, and makes the call again.
            let answered = read_record(record, caller)
                .and_then(|call| answer_pending(record, call, caller).map(|()| call));
            match answered {
                Ok(call) => {
                    self.park(connection, record, call, jobs);
                    (operation, status, served) = (call.operation, Status::Pending, Ok(()));
                }
                Err(fault) => {
                    drop(jobs);
                    served = Err(fault);
                }
            }
        }
        if let Some(compiler) = air(turn.device().compiler_mut()) {
            compiler.end_call();
        }
        if served.is_ok()
            && operation == Operation::Poll.raw()
            && status == Status::Ok
            && !self.polls(connection)
        {
            // The turn serializes enrollment with other polls and close.
            let _ = self.polling.iter().find(|slot| {
                slot.compare_exchange(0, connection.get(), Ordering::AcqRel, Ordering::Acquire)
                    .is_ok()
            });
        }
        let progress = turn.device().progress();
        let phases = turn.device().take_phases();
        let outside = self.after_turn(&mut turn);
        drop(turn);
        let left = (self.host.ticks)();
        (self.host.served)(Served {
            operation,
            status,
            waited,
            held: left.wrapping_sub(entered),
            progress: Some(progress),
            phases,
            compiled: 0,
            deferrals: u32::from(status == Status::Pending),
            longest_wait: waited,
            gave_up: false,
        });
        for reason in &refused {
            (self.host.report)(reason);
        }
        if let Some(text) = noted {
            (self.host.note)(&text);
        }
        // The frees the call's turn let go of, holding no turn, when the
        // platform's upkeep fell behind; the rest is the upkeep's.
        if let Some((outside, stack)) = outside {
            if outside.frees_pending() {
                self.try_outside_frees(&*outside, stack);
            }
            self.ask_upkeep(&*outside);
        }
        served
    }

    fn import(
        &self,
        connection: NonZeroU64,
        ranges: &[MemoryRange],
        token: NonZeroU64,
    ) -> Result<NonZeroU64, &'static str> {
        let rang = (self.host.ticks)();
        let again = self.imported.with(|recent| recent.again(ranges));
        let mut turn = self.turn();
        let entered = (self.host.ticks)();
        turn.catch_up();
        // An import nothing kept is exported and imported by the driver
        // after the turn: those host kernel calls wait for the boot
        // processor, and no other connection's call should wait with them.
        let stack = self.outside_stack.load(Ordering::Acquire);
        let mut prepared = None;
        let mut away = 0;
        if stack != 0
            && let Some(outside) = turn.device().import_outside(ranges)
        {
            drop(turn);
            let left = (self.host.ticks)();
            prepared = Some(self.outside(stack, || outside.prepare_import(ranges)));
            turn = self.turn();
            away = (self.host.ticks)().wrapping_sub(left);
            turn.catch_up();
        }
        let mut imported = None;
        (self.host.run)(self.host.stack, &mut || {
            imported = Some(match prepared.take() {
                // An import the driver refused, or a device that closed.
                Some(Err(error)) => Err(error.into()),
                Some(Ok(made)) => turn.import_with(connection, ranges, token, made),
                None => turn.import(connection, ranges, token),
            });
        });
        let outside = self.after_turn(&mut turn);
        drop(turn);
        let imported = imported.expect("the platform runs what it is given");
        // Frees the turn let go of, after it too.
        if let Some((outside, stack)) = outside {
            if outside.frees_pending() {
                self.outside(stack, || outside.run_frees());
            }
            self.ask_upkeep(&*outside);
        }
        // The kext's import holds the turn as a frontend call does, and its
        // host kernel calls are its own in the platform's count
        // (`GPU CALLS`, 0xff01 and 0xff02).
        let left = (self.host.ticks)();
        (self.host.served)(Served {
            operation: if again {
                IMPORT_AGAIN_OPERATION
            } else {
                IMPORT_OPERATION
            },
            status: imported
                .as_ref()
                .map_or_else(|fault| fault.status, |_| Status::Ok),
            waited: entered.wrapping_sub(rang),
            // The driver work outside the turn held nothing.
            held: left.wrapping_sub(entered).wrapping_sub(away),
            progress: None,
            phases: None,
            compiled: 0,
            deferrals: 0,
            longest_wait: entered.wrapping_sub(rang),
            gave_up: false,
        });
        imported.map_err(|fault| {
            let bytes: u64 = ranges.iter().map(|range| range.bytes).sum();
            (self.host.report)(&alloc::format!(
                "{fault} in an import of {} ranges, {bytes} bytes",
                ranges.len()
            ));
            fault.reason
        })
    }

    fn retired(&self) -> Option<NonZeroU64> {
        self.call(|service| service.retired())
    }

    fn quiesced(&self) -> bool {
        self.call(|service| service.quiesced())
    }

    fn signaled(&self) -> bool {
        // Whichever processor advances the gate notices what the GPU
        // finished, as soon as it wrote the marker.
        self.publish_marker();
        self.completed.swap(false, Ordering::Acquire)
    }

    fn awaits_completion(&self) -> bool {
        SharedService::awaits_completion(self)
    }

    fn bundle(&self, report: BundleReport) {
        (self.host.bundle)(report);
    }

    fn dropped(&self, reason: &'static str) {
        (self.host.report)(reason);
    }
}

/// The device's compiler, when it is the AIR compiler, whose compilations
/// can run outside the lock.
fn air<C: Compiler + 'static>(compiler: &mut C) -> Option<&mut AirCompiler> {
    (compiler as &mut dyn Any).downcast_mut::<AirCompiler>()
}

/// Makes `compiler` keep what it reflects and compiles for the boot; a warm
/// boot deserializes the NIR the pipeline store kept, and an evicted
/// compilation of this boot what the store took from it.
fn keep_compilations(compiler: &mut AirCompiler) {
    compiler.keep_compilations(compiler::NIR_BUDGET);
    #[cfg(feature = "metal-vulkan")]
    {
        use crate::metal::native::pipeline_store;
        compiler.persist(
            pipeline_store::stored_nir(),
            pipeline_store::add_nir,
            pipeline_store::recall_nir,
        );
    }
}

/// The call record at `record`, unless it is out of the caller's reach.
fn read_call(record: u64, caller: &mut dyn CallerMemory) -> Option<GateCall> {
    read_record(record, caller).ok()
}

/// The call record at `record`, or why it is out of the caller's reach.
fn read_record(record: u64, caller: &mut dyn CallerMemory) -> Result<GateCall, CallerFault> {
    let mut raw = [0; size_of::<GateCall>()];
    caller.read(record, &mut raw)?;
    Ok(wire::read(&raw).expect("a record of its own size"))
}

/// Completes `call` at `record` as `Pending`, with no reason: whether the
/// record took it.
fn answer_pending(
    record: u64,
    call: GateCall,
    caller: &mut dyn CallerMemory,
) -> Result<(), CallerFault> {
    answer(record, call, Status::Pending, caller)
}

/// Completes `call` at `record` with `status` and no reason: whether the
/// record took it.
fn answer(
    record: u64,
    mut call: GateCall,
    status: Status,
    caller: &mut dyn CallerMemory,
) -> Result<(), CallerFault> {
    call.status = status.raw();
    call.reason_length = 0;
    let mut raw = [0; size_of::<GateCall>()];
    wire::write(&call, &mut raw).expect("a record of its own size");
    caller.write(record, &raw)
}

/// The caller's memory for a run that may defer a compilation: once one is
/// deferred, the run's writes, which only answer the deferral, are dropped,
/// as the call runs again.
struct Deferrable<'a> {
    caller: &'a mut dyn CallerMemory,
    lock: Option<&'a CompileLock>,
}

impl CallerMemory for Deferrable<'_> {
    fn read(&mut self, address: u64, buffer: &mut [u8]) -> Result<(), CallerFault> {
        self.caller.read(address, buffer)
    }

    fn write(&mut self, address: u64, data: &[u8]) -> Result<(), CallerFault> {
        if self.lock.is_some_and(CompileLock::deferred) {
            return Ok(());
        }
        self.caller.write(address, data)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::metal::{
        device::fixture::{MockBackend, device_on},
        service::identity::DeviceEpoch,
    };

    const HOST: ServiceHost = ServiceHost {
        relax: || panic!("nothing waits"),
        relax_within: || panic!("nothing waits"),
        stack: 0x4000,
        run: |_, call| call(),
        report: |_| {},
        note: |_| {},
        ticks: || 7,
        served: |_| {},
        wake: || {},
        bundle: |_| {},
    };

    fn shared() -> SharedService<MockBackend, crate::metal::device::fixture::MockCompiler> {
        let service = Service::new(
            device_on(MockBackend::new()),
            DeviceEpoch::from_registered(NonZeroU64::MIN),
        );
        SharedService::new(service, HOST)
    }

    #[test]
    fn a_waiter_publishes_completion_while_a_call_holds_the_service() {
        let shared = shared();
        shared.publishable.store(true, Ordering::Release);
        shared.submitted.store(3, Ordering::Release);
        let timeline = shared.timeline.fetch_add(1, Ordering::AcqRel) + 1;
        let held = shared.try_turn().unwrap();
        // Another timeline's waiter, and one past what was submitted.
        shared.completed_through(timeline + 1, 2);
        assert!(!shared.signaled());
        shared.completed_through(timeline, 5);
        assert!(shared.signaled());
        assert_eq!(shared.reached.load(Ordering::Acquire), 3);
        assert_eq!(shared.early[3].0.load(Ordering::Acquire), 3);
        // An older point publishes nothing more.
        shared.completed_through(timeline, 2);
        assert!(!shared.signaled());
        assert!(shared.stale.load(Ordering::Acquire));
        drop(held);
    }

    #[test]
    fn only_unpublished_submissions_await_completion() {
        let shared = shared();
        // Idle: nothing submitted past what was published.
        assert!(!shared.awaits_completion());
        shared.submitted.store(3, Ordering::Release);
        assert!(shared.awaits_completion());
        shared.reached.store(3, Ordering::Release);
        assert!(!shared.awaits_completion());
        // A known loss needs no look.
        shared.submitted.store(4, Ordering::Release);
        shared.reached.store(POLL_UNAVAILABLE, Ordering::Release);
        assert!(!shared.awaits_completion());
    }

    #[test]
    fn without_coherent_memory_completion_waits_for_a_turn() {
        let shared = shared();
        shared.publishable.store(false, Ordering::Release);
        shared.submitted.store(3, Ordering::Release);
        let timeline = shared.timeline.fetch_add(1, Ordering::AcqRel) + 1;
        let held = shared.try_turn().unwrap();
        shared.completed_through(timeline, 3);
        assert!(!shared.signaled());
        assert_eq!(shared.reached.load(Ordering::Acquire), 0);
        drop(held);
    }

    #[test]
    fn a_deferred_run_writes_nothing_to_its_caller() {
        struct Writes(u32);
        impl CallerMemory for Writes {
            fn read(&mut self, _: u64, _: &mut [u8]) -> Result<(), CallerFault> {
                Ok(())
            }
            fn write(&mut self, _: u64, _: &[u8]) -> Result<(), CallerFault> {
                self.0 += 1;
                Ok(())
            }
        }
        let lock = CompileLock::new_for_tests();
        let mut writes = Writes(0);
        let mut deferrable = Deferrable {
            caller: &mut writes,
            lock: Some(&lock),
        };
        deferrable.write(0, &[1]).unwrap();
        lock.set_deferred_for_tests(true);
        deferrable.write(0, &[1]).unwrap();
        assert_eq!(writes.0, 1);
    }
}
