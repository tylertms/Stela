//! Parallel execution: every PE runs its own [`Machine`] on its own host core.
//!
//! The machines share guest RAM, which native code reaches directly, the
//! board's devices through [`SharedDevices`], and a [`Hub`] that carries what
//! one PE does to the others: a start, an event (SEV), and TLB and
//! instruction-cache maintenance broadcast to the Inner Shareable domain. A
//! PE that broadcasts maintenance waits until every other running PE has
//! applied it, as the DSB after it would on hardware. As on hardware, it
//! neither wakes nor waits for a PE that sleeps, which applies it before it
//! runs again; and a running PE leaves native code at its next chained
//! transfer to apply it ([`Hub::interrupt`]).
//!
//! A machine holds its own PE as on a board of one: the others are
//! [`Peer::Remote`](super::processors::Peer::Remote). The PEs of one physical
//! cluster still share its CTRR and cluster-scoped implementation-defined
//! registers, which the hub keeps per cluster.
//!
//! A device access that needs canonical memory (CPU PMGR writes and the
//! memory controller's drains), which only the home machine holds, goes
//! there from another PE's machine ([`Machine::forward_read`]): the home
//! machine stops every other running PE, as that access requires, serves
//! it for the PE that made it, and sends the answer back.
//!
//! Exclusives pass by value: a store-exclusive succeeds when guest RAM still
//! holds what the exclusive load read, compared and stored in one host
//! compare-and-exchange ([`super::Host::compare_exchange_ram`]). A write of
//! the same value by another PE between them goes unseen, as with LSE CAS.
use super::chain_control;
use super::{
    CanonicalDeviceMemory, CanonicalDeviceWrite, ConsumedPair, ConsumedScalar, ConsumedWrite,
    DeviceError, DeviceReadShape, Devices, Discard, FETCH_PAGE, Fetched, Host, Machine, Padded,
    RequirementKind, Reset, ScalarRead, Start, Step, Variant, clock::SharedClock, processors::Peer,
    step_timing,
};
use crate::machine::{
    ctrr,
    mmu::{self, tlbi},
    registers::Lines,
    shared::Shared,
    system::Broadcast,
};
use crate::platform::sync::Mutex;
use alloc::{
    boxed::Box,
    collections::{BTreeMap, VecDeque},
    sync::Arc,
    vec::Vec,
};
use core::sync::atomic::{AtomicBool, AtomicU8, AtomicU32, AtomicU64, AtomicUsize, Ordering};
use stela_compiler::aarch64::system::registers::Encoding;
use stela_compiler::x86_64::chain;
use stela_contracts::refuse;

mod shared_devices;
pub use shared_devices::{LockTimes, SharedDevices};
mod observation;
pub use observation::{MaterializedSource, SourceObservation, SourcePhase};
mod profile;
pub use profile::{BroadcastCounts, BroadcastKind, BroadcastProfile};

/// Brings the host core that runs a PE out of native code, or out of a wait,
/// to read its mail. Delivery is a host interrupt; a kick of a core already
/// in the engine costs it one return through its interrupt handler.
pub trait Kick: Send + Sync {
    fn kick(&self, pe: usize);
    /// The host's free-running count, which times how long the PEs wait
    /// for the devices' lock and hold it ([`SharedDevices::lock_times`]);
    /// 0 from a host that times nothing.
    fn ticks(&self) -> u64 {
        0
    }
    /// The host's counter as [`super::Host::counter`] reads it, from any
    /// core: what a wake for a PE that serves the host marks in the shared
    /// count ([`Hub::wake`]). `None` from a host that keeps no such count,
    /// whose wakes then mark nothing.
    fn count(&self) -> Option<u64> {
        None
    }
    /// Runs on the processor that just released the devices' lock, outside
    /// it: host work its device call left for that processor to do. Whether
    /// that work completed something the devices should look at now.
    fn released(&self) -> bool {
        false
    }
    /// Whether a device call left host work for [`Self::released`], without
    /// doing it: a PE in a guest exit then hands it to a sleeping PE
    /// ([`Hub::hand_off_service`]), which does it at its host service.
    fn release_due(&self) -> bool {
        false
    }
    /// Whether the work [`Self::release_due`] reports only gives a host
    /// device commands, with nothing to take from it: a start is one
    /// doorbell write, cheaper than the wake of a sleeping PE, so the
    /// releasing processor does it at once instead of handing it on.
    fn release_inline(&self) -> bool {
        false
    }
    /// Runs on the processor that just released the devices' lock: whether
    /// a host interrupt it takes now brought host work no one has been told
    /// of (a host disk's completion signal, which a core running guest code
    /// holds pending until it opens an interrupt window). The release then
    /// hands that work to a PE that sleeps waiting for an interrupt
    /// ([`Hub::hand_off_service`]); with none asleep it waits for the
    /// host's own service between slices. True once for each signal.
    fn signal_due(&self) -> bool {
        false
    }
    /// With no PE asleep to take the work a [`Self::signal_due`] signal
    /// brought, one bounded step of it on the releasing processor, outside
    /// the devices' lock and any lock the step could need. Whether it
    /// completed something the devices should look at now.
    fn signal_step(&self) -> bool {
        false
    }
}

/// The halt-poll after a wait that took `blocked` counts, from `poll`, at
/// most `most`: none on a core whose halt waits on a doorbell
/// ([`Host::halts_on_doorbell`]), where a wake is a store; elsewhere as
/// Linux's haltpoll governor fits it: halved once waits outlast the longest
/// poll (and none below a quarter of it), doubled to at least that quarter
/// when a short one had to halt.
pub(super) fn idle_poll_after(poll: u64, blocked: u64, most: u64, doorbell: bool) -> u64 {
    if doorbell {
        0
    } else if blocked > most {
        Some(poll / 2).filter(|poll| *poll >= most / 4).unwrap_or(0)
    } else {
        (poll * 2).clamp(most / 4, most)
    }
}

/// A PE that may use any context's translations ([`Inbox::translating`]).
const ANY_CONTEXT: u32 = u32::MAX;

/// [`Inbox::serving`] of a PE that does not serve the host.
const NOT_SERVING: u64 = u64::MAX;

/// [`Inbox::serving_accepts`] of a PE that takes both lines.
const ACCEPTS_ALL: u8 = 3;

/// IRQ and FIQ as [`Inbox::serving_accepts`] holds them.
fn pack_accepts(lines: Lines) -> u8 {
    u8::from(lines.irq) | u8::from(lines.fiq) << 1
}

/// Guest work that waits for a PE while its machine serves the host
/// ([`GuestWatch::pending`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Pending {
    /// Its device lines changed: an interrupt, a fast IPI among them.
    Lines,
    /// The host called for its core ([`Hub::call_host`]).
    HostCall,
    /// Another PE's device access waits for the home machine.
    Forward,
    /// The PEs are stopped or the run ends: it is to park.
    Stop,
    /// Its own timer, PMI or wait timeout is due.
    Timer,
}

impl Pending {
    pub fn name(self) -> &'static str {
        match self {
            Self::Lines => "lines",
            Self::HostCall => "call",
            Self::Forward => "forward",
            Self::Stop => "stop",
            Self::Timer => "timer",
        }
    }
}

/// What ends the host's work on one PE's core early ([`Machine::serve_host`]):
/// guest work for that PE. A PE out of its run loop answers no interrupt
/// and runs no timer of its own, so the host's work runs in short parts and
/// looks between them ([`Self::pending`]). A wake with guest work for it
/// also kicks its core, which ends a host wait for an interrupt, and holds
/// the shared count a stall limit past the wake should the PE stay out
/// (`clock`), as its own due timer does: no other PE then sees it answer
/// nothing for longer than XNU's cross-core timeouts allow.
#[derive(Clone)]
pub struct GuestWatch {
    hub: Option<Arc<Hub>>,
    pe: usize,
    /// The device line epoch its machine last read.
    epoch: u64,
    /// The host count at which its own timers next want it, not before it
    /// began to serve ([`Machine::serving_mark`]).
    wake_at: Option<u64>,
}

impl GuestWatch {
    /// A watch of PE `pe` on `hub` from line epoch `epoch`, whose own
    /// timers want it at host count `wake_at`.
    pub fn new(hub: Option<Arc<Hub>>, pe: usize, epoch: u64, wake_at: Option<u64>) -> Self {
        Self {
            hub,
            pe,
            epoch,
            wake_at,
        }
    }

    pub fn pe(&self) -> usize {
        self.pe
    }

    /// The guest work waiting for the PE at host count `host`, if any: a
    /// few loads, for a look between two parts of the host's work.
    pub fn pending(&self, host: u64) -> Option<Pending> {
        if let Some(pending) = self
            .hub
            .as_ref()
            .and_then(|hub| hub.guest_pending(self.pe, self.epoch))
        {
            return Some(pending);
        }
        self.wake_at
            .filter(|&at| host >= at)
            .map(|_| Pending::Timer)
    }

    /// [`Self::pending`], with the host counts it has waited at `host`
    /// when known: since its timer's deadline, or since the first wake that
    /// brought it.
    pub fn waited(&self, host: u64) -> Option<(Pending, Option<u64>)> {
        let pending = self.pending(host)?;
        let since = match pending {
            Pending::Timer => self.wake_at,
            _ => self.hub.as_ref().and_then(|hub| {
                let woken = hub.inboxes[self.pe].serving_woken.load(Ordering::Relaxed);
                (woken != 0).then_some(woken)
            }),
        };
        Some((pending, since.map(|since| host.saturating_sub(since))))
    }
}

/// The maintenance a PE keeps for the others ([`Machine::post`]) before it
/// sends it, DSB or not.
const OUTGOING: usize = 64;

/// [`Inbox::translating`] of a regime's ASID.
fn context_word(regime: mmu::Regime, asid: u16) -> u32 {
    (regime as u32) << 16 | u32::from(asid)
}

/// The lower half's addresses a PE may hold global translations of, only
/// ever widened. A TLBI by ASID removes those too, so it is not private
/// ([`tlbi::Private`]) there.
#[derive(Debug)]
pub struct GlobalSpan {
    first: AtomicU64,
    last: AtomicU64,
}

impl GlobalSpan {
    fn new() -> Self {
        Self {
            first: AtomicU64::new(u64::MAX),
            last: AtomicU64::new(0),
        }
    }

    /// Take in `first..=last` before using a translation there. When that
    /// widens the span, a broadcast may have passed the PE by while it
    /// walked, so it walks again: the caller's walk could predate the TLBI.
    pub fn widen(&self, first: u64, last: u64) -> bool {
        let lower = self.first.fetch_min(first, Ordering::SeqCst);
        let upper = self.last.fetch_max(last, Ordering::SeqCst);
        first < lower || last > upper
    }

    fn overlaps(&self, first: u64, last: u64) -> bool {
        self.first.load(Ordering::SeqCst) <= last && self.last.load(Ordering::SeqCst) >= first
    }
}

/// Maintenance one PE broadcasts to the others.
#[derive(Debug, Clone, Copy)]
pub(super) enum Maintenance {
    /// A TLBI's entries, which each PE reaches from its own context.
    Tlb(tlbi::Scope),
    /// Every TLB entry of every context: what a mailbox keeps in place of
    /// more than [`MAIL_TLB_SCOPES`] TLBIs none of which merge, which
    /// invalidates more than they did, as Arm allows.
    TlbAll,
    /// IC IVAU of the page holding this physical address, or IC IALLUIS.
    Code(Option<u64>),
    /// A CPU power change, which pauses or resumes the PE it reaches and
    /// which every machine's record of the board's CPU power takes.
    Power(super::ProcessorChange),
}

/// The code pages a mailbox keeps before it keeps one invalidation of
/// every page instead: a PE that sleeps for minutes still reads its mail at
/// once and from bounded memory. One PE's full outgoing list still arrives
/// whole.
const MAIL_SCOPES: usize = OUTGOING;

/// The TLB scopes a mailbox keeps ([`Mail::keep`]): past them, each new one
/// merges into the kept scope that makes the narrowest range with it, and
/// only one that merges with none of the last [`MAIL_NEAR`] makes the
/// mailbox keep an invalidation of everything. About 15 KiB a PE.
const MAIL_TLB_SCOPES: usize = 4 * OUTGOING;

/// The most recent kept TLB scopes a new one may merge into: a run of
/// TLBIs by page (XNU's and SPTM's loops of `TLBI VALE1IS`) arrives in
/// order, so each merges into the one before.
const MAIL_NEAR: usize = 16;

/// How far apart a new TLB scope's span may be from a kept one's for the
/// two to merge before the mailbox is full: three 16 KiB pages between
/// them at most, which the merged range removes besides.
const MAIL_MERGE_GAP: u64 = 0x1_0000;

/// What other PEs left for one PE.
#[derive(Debug, Default)]
pub(super) struct Mail {
    /// A device started the PE at this address.
    pub start: Option<u64>,
    /// SEV.
    pub event: bool,
    pub maintenance: Vec<Maintenance>,
    /// The serial of the last maintenance in `maintenance`.
    pub serial: u64,
    /// TLB scopes that merged into one kept before them.
    pub merged: u64,
}

impl Mail {
    /// Keep `maintenance` for the PE's next read. An instruction cache
    /// invalidation it has yet to read already covers the same page again,
    /// and so does a whole invalidation of either kind; past
    /// [`MAIL_SCOPES`] code pages one whole invalidation replaces them. A
    /// TLB scope next to one of the last kept merges with it
    /// ([`tlbi::Scope::merged`]) where that keeps the ASID either named;
    /// past [`MAIL_TLB_SCOPES`], it merges into
    /// the one of the last [`MAIL_NEAR`] that makes the narrowest range,
    /// and one that merges with none of them makes a whole invalidation.
    /// Every kept scope removes at least what the TLBIs it stands for
    /// remove, and TLB invalidations commute: the PE applies all its mail
    /// before it translates again.
    fn keep(&mut self, maintenance: Maintenance) {
        let held = |kind: fn(&Maintenance) -> bool, list: &[Maintenance]| {
            list.iter().filter(|posted| kind(posted)).count()
        };
        match maintenance {
            Maintenance::Code(page) => {
                let covered = self.maintenance.iter().any(|posted| {
                    matches!(posted, Maintenance::Code(held) if held.is_none() || *held == page)
                });
                if covered {
                    return;
                }
                let pages = |posted: &Maintenance| matches!(posted, Maintenance::Code(Some(_)));
                if page.is_some() && held(pages, &self.maintenance) >= MAIL_SCOPES {
                    self.maintenance.retain(|posted| !pages(posted));
                    self.maintenance.push(Maintenance::Code(None));
                    return;
                }
            }
            Maintenance::Tlb(scope) => {
                if self
                    .maintenance
                    .iter()
                    .any(|posted| matches!(posted, Maintenance::TlbAll))
                {
                    return;
                }
                let scopes = |posted: &Maintenance| matches!(posted, Maintenance::Tlb(_));
                let full = held(scopes, &self.maintenance) >= MAIL_TLB_SCOPES;
                let gap = if full { u64::MAX } else { MAIL_MERGE_GAP };
                // The narrowest merge with one of the last kept scopes.
                let nearest = self
                    .maintenance
                    .iter()
                    .enumerate()
                    .rev()
                    .filter_map(|(at, posted)| match posted {
                        Maintenance::Tlb(held) => Some((at, *held)),
                        _ => None,
                    })
                    .take(MAIL_NEAR)
                    .filter_map(|(at, held)| Some((at, held, held.merged(scope, gap)?)))
                    // Until the mailbox is full a merge keeps an ASID either
                    // scope named: one of every ASID would drop those VAs in
                    // every process's lookup contexts, which share the
                    // shared cache's addresses.
                    .filter(|(_, held, merged)| {
                        full || merged.named_asid().is_some()
                            || held.named_asid().is_none() && scope.named_asid().is_none()
                    })
                    .map(|(at, _, merged)| (at, merged))
                    .min_by_key(|(_, merged)| merged.span().map(|(first, last)| last - first));
                if let Some((at, merged)) = nearest {
                    self.maintenance[at] = Maintenance::Tlb(merged);
                    self.merged += 1;
                    return;
                }
                if full {
                    self.maintenance.retain(|posted| !scopes(posted));
                    self.maintenance.push(Maintenance::TlbAll);
                    return;
                }
            }
            Maintenance::TlbAll | Maintenance::Power(_) => {}
        }
        self.maintenance.push(maintenance);
    }
}

/// On cache lines of its own: its owner reads `pending` and `lines` at
/// every step, while other PEs write the inboxes beside it.
#[repr(align(64))]
struct Inbox {
    /// Mail is waiting.
    pending: AtomicBool,
    /// Its machine runs and reads its mail; a broadcast waits only for these.
    live: AtomicBool,
    /// Its host core halts until an interrupt: only then does mail kick it.
    /// A running core reads its mail at its next return to the engine,
    /// which [`Hub::interrupt`] brings forward to its next chained transfer.
    sleeping: AtomicBool,
    /// While it sleeps, it waits for an interrupt (or is powered down), so a
    /// wake for the host's service is one it takes ([`Hub::request_service`]):
    /// one waiting in WFE may hold a ticket another PE hands it, and leaves
    /// the service to others.
    serves: AtomicBool,
    /// The host needs its core ([`Hub::call_host`]).
    host_call: AtomicBool,
    /// Its machine left its run loop, for the host's service or a stop, or
    /// compiles: it runs no guest code until it reads its mail again, so a
    /// broadcast need not wait for it, as for one that sleeps.
    outside: AtomicBool,
    /// While its machine does the host's work outside its run loop
    /// ([`Machine::serve_host`]): the device line epoch it last read, else
    /// [`NOT_SERVING`]. A wake that brings it guest work then kicks it and
    /// marks the shared count ([`Hub::wake`]), and the work ends at its next
    /// part ([`GuestWatch`]).
    serving: AtomicU64,
    /// The host count of the first wake with guest work since it began
    /// serving, 0 for none: how long that work has waited.
    serving_woken: AtomicU64,
    /// The device lines that bring it back to guest work while it serves
    /// ([`Machine::accepted_lines`], [`pack_accepts`]): a line change is
    /// guest work for it only while it accepts some line.
    serving_accepts: AtomicU8,
    /// Odd while its machine acts on a fetched block: runs it natively
    /// (with the translations its host and chained blocks keep) and
    /// completes its exit, or takes the fetch's exception or checked
    /// instruction. Even between, at its run loop's top, scheduling and
    /// fetching, where it uses no translation it keeps beyond the fetch,
    /// and enters again only through [`Hub::enter_active`], which sends it
    /// back to read mail posted meanwhile. Only the PE writes it; each
    /// change adds one, so a broadcast also sees an exit and entry it did
    /// not catch.
    active: AtomicU64,
    /// Under step timing, the exit its machine completes while
    /// [`Self::active`] is odd ([`step_timing::exit_blocker`]), or 0 in
    /// native code: what a broadcast that waits for it waits for
    /// ([`step_timing::BarrierWaits`]).
    doing: AtomicU64,
    /// Stable native chaining allocation, kept even between PE owners.
    chain: Arc<chain_control::Storage>,
    /// Unique owner epoch; zero is free and u64::MAX is detaching.
    owner: AtomicU64,
    /// It runs no guest code until the world it was stopped for restarts,
    /// or while it waits in the engine for other PEs.
    parked: AtomicBool,
    /// The run is over and this PE's machine has freed its blocks.
    discarded: AtomicBool,
    /// The serial of the last maintenance posted and of the last applied.
    posted: AtomicU64,
    applied: AtomicU64,
    /// The host tick of the first post its machine has not read yet, 0
    /// for none ([`super::Statistics::mail_waits`]).
    posted_at: AtomicU64,
    /// Device lines changed since the machine last read them.
    lines: AtomicU64,
    /// The regime and ASID ([`context_word`]) of the only non-global
    /// translations it may use before it reads its mail.
    translating: AtomicU32,
    lower_globals: Arc<GlobalSpan>,
    /// Its machine's counts, as it last published them ([`Hub::publish`]).
    counts: Mutex<Counts>,
    /// Its System exits since the home machine last took them
    /// ([`Hub::add_system_exits`]).
    system_exits: Mutex<step_timing::SystemExits>,
    source: observation::Slot,
    broadcasts: profile::Slot,
    mail: Mutex<Mail>,
}

impl Inbox {
    /// Whether this PE may use a translation a TLBI private to one context
    /// removes.
    fn may_use(&self, private: tlbi::Private) -> bool {
        let translating = self.translating.load(Ordering::SeqCst);
        translating == ANY_CONTEXT
            || translating == context_word(private.regime, private.asid)
            || private
                .addresses
                .is_some_and(|(first, last)| self.lower_globals.overlaps(first, last))
    }
}

/// What one PE's machine has done, for reports on how the PEs run together:
/// work, compilation and the time spent idle or waiting on other PEs.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counts {
    pub blocks: u64,
    pub retired: u64,
    /// Blocks it compiled, and blocks another PE compiled that it installed.
    pub compiled: u64,
    pub shared: u64,
    /// Self-loops it compiled, their entries and the instructions their
    /// iterations retired past each entry's first ([`super::Statistics::self_loops`],
    /// `ChainControl::loop_entries`, `loop_iterated`).
    pub loops: [u64; 3],
    /// Its installs at EL0 and above by where the block came from
    /// ([`super::Statistics::store_sources`]).
    pub store_sources: [[u64; 5]; 2],
    /// Blocks it holds, the slots its block table has, the pages its
    /// index finds blocks in, and the parts of its code arena it released
    /// for heap pressure.
    pub live_blocks: u64,
    pub block_slots: u64,
    pub indexed_pages: u64,
    pub code_releases: u64,
    pub compile_ticks: u64,
    /// Counter ticks it spent reading and hashing fetch pages, finding
    /// shared blocks and copying them, and installing blocks
    /// ([`super::Statistics::page_ticks`]).
    pub page_ticks: u64,
    pub shared_ticks: u64,
    pub install_ticks: u64,
    /// Counter ticks its fetches spent walking translations and refilling
    /// pages ([`super::Statistics::walk_ticks`]).
    pub walk_ticks: u64,
    pub refill_ticks: u64,
    /// Device transfers whose deadline came and device line queries, with
    /// their profile clock ticks ([`super::Statistics::due_transfers`]).
    pub due_transfers: u64,
    pub transfer_ticks: u64,
    pub line_queries: u64,
    pub line_ticks: u64,
    /// Counter ticks its host core halted or spun on a monitored granule
    /// ([`super::Statistics::sleep_ticks`], `spin_ticks`) and spent waiting
    /// on other PEs ([`super::Statistics::sync_ticks`]), and its waits.
    pub sleep_ticks: u64,
    pub spin_ticks: u64,
    /// Of `sleep_ticks`, those in the host halt ([`super::Statistics::halt_ticks`]).
    pub halt_ticks: u64,
    /// Of `waits`, WFEs completed in their exit ([`super::Statistics::short_waits`]).
    pub short_waits: u64,
    pub sync_ticks: u64,
    /// The TLB mail it applied and how long it waited
    /// ([`super::Statistics::mail_waits`]).
    pub mail_waits: [u64; super::reports::MAIL_WAITS],
    pub waits: u64,
    pub broadcasts: u64,
    pub forwards: u64,
    pub exceptions: u64,
    pub device_accesses: u64,
    /// Its fetches by where they found their block, and its ITLB's fills
    /// and new keys ([`super::Statistics::fetches`], `itlb`).
    pub fetches: [u64; 5],
    pub itlb: [u64; 2],
    /// Its fetches by path, its ITLB fills by why the page was missing,
    /// its ITLB's drops and its forgotten contexts
    /// ([`super::Statistics::fetch_paths`], `itlb_misses`, `itlb_drops`,
    /// `lookup_forgets`).
    pub fetch_paths: [u64; 6],
    pub itlb_misses: [u64; 4],
    pub itlb_drops: [u64; 5],
    pub lookup_forgets: [u64; 4],
    /// Its ITLB's fills, drops and forgotten contexts by where they came
    /// from ([`super::Statistics::itlb_sources`]).
    /// GENTER completed natively and by the engine, then GEXIT's, SVC's
    /// and ERET's ([`super::Statistics::native_transitions`]).
    pub transitions: [u64; 8],
    /// Why SVC, ERET, GENTER and GEXIT took the engine path, in kind order
    /// ([`super::Statistics::transition_misses`]).
    pub transition_misses: [[u64; super::transitions::miss::REASONS]; 4],
    /// Why its runs ended at branch-kind exits, in kind order
    /// ([`super::Statistics::branch_misses`]).
    pub branch_misses: [[u64; super::reports::BRANCH_REASONS]; super::reports::BRANCH_KINDS],
    pub itlb_sources: [u64; super::ItlbSource::COUNT],
    /// Its stamped transfers ([`chain::StampCount`]'s order), the exits it
    /// gave a stamped tail, how many exits had one when it last looked, then
    /// the PAC branch exits it gave a predicted tail and its probe
    /// transfers ([`super::Statistics::lookup_hits`]).
    pub stamped: [u64; chain::STAMP_COUNTS + 4],
    /// PAC results its service computed since boot, each a memo miss.
    pub pac_fills: u64,
    /// TSC ticks in each engine phase ([`step_timing::Phase`]), while
    /// step timing is enabled.
    pub phases: [u64; step_timing::Phase::COUNT],
    /// Exits handled and the TSC ticks handling them, by
    /// [`step_timing::StepTiming::exits`]' slots, while step timing is
    /// enabled.
    pub exits: [(u64, u64); step_timing::EXIT_SLOTS],
    /// Its busy time in guest terms and its waits, while step timing is
    /// enabled ([`step_timing::Levels`]).
    pub levels: step_timing::Levels,
    /// Its installs and evictions ([`super::InstallCauses`]).
    pub installs: super::InstallCauses,
}

/// The cluster state its powered PEs share.
#[derive(Clone)]
pub(super) struct Cluster {
    pub ctrr: Shared<ctrr::Cluster>,
    pub impdef: Shared<BTreeMap<u16, u64>>,
}

/// The mailboxes of every PE, by PE number.
pub struct Hub {
    inboxes: Box<[Inbox]>,
    kick: Box<dyn Kick>,
    /// Cluster state by physical cluster (MPIDR_EL1 Aff1).
    clusters: Mutex<BTreeMap<u64, Cluster>>,
    /// The count every PE reads, from the home machine's as the board went
    /// parallel on.
    clock: Arc<SharedClock>,
    /// The home machine's PE. Every PE reads it and `stopped` at every
    /// step, each on a line of its own: a forwarded device access writes
    /// `forwarding`, `requested` and `request`.
    home: Padded<AtomicUsize>,
    home_owner: AtomicU64,
    next_owner: AtomicU64,
    /// One PE's forwarded device access at a time, and that one.
    forwarding: AtomicBool,
    requested: AtomicBool,
    request: Mutex<Option<Request>>,
    /// Every running PE but the home one and the requester parks.
    stopped: Padded<AtomicBool>,
    /// The run is over: a parked PE frees its blocks and stays parked.
    finishing: AtomicBool,
    /// A device left bytes for the host to feed or drain
    /// ([`Hub::request_service`]).
    service: AtomicBool,
    source_observations: AtomicBool,
    /// The fetch pages any PE may have compiled from ([`Hub::compiling`]).
    code_pages: CodePages,
    /// Which PEs hold each block installed, for `--profile`'s counts.
    pub(super) installs: super::install_census::Live,
    /// The compiled code the PEs share, and its grace: a PE is inside
    /// while neither `outside` nor `parked` ([`Self::section`]).
    pub(super) board: super::board::Board,
    /// The home machine's code, when its host gives the engine a code
    /// window: every PE whose host gives the same window joins it.
    code: Mutex<Option<Arc<super::board::Code>>>,
    /// Every PE's hot-block samples since the home machine last took them
    /// ([`Hub::add_hot_samples`]).
    hot: Mutex<step_timing::HotBlocks>,
}

/// Which physical fetch pages any PE may have compiled from, as a filter:
/// bit `n` covers every page of a frame number congruent to `n`. A PE sets
/// a page's bit before it reads the page to compile. So a PE that finds it
/// clear after the guest's writes knows no block holds what they replaced,
/// and an IC IVAU of data, of pages loaded for the first time, stops there.
struct CodePages(Box<[AtomicU64]>);

impl CodePages {
    /// The bits, a power of two: 16 GiB of 16 KiB pages cover it once.
    const BITS: usize = 1 << 20;

    fn new() -> Self {
        Self((0..Self::BITS / 64).map(|_| AtomicU64::new(0)).collect())
    }

    /// The word and bit of the page holding `physical`.
    fn at(&self, physical: u64) -> (&AtomicU64, u64) {
        let frame = (physical / FETCH_PAGE) as usize & (Self::BITS - 1);
        (&self.0[frame / 64], 1 << (frame % 64))
    }
}

/// A device access that only the home machine's canonical memory serves.
#[derive(Debug, Clone, Copy)]
pub(super) enum Forwarded {
    Read {
        physical: u64,
        shape: DeviceReadShape,
    },
    Write {
        physical: u64,
        bytes: u8,
        value: u64,
    },
}

/// The home machine's answer to a [`Forwarded`] access.
#[derive(Debug, Clone)]
pub(super) enum Answer {
    Read(Result<ConsumedPair, DeviceError>),
    Write(Result<(), DeviceError>),
}

struct Request {
    pe: usize,
    access: Forwarded,
    answer: Option<Answer>,
}

impl Hub {
    pub fn new(pes: usize, kick: impl Kick + 'static) -> Arc<Self> {
        Arc::new(Self {
            inboxes: (0..pes)
                .map(|_| Inbox {
                    pending: AtomicBool::new(false),
                    live: AtomicBool::new(false),
                    sleeping: AtomicBool::new(false),
                    serves: AtomicBool::new(false),
                    host_call: AtomicBool::new(false),
                    translating: AtomicU32::new(ANY_CONTEXT),
                    lower_globals: Arc::new(GlobalSpan::new()),
                    outside: AtomicBool::new(false),
                    serving: AtomicU64::new(NOT_SERVING),
                    serving_woken: AtomicU64::new(0),
                    serving_accepts: AtomicU8::new(ACCEPTS_ALL),
                    active: AtomicU64::new(0),
                    doing: AtomicU64::new(0),
                    chain: chain_control::Storage::new(),
                    owner: AtomicU64::new(0),
                    parked: AtomicBool::new(false),
                    discarded: AtomicBool::new(false),
                    posted: AtomicU64::new(0),
                    applied: AtomicU64::new(0),
                    posted_at: AtomicU64::new(0),
                    lines: AtomicU64::new(0),
                    counts: Mutex::new(Counts::default()),
                    system_exits: Mutex::new(step_timing::SystemExits::default()),
                    source: observation::Slot::default(),
                    broadcasts: profile::Slot::default(),
                    mail: Mutex::new(Mail::default()),
                })
                .collect(),
            kick: Box::new(kick),
            clusters: Mutex::new(BTreeMap::new()),
            clock: Arc::new(SharedClock::new(pes)),
            home: Padded(AtomicUsize::new(0)),
            home_owner: AtomicU64::new(0),
            next_owner: AtomicU64::new(1),
            forwarding: AtomicBool::new(false),
            requested: AtomicBool::new(false),
            request: Mutex::new(None),
            stopped: Padded(AtomicBool::new(false)),
            finishing: AtomicBool::new(false),
            service: AtomicBool::new(false),
            code_pages: CodePages::new(),
            source_observations: AtomicBool::new(false),
            installs: super::install_census::Live::default(),
            hot: Mutex::new(step_timing::HotBlocks::default()),
            board: super::board::Board::new(pes),
            code: Mutex::new(None),
        })
    }

    /// The host's count for timing lock waits ([`Kick::ticks`]).
    pub(super) fn ticks(&self) -> u64 {
        self.kick.ticks()
    }

    /// What the code the board's PEs share came to: `None` while each
    /// keeps its own.
    pub fn code_report(&self) -> Option<super::CodeReport> {
        let code = self
            .code
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .clone();
        code.map(|code| code.report())
    }

    pub fn pes(&self) -> usize {
        self.inboxes.len()
    }

    /// The count every PE reads, at the host's counter `counter`.
    pub fn clock(&self, counter: impl FnMut() -> u64) -> u64 {
        self.clock.count(counter)
    }

    /// Host counts the count waited for PEs stopped in the kernel since the
    /// last call ([`SharedClock::take_stalled`]).
    pub fn clock_stalled(&self) -> u64 {
        self.clock.take_stalled()
    }

    /// Host counts the count now lags the host's by, and the counts the
    /// slew repaid since the last call ([`SharedClock::lag`],
    /// [`SharedClock::take_repaid`]).
    pub fn clock_lag(&self) -> (u64, u64) {
        (self.clock.lag(), self.clock.take_repaid())
    }

    /// Each PE's longest gap with guest work and no instruction retired
    /// since the last call, in counts, and the phase that took most of it
    /// ([`SharedClock::take_stalls`]): a step loop phase's name, `host` for
    /// host work outside the step loop, `?` without step timing.
    pub fn clock_stalls(&self) -> Vec<(u64, &'static str)> {
        self.clock
            .take_stalls()
            .into_iter()
            .map(|(ticks, phase)| {
                let name = match step_timing::Phase::ALL.get(usize::from(phase)) {
                    Some(phase) => phase.name(),
                    None if phase == super::clock::STALL_HOST => "host",
                    None => "?",
                };
                (ticks, name)
            })
            .collect()
    }

    /// Enable optional owner observations. This changes no guest, mailbox,
    /// retirement or worker-admission state; ordinary boots leave it off.
    pub fn set_source_observations(&self, enabled: bool) {
        self.source_observations.store(enabled, Ordering::Release);
    }

    pub fn source_observations_enabled(&self) -> bool {
        self.source_observations.load(Ordering::Acquire)
    }

    /// Last materialized state copied by this PE's attached Machine. None
    /// means disabled, unpublished, out of range, or no stable copy for
    /// the current owner.
    /// A returned copy can already be stale and is no quiescence receipt.
    pub fn source_observation(&self, pe: usize) -> Option<SourceObservation> {
        if !self.source_observations_enabled() {
            return None;
        }
        let inbox = self.inboxes.get(pe)?;
        let owner = inbox.owner.load(Ordering::Acquire);
        if owner == 0 || owner == u64::MAX {
            return None;
        }
        let observed = inbox.source.read(owner)?;
        (inbox.owner.load(Ordering::Acquire) == owner).then_some(observed)
    }

    /// Bounded kept broadcast counters, enabled with owner observations.
    /// These may already be stale and never authorize native reclamation.
    pub fn broadcast_profile(&self, pe: usize) -> Option<BroadcastProfile> {
        if !self.source_observations_enabled() {
            return None;
        }
        self.inboxes.get(pe)?.broadcasts.read()
    }

    fn post(&self, pe: usize, write: impl FnOnce(&mut Mail)) {
        self.leave(pe, write);
        self.interrupt(pe);
    }

    /// Leave mail for PE `pe` to find at its next read.
    fn leave(&self, pe: usize, write: impl FnOnce(&mut Mail)) {
        let inbox = &self.inboxes[pe];
        write(&mut inbox.mail.lock().unwrap_or_else(|e| e.into_inner()));
        inbox.pending.store(true, Ordering::SeqCst);
    }

    /// Publish interruption after its source; every next chain gate compares
    /// it with the owner's pre-drain snapshot, independently of budget resets.
    /// An already-passed gate can perform at most one further transfer.
    fn interrupt(&self, pe: usize) {
        self.inboxes[pe].chain.notify();
    }

    /// Kick PE `pe`'s core if it halts, or if it serves the host and this
    /// wake brings it guest work ([`Self::wake_serving`]). Ordered after the
    /// store that tells it why, against its own store of `sleeping` before
    /// it looks ([`Self::sleep`]), or of `serving` ([`Self::set_serving`]):
    /// one of them sees the other's. A PE in its run loop costs one more
    /// load.
    fn wake(&self, pe: usize) {
        let inbox = &self.inboxes[pe];
        if inbox.sleeping.load(Ordering::SeqCst) {
            self.kick.kick(pe);
            return;
        }
        let epoch = inbox.serving.load(Ordering::SeqCst);
        if epoch != NOT_SERVING {
            self.wake_serving(pe, epoch, true);
        }
    }

    /// PE `pe` serves the host from line epoch `epoch`: with guest work for
    /// it, its first such wake kicks its core (`kick`), whose host waits for
    /// an interrupt then end. Its work looks for the guest's at its next
    /// part ([`GuestWatch`]). A line it accepts also marks its slot of the
    /// shared count with the count now, so that the count waits for it a
    /// stall limit later (`clock`): only an interrupt it would take, or that
    /// ends its wait, is guest work another core's XNU can time. A host
    /// call is not, and a forwarded access or a stop waits on PEs whose own
    /// slots hold the count while they wait in their exits.
    #[cold]
    fn wake_serving(&self, pe: usize, epoch: u64, kick: bool) {
        let Some(pending) = self.guest_pending(pe, epoch) else {
            return;
        };
        let host = self.kick.count();
        let first = self.inboxes[pe]
            .serving_woken
            .compare_exchange(
                0,
                host.unwrap_or(0).max(1),
                Ordering::Relaxed,
                Ordering::Relaxed,
            )
            .is_ok();
        if first && kick {
            self.kick.kick(pe);
        }
        // Lowers the slot to the first such wake's count; a later one, or
        // one after a host call's, leaves it or lowers it once.
        if pending == Pending::Lines
            && let Some(host) = host
        {
            let count = self.clock.count(|| host);
            self.clock.pending(pe, count);
        }
    }

    /// The device lines that bring PE `pe` back to guest work while its
    /// machine serves the host ([`Machine::accepted_lines`]), set before
    /// [`Self::set_serving`].
    pub(super) fn set_accepts(&self, pe: usize, lines: Lines) {
        self.inboxes[pe]
            .serving_accepts
            .store(pack_accepts(lines), Ordering::Relaxed);
    }

    /// PE `pe`'s machine serves the host from line epoch `epoch`, or no
    /// longer (`None`); the state before. Guest work already there when it
    /// begins marks the count as a wake would, without a kick.
    pub(super) fn set_serving(&self, pe: usize, epoch: Option<u64>) -> Option<u64> {
        let inbox = &self.inboxes[pe];
        let word = epoch.unwrap_or(NOT_SERVING);
        if epoch.is_some() {
            inbox.serving_woken.store(0, Ordering::Relaxed);
        }
        let was = inbox.serving.swap(word, Ordering::SeqCst);
        if let Some(epoch) = epoch {
            self.wake_serving(pe, epoch, false);
        }
        (was != NOT_SERVING).then_some(was)
    }

    /// The guest work waiting for PE `pe`, whose machine last read line
    /// epoch `epoch`, other than its own timers: a line change while it
    /// accepts a line ([`Self::set_accepts`]), a host call, a forwarded
    /// access for the home machine, or a stop.
    pub(super) fn guest_pending(&self, pe: usize, epoch: u64) -> Option<Pending> {
        let inbox = &self.inboxes[pe];
        if inbox.lines.load(Ordering::SeqCst) != epoch
            && inbox.serving_accepts.load(Ordering::Relaxed) != 0
        {
            Some(Pending::Lines)
        } else if inbox.host_call.load(Ordering::SeqCst) {
            Some(Pending::HostCall)
        } else if self.home.load(Ordering::Relaxed) == pe && self.requested.load(Ordering::SeqCst) {
            Some(Pending::Forward)
        } else if self.home.load(Ordering::Relaxed) != pe && self.stopped.load(Ordering::SeqCst) {
            Some(Pending::Stop)
        } else {
            None
        }
    }

    /// Whether mail, a host call, the host's service or a line change PE
    /// `pe` has not seen since `epoch` wants it awake.
    pub(super) fn woken(&self, pe: usize, epoch: u64) -> bool {
        let inbox = &self.inboxes[pe];
        inbox.pending.load(Ordering::SeqCst)
            || inbox.host_call.load(Ordering::SeqCst)
            || inbox.lines.load(Ordering::SeqCst) != epoch
            || self.service.load(Ordering::SeqCst)
    }

    /// Halt PE `pe`'s core with `halt` unless mail or a line change it has
    /// not seen since `epoch` is already there; `serves` when it waits for an
    /// interrupt, so takes the host's service a wake brings ([`Inbox::serves`]).
    fn sleep(&self, pe: usize, epoch: u64, serves: bool, halt: impl FnOnce()) {
        let inbox = &self.inboxes[pe];
        inbox.serves.store(serves, Ordering::SeqCst);
        inbox.sleeping.store(true, Ordering::SeqCst);
        if !self.woken(pe, epoch) {
            halt();
        }
        // Ordered before its look at its mail, against a broadcast's look
        // at `sleeping` ([`Self::applied`]).
        inbox.sleeping.store(false, Ordering::SeqCst);
        inbox.serves.store(false, Ordering::SeqCst);
    }

    /// A PE other than `caller` that sleeps waiting for an interrupt, which
    /// takes the host's service once woken ([`Inbox::serves`]).
    fn sleeping_server(&self, caller: usize) -> Option<usize> {
        (0..self.pes()).find(|&pe| {
            let inbox = &self.inboxes[pe];
            pe != caller
                && inbox.sleeping.load(Ordering::SeqCst)
                && inbox.serves.load(Ordering::SeqCst)
        })
    }

    /// Hand the host's service to a PE that sleeps waiting for an interrupt,
    /// so that `caller` need not do the host work its device call left: its
    /// idle core does it while `caller`'s guest runs on. Whether one was
    /// woken for it; with none, the caller does the work itself. The woken
    /// PE takes the service at its next wait pass ([`Self::take_service`]);
    /// should it find guest work first, the next PE to wait takes it.
    pub(super) fn hand_off_service(&self, caller: usize) -> bool {
        let Some(pe) = self.sleeping_server(caller) else {
            return false;
        };
        self.service.store(true, Ordering::SeqCst);
        self.kick.kick(pe);
        true
    }

    /// A device left bytes for the host to feed or drain: the host's service
    /// is due. The first PE that sleeps waiting for an interrupt wakes to
    /// serve it ([`Inbox::serves`]), which its idle core can without holding
    /// up any guest code; with none asleep, the next PE to wait takes it. Ordered against [`Self::sleep`]'s store of
    /// `sleeping` as [`Self::wake`] is.
    fn request_service(&self) {
        if self.service.swap(true, Ordering::SeqCst) {
            return;
        }
        if let Some(pe) = self.sleeping_server(usize::MAX) {
            self.kick.kick(pe);
        }
    }

    /// Whether the host's service is due with no PE asleep to wake for it:
    /// every PE runs, and only one that waits would take it.
    pub fn service_unclaimed(&self) -> bool {
        self.service.load(Ordering::SeqCst) && self.sleeping_server(usize::MAX).is_none()
    }

    /// The host has work only PE `pe`'s core can do: its run ends at the next
    /// chained transfer, or at once if it sleeps. Safe from an interrupt and
    /// from any core; ordered against [`Self::sleep`] as [`Self::wake`] is.
    pub fn call_host(&self, pe: usize) {
        self.inboxes[pe].host_call.store(true, Ordering::SeqCst);
        self.interrupt(pe);
        self.wake(pe);
    }

    /// Whether the host needs PE `pe`'s core; once for each call.
    pub(super) fn take_host_call(&self, pe: usize) -> bool {
        let call = &self.inboxes[pe].host_call;
        call.load(Ordering::Relaxed) && call.swap(false, Ordering::AcqRel)
    }

    /// Whether the host's service is due; once for each request.
    fn take_service(&self) -> bool {
        self.service.load(Ordering::Relaxed) && self.service.swap(false, Ordering::AcqRel)
    }

    /// Wait, halting with `halt`, until a device starts PE `pe`, which has
    /// no machine yet; the start's address.
    pub fn first_start(&self, pe: usize, mut halt: impl FnMut()) -> u64 {
        loop {
            if let Some(Mail {
                start: Some(address),
                ..
            }) = self.take(pe)
            {
                return address;
            }
            let epoch = self.lines_epoch(pe);
            self.sleep(pe, epoch, false, &mut halt);
        }
    }

    /// Mark PE `pe`'s machine as reading its mail, or no longer.
    pub fn set_live(&self, pe: usize, live: bool) {
        self.inboxes[pe].live.store(live, Ordering::Release);
    }

    /// A device started PE `pe` at `address`.
    pub(super) fn start(&self, pe: usize, address: u64) {
        self.post(pe, |mail| mail.start = Some(address));
        self.wake(pe);
    }

    /// SEV by PE `from`: every other running PE's event register.
    pub(super) fn event(&self, from: usize) {
        for pe in self.others(from) {
            self.post(pe, |mail| mail.event = true);
            self.wake(pe);
        }
    }

    /// Post `maintenance` to every other running PE, without waking those
    /// that sleep; the serials to wait for. An instruction cache
    /// invalidation the PE has yet to read already covers the same page
    /// again, as IC IVAU of each line of a page posts it.
    ///
    /// A PE that translates another context than these TLBIs' is neither
    /// interrupted nor waited for: nothing it can use is stale, and the
    /// mail left first is read before it uses that context, which it
    /// publishes first ([`Machine::publish_translating`]).
    #[cfg(test)]
    pub(super) fn broadcast(&self, from: usize, maintenance: &[Maintenance]) -> Vec<(usize, u64)> {
        let mut posted = Vec::new();
        self.broadcast_each(from, maintenance, |pe, serial| posted.push((pe, serial)));
        posted
    }

    /// [`Self::broadcast`], giving each PE to wait for and its serial to
    /// `reached` instead of collecting them; how many there were.
    pub(super) fn broadcast_each(
        &self,
        from: usize,
        maintenance: &[Maintenance],
        mut reached: impl FnMut(usize, u64),
    ) -> usize {
        let mut posted = 0;
        for pe in self.others(from) {
            let inbox = &self.inboxes[pe];
            let mut serial = 0;
            self.leave(pe, |mail| {
                mail.serial += 1;
                serial = mail.serial;
                for &maintenance in maintenance {
                    mail.keep(maintenance);
                }
            });
            inbox.posted.store(serial, Ordering::Release);
            let reaches = maintenance.iter().any(|maintenance| match maintenance {
                Maintenance::Tlb(scope) => {
                    scope.private().is_none_or(|private| inbox.may_use(private))
                }
                _ => true,
            });
            if reaches {
                let now = self.ticks().max(1);
                let _ =
                    inbox
                        .posted_at
                        .compare_exchange(0, now, Ordering::Relaxed, Ordering::Relaxed);
                self.interrupt(pe);
                reached(pe, serial);
                posted += 1;
            }
        }
        posted
    }

    /// A PE is about to read the page holding `physical` to compile it.
    pub(super) fn compiling(&self, physical: u64) {
        let (word, bit) = self.code_pages.at(physical);
        if word.load(Ordering::SeqCst) & bit == 0 {
            word.fetch_or(bit, Ordering::SeqCst);
        }
    }

    /// Make `code` the board's, which every PE whose host gives the same
    /// window joins ([`Machine::join_code`]).
    pub(super) fn share_code(&self, code: &Arc<super::board::Code>) {
        *self.code.lock().unwrap_or_else(|error| error.into_inner()) = Some(Arc::clone(code));
    }

    /// Note the pages of `code`'s blocks, compiled before this hub was
    /// there to note them ([`Self::compiling`]): an IC IVAU of one must
    /// reach every PE that may run them.
    pub(super) fn note_code(&self, code: &super::board::Code) {
        for frame in code.cache_at(super::board::LockSite::Other).index.frames() {
            self.compiling(frame);
        }
    }

    /// Whether any PE may have compiled from the page holding `physical`.
    pub(super) fn has_code(&self, physical: u64) -> bool {
        let (word, bit) = self.code_pages.at(physical);
        word.load(Ordering::SeqCst) & bit != 0
    }

    /// Whether every PE in `posted` applied its maintenance, stopped
    /// reading mail, or sleeps or is outside its run loop, from which it
    /// reads its mail before it runs again; the others stay in `posted`,
    /// interrupted.
    ///
    /// A PE between two fetched blocks ([`Inbox::active`] even) counts as
    /// having applied it too: it holds no translation it will use before
    /// its next [`Self::enter_active`], which finds the mail and sends it
    /// back to read it first. The two cannot miss each other: the post
    /// stores `pending` and then this loads `active`; the entry stores
    /// `active` and then loads `pending`; all four are SeqCst, so in their
    /// single order either this load sees the entry (and waits for the PE
    /// to apply the mail or leave again) or the entry's load sees the post
    /// (and the PE does not enter).
    pub(super) fn applied(&self, posted: &mut Vec<(usize, u64)>) -> bool {
        posted.retain(|&(pe, serial)| {
            let inbox = &self.inboxes[pe];
            inbox.applied.load(Ordering::Acquire) < serial
                && inbox.live.load(Ordering::Acquire)
                && !inbox.sleeping.load(Ordering::SeqCst)
                && !inbox.outside.load(Ordering::SeqCst)
                && inbox.active.load(Ordering::SeqCst) & 1 != 0
        });
        posted.is_empty()
    }

    /// PE `pe` is about to act on the block it fetched ([`Inbox::active`]):
    /// whether it may. Not when mail came since its last read: it then
    /// stays between blocks, reads the mail and fetches again, as what it
    /// fetched may have used a translation the mail removes. See
    /// [`Self::applied`] for why no post can be missed.
    pub(super) fn enter_active(&self, pe: usize) -> bool {
        let inbox = &self.inboxes[pe];
        let active = inbox.active.load(Ordering::Relaxed);
        debug_assert!(active & 1 == 0, "entered twice");
        inbox.active.store(active.wrapping_add(1), Ordering::SeqCst);
        if inbox.pending.load(Ordering::SeqCst) {
            inbox
                .active
                .store(active.wrapping_add(2), Ordering::Release);
            return false;
        }
        true
    }

    /// PE `pe` is done with its fetched block and is between blocks again.
    /// PE `pe` completes the exit `blocker` names, or 0: runs natively
    /// ([`Inbox::doing`]).
    pub(super) fn set_doing(&self, pe: usize, blocker: u64) {
        self.inboxes[pe].doing.store(blocker, Ordering::Relaxed);
    }

    /// What PE `pe` was last doing in a fetched block
    /// ([`step_timing::BarrierWaits`]).
    fn doing(&self, pe: usize) -> u64 {
        match self.inboxes[pe].doing.load(Ordering::Relaxed) {
            0 => step_timing::BLOCKER_NATIVE,
            blocker => blocker,
        }
    }

    pub(super) fn leave_active(&self, pe: usize) {
        let inbox = &self.inboxes[pe];
        let active = inbox.active.load(Ordering::Relaxed);
        if active & 1 != 0 {
            inbox
                .active
                .store(active.wrapping_add(1), Ordering::Release);
        }
    }

    pub(super) fn pending(&self, pe: usize) -> bool {
        self.inboxes[pe].pending.load(Ordering::SeqCst)
    }

    /// PE `pe`'s mail, if any, which its machine applies and then
    /// acknowledges with [`Self::acknowledge`].
    pub(super) fn take(&self, pe: usize) -> Option<Mail> {
        self.take_into(pe, &mut Vec::new())
    }

    /// [`Self::take`], leaving the emptied buffer `spare` in the inbox for
    /// the next mail, so a read and the next post allocate nothing.
    pub(super) fn take_into(&self, pe: usize, spare: &mut Vec<Maintenance>) -> Option<Mail> {
        let inbox = &self.inboxes[pe];
        if !inbox.pending.swap(false, Ordering::AcqRel) {
            return None;
        }
        spare.clear();
        let mut mail = inbox.mail.lock().unwrap_or_else(|e| e.into_inner());
        Some(Mail {
            start: mail.start.take(),
            event: core::mem::take(&mut mail.event),
            maintenance: core::mem::replace(&mut mail.maintenance, core::mem::take(spare)),
            serial: mail.serial,
            merged: core::mem::take(&mut mail.merged),
        })
    }

    pub(super) fn acknowledge(&self, pe: usize, serial: u64) {
        self.inboxes[pe].applied.fetch_max(serial, Ordering::AcqRel);
    }

    /// Publish PE `pe`'s machine's counts, for the home machine's reports.
    pub fn publish(&self, pe: usize, counts: Counts) {
        *self.inboxes[pe]
            .counts
            .lock()
            .unwrap_or_else(|e| e.into_inner()) = counts;
    }

    /// PE `pe`'s machine's last published counts.
    pub fn published(&self, pe: usize) -> Counts {
        *self.inboxes[pe]
            .counts
            .lock()
            .unwrap_or_else(|e| e.into_inner())
    }

    /// Add System exits PE `pe`'s machine took
    /// ([`Machine::take_system_exits`]), for the home machine's reports.
    pub fn add_system_exits(&self, pe: usize, exits: &step_timing::SystemExits) {
        if exits.is_empty() {
            return;
        }
        self.inboxes[pe]
            .system_exits
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .merge(exits);
    }

    /// Add the hot-block samples `take` puts in the board's table
    /// (`Machine::take_hot_samples`), for the home machine's reports.
    pub fn add_hot_samples(&self, take: impl FnOnce(&mut step_timing::HotBlocks)) {
        take(&mut self.hot.lock().unwrap_or_else(|e| e.into_inner()));
    }

    /// Every PE's hot-block samples added since the last call.
    pub fn take_hot_samples(&self) -> step_timing::HotBlocks {
        core::mem::take(&mut *self.hot.lock().unwrap_or_else(|e| e.into_inner()))
    }

    /// PE `pe`'s System exits added since the last call.
    pub fn take_system_exits(&self, pe: usize) -> step_timing::SystemExits {
        core::mem::take(
            &mut *self.inboxes[pe]
                .system_exits
                .lock()
                .unwrap_or_else(|e| e.into_inner()),
        )
    }

    /// The count of device line changes PE `pe` has had.
    pub(super) fn lines_epoch(&self, pe: usize) -> u64 {
        self.inboxes[pe].lines.load(Ordering::Acquire)
    }

    /// PE `pe`'s device lines changed; the caller wakes it ([`Self::wake`])
    /// once it has released the devices' lock.
    fn lines_changed_unwoken(&self, pe: usize) {
        self.inboxes[pe].lines.fetch_add(1, Ordering::SeqCst);
        self.interrupt(pe);
    }

    /// The state of the physical cluster `key`, made by `make` if no PE of
    /// it has run yet.
    pub(super) fn cluster(&self, key: u64, make: impl FnOnce() -> Cluster) -> Cluster {
        self.clusters
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .entry(key)
            .or_insert_with(make)
            .clone()
    }

    /// Leave PE `pe`'s forwarded `access` for the home machine, and bring
    /// that machine to it.
    fn request(&self, pe: usize, access: Forwarded) {
        *self.request.lock().unwrap_or_else(|e| e.into_inner()) = Some(Request {
            pe,
            access,
            answer: None,
        });
        self.requested.store(true, Ordering::SeqCst);
        let home = self.home.load(Ordering::Relaxed);
        self.post(home, |_| {});
        self.wake(home);
    }

    /// The forwarded access waiting for the home machine, and its PE.
    fn take_request(&self) -> Option<(usize, Forwarded)> {
        if !self.requested.swap(false, Ordering::AcqRel) {
            return None;
        }
        let request = self.request.lock().unwrap_or_else(|e| e.into_inner());
        request.as_ref().map(|request| (request.pe, request.access))
    }

    fn answer(&self, answer: Answer) {
        if let Some(request) = self
            .request
            .lock()
            .unwrap_or_else(|e| e.into_inner())
            .as_mut()
        {
            request.answer = Some(answer);
        }
    }

    /// PE `pe`'s answer, once the home machine gave it.
    fn answered(&self, pe: usize) -> Option<Answer> {
        let mut request = self.request.lock().unwrap_or_else(|e| e.into_inner());
        if request
            .as_ref()
            .is_some_and(|request| request.pe == pe && request.answer.is_some())
        {
            return request.take()?.answer;
        }
        None
    }

    /// Stop every running PE but `home` and `requester`: each parks at its
    /// next return to the engine, or already waits there or sleeps, and
    /// reads its mail before it runs again.
    /// `relax` is each turn of the wait ([`Host::relax`]).
    fn stop_others(&self, home: usize, requester: usize, mut relax: impl FnMut()) {
        self.stopped.store(true, Ordering::SeqCst);
        for pe in self.others(home).filter(|&pe| pe != requester) {
            let inbox = &self.inboxes[pe];
            while !(inbox.parked.load(Ordering::SeqCst)
                || inbox.sleeping.load(Ordering::SeqCst)
                || !inbox.live.load(Ordering::Acquire))
            {
                self.interrupt(pe);
                relax();
            }
        }
    }

    /// End the run: every PE but `home` parks for good and frees its blocks,
    /// waited for while `waiting` holds. A PE that does not get there only
    /// keeps its memory.
    fn finish(&self, home: usize, mut waiting: impl FnMut() -> bool) {
        self.finishing.store(true, Ordering::Release);
        self.stopped.store(true, Ordering::SeqCst);
        for pe in self.others(home) {
            let inbox = &self.inboxes[pe];
            while inbox.live.load(Ordering::Acquire)
                && !inbox.discarded.load(Ordering::Acquire)
                && waiting()
            {
                self.interrupt(pe);
                self.wake(pe);
                core::hint::spin_loop();
            }
        }
    }

    fn restart_others(&self) {
        self.stopped.store(false, Ordering::Release);
    }

    /// PE `pe`'s machine leaves its run loop (`true`) or enters it, before
    /// it next reads its mail: ordered against a broadcast's look at it
    /// ([`Self::applied`]) as `sleeping` is.
    pub(super) fn set_outside(&self, pe: usize, outside: bool) {
        self.inboxes[pe].outside.store(outside, Ordering::SeqCst);
        self.section(pe);
    }

    fn set_parked(&self, pe: usize, parked: bool) {
        self.inboxes[pe].parked.store(parked, Ordering::SeqCst);
        self.section(pe);
    }

    /// Whether the host work a release of the devices' lock leaves
    /// ([`Kick::released`]) may run on PE `pe` now: only outside its run
    /// loop or asleep, where a broadcast does not wait for it
    /// ([`Self::applied`]). A PE inside its loop would read no mail while
    /// that work waits on a host device. An unknown PE (`usize::MAX`) or a
    /// parked one leaves it too.
    pub(super) fn serves_released(&self, pe: usize) -> bool {
        self.inboxes.get(pe).is_some_and(|inbox| {
            inbox.outside.load(Ordering::SeqCst) || inbox.sleeping.load(Ordering::SeqCst)
        })
    }

    /// Where PE `pe` is, for the report of a maintenance wait that overran
    /// ([`RequirementKind::MaintenanceOverrun`]): parked waiting in the
    /// engine (on another PE's broadcast or a stop), or in its run loop (an
    /// exit, a native slice or a wait for the devices' lock).
    pub(super) fn waiting_state(&self, pe: usize) -> super::PeState {
        use super::PeState;
        let Some(inbox) = self.inboxes.get(pe) else {
            return PeState::Unknown;
        };
        if inbox.parked.load(Ordering::SeqCst) {
            PeState::Parked
        } else if inbox.outside.load(Ordering::SeqCst) || inbox.sleeping.load(Ordering::SeqCst) {
            PeState::Outside
        } else {
            PeState::Running
        }
    }

    /// Whether the home machine is outside its run loop: it advances the
    /// devices with time only from inside it.
    pub(super) fn home_outside(&self) -> bool {
        let home = self.home.load(Ordering::Acquire);
        self.inboxes
            .get(home)
            .is_some_and(|inbox| inbox.outside.load(Ordering::Acquire))
    }

    /// The counts in a millisecond of the count every PE reads.
    pub(super) fn counts_per_milli(&self) -> u64 {
        self.clock.counts_per_milli()
    }

    /// PE `pe` enters the board's grace section when it is neither outside
    /// its run loop nor parked, and leaves it otherwise. Only the PE itself
    /// changes its two flags, so this needs no lock.
    fn section(&self, pe: usize) {
        let inbox = &self.inboxes[pe];
        let inside =
            !inbox.outside.load(Ordering::Relaxed) && !inbox.parked.load(Ordering::Relaxed);
        let grace = &self.board.grace;
        match (inside, grace.inside(pe)) {
            (true, false) => grace.enter(pe),
            (false, true) => grace.leave(pe),
            _ => {}
        }
    }

    /// PE `pe` is at its run loop's top, where it holds nothing of the
    /// board's code it does not check again.
    pub(super) fn pass(&self, pe: usize) {
        if self.board.grace.inside(pe) {
            self.board.grace.pass(pe);
        }
    }

    /// Whether a PE but `from` acts on a fetched block ([`Inbox::active`]),
    /// which a broadcast would wait for ([`Self::applied`]).
    pub(super) fn others_active(&self, from: usize) -> bool {
        self.others(from).any(|pe| {
            let inbox = &self.inboxes[pe];
            inbox.active.load(Ordering::SeqCst) & 1 != 0
                && !inbox.sleeping.load(Ordering::SeqCst)
                && !inbox.outside.load(Ordering::SeqCst)
        })
    }

    fn others(&self, from: usize) -> impl Iterator<Item = usize> + '_ {
        (0..self.inboxes.len())
            .filter(move |&pe| pe != from && self.inboxes[pe].live.load(Ordering::Acquire))
    }
}

/// Why a parallel PE cannot acquire its hub ownership.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum ParallelAdmissionError {
    MachineAttached,
    PeOutOfBounds { pe: usize, pes: usize },
    PeOwned { pe: usize },
    HomeOwned,
    HomeUnavailable,
    OwnerEpochExhausted,
    PeersNotHeldInReset,
}
impl core::fmt::Display for ParallelAdmissionError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::MachineAttached => f.write_str("machine already owns a parallel PE"),
            Self::PeOutOfBounds { pe, pes } => {
                write!(f, "parallel PE {pe} is outside the hub's {pes} slots")
            }
            Self::PeOwned { pe } => write!(f, "parallel PE {pe} already has a live owner"),
            Self::HomeOwned => f.write_str("parallel hub already has a home owner"),
            Self::HomeUnavailable => f.write_str("parallel hub has no admitted home owner"),
            Self::OwnerEpochExhausted => f.write_str("parallel ownership epochs exhausted"),
            Self::PeersNotHeldInReset => {
                f.write_str("only PEs held in reset can move to their own machines")
            }
        }
    }
}
impl core::error::Error for ParallelAdmissionError {}

/// The wait for other PEs to apply posted maintenance past which it is
/// reported, as a fraction of a second of the system count: half a second,
/// the time a PE may spend in one exit before XNU's watchdog panics.
pub(super) const MAINTENANCE_WAIT_DIVISOR: u64 = 2;

/// One unique PE claim kept by its control owner through all native
/// invocations. The inbox independently keeps the budget allocation.
pub(super) struct Attached {
    hub: Arc<Hub>,
    pe: usize,
    epoch: u64,
    home: bool,
}
impl Attached {
    fn new(
        hub: &Arc<Hub>,
        pe: usize,
        home: bool,
    ) -> Result<(Arc<Self>, chain_control::Owner), ParallelAdmissionError> {
        let inbox = hub
            .inboxes
            .get(pe)
            .ok_or(ParallelAdmissionError::PeOutOfBounds { pe, pes: hub.pes() })?;
        let epoch = hub
            .next_owner
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |next| {
                next.checked_add(1).filter(|value| *value < u64::MAX)
            })
            .map_err(|_| ParallelAdmissionError::OwnerEpochExhausted)?;
        inbox
            .owner
            .compare_exchange(0, epoch, Ordering::AcqRel, Ordering::Acquire)
            .map_err(|_| ParallelAdmissionError::PeOwned { pe })?;
        let mut attached = Self {
            hub: Arc::clone(hub),
            pe,
            epoch,
            home: false,
        };
        if home {
            hub.home_owner
                .compare_exchange(0, epoch, Ordering::AcqRel, Ordering::Acquire)
                .map_err(|_| ParallelAdmissionError::HomeOwned)?;
            attached.home = true;
        }
        let attached = Arc::new(attached);
        // SAFETY: this is the only writer minted for the newly acquired
        // epoch. The Owner keeps its claim independently of field drop order.
        let control = unsafe {
            chain_control::Owner::from_claimed(Arc::clone(&inbox.chain), Arc::clone(&attached))
        };
        Ok((attached, control))
    }
}
impl Drop for Attached {
    fn drop(&mut self) {
        let inbox = &self.hub.inboxes[self.pe];
        if inbox
            .owner
            .compare_exchange(self.epoch, u64::MAX, Ordering::AcqRel, Ordering::Acquire)
            .is_ok()
        {
            inbox.live.store(false, Ordering::Release);
            if self.home {
                let _ = self.hub.home_owner.compare_exchange(
                    self.epoch,
                    0,
                    Ordering::AcqRel,
                    Ordering::Acquire,
                );
            }
            inbox.owner.store(0, Ordering::Release);
        }
    }
}

impl<H: Host, D: Devices> Machine<H, D> {
    /// Copy this owner's last materialized Cpu image at an engine boundary.
    /// No executing native GuestState or another PE's frame is inspected.
    /// This is optional stale diagnostic data, never a retirement receipt.
    pub fn observe_parallel_source(&mut self, phase: SourcePhase) {
        // A failed native recipe can leave an unchecked partial frame. Keep
        // the preceding materialized sample stale instead of publishing it as
        // a new safe boundary.
        if self.native_failure.is_some() {
            return;
        }
        let Some(hub) = self.parallel.as_ref() else {
            return;
        };
        let Some(inbox) = self.observation_inbox(hub) else {
            return;
        };
        // observation_inbox verified the claim, which self.attached keeps
        // live throughout this publication.
        let owner = inbox.owner.load(Ordering::Relaxed);
        let source = MaterializedSource {
            host_counter: self.host.counter(),
            phase,
            pc: self.cpu.pc,
            sp: self.cpu.state.sp,
            fp: self.cpu.state.x[29],
            lr: self.cpu.state.x[30],
            el: self.cpu.pstate.el,
            daif: self.cpu.pstate.daif,
        };
        inbox.source.publish(owner, source);
    }

    fn observation_inbox<'a>(&self, hub: &'a Arc<Hub>) -> Option<&'a Inbox> {
        if !hub.source_observations_enabled() {
            return None;
        }
        let owner = self.attached.as_ref()?;
        if owner.pe != self.current
            || !Arc::ptr_eq(&owner.hub, hub)
            || !matches!(self.peers.get(self.current), Some(Peer::Running))
        {
            return None;
        }
        hub.inboxes.get(owner.pe)
    }

    /// Become the home machine of a board whose PEs each run on their own
    /// host core: the running PE stays here, and every PE held in reset will
    /// start on its own machine ([`Self::start_parallel`]). Every PE reads
    /// the hub's [`SharedClock`] from now on, continuing this machine's
    /// count, and waits no longer leap it.
    pub fn enter_parallel(&mut self, hub: Arc<Hub>) -> Result<(), ParallelAdmissionError> {
        if self.attached.is_some() || self.parallel.is_some() {
            return Err(ParallelAdmissionError::MachineAttached);
        }
        if self
            .peers
            .iter()
            .any(|peer| !matches!(peer, Peer::Running | Peer::Off))
        {
            return Err(ParallelAdmissionError::PeersNotHeldInReset);
        }
        let (attached, control) = Attached::new(&hub, self.current, true)?;
        // Its blocks so far were compiled before the hub noted pages.
        hub.note_code(&self.code);
        // The home machine's code becomes the board's, in the one arena.
        if self.code.arena.is_some() {
            hub.share_code(&self.code);
            self.code_hub = Some(Arc::clone(&hub));
            self.read_count_natively();
        }
        self.adopt_parallel_control(control);
        // What native code may complete depends on going parallel.
        self.forget_derived();
        for peer in &mut self.peers {
            match peer {
                Peer::Running => {}
                Peer::Off => *peer = Peer::Remote,
                Peer::Parked(_) | Peer::Remote => {
                    unreachable!("peer states checked before claiming the hub");
                }
            }
        }
        // The running PE's cluster state is the one its other PEs join.
        let key = self
            .profile
            .processors
            .get(self.current)
            .map_or(0, |identity| {
                identity.mpidr & super::processors::PHYSICAL_CLUSTER_MASK
            });
        hub.cluster(key, || Cluster {
            ctrr: self.cpu.ctrr.clone(),
            impdef: self.cpu.apple.impdef.cluster_registers(),
        });
        let host = self.host.counter();
        self.clock
            .seed(&hub.clock, host, self.profile.counter_frequency);
        self.clock.attach(Arc::clone(&hub.clock), self.current);
        hub.home.store(self.current, Ordering::Relaxed);
        self.share_kept_pages(hub.pes());
        self.share_lower_globals(&hub);
        self.idle_skip = false;
        hub.set_live(self.current, true);
        self.attached = Some(attached);
        self.parallel = Some(hub);
        Ok(())
    }

    /// Run the board's code from now on, in place of this machine's own,
    /// which held nothing yet: its host gives the same code window.
    pub(super) fn join_code(&mut self, hub: &Arc<Hub>) {
        let shared = hub
            .code
            .lock()
            .unwrap_or_else(|error| error.into_inner())
            .clone();
        let Some(shared) = shared else {
            return;
        };
        // Built on the hub's code from its worker profile.
        if Arc::ptr_eq(&shared, &self.code) {
            self.code_hub = Some(Arc::clone(hub));
            self.read_count_natively();
            return;
        }
        let window = |code: &super::board::Code| code.arena.as_ref().map(|arena| arena.window);
        if window(&shared).is_some() && window(&shared) == window(&self.code) {
            debug_assert_eq!(self.code.blocks.live(), 0, "a PE joins before it installs");
            self.code = shared;
            self.code_hub = Some(Arc::clone(hub));
            self.read_count_natively();
        }
    }

    /// Set this PE's clock reader up for native count reads, as compiling a
    /// count read's thunk does: on a shared board this PE runs thunks other
    /// PEs compiled, and a reader left unset reads a count that never
    /// moves, so a guest's timed wait on this PE never ends.
    fn read_count_natively(&mut self) {
        if let Some(tsc) = self.host.tsc_counter() {
            self.clock.native(tsc);
        }
    }

    /// Whether native count reads on this PE follow the host counter, or
    /// its host has none (no code reads the count natively then).
    pub fn count_reads_natively(&self) -> bool {
        self.clock.reads_natively() || self.host.tsc_counter().is_none()
    }

    /// Run on the hub's control of this PE from now on, with this machine's
    /// contexts, its board's tables and stamps of its board: cell probes
    /// count a cell's heat only through the control's stamped cells, and
    /// without them no exit is ever promoted.
    fn adopt_parallel_control(&mut self, control: chain_control::Owner) {
        let contexts = self.chain.contexts();
        let (stamps, cells) = self.chain.stamps();
        self.chain = control;
        self.chain.initialize(
            self.code.table.address(),
            self.itlb.address(),
            contexts,
            [self.code.table.mask(), self.itlb.mask()],
        );
        self.chain.set_stamps(stamps, cells);
        self.attach_stamps();
    }

    /// Give this PE's control stamps of its board's block table and that
    /// table's stamped cells, unless it holds them: a machine that joined
    /// another board holds its first board's.
    pub(super) fn attach_stamps(&mut self) {
        let code = Arc::clone(&self.code);
        let mut cache = code.cache_at(super::board::LockSite::Other);
        let cells = cache.targets.stamped_cells();
        let (stamps, held) = self.chain.stamps();
        if held == cells && cache.targets.holds_stamps(stamps) {
            return;
        }
        let stamps = cache.targets.new_stamps();
        drop(cache);
        self.chain.set_stamps(stamps, cells);
    }

    /// Whether this machine runs one PE of a board whose PEs each run on
    /// their own host core.
    pub fn runs_parallel(&self) -> bool {
        self.parallel.is_some()
    }

    /// Start this machine's PE at `address`, as the device that started it
    /// asked through the hub; the home machine entered parallel first.
    pub fn start_parallel(
        &mut self,
        hub: Arc<Hub>,
        core_locks: Arc<ctrr::CoreLocks>,
        address: u64,
    ) -> Step {
        if self.attached.is_some() || self.parallel.is_some() {
            return Err(self.unsupported(RequirementKind::ParallelAdmission(
                ParallelAdmissionError::MachineAttached,
            )));
        }
        if hub.home_owner.load(Ordering::Acquire) == 0 {
            return Err(self.unsupported(RequirementKind::ParallelAdmission(
                ParallelAdmissionError::HomeUnavailable,
            )));
        }
        let (attached, control) = Attached::new(&hub, self.current, false)
            .map_err(|error| self.unsupported(RequirementKind::ParallelAdmission(error)))?;
        self.join_code(&hub);
        self.adopt_parallel_control(control);
        for (pe, peer) in self.peers.iter_mut().enumerate() {
            if pe != self.current {
                *peer = Peer::Remote;
            }
        }
        self.clock.attach(Arc::clone(&hub.clock), self.current);
        self.idle_skip = false;
        self.cpu.core_locks = core_locks;
        self.cpu.memory_lock = self.devices.memory_lock();
        self.share_kept_pages(hub.pes());
        self.share_lower_globals(&hub);
        hub.set_live(self.current, true);
        self.attached = Some(attached);
        self.parallel = Some(hub);
        self.restart(self.current, address)
    }

    /// Apply what other PEs left for this one: changed device lines, an
    /// event, maintenance, and a start of a powered-down PE.
    pub(super) fn read_mail(&mut self) -> Step {
        let Some(hub) = &self.parallel else {
            return Ok(());
        };
        // Acquire BEFORE looking at any source. A later notification must
        // remain different even if this drain happens to consume its data.
        let observed = hub.inboxes[self.current]
            .chain
            .interrupt_epoch()
            .load(Ordering::Acquire);
        // Nearly every return to the engine finds nothing, and every PE
        // returns all the time: this path stays off the hub's reference
        // count, which one cache line would serve them all.
        let served = match hub.home.load(Ordering::Relaxed) == self.current {
            true => hub.requested.load(Ordering::Relaxed),
            false => hub.stopped.load(Ordering::SeqCst),
        };
        if !served
            && self.deferred_code.is_empty()
            && hub.lines_epoch(self.current) == self.lines_epoch
            && !hub.pending(self.current)
        {
            self.chain.acknowledge_interrupt(observed);
            return Ok(());
        }
        let hub = Arc::clone(hub);
        let result = (|| {
            if hub.home.load(Ordering::Relaxed) == self.current {
                self.serve_forwarded(&hub)?;
            } else if hub.stopped.load(Ordering::SeqCst) {
                self.park(&hub)?;
            }
            for page in core::mem::take(&mut self.deferred_code) {
                self.maintain(Maintenance::Code(page))?;
            }
            let epoch = hub.lines_epoch(self.current);
            if epoch != self.lines_epoch {
                self.lines_epoch = epoch;
                self.device_lines = None;
            }
            if !hub.pending(self.current) {
                return Ok(());
            }
            let mut spare = core::mem::take(&mut self.mail_spare);
            let Some(mail) = hub.take_into(self.current, &mut spare) else {
                self.mail_spare = spare;
                return Ok(());
            };
            if mail.event && !self.waiting.is_some_and(|wait| wait.down) {
                self.cpu.signal_event();
            }
            self.statistics.itlb_sources[super::ItlbSource::MailMerged as usize] += mail.merged;
            let posted = hub.inboxes[self.current]
                .posted_at
                .swap(0, Ordering::Relaxed);
            let read = hub.ticks();
            for maintenance in &mail.maintenance {
                self.maintain(*maintenance)?;
            }
            hub.acknowledge(self.current, mail.serial);
            let returned = core::mem::take(&mut self.mail_returned);
            if posted != 0 && read != 0 {
                self.note_mail_wait(posted, returned, read, hub.ticks());
            }
            self.mail_spare = mail.maintenance;
            if let Some(address) = mail.start {
                self.start(Start {
                    cpu: self.current,
                    address,
                })?;
            }
            Ok(())
        })();
        if result.is_ok() {
            self.chain.acknowledge_interrupt(observed);
        }
        result
    }

    /// Count a TLB maintenance mail first posted at host tick `posted`,
    /// whose PE's native run returned at `returned` (0 when it was not in
    /// one), read at `read` and applied by `applied`
    /// ([`super::Statistics::mail_waits`]).
    fn note_mail_wait(&mut self, posted: u64, returned: u64, read: u64, applied: u64) {
        use super::reports::MailWait;
        let latency = read.saturating_sub(posted);
        let native = match returned {
            0 => 0,
            returned => returned.saturating_sub(posted).min(latency),
        };
        let waits = &mut self.statistics.mail_waits;
        waits[MailWait::Count as usize] += 1;
        waits[MailWait::Latency as usize] += latency;
        waits[MailWait::Native as usize] += native;
        waits[MailWait::Exit as usize] += latency - native;
        waits[MailWait::Apply as usize] += applied.saturating_sub(read);
        if latency > waits[MailWait::Longest as usize] {
            waits[MailWait::Longest as usize] = latency;
            waits[MailWait::LongestNative as usize] = native;
        }
    }

    /// Whether the running PE waits in WFE on an armed monitor while other
    /// cores run: another core's store to its granule clears the monitor,
    /// which sends it the event, and no interrupt tells of that store, so
    /// the wait polls the granule (`observe_writes`) instead of halting.
    pub(super) fn polls_monitor(&self) -> bool {
        self.parallel.is_some()
            && self.cpu.monitor.is_some()
            && self
                .waiting
                .is_some_and(|wait| wait.kind == super::WaitKind::Event && !wait.down)
    }

    /// Compile the block at `pc`, and whether the caller fetches again: mail
    /// reached this PE meanwhile, which it reads before it runs anything, or
    /// the block went. A compile takes far longer than a maintenance
    /// broadcast should wait, and runs no guest code.
    ///
    /// It runs outside the board's grace section, so a retirement does not
    /// wait for it: the block it published may have gone, and its slot been
    /// given to another block, before this PE is back inside. Once inside
    /// the slot is checked again: still live and the block of `pc`, it stays
    /// so until this PE passes its loop's top.
    pub(super) fn compile_aside(
        &mut self,
        pc: u64,
        physical: u64,
        variant: Variant,
    ) -> (Result<Fetched, super::Stop>, bool) {
        let Some(hub) = self.parallel.clone() else {
            return (self.compile(pc, physical, variant), false);
        };
        hub.set_outside(self.current, true);
        let compiled = self.compile(pc, physical, variant);
        hub.set_outside(self.current, false);
        let gone = match &compiled {
            Ok(Fetched::Block(slot)) => {
                let id = self.known_variant(variant);
                !self.code.blocks.installed(*slot).is_some_and(|block| {
                    (block.pc, block.physical, Some(block.variant)) == (pc, physical, id)
                })
            }
            _ => false,
        };
        (compiled, gone || hub.pending(self.current))
    }

    /// Run `work`, which uses no translation this PE may hold and can take
    /// long, without holding up the other PEs' broadcasts: a device access
    /// behind another PE's, a halt and the timer around it, or the host's
    /// work between two of the run loop's slices. This PE reads its mail
    /// before it uses one again. Nested, the outer run's state returns.
    pub fn outside_run<R>(&mut self, work: impl FnOnce(&mut Self) -> R) -> R {
        let was = self.parallel.as_ref().map(|hub| {
            let was = hub.inboxes[self.current]
                .outside
                .swap(true, Ordering::SeqCst);
            hub.section(self.current);
            was
        });
        let result = work(self);
        if let (Some(hub), Some(was)) = (&self.parallel, was) {
            hub.set_outside(self.current, was);
        }
        result
    }

    /// Run the host's `work` on this PE's core outside its run loop, as
    /// [`Self::outside_run`], while staying reachable for guest work: a wake
    /// that brings this PE an interrupt kicks its core and marks the shared
    /// count, as its own timer's deadline does, so the count holds a stall
    /// limit past either if the PE stays out (`clock`). `work` looks for
    /// that guest work between its parts ([`Self::guest_watch`]). Costs the
    /// service path a clock read, two stores and a few loads; nothing on
    /// the guest's.
    pub fn serve_host<R>(&mut self, work: impl FnOnce(&mut Self) -> R) -> R {
        // A nested service keeps the outer one's start.
        let outer = self.serving_since;
        self.serving_since = Some(outer.unwrap_or_else(|| self.host.counter()));
        let result = match self.parallel.clone() {
            None => work(self),
            Some(hub) => {
                let now = self.now();
                let mark = self.serving_mark(now);
                self.clock.serving(mark);
                hub.set_accepts(self.current, self.accepted_lines());
                let was = hub.set_serving(self.current, Some(self.lines_epoch));
                let result = self.outside_run(work);
                hub.set_serving(self.current, was);
                result
            }
        };
        self.serving_since = outer;
        result
    }

    /// The count from which the running PE, leaving its run loop at count
    /// `now` to serve the host, has guest work of its own waiting: its own
    /// next wake ([`Self::own_deadline`]), but not before `now`. Until it
    /// left it ran guest code, so a wake already past then waited for it no
    /// longer than the service has.
    pub(super) fn serving_mark(&mut self, now: u64) -> Option<u64> {
        self.own_deadline(now).map(|deadline| deadline.max(now))
    }

    /// What ends the host's work on this PE's core early: the guest work
    /// [`GuestWatch::pending`] finds for it, from the device lines it last
    /// read and its own timers' next deadline, in host counts, not before
    /// it began to serve ([`Self::serving_mark`]).
    pub fn guest_watch(&mut self) -> GuestWatch {
        let host = self.host.counter();
        let since = self.serving_since.unwrap_or(host);
        let now = self.now();
        let wake_at = self
            .own_deadline(now)
            .map(|deadline| self.clock.host_deadline(deadline).max(since));
        GuestWatch::new(
            self.parallel.clone(),
            self.current,
            self.lines_epoch,
            wake_at,
        )
    }

    /// The earliest count at which the running PE's own wake would bring it
    /// back to guest code, none while it is paused or powered down: its WFET
    /// or WFIT timeout, which ends its wait whatever its masks, and its
    /// timers and PMI, which Apple cores signal as FIQ, while it accepts the
    /// FIQ ([`Self::accepted_lines`]). A timer that is disabled or masked by
    /// its IMASK has no deadline (`timer::Channel::deadline`).
    pub(super) fn own_deadline(&mut self, now: u64) -> Option<u64> {
        if self.processor_paused(self.current) || self.waiting.is_some_and(|wait| wait.down) {
            return None;
        }
        let timeout = self.waiting.and_then(|wait| wait.timeout).map(|timeout| {
            if timeout.expired(now) {
                now
            } else {
                timeout.deadline(now)
            }
        });
        let fiq = if self.accepted_lines().fiq {
            [
                self.timer_deadline(),
                self.cpu.pmu.as_ref().and_then(|monitor| monitor.deadline()),
            ]
            .into_iter()
            .flatten()
            .min()
        } else {
            None
        };
        fiq.into_iter().chain(timeout).min()
    }

    /// The device lines that bring the running PE back to guest work now:
    /// those its core's CYC_OVRD does not disable and that end its WFI or
    /// WFE, masked or not, while it waits ([`Self::ready`]), or that it
    /// takes at its PSTATE and routing while it runs ([`Self::taken`]). A PE
    /// that runs with an interrupt masked leaves it pending until its own
    /// code unmasks it, and no other core waits on that sooner.
    pub(super) fn accepted_lines(&self) -> Lines {
        use crate::machine::exception::{DAIF_F, DAIF_I, Vector};
        if self.processor_paused(self.current) {
            return Lines::default();
        }
        let enabled = |irq: bool| self.cpu.wakes_for(Lines { irq, fiq: !irq });
        let takes = |vector: Vector, mask: u8| {
            self.waiting.is_some() || self.taken(self.cpu.interrupt_target(vector), mask)
        };
        Lines {
            irq: enabled(true) && takes(Vector::Irq, DAIF_I),
            fiq: enabled(false) && takes(Vector::Fiq, DAIF_F),
        }
    }

    /// Halt the host core until an interrupt, or, in parallel, also until
    /// mail or a device line change reaches this PE.
    pub(super) fn sleep(&mut self) {
        match self.parallel.clone() {
            Some(hub) => {
                let serves = self.waits_for_interrupt();
                hub.sleep(self.current, self.lines_epoch, serves, || {
                    self.host.wait_for_interrupt()
                })
            }
            None => self.host.wait_for_interrupt(),
        }
    }

    /// Wait for a wake on this core from host count `from` for the
    /// [`Self::idle_poll`] counts, or until `until`, whichever is first:
    /// whether the PE has something to look at. Nobody sends a polling PE a
    /// doorbell, and a halt, its timer and the doorbell that ends it take
    /// the host's hypervisor if it has one, at tens of microseconds an
    /// access.
    pub(super) fn poll_for_wake(&mut self, from: u64, until: u64) -> bool {
        let Some(hub) = &self.parallel else {
            return false;
        };
        let end = from.saturating_add(self.idle_poll);
        loop {
            if hub.woken(self.current, self.lines_epoch) || self.host.poll_interrupt() {
                return true;
            }
            let now = self.host.counter();
            if now >= until {
                return true;
            }
            if now >= end {
                return false;
            }
            core::hint::spin_loop();
        }
    }

    /// Fit [`Self::idle_poll`] to a wait that took `blocked` counts
    /// ([`idle_poll_after`]).
    pub(super) fn tune_idle_poll(&mut self, blocked: u64) {
        let most = self.profile.counter_frequency / super::processors::IDLE_POLL_HZ;
        self.idle_poll =
            idle_poll_after(self.idle_poll, blocked, most, self.host.halts_on_doorbell());
    }

    /// Complete a short WFE inside the scheduler's entry, without its wait
    /// passes: `Some` with the completion when the running PE, not the home
    /// PE and the only one of its machine, waits in WFE (no monitor armed,
    /// not powered down) and becomes ready ([`Self::ready`], the passes'
    /// own test) by its next own wake (event stream edge, timer, PMI, WFET
    /// timeout, device-line cache expiry) within [`Self::idle_poll`] host
    /// counts. That is the most the passes' own halt-poll would spin before
    /// they halt, at most 200 us (`IDLE_POLL_HZ`): far under the 0.5 s one
    /// exit may take, and no longer than today's spin to the same wake.
    ///
    /// The spin is a safe point (`Host::relax`) and serves host interrupts
    /// as the halt-poll does. Mail (another PE's SEV or maintenance) is read
    /// at once, before any block runs, and the WFE completes if that made
    /// it ready. A host call, the host's service, a host interrupt or a
    /// wake still ahead leave the wait to the passes (`None`), which then
    /// find it as they do now; so do the home PE and a run with a deadline.
    pub(super) fn short_event_wait(
        &mut self,
        run_deadline: Option<super::budget::Deadline>,
    ) -> Option<Step> {
        let hub = self.parallel.as_ref()?;
        let pe = self.current;
        let event_wait = self
            .waiting
            .is_some_and(|wait| wait.kind == super::WaitKind::Event && !wait.down);
        if run_deadline.is_some()
            || self.idle_poll == 0
            || !event_wait
            || self.cpu.monitor.is_some()
            || hub.home.load(Ordering::Relaxed) == pe
            || hub.woken(pe, self.lines_epoch)
            || self.processor_paused(pe)
            || self
                .peers
                .iter()
                .any(|peer| matches!(peer, Peer::Parked(_)))
            || self.devices.dma_deadline().is_some()
            || self.devices.waits_on_host()
        {
            return None;
        }
        let start = self.host.counter();
        let now = self.now();
        if !self.ready(pe, now) {
            let wake = self.own_wake(now)?;
            let until = self.clock.wake_at(wake, start);
            if until > start.saturating_add(self.idle_poll) {
                return None;
            }
            loop {
                let hub = self.parallel.as_ref()?;
                if hub.woken(pe, self.lines_epoch) {
                    if let Err(stop) = self.read_mail() {
                        return Some(Err(stop));
                    }
                    let now = self.now();
                    let hub = self.parallel.as_ref()?;
                    if hub.woken(pe, self.lines_epoch) || !self.ready(pe, now) {
                        self.statistics.sleep_ticks += self.host.counter().wrapping_sub(start);
                        return None;
                    }
                    break;
                }
                if self.host.poll_interrupt() {
                    self.statistics.sleep_ticks += self.host.counter().wrapping_sub(start);
                    return None;
                }
                if self.host.counter() >= until {
                    let now = self.now();
                    if !self.ready(pe, now) {
                        self.statistics.sleep_ticks += self.host.counter().wrapping_sub(start);
                        return None;
                    }
                    break;
                }
                self.host.relax();
            }
        }
        self.statistics.waits += 1;
        self.statistics.short_waits += 1;
        self.statistics.sleep_ticks += self.host.counter().wrapping_sub(start);
        // As the passes start a PE they found ready.
        self.slice_end = self.statistics.blocks + super::processors::QUANTUM;
        self.slice_deadline =
            self.host.counter() + self.profile.counter_frequency / super::processors::QUANTUM_HZ;
        let share = self.runnable();
        let work = self.work();
        self.clock.start_slice(|| self.host.counter(), work, share);
        let (retired, phases) = (self.cpu.retired, self.clock_phases());
        self.clock.busy(|| self.host.counter(), retired, phases);
        Some(self.wake())
    }

    /// The earliest count at which the running PE's own wait can end with
    /// nobody else acting: its event stream edge, timer, PMI and WFET
    /// timeout, and the expiry of its cached device lines.
    fn own_wake(&mut self, now: u64) -> Option<u64> {
        let wait = self.waiting?;
        let lines = self
            .device_lines
            .filter(|&(_, _, pe)| pe == self.current)
            .map(|(_, until, _)| until);
        [
            super::timer::next_event(&self.cpu, now),
            self.timer_deadline(),
            self.cpu.pmu.as_ref().and_then(|monitor| monitor.deadline()),
            wait.timeout.map(|timeout| timeout.deadline(now)),
            lines,
        ]
        .into_iter()
        .flatten()
        .min()
    }

    /// Maintenance another PE broadcast, reached from this PE's context.
    fn maintain(&mut self, maintenance: Maintenance) -> Step {
        match maintenance {
            Maintenance::Tlb(scope) => {
                self.applying_mail = true;
                self.tlb_maintained(
                    scope.reach(&self.cpu.tlbi_current()),
                    scope.everywhere(),
                    scope.named_asid(),
                );
                self.applying_mail = false;
            }
            Maintenance::TlbAll => {
                self.statistics.itlb_drops[4] += 1;
                self.applying_mail = true;
                self.tlb_maintained(mmu::tlbi::Reach::All, mmu::tlbi::Everywhere::All, None);
                self.applying_mail = false;
            }
            Maintenance::Code(Some(physical)) => {
                self.invalidate_posted_code_page(physical);
                self.host.code_invalidated(Some(physical));
            }
            Maintenance::Code(None) => {
                self.discard_code(Discard::Maintenance);
                self.host.code_invalidated(None);
            }
            Maintenance::Power(change) => {
                if self.processor_write().received().apply(&[change]).is_err() {
                    let requirement = self.requirement(RequirementKind::Native(
                        "another PE's CPU power change does not settle on this PE's machine",
                    ));
                    self.degrade(requirement);
                }
            }
        }
        Ok(())
    }

    /// Publish the context `controls` translate, before this PE's walks
    /// use it and before it next reads its mail ([`Hub::broadcast`]).
    pub(super) fn publish_translating(&self, controls: &mmu::Controls) {
        let Some(hub) = &self.parallel else {
            return;
        };
        let translating = &hub.inboxes[self.current].translating;
        let word = context_word(controls.regime, mmu::current_asid(controls));
        if translating.load(Ordering::Relaxed) != word {
            translating.store(word, Ordering::SeqCst);
        }
    }

    /// See [`GlobalSpan::widen`]: whether the leaf holding `address` widened
    /// this PE's span.
    pub(super) fn widen_lower_globals(&self, address: u64, size: u64) -> bool {
        self.parallel.as_ref().is_some_and(|hub| {
            let (first, last) = mmu::shadow::leaf(address, size);
            hub.inboxes[self.current].lower_globals.widen(first, last)
        })
    }

    /// Give the host what it keeps this PE's span in.
    fn share_lower_globals(&mut self, hub: &Hub) {
        self.host
            .share_lower_globals(Arc::clone(&hub.inboxes[self.current].lower_globals));
    }

    /// Send the CPU power changes this PE made to every other PE's machine,
    /// and wait until each running one has applied them.
    pub(super) fn forward_power(&mut self) -> Step {
        for change in core::mem::take(&mut self.forwarded_power) {
            self.post(Maintenance::Power(change));
            self.complete_maintenance()?;
        }
        Ok(())
    }

    /// An IC IVAU of the page holding `physical`, for the other PEs: not
    /// when no PE may have compiled from it, nor again before this PE's
    /// next DSB, which the writes of what it cleans preceded. A kernel
    /// cleans a page line by line, and its other lines add nothing.
    pub(super) fn post_code_page(&mut self, physical: u64) {
        let page = physical & !(FETCH_PAGE - 1);
        let needed = self.parallel.as_ref().is_some_and(|hub| hub.has_code(page));
        if needed && self.code_posted != Some(page) {
            self.code_posted = Some(page);
            self.post(Maintenance::Code(Some(page)));
        }
    }

    /// Keep `maintenance` for every other running PE. A TLBI or IC IVAU
    /// completes for them at this PE's next DSB, which sends what this PE
    /// kept in one piece of mail and waits for them
    /// ([`Self::complete_maintenance`]): a run of maintenance takes them
    /// from their code once. While no other PE acts on a fetched block,
    /// there is nobody to take from code: the mail goes at once, and once
    /// every PE it reaches counts as having applied it ([`Hub::applied`]),
    /// which stays so, the next DSB finds nothing outstanding and completes
    /// natively ([`Self::open_barrier`]).
    pub(super) fn post(&mut self, maintenance: Maintenance) {
        let Some(hub) = self.parallel.clone() else {
            return;
        };
        self.mirrors.barrier = stela_compiler::x86_64::register_read::Cell::default();
        self.outgoing.push(maintenance);
        if self.outgoing.len() == OUTGOING || !hub.others_active(self.current) {
            self.send_outgoing();
            if hub.applied(&mut self.posted) {
                self.open_barrier();
            }
        }
    }

    /// Send the maintenance this PE kept, which each PE applies at its next
    /// return to the engine, brought there by this.
    pub(super) fn send_outgoing(&mut self) {
        let Some(hub) = self.parallel.clone() else {
            return;
        };
        if self.outgoing.is_empty() {
            return;
        }
        let profile = self.observation_inbox(&hub).map(|inbox| &inbox.broadcasts);
        let started = profile.map(|_| self.host.counter());
        let held = &mut self.posted;
        let posted = hub.broadcast_each(self.current, &self.outgoing, |pe, serial| {
            match held.iter_mut().find(|(held, _)| *held == pe) {
                Some((_, held)) => *held = serial,
                None => held.push((pe, serial)),
            }
        });
        if let (Some(slot), Some(started)) = (profile, started) {
            for maintenance in &self.outgoing {
                let kind = match maintenance {
                    Maintenance::Tlb(_) | Maintenance::TlbAll => BroadcastKind::Tlb,
                    Maintenance::Code(_) => BroadcastKind::Code,
                    Maintenance::Power(_) => BroadcastKind::Power,
                };
                if let Some(active) = slot.begin(kind, started, posted) {
                    slot.finish(active, 0, 0);
                }
            }
        }
        self.outgoing.clear();
    }

    /// Wait until every other running PE has applied the maintenance this
    /// one posted, reading this PE's own mail meanwhile: two PEs may wait
    /// on each other. A start that arrives meanwhile is posted again.
    /// A wait past [`MAINTENANCE_WAIT_DIVISOR`] reports once
    /// [`RequirementKind::MaintenanceOverrun`], naming a PE that has not
    /// applied the maintenance, and goes on waiting: going on without it
    /// would leave that PE the old translations or code, and XNU's watchdog
    /// remains the last resort.
    pub(super) fn complete_maintenance(&mut self) -> Step {
        self.code_posted = None;
        self.mirrors.close_code_page();
        self.send_outgoing();
        if self.posted.is_empty() {
            return Ok(());
        }
        let Some(hub) = self.parallel.clone() else {
            return Ok(());
        };
        let mut posted = core::mem::take(&mut self.posted);
        let timed = self.timing_start();
        let mut blocker = None;
        let started = self.host.counter();
        self.observe_parallel_source(SourcePhase::BroadcastEnter);
        let profile = self.observation_inbox(&hub).map(|inbox| &inbox.broadcasts);
        let active =
            profile.and_then(|slot| slot.begin(BroadcastKind::Completion, started, posted.len()));
        let mut polls = 0u64;
        // A PE stopping the others need not wait for this one meanwhile.
        hub.set_parked(self.current, true);
        let mut applied = Ok(());
        let limit = self.profile.counter_frequency / MAINTENANCE_WAIT_DIVISOR;
        let mut reported = false;
        while !hub.applied(&mut posted) {
            if profile.is_some() {
                polls = polls.saturating_add(1);
            }
            if self.step_timing.is_some()
                && let Some(&(pe, _)) = posted.first()
            {
                blocker = Some(hub.doing(pe));
            }
            applied = applied.and(self.wait_for_others(&hub));
            self.host.relax();
            if !reported && let Some(&(pe, serial)) = posted.first() {
                let waited = self.host.counter().wrapping_sub(started);
                if waited >= limit {
                    // Reported once for this wait, which goes on: the
                    // unapplied PE still may hold the old translations.
                    reported = true;
                    self.statistics.degraded += 1;
                    let requirement = self.requirement(RequirementKind::MaintenanceOverrun {
                        pe,
                        serial,
                        unapplied: u32::try_from(posted.len()).unwrap_or(u32::MAX),
                        state: hub.waiting_state(pe),
                        ticks: waited,
                    });
                    self.unmodeled.push(requirement);
                }
            }
        }
        hub.set_parked(self.current, false);
        // `posted` is empty now: keep its buffer for the next DSB.
        if self.posted.is_empty() {
            self.posted = posted;
        }
        self.statistics.broadcasts += 1;
        if self.step_timing.is_some() {
            let ticks = self.timing_since(timed);
            self.system_exits.barriers.add(blocker, ticks);
        }
        let elapsed = self.host.counter().wrapping_sub(started);
        self.statistics.sync_ticks += elapsed;
        // Publish only at boundaries, including a returned maintenance error.
        if let (Some(slot), Some(active)) = (profile, active) {
            slot.finish(active, elapsed, polls);
        }
        self.observe_parallel_source(SourcePhase::BroadcastExit);
        applied
    }

    /// This machine's counts, as [`Hub::publish`] takes them.
    pub fn counts(&self) -> Counts {
        let statistics = &self.statistics;
        Counts {
            blocks: statistics.blocks,
            retired: self.retired_instructions() as u64,
            compiled: statistics.compiled,
            shared: statistics.shared,
            loops: {
                let (entries, iterated) = self.chain.loops();
                [statistics.self_loops, entries, iterated]
            },
            store_sources: statistics.store_sources,
            live_blocks: self.code.blocks.live() as u64,
            block_slots: self.code.blocks.len() as u64,
            indexed_pages: self.code.indexed_pages() as u64,
            code_releases: statistics.code_releases,
            compile_ticks: statistics.compile_ticks,
            page_ticks: statistics.page_ticks,
            shared_ticks: statistics.shared_ticks,
            install_ticks: statistics.install_ticks,
            walk_ticks: statistics.walk_ticks,
            refill_ticks: statistics.refill_ticks,
            due_transfers: statistics.due_transfers,
            transfer_ticks: statistics.transfer_ticks,
            line_queries: statistics.line_queries,
            line_ticks: statistics.line_ticks,
            sleep_ticks: statistics.sleep_ticks,
            spin_ticks: statistics.spin_ticks,
            halt_ticks: statistics.halt_ticks,
            short_waits: statistics.short_waits,
            sync_ticks: statistics.sync_ticks,
            mail_waits: statistics.mail_waits,
            waits: statistics.waits,
            broadcasts: statistics.broadcasts,
            forwards: statistics.forwards,
            exceptions: statistics.exceptions,
            device_accesses: statistics.device_accesses,
            fetches: statistics.fetches,
            itlb: statistics.itlb,
            fetch_paths: statistics.fetch_paths,
            itlb_misses: statistics.itlb_misses,
            itlb_drops: statistics.itlb_drops,
            lookup_forgets: statistics.lookup_forgets,
            itlb_sources: statistics.itlb_sources,
            transitions: [
                statistics.native_transitions[stela_compiler::x86_64::transition::GUARDED_ENTER],
                statistics.engine_transitions[stela_compiler::x86_64::transition::GUARDED_ENTER],
                statistics.native_transitions[stela_compiler::x86_64::transition::GUARDED_EXIT],
                statistics.engine_transitions[stela_compiler::x86_64::transition::GUARDED_EXIT],
                statistics.native_transitions[stela_compiler::x86_64::transition::SUPERVISOR_CALL],
                statistics.engine_transitions[stela_compiler::x86_64::transition::SUPERVISOR_CALL],
                statistics.native_transitions[stela_compiler::x86_64::transition::EXCEPTION_RETURN],
                statistics.engine_transitions[stela_compiler::x86_64::transition::EXCEPTION_RETURN],
            ],
            transition_misses: statistics.transition_misses,
            branch_misses: statistics.branch_misses,
            stamped: {
                let mut stamped = [0; chain::STAMP_COUNTS + 4];
                stamped[..chain::STAMP_COUNTS].copy_from_slice(&self.chain.stamp_counts());
                stamped[chain::STAMP_COUNTS] = statistics.stamped_tails;
                stamped[chain::STAMP_COUNTS + 1] = statistics.stamped_in_use;
                stamped[chain::STAMP_COUNTS + 2] = statistics.predicted_tails;
                stamped[chain::STAMP_COUNTS + 3] = statistics.lookup_hits;
                stamped
            },
            pac_fills: self.pointer.fills,
            phases: self
                .step_timing
                .as_ref()
                .map_or([0; step_timing::Phase::COUNT], |timing| timing.phases),
            exits: self
                .step_timing
                .as_ref()
                .map_or([(0, 0); step_timing::EXIT_SLOTS], |timing| timing.exits),
            levels: self
                .step_timing
                .as_ref()
                .map_or_else(Default::default, |timing| timing.levels),
            installs: self.install_causes(),
        }
    }

    /// Apply this PE's mail while it waits in the engine for other PEs,
    /// which may wait on it: an event, maintenance, and a start, which is
    /// posted again for the PE's next mail read.
    fn wait_for_others(&mut self, hub: &Arc<Hub>) -> Step {
        let mut spare = core::mem::take(&mut self.mail_spare);
        let Some(mail) = hub.take_into(self.current, &mut spare) else {
            self.mail_spare = spare;
            return Ok(());
        };
        if mail.event {
            self.cpu.signal_event();
        }
        let mut applied = Ok(());
        for &maintenance in &mail.maintenance {
            match maintenance {
                // The instruction this PE waits in may still name its
                // block: its code goes at the next mail read, before the PE
                // fetches again.
                Maintenance::Code(page) => self.deferred_code.push(page),
                _ => applied = applied.and(self.maintain(maintenance)),
            }
        }
        hub.acknowledge(self.current, mail.serial);
        self.mail_spare = mail.maintenance;
        if let Some(address) = mail.start {
            hub.start(self.current, address);
        }
        applied
    }

    /// The run is over: stop every other PE and free every PE's blocks for
    /// proving them. A second of the system count bounds the wait.
    pub fn discard_every_block(&mut self) {
        const WAIT: u64 = 24_000_000;
        self.discard_blocks();
        if let Some(hub) = self.parallel.clone() {
            let started = self.host.counter();
            // Each check is a turn of the wait ([`Host::relax`]).
            hub.finish(self.current, || {
                self.host.relax();
                self.host.counter().wrapping_sub(started) < WAIT
            });
        }
    }

    /// Run no guest code while the home machine has stopped the others.
    fn park(&mut self, hub: &Arc<Hub>) -> Step {
        let started = self.host.counter();
        self.observe_parallel_source(SourcePhase::ParkEnter);
        hub.set_parked(self.current, true);
        let mut applied = Ok(());
        while hub.stopped.load(Ordering::SeqCst) {
            applied = applied.and(self.wait_for_others(hub));
            if hub.finishing.load(Ordering::Acquire)
                && !hub.inboxes[self.current].discarded.load(Ordering::Relaxed)
            {
                self.discard_blocks();
                hub.inboxes[self.current]
                    .discarded
                    .store(true, Ordering::Release);
            }
            self.host.relax();
        }
        hub.set_parked(self.current, false);
        self.statistics.sync_ticks += self.host.counter().wrapping_sub(started);
        self.observe_parallel_source(SourcePhase::ParkExit);
        applied
    }

    /// Serve the device access another PE forwarded, if any, with every
    /// other running PE stopped, and send that PE the CPU power changes it
    /// made before it runs again.
    fn serve_forwarded(&mut self, hub: &Arc<Hub>) -> Step {
        let Some((pe, access)) = hub.take_request() else {
            return Ok(());
        };
        let started = self.host.counter();
        hub.stop_others(self.current, pe, || self.host.relax());
        let now = self.now();
        let answer = match access {
            Forwarded::Read { physical, shape } => {
                Answer::Read(self.serial_read_canonical_as(pe, physical, shape, now))
            }
            Forwarded::Write {
                physical,
                bytes,
                value,
            } => Answer::Write(self.serial_write_canonical_as(pe, physical, bytes, value, now)),
        };
        let forwarded = self.forward_power();
        hub.restart_others();
        hub.answer(answer);
        self.statistics.sync_ticks += self.host.counter().wrapping_sub(started);
        forwarded
    }

    /// Forward a read only canonical memory serves to the home machine.
    pub(super) fn forward_read(
        &mut self,
        physical: u64,
        shape: DeviceReadShape,
    ) -> Result<ConsumedPair, DeviceError> {
        match self.forward(Forwarded::Read { physical, shape })? {
            Answer::Read(read) => read,
            Answer::Write(_) => unreachable!("a read's answer"),
        }
    }

    /// Forward a write only canonical memory serves to the home machine.
    pub(super) fn forward_write(
        &mut self,
        physical: u64,
        bytes: u8,
        value: u64,
    ) -> Result<(), DeviceError> {
        match self.forward(Forwarded::Write {
            physical,
            bytes,
            value,
        })? {
            Answer::Write(write) => write,
            Answer::Read(_) => unreachable!("a write's answer"),
        }
    }

    /// Wait, applying this PE's mail, until the home machine answered
    /// `access`; one PE's access at a time.
    fn forward(&mut self, access: Forwarded) -> Result<Answer, DeviceError> {
        let hub = Arc::clone(self.parallel.as_ref().expect("forwarded in parallel"));
        let started = self.host.counter();
        let timed = self.timing_start();
        self.observe_parallel_source(SourcePhase::ForwardEnter);
        hub.set_parked(self.current, true);
        let mut applied = Ok(());
        while hub
            .forwarding
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            applied = applied.and(self.wait_for_others(&hub));
            self.host.relax();
        }
        hub.request(self.current, access);
        let answer = loop {
            if let Some(answer) = hub.answered(self.current) {
                break answer;
            }
            applied = applied.and(self.wait_for_others(&hub));
            self.host.relax();
        };
        hub.forwarding.store(false, Ordering::Release);
        hub.set_parked(self.current, false);
        self.statistics.forwards += 1;
        self.statistics.sync_ticks += self.host.counter().wrapping_sub(started);
        if self.step_timing.is_some() {
            let ticks = self.timing_since(timed);
            let devices = &mut self.system_exits.devices;
            devices.forwarded += 1;
            devices.forwarded_ticks += ticks;
        }
        self.observe_parallel_source(SourcePhase::ForwardExit);
        // The mail applied meanwhile settles first: its refusal is this
        // access's.
        applied.map_err(|stop| {
            DeviceError::TransferRefused(refuse!("while forwarded: {stop}", ?stop))
        })?;
        Ok(answer)
    }
}

/// Whether a TLBI reaches other PEs.
pub(super) fn broadcasts(scope: &tlbi::Scope) -> bool {
    scope.broadcast != Broadcast::NSH
}

/// What a call to the board's devices is for, which [`SharedDevices::waits`]
/// counts the contended ones by.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Call {
    /// A device register access.
    Access,
    /// The devices' DMA transfers, which the home machine runs.
    Transfer,
    /// The lines the devices drive into a PE.
    Lines,
    /// The devices' next deadline.
    Deadline,
    /// A System register the devices serve.
    Register,
    /// The home machine's questions: DMA due, a reset, the host's service.
    Home,
    Other,
}

impl Call {
    pub const ALL: [Self; 7] = [
        Self::Access,
        Self::Transfer,
        Self::Lines,
        Self::Deadline,
        Self::Register,
        Self::Home,
        Self::Other,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Access => "access",
            Self::Transfer => "transfer",
            Self::Lines => "lines",
            Self::Deadline => "deadline",
            Self::Register => "register",
            Self::Home => "home",
            Self::Other => "other",
        }
    }
}

#[cfg(test)]
mod tests;
