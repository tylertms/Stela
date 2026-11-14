//! The dispatch loop: fetch translation, block cache, native execution and
//! every exit back from native code.
mod block_inputs;
mod boundary;
mod mirrors;
mod reports;
use boundary::*;
pub use boundary::{
    CodePartitions, CodeWindow, ConsumedPair, DeviceError, Devices, Host, LocalRegisters,
    LockedMemory, NativeView, NoDevices, PassiveRegisters, PlainRead, ScalarRead, Settled, Start,
    UnlockedDma, WriteSettled,
};
use compilation::{PENDING_EXITS, PendingExits};
use mirrors::*;
use reports::*;
pub use reports::{
    DeviceRegister, Discard, Event, ExitWordsWalk, Footprint, FootprintWalk, MAIL_WAITS, MailWait,
    Occupancy, PeState, Requirement, RequirementKind, Reset, Statistics, Stop, Surveyed,
    Suspension, TRACE_HITS, TraceHit, WATCH_HITS, WatchHit,
};
mod amx;
mod apple_maintenance;
mod budget;
mod caller;
mod chain_control;
mod churn;
mod clock;
mod code_cache;
mod code_slots;
mod compilation;
mod compression;
mod contexts;
mod data_access;
mod debugging;
mod degraded;
mod device_access;
mod device_read;
mod device_write;
mod exclusive;
mod fetch;
#[cfg(all(feature = "std", feature = "native-profile"))]
pub mod floating_profile;
mod guarded;
mod install_census;
pub mod installed_code;
mod interrupts;
mod known_keys;
#[cfg(all(feature = "std", feature = "native-profile"))]
pub mod metrics;
mod monitor;
mod native_failure;
mod native_timing;
mod numbering;
mod parallel;
mod pointer_authentication;
mod regime_state;
mod scheduler;
mod system_access;
mod timing;
mod transitions;
pub use board::{CodeReport, LockCounts, LockSite, SiteCounts};
pub use churn::ChurnCounts;
pub use install_census::InstallCauses;
pub use monitor::{MonitorEvent, MonitorKind, MonitorLocation};
pub use native_timing::{NativeInstructionSample, NativeTimingPhase, NativeTimingWork};
pub use parallel::{
    BroadcastCounts, BroadcastKind, BroadcastProfile, Call, Counts, GlobalSpan, GuestWatch, Hub,
    Kick, LockTimes, MaterializedSource, ParallelAdmissionError, Pending, SharedDevices,
    SourceObservation, SourcePhase,
};
pub use rebase::RebaseCounts;
pub use stela_contracts::dma::{DmaError, TransferFault};
mod pointer;
mod prepare;
mod rebase;
pub use device_read::{CanonicalDeviceMemory, ConsumedScalar, DeviceReadRequest, DeviceReadShape};
pub use device_write::{
    CanonicalDeviceWrite, CanonicalWriteMemory, ConsumedWrite, DeviceWriteRequest, DeviceWriteShape,
};
mod block_index;
mod board;
mod page_sets;
mod processors;
mod recovery;
mod scalable;
mod shared_memory;
pub mod step_timing;
mod stored;
mod system;
mod tags;
mod unprivileged;
mod watch;
pub use budget::RunBudget;

use crate::{pauth, scalable::ScalableState};
use alloc::string::ToString;
use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet},
    format,
    string::String,
    sync::Arc,
    vec,
    vec::Vec,
};
use core::fmt;
use core::mem::offset_of;
use core::num::NonZeroU64;
#[cfg(test)]
use core::sync::atomic::AtomicU64;
use core::sync::atomic::Ordering;
use pointer::{PointerInputs, PointerOperation, PointerResult, SERVICE, TAGS, strip_place};
#[cfg(test)]
pub(crate) use prepare::test_preparation;
pub use prepare::{Description, Preparation, Prepared, Refused, window};
/// Where system-mode blocks find the PAC service's cells
/// (`x86_64::pointer::inline`), as this engine compiles them.
pub const POINTER_CELLS: stela_compiler::x86_64::pointer::inline::Cells = pointer::CELLS;
pub use processors::publication;
pub use processors::{
    PeerState, PreparedProcessorPower, ProcessorBinding, ProcessorChange, ProcessorPowerError,
    ProcessorWrite, Processors, SettledProcessorWrite,
};
pub use recovery::{NativeFailure, NativeFailureReason, UnsettledChain};
pub use shared_memory::{
    HeldRamService, MAX_NATIVE_EPOCH_BODIES, NativeCohort, NativeEpochDeadline, NativeEpochEnd,
    NativeEpochStatistics, NativeRun, NativeRunIdentity, NativeRunObservation, NativeTransport,
    ReturnedRun, ServiceHost, ServiceTransport, SharedMemory, SharedMemoryFailure,
    SharedMemoryRefusal, dispatch_native_run, recover_native_run,
};
#[cfg(feature = "native-profile")]
pub use shared_memory::{NativeDataTlbStatistics, NativeSourceStatistics};
#[cfg(test)]
use stela_compiler::x86_64::chain::ChainControl;
use stela_compiler::x86_64::chain::ExitWord;
use stela_compiler::{Anchor, Anchors, Place};
use stela_compiler::{
    GuestAddress, GuestState, Lifter, TargetCpu,
    aarch64::{
        AddressMode, Decoder, FloatingPointOperation, MemoryTransfer, Operation, Register, Width,
        register_offset::IndexExtend,
        simd::fp8_conversion,
        system::{
            exception::{AuthenticationKey, Operation as ExceptionOperation},
            registers::{Direction, Encoding, RegisterAccess, decode as decode_register},
            synchronization::{Operation as Synchronization, WaitKind},
        },
    },
    block::{
        self, ExitKind, PointerAuthentication, PointerAuthenticationKey,
        PointerAuthenticationModifier, PointerAuthenticationOperation,
        scalar::{ScalarLoad, ScalarLoadAddress, ScalarStore},
        system::SystemOperation,
    },
    x86_64::{
        MemoryChecks, NativeSystem, cache_clean, chain, native_exclusive,
        pointer as x86_64_pointer, register_read, transition, zero_block,
    },
};
pub use stela_contracts::bus::Transition;
pub use stela_contracts::dma::Memory;
pub use stela_contracts::gate::{CallerFault, CallerMemory, GateCall, GateOutcome};
pub use stored::{EntryChecks, Offer, SharedTranslation, StoreEvent, Translations, entry_checks};
pub use system::FP_TRAPS_WRITABLE;
use {
    super::cpu::Cpu, super::cpu::Exception, super::cpu::HCR_AMO, super::cpu::HCR_API,
    super::cpu::HCR_FMO, super::cpu::HCR_HCD, super::cpu::HCR_IMO, super::cpu::HCR_TGE,
    super::cpu::HCR_TSC, super::cpu::HCR_VF, super::cpu::HCR_VI, super::cpu::HCR_VSE, super::ctrr,
    super::debug, super::exception::Class, super::exception::DAIF_A, super::exception::DAIF_D,
    super::exception::DAIF_F, super::exception::DAIF_I, super::exception::IllegalReturn,
    super::exception::Pstate, super::exception::SYNCHRONOUS_EXTERNAL_ABORT,
    super::exception::Vector, super::exception::data_abort_iss, super::hacr, super::identification,
    super::mmu, super::mmu::Access, super::mmu::Controls, super::mmu::Outcome as Translated,
    super::registers, super::registers::CpuProfile, super::registers::HCR_TIDCP,
    super::registers::Lines, super::registers::Outcome as RegisterOutcome,
    super::registers::Refusal, super::streaming, super::streaming::Denial,
    super::system::Register as SystemRegister, super::timer, stela_contracts::Chip,
    stela_contracts::Feature, stela_contracts::chip::AdmitTable, stela_contracts::chip::Check,
};

use block_inputs::{FETCH_PAGE, INSTRUCTION, MAX_BLOCK_BYTES, PROFILE, is_floating_point};
/// The bytes of a block's source window a compile reads and admits first:
/// most blocks end within them, and the rest compile from the whole window.
const FIRST_WINDOW_BYTES: usize = 32 * INSTRUCTION as usize;
const FETCH_CACHE: usize = 1024;
/// Bits in each half's filter of the leaves fetched through.
const FETCHED_LEAVES: usize = 1 << 21;
/// The set words of a half's filter of the leaves fetched through that a
/// clear zeroes one by one; past them it zeroes the whole filter.
const FETCHED_WORDS: usize = 4096;
/// The leaves of one size a TLBI is checked against one by one; one that
/// reaches more counts as reaching what was fetched.
const SCANNED_LEAVES: u64 = 1 << 16;
/// The level a fetch the host refuses (`Host::fetch_permitted`) reports its
/// permission fault at: the host refuses whole pages, a level 3 leaf's.
const HOST_REFUSED_FETCH_LEVEL: i8 = 3;
/// ID_AA64PFR0_EL1.CSV3, FEAT_CSV3's field.
const CSV3_SHIFT: u32 = 60;
const JUMP_CACHE: usize = 4096;
/// The most blocks one native run chains into, which bounds how late the
/// engine sees a timer or device deadline pass (tens of microseconds of
/// guest code; every System register write and device access exits
/// anyway). Each run that reaches it costs an exit to the engine, which
/// at 256 caused frequent exits in measured workloads.
const CHAIN_LIMIT: u64 = 4096;
/// The fewest blocks a machine releases for low memory: fewer hold too
/// little metadata to relieve it, and releasing them only compiles them
/// again (`Machine::memory_release_blocks`).
const MEMORY_RELEASE_BLOCKS: usize = 4096;
/// The distinct refusals of one device register and direction reported.
const DEVICE_REFUSALS_REPORTED: usize = 16;
/// ISS CV=1 and COND=0b1110, as reported for trapped AArch64 instructions.
const TRAPPED_AARCH64: u32 = (1 << 24) | (0b1110 << 20);

/// A WFI, WFE, WFIT or WFET a PE waits in.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Wait {
    kind: WaitKind,
    /// WFIT and WFET only.
    timeout: Option<Timeout>,
    /// A WFI in CYC_OVRD's power-down mode: the core is off, its state
    /// lost, and it wakes by leaving reset (`processors`).
    down: bool,
}

/// The local timeout of WFIT and WFET (Arm `LocalTimeoutEvent`): it expires
/// once the virtual count, the physical count minus `offset`, reaches
/// `limit`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Timeout {
    limit: u64,
    /// CNTVOFF_EL2 outside the host, 0 in it (Arm `VirtualCounterTimer`).
    offset: u64,
}

impl Timeout {
    fn expired(self, physical: u64) -> bool {
        physical.wrapping_sub(self.offset) >= self.limit
    }

    /// The physical count at which it expires, from a count `now` before it.
    fn deadline(self, now: u64) -> u64 {
        now.saturating_add(self.limit - now.wrapping_sub(self.offset))
    }
}
/// HFGITR_EL2 instruction traps (FEAT_FGT).
const HFGITR_ERET: u32 = 51;
const HFGITR_SVC_EL0: u32 = 52;
const HFGITR_SVC_EL1: u32 = 53;

/// The translation regime the host's shadow tables must reflect.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Regime {
    pub controls: Controls,
    pub requester: mmu::Requester,
    /// Pages the shadow tables must leave unmapped for watchpoints.
    pub watch: Watch,
    /// For the lower and upper half, whether the current EL's tag-checked
    /// accesses through untagged pointers are checked (`Cpu::tag_checks`):
    /// the shadow tables then leave that half's Tagged memory
    /// (`mmu::attributes::TAGGED`) unmapped, apart from the `watch.pass`
    /// pages, so that every access to it exits with
    /// [`FaultCause::TagChecked`]. A tagged pointer always fails the native
    /// range check.
    pub tag_checks: [bool; 2],
}

impl Regime {
    /// Whether the 4 KiB page at `page`, whose translation has the memory
    /// `attributes`, must stay unmapped for tag checks.
    pub fn tag_blocks(&self, page: u64, attributes: u8) -> bool {
        self.tag_checks[usize::from(page >> 55 & 1 != 0)]
            && attributes == mmu::attributes::TAGGED
            && !self.watch.pass.contains(&Some(page & !(FETCH_PAGE - 1)))
    }

    /// Whether the regime keeps any Tagged memory out of the shadow.
    pub fn tag_checked(&self) -> bool {
        self.tag_checks.contains(&true)
    }
}

/// The virtual address ranges enabled watchpoints can match: a page that
/// overlaps one stays out of the shadow tables, so every access to it exits
/// with [`FaultCause::Watched`], unless it is one of the `pass` pages the
/// engine lets one instruction through.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Watch {
    /// (first, last) addresses; unused entries are empty (first > last).
    pub ranges: [(u64, u64); WATCH_RANGES],
    pub pass: [Option<u64>; 2],
}

/// Two ranges per watchpoint: the matched bits repeat with either value of
/// the bits above DebugAddrTop.
pub const WATCH_RANGES: usize = 2 * debug::WATCHPOINTS as usize;

impl Default for Watch {
    fn default() -> Self {
        Self::from_ranges(&[])
    }
}

impl Watch {
    /// Whether any watch range intersects the inclusive address span.
    /// A one-instruction pass can still admit a page inside it.
    pub fn overlaps(&self, first: u64, last: u64) -> bool {
        first <= last
            && self
                .ranges
                .iter()
                .any(|&(start, end)| start <= end && start <= last && first <= end)
    }

    /// Whether the 4 KiB page at `page` must stay unmapped.
    pub fn blocks(&self, page: u64) -> bool {
        let last = page | (FETCH_PAGE - 1);
        !self.pass.contains(&Some(page & !(FETCH_PAGE - 1))) && self.overlaps(page, last)
    }

    fn from_ranges(ranges: &[(u64, u64)]) -> Self {
        let mut watch = Self {
            ranges: [(1, 0); WATCH_RANGES],
            pass: [None; 2],
        };
        for (slot, range) in watch.ranges.iter_mut().zip(ranges) {
            *slot = *range;
        }
        watch
    }
}

/// The guest context branch predictions must not cross (FEAT_CSV2): the
/// translation regime, its table bases (with the ASID one carries) and
/// VTTBR_EL2 (the VMID). A change of any of these is a guest context
/// switch, at which the engine has the host isolate its predictions
/// ([`Host::isolate_predictions`]). An exception or return that keeps
/// them, between EL0 and EL1 of one address space, is not one.
/// CONTEXTIDR_ELx is not part of it: Arm's hardware-described context is
/// the Security state, Exception level, ASID and VMID (and SCXTNUM_ELx,
/// which this machine holds without distinguishing contexts), and a
/// kernel that writes CONTEXTIDR at every thread switch within one address
/// space gets no isolation from the chip there either.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct PredictionContext {
    pub host: bool,
    pub ttbr0: u64,
    pub ttbr1: u64,
    pub vttbr: u64,
}

impl PredictionContext {
    fn of(cpu: &Cpu) -> Self {
        use crate::machine::cpu::TranslationRegime;
        let host = cpu.regime() == TranslationRegime::El20;
        let (ttbr0, ttbr1) = if host {
            (SystemRegister::TTBR0_EL2, SystemRegister::TTBR1_EL2)
        } else {
            (SystemRegister::TTBR0_EL1, SystemRegister::TTBR1_EL1)
        };
        Self {
            host,
            ttbr0: cpu.read(ttbr0),
            ttbr1: cpu.read(ttbr1),
            vttbr: cpu.read(SystemRegister::VTTBR_EL2),
        }
    }
}

/// Entry address of installed native code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NativeCode(pub usize);

/// Why native code stopped before its block exit.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum FaultCause {
    /// The guest's translation refused the access.
    Guest(mmu::Fault),
    /// The access translated to a physical address that is not RAM.
    Device { physical: u64 },
    /// The guest configured translation this engine does not implement.
    Translation(mmu::Unsupported),
    /// An x86 #GP: the address is not canonical for x86 (tags, wild pointers).
    NonCanonical,
    /// An x86 #UD from a native check: SP alignment, an Alignment fault, or
    /// an address outside the guest's ranges.
    InvalidOpcode,
    /// The access reached a page [`Watch`] keeps unmapped.
    Watched,
    /// The access reached Tagged memory of a half whose accesses are
    /// tag checked ([`Regime::tag_checks`]).
    TagChecked,
}

/// Distinguish an exception image from a deliberate translation-helper
/// return. The helper token cannot invent hardware vector permission.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NativeFaultOrigin {
    Exception,
    TranslatedHelper,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct NativeFault {
    pub origin: NativeFaultOrigin,
    /// Byte offset of the faulting instruction in the block's native code.
    pub offset: usize,
    pub address: u64,
    pub write: bool,
    pub cause: FaultCause,
    /// X86 exception vector, or zero for a deliberate helper return.
    pub trap: u16,
    /// X86 exception error code; instruction-fetch or host page-table/
    /// protection/stack failures cannot authorize a guest data-abort projection.
    pub error_code: u64,
    /// Initialized words captured from a checked invocation stack, unavailable
    /// until the platform supplies that owned lease and exact plan.
    pub spills: stela_compiler::block::HostSpillSnapshot,
    /// Host GPRs in architectural encoding order at the faulting instruction.
    /// A platform supplies the extended bank only after capturing it
    /// or checking its architectural initialization state. Recovery refuses
    /// a missing bank before publishing any recovered guest register.
    pub registers: stela_compiler::block::HostGprSnapshot,
}

// Keep the complete interrupt capture inline: native fault return must not
// allocate or borrow storage that the next entry can overwrite.
#[allow(
    clippy::large_enum_variant,
    reason = "the owned native interrupt capture is returned without allocation or a borrowed frame lifetime"
)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum NativeExit {
    Exit { next_pc: u64, kind: u64 },
    Fault(NativeFault),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InstallError {
    /// The code arena is full; discarding every block frees it.
    Full,
}

struct Block {
    pc: u64,
    physical: u64,
    code: NativeCode,
    /// The guest instructions it retires.
    instructions: u16,
    variant: VariantId,
    /// Direct exits to static successors, its own and its PAC thunk's, at
    /// most [`board::MAX_SUCCESSORS`]: which are chained (into a block of
    /// its page, which an eviction of that block undoes) its record says.
    successors: Box<[Successor]>,
    /// The host address of the exit to the target in RAX that its RET, BR
    /// or BLR form (or that form's PAC thunk) ends with; its record says
    /// once a lookup dispatcher serves it.
    dynamic_site: Option<NonZeroU64>,
    /// What its first instruction admits on a guarded page.
    landing: fetch::Landing,
    /// The word of its last instruction, where it exits to the engine.
    terminal: u32,
    /// FNV-1a of its admitted guest bytes ([`Machine::admit`]).
    source_hash: u64,
    /// The PE that compiled it, for [`StaleCode`].
    installer: u8,
    /// The board's arena part holding its code, or [`board::NO_PART`]:
    /// what a fetch counts ([`Machine::entered`]) without a division.
    part: u8,
    /// A device site's stub ([`Machine::install_device_stub`]): its System
    /// exit is the site's access, on whichever PE runs it, whether or not
    /// that PE learnt the site.
    device_stub: bool,
}

// The board keeps one per block, millions in all.
const _: () = assert!(size_of::<Block>() <= 72);

impl Block {
    /// One source-map entry per four-byte guest instruction.
    fn source_bytes(&self) -> usize {
        usize::from(self.instructions) * INSTRUCTION as usize
    }

    /// The PC of its last instruction.
    fn terminal_pc(&self) -> u64 {
        self.pc + self.source_bytes() as u64 - INSTRUCTION
    }
}

/// The bytes a block of `variant` at `pc` may be compiled from: one
/// instruction for a single-instruction block, else [`prepare::window`].
fn source_window(pc: u64, variant: Variant) -> usize {
    if variant.single {
        INSTRUCTION as usize
    } else {
        prepare::window(pc)
    }
}

/// The exit kind of a branch to a register's value (RET, BR, BLR and their
/// PAC forms RETA*, BRA*, BLRA*), and the BTYPE it sets whatever page it is
/// in: none for a BR form through other than X16 or X17, which sets 0b11 in
/// a guarded page and 0b01 elsewhere (Arm BTypeNext).
fn dynamic_branch(word: u32) -> Option<(ExitKind, Option<u8>)> {
    // Bits 22:21 select BR, BLR or RET; bit 11 a PAC form, whose bit 24
    // selects the register modifier.
    if word & 0xfe9f_f000 != 0xd61f_0000 {
        return None;
    }
    let register = (word >> 5) & 31;
    Some(match (word >> 21) & 3 {
        0 => (
            ExitKind::Indirect,
            matches!(register, 16 | 17).then_some(0b01),
        ),
        1 => (ExitKind::Call, Some(0b10)),
        2 => (ExitKind::Return, Some(0)),
        _ => return None,
    })
}

/// Lengths of successor lists whose boxes evicted blocks leave for new
/// ones, and how many of each length are kept: about a part's blocks.
const SUCCESSOR_BOXES: usize = 8;
const KEPT_SUCCESSOR_BOXES: usize = 2048;

/// The compiled blocks one [`Machine::stale_code`] call checks: about a
/// millisecond of hashing.
pub const STALE_CHECKS: usize = 1024;

/// FNV-1a over a block's guest bytes.
fn source_hash(bytes: &[u8]) -> u64 {
    bytes.iter().fold(0xcbf2_9ce4_8422_2325, |hash, &byte| {
        (hash ^ u64::from(byte)).wrapping_mul(0x0100_0000_01b3)
    })
}

/// Compiled blocks whose guest bytes changed without a code invalidation,
/// among those one [`Machine::stale_code`] call checked.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StaleCode {
    pub blocks: usize,
    pub stale: usize,
    /// Blocks whose bytes in RAM differ from their admitted bytes alone:
    /// words the compile admitted as UDF, no change of code.
    pub admitted: usize,
    /// The first stale block's PC and physical address.
    pub first: Option<(u64, u64)>,
    /// Where the first stale block came from.
    pub origin: Option<StaleOrigin>,
}

/// Where a stale block came from: the PE that compiled it, its part of
/// the board's arena, and whether the hub notes its page as one an IC
/// IVAU must reach every PE for ([`parallel::Hub::has_code`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
pub struct StaleOrigin {
    pub pe: u8,
    pub part: Option<u32>,
    pub noted: Option<bool>,
}

/// A direct exit in native code to a static successor: its host address,
/// the successor's PC and the exit's word, its kind's byte and the
/// instructions it skips apart, as linked ones stay in their lists.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct Successor {
    site: u64,
    target: GuestAddress,
    kind: u8,
    skipped: u16,
    /// The bytes before `site` kept for a link ([`chain::link_slot`]).
    slot: u8,
}

const _: () = assert!(size_of::<Successor>() == 24);

impl Successor {
    fn new(site: u64, target: GuestAddress, word: ExitWord, slot: usize) -> Self {
        Self {
            site,
            target,
            kind: word.kind as u8,
            skipped: word.skipped,
            slot: u8::try_from(slot).expect("a link slot is a few bytes"),
        }
    }

    fn word(&self) -> ExitWord {
        ExitWord {
            kind: ExitKind::try_from(u64::from(self.kind)).expect("a successor's own kind"),
            skipped: self.skipped,
        }
    }
}

#[derive(Clone, Copy)]
struct FetchEntry {
    page: u64,
    physical: u64,
    /// FEAT_BTI's guarded page (Arm `InGuardedPage`).
    guarded: bool,
    /// The walk ended in a global leaf (nG=0).
    global: bool,
    el: u8,
    epoch: FetchEpoch,
}

/// What a half's cached fetches and lookup contexts last as long as: the
/// translation generation, and the table generation of a half that TTBR0
/// selects ([`Machine::fetch_epoch`]).
type FetchEpoch = (u64, u64);

#[derive(Clone, Copy)]
struct JumpEntry {
    pc: u64,
    physical: u64,
    variant: Variant,
    generation: u64,
    block: usize,
    /// The block's slot incarnation ([`board::Record::incarnation`]).
    incarnation: u32,
}

/// A half's recent context number, with the EL, variant and fetch epoch it
/// was found for.
#[derive(Debug, Clone, Copy)]
struct Recent {
    el: u8,
    variant: Variant,
    epoch: FetchEpoch,
    number: u64,
}

/// `T` on cache lines of its own: a shared word that PEs read on every
/// step or fetch, kept apart from words others write, or a word one PE
/// writes kept apart from words the others read.
#[repr(align(64))]
#[derive(Debug, Default)]
pub(super) struct Padded<T>(pub(super) T);

impl<T> core::ops::Deref for Padded<T> {
    type Target = T;
    fn deref(&self) -> &T {
        &self.0
    }
}

/// The recent contexts one half finds without comparing controls: enough
/// for the ELs and GXF states a kernel's exceptions alternate between.
const RECENT: usize = 4;
/// `Machine::lower_globals` with no such fetch.
const NO_GLOBALS: (u64, u64) = (u64::MAX, 0);

/// The translation inputs besides TTBR0 and the SPRR permissions that a
/// PE's contexts name at once ([`Machine::context_frames`]).
const CONTEXT_FRAMES: usize = 64;

/// The fills a PE remembers for [`Machine::refill_after`]: more than the
/// pages a kernel and the processes it switches between take between two
/// TLBIs of everything (about 300 kernel pages, then each process's).
const FILL_LOG: usize = 4096;

/// Whether the context `space` translates `asid`.
/// ASIDs compare by the eight bits every TCR.AS gives them.
fn same_asid(space: &contexts::Space, asid: u16) -> bool {
    (space.asid ^ asid) & 0xff == 0
}

/// A [`Variant`] in [`Machine::variants`]: a block's and its index keys'
/// compact name for what it was compiled for.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
struct VariantId(u32);

/// What a block is compiled for besides its PC and PA.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash)]
struct Variant {
    /// FP/SIMD enabled: otherwise the block ends before the first FP/SIMD
    /// instruction.
    fp: bool,
    /// SP and data alignment checks (SCTLR_ELx.SA or SA0, A and nAA).
    checks: MemoryChecks,
    /// One instruction only, for breakpoints and software step.
    single: bool,
    /// PE has SME, and PSTATE.SM: the block ends before the first
    /// instruction an SVE or SME access check (or, in streaming mode, any
    /// access check) applies to.
    scalable: Option<bool>,
    /// The System instructions that complete natively at the end of a
    /// block: DC ZVA while it zeroes without a trap
    /// (`system::native_zero_block`), and the DC clean operations while
    /// they only check the translation of Xt (`system::native_cache_clean`).
    system: NativeSystem,
    /// FEAT_BTI: the block's page is guarded (Arm `InGuardedPage`), which
    /// fixes the BTYPE its BR forms through other than X16 and X17 set.
    /// Contexts leave it out, as it belongs to a page, not to the PE.
    guarded_page: bool,
    /// PSTATE.DIT on a host that gives the timing guarantee: the block is
    /// compiled for the host's data-independent-timing tier
    /// ([`Machine::timing_target`]).
    timing: bool,
    /// The block's page needs the translated variant: every access checks
    /// its address and calls the host's translation helper outside the
    /// native ranges ([`Machine::run_translated`]). Like `guarded_page`, it
    /// belongs to a page, not to the PE.
    translated: bool,
}

/// A load or store addressed through Rn (bits 9:5) that names SP; literal
/// loads have no base register.
fn stack_based(word: u32) -> bool {
    word & 0x0a00_0000 == 0x0800_0000 && word & 0x3b00_0000 != 0x1800_0000 && (word >> 5) & 31 == 31
}

pub struct Machine<H: Host, D: Devices> {
    /// The running PE.
    pub cpu: Cpu,
    pub host: H,
    pub devices: D,
    pub profile: CpuProfile,
    pub statistics: Statistics,
    /// `--profile`'s phase timing, while it is on.
    step_timing: Option<step_timing::StepTiming>,
    /// Exits to the engine from compiled code (System, FP, PAC and AMX), by
    /// the instruction word that took them, except those the live blocks
    /// count at their last instructions (see [`Self::exit_words`]).
    exit_words: BTreeMap<u32, u64>,
    /// Exits at blocks' last instructions not yet added to their records'
    /// shared counts, by slot ([`Self::exit_word`]): a block every PE exits
    /// (a syscall stub, the kernel's ERET) would have each exit write its
    /// record's line.
    exit_pending: [PendingExits; PENDING_EXITS],
    /// Fetches of blocks of each ring part not yet added to the ring's
    /// shared count of the part, and their sum ([`Self::entered`]).
    entered_pending: Vec<u32>,
    entered_total: u32,
    /// Device accesses by physical address: the registers the guest reads
    /// and writes most and, under step timing, the costliest
    /// ([`Self::device_registers`]).
    device_registers: BTreeMap<u64, DeviceRegister>,
    /// The board's fast-IPI table while this PE's native IPI writes use it
    /// (`Mirrors::fast_ipi`), which keeps its words alive.
    fast_ipi: Option<Arc<crate::machine::fast_ipi::Table>>,
    /// The generation native device reads' keys hold
    /// ([`mirrors::DEVICE_READ_KEY`]), and the model epochs of the reads
    /// in `Mirrors::device_reads`, which those keep alive.
    device_read_generation: u64,
    device_read_epochs: Vec<Option<Arc<core::sync::atomic::AtomicU64>>>,
    /// The register the native fault being handled reached, which its
    /// exit's ticks are charged to ([`Self::timing_exit`]).
    timed_device: Option<u64>,
    /// Under step timing, the device write being handled: the ticks of its
    /// access call, its settle (the DMA after it) and its gate call
    /// ([`step_timing::DeviceWrites`]).
    timed_write: Option<[u64; step_timing::WRITE_PARTS]>,
    /// System exits by instruction word, with their handling time under step
    /// timing ([`Self::system_words`]).
    system_words: BTreeMap<u32, DeviceRegister>,
    /// The word of the system exit being handled.
    timed_word: Option<u32>,
    /// The System exit being handled closed every mirror cell.
    timed_closed: bool,
    /// System exits by key since they were last taken, under step timing
    /// ([`Self::take_system_exits`]).
    system_exits: Box<step_timing::SystemExits>,
    /// Hot-block samples not yet taken, under step timing
    /// ([`step_timing::HotBlocks`], [`Self::take_hot_samples`]), the runs
    /// until the next sampled one and the generator of sampled budgets.
    hot_samples: Vec<(u64, step_timing::Level)>,
    /// The sampled runs among those whose last transfer was their block's
    /// RET, BLR or BR form's ([`step_timing::HotBlocks::dynamic`]).
    dynamic_samples: Vec<step_timing::DynamicSample>,
    /// Native runs since the engine started, and the stamped cell the next
    /// promotion step looks at ([`scheduler`]'s `promote_hot_exits`).
    promotion_runs: u32,
    promotion_cursor: u32,
    hot_countdown: u32,
    hot_seed: u64,
    #[cfg(all(feature = "std", feature = "native-profile"))]
    floating_profile: Option<alloc::boxed::Box<floating_profile::Sampler>>,
    #[cfg(all(feature = "std", feature = "native-profile"))]
    preparation_metrics: Option<metrics::PreparationMetrics>,
    target: TargetCpu,
    /// The blocks, block index and block table this machine shares with
    /// its board's other machines, or keeps alone ([`board::Code`]).
    code: Arc<board::Code>,
    /// The generation of the board's ring the next install goes to
    /// ([`board::YOUNG`] unless its caller says otherwise; each install
    /// takes it back to young).
    install_stream: usize,
    /// The ring places the block install under way took, held until it
    /// publishes ([`Self::install_native`], [`Self::release_places`]).
    held_places: Vec<board::Place>,
    /// A grace of no PE: what a machine that keeps its code alone releases
    /// slots with ([`Self::grace`]).
    alone: board::Grace,
    /// The hub whose PEs share `code`, when it is the board's.
    code_hub: Option<Arc<parallel::Hub>>,
    /// The blocks this machine held when it last reclaimed code for memory,
    /// while memory has stayed low since (`reclaim_for_memory`).
    reclaimed_at: Option<usize>,
    /// The fewest live blocks it releases for low memory
    /// ([`MEMORY_RELEASE_BLOCKS`]).
    memory_release_blocks: usize,
    recovery_scratch: Vec<block::FaultRecovery>,
    /// The buffers a stored entry is relocated into for its install.
    relocated: crate::translations::Relocated,
    /// The list a block's successors are gathered in before it is boxed,
    /// and the boxes evicted blocks left, by length less one, which blocks
    /// of as many successors take ([`Machine::successor_box`]).
    successor_scratch: Vec<Successor>,
    successor_boxes: [Vec<Box<[Successor]>>; SUCCESSOR_BOXES],
    /// The slots of the board's known page keys ([`known_keys`]), sized
    /// by the store.
    known_slots: usize,
    /// WKDMC and WKDMD's page and data buffers, made at the first.
    wkdm_buffers: Option<Box<compression::Buffers>>,
    /// Why blocks were installed ([`install_census`]); the census exists
    /// while step timing is on.
    install_causes: InstallCauses,
    census: Option<Box<install_census::Census>>,
    native_failure: Option<recovery::CapturedFailure>,
    /// Every variant a block was compiled for, once: what a [`VariantId`]
    /// names. Shared by the board's machines ([`numbering`]).
    variants: Arc<numbering::Registry<Variant>>,
    /// The last block [`Machine::stale_code`] checked, where its sweep goes
    /// on; `None` to start over.
    stale_cursor: Option<block_index::Cursor>,
    /// The fetch pages read for the translation store, by physical page,
    /// until their code is invalidated or they are the least recently used
    /// ([`stored`]), and the count of their uses so far.
    pages: BTreeMap<u64, stored::StoredPage>,
    /// The most `pages` keeps ([`Self::share_kept_pages`]).
    kept_pages: usize,
    /// The buffers of pages let go, which the next pages read take: every
    /// PE reads pages all the time, and each buffer taken from the heap and
    /// given back is two turns at the heap's lock, which they all share.
    spare_pages: Vec<Box<[u8; crate::translations::PAGE_BYTES]>>,
    #[cfg(feature = "native-profile")]
    stored_pages_peak: usize,
    page_uses: u64,
    /// Entry contexts by variant and address class.
    stored_contexts: BTreeMap<(Variant, u64), crate::translations::Digest>,
    /// How each variant's blocks are compiled ([`prepare`]).
    preparations: BTreeMap<Variant, Arc<Preparation>>,
    fetch: Vec<Option<FetchEntry>>,
    jump: Vec<Option<JumpEntry>>,
    /// Advances when cached fetch translations become stale.
    translation_generation: u64,
    /// TTBR0 writes: the kernel's fetches outlive a process switch.
    table_generation: u64,
    /// The installed lookup dispatchers, by BTYPE.
    lookup_dispatchers: [Option<NativeCode>; 4],
    /// Advances when installed code is discarded or invalidated: the jump
    /// cache's entries name it.
    code_generation: u64,
    /// The block key of each page a fetch translated, by context: the ITLB
    /// `ChainControl::itlb` names, which lookup dispatchers translate
    /// their targets through.
    itlb: chain::LookupTable,
    /// The ITLB entries of live contexts that fills took the place of and
    /// that TLBIs by VA dropped, by page and salt, each `(page | cause,
    /// context)` ([`ItlbMiss`]): what tells a later fill of the page why
    /// the ITLB lacked it. Allocated at the first drop.
    itlb_ghosts: Vec<(u64, u64)>,
    /// The last fetch found its block through a block table entry that
    /// admits the PE's nonzero BTYPE: the branch target check before the
    /// block runs passes ([`Self::fetch_block`]).
    landing_admitted: bool,
    /// Each half's lookup contexts ([`contexts::Space`]) since its
    /// translations last changed in a way no control value shows, by key,
    /// up to a bound, least recently used dropped first.
    spaces: [contexts::Contexts; 2],
    /// The translation inputs of contexts besides TTBR0 and the SPRR
    /// permissions ([`contexts::Key::frame`]): a few values, which change
    /// with SCTLR, TCR, MAIR, the regime and the like.
    context_frames: Vec<(mmu::Controls, u64)>,
    /// Each half's recent contexts, most recent first.
    recent: [[Option<Recent>; RECENT]; 2],
    /// The leaf sizes a fetch of each half translated through since its
    /// contexts were last forgotten, a bit per power of two, and whether one
    /// was non-global: what a TLBI by VA or ASID reaches of its lookup entries.
    fetched_sizes: [u64; 2],
    private_fetches: [bool; 2],
    /// The first and last addresses of the lower half fetched through a
    /// global page since its contexts were last forgotten, or [`NO_GLOBALS`].
    lower_globals: (u64, u64),
    /// A filter of the leaves each half fetched through since its contexts
    /// were last forgotten: a TLBI of any other leaf reaches no lookup entry
    /// and no cached fetch, as most by far do not. Each half allocates
    /// its filter at its first recorded fetch.
    fetched_leaves: [Vec<u64>; 2],
    /// The words of each half's `fetched_leaves` that a fetch set since it
    /// was last cleared, up to [`FETCHED_WORDS`], or None past that: a
    /// clear zeroes these alone instead of the whole filter, which a
    /// TLBI VMALLE1 would otherwise write at every address space switch.
    fetched_words: [Option<Vec<u32>>; 2],
    /// The registers of the stored cells, in the order of the cells, which
    /// the board's machines share ([`numbering`]).
    stored_reads: Arc<numbering::Registry<Encoding>>,
    /// The IMPLEMENTATION DEFINED registers with a cell of their own for
    /// writes ([`STORED_WRITES`]), shared alike.
    stored_writes: Arc<numbering::Registry<Encoding>>,
    /// The TLBI being applied came in this PE's mail, from another PE
    /// ([`ItlbSource::AllReceived`]).
    applying_mail: bool,
    /// The page this PE posted an IC IVAU of since its last DSB.
    code_posted: Option<u64>,
    /// The storage the last generated read returned, if it was a plain one.
    plain_read: Option<(SystemRegister, u64)>,
    /// The next context number; numbers are never reused.
    next_context: u64,
    /// What owns each lookup salt ([`Self::context_salt`]), and when a
    /// context of it was last used: a new owner takes the least recently
    /// used salt.
    salt_owners: [u64; chain::LOOKUP_SALTS],
    salt_uses: [u64; chain::LOOKUP_SALTS],
    salt_clock: u64,
    /// A filter of the pages the ITLB took, by page and address space,
    /// whatever their context: what tells a cold fill of a page seen before
    /// (a refill after its context or entry went unnoticed) from a first
    /// one ([`ItlbSource::ColdSeen`]). Allocated at the first fill. And
    /// the bits it has set.
    itlb_seen: Vec<u64>,
    itlb_seen_bits: usize,
    /// The last [`FILL_LOG`] pages the ITLB took, each with its context,
    /// in a ring ([`Self::refill_after`]), and the next place.
    fill_log: Vec<(u64, u64)>,
    fill_next: usize,
    /// The pages forgotten contexts had taken last, by half and key: a
    /// context made anew for the key walks them again at once
    /// ([`Self::refill_pages`]).
    pending_refills: BTreeMap<(usize, contexts::Key), Vec<u64>>,
    /// The pages the contexts just made walk again before the fetch goes on.
    refills: Vec<u64>,
    /// The pages TLBIs by VA dropped of each live context, by its number:
    /// the next fetch in the context walks them again
    /// ([`Self::refill_dropped`]).
    dropped_refills: BTreeMap<u64, Vec<u64>>,
    /// The host tick at which the first native run that returned with
    /// mail waiting returned, 0 for none since the last read
    /// ([`Statistics::mail_waits`]).
    mail_returned: u64,
    /// Identification registers the CPU profile has no value for.
    profile_missing: Vec<SystemRegister>,
    /// Accessors by encoding.
    lookup: super::system::Lookup,
    /// The earliest physical count at which a timer signals FIQ; `None` when
    /// a System register write may have changed it.
    timers: Option<Option<u64>>,
    /// CNTHV_CVAL_EL2 and CNTHV_CTL_EL2 as [`Self::timers`] was computed
    /// from: native timer writes store there
    /// (`x86_64::register_read::timer_thunk`), so a change forgets it.
    timer_registers: [u64; 2],
    /// The EL FP/SIMD traps to, if any; `None` when a System register write
    /// or an EL change may have changed it.
    floating_point: Option<Option<u8>>,
    /// The memory checks of the current regime's SCTLR_ELx, while valid.
    memory_checks: Option<MemoryChecks>,
    /// [`Variant::system`] for the current EL, while valid.
    native_system: Option<NativeSystem>,
    /// Whether blocks that branch back to their own start run their
    /// iterations inside themselves (`NativeSystem::self_loops`): on, but
    /// for tests that compare both forms.
    self_loops: bool,
    /// The physical count up to which the event streams have set the event
    /// register; `None` before the first sample.
    event_sample: Option<u64>,
    /// What self-hosted debug needs of each instruction, while valid.
    debug: Option<DebugMode>,
    /// The device lines of a PE, the count before which no device changes
    /// state by itself (`Devices::deadline`) and that PE, while valid: a
    /// device or board register access, a System register write and the
    /// host between runs can change them, and clear it; an exception
    /// changes the PE alone, which the devices do not see.
    /// A PE switch keeps it, as it changes no device: another PE's lines,
    /// the running one's or a waiting one's, are then its latched pins
    /// (`Devices::pins`).
    device_lines: Option<(Lines, u64, usize)>,
    /// The devices' [`Devices::generation`] right after the call that gave
    /// `device_lines`' deadline.
    device_generation: Option<u64>,
    /// The PAC context, the memo of results and the key tag words, which
    /// native code reads and completes misses through (`pointer`).
    pointer: alloc::boxed::Box<pointer::PointerService>,
    /// The [`MIRRORED`] registers for native MRS reads
    /// (`x86_64::register_read`): each readable once an MRS exit completes
    /// as a plain read, until derived state may change.
    mirrors: alloc::boxed::Box<Mirrors>,
    /// The chip's `Chip::admit`, indexed.
    admit_table: Arc<AdmitTable>,
    system_decode: system_access::DecodeCache,
    /// The watched pages the host keeps unmapped.
    watch: Watch,
    /// The context the host last isolated predictions for.
    prediction_context: Option<PredictionContext>,
    /// The regime the host was last given ([`Self::sync_host_regime`]).
    synced_regime: Option<Regime>,
    /// The number of the running PE.
    current: usize,
    /// The WFI, WFE, WFIT or WFET the running PE waits in, until it wakes.
    waiting: Option<Wait>,
    /// Every PE by number; the running one's state is `cpu`.
    peers: Vec<processors::Peer>,
    /// Explicitly checked kept core/cluster clock and quiesce inputs.
    processor_power: Processors,
    /// Native-worker publication inventory; absent in the serial lane.
    native_publication: Option<processors::publication::WorkerPublications>,
    /// Identity of this Machine's persistent checked PE memory/event owners.
    native_memory_identity: Arc<shared_memory::ExecutionDomain>,
    /// Every assigned worker, held while the BSP executes all canonical PEs
    /// in turn through chained native blocks (`run_serialized`).
    serial_hold: Option<processors::publication::StoppedWorkers>,
    /// The other PEs' machines, when each PE runs on its own host core
    /// (`parallel`), and the count of device line changes last seen.
    parallel: Option<Arc<parallel::Hub>>,
    /// The unique admitted hub/PE epoch. The chain owner also keeps this
    /// claim, so field drop order cannot release it before control use ends.
    attached: Option<Arc<parallel::Attached>>,
    /// CPU power changes for the other PEs' machines, not yet sent.
    forwarded_power: Vec<ProcessorChange>,
    /// Instruction-cache maintenance other PEs broadcast while this one
    /// waited mid-instruction, applied at its next mail read.
    deferred_code: Vec<Option<u64>>,
    /// The latest serial of the maintenance this PE posted to each other PE
    /// and has not yet seen applied, which its next DSB waits for.
    posted: Vec<(usize, u64)>,
    /// The emptied buffer of the last mail this PE read, which the next read
    /// leaves in its inbox so neither side allocates per mail.
    mail_spare: Vec<parallel::Maintenance>,
    /// The direct exit within its page the last native run left by, when
    /// its target had no block to link to: the block, the target and the
    /// exit's word. The next fetch, if it is the target's, links it
    /// ([`Self::link_direct_exit`]), so the exit costs no second return to
    /// the engine and its target no lookup entry: no lookup reaches for a
    /// block that only linked exits enter. Any other fetch, and any discard
    /// of the block, forgets it.
    /// The block's slot incarnation is kept with it.
    direct_exit: Option<(usize, u32, u64, ExitWord)>,
    /// The PC whose fetch compiles its block now and takes its block table
    /// entry in the install's hold of the board's lock, not in a hold of
    /// its own after ([`Self::remember_target`]); none outside such a
    /// fetch.
    publishing_target: Option<u64>,
    /// Maintenance for the other PEs that this PE's next DSB sends
    /// ([`Self::send_outgoing`]).
    outgoing: Vec<parallel::Maintenance>,
    lines_epoch: u64,
    pending_device_read: Option<device_read::Pending>,
    pending_device_write: Option<device_write::Pending>,
    pending_device_tail: alloc::collections::VecDeque<shared_memory::RecoveredDeviceAccess>,
    /// Host breakpoints, ascending (`set_breakpoints`).
    breakpoints: Vec<u64>,
    /// Host trace points, ascending (`set_traces`), and the hits not yet
    /// taken (`take_trace_hits`).
    traces: Vec<u64>,
    trace_hits: Vec<TraceHit>,
    /// Emit raw guarded boundary events through Host::report, off by default.
    monitor_tracing: bool,
    /// Host watch ranges of physical device addresses, first and last
    /// byte (`set_watches`), and the hits not yet taken (`take_watch_hits`).
    watches: Vec<(u64, u64)>,
    watch_hits: Vec<WatchHit>,
    /// Every degraded operation met so far, each the first time, and those
    /// not yet taken (`take_unmodeled`); device accesses are keyed by
    /// register, direction and refusal in `degraded_devices`.
    unmodeled_met: Vec<RequirementKind>,
    unmodeled: Vec<Requirement>,
    /// The block count at which the running PE's time slice ends.
    slice_end: u64,
    /// The system counter value at which it ends, whichever comes first.
    slice_deadline: u64,
    /// The host counts a waiting PE looks for a wake before it halts
    /// ([`Self::poll_for_wake`]).
    idle_poll: u64,
    /// The system counter the PEs observe (`clock`), where native count
    /// reads find it.
    clock: alloc::boxed::Box<clock::Clock>,
    /// End waits of every PE by leaping the counter (`clock::Clock::skip_to`).
    idle_skip: bool,
    /// The longest the waits of every PE in one `run` last in all, in
    /// host counter ticks, before it returns [`Stop::Waiting`]
    /// (`set_wait_limit`).
    wait_limit: Option<u64>,
    /// Host counter ticks the PEs have waited on the host in this `run`.
    waited: u64,
    /// The instruction just completed wrote PSTATE.PACM (PACM, or an
    /// exception return), so it does not lapse.
    pacm_written: bool,
    /// The engine-path transition whose record the next native entry at
    /// its target makes (`transitions`).
    transition_note: Option<transitions::Note>,
    /// The instruction the engine is completing, already counted as retired
    /// (`Machine::completing`).
    completing: Option<u64>,
    /// The chaining state native code updates (`x86_64::chain`).
    chain: chain_control::Owner,
    /// The block count at which `run` stops.
    block_limit: u64,
    /// Suspended guest threads by thread pointer (see [`Suspension`]).
    suspended: BTreeMap<u64, Suspension>,
    /// The physical fetch pages whose blocks are compiled for the
    /// translated variant ([`Machine::run_translated`]).
    translated_pages: BTreeSet<u64>,
    /// The instructions, by physical address, that reached a device
    /// register from native code: blocks end before them and the engine
    /// completes them ([`device_access::DeviceSite`]).
    device_sites: BTreeMap<u64, device_access::DeviceSite>,
    /// The block being installed is a device site's stub.
    installing_stub: bool,
    /// A System exit at a word that is not System was reported
    /// ([`Self::report_exit_mismatch`]).
    exit_mismatch_reported: bool,
    /// The guest PC and data address that first needed the translated
    /// variant, for a report.
    first_translated: Option<(u64, u64)>,
    /// Device accesses answered without a model ([`Self::begin_survey`]).
    survey: Option<Survey>,
    /// The refusals of each device register and direction whose degraded
    /// answer has been reported (`take_unmodeled`).
    degraded_devices: BTreeMap<(u64, bool), BTreeSet<String>>,
    /// The context leaves SVE and SME instructions run as, by PC, word and
    /// scalable profile ([`scalable`]); discarded with the blocks.
    scalable_leaves:
        BTreeMap<(u64, u32, stela_compiler::scalable_context::ScalableProfile), scalable::Leaf>,
    /// The Allocation Tags of guest memory (FEAT_MTE2).
    tags: crate::machine::tags::Store,
    /// The current [`Regime::tag_checks`].
    tag_checks: [bool; 2],
    /// The PC of an instruction whose Tag Checks are done while it runs
    /// again untagged (`watch::retry_untagged`).
    tags_checked_at: Option<u64>,
}

/// Breakpoints that can match and an enabled software step make every
/// block a single instruction checked before it runs.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
struct DebugMode {
    breakpoints: bool,
    step: bool,
}

enum Fetched {
    Block(usize),
    Exception(Exception),
    /// The first instruction makes access checks the engine applies before
    /// it runs (see [`Variant::scalable`]).
    Checked(u32),
}

type Step = Result<(), Stop>;

/// An owned copy of a parallel machine's CPU profile and its immutable
/// instruction admission policy. Each worker still owns its execution state.
#[derive(Clone)]
pub struct WorkerProfile {
    profile: CpuProfile,
    admit_table: Arc<AdmitTable>,
    target: TargetCpu,
    /// The home machine's numbering, which every PE's code shares.
    numbering: numbering::Numbering,
    /// The home machine's code once it is the hub's: a worker whose host
    /// gives the same window starts on it, and never builds a table of its
    /// own.
    code: Option<Arc<board::Code>>,
}

impl WorkerProfile {
    pub fn chip(&self) -> Chip {
        self.profile.chip
    }

    /// Construct the machine on the worker that owns its host resources.
    pub fn start<H: Host, D: Devices>(
        self,
        cpu: Cpu,
        host: H,
        devices: D,
        processor: usize,
    ) -> Result<Machine<H, D>, &'static str> {
        Machine::new_at_prepared(
            cpu,
            host,
            devices,
            self.profile,
            self.target,
            processor,
            Some((self.admit_table, self.numbering, self.code)),
        )
    }
}

fn profile_admit_table(chip: Chip, withheld: u64) -> Arc<AdmitTable> {
    Arc::new(
        chip.admit_table(withheld)
            .unwrap_or_else(|reason| panic!("the profile has no admit table: {reason}")),
    )
}

impl<H: Host, D: Devices> Machine<H, D> {
    pub fn new(cpu: Cpu, host: H, devices: D, profile: CpuProfile, target: TargetCpu) -> Self {
        assert_eq!(
            cpu.registers.chip(),
            profile.chip,
            "the boot PE is not the profile's chip"
        );
        Self::new_at(cpu, host, devices, profile, target, 0)
            .expect("PE zero is present in every CPU profile")
    }

    /// Start the supplied architectural state on one existing profile PE.
    /// All other PEs remain held in reset; identities and board resource
    /// indices keep their original order. This selects no native executor.
    pub fn new_at(
        cpu: Cpu,
        host: H,
        devices: D,
        profile: CpuProfile,
        target: TargetCpu,
        boot_processor: usize,
    ) -> Result<Self, &'static str> {
        Self::new_at_prepared(cpu, host, devices, profile, target, boot_processor, None)
    }

    /// Capture the CPU profile before starting its parallel workers. If the
    /// caller edited the public profile, derive admission for its current
    /// policy before giving the immutable table to those workers.
    pub fn worker_profile(&self) -> WorkerProfile {
        let profile = &self.profile;
        let withheld = identification::withheld(profile.chip, &profile.fixed);
        let admit_table =
            if self.admit_table.chip() == profile.chip && self.admit_table.withheld() == withheld {
                Arc::clone(&self.admit_table)
            } else {
                profile_admit_table(profile.chip, withheld)
            };
        WorkerProfile {
            profile: profile.clone(),
            admit_table,
            target: self.target,
            numbering: numbering::Numbering {
                variants: Arc::clone(&self.variants),
                stored_reads: Arc::clone(&self.stored_reads),
                stored_writes: Arc::clone(&self.stored_writes),
            },
            code: self.code_hub.as_ref().map(|_| Arc::clone(&self.code)),
        }
    }

    fn new_at_prepared(
        cpu: Cpu,
        host: H,
        devices: D,
        profile: CpuProfile,
        target: TargetCpu,
        boot_processor: usize,
        shared: Option<(
            Arc<AdmitTable>,
            numbering::Numbering,
            Option<Arc<board::Code>>,
        )>,
    ) -> Result<Self, &'static str> {
        let (admit_table, numbering, board_code) = match shared {
            Some((admit_table, numbering, code)) => (Some(admit_table), Some(numbering), code),
            None => (None, None, None),
        };
        if boot_processor >= profile.processors.len().max(1) {
            return Err("the boot PE is outside the CPU profile");
        }
        if cpu.registers.chip() != profile.chip {
            return Err("the boot PE is not the profile's chip");
        }
        let admit_table = admit_table.unwrap_or_else(|| {
            profile_admit_table(
                profile.chip,
                identification::withheld(profile.chip, &profile.fixed),
            )
        });
        let processor_power = Processors::new(profile.processors.len().max(1))
            .expect("a CPU profile has at least its boot PE");
        let numbering = numbering.unwrap_or_else(numbering::Numbering::new);
        // The tables grow with the arena, which the host sizes from its
        // memory; a host without parts keeps the default sizes.
        let window = host.code_window();
        let code_bytes = window.map(|window| window.bytes).or_else(|| {
            host.code_partitions()
                .map(|parts| parts.bytes_per_part * parts.parts)
        });
        let (table_entries, itlb_entries) = code_bytes
            .map_or((chain::LOOKUP_ENTRIES, chain::ITLB_ENTRIES), |bytes| {
                (chain::lookup_entries(bytes), chain::itlb_entries(bytes))
            });
        // The hub's code when this host gives its window, else a board of
        // one PE until the machine joins a hub's.
        let code = match board_code {
            Some(code)
                if window.is_some() && code.arena.as_ref().map(|arena| arena.window) == window =>
            {
                code
            }
            _ => Arc::new(board::Code::new(
                board::slot_capacity(code_bytes),
                table_entries,
                window,
            )),
        };
        let mut machine = Self {
            cpu,
            host,
            devices,
            profile,
            statistics: Statistics::default(),
            step_timing: None,
            exit_words: BTreeMap::new(),
            exit_pending: [PendingExits::default(); PENDING_EXITS],
            entered_pending: Vec::new(),
            entered_total: 0,
            device_registers: BTreeMap::new(),
            fast_ipi: None,
            device_read_generation: 0,
            device_read_epochs: alloc::vec![None; stela_compiler::x86_64::register_read::DEVICE_READS],
            timed_device: None,
            timed_write: None,
            system_words: BTreeMap::new(),
            timed_word: None,
            timed_closed: false,
            system_exits: Box::default(),
            hot_samples: Vec::new(),
            dynamic_samples: Vec::new(),
            promotion_runs: 0,
            promotion_cursor: 0,
            hot_countdown: step_timing::HOT_SAMPLE_EVERY,
            hot_seed: 0x9e37_79b9_7f4a_7c15,
            #[cfg(all(feature = "std", feature = "native-profile"))]
            floating_profile: None,
            #[cfg(all(feature = "std", feature = "native-profile"))]
            preparation_metrics: None,
            target,
            code,
            alone: board::Grace::new(0),
            code_hub: None,
            reclaimed_at: None,
            memory_release_blocks: MEMORY_RELEASE_BLOCKS,
            recovery_scratch: Vec::new(),
            relocated: Default::default(),
            successor_scratch: Vec::new(),
            successor_boxes: Default::default(),
            known_slots: known_keys::DEFAULT_SLOTS,
            wkdm_buffers: None,
            install_causes: InstallCauses::default(),
            install_stream: board::YOUNG,
            held_places: Vec::new(),
            census: None,
            native_failure: None,
            variants: numbering.variants,
            stale_cursor: None,
            pages: BTreeMap::new(),
            kept_pages: stored::KEPT_PAGES,
            spare_pages: Vec::new(),
            #[cfg(feature = "native-profile")]
            stored_pages_peak: 0,
            page_uses: 0,
            stored_contexts: BTreeMap::new(),
            preparations: BTreeMap::new(),
            fetch: vec![None; FETCH_CACHE],
            jump: vec![None; JUMP_CACHE],
            translation_generation: 1,
            table_generation: 1,
            lookup_dispatchers: Default::default(),
            code_generation: 1,
            itlb: chain::LookupTable::itlb_with(itlb_entries),
            itlb_ghosts: Vec::new(),
            landing_admitted: false,
            spaces: [
                contexts::Contexts::new(contexts::LOWER_CONTEXTS),
                contexts::Contexts::new(contexts::UPPER_CONTEXTS),
            ],
            context_frames: Vec::new(),
            recent: [[None; RECENT]; 2],
            fetched_sizes: [0; 2],
            private_fetches: [false; 2],
            lower_globals: NO_GLOBALS,
            fetched_leaves: [Vec::new(), Vec::new()],
            fetched_words: [Some(Vec::new()), Some(Vec::new())],
            stored_reads: numbering.stored_reads,
            stored_writes: numbering.stored_writes,
            code_posted: None,
            plain_read: None,
            next_context: 1,
            salt_owners: [0; chain::LOOKUP_SALTS],
            salt_uses: [0; chain::LOOKUP_SALTS],
            salt_clock: 0,
            itlb_seen: Vec::new(),
            itlb_seen_bits: 0,
            fill_log: Vec::new(),
            fill_next: 0,
            pending_refills: BTreeMap::new(),
            refills: Vec::new(),
            dropped_refills: BTreeMap::new(),
            mail_returned: 0,
            applying_mail: false,
            profile_missing: Vec::new(),
            lookup: super::system::Lookup::new(),
            timers: None,
            timer_registers: [0; 2],
            floating_point: None,
            memory_checks: None,
            native_system: None,
            self_loops: true,
            event_sample: None,
            debug: None,
            device_lines: None,
            device_generation: None,
            pointer: alloc::boxed::Box::new(pointer::PointerService::new()),
            mirrors: alloc::boxed::Box::default(),
            admit_table,
            system_decode: system_access::DecodeCache::default(),
            watch: Watch::default(),
            prediction_context: None,
            synced_regime: None,
            current: boot_processor,
            waiting: None,
            peers: Vec::new(),
            processor_power,
            native_publication: None,
            native_memory_identity: Arc::new(shared_memory::ExecutionDomain::new()),
            serial_hold: None,
            parallel: None,
            attached: None,
            forwarded_power: Vec::new(),
            deferred_code: Vec::new(),
            posted: Vec::new(),
            mail_spare: Vec::new(),
            direct_exit: None,
            publishing_target: None,
            outgoing: Vec::new(),
            lines_epoch: 0,
            pending_device_read: None,
            pending_device_write: None,
            pending_device_tail: alloc::collections::VecDeque::new(),
            breakpoints: Vec::new(),
            traces: Vec::new(),
            trace_hits: Vec::new(),
            monitor_tracing: false,
            watches: Vec::new(),
            watch_hits: Vec::new(),
            unmodeled_met: Vec::new(),
            unmodeled: Vec::new(),
            slice_end: processors::QUANTUM,
            slice_deadline: u64::MAX,
            idle_poll: 0,
            clock: alloc::boxed::Box::default(),
            idle_skip: false,
            wait_limit: None,
            waited: 0,
            pacm_written: false,
            transition_note: None,
            completing: None,
            chain: chain_control::Owner::default(),
            block_limit: u64::MAX,
            suspended: BTreeMap::new(),
            translated_pages: BTreeSet::new(),
            device_sites: BTreeMap::new(),
            installing_stub: false,
            exit_mismatch_reported: false,
            first_translated: None,
            survey: None,
            degraded_devices: BTreeMap::new(),
            scalable_leaves: BTreeMap::new(),
            tags: crate::machine::tags::Store::default(),
            tag_checks: [false; 2],
            tags_checked_at: None,
        };
        machine.chain.set_table(
            machine.code.table.address(),
            machine.itlb.address(),
            [machine.code.table.mask(), machine.itlb.mask()],
        );
        machine.attach_stamps();
        machine.peers = processors::boot(&machine.profile, boot_processor);
        machine.load_profile();
        machine.sync_regime();
        Ok(machine)
    }
}

/// Arm `AArch64_InterruptPending`: a physical IRQ or FIQ line, or at EL0
/// and EL1 outside the host a virtual IRQ, FIQ or SError that
/// HCR_EL2.{IMO,FMO,AMO} route and {VI,VF,VSE} hold, masked or not.
/// Whether `cpu`'s performance monitor asserts its PMI at count `now`.
fn monitor_asserted(cpu: &Cpu, now: u64) -> bool {
    cpu.pmu
        .as_ref()
        .is_some_and(|monitor| monitor.asserted(now, cpu.retired))
}

fn interrupt_pending(cpu: &Cpu, lines: Lines) -> bool {
    if lines.irq || lines.fiq {
        return true;
    }
    let routed = |routing: u32, pending: u32| cpu.hcr(routing) && cpu.virtual_pending(pending);
    cpu.pstate.el != 2
        && !cpu.in_host()
        && (routed(HCR_IMO, HCR_VI) || routed(HCR_FMO, HCR_VF) || routed(HCR_AMO, HCR_VSE))
}

/// Descriptor access through the host's RAM, for the software walker.
struct RamTables<'a, H: Host>(&'a mut H);

impl<H: Host> mmu::TableMemory for RamTables<'_, H> {
    fn read_descriptor(&mut self, physical: u64) -> Option<u64> {
        self.0.read_ram_descriptor(physical)
    }

    fn update_descriptor(&mut self, physical: u64, old: u64, new: u64) -> Option<bool> {
        // The walker makes no hardware descriptor updates (no FEAT_HAFDBS).
        // Should it, other PEs walk and write the same tables at once: the
        // update is the host's single-copy-atomic compare-exchange, or none.
        let mut expected = old.to_le_bytes();
        self.0
            .compare_exchange_ram(physical, &mut expected, &new.to_le_bytes())
    }
}

#[cfg(test)]
mod density;
#[cfg(test)]
mod tests;
