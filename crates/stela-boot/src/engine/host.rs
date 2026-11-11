//! The bare-metal [`Host`]: guest RAM, the native code arena, shadow paging,
//! precise native faults, the local APIC timer and the TSC counter.
//!
//! Shadow roots are PML5 pages under LA57 and PML4 pages otherwise, whose
//! entries hold the guest's native ranges and the shared window
//! (`layout::window_entries`), which native code's guest range checks keep
//! every guest access out of; no other entry is ever mapped, which those
//! checks rely on. There is one root per
//! permission view (EL0, EL1, EL1 with PAN, GL1), and a second set for the
//! same views while tag checks keep Tagged memory out (`Regime::tag_checks`),
//! filled on demand by the #PF handler from the software walk of the
//! guest's tables, a window of the guest leaf per fault. The views keep
//! their roots of a few values of SPRR_PERM_EL0, SPRR_PERM_EL1 and E0PD1
//! (`Bank`). Each half
//! takes its tables from its own part of the pool, so a change of TTBR0
//! alone (a context switch) detaches the lower half and keeps the upper
//! half's, and records the leaves it was filled from
//! (`mmu::shadow::Extents`), so a TLBI unmaps only the ranges it reaches.
//! The detached lower halves of a few recent TTBR0s stay in their frames
//! (`mmu::shadow::Kept`), and a switch back attaches its half again.
//!
//! With SMAP, EL1's two views share EL1's root, as they differ only in what
//! PAN takes away, which is all data access or none: a leaf that PAN guards
//! is a user page, and PAN is RFLAGS.AC clear ([`State::root_of`]). PAN
//! changes around each copy to user memory, so this spares a root load, and
//! without PCIDs a TLB flush, on every one.
use super::interrupts::{Frame, SPURIOUS_VECTOR, TIMER_VECTOR};
use crate::Result;
use crate::{
    apic::{self, Apic},
    layout,
    tables::{self, Frames, NX, PAGE, PRESENT, WRITABLE},
};
use alloc::{sync::Arc, vec::Vec};
use core::arch::{asm, global_asm};
use stela_boot_core::native_resources::{CodeExtents, MAX_CODE_PARTS, code_part_bytes};
use stela_compiler::{GuestState, block::BlockExit};
use stela_contracts::refuse;
use stela_engine::machine::FaultCause;
use stela_engine::machine::Host;
use stela_engine::machine::InstallError;
use stela_engine::machine::NativeCode;
use stela_engine::machine::NativeExit;
use stela_engine::machine::NativeFault;
use stela_engine::machine::NativeView;
use stela_engine::machine::Regime;
use stela_engine::machine::engine::GlobalSpan;
use stela_engine::machine::engine::installed_code::InstalledCodeCache;
use stela_engine::machine::mmu;
use stela_engine::machine::mmu::Access;
use stela_engine::machine::mmu::Outcome;
use stela_engine::machine::mmu::placement::Placement;
use stela_engine::machine::mmu::shadow;
use stela_engine::machine::mmu::shadow::Extents;
use stela_engine::machine::mmu::shadow::Kept;
use stela_engine::machine::mmu::tlbi::Everywhere;
use stela_engine::machine::mmu::tlbi::Reach;
use stela_engine::machine::mmu::translated::Tlb;

mod bare_host;
pub(crate) mod chain_check;
mod faults;
pub(crate) mod pmu;
mod signals;
pub use bare_host::BareHost;
use faults::*;
pub(crate) use faults::{native_exception, page_fault};
pub(crate) use signals::{
    clear_interrupted, completion_signal, device_signals, doorbell_counter, doorbells,
    in_interrupt, interrupted, join_shootdown, ring, root_irq_active, shootdown,
    shootdown_interrupt, signal_interrupt, sleep_for, undelivered_doorbells, wake_interrupt,
};
pub use signals::{spurious_interrupt, stray_interrupt, timer_interrupt};
#[cfg(feature = "diagnostics")]
pub(crate) mod fp_diagnostic;
pub(super) mod local;
#[cfg(feature = "native-profile")]
use stela_boot_core::native_profile as profile;
use stela_boot_core::ram;
mod translated;
pub(crate) use local::activated_base;
pub(crate) use local::context::RootIdentity;
#[cfg(feature = "nvk")]
pub(crate) use local::processor;
use local::state;

/// Calls `action` with each host run behind `bytes` of guest RAM at `guest`, as
/// (host physical, bytes). False, partway, if part of it is not RAM.
#[cfg(feature = "nvk")]
pub(crate) fn guest_runs(guest: u64, bytes: u64, action: impl FnMut(u64, u64)) -> bool {
    ram::runs(&local::state().ram, guest, bytes, action)
}
pub(crate) use local::{MmioRoots, Owner as HostOwner};
pub use ram::RamRange;

/// Page-table frames from the handoff pool: a permanent part for roots and a
/// shadow part, split between the guest's two halves, each of which resets
/// wholesale.
struct Pool {
    base: u64,
    bytes: u64,
    /// The upper half's bytes at the shadow part's end
    /// (`host_memory::upper_half`); the lower half has the rest.
    upper: u64,
    permanent: u64,
    /// The next free shadow frame's offset of each half ([`LOWER`],
    /// [`UPPER`]).
    shadow: [u64; 2],
    /// First shadow frame.
    shadow_base: u64,
    /// The half the next frames map.
    half: usize,
    /// The most bytes of shadow frames each half has held at once.
    peak: [u64; 2],
}

/// The guest halves: VAs with bit 63 clear and set.
const LOWER: usize = 0;
const UPPER: usize = 1;

impl Frames for Pool {
    fn frame(&mut self) -> Option<u64> {
        let (_, end) = self.range(self.half);
        if self.shadow[self.half] + PAGE > end {
            return None;
        }
        let frame = self.base + self.shadow[self.half];
        self.shadow[self.half] += PAGE;
        let used = self.shadow[self.half] - self.range(self.half).0;
        self.peak[self.half] = self.peak[self.half].max(used);
        zero(frame);
        Some(frame)
    }
    fn table(&self, physical: u64) -> *mut u64 {
        layout::physical(physical) as *mut u64
    }
}

fn zero(frame: u64) {
    // SAFETY: a pool frame reached through the physmap.
    unsafe { (layout::physical(frame) as *mut u8).write_bytes(0, PAGE as usize) };
}

impl Pool {
    fn permanent(&mut self) -> Option<u64> {
        if self.permanent + PAGE > self.shadow_base - self.base {
            return None;
        }
        let frame = self.base + self.permanent;
        self.permanent += PAGE;
        zero(frame);
        Some(frame)
    }
    /// The offsets of `half`'s shadow frames: the shadow part's first bytes,
    /// or its last [`Self::upper`].
    fn range(&self, half: usize) -> (u64, u64) {
        let start = self.shadow_base - self.base;
        let middle = self.bytes - self.upper;
        if half == LOWER {
            (start, middle)
        } else {
            (middle, self.bytes)
        }
    }
    fn reset_half(&mut self, half: usize) {
        self.shadow[half] = self.range(half).0;
    }
}

/// Permission views; the tag-checked roots follow the unchecked ones.
const VIEWS: usize = 4;
/// EL0's view (GL0's too), EL1's, EL1's with PAN, and the GXF guarded
/// level's (GL1).
const EL0_VIEW: usize = 0;
const EL1_VIEW: usize = 1;
const PAN_VIEW: usize = 2;
const GUARDED_VIEW: usize = 3;
/// Why the whole shadow was reset: a half ran out of frames; controls
/// that differ in SPRR state alone (more than its permission registers),
/// in TTBR0 alone (with an ASID the upper half's non-global entries were
/// made under), or otherwise; another
/// pattern of tag-checked halves; [`Host::invalidate_translations`]; or a
/// TLBI that reaches everything.
#[derive(Clone, Copy)]
enum Cause {
    Exhausted,
    Sprr,
    Table,
    Controls,
    Tags,
    Invalidated,
    Maintained,
}
const CAUSES: usize = 7;
/// The TCR fields of the EL1&0 and EL2&0 regimes that configure TTBR0's
/// range alone: T0SZ, EPD0, IRGN0, ORGN0, SH0, TG0 (bits 15:0), TBI0, HPD0,
/// TBID0, NFD0, E0PD0 and TCMA0.
const LOWER_TCR: u64 = 0xffff | 1 << 37 | 1 << 41 | 1 << 51 | 1 << 53 | 1 << 55 | 1 << 57;
/// The TCR field of those regimes that decides EL0's accesses to TTBR1's
/// range alone: E0PD1, which faults them without a walk.
const USER_TCR: u64 = 1 << 56;
/// The [`mmu::Controls`] fields a [`Cause::Controls`] reset counts as
/// differing: SCTLR, TCR, TTBR0, TTBR1, MAIR, SPRR, CTRR, the regime and
/// stage 2 (its requirement or controls).
const FIELDS: usize = 9;

fn differing(from: &mmu::Controls, to: &mmu::Controls) -> [bool; FIELDS] {
    [
        from.sctlr != to.sctlr,
        from.tcr != to.tcr,
        from.ttbr0 != to.ttbr0,
        from.ttbr1 != to.ttbr1,
        from.mair != to.mair || from.xs != to.xs || from.tagged != to.tagged,
        from.sprr != to.sprr,
        from.ctrr != to.ctrr,
        from.regime != to.regime,
        from.stage2 != to.stage2 || from.second_stage != to.second_stage,
    ]
}

/// What decides the views' rights besides the guest's tables and the
/// controls a bank shares: SPRR_PERM_EL0, SPRR_PERM_EL1 and the E0PD1 bit
/// of TCR ([`USER_TCR`]), which faults EL0's accesses to the upper half.
type Rights = (u64, u64, u64);
/// The [`Rights`] whose roots the shadow keeps besides the current ones
/// ([`Bank`]).
const BANKS: usize = 6;

/// The roots the shadow filled under other [`Rights`], everything else as
/// it is now: a guest that alternates a few values, as XNU does around its
/// writes to protected data and between threads' EL0 permissions, and the
/// restore kernel with E0PD1, finds each one's tables again instead of
/// filling them anew and leaving the old ones taken. Every reset and unmap
/// reaches them as it does the current roots.
#[derive(Clone, Copy)]
struct Bank {
    /// Their rights, `None` while the roots are empty.
    rights: Option<Rights>,
    roots: [u64; ROOTS],
    /// When it was last the current bank, for replacement.
    used: u64,
}
const ROOTS: usize = 2 * VIEWS;
const SHADOW_ROOTS: usize = ROOTS * (BANKS + 1);
const _: () = assert!(SHADOW_ROOTS <= u64::BITS as usize);

/// Root `view`'s TLB context while PCIDE is set.
/// Set in a [`State::native_view`] word, with the view in bits 7:0 and the
/// requester's EL, PAN, guarded level and unprivileged flag above.
const NATIVE_VIEW: u64 = 1 << 63;

/// The [`State::native_view`] word of `view` for `requester`.
fn native_view_word(view: usize, requester: mmu::Requester) -> u64 {
    NATIVE_VIEW
        | view as u64
        | u64::from(requester.el) << 8
        | u64::from(requester.pan) << 10
        | u64::from(requester.guarded) << 11
        | u64::from(requester.unprivileged) << 12
}

/// The view of `regime`: EL0, EL1, EL1 with PAN, or the GXF guarded level,
/// in the tag-checked set when it checks tags.
fn view_of(regime: &Regime) -> usize {
    let view = match (
        regime.requester.el,
        regime.requester.pan,
        regime.requester.guarded,
    ) {
        (0, _, _) => EL0_VIEW,
        (_, _, true) => GUARDED_VIEW,
        (_, false, false) => EL1_VIEW,
        (_, true, false) => PAN_VIEW,
    };
    view + if regime.tag_checked() { VIEWS } else { 0 }
}

const fn view_context(view: usize) -> u64 {
    view as u64 + 2
}

/// Every PCID a root loads in while PCIDE is set: the host root's 0, the
/// loader root's 1 (`storage::TaggedRoots`) and the views'.
pub(crate) const CONTEXTS: u64 = view_context(ROOTS);

/// What the views' TLB contexts hold. With a context of its own, a change of
/// view keeps its own translations of the engine, the translated code and
/// the guest, which no mapping marks global. An invalidation reaches the
/// other contexts through INVPCID, or else marks them to flush when loaded.
struct Contexts {
    /// PCIDE is set ([`tag_views`]). Kept here because a hypervisor may
    /// intercept every CR4 read, and views change at each exception.
    tagged: bool,
    /// May hold what was invalidated since; so is one never loaded.
    stale: [bool; ROOTS],
    invpcid: bool,
    /// CR4.SMAP is set, and RFLAGS.AC stands for PAN ([`tag_views`]).
    pan_flag: bool,
}

/// Whether this core's views load in their contexts from now on, as
/// [`storage::TaggedRoots`] sets or restores PCIDE, and whether RFLAGS.AC
/// stands for PAN, as it sets or restores SMAP.
pub(super) fn tag_views(tagged: bool, pan_flag: bool) {
    if let Some(state) = local::installed() {
        state.contexts.tagged = tagged;
        state.contexts.pan_flag = pan_flag;
    }
}

/// Whether this core has the state [`tag_views`] sets.
pub(super) fn views_installed() -> bool {
    local::installed().is_some()
}

/// RFLAGS.AC, which is PSTATE.PAN clear where it stands for it.
const RFLAGS_AC: u64 = 1 << 18;

/// PSTATE.PAN by this core's RFLAGS.AC.
fn flags_pan() -> bool {
    let flags: u64;
    // SAFETY: reads RFLAGS through the stack, which the push and pop balance.
    unsafe { asm!("pushfq", "pop {}", out(reg) flags, options(preserves_flags)) };
    flags & RFLAGS_AC == 0
}

/// Lift SMAP for what runs next on this core, which is not guest code: the
/// loader root's firmware tables may mark any page a user page.
pub(super) fn allow_user_pages() {
    if local::installed().is_some_and(|state| state.contexts.pan_flag) {
        // SAFETY: CPL0 with SMAP, which `pan_flag` says this core has.
        unsafe { asm!("stac", options(nomem, nostack)) };
    }
}

/// Whether this core runs with PCIDE set, as far as its [`Contexts`] know,
/// which a read of CR4 would ask a hypervisor.
pub(super) fn views_tagged() -> Option<bool> {
    local::installed().map(|state| state.contexts.tagged)
}

impl Contexts {
    /// Drop `page`, which root `current` may translate, from every context.
    fn forget(&mut self, page: u64, current: usize) {
        invlpg(page);
        for view in 0..ROOTS {
            if view != current {
                self.forget_inactive(page, view);
            }
        }
    }

    /// Drop a page from one inactive view, whose context can be retained
    /// across CR3 loads only while tagged and fresh.
    fn forget_inactive(&mut self, page: u64, view: usize) {
        if !self.tagged || self.stale[view] {
            return;
        }
        if self.invpcid {
            let descriptor = [view_context(view), page];
            // SAFETY: a valid single-address INVPCID of an inactive PCID.
            unsafe {
                asm!("invpcid {}, [{}]", in(reg) 0u64, in(reg) &descriptor,
                    options(nostack, preserves_flags))
            };
        } else {
            self.stale[view] = true;
        }
    }
}
/// Frames reserved for roots and other permanent tables.
const PERMANENT_FRAMES: u64 = 64;
// The current roots and the banks', and the host root.
const _: () = assert!(ROOTS * (1 + BANKS) < PERMANENT_FRAMES as usize);
/// A root's entries.
const ROOT_ENTRIES: usize = 512;
/// log2 of the bytes a four-level root entry maps: 512 GiB.
const PML4_ENTRY_BITS: u32 = 39;
/// The last address of the lower half under LA57 (48 bits) and four-level
/// paging (47 bits).
const FIVE_LEVEL_LOWER_END: u64 = (1 << 48) - 1;
const FOUR_LEVEL_LOWER_END: u64 = (1 << 47) - 1;
/// The aligned bytes of a guest leaf one fault maps.
const FILL_WINDOW: u64 = 64 * 1024;
/// Pages a scoped invalidation flushes one by one; more reload CR3.
const INVLPG_PAGES: u64 = 16;
const FAULT_KIND: u64 = 0xffff;
pub(crate) const COUNTER_HZ: u64 = 24_000_000;

/// Fresh ticks from the installed physical Host's configured 24 MHz
/// clock. Native PMP services execute on the BSP owner and share this
/// same source, rather than reusing an Access timestamp or a copied sample.
pub(crate) fn fresh_counter() -> u64 {
    let state = state();
    counter_at_tsc(state, rdtsc())
}

/// Count this processor's 24 MHz clock from a base its software TSC
/// correction moved (`tsc::sync::corrected_base`): its raw counter reads
/// `offset` ticks behind the BSP's. Runs on the AP before it is ready.
pub(crate) fn correct_tsc_base(offset: i64) {
    let state = state();
    state.tsc_base = stela_boot_core::tsc::sync::corrected_base(state.tsc_base, offset);
}

fn counter_at_tsc(state: &State, tsc: u64) -> u64 {
    let ticks = tsc.saturating_sub(state.tsc_base);
    ((u128::from(ticks) * u128::from(state.counter_per_tsc)) >> 64) as u64
}

/// Convert an absolute BSP TSC deadline using the very same installed fixed
/// point ratio/base as fresh_counter; no guest-clock or nominal-Hz conversion.
pub(crate) fn counter_deadline(tsc: u64) -> stela_engine::machine::engine::NativeEpochDeadline {
    stela_engine::machine::engine::NativeEpochDeadline(counter_at_tsc(state(), tsc))
}

#[derive(Clone, Copy)]
struct Recorded {
    rip: u64,
    origin: stela_engine::machine::NativeFaultOrigin,
    trap: u16,
    error_code: u64,
    address: u64,
    write: bool,
    cause: FaultCause,
    /// RAX through R15 at the faulting instruction, by x86 register number.
    registers: stela_compiler::block::HostGprSnapshot,
}

pub struct State {
    ram: Vec<RamRange>,
    registered_ram: Option<alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>>,
    pool: Pool,
    /// Engine root without any guest mapping.
    host_root: u64,
    /// Original loader root kept from install, never caller metadata.
    window_root: u64,
    _loader_tables: &'static crate::paging::loader::LoaderTables,
    roots: [u64; ROOTS],
    /// The roots of other SPRR_PERM_EL0 and SPRR_PERM_EL1 pairs.
    banks: [Bank; BANKS],
    /// Switches to a bank's roots, and those that found the bank empty.
    bank_switches: u64,
    bank_misses: u64,
    /// Root loads for a change of view (EL, PAN, GXF or tag checks).
    view_switches: u64,
    contexts: Contexts,
    /// A shadow-map attempt or software-TLB fill may have published guest
    /// translations. Only a complete reset can prove both halves clean again.
    legacy_translations_dirty: bool,
    current: usize,
    /// The view and requester native code moved to since the host last
    /// followed it ([`NATIVE_VIEW`]), 0 when none: a native transition
    /// loads the view's root itself (`Host::native_view`).
    native_view: u64,
    regime: Option<Regime>,
    /// The halves whose Tagged memory the tag-checked roots keep out.
    tag_checks: [bool; 2],
    code_virtual: u64,
    code_physical: u64,
    code_bytes: u64,
    code_part_bytes: u64,
    code_parts: u64,
    /// The board's code window, which the engine allocates itself: run at
    /// `window_virtual`, written through the physmap. Empty without one.
    window_virtual: u64,
    window_physical: u64,
    window_bytes: u64,
    /// The private arena part new code goes to.
    code_unit: u64,
    /// Successful initialized installs in each arena unit, from its start.
    /// These extents require this State's backing, root and CPU owner.
    installed_extents: [CodeExtents; MAX_CODE_PARTS as usize],
    /// The furthest the arena's code has reached: what the run needed of it
    /// until it first filled and new code reclaimed the oldest.
    code_peak: u64,
    installed_immutable: InstalledCodeCache,
    recorded: Option<Recorded>,
    native: bool,
    tsc_base: u64,
    tsc_hz: u64,
    /// COUNTER_HZ / tsc_hz as a 0.64 fixed-point fraction: the counter is
    /// read several times per block, and a 128-bit division is a library
    /// call.
    counter_per_tsc: u64,
    apic: Apic,
    /// The APIC timer's ticks per second at divide-by-one.
    apic_hz: u64,
    /// LVT_TIMER holds the engine's unmasked one-shot `TIMER_VECTOR`, as
    /// `arm_timer` last wrote it, since the native clock's handover count
    /// ([`crate::native_clock::TIMER_HANDOVERS`]) was this: while it is
    /// unchanged an arm writes only the count. `signals::sleep_for`, the
    /// other writer here, keeps it true.
    timer_lvt: Option<u64>,
    /// This processor's doorbell, when it waits with MWAIT on one
    /// (`native_idle::Ring`): a PE's wake is then a store, not an IPI.
    ring: Option<crate::native_idle::Ring>,
    pub fills: u64,
    pub resets: u64,
    /// Resets because a half's shadow frames ran out.
    pub exhausted: u64,
    /// Resets of the lower half alone.
    pub lower_resets: u64,
    /// Whole resets by [`Cause`].
    reset_causes: [u64; CAUSES],
    /// The [`Cause::Controls`] resets each field differed in ([`differing`]).
    control_fields: [u64; FIELDS],
    /// The controls the last of those resets went from and to.
    last_controls: Option<(mmu::Controls, mmu::Controls)>,
    /// The leaves each half was filled from.
    extents: [Extents; 2],
    /// The span of global lower-half leaves other PEs' broadcasts read.
    lower_globals: Option<Arc<GlobalSpan>>,
    /// The lower halves of other TTBR0 contexts.
    kept: Kept,
    /// The lower half's root entries any root may hold, by index: a switch
    /// of TTBR0 visits these, a few, rather than every entry of the half.
    lower_used: [u64; ROOT_ENTRIES / 64],
    /// Physical shadow roots that may hold any lower entry, indexed by
    /// their permanent pool frame after the host root.
    lower_roots: u64,
    /// Switches of the lower half to another TTBR0, those that found its
    /// half kept, and the kept halves a TLBI or a bank's replacement
    /// dropped.
    pub lower_switches: u64,
    pub lower_found: u64,
    pub kept_dropped: u64,
    /// TLBIs that unmapped ranges rather than whole halves.
    pub scoped: u64,
    /// Pages mapped beside a fault's from the same leaf.
    pub prefills: u64,
    /// TSC ticks from a filled #PF's handler entry to its return.
    pub fill_ticks: u64,
    pub stray: u64,
    /// TSC ticks spent in compiled guest code, its exits and faults.
    pub native_ticks: u64,
    #[cfg(feature = "native-profile")]
    native_profile: profile::Profile,
    /// Where the guest's address space lies beside the window.
    placement: Placement,
    /// The translated variant's software TLB, tagged by root, and the
    /// helper's bounce slot; without the slot the host has no helper.
    tlb: Tlb,
    bounce: Option<translated::Bounce>,
    /// Translation helper calls, the TLB fills they made and the accesses
    /// they ran through the bounce slot; shadow faults in the guard.
    pub helper_calls: u64,
    pub helper_fills: u64,
    pub bounces: u64,
    pub guard_faults: u64,
}

pub use crate::apic::rdtsc;

fn load_root(root: u64) {
    // SAFETY: every root maps the window, which holds all code and data in use.
    unsafe { asm!("mov cr3, {}", in(reg) root, options(nostack, preserves_flags)) };
}

fn invlpg(address: u64) {
    // SAFETY: invalidating a TLB entry has no other effect.
    unsafe { asm!("invlpg [{}]", in(reg) address, options(nostack, preserves_flags)) };
}

pub struct Config {
    pub ram: Vec<RamRange>,
    pub pool_physical: u64,
    pub pool_bytes: u64,
    /// The loader's root, whose window entries every root shares.
    pub window_root: u64,
    pub(crate) loader_tables: &'static crate::paging::loader::LoaderTables,
    /// Processor-private executable alias; arena writes use the physmap.
    pub code_virtual: u64,
    pub code_physical: u64,
    pub code_bytes: u64,
    pub code_parts: u32,
    /// The board's code window, the same on every executor: (executable
    /// address, physical address, bytes), or `None`.
    pub window: Option<(u64, u64, u64)>,
    /// One common origin for every physical executor.
    pub tsc_base: u64,
    pub tsc_hz: u64,
    pub apic_base: u64,
    pub placement: Placement,
    /// The translation helper's bounce slot: its first page's address and
    /// the physical addresses of its two leaf entries.
    pub bounce: Option<(u64, [u64; 2])>,
}

/// Install the host state, leave the identity map and start the APIC.
///
/// # Safety
/// Runs once on this host processor, after its descriptor setup, with
/// interrupts disabled. Its pool, code arena and bounce entries are private to
/// this processor; the returned owner outlives every local native call/handler.
/// This owned post-EBS mount adopts stable relevant ISA/vendor/identity
/// declarations as a supported-platform requirement: trusted firmware and any
/// VMM preserve them throughout ownership. Stop and retire the owner before
/// physical reset/resume, microcode update or feature reconfiguration, including
/// SMT-shared changes; reinstall and re-enumerate afterwards. ExitBootServices
/// and CPUID do not prove that external promise or hardware validation.
pub unsafe fn install(config: Config) -> Result<HostOwner> {
    let code_part_bytes = code_part_bytes(config.code_bytes, config.code_parts)?;
    // The kernel's half takes what it needs of the shadow part, the user
    // half's cache the rest; a smaller pool splits in their sizes' ratio.
    let guest: u64 = config.ram.iter().map(|range| range.bytes).sum();
    let shadow = config.pool_bytes.saturating_sub(PERMANENT_FRAMES * PAGE);
    let upper = stela_boot_core::host_memory::upper_half(shadow, guest, PAGE);
    let mut pool = Pool {
        base: config.pool_physical,
        bytes: config.pool_bytes,
        upper,
        permanent: 0,
        shadow: [0; 2],
        shadow_base: config.pool_physical + PERMANENT_FRAMES * PAGE,
        half: LOWER,
        peak: [0; 2],
    };
    for half in [LOWER, UPPER] {
        pool.reset_half(half);
    }
    let (first, last) = layout::window_entries();
    if config.loader_tables.root() != config.window_root {
        return Err(refuse!(
            "loader root descriptor differs from its original owner"
        ));
    }
    let mut root = || -> Result<u64> {
        let frame = pool
            .permanent()
            .ok_or(refuse!("page-table pool too small"))?;
        for entry in first..=last {
            let value = config.loader_tables.boot_entry(entry).map_err(|_| {
                refuse!("original loader table context changed before host installation")
            })?;
            // SAFETY: this processor's fresh permanent pool allocation.
            unsafe {
                (layout::physical(frame) as *mut u64)
                    .add(entry)
                    .write(value)
            };
        }
        Ok(frame)
    };
    let host_root = root()?;
    let mut roots = [0; ROOTS];
    let mut banks = [Bank {
        rights: None,
        roots: [0; ROOTS],
        used: 0,
    }; BANKS];
    let shadow_roots = roots
        .iter_mut()
        .chain(banks.iter_mut().flat_map(|bank| &mut bank.roots));
    for (index, slot) in shadow_roots.enumerate() {
        *slot = root()?;
        assert_eq!(*slot, host_root + (index as u64 + 1) * PAGE);
    }
    load_root(host_root);
    let apic = Apic::new(
        config.apic_base,
        layout::physical(config.apic_base & apic::BASE_ADDRESS),
    );
    let ring = crate::native_idle::Ring::of(apic.id());
    // SAFETY: this processor owns its disjoint host state until the owner drops.
    let owner = unsafe {
        local::install(State {
            ram: config.ram,
            registered_ram: None,
            pool,
            host_root,
            window_root: config.window_root,
            _loader_tables: config.loader_tables,
            roots,
            banks,
            bank_switches: 0,
            bank_misses: 0,
            view_switches: 0,
            contexts: Contexts {
                tagged: false,
                stale: [true; ROOTS],
                invpcid: crate::native_mmio::shootdown::invpcid(),
                pan_flag: false,
            },
            legacy_translations_dirty: false,
            current: usize::MAX,
            native_view: 0,
            regime: None,
            tag_checks: [false; 2],
            code_virtual: config.code_virtual,
            code_physical: config.code_physical,
            code_bytes: config.code_bytes,
            code_part_bytes,
            code_parts: u64::from(config.code_parts),
            window_virtual: config.window.map_or(0, |window| window.0),
            window_physical: config.window.map_or(0, |window| window.1),
            window_bytes: config.window.map_or(0, |window| window.2),
            code_unit: 0,
            installed_extents: core::array::from_fn(|_| CodeExtents::default()),
            code_peak: 0,
            installed_immutable: InstalledCodeCache::default(),
            recorded: None,
            native: false,
            tsc_base: config.tsc_base,
            tsc_hz: config.tsc_hz,
            counter_per_tsc: u64::try_from(
                (u128::from(COUNTER_HZ) << 64) / u128::from(config.tsc_hz),
            )
            .expect("the TSC is faster than the 24 MHz counter"),
            apic,
            apic_hz: 0,
            timer_lvt: None,
            ring,
            fills: 0,
            native_ticks: 0,
            #[cfg(feature = "native-profile")]
            native_profile: profile::Profile::default(),
            resets: 0,
            exhausted: 0,
            lower_resets: 0,
            reset_causes: [0; CAUSES],
            control_fields: [0; FIELDS],
            last_controls: None,
            extents: [Extents::new(), Extents::new()],
            lower_globals: None,
            kept: Kept::new(),
            lower_used: [0; ROOT_ENTRIES / 64],
            lower_roots: 0,
            lower_switches: 0,
            lower_found: 0,
            kept_dropped: 0,
            scoped: 0,
            prefills: 0,
            fill_ticks: 0,
            stray: 0,
            placement: config.placement,
            tlb: Tlb::default(),
            bounce: config
                .bounce
                .map(|(address, entries)| translated::Bounce::new(address, entries)),
            helper_calls: 0,
            helper_fills: 0,
            bounces: 0,
            guard_faults: 0,
        })
    };
    let state = state();
    state.apic_hz = state
        .apic
        .start(SPURIOUS_VECTOR, TIMER_VECTOR, state.tsc_hz);
    Ok(owner)
}

/// The host's counters for the statistics line.
pub struct Statistics {
    /// Shadow faults filled, and the pages beside them mapped from the
    /// same leaves.
    pub fills: u64,
    pub prefills: u64,
    /// Nanoseconds the filled faults' handlers took, entry to return
    /// (without the exception's delivery and IRETQ).
    pub fill_ns: u64,
    /// Resets of the whole shadow, those because a half ran out of frames,
    /// of the lower half alone, and TLBIs that unmapped ranges instead.
    pub resets: u64,
    pub exhausted: u64,
    pub lower_resets: u64,
    /// Switches of the lower half to another TTBR0, those that found its
    /// half kept, and the kept halves dropped.
    pub lower_switches: u64,
    pub lower_found: u64,
    pub kept_dropped: u64,
    /// Whole resets by cause: exhausted frames, SPRR, TTBR0 and other
    /// controls, tag checks, invalidations and TLBIs of everything.
    pub reset_causes: [u64; CAUSES],
    /// The other-controls resets each field differed in: SCTLR, TCR, TTBR0,
    /// TTBR1, MAIR, SPRR, CTRR, the regime and stage 2.
    pub control_fields: [u64; FIELDS],
    /// The controls the last of those went from and to.
    pub last_controls: Option<(mmu::Controls, mmu::Controls)>,
    /// Switches to another SPRR_PERM_EL1 value's kernel roots, and those
    /// that found none kept.
    pub bank_switches: u64,
    pub bank_misses: u64,
    pub view_switches: u64,
    pub scoped: u64,
    pub stray: u64,
    /// The APIC timer frequency.
    pub apic_hz: u64,
    /// Translation helper calls, their TLB fills and bounced accesses, and
    /// shadow faults in the four-level guard.
    pub helper_calls: u64,
    pub helper_fills: u64,
    pub bounces: u64,
    pub guard_faults: u64,
    /// Each half's shadow bytes and the most it has held at once.
    pub shadow_bytes: [u64; 2],
    pub shadow_peak: [u64; 2],
    /// The code arena's bytes and the furthest its code has reached.
    pub code_bytes: u64,
    pub code_peak: u64,
}

pub fn statistics() -> Statistics {
    let state = state();
    Statistics {
        fills: state.fills,
        prefills: state.prefills,
        fill_ns: (u128::from(state.fill_ticks) * 1_000_000_000 / u128::from(state.tsc_hz.max(1)))
            as u64,
        resets: state.resets,
        exhausted: state.exhausted,
        lower_resets: state.lower_resets,
        lower_switches: state.lower_switches,
        lower_found: state.lower_found,
        kept_dropped: state.kept_dropped,
        reset_causes: state.reset_causes,
        control_fields: state.control_fields,
        last_controls: state.last_controls,
        bank_switches: state.bank_switches,
        bank_misses: state.bank_misses,
        view_switches: state.view_switches,
        scoped: state.scoped,
        stray: state.stray,
        apic_hz: state.apic_hz,
        helper_calls: state.helper_calls,
        helper_fills: state.helper_fills,
        bounces: state.bounces,
        guard_faults: state.guard_faults,
        shadow_bytes: [LOWER, UPPER].map(|half| {
            let (start, end) = state.pool.range(half);
            end - start
        }),
        shadow_peak: state.pool.peak,
        code_bytes: state.code_bytes,
        code_peak: state.code_peak,
    }
}

/// Serialize before the executable alias runs bytes just written through
/// the physmap alias. IRETQ to the next instruction serializes as CPUID
/// does, but never exits to a hypervisor, which intercepts every CPUID: a
/// block install and each link would otherwise pay a VM exit (Linux
/// `iret_to_self`).
fn serialize_code() {
    // SAFETY: CPL0 in long mode, with no red zone. The frame IRETQ pops
    // holds the current SS, RSP, RFLAGS and CS and the next instruction, so
    // it changes nothing but the stack below RSP.
    unsafe {
        core::arch::asm!(
            "mov {frame:e}, ss",
            "push {frame}",
            "lea {frame}, [rsp + 8]",
            "push {frame}",
            "pushfq",
            "mov {frame:e}, cs",
            "push {frame}",
            "lea {frame}, [rip + 2f]",
            "push {frame}",
            "iretq",
            "2:",
            frame = out(reg) _,
        );
    }
}

/// TSC ticks spent in compiled guest code so far.
pub fn native_ticks() -> u64 {
    state().native_ticks
}

/// Serialize both sides of the reference-counter observation. Compiler fences
/// also prevent moving memory work across these opt-in profiling boundaries.
#[cfg(feature = "native-profile")]
fn profile_timestamp() -> u64 {
    use core::sync::atomic::{Ordering, compiler_fence};
    compiler_fence(Ordering::SeqCst);
    core::arch::x86_64::__cpuid(0);
    // SAFETY: the installed native Host runs at CPL0 with its live CPU owner.
    let value = unsafe { core::arch::x86_64::_rdtsc() };
    core::arch::x86_64::__cpuid(0);
    compiler_fence(Ordering::SeqCst);
    value
}

#[cfg(feature = "native-profile")]
pub(crate) fn profile_snapshot() -> profile::Snapshot {
    state().native_profile.snapshot()
}

/// Inclusive work counters keep the outer phase, so native helper calls in
/// stopped services do not masquerade as checked-body execution time.
#[cfg(feature = "native-profile")]
pub(crate) fn profile_work_report() {
    let snapshot = profile_snapshot();
    for work in stela_engine::machine::engine::NativeTimingWork::ALL {
        let count = snapshot.work[work as usize];
        crate::output::arguments(format_args!(
            "NATIVE WORK sampled_bsp category={work:?} calls={} sampled_calls={} ticks={} phase_ticks={:?}\n",
            count.calls, count.sampled_calls, count.sampled_ticks, count.phase_ticks
        ));
    }
}

#[cfg(feature = "native-profile")]
pub(crate) fn profile_word_report() {
    use profile::words::Ranking;
    crate::output::arguments(format_args!(
        "NATIVE WORD coverage_sampled_bsp {:?}\n",
        state().native_profile.snapshot().words,
    ));
    for ranking in [
        Ranking::Frequency,
        Ranking::ObservedBytes,
        Ranking::MaximumBytes,
    ] {
        for row in state()
            .native_profile
            .top_words(ranking)
            .into_iter()
            .flatten()
        {
            crate::output::arguments(format_args!(
                "NATIVE WORD sampled_bsp rank={ranking:?} word={:#010x} event={:?} count_lower={} count_upper={} observed_attributed_bytes={} max_attributed_bytes={}\n",
                row.word,
                row.event,
                row.upper.saturating_sub(row.error),
                row.upper,
                row.observed_bytes,
                row.max_bytes,
            ));
        }
    }
}

impl State {
    fn owns_installed(&self, code: NativeCode, bytes: usize) -> bool {
        if bytes == 0 {
            return false;
        }
        let Some(offset) = (code.0 as u64).checked_sub(self.code_virtual) else {
            return false;
        };
        let Some(end) = offset.checked_add(bytes as u64) else {
            return false;
        };
        let unit = self.code_part_bytes;
        if unit == 0 || end > self.code_bytes {
            return false;
        }
        let Some(extents) = self.installed_extents.get((offset / unit) as usize) else {
            return false;
        };
        let base = offset / unit * unit;
        extents.contains(offset - base..end - base)
    }

    fn host_physical(&self, guest: u64) -> Option<u64> {
        self.run(guest).map(|(host, _)| host)
    }

    /// The longest RAM run starting at `guest`, as (host, bytes).
    fn run(&self, guest: u64) -> Option<(u64, u64)> {
        ram::run(&self.ram, guest)
    }

    fn copy(&self, guest: u64, length: usize, mut action: impl FnMut(u64, usize, usize)) -> bool {
        ram::copy(&self.ram, guest, length, |host, done, chunk| {
            action(layout::physical(host), done, chunk);
        })
    }

    fn reset_shadow(&mut self, cause: Cause) {
        if !self.legacy_translations_dirty {
            // Device translations can exist without any populated RAM view.
            self.tlb.clear_devices();
            return;
        }
        self.clear(LOWER);
        self.clear(UPPER);
        self.tlb.clear();
        self.resets += 1;
        self.reset_causes[cause as usize] += 1;
        self.reload();
        self.legacy_translations_dirty = false;
    }

    /// A TLBI ASIDE1 of the current ASID: it removes the non-global entries
    /// of that ASID alone (Arm `TLBIMatch`), so of the current shadow only
    /// a half that holds a non-global translation goes
    /// (`mmu::shadow::Extents::private`); the kernel's global upper half
    /// stays.
    /// Clearing the lower half drops the kept halves too, as their tables
    /// share its frames.
    fn drop_non_global(&mut self) {
        let mut cleared = false;
        for half in [LOWER, UPPER] {
            if self.extents[half].private() {
                self.clear(half);
                cleared = true;
            }
        }
        self.tlb.clear();
        if cleared {
            self.resets += 1;
            self.reset_causes[Cause::Maintained as usize] += 1;
            self.reload();
        }
    }

    /// Drop the lower half's translations only, and the kept halves.
    fn reset_lower(&mut self) {
        self.clear(LOWER);
        self.tlb.clear();
        self.lower_resets += 1;
        self.reload();
    }

    /// TTBR0 or the lower half's TCR fields changed from `from` to `to`:
    /// detach the lower half from every root and keep it, and attach the
    /// half kept for `to`, if any. Return the views the caller must flush.
    fn switch_lower(&mut self, from: (u64, u64), to: (u64, u64)) -> [bool; ROOTS] {
        self.lower_switches += 1;
        // XNU switches thousands of times a second: reuse a list's storage.
        let mut entries = self.kept.buffer();
        let mut changed = 0;
        let lower_roots = core::mem::take(&mut self.lower_roots);
        for entry in self.lower_entries() {
            let mut roots = lower_roots;
            while roots != 0 {
                let index = roots.trailing_zeros();
                roots &= roots - 1;
                let root = self.host_root + (u64::from(index) + 1) * PAGE;
                // SAFETY: permanent root frames through the physmap.
                let slot = unsafe { (layout::physical(root) as *mut u64).add(entry) };
                // SAFETY: as above.
                let value = unsafe { slot.read() };
                if value != 0 {
                    entries.push((root, entry, value));
                    // SAFETY: as above.
                    unsafe { slot.write(0) };
                    changed |= 1 << index;
                }
            }
        }
        self.lower_used = [0; ROOT_ENTRIES / 64];
        let extents = core::mem::take(&mut self.extents[LOWER]);
        if let Some(half) = self.kept.switch(from, entries, extents, to) {
            self.lower_found += 1;
            for &(root, entry, value) in &half.entries {
                // SAFETY: as above; the half's tables are still the pool's.
                unsafe { (layout::physical(root) as *mut u64).add(entry).write(value) };
                self.record_lower(root, entry);
                changed |= self.lower_root_bit(root);
            }
            self.extents[LOWER] = half.extents;
            self.kept.recycle(half.entries);
        }
        if changed == 0 {
            [false; ROOTS]
        } else {
            // Inactive bank roots have no PCID; selecting a bank flushes it.
            self.roots
                .map(|root| changed & self.lower_root_bit(root) != 0)
        }
    }

    fn lower_root_bit(&self, root: u64) -> u64 {
        let index = (root - self.host_root) / PAGE - 1;
        debug_assert!(root.is_multiple_of(PAGE) && index < SHADOW_ROOTS as u64);
        1 << index
    }

    fn record_lower(&mut self, root: u64, entry: usize) {
        self.lower_used[entry / 64] |= 1 << (entry % 64);
        self.lower_roots |= self.lower_root_bit(root);
    }

    /// The lower half's root entries [`State::lower_used`] names.
    fn lower_entries(&self) -> impl Iterator<Item = usize> + use<> {
        let (first, last) = self.half_entries(LOWER);
        self.lower_used
            .into_iter()
            .enumerate()
            .flat_map(|(word, mut bits)| {
                core::iter::from_fn(move || {
                    let bit = (bits != 0).then(|| bits.trailing_zeros() as usize)?;
                    bits &= bits - 1;
                    Some(word * 64 + bit)
                })
            })
            .filter(move |entry| (first..=last).contains(entry))
    }

    /// Drop the kept lower halves; their frames return with the half's.
    fn drop_kept(&mut self) {
        self.kept_dropped += self.kept.clear() as u64;
    }

    /// Take the roots of rights `to` from their bank, leaving those of
    /// `from` there: a bank of `to`, or else the least recently used,
    /// emptied. The caller must flush the views.
    fn switch_bank(&mut self, from: Rights, to: Rights) {
        self.bank_switches += 1;
        let index = match self.banks.iter().position(|bank| bank.rights == Some(to)) {
            Some(index) => index,
            None => {
                self.bank_misses += 1;
                let (index, _) = self
                    .banks
                    .iter()
                    .enumerate()
                    .min_by_key(|(_, bank)| bank.used)
                    .expect("banks");
                for root in self.banks[index].roots {
                    self.detach(root, LOWER);
                    self.detach(root, UPPER);
                }
                // The kept halves may hold entries of these roots, which
                // take other rights now.
                self.drop_kept();
                index
            }
        };
        let bank = &mut self.banks[index];
        core::mem::swap(&mut bank.roots, &mut self.roots);
        bank.rights = Some(from);
        bank.used = self.bank_switches;
    }

    /// Every root the shadow fills: the current ones and the banks'.
    fn all_roots(&self) -> impl Iterator<Item = u64> + use<> {
        let banked = self.banks.map(|bank| bank.roots);
        self.roots.into_iter().chain(banked.into_iter().flatten())
    }

    /// Unmap `half` from every root and free its frames, the kept lower
    /// halves' too.
    fn clear(&mut self, half: usize) {
        self.pool.reset_half(half);
        self.extents[half].clear();
        for root in self.all_roots() {
            self.detach(root, half);
        }
        if half == LOWER {
            self.lower_used = [0; ROOT_ENTRIES / 64];
            self.lower_roots = 0;
            self.drop_kept();
        }
    }

    /// Unmap `half` from `root`, leaving its tables' frames taken.
    fn detach(&self, root: u64, half: usize) {
        let write = |entry: usize| {
            // SAFETY: permanent root frames through the physmap.
            unsafe { (layout::physical(root) as *mut u64).add(entry).write(0) };
        };
        if half == LOWER {
            self.lower_entries().for_each(write);
        } else {
            let (first, last) = self.half_entries(half);
            (first..=last).for_each(write);
        }
    }

    /// The first and last root entries of `half` of the guest's native
    /// ranges: under LA57 PML5 entry 0 or 511; on a four-level host the
    /// PML4 entries of the lower half, or those from `U` up.
    fn half_entries(&self, half: usize) -> (usize, usize) {
        match (self.placement, half) {
            (Placement::FiveLevel, LOWER) => (0, 0),
            (Placement::FiveLevel, _) => (ROOT_ENTRIES - 1, ROOT_ENTRIES - 1),
            (Placement::FourLevel { .. }, LOWER) => (0, ROOT_ENTRIES / 2 - 1),
            (Placement::FourLevel { .. }, _) => (
                ((self.placement.native_upper() >> PML4_ENTRY_BITS) as usize) % ROOT_ENTRIES,
                ROOT_ENTRIES - 1,
            ),
        }
    }

    /// The first and last address of `half` of the native ranges.
    fn half_range(&self, half: usize) -> (u64, u64) {
        match (self.placement, half) {
            (Placement::FiveLevel, LOWER) => (0, FIVE_LEVEL_LOWER_END),
            (Placement::FourLevel { .. }, LOWER) => (0, FOUR_LEVEL_LOWER_END),
            _ => (self.placement.native_upper(), u64::MAX),
        }
    }

    /// The root `view` runs in: its own, or under [`Contexts::pan_flag`] for
    /// EL1 with PAN EL1's, whose tables hold the same leaves.
    fn root_of(&self, view: usize) -> usize {
        if self.contexts.pan_flag && view % VIEWS == PAN_VIEW {
            view - PAN_VIEW + EL1_VIEW
        } else {
            view
        }
    }

    /// The view and requester a native transition left (`native_view`):
    /// the walks and the view follow, as the engine's next regime will.
    fn follow_native(&mut self) {
        let word = core::mem::take(&mut self.native_view);
        if word & NATIVE_VIEW == 0 {
            return;
        }
        self.current = (word & 0xff) as usize;
        if let Some(regime) = &mut self.regime {
            regime.requester = mmu::Requester {
                el: (word >> 8 & 3) as u8,
                pan: word >> 10 & 1 != 0,
                unprivileged: word >> 12 & 1 != 0,
                guarded: word >> 11 & 1 != 0,
            };
        }
    }

    /// PSTATE.PAN is `pan`, as native code set it behind the engine, whose
    /// next regime says the same: the view and the walks' requester follow.
    fn follow_pan(&mut self, pan: bool) {
        let view = self.current % VIEWS;
        if self.contexts.pan_flag && (view == EL1_VIEW || view == PAN_VIEW) {
            self.current = self.current - view + if pan { PAN_VIEW } else { EL1_VIEW };
            if let Some(regime) = &mut self.regime {
                regime.requester.pan = pan;
            }
        }
    }

    /// Set RFLAGS.AC for a native run in the current view, where it only
    /// matters in EL1's root: clear for PAN, which keeps out of user pages.
    fn load_pan(&self) {
        if !self.contexts.pan_flag {
            return;
        }
        // SAFETY: CPL0 with SMAP, which `pan_flag` says this core has. Only
        // AC changes, and no code between here and the guest's reads it.
        unsafe {
            if self.current % VIEWS == PAN_VIEW {
                asm!("clac", options(nomem, nostack));
            } else {
                asm!("stac", options(nomem, nostack));
            }
        }
    }

    /// Load root `view` in its context, which keeps what the context holds
    /// unless it is stale.
    fn load_view(&mut self, view: usize) {
        let view = self.root_of(view);
        let root = self.roots[view];
        if !self.contexts.tagged {
            return load_root(root);
        }
        let stale = core::mem::replace(&mut self.contexts.stale[view], false);
        let keep = if stale {
            0
        } else {
            super::storage::KEEP_ENTRIES
        };
        load_root(root | view_context(view) | keep);
    }

    /// Flush the TLB of every view: the current root's context now, the
    /// others' when next loaded.
    fn reload(&mut self) {
        self.contexts.stale = [true; ROOTS];
        if self.current < ROOTS {
            self.load_view(self.current);
        } else {
            load_root(self.host_root);
        }
    }
}

/// Guest page tables read through the RAM map.
struct GuestTables<'a> {
    state: &'a State,
    descriptors: Option<stela_engine::memory::system_ram::DescriptorReader<'a>>,
}

impl<'a> GuestTables<'a> {
    fn new(state: &'a State) -> Self {
        Self {
            state,
            descriptors: state
                .registered_ram
                .as_ref()
                .map(|ram| ram.descriptor_reader()),
        }
    }
}

impl mmu::TableMemory for GuestTables<'_> {
    fn read_descriptor(&mut self, physical: u64) -> Option<u64> {
        if let Some(descriptors) = &mut self.descriptors {
            return descriptors.read_descriptor(physical).ok();
        }
        let host = self.state.host_physical(physical)?;
        // SAFETY: guest RAM through the physmap.
        Some(unsafe { (layout::physical(host) as *const u64).read_volatile() })
    }
    fn update_descriptor(&mut self, physical: u64, old: u64, new: u64) -> Option<bool> {
        if let Some(descriptors) = &self.descriptors {
            return descriptors.update_descriptor(physical, old, new).ok();
        }
        let host = self.state.host_physical(physical)?;
        let slot = layout::physical(host) as *mut u64;
        // SAFETY: guest RAM through the physmap; one PE, no concurrent walker.
        unsafe {
            if slot.read_volatile() != old {
                return Some(false);
            }
            slot.write_volatile(new);
        }
        Some(true)
    }
}

/// Complete one real owner-local native return, keeping exact fault GPRs.
fn native_return(code: NativeCode, exit: BlockExit, entered: u64) -> NativeExit {
    crate::native_mmio::shootdown::service();
    let state = state();
    state.native = false;
    state.native_ticks += rdtsc() - entered;
    #[cfg(feature = "native-profile")]
    state.native_profile.exit(exit.kind);
    if exit.kind == FAULT_KIND
        && let Some(recorded) = state.recorded.take()
    {
        return NativeExit::Fault(NativeFault {
            origin: recorded.origin,
            // A chained block may lie below the entered one.
            offset: recorded.rip.wrapping_sub(code.0 as u64) as usize,
            address: recorded.address,
            write: recorded.write,
            cause: recorded.cause,
            trap: recorded.trap,
            error_code: recorded.error_code,
            spills: stela_compiler::block::HostSpillSnapshot::new(),
            registers: recorded.registers,
        });
    }
    NativeExit::Exit {
        next_pc: exit.next_pc,
        kind: exit.kind,
    }
}
