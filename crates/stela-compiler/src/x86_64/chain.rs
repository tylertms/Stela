//! Block chaining for the system-mode engine: a block's direct exit to a
//! static successor can be redirected through a thunk straight into the
//! successor's code, so the dispatcher runs once for a chain of blocks.
//!
//! Each PE owns one [`ChainControl`] that [`Anchor::Control`] names. Before it
//! enters a block it sets `budget` to the chained transfers it allows,
//! `retired`, `linked` and `hits` to 0 and `block` to the entered block's
//! index. Each thunk takes one transfer from `budget` (or, when none is
//! left, exits exactly as the redirected exit would), adds the
//! instructions of the block it leaves to `retired` and names the block it
//! enters in `block`; only a table probe counts its transfers apart, in
//! `hits`, and the engine takes the others from the budget spent. Notifications publish a monotonic interrupt epoch;
//! every thunk compares it with the owner's acknowledged pre-drain snapshot
//! before spending the local budget. A mismatch takes the original exit. After the native
//! call returns, `block` is the block whose exit or fault ended the run, in
//! its low 32 bits, and the BTYPE it was entered with in bits 33:32: 0 from
//! every thunk but a lookup dispatcher of a BR or BLR form.
//! Thunks clobber only RAX, RCX, RDX and flags, which every direct exit
//! already may.
//!
//! Live code changes only at patch points, each one instruction of
//! [`PATCH_BYTES`] that lies in one aligned 8-byte word ([`patchable`]):
//! the `ret` of an exit ([`direct_patch`], [`dynamic_patch`]) and the start
//! of a link slot ([`slot_patch`]). Another core may be running the
//! instruction, so the host replaces it with one aligned 8-byte store, and
//! the core runs either the old instruction or the new one, each complete:
//! `ret` or a `jmp rel32` to a thunk or tail at an exit, a jump to the exit
//! or [`NOP5`] into the slot's body at a slot. Whatever a new instruction
//! reaches is written, and serialized by the writer, before the store: a
//! thunk or tail in fresh arena bytes, or a slot's body, which nothing runs
//! while its point still jumps over it. A body once linked is not written
//! again, as a core may still be in it after an unlink; a later link of its
//! exit takes a thunk. Code is emitted unpadded and padded where it lands
//! ([`align_direct_exit`]), as exits move with the code before them.
//!
//! A [`lookup_dispatcher`]'s probes serve an exit whose successor is not in
//! the block's page, or not static (a return), whose patch point then jumps
//! to a tail of its own ([`lookup_tail`], [`cell_tail`]). The tail
//! calls a probe shared by every exit of one BTYPE and jumps into the block
//! it found itself, so each exit has its own indirect jump for the host to
//! predict. A direct exit out of its page has a cell of its own
//! ([`LookupTable::link_cell`]), which holds an entry of its target as the
//! table does, so its probe does not hash or read the table.
//!
//! A probe translates the target first, through the PE's ITLB
//! ([`LookupTable::itlb`]): the [`block_key`] of the target's 4 KiB page in
//! the current context of its half of the address space ([`half`],
//! `ChainControl::contexts`), which the engine records at each fetch. A
//! context is a TLB's tag: the translation inputs, exception level and
//! block variant fetches run in, forgotten when a TLBI or a change no
//! control value shows may change what they translate; a TTBR0 write
//! changes only the lower half's. The context also moves an ITLB entry's
//! slot ([`itlb_salt`]), as the processes of a system map their pages at
//! the same addresses in contexts of their own. The block key names the
//! page's physical frame and the fetch state its blocks are compiled for,
//! so the block table ([`LookupTable::new`]) holds one entry per PC and
//! key: it outlives every TLBI and context change, and the address spaces
//! that map a frame at one PC share its entries. A slot's set holds two
//! entries in one cache line, as two hot PCs of one set would otherwise
//! take each other's entry at every call. A hit transfers like a thunk; an
//! ITLB miss, a table miss or no budget exits exactly as the redirected
//! exit would, its word in RDX whole. It also clobbers R8 and R9, which no
//! exit keeps live. The dispatcher of an exit that sets a nonzero BTYPE (BR
//! or BLR) also requires the entry to accept that BTYPE
//! (`LookupEntry::targets`), as a Branch Target check of the entered
//! block's first instruction would.
use crate::x86_64::asm::{
    Alu, BitTest, Code, Cond,
    Gpr::{self, R8, R9, R10, R11, Rax, Rcx, Rdx},
    Mem, ShiftOp,
    Size::{self, Dword, Qword},
};
use crate::{Anchor, GuestAddress, Place, block::ExitKind};
use alloc::{
    boxed::Box,
    collections::{BTreeMap, BTreeSet, btree_map},
    sync::Arc,
    vec,
    vec::Vec,
};
use core::sync::atomic::{AtomicU32, AtomicU64, Ordering, fence};

/// Bytes in a direct exit: `movabs rax, target` and a [`DYNAMIC_EXIT_BYTES`]
/// exit.
pub const DIRECT_EXIT_BYTES: usize = 10 + DYNAMIC_EXIT_BYTES;
/// Bytes of a `jmp rel32`: what an install writes over the start of a
/// direct exit before the code is reachable.
pub const JUMP_BYTES: usize = 5;
/// Bytes of the one instruction a live patch changes ([`patchable`]): a
/// `jmp rel32`, or the [`RET_ROOM`] or [`NOP5`] it replaces.
pub const PATCH_BYTES: usize = JUMP_BYTES;
/// The most NOP bytes before a patch point that keep its instruction in
/// one aligned 8-byte word ([`patch_pad`]).
const MAX_PAD: usize = 4;
/// Bytes of a [`link`]'s body, without and with the interruption gate.
pub const LINK_BODY_BYTES: usize = 34;
pub const POLLED_LINK_BODY_BYTES: usize = LINK_BODY_BYTES + 10;
/// Bytes in the largest [`thunk`]: the body and a return's `mov edx; ret`.
pub const THUNK_BYTES: usize = POLLED_LINK_BODY_BYTES + 6;
/// Bytes of the link slot before an exit in its page ([`link_slot`]): the
/// pad, the patch point and the body, without and with the gate.
pub const LINK_BYTES: usize = MAX_PAD + PATCH_BYTES + LINK_BODY_BYTES;
pub const POLLED_LINK_BYTES: usize = MAX_PAD + PATCH_BYTES + POLLED_LINK_BODY_BYTES;
/// The span within which a link needs no translation of its own.
const LINK_PAGE: u64 = 4096;
/// Bytes in an exit to a dynamic target in RAX: `mov edx, kind`, a pad,
/// then the patch point `ret` with room for a `jmp rel32`
/// ([`dynamic_patch`]).
pub const DYNAMIC_EXIT_BYTES: usize = 5 + MAX_PAD + PATCH_BYTES;
/// An unlinked exit's patch point: `ret` and the room a jump takes.
pub const RET_ROOM: [u8; PATCH_BYTES] = [0xc3, 0xcc, 0xcc, 0xcc, 0xcc];
/// A linked slot's patch point: the 5-byte NOP that falls into its body.
pub const NOP5: [u8; PATCH_BYTES] = [0x0f, 0x1f, 0x44, 0x00, 0x00];
/// The bits of a PC above which [`lookup_slot`] folds the next ones in:
/// what selects an entry of the largest block table.
const LOOKUP_BITS: u32 = 18;
/// The ITLB slots of one page, one a context takes by the low bits of its
/// number.
pub const LOOKUP_SALT_BITS: u32 = 6;
pub const LOOKUP_SALTS: usize = 1 << LOOKUP_SALT_BITS;
/// Entries in a block table by default, a power of two; a board sizes its
/// one table from host memory ([`LookupTable::with_entries`]).
pub const LOOKUP_ENTRIES: usize = 1 << LOOKUP_BITS;
/// The fewest and most entries a block table has.
pub const MIN_LOOKUP_ENTRIES: usize = 1 << 16;
pub const MAX_LOOKUP_ENTRIES: usize = 1 << 22;
/// The bits of a page number above which [`itlb_slot`] folds the next ones
/// in.
const ITLB_SET_BITS: u32 = 15;
/// Entries in an ITLB by default, two in each set: pages of every context
/// it keeps. A PE's is sized from host memory, between the two bounds
/// below ([`LookupTable::itlb_with`]): the largest takes 2 MiB.
pub const ITLB_ENTRIES: usize = 1 << 14;
pub const MIN_ITLB_ENTRIES: usize = 1 << 12;
pub const MAX_ITLB_ENTRIES: usize = 1 << 16;

/// Arena bytes per block table entry: a block with its exits and thunks
/// takes about 300 bytes, so a table of this many entries holds a little
/// more than every block the arena does.
const CODE_BYTES_PER_ENTRY: usize = 256;
/// Arena bytes per ITLB entry: 64 MiB of code, the most one PE's arena
/// holds, takes the largest ITLB.
const CODE_BYTES_PER_PAGE: usize = 4096;

/// The power of two at most `value`, within `low..=high`.
fn sized(value: usize, low: usize, high: usize) -> usize {
    match value.checked_ilog2() {
        Some(bits) => (1usize << bits).clamp(low, high),
        None => low,
    }
}

/// The block table entries for an arena of `code_bytes`, which the host
/// sizes from its memory ([`LookupTable::with_entries`]).
pub fn lookup_entries(code_bytes: usize) -> usize {
    sized(
        code_bytes / CODE_BYTES_PER_ENTRY,
        MIN_LOOKUP_ENTRIES,
        MAX_LOOKUP_ENTRIES,
    )
}

/// The ITLB entries for a PE whose arena holds `code_bytes`
/// ([`LookupTable::itlb_with`]).
pub fn itlb_entries(code_bytes: usize) -> usize {
    sized(
        code_bytes / CODE_BYTES_PER_PAGE,
        MIN_ITLB_ENTRIES,
        MAX_ITLB_ENTRIES,
    )
}

/// The page an ITLB entry translates.
const PAGE_SHIFT: u32 = 12;
/// A block key's frame number is below this bit, its fetch state above.
const KEY_STATE_SHIFT: u32 = 40;
/// The largest fetch state a [`block_key`] holds.
pub const KEY_STATES: u32 = 1 << (u64::BITS - KEY_STATE_SHIFT);

/// Where a thunk finds the running PE's [`ChainControl`].
const CONTROL: Place = Place::new(Anchor::Control, 0);
/// The PE's [`ChainControl`], for code outside the chain's thunks.
pub(crate) const CONTROL_PLACE: Place = CONTROL;
/// [`ChainControl::retired`], from the control's address.
pub(crate) const RETIRED_OFFSET: i32 = RETIRED;
/// [`ChainControl::loop_entries`] and [`ChainControl::loop_iterated`].
pub(crate) const LOOP_ENTRIES_OFFSET: i32 =
    core::mem::offset_of!(ChainControl, loop_entries) as i32;
pub(crate) const LOOP_ITERATED_OFFSET: i32 =
    core::mem::offset_of!(ChainControl, loop_iterated) as i32;

/// A direct exit to a static successor in a block's code.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DirectExit {
    /// Its offset in the block's code.
    pub offset: usize,
    pub target: GuestAddress,
    pub kind: ExitKind,
    /// The block's instructions it leaves unrun ([`ExitWord`]).
    pub skipped: u16,
}

impl DirectExit {
    pub fn word(&self) -> ExitWord {
        ExitWord {
            kind: self.kind,
            skipped: self.skipped,
        }
    }
}

/// What a block's exit returns in RDX: its [`ExitKind`] and, for the side
/// exit of a branch the block runs past, how many of the block's instructions
/// that exit left unrun. A [`lookup_tail`] holds the instructions it ran
/// above them, which only a dispatcher reads.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExitWord {
    pub kind: ExitKind,
    pub skipped: u16,
}

impl ExitWord {
    const KIND_BITS: u32 = 8;
    const SKIPPED_BITS: u32 = 12;
    const RAN_SHIFT: u32 = Self::KIND_BITS + Self::SKIPPED_BITS;

    pub const fn encoded(self) -> u32 {
        self.kind as u32 | (self.skipped as u32) << Self::KIND_BITS
    }

    /// The kind and the skipped instructions of a word from RDX.
    pub const fn split(word: u64) -> (u64, u16) {
        (
            word & ((1 << Self::KIND_BITS) - 1),
            (word >> Self::KIND_BITS & ((1 << Self::SKIPPED_BITS) - 1)) as u16,
        )
    }
}

impl From<ExitKind> for ExitWord {
    fn from(kind: ExitKind) -> Self {
        Self { kind, skipped: 0 }
    }
}

/// The engine's chaining state, which thunks find through [`Anchor::Control`].
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct ChainControl {
    /// Chained transfers still allowed; only the attached engine/native path
    /// writes it. Wraps below zero when a thunk finds none left.
    pub budget: u64,
    /// Instructions of the blocks chained out of.
    pub retired: u64,
    /// The index of the block running, and above it the BTYPE it was
    /// entered with.
    pub block: u64,
    /// The contexts the engine runs in now, for the lower and the upper
    /// half ([`half`]); ITLB entries of another miss. The engine never
    /// names a context 0.
    pub contexts: [u64; 2],
    /// The host address of the block table ([`LookupTable::new`]).
    pub table: u64,
    /// Transfers table probes made.
    pub hits: u64,
    /// Kept 0 by native code: the engine counts the other transfers from the
    /// budget they spent.
    pub linked: u64,
    /// Sources are published before a Release increment by a notifier.
    /// Saturates at MAX; no owner may acknowledge that exhausted epoch.
    pub interrupt_epoch: u64,
    /// The attached owner's Acquire snapshot taken before draining sources.
    /// A notification after that snapshot remains pending for the next boundary.
    pub observed_interrupt_epoch: u64,
    /// [`itlb_salt`] of each of the `contexts`.
    pub salts: [u32; 2],
    /// The host address of the PE's ITLB ([`LookupTable::itlb`]).
    pub itlb: u64,
    /// The block table's slot mask and the ITLB's set mask
    /// ([`LookupTable::mask`]): both tables are sized from host memory.
    pub masks: [u32; 2],
    /// The host address of this PE's stamps ([`STAMP_TABLE_BYTES`]), which
    /// [`stamped_tail`]s check, or 0.
    pub stamps: u64,
    /// The host address of the block table's first [`STAMPED_CELLS`] cells
    /// ([`LookupTable::stamped_cells`]), or 0.
    pub cells: u64,
    /// Where cell probes count the transfers through cells past the
    /// stamped ones; nothing reads it.
    pub heat: u64,
    /// Exits of blocks that ran their iterations inside themselves
    /// (`x86_64::block`'s self-loops), one per entry, and the instructions
    /// those entries retired past their first iteration (net of what joins
    /// skipped): the engine's counts, which only those exits write.
    pub loop_entries: u64,
    pub loop_iterated: u64,
}

// Epoch and observed state are naturally aligned atomic-width words. Capture
// and storage owners keep the mapped control alive for every embedded address.
const _: () = assert!(core::mem::offset_of!(ChainControl, interrupt_epoch) == 64);
const _: () = assert!(core::mem::offset_of!(ChainControl, observed_interrupt_epoch) == 72);
const _: () = assert!(core::mem::size_of::<ChainControl>() == 144);

/// In the block table, one block a fetch at `pc` runs while its page
/// translates to the frame and fetch state `key` names ([`block_key`]). In
/// the ITLB, one page: `pc` is its address, `key` the context it was
/// translated in, and `entry` the block key of its blocks there. Key 0 is
/// no entry.
#[repr(C)]
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct LookupEntry {
    pub pc: u64,
    pub key: u64,
    /// The block's code, or an ITLB entry's block key.
    pub entry: u64,
    /// The block's index.
    pub block: u32,
    /// The BTYPEs a branch to it may set, bit `b` for BTYPE `b`: every one
    /// outside a guarded page, else those its first instruction is
    /// compatible with. In the table's memory the word also holds
    /// [`META_BUSY`] and the entry's version above it; an entry read from
    /// a table has them clear.
    pub targets: u32,
}

/// An ITLB entry's `block` when the walk of its page ended in a global leaf
/// ([`LookupEntry::global_page`]).
pub const ITLB_GLOBAL: u32 = 1;

/// The bits of an entry's last word that hold its targets.
pub const META_TARGETS: u32 = 0xff;
/// Set in an entry's last word while a writer changes the entry: a reader
/// takes no entry with it set, nor one whose word changed while it read.
pub const META_BUSY: u32 = 1 << 8;
/// One step of the version above [`META_BUSY`] that each write advances.
const META_VERSION: u32 = 1 << 9;

/// One entry's storage. Several cores read a board's table while one
/// writer changes it, so an entry is a sequence lock: the writer sets
/// [`META_BUSY`], writes the fields, then clears it and advances the
/// version in one store; a reader takes the fields only when the last word
/// had the busy bit clear before them and was unchanged after them. Each
/// field is one aligned access, so the reader sees each whole.
#[repr(C)]
#[derive(Default)]
struct Slot {
    pc: AtomicU64,
    key: AtomicU64,
    entry: AtomicU64,
    block: AtomicU32,
    meta: AtomicU32,
}

impl Slot {
    /// The entry, once no write changed it while it was read.
    #[inline]
    fn load(&self) -> LookupEntry {
        loop {
            let first = self.meta.load(Ordering::Acquire);
            let entry = LookupEntry {
                pc: self.pc.load(Ordering::Relaxed),
                key: self.key.load(Ordering::Relaxed),
                entry: self.entry.load(Ordering::Relaxed),
                block: self.block.load(Ordering::Relaxed),
                targets: first & META_TARGETS,
            };
            fence(Ordering::Acquire);
            if first & META_BUSY == 0 && self.meta.load(Ordering::Relaxed) == first {
                return entry;
            }
            core::hint::spin_loop();
        }
    }

    /// The key alone: a reader that only checks for an entry of a group.
    #[inline]
    fn key(&self) -> u64 {
        self.key.load(Ordering::Relaxed)
    }

    /// Write `entry`. The table's writers take turns (one writer at a
    /// time per table); readers on other cores take the old entry or the
    /// new one whole.
    fn store(&self, entry: LookupEntry) {
        let held = self.meta.load(Ordering::Relaxed);
        self.meta.store(held | META_BUSY, Ordering::Relaxed);
        fence(Ordering::Release);
        self.pc.store(entry.pc, Ordering::Relaxed);
        self.key.store(entry.key, Ordering::Relaxed);
        self.entry.store(entry.entry, Ordering::Relaxed);
        self.block.store(entry.block, Ordering::Relaxed);
        let version = (held & !(META_TARGETS | META_BUSY)).wrapping_add(META_VERSION);
        self.meta
            .store(version | entry.targets & META_TARGETS, Ordering::Release);
    }
}

const _: () = assert!(core::mem::size_of::<Slot>() == core::mem::size_of::<LookupEntry>());
const _: () =
    assert!(core::mem::offset_of!(Slot, meta) == core::mem::offset_of!(LookupEntry, targets));

/// Zeroed [`Slot`]s that never move, which code names by address and which
/// other cores read while their writer writes: shared, never uniquely
/// borrowed.
struct Slots {
    /// The first slot; a header of [`HEADER_SLOTS`] ([`Slots::header`])
    /// precedes it, so the slots keep the allocation's 64-byte alignment,
    /// which a probe's set of two needs.
    base: core::ptr::NonNull<Slot>,
    len: usize,
}

// SAFETY: every field of a slot is an atomic; the allocation is owned.
unsafe impl Send for Slots {}
// SAFETY: as for Send: shared access goes through atomics only.
unsafe impl Sync for Slots {}

impl Slots {
    /// The layout of `len` slots, cache-line aligned.
    fn layout(len: usize) -> alloc::alloc::Layout {
        alloc::alloc::Layout::array::<Slot>(len.max(1))
            .and_then(|layout| layout.align_to(64))
            .expect("a table of bounded size")
    }

    fn zeroed(len: usize) -> Self {
        let layout = Self::layout(len + HEADER_SLOTS);
        // SAFETY: a nonzero size; all-zero bytes are a valid Slot (atomics).
        let start = unsafe { alloc::alloc::alloc_zeroed(layout) } as *mut Slot;
        let Some(start) = core::ptr::NonNull::new(start) else {
            alloc::alloc::handle_alloc_error(layout);
        };
        // SAFETY: the header slots are the allocation's first.
        let base = unsafe { start.add(HEADER_SLOTS) };
        Self { base, len }
    }

    /// The first four words of the header ([`ITLB_DROPS`]).
    fn header(&self) -> &[AtomicU64; 4] {
        // SAFETY: the header is in the live allocation, aligned bytes of
        // atomics.
        unsafe { &*(self.base.as_ptr().sub(HEADER_SLOTS) as *const [AtomicU64; 4]) }
    }

    /// The word at byte `offset` after the first slot, which a [`Slots`]
    /// made with room for it holds ([`LookupTable::stamped_cells`]).
    fn word(&self, offset: usize) -> &AtomicU32 {
        assert!(
            offset + 4 <= self.len * size_of::<Slot>(),
            "a word of the table"
        );
        // SAFETY: in bounds of the live, zero-initialized allocation, and
        // 4-aligned as `offset` is.
        unsafe { &*((self.base.as_ptr() as *const u8).add(offset) as *const AtomicU32) }
    }

    #[inline]
    fn get(&self, index: usize) -> &Slot {
        assert!(index < self.len, "a slot of the table");
        // SAFETY: in bounds of the live, initialized allocation.
        unsafe { &*self.base.as_ptr().add(index) }
    }

    fn address(&self) -> u64 {
        self.base.as_ptr() as u64
    }

    fn iter(&self) -> impl Iterator<Item = &Slot> {
        (0..self.len).map(|index| self.get(index))
    }
}

impl Drop for Slots {
    fn drop(&mut self) {
        // SAFETY: allocated in `zeroed` with this layout, from the header.
        unsafe {
            alloc::alloc::dealloc(
                self.base.as_ptr().sub(HEADER_SLOTS) as *mut u8,
                Self::layout(self.len + HEADER_SLOTS),
            )
        };
    }
}

/// A handle that reads a table's entries from any core without its writer's
/// lock ([`LookupTable::reader`]).
#[derive(Clone)]
pub struct TableReader {
    slots: Arc<Slots>,
    mask: u32,
}

impl TableReader {
    /// What [`LookupTable::find`] finds.
    #[inline]
    pub fn find(&self, slot: usize, pc: u64, key: u64) -> LookupEntry {
        find_in(&self.slots, slot, pc, key)
    }

    /// The slot of `pc` with `key` ([`lookup_slot`]).
    #[inline]
    pub fn slot(&self, pc: u64, key: u64) -> usize {
        lookup_slot(pc, key, self.mask)
    }

    /// The table's host address, for `ChainControl::table`.
    pub fn address(&self) -> u64 {
        self.slots.address()
    }

    /// The slot mask, for `ChainControl::masks`.
    pub fn mask(&self) -> u32 {
        self.mask
    }
}

/// The entry of `pc` with `key` in the set of `slot`, as a probe would find
/// it, if there is one, else one of `pc` with another key, else the slot's.
#[inline]
fn find_in(slots: &Slots, slot: usize, pc: u64, key: u64) -> LookupEntry {
    let first = slots.get(slot).load();
    if first.pc == pc && first.key == key {
        return first;
    }
    let second = slots.get(neighbour(slot)).load();
    if second.pc == pc { second } else { first }
}

impl LookupEntry {
    /// The ITLB entry of the page at `page` in `context`, whose blocks have
    /// the block key `key`.
    pub const fn page(page: u64, context: u64, key: u64) -> Self {
        Self {
            pc: page,
            key: context,
            entry: key,
            block: 0,
            targets: 0,
        }
    }

    /// [`Self::page`] for a page whose walk ended in a global leaf (nG=0):
    /// one that a TLBI by VA of any ASID reaches (Arm `TLBIMatch`). Probes
    /// read no ITLB entry's `block`, so it holds the mark.
    pub const fn global_page(page: u64, context: u64, key: u64) -> Self {
        Self {
            block: ITLB_GLOBAL,
            ..Self::page(page, context, key)
        }
    }

    /// Whether this ITLB entry was made by [`Self::global_page`].
    pub const fn is_global_page(&self) -> bool {
        self.block & ITLB_GLOBAL != 0
    }

    /// Whether this ITLB entry is one of `pages` in a context of `salts`.
    fn reached(&self, pages: &core::ops::RangeInclusive<u64>, salts: u64) -> bool {
        pages.contains(&self.pc) && salts >> (self.key % LOOKUP_SALTS as u64) & 1 != 0
    }
}

/// The key of the blocks of the 4 KiB frame holding `physical`, compiled for
/// the fetch state `state` (nonzero, below [`KEY_STATES`]): with the PC, what
/// decides which block a fetch runs. A translation names the frame; the
/// engine numbers the states (the block variant and what the entered
/// block's BTYPE check reads).
pub const fn block_key(physical: u64, state: u32) -> u64 {
    physical >> PAGE_SHIFT | (state as u64) << KEY_STATE_SHIFT
}

/// The frame number `key` names ([`block_key`]).
pub const fn key_frame(key: u64) -> u64 {
    key & ((1 << KEY_STATE_SHIFT) - 1)
}

/// How a [`LookupTable`] indexes where its entries went.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Grouping {
    /// By the 16 KiB granule of the PC: the ITLB, which a TLBI by VA
    /// reaches.
    Pc,
    /// By the frame the key names: the block table, which code
    /// invalidation and reclaim reach.
    Frame,
}

impl Grouping {
    fn of(self, entry: &LookupEntry) -> u64 {
        match self {
            Self::Pc => entry.pc >> GRANULE_SHIFT,
            Self::Frame => key_frame(entry.key),
        }
    }
}

/// Two slots make a set, a cache line, which a thunk probes from the
/// target's slot to the slot beside it.
const _: () = assert!(core::mem::size_of::<LookupEntry>() == 32);

/// The other entry of `slot`'s set.
const fn neighbour(slot: usize) -> usize {
    slot ^ 1
}

/// The PCs one [`Granule`] of the ITLB's index covers: 16 KiB, the largest
/// translation granule, so a TLBI's span of a fetched leaf is whole
/// granules.
const GRANULE_SHIFT: u32 = 14;

/// The sets the entries of one group (an ITLB's granule of PCs, a block
/// table's frame) were put in: every set holding such an entry is listed,
/// some perhaps twice or no longer holding one. A list that reaches `limit`
/// drops those, then may grow to twice what is left, so it stays within a
/// small multiple of the group's live entries. Most groups hold an entry or
/// a few: the table's entries spread over as many groups as they have,
/// each with a list.
#[derive(Debug, Clone, Default)]
struct Granule {
    sets: Vec<u32>,
    limit: usize,
}

/// The groups [`LookupTable`]'s front holds, a power of two.
const FRONT_GROUPS: usize = 1 << 12;
/// A group no entry has: frame numbers and granules stay below it.
const NO_GROUP: u64 = u64::MAX;

/// The front slot of `group`.
fn front_slot(group: u64) -> usize {
    group as usize & (FRONT_GROUPS - 1)
}

/// A group's list starts compacting at this many sets.
const GRANULE_SETS: usize = 8;

/// The fewest groups [`LookupTable::sweep_step`] begins a sweep at.
const SWEEP_GRANULES: usize = 1 << 12;
/// The sets of groups' lists one step of a sweep looks at, each a read of
/// a table set that is likely a cache miss: about 20 microseconds.
const SWEEP_STEP_SETS: usize = 256;

/// Cells in one allocation of [`LookupTable::link_cell`]'s, which never moves.
const CELL_CHUNK: usize = 256;

/// Cells that [`stamped_tail`]s can name: the first cells of a block table,
/// in one array ([`LookupTable::stamped_cells`]) followed by a `live` and
/// a `heat` word per cell. Later cells come in chunks of [`CELL_CHUNK`].
pub const STAMPED_CELLS: usize = 1 << 16;
/// Byte offset of a stamped cell's `live` word from the array: nonzero
/// while the cell holds what it held when its exit was linked.
pub const CELL_LIVE: usize = STAMPED_CELLS * size_of::<LookupEntry>();
/// Byte offset of a stamped cell's `heat` word: cell probe transfers
/// through it since it was linked.
pub const CELL_HEAT: usize = CELL_LIVE + STAMPED_CELLS * 4;
/// Cell probe transfers through a cell after which its exit is worth a
/// [`stamped_tail`] ([`LookupTable::hot_links`]).
pub const HOT_CELL: u32 = 256;
/// A stamped cell's `heat` once its exit has a stamped tail, or was found
/// unable to take one: never hot again before it wraps.
const SETTLED: u32 = 1 << 31;
/// Bytes of one PE's stamp of one stamped cell: the context, then the ITLB
/// drop epoch of the target's half, in which the PE's cell probe found the
/// cell's target ([`stamped_tail`]).
pub const STAMP_BYTES: usize = 16;
/// Bytes of one PE's stamps.
pub const STAMP_TABLE_BYTES: usize = STAMPED_CELLS * STAMP_BYTES;
/// Bytes of the counts in front of one PE's stamps ([`StampCount`]).
pub const STAMP_COUNTS_BYTES: usize = 64;

/// Entries of a PE's jump cache, in front of its stamp counts: the
/// targets its table probe found since the cache's serial last moved, by
/// guest PC, which a [`lookup_tail`] takes without a probe. Bits 2-9 of
/// the PC pick the entry.
pub const JUMP_CACHE_ENTRIES: usize = 256;
/// Bytes of one jump cache entry: the PC, the block's code, its index and
/// the BTYPEs it accepts (as [`LookupEntry`]), and the serial it holds in.
const JUMP_ENTRY_BYTES: usize = 32;
/// Bytes in front of a PE's stamp counts: the jump cache, then a cache
/// line whose first word is its serial.
pub const JUMP_CACHE_BYTES: usize = JUMP_CACHE_ENTRIES * JUMP_ENTRY_BYTES + 64;
/// Byte offset from the stamps of the jump cache's serial, which the
/// engine moves at every native entry ([`next_jump_serial`]) and a native
/// transition moves when it changes the lookup contexts: an entry holds
/// only while its serial is the cache's. Never 0, which no entry holds
/// until it is written.
pub const JUMP_SERIAL: i32 = -((STAMP_COUNTS_BYTES + 64) as i32);
/// Byte offset from the stamps of the jump cache's first entry.
const JUMP_CACHE: i32 = JUMP_SERIAL - (JUMP_CACHE_ENTRIES * JUMP_ENTRY_BYTES) as i32;
/// The PC bits that pick an entry, scaled by 4: times 8 an entry's offset.
const JUMP_INDEX: i32 = ((JUMP_CACHE_ENTRIES - 1) << 2) as i32;
const JUMP_PC: i32 = 0;
const JUMP_CODE: i32 = 8;
const JUMP_BLOCK: i32 = 16;
const JUMP_TARGETS: i32 = 20;
const JUMP_ENTRY_SERIAL: i32 = 24;
const _: () = assert!(JUMP_ENTRY_BYTES == 4 * 8 && JUMP_CACHE_ENTRIES.is_power_of_two());

/// Move the serial of the jump cache in front of `stamps` (0 for none):
/// every entry it holds goes. The engine does this at every native entry,
/// after which the ITLB, the contexts and the blocks may all be others.
///
/// # Safety
///
/// `stamps` is 0 or an address [`LookupTable::new_stamps`] returned, whose
/// table lives, and no native code of its PE runs.
pub unsafe fn next_jump_serial(stamps: u64) {
    if stamps == 0 {
        return;
    }
    let word = (stamps as i64 + i64::from(JUMP_SERIAL)) as *mut u64;
    // SAFETY: the caller's: `new_stamps` made the serial as an atomic in
    // front of `stamps`, and only this PE writes it.
    let serial = unsafe { AtomicU64::from_ptr(word) };
    serial.store(
        serial.load(Ordering::Relaxed).wrapping_add(1).max(1),
        Ordering::Relaxed,
    );
}

/// What one PE's stamped tails and stamping cell probe count, a word each
/// in front of its stamps, which only that PE's native code writes
/// ([`stamp_counts`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum StampCount {
    /// Transfers a stamp sent straight to their block.
    Hits,
    /// Misses for another ITLB drop epoch of the target's half (or no stamp).
    Epoch,
    /// Misses for another context of the target's half.
    Context,
    /// Misses for an interruption.
    Interrupted,
    /// Stamps the stamping cell probe wrote.
    Restamped,
    /// Stamping cell probe transfers through a cell changed since its link,
    /// which no stamp names until the exit is linked again.
    Changed,
    /// Table probe transfers, each of which fills the jump cache: the
    /// lookup tail transfers the cache did not serve. The other probe
    /// transfers counted in `ChainControl::hits` are the cache's hits.
    Filled,
}

/// How many [`StampCount`]s there are.
pub const STAMP_COUNTS: usize = 7;

impl StampCount {
    /// Its word's offset from the stamps.
    const fn offset(self) -> i32 {
        8 * self as i32 - STAMP_COUNTS_BYTES as i32
    }
}

const _: () = assert!(8 * STAMP_COUNTS <= STAMP_COUNTS_BYTES);

/// The [`StampCount`]s of the stamps at `stamps` ([`LookupTable::new_stamps`]),
/// or zeros for none, read by the PE whose native code counts them.
///
/// # Safety
///
/// `stamps` is 0 or an address `new_stamps` returned, whose table lives.
pub unsafe fn stamp_counts(stamps: u64) -> [u64; STAMP_COUNTS] {
    if stamps == 0 {
        return [0; STAMP_COUNTS];
    }
    core::array::from_fn(|count| {
        let word = (stamps as usize - STAMP_COUNTS_BYTES + 8 * count) as *mut u64;
        // SAFETY: the caller's: `new_stamps` made the counts in front of
        // `stamps` as atomics, which live while their table does.
        unsafe { AtomicU64::from_ptr(word) }.load(Ordering::Relaxed)
    })
}
/// Byte offset from an ITLB's address ([`LookupTable::address`]) of its
/// drop epochs, one 64-bit word per half: each grows when an entry of the
/// half leaves or changes, never at a capacity eviction; they start at 1.
pub const ITLB_DROPS: i32 = -((HEADER_SLOTS * size_of::<Slot>()) as i32);
/// Slots before a table's first: one cache line.
const HEADER_SLOTS: usize = 2;

/// Buckets of [`LookupTable`]'s count of held cells by PC.
const CELL_BUCKETS: usize = 1 << 16;

/// The bucket of a cell's target `pc`: the instruction index's low bits
/// mixed with the high bits, so one page's PCs spread out.
fn cell_bucket(pc: u64) -> usize {
    ((pc >> 2).wrapping_mul(0x9e37_79b9_7f4a_7c15) >> 48) as usize % CELL_BUCKETS
}

/// What an exit through a stamped cell was linked with
/// ([`LookupTable::note_link`]): its patch point, its target, the
/// instructions its block ran, its word and the BTYPE its branch sets (0
/// but for the exit of a [`predicted_tail`] of a BR or BLR form).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CellLink {
    pub site: u64,
    pub pc: u64,
    pub retired: u32,
    pub word: ExitWord,
    pub btype: u8,
}

/// A table of [`LookupEntry`]s in two-entry sets, by slot, and an index of
/// the sets each group's entries are in. The block table
/// ([`Self::new`], by [`lookup_slot`]) is grouped by frame: an IC IVAU or a
/// reclaim forgets the entries of a frame's blocks, which the keys scatter
/// across the table. The ITLB ([`Self::itlb`], by [`itlb_slot`]) is
/// grouped by 16 KiB granule of the page: a TLBI forgets the entries of a
/// span of pages in every context, whose slots the contexts' salts
/// scatter, at each of the thousands of TLBIs a second a system's
/// processes make. Either visits only the sets its entries were put in.
///
/// One writer at a time changes a table (`&mut self`); its entries and
/// cells are sequence locks ([`Slot`]), so a board's one block table is
/// read by every PE's probes and engine ([`Self::reader`]) while it does.
pub struct LookupTable {
    sets: Arc<Slots>,
    /// The slot mask of a block table, the set mask of an ITLB.
    mask: u32,
    grouping: Grouping,
    /// Each indexed group's list in `lists`.
    granules: BTreeMap<u64, u32>,
    /// Recently indexed groups and their lists, by the group's low bits:
    /// [`Self::set`] runs at nearly every fetch, and most find their group
    /// here without a walk of `granules`, which ranges still need.
    front: Box<[(u64, u32)]>,
    lists: Vec<Granule>,
    /// Lists no group has, which keep a small capacity for the next.
    free: Vec<u32>,
    /// The group [`Self::set`] indexed last and its list: the blocks of a
    /// page go in one after another.
    last: Option<(u64, u32)>,
    /// The groups indexed at which [`Self::set`] begins a sweep: half again
    /// those left by the last sweep.
    sweep_at: usize,
    /// The next group of the sweep under way, which [`Self::set`] takes in
    /// bounded steps ([`Self::sweep_step`]).
    sweep_from: Option<u64>,
    /// The cells of direct exits out of their pages ([`Self::link_cell`]),
    /// in chunks that never move, as code names their addresses.
    cells: Vec<Slots>,
    /// Cells no exit holds.
    free_cells: Vec<u32>,
    /// Each held cell by its target's PC.
    cell_pcs: BTreeSet<(u64, u32)>,
    /// Each cell holding an entry by the frame its key names, which
    /// forgetting a frame's entries reaches.
    cell_frames: BTreeSet<(u64, u32)>,
    /// How many held cells have a PC of each [`cell_bucket`]: [`Self::set`]
    /// runs at every block a fetch reaches, and most PCs no exit's cell
    /// names, so a zero count skips the search of `cell_pcs`. Empty until
    /// the first cell.
    cell_counts: Box<[u32]>,
    /// Each held cell by the host address of the exit holding it.
    cell_sites: BTreeMap<u64, u32>,
    /// The host address of each PE's stamps ([`Self::new_stamps`]), whose
    /// stamps of a stamped cell go when the cell changes.
    stamp_tables: Vec<u64>,
    /// The stamps those addresses name, which never move.
    stamp_store: Vec<Box<[AtomicU64]>>,
    /// The link of each stamped cell an exit holds, for its promotion
    /// ([`Self::hot_links`]).
    cell_links: Vec<Option<CellLink>>,
    /// The stamped cells whose exits have a stamped tail.
    stamped: BTreeSet<u32>,
    /// The state of [`Self::insert`]'s victim choice (xorshift).
    victims: u64,
    /// Entries [`Self::insert`] put out of a full set: the slot's resident,
    /// then its neighbour.
    evicted: [u64; 2],
}

impl LookupTable {
    /// A block table of [`LOOKUP_ENTRIES`] entries by [`lookup_slot`].
    pub fn new() -> Self {
        Self::with_entries(LOOKUP_ENTRIES)
    }

    /// A block table of `entries` entries, a power of two between
    /// [`MIN_LOOKUP_ENTRIES`] and [`MAX_LOOKUP_ENTRIES`].
    pub fn with_entries(entries: usize) -> Self {
        assert!(
            entries.is_power_of_two()
                && (MIN_LOOKUP_ENTRIES..=MAX_LOOKUP_ENTRIES).contains(&entries),
            "a block table of a supported size"
        );
        Self::with(entries, (entries - 1) as u32, Grouping::Frame)
    }

    /// An ITLB of [`ITLB_ENTRIES`] pages by [`itlb_slot`]
    /// ([`LookupEntry::page`]).
    pub fn itlb() -> Self {
        Self::itlb_with(ITLB_ENTRIES)
    }

    /// An ITLB of `entries` pages, a power of two between
    /// [`MIN_ITLB_ENTRIES`] and [`MAX_ITLB_ENTRIES`].
    pub fn itlb_with(entries: usize) -> Self {
        assert!(
            entries.is_power_of_two() && (MIN_ITLB_ENTRIES..=MAX_ITLB_ENTRIES).contains(&entries),
            "an ITLB of a supported size"
        );
        Self::with(entries, (entries / 2 - 1) as u32, Grouping::Pc)
    }

    fn with(entries: usize, mask: u32, grouping: Grouping) -> Self {
        let sets = Arc::new(Slots::zeroed(entries));
        for drops in &sets.header()[..2] {
            drops.store(1, Ordering::Relaxed);
        }
        Self {
            sets,
            mask,
            grouping,
            granules: BTreeMap::new(),
            front: vec![(NO_GROUP, 0); FRONT_GROUPS].into_boxed_slice(),
            lists: Vec::new(),
            free: Vec::new(),
            last: None,
            sweep_at: SWEEP_GRANULES,
            sweep_from: None,
            cells: Vec::new(),
            free_cells: Vec::new(),
            cell_pcs: BTreeSet::new(),
            cell_frames: BTreeSet::new(),
            cell_counts: Box::default(),
            cell_sites: BTreeMap::new(),
            stamp_tables: Vec::new(),
            stamp_store: Vec::new(),
            stamped: BTreeSet::new(),
            cell_links: Vec::new(),
            victims: 0x9e37_79b9_7f4a_7c15,
            evicted: [0; 2],
        }
    }

    /// Entries [`Self::insert`] put out of a full set so far: the slot's
    /// resident, then its neighbour.
    pub fn evicted(&self) -> [u64; 2] {
        self.evicted
    }

    /// The entries it holds room for.
    pub fn entries(&self) -> usize {
        self.sets.len
    }

    /// A block table's slot mask or an ITLB's set mask, for
    /// `ChainControl::masks`.
    pub fn mask(&self) -> u32 {
        self.mask
    }

    /// The block table slot of `pc` with `key` ([`lookup_slot`]).
    #[inline]
    pub fn slot(&self, pc: u64, key: u64) -> usize {
        debug_assert_eq!(self.grouping, Grouping::Frame);
        lookup_slot(pc, key, self.mask)
    }

    /// The ITLB slot of the page of `pc` in `context` ([`itlb_slot`]).
    #[inline]
    pub fn page_slot(&self, pc: u64, context: u64) -> usize {
        debug_assert_eq!(self.grouping, Grouping::Pc);
        itlb_slot(pc, context, self.mask)
    }

    /// The salt of `context` in this ITLB ([`itlb_salt`]), for
    /// `ChainControl::salts`.
    pub fn salt(&self, context: u64) -> u32 {
        itlb_salt(context, self.mask)
    }

    /// A reader of its entries for other cores.
    pub fn reader(&self) -> TableReader {
        TableReader {
            slots: self.sets.clone(),
            mask: self.mask,
        }
    }

    /// The heap bytes its index of groups and its cells hold, beside the
    /// table.
    pub fn index_bytes(&self) -> usize {
        let lists: usize = self.lists.iter().map(|list| list.sets.capacity()).sum();
        lists * size_of::<u32>()
            + self.lists.capacity() * size_of::<Granule>()
            + self.free.capacity() * size_of::<u32>()
            // A B-tree entry and its share of a node about half full.
            + self.granules.len() * 2 * size_of::<(u64, u32)>()
            + self.front.len() * size_of::<(u64, u32)>()
            + self.cells.iter().map(|cells| cells.len).sum::<usize>() * size_of::<LookupEntry>()
            + self.stamp_store.len() * (JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES + STAMP_TABLE_BYTES)
            + self.free_cells.capacity() * size_of::<u32>()
            + self.cell_counts.len() * size_of::<u32>()
            + (self.cell_pcs.len() + self.cell_frames.len() + self.cell_sites.len())
                * 2
                * size_of::<(u64, u32)>()
    }

    /// The bytes of its entries.
    pub fn table_bytes(&self) -> usize {
        self.sets.len * size_of::<LookupEntry>()
    }

    /// The table's host address, for `ChainControl::table` or `itlb`.
    pub fn address(&self) -> u64 {
        self.sets.address()
    }

    /// Forget every entry, the cells' too; exits keep their cells.
    pub fn clear(&mut self) {
        for slot in self.sets.iter() {
            if slot.key() != 0 {
                slot.store(LookupEntry::default());
            }
        }
        self.granules.clear();
        self.front.fill((NO_GROUP, 0));
        self.lists.clear();
        self.free.clear();
        self.last = None;
        self.sweep_at = SWEEP_GRANULES;
        self.sweep_from = None;
        for &(pc, cell) in &self.cell_pcs {
            let empty = LookupEntry {
                pc,
                ..LookupEntry::default()
            };
            put_cell(
                &self.cells,
                &mut self.cell_frames,
                &self.stamp_tables,
                cell,
                empty,
            );
        }
        self.cell_frames.clear();
        self.dropped(0);
        self.dropped(1 << 55);
    }

    /// Whether every entry is empty, and no cell holds one.
    pub fn is_empty(&self) -> bool {
        self.sets
            .iter()
            .all(|slot| slot.load() == LookupEntry::default())
            && self
                .cell_pcs
                .iter()
                .all(|&(_, cell)| self.cell(cell).key == 0)
    }

    /// A cell for the direct exit at host address `site` to `pc` in another
    /// page, and its host address, which the exit's [`cell_tail`] names. It
    /// holds `current`, an entry of `pc`, until forgetting reaches that
    /// entry, as the table's would go, and then the entry of `pc` the table
    /// takes next, of whatever key: a cell probe enters the block it names
    /// only while the target's page translates to that key. The cell stays
    /// the exit's until [`Self::release_cells`] reaches `site`.
    pub fn link_cell(&mut self, site: u64, pc: u64, current: Option<LookupEntry>) -> u64 {
        self.release_cells(site..=site);
        let cell = self.free_cells.pop().unwrap_or_else(|| {
            if self.cells.is_empty() {
                self.stamped_cells();
                if let Some(cell) = self.free_cells.pop() {
                    return cell;
                }
            }
            let cell = (STAMPED_CELLS + (self.cells.len() - 1) * CELL_CHUNK) as u32;
            self.cells.push(Slots::zeroed(CELL_CHUNK));
            self.free_cells
                .extend((cell + 1..cell + CELL_CHUNK as u32).rev());
            cell
        });
        let entry = current
            .filter(|entry| entry.pc == pc && entry.key != 0)
            .unwrap_or(LookupEntry {
                pc,
                ..LookupEntry::default()
            });
        put_cell(
            &self.cells,
            &mut self.cell_frames,
            &self.stamp_tables,
            cell,
            entry,
        );
        if (cell as usize) < STAMPED_CELLS {
            let words = &self.cells[0];
            words
                .word(CELL_HEAT + 4 * cell as usize)
                .store(0, Ordering::Relaxed);
            // After the entry: a probe that finds the word set read the
            // entry it was linked with.
            words
                .word(CELL_LIVE + 4 * cell as usize)
                .store(1, Ordering::Release);
        }
        self.cell_pcs.insert((pc, cell));
        if self.cell_counts.is_empty() {
            self.cell_counts = vec![0; CELL_BUCKETS].into_boxed_slice();
        }
        self.cell_counts[cell_bucket(pc)] += 1;
        self.cell_sites.insert(site, cell);
        Self::cell_slot(&self.cells, cell) as *const Slot as u64
    }

    /// Release the cells of the exits at host addresses in `sites`, whose code
    /// went: no exit enters through them again.
    /// One at a time, without a list: [`Self::link_cell`] releases its
    /// site's at every exit it links.
    pub fn release_cells(&mut self, sites: core::ops::RangeInclusive<u64>) {
        self.release_some_cells(sites, usize::MAX);
    }

    /// [`Self::release_cells`] of at most `most` cells: the site to go on
    /// from when cells may be left, so a caller holding a lock releases a
    /// long range in bounded steps.
    pub fn release_some_cells(
        &mut self,
        sites: core::ops::RangeInclusive<u64>,
        most: usize,
    ) -> Option<u64> {
        let mut from = *sites.start();
        if from > *sites.end() {
            return None;
        }
        for _ in 0..most {
            let (&site, &cell) = self.cell_sites.range(from..=*sites.end()).next()?;
            self.cell_sites.remove(&site);
            if let Some(link) = self.cell_links.get_mut(cell as usize) {
                *link = None;
            }
            self.stamped.remove(&cell);
            let pc = self.cell(cell).pc;
            if self.cell_pcs.remove(&(pc, cell)) {
                self.cell_counts[cell_bucket(pc)] -= 1;
            }
            put_cell(
                &self.cells,
                &mut self.cell_frames,
                &self.stamp_tables,
                cell,
                LookupEntry::default(),
            );
            self.free_cells.push(cell);
            match site.checked_add(1) {
                Some(next) if next <= *sites.end() => from = next,
                _ => return None,
            }
        }
        Some(from)
    }

    /// What the cell of the exit at host address `site` holds, if it has one.
    pub fn site_cell(&self, site: u64) -> Option<LookupEntry> {
        self.cell_sites.get(&site).map(|&cell| self.cell(cell))
    }

    /// The cells exits hold.
    pub fn cells(&self) -> usize {
        self.cell_sites.len()
    }

    fn cell(&self, cell: u32) -> LookupEntry {
        Self::cell_slot(&self.cells, cell).load()
    }

    fn cell_slot(cells: &[Slots], cell: u32) -> &Slot {
        match (cell as usize).checked_sub(STAMPED_CELLS) {
            None => cells[0].get(cell as usize),
            Some(later) => cells[1 + later / CELL_CHUNK].get(later % CELL_CHUNK),
        }
    }

    /// The host address of the stamped cells ([`STAMPED_CELLS`]), for
    /// `ChainControl::cells`, made with their `live` and `heat` words on
    /// first use.
    pub fn stamped_cells(&mut self) -> u64 {
        if self.cells.is_empty() {
            self.cells
                .push(Slots::zeroed(STAMPED_CELLS + STAMPED_CELLS / 4));
            // Cell 0 counts the transfers of cells past the stamped ones.
            self.free_cells.extend((1..STAMPED_CELLS as u32).rev());
        }
        self.cells[0].address()
    }

    /// New zeroed stamps for one PE ([`STAMP_TABLE_BYTES`]) after their
    /// counts ([`STAMP_COUNTS_BYTES`]) and its jump cache ([`JUMP_CACHE_BYTES`]),
    /// for `ChainControl::stamps`, which
    /// the table keeps while it lives and zeroes a cell's stamps in when
    /// the cell changes.
    pub fn new_stamps(&mut self) -> u64 {
        let words = (JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES + STAMP_TABLE_BYTES) / 8;
        let stamps: Box<[AtomicU64]> = (0..words).map(|_| AtomicU64::new(0)).collect();
        let address = stamps.as_ptr() as u64 + (JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES) as u64;
        // The jump cache's serial starts at 1: its zeroed entries hold in none.
        stamps[(JUMP_CACHE_BYTES - 64) / 8].store(1, Ordering::Relaxed);
        self.stamp_store.push(stamps);
        self.stamp_tables.push(address);
        address
    }

    /// Whether `stamps` is an address [`Self::new_stamps`] of this table
    /// returned.
    pub fn holds_stamps(&self, stamps: u64) -> bool {
        self.stamp_tables.contains(&stamps)
    }

    /// Note what the exit holding stamped cell `cell` ran and its word, for
    /// a promotion of the exit ([`Self::hot_links`]).
    pub fn note_link(&mut self, cell: u32, link: CellLink) {
        if (cell as usize) < STAMPED_CELLS {
            if self.cell_links.len() <= cell as usize {
                self.cell_links.resize(cell as usize + 1, None);
            }
            self.cell_links[cell as usize] = Some(link);
        }
    }

    /// Keep stamped cell `cell` from making its exit hot again: its exit
    /// has a stamped tail now.
    pub fn settle_heat(&mut self, cell: u32) {
        if (cell as usize) < STAMPED_CELLS {
            self.cells[0]
                .word(CELL_HEAT + 4 * cell as usize)
                .store(SETTLED, Ordering::Relaxed);
        }
    }

    /// Note that the exit holding stamped cell `cell` has a stamped tail
    /// now, until it releases the cell, and [`Self::settle_heat`].
    pub fn note_stamped(&mut self, cell: u32) {
        if (cell as usize) < STAMPED_CELLS {
            self.settle_heat(cell);
            self.stamped.insert(cell);
        }
    }

    /// How many exits have a stamped tail ([`Self::note_stamped`]).
    pub fn stamped_tails(&self) -> usize {
        self.stamped.len()
    }

    /// The cell number of the exit at host address `site`, if it holds one.
    pub fn site_cell_number(&self, site: u64) -> Option<u32> {
        self.cell_sites.get(&site).copied()
    }

    /// The links of the exits holding stamped cells `cells` whose cell
    /// probe transfers made them hot ([`HOT_CELL`]) since they were linked:
    /// each to get a stamped tail, after which [`Self::settle_heat`].
    pub fn hot_links(&self, cells: core::ops::Range<u32>) -> Vec<CellLink> {
        let Some(words) = self.cells.first() else {
            return Vec::new();
        };
        let end = cells.end.min(STAMPED_CELLS as u32);
        (cells.start.max(1)..end)
            .filter(|&cell| {
                let heat = words
                    .word(CELL_HEAT + 4 * cell as usize)
                    .load(Ordering::Relaxed);
                (HOT_CELL..SETTLED).contains(&heat)
            })
            .filter_map(|cell| {
                let link = (*self.cell_links.get(cell as usize)?)?;
                (self.cell_sites.get(&link.site) == Some(&cell)).then_some(link)
            })
            .collect()
    }

    /// Transfers cell probes made through stamped cell `cell` since it was
    /// linked, or [`SETTLED`] and beyond once its exit took a stamped tail.
    pub fn cell_heat(&self, cell: u32) -> u32 {
        if cell as usize >= STAMPED_CELLS {
            return 0;
        }
        self.cells[0]
            .word(CELL_HEAT + 4 * cell as usize)
            .load(Ordering::Relaxed)
    }

    /// A drop when either slot of `slot`'s set holds `entry`'s page and
    /// context with another block key: the page's translation changes,
    /// whichever slot the new entry or a moved resident then takes.
    fn note_rekey(&self, slot: usize, entry: &LookupEntry) {
        let changed = |at: usize| {
            let held = self.sets.get(at);
            held.key() == entry.key && entry.key != 0 && {
                let held = held.load();
                held.pc == entry.pc && held.key == entry.key && held != *entry
            }
        };
        if changed(slot) || changed(neighbour(slot)) {
            self.dropped(entry.pc);
        }
    }

    /// Grow the drop epoch of `pc`'s half: an ITLB entry there left or
    /// changed ([`ITLB_DROPS`]).
    fn dropped(&self, pc: u64) {
        if self.grouping == Grouping::Pc {
            self.sets.header()[half(pc)].fetch_add(1, Ordering::Relaxed);
        }
    }

    /// The entry of `pc` with `key` in the set of `slot`, as a probe would
    /// find it, if there is one, else one of `pc` with another key, else
    /// the slot's.
    #[inline]
    pub fn find(&self, slot: usize, pc: u64, key: u64) -> LookupEntry {
        find_in(&self.sets, slot, pc, key)
    }

    /// The entry in `slot`.
    pub fn entry(&self, slot: usize) -> LookupEntry {
        self.sets.get(slot).load()
    }

    /// Put `entry` in the slot its PC and key pick. What held the slot,
    /// unless it is the entry's own older self, moves beside it into an
    /// empty neighbour; with both places held, one of the two leaves,
    /// chosen at random. Always putting out the neighbour let two entries
    /// a lookup keeps reaching put each other out, round after round,
    /// while a third that nothing reaches stayed: a random victim leaves
    /// that third the one out after a few rounds.
    pub fn insert(&mut self, slot: usize, entry: LookupEntry) {
        if self.grouping == Grouping::Pc {
            self.note_rekey(slot, &entry);
        }
        let resident = self.entry(slot);
        if resident == entry {
            return;
        }
        if resident.key != 0 && (resident.pc != entry.pc || resident.key != entry.key) {
            let beside = self.entry(neighbour(slot));
            let keep_resident = if beside.key == 0 {
                true
            } else {
                self.victims ^= self.victims << 13;
                self.victims ^= self.victims >> 7;
                self.victims ^= self.victims << 17;
                let resident_stays = self.victims >> 63 != 0;
                self.evicted[usize::from(resident_stays)] += 1;
                resident_stays
            };
            if keep_resident {
                // The same set, which the resident's group lists already.
                self.sets.get(neighbour(slot)).store(resident);
            }
        }
        self.set(slot, entry);
    }

    /// Put `entry` in `slot` alone, and in every cell of its PC.
    pub fn set(&mut self, slot: usize, entry: LookupEntry) {
        if self.grouping == Grouping::Pc {
            // A page of a context that leaves: a drop. Another page put
            // out of the slot is not one; its translation still holds.
            let resident = self.sets.get(slot).load();
            if resident.key != 0 && entry.key == 0 {
                self.dropped(resident.pc);
            }
            self.note_rekey(slot, &entry);
        }
        self.sets.get(slot).store(entry);
        if entry.key == 0 {
            return;
        }
        if !self.cell_counts.is_empty() && self.cell_counts[cell_bucket(entry.pc)] != 0 {
            for &(_, cell) in self.cell_pcs.range((entry.pc, 0)..=(entry.pc, u32::MAX)) {
                put_cell(
                    &self.cells,
                    &mut self.cell_frames,
                    &self.stamp_tables,
                    cell,
                    entry,
                );
            }
        }
        let set = (slot / 2) as u32;
        let group = self.grouping.of(&entry);
        let place = match self.last {
            Some((last, place)) if last == group => place,
            _ => {
                if self.sweep_from.is_some() || self.granules.len() >= self.sweep_at {
                    self.sweep_step();
                }
                let front = front_slot(group);
                let place = match self.front[front] {
                    (cached, place) if cached == group => place,
                    _ => {
                        let place = match self.granules.entry(group) {
                            btree_map::Entry::Occupied(listed) => *listed.get(),
                            btree_map::Entry::Vacant(vacant) => {
                                *vacant.insert(self.free.pop().unwrap_or_else(|| {
                                    self.lists.push(Granule::default());
                                    (self.lists.len() - 1) as u32
                                }))
                            }
                        };
                        self.front[front] = (group, place);
                        place
                    }
                };
                self.last = Some((group, place));
                place
            }
        };
        let (sets, grouping) = (&self.sets, self.grouping);
        let listed = &mut self.lists[place as usize];
        if listed.sets.last() == Some(&set) {
            return;
        }
        if listed.sets.len() >= listed.limit.max(GRANULE_SETS) {
            listed.sets.sort_unstable();
            listed.sets.dedup();
            listed.sets.retain(|&set| holds(sets, set, grouping, group));
            listed.limit = 2 * listed.sets.len();
            let kept = 2 * listed.limit.max(GRANULE_SETS);
            if listed.sets.capacity() > 2 * kept {
                listed.sets.shrink_to(kept);
            }
        }
        listed.sets.push(set);
    }

    /// Drop the sets of each group's list that hold none of its entries,
    /// and the groups no set holds an entry of. Entries another PC's took
    /// the place of leave their groups listed, which only a sweep finds:
    /// the groups a stream of new PCs (processes of new layouts) reaches
    /// would grow the index on, while the table holds a bounded number.
    ///
    /// One step visits the groups from where the last stopped until it
    /// looked at [`SWEEP_STEP_SETS`] of their sets (at least one group):
    /// a board's block table is written under its code lock, and a whole
    /// sweep of a table of millions of entries read a set of each at
    /// random for tens of milliseconds while every PE waited (run
    /// 20261111-025825: `target` holds of 88 ms, all PEs stalled 190 ms).
    /// Each step looks at more groups than the insert that took it adds,
    /// so a sweep ends; the next begins at half again what it left.
    fn sweep_step(&mut self) {
        let from = self.sweep_from.take().unwrap_or(0);
        let (sets, grouping) = (&self.sets, self.grouping);
        let mut emptied = Vec::new();
        let mut visited = 0;
        let mut next = None;
        for (&group, &place) in self.granules.range(from..) {
            if visited >= SWEEP_STEP_SETS {
                next = Some(group);
                break;
            }
            let listed = &mut self.lists[place as usize];
            visited += listed.sets.len().max(1);
            listed.sets.sort_unstable();
            listed.sets.dedup();
            listed.sets.retain(|&set| holds(sets, set, grouping, group));
            listed.limit = 2 * listed.sets.len();
            if listed.sets.is_empty() {
                emptied.push(group);
            }
        }
        self.release(emptied);
        match next {
            Some(group) => self.sweep_from = Some(group),
            None => {
                let left = self.granules.len();
                self.sweep_at = (left + left / 2).max(SWEEP_GRANULES);
            }
        }
    }

    /// A whole sweep at once ([`Self::sweep_step`] until it ends).
    #[cfg(test)]
    fn sweep(&mut self) {
        self.sweep_step();
        while self.sweep_from.is_some() {
            self.sweep_step();
        }
    }

    /// Let the `emptied` groups' lists go to the free lists.
    fn release(&mut self, emptied: Vec<u64>) {
        for group in emptied {
            let front = &mut self.front[front_slot(group)];
            if front.0 == group {
                *front = (NO_GROUP, 0);
            }
            if let Some(place) = self.granules.remove(&group) {
                let listed = &mut self.lists[place as usize];
                listed.sets.clear();
                listed.sets.shrink_to(GRANULE_SETS);
                listed.limit = 0;
                self.free.push(place);
            }
            if self.last.is_some_and(|(last, _)| last == group) {
                self.last = None;
            }
        }
    }

    /// Forget the ITLB entries of the pages in `pages` in the contexts of
    /// `salts`, a mask of salt numbers: whether the pages had entries to
    /// look at.
    pub fn forget_range(&mut self, pages: core::ops::RangeInclusive<u64>, salts: u64) -> bool {
        self.forget_range_noting(pages, salts, |_| {})
    }

    /// [`Self::forget_range`], calling `noted` with each entry it forgets.
    pub fn forget_range_noting(
        &mut self,
        pages: core::ops::RangeInclusive<u64>,
        salts: u64,
        mut noted: impl FnMut(&LookupEntry),
    ) -> bool {
        self.forget_range_where(pages.clone(), |entry| {
            let reached = entry.reached(&pages, salts);
            if reached {
                noted(entry);
            }
            reached
        })
    }

    /// Forget the ITLB entries of the pages in `pages` that `forget` names
    /// (for one ASID's contexts, whatever salts other contexts share with
    /// them): whether the pages had entries to look at.
    pub fn forget_range_where(
        &mut self,
        pages: core::ops::RangeInclusive<u64>,
        mut forget: impl FnMut(&LookupEntry) -> bool,
    ) -> bool {
        debug_assert_eq!(self.grouping, Grouping::Pc);
        let groups = *pages.start() >> GRANULE_SHIFT..=*pages.end() >> GRANULE_SHIFT;
        let found = self.forget_groups(groups, |entry| pages.contains(&entry.pc) && forget(entry));
        debug_assert!(self.cell_pcs.is_empty());
        found
    }

    /// Forget the block table's entries of the frames numbered in `frames`
    /// that `forget` names, the cells' too: whether the frames had entries
    /// to look at.
    pub fn forget_frames(
        &mut self,
        frames: core::ops::RangeInclusive<u64>,
        mut forget: impl FnMut(&LookupEntry) -> bool,
    ) -> bool {
        debug_assert_eq!(self.grouping, Grouping::Frame);
        let reached = |entry: &LookupEntry| frames.contains(&key_frame(entry.key));
        let mut found = self.forget_groups(frames.clone(), |entry| reached(entry) && forget(entry));
        let start = (*frames.start(), 0);
        let end = (*frames.end(), u32::MAX);
        let held: Vec<u32> = self
            .cell_frames
            .range(start..=end)
            .map(|&(_, cell)| cell)
            .filter(|&cell| forget(&self.cell(cell)))
            .collect();
        for cell in held {
            let pc = self.cell(cell).pc;
            let empty = LookupEntry {
                pc,
                ..LookupEntry::default()
            };
            put_cell(
                &self.cells,
                &mut self.cell_frames,
                &self.stamp_tables,
                cell,
                empty,
            );
            found = true;
        }
        found
    }

    /// Forget the block table's entry of `pc` with `key` that names block
    /// `block`, in its set and in the cells of `pc`: a block's entries
    /// without a walk of its frame's sets. The frame's list keeps the set
    /// until a sweep or a compaction drops it. The cells of `pc` it looked
    /// at.
    pub fn forget_block(&mut self, pc: u64, key: u64, block: u32) -> usize {
        debug_assert_eq!(self.grouping, Grouping::Frame);
        let named =
            |entry: &LookupEntry| entry.pc == pc && entry.key == key && entry.block == block;
        let slot = self.slot(pc, key);
        for at in [slot, neighbour(slot)] {
            let held = self.sets.get(at);
            if held.key() == key && named(&held.load()) {
                held.store(LookupEntry::default());
            }
        }
        if self.cell_counts.is_empty() || self.cell_counts[cell_bucket(pc)] == 0 {
            return 0;
        }
        let mut visited = 0;
        for &(_, cell) in self.cell_pcs.range((pc, 0)..=(pc, u32::MAX)) {
            visited += 1;
            if named(&Self::cell_slot(&self.cells, cell).load()) {
                let empty = LookupEntry {
                    pc,
                    ..LookupEntry::default()
                };
                put_cell(
                    &self.cells,
                    &mut self.cell_frames,
                    &self.stamp_tables,
                    cell,
                    empty,
                );
            }
        }
        visited
    }

    /// Forget the entries `forget` names in the sets of the groups in
    /// `groups`: whether the groups had entries to look at.
    fn forget_groups(
        &mut self,
        groups: core::ops::RangeInclusive<u64>,
        mut forget: impl FnMut(&LookupEntry) -> bool,
    ) -> bool {
        let grouping = self.grouping;
        let mut emptied = Vec::new();
        let mut found = false;
        // A TLBI of the span: its halves' epochs grow even where this ITLB
        // holds none of its pages, as an evicted page may come back with
        // the translation the TLBI made way for.
        if grouping == Grouping::Pc {
            let (first, last) = (
                *groups.start() << GRANULE_SHIFT,
                *groups.end() << GRANULE_SHIFT,
            );
            self.dropped(first);
            if half(first) != half(last) {
                self.dropped(last);
            }
        }
        let sets = &self.sets;
        for (&group, &place) in self.granules.range(groups) {
            found = true;
            let listed = &mut self.lists[place as usize];
            listed.sets.retain(|&set| {
                for slot in [2 * set as usize, 2 * set as usize + 1] {
                    let held = sets.get(slot);
                    if held.key() == 0 {
                        continue;
                    }
                    let entry = held.load();
                    if entry.key != 0 && grouping.of(&entry) == group && forget(&entry) {
                        held.store(LookupEntry::default());
                    }
                }
                holds(sets, set, grouping, group)
            });
            if listed.sets.is_empty() {
                emptied.push(group);
            }
        }
        self.release(emptied);
        found
    }
}

/// Put `entry` in `cell`, keeping `frames`, the cells by the frame their
/// keys name, in step.
fn put_cell(
    cells: &[Slots],
    frames: &mut BTreeSet<(u64, u32)>,
    stamps: &[u64],
    cell: u32,
    entry: LookupEntry,
) {
    let held = LookupTable::cell_slot(cells, cell);
    // The entry it holds already: nothing changes, and its stamps hold.
    if held.load() == entry {
        return;
    }
    let key = held.key();
    if key != 0 {
        frames.remove(&(key_frame(key), cell));
    }
    // A stamped cell's `live` word goes before the entry changes, so a
    // probe that reads the new entry finds it clear; then every PE's
    // stamps of it, so no stamped tail enters the block it held.
    let live = ((cell as usize) < STAMPED_CELLS)
        .then(|| cells[0].word(CELL_LIVE + 4 * cell as usize))
        .filter(|live| live.load(Ordering::Relaxed) != 0);
    if let Some(live) = live {
        live.store(0, Ordering::Relaxed);
    }
    held.store(entry);
    if live.is_some() {
        for &table in stamps {
            let stamp = (table as usize + STAMP_BYTES * cell as usize) as *const [AtomicU64; 2];
            // SAFETY: `add_stamps`'s contract: live stamps of atomics.
            for word in unsafe { &*stamp } {
                word.store(0, Ordering::Relaxed);
            }
        }
    }
    if entry.key != 0 {
        frames.insert((key_frame(entry.key), cell));
    }
}

/// Whether set `set` holds an entry of `group`.
fn holds(sets: &Slots, set: u32, grouping: Grouping, group: u64) -> bool {
    [2 * set as usize, 2 * set as usize + 1]
        .into_iter()
        .any(|slot| {
            let held = sets.get(slot);
            held.key() != 0 && {
                let entry = held.load();
                entry.key != 0 && grouping.of(&entry) == group
            }
        })
}

impl Default for LookupTable {
    fn default() -> Self {
        Self::new()
    }
}

/// The bits of `ChainControl::block` above the block's index that hold the
/// BTYPE it was entered with.
pub const BLOCK_BTYPE_SHIFT: u32 = 32;

/// The half of the address space `pc` is in, by VA bit 55: 0 for TTBR0's,
/// 1 for TTBR1's.
pub const fn half(pc: u64) -> usize {
    (pc >> 55 & 1) as usize
}

/// What `context` flips of the ITLB slots of the pages it translates, in an
/// ITLB of the set mask `sets`: the salt's bits are the mask's highest.
pub const fn itlb_salt(context: u64, sets: u32) -> u32 {
    (context as u32 & ((1 << LOOKUP_SALT_BITS) - 1))
        << sets.count_ones().saturating_sub(LOOKUP_SALT_BITS)
}

/// The ITLB slot of the page of `pc` in `context`, in an ITLB of the set
/// mask `sets`: the first of the set that the page number's low bits with
/// the next ones folded in, and the context's salt, pick. The set's other
/// slot holds the entry this one held before ([`LookupTable::insert`]).
/// Neighbouring pages, which code runs together, take two sets: a third
/// page of one set takes the place of neither, as it would if they were the
/// set's two slots.
pub const fn itlb_slot(pc: u64, context: u64, sets: u32) -> usize {
    let pc = pc as u32;
    let set = pc >> PAGE_SHIFT ^ pc >> (PAGE_SHIFT + ITLB_SET_BITS) ^ itlb_salt(context, sets);
    2 * (set & sets) as usize
}

/// The block table slot of `pc` with the block key `key`, in a table of the
/// slot mask `mask`: its low bits with the next ones folded in, as code a
/// multiple of the table's span apart is common and would share every
/// slot, and the key's frame number, so that the processes' own code at one
/// address takes slots apart while the pages they share share their
/// entries.
pub const fn lookup_slot(pc: u64, key: u64, mask: u32) -> usize {
    let pc = pc as u32;
    (pc >> 2 ^ pc >> (2 + LOOKUP_BITS) ^ key as u32) as usize & mask as usize
}

// `sub qword [rcx],1` addresses the budget at offset 0.
const _: () = assert!(core::mem::offset_of!(ChainControl, budget) == 0);
const RETIRED: i32 = core::mem::offset_of!(ChainControl, retired) as i32;
const BLOCK: i32 = core::mem::offset_of!(ChainControl, block) as i32;
const CONTEXTS: i32 = core::mem::offset_of!(ChainControl, contexts) as i32;
const SALTS: i32 = core::mem::offset_of!(ChainControl, salts) as i32;
const TABLE: i32 = core::mem::offset_of!(ChainControl, table) as i32;
const ITLB: i32 = core::mem::offset_of!(ChainControl, itlb) as i32;
const MASKS: i32 = core::mem::offset_of!(ChainControl, masks) as i32;
const HITS: i32 = core::mem::offset_of!(ChainControl, hits) as i32;
const STAMPS: i32 = core::mem::offset_of!(ChainControl, stamps) as i32;
const CELLS: i32 = core::mem::offset_of!(ChainControl, cells) as i32;
const HEAT: i32 = core::mem::offset_of!(ChainControl, heat) as i32;
#[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
const LINKED: i32 = core::mem::offset_of!(ChainControl, linked) as i32;
const INTERRUPT_EPOCH: i32 = core::mem::offset_of!(ChainControl, interrupt_epoch) as i32;
const OBSERVED_INTERRUPT_EPOCH: i32 =
    core::mem::offset_of!(ChainControl, observed_interrupt_epoch) as i32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Interruption {
    Epoch,
    #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
    LockedBudget,
    #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
    Flag,
    #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
    Legacy,
}
impl Interruption {
    fn extra_bytes(self) -> usize {
        match self {
            Self::Epoch => 10,
            #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
            Self::LockedBudget => 1,
            #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
            Self::Flag => 7,
            #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
            Self::Legacy => 0,
        }
    }
}
fn interruption_gate(
    asm: &mut crate::x86_64::asm::Asm,
    policy: Interruption,
    scratch: Gpr,
) -> Option<crate::x86_64::asm::Patch> {
    gate(asm, policy, scratch, false)
}

/// [`interruption_gate`] with a 32-bit branch, for the probes' far exits.
fn far_interruption_gate(
    asm: &mut crate::x86_64::asm::Asm,
    policy: Interruption,
    scratch: Gpr,
) -> Option<crate::x86_64::asm::Patch> {
    gate(asm, policy, scratch, true)
}

fn gate(
    asm: &mut crate::x86_64::asm::Asm,
    policy: Interruption,
    scratch: Gpr,
    far: bool,
) -> Option<crate::x86_64::asm::Patch> {
    let branch = |asm: &mut crate::x86_64::asm::Asm| match far {
        true => asm.jcc(Cond::Ne),
        false => asm.jcc8(Cond::Ne),
    };
    match policy {
        Interruption::Epoch => {
            // Aligned MOV is an acquire load on x86. The owner never rewrites
            // this word; a notifier only advances it after publishing sources.
            asm.mov_load(Qword, scratch, Mem::base(Rcx, INTERRUPT_EPOCH));
            asm.alu_load(
                Alu::Cmp,
                Qword,
                scratch,
                Mem::base(Rcx, OBSERVED_INTERRUPT_EPOCH),
            );
            Some(branch(asm))
        }
        #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
        Interruption::Flag => {
            asm.alu_imm8(Alu::Cmp, Qword, Mem::base(Rcx, INTERRUPT_EPOCH), 0);
            Some(branch(asm))
        }
        #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
        Interruption::LockedBudget | Interruption::Legacy => None,
    }
}
fn spend_budget(asm: &mut crate::x86_64::asm::Asm, policy: Interruption) {
    #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
    if policy == Interruption::LockedBudget {
        asm.lock();
    }
    let _ = policy;
    // This local counter has no foreign writer in the epoch protocol.
    asm.alu_imm8(Alu::Sub, Qword, Mem::base(Rcx, 0), 1);
}
struct ThunkPlan {
    at: u64,
    retired: u32,
    block: u32,
    entry: u64,
    fallback: Fallback,
}

const _: () = assert!(core::mem::size_of::<LookupEntry>() == 32);
const _: () = assert!(core::mem::offset_of!(LookupEntry, pc) == 0);
const ENTRY_KEY: i32 = core::mem::offset_of!(LookupEntry, key) as i32;
const ENTRY_CODE: i32 = core::mem::offset_of!(LookupEntry, entry) as i32;
const ENTRY_BLOCK: i32 = core::mem::offset_of!(LookupEntry, block) as i32;
const ENTRY_TARGETS: i32 = core::mem::offset_of!(LookupEntry, targets) as i32;

/// A direct exit to `target` with `kind` as a block emits it: MOVABS RAX
/// with the target, then the dynamic exit, unpadded. Code that ends up at
/// an address its patch point does not suit is aligned before it runs
/// ([`align_direct_exit`]); [`is_direct_exit`] accepts every padding.
pub fn direct_exit(target: GuestAddress, kind: impl Into<ExitWord>) -> [u8; DIRECT_EXIT_BYTES] {
    direct_exit_padded(target, kind.into().encoded(), 0)
}

/// [`direct_exit`] for host address `at`, its patch point
/// ([`direct_patch`]) patchable there.
pub fn direct_exit_at(
    at: u64,
    target: GuestAddress,
    kind: impl Into<ExitWord>,
) -> [u8; DIRECT_EXIT_BYTES] {
    direct_exit_padded(target, kind.into().encoded(), patch_pad(at + 15))
}

fn direct_exit_padded(target: GuestAddress, word: u32, pad: usize) -> [u8; DIRECT_EXIT_BYTES] {
    const MOVABS_RAX: [u8; 2] = [0x48, 0xb8];
    let mut bytes = [0; DIRECT_EXIT_BYTES];
    bytes[..2].copy_from_slice(&MOVABS_RAX);
    bytes[2..10].copy_from_slice(&target.0.to_le_bytes());
    bytes[10..].copy_from_slice(&exit_word_bytes(word, pad));
    bytes
}

/// MOV EDX with `word`, a NOP of `pad` bytes, then the patch point: RET
/// and the room a jump takes, INT3s to the exit's end
/// ([`super::block::exit_kind_with`]).
fn exit_word_bytes(word: u32, pad: usize) -> [u8; DYNAMIC_EXIT_BYTES] {
    const MOV_EDX: u8 = 0xba;
    let mut bytes = [0xcc; DYNAMIC_EXIT_BYTES];
    bytes[0] = MOV_EDX;
    bytes[1..5].copy_from_slice(&word.to_le_bytes());
    bytes[5..5 + pad].copy_from_slice(NOPS[pad]);
    bytes[5 + pad..5 + pad + PATCH_BYTES].copy_from_slice(&RET_ROOM);
    bytes
}

/// The NOP of each length up to [`MAX_PAD`] (Intel SDM vol. 2, NOP).
const NOPS: [&[u8]; MAX_PAD + 1] = [
    &[],
    &[0x90],
    &[0x66, 0x90],
    &[0x0f, 0x1f, 0x00],
    &[0x0f, 0x1f, 0x40, 0x00],
];

/// Whether a [`PATCH_BYTES`] instruction at host address `point` lies in
/// one aligned 8-byte word, so that one aligned 8-byte store replaces it
/// whole: what a live patch needs, as another core may be running it.
pub const fn patchable(point: u64) -> bool {
    point % 8 <= (8 - PATCH_BYTES) as u64
}

/// The NOP bytes that make the instruction after them at `at` patchable.
pub const fn patch_pad(at: u64) -> usize {
    match (at % 8) as usize {
        offset if offset <= 8 - PATCH_BYTES => 0,
        offset => 8 - offset,
    }
}

/// `word`, the aligned 8-byte word that holds the patch point `point`
/// now, with `instruction` at the point instead: what a host stores, at
/// once, to patch live code.
pub fn patch_word(word: u64, point: u64, instruction: [u8; PATCH_BYTES]) -> u64 {
    debug_assert!(patchable(point));
    let mut bytes = word.to_le_bytes();
    let at = (point % 8) as usize;
    bytes[at..at + PATCH_BYTES].copy_from_slice(&instruction);
    u64::from_le_bytes(bytes)
}

/// The patch point of the dynamic exit at host address `at`, aligned for
/// it: its `ret`, which a lookup's jump replaces.
pub const fn dynamic_patch(at: u64) -> u64 {
    at + 5 + patch_pad(at + 5) as u64
}

/// The patch point of the direct exit at host address `at`, aligned for it.
pub const fn direct_patch(at: u64) -> u64 {
    dynamic_patch(at + 10)
}

/// The link slot of `bytes` at host address `at`'s patch point: the jump
/// over its body to the exit, the NOP into the body, or a jump to a thunk.
pub const fn slot_patch(at: u64) -> u64 {
    at + patch_pad(at) as u64
}

/// Whether the direct exit at `offset` of `code`, which runs from host
/// address `at`, has its patch point where a link patches it: its slot's
/// point jumping to it when `slot` is nonzero, else its own `ret`.
pub fn exit_ready(code: &[u8], offset: usize, at: u64, slot: usize) -> bool {
    let site = at + offset as u64;
    match slot {
        0 => point_holds(code, at, direct_patch(site), RET_ROOM),
        slot => {
            let point = slot_patch(site - slot as u64);
            jump(point, site).is_some_and(|jump| point_holds(code, at, point, jump))
        }
    }
}

/// Whether the dynamic exit at `offset` of `code`, which runs from host
/// address `at`, has its patch point's `ret` where a lookup patches it.
pub fn dynamic_ready(code: &[u8], offset: usize, at: u64) -> bool {
    point_holds(code, at, dynamic_patch(at + offset as u64), RET_ROOM)
}

/// Whether `code`, from host address `at`, holds `instruction` at the
/// patchable `point`.
fn point_holds(code: &[u8], at: u64, point: u64, instruction: [u8; PATCH_BYTES]) -> bool {
    let offset = point.wrapping_sub(at) as usize;
    patchable(point)
        && code
            .get(offset..offset.saturating_add(PATCH_BYTES))
            .is_some_and(|bytes| *bytes == instruction)
}

/// The encoded word of the dynamic exit `bytes` is, of any padding.
fn exit_word_of(bytes: &[u8]) -> Option<u32> {
    let word = u32::from_le_bytes(bytes.get(1..5)?.try_into().ok()?);
    (0..=MAX_PAD)
        .any(|pad| bytes == exit_word_bytes(word, pad))
        .then_some(word)
}

/// Whether `bytes` are the direct exit to `target` of `kind`, of any
/// padding.
pub fn is_direct_exit(bytes: &[u8], target: GuestAddress, kind: impl Into<ExitWord>) -> bool {
    let word = kind.into().encoded();
    (0..=MAX_PAD).any(|pad| bytes == direct_exit_padded(target, word, pad))
}

/// Whether `bytes` are the dynamic exit of `kind`, of any padding.
pub fn is_dynamic_exit(bytes: &[u8], kind: ExitKind) -> bool {
    exit_word_of(bytes) == Some(ExitWord::from(kind).encoded())
}

/// Pad the direct exit at `offset` of `code`, which runs at host address
/// `at + offset`, for its patch point there. Whether it is one.
pub fn align_direct_exit(code: &mut [u8], offset: usize, at: u64) -> bool {
    let Some(bytes) = code.get_mut(offset..offset + DIRECT_EXIT_BYTES) else {
        return false;
    };
    let Some(word) = exit_word_of(&bytes[10..]) else {
        return false;
    };
    if bytes[..2] != [0x48, 0xb8] {
        return false;
    }
    let pad = patch_pad(at + offset as u64 + 15);
    bytes[10..].copy_from_slice(&exit_word_bytes(word, pad));
    true
}

/// Pad the dynamic exit at `offset` of `code`, which runs at host address
/// `at + offset`, for its patch point there. Whether it is one.
pub fn align_dynamic_exit(code: &mut [u8], offset: usize, at: u64) -> bool {
    let Some(bytes) = code.get_mut(offset..offset + DYNAMIC_EXIT_BYTES) else {
        return false;
    };
    let Some(word) = exit_word_of(bytes) else {
        return false;
    };
    bytes.copy_from_slice(&exit_word_bytes(word, patch_pad(at + offset as u64 + 5)));
    true
}

/// Make the empty link slot of `bytes` at `offset` of `code`, which runs
/// at host address `at + offset`, the one for there.
pub fn align_link_slot(code: &mut [u8], offset: usize, bytes: usize, at: u64) -> bool {
    let Some(slot) = code.get_mut(offset..offset + bytes) else {
        return false;
    };
    if !(0..8).any(|residue| *slot == empty_link_at(residue, bytes)) {
        return false;
    }
    let empty = empty_link_at(at + offset as u64, bytes);
    slot.copy_from_slice(&empty);
    true
}

/// The bytes the direct exit of `kind` to `target` keeps before it for a
/// [`link`], in a block at `block` whose code keeps them. A transfer within
/// the page runs there, in the block's own code, for what a thunk apart would
/// cost a kernel's cold code in fetches; one elsewhere takes a lookup dispatcher.
/// Only a backward one looks for an interruption: every other way round a
/// loop is through a lookup dispatcher.
pub fn link_slot(block: Option<u64>, target: GuestAddress, kind: ExitKind) -> usize {
    match block {
        Some(block)
            if matches!(
                kind,
                ExitKind::Fallthrough | ExitKind::Direct | ExitKind::Call
            ) && (block ^ target.0) & !(LINK_PAGE - 1) == 0 =>
        {
            if target.0 <= block {
                POLLED_LINK_BYTES
            } else {
                LINK_BYTES
            }
        }
        _ => 0,
    }
}

/// The link slot of an unlinked exit, as a block emits it unpadded: its
/// patch point jumps over the room for the body to the exit.
pub(super) fn empty_link(asm: &mut crate::x86_64::asm::Asm, bytes: usize) {
    if bytes != 0 {
        let exit = asm.jmp();
        for _ in PATCH_BYTES..bytes {
            asm.int3();
        }
        asm.bind(exit);
    }
}

/// The empty link slot of `bytes` at host address `at`: the pad, then the
/// patch point's jump to the exit after the slot, then INT3s.
fn empty_link_at(at: u64, bytes: usize) -> Vec<u8> {
    let pad = patch_pad(at);
    let mut slot = alloc::vec![0xcc; bytes];
    slot[..pad].copy_from_slice(NOPS[pad]);
    let jump = jump(at + pad as u64, at + bytes as u64).expect("an exit right after its slot");
    slot[pad..pad + PATCH_BYTES].copy_from_slice(&jump);
    slot
}

/// The patch that undoes the link of the direct exit at host address
/// `site` with a link slot of `slot` bytes before it, or without one: its
/// patch point and the instruction to put there. A slot's point jumps to
/// the exit again; an exit's own point returns to the engine.
pub fn unlinked(site: u64, slot: usize) -> Option<(u64, [u8; PATCH_BYTES])> {
    if slot != 0 {
        let point = slot_patch(site - slot as u64);
        Some((point, jump(point, site)?))
    } else {
        Some((direct_patch(site), RET_ROOM))
    }
}

/// The body of the link in the slot of `bytes` at host address `at`, and
/// where it goes: right after the slot's patch point, which then becomes
/// [`NOP5`] to fall into it. It chains a block of `retired` instructions
/// into block `block`, whose code is at `entry`, or goes to the exit after
/// the slot when the control allows no transfer. `None` when `entry` is out
/// of a 32-bit jump's reach, an operand exceeds its 32-bit field or `bytes`
/// is no slot.
pub fn link(at: u64, bytes: usize, retired: u32, block: u32, entry: u64) -> Option<(u64, Vec<u8>)> {
    let polled = match bytes {
        LINK_BYTES => false,
        POLLED_LINK_BYTES => true,
        _ => return None,
    };
    let body = slot_patch(at) + PATCH_BYTES as u64;
    let exit = at + bytes as u64;
    let code = link_body(
        body,
        polled,
        retired,
        block,
        entry,
        Fall::After(exit),
        Interruption::Epoch,
    )?;
    debug_assert!(body + code.len() as u64 <= exit);
    Some((body, code))
}

/// Where a link's body goes when the control allows no transfer.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fallback {
    /// Return to the engine with this word in EDX, which the gate may have
    /// clobbered: the exit has loaded RAX already, as for a thunk its patch
    /// point jumps to.
    Return(ExitWord),
    /// Jump to the exit at this host address, as for a link slot.
    Jump(u64),
}

/// A thunk at host address `at` that chains a block of `retired`
/// instructions into block `block`, whose code is at `entry`, or takes
/// `fallback` when the control allows no transfer: what an exit's patch
/// point jumps to when its slot's body is used or it has none. It always
/// looks for an interruption. `None` when `entry` or the fallback is out of
/// a 32-bit jump's reach or an operand exceeds its 32-bit field.
pub fn thunk(at: u64, retired: u32, block: u32, entry: u64, fallback: Fallback) -> Option<Vec<u8>> {
    thunk_with_policy(
        ThunkPlan {
            at,
            retired,
            block,
            entry,
            fallback,
        },
        Interruption::Epoch,
    )
}
fn thunk_with_policy(plan: ThunkPlan, policy: Interruption) -> Option<Vec<u8>> {
    let ThunkPlan {
        at,
        retired,
        block,
        entry,
        fallback,
    } = plan;
    link_body(
        at,
        true,
        retired,
        block,
        entry,
        Fall::Thunk(fallback),
        policy,
    )
}

/// Where a link's body goes when the control allows no transfer.
#[derive(Debug, Clone, Copy)]
enum Fall {
    /// A thunk's [`Fallback`], after its body.
    Thunk(Fallback),
    /// A slot's: the exit at this host address, a few bytes after the body.
    After(u64),
}

/// The code of a link at host address `at`: count, name and jump, or
/// `fall`. Only a `polled` one has the interruption gate.
fn link_body(
    at: u64,
    polled: bool,
    retired: u32,
    block: u32,
    entry: u64,
    fall: Fall,
    policy: Interruption,
) -> Option<Vec<u8>> {
    let retired = i32::try_from(retired).ok()?;
    let block = i32::try_from(block).ok()?;
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let interrupted = polled
        .then(|| interruption_gate(&mut asm, policy, Rdx))
        .flatten();
    spend_budget(&mut asm, policy);
    let exhausted = asm.jcc8(Cond::B);
    asm.alu_imm32(Alu::Add, Qword, Mem::base(Rcx, RETIRED), retired);
    asm.mov_rm_imm32(Qword, Mem::base(Rcx, BLOCK), block);
    let transfer = asm.jmp();
    let distance = relative(at + transfer.end() as u64, entry)?;
    let body = code.len();
    debug_assert_eq!(
        body,
        LINK_BODY_BYTES + usize::from(polled) * policy.extra_bytes()
    );
    let fallen = match fall {
        Fall::Thunk(Fallback::Return(word)) => {
            let mut asm = code.asm();
            asm.mov_imm32(Rdx, word.encoded());
            asm.ret();
            body
        }
        Fall::Thunk(Fallback::Jump(exit)) => {
            let back = code.asm().jmp();
            let back_distance = relative(at + back.end() as u64, exit)?;
            code.patch(back.offset(), back_distance.to_le_bytes());
            body
        }
        // Past the body's end: the slot's INT3s up to the exit stay.
        Fall::After(exit) => usize::try_from(exit.checked_sub(at)?).ok()?,
    };
    for patch in core::iter::once(exhausted).chain(interrupted) {
        crate::x86_64::asm::resolve(&mut code, patch, fallen);
    }
    code.patch(transfer.offset(), distance.to_le_bytes());
    Some(code.into_bytes())
}

/// Bytes of the cell probe of a [`lookup_dispatcher`], which comes first.
pub const CELL_PROBE_BYTES: usize = 384;
/// Where a [`lookup_dispatcher`]'s cell probe begins.
pub const CELL_PROBE_ENTRY: usize = 0;
/// Where a [`lookup_dispatcher`]'s table probe begins: after the cell
/// probe.
pub const PROBE_ENTRY: usize = CELL_PROBE_BYTES;
/// Where a [`lookup_dispatcher`]'s stamping cell probe begins, after the
/// table probe: a [`stamped_tail`]'s miss takes it.
pub const STAMPING_PROBE_ENTRY: usize = PROBE_ENTRY + 512;
/// Where a [`lookup_dispatcher`]'s jump cache probe begins, after the
/// stamping cell probe: every [`lookup_tail`] of its BTYPE calls it.
pub const JUMP_PROBE_ENTRY: usize = STAMPING_PROBE_ENTRY + 512;
/// Bytes in a [`lookup_tail`]: `mov edx, word` and `jmp probe`.
pub const TAIL_BYTES: usize = 5 + 5;
/// Bytes in a [`cell_tail`].
pub const CELL_TAIL_BYTES: usize = 10 + TAIL_BYTES;

/// The probes the per-site tails of the exits of branches that set `btype`
/// jump to ([`lookup_tail`]): the target in RAX, the tail's word in EDX.
/// `None` when `btype` is not a BTYPE.
///
/// Both translate the target through the ITLB first: the entry of its page
/// in the current context of its half gives the block key, and without one
/// the exit returns. The table probe ([`PROBE_ENTRY`]) then finds the
/// target's entry of that key in the block table; the cell probe
/// ([`CELL_PROBE_ENTRY`]) the one entry of a [`LookupTable::link_cell`] cell
/// in R9, and the table when the cell does not hold the target with that
/// key. Each enters the block it found, the transfer counted and `block`
/// named, or returns to the engine with RAX and RDX as the exit left them.
/// Neither touches the stack: R10 and R11, free between blocks, hold what
/// a probe keeps. A lookup tail calls the jump cache probe
/// ([`JUMP_PROBE_ENTRY`]), which returns a hit to the tail and drops the
/// tail's return for anything else.
/// Other cores write the table and cells meanwhile: a probe takes an entry
/// only when its last word, read before its fields, was not busy and is
/// unchanged after them ([`Slot`]); a cell read torn starts the table
/// probe over, a table entry read torn leaves the target to the engine. A tail is a few bytes
/// of each exit's own ([`lookup_tail`], [`cell_tail`]); the probe's work,
/// and its indirect jump into the found block, are shared: the host's
/// indirect predictor tells sites apart by the path into it.
pub fn lookup_dispatcher(btype: u8) -> Option<Vec<u8>> {
    dispatcher_with_policy(btype, Interruption::Epoch)
}
fn dispatcher_with_policy(btype: u8, policy: Interruption) -> Option<Vec<u8>> {
    if btype > 3 {
        return None;
    }
    let mut code = Code::new();
    let (to_table, to_probe) = cell_probe(&mut code, policy, false, btype);
    let padding = CELL_PROBE_BYTES
        .checked_sub(code.len())
        .expect("the cell probe fits its bytes");
    for _ in 0..padding {
        code.asm().int3();
    }
    debug_assert_eq!(code.len(), PROBE_ENTRY);
    // The table probe.
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    if let Some(interrupted) = far_interruption_gate(&mut asm, policy, R9) {
        misses.push(interrupted);
    }
    translate(&mut asm, &mut misses);
    let table = code.len();
    let mut asm = code.asm();
    // R10 = the target's entry of the key in R8.
    asm.mov(Dword, R10, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R10, LOOKUP_BITS as u8);
    asm.alu(Alu::Xor, Dword, R10, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R10, 2);
    asm.alu(Alu::Xor, Dword, R10, R8);
    asm.alu_load(Alu::And, Dword, R10, Mem::base(Rcx, MASKS));
    asm.shift_imm(ShiftOp::Shl, Dword, R10, 5);
    asm.alu_load(Alu::Add, Qword, R10, Mem::base(Rcx, TABLE));
    // R11D = each entry's last word before its fields ([`Slot`]).
    asm.mov_load(Dword, R11, Mem::base(R10, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R10, 0));
    let other_pc = asm.jcc8(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R10, ENTRY_KEY));
    let hit = asm.jcc8(Cond::E);
    asm.bind(other_pc);
    asm.alu_imm8(
        Alu::Xor,
        Qword,
        R10,
        core::mem::size_of::<LookupEntry>() as i8,
    );
    asm.mov_load(Dword, R11, Mem::base(R10, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R10, 0));
    misses.push(asm.jcc(Cond::Ne));
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R10, ENTRY_KEY));
    misses.push(asm.jcc(Cond::Ne));
    asm.bind(hit);
    if btype != 0 {
        asm.test_imm(Size::Byte, R11, 1 << btype);
        misses.push(asm.jcc(Cond::E));
    }
    asm.mov_load(Dword, R8, Mem::base(R10, ENTRY_BLOCK));
    asm.mov_load(Qword, R9, Mem::base(R10, ENTRY_CODE));
    misses.extend(seqlock_check(&mut asm, R10));
    spend_budget(&mut asm, policy);
    misses.push(asm.jcc(Cond::B));
    asm.shift_imm(ShiftOp::Shr, Dword, Rdx, ExitWord::RAN_SHIFT as u8);
    asm.alu(Alu::Add, Qword, Mem::base(Rcx, RETIRED), Rdx);
    asm.alu_imm8(Alu::Add, Qword, Mem::base(Rcx, HITS), 1);
    fill_jump_cache(&mut asm);
    entered_btype(&mut asm, R8, btype);
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    found(&mut asm);
    for miss in misses {
        asm.bind(miss);
    }
    missed(&mut asm);
    crate::x86_64::asm::resolve(&mut code, to_table, table);
    crate::x86_64::asm::resolve(&mut code, to_probe, PROBE_ENTRY);
    // The stamping cell probe, which a stamped tail's miss takes.
    let padding = STAMPING_PROBE_ENTRY
        .checked_sub(code.len())
        .expect("the table probe fits its bytes");
    for _ in 0..padding {
        code.asm().int3();
    }
    let (to_table, to_probe) = cell_probe(&mut code, policy, true, btype);
    crate::x86_64::asm::resolve(&mut code, to_table, table);
    crate::x86_64::asm::resolve(&mut code, to_probe, PROBE_ENTRY);
    let padding = JUMP_PROBE_ENTRY
        .checked_sub(code.len())
        .expect("the stamping cell probe fits its bytes");
    for _ in 0..padding {
        code.asm().int3();
    }
    let to_probe = jump_probe(&mut code, policy, btype);
    crate::x86_64::asm::resolve(&mut code, to_probe, PROBE_ENTRY);
    Some(code.into_bytes())
}

/// The jump cache probe at the end of `code` that every [`lookup_tail`] of
/// a branch that sets `btype` calls, the target in RAX and the tail's word
/// in EDX: a target this PE's jump cache holds in its current serial (and
/// that accepts `btype`) is counted as a probe counts its transfers, named
/// in `block`, and returned to the tail in R9, whose own jump enters it. A
/// spent budget drops the tail's return and returns to the engine, EDX
/// whole; any other target drops it and takes the table probe, whose jump
/// the patch returned leads to. The call and its return pair, so the host
/// predicts the return, and each site keeps its own indirect jump.
fn jump_probe(code: &mut Code, policy: Interruption, btype: u8) -> crate::x86_64::asm::Patch {
    let entry = |field: i32| Mem::indexed(R10, R9, 8, JUMP_CACHE + field);
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses: Vec<_> = gate(&mut asm, policy, R8, false).into_iter().collect();
    // R10 = this PE's stamps, R9 = 8 times the entry's index.
    asm.mov_load(Qword, R10, Mem::base(Rcx, STAMPS));
    asm.mov(Dword, R9, Rax);
    asm.alu_imm32(Alu::And, Dword, R9, JUMP_INDEX);
    asm.alu_load(Alu::Cmp, Qword, Rax, entry(JUMP_PC));
    misses.push(asm.jcc8(Cond::Ne));
    asm.mov_load(Qword, R8, Mem::base(R10, JUMP_SERIAL));
    asm.alu_load(Alu::Cmp, Qword, R8, entry(JUMP_ENTRY_SERIAL));
    misses.push(asm.jcc8(Cond::Ne));
    if btype != 0 {
        asm.test_imm(Size::Byte, entry(JUMP_TARGETS), 1 << btype);
        misses.push(asm.jcc8(Cond::E));
    }
    spend_budget(&mut asm, policy);
    let exhausted = asm.jcc8(Cond::B);
    asm.mov(Dword, R8, Rdx);
    asm.shift_imm(ShiftOp::Shr, Dword, R8, ExitWord::RAN_SHIFT as u8);
    asm.alu(Alu::Add, Qword, Mem::base(Rcx, RETIRED), R8);
    asm.alu_imm8(Alu::Add, Qword, Mem::base(Rcx, HITS), 1);
    asm.mov_load(Dword, R8, entry(JUMP_BLOCK));
    entered_btype(&mut asm, R8, btype);
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    asm.jmp_rm(entry(JUMP_CODE));
    // A spent budget returns as the probe's would, its word in EDX whole.
    asm.bind(exhausted);
    missed(&mut asm);
    for miss in misses {
        asm.bind(miss);
    }
    asm.jmp()
}

/// The BTYPE bits of `block` ([`BLOCK_BTYPE_SHIFT`]) set in `register`,
/// which holds the entered block's index, for a transfer that sets `btype`.
fn entered_btype(asm: &mut crate::x86_64::asm::Asm, register: Gpr, btype: u8) {
    for bit in 0..2 {
        if btype & 1 << bit != 0 {
            asm.bit_imm(BitTest::Bts, Qword, register, BLOCK_BTYPE_SHIFT as u8 + bit);
        }
    }
}

/// A cell probe at the end of `code`, R9 at the cell (R10 keeps it across
/// the translation); a `stamping` one stamps the cell when it enters its
/// block ([`stamp`]). Every one counts the transfers through a stamped
/// cell in its `heat` word without a branch: the host's indirect predictor
/// tells the sites of the shared jump apart by the branches since each
/// tail's jump, and one more costs a misprediction a transfer. The jumps to
/// the table probe with the key in R8, and to its start for a torn read.
/// A probe of a nonzero `btype` (the exits of [`predicted_tail`]s of BR and
/// BLR forms) takes the cell's entry only when it accepts that BTYPE, and
/// names it in `block`, as the table probe does; BTYPE 0 adds nothing.
fn cell_probe(
    code: &mut Code,
    policy: Interruption,
    stamping: bool,
    btype: u8,
) -> (crate::x86_64::asm::Patch, crate::x86_64::asm::Patch) {
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    if let Some(interrupted) = far_interruption_gate(&mut asm, policy, R8) {
        misses.push(interrupted);
    }
    asm.mov(Qword, R10, R9);
    translate(&mut asm, &mut misses);
    // The cell is the target's with its page's key, or the table decides.
    // R11D = the cell's last word before its fields ([`Slot`]).
    asm.mov_load(Dword, R11, Mem::base(R10, ENTRY_TARGETS));
    asm.alu_load(Alu::Cmp, Qword, Rax, Mem::base(R10, 0));
    let stale = asm.jcc(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R10, ENTRY_KEY));
    let other = asm.jcc(Cond::Ne);
    if btype != 0 {
        asm.test_imm(Size::Byte, R11, 1 << btype);
        misses.push(asm.jcc(Cond::E));
    }
    asm.mov_load(Dword, R8, Mem::base(R10, ENTRY_BLOCK));
    asm.mov_load(Qword, R9, Mem::base(R10, ENTRY_CODE));
    let torn = seqlock_check(&mut asm, R10);
    // R11 = the cell's `heat` word, 4 bytes a cell; a cell past the
    // stamped ones, or none (`ChainControl::cells` 0), counts in the
    // control's `heat`.
    asm.mov(Qword, R11, R10);
    asm.alu_load(Alu::Sub, Qword, R11, Mem::base(Rcx, CELLS));
    asm.shift_imm(ShiftOp::Shr, Qword, R11, 3);
    asm.alu_imm32(Alu::Cmp, Qword, R11, (CELL_LIVE / 8) as i32);
    asm.mov_load(Qword, R10, Mem::base(Rcx, CELLS));
    asm.lea(Qword, R11, Mem::indexed(R10, R11, 1, CELL_HEAT as i32));
    asm.lea(Qword, R10, Mem::base(Rcx, HEAT));
    asm.cmov(Cond::Ae, Qword, R11, R10);
    asm.alu_imm8(Alu::Add, Dword, Mem::base(R11, 0), 1);
    spend_budget(&mut asm, policy);
    misses.push(asm.jcc(Cond::B));
    asm.shift_imm(ShiftOp::Shr, Dword, Rdx, ExitWord::RAN_SHIFT as u8);
    asm.alu(Alu::Add, Qword, Mem::base(Rcx, RETIRED), Rdx);
    entered_btype(&mut asm, R8, btype);
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    if stamping {
        stamp(&mut asm);
    }
    found(&mut asm);
    for miss in misses {
        asm.bind(miss);
    }
    missed(&mut asm);
    // A cell written while it was read: R8 holds the cell's block by now,
    // so the table probe starts over from the target in RAX.
    for miss in torn {
        asm.bind(miss);
    }
    let to_probe = asm.jmp();
    // Another PC or key leaves the target to the table, with the key still
    // in R8 and the control in RCX.
    asm.bind(stale);
    asm.bind(other);
    let to_table = asm.jmp();
    (to_table, to_probe)
}

/// A cell probe's stamp, after its transfer is counted, of the stamped cell
/// whose words R11 points at less CELL_LIVE (R11 = 0: none): this PE's
/// context of the target's half and its ITLB's drop epoch there, while the
/// cell still holds the entry it was linked with ([`stamped_tail`]). The
/// epoch goes in by a locked exchange, and the `live` word is read again
/// after it: a writer that cleared it meanwhile also zeroes the stamp, or
/// this does. It counts the stamps it writes and the cells it finds
/// changed ([`StampCount`]). Clobbers RDX, R8, R10, R11.
fn stamp(asm: &mut crate::x86_64::asm::Asm) {
    // R11 back to the cell's words less CELL_LIVE, or the control's heat.
    asm.lea(Qword, R11, Mem::base(R11, -(CELL_HEAT as i32)));
    asm.mov(Qword, R10, R11);
    asm.alu_load(Alu::Sub, Qword, R10, Mem::base(Rcx, CELLS));
    asm.alu_imm32(Alu::Cmp, Qword, R10, (CELL_LIVE / 8) as i32);
    let none = asm.jcc8(Cond::Ae);
    asm.mov_load(Qword, R8, Mem::base(Rcx, STAMPS));
    asm.test(Qword, R8, R8);
    let unstamped = asm.jcc8(Cond::E);
    asm.alu_imm8(Alu::Cmp, Dword, Mem::base(R11, CELL_LIVE as i32), 0);
    let live = asm.jcc8(Cond::Ne);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R8, StampCount::Changed.offset()),
        1,
    );
    let changed = asm.jmp8();
    asm.bind(live);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R8, StampCount::Restamped.offset()),
        1,
    );
    // RDX = this PE's stamp of the cell.
    asm.mov(Qword, Rdx, R11);
    asm.alu_load(Alu::Sub, Qword, Rdx, Mem::base(Rcx, CELLS));
    asm.shift_imm(ShiftOp::Shl, Qword, Rdx, 2);
    asm.alu(Alu::Add, Qword, Rdx, R8);
    asm.mov(Qword, R8, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R8, 55);
    asm.alu_imm8(Alu::And, Dword, R8, 1);
    asm.mov_load(Qword, R10, Mem::indexed(Rcx, R8, 8, CONTEXTS));
    asm.mov_store(Qword, Mem::base(Rdx, 0), R10);
    asm.mov_load(Qword, R10, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R10, Mem::indexed(R10, R8, 8, ITLB_DROPS));
    // xchg [rdx + 8], r10
    asm.fixed(&[0x4c, 0x87, 0x52, 0x08]);
    asm.alu_imm8(Alu::Cmp, Dword, Mem::base(R11, CELL_LIVE as i32), 0);
    let kept = asm.jcc8(Cond::Ne);
    asm.mov_rm_imm32(Qword, Mem::base(Rdx, 8), 0);
    asm.bind(none);
    asm.bind(changed);
    asm.bind(unstamped);
    asm.bind(kept);
}

/// Bytes in a [`stamped_tail`]: 130 at most, with a 32-bit displacement
/// for the stamp of a cell numbered 8 or above.
pub const STAMPED_TAIL_BYTES: usize = 144;

/// The tail at host address `at` of a hot direct exit to `target` out of
/// its page, whose stamped cell `number` at `cell` held `block`, entered at
/// `entry`, when it was linked: while this PE's stamp of the cell names the
/// context it runs in and its ITLB's drop epoch of the target's half, the
/// transfer is counted, `block` named and `entry` entered by a direct jump,
/// without a translation or a probe; else, as for an interruption, it is
/// the [`cell_tail`] after it, whose cell probe stamps the cell when it
/// enters its block. A budget spent returns with `word`, as a probe would.
///
/// A stamp exists only while the cell holds what it held at the link
/// (`live`, [`put_cell`] clears it and zeroes every PE's stamps first), and
/// it names the context and drop epoch in which this PE's probe found the
/// target's page translating to that entry's key; any ITLB entry of the
/// half leaving or changing on this PE grows the epoch ([`ITLB_DROPS`]),
/// and context numbers are never reused. `None` as for a [`cell_tail`], or
/// for a cell that is not stamped.
#[allow(clippy::too_many_arguments)]
pub fn stamped_tail(
    at: u64,
    cell: u64,
    number: u32,
    target: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    block: u32,
    entry: u64,
    probe: u64,
) -> Option<[u8; STAMPED_TAIL_BYTES]> {
    stamped_with_policy(
        at,
        cell,
        number,
        target,
        word.into(),
        ran,
        block,
        entry,
        probe,
        0,
        Interruption::Epoch,
    )
}

/// A [`stamped_tail`] for the direct exit of a [`predicted_tail`] of a
/// branch that sets `btype`: its hit names the BTYPE in `block` as the
/// table probe does, and `probe` is the stamping cell probe of the
/// dispatcher of `btype`, which takes only entries that accept it. The
/// linker gives one only for a `block` that accepts `btype`: a stamp lives
/// only while the cell holds that block. `None` as for a [`stamped_tail`],
/// or for a `btype` that is not a BTYPE.
#[allow(clippy::too_many_arguments)]
pub fn stamped_branch_tail(
    at: u64,
    cell: u64,
    number: u32,
    target: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    block: u32,
    entry: u64,
    probe: u64,
    btype: u8,
) -> Option<[u8; STAMPED_TAIL_BYTES]> {
    if btype > 3 {
        return None;
    }
    stamped_with_policy(
        at,
        cell,
        number,
        target,
        word.into(),
        ran,
        block,
        entry,
        probe,
        btype,
        Interruption::Epoch,
    )
}

#[allow(clippy::too_many_arguments)]
fn stamped_with_policy(
    at: u64,
    cell: u64,
    number: u32,
    target: u64,
    word: ExitWord,
    ran: u32,
    block: u32,
    entry: u64,
    probe: u64,
    btype: u8,
    policy: Interruption,
) -> Option<[u8; STAMPED_TAIL_BYTES]> {
    if number as usize >= STAMPED_CELLS || ran >= 1 << (u32::BITS - ExitWord::RAN_SHIFT) {
        return None;
    }
    let block = i32::try_from(block).ok()?;
    let half = half(target) as i32;
    let stamp = (STAMP_BYTES * number as usize) as i32;
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    // R10 = this PE's stamps, its counts in front ([`StampCount`]).
    asm.mov_load(Qword, R8, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R9, Mem::base(R8, ITLB_DROPS + 8 * half));
    asm.mov_load(Qword, R10, Mem::base(Rcx, STAMPS));
    asm.alu_load(Alu::Cmp, Qword, R9, Mem::base(R10, stamp + 8));
    let epoch = asm.jcc8(Cond::Ne);
    asm.mov_load(Qword, R9, Mem::base(Rcx, CONTEXTS + 8 * half));
    asm.alu_load(Alu::Cmp, Qword, R9, Mem::base(R10, stamp));
    let context = asm.jcc8(Cond::Ne);
    let interrupted = gate(&mut asm, policy, R8, false);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R10, StampCount::Hits.offset()),
        1,
    );
    spend_budget(&mut asm, policy);
    let exhausted = asm.jcc8(Cond::B);
    asm.alu_imm32(
        Alu::Add,
        Qword,
        Mem::base(Rcx, RETIRED),
        i32::try_from(ran).ok()?,
    );
    if btype == 0 {
        asm.mov_rm_imm32(Qword, Mem::base(Rcx, BLOCK), block);
    } else {
        let named = u64::from(block as u32) | u64::from(btype) << BLOCK_BTYPE_SHIFT;
        asm.mov_imm64(R9, named);
        asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R9);
    }
    let transfer = asm.jmp();
    let distance = relative(at + transfer.end() as u64, entry)?;
    code.patch(transfer.offset(), distance.to_le_bytes());
    let mut asm = code.asm();
    asm.bind(exhausted);
    asm.mov_imm32(Rdx, word.encoded() | ran << ExitWord::RAN_SHIFT);
    asm.ret();
    // A miss counts its cause and takes the cell tail.
    let mut to_tail = Vec::new();
    for (miss, count) in [
        (Some(epoch), StampCount::Epoch),
        (Some(context), StampCount::Context),
        (interrupted, StampCount::Interrupted),
    ] {
        let Some(miss) = miss else {
            continue;
        };
        asm.bind(miss);
        asm.alu_imm8(Alu::Add, Qword, Mem::base(R10, count.offset()), 1);
        to_tail.push(asm.jmp8());
    }
    let tail_at = code.len();
    for patch in to_tail {
        crate::x86_64::asm::resolve(&mut code, patch, tail_at);
    }
    let cell_code = cell_tail(at + tail_at as u64, cell, word, ran, probe)?;
    code.asm().fixed(&cell_code);
    let padding = STAMPED_TAIL_BYTES.checked_sub(code.len())?;
    for _ in 0..padding {
        code.asm().int3();
    }
    code.try_into().ok()
}

/// The check that ends a probe's read of the entry at `entry`, whose last
/// word it read into R11D before the fields: the entry's word is unchanged
/// and no writer was in it ([`Slot`]). The jumps of a torn read.
fn seqlock_check(asm: &mut crate::x86_64::asm::Asm, entry: Gpr) -> [crate::x86_64::asm::Patch; 2] {
    asm.alu_load(Alu::Cmp, Dword, R11, Mem::base(entry, ENTRY_TARGETS));
    let changed = asm.jcc(Cond::Ne);
    asm.test_imm(Dword, R11, META_BUSY as i32);
    let busy = asm.jcc(Cond::Ne);
    [changed, busy]
}

/// Translate the target in RAX through the ITLB, RCX at the control: R8
/// is then the block key of its page in the current context of its half;
/// a page with no entry there takes one of `misses`. Clobbers R9 and R11.
fn translate(asm: &mut crate::x86_64::asm::Asm, misses: &mut Vec<crate::x86_64::asm::Patch>) {
    // R8 = the target's half; R9 = the first slot of its page's ITLB set
    // in that half's context.
    asm.mov(Qword, R8, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R8, 55);
    asm.alu_imm8(Alu::And, Dword, R8, 1);
    asm.mov(Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, ITLB_SET_BITS as u8);
    asm.alu(Alu::Xor, Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, PAGE_SHIFT as u8);
    asm.alu_load(Alu::Xor, Dword, R9, Mem::indexed(Rcx, R8, 4, SALTS));
    asm.alu_load(Alu::And, Dword, R9, Mem::base(Rcx, MASKS + 4));
    asm.shift_imm(
        ShiftOp::Shl,
        Dword,
        R9,
        core::mem::size_of::<[LookupEntry; 2]>().trailing_zeros() as u8,
    );
    asm.alu_load(Alu::Add, Qword, R9, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R8, Mem::indexed(Rcx, R8, 8, CONTEXTS));
    // R11 = the target's page; either entry of the set may hold it.
    asm.mov(Qword, R11, Rax);
    asm.alu_imm32(Alu::And, Qword, R11, -(1 << PAGE_SHIFT));
    asm.alu_load(Alu::Cmp, Qword, R11, Mem::base(R9, 0));
    let other_page = asm.jcc8(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    let hit = asm.jcc8(Cond::E);
    asm.bind(other_page);
    asm.alu_imm8(
        Alu::Xor,
        Qword,
        R9,
        core::mem::size_of::<LookupEntry>() as i8,
    );
    asm.alu_load(Alu::Cmp, Qword, R11, Mem::base(R9, 0));
    misses.push(asm.jcc(Cond::Ne));
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    misses.push(asm.jcc(Cond::Ne));
    asm.bind(hit);
    asm.mov_load(Qword, R8, Mem::base(R9, ENTRY_CODE));
}

/// A probe's transfer, counted: into the block at R9.
fn found(asm: &mut crate::x86_64::asm::Asm) {
    asm.jmp_rm(R9);
}

/// A probe's exit to the engine, with RAX and EDX as the tail left them: the
/// return of the native run, as the tail jumped to the probe.
fn missed(asm: &mut crate::x86_64::asm::Asm) {
    asm.ret();
}

/// Bytes in a [`lookup_tail`]: `mov edx, word`, the call of the jump cache
/// probe and the site's jump, padded.
pub const LOOKUP_TAIL_BYTES: usize = 16;
// The engine installs a lookup tail where a stamped tail may go.
const _: () = assert!(LOOKUP_TAIL_BYTES <= STAMPED_TAIL_BYTES);

/// The tail at host address `at` that an exit's patch point jumps to once
/// the exit is redirected through a lookup, for a branch that sets
/// `btype`: EDX takes the exit's `word` with the `ran` instructions of the
/// block it leaves, then the tail calls the jump cache probe of the
/// dispatcher whose table probe is at `probe` ([`JUMP_PROBE_ENTRY`]). A
/// target this PE's jump cache holds in its current serial (and that
/// accepts `btype`) comes back in R9, counted as a probe counts its
/// transfers, and is entered by a jump of the site's own; any other takes
/// the table probe, which enters the block it finds, filling the cache, or
/// returns to the engine. An interruption or a spent budget leaves as the
/// probe would. `None` when `ran` exceeds its field, `btype` is no BTYPE or
/// the jump cache probe is out of a 32-bit call's reach.
///
/// An entry holds only for the serial it was written in, which moves at
/// every native entry and at every native change of the lookup contexts
/// ([`JUMP_SERIAL`]). In one serial the PE's ITLB and contexts are what
/// the probe that filled the entry translated through, and a block another
/// PE releases meanwhile keeps its code until this PE's grace: the entry
/// enters the block that probe would enter, as a direct link would. The
/// control's stamps hold the cache: every PE's control has them before it
/// runs (`LookupTable::new_stamps`).
pub fn lookup_tail(
    at: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
    btype: u8,
) -> Option<[u8; LOOKUP_TAIL_BYTES]> {
    let word = word.into();
    if btype > 3 || ran >= 1 << (u32::BITS - ExitWord::RAN_SHIFT) {
        return None;
    }
    let jump_probe = probe.checked_add((JUMP_PROBE_ENTRY - PROBE_ENTRY) as u64)?;
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.mov_imm32(Rdx, word.encoded() | ran << ExitWord::RAN_SHIFT);
    let call = asm.jmp();
    let relative = relative(at + call.end() as u64, jump_probe)?;
    code.patch(call.offset(), relative.to_le_bytes());
    let padding = LOOKUP_TAIL_BYTES.checked_sub(code.len())?;
    for _ in 0..padding {
        code.asm().int3();
    }
    code.try_into().ok()
}

/// Count a table probe transfer ([`StampCount::Filled`]) and fill this
/// PE's jump cache entry of the target in RAX ([`lookup_tail`]) with the
/// block the table probe just entered: its code in R9, its index
/// in R8D and its entry's last word in R11D, as the probe read them, whose
/// low byte is the BTYPEs it accepts (`META_TARGETS`), all a tail reads of
/// it; nothing for a control without stamps. Clobbers RDX, R10 and R11.
fn fill_jump_cache(asm: &mut crate::x86_64::asm::Asm) {
    const _: () = assert!(META_TARGETS == 0xff);
    let entry = |field: i32| Mem::indexed(R10, Rdx, 8, JUMP_CACHE + field);
    asm.mov_load(Qword, R10, Mem::base(Rcx, STAMPS));
    asm.test(Qword, R10, R10);
    let none = asm.jcc8(Cond::E);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R10, StampCount::Filled.offset()),
        1,
    );
    asm.mov(Dword, Rdx, Rax);
    asm.alu_imm32(Alu::And, Dword, Rdx, JUMP_INDEX);
    asm.mov_store(Qword, entry(JUMP_PC), Rax);
    asm.mov_store(Qword, entry(JUMP_CODE), R9);
    asm.mov_store(Dword, entry(JUMP_BLOCK), R8);
    asm.mov_store(Dword, entry(JUMP_TARGETS), R11);
    asm.mov_load(Qword, R11, Mem::base(R10, JUMP_SERIAL));
    asm.mov_store(Qword, entry(JUMP_ENTRY_SERIAL), R11);
    asm.bind(none);
}

/// The tail at host address `at` of a direct exit out of its page, whose
/// target the cell at `cell` holds ([`LookupTable::link_cell`]): the cell
/// probe at `probe` ([`CELL_PROBE_ENTRY`]), then the jump or the exit as in
/// a [`lookup_tail`]. `None` as for a [`lookup_tail`].
pub fn cell_tail(
    at: u64,
    cell: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
) -> Option<[u8; CELL_TAIL_BYTES]> {
    let mut code = Code::new();
    code.asm().mov_imm64(R9, cell);
    tail(&mut code, at, word.into(), ran, probe)?;
    code.try_into().ok()
}

/// Where a [`predicted_tail`]'s direct exit to its predicted target begins:
/// its patch point is [`direct_patch`] of the tail's address plus this.
pub const PREDICTED_EXIT: usize = 15;
/// Bytes in a [`predicted_tail`]: the compare, the direct exit and a
/// [`lookup_tail`].
pub const PREDICTED_TAIL_BYTES: usize = PREDICTED_EXIT + DIRECT_EXIT_BYTES + TAIL_BYTES;

/// The tail at host address `at` of an exit to the target in RAX that the
/// engine predicts is `target`: a one-target inline cache. When RAX is
/// `target` it takes a direct exit to `target` of `word`
/// ([`PREDICTED_EXIT`]), which returns RAX and EDX exactly as the
/// redirected exit does and which the engine links like a block's direct
/// exit, through a cell ([`cell_tail`], [`stamped_branch_tail`]); any other
/// target takes the [`lookup_tail`] to `probe` after it. Both leave EDX as
/// the redirected exit or its lookup tail would, so whichever runs, the
/// transfer or exit is the one the lookup tail makes. `None` as for a
/// [`lookup_tail`].
pub fn predicted_tail(
    at: u64,
    target: GuestAddress,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
) -> Option<[u8; PREDICTED_TAIL_BYTES]> {
    let word = word.into();
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.mov_imm64(R8, target.0);
    asm.alu(Alu::Cmp, Qword, Rax, R8);
    let other = asm.jcc8(Cond::Ne);
    debug_assert_eq!(asm.len(), PREDICTED_EXIT);
    let exit = at + PREDICTED_EXIT as u64;
    asm.fixed(&direct_exit_at(exit, target, word));
    asm.bind(other);
    tail(&mut code, at, word, ran, probe)?;
    code.try_into().ok()
}

fn tail(code: &mut Code, at: u64, word: ExitWord, ran: u32, probe: u64) -> Option<()> {
    if ran >= 1 << (u32::BITS - ExitWord::RAN_SHIFT) {
        return None;
    }
    let mut asm = code.asm();
    asm.mov_imm32(Rdx, word.encoded() | ran << ExitWord::RAN_SHIFT);
    let jump = asm.jmp();
    let relative = relative(at + jump.end() as u64, probe)?;
    code.patch(jump.offset(), relative.to_le_bytes());
    Some(())
}

/// An exit, its tail and the probes in one piece, to run without an arena:
/// the cell probe with `cell`, else the table probe.
#[cfg(test)]
fn lookup_thunk(
    ran: u32,
    target: Option<GuestAddress>,
    kind: impl Into<ExitWord>,
    btype: u8,
) -> Option<Vec<u8>> {
    lookup_with_policy(ran, target, kind, btype, None, Interruption::Epoch)
}
/// [`lookup_thunk`] through the cell at `cell` for `target`.
#[cfg(test)]
fn lookup_thunk_with_cell(
    ran: u32,
    target: GuestAddress,
    kind: impl Into<ExitWord>,
    cell: u64,
) -> Option<Vec<u8>> {
    lookup_with_policy(ran, Some(target), kind, 0, Some(cell), Interruption::Epoch)
}
#[cfg(test)]
fn lookup_with_policy(
    ran: u32,
    target: Option<GuestAddress>,
    kind: impl Into<ExitWord>,
    btype: u8,
    cell: Option<u64>,
    policy: Interruption,
) -> Option<Vec<u8>> {
    let mut code = Code::new();
    let mut asm = code.asm();
    if let Some(target) = target {
        asm.mov_imm64(Rax, target.0);
    }
    asm.mov_imm32(Rdx, kind.into().encoded() | ran << ExitWord::RAN_SHIFT);
    if let Some(cell) = cell {
        asm.mov_imm64(R9, cell);
    }
    let call = asm.jmp();
    let probes = code.len();
    let entry = match (cell, target) {
        (Some(_), Some(_)) => CELL_PROBE_ENTRY,
        _ => PROBE_ENTRY,
    };
    crate::x86_64::asm::resolve(&mut code, call, probes + entry);
    let mut bytes = code.into_bytes();
    bytes.extend(dispatcher_with_policy(btype, policy)?);
    Some(bytes)
}

/// The exit to a dynamic target in RAX that blocks end with, unpadded
/// ([`direct_exit`]).
pub fn dynamic_exit(kind: ExitKind) -> [u8; DYNAMIC_EXIT_BYTES] {
    exit_word_bytes(ExitWord::from(kind).encoded(), 0)
}

/// [`dynamic_exit`] for host address `at`, its patch point
/// ([`dynamic_patch`]) patchable there.
pub fn dynamic_exit_at(at: u64, kind: ExitKind) -> [u8; DYNAMIC_EXIT_BYTES] {
    exit_word_bytes(ExitWord::from(kind).encoded(), patch_pad(at + 5))
}

/// `code` without the check failures that end a block, after its exits.
pub fn without_failures(code: &[u8]) -> &[u8] {
    const UD2: [u8; 2] = [0x0f, 0x0b];
    let failures = code
        .rchunks_exact(UD2.len())
        .take_while(|bytes| *bytes == UD2);
    &code[..code.len() - failures.count() * UD2.len()]
}

/// `jmp rel32` at host address `at` to `to`, or `None` out of reach.
pub fn jump(at: u64, to: u64) -> Option<[u8; JUMP_BYTES]> {
    let relative = relative(at + JUMP_BYTES as u64, to)?;
    let mut code = Code::new();
    let field = code.asm().jmp();
    code.patch(field.offset(), relative.to_le_bytes());
    code.try_into().ok()
}

/// The rel32 displacement of a branch ending at host address `end` to
/// `to`, or `None` out of reach.
fn relative(end: u64, to: u64) -> Option<i32> {
    i32::try_from(to.wrapping_sub(end) as i64).ok()
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::x86_64::asm::Code;
    use alloc::vec;

    /// The default block table's slot mask and ITLB's set mask.
    const TABLE_MASK: u32 = LOOKUP_ENTRIES as u32 - 1;
    const ITLB_MASK: u32 = ITLB_ENTRIES as u32 / 2 - 1;

    fn lookup_slot(pc: u64, key: u64) -> usize {
        super::lookup_slot(pc, key, TABLE_MASK)
    }

    fn itlb_slot(pc: u64, context: u64) -> usize {
        super::itlb_slot(pc, context, ITLB_MASK)
    }

    fn itlb_salt(context: u64) -> u32 {
        super::itlb_salt(context, ITLB_MASK)
    }

    /// Tables grow with the arena, in powers of two within their bounds:
    /// a 64 MiB arena keeps the defaults.
    #[test]
    fn tables_are_sized_from_the_arena() {
        const MIB: usize = 1 << 20;
        assert_eq!(lookup_entries(64 * MIB), LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(64 * MIB), ITLB_ENTRIES);
        assert_eq!(lookup_entries(16 * MIB), MIN_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(16 * MIB), MIN_ITLB_ENTRIES);
        assert_eq!(lookup_entries(100 * MIB), LOOKUP_ENTRIES);
        assert_eq!(lookup_entries(512 * MIB), 1 << 21);
        assert_eq!(lookup_entries(64 * 1024 * MIB), MAX_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(128 * MIB), 1 << 15);
        assert_eq!(itlb_entries(256 * MIB), MAX_ITLB_ENTRIES);
        assert_eq!(itlb_entries(720 * MIB), MAX_ITLB_ENTRIES);
        assert_eq!(lookup_entries(0), MIN_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(0), MIN_ITLB_ENTRIES);
    }

    /// A span's ITLB entries go in every context its salts reach, wherever
    /// their sets are, and only theirs; a granule's index survives its
    /// compaction and the entries that moved beside a newer one.
    #[test]
    fn forgetting_a_span_finds_its_pages_in_every_context() {
        const PAGE: u64 = 0xffff_fe00_0123_4000;
        let mut itlb = LookupTable::itlb();
        let put = |itlb: &mut LookupTable, page: u64, context: u64| {
            itlb.insert(
                itlb_slot(page, context),
                LookupEntry::page(page, context, 1),
            );
        };
        let held = |itlb: &LookupTable, page: u64, context: u64| {
            let entry = itlb.find(itlb_slot(page, context), page, context);
            entry.pc == page && entry.key == context
        };
        // Enough pages and contexts in one granule that its list compacts.
        let pages: Vec<u64> = (0..4).map(|n| PAGE + 0x1000 * n).collect();
        for context in 1..=8 {
            for &page in &pages {
                put(&mut itlb, page, context);
            }
        }
        // Another page of the first's slot moves the first beside it.
        let other = PAGE ^ (1 << (PAGE_SHIFT + ITLB_SET_BITS) | 1 << PAGE_SHIFT);
        assert_eq!(itlb_slot(other, 1), itlb_slot(PAGE, 1));
        put(&mut itlb, other, 1);
        assert!(held(&itlb, PAGE, 1) && held(&itlb, other, 1));
        let next = PAGE + 0x4000;
        put(&mut itlb, next, 2);
        // Salt 1 alone: context 1's pages of the granule go.
        assert!(itlb.forget_range(PAGE..=PAGE + 0x3fff, 1 << 1));
        assert!(pages.iter().all(|&page| !held(&itlb, page, 1)));
        assert!(
            pages
                .iter()
                .all(|&page| (2..=8).all(|context| held(&itlb, page, context)))
        );
        assert!(held(&itlb, other, 1) && held(&itlb, next, 2));
        // Every salt: the granule's pages go, the next granule's stay.
        assert!(itlb.forget_range(PAGE..=PAGE + 0x3fff, u64::MAX));
        assert!(
            pages
                .iter()
                .all(|&page| (2..=8).all(|context| !held(&itlb, page, context)))
        );
        assert!(held(&itlb, other, 1) && held(&itlb, next, 2));
        assert!(!itlb.forget_range(PAGE..=PAGE + 0x3fff, u64::MAX));
        assert!(itlb.forget_range(0..=u64::MAX, u64::MAX));
        assert!(itlb.is_empty());
        assert!(itlb.granules.is_empty());
        assert_eq!(itlb.free.len(), itlb.lists.len());
        // A new granule takes a freed list.
        put(&mut itlb, next, 1);
        assert_eq!(itlb.free.len() + 1, itlb.lists.len());
        assert!(itlb.forget_range(next..=next, u64::MAX));
        assert!(!held(&itlb, next, 1));
    }

    /// A group forgotten and indexed again, after its list went to another
    /// group, finds its own list, not the one the front cached.
    #[test]
    fn a_forgotten_group_leaves_the_front() {
        let mut table = LookupTable::new();
        let entry = |pc: u64, frame: u64| LookupEntry {
            pc,
            key: block_key(frame << PAGE_SHIFT, 1),
            entry: 1,
            block: 0,
            targets: 0b1111,
        };
        let put = |table: &mut LookupTable, entry: LookupEntry| {
            table.set(lookup_slot(entry.pc, entry.key), entry);
        };
        let (first, other) = (5, 5 + FRONT_GROUPS as u64);
        put(&mut table, entry(0x1000, first));
        assert!(table.forget_frames(first..=first, |_| true));
        // The freed list goes to a group of the same front slot.
        put(&mut table, entry(0x2000, other));
        put(&mut table, entry(0x3000, first));
        assert!(table.forget_frames(other..=other, |_| true));
        let found = entry(0x3000, first);
        assert_eq!(
            table.find(lookup_slot(found.pc, found.key), found.pc, found.key),
            found
        );
        assert!(table.forget_frames(first..=first, |_| true));
        assert!(table.is_empty());
        assert!(table.granules.is_empty());
    }

    /// Entries of new frames taking the places of others' leave the index
    /// no larger than the table's own entries need, and every entry left is
    /// still found by forgetting its frame.
    #[test]
    fn a_stream_of_new_frames_keeps_the_index_bounded() {
        let mut table = LookupTable::new();
        let frames = 4 * LOOKUP_ENTRIES as u64;
        for frame in 0..frames {
            let pc = 0x1_0000_0000 + (frame << PAGE_SHIFT) + 4 * (frame % 64);
            let key = block_key((0x10_0000 + frame) << PAGE_SHIFT, 1 + (frame % 3) as u32);
            let entry = LookupEntry {
                pc,
                key,
                entry: 1,
                block: 0,
                targets: 1,
            };
            table.insert(lookup_slot(pc, key), entry);
        }
        assert!(table.granules.len() <= 2 * LOOKUP_ENTRIES);
        assert!(
            table.index_bytes() <= LOOKUP_ENTRIES * 160,
            "{}",
            table.index_bytes()
        );
        assert!(table.forget_frames(0..=key_frame(u64::MAX), |_| true));
        assert!(table.is_empty());
        assert!(table.granules.is_empty());
    }

    /// A sweep of the index is taken in bounded steps by the inserts after
    /// the one that began it: no insert looks at every group's list, and
    /// the sweep still ends and drops the groups no set holds.
    #[test]
    fn a_sweep_of_the_index_is_taken_in_bounded_steps() {
        let mut table = LookupTable::with_entries(MIN_LOOKUP_ENTRIES);
        let mask = table.mask;
        let entry = |frame: u64| {
            let pc = 0x1_0000_0000 + (frame << PAGE_SHIFT);
            let key = block_key((0x10_0000 + frame) << PAGE_SHIFT, 1);
            (
                super::lookup_slot(pc, key, mask),
                LookupEntry {
                    pc,
                    key,
                    entry: 1,
                    block: 0,
                    targets: 1,
                },
            )
        };
        let mut frame = 0;
        while table.sweep_from.is_none() {
            let (slot, held) = entry(frame);
            table.insert(slot, held);
            frame += 1;
        }
        let begun = table.granules.len();
        assert!(begun >= SWEEP_GRANULES);
        // The insert that began it looked at one step's sets.
        let swept = table
            .granules
            .range(..table.sweep_from.expect("under way"))
            .count();
        assert!(swept <= SWEEP_STEP_SETS, "{swept} groups in one step");
        let mut steps = 0;
        while table.sweep_from.is_some() {
            let (slot, held) = entry(frame);
            table.insert(slot, held);
            frame += 1;
            steps += 1;
            assert!(steps <= begun, "the sweep ends");
        }
        assert!(steps > 1);
        // Groups whose entries were put out after their step stay listed
        // until the next sweep; the rest hold an entry each.
        let left = table.granules.len();
        assert!(left <= MIN_LOOKUP_ENTRIES + steps, "{left} groups");
        table.sweep();
        assert!(table.granules.len() <= left);
        assert!(table.granules.len() <= MIN_LOOKUP_ENTRIES);
        assert!(table.sweep_from.is_none());
    }

    /// A frame's entries go wherever their keys put them, of every state
    /// and only those `forget` names; other frames' entries of the same PCs
    /// stay.
    #[test]
    fn forgetting_a_frame_finds_its_entries_of_every_state() {
        const PC: u64 = 0x1_8000_4000;
        let mut table = LookupTable::new();
        let put = |table: &mut LookupTable, pc: u64, key: u64, block: u32| {
            let entry = LookupEntry {
                pc,
                key,
                entry: 1,
                block,
                targets: 1,
            };
            table.insert(lookup_slot(pc, key), entry);
        };
        let held = |table: &LookupTable, pc: u64, key: u64| {
            let entry = table.find(lookup_slot(pc, key), pc, key);
            entry.pc == pc && entry.key == key
        };
        let (frame, other) = (0x8_0000_0000, 0x8_0000_2000);
        let keys = [block_key(frame, 1), block_key(frame, 2)];
        let elsewhere = block_key(other, 1);
        // One PC's keys of one frame share a set, another frame's take
        // another.
        assert_ne!(lookup_slot(PC, keys[0]) / 2, lookup_slot(PC, elsewhere) / 2);
        assert_eq!(lookup_slot(PC, keys[0]), lookup_slot(PC, keys[1]));
        let pcs = |n: u64| PC + 0x10 * n;
        for (n, pc) in (0..32).map(|n| (n, pcs(n))) {
            put(&mut table, pc, keys[0], n as u32);
            put(&mut table, pc, keys[1], 100);
            put(&mut table, pc, elsewhere, 200);
        }
        // Only the named blocks go.
        assert!(
            table.forget_frames(key_frame(keys[0])..=key_frame(keys[0]), |entry| entry.block
                == 100)
        );
        assert!((0..32).all(|n| held(&table, pcs(n), keys[0])));
        assert!((0..32).all(|n| !held(&table, pcs(n), keys[1])));
        assert!(table.forget_frames(key_frame(keys[0])..=key_frame(keys[0]), |_| true));
        assert!((0..32).all(|n| !held(&table, pcs(n), keys[0])));
        assert!((0..32).all(|n| held(&table, pcs(n), elsewhere)));
    }

    #[test]
    fn direct_exits_match_the_block_emitter() {
        let side = ExitWord {
            kind: ExitKind::Direct,
            skipped: 300,
        };
        for (target, word) in [
            (0x1234, ExitWord::from(ExitKind::Direct)),
            (0xffff_fe00_1576_42fc, side),
            (u64::MAX, ExitKind::PointerAuthentication.into()),
            (0, ExitKind::Fallthrough.into()),
        ] {
            let mut emitted = Code::new();
            super::super::block::direct_exit_with(&mut emitted.asm(), GuestAddress(target), word);
            let emitted = emitted.into_bytes();
            assert_eq!(emitted, direct_exit(GuestAddress(target), word));
            for at in 0..16 {
                // Every padding is the same exit; each address has its own,
                // whose patch point is patchable and returns.
                let padded = direct_exit_at(at, GuestAddress(target), word);
                assert!(is_direct_exit(&padded, GuestAddress(target), word));
                let point = direct_patch(at);
                assert!(patchable(point));
                let offset = (point - at) as usize;
                assert_eq!(padded[offset..offset + PATCH_BYTES], RET_ROOM);
                let mut code = emitted.clone();
                assert!(align_direct_exit(&mut code, 0, at));
                assert_eq!(code, padded);
            }
        }
    }

    /// A link touches the exit's patch points alone, and its body, which
    /// lies in the slot past the point. Undoing it puts back what the block
    /// emitter left there.
    #[test]
    fn links_patch_one_patchable_instruction() {
        let block = 0x4000;
        for target in [0x4040, block] {
            let target = GuestAddress(target);
            let mut emitted = Code::new();
            emitted.set_linked_block(block);
            super::super::block::direct_exit_with(&mut emitted.asm(), target, ExitKind::Direct);
            let slot = link_slot(Some(block), target, ExitKind::Direct);
            assert_ne!(slot, 0);
            let emitted = emitted.into_bytes();
            for at in 0x10_0000..0x10_0010 {
                let mut code = emitted.clone();
                assert!(align_link_slot(&mut code, 0, slot, at));
                assert!(align_direct_exit(&mut code, slot, at));
                let site = at + slot as u64;
                let point = slot_patch(at);
                assert!(patchable(point));
                // Ready to link where it lies, and not when it lies elsewhere.
                assert!(exit_ready(&code, slot, at, slot) && exit_ready(&code, slot, at, 0));
                let moved = (1..8).any(|by| !exit_ready(&code, slot, at + by, slot));
                assert!(moved);
                let offset = (point - at) as usize;
                // The empty slot jumps to the exit, as an unlink does.
                let (undone, jump_back) = unlinked(site, slot).unwrap();
                assert_eq!(undone, point);
                assert_eq!(code[offset..offset + PATCH_BYTES], jump_back);
                let (body, bytes) = link(at, slot, 3, 7, 0x10_8000).unwrap();
                assert_eq!(body, point + PATCH_BYTES as u64);
                let start = (body - at) as usize;
                assert!(start + bytes.len() <= slot);
                // The body's every way out of the slot: the target, or the
                // exit right after it.
                let mut linked = code.clone();
                linked[start..start + bytes.len()].copy_from_slice(&bytes);
                linked[offset..offset + PATCH_BYTES].copy_from_slice(&NOP5);
                assert_eq!(linked[slot..], code[slot..]);
                // An exit without a slot is patched at its RET.
                let (exit_point, ret) = unlinked(site, 0).unwrap();
                assert_eq!(exit_point, direct_patch(site));
                let exit = (exit_point - at) as usize;
                assert_eq!(code[exit..exit + PATCH_BYTES], ret);
            }
        }
        assert!(link(0, LINK_BYTES + 1, 3, 7, 0).is_none());
        let word = 0x1122_3344_5566_7788;
        assert_eq!(patch_word(word, 0x1003, RET_ROOM), 0xcccc_cccc_c366_7788);
    }

    #[test]
    fn thunks_count_name_and_jump_or_fall_back() {
        let word = ExitWord::from(ExitKind::Fallthrough);
        let code = thunk(0x1000, 7, 3, 0x2000, Fallback::Return(word)).unwrap();
        let mut expected = vec![0x48, 0x8b, 0x8f];
        expected.extend(824u32.to_le_bytes());
        expected.extend([0x48, 0x8b, 0x51, 0x40, 0x48, 0x3b, 0x51, 0x48, 0x75, 27]);
        expected.extend([0x48, 0x83, 0x29, 0x01, 0x72, 21]);
        expected.extend([0x48, 0x81, 0x41, 0x08, 7, 0, 0, 0]);
        expected.extend([0x48, 0xc7, 0x41, 0x10, 3, 0, 0, 0]);
        // The jump ends 0x2c bytes into the thunk.
        expected.extend([0xe9]);
        expected.extend((0x2000i32 - 0x102c).to_le_bytes());
        expected.push(0xba);
        expected.extend(word.encoded().to_le_bytes());
        expected.push(0xc3);
        assert_eq!(code, expected);
        assert_eq!(code.len(), THUNK_BYTES);
        let back = thunk(0x1000, 7, 3, 0x2000, Fallback::Jump(0x800)).unwrap();
        assert_eq!(
            back[..POLLED_LINK_BODY_BYTES],
            expected[..POLLED_LINK_BODY_BYTES]
        );
        assert_eq!(back.len(), POLLED_LINK_BODY_BYTES + JUMP_BYTES);
        assert_eq!(back[POLLED_LINK_BODY_BYTES], 0xe9);
        assert_eq!(
            back[POLLED_LINK_BODY_BYTES + 1..],
            (0x800i32 - 0x1000 - back.len() as i32).to_le_bytes()
        );
        assert!(thunk(0, 0, 0, 1 << 40, Fallback::Return(word)).is_none());
    }

    #[test]
    fn jumps_reach_32_bits_either_way() {
        assert_eq!(jump(0x1000, 0x1005), Some([0xe9, 0, 0, 0, 0]));
        assert_eq!(jump(0x1000, 0x1000), Some([0xe9, 0xfb, 0xff, 0xff, 0xff]));
        assert_eq!(jump(0, 1 << 32), None);
    }

    #[test]
    fn dynamic_exits_match_the_block_emitter() {
        for kind in [ExitKind::Return, ExitKind::Call, ExitKind::System] {
            let mut emitted = Code::new();
            super::super::block::exit_kind(&mut emitted, kind);
            assert_eq!(emitted, dynamic_exit(kind));
            let emitted = emitted.into_bytes();
            for at in 0..16 {
                let padded = dynamic_exit_at(at, kind);
                assert!(is_dynamic_exit(&padded, kind));
                assert!(!is_dynamic_exit(&padded, ExitKind::Direct));
                let offset = (dynamic_patch(at) - at) as usize;
                assert!(patchable(dynamic_patch(at)));
                assert_eq!(padded[offset..offset + PATCH_BYTES], RET_ROOM);
                let mut code = emitted.clone();
                assert!(align_dynamic_exit(&mut code, 0, at));
                assert_eq!(code, padded);
                assert!(dynamic_ready(&code, 0, at));
            }
        }
        let span = 4 * LOOKUP_ENTRIES as u64;
        assert_eq!(lookup_slot(4, 0), 1);
        assert_eq!(lookup_slot(span, 0), 1);
        assert_ne!(lookup_slot(span + 4, 0), lookup_slot(4, 0));
        assert_ne!(lookup_slot(4, 1), lookup_slot(4, 0));
        assert_eq!(
            lookup_slot(4, block_key(0x1000, 1)),
            lookup_slot(4, block_key(0x1000, 2)),
            "a key's frame alone picks its slot"
        );
        assert_ne!(
            lookup_slot(4, block_key(0x1000, 1)),
            lookup_slot(4, block_key(0x2000, 1))
        );
        assert_eq!(key_frame(block_key(0x1234_5678, 9)), 0x1_2345);
        // An ITLB slot is a page's: every PC of it shares one.
        assert_eq!(itlb_slot(0x4000, 7), itlb_slot(0x4ffc, 7));
        assert_ne!(itlb_slot(0x4000, 7), itlb_slot(0x5000, 7));
        // Each the first of its own set: a neighbouring page's is another.
        assert_eq!(itlb_slot(0x4000, 7) % 2, 0);
        assert_ne!(itlb_slot(0x4000, 7) / 2, itlb_slot(0x5000, 7) / 2);
        assert_ne!(itlb_slot(0x4000, 7), itlb_slot(0x4000, 8));
        assert_eq!(
            itlb_slot(0x4000, 7),
            itlb_slot(0x4000, 7 + LOOKUP_SALTS as u64),
            "only a context's low bits pick its slot"
        );
    }

    /// Lookup dispatchers run on the host: a target whose page the ITLB
    /// translates in the current context of its half, with a table entry
    /// of that key and budget left, counts and enters the entry's block; an
    /// untranslated page, another context, another PC or key in the slot,
    /// or no budget exits with the target and kind.
    #[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
    #[test]
    fn lookup_thunks_enter_current_entries_and_exit_otherwise() {
        use core::ffi::c_void;
        unsafe extern "C" {
            fn mmap(
                address: *mut c_void,
                bytes: usize,
                protection: i32,
                flags: i32,
                fd: i32,
                offset: i64,
            ) -> *mut c_void;
            fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
            fn munmap(address: *mut c_void, bytes: usize) -> i32;
        }
        #[cfg(target_os = "macos")]
        const MAP_ANONYMOUS: i32 = 0x1000;
        #[cfg(target_os = "linux")]
        const MAP_ANONYMOUS: i32 = 0x20;
        const PAGE: usize = 8192;
        const BLOCK_AT: usize = 6144;
        const TARGET: u64 = 0xffff_fe00_0123_4568;
        const KEY: u64 = block_key(0x8_0123_4000, 1);
        let mut table = LookupTable::new();
        let mut itlb = LookupTable::itlb();
        let mut control = ChainControl {
            table: table.address(),
            itlb: itlb.address(),
            contexts: [5, 7],
            salts: [itlb_salt(5), itlb_salt(7)],
            masks: [TABLE_MASK, ITLB_MASK],
            ..ChainControl::default()
        };
        let translate = |itlb: &mut LookupTable, pc: u64, context: u64| {
            let page = LookupEntry::page(pc & !0xfff, context, KEY);
            itlb.insert(itlb_slot(pc, context), page);
        };
        let mut state = crate::GuestState::default();
        state.anchors[Anchor::Control] = &mut control as *mut ChainControl as u64;
        let frame = &mut state as *mut crate::GuestState as u64;
        // A target block that returns a marker; the dynamic thunk takes its
        // target from the second argument; the static one names TARGET.
        let block = [
            0xb8, 0x51, 0x51, 0, 0, // mov eax,0x5151
            0xba, 99, 0, 0, 0, // mov edx,99
            0xc3,
        ];
        let dynamic = lookup_thunk(9, None, ExitKind::Return, 0).unwrap();
        let fixed = lookup_thunk(3, Some(GuestAddress(TARGET)), ExitKind::Call, 0).unwrap();
        let branch = lookup_thunk(5, None, ExitKind::Indirect, 3).unwrap();
        assert!(lookup_dispatcher(4).is_none());
        // SAFETY: private anonymous memory, RX after the copies, unmapped
        // at the end; the thunks touch only `control`, `table` and `itlb`.
        unsafe {
            let page = mmap(core::ptr::null_mut(), PAGE, 3, 2 | MAP_ANONYMOUS, -1, 0);
            assert_ne!(page as usize, usize::MAX);
            let base = page as *mut u8;
            let mut code = vec![0x48, 0x89, 0xf0]; // mov rax,rsi
            code.extend(&dynamic);
            let fixed_at = code.len();
            code.extend(&fixed);
            let branch_at = code.len();
            code.extend([0x48, 0x89, 0xf0]); // mov rax,rsi
            code.extend(&branch);
            assert!(code.len() <= BLOCK_AT);
            core::ptr::copy_nonoverlapping(code.as_ptr(), base, code.len());
            core::ptr::copy_nonoverlapping(block.as_ptr(), base.add(BLOCK_AT), block.len());
            assert_eq!(mprotect(page, PAGE, 5), 0);
            type Thunk = unsafe extern "sysv64" fn(u64, u64) -> u128;
            let dynamic: Thunk = core::mem::transmute(page);
            let fixed: Thunk = core::mem::transmute(base.add(fixed_at));
            let branch: Thunk = core::mem::transmute(base.add(branch_at));
            let exit = |result: u128| {
                let word = (result >> 64) as u64;
                (result as u64, ExitWord::split(word).0)
            };
            let entry = LookupEntry {
                pc: TARGET,
                key: KEY,
                entry: base.add(BLOCK_AT) as u64,
                block: 42,
                targets: 0b0111,
            };
            table.set(lookup_slot(TARGET, KEY), entry);
            control.budget = 2;
            // No translation of the target's page yet: the exit returns.
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!(control.budget, 2);
            translate(&mut itlb, TARGET, 7);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!(
                (control.budget, control.retired, control.hits, control.block),
                (1, 9, 1, 42)
            );
            assert_eq!(exit(fixed(frame, 0)), (0x5151, 99));
            assert_eq!((control.budget, control.retired, control.hits), (0, 12, 2));
            // No budget left: the budget wraps and the exit is taken.
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!((control.budget, control.retired), (u64::MAX, 12));
            control.budget = 5;
            // Another PC in the slot misses.
            let other = TARGET ^ (1 << (2 + LOOKUP_BITS) | 1 << 2);
            translate(&mut itlb, other, 7);
            assert_eq!(lookup_slot(other, KEY), lookup_slot(TARGET, KEY));
            assert_eq!(
                exit(dynamic(frame, other)),
                (other, ExitKind::Return as u64)
            );
            // The set's other entry serves too.
            let (hits, retired) = (control.hits, control.retired);
            table.insert(lookup_slot(TARGET, KEY), LookupEntry { pc: other, ..entry });
            assert_eq!(table.entry(neighbour(lookup_slot(TARGET, KEY))).pc, TARGET);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!(table.find(lookup_slot(TARGET, KEY), TARGET, KEY).pc, TARGET);
            assert_eq!(exit(dynamic(frame, other)), (0x5151, 99));
            assert_eq!(control.hits, hits + 2);
            control.retired = retired;
            control.hits = hits;
            control.budget = 5;
            table.set(neighbour(lookup_slot(TARGET, KEY)), LookupEntry::default());
            // An entry of another key, another frame or fetch state, misses.
            table.set(
                lookup_slot(TARGET, KEY),
                LookupEntry {
                    key: KEY + 1,
                    ..entry
                },
            );
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!(exit(fixed(frame, 0)), (TARGET, ExitKind::Call as u64));
            assert_eq!((control.budget, control.hits, control.block), (5, 2, 42));
            table.set(lookup_slot(TARGET, KEY), entry);
            // A new context translates the page anew, and then finds the
            // table's entry, which outlived the old one.
            control.contexts[1] = 9;
            control.salts[1] = itlb_salt(9);
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            translate(&mut itlb, TARGET, 9);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!((control.budget, control.hits), (4, 3));
            // The lower half's context translates a lower-half target.
            let lower = TARGET & !(1 << 55);
            table.set(lookup_slot(lower, KEY), LookupEntry { pc: lower, ..entry });
            translate(&mut itlb, lower, 9);
            assert_eq!(
                exit(dynamic(frame, lower)),
                (lower, ExitKind::Return as u64)
            );
            translate(&mut itlb, lower, 5);
            assert_eq!(exit(dynamic(frame, lower)), (0x5151, 99));
            assert_eq!((control.budget, control.hits), (3, 4));
            // A BR with BTYPE 0b11 needs the entry to accept it, and names
            // the BTYPE beside the block.
            assert_eq!(
                exit(branch(frame, TARGET)),
                (TARGET, ExitKind::Indirect as u64)
            );
            table.set(
                lookup_slot(TARGET, KEY),
                LookupEntry {
                    targets: 0b1001,
                    ..entry
                },
            );
            assert_eq!(exit(branch(frame, TARGET)), (0x5151, 99));
            assert_eq!(
                (control.budget, control.retired, control.hits, control.block),
                (2, 35, 5, 3 << BLOCK_BTYPE_SHIFT | 42)
            );
            control.budget = 5;
            control.interrupt_epoch = 1;
            control.observed_interrupt_epoch = 0;
            let prior = (
                control.budget,
                control.retired,
                control.hits,
                control.linked,
                control.block,
            );
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!(exit(fixed(frame, 0)), (TARGET, ExitKind::Call as u64));
            assert_eq!(
                exit(branch(frame, TARGET)),
                (TARGET, ExitKind::Indirect as u64)
            );
            assert_eq!(
                (
                    control.budget,
                    control.retired,
                    control.hits,
                    control.linked,
                    control.block
                ),
                prior
            );
            control.observed_interrupt_epoch = 1;
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!(control.budget, 4);
            assert_eq!(munmap(page, PAGE), 0);
        }
    }
    /// A cell holds the entry it was linked with, takes each entry of its
    /// PC the table takes, of any key, goes with its frame's forgetting as
    /// the table's entries do, even after the table let that entry go, and
    /// is freed with its exit's code.
    #[test]
    fn cells_follow_the_table_entries_of_their_pc() {
        const PC: u64 = 0xffff_fe00_0123_4000;
        let key = |frame: u64| block_key(frame << PAGE_SHIFT, 1);
        let entry = |frame: u64, entry: u64| LookupEntry {
            pc: PC,
            key: key(frame),
            entry,
            block: 9,
            targets: 1,
        };
        let mut table = LookupTable::new();
        let site = 0x9000;
        let address = table.link_cell(site, PC, Some(entry(5, 0x100)));
        assert_eq!(table.site_cell(site), Some(entry(5, 0x100)));
        // SAFETY: the cell stays at its address while its exit holds it.
        let raw = unsafe { *(address as *const LookupEntry) };
        assert_eq!(raw.targets & META_BUSY, 0);
        assert_eq!(
            LookupEntry {
                targets: raw.targets & META_TARGETS,
                ..raw
            },
            entry(5, 0x100)
        );
        assert_eq!(table.cells(), 1);
        // An entry of another PC is not the cell's.
        table.set(
            lookup_slot(PC + 4, key(5)),
            LookupEntry {
                pc: PC + 4,
                ..entry(5, 0x200)
            },
        );
        assert_eq!(table.site_cell(site), Some(entry(5, 0x100)));
        // A new entry of its PC, of any key, is the cell's.
        table.insert(lookup_slot(PC, key(6)), entry(6, 0x300));
        assert_eq!(table.site_cell(site), Some(entry(6, 0x300)));
        // Forgetting another frame's entries leaves it.
        assert!(table.forget_frames(5..=5, |_| true));
        assert_eq!(table.site_cell(site), Some(entry(6, 0x300)));
        // Forgetting its frame's empties it, keeping its PC.
        assert!(table.forget_frames(6..=6, |_| true));
        let empty = LookupEntry {
            pc: PC,
            ..LookupEntry::default()
        };
        assert_eq!(table.site_cell(site), Some(empty));
        // The next entry of its PC fills it again; the table may let its
        // own go, and a release of the block still empties the cell.
        table.set(lookup_slot(PC, key(7)), entry(7, 0x400));
        assert_eq!(table.site_cell(site), Some(entry(7, 0x400)));
        table.set(lookup_slot(PC, key(7)), LookupEntry::default());
        assert_eq!(table.site_cell(site), Some(entry(7, 0x400)));
        table.forget_frames(0..=key_frame(u64::MAX), |entry| entry.block == 9);
        assert_eq!(table.site_cell(site), Some(empty));
        table.set(lookup_slot(PC, key(7)), entry(7, 0x400));
        table.clear();
        assert_eq!(table.site_cell(site), Some(empty));
        assert!(table.is_empty() && table.cell_frames.is_empty());
        // A link with no current entry, or one of another PC, starts empty.
        let other = table.link_cell(
            site + 8,
            PC,
            Some(LookupEntry {
                pc: PC + 4,
                ..entry(5, 1)
            }),
        );
        assert_eq!(table.site_cell(site + 8), Some(empty));
        assert_ne!(other, address);
        // Its exit's code goes: the cell is free for another exit.
        table.release_cells(site..=site);
        assert_eq!((table.site_cell(site), table.cells()), (None, 1));
        assert_eq!(table.link_cell(site + 16, PC + 8, None), address);
        table.release_cells(0..=u64::MAX);
        assert_eq!(table.cells(), 0);
        assert!(table.cell_frames.is_empty());
        table.set(
            lookup_slot(PC + 8, key(5)),
            LookupEntry {
                pc: PC + 8,
                ..entry(5, 1)
            },
        );
        assert_eq!(table.site_cell(site + 16), None);
    }

    /// Each tail calls its probe and jumps through R9 itself: the site's own
    /// indirect jump. A probe set's pieces keep their places in every policy.
    #[test]
    fn tails_jump_to_the_probes() {
        let mut plain = Code::new();
        tail(&mut plain, 0x1000, ExitKind::Return.into(), 3, 0x2000).unwrap();
        // EDX the word with the instructions run, then the jump.
        let mut expected = vec![0xba];
        expected.extend((ExitKind::Return as u32 | 3 << ExitWord::RAN_SHIFT).to_le_bytes());
        expected.push(0xe9);
        expected.extend((0x2000i32 - 0x100a).to_le_bytes());
        assert_eq!(plain.to_vec(), expected);
        // The cached tail ends its misses in the same word and jump.
        let cached = lookup_tail(0x1000, ExitKind::Return, 3, 0x2000, 0).unwrap();
        let word = &expected[..5];
        assert!(cached.windows(5).any(|bytes| bytes == word));
        assert_eq!(cached.last(), Some(&0xcc));
        // Every BTYPE's tail fits, the longest's both BTS included.
        for btype in 0..4 {
            assert!(lookup_tail(0x1000, ExitKind::Return, 3, 0x2000, btype).is_some());
        }
        assert!(lookup_tail(0x1000, ExitKind::Return, 3, 0x2000, 4).is_none());
        let cell = cell_tail(0x1000, 0xdead_beef_0000, ExitKind::Return, 3, 0x2000).unwrap();
        assert_eq!(&cell[..2], &[0x49, 0xb9]);
        assert_eq!(&cell[2..10], &0xdead_beef_0000u64.to_le_bytes());
        assert_eq!(&cell[10..15], &expected[..5]);
        assert_eq!(&cell[15..20], &[0xe9, 0xec, 0x0f, 0, 0]);
        assert!(lookup_tail(0, ExitKind::Return, 0, 1 << 40, 0).is_none());
        assert!(lookup_tail(0, ExitKind::Return, 1 << 12, 0, 0).is_none());
        for btype in 0..4 {
            let probes = lookup_dispatcher(btype).unwrap();
            // Each probe starts with the control's address.
            for at in [CELL_PROBE_ENTRY, PROBE_ENTRY] {
                assert_eq!(&probes[at..at + 3], &[0x48, 0x8b, 0x8f]);
            }
            #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
            for policy in [
                Interruption::Legacy,
                Interruption::LockedBudget,
                Interruption::Flag,
            ] {
                assert!(dispatcher_with_policy(btype, policy).is_some());
            }
        }
    }

    /// A cell probe enters the block its cell names when the cell holds the
    /// target with the key the ITLB gives the target's page in the current
    /// context of its half, counting a transfer but not a lookup hit;
    /// otherwise the table decides. With no translation, no budget left or
    /// a pending interruption the exit returns, the budget spent at most
    /// once.
    #[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
    #[test]
    fn cell_tails_enter_their_cells_blocks_and_fall_back_to_the_table() {
        use core::ffi::c_void;
        unsafe extern "C" {
            fn mmap(
                address: *mut c_void,
                bytes: usize,
                protection: i32,
                flags: i32,
                fd: i32,
                offset: i64,
            ) -> *mut c_void;
            fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
            fn munmap(address: *mut c_void, bytes: usize) -> i32;
        }
        #[cfg(target_os = "macos")]
        const MAP_ANONYMOUS: i32 = 0x1000;
        #[cfg(target_os = "linux")]
        const MAP_ANONYMOUS: i32 = 0x20;
        const PAGE: usize = 4096;
        const CELL_BLOCK: usize = 2048;
        const TABLE_BLOCK: usize = 3072;
        const KEY: u64 = block_key(0x8_0000_2000, 1);
        for target in [0xffff_fe00_0123_4568_u64, 0x0000_0001_0000_2468] {
            let half = half(target);
            let mut table = LookupTable::new();
            let mut itlb = LookupTable::itlb();
            let mut control = ChainControl {
                table: table.address(),
                itlb: itlb.address(),
                contexts: [5, 7],
                salts: [itlb_salt(5), itlb_salt(7)],
                masks: [TABLE_MASK, ITLB_MASK],
                ..ChainControl::default()
            };
            let context = control.contexts[half];
            let mut state = crate::GuestState::default();
            state.anchors[Anchor::Control] = &mut control as *mut ChainControl as u64;
            let frame = &mut state as *mut crate::GuestState as u64;
            // SAFETY: private anonymous memory, RX after the copies, unmapped
            // at the end; the code touches only `control`, the tables and
            // cells.
            unsafe {
                let page = mmap(core::ptr::null_mut(), PAGE, 3, 2 | MAP_ANONYMOUS, -1, 0);
                assert_ne!(page as usize, usize::MAX);
                let base = page as *mut u8;
                let marked = |marker: u8| [0xb8, marker, 0x51, 0, 0, 0xba, 99, 0, 0, 0, 0xc3];
                let cell_entry = LookupEntry {
                    pc: target,
                    key: KEY,
                    entry: base.add(CELL_BLOCK) as u64,
                    block: 42,
                    targets: 1,
                };
                let table_entry = LookupEntry {
                    entry: base.add(TABLE_BLOCK) as u64,
                    block: 43,
                    ..cell_entry
                };
                let cell = table.link_cell(1, target, Some(cell_entry));
                let code =
                    lookup_thunk_with_cell(5, GuestAddress(target), ExitKind::Call, cell).unwrap();
                core::ptr::copy_nonoverlapping(code.as_ptr(), base, code.len());
                for (at, marker) in [(CELL_BLOCK, 0x11), (TABLE_BLOCK, 0x22)] {
                    let block = marked(marker);
                    core::ptr::copy_nonoverlapping(block.as_ptr(), base.add(at), block.len());
                }
                assert_eq!(mprotect(page, PAGE, 5), 0);
                let run: unsafe extern "sysv64" fn(u64) -> u128 = core::mem::transmute(page);
                let exit = |result: u128| {
                    let word = (result >> 64) as u64;
                    (result as u64, ExitWord::split(word).0)
                };
                control.budget = 3;
                // The target's page untranslated: the exit returns.
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(control.budget, 3);
                let translated = LookupEntry::page(target & !0xfff, context, KEY);
                itlb.insert(itlb_slot(target, context), translated);
                assert_eq!(exit(run(frame)), (0x5111, 99));
                assert_eq!(
                    (control.budget, control.retired, control.hits, control.block),
                    (2, 5, 0, 42)
                );
                // Another key: the table's entry, a lookup hit.
                *(cell as *mut LookupEntry) = LookupEntry {
                    key: KEY + 1,
                    ..cell_entry
                };
                assert_eq!(
                    exit(run(frame)),
                    (target, ExitKind::Call as u64),
                    "no table entry"
                );
                assert_eq!(control.budget, 2);
                table.set(lookup_slot(target, KEY), table_entry);
                // The set filled the cell with the table's entry.
                assert_eq!(table.site_cell(1), Some(table_entry));
                *(cell as *mut LookupEntry) = LookupEntry {
                    key: KEY + 1,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!(
                    (control.budget, control.retired, control.hits, control.block),
                    (1, 10, 1, 43)
                );
                // A cell of another PC is never entered.
                *(cell as *mut LookupEntry) = LookupEntry {
                    pc: target + 4,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!((control.budget, control.hits), (0, 2));
                // No budget: one spend wraps it, and the exit returns.
                *(cell as *mut LookupEntry) = cell_entry;
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!((control.budget, control.retired), (u64::MAX, 15));
                // Another context of the same salt: the page translates
                // anew, whatever the cell holds.
                control.budget = 5;
                control.contexts[half] = context + LOOKUP_SALTS as u64;
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(control.budget, 5);
                control.contexts[half] = context;
                // A cell a writer holds (or changed while it was read): the
                // table probe looks the target up anew by its key and finds
                // the table's entry, never a key made of the cell's block.
                *(cell as *mut LookupEntry) = LookupEntry {
                    targets: 1 | META_BUSY,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!((control.budget, control.hits, control.block), (4, 3, 43));
                *(cell as *mut LookupEntry) = cell_entry;
                control.budget = 5;
                // A pending interruption: nothing spent or counted.
                control.interrupt_epoch = 1;
                let prior = (control.budget, control.retired, control.hits, control.block);
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                *(cell as *mut LookupEntry) = LookupEntry {
                    key: 0,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(
                    (control.budget, control.retired, control.hits, control.block),
                    prior
                );
                control.observed_interrupt_epoch = 1;
                *(cell as *mut LookupEntry) = cell_entry;
                assert_eq!(exit(run(frame)), (0x5111, 99));
                assert_eq!(control.budget, 4);
                assert_eq!(munmap(page, PAGE), 0);
            }
        }
    }

    /// A return's transfers are validated against the ITLB and the table,
    /// never predicted from the call that made them: a return to where no
    /// call was made (longjmp-like unwinding, a tail call's return to its
    /// caller's caller), to a PAC-poisoned address, or after a TLBI of the
    /// target's page, exits; the same tail serves every target the table
    /// holds.
    #[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
    #[test]
    fn return_tails_validate_every_target() {
        use core::ffi::c_void;
        unsafe extern "C" {
            fn mmap(
                address: *mut c_void,
                bytes: usize,
                protection: i32,
                flags: i32,
                fd: i32,
                offset: i64,
            ) -> *mut c_void;
            fn mprotect(address: *mut c_void, bytes: usize, protection: i32) -> i32;
            fn munmap(address: *mut c_void, bytes: usize) -> i32;
        }
        #[cfg(target_os = "macos")]
        const MAP_ANONYMOUS: i32 = 0x1000;
        #[cfg(target_os = "linux")]
        const MAP_ANONYMOUS: i32 = 0x20;
        const PAGE: usize = 4096;
        let callers = [
            0xffff_fe00_0010_0004_u64,
            0xffff_fe00_0020_0008,
            0x1_0000_000c,
        ];
        let mut table = LookupTable::new();
        let mut itlb = LookupTable::itlb();
        let mut control = ChainControl {
            table: table.address(),
            itlb: itlb.address(),
            contexts: [5, 7],
            salts: [itlb_salt(5), itlb_salt(7)],
            masks: [TABLE_MASK, ITLB_MASK],
            ..ChainControl::default()
        };
        let mut state = crate::GuestState::default();
        state.anchors[Anchor::Control] = &mut control as *mut ChainControl as u64;
        let frame = &mut state as *mut crate::GuestState as u64;
        // SAFETY: private anonymous memory, RX after the copies, unmapped at
        // the end; the code touches only `control` and the tables.
        unsafe {
            let page = mmap(core::ptr::null_mut(), PAGE, 3, 2 | MAP_ANONYMOUS, -1, 0);
            assert_ne!(page as usize, usize::MAX);
            let base = page as *mut u8;
            let mut code = vec![0x48, 0x89, 0xf0]; // mov rax,rsi
            code.extend(lookup_thunk(4, None, ExitKind::Return, 0).unwrap());
            core::ptr::copy_nonoverlapping(code.as_ptr(), base, code.len());
            for (index, &caller) in callers.iter().enumerate() {
                let at = 2048 + 64 * index;
                let block = [0xb8, index as u8, 0x51, 0, 0, 0xba, 99, 0, 0, 0, 0xc3];
                core::ptr::copy_nonoverlapping(block.as_ptr(), base.add(at), block.len());
                let context = control.contexts[half(caller)];
                let key = block_key(0x4000_0000 + 0x1000_0000 * index as u64, 1);
                itlb.insert(
                    itlb_slot(caller, context),
                    LookupEntry::page(caller & !0xfff, context, key),
                );
                table.insert(
                    lookup_slot(caller, key),
                    LookupEntry {
                        pc: caller,
                        key,
                        entry: base.add(at) as u64,
                        block: index as u32,
                        targets: 1,
                    },
                );
            }
            assert_eq!(mprotect(page, PAGE, 5), 0);
            let ret: unsafe extern "sysv64" fn(u64, u64) -> u128 = core::mem::transmute(page);
            let exit = |result: u128| (result as u64, ExitWord::split((result >> 64) as u64).0);
            control.budget = 100;
            // Returns to each caller in turn, through one site's tail.
            for round in 0..3 {
                for (index, &caller) in callers.iter().enumerate() {
                    assert_eq!(
                        exit(ret(frame, caller)),
                        (0x5100 | index as u64, 99),
                        "{round}"
                    );
                    assert_eq!(control.block, index as u64);
                }
            }
            assert_eq!((control.hits, control.retired), (9, 36));
            // A return past its caller, to where no entry is, and to a
            // poisoned address exit with the target as computed.
            let poisoned = callers[0] ^ 1 << 53;
            for target in [callers[1] + 4, poisoned] {
                assert_eq!(exit(ret(frame, target)), (target, ExitKind::Return as u64));
            }
            // The caller's page went with a TLBI of it.
            let caller_page = callers[1] & !0xfff;
            assert!(itlb.forget_range(caller_page..=caller_page | 0xfff, u64::MAX));
            assert_eq!(
                exit(ret(frame, callers[1])),
                (callers[1], ExitKind::Return as u64)
            );
            assert_eq!(exit(ret(frame, callers[2])), (0x5102, 99));
            assert_eq!((control.hits, control.budget), (10, 90));
            assert_eq!(munmap(page, PAGE), 0);
        }
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod budget;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod differential;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod jump_cache;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod patching;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod predicted;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod shared;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod stamped;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod transfers;

    /// A block's entry goes from its set and from the cells of its PC by
    /// its PC, key and block alone: another block's entry of the PC and
    /// key, and the frame's other entries, stay.
    #[test]
    fn a_block_entry_goes_from_its_set_and_cells_without_its_frame() {
        let mut table = LookupTable::new();
        let key = block_key(0x8000_0000, 2);
        let (pc, site) = (0x1000, 0x9000);
        let entry = LookupEntry {
            pc,
            key,
            entry: 0x5000,
            block: 7,
            targets: 0b1111,
        };
        let other = LookupEntry {
            pc: pc + 4,
            block: 8,
            entry: 0x6000,
            ..entry
        };
        let slot = table.slot(pc, key);
        table.insert(slot, entry);
        let other_slot = table.slot(other.pc, key);
        table.insert(other_slot, other);
        table.link_cell(site, pc, Some(entry));
        assert_eq!(table.site_cell(site), Some(entry));
        assert_eq!(table.forget_block(pc, key, 9), 1);
        assert_eq!(table.find(slot, pc, key), entry);
        assert_eq!(table.forget_block(pc, key, 7), 1);
        assert_ne!(table.find(slot, pc, key).pc, pc);
        assert_eq!(
            table.site_cell(site).map(|cell| (cell.pc, cell.key)),
            Some((pc, 0))
        );
        assert_eq!(table.find(other_slot, other.pc, key), other);
    }

    /// Two entries a lookup keeps reaching and a third that nothing
    /// reaches share one set: as an engine does, each lookup that misses
    /// inserts its entry again. The two settle in the set within a few
    /// rounds; always putting out the neighbour had them put each other out
    /// at every round (two misses a round, the guest's call/return loop at
    /// an engine exit per return).
    #[test]
    fn two_reached_entries_put_a_third_out_of_their_set() {
        let mut table = LookupTable::new();
        let key = block_key(0x8_0000_0000, 1);
        // `pc >> 2` even: `pc + 4` takes the neighbouring slot.
        let pc = 0x1_0000_e030 & !0x4;
        let entry = |pc: u64, key: u64, block: u32| LookupEntry {
            pc,
            key,
            entry: 0x1000 * u64::from(block),
            block,
            targets: 0b1111,
        };
        let hot = [entry(pc, key, 1), entry(pc + 4, key, 2)];
        assert_eq!(lookup_slot(pc, key), neighbour(lookup_slot(pc + 4, key)));
        // Another state's block of the first PC: the first's slot.
        let stale = entry(pc, block_key(0x8_0000_0000, 2), 3);
        let slot = |entry: &LookupEntry| lookup_slot(entry.pc, entry.key);
        assert_eq!(slot(&stale), slot(&hot[0]));
        table.insert(slot(&stale), stale);
        let rounds = 1000;
        let mut misses = 0;
        for _ in 0..rounds {
            for wanted in &hot {
                if table.find(slot(wanted), wanted.pc, wanted.key) != *wanted {
                    misses += 1;
                    table.insert(slot(wanted), *wanted);
                }
            }
        }
        assert!(misses < 40, "{misses} misses in {rounds} rounds");
        for wanted in &hot {
            assert_eq!(table.find(slot(wanted), wanted.pc, wanted.key), *wanted);
        }
        assert_ne!(table.find(slot(&stale), stale.pc, stale.key), stale);
        assert!(table.evicted().iter().sum::<u64>() > 0);
    }

    /// The largest ITLB, 64 Ki entries in 2 MiB, keeps every invalidation
    /// rule of the default one: a TLBI by VA drops its pages in every
    /// context and grows their half's drop epoch, a renewed context of an
    /// old one's salt takes its slots but none of its entries and goes
    /// with its salt, and forgetting everything empties it and grows both
    /// epochs. Pages the default ITLB keeps in one set take two.
    #[test]
    fn the_largest_itlb_keeps_its_invalidation_rules() {
        const LOWER: u64 = 0x0000_7f12_3456_0000;
        const UPPER: u64 = 0xffff_fe00_0123_4000;
        let mut itlb = LookupTable::itlb_with(MAX_ITLB_ENTRIES);
        assert_eq!(itlb.entries(), 1 << 16);
        assert_eq!(itlb.mask(), (1 << 15) - 1);
        assert_eq!(itlb.table_bytes(), 2 << 20);
        // Page number bit 14: one set of the default ITLB, two here.
        let apart = UPPER ^ 1 << (PAGE_SHIFT + 14);
        let small = LookupTable::itlb();
        assert_eq!(small.page_slot(UPPER, 1) / 2, small.page_slot(apart, 1) / 2);
        assert_ne!(itlb.page_slot(UPPER, 1) / 2, itlb.page_slot(apart, 1) / 2);
        let put = |itlb: &mut LookupTable, page: u64, context: u64| {
            itlb.insert(
                itlb.page_slot(page, context),
                LookupEntry::page(page, context, 1),
            );
        };
        let held = |itlb: &LookupTable, page: u64, context: u64| {
            let entry = itlb.find(itlb.page_slot(page, context), page, context);
            entry.pc == page && entry.key == context
        };
        let epoch =
            |itlb: &LookupTable, page: u64| itlb.sets.header()[half(page)].load(Ordering::Relaxed);
        let pages = [LOWER, LOWER + 0x1000, UPPER, apart];
        for context in 1..=8 {
            for page in pages {
                put(&mut itlb, page, context);
            }
        }
        assert!(
            pages
                .iter()
                .all(|&page| (1..=8).all(|c| held(&itlb, page, c)))
        );
        // A TLBI by VA of one page, every ASID: that page in every context.
        let (upper, lower) = (epoch(&itlb, UPPER), epoch(&itlb, LOWER));
        assert!(itlb.forget_range(UPPER..=UPPER + 0xfff, u64::MAX));
        assert!((1..=8).all(|c| !held(&itlb, UPPER, c)));
        assert!((1..=8).all(|c| held(&itlb, apart, c) && held(&itlb, LOWER, c)));
        assert!(epoch(&itlb, UPPER) > upper);
        assert_eq!(epoch(&itlb, LOWER), lower);
        // Context 1 renewed as the number of its salt: its slots, none of
        // its entries.
        let renewed = 1 + LOOKUP_SALTS as u64;
        assert_eq!(itlb.salt(renewed), itlb.salt(1));
        assert_eq!(itlb.page_slot(LOWER, renewed), itlb.page_slot(LOWER, 1));
        assert!(!held(&itlb, LOWER, renewed));
        put(&mut itlb, LOWER, renewed);
        assert!(held(&itlb, LOWER, renewed) && held(&itlb, LOWER, 1));
        // Forgetting salt 1's pages takes both numbers and no other salt.
        let lower = epoch(&itlb, LOWER);
        assert!(itlb.forget_range(LOWER..=LOWER + 0x1fff, 1 << 1));
        assert!(!held(&itlb, LOWER, renewed) && !held(&itlb, LOWER, 1));
        assert!(!held(&itlb, LOWER + 0x1000, 1));
        assert!((2..=8).all(|c| held(&itlb, LOWER, c) && held(&itlb, LOWER + 0x1000, c)));
        assert!(epoch(&itlb, LOWER) > lower);
        // Everything: empty, both halves' epochs grown.
        let (upper, lower) = (epoch(&itlb, UPPER), epoch(&itlb, LOWER));
        itlb.clear();
        assert!(itlb.is_empty());
        assert!(epoch(&itlb, UPPER) > upper && epoch(&itlb, LOWER) > lower);
    }
}
