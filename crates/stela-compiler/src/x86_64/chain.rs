//! Block chaining for the system-mode engine: a block's direct exit to a
//! static successor can be redirected through a thunk straight into the
//! successor's code, so the dispatcher runs once for a chain of blocks.
//!
//! Each PE owns one [`ChainControl`] that [`Anchor::Control`] names. Before it
//! enters a block the engine sets `linked` and `hits` to 0 and `block` to the
//! entered block's index, and the run's [`Account`] to the chained transfers
//! it allows and no instructions. Native code keeps the account in a host
//! register, [`ACCOUNT`], whose home outside native code is
//! `GuestState::account` ([`super::pinned`]). Each link adds the instructions
//! of the block it leaves and takes one transfer off in one LEA, then tests
//! the transfers left by MOVZX and JRCXZ: none left takes the instructions
//! off again and exits exactly as the redirected exit would. Otherwise it
//! names the block it enters in `block`; only a table probe counts its
//! transfers apart, in `hits`, and the engine takes the others from the
//! budget spent. Notifications publish a monotonic interrupt epoch; every
//! polled link compares it with the owner's acknowledged pre-drain snapshot
//! before spending the budget. A mismatch takes the original exit. After the
//! native call returns, `block` is the block whose exit or fault ended the
//! run, in its low 32 bits, and the BTYPE it was entered with in bits 33:32:
//! 0 from every link but a lookup dispatcher of a BR or BLR form.
//! Links clobber only RAX, RCX and RDX, which every direct exit already may,
//! and never the flags: LEA, MOVZX, NOT, MOV and JRCXZ leave them, so NZCV a
//! block keeps in the host's flags could cross one. Tails and probes compare
//! targets and clobber the flags.
//!
//! Live code changes only at patch points, each one instruction of
//! [`PATCH_BYTES`] that lies in one aligned 8-byte word ([`patchable`]):
//! the point of an exit ([`direct_patch`], [`dynamic_patch`]) and the start
//! of a link slot ([`slot_patch`]). Another core may be running the
//! instruction, so the host replaces it with one aligned 8-byte store, and
//! the core runs either the old instruction or the new one, each complete:
//! the jump to the exit's return to the engine ([`RET_ROOM`]) or a
//! `jmp rel32` to a thunk or tail at an exit, a jump to the exit or the
//! first bytes of the slot's body at a slot ([`link`]). Whatever a new
//! instruction reaches is written, and serialized by the writer, before the
//! store: a thunk or tail in fresh arena bytes, or the rest of a slot's
//! body, which nothing runs while its point still jumps over it. A body once linked is not written
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
//!
//! Every return to the engine reloads RSP from the state's saved entry
//! stack pointer first ([`EXIT_RETURN`]), so native code may leave words
//! on the host stack between blocks: a guest call pushes a paired frame
//! and calls its own exit ([`paired_call`]), and the return whose target
//! and contexts are the frame's takes a host `ret` into the call's
//! continuation ([`paired_return`], [`paired_tail`]), which the CPU's
//! return predictor serves, instead of a probe and an indirect jump.
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
use core::sync::atomic::{AtomicPtr, AtomicU32, AtomicU64, Ordering, fence};

/// Bytes in a direct exit: its patch point with the pads around it
/// ([`EXIT_POINT_BYTES`]), `movabs rax, target`, `mov edx, word` and the
/// [`EXIT_RETURN`]. Only the unlinked path runs past the point.
pub const DIRECT_EXIT_BYTES: usize = 10 + DYNAMIC_EXIT_BYTES;
/// Bytes of a `jmp rel32`: what an install writes over the start of a
/// direct exit before the code is reachable.
pub const JUMP_BYTES: usize = 5;
/// Bytes of the one instruction a live patch changes ([`patchable`]): a
/// `jmp rel32`, the [`RET_ROOM`] it replaces, or the first bytes of a link
/// slot's body ([`link`]).
pub const PATCH_BYTES: usize = JUMP_BYTES;
/// The most NOP bytes before a patch point that keep its instruction in
/// one aligned 8-byte word ([`patch_pad`]).
const MAX_PAD: usize = 4;
/// Bytes of a [`link`]'s body, without and with the interruption gate, at
/// most ([`LinkGate::Jrcxz`]'s): a slot keeps room for either tier's.
pub const LINK_BODY_BYTES: usize = 33;
pub const POLLED_LINK_BODY_BYTES: usize = LINK_BODY_BYTES + 4;
/// Bytes in the largest [`thunk`]: the body and the jump to the exit.
pub const THUNK_BYTES: usize = POLLED_LINK_BODY_BYTES + JUMP_BYTES;
/// Bytes of the link slot before an exit in its page ([`link_slot`]): the
/// pad and the body, which starts at the patch point, without and with the
/// gate.
pub const LINK_BYTES: usize = MAX_PAD + LINK_BODY_BYTES;
pub const POLLED_LINK_BYTES: usize = MAX_PAD + POLLED_LINK_BODY_BYTES;
/// The span within which a link needs no translation of its own.
const LINK_PAGE: u64 = 4096;
/// Bytes in an exit to a dynamic target in RAX: its patch point with the
/// pads around it ([`EXIT_POINT_BYTES`], [`dynamic_patch`]), then
/// `mov edx, kind` and the [`EXIT_RETURN`], which only the unlinked path
/// runs: a linked point jumps to a tail that sets EDX where it needs it.
pub const DYNAMIC_EXIT_BYTES: usize = EXIT_POINT_BYTES + 5 + EXIT_RETURN_BYTES;
/// Bytes every exit starts with: a NOP of the pad that makes its patch
/// point patchable where it lies ([`patch_pad`]), the point, and a NOP of
/// the rest of [`MAX_PAD`], so what follows lies at one offset of the exit
/// however it is padded (a target's guest address field among it).
pub const EXIT_POINT_BYTES: usize = MAX_PAD + PATCH_BYTES;
/// [`crate::GuestState::exit_rsp`], from RDI.
pub const EXIT_RSP_OFFSET: i32 = core::mem::offset_of!(crate::GuestState, exit_rsp) as i32;
/// Bytes of [`EXIT_RETURN`].
pub const EXIT_RETURN_BYTES: usize = 8;
/// Every return of native code to the engine: `mov rsp, [rdi + exit_rsp]`
/// and `ret`. RSP is then where the entry's `call` left it, whatever words
/// native code left on the host stack since, so the `ret` reaches the
/// entry.
pub const EXIT_RETURN: [u8; EXIT_RETURN_BYTES] = {
    let d = EXIT_RSP_OFFSET.to_le_bytes();
    [0x48, 0x8b, 0xa7, d[0], d[1], d[2], d[3], 0xc3]
};
/// What an entry runs right before it calls native code (RSP 8 bytes above
/// where the `call` leaves it, RDI at the state): `lea r11, [rsp - 8]` and
/// `mov [rdi + exit_rsp], r11`.
pub const EXIT_RSP_STORE: [u8; 12] = {
    let d = EXIT_RSP_OFFSET.to_le_bytes();
    [
        0x4c, 0x8d, 0x5c, 0x24, 0xf8, 0x4c, 0x89, 0x9f, d[0], d[1], d[2], d[3],
    ]
};
/// An unlinked exit's patch point: a 5-byte NOP into the return to the
/// engine after it ([`unlinked_return`]).
pub const RET_ROOM: [u8; PATCH_BYTES] = NOP5;
/// A linked slot's patch point ([`link`]): the first bytes of its body's
/// first instruction, `mov rdx, [rdi + anchors + 8 * Control]`, the same
/// in every slot.
pub const LINKED_SLOT: [u8; PATCH_BYTES] = {
    let anchor = core::mem::offset_of!(crate::GuestState, anchors) + Anchor::Control as usize * 8;
    let d = (anchor as u32).to_le_bytes();
    [0x48, 0x8b, 0x97, d[0], d[1]]
};
/// The 5-byte NOP (Intel SDM vol. 2, NOP).
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

/// The block table entries an arena of `code_bytes` full of blocks takes,
/// which the host sizes from its memory: the power of two at least one
/// entry per [`CODE_BYTES_PER_ENTRY`]. A board's table starts at
/// [`first_lookup_entries`] and grows to it as its blocks fill it. Rounded down, a 976 MiB
/// window took 2 M entries for its 2.9 M blocks, and half of the entries
/// fetches would add were put off as other inserts kept displacing them
/// (run 20261111-025825: `targets=` deferred 513 K of 1.09 M a beat).
pub fn lookup_entries(code_bytes: usize) -> usize {
    (code_bytes / CODE_BYTES_PER_ENTRY)
        .max(1)
        .next_power_of_two()
        .clamp(MIN_LOOKUP_ENTRIES, MAX_LOOKUP_ENTRIES)
}

/// Arena bytes per entry of a board's first block table, which doubles
/// as its blocks fill it ([`LookupTable::grow`]): half of
/// [`CODE_BYTES_PER_ENTRY`], so a window whose blocks stay few keeps half
/// the table. Run 20261120 M4n held 1.22 M blocks in a 640 MiB window,
/// whose 4 M entries (128 MiB) were 29% full.
const FIRST_CODE_BYTES_PER_ENTRY: usize = 2 * CODE_BYTES_PER_ENTRY;

/// The block table entries a board starts with for an arena of
/// `code_bytes`: the power of two at least one entry per
/// [`FIRST_CODE_BYTES_PER_ENTRY`].
pub fn first_lookup_entries(code_bytes: usize) -> usize {
    (code_bytes / FIRST_CODE_BYTES_PER_ENTRY)
        .max(1)
        .next_power_of_two()
        .clamp(MIN_LOOKUP_ENTRIES, MAX_LOOKUP_ENTRIES)
}

/// A block table doubles once its live blocks reach this many tenths of
/// its entries ([`LookupTable::grow`]).
const GROW_LOAD_TENTHS: usize = 7;

/// The sets of the old table one growth step copies, under the board's
/// lock: 0.4 ms on the host (a 2 M-entry table's growth took 64 steps,
/// the longest 0.55 ms), far below the 0.5 s an exit may take.
const GROW_STEP_SETS: usize = 1 << 14;

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

/// The host register that holds the run's [`Account`] while native code
/// runs: R13, which X17 gave up ([`super::pinned::PINNED`]). Emitters never
/// take it for scratch.
pub(crate) const ACCOUNT: Gpr = Gpr::R13;

/// `destination` = the instructions the account counts
/// ([`Account::retired`]), signed.
pub(crate) fn account_retired(asm: &mut crate::x86_64::asm::Asm, destination: Gpr) {
    asm.mov(Qword, destination, ACCOUNT);
    asm.shift_imm(ShiftOp::Sar, Qword, destination, ACCOUNT_LANE_BITS as u8);
}

/// Add `instructions`, a signed count in RAX, to the account's
/// instructions: a self-loop's or region's settle before an exit. Clobbers
/// RAX and the flags.
pub(crate) fn account_add(asm: &mut crate::x86_64::asm::Asm) {
    asm.shift_imm(ShiftOp::Shl, Qword, Rax, ACCOUNT_LANE_BITS as u8);
    asm.alu(Alu::Add, Qword, ACCOUNT, Rax);
}
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
    /// Whether no interruption is pending, which every polled link, tail and
    /// probe reads on the cache line of `block` that it writes anyway: the
    /// epoch protocol keeps it ([`interrupt`], [`acknowledge`]).
    pub quiet: Quiet,
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
    /// Where cell probes count their transfers, in its low 32 bits: the
    /// engine zeroes it before each native call and adds it to its counts
    /// after.
    pub heat: u64,
    /// Sources are published before a notifier's increment ([`interrupt`]).
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
    /// The host address of the cell of the last transfer a cell probe
    /// named, one in [`CELL_SAMPLE`] of those it counts in `heat`, or 0:
    /// the engine zeroes it before each native call and reads it after.
    pub sampled: u64,
    /// Exits of blocks that ran their iterations inside themselves
    /// (`x86_64::block`'s self-loops), one per entry, and the instructions
    /// those entries retired past their first iteration (net of what joins
    /// skipped): the engine's counts, which only those exits write.
    pub loop_entries: u64,
    pub loop_iterated: u64,
    /// The most bytes of paired frames ([`paired_call`]) one native run
    /// keeps on the host stack below the entry's: 0 pushes none, and a call
    /// that would go past it takes the plain exit.
    pub frame_room: u32,
    /// Native code neither reads nor writes it.
    pub reserved: u32,
    /// The host address of the board's retire epoch, which every step that
    /// retires blocks advances, or 0 with `frame_room` 0. A frame holds
    /// the epoch of its push; a return takes it only while the epoch is
    /// unchanged, as its continuation may lie in a block retired since.
    pub retire_epoch: u64,
    /// What paired frames did ([`PairCount`]): native code's counts, which
    /// only grow.
    pub pairs: [u64; PAIR_COUNTS],
}

/// The counts of [`ChainControl::pairs`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PairCount {
    /// Returns that took their frame's continuation.
    Matched,
    /// Returns with frames on the stack that took the exit's lookup.
    Missed,
    /// Calls past the frame bound, which took the plain exit.
    Refused,
}

/// Entries of [`ChainControl::pairs`].
pub const PAIR_COUNTS: usize = 3;

impl PairCount {
    const fn offset(self) -> i32 {
        PAIRS + 8 * self as i32
    }
}

/// The paired frames the engine lets one native run keep by default
/// ([`ChainControl::frame_room`]).
pub const DEFAULT_FRAME_BOUND: u32 = 64;
/// Bytes of one paired frame on the host stack, from the top of the stack
/// down: the host return address, the guest return address, the calling
/// block's word, the retire epoch and the lower and upper half's contexts
/// ([`FrameWord`]). A multiple of 16, so code between blocks keeps its
/// stack alignment.
pub const FRAME_BYTES: usize = 48;

/// The words of a paired frame above its host return address, by their
/// offset from RSP while the return address is on top.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum FrameWord {
    Address = 8,
    Block = 16,
    Epoch = 24,
    LowerContext = 32,
    UpperContext = 40,
}
const _: () = assert!(FrameWord::UpperContext as usize + 8 == FRAME_BYTES);

// Epoch and observed state are naturally aligned atomic-width words. Capture
// and storage owners keep the mapped control alive for every embedded address.
const _: () = assert!(core::mem::offset_of!(ChainControl, interrupt_epoch) == 64);
const _: () = assert!(core::mem::offset_of!(ChainControl, observed_interrupt_epoch) == 72);
const _: () = assert!(core::mem::size_of::<ChainControl>() == 184);
// The gates read the quiet word with no displacement, on `block`'s line.
const _: () = assert!(core::mem::offset_of!(ChainControl, quiet) == 0);

/// The bits of an [`Account`] below its instructions: the transfers left.
pub const ACCOUNT_LANE_BITS: u32 = 16;
/// The most transfers one native run may allow.
pub const MAX_BUDGET: u64 = ACCOUNT_LANE;
/// The bits of an [`Account`]'s transfers part.
const ACCOUNT_LANE: u64 = (1 << ACCOUNT_LANE_BITS) - 1;

/// [`ChainControl::quiet`] while no interruption is pending: the account's
/// whole transfers part, so a tail's `test [control], r13w` finds a
/// transfer left and no interruption in one AND. 0 once one is pending.
pub const QUIET: u64 = ACCOUNT_LANE;

/// [`ChainControl::quiet`]: [`QUIET`] or 0. A control is made quiet.
#[repr(transparent)]
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Quiet(pub u64);

impl Default for Quiet {
    fn default() -> Self {
        Self(QUIET)
    }
}

/// Post an interruption to the PE whose control is at `control`: its epoch
/// moves on (but past `u64::MAX`, where it stays), then its quiet word
/// becomes 0, both sequentially consistent, so the PE's next polled
/// transfer exits. Its owner's [`acknowledge`] of a snapshot taken before
/// the epoch moved either finds the move after it makes the word quiet, or
/// this zeroes the word after that.
///
/// # Safety
///
/// `control` is a live [`ChainControl`], whose epoch and quiet words every
/// other writer accesses atomically.
pub unsafe fn interrupt(control: *mut ChainControl) {
    // SAFETY: the caller's; both words are naturally aligned.
    let (epoch, quiet) = unsafe {
        (
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*control).interrupt_epoch)),
            AtomicU64::from_ptr(core::ptr::addr_of_mut!((*control).quiet.0)),
        )
    };
    let _ = epoch.try_update(Ordering::SeqCst, Ordering::Relaxed, |epoch| {
        epoch.checked_add(1)
    });
    quiet.store(0, Ordering::SeqCst);
}

/// The owner's acknowledgment of `epoch`, its snapshot of the control's
/// epoch taken before it drained the sources: the observed epoch becomes
/// it (at most `u64::MAX - 1`, so a saturated epoch stays pending), and the
/// quiet word [`QUIET`] unless the epoch moved since, which a notifier's
/// [`interrupt`] may have done before this made the word quiet.
///
/// # Safety
///
/// As for [`interrupt`]; the caller owns the control, outside its native
/// calls.
pub unsafe fn acknowledge(control: *mut ChainControl, epoch: u64) {
    let observed = epoch.min(u64::MAX - 1);
    // SAFETY: the caller's; both words are naturally aligned, and only the
    // owner writes the observed epoch.
    unsafe {
        core::ptr::addr_of_mut!((*control).observed_interrupt_epoch).write(observed);
        let epoch = AtomicU64::from_ptr(core::ptr::addr_of_mut!((*control).interrupt_epoch));
        let quiet = AtomicU64::from_ptr(core::ptr::addr_of_mut!((*control).quiet.0));
        quiet.store(QUIET, Ordering::SeqCst);
        if epoch.load(Ordering::SeqCst) != observed {
            quiet.store(0, Ordering::SeqCst);
        }
    }
}

/// A native run's account, which [`ACCOUNT`] holds while native code runs
/// and `GuestState::account` outside it: the instructions of the blocks
/// transfers left, a signed count, from bit [`ACCOUNT_LANE_BITS`] up, and
/// the transfers still allowed below it. A transfer checks the lane first
/// and exits at 0, else adds `ran << ACCOUNT_LANE_BITS` less one in one
/// LEA, which never borrows; nothing is ever taken back. The instructions
/// are signed as a self-loop's or region's settle before an exit may add
/// less than zero (`GuestState::loop_retired` short of the exit's rows),
/// which the exit's rows then repay, in the link that counts them or in the
/// rows the engine adds; the lane is apart from them, so a settle never
/// borrows a transfer.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Account(pub u64);

impl Account {
    /// The account of a run that allows `budget` transfers, at most
    /// [`MAX_BUDGET`], and has counted no instructions.
    pub const fn new(budget: u64) -> Self {
        debug_assert!(budget <= MAX_BUDGET);
        Self(budget)
    }

    /// The account of a run with no transfer left, having counted
    /// `retired` instructions.
    pub const fn spent(retired: u64) -> Self {
        Self(retired << ACCOUNT_LANE_BITS)
    }

    /// The transfers still allowed: 0 once the run used its budget, whether
    /// or not a transfer then found none.
    pub const fn budget(self) -> u64 {
        self.0 & ACCOUNT_LANE
    }

    /// The instructions of the blocks transfers left, a signed count: one
    /// below zero reads as its two's complement, which the rows the engine
    /// adds for the exit complete.
    pub const fn retired(self) -> u64 {
        ((self.0 as i64) >> ACCOUNT_LANE_BITS) as u64
    }

    /// `budget` transfers allowed, the instructions counted kept.
    pub const fn with_budget(self, budget: u64) -> Self {
        debug_assert!(budget <= MAX_BUDGET);
        Self(self.0 & !ACCOUNT_LANE | budget)
    }

    /// `retired` instructions counted, the budget kept: an exhausted one
    /// stays exhausted.
    pub const fn with_retired(self, retired: u64) -> Self {
        Self(retired << ACCOUNT_LANE_BITS | self.0 & ACCOUNT_LANE)
    }
}

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

    /// `len` slots of which only the header is written: for a table whose
    /// every slot [`Self::write`] fills before anything reads it
    /// ([`LookupTable::grow`]), so its bytes are not written twice.
    fn unwritten(len: usize) -> Self {
        let layout = Self::layout(len + HEADER_SLOTS);
        // SAFETY: a nonzero size.
        let start = unsafe { alloc::alloc::alloc(layout) } as *mut Slot;
        let Some(start) = core::ptr::NonNull::new(start) else {
            alloc::alloc::handle_alloc_error(layout);
        };
        // SAFETY: the header slots are the allocation's first; zero bytes
        // are valid slots.
        unsafe { core::ptr::write_bytes(start.as_ptr(), 0, HEADER_SLOTS) };
        // SAFETY: as in `zeroed`.
        let base = unsafe { start.add(HEADER_SLOTS) };
        Self { base, len }
    }

    /// Write `entry` in slot `index` of a table no other core reads yet,
    /// whether or not the slot was written before ([`Self::unwritten`]).
    fn write(&self, index: usize, entry: LookupEntry) {
        assert!(index < self.len, "a slot of the table");
        let slot = Slot {
            pc: AtomicU64::new(entry.pc),
            key: AtomicU64::new(entry.key),
            entry: AtomicU64::new(entry.entry),
            block: AtomicU32::new(entry.block),
            meta: AtomicU32::new(entry.targets & META_TARGETS),
        };
        // SAFETY: in bounds of the live allocation; no reference to the
        // slot is held, as no reader has the table yet and its writer
        // takes none across this.
        unsafe { self.base.as_ptr().add(index).write(slot) };
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

/// A table's slots and slot mask as its writer published them
/// ([`TableView`]).
struct Generation {
    slots: Arc<Slots>,
    mask: u32,
}

/// The table a [`LookupTable`]'s readers read: the one its writer published
/// last, and the one before while a growth's readers may still hold it
/// ([`LookupTable::grow`], [`LookupTable::release_retired`]). Each is a
/// leaked box this view frees.
struct TableView {
    current: AtomicPtr<Generation>,
    retired: AtomicPtr<Generation>,
}

impl TableView {
    fn new(slots: Arc<Slots>, mask: u32) -> Self {
        let current = Box::into_raw(Box::new(Generation { slots, mask }));
        Self {
            current: AtomicPtr::new(current),
            retired: AtomicPtr::new(core::ptr::null_mut()),
        }
    }
}

impl Drop for TableView {
    fn drop(&mut self) {
        for held in [self.current.get_mut(), self.retired.get_mut()] {
            if !held.is_null() {
                // SAFETY: made by `Box::into_raw`, and no reader is left.
                drop(unsafe { Box::from_raw(*held) });
            }
        }
    }
}

/// A handle that reads a table's entries from any core without its writer's
/// lock ([`LookupTable::reader`]): the table its writer published last,
/// which a growth replaces with one of twice the entries.
#[derive(Clone)]
pub struct TableReader {
    view: Arc<TableView>,
}

impl TableReader {
    /// The table published last.
    #[inline]
    fn generation(&self) -> &Generation {
        // SAFETY: published whole (a release store after its last write);
        // a replaced one is freed only once no reader holds it
        // ([`LookupTable::release_retired`]).
        unsafe { &*self.view.current.load(Ordering::Acquire) }
    }

    /// What [`LookupTable::find`] finds. A `slot` of a table published
    /// before finds what the probe of a stale table would: at most a miss,
    /// as a later table is larger.
    #[inline]
    pub fn find(&self, slot: usize, pc: u64, key: u64) -> LookupEntry {
        find_in(&self.generation().slots, slot, pc, key)
    }

    /// The slot of `pc` with `key` ([`lookup_slot`]).
    #[inline]
    pub fn slot(&self, pc: u64, key: u64) -> usize {
        lookup_slot(pc, key, self.generation().mask)
    }

    /// The entry of `pc` with `key` as a probe would find it
    /// ([`Self::find`] at [`Self::slot`]), in one table.
    #[inline]
    pub fn lookup(&self, pc: u64, key: u64) -> LookupEntry {
        let generation = self.generation();
        find_in(
            &generation.slots,
            lookup_slot(pc, key, generation.mask),
            pc,
            key,
        )
    }

    /// The table's host address, for `ChainControl::table`.
    pub fn address(&self) -> u64 {
        self.generation().slots.address()
    }

    /// The slot mask, for `ChainControl::masks`.
    pub fn mask(&self) -> u32 {
        self.generation().mask
    }

    /// The host address and slot mask of one table, for a PE's
    /// `ChainControl` at its next native run.
    pub fn current(&self) -> (u64, u32) {
        let generation = self.generation();
        (generation.slots.address(), generation.mask)
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
    /// The table's sets when `sets` were numbered: a growth doubles them,
    /// and set `s` of its entries is `s` or `s + at` after ([`renumber`]).
    at: u32,
}

/// Number `listed`'s sets for a table of `sets` sets, after the growths
/// since they were numbered: each set's entries are in it or in the set
/// as many sets on. Sets that hold none of the group's entries go at its
/// next compaction or sweep.
fn renumber(listed: &mut Granule, sets: u32) {
    if listed.sets.is_empty() {
        listed.at = sets;
        return;
    }
    while listed.at < sets {
        let at = listed.at;
        for index in 0..listed.sets.len() {
            let set = listed.sets[index];
            listed.sets.push(set + at);
        }
        listed.limit *= 2;
        listed.at = 2 * at;
    }
}

/// The new table of a growth under way ([`LookupTable::grow`]): the old
/// sets below `next` are copied, and each write to one copies it again.
struct Growth {
    sets: Arc<Slots>,
    mask: u32,
    next: usize,
}

/// Copy set `set` of `old` to the two sets of `growth`'s table its entries
/// go to, `set` and `set` plus the old table's sets, each entry in the
/// slot its PC and key pick there or beside it: of an old set's two
/// entries, each new set takes at most both.
fn copy_set(old: &Slots, set: usize, growth: &Growth) {
    let sets = old.len / 2;
    for at in [2 * set, 2 * set + 1, 2 * (set + sets), 2 * (set + sets) + 1] {
        growth.sets.write(at, LookupEntry::default());
    }
    let mut taken = [usize::MAX; 2];
    for (position, at) in [2 * set, 2 * set + 1].into_iter().enumerate() {
        let entry = old.get(at).load();
        if entry.key == 0 {
            continue;
        }
        let home = lookup_slot(entry.pc, entry.key, growth.mask);
        let slot = if taken[0] == home {
            neighbour(home)
        } else {
            home
        };
        taken[position] = slot;
        growth.sets.write(slot, entry);
    }
}

/// Keep what a write to set `set` of `sets` (slot mask `mask`) leaves in
/// the tables beside it: a growth's copy of the set, and an old table
/// that PEs may still probe, which holds only entries the current one
/// holds alike (an entry forgotten or replaced goes from both).
fn mirror(
    sets: &Slots,
    mask: u32,
    set: usize,
    growth: Option<&Growth>,
    old: Option<&(Arc<Slots>, u32)>,
) {
    if let Some(growth) = growth
        && set < growth.next
    {
        copy_set(sets, set, growth);
    }
    if let Some((old, _)) = old {
        let old_set = set & (old.len / 2 - 1);
        for at in [2 * old_set, 2 * old_set + 1] {
            let held = old.get(at);
            if held.key() == 0 {
                continue;
            }
            let entry = held.load();
            if entry.key != 0
                && find_in(
                    sets,
                    lookup_slot(entry.pc, entry.key, mask),
                    entry.pc,
                    entry.key,
                ) != entry
            {
                held.store(LookupEntry::default());
            }
        }
    }
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
/// in one array ([`LookupTable::stamped_cells`]) followed by a `live` word
/// per cell, which only the exits of hot cells take
/// ([`LookupTable::link_hot_cell`]). Every other cell comes in chunks of
/// [`CELL_CHUNK`].
pub const STAMPED_CELLS: usize = 1 << 16;
/// Byte offset of a stamped cell's `live` word from the array: nonzero
/// while the cell holds what it held when its exit was linked.
pub const CELL_LIVE: usize = STAMPED_CELLS * size_of::<LookupEntry>();
/// A cell probe names one transfer in this many of its PE's in
/// `ChainControl::sampled` ([`cell_probe`]): the engine gives the exits
/// whose cells it names often a stamped cell and a [`stamped_tail`].
pub const CELL_SAMPLE: u32 = 256;
/// Bytes of one PE's stamp of one stamped cell: the context, then the ITLB
/// drop epoch of the target's half, in which the PE's cell probe found the
/// cell's target ([`stamped_tail`]).
pub const STAMP_BYTES: usize = 16;
/// Bytes of one PE's stamps.
pub const STAMP_TABLE_BYTES: usize = STAMPED_CELLS * STAMP_BYTES;
/// Bytes of the counts in front of one PE's stamps ([`StampCount`]).
pub const STAMP_COUNTS_BYTES: usize = 64;

/// Entries of a PE's jump cache, in front of its stamp counts: targets its
/// table probe entered, by guest PC, which a [`lookup_tail`] takes without
/// a probe. Bits 2-11 of the PC pick the entry. With 256 a lookup site
/// over 1024 targets (`transfer_path_costs`) missed 75% of its transfers
/// into the table probe, 14.4 ns a transfer on an i7-12700KF; with 1024 it
/// misses none, 8.8 ns, and 4096 gains nothing more. 64 KiB a PE.
///
/// An entry holds across native runs, while the target's half has the
/// context and ITLB drop epoch the entry names, as a stamp holds
/// ([`stamped_tail`]), and while the block table keeps the entry it was
/// filled from: every change of a table entry takes that PC's jump cache
/// entries from every PE's cache ([`forget_jumps`]). The probe that fills
/// an entry stores its PC with a locked exchange and reads the table
/// entry's last word again after it, taking the entry back when the word
/// moved; the table's writer stores the entry, fences and then reads the
/// caches, so one of the two sees the other.
pub const JUMP_CACHE_ENTRIES: usize = 1024;
/// Bytes of one jump cache entry, a cache line: the PC, the block's code,
/// its index and the table entry's last word as the probe read it, whose
/// low byte is the BTYPEs the block accepts ([`LookupEntry`]), then the
/// context and ITLB drop epoch of the target's half it holds in.
const JUMP_ENTRY_BYTES: usize = 64;
/// Bytes of a PE's jump cache, in front of its stamp counts.
pub const JUMP_CACHE_BYTES: usize = JUMP_CACHE_ENTRIES * JUMP_ENTRY_BYTES;
/// Byte offset from the stamps of the jump cache's first entry.
const JUMP_CACHE: i32 = -((STAMP_COUNTS_BYTES + JUMP_CACHE_BYTES) as i32);
/// The PC bits that pick an entry, scaled by 4: shifted left by
/// [`JUMP_INDEX_SHIFT`], an entry's offset.
const JUMP_INDEX: i32 = ((JUMP_CACHE_ENTRIES - 1) << 2) as i32;
const JUMP_INDEX_SHIFT: u8 = JUMP_ENTRY_BYTES.trailing_zeros() as u8 - 2;
const JUMP_PC: i32 = 0;
const JUMP_CODE: i32 = 8;
const JUMP_BLOCK: i32 = 16;
const JUMP_TARGETS: i32 = 20;
const JUMP_CONTEXT: i32 = 24;
const JUMP_EPOCH: i32 = 32;
const _: () = assert!(JUMP_EPOCH as usize + 8 <= JUMP_ENTRY_BYTES);
const _: () = assert!(JUMP_ENTRY_BYTES == 64 && JUMP_CACHE_ENTRIES.is_power_of_two());
const _: () = assert!(STAMP_COUNTS_BYTES.is_multiple_of(64));

/// The host address of the entry of `pc` in the jump cache in front of
/// `stamps` ([`LookupTable::new_stamps`]).
pub const fn jump_entry(stamps: u64, pc: u64) -> u64 {
    stamps
        .wrapping_add(JUMP_CACHE as i64 as u64)
        .wrapping_add((pc & JUMP_INDEX as u64) << JUMP_INDEX_SHIFT)
}

/// Take the entries of `pc` out of the jump caches in front of `stamps`,
/// after a change of the block table entry they may have been filled
/// from, which the caller stored before: the fence orders that store
/// before these reads, as the filling probe's locked exchange orders its
/// store of the PC before its read of the table entry ([`fill_jump_cache`]).
fn forget_jumps(stamps: &[u64], pc: u64) {
    if stamps.is_empty() {
        return;
    }
    fence(Ordering::SeqCst);
    for &table in stamps {
        // SAFETY: `new_stamps` made the cache in front of each of `stamps`
        // as atomics, which live while their table does.
        let held = unsafe { AtomicU64::from_ptr(jump_entry(table, pc) as *mut u64) };
        if held.load(Ordering::Relaxed) == pc {
            held.store(0, Ordering::Relaxed);
        }
    }
}

/// Take every entry out of the jump caches in front of `stamps`, as
/// [`forget_jumps`] does one PC's.
fn forget_all_jumps(stamps: &[u64]) {
    fence(Ordering::SeqCst);
    for &table in stamps {
        for entry in 0..JUMP_CACHE_ENTRIES {
            let at =
                (table as i64 + i64::from(JUMP_CACHE)) as u64 + (entry * JUMP_ENTRY_BYTES) as u64;
            // SAFETY: as for `forget_jumps`.
            unsafe { AtomicU64::from_ptr(at as *mut u64) }.store(0, Ordering::Relaxed);
        }
    }
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
    /// Misses for an interruption or no transfer left.
    Stopped,
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

/// What an exit through a cell was linked with
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

/// A [`CellLink`] as a table keeps it for every cell, its target in the
/// cell: the patch point (0 for none), the instructions its block ran with
/// the BTYPE above them, and its word.
#[derive(Debug, Clone, Copy, Default)]
struct CellNote {
    site: u64,
    ran: u32,
    word: u32,
}
const _: () = assert!(size_of::<CellNote>() == 16);
const NOTE_BTYPE_SHIFT: u32 = 24;

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
    /// What its readers read ([`Self::reader`]).
    view: Arc<TableView>,
    /// The doubled table a growth fills ([`Self::grow`]).
    growth: Option<Growth>,
    /// The table before the last growth and its slot mask, which PEs may
    /// probe until [`Self::release_retired`]: it holds only entries this
    /// one holds alike ([`mirror`]).
    old: Option<(Arc<Slots>, u32)>,
    /// Growths so far.
    grown: u32,
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
    /// Cells past the stamped ones no exit holds.
    free_cells: Vec<u32>,
    /// Stamped cells no exit holds, for hot exits ([`Self::link_hot_cell`]).
    free_stamped: Vec<u32>,
    /// The first cell of each chunk past the stamped ones by its address,
    /// which a cell probe's sample names ([`Self::sampled_link`]).
    chunks: BTreeMap<u64, u32>,
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
    /// How many held cells are past the stamped ones ([`STAMPED_CELLS`]),
    /// whose exits keep the cell probe.
    cells_past: usize,
    /// The host address of each PE's stamps ([`Self::new_stamps`]), whose
    /// stamps of a stamped cell go when the cell changes.
    stamp_tables: Vec<u64>,
    /// The stamps those addresses name, which never move.
    stamp_store: Vec<Box<[AtomicU64]>>,
    /// The link of each cell an exit holds, by cell number, for its
    /// promotion ([`Self::sampled_link`]).
    cell_links: Vec<CellNote>,
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
            view: Arc::new(TableView::new(Arc::clone(&sets), mask)),
            sets,
            mask,
            growth: None,
            old: None,
            grown: 0,
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
            free_stamped: Vec::new(),
            chunks: BTreeMap::new(),
            cell_pcs: BTreeSet::new(),
            cell_frames: BTreeSet::new(),
            cell_counts: Box::default(),
            cell_sites: BTreeMap::new(),
            cells_past: 0,
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
            view: Arc::clone(&self.view),
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
            + self.stamp_store.len()
                * (64 + JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES + STAMP_TABLE_BYTES)
            + (self.free_cells.capacity() + self.free_stamped.capacity()) * size_of::<u32>()
            + self.cell_links.capacity() * size_of::<CellNote>()
            + self.chunks.len() * 2 * size_of::<(u64, u32)>()
            + self.cell_counts.len() * size_of::<u32>()
            + (self.cell_pcs.len() + self.cell_frames.len() + self.cell_sites.len())
                * 2
                * size_of::<(u64, u32)>()
    }

    /// The bytes of its entries.
    pub fn table_bytes(&self) -> usize {
        let growth = self.growth.as_ref().map_or(0, |growth| growth.sets.len);
        let old = self.old.as_ref().map_or(0, |(old, _)| old.len);
        (self.sets.len + growth + old) * size_of::<LookupEntry>()
    }

    /// The table's host address, for `ChainControl::table` or `itlb`.
    pub fn address(&self) -> u64 {
        self.sets.address()
    }

    /// Take a step of doubling a block table that `live` blocks have
    /// filled to [`GROW_LOAD_TENTHS`], up to [`MAX_LOOKUP_ENTRIES`]:
    /// whether this step published the doubled table to its readers
    /// ([`Self::reader`]). One step copies [`GROW_STEP_SETS`] sets, so the
    /// writer's hold stays short; meanwhile the table works as before,
    /// and a write to a copied set copies it again, so the new table is
    /// whole when it is published. None begins while the old one is held
    /// ([`Self::release_retired`]).
    pub fn grow(&mut self, live: usize) -> bool {
        if self.grouping != Grouping::Frame {
            return false;
        }
        let entries = self.sets.len;
        if self.growth.is_none() {
            if self.old.is_some()
                || entries >= MAX_LOOKUP_ENTRIES
                || live * 10 < entries * GROW_LOAD_TENTHS
            {
                return false;
            }
            let sets = Slots::unwritten(2 * entries);
            for (word, held) in sets.header().iter().zip(self.sets.header()) {
                word.store(held.load(Ordering::Relaxed), Ordering::Relaxed);
            }
            self.growth = Some(Growth {
                sets: Arc::new(sets),
                mask: (2 * entries - 1) as u32,
                next: 0,
            });
        }
        let growth = self.growth.as_mut().expect("a growth under way");
        let end = (growth.next + GROW_STEP_SETS).min(entries / 2);
        for set in growth.next..end {
            copy_set(&self.sets, set, growth);
        }
        growth.next = end;
        if end < entries / 2 {
            return false;
        }
        let growth = self.growth.take().expect("a growth under way");
        let old = core::mem::replace(&mut self.sets, growth.sets);
        let old_mask = core::mem::replace(&mut self.mask, growth.mask);
        let published = Box::into_raw(Box::new(Generation {
            slots: Arc::clone(&self.sets),
            mask: self.mask,
        }));
        let replaced = self.view.current.swap(published, Ordering::AcqRel);
        let held = self.view.retired.swap(replaced, Ordering::AcqRel);
        debug_assert!(held.is_null(), "one retired table at a time");
        self.old = Some((old, old_mask));
        self.grown += 1;
        true
    }

    /// Whether a growth is under way.
    pub fn growing(&self) -> bool {
        self.growth.is_some()
    }

    /// Whether the table before the last growth is still held
    /// ([`Self::release_retired`]).
    pub fn holds_retired(&self) -> bool {
        self.old.is_some()
    }

    /// The growths so far.
    pub fn growths(&self) -> u32 {
        self.grown
    }

    /// Free the table the last growth replaced.
    ///
    /// # Safety
    ///
    /// No reader holds it: every reader that may have read the table
    /// before the growth published its successor has since read again
    /// ([`TableReader`] calls end before it) and no core's code probes it
    /// (each PE's `ChainControl` names the new one): a grace period of the
    /// board's PEs that began after the publication.
    pub unsafe fn release_retired(&mut self) {
        self.old = None;
        let retired = self
            .view
            .retired
            .swap(core::ptr::null_mut(), Ordering::AcqRel);
        if !retired.is_null() {
            // SAFETY: made by `Box::into_raw` in `grow`; the caller's
            // contract: no reader holds it.
            drop(unsafe { Box::from_raw(retired) });
        }
    }

    /// Forget every entry, the cells' too; exits keep their cells.
    pub fn clear(&mut self) {
        let old = self.old.as_ref().map(|(old, _)| old.as_ref());
        for slot in self
            .sets
            .iter()
            .chain(old.into_iter().flat_map(Slots::iter))
        {
            if slot.key() != 0 {
                slot.store(LookupEntry::default());
            }
        }
        // Begun again, from the empty table, by the next insert.
        self.growth = None;
        if !self.stamp_tables.is_empty() {
            forget_all_jumps(&self.stamp_tables);
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
    /// the exit's until [`Self::release_cells`] reaches `site`. It is past
    /// the stamped ones: a stamped cell is for a hot exit
    /// ([`Self::link_hot_cell`]).
    pub fn link_cell(&mut self, site: u64, pc: u64, current: Option<LookupEntry>) -> u64 {
        self.release_cells(site..=site);
        self.stamped_cells();
        let cell = self.free_cells.pop().unwrap_or_else(|| {
            let cell = (STAMPED_CELLS + (self.cells.len() - 1) * CELL_CHUNK) as u32;
            let chunk = Slots::zeroed(CELL_CHUNK);
            self.chunks.insert(chunk.address(), cell);
            self.cells.push(chunk);
            self.free_cells
                .extend((cell + 1..cell + CELL_CHUNK as u32).rev());
            cell
        });
        self.link_numbered(site, pc, current, cell)
    }

    /// [`Self::link_cell`] with a stamped cell, which a [`stamped_tail`] can
    /// name, for an exit whose cell probe transfers are many: `None` while
    /// every stamped cell is held.
    pub fn link_hot_cell(
        &mut self,
        site: u64,
        pc: u64,
        current: Option<LookupEntry>,
    ) -> Option<u64> {
        self.stamped_cells();
        if self.free_stamped.is_empty() {
            return None;
        }
        self.release_cells(site..=site);
        let cell = self.free_stamped.pop()?;
        Some(self.link_numbered(site, pc, current, cell))
    }

    /// Whether a stamped cell is free for [`Self::link_hot_cell`].
    pub fn has_free_stamped(&self) -> bool {
        self.cells.is_empty() || !self.free_stamped.is_empty()
    }

    fn link_numbered(
        &mut self,
        site: u64,
        pc: u64,
        current: Option<LookupEntry>,
        cell: u32,
    ) -> u64 {
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
            // After the entry: a probe that finds the word set read the
            // entry it was linked with.
            self.cells[0]
                .word(CELL_LIVE + 4 * cell as usize)
                .store(1, Ordering::Release);
        }
        self.cell_pcs.insert((pc, cell));
        if self.cell_counts.is_empty() {
            self.cell_counts = vec![0; CELL_BUCKETS].into_boxed_slice();
        }
        self.cell_counts[cell_bucket(pc)] += 1;
        self.cell_sites.insert(site, cell);
        if cell as usize >= STAMPED_CELLS {
            self.cells_past += 1;
        }
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
            if cell as usize >= STAMPED_CELLS {
                self.cells_past -= 1;
            }
            if let Some(link) = self.cell_links.get_mut(cell as usize) {
                *link = CellNote::default();
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
            match (cell as usize) < STAMPED_CELLS {
                true => self.free_stamped.push(cell),
                false => self.free_cells.push(cell),
            }
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

    /// The cells exits hold past the stamped ones ([`STAMPED_CELLS`]):
    /// their exits keep the cell probe until their transfers make them hot
    /// ([`Self::link_hot_cell`]).
    pub fn cells_past_stamped(&self) -> usize {
        self.cells_past
    }

    /// The cells exits hold that hold no entry: their cell probes send
    /// every transfer on to the table probe until the table takes an entry
    /// of the cell's PC.
    pub fn cells_empty(&self) -> usize {
        self.cell_sites.len().saturating_sub(self.cell_frames.len())
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
    /// `ChainControl::cells`, made with their `live` words on first use.
    pub fn stamped_cells(&mut self) -> u64 {
        if self.cells.is_empty() {
            const _: () = assert!((STAMPED_CELLS * 4).is_multiple_of(size_of::<LookupEntry>()));
            let live = STAMPED_CELLS * 4 / size_of::<LookupEntry>();
            self.cells.push(Slots::zeroed(STAMPED_CELLS + live));
            // Cell 0 is never held: no stamp is at the stamps' address.
            self.free_stamped.extend((1..STAMPED_CELLS as u32).rev());
        }
        self.cells[0].address()
    }

    /// New zeroed stamps for one PE ([`STAMP_TABLE_BYTES`]) after their
    /// counts ([`STAMP_COUNTS_BYTES`]) and its jump cache ([`JUMP_CACHE_BYTES`]),
    /// for `ChainControl::stamps`, which
    /// the table keeps while it lives and zeroes a cell's stamps in when
    /// the cell changes.
    pub fn new_stamps(&mut self) -> u64 {
        // A cache line more, for the jump cache's entries to start one.
        let words = (64 + JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES + STAMP_TABLE_BYTES) / 8;
        let stamps: Box<[AtomicU64]> = (0..words).map(|_| AtomicU64::new(0)).collect();
        let address = (stamps.as_ptr() as u64).next_multiple_of(64)
            + (JUMP_CACHE_BYTES + STAMP_COUNTS_BYTES) as u64;
        self.stamp_store.push(stamps);
        self.stamp_tables.push(address);
        address
    }

    /// Whether `stamps` is an address [`Self::new_stamps`] of this table
    /// returned.
    pub fn holds_stamps(&self, stamps: u64) -> bool {
        self.stamp_tables.contains(&stamps)
    }

    /// Note what the exit holding cell `cell` ran, its word and its BTYPE,
    /// for a promotion of the exit ([`Self::sampled_link`]); a link whose
    /// fields do not fit is not noted.
    pub fn note_link(&mut self, cell: u32, link: CellLink) {
        if link.retired >= 1 << NOTE_BTYPE_SHIFT || link.site == 0 {
            return;
        }
        if self.cell_links.len() <= cell as usize {
            self.cell_links
                .resize(cell as usize + 1, CellNote::default());
        }
        self.cell_links[cell as usize] = CellNote {
            site: link.site,
            ran: link.retired | u32::from(link.btype) << NOTE_BTYPE_SHIFT,
            word: link.word.encoded(),
        };
    }

    /// The number and link of the held cell at host address `address`, as
    /// a cell probe names it (`ChainControl::sampled`), if an exit holds it
    /// and its link was noted ([`Self::note_link`]).
    pub fn sampled_link(&self, address: u64) -> Option<(u32, CellLink)> {
        let stamped = self.cells.first()?.address();
        let cell = match address.checked_sub(stamped) {
            Some(offset) if offset < CELL_LIVE as u64 => offset / size_of::<LookupEntry>() as u64,
            _ => {
                let (&chunk, &first) = self.chunks.range(..=address).next_back()?;
                let index = (address - chunk) / size_of::<LookupEntry>() as u64;
                if index >= CELL_CHUNK as u64 {
                    return None;
                }
                u64::from(first) + index
            }
        };
        let cell = u32::try_from(cell).ok()?;
        let note = *self.cell_links.get(cell as usize)?;
        if note.site == 0 || self.cell_sites.get(&note.site) != Some(&cell) {
            return None;
        }
        let (kind, skipped) = ExitWord::split(u64::from(note.word));
        let link = CellLink {
            site: note.site,
            pc: self.cell(cell).pc,
            retired: note.ran & ((1 << NOTE_BTYPE_SHIFT) - 1),
            word: ExitWord {
                kind: ExitKind::try_from(kind).ok()?,
                skipped,
            },
            btype: (note.ran >> NOTE_BTYPE_SHIFT) as u8,
        };
        Some((cell, link))
    }

    /// Note that the exit holding stamped cell `cell` has a stamped tail
    /// now, until it releases the cell.
    pub fn note_stamped(&mut self, cell: u32) {
        if (cell as usize) < STAMPED_CELLS {
            self.stamped.insert(cell);
        }
    }

    /// Whether stamped cell `cell` still holds the entry its exit's stamped
    /// tail jumps to: [`put_cell`] clears its `live` word when the entry
    /// changes, and then every stamped transfer of it misses to the cell
    /// probe, which stamps it no more.
    pub fn stamped_live(&self, cell: u32) -> bool {
        (cell as usize) < STAMPED_CELLS
            && !self.cells.is_empty()
            && self.cells[0]
                .word(CELL_LIVE + 4 * cell as usize)
                .load(Ordering::Acquire)
                != 0
    }

    /// Set the `live` word of stamped cell `cell` again, for an exit whose
    /// patch point now jumps to a stamped tail of `entry`, the entry the
    /// cell holds: false, with the cell as it was, when it holds another,
    /// holds none or is no exit's stamped cell. Every PE's stamps of the
    /// cell are zero while the word is clear ([`stamp`]), so a stale tail
    /// that a core is still in misses as before.
    pub fn revive_stamped(&mut self, cell: u32, entry: LookupEntry) -> bool {
        if entry.key == 0
            || !self.stamped.contains(&cell)
            || Self::cell_slot(&self.cells, cell).load() != entry
        {
            return false;
        }
        self.cells[0]
            .word(CELL_LIVE + 4 * cell as usize)
            .store(1, Ordering::Release);
        true
    }

    /// How many exits have a stamped tail ([`Self::note_stamped`]).
    pub fn stamped_tails(&self) -> usize {
        self.stamped.len()
    }

    /// The host address of cell `cell`, as its exit's tail names it.
    pub fn cell_address(&self, cell: u32) -> u64 {
        Self::cell_slot(&self.cells, cell) as *const Slot as u64
    }

    /// The cell number of the exit at host address `site`, if it holds one.
    pub fn site_cell_number(&self, site: u64) -> Option<u32> {
        self.cell_sites.get(&site).copied()
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
    ///
    /// The entry's older self beside it, which a resident put there, leaves
    /// the set, its place free for the resident, and its PC the jump caches
    /// ([`Self::put`]). Kept, it would come back when the new entry leaves,
    /// and a jump cache entry filled from it would go on entering the block
    /// and BTYPEs it named while the probes take the new entry's.
    pub fn insert(&mut self, slot: usize, entry: LookupEntry) {
        if self.grouping == Grouping::Pc {
            self.note_rekey(slot, &entry);
        }
        let resident = self.entry(slot);
        if resident == entry {
            return;
        }
        let beside = self.entry(neighbour(slot));
        let older_beside = beside.key != 0 && beside.pc == entry.pc && beside.key == entry.key;
        let other_resident =
            resident.key != 0 && (resident.pc != entry.pc || resident.key != entry.key);
        if older_beside && !other_resident {
            self.put(neighbour(slot), LookupEntry::default());
        }
        if other_resident {
            let keep_resident = if beside.key == 0 || older_beside {
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
                self.put(neighbour(slot), resident);
            }
        }
        self.set(slot, entry);
    }

    /// Store `entry` in `slot`; the entry it puts out of a block table
    /// goes from every PE's jump cache ([`forget_jumps`]).
    fn put(&self, slot: usize, entry: LookupEntry) {
        let held = self.sets.get(slot);
        let old = (!self.stamp_tables.is_empty()).then(|| held.load());
        held.store(entry);
        mirror(
            &self.sets,
            self.mask,
            slot / 2,
            self.growth.as_ref(),
            self.old.as_ref(),
        );
        if let Some(old) = old
            && old.key != 0
            && old != entry
        {
            forget_jumps(&self.stamp_tables, old.pc);
        }
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
        self.put(slot, entry);
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
        renumber(listed, (sets.len / 2) as u32);
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
            renumber(listed, (sets.len / 2) as u32);
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
                mirror(
                    &self.sets,
                    self.mask,
                    at / 2,
                    self.growth.as_ref(),
                    self.old.as_ref(),
                );
                forget_jumps(&self.stamp_tables, pc);
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
        let (mask, growth, old) = (self.mask, self.growth.as_ref(), self.old.as_ref());
        let stamps = &self.stamp_tables;
        for (&group, &place) in self.granules.range(groups) {
            found = true;
            let listed = &mut self.lists[place as usize];
            renumber(listed, (sets.len / 2) as u32);
            listed.sets.retain(|&set| {
                for slot in [2 * set as usize, 2 * set as usize + 1] {
                    let held = sets.get(slot);
                    if held.key() == 0 {
                        continue;
                    }
                    let entry = held.load();
                    if entry.key != 0 && grouping.of(&entry) == group && forget(&entry) {
                        held.store(LookupEntry::default());
                        mirror(sets, mask, set as usize, growth, old);
                        forget_jumps(stamps, entry.pc);
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
const SAMPLED: i32 = core::mem::offset_of!(ChainControl, sampled) as i32;
#[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
const LINKED: i32 = core::mem::offset_of!(ChainControl, linked) as i32;
/// [`ChainControl::quiet`].
pub const QUIET_OFFSET: i32 = core::mem::offset_of!(ChainControl, quiet) as i32;
#[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
const INTERRUPT_EPOCH: i32 = core::mem::offset_of!(ChainControl, interrupt_epoch) as i32;
#[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
const OBSERVED_INTERRUPT_EPOCH: i32 =
    core::mem::offset_of!(ChainControl, observed_interrupt_epoch) as i32;
const FRAME_ROOM: i32 = core::mem::offset_of!(ChainControl, frame_room) as i32;
const RETIRE_EPOCH: i32 = core::mem::offset_of!(ChainControl, retire_epoch) as i32;
const PAIRS: i32 = core::mem::offset_of!(ChainControl, pairs) as i32;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Interruption {
    /// The quiet word ([`ChainControl::quiet`]).
    Quiet,
    /// No gate: the budget alone, for the tests that show the gate is what
    /// stops a run.
    #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
    Legacy,
}
impl Interruption {
    /// Bytes a polled link's checks ([`link_checks`]) take past an unpolled
    /// one's.
    fn link_bytes(self, gate: LinkGate) -> usize {
        match (self, gate) {
            (Self::Quiet, LinkGate::Jrcxz) => 4,
            (Self::Quiet, LinkGate::Pext) => 1,
            #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
            (Self::Legacy, _) => 0,
        }
    }
}

/// How a polled link finds, without the flags, that no interruption is
/// pending and a transfer is left: a host tier the engine picks from its
/// target ([`Self::for_target`]). Both fall through when the run goes on,
/// and use no instruction whose cost differs much between vendors, as
/// LOOP's (7 uops on Intel's cores) did.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum LinkGate {
    /// `mov ecx, [rdx]` and JRCXZ on the quiet word, then `movzx ecx,
    /// r13w` and JRCXZ on the account: any x86-64 host.
    Jrcxz,
    /// `pext ecx, r13d, [rdx]`, the account's transfers part under the
    /// quiet word as a mask ([`QUIET`] or 0), and one JRCXZ: a host whose
    /// PEXT runs in hardware ([`crate::TargetCpu::FastPdep`]).
    Pext,
}

impl LinkGate {
    /// The tier `target` runs best.
    pub const fn for_target(target: crate::TargetCpu) -> Self {
        if target.supports(crate::TargetCpu::Baseline.with_bmi2().with_fast_pdep()) {
            Self::Pext
        } else {
            Self::Jrcxz
        }
    }
}

/// A tail's or probe's check before its transfer, RCX at the control, which
/// clobbers the flags: a transfer left and no interruption pending, in one
/// `test [rcx], r13w`, as the quiet word is the account's transfers part
/// while quiet and 0 once an interruption is pending. The jump (8-bit
/// unless `far`) a run that must stop takes, with nothing spent.
fn stop_check(
    asm: &mut crate::x86_64::asm::Asm,
    policy: Interruption,
    far: bool,
) -> crate::x86_64::asm::Patch {
    match policy {
        Interruption::Quiet => asm.test(Size::Word, Mem::base(Rcx, QUIET_OFFSET), ACCOUNT),
        #[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
        Interruption::Legacy => asm.test(Size::Word, ACCOUNT, ACCOUNT),
    }
    match far {
        true => asm.jcc(Cond::E),
        false => asm.jcc8(Cond::E),
    }
}

/// What a transfer from a block of `ran` instructions adds to the
/// [`Account`]: its instructions, less one transfer. `None` when it exceeds
/// a 32-bit displacement.
fn transfer_change(ran: u32) -> Option<i32> {
    i32::try_from((i64::from(ran) << ACCOUNT_LANE_BITS) - 1).ok()
}

/// A link's checks, RDX at the control, without the flags: a transfer
/// left and, `polled`, no interruption pending, by `gate`. The JRCXZs a run
/// that must stop takes; the run goes on through them. Clobbers RCX.
fn link_checks(
    asm: &mut crate::x86_64::asm::Asm,
    polled: bool,
    policy: Interruption,
    gate: LinkGate,
) -> Vec<crate::x86_64::asm::Patch> {
    let mut stops = Vec::new();
    match (polled, policy, gate) {
        (true, Interruption::Quiet, LinkGate::Pext) => {
            asm.pext(Rcx, ACCOUNT, Mem::base(Rdx, QUIET_OFFSET));
            stops.push(asm.jrcxz());
            return stops;
        }
        (true, Interruption::Quiet, LinkGate::Jrcxz) => {
            asm.mov_load(Dword, Rcx, Mem::base(Rdx, QUIET_OFFSET));
            stops.push(asm.jrcxz());
        }
        _ => {}
    }
    asm.movzx(Dword, Rcx, Size::Word, ACCOUNT);
    stops.push(asm.jrcxz());
    stops
}

/// `lea r13, [r13 + change]`: one transfer and its block's instructions
/// ([`transfer_change`]), once a check found a transfer left. No flags.
fn count(asm: &mut crate::x86_64::asm::Asm, change: i32) {
    asm.lea(Qword, ACCOUNT, Mem::base_disp32(ACCOUNT, change));
}

/// A probe's transfer, which its [`stop_check`] at its start allowed:
/// one transfer and the instructions of its block, which come in EDX above
/// its word. Clobbers EDX.
fn spend_transfer(asm: &mut crate::x86_64::asm::Asm) {
    asm.shift_imm(ShiftOp::Shr, Dword, Rdx, ExitWord::RAN_SHIFT as u8);
    asm.shift_imm(ShiftOp::Shl, Dword, Rdx, ACCOUNT_LANE_BITS as u8);
    asm.lea(Qword, ACCOUNT, Mem::indexed(ACCOUNT, Rdx, 1, -1));
}
// A tail's `ran` shifted into the account fits EDX.
const _: () = assert!(u32::BITS - ExitWord::RAN_SHIFT + ACCOUNT_LANE_BITS <= 32);
struct ThunkPlan {
    at: u64,
    retired: u32,
    block: u32,
    entry: u64,
    exit: u64,
    gate: LinkGate,
}

const _: () = assert!(core::mem::size_of::<LookupEntry>() == 32);
const _: () = assert!(core::mem::offset_of!(LookupEntry, pc) == 0);
const ENTRY_KEY: i32 = core::mem::offset_of!(LookupEntry, key) as i32;
const ENTRY_CODE: i32 = core::mem::offset_of!(LookupEntry, entry) as i32;
const ENTRY_BLOCK: i32 = core::mem::offset_of!(LookupEntry, block) as i32;
const ENTRY_TARGETS: i32 = core::mem::offset_of!(LookupEntry, targets) as i32;

/// A direct exit to `target` with `kind` as a block emits it, unpadded:
/// the patch point, MOVABS RAX with the target, MOV EDX with the word and
/// the return. Code that ends up at an address its patch point does not
/// suit is aligned before it runs ([`align_direct_exit`]);
/// [`is_direct_exit`] accepts every padding.
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
    direct_exit_padded(target, kind.into().encoded(), patch_pad(at))
}

/// Where a direct exit's MOVABS RAX starts, however it is padded.
const DIRECT_TARGET: usize = EXIT_POINT_BYTES;
const MOVABS_RAX: [u8; 2] = [0x48, 0xb8];

fn direct_exit_padded(target: GuestAddress, word: u32, pad: usize) -> [u8; DIRECT_EXIT_BYTES] {
    let mut bytes = [0; DIRECT_EXIT_BYTES];
    bytes[..EXIT_POINT_BYTES].copy_from_slice(&exit_point(pad));
    bytes[DIRECT_TARGET..DIRECT_TARGET + 2].copy_from_slice(&MOVABS_RAX);
    bytes[DIRECT_TARGET + 2..DIRECT_TARGET + 10].copy_from_slice(&target.0.to_le_bytes());
    bytes[DIRECT_TARGET + 10..].copy_from_slice(&word_return(word));
    bytes
}

/// A NOP of `pad` bytes, the unlinked patch point ([`RET_ROOM`]) and a NOP
/// of the rest of [`MAX_PAD`]: the start of every exit.
pub(crate) const fn exit_point(pad: usize) -> [u8; EXIT_POINT_BYTES] {
    let mut bytes = [0; EXIT_POINT_BYTES];
    let mut at = 0;
    while at < pad {
        bytes[at] = NOPS[pad][at];
        at += 1;
    }
    while at < pad + PATCH_BYTES {
        bytes[at] = RET_ROOM[at - pad];
        at += 1;
    }
    let rest = MAX_PAD - pad;
    while at < EXIT_POINT_BYTES {
        bytes[at] = NOPS[rest][at - pad - PATCH_BYTES];
        at += 1;
    }
    bytes
}

/// MOV EDX with `word` and the [`EXIT_RETURN`]: how every exit ends.
pub(crate) const fn word_return(word: u32) -> [u8; 5 + EXIT_RETURN_BYTES] {
    const MOV_EDX: u8 = 0xba;
    let mut bytes = [0; 5 + EXIT_RETURN_BYTES];
    bytes[0] = MOV_EDX;
    let word = word.to_le_bytes();
    let mut at = 0;
    while at < 4 {
        bytes[1 + at] = word[at];
        at += 1;
    }
    while at < 4 + EXIT_RETURN_BYTES {
        bytes[1 + at] = EXIT_RETURN[at - 4];
        at += 1;
    }
    bytes
}

/// The patch point padded by `pad`, MOV EDX with `word` and the return:
/// the dynamic exit ([`super::block::exit_kind_with`]).
fn exit_word_bytes(word: u32, pad: usize) -> [u8; DYNAMIC_EXIT_BYTES] {
    let mut bytes = [0; DYNAMIC_EXIT_BYTES];
    bytes[..EXIT_POINT_BYTES].copy_from_slice(&exit_point(pad));
    bytes[EXIT_POINT_BYTES..].copy_from_slice(&word_return(word));
    bytes
}

/// The unpadded direct exit to `pc` of the encoded `word`
/// ([`ExitWord::encoded`]).
#[cfg(test)]
pub(crate) fn direct_exit_word(pc: GuestAddress, word: u32) -> [u8; DIRECT_EXIT_BYTES] {
    direct_exit_padded(pc, word, 0)
}

/// The return to the engine ([`EXIT_RETURN`]).
pub(crate) fn exit_return(asm: &mut crate::x86_64::asm::Asm) {
    asm.fixed(&EXIT_RETURN);
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
/// it: what a lookup's jump replaces, the exit's first instruction but its
/// pad.
pub const fn dynamic_patch(at: u64) -> u64 {
    at + patch_pad(at) as u64
}

/// The patch point of the direct exit at host address `at`, aligned for it.
pub const fn direct_patch(at: u64) -> u64 {
    dynamic_patch(at)
}

/// Where the exit whose patch point is `point` returns to the engine
/// from while unlinked: right after the point, the target loaded (a direct
/// exit's), the word and the return. A thunk at the point that allows no
/// transfer goes on there.
pub const fn unlinked_return(point: u64) -> u64 {
    point + PATCH_BYTES as u64
}

/// The link slot of `bytes` at host address `at`'s patch point: the jump
/// over its body to the exit, the NOP into the body, or a jump to a thunk.
pub const fn slot_patch(at: u64) -> u64 {
    at + patch_pad(at) as u64
}

/// Whether the direct exit at `offset` of `code`, which runs from host
/// address `at`, has its patch point where a link patches it: its slot's
/// point jumping to it when `slot` is nonzero, else its own.
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
/// address `at`, has its unlinked patch point where a lookup patches it.
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
    let field = EXIT_POINT_BYTES + 1;
    let word = u32::from_le_bytes(bytes.get(field..field + 4)?.try_into().ok()?);
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

/// Whether `bytes` are a dynamic exit of any kind and padding.
pub fn is_dynamic_exit_of_any_kind(bytes: &[u8]) -> bool {
    exit_word_of(bytes).is_some()
}

/// The target and encoded word of the direct exit `bytes` are, of any
/// padding.
fn direct_exit_of(bytes: &[u8]) -> Option<(GuestAddress, u32)> {
    let target = bytes.get(DIRECT_TARGET + 2..DIRECT_TARGET + 10)?;
    let target = GuestAddress(u64::from_le_bytes(target.try_into().ok()?));
    let field = DIRECT_TARGET + 10 + 1;
    let word = u32::from_le_bytes(bytes.get(field..field + 4)?.try_into().ok()?);
    (0..=MAX_PAD)
        .any(|pad| bytes == direct_exit_padded(target, word, pad))
        .then_some((target, word))
}

/// Whether `bytes` are a direct exit of any target, kind and padding: what
/// ends a block whose exits are not linked.
pub fn is_direct_exit_of_any_kind(bytes: &[u8]) -> bool {
    direct_exit_of(bytes).is_some()
}

/// Pad the direct exit at `offset` of `code`, which runs at host address
/// `at + offset`, for its patch point there. Whether it is one.
pub fn align_direct_exit(code: &mut [u8], offset: usize, at: u64) -> bool {
    let Some(bytes) = code.get_mut(offset..offset + DIRECT_EXIT_BYTES) else {
        return false;
    };
    let Some((target, word)) = direct_exit_of(bytes) else {
        return false;
    };
    bytes.copy_from_slice(&direct_exit_padded(
        target,
        word,
        patch_pad(at + offset as u64),
    ));
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
    bytes.copy_from_slice(&exit_word_bytes(word, patch_pad(at + offset as u64)));
    true
}

/// Make the empty link slot of `bytes` at `offset` of `code`, which runs
/// at host address `at + offset`, the one for there.
pub fn align_link_slot(code: &mut [u8], offset: usize, bytes: usize, at: u64) -> bool {
    let Some(slot) = code.get_mut(offset..offset + bytes) else {
        return false;
    };
    if bytes > MAX_LINK_BYTES {
        return false;
    }
    let mut empty = [0; MAX_LINK_BYTES];
    let empty = &mut empty[..bytes];
    if !(0..8).any(|residue| {
        empty_link_at(residue, empty);
        *slot == *empty
    }) {
        return false;
    }
    empty_link_at(at + offset as u64, slot);
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

/// The largest link slot.
const MAX_LINK_BYTES: usize = if LINK_BYTES > POLLED_LINK_BYTES {
    LINK_BYTES
} else {
    POLLED_LINK_BYTES
};

/// Write into `slot` the empty link slot of its length at host address
/// `at`: the pad, then the patch point's jump to the exit after the slot,
/// then INT3s.
fn empty_link_at(at: u64, slot: &mut [u8]) {
    let bytes = slot.len();
    let pad = patch_pad(at);
    slot.fill(0xcc);
    slot[..pad].copy_from_slice(NOPS[pad]);
    let jump = jump(at + pad as u64, at + bytes as u64).expect("an exit right after its slot");
    slot[pad..pad + PATCH_BYTES].copy_from_slice(&jump);
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

/// The link in the slot of `bytes` at host address `at`: its body starts
/// at the slot's patch point, whose instruction becomes the first bytes of
/// the body's, the control's load. A link writes the
/// body's bytes past the point first ([`SlotLink::rest`]), which nothing
/// runs while the point jumps over them, then the point's
/// ([`SlotLink::point`]). The body chains a block of `retired` instructions
/// into block `block`, whose code is at `entry`, or goes to the exit after
/// the slot when the control allows no transfer. `None` when `entry` is out
/// of a 32-bit jump's reach, an operand exceeds its 32-bit field or `bytes`
/// is no slot.
pub fn link(
    at: u64,
    bytes: usize,
    retired: u32,
    block: u32,
    entry: u64,
    gate: LinkGate,
) -> Option<SlotLink> {
    let polled = match bytes {
        LINK_BYTES => false,
        POLLED_LINK_BYTES => true,
        _ => return None,
    };
    let point = slot_patch(at);
    let exit = at + bytes as u64;
    let code = link_body(
        point,
        polled,
        retired,
        block,
        entry,
        Fall::After(exit),
        Interruption::Quiet,
        gate,
    )?;
    debug_assert_eq!(point + code.len() as u64, exit);
    let (first, rest) = code.split_at(PATCH_BYTES);
    debug_assert_eq!(first, LINKED_SLOT);
    Some(SlotLink {
        point: first.try_into().ok()?,
        rest_at: point + PATCH_BYTES as u64,
        rest: rest.to_vec(),
    })
}

/// A slot's [`link`]: the bytes past its patch point, written first, and
/// the point's instruction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SlotLink {
    /// What the patch point becomes: the first bytes of the body's first
    /// instruction, whose others start [`Self::rest`].
    pub point: [u8; PATCH_BYTES],
    /// The host address right after the point, where `rest` goes.
    pub rest_at: u64,
    pub rest: Vec<u8>,
}

impl SlotLink {
    /// The body whole, from the patch point.
    pub fn body(&self) -> Vec<u8> {
        let mut body = self.point.to_vec();
        body.extend(&self.rest);
        body
    }
}

/// A thunk at host address `at` that chains a block of `retired`
/// instructions into block `block`, whose code is at `entry`, or jumps to
/// `exit` when the control allows no transfer: what an exit's patch point
/// jumps to when its slot's body is used or it has none. `exit` is the
/// exit's own unlinked path ([`unlinked_return`]), or for a slot's point
/// the exit after the slot. It always looks for an interruption. `None`
/// when `entry` or `exit` is out of a 32-bit jump's reach or an operand
/// exceeds its 32-bit field.
pub fn thunk(
    at: u64,
    retired: u32,
    block: u32,
    entry: u64,
    exit: u64,
    gate: LinkGate,
) -> Option<Vec<u8>> {
    thunk_with_policy(
        ThunkPlan {
            at,
            retired,
            block,
            entry,
            exit,
            gate,
        },
        Interruption::Quiet,
    )
}
fn thunk_with_policy(plan: ThunkPlan, policy: Interruption) -> Option<Vec<u8>> {
    let ThunkPlan {
        at,
        retired,
        block,
        entry,
        exit,
        gate,
    } = plan;
    link_body(
        at,
        true,
        retired,
        block,
        entry,
        Fall::Thunk(exit),
        policy,
        gate,
    )
}

/// Where a link's body goes when the control allows no transfer.
#[derive(Debug, Clone, Copy)]
enum Fall {
    /// A thunk's: a jump to the exit at this host address, after its body.
    Thunk(u64),
    /// A slot's: the exit at this host address, a few bytes after the body.
    After(u64),
}

/// The code of a link at host address `at`, which leaves the flags: the
/// control in RDX, check ([`link_checks`]), count, name and jump, or `fall`
/// when the run must stop, nothing spent. Only a `polled` one looks for an
/// interruption.
#[allow(clippy::too_many_arguments)]
fn link_body(
    at: u64,
    polled: bool,
    retired: u32,
    block: u32,
    entry: u64,
    fall: Fall,
    policy: Interruption,
    gate: LinkGate,
) -> Option<Vec<u8>> {
    let block = i32::try_from(block).ok()?;
    let change = transfer_change(retired)?;
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.place(Rdx, CONTROL);
    let stops = link_checks(&mut asm, polled, policy, gate);
    count(&mut asm, change);
    asm.mov_rm_imm32(Qword, Mem::base(Rdx, BLOCK), block);
    let transfer = asm.jmp();
    let distance = relative(at + transfer.end() as u64, entry)?;
    debug_assert_eq!(
        code.len(),
        LINK_BODY_BYTES + usize::from(polled) * policy.link_bytes(gate)
    );
    let stop = match fall {
        Fall::Thunk(exit) => {
            let stop = code.len();
            let back = code.asm().jmp();
            let back_distance = relative(at + back.end() as u64, exit)?;
            code.patch(back.offset(), back_distance.to_le_bytes());
            stop
        }
        // The exit after the slot; INT3s up to it, which nothing runs.
        Fall::After(exit) => {
            let stop = usize::try_from(exit.checked_sub(at)?).ok()?;
            let mut asm = code.asm();
            for _ in asm.len()..stop {
                asm.int3();
            }
            stop
        }
    };
    for patch in stops {
        crate::x86_64::asm::resolve(&mut code, patch, stop);
    }
    code.patch(transfer.offset(), distance.to_le_bytes());
    Some(code.into_bytes())
}

/// The start of a call whose guest return address is `return_address`,
/// in code that RCX, R8 and flags are free in (RAX may hold the call's
/// target, RDX its word), before its exit: unless the frames already below
/// the entry's stack fill the control's room, a paired frame
/// ([`FRAME_BYTES`]) and a host `call` of the exit that follows, whose
/// return comes back to a jump to the call's continuation. The frame holds
/// `return_address`, the calling block's word, the board's retire epoch
/// and both halves' contexts, under the call's own return address, so a
/// guest return to `return_address` in this native run, in the same
/// contexts, can `ret` into the continuation ([`paired_return`]). The
/// continuation is the caller's own exit to `return_address`, which runs
/// none of the block's instructions: the call's exit counted them. A call
/// past the room counts its refusal and takes the exit as a plain one;
/// code run without a control pushes none.
///
/// Between blocks the host stack holds paired frames and nothing else, from
/// the state's saved entry stack pointer down ([`EXIT_RETURN`]): every
/// block, thunk and tail leaves RSP as it found it but for these. A run
/// that leaves native code at any exit leaves every frame with it, as the
/// exit reloads RSP: an exit the called code takes before its return, or
/// the call's own exit while unlinked, discards the frames whole. The jump
/// to the continuation is the patch this returns, which the caller binds
/// after the call's exit.
pub(crate) fn paired_call(
    asm: &mut crate::x86_64::asm::Asm,
    return_address: GuestAddress,
) -> crate::x86_64::asm::Patch {
    asm.place(Rcx, CONTROL);
    asm.test(Qword, Rcx, Rcx);
    let uncontrolled = asm.jcc(Cond::E);
    // R8 = the bytes of frames below the entry's stack now.
    asm.mov_load(Qword, R8, Mem::base(Gpr::Rdi, EXIT_RSP_OFFSET));
    asm.alu(Alu::Sub, Qword, R8, Gpr::Rsp);
    asm.alu_load(Alu::Cmp, Dword, R8, Mem::base(Rcx, FRAME_ROOM));
    let refused = asm.jcc(Cond::Ae);
    asm.push_rm(Mem::base(Rcx, CONTEXTS + 8));
    asm.push_rm(Mem::base(Rcx, CONTEXTS));
    asm.mov_load(Qword, R8, Mem::base(Rcx, RETIRE_EPOCH));
    asm.push_rm(Mem::base(R8, 0));
    asm.push_rm(Mem::base(Rcx, BLOCK));
    asm.guest_imm64(R8, return_address);
    asm.push(R8);
    let call = asm.call();
    let continuation = asm.jmp();
    asm.bind(refused);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(Rcx, PairCount::Refused.offset()),
        1,
    );
    asm.bind(call);
    asm.bind(uncontrolled);
    continuation
}

/// The start of a return to the target in RAX, from a block that ran `ran`
/// instructions, in code that RCX, R8 and flags are free in, before its
/// lookup: when the host stack holds a frame ([`paired_call`]), which it
/// does below the entry's stack pointer, and that frame holds the target,
/// the board's retire epoch has not moved since its push, both halves'
/// contexts are the frame's, no interruption is pending and budget is
/// left, the return counts `ran`, names the calling block again and
/// `ret`s into the frame's continuation, taking the frame. Anything else
/// goes on to the lookup, its frames left in place.
///
/// A frame's continuation is an exit to its address from a block that ran
/// in the frame's contexts, so any return to that address in those
/// contexts in the same native run may take it, whichever call pushed it:
/// in one run the PE's ITLB changes only by contexts (a native SVC, ERET
/// or exception entry loads another state's), and a TLBI, a TTBR or
/// translation control write and every fill leave native code, which
/// discards the frames. A block retired meanwhile, by any PE, advances
/// the epoch, which every frame of the run then fails, so a return never
/// enters a retired block's continuation through its frame.
pub(crate) fn paired_return(asm: &mut crate::x86_64::asm::Asm, ran: u32) -> Option<()> {
    let ran = i32::try_from(i64::from(ran) << ACCOUNT_LANE_BITS).ok()?;
    let frame = |word: FrameWord| Mem::base(Gpr::Rsp, word as i32);
    asm.alu_load(
        Alu::Cmp,
        Qword,
        Gpr::Rsp,
        Mem::base(Gpr::Rdi, EXIT_RSP_OFFSET),
    );
    let empty = asm.jcc8(Cond::E);
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    asm.alu_load(Alu::Cmp, Qword, Rax, frame(FrameWord::Address));
    misses.push(asm.jcc8(Cond::Ne));
    asm.mov_load(Qword, R8, Mem::base(Rcx, RETIRE_EPOCH));
    asm.mov_load(Qword, R8, Mem::base(R8, 0));
    asm.alu_load(Alu::Cmp, Qword, R8, frame(FrameWord::Epoch));
    misses.push(asm.jcc8(Cond::Ne));
    for (half, word) in [(0, FrameWord::LowerContext), (1, FrameWord::UpperContext)] {
        asm.mov_load(Qword, R8, Mem::base(Rcx, CONTEXTS + 8 * half));
        asm.alu_load(Alu::Cmp, Qword, R8, frame(word));
        misses.push(asm.jcc8(Cond::Ne));
    }
    // A transfer left and no interruption pending. A return spends none,
    // and counts the block's instructions.
    misses.push(stop_check(asm, Interruption::Quiet, false));
    asm.alu_imm32(Alu::Add, Qword, ACCOUNT, ran);
    // The block's index, entered with BTYPE 0 as a continuation is.
    asm.mov_load(Dword, R8, frame(FrameWord::Block));
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(Rcx, PairCount::Matched.offset()),
        1,
    );
    // ret 40: into the continuation, the rest of the frame taken.
    asm.fixed(&[0xc2, (FRAME_BYTES - 8) as u8, 0]);
    for miss in misses {
        asm.bind(miss);
    }
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(Rcx, PairCount::Missed.offset()),
        1,
    );
    asm.bind(empty);
    Some(())
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
pub const STAMPING_PROBE_ENTRY: usize = PROBE_ENTRY + 640;
/// Bytes in a tail's call of a probe: `mov edx, word`, `call probe` and
/// `jmp r9` into the block the probe found.
pub const TAIL_BYTES: usize = 5 + 5 + 3;
/// Bytes in a [`cell_tail`]: the target's and the cell's MOVABS and the
/// call of the cell probe.
pub const CELL_TAIL_BYTES: usize = 10 + 10 + TAIL_BYTES;

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
/// R10 and R11, free between blocks, hold what a probe keeps.
/// Every tail calls its probe, which returns to it with the block's code in
/// R9 for the tail's own `jmp r9`: the host predicts that jump per site,
/// where one shared indirect jump in the probes mispredicted across many
/// hot sites. The return pops the call's return stack entry, so the stack
/// stays as the tail found it; a cell probe that goes on to the table probe
/// jumps there with the return address kept, and an exit to the engine
/// reloads RSP ([`EXIT_RETURN`]).
/// Other cores write the table and cells meanwhile: a probe takes an entry
/// only when its last word, read before its fields, was not busy and is
/// unchanged after them ([`Slot`]); a cell read torn starts the table
/// probe over, a table entry read torn leaves the target to the engine. A tail is a few bytes
/// of each exit's own ([`lookup_tail`], [`cell_tail`]); the probe's work,
/// and its indirect jump into the found block, are shared: the host's
/// indirect predictor tells sites apart by the path into it.
pub fn lookup_dispatcher(btype: u8) -> Option<Vec<u8>> {
    dispatcher_with_policy(btype, Interruption::Quiet)
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
    // A run that must stop leaves before the tables, with EDX whole.
    let mut misses = vec![stop_check(&mut asm, policy, true)];
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
    spend_transfer(&mut asm);
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
    Some(code.into_bytes())
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
/// block ([`stamp`]). Every one counts its transfers in the control's
/// `heat`, a word only its PE writes, and names the cell of one in
/// [`CELL_SAMPLE`] in `sampled`, by a branch taken that rarely. The jumps to
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
    let mut misses = vec![stop_check(&mut asm, policy, true)];
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
    const _: () = assert!(CELL_SAMPLE == 256);
    asm.alu_imm8(Alu::Add, Dword, Mem::base(Rcx, HEAT), 1);
    asm.test_imm(Size::Byte, Mem::base(Rcx, HEAT), 0xff);
    let unsampled = asm.jcc8(Cond::Ne);
    asm.mov_store(Qword, Mem::base(Rcx, SAMPLED), R10);
    asm.bind(unsampled);
    spend_transfer(&mut asm);
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

/// A cell probe's stamp, after its transfer is counted, of the cell at R10
/// when it is a stamped one: this PE's context of the target's half and its
/// ITLB's drop epoch there, while the cell still holds the entry it was
/// linked with ([`stamped_tail`]). The epoch goes in by a locked exchange,
/// and the `live` word is read again after it: a writer that cleared it
/// meanwhile also zeroes the stamp, or this does. It counts the stamps it
/// writes and the cells it finds changed ([`StampCount`]). Clobbers RDX,
/// R8, R10, R11.
fn stamp(asm: &mut crate::x86_64::asm::Asm) {
    const _: () = assert!(size_of::<LookupEntry>() == 2 * STAMP_BYTES);
    // R11 = 32 times the cell's number, or more for a cell past the
    // stamped ones, whose chunks lie elsewhere.
    asm.mov(Qword, R11, R10);
    asm.alu_load(Alu::Sub, Qword, R11, Mem::base(Rcx, CELLS));
    asm.alu_imm32(Alu::Cmp, Qword, R11, CELL_LIVE as i32);
    let none = asm.jcc8(Cond::Ae);
    asm.mov_load(Qword, R8, Mem::base(Rcx, STAMPS));
    asm.test(Qword, R8, R8);
    let unstamped = asm.jcc8(Cond::E);
    // RDX = the cell's `live` word less CELL_LIVE, 4 bytes a cell.
    asm.mov(Qword, Rdx, R11);
    asm.shift_imm(ShiftOp::Shr, Qword, Rdx, 3);
    asm.alu_load(Alu::Add, Qword, Rdx, Mem::base(Rcx, CELLS));
    asm.alu_imm8(Alu::Cmp, Dword, Mem::base(Rdx, CELL_LIVE as i32), 0);
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
    // R11 = this PE's stamp of the cell.
    asm.shift_imm(ShiftOp::Shr, Qword, R11, 1);
    asm.alu(Alu::Add, Qword, R11, R8);
    asm.mov(Qword, R8, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R8, 55);
    asm.alu_imm8(Alu::And, Dword, R8, 1);
    asm.mov_load(Qword, R10, Mem::indexed(Rcx, R8, 8, CONTEXTS));
    asm.mov_store(Qword, Mem::base(R11, 0), R10);
    asm.mov_load(Qword, R10, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R10, Mem::indexed(R10, R8, 8, ITLB_DROPS));
    // xchg [r11 + 8], r10
    asm.fixed(&[0x4d, 0x87, 0x53, 0x08]);
    asm.alu_imm8(Alu::Cmp, Dword, Mem::base(Rdx, CELL_LIVE as i32), 0);
    let kept = asm.jcc8(Cond::Ne);
    asm.mov_rm_imm32(Qword, Mem::base(R11, 8), 0);
    asm.bind(none);
    asm.bind(changed);
    asm.bind(unstamped);
    asm.bind(kept);
}

/// Bytes in a [`stamped_tail`]: 132 at most, with a 32-bit displacement
/// for the stamp of a cell numbered 8 or above.
pub const STAMPED_TAIL_BYTES: usize = 176;

/// The tail at host address `at` of a hot direct exit to `target` out of
/// its page, whose stamped cell `number` at `cell` held `block`, entered at
/// `entry`, when it was linked: while this PE's stamp of the cell names the
/// context it runs in and its ITLB's drop epoch of the target's half, the
/// transfer is counted, `block` named and `entry` entered by a direct jump,
/// without a translation or a probe; else, as for an interruption or no
/// transfer left, it is the [`cell_tail`] after it, whose cell probe stamps
/// the cell when it enters its block, or stops the run.
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
        Interruption::Quiet,
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
        Interruption::Quiet,
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
    let stopped = stop_check(&mut asm, policy, false);
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R10, StampCount::Hits.offset()),
        1,
    );
    count(&mut asm, transfer_change(ran)?);
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
    // A miss counts its cause and takes the cell tail, whose probe stops a
    // run that must stop as it would.
    let mut to_tail = Vec::new();
    for (miss, count) in [
        (epoch, StampCount::Epoch),
        (context, StampCount::Context),
        (stopped, StampCount::Stopped),
    ] {
        asm.bind(miss);
        asm.alu_imm8(Alu::Add, Qword, Mem::base(R10, count.offset()), 1);
        to_tail.push(asm.jmp8());
    }
    let tail_at = code.len();
    for patch in to_tail {
        crate::x86_64::asm::resolve(&mut code, patch, tail_at);
    }
    let cell_code = cell_tail(at + tail_at as u64, cell, target, word, ran, probe)?;
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
/// is then the block key of its page in the current context of its half,
/// or for a lower-half page in the upper half's, where the engine keeps
/// the lower-half pages whose walk ended in a global leaf (a global entry
/// matches every ASID, and the upper half's context is keyed by the same
/// translation inputs, EL and variant but TTBR0); a page with no entry in
/// either takes one of `misses`. Clobbers R9 and R11.
fn translate(asm: &mut crate::x86_64::asm::Asm, misses: &mut Vec<crate::x86_64::asm::Patch>) {
    // R8 = the target's half; R9 = the first slot of its page's ITLB set
    // in that half's context.
    asm.mov(Qword, R8, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R8, 55);
    asm.alu_imm8(Alu::And, Dword, R8, 1);
    itlb_set(asm, Mem::indexed(Rcx, R8, 4, SALTS));
    asm.mov_load(Qword, R8, Mem::indexed(Rcx, R8, 8, CONTEXTS));
    // R11 = the target's page; either entry of the set may hold it.
    asm.mov(Qword, R11, Rax);
    asm.alu_imm32(Alu::And, Qword, R11, -(1 << PAGE_SHIFT));
    let mut absent = Vec::new();
    let first = itlb_compare(asm, &mut absent, false);
    let second = asm.jmp();
    // A lower-half page absent from its half's context: its global entry,
    // in the upper half's.
    for miss in absent {
        asm.bind(miss);
    }
    asm.bit_imm(BitTest::Bt, Qword, Rax, 55);
    misses.push(asm.jcc(Cond::B));
    asm.mov_load(Qword, R8, Mem::base(Rcx, CONTEXTS + 8));
    // Context 0 is none: an empty entry's key.
    asm.test(Qword, R8, R8);
    misses.push(asm.jcc(Cond::E));
    itlb_set(asm, Mem::base(Rcx, SALTS + 4));
    let shared = itlb_compare(asm, misses, true);
    asm.bind(first);
    asm.bind(second);
    asm.bind(shared);
    asm.mov_load(Qword, R8, Mem::base(R9, ENTRY_CODE));
}

/// R9 = the first slot of the ITLB set of the target's page in RAX for a
/// context whose salt is at `salt`, RCX at the control.
fn itlb_set(asm: &mut crate::x86_64::asm::Asm, salt: Mem) {
    asm.mov(Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, ITLB_SET_BITS as u8);
    asm.alu(Alu::Xor, Dword, R9, Rax);
    asm.shift_imm(ShiftOp::Shr, Dword, R9, PAGE_SHIFT as u8);
    asm.alu_load(Alu::Xor, Dword, R9, salt);
    asm.alu_load(Alu::And, Dword, R9, Mem::base(Rcx, MASKS + 4));
    asm.shift_imm(
        ShiftOp::Shl,
        Dword,
        R9,
        core::mem::size_of::<[LookupEntry; 2]>().trailing_zeros() as u8,
    );
    asm.alu_load(Alu::Add, Qword, R9, Mem::base(Rcx, ITLB));
}

/// Find the entry of the page in R11 and the context in R8 in the ITLB set
/// at R9, leaving R9 at it: a hit in the second entry falls through, one in
/// the first takes the jump returned (8-bit when `near`), and a miss takes
/// one of the jumps pushed to `misses`.
fn itlb_compare(
    asm: &mut crate::x86_64::asm::Asm,
    misses: &mut Vec<crate::x86_64::asm::Patch>,
    near: bool,
) -> crate::x86_64::asm::Patch {
    asm.alu_load(Alu::Cmp, Qword, R11, Mem::base(R9, 0));
    let other_page = asm.jcc8(Cond::Ne);
    asm.alu_load(Alu::Cmp, Qword, R8, Mem::base(R9, ENTRY_KEY));
    let first = if near {
        asm.jcc8(Cond::E)
    } else {
        asm.jcc(Cond::E)
    };
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
    first
}

/// A probe's transfer, counted: back to the tail that called it, whose
/// `jmp r9` enters the block at R9.
fn found(asm: &mut crate::x86_64::asm::Asm) {
    asm.ret();
}

/// A probe's exit to the engine, with RAX and EDX as the tail left them: the
/// return of the native run, as the tail jumped to the probe.
fn missed(asm: &mut crate::x86_64::asm::Asm) {
    exit_return(asm);
}

/// Bytes in a [`lookup_tail`]: the jump cache's hit and the plain tail
/// ([`TAIL_BYTES`]) of a miss.
pub const LOOKUP_TAIL_BYTES: usize = 224;

/// The tail at host address `at` that an exit's patch point jumps to once
/// the exit is redirected through a lookup, for a branch that sets
/// `btype`: a target this PE's jump cache holds (and that accepts `btype`)
/// is entered from here, counted as a probe
/// counts its transfers, by a jump of the site's own; any other takes EDX
/// the exit's `word` with the `ran` instructions of the block it leaves,
/// which only a probe reads, then a jump to the table probe at `probe`,
/// which enters the block it finds, filling the cache, or returns to the
/// engine. An interruption or no transfer left takes the plain tail too,
/// whose probe stops the run.
/// `None` when `ran` exceeds its field, `btype` is no BTYPE or `probe` is
/// out of a 32-bit jump's reach.
///
/// An entry holds while the target's half has the context and this PE's
/// ITLB the drop epoch of that half that it names, as a stamp holds
/// ([`stamped_tail`]): the PE's translation of the target's page is the one
/// the probe that filled it found. The block table's change of the entry
/// it was filled from takes it from the cache ([`JUMP_CACHE_ENTRIES`]); a
/// block another PE releases meanwhile keeps its code until this PE's
/// grace, so an entry read just before enters the block that probe would
/// have entered, as a direct link would. The control's stamps hold the
/// cache: every PE's control has them before it runs
/// (`LookupTable::new_stamps`).
pub fn lookup_tail(
    at: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
    btype: u8,
) -> Option<[u8; LOOKUP_TAIL_BYTES]> {
    lookup_tail_with_policy(at, word.into(), ran, probe, btype, Interruption::Quiet)
}

fn lookup_tail_with_policy(
    at: u64,
    word: ExitWord,
    ran: u32,
    probe: u64,
    btype: u8,
    policy: Interruption,
) -> Option<[u8; LOOKUP_TAIL_BYTES]> {
    let mut code = Code::new();
    lookup_tail_code(&mut code, at, word, ran, probe, btype, policy)?;
    let padding = LOOKUP_TAIL_BYTES.checked_sub(code.len())?;
    for _ in 0..padding {
        code.asm().int3();
    }
    code.try_into().ok()
}

/// Bytes in a [`paired_tail`], padded.
pub const PAIRED_TAIL_BYTES: usize = 320;
/// Bytes of the largest tail the engine installs: a lookup, stamped or
/// paired one.
pub const TAIL_ROOM: usize = {
    let room = if STAMPED_TAIL_BYTES > PAIRED_TAIL_BYTES {
        STAMPED_TAIL_BYTES
    } else {
        PAIRED_TAIL_BYTES
    };
    if LOOKUP_TAIL_BYTES > room {
        LOOKUP_TAIL_BYTES
    } else {
        room
    }
};

/// The tail at host address `at` of a return's exit: a [`lookup_tail`]
/// whose return first takes the frame of its call when that frame holds
/// the target in the contexts it was pushed in ([`paired_return`]), where
/// the block of `ran` instructions it leaves is counted as the lookup
/// would. Every RET form's exit is one, RETA*'s after its thunk
/// authenticated the target, so a forged or stale-key address, which the
/// thunk leaves poisoned, never equals a frame's. `None` as for a
/// [`lookup_tail`].
pub fn paired_tail(
    at: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
) -> Option<[u8; PAIRED_TAIL_BYTES]> {
    let mut code = Code::new();
    paired_return(&mut code.asm(), ran)?;
    lookup_tail_code(
        &mut code,
        at,
        word.into(),
        ran,
        probe,
        0,
        Interruption::Quiet,
    )?;
    let padding = PAIRED_TAIL_BYTES.checked_sub(code.len())?;
    for _ in 0..padding {
        code.asm().int3();
    }
    code.try_into().ok()
}

/// [`lookup_tail`]'s code, appended to `code`, which runs from `at`.
fn lookup_tail_code(
    code: &mut Code,
    at: u64,
    word: ExitWord,
    ran: u32,
    probe: u64,
    btype: u8,
    policy: Interruption,
) -> Option<()> {
    if btype > 3 || ran >= 1 << (u32::BITS - ExitWord::RAN_SHIFT) {
        return None;
    }
    let entry = |field: i32| Mem::indexed(R10, R9, 1, JUMP_CACHE + field);
    let mut asm = code.asm();
    asm.place(Rcx, CONTROL);
    let mut misses = Vec::new();
    // R10 = this PE's stamps, R9 = the entry's offset.
    asm.mov_load(Qword, R10, Mem::base(Rcx, STAMPS));
    asm.mov(Dword, R9, Rax);
    asm.alu_imm32(Alu::And, Dword, R9, JUMP_INDEX);
    asm.shift_imm(ShiftOp::Shl, Dword, R9, JUMP_INDEX_SHIFT);
    asm.alu_load(Alu::Cmp, Qword, Rax, entry(JUMP_PC));
    misses.push(asm.jcc(Cond::Ne));
    // The target's half still has the context and ITLB drop epoch the
    // entry holds in. No context is 0, so an empty entry holds in none.
    asm.mov(Qword, R8, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R8, 55);
    asm.alu_imm8(Alu::And, Dword, R8, 1);
    asm.mov_load(Qword, R11, Mem::indexed(Rcx, R8, 8, CONTEXTS));
    asm.alu_load(Alu::Cmp, Qword, R11, entry(JUMP_CONTEXT));
    misses.push(asm.jcc(Cond::Ne));
    asm.mov_load(Qword, R11, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R11, Mem::indexed(R11, R8, 8, ITLB_DROPS));
    asm.alu_load(Alu::Cmp, Qword, R11, entry(JUMP_EPOCH));
    misses.push(asm.jcc(Cond::Ne));
    if btype != 0 {
        asm.test_imm(Size::Byte, entry(JUMP_TARGETS), 1 << btype);
        misses.push(asm.jcc(Cond::E));
    }
    // A run that must stop takes the plain tail, whose probe stops it.
    misses.push(stop_check(&mut asm, policy, true));
    count(&mut asm, transfer_change(ran)?);
    asm.alu_imm8(Alu::Add, Qword, Mem::base(Rcx, HITS), 1);
    asm.mov_load(Dword, R8, entry(JUMP_BLOCK));
    entered_btype(&mut asm, R8, btype);
    asm.mov_store(Qword, Mem::base(Rcx, BLOCK), R8);
    asm.jmp_rm(entry(JUMP_CODE));
    for miss in misses {
        asm.bind(miss);
    }
    // The plain tail's jump is relative to its own end in `code`.
    tail(code, at, word, ran, probe)
}

/// Count a table probe transfer ([`StampCount::Filled`]) and fill this
/// PE's jump cache entry of the target in RAX ([`lookup_tail`]) with the
/// block the table probe just entered from the entry at R10: its code in
/// R9, its index in R8D and the entry's last word in R11D, as the probe
/// read them, whose low byte is the BTYPEs it accepts (`META_TARGETS`),
/// with the context and this PE's ITLB drop epoch of the target's half;
/// nothing for a control without stamps. An entry of another PC loses its
/// PC first; the PC goes in last, by a locked exchange, and the table
/// entry's last word is read again after it: a writer that changed the
/// entry meanwhile either finds the PC and takes it out ([`forget_jumps`]),
/// or this does. An entry that holds the PC already keeps it, without the
/// exchange: a writer finds it there, and one that took it out leaves the
/// exchange to this. Clobbers RDX and R11.
fn fill_jump_cache(asm: &mut crate::x86_64::asm::Asm) {
    const _: () = assert!(META_TARGETS == 0xff);
    let entry = |field: i32| Mem::base(Rdx, JUMP_CACHE + field);
    asm.alu_imm8(Alu::Cmp, Qword, Mem::base(Rcx, STAMPS), 0);
    let none = asm.jcc(Cond::E);
    // RDX = the entry's address, its PC out first unless it is the target.
    asm.mov(Dword, Rdx, Rax);
    asm.alu_imm32(Alu::And, Dword, Rdx, JUMP_INDEX);
    asm.shift_imm(ShiftOp::Shl, Dword, Rdx, JUMP_INDEX_SHIFT);
    asm.alu_load(Alu::Add, Qword, Rdx, Mem::base(Rcx, STAMPS));
    asm.alu_load(Alu::Cmp, Qword, Rax, entry(JUMP_PC));
    let held = asm.jcc8(Cond::E);
    asm.mov_rm_imm32(Qword, entry(JUMP_PC), 0);
    asm.bind(held);
    asm.mov_store(Qword, entry(JUMP_CODE), R9);
    asm.mov_store(Dword, entry(JUMP_BLOCK), R8);
    asm.mov_store(Dword, entry(JUMP_TARGETS), R11);
    asm.mov_load(Qword, R11, Mem::base(Rcx, STAMPS));
    asm.alu_imm8(
        Alu::Add,
        Qword,
        Mem::base(R11, StampCount::Filled.offset()),
        1,
    );
    asm.mov(Qword, R11, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R11, 55);
    asm.alu_imm8(Alu::And, Dword, R11, 1);
    asm.mov_load(Qword, R11, Mem::indexed(Rcx, R11, 8, CONTEXTS));
    asm.mov_store(Qword, entry(JUMP_CONTEXT), R11);
    asm.mov(Qword, R11, Rax);
    asm.shift_imm(ShiftOp::Shr, Qword, R11, 55);
    asm.alu_imm8(Alu::And, Dword, R11, 1);
    asm.shift_imm(ShiftOp::Shl, Dword, R11, 3);
    asm.alu_load(Alu::Add, Qword, R11, Mem::base(Rcx, ITLB));
    asm.mov_load(Qword, R11, Mem::base(R11, ITLB_DROPS));
    asm.mov_store(Qword, entry(JUMP_EPOCH), R11);
    asm.alu_imm8(Alu::Cmp, Qword, entry(JUMP_PC), 0);
    let kept_pc = asm.jcc8(Cond::Ne);
    // xchg [rdx + JUMP_CACHE], r11 with R11 the PC: a locked store.
    asm.mov(Qword, R11, Rax);
    asm.fixed(&[0x4c, 0x87, 0x9a]);
    asm.fixed(&(JUMP_CACHE + JUMP_PC).to_le_bytes());
    asm.mov_load(Dword, R11, entry(JUMP_TARGETS));
    asm.alu_load(Alu::Cmp, Dword, R11, Mem::base(R10, ENTRY_TARGETS));
    let kept = asm.jcc8(Cond::E);
    asm.mov_rm_imm32(Qword, entry(JUMP_PC), 0);
    asm.bind(kept);
    asm.bind(kept_pc);
    asm.bind(none);
}

/// The tail at host address `at` of a direct exit to `target` out of its
/// page, whose target the cell at `cell` holds ([`LookupTable::link_cell`]):
/// the target in RAX, which the exit's patch point did not load, the word
/// in EDX as in a [`lookup_tail`], then a call of the cell probe at `probe`
/// ([`CELL_PROBE_ENTRY`]), which returns only when it found the block, and
/// the tail's own jump into it. `None` as for a [`lookup_tail`].
pub fn cell_tail(
    at: u64,
    cell: u64,
    target: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
) -> Option<[u8; CELL_TAIL_BYTES]> {
    let mut code = Code::new();
    code.asm().mov_imm64(Rax, target);
    code.asm().mov_imm64(R9, cell);
    tail(&mut code, at, word.into(), ran, probe)?;
    code.try_into().ok()
}

/// A [`cell_tail`] that takes the target in RAX as it is: for tests that
/// send other targets through a cell.
#[cfg(all(test, target_arch = "x86_64", target_os = "linux"))]
fn rax_cell_tail(
    at: u64,
    cell: u64,
    word: impl Into<ExitWord>,
    ran: u32,
    probe: u64,
) -> Option<Vec<u8>> {
    let mut code = Code::new();
    code.asm().mov_imm64(R9, cell);
    tail(&mut code, at, word.into(), ran, probe)?;
    Some(code.into_bytes())
}

/// Where a [`predicted_tail`]'s direct exit to its predicted target begins:
/// its patch point is [`direct_patch`] of the tail's address plus this.
pub const PREDICTED_EXIT: usize = 15;
/// Where a [`predicted_tail`]'s miss begins, after its direct exit: an
/// [`open_predicted_tail`]'s is a dynamic exit, whose patch point is
/// [`dynamic_patch`] of the tail's address plus this.
pub const PREDICTED_MISS: usize = PREDICTED_EXIT + DIRECT_EXIT_BYTES;
/// Bytes in a [`predicted_tail`] or an [`open_predicted_tail`]: the
/// compare, the direct exit and the miss, a lookup's call of the table
/// probe or a dynamic exit, padded.
pub const PREDICTED_TAIL_BYTES: usize = PREDICTED_MISS + DYNAMIC_EXIT_BYTES;
const _: () = assert!(TAIL_BYTES <= DYNAMIC_EXIT_BYTES);

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
    while code.len() < PREDICTED_TAIL_BYTES {
        code.asm().int3();
    }
    code.try_into().ok()
}

/// A [`predicted_tail`] whose miss is the exit's dynamic exit of `word`
/// ([`PREDICTED_MISS`]), returning to the engine with RAX and EDX as the
/// redirected exit did: the engine learns a site's second target from it
/// and sends its patch point to a [`predicted_tail`] of that target, or to
/// a lookup. `None` when `target`'s direct exit cannot be made.
pub fn open_predicted_tail(
    at: u64,
    target: GuestAddress,
    word: impl Into<ExitWord>,
) -> Option<[u8; PREDICTED_TAIL_BYTES]> {
    let word = word.into();
    let mut code = Code::new();
    let mut asm = code.asm();
    asm.mov_imm64(R8, target.0);
    asm.alu(Alu::Cmp, Qword, Rax, R8);
    let other = asm.jcc8(Cond::Ne);
    debug_assert_eq!(asm.len(), PREDICTED_EXIT);
    asm.fixed(&direct_exit_at(at + PREDICTED_EXIT as u64, target, word));
    asm.bind(other);
    debug_assert_eq!(asm.len(), PREDICTED_MISS);
    let miss = at + PREDICTED_MISS as u64;
    asm.fixed(&exit_word_bytes(word.encoded(), patch_pad(miss)));
    code.try_into().ok()
}

/// A tail's call of the probe at `probe`, in `code`, which runs from `at`:
/// `mov edx` with `word` and `ran`, `call probe` and `jmp r9`, the site's
/// own jump into the block the probe found ([`lookup_dispatcher`]).
fn tail(code: &mut Code, at: u64, word: ExitWord, ran: u32, probe: u64) -> Option<()> {
    if ran >= 1 << (u32::BITS - ExitWord::RAN_SHIFT) {
        return None;
    }
    let mut asm = code.asm();
    asm.mov_imm32(Rdx, word.encoded() | ran << ExitWord::RAN_SHIFT);
    let call = asm.call();
    let relative = relative(at + call.end() as u64, probe)?;
    code.patch(call.offset(), relative.to_le_bytes());
    code.asm().jmp_rm(R9);
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
    lookup_with_policy(ran, target, kind, btype, None, Interruption::Quiet)
}
/// [`lookup_thunk`] through the cell at `cell` for `target`.
#[cfg(test)]
fn lookup_thunk_with_cell(
    ran: u32,
    target: GuestAddress,
    kind: impl Into<ExitWord>,
    cell: u64,
) -> Option<Vec<u8>> {
    lookup_with_policy(ran, Some(target), kind, 0, Some(cell), Interruption::Quiet)
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
    // The probe is called, as a tail calls it.
    let call = asm.call();
    asm.jmp_rm(R9);
    let entry = match (cell, target) {
        (Some(_), Some(_)) => CELL_PROBE_ENTRY,
        _ => PROBE_ENTRY,
    };
    let probes = code.len();
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
    exit_word_bytes(ExitWord::from(kind).encoded(), patch_pad(at))
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
    let relative = relative(at + JUMP_BYTES as u64, to)?.to_le_bytes();
    // `jmp rel32`, as `Asm::jmp` emits it.
    Some([0xe9, relative[0], relative[1], relative[2], relative[3]])
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

    /// The account in the frame at `frame`, which [`ACCOUNT`] left there.
    ///
    /// # Safety
    /// `frame` must be a live `GuestState`'s address.
    pub(super) unsafe fn account(frame: u64) -> Account {
        // SAFETY: the caller's contract.
        Account(unsafe { (*(frame as *const crate::GuestState)).account })
    }

    /// Put `account` in the frame at `frame`, for [`ACCOUNT`] to take.
    ///
    /// # Safety
    /// As for [`account`].
    pub(super) unsafe fn set_account(frame: u64, account: Account) {
        // SAFETY: the caller's contract.
        unsafe { (*(frame as *mut crate::GuestState)).account = account.0 };
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    /// The link tier of this host ([`LinkGate::for_target`]).
    pub(super) fn gate() -> LinkGate {
        LinkGate::for_target(crate::TargetCpu::detect())
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    /// A control with the account of the runs that use it, which native
    /// code keeps in [`ACCOUNT`] and the engine apart: comparing two
    /// compares both.
    #[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
    pub(super) struct Controlled {
        pub control: ChainControl,
        pub account: Account,
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    impl core::ops::Deref for Controlled {
        type Target = ChainControl;
        fn deref(&self) -> &ChainControl {
            &self.control
        }
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    impl core::ops::DerefMut for Controlled {
        fn deref_mut(&mut self) -> &mut ChainControl {
            &mut self.control
        }
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    impl Controlled {
        /// Run `run` with a frame that finds the control and holds the
        /// account, then keep the account it left.
        pub(super) fn run<T>(&mut self, run: impl FnOnce(&mut crate::GuestState) -> T) -> T {
            let mut frame = alloc::boxed::Box::new(crate::GuestState::default());
            frame.anchors[Anchor::Control] = &mut self.control as *mut ChainControl as u64;
            frame.account = self.account.0;
            let result = run(&mut frame);
            self.account = Account(frame.account);
            result
        }

        /// Post an interruption ([`interrupt`]).
        pub(super) fn interrupt(&mut self) {
            // SAFETY: the control is this one's, which no native code runs on.
            unsafe { interrupt(&mut self.control) };
        }

        /// Acknowledge the epoch the control holds ([`acknowledge`]).
        pub(super) fn acknowledge(&mut self) {
            let epoch = self.control.interrupt_epoch;
            // SAFETY: as for `interrupt`.
            unsafe { acknowledge(&mut self.control, epoch) };
        }
    }

    fn lookup_slot(pc: u64, key: u64) -> usize {
        super::lookup_slot(pc, key, TABLE_MASK)
    }

    fn itlb_slot(pc: u64, context: u64) -> usize {
        super::itlb_slot(pc, context, ITLB_MASK)
    }

    fn itlb_salt(context: u64) -> u32 {
        super::itlb_salt(context, ITLB_MASK)
    }

    /// Tables grow with the arena, in powers of two within their bounds
    /// (the block table's at least one entry per 256 bytes of it): a 64 MiB
    /// arena keeps the defaults.
    #[test]
    fn tables_are_sized_from_the_arena() {
        const MIB: usize = 1 << 20;
        assert_eq!(lookup_entries(64 * MIB), LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(64 * MIB), ITLB_ENTRIES);
        assert_eq!(lookup_entries(16 * MIB), MIN_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(16 * MIB), MIN_ITLB_ENTRIES);
        assert_eq!(lookup_entries(100 * MIB), 2 * LOOKUP_ENTRIES);
        assert_eq!(lookup_entries(512 * MIB), 1 << 21);
        assert_eq!(lookup_entries(976 * MIB), 1 << 22);
        assert_eq!(lookup_entries(64 * 1024 * MIB), MAX_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(128 * MIB), 1 << 15);
        assert_eq!(itlb_entries(256 * MIB), MAX_ITLB_ENTRIES);
        assert_eq!(itlb_entries(720 * MIB), MAX_ITLB_ENTRIES);
        assert_eq!(lookup_entries(0), MIN_LOOKUP_ENTRIES);
        assert_eq!(itlb_entries(0), MIN_ITLB_ENTRIES);
        // A board's first table: half, which grows as blocks fill it.
        assert_eq!(first_lookup_entries(512 * MIB), 1 << 20);
        assert_eq!(first_lookup_entries(640 * MIB), 1 << 21);
        assert_eq!(first_lookup_entries(976 * MIB), 1 << 21);
        assert_eq!(first_lookup_entries(16 * MIB), MIN_LOOKUP_ENTRIES);
        assert_eq!(first_lookup_entries(64 * 1024 * MIB), MAX_LOOKUP_ENTRIES);
    }

    /// An arena full of blocks of the size a launch measured (280 bytes a
    /// block with its exits: 2.9 M blocks in 26 of a 976 MiB window's 32
    /// parts) keeps most of their entries in a table sized from it; the
    /// table rounded down kept under three in four, the rest put off
    /// (measured: 99% and 70%).
    #[test]
    fn a_table_sized_from_a_full_arena_keeps_most_of_its_blocks_entries() {
        const MIB: usize = 1 << 20;
        const ARENA: usize = 96 * MIB;
        const BLOCK_BYTES: usize = 280;
        let blocks = (ARENA / BLOCK_BYTES) as u64;
        let kept = |entries: usize| {
            let mut table = LookupTable::with_entries(entries);
            let entry = |block: u64| {
                let pc = 0x1_0000_0000 + 12 * block;
                let key = block_key(0x40_0000_0000 + 12 * block, 1);
                LookupEntry {
                    pc,
                    key,
                    entry: 1,
                    block: block as u32,
                    targets: 1,
                }
            };
            for block in 0..blocks {
                let held = entry(block);
                table.insert(table.slot(held.pc, held.key), held);
            }
            let found = (0..blocks)
                .filter(|&block| {
                    let held = entry(block);
                    table.find(table.slot(held.pc, held.key), held.pc, held.key) == held
                })
                .count();
            found as f64 / blocks as f64
        };
        let sized = kept(lookup_entries(ARENA));
        let rounded_down = kept(1 << (ARENA / CODE_BYTES_PER_ENTRY).ilog2());
        assert!(sized >= 0.95, "{sized:.3} kept");
        assert!(rounded_down < 0.75, "{rounded_down:.3} kept rounded down");
    }

    /// The same arena's table, begun at half the size and grown as its
    /// blocks fill it ([`LookupTable::grow`]), ends at the size the arena
    /// gives ([`lookup_entries`]) and keeps as many of their entries, once
    /// the fetches of the entries a full set put off add them again.
    #[test]
    fn a_table_grown_with_a_full_arena_keeps_most_of_its_blocks_entries() {
        const MIB: usize = 1 << 20;
        const ARENA: usize = 96 * MIB;
        const BLOCK_BYTES: usize = 280;
        let blocks = (ARENA / BLOCK_BYTES) as u64;
        let entry = |block: u64| {
            let pc = 0x1_0000_0000 + 12 * block;
            let key = block_key(0x40_0000_0000 + 12 * block, 1);
            LookupEntry {
                pc,
                key,
                entry: 1,
                block: block as u32,
                targets: 1,
            }
        };
        let mut table = LookupTable::with_entries(first_lookup_entries(ARENA));
        assert_eq!(table.entries(), lookup_entries(ARENA) / 2);
        let mut most_steps = 0;
        let mut steps = 0;
        let held = |table: &LookupTable, block: u64| {
            let held = entry(block);
            table.find(table.slot(held.pc, held.key), held.pc, held.key) == held
        };
        for pass in 0..2 {
            for block in 0..blocks {
                if pass == 1 && held(&table, block) {
                    continue;
                }
                let held = entry(block);
                table.insert(table.slot(held.pc, held.key), held);
                let growing = table.growing();
                table.grow(block as usize + 1);
                if table.growing() || growing {
                    steps += 1;
                } else {
                    most_steps = most_steps.max(steps);
                    steps = 0;
                }
                if table.holds_retired() {
                    // SAFETY: no reader.
                    unsafe { table.release_retired() };
                }
            }
        }
        assert_eq!(table.entries(), lookup_entries(ARENA));
        assert_eq!(table.growths(), 1);
        let sets = lookup_entries(ARENA) / 4;
        assert_eq!(most_steps, sets.div_ceil(GROW_STEP_SETS));
        let kept = (0..blocks).filter(|&block| held(&table, block)).count();
        let kept = kept as f64 / blocks as f64;
        assert!(kept >= 0.95, "{kept:.3} kept");
    }

    /// The entry of `pc` with `key` as native code finds it: through the
    /// host address and slot mask a PE's `ChainControl` holds.
    fn probe(table: u64, mask: u32, pc: u64, key: u64) -> LookupEntry {
        let slot = |index: usize| {
            // SAFETY: a table of `mask + 1` slots the caller keeps alive.
            unsafe { &*(table as *const Slot).add(index) }.load()
        };
        let home = super::lookup_slot(pc, key, mask);
        let first = slot(home);
        if first.pc == pc && first.key == key {
            return first;
        }
        let second = slot(neighbour(home));
        if second.pc == pc { second } else { first }
    }

    /// An entry of block `block` whose fields check each other, so a
    /// reader tells it from bytes no writer wrote whole.
    fn checked(pc: u64, block: u32) -> LookupEntry {
        LookupEntry {
            pc,
            key: block_key(pc ^ 0x55_0000_0000, 1),
            entry: pc.rotate_left(17) ^ 0x5eed,
            block,
            targets: 1,
        }
    }

    fn is_checked(entry: &LookupEntry) -> bool {
        entry.key == 0 || *entry == checked(entry.pc, entry.block)
    }

    /// While a block table doubles twice under inserts, PEs probing it
    /// natively through the address and mask they took at their last
    /// refresh, and lookups without the lock, find every entry no insert
    /// put out, whole: neither reads a table before its last slot was
    /// written. The old table goes after both passed a refresh since its
    /// successor was published (a grace).
    #[test]
    fn readers_find_every_entry_while_the_table_grows() {
        use std::sync::atomic::AtomicBool;
        let mut table = LookupTable::with_entries(MIN_LOOKUP_ENTRIES);
        let old_set = |entry: &LookupEntry| {
            super::lookup_slot(entry.pc, entry.key, MIN_LOOKUP_ENTRIES as u32 - 1) / 2
        };
        // Kept entries alone in their sets of the first table, so no
        // insert puts them out of it or of its successors.
        let all: Vec<LookupEntry> = (0..200_000u32)
            .map(|block| checked(0x2_0000_0000 + 28 * u64::from(block), block))
            .collect();
        let mut taken = alloc::collections::BTreeSet::new();
        let kept: Vec<LookupEntry> = all
            .iter()
            .step_by(64)
            .filter(|entry| taken.len() < 512 && taken.insert(old_set(entry)))
            .copied()
            .collect();
        let filler: Vec<LookupEntry> = all
            .iter()
            .filter(|entry| !taken.contains(&old_set(entry)))
            .copied()
            .collect();
        for entry in &kept {
            table.insert(table.slot(entry.pc, entry.key), *entry);
        }
        let reader = table.reader();
        let done = AtomicBool::new(false);
        let passes = [AtomicU64::new(0), AtomicU64::new(0)];
        std::thread::scope(|scope| {
            // A PE's native probes, refreshed at each pass as at each run.
            scope.spawn(|| {
                while !done.load(Ordering::Acquire) {
                    let (address, mask) = reader.current();
                    for entry in &kept {
                        let found = probe(address, mask, entry.pc, entry.key);
                        assert_eq!(found, *entry, "a kept entry, natively");
                    }
                    for entry in filler.iter().step_by(97) {
                        let found = probe(address, mask, entry.pc, entry.key);
                        assert!(is_checked(&found), "{found:?} read whole");
                    }
                    passes[0].fetch_add(1, Ordering::Release);
                }
            });
            // The engine's lookups without the lock.
            scope.spawn(|| {
                while !done.load(Ordering::Acquire) {
                    for entry in &kept {
                        assert_eq!(reader.lookup(entry.pc, entry.key), *entry);
                    }
                    for entry in filler.iter().step_by(89) {
                        assert!(is_checked(&reader.lookup(entry.pc, entry.key)));
                    }
                    passes[1].fetch_add(1, Ordering::Release);
                }
            });
            let mut grace = None;
            let mut live = kept.len();
            for entry in &filler {
                table.insert(table.slot(entry.pc, entry.key), *entry);
                live += 1;
                if let Some(then) = &grace {
                    let now = passes.each_ref().map(|pass| pass.load(Ordering::Acquire));
                    // A pass begun before the snapshot may hold the old
                    // table: two passes since.
                    if now.iter().zip(then).all(|(now, then)| now >= &(then + 2)) {
                        // SAFETY: each reader read the table again since.
                        unsafe { table.release_retired() };
                        grace = None;
                    }
                } else if table.grow(live) {
                    grace = Some(passes.each_ref().map(|pass| pass.load(Ordering::Acquire)));
                }
                if table.growths() == 2 && grace.is_none() {
                    break;
                }
            }
            done.store(true, Ordering::Release);
        });
        assert_eq!(table.growths(), 2);
        assert_eq!(table.entries(), 4 * MIN_LOOKUP_ENTRIES);
        for entry in &kept {
            assert_eq!(
                table.find(table.slot(entry.pc, entry.key), entry.pc, entry.key),
                *entry
            );
        }
    }

    /// Entries forgotten or put out while a table grows leave its copy,
    /// and once it is published, the old table that PEs may still probe:
    /// the old table holds only entries the new one holds alike.
    #[test]
    fn a_growing_table_forgets_in_its_copy_and_its_old_table() {
        let mut table = LookupTable::with_entries(MIN_LOOKUP_ENTRIES);
        let entries: Vec<LookupEntry> = (0..40_000u32)
            .map(|block| checked(0x3_0000_0000 + 4 * u64::from(block) * 5, block))
            .collect();
        for entry in &entries {
            table.insert(table.slot(entry.pc, entry.key), *entry);
        }
        let in_table = |table: &LookupTable, entry: &LookupEntry| {
            table.find(table.slot(entry.pc, entry.key), entry.pc, entry.key) == *entry
        };
        // Half the copy made: forget entries in sets on either side of it.
        assert!(!table.grow(entries.len() * 2));
        assert!(table.growing());
        let (old, old_mask) = table.reader().current();
        let forgotten: Vec<LookupEntry> = entries
            .iter()
            .step_by(7)
            .filter(|entry| in_table(&table, entry))
            .copied()
            .collect();
        for entry in &forgotten {
            table.forget_block(entry.pc, entry.key, entry.block);
        }
        while !table.grow(entries.len() * 2) {}
        let (new, new_mask) = table.reader().current();
        assert_eq!(new_mask, 2 * old_mask + 1);
        let absent = |table: u64, mask: u32, entry: &LookupEntry| {
            let found = probe(table, mask, entry.pc, entry.key);
            (found.pc, found.key) != (entry.pc, entry.key)
        };
        for entry in &forgotten {
            assert!(
                absent(new, new_mask, entry),
                "{entry:?} forgotten in the copy"
            );
        }
        // Every entry of the old table is the new one's too, and stays so
        // through frames forgotten and entries put out after.
        let frames: Vec<u64> = entries
            .iter()
            .step_by(1000)
            .map(|entry| key_frame(entry.key))
            .collect();
        for &frame in &frames {
            table.forget_frames(frame..=frame, |_| true);
        }
        for block in 0..20_000u32 {
            let entry = checked(0x7_0000_0000 + 4 * u64::from(block) * 3, block);
            table.insert(table.slot(entry.pc, entry.key), entry);
        }
        let mut held = 0;
        for index in 0..=old_mask as usize {
            // SAFETY: the old table is held until `release_retired`.
            let entry = unsafe { &*(old as *const Slot).add(index) }.load();
            if entry.key != 0 {
                held += 1;
                assert!(in_table(&table, &entry), "{entry:?} in the new table");
            }
        }
        assert!(held > 10_000, "{held} entries left in the old table");
        for entry in &forgotten {
            assert!(
                absent(old, old_mask, entry),
                "{entry:?} forgotten in the old"
            );
        }
        // SAFETY: no reader.
        unsafe { table.release_retired() };
        assert!(!table.holds_retired());
        assert_eq!(
            table.table_bytes(),
            2 * MIN_LOOKUP_ENTRIES * size_of::<LookupEntry>()
        );
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
                let linking = link(at, slot, 3, 7, 0x10_8000, LinkGate::Jrcxz).unwrap();
                assert_eq!(linking.rest_at, point + PATCH_BYTES as u64);
                let start = (linking.rest_at - at) as usize;
                // The body fills the slot from its point to the exit.
                assert_eq!(start + linking.rest.len(), slot);
                // The point's instruction is the first of the body's, the
                // control's load, whose bytes go on past the point.
                let mut first = Code::new();
                first.asm().place(Rdx, CONTROL);
                let first = first.into_bytes();
                assert!(first.len() > PATCH_BYTES);
                assert_eq!(linking.body()[..first.len()], first);
                assert_eq!(linking.point, LINKED_SLOT);
                // The body's every way out of the slot: the target, or the
                // exit right after it.
                let mut linked = code.clone();
                linked[start..start + linking.rest.len()].copy_from_slice(&linking.rest);
                linked[offset..offset + PATCH_BYTES].copy_from_slice(&linking.point);
                assert_eq!(linked[slot..], code[slot..]);
                // An exit without a slot is patched at its point.
                let (exit_point, ret) = unlinked(site, 0).unwrap();
                assert_eq!(exit_point, direct_patch(site));
                let exit = (exit_point - at) as usize;
                assert_eq!(code[exit..exit + PATCH_BYTES], ret);
            }
        }
        assert!(link(0, LINK_BYTES + 1, 3, 7, 0, LinkGate::Jrcxz).is_none());
        let word = 0x1122_3344_5566_7788;
        assert_eq!(patch_word(word, 0x1003, RET_ROOM), 0x0000_441f_0f66_7788);
    }

    #[test]
    fn thunks_count_name_and_jump_or_fall_back() {
        let code = thunk(0x1000, 7, 3, 0x2000, 0x800, LinkGate::Jrcxz).unwrap();
        // mov rdx, [rdi + control]
        let mut expected = vec![0x48, 0x8b, 0x97];
        expected.extend(824u32.to_le_bytes());
        // The gate without the flags: mov ecx, [rdx], the quiet word, whose
        // JRCXZ goes to the jump back.
        expected.extend([0x8b, 0x0a, 0xe3, 26]);
        // movzx ecx, r13w; JRCXZ there too when no transfer is left; then
        // lea r13, [r13 + (7 << 16) - 1]: one transfer and 7 instructions.
        expected.extend([0x41, 0x0f, 0xb7, 0xcd, 0xe3, 20]);
        expected.extend([0x4d, 0x8d, 0xad, 0xff, 0xff, 0x06, 0x00]);
        expected.extend([0x48, 0xc7, 0x42, 0x08, 3, 0, 0, 0]);
        // The jump ends 0x25 bytes into the thunk.
        expected.extend([0xe9]);
        expected.extend((0x2000i32 - 0x1025).to_le_bytes());
        assert_eq!(code.len(), THUNK_BYTES);
        assert_eq!(code[..POLLED_LINK_BODY_BYTES], expected);
        // The jump back, nothing spent.
        assert_eq!(code[POLLED_LINK_BODY_BYTES], 0xe9);
        assert_eq!(
            code[POLLED_LINK_BODY_BYTES + 1..],
            (0x800i32 - 0x1000 - code.len() as i32).to_le_bytes()
        );
        // The thunk of a memory account took 51 bytes, 39 of them run.
        const _: () = assert!(THUNK_BYTES <= 46);
        // A block of more instructions than an 8-bit displacement holds.
        let long = thunk(0x1000, 200, 3, 0x2000, 0x800, LinkGate::Jrcxz).unwrap();
        assert_eq!(long.len(), THUNK_BYTES);
        assert!(thunk(0, 0, 0, 1 << 40, 0x800, LinkGate::Jrcxz).is_none());
        // The PEXT tier: pext ecx, r13d, [rdx], the transfers part under the
        // quiet word, and one JRCXZ to the jump back; the rest as above.
        let code = thunk(0x1000, 7, 3, 0x2000, 0x800, LinkGate::Pext).unwrap();
        let body = LINK_BODY_BYTES + 1;
        let mut expected = vec![0x48, 0x8b, 0x97];
        expected.extend(824u32.to_le_bytes());
        expected.extend([0xc4, 0xe2, 0x12, 0xf5, 0x0a, 0xe3, 20]);
        expected.extend([0x4d, 0x8d, 0xad, 0xff, 0xff, 0x06, 0x00]);
        expected.extend([0x48, 0xc7, 0x42, 0x08, 3, 0, 0, 0, 0xe9]);
        expected.extend((0x2000i32 - 0x1000 - body as i32).to_le_bytes());
        assert_eq!(code[..body], expected);
        assert_eq!(code.len(), body + JUMP_BYTES);
        // Either tier's slot body fits the slot.
        for gate in [LinkGate::Jrcxz, LinkGate::Pext] {
            let linking = link(0x1000, POLLED_LINK_BYTES, 7, 3, 0x2000, gate).unwrap();
            assert_eq!(linking.body().len(), POLLED_LINK_BYTES - patch_pad(0x1000));
        }
    }

    /// The PEXT tier is a fast-PDEP host's: Zen 1 and 2 microcode PEXT.
    #[test]
    fn link_gates_follow_the_target() {
        use crate::TargetCpu;
        assert_eq!(LinkGate::for_target(TargetCpu::Baseline), LinkGate::Jrcxz);
        assert_eq!(
            LinkGate::for_target(TargetCpu::X86_64_V2.with_bmi2()),
            LinkGate::Jrcxz
        );
        assert_eq!(
            LinkGate::for_target(TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep()),
            LinkGate::Pext
        );
    }

    /// An account's transfers and instructions are apart: a transfer's LEA,
    /// which runs only with a transfer left, adds its block's instructions
    /// less one transfer, and a settle below zero leaves the transfers as
    /// they were.
    #[test]
    fn accounts_keep_transfers_apart_from_instructions() {
        let transfer = |account: Account, ran: u32| {
            assert_ne!(account.budget(), 0, "a check stops a run with none left");
            Account(
                account
                    .0
                    .wrapping_add(transfer_change(ran).unwrap() as i64 as u64),
            )
        };
        let full = Account::new(MAX_BUDGET);
        assert_eq!((full.budget(), full.retired()), (MAX_BUDGET, 0));
        let spent = transfer(Account::new(1), 7);
        assert_eq!(
            (spent, spent.budget(), spent.retired()),
            (Account::spent(7), 0, 7)
        );
        // A settle 5 below zero, then a transfer of 9: the transfers part
        // never borrowed.
        let settled = Account::new(2).with_retired((-5i64) as u64);
        assert_eq!((settled.budget(), settled.retired() as i64), (2, -5));
        let after = transfer(settled, 9);
        assert_eq!((after.budget(), after.retired()), (1, 4));
        // The most instructions a transfer adds fit its displacement.
        let ran = (1 << (u32::BITS - ExitWord::RAN_SHIFT)) - 1;
        assert!(transfer_change(ran).is_some());
        let most = transfer(Account::new(MAX_BUDGET), ran);
        assert_eq!(
            (most.budget(), most.retired()),
            (MAX_BUDGET - 1, u64::from(ran))
        );
        assert_eq!(
            Account::new(3).with_budget(0).with_retired(11),
            Account::spent(11)
        );
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
            let thunk = |at: *const u8| {
                move |frame: u64, target: u64| {
                    crate::x86_64::native::enter_counted(
                        at,
                        frame as *mut crate::GuestState,
                        target,
                        0,
                    )
                }
            };
            let dynamic = thunk(page as *const u8);
            let fixed = thunk(base.add(fixed_at));
            let branch = thunk(base.add(branch_at));
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
            set_account(frame, account(frame).with_budget(2));
            // No translation of the target's page yet: the exit returns.
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!(account(frame).budget(), 2);
            translate(&mut itlb, TARGET, 7);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!(
                (
                    account(frame).budget(),
                    account(frame).retired(),
                    control.hits,
                    control.block
                ),
                (1, 9, 1, 42)
            );
            assert_eq!(exit(fixed(frame, 0)), (0x5151, 99));
            assert_eq!(
                (
                    account(frame).budget(),
                    account(frame).retired(),
                    control.hits
                ),
                (0, 12, 2)
            );
            // No budget left: the exit is taken, nothing spent.
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!((account(frame).budget(), account(frame).retired()), (0, 12));
            // A settle before the exit left the count 5 below zero: the
            // probe's 9 instructions repay it, and the one transfer left,
            // which the settle never touched, is taken.
            let hits = control.hits;
            set_account(frame, Account::new(1).with_retired((-5i64) as u64));
            assert_eq!(
                (account(frame).budget(), account(frame).retired()),
                (1, (-5i64) as u64)
            );
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!((account(frame).budget(), account(frame).retired()), (0, 4));
            assert_eq!(control.hits, hits + 1);
            control.hits = hits;
            // With none left it exits, the count as it was.
            set_account(frame, Account::new(0).with_retired((-5i64) as u64));
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            assert_eq!(
                (account(frame).budget(), account(frame).retired()),
                (0, (-5i64) as u64)
            );
            set_account(frame, account(frame).with_retired(12));
            set_account(frame, account(frame).with_budget(5));
            // Another PC in the slot misses.
            let other = TARGET ^ (1 << (2 + LOOKUP_BITS) | 1 << 2);
            translate(&mut itlb, other, 7);
            assert_eq!(lookup_slot(other, KEY), lookup_slot(TARGET, KEY));
            assert_eq!(
                exit(dynamic(frame, other)),
                (other, ExitKind::Return as u64)
            );
            // The set's other entry serves too.
            let (hits, retired) = (control.hits, account(frame).retired());
            table.insert(lookup_slot(TARGET, KEY), LookupEntry { pc: other, ..entry });
            assert_eq!(table.entry(neighbour(lookup_slot(TARGET, KEY))).pc, TARGET);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!(table.find(lookup_slot(TARGET, KEY), TARGET, KEY).pc, TARGET);
            assert_eq!(exit(dynamic(frame, other)), (0x5151, 99));
            assert_eq!(control.hits, hits + 2);
            set_account(frame, account(frame).with_retired(retired));
            control.hits = hits;
            set_account(frame, account(frame).with_budget(5));
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
            assert_eq!(
                (account(frame).budget(), control.hits, control.block),
                (5, 2, 42)
            );
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
            assert_eq!((account(frame).budget(), control.hits), (4, 3));
            // The lower half's context translates a lower-half target, and
            // the upper half's one the lower half's lacks (a global page).
            let lower = TARGET & !(1 << 55);
            table.set(lookup_slot(lower, KEY), LookupEntry { pc: lower, ..entry });
            translate(&mut itlb, lower, 9);
            let before = (account(frame), control.hits);
            assert_eq!(exit(dynamic(frame, lower)), (0x5151, 99));
            set_account(frame, before.0);
            control.hits = before.1;
            itlb.forget_range_where(lower & !0xfff..=lower & !0xfff, |_| true);
            assert_eq!(
                exit(dynamic(frame, lower)),
                (lower, ExitKind::Return as u64)
            );
            // An upper-half target is never translated in the lower half's.
            control.contexts = [9, 5];
            control.salts = [itlb_salt(9), itlb_salt(5)];
            assert_eq!(
                exit(dynamic(frame, TARGET)),
                (TARGET, ExitKind::Return as u64)
            );
            control.contexts = [5, 9];
            control.salts = [itlb_salt(5), itlb_salt(9)];
            translate(&mut itlb, lower, 5);
            assert_eq!(exit(dynamic(frame, lower)), (0x5151, 99));
            assert_eq!((account(frame).budget(), control.hits), (3, 4));
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
                (
                    account(frame).budget(),
                    account(frame).retired(),
                    control.hits,
                    control.block
                ),
                (2, 35, 5, 3 << BLOCK_BTYPE_SHIFT | 42)
            );
            set_account(frame, account(frame).with_budget(5));
            interrupt(&mut control);
            let prior = (
                account(frame).budget(),
                account(frame).retired(),
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
                    account(frame).budget(),
                    account(frame).retired(),
                    control.hits,
                    control.linked,
                    control.block
                ),
                prior
            );
            acknowledge(&mut control, 1);
            assert_eq!(exit(dynamic(frame, TARGET)), (0x5151, 99));
            assert_eq!((account(frame).budget(), control.block), (4, 42));
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

    /// The census of held cells: those past the stamped ones, which no
    /// promotion reaches, and those holding no entry, follow links,
    /// relinks, the table's entries and releases.
    #[test]
    fn the_cell_census_counts_cells_past_the_stamped_ones_and_empty_ones() {
        const PC: u64 = 0xffff_fe00_0123_4000;
        let key = block_key(5 << PAGE_SHIFT, 1);
        let entry = LookupEntry {
            pc: PC,
            key,
            entry: 0x100,
            block: 9,
            targets: 1,
        };
        let mut table = LookupTable::new();
        let census = |table: &LookupTable| {
            (
                table.cells(),
                table.cells_past_stamped(),
                table.cells_empty(),
            )
        };
        // Plain links take cells past the stamped ones, one of them empty.
        let address = table.link_cell(1, PC, Some(entry));
        table.link_cell(2, PC + 4, None);
        assert_eq!(census(&table), (2, 2, 1));
        // A relink keeps one cell for its exit.
        table.link_cell(2, PC + 4, None);
        assert_eq!(census(&table), (2, 2, 1));
        // The table's entry of its PC fills the empty one.
        table.set(
            lookup_slot(PC + 4, key),
            LookupEntry {
                pc: PC + 4,
                ..entry
            },
        );
        assert_eq!(census(&table), (2, 2, 0));
        // A hot exit moves to a stamped cell, and its old cell is free.
        let hot = table.link_hot_cell(1, PC, Some(entry)).unwrap();
        assert_ne!(hot, address);
        assert_eq!(census(&table), (2, 1, 0));
        assert_eq!(table.link_cell(3, PC, Some(entry)), address);
        // Every stamped cell held: no more hot ones.
        for site in 4..2 + STAMPED_CELLS as u64 {
            assert!(table.link_hot_cell(site, PC, Some(entry)).is_some());
        }
        assert!(!table.has_free_stamped());
        assert_eq!(table.link_hot_cell(1 << 40, PC, Some(entry)), None);
        assert_eq!(census(&table), (STAMPED_CELLS + 1, 2, 0));
        // Forgetting the frame empties every cell of it.
        table.forget_frames(5..=5, |_| true);
        assert_eq!(census(&table), (STAMPED_CELLS + 1, 2, STAMPED_CELLS + 1));
        // Releases go from the counts, and free their stamped cells.
        table.release_cells(1..=3);
        assert_eq!(census(&table), (STAMPED_CELLS - 2, 0, STAMPED_CELLS - 2));
        assert!(table.has_free_stamped());
        table.release_cells(0..=u64::MAX);
        assert_eq!(census(&table), (0, 0, 0));
    }

    /// A cell probe's sample, the cell's address, names the exit holding
    /// the cell with what it was linked with, stamped or past the stamped
    /// ones; a released cell names none.
    #[test]
    fn a_sampled_cell_names_its_exits_link() {
        const PC: u64 = 0xffff_fe00_0123_4000;
        let mut table = LookupTable::new();
        let link = |site: u64| CellLink {
            site,
            pc: PC,
            retired: 7,
            word: ExitWord {
                kind: ExitKind::Call,
                skipped: 3,
            },
            btype: 2,
        };
        let mut cells = Vec::new();
        for site in 1..=300u64 {
            let address = table.link_cell(site, PC, None);
            let number = table.site_cell_number(site).unwrap();
            table.note_link(number, link(site));
            cells.push((site, address, number));
        }
        let hot = table.link_hot_cell(1000, PC, None).unwrap();
        let number = table.site_cell_number(1000).unwrap();
        assert!((number as usize) < STAMPED_CELLS);
        table.note_link(number, link(1000));
        cells.push((1000, hot, number));
        for &(site, address, number) in &cells {
            assert_eq!(table.sampled_link(address), Some((number, link(site))));
        }
        assert_eq!(
            table.sampled_link(cells[0].1 + 8),
            table.sampled_link(cells[0].1)
        );
        assert_eq!(table.sampled_link(0x1000), None);
        table.release_cells(1..=1);
        assert_eq!(table.sampled_link(cells[0].1), None);
    }

    /// A lookup tail jumps to its probe; a cell tail calls its probe and
    /// jumps through R9 itself, the site's own indirect jump. A probe set's
    /// pieces keep their places in every policy.
    #[test]
    fn tails_jump_to_the_probes() {
        let mut plain = Code::new();
        tail(&mut plain, 0x1000, ExitKind::Return.into(), 3, 0x2000).unwrap();
        // EDX the word with the instructions run, the call, then the jump
        // into the block the probe found.
        let mut expected = vec![0xba];
        expected.extend((ExitKind::Return as u32 | 3 << ExitWord::RAN_SHIFT).to_le_bytes());
        expected.push(0xe8);
        expected.extend((0x2000i32 - 0x100a).to_le_bytes());
        expected.extend([0x41, 0xff, 0xe1]);
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
        let cell = cell_tail(
            0x1000,
            0xdead_beef_0000,
            0x4000,
            ExitKind::Return,
            3,
            0x2000,
        )
        .unwrap();
        assert_eq!(&cell[..2], &[0x48, 0xb8]);
        assert_eq!(&cell[2..10], &0x4000u64.to_le_bytes());
        assert_eq!(&cell[10..12], &[0x49, 0xb9]);
        assert_eq!(&cell[12..20], &0xdead_beef_0000u64.to_le_bytes());
        assert_eq!(&cell[20..25], &expected[..5]);
        // call probe; jmp r9.
        assert_eq!(&cell[25..30], &[0xe8, 0xe2, 0x0f, 0, 0]);
        assert_eq!(&cell[30..], &[0x41, 0xff, 0xe1]);
        assert!(lookup_tail(0, ExitKind::Return, 0, 1 << 40, 0).is_none());
        assert!(lookup_tail(0, ExitKind::Return, 1 << 12, 0, 0).is_none());
        for btype in 0..4 {
            let probes = lookup_dispatcher(btype).unwrap();
            // Each probe starts with the control's address.
            for at in [CELL_PROBE_ENTRY, PROBE_ENTRY] {
                assert_eq!(&probes[at..at + 3], &[0x48, 0x8b, 0x8f]);
            }
            #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
            assert!(dispatcher_with_policy(btype, Interruption::Legacy).is_some());
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
                let run = |frame: u64| {
                    crate::x86_64::native::enter_counted(
                        page as *const u8,
                        frame as *mut crate::GuestState,
                        0,
                        0,
                    )
                };
                let exit = |result: u128| {
                    let word = (result >> 64) as u64;
                    (result as u64, ExitWord::split(word).0)
                };
                set_account(frame, account(frame).with_budget(3));
                // The target's page untranslated: the exit returns.
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(account(frame).budget(), 3);
                let translated = LookupEntry::page(target & !0xfff, context, KEY);
                itlb.insert(itlb_slot(target, context), translated);
                assert_eq!(exit(run(frame)), (0x5111, 99));
                assert_eq!(
                    (
                        account(frame).budget(),
                        account(frame).retired(),
                        control.hits,
                        control.block
                    ),
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
                assert_eq!(account(frame).budget(), 2);
                table.set(lookup_slot(target, KEY), table_entry);
                // The set filled the cell with the table's entry.
                assert_eq!(table.site_cell(1), Some(table_entry));
                *(cell as *mut LookupEntry) = LookupEntry {
                    key: KEY + 1,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!(
                    (
                        account(frame).budget(),
                        account(frame).retired(),
                        control.hits,
                        control.block
                    ),
                    (1, 10, 1, 43)
                );
                // A cell of another PC is never entered.
                *(cell as *mut LookupEntry) = LookupEntry {
                    pc: target + 4,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!((account(frame).budget(), control.hits), (0, 2));
                // No budget: the exit returns, nothing spent.
                *(cell as *mut LookupEntry) = cell_entry;
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!((account(frame).budget(), account(frame).retired()), (0, 15));
                // Another context of the same salt: the page translates
                // anew, whatever the cell holds.
                set_account(frame, account(frame).with_budget(5));
                control.contexts[half] = context + LOOKUP_SALTS as u64;
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(account(frame).budget(), 5);
                control.contexts[half] = context;
                // A cell a writer holds (or changed while it was read): the
                // table probe looks the target up anew by its key and finds
                // the table's entry, never a key made of the cell's block.
                *(cell as *mut LookupEntry) = LookupEntry {
                    targets: 1 | META_BUSY,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (0x5122, 99));
                assert_eq!(
                    (account(frame).budget(), control.hits, control.block),
                    (4, 3, 43)
                );
                *(cell as *mut LookupEntry) = cell_entry;
                set_account(frame, account(frame).with_budget(5));
                // A pending interruption: nothing spent or counted.
                interrupt(&mut control);
                let prior = (
                    account(frame).budget(),
                    account(frame).retired(),
                    control.hits,
                    control.block,
                );
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                *(cell as *mut LookupEntry) = LookupEntry {
                    key: 0,
                    ..cell_entry
                };
                assert_eq!(exit(run(frame)), (target, ExitKind::Call as u64));
                assert_eq!(
                    (
                        account(frame).budget(),
                        account(frame).retired(),
                        control.hits,
                        control.block
                    ),
                    prior
                );
                acknowledge(&mut control, 1);
                *(cell as *mut LookupEntry) = cell_entry;
                assert_eq!(exit(run(frame)), (0x5111, 99));
                assert_eq!((account(frame).budget(), control.block), (4, 42));
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
            let ret = |frame: u64, target: u64| {
                crate::x86_64::native::enter_counted(
                    page as *const u8,
                    frame as *mut crate::GuestState,
                    target,
                    0,
                )
            };
            let exit = |result: u128| (result as u64, ExitWord::split((result >> 64) as u64).0);
            set_account(frame, account(frame).with_budget(100));
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
            assert_eq!((control.hits, account(frame).retired()), (9, 36));
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
            assert_eq!((control.hits, account(frame).budget()), (10, 90));
            assert_eq!(munmap(page, PAGE), 0);
        }
    }

    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod budget;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod costs;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod differential;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod jump_cache;
    #[cfg(all(target_arch = "x86_64", target_os = "linux"))]
    mod paired;
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
