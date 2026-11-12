//! Block control transfers: every branch returns to the dispatcher.
use crate::x86_64::asm::{Alu, Asm, Code, Cond, Gpr, Mem, Size};
use alloc::vec;
use alloc::vec::Vec;
pub(crate) mod checked_memory;
mod recovery;
pub(crate) use super::region::ResidentLoopPlan;

use super::chain::ExitWord;
use crate::{
    GuestAddress, NativeBlock,
    aarch64::{
        Instruction, Operation, Register, Shift, Width, integer_math, integer_unary, pc_relative,
        simd::{Operand as Simd, rearrange::Operand as Rearrange},
    },
    block::{ExitKind, MAX_BLOCK_INSTRUCTIONS, Terminator},
};

pub(super) fn exit_kind(code: &mut Code, kind: ExitKind) {
    exit_kind_with(&mut code.asm(), kind);
}

/// `mov edx,word; ret`: the exit to the target in RAX, unpadded
/// (`chain::dynamic_exit`).
pub(super) fn exit_kind_with(asm: &mut Asm, word: impl Into<ExitWord>) {
    // An exit hands the engine the state block, which must be current.
    asm.refuse_dirty_homes("a block exit");
    // MOV EDX zero-extends to the ABI's u64.
    asm.mov_imm32(Gpr::Rdx, word.into().encoded());
    asm.ret();
    // The rest of the room a jump at the RET takes, and of the pad that
    // makes the RET patchable where the code lands (`chain::dynamic_patch`).
    for _ in 6..super::chain::DYNAMIC_EXIT_BYTES {
        asm.int3();
    }
}

/// Pad the exits of `code`, a block's whose direct exits are `exits`, for
/// where they lie in it: each exit's patch point and link slot, and the
/// dynamic exit ending its terminal, if it has one. Code installs at
/// 8-byte aligned addresses, so an offset's alignment is its address's.
pub(crate) fn align_exits(
    code: &mut [u8],
    exits: &[super::chain::DirectExit],
    linked: Option<u64>,
) {
    use super::chain::{
        DYNAMIC_EXIT_BYTES, align_direct_exit, align_dynamic_exit, align_link_slot,
    };
    let mut terminal = super::chain::without_failures(code).len();
    for exit in exits {
        let slot = super::chain::link_slot(linked, exit.target, exit.kind);
        if slot != 0 {
            align_link_slot(code, exit.offset - slot, slot, 0);
        }
        align_direct_exit(code, exit.offset, 0);
        terminal = terminal.min(exit.offset - slot);
    }
    if let Some(offset) = terminal.checked_sub(DYNAMIC_EXIT_BYTES) {
        align_dynamic_exit(code, offset, 0);
    }
}

pub(crate) fn direct_exit(code: &mut Code, target: GuestAddress, kind: ExitKind) {
    direct_exit_with(&mut code.asm(), target, kind);
}

/// `movabs rax,target` and the exit of `word`, after its empty link slot in
/// code whose exits keep one.
pub(super) fn direct_exit_with(asm: &mut Asm, target: GuestAddress, word: impl Into<ExitWord>) {
    let word = word.into();
    let slot = super::chain::link_slot(asm.linked_block(), target, word.kind);
    super::chain::empty_link(asm, slot);
    asm.guest_imm64(Gpr::Rax, target);
    exit_kind_with(asm, word);
}

fn conditional_exit(
    code: &mut Code,
    taken: Option<Cond>,
    target: GuestAddress,
    fallthrough: GuestAddress,
) {
    let mut asm = code.asm();
    if let Some(taken) = taken {
        let branch = asm.jcc(taken);
        direct_exit_with(&mut asm, fallthrough, ExitKind::Fallthrough);
        // The taken branch skips the complete fallthrough exit. Both paths
        // leave saved NZCV untouched regardless of host predicate evaluation.
        asm.bind(branch);
    }
    direct_exit_with(&mut asm, target, ExitKind::Direct);
}

#[cfg(test)]
fn emit(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
) -> NativeBlock {
    emit_for_target(
        instructions,
        terminal,
        fallthrough,
        crate::TargetCpu::Baseline,
        super::MemoryChecks::DEFAULT,
    )
}

pub(crate) fn emit_ordinary_for_target(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
) -> NativeBlock {
    emit_selected(
        instructions,
        terminal,
        fallthrough,
        target,
        RegionPolicy {
            optimize: true,
            fault_table: false,
            pinned: false,
            frame_loop: None,
            self_loop: None,
        },
        super::MemoryChecks::DEFAULT,
        super::NativeSystem::NONE,
    )
    .0
}

pub(crate) fn emit_frame_loop(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
    entry: GuestAddress,
) -> NativeBlock {
    emit_selected(
        instructions,
        terminal,
        fallthrough,
        target,
        RegionPolicy {
            optimize: true,
            fault_table: false,
            pinned: false,
            frame_loop: Some(entry),
            self_loop: None,
        },
        super::MemoryChecks::DEFAULT,
        super::NativeSystem::NONE,
    )
    .0
}

/// Every artifact in this profile accepts the same invocation-local budget ABI.
pub(crate) fn emit_resident_fallback_for_target(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
) -> NativeBlock {
    let base = instructions
        .first()
        .map(|instruction| instruction.pc)
        .or_else(|| terminal.map(|(pc, _)| *pc))
        .unwrap_or(fallthrough);
    let native = emit_ordinary_for_target(instructions, terminal, fallthrough, target);
    super::region::prefix_with_budget(native, base)
}

#[cfg(test)]
pub(crate) fn emit_for_target(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
    checks: super::MemoryChecks,
) -> NativeBlock {
    emit_selected(
        instructions,
        terminal,
        fallthrough,
        target,
        RegionPolicy {
            optimize: false,
            fault_table: false,
            pinned: false,
            frame_loop: None,
            self_loop: None,
        },
        checks,
        super::NativeSystem::NONE,
    )
    .0
}

/// A block for the system-mode engine, which may also complete a terminal
/// System instruction natively and fall through: DC ZVA by
/// zeroing its block (`x86_64::zero_block`) and DC CVAC, DC CVAU and DC
/// CIVAC by reading the byte at Xt (`x86_64::cache_clean`).
#[cfg(test)]
pub(crate) fn emit_system_mode(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
    checks: super::MemoryChecks,
    native: super::NativeSystem,
) -> (NativeBlock, Vec<crate::block::FaultRecovery>) {
    let (block, recovery, _) =
        emit_system_mode_looped(instructions, terminal, fallthrough, target, checks, native);
    (block, recovery)
}

/// [`emit_system_mode`], and whether the block runs its iterations inside
/// itself ([`SelfLoop`]).
pub(crate) fn emit_system_mode_looped(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
    checks: super::MemoryChecks,
    native: super::NativeSystem,
) -> (NativeBlock, Vec<crate::block::FaultRecovery>, bool) {
    // Guest registers stay in host registers (`x86_64::pinned`), which the
    // engine's fault settlement reads. Pure spans can temporarily keep other
    // values in scratch registers; the ordinary regions use the reserved
    // bank and cannot run in this ABI. Accesses check the
    // guest range (`x86_64::guest_range`). PEs run concurrently, so atomics
    // stay locked and barriers fence.
    emit_selected(
        instructions,
        terminal,
        fallthrough,
        target,
        RegionPolicy {
            optimize: false,
            fault_table: true,
            pinned: true,
            frame_loop: None,
            self_loop: SelfLoop::admit(instructions, terminal, native),
        },
        super::MemoryChecks {
            guest_range: true,
            ..checks
        },
        native,
    )
}

/// A system-mode block whose conditional terminal branches back to its own
/// first instruction, run as a loop inside its code: each entry runs
/// iterations until the branch falls through, a side exit leaves, or another
/// iteration would retire more than [`SelfLoop::LIMIT`] instructions in the
/// entry, when the taken branch
/// exits to the block's start as it would without the loop, through its
/// link and the transfer's budget and interrupt check.
///
/// The body is emitted twice. The first copy runs the first iteration of an
/// entry with every check a block makes. The second runs the later ones and
/// branches back to itself; its accesses skip the range checks that
/// `guest_range` would skip in a block holding the body three times, so a
/// base a skipped access goes through was checked in the first copy and has
/// moved since by immediates only, within the instructions an entry runs.
///
/// `GuestState::loop_retired` counts the instructions of the iterations
/// before the running one. Every exit adds it to the chain's retired count
/// (`chain::ChainControl::retired`) and clears it; a fault's settlement does
/// so in the engine. Exits and faults then count the running iteration's
/// instructions as a block's do.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct SelfLoop {
    /// The instructions of one iteration, its terminal included.
    instructions: u8,
    /// Whether a branch the body runs past to a later instruction of the
    /// body jumps there inside the loop ([`loop_joins`]) rather than
    /// leaving by its side exit.
    joins: bool,
}

impl SelfLoop {
    /// The instructions an entry retires before its taken branch exits,
    /// at most: a block's own bound, so that an entry polls as often.
    pub(crate) const LIMIT: i32 = MAX_BLOCK_INSTRUCTIONS as i32;
    /// The longest body admitted: an entry runs at least two iterations.
    const MAX_INSTRUCTIONS: usize = 64;

    fn admit(
        instructions: &[Instruction],
        terminal: Option<&(GuestAddress, Terminator)>,
        native: super::NativeSystem,
    ) -> Option<Self> {
        let entry = instructions.first()?.pc;
        let (_, Terminator::Branch(operation)) = terminal? else {
            return None;
        };
        let target = match *operation {
            Operation::BranchCondition { target, condition } if condition < 14 => target,
            Operation::BranchZero { target, .. } | Operation::BranchBit { target, .. } => target,
            // A loop left only by the branches it runs past.
            Operation::Branch { target } => target,
            _ => return None,
        };
        // Code that leaves the block other than through its exits, or
        // calls a service, is left to links.
        let admitted = instructions.iter().all(|instruction| {
            !matches!(
                instruction.operation,
                Operation::PointerAuthentication(_)
                    | Operation::MirroredRead { .. }
                    | Operation::Amx(_)
            )
        });
        (native.self_loops
            && target == entry
            && admitted
            && instructions.len() < Self::MAX_INSTRUCTIONS)
            .then(|| Self {
                instructions: u8::try_from(instructions.len() + 1).expect("bounded body"),
                joins: true,
            })
    }
}

/// A [`SelfLoop`]'s jumps to its exits, from both copies of its body.
#[derive(Default)]
struct LoopExits {
    /// Where the second copy starts, and the source rows before it.
    start: usize,
    first_rows: usize,
    /// The back edges' jumps when the branch falls through.
    fallthrough: Vec<super::asm::Patch>,
    /// Each branch run past: its jump (none for a join, whose exit stays for
    /// `successor_exits`), target, the rows up to it and the locals its
    /// copy's run has written there.
    side: Vec<(
        Option<super::asm::Patch>,
        GuestAddress,
        usize,
        super::pinned::Dirty,
    )>,
    /// Each join's jump ([`loop_joins`]): the copy, its landing and the
    /// instructions it skips.
    joins: Vec<(super::asm::Patch, usize, usize, u8)>,
    /// Where each copy's landings start.
    landings: [Vec<Option<usize>>; 2],
    /// The locals the loop's run has written at its back edge.
    dirty: super::pinned::Dirty,
    /// The jump to the taken branch's exit once an entry's share is spent.
    taken: Option<super::asm::Patch>,
}

/// `GuestState::loop_retired`.
const LOOP_RETIRED: u32 = core::mem::offset_of!(crate::GuestState, loop_retired) as u32;

/// Move `GuestState::loop_retired`, less `less`, to the chain's retired
/// count and clear it, before an exit of a [`SelfLoop`].
fn settle_loop(asm: &mut Asm, less: u8, counts: bool) {
    asm.mov_load(Size::Qword, Gpr::Rax, super::frame(LOOP_RETIRED));
    if less != 0 {
        asm.alu_imm8(Alu::Sub, Size::Qword, Gpr::Rax, less as i8);
    }
    asm.place(Gpr::Rcx, super::chain::CONTROL_PLACE);
    asm.alu(
        Alu::Add,
        Size::Qword,
        Mem::base(Gpr::Rcx, super::chain::RETIRED_OFFSET),
        Gpr::Rax,
    );
    // The engine's counts, while it profiles: the entry and what its
    // iterations retired.
    if counts {
        asm.alu(
            Alu::Add,
            Size::Qword,
            Mem::base(Gpr::Rcx, super::chain::LOOP_ITERATED_OFFSET),
            Gpr::Rax,
        );
        asm.alu_imm8(
            Alu::Add,
            Size::Qword,
            Mem::base(Gpr::Rcx, super::chain::LOOP_ENTRIES_OFFSET),
            1,
        );
    }
    asm.alu_imm8(Alu::And, Size::Qword, super::frame(LOOP_RETIRED), 0);
}

/// A [`SelfLoop`]'s branches that skip forward inside its body: for each,
/// the branch's index and the index of the instruction it lands on (the
/// terminal's is the body's length). The taken branch jumps there inside
/// the loop's copy, after taking the instructions it skips off
/// `GuestState::loop_retired`, so an iteration's count, a fault's position
/// and the entry's bound stay exact. At each landing nothing is held: the
/// emitter forgets what registers held, its masked bases, and the range and
/// stack checks' known bases, as at a block's start. Only where one run of
/// locals covers the body, or none runs, so both paths agree on where each
/// guest register lives.
fn loop_joins(instructions: &[Instruction]) -> Vec<(usize, usize)> {
    instructions
        .iter()
        .enumerate()
        .filter_map(|(at, branch)| {
            let target = branch.operation.conditional_target()?;
            let landing = instructions[at + 1..]
                .iter()
                .position(|instruction| instruction.pc == target)
                .map(|offset| at + 1 + offset)
                .or_else(|| {
                    // The terminal, just past the body.
                    let end = instructions.last()?.pc.0.checked_add(4)?;
                    (target.0 == end).then_some(instructions.len())
                })?;
            Some((at, landing))
        })
        .collect()
}

/// Run `analysis` (`guest_range::checks`, `stack_alignment::checks`) on
/// each stretch of `instructions` between the landings in `starts`, each
/// as if a block started there.
fn segmented(
    analysis: fn(&[Instruction], &mut [bool]),
    instructions: &[Instruction],
    starts: &[bool],
    rows: &mut [bool],
) {
    let mut begin = 0;
    for end in (1..=instructions.len()).filter(|&end| end == instructions.len() || starts[end]) {
        analysis(&instructions[begin..end], &mut rows[begin..end]);
        begin = end;
    }
}

/// Selection and recovery policy for register-resident regions.
#[derive(Clone, Copy)]
struct RegionPolicy {
    optimize: bool,
    fault_table: bool,
    /// Keep guest registers in host registers ([`super::pinned`]).
    pinned: bool,
    frame_loop: Option<GuestAddress>,
    /// Run the block's iterations inside it ([`SelfLoop`]).
    self_loop: Option<SelfLoop>,
}

/// The final address of a flagless ADD that overwrites its adjacent ADR(P).
pub(super) fn fold_address_pair(
    first: &Operation,
    second: &Operation,
) -> Option<pc_relative::Operand> {
    let Operation::PcRelative(address) = *first else {
        return None;
    };
    let Operation::AddSub {
        width: Width::X64,
        subtract: false,
        flags: false,
        destination: destination @ Register::X(_),
        source,
        immediate,
    } = *second
    else {
        return None;
    };
    (destination == address.destination && source == destination).then_some(pc_relative::Operand {
        destination,
        address: GuestAddress(address.address.0.wrapping_add(u64::from(immediate))),
    })
}

/// With `fault_table`, region accesses keep dirty values in registers and
/// the returned table says where ([`crate::block::FaultRecovery`]).
fn emit_selected(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    target: crate::TargetCpu,
    regions: RegionPolicy,
    checks: super::MemoryChecks,
    native: super::NativeSystem,
) -> (NativeBlock, Vec<crate::block::FaultRecovery>, bool) {
    let RegionPolicy {
        optimize: optimize_regions,
        fault_table,
        pinned,
        frame_loop,
        self_loop,
    } = regions;
    let mut fault_recovery = vec![];
    let source_rows = instructions.len() + usize::from(terminal.is_some());
    let mut block = super::Emitted {
        source_map: Vec::with_capacity(source_rows),
        ..super::Emitted::default()
    };
    let code = &mut block.x86_64_code;
    code.set_pinned(pinned);
    if pinned
        && let Some(first) = instructions
            .first()
            .map(|first| first.pc)
            .or(terminal.map(|(pc, _)| *pc))
    {
        code.set_linked_block(first.0);
    }
    code.set_constant_time(target.data_independent_timing());
    let frame_loop = frame_loop.map(|entry| super::region::FrameLoop::begin(code, entry));
    let mut fixups = vec![];
    // The DIT audit covers each instruction's own lowering, not the
    // register-allocated and clean-load regions' recipes.
    let optimize_regions = optimize_regions && !target.data_independent_timing();
    let prefix_sum =
        if optimize_regions && !pinned && !fault_table && checks == super::MemoryChecks::DEFAULT {
            super::byte_prefix::Plan::new(instructions, terminal)
                .map(|plan| plan.emit(code, target))
        } else {
            None
        };
    let fuse_pairs = optimize_regions && frame_loop.is_none() && !fault_table && !pinned;
    let zero_branch = if cfg!(feature = "std") && fuse_pairs {
        alu_zero_branch(instructions, terminal)
    } else {
        None
    };
    // Neither region gate admits one instruction. Keep the single
    // frame ADD/SUB lowering, whose final store is a flag-preserving MOV.
    let optimize_regions = optimize_regions && zero_branch.is_none();
    // Long-mode LAHF is part of the v2 floor. Some BMI2 hosts execute PEXT
    // in microcode and keep the existing capture and predicate there.
    let fuse_branch_flags =
        fuse_pairs && target.supports(crate::TargetCpu::X86_64_V2.with_bmi2().with_fast_pdep());
    let native_branch_after = |index: usize| {
        if !fuse_branch_flags
            || !matches!(
                instructions[index].operation,
                Operation::AddSub { flags: true, .. }
                    | Operation::AddSubRegister { flags: true, .. }
            )
        {
            return false;
        }
        let next = match instructions.get(index + 1) {
            Some(next) => Some((next.pc, &next.operation)),
            None => match terminal {
                Some((pc, Terminator::Branch(operation))) => Some((*pc, operation)),
                _ => None,
            },
        };
        matches!(next, Some((pc, Operation::BranchCondition { condition, .. }))
            if *condition < 14 && instructions[index].pc.0.checked_add(4) == Some(pc.0))
    };
    let fuse_simd = !target.data_independent_timing() && (pinned || fuse_pairs);
    let fuse_aes_rounds = fuse_simd && target.supports(crate::TargetCpu::Aes);
    let fuse_vaes_rounds = fuse_simd && target.supports(crate::TargetCpu::Vaes);
    let fuse_ssse3 = fuse_simd && target.supports(crate::TargetCpu::Ssse3);
    let fuse_single_table =
        fuse_ssse3 && !super::asm::vector_extensions::table_supported(1, target);
    // The parser bounds the block, so these per-instruction rows stay local
    // to the emitter. Region admission can still borrow a suffix of them.
    // A self-loop's forward branches inside its body ([`loop_joins`]),
    // where one run covers the body or none runs, and where they land.
    let joins = match self_loop {
        Some(self_loop)
            if self_loop.joins
                && (!(pinned && !code.constant_time() && !checks.translated)
                    || super::pinned::Span::plan_loop(instructions).is_some()) =>
        {
            loop_joins(instructions)
        }
        _ => vec![],
    };
    let mut landing = [false; MAX_BLOCK_INSTRUCTIONS + 1];
    for &(_, at) in &joins {
        landing[at] = true;
    }
    let landing = &landing[..=instructions.len()];
    let mut range_rows = [true; MAX_BLOCK_INSTRUCTIONS];
    let ranges = if checks.guest_range {
        let rows = &mut range_rows[..instructions.len()];
        if !checks.checks_every_access() {
            segmented(super::guest_range::checks, instructions, landing, rows);
        }
        Some(&rows[..])
    } else {
        None
    };
    // Which accesses through SP check its alignment (`stack_alignment`).
    let mut stack_rows = [true; MAX_BLOCK_INSTRUCTIONS];
    let stack = &mut stack_rows[..instructions.len()];
    segmented(super::stack_alignment::checks, instructions, landing, stack);
    // A self-loop's second copy of its body: the checks of a block holding
    // the body three times, the later two copies' ([`SelfLoop`]).
    let mut loop_rows = [true; MAX_BLOCK_INSTRUCTIONS];
    let loop_ranges = match self_loop {
        Some(_) if checks.guest_range => {
            let count = instructions.len();
            if !checks.checks_every_access() {
                let tripled: Vec<Instruction> = instructions
                    .iter()
                    .chain(instructions)
                    .chain(instructions)
                    .cloned()
                    .collect();
                let mut rows = vec![true; tripled.len()];
                let starts: Vec<bool> = (0..tripled.len()).map(|at| landing[at % count]).collect();
                segmented(super::guest_range::checks, &tripled, &starts, &mut rows);
                for (index, row) in loop_rows[..count].iter_mut().enumerate() {
                    *row = rows[count + index] || rows[2 * count + index];
                }
            }
            Some(&loop_rows[..count])
        }
        _ => None,
    };
    // Which copy of a self-loop's body is being emitted.
    let second_copy = core::cell::Cell::new(false);
    let checks_at = |index: usize| super::MemoryChecks {
        guest_range: if second_copy.get() {
            loop_ranges.is_some_and(|ranges| ranges[index])
        } else {
            ranges.is_some_and(|ranges| ranges[index])
        },
        stack_checked: !stack[index],
        ..checks
    };
    // The regions check SP at each access through it, and their cost gates
    // weigh them against instructions that do too.
    let gate_at = |index: usize| super::MemoryChecks {
        stack_checked: false,
        ..checks_at(index)
    };
    let mut cursor = 0;
    let mut next_pinned_span = 0;
    // The run of locals being emitted (`pinned::Span`), where it ends, and
    // the instructions emitted in it so far.
    let mut run: Option<(super::pinned::Run, usize)> = None;
    let mut run_emitted = 0;
    let runs = pinned && !code.constant_time() && !checks.translated;
    // What the last instruction emitted left in registers.
    let mut after = Held::NOTHING;
    let mut out_of_line = OutOfLine::default();
    // Forwarding probes encode one expected MOV at a time. Keep their
    // bytes in one buffer across the block and its temporary candidates.
    let mut scratch = Code::default();
    // Each branch the block runs past: its jump to a side exit, its target
    // and the instructions up to it.
    let mut side_exits = vec![];
    // Accesses that address memory from an earlier access's masked address
    // (`masked_base`), which the instructions emitted alone keep track of.
    let reuse_bases = pinned
        && checks.guest_range
        && checks.every_access_masked
        && !checks.translated
        && !checks.strips();
    let mut bases = super::masked_base::Tracker::default();
    let mut emitted_alone = true;
    // The out-of-line paths of the PAC instructions completed inline.
    let mut pointer_paths = vec![];
    // And those of the mirrored reads (`mirrored_read`).
    let mut mirrored_paths = vec![];
    // And those of the AMX moves (`amx_transfer`).
    let mut amx_paths = vec![];
    // A self-loop's jumps to its exits from both copies of its body, its
    // body twice for the masked bases' lookahead, and the masked bases its
    // back edges carry ([`masked_base::Tracker::carriers`]).
    let mut loop_exits = LoopExits::default();
    let lookahead: Vec<Instruction> = match self_loop {
        Some(_) => instructions.iter().chain(instructions).cloned().collect(),
        None => vec![],
    };
    let mut carried = vec![];
    // Whether one run covers a self-loop's body ([`pinned::Span::plan_loop`]).
    let mut loop_run = false;
    // Guest vectors stay in host registers between the instructions that
    // only compute ([`super::vector_cache`]): in system-mode blocks, whose
    // faults the fault table settles from the state block the cache keeps
    // current at every fault site and exit.
    let vectors = pinned && fault_table && !code.constant_time() && !checks.translated;
    // Pinned code leaves a flag setter's NZCV in the host flags while only
    // its readers follow, the host CF the complement of C where `borrow`
    // (`flag_word::capture`): `pending` holds `borrow` then. It stores the
    // word before any other instruction that may need it, and at the end of
    // the body, so every fault, exit and frame-bound emitter finds it.
    let lazy_flags = pinned && !code.constant_time();
    let mut pending: Option<bool> = None;
    for copy in 0..if self_loop.is_some() { 2_usize } else { 1 } {
        if copy == 0
            && self_loop.is_some()
            && runs
            && let Some(plan) = super::pinned::Span::plan_loop(instructions)
        {
            // One run covers the body: its locals stay in their hosts across
            // the back edges, and the exits store them.
            if plan.takes_slots() {
                bases.reset();
            }
            run = Some((plan.enter_all(code), usize::MAX));
            next_pinned_span = usize::MAX;
            loop_run = true;
        }
        if copy == 1 {
            // The second copy starts where its own back edge lands, with
            // nothing held and only the loop's run open.
            code.forget_vectors();
            second_copy.set(true);
            loop_exits.start = code.len();
            loop_exits.first_rows = block.source_map.len();
            cursor = 0;
            if !loop_run {
                next_pinned_span = 0;
            }
            run_emitted = 0;
            after = Held::NOTHING;
            bases = super::masked_base::Tracker::carried(&carried);
            emitted_alone = true;
        }
        if self_loop.is_some() {
            loop_exits.landings[copy] = vec![None; instructions.len() + 1];
        }
        while cursor < instructions.len() {
            if let Some(borrow) = pending
                && (landing[cursor]
                    || joins.iter().any(|&(at, _)| at == cursor)
                    || !reads_host_flags(&instructions[cursor].operation))
            {
                pending = None;
                if !super::flag_word::dead_from(instructions, cursor) {
                    super::flag_word::capture(code, borrow);
                    after.rax = super::Rax::Nzcv;
                }
            }
            if vectors {
                if landing[cursor] {
                    // A join lands here from a point that stored them all.
                    code.settle_vectors();
                }
                // A fused group addresses the state itself.
                let class_at = |at: usize| {
                    if fuse_simd
                        && super::simd::sha256::HashGroup::recognize(&instructions[at..]).is_some()
                    {
                        VectorUse::Settle
                    } else {
                        vector_use(&instructions[at].operation, target)
                    }
                };
                let class = class_at(cursor);
                // Where the next instruction stores or forgets the homes
                // anyway, results go straight to the state.
                let next_keeps = cursor + 1 < instructions.len()
                    && !landing[cursor + 1]
                    && matches!(class_at(cursor + 1), VectorUse::Keep | VectorUse::Access);
                code.set_vector_write_through(!next_keeps);
                match class {
                    VectorUse::Keep | VectorUse::Access => code.set_vector_cache(true),
                    VectorUse::Flush => {
                        code.set_vector_cache(true);
                        code.flush_vectors();
                    }
                    VectorUse::Settle => {
                        code.settle_vectors();
                        code.set_vector_cache(false);
                    }
                }
            }
            let mut held = core::mem::replace(&mut after, Held::NOTHING);
            if landing[cursor] {
                // A join lands here: nothing held, no base known.
                held = Held::NOTHING;
                bases.reset();
                loop_exits.landings[copy][cursor] = Some(code.len());
            }
            // A group emitted otherwise may have used the slots.
            if !core::mem::take(&mut emitted_alone) {
                bases.reset();
            }
            // A run keeps the guest registers it uses most in scratch registers
            // (`pinned::Span`). A fault in its next instruction recovers the
            // ones it has written from there; its end stores them.
            if let Some((active, end)) = run.as_mut() {
                active.emitted(code, &instructions[run_emitted..cursor]);
                run_emitted = cursor;
                if cursor >= *end
                    && let Some((active, _)) = run.take()
                {
                    active.leave(code);
                }
            }
            if runs && run.is_none() && cursor >= next_pinned_span {
                let (count, plan) = super::pinned::Span::plan(&instructions[cursor..]);
                next_pinned_span = cursor + count.max(1);
                if let Some(plan) = plan {
                    // Its locals overwrite the slots' kept addresses.
                    if plan.takes_slots() {
                        bases.reset();
                    }
                    let held_register = match held.rax {
                        super::Rax::Guest(register) => Some(register),
                        _ => None,
                    };
                    run = Some((plan.enter(code, held_register), cursor + plan.rows()));
                    run_emitted = cursor;
                }
            }
            if fuse_simd
                && let Some(group) =
                    super::simd::sha256::HashGroup::recognize(&instructions[cursor..])
            {
                let (offsets, count, result) = group.emit(code, target);
                block.source_map.extend(
                    instructions[cursor..cursor + count]
                        .iter()
                        .zip(offsets)
                        .map(|(instruction, offset)| (instruction.pc, offset)),
                );
                code.set_instruction(block.source_map.len() - 1);
                after.xmm0 = result;
                cursor += count;
                continue;
            }
            if (fuse_aes_rounds || fuse_vaes_rounds)
                && matches!(
                    instructions[cursor].operation,
                    Operation::Simd(Simd::Aes(_))
                )
            {
                if fuse_vaes_rounds
                    && super::simd::aes::packed_rounds::emit(
                        code,
                        &mut block.source_map,
                        &instructions[cursor..],
                    )
                    .is_some()
                {
                    cursor += 4;
                    continue;
                }
                if fuse_aes_rounds
                    && let Some(result) = super::simd::aes::emit_round_pair(
                        code,
                        &mut block.source_map,
                        &instructions[cursor..],
                        target,
                    )
                {
                    after.xmm0 = Some(result);
                    cursor += 2;
                    continue;
                }
            }
            if fuse_ssse3
                && let Operation::Simd(Simd::Rearrange(operand)) = instructions[cursor].operation
            {
                let group = match operand {
                    Rearrange::Permute { .. } => super::simd::rearrange::emit_reverse_dword_run(
                        code,
                        &mut block.source_map,
                        &instructions[cursor..],
                        target,
                    ),
                    Rearrange::Table {
                        registers: 1,
                        preserve: false,
                        ..
                    } if fuse_single_table => super::simd::rearrange::emit_single_table_run(
                        code,
                        &mut block.source_map,
                        &instructions[cursor..],
                        target,
                    ),
                    _ => None,
                };
                if let Some((count, result)) = group {
                    after.xmm0 = Some(result);
                    cursor += count;
                    continue;
                }
            }
            if fuse_simd
                && let Some(result) = super::simd::precision::emit_half_widen_pair(
                    code,
                    &mut block.source_map,
                    &instructions[cursor..],
                )
            {
                after.xmm0 = Some(result);
                cursor += 2;
                continue;
            }
            if pinned
                && !code.constant_time()
                && matches!(
                    instructions[cursor].operation,
                    Operation::AddSubCarry { flags: true, .. }
                )
                && let Some(count) = super::pinned::emit_carry_run(
                    code,
                    &mut block.source_map,
                    &instructions[cursor..],
                    held.rax == super::Rax::Nzcv,
                )
            {
                code.set_instruction(block.source_map.len() - 1);
                after.rax = super::Rax::Nzcv;
                cursor += count;
                continue;
            }
            // The pair's second fault finds the first's result, which a run's
            // dirty locals at the pair's start do not hold.
            if (pinned || fuse_pairs)
                && run.is_none()
                && !code.constant_time()
                && !checks.checks_every_access()
                && matches!(
                    instructions[cursor].operation,
                    Operation::MemoryRegisterOffset(_)
                )
                && super::register_offset::emit_load_store(
                    code,
                    &mut block.source_map,
                    &instructions[cursor..],
                    checks_at(cursor),
                    held.rax,
                )
            {
                cursor += 2;
                continue;
            }
            if fuse_pairs {
                let group = match instructions[cursor].operation {
                    Operation::Memory {
                        transfer: super::MemoryTransfer::Load { .. },
                        mode: super::AddressMode::Offset | super::AddressMode::PostIndex,
                        ..
                    } => super::memory_recipe::CompareLoads::new(
                        &instructions[cursor..],
                        held.rax,
                        checks,
                    ),
                    Operation::MemoryPair {
                        transfer: super::MemoryTransfer::Load { .. },
                        mode: super::AddressMode::Offset,
                        ..
                    } => super::memory_recipe::CompareLoads::from_pair(
                        &instructions[cursor..],
                        checks,
                    ),
                    _ => None,
                };
                if let Some(group) = group {
                    let compare = cursor + group.compare_index();
                    after.rax = group.emit(
                        code,
                        &mut block.source_map,
                        &instructions[cursor..],
                        checks,
                        native_branch_after(compare),
                    );
                    cursor = compare + 1;
                    continue;
                }
            }
            if (pinned || fuse_pairs)
                && !code.constant_time()
                && matches!(instructions[cursor].operation, Operation::IntegerMath(operand)
                if matches!(operand.kind, integer_math::Kind::Divide { .. }))
                && super::integer_math::emit_divide_remainder(
                    code,
                    &mut block.source_map,
                    &instructions[cursor..],
                )
            {
                cursor += 2;
                continue;
            }
            // Build the new candidate privately. Rejection leaves both cursor and
            // output untouched, so the legacy clean-load chunk partition is exact.
            if optimize_regions
                && let Ok(Some(region)) =
                    super::region::emit_allocated_prefix(&instructions[cursor..])
            {
                let count = region.consumed_instructions;
                let mut baseline = Code::new();
                let mut candidate_fixups = vec![];
                for (index, instruction) in instructions[cursor..cursor + count].iter().enumerate()
                {
                    super::emit_instruction(
                        &mut baseline,
                        instruction,
                        &mut candidate_fixups,
                        target,
                        gate_at(cursor + index),
                    );
                }
                if candidate_fixups.is_empty()
                    && super::region::admits_allocated(&region, &baseline)
                {
                    let offset = code.len();
                    block
                        .source_map
                        .extend(region.source_map.iter().map(|(pc, at)| (*pc, offset + at)));
                    code.append(region.code);
                    // The value allocator emits only GPR operations and saves.
                    after.xmm0 = held.xmm0;
                    after.xmm_pair = held.xmm_pair;
                    cursor += count;
                    continue;
                }
            }
            if optimize_regions
                && let Ok(region) = super::region::emit_clean_load_prefix_with(
                    &instructions[cursor..],
                    checks,
                    fault_table,
                    ranges.map(|ranges| &ranges[cursor..]),
                )
                && region.consumed_instructions != 0
            {
                let count = region.consumed_instructions;
                let chunk = &instructions[cursor..cursor + count];
                let mut baseline = |checks_at: &dyn Fn(usize) -> super::MemoryChecks| {
                    let mut code = Code::new();
                    let mut map = vec![];
                    let mut fixups = vec![];
                    let mut after = held;
                    for (index, instruction) in chunk.iter().enumerate() {
                        map.push((instruction.pc, code.len()));
                        after = emit_single(
                            &mut code,
                            &mut scratch,
                            None,
                            instruction,
                            &mut fixups,
                            SingleOptions {
                                target,
                                checks: checks_at(cursor + index),
                                live_nzcv: native_branch_after(cursor + index),
                                defer_flags: false,
                            },
                            after,
                            &mut None,
                        );
                    }
                    debug_assert!(fixups.is_empty());
                    (code, map, after)
                };
                let offset = code.len();
                let gate_baseline = baseline(&gate_at);
                if super::region::admits_ordinary(&region, &gate_baseline.0) {
                    block
                        .source_map
                        .extend(region.source_map.iter().map(|(pc, at)| (*pc, offset + at)));
                    fault_recovery.extend(region.fault_recovery.into_iter().map(|entry| {
                        crate::block::FaultRecovery {
                            start: offset + entry.start,
                            end: offset + entry.end,
                            retirement: entry
                                .retirement
                                .rebased(u16::try_from(cursor).expect("bounded block cursor"))
                                .expect("bounded block retirement"),
                            ..entry
                        }
                    }));
                    code.append(region.code);
                    if (held.xmm0.is_some() || held.xmm_pair.is_some())
                        && chunk
                            .iter()
                            .all(|instruction| preserves_held_vectors(&instruction.operation))
                    {
                        after.xmm0 = held.xmm0;
                        after.xmm_pair = held.xmm_pair;
                    }
                } else {
                    // Reject the complete eligible chunk. Rescanning its suffix
                    // could bypass the measured cost gate with a different shape.
                    let same_stack_checks = chunk.iter().enumerate().all(|(index, instruction)| {
                        match instruction.operation {
                            Operation::Memory { base, .. } | Operation::MemoryPair { base, .. }
                                if base == Register::Sp =>
                            {
                                stack[cursor + index]
                            }
                            _ => true,
                        }
                    });
                    let (baseline_code, baseline_map, baseline_after) = if same_stack_checks {
                        gate_baseline
                    } else {
                        baseline(&checks_at)
                    };
                    block
                        .source_map
                        .extend(baseline_map.iter().map(|(pc, at)| (*pc, offset + at)));
                    code.append(baseline_code);
                    after = baseline_after;
                }
                // Each region wrote back the full register frame, so what follows
                // sees the ordinary direct-RAM ABI.
                cursor += count;
                continue;
            }
            let instruction = &instructions[cursor];
            // System chains only enter and poll at block boundaries. Neither
            // operation has a guest fault site, and an independent entry at ADD
            // still compiles from its own source. Preserve both retirement rows.
            if pinned
                && !code.constant_time()
                && let Some(second) = instructions.get(cursor + 1)
                && instruction.pc.0.checked_add(4) == Some(second.pc.0)
                && let Some(address) = fold_address_pair(&instruction.operation, &second.operation)
            {
                block.source_map.push((instruction.pc, code.len()));
                code.set_instruction(block.source_map.len() - 1);
                code.asm().nop();
                block.source_map.push((second.pc, code.len()));
                code.set_instruction(block.source_map.len() - 1);
                super::pinned::emit_pc_relative(code, address);
                if code.host(address.destination).is_none() {
                    after.rax = super::Rax::Guest(address.destination);
                }
                bases.after(instruction, None);
                bases.after(second, None);
                emitted_alone = true;
                cursor += 2;
                continue;
            }
            // An ordinary block runs the pair together. A truncated block or
            // an entry at CLZ still emits its original operation.
            if fuse_pairs
                && let Operation::IntegerMath(reverse) = instruction.operation
                && let integer_math::Kind::ReverseBits { width } = reverse.kind
                && let Some(second) = instructions.get(cursor + 1)
                && let Operation::IntegerMath(count) = second.operation
                && count.kind == (integer_math::Kind::LeadingZeros { width })
                && reverse.destination != Register::Zero
                && count.left == reverse.destination
                && count.destination == reverse.destination
                && instruction.pc.0.checked_add(4) == Some(second.pc.0)
            {
                block.source_map.push((instruction.pc, code.len()));
                code.set_instruction(block.source_map.len() - 1);
                code.asm().nop();
                block.source_map.push((second.pc, code.len()));
                code.set_instruction(block.source_map.len() - 1);
                super::integer_unary::emit(
                    code,
                    &integer_unary::Operand {
                        kind: integer_unary::Kind::TrailingZeros,
                        width,
                        destination: count.destination,
                        source: reverse.left,
                    },
                    target,
                );
                cursor += 2;
                continue;
            }
            block.source_map.push((instruction.pc, code.len()));
            code.set_instruction(block.source_map.len() - 1);
            if let Some(target) = instruction.operation.conditional_target() {
                let taken = match (&instruction.operation, pending.as_mut()) {
                    (Operation::BranchCondition { condition, .. }, Some(borrow)) => {
                        super::flag_word::host_condition(code, *condition, borrow)
                    }
                    _ => branch_condition(code, &instruction.operation, held.rax)
                        .expect("a conditional branch's condition"),
                };
                // A side exit's stub is its link slot and exit alone, which is
                // where `successor_exits` finds it: what the run has written is
                // stored before the branch, flags untouched.
                let mut dirty = super::pinned::Dirty::default();
                if let Some((active, _)) = run.as_mut() {
                    if loop_run {
                        // The exit stores what the run has written.
                        dirty = active.dirty();
                    } else {
                        active.flush(code);
                    }
                }
                let jump = match pending {
                    // The exit stores the word the fallthrough keeps in the
                    // host flags.
                    Some(borrow) => {
                        let stay = code.asm().jcc(taken.negate());
                        super::flag_word::capture(code, borrow);
                        let jump = code.asm().jmp();
                        code.asm().bind(stay);
                        jump
                    }
                    None => code.asm().jcc(taken),
                };
                if self_loop.is_some() {
                    // Both copies' jumps go to one exit, which settles first.
                    let rows = block.source_map.len() - loop_exits.first_rows * copy;
                    if let Some(&(at, to)) = joins.iter().find(|&&(at, _)| at == cursor) {
                        let skipped = u8::try_from(to - at - 1).expect("bounded body");
                        loop_exits.joins.push((jump, copy, to, skipped));
                        loop_exits.side.push((None, target, rows, dirty));
                    } else {
                        loop_exits.side.push((Some(jump), target, rows, dirty));
                    }
                } else {
                    side_exits.push((jump, target, block.source_map.len()));
                }
                bases.after(instruction, None);
                emitted_alone = true;
                cursor += 1;
                continue;
            }
            if let Operation::PointerAuthentication(operation) = instruction.operation {
                let cells = native
                    .pointer
                    .expect("blocks hold PAC instructions only given the service's cells");
                pointer_paths.push(super::pointer::inline::emit(
                    code,
                    operation,
                    instruction.pc,
                    cells,
                ));
                // A service call may change both masked-base slots.
                bases.reset();
                emitted_alone = true;
                cursor += 1;
                continue;
            }
            if let Operation::MirroredRead { word } = instruction.operation {
                let cell = native
                    .mirrored
                    .cell(word)
                    .expect("blocks hold mirrored reads only given their cells");
                mirrored_paths.push(super::mirrored_read::emit(code, word, instruction.pc, cell));
                // Xt alone changes; its masked base, if any, is gone.
                bases.after(instruction, None);
                emitted_alone = true;
                cursor += 1;
                continue;
            }
            if let Operation::ZeroBlock { rt } = instruction.operation {
                let bytes = native
                    .zero_block
                    .expect("blocks hold DC ZVA only given its block size");
                code.set_base_request(None);
                super::zero_block::emit(code, rt, bytes, checks_at(cursor));
                // RAX, RCX, RDX and the flags change; no guest register does.
                after = Held::NOTHING;
                bases.after(instruction, None);
                emitted_alone = true;
                cursor += 1;
                continue;
            }
            if let Operation::Amx(amx) = instruction.operation {
                amx_paths.push(super::amx_transfer::emit(code, &amx, checks, target));
                bases.reset();
                emitted_alone = true;
                cursor += 1;
                continue;
            }
            if reuse_bases {
                // A self-loop's accesses look ahead into the next iteration.
                let ahead = if self_loop.is_some() {
                    &lookahead[cursor..]
                } else {
                    &instructions[cursor..]
                };
                code.set_base_request(bases.plan(ahead));
            }
            let frame_bound = pinned && super::pinned::frame_bound(&instruction.operation);
            if frame_bound {
                code.set_pinned(false);
                super::pinned::write_back(code);
            }
            after = emit_single(
                code,
                &mut scratch,
                Some(&mut out_of_line),
                instruction,
                &mut fixups,
                SingleOptions {
                    target,
                    checks: checks_at(cursor),
                    live_nzcv: native_branch_after(cursor),
                    defer_flags: lazy_flags && !frame_bound,
                },
                held,
                &mut pending,
            );
            if frame_bound {
                super::pinned::reload(code);
                code.set_pinned(true);
            } else if after.rax == super::Rax::Nzcv
                && super::flag_word::dead_after(instructions, cursor)
                && let Some(start) = block.source_map.last().map(|&(_, start)| start)
                && let Some(tail) = super::flag_word::captured_tail(code, start)
            {
                // The next flag setter defines NZCV again before anything reads
                // it or the frame must hold it: this capture is dead.
                super::flag_word::drop_capture(code, tail);
                after.rax = super::Rax::Unknown;
            }
            let applied = code.finish_base_request();
            bases.after(instruction, applied);
            emitted_alone = true;
            cursor += 1;
        }
        // The back edge, the terminal and their exits read the state.
        code.flush_vectors();
        if loop_run {
            if let Some((active, _)) = run.as_mut() {
                active.emitted(code, &instructions[run_emitted..]);
                run_emitted = instructions.len();
                loop_exits.dirty = active.dirty();
            }
        } else if let Some((mut active, _)) = run.take() {
            active.emitted(code, &instructions[run_emitted..]);
            active.leave(code);
        }
        // The block's exits and the back edge's count need the word.
        if let Some(borrow) = pending.take() {
            super::flag_word::capture(code, borrow);
            after.rax = super::Rax::Nzcv;
        }
        if let (Some(self_loop), Some((pc, Terminator::Branch(operation)))) = (self_loop, terminal)
        {
            // The back edge: the iteration's instructions count, and the second
            // copy runs again until an entry's share is spent.
            if landing[instructions.len()] {
                code.forget_vectors();
                after = Held::NOTHING;
                bases.reset();
                loop_exits.landings[copy][instructions.len()] = Some(code.len());
            }
            block.source_map.push((*pc, code.len()));
            code.set_instruction(block.source_map.len() - 1);
            let flags_live =
                after.rax == super::Rax::Nzcv && code.ends_with(&nzcv_store(code.is_pinned()));
            if !matches!(operation, Operation::Branch { .. })
                && let Some(taken) = branch_condition_with(code, operation, after.rax, flags_live)
            {
                loop_exits.fallthrough.push(code.asm().jcc(taken.negate()));
            }
            // Each carried slot moves with its register, which then is its
            // kept address again at the head; LEA leaves the flags alone.
            // An entry's iterations move it at most 2^30 from its check
            // (`masked_base`), or it is not carried. An access through it is
            // then within 2^31 of the checked address: within the 2^35 that
            // `guest_range` shows safe, and non-canonical from
            // `GuestState::UNREACHABLE` after a mispredicted check.
            if copy == 0 {
                // The shortest iteration skips every join's instructions.
                let skipped: usize = joins.iter().map(|&(at, to)| to - at - 1).sum();
                let shortest = (usize::from(self_loop.instructions) - skipped).max(1);
                let iterations = i64::from(SelfLoop::LIMIT) / shortest as i64 + 1;
                carried = if reuse_bases && emitted_alone {
                    bases.carriers()
                } else {
                    vec![]
                };
                carried
                    .retain(|&(_, _, offset)| offset.abs().saturating_mul(iterations) <= 1 << 30);
            } else if !emitted_alone
                || carried
                    .iter()
                    .any(|&(register, host, offset)| bases.offset(register, host) != Some(offset))
            {
                // The second copy moved its bases otherwise: emit the block
                // without the loop.
                return emit_selected(
                    instructions,
                    terminal,
                    fallthrough,
                    target,
                    RegionPolicy {
                        self_loop: None,
                        ..regions
                    },
                    checks,
                    native,
                );
            }
            for &(_, host, offset) in &carried {
                if offset != 0 {
                    code.asm().lea(
                        Size::Qword,
                        host,
                        Mem::base(host, i32::try_from(offset).expect("bounded offset")),
                    );
                }
            }
            let mut asm = code.asm();
            asm.alu_imm8(
                Alu::Add,
                Size::Qword,
                super::frame(LOOP_RETIRED),
                self_loop.instructions as i8,
            );
            if copy == 1 {
                // Again only while the next iteration keeps the entry within
                // the limit: a native run then retires no more per transfer
                // than with blocks, which the engine's bounds assume.
                asm.alu_imm32(
                    Alu::Cmp,
                    Size::Qword,
                    super::frame(LOOP_RETIRED),
                    SelfLoop::LIMIT - i32::from(self_loop.instructions) + 1,
                );
                let again = asm.jcc(Cond::B);
                asm.bind_back(again, loop_exits.start);
                // The share is spent: the taken branch's own exit, which counts
                // this iteration as a block's would.
                super::pinned::write_back_locals(code, loop_exits.dirty);
                let mut asm = code.asm();
                settle_loop(&mut asm, self_loop.instructions, native.loop_counts);
                loop_exits.taken = Some(asm.jmp());
            }
            after = Held::NOTHING;
        }
    }
    if let Some((active, _)) = run.take() {
        active.forget(code);
    }
    code.set_vector_cache(false);
    if let Some(self_loop) = self_loop {
        // A group emitted across a landing leaves it without a start: the
        // loop then takes those branches' side exits.
        if loop_exits
            .joins
            .iter()
            .any(|&(_, copy, to, _)| loop_exits.landings[copy][to].is_none())
        {
            return emit_selected(
                instructions,
                terminal,
                fallthrough,
                target,
                RegionPolicy {
                    self_loop: Some(SelfLoop {
                        joins: false,
                        ..self_loop
                    }),
                    ..regions
                },
                checks,
                native,
            );
        }
        // Each join takes the instructions it skips off the loop's count
        // and lands in its copy; the flags are not held there.
        for (jump, copy, to, skipped) in core::mem::take(&mut loop_exits.joins) {
            let mut asm = code.asm();
            asm.bind(jump);
            asm.alu_imm8(
                Alu::Sub,
                Size::Qword,
                super::frame(LOOP_RETIRED),
                skipped as i8,
            );
            let land = asm.jmp();
            asm.bind_back(land, loop_exits.landings[copy][to].expect("a landing"));
        }
    }
    debug_assert!(
        fixups.is_empty(),
        "ordinary instruction prefix cannot branch"
    );
    // A capture the body ends with leaves the host flags live for the
    // terminal's branch: only the jumps over the moved code and the paths
    // below come between, and moved code returns into the body.
    let flags_live = after.rax == super::Rax::Nzcv && code.ends_with(&nzcv_store(code.is_pinned()));
    out_of_line.place(code);
    // Before the terminal, whose direct exits end the code.
    if !pointer_paths.is_empty() || !mirrored_paths.is_empty() || !amx_paths.is_empty() {
        let over = code.asm().jmp();
        if let Some(cells) = native.pointer {
            for path in pointer_paths {
                super::pointer::inline::place(code, path, cells);
            }
        }
        for path in mirrored_paths {
            super::mirrored_read::place(code, path);
        }
        for path in amx_paths {
            super::amx_transfer::place(code, path);
        }
        code.asm().bind(over);
    }
    if let Some(fast) = prefix_sum {
        code.asm().bind(fast);
        // All fast-path state is settled. The scalar source rows describe
        // its fallback accesses, while the single vector fault is at row zero.
        block.source_map[0].1 = 0;
        after = Held::NOTHING;
    }
    // A self-loop's second copy's source rows, which its faults' rows name
    // as the first copy's.
    let mut second_rows = vec![];
    if let (Some(_), Some((_, Terminator::Branch(operation)))) = (self_loop, terminal) {
        second_rows = block.source_map.split_off(loop_exits.first_rows);
        let (entry, conditional) = match *operation {
            Operation::Branch { target } => (target, false),
            _ => (
                operation
                    .conditional_target()
                    .expect("a self-loop ends with a branch back"),
                true,
            ),
        };
        // An unconditional loop has no fallthrough exit, as its block has
        // none (`terminal_exits`).
        let to_fallthrough = conditional.then(|| {
            for jump in loop_exits.fallthrough.drain(..) {
                code.asm().bind(jump);
            }
            super::pinned::write_back_locals(code, loop_exits.dirty);
            let mut asm = code.asm();
            settle_loop(&mut asm, 0, native.loop_counts);
            asm.jmp()
        });
        // One exit per branch run past, in their order, for both copies:
        // each jump stores what its copy's run has written there.
        let mut rows: Vec<usize> = loop_exits
            .side
            .iter()
            .map(|&(_, _, rows, _)| rows)
            .collect();
        rows.dedup();
        let mut seen = vec![];
        for row in rows {
            if seen.contains(&row) {
                continue;
            }
            seen.push(row);
            let mut target = None;
            let mut stored = vec![];
            for &(jump, to, at, dirty) in &loop_exits.side {
                if at == row {
                    target = Some(to);
                    // A join's exit stays, with no jump to it.
                    let Some(jump) = jump else { continue };
                    code.asm().bind(jump);
                    super::pinned::write_back_locals(code, dirty);
                    stored.push(code.asm().jmp());
                }
            }
            let mut asm = code.asm();
            for jump in stored {
                asm.bind(jump);
            }
            settle_loop(&mut asm, 0, native.loop_counts);
            side_exits.push((asm.jmp(), target.expect("a branch's jumps"), row));
        }
        let mut asm = code.asm();
        if let Some(to_fallthrough) = to_fallthrough {
            asm.bind(to_fallthrough);
            direct_exit_with(&mut asm, fallthrough, ExitKind::Fallthrough);
        }
        asm.bind(
            loop_exits
                .taken
                .take()
                .expect("the second copy's back edge"),
        );
        direct_exit_with(&mut asm, entry, ExitKind::Direct);
    } else if let Some(frame_loop) = frame_loop {
        let Some((pc, Terminator::Branch(operation))) = terminal else {
            unreachable!("a frame loop has a validated direct branch")
        };
        debug_assert!(side_exits.is_empty());
        block.source_map[0].1 = 0;
        block.source_map.push((*pc, code.len()));
        code.set_instruction(block.source_map.len() - 1);
        let taken = branch_condition(code, operation, after.rax);
        frame_loop.finish(code, taken, fallthrough);
    } else {
        emit_terminal(
            code,
            &mut block.source_map,
            terminal,
            TerminalContext {
                fallthrough,
                native,
                checks,
                rax: after.rax,
                flags_live,
                target,
                zero_branch,
            },
        );
    }
    // After the terminal, whose direct exits end the code before them.
    let instructions_run = block.source_map.len();
    debug_assert_eq!(instructions_run, source_rows);
    for (jump, target, run) in side_exits {
        let mut asm = code.asm();
        asm.bind(jump);
        direct_exit_with(&mut asm, target, side_exit(instructions_run - run));
    }
    code.place_failures();
    if fault_table && !second_rows.is_empty() {
        // Both copies' rows, against the body as often; a second-copy row
        // retires as many instructions of its iteration as the first's.
        let first = block.source_map.len();
        let terminal = terminal.map(|(pc, terminal)| match terminal {
            Terminator::Branch(operation) => Instruction {
                pc: *pc,
                operation: operation.clone(),
            },
            _ => unreachable!("a self-loop ends with a branch"),
        });
        let source: Vec<Instruction> = instructions
            .iter()
            .cloned()
            .chain(terminal)
            .chain(instructions.iter().cloned())
            .collect();
        let map: Vec<_> = block
            .source_map
            .iter()
            .chain(&second_rows)
            .cloned()
            .collect();
        fault_recovery = recovery::bind(&block.x86_64_code, &map, &source, fault_recovery)
            .into_iter()
            .map(|mut row| {
                let retired = usize::from(row.retirement.retired_before());
                if retired >= first {
                    row.retirement = crate::block::RetirementPoint::before(
                        row.retirement.instruction_pc(),
                        u16::try_from(retired - first).expect("bounded block"),
                    )
                    .expect("a first-copy position");
                }
                row
            })
            .collect();
    } else if fault_table {
        fault_recovery = recovery::bind(
            &block.x86_64_code,
            &block.source_map,
            instructions,
            fault_recovery,
        );
    }
    (block.into_native(), fault_recovery, self_loop.is_some())
}

/// The zero test can consume the adjacent frame ALU result's host flags.
fn alu_zero_branch(
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
) -> Option<Cond> {
    let [instruction] = instructions else {
        return None;
    };
    let Operation::AddSub {
        width,
        flags: false,
        destination: destination @ Register::X(_),
        source,
        ..
    } = instruction.operation
    else {
        return None;
    };
    let (
        branch_pc,
        Terminator::Branch(Operation::BranchZero {
            width: branch_width,
            source: tested,
            nonzero,
            ..
        }),
    ) = terminal?
    else {
        return None;
    };
    // A W write zero-extends, so its ZF also describes the full X value.
    (destination == source
        && destination == *tested
        && (width == *branch_width || width == crate::aarch64::Width::W32)
        && instruction.pc.0.checked_add(4) == Some(branch_pc.0))
    .then_some(if *nonzero { Cond::Ne } else { Cond::E })
}

/// What registers hold where an instruction outside the regions begins,
/// left there by the previous one.
#[derive(Clone, Copy)]
struct Held {
    rax: super::Rax,
    /// The guest vector XMM0 holds whole.
    xmm0: Option<u8>,
    /// The guest vectors XMM0 and XMM1 hold whole.
    xmm_pair: Option<[u8; 2]>,
}

impl Held {
    const NOTHING: Self = Self {
        rax: super::Rax::Unknown,
        xmm0: None,
        xmm_pair: None,
    };
}

/// What an instruction asks of the guest vectors kept in host registers
/// ([`super::vector_cache`]).
#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub(super) enum VectorUse {
    /// It neither faults nor leaves, reaches V only through the cache's
    /// operand layer and writes no pool register: the homes stay.
    Keep,
    /// As [`Self::Keep`], but it can fault, at exact sites only, whose
    /// recovery reads the dirty homes from the saved XMM8-15
    /// ([`crate::block::RecoveryBinding::vector`]): the homes stay.
    Access,
    /// As [`Self::Keep`], but it can fault or leave: the dirty homes are
    /// stored first, and stay as clean copies.
    Flush,
    /// Anything else: every home is stored and forgotten, and the
    /// instruction is emitted without the cache.
    Settle,
}

fn vector_use(operation: &Operation, target: crate::TargetCpu) -> VectorUse {
    match operation {
        Operation::AddSub { .. }
        | Operation::AddSubRegister { .. }
        | Operation::AddSubExtended { .. }
        | Operation::AddSubCarry { .. }
        | Operation::LogicalImmediate { .. }
        | Operation::LogicalRegister { .. }
        | Operation::MoveWide { .. }
        | Operation::Bitfield(_)
        | Operation::PcRelative(_)
        | Operation::ConditionalSelect(_)
        | Operation::ConditionalCompare(_)
        | Operation::IntegerMath(_)
        | Operation::IntegerMinMax(_)
        | Operation::IntegerUnary(_)
        | Operation::Nop => VectorUse::Keep,
        Operation::Memory { .. }
        | Operation::MemoryPair { .. }
        | Operation::MemoryRegisterOffset(_) => VectorUse::Access,
        Operation::BranchCondition { .. }
        | Operation::BranchZero { .. }
        | Operation::BranchBit { .. } => VectorUse::Flush,
        Operation::Simd(operand) => super::simd::vector_use(operand, target),
        _ => VectorUse::Settle,
    }
}

/// These recipes only touch GPRs, flags and their frame slots. A memory
/// access can call a helper, and integer math can use vector scratch.
fn preserves_held_vectors(operation: &Operation) -> bool {
    matches!(
        operation,
        Operation::AddSub { .. }
            | Operation::AddSubRegister { .. }
            | Operation::AddSubExtended { .. }
            | Operation::LogicalImmediate { .. }
            | Operation::LogicalRegister { .. }
            | Operation::MoveWide { .. }
            | Operation::Bitfield(_)
            | Operation::PcRelative(_)
            | Operation::Nop
    )
}

struct SingleOptions {
    target: crate::TargetCpu,
    checks: super::MemoryChecks,
    live_nzcv: bool,
    /// Leave a flag setter's NZCV in the host flags ([`sets_flags_alone`]).
    defer_flags: bool,
}

/// Whether `operation` sets all of NZCV from its own result, reading none
/// of it, in an emission whose last flag-setting host instruction leaves
/// them: one pinned code may leave in the host flags.
fn sets_flags_alone(operation: &Operation) -> bool {
    matches!(
        operation,
        Operation::AddSub { flags: true, .. }
            | Operation::AddSubRegister { flags: true, .. }
            | Operation::AddSubExtended { flags: true, .. }
            | Operation::LogicalImmediate { flags: true, .. }
            | Operation::LogicalRegister { flags: true, .. }
    )
}

/// Whether `operation` takes NZCV from the host flags of a setter before
/// it ([`sets_flags_alone`]) and leaves them: a conditional branch, or a
/// select that does ([`super::conditional_select::keeps_host_flags`]).
fn reads_host_flags(operation: &Operation) -> bool {
    match operation {
        Operation::BranchCondition { condition, .. } => *condition < 14,
        Operation::ConditionalSelect(operand) => {
            super::conditional_select::keeps_host_flags(operand)
        }
        _ => false,
    }
}

/// One instruction outside the regions, after an emission that left
/// `held`; returns what it leaves. An instruction that first loads the
/// register the previous one just stored from RAX finds it there, and one
/// that reads a vector XMM0 holds may take it from there. With
/// `out_of_line`, numerical instructions move their slow paths there.
/// `flags` holds the borrow of NZCV that only the host flags hold, before
/// the instruction and after it.
#[allow(clippy::too_many_arguments)]
fn emit_single(
    code: &mut Code,
    scratch: &mut Code,
    out_of_line: Option<&mut OutOfLine>,
    instruction: &Instruction,
    fixups: &mut Vec<(GuestAddress, GuestAddress, usize)>,
    options: SingleOptions,
    held: Held,
    flags: &mut Option<bool>,
) -> Held {
    let SingleOptions {
        target,
        checks,
        live_nzcv,
        defer_flags,
    } = options;
    if let Operation::FloatingPoint(operand) = &instruction.operation {
        let Some(out_of_line) = out_of_line else {
            super::simd::emit_numerical(code, operand, instruction.pc, target);
            return Held::NOTHING;
        };
        let start = code.len();
        let entry = super::simd::numerical::Entry {
            checked: !out_of_line.continues_at(start),
            xmm0: held.xmm0,
        };
        let split = super::simd::emit_numerical_split(code, operand, instruction.pc, target, entry);
        let xmm0 = split.as_ref().map(|split| split.result);
        out_of_line.take(code, start, split);
        return Held {
            xmm0,
            ..Held::NOTHING
        };
    }
    let capture: fn(&mut Code, bool) = if live_nzcv {
        super::flag_word::capture_arm_live
    } else {
        super::nzcv
    };
    let start = code.len();
    let deferred = defer_flags && sets_flags_alone(&instruction.operation);
    if deferred {
        code.defer_nzcv();
    }
    if let Some(borrow) = flags.as_mut()
        && let Operation::ConditionalSelect(operand) = &instruction.operation
    {
        super::conditional_select::emit_host(code, operand, borrow);
    } else if let Operation::Simd(operand) = &instruction.operation {
        super::simd::emit_after(code, operand, target, checks, held.xmm0, held.xmm_pair);
        // A first load of the vector XMM0 holds changes nothing.
        if let Some(vector) = held.xmm0 {
            let load = match code.vector_cache().peek(vector) {
                Some(host) => super::simd::xmm0_home_load(scratch, host),
                None => super::simd::xmm0_load(scratch, vector),
            };
            if code[start..].starts_with(load) && code.len() > start + load.len() {
                code.remove_instruction(start..start + load.len());
            }
        }
    } else if code.is_pinned()
        && !code.constant_time()
        && held.rax == super::Rax::Nzcv
        && matches!(instruction.operation, Operation::AddSubCarry { .. })
    {
        super::pinned::emit_live_carry(code, &instruction.operation);
    } else if let Operation::ConditionalSelect(operand) = &instruction.operation
        && code.is_pinned()
        && !code.constant_time()
        && held.rax == super::Rax::Nzcv
        && matches!(
            operand.otherwise,
            crate::aarch64::conditional_select::Otherwise::Unchanged
                | crate::aarch64::conditional_select::Otherwise::Increment
                | crate::aarch64::conditional_select::Otherwise::Invert
        )
    {
        // The preceding pinned capture preserves the host flags. The
        // select keeps them live while preparing its operands.
        super::conditional_select::emit(code, operand, true);
    } else if let Operation::ConditionalCompare(operand) = &instruction.operation
        && code.is_pinned()
        && !code.constant_time()
        && held.rax == super::Rax::Nzcv
    {
        super::conditional_select::emit_compare(code, operand, true);
    } else if let super::Rax::Guest(register) = held.rax
        && let Operation::AddSub {
            width,
            subtract: true,
            flags: true,
            destination: Register::Zero,
            source,
            immediate,
        } = instruction.operation
        && source == register
    {
        super::pinned::compare_held_immediate(code, width, immediate);
        capture(code, true);
    } else if !code.constant_time()
        && let super::Rax::Guest(register) = held.rax
        && let Operation::AddSubRegister {
            width,
            subtract: true,
            flags: true,
            destination: Register::Zero,
            left: left @ Register::X(_),
            right,
            amount: 0,
            ..
        } = instruction.operation
        && right == register
        && left != right
    {
        super::pinned::compare_held_right(code, width, left);
        capture(code, true);
    } else if !code.is_pinned()
        && !code.constant_time()
        && let super::Rax::Guest(register) = held.rax
        && let Operation::AddSubRegister {
            width,
            subtract: false,
            flags: false,
            destination,
            left: left @ Register::X(_),
            right,
            shift,
            amount,
        } = instruction.operation
        && right == register
        && left != right
        && destination != Register::Zero
        && (amount == 0 || (shift == Shift::Lsl && amount <= 3))
    {
        let mut asm = code.asm();
        let left = asm.guest(left).expect("ADD left register");
        let size = Size::of(width == Width::X64);
        if amount == 0 {
            // ADD is commutative with an unshifted right operand.
            asm.alu_load(Alu::Add, size, Gpr::Rax, left);
        } else {
            // The SIB scale uses the settled right operand in RAX. LEA
            // truncates a W result to 32 bits and clears its upper half.
            asm.mov_load(size, Gpr::Rcx, left);
            asm.lea(
                size,
                Gpr::Rax,
                Mem::indexed(Gpr::Rcx, Gpr::Rax, 1 << amount, 0),
            );
        }
        super::store(code, destination);
    } else if let Operation::AddSubExtended {
        width,
        subtract: true,
        flags: true,
        destination: Register::Zero,
        left,
        right,
        extend,
        amount,
    } = instruction.operation
        && !code.constant_time()
        && held.rax != super::Rax::Guest(left)
        && let Some(left) = code.asm().guest(left)
    {
        super::compare_extended(code, width, left, right, extend, amount);
    } else if let super::Rax::PairStoreAddress { base, displacement } = held.rax
        && let Operation::AddSub {
            width: Width::X64,
            subtract: false,
            flags: false,
            destination,
            source,
            immediate,
        } = instruction.operation
        && source == base
        && destination == base
    {
        // The paired store and both of its fault sites precede this row.
        // Undo its displacement while applying the guest's base advance.
        let adjustment = i32::try_from(i64::from(immediate) - i64::from(displacement))
            .expect("decoded Q pair displacement and ADD immediate fit i32");
        super::address_add(code, adjustment, super::AddressMode::Offset);
        super::store(code, destination);
    } else if let Operation::MemoryRegisterOffset(operand) = &instruction.operation {
        super::register_offset::emit_after(code, operand, checks, held.rax);
    } else if live_nzcv {
        assert!(
            super::pinned::emit_direct_with_add_sub_capture(
                code,
                &instruction.operation,
                target,
                capture
            ),
            "the native branch plan selected ADD/SUB"
        );
    } else {
        super::emit_instruction(code, instruction, fixups, target, checks);
    }
    // The instruction keeps a non-empty native range for its source map.
    if let super::Rax::Guest(register) = held.rax
        && let Some(load) = super::register_load(scratch, register, code.is_pinned())
        && code[start..].starts_with(load)
        && code.len() > start + load.len()
    {
        code.remove_instruction(start..start + load.len());
    }
    if deferred {
        *flags = Some(
            code.take_deferred_nzcv()
                .expect("a flag setter's emission captures NZCV"),
        );
    } else {
        debug_assert!(flags.is_none() || reads_host_flags(&instruction.operation));
    }
    let rax = match super::rax_after(&instruction.operation, code, checks) {
        super::Rax::Nzcv if deferred => super::Rax::Unknown,
        // A direct form may leave the result only in its host register.
        super::Rax::Guest(register)
            if code.is_pinned()
                && !super::register_store(scratch, register, true)
                    .is_some_and(|store| code.ends_with(store)) =>
        {
            super::Rax::Unknown
        }
        super::Rax::Nzcv if live_nzcv => super::Rax::LiveNzcv,
        rax => rax,
    };
    debug_assert!(match rax {
        super::Rax::Unknown | super::Rax::PairStoreAddress { .. } => true,
        super::Rax::Nzcv | super::Rax::LiveNzcv => code.ends_with(&nzcv_store(code.is_pinned())),
        super::Rax::Guest(register) => super::register_store(scratch, register, code.is_pinned())
            .is_some_and(|store| code.ends_with(store)),
    });
    let keep_vectors = !code.constant_time() && preserves_held_vectors(&instruction.operation);
    let xmm0 = match &instruction.operation {
        Operation::Simd(operand) => super::simd::xmm0_after(
            scratch,
            operand,
            &code[start..],
            held.xmm0,
            code.vector_cache(),
        ),
        _ if keep_vectors => held.xmm0,
        _ => None,
    };
    let xmm_pair = match &instruction.operation {
        Operation::Simd(Simd::MemoryPair { vectors, .. }) if !code.constant_time() => {
            Some(*vectors)
        }
        _ if keep_vectors => held.xmm_pair,
        _ => None,
    };
    Held {
        rax,
        xmm0,
        xmm_pair,
    }
}

/// Where a branch field or its target lies once the block is laid out.
#[derive(Clone, Copy)]
enum Place {
    Inline(usize),
    OutOfLine(usize),
}

const JMP_REL32: u8 = 0xe9;
const REL32_BYTES: usize = size_of::<i32>();

/// The slow paths of numerical instructions ([`ColdSplit`]), moved after
/// the block's last ordinary instruction behind one jump, before the
/// terminal, whose direct exits must end the code. Inline, each such
/// instruction is its sticky-inexact path alone, falling through to the
/// next instruction; its moved part ends with a jump back to there.
///
/// Consecutive such instructions form a run in which only the first
/// checks FPCR and FPSR: a sticky path changes neither, so the next one
/// holds whenever it is reached inline, and the moved part of one
/// continues in the next one's full path, which is correct in every
/// state. Moved code accesses no guest memory, so the source map entry
/// of the last ordinary instruction that also covers it names no fault.
///
/// [`ColdSplit`]: super::simd::numerical::ColdSplit
#[derive(Default)]
struct OutOfLine {
    code: Code,
    /// rel32 fields with the places they target.
    fixups: Vec<(Place, Place)>,
    /// The jump ending the last moved part and where its instruction ended
    /// inline: the next instruction's full path continues the run if it
    /// starts there.
    open: Option<(usize, usize)>,
}

impl OutOfLine {
    /// Whether an instruction starting at `start` continues a run. Every
    /// instruction emits code (the source map's offsets strictly increase),
    /// so nothing came between.
    fn continues_at(&self, start: usize) -> bool {
        self.open.is_some_and(|(_, end)| end == start)
    }

    /// Move the slow paths of the numerical instruction emitted from
    /// `start` to the end of `code`, which ends its emission.
    fn take(
        &mut self,
        code: &mut Code,
        start: usize,
        split: Option<super::simd::numerical::ColdSplit>,
    ) {
        let base = self.code.len();
        let continued = split.as_ref().filter(|_| self.continues_at(start));
        if let Some((site, end)) = self.open.take() {
            let target = continued.map_or(Place::Inline(end), |split| {
                Place::OutOfLine(base + split.full - split.cold.offset())
            });
            self.fixups.push((Place::OutOfLine(site), target));
        }
        let Some(split) = split else {
            return;
        };
        let end = code.len();
        let inline_end = split.inline_end.offset();
        let cold = split.cold.offset();
        debug_assert_eq!(code[inline_end], JMP_REL32);
        debug_assert_eq!(target_of(code, inline_end + 1), end);
        // A branch from the sticky path past the full path now falls
        // through; one from the moved part to its end takes the jump back.
        let place = |offset: usize, inline: bool| {
            debug_assert!(!(inline_end..cold).contains(&offset));
            if offset < inline_end || (offset == end && inline) {
                Place::Inline(offset.min(inline_end))
            } else {
                Place::OutOfLine(base + offset - cold)
            }
        };
        for &site in &split.crossing {
            let inline = site < inline_end;
            let target = target_of(code, site);
            self.fixups
                .push((place(site, inline), place(target, inline)));
        }
        self.code.append(code.split_off(split.cold));
        let back = self.code.asm().jmp();
        self.open = Some((back.offset(), inline_end));
        code.truncate(split.inline_end);
    }

    /// Lay out the moved code at the end of `code` behind a jump over it.
    fn place(mut self, code: &mut Code) {
        if let Some((site, end)) = self.open.take() {
            self.fixups
                .push((Place::OutOfLine(site), Place::Inline(end)));
        }
        if self.code.is_empty() {
            return;
        }
        let over = code.asm().jmp();
        let base = code.len();
        code.append(self.code);
        code.asm().bind(over);
        let at = |place| match place {
            Place::Inline(offset) => offset,
            Place::OutOfLine(offset) => base + offset,
        };
        for (site, target) in self.fixups {
            super::simd::numerical::resolve(code, at(site), at(target));
        }
    }
}

/// The target of the rel32 field at `site`.
fn target_of(code: &[u8], site: usize) -> usize {
    let field = i32::from_le_bytes(code[site..site + REL32_BYTES].try_into().unwrap());
    (site + REL32_BYTES).wrapping_add_signed(field as isize)
}

/// `mov [rdi+GuestState.nzcv],eax`, which ends each emission after which
/// RAX holds [`super::Rax::Nzcv`].
fn nzcv_store(pinned: bool) -> Code {
    let mut code = Code::new();
    code.set_pinned(pinned);
    match pinned {
        true => super::flag_word::store_captured(&mut code.asm()),
        false => super::store_nzcv(&mut code.asm()),
    }
    code
}
/// The exit word of a side exit that skips `skipped` instructions.
fn side_exit(skipped: usize) -> ExitWord {
    ExitWord {
        kind: ExitKind::Direct,
        skipped: u16::try_from(skipped).expect("bounded block instructions"),
    }
}

/// The direct exits to static successors that [`emit_terminal`] ended
/// `native`'s code with, the target's last, after a conditional branch's
/// fallthrough, and the side exits of the branches in `instructions`, in
/// their order, that follow them.
pub(crate) fn successor_exits(
    native: &NativeBlock,
    instructions: &[Instruction],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    linked: Option<u64>,
) -> Vec<super::chain::DirectExit> {
    use super::chain::{DIRECT_EXIT_BYTES, DirectExit, link_slot};
    let branches: Vec<_> = instructions
        .iter()
        .enumerate()
        .filter_map(|(at, branch)| Some((at + 1, branch.operation.conditional_target()?)))
        .collect();
    let slot = |target| link_slot(linked, target, ExitKind::Direct);
    let side_bytes: usize = branches
        .iter()
        .map(|(_, target)| slot(*target) + DIRECT_EXIT_BYTES)
        .sum();
    // Check failures end the code, after every exit's RET.
    let mut end = super::chain::without_failures(&native.x86_64_code).len() - side_bytes;
    let mut exits = terminal_exits(&native.x86_64_code[..end], terminal, fallthrough, linked);
    for (run, target) in branches {
        let word = side_exit(native.source_map.len() - run);
        let offset = end + slot(target);
        exits.push(DirectExit {
            offset,
            target,
            kind: word.kind,
            skipped: word.skipped,
        });
        end = offset + DIRECT_EXIT_BYTES;
    }
    exits
}

/// The direct exits that end `code`.
fn terminal_exits(
    code: &[u8],
    terminal: Option<&(GuestAddress, Terminator)>,
    fallthrough: GuestAddress,
    linked: Option<u64>,
) -> Vec<super::chain::DirectExit> {
    use super::chain::{DIRECT_EXIT_BYTES, DirectExit, is_direct_exit, link_slot};
    let (target, kind, conditional) = match terminal.map(|(_, terminal)| terminal) {
        None => (fallthrough, ExitKind::Fallthrough, false),
        Some(Terminator::DirectCall(target)) => (*target, ExitKind::Call, false),
        Some(Terminator::Branch(Operation::Branch { target })) => {
            (*target, ExitKind::Direct, false)
        }
        Some(Terminator::Branch(
            Operation::BranchCondition { target, .. }
            | Operation::BranchZero { target, .. }
            | Operation::BranchBit { target, .. },
        )) => (*target, ExitKind::Direct, true),
        // A DC ZVA, DC clean or exclusive completed natively falls through;
        // any other System instruction exits at itself, which the stub check below
        // rejects.
        Some(Terminator::System(request)) => (request.next_pc, ExitKind::Fallthrough, false),
        // So does a numerical operation completed inline; otherwise the same check
        // rejects the exit at the instruction.
        Some(Terminator::FloatingPoint { .. }) => (fallthrough, ExitKind::Fallthrough, false),
        Some(_) => return vec![],
    };
    // The target's exit ends the code, after a conditional's fallthrough's.
    let last = code.len().checked_sub(DIRECT_EXIT_BYTES);
    let before =
        last.and_then(|last| last.checked_sub(link_slot(linked, target, kind) + DIRECT_EXIT_BYTES));
    let candidates = [
        conditional
            .then_some(before)
            .flatten()
            .map(|offset| (offset, fallthrough, ExitKind::Fallthrough)),
        last.map(|offset| (offset, target, kind)),
    ];
    candidates
        .into_iter()
        .flatten()
        .filter_map(|(offset, target, kind)| {
            // An always-true condition emits no fallthrough exit.
            is_direct_exit(&code[offset..offset + DIRECT_EXIT_BYTES], target, kind).then_some(
                DirectExit {
                    offset,
                    target,
                    kind,
                    skipped: 0,
                },
            )
        })
        .collect()
}

/// What a block's terminal is emitted against.
struct TerminalContext {
    fallthrough: GuestAddress,
    native: super::NativeSystem,
    checks: super::MemoryChecks,
    rax: super::Rax,
    /// The body ends with the NZCV capture, out-of-line paths aside, so
    /// the host flags still hold it ([`branch_condition_with`]).
    flags_live: bool,
    target: crate::TargetCpu,
    zero_branch: Option<Cond>,
}

fn emit_terminal(
    code: &mut Code,
    source_map: &mut Vec<(GuestAddress, usize)>,
    terminal: Option<&(GuestAddress, Terminator)>,
    context: TerminalContext,
) {
    let TerminalContext {
        fallthrough,
        native,
        checks,
        rax,
        flags_live,
        target,
        zero_branch,
    } = context;
    if let Some((pc, terminal)) = terminal {
        source_map.push((*pc, code.len()));
        code.set_instruction(source_map.len() - 1);
        match terminal {
            Terminator::FloatingPoint { inline } => {
                // A numerical terminal is unexecuted. Ordinary profiles put
                // inline-capable forms in the body; the checked system profile
                // delegates every numerical form to the runtime service. What
                // ends its block on every host completes here on a host with
                // the instructions for it.
                match inline {
                    Some(operation)
                        if super::simd::numerical_terminal_inline(operation, target) =>
                    {
                        super::simd::emit_numerical(code, operation, *pc, target);
                        direct_exit(code, fallthrough, ExitKind::Fallthrough);
                    }
                    _ => direct_exit(code, *pc, ExitKind::FloatingPoint),
                }
            }
            Terminator::Clock(request) => {
                // MRS results and ISB come from the runtime clock service.
                direct_exit(code, request.instruction_pc, ExitKind::Clock);
            }
            Terminator::ThreadRegister(request) => {
                direct_exit(code, request.instruction_pc, ExitKind::ThreadRegister);
            }
            Terminator::Amx(request) => {
                // The runtime runs the operation on the thread's AMX state.
                direct_exit(code, request.instruction_pc, ExitKind::Amx);
            }
            Terminator::PointerAuthentication(request) => {
                // Return to the checked runtime helper with unchanged guest
                // state at this instruction. Never embed keys/helper pointers.
                direct_exit(
                    code,
                    request.instruction_pc,
                    ExitKind::PointerAuthentication,
                );
            }
            Terminator::System(request) => {
                let zero = native.zero_block.zip(super::zero_block::operand(request));
                let clean = super::cache_clean::operand(request)
                    .filter(|(operation, _)| native.cache_clean.contains(*operation));
                let exclusive =
                    super::native_exclusive::operand(request).filter(|_| native.exclusive);
                if let Some(transfer) = exclusive {
                    super::native_exclusive::emit(code, transfer, request, checks);
                } else if let Some((bytes, rt)) = zero {
                    super::zero_block::emit(code, rt, bytes, checks);
                    direct_exit(code, request.next_pc, ExitKind::Fallthrough);
                } else if let Some((_, rt)) = clean {
                    super::cache_clean::emit(code, rt, checks);
                    direct_exit(code, request.next_pc, ExitKind::Fallthrough);
                } else {
                    direct_exit(code, request.instruction_pc, ExitKind::System);
                }
            }
            Terminator::Trap(trap) => {
                // SVC and BRK stop at the trapping instruction; the runtime
                // handles the trap.
                direct_exit(code, trap.instruction_pc, ExitKind::Trap);
            }
            Terminator::DirectCall(target) => {
                super::guest_immediate(code, fallthrough);
                super::store(code, Register::X(30));
                direct_exit(code, *target, ExitKind::Call);
            }
            Terminator::Register { source, kind } => {
                load_unless_held(code, *source, rax);
                if *kind == ExitKind::Call {
                    // Capture BLR X30's target before LR is overwritten.
                    code.asm().mov(Size::Qword, Gpr::Rcx, Gpr::Rax);
                    super::guest_immediate(code, fallthrough);
                    super::store(code, Register::X(30));
                    code.asm().mov(Size::Qword, Gpr::Rax, Gpr::Rcx);
                }
                exit_kind(code, *kind);
            }
            Terminator::Branch(operation) => match *operation {
                Operation::Branch { target } => direct_exit(code, target, ExitKind::Direct),
                Operation::BranchCondition { target, .. }
                | Operation::BranchZero { target, .. }
                | Operation::BranchBit { target, .. } => {
                    let taken = zero_branch
                        .or_else(|| branch_condition_with(code, operation, rax, flags_live));
                    conditional_exit(code, taken, target, fallthrough);
                }
                _ => unreachable!("block decoder restricts branch terminators"),
            },
        }
    } else {
        direct_exit(code, fallthrough, ExitKind::Fallthrough);
    }
}

/// Load a register unless RAX, as `rax` says, already holds it.
fn load_unless_held(code: &mut Code, register: Register, rax: super::Rax) {
    if rax != super::Rax::Guest(register) {
        super::load(code, register);
    }
}

/// Test a conditional branch's condition: the host condition it is taken
/// in, or `None` when it always is. `rax` says what RAX holds.
fn branch_condition(code: &mut Code, operation: &Operation, rax: super::Rax) -> Option<Cond> {
    branch_condition_with(code, operation, rax, false)
}

/// [`branch_condition`], with the NZCV capture known to have left the host
/// flags live where `flags_live` (a block's out-of-line paths, which the
/// body jumps over, come between the capture and its terminal).
fn branch_condition_with(
    code: &mut Code,
    operation: &Operation,
    rax: super::Rax,
    flags_live: bool,
) -> Option<Cond> {
    match *operation {
        Operation::BranchCondition { condition, .. } => {
            if rax == super::Rax::LiveNzcv {
                return Some(super::flag_word::condition(code, condition, true));
            }
            let loaded = rax == super::Rax::Nzcv
                && (flags_live || code.ends_with(&nzcv_store(code.is_pinned())));
            super::condition_holds(code, condition, loaded)
        }
        Operation::BranchZero {
            source,
            width,
            nonzero,
            ..
        } => {
            let size = Size::of(width == crate::aarch64::Width::X64);
            if let Some(host) = code.host(source) {
                code.asm().test(size, host, host);
            } else if let Some(slot) = super::offset(source)
                && rax != super::Rax::Guest(source)
            {
                code.asm().alu_imm8(Alu::Cmp, size, super::frame(slot), 0);
            } else {
                load_unless_held(code, source, rax);
                return Some(super::test_zero(code, width, nonzero));
            }
            Some(if nonzero { Cond::Ne } else { Cond::E })
        }
        Operation::BranchBit {
            source,
            bit,
            nonzero,
            ..
        } => Some(if let Some(host) = code.host(source) {
            super::test_bit_of(code.asm(), host, bit, nonzero)
        } else if let Some(slot) = super::offset(source)
            && rax != super::Rax::Guest(source)
        {
            let byte = super::frame(slot + u32::from(bit / 8));
            code.asm().test_imm(Size::Byte, byte, 1 << (bit % 8));
            if nonzero { Cond::Ne } else { Cond::E }
        } else {
            load_unless_held(code, source, rax);
            super::test_bit(code, bit, nonzero)
        }),
        _ => unreachable!("not a conditional branch"),
    }
}

#[cfg(test)]
mod flag_cost_tests;
#[cfg(test)]
mod host_cost;
#[cfg(test)]
mod lazy_flags_tests;
#[cfg(test)]
mod tests;
#[cfg(test)]
mod vector_cache_tests;
