//! Block boundaries, static successors and dynamic edges, found without emitting code.
use super::{
    BlockProfile, ClockInstruction, ExitKind, Features, MAX_BLOCK_INSTRUCTIONS, NOP_ENCODING,
    PointerAuthentication, PointerAuthenticationOperation, SynchronousTrap, SystemInstruction,
    Terminator, ThreadRegisterInstruction, decode_clock_instruction, decode_pointer_authentication,
    decode_system_instruction, decode_terminator, decode_thread_register_instruction, memory,
    system,
};
use crate::{
    CompileError, CompileErrorKind, GuestAddress, Lifter,
    aarch64::{Decoder, Instruction, Operation, Register},
    x86_64::NativeSystem,
};
use alloc::vec::Vec;
use serde::Serialize;

pub(super) struct ParsedBlock {
    pub(super) instructions: Vec<Instruction>,
    pub(super) terminal: Option<(GuestAddress, Terminator)>,
    pub(super) next_pc: GuestAddress,
    pub(super) source_bytes: usize,
}

/// An analysis failure at the instruction that caused it.
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct BlockAnalysisError {
    pub pc: GuestAddress,
    pub encoding: Option<u32>,
    pub kind: CompileErrorKind,
}
impl core::fmt::Display for BlockAnalysisError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "{} at {:#x}", self.kind, self.pc.0)?;
        if let Some(word) = self.encoding {
            write!(f, " ({word:#010x})")?;
        }
        Ok(())
    }
}
impl core::error::Error for BlockAnalysisError {}
impl From<BlockAnalysisError> for CompileError {
    fn from(error: BlockAnalysisError) -> Self {
        let result = CompileError::new(error.kind);
        match error.encoding {
            Some(word) => result.at_instruction(error.pc, word),
            None => result.at(error.pc),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum SuccessorKind {
    Direct,
    ConditionalTarget,
    ConditionalFallthrough,
    CallTarget,
    CallContinuation,
    Fallthrough,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct StaticSuccessor {
    pub kind: SuccessorKind,
    pub target: GuestAddress,
}
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize)]
#[serde(rename_all = "snake_case")]
pub enum DynamicEdgeKind {
    Branch,
    Call,
    Return,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct DynamicEdge {
    pub kind: DynamicEdgeKind,
    /// Architectural Xn source; index31 denotes Zero, never SP.
    pub register: u8,
}
#[derive(Debug, Clone, PartialEq, Eq, Serialize)]
pub struct BlockAnalysis {
    pub source_bytes: usize,
    pub terminator_pc: Option<GuestAddress>,
    pub static_successors: Vec<StaticSuccessor>,
    pub dynamic_edge: Option<DynamicEdge>,
    pub trap: Option<SynchronousTrap>,
    pub system_instruction: Option<SystemInstruction>,
    pub pointer_authentication: Option<PointerAuthentication>,
    pub clock_instruction: Option<ClockInstruction>,
    pub thread_register_instruction: Option<ThreadRegisterInstruction>,
    pub amx_instruction: Option<crate::amx::AmxInstruction>,
}
/// Find block boundaries and edges with the parser shared by AOT and JIT. Call
/// continuations are candidates, since the callee may not return; dynamic
/// targets and trap resumption are resolved at run time.
pub fn analyze(source: &[u8], base: GuestAddress) -> Result<BlockAnalysis, BlockAnalysisError> {
    analyze_with_profile(source, base, BlockProfile::ORDINARY)
}

pub fn analyze_with_profile(
    source: &[u8],
    base: GuestAddress,
    profile: BlockProfile,
) -> Result<BlockAnalysis, BlockAnalysisError> {
    parse(source, base, profile).map(|parsed| analyze_parsed(&parsed))
}

/// The analysis of a parsed block.
pub(super) fn analyze_parsed(parsed: &ParsedBlock) -> BlockAnalysis {
    let mut successors = Vec::new();
    let mut dynamic_edge = None;
    let mut trap = None;
    let mut system_instruction = None;
    let mut pointer_authentication = None;
    let mut clock_instruction = None;
    let mut thread_register_instruction = None;
    let mut amx_instruction = None;
    let mut add = |kind, target| successors.push(StaticSuccessor { kind, target });
    for instruction in &parsed.instructions {
        if let Some(target) = instruction.operation.conditional_target() {
            add(SuccessorKind::ConditionalTarget, target);
        }
    }
    match parsed.terminal.as_ref().map(|(_, terminal)| terminal) {
        None => add(SuccessorKind::Fallthrough, parsed.next_pc),
        Some(Terminator::Branch(Operation::Branch { target })) => {
            add(SuccessorKind::Direct, *target)
        }
        Some(Terminator::Branch(
            Operation::BranchCondition { target, .. }
            | Operation::BranchZero { target, .. }
            | Operation::BranchBit { target, .. },
        )) => {
            add(SuccessorKind::ConditionalTarget, *target);
            add(SuccessorKind::ConditionalFallthrough, parsed.next_pc);
        }
        Some(Terminator::DirectCall(target)) => {
            add(SuccessorKind::CallTarget, *target);
            add(SuccessorKind::CallContinuation, parsed.next_pc);
        }
        Some(Terminator::Register { source, kind }) => {
            let edge = match kind {
                ExitKind::Indirect => DynamicEdgeKind::Branch,
                ExitKind::Call => DynamicEdgeKind::Call,
                ExitKind::Return => DynamicEdgeKind::Return,
                _ => unreachable!("private parser only constructs ordinary register transfers"),
            };
            let register = match source {
                Register::X(index) => *index,
                Register::Zero => 31,
                Register::Sp => unreachable!("transfer source never SP"),
            };
            dynamic_edge = Some(DynamicEdge {
                kind: edge,
                register,
            });
            if *kind == ExitKind::Call {
                add(SuccessorKind::CallContinuation, parsed.next_pc);
            }
        }
        Some(Terminator::Trap(value)) => trap = Some(*value),
        Some(Terminator::System(value)) => {
            system_instruction = Some(*value);
            if matches!(
                value.operation,
                system::SystemOperation::TagMemory(_)
                    | system::SystemOperation::TagOffset(_)
                    | system::SystemOperation::SendEvent { .. }
                    | system::SystemOperation::Register(_)
                    | system::SystemOperation::PstateImmediate(_)
                    | system::SystemOperation::Wait { .. }
                    | system::SystemOperation::Unprivileged(_)
                    | system::SystemOperation::Exclusive(_)
                    | system::SystemOperation::CheckedPointer(_)
                    | system::SystemOperation::Compression(_)
            ) {
                add(SuccessorKind::Fallthrough, value.next_pc);
            }
        }
        Some(Terminator::PointerAuthentication(value)) => {
            pointer_authentication = Some(*value);
            if value.operation.is_return() {
                // Signed LR is only a candidate: the service authenticates it.
                dynamic_edge = Some(DynamicEdge {
                    kind: DynamicEdgeKind::Return,
                    register: 30,
                });
            } else if let PointerAuthenticationOperation::Branch { target, link, .. } =
                value.operation
            {
                dynamic_edge = Some(DynamicEdge {
                    kind: if link {
                        DynamicEdgeKind::Call
                    } else {
                        DynamicEdgeKind::Branch
                    },
                    register: target,
                });
                if link {
                    add(SuccessorKind::CallContinuation, value.next_pc);
                }
            } else {
                add(SuccessorKind::Fallthrough, value.next_pc);
            }
        }
        Some(Terminator::Clock(value)) => {
            clock_instruction = Some(*value);
            add(SuccessorKind::Fallthrough, value.next_pc);
        }
        Some(Terminator::ThreadRegister(value)) => {
            thread_register_instruction = Some(*value);
            add(SuccessorKind::Fallthrough, value.next_pc);
        }
        Some(Terminator::FloatingPoint { .. }) => {
            add(SuccessorKind::Fallthrough, parsed.next_pc);
        }
        Some(Terminator::Amx(value)) => {
            amx_instruction = Some(*value);
            add(SuccessorKind::Fallthrough, value.next_pc);
        }
        Some(Terminator::Branch(_)) => {
            unreachable!("private parser only stores branch operations as terminators")
        }
    }
    BlockAnalysis {
        source_bytes: parsed.source_bytes,
        terminator_pc: parsed.terminal.as_ref().map(|(pc, _)| *pc),
        static_successors: successors,
        dynamic_edge,
        trap,
        system_instruction,
        pointer_authentication,
        clock_instruction,
        thread_register_instruction,
        amx_instruction,
    }
}

/// The forward conditional branches one system-mode block runs past.
const MAX_SIDE_EXITS: usize = 4;

/// The instructions a parse makes room for at first: 39 blocks in 40 of
/// a kernel's code have at most this many, and the room stays under 1 KiB,
/// which allocators keep at hand.
const INSTRUCTIONS_HINT: usize = 24;

pub(super) fn parse(
    source: &[u8],
    base: GuestAddress,
    profile: BlockProfile,
) -> Result<ParsedBlock, BlockAnalysisError> {
    parse_admitted(source, base, profile, NativeSystem::NONE, |_| true)
}

/// [`parse`], with the PAC instructions (`x86_64::pointer::inline`) and the
/// MRS of mirrored registers (`x86_64::mirrored_read`) that blocks compiled
/// for `native` complete inside themselves parsed as instructions.
pub(super) fn parse_with(
    source: &[u8],
    base: GuestAddress,
    profile: BlockProfile,
    native: NativeSystem,
) -> Result<ParsedBlock, BlockAnalysisError> {
    parse_admitted(source, base, profile, native, |_| true)
}

pub(super) fn parse_admitted(
    source: &[u8],
    base: GuestAddress,
    profile: BlockProfile,
    native: NativeSystem,
    mut admit: impl FnMut(u32) -> bool,
) -> Result<ParsedBlock, BlockAnalysisError> {
    let inline_pointer = native.pointer.is_some();
    // Only the engine's own system-mode blocks read its cells.
    let mirrored = if profile == BlockProfile::new(Features::SYSTEM_MODE) {
        native.mirrored
    } else {
        crate::x86_64::mirrored_read::Reads::NONE
    };
    if source.is_empty() || !source.len().is_multiple_of(4) || !base.0.is_multiple_of(4) {
        return Err(BlockAnalysisError {
            pc: base,
            encoding: None,
            kind: CompileErrorKind::InvalidSourceLayout,
        });
    }
    if source.len() / 4 > MAX_BLOCK_INSTRUCTIONS {
        return Err(BlockAnalysisError {
            pc: base,
            encoding: None,
            kind: CompileErrorKind::TooManyInstructions {
                maximum: MAX_BLOCK_INSTRUCTIONS,
            },
        });
    }
    // Room for most blocks' instructions, rather than growing from empty.
    let mut instructions = Vec::with_capacity((source.len() / 4).min(INSTRUCTIONS_HINT));
    let mut terminal = None;
    let mut source_bytes = 0;
    let mut side_exits = 0;
    let mut next_pc = base;
    let full_system = profile.has(Features::FULL_SYSTEM);
    let pointer_authentication = profile.has(Features::POINTER_AUTHENTICATION);
    let checked_memory = profile.checked_memory().is_some();
    let restricted_system = profile.has(Features::SYSTEM) && !full_system;
    for bytes in source.as_chunks::<4>().0 {
        let word = u32::from_le_bytes(*bytes);
        if !admit(word) {
            break;
        }
        let pc = next_pc;
        next_pc = GuestAddress(pc.0.checked_add(4).ok_or(BlockAnalysisError {
            pc,
            encoding: None,
            kind: CompileErrorKind::GuestAddressOverflow,
        })?);
        source_bytes += 4;
        // The commonest integer words, which no decoder here takes either
        // (`tests::direct_words_skip_the_block_decoders`), go to their
        // decoder directly.
        let direct = crate::aarch64::skips_decoders(pc, word);
        if full_system && !direct {
            // PAuth hints share the hint space, so they are matched first.
            if let Some(decoded) =
                decode_pointer_authentication(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
            {
                if inline_pointer && crate::x86_64::pointer::inline::inlines(decoded.operation) {
                    instructions.push(Instruction {
                        pc,
                        operation: Operation::PointerAuthentication(decoded.operation),
                    });
                    continue;
                }
                terminal = Some((pc, Terminator::PointerAuthentication(decoded)));
                break;
            }
            use crate::aarch64::system::synchronization::{Operation as Sync, decode};
            let synchronization = decode(word);
            // BTI executes as NOP (Arm `BTI`). PSTATE.BTYPE can be nonzero
            // only for a block's first instruction, after the indirect
            // branch that ended the previous block, and the engine checks
            // that instruction before entering the block.
            if matches!(synchronization, Some(Sync::BranchTarget { .. })) {
                instructions.push(Instruction {
                    pc,
                    operation: Operation::Nop,
                });
                continue;
            }
            // ISB executes as NOP in the serialized engine (Arm `ISB`): what
            // it synchronizes, a System register write, TLB or cache
            // maintenance, exception entry and return, ends its block and
            // completes in the engine, which recomputes what derives from
            // it before the next fetch. The shared lane's PEs keep it.
            if profile == BlockProfile::new(Features::SYSTEM_MODE)
                && matches!(
                    synchronization,
                    Some(Sync::InstructionSynchronization { .. })
                )
            {
                instructions.push(Instruction {
                    pc,
                    operation: Operation::Nop,
                });
                continue;
            }
            if let Some(decoded) = system::decode_system_mode_with_sync(pc, word, synchronization)
                .map_err(|error| BlockAnalysisError {
                pc,
                encoding: Some(word),
                kind: error.kind,
            })? {
                if let system::SystemOperation::Register(access) = decoded.operation
                    && access.direction() == crate::aarch64::system::registers::Direction::Read
                    && mirrored.cell(word).is_some()
                {
                    instructions.push(Instruction {
                        pc,
                        operation: Operation::MirroredRead { word },
                    });
                    continue;
                }
                // DC ZVA completes natively inside the engine's own blocks
                // (`x86_64::zero_block`) with self-loops on: a loop zeroing
                // memory stays one block, and runs inside it.
                if profile == BlockProfile::new(Features::SYSTEM_MODE)
                    && native.self_loops
                    && native.zero_block.is_some()
                    && let Some(rt) = crate::x86_64::zero_block::operand(&decoded)
                {
                    instructions.push(Instruction {
                        pc,
                        operation: Operation::ZeroBlock { rt },
                    });
                    continue;
                }
                // The shared lane executes these through the existing typed
                // all-writer memory service, not the single-agent CPU monitor.
                let checked = checked_memory
                    && matches!(decoded.operation, system::SystemOperation::Exclusive(Ok(_)));
                if !checked {
                    terminal = Some((pc, Terminator::System(decoded)));
                    break;
                }
            }
        }
        if restricted_system
            && !direct
            && let Some(decoded) =
                decode_system_instruction(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
            && (profile.has(Features::SYSTEM_REGISTERS)
                || !matches!(
                    decoded.operation,
                    system::SystemOperation::Register(_)
                        | system::SystemOperation::PstateImmediate(_)
                ))
        {
            terminal = Some((pc, Terminator::System(decoded)));
            break;
        }
        if !full_system
            && !direct
            && pointer_authentication
            && let Some(decoded) =
                decode_pointer_authentication(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
        {
            terminal = Some((pc, Terminator::PointerAuthentication(decoded)));
            break;
        }
        if profile.has(Features::CLOCK)
            && !direct
            && let Some(decoded) =
                decode_clock_instruction(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
        {
            terminal = Some((pc, Terminator::Clock(decoded)));
            break;
        }
        if profile.has(Features::THREAD_REGISTERS)
            && !direct
            && let Some(decoded) =
                decode_thread_register_instruction(pc, word).map_err(|error| {
                    BlockAnalysisError {
                        pc,
                        encoding: Some(word),
                        kind: error.kind,
                    }
                })?
        {
            terminal = Some((pc, Terminator::ThreadRegister(decoded)));
            break;
        }
        if profile.has(Features::AMX)
            && !direct
            && let Some(decoded) =
                crate::amx::decode(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
        {
            // The engine's own blocks move AMX rows inside themselves.
            if native.amx
                && profile == BlockProfile::new(Features::SYSTEM_MODE)
                && crate::x86_64::amx_transfer::in_block(decoded.operation)
            {
                instructions.push(Instruction {
                    pc,
                    operation: Operation::Amx(decoded),
                });
                continue;
            }
            terminal = Some((pc, Terminator::Amx(decoded)));
            break;
        }
        if !direct
            && let Some(decoded) =
                decode_terminator(pc, word).map_err(|error| BlockAnalysisError {
                    pc,
                    encoding: Some(word),
                    kind: error.kind,
                })?
        {
            terminal = Some((pc, decoded));
            break;
        }
        let instruction = if direct {
            crate::aarch64::lift_direct(pc, word)
        } else {
            Decoder.lift(pc, word)
        };
        let instruction = instruction.map_err(|error| BlockAnalysisError {
            pc,
            encoding: Some(word),
            kind: CompileErrorKind::Unsupported(error),
        })?;
        if checked_memory && instruction.operation == Operation::Nop && word != NOP_ENCODING {
            return Err(BlockAnalysisError {
                pc,
                encoding: Some(word),
                kind: CompileErrorKind::UnadmittedNopProvenance,
            });
        }
        if restricted_system && !crate::scalable_context::ordinary_permitted(&instruction.operation)
        {
            return Err(BlockAnalysisError {
                pc,
                encoding: Some(word),
                kind: CompileErrorKind::OutsideRestrictedSystemProfile,
            });
        }
        if let Operation::FloatingPoint(operand) = &instruction.operation {
            let checked = full_system && checked_memory;
            if checked || crate::x86_64::numerical_always_exits(operand) {
                // The checked full-system prefix has no inline numerical recipe.
                // Recover its exact prefix and complete through the existing FP
                // service, including forms other profiles can execute inline.
                let inline = (!checked).then_some(*operand);
                terminal = Some((pc, Terminator::FloatingPoint { inline }));
                break;
            }
        }
        // Code lays its likely path out straight ahead, so a block runs past
        // a forward conditional branch and leaves by a side exit when it is
        // taken. A branch back is likely taken, and ends the block.
        if side_exits < MAX_SIDE_EXITS
            && source_bytes < source.len()
            && full_system
            && !checked_memory
            && instruction
                .operation
                .conditional_target()
                .is_some_and(|target| target.0 > pc.0)
        {
            side_exits += 1;
            instructions.push(instruction);
            continue;
        }
        if matches!(
            instruction.operation,
            Operation::Branch { .. }
                | Operation::BranchCondition { .. }
                | Operation::BranchZero { .. }
                | Operation::BranchBit { .. }
        ) {
            terminal = Some((pc, Terminator::Branch(instruction.operation)));
            break;
        }
        instructions.push(instruction);
    }
    Ok(ParsedBlock {
        instructions,
        terminal,
        next_pc,
        source_bytes,
    })
}

/// Validate and identify the consumed prefix without generating native code.
/// Uses the same semantic lift and transfer boundaries as [`compile`].
pub fn source_bytes(source: &[u8], base: GuestAddress) -> Result<usize, CompileError> {
    source_bytes_with_profile(source, base, BlockProfile::ORDINARY)
}

pub fn source_bytes_with_profile(
    source: &[u8],
    base: GuestAddress,
    profile: BlockProfile,
) -> Result<usize, CompileError> {
    let parsed = parse(source, base, profile).map_err(CompileError::from)?;
    if let Some(selected) = profile.checked_memory() {
        memory::CheckedPlan::new(
            parsed.instructions,
            parsed.terminal.as_ref(),
            selected,
            profile,
        )?;
    }
    Ok(parsed.source_bytes)
}
