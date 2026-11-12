//! What a compiled guest snippet costs on the host: the instructions on its
//! fallthrough path, from the block's start to its first exit, by category.
//! Taken side exits and moved slow paths are not on that path. A length
//! decoder for the encodings the emitters produce walks the code; an
//! encoding it does not know fails the test that met it.
use crate::{
    GuestAddress, Options, TargetCpu,
    block::{BlockProfile, CompilationPlan, Features},
    x86_64::{MemoryChecks, NativeSystem, RangeCheck},
};
use alloc::vec::Vec;

/// The host instructions on a block's fallthrough path.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub(super) struct Cost {
    pub(super) instructions: usize,
    /// Instructions with a memory operand (not LEA or a hinting NOP).
    pub(super) memory: usize,
    /// Instructions that form, store, load or test the NZCV word: LAHF,
    /// SAHF, CMC, SETO, `add al,0x7f` and any access to the word.
    pub(super) flags: usize,
    /// Conditional and unconditional jumps.
    pub(super) branches: usize,
}

/// One decoded instruction.
#[derive(Debug, Clone, Copy)]
struct Decoded {
    length: usize,
    /// The opcode map (0 one-byte, 1 0F, 2 0F38, 3 0F3A) and opcode.
    map: u8,
    opcode: u8,
    /// The memory operand's base register and displacement, if any.
    memory: Option<(Option<u8>, i32)>,
    /// A relative branch's target offset, and whether it is conditional.
    branch: Option<(usize, bool)>,
    /// The first immediate byte.
    immediate: Option<u8>,
}

/// Decode the instruction at `at`.
fn decode(code: &[u8], at: usize) -> Decoded {
    let fail = || -> ! { panic!("unknown encoding at {at:#x}: {:02x?}", &code[at..]) };
    let mut cursor = at;
    let mut operand16 = false;
    while matches!(
        code[cursor],
        0x66 | 0x67 | 0xf0 | 0xf2 | 0xf3 | 0x2e | 0x3e | 0x26 | 0x36 | 0x64 | 0x65
    ) {
        operand16 |= code[cursor] == 0x66;
        cursor += 1;
    }
    let mut rex = 0;
    if code[cursor] & 0xf0 == 0x40 {
        rex = code[cursor];
        cursor += 1;
    }
    let wide = rex & 8 != 0;
    let mut base_extension = rex & 1;
    let (map, opcode) = match code[cursor] {
        0xc5 => {
            let opcode = code[cursor + 2];
            cursor += 3;
            (1, opcode)
        }
        0xc4 => {
            base_extension = u8::from(code[cursor + 1] & 0x20 == 0);
            let map = code[cursor + 1] & 0x1f;
            let opcode = code[cursor + 3];
            cursor += 4;
            (map, opcode)
        }
        0x62 => {
            base_extension = u8::from(code[cursor + 1] & 0x20 == 0);
            let map = code[cursor + 1] & 7;
            let opcode = code[cursor + 4];
            cursor += 5;
            (map, opcode)
        }
        0x0f => match code[cursor + 1] {
            0x38 => {
                cursor += 3;
                (2, code[cursor - 1])
            }
            0x3a => {
                cursor += 3;
                (3, code[cursor - 1])
            }
            opcode => {
                cursor += 2;
                (1, opcode)
            }
        },
        opcode => {
            cursor += 1;
            (0, opcode)
        }
    };
    let immediate_z = if operand16 { 2 } else { 4 };
    // Whether the instruction has a ModRM, its immediate bytes and its
    // relative displacement's bytes.
    let (modrm, mut immediate, relative) = match (map, opcode) {
        (0, 0x00..=0x3f) => match opcode & 7 {
            0..=3 => (true, 0, 0),
            4 => (false, 1, 0),
            5 => (false, immediate_z, 0),
            _ => fail(),
        },
        (0, 0x50..=0x5f | 0x90..=0x99 | 0x9c..=0x9f | 0xc3 | 0xcc | 0xf4 | 0xf5 | 0xf8..=0xfd) => {
            (false, 0, 0)
        }
        (0, 0x63 | 0x84..=0x8f | 0xd0..=0xd3 | 0xfe | 0xff) => (true, 0, 0),
        (0, 0x68) => (false, immediate_z, 0),
        (0, 0x69 | 0x81 | 0xc7) => (true, immediate_z, 0),
        (0, 0x6a | 0xa8 | 0xb0..=0xb7) => (false, 1, 0),
        (0, 0x6b | 0x80 | 0x83 | 0xc0 | 0xc1 | 0xc6) => (true, 1, 0),
        (0, 0x70..=0x7f | 0xeb) => (false, 0, 1),
        (0, 0xa9) => (false, immediate_z, 0),
        (0, 0xb8..=0xbf) => (false, if wide { 8 } else { immediate_z }, 0),
        (0, 0xc2) => (false, 2, 0),
        (0, 0xe8 | 0xe9) => (false, 0, 4),
        // F6 and F7 /0 and /1 (TEST) take an immediate; set below.
        (0, 0xf6 | 0xf7) => (true, 0, 0),
        (1, 0x05 | 0x0b | 0x31 | 0xa2 | 0x77 | 0xc8..=0xcf) => (false, 0, 0),
        (1, 0x80..=0x8f) => (false, 0, 4),
        (1, 0x70..=0x73 | 0xa4 | 0xac | 0xba | 0xc2 | 0xc4..=0xc6) => (true, 1, 0),
        (
            1,
            0x00
            | 0x01
            | 0x0d
            | 0x10..=0x18
            | 0x1f
            | 0x28..=0x2f
            | 0x40..=0x6f
            | 0x74..=0x76
            | 0x7c..=0x7f
            | 0x90..=0x9f
            | 0xa3
            | 0xa5
            | 0xab
            | 0xad..=0xb1
            | 0xb3
            | 0xb6..=0xb8
            | 0xbb..=0xc1
            | 0xc3
            | 0xc7
            | 0xd0..=0xff,
        ) => (true, 0, 0),
        (2, _) => (true, 0, 0),
        (3, _) => (true, 1, 0),
        _ => fail(),
    };
    let mut memory = None;
    if modrm {
        let byte = code[cursor];
        cursor += 1;
        let mode = byte >> 6;
        let reg = (byte >> 3) & 7;
        let mut rm = byte & 7;
        if (map, opcode) == (0, 0xf6) && reg < 2 {
            immediate = 1;
        }
        if (map, opcode) == (0, 0xf7) && reg < 2 {
            immediate = immediate_z;
        }
        if mode != 3 {
            let mut base = Some(rm | base_extension << 3);
            if rm == 4 {
                let sib = code[cursor];
                cursor += 1;
                rm = sib & 7;
                base = Some(rm | base_extension << 3);
                if (sib >> 3) & 7 != 4 || rex & 2 != 0 {
                    // An index: not a plain base and displacement.
                    base = None;
                }
                if mode == 0 && rm == 5 {
                    base = None;
                }
            } else if mode == 0 && rm == 5 {
                // RIP-relative.
                base = None;
            }
            let displacement = match mode {
                0 if rm == 5 => 4,
                1 => 1,
                2 => 4,
                _ => 0,
            };
            let value = match displacement {
                1 => i32::from(code[cursor] as i8),
                4 => i32::from_le_bytes(code[cursor..cursor + 4].try_into().unwrap()),
                _ => 0,
            };
            cursor += displacement;
            memory = Some((base, value));
        }
    }
    let first_immediate = (immediate > 0).then(|| code[cursor]);
    cursor += immediate;
    let branch = match relative {
        1 => {
            cursor += 1;
            let displacement = code[cursor - 1] as i8 as isize;
            Some((cursor.wrapping_add_signed(displacement), opcode != 0xeb))
        }
        4 => {
            cursor += 4;
            let displacement =
                i32::from_le_bytes(code[cursor - 4..cursor].try_into().unwrap()) as isize;
            Some((cursor.wrapping_add_signed(displacement), opcode != 0xe9))
        }
        _ => None,
    };
    Decoded {
        length: cursor - at,
        map,
        opcode,
        memory,
        branch,
        immediate: first_immediate,
    }
}

/// Whether `decoded` forms, stores, loads or tests the NZCV word.
fn is_flag_operation(decoded: &Decoded) -> bool {
    const NZCV: i32 = core::mem::offset_of!(crate::GuestState, nzcv) as i32;
    const RDI: u8 = 7;
    match (decoded.map, decoded.opcode) {
        (0, 0x9e | 0x9f | 0xf5) | (1, 0x90) => true,
        (0, 0x04) => decoded.immediate == Some(0x7f),
        _ => matches!(decoded.memory, Some((Some(RDI), displacement))
            if (NZCV..NZCV + 4).contains(&displacement)),
    }
}

/// Whether the code at `at` is a side exit's capture: flag operations,
/// then a jump.
fn is_capture_stub(code: &[u8], mut at: usize) -> bool {
    let mut captured = false;
    while at < code.len() {
        let decoded = decode(code, at);
        if is_flag_operation(&decoded) {
            captured = true;
            at += decoded.length;
            continue;
        }
        return captured && decoded.branch.is_some_and(|(_, conditional)| !conditional);
    }
    false
}

/// The cost of `code`'s fallthrough path from its start to the first of
/// `exits` (offsets), a RET, a jump back or its end. A forward jump is
/// followed: it skips moved code. So is a conditional jump over a side
/// exit's capture ([`is_capture_stub`]), the guest's fallthrough.
pub(super) fn fallthrough(code: &[u8], exits: &[usize]) -> Cost {
    let mut cost = Cost::default();
    let mut at = 0;
    while at < code.len() && !exits.contains(&at) {
        let decoded = decode(code, at);
        cost.instructions += 1;
        let memory_operand = decoded.memory.is_some()
            && !matches!(
                (decoded.map, decoded.opcode),
                (0, 0x8d) | (1, 0x0d | 0x18 | 0x1f)
            );
        cost.memory += usize::from(memory_operand);
        cost.flags += usize::from(is_flag_operation(&decoded));
        let next = at + decoded.length;
        match decoded.branch {
            Some((target, conditional)) => {
                cost.branches += 1;
                if conditional && is_capture_stub(code, next) {
                    at = target;
                } else if conditional {
                    at = next;
                } else if target > at {
                    at = target;
                } else {
                    break;
                }
            }
            None if (decoded.map, decoded.opcode) == (0, 0xc3) => break,
            None => at = next,
        }
    }
    cost
}

/// The options the system-mode engine compiles EL0 blocks with, on
/// `target`, without self-loops.
pub(super) fn options(target: TargetCpu) -> Options {
    let checks = MemoryChecks::system(true, false, true, true)
        .with_range(RangeCheck::FIVE_LEVEL.one_sided(true));
    let native = NativeSystem {
        exclusive: true,
        ..NativeSystem::NONE
    };
    Options::new(BlockProfile::new(Features::SYSTEM_MODE), target)
        .with_memory_checks(checks)
        .with_native_system(native)
}

/// The block that `words`, placed at 0x1000, start, compiled for x86-64-v3,
/// and its fallthrough path's cost.
pub(super) fn snippet(words: &[u32]) -> (crate::block::CompiledBlock, Cost) {
    let source: Vec<u8> = words.iter().flat_map(|word| word.to_le_bytes()).collect();
    let compiled = CompilationPlan::new(
        &source,
        GuestAddress(0x1000),
        &options(TargetCpu::X86_64_V3),
    )
    .and_then(|plan| plan.compile())
    .unwrap_or_else(|error| panic!("{words:08x?}: {error:?}"));
    let exits: Vec<usize> = compiled
        .direct_exits
        .iter()
        .map(|exit| exit.offset)
        .collect();
    let cost = fallthrough(&compiled.native.x86_64_code, &exits);
    (compiled, cost)
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The decoder takes the lengths of the forms the capture, reload and
    /// exits use.
    #[test]
    fn decodes_the_flag_forms_and_exits() {
        // cmp rsi,rax; cmc; lahf; seto al; mov [rdi+0x100],ax;
        // movzx eax,word [rdi+0x100]; add al,0x7f; sahf; jl +0; jmp +0;
        // test byte [rdi+0x101],0x40; ret
        let code = [
            0x48, 0x39, 0xc6, 0xf5, 0x9f, 0x0f, 0x90, 0xc0, 0x66, 0x89, 0x87, 0x00, 0x01, 0x00,
            0x00, 0x0f, 0xb7, 0x87, 0x00, 0x01, 0x00, 0x00, 0x04, 0x7f, 0x9e, 0x0f, 0x8c, 0x00,
            0x00, 0x00, 0x00, 0xe9, 0x00, 0x00, 0x00, 0x00, 0xf6, 0x87, 0x01, 0x01, 0x00, 0x00,
            0x40, 0xc3,
        ];
        let cost = fallthrough(&code, &[]);
        assert_eq!(
            cost,
            Cost {
                instructions: 12,
                memory: 3,
                flags: 8,
                branches: 2,
            }
        );
    }
}
