//! NZCV that pinned code leaves in the host flags between a setter and its
//! readers ([`super::emit_selected`]): every condition after every setter
//! over the carry and overflow edges, in both widths, against the frame
//! emitter, which stores NZCV at each setter; and the word a fault, an
//! exit and a frame-bound emitter find.
use super::{emit_for_target, emit_system_mode};
use crate::{
    GuestAddress, GuestState, Lifter, TargetCpu,
    aarch64::{Decoder, Instruction},
    state::Limit,
    x86_64::{MemoryChecks, NativeSystem, flag_word, native, pinned},
};
use alloc::{vec, vec::Vec};

const CMP_X0_X1: u32 = 0xeb01_001f;
const CMP_W0_W1: u32 = 0x6b01_001f;
const CMP_X0_1: u32 = 0xf100_041f;
const CMP_W0_1: u32 = 0x7100_041f;
const CMN_X0_X1: u32 = 0xab01_001f;
const CMN_W0_1: u32 = 0x3100_041f;
const SUBS_X2: u32 = 0xeb01_0002;
const SUBS_W2: u32 = 0x6b01_0002;
const ADDS_X2: u32 = 0xab01_0002;
const ADDS_W2: u32 = 0x2b01_0002;
const ANDS_X2: u32 = 0xea01_0002;
const ANDS_W2_FF: u32 = 0x7200_1c02;
const TST_X0_X1: u32 = 0xea01_001f;
const BICS_W2: u32 = 0x6a21_0002;
/// CMP X0, W1, UXTW: the extended form.
const CMP_X0_W1_UXTW: u32 = 0xeb21_401f;
const ADDS_X2_W1_SXTW: u32 = 0xab21_c002;

const SETTERS: [u32; 16] = [
    CMP_X0_X1,
    CMP_W0_W1,
    CMP_X0_1,
    CMP_W0_1,
    CMN_X0_X1,
    CMN_W0_1,
    SUBS_X2,
    SUBS_W2,
    ADDS_X2,
    ADDS_W2,
    ANDS_X2,
    ANDS_W2_FF,
    TST_X0_X1,
    BICS_W2,
    CMP_X0_W1_UXTW,
    ADDS_X2_W1_SXTW,
];

const ADD_X3: u32 = 0x9100_0463;
const ADDS_X4: u32 = 0xab01_0004;
const CSEL_X5: u32 = 0x9a81_0005;
const CSINC_W6: u32 = 0x1a80_0426;
const CSINV_X7: u32 = 0xda81_0007;
const CSET_X8: u32 = 0x9a9f_07e8;
const CSETM_X9: u32 = 0xda9f_03e9;
const CSNEG_X10: u32 = 0xda81_040a;
const LDR_X11_X12: u32 = 0xf940_018b;
const MRS_X13_NZCV: u32 = 0xd53b_420d;
const MSR_NZCV_X14: u32 = 0xd51b_420e;

/// The operands: zero, one, the signed and unsigned edges of both widths.
const VALUES: [u64; 10] = [
    0,
    1,
    0x7fff_ffff,
    0x8000_0000,
    0xffff_ffff,
    0x1_0000_0000,
    0x7fff_ffff_ffff_ffff,
    0x8000_0000_0000_0000,
    0xffff_ffff_ffff_fffe,
    u64::MAX,
];

/// B.cond to `offset` bytes from itself.
const fn b_cond(condition: u32, offset: i32) -> u32 {
    0x5400_0000 | ((offset >> 2) as u32 & 0x7_ffff) << 5 | condition
}

/// `word` with its condition field (bits 15:12) set to `condition`.
const fn with_condition(word: u32, condition: u32) -> u32 {
    word & !(0xf << 12) | condition << 12
}

fn lift(words: &[u32]) -> Vec<Instruction> {
    words
        .iter()
        .enumerate()
        .map(|(index, &word)| {
            Decoder
                .lift(GuestAddress(0x1000 + 4 * index as u64), word)
                .unwrap_or_else(|error| panic!("{word:08x}: {error:?}"))
        })
        .collect()
}

/// The pinned system-mode block of `words` and the frame emitter's.
fn blocks(words: &[u32], target: TargetCpu) -> (Vec<u8>, Vec<u8>) {
    let instructions = lift(words);
    let end = GuestAddress(0x1000 + 4 * words.len() as u64);
    let (system, _) = emit_system_mode(
        &instructions,
        None,
        end,
        target,
        MemoryChecks::DEFAULT,
        NativeSystem::NONE,
    );
    let frame = emit_for_target(&instructions, None, end, target, MemoryChecks::DEFAULT);
    (pinned::entered(&system.x86_64_code), frame.x86_64_code)
}

/// Run both blocks of `words` from states that hold each pair of `VALUES`
/// in X0 and X1 and assert that they leave the same state and exit.
fn agree(words: &[u32], target: TargetCpu, memory: &mut [u64; 2]) -> usize {
    let (system, frame) = blocks(words, target);
    let mut runs = 0;
    for (index, &left) in VALUES.iter().enumerate() {
        for &right in &VALUES {
            let mut expected = GuestState {
                nzcv: ((index as u32 * 7 + runs as u32) % 16) << 28,
                lower_limit: Limit(u64::MAX),
                ..GuestState::default()
            };
            for (register, x) in expected.x.iter_mut().enumerate() {
                *x = 0x1111 * register as u64;
            }
            expected.x[0] = left;
            expected.x[1] = right;
            expected.x[12] = memory.as_mut_ptr() as u64;
            expected.x[14] = u64::from(expected.nzcv ^ 0xa000_0000);
            let mut actual = expected.clone();
            let frame_exit = native::run(&frame, &mut expected);
            let system_exit = native::run(&system, &mut actual);
            assert_eq!(
                (system_exit, &actual),
                (frame_exit, &expected),
                "{words:08x?} {left:#x} {right:#x}"
            );
            runs += 1;
        }
    }
    runs
}

fn targets() -> Vec<TargetCpu> {
    let host = TargetCpu::detect();
    [TargetCpu::Baseline, TargetCpu::X86_64_V3, host]
        .into_iter()
        .filter(|target| host.supports(*target))
        .collect()
}

/// Every condition after every setter: by a side exit, a select, a second
/// select after HI or LS turned the carry over, a CSET and the word the
/// block's end stores, against the frame emitter.
#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn every_condition_after_every_setter_matches_the_frame_emitter() {
    let mut memory = [0x1234_5678_9abc_def0, 0];
    let mut runs = 0;
    for target in targets() {
        for setter in SETTERS {
            for condition in 0..14 {
                let other = (condition + 5) % 14;
                let shapes: [Vec<u32>; 4] = [
                    vec![setter, b_cond(condition, 0x40), ADD_X3],
                    vec![
                        setter,
                        with_condition(CSEL_X5, condition),
                        with_condition(CSINC_W6, other),
                        with_condition(CSINV_X7, condition ^ 1),
                        b_cond(other, 0x40),
                        with_condition(CSET_X8, condition),
                    ],
                    vec![
                        setter,
                        b_cond(condition, 0x40),
                        with_condition(CSETM_X9, other),
                        b_cond(other, 0x40),
                        with_condition(CSNEG_X10, condition),
                    ],
                    vec![
                        setter,
                        b_cond(condition, 0x40),
                        ADDS_X4,
                        b_cond(other, 0x40),
                    ],
                ];
                for words in shapes {
                    runs += agree(&words, target, &mut memory);
                }
            }
        }
    }
    assert!(runs > 30_000, "{runs}");
}

/// The word between a setter and its reader where something else comes
/// first: an ADD, a load, MRS and MSR of NZCV (frame-bound), and a reader
/// that a dead setter follows.
#[cfg(all(target_arch = "x86_64", any(target_os = "macos", target_os = "linux")))]
#[test]
fn the_word_is_stored_before_what_may_need_it() {
    let mut memory = [0x8000_0000_0000_0000, 0];
    let mut runs = 0;
    for target in targets() {
        for setter in [CMP_X0_X1, ADDS_W2, ANDS_X2, CMP_X0_W1_UXTW] {
            for condition in [0, 2, 4, 6, 8, 9, 10, 13] {
                let shapes: [Vec<u32>; 5] = [
                    vec![setter, ADD_X3, b_cond(condition, 0x40)],
                    vec![setter, LDR_X11_X12, b_cond(condition, 0x40), ADD_X3],
                    vec![setter, MRS_X13_NZCV, b_cond(condition, 0x40)],
                    vec![setter, b_cond(condition, 0x40), MSR_NZCV_X14, ADD_X3],
                    vec![
                        setter,
                        with_condition(CSEL_X5, condition),
                        setter ^ 0x0001_0000,
                        b_cond(condition ^ 1, 0x40),
                    ],
                ];
                for words in shapes {
                    runs += agree(&words, target, &mut memory);
                }
            }
        }
    }
    assert!(runs > 3_000, "{runs}");
}

/// A load that faults after a setter, before its reader: the word at the
/// fault is the setter's NZCV, whether a reader or another setter follows,
/// so the engine's settlement finds it.
#[cfg(all(target_arch = "x86_64", target_os = "linux"))]
#[test]
fn a_fault_after_a_setter_finds_its_nzcv() {
    use super::vector_cache_tests::faulting;
    let guard = faulting::guard_page();
    let mut faults = 0;
    for target in targets() {
        let setters: [(u32, Model); 2] = [(CMP_X0_X1, nzcv_of_subtract), (CMN_X0_X1, nzcv_of_add)];
        for (setter, flags) in setters {
            for after in [b_cond(0, 0x40), ADDS_X4, with_condition(CSEL_X5, 8)] {
                let words = [setter, LDR_X11_X12, after, ADD_X3];
                let instructions = lift(&words);
                let end = GuestAddress(0x1000 + 4 * words.len() as u64);
                let (system, _) = emit_system_mode(
                    &instructions,
                    None,
                    end,
                    target,
                    MemoryChecks::DEFAULT,
                    NativeSystem::NONE,
                );
                let code = pinned::entered(&system.x86_64_code);
                for &left in &VALUES {
                    for &right in &VALUES {
                        let mut state = GuestState {
                            nzcv: 0x5000_0000,
                            lower_limit: Limit(u64::MAX),
                            ..GuestState::default()
                        };
                        state.x[0] = left;
                        state.x[1] = right;
                        state.x[12] = guard;
                        assert!(
                            faulting::run(&code, &mut state).is_err(),
                            "{words:08x?} does not fault"
                        );
                        assert_eq!(
                            flag_word::arm(state.nzcv),
                            flags(left, right),
                            "{words:08x?} {left:#x} {right:#x}"
                        );
                        faults += 1;
                    }
                }
            }
        }
    }
    assert!(faults >= 600, "{faults}");
}

/// The Arm NZCV of a setter from its operands.
type Model = fn(u64, u64) -> u32;

/// Arm NZCV of the 64-bit subtraction `left - right`.
fn nzcv_of_subtract(left: u64, right: u64) -> u32 {
    let (result, borrow) = left.overflowing_sub(right);
    let overflow = (left as i64).overflowing_sub(right as i64).1;
    nzcv(result, !borrow, overflow)
}

/// Arm NZCV of the 64-bit addition `left + right`.
fn nzcv_of_add(left: u64, right: u64) -> u32 {
    let (result, carry) = left.overflowing_add(right);
    let overflow = (left as i64).overflowing_add(right as i64).1;
    nzcv(result, carry, overflow)
}

fn nzcv(result: u64, carry: bool, overflow: bool) -> u32 {
    u32::from(result >> 63 != 0) << 31
        | u32::from(result == 0) << 30
        | u32::from(carry) << 29
        | u32::from(overflow) << 28
}
