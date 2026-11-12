//! Fused multiply-add: FMA3 when available, otherwise an exact-product SSE2 path.
use super::super::isa::ShiftOpcode;
use super::numerical::{
    ArithmeticOpcode, ColdSplit, Condition, Entry, FPSR_UNDERFLOW, FusedOpcode,
    INPUT_MINIMUM_EXPONENT, Input, MXCSR_UNDERFLOW, NanRegisters, RESULT_MINIMUM_EXPONENT,
    Rounding, ZeroProduct, active_lanes, branch, capture_environment_status,
    clear_environment_status, clear_scalar, commit_flush_status, commit_inexact,
    commit_invalid_lanes, commit_rounded_status, emit_fma, emit_opcode, environment,
    environment_directed, float_lanes, flush_subnormal_lanes, jump, merge_nans,
    minimum_normal_lanes, prepare_vector_input_range, prepare_vector_result_range, quiet_lanes,
    read_inputs, read_numerical_right, read_numerical_vector, reject_exceptional_lanes,
    reject_other_status, reject_product_status, resolve, restore_environment,
    restore_environment_with_product, save_product_status, scalar_ordered_range, select_input_nans,
    small_lanes, special_lanes, sticky_inexact, sticky_result_check, subnormal_lanes, test_control,
    vector_ordered_range, zero_product_check,
};
use super::{Host, commit, packed};
use crate::x86_64::asm::{Alu, Code, FloatOp, Gpr, Mem, Rm, Size};
use crate::{
    GuestAddress, TargetCpu,
    aarch64::{
        floating::{FPCR_AH, FPCR_EXCEPTION_ENABLE_MASK, FPCR_FIZ, FPCR_FZ, FPCR_NEP, Format},
        simd::{arithmetic::Shape, fused::Operand},
    },
    block::ExitKind,
};
use alloc::vec;
use alloc::vec::Vec;
mod half;
#[cfg(all(
    test,
    target_arch = "x86_64",
    any(target_os = "macos", target_os = "linux")
))]
mod tests;
mod wide_half;

const LEFT: u8 = 0;
const RIGHT: u8 = 1;
const SIGN_MASK: u8 = 2;
const ADDEND: u8 = 6;
const ORIGINAL_ADDEND: u8 = 7;
/// The zero classifier's scratch: the scalar-only original addend's register.
const ZERO_RANGE_SCRATCH: u8 = ORIGINAL_ADDEND;
/// [`flush_subnormal_lanes`]' scratch, free before the input guards.
const FLUSH_SCRATCH: [u8; 4] = [2, 3, 4, 5];
/// [`select_input_nans`]' registers, which alias no source.
const NAN_REGISTERS: NanRegisters = NanRegisters {
    selected: 2,
    inputs: 3,
    invalid: 4,
    mask: 5,
    temporary: 7,
};
/// [`minimum_normal_lanes`]' scratch after [`merge_nans`].
const MINIMUM_SCRATCH: [u8; 2] = [2, 3];
/// [`small_lanes`]' scratch after [`merge_nans`], leaving its lanes where
/// [`minimum_normal_lanes`] does.
const SMALL_SCRATCH: [u8; 3] = [2, 3, 5];
/// The unordered compare predicate.
const UNORDERED: u8 = 3;
/// PSHUFD selector moving the high qword to the low.
const HIGH_QWORD: u8 = 0xee;
/// [`operand_special_lanes`]' result.
const SPECIAL_LANES: u8 = 2;

/// Flip the sign of the multiplicand and addend that FMLS, FMSUB, FNMADD
/// and FNMSUB negate (Arm's FPNeg, which flips a NaN's sign too); a
/// second call restores them. Clobbers NAN_REGISTERS.temporary.
fn negate_operands(code: &mut Code, operand: &Operand) {
    let negated = [
        (operand.operation.negate_left(), LEFT),
        (operand.operation.negate_addend(), ADDEND),
    ];
    if !negated.iter().any(|&(negate, _)| negate) {
        return;
    }
    let sign = NAN_REGISTERS.temporary;
    packed(code, Host::EqualDword, sign, sign);
    let shift = if operand.format == Format::Single {
        ShiftOpcode::LeftDword
    } else {
        ShiftOpcode::LeftQword
    };
    shift.emit(code, sign, (operand.format.bits() - 1) as u8);
    for (negate, source) in negated {
        if negate {
            packed(code, Host::Xor, source, sign);
        }
    }
}

fn emit_checked_fma(code: &mut Code, operand: &Operand, target: TargetCpu) {
    emit_fma(
        code,
        operand.format,
        operand.shape,
        FusedOpcode::from(operand.operation),
        target,
    );
}

fn prepare_right(code: &mut Code, operand: &Operand) {
    read_numerical_right(
        code,
        operand.format,
        operand.shape,
        operand.right,
        operand.right_lane,
    );
    if operand.shape == Shape::Vector64 && operand.right_lane.is_some() {
        // Inactive lanes multiply zeros, so an infinite element raises no
        // host invalid there.
        packed(code, Host::MoveQword, RIGHT, RIGHT);
    }
}

fn prepare_negation(code: &mut Code, operand: &Operand) -> u8 {
    let merge = if operand.shape == Shape::Scalar && operand.operation.negate_addend() {
        packed(code, Host::MoveAligned, ORIGINAL_ADDEND, ADDEND);
        ORIGINAL_ADDEND
    } else {
        ADDEND
    };
    if operand.operation.negate_addend() || operand.operation.negate_left() {
        packed(code, Host::EqualDword, SIGN_MASK, SIGN_MASK);
        let shift = if operand.format == Format::Single {
            ShiftOpcode::LeftDword
        } else {
            ShiftOpcode::LeftQword
        };
        shift.emit(code, SIGN_MASK, (operand.format.bits() - 1) as u8);
        if operand.operation.negate_left() {
            packed(code, Host::Xor, LEFT, SIGN_MASK);
        }
        if operand.operation.negate_addend() {
            packed(code, Host::Xor, ADDEND, SIGN_MASK);
        }
    }
    merge
}

/// With FMA3, S/D forms return their [`ColdSplit`].
pub(in crate::x86_64) fn emit(
    code: &mut Code,
    operand: &Operand,
    pc: GuestAddress,
    target: TargetCpu,
    entry: Entry,
) -> Option<ColdSplit> {
    if operand.format == Format::Half {
        half::emit(code, operand, pc, target);
        return None;
    }
    let native_fma = target.supports(TargetCpu::Fma);
    let (mut helper, sticky) = if native_fma {
        let StickyPath {
            full,
            helper,
            done,
            mut crossing,
            cold,
            inline_end,
        } = sticky_path(code, operand, target, entry);
        let full_start = code.len();
        for &patch in &full {
            resolve(code, patch, full_start);
        }
        crossing.extend(full);
        (helper, Some((done, full_start, crossing, cold, inline_end)))
    } else {
        (Vec::new(), None)
    };
    test_control(code, FPCR_EXCEPTION_ENABLE_MASK);
    helper.push(branch(code, Condition::NotZero));
    read_numerical_vector(code, operand.shape, operand.left, LEFT);
    read_numerical_vector(code, operand.shape, operand.addend, ADDEND);
    prepare_right(code, operand);
    // FZ or FIZ with AH clear flush subnormal inputs to their signed zeros
    // (FZ raising IDC, committed with the result); AH=1 keeps the helper.
    code.asm().alu(Alu::Xor, Size::Dword, Gpr::R11, Gpr::R11);
    if !native_fma {
        code.asm().alu(Alu::Xor, Size::Dword, Gpr::R10, Gpr::R10);
    }
    // With FMA3 and FZ, FIZ and AH clear, subnormal inputs and tiny,
    // overflowing and infinite results are exact: the host's single
    // unflushed rounding in Arm's mode is FPMulAdd's, and its status FPSR's
    // ([`commit_rounded_status`]). NaN lanes follow FPProcessNaNs3 over
    // the addend and the multiplicands, after FMLS, FMSUB and FNMADD negate
    // theirs ([`select_input_nans`]), and an invalid operation gives the
    // default NaN with IOC ([`merge_nans`]), also beside a quiet NaN
    // addend, where Arm does not propagate the addend. A minimum normal
    // result takes the helper. Without FMA3, lanes with an infinite or NaN
    // operand (AH clear) complete apart from the exact-product path's
    // ([`clear_special_lanes`], [`special_results`]).
    test_control(code, FPCR_AH);
    let alternative = branch(code, Condition::NotZero);
    test_control(code, FPCR_FZ | FPCR_FIZ);
    let unflushed = branch(code, Condition::Zero);
    for source in [LEFT, RIGHT, ADDEND] {
        flush_subnormal_lanes(code, operand.format, operand.shape, source, FLUSH_SCRATCH);
    }
    let flushed = code.len();
    resolve(code, unflushed, flushed);
    let rounded = native_fma.then(|| {
        environment(code);
        negate_operands(code, operand);
        select_input_nans(
            code,
            operand.format,
            operand.shape,
            &[ADDEND, LEFT, RIGHT],
            NAN_REGISTERS,
        );
        invalid_products_beside_quiet_nans(code, operand.format);
        code.asm().alu(Alu::Xor, Size::Dword, Gpr::Rax, Gpr::Rax);
        // The product's status is not the guest's.
        clear_environment_status(code);
        negate_operands(code, operand);
        // The original Vn, for FZ's flush.
        code.asm().movq_from_xmm(Gpr::R8, LEFT);
        super::shuffle(code, NAN_REGISTERS.temporary, LEFT, HIGH_QWORD);
        code.asm().movq_from_xmm(Gpr::R9, NAN_REGISTERS.temporary);
        emit_checked_fma(code, operand, target);
        merge_nans(code, operand.format, operand.shape, LEFT, NAN_REGISTERS);
        // Under FZ, a result Arm flushes completes inline.
        test_control(code, FPCR_FZ);
        let unflushing = branch(code, Condition::Zero);
        flush_tiny_fused_results(code, operand, target);
        let flushed = jump(code);
        let minimum = code.len();
        resolve(code, unflushing, minimum);
        minimum_normal_lanes(code, operand.format, LEFT, MINIMUM_SCRATCH);
        Host::DwordSignMask.register(
            code,
            Gpr::Rcx as u8,
            MINIMUM_SCRATCH[0],
            TargetCpu::Baseline,
        );
        code.asm().alu(Alu::Or, Size::Dword, Gpr::Rax, Gpr::Rcx);
        let classified = code.len();
        resolve(code, flushed, classified);
        restore_environment(code);
        helper.push(reject_exceptional_lanes(
            code,
            operand.format,
            operand.shape,
        ));
        commit_flush_status(code);
        commit_invalid_lanes(code, operand.format, operand.shape, NAN_REGISTERS.invalid);
        commit_rounded_status(code);
        jump(code)
    });
    if !native_fma {
        clear_special_lanes(code, operand);
    }
    let strict = code.len();
    resolve(code, alternative, strict);
    if operand.shape != Shape::Scalar {
        prepare_vector_input_range(code, operand.format);
    }
    // A zero input or lane is exact on both sides: FPMulAdd and the host's
    // FMA follow IEEE 754's signed-zero sums under the installed rounding
    // (an exact zero sum is +0, -0 rounding toward minus infinity, and a sum
    // of two zeros of one sign keeps it), a zero product adds exactly, and
    // a zero is not subnormal, so FZ and FIZ flush nothing, and neither side
    // raises status for it. An infinite input is exact too: its result is an
    // infinity in every rounding mode, without status, unless it meets a zero
    // factor or an opposite infinity, which raise the host's invalid (the
    // separate product's too) and go to the helper; a finite product that
    // overflowed raised overflow. The SSE2 product's own status rejects any
    // rounding before the sum.
    for source in [LEFT, RIGHT, ADDEND] {
        if operand.shape == Shape::Scalar {
            helper.push(scalar_ordered_range(
                code,
                operand.format,
                source,
                INPUT_MINIMUM_EXPONENT,
            ));
        } else {
            vector_ordered_range(
                code,
                operand.format,
                operand.shape,
                source,
                ZERO_RANGE_SCRATCH,
            );
            helper.push(branch(code, Condition::NotZero));
        }
    }
    let merge = if native_fma {
        ADDEND
    } else {
        prepare_negation(code, operand)
    };
    environment(code);
    if native_fma {
        emit_checked_fma(code, operand, target);
        restore_environment(code);
    } else {
        emit_opcode(
            code,
            operand.format,
            operand.shape,
            ArithmeticOpcode::Multiply,
            RIGHT,
        );
        save_product_status(code);
        emit_opcode(
            code,
            operand.format,
            operand.shape,
            ArithmeticOpcode::Add,
            ADDEND,
        );
        restore_environment_with_product(code);
        // All host status and stack state are restored before either side exit.
        helper.push(reject_product_status(code));
    }
    helper.push(reject_other_status(code));
    if !native_fma {
        flush_exact_tiny_results(code, operand);
    }
    let success = if operand.shape == Shape::Scalar {
        // An exact zero result raised no status; a nonzero one rounded to
        // zero raised underflow, which went to the helper above.
        helper.push(scalar_ordered_range(
            code,
            operand.format,
            LEFT,
            RESULT_MINIMUM_EXPONENT,
        ));
        jump(code)
    } else {
        prepare_vector_result_range(code);
        // A zero result is exact: one that underflowed raised the host's
        // underflow status, which went to the helper above.
        vector_ordered_range(
            code,
            operand.format,
            operand.shape,
            LEFT,
            ZERO_RANGE_SCRATCH,
        );
        branch(code, Condition::Zero)
    };
    let helper_start = code.len();
    super::super::block::direct_exit(code, pc, ExitKind::FloatingPoint);
    let success_start = code.len();
    for patch in helper {
        resolve(code, patch, helper_start);
    }
    resolve(code, success, success_start);
    commit_inexact(code);
    commit_flush_status(code);
    if !native_fma {
        special_results(code, operand, merge);
    }
    if let Some(rounded) = rounded {
        let tail = code.len();
        resolve(code, rounded, tail);
    }
    if operand.shape == Shape::Scalar {
        // Register MOVSS/MOVSD replaces only the result lane of A.
        if operand.format == Format::Single {
            Host::MoveScalarSingle
        } else {
            Host::MoveScalarDouble
        }
        .register(code, merge, LEFT, crate::TargetCpu::Baseline);
        test_control(code, FPCR_NEP);
        let merging = branch(code, Condition::NotZero);
        clear_scalar(code, operand.format, merge);
        let commit_start = code.len();
        resolve(code, merging, commit_start);
        commit(code, operand.destination, merge);
        // XMM0 holds the destination wherever the emission ends.
        packed(code, Host::MoveAligned, LEFT, merge);
    } else {
        if operand.shape == Shape::Vector64 {
            packed(code, Host::MoveQword, LEFT, LEFT);
        }
        commit(code, operand.destination, LEFT);
    }
    let (done, full, crossing, cold, inline_end) = sticky?;
    let end = code.len();
    resolve(code, done, end);
    Some(ColdSplit {
        cold,
        inline_end,
        full,
        crossing,
        result: operand.destination,
    })
}

/// With FMA3, FPCR.FZ set and AH clear, after [`merge_nans`] in the
/// private environment: Arm flushes a result whose exact value is nonzero
/// and below the minimum normal in magnitude to the zero of its sign, with
/// UFC alone. A lane can be tiny only if the host's result is nonzero and
/// at most the minimum normal, or it rounded to zero, raising the host's
/// underflow; then the operation runs again rounding up (U) and down (D)
/// from the original Vn, kept in R8 and R9: the exact value is tiny exactly
/// when the smaller of |U| and |D| is below the minimum normal and the
/// larger nonzero, and its sign is U's. Tiny lanes add UFC to R11D, take
/// that signed zero, and give up their host status: the operation runs a
/// last time in the control's rounding with their operands zero, for the
/// other lanes' status. Leaves the result in LEFT and EAX zero; clobbers
/// RIGHT, ADDEND's flushed lanes (a scalar's merge keeps its upper lanes),
/// XMM2, XMM3, XMM5, XMM7 and ECX.
fn flush_tiny_fused_results(code: &mut Code, operand: &Operand, target: TargetCpu) {
    const LESS: u8 = 1;
    const NOT_EQUAL: u8 = 4;
    const ORDERED: u8 = 7;
    const UP: u8 = 2;
    const DOWN: u8 = 3;
    const RESULT: u8 = 5;
    const VN: u8 = 7;
    let (format, shape) = (operand.format, operand.shape);
    let lanes = float_lanes(format, Shape::Vector128);
    let (left_shift, right_shift) = if format == Format::Double {
        (ShiftOpcode::LeftQword, ShiftOpcode::RightQword)
    } else {
        (ShiftOpcode::LeftDword, ShiftOpcode::RightDword)
    };
    let restore_vn = |code: &mut Code, xmm: u8, scratch: u8| {
        packed(code, Host::MoveGprQword, xmm, Gpr::R8 as u8);
        packed(code, Host::MoveGprQword, scratch, Gpr::R9 as u8);
        packed(code, Host::UnpackLowQword, xmm, scratch);
    };
    small_lanes(code, format, LEFT, SMALL_SCRATCH);
    active_lanes(code, shape, format, DOWN);
    packed(code, Host::And, SMALL_SCRATCH[0], DOWN);
    Host::DwordSignMask.register(code, Gpr::Rax as u8, SMALL_SCRATCH[0], TargetCpu::Baseline);
    capture_environment_status(code);
    code.asm()
        .alu_imm32(Alu::And, Size::Dword, Gpr::Rcx, MXCSR_UNDERFLOW as i32);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::Rax, Gpr::Rcx);
    let exact = branch(code, Condition::Zero);
    packed(code, Host::MoveAligned, RESULT, LEFT);
    for (rounding, destination) in [(Rounding::Positive, UP), (Rounding::Negative, DOWN)] {
        restore_vn(code, LEFT, destination);
        environment_directed(code, rounding);
        emit_checked_fma(code, operand, target);
        packed(code, Host::MoveAligned, destination, LEFT);
    }
    // LEFT: U's sign; VN: the magnitude mask.
    packed(code, Host::EqualDword, LEFT, LEFT);
    left_shift.emit(code, LEFT, (format.bits() - 1) as u8);
    packed(code, Host::And, LEFT, UP);
    packed(code, Host::EqualDword, VN, VN);
    right_shift.emit(code, VN, 1);
    packed(code, Host::And, UP, VN);
    packed(code, Host::And, DOWN, VN);
    packed(code, Host::MoveAligned, VN, UP);
    code.asm().float(FloatOp::Max, lanes, VN, Rm::Xmm(DOWN));
    code.asm().float(FloatOp::Min, lanes, UP, Rm::Xmm(DOWN));
    // UP: tiny lanes, ordered, active.
    code.asm().mov_imm64(Gpr::Rax, 1 << format.fraction_bits());
    packed(code, Host::MoveGprQword, DOWN, Gpr::Rax as u8);
    if format == Format::Single {
        super::shuffle(code, DOWN, DOWN, 0);
    } else {
        packed(code, Host::UnpackLowQword, DOWN, DOWN);
    }
    code.asm().float_compare(lanes, UP, Rm::Xmm(DOWN), LESS);
    packed(code, Host::Xor, DOWN, DOWN);
    code.asm()
        .float_compare(lanes, VN, Rm::Xmm(DOWN), NOT_EQUAL);
    packed(code, Host::And, UP, VN);
    packed(code, Host::MoveAligned, DOWN, RESULT);
    code.asm()
        .float_compare(lanes, DOWN, Rm::Xmm(DOWN), ORDERED);
    packed(code, Host::And, UP, DOWN);
    active_lanes(code, shape, format, DOWN);
    packed(code, Host::And, UP, DOWN);
    // DOWN: the result, the tiny lanes' signed zeros.
    packed(code, Host::And, LEFT, UP);
    packed(code, Host::MoveAligned, DOWN, UP);
    packed(code, Host::AndNot, DOWN, RESULT);
    packed(code, Host::Or, DOWN, LEFT);
    Host::DwordSignMask.register(code, Gpr::Rax as u8, UP, TargetCpu::Baseline);
    code.asm().test(Size::Dword, Gpr::Rax, Gpr::Rax);
    let kept = branch(code, Condition::Zero);
    code.asm()
        .alu_imm8(Alu::Or, Size::Dword, Gpr::R11, FPSR_UNDERFLOW as i8);
    let flushed = code.len();
    resolve(code, kept, flushed);
    restore_vn(code, LEFT, RESULT);
    for source in [LEFT, RIGHT, ADDEND] {
        packed(code, Host::MoveAligned, RESULT, UP);
        packed(code, Host::AndNot, RESULT, source);
        packed(code, Host::MoveAligned, source, RESULT);
    }
    clear_environment_status(code);
    emit_checked_fma(code, operand, target);
    packed(code, Host::MoveAligned, LEFT, DOWN);
    let end = code.len();
    resolve(code, exact, end);
    code.asm().alu(Alu::Xor, Size::Dword, Gpr::Rax, Gpr::Rax);
}

/// Without FMA3, with FPCR.FZ set and AH clear, after the exact-product
/// path: its sum of an exact product is exact when tiny, so the tiny
/// lanes are its subnormal results, which raised no host status; Arm
/// flushes them to the zero of their sign with UFC, in R11D. Clobbers
/// RIGHT, XMM2, XMM3 and EAX.
fn flush_exact_tiny_results(code: &mut Code, operand: &Operand) {
    const TINY: u8 = 2;
    const SCRATCH: u8 = 3;
    let (format, shape) = (operand.format, operand.shape);
    test_control(code, FPCR_FZ);
    let unflushing = branch(code, Condition::Zero);
    test_control(code, FPCR_AH);
    let alternative = branch(code, Condition::NotZero);
    subnormal_lanes(code, format, TINY, LEFT, [SCRATCH, RIGHT]);
    active_lanes(code, shape, format, SCRATCH);
    packed(code, Host::And, TINY, SCRATCH);
    Host::DwordSignMask.register(code, Gpr::Rax as u8, TINY, TargetCpu::Baseline);
    code.asm().test(Size::Dword, Gpr::Rax, Gpr::Rax);
    let none = branch(code, Condition::Zero);
    code.asm()
        .alu_imm8(Alu::Or, Size::Dword, Gpr::R11, FPSR_UNDERFLOW as i8);
    // Clear the tiny lanes' magnitudes.
    packed(code, Host::EqualDword, SCRATCH, SCRATCH);
    if format == Format::Double {
        ShiftOpcode::RightQword.emit(code, SCRATCH, 1);
    } else {
        ShiftOpcode::RightDword.emit(code, SCRATCH, 1);
    }
    packed(code, Host::And, SCRATCH, TINY);
    packed(code, Host::AndNot, SCRATCH, LEFT);
    packed(code, Host::MoveAligned, LEFT, SCRATCH);
    let end = code.len();
    for patch in [unflushing, alternative, none] {
        resolve(code, patch, end);
    }
}

/// After [`select_input_nans`] over the negated operands, in the private
/// environment: Arm gives an infinity times a zero beside a quiet NaN
/// addend the default NaN with IOC, not the addend. Those lanes' addends
/// become zeros and leave the NaN-input lanes, so that the host's invalid
/// product gives [`merge_nans`]' default NaN with IOC. A NaN factor's lane
/// keeps FPProcessNaNs3's selection. Clobbers NAN_REGISTERS.mask and
/// NAN_REGISTERS.temporary.
fn invalid_products_beside_quiet_nans(code: &mut Code, format: Format) {
    let [product, quiet] = [NAN_REGISTERS.mask, NAN_REGISTERS.temporary];
    let lanes = float_lanes(format, Shape::Vector128);
    packed(code, Host::MoveAligned, product, LEFT);
    code.asm()
        .float(FloatOp::Mul, lanes, product, Rm::Xmm(RIGHT));
    code.asm()
        .float_compare(lanes, product, Rm::Xmm(product), UNORDERED);
    quiet_lanes(code, format, quiet, ADDEND);
    packed(code, Host::And, product, quiet);
    packed(code, Host::MoveAligned, quiet, ADDEND);
    code.asm()
        .float_compare(lanes, quiet, Rm::Xmm(quiet), UNORDERED);
    packed(code, Host::And, product, quiet);
    // Not a NaN factor's product: an infinity times a zero.
    for factor in [LEFT, RIGHT] {
        packed(code, Host::MoveAligned, quiet, factor);
        code.asm()
            .float_compare(lanes, quiet, Rm::Xmm(quiet), UNORDERED);
        packed(code, Host::AndNot, quiet, product);
        packed(code, Host::MoveAligned, product, quiet);
    }
    let inputs = NAN_REGISTERS.inputs;
    packed(code, Host::And, product, inputs);
    packed(code, Host::MoveAligned, quiet, product);
    packed(code, Host::AndNot, quiet, ADDEND);
    packed(code, Host::MoveAligned, ADDEND, quiet);
    packed(code, Host::AndNot, product, inputs);
    packed(code, Host::MoveAligned, inputs, product);
}

/// The exact-product path's lanes with an infinite or NaN operand, in
/// SPECIAL_LANES ([`special_lanes`]); clobbers XMM3, XMM4 and EAX.
fn operand_special_lanes(code: &mut Code, format: Format, shape: Shape) {
    const SCRATCH: u8 = 3;
    const OTHER: u8 = 4;
    special_lanes(code, format, SPECIAL_LANES, LEFT, SCRATCH);
    for source in [RIGHT, ADDEND] {
        special_lanes(code, format, OTHER, source, SCRATCH);
        packed(code, Host::Or, SPECIAL_LANES, OTHER);
    }
    active_lanes(code, shape, format, OTHER);
    packed(code, Host::And, SPECIAL_LANES, OTHER);
}

/// Without FMA3, with FPCR.AH clear, after the input flush: the operands
/// of each lane with an infinite or NaN operand become zeros, which the
/// exact-product path completes exactly and without status, and R10D
/// becomes nonzero for [`special_results`]. Clobbers XMM2 to XMM4 and EAX.
fn clear_special_lanes(code: &mut Code, operand: &Operand) {
    const KEPT: u8 = 3;
    operand_special_lanes(code, operand.format, operand.shape);
    Host::DwordSignMask.register(code, Gpr::Rax as u8, SPECIAL_LANES, TargetCpu::Baseline);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rax);
    code.asm().test(Size::Dword, Gpr::Rax, Gpr::Rax);
    let ordinary = branch(code, Condition::Zero);
    for source in [LEFT, RIGHT, ADDEND] {
        packed(code, Host::MoveAligned, KEPT, SPECIAL_LANES);
        packed(code, Host::AndNot, KEPT, source);
        packed(code, Host::MoveAligned, source, KEPT);
    }
    let end = code.len();
    resolve(code, ordinary, end);
}

/// Without FMA3, after the exact-product path completed the other lanes
/// into LEFT (and before the commit), when [`clear_special_lanes`] found
/// lanes with an infinite or NaN operand: their results, which round
/// nothing. From the operands read again (flushed again under FZ or FIZ,
/// their IDC committed already) and negated as the operation does, a
/// separate host product and sum give FPMulAdd's infinity or invalid
/// operation once a lane whose factors are both finite has a zero product
/// (its exact product cannot change an infinite or NaN addend, but its host
/// product could overflow). NaN lanes follow FPProcessNaNs3 and an invalid
/// operation gives the default NaN with IOC ([`select_input_nans`],
/// [`invalid_products_beside_quiet_nans`], [`merge_nans`]). The other
/// lanes add the exact-product result to a zero product of its sign, which
/// keeps it exactly. Only IOC reaches FPSR. Keeps `merge` and leaves the
/// result in LEFT; clobbers XMM1 to XMM7 but `merge`, RAX and RCX.
fn special_results(code: &mut Code, operand: &Operand, merge: u8) {
    const SAVED_BYTES: i8 = 32;
    const RESULT_SLOT: i32 = 0;
    const MERGE_SLOT: i32 = 16;
    const FACTORS: u8 = SPECIAL_LANES;
    const SCRATCH: u8 = 3;
    const OTHER: u8 = 4;
    const RESULT: u8 = 5;
    const SPECIAL: u8 = 7;
    let (format, shape) = (operand.format, operand.shape);
    code.asm().test(Size::Dword, Gpr::R10, Gpr::R10);
    let ordinary = branch(code, Condition::Zero);
    code.asm()
        .alu_imm8(Alu::Sub, Size::Qword, Gpr::Rsp, SAVED_BYTES);
    Host::StoreUnaligned.emit(
        code,
        LEFT,
        Mem::base(Gpr::Rsp, RESULT_SLOT),
        TargetCpu::Baseline,
    );
    Host::StoreUnaligned.emit(
        code,
        merge,
        Mem::base(Gpr::Rsp, MERGE_SLOT),
        TargetCpu::Baseline,
    );
    read_numerical_vector(code, shape, operand.left, LEFT);
    read_numerical_vector(code, shape, operand.addend, ADDEND);
    prepare_right(code, operand);
    test_control(code, FPCR_FZ | FPCR_FIZ);
    let unflushed = branch(code, Condition::Zero);
    for source in [LEFT, RIGHT, ADDEND] {
        flush_subnormal_lanes(code, format, shape, source, FLUSH_SCRATCH);
    }
    let flushed = code.len();
    resolve(code, unflushed, flushed);
    negate_operands(code, operand);
    special_lanes(code, format, FACTORS, LEFT, SCRATCH);
    special_lanes(code, format, OTHER, RIGHT, SCRATCH);
    packed(code, Host::Or, FACTORS, OTHER);
    special_lanes(code, format, SPECIAL, ADDEND, SCRATCH);
    packed(code, Host::Or, SPECIAL, FACTORS);
    Host::LoadUnaligned.emit(
        code,
        RESULT,
        Mem::base(Gpr::Rsp, RESULT_SLOT),
        TargetCpu::Baseline,
    );
    // Vn is zero where neither factor is special.
    packed(code, Host::And, LEFT, FACTORS);
    // Elsewhere Vm is the exact-product result's signed zero, and the
    // addend that result.
    packed(code, Host::EqualDword, SCRATCH, SCRATCH);
    if format == Format::Single {
        ShiftOpcode::LeftDword
    } else {
        ShiftOpcode::LeftQword
    }
    .emit(code, SCRATCH, (format.bits() - 1) as u8);
    packed(code, Host::And, SCRATCH, RESULT);
    for (source, ordinary_value) in [(RIGHT, SCRATCH), (ADDEND, RESULT)] {
        packed(code, Host::And, source, SPECIAL);
        packed(code, Host::MoveAligned, OTHER, SPECIAL);
        packed(code, Host::AndNot, OTHER, ordinary_value);
        packed(code, Host::Or, source, OTHER);
    }
    environment(code);
    select_input_nans(code, format, shape, &[ADDEND, LEFT, RIGHT], NAN_REGISTERS);
    invalid_products_beside_quiet_nans(code, format);
    emit_opcode(code, format, shape, ArithmeticOpcode::Multiply, RIGHT);
    emit_opcode(code, format, shape, ArithmeticOpcode::Add, ADDEND);
    merge_nans(code, format, shape, LEFT, NAN_REGISTERS);
    restore_environment(code);
    commit_invalid_lanes(code, format, shape, NAN_REGISTERS.invalid);
    Host::LoadUnaligned.emit(
        code,
        merge,
        Mem::base(Gpr::Rsp, MERGE_SLOT),
        TargetCpu::Baseline,
    );
    code.asm()
        .alu_imm8(Alu::Add, Size::Qword, Gpr::Rsp, SAVED_BYTES);
    let end = code.len();
    resolve(code, ordinary, end);
}

/// The sticky-inexact path (`numerical::sticky_inexact`) for a host with
/// FMA3, whose single rounding is the guest's: the operation in the entry
/// MXCSR, its result classification and the commit, which clears scalar
/// upper bits as NEP is zero there. A rejected zero takes the out-of-line
/// zero-product check, and other rejected results the full path. Returns the branches to the full path, to the helper
/// and past the full path, and those between the sticky path and the rest
/// ([`ColdSplit`]).
struct StickyPath {
    full: Vec<usize>,
    helper: Vec<usize>,
    done: usize,
    crossing: Vec<usize>,
    cold: crate::x86_64::asm::Checkpoint,
    inline_end: crate::x86_64::asm::Checkpoint,
}

fn sticky_path(code: &mut Code, operand: &Operand, target: TargetCpu, entry: Entry) -> StickyPath {
    let full = sticky_inexact(code, entry);
    // A zeroing load of the destination LEFT leaves a scalar result's
    // upper bits clear.
    let left = if operand.shape == Shape::Scalar {
        Input::Scalar(operand.format)
    } else {
        Input::Vector(operand.shape)
    };
    let vector = Input::Vector(operand.shape);
    let mut inputs = vec![(left, operand.left, LEFT), (vector, operand.addend, ADDEND)];
    if operand.right_lane.is_none() {
        inputs.push((vector, operand.right, RIGHT));
    }
    read_inputs(code, entry.xmm0, target, &inputs);
    if operand.right_lane.is_some() {
        prepare_right(code, operand);
    }
    emit_checked_fma(code, operand, target);
    let rejected = sticky_result_check(code, operand.format, operand.shape, LEFT, false, target);
    let resume = code.len();
    if operand.shape == Shape::Vector64 {
        packed(code, Host::MoveQword, LEFT, LEFT);
    }
    commit(code, operand.destination, LEFT);
    let inline_end = code.checkpoint();
    let done = jump(code);
    let cold = code.checkpoint();
    let check = code.len();
    resolve(code, rejected, check);
    let mut crossing = vec![rejected];
    // An infinite result may be exact: the full path tells, from its inputs
    // and the host's status, rather than the helper.
    let mut full = full;
    let unexplained = zero_product_check(
        code,
        &ZeroProduct {
            format: operand.format,
            shape: operand.shape,
            right: RIGHT,
            left: operand.left,
            addend: Some(ADDEND),
        },
        resume,
        &mut crossing,
    );
    if operand.operation.negate_left() || operand.operation.negate_addend() {
        full.extend(unexplained);
    } else {
        let special = code.len();
        for patch in unexplained {
            resolve(code, patch, special);
        }
        full.extend(propagated_specials(code, operand, resume, &mut crossing));
    }
    StickyPath {
        full,
        helper: vec![],
        done,
        crossing,
        cold,
        inline_end,
    }
}

/// Out of line after the sticky path's result check and its zero-product
/// check, for FMLA and FMADD (no operand negated) with FPCR at the sticky
/// defaults: return to `resume` when every active lane the checks rejected
/// propagates a quiet NaN or keeps an infinite operand's infinity, and each
/// other active lane is at least twice the minimum normal, which the
/// checks accept. Those lanes round nothing and raise no status: a NaN
/// lane takes FPProcessNaNs3's quiet NaN, the addend's, else Vn's, else
/// Vm's, and an infinite result with an infinite operand overflowed
/// nothing. A signalling NaN, an infinity times a zero, opposite
/// infinities, an infinity from finite operands, DN and the remaining tiny
/// lanes take the returned branches to the full path, which reads its
/// operands again. `resumes` receives the branch to `resume`. Reads the
/// operands again; clobbers XMM1 to XMM7, EAX, ECX, EDX and R8D to R11D,
/// and LEFT only on the way to `resume`.
fn propagated_specials(
    code: &mut Code,
    operand: &Operand,
    resume: usize,
    resumes: &mut Vec<usize>,
) -> Vec<usize> {
    use crate::aarch64::floating::FPCR_DN;
    use crate::x86_64::asm::ShiftOp;
    const LESS: u8 = 1;
    const VN: u8 = 2;
    const TEMPORARY: u8 = 3;
    const ACTIVE: u8 = 4;
    const OTHER: u8 = 5;
    /// MOVMSKPS bits of each D lane's high dword.
    const HIGH_DWORDS: i8 = 0b1010;
    let (format, shape) = (operand.format, operand.shape);
    let lanes = float_lanes(format, Shape::Vector128);
    let double = format == Format::Double;
    let (left_shift, right_shift) = if double {
        (ShiftOpcode::LeftQword, ShiftOpcode::RightQword)
    } else {
        (ShiftOpcode::LeftDword, ShiftOpcode::RightDword)
    };
    let mask = |code: &mut Code, gpr: Gpr, xmm: u8| {
        Host::DwordSignMask.register(code, gpr as u8, xmm, TargetCpu::Baseline);
    };
    // GPR = the lanes of `source` that are NaNs, as MOVMSKPS bits.
    let nan_bits = |code: &mut Code, gpr: Gpr, source: u8| {
        packed(code, Host::MoveAligned, TEMPORARY, source);
        code.asm()
            .float_compare(lanes, TEMPORARY, Rm::Xmm(TEMPORARY), UNORDERED);
        mask(code, gpr, TEMPORARY);
    };
    // GPR = the lanes of `source` that are infinite or NaN: a zero times
    // them is a NaN.
    let unbounded_bits = |code: &mut Code, gpr: Gpr, source: u8| {
        packed(code, Host::Xor, TEMPORARY, TEMPORARY);
        code.asm()
            .float(FloatOp::Mul, lanes, TEMPORARY, Rm::Xmm(source));
        code.asm()
            .float_compare(lanes, TEMPORARY, Rm::Xmm(TEMPORARY), UNORDERED);
        mask(code, gpr, TEMPORARY);
    };
    // `destination` &= !`source`, through ECX.
    let and_not = |code: &mut Code, destination: Gpr, source: Gpr| {
        let mut asm = code.asm();
        asm.mov(Size::Dword, Gpr::Rcx, source);
        asm.alu_imm8(Alu::Xor, Size::Dword, Gpr::Rcx, -1);
        asm.alu(Alu::And, Size::Dword, destination, Gpr::Rcx);
    };
    let mut full = Vec::new();
    test_control(code, FPCR_DN);
    full.push(branch(code, Condition::NotZero));
    read_numerical_vector(code, shape, operand.left, VN);
    read_numerical_vector(code, shape, operand.addend, ADDEND);
    prepare_right(code, operand);
    poisoned_accumulator(code, operand, resume, resumes);
    read_numerical_vector(code, shape, operand.left, VN);
    read_numerical_vector(code, shape, operand.addend, ADDEND);
    prepare_right(code, operand);
    // R8D: NaN operands; R9D: infinite operands; R10D: rejected lanes;
    // R11D: NaN factors.
    for gpr in [Gpr::R8, Gpr::R9, Gpr::R10] {
        code.asm().alu(Alu::Xor, Size::Dword, gpr, gpr);
    }
    for source in [VN, RIGHT, ADDEND] {
        if source == ADDEND {
            code.asm().mov(Size::Dword, Gpr::R11, Gpr::R8);
        }
        nan_bits(code, Gpr::Rax, source);
        code.asm().alu(Alu::Or, Size::Dword, Gpr::R8, Gpr::Rax);
        // A signalling NaN: a NaN whose quiet bit, shifted to the sign, is
        // clear.
        packed(code, Host::MoveAligned, TEMPORARY, source);
        left_shift.emit(
            code,
            TEMPORARY,
            (format.bits() - format.fraction_bits()) as u8,
        );
        mask(code, Gpr::Rdx, TEMPORARY);
        if double {
            // Only each lane's high dword holds its quiet bit.
            let mut asm = code.asm();
            asm.alu_imm8(Alu::And, Size::Dword, Gpr::Rdx, HIGH_DWORDS);
            asm.mov(Size::Dword, Gpr::Rcx, Gpr::Rdx);
            asm.shift_imm(ShiftOp::Shr, Size::Dword, Gpr::Rcx, 1);
            asm.alu(Alu::Or, Size::Dword, Gpr::Rdx, Gpr::Rcx);
        }
        and_not(code, Gpr::Rax, Gpr::Rdx);
        code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rax);
        // Infinite: unbounded and not a NaN.
        nan_bits(code, Gpr::Rax, source);
        unbounded_bits(code, Gpr::Rdx, source);
        and_not(code, Gpr::Rdx, Gpr::Rax);
        code.asm().alu(Alu::Or, Size::Dword, Gpr::R9, Gpr::Rdx);
    }
    // An infinity times a zero: a NaN product of factors that are not NaNs.
    packed(code, Host::MoveAligned, OTHER, VN);
    code.asm().float(FloatOp::Mul, lanes, OTHER, Rm::Xmm(RIGHT));
    nan_bits(code, Gpr::Rax, OTHER);
    and_not(code, Gpr::Rax, Gpr::R11);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rax);
    // A NaN result without a NaN operand: opposite infinities.
    nan_bits(code, Gpr::Rax, LEFT);
    code.asm().mov(Size::Dword, Gpr::Rdx, Gpr::Rax);
    and_not(code, Gpr::Rdx, Gpr::R8);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rdx);
    // An infinite result without an infinite operand: an overflow.
    unbounded_bits(code, Gpr::Rdx, LEFT);
    and_not(code, Gpr::Rdx, Gpr::Rax);
    and_not(code, Gpr::Rdx, Gpr::R9);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rdx);
    // A magnitude below twice the minimum normal (a NaN compares false).
    packed(code, Host::MoveAligned, TEMPORARY, LEFT);
    left_shift.emit(code, TEMPORARY, 1);
    right_shift.emit(code, TEMPORARY, 1);
    code.asm().mov_imm64(Gpr::Rax, 2 << format.fraction_bits());
    packed(code, Host::MoveGprQword, OTHER, Gpr::Rax as u8);
    if double {
        packed(code, Host::UnpackLowQword, OTHER, OTHER);
    } else {
        super::shuffle(code, OTHER, OTHER, 0);
    }
    code.asm()
        .float_compare(lanes, TEMPORARY, Rm::Xmm(OTHER), LESS);
    mask(code, Gpr::Rax, TEMPORARY);
    code.asm().alu(Alu::Or, Size::Dword, Gpr::R10, Gpr::Rax);
    active_lanes(code, shape, format, ACTIVE);
    mask(code, Gpr::Rax, ACTIVE);
    code.asm().test(Size::Dword, Gpr::R10, Gpr::Rax);
    full.push(branch(code, Condition::NotZero));
    // Vm's NaNs, then Vn's, then the addend's, each over the last.
    for source in [RIGHT, VN, ADDEND] {
        packed(code, Host::MoveAligned, TEMPORARY, source);
        code.asm()
            .float_compare(lanes, TEMPORARY, Rm::Xmm(TEMPORARY), UNORDERED);
        packed(code, Host::And, TEMPORARY, ACTIVE);
        packed(code, Host::MoveAligned, OTHER, TEMPORARY);
        packed(code, Host::AndNot, OTHER, LEFT);
        packed(code, Host::And, TEMPORARY, source);
        packed(code, Host::Or, TEMPORARY, OTHER);
        packed(code, Host::MoveAligned, LEFT, TEMPORARY);
    }
    let resumed = jump(code);
    resolve(code, resumed, resume);
    resumes.push(resumed);
    full
}

/// [`propagated_specials`]' first check, for an accumulator a NaN has
/// reached, with the operands read into XMM2 (Vn), RIGHT and ADDEND:
/// return to `resume` when each active NaN result lane has a quiet NaN
/// addend and factors whose product is finite (so neither is a NaN, an
/// infinity or a signalling NaN, and no infinity meets a zero), where
/// FPProcessNaNs3 gives the addend without status, and every other active
/// lane is at least twice the minimum normal and finite. Otherwise fall
/// through. Clobbers XMM1 to XMM5, XMM7, EAX, ECX and EDX, and LEFT only
/// on the way to `resume`.
fn poisoned_accumulator(
    code: &mut Code,
    operand: &Operand,
    resume: usize,
    resumes: &mut Vec<usize>,
) {
    use crate::x86_64::asm::ShiftOp;
    const VN: u8 = 2;
    const TEMPORARY: u8 = 3;
    const QUIET: u8 = 4;
    const OTHER: u8 = 5;
    const NAN: u8 = 7;
    /// PSHUFD selector copying each qword's high dword over its low one.
    const HIGH_DWORDS: u8 = 0xf5;
    let (format, shape) = (operand.format, operand.shape);
    let lanes = float_lanes(format, Shape::Vector128);
    let double = format == Format::Double;
    let (left_shift, right_shift) = if double {
        (ShiftOpcode::LeftQword, ShiftOpcode::RightQword)
    } else {
        (ShiftOpcode::LeftDword, ShiftOpcode::RightDword)
    };
    let mask = |code: &mut Code, gpr: Gpr, xmm: u8| {
        Host::DwordSignMask.register(code, gpr as u8, xmm, TargetCpu::Baseline);
    };
    let unordered = |code: &mut Code, xmm: u8| {
        code.asm()
            .float_compare(lanes, xmm, Rm::Xmm(xmm), UNORDERED);
    };
    // TEMPORARY: lanes whose product is not finite.
    packed(code, Host::Xor, OTHER, OTHER);
    packed(code, Host::MoveAligned, TEMPORARY, VN);
    code.asm()
        .float(FloatOp::Mul, lanes, TEMPORARY, Rm::Xmm(RIGHT));
    code.asm()
        .float(FloatOp::Mul, lanes, TEMPORARY, Rm::Xmm(OTHER));
    unordered(code, TEMPORARY);
    // QUIET: lanes whose addend's quiet bit is set.
    packed(code, Host::MoveAligned, QUIET, ADDEND);
    left_shift.emit(code, QUIET, (format.bits() - format.fraction_bits()) as u8);
    ShiftOpcode::ArithmeticDword.emit(code, QUIET, 31);
    if double {
        super::shuffle(code, QUIET, QUIET, HIGH_DWORDS);
    }
    // TEMPORARY: lanes with a quiet NaN addend and a finite product.
    packed(code, Host::MoveAligned, OTHER, ADDEND);
    unordered(code, OTHER);
    packed(code, Host::And, OTHER, QUIET);
    packed(code, Host::AndNot, TEMPORARY, OTHER);
    // OTHER: NaN result lanes without them.
    packed(code, Host::MoveAligned, NAN, LEFT);
    unordered(code, NAN);
    packed(code, Host::MoveAligned, OTHER, TEMPORARY);
    packed(code, Host::AndNot, OTHER, NAN);
    // QUIET: lanes whose result exponent is neither of the lowest two nor
    // all ones; a D lane's lands in its low dword, its high dword zero.
    packed(code, Host::MoveAligned, QUIET, LEFT);
    left_shift.emit(code, QUIET, 1);
    right_shift.emit(code, QUIET, format.fraction_bits() as u8 + 1);
    packed(code, Host::EqualDword, RIGHT, RIGHT);
    ShiftOpcode::RightDword.emit(code, RIGHT, 31);
    packed(code, Host::EqualDword, VN, VN);
    ShiftOpcode::RightDword.emit(
        code,
        VN,
        (u32::BITS - (format.bits() - format.fraction_bits() - 1)) as u8,
    );
    packed(code, Host::MoveAligned, TEMPORARY, QUIET);
    packed(code, Host::GreaterDword, TEMPORARY, RIGHT);
    packed(code, Host::GreaterDword, VN, QUIET);
    packed(code, Host::And, TEMPORARY, VN);
    mask(code, Gpr::Rax, TEMPORARY);
    if double {
        let mut asm = code.asm();
        asm.alu_imm8(Alu::And, Size::Dword, Gpr::Rax, 0b0101);
        asm.mov(Size::Dword, Gpr::Rcx, Gpr::Rax);
        asm.shift_imm(ShiftOp::Shl, Size::Dword, Gpr::Rcx, 1);
        asm.alu(Alu::Or, Size::Dword, Gpr::Rax, Gpr::Rcx);
    }
    // Rejected: lanes neither in range nor NaNs, and the NaNs in OTHER.
    mask(code, Gpr::Rdx, NAN);
    let mut asm = code.asm();
    asm.alu(Alu::Or, Size::Dword, Gpr::Rax, Gpr::Rdx);
    asm.alu_imm8(Alu::Xor, Size::Dword, Gpr::Rax, -1);
    mask(code, Gpr::Rdx, OTHER);
    let mut asm = code.asm();
    asm.alu(Alu::Or, Size::Dword, Gpr::Rax, Gpr::Rdx);
    let active: i8 = match (shape, format) {
        (Shape::Vector128, _) => 0b1111,
        (Shape::Vector64, Format::Single) | (Shape::Scalar, Format::Double) => 0b0011,
        _ => 0b0001,
    };
    asm.alu_imm8(Alu::And, Size::Dword, Gpr::Rax, active);
    let rejected = branch(code, Condition::NotZero);
    // The active NaN lanes take the addend.
    active_lanes(code, shape, format, TEMPORARY);
    packed(code, Host::And, NAN, TEMPORARY);
    packed(code, Host::MoveAligned, OTHER, NAN);
    packed(code, Host::AndNot, OTHER, LEFT);
    packed(code, Host::And, NAN, ADDEND);
    packed(code, Host::Or, NAN, OTHER);
    packed(code, Host::MoveAligned, LEFT, NAN);
    let resumed = jump(code);
    resolve(code, resumed, resume);
    resumes.push(resumed);
    let end = code.len();
    resolve(code, rejected, end);
}
