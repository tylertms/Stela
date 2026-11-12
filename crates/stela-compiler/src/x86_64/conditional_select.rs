//! Branchless conditional select and compare using guest NZCV.
use super::{capture_nzcv, condition_holds, flag_word, load_with, pinned, store, store_nzcv};
use crate::aarch64::{
    Register, Width,
    conditional_select::{CompareOperand, CompareRight, Operand, Otherwise},
};
use crate::x86_64::asm::{
    Alu, Asm, Code, Cond,
    Gpr::{R8, Rax, Rdx},
    Mem, Rm, Size, Unary,
};

const NZCV_SHIFT: u8 = 28;

/// The comparison runs on both arms and a CMOV picks its flags or the
/// fallback. A preceding pinned capture can supply the predicate directly.
pub(super) fn emit_compare(code: &mut Code, operand: &CompareOperand, live_nzcv: bool) {
    let predicate = condition_holds(code, operand.condition, live_nzcv);
    if let Some(cond) = predicate {
        // R8 is scratch under the native leaf ABI.
        code.asm().setcc(cond, R8);
    }
    let size = Size::of(operand.width == Width::X64);
    let op = if operand.subtract { Alu::Cmp } else { Alu::Add };
    match operand.right {
        CompareRight::Register(register) => {
            let emitted =
                pinned::unshifted_binary(&mut code.asm(), op, size, Rax, operand.left, register);
            debug_assert!(emitted, "RAX is scratch for conditional compare");
        }
        CompareRight::Immediate(value) => {
            let mut asm = code.asm();
            let destination = match asm.guest(operand.left) {
                Some(left) if operand.subtract => left,
                _ => {
                    load_with(&mut asm, operand.left);
                    Rax.into()
                }
            };
            asm.alu_imm(op, size, destination, i32::from(value));
        }
    }
    let pinned = code.is_pinned();
    if pinned {
        flag_word::capture_in_rax(code, operand.subtract);
    } else {
        capture_nzcv(code, operand.subtract);
    }
    let fallback = flag_word::constant(code, u32::from(operand.fallback) << NZCV_SHIFT);
    let mut asm = code.asm();
    if predicate.is_some() {
        // The fallback NZCV word where the condition fails.
        asm.mov_imm32(Rdx, fallback);
        asm.test(Size::Byte, R8, R8);
        asm.cmov(Cond::E, Size::Dword, Rax, Rdx);
    }
    if pinned {
        flag_word::store(&mut asm, Rax);
    } else {
        store_nzcv(&mut asm);
    }
}

/// Where a select finds NZCV.
enum Source {
    /// The state block.
    Frame,
    /// The host flags a capture just left ([`condition_holds`]).
    Live,
    /// The host flags of a deferred capture ([`flag_word::host_condition`]).
    Host { borrow: bool },
}

impl Source {
    /// The host condition under which Arm `condition` holds, or `None` for
    /// AL and NV.
    fn predicate(&mut self, code: &mut Code, condition: u8) -> Option<Cond> {
        match self {
            Self::Frame => condition_holds(code, condition, false),
            Self::Live => condition_holds(code, condition, true),
            Self::Host { borrow } => {
                (condition < 14).then(|| flag_word::host_condition(code, condition, borrow))
            }
        }
    }
}

pub(super) fn emit(code: &mut Code, operand: &Operand, live_nzcv: bool) {
    let mut source = if live_nzcv {
        Source::Live
    } else {
        Source::Frame
    };
    emit_from(code, operand, &mut source);
}

/// A select of pinned code on NZCV that only the host flags hold, the host
/// CF the complement of C where `borrow`. Every form but CSETM and CSNEG
/// leaves them as they were, but for a carry HI or LS turns over
/// ([`flag_word::host_condition`]), which `borrow` then says.
pub(super) fn emit_host(code: &mut Code, operand: &Operand, borrow: &mut bool) {
    debug_assert!(keeps_host_flags(operand));
    let mut source = Source::Host { borrow: *borrow };
    emit_from(code, operand, &mut source);
    if let Source::Host { borrow: after } = source {
        *borrow = after;
    }
}

/// Whether [`emit_host`] takes `operand` and leaves the host flags: CSETM
/// and CSNEG negate after the predicate, and AL and NV need none.
pub(super) fn keeps_host_flags(operand: &Operand) -> bool {
    let setm = operand.left == Register::Zero && operand.right == Register::Zero;
    operand.condition < 14
        && match operand.otherwise {
            Otherwise::Unchanged | Otherwise::Increment => true,
            Otherwise::Invert => !setm,
            Otherwise::Negate => false,
        }
}

fn emit_from(code: &mut Code, operand: &Operand, source: &mut Source) {
    let live_nzcv = !matches!(source, Source::Frame);
    let size = Size::of(operand.width == Width::X64);
    // The false arm's scratch: RDX in pinned code, whose predicates leave it
    // alone, which leaves R8 to a run's locals (`pinned::Span`).
    let spare = if code.is_pinned() { Rdx } else { R8 };
    let destination = code.host(operand.destination);
    let output = destination.unwrap_or(Rax);
    if operand.left == Register::Zero
        && operand.right == Register::Zero
        && matches!(operand.otherwise, Otherwise::Increment | Otherwise::Invert)
        && let Some(predicate) = source.predicate(code, operand.condition)
    {
        // CSET and CSETM materialize the condition as a bit or a mask.
        let mut asm = code.asm();
        asm.setcc(predicate.negate(), output);
        asm.movzx(Size::Dword, output, Size::Byte, output);
        if operand.otherwise == Otherwise::Invert {
            asm.unary(Unary::Neg, size, output);
        }
        if destination.is_none() {
            store(code, operand.destination);
        }
        return;
    }
    let otherwise = if operand.condition >= 14 {
        // AL and NV always select Rn.
        None
    } else if operand.otherwise == Otherwise::Unchanged
        && let Some(source) = code.asm().guest(operand.right)
    {
        Some(source)
    } else {
        // Prepare the false arm before the predicate. The spare survives
        // predicate emission, and the true arm is a MOV that leaves host
        // flags intact.
        let mut asm = code.asm();
        select_input(&mut asm, spare, size, operand.right);
        match operand.otherwise {
            Otherwise::Increment if live_nzcv => asm.lea(size, spare, Mem::base(spare, 1)),
            Otherwise::Increment => asm.inc(size, spare),
            Otherwise::Invert => asm.unary(Unary::Not, size, spare),
            Otherwise::Negate => asm.unary(Unary::Neg, size, spare),
            Otherwise::Unchanged => {}
        }
        Some(Rm::Reg(spare))
    };
    // If the unchanged false arm already lives in the result register,
    // keep it there and conditionally move the true arm instead.
    let keep_false = operand.left != operand.destination
        && matches!(otherwise, Some(Rm::Reg(source)) if source == output);
    let (initial, alternate) = if keep_false {
        let mut asm = code.asm();
        let source = asm.guest(operand.left).unwrap_or_else(|| {
            asm.mov_imm32(spare, 0);
            Rm::Reg(spare)
        });
        (operand.destination, Some(source))
    } else {
        (operand.left, otherwise)
    };
    let predicate = source.predicate(code, operand.condition);
    let mut asm = code.asm();
    let selected = predicate.zip(alternate);
    let already_placed = destination.is_some() && initial == operand.destination;
    // CMOVcc r32 in 64-bit mode clears the upper half even when its
    // predicate is false (Intel SDM, CMOVcc).
    if selected.is_none() || !already_placed {
        select_input(&mut asm, output, size, initial);
    }
    if let Some((predicate, alternate)) = selected {
        let take_alternate = if keep_false {
            predicate
        } else {
            predicate.negate()
        };
        asm.cmov(take_alternate, size, output, alternate);
    }
    if destination.is_none() {
        store(code, operand.destination);
    }
}

/// A W source is zero-extended by the sized MOV. XZR uses an immediate
/// instead of XOR so it can be loaded after the predicate.
fn select_input(asm: &mut Asm, host: crate::x86_64::asm::Gpr, size: Size, guest: Register) {
    if let Some(source) = asm.guest(guest) {
        asm.mov_load(size, host, source);
    } else {
        asm.mov_imm32(host, 0);
    }
}

#[cfg(all(
    test,
    target_arch = "x86_64",
    any(target_os = "macos", target_os = "linux")
))]
mod tests {
    use super::*;
    use crate::GuestState;

    #[test]
    fn compares_registers_and_immediates_without_changing_operands() {
        for width in [Width::W32, Width::X64] {
            let sign = match width {
                Width::W32 => 1 << 31,
                Width::X64 => 1 << 63,
            };
            let initial = {
                let mut state = GuestState::default();
                state.x[1] = sign;
                state.x[2] = u64::MAX;
                state.x[3] = sign;
                state.x[8] = u64::MAX;
                state
            };
            use CompareRight::{Immediate, Register as FromRegister};
            use Register::{X, Zero};
            for (left, right, sub_flags, add_flags) in [
                (X(1), FromRegister(X(2)), 8, 3),
                (X(3), FromRegister(X(8)), 8, 3),
                (X(1), FromRegister(X(1)), 6, 7),
                (X(1), FromRegister(Zero), 10, 8),
                (X(3), Immediate(0), 10, 8),
                (X(3), Immediate(31), 3, 8),
                (Zero, Immediate(31), 8, 0),
            ] {
                for (subtract, flags) in [(true, sub_flags), (false, add_flags)] {
                    for condition in [0, 14, 15] {
                        let operand = CompareOperand {
                            width,
                            subtract,
                            left,
                            right,
                            condition,
                            fallback: 5,
                        };
                        let mut frame = Code::new();
                        let mut pinned = Code::pinned();
                        emit_compare(&mut frame, &operand, false);
                        emit_compare(&mut pinned, &operand, false);
                        pinned.asm().ret();
                        let pinned = super::super::pinned::entered(&pinned);
                        for nzcv in [0, 4] {
                            let expected = if condition == 0 && nzcv == 0 {
                                5
                            } else {
                                flags
                            };
                            for bytes in [&frame[..], &pinned[..]] {
                                let mut state = initial.clone();
                                state.nzcv = nzcv << NZCV_SHIFT;
                                super::super::native::run(bytes, &mut state);
                                assert_eq!(state.x, initial.x, "{operand:?}");
                                assert_eq!(state.nzcv, expected << NZCV_SHIFT, "{operand:?}");
                            }
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn selects_preserve_nzcv_and_zero_extend_w_results() {
        const LEFT: u64 = 0x1234_5678_9abc_def0;
        const RIGHT: u64 = 0xfedc_ba98_7654_3210;
        let cases = [Width::W32, Width::X64]
            .into_iter()
            .flat_map(|width| [1, 2, 8].map(|destination| (width, destination)));
        for (width, destination) in cases {
            for (left, lhs) in [(Register::X(1), LEFT), (Register::Zero, 0)] {
                for (right, rhs) in [(Register::X(8), RIGHT), (Register::Zero, 0)] {
                    for (otherwise, alternate) in [
                        (Otherwise::Unchanged, rhs),
                        (Otherwise::Increment, rhs.wrapping_add(1)),
                        (Otherwise::Invert, !rhs),
                        (Otherwise::Negate, rhs.wrapping_neg()),
                    ] {
                        for condition in [0, 1, 10, 12, 14, 15] {
                            let operand = Operand {
                                width,
                                destination: Register::X(destination),
                                left,
                                right,
                                condition,
                                otherwise,
                            };
                            let mut frame = Code::new();
                            let mut pinned = Code::pinned();
                            emit(&mut frame, &operand, false);
                            emit(&mut pinned, &operand, false);
                            pinned.asm().ret();
                            let pinned = super::super::pinned::entered(&pinned);
                            for flags in 0..16 {
                                let n = flags & 8 != 0;
                                let z = flags & 4 != 0;
                                let v = flags & 1 != 0;
                                let select_left = match condition {
                                    0 => z,
                                    1 => !z,
                                    10 => n == v,
                                    12 => n == v && !z,
                                    _ => true,
                                };
                                let expected = if select_left { lhs } else { alternate };
                                let expected = if width == Width::W32 {
                                    u64::from(expected as u32)
                                } else {
                                    expected
                                };
                                for code in [&frame[..], &pinned] {
                                    let mut state = GuestState {
                                        nzcv: flags << 28,
                                        ..Default::default()
                                    };
                                    state.x[1] = LEFT;
                                    state.x[8] = RIGHT;
                                    super::super::native::run(code, &mut state);
                                    let mut expected_registers = [0; 31];
                                    expected_registers[1] = LEFT;
                                    expected_registers[8] = RIGHT;
                                    expected_registers[usize::from(destination)] = expected;
                                    assert_eq!(state.x, expected_registers, "{operand:?}");
                                    assert_eq!(state.nzcv, flags << 28);
                                }
                            }
                        }
                    }
                }
            }
        }
    }
}
