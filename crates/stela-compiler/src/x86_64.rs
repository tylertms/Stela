//! x86-64 code emission. RDI holds the guest state pointer throughout.
use crate::x86_64::asm::Code;
use crate::{
    Backend, GuestAddress, NativeBlock, UnsupportedReason,
    aarch64::{
        AddressMode, Instruction, Logical, MemoryTransfer, Operation, Register, RegisterExtend,
        Shift, WideMove, Width,
    },
};
use alloc::vec;
use alloc::vec::Vec;

pub struct Emitter;

/// Check semantic host requirements before either leaf or block emission.
pub(crate) fn validate_target(
    instruction: &Instruction,
    target: crate::TargetCpu,
) -> Result<(), UnsupportedReason> {
    validate_target_with(instruction, target, MemoryChecks::DEFAULT)
}

/// [`validate_target`] for code emitted with `checks`.
pub(crate) fn validate_target_with(
    instruction: &Instruction,
    target: crate::TargetCpu,
    checks: MemoryChecks,
) -> Result<(), UnsupportedReason> {
    if matches!(
        instruction.operation,
        Operation::Exclusive(_) | Operation::CheckedPointer(_)
    ) {
        return Err(UnsupportedReason::PrivilegedOperation {
            address: instruction.pc,
        });
    }
    if let Operation::Synchronization(operation) = &instruction.operation
        && !matches!(
            operation,
            crate::aarch64::system::synchronization::Operation::Yield
                | crate::aarch64::system::synchronization::Operation::ConsumptionBarrier
                | crate::aarch64::system::synchronization::Operation::SpeculationBarrier
        )
    {
        return Err(UnsupportedReason::PrivilegedOperation {
            address: instruction.pc,
        });
    }
    if let Operation::Ordered(operand) = &instruction.operation
        && ordered::requires_atomic_load16(operand, target, checks)
        && !target.supports(crate::TargetCpu::AtomicLoad16)
    {
        return Err(UnsupportedReason::HostCapabilityRequired {
            address: instruction.pc,
            required: crate::TargetCpu::AtomicLoad16,
        });
    }
    if let Operation::Atomic(operand) = &instruction.operation
        && atomic::requires_cx16(operand, target, checks)
        && !target.supports(crate::TargetCpu::Cx16)
    {
        return Err(UnsupportedReason::HostCapabilityRequired {
            address: instruction.pc,
            required: crate::TargetCpu::Cx16,
        });
    }
    Ok(())
}

/// Native code under construction, with its counted instructions and the
/// native offset of each guest instruction.
#[derive(Debug, Default)]
pub(crate) struct Emitted {
    pub(crate) x86_64_code: asm::Code,
    pub(crate) source_map: Vec<(crate::GuestAddress, usize)>,
}

impl Emitted {
    pub(crate) fn into_native(self) -> crate::NativeBlock {
        let (x86_64_code, relocations) = self.x86_64_code.into_parts();
        crate::NativeBlock {
            x86_64_code,
            source_map: self.source_map,
            relocations: Some(relocations),
        }
    }
}

pub mod amx;
pub mod amx_routine;
pub mod amx_transfer;
pub(crate) mod asm;
mod atomic;
mod bitfield;
pub(crate) mod block;
mod byte_prefix;
pub mod chain;
pub(crate) mod checked_pointer;
mod conditional_select;
mod crc;
pub(crate) mod exclusive;
pub mod flag_word;
mod flags;
mod fp_register;
mod guest_range;
mod integer_math;
mod integer_minmax;
mod integer_unary;
pub mod isa;
mod literal;
mod masked_base;
pub(crate) mod memory_context;
mod memory_recipe;
#[cfg(all(
    any(test, feature = "test-support"),
    target_arch = "x86_64",
    target_os = "linux"
))]
pub mod modeled;
pub mod pinned;
/// Without the Linux signal-frame models, tests run only what the host has.
#[cfg(all(
    any(test, feature = "test-support"),
    target_arch = "x86_64",
    target_os = "macos"
))]
pub mod modeled {
    pub fn host() -> crate::TargetCpu {
        crate::TargetCpu::detect()
    }
}
pub(crate) mod memory_store;
#[cfg(all(
    test,
    target_arch = "x86_64",
    any(target_os = "macos", target_os = "linux")
))]
mod native;
mod non_temporal;
mod ordered;
mod pc_relative;
pub mod pointer;
mod pointer_checks;
mod prefetch;
mod region;
pub(crate) use region::AllocationStats;
pub use region::RegionError;
mod register_offset;
pub mod register_read;
mod scalable;
mod scalable_matrix;
pub(crate) mod shared_memory;
mod simd;
mod stack_alignment;
pub mod transition;
pub(crate) use simd::CheckedMemoryOperand as CheckedFixedMemoryOperand;
pub mod cache_clean;
pub mod native_exclusive;
pub(crate) mod sme;
pub(crate) mod sve;
mod system;
mod tag_arithmetic;
mod top_byte;
pub use top_byte::TopByte;
pub mod mirrored_read;
mod unprivileged;
pub(crate) mod vector_cache;
pub mod zero_block;

/// The System instructions a system-mode block ending in one completes
/// natively instead of exiting: DC ZVA zeroing a block of `zero_block`
/// bytes, and the DC clean operations of `cache_clean`.
#[derive(
    Debug,
    Clone,
    Copy,
    Default,
    PartialEq,
    Eq,
    PartialOrd,
    Ord,
    Hash,
    serde::Serialize,
    serde::Deserialize,
)]
pub struct NativeSystem {
    pub zero_block: Option<u16>,
    pub cache_clean: cache_clean::Operations,
    /// PEs run on host cores of their own, and exclusives pass by value.
    pub exclusive: bool,
    /// Where the engine's PAC service keeps what native code completes PAC
    /// instructions from, which blocks then complete inside themselves
    /// rather than ending at one (`x86_64::pointer::inline`).
    pub pointer: Option<pointer::inline::Cells>,
    /// The System registers whose MRS blocks complete inside themselves from
    /// the engine's cells rather than ending at one (`x86_64::mirrored_read`).
    pub mirrored: mirrored_read::Reads,
    /// The machine has AMX: its moves complete inside blocks from the PE's
    /// register file rather than ending them (`x86_64::amx_transfer`).
    pub amx: bool,
    /// A block that branches back to its own start runs its iterations
    /// inside itself, a bounded number per entry (`x86_64::block`'s loop
    /// mode), rather than through a link at each one.
    #[serde(default)]
    pub self_loops: bool,
    /// Self-loops count their entries and iterations in the PE's chain
    /// control (`chain::ChainControl::loop_entries`): only while the engine
    /// profiles.
    #[serde(default)]
    pub loop_counts: bool,
    /// Every flag-setting instruction stores NZCV as it runs, none leaving
    /// it to the exit of the B.cond after it (`x86_64::block`'s deferred
    /// captures): for tests that compare both forms.
    #[serde(default)]
    pub capture_every_nzcv: bool,
}

impl NativeSystem {
    /// Every System instruction exits.
    pub const NONE: Self = Self {
        zero_block: None,
        cache_clean: cache_clean::Operations::NONE,
        exclusive: false,
        pointer: None,
        mirrored: mirrored_read::Reads::NONE,
        amx: false,
        self_loops: false,
        loop_counts: false,
        capture_every_nzcv: false,
    };
}

fn offset(register: Register) -> Option<u32> {
    match register {
        Register::X(index) => Some(u32::from(index) * 8),
        Register::Sp => Some(248),
        Register::Zero => None,
    }
}

/// `[rdi+offset]` with a 32-bit displacement: a field of the guest frame.
fn frame(offset: u32) -> asm::Mem {
    asm::Mem::base_disp32(asm::Gpr::Rdi, offset as i32)
}

/// The frame's end of the native lower range ([`RangeCheck::one_sided`]).
fn lower_limit() -> asm::Mem {
    frame(core::mem::offset_of!(crate::GuestState, lower_limit) as u32)
}

/// `[rax]`: the access address the memory emitters compute.
fn accessed() -> asm::Mem {
    asm::Mem::base(asm::Gpr::Rax, 0)
}

fn logical(kind: Logical) -> asm::Alu {
    match kind {
        Logical::And => asm::Alu::And,
        Logical::Or => asm::Alu::Or,
        Logical::Xor => asm::Alu::Xor,
    }
}

/// `add` or `sub (e/r)ax,(e/r)cx`.
fn add_sub(code: &mut Code, width: Width, subtract: bool) {
    let op = if subtract {
        asm::Alu::Sub
    } else {
        asm::Alu::Add
    };
    code.asm().alu(
        op,
        asm::Size::of(width == Width::X64),
        asm::Gpr::Rax,
        asm::Gpr::Rcx,
    );
}

/// `test (e/r)ax,(e/r)ax`: the condition under which RAX at `width` is
/// nonzero, or zero.
pub(super) fn test_zero(code: &mut Code, width: Width, nonzero: bool) -> asm::Cond {
    use asm::Gpr::Rax;
    code.asm()
        .test(asm::Size::of(width == Width::X64), Rax, Rax);
    if nonzero { asm::Cond::Ne } else { asm::Cond::E }
}

/// Stage the extended right operand in RCX.
fn extended_operand(code: &mut Code, right: Register, extend: RegisterExtend, amount: u8) {
    use asm::Size::{Byte, Dword, Qword, Word};
    use asm::{Alu, Gpr::Rcx};
    let mut asm = code.asm();
    if let Some(storage) = asm.guest(right) {
        match extend {
            RegisterExtend::Uxtb => asm.movzx(Dword, Rcx, Byte, storage),
            RegisterExtend::Uxth => asm.movzx(Dword, Rcx, Word, storage),
            RegisterExtend::Uxtw => asm.mov_load(Dword, Rcx, storage),
            RegisterExtend::Sxtb => asm.movsx(Qword, Rcx, Byte, storage),
            RegisterExtend::Sxth => asm.movsx(Qword, Rcx, Word, storage),
            RegisterExtend::Sxtw => asm.movsxd(Rcx, storage),
            RegisterExtend::Uxtx | RegisterExtend::Sxtx => asm.mov_load(Qword, Rcx, storage),
        }
    } else {
        asm.alu(Alu::Xor, Dword, Rcx, Rcx);
    }
    if amount != 0 {
        asm.shift_imm(asm::ShiftOp::Shl, Qword, Rcx, amount);
    }
}

/// Compare an extended right operand against the left operand in its existing
/// location. The flags and their Arm subtraction carry are captured normally.
fn compare_extended(
    code: &mut Code,
    width: Width,
    left: asm::Rm,
    right: Register,
    extend: RegisterExtend,
    amount: u8,
) {
    extended_operand(code, right, extend, amount);
    code.asm().alu(
        asm::Alu::Cmp,
        asm::Size::of(width == Width::X64),
        left,
        asm::Gpr::Rcx,
    );
    nzcv(code, true);
}

/// `bt rax,bit`: the condition under which the bit is set, or clear.
pub(super) fn test_bit(code: &mut Code, bit: u8, nonzero: bool) -> asm::Cond {
    test_bit_of(code.asm(), asm::Gpr::Rax, bit, nonzero)
}

/// `bt register,bit`: the condition under which the bit is set, or clear.
pub(super) fn test_bit_of(
    mut asm: asm::Asm,
    register: asm::Gpr,
    bit: u8,
    nonzero: bool,
) -> asm::Cond {
    asm.bit_imm(asm::BitTest::Bt, asm::Size::Qword, register, bit);
    if nonzero { asm::Cond::B } else { asm::Cond::Ae }
}

/// Backend policy shared by block parsing and emission. Terminal numerical
/// mappings consume only their own word; conditional mappings keep a suffix.
pub(crate) fn numerical_always_exits(operation: &crate::aarch64::FloatingPointOperation) -> bool {
    simd::numerical_always_exits(operation)
}

fn load(code: &mut Code, register: Register) {
    load_with(&mut code.asm(), register);
}

/// RAX = the guest register (zero for XZR), from the frame at RDI.
fn load_with(asm: &mut asm::Asm, register: Register) {
    use asm::{Alu, Gpr, Size};
    match asm.guest(register) {
        Some(storage) => asm.mov_load(Size::Qword, Gpr::Rax, storage),
        None => asm.alu(Alu::Xor, Size::Dword, Gpr::Rax, Gpr::Rax),
    }
}

fn store(code: &mut Code, register: Register) {
    store_with(&mut code.asm(), register);
}

/// The guest register (nothing for XZR) = RAX, in the frame at RDI.
fn store_with(asm: &mut asm::Asm, register: Register) {
    use asm::{Gpr, Size};
    if let Some(storage) = asm.guest(register) {
        asm.mov_store(Size::Qword, storage, Gpr::Rax);
    }
}

/// A decoded immediate memory address and its writeback mode.
#[derive(Clone, Copy)]
struct ImmediateAddress {
    base: Register,
    displacement: i32,
    mode: AddressMode,
}

/// Direct ordinary-RAM profile. Guest virtual addresses equal native addresses;
/// the caller keeps code/state separate from guest mappings. Guest NZCV is untouched.
fn memory(
    code: &mut Code,
    checks: MemoryChecks,
    transfer: MemoryTransfer,
    bytes: u8,
    data: Register,
    address: ImmediateAddress,
) {
    memory_recipe::scalar(
        code,
        checks,
        memory_recipe::Scalar {
            transfer,
            bytes,
            data,
            address,
        },
        None,
        None,
    );
}

/// Guest checks fixed into a block's native memory accesses. Each failed
/// check raises a native #UD before the access, which the system-mode engine
/// turns into the guest's SP alignment or Alignment fault.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct MemoryChecks {
    /// SCTLR_ELx.SA (SA0 at EL0) is set: an access through an SP that is not
    /// 16-byte aligned faults.
    pub stack_alignment: bool,
    /// SCTLR_ELx.A is set: a data access not aligned to the size of each
    /// element it transfers faults (Arm `AlignmentEnforced`).
    pub alignment: bool,
    /// SCTLR_ELx.nAA is clear: a load-acquire or store-release that crosses a
    /// 16-byte granule faults. Atomics always fault there (FEAT_LSE2).
    pub ordered_granule: bool,
    /// Every access checks its address against the guest's native ranges
    /// ([`RangeCheck`]). The direct system-mode profile sets it for every block, whatever
    /// the engine asks, and no other profile does.
    pub(crate) guest_range: bool,
    /// FEAT_CSV3: every access's address is masked ([`RangeCheck`]), those
    /// [`guest_range`] would leave to an earlier check through the same base
    /// included: by its own check, or in pinned blocks derived by data from
    /// an earlier access's masked address ([`masked_base`]). A mispredicted
    /// check then never runs an access whose address no check masked.
    pub every_access_masked: bool,
    /// An earlier access of the block checked SP's alignment, which holds
    /// since ([`stack_alignment::checks`]): an access through SP relies on
    /// [`Self::stack_alignment`] without its own check.
    pub(crate) stack_checked: bool,
    /// The guest's native ranges on this host's placement.
    pub range: RangeCheck,
    /// The translated variant:
    /// every access checks its address, as with
    /// [`Self::every_access_masked`], and a failed check calls the engine's
    /// translation helper through [`Self::translation_helper`] in place of
    /// the UD2.
    pub translated: bool,
    /// The address of the slot holding the translation helper's address,
    /// for the translated variant.
    pub translation_helper: u64,
    /// The tagged variant: every access checks its address, as with
    /// [`Self::every_access_masked`], after clearing the top byte its
    /// translation regime ignores ([`TopByte`]).
    #[serde(default)]
    pub top_byte: TopByte,
}

impl MemoryChecks {
    /// The process profiles' checks, and system mode's with SCTLR_ELx.SA set
    /// and A and nAA clear.
    pub const DEFAULT: Self = Self {
        stack_alignment: true,
        alignment: false,
        ordered_granule: true,
        guest_range: false,
        every_access_masked: false,
        stack_checked: false,
        range: RangeCheck::FIVE_LEVEL,
        translated: false,
        translation_helper: 0,
        top_byte: TopByte::Kept,
    };
    /// SCTLR_ELx's checks, as the system-mode engine selects them, and
    /// whether every access masks its address
    /// ([`Self::every_access_masked`]).
    pub const fn system(
        stack_alignment: bool,
        alignment: bool,
        ordered_granule: bool,
        every_access_masked: bool,
    ) -> Self {
        Self {
            stack_alignment,
            alignment,
            ordered_granule,
            guest_range: false,
            every_access_masked,
            stack_checked: false,
            range: RangeCheck::FIVE_LEVEL,
            translated: false,
            translation_helper: 0,
            top_byte: TopByte::Kept,
        }
    }

    /// These checks on a host whose placement gives the guest `range`.
    pub const fn with_range(self, range: RangeCheck) -> Self {
        assert!(range.valid(), "a range check's fields fit its encoding");
        Self { range, ..self }
    }

    /// These checks in the translated variant, calling the helper whose
    /// address the slot at `helper` holds ([`Self::translated`]).
    pub const fn with_translation(self, helper: u64) -> Self {
        Self {
            translated: true,
            translation_helper: helper,
            ..self
        }
    }

    /// These checks in the tagged variant, whose accesses ignore the top
    /// byte of the halves `top_byte` selects ([`Self::top_byte`]).
    pub const fn with_top_byte(self, top_byte: TopByte) -> Self {
        Self { top_byte, ..self }
    }

    /// Whether every access checks the guest range, none left to an earlier
    /// check through the same base.
    pub(crate) const fn checks_every_access(self) -> bool {
        self.every_access_masked || self.translated || self.strips()
    }

    /// Whether accesses clear a top byte before their check (the tagged
    /// variant).
    pub(crate) const fn strips(self) -> bool {
        !matches!(self.top_byte, TopByte::Kept)
    }
}

/// Whether atomics, ordered accesses and barriers lower to plain accesses:
/// only in the data-independent-timing mode, which requires that no other
/// agent accesses guest memory ([`crate::TargetCpu::data_independent_timing`])
/// because LOCK, XCHG and MFENCE are not on Intel's data operand independent
/// timing list. System mode runs its PEs concurrently, so every other target
/// keeps the locked recipes whatever its checks.
fn plain_ordering(target: crate::TargetCpu, _checks: MemoryChecks) -> bool {
    target.data_independent_timing()
}

/// The system-mode check of the access address in RAX against the guest's
/// native ranges, the only parts of the host address space its shadow
/// tables map:
///
/// ```text
/// mov rcx,rax; sar rcx,shift; add ecx,bias (inc ecx for 1); cmp ecx,bound
/// jbe ok; ud2; ok: mov rcx,masked; cmova rax,rcx
/// ```
///
/// Five-level paging (LA57) gives the guest PML5 entries 0 and 511, so bits
/// 63:48 must be all clear or all set: shift 48, bias 1, bound 1. A
/// four-level host gives it the lower half and `[U, 2^64)`, with
/// `U = 0xffff_8000_0000_0000 + m * 2^41` and Stela's window and guard in
/// the `m` slices of 2 TiB below `U`: shift 41, bias `64 - m`, bound
/// `127 - m`. Any other address raises #UD before the access, which the
/// engine turns into the fault the guest's translation gives it, or into
/// the translated variant. Stela's memory is then out of a guest access's
/// reach. RCX and host flags are scratch.
///
/// The check's branch can be mispredicted, so past it a CMOVA, which uses
/// the comparison's flags and no prediction, replaces an out-of-range RAX
/// with the comparison's own ECX, zero-extended: an address below 2^32, in
/// the guest's lower range, which the current root maps only as the guest's
/// translation does. An access that speculation runs past a failed check
/// never reaches Stela's memory (Intel's and AMD's bounds-check bypass
/// guidance). A passed check leaves RAX unchanged, and the UD2 runs before
/// the CMOVA with the guest's address.
#[derive(
    Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord, Hash, serde::Serialize, serde::Deserialize,
)]
pub struct RangeCheck {
    pub shift: u8,
    pub bias: u8,
    pub bound: u8,
    /// The EL0 form ([`Self::one_sided`]): `cmp rax,[lower_limit]` and the
    /// mask on above-or-equal, passing the lower range alone.
    #[serde(default)]
    pub one_sided: bool,
}

/// A four-level check's slices of 2 TiB.
const SLICE_SHIFT: u8 = 41;
/// The slices of each half.
const HALF_SLICES: u8 = 64;
/// The largest positive sign-extended imm8.
const IMM8_MAX: u8 = 127;
const OPCODE_INC: u8 = 1;

impl RangeCheck {
    /// Five-level paging (LA57): bits 63:48 all clear or all set.
    pub const FIVE_LEVEL: Self = Self {
        shift: 48,
        bias: 1,
        bound: 1,
        one_sided: false,
    };

    /// Four-level paging with Stela's window and guard in the `slices`
    /// slices of 2 TiB below `U` (2 to 63): the window's, then the guard,
    /// the whole slice below `U`.
    pub const fn four_level(slices: u8) -> Self {
        assert!(
            slices >= 2 && slices < HALF_SLICES,
            "a four-level placement takes 2 to 63 slices: at least one for \
             the window and the guard, the whole slice below U"
        );
        Self {
            shift: SLICE_SHIFT,
            bias: HALF_SLICES - slices,
            bound: IMM8_MAX - slices,
            one_sided: false,
        }
    }

    /// The end of the native lower range, `[0, limit)`: 2^48 with LA57
    /// (PML5 entry 0), 2^47 on a four-level host (its 64 slices of the
    /// lower half). Every address below it passes the two-sided check.
    pub const fn lower_limit(self) -> u64 {
        if self.shift == SLICE_SHIFT {
            (HALF_SLICES as u64) << SLICE_SHIFT
        } else {
            1 << self.shift
        }
    }

    /// This check in its EL0 form if `el0`: an address passes only below
    /// [`Self::lower_limit`], which the engine keeps in the frame
    /// (`GuestState::lower_limit`), so one compare with the frame and the
    /// mask replace the copy, shift, bias and compare. It passes a subset
    /// of what the two-sided check passes, so no access reaches more than
    /// before; an EL0 access to the upper range fails it, and the engine
    /// takes its fault or runs its page with the two-sided check.
    pub const fn one_sided(self, el0: bool) -> Self {
        Self {
            one_sided: el0,
            ..self
        }
    }

    /// Whether [`Self::emit`] passes `address`.
    pub const fn passes(self, address: u64) -> bool {
        if self.one_sided {
            return address < self.lower_limit();
        }
        let slice = ((address as i64) >> self.shift) as u32;
        slice.wrapping_add(self.bias as u32) <= self.bound as u32
    }

    /// Whether the fields fit the check's encoding: a shift of an address
    /// and a bias and bound that are positive imm8s.
    pub const fn valid(self) -> bool {
        self.shift < u64::BITS as u8 && self.bias <= IMM8_MAX && self.bound <= IMM8_MAX
    }

    /// The shift, the bias and the comparison: above fails.
    fn compare(self, asm: &mut asm::Asm) {
        use asm::{
            Alu,
            Gpr::{Rax, Rcx},
            ShiftOp, Size,
        };
        asm.mov(Size::Qword, Rcx, Rax);
        asm.shift_imm(ShiftOp::Sar, Size::Qword, Rcx, self.shift);
        if self.bias == OPCODE_INC {
            asm.inc(Size::Dword, Rcx);
        } else {
            asm.alu_imm8(Alu::Add, Size::Dword, Rcx, self.bias as i8);
        }
        asm.alu_imm8(Alu::Cmp, Size::Dword, Rcx, self.bound as i8);
    }

    /// The check of `base`, a pinned guest register whose access moves at
    /// most 2^16 bytes from it ([`guest_range`]): the address needs no copy.
    pub(crate) fn emit_base(self, code: &mut Code, base: asm::Gpr) {
        use asm::{Alu, Gpr::Rcx, ShiftOp, Size};
        let mut asm = code.asm();
        if self.one_sided {
            asm.alu_load(Alu::Cmp, Size::Qword, base, lower_limit());
            code.check_failure(asm::Cond::Ae);
            return;
        }
        asm.mov(Size::Qword, Rcx, base);
        asm.shift_imm(ShiftOp::Sar, Size::Qword, Rcx, self.shift);
        if self.bias == OPCODE_INC {
            asm.inc(Size::Dword, Rcx);
        } else {
            asm.alu_imm8(Alu::Add, Size::Dword, Rcx, self.bias as i8);
        }
        asm.alu_imm8(Alu::Cmp, Size::Dword, Rcx, self.bound as i8);
        code.check_failure(asm::Cond::A);
    }

    /// [`Self::compare`] and the branch past the failure, which the caller
    /// binds.
    fn test(self, asm: &mut asm::Asm) -> asm::Patch {
        self.compare(asm);
        asm.jcc8(asm::Cond::Be)
    }

    /// The mask after the check.
    fn mask(asm: &mut asm::Asm) {
        use asm::{
            Cond,
            Gpr::{Rax, Rcx},
            Size,
        };
        asm.cmov(Cond::A, Size::Qword, Rax, Rcx);
    }

    /// The check, which in pinned code leaves the failure to the access: a
    /// branch per access, taken each time, costs a kernel's cold code far
    /// more than a move. Elsewhere it fails by UD2.
    pub(crate) fn emit(self, code: &mut Code) {
        let mut asm = code.asm();
        if self.one_sided {
            // `cmp rax,[lower_limit]; cmovae rax,[unreachable]`, with the
            // UD2 between them outside pinned code.
            asm.alu_load(
                asm::Alu::Cmp,
                asm::Size::Qword,
                asm::Gpr::Rax,
                lower_limit(),
            );
            if !asm.is_pinned() {
                let passed = asm.jcc8(asm::Cond::B);
                asm.guest_check();
                asm.bind(passed);
            }
            unreachable_if(&mut asm, asm::Cond::Ae);
            return;
        }
        if asm.is_pinned() {
            self.compare(&mut asm);
            unreachable_if(&mut asm, asm::Cond::A);
            return;
        }
        let passed = self.test(&mut asm);
        asm.guest_check();
        asm.bind(passed);
        Self::mask(&mut asm);
    }

    /// The translated variant's check: a failure calls the translation
    /// helper whose address the slot at `helper` holds, with the guest
    /// address in RCX and `access` in the two bytes after the call; the
    /// helper returns past them with the host address of the access in RAX,
    /// which skips the mask.
    pub(crate) fn emit_translated(self, code: &mut Code, helper: u64, access: Access) {
        use asm::{
            Gpr::{Rax, Rcx},
            Size,
        };
        let mut asm = code.asm();
        let passed = self.test(&mut asm);
        asm.mov(Size::Qword, Rcx, Rax);
        asm.mov_rax_moffs(helper);
        asm.call_rm(Rax);
        asm.translated_helper_return(access.bytes, access.flags);
        let translated = asm.jmp8();
        asm.bind(passed);
        Self::mask(&mut asm);
        asm.bind(translated);
    }

    /// [`Self::emit`]'s bytes.
    #[cfg(test)]
    pub(crate) fn code(self) -> Vec<u8> {
        let mut code = Code::new();
        self.emit(&mut code);
        code.into_bytes()
    }

    /// The comparison every pinned check holds, whichever register it takes
    /// and however it fails.
    #[cfg(test)]
    pub(crate) fn pinned_code(self) -> Vec<u8> {
        const COPY_BYTES: usize = 3;
        let mut asm = Code::new();
        self.compare(&mut asm.asm());
        asm.into_bytes().split_off(COPY_BYTES)
    }

    /// Golden-byte oracle for a check of any placement. The fragment ends
    /// include the translated descriptor data and must not be used as counts.
    #[cfg(test)]
    pub(crate) fn at(code: &[u8]) -> Option<(usize, &'static [usize])> {
        // The ends with INC and with ADD, with a UD2 and with the
        // translated variant's call (including a separate access-data fragment).
        const ENDS: [[&[usize]; 2]; 2] = [
            [
                &[3, 7, 9, 12, 14, 16, 20],
                &[3, 7, 9, 12, 14, 17, 27, 29, 31, 33, 37],
            ],
            [
                &[3, 7, 10, 13, 15, 17, 21],
                &[3, 7, 10, 13, 15, 18, 28, 30, 32, 34, 38],
            ],
        ];
        let [0x48, 0x89, 0xc1, 0x48, 0xc1, 0xf9, shift, rest @ ..] = code else {
            return None;
        };
        let (add, rest) = match rest {
            [0xff, 0xc1, rest @ ..] => (0, rest),
            [0x83, 0xc1, _, rest @ ..] => (1, rest),
            _ => return None,
        };
        // The pinned check, with the same shift and comparison.
        const PINNED_ENDS: [&[usize]; 2] = [&[3, 7, 9, 12, 20], &[3, 7, 10, 13, 21]];
        if let [0x83, 0xf9, bound, 0x48, 0x0f, 0x47, 0x87, _, _, _, _, ..] = rest {
            let ends = PINNED_ENDS[add];
            return (*shift < u64::BITS as u8 && *bound <= IMM8_MAX)
                .then(|| (ends[ends.len() - 1], ends));
        }
        let [0x83, 0xf9, bound, 0x76, failure, rest @ ..] = rest else {
            return None;
        };
        let (translated, rest) = match (*failure, rest) {
            (2, [0x0f, 0x0b, rest @ ..]) => (0, rest),
            (TRANSLATION_CALL_BYTES, [0x48, 0x89, 0xc1, 0x48, 0xa1, rest @ ..]) => {
                let [
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    _,
                    0xff,
                    0xd0,
                    _,
                    _,
                    0xeb,
                    MASK_BYTES,
                    rest @ ..,
                ] = rest
                else {
                    return None;
                };
                (1, rest)
            }
            _ => return None,
        };
        let [0x48, 0x0f, 0x47, 0xc1, ..] = rest else {
            return None;
        };
        if *shift >= u64::BITS as u8 || *bound > IMM8_MAX {
            return None;
        }
        let ends = ENDS[add][translated];
        Some((ends[ends.len() - 1], ends))
    }
}

/// The translated variant's call, from `mov rcx,rax` through the jump
/// past the mask.
#[cfg(test)]
const TRANSLATION_CALL_BYTES: u8 = 19;
/// `cmova rax,rcx`.
#[cfg(test)]
const MASK_BYTES: u8 = 4;

/// What a guest access makes, which the translated variant's helper
/// translates its address for: the bytes it covers from its address
/// (saturating at 255: a larger access, DC ZVA's, is aligned to its size
/// and lies in one page) and whether it writes, is unprivileged or is an
/// atomic or exclusive access.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(crate) struct Access {
    bytes: u8,
    flags: u8,
}

impl Access {
    const WRITE: u8 = 1;
    const ATOMIC: u8 = 4;

    /// An access of `bytes` that writes if `write`, else reads.
    pub(crate) const fn of(write: bool, bytes: u16) -> Self {
        Self {
            bytes: if bytes > u8::MAX as u16 {
                u8::MAX
            } else {
                bytes as u8
            },
            flags: if write { Self::WRITE } else { 0 },
        }
    }
    pub(crate) const fn read(bytes: u16) -> Self {
        Self::of(false, bytes)
    }
    pub(crate) const fn write(bytes: u16) -> Self {
        Self::of(true, bytes)
    }
    /// The same access, made by an atomic instruction, which needs the
    /// location itself.
    pub(crate) const fn atomic(self) -> Self {
        Self {
            flags: self.flags | Self::ATOMIC,
            ..self
        }
    }
}

/// SCTLR_ELx.SA's check of an SP base in RAX, before the address addition
/// as Arm checks it ([`MemoryChecks::stack_alignment`]).
pub(crate) fn stack_alignment_check(code: &mut Code) {
    alignment_test(&mut code.asm(), 16);
}

/// `test al,unit-1; jz +2; ud2`: #UD unless RAX is a multiple of `unit`.
fn alignment_test(asm: &mut asm::Asm, unit: u8) {
    asm.test_accumulator_imm(asm::Size::Byte, i32::from(unit - 1));
    alignment_tail(asm);
}

/// RAX, the address of the access to come, made one that faults if `failed`.
fn unreachable_if(asm: &mut asm::Asm, failed: asm::Cond) {
    const UNREACHABLE: u32 = core::mem::offset_of!(crate::GuestState, unreachable) as u32;
    asm.cmov(failed, asm::Size::Qword, asm::Gpr::Rax, frame(UNREACHABLE));
}

fn alignment_tail(asm: &mut asm::Asm) {
    if asm.is_pinned() {
        unreachable_if(asm, asm::Cond::Ne);
        return;
    }
    let aligned = asm.jcc8(asm::Cond::E);
    asm.guest_check();
    asm.bind(aligned);
}
fn stack_alignment_check_register(code: &mut Code, register: asm::Gpr) {
    let mut asm = code.asm();
    asm.test_imm(asm::Size::Byte, register, 15);
    alignment_tail(&mut asm);
}

/// Golden bytes of the SP alignment check.
#[cfg(test)]
pub(crate) const STACK_ALIGNMENT_CHECK: [u8; 6] = [0xa8, 15, 0x74, 2, 0x0f, 0x0b];

/// Fragment ends in the independent golden-byte oracle.
#[cfg(test)]
pub(crate) const STACK_ALIGNMENT_CHECK_ENDS: [usize; 3] = [2, 4, 6];

/// The guest range check of `access`'s address in RAX, in the variant
/// `checks` selects, after the tagged variant's strip of its top byte
/// ([`TopByte`]), which leaves RAX the address the access then makes.
pub(crate) fn range_check(code: &mut Code, checks: MemoryChecks, access: Access) {
    checks.top_byte.emit(code);
    if checks.translated {
        checks
            .range
            .emit_translated(code, checks.translation_helper, access);
    } else {
        checks.range.emit(code);
    }
}

fn guest_range_check(code: &mut Code, checks: MemoryChecks, access: Access) {
    if checks.guest_range {
        range_check(code, checks, access);
    }
}

/// The checks of `access`'s final address in RAX: the guest range, then
/// SCTLR_ELx.A against `unit`, the bytes of each element it transfers. The
/// translated variant's host address keeps the guest address's offset in
/// its page, so the alignment checks see the same low bits.
fn address_checks(code: &mut Code, checks: MemoryChecks, unit: u8, access: Access) {
    guest_range_check(code, checks, access);
    alignment_check(code, checks, unit);
}

/// SCTLR_ELx.A's check of the effective address in RAX against `unit`, the
/// bytes of each element the access transfers.
fn alignment_check(code: &mut Code, checks: MemoryChecks, unit: u8) {
    if checks.alignment && unit > 1 {
        debug_assert!(unit.is_power_of_two() && unit <= 16);
        alignment_test(&mut code.asm(), unit);
    }
}

/// RAX is the access address and, for pre- and post-index forms, RDX the
/// pending writeback ([`address_add`]). `unit` is the bytes of each element
/// the access transfers, which SCTLR_ELx.A checks, and `access` the whole
/// access.
fn memory_address(
    code: &mut Code,
    checks: MemoryChecks,
    address: ImmediateAddress,
    unit: u8,
    access: Access,
) {
    memory_recipe::address(code, checks, address, unit, access, None);
}

/// An immediate offset added to the base in RAX, modulo 2^64 and without
/// touching host flags: RAX becomes the access address, and pre- and
/// post-index forms leave the pending writeback in RDX. An Offset access
/// leaves RDX unspecified and emits nothing for a zero offset.
pub(crate) fn address_add(code: &mut Code, displacement: i32, mode: AddressMode) {
    use asm::{
        Gpr::{Rax, Rdx},
        Mem, Size,
    };
    let offset = mode == AddressMode::Offset;
    let mut asm = code.asm();
    if displacement == 0 {
        if !offset {
            asm.mov(Size::Qword, Rdx, Rax);
        }
        return;
    }
    let sum = match i8::try_from(displacement) {
        Ok(short) => Mem::base_disp8(Rax, short),
        Err(_) => Mem::base_disp32(Rax, displacement),
    };
    asm.lea(Size::Qword, if offset { Rax } else { Rdx }, sum);
    if mode == AddressMode::PreIndex {
        asm.mov(Size::Qword, Rax, Rdx);
    }
}

fn memory_writeback(code: &mut Code, base: Register, mode: AddressMode) {
    if mode != AddressMode::Offset {
        let mut asm = code.asm();
        let storage = asm.guest(base).expect("memory base cannot be ZR");
        asm.mov_store(asm::Size::Qword, storage, asm::Gpr::Rdx);
    }
}

/// Pair loads stage both values before committing either guest destination.
/// Pair stores expose completed first-element writes if the second access faults.
/// A 16-byte aligned pair of X registers is one MOVDQA, single-copy atomic as
/// FEAT_LSE2 requires ([`memory_recipe::pair`]).
fn memory_pair(
    code: &mut Code,
    checks: MemoryChecks,
    transfer: MemoryTransfer,
    bytes: u8,
    data: [Register; 2],
    address: ImmediateAddress,
) {
    memory_recipe::pair(
        code,
        checks,
        memory_recipe::Pair {
            transfer,
            bytes,
            data,
            address,
        },
        None,
    );
}

fn right_operand(code: &mut Code, register: Register, width: Width, shift: Shift, amount: u8) {
    right_operand_with(&mut code.asm(), register, width, shift, amount);
}

/// RCX = the guest register (zero for XZR) shifted by `amount`.
fn right_operand_with(
    asm: &mut asm::Asm,
    register: Register,
    width: Width,
    shift: Shift,
    amount: u8,
) {
    use asm::{Alu, Gpr, ShiftOp, Size};
    match asm.guest(register) {
        Some(storage) => asm.mov_load(Size::Qword, Gpr::Rcx, storage),
        None => asm.alu(Alu::Xor, Size::Dword, Gpr::Rcx, Gpr::Rcx),
    }
    if amount != 0 {
        let op = match shift {
            Shift::Lsl => ShiftOp::Shl,
            Shift::Lsr => ShiftOp::Shr,
            Shift::Asr => ShiftOp::Sar,
            Shift::Ror => ShiftOp::Ror,
        };
        asm.shift_imm(op, Size::of(width == Width::X64), Gpr::Rcx, amount);
    }
}

/// A rel32 jump to guest `target`, taken under `cond` or always, whose
/// displacement field `fixups` records for `pc`.
fn jump(
    code: &mut Code,
    cond: Option<asm::Cond>,
    pc: GuestAddress,
    target: GuestAddress,
    fixups: &mut Vec<(GuestAddress, GuestAddress, usize)>,
) {
    let mut asm = code.asm();
    let patch = match cond {
        Some(cond) => asm.jcc(cond),
        None => asm.jmp(),
    };
    fixups.push((pc, target, patch.offset()));
}

/// The host condition under which ARM `condition` holds, from saved NZCV or
/// from the NZCV word already in EAX when `loaded` ([`Rax::Nzcv`]). Clobbers
/// EAX and ECX.
fn condition_holds(code: &mut Code, condition: u8, loaded: bool) -> Option<asm::Cond> {
    use asm::{
        Alu, Cond,
        Gpr::{Rax, Rcx},
        ShiftOp, Size,
    };
    const NZCV: u32 = core::mem::offset_of!(crate::GuestState, nzcv) as u32;
    if condition >= 14 {
        return None;
    } // AL and NV both satisfy ConditionHolds.
    let loaded = if !code.is_pinned() {
        loaded
    } else if code.constant_time() {
        flag_word::load_arm(code, Rax);
        true
    } else {
        return Some(flag_word::condition(code, condition, loaded));
    };
    let mut asm = code.asm();
    let group = condition / 2;
    if group <= 3 {
        let bit = [30, 29, 31, 28][usize::from(group)];
        if loaded {
            asm.test_accumulator_imm(Size::Dword, (1u32 << bit) as i32);
        } else {
            asm.test_imm(Size::Byte, frame(NZCV + bit / 8), 1 << (bit % 8));
        }
        return Some(if condition & 1 == 0 {
            Cond::Ne
        } else {
            Cond::E
        });
    }
    if !loaded {
        asm.mov_load(Size::Dword, Rax, frame(NZCV));
    }
    let (positive, negative) = match group {
        4 => {
            asm.alu_accumulator_imm32(Alu::And, Size::Dword, 0x60000000);
            asm.alu_accumulator_imm32(Alu::Cmp, Size::Dword, 0x20000000);
            (Cond::E, Cond::Ne) // C && !Z
        }
        group => {
            // EAX ^= EAX >> 3 leaves N ^ V in bit 28 and Z in bit 30, whatever
            // the bits below NZCV hold.
            asm.mov(Size::Dword, Rcx, Rax);
            asm.shift_imm(ShiftOp::Shr, Size::Dword, Rcx, 3);
            asm.alu(Alu::Xor, Size::Dword, Rax, Rcx);
            let mask: u32 = if group == 5 { 0x10000000 } else { 0x50000000 };
            asm.test_accumulator_imm(Size::Dword, mask as i32);
            (Cond::E, Cond::Ne) // N == V, and for GT !Z
        }
    };
    Some(if condition & 1 == 0 {
        positive
    } else {
        negative
    })
}

/// What RAX holds after an instruction's emission, for the next one to
/// use instead of reloading it from the frame.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Rax {
    Unknown,
    /// The emission ended by storing the NZCV word in EAX ([`nzcv`]).
    Nzcv,
    /// The canonical NZCV is settled and the normalized host flags still
    /// hold the same condition.
    LiveNzcv,
    /// The emission ended by storing RAX to this register's frame slot
    /// ([`store`]).
    Guest(Register),
    /// The direct Q pair store left its offset address in RAX and did
    /// not change the guest base register.
    PairStoreAddress {
        base: Register,
        displacement: i32,
    },
}

/// What RAX holds after `operation`'s emission by [`emit_instruction`].
pub(super) fn rax_after(operation: &Operation, code: &Code, checks: MemoryChecks) -> Rax {
    match *operation {
        Operation::Memory {
            transfer: MemoryTransfer::Load { .. },
            data,
            mode: AddressMode::Offset,
            ..
        }
        | Operation::MemoryRegisterOffset(crate::aarch64::register_offset::Operand {
            transfer: MemoryTransfer::Load { .. },
            data,
            ..
        }) if (code.is_pinned() || !code.constant_time()) && data != Register::Zero => {
            Rax::Guest(data)
        }
        Operation::Simd(crate::aarch64::simd::Operand::MemoryPair {
            load: false,
            bytes: 16,
            base: base @ Register::X(_),
            displacement,
            mode: AddressMode::Offset,
            ..
        }) if !code.is_pinned() && !code.constant_time() && checks == MemoryChecks::DEFAULT => {
            // The ordinary recipe forms base + displacement in RAX.
            // Vector reads and both native stores leave it in place.
            Rax::PairStoreAddress { base, displacement }
        }
        Operation::Bitfield(operand) if bitfield::is_low_dword_insert(&operand) => {
            // This recipe writes the frame or pinned destination directly.
            Rax::Unknown
        }
        Operation::Bitfield(crate::aarch64::bitfield::Operand { destination, .. })
        | Operation::PcRelative(crate::aarch64::pc_relative::Operand { destination, .. })
            if !code.constant_time() && destination != Register::Zero =>
        {
            Rax::Guest(destination)
        }
        Operation::AddSub { flags: true, .. }
        | Operation::AddSubRegister { flags: true, .. }
        | Operation::AddSubExtended { flags: true, .. }
        | Operation::AddSubCarry { flags: true, .. }
        | Operation::LogicalImmediate { flags: true, .. }
        | Operation::LogicalRegister { flags: true, .. } => Rax::Nzcv,
        Operation::AddSub { destination, .. }
        | Operation::AddSubRegister { destination, .. }
        | Operation::AddSubExtended { destination, .. }
        | Operation::AddSubCarry { destination, .. }
        | Operation::LogicalImmediate { destination, .. }
        | Operation::LogicalRegister { destination, .. }
        | Operation::MoveWide { destination, .. }
        | Operation::Simd(
            crate::aarch64::simd::Operand::Immediate(
                crate::aarch64::simd::immediates::Operand::LaneMove(
                    crate::aarch64::simd::rearrange::Operand::Extract { destination, .. },
                ),
            )
            | crate::aarch64::simd::Operand::Rearrange(
                crate::aarch64::simd::rearrange::Operand::Extract { destination, .. },
            ),
        ) if destination != Register::Zero => Rax::Guest(destination),
        _ => Rax::Unknown,
    }
}

/// [`load`]'s bytes for a register, but for XZR.
pub(super) fn register_load(scratch: &mut Code, register: Register, pinned: bool) -> Option<&[u8]> {
    offset(register)?;
    scratch.clear();
    scratch.set_pinned(pinned);
    load(scratch, register);
    Some(scratch)
}

/// [`store`]'s bytes for a register, but for XZR.
pub(super) fn register_store(
    scratch: &mut Code,
    register: Register,
    pinned: bool,
) -> Option<&[u8]> {
    offset(register)?;
    scratch.clear();
    scratch.set_pinned(pinned);
    store(scratch, register);
    Some(scratch)
}

/// `movabs rax,value`.
fn immediate(code: &mut Code, value: u64) {
    code.asm().mov_imm64(asm::Gpr::Rax, value);
}

/// `movabs rax,address`, its field a relocation of the block.
fn guest_immediate(code: &mut Code, address: GuestAddress) {
    code.asm().guest_imm64(asm::Gpr::Rax, address);
}

fn nzcv(code: &mut Code, subtract: bool) {
    if code.is_pinned() {
        return flag_word::capture(code, subtract);
    }
    capture_nzcv(code, subtract);
    store_nzcv(&mut code.asm());
}

/// `mov dword [rdi+GuestState.nzcv],eax`.
fn store_nzcv(asm: &mut asm::Asm) {
    const NZCV: u32 = core::mem::offset_of!(crate::GuestState, nzcv) as u32;
    asm.mov_store(asm::Size::Dword, frame(NZCV), asm::Gpr::Rax);
}

/// Expected bytes of the capture's `lea eax,[rax+rdx*4]`.
#[cfg(test)]
const CAPTURE_COMBINE: [u8; 3] = [0x8d, 0x04, 0x90];
/// The scale of the capture's LEA: C and V move above N and Z.
const CAPTURE_CARRY_SHIFT: u8 = 2;
/// The SETcc bytes' low bits.
#[cfg(test)]
const CAPTURED_BITS: u32 = 0x0101;
/// One product moves N (bit 0) to 31, Z (8) to 30, C (2) to 29 and V (10)
/// to 28. Every other partial product of the low 16 bits lands below bit 28
/// or above bit 31, on a bit no other lands on, so nothing carries into
/// NZCV; those of bits 16 and up, which the SETcc leave as they were, land
/// at bit 34 or above and drop out of the 32-bit product.
const CAPTURE_MULTIPLIER: u32 = (1 << 31) | (1 << 22) | (1 << 27) | (1 << 18);
const CAPTURE_NZCV_MASK: u32 = 0xf000_0000;
const _: () = assert!(CAPTURE_MULTIPLIER.trailing_zeros() + u16::BITS >= u32::BITS);

/// The host flags as an Arm NZCV word in EAX, without a data-dependent
/// branch or an instruction outside Intel's data operand independent timing
/// list (SETcc, LEA, IMUL, AND). `borrow` takes C as an Arm subtraction
/// does, the complement of the host carry. Clobbers RDX; RCX is intact.
pub(crate) fn capture_nzcv(code: &mut Code, borrow: bool) {
    use asm::{
        Alu, Cond,
        Gpr::{Rax, Rdx},
        Mem, Size,
        control::HighByte,
    };
    let mut asm = code.asm();
    // N | Z << 8 in AX and C | V << 8 in DX.
    asm.setcc(Cond::S, Rax);
    asm.setcc_high(Cond::E, HighByte::Ah);
    asm.setcc(if borrow { Cond::Ae } else { Cond::B }, Rdx);
    asm.setcc_high(Cond::O, HighByte::Dh);
    // N | C << 2 | Z << 8 | V << 10 in the low 16 bits.
    asm.lea(
        Size::Dword,
        Rax,
        Mem::indexed(Rax, Rdx, 1 << CAPTURE_CARRY_SHIFT, 0),
    );
    asm.imul_imm32(Size::Dword, Rax, Rax, CAPTURE_MULTIPLIER as i32);
    asm.alu_accumulator_imm32(Alu::And, Size::Dword, CAPTURE_NZCV_MASK as i32);
}

/// Shared semantic emission. Block compilation handles control transfers itself.
pub(crate) fn emit_instruction(
    code: &mut Code,
    instruction: &Instruction,
    fixups: &mut Vec<(GuestAddress, GuestAddress, usize)>,
    target: crate::TargetCpu,
    checks: MemoryChecks,
) {
    use asm::{
        Alu,
        Gpr::{Rax, Rcx},
    };
    if pinned::emit_direct(code, &instruction.operation, target) {
        return;
    }
    match instruction.operation {
        Operation::AddSub { .. }
        | Operation::AddSubRegister { .. }
        | Operation::LogicalImmediate { .. }
        | Operation::LogicalRegister { .. } => {
            unreachable!("common integer emitter handled the operation")
        }
        Operation::FloatingPoint(_) => {
            unreachable!("numerical instructions require block emission")
        }
        Operation::PointerAuthentication(_) => {
            unreachable!("PAC instructions inside a block require its emission")
        }
        Operation::MirroredRead { .. } => {
            unreachable!("mirrored reads require block emission with their cells")
        }
        Operation::Amx(_) => {
            unreachable!("AMX moves require block emission with the register file")
        }
        Operation::ZeroBlock { .. } => {
            unreachable!("DC ZVA inside a block requires its block size")
        }
        Operation::FloatingPointRegister(operand) => fp_register::emit(code, &operand),
        Operation::Simd(operand) => simd::emit_for_target(code, &operand, target, checks),
        Operation::Bitfield(operand) => bitfield::emit(code, &operand),
        Operation::PcRelative(operand) => pc_relative::emit(code, &operand),
        Operation::ConditionalSelect(operand) => conditional_select::emit(code, &operand, false),
        Operation::ConditionalCompare(operand) => {
            conditional_select::emit_compare(code, &operand, false)
        }
        Operation::Crc(operand) => crc::emit(code, &operand, target),
        Operation::Flags(operand) => flags::emit(code, &operand),
        Operation::Synchronization(operand) => system::synchronization::emit(code, &operand)
            .expect("decoder admits only context-independent synchronization"),
        Operation::IntegerMath(operand) => integer_math::emit(code, &operand, target),
        Operation::IntegerMinMax(operand) => integer_minmax::emit(code, &operand),
        Operation::IntegerUnary(operand) => integer_unary::emit(code, &operand, target),
        Operation::Literal(operand) => literal::emit(code, &operand, checks),
        Operation::MemoryPair {
            transfer,
            bytes,
            data,
            base,
            displacement,
            mode,
        } => {
            memory_pair(
                code,
                checks,
                transfer,
                bytes,
                data,
                crate::x86_64::ImmediateAddress {
                    base,
                    displacement,
                    mode,
                },
            );
        }
        Operation::Memory {
            transfer,
            bytes,
            data,
            base,
            displacement,
            mode,
        } => {
            memory(
                code,
                checks,
                transfer,
                bytes,
                data,
                ImmediateAddress {
                    base,
                    displacement,
                    mode,
                },
            );
        }
        Operation::Branch { target } => jump(code, None, instruction.pc, target, fixups),
        Operation::BranchCondition {
            target,
            condition: cond,
        } => {
            let holds = condition_holds(code, cond, false);
            jump(code, holds, instruction.pc, target, fixups);
        }
        Operation::BranchZero {
            target,
            source,
            width,
            nonzero,
        } => {
            load(code, source);
            let holds = test_zero(code, width, nonzero);
            jump(code, Some(holds), instruction.pc, target, fixups);
        }
        Operation::BranchBit {
            target,
            source,
            bit,
            nonzero,
        } => {
            load(code, source);
            let holds = test_bit(code, bit, nonzero);
            jump(code, Some(holds), instruction.pc, target, fixups);
        }
        Operation::AddSubCarry {
            width,
            subtract,
            flags,
            destination,
            left,
            right,
        } => {
            load(code, left);
            right_operand(code, right, width, Shift::Lsl, 0);
            // Arm's SBC is AddWithCarry(left, NOT(right), C): ADC of the
            // complement, whose host CF and OF are Arm's C and V. (CMC is
            // not on Intel's data operand independent timing list.)
            const CARRY_BIT: u8 = 29;
            let size = asm::Size::of(width == Width::X64);
            let mut asm = code.asm();
            if subtract {
                asm.unary(asm::Unary::Not, size, Rcx);
            }
            // Recover guest C after operand loads, including XOR-based
            // ZR loads that clobber host flags.
            flag_word::carry(&mut asm, CARRY_BIT);
            asm.alu(Alu::Adc, size, Rax, Rcx);
            store(code, destination);
            if flags {
                nzcv(code, false);
            }
        }
        Operation::AddSubExtended {
            width,
            subtract,
            flags,
            destination,
            left,
            right,
            extend,
            amount,
        } => {
            // Read both operands before a possibly overlapping destination/SP write.
            load(code, left);
            extended_operand(code, right, extend, amount);
            // W32 arithmetic truncates both operands and zero-extends even WSP.
            add_sub(code, width, subtract);
            store(code, destination);
            if flags {
                nzcv(code, subtract);
            }
        }
        Operation::MemoryRegisterOffset(operand) => register_offset::emit(code, &operand, checks),
        Operation::MemoryNonTemporal(operand) => non_temporal::emit(code, &operand, checks),
        Operation::MemoryUnprivileged(operand) => unprivileged::emit(code, &operand, checks),
        Operation::TagArithmetic(operand) => tag_arithmetic::emit(code, &operand),
        Operation::Atomic(operand) => atomic::emit(code, &operand, target, checks),
        Operation::Ordered(operand) => ordered::emit(code, &operand, target, checks),
        Operation::Prefetch(operand) => prefetch::emit(code, &operand),
        Operation::Return => code.asm().ret(),
        Operation::Exclusive(_) => unreachable!("exclusive requires checked context emitter"),
        Operation::CheckedPointer(_) => {
            unreachable!("CPA arithmetic requires checked context emitter")
        }
        Operation::Nop => code.asm().nop(),
        Operation::MoveWide {
            width,
            kind,
            destination,
            immediate: value,
            shift,
        } => {
            use asm::Size::{Dword, Qword};
            let value = u64::from(value) << shift;
            match kind {
                WideMove::Zero => immediate(code, value),
                WideMove::Not => immediate(code, !value),
                WideMove::Keep => {
                    load(code, destination);
                    let mut asm = code.asm();
                    asm.mov_imm64(Rcx, !(0xffffu64 << shift));
                    asm.alu(Alu::And, Qword, Rax, Rcx);
                    asm.mov_imm64(Rcx, value);
                    asm.alu(Alu::Or, Qword, Rax, Rcx);
                }
            }
            if width == Width::W32 {
                // Zero-extend.
                code.asm().mov(Dword, Rax, Rax);
            }
            store(code, destination);
        }
    }
}

impl Backend<Instruction> for Emitter {
    fn emit(&self, instructions: &[Instruction]) -> Result<NativeBlock, UnsupportedReason> {
        self.emit_for_target(instructions, crate::TargetCpu::Baseline)
    }
}
impl Emitter {
    pub fn emit_for_target(
        &self,
        instructions: &[Instruction],
        target: crate::TargetCpu,
    ) -> Result<NativeBlock, UnsupportedReason> {
        let mut block = Emitted::default();
        let code = &mut block.x86_64_code;
        let mut fixups = vec![];
        for instruction in instructions {
            validate_target(instruction, target)?;
            if matches!(instruction.operation, Operation::FloatingPoint(_)) {
                return Err(UnsupportedReason::NumericalRuntimeRequired {
                    address: instruction.pc,
                });
            }
            block.source_map.push((instruction.pc, code.len()));
            emit_instruction(
                code,
                instruction,
                &mut fixups,
                target,
                MemoryChecks::DEFAULT,
            );
        }
        let offsets: alloc::collections::BTreeMap<_, _> = block
            .source_map
            .iter()
            .map(|(pc, offset)| (pc.0, *offset))
            .collect();
        for (pc, target, displacement) in fixups {
            let error = || UnsupportedReason::UnresolvedDirectTarget {
                address: pc,
                target,
            };
            let destination = *offsets.get(&target.0).ok_or_else(error)?;
            let relative = destination as i128 - (displacement + 4) as i128;
            let relative = i32::try_from(relative).map_err(|_| error())?;
            code.patch(displacement, relative.to_le_bytes());
        }
        Ok(block.into_native())
    }
}

#[cfg(test)]
mod range_check_tests;
#[cfg(test)]
mod tests;
