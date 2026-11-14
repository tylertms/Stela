//! Model-specific registers (Intel SDM Vol. 4; AMD APM Vol. 2 appendix A).
//! Each `Msr` names one register and the enumeration that says this CPU
//! implements it: a CPUID bit, a bit of another MSR, the vendor, or x86-64
//! itself. `read` and `write` take the caller's proof of that enumeration
//! (`implemented`). Every MSR access in the mount goes through this module,
//! except the AP startup assembly in `native_smp::bootstrap`, which runs
//! before Rust code can.
use core::arch::{
    asm,
    x86_64::{__cpuid, __cpuid_count},
};

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Register {
    Eax,
    Ebx,
    Ecx,
    Edx,
}

/// One CPUID bit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bit {
    pub leaf: u32,
    pub subleaf: u32,
    pub register: Register,
    pub bit: u8,
}

impl Bit {
    pub const fn new(leaf: u32, register: Register, bit: u8) -> Self {
        Self {
            leaf,
            subleaf: 0,
            register,
            bit,
        }
    }

    pub const fn sub(leaf: u32, subleaf: u32, register: Register, bit: u8) -> Self {
        Self {
            leaf,
            subleaf,
            register,
            bit,
        }
    }

    /// The bit is set, with `leaf` within the CPU's highest leaf of its range.
    pub fn present(self) -> bool {
        if self.leaf > __cpuid(self.leaf & 0x8000_0000).eax {
            return false;
        }
        let result = __cpuid_count(self.leaf, self.subleaf);
        let value = match self.register {
            Register::Eax => result.eax,
            Register::Ebx => result.ebx,
            Register::Ecx => result.ecx,
            Register::Edx => result.edx,
        };
        value & (1 << self.bit) != 0
    }
}

/// What enumerates an MSR.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Gate {
    /// Every x86-64 CPU implements it.
    LongMode,
    /// CPUID sets this bit.
    Cpuid(Bit),
    /// CPUID sets any of these bits.
    AnyCpuid(&'static [Bit]),
    /// A bit of another, itself enumerated, MSR.
    Msr(&'static Msr, u64),
    /// An AMD CPU of family 0Fh or later (APM Vol. 2 appendix A.7).
    Amd,
}

/// One model-specific register.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Msr {
    pub index: u32,
    pub gate: Gate,
}

pub const AMD_VENDOR: [u32; 3] = [
    u32::from_le_bytes(*b"Auth"),
    u32::from_le_bytes(*b"enti"),
    u32::from_le_bytes(*b"cAMD"),
];

impl Msr {
    pub const fn new(index: u32, gate: Gate) -> Self {
        Self { index, gate }
    }

    /// The register `offset` past this one in its family (MTRR pairs, x2APIC
    /// registers), with the same enumeration.
    pub const fn at(self, offset: u32) -> Self {
        Self {
            index: self.index + offset,
            gate: self.gate,
        }
    }

    /// This CPU implements the register. CPUID is serializing and traps in a
    /// guest, so callers test once and keep the answer.
    pub fn implemented(self) -> bool {
        match self.gate {
            Gate::LongMode => true,
            Gate::Cpuid(bit) => bit.present(),
            Gate::AnyCpuid(bits) => bits.iter().any(|bit| bit.present()),
            // SAFETY: `implemented` checks the enumerating MSR first; reading
            // an implemented MSR has no effects.
            Gate::Msr(msr, bit) => msr.implemented() && unsafe { rdmsr(msr.index) } & bit != 0,
            Gate::Amd => {
                let vendor = __cpuid(0);
                let family = (__cpuid(1).eax >> 8) & 0xf;
                [vendor.ebx, vendor.edx, vendor.ecx] == AMD_VENDOR && family == 0xf
            }
        }
    }

    /// # Safety
    /// CPL0, and this CPU implements the register (`implemented`).
    pub unsafe fn read(self) -> u64 {
        // SAFETY: the caller's contract.
        unsafe { rdmsr(self.index) }
    }

    /// # Safety
    /// CPL0, this CPU implements the register, and `value` is valid for it
    /// with every effect the write has on this processor.
    pub unsafe fn write(self, value: u64) {
        // SAFETY: the caller's contract.
        unsafe { wrmsr(self.index, value) }
    }
}

/// RDMSR of `index`.
///
/// # Safety
/// CPL0, and this CPU implements `index`. Callers with a fixed register use
/// its `Msr`; this serves indices taken from a recorded list.
pub unsafe fn rdmsr(index: u32) -> u64 {
    let (low, high): (u32, u32);
    // SAFETY: the caller's contract.
    unsafe {
        asm!("rdmsr", in("ecx") index, out("eax") low, out("edx") high,
            options(nomem, nostack, preserves_flags));
    }
    u64::from(high) << 32 | u64::from(low)
}

/// WRMSR of `value` to `index`.
///
/// # Safety
/// As `Msr::write`, for `index`.
pub unsafe fn wrmsr(index: u32, value: u64) {
    // SAFETY: the caller's contract.
    unsafe {
        asm!("wrmsr", in("ecx") index, in("eax") value as u32, in("edx") (value >> 32) as u32,
            options(nostack, preserves_flags));
    }
}

/// WRMSR after MFENCE and LFENCE, so earlier stores are globally visible
/// before a non-serializing WRMSR (x2APIC ICR) takes effect: Linux's
/// `weak_wrmsr_fence`.
///
/// # Safety
/// As `Msr::write`.
pub unsafe fn write_fenced(msr: Msr, value: u64) {
    // SAFETY: the caller's contract.
    unsafe {
        asm!("mfence", "lfence", "wrmsr", in("ecx") msr.index, in("eax") value as u32,
            in("edx") (value >> 32) as u32, options(nostack, preserves_flags));
    }
}

use Register::{Eax, Ebx, Ecx, Edx};

const APIC: Bit = Bit::new(1, Edx, 9);
const MTRR: Bit = Bit::new(1, Edx, 12);
/// Machine-check architecture (CPUID 1 EDX 14).
const MCA: Bit = Bit::new(1, Edx, 14);
const PAT_BIT: Bit = Bit::new(1, Edx, 16);
const X2APIC_BIT: Bit = Bit::new(1, Ecx, 21);
const TSC_ADJUST_BIT: Bit = Bit::new(7, Ebx, 1);
const ARCH_CAPABILITIES_BIT: Bit = Bit::new(7, Edx, 29);
/// Intel IBRS/IBPB (CPUID 7 EDX 26), AMD IBPB (8000_0008h EBX 12).
const IBPB: [Bit; 2] = [Bit::new(7, Edx, 26), Bit::new(0x8000_0008, Ebx, 12)];
/// XSAVES (0Dh.1 EAX 3) and XFD (0Dh.1 EAX 4).
const XSAVES: Bit = Bit::sub(0xd, 1, Eax, 3);
const XFD_BIT: Bit = Bit::sub(0xd, 1, Eax, 4);

pub const APIC_BASE: Msr = Msr::new(0x1b, Gate::Cpuid(APIC));
pub const TSC_ADJUST: Msr = Msr::new(0x3b, Gate::Cpuid(TSC_ADJUST_BIT));
pub const PRED_CMD: Msr = Msr::new(0x49, Gate::AnyCpuid(&IBPB));
pub const MTRR_CAP: Msr = Msr::new(0xfe, Gate::Cpuid(MTRR));
pub const ARCH_CAPABILITIES: Msr = Msr::new(0x10a, Gate::Cpuid(ARCH_CAPABILITIES_BIT));
/// IA32_MCG_CAP; with it IA32_MCG_STATUS and the banks it counts
/// (`machine_check`).
pub const MCG_CAP: Msr = Msr::new(crate::machine_check::MCG_CAP, Gate::Cpuid(MCA));
/// IA32_MTRR_PHYSBASE0; PHYSMASKn and later pairs follow (`at`).
pub const MTRR_PHYS_BASE: Msr = Msr::new(0x200, Gate::Cpuid(MTRR));
pub const XFD: Msr = Msr::new(0x1c4, Gate::Cpuid(XFD_BIT));
pub const PAT: Msr = Msr::new(0x277, Gate::Cpuid(PAT_BIT));
pub const MTRR_DEFAULT: Msr = Msr::new(0x2ff, Gate::Cpuid(MTRR));
pub const XSS: Msr = Msr::new(0xda0, Gate::Cpuid(XSAVES));
/// The first x2APIC register; register `r` of the xAPIC page is at `r >> 4`.
pub const X2APIC: Msr = Msr::new(0x800, Gate::Cpuid(X2APIC_BIT));
/// Enumerated by IA32_ARCH_CAPABILITIES bit 12 (DOITM).
pub const UARCH_MISC_CTL: Msr = Msr::new(0x1b01, Gate::Msr(&ARCH_CAPABILITIES, 1 << 12));
pub const EFER: Msr = Msr::new(0xc000_0080, Gate::LongMode);
pub const GS_BASE: Msr = Msr::new(0xc000_0101, Gate::LongMode);
pub const AMD_SYSCFG: Msr = Msr::new(0xc001_0010, Gate::Amd);

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn family_members_keep_their_enumeration() {
        let mask = MTRR_PHYS_BASE.at(1);
        assert_eq!(mask.index, 0x201);
        assert_eq!(mask.gate, MTRR_PHYS_BASE.gate);
        assert_eq!(X2APIC.at(0x30 >> 4).index, 0x803);
    }

    #[test]
    fn long_mode_registers_need_no_cpuid() {
        assert!(EFER.implemented());
        assert!(GS_BASE.implemented());
        // The hosted CPU answers CPUID gates like any other.
        assert_eq!(PAT.implemented(), PAT_BIT.present());
    }
}
