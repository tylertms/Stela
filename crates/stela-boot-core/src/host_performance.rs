//! The host processor's own performance request. After ExitBootServices no
//! operating system asks the CPU for speed, so a core runs at whatever its
//! firmware left: with hardware-controlled P-states (Intel HWP, AMD CPPC)
//! not enabled, or enabled with a balanced energy preference, a busy core
//! may stay below its highest clock. Each PE's processor asks for the
//! performance a busy guest needs, as Linux's `performance` policy does in
//! active mode (intel_pstate HWP, amd-pstate EPP): the lowest request is the
//! guaranteed (Intel) or nominal (AMD) level, the highest is the highest
//! level, the desired level is left to the hardware, and the energy
//! preference is 0, full performance. Idle states are unchanged: a waiting
//! processor still sleeps (`native_idle`).
//!
//! Registers: Intel SDM Vol. 3B 15.4 (CPUID.06H:EAX[7] HWP, [10] EPP;
//! IA32_PM_ENABLE 770H, IA32_HWP_CAPABILITIES 771H, IA32_HWP_REQUEST 774H);
//! AMD PPR "Collaborative Processor Performance Control" (CPUID
//! 8000_0008H:EBX[27]; CPPC_CAP1 C001_02B0H, CPPC_ENABLE C001_02B1H,
//! CPPC_REQ C001_02B3H). Both enables are one-way until reset. A CPU with
//! neither (legacy P-states, or a hypervisor that hides them) is left as the
//! firmware set it and reported.
use crate::msr::{Bit, Gate, Msr, Register};
use alloc::{format, string::String};
use core::sync::atomic::{AtomicU32, Ordering};

const HWP_BIT: Bit = Bit::new(6, Register::Eax, 7);
const HWP_EPP_BIT: Bit = Bit::new(6, Register::Eax, 10);
const AMD_CPPC_BIT: Bit = Bit::new(0x8000_0008, Register::Ebx, 27);

pub const PM_ENABLE: Msr = Msr::new(0x770, Gate::Cpuid(HWP_BIT));
pub const HWP_CAPABILITIES: Msr = Msr::new(0x771, Gate::Cpuid(HWP_BIT));
pub const HWP_REQUEST: Msr = Msr::new(0x774, Gate::Cpuid(HWP_BIT));
pub const AMD_CPPC_CAP1: Msr = Msr::new(0xc001_02b0, Gate::Cpuid(AMD_CPPC_BIT));
pub const AMD_CPPC_ENABLE: Msr = Msr::new(0xc001_02b1, Gate::Cpuid(AMD_CPPC_BIT));
pub const AMD_CPPC_REQ: Msr = Msr::new(0xc001_02b3, Gate::Cpuid(AMD_CPPC_BIT));

/// The performance levels a processor publishes, on its own 8-bit scale.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Levels {
    pub highest: u8,
    /// Intel's guaranteed level; AMD's nominal level.
    pub sustained: u8,
    pub lowest: u8,
}

impl Levels {
    /// IA32_HWP_CAPABILITIES: highest 7:0, guaranteed 15:8, most efficient
    /// 23:16, lowest 31:24.
    pub fn intel(capabilities: u64) -> Self {
        Self {
            highest: capabilities as u8,
            sustained: (capabilities >> 8) as u8,
            lowest: (capabilities >> 24) as u8,
        }
    }

    /// CPPC_CAP1: lowest 7:0, lowest nonlinear 15:8, nominal 23:16,
    /// highest 31:24.
    pub fn amd(capabilities: u64) -> Self {
        Self {
            highest: (capabilities >> 24) as u8,
            sustained: (capabilities >> 16) as u8,
            lowest: capabilities as u8,
        }
    }

    /// The bounds of the request, or none when the published levels are
    /// not ordered (an absent or unprogrammed capability register), so the
    /// firmware's request stays.
    fn bounds(self) -> Option<(u8, u8)> {
        if self.highest == 0 || self.lowest > self.highest {
            return None;
        }
        let minimum = self.sustained.clamp(self.lowest, self.highest);
        Some((minimum, self.highest))
    }
}

/// The request word for `levels`, keeping `current`'s other fields (Intel's
/// activity window and package control, reserved bits): minimum 7:0 (AMD:
/// maximum), maximum 15:8 (AMD: minimum), desired 23:16 = 0 (hardware
/// chooses), energy preference 31:24 = 0.
fn request(current: u64, low: u8, high: u8) -> u64 {
    (current & !0xffff_ffff) | u64::from(low) | u64::from(high) << 8
}

/// The IA32_HWP_REQUEST value for `capabilities`, or none to keep the
/// firmware's request.
pub fn intel_request(capabilities: u64, current: u64) -> Option<u64> {
    let (minimum, maximum) = Levels::intel(capabilities).bounds()?;
    Some(request(current, minimum, maximum))
}

/// The CPPC_REQ value for `capabilities`, or none to keep the firmware's
/// request. AMD orders maximum before minimum.
pub fn amd_request(capabilities: u64, current: u64) -> Option<u64> {
    let (minimum, maximum) = Levels::amd(capabilities).bounds()?;
    Some(request(current, maximum, minimum))
}

/// What this processor's performance request became.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Outcome {
    /// Intel HWP asks for these levels with energy preference 0.
    Hwp { levels: Levels, epp: bool },
    /// AMD CPPC asks for these levels with energy preference 0.
    Cppc { levels: Levels },
    /// The capability register published no usable levels.
    Unpublished,
    /// Neither interface is enumerated; the firmware's P-state stays.
    Legacy,
}

impl Outcome {
    fn index(self) -> usize {
        match self {
            Self::Hwp { .. } => 0,
            Self::Cppc { .. } => 1,
            Self::Unpublished => 2,
            Self::Legacy => 3,
        }
    }
}

/// Processors per outcome, in `Outcome::index` order, and the last
/// published levels, for one report after the processors start.
static OUTCOMES: [AtomicU32; 4] = [const { AtomicU32::new(0) }; 4];
static LEVELS: AtomicU32 = AtomicU32::new(0);

/// Count one processor's outcome.
pub fn record(outcome: Outcome) {
    OUTCOMES[outcome.index()].fetch_add(1, Ordering::Relaxed);
    if let Outcome::Hwp { levels, .. } | Outcome::Cppc { levels } = outcome {
        let packed = u32::from(levels.highest) << 16
            | u32::from(levels.sustained) << 8
            | u32::from(levels.lowest);
        LEVELS.store(packed, Ordering::Relaxed);
    }
}

/// The recorded outcomes as one report line.
pub fn summary() -> String {
    let count = |index: usize| OUTCOMES[index].load(Ordering::Relaxed);
    let levels = LEVELS.load(Ordering::Relaxed);
    format!(
        "HOST PERFORMANCE requested hwp={} cppc={} unpublished={} legacy={} levels highest/sustained/lowest={}/{}/{} preference=performance\n",
        count(0),
        count(1),
        count(2),
        count(3),
        levels >> 16 & 0xff,
        levels >> 8 & 0xff,
        levels & 0xff,
    )
}

/// Ask this processor for full performance (module documentation).
///
/// # Safety
/// CPL0 on the processor being configured, with interrupts disabled, before
/// guest code runs on it; nothing else on this processor owns its P-state
/// registers.
pub unsafe fn request_full_performance() -> Outcome {
    if HWP_REQUEST.implemented() {
        // SAFETY: CPUID.06H:EAX[7] enumerates the HWP registers; enabling
        // HWP first is the SDM's order, as the request register faults
        // while HWP is disabled. Each value keeps the reserved fields.
        unsafe {
            let enable = PM_ENABLE.read();
            if enable & 1 == 0 {
                PM_ENABLE.write(enable | 1);
            }
            let capabilities = HWP_CAPABILITIES.read();
            let Some(value) = intel_request(capabilities, HWP_REQUEST.read()) else {
                return Outcome::Unpublished;
            };
            HWP_REQUEST.write(value);
            return Outcome::Hwp {
                levels: Levels::intel(capabilities),
                epp: HWP_EPP_BIT.present(),
            };
        }
    }
    if AMD_CPPC_REQ.implemented() {
        // SAFETY: CPUID 8000_0008H:EBX[27] enumerates the CPPC registers;
        // the enable precedes the request as amd-pstate orders them.
        unsafe {
            let enable = AMD_CPPC_ENABLE.read();
            if enable & 1 == 0 {
                AMD_CPPC_ENABLE.write(enable | 1);
            }
            let capabilities = AMD_CPPC_CAP1.read();
            let Some(value) = amd_request(capabilities, AMD_CPPC_REQ.read()) else {
                return Outcome::Unpublished;
            };
            AMD_CPPC_REQ.write(value);
            return Outcome::Cppc {
                levels: Levels::amd(capabilities),
            };
        }
    }
    Outcome::Legacy
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn intel_requests_guaranteed_to_highest_with_full_performance_preference() {
        // highest 0x30, guaranteed 0x24, most efficient 0x10, lowest 0x08.
        let capabilities = 0x0810_2430;
        // Firmware: balanced EPP 0x80, desired 0, activity window and
        // package control set.
        let current = 0x0000_0601_8000_3008;
        let value = intel_request(capabilities, current).unwrap();
        assert_eq!(value & 0xff, 0x24, "minimum is the guaranteed level");
        assert_eq!(value >> 8 & 0xff, 0x30, "maximum is the highest level");
        assert_eq!(
            value >> 16 & 0xff,
            0,
            "the hardware picks the desired level"
        );
        assert_eq!(value >> 24 & 0xff, 0, "energy preference is performance");
        assert_eq!(value >> 32, 0x601, "activity window and package bits stay");
    }

    #[test]
    fn amd_orders_maximum_before_minimum() {
        // CAP1: highest 0xa6, nominal 0x7c, lowest nonlinear 0x38, lowest 0x12.
        let capabilities = 0xa67c_3812;
        let current = 0xdead_0000_8000_12a6;
        let value = amd_request(capabilities, current).unwrap();
        assert_eq!(value & 0xff, 0xa6, "maximum is the highest level");
        assert_eq!(value >> 8 & 0xff, 0x7c, "minimum is the nominal level");
        assert_eq!(value >> 16 & 0xff, 0);
        assert_eq!(value >> 24 & 0xff, 0);
        assert_eq!(value >> 32, 0xdead_0000, "upper bits stay");
    }

    #[test]
    fn unpublished_or_unordered_levels_keep_the_firmware_request() {
        assert_eq!(intel_request(0, 0x8000_ff01), None);
        assert_eq!(amd_request(0, 0x8000_ff01), None);
        // Lowest above highest.
        assert_eq!(intel_request(0x4000_0020, 0), None);
    }

    #[test]
    fn the_summary_counts_outcomes_and_keeps_the_published_levels() {
        let levels = Levels::amd(0xa67c_3812);
        record(Outcome::Cppc { levels });
        record(Outcome::Legacy);
        let line = summary();
        assert!(line.starts_with("HOST PERFORMANCE requested "));
        assert!(line.contains("levels highest/sustained/lowest=166/124/18"));
    }

    #[test]
    fn a_sustained_level_outside_the_range_is_clamped() {
        // Guaranteed 0x40 above highest 0x30 (a part that reports its
        // guaranteed level above a reduced highest level).
        let value = intel_request(0x0800_4030, 0).unwrap();
        assert_eq!(value & 0xff, 0x30);
        // Guaranteed 0 below lowest 8.
        let value = intel_request(0x0800_0030, 0).unwrap();
        assert_eq!(value & 0xff, 0x08);
    }
}
