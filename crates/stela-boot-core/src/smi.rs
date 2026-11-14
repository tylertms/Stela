//! System management interrupts counted over host service turns, so that a
//! slow turn's line (`HOST SERVICE SLOW ... smis=`) confirms or refutes
//! that firmware SMIs, stretched by processors that never answer them,
//! took its time. Intel counts them in `MSR_SMI_COUNT` (SDM Vol. 4, every
//! family 06H processor since Nehalem); the read is probed, and a refused
//! one means no counter. AMD documents no counter register: its SMIs are a
//! core performance event, which the profile's counters own, so AMD
//! processors report none.

/// Intel's `MSR_SMI_COUNT`; bits 31:0 count.
pub const INTEL_SMI_COUNT: u32 = 0x34;

/// The counter register of a processor whose vendor is Intel or not.
pub fn counter(intel: bool) -> Option<u32> {
    intel.then_some(INTEL_SMI_COUNT)
}

/// The SMIs between two reads of the counter, across its wrap.
pub fn taken(before: u64, after: u64) -> u32 {
    (after as u32).wrapping_sub(before as u32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn only_intel_has_the_counter() {
        assert_eq!(counter(true), Some(0x34));
        assert_eq!(counter(false), None);
    }

    #[test]
    fn taken_counts_across_the_wrap_of_the_low_word() {
        assert_eq!(taken(5, 9), 4);
        assert_eq!(taken(0xffff_fffe, 0x1_0000_0001), 3);
        // Bits above 31 are reserved and ignored.
        assert_eq!(taken(0xdead_0000_0000_0007, 7), 0);
    }
}
