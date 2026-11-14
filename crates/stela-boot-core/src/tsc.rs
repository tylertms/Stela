// SPDX-License-Identifier: GPL-2.0-only
//! Pure native-time policy and calibration arithmetic. Rules follow pinned Linux
//! 7.2.12 cc9de5d4b5ab6459f5774946aa3195ad51236fa8 arch/x86/kernel/{tsc,tsc_sync,hpet}.c
//! and arch/x86/include/asm/tsc.h. These values grant no CPU, clock, register,
//! mapping, lifecycle, synchronization or interrupt permission. Native admission
//! must observe the real owner/controls, reliability and progress.
mod calibration;
mod minimum;
pub use minimum::LowerScale;
mod features;
mod measurement;
mod rate;
mod reference;
pub use rate::NominalRate;
pub use reference::{Calibrated, Measured, Reference, WINDOW_NS, Window, measure, settle};
mod sequence;
pub mod sync;
pub use calibration::{RateWindow, RationalRate, ReferenceScale, Sample, wrapped_delta};
pub use features::{
    CounterPolicy, Deadline, DeadlineInputs, HpetPowerConstraint, Leaf, RateRequirement,
    Reliability, Serialization, counter_policy, deadline_policy, hpet_power_constraint,
};
pub use measurement::{Calibration, WindowEvidence, calibrate_hpet};
pub use sequence::CounterSequence;
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    MalformedCapture,
    MissingCpuid(u32),
    DuplicateCpuid(u32),
    MissingMsr(u32),
    DuplicateMsr(u32),
    TscUnavailable,
    InvariantUnavailable,
    HypervisorContractRequired,
    InvalidPackageCount,
    Pc10StopsHpet,
    InvalidRate,
    InvalidMask,
    NoProgress,
    Backwards,
    Disturbed,
    Overflow,
    InconsistentRate,
    AdjustmentChanged,
}
#[cfg(test)]
mod tests;
