// SPDX-License-Identifier: GPL-2.0-only
//! LAPIC idle/rate eligibility, including Linux init_amd family rules.
use super::super::WorkError;
use core::arch::x86_64::__cpuid_count;
use stela_boot_core::tsc::{Deadline, DeadlineInputs, Leaf};

/// The captured processor's deadline inputs: its vendor, family and ARAT.
/// Each check compares these alone, not whole leaves, whose other bits
/// follow controls that change while the timer runs ([`DeadlineInputs`]).
pub(in crate::native_clock) struct Control {
    inputs: DeadlineInputs,
}
fn query(leaf: u32) -> Leaf {
    let value = __cpuid_count(leaf, 0);
    [value.eax, value.ebx, value.ecx, value.edx]
}
fn observe() -> Result<DeadlineInputs, WorkError> {
    let identity = query(0);
    if identity[0] < 1 {
        return Err(WorkError::Unavailable);
    }
    let thermal = (identity[0] >= 6).then(|| query(6));
    Ok(DeadlineInputs::observe(identity, query(1), thermal))
}
impl Control {
    pub(in crate::native_clock) fn capture() -> Result<Self, WorkError> {
        let inputs = observe()?;
        if inputs.policy() != Deadline::Arat {
            return Err(WorkError::Unavailable);
        }
        Ok(Self { inputs })
    }
    pub(in crate::native_clock) fn check(&self) -> Result<(), WorkError> {
        if observe()? != self.inputs {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
}
