// SPDX-License-Identifier: GPL-2.0-only
//! Live HPET eligibility following Linux 7.2.12 cc9de5d4 hpet.c:
//! mwait_pc10_supported/hpet_is_pc10_damaged. No idle limit is written.
use super::super::WorkError;
use core::arch::x86_64::__cpuid_count;
use stela_boot_core::{
    topology::{Cpuid, Msr, Snapshot},
    tsc,
};
const PACKAGE_LIMIT: u32 = 0xe2;
const INTEL: [u32; 3] = [
    u32::from_le_bytes(*b"Genu"),
    u32::from_le_bytes(*b"ineI"),
    u32::from_le_bytes(*b"ntel"),
];

#[derive(Clone, Copy)]
pub(in crate::native_clock) struct PowerControl {
    identity: [u32; 4],
    basic: [u32; 4],
    monitor: Option<[u32; 4]>,
    package: Option<u64>,
}
fn query(leaf: u32) -> [u32; 4] {
    let value = __cpuid_count(leaf, 0);
    [value.eax, value.ebx, value.ecx, value.edx]
}
fn record(leaf: u32, words: [u32; 4]) -> Cpuid {
    Cpuid {
        leaf,
        subleaf: 0,
        eax: words[0],
        ebx: words[1],
        ecx: words[2],
        edx: words[3],
    }
}
impl PowerControl {
    /// # Safety
    /// Caller checked its original privileged process CPU. CPUID observations
    /// select which controls are architecturally readable.
    pub(in crate::native_clock) unsafe fn capture() -> Result<Self, WorkError> {
        let identity = query(0);
        if identity[0] < 1 {
            return Err(WorkError::Unavailable);
        }
        let basic = query(1);
        let monitor = (identity[0] >= 5).then(|| query(5));
        let pc10 = [identity[1], identity[3], identity[2]] == INTEL
            && basic[2] & (1 << 3) != 0
            && monitor.is_some_and(|value| value[2] & 3 == 3 && value[3] & (15 << 28) != 0);
        let package = if pc10 {
            // SAFETY: the Intel MWAIT/PC10 enumeration above is the
            // Linux gate for MSR_PKG_CST_CONFIG_CONTROL on this owning CPU.
            Some(unsafe { crate::cpu::msr::rdmsr(PACKAGE_LIMIT) })
        } else {
            None
        };
        // Only construction uses the large capture representation. IRQ reads
        // keep the small exact observation above and never build a Snapshot.
        let mut snapshot = Snapshot::default();
        snapshot.records[0] = record(0, identity);
        snapshot.records[1] = record(1, basic);
        snapshot.count = 2;
        if let Some(value) = monitor {
            snapshot.records[2] = record(5, value);
            snapshot.count = 3;
        }
        if let Some(value) = package {
            snapshot.msrs[0] = Msr {
                index: PACKAGE_LIMIT,
                value,
            };
            snapshot.msr_count = 1;
        }
        if let Err(error) = tsc::hpet_power_constraint(&snapshot) {
            crate::output::arguments(format_args!("NATIVE HPET POLICY REFUSE {error:?}\n"));
            return Err(WorkError::Unavailable);
        }
        Ok(Self {
            identity,
            basic,
            monitor,
            package,
        })
    }

    /// # Safety
    /// Caller checked the same privileged CPU as capture. Changed
    /// observations invalidate this source epoch.
    pub(in crate::native_clock) unsafe fn check(&self) -> Result<(), WorkError> {
        if query(0) != self.identity
            || query(1) != self.basic
            || self.monitor.is_some_and(|value| query(5) != value)
        {
            return Err(WorkError::corrupt());
        }
        if let Some(expected) = self.package {
            // SAFETY: the live CPUID gate was rechecked above; this
            // source only observes the control and never disables deep idle.
            if unsafe { crate::cpu::msr::rdmsr(PACKAGE_LIMIT) } != expected {
                return Err(WorkError::corrupt());
            }
        }
        Ok(())
    }
}
