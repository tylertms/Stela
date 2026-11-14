//! The report of a machine check: the global status and every error
//! reporting bank that holds an error (Intel SDM Vol. 3B 16.3, AMD APM
//! Vol. 2 9.3), read on the processor that took #MC and added to the
//! engine's fatal report, which the stop record keeps
//! (`engine::interrupts::fatal`). Nothing is written back: the banks keep
//! the error for the firmware's own report after the restart.

use core::fmt;

/// CR4.MCE (Intel SDM Vol. 3A 2.5): a machine check raises #MC instead of
/// shutting the processor down. Stela sets it on every processor it runs,
/// once a handler takes #MC.
pub const CR4_MCE: u64 = 1 << 6;

/// `IA32_MCG_CAP`; its bits 7:0 count the banks.
pub const MCG_CAP: u32 = 0x179;
/// `IA32_MCG_STATUS`.
pub const MCG_STATUS: u32 = 0x17a;
/// The banks read at most; architectural MCG_CAP counts reach 255, real
/// processors have a few dozen.
const MAX_BANKS: u64 = 64;
/// The banks a report names; the others holding an error are counted.
const REPORTED: usize = 6;

/// `IA32_MCi_STATUS` of `bank`; its ADDR and MISC registers follow it.
pub fn status_msr(bank: u8) -> u32 {
    0x401 + 4 * u32::from(bank)
}

const VALID: u64 = 1 << 63;
const OVERFLOW: u64 = 1 << 62;
const UNCORRECTED: u64 = 1 << 61;
const MISC_VALID: u64 = 1 << 59;
const ADDRESS_VALID: u64 = 1 << 58;
const CONTEXT_CORRUPT: u64 = 1 << 57;

/// One bank that holds an error.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Bank {
    pub index: u8,
    pub status: u64,
    pub address: Option<u64>,
    pub misc: Option<u64>,
}

/// A machine check as the processor that took it reported it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MachineCheck {
    pub apic: u32,
    pub mcg_status: u64,
    banks: [Option<Bank>; REPORTED],
    /// Banks holding an error, those past [`REPORTED`] included.
    holding: usize,
}

impl MachineCheck {
    /// Read the report of processor `apic` through `read`, an RDMSR. The
    /// caller has checked that the processor implements machine-check
    /// architecture (CPUID.01H:EDX[14]): every register read here is then
    /// architectural.
    pub fn read(apic: u32, mut read: impl FnMut(u32) -> u64) -> Self {
        let count = (read(MCG_CAP) & 0xff).min(MAX_BANKS) as u8;
        let mut report = Self {
            apic,
            mcg_status: read(MCG_STATUS),
            banks: [None; REPORTED],
            holding: 0,
        };
        for index in 0..count {
            let msr = status_msr(index);
            let status = read(msr);
            if status & VALID == 0 {
                continue;
            }
            if let Some(slot) = report.banks.get_mut(report.holding) {
                *slot = Some(Bank {
                    index,
                    status,
                    address: (status & ADDRESS_VALID != 0).then(|| read(msr + 1)),
                    misc: (status & MISC_VALID != 0).then(|| read(msr + 2)),
                });
            }
            report.holding += 1;
        }
        report
    }

    /// The banks the report names.
    pub fn banks(&self) -> impl Iterator<Item = &Bank> {
        self.banks.iter().flatten()
    }
}

impl fmt::Display for MachineCheck {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "machine check on APIC {:#x}: MCG_STATUS={:#x}",
            self.apic, self.mcg_status
        )?;
        for (bit, name) in [(2, "MCIP"), (0, "RIPV"), (1, "EIPV")] {
            if self.mcg_status & 1 << bit != 0 {
                write!(f, " {name}")?;
            }
        }
        if self.holding == 0 {
            return f.write_str("; no bank holds an error");
        }
        for bank in self.banks() {
            write!(f, "; bank {} status={:#x}", bank.index, bank.status)?;
            for (bit, name) in [
                (UNCORRECTED, "uncorrected"),
                (CONTEXT_CORRUPT, "context corrupt"),
                (OVERFLOW, "overflow"),
            ] {
                if bank.status & bit != 0 {
                    write!(f, " {name}")?;
                }
            }
            if let Some(address) = bank.address {
                write!(f, " addr={address:#x}")?;
            }
            if let Some(misc) = bank.misc {
                write!(f, " misc={misc:#x}")?;
            }
        }
        match self.holding.saturating_sub(REPORTED) {
            0 => Ok(()),
            more => write!(f, "; {more} more banks hold errors"),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::{collections::BTreeMap, format, vec::Vec};

    fn read_from(registers: &BTreeMap<u32, u64>, reads: &mut Vec<u32>) -> impl FnMut(u32) -> u64 {
        move |msr| {
            reads.push(msr);
            registers.get(&msr).copied().unwrap_or(0)
        }
    }

    #[test]
    fn report_names_each_bank_holding_an_error_with_its_valid_registers() {
        let mut registers = BTreeMap::new();
        registers.insert(MCG_CAP, 0x0c00_0a06);
        registers.insert(MCG_STATUS, 0b101);
        // Bank 1: corrected, nothing more valid.
        registers.insert(status_msr(1), VALID | 0x0005);
        // Bank 4: uncorrected, context corrupt, with address and misc.
        registers.insert(
            status_msr(4),
            VALID | UNCORRECTED | CONTEXT_CORRUPT | ADDRESS_VALID | MISC_VALID | 0x0150,
        );
        registers.insert(status_msr(4) + 1, 0x1234_5000);
        registers.insert(status_msr(4) + 2, 0x86);
        let mut reads = Vec::new();
        let report = MachineCheck::read(3, read_from(&registers, &mut reads));
        assert_eq!(
            format!("{report}"),
            "machine check on APIC 0x3: MCG_STATUS=0x5 MCIP RIPV; bank 1 status=0x8000000000000005; \
             bank 4 status=0xae00000000000150 uncorrected context corrupt addr=0x12345000 misc=0x86"
        );
        // Six banks counted: only their status registers, and bank 4's
        // address and misc, are read.
        assert_eq!(
            reads,
            [
                MCG_CAP,
                MCG_STATUS,
                status_msr(0),
                status_msr(1),
                status_msr(2),
                status_msr(3),
                status_msr(4),
                status_msr(4) + 1,
                status_msr(4) + 2,
                status_msr(5),
            ]
        );
    }

    #[test]
    fn report_says_when_no_bank_holds_an_error() {
        let mut registers = BTreeMap::new();
        registers.insert(MCG_CAP, 4);
        let mut reads = Vec::new();
        let report = MachineCheck::read(0x10, read_from(&registers, &mut reads));
        assert_eq!(
            format!("{report}"),
            "machine check on APIC 0x10: MCG_STATUS=0x0; no bank holds an error"
        );
    }

    #[test]
    fn report_counts_the_banks_past_those_it_names() {
        let mut registers = BTreeMap::new();
        registers.insert(MCG_CAP, 9);
        for bank in 0..9 {
            registers.insert(status_msr(bank), VALID);
        }
        let mut reads = Vec::new();
        let report = MachineCheck::read(1, read_from(&registers, &mut reads));
        assert_eq!(report.banks().count(), REPORTED);
        assert!(format!("{report}").ends_with("; 3 more banks hold errors"));
    }
}
