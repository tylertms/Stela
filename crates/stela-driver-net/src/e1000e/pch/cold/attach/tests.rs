// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Admission table and the native register adapter.
use super::super::{Registers, nvm::Registers as _};
use super::*;
use crate::bus::pci::{Address, ConfigError, ConfigFailure};
use alloc::vec;
use core::cell::Cell;

/// Linux e1000e's `e1000_pci_tbl`, grouped by `board_*`.
const PCH2: &[u16] = &[0x1502, 0x1503];
const LPT: &[u16] = &[0x153a, 0x153b, 0x15a0, 0x15a1];
const LPT_UNIMPLEMENTED: &[u16] = &[0x155a, 0x1559, 0x15a2, 0x15a3];
const SPT_TO_ADP: &[u16] = &[
    0x156f, 0x1570, 0x15b7, 0x15b8, 0x15b9, 0x15d7, 0x15d8, 0x15e3, 0x15d6, 0x0d53, 0x0d55, 0x15bd,
    0x15be, 0x15bb, 0x15bc, 0x15df, 0x15e0, 0x15e1, 0x15e2, 0x0d4e, 0x0d4f, 0x0d4c, 0x0d4d, 0x15fb,
    0x15fc, 0x15f9, 0x15fa, 0x15f4, 0x15f5, 0x0dc5, 0x0dc6, 0x1a1e, 0x1a1f, 0x1a1c, 0x1a1d, 0x0dc7,
    0x0dc8, 0x550c, 0x550d,
];
const MTP_PTP: &[u16] = &[
    0x550a, 0x550b, 0x550e, 0x550f, 0x5510, 0x5511, 0x57a0, 0x57a1, 0x57b3, 0x57b4, 0x57b7, 0x57b8,
    0x57b9, 0x57ba,
];
/// board_ich8lan..board_pchlan and the discrete 8257x parts.
const OTHER_E1000E: &[u16] = &[
    0x104c, 0x10c5, 0x10c4, 0x104a, 0x104b, 0x104d, 0x1049, 0x1501, 0x10c0, 0x10c2, 0x10c3, 0x10bd,
    0x294c, 0x10e5, 0x10bf, 0x10f5, 0x10cb, 0x10cc, 0x10cd, 0x10ce, 0x10de, 0x10df, 0x1525, 0x10ea,
    0x10eb, 0x10ef, 0x10f0, 0x10d3, 0x10f6, 0x105e, 0x10b9, 0x108b, 0x150c,
];

fn identity(vendor: u16, device: u16, class: [u8; 3]) -> pci::Identity {
    pci::Identity {
        address: Address {
            segment: 0,
            bus: 0,
            device: 0x1f,
            function: 6,
        },
        vendor,
        device,
        class,
    }
}

#[test]
fn every_linux_pch_table_entry_selects_its_board_cohort() {
    for (ids, cohort) in [
        (PCH2, Cohort::Pch2),
        (LPT, Cohort::Lpt),
        (SPT_TO_ADP, Cohort::I219),
        (MTP_PTP, Cohort::Mtp),
    ] {
        for &device in ids {
            assert_eq!(Cohort::of(device), Some(cohort), "{device:04x}");
            assert!(matches_identity(&identity(0x8086, device, [2, 0, 0])));
        }
    }
    assert_eq!(
        PCH2.len() + LPT.len() + SPT_TO_ADP.len() + MTP_PTP.len(),
        59,
        "every implemented PCH ID of the pinned table"
    );
}

#[test]
fn unimplemented_e1000e_boards_vendors_and_classes_stay_unclaimed() {
    for &device in LPT_UNIMPLEMENTED.iter().chain(OTHER_E1000E) {
        assert_eq!(Cohort::of(device), None, "{device:04x}");
        assert!(!matches_identity(&identity(0x8086, device, [2, 0, 0])));
    }
    assert!(!matches_identity(&identity(0x10ec, 0x15b7, [2, 0, 0])));
    assert!(!matches_identity(&identity(0x8086, 0x15b7, [2, 0x80, 0])));
}

#[test]
fn only_pre_spt_cohorts_use_a_separate_flash_bar() {
    assert!(Cohort::Pch2.separate_flash());
    assert!(Cohort::Lpt.separate_flash());
    assert!(!Cohort::I219.separate_flash());
    assert!(!Cohort::Mtp.separate_flash());
}

struct Config {
    words: [u32; 64],
    reads: Cell<usize>,
    refuse: bool,
}

impl ConfigSpace for Config {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        Ok((self.words[usize::from(offset / 4)] >> ((offset & 3) * 8)) as u8)
    }
    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        if self.refuse && offset >= 0x40 {
            return Err(ConfigError::before_effect(
                ConfigFailure::UnqualifiedControl,
            ));
        }
        self.reads.set(self.reads.get() + 1);
        Ok((self.words[usize::from(offset / 4)] >> ((offset & 2) * 8)) as u16)
    }
    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        Ok(self.words[usize::from(offset / 4)])
    }
    fn write8(&mut self, _: u16, _: u8) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(
            ConfigFailure::UnqualifiedControl,
        ))
    }
    fn write16(&mut self, _: u16, _: u16) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(
            ConfigFailure::UnqualifiedControl,
        ))
    }
    fn write32(&mut self, _: u16, _: u32) -> Result<(), ConfigError> {
        Err(ConfigError::before_effect(
            ConfigFailure::UnqualifiedControl,
        ))
    }
}

/// Plain RAM standing in for the two register windows: the adapter only
/// forwards accesses, so their placement and bounds are what is checked.
struct Windows {
    mac: Vec<u32>,
    flash: Vec<u32>,
}

fn adapter(windows: &mut Windows, refuse: bool) -> PciIo<Config> {
    let mut config = Config {
        words: [0; 64],
        reads: Cell::new(0),
        refuse,
    };
    config.words[0] = (0x15b7 << 16) | 0x8086;
    config.words[2] = 0x0200_0000;
    config.words[0xa8 / 4] = 0x1003_1002;
    let function = Function::new(identity(0, 0, [0; 3]).address, config).unwrap();
    // SAFETY: both windows are live test RAM owned by `windows`, which
    // outlives the adapter in every test.
    unsafe {
        PciIo::new(
            Mmio::new(windows.mac.as_mut_ptr() as u64, windows.mac.len() * 4),
            Mmio::new(windows.flash.as_mut_ptr() as u64, windows.flash.len() * 4),
            function,
        )
    }
}

#[test]
fn register_adapter_reaches_each_window_and_refuses_outside_or_misaligned() {
    let mut windows = Windows {
        mac: vec![0; 0x6000 / 4],
        flash: vec![0; 0x100 / 4],
    };
    let mut io = adapter(&mut windows, false);
    Registers::write32(&mut io, 0x5b54, 0x8040).unwrap();
    assert_eq!(Registers::read32(&mut io, 0x5b54), Ok(0x8040));
    assert_eq!(io.read_mac32(0x5b54), Ok(0x8040));
    assert_eq!(
        Registers::read32(&mut io, 0x6000),
        Err(super::super::Fault::OutsideBar)
    );
    assert_eq!(
        Registers::write32(&mut io, 0x5ffe, 1),
        Err(super::super::Fault::OutsideBar)
    );
    super::super::nvm::Registers::write32(&mut io, 0x10, 0xdead_beef).unwrap();
    io.write16(0x6, 0x1234).unwrap();
    assert_eq!(
        super::super::nvm::Registers::read32(&mut io, 0x10),
        Ok(0xdead_beef)
    );
    assert_eq!(io.read16(0x6), Ok(0x1234));
    assert_eq!(io.read16(0x7), Err(super::super::nvm::Error::Bounds));
    assert_eq!(
        super::super::nvm::Registers::read32(&mut io, 0x100),
        Err(super::super::nvm::Error::Bounds)
    );
    assert_eq!(windows.mac[0x5b54 / 4], 0x8040);
    assert_eq!(windows.flash[0x10 / 4], 0xdead_beef);
    assert_eq!(windows.flash[1] >> 16, 0x1234);
}

#[test]
fn ltr_reads_are_live_configuration_reads_and_refusals_stay_errors() {
    let mut windows = Windows {
        mac: vec![0; 0x6000 / 4],
        flash: vec![0; 0x100 / 4],
    };
    let mut io = adapter(&mut windows, false);
    assert_eq!(io.read_config16(0xa8), Ok(0x1002));
    assert_eq!(io.read_config16(0xaa), Ok(0x1003));
    io.function.config().words[0xa8 / 4] = 0x1003_1001;
    assert_eq!(io.read_config16(0xa8), Ok(0x1001));
    assert_eq!(io.function.config().reads.get(), 3);
    let mut refused = adapter(&mut windows, true);
    assert_eq!(
        refused.read_config16(0xe4),
        Err(super::super::Fault::Access)
    );
}

#[test]
fn smbios_fields_pass_unchanged_to_the_mtp_board_quirk() {
    use crate::firmware::smbios::{Observation as Smbios, SystemIdentity};
    // A missing capture stays NotObserved, which the MTP cohort refuses.
    assert_eq!(
        host_identity(&SystemIdentity::default()),
        HostIdentity::default()
    );
    let system = SystemIdentity {
        system_vendor: Smbios::Value(b"LENOVO"),
        product_name: Smbios::Absent,
        product_version: Smbios::Value(b"ThinkPad P14s Gen 5\0"),
        board_vendor: Smbios::Value(b"LENOVO"),
        board_name: Smbios::NotObserved,
    };
    assert_eq!(
        host_identity(&system),
        HostIdentity {
            system_vendor: Observation::Value(b"LENOVO"),
            product_name: Observation::Absent,
            product_version: Observation::Value(b"ThinkPad P14s Gen 5\0"),
        }
    );
}
