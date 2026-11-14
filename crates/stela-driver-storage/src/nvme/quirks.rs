// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! NVMe controller quirks: hardware facts Linux keeps in `pci.c`'s
//! `nvme_id_table` and `check_vendor_combination_bug` and `core.c`'s
//! `core_quirks`, at a804c5742bff224d4cb4d25dd383554d598be32b. They are
//! keyed to the PCI vendor and device ID, to Identify Controller's vendor,
//! model and firmware revision, and to the board's SMBIOS names, as there.
//! Only quirks of paths this driver has are kept; Linux's emulated
//! controllers are left out.

use alloc::{string::String, vec::Vec};
use core::ops::BitOr;
use stela_driver_core::firmware::smbios::{Observation, SystemIdentity};

/// A set of Linux `NVME_QUIRK_*` flags this driver acts on.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Quirks(u16);

impl Quirks {
    pub const NONE: Self = Self(0);
    /// `NVME_QUIRK_DELAY_BEFORE_CHK_RDY`: CSTS is read only 2.3 s after
    /// CC.EN is cleared.
    pub const READY_DELAY: Self = Self(1 << 0);
    /// `NVME_QUIRK_NO_APST`: autonomous power state transitions are turned
    /// off.
    pub const NO_APST: Self = Self(1 << 1);
    /// `NVME_QUIRK_NO_DEEPEST_PS`: APST never targets the deepest state.
    pub const NO_DEEPEST_POWER_STATE: Self = Self(1 << 2);
    /// `NVME_QUIRK_QDEPTH_ONE`: one I/O command at a time.
    pub const QUEUE_DEPTH_ONE: Self = Self(1 << 3);
    /// `NVME_QUIRK_MEDIUM_PRIO_SQ`: the I/O submission queue is created
    /// with medium priority.
    pub const MEDIUM_PRIORITY_QUEUE: Self = Self(1 << 4);
    /// `NVME_QUIRK_DISABLE_WRITE_ZEROES`: Write Zeroes is never offered.
    pub const NO_WRITE_ZEROES: Self = Self(1 << 5);
    /// `NVME_QUIRK_NO_NS_DESC_LIST`: the Namespace Identification
    /// Descriptor list is never asked for.
    pub const NO_DESCRIPTOR_LIST: Self = Self(1 << 6);
    /// `NVME_QUIRK_BOGUS_NID`: the namespace's EUI64, NGUID and UUID are
    /// garbage and ignored.
    pub const BOGUS_IDS: Self = Self(1 << 7);
    /// `NVME_QUIRK_NO_FUA`: FUA is never offered.
    pub const NO_FUA: Self = Self(1 << 8);

    const NAMES: [(Self, &'static str); 9] = [
        (Self::READY_DELAY, "ready check delayed 2.3 s"),
        (Self::NO_APST, "APST off"),
        (Self::NO_DEEPEST_POWER_STATE, "no deepest power state"),
        (Self::QUEUE_DEPTH_ONE, "one command at a time"),
        (Self::MEDIUM_PRIORITY_QUEUE, "medium priority I/O queue"),
        (Self::NO_WRITE_ZEROES, "Write Zeroes off"),
        (Self::NO_DESCRIPTOR_LIST, "no namespace descriptor list"),
        (Self::BOGUS_IDS, "namespace IDs ignored"),
        (Self::NO_FUA, "FUA off"),
    ];

    pub fn contains(self, other: Self) -> bool {
        self.0 & other.0 == other.0
    }

    pub fn is_empty(self) -> bool {
        self.0 == 0
    }

    /// The quirks `nvme_id_table` gives a PCI ID. As there, the first entry
    /// that matches wins.
    pub fn for_device(vendor: u16, device: u16) -> Self {
        DEVICES
            .iter()
            .find(|entry| entry.0 == vendor && entry.1 == device)
            .map_or(Self::NONE, |entry| entry.2)
    }

    /// The quirks `core_quirks` gives an Identify Controller page: every
    /// entry whose vendor ID, model and firmware revision match.
    pub fn for_controller(identify: &[u8]) -> Self {
        let vendor = u16::from_le_bytes([identify[0], identify[1]]);
        let model = &identify[super::CONTROLLER_MODEL];
        let firmware = &identify[CONTROLLER_FIRMWARE];
        CONTROLLERS
            .iter()
            .filter(|entry| {
                entry.vendor == vendor
                    && entry.model.is_none_or(|text| padded_matches(model, text))
                    && entry
                        .firmware
                        .is_none_or(|text| padded_matches(firmware, text))
            })
            .fold(Self::NONE, |quirks, entry| quirks | entry.quirks)
    }

    /// `check_vendor_combination_bug`'s quirks of a PCI ID on the board
    /// SMBIOS names, compared whole as Linux `dmi_match` does.
    pub fn for_platform(vendor: u16, device: u16, platform: &SystemIdentity<'_>) -> Self {
        let is = |field: Observation<'_>, text: &str| smbios_is(field, text);
        match (vendor, device) {
            // Samsung SM951/PM951/950 PRO drop off the bus from the deepest
            // state on two Dell laptops.
            (0x144d, 0xa802)
                if is(platform.system_vendor, "Dell Inc.")
                    && (is(platform.product_name, "XPS 15 9550")
                        || is(platform.product_name, "Precision 5510")) =>
            {
                return Self::NO_DEEPEST_POWER_STATE;
            }
            // Samsung 960 EVO drops off the bus on two ASUS boards.
            (0x144d, 0xa804)
                if is(platform.board_vendor, "ASUSTeK COMPUTER INC.")
                    && (is(platform.board_name, "PRIME B350M-A")
                        || is(platform.board_name, "PRIME Z370-A")) =>
            {
                return Self::NO_APST;
            }
            _ => {}
        }
        // Any drive drops off the bus after hours idle on a Lenovo N60z.
        if is(platform.board_name, "LXKT-ZXEG-N6") {
            return Self::NO_APST;
        }
        Self::NONE
    }

    /// The flags' names, for the boot log.
    pub fn names(self) -> Vec<&'static str> {
        Self::NAMES
            .iter()
            .filter(|(quirk, _)| self.contains(*quirk))
            .map(|(_, name)| *name)
            .collect()
    }

    /// One line saying which quirks apply and where they came from.
    pub fn describe(self) -> String {
        let mut line = String::from("NVMe quirks from Linux's tables: ");
        line.push_str(&self.names().join(", "));
        line
    }
}

impl BitOr for Quirks {
    type Output = Self;

    fn bitor(self, other: Self) -> Self {
        Self(self.0 | other.0)
    }
}

/// Identify Controller's FR.
const CONTROLLER_FIRMWARE: core::ops::Range<usize> = 64..72;

/// Linux `string_matches`: `text`, then nothing but the spaces that pad the
/// field.
fn padded_matches(field: &[u8], text: &str) -> bool {
    field
        .strip_prefix(text.as_bytes())
        .is_some_and(|rest| rest.iter().all(|byte| *byte == b' '))
}

/// An SMBIOS string as Linux `dmi_match` compares it: whole, without the
/// terminating NUL a parsed value keeps.
fn smbios_is(field: Observation<'_>, text: &str) -> bool {
    match field {
        Observation::Value(value) => value.strip_suffix(&[0]).unwrap_or(value) == text.as_bytes(),
        Observation::NotObserved | Observation::Absent => false,
    }
}

struct Controller {
    vendor: u16,
    model: Option<&'static str>,
    firmware: Option<&'static str>,
    quirks: Quirks,
}

/// `core_quirks`, without LiteON's suspend entry.
const CONTROLLERS: [Controller; 3] = [
    // This Toshiba device dies in any APST state.
    Controller {
        vendor: 0x1179,
        model: Some("THNSF5256GPUK TOSHIBA"),
        firmware: None,
        quirks: Quirks::NO_APST,
    },
    // Kioxia CD6-V / HPE PE8030 time out and abort I/O under load.
    Controller {
        vendor: 0x1e0f,
        model: Some("KCD6XVUL6T40"),
        firmware: None,
        quirks: Quirks::NO_APST,
    },
    // The external Samsung X5 shares the 970 EVO Plus's PCI ID.
    Controller {
        vendor: 0x144d,
        model: Some("Samsung Portable SSD X5"),
        firmware: None,
        quirks: Quirks(Quirks::READY_DELAY.0 | Quirks::NO_DEEPEST_POWER_STATE.0),
    },
];

const fn q(quirks: &[Quirks]) -> Quirks {
    let mut bits = 0;
    let mut at = 0;
    while at < quirks.len() {
        bits |= quirks[at].0;
        at += 1;
    }
    Quirks(bits)
}

const DEEPEST: Quirks = Quirks::NO_DEEPEST_POWER_STATE;
const READY: Quirks = Quirks::READY_DELAY;
const ZEROES: Quirks = Quirks::NO_WRITE_ZEROES;
const DESCRIPTORS: Quirks = Quirks::NO_DESCRIPTOR_LIST;
const IDS: Quirks = Quirks::BOGUS_IDS;

/// `nvme_id_table`'s entries with a quirk of this driver, in Linux's order,
/// keeping only the flags this driver acts on. A later Linux entry for an
/// ID an earlier one already names never matches there and is left out.
const DEVICES: &[(u16, u16, Quirks)] = &[
    (0x8086, 0x0a54, IDS), // Intel P4500/P4600
    // Intel 600P/P3100
    (
        0x8086,
        0xf1a5,
        q(&[DEEPEST, Quirks::MEDIUM_PRIORITY_QUEUE, ZEROES]),
    ),
    (0x126f, 0x1001, DEEPEST),                  // Silicon Motion generic
    (0x126f, 0x2262, q(&[DEEPEST, IDS])),       // Silicon Motion generic
    (0x126f, 0x2263, q(&[DESCRIPTORS, IDS])),   // Silicon Motion unidentified
    (0x1bb1, 0x0100, q(&[READY, DESCRIPTORS])), // Seagate Nytro Flash Storage
    (0x1c58, 0x0003, READY),                    // HGST adapter
    (0x1c58, 0x0023, READY),                    // WDC SN200 adapter
    (0x1c5f, 0x0540, READY),                    // Memblaze Pblaze4 adapter
    (0x1c5f, 0x0555, DESCRIPTORS),              // Memblaze Pblaze5 adapter
    (0x144d, 0xa821, READY),                    // Samsung PM1725
    (0x144d, 0xa822, q(&[READY, ZEROES])),      // Samsung PM1725a
    (0x15b7, 0x5009, DEEPEST),                  // Sandisk SN550
    (0x1987, 0x5012, IDS),                      // Phison E12
    (0x1987, 0x5016, IDS),                      // Phison E16
    (0x1987, 0x5019, ZEROES),                   // Phison E19
    (0x1987, 0x5021, ZEROES),                   // Phison E21
    (0x1b4b, 0x1092, DESCRIPTORS),              // Lexar 256 GB SSD
    (0x1cc1, 0x33f8, IDS),                      // ADATA IM2P33F8ABR1 1 TB
    (0x10ec, 0x5762, IDS),                      // ADATA SX6000LNP
    (0x10ec, 0x5763, IDS),                      // ADATA SX6000PNP
    (0x1cc1, 0x8201, DEEPEST),                  // ADATA SX8200PNP 512GB
    (0x1344, 0x6001, IDS),                      // Micron Nitro NVMe
    (0x1c5c, 0x1504, ZEROES),                   // SK Hynix PC400
    (0x1c5c, 0x174a, IDS),                      // SK Hynix P31 SSD
    (0x1c5c, 0x1d59, ZEROES),                   // SK Hynix BC901
    (0x15b7, 0x2001, ZEROES),                   // Sandisk Skyhawk
    (0x1d97, 0x2263, ZEROES),                   // SPCC
    (0x144d, 0xa80b, q(&[ZEROES, IDS])),        // Samsung PM9B1 256G and 512G
    (0x144d, 0xa809, ZEROES),                   // Samsung MZALQ256HBJD 256G
    (0x144d, 0xa802, IDS),                      // Samsung SM953
    (0x1cc4, 0x6303, ZEROES),                   // UMIS RPJTJ512MGE1QDY 512G
    (0x1cc4, 0x6302, ZEROES),                   // UMIS RPJTJ256MGE1QDY 256G
    (0x2646, 0x2262, DEEPEST),                  // Kingston SKC2000
    (0x2646, 0x2263, DEEPEST),                  // Kingston A2000
    (0x2646, 0x5018, ZEROES),                   // Kingston OM8SFP4xxxxP
    (0x2646, 0x5016, ZEROES),                   // Kingston OM3PGP4xxxxP
    (0x2646, 0x501a, ZEROES),                   // Kingston OM8PGP4xxxxP
    (0x2646, 0x501b, ZEROES),                   // Kingston OM8PGP4xxxxQ
    (0x2646, 0x501e, ZEROES),                   // Kingston OM3PGP4xxxxQ
    (0x2646, 0x502f, ZEROES),                   // Kingston OM3SGP4xxxxK
    (0x1f40, 0x1202, IDS),                      // Netac NV3000
    (0x1f40, 0x5236, IDS),                      // Netac NV7000
    (0x1e4b, 0x1001, IDS),                      // MAXIO MAP1001
    (0x1e4b, 0x1002, IDS),                      // MAXIO MAP1002
    (0x1e4b, 0x1202, IDS),                      // MAXIO MAP1202
    (0x1e4b, 0x1602, IDS),                      // MAXIO MAP1602
    (0x1cc1, 0x5350, IDS),                      // ADATA XPG GAMMIX S50
    (0x1cc1, 0x5370, Quirks::NO_FUA),           // ADATA XPG GAMMIX S70 BLADE
    (0x1dbe, 0x5216, IDS),                      // Acer/INNOGRIT FA100/5216
    (0x1dbe, 0x5236, IDS),                      // ADATA XPG GAMMIX S70
    (0x1e49, 0x0021, DEEPEST),                  // ZHITAI TiPro5000
    (0x1e49, 0x0041, DEEPEST),                  // ZHITAI TiPro7000
    (0x025e, 0xf1ac, DEEPEST),                  // Solidigm P44 Pro
    (0xc0a9, 0x540a, IDS),                      // Crucial P2
    (0x1d97, 0x1d97, IDS),                      // Lexar NM620
    (0x1d97, 0x2269, IDS),                      // Lexar NM760
    (0x10ec, 0x5765, IDS),                      // TEAMGROUP MP33 2TB
    // Apple's MacBook8,1 controller; its single vector is this driver's
    // only one anyway.
    (0x106b, 0x2001, Quirks::QUEUE_DEPTH_ONE),
];

#[cfg(test)]
mod tests {
    use super::*;

    fn identify(vendor: u16, model: &str, firmware: &str) -> Vec<u8> {
        let mut page = alloc::vec![b' '; 4096];
        page[..2].copy_from_slice(&vendor.to_le_bytes());
        page[24..24 + model.len()].copy_from_slice(model.as_bytes());
        page[64..64 + firmware.len()].copy_from_slice(firmware.as_bytes());
        page
    }

    #[test]
    fn pci_ids_take_the_first_matching_linux_entry() {
        assert_eq!(
            Quirks::for_device(0x126f, 0x2262),
            DEEPEST | Quirks::BOGUS_IDS
        );
        assert_eq!(Quirks::for_device(0x126f, 0x2263), DESCRIPTORS | IDS);
        assert_eq!(
            Quirks::for_device(0x8086, 0xf1a5),
            DEEPEST | Quirks::MEDIUM_PRIORITY_QUEUE | ZEROES
        );
        assert_eq!(Quirks::for_device(0x1c58, 0x0003), READY);
        assert_eq!(Quirks::for_device(0x1cc1, 0x5370), Quirks::NO_FUA);
        // SPCC's entry precedes Lexar NM610's for the same ID.
        assert_eq!(Quirks::for_device(0x1d97, 0x2263), ZEROES);
        assert_eq!(Quirks::for_device(0x106b, 0x2001), Quirks::QUEUE_DEPTH_ONE);
        // A Samsung 970 EVO Plus, Linux's emulated controllers and a drive
        // whose only Linux quirks are of paths this driver lacks.
        for (vendor, device) in [(0x144d, 0xa808), (0x8086, 0x5845), (0x1b36, 0x0010)] {
            assert_eq!(Quirks::for_device(vendor, device), Quirks::NONE);
        }
        assert_eq!(Quirks::for_device(0x15b7, 0x5008), Quirks::NONE);
    }

    #[test]
    fn model_strings_match_whole_up_to_their_space_padding() {
        let toshiba = identify(0x1179, "THNSF5256GPUK TOSHIBA", "5KDA4101");
        assert_eq!(Quirks::for_controller(&toshiba), Quirks::NO_APST);
        let kioxia = identify(0x1e0f, "KCD6XVUL6T40", "1");
        assert_eq!(Quirks::for_controller(&kioxia), Quirks::NO_APST);
        let x5 = identify(0x144d, "Samsung Portable SSD X5", "1B2QEXK7");
        assert_eq!(Quirks::for_controller(&x5), READY | DEEPEST);
        // A longer model, another vendor ID, and the internal drive that
        // shares the X5's PCI ID.
        for page in [
            identify(0x1179, "THNSF5256GPUK TOSHIBA 2", ""),
            identify(0x1e0f, "THNSF5256GPUK TOSHIBA", ""),
            identify(0x144d, "Samsung SSD 970 EVO Plus 1TB", "2B2QEXM7"),
        ] {
            assert_eq!(Quirks::for_controller(&page), Quirks::NONE);
        }
        assert!(padded_matches(b"22301111", "22301111"));
        assert!(!padded_matches(b"2230111x", "22301111"));
    }

    #[test]
    fn board_names_add_linux_vendor_combination_quirks() {
        let mut platform = SystemIdentity {
            system_vendor: Observation::Value(b"Dell Inc.\0"),
            product_name: Observation::Value(b"XPS 15 9550\0"),
            ..SystemIdentity::default()
        };
        assert_eq!(Quirks::for_platform(0x144d, 0xa802, &platform), DEEPEST);
        assert_eq!(
            Quirks::for_platform(0x144d, 0xa804, &platform),
            Quirks::NONE
        );
        platform.product_name = Observation::Value(b"XPS 15 9560\0");
        assert_eq!(
            Quirks::for_platform(0x144d, 0xa802, &platform),
            Quirks::NONE
        );
        let asus = SystemIdentity {
            board_vendor: Observation::Value(b"ASUSTeK COMPUTER INC.\0"),
            board_name: Observation::Value(b"PRIME Z370-A\0"),
            ..SystemIdentity::default()
        };
        assert_eq!(Quirks::for_platform(0x144d, 0xa804, &asus), Quirks::NO_APST);
        let lenovo = SystemIdentity {
            board_name: Observation::Value(b"LXKT-ZXEG-N6\0"),
            ..SystemIdentity::default()
        };
        // Any drive, also one whose own entry did not match.
        for (vendor, device) in [(0x144d, 0xa802), (0x2646, 0x2263)] {
            assert_eq!(
                Quirks::for_platform(vendor, device, &lenovo),
                Quirks::NO_APST
            );
        }
        assert_eq!(
            Quirks::for_platform(0x144d, 0xa804, &SystemIdentity::default()),
            Quirks::NONE
        );
    }

    #[test]
    fn quirks_are_named_for_the_boot_log() {
        assert_eq!(
            (DEEPEST | ZEROES).describe(),
            "NVMe quirks from Linux's tables: no deepest power state, Write Zeroes off"
        );
    }
}
