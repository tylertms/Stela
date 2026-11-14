// SPDX-License-Identifier: GPL-2.0-or-later
// Copyright 2004-2005 Red Hat, Inc.
//! AHCI host bus adapters with SATA disks, up to 32 queued commands per port.
//! Controller setup follows Linux `drivers/ata/ahci.c` and `libahci.c` at
//! a804c5742bff224d4cb4d25dd383554d598be32b: firmware-programmed capability
//! and port registers survive HBA reset, legacy Intel port enables are set,
//! and each port fails on its own. A failed command goes through libata's
//! error handler: the AHCI error interrupt and taskfile are classified as
//! `ahci_error_intr` and `ata_eh_link_autopsy` do, the engine restarts or the
//! link is hard reset and the disk revalidated, and sd's retries apply.
use crate::{
    Clock, Deadline, Handoff,
    block::{
        BlockDevice, BlockError, Command, DiscardLimits, Extent, Features, Finished, Identity,
        Rotation, Sense, SenseCompletion, Transport, ascii,
    },
    bus::{
        dma::{DmaAllocator, DmaBuffer},
        mmio::Mmio,
        pci::{ConfigSpace, Function},
    },
    registers,
};
use alloc::{format, rc::Rc, vec::Vec};
use core::cell::Cell;
use core::sync::atomic::{Ordering, fence};

mod commands;
mod configuration;
mod error_handling;
mod minimum;
mod quirks;
pub use minimum::{AttachRefusal, Host};
use minimum::{Control, DmaResource, PortRefusal};

pub const CLASS: [u8; 3] = [0x01, 0x06, 0x01];
/// `ahci_pci_tbl`: AMD uses the RAID class only for AHCI controllers.
const VENDOR_AMD: u16 = 0x1022;
const CLASS_RAID: [u8; 3] = [0x01, 0x04, 0x00];

/// Whether `ahci_pci_tbl` binds the function: the AHCI class, an AMD
/// function of the RAID class, or an Intel controller the table lists by
/// ID, such as a chipset SATA controller in RAID ("RST") mode, which is
/// AHCI underneath. Intel VMD is none of these. The table's Dell S140/S150
/// entry matches by subsystem vendor, which these identities do not carry.
pub fn matches<C>(function: &Function<C>) -> bool {
    matches_identity(&function.identity())
}

/// Candidate selection from copied identity DATA; owned attachment still
/// revalidates configuration and platform requirements.
pub fn matches_identity(function: &crate::bus::pci::Identity) -> bool {
    function.class == CLASS
        || function.vendor == VENDOR_AMD && function.class == CLASS_RAID
        || function.vendor == INTEL && INTEL_AHCI.binary_search(&function.device).is_ok()
}

const CAPABILITIES: usize = 0x00;
const GLOBAL_CONTROL: usize = 0x04;
/// IS: one pending-interrupt bit per port, cleared by writing it.
const HOST_INTERRUPTS: usize = 0x08;
const PORTS_IMPLEMENTED: usize = 0x0c;
const CAPABILITIES_2: usize = 0x24;
const HANDOFF: usize = 0x28;

const CAP_64BIT: u32 = 1 << 31;
/// CAP.SNCQ: Linux sets `ATA_FLAG_NCQ` and `ATA_FLAG_FPDMA_AA` from it.
const CAP_NCQ: u32 = 1 << 30;
const CAP_STAGGERED_SPIN_UP: u32 = 1 << 27;
/// CAP.SALP, SSC, PSC and SXS; CAP2.SDS.
const CAP_AGGRESSIVE_LPM: u32 = 1 << 26;
const CAP_SLUMBER: u32 = 1 << 14;
const CAP_PARTIAL: u32 = 1 << 13;
const CAP_EXTERNAL: u32 = 1 << 5;
const CAP2_DEVICE_SLEEP: u32 = 1 << 3;
const CAP2_AGGRESSIVE_DEVICE_SLEEP: u32 = 1 << 4;
/// CAP.NP: the number of ports, minus one.
const CAP_PORTS: u32 = 0x1f;
/// CAP.NCS: the command slots per port, minus one.
const CAP_SLOTS_SHIFT: u32 = 8;
const CAP_SLOTS: u32 = 0x1f;
const CAP2_HANDOFF: u32 = 1 << 0;
const HANDOFF_BIOS_OWNED: u32 = 1 << 0;
const HANDOFF_OS_OWNED: u32 = 1 << 1;
const HANDOFF_BIOS_BUSY: u32 = 1 << 4;
const GLOBAL_RESET: u32 = 1 << 0;
/// GHC.IE: the HBA sends its interrupt message for pending port bits.
const GLOBAL_INTERRUPT_ENABLE: u32 = 1 << 1;
const GLOBAL_AHCI_ENABLE: u32 = 1 << 31;

const PORT_LIST: usize = 0x00;
const PORT_FIS: usize = 0x08;
const PORT_INTERRUPTS: usize = 0x10;
const PORT_INTERRUPT_ENABLE: usize = 0x14;
const PORT_COMMAND: usize = 0x18;
const PORT_TASK_FILE: usize = 0x20;
const PORT_SIGNATURE: usize = 0x24;
const PORT_SATA_STATUS: usize = 0x28;
const PORT_SATA_CONTROL: usize = 0x2c;
const PORT_SATA_ERROR: usize = 0x30;
const PORT_SATA_ACTIVE: usize = 0x34;
const PORT_DEVICE_SLEEP: usize = 0x44;
/// PxDEVSLP: aggressive DevSleep enabled, and DevSleep present.
const DEVICE_SLEEP_AGGRESSIVE: u32 = 1 << 0;
const DEVICE_SLEEP_PRESENT: u32 = 1 << 1;
const PORT_ISSUE: usize = 0x38;

/// Intel Port Control and Status: one enable bit per port (Linux `PCS_6`).
const INTEL_PORT_CONTROL: u16 = 0x92;
const INTEL: u16 = 0x8086;
/// Every Intel ID Linux `ahci_pci_tbl` lists, whatever the function's
/// class, in ascending order: AHCI and RAID-mode SATA controllers.
const INTEL_AHCI: &[u16] = &[
    0x02d3, 0x02d7, 0x06d6, 0x06d7, 0x0f22, 0x0f23, 0x19b0, 0x19b1, 0x19b2, 0x19b3, 0x19b4, 0x19b5,
    0x19b6, 0x19b7, 0x19be, 0x19bf, 0x19c0, 0x19c1, 0x19c2, 0x19c3, 0x19c4, 0x19c5, 0x19c6, 0x19c7,
    0x19ce, 0x19cf, 0x1c02, 0x1c03, 0x1c04, 0x1c05, 0x1c06, 0x1c07, 0x1d02, 0x1d04, 0x1d06, 0x1e02,
    0x1e03, 0x1e04, 0x1e05, 0x1e06, 0x1e07, 0x1e0e, 0x1f22, 0x1f23, 0x1f24, 0x1f25, 0x1f26, 0x1f27,
    0x1f2e, 0x1f2f, 0x1f32, 0x1f33, 0x1f34, 0x1f35, 0x1f36, 0x1f37, 0x1f3e, 0x1f3f, 0x22a3, 0x2323,
    0x23a3, 0x2652, 0x2653, 0x2681, 0x2682, 0x2683, 0x27c1, 0x27c3, 0x27c5, 0x27c6, 0x2821, 0x2822,
    0x2823, 0x2824, 0x2826, 0x2827, 0x2829, 0x282a, 0x282f, 0x2922, 0x2923, 0x2924, 0x2925, 0x2927,
    0x2929, 0x292a, 0x292b, 0x292c, 0x292f, 0x294d, 0x294e, 0x34d3, 0x3a05, 0x3a22, 0x3a25, 0x3b22,
    0x3b23, 0x3b24, 0x3b25, 0x3b29, 0x3b2b, 0x3b2c, 0x3b2f, 0x43d4, 0x43d5, 0x43d6, 0x43d7, 0x4b63,
    0x502a, 0x502b, 0x5ae3, 0x8c02, 0x8c03, 0x8c04, 0x8c05, 0x8c06, 0x8c07, 0x8c0e, 0x8c0f, 0x8c82,
    0x8c83, 0x8c84, 0x8c85, 0x8c86, 0x8c87, 0x8c8e, 0x8c8f, 0x8d02, 0x8d04, 0x8d06, 0x8d0e, 0x8d62,
    0x8d64, 0x8d66, 0x8d6e, 0x9c02, 0x9c03, 0x9c04, 0x9c05, 0x9c06, 0x9c07, 0x9c0e, 0x9c0f, 0x9c83,
    0x9c85, 0x9c87, 0x9c8f, 0x9d03, 0x9d05, 0x9d07, 0x9dd3, 0xa102, 0xa103, 0xa105, 0xa106, 0xa107,
    0xa10f, 0xa182, 0xa186, 0xa1d2, 0xa1d6, 0xa202, 0xa206, 0xa252, 0xa256, 0xa356, 0xa386,
];
/// Linux `ahci_pci_tbl` Intel IDs whose board sets `AHCI_HFLAG_INTEL_PCS_QUIRK`.
const INTEL_PCS_QUIRK: &[u16] = &[
    0x02d3, 0x02d7, 0x06d6, 0x06d7, 0x0f22, 0x0f23, 0x1c02, 0x1c03, 0x1c04, 0x1c05, 0x1c06, 0x1c07,
    0x1d02, 0x1d04, 0x1d06, 0x1e02, 0x1e03, 0x1e04, 0x1e05, 0x1e06, 0x1e07, 0x1e0e, 0x1f22, 0x1f23,
    0x1f24, 0x1f25, 0x1f26, 0x1f27, 0x1f2e, 0x1f2f, 0x1f32, 0x1f33, 0x1f34, 0x1f35, 0x1f36, 0x1f37,
    0x1f3e, 0x1f3f, 0x22a3, 0x2323, 0x23a3, 0x2652, 0x2653, 0x2681, 0x2682, 0x2683, 0x27c1, 0x27c3,
    0x27c5, 0x27c6, 0x2821, 0x2822, 0x2823, 0x2824, 0x2826, 0x2827, 0x2829, 0x282a, 0x282f, 0x2922,
    0x2923, 0x2924, 0x2925, 0x2927, 0x2929, 0x292a, 0x292b, 0x292c, 0x292f, 0x294d, 0x294e, 0x34d3,
    0x3a05, 0x3a22, 0x3a25, 0x3b22, 0x3b23, 0x3b24, 0x3b25, 0x3b29, 0x3b2b, 0x3b2c, 0x3b2f, 0x43d4,
    0x43d5, 0x43d6, 0x43d7, 0x4b63, 0x502a, 0x502b, 0x5ae3, 0x8c02, 0x8c03, 0x8c04, 0x8c05, 0x8c06,
    0x8c07, 0x8c0e, 0x8c0f, 0x8c82, 0x8c83, 0x8c84, 0x8c85, 0x8c86, 0x8c87, 0x8c8e, 0x8c8f, 0x8d02,
    0x8d04, 0x8d06, 0x8d0e, 0x8d62, 0x8d64, 0x8d66, 0x8d6e, 0x9c02, 0x9c03, 0x9c04, 0x9c05, 0x9c06,
    0x9c07, 0x9c0e, 0x9c0f, 0x9c83, 0x9c85, 0x9c87, 0x9c8f, 0x9d03, 0x9d05, 0x9d07, 0x9dd3, 0xa102,
    0xa103, 0xa105, 0xa106, 0xa107, 0xa10f, 0xa182, 0xa186, 0xa1d2, 0xa1d6, 0xa202, 0xa206, 0xa252,
    0xa256, 0xa356, 0xa386,
];

const COMMAND_START: u32 = 1 << 0;
const COMMAND_SPIN_UP: u32 = 1 << 1;
const COMMAND_FIS_RECEIVE: u32 = 1 << 4;
const COMMAND_FIS_RUNNING: u32 = 1 << 14;
const COMMAND_LIST_RUNNING: u32 = 1 << 15;
/// PxCMD.HPCP and ESP, ALPE and ASP, and ICC's Active request.
const COMMAND_HOTPLUG: u32 = 1 << 18;
const COMMAND_EXTERNAL: u32 = 1 << 21;
const COMMAND_AGGRESSIVE_LPM: u32 = 1 << 26;
const COMMAND_AGGRESSIVE_SLUMBER: u32 = 1 << 27;
const COMMAND_INTERFACE_ACTIVE: u32 = 1 << 28;
const TASK_FILE_BUSY: u32 = 0x88;
/// Port interrupt status bits that end a command (Linux `PORT_IRQ_ERROR`).
const INTERRUPT_TASK_FILE_ERROR: u32 = 1 << 30;
const INTERRUPT_HOST_BUS_FATAL: u32 = 1 << 29;
const INTERRUPT_HOST_BUS_DATA: u32 = 1 << 28;
const INTERRUPT_INTERFACE_FATAL: u32 = 1 << 27;
const INTERRUPT_BAD_PORT_MULTIPLIER: u32 = 1 << 23;
const INTERRUPT_PHY_READY_CHANGE: u32 = 1 << 22;
const INTERRUPT_CONNECT_CHANGE: u32 = 1 << 6;
const INTERRUPT_UNKNOWN_FIS: u32 = 1 << 4;
/// Linux `PORT_IRQ_FREEZE`: the errors that freeze the port.
const INTERRUPT_FREEZE: u32 = INTERRUPT_HOST_BUS_FATAL
    | INTERRUPT_INTERFACE_FATAL
    | INTERRUPT_CONNECT_CHANGE
    | INTERRUPT_PHY_READY_CHANGE
    | INTERRUPT_UNKNOWN_FIS
    | INTERRUPT_BAD_PORT_MULTIPLIER;
const INTERRUPT_ERROR: u32 = INTERRUPT_TASK_FILE_ERROR
    | INTERRUPT_HOST_BUS_FATAL
    | INTERRUPT_HOST_BUS_DATA
    | INTERRUPT_INTERFACE_FATAL
    | INTERRUPT_BAD_PORT_MULTIPLIER
    | INTERRUPT_PHY_READY_CHANGE
    | INTERRUPT_CONNECT_CHANGE
    | INTERRUPT_UNKNOWN_FIS;
/// The port interrupt status bits a command ends with: a D2H Register, PIO
/// Setup or Set Device Bits FIS (Linux `PORT_IRQ_D2H_REG_FIS`,
/// `PORT_IRQ_PIOS_FIS`, `PORT_IRQ_SDB_FIS`). Linux's `DEF_PORT_IRQ` adds the
/// DMA Setup FIS, which ends nothing; with it enabled, a message for one
/// that no look follows would hold IS set through the completion.
const INTERRUPT_COMPLETION: u32 = 1 << 0 | 1 << 1 | 1 << 3;
/// ATA status and error register bits (Linux `include/linux/ata.h`).
const STATUS_BUSY: u8 = 1 << 7;
const STATUS_READY: u8 = 1 << 6;
const STATUS_FAULT: u8 = 1 << 5;
const STATUS_REQUEST: u8 = 1 << 3;
/// `ATA_SENSE`: sense data is available.
const STATUS_SENSE: u8 = 1 << 1;
/// `ATA_CORR`: corrected data error.
const STATUS_CORRECTED: u8 = 1 << 2;
const STATUS_ERROR: u8 = 1 << 0;
const ERROR_INTERFACE_CRC: u8 = 1 << 7;
const ERROR_UNCORRECTABLE: u8 = 1 << 6;
const ERROR_ID_NOT_FOUND: u8 = 1 << 4;
const ERROR_ADDRESS_MARK: u8 = 1 << 0;
/// Device register obsolete bits (`ATA_DEVICE_OBS`).
const DEVICE_OBSOLETE: u8 = 0xa0;
/// SError bits (Linux `SERR_*`).
const SERROR_DATA: u32 = 1 << 8;
const SERROR_PERSISTENT: u32 = 1 << 9;
const SERROR_PROTOCOL: u32 = 1 << 10;
const SERROR_INTERNAL: u32 = 1 << 11;
const SERROR_PHY_READY_CHANGE: u32 = 1 << 16;
const SERROR_DEVICE_EXCHANGED: u32 = 1 << 26;
/// Linux `AC_ERR_*`: what a failed command is charged with.
const ERR_DEVICE: u32 = 1 << 0;
const ERR_HSM: u32 = 1 << 1;
const ERR_TIMEOUT: u32 = 1 << 2;
const ERR_MEDIA: u32 = 1 << 3;
const ERR_ATA_BUS: u32 = 1 << 4;
const ERR_HOST_BUS: u32 = 1 << 5;
const ERR_SYSTEM: u32 = 1 << 6;
const ERR_INVALID: u32 = 1 << 7;
/// SStatus.DET: a device is present and Phy communication is established.
const DEVICE_PRESENT: u32 = 3;
const SATA_STATUS_DETECTION: u32 = 0xf;
/// SControl.DET: perform interface initialization (COMRESET).
const SATA_CONTROL_DETECTION: u32 = 0xf;
const SATA_CONTROL_RESET: u32 = 1;
/// `sata_link_hardreset`: keep SPD, disable partial and slumber, then
/// COMRESET (DET 1) and release it (DET 0).
const SATA_CONTROL_SPEED: u32 = 0x0f0;
/// SControl.IPM: transitions to Partial, Slumber and DevSleep disallowed.
const SATA_CONTROL_NO_PARTIAL: u32 = 0x100;
const SATA_CONTROL_NO_SLUMBER: u32 = 0x200;
const SATA_CONTROL_NO_DEVICE_SLEEP: u32 = 0x400;
const SATA_CONTROL_POWER: u32 = 0x700;
const SATA_CONTROL_COMRESET: u32 = 0x301;
const SATA_CONTROL_RESUMED: u32 = 0x300;
const SATA_CONTROL_RESUME_MASK: u32 = 0xf0f;
/// `sata_link_hardreset` turns the Phy off (DET 4) to change SPD.
const SATA_CONTROL_OFFLINE: u32 = 0x304;
const SPEED_SHIFT: u32 = 4;
/// Device signatures (AHCI 1.3.1 Â§3.3.9, Linux `SATA_SIGNATURE`s).
const SIGNATURE_ATA: u32 = 0x0000_0101;
const SIGNATURE_ATAPI: u32 = 0xeb14_0101;
const SIGNATURE_PORT_MULTIPLIER: u32 = 0x9669_0101;
const SIGNATURE_ENCLOSURE: u32 = 0xc33c_0101;
/// `ahci-remap.h`: an Intel vendor capability bit, the remap capability
/// with one bit per remapped device, and each device's class code, in a
/// BAR of at least 512 KiB.
const VENDOR_CAPABILITIES: usize = 0xa4;
const REMAP_CAPABILITIES: usize = 0x800;
const REMAP_CLASS: usize = 0x880;
const REMAP_STRIDE: usize = 0x80;
const REMAP_DEVICES: usize = 3;
const REMAP_REGISTERS: u64 = 0x1000;
const REMAP_BAR_BYTES: u64 = 512 * 1024;
const REMAP_NVME_CLASS: u32 = 0x01_0802;
const PORT_REGISTERS: u64 = 0x80;
const PORT_BASE: u64 = 0x100;
const MAX_PORTS: u32 = 32;

/// Host-to-device Register FIS (SATA 3.x Â§10.5.6): type, the Command bit,
/// and the device register's LBA mode bit.
const FIS_REGISTER_HOST_TO_DEVICE: u8 = 0x27;
const FIS_COMMAND: u8 = 0x80;
const DEVICE_LBA: u8 = 0x40;
/// Command table: the FIS area, then the first PRDT entry's address and
/// byte count (AHCI 1.3.1 Â§4.2.3).
const TABLE_HEADER_BYTES: usize = 0x90;
const PRDT_ADDRESS: usize = 0x80;
const PRDT_BYTE_COUNT: usize = 0x8c;
const PRDT_ENTRY_BYTES: usize = 16;
/// IDENTIFY DEVICE words (ACS-3 Â§7.12.7).
const IDENTIFY_CONFIG: usize = 0;
const IDENTIFY_SERIAL: core::ops::Range<usize> = 10..20;
const IDENTIFY_FIRMWARE: core::ops::Range<usize> = 23..27;
/// `ata_id_is_sata` reads the major version and hardware reset words.
const IDENTIFY_MAJOR_VERSION: usize = 80;
const IDENTIFY_HARDWARE_RESET: usize = 93;
/// Word 49 bit 11: IORDY supported; word 51 bits 15:8: the old PIO mode;
/// word 53 bits 1 and 2: words 64 and 88 valid; words 64 and 88: advanced
/// PIO and UDMA modes (`ata_id_xfermask`).
const IDENTIFY_CAPABILITY: usize = 49;
const CAPABILITY_IORDY: u16 = 1 << 11;
const IDENTIFY_OLD_PIO: usize = 51;
const IDENTIFY_FIELD_VALID: usize = 53;
const FIELD_VALID_PIO: u16 = 1 << 1;
const FIELD_VALID_UDMA: u16 = 1 << 2;
const IDENTIFY_PIO_MODES: usize = 64;
const IDENTIFY_UDMA_MODES: usize = 88;
/// Word 128 bits 2:0: security supported, enabled and locked.
const IDENTIFY_SECURITY: usize = 128;
/// SATA capabilities (word 76, HIPM in bit 9) and features supported (word
/// 78, DIPM in bit 3).
const IDENTIFY_SATA_CAPABILITY: usize = 76;
/// `ata_id_queue_depth`: the queued commands the disk takes, minus one.
const IDENTIFY_QUEUE_DEPTH: usize = 75;
const QUEUE_DEPTH_MASK: u16 = 0x1f;
const IDENTIFY_SATA_FEATURES: usize = 78;
const SATA_CAPABILITY_HIPM: u16 = 1 << 9;
const SATA_CAPABILITY_NCQ: u16 = 1 << 8;
const SATA_FEATURE_DIPM: u16 = 1 << 3;
const SATA_FEATURE_FPDMA_AA: u16 = 1 << 2;
const SATA_FEATURE_NCQ_AUTOSENSE: u16 = 1 << 7;
const SATA_FEATURE_DEVICE_SLEEP: u16 = 1 << 8;
const SECURITY_LOCKED: u16 = 0x7;
const IDENTIFY_MODEL: core::ops::Range<usize> = 27..47;
const IDENTIFY_COMMANDS_2: usize = 83;
const COMMANDS_2_LBA48: u16 = 1 << 10;
const COMMANDS_2_FLUSH_EXT: u16 = 1 << 13;
/// Words 83 and 87 are valid when bits 15:14 read 01b.
const WORD_VALID_MASK: u16 = 0xc000;
const WORD_VALID: u16 = 0x4000;
const IDENTIFY_ENABLED_1: usize = 85;
const ENABLED_1_WRITE_CACHE: u16 = 1 << 5;
const IDENTIFY_DEFAULTS: usize = 87;
/// Word 169 bit 0: DATA SET MANAGEMENT with the TRIM bit.
const IDENTIFY_DATA_SET_MANAGEMENT: usize = 169;
const DATA_SET_MANAGEMENT_TRIM: u16 = 1;
/// Word 86 bit 15: words 119 and 120 are valid; their bit 6 is Sense Data
/// Reporting supported and enabled.
const IDENTIFY_ENABLED_2: usize = 86;
const ENABLED_2_WORDS_119_120: u16 = 1 << 15;
const IDENTIFY_COMMANDS_3: usize = 119;
const IDENTIFY_ENABLED_3: usize = 120;
const SENSE_DATA_REPORTING: u16 = 1 << 6;
/// Word 119: READ LOG DMA EXT.
const COMMANDS_3_READ_LOG_DMA: u16 = 1 << 3;
const IDENTIFY_LBA48_SECTORS: core::ops::Range<usize> = 100..104;
const IDENTIFY_SECTOR_SIZE: usize = 106;
const SECTOR_SIZE_VALID_MASK: u16 = 0xc000;
const SECTOR_SIZE_VALID: u16 = 0x4000;
const SECTOR_SIZE_LOGICAL_LONG: u16 = 1 << 12;
const IDENTIFY_LOGICAL_WORDS: usize = 117;
/// Word 217: the nominal media rotation rate (`ata_id_rotation_rate`).
const IDENTIFY_ROTATION: usize = 217;
const ATA_SECTOR_BYTES: u32 = 512;

/// The 32-slot command list (1 KiB aligned) followed by the received-FIS
/// area (256 bytes), and one command table (AHCI 1.3.1 Â§4.2).
const COMMAND_LIST_BYTES: usize = 1024;
const COMMAND_HEADER_BYTES: usize = 32;
const RECEIVED_FIS_BYTES: usize = 256;
/// The received PIO Setup FIS a PIO data command ends with.
const RECEIVED_PIO: usize = COMMAND_LIST_BYTES + 0x20;
/// The received D2H Register FIS, set to BSY before a hardreset.
const RECEIVED_D2H: usize = COMMAND_LIST_BYTES + 0x40;
/// The Set Device Bits FIS a queued command completes with.
const RECEIVED_SDB: usize = COMMAND_LIST_BYTES + 0x58;
/// FIS types (SATA 3.5 §10.5): D2H Register, PIO Setup, Set Device Bits.
const FIS_REGISTER_DEVICE_TO_HOST: u8 = 0x34;
const FIS_PIO_SETUP: u8 = 0x5f;
const FIS_SET_DEVICE_BITS: u8 = 0xa1;
/// Polls of the received-FIS area between reads of PxIS and PxCI, which a
/// hypervisor traps, while a command runs: a failure may post no FIS.
const REGISTER_POLL_INTERVAL: u32 = 64;
const FIS_BYTES: usize = 20;
const COMMAND_TABLE_BYTES: usize = 4096;
const COMMAND_TABLE_ALIGN: usize = 128;
/// Revalidation IDENTIFY data, in the command table past its one PRDT entry,
/// so a failed write's data stays in the bounce buffer for its retry.
const IDENTIFY_DATA: usize = 0x200;
/// Where log pages are read, past the IDENTIFY data.
const LOG_DATA: usize = 0x400;

/// Command header DW0: FIS length in dwords, direction, PRDT entries.
const COMMAND_FIS_DWORDS: u32 = 5;
const COMMAND_WRITE_SHIFT: u32 = 6;
const COMMAND_PRDT_SHIFT: u32 = 16;

const ATA_READ_DMA_EXT: u8 = 0x25;
const ATA_WRITE_DMA_EXT: u8 = 0x35;
const ATA_READ_FPDMA_QUEUED: u8 = 0x60;
const ATA_WRITE_FPDMA_QUEUED: u8 = 0x61;
const ATA_READ_LOG_EXT: u8 = 0x2f;
const ATA_READ_LOG_DMA_EXT: u8 = 0x47;
/// The NCQ Command Error log (`ATA_LOG_SATA_NCQ`): byte 0 holds NQ, set
/// when no queued command failed, and the failed command's tag.
const LOG_SATA_NCQ: u8 = 0x10;
const LOG_NCQ_NQ: u8 = 0x80;
const LOG_NCQ_TAG: u8 = 0x1f;
/// The general purpose log directory, and the IDENTIFY DEVICE data log
/// with its SATA Settings page.
const LOG_DIRECTORY: u8 = 0x00;
const LOG_IDENTIFY_DEVICE: u8 = 0x30;
const LOG_SATA_SETTINGS: u8 = 0x08;
const ATA_FLUSH_CACHE: u8 = 0xe7;
const ATA_FLUSH_CACHE_EXT: u8 = 0xea;
const ATA_STANDBY_IMMEDIATE: u8 = 0xe0;
/// DATA SET MANAGEMENT, its TRIM feature, and the ranges one 512-byte block
/// of them holds, each at most 65535 sectors (Linux `ATA_MAX_TRIM_RNUM`).
const ATA_DATA_SET_MANAGEMENT: u8 = 0x06;
const TRIM: u16 = 1;
const TRIM_RANGES: u64 = 64;
const TRIM_RANGE_SECTORS: u64 = 0xffff;
const ATA_IDENTIFY: u8 = 0xec;
const ATA_SET_FEATURES: u8 = 0xef;
/// `SETFEATURES_XFER`, with the transfer mode in the count.
const FEATURE_TRANSFER_MODE: u8 = 0x03;
/// `XFER_PIO_0`, `XFER_PIO_2` and `XFER_UDMA_0`.
const MODE_PIO_0: u8 = 0x08;
const MODE_PIO_2: u8 = 0x0a;
const MODE_UDMA_0: u8 = 0x40;
/// `ahci_port_info`: PIO 0-4 and UDMA 0-6, no MWDMA.
const HOST_PIO_MASK: u32 = 0x1f;
const HOST_UDMA_MASK: u32 = 0x7f;
/// `ATA_UDMA_MASK_40C`: UDMA 0-2.
const UDMA_MASK_40_WIRE: u32 = 0x07;
const ATA_REQUEST_SENSE_DATA_EXT: u8 = 0x0b;
/// `SETFEATURE_SENSE_DATA`, enabled with a count of one.
const FEATURE_SENSE_DATA: u8 = 0xc3;
/// SET FEATURES enabling or disabling a SATA feature: DIPM.
const FEATURE_SATA_ENABLE: u8 = 0x10;
const FEATURE_SATA_DISABLE: u8 = 0x90;
const SATA_DIPM: u16 = 0x03;
/// First-party DMA auto-activation.
const SATA_FPDMA_AA: u16 = 0x02;
const SATA_DEVICE_SLEEP: u16 = 0x09;
/// Linux's `ATA_LPM_*` policies this driver uses: unknown before the first
/// configuration, maximum power, and CONFIG_SATA_MOBILE_LPM_POLICY's default.
const LPM_UNKNOWN: u8 = 0;
const LPM_MAX_POWER: u8 = 1;
const LPM_MED_POWER_WITH_DIPM: u8 = 3;
/// `ahci_set_lpm`'s wait for the interface to leave a low-power state.
const LPM_WAKE_US: u64 = 10_000;
/// `ATA_TMOUT_SPURIOUS_PHY`: how long after an LPM policy change a PhyRdy
/// change may be the change's own.
const SPURIOUS_PHY_US: u64 = 10_000_000;
/// The highest sense key REQUEST SENSE DATA EXT may report.
const SENSE_COMPLETED: u8 = 0x0f;

/// The most data one command moves, through slot 0's bounce buffer or the
/// pool pages a queued command in another slot holds.
const BOUNCE_BYTES: usize = 128 * 1024;
const PAGE: usize = 4096;
const CHUNK_PAGES: usize = BOUNCE_BYTES / PAGE;
/// The queued commands a port runs at once, each in its own slot with its
/// own command table: Linux `ATA_MAX_QUEUE`.
const DEPTH: usize = 32;
/// The bounce pages slots 1 and up share: as much memory as seven commands
/// of [`BOUNCE_BYTES`] take, which many more small commands share.
const POOL_PAGES: usize = 7 * CHUNK_PAGES;
const RESET_TIMEOUT_US: u64 = 1_000_000;
/// Linux `ahci_stop_engine` and `ahci_stop_fis_rx` waits.
const STOP_ENGINE_TIMEOUT_US: u64 = 500_000;
const STOP_FIS_TIMEOUT_US: u64 = 1_000_000;
const COMRESET_HOLD_US: u64 = 1_000;
const SPIN_UP_TIMEOUT_US: u64 = 10_000_000;
const COMMAND_TIMEOUT_US: u64 = 30_000_000;
/// `ata_eh_reset_timeouts`: one hardreset per entry.
const RESET_TIMEOUTS_US: [u64; 4] = [10_000_000, 10_000_000, 35_000_000, 5_000_000];
/// `ATA_EH_DEV_TRIES`: revalidations before the disk is disabled.
const DEVICE_TRIES: usize = 3;
/// `SD_MAX_RETRIES`.
const COMMAND_RETRIES: u32 = 5;
/// `ATA_LINK_RESUME_TRIES`, each followed by a 200 ms settle.
const RESUME_TRIES: usize = 5;
const RESUME_SETTLE_US: u64 = 200_000;
/// `sata_deb_timing_normal` and `_hotplug`: interval, stable duration, limit.
const DEBOUNCE_NORMAL: [u64; 3] = [5_000, 100_000, 2_000_000];
const DEBOUNCE_HOTPLUG: [u64; 3] = [25_000, 500_000, 2_000_000];
/// Linux `ATA_EFLAG_*`: what `ata_eh_speed_down` records with an error.
const EFLAG_IO: u32 = 1 << 0;
const EFLAG_DUBIOUS: u32 = 1 << 1;
const EFLAG_OLD: u32 = 1 << 31;
/// Linux `ATA_ECAT_*`, each offset by `DUBIOUS` before the transfer is
/// verified.
const CATEGORY_ATA_BUS: usize = 1;
const CATEGORY_TIMEOUT_HSM: usize = 2;
const CATEGORY_UNKNOWN_DEVICE: usize = 3;
const CATEGORY_DUBIOUS: usize = 4;
const CATEGORIES: usize = 8;
/// Linux `ATA_EH_SPDN_*` verdicts.
const VERDICT_NCQ_OFF: u32 = 1 << 0;
const VERDICT_SPEED_DOWN: u32 = 1 << 1;
const VERDICT_FALLBACK_TO_PIO: u32 = 1 << 2;
const VERDICT_KEEP_ERRORS: u32 = 1 << 3;
/// `ATA_ERING_SIZE`, and the verdict's five- and ten-minute windows.
const ERROR_RING: usize = 32;
const VERDICT_SHORT_US: u64 = 5 * 60 * 1_000_000;
const VERDICT_LONG_US: u64 = 10 * 60 * 1_000_000;
/// The disk left the port or another device answered in its place.
const DETACHED: BlockError = BlockError::Protocol("the SATA disk detached or changed");
/// `-EAGAIN` from configuration: another EH round without prejudice.
const AGAIN: BlockError = BlockError::Protocol("the SATA link needs another reset");

/// Command register side effects, allowing hardware-like W1C and completion
/// tests against the table and DMA algorithm.
trait CommandPort {
    fn read32(&self, offset: usize) -> u32;
    fn write32(&self, offset: usize, value: u32);
}

impl CommandPort for Mmio {
    fn read32(&self, offset: usize) -> u32 {
        Mmio::read32(self, offset)
    }
    fn write32(&self, offset: usize, value: u32) {
        Mmio::write32(self, offset, value);
    }
}

pub struct AhciDisk {
    port: Mmio,
    commands: DmaResource<DmaBuffer>,
    table: DmaResource<DmaBuffer>,
    bounce: DmaResource<DmaBuffer>,
    minimum: Rc<Control>,
    retain: Rc<Cell<bool>>,
    identity: Identity,
    clock: Clock,
    command_timeout_us: u64,
    /// Set when error handling cannot recover the disk, as libata disables
    /// it. The queue, FIS and bounce allocations remain reserved for this boot.
    fault: Option<BlockError>,
    /// The cache flush command, or `None` when the write cache is off.
    flush: Option<u8>,
    /// `dev->ering`, the history `ata_eh_speed_down` judges.
    errors: ErrorRing,
    /// `ATA_DFLAG_DUBIOUS_XFER`: no read or write has completed since the
    /// disk was configured.
    dubious: bool,
    /// `link->sata_spd_limit`, a mask of allowed SPD values less one, and
    /// `link->sata_spd`, the SStatus SPD after the last reset.
    speed_limit: u32,
    speed: u32,
    /// `ata_id_sense_reporting_enabled` for the last IDENTIFY data read.
    sense_reporting: bool,
    /// The last IDENTIFY data's transfer modes, the device's own masks that
    /// speed-downs narrow, the configured mode and `spdn_cnt`.
    transfer: Transfer,
    pio_mask: u32,
    udma_mask: u32,
    mode: u8,
    step_downs: u8,
    /// `ata_id_is_locked` for the last IDENTIFY data read.
    locked: bool,
    lpm: Lpm,
    ncq: Ncq,
    /// `ata_id_has_read_log_dma_ext` for the last IDENTIFY data, and
    /// `ATA_QUIRK_NO_DMA_LOG`, set for good when READ LOG DMA EXT fails.
    log_dma: bool,
    no_dma_log: bool,
    /// `dev->quirks`: the table's, which configuration accumulates.
    quirks: u32,
    /// `ATA_DFLAG_DEVSLP`: the disk has DevSleep and its SATA Settings page.
    devslp: bool,
    /// Slots 1 and up's command tables and pool pages, for queued commands;
    /// slot 0's are `table` and `bounce`.
    slots: DmaResource<Vec<Slot>>,
    pool: DmaResource<Pool>,
    /// The block commands started on the port, each in its own slot, and
    /// the next one's tag.
    running: Vec<Running>,
    next_tag: u32,
    /// Looks at the port since a command was last launched alone.
    looks: u32,
    /// Where the port's completions raise the HBA's interrupt message, when
    /// the function has MSI or MSI-X; `None` polls.
    signal: Option<Signal>,
}

/// The HBA registers and the port's bit in IS, for a port whose
/// completions send the interrupt message.
#[derive(Clone, Copy)]
struct Signal {
    hba: Mmio,
    port: u32,
}

/// A slot's command table, and the pool pages its block command holds.
struct Slot {
    table: DmaBuffer,
    pages: Vec<u16>,
}

/// Bounce pages for queued commands in slots 1 and up. A block command
/// takes the pages of its largest part when it starts and gives them back
/// when its result is taken. A disk error handling disabled keeps them, as
/// it keeps all its DMA memory.
#[derive(Default)]
struct Pool {
    pages: Vec<DmaBuffer>,
    /// Indices into `pages`, the lowest taken first.
    free: Vec<u16>,
}

impl Pool {
    fn take(&mut self, count: usize) -> Option<Vec<u16>> {
        let at = self.free.len().checked_sub(count)?;
        let mut pages = self.free.split_off(at);
        pages.reverse();
        Some(pages)
    }

    fn give_back(&mut self, pages: &mut Vec<u16>) {
        self.free.extend(pages.drain(..).rev());
    }
}

/// Task-file registers and the DMA region encoded for one command issue:
/// at `buffer`, or with `None` in the issuing slot's own memory.
struct AtaRequest {
    command: u8,
    feature: u16,
    lba: u64,
    count: u16,
    bytes: usize,
    write: bool,
    buffer: Option<u64>,
}

/// A command issued in its slot, awaiting its ending FIS or an error.
struct Launched {
    slot: usize,
    queued: bool,
    deadline: Deadline,
}

/// One look at the port's launched commands ([`AhciDisk::look`]): the
/// slots that completed, and a failure with the slots it is charged to
/// unless the NCQ error log names one.
struct Seen {
    done: u32,
    failure: Option<Failure>,
    blamed: u32,
}

/// One command through libata's error handling ([`AhciDisk::issue_with`]),
/// a step at a time.
struct Issue {
    /// The slot whose command table and bounce buffer it uses.
    slot: usize,
    command: u8,
    feature: u16,
    lba: u64,
    count: u16,
    bytes: usize,
    write: bool,
    /// Only reads and writes carry ATA_QCFLAG_IO.
    io: bool,
    /// When the command was first prepared, for sd's runtime limit.
    prepared: u64,
    retries: u32,
    launched: Option<Launched>,
    /// The delay sense data asked for before the command runs again.
    resume: Option<Deadline>,
}

/// A started block command, run [`BOUNCE_BYTES`] at a time in its slot.
struct Running {
    tag: u32,
    slot: usize,
    command: Command,
    /// Bytes done; a flush counts one once complete.
    done: usize,
    data: Vec<u8>,
    issue: Option<Issue>,
    result: Option<Finished>,
}

/// The received FIS types a command ends with, and where they land: a
/// queued command's Set Device Bits, another's D2H Register, or for PIO
/// data a PIO Setup, which precedes the data, from when on the registers
/// are read every look.
fn ending(queued: bool) -> &'static [(usize, u8)] {
    if queued {
        &[(RECEIVED_SDB, FIS_SET_DEVICE_BITS)]
    } else {
        &[
            (RECEIVED_D2H, FIS_REGISTER_DEVICE_TO_HOST),
            (RECEIVED_PIO, FIS_PIO_SETUP),
        ]
    }
}

/// NCQ as Linux `ata_dev_config_ncq` configures it. A queued command's tag
/// is its slot.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Ncq {
    /// CAP.SNCQ.
    host: bool,
    /// CAP.NCS + 1, and `ata_id_queue_depth` of the last IDENTIFY data.
    host_slots: usize,
    depth: usize,
    /// `ATA_DFLAG_NCQ`: the host and the last IDENTIFY data support it.
    device: bool,
    /// `ATA_DFLAG_NCQ_OFF`: a speed-down turned it off for good.
    off: bool,
    /// `ata_id_has_ncq_autosense`.
    autosense: bool,
    /// `ATA_QUIRK_BROKEN_FPDMA_AA`: enabling auto-activation failed.
    aa_broken: bool,
}

impl Ncq {
    /// `ata_ncq_enabled`.
    fn enabled(&self) -> bool {
        self.device && !self.off
    }
}

/// Linux link power management for one port.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Lpm {
    capabilities: u32,
    capabilities_2: u32,
    /// Not `ATA_LFLAG_NO_LPM`: an internal, LPM-capable port whose
    /// configuration has not failed with one try left.
    allowed: bool,
    /// `link->lpm_policy`.
    policy: u8,
    /// `ata_id_has_hipm` and `ata_id_has_dipm` for the last IDENTIFY data.
    hipm: bool,
    dipm: bool,
    /// `ATA_QUIRK_NOLPM`: the device's LPM is broken; max_power is forced.
    broken: bool,
    /// `link->last_lpm_change` with `ATA_LFLAG_CHANGED`: the clock when a
    /// policy was last applied.
    changed: Option<u64>,
}

impl Lpm {
    /// Linux `ahci_mark_external_port` and `ahci_update_initial_lpm_policy`:
    /// none on a hotplug-capable or eSATA port, where LPM hides removal, or
    /// on a host without Partial, Slumber and DevSleep.
    fn new(capabilities: u32, capabilities_2: u32, command: u32) -> Self {
        let external = command & COMMAND_EXTERNAL != 0 && capabilities & CAP_EXTERNAL != 0
            || command & COMMAND_HOTPLUG != 0;
        let states = capabilities & (CAP_PARTIAL | CAP_SLUMBER) != 0
            || capabilities_2 & CAP2_DEVICE_SLEEP != 0;
        Self {
            capabilities,
            capabilities_2,
            allowed: !external && states,
            policy: LPM_UNKNOWN,
            ..Self::default()
        }
    }

    /// `ap->target_lpm_policy`. The s2idle upgrade to the minimum-power
    /// policies needs the FADT low-power S0 flag, which the handoff does not
    /// carry; the S3 default stands.
    fn target(&self) -> u8 {
        if self.allowed && !self.broken {
            LPM_MED_POWER_WITH_DIPM
        } else {
            LPM_MAX_POWER
        }
    }

    /// Not `ATA_FLAG_NO_DIPM`: the host has both Partial and Slumber.
    fn host_dipm(&self) -> bool {
        self.capabilities & (CAP_PARTIAL | CAP_SLUMBER) == CAP_PARTIAL | CAP_SLUMBER
    }
}

/// One `ata_ering_entry`: its flags, `AC_ERR_*` mask and raw clock origin.
#[derive(Clone, Copy, Default)]
struct Recorded {
    eflags: u32,
    mask: u32,
    at: u64,
}

/// Linux `struct ata_ering`: the newest error at the cursor.
#[derive(Default)]
struct ErrorRing {
    entries: [Recorded; ERROR_RING],
    cursor: usize,
}

impl ErrorRing {
    /// `ata_ering_record`.
    fn record(&mut self, eflags: u32, mask: u32, at: u64) {
        self.cursor = (self.cursor + 1) % ERROR_RING;
        self.entries[self.cursor] = Recorded { eflags, mask, at };
    }

    /// `ata_ering_map`'s order: newest first, up to the first empty entry.
    fn newest(&mut self) -> impl Iterator<Item = &mut Recorded> {
        let (head, tail) = self.entries.split_at_mut(self.cursor + 1);
        head.iter_mut()
            .rev()
            .chain(tail.iter_mut().rev())
            .take_while(|entry| entry.mask != 0)
    }

    /// `ata_ering_top`: the newest entry, if any.
    fn top(&mut self) -> Option<&mut Recorded> {
        Some(&mut self.entries[self.cursor]).filter(|entry| entry.mask != 0)
    }

    /// `ata_ering_clear`: entries stay but no longer count.
    fn clear(&mut self) {
        for entry in self.newest() {
            entry.eflags |= EFLAG_OLD;
        }
    }
}

/// Linux `ata_eh_categorize_error`. `verified` turns true at the first
/// error recorded after a verified transfer and stays so for older ones.
fn categorize(eflags: u32, mask: u32, verified: &mut bool) -> usize {
    if eflags & EFLAG_DUBIOUS == 0 {
        *verified = true;
    }
    let base = if *verified { 0 } else { CATEGORY_DUBIOUS };
    if mask & ERR_ATA_BUS != 0 {
        return base + CATEGORY_ATA_BUS;
    }
    if mask & ERR_TIMEOUT != 0 {
        return base + CATEGORY_TIMEOUT_HSM;
    }
    if eflags & EFLAG_IO != 0 {
        if mask & ERR_HSM != 0 {
            return base + CATEGORY_TIMEOUT_HSM;
        }
        if mask & (ERR_DEVICE | ERR_MEDIA | ERR_INVALID) == ERR_DEVICE {
            return base + CATEGORY_UNKNOWN_DEVICE;
        }
    }
    0
}

/// `fls`: the highest set bit, counting from one.
fn fls(value: u32) -> u32 {
    u32::BITS - value.leading_zeros()
}

/// `__sata_set_spd_needed`'s target SPD for a limit.
fn speed_target(limit: u32) -> u32 {
    if limit == u32::MAX {
        0
    } else {
        fls(limit) & 0xf
    }
}

/// Linux's flush: none unless the write cache is enabled
/// (`ata_id_wcache_enabled`, which sd reads as WCE), then FLUSH CACHE EXT
/// when the disk has it (`ata_id_has_flush_ext`), else FLUSH CACHE.
fn flush_command(word: impl Fn(usize) -> u16) -> Option<u8> {
    if word(IDENTIFY_DEFAULTS) & WORD_VALID_MASK != WORD_VALID
        || word(IDENTIFY_ENABLED_1) & ENABLED_1_WRITE_CACHE == 0
    {
        return None;
    }
    let commands = word(IDENTIFY_COMMANDS_2);
    Some(
        if commands & WORD_VALID_MASK == WORD_VALID && commands & COMMANDS_2_FLUSH_EXT != 0 {
            ATA_FLUSH_CACHE_EXT
        } else {
            ATA_FLUSH_CACHE
        },
    )
}

/// What IDENTIFY DEVICE says about a disk.
struct Identified {
    model: alloc::string::String,
    serial: alloc::string::String,
    blocks: u64,
    block_size: u32,
    flush: Option<u8>,
    /// Sense Data Reporting supported, and enabled.
    sense_reporting: (bool, bool),
    transfer: Transfer,
    /// `ata_id_is_locked`.
    locked: bool,
    /// `ata_id_has_hipm` and `ata_id_has_dipm`.
    hipm: bool,
    dipm: bool,
    /// `ata_id_has_ncq`, `ata_id_has_fpdma_aa`, `ata_id_has_ncq_autosense`
    /// and `ata_id_has_read_log_dma_ext`.
    ncq: bool,
    fpdma_aa: bool,
    ncq_autosense: bool,
    read_log_dma: bool,
    /// `ata_id_queue_depth`.
    queue_depth: usize,
    /// `ata_dev_quirks` for the model number and firmware revision.
    quirks: u32,
    /// Word 76's supported SATA speeds.
    sata_speeds: u16,
    /// `ata_id_has_devslp`.
    devslp: bool,
    /// `ata_id_has_trim`.
    trim: bool,
    /// Word 217.
    rotation: Rotation,
}

/// What `ata_id_xfermask`, `ata_id_has_iordy` and `ata_id_is_cfa` read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Transfer {
    pio: u32,
    udma: u32,
    iordy: bool,
    compact_flash: bool,
    /// `ata_id_is_sata`.
    sata: bool,
}

impl Transfer {
    fn from_identify(word: impl Fn(usize) -> u16) -> Self {
        let pio = if word(IDENTIFY_FIELD_VALID) & FIELD_VALID_PIO != 0 {
            u32::from(word(IDENTIFY_PIO_MODES) & 0x03) << 3 | 0x07
        } else {
            match word(IDENTIFY_OLD_PIO) >> 8 {
                mode @ 0..5 => (2 << mode) - 1,
                _ => 1,
            }
        };
        let udma = if word(IDENTIFY_FIELD_VALID) & FIELD_VALID_UDMA != 0 {
            u32::from(word(IDENTIFY_UDMA_MODES) & 0xff)
        } else {
            0
        };
        let config = word(IDENTIFY_CONFIG);
        Self {
            pio,
            udma,
            iordy: word(IDENTIFY_CAPABILITY) & CAPABILITY_IORDY != 0,
            compact_flash: config == 0x848a
                || config == 0x844a
                || word(IDENTIFY_COMMANDS_2) & 0xc004 == 0x4004,
            sata: word(IDENTIFY_HARDWARE_RESET) == 0 && word(IDENTIFY_MAJOR_VERSION) as i16 >= 0x20,
        }
    }
}

/// `ata_xfer_mask2mode` for one mode class: its highest mode.
fn highest_mode(mask: u32, base: u8) -> Option<u8> {
    (mask != 0).then(|| base + (fls(mask) - 1) as u8)
}

/// Linux `ATA_DNXFER_*` steps.
#[derive(Clone, Copy)]
enum StepDown {
    Pio,
    Dma,
    FortyWire,
}

/// Linux `ata_id_has_sense_reporting` and `ata_id_sense_reporting_enabled`.
fn sense_reporting(word: impl Fn(usize) -> u16) -> (bool, bool) {
    let valid = |index| word(index) & WORD_VALID_MASK == WORD_VALID;
    let supported = word(IDENTIFY_ENABLED_2) & ENABLED_2_WORDS_119_120 != 0
        && valid(IDENTIFY_COMMANDS_3)
        && word(IDENTIFY_COMMANDS_3) & SENSE_DATA_REPORTING != 0;
    let enabled = supported
        && valid(IDENTIFY_ENABLED_3)
        && word(IDENTIFY_ENABLED_3) & SENSE_DATA_REPORTING != 0;
    (supported, enabled)
}

fn identify(data: &[u8]) -> Result<Identified, BlockError> {
    let word = |index: usize| u16::from_le_bytes([data[2 * index], data[2 * index + 1]]);
    // ATA strings hold two characters per word, high byte first.
    let string = |words: core::ops::Range<usize>| {
        let bytes: Vec<u8> = words.flat_map(|index| word(index).to_be_bytes()).collect();
        ascii(&bytes)
    };
    if word(IDENTIFY_COMMANDS_2) & COMMANDS_2_LBA48 == 0 {
        return Err(BlockError::Unsupported(
            "SATA disk without 48-bit addressing",
        ));
    }
    let blocks = IDENTIFY_LBA48_SECTORS
        .rev()
        .fold(0u64, |value, index| value << 16 | u64::from(word(index)));
    let sector = word(IDENTIFY_SECTOR_SIZE);
    let block_size = if sector & SECTOR_SIZE_VALID_MASK == SECTOR_SIZE_VALID
        && sector & SECTOR_SIZE_LOGICAL_LONG != 0
    {
        // The logical sector size in 16-bit words.
        (u32::from(word(IDENTIFY_LOGICAL_WORDS))
            | u32::from(word(IDENTIFY_LOGICAL_WORDS + 1)) << 16)
            * 2
    } else {
        ATA_SECTOR_BYTES
    };
    if !(ATA_SECTOR_BYTES..=BOUNCE_BYTES as u32).contains(&block_size)
        || !block_size.is_power_of_two()
    {
        return Err(BlockError::Unsupported("SATA logical sector size"));
    }
    // ata_id_c_string: up to a NUL, without trailing spaces.
    let c_string = |words: core::ops::Range<usize>| {
        let mut bytes: Vec<u8> = words.flat_map(|index| word(index).to_be_bytes()).collect();
        if let Some(end) = bytes.iter().position(|byte| *byte == 0) {
            bytes.truncate(end);
        }
        while bytes.last() == Some(&b' ') {
            bytes.pop();
        }
        bytes
    };
    let sata_valid = !matches!(word(IDENTIFY_SATA_CAPABILITY), 0 | 0xffff);
    Ok(Identified {
        model: string(IDENTIFY_MODEL),
        serial: string(IDENTIFY_SERIAL),
        blocks,
        block_size,
        flush: flush_command(word),
        sense_reporting: sense_reporting(word),
        transfer: Transfer::from_identify(word),
        locked: word(IDENTIFY_SECURITY) & SECURITY_LOCKED == SECURITY_LOCKED,
        hipm: sata_valid && word(IDENTIFY_SATA_CAPABILITY) & SATA_CAPABILITY_HIPM != 0,
        dipm: sata_valid && word(IDENTIFY_SATA_FEATURES) & SATA_FEATURE_DIPM != 0,
        ncq: word(IDENTIFY_SATA_CAPABILITY) & SATA_CAPABILITY_NCQ != 0,
        fpdma_aa: sata_valid && word(IDENTIFY_SATA_FEATURES) & SATA_FEATURE_FPDMA_AA != 0,
        ncq_autosense: word(IDENTIFY_SATA_FEATURES) & SATA_FEATURE_NCQ_AUTOSENSE != 0,
        read_log_dma: word(IDENTIFY_ENABLED_2) & ENABLED_2_WORDS_119_120 != 0
            && word(IDENTIFY_COMMANDS_3) & COMMANDS_3_READ_LOG_DMA != 0,
        queue_depth: usize::from(word(IDENTIFY_QUEUE_DEPTH) & QUEUE_DEPTH_MASK) + 1,
        quirks: quirks::quirks(&c_string(IDENTIFY_MODEL), &c_string(IDENTIFY_FIRMWARE)),
        sata_speeds: word(IDENTIFY_SATA_CAPABILITY) & 0xe,
        devslp: word(IDENTIFY_SATA_FEATURES) & SATA_FEATURE_DEVICE_SLEEP != 0,
        trim: word(IDENTIFY_DATA_SET_MANAGEMENT) & DATA_SET_MANAGEMENT_TRIM != 0,
        rotation: Rotation::from_nominal(word(IDENTIFY_ROTATION)),
    })
}

/// A failed command as libata's error handler sees it.
#[derive(Clone, Debug)]
struct Failure {
    error: BlockError,
    /// The `AC_ERR_*` mask.
    mask: u32,
    /// The port froze: only a reset recovers it.
    frozen: bool,
    /// `ATA_EH_RESET` was requested.
    reset: bool,
    /// `ata_ehi_hotplugged`: debounce with the hotplug timing.
    hotplug: bool,
    /// The status reported sense data, without a device fault.
    sense_reported: bool,
    /// `ATA_QCFLAG_SENSE_VALID`: the sense data fetched for it.
    sense: Option<Sense>,
    /// PxIS's error bits and SError.
    interrupts: u32,
    serror: u32,
    /// A queued command failed.
    queued: bool,
    /// The slots of the queued commands it reached (`link->sactive`), and
    /// the one the NCQ Command Error log named.
    slots: u32,
    named: Option<usize>,
    /// The result taskfile's status and error (`qc->result_tf`), captured
    /// from the FIS the command completed with.
    result: [u8; 2],
}

impl Failure {
    /// A failure no command analysis applies to.
    fn new(error: BlockError) -> Self {
        Self {
            error,
            mask: 0,
            frozen: false,
            reset: false,
            hotplug: false,
            sense_reported: false,
            sense: None,
            interrupts: 0,
            serror: 0,
            queued: false,
            slots: 0,
            named: None,
            result: [0; 2],
        }
    }

    /// `ata_exec_internal`'s minimal analysis gives exactly `AC_ERR_DEV`:
    /// the device refused the command, which did not time out.
    fn refused(&self) -> bool {
        self.mask & ERR_TIMEOUT == 0
            && (self.interrupts & INTERRUPT_TASK_FILE_ERROR != 0
                || self.result[0] & (STATUS_ERROR | STATUS_FAULT) != 0)
    }
}

/// Linux `ata_to_sense_error`, from the 3ware translation table: the error
/// register's best match first, then the status register.
fn translate_error(status: u8, error: u8) -> Sense {
    const ERRORS: [(u8, u8, u8, u8); 13] = [
        (0xd1, Sense::ABORTED_COMMAND, 0x00, 0x00),
        (0xd0, Sense::ABORTED_COMMAND, 0x00, 0x00),
        (0x61, Sense::HARDWARE_ERROR, 0x00, 0x00),
        (0x84, Sense::ABORTED_COMMAND, 0x47, 0x00),
        (0x37, Sense::NOT_READY, 0x04, 0x00),
        (0x09, Sense::NOT_READY, 0x04, 0x00),
        (0x01, Sense::MEDIUM_ERROR, 0x13, 0x00),
        (0x02, Sense::HARDWARE_ERROR, 0x00, 0x00),
        (0x08, Sense::NOT_READY, 0x04, 0x00),
        (0x10, Sense::ILLEGAL_REQUEST, 0x21, 0x00),
        (0x20, Sense::UNIT_ATTENTION, 0x28, 0x00),
        (0x40, Sense::MEDIUM_ERROR, 0x11, 0x04),
        (0x80, Sense::MEDIUM_ERROR, 0x11, 0x04),
    ];
    const STATUSES: [(u8, u8, u8, u8); 3] = [
        // Busy means no other bit is valid.
        (STATUS_BUSY, Sense::ABORTED_COMMAND, 0x00, 0x00),
        (STATUS_FAULT, Sense::HARDWARE_ERROR, 0x44, 0x00),
        (STATUS_CORRECTED, Sense::RECOVERED_ERROR, 0x00, 0x00),
    ];
    let sense = |(_, key, asc, ascq): (u8, u8, u8, u8)| Sense { key, asc, ascq };
    let error = if status & STATUS_BUSY != 0 { 0 } else { error };
    // ABRT alone is not descriptive enough: the status decides.
    ERRORS
        .iter()
        .find(|entry| error != 0 && entry.0 & error == entry.0)
        .or_else(|| STATUSES.iter().find(|entry| entry.0 & status != 0))
        .copied()
        .map_or(
            Sense {
                key: Sense::ABORTED_COMMAND,
                asc: 0,
                ascq: 0,
            },
            sense,
        )
}

/// Linux `ahci_error_intr`, `ata_eh_analyze_serror` and `ata_eh_analyze_tf`
/// for a single non-NCQ command on a port without a multiplier. With `lpm`
/// (`link->lpm_policy > ATA_LPM_MAX_POWER`) SError's PhyRdy and exchange
/// bits are no hotplug: the link's own power transitions set them.
fn classify(interrupts: u32, serror: u32, task_file: u32, timed_out: bool, lpm: bool) -> Failure {
    let mut mask = 0;
    let mut reset = false;
    let mut hotplug = false;
    if interrupts & INTERRUPT_TASK_FILE_ERROR != 0 {
        mask |= ERR_DEVICE;
    }
    if interrupts & INTERRUPT_UNKNOWN_FIS != 0 {
        mask |= ERR_HSM;
        reset = true;
    }
    if interrupts & (INTERRUPT_HOST_BUS_FATAL | INTERRUPT_HOST_BUS_DATA) != 0 {
        mask |= ERR_HOST_BUS;
        reset = true;
    }
    if interrupts & INTERRUPT_INTERFACE_FATAL != 0 {
        mask |= ERR_ATA_BUS;
        reset = true;
    }
    let hotplug_serror = if lpm {
        0
    } else {
        SERROR_PHY_READY_CHANGE | SERROR_DEVICE_EXCHANGED
    };
    if interrupts & (INTERRUPT_CONNECT_CHANGE | INTERRUPT_PHY_READY_CHANGE) != 0
        || serror & hotplug_serror != 0
    {
        hotplug = true;
    }
    if serror & (SERROR_PERSISTENT | SERROR_DATA) != 0 {
        mask |= ERR_ATA_BUS;
        reset = true;
    }
    if serror & SERROR_PROTOCOL != 0 {
        mask |= ERR_HSM;
        reset = true;
    }
    if serror & SERROR_INTERNAL != 0 {
        mask |= ERR_SYSTEM;
        reset = true;
    }
    if hotplug {
        mask |= ERR_ATA_BUS;
        reset = true;
    }
    // A timed-out command freezes the port.
    let frozen = interrupts & INTERRUPT_FREEZE != 0 || timed_out;
    if timed_out {
        mask |= ERR_TIMEOUT;
    }
    let status = task_file as u8;
    let error = (task_file >> 8) as u8;
    let mut sense_reported = false;
    if status & (STATUS_BUSY | STATUS_REQUEST | STATUS_READY) != STATUS_READY {
        mask |= ERR_HSM;
        reset = true;
    } else if status & (STATUS_ERROR | STATUS_FAULT) != 0 {
        mask |= ERR_DEVICE;
        // Sense data reporting does not work if the device fault bit is set.
        sense_reported = status & (STATUS_SENSE | STATUS_FAULT) == STATUS_SENSE;
        if error & ERROR_INTERFACE_CRC != 0 {
            mask |= ERR_ATA_BUS;
        }
        if error & (ERROR_UNCORRECTABLE | ERROR_ADDRESS_MARK) != 0 {
            mask |= ERR_MEDIA;
        }
        if error & ERROR_ID_NOT_FOUND != 0 {
            mask |= ERR_INVALID;
        }
        if mask & (ERR_HSM | ERR_TIMEOUT | ERR_ATA_BUS) != 0 {
            reset = true;
        }
    }
    // Device errors are probably spurious beside an ATA bus error.
    if mask & ERR_ATA_BUS != 0 {
        mask &= !(ERR_DEVICE | ERR_MEDIA | ERR_INVALID);
    }
    let error = if timed_out {
        BlockError::Timeout
    } else if interrupts & !INTERRUPT_TASK_FILE_ERROR != 0 {
        BlockError::Device { status: interrupts }
    } else {
        BlockError::Device { status: task_file }
    };
    Failure {
        mask,
        frozen,
        reset,
        hotplug,
        sense_reported,
        interrupts,
        serror,
        ..Failure::new(error)
    }
}

/// Linux `ata_scsi_sense_is_valid`.
fn valid_sense(key: u8, asc: u8, ascq: u8) -> Option<Sense> {
    (key <= SENSE_COMPLETED && (key, asc, ascq) != (0, 0, 0)).then_some(Sense { key, asc, ascq })
}

/// Linux `ata_eh_worth_retry`.
fn worth_retry(mask: u32, io: bool) -> bool {
    if mask & ERR_MEDIA != 0 {
        return false;
    }
    if io {
        return true;
    }
    if mask & ERR_INVALID != 0 {
        return false;
    }
    mask != ERR_DEVICE
}

/// Linux `ahci_start_engine`.
fn start_engine(port: &impl CommandPort) {
    port.write32(PORT_COMMAND, port.read32(PORT_COMMAND) | COMMAND_START);
    port.read32(PORT_COMMAND);
}

/// Linux `ahci_stop_engine`: clear ST and wait for the command list to stop.
fn stop_engine(port: &impl CommandPort, clock: Clock) -> Result<(), BlockError> {
    let command = port.read32(PORT_COMMAND);
    if command & (COMMAND_START | COMMAND_LIST_RUNNING) == 0 {
        return Ok(());
    }
    if command == u32::MAX {
        return Err(BlockError::Unsupported(
            "the AHCI controller is unavailable",
        ));
    }
    port.write32(PORT_COMMAND, command & !COMMAND_START);
    if !clock.wait(STOP_ENGINE_TIMEOUT_US, || {
        port.read32(PORT_COMMAND) & COMMAND_LIST_RUNNING == 0
    }) {
        return Err(BlockError::Timeout);
    }
    Ok(())
}

/// Linux `sata_link_debounce`: wait until SStatus.DET holds still for the
/// stable duration, or a DET of 1 settles, before the limit.
fn debounce(
    port: &impl CommandPort,
    clock: Clock,
    minimum: &Control,
    timing: [u64; 3],
    limit_us: u64,
) -> Result<(), BlockError> {
    match debounce_ports(&[port], clock, minimum, timing, limit_us)?[..] {
        [Debounced { settled: true, .. }] => Ok(()),
        _ => Err(BlockError::Timeout),
    }
}

/// A port's link reset ([`AhciDisk::link_ports`]): PxCMD before the port
/// stopped, the SPD limit and SControl without DET.
pub(super) struct Link {
    command: u32,
    speed_limit: u32,
    control: u32,
}

/// One port's SStatus.DET once [`debounce_ports`] stopped watching it, and
/// whether it held still rather than changing past the limit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Debounced {
    detection: u32,
    settled: bool,
}

/// [`debounce`] for several ports at once, each judged on its own while
/// one wait serves all, as Linux debounces every port's link in parallel.
fn debounce_ports<P: CommandPort>(
    ports: &[&P],
    clock: Clock,
    minimum: &Control,
    timing: [u64; 3],
    limit_us: u64,
) -> Result<Vec<Debounced>, BlockError> {
    minimum.check()?;
    let [interval, duration, timeout] = timing;
    if interval == 0 || duration.checked_add(interval).is_none() {
        return Err(BlockError::Protocol("invalid AHCI debounce interval"));
    }
    let limit = limit_us.min(timeout);
    let start = clock.try_start().map_err(|_| BlockError::Timeout)?;
    let elapsed = || clock.try_elapsed_us(start).map_err(|_| BlockError::Timeout);
    let detection = |port: &P| port.read32(PORT_SATA_STATUS) & SATA_STATUS_DETECTION;
    let mut ports: Vec<(&P, Debounced, u64, bool)> = ports
        .iter()
        .map(|port| {
            let last = Debounced {
                detection: detection(port),
                settled: false,
            };
            (*port, last, 0, false)
        })
        .collect();
    while ports.iter().any(|(.., done)| !done) {
        // Actual lower-bound waits establish stability; nominal elapsed time
        // controls only refusal/deadlines and cannot shorten this minimum.
        minimum.wait_after(interval, || {})?;
        let now = elapsed()?;
        for (port, last, stable, done) in ports.iter_mut().filter(|port| !port.3) {
            let current = detection(port);
            if current == last.detection {
                if current == 1 && now < limit {
                    continue;
                }
                *stable = stable.saturating_add(interval);
                if *stable > duration {
                    (last.settled, *done) = (true, true);
                }
                continue;
            }
            last.detection = current;
            *stable = 0;
            if now > limit {
                *done = true;
            }
        }
    }
    Ok(ports.into_iter().map(|(_, last, ..)| last).collect())
}

/// The disks behind one controller, and the ports that failed on their own.
pub struct Attached {
    pub host: Host,
    pub disks: Vec<AhciDisk>,
    pub failures: Vec<(u32, BlockError)>,
    /// NVMe controllers remapped behind the BAR of an Intel controller in
    /// RAID mode ([`REMAPPED_NVME`]), which stay unreachable.
    pub remapped_nvme: u32,
}

/// What to report once for an [`Attached::remapped_nvme`] other than 0.
pub const REMAPPED_NVME: &str = "NVMe drives are remapped behind the SATA controller in RAID mode; set the firmware's SATA mode to AHCI to use them";

/// Take the HBA from the firmware, reset it and return one device per port
/// with an ATA disk behind it.
pub fn attach(
    function: Function,
    handoff: &Handoff,
    dma: &mut dyn DmaAllocator,
    minimum: Rc<dyn crate::bus::pci::power::Timer>,
) -> Result<Attached, AttachRefusal> {
    let mut kept = crate::Retained::new(alloc::boxed::Box::new(minimum::Build {
        host: Host::new(function, minimum),
        hba: None,
        disks: Vec::new(),
        failures: Vec::new(),
        remapped_nvme: 0,
    }));
    let result = (|| {
        let control = kept.host.control.clone();
        control.check()?;
        validate_clock(handoff.clock)?;
        let mut owned_function = control.function.borrow_mut();
        let function = owned_function
            .as_mut()
            .ok_or(BlockError::Protocol("AHCI host lost its function"))?;
        function.power_up(control.minimum.as_ref())?;
        let bars = function.bars()?;
        kept.get_mut().hba = Some(registers(handoff, bars[5], 0, PORT_BASE)?);
        let hba = kept.hba.expect("same checked HBA");
        function.enable_polled()?;
        // MSI or MSI-X only: without either the HBA stays polled, INTx
        // disabled.
        let signalled = match handoff.completion {
            Some(message) => function.enable_message(handoff, &bars, message)?,
            None => false,
        };
        let clock = handoff.clock;
        if hba.read32(CAPABILITIES_2) & CAP2_HANDOFF != 0 {
            hba.write32(HANDOFF, hba.read32(HANDOFF) | HANDOFF_OS_OWNED);
            clock.wait(25_000, || hba.read32(HANDOFF) & HANDOFF_BIOS_OWNED == 0);
            if hba.read32(HANDOFF) & HANDOFF_BIOS_BUSY != 0 {
                clock.wait(2_000_000, || hba.read32(HANDOFF) & HANDOFF_BIOS_BUSY == 0);
            }
            if hba.read32(HANDOFF) & HANDOFF_BIOS_OWNED != 0 {
                return Err(BlockError::Unsupported(
                    "firmware keeps the AHCI controller",
                ));
            }
        }
        hba.write32(GLOBAL_CONTROL, GLOBAL_AHCI_ENABLE);
        let capabilities = hba.read32(CAPABILITIES);
        let capabilities_2 = hba.read32(CAPABILITIES_2);
        let implemented = port_map(capabilities, hba.read32(PORTS_IMPLEMENTED));
        hba.write32(GLOBAL_CONTROL, GLOBAL_AHCI_ENABLE | GLOBAL_RESET);
        if !clock.wait(RESET_TIMEOUT_US, || {
            hba.read32(GLOBAL_CONTROL) & GLOBAL_RESET == 0
        }) {
            return Err(BlockError::Timeout);
        }
        hba.write32(GLOBAL_CONTROL, GLOBAL_AHCI_ENABLE);
        hba.write32(CAPABILITIES, capabilities);
        if capabilities_2 != 0 {
            hba.write32(CAPABILITIES_2, capabilities_2);
        }
        hba.write32(PORTS_IMPLEMENTED, implemented);
        hba.read32(PORTS_IMPLEMENTED);
        if function.vendor == INTEL && INTEL_PCS_QUIRK.contains(&function.device) {
            let enables = implemented as u16;
            let value = function.config().read16(INTEL_PORT_CONTROL)?;
            if value & enables != enables {
                function
                    .config()
                    .write16(INTEL_PORT_CONTROL, value | enables)?;
            }
        }
        let address = function.address;
        kept.get_mut().remapped_nvme = remapped_nvme(handoff, function, bars[5]);
        let mut ports = Vec::new();
        for index in (0..MAX_PORTS).filter(|index| implemented >> index & 1 == 1) {
            let offset = PORT_BASE + PORT_REGISTERS * u64::from(index);
            ports.push((index, registers(handoff, bars[5], offset, PORT_REGISTERS)?));
        }
        // Every link resets and debounces at once; a port whose link stays
        // down is empty.
        let links = AhciDisk::link_ports(
            &ports.iter().map(|(_, port)| port).collect::<Vec<_>>(),
            capabilities,
            clock,
            &control,
        );
        for ((index, port), link) in ports.into_iter().zip(links) {
            // A prior minimum failure forbids even starting the next port.
            control.ready()?;
            let started = match link {
                Ok(Some(link)) => AhciDisk::configure_port(
                    port,
                    link,
                    capabilities,
                    capabilities_2,
                    clock,
                    dma,
                    control.clone(),
                ),
                Ok(None) => Ok(None),
                Err(error) => Err(PortRefusal::unstarted(error, control.clone(), port)),
            };
            match started {
                Ok(Some(mut disk)) => {
                    disk.identity.location = format!("{address} port {index}");
                    if signalled {
                        let port = disk.port;
                        disk.enable_signal(&port, hba, index);
                    }
                    kept.get_mut().disks.push(disk);
                }
                Ok(None) => {}
                Err(refusal) => {
                    let error = refusal.error.clone();
                    // Keep the actual partial port BEFORE any other decision.
                    kept.get_mut().host.retained_ports.push(refusal);
                    control.ready()?;
                    kept.get_mut().failures.push((index, error));
                }
            }
        }
        // ahci_init_controller: IS cleared, then GHC.IE set, once a port
        // sends messages.
        if kept.disks.iter().any(|disk| disk.signal.is_some()) {
            hba.write32(HOST_INTERRUPTS, hba.read32(HOST_INTERRUPTS));
            hba.write32(GLOBAL_CONTROL, GLOBAL_AHCI_ENABLE | GLOBAL_INTERRUPT_ENABLE);
        }
        Ok(())
    })();
    if let Err(error) = result {
        return Err(AttachRefusal::new(error, kept));
    }
    let minimum::Build {
        host,
        hba: _,
        disks,
        failures,
        remapped_nvme,
    } = *kept.into_inner();
    Ok(Attached {
        host,
        disks,
        failures,
        remapped_nvme,
    })
}

/// Linux `ahci_remap_check`: the NVMe controllers an Intel SATA controller
/// in RAID mode hides behind its own BAR, which no driver here reaches.
/// The firmware's SATA mode must be AHCI for them to appear. Registers the
/// mount did not map are not looked at.
fn remapped_nvme(handoff: &Handoff, function: &Function, bar: Option<crate::bus::pci::Bar>) -> u32 {
    let large = bar
        .and_then(|bar| bar.memory())
        .is_some_and(|range| range.end - range.start >= REMAP_BAR_BYTES);
    if function.vendor != INTEL || !large {
        return 0;
    }
    let Ok(hba) = registers(handoff, bar, 0, REMAP_REGISTERS) else {
        return 0;
    };
    if hba.read32(VENDOR_CAPABILITIES) & 1 == 0 {
        return 0;
    }
    let capabilities = hba.read64(REMAP_CAPABILITIES);
    (0..REMAP_DEVICES)
        .filter(|&index| {
            capabilities >> index & 1 != 0
                && hba.read32(REMAP_CLASS + REMAP_STRIDE * index) == REMAP_NVME_CLASS
        })
        .count() as u32
}
/// The implemented ports. An unprogrammed PI falls back to CAP.NP's count,
/// as Linux `ahci_save_initial_config` does.
fn port_map(capabilities: u32, implemented: u32) -> u32 {
    if implemented != 0 {
        return implemented;
    }
    let ports = (capabilities & CAP_PORTS) + 1;
    u32::MAX >> (MAX_PORTS - ports)
}

fn validate_clock(clock: Clock) -> Result<(), BlockError> {
    clock.check_duration(COMMAND_TIMEOUT_US).map_err(|_| {
        BlockError::Unsupported("AHCI requires a calibrated, representable operation clock")
    })
}

/// Stop the command list engine, then FIS reception, each waiting for the
/// port to report it stopped (AHCI 1.3.1 Â§10.1.2; Linux `ahci_stop_engine`,
/// `ahci_stop_fis_rx`).
fn stop_port(port: &impl CommandPort, clock: Clock) -> Result<(), BlockError> {
    validate_clock(clock)?;
    stop_engine(port, clock)?;
    port.write32(
        PORT_COMMAND,
        port.read32(PORT_COMMAND) & !COMMAND_FIS_RECEIVE,
    );
    if !clock.wait(STOP_FIS_TIMEOUT_US, || {
        port.read32(PORT_COMMAND) & COMMAND_FIS_RUNNING == 0
    }) {
        return Err(BlockError::Timeout);
    }
    Ok(())
}

impl AhciDisk {
    /// Linux `sata_lpm_ignore_phy_events`: a PhyRdy change means nothing
    /// while LPM is on, nor in the first [`SPURIOUS_PHY_US`] after a policy
    /// change. This driver polls PxIS, which latches PRCS whatever PxIE
    /// masks, so a look drops it here as Linux's interrupt handler does.
    fn ignore_phy_events(&self) -> bool {
        self.lpm.policy > LPM_MAX_POWER
            || self.lpm.changed.is_some_and(|at| {
                self.clock
                    .try_elapsed_us(at)
                    .is_ok_and(|elapsed| elapsed < SPURIOUS_PHY_US)
            })
    }

    fn ready(&self) -> Result<(), BlockError> {
        self.minimum.ready()?;
        self.fault.clone().map_or(Ok(()), Err)
    }

    /// PxIE under LPM policy `policy`: Linux `DEF_PORT_IRQ`'s error bits and
    /// the completion bits ([`INTERRUPT_COMPLETION`]). With LPM on,
    /// `ahci_set_lpm` masks PhyRdy changes; COMINIT's port connect change is
    /// masked with them, as a look ignores both ([`Self::ignore_phy_events`]).
    fn interrupt_mask(policy: u8) -> u32 {
        let mask = INTERRUPT_ERROR | INTERRUPT_COMPLETION;
        if policy > LPM_MAX_POWER {
            mask & !(INTERRUPT_PHY_READY_CHANGE | INTERRUPT_CONNECT_CHANGE)
        } else {
            mask
        }
    }

    /// Let the port's completions send the HBA's interrupt message, port
    /// `index` in IS, once its MSI or MSI-X is programmed; GHC.IE follows
    /// for the HBA.
    fn enable_signal(&mut self, port: &impl CommandPort, hba: Mmio, index: u32) {
        self.signal = Some(Signal { hba, port: index });
        port.write32(PORT_INTERRUPT_ENABLE, Self::interrupt_mask(self.lpm.policy));
    }

    /// PxIE for LPM policy `policy`, on a port that sends messages.
    fn mask_interrupts(&self, port: &impl CommandPort, policy: u8) {
        if self.signal.is_some() {
            port.write32(PORT_INTERRUPT_ENABLE, Self::interrupt_mask(policy));
        }
    }

    /// After a look read and cleared the port's status (Linux
    /// `ahci_handle_port_intr` after `ahci_port_intr`): its IS bit, so the
    /// next completion sends the next message.
    fn clear_signal(&self) {
        if let Some(signal) = self.signal {
            signal.hba.write32(HOST_INTERRUPTS, 1 << signal.port);
        }
    }

    fn chunk_bytes(&self) -> usize {
        let block = self.identity.block_size as usize;
        BOUNCE_BYTES / block * block
    }

    fn sectors(&self, bytes: usize) -> u16 {
        (bytes / self.identity.block_size as usize) as u16
    }
}

impl Drop for AhciDisk {
    fn drop(&mut self) {
        if self.fault.is_some() {
            self.retain.set(true);
        }
    }
}

impl BlockDevice for AhciDisk {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        self.ready()?;
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_bytes();
        for (index, part) in buffer.chunks_mut(chunk).enumerate() {
            let first = lba + (index * chunk / block) as u64;
            let count = self.sectors(part.len());
            self.issue(ATA_READ_DMA_EXT, first, count, part.len(), false)?;
            part.copy_from_slice(&self.bounce.bytes()[..part.len()]);
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.identity.check_write(lba, data.len())?;
        self.ready()?;
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_bytes();
        for (index, part) in data.chunks(chunk).enumerate() {
            let first = lba + (index * chunk / block) as u64;
            let count = self.sectors(part.len());
            self.bounce.bytes_mut()[..part.len()].copy_from_slice(part);
            self.issue(ATA_WRITE_DMA_EXT, first, count, part.len(), true)?;
        }
        Ok(())
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        match self.flush {
            Some(command) => self.issue(command, 0, 0, 0, false),
            None => self.ready(),
        }
    }

    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        let issue = self.trim(lba, blocks, 0)?;
        let port = self.port;
        self.complete(&port, issue)
    }

    /// Flush, and before power-off spin the disk down, as Linux libata's
    /// `ata_dev_power_set_standby` does on `SYSTEM_POWER_OFF`.
    fn shutdown(&mut self, power_off: bool) -> Result<(), BlockError> {
        self.flush_cache()?;
        if power_off {
            let port = self.port;
            let buffer = self.bounce.bus_address();
            self.internal(&port, ATA_STANDBY_IMMEDIATE, 0, 0, 0, buffer)
                .map_err(|failure| failure.error)?;
        }
        Ok(())
    }

    /// A command per slot while NCQ is on, else one in slot 0, started
    /// under the caller's lock and completed by a later look
    /// ([`Self::finish`]).
    fn depth(&self) -> usize {
        if self.ready().is_ok() {
            self.queue_depth()
        } else {
            0
        }
    }

    /// A free slot: slot 0 with its own bounce buffer, any other while the
    /// pool holds the pages of the largest command.
    fn room(&self) -> bool {
        self.ready().is_ok()
            && match self.free_slot() {
                Some(0) => true,
                Some(_) => self.pool.free.len() >= CHUNK_PAGES,
                None => false,
            }
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        let tag = self.begin(command)?;
        let port = self.port;
        self.run(&port);
        Ok(tag)
    }

    fn finish(&mut self, tag: u32) -> Option<Finished> {
        let index = self.running.iter().position(|running| running.tag == tag)?;
        if self.running[index].result.is_none() {
            let port = self.port;
            self.run(&port);
        }
        let index = self.running.iter().position(|running| running.tag == tag)?;
        let result = self.running[index].result.take()?;
        let slot = self.running.remove(index).slot;
        self.release(slot);
        Some(result)
    }

    /// Whether the port's completions send the HBA's interrupt message.
    fn signals(&self) -> bool {
        self.signal.is_some()
    }

    /// A started command has a result, or a launched one's ending FIS has
    /// arrived: memory alone. A failure that posts none is seen at a later
    /// look.
    fn posted(&self) -> bool {
        if self.minimum.ready().is_err() {
            return !self.running.is_empty();
        }
        self.running.iter().any(|running| {
            running.result.is_some()
                || running
                    .issue
                    .as_ref()
                    .and_then(|issue| issue.launched.as_ref())
                    .is_some_and(|launched| {
                        ending(launched.queued)
                            .iter()
                            .any(|&(at, kind)| self.commands.read::<u8>(at) == kind)
                    })
        })
    }
}

#[cfg(test)]
mod tests;
