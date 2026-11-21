//! The Apple GPUs 26A434 supports: each SoC's `sgx` compatible, the AGX kext
//! its kernelcache links, the GFX firmware its BuildManifest names, and the
//! facts each kext fixes about the AP-visible interface (from a census of
//! all 59 board DeviceTrees and all 13 AGX kexts).
//!
//! Known (every 26A434 kernelcache, `AGXAccelerator*::readChipInfo`,
//! `AGXSecureMonitor*::setupMMUConfig`/`invalidateGPUTTBACache` and
//! `AGXArmFirmware::initFirmwareData` of each kext; offsets per kext in §6):
//! - every kext reads the identification words at 0xd04000.. and requires
//!   one generation in bits 31:24 of 0xd04000 ([`Family::generations`]);
//! - G13's MMU block puts the TTBAT page at 0xd08024 and the TTBA-cache
//!   invalidation at 0xd08028 behind eight fixed words; G14 onwards use
//!   0xd0802c and 0xd08030 behind ten ([`Mmu`]). The page field is 26 bits on
//!   the G13G and G14G kexts and 28 bits on every other;
//! - the initdata root's first word differs between interface revisions; the
//!   G13G, G13X and G14G kexts write the same one ([`Family::initdata_magic`]).
//!
//! A family is admitted ([`Gpu::admitted`]) only when every layout the GFX
//! firmware model and the `sgx` model use has been checked against that
//! family's own kext and firmware ([`Interface::Modeled`] carries them as a
//! [`Model`]). Today that is G14G, the two G13 families, whose kexts write
//! the same initdata, rings and channels (aligned function by function) and
//! whose firmware runs the same device-control and signal handlers except
//! the dashboard signal it lacks ([`Family::dashboard_signals`]), and G14X,
//! whose kext and firmware keep the same protocol with its own initdata
//! magic, shared-object offsets ([`Layout`]), identification words and GPC
//! block, and G15G, which keeps G14X's protocol
//! with its own initdata magic and three shared offsets. Every other family
//! is named here with the facts recovered so far and stays hidden by the
//! loader, its refusal naming what is missing. A modeled family's GPU is
//! admitted only with an identification a source states ([`Gpu::identity`]).

use super::Identity;
use crate::Result;
use stela_contracts::refuse;

/// How `readChipInfo` reads the GPU's size from the identification words.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Identification {
    /// 0xd04010 holds the cores (bits 7:0) and fragment units (15:8) per
    /// cluster, and 0xd0401c bits 19:12 the clusters (`AGXAcceleratorG13/
    /// G14::readChipInfo`).
    Clusters,
    /// 0xd04010 holds the cores per cluster (bits 7:0), the clusters per die
    /// (15:8) and the dies (19:16); the kext reads the fragment units as the
    /// cores and 0xd0401c names no count (`AGXAcceleratorG14X::readChipInfo`;
    /// Public, Asahi `regs.rs` `get_gpu_id`).
    Dies,
}

/// Where a firmware interface keeps the shared words the device-control
/// model writes (`AGXArmFirmware::initFirmwareData` and the firmware's
/// device-control handlers).
#[derive(Debug, PartialEq, Eq)]
pub struct Layout {
    /// The handled-entry count in the root's +0xb0 object, which the
    /// firmware increments after each counted entry.
    pub device_control_count: u64,
    /// Bytes of one slot of the HWDSID table the ring directory's +0x2c8
    /// names (+0 the requested priority, +4 the firmware's matching word).
    pub dsid_slot_bytes: u64,
    /// The ring directory's context-switch histogram enables byte, one bit
    /// per data master, followed by the histograms
    /// (`getDMContextSwitchTimeHistogram` reads them).
    pub histogram_enables: u64,
    /// The power-counter sample count and the samples in the root's +0xb0
    /// object (`getAndGatherPowerCounterData` reads them).
    pub power_counter_samples: u64,
    pub power_counters: u64,
    /// The RIART U-level disable mask and T-disable word in the root's
    /// +0x20 object.
    pub riart_u_levels_disabled: u64,
    pub riart_t_disabled: u64,
    /// The firmware's recovery handshake words, where recovered.
    pub recovery: Option<RecoveryWords>,
}

/// The recovery handshake's words in the root's +0xb0 object
/// (`AGXArmFirmware::isFirmwareStoppedFor*Recovery`, `resumeFirmware*Recovery`
/// and `stopFirmwareForRecoveryInspection`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct RecoveryWords {
    /// The u64 the firmware advances once stopped.
    pub generation: u64,
    /// 1 stopped before recovery, 2 resumed, 3 stopped after it, 0 running.
    pub stop_state: u64,
    /// Nonzero once stopped (`isFirmwareStoppedForHostRecovery`).
    pub stopped: u64,
}

/// The G13G, G13X and G14G interface (`armfw_g14g` handlers 0xffffff8000004b8c,
/// 0xffffff80000043a0, 0xffffff8000003e88, 0xffffff8000003b20,
/// 0xffffff8000004678 and 0xffffff8000003d24).
const G13_G14G_LAYOUT: Layout = Layout {
    device_control_count: 0xe838,
    dsid_slot_bytes: 0x10,
    histogram_enables: 0x3e4,
    power_counter_samples: 0xca30,
    power_counters: 0xca48,
    riart_u_levels_disabled: 0xe30,
    riart_t_disabled: 0xe34,
    recovery: Some(RecoveryWords {
        generation: 0x4bf0,
        stop_state: 0x4c00,
        stopped: 0x4c10,
    }),
};

/// The G14X interface (`armfw_g14s`, `armfw_g14c` and `armfw_g14d`, whose
/// handlers differ only in their own globals: the tail at
/// 0xffffff800000497c, 0xe at 0xffffff8000004554, 0x14 at
/// 0xffffff8000003b78, the histogram byte at 0xffffff8000004a30, 0x32 at
/// 0xffffff8000003ad4, 0x34 at 0xffffff80000047d4 and 0x35 at
/// 0xffffff8000003c6c of `armfw_g14s`), matching the `AGXG14X` kext's
/// `initFirmwareData` stores.
const G14X_LAYOUT: Layout = Layout {
    device_control_count: 0xedc8,
    dsid_slot_bytes: 8,
    histogram_enables: 0x3d4,
    power_counter_samples: 0xcfc0,
    power_counters: 0xcfd8,
    riart_u_levels_disabled: 0x123c,
    riart_t_disabled: 0x1240,
    recovery: None,
};

/// The G15G interface (`armfw_g15g` c000: the dispatcher at
/// 0xfffffc00000039ac and its tail at 0xfffffc0000005290, 0xe at
/// 0xfffffc0000004030, 0x14 at 0xfffffc0000004b84, the histogram byte at
/// 0xfffffc0000003b4c and 0xfffffc0000005270, 0x32 at 0xfffffc0000003f44, 0x34
/// at 0xfffffc0000004abc and 0x35 at 0xfffffc0000004cd0; the root's objects
/// held in the globals it copies them to at 0xfffffc00000364c8..). G14X's
/// shared objects and offsets, except the histogram enables, four bytes
/// higher, and the RIART words, packed at +0xe35 and +0xe39 of the root's
/// +0x20 object; after storing one, the firmware also reprograms its own
/// registers (0xfffffc000002d108), which the AP does not share.
const G15G_LAYOUT: Layout = Layout {
    histogram_enables: 0x3d8,
    riart_u_levels_disabled: 0xe35,
    riart_t_disabled: 0xe39,
    ..G14X_LAYOUT
};

/// What the models need of a family they implement.
#[derive(Debug, PartialEq, Eq)]
pub struct Model {
    pub identification: Identification,
    /// The GPC block `AGXFenderCommon` reads: the performance state at +0,
    /// the host-interrupt enables at +0x10 and their status at +0x14.
    pub gpc: u64,
    /// Bits of a UAT output address (Public: Asahi `hw/*.rs` `uat_oas`,
    /// pinned `94fb23346d522edf53722357c426a3e58030beea`: 40 on T8103 and
    /// T8112, 42 on T600x and T602x, whose DRAM lies at 1 TiB).
    pub uat_output_bits: u32,
    /// The native ASC control word required by a recovered firmware callback.
    /// Only G14G's global callback has this checked binding (26A434
    /// armfw_g14g startup 0x371b0..0x371f8 and kept wake 0x378d0..0x37904).
    /// Other families' unrecovered callbacks remain protocol refusals; their
    /// SGX apertures must not inherit G14G's private physical requirement.
    pub native_asc_control: Option<u64>,
    pub layout: Layout,
}

const G13G_MODEL: Model = Model {
    identification: Identification::Clusters,
    gpc: 0xd0_1000,
    uat_output_bits: 40,
    native_asc_control: None,
    layout: G13_G14G_LAYOUT,
};
const G13X_MODEL: Model = Model {
    uat_output_bits: 42,
    ..G13G_MODEL
};
const G14G_MODEL: Model = Model {
    native_asc_control: Some(0x14_0120),
    ..G13G_MODEL
};
const G14X_MODEL: Model = Model {
    identification: Identification::Dies,
    gpc: 0xe0_1000,
    uat_output_bits: 42,
    native_asc_control: None,
    layout: G14X_LAYOUT,
};
/// G15G reads its identification and GPC block as G14X does (the same
/// `readGFXReg32`/`writeGFXReg32` callers and offsets), and its boards place
/// DRAM at 1 TiB (`/chosen` `dram-base` 0x100_0000_0000 on J504AP).
const G15G_MODEL: Model = Model {
    layout: G15G_LAYOUT,
    ..G14X_MODEL
};

/// The MMU block's layout (`setupMMUConfig`, `invalidateGPUTTBACache`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Mmu {
    /// The TTBAT physical page number register.
    pub ttbat: u64,
    /// Width of its page-number field (`ubfx x2, x0, #14, #bits`).
    pub ttbat_page_bits: u32,
    /// The TTBA-cache invalidation register (`a << 8 | b`).
    pub invalidate: u64,
    /// The fixed configuration words `setupMMUConfig` writes, with the only
    /// value each accepts.
    pub configuration: &'static [(u64, u32)],
}

/// The MMU enable, read first by every `setupMMUConfig` (nonzero skips it).
pub const MMU_ENABLE: u64 = 0xd0_8000;

/// G13's fixed words (`AGXSecureMonitorG13::setupMMUConfig`).
const G13_MMU_CONFIGURATION: [(u64, u32); 8] = [
    (0xd0_800c, 0x80d8),
    (0xd0_8010, 0x80d8),
    (0xd0_8004, 0),
    (0xd0_8008, 0),
    (0xd0_8014, 0),
    (0xd0_8018, 0),
    (0xd0_801c, 0),
    (0xd0_8020, 0),
];

/// G14 and later (`AGXSecureMonitorG14::setupMMUConfig` and its successors).
const G14_MMU_CONFIGURATION: [(u64, u32); 10] = [
    (0xd0_8014, 0x80d8),
    (0xd0_8018, 0x80d8),
    (0xd0_800c, 0),
    (0xd0_8010, 0),
    (0xd0_801c, 0),
    (0xd0_8020, 0),
    (0xd0_8024, 0),
    (0xd0_8028, 0),
    (0xd0_8008, 1),
    (0xd0_8004, 0x3ff),
];

const fn g13_mmu(ttbat_page_bits: u32) -> Mmu {
    Mmu {
        ttbat: 0xd0_8024,
        ttbat_page_bits,
        invalidate: 0xd0_8028,
        configuration: &G13_MMU_CONFIGURATION,
    }
}

const fn g14_mmu(ttbat_page_bits: u32) -> Mmu {
    Mmu {
        ttbat: 0xd0_802c,
        ttbat_page_bits,
        invalidate: 0xd0_8030,
        configuration: &G14_MMU_CONFIGURATION,
    }
}

/// The core-mask words of G13 and G14G (`AGXAcceleratorG13/G14::
/// readChipInfo`: 0xd01500 and the word 0x14 above it) and of G14X onwards
/// (0xe01500 and the two words after it; Public, Asahi `regs.rs`
/// `CORE_MASKS_G14X`).
const G13_G14G_CORE_MASKS: [u64; 2] = [0xd0_1500, 0xd0_1514];
const G14X_CORE_MASKS: [u64; 3] = [0xe0_1500, 0xe0_1504, 0xe0_1508];

/// The initdata root magic of the G13G, G13X and G14G kexts.
pub const MAGIC_G13_G14G: u64 = 0x0c8a_6342_8392_04c8;

/// Initdata root magics of later kext interface revisions whose layouts the
/// models have not been checked against, as `(build, family, magic)`.
///
/// Known (26B123 Mac14,3 kernelcache, the `movk #0x0c8a, lsl #48` of
/// `AGXArmFirmware::initFirmwareData` at file offset 0x1b67818; 26A434's is
/// at 0x1b5b368): the 26B123 G14G kext writes `0x0c8a_8342_8392_04c8`, and
/// the same function reads and writes its objects at fields moved by 8 bytes
/// (for example +0x888 to +0x8a0, +0x600 to +0x608, size 0x1078 to 0x1088).
pub const UNCHECKED_INITDATA: &[(&str, &str, u64)] = &[("26B123", "G14G", 0x0c8a_8342_8392_04c8)];

/// The `(build, family)` whose unchecked kext interface revision writes
/// `magic` as its initdata root's first word.
pub fn unchecked_initdata(magic: u64) -> Option<(&'static str, &'static str)> {
    UNCHECKED_INITDATA
        .iter()
        .find(|&&(_, _, known)| known == magic)
        .map(|&(build, family, _)| (build, family))
}

/// Whether the model implements a family's interface.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Interface {
    /// Every layout the models use is checked against this family.
    Modeled(&'static Model),
    /// Not admitted, for the stated reason.
    Unrecovered(&'static str),
}

/// One AGX kext of the 26A434 kernelcaches and the interface it fixes.
#[derive(Debug, PartialEq, Eq)]
pub struct Family {
    /// The kext's family name (`AGX<name>`).
    pub name: &'static str,
    /// The kext's bundle identifier.
    pub kext: &'static str,
    /// The kernelcache that links it (`kernelcache.release.<kernelcache>`).
    pub kernelcache: &'static str,
    /// The generations `readChipInfo` accepts in 0xd04000 bits 31:24.
    pub generations: &'static [u8],
    /// The core-mask words, cluster 0's cores first (`readChipInfo`).
    pub core_masks: &'static [u64],
    pub mmu: Mmu,
    /// The initdata root's first word, when it is a constant the kext
    /// writes (`AGXArmFirmware::initFirmwareData`); `None` when the kext
    /// builds it otherwise and the census has not recovered it.
    pub initdata_magic: Option<u64>,
    /// Whether the family's kext sends dashboard-signal commands
    /// (`submitDashboardSignalCommand`, stream record 0xf). Only the G14G and
    /// G17G kexts have the sender. The G13G/G13X firmware has no handler (no
    /// trace 0x126, no record jump table entry) and the G14X firmware's
    /// record table ends at 0xe; G14G's firmware runs the record.
    pub dashboard_signals: bool,
    pub interface: Interface,
}

/// One SoC's GPU.
#[derive(Debug, PartialEq, Eq)]
pub struct Gpu {
    /// The `sgx` node's `compatible`.
    pub compatible: &'static str,
    /// The SoC's ApChipID.
    pub chip_id: u16,
    pub family: &'static Family,
    /// The GFX firmware the BuildManifest names (`Firmware/agx/<name>.im4p`).
    pub firmware: &'static str,
    /// The identification the `sgx` registers report, when a source states
    /// it.
    pub identity: Option<Identity>,
}

const NOT_CHECKED: &str = "its GFX firmware layouts have not been checked against the model's";

pub const G13G: Family = Family {
    name: "G13G",
    kext: "com.apple.AGXG13G",
    kernelcache: "mac13g",
    generations: &[4],
    core_masks: &G13_G14G_CORE_MASKS,
    mmu: g13_mmu(26),
    initdata_magic: Some(MAGIC_G13_G14G),
    dashboard_signals: false,
    interface: Interface::Modeled(&G13G_MODEL),
};
pub const G13X: Family = Family {
    name: "G13X",
    kext: "com.apple.AGXG13X",
    kernelcache: "mac13j",
    generations: &[4],
    core_masks: &G13_G14G_CORE_MASKS,
    mmu: g13_mmu(28),
    initdata_magic: Some(MAGIC_G13_G14G),
    dashboard_signals: false,
    interface: Interface::Modeled(&G13X_MODEL),
};
pub const G14G: Family = Family {
    name: "G14G",
    kext: "com.apple.AGXG14G",
    kernelcache: "mac14g",
    generations: &[5],
    core_masks: &G13_G14G_CORE_MASKS,
    mmu: g14_mmu(26),
    initdata_magic: Some(MAGIC_G13_G14G),
    dashboard_signals: true,
    interface: Interface::Modeled(&G14G_MODEL),
};
pub const G14X: Family = Family {
    name: "G14X",
    kext: "com.apple.AGXG14X",
    kernelcache: "mac14j",
    generations: &[6],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: Some(0x0c8a_636f_0494_04b8),
    dashboard_signals: false,
    interface: Interface::Modeled(&G14X_MODEL),
};
pub const G15G: Family = Family {
    name: "G15G",
    kext: "com.apple.AGXG15G",
    kernelcache: "mac15g",
    generations: &[7],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: Some(0x0c8a_636f_0394_04b8),
    dashboard_signals: false,
    interface: Interface::Modeled(&G15G_MODEL),
};
pub const G15S: Family = Family {
    name: "G15S",
    kext: "com.apple.AGXG15S",
    kernelcache: "mac15s",
    generations: &[7],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: Some(0x0c8a_636f_0394_04b8),
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G15C: Family = Family {
    name: "G15C",
    kext: "com.apple.AGXG15C",
    kernelcache: "mac15j",
    generations: &[7],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: Some(0x0c8a_636f_0496_04b8),
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G16G: Family = Family {
    name: "G16G",
    kext: "com.apple.AGXG16G",
    kernelcache: "mac16g",
    generations: &[8, 0xa],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: Some(0x0c8a_636f_0392_04b8),
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G16X: Family = Family {
    name: "G16X",
    kext: "com.apple.AGXG16X",
    kernelcache: "mac16j",
    generations: &[8, 0xa],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: None,
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G17G: Family = Family {
    name: "G17G",
    kext: "com.apple.AGXG17G",
    kernelcache: "mac17g",
    generations: &[0xb],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: None,
    dashboard_signals: true,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G17X: Family = Family {
    name: "G17X",
    kext: "com.apple.AGXG17X",
    kernelcache: "mac17j",
    generations: &[0xb],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: None,
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G17P: Family = Family {
    name: "G17P",
    kext: "com.apple.AGXG17P",
    kernelcache: "mac17p",
    generations: &[8, 0xa],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: None,
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};
pub const G18G: Family = Family {
    name: "G18G",
    kext: "com.apple.AGXG18G",
    kernelcache: "mac18g",
    generations: &[0xc],
    core_masks: &G14X_CORE_MASKS,
    mmu: g14_mmu(28),
    initdata_magic: None,
    dashboard_signals: false,
    interface: Interface::Unrecovered(NOT_CHECKED),
};

/// T8112: G14G B0, one cluster of 10 cores, 10 fragment units and 4 GPs
/// (Public: Asahi `hw/t8112.rs` for the counts, `regs.rs` for variant G =
/// 2; revision B0 is m1n1's `gpu_rev`, read before m1n1's AI notice).
pub const T8112: Identity = Identity {
    generation: 5,
    variant: 2,
    revision: 0x10,
    dies: 1,
    clusters: 1,
    cores: 10,
    frags: 10,
    gps: 4,
    core_mask: [0x3ff, 0, 0],
};

/// The M1 GPUs (G13, generation 4). Counts are per cluster, as
/// `AGXAcceleratorG13::readChipInfo` reads them (it multiplies each by the
/// cluster count in 0xd0401c bits 19:12), and Public: Asahi `hw/t8103.rs`
/// and `hw/t600x.rs` (8 cores, 8 fragment units and 4 GPs per cluster;
/// 1, 2, 4 and 8 clusters) and `regs.rs` (variant G = 2, S = 3, C and D =
/// 4). The kext reads one core-mask word for variant G and both for S, C
/// and D, and on those accepts 2, 4 or 8 clusters (16, 32 or 64 cores).
/// Revision: Inferred. No public source states it; the kext accepts A0, B0,
/// B1 and C0 (bits 15:12 and 11:8), and each SoC's newest GFX firmware image
/// is the one a shipping part runs (`armfw_g13g` b100, `armfw_g13x` c000).
/// T6002 spans two dies, which these identification words do not count.
const fn g13(variant: u8, revision: u8, dies: u8, clusters: u8, core_mask: [u32; 3]) -> Identity {
    Identity {
        generation: 4,
        variant,
        revision,
        dies,
        clusters,
        cores: 8,
        frags: 8,
        gps: 4,
        core_mask,
    }
}
pub const T8103: Identity = g13(2, 0x11, 1, 1, [0xff, 0, 0]);
pub const T6000: Identity = g13(3, 0x20, 1, 2, [0xffff, 0, 0]);
pub const T6001: Identity = g13(4, 0x20, 1, 4, [u32::MAX, 0, 0]);
pub const T6002: Identity = g13(4, 0x20, 2, 8, [u32::MAX, u32::MAX, 0]);

/// The M2 Pro, Max and Ultra GPUs (G14X, generation 6). Public: Asahi
/// `hw/t602x.rs` (10 cores, 10 fragment units and 4 GPs per cluster; 2, 4
/// and 8 clusters over 1, 1 and 2 dies) and `regs.rs` (variant S = 3, C and
/// D = 4). `AGXAcceleratorG14X::readChipInfo` takes variant 3 as a 20-core
/// part, variant 4 on one die as 40 cores and on more as 80, multiplies the
/// per-cluster counts by the clusters, and reads three core-mask words,
/// packed 10 bits per cluster. Revision: Inferred, B0: the kext accepts A0,
/// B0 and B1, and each SoC's newest GFX firmware image is b000.
const fn g14x(variant: u8, dies: u8, clusters: u8, core_mask: [u32; 3]) -> Identity {
    Identity {
        generation: 6,
        variant,
        revision: 0x10,
        dies,
        clusters,
        cores: 10,
        frags: 10,
        gps: 4,
        core_mask,
    }
}
pub const T6020: Identity = g14x(3, 1, 2, [0xf_ffff, 0, 0]);
pub const T6021: Identity = g14x(4, 1, 4, [u32::MAX, 0xff, 0]);
pub const T6022: Identity = g14x(4, 2, 8, [u32::MAX, u32::MAX, 0xffff]);

const fn gpu(
    compatible: &'static str,
    chip_id: u16,
    family: &'static Family,
    firmware: &'static str,
) -> Gpu {
    Gpu {
        compatible,
        chip_id,
        family,
        firmware,
        identity: None,
    }
}

const fn known(
    compatible: &'static str,
    chip_id: u16,
    family: &'static Family,
    firmware: &'static str,
    identity: Identity,
) -> Gpu {
    Gpu {
        identity: Some(identity),
        ..gpu(compatible, chip_id, family, firmware)
    }
}

/// Every `sgx` of the 26A434 board DeviceTrees, by ApChipID.
pub const GPUS: [Gpu; 20] = [
    known("gpu,t8103", 0x8103, &G13G, "armfw_g13g", T8103),
    // M1 Pro, Max and Ultra boards all name `gpu,t6000`; the chip is the
    // board's `/chosen` `chip-id`.
    known("gpu,t6000", 0x6000, &G13X, "armfw_g13x", T6000),
    known("gpu,t6000", 0x6001, &G13X, "armfw_g13x", T6001),
    known("gpu,t6000", 0x6002, &G13X, "armfw_g13x", T6002),
    known("gpu,t8112", 0x8112, &G14G, "armfw_g14g", T8112),
    known("gpu,t6020", 0x6020, &G14X, "armfw_g14s", T6020),
    known("gpu,t6021", 0x6021, &G14X, "armfw_g14c", T6021),
    known("gpu,t6022", 0x6022, &G14X, "armfw_g14d", T6022),
    gpu("gpu,t8122", 0x8122, &G15G, "armfw_g15g"),
    gpu("gpu,t6030", 0x6030, &G15S, "armfw_g15s"),
    gpu("gpu,t6031", 0x6031, &G15C, "armfw_g15c"),
    gpu("gpu,t6032", 0x6032, &G15C, "armfw_g15d"),
    gpu("gpu,t6034", 0x6034, &G15C, "armfw_g15c"),
    gpu("gpu,t8132", 0x8132, &G16G, "armfw_g16g"),
    gpu("gpu,t6040", 0x6040, &G16X, "armfw_g16s"),
    gpu("gpu,t6041", 0x6041, &G16X, "armfw_g16c"),
    gpu("gpu,t8140", 0x8140, &G17P, "armfw_g17p"),
    gpu("gpu,t8142", 0x8142, &G17G, "armfw_g17g"),
    gpu("gpu,t6050", 0x6050, &G17X, "armfw_g17x"),
    gpu("gpu,t8152", 0x8152, &G18G, "armfw_g18g"),
];

/// The GPU of the SoC with ApChipID `chip_id`.
pub fn by_chip(chip_id: u32) -> Option<&'static Gpu> {
    GPUS.iter().find(|gpu| u32::from(gpu.chip_id) == chip_id)
}

/// Whether every SoC whose `sgx` carries `compatible` is admitted, so the
/// node can stay visible on any board that names it (several SoCs share
/// one; the chip is then the board's `chip-id`).
pub fn compatible_admitted(compatible: &str) -> bool {
    let mut gpus = GPUS.iter().filter(|gpu| gpu.compatible == compatible);
    gpus.clone().next().is_some() && gpus.all(|gpu| gpu.admitted().is_ok())
}

impl Family {
    /// What the models need of this family, when its interface is modeled;
    /// otherwise why it is not.
    pub fn model(&self) -> Result<&'static Model> {
        match self.interface {
            Interface::Modeled(model) => Ok(model),
            Interface::Unrecovered(reason) => Err(reason.into()),
        }
    }
}

impl Gpu {
    /// The identification the model reports, when the GPU's whole interface
    /// is modeled; otherwise why it is not.
    pub fn admitted(&self) -> Result<Identity> {
        self.family.model()?;
        let identity = self.identity.ok_or(refuse!(
            "no source states the GPU's identification registers"
        ))?;
        if !self.family.generations.contains(&identity.generation) {
            return Err(refuse!("the GPU's generation is not one its kext accepts"));
        }
        Ok(identity)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// Setup must never install by default a build whose GFX interface this
    /// model refuses: the guest would stall at the installer's "load sep os".
    #[test]
    fn unchecked_initdata_builds_are_never_the_setup_default() {
        use stela_formats::builds;
        for &(build, ..) in UNCHECKED_INITDATA {
            assert!(builds::GPU_UNCHECKED.contains(&build), "{build}");
            assert_ne!(builds::DEFAULT.build, build);
        }
        for build in builds::GPU_UNCHECKED {
            assert!(
                UNCHECKED_INITDATA.iter().any(|&(known, ..)| known == *build),
                "{build}"
            );
        }
    }

    #[test]
    fn an_unchecked_initdata_revision_is_no_modeled_familys_magic() {
        for &(build, family, magic) in UNCHECKED_INITDATA {
            assert!(
                GPUS.iter()
                    .all(|gpu| gpu.family.initdata_magic != Some(magic))
            );
            assert!(GPUS.iter().any(|gpu| gpu.family.name == family));
            assert_eq!(unchecked_initdata(magic), Some((build, family)));
        }
        assert_eq!(unchecked_initdata(MAGIC_G13_G14G), None);
    }

    #[test]
    fn every_census_gpu_is_named_once() {
        for (index, gpu) in GPUS.iter().enumerate() {
            assert!(gpu.compatible.starts_with("gpu,t"));
            assert!(
                GPUS[..index]
                    .iter()
                    .all(|other| other.chip_id != gpu.chip_id)
            );
            assert_eq!(by_chip(gpu.chip_id.into()), Some(gpu));
            assert!(gpu.firmware.starts_with("armfw_g"));
            assert!(gpu.family.kext.ends_with(gpu.family.name));
        }
    }

    #[test]
    fn only_m1_and_m2_gpus_are_admitted() {
        let admitted: alloc::vec::Vec<_> = GPUS
            .iter()
            .filter(|gpu| gpu.admitted().is_ok())
            .map(|gpu| gpu.chip_id)
            .collect();
        assert_eq!(
            admitted,
            [
                0x8103, 0x6000, 0x6001, 0x6002, 0x8112, 0x6020, 0x6021, 0x6022
            ]
        );
        assert_eq!(by_chip(0x8112).unwrap().admitted(), Ok(T8112));
        assert_eq!(by_chip(0x6022).unwrap().admitted(), Ok(T6022));
        // G15G's interface is modeled; no source states an M3's
        // identification words, so its GPU stays hidden.
        assert_eq!(G15G.model(), Ok(&G15G_MODEL));
        assert_eq!(
            by_chip(0x8122).unwrap().admitted(),
            Err(refuse!(
                "no source states the GPU's identification registers"
            ))
        );
        assert_eq!(G15S.model(), Err(NOT_CHECKED.into()));
        assert!(by_chip(0x9999).is_none());
        // M1 Pro, Max and Ultra share one compatible, admitted as a whole.
        assert!(compatible_admitted("gpu,t6000"));
        assert!(compatible_admitted("gpu,t8112"));
        assert!(compatible_admitted("gpu,t6020"));
        assert!(!compatible_admitted("gpu,t8122"));
        assert!(!compatible_admitted("gpu,t9999"));
    }

    #[test]
    fn only_g14g_and_g17g_send_dashboard_signals() {
        // Only the G14G and G17G kexts send dashboard signals.
        let dashboards: alloc::vec::Vec<_> = GPUS
            .iter()
            .filter(|gpu| gpu.family.dashboard_signals)
            .map(|gpu| gpu.family.name)
            .collect();
        assert_eq!(dashboards, ["G14G", "G17G"]);
    }

    #[test]
    fn g14x_identities_are_ones_its_kext_accepts() {
        for gpu in GPUS.iter().filter(|gpu| gpu.family.name == "G14X") {
            let id = gpu.admitted().unwrap();
            let model = gpu.family.model().unwrap();
            assert_eq!(model.identification, Identification::Dies);
            assert_ne!(model.layout, G14G_MODEL.layout);
            // `readChipInfo`: variants 3 and 4; revisions A0, B0 and B1;
            // the variant's core capacity holds every core.
            assert!([0x00, 0x10, 0x11].contains(&id.revision));
            let capacity = match (id.variant, id.dies) {
                (3, _) => 20,
                (4, 1) => 40,
                (4, _) => 80,
                _ => panic!("variant the kext does not size"),
            };
            let cores = u32::from(id.cores) * u32::from(id.clusters);
            assert!(cores <= capacity);
            assert_eq!(id.clusters % id.dies, 0);
            let enabled: u32 = id.core_mask.iter().map(|word| word.count_ones()).sum();
            assert_eq!(enabled, cores);
            assert!(!gpu.family.dashboard_signals);
        }
    }

    #[test]
    fn g13_identities_are_ones_its_kext_accepts() {
        for gpu in GPUS.iter().filter(|gpu| gpu.family.generations == [4]) {
            let id = gpu.admitted().unwrap();
            // `readChipInfo`: variants 0, 2, 3, 4; revisions A0, B0, B1, C0.
            assert!([2, 3, 4].contains(&id.variant));
            assert!([0x00, 0x10, 0x11, 0x20].contains(&id.revision));
            let cores = u32::from(id.cores) * u32::from(id.clusters);
            let enabled: u32 = id.core_mask.iter().map(|word| word.count_ones()).sum();
            assert_eq!(enabled, cores);
            if id.variant == 2 {
                assert_eq!(id.core_mask[1..], [0, 0]);
            } else {
                assert!([2, 4, 8].contains(&id.clusters));
            }
            assert!(!gpu.family.dashboard_signals);
        }
    }

    #[test]
    fn mmu_layouts_follow_the_two_secure_monitor_generations() {
        for gpu in &GPUS {
            let mmu = gpu.family.mmu;
            let g13 = gpu.family.generations == [4];
            assert_eq!(mmu.ttbat, if g13 { 0xd0_8024 } else { 0xd0_802c });
            assert_eq!(mmu.invalidate, mmu.ttbat + 4);
            assert!(mmu.configuration.iter().all(|&(offset, _)| {
                (MMU_ENABLE + 4..mmu.ttbat).contains(&offset) && offset % 4 == 0
            }));
            let narrow = ["G13G", "G14G"].contains(&gpu.family.name);
            assert_eq!(mmu.ttbat_page_bits, if narrow { 26 } else { 28 });
        }
    }
}
