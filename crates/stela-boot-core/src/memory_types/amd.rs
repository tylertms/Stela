//! AMD APM 24593 rev3.44 §§3.2.1,7.9: routing controls beyond common MTRRs.
//! Common AMD64 controls are selected by vendor/capability observations;
//! family names do not grant additional model-specific or encryption permission.
use crate::Result;
use crate::topology::Snapshot;
use stela_contracts::refuse;

pub const SYSCFG: u32 = 0xc001_0010;
pub const IORR_BASE: u32 = 0xc001_0016;
pub const TOP_MEM: u32 = 0xc001_001a;
pub const TOP_MEM2: u32 = 0xc001_001d;
pub const FIXED_DRAM: u64 = 1 << 18;
pub const FIXED_MODIFY: u64 = 1 << 19;
pub const VARIABLE_DRAM: u64 = 1 << 20;
pub const TOM2_ENABLE: u64 = 1 << 21;
pub const TOM2_WB: u64 = 1 << 22;
/// MemEncryptionModeEn, the firmware's "SME" option: page-table entries
/// may then select encryption with the C-bit (APM Vol. 2 7.10.1). Memory
/// they leave it clear in, as Stela's do, stays plain, and DMA is
/// unaffected. Transparent SME (TSME) is invisible here and to software.
pub const MEMORY_ENCRYPTION: u64 = 1 << 23;
/// SecureNestedPagingEn: every access is checked against the reverse map
/// table, whose owner Stela is not (APM Vol. 2 15.36.3).
pub const SECURE_NESTED_PAGING: u64 = 1 << 24;
/// VMPLEn: privilege levels inside SNP guests; no effect on the host.
pub const VMPL: u64 = 1 << 25;
/// HostMultiKeyMemEncrModeEn: physical-address bits select host keys.
pub const HOST_MULTI_KEY: u64 = 1 << 26;
/// RdMem and WrMem of each fixed-range byte: whether reads and writes of
/// the range go to DRAM (APM Vol. 2 7.9.1).
pub const FIXED_ROUTING: u64 = 0x1818_1818_1818_1818;

/// Whether fixed-range register `index` covers only the legacy hole,
/// 0xa0000 to 0xfffff: video memory and the option ROM and firmware shadow,
/// which no memory map gives as RAM. Firmware may route it differently on
/// each processor (an Insyde laptop write-protects the firmware shadow on
/// the BSP alone), and Linux leaves each processor's routing there as it
/// finds it. The registers below it (0x250, 0x258) cover conventional memory.
pub const fn legacy_hole(index: u32) -> bool {
    matches!(index, 0x259 | 0x268..=0x26f)
}

pub const REGISTERS: [u32; 7] = [
    SYSCFG,
    IORR_BASE,
    IORR_BASE + 1,
    IORR_BASE + 2,
    IORR_BASE + 3,
    TOP_MEM,
    TOP_MEM2,
];
pub const VENDOR: [u32; 3] = [
    u32::from_le_bytes(*b"Auth"),
    u32::from_le_bytes(*b"enti"),
    u32::from_le_bytes(*b"cAMD"),
];

/// Why memory encryption as `syscfg` sets it stops a boot: only the
/// controls Stela cannot run under, each with the firmware setting to
/// change. SME alone and VMPL alone are not among them.
pub fn encryption_refusal(syscfg: u64) -> Option<&'static str> {
    if syscfg & SECURE_NESTED_PAGING != 0 {
        Some(
            "SEV-SNP is on in the firmware setup, and Stela does not own its reverse map table: \
             turn off SEV-SNP (sometimes called SNP Memory Coverage) in the firmware setup",
        )
    } else if syscfg & HOST_MULTI_KEY != 0 {
        Some(
            "AMD host multi-key memory encryption is on in the firmware setup, which Stela does \
             not support: turn it off in the firmware setup",
        )
    } else {
        None
    }
}

/// The physical address width left to memory: with SME enabled, CPUID
/// Fn8000_001F EBX[11:6] bits fewer than `maximum`, as Linux's
/// `early_detect_mem_encrypt` reduces `x86_phys_bits`, so that no address
/// reaches the C-bit (EBX[5:0]). `encryption` is that leaf's EBX where the
/// CPU enumerates SME (EAX bit 0).
pub fn usable_physical_bits(maximum: u8, syscfg: u64, encryption: Option<u32>) -> u8 {
    match encryption {
        Some(ebx) if syscfg & MEMORY_ENCRYPTION != 0 => {
            let reduction = ((ebx >> 6) & 0x3f) as u8;
            let c_bit = (ebx & 0x3f) as u8;
            maximum.saturating_sub(reduction).min(c_bit.max(32))
        }
        _ => maximum,
    }
}

/// The routing registers `syscfg` enables (APM Vol. 2 7.9): MtrrVarDramEn
/// the IORRs and TOP_MEM, MtrrTom2En TOP_MEM2. Disabled registers have no
/// effect and are never read, as a hypervisor need not implement them (KVM
/// raises #GP for the IORRs).
pub fn enabled(syscfg: u64) -> impl Iterator<Item = u32> {
    REGISTERS.into_iter().skip(1).filter(move |&index| {
        syscfg
            & if index == TOP_MEM2 {
                TOM2_ENABLE
            } else {
                VARIABLE_DRAM
            }
            != 0
    })
}

pub fn architectural(snapshot: &Snapshot) -> bool {
    let Some(vendor) = snapshot.get(0, 0) else {
        return false;
    };
    if [vendor.ebx, vendor.edx, vendor.ecx] != VENDOR {
        return false;
    }
    let Some(leaf) = snapshot.get(1, 0) else {
        return false;
    };
    // AMD APM common AMD64 system MSRs; model-specific fields remain opaque.
    // Fixed registers are additionally enumerated through MTRRcap.FIX.
    const SYSTEM: u32 = (1 << 5) | (1 << 12) | (1 << 16); // MSR/MTRR/PAT
    leaf.edx & SYSTEM == SYSTEM
        && snapshot
            .get(0x8000_0001, 0)
            .is_some_and(|extended| extended.edx & (1 << 29) != 0)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct State {
    pub syscfg: u64,
    pub iorr: [(u64, u64); 2],
    pub top_mem: u64,
    pub top_mem2: u64,
}

impl State {
    pub fn from_snapshot(snapshot: &Snapshot, physical_bits: u8) -> Result<Self> {
        if !architectural(snapshot) || !snapshot.amd_fixed_raw {
            return Err(refuse!(
                "AMD64 controls or full fixed-DRAM observation is unqualified"
            ));
        }
        let syscfg = super::register(snapshot, SYSCFG)?;
        // A register SYSCFG disables reads as zero, which routes nothing.
        let register = |index| {
            if enabled(syscfg).any(|enabled| enabled == index) {
                super::register(snapshot, index)
            } else {
                Ok(0)
            }
        };
        let state = Self {
            syscfg,
            iorr: [
                (register(IORR_BASE)?, register(IORR_BASE + 1)?),
                (register(IORR_BASE + 2)?, register(IORR_BASE + 3)?),
            ],
            top_mem: register(TOP_MEM)?,
            top_mem2: register(TOP_MEM2)?,
        };
        if let Some(reason) = encryption_refusal(state.syscfg) {
            return Err(refuse!(reason));
        }
        let address = ((1u64 << physical_bits) - 1) & !0xfff;
        if state.top_mem & !(address & !((1 << 23) - 1)) != 0
            || state.top_mem2 & !(address & !((1 << 23) - 1)) != 0
            || state.top_mem > 1 << 32
        {
            return Err(refuse!("unsupported AMD top-of-memory geometry"));
        }
        for (base, mask) in state.iorr {
            if base & !(address | 0x18) != 0 || mask & !(address | (1 << 11)) != 0 {
                return Err(refuse!("unsupported AMD IORR fields"));
            }
        }
        if state.iorr.iter().all(|(_, mask)| mask & (1 << 11) != 0) {
            // APM overlapping IORRs are implementation dependent. Refuse any
            // overlap rather than choose undocumented precedence.
            let [(left, lm), (right, rm)] = state.iorr;
            if (left ^ right) & lm & rm & address == 0 {
                return Err(refuse!("overlapping AMD IORRs are unqualified"));
            }
        }
        Ok(state)
    }

    /// Whether the processor implements the DRAM routing registers. Firmware
    /// on AMD hardware always enables variable DRAM routing and sets TOP_MEM;
    /// a hypervisor without the registers (QEMU's TCG) reads every routing
    /// field as zero. Without them RAM is routed as the firmware's memory map
    /// classifies it, and only the MTRR types are checked.
    pub fn routes(&self) -> bool {
        self.syscfg & (FIXED_DRAM | VARIABLE_DRAM | TOM2_ENABLE | TOM2_WB) != 0
            || self.top_mem != 0
            || self.top_mem2 != 0
            || self.iorr.iter().any(|&(_, mask)| mask & (1 << 11) != 0)
    }

    /// Per-processor opaque non-memory fields are held, never interpreted
    /// as feature permission or overwritten from a different processor.
    pub fn compatible(self, other: Self) -> Result<()> {
        if (self.syscfg ^ other.syscfg) & !FIXED_MODIFY != 0
            || self.iorr != other.iorr
            || self.top_mem != other.top_mem
            || self.top_mem2 != other.top_mem2
        {
            return Err(refuse!(
                "AMD initial routing or opaque controls differ across selected cores"
            ));
        }
        Ok(())
    }
}

/// APM Table7-13. Only the defined type/routing combinations are accepted.
pub fn fixed_type(value: u8, cap: u64) -> bool {
    let kind = value & 7;
    let route = value & 0x18;
    value & !0x1f == 0
        && super::memory_type(kind, cap)
        && match route {
            0 => kind != 6,
            8 => kind == 0,
            16 => matches!(kind, 0 | 5),
            24 => kind != 5,
            _ => false,
        }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::topology::{Cpuid, Msr};
    const SIGNATURE: u32 = 0x00b4_0f40;
    fn snapshot() -> Snapshot {
        let mut snapshot = Snapshot::default();
        snapshot.records[0] = Cpuid {
            leaf: 0,
            ebx: VENDOR[0],
            edx: VENDOR[1],
            ecx: VENDOR[2],
            ..Cpuid::default()
        };
        snapshot.records[1] = Cpuid {
            leaf: 1,
            eax: SIGNATURE,
            edx: (1 << 5) | (1 << 12) | (1 << 16),
            ..Cpuid::default()
        };
        snapshot.records[2] = Cpuid {
            leaf: 0x8000_0001,
            edx: 1 << 29,
            ..Cpuid::default()
        };
        snapshot.count = 3;
        snapshot.amd_fixed_raw = true;
        for (slot, index) in REGISTERS.into_iter().enumerate() {
            snapshot.msrs[slot] = Msr { index, value: 0 };
        }
        snapshot.msr_count = REGISTERS.len() as u32;
        snapshot
    }
    #[test]
    fn amd64_controls_hidden_attributes_and_encryption_are_required() {
        let source = snapshot();
        assert!(architectural(&source));
        assert!(State::from_snapshot(&source, 52).is_ok());
        let mut changed = source;
        // Other AMD families/models/steppings use these same documented
        // architectural fields; marketing names cannot grant/deny permission.
        for signature in [
            0x0010_0f20,
            0x0060_0f20,
            0x0080_0f10,
            0x00a2_0f10,
            SIGNATURE + 1,
        ] {
            changed.records[1].eax = signature;
            assert!(architectural(&changed));
            assert!(State::from_snapshot(&changed, 48).is_ok());
        }
        for bit in [5, 12, 16] {
            changed = source;
            changed.records[1].edx &= !(1 << bit);
            assert!(!architectural(&changed));
        }
        changed = source;
        changed.records[2].edx = 0;
        assert!(!architectural(&changed));
        changed = source;
        changed.records[0].ebx ^= 1;
        assert!(!architectural(&changed));
        changed = source;
        changed.amd_fixed_raw = false;
        assert!(State::from_snapshot(&changed, 52).is_err());
        // Only SYSCFG is observed while it enables no routing register; an
        // enabled register must have been observed.
        changed = source;
        changed.msr_count = 1;
        assert_eq!(enabled(0).count(), 0);
        let state = State::from_snapshot(&changed, 52).unwrap();
        assert!(!state.routes());
        for enable in [VARIABLE_DRAM, TOM2_ENABLE] {
            changed.msrs[0].value = enable;
            assert!(State::from_snapshot(&changed, 52).is_err());
        }
        assert!(enabled(VARIABLE_DRAM).eq([
            IORR_BASE,
            IORR_BASE + 1,
            IORR_BASE + 2,
            IORR_BASE + 3,
            TOP_MEM
        ]));
        assert!(enabled(TOM2_ENABLE).eq([TOP_MEM2]));
        for (bit, refused) in [(23, false), (24, true), (25, false), (26, true)] {
            changed = source;
            changed.msrs[0].value = 1 << bit;
            assert_eq!(
                State::from_snapshot(&changed, 52).is_err(),
                refused,
                "bit {bit}"
            );
        }
    }
    #[test]
    fn sme_alone_boots_and_snp_says_which_setting_to_change() {
        assert_eq!(encryption_refusal(0), None);
        assert_eq!(encryption_refusal(MEMORY_ENCRYPTION), None);
        assert_eq!(encryption_refusal(MEMORY_ENCRYPTION | VMPL), None);
        let snp = encryption_refusal(MEMORY_ENCRYPTION | SECURE_NESTED_PAGING).unwrap();
        assert!(snp.contains("turn off SEV-SNP"), "{snp}");
        assert!(encryption_refusal(HOST_MULTI_KEY).is_some());
    }
    #[test]
    fn sme_reduces_the_physical_address_width_below_the_c_bit() {
        // A Zen 4 CPU: C-bit 51, 6 bits fewer with SME on.
        let ebx = 51 | 6 << 6;
        assert_eq!(usable_physical_bits(52, 0, Some(ebx)), 52);
        assert_eq!(usable_physical_bits(52, MEMORY_ENCRYPTION, None), 52);
        assert_eq!(usable_physical_bits(52, MEMORY_ENCRYPTION, Some(ebx)), 46);
        // A Zen 2 CPU: C-bit 47 inside a 48-bit space, 5 bits fewer.
        assert_eq!(
            usable_physical_bits(48, MEMORY_ENCRYPTION, Some(47 | 5 << 6)),
            43
        );
        // The C-bit bounds the width even where no reduction is reported.
        assert_eq!(usable_physical_bits(48, MEMORY_ENCRYPTION, Some(47)), 47);
    }
    #[test]
    fn geometry_overlap_and_preserved_routing_controls_refuse_before_installation() {
        let source = snapshot();
        let baseline = State::from_snapshot(&source, 48).unwrap();
        let mut changed = baseline;
        changed.syscfg ^= FIXED_MODIFY;
        assert_eq!(baseline.compatible(changed), Ok(()));
        for bit in [0, 18, 20, 21, 22, 32] {
            changed = baseline;
            changed.syscfg ^= 1 << bit;
            assert!(baseline.compatible(changed).is_err());
        }
        changed = baseline;
        changed.top_mem = 8 << 23;
        assert!(baseline.compatible(changed).is_err());
        for (slot, value) in [(5, 1), (5, (1 << 32) + (1 << 23)), (6, 1 << 48), (1, 0x20)] {
            let mut changed = source;
            changed.msrs[0].value = VARIABLE_DRAM | TOM2_ENABLE;
            assert!(State::from_snapshot(&changed, 48).is_ok());
            changed.msrs[slot].value = value;
            assert!(State::from_snapshot(&changed, 48).is_err());
        }
        let mut changed = source;
        changed.msrs[0].value = VARIABLE_DRAM;
        changed.msrs[2].value = 0xffff_ffff_f800;
        changed.msrs[4].value = 0xffff_ffff_f800;
        assert!(State::from_snapshot(&changed, 48).is_err());
    }
    #[test]
    fn routing_types_follow_defined_combinations_and_reject_reserved_fields() {
        for route in [0, 8, 16, 24] {
            for kind in [0, 1, 4, 5, 6] {
                let expected = match route {
                    0 => kind != 6,
                    8 => kind == 0,
                    16 => matches!(kind, 0 | 5),
                    24 => kind != 5,
                    _ => unreachable!(),
                };
                assert_eq!(fixed_type(route | kind, 1 << 10), expected);
                assert!(!fixed_type(route | kind | 0x80, 1 << 10));
            }
        }
        assert!(!fixed_type(0x19, 0));
    }
}
