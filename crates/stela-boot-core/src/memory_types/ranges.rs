//! Read-only physical cache/routing query over the captured installation image.
//! A caller must separately prove WB PAT mappings, enabled caches and
//! exclusion of incompatible aliases; this image does not grant RAM ownership.
use super::{Image, VARIABLE_BASE, amd};
use crate::Result;
use stela_contracts::refuse;

/// Whether variable range `(base, mask)` matches `cursor`, and the next
/// address above it where that may change. An address matches when its
/// masked bits equal the base's (Intel SDM Vol. 3A 12.11.2.3; AMD APM
/// Vol. 2 7.7.1), so a base not aligned to the range's size starts at
/// `base & mask`, and a mask with holes matches a set of aligned blocks the
/// size of its lowest set bit. Linux runs on both shapes.
pub fn variable(base: u64, mask: u64, bits: u8, cursor: u64) -> (bool, u64) {
    let address = ((1u64 << bits) - 1) & !4095;
    let mask = mask & address;
    let matches = (cursor ^ base) & mask == 0;
    let size = (!mask & address) + 4096;
    if size.is_power_of_two() {
        let start = base & mask;
        let end = start.saturating_add(size);
        return match (cursor < start, matches) {
            (true, _) => (false, start),
            (false, true) => (true, end),
            (false, false) => (false, u64::MAX),
        };
    }
    // Bits below the lowest set one never take part, so the answer holds
    // to the end of the block of that size holding `cursor`.
    let block = 1u64 << mask.trailing_zeros().max(12);
    (matches, (cursor | (block - 1)).saturating_add(1))
}

impl Image {
    fn value(&self, index: u32) -> Result<u64> {
        let value = self.writes[..self.count]
            .iter()
            .find(|write| write.index == index)
            .ok_or(refuse!("incomplete installed memory-type image"))?;
        Ok(u64::from(value.low) | u64::from(value.high) << 32)
    }

    /// Proves WB MTRR type and, on AMD with routing registers
    /// ([`amd::State::routes`]), both read/write DRAM routing for every byte.
    /// Undefined overlaps and partial coverage refuse. No page/PAT or
    /// live-current-control assertion is implied by a captured image query.
    pub fn writeback_ram(&self, start: u64, bytes: u64) -> Result<()> {
        let end = start
            .checked_add(bytes)
            .ok_or(refuse!("native RAM range overflow"))?;
        if bytes == 0 || end > 1u64 << self.physical_bits {
            return Err(refuse!(
                "native RAM range exceeds physical memory-type geometry"
            ));
        }
        let routing = self.amd.filter(amd::State::routes);
        let mut cursor = start;
        while cursor < end {
            let mut next = end;
            let fixed = cursor < 1 << 20 && self.default & (1 << 10) != 0;
            let mut matched = false;
            let mut kind = self.default as u8;
            if fixed {
                let (index, width, base) = if cursor < 0x80000 {
                    (0x250, 0x10000, 0)
                } else if cursor < 0xc0000 {
                    let index = ((cursor - 0x80000) / 0x20000) as u32;
                    (0x258 + index, 0x4000, 0x80000 + u64::from(index) * 0x20000)
                } else {
                    let index = ((cursor - 0xc0000) / 0x8000) as u32;
                    (0x268 + index, 0x1000, 0xc0000 + u64::from(index) * 0x8000)
                };
                let offset = (cursor - base) / width;
                kind = (self.value(index)? >> (8 * offset)) as u8;
                next = next.min(base + (offset + 1) * width);
                if let Some(state) = routing
                    && (state.syscfg & amd::FIXED_DRAM == 0 || kind & 0x18 != 0x18)
                {
                    return Err(refuse!("AMD fixed range is not read/write DRAM"));
                }
                kind &= 7;
            } else {
                if cursor < 1 << 20 {
                    next = next.min(1 << 20);
                    if routing.is_some() {
                        return Err(refuse!("AMD low RAM without fixed routing is unqualified"));
                    }
                }
                for index in 0..u32::from(self.cap as u8) {
                    let base = self.value(VARIABLE_BASE + 2 * index)?;
                    let mask = self.value(VARIABLE_BASE + 2 * index + 1)?;
                    if mask & (1 << 11) == 0 {
                        continue;
                    }
                    let (inside, change) = variable(base, mask, self.physical_bits, cursor);
                    next = next.min(change);
                    if inside {
                        // Every matched type must be WB. UC/WT precedence and
                        // undefined mixed overlaps therefore cannot admit RAM.
                        if base as u8 != 6 {
                            return Err(refuse!("native RAM intersects a non-WB MTRR"));
                        }
                        kind = 6;
                        matched = true;
                    }
                }
                if let Some(state) = routing {
                    // Require TOM and any matching IORR to agree on DRAM.
                    // Do not infer a permissive precedence for conflicting
                    // routing descriptions from the common architectural text.
                    let dram = (cursor < 1 << 32
                        && state.syscfg & amd::VARIABLE_DRAM != 0
                        && cursor < state.top_mem)
                        || (cursor >= 1 << 32
                            && state.syscfg & amd::TOM2_ENABLE != 0
                            && cursor < state.top_mem2);
                    for boundary in [1 << 32, state.top_mem, state.top_mem2] {
                        if boundary > cursor {
                            next = next.min(boundary);
                        }
                    }
                    if state.syscfg & amd::VARIABLE_DRAM != 0 {
                        for (base, mask) in state.iorr {
                            if mask & (1 << 11) == 0 {
                                continue;
                            }
                            let (inside, change) = variable(base, mask, self.physical_bits, cursor);
                            next = next.min(change);
                            if inside && base & 0x18 != 0x18 {
                                return Err(refuse!(
                                    "AMD IORR does not route both directions to DRAM"
                                ));
                            }
                        }
                    }
                    if !dram {
                        return Err(refuse!("AMD native RAM is not routed to read/write DRAM"));
                    }
                    if !matched
                        && state.syscfg & (amd::TOM2_WB | amd::TOM2_ENABLE)
                            == (amd::TOM2_WB | amd::TOM2_ENABLE)
                        && cursor >= 1 << 32
                        && cursor < state.top_mem2
                    {
                        kind = 6;
                    }
                }
            }
            if kind != 6 {
                return Err(refuse!("native RAM lacks full WB memory-type coverage"));
            }
            cursor = next;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::super::{FIXED, MAX_WRITABLE, Write};
    use super::*;

    fn image() -> Image {
        let mut image = Image {
            cap: (1 << 8) | 1,
            physical_bits: 48,
            default: 0xc06,
            writes: [Write::default(); MAX_WRITABLE],
            count: 0,
            amd: Some(amd::State {
                syscfg: amd::VARIABLE_DRAM | amd::TOM2_ENABLE | amd::FIXED_DRAM,
                iorr: [(0, 0); 2],
                top_mem: 3 << 30,
                top_mem2: 8 << 30,
            }),
        };
        for index in FIXED.into_iter().chain(VARIABLE_BASE..VARIABLE_BASE + 2) {
            set(
                &mut image,
                index,
                if FIXED.contains(&index) {
                    0x1e1e_1e1e_1e1e_1e1e
                } else {
                    0
                },
            );
        }
        image
    }
    fn set(image: &mut Image, index: u32, value: u64) {
        let slot = image.writes[..image.count]
            .iter()
            .position(|write| write.index == index)
            .unwrap_or_else(|| {
                let slot = image.count;
                image.count += 1;
                slot
            });
        image.writes[slot] = Write {
            index,
            low: value as u32,
            high: (value >> 32) as u32,
        };
    }
    fn span(image: &mut Image, base: u64, bytes: u64, kind: u64) {
        set(image, VARIABLE_BASE, base | kind);
        set(
            image,
            VARIABLE_BASE + 1,
            (((1u64 << image.physical_bits) - 1) & !(bytes - 1)) | (1 << 11),
        );
    }
    #[test]
    fn top_of_memory_boundaries_and_partial_dram_coverage_are_exact() {
        let image = image();
        for (start, bytes, expected) in [
            (1 << 20, 1, true),
            (3 << 30, 1, false),
            ((3 << 30) - 1, 2, false),
            (4 << 30, 4 << 30, true),
            (8 << 30, 1, false),
            ((8 << 30) - 1, 2, false),
            (0, 0, false),
            (u64::MAX, 2, false),
            (1 << 48, 1, false),
        ] {
            assert_eq!(
                image.writeback_ram(start, bytes).is_ok(),
                expected,
                "{start:x}/{bytes:x}"
            );
        }
        let mut changed = image;
        changed.amd.as_mut().unwrap().syscfg &= !amd::TOM2_ENABLE;
        assert!(changed.writeback_ram(4 << 30, 4096).is_err());
        changed.amd.as_mut().unwrap().syscfg &= !amd::VARIABLE_DRAM;
        assert!(changed.writeback_ram(1 << 20, 4096).is_err());
    }
    #[test]
    fn forced_high_wb_is_only_a_default_and_explicit_mtrrs_override_it() {
        let mut image = image();
        image.default = 0xc00;
        assert!(image.writeback_ram(4 << 30, 4096).is_err());
        image.amd.as_mut().unwrap().syscfg |= amd::TOM2_WB;
        assert!(image.writeback_ram(4 << 30, 4096).is_ok());
        assert!(image.writeback_ram(1 << 20, 4096).is_err());
        span(&mut image, 4 << 30, 2 << 20, 0);
        assert!(image.writeback_ram(4 << 30, 4096).is_err());
        span(&mut image, 4 << 30, 2 << 20, 6);
        assert!(image.writeback_ram(4 << 30, 4096).is_ok());
        image.amd.as_mut().unwrap().syscfg &= !amd::TOM2_WB;
        assert!(image.writeback_ram((4 << 30) + (2 << 20) - 1, 2).is_err());
    }
    #[test]
    fn iorr_and_top_mem_must_agree_on_both_directions() {
        let mut image = image();
        let base = 3 << 30;
        let mask = (((1u64 << 48) - 1) & !((2 << 20) - 1)) | (1 << 11);
        image.amd.as_mut().unwrap().iorr[0] = (base | 0x18, mask);
        assert!(image.writeback_ram(base, 2 << 20).is_err()); // conflicting TOM/IORR refuses
        image.amd.as_mut().unwrap().iorr[0].0 = (2 << 20) | 0x18;
        assert!(image.writeback_ram(2 << 20, 2 << 20).is_ok());
        for route in [0, 8, 16] {
            image.amd.as_mut().unwrap().iorr[0].0 = (2 << 20) | route;
            assert!(image.writeback_ram(2 << 20, 4096).is_err());
        }
    }
    #[test]
    fn without_routing_registers_ram_needs_only_wb_types() {
        let mut image = image();
        image.amd = Some(amd::State {
            syscfg: 0,
            iorr: [(0, 0); 2],
            top_mem: 0,
            top_mem2: 0,
        });
        for index in super::super::FIXED {
            set(&mut image, index, 0x0606_0606_0606_0606);
        }
        span(&mut image, 0, 16 << 30, 6);
        assert!(image.writeback_ram(0, 1 << 20).is_ok());
        assert!(image.writeback_ram(1 << 20, 8 << 30).is_ok());
        // An MTRR type still refuses.
        span(&mut image, 0, 16 << 30, 0);
        assert!(image.writeback_ram(1 << 20, 4096).is_err());
        // One routing value present restores every routing check.
        span(&mut image, 0, 16 << 30, 6);
        image.amd.as_mut().unwrap().top_mem = 3 << 30;
        assert!(image.writeback_ram(1 << 20, 4096).is_err());
    }

    #[test]
    fn fixed_entries_use_their_exact_width_and_require_wb_dram() {
        let mut image = image();
        assert!(image.writeback_ram(0, 1 << 20).is_ok());
        for (index, start, width) in [
            (0x250, 0, 0x10000),
            (0x258, 0x80000, 0x4000),
            (0x259, 0xa0000, 0x4000),
            (0x268, 0xc0000, 0x1000),
            (0x26f, 0xf8000, 0x1000),
        ] {
            set(&mut image, index, 0x1e1e_1e1e_1e1e_1e00);
            assert!(image.writeback_ram(start, 1).is_err());
            assert!(image.writeback_ram(start + width, 1).is_ok());
            set(&mut image, index, 0x1e1e_1e1e_1e1e_1e1e);
        }
        image.amd.as_mut().unwrap().syscfg &= !amd::FIXED_DRAM;
        assert!(image.writeback_ram(0x8000, 4096).is_err());
    }
    #[test]
    fn every_non_wb_overlap_refuses() {
        let mut image = image();
        for kind in [0, 1, 4, 5] {
            span(&mut image, 2 << 20, 2 << 20, kind);
            assert!(image.writeback_ram((2 << 20) - 1, 2).is_err());
        }
    }
    #[test]
    fn a_misaligned_base_and_a_mask_with_holes_match_as_the_hardware_does() {
        const MIB: u64 = 1 << 20;
        // A 32 MiB range whose base has a low bit set starts at 32 MiB.
        let mask = 0xffff_fe00_0000;
        assert_eq!(variable(0x200_1000, mask, 48, 16 * MIB), (false, 32 * MIB));
        assert_eq!(variable(0x200_1000, mask, 48, 32 * MIB), (true, 64 * MIB));
        assert_eq!(variable(0x200_1000, mask, 48, 64 * MIB), (false, u64::MAX));
        // A 2 MiB mask with bit 30 clear matches 2 MiB at 0 and at 1 GiB.
        let holed = 0xffff_bfe0_0000;
        assert_eq!(variable(0, holed, 48, 0), (true, 2 * MIB));
        assert_eq!(variable(0, holed, 48, 2 * MIB), (false, 4 * MIB));
        assert_eq!(variable(0, holed, 48, 1 << 30), (true, (1 << 30) + 2 * MIB));
        let mut image = image();
        set(&mut image, VARIABLE_BASE, 0x200_1000);
        set(&mut image, VARIABLE_BASE + 1, mask | 1 << 11);
        assert!(image.writeback_ram(16 * MIB, 4096).is_ok());
        assert!(image.writeback_ram(33 * MIB, 4096).is_err());
        set(&mut image, VARIABLE_BASE, 0);
        set(&mut image, VARIABLE_BASE + 1, holed | 1 << 11);
        assert!(image.writeback_ram(4 * MIB, MIB).is_ok());
        assert!(image.writeback_ram((1 << 30) + MIB, 4096).is_err());
    }

    /// A Ryzen desktop with 32 GiB: UC default, WB variable ranges below
    /// TOP_MEM only, and SYSCFG 0x740000 (MtrrFixDramEn, MtrrVarDramEn,
    /// MtrrTom2En, Tom2ForceMemTypeWB) making RAM from 4 GiB to TOP_MEM2
    /// write-back without any variable range.
    #[test]
    fn ryzen_tom2_force_wb_covers_ram_above_4_gib() {
        let mut image = image();
        image.default = 0xc00;
        image.cap = (1 << 8) | 2;
        let state = image.amd.as_mut().unwrap();
        state.syscfg = 0x74_0000;
        state.top_mem = 0xe000_0000;
        state.top_mem2 = 0x8_2000_0000;
        set(&mut image, VARIABLE_BASE, 6);
        set(&mut image, VARIABLE_BASE + 1, 0xffff_8000_0800);
        set(&mut image, VARIABLE_BASE + 2, 0x8000_0006);
        set(&mut image, VARIABLE_BASE + 3, 0xffff_c000_0800);
        for (start, bytes, expected) in [
            (1 << 20, (3 << 30) - (1 << 20), true),
            (0xc000_0000, 4096, false),
            (0xe000_0000, 4096, false),
            (4 << 30, 32 << 20, true),
            (0x8_2000_0000 - (32 << 20), 32 << 20, true),
            (0x8_2000_0000, 4096, false),
        ] {
            assert_eq!(
                image.writeback_ram(start, bytes).is_ok(),
                expected,
                "{start:x}/{bytes:x}"
            );
        }
        // Without the force bit the same memory reads as the UC default.
        image.amd.as_mut().unwrap().syscfg &= !amd::TOM2_WB;
        assert!(image.writeback_ram(4 << 30, 4096).is_err());
    }
}
