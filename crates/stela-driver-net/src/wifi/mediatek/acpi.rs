// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2023 MediaTek Inc.
//! The platform tables `mt792x_acpi_sar.c` reads from the function's ACPI
//! companion: MTCL, the countries allowed 5.9 GHz, 6 GHz and EHT operation;
//! MTDS, the dynamic SAR power limits; MTGS, their per-region caps; and
//! MTFG, the platform's flags for MT7921 and MT7922's CLC.
//! A table that is absent or malformed is not used, as there.

use alloc::vec::Vec;

/// `cc_list_all`: MTCL's countries, eight to a byte from bit 7 down.
const COUNTRIES: [&[u8; 2]; 32] = [
    b"00", b"EU", b"AR", b"AU", b"AZ", b"BY", b"BO", b"BR", b"CA", b"CL", b"CN", b"ID", b"JP",
    b"MY", b"MX", b"ME", b"MA", b"NZ", b"NG", b"PH", b"RU", b"RS", b"SG", b"KR", b"TW", b"TH",
    b"UA", b"GB", b"US", b"VN", b"KH", b"PY",
];
/// `cc_list_eu`: the countries MTCL's "EU" bit covers.
const EUROPE: [&[u8; 2]; 33] = [
    b"AD", b"AT", b"BE", b"BG", b"CY", b"CZ", b"HR", b"DK", b"EE", b"FI", b"FR", b"DE", b"GR",
    b"HU", b"IS", b"IE", b"IT", b"LV", b"LI", b"LT", b"LU", b"MC", b"MT", b"NL", b"NO", b"PL",
    b"PT", b"RO", b"SK", b"SI", b"ES", b"SE", b"CH",
];
/// `cc_list_be`: MTCL v3's EHT countries; empty names match nothing.
const EHT_COUNTRIES: [&[u8]; 25] = [
    b"AR", b"BR", b"BY", b"CL", b"IQ", b"MX", b"OM", b"RU", b"RW", b"VN", b"KR", b"UA", b"", b"",
    b"", b"", b"EU", b"AT", b"CN", b"CA", b"TW", b"NZ", b"PH", b"UK", b"US",
];

/// `sizeof(struct mt792x_asar_cl)` and its v3 extension.
const MTCL_BYTES: usize = 19;
const MTCL_V3_BYTES: usize = 26;
/// `MT792x_ASAR_MIN_FG`.
const MTFG_MIN_BYTES: usize = 8;

/// One function's SAR tables, as `mt792x_init_acpi_sar` keeps them.
#[derive(Clone, Debug, Default)]
pub struct Sar {
    countries: Option<Vec<u8>>,
    /// MTDS and its layout version.
    dynamic: Option<(Vec<u8>, u8)>,
    /// MTGS and its layout version.
    geographic: Option<(Vec<u8>, u8)>,
    /// Whether the tables were read from ACPI at all: `phy->acpisar`.
    acpi: bool,
    /// MTFG, when long enough.
    flags: Option<Vec<u8>>,
}

/// `mt792x_asar_fits`: `table` holds a header of `header` bytes and its
/// count byte at `count` of entries of `entry` bytes, within `min..=max`.
fn fits(table: &[u8], header: usize, entry: usize, count: usize, min: usize, max: usize) -> bool {
    let Some(rows) = table.len().checked_sub(header) else {
        return false;
    };
    rows.is_multiple_of(entry)
        && (min..=max).contains(&(rows / entry))
        && rows / entry == usize::from(table[count])
}

/// The layout a dynamic or geographic table actually has, as
/// `mt792x_asar_acpi_read_mtds` and `_mtgs` choose it whatever the version
/// MTCL names.
fn layout(
    table: &[u8],
    v1: (usize, usize),
    v2: (usize, usize),
    rows: (usize, usize),
) -> Option<u8> {
    let (min, max) = rows;
    if fits(table, v1.0, v1.1, v1.0 - 1, min, max) {
        Some(1)
    } else if fits(table, v2.0, v2.1, v2.0 - 1, min, max) {
        Some(2)
    } else {
        None
    }
}

impl Sar {
    /// Each table is the package of integers `mt792x_acpi_read` returned,
    /// each truncated to a byte.
    pub fn new(mtcl: Option<Vec<u8>>, mtds: Option<Vec<u8>>, mtgs: Option<Vec<u8>>) -> Self {
        Self {
            countries: mtcl.filter(|t| matches!(t.len(), MTCL_BYTES | MTCL_V3_BYTES)),
            // MTDS must also be enabled.
            dynamic: mtds.and_then(|t| {
                let version = layout(&t, (6, 6), (7, 12), (1, 8))?;
                (t[4] != 0).then_some((t, version))
            }),
            geographic: mtgs
                .and_then(|t| layout(&t, (6, 5), (7, 7), (3, 8)).map(|version| (t, version))),
            acpi: false,
            flags: None,
        }
    }

    /// The tables as `mt792x_init_acpi_sar` keeps them on an ACPI platform,
    /// with its optional MTFG.
    pub fn on_acpi(mut self, mtfg: Option<Vec<u8>>) -> Self {
        self.acpi = true;
        self.flags = mtfg.filter(|t| t.len() >= MTFG_MIN_BYTES);
        self
    }

    /// `mt792x_acpi_get_flags`: bit 0 on an ACPI platform, and MTFG's flags
    /// 1 and 4 as the CLC's bits 1 and 2.
    pub fn flags(&self) -> u8 {
        if !self.acpi {
            return 0;
        }
        let Some(table) = self.flags.as_deref() else {
            return 1;
        };
        // nr_flag at 6, the flags from 8, bounded by what was read.
        let count = usize::from(table[6]);
        table[8..]
            .iter()
            .take(count)
            .fold(1, |flags, &flag| match flag {
                1 => flags | 1 << 1,
                4 => flags | 1 << 2,
                _ => flags,
            })
    }

    /// `mt792x_acpi_get_mtcl_conf`: bits 0-1 enable 5.9 GHz, bits 2-3 6 GHz
    /// and bits 4-5 EHT, `None` without a usable MTCL.
    pub fn mtcl_configuration(&self, alpha2: [u8; 2]) -> Option<u32> {
        let table = self.countries.as_deref()?;
        let version = table[4];
        let bit = |bytes: Option<&[u8]>, row: usize, column: usize| {
            bytes.is_some_and(|b| b[row] & 1 << column != 0)
        };
        // mt792x_acpi_get_mtcl_map_v3; a 19-byte table has no EHT row.
        let eht = |row: usize, column: usize| {
            let mode = table
                .get(19)
                .filter(|&&m| m <= 2)
                .map_or(0, |&m| u32::from(m));
            if version > 2 && bit(table.get(20..26), row, column) {
                (mode & 3) << 4
            } else {
                0
            }
        };
        // mt792x_acpi_get_mtcl_map.
        let bands = |row: usize, column: usize| {
            let six = if table[5] > 2 { 0 } else { u32::from(table[5]) };
            let unii4 = if table[12] > 1 {
                0
            } else {
                u32::from(table[12])
            };
            let mut configuration = 0;
            if bit(Some(&table[6..12]), row, column) || table[5] == 2 {
                configuration |= (six & 3) << 2;
            }
            if version > 1 && bit(Some(&table[13..19]), row, column) {
                configuration |= unii4 & 3;
            }
            configuration
        };
        let cell = |i: usize| (i / 8, 7 - i % 8);
        let european = EUROPE.contains(&&alpha2);
        let v3 = if version != 3 {
            0x20
        } else if let Some(i) = EHT_COUNTRIES.iter().position(|c| *c == alpha2) {
            eht(cell(i).0, cell(i).1)
        } else if european {
            eht(3, 7)
        } else {
            0x20
        };
        let listed = COUNTRIES.iter().position(|c| **c == alpha2);
        let v1 = match (listed, european) {
            (Some(i), _) => bands(cell(i).0, cell(i).1),
            (None, true) => bands(0, 6),
            (None, false) => bands(0, 7),
        };
        Some(v3 | v1)
    }

    /// `mt792x_asar_range_pwr` for `mt76_sar_freq_ranges[index]`, starting at
    /// `start_mhz`: MTDS's first row capped by MTGS for the country's
    /// `dfs_region`, in half-dBm. `None` without MTDS, which leaves the
    /// range unlimited.
    pub fn range_power(&self, index: usize, start_mhz: u32, dfs_region: u8) -> Option<i8> {
        let (table, version) = self.dynamic.as_ref()?;
        // The first row's limits follow its index byte.
        let (at, count) = if *version == 1 { (7, 5) } else { (8, 11) };
        if index >= count {
            return Some(127);
        }
        let band = if start_mhz >= 5945 {
            2
        } else if start_mhz >= 5150 {
            1
        } else {
            0
        };
        let limit = table[at + index];
        Some(self.geographic_power(
            band,
            dfs_region,
            if limit == 0xff { 127 } else { limit as i8 },
        ))
    }

    /// `mt792x_asar_get_geo_pwr`: `dynamic` under the region's MTGS row for
    /// `band` (0 for 2.4 GHz, 1 for 5 GHz, 2 for 6 GHz).
    fn geographic_power(&self, band: usize, dfs_region: u8, dynamic: i8) -> i8 {
        let Some((table, version)) = &self.geographic else {
            return dynamic;
        };
        // NL80211_DFS_FCC, NL80211_DFS_ETSI, then worldwide.
        let row = match dfs_region {
            1 => 0,
            2 => 1,
            _ => 2,
        };
        let (header, entry, bands) = if *version == 1 { (6, 5, 2) } else { (7, 7, 3) };
        if band >= bands {
            return dynamic;
        }
        let at = header + row * entry + 1 + band * 2;
        let (power, offset) = (table[at], table[at + 1]);
        // 0xff marks no limit.
        if power == 0xff {
            return dynamic;
        }
        i32::from(power as i8).min((i32::from(dynamic) + i32::from(offset)).min(127)) as i8
    }
}

/// Read the tables from the function's ACPI companion as
/// `mt792x_acpi_read` evaluates them: each a package of at least four
/// integers.
#[cfg(feature = "native-acpi")]
pub fn read(companion: Option<stela_driver_base::platform::acpi::Handle<'static>>) -> Sar {
    use stela_driver_base::platform::acpi::Value;
    let table = |method: &str| -> Option<Vec<u8>> {
        let Ok(Some(Value::Package(items))) = companion?.evaluate(Some(method), &[]) else {
            return None;
        };
        if items.len() < 4 {
            return None;
        }
        items
            .iter()
            .map(|item| match item {
                Value::Integer(value) => Some(*value as u8),
                _ => None,
            })
            .collect()
    };
    Sar::new(table("MTCL"), table("MTDS"), table("MTGS")).on_acpi(table("MTFG"))
}

#[cfg(test)]
mod tests;
