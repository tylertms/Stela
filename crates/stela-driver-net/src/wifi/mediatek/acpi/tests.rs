use super::*;

/// An MTCL of `bytes` and `version` whose 6 GHz, 5.9 GHz and EHT modes are
/// one, with `bits` set in its rows of each list.
fn mtcl(bytes: usize, version: u8, bits: &[(usize, u8)]) -> Vec<u8> {
    let mut table = vec![0; bytes];
    table[..4].copy_from_slice(b"MTCL");
    table[4] = version;
    table[5] = 1;
    table[12] = 1;
    if bytes == MTCL_V3_BYTES {
        table[19] = 1;
    }
    for &(row, bit) in bits {
        for list in [6, 13, 20] {
            if let Some(byte) = table.get_mut(list + row) {
                *byte |= bit;
            }
        }
    }
    table
}

#[test]
fn mtcl_gates_bands_by_country_list_and_version() {
    // "US" is cc_list_all's 29th entry (row 3, bit 3) and cc_list_be's 25th
    // (row 3, bit 7).
    let us = [(3, 1 << 3)];
    for (table, expected) in [
        (mtcl(MTCL_BYTES, 1, &us), 0x20 | 4),
        (mtcl(MTCL_BYTES, 2, &us), 0x20 | 4 | 1),
        (mtcl(MTCL_BYTES, 2, &[]), 0x20),
        (
            mtcl(MTCL_V3_BYTES, 3, &[(3, 1 << 7 | 1 << 3)]),
            0x10 | 4 | 1,
        ),
        (mtcl(MTCL_V3_BYTES, 3, &us), 4 | 1),
        // A short table cannot name EHT countries.
        (mtcl(MTCL_BYTES, 3, &[(3, 0xff)]), 4 | 1),
    ] {
        let sar = Sar::new(Some(table), None, None);
        assert_eq!(sar.mtcl_configuration(*b"US"), Some(expected));
    }
    // Unlisted countries take row 0's bit 7, Europe's its bit 6.
    let sar = Sar::new(Some(mtcl(MTCL_BYTES, 2, &[(0, 1 << 6)])), None, None);
    assert_eq!(sar.mtcl_configuration(*b"DE"), Some(0x20 | 4 | 1));
    assert_eq!(sar.mtcl_configuration(*b"ZZ"), Some(0x20));
    // Mode 2 enables 6 GHz everywhere.
    let mut everywhere = mtcl(MTCL_BYTES, 1, &[]);
    everywhere[5] = 2;
    let sar = Sar::new(Some(everywhere), None, None);
    assert_eq!(sar.mtcl_configuration(*b"ZZ"), Some(0x20 | 8));
    for absent in [None, Some(vec![0; 20])] {
        assert_eq!(
            Sar::new(absent, None, None).mtcl_configuration(*b"US"),
            None
        );
    }
}

/// An enabled MTDS of `rows` rows of `limits`, in the layout of `version`.
fn mtds(version: u8, rows: usize, limits: &[u8]) -> Vec<u8> {
    let (header, entry) = if version == 1 { (6, 6) } else { (7, 12) };
    let mut table = vec![0; header + rows * entry];
    table[4] = 1;
    table[header - 1] = rows as u8;
    table[header + 1..header + 1 + limits.len()].copy_from_slice(limits);
    table
}

#[test]
fn dynamic_limits_take_the_first_row_in_either_layout() {
    for version in [1, 2] {
        let sar = Sar::new(None, Some(mtds(version, 2, &[20, 0xff, 0x90])), None);
        assert_eq!(sar.range_power(0, 2402, 1), Some(20));
        assert_eq!(sar.range_power(1, 5150, 1), Some(127));
        assert_eq!(sar.range_power(2, 5350, 1), Some(-112));
    }
    let v1 = Sar::new(None, Some(mtds(1, 1, &[20; 5])), None);
    assert_eq!(v1.range_power(5, 5945, 1), Some(127));
    let mut disabled = mtds(1, 1, &[20; 5]);
    disabled[4] = 0;
    for table in [None, Some(disabled), Some(mtds(1, 9, &[20; 5]))] {
        assert_eq!(Sar::new(None, table, None).range_power(0, 2402, 1), None);
    }
}

#[test]
fn geographic_rows_cap_dynamic_limits_by_region_and_band() {
    // Rows FCC, ETSI and worldwide; 2.4 GHz then 5 GHz (power, offset).
    let mut mtgs = vec![0; 6 + 3 * 5];
    mtgs[5] = 3;
    for (row, bands) in [[(30, 4), (16, 0)], [(0xff, 0), (24, 2)], [(10, 0), (10, 0)]]
        .into_iter()
        .enumerate()
    {
        for (band, (power, offset)) in bands.into_iter().enumerate() {
            mtgs[6 + row * 5 + 1 + band * 2..][..2].copy_from_slice(&[power, offset]);
        }
    }
    let sar = Sar::new(None, Some(mtds(1, 1, &[20; 5])), Some(mtgs));
    for (region, start, expected) in [
        (1, 2402, 24),
        (1, 5150, 16),
        (2, 2402, 20),
        (2, 5150, 22),
        (0, 2402, 10),
        (3, 5150, 10),
        // The v1 layout has no 6 GHz column.
        (1, 5945, 20),
    ] {
        assert_eq!(sar.range_power(0, start, region), Some(expected));
    }
}

#[test]
fn mtfg_maps_flags_1_and_4_to_the_clc_bits() {
    // mt792x_acpi_get_flags: 0 off ACPI, bit 0 on it, MTFG flags 1 and 4.
    assert_eq!(Sar::default().flags(), 0);
    assert_eq!(Sar::default().on_acpi(None).flags(), 1);
    assert_eq!(Sar::default().on_acpi(Some(vec![0; 7])).flags(), 1);
    let mut mtfg = vec![0; 8];
    mtfg.extend_from_slice(&[4, 2, 1]);
    mtfg[6] = 2;
    assert_eq!(Sar::default().on_acpi(Some(mtfg.clone())).flags(), 5);
    mtfg[6] = 9;
    assert_eq!(Sar::default().on_acpi(Some(mtfg)).flags(), 7);
}
