//! AML SystemMemory accesses that land in an MCFG window. ECAM is memory
//! mapped, so firmware AML may read a function's configuration through a
//! SystemMemory field, as Linux serves it by mapping the window. Here each
//! access is decoded into configuration reads the native owner performs.
use stela_drivers::bus::pci::{Address, EcamWindow};

/// The function and register `physical` decodes to in `windows`.
pub fn decode(windows: &[EcamWindow], physical: u64) -> Option<(Address, u16)> {
    windows.iter().find_map(|window| {
        if !window.range().contains(&physical) {
            return None;
        }
        let offset = physical - window.base;
        Some((
            Address {
                segment: window.segment,
                bus: (offset >> 20) as u8,
                device: ((offset >> 15) & 0x1f) as u8,
                function: ((offset >> 12) & 7) as u8,
            },
            (offset & 0xfff) as u16,
        ))
    })
}

/// Whether `physical..physical + bytes` touches any of `windows`.
pub fn touches(windows: &[EcamWindow], physical: u64, bytes: u64) -> bool {
    let end = physical.saturating_add(bytes.max(1));
    windows.iter().any(|window| {
        let range = window.range();
        physical < range.end && range.start < end
    })
}

/// Why an access was not served.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Refusal<E> {
    /// The access is not 1, 2, 4 or 8 bytes.
    Width,
    /// Part of the access lies outside every window.
    Outside { physical: u64 },
    /// The configuration owner refused one read.
    Config {
        address: Address,
        offset: u16,
        error: E,
    },
}

/// The naturally aligned pieces of 1, 2 or 4 bytes that cover
/// `physical..physical + bytes`, none crossing a dword, as a processor's
/// access reaches PCI as byte-enabled dword requests.
fn pieces(physical: u64, bytes: u64) -> impl Iterator<Item = (u64, u16)> {
    let end = physical + bytes;
    let mut at = physical;
    core::iter::from_fn(move || {
        (at < end).then(|| {
            let width = [4u16, 2, 1]
                .into_iter()
                .find(|&width| at.is_multiple_of(u64::from(width)) && at + u64::from(width) <= end)
                .expect("a byte always fits");
            let piece = (at, width);
            at += u64::from(width);
            piece
        })
    })
}

/// Read `bytes` (1, 2, 4 or 8) at `physical` in `windows`, little-endian,
/// through `read(address, offset, width)`, which reads one naturally
/// aligned register of 1, 2 or 4 bytes as the configuration owner allows.
pub fn read<E>(
    windows: &[EcamWindow],
    physical: u64,
    bytes: u64,
    mut read: impl FnMut(Address, u16, u16) -> Result<u32, E>,
) -> Result<u64, Refusal<E>> {
    if !matches!(bytes, 1 | 2 | 4 | 8) || physical.checked_add(bytes).is_none() {
        return Err(Refusal::Width);
    }
    let mut value = 0u64;
    for (at, width) in pieces(physical, bytes) {
        let (address, offset) = decode(windows, at).ok_or(Refusal::Outside { physical: at })?;
        let piece = read(address, offset, width).map_err(|error| Refusal::Config {
            address,
            offset,
            error,
        })?;
        let mask = (1u64 << (u32::from(width) * 8)) - 1;
        value |= (u64::from(piece) & mask) << ((at - physical) * 8);
    }
    Ok(value)
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::vec::Vec;

    const WINDOWS: [EcamWindow; 2] = [
        EcamWindow {
            base: 0xc000_0000,
            segment: 0,
            start_bus: 0,
            end_bus: 0x7f,
        },
        EcamWindow {
            base: 0xd000_0000,
            segment: 1,
            start_bus: 0x10,
            end_bus: 0x1f,
        },
    ];

    fn at(segment: u16, bus: u8, device: u8, function: u8) -> Address {
        Address {
            segment,
            bus,
            device,
            function,
        }
    }

    #[test]
    fn decodes_segment_bus_device_function_and_register() {
        // A DSDT's module-level read of a root port's secondary bus.
        assert_eq!(
            decode(&WINDOWS, 0xc00e_0019),
            Some((at(0, 0, 0x1c, 0), 0x19))
        );
        // A whole function's page, from its first byte.
        assert_eq!(decode(&WINDOWS, 0xc00a_3000), Some((at(0, 0, 0x14, 3), 0)));
        assert_eq!(
            decode(&WINDOWS, 0xc7ff_ffff),
            Some((at(0, 0x7f, 31, 7), 0xfff))
        );
        // A window's bus offset counts from its base, not its first bus.
        assert_eq!(
            decode(
                &WINDOWS,
                0xd000_0000 + (0x12 << 20) + (5 << 15) + (1 << 12) + 0x100
            ),
            Some((at(1, 0x12, 5, 1), 0x100))
        );
        assert_eq!(decode(&WINDOWS, 0xc800_0000), None);
        assert_eq!(decode(&WINDOWS, 0xd000_0000), None, "below the first bus");
        assert_eq!(decode(&WINDOWS, 0xbfff_ffff), None);
    }

    #[test]
    fn touches_any_overlap_of_a_window() {
        assert!(touches(&WINDOWS, 0xc000_0000, 1));
        assert!(touches(&WINDOWS, 0xbfff_fffc, 8));
        assert!(!touches(&WINDOWS, 0xbfff_fff8, 8));
        assert!(!touches(&WINDOWS, 0xc800_0000, 4));
        assert!(touches(&WINDOWS, 0xd1ff_ffff, 1));
        assert!(!touches(&WINDOWS, 0xd200_0000, 1));
    }

    /// A configuration space whose byte at offset `n` reads as `n`, and
    /// whose function 0:1f.7 is absent and answers all ones.
    fn space(
        log: &mut Vec<(Address, u16, u16)>,
    ) -> impl FnMut(Address, u16, u16) -> Result<u32, ()> + '_ {
        move |address, offset, width| {
            log.push((address, offset, width));
            if address == at(0, 0, 0x1f, 7) {
                return Ok(u32::MAX);
            }
            let bytes = [0, 1, 2, 3].map(|n| (offset as u8).wrapping_add(n));
            Ok(u32::from_le_bytes(bytes) & (u32::MAX >> (32 - u32::from(width) * 8)))
        }
    }

    #[test]
    fn reads_each_width_as_aligned_registers() {
        let base = 0xc00e_0000;
        for (offset, bytes, value, reads) in [
            (0x19u64, 1u64, 0x19u64, &[(0x19u16, 1u16)][..]),
            (0x18, 2, 0x1918, &[(0x18, 2)]),
            (0x18, 4, 0x1b1a_1918, &[(0x18, 4)]),
            (0x18, 8, 0x1f1e_1d1c_1b1a_1918, &[(0x18, 4), (0x1c, 4)]),
            // Unaligned: as a processor splits it into byte-enabled dwords.
            (0x19, 2, 0x1a19, &[(0x19, 1), (0x1a, 1)]),
            (0x1a, 4, 0x1d1c_1b1a, &[(0x1a, 2), (0x1c, 2)]),
            (
                0x1b,
                8,
                0x2221_201f_1e1d_1c1b,
                &[(0x1b, 1), (0x1c, 4), (0x20, 2), (0x22, 1)],
            ),
        ] {
            let mut log = Vec::new();
            assert_eq!(
                read(&WINDOWS, base + offset, bytes, space(&mut log)),
                Ok(value),
                "{bytes} bytes at {offset:#x}"
            );
            let expected: Vec<_> = reads
                .iter()
                .map(|&(offset, width)| (at(0, 0, 0x1c, 0), offset, width))
                .collect();
            assert_eq!(log, expected, "{bytes} bytes at {offset:#x}");
        }
    }

    #[test]
    fn an_absent_function_reads_all_ones_at_every_width() {
        let base = 0xc00f_f000;
        for bytes in [1, 2, 4, 8] {
            let mut log = Vec::new();
            let all = if bytes == 8 {
                u64::MAX
            } else {
                (1 << (bytes * 8)) - 1
            };
            assert_eq!(read(&WINDOWS, base, bytes, space(&mut log)), Ok(all));
        }
    }

    #[test]
    fn an_access_across_functions_reads_each_one() {
        let mut log = Vec::new();
        assert_eq!(
            read(&WINDOWS, 0xc00e_0ffc, 8, space(&mut log)),
            Ok(0x0302_0100_fffe_fdfc)
        );
        assert_eq!(
            log,
            [(at(0, 0, 0x1c, 0), 0xffc, 4), (at(0, 0, 0x1c, 1), 0, 4)]
        );
    }

    #[test]
    fn refusals_name_what_was_not_served() {
        let mut log = Vec::new();
        assert_eq!(
            read(&WINDOWS, 0xc000_0000, 3, space(&mut log)),
            Err(Refusal::Width)
        );
        assert_eq!(
            read(&WINDOWS, u64::MAX - 1, 4, space(&mut log)),
            Err(Refusal::Width)
        );
        // Straddling the window's end: the part beyond it is not ECAM.
        assert_eq!(
            read(&WINDOWS, 0xc7ff_fffc, 8, space(&mut log)),
            Err(Refusal::Outside {
                physical: 0xc800_0000
            })
        );
        let refused = read(&WINDOWS, 0xc00a_3004, 4, |_, _, _| Err("custody"));
        assert_eq!(
            refused,
            Err(Refusal::Config {
                address: at(0, 0, 0x14, 3),
                offset: 4,
                error: "custody"
            })
        );
    }
}
