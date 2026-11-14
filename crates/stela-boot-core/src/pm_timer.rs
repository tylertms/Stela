//! ACPI fixed PM timer resource parsing and bounded counter arithmetic. The firmware table supplies a port, never a rate.
//! ACPI 6.5 section 4.8.3.3 fixes the source at 3.579545 MHz.
use stela_crypto::Sha256;
use stela_crypto::digest::Digest;
use stela_drivers::platform::work::WorkError;

pub const HZ: u64 = 3_579_545;
pub const NS: u64 = 1_000_000_000;

#[derive(Clone, Copy)]
pub struct Prepared {
    pub port: u16,
    pub mask: u32,
    max_gap: u32,
    bounded_24: bool,
    pub fadt_sha256: [u8; 32],
}

impl Prepared {
    /// Parse only the architected 32-bit SystemIO access to the fixed timer.
    /// Extended GAS takes precedence; an unsupported GAS is never hidden by
    /// falling back to the legacy block. Hardware-reduced ACPI has no timer.
    pub fn from_fadt(table: &[u8]) -> Result<Self, WorkError> {
        if table.len() < 116
            || &table[..4] != b"FACP"
            || u32::from_le_bytes(table[4..8].try_into().unwrap()) as usize != table.len()
            || table.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)) != 0
        {
            return Err(WorkError::Unavailable);
        }
        let flags = u32::from_le_bytes(table[112..116].try_into().unwrap());
        if flags & (1 << 20) != 0 || table[91] != 4 {
            return Err(WorkError::Unavailable);
        }
        let mut address = u64::from(u32::from_le_bytes(table[76..80].try_into().unwrap()));
        if table.len() >= 220 {
            let gas = &table[208..220];
            let extended = u64::from_le_bytes(gas[4..12].try_into().unwrap());
            if extended != 0 {
                if gas[0] != 1 || gas[1] != 32 || gas[2] != 0 || !matches!(gas[3], 0 | 3) {
                    return Err(WorkError::Unavailable);
                }
                address = extended;
            }
        }
        let port = u16::try_from(address).map_err(|_| WorkError::Unavailable)?;
        if port == 0 || port > u16::MAX - 3 {
            return Err(WorkError::Unavailable);
        }
        let mask = if flags & (1 << 8) != 0 {
            u32::MAX
        } else {
            0x00ff_ffff
        };
        Ok(Self {
            port,
            mask,
            max_gap: mask / 2,
            bounded_24: false,
            fadt_sha256: Sha256::digest(table).into(),
        })
    }

    /// # Safety
    /// Source/entry review proves EVERY sample gap for this root
    /// (including firmware/SMM, masked original C, calibration and idle) is at
    /// most max_gap_ns. Periodic IRQ delivery alone is not such a proof. This
    /// enables short 24-bit diagnostics; it does not check an arbitrary Linux
    /// driver domain whose masked execution has no established upper bound.
    pub unsafe fn qualify_bounded(mut self, max_gap_ns: u64) -> Result<Self, WorkError> {
        let ticks = ticks_ceil(max_gap_ns)?;
        if ticks == 0 || ticks > u64::from(self.mask / 2) {
            return Err(WorkError::Unavailable);
        }
        self.max_gap = ticks as u32;
        self.bounded_24 = true;
        Ok(self)
    }

    pub fn admit(self) -> Result<(), WorkError> {
        if self.mask != u32::MAX && !self.bounded_24 {
            return Err(WorkError::Unavailable);
        }
        Ok(())
    }

    pub fn delta(self, previous: u32, current: u32) -> Result<u64, WorkError> {
        let delta = current.wrapping_sub(previous) & self.mask;
        if delta > self.max_gap {
            return Err(WorkError::corrupt());
        }
        Ok(u64::from(delta))
    }
}

pub fn ns(ticks: u64) -> Result<u64, WorkError> {
    u64::try_from(u128::from(ticks) * u128::from(NS) / u128::from(HZ))
        .map_err(|_| WorkError::Exhausted)
}
pub fn ticks_ceil(ns: u64) -> Result<u64, WorkError> {
    u64::try_from((u128::from(ns) * u128::from(HZ)).div_ceil(u128::from(NS)))
        .map_err(|_| WorkError::Exhausted)
}

#[cfg(test)]
mod tests {
    use super::*;
    extern crate alloc;
    fn fadt(width32: bool) -> alloc::vec::Vec<u8> {
        let mut table = alloc::vec![0; 220];
        table[..4].copy_from_slice(b"FACP");
        table[4..8].copy_from_slice(&220u32.to_le_bytes());
        table[76..80].copy_from_slice(&0x608u32.to_le_bytes());
        table[91] = 4;
        if width32 {
            table[113] = 1;
        }
        seal(&mut table);
        table
    }
    fn seal(table: &mut [u8]) {
        table[9] = 0;
        table[9] = 0u8.wrapping_sub(table.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)));
    }
    #[test]
    fn fixed_source_wrap_and_checked_time_conversion() {
        let source = Prepared::from_fadt(&fadt(false)).unwrap();
        assert_eq!(source.delta(0x00ff_fff0, 0x10), Ok(32));
        assert_eq!(source.delta(100, 99), Err(WorkError::CorruptContext));
        assert_eq!(ns(HZ), Ok(NS));
        assert_eq!(ticks_ceil(1), Ok(1));
        assert_eq!(ticks_ceil(NS), Ok(HZ));
        assert_eq!(ns(u64::MAX), Err(WorkError::Exhausted));
    }
    #[test]
    fn extended_io_resource_overrides_legacy_and_other_gas_refuses() {
        let mut table = fadt(true);
        table[208] = 1;
        table[209] = 32;
        table[211] = 3;
        table[212..220].copy_from_slice(&0x408u64.to_le_bytes());
        seal(&mut table);
        let source = Prepared::from_fadt(&table).unwrap();
        assert_eq!((source.port, source.mask), (0x408, u32::MAX));
        table[208] = 0;
        seal(&mut table);
        assert!(Prepared::from_fadt(&table).is_err());
        table[208] = 1;
        table[114] |= 1 << 4;
        seal(&mut table);
        assert!(Prepared::from_fadt(&table).is_err());
    }
}
