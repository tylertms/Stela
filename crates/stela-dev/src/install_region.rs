//! Sector-aligned, read-only access confined to an explicit host region.
use std::{
    fs::File,
    io::{self, Read, Seek, SeekFrom},
};

pub(super) struct Region {
    file: File,
    start: u64,
    bytes: u64,
}

impl Region {
    pub(super) fn new(file: File, start: u64, bytes: u64) -> io::Result<Self> {
        if bytes < 4096
            || !start.is_multiple_of(512)
            || !bytes.is_multiple_of(512)
            || start.checked_add(bytes).is_none()
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "invalid region bounds",
            ));
        }
        Ok(Self { file, start, bytes })
    }
    pub(super) fn len(&self) -> u64 {
        self.bytes
    }
    pub(super) fn read(&mut self, at: u64, out: &mut [u8]) -> io::Result<()> {
        if at
            .checked_add(out.len() as u64)
            .is_none_or(|end| end > self.bytes)
        {
            return Err(io::Error::new(
                io::ErrorKind::InvalidInput,
                "read outside region",
            ));
        }
        if out.is_empty() {
            return Ok(());
        }
        let position = self
            .start
            .checked_add(at)
            .ok_or_else(|| io::Error::other("offset overflow"))?;
        let aligned = position / 512 * 512;
        let prefix = (position - aligned) as usize;
        let length = prefix
            .checked_add(out.len())
            .and_then(|n| n.checked_add(511))
            .map(|n| n / 512 * 512)
            .ok_or_else(|| io::Error::other("read length overflow"))?;
        // Bounds are sector-aligned, so rounding cannot expose adjacent partitions.
        let mut buffer = vec![0u8; length];
        self.file.seek(SeekFrom::Start(aligned))?;
        self.file.read_exact(&mut buffer)?;
        out.copy_from_slice(&buffer[prefix..prefix + out.len()]);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::{
        io::Write,
        sync::atomic::{AtomicU64, Ordering},
    };
    static NEXT: AtomicU64 = AtomicU64::new(0);
    struct Fixture(std::path::PathBuf);
    impl Drop for Fixture {
        fn drop(&mut self) {
            let _ = std::fs::remove_file(&self.0);
        }
    }
    fn fixture() -> (Fixture, Region) {
        let path = std::env::temp_dir().join(format!(
            "stela-region-{}-{}",
            std::process::id(),
            NEXT.fetch_add(1, Ordering::Relaxed)
        ));
        let mut file = File::options()
            .create_new(true)
            .write(true)
            .open(&path)
            .unwrap();
        let data: Vec<u8> = (0..5120).map(|n| (n % 251) as u8).collect();
        file.write_all(&data).unwrap();
        drop(file);
        let region = Region::new(File::open(&path).unwrap(), 512, 4096).unwrap();
        (Fixture(path), region)
    }
    #[test]
    fn unaligned_read_uses_region_origin() {
        let (_fixture, mut r) = fixture();
        let mut out = [0; 777];
        r.read(13, &mut out).unwrap();
        assert!(
            out.iter()
                .enumerate()
                .all(|(i, b)| *b == ((525 + i) % 251) as u8)
        );
        assert_eq!(r.len(), 4096);
    }
    #[test]
    fn last_byte_and_empty_end_are_valid() {
        let (_fixture, mut r) = fixture();
        let mut out = [0];
        r.read(4095, &mut out).unwrap();
        assert_eq!(out[0], (4607 % 251) as u8);
        r.read(4096, &mut []).unwrap();
    }
    #[test]
    fn rejects_crossing_end_and_overflow_without_touching_output() {
        let (_fixture, mut r) = fixture();
        let mut out = [99; 2];
        assert!(r.read(4095, &mut out).is_err());
        assert!(r.read(u64::MAX, &mut out).is_err());
        assert!(r.read(4097, &mut []).is_err());
        assert_eq!(out, [99; 2]);
    }
    #[test]
    fn rejects_unaligned_small_or_overflowing_regions() {
        let (fixture, _) = fixture();
        for (start, bytes) in [(1, 4096), (0, 4095), (0, 512), (u64::MAX - 511, 4096)] {
            assert!(Region::new(File::open(&fixture.0).unwrap(), start, bytes).is_err());
        }
    }
    #[test]
    fn truncated_source_is_an_error() {
        let (fixture, _) = fixture();
        let mut r = Region::new(File::open(&fixture.0).unwrap(), 4096, 4096).unwrap();
        assert!(r.read(2048, &mut [0; 1]).is_err());
    }
}
