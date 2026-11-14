//! What a partition holds, read from the signatures that file systems,
//! encryption, volume managers, RAID and partition tables write at their
//! documented offsets, near the start of the partition or its end. A
//! partition is blank only when no signature is found and every byte a
//! signature can sit in is zero: its first [`HEAD_BYTES`] and its last
//! [`TAIL_BYTES`]. A partition that cannot be read is never blank; the
//! caller refuses it.
use anyhow::{Context, Result};

/// The bytes read from the start: past btrfs's and ReiserFS's superblocks
/// at 64 KiB, ZFS's two front labels (512 KiB) and the UDF anchor at block
/// 256 of a 4 KiB-block disk (1 MiB).
pub(super) const HEAD_BYTES: u64 = 2 << 20;
/// The bytes read from the end: ZFS's two back labels (512 KiB), MD RAID
/// 0.90 and 1.0 superblocks (within 128 KiB) and the backup GPT header.
pub(super) const TAIL_BYTES: u64 = 1 << 20;

/// Where a signature sits in a partition of a given size.
#[derive(Clone, Copy, Debug)]
enum At {
    /// Bytes from the start.
    Start(u64),
    /// Bytes before the end.
    End(u64),
    /// An MD RAID 0.90 superblock: the last 64 KiB-aligned 64 KiB.
    Md090,
    /// An MD RAID 1.0 superblock: 8 KiB before the end, 4 KiB aligned.
    Md10,
    /// A ZFS uberblock: any 1 KiB slot of the ring at 128 KiB of the first
    /// label.
    ZfsRing,
}

impl At {
    /// The offsets it names in a partition of `bytes`.
    fn offsets(self, bytes: u64) -> Vec<u64> {
        match self {
            Self::Start(offset) => vec![offset],
            Self::End(back) => bytes.checked_sub(back).into_iter().collect(),
            Self::Md090 => (bytes & !0xffff)
                .checked_sub(0x1_0000)
                .into_iter()
                .collect(),
            Self::Md10 => (bytes >> 9)
                .checked_sub(16)
                .map(|sector| (sector & !7) << 9)
                .into_iter()
                .collect(),
            Self::ZfsRing => (0..128).map(|slot| (128 << 10) + slot * 1024).collect(),
        }
    }
}

/// One signature: what it marks and its bytes.
#[derive(Clone, Copy, Debug)]
pub(super) struct Signature {
    /// How a message names what holds it.
    pub(super) name: &'static str,
    at: At,
    magic: &'static [u8],
}

const fn sig(name: &'static str, at: At, magic: &'static [u8]) -> Signature {
    Signature { name, at, magic }
}

/// A GUID partition table's header, at block 1 of 512- or 4096-byte blocks:
/// at a partition's start, the guest disk an install wrote.
pub(super) const GPT_HEADER: &[u8] = b"EFI PART";
pub(super) const GPT_HEADER_OFFSETS: [u64; 2] = [512, 4096];

/// Every signature probed, most specific first.
pub(super) const SIGNATURES: &[Signature] = &[
    sig("APFS", At::Start(32), b"NXSB"),
    sig("HFS+", At::Start(1024), b"H+\x00\x04"),
    sig("HFS+", At::Start(1024), b"HX\x00\x05"),
    sig("XFS", At::Start(0), b"XFSB"),
    sig("btrfs", At::Start(0x1_0040), b"_BHRfS_M"),
    sig("F2FS", At::Start(1024), &[0x10, 0x20, 0xf5, 0xf2]),
    sig(
        "bcachefs",
        At::Start(4096 + 24),
        &[
            0xc6, 0x85, 0x73, 0xf6, 0x4e, 0x1a, 0x45, 0xca, 0x82, 0x65, 0xf5, 0x7f, 0x48, 0xba,
            0x6d, 0x81,
        ],
    ),
    sig("ZFS", At::ZfsRing, &[0x0c, 0xb1, 0xba, 0, 0, 0, 0, 0]),
    sig("ZFS", At::ZfsRing, &[0, 0, 0, 0, 0, 0xba, 0xb1, 0x0c]),
    sig("ReiserFS", At::Start(0x1_0034), b"ReIsEr"),
    sig("ReiserFS", At::Start(0x2034), b"ReIsEr"),
    sig("JFS", At::Start(0x8000), b"JFS1"),
    sig("NTFS", At::Start(3), b"NTFS    "),
    sig("exFAT", At::Start(3), b"EXFAT   "),
    sig("BitLocker", At::Start(3), b"-FVE-FS-"),
    sig("ReFS", At::Start(3), b"ReFS\x00\x00\x00\x00"),
    sig("FAT", At::Start(54), b"FAT1"),
    sig("FAT", At::Start(82), b"FAT32   "),
    sig("LUKS encryption", At::Start(0), b"LUKS\xba\xbe"),
    sig("LUKS encryption", At::Start(16384), b"SKUL\xba\xbe"),
    sig("an LVM volume", At::Start(0), b"LABELONE"),
    sig("an LVM volume", At::Start(512), b"LABELONE"),
    sig("an LVM volume", At::Start(1024), b"LABELONE"),
    sig("an LVM volume", At::Start(1536), b"LABELONE"),
    sig("Linux RAID", At::Start(0), &[0xfc, 0x4e, 0x2b, 0xa9]),
    sig("Linux RAID", At::Start(4096), &[0xfc, 0x4e, 0x2b, 0xa9]),
    sig("Linux RAID", At::Md090, &[0xfc, 0x4e, 0x2b, 0xa9]),
    sig("Linux RAID", At::Md10, &[0xfc, 0x4e, 0x2b, 0xa9]),
    sig("Linux swap", At::Start(4086), b"SWAPSPACE2"),
    sig("Linux swap", At::Start(65526), b"SWAPSPACE2"),
    sig("ISO 9660", At::Start(32769), b"CD001"),
    sig("UDF", At::Start(32769), b"BEA01"),
    sig("ext2, ext3 or ext4", At::Start(1080), &[0x53, 0xef]),
    sig("a backup GPT", At::End(512), GPT_HEADER),
    sig("a backup GPT", At::End(4096), GPT_HEADER),
    sig("a boot sector", At::Start(510), &[0x55, 0xaa]),
];

/// What a partition holds.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Found {
    /// No signature, and zeros wherever one can be.
    Blank,
    /// A GUID partition table at its start: a disk of its own.
    Gpt,
    /// A known signature: what it marks.
    Known(&'static str),
    /// Bytes that are not zero, of no known kind.
    Unknown,
}

/// What the partition of `bytes` holds; `read(offset, length)` reads it.
pub(super) fn probe(
    bytes: u64,
    read: &mut dyn FnMut(u64, usize) -> Result<Vec<u8>>,
) -> Result<Found> {
    let head_end = bytes.min(HEAD_BYTES);
    let head = read(0, usize::try_from(head_end)?).context("reading its start")?;
    let tail_start = bytes.saturating_sub(TAIL_BYTES).max(head_end);
    let tail = read(tail_start, usize::try_from(bytes - tail_start)?).context("reading its end")?;
    // The bytes at `offset`, when read.
    let at = |offset: u64, length: usize| -> Option<&[u8]> {
        let end = offset.checked_add(length as u64)?;
        if end <= head_end {
            head.get(offset as usize..end as usize)
        } else if offset >= tail_start && end <= bytes {
            tail.get((offset - tail_start) as usize..(end - tail_start) as usize)
        } else {
            None
        }
    };
    if GPT_HEADER_OFFSETS
        .iter()
        .any(|&offset| at(offset, GPT_HEADER.len()) == Some(GPT_HEADER))
    {
        return Ok(Found::Gpt);
    }
    for signature in SIGNATURES {
        let length = signature.magic.len();
        if signature
            .at
            .offsets(bytes)
            .into_iter()
            .any(|offset| at(offset, length) == Some(signature.magic))
        {
            return Ok(Found::Known(signature.name));
        }
    }
    Ok(match head.iter().chain(&tail).all(|&byte| byte == 0) {
        true => Found::Blank,
        false => Found::Unknown,
    })
}

#[cfg(test)]
mod tests {
    use super::*;

    const BYTES: u64 = 16 << 20;

    /// A partition of `bytes` that holds `writes`, read as a host reads
    /// one, with every read recorded.
    fn found(bytes: u64, writes: &[(u64, &[u8])]) -> Result<Found> {
        let mut image = vec![0u8; bytes as usize];
        for &(offset, data) in writes {
            image[offset as usize..offset as usize + data.len()].copy_from_slice(data);
        }
        probe(bytes, &mut |offset, length| {
            let end = offset as usize + length;
            anyhow::ensure!(end <= image.len(), "read past the partition");
            Ok(image[offset as usize..end].to_vec())
        })
    }

    #[test]
    fn every_signature_is_found_at_its_offset_and_named() {
        for signature in SIGNATURES {
            for offset in signature.at.offsets(BYTES) {
                assert_eq!(
                    found(BYTES, &[(offset, signature.magic)]).unwrap(),
                    Found::Known(signature.name),
                    "{signature:?} at {offset:#x}"
                );
            }
        }
    }

    #[test]
    fn signatures_past_the_first_8_kib_are_found() {
        for (offset, magic, name) in [
            (0x1_0040, &b"_BHRfS_M"[..], "btrfs"),
            (0x1_0034, b"ReIsEr2Fs", "ReiserFS"),
            (0x8000, b"JFS1", "JFS"),
            // ZFS's first label starts with 8 KiB of zeros; its uberblocks
            // sit at 128 KiB, any slot of the ring.
            (
                (128 << 10) + 5 * 1024,
                &[0x0c, 0xb1, 0xba, 0, 0, 0, 0, 0],
                "ZFS",
            ),
            (16384, b"SKUL\xba\xbe", "LUKS encryption"),
            (32769, b"CD001", "ISO 9660"),
            (65526, b"SWAPSPACE2", "Linux swap"),
        ] {
            assert_eq!(
                found(BYTES, &[(offset, magic)]).unwrap(),
                Found::Known(name)
            );
        }
    }

    #[test]
    fn metadata_at_the_end_is_found() {
        // MD RAID 0.90 and 1.0 keep their superblock at the end, and a
        // partition that was once a whole disk keeps its backup GPT there.
        let md = [0xfc, 0x4e, 0x2b, 0xa9];
        let odd = BYTES + 3 * 4096 + 512;
        let md090 = (odd & !0xffff) - 0x1_0000;
        assert_eq!(
            found(odd, &[(md090, &md)]).unwrap(),
            Found::Known("Linux RAID")
        );
        let md10 = (((odd >> 9) - 16) & !7) << 9;
        assert_eq!(
            found(odd, &[(md10, &md)]).unwrap(),
            Found::Known("Linux RAID")
        );
        assert_eq!(
            found(BYTES, &[(BYTES - 512, GPT_HEADER)]).unwrap(),
            Found::Known("a backup GPT")
        );
        // Any byte left at the end is data.
        assert_eq!(
            found(BYTES, &[(BYTES - 100, b"x")]).unwrap(),
            Found::Unknown
        );
    }

    #[test]
    fn a_gpt_at_the_start_is_a_disk_of_its_own() {
        for offset in GPT_HEADER_OFFSETS {
            assert_eq!(found(BYTES, &[(offset, GPT_HEADER)]).unwrap(), Found::Gpt);
        }
    }

    #[test]
    fn only_zeros_where_signatures_sit_are_blank() {
        assert_eq!(found(BYTES, &[]).unwrap(), Found::Blank);
        // Within the bytes read and of no known kind: data.
        assert_eq!(
            found(BYTES, &[(HEAD_BYTES - 1, b"x")]).unwrap(),
            Found::Unknown
        );
        // A partition smaller than the bytes probed is read whole.
        assert_eq!(found(1 << 20, &[]).unwrap(), Found::Blank);
        assert_eq!(
            found(1 << 20, &[(0x8000, b"JFS1")]).unwrap(),
            Found::Known("JFS")
        );
    }

    #[test]
    fn a_partition_that_cannot_be_read_is_not_blank() {
        let error = probe(BYTES, &mut |_, _| anyhow::bail!("no medium")).unwrap_err();
        assert!(format!("{error:#}").contains("no medium"), "{error:#}");
    }
}
