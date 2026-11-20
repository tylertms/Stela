// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2018-2019 Realtek Corporation
// Copyright(c) 2019-2020 Realtek Corporation
//! Borrowed firmware layouts from pinned rtw88/mac.c and rtw89/fw.c.
//! This parses bytes, not vendor authentication, calibration or radio readiness.
use alloc::vec::Vec;
use core::ops::Range;

const MAX_IMAGE: usize = 16 * 1024 * 1024;
const LEGACY_HEADER: usize = 64;
const CHECKSUM: usize = 8;
const MFW_HEADER: usize = 16;
const MFW_ENTRY: usize = 16;
const AX_HEADER: usize = 32;
const AX_SECTION: usize = 16;
const MAX_SECTIONS: usize = 10;
const SECURITY_SECTION: u8 = 9;
const SECURITY_SIGNATURE: usize = 512;
const NORMAL: u8 = 1;
const NORMAL_CE: u8 = 5;
const FORMATTED_MSSC: u8 = 0xff;
const MSS_POOL_HEADER: usize = 32;
const MSS_POOL_DEFAULT_KEYSETS: usize = 8;
const MSS_SIGNATURE: [u8; 8] = *b"MSSKPOOL";
/// `FWDL_SECTION_PER_PKT_LEN`, the AX download chunk size.
const AX_PART_BYTES: usize = 2020;

/// Whether an `Rtw89::parse` may take a CE (type 5) image before a normal one.
/// RTL8852B sets `try_ce_fw`; RTL8852C does not.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Ce {
    Prefer,
    Never,
}

#[derive(Clone, Debug, Eq, PartialEq)]
pub enum ParseError {
    Truncated { offset: usize, bytes: usize },
    ResourceLimit,
    Length,
    AddressOverflow,
    Signature,
    UnsupportedHeader(u8),
    UnsupportedFormattedSecurity(u32),
    NoCompatibleFirmware { cut: u8 },
    EmptyContainer,
    TooManySections(u8),
    DynamicHeader,
}

#[derive(Debug)]
pub enum Layout<'a> {
    Rtw88(Rtw88<'a>),
    Rtw89(Rtw89<'a>),
}

fn bytes(data: &[u8], offset: usize, len: usize) -> Result<&[u8], ParseError> {
    let end = offset.checked_add(len).ok_or(ParseError::ResourceLimit)?;
    data.get(offset..end)
        .ok_or(ParseError::Truncated { offset, bytes: len })
}
fn le16(data: &[u8], offset: usize) -> Result<u16, ParseError> {
    bytes(data, offset, 2).map(|field| stela_driver_core::bytes::le16(field, 0))
}
fn le32(data: &[u8], offset: usize) -> Result<u32, ParseError> {
    bytes(data, offset, 4).map(|field| stela_driver_core::bytes::le32(field, 0))
}
fn image_limit(data: &[u8]) -> Result<(), ParseError> {
    if data.len() > MAX_IMAGE {
        Err(ParseError::ResourceLimit)
    } else {
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LegacyMemory {
    Dmem,
    Imem,
    Emem,
}

#[derive(Debug)]
pub struct LegacySection<'a> {
    pub memory: LegacyMemory,
    pub raw_address: u32,
    pub download_address: u32,
    /// Includes the eight checksum bytes which the real device must verify.
    pub data: &'a [u8],
    pub payload_bytes: usize,
    pub range: Range<usize>,
}

#[derive(Debug)]
pub struct Rtw88<'a> {
    pub header: &'a [u8; LEGACY_HEADER],
    pub version: u16,
    pub subversion: u8,
    pub subindex: u8,
    pub h2c_format: u16,
    pub sections: Vec<LegacySection<'a>>,
}
impl<'a> Rtw88<'a> {
    /// `__update_firmware_feature`: the word counts only under its signature.
    pub fn features(&self) -> u32 {
        let [a, b, c, d] = [0x0c, 0x0d, 0x0e, 0x0f].map(|at| self.header[at]);
        let features = u32::from_le_bytes([a, b, c, d]);
        if features & 1 != 0 { features } else { 0 }
    }

    pub fn parse(data: &'a [u8]) -> Result<Self, ParseError> {
        image_limit(data)?;
        let header: &[u8; LEGACY_HEADER] = bytes(data, 0, LEGACY_HEADER)?.try_into().unwrap();
        if le16(data, 0)? != 0x8822 {
            return Err(ParseError::Signature);
        }
        let mut sections = Vec::with_capacity(3);
        let mut offset = LEGACY_HEADER;
        for (memory, address, payload) in [
            (LegacyMemory::Dmem, 0x20, 0x24),
            (LegacyMemory::Imem, 0x3c, 0x30),
            (LegacyMemory::Emem, 0x38, 0x34),
        ] {
            let payload_bytes = le32(data, payload)? as usize;
            if memory == LegacyMemory::Emem && (header[0x18] & 0x10 == 0 || payload_bytes == 0) {
                continue;
            }
            let length = payload_bytes
                .checked_add(CHECKSUM)
                .ok_or(ParseError::ResourceLimit)?;
            let section = bytes(data, offset, length)?;
            let raw_address = le32(data, address)?;
            let download_address = raw_address & !(1 << 31);
            download_address
                .checked_add(length as u32)
                .ok_or(ParseError::AddressOverflow)?;
            sections.push(LegacySection {
                memory,
                raw_address,
                download_address,
                data: section,
                payload_bytes,
                range: offset..offset + length,
            });
            offset += length;
        }
        if offset != data.len() {
            return Err(ParseError::Length);
        }
        Ok(Self {
            header,
            version: le16(data, 4)?,
            subversion: header[6],
            subindex: header[7],
            h2c_format: le16(data, 0x1c)?,
            sections,
        })
    }
}

#[derive(Debug)]
pub struct Entry<'a> {
    pub descriptor: &'a [u8],
    pub cut: u8,
    pub kind: u8,
    pub mp: u8,
    pub range: Range<usize>,
    pub data: &'a [u8],
}

#[derive(Debug)]
pub struct Section<'a> {
    pub descriptor: &'a [u8],
    pub raw_address: u32,
    pub download_address: u32,
    pub kind: u8,
    pub redownload: bool,
    pub checksum: bool,
    /// Includes the checksum, if present; excludes any appended MSS signatures.
    pub data: &'a [u8],
    pub range: Range<usize>,
    pub mssc: u32,
    /// Opaque signatures kept without claiming authentication or a key choice.
    pub signatures: &'a [u8],
}

/// One section the AX download sends: signature bytes and any formatted MSS
/// pool are already skipped.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct AxSection<'a> {
    pub address: u32,
    pub kind: u8,
    pub checksum: bool,
    pub bytes: &'a [u8],
}

/// `__parse_formatted_mssc`: the byte length of the MSS pool that follows a
/// security section whose `mssc` is 0xFF. `content` starts at the section;
/// `len` is its length. The v0 header has no DSP checksum.
fn formatted_pool_len(content: &[u8], len: usize) -> Option<usize> {
    let pool = content.get(len..)?;
    let header = pool.get(..MSS_POOL_HEADER)?;
    if header[..8] != MSS_SIGNATURE || header[20] != 0 {
        return None;
    }
    let defen = header[16] != 0;
    let (devices, keypairs) = (usize::from(header[21]), usize::from(le16(header, 22).ok()?));
    let (customers, keys) = (
        usize::from(le16(header, 24).ok()?),
        usize::from(le16(header, 26).ok()?),
    );
    let mut table = keys.checked_mul(customers)?.checked_mul(devices)? >> 3;
    if defen {
        table += MSS_POOL_DEFAULT_KEYSETS;
    }
    if table.checked_add(MSS_POOL_HEADER)? != le32(header, 12).ok()? as usize {
        return None;
    }
    let mut sign = usize::from(le16(content, 60).ok()? >> 2);
    if sign == 0 {
        sign = SECURITY_SIGNATURE;
    }
    let total = MSS_POOL_HEADER
        .checked_add(table)?
        .checked_add(keypairs.checked_mul(sign)?)?;
    (pool.len() >= total).then_some(total)
}

#[derive(Debug)]
pub struct AxImage<'a> {
    pub header: &'a [u8],
    /// The fixed header and every section descriptor, without the dynamic part.
    base_header: &'a [u8],
    pub dynamic_header: &'a [u8],
    pub version: [u8; 4],
    pub commit_id: u32,
    pub idmem_share_mode: u8,
    /// The AX donor uses 2020, independent of the stored part-size field.
    pub part_bytes: usize,
    pub sections: Vec<Section<'a>>,
}
impl<'a> AxImage<'a> {
    fn parse(data: &'a [u8]) -> Result<Self, ParseError> {
        let fixed = bytes(data, 0, AX_HEADER)?;
        let header_version = (le32(data, 12)? >> 24) as u8;
        if header_version != 0 {
            return Err(ParseError::UnsupportedHeader(header_version));
        }
        let count = ((le32(data, 24)? >> 8) & 255) as u8;
        if usize::from(count) > MAX_SECTIONS {
            return Err(ParseError::TooManySections(count));
        }
        let base = AX_HEADER + usize::from(count) * AX_SECTION;
        bytes(data, 0, base)?;
        let w7 = le32(data, 28)?;
        let header_bytes = if w7 & (1 << 16) != 0 {
            ((le32(data, 12)? >> 16) & 255) as usize
        } else {
            base
        };
        if header_bytes < base {
            return Err(ParseError::DynamicHeader);
        }
        let dynamic_header = bytes(data, base, header_bytes - base)?;
        if w7 & (1 << 16) != 0
            && (dynamic_header.len() < 4
                || le32(dynamic_header, 0)? as usize != dynamic_header.len())
        {
            return Err(ParseError::DynamicHeader);
        }
        let mut offset = header_bytes;
        let mut sections = Vec::with_capacity(usize::from(count));
        for index in 0..usize::from(count) {
            let at = AX_HEADER + index * AX_SECTION;
            let descriptor = bytes(data, at, AX_SECTION)?;
            let raw_address = le32(descriptor, 0)?;
            let download_address = raw_address & 0x1fff_ffff;
            let flags = le32(descriptor, 4)?;
            let kind = ((flags >> 24) & 15) as u8;
            let checksum = flags & (1 << 28) != 0;
            let length = (flags & 0xff_ffff) as usize + if checksum { CHECKSUM } else { 0 };
            let mssc = if kind == SECURITY_SECTION {
                le32(descriptor, 8)?
            } else {
                0
            };
            let section = bytes(data, offset, length)?;
            let signature_bytes =
                if kind == SECURITY_SECTION && mssc & 255 == u32::from(FORMATTED_MSSC) {
                    formatted_pool_len(&data[offset..], length)
                        .ok_or(ParseError::UnsupportedFormattedSecurity(mssc))?
                } else {
                    (mssc as usize)
                        .checked_mul(SECURITY_SIGNATURE)
                        .ok_or(ParseError::ResourceLimit)?
                };
            let signatures = bytes(data, offset + length, signature_bytes)?;
            download_address
                .checked_add(length as u32)
                .ok_or(ParseError::AddressOverflow)?;
            sections.push(Section {
                descriptor,
                raw_address,
                download_address,
                kind,
                redownload: flags & (1 << 29) != 0,
                checksum,
                data: section,
                range: offset..offset + length,
                mssc,
                signatures,
            });
            offset += length + signature_bytes;
        }
        if offset != data.len() {
            return Err(ParseError::Length);
        }
        Ok(Self {
            header: fixed,
            base_header: &data[..base],
            dynamic_header,
            version: fixed[4..8].try_into().unwrap(),
            commit_id: le32(data, 8)?,
            idmem_share_mode: ((w7 >> 18) & 15) as u8,
            part_bytes: AX_PART_BYTES,
            sections,
        })
    }
}

impl<'a> AxImage<'a> {
    /// The header packet `__rtw89_fw_download_hdr` sends: the fixed header and
    /// section descriptors, without the dynamic header, with W7[15:0] set to
    /// the 2020-byte chunk size (`__rtw89_fw_download_tweak_hdr_v0`).
    pub fn header_packet(&self) -> Vec<u8> {
        let mut packet = self.base_header.to_vec();
        packet[28..30].copy_from_slice(&(AX_PART_BYTES as u16).to_le_bytes());
        packet
    }

    /// The sections the download sends, in order. A formatted-MSS firmware
    /// keeps only its first security section (`fw.c:347-357`).
    pub fn sections(&self) -> Result<Vec<AxSection<'a>>, ParseError> {
        let mut security_seen = false;
        let mut kept = Vec::with_capacity(self.sections.len());
        for section in &self.sections {
            if section.kind == SECURITY_SECTION {
                if section.mssc & 255 == u32::from(FORMATTED_MSSC) && security_seen {
                    continue;
                }
                security_seen = true;
            }
            kept.push(AxSection {
                address: section.download_address,
                kind: section.kind,
                checksum: section.checksum,
                bytes: section.data,
            });
        }
        Ok(kept)
    }

    /// (major, minor, sub, idx) from header W1.
    pub fn version(&self) -> (u8, u8, u8, u8) {
        let [major, minor, sub, idx] = self.version;
        (major, minor, sub, idx)
    }

    pub fn idmem_share_mode(&self) -> u8 {
        self.idmem_share_mode
    }
}

#[derive(Debug)]
pub struct Rtw89<'a> {
    pub container_header: Option<&'a [u8]>,
    /// Includes unselected WOW/log/test records; unknown kinds are held.
    pub entries: Vec<Entry<'a>>,
    pub selected: usize,
    pub image: AxImage<'a>,
    /// Complete trailing firmware elements, including tables not yet consumed.
    pub trailing_elements: &'a [u8],
}
impl<'a> Rtw89<'a> {
    pub fn parse(data: &'a [u8], observed_cut: u8, ce: Ce) -> Result<Self, ParseError> {
        image_limit(data)?;
        let mut entries = Vec::new();
        let (container_header, end) = if data.first() == Some(&0xff) {
            let container_header = Some(bytes(data, 0, MFW_HEADER)?);
            let count = usize::from(data[1]);
            if count == 0 {
                return Err(ParseError::EmptyContainer);
            }
            let table_end = MFW_HEADER + count * MFW_ENTRY;
            bytes(data, 0, table_end)?;
            let mut max_end = table_end;
            for index in 0..count {
                let descriptor = bytes(data, MFW_HEADER + index * MFW_ENTRY, MFW_ENTRY)?;
                let offset = le32(descriptor, 4)? as usize;
                let length = le32(descriptor, 8)? as usize;
                if offset < table_end {
                    return Err(ParseError::Length);
                }
                let content = bytes(data, offset, length)?;
                max_end = max_end.max(offset + length);
                entries.push(Entry {
                    descriptor,
                    cut: descriptor[0],
                    kind: descriptor[1],
                    mp: descriptor[2],
                    range: offset..offset + length,
                    data: content,
                });
            }
            (container_header, max_end)
        } else {
            // The donor permits a legacy single image for normal type only.
            entries.push(Entry {
                descriptor: &[],
                cut: 0,
                kind: NORMAL,
                mp: 0,
                range: 0..data.len(),
                data,
            });
            (None, data.len())
        };
        let best = |kind| {
            let mut selected = None;
            for (index, entry) in entries.iter().enumerate() {
                if entry.kind == kind
                    && entry.mp == 0
                    && entry.cut <= observed_cut
                    && selected.is_none_or(|old: usize| entries[old].cut < entry.cut)
                {
                    selected = Some(index);
                }
            }
            selected
        };
        // try_ce_fw (RTL8852B) prefers CE, then ordinary normal firmware.
        let selected = match ce {
            Ce::Prefer => best(NORMAL_CE),
            Ce::Never => None,
        }
        .or_else(|| best(NORMAL))
        .ok_or(ParseError::NoCompatibleFirmware { cut: observed_cut })?;
        let image = AxImage::parse(entries[selected].data)?;
        Ok(Self {
            container_header,
            entries,
            selected,
            image,
            trailing_elements: &data[end..],
        })
    }
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    pub(crate) fn legacy_fixture() -> Vec<u8> {
        let mut b = vec![0u8; 64 + 12 + 16 + 10];
        b[..2].copy_from_slice(&0x8822u16.to_le_bytes());
        b[4..6].copy_from_slice(&9u16.to_le_bytes());
        b[6] = 9;
        b[7] = 15;
        b[0x18] = 0x10;
        for (at, value) in [
            (0x20, 0x80200000u32),
            (0x24, 4),
            (0x3c, 0x80030000),
            (0x30, 8),
            (0x38, 0x80100000),
            (0x34, 2),
        ] {
            b[at..at + 4].copy_from_slice(&value.to_le_bytes());
        }
        for (i, x) in b[64..].iter_mut().enumerate() {
            *x = i as u8;
        }
        b
    }
    fn ax_fixture() -> Vec<u8> {
        let mut b = vec![0u8; 32 + 32 + 12 + 512 + 3];
        b[4..8].copy_from_slice(&[1, 2, 3, 4]);
        b[24..28].copy_from_slice(&(2u32 << 8).to_le_bytes());
        b[32..36].copy_from_slice(&0xb8000000u32.to_le_bytes());
        b[36..40].copy_from_slice(&(4u32 | 1 << 28 | 1 << 29 | 9 << 24).to_le_bytes());
        b[40..44].copy_from_slice(&1u32.to_le_bytes());
        b[48..52].copy_from_slice(&0xb9000000u32.to_le_bytes());
        b[52..56].copy_from_slice(&(3u32 | 2 << 24).to_le_bytes());
        b
    }
    fn container_fixture() -> Vec<u8> {
        let image = ax_fixture();
        let mut b = vec![0u8; 16 + 4 * 16];
        b[0] = 255;
        b[1] = 4;
        for (i, (cut, kind, mp)) in [(2, 1, 0), (1, 5, 0), (2, 5, 1), (0, 255, 0)]
            .into_iter()
            .enumerate()
        {
            let at = 16 + i * 16;
            let offset = b.len();
            b[at] = cut;
            b[at + 1] = kind;
            b[at + 2] = mp;
            b[at + 4..at + 8].copy_from_slice(&(offset as u32).to_le_bytes());
            b[at + 8..at + 12].copy_from_slice(&(image.len() as u32).to_le_bytes());
            b.extend_from_slice(&image);
        }
        b.extend_from_slice(&[0xa5; 17]);
        b
    }
    /// A security section whose key-signature length field (bytes 60..62) is
    /// zero, which `__parse_formatted_mssc` reads as 512.
    fn security_content() -> [u8; 64] {
        let mut content = [0x33; 64];
        content[60..62].fill(0);
        content
    }
    /// A v0 image of type 2 (4 bytes), type 1 (3 bytes) and `security`
    /// 64-byte security sections with `mssc` 0xFF, each followed by a
    /// formatted MSS pool of one table byte and two 512-byte keys.
    fn formatted_fixture(security: usize) -> Vec<u8> {
        let count = 2 + security;
        let mut b = vec![0u8; 32 + count * 16];
        b[4..8].copy_from_slice(&[0, 29, 29, 18]);
        b[24..28].copy_from_slice(&((count as u32) << 8).to_le_bytes());
        b[28..32].copy_from_slice(&(0x1234u32 | 1 << 18).to_le_bytes());
        let mut rows = vec![
            (0xa000_0000u32, 4u32 | 2 << 24, 0u32),
            (0xa100_0000, 3 | 1 << 24, 0),
        ];
        for index in 0..security {
            rows.push((0xa200_0000 + index as u32 * 0x1000, 64 | 9 << 24, 0xff));
        }
        for (index, (address, flags, mssc)) in rows.into_iter().enumerate() {
            let at = 32 + index * 16;
            b[at..at + 4].copy_from_slice(&address.to_le_bytes());
            b[at + 4..at + 8].copy_from_slice(&flags.to_le_bytes());
            b[at + 8..at + 12].copy_from_slice(&mssc.to_le_bytes());
        }
        b.extend_from_slice(&[0x11; 4]);
        b.extend_from_slice(&[0x22; 3]);
        for _ in 0..security {
            b.extend_from_slice(&security_content());
            let mut pool = vec![0xee; 32 + 1 + 2 * 512];
            pool[..8].copy_from_slice(b"MSSKPOOL");
            pool[8..12].copy_from_slice(&32u32.to_le_bytes());
            pool[12..16].copy_from_slice(&33u32.to_le_bytes());
            pool[16..20].fill(0);
            pool[20] = 0;
            pool[21] = 1;
            pool[22..24].copy_from_slice(&2u16.to_le_bytes());
            pool[24..26].copy_from_slice(&1u16.to_le_bytes());
            pool[26..28].copy_from_slice(&8u16.to_le_bytes());
            b.extend_from_slice(&pool);
        }
        b
    }
    /// An MFW container of `(cv, type)` entries, each holding `ax_fixture`.
    fn mfw(entries: &[(u8, u8)]) -> Vec<u8> {
        let image = ax_fixture();
        let mut b = vec![0u8; 16 + entries.len() * 16];
        b[0] = 255;
        b[1] = entries.len() as u8;
        for (i, (cut, kind)) in entries.iter().enumerate() {
            let at = 16 + i * 16;
            let offset = b.len();
            b[at] = *cut;
            b[at + 1] = *kind;
            b[at + 4..at + 8].copy_from_slice(&(offset as u32).to_le_bytes());
            b[at + 8..at + 12].copy_from_slice(&(image.len() as u32).to_le_bytes());
            b.extend_from_slice(&image);
        }
        b
    }
    #[test]
    fn selects_largest_cv_not_above_cut() {
        let b = mfw(&[(0, 1), (1, 1), (2, 1), (1, 3)]);
        for (cut, cv) in [(0, 0), (1, 1), (2, 2), (3, 2), (15, 2)] {
            let p = Rtw89::parse(&b, cut, Ce::Never).unwrap();
            assert_eq!(p.entries[p.selected].cut, cv, "cut {cut}");
            assert_eq!(p.entries[p.selected].kind, 1);
        }
    }
    #[test]
    fn ce_never_ignores_normal_ce_entries() {
        let b = mfw(&[(0, 5)]);
        assert_eq!(
            Rtw89::parse(&b, 0, Ce::Never).unwrap_err(),
            ParseError::NoCompatibleFirmware { cut: 0 }
        );
        assert_eq!(Rtw89::parse(&b, 0, Ce::Prefer).unwrap().selected, 0);
        let b = mfw(&[(0, 1), (0, 5)]);
        assert_eq!(Rtw89::parse(&b, 0, Ce::Never).unwrap().selected, 0);
        assert_eq!(Rtw89::parse(&b, 0, Ce::Prefer).unwrap().selected, 1);
    }
    #[test]
    fn sections_skip_formatted_mss_pool() {
        let b = formatted_fixture(1);
        let image = AxImage::parse(&b).unwrap();
        let sections = image.sections().unwrap();
        assert_eq!(sections.len(), 3);
        assert_eq!(
            sections
                .iter()
                .map(|s| (s.address, s.kind))
                .collect::<Vec<_>>(),
            [(0x0000_0000, 2), (0x0100_0000, 1), (0x0200_0000, 9)]
        );
        assert_eq!(sections[0].bytes, &[0x11; 4]);
        assert_eq!(sections[1].bytes, &[0x22; 3]);
        assert_eq!(sections[2].bytes, &security_content()[..]);
        assert!(!sections[2].checksum);
        assert_eq!(image.sections[2].signatures.len(), 32 + 1 + 1024);
        assert_eq!(image.version(), (0, 29, 29, 18));
        assert_eq!(image.idmem_share_mode(), 1);
        // A pool that is not what the header promises is not skipped blindly.
        let mut bad = b.clone();
        let at = 32 + 48 + 4 + 3 + 64;
        bad[at] = b'X';
        assert_eq!(
            AxImage::parse(&bad).unwrap_err(),
            ParseError::UnsupportedFormattedSecurity(255)
        );
        assert!(AxImage::parse(&b[..b.len() - 1]).is_err());
    }
    #[test]
    fn only_the_first_formatted_security_section_is_kept() {
        let b = formatted_fixture(2);
        let image = AxImage::parse(&b).unwrap();
        assert_eq!(image.sections.len(), 4);
        let kept = image.sections().unwrap();
        assert_eq!(kept.len(), 3);
        assert_eq!(kept[2].address, 0x0200_0000);
    }
    #[test]
    fn header_packet_patches_chunk_size() {
        let mut b = formatted_fixture(1);
        let packet = AxImage::parse(&b).unwrap().header_packet();
        assert_eq!(packet.len(), 32 + 16 * 3);
        assert_eq!(le16(&packet, 28).unwrap(), 2020);
        // The rest of W7 and every other byte are untouched.
        assert_eq!(le32(&packet, 28).unwrap() >> 16, 1 << 2);
        assert_eq!(packet[..28], b[..28]);
        assert_eq!(packet[32..], b[32..80]);
        // A dynamic header is removed from the packet.
        b[28..32].copy_from_slice(&(1u32 << 16).to_le_bytes());
        b[14] = 84;
        b.splice(80..80, [4u8, 0, 0, 0]);
        let image = AxImage::parse(&b).unwrap();
        assert_eq!(image.dynamic_header, &[4, 0, 0, 0]);
        let packet = image.header_packet();
        assert_eq!(packet.len(), 32 + 16 * 3);
        assert_eq!(le32(&packet, 28).unwrap(), 1 << 16 | 2020);
    }
    #[test]
    fn legacy_layout_borrows_exact_checksum_inclusive_memory_regions() {
        let b = legacy_fixture();
        let p = Rtw88::parse(&b).unwrap();
        assert_eq!(p.sections.len(), 3);
        assert_eq!(p.version, 9);
        for (s, (start, len, addr)) in
            p.sections
                .iter()
                .zip([(64, 12, 0x200000), (76, 16, 0x30000), (92, 10, 0x100000)])
        {
            assert_eq!(s.range, start..start + len);
            assert_eq!(s.data, &b[start..start + len]);
            assert_eq!(s.data.as_ptr(), b[start..].as_ptr());
            assert_eq!(s.download_address, addr);
        }
        for cut in 0..b.len() {
            assert!(Rtw88::parse(&b[..cut]).is_err());
        }
        let mut trailing = b.clone();
        trailing.push(0);
        assert_eq!(Rtw88::parse(&trailing).unwrap_err(), ParseError::Length);
        let mut huge = b.clone();
        huge[0x24..0x28].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Rtw88::parse(&huge).is_err());
        let mut no_emem = b[..92].to_vec();
        no_emem[0x18] = 0;
        assert_eq!(Rtw88::parse(&no_emem).unwrap().sections.len(), 2);
    }
    #[test]
    fn ax_sections_keep_checksum_security_signatures_flags_and_raw_addresses() {
        let b = ax_fixture();
        let p = AxImage::parse(&b).unwrap();
        assert_eq!(p.sections.len(), 2);
        assert_eq!(p.part_bytes, 2020);
        let s = &p.sections[0];
        assert!(s.checksum && s.redownload);
        assert_eq!(s.kind, 9);
        assert_eq!(s.raw_address, 0xb8000000);
        assert_eq!(s.download_address, 0x18000000);
        assert_eq!(s.data.len(), 12);
        assert_eq!(s.signatures.len(), 512);
        assert_eq!(s.mssc, 1);
        for n in 0..b.len() {
            assert!(AxImage::parse(&b[..n]).is_err());
        }
        let mut f = b.clone();
        f[40..44].copy_from_slice(&255u32.to_le_bytes());
        assert_eq!(
            AxImage::parse(&f).unwrap_err(),
            ParseError::UnsupportedFormattedSecurity(255)
        );
        f = b.clone();
        f[15] = 1;
        assert_eq!(
            AxImage::parse(&f).unwrap_err(),
            ParseError::UnsupportedHeader(1)
        );
        f = b.clone();
        f[25] = 11;
        assert_eq!(
            AxImage::parse(&f).unwrap_err(),
            ParseError::TooManySections(11)
        );
    }
    #[test]
    fn closest_production_ce_cut_wins_independently_of_entry_order() {
        let b = container_fixture();
        for cut in 1..16 {
            let p = Rtw89::parse(&b, cut, Ce::Prefer).unwrap();
            assert_eq!(p.selected, 1);
            assert_eq!(p.entries.len(), 4);
            assert_eq!(p.trailing_elements, &[0xa5; 17]);
            assert_eq!(p.entries[3].kind, 255);
            assert_eq!(p.entries[2].mp, 1);
        }
        assert_eq!(
            Rtw89::parse(&b, 0, Ce::Prefer).unwrap_err(),
            ParseError::NoCompatibleFirmware { cut: 0 }
        );
        let mut f = b.clone();
        f[16 + 16 + 1] = 3;
        let p = Rtw89::parse(&f, 2, Ce::Prefer).unwrap();
        assert_eq!(p.selected, 0);
        let mut f = b.clone();
        f[16 + 4..16 + 8].copy_from_slice(&u32::MAX.to_le_bytes());
        assert!(Rtw89::parse(&f, 2, Ce::Prefer).is_err());
        let mut f = b;
        f[1] = 0;
        assert_eq!(
            Rtw89::parse(&f, 2, Ce::Prefer).unwrap_err(),
            ParseError::EmptyContainer
        );
    }
    #[test]
    fn dynamic_header_length_is_checked_before_any_section_payload() {
        let b = ax_fixture();
        let mut f = b.clone();
        f[28..32].copy_from_slice(&(1u32 << 16).to_le_bytes());
        f[14] = 68;
        f.splice(64..64, [4u8, 0, 0, 0]);
        assert_eq!(AxImage::parse(&f).unwrap().dynamic_header, &[4, 0, 0, 0]);
        f[64] = 8;
        assert_eq!(AxImage::parse(&f).unwrap_err(), ParseError::DynamicHeader);
        f[14] = 32;
        assert_eq!(AxImage::parse(&f).unwrap_err(), ParseError::DynamicHeader);
    }
    #[test]
    #[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
    fn vendor_layouts_are_borrowed_complete_and_versioned() {
        extern crate std;
        let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
        let b = std::fs::read(root.join("rtw88/rtw8822c_fw.bin")).unwrap();
        let p = Rtw88::parse(&b).unwrap();
        assert_eq!(
            (p.version, p.subversion, p.subindex, p.h2c_format),
            (9, 9, 15, 15)
        );
        assert_eq!(
            p.sections.iter().map(|s| s.data.len()).collect::<Vec<_>>(),
            [19336, 54072, 129128]
        );
        let b = std::fs::read(root.join("rtw89/rtw8852b_fw-2.bin")).unwrap();
        for cut in 1..16 {
            let p = Rtw89::parse(&b, cut, Ce::Prefer).unwrap();
            assert_eq!(p.entries.len(), 5);
            assert_eq!(p.entries[p.selected].cut, cut.min(2));
            assert_eq!(p.entries[p.selected].kind, 5);
            assert_eq!(p.image.sections.len(), 3);
            assert_eq!(p.image.dynamic_header.len(), 64);
            assert!(!p.trailing_elements.is_empty());
            assert_eq!(p.image.version, [0, 29, 29, 18]);
        }
    }

    #[test]
    #[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES"]
    fn real_8852c_fw_2_matches_spec() {
        extern crate std;
        let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
        let b = std::fs::read(root.join("rtw89/rtw8852c_fw-2.bin")).unwrap();
        assert_eq!(b.len(), 2_644_326);
        let p = Rtw89::parse(&b, 0, Ce::Never).unwrap();
        let entry = &p.entries[p.selected];
        assert_eq!(
            (entry.cut, entry.kind, entry.range.start, entry.data.len()),
            (0, 1, 128, 365_736)
        );
        let sections = p.image.sections().unwrap();
        assert_eq!(
            sections
                .iter()
                .map(|s| (s.address, s.bytes.len(), s.kind))
                .collect::<Vec<_>>(),
            [
                (0x0010_0000, 348_480, 2),
                (0x0021_b000, 14_832, 1),
                (0x0023_7c00, 960, 9)
            ]
        );
        assert_eq!(p.image.idmem_share_mode(), 1);
        assert!(!p.image.dynamic_header.is_empty());
        for (cut, shift, size) in [
            (1, 365_864, 338_128),
            (2, 703_992, 334_064),
            (3, 703_992, 334_064),
        ] {
            let p = Rtw89::parse(&b, cut, Ce::Never).unwrap();
            let entry = &p.entries[p.selected];
            assert_eq!(
                (entry.range.start, entry.data.len()),
                (shift, size),
                "cut {cut}"
            );
            assert_eq!(p.image.header_packet().len(), 32 + 16 * 3);
        }
    }

    #[test]
    #[ignore = "oracle: STELA_REALTEK_WIFI_IMAGES, STELA_REALTEK_WIFI_ORACLE"]
    fn unchanged_linux_selection_and_complete_payloads_agree() {
        extern crate std;
        fn word(trace: &[u8], at: &mut usize) -> u32 {
            let value = le32(trace, *at).unwrap();
            *at += 4;
            value
        }
        let root = std::path::PathBuf::from(std::env::var("STELA_REALTEK_WIFI_IMAGES").unwrap());
        let trace = std::fs::read(std::env::var("STELA_REALTEK_WIFI_ORACLE").unwrap()).unwrap();
        let mut at = 0;
        assert_eq!(word(&trace, &mut at), 0x38385752);
        let legacy = std::fs::read(root.join("rtw88/rtw8822c_fw.bin")).unwrap();
        let parsed = Rtw88::parse(&legacy).unwrap();
        for section in parsed.sections {
            assert_eq!(word(&trace, &mut at), section.download_address);
            assert_eq!(word(&trace, &mut at) as usize, section.data.len());
            assert_eq!(word(&trace, &mut at) as usize, section.range.start);
            assert_eq!(&trace[at..at + section.data.len()], section.data);
            at += section.data.len();
        }
        assert_eq!(word(&trace, &mut at), 0x39385752);
        let ax = std::fs::read(root.join("rtw89/rtw8852b_fw-2.bin")).unwrap();
        for cut in 0..16 {
            assert_eq!(word(&trace, &mut at), u32::from(cut));
            let status = word(&trace, &mut at);
            let parsed = Rtw89::parse(&ax, cut, Ce::Prefer);
            if status != 0 {
                assert_eq!(
                    parsed.unwrap_err(),
                    ParseError::NoCompatibleFirmware { cut }
                );
                continue;
            }
            let parsed = parsed.unwrap();
            let selected = &parsed.entries[parsed.selected];
            assert_eq!(word(&trace, &mut at), u32::from(selected.kind));
            assert_eq!(word(&trace, &mut at) as usize, selected.range.start);
            assert_eq!(word(&trace, &mut at) as usize, selected.data.len());
            assert_eq!(
                word(&trace, &mut at) as usize,
                32 + 16 * parsed.image.sections.len() + parsed.image.dynamic_header.len()
            );
            assert_eq!(
                word(&trace, &mut at) as usize,
                parsed.image.dynamic_header.len()
            );
            assert_eq!(word(&trace, &mut at) as usize, parsed.image.part_bytes);
            assert_eq!(word(&trace, &mut at) as usize, parsed.image.sections.len());
            for section in parsed.image.sections {
                assert_eq!(word(&trace, &mut at), u32::from(section.kind));
                assert_eq!(word(&trace, &mut at), section.download_address);
                assert_eq!(word(&trace, &mut at) as usize, section.data.len());
                assert_eq!(word(&trace, &mut at), u32::from(section.redownload));
                assert_eq!(word(&trace, &mut at), section.mssc);
                assert_eq!(word(&trace, &mut at) as usize, section.range.start);
                assert_eq!(&trace[at..at + section.data.len()], section.data);
                at += section.data.len();
                assert_eq!(
                    &trace[at..at + section.signatures.len()],
                    section.signatures
                );
                at += section.signatures.len();
            }
        }
        assert_eq!(at, trace.len());
    }
}
