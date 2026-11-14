// SPDX-License-Identifier: GPL-2.0-or-later
//! Physical-host SMBIOS observations, distinct from translated guest identity.
//! Layouts: DMTF DSP0134 3.8.0 §§5.2,6.1,7.2. No source mapping permission is
//! granted by this pure parser; boot must copy readable pre-EBS data.
use super::{Image, Provenance, Provider, Requirement, Resource, Target};
use crate::bytes::{le16, le32};
use alloc::{format, string::String};
use stela_crypto::{Sha256, digest::Digest};

pub const MAX_ENTRY_BYTES: usize = 255;
/// Native collection bound, not a maximum imposed by the SMBIOS specification.
pub const MAX_TABLE_BYTES: usize = 1024 * 1024;
pub const NAME: &str = "host/smbios/entry-and-table.bin";
const MEMBER: &str = "SMBIOS entry point + structure table";
const LICENSE: &str = "LicenseRef-Local-UEFI-Observation";
pub const TARGET: Target<'static> = Target {
    driver: "host-smbios",
    hardware: "physical-host",
    revision: "entry-and-table-v1",
    board: None,
};
pub const REQUIREMENT: Requirement<'static> = Requirement {
    name: NAME,
    target: TARGET,
    minimum_bytes: 24,
};
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Format {
    Smbios2,
    Smbios3,
}
impl Format {
    pub fn guid(self) -> &'static str {
        match self {
            Self::Smbios2 => "eb9d2d31-2d88-11d3-9a16-0090273fc14d",
            Self::Smbios3 => "f2fd1544-9794-4a2c-992e-e5bbcf20e394",
        }
    }
    pub fn source(self) -> &'static str {
        match self {
            Self::Smbios2 => "UEFI SMBIOS configuration table eb9d2d31-2d88-11d3-9a16-0090273fc14d",
            Self::Smbios3 => "UEFI SMBIOS configuration table f2fd1544-9794-4a2c-992e-e5bbcf20e394",
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    NotCaptured,
    NoConfigurationTable,
    AmbiguousConfiguration,
    MemoryMapUnavailable,
    UnreadableRange,
    AddressOverflow,
    NoncanonicalAddress,
    EntryAnchor,
    EntryLength,
    EntryChecksum,
    EntryRevision,
    EntryVersion,
    IntermediateChecksum,
    TableBound { bytes: usize },
    TableLength,
    StructureLength,
    UnterminatedStrings,
    InvalidStringIndex,
    MissingType1,
    DuplicateType1,
    DuplicateHandle,
    MissingEnd,
    StructureCount,
    MaximumStructureSize,
    ChangedEntry,
    AllocationFailed,
    ImageBinding,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Observation<'a> {
    NotObserved,
    Absent,
    Value(&'a [u8]),
}
/// Raw parsed observations only, not a physical-host/policy credential. An
/// arbitrary Provider can replay identical bytes and provenance. Native unsafe
/// admission must bind this boot's capture owner and Function.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SystemIdentity<'a> {
    pub system_vendor: Observation<'a>,
    pub product_name: Observation<'a>,
    pub product_version: Observation<'a>,
    /// The first type-2 baseboard's manufacturer, the structure Linux's
    /// `dmi_decode` saves as DMI_BOARD_VENDOR. Not observed without one.
    pub board_vendor: Observation<'a>,
    /// That baseboard's product, Linux's DMI_BOARD_NAME.
    pub board_name: Observation<'a>,
}
impl Default for SystemIdentity<'_> {
    fn default() -> Self {
        Self {
            system_vendor: Observation::NotObserved,
            product_name: Observation::NotObserved,
            product_version: Observation::NotObserved,
            board_vendor: Observation::NotObserved,
            board_name: Observation::NotObserved,
        }
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Entry {
    pub format: Format,
    pub entry_bytes: usize,
    pub table_address: u64,
    pub table_bytes: usize,
    pub major: u8,
    pub minor: u8,
    pub docrev: u8,
    pub structures: Option<u16>,
    pub maximum_structure_bytes: Option<u16>,
}
fn checksum(bytes: &[u8]) -> bool {
    bytes.iter().fold(0u8, |sum, b| sum.wrapping_add(*b)) == 0
}
/// The first eight bytes are enough to bound the full entry-point read.
pub fn entry_length(prefix: &[u8], format: Format) -> Result<usize, Error> {
    if prefix.len() < 8 {
        return Err(Error::EntryLength);
    }
    let (anchor, offset, minimum): (&[u8], usize, usize) = match format {
        Format::Smbios2 => (b"_SM_", 5, 31),
        Format::Smbios3 => (b"_SM3_", 6, 24),
    };
    if !prefix.starts_with(anchor) {
        return Err(Error::EntryAnchor);
    }
    let length = usize::from(prefix[offset]);
    if !(minimum..=MAX_ENTRY_BYTES).contains(&length) {
        return Err(Error::EntryLength);
    }
    Ok(length)
}
pub fn entry(bytes: &[u8], format: Format) -> Result<Entry, Error> {
    let length = entry_length(bytes, format)?;
    if bytes.len() != length {
        return Err(Error::EntryLength);
    }
    if !checksum(bytes) {
        return Err(Error::EntryChecksum);
    }
    let entry = match format {
        Format::Smbios3 => {
            if bytes[10] != 1 || bytes[11] != 0 {
                return Err(Error::EntryRevision);
            }
            if bytes[7] < 3 {
                return Err(Error::EntryVersion);
            }
            Entry {
                format,
                entry_bytes: length,
                table_address: u64::from_le_bytes(bytes[16..24].try_into().unwrap()),
                table_bytes: le32(bytes, 12) as usize,
                major: bytes[7],
                minor: bytes[8],
                docrev: bytes[9],
                structures: None,
                maximum_structure_bytes: None,
            }
        }
        Format::Smbios2 => {
            if bytes[10] != 0 {
                return Err(Error::EntryRevision);
            }
            if bytes[6] < 2 || (bytes[6] == 2 && bytes[7] < 2) {
                return Err(Error::EntryVersion);
            }
            if &bytes[16..21] != b"_DMI_" || !checksum(&bytes[16..31]) {
                return Err(Error::IntermediateChecksum);
            }
            Entry {
                format,
                entry_bytes: length,
                table_address: u64::from(le32(bytes, 24)),
                table_bytes: usize::from(le16(bytes, 22)),
                major: bytes[6],
                minor: bytes[7],
                docrev: 0,
                structures: Some(le16(bytes, 28)),
                maximum_structure_bytes: Some(le16(bytes, 8)),
            }
        }
    };
    if entry.table_bytes == 0 || entry.table_bytes > MAX_TABLE_BYTES {
        return Err(Error::TableBound {
            bytes: entry.table_bytes,
        });
    }
    entry
        .table_address
        .checked_add(entry.table_bytes as u64)
        .ok_or(Error::AddressOverflow)?;
    if entry.table_address == 0 {
        return Err(Error::UnreadableRange);
    }
    // No mutation/normalization of any firmware byte or address.
    Ok(entry)
}
fn field<'a>(index: u8, strings: &'a [u8]) -> Result<Observation<'a>, Error> {
    if index == 0 {
        return Ok(Observation::Absent);
    }
    let mut start = 0;
    for number in 1..=index {
        let end = strings[start..]
            .iter()
            .position(|b| *b == 0)
            .ok_or(Error::InvalidStringIndex)?
            + start;
        if end == start {
            return Err(Error::InvalidStringIndex);
        }
        if number == index {
            return Ok(Observation::Value(&strings[start..=end]));
        }
        start = end + 1;
    }
    Err(Error::InvalidStringIndex)
}
/// Walk the complete declared table. Every string-set has a bounded double NUL;
/// only a zero string index establishes verified field absence.
pub fn identity(entry: Entry, table: &[u8]) -> Result<SystemIdentity<'_>, Error> {
    if table.len() > MAX_TABLE_BYTES || table.len() != entry.table_bytes {
        return Err(Error::TableLength);
    }
    let mut at = 0usize;
    let mut count = 0usize;
    let mut found = None;
    let mut board = None;
    let mut ended = false;
    let mut handles = [0u8; 8192];
    while at < table.len() {
        let head = table.get(at..at + 4).ok_or(Error::StructureLength)?;
        let handle = usize::from(le16(head, 2));
        let mask = 1u8 << (handle % 8);
        if handles[handle / 8] & mask != 0 {
            return Err(Error::DuplicateHandle);
        }
        handles[handle / 8] |= mask;
        let length = usize::from(head[1]);
        if length < 4 || at + length > table.len() {
            return Err(Error::StructureLength);
        }
        let pool = &table[at + length..];
        let end = pool
            .windows(2)
            .position(|pair| pair == [0, 0])
            .ok_or(Error::UnterminatedStrings)?
            + 2;
        let next = at + length + end;
        if entry
            .maximum_structure_bytes
            .is_some_and(|maximum| next - at > usize::from(maximum))
        {
            return Err(Error::MaximumStructureSize);
        }
        count += 1;
        if count > 65535 {
            return Err(Error::StructureCount);
        }
        if head[0] == 1 {
            if found.is_some() {
                return Err(Error::DuplicateType1);
            }
            if length < 8 {
                return Err(Error::StructureLength);
            }
            let strings = &table[at + length..next];
            for offset in [7, 25, 26] {
                if offset < length {
                    let _ = field(table[at + offset], strings)?;
                }
            }
            found = Some(SystemIdentity {
                system_vendor: field(table[at + 4], strings)?,
                product_name: field(table[at + 5], strings)?,
                product_version: field(table[at + 6], strings)?,
                board_vendor: Observation::NotObserved,
                board_name: Observation::NotObserved,
            });
        }
        if head[0] == 2 && board.is_none() {
            // dmi_save_ident skips a field past the structure's length.
            let strings = &table[at + length..next];
            let saved = |offset: usize| {
                if offset < length {
                    field(table[at + offset], strings)
                } else {
                    Ok(Observation::NotObserved)
                }
            };
            board = Some((saved(4)?, saved(5)?));
        }
        at = next;
        if head[0] == 127 {
            ended = true;
            break;
        }
    }
    if entry
        .structures
        .is_some_and(|expected| count != usize::from(expected))
    {
        return Err(Error::StructureCount);
    }
    if !ended {
        return Err(Error::MissingEnd);
    }
    found
        .map(|identity| {
            let (board_vendor, board_name) =
                board.unwrap_or((Observation::NotObserved, Observation::NotObserved));
            SystemIdentity {
                board_vendor,
                board_name,
                ..identity
            }
        })
        .ok_or(Error::MissingType1)
}
fn split(bytes: &[u8]) -> Result<(Entry, &[u8]), Error> {
    let format = if bytes.starts_with(b"_SM3_") {
        Format::Smbios3
    } else {
        Format::Smbios2
    };
    let length = entry_length(bytes, format)?;
    let raw = bytes.get(..length).ok_or(Error::EntryLength)?;
    let entry = entry(raw, format)?;
    let table = &bytes[length..];
    identity(entry, table)?;
    Ok((entry, table))
}
fn revision(entry: &[u8], table: &[u8]) -> String {
    let hex = |bytes: &[u8]| -> String {
        Sha256::digest(bytes)
            .iter()
            .map(|b| format!("{b:02x}"))
            .collect()
    };
    format!("entry-sha256:{};table-sha256:{}", hex(entry), hex(table))
}
/// Byte ownership is supplied by boot's permanent-page allocation.
pub struct Captured<B> {
    bytes: Option<B>,
    entry: Entry,
    revision: String,
}
impl<B: AsRef<[u8]>> Captured<B> {
    pub fn new(bytes: B) -> Result<Self, Error> {
        let (entry, table) = split(bytes.as_ref())?;
        let revision = revision(&bytes.as_ref()[..entry.entry_bytes], table);
        Ok(Self {
            bytes: Some(bytes),
            entry,
            revision,
        })
    }
    pub fn take_resource(&mut self) -> Option<Resource<'_, B>> {
        let bytes = self.bytes.take()?;
        Some(Resource {
            name: NAME,
            target: TARGET,
            provenance: Provenance {
                source: self.entry.format.source(),
                revision: &self.revision,
                member: MEMBER,
                license: LICENSE,
            },
            expected_length: bytes.as_ref().len(),
            expected_sha256: Sha256::digest(bytes.as_ref()).into(),
            bytes,
        })
    }
}
pub struct Snapshot<'a> {
    pub image: Image<'a>,
    pub entry: Entry,
    pub identity: SystemIdentity<'a>,
}
/// Canonical image bytes and all three borrowed fields share the Provider owner.
/// Integrity/identity checks do not authenticate that owner as current hardware.
#[derive(Debug)]
pub enum LoadError {
    Resource(super::Error),
    Invalid(Error),
}
impl From<Error> for LoadError {
    fn from(error: Error) -> Self {
        Self::Invalid(error)
    }
}
pub fn load(provider: &dyn Provider) -> Result<Snapshot<'_>, LoadError> {
    let image = provider.load(REQUIREMENT).map_err(LoadError::Resource)?;
    if image.name() != NAME || image.target() != TARGET {
        return Err(Error::ImageBinding.into());
    }
    let (entry, table) = split(image.bytes())?;
    let p = image.provenance();
    if p.source != entry.format.source()
        || p.member != MEMBER
        || p.license != LICENSE
        || p.revision != revision(&image.bytes()[..entry.entry_bytes], table)
    {
        return Err(Error::ImageBinding.into());
    }
    Ok(Snapshot {
        image,
        entry,
        identity: identity(entry, table)?,
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::firmware::{Catalog, ErrorKind};
    use alloc::vec::Vec;
    fn sum(bytes: &mut [u8], at: usize) {
        bytes[at] = 0;
        bytes[at] = 0u8.wrapping_sub(bytes.iter().fold(0u8, |a, b| a.wrapping_add(*b)));
    }
    fn fixture(format: Format) -> (Vec<u8>, Vec<u8>) {
        let mut table = alloc::vec![1, 8, 1, 0, 1, 2, 3, 4];
        table.extend_from_slice(b"Vendor\xff\0Product\0Version\0SYNTHETIC-ONLY\0\0");
        table.extend_from_slice(&[127, 4, 255, 255, 0, 0]);
        let mut raw = match format {
            Format::Smbios3 => {
                let mut r = alloc::vec![0;24];
                r[..5].copy_from_slice(b"_SM3_");
                r[6] = 24;
                r[7] = 3;
                r[8] = 8;
                r[10] = 1;
                r[12..16].copy_from_slice(&(table.len() as u32).to_le_bytes());
                r[16..24].copy_from_slice(&0x2000u64.to_le_bytes());
                r
            }
            Format::Smbios2 => {
                let mut r = alloc::vec![0;31];
                r[..4].copy_from_slice(b"_SM_");
                r[5] = 31;
                r[6] = 2;
                r[7] = 8;
                r[8..10].copy_from_slice(&512u16.to_le_bytes());
                r[16..21].copy_from_slice(b"_DMI_");
                r[22..24].copy_from_slice(&(table.len() as u16).to_le_bytes());
                r[24..28].copy_from_slice(&0x2000u32.to_le_bytes());
                r[28..30].copy_from_slice(&2u16.to_le_bytes());
                sum(&mut r[16..31], 5);
                r
            }
        };
        sum(&mut raw, if format == Format::Smbios3 { 5 } else { 4 });
        (raw, table)
    }
    #[test]
    fn exact_raw_first_nul_fields_and_verified_zero_indices_are_distinct() {
        for format in [Format::Smbios2, Format::Smbios3] {
            let (raw, mut table) = fixture(format);
            let entry = entry(&raw, format).unwrap();
            let host = identity(entry, &table).unwrap();
            assert_eq!(host.system_vendor, Observation::Value(&b"Vendor\xff\0"[..]));
            assert_eq!(host.product_name, Observation::Value(&b"Product\0"[..]));
            assert_eq!(host.product_version, Observation::Value(&b"Version\0"[..]));
            let Observation::Value(value) = host.system_vendor else {
                panic!()
            };
            assert_eq!(value.as_ptr(), table[8..].as_ptr());
            for offset in [4, 5, 6] {
                table[offset] = 0;
            }
            assert_eq!(
                identity(entry, &table).unwrap(),
                SystemIdentity {
                    system_vendor: Observation::Absent,
                    product_name: Observation::Absent,
                    product_version: Observation::Absent,
                    board_vendor: Observation::NotObserved,
                    board_name: Observation::NotObserved,
                }
            );
            assert_eq!(
                SystemIdentity::default().system_vendor,
                Observation::NotObserved
            );
        }
    }
    #[test]
    fn the_first_baseboard_names_the_board_vendor_and_name() {
        let (raw, table) = fixture(Format::Smbios3);
        let end = table.len() - 6;
        let mut boards = table[..end].to_vec();
        boards.extend_from_slice(&[2, 8, 2, 0, 1, 2, 0, 0]);
        boards.extend_from_slice(b"Board Co\0PRIME Z370-A\0\0");
        boards.extend_from_slice(&[2, 8, 3, 0, 1, 0, 0, 0]);
        boards.extend_from_slice(b"Second\0\0");
        boards.extend_from_slice(&table[end..]);
        let mut entry = entry(&raw, Format::Smbios3).unwrap();
        entry.table_bytes = boards.len();
        let first = identity(entry, &boards).unwrap();
        assert_eq!(first.board_vendor, Observation::Value(&b"Board Co\0"[..]));
        assert_eq!(first.board_name, Observation::Value(&b"PRIME Z370-A\0"[..]));
        let mut short = table[..end].to_vec();
        short.extend_from_slice(&[2, 4, 2, 0, 0, 0]);
        short.extend_from_slice(&table[end..]);
        entry.table_bytes = short.len();
        let short = identity(entry, &short).unwrap();
        assert_eq!(short.board_vendor, Observation::NotObserved);
        assert_eq!(short.board_name, Observation::NotObserved);
    }
    #[test]
    fn checked_entry_lengths_checksums_versions_and_native_bounds_refuse() {
        for format in [Format::Smbios2, Format::Smbios3] {
            let (raw, _) = fixture(format);
            for length in 0..raw.len() {
                assert!(entry(&raw[..length], format).is_err());
            }
            for index in 0..raw.len() {
                let mut changed = raw.clone();
                changed[index] ^= 1;
                assert!(
                    entry(&changed, format).is_err(),
                    "mutation {format:?}/{index}"
                );
            }
        }
        let (mut raw, _) = fixture(Format::Smbios3);
        raw[12..16].copy_from_slice(&((MAX_TABLE_BYTES + 1) as u32).to_le_bytes());
        sum(&mut raw, 5);
        assert_eq!(
            entry(&raw, Format::Smbios3),
            Err(Error::TableBound {
                bytes: MAX_TABLE_BYTES + 1
            })
        );
        raw[12..16].copy_from_slice(&32u32.to_le_bytes());
        raw[16..24].copy_from_slice(&u64::MAX.to_le_bytes());
        sum(&mut raw, 5);
        assert_eq!(entry(&raw, Format::Smbios3), Err(Error::AddressOverflow));
        raw[16..24].fill(0);
        sum(&mut raw, 5);
        assert_eq!(entry(&raw, Format::Smbios3), Err(Error::UnreadableRange));
        let (mut raw, _) = fixture(Format::Smbios2);
        raw[21] ^= 1;
        sum(&mut raw, 4);
        assert_eq!(
            entry(&raw, Format::Smbios2),
            Err(Error::IntermediateChecksum)
        );
    }
    #[test]
    fn malformed_missing_ambiguous_or_truncated_type1_never_becomes_absence() {
        let (raw, table) = fixture(Format::Smbios3);
        let entry = entry(&raw, Format::Smbios3).unwrap();
        for length in 0..table.len() {
            let mut e = entry;
            e.table_bytes = length;
            assert!(identity(e, &table[..length]).is_err());
        }
        for offset in [4, 5, 6, 7] {
            let mut changed = table.clone();
            changed[offset] = 255;
            assert_eq!(identity(entry, &changed), Err(Error::InvalidStringIndex));
        }
        let mut changed = table.clone();
        changed[0] = 2;
        assert_eq!(identity(entry, &changed), Err(Error::MissingType1));
        let mut changed = table.clone();
        changed[1] = 3;
        assert_eq!(identity(entry, &changed), Err(Error::StructureLength));
        let mut changed = table[..table.len() - 6].to_vec();
        let mut second = table.clone();
        second[2] = 2;
        changed.extend_from_slice(&second);
        let mut e = entry;
        e.table_bytes = changed.len();
        assert_eq!(identity(e, &changed), Err(Error::DuplicateType1));
        let mut changed = table.clone();
        changed[table.len() - 2..].fill(1);
        assert_eq!(identity(entry, &changed), Err(Error::UnterminatedStrings));
        let (raw, table) = fixture(Format::Smbios2);
        let mut e = entry_fn(&raw, Format::Smbios2);
        e.structures = Some(1);
        assert_eq!(identity(e, &table), Err(Error::StructureCount));
        e.structures = Some(2);
        e.maximum_structure_bytes = Some(4);
        assert_eq!(identity(e, &table), Err(Error::MaximumStructureSize));
    }
    #[test]
    fn explicit_end_count_padding_and_unique_handles_follow_table_contract() {
        for format in [Format::Smbios2, Format::Smbios3] {
            let (raw, mut table) = fixture(format);
            let mut e = entry(&raw, format).unwrap();
            table.extend_from_slice(&[0xa5; 13]);
            e.table_bytes = table.len();
            assert!(identity(e, &table).is_ok());
            table.truncate(table.len() - 13 - 6);
            e.table_bytes = table.len();
            e.structures = e.structures.map(|_| 1);
            assert_eq!(identity(e, &table), Err(Error::MissingEnd));
            table.extend_from_slice(&[2, 4, 1, 0, 0, 0, 127, 4, 255, 255, 0, 0]);
            e.table_bytes = table.len();
            e.structures = e.structures.map(|_| 3);
            assert_eq!(identity(e, &table), Err(Error::DuplicateHandle));
        }
    }

    fn entry_fn(raw: &[u8], format: Format) -> Entry {
        entry(raw, format).unwrap()
    }
    #[test]
    fn snapshot_keeps_one_canonical_image_and_rejects_provider_redirection() {
        struct Redirect<'a> {
            image: Image<'a>,
        }
        impl Provider for Redirect<'_> {
            fn load(&self, _: Requirement<'_>) -> Result<Image<'_>, super::super::Error> {
                Ok(self.image)
            }
        }
        for changed in 0..10 {
            let (mut raw, table) = fixture(Format::Smbios3);
            raw.extend_from_slice(&table);
            let pointer = raw.as_ptr();
            let mut captured = Captured::new(raw).unwrap();
            let mut resource = captured.take_resource().unwrap();
            match changed {
                0 => {}
                1 => resource.name = "host/smbios/redirect.bin",
                2 => resource.target.driver = "other",
                3 => resource.target.hardware = "guest",
                4 => resource.target.revision = "v2",
                5 => resource.target.board = Some("board"),
                6 => resource.provenance.source = "invented",
                7 => resource.provenance.revision = "invented",
                8 => resource.provenance.member = "invented",
                _ => resource.provenance.license = "invented",
            }
            let requirement = Requirement {
                name: resource.name,
                target: resource.target,
                minimum_bytes: 1,
            };
            let catalog = Catalog::from_handoff(alloc::vec![resource]).unwrap();
            let provider = Redirect {
                image: catalog.load(requirement).unwrap(),
            };
            let loaded = load(&provider);
            if changed == 0 {
                let snapshot = loaded.unwrap();
                assert_eq!(snapshot.image.bytes().as_ptr(), pointer);
                assert_eq!(
                    snapshot.identity.product_name,
                    Observation::Value(&b"Product\0"[..])
                );
            } else {
                assert!(matches!(
                    loaded,
                    Err(LoadError::Invalid(Error::ImageBinding))
                ));
            }
        }
        let empty = Catalog::from_handoff(alloc::vec![]).unwrap();
        assert!(
            matches!(load(&empty),Err(LoadError::Resource(error)) if error.name==NAME && error.kind==ErrorKind::Missing)
        );
    }
}
