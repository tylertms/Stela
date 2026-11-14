//! The ACPI tables the host drivers need: the RSDP's root table, the
//! MCFG table that locates PCI configuration space, and any table by
//! address ([`power`](super::power) reads the FADT and DSDT).
use alloc::vec::Vec;
use core::sync::atomic::{AtomicPtr, AtomicU32, Ordering};
use stela_contracts::bytes::{le32, le64};
use stela_drivers::bus::pci::EcamWindow;

const HEADER: usize = 36;
/// The longest table accepted. The largest DSDTs are a few hundred KiB; a
/// header naming more is no table, and reading that far could leave the
/// firmware's memory.
const LONGEST: usize = 16 << 20;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AcpiError {
    BadRsdp,
    BadTable([u8; 4]),
    Missing([u8; 4]),
}

fn checksum(bytes: &[u8]) -> bool {
    bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte)) == 0
}

/// The signatures of the tables used with a wrong checksum, each once.
static FAULTS: [AtomicU32; 16] = [const { AtomicU32::new(0) }; 16];
/// Called once for each table used with a wrong checksum: a `fn([u8; 4])`,
/// null while none is set.
static ON_FAULT: AtomicPtr<()> = AtomicPtr::new(core::ptr::null_mut());

/// Call `hook` once for each table used with a wrong checksum, those already
/// used included. Firmware that patches a table at start without fixing its
/// checksum is common; the table is used as it is, and the log says so once.
pub fn on_checksum_fault(hook: fn([u8; 4])) {
    ON_FAULT.store(hook as *mut (), Ordering::Release);
    for fault in &FAULTS {
        match fault.load(Ordering::Acquire) {
            0 => break,
            signature => hook(signature.to_le_bytes()),
        }
    }
}

/// Note a wrong checksum on `signature`'s table, once.
fn checksum_fault(signature: [u8; 4]) {
    // 0 marks a free slot.
    let word = u32::from_le_bytes(signature).max(1);
    for fault in &FAULTS {
        match fault.compare_exchange(0, word, Ordering::AcqRel, Ordering::Acquire) {
            Ok(_) => break,
            Err(seen) if seen == word => return,
            Err(_) => {}
        }
    }
    let hook = ON_FAULT.load(Ordering::Acquire);
    if !hook.is_null() {
        // SAFETY: `on_checksum_fault` stored a `fn([u8; 4])`, the pointer's
        // only source.
        let hook: fn([u8; 4]) = unsafe { core::mem::transmute(hook) };
        hook(signature);
    }
}

/// The table at `address`. Its length must be at least a header and at most
/// [`LONGEST`]. A wrong checksum is reported once ([`on_checksum_fault`])
/// and the table used, as the ACPI reference implementation does by default.
///
/// # Safety
///
/// `address` must be readable, identity-mapped memory holding at least an
/// SDT header, and the length that header names.
pub unsafe fn table(address: u64) -> Result<&'static [u8], AcpiError> {
    table_in(&Identity, address)
}

/// Find a table by signature through the XSDT, or the RSDT on ACPI 1.0 or
/// when the XSDT's header is unusable. Only the wanted table matters: null
/// entries are skipped and every other table is read no further than its
/// signature, so a damaged table elsewhere never stops a lookup. The first
/// table with the signature and a usable length is returned; a wrong
/// checksum on it or on the root is reported once and the table used
/// ([`table`]).
///
/// # Safety
///
/// `rsdp` must be the firmware's RSDP, with every table it reaches mapped.
pub unsafe fn find(rsdp: u64, signature: &[u8; 4]) -> Result<&'static [u8], AcpiError> {
    find_in(&Identity, rsdp, signature)
}

/// The memory the tables are read from.
trait Tables {
    /// The `length` bytes at physical `address`.
    fn bytes(&self, address: u64, length: usize) -> &'static [u8];
}

/// The identity-mapped firmware memory. Built only by [`table`] and
/// [`find`], whose callers promise that every byte read is mapped.
struct Identity;

impl Tables for Identity {
    fn bytes(&self, address: u64, length: usize) -> &'static [u8] {
        // SAFETY: `Identity` exists only inside `table` and `find`, whose
        // callers promise every table header and the length it names, and the
        // RSDP, readable and identity-mapped for the rest of the boot.
        unsafe { core::slice::from_raw_parts(address as *const u8, length) }
    }
}

/// [`table`] in `memory`. Its length must be at least a header and at most
/// [`LONGEST`]. A wrong checksum is reported once ([`on_checksum_fault`])
/// and the table used, as the ACPI reference implementation does by default.
fn table_in(memory: &impl Tables, address: u64) -> Result<&'static [u8], AcpiError> {
    let header = memory.bytes(address, HEADER);
    let signature: [u8; 4] = header[..4].try_into().unwrap();
    let length = le32(header, 4) as usize;
    if !(HEADER..=LONGEST).contains(&length) {
        return Err(AcpiError::BadTable(signature));
    }
    let bytes = memory.bytes(address, length);
    if !checksum(bytes) {
        checksum_fault(signature);
    }
    Ok(bytes)
}

/// The root table at `address` with `width`-byte entries, when there is one
/// and its header is usable and holds at least one entry.
fn root_table(memory: &impl Tables, address: u64, width: usize) -> Option<&'static [u8]> {
    if address == 0 {
        return None;
    }
    let root = table_in(memory, address).ok()?;
    (root.len() >= HEADER + width).then_some(root)
}

/// [`find`] in `memory`.
fn find_in(
    memory: &impl Tables,
    rsdp: u64,
    signature: &[u8; 4],
) -> Result<&'static [u8], AcpiError> {
    let short = memory.bytes(rsdp, 20);
    if &short[..8] != b"RSD PTR " || !checksum(short) {
        return Err(AcpiError::BadRsdp);
    }
    let rsdt = u64::from(le32(short, 16));
    let xsdt = if short[15] >= 2 {
        let long = memory.bytes(rsdp, 36);
        if !checksum(long) {
            return Err(AcpiError::BadRsdp);
        }
        le64(long, 24)
    } else {
        0
    };
    let (root, width) = match root_table(memory, xsdt, 8) {
        Some(root) => (root, 8),
        None => (root_table(memory, rsdt, 4).ok_or(AcpiError::BadRsdp)?, 4),
    };
    let mut unusable = None;
    for entry in root[HEADER..].chunks_exact(width) {
        let address = match width {
            8 => le64(entry, 0),
            _ => u64::from(le32(entry, 0)),
        };
        if address == 0 || memory.bytes(address, 4) != signature {
            continue;
        }
        match table_in(memory, address) {
            Ok(found) => return Ok(found),
            Err(error) => unusable = Some(error),
        }
    }
    Err(unusable.unwrap_or(AcpiError::Missing(*signature)))
}

/// The ECAM windows of an MCFG table.
pub fn mcfg(table: &[u8]) -> Result<Vec<EcamWindow>, AcpiError> {
    const ENTRIES: usize = HEADER + 8;
    if table.len() < ENTRIES
        || &table[..4] != b"MCFG"
        || !(table.len() - ENTRIES).is_multiple_of(16)
    {
        return Err(AcpiError::BadTable(*b"MCFG"));
    }
    table[ENTRIES..]
        .as_chunks::<16>()
        .0
        .iter()
        .map(|entry| {
            let window = EcamWindow {
                base: le64(entry, 0),
                segment: u16::from_le_bytes([entry[8], entry[9]]),
                start_bus: entry[10],
                end_bus: entry[11],
            };
            match window.start_bus <= window.end_bus && window.base.is_multiple_of(1 << 20) {
                true => Ok(window),
                false => Err(AcpiError::BadTable(*b"MCFG")),
            }
        })
        .collect()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sealed(mut bytes: Vec<u8>) -> Vec<u8> {
        let length = bytes.len() as u32;
        bytes[4..8].copy_from_slice(&length.to_le_bytes());
        let sum = bytes.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        bytes[9] = bytes[9].wrapping_sub(sum);
        bytes
    }

    fn header(signature: &[u8; 4]) -> Vec<u8> {
        let mut bytes = vec![0; HEADER];
        bytes[..4].copy_from_slice(signature);
        bytes
    }

    fn mcfg_table(windows: &[(u64, u16, u8, u8)]) -> Vec<u8> {
        let mut bytes = header(b"MCFG");
        bytes.extend([0; 8]);
        for (base, segment, start, end) in windows {
            bytes.extend(base.to_le_bytes());
            bytes.extend(segment.to_le_bytes());
            bytes.extend([*start, *end, 0, 0, 0, 0]);
        }
        sealed(bytes)
    }

    #[test]
    fn mcfg_windows_parse_and_reject_bad_entries() {
        let table = mcfg_table(&[(0xb000_0000, 0, 0, 0xff), (0xe000_0000, 1, 0x10, 0x1f)]);
        assert_eq!(
            mcfg(&table).unwrap(),
            [
                EcamWindow {
                    base: 0xb000_0000,
                    segment: 0,
                    start_bus: 0,
                    end_bus: 0xff
                },
                EcamWindow {
                    base: 0xe000_0000,
                    segment: 1,
                    start_bus: 0x10,
                    end_bus: 0x1f
                }
            ]
        );
        assert!(mcfg(&mcfg_table(&[(0xb000_0000, 0, 9, 8)])).is_err());
        assert!(mcfg(&mcfg_table(&[(0xb000_1000, 0, 0, 1)])).is_err());
        assert!(mcfg(&table[..table.len() - 1]).is_err());
    }

    /// Tables at made-up physical addresses. A read outside them fails the
    /// test, so a test also shows which bytes a lookup reads.
    struct Fake(Vec<(u64, &'static [u8])>);

    impl Tables for Fake {
        fn bytes(&self, address: u64, length: usize) -> &'static [u8] {
            self.0
                .iter()
                .find_map(|(base, bytes)| {
                    let offset = usize::try_from(address.checked_sub(*base)?).ok()?;
                    bytes.get(offset..offset + length)
                })
                .unwrap_or_else(|| {
                    panic!("{length} bytes read at {address:#x}, outside every table")
                })
        }
    }

    impl Fake {
        fn new(tables: Vec<(u64, Vec<u8>)>) -> Self {
            Self(
                tables
                    .into_iter()
                    .map(|(base, bytes)| (base, &*bytes.leak()))
                    .collect(),
            )
        }
    }

    const RSDP: u64 = 0xe_0000;

    fn rsdp(revision: u8, rsdt: u32, xsdt: u64) -> Vec<u8> {
        let mut rsdp = vec![0u8; 36];
        rsdp[..8].copy_from_slice(b"RSD PTR ");
        rsdp[15] = revision;
        rsdp[16..20].copy_from_slice(&rsdt.to_le_bytes());
        rsdp[20..24].copy_from_slice(&36u32.to_le_bytes());
        rsdp[24..32].copy_from_slice(&xsdt.to_le_bytes());
        let short = rsdp[..20]
            .iter()
            .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        rsdp[8] = rsdp[8].wrapping_sub(short);
        let long = rsdp.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        rsdp[32] = rsdp[32].wrapping_sub(long);
        rsdp
    }

    fn xsdt(entries: &[u64]) -> Vec<u8> {
        let mut xsdt = header(b"XSDT");
        for entry in entries {
            xsdt.extend(entry.to_le_bytes());
        }
        sealed(xsdt)
    }

    fn rsdt(entries: &[u32]) -> Vec<u8> {
        let mut rsdt = header(b"RSDT");
        for entry in entries {
            rsdt.extend(entry.to_le_bytes());
        }
        sealed(rsdt)
    }

    /// `table` with its checksum wrong.
    fn spoiled(mut table: Vec<u8>) -> Vec<u8> {
        table[9] = table[9].wrapping_add(1);
        table
    }

    fn faulted(signature: &[u8; 4]) -> bool {
        let word = u32::from_le_bytes(*signature);
        FAULTS
            .iter()
            .any(|fault| fault.load(Ordering::Acquire) == word)
    }

    #[test]
    fn find_reads_other_tables_no_further_than_their_signature() {
        let mcfg = mcfg_table(&[(0xb000_0000, 0, 0, 0xff)]);
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0, 0x1000)),
            (0x1000, xsdt(&[0x2000, 0x3000, 0x4000])),
            // A vendor table patched without its checksum fixed.
            (0x2000, spoiled(sealed(header(b"SSDT")))),
            // Four bytes, so reading its header fails the test.
            (0x3000, b"OEMX".to_vec()),
            (0x4000, mcfg.clone()),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"MCFG"), Ok(&mcfg[..]));
        assert!(!faulted(b"SSDT"));
    }

    #[test]
    fn find_skips_null_entries() {
        let mcfg = mcfg_table(&[(0xb000_0000, 0, 0, 0xff)]);
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0, 0x1000)),
            (0x1000, xsdt(&[0, 0x2000, 0])),
            (0x2000, mcfg.clone()),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"MCFG"), Ok(&mcfg[..]));
        assert_eq!(
            find_in(&memory, RSDP, b"HPET"),
            Err(AcpiError::Missing(*b"HPET"))
        );
    }

    static TPM2_FAULTS: AtomicU32 = AtomicU32::new(0);

    fn count_tpm2(signature: [u8; 4]) {
        if &signature == b"TPM2" {
            TPM2_FAULTS.fetch_add(1, Ordering::Relaxed);
        }
    }

    #[test]
    fn find_uses_the_wanted_table_with_a_wrong_checksum_and_reports_it_once() {
        on_checksum_fault(count_tpm2);
        let tpm2 = spoiled(sealed(header(b"TPM2")));
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0, 0x1000)),
            (0x1000, xsdt(&[0x2000])),
            (0x2000, tpm2.clone()),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"TPM2"), Ok(&tpm2[..]));
        assert_eq!(find_in(&memory, RSDP, b"TPM2"), Ok(&tpm2[..]));
        assert_eq!(TPM2_FAULTS.load(Ordering::Relaxed), 1);
    }

    #[test]
    fn find_uses_a_root_with_a_wrong_checksum_and_reports_it() {
        let ecdt = sealed(header(b"ECDT"));
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0, 0x1000)),
            (0x1000, spoiled(xsdt(&[0x2000]))),
            (0x2000, ecdt.clone()),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"ECDT"), Ok(&ecdt[..]));
        assert!(faulted(b"XSDT"));
    }

    #[test]
    fn find_passes_over_a_wanted_table_with_an_impossible_length() {
        let mut huge = header(b"BGRT");
        huge[4..8].copy_from_slice(&u32::MAX.to_le_bytes());
        let bgrt = sealed(header(b"BGRT"));
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0, 0x1000)),
            (0x1000, xsdt(&[0x2000, 0x3000])),
            (0x2000, huge.clone()),
            (0x3000, bgrt.clone()),
            (0x5000, xsdt(&[0x2000])),
            (0x6000, rsdp(2, 0, 0x5000)),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"BGRT"), Ok(&bgrt[..]));
        assert_eq!(
            find_in(&memory, 0x6000, b"BGRT"),
            Err(AcpiError::BadTable(*b"BGRT"))
        );
    }

    #[test]
    fn find_falls_back_to_the_rsdt_when_the_xsdt_header_is_unusable() {
        let mcfg = mcfg_table(&[(0xb000_0000, 0, 0, 0xff)]);
        let mut short = header(b"XSDT");
        short[4..8].copy_from_slice(&8u32.to_le_bytes());
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0x1000, 0x8000)),
            (0x1000, rsdt(&[0, 0x2000])),
            (0x2000, mcfg.clone()),
            (0x8000, short),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"MCFG"), Ok(&mcfg[..]));
    }

    #[test]
    fn find_uses_the_rsdt_on_acpi_1() {
        let mcfg = mcfg_table(&[(0xb000_0000, 0, 0, 0xff)]);
        let memory = Fake::new(vec![
            (RSDP, rsdp(0, 0x1000, 0x8000)),
            (0x1000, rsdt(&[0x2000])),
            (0x2000, mcfg.clone()),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"MCFG"), Ok(&mcfg[..]));
    }

    #[test]
    fn find_refuses_roots_with_no_entry() {
        let memory = Fake::new(vec![
            (RSDP, rsdp(2, 0x1000, 0x2000)),
            (0x1000, rsdt(&[])),
            (0x2000, xsdt(&[])),
        ]);
        assert_eq!(find_in(&memory, RSDP, b"MCFG"), Err(AcpiError::BadRsdp));
    }

    #[test]
    fn find_walks_the_xsdt_in_identity_mapped_memory() {
        let mcfg = mcfg_table(&[(0xb000_0000, 0, 0, 0xff)]);
        let apic = sealed(header(b"APIC"));
        let mut xsdt = header(b"XSDT");
        xsdt.extend((apic.as_ptr() as u64).to_le_bytes());
        xsdt.extend((mcfg.as_ptr() as u64).to_le_bytes());
        let xsdt = sealed(xsdt);
        let mut rsdp = [0u8; 36];
        rsdp[..8].copy_from_slice(b"RSD PTR ");
        rsdp[15] = 2;
        rsdp[20..24].copy_from_slice(&36u32.to_le_bytes());
        rsdp[24..32].copy_from_slice(&(xsdt.as_ptr() as u64).to_le_bytes());
        let short = rsdp[..20]
            .iter()
            .fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        rsdp[8] = rsdp[8].wrapping_sub(short);
        let long = rsdp.iter().fold(0u8, |sum, byte| sum.wrapping_add(*byte));
        rsdp[32] = rsdp[32].wrapping_sub(long);
        let rsdp_address = rsdp.as_ptr() as u64;
        // SAFETY: every table above lives until the end of the test.
        unsafe {
            assert_eq!(find(rsdp_address, b"MCFG").unwrap(), &mcfg[..]);
            assert_eq!(
                find(rsdp_address, b"HPET"),
                Err(AcpiError::Missing(*b"HPET"))
            );
        }
        rsdp[0] = b'X';
        assert_eq!(
            // SAFETY: as above.
            unsafe { find(rsdp.as_ptr() as u64, b"MCFG") },
            Err(AcpiError::BadRsdp)
        );
    }
}
