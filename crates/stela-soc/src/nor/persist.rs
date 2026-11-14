//! The NOR image on a host partition, so NVRAM survives reboots.
//!
//! The partition's first [`CAPACITY`] bytes hold the image as the die
//! stores it. An all-zero region is a partition never used for NOR (a new
//! GPT partition reads as zeros) and receives a fresh image; any other
//! content must identify a NOR image ([`Image::boot_bank`]), so a wrong
//! partition is refused. Damaged banks remain unchanged: boot firmware
//! supplies an empty proxy until the guest saves. Committed changes are
//! stored by erase sector.
use super::{CAPACITY, Image, SECTOR_SIZE};
use crate::Result;
use alloc::{collections::BTreeMap, vec, vec::Vec};
use stela_contracts::refuse;
use stela_drivers::block::{BlockDevice, Identity, Request, Submission};
use stela_soc_data::nor::{ANVRAM, nvram};

/// The image `device` holds, and whether it was blank and has just been
/// initialized.
pub fn load(device: &mut dyn BlockDevice) -> Result<(Image, bool)> {
    let block_size = sector_blocks(device.identity())?.1;
    let mut bytes = vec![0; CAPACITY];
    device
        .read(0, &mut bytes)
        .map_err(|error| refuse!("NOR partition read: {error}", ?error))?;
    if bytes.iter().all(|&b| b == 0) {
        let image = Image::fresh();
        device
            .write(0, image.bytes())
            .and_then(|()| device.flush())
            .map_err(|error| refuse!("NOR partition initialization: {error}", ?error))?;
        return Ok((image, true));
    }
    let image = Image::from_bytes(bytes)?;
    image.boot_bank().map_err(|_| {
        refuse!(
            "the NOR partition ({block_size}-byte blocks) is neither blank nor an identifiable \
             NOR image",
            block_size
        )
    })?;
    Ok((image, false))
}

/// Store the sector at `offset`, durably.
pub fn store(device: &mut dyn BlockDevice, offset: usize, sector: &[u8]) -> Result<()> {
    let block_size = device.identity().block_size;
    request(device.identity(), offset, sector.to_vec())?
        .run(device)
        .map(drop)
        .map_err(|error| {
            refuse!(
                "NOR partition write at {offset:#x} ({block_size}-byte blocks): {error}",
                offset,
                block_size,
                ?error
            )
        })
}

/// The durable write that stores `sector`, at `offset`, on a device of
/// `identity`.
pub fn request(identity: &Identity, offset: usize, sector: Vec<u8>) -> Result<Request> {
    let per_sector = sector_blocks(identity)?.0;
    if !offset.is_multiple_of(SECTOR_SIZE) || sector.len() != SECTOR_SIZE || offset >= CAPACITY {
        return Err(refuse!("a NOR store is one whole sector inside the image"));
    }
    Ok(Request::Write {
        lba: (offset / SECTOR_SIZE) as u64 * per_sector,
        data: sector,
        durable: true,
    })
}

/// Save `bank` into `image`'s next NVRAM bank ([`Image::save_bank`]) and
/// store every sector it changed; returns the bank written.
pub fn save_bank(device: &mut dyn BlockDevice, image: &mut Image, bank: &[u8]) -> Result<u32> {
    let (written, sectors) = image.save_bank(bank)?;
    for offset in sectors {
        let sector = image.bytes()[offset..offset + SECTOR_SIZE].to_vec();
        store(device, offset, &sector)?;
    }
    Ok(written)
}

/// The NVRAM bank a sector at `offset` belongs to, if any.
fn bank_of(offset: usize) -> Option<usize> {
    let start = ANVRAM.0 as usize;
    let end = start + nvram::BANK_COUNT as usize * nvram::BANK_SIZE;
    (start..end)
        .contains(&offset)
        .then(|| (offset - start) / nvram::BANK_SIZE)
}

/// Take the sectors of the next store from `waiting`, leaving the rest
/// waiting: every sector, except that the two NVRAM banks never go in one
/// store. A store's writes run together and complete in any order (FUA
/// writes in one submission), so with both banks in flight a power cut
/// could leave neither valid; a Mac's flash writes one bank whole before it
/// erases the other, and so one always survives. Of two banks waiting, the
/// one the guest wrote first goes first: one whose first sector is not
/// waiting (the rest of a bank an earlier store began), else the one whose
/// first sector holds the lower generation (an erased one, being rewritten,
/// last).
pub fn next_store(waiting: &mut BTreeMap<usize, Vec<u8>>) -> Vec<(usize, Vec<u8>)> {
    let mut banks = [false; nvram::BANK_COUNT as usize];
    for &offset in waiting.keys() {
        if let Some(bank) = bank_of(offset) {
            banks[bank] = true;
        }
    }
    if banks.iter().filter(|&&waits| waits).count() < 2 {
        return core::mem::take(waiting).into_iter().collect();
    }
    let order = |bank: usize| {
        let first = ANVRAM.0 as usize + bank * nvram::BANK_SIZE;
        match waiting.get(&first) {
            None => (0, 0, bank),
            Some(sector) => (1, nvram::generation(sector), bank),
        }
    };
    let first = (0..banks.len())
        .min_by_key(|&bank| order(bank))
        .expect("two banks wait");
    let taken: Vec<usize> = waiting
        .keys()
        .copied()
        .filter(|&offset| bank_of(offset).is_none_or(|bank| bank == first))
        .collect();
    taken
        .into_iter()
        .map(|offset| {
            let sector = waiting.remove(&offset).expect("a waiting sector");
            (offset, sector)
        })
        .collect()
}

/// The NOR's stores on its partition, which never wait for the device:
/// changed sectors wait while a store runs, and the next store takes them
/// ([`next_store`]) once the one in progress is durable.
#[derive(Default)]
pub struct Stores {
    /// Changed sectors by offset; a newer copy replaces a waiting one.
    waiting: BTreeMap<usize, Vec<u8>>,
    /// The store in progress, and its sectors.
    running: Option<(Submission, usize)>,
}

impl Stores {
    pub fn new() -> Self {
        Self::default()
    }

    /// Store `dirty` after the store in progress.
    pub fn add(&mut self, dirty: impl IntoIterator<Item = (usize, Vec<u8>)>) {
        self.waiting.extend(dirty);
    }

    /// Collect the device's completions and start the next store, without
    /// waiting: whether every store is durable.
    pub fn advance(&mut self, device: &mut dyn BlockDevice) -> Result<bool> {
        loop {
            if let Some((running, _)) = self.running.as_mut() {
                match running.poll(device) {
                    None => return Ok(false),
                    Some(Err(error)) => {
                        return Err(refuse!("NOR store: {error}", ?error));
                    }
                    Some(Ok(_)) => self.running = None,
                }
            }
            if self.waiting.is_empty() {
                return Ok(true);
            }
            let identity = device.identity().clone();
            let sectors = next_store(&mut self.waiting);
            let count = sectors.len();
            let requests = sectors
                .into_iter()
                .map(|(offset, sector)| request(&identity, offset, sector))
                .collect::<Result<Vec<_>>>()?;
            let store = Submission::batch(requests, &identity)
                .map_err(|error| refuse!("NOR persistence batch: {error}", %error))?;
            self.running = Some((store, count));
        }
    }
}

/// Blocks per sector and the block size; the partition must hold the image
/// in whole sectors.
fn sector_blocks(identity: &Identity) -> Result<(u64, u32)> {
    let block_size = identity.block_size;
    if block_size == 0 || !SECTOR_SIZE.is_multiple_of(block_size as usize) {
        return Err(refuse!(
            "NOR partition block size {block_size} does not divide a sector",
            block_size
        ));
    }
    if identity.read_only || identity.blocks * u64::from(block_size) < CAPACITY as u64 {
        return Err(refuse!(
            "the NOR partition is read-only or smaller than the 8 MiB image"
        ));
    }
    Ok(((SECTOR_SIZE / block_size as usize) as u64, block_size))
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_drivers::block::{BlockError, Command, Finished, RamDisk};

    /// A running write: its tag, first block and data.
    type Running = (u32, u64, Vec<u8>);
    /// Which running write completes next, given those running and the
    /// number completed so far.
    type Pick = fn(&[Running], usize) -> usize;

    /// A disk with a volatile cache and Force Unit Access that runs queued
    /// writes and completes them in the order `pick` chooses among those
    /// running, one each [`Self::complete`]. What it completed is durable
    /// ([`Self::durable`]), what still runs is not: the disk as a power
    /// cut leaves it.
    struct Adversary {
        identity: Identity,
        durable: Vec<u8>,
        running: Vec<Running>,
        done: BTreeMap<u32, Finished>,
        next: u32,
        pick: Pick,
        completed: usize,
    }

    impl Adversary {
        fn new(bytes: Vec<u8>, pick: Pick) -> Self {
            let mut identity = RamDisk::new("nor", bytes.len() as u64 / 512, 512).identity;
            identity.features.write_cache = true;
            identity.features.fua = true;
            Self {
                identity,
                durable: bytes,
                running: Vec::new(),
                done: BTreeMap::new(),
                next: 0,
                pick,
                completed: 0,
            }
        }

        /// Complete one running write, if any.
        fn complete(&mut self) -> bool {
            if self.running.is_empty() {
                return false;
            }
            let index = (self.pick)(&self.running, self.completed);
            let (tag, lba, data) = self.running.remove(index);
            let at = lba as usize * 512;
            self.durable[at..at + data.len()].copy_from_slice(&data);
            self.done.insert(tag, Ok(None));
            self.completed += 1;
            true
        }

        fn valid_bank(&self) -> bool {
            Image::from_bytes(self.durable[..CAPACITY].to_vec())
                .unwrap()
                .current_bank()
                .is_ok()
        }
    }

    impl BlockDevice for Adversary {
        fn identity(&self) -> &Identity {
            &self.identity
        }
        fn read(&mut self, _: u64, _: &mut [u8]) -> core::result::Result<(), BlockError> {
            Err(BlockError::Unsupported("writes only"))
        }
        fn write(&mut self, _: u64, _: &[u8]) -> core::result::Result<(), BlockError> {
            Err(BlockError::Unsupported("queued writes only"))
        }
        fn depth(&self) -> usize {
            64
        }
        fn room(&self) -> bool {
            self.running.len() < 64
        }
        fn start(&mut self, command: Command) -> core::result::Result<u32, BlockError> {
            let Command::Write { lba, data, fua } = command else {
                return Err(BlockError::Unsupported("writes only"));
            };
            assert!(fua, "a NOR store is durable");
            let tag = self.next;
            self.next += 1;
            self.running.push((tag, lba, data));
            Ok(tag)
        }
        fn finish(&mut self, tag: u32) -> Option<Finished> {
            self.done.remove(&tag)
        }
    }

    /// The image and two NVRAM saves of many sectors each, the second to
    /// the other bank: what the guest commits between two of the host's
    /// stores. The image as the disk held it before, and the sectors.
    fn two_saves() -> (Vec<u8>, Vec<(usize, Vec<u8>)>) {
        use super::super::nvram::{Partition, Variables};
        let mut image = Image::fresh();
        let before = image.bytes().to_vec();
        let mut dirty = BTreeMap::new();
        for (round, fill) in [(0u8, 0x11u8), (1, 0x22)] {
            let current = image.current_bank().unwrap();
            let mut variables = Variables::parse(image.bank(current).unwrap()).unwrap();
            let value = vec![fill.wrapping_add(round); 30_000];
            variables.set(Partition::Common, b"big", &value).unwrap();
            let bank = variables.encode().unwrap();
            let (_, sectors) = image.save_bank(&bank).unwrap();
            assert!(sectors.len() > 4, "a save spans many sectors");
            for offset in sectors {
                dirty.insert(offset, image.bytes()[offset..offset + SECTOR_SIZE].to_vec());
            }
        }
        (before, dirty.into_iter().collect())
    }

    /// Completion orders: first started first, last first, and the two
    /// ends in turn, which tears both banks at once if both run.
    fn oldest(_: &[Running], _: usize) -> usize {
        0
    }
    fn newest(running: &[Running], _: usize) -> usize {
        running.len() - 1
    }
    fn ends(running: &[Running], step: usize) -> usize {
        if step.is_multiple_of(2) {
            0
        } else {
            running.len() - 1
        }
    }

    #[test]
    fn a_power_cut_at_any_point_of_two_bank_saves_leaves_a_valid_bank() {
        for pick in [oldest, newest, ends] {
            let (before, dirty) = two_saves();
            // The number of completions a whole run takes.
            let total = {
                let mut disk = Adversary::new(before.clone(), pick);
                let mut stores = Stores::new();
                stores.add(dirty.clone());
                while !stores.advance(&mut disk).unwrap() {
                    assert!(disk.complete());
                }
                disk.completed
            };
            assert_eq!(total, dirty.len());
            for cut in 0..=total {
                let mut disk = Adversary::new(before.clone(), pick);
                let mut stores = Stores::new();
                stores.add(dirty.clone());
                stores.advance(&mut disk).unwrap();
                for _ in 0..cut {
                    assert!(disk.complete());
                    stores.advance(&mut disk).unwrap();
                }
                assert!(
                    disk.valid_bank(),
                    "a cut after {cut} of {total} completions left no valid bank"
                );
            }
        }
    }

    #[test]
    fn both_banks_in_one_store_can_leave_none() {
        // What the split above prevents: the two saves' writes run together.
        let (before, dirty) = two_saves();
        let mut disk = Adversary::new(before, ends);
        let identity = disk.identity.clone();
        let requests = dirty
            .into_iter()
            .map(|(offset, sector)| request(&identity, offset, sector).unwrap())
            .collect();
        let mut store = Submission::batch(requests, &identity).unwrap();
        assert!(store.poll(&mut disk).is_none());
        disk.complete();
        disk.complete();
        assert!(!disk.valid_bank());
    }

    #[test]
    fn the_bank_a_store_began_goes_on_before_the_other() {
        let base = ANVRAM.0 as usize;
        let bank = nvram::BANK_SIZE;
        let header = |generation: u32| {
            let mut sector = vec![0; SECTOR_SIZE];
            sector[20..24].copy_from_slice(&generation.to_le_bytes());
            sector
        };
        // Bank 1's rest, with bank 0's new first sector: bank 1 first.
        let mut waiting = BTreeMap::from([
            (0x1000, vec![1; SECTOR_SIZE]),
            (base, header(9)),
            (base + bank + SECTOR_SIZE, vec![2; SECTOR_SIZE]),
        ]);
        let first: Vec<usize> = next_store(&mut waiting).into_iter().map(|s| s.0).collect();
        assert_eq!(first, [0x1000, base + bank + SECTOR_SIZE]);
        assert_eq!(waiting.keys().copied().collect::<Vec<_>>(), [base]);
        // Both whole: the lower generation first, an erased one last.
        let mut waiting = BTreeMap::from([(base, header(7)), (base + bank, header(6))]);
        assert_eq!(next_store(&mut waiting)[0].0, base + bank);
        let mut waiting =
            BTreeMap::from([(base, vec![0xff; SECTOR_SIZE]), (base + bank, header(6))]);
        assert_eq!(next_store(&mut waiting)[0].0, base + bank);
        // One bank or none: everything at once.
        let mut waiting = BTreeMap::from([(0, vec![0; SECTOR_SIZE]), (base, header(3))]);
        assert_eq!(next_store(&mut waiting).len(), 2);
        assert!(waiting.is_empty());
    }

    #[test]
    fn a_blank_partition_is_initialized_and_sectors_persist() {
        let mut disk = RamDisk::new("nor", CAPACITY as u64 / 512, 512);
        let (image, fresh) = load(&mut disk).unwrap();
        assert!(fresh);
        assert_eq!(&disk.bytes[..CAPACITY], image.bytes());
        let mut sector = vec![0xff; SECTOR_SIZE];
        sector[0] = 0x42;
        store(&mut disk, 0x2000, &sector).unwrap();
        let (again, fresh) = load(&mut disk).unwrap();
        assert!(!fresh);
        assert_eq!(again.bytes()[0x2000], 0x42);
        assert!(store(&mut disk, 0x2001, &sector).is_err());
        assert!(store(&mut disk, CAPACITY, &sector).is_err());
    }

    #[test]
    fn a_saved_bank_goes_to_the_next_bank_and_survives_a_reload() {
        use super::super::nvram::{Partition, Variables};
        let mut disk = RamDisk::new("nor", CAPACITY as u64 / 512, 512);
        let (mut image, _) = load(&mut disk).unwrap();
        let mut variables = Variables::parse(image.bank(0).unwrap()).unwrap();
        variables
            .set(Partition::System, b"boot-volume", b"x:y:z")
            .unwrap();
        let bank = variables.encode().unwrap();
        assert_eq!(save_bank(&mut disk, &mut image, &bank).unwrap(), 1);
        let (again, _) = load(&mut disk).unwrap();
        assert_eq!(again.current_bank().unwrap(), 1);
        assert_eq!(again.bank(1).unwrap(), bank.as_slice());
        // An older or equal generation is refused.
        assert!(save_bank(&mut disk, &mut image, &bank).is_err());
    }

    #[test]
    fn foreign_and_small_partitions_are_refused() {
        let mut foreign = RamDisk::new("x", CAPACITY as u64 / 512, 512);
        foreign.bytes[7] = 1;
        assert!(load(&mut foreign).is_err());
        assert_eq!(foreign.bytes[7], 1);
        assert!(load(&mut RamDisk::new("small", 100, 512)).is_err());
        assert!(load(&mut RamDisk::new("odd", 4096, 3000)).is_err());
    }

    #[test]
    fn loading_torn_banks_preserves_disk_bytes_until_the_guest_syncs() {
        let mut disk = RamDisk::new("nor", CAPACITY as u64 / 512, 512);
        let (image, _) = load(&mut disk).unwrap();
        let first = super::super::ANVRAM.0 as usize;
        // Neither bank validates: bank 0 has an incomplete payload; bank
        // 1 was erased before an interrupted program wrote its header.
        disk.bytes[first + 256..first + super::super::nvram::BANK_SIZE].fill(0xff);
        let before = disk.bytes.clone();
        let (loaded, fresh) = load(&mut disk).unwrap();
        assert!(!fresh);
        assert!(loaded.current_bank().is_err());
        assert_eq!(
            loaded.boot_bank().unwrap().1,
            super::super::nvram::empty_bank()
        );
        assert_eq!(loaded.bytes(), before);
        assert_eq!(disk.bytes, before);
        assert_ne!(loaded, image);
    }
}
