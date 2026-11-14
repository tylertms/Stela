//! The persistent translation store's home on the stick: where `stela.efi`
//! keeps its `STLTPACK` packs, how much room each may take, and which packs are
//! kept. The pack format is the engine's; setup never reads a pack beyond
//! its header.
//!
//! ```text
//! stela/store/store.cfg      `bytes N`: one pack's bound
//! stela/store/XXXXXXXX.pak   one pack per translator id
//! stela/store/spare.bin      a blank pack file of the bound
//! stela/store/pipeline.bin   the Metal device's compiled pipelines
//! ```
//!
//! `stela.efi` never grows a file: the firmware grows one by writing every
//! byte, minutes over USB. A new translator's first boot takes the spare by
//! renaming it to its pack's name, and setup prepares the next spare
//! whenever it publishes the store, an update included. A pack that is
//! still blank is sound: its translator has not written it yet.
//!
//! Every name is an 8.3 name, which `stela.efi`'s own FAT32 reader finds
//! after boot services end to rewrite its pack in place: a pack is named by
//! the first four bytes of its translator id in hex, and its header names
//! the whole id.
//!
//! The home is the stick's FAT32 data partition, which also holds the install
//! medium and the boot bundles, as the firmware writes it. `stela.efi`
//! writes only its own translator id's pack, never past the bound. Setup
//! keeps the most recently written pack and removes the others when Stela is
//! updated or repaired. A damaged pack is harmless (the engine
//! discards it); `status` names it and `repair` removes it.
use super::{MARGIN, host::FileSystem, host::Host};
use crate::efi;
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use std::{
    fs,
    path::{Path, PathBuf},
    time::SystemTime,
};
use stela_boot_core::memory_budget::DEFAULT_TRANSLATION_PACK_BYTES;
use stela_engine::translations::{Digest, pack};
use stela_formats::journal::{self, DATA_START};

/// The store's directory on its volume.
pub(super) const DIRECTORY: &str = "stela/store";
/// The bound setup gives `stela.efi`.
pub(super) const CONFIG: &str = "store.cfg";
const BYTES_KEY: &str = "bytes";
const PACK_EXTENSION: &str = "pak";
/// The blank pack file a new translator's pack takes by rename.
pub(super) const SPARE: &str = "spare.bin";
/// The Metal device's pipeline store, which `stela.efi` rewrites in place as
/// it does a pack.
/// A blank one holds nothing yet.
pub(super) const PIPELINES: &str = "pipeline.bin";
/// The pipeline store's size, its bound.
pub(super) const PIPELINE_BYTES: u64 = 64 * MIB;
/// The translator id bytes a pack's file name holds.
const NAME_BYTES: usize = 4;
const MIB: u64 = 1024 * 1024;
/// One pack's bound by default.
pub(super) const DEFAULT_PACK_BYTES: u64 = DEFAULT_TRANSLATION_PACK_BYTES;
/// The smallest bound worth keeping a store for.
pub(super) const MIN_PACK_BYTES: u64 = 64 * MIB;
/// FAT32's largest file, on a 1 MiB boundary.
pub(super) const MAX_PACK_BYTES: u64 = 4 * 1024 * MIB - MIB;
/// Translator ids whose packs are kept: the running `stela.efi`'s.
pub(super) const PACKS_KEPT: usize = 1;

/// The store a setup keeps.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub(super) struct Store {
    /// The FAT32 volume of the stick that holds it.
    pub(super) volume: PathBuf,
    /// One pack's bound.
    pub(super) pack_bytes: u64,
}

impl Store {
    pub(super) fn directory(&self) -> PathBuf {
        super::join_slashed(&self.volume, DIRECTORY)
    }

    fn config_text(&self) -> String {
        format!("{BYTES_KEY} {}\n", self.pack_bytes)
    }

    /// Create the directory, write `store.cfg` and prepare the spare.
    pub(super) fn publish(&self) -> Result<()> {
        let directory = self.directory();
        fs::create_dir_all(&directory)
            .with_context(|| format!("cannot create {}", directory.display()))?;
        // A failed allocation leaves the previous configuration and every
        // translator's pack untouched; publishing the bound comes last.
        self.prepare_spare()?;
        self.prepare_pipelines()?;
        efi::write_atomic(
            &directory.join(CONFIG),
            self.config_text().as_bytes(),
            false,
        )
    }

    /// Whether the spare holds the bound's bytes.
    pub(super) fn spare_ready(&self) -> bool {
        fs::metadata(self.directory().join(SPARE))
            .is_ok_and(|m| m.is_file() && m.len() >= self.pack_bytes)
    }

    /// Allocate the spare at the bound: FAT32 has no sparse files, so the
    /// volume gives it zeroed clusters now, on the Mac.
    fn prepare_spare(&self) -> Result<()> {
        if self.spare_ready() {
            return Ok(());
        }
        let path = self.directory().join(SPARE);
        let progress = crate::progress::Phase::spinner("Preparing the translation cache");
        let file =
            fs::File::create(&path).with_context(|| format!("cannot create {}", path.display()))?;
        file.set_len(self.pack_bytes)
            .and_then(|()| file.sync_all())
            .with_context(|| {
                format!(
                    "cannot allocate {} bytes for {}",
                    self.pack_bytes,
                    path.display()
                )
            })?;
        progress.finish();
        Ok(())
    }

    /// Whether the pipeline store holds its bound's bytes.
    pub(super) fn pipelines_ready(&self) -> bool {
        fs::metadata(self.directory().join(PIPELINES))
            .is_ok_and(|m| m.is_file() && m.len() >= PIPELINE_BYTES)
    }

    /// Allocate a blank pipeline store, as [`Self::prepare_spare`] does the
    /// spare. One already of its size keeps what it holds.
    fn prepare_pipelines(&self) -> Result<()> {
        if self.pipelines_ready() {
            return Ok(());
        }
        let path = self.directory().join(PIPELINES);
        let file =
            fs::File::create(&path).with_context(|| format!("cannot create {}", path.display()))?;
        file.set_len(PIPELINE_BYTES)
            .and_then(|()| file.sync_all())
            .with_context(|| {
                format!(
                    "cannot allocate {PIPELINE_BYTES} bytes for {}",
                    path.display()
                )
            })
    }

    /// Whether `store.cfg` says what the record does.
    pub(super) fn config_intact(&self) -> bool {
        fs::read_to_string(self.directory().join(CONFIG)).is_ok_and(|t| t == self.config_text())
    }

    /// The packs on the volume, most recently written first.
    pub(super) fn packs(&self) -> Vec<Pack> {
        let Ok(entries) = fs::read_dir(self.directory()) else {
            return Vec::new();
        };
        let mut packs: Vec<Pack> = entries
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| path.extension().is_some_and(|e| e == PACK_EXTENSION))
            .map(|path| Pack::read(&path, self.pack_bytes))
            .collect();
        packs.sort_by(|a, b| b.modified.cmp(&a.modified).then(a.path.cmp(&b.path)));
        packs
    }

    /// The packs to remove: damaged ones, and those past the
    /// [`PACKS_KEPT`] most recently written intact ones.
    pub(super) fn stale(&self) -> Vec<Pack> {
        let mut intact = 0;
        self.packs()
            .into_iter()
            .filter(|pack| match pack.problem {
                Some(_) => true,
                None => {
                    intact += 1;
                    intact > PACKS_KEPT
                }
            })
            .collect()
    }

    /// Remove the stale packs; returns them.
    pub(super) fn prune(&self) -> Result<Vec<Pack>> {
        let stale = self.stale();
        for pack in &stale {
            fs::remove_file(&pack.path)
                .with_context(|| format!("cannot remove {}", pack.path.display()))?;
        }
        Ok(stale)
    }

    /// What the store's packs take now.
    pub(super) fn used_bytes(&self) -> u64 {
        self.packs().iter().map(|p| p.bytes).sum()
    }

    /// The store, for plans and `status`.
    pub(super) fn describe(&self) -> String {
        format!(
            "{}: one pack of up to {:.1} GB, the newest",
            self.directory().display(),
            self.pack_bytes as f64 / 1e9
        )
    }

    /// What [`Self::repair`] would change: a missing or changed
    /// `store.cfg`, and each pack to remove, damaged or past the kept
    /// ones.
    pub(super) fn repairs(&self) -> Vec<String> {
        let mut repairs = Vec::new();
        if !self.config_intact() {
            repairs.push(format!(
                "write {} (bytes {})",
                self.directory().join(CONFIG).display(),
                self.pack_bytes
            ));
        }
        if !self.spare_ready() {
            repairs.push(self.spare_repair());
        }
        if !self.pipelines_ready() {
            repairs.push(format!(
                "prepare {} ({PIPELINE_BYTES} bytes)",
                self.directory().join(PIPELINES).display()
            ));
        }
        for pack in self.stale() {
            repairs.push(format!(
                "remove {}{}",
                pack.path.display(),
                pack.problem
                    .map(|problem| format!(", which {problem}"))
                    .unwrap_or_else(|| ", an older translator's".into())
            ));
        }
        repairs
    }

    /// The repair that prepares the spare.
    pub(super) fn spare_repair(&self) -> String {
        format!(
            "prepare {} ({} bytes)",
            self.directory().join(SPARE).display(),
            self.pack_bytes
        )
    }

    /// Whether a translator took the spare for its pack, as the first boot
    /// of each new `stela.efi` does: the store works, and only the next new
    /// `stela.efi` would find no spare until setup prepares one. A spare that
    /// is missing beside no pack, or short, is damage.
    pub(super) fn spare_taken(&self) -> bool {
        !self.directory().join(SPARE).exists() && !self.packs().is_empty()
    }

    /// Write `store.cfg` again and remove the stale packs.
    pub(super) fn repair(&self) -> Result<()> {
        self.publish()?;
        self.prune()?;
        Ok(())
    }
}

/// One pack file, checked as far as setup checks it.
#[derive(Debug, Clone)]
pub(super) struct Pack {
    pub(super) path: PathBuf,
    pub(super) bytes: u64,
    modified: Option<SystemTime>,
    /// Why the engine would discard it; `None` when its header is sound.
    pub(super) problem: Option<String>,
}

impl Pack {
    fn read(path: &Path, bound: u64) -> Self {
        let metadata = fs::metadata(path).ok();
        let bytes = metadata.as_ref().map_or(0, |m| m.len());
        let modified = metadata.and_then(|m| m.modified().ok());
        let problem = check(path, bytes, bound).err();
        Self {
            path: path.to_path_buf(),
            bytes,
            modified,
            problem,
        }
    }
}

/// A pack file is a translation journal (`stela_formats::journal`) whose
/// newest commit names its file's translator id, or blank, and its size is
/// within the bound.
fn check(path: &Path, bytes: u64, bound: u64) -> Result<(), String> {
    if bytes > bound {
        return Err(format!("holds {bytes} bytes, more than the bound {bound}"));
    }
    let mut slots = vec![0u8; DATA_START.min(bytes) as usize];
    use std::io::Read;
    fs::File::open(path)
        .and_then(|mut file| file.read_exact(&mut slots))
        .map_err(|_| "cannot be read".to_string())?;
    // A spare its translator took and has not written yet.
    if slots.iter().all(|&byte| byte == 0) {
        return Ok(());
    }
    if bytes < DATA_START {
        return Err("is shorter than a translation journal's commit slots".into());
    }
    let Some(commit) = journal::candidates(&slots, pack::JOURNAL_DOMAIN, bytes)
        .into_iter()
        .next()
    else {
        return Err("holds no translation journal commit".into());
    };
    let name = path.file_name().and_then(|s| s.to_str()).unwrap_or("");
    if !name.eq_ignore_ascii_case(&pack_name(&commit.key)) {
        let id = Digest(commit.key).hex();
        return Err(format!("names translator {id}, not its file name"));
    }
    Ok(())
}

/// The file name of translator `id`'s pack.
pub(super) fn pack_name(id: &[u8]) -> String {
    format!("{}.{PACK_EXTENSION}", hex::encode(&id[..NAME_BYTES]))
}

/// The bytes a store of `pack_bytes` needs on its volume: the kept packs
/// and the spare beside them, the pipeline store, and a margin.
pub(super) fn needed_bytes(pack_bytes: u64) -> u64 {
    pack_bytes * (PACKS_KEPT as u64 + 1) + PIPELINE_BYTES + MARGIN
}

/// The largest bound, on a 1 MiB boundary, whose store fits in `room`.
fn fitting(room: u64) -> u64 {
    let bound = room.saturating_sub(PIPELINE_BYTES + MARGIN) / (PACKS_KEPT as u64 + 1);
    bound / MIB * MIB
}

/// The store's home among `candidates` (the stick's data partition): the
/// first FAT32 one. `planned` is what setup writes to a volume besides the
/// store, and `requested` the user's bound (`--translation-store-bytes`, 0
/// for none).
/// The default bound shrinks to what fits; below [`MIN_PACK_BYTES`] the
/// setup keeps no store, which the returned note says. A requested bound
/// that does not fit is refused with the sizes.
pub(super) fn choose(
    host: &dyn Host,
    candidates: &[&Path],
    planned: impl Fn(&Path) -> u64,
    requested: Option<u64>,
) -> Result<(Option<Store>, Option<String>)> {
    if requested == Some(0) {
        return Ok((
            None,
            Some(
                "no translation store (--translation-store-bytes 0): every boot translates anew"
                    .into(),
            ),
        ));
    }
    if let Some(bytes) = requested {
        ensure!(
            (MIN_PACK_BYTES..=MAX_PACK_BYTES).contains(&bytes),
            "--translation-store-bytes {bytes} is outside {MIN_PACK_BYTES}..={MAX_PACK_BYTES} (64 \
             MiB to FAT32's largest file), or 0 for no store"
        );
    }
    let mut seen: Vec<&Path> = Vec::new();
    for &volume in candidates {
        if seen.contains(&volume) {
            continue;
        }
        seen.push(volume);
        let info = host.volume(volume)?;
        if info.filesystem != FileSystem::Fat {
            continue;
        }
        let existing = Store {
            volume: volume.to_path_buf(),
            pack_bytes: MAX_PACK_BYTES,
        }
        .used_bytes();
        let room = (info.free_bytes + existing).saturating_sub(planned(volume));
        let pack_bytes = match requested {
            Some(bytes) => {
                let need = needed_bytes(bytes);
                if room < need {
                    bail!(
                        "{} has {:.1} GB for the translation store; {} packs of {:.1} GB and a \
                         rewrite need {:.1} GB. Pass a smaller --translation-store-bytes, or 0 \
                         for none.",
                        volume.display(),
                        room as f64 / 1e9,
                        PACKS_KEPT,
                        bytes as f64 / 1e9,
                        need as f64 / 1e9
                    );
                }
                bytes
            }
            None => fitting(room).min(DEFAULT_PACK_BYTES),
        };
        if pack_bytes < MIN_PACK_BYTES {
            return Ok((
                None,
                Some(format!(
                    "no translation store: {} ({:.1} GB free) has no room for {} packs of 64 \
                     MiB; every boot translates anew",
                    volume.display(),
                    room as f64 / 1e9,
                    PACKS_KEPT
                )),
            ));
        }
        return Ok((
            Some(Store {
                volume: volume.to_path_buf(),
                pack_bytes,
            }),
            None,
        ));
    }
    Ok((
        None,
        Some(
            "no translation store: the stick has no FAT32 volume the firmware writes; every boot \
             translates anew"
                .into(),
        ),
    ))
}

/// The store's directory, refused unless it is where setup puts it, for
/// `uninstall`.
pub(super) fn checked_directory(store: &Store) -> Result<PathBuf> {
    let directory = store.directory();
    ensure!(
        directory.ends_with(DIRECTORY),
        "the setup record names an unexpected translation store {}",
        directory.display()
    );
    Ok(directory)
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;

    const ID_BYTES: usize = 32;

    /// The bytes of the smallest pack file [`write_pack`] writes: the
    /// commit slots and one block.
    pub(in crate::setup) const SMALL_PACK: usize = (DATA_START + journal::ALIGN) as usize;

    /// A pack file of translator `id` holding `bytes` bytes in all, at
    /// least [`SMALL_PACK`]: a journal whose commit names an empty segment.
    pub(in crate::setup) fn write_pack(directory: &Path, id: [u8; 32], bytes: usize) -> PathBuf {
        let segment = pack::Layout::new(&Digest(id), Vec::new()).to_vec();
        let commit = journal::Commit {
            domain: pack::JOURNAL_DOMAIN,
            generation: 1,
            key: id,
            entries: vec![journal::Entry {
                kind: pack::SEGMENT_BLOB,
                offset: DATA_START,
                length: segment.len() as u64,
                digest: Digest::of(&segment).0,
            }],
        };
        let mut file = vec![0; bytes.max(SMALL_PACK)];
        let slot = commit.slot_offset() as usize;
        file[slot..slot + journal::SLOT_BYTES as usize].copy_from_slice(&commit.encode());
        file[DATA_START as usize..][..segment.len()].copy_from_slice(&segment);
        fs::create_dir_all(directory).unwrap();
        let path = directory.join(pack_name(&id));
        fs::write(&path, file).unwrap();
        path
    }

    #[test]
    fn packs_are_checked_by_header_and_the_newest_ids_are_kept() {
        let dir = tempfile::tempdir().unwrap();
        let store = Store {
            volume: dir.path().to_path_buf(),
            pack_bytes: 2 * SMALL_PACK as u64,
        };
        store.publish().unwrap();
        assert!(store.config_intact());
        assert!(store.spare_ready());
        assert!(store.repairs().is_empty(), "{:?}", store.repairs());
        // A spare taken under a pack's name is sound until it is written.
        let blank = store.directory().join(pack_name(&[8u8; 32]));
        fs::rename(store.directory().join(SPARE), &blank).unwrap();
        assert!(!store.spare_ready());
        assert!(
            store.repairs()[0].starts_with("prepare "),
            "{:?}",
            store.repairs()
        );
        assert_eq!(store.packs()[0].problem, None);
        fs::remove_file(&blank).unwrap();
        store.publish().unwrap();
        assert_eq!(
            fs::read_to_string(store.directory().join(CONFIG)).unwrap(),
            format!("bytes {}\n", 2 * SMALL_PACK)
        );
        let mut paths = Vec::new();
        for n in 0..5u8 {
            paths.push(write_pack(&store.directory(), [n; 32], SMALL_PACK));
            let time = SystemTime::UNIX_EPOCH + std::time::Duration::from_secs(1000 + n as u64);
            fs::File::options()
                .write(true)
                .open(&paths[n as usize])
                .unwrap()
                .set_modified(time)
                .unwrap();
        }
        // A pack under another id's name, a short one and one past the
        // bound are damaged.
        let wrong = store.directory().join(pack_name(&[9u8; 32]));
        fs::copy(&paths[0], &wrong).unwrap();
        fs::write(store.directory().join(pack_name(&[0xab; 32])), b"STLT").unwrap();
        write_pack(&store.directory(), [7; 32], 3 * SMALL_PACK);
        let problems: Vec<String> = store
            .packs()
            .into_iter()
            .filter_map(|p| p.problem)
            .collect();
        assert_eq!(problems.len(), 3, "{problems:?}");
        assert!(problems.iter().any(|p| p.contains("not its file name")));
        assert!(problems.iter().any(|p| p.contains("shorter")));
        assert!(problems.iter().any(|p| p.contains("more than the bound")));
        // The newest intact pack stays.
        let removed = store.prune().unwrap();
        assert_eq!(removed.len(), 7);
        let kept: Vec<PathBuf> = store.packs().into_iter().map(|p| p.path).collect();
        assert_eq!(kept, [paths[4].clone()]);
        assert_eq!(store.used_bytes(), SMALL_PACK as u64);
        fs::remove_dir_all(checked_directory(&store).unwrap()).unwrap();
        assert!(!store.directory().exists());
    }

    #[test]
    fn repair_keeps_a_journal_and_removes_a_pack_of_another_layout() {
        let dir = tempfile::tempdir().unwrap();
        let store = Store {
            volume: dir.path().to_path_buf(),
            pack_bytes: 4 * SMALL_PACK as u64,
        };
        let path = write_pack(&store.directory(), [1; ID_BYTES], SMALL_PACK);
        let bytes = fs::read(&path).unwrap();
        // A pack of the layout before journals, as its first bytes.
        let old = store.directory().join(pack_name(&[2; ID_BYTES]));
        let mut old_bytes = pack::MAGIC.to_vec();
        old_bytes.extend_from_slice(&pack::SCHEMA.to_le_bytes());
        old_bytes.resize(SMALL_PACK, 0);
        fs::write(&old, old_bytes).unwrap();
        let problems: Vec<String> = store
            .packs()
            .into_iter()
            .filter_map(|p| p.problem)
            .collect();
        assert_eq!(problems, ["holds no translation journal commit"]);
        store.repair().unwrap();
        assert_eq!(fs::read(path).unwrap(), bytes);
        assert!(!old.exists());
    }

    #[test]
    fn spare_failure_preserves_configuration_and_packs() {
        let dir = tempfile::tempdir().unwrap();
        let store = Store {
            volume: dir.path().to_path_buf(),
            pack_bytes: 4096,
        };
        fs::create_dir_all(store.directory()).unwrap();
        let config = store.directory().join(CONFIG);
        let previous = b"bytes 2048\n";
        fs::write(&config, previous).unwrap();
        let pack = write_pack(&store.directory(), [3; 32], SMALL_PACK);
        let bytes = fs::read(&pack).unwrap();
        let spare = store.directory().join(SPARE);
        fs::create_dir(&spare).unwrap();
        assert!(
            !store.spare_ready(),
            "a directory is not a preallocated file"
        );
        assert!(store.publish().is_err());
        assert_eq!(fs::read(&config).unwrap(), previous);
        assert_eq!(fs::read(&pack).unwrap(), bytes);
        fs::remove_dir(&spare).unwrap();
        // A partial file left by interrupted preparation is completed on retry.
        fs::write(&spare, [0; 100]).unwrap();
        store.publish().unwrap();
        assert_eq!(fs::metadata(&spare).unwrap().len(), store.pack_bytes);
        assert!(store.config_intact());
        assert_eq!(fs::read(&pack).unwrap(), bytes);
    }

    #[test]
    fn translation_bound_fits_volume() {
        assert_eq!(
            fitting(needed_bytes(DEFAULT_PACK_BYTES)),
            DEFAULT_PACK_BYTES
        );
        assert_eq!(
            fitting(needed_bytes(MIN_PACK_BYTES) - 1),
            MIN_PACK_BYTES - MIB
        );
        assert_eq!(fitting(0), 0);
    }
}
