//! The install medium: the full IPSW on the stick's FAT32 data partition, as
//! `stela.efi`'s `--install` reads it.
//!
//! ```text
//! stela/install/<build>/ipsw.000 ...   PIECE_BYTES each, the last shorter
//! stela/install/<build>/pieces.bin     SHA-256 of every 64 KiB block of every piece
//! stela/install/<build>/blocks.bin     SHA-256 of every 64 KiB block of every member
//! stela/install/<build>/payload.json   the restore inventory, written last
//! ```
//!
//! No partition is created or formatted: the pieces are files below FAT32's
//! 4 GiB limit, so their concatenation is the IPSW byte for byte. Each piece
//! is written as `ipsw.NNN.partial`, read back, and renamed; a rerun keeps
//! every complete piece whose bytes still match the IPSW and writes only
//! the rest. While writing the IPSW, setup checks its SHA-256 and any
//! catalog SHA-1 supplied by the download receipt. It writes
//! `payload.json` after verification, so a medium without it is never
//! taken for a complete one.
//!
//! A remote medium ([`write_remote`]) holds only `payload.json` and
//! `blocks.bin`: `stela.efi` streams the IPSW from Apple's servers during
//! the install (`payload::Layout::Remote`).
use super::{MARGIN, host::FileSystem, host::Host};
use crate::{efi, restore};
use anyhow::{Context, Result, bail, ensure};
use serde::{Deserialize, Serialize};
use sha1::Sha1;
use sha2::{Digest, Sha256};
use std::{
    fs,
    io::{Read, Seek, SeekFrom, Write},
    path::{Path, PathBuf},
};
use stela_boot_core::options::System;
use {
    stela_formats::payload::BLOCK_HASHES_FILE, stela_formats::payload::BlockHashes,
    stela_formats::payload::HASH_BLOCK_BYTES, stela_formats::payload::Layout,
    stela_formats::payload::MANIFEST_FILE, stela_formats::payload::MAX_PIECES,
    stela_formats::payload::PIECE_HASHES_FILE, stela_formats::payload::Piece,
    stela_formats::payload::install_directory, stela_formats::payload::piece_name,
    stela_formats::payload::piece_sizes,
};

pub(super) mod download;
pub(super) mod image;

/// Bytes read at a time: a whole number of hash blocks.
const BUFFER: usize = 64 * HASH_BLOCK_BYTES as usize;

/// The install medium a setup wrote.
#[derive(Debug, Clone, Serialize, Deserialize, PartialEq, Eq)]
pub(super) struct Medium {
    pub(super) build: String,
    /// The mounted FAT32 data partition of the stick that holds it.
    pub(super) volume: PathBuf,
    /// That partition's GPT unique GUID, by which `stela.efi` finds it.
    pub(super) partition_guid: String,
    /// SHA-256 of its `payload.json`.
    pub(super) manifest_sha256: String,
    /// It names the IPSW on Apple's servers instead of holding it.
    #[serde(default, skip_serializing_if = "std::ops::Not::not")]
    pub(super) remote: bool,
    /// Pieces a repair read and found damaged ([`Medium::damaged_pieces`]):
    /// written again like missing ones, and removed only once nothing can
    /// refuse the repair. Never recorded.
    #[serde(skip)]
    pub(super) damaged: Vec<String>,
}

impl Medium {
    pub(super) fn directory(&self) -> PathBuf {
        directory(&self.volume, &self.build)
    }

    /// The bytes its files take on its volume, which removing it frees.
    pub(super) fn bytes_on_volume(&self) -> u64 {
        fs::read_dir(self.directory())
            .into_iter()
            .flatten()
            .filter_map(|entry| entry.ok()?.metadata().ok())
            .filter(fs::Metadata::is_file)
            .map(|metadata| metadata.len())
            .sum()
    }

    /// Its directory, checked to be an install medium's on its volume
    /// before setup removes it.
    pub(super) fn removable_directory(&self) -> Result<PathBuf> {
        let directory = self.directory();
        ensure!(
            directory.starts_with(&self.volume)
                && directory
                    .parent()
                    .is_some_and(|p| p.ends_with("stela/install")),
            "the setup record names an unexpected install medium {}",
            directory.display()
        );
        Ok(directory)
    }

    /// `payload.json` as `stela.cfg` names it on the firmware's volume.
    pub(super) fn firmware_manifest(&self) -> String {
        format!(
            "\\{}\\{MANIFEST_FILE}",
            install_directory(&self.build).replace('/', "\\")
        )
    }

    /// The engine options that restore from it, set in `system`.
    pub(super) fn restore_from(&self, system: &mut System) -> Result<()> {
        system.install = Some(self.firmware_manifest());
        system.install_source = match self.remote {
            true => None,
            false => Some(
                self.partition_guid
                    .parse()
                    .map_err(anyhow::Error::msg)
                    .with_context(|| format!("install medium partition {}", self.partition_guid))?,
            ),
        };
        Ok(())
    }

    /// What is wrong with it: a missing or changed `payload.json`, or a
    /// missing or short piece. Empty when it is intact; the pieces' bytes
    /// are checked block by block when `stela.efi` reads them.
    pub(super) fn problems(&self) -> Vec<String> {
        let directory = self.directory();
        let manifest = directory.join(MANIFEST_FILE);
        let bytes = match fs::read(&manifest) {
            Ok(bytes) => bytes,
            Err(_) => return vec![format!("{} is missing", manifest.display())],
        };
        if stela_formats::sha256(&bytes) != self.manifest_sha256 {
            return vec![format!(
                "{} differs from the one setup wrote",
                manifest.display()
            )];
        }
        let Ok(parsed) = stela_formats::payload::PayloadManifest::parse(&bytes) else {
            return vec![format!("{} is not a restore inventory", manifest.display())];
        };
        if self.remote {
            return match directory.join(BLOCK_HASHES_FILE).is_file() {
                true => Vec::new(),
                false => vec![format!(
                    "{} is missing",
                    directory.join(BLOCK_HASHES_FILE).display()
                )],
            };
        }
        let Layout::Pieces { pieces, .. } = &parsed.layout else {
            return vec![format!("{} names no pieces", manifest.display())];
        };
        let mut problems = Vec::new();
        for (index, piece) in pieces.iter().enumerate() {
            let path = directory.join(piece_name(index));
            match fs::metadata(&path) {
                Ok(metadata) if metadata.len() == piece.bytes => {}
                Ok(metadata) => problems.push(format!(
                    "{} holds {} bytes, not {}",
                    path.display(),
                    metadata.len(),
                    piece.bytes
                )),
                Err(_) => problems.push(format!("{} is missing", path.display())),
            }
        }
        for piece in &self.damaged {
            problems.push(format!(
                "{} differs from its recorded SHA-256",
                directory.join(piece).display()
            ));
        }
        for file in [PIECE_HASHES_FILE, BLOCK_HASHES_FILE] {
            if !directory.join(file).is_file() {
                problems.push(format!("{} is missing", directory.join(file).display()));
            }
        }
        problems
    }
}

impl Medium {
    /// The pieces whose bytes differ from the SHA-256 `payload.json`
    /// records for them, each read whole: what a block check during an
    /// install would refuse. Missing or short pieces are [`Medium::problems`]
    /// and are named here too.
    pub(super) fn damaged_pieces(&self) -> Result<Vec<String>> {
        let directory = self.directory();
        let manifest = fs::read(directory.join(MANIFEST_FILE))
            .with_context(|| format!("reading {}", directory.join(MANIFEST_FILE).display()))?;
        let parsed = stela_formats::payload::PayloadManifest::parse(&manifest)
            .map_err(|error| anyhow::anyhow!("{}: {error}", directory.display()))?;
        let Layout::Pieces { pieces, .. } = &parsed.layout else {
            return Ok(Vec::new());
        };
        let total = pieces.iter().map(|piece| piece.bytes).sum();
        let progress =
            crate::progress::Phase::bytes("Checking every piece of the install medium", total);
        let mut damaged = Vec::new();
        for (index, piece) in pieces.iter().enumerate() {
            let path = directory.join(piece_name(index));
            let intact = fs::metadata(&path).is_ok_and(|m| m.len() == piece.bytes)
                && hash_file(&path, piece.bytes, &progress).is_ok_and(|sha| sha == piece.sha256);
            if !intact {
                damaged.push(piece_name(index));
            }
        }
        progress.finish();
        Ok(damaged)
    }
}

/// The pieces of the install medium the stick's boot log says failed their
/// block check during an install (`DAMAGED_LOG_LINE`, `ipsw.003 block 12`),
/// in lines written after byte `after` of the log's stream: the log's head
/// when a repair last checked the medium ([`log_head`]). Each piece once.
pub(super) fn reported_damage(esp: &Path, after: u64) -> Vec<String> {
    use stela_formats::payload::pieces::DAMAGED_LOG_LINE;
    let Some(log) = fs::read(esp.join(efi::BOOT_LOG))
        .ok()
        .and_then(|region| stela_runtime::pc::boot_log::read(&region).ok())
    else {
        return Vec::new();
    };
    let start = log.head - log.stream.len() as u64;
    let mut found: Vec<String> = Vec::new();
    let mut offset = 0u64;
    for line in log.stream.split(|&byte| byte == b'\n') {
        offset += line.len() as u64 + 1;
        if start + offset <= after {
            continue;
        }
        let text = String::from_utf8_lossy(line);
        let Some((_, block)) = text.split_once(DAMAGED_LOG_LINE) else {
            continue;
        };
        let piece = block
            .split_whitespace()
            .next()
            .unwrap_or_default()
            .to_owned();
        if !piece.is_empty() && !found.contains(&piece) {
            found.push(piece);
        }
    }
    found
}

/// The end of the stick's boot log stream: what a check of the medium now
/// has seen ([`reported_damage`]). Zero without a log.
pub(super) fn log_head(esp: &Path) -> u64 {
    fs::read(esp.join(efi::BOOT_LOG))
        .ok()
        .and_then(|region| stela_runtime::pc::boot_log::read(&region).ok())
        .map_or(0, |log| log.head)
}

/// `stela/install/<build>` on `volume`.
pub(super) fn directory(volume: &Path, build: &str) -> PathBuf {
    super::join_slashed(volume, &install_directory(build))
}

/// The USB's FAT32 data partition, which holds the boot bundles, the install
/// medium and the translation store: `explicit`, or the first mounted FAT
/// partition of the ESP's disk that is not the ESP. Returns its mount and
/// GPT unique GUID.
pub(super) fn locate(
    host: &dyn Host,
    esp: &Path,
    explicit: Option<&Path>,
) -> Result<(PathBuf, String)> {
    let usb = host.volume(esp)?.disk.with_context(|| {
        format!(
            "this computer does not say which disk {} is on",
            esp.display()
        )
    })?;
    let disks = host.disks()?;
    let disk = disks
        .iter()
        .find(|disk| disk.identifier == usb)
        .with_context(|| format!("this computer does not list {usb}'s partitions"))?;
    let mut reasons = String::new();
    for partition in &disk.partitions {
        let Some(mount) = &partition.mount else {
            continue;
        };
        if explicit.is_some_and(|explicit| explicit != mount) {
            continue;
        }
        if partition.content == "EFI" {
            reasons.push_str(&format!(
                "  {} is the EFI partition, which keeps Stela\n",
                mount.display()
            ));
            continue;
        }
        let filesystem = host.volume(mount)?.filesystem;
        if filesystem != FileSystem::Fat {
            reasons.push_str(&format!(
                "  {} is {filesystem}, not FAT32\n",
                mount.display()
            ));
            continue;
        }
        let guid = host.partition(&partition.identifier)?.guid;
        ensure!(
            !guid.is_empty(),
            "{} has no GPT partition GUID, by which stela.efi would find it",
            partition.identifier
        );
        return Ok((mount.clone(), guid));
    }
    if let Some(explicit) = explicit {
        bail!(
            "{} is not a FAT32 data partition of the stick that holds {}:\n{reasons}",
            explicit.display(),
            esp.display()
        );
    }
    bail!(
        "The boot bundles and the install medium go on a FAT32 data partition of the stick beside \
         its EFI partition, and {usb} has none mounted:\n{reasons}Setup does not repartition the stick: add a FAT32 \
         partition of at least 30 GB with a disk utility, mount it, and run setup again."
    )
}

/// What writing the medium needs: the IPSW's pieces not already on
/// `volume` with their sizes, less what removing a damaged piece frees
/// ([`write`] removes it first), the hash files and a margin.
pub(super) fn needed_bytes(volume: &Path, build: &str, ipsw_bytes: u64, piece_bytes: u64) -> u64 {
    let directory = directory(volume, build);
    let mut need = MARGIN;
    for (index, size) in piece_sizes(ipsw_bytes, piece_bytes).into_iter().enumerate() {
        let present = fs::metadata(directory.join(piece_name(index))).map_or(0, |m| m.len());
        need += size.saturating_sub(present);
        // Its block hashes in pieces.bin.
        need += size.div_ceil(HASH_BLOCK_BYTES) * 32;
    }
    need
}

/// The bytes of the IPSW's pieces not on `volume` at their size: what a
/// download onto it fetches, the intact pieces being kept. The `damaged`
/// pieces ([`Medium::damaged`]) count as missing.
pub(super) fn missing_bytes(
    volume: &Path,
    build: &str,
    ipsw_bytes: u64,
    piece_bytes: u64,
    damaged: &[String],
) -> u64 {
    let directory = directory(volume, build);
    piece_sizes(ipsw_bytes, piece_bytes)
        .into_iter()
        .enumerate()
        .filter(|(index, size)| {
            damaged.contains(&piece_name(*index))
                || fs::metadata(directory.join(piece_name(*index)))
                    .map(|m| m.len())
                    .ok()
                    != Some(*size)
        })
        .map(|(_, size)| size)
        .sum()
}

/// Volume and piece layout whose required capacity is checked.
pub(super) struct SpaceOptions<'a> {
    pub(super) volume: &'a Path,
    pub(super) build: &'a str,
    pub(super) ipsw_bytes: u64,
    pub(super) piece_bytes: u64,
}

/// Refuse a volume without room for the medium, with the sizes.
pub(super) fn check_space(host: &dyn Host, options: SpaceOptions<'_>) -> Result<u64> {
    let SpaceOptions {
        volume,
        build,
        ipsw_bytes,
        piece_bytes,
    } = options;
    let need = needed_bytes(volume, build, ipsw_bytes, piece_bytes);
    let free = host.volume(volume)?.free_bytes;
    ensure!(
        free >= need,
        "{} has {:.1} GB free; the install medium needs {:.1} GB more (the macOS {build} IPSW is \
         {:.1} GB). Free space on it, or use a larger stick.",
        volume.display(),
        free as f64 / 1e9,
        need as f64 / 1e9,
        ipsw_bytes as f64 / 1e9
    );
    Ok(need)
}

/// The digests a download receipt names for the IPSW.
#[derive(Debug, Clone, Default)]
pub(super) struct Expected {
    pub(super) sha256: Option<String>,
    /// Apple's catalog SHA-1.
    pub(super) sha1: Option<String>,
}

impl Expected {
    pub(super) fn of(receipt: Option<&restore::Receipt>) -> Self {
        Self {
            sha256: receipt.map(|r| r.sha256.clone()),
            sha1: receipt.and_then(|r| r.image.sha1.clone()),
        }
    }
}

/// A piece's SHA-256 and the SHA-256 of each of its hash blocks.
#[derive(Default)]
struct Hashes {
    whole: Sha256,
    blocks: Vec<u8>,
}

impl Hashes {
    /// `bytes` is a whole number of blocks, but for a piece's last.
    fn update(&mut self, bytes: &[u8]) {
        self.whole.update(bytes);
        for block in bytes.chunks(HASH_BLOCK_BYTES as usize) {
            self.blocks.extend(Sha256::digest(block));
        }
    }

    fn finish(self) -> (String, Vec<u8>) {
        (hex::encode(self.whole.finalize()), self.blocks)
    }
}

/// Read `size` bytes of `source` at its position, feeding each buffer to
/// `each`.
fn stream(
    source: &mut impl Read,
    size: u64,
    mut each: impl FnMut(&[u8]) -> Result<()>,
) -> Result<()> {
    let mut buffer = vec![0u8; BUFFER];
    let mut left = size;
    while left > 0 {
        let n = left.min(BUFFER as u64) as usize;
        source.read_exact(&mut buffer[..n])?;
        each(&buffer[..n])?;
        left -= n as u64;
    }
    Ok(())
}

fn stream_file(
    path: &Path,
    size: u64,
    progress: &crate::progress::Phase,
    mut consume: impl FnMut(&[u8]),
) -> Result<()> {
    let mut file =
        fs::File::open(path).with_context(|| format!("cannot read {}", path.display()))?;
    stream(&mut file, size, |bytes| {
        consume(bytes);
        progress.advance(bytes.len() as u64);
        Ok(())
    })
}

fn hash_file(path: &Path, size: u64, progress: &crate::progress::Phase) -> Result<String> {
    let mut hash = Sha256::new();
    stream_file(path, size, progress, |bytes| hash.update(bytes))?;
    Ok(hex::encode(hash.finalize()))
}

/// The whole IPSW's SHA-256 and its SHA-1 if the catalog supplies one.
struct Whole<'a> {
    sha256: Sha256,
    catalog: Option<(&'a str, Sha1)>,
}

/// Stream a source piece, copying it durably when `partial` is supplied.
fn read_piece(
    ipsw: &mut fs::File,
    offset: u64,
    size: u64,
    partial: Option<&Path>,
    progress: &crate::progress::Phase,
    mut consume: impl FnMut(&[u8]),
) -> Result<()> {
    ipsw.seek(SeekFrom::Start(offset))?;
    let mut out = partial
        .map(|path| {
            fs::File::create(path).with_context(|| format!("cannot write {}", path.display()))
        })
        .transpose()?;
    stream(ipsw, size, |bytes| {
        consume(bytes);
        match (&mut out, partial) {
            (Some(out), Some(path)) => out
                .write_all(bytes)
                .with_context(|| format!("cannot write {}", path.display())),
            _ => Ok(()),
        }?;
        progress.advance(bytes.len() as u64);
        Ok(())
    })?;
    if let Some(out) = out {
        out.sync_all()?;
        crate::progress::drop_cached(&out)?;
    }
    Ok(())
}

/// What [`write`] did.
#[derive(Debug, Default)]
pub(super) struct Written {
    pub(super) kept: usize,
    pub(super) wrote: usize,
    pub(super) manifest_sha256: String,
}

/// Source inventory and destination layout for one install medium.
pub(super) struct WriteOptions<'a> {
    pub(super) volume: &'a Path,
    pub(super) ipsw: &'a Path,
    pub(super) inventory: restore::payload::Inventory,
    pub(super) expected: &'a Expected,
    pub(super) piece_bytes: u64,
}

/// Write the medium for `ipsw` to `volume`, with `inventory`, its restore
/// inventory, in `piece_bytes` pieces.
pub(super) fn write(options: WriteOptions<'_>) -> Result<Written> {
    let WriteOptions {
        volume,
        ipsw,
        inventory,
        expected,
        piece_bytes,
    } = options;
    let mut manifest = inventory.manifest;
    let build = manifest.product_build.clone();
    let directory = directory(volume, &build);
    fs::create_dir_all(&directory)?;
    // The old manifest goes first, so a medium being rewritten is never
    // taken for a complete one.
    let manifest_path = directory.join(MANIFEST_FILE);
    if manifest_path.exists() {
        fs::remove_file(&manifest_path)?;
    }
    let mut file =
        fs::File::open(ipsw).with_context(|| format!("cannot read {}", ipsw.display()))?;
    let total = file.metadata()?.len();
    ensure!(
        total == manifest.source.bytes,
        "{} changed while setup read it",
        ipsw.display()
    );
    let sizes = piece_sizes(total, piece_bytes);
    ensure!(
        !sizes.is_empty() && sizes.len() <= MAX_PIECES,
        "{} takes {} pieces of {piece_bytes} bytes",
        ipsw.display(),
        sizes.len()
    );
    let mut whole = Whole {
        sha256: Sha256::new(),
        catalog: expected.sha1.as_deref().map(|digest| (digest, Sha1::new())),
    };
    let mut pieces = Vec::new();
    let mut piece_blocks = Vec::new();
    let mut written = Written::default();
    let mut offset = 0;
    let progress =
        crate::progress::Phase::bytes("Writing and verifying the stick image", total * 2);
    for (index, &size) in sizes.iter().enumerate() {
        let name = piece_name(index);
        let path = directory.join(&name);
        let partial = directory.join(format!("{name}.partial"));
        let present = fs::metadata(&path).is_ok_and(|m| m.len() == size);
        // A piece of another size is damaged: it goes first, so a full USB
        // has room for its replacement.
        if !present && path.exists() {
            fs::remove_file(&path)?;
        }
        // The IPSW's bytes for this piece, read once: hashed, added to the
        // IPSW's digests, and written when the piece is not there yet.
        let mut hashes = Hashes::default();
        read_piece(
            &mut file,
            offset,
            size,
            (!present).then_some(partial.as_path()),
            &progress,
            |bytes| {
                hashes.update(bytes);
                whole.sha256.update(bytes);
                if let Some((_, hash)) = &mut whole.catalog {
                    hash.update(bytes);
                }
            },
        )?;
        let (digest, blocks) = hashes.finish();
        // The whole-piece digest verifies the bytes represented by the
        // already-recorded block hashes.
        let keep = present && hash_file(&path, size, &progress)? == digest;
        if keep {
            written.kept += 1;
        } else {
            if present {
                // A complete piece whose bytes differ: removed first, as a
                // piece of another size is, and written again.
                fs::remove_file(&path)?;
                progress.add_work(size * 2);
                read_piece(&mut file, offset, size, Some(&partial), &progress, |_| {})?;
            }
            // Read back, so a USB that loses writes is found now.
            ensure!(
                hash_file(&partial, size, &progress)? == digest,
                "{} does not read back as written; the stick may be failing",
                partial.display()
            );
            fs::rename(&partial, &path)?;
            written.wrote += 1;
        }
        pieces.push(Piece {
            bytes: size,
            sha256: digest,
        });
        piece_blocks.extend(blocks);
        offset += size;
    }
    // Leftovers of an interrupted or larger earlier copy.
    for entry in fs::read_dir(&directory)? {
        let entry = entry?;
        let name = entry.file_name().to_string_lossy().into_owned();
        let stale = name.ends_with(".partial")
            || name
                .strip_prefix("ipsw.")
                .and_then(|n| n.parse::<usize>().ok())
                .is_some_and(|n| n >= sizes.len());
        if stale {
            fs::remove_file(entry.path())?;
        }
    }
    let sha256 = hex::encode(whole.sha256.finalize());
    ensure!(
        manifest.source.sha256.as_ref() == Some(&sha256),
        "{} changed while setup read it: its SHA-256 is now {sha256}",
        ipsw.display()
    );
    if let Some(receipt) = &expected.sha256 {
        ensure!(
            &sha256 == receipt,
            "{}'s SHA-256 {sha256} differs from its download receipt's {receipt}; download it \
             again",
            ipsw.display()
        );
    }
    manifest.source.catalog_sha1 = if let Some((catalog, hash)) = whole.catalog {
        let sha1 = hex::encode(hash.finalize());
        ensure!(
            sha1.eq_ignore_ascii_case(catalog),
            "{}'s SHA-1 {sha1} differs from Apple's catalog digest {catalog}; download it again",
            ipsw.display()
        );
        Some(sha1)
    } else {
        None
    };
    manifest.layout = Layout::Pieces {
        piece_bytes,
        pieces,
        block_hashes: BlockHashes {
            file: PIECE_HASHES_FILE.into(),
            block_bytes: HASH_BLOCK_BYTES,
            sha256: stela_formats::sha256(&piece_blocks),
        },
    };
    manifest
        .validate()
        .map_err(|error| anyhow::anyhow!("the install medium's inventory: {error}"))?;
    efi::write_atomic(&directory.join(PIECE_HASHES_FILE), &piece_blocks, false)?;
    efi::write_atomic(&directory.join(BLOCK_HASHES_FILE), &inventory.blocks, false)?;
    let json = manifest.to_json();
    efi::write_atomic(&manifest_path, json.as_bytes(), false)?;
    written.manifest_sha256 = stela_formats::sha256(json.as_bytes());
    progress.finish();
    Ok(written)
}

/// Write a remote medium to `volume`: `inventory`, which names the IPSW on
/// Apple's servers ([`restore::remote_inventory`]), and its block hashes;
/// the SHA-256 of its `payload.json`.
pub(super) fn write_remote(
    volume: &Path,
    inventory: restore::payload::Inventory,
) -> Result<String> {
    ensure!(
        matches!(inventory.manifest.layout, Layout::Remote { .. }),
        "a remote medium's inventory names a remote image"
    );
    let directory = directory(volume, &inventory.manifest.product_build);
    fs::create_dir_all(&directory)?;
    efi::write_atomic(&directory.join(BLOCK_HASHES_FILE), &inventory.blocks, false)?;
    let json = inventory.manifest.to_json();
    efi::write_atomic(&directory.join(MANIFEST_FILE), json.as_bytes(), false)?;
    Ok(stela_formats::sha256(json.as_bytes()))
}

/// Remove what `previous`, the medium of the same build that `current`
/// replaced, left: its whole directory when it was elsewhere, or the IPSW's
/// pieces when a network medium took its directory. The files `current`
/// wrote stay.
pub(super) fn remove_replaced(previous: &Medium, current: &Medium) -> Result<()> {
    let directory = previous.removable_directory()?;
    if directory != current.directory() {
        if directory.is_dir() {
            fs::remove_dir_all(&directory)
                .with_context(|| format!("removing {}", directory.display()))?;
        }
        return Ok(());
    }
    if previous.remote || !current.remote {
        return Ok(());
    }
    for entry in fs::read_dir(&directory)? {
        let entry = entry?;
        let name = entry.file_name();
        let name = name.to_string_lossy();
        if name.starts_with("ipsw.") || name == PIECE_HASHES_FILE {
            fs::remove_file(entry.path())
                .with_context(|| format!("removing {}", entry.path().display()))?;
        }
    }
    Ok(())
}

#[cfg(test)]
pub(super) mod tests {
    use super::*;
    use {stela_formats::payload::PayloadManifest, stela_formats::payload::pieces::Extent};

    /// Small pieces, so the synthetic IPSW takes several.
    pub(in crate::setup) const PIECE_BYTES: u64 = HASH_BLOCK_BYTES;

    fn ipsw(dir: &Path) -> PathBuf {
        let path = dir.join("Synthetic.ipsw");
        efi::synthetic_ipsw(&path).unwrap();
        path
    }

    fn inventory(ipsw: &Path) -> restore::payload::Inventory {
        restore::payload::inventory(
            ipsw,
            restore::payload::SYNTHETIC_MODEL,
            restore::payload::KeyOptions::default(),
        )
        .unwrap()
    }

    fn expected(ipsw: &Path) -> Expected {
        let bytes = fs::read(ipsw).unwrap();
        Expected {
            sha256: Some(stela_formats::sha256(&bytes)),
            sha1: Some(hex::encode(Sha1::digest(&bytes))),
        }
    }

    /// An in-memory disk image holding the pieces back to back.
    struct Joined(Vec<u8>);

    impl stela_formats::payload::ReadAt for Joined {
        type Error = &'static str;
        fn len(&self) -> u64 {
            self.0.len() as u64
        }
        fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> Result<(), Self::Error> {
            let start = offset as usize;
            buffer.copy_from_slice(self.0.get(start..start + buffer.len()).ok_or("past end")?);
            Ok(())
        }
    }

    #[test]
    fn medium_parts_reassemble_into_ipsw() {
        let dir = tempfile::tempdir().unwrap();
        let ipsw = ipsw(dir.path());
        let volume = dir.path().join("STELA");
        fs::create_dir_all(&volume).unwrap();
        let bytes = fs::read(&ipsw).unwrap();
        let first = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &expected(&ipsw),
            piece_bytes: PIECE_BYTES,
        })
        .unwrap();
        let count = piece_sizes(bytes.len() as u64, PIECE_BYTES).len();
        assert!(count > 1, "the synthetic IPSW takes several pieces");
        assert_eq!((first.kept, first.wrote), (0, count));
        let directory = directory(&volume, restore::payload::SYNTHETIC_BUILD);
        let manifest =
            PayloadManifest::parse(&fs::read(directory.join(MANIFEST_FILE)).unwrap()).unwrap();
        assert_eq!(
            manifest.source.catalog_sha1.as_deref(),
            Some(hex::encode(Sha1::digest(&bytes)).as_str())
        );
        // The restore host's own reader joins the pieces into the IPSW,
        // checking every block against pieces.bin.
        let mut disk = Vec::new();
        let mut extents = Vec::new();
        for index in 0..count {
            let piece = fs::read(directory.join(piece_name(index))).unwrap();
            extents.push(vec![Extent {
                offset: disk.len() as u64,
                bytes: piece.len() as u64,
            }]);
            disk.extend(piece);
        }
        let hashes = fs::read(directory.join(PIECE_HASHES_FILE)).unwrap();
        let mut joined =
            stela_formats::payload::pieces::Joined::new(Joined(disk), &manifest, extents, &hashes)
                .unwrap();
        let mut read = vec![0; bytes.len()];
        stela_formats::payload::ReadAt::read_at(&mut joined, 0, &mut read).unwrap();
        assert_eq!(read, bytes);
        assert!(
            Medium {
                build: manifest.product_build.clone(),
                volume: volume.clone(),
                partition_guid: "G".into(),
                manifest_sha256: first.manifest_sha256.clone(),
                remote: false,
                damaged: Vec::new(),
            }
            .problems()
            .is_empty()
        );

        // A rerun keeps the complete pieces, rewrites a changed one and a
        // half-written one, and removes leftovers.
        let last = count - 1;
        fs::write(directory.join(piece_name(0)), vec![0; PIECE_BYTES as usize]).unwrap();
        fs::remove_file(directory.join(piece_name(last))).unwrap();
        let partial = directory.join(format!("{}.partial", piece_name(last)));
        fs::write(&partial, b"interrupted").unwrap();
        fs::write(directory.join(piece_name(count + 3)), b"stale").unwrap();
        let again = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &expected(&ipsw),
            piece_bytes: PIECE_BYTES,
        })
        .unwrap();
        assert_eq!((again.kept, again.wrote), (count - 2, 2));
        assert_eq!(again.manifest_sha256, first.manifest_sha256);
        assert_eq!(fs::read(directory.join(PIECE_HASHES_FILE)).unwrap(), hashes);
        assert!(!partial.exists());
        assert!(!directory.join(piece_name(count + 3)).exists());
        let joined: Vec<u8> = (0..count)
            .flat_map(|index| fs::read(directory.join(piece_name(index))).unwrap())
            .collect();
        assert_eq!(joined, bytes);

        // A short piece is rewritten too.
        fs::write(directory.join(piece_name(0)), b"short").unwrap();
        let short = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &expected(&ipsw),
            piece_bytes: PIECE_BYTES,
        })
        .unwrap();
        assert_eq!((short.kept, short.wrote), (count - 1, 1));
        assert_eq!(
            fs::read(directory.join(piece_name(0))).unwrap(),
            bytes[..PIECE_BYTES as usize]
        );

        let manual = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &Expected::default(),
            piece_bytes: PIECE_BYTES,
        })
        .unwrap();
        assert_eq!((manual.kept, manual.wrote), (count, 0));
        let manual_manifest =
            PayloadManifest::parse(&fs::read(directory.join(MANIFEST_FILE)).unwrap()).unwrap();
        assert_eq!(manual_manifest.source.sha256, manifest.source.sha256);
        assert!(manual_manifest.source.catalog_sha1.is_none());
    }

    #[test]
    fn ipsw_receipt_mismatch_blocks_manifest_publication() {
        let dir = tempfile::tempdir().unwrap();
        let ipsw = ipsw(dir.path());
        let volume = dir.path().join("STELA");
        let mut wrong = expected(&ipsw);
        wrong.sha1 = Some("0".repeat(40));
        let error = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &wrong,
            piece_bytes: PIECE_BYTES,
        })
        .unwrap_err()
        .to_string();
        assert!(
            error.contains("differs from Apple's catalog digest"),
            "{error}"
        );
        let directory = directory(&volume, restore::payload::SYNTHETIC_BUILD);
        assert!(!directory.join(MANIFEST_FILE).exists());
        let mut wrong = expected(&ipsw);
        wrong.sha256 = Some("0".repeat(64));
        let error = write(WriteOptions {
            volume: &volume,
            ipsw: &ipsw,
            inventory: inventory(&ipsw),
            expected: &wrong,
            piece_bytes: PIECE_BYTES,
        })
        .unwrap_err()
        .to_string();
        assert!(
            error.contains("differs from its download receipt"),
            "{error}"
        );
        assert!(!directory.join(MANIFEST_FILE).exists());
        let medium = Medium {
            build: restore::payload::SYNTHETIC_BUILD.into(),
            volume,
            partition_guid: "G".into(),
            manifest_sha256: String::new(),
            remote: false,
            damaged: Vec::new(),
        };
        assert_eq!(medium.problems().len(), 1);
    }

    #[test]
    fn remote_medium_uses_inventory_without_partition() {
        let dir = tempfile::tempdir().unwrap();
        let ipsw = ipsw(dir.path());
        let volume = dir.path().join("STELA");
        let bytes = fs::metadata(&ipsw).unwrap().len();
        let layout = Layout::Remote {
            url: "http://updates-http.example.test/Synthetic.ipsw".into(),
            etag: "\"e\"".into(),
        };
        let remote = restore::payload::remote_inventory(
            fs::File::open(&ipsw).unwrap(),
            restore::payload::RemoteInventoryOptions {
                image: &ipsw,
                bytes,
                layout,
                model: restore::payload::SYNTHETIC_MODEL,
                keys_in: None,
            },
        )
        .unwrap();
        assert!(write_remote(&volume, inventory(&ipsw)).is_err());
        let medium = Medium {
            build: restore::payload::SYNTHETIC_BUILD.into(),
            volume: volume.clone(),
            partition_guid: String::new(),
            manifest_sha256: write_remote(&volume, remote).unwrap(),
            remote: true,
            damaged: Vec::new(),
        };
        assert!(medium.problems().is_empty(), "{:?}", medium.problems());
        let mut names: Vec<_> = fs::read_dir(medium.directory())
            .unwrap()
            .map(|e| e.unwrap().file_name().into_string().unwrap())
            .collect();
        names.sort();
        assert_eq!(names, [BLOCK_HASHES_FILE, MANIFEST_FILE]);
        let mut system = System::default();
        medium.restore_from(&mut system).unwrap();
        assert_eq!(system.install, Some(medium.firmware_manifest()));
        assert_eq!(system.install_source, None);
        fs::remove_file(medium.directory().join(BLOCK_HASHES_FILE)).unwrap();
        assert_eq!(medium.problems().len(), 1);
    }

    #[test]
    fn only_an_install_directory_on_its_volume_is_removable() {
        let medium = |build: &str| Medium {
            build: build.into(),
            volume: PathBuf::from("/Volumes/STELA"),
            partition_guid: String::new(),
            manifest_sha256: String::new(),
            remote: false,
            damaged: Vec::new(),
        };
        assert_eq!(
            medium("26A434").removable_directory().unwrap(),
            Path::new("/Volumes/STELA/stela/install/26A434")
        );
        assert!(medium("../../../Users").removable_directory().is_err());
    }

    #[test]
    fn space_counts_only_the_pieces_still_to_write() {
        let dir = tempfile::tempdir().unwrap();
        let volume = dir.path();
        let build = "26A434";
        let bytes = 5 * HASH_BLOCK_BYTES + 7;
        let all = needed_bytes(volume, build, bytes, PIECE_BYTES);
        assert_eq!(all, MARGIN + bytes + 6 * 32);
        assert_eq!(piece_sizes(bytes, PIECE_BYTES).len(), 6);
        let directory = directory(volume, build);
        fs::create_dir_all(&directory).unwrap();
        fs::write(directory.join(piece_name(0)), vec![0; PIECE_BYTES as usize]).unwrap();
        assert_eq!(
            needed_bytes(volume, build, bytes, PIECE_BYTES),
            all - PIECE_BYTES
        );
        // A short piece is removed before its replacement is written: only
        // the bytes it lacks are needed.
        fs::write(directory.join(piece_name(1)), vec![0; 100]).unwrap();
        assert_eq!(
            needed_bytes(volume, build, bytes, PIECE_BYTES),
            all - PIECE_BYTES - 100
        );
    }
}
