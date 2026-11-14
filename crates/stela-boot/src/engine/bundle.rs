//! The kernel boot bundle: `bundle.json` and the files it lists, as prepared
//! on the host by `stela-dev boot prepare` (the iBoot side of the hand-off).
//! `stela_formats::boot_bundle` checks every region's SHA-256, writes the
//! regions and their zero tails into guest DRAM, and fills the patch slots
//! only the running machine knows; this supplies the machine's side: guest
//! RAM through the engine's host, seeded host entropy and the timebase.
//! The format is `stela_formats::boot_bundle`'s.
//!
//! [`read`] runs before boot services end and reads the files into their own
//! `LOADER_DATA` pages, so a restore ramdisk does not have to fit the heap.
//! [`from_disk`] runs after them instead: Stela's own storage drivers read
//! the installed system on a host disk's APFS container, and
//! `stela_loader::installed` builds the same bundle `stela-dev boot prepare
//! --disk` writes, in the heap. [`upgrade_from_disk`] and
//! [`recovery_from_disk`] build a staged update's and a recoveryOS's the
//! same way (`stela_loader::upgrade`, `stela_loader::recovery`).
use crate::Result;
use crate::{
    handoff::{File, MAX_FILES, Region},
    layout, output,
    paging::PAGE,
};
use alloc::string::ToString;
use alloc::{boxed::Box, format, string::String, vec, vec::Vec};
use core::{arch::asm, ops::RangeInclusive, ptr::NonNull};
pub(crate) use stela_boot_core::bundle::{live_tree, source};
use stela_contracts::refuse;
use stela_drivers::block::{BlockDevice, BlockError};
use stela_engine::machine::Host;
use stela_formats::boot_bundle::Framebuffer;
use stela_formats::boot_bundle::Machine;
use stela_formats::boot_bundle::Manifest;
use stela_formats::boot_bundle::Nvram;
use stela_formats::boot_bundle::{MANIFEST_FILE, MAX_MANIFEST_BYTES};
use stela_formats::payload;
use stela_formats::payload::PayloadManifest;
use stela_formats::payload::ReadAt;
use stela_loader::{
    apple_device_tree::DeviceTree, boot_bundle::Options, boot_selection::BootVolume, boot_trace,
    installed, recovery, upgrade,
};
use uefi::{
    CString16, boot,
    proto::media::{
        file::{Directory, File as _, FileAttribute, FileInfo, FileMode, RegularFile},
        fs::SimpleFileSystem,
    },
};

/// The counter frequency the machine presents in CNTFRQ_EL0.
const TIMEBASE_HZ: u64 = 24_000_000;
pub struct Entry {
    pub pc: u64,
    pub x0: u64,
    /// Verified selected PE in prepared/live ADT order, never source index.
    pub boot_processor: Option<usize>,
    /// Original CLPC class paths with verified hidden admission.
    pub hidden_clpc: Vec<String>,
    pub dram_base: u64,
    pub regions: usize,
    pub patches: usize,
    pub kept: Vec<String>,
    pub hidden_devices: Vec<String>,
    /// Guest RAM after all fixed-offset runtime patches, not its input file.
    pub live_tree: live_tree::LiveTree,
    /// Verified immutable original input and exact selected manifest metadata.
    pub source: source::Intake,
    /// The AMCC CTRR-A region iBoot programs, `(base, bytes)`.
    pub read_only_region: Option<(u64, u64)>,
    /// What iBoot leaves in the cores' lock registers (M4 and later).
    pub core_locks: stela_engine::machine::ctrr::CoreLocks,
}

pub(super) fn contents(file: &File) -> &'static [u8] {
    // SAFETY: the loader left every bundle file allocated; the physmap maps it.
    unsafe {
        core::slice::from_raw_parts(
            layout::physical(file.contents.physical) as *const u8,
            file.contents.bytes as usize,
        )
    }
}

/// `file`'s contents to change: once, by the one owner of the file.
///
/// # Safety
/// Nothing else reads or writes `file`'s contents for as long as the slice
/// lives.
pub(super) unsafe fn contents_mut(file: &File) -> &'static mut [u8] {
    // SAFETY: the loader left every bundle file allocated; the physmap maps
    // it, and the caller owns it alone.
    unsafe {
        core::slice::from_raw_parts_mut(
            layout::physical(file.contents.physical) as *mut u8,
            file.contents.bytes as usize,
        )
    }
}

fn open(root: &mut Directory, path: &str) -> Result<RegularFile> {
    let path = CString16::try_from(path).map_err(|_| refuse!("bundle path is not UCS-2"))?;
    root.open(&path, FileMode::Read, FileAttribute::empty())
        .map_err(|_| refuse!("cannot open a bundle file"))?
        .into_regular_file()
        .ok_or(refuse!("a bundle path is a directory"))
}

fn size(file: &mut RegularFile) -> Result<u64> {
    Ok(file
        .get_boxed_info::<FileInfo>()
        .map_err(|_| refuse!("cannot read a bundle file's size"))?
        .file_size())
}

/// The part of a large file read between progress updates.
const PROGRESS_STEP: usize = 16 << 20;

/// Firmware pages held privately until their caller commits the read.
struct ReadPages {
    start: NonNull<u8>,
    count: usize,
    length: usize,
}

impl ReadPages {
    fn bytes(&self) -> &[u8] {
        // SAFETY: only a complete read leaves `read_sized`, and these pages
        // remain owned for the duration of the borrow.
        unsafe { core::slice::from_raw_parts(self.start.as_ptr(), self.length) }
    }

    fn retain(self) -> Region {
        let region = Region {
            physical: self.start.as_ptr() as u64,
            bytes: self.length as u64,
        };
        core::mem::forget(self);
        region
    }
}

impl Drop for ReadPages {
    fn drop(&mut self) {
        // SAFETY: this is the exact, unpublished allocation made before EBS.
        let _ = unsafe { boot::free_pages(self.start, self.count) };
    }
}

fn read_staged(root: &mut Directory, path: &str, sizes: RangeInclusive<u64>) -> Result<ReadPages> {
    let mut file = open(root, path)?;
    let bytes = size(&mut file)?;
    if !sizes.contains(&bytes) {
        return Err(refuse!("file size outside configured bound", %path, bytes));
    }
    read_sized(file, path, bytes)
}

/// Preserve the restore inventory's per-file retention contract.
fn read_bounded(root: &mut Directory, path: &str, sizes: RangeInclusive<u64>) -> Result<Region> {
    read_staged(root, path, sizes).map(ReadPages::retain)
}

/// Read a size-checked file into private `LOADER_DATA` pages.
fn read_sized(mut file: RegularFile, path: &str, bytes: u64) -> Result<ReadPages> {
    let length = usize::try_from(bytes)
        .ok()
        .filter(|&length| length <= isize::MAX as usize)
        .ok_or(refuse!("bundle file is too large"))?;
    let count = length.div_ceil(PAGE).max(1);
    let pages = ReadPages {
        start: crate::firmware::any_pages(count)
            .map_err(|_| refuse!("firmware has no memory for Stela"))?,
        count,
        length,
    };
    // SAFETY: fresh pages owned exclusively by this read and sized for length.
    let buffer = unsafe { core::slice::from_raw_parts_mut(pages.start.as_ptr(), length) };
    // Large files are read in parts, with their progress on the screen.
    let name = path.rsplit('\\').next().unwrap_or(path);
    if length > PROGRESS_STEP {
        crate::status::progress_fraction(&format!("{name}  0 of {} MB", length >> 20), 0, bytes);
    } else {
        // Do not leave the preceding large file's completed bar on a new file.
        crate::status::progress(name);
    }
    let mut done = 0;
    while done < length {
        let end = length.min(done + PROGRESS_STEP);
        let count = file
            .read(&mut buffer[done..end])
            .map_err(|_| refuse!("cannot read a bundle file"))?;
        if count == 0 {
            return Err(refuse!("a bundle file ended early"));
        }
        done += count;
        if length > PROGRESS_STEP {
            crate::status::progress_fraction(
                &format!("{name}  {} of {} MB", done >> 20, length >> 20),
                done as u64,
                bytes,
            );
        }
    }
    Ok(pages)
}

pub(crate) struct Retained {
    pub files: Vec<(String, Region)>,
    pub processors: Option<usize>,
}

/// Read `bundle.json` from `directory` on the first volume that has it, then
/// every file it lists, unchanged; `bundle.json` comes first. Run before boot
/// services end, with no other exclusive opening of the volume.
pub(crate) fn read(directory: &str, verify_topology: bool) -> Result<Retained> {
    let directory = directory.trim_end_matches('\\');
    let manifest_path = format!("{directory}\\{MANIFEST_FILE}");
    let handles =
        boot::find_handles::<SimpleFileSystem>().map_err(|_| refuse!("no file systems"))?;
    for handle in handles {
        let Ok(mut volume) = boot::open_protocol_exclusive::<SimpleFileSystem>(handle) else {
            continue;
        };
        let Ok(mut root) = volume.open_volume() else {
            continue;
        };
        let Ok(mut manifest_file) = open(&mut root, &manifest_path) else {
            continue;
        };
        let Ok(bytes) = size(&mut manifest_file) else {
            continue;
        };
        if bytes > MAX_MANIFEST_BYTES as u64 {
            return Err(refuse!("bundle.json exceeds the manifest size limit"));
        }
        let Ok(json) = read_sized(manifest_file, &manifest_path, bytes) else {
            continue;
        };
        let manifest = Manifest::parse(json.bytes()).map_err(|error| {
            output::arguments(format_args!("bundle refusal: {error}\n"));
            "bundle.json is not a boot bundle"
        })?;
        if manifest.files.len() >= MAX_FILES {
            return Err(refuse!("bundle lists too many files"));
        }
        let mut files = Vec::with_capacity(manifest.files.len() + 1);
        files.push((String::from(MANIFEST_FILE), json));
        crate::status::begin("Reading the boot files");
        for file in &manifest.files {
            let path = format!("{directory}\\{}", file.name);
            output::arguments(format_args!("READ {path}\n"));
            let contents = read_staged(&mut root, &path, file.bytes..=file.bytes)
                .inspect_err(|error| crate::status::fail(&error.to_string()))?;
            files.push((file.name.clone(), contents));
        }
        let processors = verify_topology
            .then(|| {
                source::processor_count(&manifest, &|name| {
                    files
                        .iter()
                        .find(|(file, _)| file == name)
                        .map(|(_, pages)| pages.bytes())
                })
            })
            .transpose()?;
        let bytes: u64 = files.iter().map(|(_, pages)| pages.length as u64).sum();
        crate::status::done(&format!("{} MB", bytes >> 20));
        // Reserve the handoff list before relinquishing any page owner.
        let mut retained = Vec::with_capacity(files.len());
        for (name, pages) in files {
            retained.push((name, pages.retain()));
        }
        return Ok(Retained {
            files: retained,
            processors,
        });
    }
    Err(refuse!("no volume has the bundle's bundle.json"))
}

/// The restore inventory as it was read, and what it names beside itself.
pub struct Inventory {
    pub manifest: Region,
    /// Explicit opt-in experiment beside the inventory; never factory trust.
    pub fdr_diagnostic: Option<Region>,
    /// Its members' precomputed block hashes (`payload::BlockHashes`), when
    /// it names them.
    pub blocks: Option<Region>,
    /// An install medium's piece block hashes (`Layout::Pieces`).
    pub pieces: Option<Region>,
}

/// The most bytes a hash file may hold: a million blocks' hashes.
const MAX_HASH_FILE_BYTES: u64 = payload::MAX_HASH_BLOCKS * 32;

/// Bounded restore inventory, read before ExitBootServices, with the hash
/// files it names from the same directory of the same volume. Reject
/// ambiguous paths across firmware volumes rather than choosing a different
/// inventory.
pub fn restore_inventory(path: &str) -> Result<Inventory> {
    with_inventory(path, None, |_, _, _| Ok(()))
}

/// [`restore_inventory`], also running `check` on the inventory's volume
/// while it is open, with the inventory's directory. With `source`, the
/// install medium's GPT partition, only the volume on that partition is
/// searched (`stela_boot_core::bundle::inventory_volume`).
pub fn with_inventory(
    path: &str,
    source: Option<[u8; 16]>,
    mut check: impl FnMut(&mut Directory, &str, &PayloadManifest) -> Result<()>,
) -> Result<Inventory> {
    const LIMIT: u64 = 16 * 1024 * 1024;
    let mut found = None;
    for handle in
        boot::find_handles::<SimpleFileSystem>().map_err(|_| refuse!("no file systems"))?
    {
        if !stela_boot_core::bundle::inventory_volume(
            source,
            source.and_then(|_| super::translations::partition(handle)),
        ) {
            continue;
        }
        let Ok(mut volume) = boot::open_protocol_exclusive::<SimpleFileSystem>(handle) else {
            continue;
        };
        let Ok(mut root) = volume.open_volume() else {
            continue;
        };
        if open(&mut root, path).is_err() {
            continue;
        }
        if found.is_some() {
            return Err(refuse!(
                "restore inventory path appears on multiple volumes"
            ));
        }
        let manifest = read_bounded(&mut root, path, 0..=LIMIT)?;
        // SAFETY: just read into Stela's own pages.
        let bytes = unsafe {
            core::slice::from_raw_parts(manifest.physical as *const u8, manifest.bytes as usize)
        };
        let parsed = PayloadManifest::parse(bytes).map_err(|error| {
            output::arguments(format_args!("restore inventory refusal: {error}\n"));
            "the restore inventory is not a payload manifest"
        })?;
        let directory = path
            .rsplit_once('\\')
            .map_or("", |(directory, _)| directory);
        let mut hash_file = |entry: Option<&payload::BlockHashes>| {
            entry
                .map(|entry| {
                    let path = format!("{directory}\\{}", entry.file);
                    output::arguments(format_args!("READ {path}\n"));
                    read_bounded(&mut root, &path, 0..=MAX_HASH_FILE_BYTES).map_err(|_| {
                        refuse!("a hash file the restore inventory names cannot be read")
                    })
                })
                .transpose()
        };
        let blocks = hash_file(parsed.block_hashes.as_ref())?;
        let pieces = hash_file(match &parsed.layout {
            payload::Layout::Pieces { block_hashes, .. } => Some(block_hashes),
            _ => None,
        })?;
        let diagnostic_path = format!("{directory}\\{}", stela_runtime::restore::fdr::FILE);
        let diagnostic_name = CString16::try_from(diagnostic_path.as_str())
            .map_err(|_| refuse!("FDR diagnostic path is not UCS-2"))?;
        let fdr_diagnostic =
            match root.open(&diagnostic_name, FileMode::Read, FileAttribute::empty()) {
                Ok(file) => {
                    drop(file);
                    let seed = read_bounded(
                        &mut root,
                        &diagnostic_path,
                        0..=stela_runtime::restore::fdr::MAX_BYTES as u64,
                    )?;
                    let state_path = format!("{directory}\\{}", super::fdr::STATE_FILE);
                    let image = read_bounded(
                        &mut root,
                        &state_path,
                        0..=stela_runtime::pc::fdr_store::FILE_BYTES as u64,
                    )
                    .map_err(|_| refuse!("FDR diagnostic needs its preallocated fdrstate.bin"))?;
                    let partition = super::translations::partition(handle)
                        .ok_or(refuse!("FDR storage volume needs a GPT partition identity"))?;
                    // SAFETY: both regions were just read into kept loader pages.
                    let bytes = |region: Region| unsafe {
                        core::slice::from_raw_parts(
                            region.physical as *const u8,
                            region.bytes as usize,
                        )
                    };
                    let path = state_path.trim_start_matches('\\').replace('\\', "/");
                    Some(
                        super::fdr::prepare(partition, &path, bytes(seed), bytes(image)).map_err(
                            |error| {
                                output::arguments(format_args!("FDR state refusal: {error}\n"));
                                "FDR persisted state could not be loaded"
                            },
                        )?,
                    )
                }
                Err(error) if error.status() == uefi::Status::NOT_FOUND => None,
                Err(_) => return Err(refuse!("cannot open the FDR diagnostic")),
            };
        check(&mut root, directory, &parsed)?;
        found = Some(Inventory {
            manifest,
            fdr_diagnostic,
            blocks,
            pieces,
        });
    }
    found.ok_or(refuse!("restore inventory file was not found"))
}

/// The size of the file at `path` of `root`, when it is one.
pub fn file_size(root: &mut Directory, path: &str) -> Result<u64> {
    let mut file = open(root, path)?;
    Ok(file
        .get_boxed_info::<FileInfo>()
        .map_err(|_| refuse!("cannot read a file's size"))?
        .file_size())
}

/// The largest transfer [`Bytes`] asks a device for.
const TRANSFER_BYTES: usize = 1 << 20;

/// Byte-addressed reads over a block device, for the APFS reader.
pub(crate) struct Bytes<D> {
    device: D,
    block: Vec<u8>,
}

impl<D: BlockDevice> Bytes<D> {
    pub(crate) fn new(device: D) -> Self {
        let block = vec![0; device.identity().block_size as usize];
        Self { device, block }
    }

    pub(crate) fn into_inner(self) -> D {
        self.device
    }
}

impl<D: BlockDevice> ReadAt for Bytes<D> {
    type Error = BlockError;

    fn len(&self) -> u64 {
        self.device.identity().bytes()
    }

    fn read_at(&mut self, mut offset: u64, mut buffer: &mut [u8]) -> Result<(), BlockError> {
        let size = self.block.len() as u64;
        while !buffer.is_empty() {
            let lba = offset / size;
            let within = (offset % size) as usize;
            let whole = buffer.len().min(TRANSFER_BYTES) / size as usize * size as usize;
            let done = if within == 0 && whole > 0 {
                self.device.read(lba, &mut buffer[..whole])?;
                whole
            } else {
                self.device.read(lba, &mut self.block)?;
                let count = buffer.len().min(self.block.len() - within);
                buffer[..count].copy_from_slice(&self.block[within..within + count]);
                count
            };
            offset += done as u64;
            buffer = &mut buffer[done..];
        }
        Ok(())
    }
}

/// Build the boot bundle of the installed system on `partition`, an APFS
/// container or a disk image with a GUID partition table, as `request`
/// names it, with `options` (`stela-dev boot prepare`'s defaults and the boot's
/// own boot-args); returns `bundle.json` first and then every file it
/// lists. Runs after boot services end, through Stela's own drivers.
pub fn from_disk(
    partition: Box<dyn BlockDevice>,
    request: &installed::Request,
    options: &Options,
) -> Result<Vec<(String, Vec<u8>)>> {
    let (offset, installed) =
        installed::prepare_disk(&mut Bytes::new(partition), request, options)?;
    output::arguments(format_args!(
        "PREBOOT {} volume={:?} container={offset:#x} objects={} named-by={}\n",
        installed.set.directory(),
        installed.preboot.name,
        installed.set.objects.len(),
        installed
            .evidence
            .named_by
            .as_deref()
            .unwrap_or("evidence alone")
    ));
    let manifest = serde_json::to_vec(&installed.bundle.manifest)
        .map_err(|error| refuse!("bundle.json: {error}", %error))?;
    let mut files = vec![(String::from(MANIFEST_FILE), manifest)];
    files.extend(installed.bundle.files);
    Ok(files)
}

/// Build the boot bundle of the macOS update staged on `partition`'s Update
/// volume (`stela_loader::upgrade`), with `options` plus the update's own
/// boot-args and OS environment; returns `bundle.json` first. Runs after
/// boot services end.
pub fn upgrade_from_disk(
    partition: Box<dyn BlockDevice>,
    volume: Option<BootVolume>,
    options: &Options,
) -> Result<Vec<(String, Vec<u8>)>> {
    let mut options = options.clone();
    options.boot_args = upgrade_boot_args(&options.boot_args);
    options.osenvironment = Some(upgrade::ENVIRONMENT);
    let (offset, staged) =
        upgrade::prepare_disk(&mut Bytes::new(partition), volume, None, &options)?;
    output::arguments(format_args!(
        "UPGRADE {} container={offset:#x} objects={} firmware-set-aside={}\n",
        staged.volume.name,
        staged.update.root.len(),
        staged.update.other_firmware.len()
    ));
    let manifest = serde_json::to_vec(&staged.bundle.manifest)
        .map_err(|error| refuse!("bundle.json: {error}", %error))?;
    let mut files = vec![(String::from(MANIFEST_FILE), manifest)];
    files.extend(staged.bundle.files);
    Ok(files)
}

/// Build the boot bundle of the recoveryOS of `kind` on `partition`
/// (`stela_loader::recovery`): the paired recoveryOS of the boot container's
/// Recovery volume, or System Recovery of the RecoveryOSContainer, as
/// NVRAM's boot `volume` names them; returns `bundle.json` first. Runs
/// after boot services end.
pub fn recovery_from_disk(
    partition: Box<dyn BlockDevice>,
    kind: recovery::Kind,
    volume: Option<BootVolume>,
    options: &Options,
) -> Result<Vec<(String, Vec<u8>)>> {
    let (offset, recovered) = recovery::prepare_disk(
        &mut Bytes::new(partition),
        kind,
        volume.as_ref(),
        None,
        options,
    )?;
    output::arguments(format_args!(
        "RECOVERY {} {} volume={:?} container={offset:#x} objects={} named-by={}\n",
        kind.label(),
        recovered.set.directory(),
        recovered.volume.name,
        recovered.set.objects.len(),
        recovered
            .evidence
            .named_by
            .as_deref()
            .unwrap_or("evidence alone")
    ));
    let manifest = serde_json::to_vec(&recovered.bundle.manifest)
        .map_err(|error| refuse!("bundle.json: {error}", %error))?;
    let mut files = vec![(String::from(MANIFEST_FILE), manifest)];
    files.extend(recovered.bundle.files);
    Ok(files)
}

/// The update ramdisk's boot-args: the console args kept, plus iBoot's
/// `rd=md0`, `-progress` and `-restore` that the restore kexts read.
fn upgrade_boot_args(console: &str) -> String {
    let mut args: Vec<&str> = console.split_ascii_whitespace().collect();
    for extra in upgrade::BOOT_ARGS {
        let key = extra.split('=').next().unwrap_or(extra);
        if !args
            .iter()
            .any(|a| *a == key || a.split('=').next() == Some(key))
        {
            args.push(extra);
        }
    }
    args.join(" ")
}

/// Fill from the boot's checked, seeded cryptographic stream.
pub(super) fn entropy(buffer: &mut [u8]) -> Result<()> {
    crate::random::fill(buffer).map_err(|_| refuse!("host cryptographic entropy is unavailable"))
}

/// The bundle's `bundle.json`, from the files the loader read or a bundle
/// built in the heap.
pub(super) fn manifest(files: &[File], built: Option<&[(String, Vec<u8>)]>) -> Result<Manifest> {
    let json = match built {
        Some(built) => built
            .iter()
            .find(|(name, _)| name == MANIFEST_FILE)
            .map(|(_, bytes)| bytes.as_slice()),
        None => files
            .iter()
            .find(|file| file.name() == MANIFEST_FILE)
            .map(contents),
    }
    .ok_or(refuse!("bundle.json was not read"))?;
    Manifest::parse(json).map_err(|error| refuse!("bundle manifest: {error}", %error))
}

/// The guest's DRAM: `backed` bytes of host RAM from `base`, within the
/// `installed` bytes the Mac reports.
#[derive(Debug, Clone, Copy)]
pub struct Dram {
    pub base: u64,
    pub backed: u64,
    pub installed: u64,
}

/// Load the bundle the loader read into guest DRAM `dram`, with the guest
/// framebuffer when the machine shows one.
pub fn load(
    host: &mut impl Host,
    files: &[File],
    dram: Dram,
    framebuffer: Option<Framebuffer>,
    nvram: Option<Nvram>,
) -> Result<Entry> {
    let named = |name: &str| files.iter().find(|file| file.name() == name).map(contents);
    load_named(host, &named, dram, framebuffer, nvram)
}

/// Load a bundle built in the heap, as [`from_disk`] returns it.
pub fn load_built(
    host: &mut impl Host,
    files: &[(String, Vec<u8>)],
    dram: Dram,
    framebuffer: Option<Framebuffer>,
    nvram: Option<Nvram>,
) -> Result<Entry> {
    let named = |name: &str| {
        files
            .iter()
            .find(|(file, _)| file == name)
            .map(|(_, bytes)| bytes.as_slice())
    };
    load_named(host, &named, dram, framebuffer, nvram)
}

fn load_named<'a>(
    host: &mut impl Host,
    named: &impl Fn(&str) -> Option<&'a [u8]>,
    dram: Dram,
    framebuffer: Option<Framebuffer>,
    nvram: Option<Nvram>,
) -> Result<Entry> {
    let mut trace = boot_trace::Trace::new(TIMEBASE_HZ as u32)
        .map_err(|error| refuse!("boot trace: {error}", %error))?;
    trace_record(host, &mut trace, *b"SVFY")?;
    let json = named(MANIFEST_FILE).ok_or(refuse!("bundle.json was not read"))?;
    let manifest =
        Manifest::parse(json).map_err(|error| refuse!("bundle manifest: {error}", %error))?;
    let verified = manifest
        .verify(&|name| named(name))
        .map_err(|error| refuse!("bundle verification: {error}", %error))?;
    // The new reader contract is checked before any guest RAM load/patch.
    let source = source::Intake::from_verified(json, &verified, named)?;
    trace_record(host, &mut trace, *b"EVFY")?;
    // Use the verified tree region, including its file offset. The trace
    // must stay in the original reserved range below managed memory.
    let trace_region = manifest
        .regions
        .iter()
        .find(|region| region.name == "DeviceTree")
        .map(|region| -> Result<_> {
            let file =
                named(&region.file).ok_or(refuse!("verified DeviceTree file disappeared"))?;
            let start = usize::try_from(region.file_offset)
                .map_err(|_| refuse!("DeviceTree offset overflows"))?;
            let length = usize::try_from(region.bytes)
                .map_err(|_| refuse!("DeviceTree length overflows"))?;
            let end = start
                .checked_add(length)
                .ok_or(refuse!("DeviceTree range wraps"))?;
            let bytes = file
                .get(start..end)
                .ok_or(refuse!("DeviceTree range is out of bounds"))?;
            let tree = DeviceTree::parse(bytes)
                .map_err(|error| refuse!("boot trace tree: {error}", %error))?;
            super::remember_panic_region(&tree);
            let trace = boot_trace::Region::from_tree(&tree)
                .map_err(|error| refuse!("boot trace region: {error}", %error))?;
            if let Some(trace) = trace {
                trace
                    .validate(
                        dram.base,
                        manifest.options.phys_base,
                        manifest
                            .regions
                            .iter()
                            .map(|region| (region.guest_pa, region.memory_bytes)),
                    )
                    .map_err(|error| refuse!("boot trace placement: {error}", %error))?;
            }
            Ok(trace)
        })
        .transpose()?
        .flatten();
    let machine = Machine {
        dram_base: dram.base,
        ram_bytes: dram.backed,
        dram_bytes: dram.installed,
        timebase_hz: TIMEBASE_HZ,
        framebuffer,
        boot_args: None,
        nvram,
    };
    trace_record(host, &mut trace, *b"SLOD")?;
    let image = verified
        .load(&machine, &mut entropy, &mut |pa, data| {
            host.write_ram(pa, data)
        })
        .map_err(|error| refuse!("kernel load: {error}", %error))?;
    let core_locks = manifest
        .core_locks(&machine)
        .map_err(|error| refuse!("core locks: {error}", %error))?
        .unwrap_or_default();
    trace_record(host, &mut trace, *b"ELOD")?;
    if let Some(region) = trace_region {
        if !host.write_ram(region.base, trace.bytes()) {
            return Err(refuse!("boot trace guest RAM write failed"));
        }
        output::arguments(format_args!(
            "BOOTTRACE format=BTRD origin=stela-native stages=verify,load records=4 hz={TIMEBASE_HZ} pa={:#x} bytes={}\n",
            region.base,
            trace.bytes().len()
        ));
    }
    // The load blanked them before it wrote the regions.
    output::arguments(format_args!(
        "CARVEOUTS blank count={} bytes={:#x}\n",
        manifest.carveouts.len(),
        manifest
            .carveouts
            .iter()
            .map(|carveout| carveout.bytes)
            .sum::<u64>()
    ));
    for region in &image.regions {
        output::arguments(format_args!(
            "REGION {} pa={:#x} bytes={:#x} memory={:#x}\n",
            region.name, region.guest_pa, region.bytes, region.memory_bytes
        ));
    }
    let live_tree = live_tree::LiveTree::capture(host, &image.regions, image.x0)?;
    source.validate_live(live_tree.bytes())?;
    Ok(Entry {
        pc: image.entry_pa,
        x0: image.x0,
        boot_processor: Some(source.boot_processor()),
        hidden_clpc: source.hidden_clpc().to_vec(),
        dram_base: dram.base,
        regions: image.regions.len(),
        patches: image.patches.len(),
        kept: image.kept,
        hidden_devices: manifest.hidden_devices,
        live_tree,
        source,
        read_only_region: manifest
            .read_only_region
            .map(|region| (region.base, region.bytes)),
        core_locks,
    })
}

/// Observe the same 24 MHz counter the first guest slice reads. No guest
/// slice has run yet, so no execution lag or idle leap has been charged.
/// The inline native RIP explicitly identifies Stela as the record source.
#[inline(always)]
fn trace_record(host: &mut impl Host, trace: &mut boot_trace::Trace, tag: [u8; 4]) -> Result<()> {
    let pc: u64;
    // SAFETY: obtains this native instruction address without accessing memory.
    unsafe {
        asm!("lea {pc}, [rip + 2f]", "2:", pc = out(reg) pc, options(nomem, nostack, preserves_flags));
    }
    trace
        .record(host.counter(), pc, tag, 0)
        .map_err(|error| refuse!("boot trace record: {error}", %error))
}
