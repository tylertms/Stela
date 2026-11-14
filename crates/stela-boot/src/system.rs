//! System mode, loader phase: everything that needs the firmware.
//!
//! It runs identity-mapped under the firmware. It keeps a pristine copy of
//! this image (taken before any Rust state exists), chooses the host
//! placement, places a relocated copy in the Stela window, allocates the
//! engine's memory, builds its page tables (five-level under LA57, else
//! four-level), releases the pristine copy, and leaves boot services. It
//! loads the tables through the LA57 trampoline when five-level and calls
//! the engine phase in the window.
use crate::Result;
use crate::{
    console::Framebuffer,
    firmware,
    handoff::{File, Handoff, MAGIC, MAX_FILES, MAX_GUEST_RANGES, MAX_NAME, Region, Storage},
    la57, layout, output, paging,
    selection::Refusal,
    storage,
    tables::{self, Frames, LARGE, NX, PAGE, PRESENT, UNCACHED, WRITABLE, WRITE_COMBINING},
};
use alloc::{format, string::String, vec::Vec};
use core::{arch::asm, ops::Range, ptr::NonNull};
use stela_boot_core::firmware_map::{self, RuntimeMapping};
use stela_boot_core::options::{DEFAULT_WIFI_KEY, System};
use stela_contracts::refuse;
use stela_engine::machine::mmu::placement::Placement;
use stela_runtime::pc;
use stela_runtime::pc::disk_writes::Pending;
use stela_runtime::pc::gpt::Guid;
use uefi::{
    boot::{self, AllocateType, MemoryType},
    fs::FileSystem,
    mem::memory_map::{MemoryMap, MemoryMapRef},
    proto::loaded_image::LoadedImage,
};

use crate::native_smp::resources::{Resources as NativeResources, STACK_BYTES, STACK_STRIDE};
mod handoff_phase;
mod memory;
const MIB: u64 = 1024 * 1024;
/// The loader heap's free bytes kept past boot services, for what still runs
/// before the engine does; the rest of it goes to the guest.
const LOADER_HEAP_KEPT: usize = 16 * MIB as usize;
const BELOW_4G: u64 = 0xffff_f000;
const RELOCATION_DIRECTORY: usize = 5;

/// The unique successful exit used to seal the native aperture capture.
pub(crate) struct EbsOwnership {
    _private: (),
}

/// Keeps the final memory map and its receipt together on the loader side.
pub(crate) struct SuccessfulExit {
    map: MemoryMapRef<'static>,
    ownership: Option<EbsOwnership>,
}

impl SuccessfulExit {
    pub(crate) fn map(&self) -> &MemoryMapRef<'static> {
        &self.map
    }

    pub(crate) fn take_ownership(&mut self) -> Option<EbsOwnership> {
        self.ownership.take()
    }
}
const IMAGE_REL_BASED_ABSOLUTE: u16 = 0;
const IMAGE_REL_BASED_DIR64: u16 = 10;

/// Private firmware pages owned only while boot services are active.
pub(crate) struct Snapshot {
    copy: NonNull<u8>,
    start: usize,
    bytes: usize,
}

impl Drop for Snapshot {
    fn drop(&mut self) {
        let physical = self.copy.as_ptr() as usize;
        // SAFETY: this owner holds exactly the pages allocated in take_snapshot.
        // The entry and system loader both drop it before exiting boot services.
        let result = unsafe { boot::free_pages(self.copy, self.bytes / PAGE as usize) };
        output::arguments(format_args!(
            "BOOT IMAGE snapshot physical={physical:#x} bytes={:#x} release={}\n",
            self.bytes,
            Released(result),
        ));
    }
}

/// Copy this image as the firmware loaded it, before Rust state changes it.
/// Only the firmware entry glue has run; its statics are not used later.
pub(crate) fn take_snapshot() -> Option<Snapshot> {
    let image = boot::open_protocol_exclusive::<LoadedImage>(boot::image_handle()).ok()?;
    let (base, bytes) = image.info();
    let extent = firmware::loaded_extent(base as usize, bytes).ok()?;
    let bytes = extent.mapped_bytes;
    // Taken before the loader's heap exists, so without the fallback of
    // `firmware::any_pages`, which reads the memory map into it.
    let copy = boot::allocate_pages(
        AllocateType::AnyPages,
        MemoryType::LOADER_DATA,
        bytes / PAGE as usize,
    )
    .ok()?;
    // SAFETY: exact checked firmware source extent and distinct fresh owned
    // pages. Zero only the destination tail, without reading source padding.
    unsafe {
        core::ptr::copy_nonoverlapping(base.cast::<u8>(), copy.as_ptr(), extent.bytes);
        copy.as_ptr()
            .add(extent.bytes)
            .write_bytes(0, bytes - extent.bytes);
    }
    Some(Snapshot {
        copy,
        start: base as usize,
        bytes,
    })
}

fn low_page(kind: MemoryType) -> Result<u64> {
    let page = boot::allocate_pages(AllocateType::MaxAddress(BELOW_4G), kind, 1)
        .map_err(|_| refuse!("no page below 4 GiB for the page-table root"))?;
    // SAFETY: a fresh page owned by Stela.
    unsafe { page.as_ptr().write_bytes(0, PAGE as usize) };
    Ok(page.as_ptr() as u64)
}

fn allocate(bytes: u64) -> Result<Region> {
    Ok(Region {
        physical: firmware::pages(bytes.div_ceil(PAGE) as usize)? as u64,
        bytes,
    })
}

/// The host RAM the guest and Stela's parts may take: the free and boot
/// services ranges in whole 2 MiB pages, as the guest's ranges are found
/// after boot services end.
fn free_ram() -> Result<u64> {
    let map = boot::memory_map(MemoryType::LOADER_DATA).map_err(|_| refuse!("no memory map"))?;
    Ok(map
        .entries()
        .filter(|entry| {
            matches!(
                entry.ty,
                MemoryType::CONVENTIONAL
                    | MemoryType::BOOT_SERVICES_CODE
                    | MemoryType::BOOT_SERVICES_DATA
            )
        })
        .map(|entry| {
            let start = entry.phys_start.next_multiple_of(LARGE);
            let end = (entry.phys_start + entry.page_count * PAGE) & !(LARGE - 1);
            end.saturating_sub(start)
        })
        .sum())
}

/// Return the loader heap's unused end to the firmware, whose free ranges
/// the guest's are found from: the heap is sized before anything is known of
/// what the boot will load. A refusal only leaves those pages to Stela.
fn release_loader_heap() {
    let Some(released) = crate::ALLOCATOR.release_tail(LOADER_HEAP_KEPT) else {
        return;
    };
    let pages = released.len() / PAGE as usize;
    let Some(start) = core::ptr::NonNull::new(released.start as *mut u8) else {
        return;
    };
    // SAFETY: LOADER_DATA pages of the heap's firmware allocation, which the
    // heap no longer manages and nothing else refers to.
    let result = unsafe { boot::free_pages(start, pages) };
    output::arguments(format_args!(
        "BOOT HEAP released {}M: {}\n",
        released.len() >> 20,
        Released(result)
    ));
}

/// A page release's result in the log: `ok`, or the firmware's status.
struct Released(uefi::Result);

impl core::fmt::Display for Released {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match &self.0 {
            Ok(()) => f.write_str("ok"),
            Err(error) => write!(f, "{:?}", error.status()),
        }
    }
}

/// Whether a firmware memory map entry is RAM: every type but the reserved,
/// unusable, MMIO and PAL code ones.
fn is_ram(ty: MemoryType) -> bool {
    !matches!(
        ty,
        MemoryType::RESERVED
            | MemoryType::UNUSABLE
            | MemoryType::MMIO
            | MemoryType::MMIO_PORT_SPACE
            | MemoryType::PAL_CODE
    )
}

fn read32(base: usize, offset: usize) -> usize {
    // SAFETY: callers stay inside the image copy.
    unsafe { ((base + offset) as *const u32).read_unaligned() as usize }
}

/// Apply the image's DIR64 base relocations for a move by `delta`.
fn relocate(copy: usize, bytes: usize, delta: u64) -> Result<()> {
    let header = read32(copy, 0x3c);
    if read32(copy, header) != 0x4550 {
        return Err(refuse!("image copy is not PE"));
    }
    let optional = header + 24;
    let directory = optional + 112 + RELOCATION_DIRECTORY * 8;
    let (mut block, end) = (
        read32(copy, directory),
        read32(copy, directory) + read32(copy, directory + 4),
    );
    if end > bytes {
        return Err(refuse!("relocation directory outside the image"));
    }
    while block < end {
        let page = read32(copy, block);
        let size = read32(copy, block + 4);
        if size < 8 || block + size > end {
            return Err(refuse!("malformed relocation block"));
        }
        for entry in (block + 8..block + size).step_by(2) {
            // SAFETY: inside the relocation directory checked above.
            let value = unsafe { ((copy + entry) as *const u16).read_unaligned() };
            let offset = page + usize::from(value & 0xfff);
            match value >> 12 {
                IMAGE_REL_BASED_ABSOLUTE => {}
                IMAGE_REL_BASED_DIR64 if offset + 8 <= bytes => {
                    let slot = (copy + offset) as *mut u64;
                    // SAFETY: a relocated pointer inside the copy.
                    unsafe { slot.write_unaligned(slot.read_unaligned().wrapping_add(delta)) };
                }
                _ => return Err(refuse!("unsupported base relocation")),
            }
        }
        block += size;
    }
    Ok(())
}

fn rdtsc() -> u64 {
    let (low, high): (u32, u32);
    // SAFETY: RDTSC has no side effects.
    unsafe { asm!("rdtsc", out("eax") low, out("edx") high, options(nomem, nostack)) };
    (u64::from(high) << 32) | u64::from(low)
}

/// Identity-map RAM RW+NX in 2 MiB pages where whole chunks are RAM, with the
/// listed pages given their own rights.
fn identity(
    frames: &mut dyn Frames,
    root: u64,
    ram: &[(usize, usize)],
    special: &[(u64, u64, u64)],
) -> Result<()> {
    let covered =
        |start: u64, end: u64| ram.iter().any(|r| r.0 as u64 <= start && end <= r.1 as u64);
    let top = ram.iter().map(|r| r.1 as u64).max().unwrap_or(0);
    for chunk in (0..top).step_by(LARGE as usize) {
        let end = chunk + LARGE;
        let has_special = special.iter().any(|s| s.0 < end && chunk < s.1);
        if covered(chunk, end) && !has_special {
            tables::map_large(frames, root, chunk, chunk, PRESENT | WRITABLE | NX)?;
            continue;
        }
        for page in (chunk..end).step_by(PAGE as usize) {
            let rights = if let Some(s) = special.iter().find(|s| s.0 <= page && page < s.1) {
                s.2
            } else if covered(page, page + PAGE) {
                PRESENT | WRITABLE | NX
            } else {
                continue;
            };
            // Page zero stays unmapped.
            if page != 0 {
                tables::map_page(frames, root, page, page, rights)?;
            }
        }
    }
    Ok(())
}

/// Bundle files by name, `bundle.json` first, each in its own pages.
type BundleFiles = Vec<(String, Region)>;

/// The names the restore inventory and its block hashes take among the
/// bundle files the engine receives.
pub const RESTORE_INVENTORY_FILE: &str = "restore-payload.json";
pub const RESTORE_BLOCKS_FILE: &str = "restore-blocks.bin";
pub const INSTALL_PIECES_FILE: &str = "install-pieces.bin";

/// The guest inputs: `source.bin`, or with `--bundle DIR` the directory's
/// `bundle.json` and every file it lists, read unchanged into their own
/// pages. The bundle reader opens the volumes itself, so the configured
/// volume is closed first. With `--preboot SERIAL:GUID` nothing is read: the
/// engine builds the bundle from that disk with the host storage drivers,
/// whose ranges are found here, as they are for every other option that
/// opens a host controller: disks, the NOR store, input and the networks.
fn inputs(options: &[u8], mut volume: FileSystem) -> Result<Inputs> {
    let text = core::str::from_utf8(options).map_err(|_| refuse!("stela.cfg is not UTF-8"))?;
    let system = System::parse(text).map_err(stela_boot_core::options::Error::message)?;
    #[cfg(not(feature = "diagnostics"))]
    if system.diagnostic_option().is_some() {
        return Err(refuse!("diagnostic options require a diagnostics build"));
    }
    // Read while the firmware's FAT driver runs; the engine reports a file
    // it cannot use.
    let wifi_key = system
        .ssid
        .as_ref()
        .and_then(|_| {
            let path = system.wifi_key.as_deref().unwrap_or(DEFAULT_WIFI_KEY);
            let path = uefi::CString16::try_from(path).ok()?;
            volume.read(uefi::fs::PathBuf::from(path)).ok()
        })
        .unwrap_or_default();
    let host_devices = system.storage.is_some()
        || system.preboot.is_some()
        || system.restore_source.is_some()
        || system.install_source.is_some()
        || system.nor.is_some()
        || system.network_check.is_some()
        || system.audio_check.is_some()
        || !system.usb_storage.is_empty()
        || system.input
        || system.install.is_some()
        || system.guest_network;
    let storage = if host_devices {
        crate::status::begin("Finding the PCI devices Stela drives itself");
        let found = storage::discover()?;
        crate::status::done("");
        if let Some(notice) = storage::gpu_notice_found() {
            crate::status::info(notice);
        }
        Some(found)
    } else {
        None
    };
    let (source, mut files, bundle_pes) = if system.preboot.is_some() {
        drop(volume);
        (Vec::new(), Vec::new(), None)
    } else if let Some(directory) = system.bundle_directory.as_deref() {
        drop(volume);
        let verify_topology = !(system.native_bootstrap_check || system.native_shared_check);
        let bundle = crate::engine::bundle::read(directory, verify_topology)?;
        (Vec::new(), bundle.files, bundle.processors)
    } else {
        let source = firmware::source(&mut volume)?;
        drop(volume);
        (source, Vec::new(), None)
    };
    let inventory = match (
        system.restore_manifest_path.as_deref(),
        system.install.as_deref(),
    ) {
        (Some(_), Some(_)) => {
            return Err(refuse!(
                "--install and --restore-manifest name one inventory"
            ));
        }
        (Some(path), None) => Some(crate::engine::bundle::restore_inventory(path)?),
        // The install medium's pieces are checked through the firmware's
        // FAT driver now, while it is there; the engine reads them through
        // its own after boot services end.
        (None, Some(path)) => {
            crate::status::begin("Checking the macOS install files on the USB");
            let inventory = crate::engine::bundle::with_inventory(
                path,
                system.install_source.map(|guid| guid.0),
                crate::engine::install::check_pieces,
            )?;
            crate::status::done("");
            Some(inventory)
        }
        (None, None) => None,
    };
    if let Some(inventory) = inventory {
        let reserved = [
            RESTORE_INVENTORY_FILE,
            RESTORE_BLOCKS_FILE,
            INSTALL_PIECES_FILE,
            stela_runtime::restore::fdr::FILE,
        ];
        if files
            .iter()
            .any(|(name, _)| reserved.contains(&name.as_str()))
        {
            return Err(refuse!(
                "a bundle file takes a name the restore inventory reserves"
            ));
        }
        files.push((String::from(RESTORE_INVENTORY_FILE), inventory.manifest));
        if let Some(diagnostic) = inventory.fdr_diagnostic {
            files.push((String::from(stela_runtime::restore::fdr::FILE), diagnostic));
        }
        if let Some(blocks) = inventory.blocks {
            files.push((String::from(RESTORE_BLOCKS_FILE), blocks));
        }
        if let Some(pieces) = inventory.pieces {
            files.push((String::from(INSTALL_PIECES_FILE), pieces));
        }
    }
    if files.len() > MAX_FILES {
        return Err(refuse!("bundle lists too many files"));
    }
    // The Metal device's pipelines, rewritten over the translation store's
    // partition.
    #[cfg(feature = "nvk")]
    if files.len() < MAX_FILES
        && let Some(store) = crate::engine::pipelines::read()
    {
        files.push((String::from(crate::engine::pipelines::FILE), store));
    }
    Ok(Inputs {
        source,
        files,
        bundle_pes,
        storage,
        wifi_key,
    })
}

/// Loader-owned source files, verified bundle cardinality and host inputs.
struct Inputs {
    source: Vec<u8>,
    files: BundleFiles,
    bundle_pes: Option<usize>,
    storage: Option<stela_drivers::Handoff>,
    wifi_key: Vec<u8>,
}

/// The firmware's runtime regions, and its `ResetSystem` service: zero when
/// the system table has none.
struct Runtime {
    /// (start, end, mapping) of each runtime region.
    regions: Vec<(u64, u64, RuntimeMapping)>,
    reset: u64,
}

impl Runtime {
    /// Read before boot services end. Without SetVirtualAddressMap the
    /// firmware's runtime services run at their physical addresses.
    fn capture() -> Result<Self> {
        let map =
            boot::memory_map(MemoryType::LOADER_DATA).map_err(|_| refuse!("no memory map"))?;
        let regions = map
            .entries()
            .filter(|entry| entry.att.bits() & firmware_map::MEMORY_RUNTIME != 0)
            .map(|entry| {
                let start = entry.phys_start;
                let mapping = firmware_map::runtime_mapping(entry.ty.0, entry.att.bits());
                (start, start + entry.page_count * PAGE, mapping)
            })
            .collect();
        let reset = uefi::table::system_table_raw().map_or(0, |table| {
            // SAFETY: the firmware's live system table, before EBS.
            let runtime = unsafe { table.as_ref().runtime_services };
            if runtime.is_null() {
                0
            } else {
                // SAFETY: as above; its runtime services table.
                unsafe { (*runtime).reset_system as usize as u64 }
            }
        });
        Ok(Self { regions, reset })
    }

    /// Runtime code, which the identity map makes writable and executable,
    /// as Linux maps it without a memory attributes table.
    fn code(&self) -> impl Iterator<Item = (u64, u64, u64)> + '_ {
        self.regions
            .iter()
            .filter(|&&(_, _, mapping)| mapping == RuntimeMapping::Code)
            .map(|&(start, end, _)| (start, end, PRESENT | WRITABLE))
    }

    /// Map the runtime regions outside RAM by identity: MMIO and regions
    /// the firmware does not mark write-back capable uncached, the rest
    /// write-back, code RWX and the rest RW+NX. A
    /// region overlapping RAM, which the identity map holds, or the storage
    /// registers, mapped already, is left as it is.
    fn map(
        &self,
        frames: &mut dyn Frames,
        root: u64,
        ram: &[(usize, usize)],
        devices: &[Range<u64>],
    ) -> Result<()> {
        for &(start, end, mapping) in &self.regions {
            let overlaps = ram.iter().any(|r| (r.0 as u64) < end && start < r.1 as u64)
                || devices
                    .iter()
                    .any(|r| r.start & !(PAGE - 1) < end && start < r.end.next_multiple_of(PAGE));
            if overlaps {
                continue;
            }
            let attributes = match mapping {
                RuntimeMapping::Uncached => NX | UNCACHED,
                RuntimeMapping::Code => 0,
                RuntimeMapping::WriteBack => NX,
            };
            for page in (start..end).step_by(PAGE as usize) {
                tables::map_page(frames, root, page, page, PRESENT | WRITABLE | attributes)?;
            }
        }
        Ok(())
    }
}

/// Identity-map the storage controllers' registers uncached; none may
/// overlap RAM.
fn map_devices(
    frames: &mut dyn Frames,
    root: u64,
    ram: &[(usize, usize)],
    ranges: &[Range<u64>],
) -> Result<()> {
    for range in ranges {
        let start = range.start & !(PAGE - 1);
        let end = range.end.next_multiple_of(PAGE);
        if ram.iter().any(|r| (r.0 as u64) < end && start < r.1 as u64) {
            return Err(refuse!("storage registers overlap RAM"));
        }
        tables::map_range(
            frames,
            root,
            start,
            start,
            end - start,
            PRESENT | WRITABLE | NX | UNCACHED,
        )?;
    }
    Ok(())
}

/// What the menu says when the machine can only be restored.
fn held(refusal: Refusal) -> &'static str {
    match refusal {
        Refusal::BlankDisk => "The Mac's SSD is not prepared yet: choose Install macOS first.",
        Refusal::NoRecovery { installed: false } => {
            "No macOS is installed and the Mac's SSD holds no System Recovery: choose Install \
             macOS."
        }
        Refusal::NoRecovery { installed: true } => {
            "NVRAM asks for a recoveryOS the SSD lacks. Install macOS erases the SSD."
        }
        Refusal::Stale(stale) => stale.screen(),
        _ => "The Mac's last install did not finish: choose Install macOS again.",
    }
}

/// Log which case makes the chosen disk stale, once per check.
fn log_stale(stale: stela_boot_core::chosen_disk::Stale) {
    output::arguments(format_args!("CHOSEN DISK stale: {}\n", stale.case()));
}

/// Why the default boot, `options`, can only wait for a restore, checked
/// before the startup menu opens so that the menu starts there. `pending`
/// is the disk work decided so far.
pub fn held_at_start(options: &[u8], pending: &Pending) -> Option<&'static str> {
    if !options.starts_with(b"--system") {
        return None;
    }
    if crate::choice::unchosen(options) {
        return Some("No disk is chosen for macOS yet: choose Install macOS first.");
    }
    crate::status::begin("Checking the internal SSD and NVRAM");
    let (held, found) = match crate::selection::peek(options.to_vec(), pending) {
        Err(refusal @ Refusal::BlankDisk) => (Some(held(refusal)), "no macOS installed yet"),
        Err(refusal @ Refusal::RecoveryMode) => (Some(held(refusal)), "an install did not finish"),
        Err(refusal @ Refusal::NoRecovery { .. }) => (Some(held(refusal)), "no recoveryOS to boot"),
        Err(Refusal::Stale(stale)) => {
            log_stale(stale);
            (Some(stale.screen()), stale.reason())
        }
        _ => (None, ""),
    };
    crate::status::done(found);
    held
}

/// Run the engine; never returns.
/// `pending` is the disk work decided so far, which the engine's drivers
/// do (`pc::disk_writes`): the change the disk picker planned for the disk
/// `options` name, and the partition updates. `log` is the partition
/// holding the USB's boot log, when the loader keeps one there
/// (`log_file`).
pub(crate) fn boot(
    mut options: Vec<u8>,
    mut pending: Pending,
    mut volume: FileSystem,
    log: Option<Guid>,
    host_firmware: crate::host_firmware::Resources,
    snapshot: Option<Snapshot>,
) -> ! {
    // NVRAM chooses macOS or recoveryOS before any input is read. A machine
    // NVRAM holds in recovery mode waits at the startup choices, as LLB
    // waits for a restore host, until one that boots is chosen.
    let options = loop {
        crate::status::begin("Reading the startup request in NVRAM");
        let routed = crate::selection::route(options, &mut pending);
        crate::status::done("");
        match routed {
            Ok(routed) => break routed,
            Err(Refusal::Other(error)) => output::fatal(error),
            Err(refusal) => {
                if let Refusal::Stale(stale) = &refusal {
                    log_stale(*stale);
                }
                // The new choice replaces the options and their plan: a
                // plan dropped here wrote nothing. The NVRAM consumed for
                // the refused boot stays consumed, unless the new plan
                // makes its partition afresh.
                let chosen = crate::choice::hold(&mut volume, held(refusal), &pending)
                    .unwrap_or_else(|| {
                        output::fatal(
                            "the machine can only be restored and this volume has no startup \
                             choices to restore it from",
                        )
                    });
                options = chosen.options;
                pending.replace_plan(chosen.plan);
                if !options.starts_with(b"--system") {
                    output::fatal("the chosen startup option is not a system-mode boot");
                }
                crate::status::take_screen(Framebuffer::from_firmware());
            }
        }
    };
    // `inputs` closes the volume: protocols must be closed before boot
    // services end.
    let inputs = inputs(&options, volume);
    let result = inputs.and_then(|inputs| {
        load(
            options,
            pending,
            inputs,
            log,
            host_firmware,
            snapshot.ok_or(refuse!("no pristine image copy"))?,
        )
    });
    match result {
        Ok(never) => match never {},
        Err(error) => output::fatal(error),
    }
}

/// Read the translation store, when a volume has one and a file slot is
/// left, within what this PC's free RAM leaves after Stela's parts and the
/// guest's floor (`host_memory::pack_budget`), less `pending` bytes the
/// loader allocates before it reserves the parts: a cache, it never stops
/// the boot. Rewriting its pack needs the storage drivers.
fn translation_store(
    mut files: BundleFiles,
    mut storage: Option<stela_drivers::Handoff>,
    cores: usize,
    pending: u64,
) -> (BundleFiles, Option<stela_drivers::Handoff>) {
    use crate::engine::translations::PackRead;
    crate::status::begin("Opening the translation cache");
    // Without a memory map the pack gets nothing; the parts' sizing
    // reports the map's absence.
    let free = free_ram().map_or(0, |free| free.saturating_sub(pending));
    let budget = stela_boot_core::host_memory::pack_budget(free, cores, cores);
    let Some(store) = crate::engine::translations::read(budget) else {
        crate::status::done("none on this USB");
        return (files, storage);
    };
    let mb = |bytes: u64| bytes / 1_000_000;
    let stays = "new translations stay in memory this boot";
    match store.pack {
        PackRead::Skipped => crate::status::warn(&format!(
            "left on the USB: this PC's memory has no room for it; {stays}"
        )),
        PackRead::Part { read, held } => crate::status::warn(&format!(
            "{} of {} MB, as much as this PC's memory holds",
            mb(read),
            mb(held)
        )),
        PackRead::Whole(bytes) if store.writable => {
            crate::status::done(&format!("{} MB", mb(bytes)));
        }
        PackRead::None if store.writable => crate::status::done("empty"),
        PackRead::Whole(_) | PackRead::None if budget == 0 => {
            crate::status::warn(&format!("this PC's memory has no room for it; {stays}"))
        }
        PackRead::Whole(bytes) => crate::status::warn(&format!(
            "{} MB; {stays}: `stela repair` on another computer prepares the file",
            mb(bytes)
        )),
        PackRead::None => crate::status::warn(&format!(
            "{stays}: `stela repair` on another computer prepares the file"
        )),
    }
    if files.len() < MAX_FILES {
        files.push((
            String::from(crate::engine::translations::FILE),
            store.region,
        ));
        if store.writable && storage.is_none() {
            storage = storage::discover().ok();
        }
    }
    (files, storage)
}

fn load(
    options: Vec<u8>,
    pending: Pending,
    inputs: Inputs,
    log: Option<Guid>,
    host_firmware: crate::host_firmware::Resources,
    snapshot: Snapshot,
) -> Result<core::convert::Infallible> {
    let Inputs {
        source,
        files: bundle,
        bundle_pes,
        storage,
        wifi_key,
    } = inputs;
    let text = core::str::from_utf8(&options).map_err(|_| refuse!("stela.cfg is not UTF-8"))?;
    let system = System::parse(text).map_err(stela_boot_core::options::Error::message)?;
    // The engine's drivers apply the plan to the `--storage` partition's
    // disk and no other.
    if let Some(plan) = &pending.plan
        && !system
            .storage
            .as_ref()
            .is_some_and(|target| plan.is_for(target))
    {
        return Err(refuse!(
            "the disk chosen for macOS is not this boot's --storage, so it cannot be prepared; \
             nothing was written to it"
        ));
    }
    let disk_writes = if pending.is_empty() {
        Vec::new()
    } else {
        pending.encode()
    };
    let (native_processors, available_cores) = crate::native_smp::firmware::capture_inventory()
        .map_err(crate::native_smp::firmware::Error::message)?;
    let cores = system.assigned_native_cores(available_cores, bundle_pes)?;
    // The translation cache takes only what the parts and the guest leave,
    // so it is read last before they are reserved: only the engine image's
    // copy comes between.
    let image_bytes = crate::native_image::ResidentImage::allocation_bytes(snapshot.bytes);
    let (bundle, storage) = translation_store(bundle, storage, cores, image_bytes);
    crate::status::begin("Setting up Stela's memory");
    let image = memory::prepare_image(&snapshot)?;
    let mut memory = memory::reserve(cores, &image.ram)?;
    let tsc_hz = crate::firmware::tsc_hz();
    if !crate::cpu::INVARIANT_TSC.present() {
        output::arguments(format_args!(
            "UNSUPPORTED host TSC is not invariant: guest time and every timeout follow the cores' clock changes\n"
        ));
    }
    let wall = read_rtc();
    let power = host_power();
    if let Some(power) = &power {
        crate::stop::arm(power, tsc_hz);
    }
    let random_seed = match crate::entropy::retain_seed() {
        Ok((region, source)) => {
            output::arguments(format_args!("HOST entropy seeded source={source}\n"));
            region
        }
        Err(error) => {
            output::arguments(format_args!("UNSUPPORTED host entropy: {error}\n"));
            Region::default()
        }
    };

    let remapping = crate::native_dma::firmware_units()?;
    let amd_tables = crate::native_dma::reserve_amd_tables(&remapping);

    let mut native_mmio = crate::native_mmio::Prepared::capture().map_err(|error| {
        output::arguments(format_args!("NATIVE MMIO unavailable capture={error}\n"));
        alloc::format!("the firmware's PCI and ACPI settings were not copied: {error}")
    });

    let runtime = Runtime::capture()?;
    let storage = memory::map(
        &image,
        &mut memory,
        &runtime,
        storage.as_ref(),
        native_mmio.as_mut().ok(),
        &host_firmware,
    )?;
    let memory::EngineImage {
        image,
        placement,
        screen,
        resident,
        ..
    } = image;
    let memory::EngineMemory {
        heap,
        native_resources,
        native_task_allocations,
        stack,
        code,
        pool,
        handoff_page,
        trampoline,
        root,
        loader_tables,
        sipi_taken,
    } = memory;
    let loader_tables = loader_tables
        .retain()
        .map_err(|_| refuse!("loader table ownership validation failed"))?;
    let bounce = native_resources.bounce[0];
    let handoff_phase::RetainedFiles {
        options: options_region,
        source: source_region,
        files,
        file_count,
        disk_writes,
    } = handoff_phase::retain_files(options, source, bundle, disk_writes)?;
    let handoff = handoff_page.physical as *mut Handoff;
    let engine_entry =
        layout::image() + (crate::engine::entry as *const () as u64 - image.start as u64);

    let mut boot_disk = [0u8; crate::handoff::BOOT_DISK_BYTES];
    let mut boot_disk_bytes = 0;
    if let Some(location) = crate::installer::boot_disk_location() {
        output::arguments(format_args!(
            "BOOT DISK the firmware found it at {location}\n"
        ));
        let mut take = location.len().min(boot_disk.len());
        while !location.is_char_boundary(take) {
            take -= 1;
        }
        boot_disk[..take].copy_from_slice(&location.as_bytes()[..take]);
        boot_disk_bytes = take;
    }
    let (device_skips, device_needs) = device_starts(&system);
    // Found now, through the firmware's own protocol; claimed as the first
    // act after the firmware's exit.
    let usb_legacy = crate::usb_legacy::find();
    crate::status::done("");
    let boot_heap = crate::ALLOCATOR.usage();
    output::arguments(format_args!(
        "BOOT HEAP total={}M peak={}M live={}M\n",
        boot_heap.total >> 20,
        boot_heap.peak >> 20,
        boot_heap.live >> 20,
    ));
    release_loader_heap();
    // Reservations have already been sized and the relocated image owns its
    // bytes. The final firmware map can now offer the pristine pages to macOS.
    drop(snapshot);
    // SAFETY: the fresh immutable MP observation is held LOADER_DATA. Identity
    // mappings and the calibrated local APIC remain valid across EBS.
    let capture = unsafe {
        crate::native_smp::firmware::Capture::read(native_processors.physical as *const u8)
    };
    // The final map's seal runs after the log closes; this one is its record.
    if let Ok(prepared) = native_mmio.as_mut() {
        output::arguments(format_args!(
            "NATIVE MMIO memory map before exit: {}\n",
            prepared.rehearse()
        ));
    }
    crate::status::begin("Taking control of the other processors");
    // The last allocation Stela makes before the exit: from here to the
    // exit's own map read only the firmware's timer and logs close.
    let final_map = crate::boot_exit::Buffer::allocate();
    crate::output::leave_boot_services();
    // SAFETY: no protocol or pool allocation outlives this call; every page
    // Stela keeps is LOADER_DATA or LOADER_CODE.
    let map = unsafe { crate::boot_exit::exit(final_map) };
    let mut exit = SuccessfulExit {
        map,
        ownership: Some(EbsOwnership { _private: () }),
    };
    // The firmware's exception handlers ended with its boot services. Until
    // the engine's root is entered a fault shows on the screen instead of
    // resetting the PC with no record.
    // SAFETY: boot services have ended; interrupts stay off from here to
    // the engine's own tables, and no firmware handler runs again.
    unsafe {
        asm!("cli", options(nomem, nostack));
        crate::paging::report_faults();
    }
    // The log is closed from here to the engine's start, so a freeze in
    // this step shows only on the screen: each part names itself there.
    // Before anything stops a controller: firmware that still drives USB
    // from system management mode hangs there when its controller's bus
    // mastering ends under it (issue 36). Linux claims them this early too.
    crate::status::progress("the firmware has left; its USB controllers");
    // SAFETY: boot services ended above with interrupts off, the firmware's
    // identity map is loaded and no Stela driver has started.
    unsafe { usb_legacy.take(tsc_hz) };
    crate::status::progress("the firmware's DMA remapping");
    let dmar_flags = remapping.dmar_flags();
    // SAFETY: the firmware's identity map is still loaded, the APs idle in
    // firmware code, and no Stela driver or memory reuse has started.
    let inactive_firmware_units =
        unsafe { crate::native_dma::release_firmware_units(remapping, tsc_hz) };
    crate::status::progress("the final memory map");
    let native_mmio_capture = match native_mmio {
        Ok(prepared) => match prepared.finish(&mut exit) {
            Ok(sealed) => crate::native_mmio::CaptureHandoff::new(sealed),
            Err(error) => {
                output::arguments(format_args!("NATIVE MMIO unavailable final-map={error}\n"));
                crate::native_mmio::CaptureHandoff::refused(&alloc::format!("{error}"))
            }
        },
        Err(why) => crate::native_mmio::CaptureHandoff::refused(&why),
    };
    // SAFETY: Stop every enabled firmware AP before offering its former code/data to
    // guest RAM, including workers that will later receive their owned SIPI.
    // The BSP alone sends ICR commands, with firmware IRQ handlers excluded.
    unsafe { asm!("cli", options(nomem, nostack)) };
    // x2APIC whenever CPUID offers it and the firmware's DMAR does not opt
    // out, from here on: the firmware no longer runs on this processor, and
    // every later APIC handle is made from the base returned here (the
    // handoff's) or read afresh. The APs take the same mode as they enter
    // Stela (`native_smp::executor`).
    crate::status::progress("the local APIC");
    let keep_xapic = crate::apic::honours_x2apic_opt_out(
        dmar_flags,
        capture.processors[..capture.processor_count as usize]
            .iter()
            .filter(|processor| processor.flags & stela_boot_core::topology::ENABLED != 0)
            .map(|processor| processor.apic_id),
    );
    // SAFETY: CPL0, interrupts masked, no APIC handle of this processor made.
    let apic_base = unsafe { crate::apic::choose_mode(keep_xapic) };
    let sender = crate::apic::Apic::new(apic_base, apic_base & crate::apic::BASE_ADDRESS);
    output::arguments(format_args!(
        "HOST APIC {}\n",
        match (sender.is_x2apic(), keep_xapic) {
            (true, _) => "x2APIC",
            (false, true) => "xAPIC (the firmware's DMAR opts out of x2APIC)",
            (false, false) => "xAPIC (no x2APIC offered)",
        }
    ));
    crate::status::progress("stopping the firmware's processors");
    // SAFETY: interrupts are masked, the BSP alone sends ICR commands, and `capture` lists the
    // enabled firmware APs.
    let deliveries = match unsafe {
        crate::native_smp::executor::quiesce_firmware_aps(capture, &sender, tsc_hz)
    } {
        Ok(deliveries) => deliveries,
        Err(error) => {
            output::fatal(format_args!(
                "firmware processor handoff refused before memory reuse: {error:?}"
            ));
        }
    };
    output::arguments(format_args!(
        "NATIVE FIRMWARE INIT deliveries={deliveries}; delivery only, no execution acknowledgement\n"
    ));
    crate::status::done("");
    // Keep this post-EBS boundary when the relocated engine replays status.
    let mut history = [0; crate::status::HISTORY_BYTES];
    let history_bytes = crate::status::history(&mut history);
    let mut host = [0; crate::host_inventory::HANDOFF_BYTES];
    let host_bytes = crate::host_inventory::save(&mut host);
    let handoff_phase::GuestMemory {
        ranges: guest,
        count,
        installed,
    } = handoff_phase::guest_memory(exit.map());
    // The other cores' startup page must still be Stela's in the final map:
    // a boot services page taken from an earlier map that something
    // allocated since would be overwritten by the engine.
    let sipi = native_resources.sipi;
    if sipi != 0
        && !stela_boot_core::startup_page::kept(
            exit.map()
                .entries()
                .map(|entry| (entry.ty.0, entry.phys_start, entry.page_count)),
            sipi,
            sipi_taken,
        )
    {
        output::fatal(format_args!(
            "the page below 1 MiB the other cores start from ({sipi:#x}) was allocated to \
             something else before the firmware exited"
        ));
    }
    // The lines since the firmware's log closed, which no log stored, for
    // the engine's record: the USB's log takes them when it resumes.
    let loader_lines = {
        let physical = handoff_page.physical + size_of::<Handoff>() as u64;
        // SAFETY: the handoff allocation's tail, sized for these lines and
        // used by nothing else; the firmware's identity map is loaded.
        let into = unsafe {
            core::slice::from_raw_parts_mut(
                physical as *mut u8,
                stela_boot_core::loader_lines::BYTES,
            )
        };
        Region {
            physical,
            bytes: output::unkept_lines(into) as u64,
        }
    };
    // SAFETY: fresh LOADER_DATA pages sized for the handoff.
    unsafe {
        handoff.write(Handoff {
            magic: MAGIC,
            options: options_region,
            disk_writes,
            random_seed,
            source: source_region,
            files,
            file_count,
            heap,
            stack,
            code,
            pool,
            placement,
            root,
            bounce,
            screen,
            history,
            history_bytes,
            loader_lines,
            host,
            host_bytes,
            log_partition: log.map(|guid| guid.0),
            boot_disk,
            boot_disk_bytes,
            tsc_hz,
            tsc_source: crate::firmware::tsc_source(),
            tsc_tolerance_ppm: crate::firmware::tsc_tolerance_ppm(),
            apic_base,
            native_processors,
            native_resources,
            native_mmio: native_mmio_capture.region().unwrap_or_default(),
            wall,
            guest,
            guest_count: count,
            installed,
            storage,
            host_firmware,
            power,
            efi_reset: runtime.reset,
            rsdp: storage::rsdp().unwrap_or(0),
            wifi_key: Region {
                physical: wifi_key.as_ptr() as u64,
                bytes: wifi_key.len() as u64,
            },
            native_task_allocations,
            native_loader_tables: loader_tables,
            native_image: resident,
            native_mmio_capture,
            inactive_firmware_units,
            amd_tables,
            device_skips,
            device_needs,
            stop_record: crate::stop_record::services(),
        });
    }
    // SAFETY: boot services ended; the completed root maps this code and
    // trampoline, and all engine-window reservations remain allocated.
    unsafe {
        handoff_phase::Transfer {
            placement,
            root,
            trampoline,
            engine_entry,
            handoff,
        }
        .enter()
    }
}

/// The PCI functions an earlier boot did not finish starting, as the USB's
/// log gave them, and, when there are any, the controllers of the disks
/// `system` names, which the engine never skips.
fn device_starts(system: &System) -> (pc::device_starts::Skips, pc::device_starts::Needed) {
    let skips = crate::log_file::device_skips();
    let mut needs = pc::device_starts::Needed::EMPTY;
    if skips.is_empty() {
        return (skips, needs);
    }
    let serials: Vec<&str> = [
        &system.storage,
        &system.preboot,
        &system.restore_source,
        &system.nor,
    ]
    .into_iter()
    .flatten()
    .map(|target| target.serial.as_str())
    .collect();
    for address in crate::selection::disk_controllers(&serials) {
        output::arguments(format_args!(
            "HOST DEVICE NEEDED pci={address}: a disk this boot needs is on it\n"
        ));
        needs.add(address);
    }
    (skips, needs)
}

/// The host's ACPI power hardware, reported while the firmware's tables
/// are still mapped; the engine gets it without the report.
pub(crate) fn host_power() -> Option<pc::power::HostPower> {
    let rsdp = storage::rsdp()?;
    // SAFETY: the firmware's RSDP, its tables mapped while boot services last.
    match unsafe { pc::power::discover(rsdp) } {
        Ok(power) => {
            if let Some(reason) = power.refused {
                output::arguments(format_args!("POWER host refusal: {reason}\n"));
            }
            Some(pc::power::HostPower {
                refused: None,
                ..power
            })
        }
        Err(error) => {
            output::arguments(format_args!("POWER host has no usable FADT: {error:?}\n"));
            None
        }
    }
}

/// The firmware's real-time clock, read between two TSC values. An
/// unspecified time zone is read as UTC, as Linux reads the RTC and as QEMU
/// and Macs keep it; a time the firmware cannot give is reported.
fn read_rtc() -> Option<crate::handoff::RtcReading> {
    use stela_boot_core::firmware_time::{self, UnspecifiedZone};
    let tsc_before = rdtsc();
    let read = uefi::runtime::get_time_and_caps();
    let tsc_after = rdtsc();
    let (time, capabilities) = read
        .inspect_err(|error| {
            output::arguments(format_args!(
                "HOST RTC unavailable: GetTime failed: {:?}\n",
                error.status()
            ))
        })
        .ok()?;
    let fields = firmware_time::Time {
        year: time.year(),
        month: time.month(),
        day: time.day(),
        hour: time.hour(),
        minute: time.minute(),
        second: time.second(),
        nanosecond: time.nanosecond(),
        timezone_minutes: time.time_zone().unwrap_or(firmware_time::UNSPECIFIED_ZONE),
        daylight: time.daylight().bits(),
    };
    let utc = firmware_time::utc(fields, UnspecifiedZone::RtcIsUtc)
        .inspect_err(|error| output::arguments(format_args!("HOST RTC unusable: {error}\n")))
        .ok()?;
    Some(crate::handoff::RtcReading {
        utc,
        resolution: capabilities.resolution,
        tsc_before,
        tsc_after,
    })
}
