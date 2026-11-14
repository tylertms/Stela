//! Plain data the loader phase passes to the engine phase. Addresses are host
//! physical; the engine reaches them through its physmap.
use crate::console::Framebuffer;
use stela_engine::machine::mmu::placement::Placement;
use stela_runtime::pc;

pub const MAGIC: u64 = 0x5354_454c_4153_5953; // "SYSALETS"
pub const MAX_GUEST_RANGES: usize = 128;
/// A restore bundle's files (13 for a MacBookPro18,3 with its USB-C port
/// controllers' firmware) and the install inventory's four, with room.
pub const MAX_FILES: usize = 32;
pub use stela_boot_core::handoff_records::{File, MAX_NAME, Region, Storage};

#[repr(C)]
pub struct Handoff {
    pub magic: u64,
    /// `stela.cfg`.
    pub options: Region,
    /// The disk work decided before the firmware exited (`pc::disk_writes`,
    /// encoded), which the engine's drivers do once: the change Install
    /// macOS's disk picker planned for the `--storage` partition's disk,
    /// before that partition is looked up, and then the updates of the
    /// boot's partitions, before anything reads them. Empty when nothing
    /// is pending. It lives only in this memory.
    pub disk_writes: Region,
    /// Exclusive 256-bit pre-EBS seed, erased by the engine on consumption.
    /// An empty region means no checked entropy source was available.
    pub random_seed: Region,
    /// `source.bin`: the guest image, when no boot bundle is configured.
    pub source: Region,
    /// The boot bundle's `bundle.json` and every file it lists.
    pub files: [File; MAX_FILES],
    pub file_count: usize,
    pub heap: Region,
    pub stack: Region,
    pub code: Region,
    /// Page-table frames: engine roots and shadow tables.
    pub pool: Region,
    /// Where the guest's address space lies beside the window.
    pub placement: Placement,
    /// The loader's root, whose window entries every engine root shares.
    pub root: u64,
    /// The physical addresses of the bounce slot's two leaf entries.
    pub bounce: [u64; 2],
    pub screen: Option<Framebuffer>,
    /// The loader's finished status lines, which the engine's screen starts
    /// with (`status::replay`).
    pub history: [u8; crate::status::HISTORY_BYTES],
    pub history_bytes: usize,
    /// The loader's lines that no log stored, after the firmware's log
    /// closed (`output::unkept_lines`), in the handoff allocation's tail,
    /// which the engine's record starts with (`output::carry_loader_lines`).
    pub loader_lines: Region,
    /// The loader's facts about the PC, as a host record, which the engine
    /// takes up (`host_inventory::restore`).
    pub host: [u8; crate::host_inventory::HANDOFF_BYTES],
    pub host_bytes: usize,
    /// The partition holding the USB's boot log the loader keeps
    /// (`log_file`), which the engine writes on to.
    pub log_partition: Option<[u8; 16]>,
    /// Where the firmware found the disk Stela started from
    /// (`pci_io::disk_location`), as text, for the account a failed disk
    /// search gives.
    pub boot_disk: [u8; BOOT_DISK_BYTES],
    pub boot_disk_bytes: usize,
    /// TSC ticks per second (`firmware::tsc_hz`).
    pub tsc_hz: u64,
    /// How `tsc_hz` was found.
    pub tsc_source: crate::firmware::TscSource,
    /// The most `tsc_hz` may be off by, in parts per million.
    pub tsc_tolerance_ppm: u32,
    /// IA32_APIC_BASE as the firmware left it.
    pub apic_base: u64,
    /// Immutable copied UEFI processor identities and per-physical-core raw
    /// CPUID/control observations; absent unless native cores were requested.
    pub native_processors: Region,
    pub native_resources: crate::native_smp::resources::Resources,
    /// Sealed owned UEFI/ACPI resource inventory and empty MMIO aperture.
    /// Zero when capture is unavailable; this alone grants no device owner.
    pub native_mmio: Region,
    /// The firmware's real-time clock at boot; `None` when the firmware has
    /// no valid time.
    pub wall: Option<RtcReading>,
    /// Free host RAM after boot services, in 2 MiB-aligned ranges.
    pub guest: [Region; MAX_GUEST_RANGES],
    pub guest_count: usize,
    /// The host's RAM in the firmware's memory map, every RAM type
    /// included: the DRAM the guest Mac reports.
    pub installed: u64,
    /// The host storage drivers' ranges, for a bundle built from a disk.
    pub storage: Storage,
    /// Host-device firmware, separate from guest boot-bundle files.
    pub host_firmware: crate::host_firmware::Resources,
    /// The host's ACPI power hardware, when its FADT is usable.
    pub power: Option<pc::power::HostPower>,
    /// The firmware's `ResetSystem` runtime service, whose regions the
    /// loader keeps mapped by identity; zero without one.
    pub efi_reset: u64,
    /// The firmware's ACPI RSDP; zero without one.
    pub rsdp: u64,
    /// The `--wifi-key` file's bytes, read from the ESP; empty when
    /// `--wifi` names no network or the file could not be read. The engine
    /// parses and wipes them.
    pub wifi_key: Region,
    /// Firmware-created task allocation owners; the preceding wire fields keep
    /// their offsets. The engine can claim each owner once, after mapping checks.
    pub(crate) native_task_allocations: crate::native_smp::resources::TaskAllocations,
    /// Original loader table allocations; root is only their copied descriptor.
    pub(crate) native_loader_tables: crate::paging::loader::LoaderTables,
    /// Moved original allocation of the relocated static Rust/C engine image.
    /// Observed PE/header ranges cannot recreate this non-Clone owner.
    pub(crate) native_image: crate::native_image::ResidentImage,
    /// Original aperture capture sealed with this boot's successful exit.
    pub(crate) native_mmio_capture: crate::native_mmio::CaptureHandoff,
    /// Original remapping inventory qualified at firmware exit.
    pub(crate) inactive_firmware_units: crate::native_dma::InactiveFirmwareUnits,
    /// Exclusive firmware RAM below 4 GiB for AMD-Vi device tables.
    pub(crate) amd_tables: Region,
    /// The PCI functions an earlier boot did not finish starting, or
    /// skipped since, as the USB's log gives them (`log_file::device_skips`).
    pub(crate) device_skips: pc::device_starts::Skips,
    /// The controllers of the disks this boot cannot continue without,
    /// which it never skips (`selection::disk_controllers`).
    pub(crate) device_needs: pc::device_starts::Needed,
    /// The firmware's variable services, whose regions the loader keeps
    /// mapped by identity as it does `efi_reset`'s, and this boot's number
    /// in the USB's log: what a stop the log cannot keep leaves its record
    /// with (`stop_record`).
    pub(crate) stop_record: crate::stop_record::Services,
    /// Whether the loader found a UART at COM1 (`serial::present`). The
    /// engine runs a copy of this image taken before any Rust state
    /// existed, so it learns the result here, not from its own statics.
    pub(crate) serial_present: bool,
}

/// A read of the firmware's real-time clock: its time as UTC and its
/// resolution in counts per second, read between TSC values `tsc_before`
/// and `tsc_after`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct RtcReading {
    pub utc: stela_boot_core::firmware_time::Timestamp,
    pub resolution: u32,
    pub tsc_before: u64,
    pub tsc_after: u64,
}

impl RtcReading {
    /// The Unix seconds at TSC `tsc`, carried forward at `tsc_hz`.
    pub fn unix_seconds_at(&self, tsc: u64, tsc_hz: u64) -> u64 {
        self.utc.seconds + tsc.saturating_sub(self.tsc_after) / tsc_hz
    }
}

/// The most [`Handoff::boot_disk`] holds.
pub const BOOT_DISK_BYTES: usize = 128;

impl Handoff {
    /// [`Self::boot_disk`] as text, when the loader found it.
    pub fn boot_disk(&self) -> Option<&str> {
        let bytes = self.boot_disk.get(..self.boot_disk_bytes)?;
        core::str::from_utf8(bytes)
            .ok()
            .filter(|text| !text.is_empty())
    }
}
