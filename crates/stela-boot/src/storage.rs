//! The host storage hook: what `stela.efi` records for the runtime's PC
//! storage drivers before boot services end, and the disk report and check
//! it runs after.
//!
//! `--storage SERIAL:GUID` in `stela.cfg` opens one partition and reports it
//! without writing. `--storage-check SERIAL:GUID` also overwrites the start
//! of that partition with a known pattern, reads it back and reports its
//! hash. Without either option nothing here touches the hardware.
mod bluetooth_platform;
#[cfg(feature = "diagnostics")]
mod media_check;
#[cfg(feature = "diagnostics")]
mod minimum;
use crate::Result;
use stela_boot_core::storage as options;
use stela_contracts::refuse;
#[cfg(feature = "diagnostics")]
mod reset_check;
pub use options::split;

use crate::{output, paging::Tables};
use alloc::{sync::Arc, vec::Vec};
use core::time::Duration;
use stela_drivers::{
    Clock, Handoff, PlatformCaptures,
    block::BlockDevice,
    bus::{dma::RegionAllocator, pci},
    probe::probe,
};
use stela_runtime::pc::{
    acpi,
    storage::{self, Target},
};
use uefi::{
    boot::{self, AllocateType, MemoryType, OpenProtocolAttributes, OpenProtocolParams},
    proto::{
        nvme::{NvmeQueueType, NvmeRequestBuilder, pass_thru::NvmePassThru},
        pci::{configuration::ResourceRangeType, root_bridge::PciRootBridgeIo},
    },
    table::cfg::ConfigTableEntry,
};

/// 16 MiB below 4 GiB: NVMe's bounce buffers, the USB and SATA rings, and
/// each Ethernet controller's 256-descriptor rings.
const DMA_PAGES: usize = 4096;
/// `max_host_mem_size_mb`: the most host memory buffer one NVMe controller
/// gets.
const HOST_MEMORY_MAXIMUM: u64 = 128 << 20;
/// Bytes the check writes: several transfers, past one PRP list page.
const CHECK_BYTES: usize = 1024 * 1024 + 3 * 4096;

pub struct Prepared {
    handoff: Handoff,
    captures: PlatformCaptures,
    open: Vec<Target>,
    check: Vec<Target>,
    #[cfg(feature = "diagnostics")]
    usb_check: Option<(UsbCheck, minimum::Prepared)>,
}

#[cfg(feature = "diagnostics")]
enum UsbCheck {
    Reset,
    Media,
}

/// The board captures the drivers read after ExitBootServices, taken while
/// firmware variables are reachable; `wifi` comes from the loader's files.
pub fn captures(wifi: stela_drivers::net::wifi::intel::platform::Observations) -> PlatformCaptures {
    PlatformCaptures {
        bluetooth: bluetooth_platform::capture(),
        wifi,
    }
}

/// Whether a function whose probe failed this way could hold a disk: a
/// storage, USB or unnamed device's failure.
pub(crate) fn may_hold_disks(
    devices: &stela_drivers::probe::Devices,
    address: pci::Address,
    error: &stela_drivers::probe::Error,
) -> bool {
    matches!(
        devices.subsystem(address, error),
        "storage" | "USB" | "device"
    )
}

/// Report a PCI function probe could not bring up, by its subsystem.
pub(crate) fn report_failure(
    devices: &stela_drivers::probe::Devices,
    address: pci::Address,
    error: &stela_drivers::probe::Error,
) {
    output::arguments(format_args!(
        "HOST DEVICE FAILED pci={address} ({}) error={error}\n",
        devices.subsystem(address, error)
    ));
}

pub fn rsdp() -> Option<u64> {
    uefi::system::with_config_table(|entries| {
        let find = |guid| entries.iter().find(|entry| entry.guid == guid);
        find(ConfigTableEntry::ACPI2_GUID)
            .or_else(|| find(ConfigTableEntry::ACPI_GUID))
            .map(|entry| entry.address as u64)
    })
}

/// Before ExitBootServices: find ECAM, reserve DMA pages, measure the TSC
/// and collect the storage controllers' register ranges.
pub fn prepare(
    options: &[u8],
    firmware: Option<Arc<stela_drivers::firmware::Catalog>>,
    captures: PlatformCaptures,
) -> Result<Option<Prepared>> {
    let text = core::str::from_utf8(options).map_err(|_| refuse!("stela.cfg is not UTF-8"))?;
    let (open, check, _) = split(text)?;
    let usb_reset_check = options::reset_requested(text, &open, &check)?;
    let usb_media_check = options::media_requested(text, &open, &check)?;
    options::usb_hpet_handoff(text, usb_reset_check || usb_media_check)?;
    #[cfg(not(feature = "diagnostics"))]
    if usb_reset_check || usb_media_check {
        return Err(refuse!("USB checks require a diagnostics build"));
    }
    if open.is_empty() && check.is_empty() {
        return Ok(None);
    }
    let mut handoff = discover()?;
    handoff.firmware = firmware;
    #[cfg(feature = "diagnostics")]
    let usb_check = if usb_media_check {
        Some((UsbCheck::Media, minimum::Prepared::capture()?))
    } else if usb_reset_check {
        Some((UsbCheck::Reset, minimum::Prepared::capture()?))
    } else {
        None
    };
    Ok(Some(Prepared {
        handoff,
        captures,
        open,
        check,
        #[cfg(feature = "diagnostics")]
        usb_check,
    }))
}

/// The pages host memory buffers take in the DMA pool, for each NVMe
/// controller the firmware drives, as `nvme_setup_host_mem` sizes them:
/// HMPRE capped at `max_host_mem_size_mb`, none when HMMIN exceeds that,
/// and a page for each descriptor list.
fn host_memory_pages() -> usize {
    let Ok(handles) = boot::find_handles::<NvmePassThru>() else {
        return 0;
    };
    let required = |handle| {
        // SAFETY: one Identify Controller while boot services run; the
        // firmware keeps driving the controller.
        let pass_thru = unsafe {
            boot::open_protocol::<NvmePassThru>(
                OpenProtocolParams {
                    handle,
                    agent: boot::image_handle(),
                    controller: None,
                },
                OpenProtocolAttributes::GetProtocol,
            )
        }
        .ok()?;
        let request = NvmeRequestBuilder::new(pass_thru.io_align(), 0x06, NvmeQueueType::ADMIN)
            .with_timeout(Duration::from_secs(1))
            .with_cdw10(1)
            .with_transfer_buffer(4096)
            .ok()?
            .build();
        let response = pass_thru.controller().execute_command(request).ok()?;
        let identify = response.transfer_buffer()?;
        let pages = |at: usize| {
            let bytes = identify.get(at..at + 4)?;
            Some(u64::from(u32::from_le_bytes(bytes.try_into().ok()?)) * 4096)
        };
        let (preferred, minimum) = (pages(272)?, pages(276)?);
        if preferred == 0 || minimum > HOST_MEMORY_MAXIMUM {
            return None;
        }
        Some(preferred.min(HOST_MEMORY_MAXIMUM) as usize / crate::paging::PAGE + 1)
    };
    handles.iter().copied().filter_map(required).sum()
}

/// The host bridges: each root bridge's segment and bus range, as the
/// firmware decodes them, its `_CRS` bus-number descriptor (Linux reads the
/// same from the PNP0A08 device's AML; the firmware's root bridge protocol
/// reports it before boot services end, when the engine cannot run AML).
fn root_bridges() -> Vec<(u16, core::ops::RangeInclusive<u8>)> {
    let Ok(handles) = boot::find_handles::<PciRootBridgeIo>() else {
        return Vec::new();
    };
    let bridge = |handle| {
        // SAFETY: a read of its configuration while boot services run; the
        // firmware keeps the root bridge.
        let bridge = unsafe {
            boot::open_protocol::<PciRootBridgeIo>(
                OpenProtocolParams {
                    handle,
                    agent: boot::image_handle(),
                    controller: None,
                },
                OpenProtocolAttributes::GetProtocol,
            )
        }
        .ok()?;
        let segment = u16::try_from(bridge.segment_nr()).ok()?;
        let buses = bridge
            .configuration()
            .ok()?
            .into_iter()
            .find(|descriptor| descriptor.resource_range_type == ResourceRangeType::Bus)?;
        let first = u8::try_from(buses.address_min).ok()?;
        let last = u8::try_from(buses.address_max).unwrap_or(u8::MAX);
        (first <= last).then_some((segment, first..=last))
    };
    handles.iter().copied().filter_map(bridge).collect()
}

/// The most `HOST PCI` lines one census writes; a PC has under 100
/// functions.
const CENSUS_LINES: usize = 256;

/// One `HOST PCI` line for each function the firmware lists, with what the
/// drivers will do with it, written while the firmware still stores the
/// boot log: a later report that names only a PCI address then maps to a
/// device. Written once per boot.
fn report_census(functions: &[crate::pci_io::FunctionObservation]) {
    static REPORTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if REPORTED.swap(true, core::sync::atomic::Ordering::Relaxed) {
        return;
    }
    let displays: Vec<_> = functions
        .iter()
        .map(crate::pci_io::FunctionObservation::identity)
        .filter(|identity| identity.class[0] == 0x03 && identity.address.function == 0)
        .map(|identity| identity.address)
        .collect();
    for function in functions.iter().take(CENSUS_LINES) {
        let identity = function.identity();
        let header = &function.header;
        let word = |offset: usize| u16::from_le_bytes([header[offset], header[offset + 1]]);
        let [base, sub, interface] = identity.class;
        output::arguments(format_args!(
            "HOST PCI {} {:04x}:{:04x} class={base:02x}{sub:02x}{interface:02x} rev={:02x}",
            identity.address, identity.vendor, identity.device, header[8]
        ));
        // Only a type 0 header has its subsystem at 0x2c.
        if header[0x0e] & 0x7f == 0 {
            output::arguments(format_args!(
                " subsystem={:04x}:{:04x}",
                word(0x2c),
                word(0x2e)
            ));
        }
        output::arguments(format_args!(
            ": {}\n",
            stela_drivers::probe::plan(&identity, &displays)
        ));
    }
    if functions.len() > CENSUS_LINES {
        output::arguments(format_args!(
            "HOST PCI {} more functions not listed\n",
            functions.len() - CENSUS_LINES
        ));
    }
    let nvidia: Vec<u16> = functions
        .iter()
        .map(crate::pci_io::FunctionObservation::identity)
        .filter(|identity| identity.vendor == NVIDIA && identity.class[0] == 0x03)
        .map(|identity| identity.device)
        .collect();
    let notice = gpu_notice(&nvidia);
    if nvidia.is_empty() {
        output::arguments(format_args!(
            "UNSUPPORTED host GPU: no NVIDIA display controller among {} PCI functions\n",
            functions.len()
        ));
    } else if notice != 0 {
        output::arguments(format_args!(
            "UNSUPPORTED host GPU: no NVIDIA display controller here is of a chip Stela drives: {nvidia:04x?}\n"
        ));
    }
    GPU_NOTICE.store(notice, core::sync::atomic::Ordering::Relaxed);
}

const NVIDIA: u16 = 0x10de;
/// Which notice the census chose: 0 none, 1 no NVIDIA card, 2 none Stela drives.
static GPU_NOTICE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);

/// Said once before an install or a start on a PC with no NVIDIA graphics
/// card: the install itself draws through no GPU and would finish, and the
/// installed macOS then could not start.
pub const NO_NVIDIA_GPU_NOTICE: &str = "No NVIDIA graphics card found: macOS can be installed, \
    but it starts only with a GeForce GTX 16 or RTX 20, 30, 40 or 50 series card.";

/// Said in the same place on a PC whose NVIDIA cards are all of chips Stela
/// has no driver for, older or newer than the series it names.
pub const NO_DRIVEN_NVIDIA_GPU_NOTICE: &str = "The NVIDIA graphics card here is not one Stela \
    drives: macOS can be installed, but it starts only with a GeForce GTX 16 or RTX 20, 30, 40 \
    or 50 series card.";

/// The notice ([`GPU_NOTICE`]) for a PC whose NVIDIA display controllers have
/// these PCI device IDs: none when one of them is of a chip Stela's catalog
/// names.
fn gpu_notice(nvidia: &[u16]) -> u8 {
    use stela_loader::host_firmware::recipes::nvidia_chip;
    if nvidia.is_empty() {
        1
    } else if nvidia.iter().all(|device| nvidia_chip(*device).is_none()) {
        2
    } else {
        0
    }
}

/// What to say before an install or a start when the census ([`discover`])
/// found no NVIDIA display controller Stela drives.
pub fn gpu_notice_found() -> Option<&'static str> {
    match GPU_NOTICE.load(core::sync::atomic::Ordering::Relaxed) {
        0 => None,
        1 => Some(NO_NVIDIA_GPU_NOTICE),
        _ => Some(NO_DRIVEN_NVIDIA_GPU_NOTICE),
    }
}

/// What the storage drivers need after ExitBootServices: the ECAM windows,
/// DMA pages below 4 GiB, the TSC rate and every range to map uncached.
/// Runs before ExitBootServices.
pub fn discover() -> Result<Handoff> {
    let rsdp = rsdp().ok_or(refuse!("firmware has no ACPI tables"))?;
    // SAFETY: the firmware's RSDP, its tables mapped while boot services last.
    let mcfg =
        unsafe { acpi::find(rsdp, b"MCFG") }.map_err(|_| refuse!("no valid ACPI MCFG table"))?;
    let mcfg = acpi::mcfg(mcfg).map_err(|_| refuse!("invalid ACPI MCFG table"))?;
    let bridges = root_bridges();
    let ecam = if bridges.is_empty() {
        output::arguments(format_args!(
            "UNSUPPORTED PCI root bridges unknown: each MCFG window is scanned from its first bus alone\n"
        ));
        mcfg
    } else {
        let rooted = pci::rooted(&mcfg, &bridges);
        for (segment, buses) in &bridges {
            if !rooted
                .iter()
                .any(|window| window.segment == *segment && window.start_bus == *buses.start())
            {
                output::arguments(format_args!(
                    "UNSUPPORTED PCI root bridge {segment:04x}:{:02x}-{:02x} outside every MCFG window: not scanned\n",
                    buses.start(),
                    buses.end()
                ));
            }
        }
        rooted
    };
    let allocate = |pages| {
        boot::allocate_pages(
            AllocateType::MaxAddress(0xffff_ffff),
            MemoryType::LOADER_DATA,
            pages,
        )
        .map(|pages| pages.as_ptr() as u64)
    };
    // Without room for the host memory buffers the controllers run without
    // them, as Linux's do when that allocation fails.
    let extra = host_memory_pages();
    let (dma, pages) = match allocate(DMA_PAGES + extra) {
        Ok(dma) => (dma, DMA_PAGES + extra),
        Err(_) => (
            allocate(DMA_PAGES).map_err(|_| refuse!("firmware has no DMA pages below 4 GiB"))?,
            DMA_PAGES,
        ),
    };
    // SAFETY: RDTSC has no memory effects.
    let start = unsafe { core::arch::x86_64::_rdtsc() };
    boot::stall(Duration::from_millis(10));
    // SAFETY: RDTSC has no memory effects.
    let ticks = unsafe { core::arch::x86_64::_rdtsc() } - start;
    let mut mapped: Vec<_> = ecam.iter().map(|window| window.range()).collect();
    match crate::pci_io::capture() {
        Ok(census) => {
            for failure in census.failures {
                output::arguments(format_args!(
                    "PCI firmware observation incomplete: {failure}; native owned discovery required\n"
                ));
            }
            report_census(&census.functions);
            for function in census.functions {
                let identity = function.identity();
                if !stela_drivers::probe::claims_identity(&identity) {
                    continue;
                }
                for (bar, regions) in function.bars.into_iter().enumerate() {
                    match regions {
                        Ok(regions) => {
                            for region in regions {
                                if region.resource_type == 0 {
                                    if region.host != region.pci {
                                        output::arguments(format_args!(
                                            "UNSUPPORTED PCI BAR translation {} bar={bar} host={:x?} pci={:x?}; native translated owner required\n",
                                            identity.address, region.host, region.pci
                                        ));
                                    } else {
                                        mapped.push(region.host);
                                    }
                                }
                            }
                        }
                        Err(crate::pci_io::Error::Firmware(uefi::Status::UNSUPPORTED)) => {}
                        Err(error) => output::arguments(format_args!(
                            "PCI firmware BAR observation refused {} bar={bar}: {error}; native owned discovery required\n",
                            identity.address
                        )),
                    }
                }
            }
        }
        Err(crate::pci_io::CaptureFailure::Function(crate::pci_io::Error::Release(status))) => {
            output::arguments(format_args!(
                "PCI firmware resource buffer release failed: {status:?}\n"
            ));
            return Err(refuse!(
                "PCI firmware resource buffer remains retained; handoff refused"
            ));
        }
        Err(error) => output::arguments(format_args!(
            "PCI firmware observation unavailable: {error}; native owned discovery required\n"
        )),
    }

    Ok(Handoff {
        firmware: None,
        completion: None,
        ecam,
        mapped,
        dma: dma..dma + (pages * crate::paging::PAGE) as u64,
        clock: Clock {
            ticks_per_microsecond: (ticks / 10_000).max(1),
        },
    })
}

impl Prepared {
    /// Map ECAM and the controllers' registers uncached.
    pub fn map(&mut self, tables: &mut Tables) -> Result<()> {
        #[cfg(feature = "diagnostics")]
        if let Some((_, clock)) = &mut self.usb_check {
            clock.map(tables)?;
        }
        self.handoff
            .mapped
            .iter()
            .try_for_each(|range| tables.device(range.start as usize, range.end as usize))
    }

    /// After ExitBootServices: bring up the controllers, report every disk,
    /// then open and check the named partitions. Any failure is fatal.
    pub fn run(self, #[cfg(feature = "diagnostics")] root: &crate::paging::ActivatedTables) {
        // SAFETY: one CPU after ExitBootServices; `dma` was reserved for this.
        let mut dma = unsafe { RegionAllocator::new(self.handoff.dma.clone()) };
        #[cfg(feature = "diagnostics")]
        let mut devices = if let Some((check, clock)) = self.usb_check {
            // SAFETY: these explicit USB fixtures own their clocks and
            // controllers, under the standalone root before any device I/O.
            let minimum =
                unsafe { clock.attach(root) }.unwrap_or_else(|error| output::fatal(error));
            let result = match check {
                // SAFETY: exact post-EBS fixture ownership and retained timer.
                UsbCheck::Media => unsafe {
                    media_check::probe_and_rescan(&self.handoff, &mut dma, &self.check[0], minimum)
                },
                // SAFETY: exact post-EBS fixture ownership and retained timer.
                UsbCheck::Reset => unsafe {
                    reset_check::probe_and_reset(&self.handoff, &mut dma, &self.check[0], minimum)
                },
            };
            result.unwrap_or_else(|error| output::fatal(error))
        } else {
            // SAFETY: this handoff describes the running machine and its tables.
            unsafe { probe(&self.handoff, self.captures, &mut dma) }
        };
        #[cfg(not(feature = "diagnostics"))]
        // SAFETY: this handoff describes the running machine and its tables.
        let mut devices = unsafe { probe(&self.handoff, self.captures, &mut dma) };
        for window in &devices.unmapped {
            output::arguments(format_args!("STORAGE UNMAPPED ecam={window:x?}\n"));
        }
        for (address, error) in &devices.failures {
            report_failure(&devices, *address, error);
        }
        for (location, error) in &devices.platform_failures {
            output::arguments(format_args!("PLATFORM FAILED {location}: {error}\n"));
        }
        for (address, warning) in &devices.warnings {
            output::arguments(format_args!("HOST WARNING pci={address}: {warning}\n"));
        }
        for (location, error) in devices.bluetooth_failures() {
            output::arguments(format_args!("BLUETOOTH FAILED {location}: {error}\n"));
        }
        output::arguments(format_args!(
            "BLUETOOTH PLATFORM {:?}\n",
            devices.bluetooth_platform.observation
        ));
        for (location, error) in &devices.usb_vendor_failures {
            output::arguments(format_args!("USB VENDOR FAILED {location}: {error}\n"));
        }
        for (location, error) in &devices.usb_vendor_warnings {
            output::arguments(format_args!(
                "USB VENDOR WARNING {location}: {error}; started without its firmware patch\n"
            ));
        }
        for device in &devices.disks {
            let id = device.identity();
            output::arguments(format_args!(
                "STORAGE DEVICE {:?} at {} serial={:?} model={:?} blocks={} block_size={}\n",
                id.transport, id.location, id.serial, id.model, id.blocks, id.block_size
            ));
        }
        let targets = self
            .open
            .iter()
            .map(|target| (target, false))
            .chain(self.check.iter().map(|target| (target, true)));
        for (target, write) in targets {
            let mut partition = match storage::open(&mut devices.disks, target) {
                Ok(partition) => partition,
                Err(error) => {
                    output::arguments(format_args!(
                        "STORAGE REFUSED {}:{} {error}\n",
                        target.serial, target.partition
                    ));
                    output::fatal(format_args!("storage target refused: {error}"))
                }
            };
            let disk = partition.disk().clone();
            let blocks = partition.identity().blocks;
            let result = if write {
                storage::check(&mut partition, CHECK_BYTES)
            } else {
                let mut first = alloc::vec![0; disk.block_size as usize];
                partition
                    .read(0, &mut first)
                    .map(|()| stela_formats::sha256(&first))
                    .map_err(storage::StorageError::Device)
            };
            match result {
                Ok(hash) => output::arguments(format_args!(
                    "STORAGE {} {:?} serial={} partition={} blocks={} bytes={} sha256={hash}\n",
                    if write { "CHECKED" } else { "OPENED" },
                    disk.transport,
                    disk.serial,
                    target.partition,
                    blocks,
                    if write {
                        CHECK_BYTES
                    } else {
                        disk.block_size as usize
                    },
                )),
                Err(error) => {
                    output::arguments(format_args!(
                        "STORAGE ERROR {}:{} {error}\n",
                        target.serial, target.partition
                    ));
                    output::fatal(format_args!("storage check failed: {error}"))
                }
            }
        }
    }
}
