//! What Stela takes from the firmware before leaving boot services: its
//! options and guest image from the boot volume, RAM for the heap, stack and
//! frame arenas, the framebuffer, and the memory map its page tables cover.
use crate::Result;
use crate::console::Framebuffer;
use crate::paging::{LARGE_PAGE, NX, PAGE, PRESENT, Tables, WRITABLE};
use alloc::vec::Vec;
use core::{
    ffi::c_void,
    ptr::NonNull,
    sync::atomic::{AtomicPtr, Ordering},
    time::Duration,
};
use stela_boot_core::firmware_connect::{self, Kind};
use stela_contracts::refuse;
use uefi::{
    CStr16, Event, Handle,
    boot::{self, AllocateType, EventType, MemoryType, TimerTrigger, Tpl},
    cstr16,
    fs::FileSystem,
    mem::memory_map::MemoryMap,
    proto::{
        device_path::{DevicePath, DeviceSubType, DeviceType},
        loaded_image::LoadedImage,
        media::{file::Directory, fs::SimpleFileSystem},
        pci::root_bridge::PciRootBridgeIo,
    },
};

/// Mount options.
const OPTIONS: &CStr16 = cstr16!("\\EFI\\stela\\stela.cfg");
/// A system-mode guest image without a bundle, loaded unchanged.
const SOURCE: &CStr16 = cstr16!("\\EFI\\stela\\source.bin");
pub(crate) const STACK_PAGES: usize = 256;

pub(crate) mod entries;
mod tsc;
pub(crate) use pe::loaded_extent;
use stela_boot_core::pe;
pub use tsc::{TscSource, adopt as adopt_tsc, tsc_hz, tsc_source, tsc_tolerance_ppm};

/// What the host storage check runs with after boot services end.
pub struct Handoff {
    pub stack_top: usize,
    /// The builder after activation; configuration factories must
    /// additionally check resources/current context before hardware access.
    pub(crate) active_tables: Option<crate::paging::ActivatedTables>,
    /// The firmware's framebuffer, mapped RW+NX, when it has a usable one.
    pub screen: Option<Framebuffer>,
    /// The disks `stela.cfg` names, until the check takes them.
    pub storage: Option<crate::storage::Prepared>,
}

pub(crate) fn pages(count: usize) -> Result<usize> {
    any_pages(count)
        .map(|page| page.as_ptr() as usize)
        .map_err(|_| refuse!("firmware has no memory for Stela"))
}

/// `count` pages of `LOADER_DATA` at any address. Where the firmware refuses
/// a request that a free run of its own memory map holds, the pages are
/// taken by address at the top of such a run
/// ([`stela_boot_core::firmware_pages`]); a refusal that stands is logged
/// with the largest free runs.
pub(crate) fn any_pages(count: usize) -> uefi::Result<NonNull<u8>> {
    use stela_boot_core::firmware_pages;
    let refusal = match boot::allocate_pages(AllocateType::AnyPages, MemoryType::LOADER_DATA, count)
    {
        Ok(page) => return Ok(page),
        Err(refusal) => refusal,
    };
    let Ok(map) = boot::memory_map(MemoryType::LOADER_DATA) else {
        return Err(refusal);
    };
    let free: Vec<(u64, u64)> = map
        .entries()
        .filter(|entry| entry.ty == MemoryType::CONVENTIONAL)
        .map(|entry| (entry.phys_start, entry.page_count))
        .collect();
    drop(map);
    for base in firmware_pages::placements(&free, count as u64) {
        if let Ok(page) =
            boot::allocate_pages(AllocateType::Address(base), MemoryType::LOADER_DATA, count)
        {
            crate::output::arguments(format_args!(
                "FIRMWARE MEMORY refused {count} pages at any address ({:?}); taken at {base:#x}\n",
                refusal.status()
            ));
            return Ok(page);
        }
    }
    crate::output::arguments(format_args!(
        "FIRMWARE MEMORY refused {count} pages ({:?}); largest free runs in pages: {:?}\n",
        refusal.status(),
        firmware_pages::largest(&free, 4)
    ));
    Err(refusal)
}

fn configured(volume: &mut FileSystem) -> bool {
    matches!(volume.try_exists(OPTIONS), Ok(true))
}

fn boot_volume() -> Option<(FileSystem, Directory)> {
    let mut protocol = boot::get_image_file_system(boot::image_handle()).ok()?;
    let root = protocol.open_volume().ok()?;
    Some((FileSystem::new(protocol), root))
}

/// Connect the firmware's drivers to what the boot needs: storage, USB, the
/// display and the console's devices, with their children, but no network
/// controller, whose firmware drivers nothing before boot services end
/// uses (`stela_boot_core::firmware_connect`). A firmware that boots fast
/// connects only the path to the boot loader, and the bundle, the install
/// medium and the host disks lie on other partitions and devices. Each
/// connection that takes a second or more is named in the boot log and on
/// the screen.
pub(crate) fn connect_needed() -> firmware_connect::Summary {
    // Root bridges appear only with the firmware's own drivers, all before
    // the mount starts, so the first list of them holds.
    firmware_connect::connect(&mut Connector {
        root_bridges: boot::find_handles::<PciRootBridgeIo>().unwrap_or_default(),
    })
}

/// The firmware calls of [`connect_needed`].
struct Connector {
    root_bridges: Vec<Handle>,
}

impl firmware_connect::Firmware for Connector {
    type Handle = Handle;

    fn handles(&mut self) -> Vec<Handle> {
        boot::locate_handle_buffer(boot::SearchType::AllHandles)
            .map(|handles| handles.to_vec())
            .unwrap_or_default()
    }

    fn kind(&mut self, handle: Handle) -> Kind {
        if self.root_bridges.contains(&handle) {
            return Kind::RootBridge;
        }
        if let Some(class) = crate::pci_io::function_class(handle) {
            return Kind::Pci(class);
        }
        if has_mac_node(handle) {
            return Kind::Network;
        }
        Kind::Other
    }

    fn connect(&mut self, handle: Handle, recursive: bool) {
        // On screen while it runs, so a stall shows which function it is.
        if let Some(address) = crate::pci_io::function_address(handle) {
            crate::status::progress(&alloc::format!("{address}"));
        }
        let _ = boot::connect_controller(handle, &[], None, recursive);
        // The next handles' time is not this function's.
        crate::status::progress("");
    }

    fn millis(&mut self) -> u64 {
        crate::output::millis_since(0)
    }

    fn slow(&mut self, handle: Handle, kind: Kind, millis: u64) {
        let device = describe(handle, kind);
        let seconds = millis.div_ceil(1000);
        crate::output::arguments(format_args!(
            "FIRMWARE slow driver start on {device}: {millis} ms\n"
        ));
        crate::status::info(&alloc::format!(
            "The firmware's drivers took {seconds} s to start on {device}"
        ));
    }
}

fn device_path(handle: Handle) -> uefi::Result<boot::ScopedProtocol<DevicePath>> {
    // SAFETY: a scoped read-only protocol observation during boot services.
    unsafe {
        boot::open_protocol::<DevicePath>(
            boot::OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            boot::OpenProtocolAttributes::GetProtocol,
        )
    }
}

/// Whether `handle`'s device path has a MAC address node: a network
/// interface a driver has already started.
fn has_mac_node(handle: Handle) -> bool {
    device_path(handle).is_ok_and(|path| {
        path.node_iter().any(|node| {
            node.device_type() == DeviceType::MESSAGING
                && node.sub_type() == DeviceSubType::MESSAGING_MAC_ADDRESS
        })
    })
}

/// A connected handle for a report: a PCI function's address and class;
/// for a device under a PCI controller, the controller's address and the
/// bus nodes below it without identifiers they may carry; else the
/// device path.
fn describe(handle: Handle, kind: Kind) -> alloc::string::String {
    use alloc::string::ToString;
    use uefi::proto::device_path::text::{AllowShortcuts, DisplayOnly};
    match kind {
        Kind::Pci(class) => {
            let address = crate::pci_io::function_address(handle)
                .map(|address| alloc::format!("{address}"))
                .unwrap_or_else(|| "a PCI function".into());
            match class {
                Some(class) => alloc::format!("{address} class {class:06x}"),
                None => address,
            }
        }
        Kind::RootBridge => "a PCI root bridge".into(),
        Kind::Network | Kind::Other => {
            if let Some(location) = crate::pci_io::disk_location(handle) {
                return location.trim_end().into();
            }
            let text = device_path(handle).ok().and_then(|path| {
                path.to_string16(DisplayOnly(true), AllowShortcuts(false))
                    .ok()
                    .map(|text| text.to_string())
            });
            match text {
                Some(text) => text.chars().take(120).collect(),
                None => "a handle without a device path".into(),
            }
        }
    }
}

/// The first file system that has `stela.cfg`.
fn configured_volume() -> Option<(FileSystem, Directory)> {
    boot::find_handles::<SimpleFileSystem>()
        .ok()?
        .into_iter()
        .find_map(|handle| {
            let mut protocol = boot::open_protocol_exclusive::<SimpleFileSystem>(handle).ok()?;
            let root = protocol.open_volume().ok()?;
            let mut volume = FileSystem::new(protocol);
            configured(&mut volume).then_some((volume, root))
        })
}

/// The volume Stela was loaded from. When the firmware names no such volume,
/// as for an image a boot manager starts from memory, or that volume has no
/// `stela.cfg`, the first volume with one is used instead.
pub(crate) fn volume() -> Result<(FileSystem, Directory)> {
    let own = boot_volume().map(|(mut volume, root)| {
        let configured = configured(&mut volume);
        ((volume, root), configured)
    });
    Ok(match own {
        Some((volume, true)) => volume,
        Some((volume, false)) => {
            // Close the boot volume so the scan can open every volume.
            drop(volume);
            match configured_volume() {
                Some(volume) => volume,
                None => boot_volume().ok_or(refuse!("cannot reopen the boot volume"))?,
            }
        }
        None => configured_volume().ok_or(refuse!(
            "no boot volume, and no volume has \\EFI\\stela\\stela.cfg"
        ))?,
    })
}

/// `stela.cfg`, or nothing when the volume has none.
pub(crate) fn options(volume: &mut FileSystem) -> Result<Vec<u8>> {
    if configured(volume) {
        volume
            .read(OPTIONS)
            .map_err(|_| refuse!("cannot read stela.cfg"))
    } else {
        Ok(Vec::new())
    }
}

/// `\EFI\stela\source.bin`, the unchanged guest image.
pub(crate) fn source(volume: &mut FileSystem) -> Result<Vec<u8>> {
    volume
        .read(SOURCE)
        .map_err(|_| refuse!("cannot read \\EFI\\stela\\source.bin"))
}

/// Page rights of Stela's own PE image: code RX, constants R, data RW+NX.
pub(crate) struct Image {
    pub start: usize,
    pub end: usize,
    pub sections: Vec<(usize, usize, u64)>,
}

impl Image {
    pub(crate) fn loaded() -> Result<Self> {
        let image = boot::open_protocol_exclusive::<LoadedImage>(boot::image_handle())
            .map_err(|_| refuse!("no loaded image"))?;
        let (base, bytes) = image.info();
        let extent = loaded_extent(base as usize, bytes)?;
        let sections = pe::sections(extent.bytes, |offset, out| {
            // SAFETY: pe::sections checks each short read against the exact
            // firmware-owned LoadedImage extent before invoking this callback.
            // No shared slice covers live writable image statics.
            unsafe {
                core::ptr::copy_nonoverlapping(
                    base.cast::<u8>().add(offset),
                    out.as_mut_ptr(),
                    out.len(),
                );
            }
        })?
        .into_iter()
        .map(|section| {
            let rights = if section.executable {
                PRESENT
            } else if section.writable {
                PRESENT | WRITABLE | NX
            } else {
                PRESENT | NX
            };
            // Both RVAs lie within the checked loaded extent.
            (
                extent.start + section.first,
                extent.start + section.end,
                rights,
            )
        })
        .collect();
        Ok(Self {
            start: extent.start,
            end: extent.end,
            sections,
        })
    }

    pub(crate) fn rights(&self, page: usize) -> u64 {
        self.sections
            .iter()
            .find(|(start, end, _)| (*start..*end).contains(&page))
            .map_or(PRESENT | NX, |section| section.2)
    }
}

/// Write-back RAM the firmware reports, sorted and merged.
pub(crate) fn ram() -> Result<Vec<(usize, usize)>> {
    let map = boot::memory_map(MemoryType::LOADER_DATA).map_err(|_| refuse!("no memory map"))?;
    let mut ranges: Vec<(usize, usize)> = map
        .entries()
        .filter(|entry| {
            matches!(
                entry.ty,
                MemoryType::LOADER_CODE
                    | MemoryType::LOADER_DATA
                    | MemoryType::BOOT_SERVICES_CODE
                    | MemoryType::BOOT_SERVICES_DATA
                    | MemoryType::RUNTIME_SERVICES_CODE
                    | MemoryType::RUNTIME_SERVICES_DATA
                    | MemoryType::CONVENTIONAL
                    | MemoryType::ACPI_RECLAIM
                    | MemoryType::ACPI_NON_VOLATILE
            )
        })
        .map(|entry| {
            let start = entry.phys_start as usize;
            (start, start + entry.page_count as usize * PAGE)
        })
        .collect();
    ranges.sort_unstable();
    let mut merged: Vec<(usize, usize)> = Vec::new();
    for (start, end) in ranges {
        match merged.last_mut() {
            Some(last) if start <= last.1 => last.1 = last.1.max(end),
            _ => merged.push((start, end)),
        }
    }
    Ok(merged)
}

/// Identity-map RAM and the framebuffer RW+NX with 2 MiB pages where they
/// fill a whole chunk and 4 KiB pages elsewhere. Page zero and the stack
/// guard stay unmapped; the image gets its section rights.
pub(crate) fn tables(
    image: &Image,
    holes: [usize; 2],
    screen: Option<&Framebuffer>,
) -> Result<Tables> {
    let mut ram = ram()?;
    if let Some(screen) = screen {
        let start = screen.base & !(PAGE - 1);
        ram.push((start, (screen.base + screen.bytes).next_multiple_of(PAGE)));
        ram.sort_unstable();
    }
    let covered = |start: usize, end: usize| ram.iter().any(|r| r.0 <= start && end <= r.1);
    let overlaps = |start: usize, end: usize| ram.iter().any(|r| r.0 < end && start < r.1);
    let mut tables = Tables::new()?;
    let top = ram.last().map_or(0, |r| r.1).max(image.end);
    for chunk in (0..top).step_by(LARGE_PAGE) {
        let end = chunk + LARGE_PAGE;
        let special = holes.iter().any(|&hole| (chunk..end).contains(&hole))
            || (image.start < end && chunk < image.end);
        if covered(chunk, end) && !special {
            tables.large(chunk)?;
        } else if overlaps(chunk, end) || special {
            tables.small(chunk)?;
            for page in (chunk..end).step_by(PAGE) {
                let rights = if holes.contains(&page) {
                    continue;
                } else if (image.start..image.end).contains(&page) {
                    image.rights(page)
                } else if covered(page, page + PAGE) {
                    PRESENT | WRITABLE | NX
                } else {
                    continue;
                };
                tables.page(page, rights)?;
            }
        }
    }
    Ok(tables)
}

/// Run install mode, or collect what the host storage check needs and leave
/// boot services. The returned tables are not active yet.
pub fn take_over(
    options: Vec<u8>,
    host_firmware: crate::host_firmware::Resources,
) -> Result<(Handoff, Tables)> {
    // Install mode verifies a payload and stops here; otherwise nothing
    // changes.
    let options = crate::payload::run(options)?;
    let text = core::str::from_utf8(&options).map_err(|_| refuse!("stela.cfg is not UTF-8"))?;
    if !crate::storage::split(text)?.2.is_empty() {
        return Err(refuse!("stela.cfg names options that need --system"));
    }
    let captures = crate::storage::captures(host_firmware.wifi_observations());
    // SAFETY: standalone mount keeps identity mappings and this allocator.
    let host_firmware = unsafe { crate::host_firmware::identity_catalog(&host_firmware) }?;
    let mut storage =
        crate::storage::prepare(&options, host_firmware, captures)?.ok_or(refuse!(
            "stela.cfg starts with neither --system nor --host-check and names no install or \
         storage check"
        ))?;
    let guard = pages(STACK_PAGES + 1)?;
    let image = Image::loaded()?;
    let screen = Framebuffer::from_firmware();
    let mut tables = tables(&image, [0, guard], screen.as_ref())?;
    storage.map(&mut tables)?;
    let final_map = crate::boot_exit::Buffer::allocate();
    crate::output::leave_boot_services();
    // SAFETY: no protocol or pool allocation outlives this call; the heap,
    // stack and tables are LOADER_DATA, which Stela keeps.
    let _ = unsafe { crate::boot_exit::exit(final_map) };
    Ok((
        Handoff {
            active_tables: None,
            stack_top: guard + (STACK_PAGES + 1) * PAGE,
            screen,
            storage: Some(storage),
        },
        tables,
    ))
}

/// The firmware timer that moves the running step's marker while boot
/// services run, so a long firmware call (a file grown, a disk read) still
/// shows motion: an `Event` pointer, null when none runs.
static TICKER: AtomicPtr<c_void> = AtomicPtr::new(core::ptr::null_mut());
/// Eight frames a second.
const TICK_PERIOD: Duration = Duration::from_millis(125);

/// Start the marker's timer. Its callback runs at `TPL_NOTIFY`, above the
/// level the firmware's file and disk drivers hold their locks at, so it
/// also runs while one of their calls takes long.
pub(crate) fn start_ticker() {
    unsafe extern "efiapi" fn tick(_: Event, _: Option<NonNull<c_void>>) {
        crate::status::tick();
    }
    // SAFETY: the callback allocates nothing and skips a frame while the
    // state or the console is in use; [`stop_ticker`] closes the event
    // before boot services end.
    let event = unsafe {
        boot::create_event(
            EventType::TIMER | EventType::NOTIFY_SIGNAL,
            Tpl::NOTIFY,
            Some(tick),
            None,
        )
    };
    let Ok(event) = event else {
        return;
    };
    if boot::set_timer(&event, TimerTrigger::Periodic(TICK_PERIOD)).is_ok() {
        TICKER.store(event.as_ptr(), Ordering::Relaxed);
    } else {
        let _ = boot::close_event(event);
    }
}

/// Close the marker's timer, before boot services end.
pub(crate) fn stop_ticker() {
    let pointer = TICKER.swap(core::ptr::null_mut(), Ordering::Relaxed);
    // SAFETY: `start_ticker` stored this event, which nothing else closes.
    if let Some(event) = unsafe { Event::from_ptr(pointer) } {
        let _ = boot::close_event(event);
    }
}
