//! Install macOS's disk picker, before boot services end: full-screen pages
//! on the firmware's framebuffer where the user chooses the PC disk macOS
//! is installed on and the space it takes, as a Linux installer's
//! partitioner does. Each disk shows its partitions as a bar; macOS can
//! take free space or a Stela partition already there, a partition can be
//! deleted, the whole disk erased, and the new partition's size set.
//!
//! The picker writes nothing to any disk: it reads the disks through the
//! firmware and, when the last page confirms a choice, makes a plan of the
//! change (`pc::disk_plan`). Stela's own drivers make that change after the
//! firmware has exited, before the engine looks the partition up: the
//! disk's GPT gains the partition (the Stela Apple region type,
//! `payload::gpt`), which holds the Mac's internal SSD and, in its tail,
//! the machine state (`pc::machine_state`); its first MiB and that tail are
//! zeroed, so the restore and the NOR start blank. The caller records the
//! choice (`mount_interface::DISK_FILE`) and hands the plan on. A boot
//! that stops before the plan is applied leaves the disk as it was.
//!
//! Disks are listed as the engine finds them after boot (`pc::storage`): by
//! serial number, so one without a serial cannot be chosen, and the USB
//! Stela started from is left out. Filesystems are never resized: another
//! system's space is freed only by deleting its partition.
use crate::Refusal;
use crate::Result;
use crate::{
    choice::{self, Press, Wait},
    console::{Framebuffer, font},
    output,
    payload::{Aligned, device_path, disk_model, disk_serial},
    ui::{self, Canvas},
};
use alloc::string::ToString;
use alloc::{format, string::String, vec, vec::Vec};
use stela_boot_core::chosen_disk;
use stela_boot_core::screen::{disk_size as size, window};
use stela_contracts::refuse;
use stela_crypto::Sha256;
use stela_crypto::digest::Digest;
use uefi::{
    Handle,
    boot::{self, OpenProtocolAttributes, OpenProtocolParams, ScopedProtocol},
    proto::{loaded_image::LoadedImage, media::block::BlockIO},
};
use {
    stela_formats::payload::gpt, stela_formats::payload::gpt::Found,
    stela_formats::payload::gpt::Guid, stela_formats::payload::gpt::Role,
    stela_formats::payload::gpt::Table, stela_runtime::pc::disk_plan,
    stela_runtime::pc::disk_plan::Plan, stela_runtime::pc::machine_state,
};

const MIB: u64 = 1 << 20;
const GB: u64 = 1_000_000_000;
/// The least partition macOS is given: the system's minimum
/// (`gpt::DEFAULT_MIN_APPLE_BYTES`, which setup also checks) and the
/// machine state's tail after it.
const MIN_BYTES: u64 = gpt::DEFAULT_MIN_APPLE_BYTES + machine_state::MINIMUM_BYTES;
/// The messages name the system's minimum, in decimal GB as disks are sold.
const _: () = assert!(gpt::DEFAULT_MIN_APPLE_BYTES == 60 * GB);
/// The new partition's GPT name.
const NAME: &str = "Stela macOS";

/// One disk of this PC.
struct Disk {
    io: ScopedProtocol<BlockIO>,
    media: u32,
    geometry: gpt::Disk,
    serial: Option<String>,
    model: String,
    bus: &'static str,
    /// Its GPT; `None` on a blank disk or one with another scheme.
    table: Option<Table>,
    /// Data without a GPT, which only erasing the disk replaces.
    foreign: bool,
    /// Why it cannot take macOS, when it cannot.
    refusal: Option<&'static str>,
}

impl Disk {
    fn bytes(&self) -> u64 {
        self.geometry.blocks * self.geometry.block_bytes
    }

    fn mib_blocks(&self) -> u64 {
        (MIB / self.geometry.block_bytes).max(1)
    }

    fn partition(&self, index: usize) -> &gpt::Partition {
        &self
            .table
            .as_ref()
            .expect("a partition is in the table")
            .partitions[index]
    }

    fn read(&mut self, lba: u64, blocks: u64) -> Result<Vec<u8>> {
        let mut buffer = Aligned::new((blocks * self.geometry.block_bytes) as usize);
        self.io
            .read_blocks(self.media, lba, buffer.bytes())
            .map_err(|_| refuse!("the disk cannot be read"))?;
        Ok(buffer.bytes().to_vec())
    }

    /// The digest of the disk's first blocks as they are now, which the
    /// plan's disk is told by (`disk_plan::seen`).
    fn seen(&mut self) -> Result<[u8; 32]> {
        Ok(disk_plan::seen(&self.read(0, disk_plan::SEEN_BLOCKS)?))
    }

    /// GUIDs for a new partition or table on this disk.
    fn entropy(&self) -> impl FnMut() -> Guid + use<> {
        let serial = self.serial.clone().unwrap_or_default();
        let mut seed = 0u64;
        move || {
            seed += 1;
            // SAFETY: RDTSC only reads the time-stamp counter.
            let time = unsafe { core::arch::x86_64::_rdtsc() };
            let digest = Sha256::new()
                .chain_update(serial.as_bytes())
                .chain_update(time.to_le_bytes())
                .chain_update(seed.to_le_bytes())
                .finalize();
            Guid::from_entropy(digest[..16].try_into().expect("16 bytes"))
        }
    }
}

/// What a stretch of a disk holds.
#[derive(Clone, Copy, PartialEq, Eq)]
enum Content {
    /// Partition `n` of the disk's table.
    Partition(usize),
    Free,
    /// Data of another partition scheme.
    Foreign,
}

/// A stretch of a disk, in blocks from `first` to before `end`.
#[derive(Clone, Copy)]
struct Segment {
    first: u64,
    end: u64,
    content: Content,
}

/// Where macOS goes.
#[derive(Clone, Copy)]
enum Place {
    /// Free space from `first` to before `end`, aligned to MiB.
    Free { first: u64, end: u64 },
    /// A Stela partition already there.
    Reuse(usize),
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum Page {
    Disks,
    Partitions,
    Size,
    Confirm,
}

/// What a key press leads to.
enum Step {
    Stay,
    Leave,
    /// The last page confirmed the choice.
    Confirm,
}

struct Picker {
    disks: Vec<Disk>,
    page: Page,
    disk: usize,
    /// The highlighted row of the partitions page.
    row: usize,
    /// Partitions of the chosen disk to delete, in the order deleted.
    deleted: Vec<usize>,
    erased: bool,
    place: Option<Place>,
    /// The new partition's size.
    size: u64,
    /// Whether the confirmation's install button has the focus.
    install: bool,
    notice: Option<Refusal>,
}

/// Choose the partition macOS is installed on; the plan that makes it, new
/// if need be, and blank. `None` when the user leaves, or without a
/// framebuffer or keyboard to choose with.
pub fn choose() -> Option<Plan> {
    let Some((screen, font)) = Framebuffer::from_firmware().zip(font::console()) else {
        output::arguments(format_args!(
            "INSTALLER refused: no framebuffer to draw on\n"
        ));
        return None;
    };
    let mut canvas = Canvas::new(screen, font);
    let mut picker = Picker {
        disks: disks(),
        page: Page::Disks,
        disk: 0,
        row: 0,
        deleted: Vec::new(),
        erased: false,
        place: None,
        size: 0,
        install: false,
        notice: None,
    };
    for disk in &picker.disks {
        output::arguments(format_args!(
            "INSTALLER disk {} {} serial={:?} bytes={} table={} refusal={:?}\n",
            disk.bus,
            disk.model,
            disk.serial,
            disk.bytes(),
            disk.table.as_ref().map_or(0, |t| t.partitions.len()),
            disk.refusal
        ));
    }
    loop {
        picker.draw(&mut canvas);
        canvas.show();
        let press = match choice::key(None) {
            Wait::Key(press) => press,
            Wait::NoInput => return None,
            Wait::Nothing | Wait::Timeout => continue,
        };
        picker.notice = None;
        match picker.press(press) {
            Step::Stay => {}
            Step::Leave => return None,
            Step::Confirm => match picker.plan() {
                Ok(plan) => return Some(plan),
                Err(error) => {
                    output::arguments(format_args!("INSTALLER plan refused: {error}\n"));
                    picker.notice = Some(error);
                }
            },
        }
    }
}

impl Picker {
    fn chosen(&self) -> &Disk {
        &self.disks[self.disk]
    }

    /// The chosen disk as it will be.
    fn segments(&self) -> Vec<Segment> {
        segments_of(self.chosen(), &self.deleted, self.erased)
    }

    /// A free segment's MiB-aligned span.
    fn aligned(&self, segment: Segment) -> (u64, u64) {
        let unit = self.chosen().mib_blocks();
        (
            segment.first.div_ceil(unit) * unit,
            segment.end / unit * unit,
        )
    }

    fn span_bytes(&self, (first, end): (u64, u64)) -> u64 {
        end.saturating_sub(first) * self.chosen().geometry.block_bytes
    }

    fn open_disk(&mut self) {
        self.deleted.clear();
        self.erased = false;
        self.place = None;
        self.page = Page::Partitions;
        let segments = self.segments();
        // The largest free space starts highlighted.
        self.row = (0..segments.len())
            .filter(|&n| segments[n].content == Content::Free)
            .max_by_key(|&n| self.span_bytes(self.aligned(segments[n])))
            .unwrap_or(0);
    }

    fn press(&mut self, press: Press) -> Step {
        match self.page {
            Page::Disks => self.press_disks(press),
            Page::Partitions => self.press_partitions(press),
            Page::Size => self.press_size(press),
            Page::Confirm => self.press_confirm(press),
        }
    }

    fn press_disks(&mut self, press: Press) -> Step {
        let count = self.disks.len();
        match press {
            Press::Escape => return Step::Leave,
            Press::Up if count > 0 => self.disk = (self.disk + count - 1) % count,
            Press::Down if count > 0 => self.disk = (self.disk + 1) % count,
            Press::Enter if count > 0 => match self.chosen().refusal {
                Some(refusal) => self.notice = Some(refusal.into()),
                None => self.open_disk(),
            },
            _ => {}
        }
        Step::Stay
    }

    fn press_partitions(&mut self, press: Press) -> Step {
        let segments = self.segments();
        let count = segments.len();
        self.row = self.row.min(count.saturating_sub(1));
        let segment = segments.get(self.row).copied();
        match press {
            Press::Escape => self.page = Page::Disks,
            Press::Up if count > 0 => self.row = (self.row + count - 1) % count,
            Press::Down if count > 0 => self.row = (self.row + 1) % count,
            Press::Character('e' | 'E') => {
                self.erased = true;
                self.row = 0;
            }
            Press::Character('u' | 'U') => {
                if self.erased {
                    self.erased = false;
                } else {
                    self.deleted.pop();
                }
            }
            Press::Delete | Press::Character('d' | 'D') => match segment.map(|s| s.content) {
                Some(Content::Partition(n)) => self.deleted.push(n),
                Some(Content::Foreign) => self.erased = true,
                _ => {}
            },
            Press::Enter => match segment {
                Some(
                    segment @ Segment {
                        content: Content::Free,
                        ..
                    },
                ) => {
                    let (first, end) = self.aligned(segment);
                    let bytes = self.span_bytes((first, end));
                    if bytes < MIN_BYTES {
                        self.notice = Some(refuse!("macOS needs at least 60 GB"));
                    } else {
                        self.place = Some(Place::Free { first, end });
                        self.size = bytes / MIB * MIB;
                        self.page = Page::Size;
                    }
                }
                Some(Segment {
                    content: Content::Partition(n),
                    ..
                }) if self.chosen().partition(n).role() == Some(Role::AppleRegion) => {
                    self.place = Some(Place::Reuse(n));
                    self.install = false;
                    self.page = Page::Confirm;
                }
                Some(Segment {
                    content: Content::Partition(_),
                    ..
                }) => {
                    self.notice =
                        Some(refuse!("Choose free space, or delete this partition first"));
                }
                Some(Segment {
                    content: Content::Foreign,
                    ..
                }) => {
                    self.notice = Some(refuse!("Erase the disk first"));
                }
                None => {}
            },
            _ => {}
        }
        Step::Stay
    }

    /// The new partition's size limits and step.
    fn size_range(&self) -> (u64, u64, u64) {
        let Some(Place::Free { first, end }) = self.place else {
            return (0, 0, 0);
        };
        let most = self.span_bytes((first, end)) / MIB * MIB;
        // A round step: the power of two GB nearest a 32nd of the range.
        let step = ((most - MIN_BYTES) / 32 / GB).max(1).next_power_of_two() * GB;
        (MIN_BYTES, most, step)
    }

    fn press_size(&mut self, press: Press) -> Step {
        let (least, most, step) = self.size_range();
        match press {
            Press::Escape => self.page = Page::Partitions,
            Press::Left | Press::Down => {
                self.size = self.size.saturating_sub(step).max(least) / MIB * MIB
            }
            Press::Right | Press::Up => self.size = (self.size + step).min(most) / MIB * MIB,
            Press::Home => self.size = least,
            Press::End => self.size = most,
            Press::Enter => {
                self.install = false;
                self.page = Page::Confirm;
            }
            _ => {}
        }
        Step::Stay
    }

    fn press_confirm(&mut self, press: Press) -> Step {
        let back = match self.place {
            Some(Place::Free { .. }) => Page::Size,
            _ => Page::Partitions,
        };
        match press {
            Press::Escape => self.page = back,
            Press::Left | Press::Right | Press::Character('\t') => self.install = !self.install,
            Press::Enter if self.install => return Step::Confirm,
            Press::Enter => self.page = back,
            _ => {}
        }
        Step::Stay
    }

    /// The partitions the confirmed change deletes.
    fn deleting(&self) -> Vec<usize> {
        match (&self.chosen().table, self.erased) {
            (Some(table), true) => (0..table.partitions.len()).collect(),
            _ => self.deleted.clone(),
        }
    }

    /// The plan of the confirmed change. Nothing is written here.
    fn plan(&mut self) -> Result<Plan> {
        let place = match self.place.ok_or(refuse!("nothing was chosen"))? {
            Place::Reuse(n) => disk_plan::Place::Reuse(n),
            Place::Free { first, .. } => disk_plan::Place::Free {
                first_lba: first,
                bytes: self.size,
            },
        };
        let deleted = self.deleted.clone();
        let erased = self.erased;
        let disk = &mut self.disks[self.disk];
        let seen = disk.seen()?;
        let plan = disk_plan::plan(
            &disk_plan::Decision {
                serial: disk.serial.as_deref().unwrap_or_default(),
                geometry: disk.geometry,
                seen,
                table: disk.table.as_ref(),
                deleted: &deleted,
                erased,
                place,
                name: NAME,
            },
            &mut disk.entropy(),
        )
        .map_err(|error| refuse!("{error}", %error))?;
        log(&plan, "chosen");
        Ok(plan)
    }
}

/// Log `plan`, which Stela's drivers apply after the firmware has exited.
fn log(plan: &Plan, how: &str) {
    output::arguments(format_args!(
        "INSTALLER plan for {}:{} ({how}) first_lba={} blocks={} table_writes={}: nothing is \
         written now; Stela's drivers write it after the firmware has exited\n",
        plan.serial,
        plan.partition.guid,
        plan.partition.first_lba,
        plan.partition.blocks,
        plan.table.len()
    ));
}

/// Open `handle`'s block I/O, without taking it from its driver.
fn block_io(handle: Handle) -> Option<ScopedProtocol<BlockIO>> {
    // SAFETY: the protocol is only read: media, and blocks of whole disks.
    unsafe {
        boot::open_protocol::<BlockIO>(
            OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
    }
    .ok()
}

/// Where the firmware found the disk Stela started from: its controller's
/// PCI address and the path below it (`pci_io::disk_location`).
pub(crate) fn boot_disk_location() -> Option<String> {
    let image = boot::open_protocol_exclusive::<LoadedImage>(boot::image_handle()).ok()?;
    let device = image.device()?;
    let location = crate::pci_io::disk_location(device)?;
    // The firmware's logical block size: Stela's driver must read the same
    // one, or the stick's GPT is not where it looks.
    Some(match block_io(device) {
        Some(io) => format!("{location} block={}", io.media().block_size()),
        None => location,
    })
}

/// The device path of the whole disk Stela started from.
fn boot_disk() -> Option<String> {
    let image = boot::open_protocol_exclusive::<LoadedImage>(boot::image_handle()).ok()?;
    let path = device_path(image.device()?)?;
    Some(path.split("/HD(").next()?.into())
}

/// The bus a disk's device path names.
fn bus(path: &str) -> &'static str {
    [
        ("/NVMe(", "NVMe"),
        ("/Sata(", "SATA"),
        ("/USB(", "USB"),
        ("/Scsi(", "SCSI"),
        ("/eMMC(", "eMMC"),
        ("/SD(", "SD card"),
    ]
    .into_iter()
    .find(|(node, _)| path.contains(node))
    .map_or("Disk", |(_, bus)| bus)
}

/// Every writable whole disk of this PC but the one Stela started from.
fn disks() -> Vec<Disk> {
    let started = boot_disk();
    let Ok(handles) = boot::find_handles::<BlockIO>() else {
        return Vec::new();
    };
    let mut disks = Vec::new();
    for handle in handles {
        let Some(io) = block_io(handle) else {
            continue;
        };
        let media = io.media();
        if media.is_logical_partition() || !media.is_media_present() || media.is_read_only() {
            continue;
        }
        let path = device_path(handle).unwrap_or_default();
        if started.as_deref() == Some(path.as_str()) {
            continue;
        }
        let geometry = gpt::Disk {
            block_bytes: u64::from(media.block_size()),
            blocks: media.last_block() + 1,
        };
        let serial = disk_serial(handle, &path);
        let mut disk = Disk {
            media: media.media_id(),
            io,
            geometry,
            model: disk_model(handle, &path).unwrap_or_else(|| format!("{} disk", bus(&path))),
            bus: bus(&path),
            serial,
            table: None,
            foreign: false,
            refusal: None,
        };
        let found = gpt::read(geometry, |lba, blocks| {
            disk.read(lba, blocks)
                .map_err(|error| stela_formats::payload::PayloadError::Io(error.to_string()))
        });
        match found {
            Ok(Found::Table { table, .. }) => disk.table = Some(table),
            Ok(Found::Blank | Found::Interrupted) => {}
            Err(_) => disk.foreign = true,
        }
        // Why Stela's own drivers cannot reach the disk, if they cannot.
        disk.refusal = crate::pci_io::disk_controller(handle)
            .and_then(|function| stela_drivers::probe::disk_refusal(&function));
        disks.push(disk);
    }
    // Stela's own drivers must reach each disk, and find it again by a
    // serial number no other listed disk reports.
    let candidates: Vec<_> = disks
        .iter()
        .map(|disk| chosen_disk::Candidate {
            serial: disk.serial.as_deref(),
            bytes: disk.bytes(),
            controller: disk.refusal,
        })
        .collect();
    let refusals: Vec<_> = (0..disks.len())
        .map(|index| chosen_disk::picker_refusal(&candidates, index, MIN_BYTES))
        .collect();
    for (disk, refusal) in disks.iter_mut().zip(refusals) {
        disk.refusal = refusal;
    }
    disks
}

/// A partition type's name and colour.
fn kind(partition: &gpt::Partition) -> (&'static str, u32) {
    if let Some(role) = partition.role() {
        return match role {
            Role::AppleRegion => (NAME, ui::BLUE),
            Role::Esp => ("EFI System", 0x7d818b),
            role => (role.name(), OTHER),
        };
    }
    const KNOWN: [(&str, &str, u32); 7] = [
        (
            "EBD0A0A2-B9E5-4433-87C0-68B6B72699C7",
            "Windows data",
            0x7fb0f2,
        ),
        (
            "E3C9E316-0B5C-4DB8-817D-F92DF00215AE",
            "Microsoft reserved",
            0x5a5e68,
        ),
        (
            "DE94BBA4-06D1-4D40-A16A-BFD50179D6AC",
            "Windows recovery",
            0x9196a1,
        ),
        (
            "0FC63DAF-8483-4772-8E79-3D69D8477DE4",
            "Linux filesystem",
            0xf2a660,
        ),
        (
            "0657FD6D-A4AB-43C4-84E5-0933C84B4F4F",
            "Linux swap",
            0xc4773a,
        ),
        (
            "7C3457EF-0000-11AA-AA11-00306543ECAC",
            "Apple APFS",
            0xb8bbc3,
        ),
        (
            "48465300-0000-11AA-AA11-00306543ECAC",
            "Apple HFS+",
            0xa0a4ad,
        ),
    ];
    KNOWN
        .into_iter()
        .find(|(guid, ..)| Guid::parse(guid) == Some(partition.type_guid))
        .map_or(("Other", OTHER), |(_, name, colour)| (name, colour))
}

/// A partition's colour and the data of another scheme's.
const OTHER: u32 = 0x6b6f78;
const ROW: usize = 36;

/// A segment of `disk`: its name, its type when the name is not that, and
/// colour.
fn segment_label(disk: &Disk, segment: Segment) -> (String, &'static str, u32) {
    match segment.content {
        Content::Partition(n) => {
            let partition = disk.partition(n);
            let (kind, colour) = kind(partition);
            match partition.name.trim() {
                "" => (String::from(kind), "", colour),
                name if name == kind => (String::from(kind), "", colour),
                name => (String::from(name), kind, colour),
            }
        }
        Content::Free => (String::from("Free space"), "", ui::LINE),
        Content::Foreign => (String::from("Unknown layout"), "", OTHER),
    }
}

impl Picker {
    fn draw(&self, canvas: &mut Canvas) {
        let (width, height) = canvas.size();
        canvas.fill(0, 0, width, height, ui::BACKGROUND);
        let top = canvas.header() + 16;
        let column = width.saturating_sub(32).min(720);
        let layout = Layout {
            left: (width - column) / 2,
            column,
            top,
            height,
        };
        match self.page {
            Page::Disks => self.draw_disks(canvas, &layout),
            Page::Partitions => self.draw_partitions(canvas, &layout),
            Page::Size => self.draw_size(canvas, &layout),
            Page::Confirm => self.draw_confirm(canvas, &layout),
        }
        if let Some(notice) = &self.notice {
            canvas.text_centred(width / 2, height - 76, &notice.to_string(), 1, ui::ORANGE);
        }
        let keys: &[(&str, &str)] = match self.page {
            Page::Partitions => &[
                ("Enter", "Install here"),
                ("D", "Delete"),
                ("E", "Erase disk"),
                ("U", "Undo"),
                ("Esc", "Back"),
            ],
            _ => &[],
        };
        let gap = 24;
        let total: usize = keys
            .iter()
            .map(|(key, label)| canvas.text_width(key, 1) + 8 + canvas.text_width(label, 1))
            .sum::<usize>()
            + gap * keys.len().saturating_sub(1);
        let mut x = width.saturating_sub(total) / 2;
        for (key, label) in keys {
            canvas.text(x, height - 40, key, 1, ui::ORANGE);
            x += canvas.text_width(key, 1) + 8;
            canvas.text(x, height - 40, label, 1, ui::DIM);
            x += canvas.text_width(label, 1) + gap;
        }
    }

    /// The page's title, and its size on the right.
    fn title(&self, canvas: &mut Canvas, layout: &Layout, title: &str, right: &str) {
        canvas.text(layout.left, layout.top, title, 1, ui::TEXT);
        canvas.text_right(layout.left + layout.column, layout.top, right, 1, ui::DIM);
    }

    /// A row raised and marked orange.
    fn selection(canvas: &mut Canvas, x: usize, y: usize, width: usize, height: usize) {
        canvas.panel(x, y, width, height, 8, ui::RAISED);
        canvas.panel(x + 6, y + 9, 3, height.saturating_sub(18), 1, ui::ORANGE);
    }

    fn draw_disks(&self, canvas: &mut Canvas, layout: &Layout) {
        self.title(canvas, layout, "Choose a disk", "");
        if self.disks.is_empty() {
            canvas.text(layout.left, layout.top + 40, "No disk found", 1, ui::DIM);
            return;
        }
        const CARD: usize = 92;
        const GAP: usize = 10;
        let top = layout.top + 36;
        let fits = (layout.height.saturating_sub(top + 96) / (CARD + GAP)).max(1);
        let first = window(self.disk, self.disks.len(), fits);
        for (row, n) in (first..self.disks.len()).take(fits).enumerate() {
            let disk = &self.disks[n];
            let y = top + row * (CARD + GAP);
            if n == self.disk {
                Self::selection(canvas, layout.left, y, layout.column, CARD);
            } else {
                canvas.panel(layout.left, y, layout.column, CARD, 8, ui::CARD);
            }
            let inner = layout.left + 20;
            let right = layout.left + layout.column - 20;
            let colour = if disk.refusal.is_some() {
                ui::DIM
            } else {
                ui::TEXT
            };
            canvas.text(inner, y + 14, &disk.model, 1, colour);
            canvas.text_right(right, y + 14, &size(disk.bytes()), 1, colour);
            let bar = Bar {
                x: inner,
                y: y + 42,
                width: right - inner,
                height: 10,
            };
            let segments = segments_of(disk, &[], false);
            self.draw_bar(canvas, n, &segments, &bar, None, None);
            match disk.refusal {
                Some(refusal) => canvas.text(inner, y + 64, refusal, 1, ui::ORANGE),
                None => self.legend(canvas, n, &segments, inner, right, y + 64),
            }
        }
    }

    /// Each partition of disk `n` as a swatch and its name, largest first,
    /// as many as fit between `x` and `right`.
    fn legend(
        &self,
        canvas: &mut Canvas,
        n: usize,
        segments: &[Segment],
        mut x: usize,
        right: usize,
        y: usize,
    ) {
        let disk = &self.disks[n];
        let mut names: Vec<(String, u32, u64)> = segments
            .iter()
            .filter_map(|segment| match segment.content {
                Content::Partition(p) => {
                    let partition = &disk.table.as_ref()?.partitions[p];
                    let (kind, colour) = kind(partition);
                    let name = match partition.name.trim() {
                        "" => kind,
                        name => name,
                    };
                    Some((String::from(name), colour, partition.blocks()))
                }
                Content::Foreign => Some((String::from("Unknown layout"), OTHER, 0)),
                Content::Free => None,
            })
            .collect();
        names.sort_by_key(|(.., blocks)| core::cmp::Reverse(*blocks));
        if names.is_empty() {
            canvas.text(x, y, "Empty", 1, ui::DIM);
        }
        for (index, (name, colour, _)) in names.iter().enumerate() {
            let width = 18 + canvas.text_width(name, 1);
            let more = names.len() - index;
            if x + width + 64 > right && more > 1 {
                canvas.text(x, y, &format!("+{more}"), 1, ui::DIM);
                break;
            }
            canvas.panel(x, y + 3, 10, 10, 3, *colour);
            canvas.text(x + 18, y, name, 1, ui::DIM);
            x += width + 20;
        }
    }

    /// Disk `n`'s `segments` as a bar, `highlight` outlined and `new` (a
    /// span of blocks) drawn as the partition macOS gets.
    fn draw_bar(
        &self,
        canvas: &mut Canvas,
        n: usize,
        segments: &[Segment],
        bar: &Bar,
        highlight: Option<usize>,
        new: Option<(u64, u64)>,
    ) {
        let disk = &self.disks[n];
        let blocks = disk.geometry.blocks.max(1);
        let x_of =
            |lba: u64| bar.x + (u128::from(lba) * bar.width as u128 / u128::from(blocks)) as usize;
        canvas.panel(bar.x, bar.y, bar.width, bar.height, 3, ui::LINE);
        let label = |canvas: &mut Canvas, x: usize, width: usize, name: &str| {
            if bar.height >= 30 && canvas.text_width(name, 1) + 16 < width {
                canvas.text(
                    x + 8,
                    bar.y + (bar.height - 16) / 2,
                    name,
                    1,
                    ui::BACKGROUND,
                );
            }
        };
        for (index, segment) in segments.iter().enumerate() {
            let (x, end) = (x_of(segment.first), x_of(segment.end));
            let width = end.saturating_sub(x).max(2);
            let colour = match segment.content {
                Content::Partition(p) => kind(&disk.table.as_ref().expect("table").partitions[p]).1,
                Content::Foreign => OTHER,
                Content::Free => ui::LINE,
            };
            if highlight == Some(index) {
                canvas.panel(
                    x,
                    bar.y.saturating_sub(3),
                    width,
                    bar.height + 6,
                    4,
                    ui::ORANGE,
                );
                canvas.panel(
                    x + 2,
                    bar.y - 1,
                    width.saturating_sub(4).max(1),
                    bar.height + 2,
                    3,
                    colour,
                );
            } else {
                canvas.fill(
                    x + 1,
                    bar.y,
                    width.saturating_sub(2).max(1),
                    bar.height,
                    colour,
                );
            }
            if segment.content != Content::Free {
                let (name, _, _) = segment_label(disk, *segment);
                label(canvas, x, width, &name);
            }
        }
        if let Some((first, end)) = new {
            let (x, right) = (x_of(first), x_of(end));
            let width = right.saturating_sub(x).max(2);
            canvas.fill(
                x + 1,
                bar.y,
                width.saturating_sub(2).max(1),
                bar.height,
                ui::BLUE,
            );
            label(canvas, x, width, "macOS");
        }
    }

    fn big_bar(&self, layout: &Layout) -> Bar {
        Bar {
            x: layout.left,
            y: layout.top + 36,
            width: layout.column,
            height: 36,
        }
    }

    fn draw_partitions(&self, canvas: &mut Canvas, layout: &Layout) {
        let disk = self.chosen();
        self.title(canvas, layout, &disk.model, &size(disk.bytes()));
        let segments = self.segments();
        let bar = self.big_bar(layout);
        self.draw_bar(canvas, self.disk, &segments, &bar, Some(self.row), None);
        let top = bar.y + bar.height + 24;
        let fits = (layout.height.saturating_sub(top + 120) / ROW).max(1);
        let first = window(self.row, segments.len(), fits);
        for (line, n) in (first..segments.len()).take(fits).enumerate() {
            let segment = segments[n];
            let y = top + line * ROW;
            let selected = n == self.row;
            if selected {
                Self::selection(canvas, layout.left, y, layout.column, ROW - 4);
            }
            let (name, kind, colour) = segment_label(disk, segment);
            let swatch = layout.left + 22;
            canvas.panel(swatch, y + 10, 12, 12, 3, colour);
            // Free space is an outline, not a colour of its own.
            if segment.content == Content::Free {
                let inside = if selected { ui::RAISED } else { ui::BACKGROUND };
                canvas.panel(swatch + 2, y + 12, 8, 8, 2, inside);
            }
            canvas.text(swatch + 24, y + 8, &name, 1, ui::TEXT);
            canvas.text(
                layout.left + layout.column * 11 / 20,
                y + 8,
                kind,
                1,
                ui::DIM,
            );
            let bytes = (segment.end - segment.first) * disk.geometry.block_bytes;
            canvas.text_right(
                layout.left + layout.column - 16,
                y + 8,
                &size(bytes),
                1,
                ui::TEXT,
            );
        }
        let y = top + fits.min(segments.len()) * ROW + 12;
        let deleting = self.deleting();
        let pending = if self.erased {
            String::from("Erasing the whole disk")
        } else if deleting.is_empty() {
            return;
        } else {
            let names: Vec<&str> = deleting
                .iter()
                .map(|&n| match disk.partition(n).name.trim() {
                    "" => kind(disk.partition(n)).0,
                    name => name,
                })
                .collect();
            format!("Deleting {}", names.join(", "))
        };
        canvas.text(layout.left + 22, y, &pending, 1, ui::ORANGE);
    }

    fn draw_size(&self, canvas: &mut Canvas, layout: &Layout) {
        let disk = self.chosen();
        self.title(canvas, layout, "Size for macOS", &disk.model);
        let bar = self.big_bar(layout);
        let block = disk.geometry.block_bytes;
        let new = match self.place {
            Some(Place::Free { first, .. }) => Some((first, first + self.size / block)),
            _ => None,
        };
        self.draw_bar(canvas, self.disk, &self.segments(), &bar, None, new);
        let (least, most, _) = self.size_range();
        let middle = layout.left + layout.column / 2;
        let y = bar.y + bar.height + 56;
        let shown = size(self.size);
        let half = canvas.text_width(&shown, 2) / 2;
        canvas.text_centred(middle, y, &shown, 2, ui::TEXT);
        canvas.text_right(middle - half - 24, y, "<", 2, ui::ORANGE);
        canvas.text(middle + half + 24, y, ">", 2, ui::ORANGE);
        let limits = format!("{} - {}", size(least), size(most));
        canvas.text_centred(middle, y + 48, &limits, 1, ui::DIM);
    }

    fn draw_confirm(&self, canvas: &mut Canvas, layout: &Layout) {
        let deleting = self.deleting();
        let destructive =
            self.erased || !deleting.is_empty() || matches!(self.place, Some(Place::Reuse(_)));
        let title = if destructive {
            "Erase and install macOS?"
        } else {
            "Install macOS?"
        };
        self.title(canvas, layout, title, "");
        let disk = self.chosen();
        let block = disk.geometry.block_bytes;
        let mut rows: Vec<(&str, String, u32)> = Vec::new();
        // Two disks of one model differ by size or serial number.
        let disk_line = format!("{}, {}", disk.model, size(disk.bytes()));
        rows.push(("Disk", disk_line, ui::TEXT));
        if let Some(serial) = &disk.serial {
            rows.push(("Serial", serial.clone(), ui::TEXT));
        }
        rows.push((
            "macOS",
            match self.place {
                Some(Place::Reuse(n)) => {
                    format!("{}, replaced", size(disk.partition(n).blocks() * block))
                }
                _ => format!("New {} partition", size(self.size)),
            },
            ui::TEXT,
        ));
        if self.erased {
            rows.push((
                "Erases",
                String::from("Everything on this disk"),
                ui::ORANGE,
            ));
        } else {
            for &n in &deleting {
                let partition = disk.partition(n);
                let name = match partition.name.trim() {
                    "" => kind(partition).0,
                    name => name,
                };
                let line = format!("{name}, {}", size(partition.blocks() * block));
                rows.push(("Deletes", line, ui::ORANGE));
            }
        }
        let card = 24 + rows.len().min(9) * 28;
        let top = layout.top + 36;
        canvas.panel(layout.left, top, layout.column, card, 8, ui::CARD);
        for (index, (label, value, colour)) in rows.iter().take(9).enumerate() {
            let y = top + 16 + index * 28;
            canvas.text(layout.left + 20, y, label, 1, ui::DIM);
            canvas.text(layout.left + 120, y, value, 1, *colour);
        }
        let buttons = top + card + 32;
        let action = if destructive {
            "Erase and Install"
        } else {
            "Install"
        };
        let right = layout.left + layout.column;
        let action_width = canvas.text_width(action, 1) + 48;
        Self::button(
            canvas,
            right - action_width,
            buttons,
            action_width,
            action,
            self.install,
        );
        let back_width = canvas.text_width("Back", 1) + 48;
        Self::button(
            canvas,
            right - action_width - 12 - back_width,
            buttons,
            back_width,
            "Back",
            !self.install,
        );
    }

    fn button(canvas: &mut Canvas, x: usize, y: usize, width: usize, text: &str, focused: bool) {
        let (background, foreground) = if focused {
            (ui::ORANGE, ui::BACKGROUND)
        } else {
            (ui::RAISED, ui::TEXT)
        };
        canvas.panel(x, y, width, 36, 8, background);
        canvas.text(x + 24, y + 10, text, 1, foreground);
    }
}

/// `disk` as it will be: its partitions but the `deleted` ones, or none
/// when it is `erased`, and the free space of a MiB or more between them.
fn segments_of(disk: &Disk, deleted: &[usize], erased: bool) -> Vec<Segment> {
    let (first, end) = (
        disk.geometry.first_usable(),
        disk.geometry.last_usable() + 1,
    );
    if disk.foreign && !erased {
        return vec![Segment {
            first,
            end,
            content: Content::Foreign,
        }];
    }
    let mut kept: Vec<(usize, &gpt::Partition)> = match (&disk.table, erased) {
        (Some(table), false) => table
            .partitions
            .iter()
            .enumerate()
            .filter(|(n, _)| !deleted.contains(n))
            .collect(),
        _ => Vec::new(),
    };
    kept.sort_by_key(|(_, p)| p.first_lba);
    let mut segments = Vec::new();
    let mut at = first;
    for (n, partition) in kept {
        if partition.first_lba >= at + disk.mib_blocks() {
            segments.push(Segment {
                first: at,
                end: partition.first_lba,
                content: Content::Free,
            });
        }
        segments.push(Segment {
            first: partition.first_lba,
            end: partition.last_lba + 1,
            content: Content::Partition(n),
        });
        at = at.max(partition.last_lba + 1);
    }
    if end >= at + disk.mib_blocks() {
        segments.push(Segment {
            first: at,
            end,
            content: Content::Free,
        });
    }
    segments
}

struct Layout {
    left: usize,
    column: usize,
    /// The first row under the header.
    top: usize,
    height: usize,
}

struct Bar {
    x: usize,
    y: usize,
    width: usize,
    height: usize,
}
