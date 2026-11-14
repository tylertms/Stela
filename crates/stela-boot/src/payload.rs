//! Install mode: find the restore payload on any volume and verify its
//! Erase subset, before boot services end. Nothing is written.
//!
//! `stela.cfg` turns this on with `--install-mode verify` and
//! `--install-build BUILD`. Without `--install-mode` the options pass
//! through to the mount unchanged.
//!
//! The firmware's names for a whole disk, which the boot choice and Install
//! macOS's disk picker find disks by, are read here too ([`disk_serial`],
//! [`disk_model`], [`device_path`]).
use crate::Refusal;
use crate::Result;
use crate::output;
use alloc::{
    format,
    string::{String, ToString},
    vec,
    vec::Vec,
};
use stela_contracts::refuse;
use uefi::{
    CString16, Handle,
    boot::{self, OpenProtocolAttributes, OpenProtocolParams, ScopedProtocol},
    proto::{
        device_path::{
            DevicePath,
            text::{AllowShortcuts, DisplayOnly},
        },
        media::{
            file::{Directory, File, FileAttribute, FileInfo, FileMode, RegularFile},
            fs::SimpleFileSystem,
        },
    },
};
use {
    stela_formats::payload::CopyError, stela_formats::payload::Layout,
    stela_formats::payload::MANIFEST_FILE, stela_formats::payload::MemberRead,
    stela_formats::payload::PayloadError, stela_formats::payload::PayloadManifest,
    stela_formats::payload::PayloadSource, stela_formats::payload::ReadAt,
    stela_formats::payload::copy_verified, stela_formats::payload::images_directory,
    stela_formats::payload::slice_count, stela_formats::payload::slice_path,
    stela_formats::payload::zip::Archive, stela_formats::payload::zip::MemberStream,
};

/// The exit code after an install-mode run; the mount itself does not
/// start.
const VERIFIED: u32 = 0x12;
const COPY_BYTES: usize = 1024 * 1024;

#[derive(Debug, Default)]
struct Install {
    verify: bool,
    build: Option<String>,
}

/// Read the install options in `stela.cfg`, skipping the mount's own.
fn parse(options: &str) -> Result<Install> {
    let mut install = Install::default();
    let mut words = options.split_ascii_whitespace();
    while let Some(name) = words.next() {
        if !name.starts_with("--install-") {
            words.next();
            continue;
        }
        let value = words
            .next()
            .ok_or(refuse!("install option without a value"))?;
        match name {
            "--install-mode" if value == "verify" => install.verify = true,
            "--install-mode" => return Err(refuse!("--install-mode must be verify")),
            "--install-build" => install.build = Some(value.into()),
            _ => return Err(refuse!("unknown install option")),
        }
    }
    if !install.verify && install.build.is_some() {
        return Err(refuse!("install options need --install-mode"));
    }
    Ok(install)
}

/// Run install mode if `stela.cfg` asks for it, which ends in
/// [`output::terminate`]. Otherwise return the mount's own options.
pub fn run(options: Vec<u8>) -> Result<Vec<u8>> {
    let text = core::str::from_utf8(&options).map_err(|_| refuse!("stela.cfg is not UTF-8"))?;
    let install = parse(text)?;
    if !install.verify {
        return Ok(options);
    }
    let build = install
        .build
        .as_deref()
        .ok_or(refuse!("install mode needs --install-build"))?;
    let (mut source, payload_path) = Source::find(build)?;
    output::arguments(format_args!(
        "PAYLOAD build={} model={} layout={:?} members={} bytes={} volume={payload_path}\n",
        source.manifest.product_build,
        source.manifest.model,
        source.manifest.layout,
        source.manifest.members.len(),
        source.manifest.total_bytes()
    ));
    let subset = source.manifest.erase_subset().map_err(report)?;
    let mut buffer = vec![0u8; COPY_BYTES];
    for member in &subset.members {
        copy_verified(&mut source, member, &mut buffer, |_| Ok::<(), ()>(()))
            .map_err(copy_report)?;
        output::arguments(format_args!(
            "VERIFIED {} bytes={}\n",
            member.name, member.bytes
        ));
    }
    output::arguments(format_args!(
        "STELA PAYLOAD VERIFIED members={} bytes={}; nothing was written\n",
        subset.members.len(),
        subset.total_bytes()
    ));
    output::terminate(VERIFIED)
}

fn report(error: PayloadError) -> Refusal {
    output::arguments(format_args!("payload refusal: {error}\n"));
    refuse!("install mode failed: {error}", %error)
}

fn copy_report<E: core::fmt::Debug>(error: CopyError<E>) -> Refusal {
    output::arguments(format_args!("payload refusal: {error}\n"));
    refuse!("install mode failed: {error}", %error)
}

fn path16(path: &str) -> Result<CString16> {
    CString16::try_from(path.replace('/', "\\").as_str()).map_err(|_| refuse!("path is not UCS-2"))
}

fn open_file(root: &mut Directory, path: &str) -> Result<RegularFile> {
    root.open(&path16(path)?, FileMode::Read, FileAttribute::empty())
        .map_err(|_| refuse!("cannot open a payload file"))?
        .into_regular_file()
        .ok_or(refuse!("a payload path is a directory"))
}

fn file_size(file: &mut RegularFile) -> Result<u64> {
    file.get_boxed_info::<FileInfo>()
        .map(|info| info.file_size())
        .map_err(|_| refuse!("cannot read a payload file's size"))
}

/// A whole file read at offsets, such as an IPSW.
struct UefiFile {
    file: RegularFile,
    length: u64,
}

impl ReadAt for UefiFile {
    type Error = Refusal;
    fn len(&self) -> u64 {
        self.length
    }
    fn read_at(&mut self, offset: u64, buffer: &mut [u8]) -> Result<()> {
        self.file
            .set_position(offset)
            .map_err(|_| refuse!("cannot seek a payload file"))?;
        let mut done = 0;
        while done < buffer.len() {
            let count = self
                .file
                .read(&mut buffer[done..])
                .map_err(|_| refuse!("cannot read a payload file"))?;
            if count == 0 {
                return Err(refuse!("payload file ended early"));
            }
            done += count;
        }
        Ok(())
    }
}

enum Store {
    Split {
        root: Directory,
        directory: String,
        slice_bytes: u64,
    },
    Archive {
        file: UefiFile,
        archive: Archive,
    },
}

/// The payload on one volume, which stays open while it is read.
struct Source {
    manifest: PayloadManifest,
    store: Store,
    _volume: ScopedProtocol<SimpleFileSystem>,
}

impl Source {
    /// The first volume with this build's payload, and its device path.
    fn find(build: &str) -> Result<(Self, String)> {
        let directory = images_directory(build);
        let manifest_path = format!("{directory}/{MANIFEST_FILE}");
        let handles =
            boot::find_handles::<SimpleFileSystem>().map_err(|_| refuse!("no file systems"))?;
        for handle in handles {
            let Ok(mut volume) = boot::open_protocol_exclusive::<SimpleFileSystem>(handle) else {
                continue;
            };
            let Ok(mut root) = volume.open_volume() else {
                continue;
            };
            let Ok(mut file) = open_file(&mut root, &manifest_path) else {
                continue;
            };
            let length = file_size(&mut file)?;
            let mut bytes =
                vec![0u8; usize::try_from(length).map_err(|_| refuse!("manifest size"))?];
            let mut reader = UefiFile { file, length };
            reader.read_at(0, &mut bytes)?;
            let manifest = PayloadManifest::parse(&bytes).map_err(report)?;
            if manifest.product_build != build {
                return Err(refuse!("payload.json is for another build"));
            }
            let store = match &manifest.layout {
                Layout::Split { slice_bytes } => Store::Split {
                    root,
                    directory: directory.clone(),
                    slice_bytes: *slice_bytes,
                },
                Layout::Archive { path } => {
                    let mut file = open_file(&mut root, path)?;
                    let length = file_size(&mut file)?;
                    if length != manifest.source.bytes {
                        return Err(refuse!("the IPSW's size differs from payload.json"));
                    }
                    let mut file = UefiFile { file, length };
                    let archive = Archive::open(&mut file).map_err(report)?;
                    Store::Archive { file, archive }
                }
                Layout::Staged => return Err(refuse!("payload.json describes a staged copy")),
                Layout::Pieces { .. } | Layout::Remote { .. } => {
                    return Err(refuse!(
                        "payload.json describes an install image, which --install reads"
                    ));
                }
            };
            let path = device_path(handle).unwrap_or_default();
            return Ok((
                Self {
                    manifest,
                    store,
                    _volume: volume,
                },
                path,
            ));
        }
        Err(refuse!("no volume has the payload for --install-build"))
    }
}

/// A split member, read slice after slice.
struct Slices<'a> {
    root: &'a mut Directory,
    directory: &'a str,
    name: String,
    bytes: u64,
    slice_bytes: u64,
    index: u64,
    current: Option<(RegularFile, u64)>,
    done: u64,
}

impl MemberRead for Slices<'_> {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, PayloadError> {
        let failed = |what: Refusal| PayloadError::Io(what.to_string());
        loop {
            if self.done == self.bytes {
                return Ok(0);
            }
            if let Some((file, left)) = &mut self.current
                && *left > 0
            {
                let take = buffer.len().min(*left as usize);
                let count = file
                    .read(&mut buffer[..take])
                    .map_err(|_| failed(refuse!("cannot read a slice")))?;
                if count == 0 {
                    return Err(failed(refuse!("a slice ended early")));
                }
                *left -= count as u64;
                self.done += count as u64;
                return Ok(count);
            }
            if self.index >= slice_count(self.bytes, self.slice_bytes) {
                return Err(failed(refuse!("a member has too few slices")));
            }
            let path = format!("{}/{}", self.directory, slice_path(&self.name, self.index));
            let mut file = open_file(self.root, &path).map_err(failed)?;
            let expected = (self.bytes - self.index * self.slice_bytes).min(self.slice_bytes);
            if file_size(&mut file).map_err(failed)? != expected {
                return Err(failed(refuse!("a slice has the wrong size")));
            }
            self.current = Some((file, expected));
            self.index += 1;
        }
    }
}

enum Reader<'a> {
    Slices(Slices<'a>),
    Archive(MemberStream<'a, UefiFile>),
}

impl MemberRead for Reader<'_> {
    fn read(&mut self, buffer: &mut [u8]) -> Result<usize, PayloadError> {
        match self {
            Self::Slices(reader) => reader.read(buffer),
            Self::Archive(reader) => reader.read(buffer),
        }
    }
}

impl PayloadSource for Source {
    type Reader<'a>
        = Reader<'a>
    where
        Self: 'a;

    fn manifest(&self) -> &PayloadManifest {
        &self.manifest
    }

    fn open(&mut self, name: &str) -> Result<Reader<'_>, PayloadError> {
        let member = self
            .manifest
            .member(name)
            .ok_or(PayloadError::Manifest("member is not in the payload"))?
            .clone();
        match &mut self.store {
            Store::Split {
                root,
                directory,
                slice_bytes,
            } => Ok(Reader::Slices(Slices {
                root,
                directory,
                name: member.name,
                bytes: member.bytes,
                slice_bytes: *slice_bytes,
                index: 0,
                current: None,
                done: 0,
            })),
            Store::Archive { file, archive } => {
                let entry = archive
                    .find(name)
                    .ok_or(PayloadError::Archive("member is not in the IPSW"))?
                    .clone();
                if entry.bytes != member.bytes {
                    return Err(PayloadError::Verification {
                        member: member.name,
                        reason: "IPSW entry size differs from the manifest",
                    });
                }
                Ok(Reader::Archive(archive.open_member(file, &entry)?))
            }
        }
    }
}

/// The C `wcslen` that `uefi`'s device-path text conversion links against;
/// the UEFI target's builtins have none. UEFI wide characters are 16 bits.
///
/// # Safety
///
/// `text` must point to a NUL-terminated UCS-2 string.
#[unsafe(no_mangle)]
pub unsafe extern "C" fn wcslen(text: *const u16) -> usize {
    let mut length = 0;
    // SAFETY: the caller passes a NUL-terminated string.
    while unsafe { text.add(length).read() } != 0 {
        length += 1;
    }
    length
}

pub(crate) fn device_path(handle: Handle) -> Option<String> {
    // SAFETY: the path is only read, while no driver changes this handle.
    let path = unsafe {
        boot::open_protocol::<DevicePath>(
            OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
    }
    .ok()?;
    path.to_string16(DisplayOnly(false), AllowShortcuts(false))
        .ok()
        .map(|text| text.to_string())
}

/// A page-aligned buffer, which every block device's IoAlign accepts.
pub(crate) struct Aligned {
    storage: Vec<u8>,
    start: usize,
    length: usize,
}

impl Aligned {
    pub(crate) fn new(length: usize) -> Self {
        let storage = vec![0u8; length + 4096];
        let start = storage.as_ptr().align_offset(4096);
        Self {
            storage,
            start,
            length,
        }
    }
    pub(crate) fn bytes(&mut self) -> &mut [u8] {
        &mut self.storage[self.start..self.start + self.length]
    }
}

/// The NVMe admin command that returns controller data, and the controller
/// data structure's serial number field.
const NVME_IDENTIFY: u8 = 0x06;
const NVME_IDENTIFY_CONTROLLER: u32 = 1;
const NVME_IDENTIFY_BYTES: usize = 4096;
const NVME_SERIAL: core::ops::Range<usize> = 4..24;
/// edk2 treats a zero timeout as already expired; its own admin commands
/// allow 5 seconds.
const NVME_TIMEOUT: core::time::Duration = core::time::Duration::from_secs(5);
/// ATA IDENTIFY DEVICE words 10-19: the serial number, and words 27-46:
/// the model number, two characters per word with the first in the high
/// byte.
const ATA_IDENTIFY_BYTES: usize = 512;
const ATA_SERIAL: core::ops::Range<usize> = 20..40;
const ATA_MODEL: core::ops::Range<usize> = 54..94;
/// ATA IDENTIFY DEVICE word 217: the nominal media rotation rate.
const ATA_ROTATION: usize = 217;
/// NVMe Identify Controller bytes 24-63: the model number.
const NVME_MODEL: core::ops::Range<usize> = 24..64;

fn trimmed(bytes: &[u8]) -> Option<String> {
    let text = core::str::from_utf8(bytes).ok()?.trim_matches([' ', '\0']);
    (!text.is_empty()).then(|| text.into())
}

/// A whole disk's serial number, from its NVMe controller, its ATA
/// IDENTIFY data or its USB device's serial number string. Other buses,
/// such as virtio, report none.
pub(crate) fn disk_serial(handle: Handle, path: &str) -> Option<String> {
    nvme_identify(path)
        .and_then(|data| trimmed(data.get(NVME_SERIAL)?))
        .or_else(|| ata_text(handle, ATA_SERIAL))
        .or_else(|| usb_string(path, |device| device.str_serial_number))
}

/// A whole disk's model name, from the same sources as [`disk_serial`]
/// (the USB device's product string).
pub(crate) fn disk_model(handle: Handle, path: &str) -> Option<String> {
    nvme_identify(path)
        .and_then(|data| trimmed(data.get(NVME_MODEL)?))
        .or_else(|| ata_text(handle, ATA_MODEL))
        .or_else(|| usb_string(path, |device| device.str_product))
}

/// A string of the USB device the disk belongs to, `index` naming it in
/// the device descriptor, in US English and trimmed as Stela's own USB
/// driver reads it (`stela_drivers::usb::xhci`), so both name the disk
/// alike. Hubs' paths prefix their devices', so the device is the one with
/// the longest path the disk's starts with.
fn usb_string(
    path: &str,
    index: impl FnOnce(&uefi::proto::usb::DeviceDescriptor) -> u8,
) -> Option<String> {
    use uefi::proto::usb::io::UsbIo;
    const LANGUAGE_EN_US: u16 = 0x0409;
    let (device, _) = boot::find_handles::<UsbIo>()
        .ok()?
        .into_iter()
        .filter_map(|device| Some((device, device_path(device)?)))
        .filter(|(_, prefix)| path == prefix || path.starts_with(format!("{prefix}/").as_str()))
        .max_by_key(|(_, prefix)| prefix.len())?;
    // SAFETY: descriptors are only read.
    let mut io = unsafe {
        boot::open_protocol::<UsbIo>(
            OpenProtocolParams {
                handle: device,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
    }
    .ok()?;
    let index = index(&io.device_descriptor().ok()?);
    if index == 0 {
        return None;
    }
    let text = io
        .string_descriptor(LANGUAGE_EN_US, index)
        .ok()?
        .to_string();
    let text = text.trim();
    (!text.is_empty()).then(|| text.into())
}

/// The Identify Controller data of the NVMe controller `path` is on.
fn nvme_identify(path: &str) -> Option<Vec<u8>> {
    use uefi::proto::nvme::{NvmeQueueType, NvmeRequestBuilder, pass_thru::NvmePassThru};
    for controller in boot::find_handles::<NvmePassThru>().ok()? {
        let Some(prefix) = device_path(controller) else {
            continue;
        };
        if !path.starts_with(format!("{prefix}/NVMe(").as_str()) {
            continue;
        }
        // SAFETY: an admin Identify command reads controller data only.
        let pass_thru = unsafe {
            boot::open_protocol::<NvmePassThru>(
                OpenProtocolParams {
                    handle: controller,
                    agent: boot::image_handle(),
                    controller: None,
                },
                OpenProtocolAttributes::GetProtocol,
            )
        }
        .ok()?;
        let request =
            NvmeRequestBuilder::new(pass_thru.io_align(), NVME_IDENTIFY, NvmeQueueType::ADMIN)
                .with_cdw10(NVME_IDENTIFY_CONTROLLER)
                .with_timeout(NVME_TIMEOUT)
                .with_transfer_buffer(NVME_IDENTIFY_BYTES)
                .ok()?
                .build();
        let response = pass_thru.controller().execute_command(request).ok()?;
        return Some(response.transfer_buffer()?.to_vec());
    }
    None
}

/// A whole disk's nominal rotation rate, from its ATA IDENTIFY data (word
/// 217), read as Stela's AHCI driver reads it. Other buses report none
/// through the firmware: its USB and NVMe disks give no such field.
pub(crate) fn disk_rotation(handle: Handle) -> stela_drivers::block::Rotation {
    ata_identify(handle)
        .and_then(|data| {
            data.get(2 * ATA_ROTATION..2 * ATA_ROTATION + 2)?
                .first_chunk()
                .copied()
        })
        .map_or(stela_drivers::block::Rotation::Unreported, |word| {
            stela_drivers::block::Rotation::from_nominal(u16::from_le_bytes(word))
        })
}

/// The disk's ATA IDENTIFY data, as much as the firmware's disk information
/// keeps for an AHCI or IDE disk.
fn ata_identify(handle: Handle) -> Option<Vec<u8>> {
    use uefi::proto::media::disk_info::{DiskInfo, DiskInfoInterface};
    // SAFETY: the identify data is only read.
    let info = unsafe {
        boot::open_protocol::<DiskInfo>(
            OpenProtocolParams {
                handle,
                agent: boot::image_handle(),
                controller: None,
            },
            OpenProtocolAttributes::GetProtocol,
        )
    }
    .ok()?;
    if !matches!(
        info.interface(),
        DiskInfoInterface::AHCI | DiskInfoInterface::IDE
    ) {
        return None;
    }
    let mut data = vec![0u8; ATA_IDENTIFY_BYTES];
    let bytes = info.identify(&mut data).ok()?;
    data.truncate(bytes);
    Some(data)
}

/// The text in `range` of the disk's ATA IDENTIFY data.
fn ata_text(handle: Handle, range: core::ops::Range<usize>) -> Option<String> {
    let mut text = ata_identify(handle)?.get(range)?.to_vec();
    for pair in text.as_chunks_mut::<2>().0 {
        pair.swap(0, 1);
    }
    // As Stela's AHCI driver reads the field, so both name the disk alike.
    let text = stela_drivers::block::ascii(&text);
    (!text.is_empty()).then_some(text)
}
