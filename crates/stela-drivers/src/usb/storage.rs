//! SCSI-transparent USB disks over native BOT and UAS owners. Logical units
//! share their transport's serialized command/tag/DMA and quarantine boundary.
//! Every command carries the transport LUN; uncertain DMA prevents
//! publication of any disk belonging to the interface.
// SPDX-License-Identifier: GPL-2.0-only AND GPL-2.0-or-later
// Protocol/SCSI references: Linux usb-storage, Matthew Dharm and contributors;
// see NOTICE for the pinned sources, notices and implementation scope.
use super::descriptor::{Endpoint, Interface, setup};
use super::xhci::{Device, MAX_TD, Plan, STALL, Shared};
use crate::{
    block::{
        BlockDevice, BlockError, Command, Features, Finished, Identity, Rotation, Sense,
        SenseCompletion, Transport,
    },
    bus::dma::DmaAllocator,
};
#[cfg(test)]
use alloc::vec;
use alloc::{format, rc::Rc, string::String, vec::Vec};
use core::cell::{Cell, RefCell};
mod media;
pub use media::UsbStorage;
mod bot;
mod scsi;
pub mod uas;
use bot::{Bot, Io, Native};
pub(in crate::usb) use bot::{EhciBulk, ehci_error};
pub const CLASS: u8 = 8;
const SCSI: u8 = 6;
const BULK_ONLY: u8 = 0x50;
const READY_TRIES: usize = 20;
const READY_DELAY_US: u64 = 100_000;
/// sd's `SD_MAX_RETRIES`, and the runtime after which a command is no
/// longer requeued: (`SD_MAX_RETRIES` + 1) times sd's 30 s timeout
/// (`scsi_cmd_runtime_exceeced`).
const SD_MAX_RETRIES: u32 = 5;
const SD_RUNTIME_US: u64 = (SD_MAX_RETRIES as u64 + 1) * 30_000_000;

mod binding;
pub mod settle;
pub use binding::Binding;
pub(super) use binding::discover as discover_binding;

pub(super) fn plan(
    device: &Device,
    interface: &Interface,
    all: &[Interface],
    controller: Option<&super::xhci::Controller>,
) -> Result<Plan, &'static str> {
    binding::select(device, interface, all, controller).map(Plan::Storage)
}

/// Which way a command's data goes.
#[derive(Clone, Copy)]
pub enum Data<'a> {
    None,
    In(usize),
    Out(&'a [u8]),
}

enum BlockData<'a> {
    None,
    Read(&'a mut [u8]),
    Write(&'a [u8]),
}

const SHORT_READ: BlockError = BlockError::Protocol("short USB storage read");

fn copy_read(output: &mut [u8], bytes: &[u8]) -> Result<(), BlockError> {
    if bytes.len() != output.len() {
        return Err(SHORT_READ);
    }
    output.copy_from_slice(bytes);
    Ok(())
}

/// How a unit's write cache is found, as Linux `sd_read_cache_type` finds
/// it: from the Caching mode page, or assumed when a transport's devices
/// mishandle that page (`skip_ms_page_8`, with or without `wce_default_on`).
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Caching {
    Query,
    WriteThrough,
    WriteBack,
}

/// SCSI semantics are shared across BOT and UAS; each transport keeps its
/// own framing, LUN address type, command ownership and uncertainty boundary.
trait Scsi {
    type Lun: Copy + Default + PartialEq + core::fmt::Display;
    fn command(
        &mut self,
        lun: Self::Lun,
        cdb: &[u8],
        data: Data<'_>,
    ) -> Result<Vec<u8>, BlockError>;
    fn read_into(
        &mut self,
        lun: Self::Lun,
        cdb: &[u8],
        output: &mut [u8],
    ) -> Result<(), BlockError> {
        let bytes = self.command(lun, cdb, Data::In(output.len()))?;
        copy_read(output, &bytes)
    }
    fn delay(&mut self, micros: u64) -> Result<(), BlockError>;
    /// The transport's clock, in microseconds.
    fn now_us(&self) -> u64;
    fn poisoned(&self) -> bool;
    /// Whether the transport's host controller sends the handoff's
    /// completion signal as the transport's transfers complete.
    fn signals(&self) -> bool {
        false
    }
    /// Whether a begun command's transfer may have completed; a look at
    /// memory alone.
    fn posted(&self) -> bool {
        false
    }
    /// Whether [`Self::begin`] runs commands without waiting for them.
    fn queued(&self) -> bool {
        false
    }
    /// Begin `cdb` without waiting for its transfers; `None` from a
    /// transport that runs commands only to completion.
    fn begin(
        &mut self,
        _lun: Self::Lun,
        _cdb: &[u8],
        _data: Data<'_>,
    ) -> Option<Result<(), BlockError>> {
        None
    }
    /// The begun command's result once complete; never waits.
    fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        None
    }
    /// Whether a begun command has not ended, or the transport recovers.
    fn busy(&mut self) -> bool {
        false
    }
    /// What the last command's data phase moved in before it failed.
    fn take_failed_data(&mut self) -> Option<Vec<u8>>;
    fn media_publication(&mut self, _lun: Self::Lun, _valid: Rc<Cell<bool>>, _removable: bool) {}
    fn max_lun(&self) -> u8 {
        0
    }
    fn generation(&self) -> u64 {
        0
    }
    fn max_transfer_bytes(&self) -> usize {
        MAX_TD
    }
    /// Live transport/DMA ceiling for a single read, including enclosure limits.
    fn read_command_ceiling(&self) -> usize {
        self.max_transfer_bytes()
    }
    fn capacity16_first(&self) -> bool {
        false
    }
    fn adjust_capacity(&self, blocks: u64) -> Option<u64> {
        Some(blocks)
    }
    fn separate_last_sector(&self) -> bool {
        false
    }
    fn read_before_mode_sense(
        &mut self,
        _lun: Self::Lun,
        _block_size: usize,
    ) -> Result<(), BlockError> {
        Ok(())
    }
    /// The mode page that reports write protection, or none when the unit
    /// is assumed writable (Linux `skip_ms_page_3f`).
    fn write_protect_page(&self) -> Option<u8> {
        Some(scsi::ALL_PAGES)
    }
    fn caching(&self) -> Caching {
        Caching::Query
    }
    fn report_luns_permitted(&self) -> bool {
        true
    }
    /// Whether the transport's quirks let the unit take FUA writes.
    fn fua_permitted(&self) -> bool {
        false
    }
    /// The step after a failed class reset, as Linux `usb_stor_port_reset`
    /// and the re-enumeration after it: reset the device's port, check that
    /// the same device answers, and return a fresh transport bound to it.
    /// None for a transport without that step. `dma` supplies whatever the
    /// host cannot reuse from the old binding.
    #[allow(
        clippy::type_complexity,
        reason = "the replacement transport, or the failed operation and its exact cause"
    )]
    fn reset_device(
        &mut self,
        _dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Self, (String, BlockError)>>
    where
        Self: Sized,
    {
        None
    }
    /// Whether [`Self::reset_device`] needs memory its old binding cannot
    /// give back: a device behind a hub gets new rings.
    fn reset_needs_allocation(&self) -> bool {
        false
    }
}
impl<I: Io> Scsi for Bot<I> {
    type Lun = u8;
    fn command(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
        Bot::command(self, lun, cdb, data)
    }
    fn read_into(&mut self, lun: u8, cdb: &[u8], output: &mut [u8]) -> Result<(), BlockError> {
        Bot::read_into(self, lun, cdb, output)
    }
    fn take_failed_data(&mut self) -> Option<Vec<u8>> {
        Bot::take_failed_data(self)
    }
    fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
        Bot::delay(self, micros)
    }
    fn now_us(&self) -> u64 {
        Bot::now_us(self)
    }
    fn poisoned(&self) -> bool {
        Bot::poisoned(self)
    }
    fn signals(&self) -> bool {
        self.io.signals()
    }
    fn posted(&self) -> bool {
        self.io.posted()
    }
    fn queued(&self) -> bool {
        true
    }
    fn begin(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Option<Result<(), BlockError>> {
        Some(Bot::begin(self, lun, cdb, data))
    }
    fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        Bot::end(self)
    }
    fn busy(&mut self) -> bool {
        Bot::busy(self)
    }
    fn max_lun(&self) -> u8 {
        Bot::max_lun(self)
    }
    /// Linux usb-storage skips the Caching page for every disk and assumes
    /// write-through without its `WRITE_CACHE` quirk.
    fn caching(&self) -> Caching {
        if self.write_cache() {
            Caching::WriteBack
        } else {
            Caching::WriteThrough
        }
    }
    fn write_protect_page(&self) -> Option<u8> {
        self.write_protect().then_some(scsi::ALL_PAGES)
    }
    fn generation(&self) -> u64 {
        Bot::generation(self)
    }
    fn max_transfer_bytes(&self) -> usize {
        Bot::max_transfer_bytes(self)
    }
    fn capacity16_first(&self) -> bool {
        Bot::capacity16_first(self)
    }
    fn adjust_capacity(&self, blocks: u64) -> Option<u64> {
        Bot::capacity(self, blocks)
    }
    fn separate_last_sector(&self) -> bool {
        true
    }
    fn read_before_mode_sense(&mut self, lun: u8, block_size: usize) -> Result<(), BlockError> {
        Bot::read_before_mode_sense(self, lun, block_size)
    }
    fn reset_device(
        &mut self,
        dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Self, (String, BlockError)>> {
        I::reset_device(self, dma)
    }
    fn reset_needs_allocation(&self) -> bool {
        self.io.reset_needs_allocation()
    }
}
impl Scsi for uas::Host {
    type Lun = uas::Lun;
    fn command(
        &mut self,
        lun: uas::Lun,
        cdb: &[u8],
        data: Data<'_>,
    ) -> Result<Vec<u8>, BlockError> {
        uas::Host::command(self, lun.bytes(), cdb, data)
    }
    fn read_into(
        &mut self,
        lun: uas::Lun,
        cdb: &[u8],
        output: &mut [u8],
    ) -> Result<(), BlockError> {
        uas::Host::read_into(self, lun.bytes(), cdb, output)
    }
    fn take_failed_data(&mut self) -> Option<Vec<u8>> {
        uas::Host::take_failed_data(self)
    }
    fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
        uas::Host::delay(self, micros)
    }
    fn now_us(&self) -> u64 {
        uas::Host::now_us(self)
    }
    fn poisoned(&self) -> bool {
        self.requires_rebind()
    }
    fn signals(&self) -> bool {
        uas::Host::signals(self)
    }
    fn posted(&self) -> bool {
        uas::Host::posted(self)
    }
    fn queued(&self) -> bool {
        true
    }
    fn begin(
        &mut self,
        lun: uas::Lun,
        cdb: &[u8],
        data: Data<'_>,
    ) -> Option<Result<(), BlockError>> {
        Some(uas::Host::begin(self, lun.bytes(), cdb, data))
    }
    fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        uas::Host::end(self)
    }
    fn busy(&mut self) -> bool {
        uas::Host::busy(self)
    }
    fn write_protect_page(&self) -> Option<u8> {
        uas::Host::write_protect_page(self)
    }
    fn caching(&self) -> Caching {
        uas::Host::caching(self)
    }
    fn report_luns_permitted(&self) -> bool {
        uas::Host::report_luns_permitted(self)
    }
    fn fua_permitted(&self) -> bool {
        uas::Host::fua_permitted(self)
    }
    fn reset_device(
        &mut self,
        dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Self, (String, BlockError)>> {
        Some(uas::Host::reset_and_rebind(self, dma))
    }
    fn reset_needs_allocation(&self) -> bool {
        uas::Host::reset_needs_allocation(self)
    }
    fn max_transfer_bytes(&self) -> usize {
        uas::Host::max_transfer_bytes(self)
    }
    fn read_command_ceiling(&self) -> usize {
        // UAS's transfer limit already includes its enclosure-specific
        // sector cap, so it is the complete read-command ceiling.
        uas::Host::max_transfer_bytes(self)
    }
    fn read_before_mode_sense(
        &mut self,
        lun: uas::Lun,
        block_size: usize,
    ) -> Result<(), BlockError> {
        uas::Host::read_before_mode_sense(self, lun.bytes(), block_size)
    }
}

/// One logical unit. Its shared owner serializes complete CDBs and sense.
pub struct UsbDisk {
    disk: UsbTransport,
}
enum UsbTransport {
    Bot(Disk<media::Owner<Bot<Native>>>),
    EhciBot(Disk<media::Owner<Bot<EhciBulk>>>),
    Uas(Disk<media::Owner<uas::Host>>),
}

struct Disk<T: Scsi> {
    bot: Rc<RefCell<T>>,
    lun: T::Lun,
    identity: Identity,
    media: Rc<Cell<bool>>,
    /// INQUIRY's RMB: a UNIT ATTENTION reports a media change.
    removable: bool,
    /// The command this unit runs, as SCSI commands one at a time
    /// ([`Disk::start`]).
    running: Option<Running>,
    next_tag: u32,
}

/// A block command a unit runs as consecutive SCSI commands, none waited
/// for: a transfer split where the transport needs it.
struct Running {
    tag: u32,
    command: Command,
    /// The bytes of the transfer its SCSI commands have moved.
    done: usize,
    /// The begun SCSI command's transfer bytes.
    begun: Option<usize>,
    /// A read's bytes so far.
    data: Vec<u8>,
    result: Option<Finished>,
}

pub(super) fn discover_retained(
    controller: Shared,
    device: &Device,
    interface: &Interface,
    endpoints: [Endpoint; 2],
    dma: &mut dyn DmaAllocator,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<(UsbStorage, Vec<UsbDisk>), BlockError> {
    let command_us = bot::scan_command_time();
    let bot = Bot::native(
        Native {
            controller,
            id: device.id,
            endpoints,
            command_us: command_us.clone(),
        },
        device,
        interface,
        dma,
    )?;
    let owner = Rc::new(RefCell::new(media::Owner::new(
        bot,
        format!("{} interface {}", device.location, interface.number),
        device.serial.as_deref().unwrap_or(&device.location).into(),
    )));
    let mut storage = UsbStorage {
        owner: media::TransportOwner::Bot(owner),
    };
    // As the SCSI error handler does for a first command that times out:
    // one device reset and fresh probes before the interface is given up.
    let disks = storage.service(dma, failures);
    bot::scanned(&command_us);
    Ok((storage, disks?))
}

/// A SCSI-transparent Bulk-Only interface.
pub(in crate::usb) fn bulk_only(interface: &Interface) -> bool {
    (interface.class, interface.subclass, interface.protocol) == (CLASS, SCSI, BULK_ONLY)
}

/// Discover block devices through a Bulk-Only interface of a configured
/// high-speed EHCI device, kept for rescan as [`discover_retained`] keeps
/// an xHCI one.
pub(in crate::usb) fn discover_ehci(
    io: EhciBulk,
    interface: &Interface,
    location: String,
    serial: String,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<(UsbStorage, Vec<UsbDisk>), BlockError> {
    let bot = Bot::ehci(io, interface)?;
    let owner = Rc::new(RefCell::new(media::Owner::new(
        bot,
        format!("{location} interface {}", interface.number),
        serial,
    )));
    let mut storage = UsbStorage {
        owner: media::TransportOwner::EhciBot(owner),
    };
    let disks = storage.rescan(failures)?;
    Ok((storage, disks))
}

/// Discover block devices through a configured UAS interface.
/// Use [`discover_uas_retained`] to keep initially empty interfaces for rescan.
///
/// # Safety
/// The exclusive attachment/configuration/unconfigured endpoint requirements of
/// [`uas::Host::new`] apply.
pub unsafe fn discover_uas(
    controller: Shared,
    device: &Device,
    binding: uas::Binding,
    dma: &mut dyn DmaAllocator,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<Vec<UsbDisk>, BlockError> {
    // SAFETY: this caller grants the identical constructor contract.
    unsafe { discover_uas_retained(controller, device, binding, dma, failures) }
        .map(|(_, disks)| disks)
}

/// Keep a UAS interface independently of the current ready-media inventory.
///
/// # Safety
/// The exclusive attachment/configuration/unconfigured endpoint requirements of
/// [`uas::Host::new`] apply.
pub unsafe fn discover_uas_retained(
    controller: Shared,
    device: &Device,
    binding: uas::Binding,
    dma: &mut dyn DmaAllocator,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<(UsbStorage, Vec<UsbDisk>), BlockError> {
    let prefix = format!("{} interface {}", device.location, binding.interface());
    let serial = device.serial.as_deref().unwrap_or(&device.location).into();
    // SAFETY: caller owns this exact configured interface and epoch.
    let host = unsafe { uas::Host::new(controller, device, binding, dma)? };
    let owner = Rc::new(RefCell::new(media::Owner::new(host, prefix, serial)));
    let mut storage = UsbStorage {
        owner: media::TransportOwner::Uas(owner),
    };
    // As the SCSI error handler does for a first command that times out:
    // one device reset and fresh probes before the interface is given up.
    let disks = storage.service(dma, failures)?;
    Ok((storage, disks))
}

fn discover_uas_units<T: Scsi<Lun = uas::Lun>>(
    owner: Rc<RefCell<T>>,
    prefix: &str,
    serial: &str,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<Vec<Disk<T>>, BlockError> {
    let zero = uas::Lun::default();
    let initial = inquiry(&mut *owner.borrow_mut(), zero);
    if owner.borrow().poisoned() {
        return Err(quarantined());
    }
    // The initial INQUIRY establishes the ANSI level. Failure to learn
    // it keeps the source's conservative eight-unit sequential fallback.
    let version = initial.as_ref().map_or(0, |bytes| bytes[2] & 0x0f);
    let mut disks = Vec::new();
    // The source scans LUN0 before REPORT LUNS. A real attached unit
    // must survive a report that omits zero; an absent unit is never invented.
    let initial_present = initial
        .as_ref()
        .is_ok_and(|bytes| bytes[0] >> 5 != 3 && bytes[0] & 31 != 31);
    if initial_present {
        publish_unit(
            &owner,
            zero,
            initial.clone(),
            prefix,
            serial,
            failures,
            &mut disks,
        )?;
    }
    let reported = if version >= 3 && owner.borrow().report_luns_permitted() {
        report_luns(&mut *owner.borrow_mut())?
    } else {
        None
    };
    if let Some(luns) = reported {
        for lun in luns {
            if lun == zero && initial_present {
                continue;
            }
            let found = if lun == zero {
                initial.clone()
            } else {
                inquiry(&mut *owner.borrow_mut(), lun)
            };
            publish_unit(&owner, lun, found, prefix, serial, failures, &mut disks)?;
        }
    } else {
        let limit = if version >= 3 { 256 } else { 8 };
        for number in 0..limit {
            if number == 0 && initial_present {
                continue;
            }
            let lun = uas::Lun::sequential(number);
            let found = if number == 0 {
                initial.clone()
            } else {
                inquiry(&mut *owner.borrow_mut(), lun)
            };
            if owner.borrow().poisoned() {
                return Err(quarantined());
            }
            // Sequential fallback stops at the first absent logical
            // unit, not at a present card-reader slot with no inserted media.
            if found
                .as_ref()
                .is_ok_and(|bytes| bytes[0] >> 5 == 3 || bytes[0] & 31 == 31)
                || matches!(&found, Err(BlockError::Sense(sense))
                    if sense.key == Sense::ILLEGAL_REQUEST && sense.asc == Sense::UNSUPPORTED_UNIT)
            {
                break;
            }
            let failed_inquiry = found.is_err();
            publish_unit(&owner, lun, found, prefix, serial, failures, &mut disks)?;
            if failed_inquiry {
                break;
            }
        }
    }
    Ok(disks)
}

fn quarantined() -> BlockError {
    BlockError::Unsupported("USB storage interface quarantined during discovery")
}

fn publication<T: Scsi>(
    owner: &Rc<RefCell<T>>,
    lun: T::Lun,
    (mut identity, removable): (Identity, bool),
) -> Result<Disk<T>, BlockError> {
    let media = Rc::new(Cell::new(true));
    // The cache query is still part of revalidation. Register this publication
    // before it can report a media change, including for the first/only LUN.
    owner
        .borrow_mut()
        .media_publication(lun, media.clone(), removable);
    {
        let mut transport = owner.borrow_mut();
        let (write_cache, fua) = caching(&mut *transport, lun)?;
        let size = identity.block_size as usize;
        let max_transfer = transport.max_transfer_bytes() / size * size;
        let ceiling = transport.read_command_ceiling();
        let max_read_transfer =
            scsi::probe_read_limit(size, max_transfer, ceiling, |cdb, bytes| {
                transport.command(lun, cdb, Data::In(bytes))
            })?;
        identity.features = Features {
            write_cache,
            fua: write_cache && fua && transport.fua_permitted(),
            max_transfer,
            max_read_transfer,
            discard: None,
            write_zeroes: None,
            // Bulk-Only units skip the VPD pages, Block Device
            // Characteristics among them, as Linux's usb-storage does; a UAS
            // unit's is not read.
            rotation: Rotation::Unreported,
        };
    }
    Ok(Disk {
        bot: owner.clone(),
        lun,
        identity,
        media,
        removable,
        running: None,
        next_tag: 0,
    })
}

/// Linux `sd_read_cache_type`: the Caching page's WCE bit and its header's
/// DPOFUA bit, or the transport's source-selected assumption, which reads
/// no DPOFUA. Its recognized 05/24/00 invalid field response permits the
/// default write-through policy. Other failures cannot establish cache
/// policy for a publication of the earlier metadata. usb-storage skips the
/// VPD pages (`skip_vpd_pages`), so no unit is offered discard.
fn caching<T: Scsi>(bot: &mut T, lun: T::Lun) -> Result<(bool, bool), BlockError> {
    match bot.caching() {
        Caching::WriteThrough => Ok((false, false)),
        Caching::WriteBack => Ok((true, false)),
        Caching::Query => match bot.command(
            lun,
            &scsi::mode_sense(scsi::CACHING_PAGE),
            Data::In(scsi::MODE_SENSE_BYTES),
        ) {
            // A complete header-only response establishes that no caching
            // page exists (sd_read_cache_type's No Caching mode page path).
            Ok(mode) if mode.len() >= 4 && mode[0] == 3 && mode[3] == 0 => Ok((false, false)),
            Ok(mode) if scsi::write_protected(&mode).is_some() => scsi::write_cache_enabled(&mode)
                .map(|enabled| (enabled, scsi::fua_supported(&mode)))
                .ok_or(BlockError::Protocol(
                    "missing or malformed USB Caching mode page",
                )),
            Ok(_) => Err(BlockError::Protocol("malformed USB mode parameter header")),
            Err(BlockError::Sense(Sense {
                key: Sense::ILLEGAL_REQUEST,
                asc: 0x24,
                ascq: 0,
            })) => Ok((false, false)),
            Err(error) => Err(error),
        },
    }
}

fn publish_unit<T: Scsi>(
    owner: &Rc<RefCell<T>>,
    lun: T::Lun,
    found: Result<Vec<u8>, BlockError>,
    prefix: &str,
    serial: &str,
    failures: &mut Vec<(String, BlockError)>,
    disks: &mut Vec<Disk<T>>,
) -> Result<(), BlockError> {
    let location = format!("{prefix} LUN {lun}");
    let result = found
        .and_then(|bytes| {
            probe_identified(&mut *owner.borrow_mut(), lun, &bytes, &location, serial)
        })
        .and_then(|identified| publication(owner, lun, identified));
    match result {
        Ok(disk) => disks.push(disk),
        Err(error) => failures.push((location, error)),
    }
    if owner.borrow().poisoned() {
        return Err(quarantined());
    }
    Ok(())
}

/// Linux's initial 4 KiB report grows to the list extent. Stela's native
/// transfer bound is explicit: an oversized, malformed or unstable list cannot
/// become an apparently complete prefix of disks.
fn report_luns<T: Scsi<Lun = uas::Lun>>(
    owner: &mut T,
) -> Result<Option<Vec<uas::Lun>>, BlockError> {
    let mut allocation = 4096usize;
    for _ in 0..3 {
        let mut cdb = [0u8; 12];
        cdb[0] = scsi::REPORT_LUNS;
        cdb[6..10].copy_from_slice(&(allocation as u32).to_be_bytes());
        let mut received = None;
        for attempt in 0..3 {
            match owner.command(uas::Lun::default(), &cdb, Data::In(allocation)) {
                Ok(bytes) => {
                    received = Some(bytes);
                    break;
                }
                Err(_) if owner.poisoned() => return Err(quarantined()),
                Err(BlockError::Sense(sense))
                    if sense.key == Sense::UNIT_ATTENTION && attempt < 2 =>
                {
                    continue;
                }
                // Source command failures fall back to sequential
                // INQUIRY. A poisoned host can never enter this fallback.
                Err(_) => return Ok(None),
            }
        }
        let bytes = received.ok_or(BlockError::Unsupported("REPORT LUNS did not complete"))?;
        if bytes.len() < 8 {
            return Err(BlockError::Unsupported("short REPORT LUNS header"));
        }
        let length = u32::from_be_bytes(bytes[..4].try_into().unwrap()) as usize;
        let total = length
            .checked_add(8)
            .ok_or(BlockError::Unsupported("REPORT LUNS length overflow"))?;
        if length & 7 != 0 || total > MAX_TD {
            return Err(BlockError::Unsupported(
                "REPORT LUNS extent outside native bound",
            ));
        }
        if total > allocation {
            allocation = total;
            continue;
        }
        if total > bytes.len() {
            return Err(BlockError::Unsupported("short REPORT LUNS list"));
        }
        let mut luns = Vec::new();
        for address in bytes[8..total].as_chunks::<8>().0 {
            let lun = uas::Lun::from_bytes(*address);
            if luns.contains(&lun) {
                return Err(BlockError::Unsupported("duplicate REPORT LUNS address"));
            }
            luns.push(lun);
        }
        return Ok(Some(luns));
    }
    Err(BlockError::Unsupported(
        "REPORT LUNS list changed repeatedly",
    ))
}

fn discover_units<T: Scsi<Lun = u8>>(
    bot: Rc<RefCell<T>>,
    prefix: &str,
    serial: &str,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<Vec<Disk<T>>, BlockError> {
    let mut disks = Vec::new();
    let max_lun = bot.borrow().max_lun();
    for lun in 0..=max_lun {
        let location = format!("{prefix} LUN {lun}");
        let identified = probe(&mut *bot.borrow_mut(), lun, &location, serial);
        let result = identified.and_then(|identified| publication(&bot, lun, identified));
        match result {
            Ok(disk) => disks.push(disk),
            Err(error) => {
                failures.push((location, error));
                // An uncertain interface cannot publish previously found disks
                // as usable: quarantine the entire owner and discard the list.
                if bot.borrow().poisoned() {
                    return Err(BlockError::Unsupported(
                        "USB storage interface quarantined during discovery",
                    ));
                }
            }
        }
    }
    Ok(disks)
}

/// The unit's identity and whether its medium is removable.
fn probe<T: Scsi>(
    bot: &mut T,
    lun: T::Lun,
    location: &str,
    serial: &str,
) -> Result<(Identity, bool), BlockError> {
    let inquiry = inquiry(bot, lun)?;
    probe_identified(bot, lun, &inquiry, location, serial)
}
fn inquiry<T: Scsi>(bot: &mut T, lun: T::Lun) -> Result<Vec<u8>, BlockError> {
    let mut result = Err(BlockError::Unsupported("INQUIRY did not complete"));
    for _ in 0..3 {
        result = bot.command(lun, &scsi::inquiry(), Data::In(scsi::INQUIRY_BYTES));
        if !matches!(&result, Err(BlockError::Sense(sense)) if sense.key == Sense::UNIT_ATTENTION
            && matches!(sense.asc, Sense::MEDIUM_CHANGED | Sense::RESET_OCCURRED))
        {
            break;
        }
    }
    let bytes = result?;
    if bytes.len() < scsi::INQUIRY_BYTES {
        return Err(BlockError::Unsupported("short INQUIRY data"));
    }
    Ok(bytes)
}
fn probe_identified<T: Scsi>(
    bot: &mut T,
    lun: T::Lun,
    inquiry: &[u8],
    location: &str,
    serial: &str,
) -> Result<(Identity, bool), BlockError> {
    if inquiry[0] != 0 {
        return Err(BlockError::Unsupported(
            "USB LUN is not an attached direct-access block device",
        ));
    }
    let text = |bytes: &[u8]| String::from(String::from_utf8_lossy(bytes).trim());
    let mut ready = false;
    let mut started = false;
    for attempt in 0..READY_TRIES {
        match bot.command(lun, &[scsi::TEST_UNIT_READY, 0, 0, 0, 0, 0], Data::None) {
            Ok(_) => {
                ready = true;
                break;
            }
            // No-media must precede the general UA retry: key6/ASC3a also
            // reports absence, and must never lead to capacity/mode queries.
            Err(BlockError::Sense(sense)) if media::no_media(&sense) => {
                return Err(BlockError::Sense(sense));
            }
            // Linux `sd_spinup_disk`: a unit that needs an initializing
            // command gets START STOP UNIT once, then is polled again.
            Err(BlockError::Sense(sense))
                if !started
                    && sense.key == Sense::NOT_READY
                    && sense.asc == Sense::NOT_READY_CAUSE
                    && sense.ascq == Sense::INITIALIZING_COMMAND_REQUIRED =>
            {
                started = true;
                bot.command(lun, &scsi::start_unit(), Data::None)?;
            }
            // The command has already completed same-LUN REQUEST SENSE.
            Err(BlockError::Sense(sense))
                if sense.key == Sense::UNIT_ATTENTION
                    || sense.key == Sense::NOT_READY
                        && sense.asc == Sense::NOT_READY_CAUSE
                        && !matches!(sense.ascq, 3 | 0xb | 0xc | 0x1b | 0x24 | 0x25) =>
            {
                if attempt + 1 < READY_TRIES {
                    bot.delay(READY_DELAY_US)?;
                }
            }
            Err(error) => return Err(error),
        }
    }
    if !ready {
        return Err(BlockError::Unsupported("USB storage never became ready"));
    }
    let (reported, block_size) = if bot.capacity16_first() {
        capacity16(bot, lun)?
    } else {
        let mut cdb = [0; 10];
        cdb[0] = scsi::READ_CAPACITY_10;
        let capacity = bot.command(lun, &cdb, Data::In(8))?;
        if capacity.len() < 8 {
            return Err(BlockError::Unsupported("short READ CAPACITY data"));
        }
        let last = u32::from_be_bytes(capacity[..4].try_into().unwrap());
        if last == u32::MAX {
            capacity16(bot, lun)?
        } else {
            (
                u64::from(last) + 1,
                u32::from_be_bytes(capacity[4..8].try_into().unwrap()),
            )
        }
    };
    let blocks = bot
        .adjust_capacity(reported)
        .ok_or(BlockError::Unsupported(
            "USB storage invalid adjusted capacity",
        ))?;
    if block_size < 512
        || !block_size.is_power_of_two()
        || block_size as usize > bot.max_transfer_bytes()
        || blocks.checked_mul(u64::from(block_size)).is_none()
    {
        return Err(BlockError::Unsupported(
            "USB storage block size or capacity",
        ));
    }
    bot.read_before_mode_sense(lun, block_size as usize)?;
    let read_only = match bot.write_protect_page() {
        None => false,
        Some(page) => {
            match bot.command(
                lun,
                &scsi::mode_sense(page),
                Data::In(scsi::MODE_SENSE_BYTES),
            ) {
                Ok(mode) => scsi::write_protected(&mode)
                    .ok_or(BlockError::Unsupported("short MODE SENSE data"))?,
                // MODE SENSE is optional only when the unit reports an
                // unsupported CDB. USB/transport errors cannot default to writable.
                Err(BlockError::Sense(sense)) if sense.key == Sense::ILLEGAL_REQUEST => false,
                Err(error) => return Err(error),
            }
        }
    };
    let identity = Identity {
        transport: Transport::Usb,
        location: location.into(),
        model: format!("{} {}", text(&inquiry[8..16]), text(&inquiry[16..32]))
            .trim()
            .into(),
        serial: if lun == T::Lun::default() {
            serial.into()
        } else {
            format!("{serial}:lun{lun}")
        },
        blocks,
        block_size,
        read_only,
        // Read with the Caching page once the disk is published.
        features: Features::default(),
    };
    Ok((identity, inquiry[1] & 0x80 != 0))
}

fn capacity16<T: Scsi>(bot: &mut T, lun: T::Lun) -> Result<(u64, u32), BlockError> {
    let mut cdb = [0; 16];
    cdb[0] = scsi::SERVICE_ACTION_IN_16;
    cdb[1] = scsi::READ_CAPACITY_16;
    cdb[13] = 32;
    let bytes = bot.command(lun, &cdb, Data::In(32))?;
    if bytes.len() < 12 {
        return Err(BlockError::Unsupported("short READ CAPACITY (16) data"));
    }
    let blocks = u64::from_be_bytes(bytes[..8].try_into().unwrap())
        .checked_add(1)
        .ok_or(BlockError::Unsupported("USB storage capacity overflow"))?;
    Ok((blocks, u32::from_be_bytes(bytes[8..12].try_into().unwrap())))
}

impl<T: Scsi> Disk<T> {
    /// Execute a SCSI CDB on this unit, including same-LUN auto-sense.
    pub fn command(&mut self, cdb: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
        self.with_media(|bot, lun| bot.command(lun, cdb, data))
    }
    fn with_media<R>(
        &self,
        run: impl FnOnce(&mut T, T::Lun) -> Result<R, BlockError>,
    ) -> Result<R, BlockError> {
        self.check_media()?;
        let result = run(&mut self.bot.borrow_mut(), self.lun);
        if matches!(&result, Err(BlockError::Sense(sense)) if media::changed(sense, self.removable))
        {
            self.media.set(false);
        }
        result
    }
    /// sd's I/O on this unit: `scsi_check_sense` retries NEEDS_RETRY sense
    /// up to `SD_MAX_RETRIES` times, and `scsi_io_completion_action`
    /// completes the rest: a UNIT ATTENTION is retried at once on a fixed
    /// unit and is a media change on a removable one, a transient NOT READY
    /// is requeued after its delay, and nothing is retried once the command
    /// has run past sd's runtime limit.
    fn io(&mut self, cdb: &[u8], mut data: BlockData<'_>) -> Result<(), BlockError> {
        let started = self.bot.borrow().now_us();
        let mut retries = 0;
        loop {
            let result = self.with_media(|bot, lun| match &mut data {
                BlockData::Read(output) => bot.read_into(lun, cdb, output),
                BlockData::Write(bytes) => bot.command(lun, cdb, Data::Out(bytes)).map(|_| ()),
                BlockData::None => bot.command(lun, cdb, Data::None).map(|_| ()),
            });
            let sense = match result {
                Err(BlockError::Sense(sense)) => sense,
                result => return result,
            };
            let failed = Err(BlockError::Sense(sense));
            let exceeded = self.bot.borrow().now_us().wrapping_sub(started) > SD_RUNTIME_US;
            if sense.needs_retry() && retries < SD_MAX_RETRIES && !exceeded {
                retries += 1;
                continue;
            }
            match sense.completion(self.removable) {
                // sd_done counts the whole buffer good; only data the
                // transport moved in full is returned.
                SenseCompletion::Done => {
                    return match data {
                        BlockData::Read(output) => match self.bot.borrow_mut().take_failed_data() {
                            Some(bytes) if bytes.len() == output.len() => copy_read(output, &bytes),
                            _ => failed,
                        },
                        _ => Ok(()),
                    };
                }
                SenseCompletion::Retry(_) if exceeded => return failed,
                SenseCompletion::Retry(delay) => {
                    if delay != 0 {
                        self.bot.borrow_mut().delay(delay)?;
                    }
                }
                SenseCompletion::MediaChanged | SenseCompletion::Fail => return failed,
            }
        }
    }
    fn check_media(&self) -> Result<(), BlockError> {
        if self.media.get() {
            Ok(())
        } else {
            Err(BlockError::Unsupported(
                "USB storage media publication is stale",
            ))
        }
    }
    /// The bytes of the next command of a transfer of `remaining` bytes at
    /// `lba`: at most the transport's bound, and the last sector alone
    /// where the device needs it so.
    fn command_bytes(&self, lba: u64, remaining: usize, read: bool) -> usize {
        let size = self.identity.block_size as usize;
        let transport = self.bot.borrow();
        let legacy = transport.max_transfer_bytes();
        let maximum = if read && let Some(limit) = self.identity.features.max_read_transfer {
            limit
                .get()
                .min(transport.read_command_ceiling().max(legacy))
        } else {
            legacy
        } / size
            * size;
        let mut length = remaining.min(maximum);
        if transport.separate_last_sector()
            && lba + (length / size) as u64 == self.identity.blocks
            && length > size
        {
            length -= size;
        }
        length
    }

    /// Begin the running command's next SCSI command, or settle its result
    /// once it needs none or one cannot begin.
    fn step(&mut self) {
        let Some(running) = self.running.as_ref() else {
            return;
        };
        if running.begun.is_some() || running.result.is_some() {
            return;
        }
        let size = self.identity.block_size as usize;
        let total = Self::total(&running.command);
        let (cdb, length) = match &running.command {
            Command::Read { lba, .. } | Command::Write { lba, .. } if running.done < total => {
                let at = lba + (running.done / size) as u64;
                let (write, fua) = match &running.command {
                    Command::Write { fua, .. } => (true, *fua),
                    _ => (false, false),
                };
                let length = self.command_bytes(at, total - running.done, !write);
                (
                    scsi::read_write(write, fua, at, (length / size) as u32),
                    length,
                )
            }
            Command::Read { .. } | Command::Write { .. } => {
                let running = self.running.as_mut().unwrap();
                let read = matches!(running.command, Command::Read { .. });
                let data = core::mem::take(&mut running.data);
                running.result = Some(Ok(read.then_some(data)));
                return;
            }
            Command::Flush if running.done == 0 => (scsi::Cdb::Ten(scsi::synchronize_cache()), 0),
            Command::Flush => {
                self.running.as_mut().unwrap().result = Some(Ok(None));
                return;
            }
            Command::Discard { .. } | Command::WriteZeroes { .. } => {
                self.running.as_mut().unwrap().result =
                    Some(Err(BlockError::Unsupported("USB storage takes no discard")));
                return;
            }
        };
        let begun = {
            let running = self.running.as_ref().unwrap();
            let mut bot = self.bot.borrow_mut();
            let cdb = cdb.as_ref();
            match &running.command {
                Command::Write { data, .. } => {
                    let part = &data[running.done..running.done + length];
                    bot.begin(self.lun, cdb, Data::Out(part))
                }
                Command::Read { .. } => bot.begin(self.lun, cdb, Data::In(length)),
                _ => bot.begin(self.lun, cdb, Data::None),
            }
        };
        let running = self.running.as_mut().unwrap();
        match begun {
            Some(Ok(())) => running.begun = Some(length),
            Some(Err(error)) => running.result = Some(Err(error)),
            None => {
                running.result = Some(Err(BlockError::Unsupported(
                    "the USB storage transport runs no queued commands",
                )));
            }
        }
    }

    /// A transfer's bytes.
    fn total(command: &Command) -> usize {
        match command {
            Command::Read { bytes, .. } => *bytes,
            Command::Write { data, .. } => data.len(),
            _ => 0,
        }
    }

    /// Write `data` at `lba` as the transport takes it, with `fua`.
    fn write_with(&mut self, lba: u64, data: &[u8], fua: bool) -> Result<(), BlockError> {
        self.check_media()?;
        self.identity.check_write(lba, data.len())?;
        let size = self.identity.block_size as usize;
        let (mut lba, mut remaining) = (lba, data);
        while !remaining.is_empty() {
            let length = self.command_bytes(lba, remaining.len(), false);
            let (chunk, rest) = remaining.split_at(length);
            remaining = rest;
            let blocks = (chunk.len() / size) as u32;
            let cdb = scsi::read_write(true, fua, lba, blocks);
            self.io(cdb.as_ref(), BlockData::Write(chunk))?;
            lba += u64::from(blocks);
        }
        Ok(())
    }
}
impl<T: Scsi> BlockDevice for Disk<T> {
    fn identity(&self) -> &Identity {
        &self.identity
    }
    fn signals(&self) -> bool {
        self.bot
            .try_borrow()
            .is_ok_and(|transport| transport.signals())
    }
    fn posted(&self) -> bool {
        self.running.is_some()
            && self
                .bot
                .try_borrow()
                .is_ok_and(|transport| transport.posted())
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.check_media()?;
        self.identity.check(lba, buffer.len())?;
        let size = self.identity.block_size as usize;
        let (mut lba, mut remaining) = (lba, buffer);
        while !remaining.is_empty() {
            let length = self.command_bytes(lba, remaining.len(), true);
            let (chunk, rest) = remaining.split_at_mut(length);
            remaining = rest;
            let blocks = (chunk.len() / size) as u32;
            let cdb = scsi::read_write(false, false, lba, blocks);
            self.io(cdb.as_ref(), BlockData::Read(chunk))?;
            lba += u64::from(blocks);
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_with(lba, data, false)
    }

    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.write_with(lba, data, true)
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.check_media()?;
        self.io(&scsi::synchronize_cache(), BlockData::None)
    }

    /// A transport that begins commands without waiting runs one block
    /// command at a time.
    fn depth(&self) -> usize {
        usize::from(self.bot.borrow().queued())
    }

    fn room(&self) -> bool {
        self.running.is_none() && !self.bot.borrow_mut().busy()
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        self.check_media()?;
        let command = command.within(&self.identity, 0)?;
        if !self.room() {
            return Err(BlockError::Protocol(
                "a USB storage unit runs one command at a time",
            ));
        }
        let tag = self.next_tag;
        self.next_tag = tag.wrapping_add(1);
        self.running = Some(Running {
            tag,
            command,
            done: 0,
            begun: None,
            data: Vec::new(),
            result: None,
        });
        self.step();
        Ok(tag)
    }

    fn finish(&mut self, tag: u32) -> Option<Finished> {
        if self.running.as_ref()?.tag != tag {
            return None;
        }
        if let Some(length) = self.running.as_ref()?.begun {
            let ended = self.bot.borrow_mut().end()?;
            let running = self.running.as_mut()?;
            running.begun = None;
            match ended {
                Err(error) => {
                    if matches!(&error, BlockError::Sense(sense)
                        if media::changed(sense, self.removable))
                    {
                        self.media.set(false);
                    }
                    running.result = Some(Err(error));
                }
                Ok(bytes) => {
                    if let Command::Read { .. } = running.command {
                        if bytes.len() != length {
                            running.result =
                                Some(Err(BlockError::Protocol("short USB storage read")));
                        } else {
                            running.data.extend_from_slice(&bytes);
                        }
                    }
                    // A flush counts as one byte once complete.
                    running.done += length.max(1);
                }
            }
            self.step();
        }
        let result = self.running.as_mut()?.result.take()?;
        self.running = None;
        Some(result)
    }
}

impl UsbDisk {
    /// Explicit recovery for an exclusively owned root-attached storage device.
    /// All old disk aliases remain quarantined, including on failure. Returned
    /// disks carry fresh INQUIRY/capacity/protection state; no CDB is replayed.
    /// Use [`Self::reset_and_rebind_retained`] to keep an initially empty fresh
    /// interface available for rescan. This does not install a hot-plug scheduler.
    pub fn reset_and_rebind(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<Vec<UsbDisk>, BlockError> {
        self.reset_and_rebind_retained(dma, failures)
            .map(|(_, disks)| disks)
    }

    /// Execute a SCSI CDB on this unit, including same-LUN auto-sense.
    pub fn command(&mut self, cdb: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.command(cdb, data),
            UsbTransport::EhciBot(disk) => disk.command(cdb, data),
            UsbTransport::Uas(disk) => disk.command(cdb, data),
        }
    }
}
impl BlockDevice for UsbDisk {
    fn identity(&self) -> &Identity {
        match &self.disk {
            UsbTransport::Bot(disk) => disk.identity(),
            UsbTransport::EhciBot(disk) => disk.identity(),
            UsbTransport::Uas(disk) => disk.identity(),
        }
    }
    fn signals(&self) -> bool {
        match &self.disk {
            UsbTransport::Bot(disk) => disk.signals(),
            UsbTransport::EhciBot(disk) => disk.signals(),
            UsbTransport::Uas(disk) => disk.signals(),
        }
    }
    fn posted(&self) -> bool {
        match &self.disk {
            UsbTransport::Bot(disk) => disk.posted(),
            UsbTransport::EhciBot(disk) => disk.posted(),
            UsbTransport::Uas(disk) => disk.posted(),
        }
    }
    fn read(&mut self, lba: u64, bytes: &mut [u8]) -> Result<(), BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.read(lba, bytes),
            UsbTransport::EhciBot(disk) => disk.read(lba, bytes),
            UsbTransport::Uas(disk) => disk.read(lba, bytes),
        }
    }
    fn write(&mut self, lba: u64, bytes: &[u8]) -> Result<(), BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.write(lba, bytes),
            UsbTransport::EhciBot(disk) => disk.write(lba, bytes),
            UsbTransport::Uas(disk) => disk.write(lba, bytes),
        }
    }
    fn write_fua(&mut self, lba: u64, bytes: &[u8]) -> Result<(), BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.write_fua(lba, bytes),
            UsbTransport::EhciBot(disk) => disk.write_fua(lba, bytes),
            UsbTransport::Uas(disk) => disk.write_fua(lba, bytes),
        }
    }
    fn flush_cache(&mut self) -> Result<(), BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.flush_cache(),
            UsbTransport::EhciBot(disk) => disk.flush_cache(),
            UsbTransport::Uas(disk) => disk.flush_cache(),
        }
    }
    fn depth(&self) -> usize {
        match &self.disk {
            UsbTransport::Bot(disk) => disk.depth(),
            UsbTransport::EhciBot(disk) => disk.depth(),
            UsbTransport::Uas(disk) => disk.depth(),
        }
    }
    fn room(&self) -> bool {
        match &self.disk {
            UsbTransport::Bot(disk) => disk.room(),
            UsbTransport::EhciBot(disk) => disk.room(),
            UsbTransport::Uas(disk) => disk.room(),
        }
    }
    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.start(command),
            UsbTransport::EhciBot(disk) => disk.start(command),
            UsbTransport::Uas(disk) => disk.start(command),
        }
    }
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        match &mut self.disk {
            UsbTransport::Bot(disk) => disk.finish(tag),
            UsbTransport::EhciBot(disk) => disk.finish(tag),
            UsbTransport::Uas(disk) => disk.finish(tag),
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use alloc::collections::VecDeque;

    /// A unit answering each command with the next sense, or success; each
    /// command takes a second.
    struct Unit {
        senses: VecDeque<Option<Sense>>,
        commands: usize,
        now: Cell<u64>,
        delays: RefCell<Vec<u64>>,
        failed: Option<Vec<u8>>,
        minimum_error: Option<BlockError>,
    }
    impl Scsi for Unit {
        type Lun = u8;
        fn command(&mut self, _: u8, _: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
            self.commands += 1;
            self.now.set(self.now.get() + 1_000_000);
            match self.senses.pop_front().flatten() {
                // A failed read still moves its data.
                Some(sense) => {
                    if let Data::In(bytes) = data {
                        self.failed = Some(vec![0x5a; bytes]);
                    }
                    Err(BlockError::Sense(sense))
                }
                None => Ok(match data {
                    Data::In(bytes) => vec![0; bytes],
                    _ => Vec::new(),
                }),
            }
        }
        fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
            if let Some(error) = &self.minimum_error {
                return Err(error.clone());
            }
            self.now.set(self.now.get() + micros);
            self.delays.borrow_mut().push(micros);
            Ok(())
        }
        fn now_us(&self) -> u64 {
            self.now.get()
        }
        fn poisoned(&self) -> bool {
            false
        }
        fn take_failed_data(&mut self) -> Option<Vec<u8>> {
            self.failed.take()
        }
    }

    fn disk(removable: bool, senses: &[Option<Sense>]) -> Disk<Unit> {
        Disk {
            bot: Rc::new(RefCell::new(Unit {
                senses: senses.iter().copied().collect(),
                commands: 0,
                now: Cell::new(0),
                delays: RefCell::new(Vec::new()),
                failed: None,
                minimum_error: None,
            })),
            lun: 0,
            identity: Identity {
                transport: Transport::Usb,
                location: String::new(),
                model: String::new(),
                serial: String::new(),
                blocks: 8,
                block_size: 512,
                read_only: false,
                features: Features {
                    write_cache: true,
                    ..Features::default()
                },
            },
            media: Rc::new(Cell::new(true)),
            removable,
            running: None,
            next_tag: 0,
        }
    }

    #[test]
    fn retry_quiet_failure_returns_original_cause_before_retrying() {
        let becoming = Some(Sense {
            key: Sense::NOT_READY,
            asc: Sense::NOT_READY_CAUSE,
            ascq: 0x01,
        });
        let mut disk = disk(false, &[becoming; 8]);
        disk.bot.borrow_mut().minimum_error = Some(BlockError::PciPower(
            crate::bus::pci::power::Error::Interrupted,
        ));
        let mut bytes = [0x7a; 512];
        assert_eq!(
            disk.read(0, &mut bytes),
            Err(BlockError::PciPower(
                crate::bus::pci::power::Error::Interrupted
            ))
        );
        assert_eq!(disk.bot.borrow().commands, 6);
        assert_eq!(bytes, [0x7a; 512]);
        assert!(disk.bot.borrow().delays.borrow().is_empty());
    }
    #[test]
    fn io_sense_follows_scsi_check_sense_and_sd_completion() {
        let sense = |key, asc, ascq| Some(Sense { key, asc, ascq });
        let reset = sense(Sense::UNIT_ATTENTION, Sense::RESET_OCCURRED, 0);
        // A fixed unit retries a UNIT ATTENTION and keeps its medium.
        let mut fixed = disk(false, &[reset]);
        assert_eq!(fixed.flush_cache(), Ok(()));
        assert_eq!(fixed.bot.borrow().commands, 2);
        assert!(fixed.media.get());
        // A removable one reports a media change and refuses access.
        let mut removable = disk(true, &[reset]);
        assert!(matches!(removable.flush_cache(), Err(BlockError::Sense(_))));
        assert!(!removable.media.get());
        // Becoming ready: five NEEDS_RETRY retries, then a delayed requeue.
        let becoming = sense(Sense::NOT_READY, Sense::NOT_READY_CAUSE, 0x01);
        let mut unit = disk(false, &[becoming; 7]);
        assert_eq!(unit.read(0, &mut [0; 512]), Ok(()));
        assert_eq!(unit.bot.borrow().commands, 8);
        assert_eq!(*unit.bot.borrow().delays.borrow(), [9_000, 9_000]);
        // An aborted command is retried five times, then fails.
        let aborted = sense(Sense::ABORTED_COMMAND, 0, 0);
        let mut unit = disk(false, &[aborted; 8]);
        assert!(unit.write(0, &[0; 512]).is_err());
        assert_eq!(unit.bot.borrow().commands, 6);
        // A recovered error completes a write, and a read with its data.
        let recovered = sense(Sense::RECOVERED_ERROR, 0x17, 0x01);
        assert_eq!(disk(false, &[recovered]).write(0, &[0; 512]), Ok(()));
        let mut block = [0; 512];
        assert_eq!(disk(false, &[recovered]).read(0, &mut block), Ok(()));
        assert_eq!(block, [0x5a; 512]);
        // Requeues stop once the command has run for six timeouts.
        let mut unit = disk(false, &[reset; 400]);
        assert!(unit.flush_cache().is_err());
        assert_eq!(unit.bot.borrow().commands, 181);
    }
}
