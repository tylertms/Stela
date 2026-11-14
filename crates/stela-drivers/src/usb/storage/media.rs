//! Class lifetime and permanent per-publication media validity.
// SPDX-License-Identifier: GPL-2.0-only
// Linux sd.c::sd_check_events/media_not_present/sd_revalidate_disk and
// scsi_lib.c::scsi_io_completion_action. See NOTICE.
use super::*;

pub(super) fn no_media(sense: &Sense) -> bool {
    matches!(sense.key, Sense::NOT_READY | Sense::UNIT_ATTENTION)
        && sense.asc == Sense::MEDIUM_NOT_PRESENT
}
/// A change sd acts on: an absent medium or unit, and any UNIT ATTENTION
/// on a removable unit (`scsi_io_completion_action`'s `sdev->changed`).
pub(super) fn changed(sense: &Sense, removable: bool) -> bool {
    sense.key == Sense::UNIT_ATTENTION && removable
        || no_media(sense)
        || sense.key == Sense::ILLEGAL_REQUEST && sense.asc == Sense::UNSUPPORTED_UNIT
}

/// Port resets an owner may run on its own in [`RESET_WINDOW_US`]; past
/// that the device is left failed until an explicit rescan rebinds it.
const RESET_LIMIT: usize = 3;
const RESET_WINDOW_US: u64 = 60_000_000;

pub(super) struct Owner<T: Scsi> {
    pub transport: T,
    prefix: String,
    serial: String,
    units: Vec<Unit<T::Lun>>,
    /// Provisional publications: unit, validity and removability.
    pending: Vec<(T::Lun, Rc<Cell<bool>>, bool)>,
    /// The device was reset since its last UNIT ATTENTION: the next one,
    /// unless it reports a changed medium, is retried once, as Linux
    /// `scsi_check_sense` retries with `expecting_cc_ua`.
    expecting_reset: bool,
    /// When this owner's recent port resets ran, by the transport's clock.
    resets: Vec<u64>,
    /// A port reset rebound the transport during the current command.
    rebound: bool,
    /// Port resets that rebound the transport.
    rebinds: u64,
    /// The command begun without waiting ([`Scsi::begin`]), for its retries.
    begun: Option<Begun<T::Lun>>,
    /// The begun command's result, taken while a waiting command ran first
    /// ([`Self::run`]), for the next [`Scsi::end`].
    ended: Option<Result<Vec<u8>, BlockError>>,
}
/// A begun command and what [`Owner::run`]'s retries of it still allow.
struct Begun<L> {
    lun: L,
    cdb: Vec<u8>,
    data: Retry,
    generation: u64,
    /// A UNIT ATTENTION after a reset may still retry this attempt.
    attention: bool,
    /// A command a port reset ended may still run once more.
    rebound: bool,
}
/// A begun command's data, kept for its retry.
enum Retry {
    None,
    In(usize),
    Out(Vec<u8>),
}
impl Retry {
    fn new(data: &Data<'_>) -> Self {
        match data {
            Data::None => Self::None,
            Data::In(length) => Self::In(*length),
            Data::Out(bytes) => Self::Out(bytes.to_vec()),
        }
    }
    fn data(&self) -> Data<'_> {
        match self {
            Self::None => Data::None,
            Self::In(length) => Data::In(*length),
            Self::Out(bytes) => Data::Out(bytes),
        }
    }
}
struct Unit<L> {
    lun: L,
    identity: Identity,
    valid: Rc<Cell<bool>>,
    removable: bool,
}
impl<T: Scsi> Owner<T> {
    pub fn new(transport: T, prefix: String, serial: String) -> Self {
        Self {
            transport,
            prefix,
            serial,
            units: Vec::new(),
            pending: Vec::new(),
            expecting_reset: false,
            resets: Vec::new(),
            rebound: false,
            rebinds: 0,
            begun: None,
            ended: None,
        }
    }
    pub fn invalidate(&mut self) {
        for (_, valid, _) in &self.pending {
            valid.set(false);
        }
        for unit in &self.units {
            unit.valid.set(false);
        }
    }
    /// Whether `lun` is published and still valid.
    fn valid(&self, lun: T::Lun) -> bool {
        self.units
            .iter()
            .any(|unit| unit.lun == lun && unit.valid.get())
    }
    /// Whether `lun` is removable; a unit not yet published is assumed so.
    fn removable(&self, lun: T::Lun) -> bool {
        let units = self.units.iter().map(|unit| (unit.lun, unit.removable));
        let pending = self
            .pending
            .iter()
            .map(|(unit, _, removable)| (*unit, *removable));
        units
            .chain(pending)
            .find(|(unit, _)| *unit == lun)
            .is_none_or(|(_, removable)| removable)
    }
    fn changed(&mut self, lun: T::Lun) {
        for (unit, valid, _) in &self.pending {
            if *unit == lun {
                valid.set(false);
            }
        }
        for unit in &self.units {
            if unit.lun == lun {
                unit.valid.set(false);
            }
        }
    }
    fn reconcile(&mut self, disks: &mut [Disk<Self>]) {
        for old in &self.units {
            if let Some(disk) = disks.iter_mut().find(|disk| disk.lun == old.lun) {
                if old.valid.get() && disk.identity == old.identity {
                    disk.media = old.valid.clone();
                } else {
                    old.valid.set(false);
                }
            } else {
                old.valid.set(false);
            }
        }
        self.units = disks
            .iter()
            .map(|disk| Unit {
                lun: disk.lun,
                identity: disk.identity.clone(),
                valid: disk.media.clone(),
                removable: disk.removable,
            })
            .collect();
    }

    fn run<R>(
        &mut self,
        lun: T::Lun,
        cdb: &[u8],
        mut command: impl FnMut(&mut T) -> Result<R, BlockError>,
    ) -> Result<R, BlockError> {
        self.end_first();
        self.rebound = false;
        let mut result = self.attempt(lun, cdb, &mut command);
        // Linux's SCSI error handler retries the command a reset ended.
        if result.is_err() && core::mem::take(&mut self.rebound) && self.valid(lun) {
            result = self.attempt(lun, cdb, &mut command);
            self.rebound = false;
        }
        result
    }

    /// One run of `command` and what its end means for the units.
    fn attempt<R>(
        &mut self,
        lun: T::Lun,
        cdb: &[u8],
        command: &mut impl FnMut(&mut T) -> Result<R, BlockError>,
    ) -> Result<R, BlockError> {
        let generation = self.transport.generation();
        let mut result = command(&mut self.transport);
        if let Err(BlockError::Sense(sense)) = &result
            && sense.key == Sense::UNIT_ATTENTION
            && core::mem::take(&mut self.expecting_reset)
            && !(sense.asc == Sense::MEDIUM_CHANGED && sense.ascq == 0)
        {
            result = command(&mut self.transport);
        }
        // A removable unit's UA identifies an invalidated media view, even
        // when a later TUR consumes it and the capacity is unchanged.
        let removable = self.removable(lun);
        if matches!(&result, Err(BlockError::Sense(sense)) if changed(sense, removable))
            || result.is_ok()
                && cdb.first() == Some(&scsi::START_STOP_UNIT)
                && cdb
                    .get(4)
                    .is_some_and(|flags| flags & scsi::LOAD_EJECT != 0)
        {
            self.changed(lun);
        }
        self.settle(generation, &result);
        result
    }

    /// The transport runs one command at a time: a begun one ends before a
    /// waiting one starts, its result kept for the disk that began it.
    fn end_first(&mut self) {
        while self.begun.is_some() && self.ended.is_none() {
            if let Some(result) = self.end_begun() {
                self.ended = Some(result);
            } else {
                crate::waiting();
                core::hint::spin_loop();
            }
        }
    }

    /// Begin attempt `begun` of a command; its failure ends it as
    /// [`Self::end_begun`] would. `None` from a transport that runs commands only to completion.
    fn begin_attempt(&mut self, begun: Begun<T::Lun>) -> Option<Result<(), BlockError>> {
        self.rebound = false;
        let started = self
            .transport
            .begin(begun.lun, &begun.cdb, begun.data.data())?;
        self.begun = Some(begun);
        Some(match started {
            Ok(()) => Ok(()),
            // A retry may have begun meanwhile.
            Err(error) => self
                .ended_with(Err(error))
                .map_or(Ok(()), |result| result.map(drop)),
        })
    }

    /// The begun command's result once it has one, after the retries
    /// [`Self::run`] makes of a waiting command: once for a UNIT ATTENTION
    /// a reset raised, and once more after a port reset rebound the
    /// transport. A retry begins at once; its result comes at a later look.
    fn end_begun(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        if self.begun.is_none() {
            return self.transport.end();
        }
        let result = self.transport.end()?;
        self.ended_with(result)
    }

    fn ended_with(
        &mut self,
        result: Result<Vec<u8>, BlockError>,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        let mut begun = self.begun.take()?;
        if let Err(BlockError::Sense(sense)) = &result
            && sense.key == Sense::UNIT_ATTENTION
            && core::mem::take(&mut begun.attention)
            && core::mem::take(&mut self.expecting_reset)
            && !(sense.asc == Sense::MEDIUM_CHANGED && sense.ascq == 0)
        {
            return self.retry(begun);
        }
        let removable = self.removable(begun.lun);
        if matches!(&result, Err(BlockError::Sense(sense)) if changed(sense, removable))
            || result.is_ok()
                && begun.cdb.first() == Some(&scsi::START_STOP_UNIT)
                && begun
                    .cdb
                    .get(4)
                    .is_some_and(|flags| flags & scsi::LOAD_EJECT != 0)
        {
            self.changed(begun.lun);
        }
        self.settle(begun.generation, &result);
        if result.is_err()
            && core::mem::take(&mut self.rebound)
            && core::mem::take(&mut begun.rebound)
            && self.valid(begun.lun)
        {
            begun.attention = true;
            begun.generation = self.transport.generation();
            return self.retry(begun);
        }
        Some(result)
    }

    /// Begin `begun` again: `None` while it runs, its failure to begin as
    /// its result.
    fn retry(&mut self, begun: Begun<T::Lun>) -> Option<Result<Vec<u8>, BlockError>> {
        match self.begin_attempt(begun)? {
            Ok(()) => None,
            Err(error) => Some(Err(error)),
        }
    }

    /// What a command's end means for every unit: a class reset is expected
    /// to raise a UNIT ATTENTION, a poisoned transport escalates to a port
    /// reset, and a changed LUN inventory ends every publication.
    fn settle<R>(&mut self, generation: u64, result: &Result<R, BlockError>) {
        if generation != self.transport.generation() {
            self.expecting_reset = true;
        }
        if self.transport.poisoned() {
            self.reset_device();
        }
        if matches!(
            result,
            Err(BlockError::Sense(Sense {
                key: Sense::UNIT_ATTENTION,
                asc: Sense::CONDITIONS_CHANGED,
                ascq: Sense::LUNS_CHANGED,
            }))
        ) {
            self.invalidate();
        }
    }

    /// Linux usb-storage's ladder after a failed class reset: reset the
    /// device's port, rebind a fresh transport to the same device, and keep
    /// every unit that probes as it was published, so the disks published on
    /// it go on working; [`Owner::run`] then retries the failed command once. At most
    /// [`RESET_LIMIT`] resets run in [`RESET_WINDOW_US`]; past that, or when
    /// the reset fails, every unit fails until an explicit rescan.
    fn reset_device(&mut self) {
        let now = self.transport.now_us();
        self.resets
            .retain(|at| now.wrapping_sub(*at) < RESET_WINDOW_US);
        if self.resets.len() >= RESET_LIMIT {
            self.invalidate();
            return;
        }
        // A root device's reset reuses its old binding's rings and
        // buffers. One behind a hub needs new rings, which only
        // [`UsbStorage::service`] with the drivers' allocator supplies; that
        // reset disables the old slot and resets the port before it
        // allocates, so begun here it would lose the device for good. The
        // transport stays poisoned for the service's reset.
        if self.transport.reset_needs_allocation() {
            self.invalidate();
            return;
        }
        // SAFETY: the empty range hands out no memory.
        let mut none = unsafe { crate::bus::dma::RegionAllocator::new(0..0) };
        let Some(replaced) = self.transport.reset_device(&mut none) else {
            self.invalidate();
            return;
        };
        self.resets.push(now);
        match replaced {
            Ok(transport) => {
                self.transport = transport;
                self.expecting_reset = true;
                self.revalidate();
                self.rebound = true;
                self.rebinds += 1;
            }
            Err(_) => self.invalidate(),
        }
    }

    /// After a device reset, keep each published unit whose probe gives the
    /// identity it was published with, and end the others.
    fn revalidate(&mut self) {
        for (_, valid, _) in &self.pending {
            valid.set(false);
        }
        let serial = self.serial.clone();
        for index in 0..self.units.len() {
            let (lun, location) = (
                self.units[index].lun,
                self.units[index].identity.location.clone(),
            );
            let probed = probe(&mut self.transport, lun, &location, &serial);
            let unit = &self.units[index];
            let same = probed.is_ok_and(|(identity, removable)| {
                removable == unit.removable
                    && Identity {
                        features: unit.identity.features,
                        ..identity
                    } == unit.identity
            });
            if !same || self.transport.poisoned() {
                unit.valid.set(false);
            }
        }
    }
}
impl<T: Scsi> Scsi for Owner<T> {
    type Lun = T::Lun;
    fn take_failed_data(&mut self) -> Option<Vec<u8>> {
        self.transport.take_failed_data()
    }
    fn command(
        &mut self,
        lun: Self::Lun,
        cdb: &[u8],
        data: Data<'_>,
    ) -> Result<Vec<u8>, BlockError> {
        self.run(lun, cdb, |transport| transport.command(lun, cdb, data))
    }
    fn read_into(
        &mut self,
        lun: Self::Lun,
        cdb: &[u8],
        output: &mut [u8],
    ) -> Result<(), BlockError> {
        self.run(lun, cdb, |transport| {
            transport.read_into(lun, cdb, &mut *output)
        })
    }
    fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
        self.transport.delay(micros)
    }
    fn now_us(&self) -> u64 {
        self.transport.now_us()
    }
    fn poisoned(&self) -> bool {
        self.transport.poisoned()
    }
    fn signals(&self) -> bool {
        self.transport.signals()
    }
    fn posted(&self) -> bool {
        self.transport.posted()
    }
    fn write_protect_page(&self) -> Option<u8> {
        self.transport.write_protect_page()
    }
    fn caching(&self) -> Caching {
        self.transport.caching()
    }
    fn report_luns_permitted(&self) -> bool {
        self.transport.report_luns_permitted()
    }
    fn generation(&self) -> u64 {
        self.transport.generation()
    }
    fn max_transfer_bytes(&self) -> usize {
        self.transport.max_transfer_bytes()
    }
    fn read_command_ceiling(&self) -> usize {
        self.transport.read_command_ceiling()
    }
    fn capacity16_first(&self) -> bool {
        self.transport.capacity16_first()
    }
    fn adjust_capacity(&self, blocks: u64) -> Option<u64> {
        self.transport.adjust_capacity(blocks)
    }
    fn separate_last_sector(&self) -> bool {
        self.transport.separate_last_sector()
    }
    fn read_before_mode_sense(
        &mut self,
        lun: Self::Lun,
        block_size: usize,
    ) -> Result<(), BlockError> {
        self.end_first();
        let generation = self.transport.generation();
        let result = self.transport.read_before_mode_sense(lun, block_size);
        let removable = self.removable(lun);
        if matches!(&result, Err(BlockError::Sense(sense)) if changed(sense, removable)) {
            self.changed(lun);
        }
        self.settle(generation, &result);
        result
    }
    fn media_publication(&mut self, lun: Self::Lun, valid: Rc<Cell<bool>>, removable: bool) {
        self.pending.push((lun, valid, removable));
    }
    fn max_lun(&self) -> u8 {
        self.transport.max_lun()
    }
    fn queued(&self) -> bool {
        self.transport.queued()
    }
    fn begin(
        &mut self,
        lun: Self::Lun,
        cdb: &[u8],
        data: Data<'_>,
    ) -> Option<Result<(), BlockError>> {
        if self.begun.is_some() || self.ended.is_some() {
            return Some(Err(BlockError::Protocol(
                "a USB storage transport runs one command at a time",
            )));
        }
        self.begin_attempt(Begun {
            lun,
            cdb: cdb.to_vec(),
            data: Retry::new(&data),
            generation: self.transport.generation(),
            attention: true,
            rebound: true,
        })
    }
    fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        self.ended.take().or_else(|| self.end_begun())
    }
    fn busy(&mut self) -> bool {
        self.begun.is_some() || self.ended.is_some() || self.transport.busy()
    }
}

fn finish<T: Scsi>(
    owner: &Rc<RefCell<Owner<T>>>,
    result: Result<Vec<Disk<Owner<T>>>, BlockError>,
) -> Result<Vec<Disk<Owner<T>>>, BlockError> {
    match result {
        Ok(mut disks) => {
            let mut owner = owner.borrow_mut();
            owner.reconcile(&mut disks);
            owner.pending.clear();
            Ok(disks)
        }
        Err(error) => {
            let mut owner = owner.borrow_mut();
            owner.invalidate();
            owner.pending.clear();
            Err(error)
        }
    }
}
fn scan<T: Scsi>(
    owner: Rc<RefCell<Owner<T>>>,
    mut discover: impl FnMut() -> Result<Vec<Disk<Owner<T>>>, BlockError>,
) -> Result<Vec<Disk<Owner<T>>>, BlockError> {
    for _ in 0..3 {
        let rebinds = owner.borrow().rebinds;
        let result = discover();
        if result.is_err() {
            return finish(&owner, result);
        }
        // A port reset during discovery ended the commands it interrupted,
        // so their units are probed again, as a changed view is.
        let unstable = owner.borrow().rebinds != rebinds
            || owner
                .borrow()
                .pending
                .iter()
                .any(|(_, valid, _)| !valid.get());
        if !unstable {
            return finish(&owner, result);
        }
        // A later command invalidated an already-probed view. Discard every
        // provisional disk and repeat only inventory/readiness/metadata probes.
        let mut owner = owner.borrow_mut();
        for (_, valid, _) in &owner.pending {
            valid.set(false);
        }
        owner.pending.clear();
    }
    finish(
        &owner,
        Err(BlockError::Unsupported(
            "USB media changed repeatedly during rescan",
        )),
    )
}
pub(super) fn scan_bot<T: Scsi<Lun = u8>>(
    owner: Rc<RefCell<Owner<T>>>,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<Vec<Disk<Owner<T>>>, BlockError> {
    let (prefix, serial) = {
        let o = owner.borrow();
        (o.prefix.clone(), o.serial.clone())
    };
    scan(owner.clone(), || {
        discover_units(owner.clone(), &prefix, &serial, failures)
    })
}
pub(super) fn scan_uas<T: Scsi<Lun = uas::Lun>>(
    owner: Rc<RefCell<Owner<T>>>,
    failures: &mut Vec<(String, BlockError)>,
) -> Result<Vec<Disk<Owner<T>>>, BlockError> {
    let (prefix, serial) = {
        let o = owner.borrow();
        (o.prefix.clone(), o.serial.clone())
    };
    scan(owner.clone(), || {
        discover_uas_units(owner.clone(), &prefix, &serial, failures)
    })
}

/// A configured storage interface kept independently of inserted media.
/// Clones share the same serialized transport and permanent disk validity.
#[derive(Clone)]
pub struct UsbStorage {
    pub(super) owner: TransportOwner,
}
#[derive(Clone)]
pub(super) enum TransportOwner {
    Bot(Rc<RefCell<Owner<Bot<Native>>>>),
    EhciBot(Rc<RefCell<Owner<Bot<EhciBulk>>>>),
    Uas(Rc<RefCell<Owner<uas::Host>>>),
}
impl UsbStorage {
    /// Controller/physical-interface location, including an empty reader.
    pub fn location(&self) -> String {
        match &self.owner {
            TransportOwner::Bot(owner) => owner.borrow().prefix.clone(),
            TransportOwner::EhciBot(owner) => owner.borrow().prefix.clone(),
            TransportOwner::Uas(owner) => owner.borrow().prefix.clone(),
        }
    }
    /// Poll LUN inventory/readiness and revalidate current media. The
    /// caller replaces its disk inventory with this result. Existing aliases
    /// remain valid only for unchanged media without an observed change event.
    /// This synchronous operation does not install a polling/hotplug scheduler.
    pub fn rescan(
        &mut self,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<Vec<UsbDisk>, BlockError> {
        match &self.owner {
            TransportOwner::Bot(owner) => scan_bot(owner.clone(), failures).map(|units| {
                units
                    .into_iter()
                    .map(|disk| UsbDisk {
                        disk: UsbTransport::Bot(disk),
                    })
                    .collect()
            }),
            TransportOwner::EhciBot(owner) => scan_bot(owner.clone(), failures).map(|units| {
                units
                    .into_iter()
                    .map(|disk| UsbDisk {
                        disk: UsbTransport::EhciBot(disk),
                    })
                    .collect()
            }),
            TransportOwner::Uas(owner) => scan_uas(owner.clone(), failures).map(|units| {
                units
                    .into_iter()
                    .map(|disk| UsbDisk {
                        disk: UsbTransport::Uas(disk),
                    })
                    .collect()
            }),
        }
    }
    /// One bounded ordinary service pass: poll real media, and replace a
    /// poisoned transport through acknowledged physical reset and fresh probes.
    /// Only metadata is reissued; a failed caller data command is never replayed.
    /// The native event loop must call this method and replace its inventory.
    pub fn service(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<Vec<UsbDisk>, BlockError> {
        if self.requires_rebind() {
            return self.reset_and_rebind(dma, failures);
        }
        let result = self.rescan(failures);
        if result.is_err() && self.requires_rebind() {
            self.reset_and_rebind(dma, failures)
        } else {
            result
        }
    }
    fn requires_rebind(&self) -> bool {
        match &self.owner {
            TransportOwner::Bot(owner) => owner.borrow().transport.poisoned(),
            TransportOwner::EhciBot(owner) => owner.borrow().transport.poisoned(),
            TransportOwner::Uas(owner) => owner.borrow().transport.requires_rebind(),
        }
    }
    /// Device reset followed by fresh transport, descriptors and media
    /// probes. This owner keeps the new interface even when no LUN is ready.
    /// Other copies of the old owner and all its disks remain quarantined.
    pub fn reset_and_rebind(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<Vec<UsbDisk>, BlockError> {
        self.owner = match &self.owner {
            TransportOwner::Bot(old) => {
                let host = {
                    let mut old = old.borrow_mut();
                    old.invalidate();
                    old.transport.reset_and_rebind(dma)
                }
                .map_err(|(operation, error)| {
                    failures.push((operation, error.clone()));
                    error
                })?;
                let prefix = format!("{} interface {}", host.device().location, host.interface());
                let serial = String::from(
                    host.device()
                        .serial
                        .as_deref()
                        .unwrap_or(&host.device().location),
                );
                TransportOwner::Bot(Rc::new(RefCell::new(Owner::new(host, prefix, serial))))
            }
            TransportOwner::EhciBot(old) => {
                let mut old = old.borrow_mut();
                old.invalidate();
                let host =
                    old.transport
                        .reset_and_rebind_ehci()
                        .map_err(|(operation, error)| {
                            failures.push((operation, error.clone()));
                            error
                        })?;
                let (prefix, serial) = (old.prefix.clone(), old.serial.clone());
                TransportOwner::EhciBot(Rc::new(RefCell::new(Owner::new(host, prefix, serial))))
            }
            TransportOwner::Uas(old) => {
                let host = {
                    let mut old = old.borrow_mut();
                    old.invalidate();
                    old.transport.reset_and_rebind(dma)
                }
                .map_err(|(operation, error)| {
                    failures.push((operation, error.clone()));
                    error
                })?;
                let prefix = format!("{} interface {}", host.device().location, host.interface());
                let serial = String::from(
                    host.device()
                        .serial
                        .as_deref()
                        .unwrap_or(&host.device().location),
                );
                TransportOwner::Uas(Rc::new(RefCell::new(Owner::new(host, prefix, serial))))
            }
        };
        let result = self.rescan(failures);
        if result.is_err() {
            match &self.owner {
                TransportOwner::Bot(owner) => owner.borrow_mut().transport.quarantine(),
                TransportOwner::EhciBot(owner) => owner.borrow_mut().transport.quarantine(),
                TransportOwner::Uas(owner) => owner.borrow_mut().transport.quarantine(),
            }
        }
        result
    }
}
impl UsbDisk {
    /// Keep the configured interface for later rescan, even after removing
    /// every published disk from the caller's inventory.
    pub fn storage(&self) -> UsbStorage {
        UsbStorage {
            owner: match &self.disk {
                UsbTransport::Bot(disk) => TransportOwner::Bot(disk.bot.clone()),
                UsbTransport::EhciBot(disk) => TransportOwner::EhciBot(disk.bot.clone()),
                UsbTransport::Uas(disk) => TransportOwner::Uas(disk.bot.clone()),
            },
        }
    }
    /// Recovery variant that preserves the fresh owner even with zero disks.
    pub fn reset_and_rebind_retained(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<(UsbStorage, Vec<UsbDisk>), BlockError> {
        let mut owner = self.storage();
        let disks = owner.reset_and_rebind(dma, failures)?;
        Ok((owner, disks))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    #[ignore = "oracle: STELA_USB_MEDIA_ORACLE"]
    fn unchanged_linux_media_helpers_match_all_normalized_sense_profiles() {
        extern crate std;
        let trace = std::fs::read(
            std::env::var("STELA_USB_MEDIA_ORACLE").expect("media helper trace path"),
        )
        .unwrap();
        assert_eq!(trace.len(), 131_072 * 10);
        for row in trace.as_chunks::<10>().0 {
            let [
                key,
                asc,
                ascq,
                valid,
                removable,
                present,
                absent,
                removed_changed,
                tur_changed,
                zero,
            ] = *row;
            let sense = Sense { key, asc, ascq };
            assert_eq!(absent != 0, valid != 0 && no_media(&sense));
            assert_eq!(removed_changed != 0, absent != 0 && present != 0);
            assert_eq!(zero != 0, absent != 0 && removable != 0);
            assert_eq!(tur_changed != 0, valid != 0 && removable != 0 && key == 6);
            if valid != 0 && removable != 0 {
                assert_eq!(
                    changed(&sense, true),
                    absent != 0 || tur_changed != 0 || key == 5 && asc == 0x25
                );
            }
        }
    }
    #[test]
    fn a_hung_uas_command_resets_the_port_and_a_changed_capacity_ends_the_disk() {
        use crate::block::BlockDevice;
        use crate::usb::xhci;
        for speed in [xhci::HIGH_SPEED, xhci::SUPER_SPEED] {
            let mut memory = vec![0u64; 512 * 1024];
            // The model's capacity grows with each reset.
            let (controller, device, hardware, mut dma) =
                xhci::model::reset_fixture(&mut memory, speed, 32, 3);
            let binding = uas::Binding::new(
                &device,
                &crate::usb::descriptor::interfaces(&device.configuration)[0],
            )
            .unwrap();
            let mut failures = Vec::new();
            // SAFETY: the fixture grants this owner its one unconfigured interface.
            let (mut storage, mut disks) = unsafe {
                super::super::discover_uas_retained(
                    controller,
                    &device,
                    binding,
                    &mut dma,
                    &mut failures,
                )
            }
            .unwrap();
            let before = hardware.borrow().scsi.len();
            hardware.borrow_mut().hang_next = true;
            // The hung write poisons the transport and the owner resets the
            // port at once. The device answers with another capacity, so
            // the disk ends and the write is not retried.
            assert!(disks[0].write(1, &[0xaa; 512]).is_err());
            assert_eq!(hardware.borrow().resets, 1);
            assert_eq!(
                hardware.borrow().scsi[before..]
                    .iter()
                    .filter(|(_, op)| *op == scsi::WRITE_10)
                    .count(),
                1
            );
            let accesses = hardware.borrow().accesses;
            assert!(disks[0].flush_cache().is_err());
            assert_eq!(hardware.borrow().accesses, accesses);
            // The rebound transport publishes the new medium.
            let mut fresh = storage.service(&mut dma, &mut failures).unwrap();
            assert_eq!(fresh[0].identity().blocks, 2048);
            assert_eq!(hardware.borrow().resets, 1);
            hardware.borrow_mut().empty_media = true;
            assert!(storage.service(&mut dma, &mut failures).unwrap().is_empty());
            assert!(fresh[0].read(0, &mut [0; 512]).is_err());
            hardware.borrow_mut().empty_media = false;
            let mut inserted = storage.service(&mut dma, &mut failures).unwrap();
            inserted[0].write(1, &[0xbb; 512]).unwrap();
            let mut bytes = [0; 512];
            inserted[0].read(1, &mut bytes).unwrap();
            assert_eq!(bytes, [0xbb; 512]);
        }
    }

    /// A unit whose first command at attach never completes, as a stick
    /// that is not ready yet: one port reset, then the fresh probe publishes
    /// the disk, with the first attempt's failure kept for the log.
    #[test]
    fn a_first_command_that_hangs_at_attach_resets_the_device_and_probes_again() {
        use crate::usb::xhci;
        let mut memory = vec![0u64; 512 * 1024];
        let (controller, device, hardware, mut dma) =
            xhci::model::reset_fixture(&mut memory, xhci::SUPER_SPEED, 32, 3);
        let binding = uas::Binding::new(
            &device,
            &crate::usb::descriptor::interfaces(&device.configuration)[0],
        )
        .unwrap();
        let mut failures = Vec::new();
        hardware.borrow_mut().hang_next = true;
        // SAFETY: exclusive fresh fixture interface; no endpoints configured.
        let (_storage, disks) = unsafe {
            super::super::discover_uas_retained(
                controller,
                &device,
                binding,
                &mut dma,
                &mut failures,
            )
        }
        .unwrap();
        assert_eq!(disks.len(), 1);
        assert_eq!(hardware.borrow().resets, 1);
        assert!(!failures.is_empty());
    }

    #[test]
    fn service_recovers_poison_observed_during_metadata_scan_once() {
        use crate::usb::xhci;
        let mut memory = vec![0u64; 512 * 1024];
        let (controller, device, hardware, mut dma) =
            xhci::model::reset_fixture(&mut memory, xhci::HIGH_SPEED, 32, 3);
        let binding = uas::Binding::new(
            &device,
            &crate::usb::descriptor::interfaces(&device.configuration)[0],
        )
        .unwrap();
        let mut failures = Vec::new();
        // SAFETY: exclusive fresh fixture interface; no endpoints configured.
        let (mut storage, mut disks) = unsafe {
            super::super::discover_uas_retained(
                controller,
                &device,
                binding,
                &mut dma,
                &mut failures,
            )
        }
        .unwrap();
        hardware.borrow_mut().hang_next = true;
        assert_eq!(storage.service(&mut dma, &mut failures).unwrap().len(), 1);
        assert_eq!(hardware.borrow().resets, 1);
        use crate::block::BlockDevice;
        assert!(disks[0].read(0, &mut [0; 512]).is_err());
    }
}
