//! Original ECAM scopes. Temporary bus maps and kept read-only function
//! views are private to their consumers and never enter physical lookup.
use super::*;
use crate::native_mmio::ecam::EcamInventory;
use core::{mem::ManuallyDrop, ops::RangeInclusive};
use stela_boot_core::ecam::census;
use stela_drivers::bus::{mmio::owned::Mapping, pci::Address};
mod containment;
pub(super) use containment::PciSlots;
pub(crate) use containment::{ContainedPci, ContainmentError, PciScope};

/// Log the first memory map conflict that keeps a bus's configuration
/// space unmapped, once per boot: the census error itself names no bus.
fn report_ecam_conflict(line: core::fmt::Arguments<'_>) {
    static REPORTED: core::sync::atomic::AtomicBool = core::sync::atomic::AtomicBool::new(false);
    if !REPORTED.swap(true, core::sync::atomic::Ordering::Relaxed) {
        crate::output::arguments(line);
    }
}

/// Log a census oddity that leaves the segment usable, the first time
/// each kind is seen this boot.
fn report_oddity(oddity: census::Oddity, address: Address) {
    static REPORTED: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
    let bit = oddity.bit();
    if REPORTED.fetch_or(bit, core::sync::atomic::Ordering::Relaxed) & bit == 0 {
        crate::output::arguments(format_args!("HOST PCI {address} {}\n", oddity.message()));
    }
}

pub(crate) struct ConfigScope {
    pending: ManuallyDrop<Box<Pending>>,
    close: Option<AdmissionClose>,
}

struct Pending {
    assignment: Option<Assignment<Reservation>>,
    resource: Option<Resource>,
    mapping: Option<Mapping>,
}

impl ConfigScope {
    /// # Safety
    /// The caller holds this original configuration extent and exclusion.
    unsafe fn open(source: &Source, span: PhysicalSpan) -> Result<Self, Error> {
        let bytes = usize::try_from(span.bytes()).map_err(|_| Error::InvalidRange)?;
        let (resource, assignment) = source.reservation(span, ResourceKind::Registers)?;
        let close = assignment.admission_close();
        let mut scope = Self {
            pending: ManuallyDrop::new(Box::new(Pending {
                assignment: Some(assignment),
                resource: Some(resource.clone()),
                mapping: None,
            })),
            close: Some(close),
        };
        scope.pending.mapping = Some(source.authority.map(
            resource,
            0,
            bytes,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )?);
        Ok(scope)
    }

    fn read(&self, offset: usize, width: u16) -> Result<u32, Error> {
        let mapping = self.pending.mapping.as_ref().ok_or(Error::Unavailable)?;
        match width {
            1 => {
                let address = mapping.address(offset, 1)?;
                // SAFETY: the live read-only map covers this byte.
                Ok(u32::from(unsafe { address.as_ptr().read_volatile() }))
            }
            2 if offset.is_multiple_of(2) => {
                let address = mapping.address(offset, 2)?;
                // SAFETY: the map is page-aligned and the word offset is aligned.
                let value = unsafe { address.cast::<u16>().as_ptr().read_volatile() };
                Ok(u32::from(u16::from_le(value)))
            }
            4 => mapping.read32(offset),
            _ => Err(Error::InvalidRange),
        }
    }

    /// # Safety
    /// Every accepted mapping user has stopped, with no escaping aliases.
    unsafe fn retire(&mut self) -> Result<(), Error> {
        self.close
            .as_ref()
            .expect("original configuration gate")
            .close();
        if let Some(mapping) = &mut self.pending.mapping {
            // SAFETY: the original scope has no remaining mapping users.
            unsafe { mapping.retire() }?;
            self.pending.mapping = None;
        }
        self.return_assignment()
    }

    /// Keep the original configuration assignment on this CPU while the
    /// unique shared mapping is retired by the other processor.
    ///
    /// # Safety
    /// All mapping users have drained. The callback returns success only
    /// after retiring this exact mapping and acquiring the remote return.
    #[cfg(feature = "diagnostics")]
    pub(crate) unsafe fn retire_remotely(
        mut self,
        retire: impl FnOnce(Mapping) -> crate::Result<()>,
    ) -> crate::Result<()> {
        self.close
            .as_ref()
            .expect("original configuration gate")
            .close();
        retire(self.pending.mapping.take().expect("original map"))?;
        self.return_assignment().map_err(|error| {
            stela_contracts::refuse!(
                "AP configuration assignment return refused: {error}",
                ?error
            )
        })
    }

    fn return_assignment(&mut self) -> Result<(), Error> {
        drop(self.pending.resource.take());
        self.close
            .take()
            .expect("original configuration gate")
            .dismiss();
        let assignment = self
            .pending
            .assignment
            .take()
            .expect("original configuration assignment");
        match assignment.into_original() {
            Ok(_) => Ok(()),
            Err(refusal) => {
                self.pending.assignment = Some(refusal.owner);
                Err(Error::Assignment(refusal.error))
            }
        }
    }
}

impl Drop for ConfigScope {
    fn drop(&mut self) {
        if self.pending.assignment.is_none() {
            // SAFETY: acknowledged map withdrawal and original assignment
            // return left no provider obligation in this allocation.
            unsafe { ManuallyDrop::drop(&mut self.pending) };
        }
    }
}

/// The original source lends a read-only bus at a time. A mapped bus is
/// withdrawn and acknowledged before a different bus is admitted.
pub(crate) struct ConfigInventory<'a> {
    source: &'a Source,
    windows: &'a [EcamWindow],
    geometry: EcamInventory<'a>,
    mapped: Option<((u16, u8), ConfigScope)>,
}

impl ConfigInventory<'_> {
    pub(crate) fn read32(&mut self, address: Address, offset: u16) -> Result<u32, Error> {
        self.read(address, offset, 4)
    }

    fn read(&mut self, address: Address, offset: u16, width: u8) -> Result<u32, Error> {
        let width = u16::from(width);
        if address.device >= 32
            || address.function >= 8
            || !matches!(width, 1 | 2 | 4)
            || offset > 4096 - width
            || !offset.is_multiple_of(width)
        {
            return Err(Error::InvalidRange);
        }
        let key = (address.segment, address.bus);
        if !self
            .mapped
            .as_ref()
            .is_some_and(|(current, _)| *current == key)
        {
            self.withdraw()?;
            let index = self
                .windows
                .iter()
                .position(|window| {
                    window.segment == address.segment
                        && (window.start_bus..=window.end_bus).contains(&address.bus)
                })
                .ok_or(Error::OutsideResource)?;
            let (_, span) = self.geometry.subset(
                index,
                address.bus..=address.bus,
                self.source.platform.wire().roots(),
            )?;
            // SAFETY: the caller owns the original complete captured buses.
            // Only this bounded read observes the newly mapped bus.
            let scope = unsafe { ConfigScope::open(self.source, span) }?;
            self.mapped = Some((key, scope));
        }
        let (_, scope) = self.mapped.as_ref().expect("current read-only bus");
        let offset = (usize::from(address.device) << 15)
            | (usize::from(address.function) << 12)
            | usize::from(offset);
        scope.read(offset, width)
    }

    /// Check a bridge's buses against its root grant, the ECAM windows and
    /// the bridges `admitted` before it (`census::admit`), returning the
    /// buses it forwards. The primary field routes nothing; like Linux, a
    /// wrong one is kept.
    pub(crate) fn check_bridge(
        &self,
        address: Address,
        buses: u32,
        admitted: &[(Address, RangeInclusive<u8>)],
    ) -> Result<Option<RangeInclusive<u8>>, census::Misconfigured> {
        let roots = self.source.platform.wire().roots();
        let within = |range| self.geometry.check_bridge(roots, address, range).is_ok();
        let forwarded = census::admit(address, buses, within, admitted)?;
        if forwarded.is_some() && buses as u8 != address.bus {
            report_oddity(census::Oddity::PrimaryBus, address);
        }
        Ok(forwarded)
    }

    fn withdraw(&mut self) -> Result<(), Error> {
        if let Some((_, mut scope)) = self.mapped.take() {
            // SAFETY: only synchronous reads, with no external map views.
            unsafe { scope.retire() }?;
        }
        Ok(())
    }
}

impl Source {
    /// Every ECAM decode window from the same sealed MCFG used by the census.
    pub(crate) fn configuration_windows(&self) -> Result<Vec<EcamWindow>, Error> {
        let mcfg = self.platform.wire().mcfg();
        if mcfg.is_empty() {
            return Ok(Vec::new());
        }
        let windows = stela_runtime::pc::acpi::mcfg(mcfg).map_err(failed)?;
        EcamInventory::validate(&windows, self.platform.space.physical_bits)?;
        Ok(windows)
    }

    /// # Safety
    /// The boot owns all captured configuration buses, excluding firmware,
    /// reset, topology and SR-IOV writers through the observation and its use.
    pub(crate) unsafe fn with_config_inventory<T, E: From<Error>>(
        &self,
        body: impl FnOnce(&[EcamWindow], &mut ConfigInventory<'_>) -> Result<T, E>,
    ) -> Result<T, E> {
        let wire = self.platform.wire();
        let windows = stela_runtime::pc::acpi::mcfg(wire.mcfg()).map_err(failed)?;
        if windows.is_empty() {
            return Err(Error::Unavailable.into());
        }
        let bits = self.platform.space.physical_bits;
        let original = EcamInventory::validate(&windows, bits)?;
        let windows = original.root_windows(wire.roots())?;
        let geometry = EcamInventory::validate(&windows, bits)?;
        for (index, window) in windows.iter().enumerate() {
            for bus in window.start_bus..=window.end_bus {
                let (_, span) = geometry.subset(index, bus..=bus, wire.roots())?;
                if let Some((start, end, kind)) = wire.memory_conflict(span.start(), span.end()) {
                    report_ecam_conflict(format_args!(
                        "HOST PCI configuration space of bus {:04x}:{bus:02x} at {:#x} lies in \
                         the firmware memory map's {} at {start:#x}-{end:#x}; it must be \
                         reserved or MMIO, so Stela does not map it\n",
                        window.segment,
                        span.start(),
                        stela_boot_core::firmware_map::kind_name(kind),
                    ));
                    return Err(Error::OutsideResource.into());
                }
            }
        }
        let mut reader = ConfigInventory {
            source: self,
            windows: &windows,
            geometry,
            mapped: None,
        };
        let result = body(&windows, &mut reader);
        reader.withdraw()?;
        result
    }

    fn config_page(&self, address: Address) -> Result<(EcamWindow, PhysicalSpan), Error> {
        let wire = self.platform.wire();
        let windows = stela_runtime::pc::acpi::mcfg(wire.mcfg()).map_err(failed)?;
        let inventory = super::super::ecam::EcamInventory::validate(
            &windows,
            self.platform.space.physical_bits,
        )?;
        let (window, span) = inventory.function(address, wire.roots())?;
        if !wire.excludes_memory(span.start(), span.end()) {
            return Err(Error::OutsideResource);
        }
        Ok((window, span))
    }

    /// Read an AML PCI operation through a scoped UC map without lending a
    /// writable address or letting the provider change the physical function.
    /// A bridge whose forwarding is restored is read through the owner that
    /// keeps its page. A bus of an MCFG window that no root bridge forwards
    /// reads all ones, as the root complex completes such a request.
    pub(crate) fn read_firmware_config(
        &self,
        address: Address,
        offset: u16,
        width: u16,
    ) -> Result<u32, Error> {
        if !matches!(width, 1 | 2 | 4)
            || offset > PAGE as u16 - width
            || !offset.is_multiple_of(width)
            || address.device >= 32
            || address.function >= 8
        {
            return Err(Error::InvalidRange);
        }
        let wire = self.platform.wire();
        let windows = stela_runtime::pc::acpi::mcfg(wire.mcfg()).map_err(failed)?;
        if windows.iter().any(|window| {
            window.segment == address.segment
                && (window.start_bus..=window.end_bus).contains(&address.bus)
        }) && !stela_boot_core::ecam::root_decodes(wire.roots(), address.segment, address.bus)
        {
            return Ok(u32::MAX >> (32 - u32::from(width) * 8));
        }
        if let Some(value) = self.read_forwarding_bridge(address, offset, width) {
            return value;
        }
        let (_, span) = self.config_page(address)?;
        // SAFETY: the original capture supplies this function extent. This
        // synchronous BSP call lends no mapping or configuration writer.
        let mut scope = unsafe { ConfigScope::open(self, span) }?;
        let value = scope.read(usize::from(offset), width);
        // SAFETY: the only reader was the synchronous scalar access above.
        unsafe { scope.retire() }?;
        value
    }

    /// Map one exact read-only function for a kept native observer.
    /// # Safety
    /// The original firmware and competing configuration writers are excluded
    /// through this function's observation and subsequent map retirement.
    pub(crate) unsafe fn observe_config_page(
        &self,
        address: Address,
    ) -> Result<(EcamWindow, Mapping), Error> {
        let (window, span) = self.config_page(address)?;
        let mut reservations = self
            .reservations
            .try_borrow_mut()
            .map_err(|_| Error::Busy)?;
        reservations.try_reserve(1).map_err(|_| Error::Exhausted)?;
        let (resource, assignment) = self.reservation(span, ResourceKind::Registers)?;
        reservations.push(assignment);
        drop(reservations);
        let mapping = self.authority.map(
            resource,
            0,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadOnly,
        )?;
        Ok((window, mapping))
    }

    /// Map and read the first captured configuration page on the BSP. The
    /// scope keeps the original reservation and sole mapping for retirement.
    ///
    /// # Safety
    /// This boot keeps the original function stable and excludes competing
    /// writers until the scope withdraws the mapping.
    #[cfg(feature = "diagnostics")]
    pub(crate) unsafe fn check_shootdown_mapping(&self) -> Result<(ConfigScope, u32), Error> {
        let windows = stela_runtime::pc::acpi::mcfg(self.platform.wire().mcfg()).map_err(failed)?;
        let window = windows.first().ok_or(Error::Unavailable)?;
        let (_, span) = self.config_page(Address {
            segment: window.segment,
            bus: window.start_bus,
            device: 0,
            function: 0,
        })?;
        // SAFETY: the caller keeps this exact original configuration extent.
        let mut scope = unsafe { ConfigScope::open(self, span) }?;
        let result = scope
            .pending
            .mapping
            .as_ref()
            .expect("original map")
            .read32(0);
        match result {
            Ok(identity) => Ok((scope, identity)),
            Err(error) => {
                // SAFETY: the synchronous observation has no aliases.
                unsafe { scope.retire() }?;
                Err(error)
            }
        }
    }

    /// Retire on the BSP after the APs start and return the acknowledged generation.
    ///
    /// # Safety
    /// This boot retains the diagnostic configuration page through retirement.
    #[cfg(feature = "diagnostics")]
    pub(crate) unsafe fn check_shootdown(&self) -> Result<u64, Error> {
        // SAFETY: the caller keeps the original function stable.
        let (mut scope, identity) = unsafe { self.check_shootdown_mapping() }?;
        let before = PROCESSORS.requested();
        // SAFETY: the synchronous read was the mapping's only use.
        unsafe { scope.retire() }?;
        let generation = PROCESSORS.requested();
        if identity == u32::MAX || generation == before || PROCESSORS.alone() {
            return Err(Error::InvalidPlatformReply);
        }
        Ok(generation)
    }
}
