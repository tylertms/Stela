//! Firmware resource observations, captured before EBS. They become
//! permissions only when joined to this boot's real root/controller
//! handoff and the single native mapping registry.
use super::{APERTURE, Error, LEAVES, PAGE};
use alloc::vec::Vec;
use stela_boot_core::{
    ecam::{EcamInventory, RootWindow},
    firmware_map,
};
use stela_drivers::bus::{mmio::owned::PhysicalSpan, pci::EcamWindow};
use stela_runtime::pc::acpi;
use uefi::{
    boot,
    mem::memory_map::MemoryMap,
    proto::pci::{configuration::ResourceRangeType, root_bridge::PciRootBridgeIo},
};

/// Captured metadata is owned, private and not Clone. It contains no firmware
/// protocol or descriptor pointer and cannot itself construct an access grant.
pub(crate) struct Prepared {
    pub(super) windows: Vec<EcamWindow>,
    exclusions: Vec<PhysicalSpan>,
    pub(super) physical_bits: u8,
    #[cfg(feature = "diagnostics")]
    pub(super) hpet: Option<crate::native_clock::hpet::Descriptor>,
    pub(super) leaves: Option<[u64; LEAVES]>,
}
impl Prepared {
    pub(crate) fn capture() -> Result<Self, Error> {
        let extended = core::arch::x86_64::__cpuid(0x8000_0000).eax;
        if extended < 0x8000_0008 {
            return Err(Error::ResourceInventory);
        }
        let physical_bits = crate::cpu::physical_bits();
        let rsdp = crate::storage::rsdp().ok_or(Error::ResourceInventory)?;
        // SAFETY: UEFI configuration table before EBS; the ACPI parser
        // checks the source table's length and returns owned MCFG observations.
        let table = unsafe { acpi::find(rsdp, b"MCFG") }.map_err(|_| Error::ResourceInventory)?;
        let all = acpi::mcfg(table).map_err(|_| Error::ResourceInventory)?;
        let inventory = EcamInventory::validate(&all, physical_bits).map_err(Error::Mapping)?;
        let handles = boot::find_handles::<PciRootBridgeIo>()
            .map_err(|error| Error::Firmware(error.status()))?;
        let mut roots = Vec::new();
        // SAFETY: pre-EBS BSP; joined MP callbacks and this short
        // Notify-level read exclude event-driven root settings changes. No
        // connect/disconnect/assignment occurs while copying descriptors.
        let tpl = unsafe { boot::raise_tpl(boot::Tpl::NOTIFY) };
        for (index, handle) in handles.into_iter().enumerate() {
            // SAFETY: boot-only read of the protocol. Firmware keeps
            // the descriptor storage; configuration() copies its data. Neither
            // this borrow nor any callback address survives capture/EBS.
            let bridge = unsafe {
                boot::open_protocol::<PciRootBridgeIo>(
                    boot::OpenProtocolParams {
                        handle,
                        agent: boot::image_handle(),
                        controller: None,
                    },
                    boot::OpenProtocolAttributes::GetProtocol,
                )
            }
            .map_err(|error| Error::Firmware(error.status()))?;
            let descriptors = bridge
                .configuration()
                .map_err(|error| Error::Firmware(error.status()))?;
            roots
                .try_reserve(descriptors.len())
                .map_err(|_| Error::Allocation)?;
            for descriptor in descriptors {
                let space = match descriptor.resource_range_type {
                    ResourceRangeType::Memory => 0,
                    ResourceRangeType::Io => 1,
                    ResourceRangeType::Bus => 2,
                    ResourceRangeType::Unknown(_) => return Err(Error::ResourceInventory),
                };
                roots.push(RootWindow {
                    segment: bridge.segment_nr(),
                    bridge: u32::try_from(index + 1).map_err(|_| Error::ResourceInventory)?,
                    flags: descriptor.general_flags,
                    specific: descriptor.type_specific_flags,
                    space,
                    reserved: 0,
                    minimum: descriptor.address_min,
                    maximum: descriptor.address_max,
                    translation: descriptor.translation_offset,
                    length: descriptor.address_length,
                });
            }
        }
        drop(tpl);
        let mut windows = Vec::new();
        for root in roots.iter().filter(|root| root.space == 2) {
            let segment = u16::try_from(root.segment).map_err(|_| Error::ResourceInventory)?;
            let first = u8::try_from(root.minimum).map_err(|_| Error::ResourceInventory)?;
            let last = u8::try_from(root.maximum).map_err(|_| Error::ResourceInventory)?;
            let mut covered = [false; 256];
            for (index, window) in all
                .iter()
                .enumerate()
                .filter(|(_, window)| window.segment == segment)
            {
                let start = first.max(window.start_bus);
                let end = last.min(window.end_bus);
                if start > end {
                    continue;
                }
                let (selected, _) = inventory
                    .subset(index, start..=end, &roots)
                    .map_err(Error::Mapping)?;
                for bus in start..=end {
                    covered[usize::from(bus)] = true;
                }
                windows.try_reserve(1).map_err(|_| Error::Allocation)?;
                windows.push(selected);
            }
            if first > last || (first..=last).any(|bus| !covered[usize::from(bus)]) {
                return Err(Error::ResourceInventory);
            }
        }
        if windows.is_empty() {
            return Err(Error::ResourceInventory);
        }
        EcamInventory::validate(&windows, physical_bits).map_err(Error::Mapping)?;
        let map = boot::memory_map(boot::MemoryType::LOADER_DATA)
            .map_err(|error| Error::Firmware(error.status()))?;
        let mut exclusions = Vec::new();
        for entry in map.entries() {
            if firmware_map::leaves_to_devices(entry.ty.0) || entry.page_count == 0 {
                continue;
            }
            let bytes = entry
                .page_count
                .checked_mul(PAGE as u64)
                .ok_or(Error::ResourceInventory)?;
            exclusions.try_reserve(1).map_err(|_| Error::Allocation)?;
            exclusions.push(PhysicalSpan::new(entry.phys_start, bytes).map_err(Error::Mapping)?);
        }
        let result = Self {
            windows,
            exclusions,
            physical_bits,
            #[cfg(feature = "diagnostics")]
            hpet: None,
            leaves: None,
        };
        for window in &result.windows {
            result.device_range(window.range().start, window.range().end)?;
        }
        Ok(result)
    }
    #[cfg(feature = "diagnostics")]
    pub(crate) fn capture_hpet(&mut self) -> Result<(), Error> {
        let rsdp = crate::storage::rsdp().ok_or(Error::ResourceInventory)?;
        // SAFETY: these are the original UEFI ACPI tables before EBS.
        let table = unsafe { acpi::find(rsdp, b"HPET") }.map_err(|_| Error::ResourceInventory)?;
        let descriptor = crate::native_clock::hpet::Descriptor::from_sdt(table)
            .map_err(|_| Error::ResourceInventory)?;
        let start = descriptor.registers().start();
        self.device_range(
            start,
            start
                .checked_add(PAGE as u64)
                .ok_or(Error::ResourceInventory)?,
        )?;
        self.hpet = Some(descriptor);
        Ok(())
    }
    /// Allocate empty leaf tables before EBS. No device is mapped yet.
    pub(crate) fn map(&mut self, tables: &mut super::super::Tables) -> Result<(), Error> {
        if self.leaves.is_some() {
            return Err(Error::ResourceConflict);
        }
        let mut leaves = [0; LEAVES];
        for (index, leaf) in leaves.iter_mut().enumerate() {
            let address = APERTURE + index * super::super::LARGE_PAGE;
            tables.small(address).map_err(Error::Tables)?;
            let entry = tables.directory_entry(address).map_err(Error::Tables)?;
            // SAFETY: exclusive pre-activation builder and exact returned slot.
            *leaf = unsafe { entry.read() } & super::super::ADDRESS;
        }
        self.leaves = Some(leaves);
        Ok(())
    }
    pub(super) fn device_range(&self, start: u64, end: u64) -> Result<(), Error> {
        if start >= end
            || !(32..=52).contains(&self.physical_bits)
            || end > 1u64 << self.physical_bits
            || !start.is_multiple_of(PAGE as u64)
            || !end.is_multiple_of(PAGE as u64)
            || self
                .exclusions
                .iter()
                .any(|span| span.start() < end && start < span.end())
        {
            return Err(Error::ResourceConflict);
        }
        Ok(())
    }
}
