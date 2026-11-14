//! Native IOMMU backends. Discovery/ownership must supply a checked unit;
//! there is no identity-mapping substitute for an unavailable domain.
use crate::Result;
pub use stela_boot_core::dmar;
pub use stela_boot_core::firmware_state;
pub use stela_boot_core::{requester, vtd};
use stela_contracts::refuse;

use crate::output;
use firmware_state::{Found, Kind, TableError, Units, Window};
use stela_runtime::pc::acpi;

/// Original table inventory, including whether discovery covered every unit.
pub(crate) struct FirmwareUnits {
    units: Units,
    complete: bool,
}

/// The complete original IOMMU inventory at firmware exit, twice: `found`
/// when every unit was observed inactive as the firmware left it, which
/// the proof for requesters that start blocked needs, and `off` when every
/// unit is inactive now, found so or turned off by
/// [`release_firmware_units`] with its status wait passed. Linux turns a
/// unit the firmware left on off and carries on; so does the rest of the
/// boot from `off`.
pub(crate) struct InactiveFirmwareUnits {
    found: Option<Units>,
    off: Option<Units>,
}

fn matches_amd(units: &Units, ivrs: &stela_drivers::bus::dma::amd::ivrs::Ivrs<'_>) -> bool {
    units.as_slice().len() == ivrs.units().count()
        && units.as_slice().iter().all(|unit| {
            unit.kind == Kind::AmdVi
                && ivrs.units().any(|entry| {
                    entry.id().segment == unit.segment && entry.mmio_address() == unit.base
                })
        })
}

impl InactiveFirmwareUnits {
    pub(crate) fn contains(&self, kind: Kind, segment: u16, base: u64) -> bool {
        let original = firmware_state::Unit {
            kind,
            segment,
            base,
        };
        self.found
            .as_ref()
            .is_some_and(|units| units.as_slice().contains(&original))
    }

    pub(crate) fn matches_amd(&self, ivrs: &stela_drivers::bus::dma::amd::ivrs::Ivrs<'_>) -> bool {
        self.found
            .as_ref()
            .is_some_and(|units| matches_amd(units, ivrs))
    }

    /// Every unit of `ivrs` is off now.
    pub(crate) fn off_amd(&self, ivrs: &stela_drivers::bus::dma::amd::ivrs::Ivrs<'_>) -> bool {
        self.off
            .as_ref()
            .is_some_and(|units| matches_amd(units, ivrs))
    }

    /// Every unit of the complete early inventory is off now. This holds for
    /// a table the boot could not fully parse later: the early walk only
    /// needs each structure's length and register base.
    pub(crate) fn all_off(&self) -> bool {
        self.off.is_some()
    }

    /// Every unit of `descriptors` is off now.
    pub(crate) fn off_dmar(&self, descriptors: &[dmar::Drhd]) -> bool {
        self.off
            .as_ref()
            .is_some_and(|units| units.matches_dmar(descriptors))
    }

    pub(crate) fn matches_dmar(&self, descriptors: &[dmar::Drhd]) -> bool {
        self.found
            .as_ref()
            .is_some_and(|units| units.matches_dmar(descriptors))
    }
}

/// Before ExitBootServices: every remapping unit the firmware's DMAR and IVRS
/// tables list. A table Stela cannot read is reported once and the units
/// past the defect stay unchecked.
pub(crate) fn firmware_units() -> Result<FirmwareUnits> {
    let mut units = Units::default();
    let Some(rsdp) = crate::storage::rsdp() else {
        return Ok(FirmwareUnits {
            units,
            complete: false,
        });
    };
    let mut complete = true;
    for signature in [b"DMAR", b"IVRS"] {
        // SAFETY: the firmware's RSDP; its tables stay mapped while boot
        // services run, and `find` checks each table's length.
        let added = match unsafe { acpi::find(rsdp, signature) } {
            Ok(table) if signature == b"DMAR" => units.add_dmar(table),
            Ok(table) => units.add_ivrs(table),
            Err(acpi::AcpiError::Missing(_)) => Ok(()),
            Err(_) => Err(TableError::Header),
        };
        match added {
            Ok(()) => {}
            Err(TableError::Capacity) => {
                return Err(refuse!(
                    "the host lists more IOMMU units than Stela can check"
                ));
            }
            Err(error) => {
                complete = false;
                output::arguments(format_args!(
                    "UNSUPPORTED host IOMMU table {} unreadable ({error:?}): units past the defect are not checked for firmware DMA protection\n",
                    core::str::from_utf8(signature).unwrap_or("?"),
                ));
            }
        }
    }
    Ok(FirmwareUnits { units, complete })
}

/// Reserve the AMD device-table pool independently of all driver DMA windows.
pub(crate) fn reserve_amd_tables(units: &FirmwareUnits) -> crate::handoff::Region {
    use crate::paging::PAGE;
    use stela_drivers::bus::dma::amd::device::DEVICE_TABLE_BYTES;
    use uefi::boot::{self, AllocateType, MemoryType};

    if !units.complete {
        return crate::handoff::Region::default();
    }
    let units = units
        .units
        .as_slice()
        .iter()
        .filter(|unit| unit.kind == Kind::AmdVi)
        .count();
    if units == 0 {
        return crate::handoff::Region::default();
    }
    // Each table is contiguous, with separate page-alignment and allocator
    // bookkeeping space in this exclusive native heap.
    let pages = units * (DEVICE_TABLE_BYTES.div_ceil(PAGE) + 2);
    match boot::allocate_pages(
        AllocateType::MaxAddress(u32::MAX.into()),
        MemoryType::LOADER_DATA,
        pages,
    ) {
        Ok(allocation) => {
            let region = crate::handoff::Region {
                physical: allocation.as_ptr() as u64,
                bytes: (pages * PAGE) as u64,
            };
            if amd_table_range_is_private(region) {
                region
            } else {
                // Keep this firmware allocation out of guest RAM even when
                // we cannot prove it is private enough for IOMMU metadata.
                output::arguments(format_args!(
                    "HOST AMD-Vi table reservation withheld: firmware DMA windows unqualified\n"
                ));
                crate::handoff::Region::default()
            }
        }
        Err(error) => {
            output::arguments(format_args!(
                "HOST AMD-Vi table reservation unavailable: {error:?}\n"
            ));
            crate::handoff::Region::default()
        }
    }
}

fn amd_table_range_is_private(region: crate::handoff::Region) -> bool {
    use stela_drivers::bus::dma::amd::ivrs::Ivrs;

    let Some(rsdp) = crate::storage::rsdp() else {
        return false;
    };
    let start = region.physical;
    let end = start + region.bytes;
    // SAFETY: boot services still keep these original ACPI tables mapped.
    let Ok(ivrs) = (unsafe { acpi::find(rsdp, b"IVRS") }) else {
        return false;
    };
    let Ok(ivrs) = Ivrs::parse(ivrs, firmware_state::MAX_UNITS) else {
        return false;
    };
    // A block stepped over may define memory this check cannot see.
    if !ivrs.skipped().is_empty()
        || ivrs
            .memory_definitions()
            .any(|range| range.start() < end && start < range.end())
    {
        return false;
    }
    // SAFETY: same mapped firmware table inventory. Missing DMAR means no
    // Intel reserved windows. An unreadable table supplies no exclusion proof.
    match unsafe { acpi::find(rsdp, b"DMAR") } {
        // A dropped region supplies no exclusion proof either.
        Ok(table) => dmar::parse(table).is_ok_and(|table| {
            table.complete()
                && table
                    .reserved
                    .iter()
                    .all(|range| range.base >= end || range.limit < start)
        }),
        Err(acpi::AcpiError::Missing(_)) => true,
        Err(_) => false,
    }
}

/// A unit's registers through the firmware's identity map, with a TSC
/// deadline.
struct Identity {
    base: *mut u8,
    timeout: u64,
}

impl Identity {
    fn at<T>(&self, offset: usize) -> *mut T {
        // SAFETY: offsets stay within the unit's register window (`Window`).
        unsafe { self.base.add(offset).cast() }
    }
}

// SAFETY: `release_firmware_units` builds this only for a unit's register
// base, which the firmware's page tables still identity-map uncached.
unsafe impl Window for Identity {
    fn read32(&self, offset: usize) -> u32 {
        // SAFETY: aligned register within the mapped window.
        unsafe { self.at::<u32>(offset).read_volatile() }
    }
    fn read64(&self, offset: usize) -> u64 {
        // SAFETY: aligned register within the mapped window.
        unsafe { self.at::<u64>(offset).read_volatile() }
    }
    fn write32(&self, offset: usize, value: u32) {
        // SAFETY: aligned register within the mapped window.
        unsafe { self.at::<u32>(offset).write_volatile(value) }
    }
    fn write64(&self, offset: usize, value: u64) {
        // SAFETY: aligned register within the mapped window.
        unsafe { self.at::<u64>(offset).write_volatile(value) }
    }
    fn expired(&self, start: u64) -> bool {
        self.now().wrapping_sub(start) > self.timeout
    }
    fn now(&self) -> u64 {
        // SAFETY: RDTSC has no memory effects.
        unsafe { core::arch::x86_64::_rdtsc() }
    }
}

/// After ExitBootServices, before any Stela driver starts DMA or firmware
/// memory is reused: turn off the translation, interrupt remapping, logs and
/// protected memory regions the firmware left on in each unit, reporting
/// each once; a unit that does not turn off stops the boot. Only complete
/// discovery and initial inactivity of every unit qualify the returned record.
///
/// # Safety
/// CR3 is still the firmware's identity map, which reaches every unit's
/// registers uncached; only this CPU runs Stela code and no firmware code
/// touches the units.
pub(crate) unsafe fn release_firmware_units(
    inventory: FirmwareUnits,
    tsc_hz: u64,
) -> InactiveFirmwareUnits {
    let FirmwareUnits { units, complete } = inventory;
    let mut initially_inactive = complete;
    // Every unit read and, where it was on, turned off below.
    let mut off = complete;
    let opt_in = if units.dmar_flags & 4 != 0 {
        " (DMAR asks the OS to keep DMA protection on)"
    } else {
        ""
    };
    for unit in units.as_slice() {
        let window = Identity {
            base: unit.base as *mut u8,
            timeout: tsc_hz,
        };
        let kind = match unit.kind {
            Kind::Vtd => "VT-d",
            Kind::AmdVi => "AMD-Vi",
        };
        match firmware_state::inspect(unit.kind, &window) {
            Found::Inactive => continue,
            Found::Unreadable => {
                initially_inactive = false;
                off = false;
                output::arguments(format_args!(
                    "UNSUPPORTED host IOMMU {kind} unit {:#x} segment {} reads all ones: ignored\n",
                    unit.base, unit.segment,
                ));
                continue;
            }
            found => {
                initially_inactive = false;
                output::arguments(format_args!(
                    "UNSUPPORTED host IOMMU {kind} unit {:#x} segment {} left on by the firmware {found:?}{opt_in}: Stela turns it off and host devices DMA untranslated\n",
                    unit.base, unit.segment,
                ));
            }
        }
        if let Err(firmware_state::Failure::Timeout(feature)) =
            // SAFETY: forwarded post-EBS exclusivity; no Stela driver runs yet.
            unsafe { firmware_state::release(unit.kind, &window) }
        {
            output::arguments(format_args!(
                "HOST IOMMU {kind} unit {:#x}: {feature} did not complete within one second\n",
                unit.base,
            ));
            output::fatal(
                "an IOMMU unit the firmware left on did not turn off; turn off pre-boot DMA \
                 protection (VT-d or AMD-Vi) in the firmware settings",
            );
        }
    }
    InactiveFirmwareUnits {
        found: initially_inactive.then_some(units),
        off: off.then_some(units),
    }
}
