//! The host's DMA remapping units, VT-d's (DMAR) and AMD-Vi's (IVRS, in
//! [`amd`]), taken before the host drivers start
//! ([`stela_boot_core::remapping`] decides which, and
//! [`initial`](stela_boot_core::remapping::initial) how each starts, from
//! the drivers of the devices behind it). A unit whose devices include one
//! whose driver programs physical addresses in the drivers' DMA pool (NVMe,
//! AHCI, xHCI, Ethernet, a USB firmware handoff) starts in a window that
//! maps the pool and the unit's reserved regions one to one. A unit with
//! none starts blocked. When a device the machine needs attaches only
//! natively (VMD, EHCI) and the fabric cannot be blocked whole, no unit is
//! taken. With every unit off, units that cannot all start blocked are all
//! left, as any other mix of taken units keeps the IOAPICs and every native
//! device refused; a blocked fabric that does not qualify then stops its
//! taken units the same way. A driver that
//! asks the native services for a domain gets its own
//! ([`Remappers::domain`]).
//!
//! With no unit taken and every described unit proven off since
//! ExitBootServices, the machine is treated as one that describes none.
//!
//! The display functions a driver maps the firmware's reserved regions for
//! are a policy input: none yet, so a unit whose scope holds a display
//! controller is left untranslated until a display driver claims it.
mod amd;
mod preparation;

use super::storage::RestoreRoot;
use crate::{
    native_dma::{
        InactiveFirmwareUnits, dmar,
        firmware_state::Kind,
        requester::RequesterClaim,
        vtd::interrupt::DirectDelivery,
        vtd::{
            BlockedRequester, BoundRequester, InitialBlock, MmioRegisters, TakeOverError, Unit,
            VtdError,
        },
    },
    native_mmio, output,
};
use alloc::{sync::Arc, vec::Vec};
use core::ops::Range;
use stela_boot_core::remapping::{
    self, Present,
    initial::{self, Access, Candidate, Left, Mode, Requester},
};
use stela_drivers::bus::{
    dma::{
        mapping::{DmaDomain, DmaError},
        memory::CacheAttribute,
    },
    mmio::owned::{Error as MappingError, PhysicalSpan, Protection},
    pci::{
        Address, ConfigError, ConfigFailure,
        requester::{FunctionRole, RequesterBinding, roles},
    },
};
use stela_runtime::pc::acpi;

/// The processor's physical address width, less what AMD memory
/// encryption takes, so that no window address sets the C-bit.
fn physical_bits() -> u8 {
    crate::cpu::physical_bits()
}

/// The display functions whose driver maps their unit's reserved regions.
const CLAIMED_DISPLAYS: &[u16] = &[];

struct Taken {
    descriptor: usize,
    unit: Arc<Unit<MmioRegisters>>,
    initial: InitialContexts,
}

enum InitialContexts {
    Window,
    Blocked,
}

/// The drained initial cohorts of every original Intel DMA unit, paired
/// with the same MMIO source that contained the PCI functions. Line
/// controller and endpoint-engine retirement remain independent.
struct BlockedFabric {
    source: &'static native_mmio::Source,
    units: Vec<(usize, InitialBlock<MmioRegisters>)>,
}

impl BlockedFabric {
    /// Observe every original unit in the same direct-delivery epoch. The
    /// caller may hold the pins across a later IRQ handoff.
    ///
    /// # Safety
    /// The original Source is mapped on the owning process CPU and excludes
    /// register/reset writers. No native interrupt-format change is in flight.
    unsafe fn pin_direct_delivery(&self) -> Result<Vec<DirectDelivery<MmioRegisters>>, VtdError> {
        let mut pins = Vec::new();
        pins.try_reserve_exact(self.units.len())
            .map_err(|_| VtdError::Exhausted)?;
        for (_, cohort) in &self.units {
            // SAFETY: caller holds this exact mounted source and process root.
            pins.push(unsafe { cohort.pin_direct_delivery() }?);
        }
        for pin in &pins {
            // SAFETY: same mounted context. All original units remain pinned
            // until the complete cohort has been checked.
            unsafe { pin.check() }?;
        }
        Ok(pins)
    }

    fn requester(
        &self,
        source: &native_mmio::Source,
        descriptor: usize,
        rid: u16,
        aliases: &[u16],
    ) -> Result<BlockedRequester<MmioRegisters>, DmaError> {
        if !core::ptr::eq(self.source, source) {
            return Err(DmaError::IommuUnavailable);
        }
        let (_, cohort) = self
            .units
            .iter()
            .find(|(index, _)| *index == descriptor)
            .ok_or(DmaError::IommuUnavailable)?;
        cohort.claim(rid, aliases)
    }
}

/// The units the mount translates for.
pub(super) struct Remappers {
    taken: Vec<Taken>,
    initial_fabric: Option<BlockedFabric>,
    initial_lines: Option<native_mmio::InitialPciLines>,
    requesters: core::cell::RefCell<Vec<BoundRequester<MmioRegisters>>>,
    amd: Vec<amd::Taken>,
    amd_fabric: Option<&'static native_mmio::Source>,
    roles: core::cell::RefCell<roles::Inventory>,
    topology: core::cell::RefCell<Vec<(u16, Vec<Present>)>>,
    dmar: Option<dmar::Remapping>,
    /// The firmware describes no remapping hardware: every requester's DMA
    /// and interrupt messages reach memory and the APICs untranslated.
    unremapped: bool,
}

#[derive(Clone, Copy)]
pub(super) enum PreparationError {
    Dma(DmaError),
    Interrupt(MappingError),
    Configuration(native_mmio::ContainmentError),
    AmdConfiguration,
}

impl From<DmaError> for PreparationError {
    fn from(error: DmaError) -> Self {
        Self::Dma(error)
    }
}

fn address(segment: u16, rid: u16) -> Address {
    Address {
        segment,
        bus: (rid >> 8) as u8,
        device: (rid >> 3 & 31) as u8,
        function: (rid & 7) as u8,
    }
}

/// The requester ID of the function at `address`.
fn requester(address: Address) -> u16 {
    u16::from(address.bus) << 8 | u16::from(address.device) << 3 | u16::from(address.function)
}

impl Remappers {
    /// Whether a complete blocked fabric qualified. The policy chooses one
    /// only when no device needs the drivers' pool, so legacy probe has
    /// nothing to write; it keeps the original PCI view for reading.
    pub(super) fn has_blocked_fabric(&self) -> bool {
        self.initial_fabric.is_some() || self.amd_fabric.is_some()
    }

    /// How the DMA of the function at `address` reaches memory until its
    /// driver attaches a domain of its own.
    pub(super) fn initial_access(&self, address: Address) -> Access {
        if self.unremapped {
            return Access::Untranslated;
        }
        let rid = requester(address);
        match self.amd(address.segment, rid) {
            Ok(Some(taken)) if taken.windowed() && taken.covers(address.segment, rid) => {
                return Access::Window;
            }
            Ok(Some(_)) | Err(_) => return Access::Blocked,
            Ok(None) => {}
        }
        match self.vtd(address.segment, rid) {
            Ok(Some(taken)) if !taken.unit.allows(rid) => Access::Blocked,
            Ok(Some(taken)) => match taken.initial {
                InitialContexts::Window => Access::Window,
                InitialContexts::Blocked => Access::Blocked,
            },
            Ok(None) => Access::Untranslated,
            Err(_) => Access::Blocked,
        }
    }

    pub(super) const fn none() -> Self {
        Self {
            taken: Vec::new(),
            initial_fabric: None,
            initial_lines: None,
            requesters: core::cell::RefCell::new(Vec::new()),
            amd: Vec::new(),
            amd_fabric: None,
            roles: core::cell::RefCell::new(roles::Inventory::empty()),
            topology: core::cell::RefCell::new(Vec::new()),
            dmar: None,
            unremapped: false,
        }
    }

    /// Take the units of the firmware's DMAR the policy allows. A unit with
    /// retained USB reserved regions gets a window over those regions and
    /// `drivers`. Other units start blocked. Units the policy leaves and units
    /// that refuse are reported.
    ///
    /// # Safety
    /// No host driver has started. `root` is the loader root for the ACPI
    /// tables at `rsdp`, and firmware translation ended at ExitBootServices.
    /// PCI function, SR-IOV, reset and routing writers stay excluded through
    /// inventory and later claims. `pci` retains the same captured ECAM buses.
    pub(super) unsafe fn take(
        rsdp: u64,
        root: u64,
        drivers: Range<u64>,
        pci: &native_mmio::ContainedPci<'static>,
        inactive_firmware_units: &InactiveFirmwareUnits,
        tsc_hz: u64,
    ) -> Self {
        let mmio = pci.source();
        let mut remappers = Self::none();
        // A function whose Bus Master bit stayed set may master DMA that no
        // stop accounted for. Every unit then stays as the firmware left it
        // at ExitBootServices and no native requester is prepared, as when
        // containment fails: the legacy probe path runs with nothing proven
        // stopped, and a blocked fabric or untranslated settle, which both
        // rest on every function being stopped, is never published.
        if let Some(address) = pci.unstopped_master() {
            output::arguments(format_args!(
                "HOST IOMMU not taken: PCI function {address} may still master DMA\n"
            ));
            return remappers;
        }
        let (dmar, ivrs) = {
            // SAFETY: forwarded; the loader root maps the original tables.
            let _root = unsafe { RestoreRoot::enter(root) };
            let find = |signature| {
                if rsdp == 0 {
                    return None;
                }
                // SAFETY: the firmware's RSDP, whose tables the loader root maps.
                unsafe { acpi::find(rsdp, signature) }
                    .ok()
                    .map(<[u8]>::to_vec)
            };
            (find(b"DMAR"), find(b"IVRS"))
        };
        // SAFETY: the original source owns every captured bus and excludes
        // configuration writers throughout the complete topology/role census.
        let inventory = unsafe {
            mmio.with_config_inventory(|windows, reader| {
                let inventories =
                    remapping::try_enumerate_segments(windows, |window, rid, offset| {
                        // The original containment census validated the
                        // bridge layout and bus route before sealing them;
                        // what it left out reads as absent here too.
                        pci.inventory_read(reader, address(window.segment, rid), offset)
                    })?;
                let functions: Vec<_> = inventories
                    .iter()
                    .flat_map(|(segment, present)| {
                        present
                            .iter()
                            .map(|function| address(*segment, function.rid))
                    })
                    .collect();
                if functions.len() != pci.functions() {
                    return Err(MappingError::InvalidPlatformReply.into());
                }
                // Each function's vendor and device, for what its driver
                // needs of the unit that routes it.
                let mut identities = Vec::new();
                identities
                    .try_reserve_exact(functions.len())
                    .map_err(|_| MappingError::Exhausted)?;
                for &address in &functions {
                    let value = reader.read32(address, 0)?;
                    pci.check_observation(address, 0, value)?;
                    identities.push((address, (value as u16, (value >> 16) as u16)));
                }
                let roles = roles::Inventory::capture(&functions, |address, offset| {
                    reader
                        .read32(address, offset)
                        .map(Some)
                        .map_err(|error| ConfigError::before_effect(ConfigFailure::Mapping(error)))
                });
                pci.check_stopped(reader, native_mmio::PciScope::Fabric)
                    .map(|()| (inventories, identities, roles))
            })
        };
        let (inventories, identities, roles) = match inventory {
            Ok(inventory) => inventory,
            Err(error) => {
                output::arguments(format_args!(
                    "UNSUPPORTED Stela PCI topology inventory: {error:?}\n"
                ));
                return remappers;
            }
        };
        let roles = match roles {
            Ok(roles) => roles,
            Err(error) => {
                output::arguments(format_args!(
                    "UNSUPPORTED Stela PCI function-role inventory: {error:?}\n"
                ));
                return remappers;
            }
        };
        for issue in roles.issues() {
            output::arguments(format_args!(
                "UNSUPPORTED Stela PCI function-role inventory: {issue:?}\n"
            ));
        }
        *remappers.roles.get_mut() = roles;
        *remappers.topology.get_mut() = inventories.clone();
        if dmar.is_none() && ivrs.is_none() {
            remappers.unremapped = true;
            output::arguments(format_args!(
                "HOST IOMMU none described: device DMA and interrupts are untranslated\n"
            ));
            return remappers;
        }
        let requesters: Vec<Requester> = inventories
            .iter()
            .flat_map(|(segment, present)| {
                initial::requesters(*segment, present, |rid| {
                    let at = address(*segment, rid);
                    identities
                        .iter()
                        .find(|(address, _)| *address == at)
                        .map(|(_, identity)| *identity)
                })
            })
            .collect();
        // Whether a unit was left in a state no proof covers: partly
        // programmed, or not stopped after the policy refused it.
        let mut uncertain = false;
        // A table with blocks or entries stepped over may hide units or
        // routes, so no AMD-Vi unit translates from it; each stays as the
        // boot left it, and `settle` decides from whether all read off.
        let ivrs_complete = ivrs.as_ref().is_none_or(|table| {
            match stela_drivers::bus::dma::amd::ivrs::Ivrs::parse(table, amd::MAXIMUM_BLOCKS) {
                Ok(parsed) if !parsed.skipped().is_empty() => {
                    output::arguments(format_args!(
                        "UNSUPPORTED host IVRS parts stepped over {:?}: AMD-Vi units stay untranslated\n",
                        parsed.skipped()
                    ));
                    false
                }
                // `amd::take` reports a table it cannot read.
                _ => true,
            }
        });
        if let Some(ivrs) = ivrs.as_ref().filter(|_| ivrs_complete) {
            // SAFETY: forwarded.
            let outcome = unsafe {
                amd::take(
                    ivrs,
                    &inventories,
                    &requesters,
                    &drivers,
                    mmio,
                    dmar.is_none().then_some(inactive_firmware_units),
                    tsc_hz,
                    physical_bits(),
                )
            };
            remappers.amd = outcome.units;
            uncertain |= outcome.uncertain;
            let blocked = outcome.fabric;
            if blocked == Some(true) {
                // SAFETY: no driver, AP or competing configuration writer
                // can change the original stopped PCI cohort during takeover.
                match unsafe {
                    remappers.recheck_stopped_by_page(pci, native_mmio::PciScope::Fabric)
                } {
                    Ok(()) => {
                        remappers.amd_fabric = Some(mmio);
                        output::arguments(format_args!(
                            "HOST IOMMU AMD-Vi blocked PCI fabric qualified: {} units\n",
                            remappers.amd.len()
                        ));
                    }
                    Err(error) => output::arguments(format_args!(
                        "HOST IOMMU AMD-Vi blocked PCI fabric changed: {error}\n"
                    )),
                }
            } else if blocked == Some(false) {
                output::arguments(format_args!(
                    "HOST IOMMU AMD-Vi blocked PCI fabric incomplete\n"
                ));
            }
        }
        let Some(table) = dmar else {
            let units_off = ivrs.as_ref().is_some_and(|ivrs| {
                match stela_drivers::bus::dma::amd::ivrs::Ivrs::parse(ivrs, amd::MAXIMUM_BLOCKS) {
                    Ok(ivrs) => inactive_firmware_units.off_amd(&ivrs),
                    Err(_) => inactive_firmware_units.all_off(),
                }
            });
            remappers.settle(units_off && !uncertain);
            return remappers;
        };
        let parsed = match dmar::parse(&table) {
            Ok(parsed) => parsed,
            Err(error) => {
                // No unit was taken from this table. With every unit the
                // early inventory found off, nothing translates, as with no
                // IOMMU; otherwise native attachments stay refused.
                output::arguments(format_args!(
                    "UNSUPPORTED host DMAR unreadable ({error:?}): no VT-d unit translates\n"
                ));
                remappers.settle(inactive_firmware_units.all_off() && !uncertain);
                return remappers;
            }
        };
        if !parsed.skipped.is_empty() {
            output::arguments(format_args!(
                "UNSUPPORTED host DMAR parts passed over {:?}{}\n",
                parsed.skipped,
                if parsed.complete() {
                    ""
                } else {
                    ": no VT-d unit translates"
                }
            ));
        }
        let mut cohorts = Vec::new();
        remappers.dmar = Some(parsed.clone());
        if cohorts.try_reserve_exact(parsed.units.len()).is_err()
            || remappers
                .taken
                .try_reserve_exact(parsed.units.len())
                .is_err()
        {
            output::arguments(format_args!(
                "HOST IOMMU VT-d inventory could not be retained\n"
            ));
            return remappers;
        }
        let mut covered = 0;
        // Every unit the planner resolved, with the reserved regions its
        // requesters still use, then how each starts.
        let mut planned = Vec::new();
        for (segment, present) in &inventories {
            for (index, decision) in remapping::plan(&parsed, *segment, present, CLAIMED_DISPLAYS) {
                let reserved: Vec<Range<u64>> = match &decision {
                    Ok(requesters) => parsed
                        .reserved
                        .iter()
                        .filter(|region| {
                            region.segment == *segment
                                && region.scopes.iter().any(|scope| {
                                    scope
                                        .resolve(present)
                                        .iter()
                                        .any(|rid| requesters.contains(rid))
                                })
                        })
                        .map(|region| region.base..region.limit + 1)
                        .collect(),
                    Err(_) => Vec::new(),
                };
                planned.push((*segment, index, decision, reserved));
            }
        }
        let found_off = inactive_firmware_units.matches_dmar(&parsed.units);
        let candidates: Vec<Candidate> = planned
            .iter()
            .map(|(segment, _, decision, reserved)| Candidate {
                segment: *segment,
                decision: decision.clone(),
                // A blocked start needs the units found off, not turned off.
                blockable: reserved.is_empty() && found_off,
            })
            .collect();
        let units_off = inactive_firmware_units.off_dmar(&parsed.units);
        // A unit on a segment without a captured configuration space is
        // never planned, so never taken.
        let refused = planned.len() < parsed.units.len();
        let modes = initial::modes(&candidates, &requesters, units_off, refused);
        for ((_, index, decision, reserved), mode) in planned.into_iter().zip(modes) {
            let unit = &parsed.units[index];
            let (requesters, mut window) = match (decision, mode) {
                (Ok(requesters), Mode::Blocked) => (requesters, Vec::new()),
                (Ok(requesters), Mode::Window) => (requesters, reserved),
                (_, Mode::Left(Left::Native { segment, rid })) => {
                    output::arguments(format_args!(
                        "HOST IOMMU VT-d unit {:#x} left untranslated: {} attaches only natively, which a taken unit beside the drivers' window refuses\n",
                        unit.base,
                        address(segment, rid)
                    ));
                    continue;
                }
                (_, Mode::Left(Left::Incomplete)) => {
                    output::arguments(format_args!(
                        "HOST IOMMU VT-d unit {:#x} left untranslated: the units form no complete blocked fabric and every unit is off, so the machine runs untranslated\n",
                        unit.base
                    ));
                    continue;
                }
                (Err(why), _) | (_, Mode::Left(Left::Policy(why))) => {
                    output::arguments(format_args!(
                        "HOST IOMMU VT-d unit {:#x} left untranslated: {why:?}\n",
                        unit.base
                    ));
                    continue;
                }
            };
            if mode == Mode::Window {
                window.push(drivers.clone());
            }
            // SAFETY: forwarded; the unit's registers are the mount's.
            match unsafe {
                Self::take_unit(
                    unit,
                    &requesters,
                    &window,
                    parsed.physical_bits,
                    mmio,
                    tsc_hz,
                )
            } {
                Ok(taken) => {
                    let initial = if window.is_empty() {
                        // A blocked new root cannot qualify a previous
                        // firmware root that was disabled without a drain.
                        let result =
                            if inactive_firmware_units.contains(Kind::Vtd, unit.segment, unit.base)
                            {
                                taken.acknowledge_initial_block(&requesters)
                            } else {
                                Err(DmaError::DrainUnavailable)
                            };
                        match result {
                            Ok(cohort) => {
                                covered += requesters.len();
                                cohorts.push((index, cohort));
                                output::arguments(format_args!(
                                    "HOST IOMMU VT-d unit {:#x} blocked and drained {} initial requesters\n",
                                    unit.base,
                                    requesters.len()
                                ));
                            }
                            Err(error) => output::arguments(format_args!(
                                "HOST IOMMU VT-d unit {:#x} blocked cohort not qualified: {error:?}\n",
                                unit.base
                            )),
                        }
                        InitialContexts::Blocked
                    } else {
                        output::arguments(format_args!(
                            "HOST IOMMU VT-d unit {:#x} maps the drivers' DMA pool and {} reserved regions for {} requesters\n",
                            unit.base,
                            window.len() - 1,
                            requesters.len()
                        ));
                        InitialContexts::Window
                    };
                    remappers.taken.push(Taken {
                        descriptor: index,
                        unit: taken,
                        initial,
                    });
                }
                Err(error) => {
                    // A takeover that wrote the unit, or read it on or
                    // unreadable, leaves a state no proof covers.
                    let off = error.leaves_unit_off();
                    uncertain |= !off;
                    output::arguments(format_args!(
                        "HOST IOMMU VT-d unit {:#x} left untranslated: {:?}{}\n",
                        unit.base,
                        error.error,
                        if off { "" } else { ", its state uncertain" }
                    ));
                }
            }
        }
        // The planner deduplicates RIDs inside each unit and refuses
        // overlaps between units. A full count therefore requires each
        // function from the independently checked PCI census exactly once.
        if !cohorts.is_empty()
            && cohorts.len() == parsed.units.len()
            && inactive_firmware_units.matches_dmar(&parsed.units)
            && covered == pci.functions()
        {
            // SAFETY: no PCI driver, controller or AP can write configuration
            // between the original census and this post-drain readback.
            if let Err(error) = unsafe { pci.recheck_stopped(native_mmio::PciScope::Fabric) } {
                output::arguments(format_args!(
                    "HOST IOMMU VT-d blocked PCI fabric changed after drain: {error}\n"
                ));
                return remappers;
            }
            let fabric = BlockedFabric {
                source: mmio,
                units: cohorts,
            };
            // SAFETY: same original exclusive mount context as unit takeover.
            // This startup check publishes no IRQ route. The pins release
            // before later independent IRQ-mode selection.
            match unsafe { fabric.pin_direct_delivery() } {
                Ok(pins) => {
                    drop(pins);
                    remappers.initial_fabric = Some(fabric);
                    output::arguments(format_args!(
                        "HOST IOMMU VT-d blocked PCI fabric qualified: {} units, {covered} functions\n",
                        parsed.units.len()
                    ));
                }
                Err(error) => output::arguments(format_args!(
                    "HOST IOMMU VT-d initial interrupt mode refused: {error:?}\n"
                )),
            }
        } else if !parsed.units.is_empty() {
            output::arguments(format_args!(
                "HOST IOMMU VT-d blocked PCI fabric incomplete: {}/{} units, {covered}/{} functions\n",
                cohorts.len(),
                parsed.units.len(),
                pci.functions()
            ));
        }
        // With every unit off, the policy takes units only for a complete
        // blocked fabric. One that did not qualify would leave the IOAPICs
        // and every native device refused, so the taken units stop and the
        // machine settles untranslated, as when the policy leaves them all.
        if remappers.initial_fabric.is_none() && units_off && !uncertain {
            uncertain = !remappers.retire_taken();
        }
        remappers.settle(units_off && !uncertain);
        remappers
    }

    /// Stop every taken VT-d unit. False when one did not stop: it stays
    /// taken, and its shutdown failure keeps it held.
    fn retire_taken(&mut self) -> bool {
        let dmar = self.dmar.as_ref();
        self.taken.retain(|taken| {
            // SAFETY: no driver has started, so no requester has a domain,
            // and every requester's Bus Master stays stopped since the
            // original census.
            let Err(error) = (unsafe { taken.unit.shut_down() }) else {
                return false;
            };
            output::arguments(format_args!(
                "HOST IOMMU VT-d unit {:#x} did not stop: {error:?}\n",
                dmar.and_then(|dmar| dmar.units.get(taken.descriptor))
                    .map_or(0, |unit| unit.base)
            ));
            true
        });
        self.taken.is_empty()
    }

    /// With no unit taken and every described unit `off` since
    /// ExitBootServices, no translation path, IOTLB or interrupt remapping
    /// sits between any requester and memory: treat the machine as one that
    /// describes no remapping hardware. Otherwise a requester no taken unit
    /// routes still reaches memory untranslated, but the native paths that
    /// need the whole machine's state stay refused.
    fn settle(&mut self, off: bool) {
        if !self.taken.is_empty() || !self.amd.is_empty() {
            return;
        }
        if !off {
            output::arguments(format_args!(
                "HOST IOMMU no unit taken, but not every unit is proven off: native attachments stay refused\n"
            ));
            return;
        }
        self.unremapped = true;
        self.dmar = None;
        output::arguments(format_args!(
            "HOST IOMMU every unit left untranslated and off: device DMA and interrupts are untranslated\n"
        ));
    }

    /// # Safety
    /// Whole native IOAPIC ownership was explicitly handed off. The captured
    /// platform and every current DMA/IRQ mode writer stay exclusive.
    pub(super) unsafe fn install_ioapics(
        &mut self,
        pci: &native_mmio::ContainedPci<'static>,
    ) -> Result<usize, MappingError> {
        let source = pci.source();
        let modes = if let Some(fabric) = self
            .initial_fabric
            .as_ref()
            .filter(|fabric| core::ptr::eq(fabric.source, source))
        {
            // SAFETY: original complete Intel unit cohort and live root.
            unsafe { fabric.pin_direct_delivery() }
                .map_err(|error| {
                    output::arguments(format_args!("HOST IOAPIC direct mode refused: {error:?}\n"));
                    MappingError::Unavailable
                })?
                .into_iter()
                .map(native_mmio::ModePin::Intel)
                .collect()
        } else if self
            .amd_fabric
            .is_some_and(|amd| core::ptr::eq(amd, source))
        {
            self.amd
                .iter()
                .map(|unit| {
                    unit.pin_direct_delivery()
                        .map(native_mmio::ModePin::Amd)
                        .map_err(|error| {
                            output::arguments(format_args!(
                                "HOST IOAPIC AMD-Vi direct mode refused: {error:?}\n"
                            ));
                            MappingError::Unavailable
                        })
                })
                .collect::<Result<Vec<_>, _>>()?
        } else if self.unremapped {
            alloc::vec![native_mmio::ModePin::Unremapped]
        } else {
            return Err(MappingError::Unavailable);
        };
        // SAFETY: forwarded controller handoff and complete original modes.
        let count = unsafe { source.install_ioapics(modes) }?;
        match source.initial_pci_lines() {
            Ok(lines) => {
                self.initial_lines = Some(lines);
                output::arguments(format_args!("HOST PCI initial interrupt paths contained\n"));
            }
            Err(error) => output::arguments(format_args!(
                "HOST PCI initial interrupt containment unavailable: {error:?}\n"
            )),
        }
        Ok(count)
    }

    fn initial_requester(
        &self,
        source: &native_mmio::Source,
        taken: &Taken,
        rid: u16,
        aliases: &[u16],
    ) -> Result<Option<BlockedRequester<MmioRegisters>>, DmaError> {
        match taken.initial {
            InitialContexts::Window => Ok(None),
            InitialContexts::Blocked => self
                .initial_fabric
                .as_ref()
                .ok_or(DmaError::DrainUnavailable)?
                .requester(source, taken.descriptor, rid, aliases)
                .map(Some),
        }
    }

    /// # Safety
    /// As for [`Self::take`].
    unsafe fn take_unit(
        unit: &dmar::Drhd,
        requesters: &[u16],
        window: &[Range<u64>],
        physical_bits: u8,
        mmio: &native_mmio::Source,
        tsc_hz: u64,
    ) -> Result<Arc<Unit<MmioRegisters>>, TakeOverError> {
        let map = || {
            let span = PhysicalSpan::new(unit.base, unit.bytes as u64)
                .map_err(|_| VtdError::RegisterRange)?;
            // SAFETY: the DMAR names this unit's registers, which no driver owns.
            let resource =
                unsafe { mmio.claim_platform(span) }.map_err(|_| VtdError::RegisterRange)?;
            let mapping = mmio
                .authority()
                .map(
                    resource,
                    0,
                    unit.bytes,
                    CacheAttribute::Uncached,
                    Protection::ReadWrite,
                )
                .map_err(|_| VtdError::RegisterRange)?;
            // SAFETY: the unit's exclusive uncached register window is mapped
            // through this boot's shared aperture.
            unsafe { MmioRegisters::from_mapping(mapping, crate::apic::rdtsc, tsc_hz.max(1)) }
        };
        // Mapping the registers writes none of them.
        let registers = map().map_err(|error| TakeOverError {
            error,
            started: false,
        })?;
        // SAFETY: forwarded: the policy resolved the scope and the window
        // holds the reserved regions the covered requesters use.
        unsafe {
            Unit::take_over(
                registers,
                &crate::native_memory::SOURCE,
                requesters,
                physical_bits.min(52),
                window,
            )
        }
    }

    /// Report the faults each taken unit recorded: DMA a device sent outside
    /// its domain.
    pub(super) fn report_faults(&self) {
        for taken in &self.taken {
            match taken.unit.pending_faults() {
                Ok(faults) => {
                    for fault in faults {
                        output::arguments(format_args!(
                            "HOST IOMMU fault requester={:#06x} address={:#x} reason={:#x} {}\n",
                            fault.requester,
                            fault.address,
                            fault.reason,
                            if fault.read { "read" } else { "write" },
                        ));
                    }
                }
                Err(error) => {
                    output::arguments(format_args!("HOST IOMMU faults unreadable: {error:?}\n"))
                }
            }
        }
    }

    /// The taken hardware unit selected by this requester's firmware route.
    /// A routed but untaken unit supplies no requester-domain grant, even
    /// when the parent bridge uses a different taken unit.
    fn vtd(&self, segment: u16, rid: u16) -> Result<Option<&Taken>, VtdError> {
        let Some(dmar) = &self.dmar else {
            return Ok(None);
        };
        let topology = self.topology.borrow();
        let Some((_, present)) = topology.iter().find(|(found, _)| *found == segment) else {
            return Err(VtdError::Unsupported);
        };
        let selected =
            remapping::route(dmar, segment, present, rid).map_err(|_| VtdError::Unsupported)?;
        selected
            .map(|descriptor| {
                self.taken
                    .iter()
                    .find(|taken| taken.descriptor == descriptor)
                    .ok_or(VtdError::Unsupported)
            })
            .transpose()
    }

    fn amd(&self, segment: u16, rid: u16) -> Result<Option<&amd::Taken>, VtdError> {
        let mut selected = None;
        for taken in &self.amd {
            if taken.routes(segment, rid).map_err(|_| VtdError::Busy)? {
                if selected.is_some() {
                    return Err(VtdError::Unsupported);
                }
                selected = Some(taken);
            }
        }
        Ok(selected)
    }

    /// Initial complete PF/VF inventory, kept under the original PCI epoch.
    /// New arrivals require a fresh role inventory before a native claim.
    pub(super) fn function_role(&self, address: Address) -> FunctionRole {
        self.roles.borrow().role(address)
    }

    /// Join the complete adopted subtree and the original bridge's updated
    /// bus numbers before selecting firmware routes or granting native claims.
    pub(super) fn observe_arrivals(
        &self,
        bridge: Address,
        functions: &[Address],
        read32: &mut dyn FnMut(Address, u16) -> Result<Option<u32>, ConfigError>,
    ) -> Result<bool, ConfigError> {
        let fresh = roles::Inventory::capture(functions, &mut *read32).inspect_err(|_| {
            self.roles.borrow_mut().invalidate_topology(bridge);
        })?;
        let mut updates = Vec::with_capacity(functions.len() + 1);
        for address in core::iter::once(bridge).chain(functions.iter().copied()) {
            let observation = (|| {
                let read = |value: Option<u32>| value.filter(|value| *value != u32::MAX);
                let Some(class) = read(read32(address, 8)?) else {
                    return Ok(None);
                };
                let Some(header) = read(read32(address, 12)?) else {
                    return Ok(None);
                };
                let buses = if header >> 16 & 0x7f == 1 {
                    let Some(buses) = read(read32(address, 24)?) else {
                        return Ok(None);
                    };
                    let secondary = (buses >> 8) as u8;
                    let subordinate = (buses >> 16) as u8;
                    if secondary <= address.bus || subordinate < secondary {
                        return Ok(None);
                    }
                    Some((secondary, subordinate))
                } else {
                    None
                };
                Ok(Some(Present {
                    rid: requester(address),
                    class: [(class >> 24) as u8, (class >> 16) as u8, (class >> 8) as u8],
                    bridge: buses,
                }))
            })();
            match observation {
                Ok(Some(observation)) => updates.push(observation),
                Ok(None) => {
                    self.roles.borrow_mut().invalidate_topology(address);
                    return Ok(false);
                }
                Err(error) => {
                    self.roles.borrow_mut().invalidate_topology(address);
                    return Err(error);
                }
            }
        }
        let mut topology = self.topology.borrow_mut();
        let Some((_, present)) = topology
            .iter_mut()
            .find(|(segment, _)| *segment == bridge.segment)
        else {
            self.roles.borrow_mut().invalidate_topology(bridge);
            return Ok(false);
        };
        if !present
            .iter()
            .any(|entry| entry.rid == requester(bridge) && entry.class[..2] == [6, 4])
            || updates[0].class[..2] != [6, 4]
            || updates[0].bridge.is_none()
        {
            self.roles.borrow_mut().invalidate_topology(bridge);
            return Ok(false);
        }
        if updates.iter().any(|update| {
            present
                .iter()
                .any(|entry| entry.rid == update.rid && entry.class != update.class)
        }) {
            self.roles.borrow_mut().invalidate_topology(bridge);
            return Ok(false);
        }
        for update in updates {
            if let Some(entry) = present.iter_mut().find(|entry| entry.rid == update.rid) {
                *entry = update;
            } else {
                present.push(update);
            }
        }
        present.sort_unstable_by_key(|entry| entry.rid);
        let mut roles = self.roles.borrow_mut();
        let previous = roles.issues().len();
        roles.extend(fresh);
        for issue in &roles.issues()[previous..] {
            output::arguments(format_args!(
                "UNSUPPORTED Stela PCI function-role arrival: {issue:?}\n"
            ));
        }
        Ok(functions
            .iter()
            .all(|address| roles.role(*address) != FunctionRole::Unverified))
    }

    /// Whether a taken unit confines the DMA of the function at `address`.
    pub(super) fn translates(&self, address: Address) -> bool {
        let rid = requester(address);
        self.amd(address.segment, rid)
            .ok()
            .flatten()
            .is_some_and(|taken| taken.covers(address.segment, rid))
            || self
                .vtd(address.segment, rid)
                .ok()
                .flatten()
                .is_some_and(|taken| taken.unit.allows(rid))
    }

    /// Confinement for every possible future child, rather than only the
    /// bridge's own RID. A still-untranslated explicit child unit cannot be
    /// hidden by an active include-all unit that happens to route its parent.
    pub(super) fn translates_subtree(&self, address: Address) -> bool {
        let topology = self.topology.borrow();
        let Some((_, present)) = topology
            .iter()
            .find(|(segment, _)| *segment == address.segment)
        else {
            return false;
        };
        let rid = requester(address);
        let Some((first, last)) = present
            .iter()
            .find(|entry| entry.rid == rid)
            .and_then(|entry| entry.bridge)
        else {
            return false;
        };
        if !self.amd.is_empty() {
            return (first..=last).all(|bus| {
                (0..=255u16).all(|function| {
                    self.amd(address.segment, u16::from(bus) << 8 | function)
                        .is_ok_and(|taken| taken.is_some())
                })
            });
        }
        let Some(dmar) = &self.dmar else {
            return false;
        };
        remapping::subtree_routes(dmar, address.segment, present, rid).is_some_and(|routes| {
            !routes.is_empty()
                && routes.iter().all(|descriptor| {
                    self.taken
                        .iter()
                        .any(|taken| taken.descriptor == *descriptor)
                })
        })
    }

    /// Admit a newly observed requester through its own firmware route.
    /// Initial blocked entries stay blocked until the driver's stopped receipt
    /// authorizes private attachment; parent coverage grants no child route.
    pub(super) fn admit(&self, address: Address) -> Result<(), VtdError> {
        let rid = requester(address);
        if let Some(taken) = self.amd(address.segment, rid)? {
            return taken.admit(rid).map_err(|error| {
                output::arguments(format_args!(
                    "HOST IOMMU AMD-Vi admission refused: {error:?}\n"
                ));
                VtdError::Busy
            });
        }
        self.vtd(address.segment, rid)?
            .map_or(Ok(()), |taken| taken.unit.admit_requester(rid))
    }

    /// The domain of its own for `binding`'s requester when a taken unit
    /// covers it; the binding back when none does.
    pub(super) fn domain(
        &self,
        source: &native_mmio::Source,
        binding: RequesterBinding,
    ) -> Result<Result<Arc<dyn DmaDomain>, VtdError>, RequesterBinding> {
        let binding_address = binding.address();
        let segment = binding_address.segment;
        let rid = binding.requester();
        let amd = match self.amd(segment, rid) {
            Ok(taken) => taken,
            Err(error) => return Ok(Err(error)),
        };
        if let Some(taken) = amd {
            if !taken.covers(segment, rid) {
                return Ok(Err(VtdError::Unsupported));
            }
            return Ok(taken.domain(binding).map_err(|error| {
                output::arguments(format_args!(
                    "HOST IOMMU AMD-Vi attach refused: {error:?}\n"
                ));
                VtdError::Busy
            }));
        }
        let taken = match self.vtd(segment, rid) {
            Ok(Some(taken)) if taken.unit.allows(rid) => taken,
            Ok(Some(_)) | Err(_) => return Ok(Err(VtdError::Unsupported)),
            Ok(None) => return Err(binding),
        };
        // SAFETY: the driver's binding of its stopped function, whose bus
        // mastering it enables after mapping through this domain.
        let claim = match unsafe { RequesterClaim::from_binding(binding) } {
            Ok(claim) => claim,
            Err((error, binding)) => {
                // A failed epoch check returns the SAME retained binding.
                // Refusal cannot authorize releasing its original graph.
                let _kept = core::mem::ManuallyDrop::new(alloc::boxed::Box::new(
                    core::mem::ManuallyDrop::new(binding),
                ));
                return Ok(Err(error));
            }
        };
        // A unit beside the drivers' window claims no alias, so it attaches
        // the requester's own ID alone, as before any requester attached.
        let aliases = match taken.initial {
            InitialContexts::Blocked => self.vtd_aliases(source, binding_address, taken),
            InitialContexts::Window => Ok(Vec::new()),
        };
        let initial = aliases
            .map_err(|_| DmaError::IommuUnavailable)
            .and_then(|aliases| {
                let blocked = self.initial_requester(source, taken, rid, &aliases)?;
                blocked.map_or(Ok(()), |requester| requester.drain())?;
                Ok(aliases)
            });
        let aliases = match initial {
            Ok(aliases) => aliases,
            Err(error) => {
                output::arguments(format_args!(
                    "HOST IOMMU VT-d requester {rid:#06x} initial handoff refused: {error:?}\n"
                ));
                let _kept = core::mem::ManuallyDrop::new(claim);
                return Ok(Err(match error {
                    DmaError::InvalidationFailed => VtdError::Invalidation,
                    _ => VtdError::Unsupported,
                }));
            }
        };
        // SAFETY: the same checked stopped claim and original unit remain
        // held, and the blocked claim covered each alias.
        let owner = match unsafe { taken.unit.attach_requester(claim, &aliases) } {
            Ok(owner) => owner,
            Err(failure) => return Ok(Err(failure.error)),
        };
        let domain = owner.domain();
        // The serialized service keeps affine config custody. The returned
        // DMA projection contains no command/configuration callback.
        self.requesters.borrow_mut().push(owner);
        Ok(Ok(domain as Arc<dyn DmaDomain>))
    }

    /// The other requester IDs the DMA of the function at `address` may
    /// arrive under ([`native_mmio::Source::requester_aliases`]), each
    /// routed to the same `taken` unit and allowed there. An ID no function
    /// has is admitted: no function uses it, so its context entry only
    /// ever carries this requester's DMA once the requester claims it.
    fn vtd_aliases(
        &self,
        source: &native_mmio::Source,
        address: Address,
        taken: &Taken,
    ) -> Result<Vec<u16>, native_mmio::ContainmentError> {
        let aliases = source.requester_aliases(address)?;
        for &alias in &aliases {
            let same = matches!(
                self.vtd(address.segment, alias),
                Ok(Some(unit)) if Arc::ptr_eq(&unit.unit, &taken.unit)
            );
            if !same || taken.unit.admit_requester(alias).is_err() {
                return Err(native_mmio::ContainmentError::bridge_alias(address));
            }
        }
        Ok(aliases)
    }
}
