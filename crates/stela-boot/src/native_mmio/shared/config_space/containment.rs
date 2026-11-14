//! Retain the original PCI fabric after disabling each captured function.
use super::*;
use alloc::rc::Rc;
use core::{any::Any, cell::RefCell, cmp::Reverse, fmt};
use stela_boot_core::ecam::topology;
use stela_drivers::bus::dma::mapping::DmaError;
use stela_drivers::{
    bus::pci::{
        ConfigEffect, ConfigError, ConfigFailure,
        owned::{
            ConfigParts, OwnedConfig, PreparedConfig, UnpublishedConfig,
            grant::{
                CompletionCheck, ConfigCustody, ConfigEpoch, ConfigOperation, DriverCustody,
                DriverGrant, Operation, PrepublicationCustody, PrepublicationGrant,
            },
        },
        regs::{COMMAND, COMMAND_MASTER},
        requester::{
            ClaimError,
            containment::{Plan, requester_aliases},
            wait_for_pending_transactions,
        },
    },
    sync::{FaultState, FirstFault},
};

mod bus_reset;

/// The captured native fabric with PCI command, interrupt and requester
/// controls contained and its original writable maps retired.
pub(crate) struct ContainedPci<'a> {
    source: &'a Source,
    functions: usize,
}

impl<'a> ContainedPci<'a> {
    pub(crate) fn source(&self) -> &'a Source {
        self.source
    }

    pub(crate) fn functions(&self) -> usize {
        self.functions
    }

    /// The first function refused at containment whose Bus Master bit did
    /// not read clear. It may master DMA that nothing here stopped, so
    /// nothing that relies on every function being stopped may start.
    pub(crate) fn unstopped_master(&self) -> Option<Address> {
        match &*self.source.pci_slots.0.borrow() {
            Cohort::Parked(fabric) => fabric.unstopped_master,
            _ => None,
        }
    }

    /// Original bridges with an assigned secondary bus, in upstream-first
    /// order, whether or not functions sit below them.
    pub(crate) fn bridges(&self) -> Result<Vec<Address>, Error> {
        let cohort = self
            .source
            .pci_slots
            .0
            .try_borrow()
            .map_err(|_| Error::Busy)?;
        let Cohort::Parked(fabric) = &*cohort else {
            return Err(Error::Unavailable);
        };
        let mut bridges = Vec::new();
        for slot in &fabric.slots {
            if slot.refusal.is_none()
                && slot
                    .header
                    .bridge
                    .and_then(|buses| topology::secondary_bus(slot.address, buses))
                    .is_some()
            {
                bridges.try_reserve(1).map_err(|_| Error::Exhausted)?;
                bridges.push(slot.address);
            }
        }
        Ok(bridges)
    }

    /// Compare firmware's framebuffer association with the same parked
    /// function and its current BAR. This grants no reset or scanout access.
    ///
    /// # Safety
    /// The caller keeps the original configuration writers excluded.
    pub(crate) unsafe fn check_framebuffer(
        &self,
        source: crate::pci_io::FramebufferSource,
    ) -> Result<(), Error> {
        // SAFETY: the original source and its configuration exclusion are kept.
        unsafe {
            self.source.with_config_inventory(|_, reader| {
                for (offset, expected) in source.bar.words() {
                    let observed = reader.read32(source.address, offset)?;
                    if offset < 0x10 {
                        self.check_observation(source.address, offset, observed)?;
                    }
                    if expected != observed {
                        return Err(Error::InvalidPlatformReply);
                    }
                }
                Ok(())
            })
        }?;
        self.source
            .framebuffer
            .set(Ok(crate::pci_io::ScreenOwner::Function(source)))
            .map_err(|_| Error::Busy)
    }

    pub(crate) fn check_observation(
        &self,
        address: Address,
        offset: u16,
        value: u32,
    ) -> Result<(), Error> {
        let cohort = self.source.pci_slots.0.borrow();
        let Cohort::Parked(fabric) = &*cohort else {
            return Err(Error::Unavailable);
        };
        let index = fabric
            .slots
            .binary_search_by_key(&address, |slot| slot.address)
            .map_err(|_| Error::OutsideResource)?;
        if fabric.slots[index].header.word(offset) != Some(value) {
            return Err(Error::InvalidPlatformReply);
        }
        Ok(())
    }

    /// The dword at `offset` of `address` for a read-only inventory of this
    /// fabric through `reader`: None where the census left the function or
    /// its bus out (`census::LeftOut`), as it read them, else the value,
    /// which must match the parked header where the census held a function.
    /// A function that answers where the census held none refuses, naming
    /// its address.
    pub(crate) fn inventory_read(
        &self,
        reader: &mut ConfigInventory<'_>,
        address: Address,
        offset: u16,
    ) -> Result<Option<u32>, ContainmentError> {
        let mut read = || {
            if !core::ptr::eq(self.source, reader.source) {
                return Err(Error::InvalidPlatformReply);
            }
            {
                let cohort = self.source.pci_slots.0.borrow();
                let Cohort::Parked(fabric) = &*cohort else {
                    return Err(Error::Unavailable);
                };
                if fabric.left_out.holds(address) {
                    return Ok(None);
                }
            }
            let value = reader.read32(address, offset)?;
            if offset != 0 || value as u16 != u16::MAX {
                self.check_observation(address, offset, value)?;
            }
            Ok(Some(value))
        };
        read().map_err(|error| ContainmentError::from(error).at(address))
    }

    /// Observe the parked functions in `scope` through the same read-only
    /// census.
    pub(crate) fn check_stopped(
        &self,
        reader: &mut ConfigInventory<'_>,
        scope: PciScope,
    ) -> Result<(), ContainmentError> {
        if !core::ptr::eq(self.source, reader.source) {
            return Err(Error::InvalidPlatformReply.into());
        }
        let cohort = self.source.pci_slots.0.borrow();
        let Cohort::Parked(fabric) = &*cohort else {
            return Err(Error::Unavailable.into());
        };
        for slot in fabric.scoped(scope)? {
            slot.check_stopped(|offset, width| {
                reader
                    .read(slot.address, offset, width)
                    .map_err(mapping_error)
            })?;
        }
        Ok(())
    }

    /// Recheck the parked functions in `scope` one page at a time,
    /// borrowing any already kept remapper configuration through its own
    /// original read-only owner.
    ///
    /// # Safety
    /// Caller keeps the original writer exclusion. `kept` returns a reader
    /// only for the exact original remapper function; other pages are disjoint.
    pub(crate) unsafe fn recheck_stopped_scoped(
        &self,
        scope: PciScope,
        mut kept: impl FnMut(Address, u16, u8) -> Option<Result<u32, ConfigError>>,
    ) -> Result<(), ContainmentError> {
        let cohort = self.source.pci_slots.0.borrow();
        let Cohort::Parked(fabric) = &*cohort else {
            return Err(Error::Unavailable.into());
        };
        for slot in fabric.scoped(scope)? {
            let mut map = None;
            let checked = slot.check_stopped(|offset, width| {
                if let Some(value) = kept(slot.address, offset, width) {
                    return value;
                }
                if map.is_none() {
                    let (_, span) = self
                        .source
                        .config_page(slot.address)
                        .map_err(mapping_error)?;
                    // SAFETY: this exact original slot has no other current
                    // observation owner. Its sole read-only view is scoped.
                    map = Some(
                        unsafe { ConfigScope::open(self.source, span) }.map_err(mapping_error)?,
                    );
                }
                map.as_ref()
                    .expect("scoped function map")
                    .read(usize::from(offset), u16::from(width))
                    .map_err(mapping_error)
            });
            if let Some(mut map) = map {
                // SAFETY: all observations returned synchronously.
                unsafe { map.retire() }
                    .map_err(|error| ContainmentError::from(error).at(slot.address))?;
            }
            checked?;
        }
        Ok(())
    }

    /// Reopen a read-only view of the parked functions in `scope`, after
    /// the remapping units have drained or before a bridge forwards.
    ///
    /// # Safety
    /// The original source still excludes every configuration and topology writer.
    pub(crate) unsafe fn recheck_stopped(&self, scope: PciScope) -> Result<(), ContainmentError> {
        // SAFETY: the caller retains the original exclusive census ownership.
        unsafe {
            self.source
                .with_config_inventory(|_, reader| self.check_stopped(reader, scope))
        }
    }
}

/// The parked functions a recheck reads.
#[derive(Clone, Copy)]
pub(crate) enum PciScope {
    /// Every function of the fabric.
    Fabric,
    /// One bridge and the functions on its secondary bus: what turning on
    /// its forwarding exposes. A bridge on that bus stays closed until its
    /// own forwarding is restored with its own bus rechecked, so a function
    /// that changed below it withholds only that branch.
    Bridge(Address),
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct ContainmentError {
    address: Option<Address>,
    cause: Cause,
    effect: ConfigEffect,
}

#[derive(Clone, Copy, Debug)]
enum Cause {
    Claim(ClaimError),
    Topology(topology::Error),
    BridgeForwarding,
    BridgeAlias,
    /// A bridge whose bus dword is not used (`census::admit`): it and the
    /// buses behind it stay undriven.
    BridgeBuses {
        buses: u32,
        reason: census::Misconfigured,
    },
}

/// A plain sentence for the boot log: the function, what failed and what it
/// means for containment.
impl fmt::Display for ContainmentError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if let Some(address) = self.address {
            write!(f, "{address}: ")?;
        }
        match self.cause {
            Cause::Claim(ClaimError::Mapping(error)) if self.address.is_none() => {
                write!(
                    f,
                    "the PCI configuration space cannot be mapped ({error:?})"
                )
            }
            Cause::Claim(ClaimError::Config(error)) => {
                write!(f, "{error}")?;
                match error.reason {
                    ConfigFailure::Readback {
                        offset: COMMAND,
                        expected,
                        observed,
                        ..
                    } if (expected ^ observed) & u32::from(COMMAND_MASTER) != 0 => f.write_str(
                        "; the function keeps bus mastering on, so Stela cannot hold it stopped",
                    ),
                    ConfigFailure::Readback {
                        offset: COMMAND, ..
                    } => f.write_str(
                        "; the function does not hold INTx Disable, so its INTx cannot be masked \
                         at the function",
                    ),
                    ConfigFailure::Readback {
                        offset: BRIDGE_CONTROL,
                        ..
                    } => f.write_str(
                        "; its Secondary Bus Reset is set, so the buses below it stay in reset",
                    ),
                    _ => Ok(()),
                }
            }
            Cause::Claim(cause) => {
                write!(f, "{cause}")?;
                if let ConfigEffect::WriteMayHaveApplied { offset, .. } = self.effect {
                    write!(f, " after Stela wrote its register {offset:#x}")?;
                }
                Ok(())
            }
            Cause::Topology(cause) => write!(f, "the PCI topology is inconsistent ({cause:?})"),
            Cause::BridgeForwarding => f.write_str("upstream bridge forwarding is unavailable"),
            Cause::BridgeAlias => f.write_str(
                "a bridge above it may forward its DMA under another requester ID, which \
                 its remapping unit cannot give the function's domain",
            ),
            Cause::BridgeBuses { buses, reason } => {
                let [_, secondary, subordinate, _] = buses.to_le_bytes();
                write!(
                    f,
                    "the bridge {} (secondary bus {secondary:#04x}, subordinate bus \
                     {subordinate:#04x})",
                    reason.message()
                )?;
                if let census::Misconfigured::Overlaps(other) = reason {
                    write!(f, " at {other}")?;
                }
                f.write_str(", so the buses behind it are not scanned")
            }
        }
    }
}

/// The requester ID of the function at `address` on its segment.
fn requester_id(address: Address) -> u16 {
    u16::from(address.bus) << 8 | u16::from(address.device) << 3 | u16::from(address.function)
}

impl ContainmentError {
    /// The function at `address`, whose DMA may arrive under requester IDs
    /// of a bridge above it ([`Source::requester_aliases`]), cannot have
    /// each of them in its domain.
    pub(crate) fn bridge_alias(address: Address) -> Self {
        Self {
            address: Some(address),
            cause: Cause::BridgeAlias,
            effect: ConfigEffect::None,
        }
    }
}

impl From<ClaimError> for ContainmentError {
    fn from(cause: ClaimError) -> Self {
        let effect = match cause {
            ClaimError::Config(error) => error.effect,
            _ => ConfigEffect::None,
        };
        Self {
            address: None,
            cause: Cause::Claim(cause),
            effect,
        }
    }
}

impl From<topology::Error> for ContainmentError {
    fn from(cause: topology::Error) -> Self {
        Self {
            address: None,
            cause: Cause::Topology(cause),
            effect: ConfigEffect::None,
        }
    }
}

impl From<Error> for ContainmentError {
    fn from(error: Error) -> Self {
        ClaimError::Mapping(error).into()
    }
}

impl From<ConfigError> for ContainmentError {
    fn from(error: ConfigError) -> Self {
        ClaimError::Config(error).into()
    }
}

impl ContainmentError {
    fn at(mut self, address: Address) -> Self {
        self.address = Some(address);
        self
    }

    /// Whether this refusal concerns the function alone, as Linux carries
    /// on past each: a stop control that does not read back (INTx Disable,
    /// Bus Master, a capability's enable, Secondary Bus Reset), a control
    /// left in a state Stela does not change, or transactions still
    /// pending after the wait, or a capability its plan left as found
    /// (`Plan::refusal`), or a bridge whose bus numbers are not used. A
    /// configuration space that cannot be mapped or owned, or a header that
    /// changed, still stops the whole fabric.
    fn per_function(&self) -> bool {
        match self.cause {
            Cause::Claim(
                ClaimError::Active { .. }
                | ClaimError::MalformedCapability { .. }
                | ClaimError::UnsupportedCapability { .. }
                | ClaimError::UnsupportedVersion { .. },
            ) => true,
            Cause::Claim(ClaimError::Config(error)) => match error.reason {
                ConfigFailure::Readback { offset, .. } => !matches!(offset, 0 | 8 | 0x0c | 0x18),
                ConfigFailure::BusMasterSealed | ConfigFailure::UnqualifiedControl => true,
                _ => false,
            },
            Cause::BridgeBuses { .. } => true,
            _ => false,
        }
    }
}

struct Slot {
    address: Address,
    configuration: Option<PreparedConfig<PrepublicationGrant>>,
    refusal: Option<SlotRefusal>,
    header: Header,
    plan: Rc<Plan>,
    ancestor: Parent,
    route: topology::Parent,
    /// A bridge stopped, sealed and refused alone (`Cause::BridgeBuses`).
    misconfigured: Option<census::Misconfigured>,
    forwarding: Option<Forwarding>,
    /// The writes a bus reset admits through this parked function's
    /// custody: its state saved before the reset, each written back once.
    restore: Rc<RefCell<Vec<(u16, u8, u32)>>>,
}

impl Slot {
    fn check_stopped(
        &self,
        mut read: impl FnMut(u16, u8) -> Result<u32, ConfigError>,
    ) -> Result<(), ContainmentError> {
        // A refused function is never driven. Its stop was judged when it
        // was refused (see `unstopped_master`); only its place is rechecked.
        if self.refusal.is_some() {
            return self
                .header
                .verify(|offset| read(offset, 4))
                .map_err(|error| ContainmentError::from(error).at(self.address));
        }
        self.header
            .verify(|offset| read(offset, 4))
            .and_then(|()| {
                self.plan
                    .check_disabled(|offset| read(offset, 2).map(|word| word as u16))
            })
            .and_then(|()| {
                if self.header.bridge.is_some() {
                    let prepared = self
                        .configuration
                        .as_ref()
                        .ok_or_else(|| mapping_error(Error::Unavailable))?;
                    prepared.check_bridge_routing(read)?;
                }
                Ok(())
            })
            .map_err(|error| ContainmentError::from(error).at(self.address))
    }
}

enum Forwarding {
    Open {
        config: OwnedConfig,
        blocked: Box<dyn Fn() -> Result<(), DmaError>>,
        command: u16,
    },
    Refused {
        _original: ConfigParts<DriverGrant>,
        _blocked: Box<dyn Fn() -> Result<(), DmaError>>,
    },
}

impl Forwarding {
    fn check(&self) -> Result<(), ContainmentError> {
        let Self::Open {
            config,
            blocked,
            command,
        } = self
        else {
            return Err(Error::Unavailable.into());
        };
        blocked().map_err(|_| Error::Unavailable)?;
        check_command(config, *command, ConfigEffect::None)?;
        Ok(())
    }
}

/// Type 1 Bridge Control, whose Secondary Bus Reset a bridge may hold.
const BRIDGE_CONTROL: u16 = 0x3e;

/// How long a stopped function may keep Transactions Pending set: Linux's
/// `pci_wait_for_pending_transaction` polls at 0, 100, 300 and 700 ms and
/// then carries on.
const PENDING_TRANSACTIONS_MS: u64 = 700;

fn check_command(
    config: &OwnedConfig,
    expected: u16,
    effect: ConfigEffect,
) -> Result<(), ConfigError> {
    let observed = config.try_read16(COMMAND).map_err(|mut error| {
        if effect != ConfigEffect::None {
            error.effect = effect;
        }
        error
    })?;
    if observed == expected {
        return Ok(());
    }
    Err(ConfigError {
        reason: ConfigFailure::Readback {
            offset: COMMAND,
            width: 2,
            expected: expected.into(),
            observed: observed.into(),
        },
        effect,
    })
}

#[derive(Default)]
struct Fabric {
    roots: Vec<Option<Assignment<RootBus>>>,
    slots: Vec<Slot>,
    refusal: Option<SlotRefusal>,
    /// See [`ContainedPci::unstopped_master`].
    unstopped_master: Option<Address>,
    /// What answered at the census but is not in `slots`.
    left_out: census::LeftOut,
}

impl Fabric {
    fn scoped(&self, scope: PciScope) -> Result<impl Iterator<Item = &Slot>, ContainmentError> {
        let exposed = match scope {
            PciScope::Fabric => None,
            PciScope::Bridge(address) => {
                let index = self
                    .slots
                    .binary_search_by_key(&address, |slot| slot.address)
                    .map_err(|_| ContainmentError::from(Error::OutsideResource).at(address))?;
                let secondary = self.slots[index]
                    .header
                    .bridge
                    .and_then(|buses| topology::secondary_bus(address, buses))
                    .ok_or_else(|| ContainmentError::from(Error::OutsideResource).at(address))?;
                Some((address, secondary))
            }
        };
        Ok(self.slots.iter().filter(move |slot| {
            exposed.is_none_or(|(bridge, secondary)| {
                topology::exposed_by(bridge, secondary, slot.address)
            })
        }))
    }

    /// The requester IDs other than its own that the DMA of the function
    /// at `address` may arrive under, through each bridge on its `route`
    /// that forwards under an ID of its own.
    fn requester_aliases(
        &self,
        address: Address,
        mut route: topology::Parent,
    ) -> Result<Vec<u16>, ContainmentError> {
        let mut forwarded = Vec::new();
        while let topology::Parent::Bridge(bridge) = route {
            let index = self
                .slots
                .binary_search_by_key(&bridge, |slot| slot.address)
                .expect("original parent remains in the fabric");
            let slot = &self.slots[index];
            let secondary = slot
                .header
                .bridge
                .and_then(|buses| topology::secondary_bus(bridge, buses))
                .ok_or_else(|| ContainmentError::from(Error::OutsideResource).at(bridge))?;
            forwarded.extend(
                slot.plan
                    .forwarded_requester(requester_id(bridge), secondary),
            );
            route = slot.route;
        }
        Ok(requester_aliases(requester_id(address), forwarded))
    }

    fn check_upstream(&self, mut route: topology::Parent) -> Result<(), ContainmentError> {
        while let topology::Parent::Bridge(address) = route {
            let index = self
                .slots
                .binary_search_by_key(&address, |slot| slot.address)
                .expect("original parent remains in the fabric");
            let bridge = &self.slots[index];
            bridge
                .forwarding
                .as_ref()
                .ok_or(ContainmentError {
                    address: Some(address),
                    cause: Cause::BridgeForwarding,
                    effect: ConfigEffect::None,
                })?
                .check()
                .map_err(|error| error.at(address))?;
            route = bridge.route;
        }
        Ok(())
    }
}

struct SlotRefusal {
    error: ContainmentError,
    /// Judged when containment refuses it: its Bus Master bit read clear,
    /// or hard-wired as its plan allows.
    master_stopped: bool,
    _original: ConfigParts<PrepublicationGrant>,
}

impl SlotRefusal {
    fn new(error: impl Into<ContainmentError>, original: ConfigParts<PrepublicationGrant>) -> Self {
        original.close_admission();
        Self {
            error: error.into(),
            master_stopped: false,
            _original: original,
        }
    }

    /// Refuse a function that is stopped and sealed, whose plan left a
    /// capability's control as found or whose bus numbers are not used.
    fn left(
        cause: impl Into<ContainmentError>,
        parked: PreparedConfig<PrepublicationGrant>,
    ) -> Self {
        Self {
            master_stopped: true,
            ..Self::new(cause, parked.into_parts())
        }
    }

    /// Refuse a function whose stop did not complete, reading COMMAND once
    /// to judge whether it can still master DMA.
    fn unstopped(
        error: impl Into<ContainmentError>,
        config: UnpublishedConfig,
        plan: &Plan,
    ) -> Self {
        let master_stopped = config
            .try_read16(COMMAND)
            .is_ok_and(|command| plan.master_stopped(command));
        Self {
            master_stopped,
            ..Self::new(error, config.into_parts())
        }
    }
}

enum Cohort {
    Unclaimed,
    Incomplete(Fabric),
    Parked(Fabric),
}

struct RootBus {
    _descriptor: &'static RootWindow,
    platform: Arc<Platform>,
    aperture: Parent,
}

// SAFETY: the descriptor comes from the retained capture. Each unique root
// bus assignment is a child of that capture's aperture owner.
unsafe impl PhysicalAssignment for RootBus {
    fn parent(&self) -> Option<&Parent> {
        Some(&self.aperture)
    }

    fn check_control(&self) -> Result<(), assignment::Error> {
        self.platform
            .check_mapping_control()
            .map_err(|_| assignment::Error::Context)
    }
}

pub(in crate::native_mmio::shared) struct PciSlots(RefCell<Cohort>);

impl PciSlots {
    pub(in crate::native_mmio::shared) const fn new() -> Self {
        Self(RefCell::new(Cohort::Unclaimed))
    }
}

#[derive(Clone, Copy)]
struct Header {
    identity: u32,
    class_revision: u32,
    layout: u32,
    bridge: Option<u32>,
}

impl Header {
    fn word(self, offset: u16) -> Option<u32> {
        match offset {
            0 => Some(self.identity),
            8 => Some(self.class_revision),
            0x0c => Some(self.layout),
            0x18 => self.bridge,
            _ => None,
        }
    }

    fn verify(
        self,
        mut read: impl FnMut(u16) -> Result<u32, ConfigError>,
    ) -> Result<(), ConfigError> {
        for offset in [0, 8, 0x0c, 0x18] {
            let Some(expected) = self.word(offset) else {
                continue;
            };
            let observed = read(offset)?;
            if observed != expected {
                return Err(ConfigError::before_effect(ConfigFailure::Readback {
                    offset,
                    width: 4,
                    expected,
                    observed,
                }));
            }
        }
        Ok(())
    }
}

struct Input {
    address: Address,
    window: EcamWindow,
    span: PhysicalSpan,
    header: Header,
    plan: Rc<Plan>,
    /// A bridge whose bus dword is not used: `header.bridge` keeps the
    /// dword as found, the topology sees no forwarding, and containment
    /// refuses it alone once it is stopped.
    misconfigured: Option<census::Misconfigured>,
}

enum Observed {
    /// Nothing answers.
    Absent,
    /// Something answers that the census reads as no function
    /// (`census::LeftOut`).
    Skipped,
    Function(Input),
    /// A function the fabric does not drive, left idle with the buses
    /// behind it, and its layout dword.
    Left {
        layout: u32,
        behind: Option<RangeInclusive<u8>>,
    },
}

/// Whether `candidate` repeats single-function `first` (census::alias).
fn repeats(
    reader: &mut ConfigInventory<'_>,
    first: Address,
    candidate: Address,
) -> Result<bool, Error> {
    let mut words = [[0; 9]; 2];
    for (at, offset) in census::ALIAS_OFFSETS.into_iter().enumerate() {
        words[0][at] = reader.read32(first, offset)?;
        words[1][at] = reader.read32(candidate, offset)?;
    }
    Ok(census::alias(&words[0], &words[1]))
}

impl Input {
    /// `root_bus`: no bridge the census found forwards to `address`'s bus.
    /// `single`: function 0 of this device, when it has no multifunction bit.
    /// `admitted`: the bridges admitted so far with their buses.
    fn observe(
        reader: &mut ConfigInventory<'_>,
        address: Address,
        root_bus: bool,
        single: Option<Address>,
        admitted: &[(Address, RangeInclusive<u8>)],
    ) -> Result<Observed, ContainmentError> {
        let identity = reader.read32(address, 0)?;
        // A function without a vendor is skipped only while its other
        // registers show it cannot reach memory or raise INTx.
        match census::found(identity, || reader.read32(address, 4))? {
            census::Found::Absent => return Ok(Observed::Absent),
            census::Found::Skipped(oddity) => {
                report_oddity(oddity, address);
                return Ok(Observed::Skipped);
            }
            census::Found::ActiveWithoutVendor => {
                return Err(topology::Error::ActiveWithoutVendor(address).into());
            }
            census::Found::Present => {}
        }
        if let Some(first) = single
            && repeats(reader, first, address)?
        {
            report_oddity(census::Oddity::Alias, address);
            return Ok(Observed::Skipped);
        }
        let class_revision = reader.read32(address, 8)?;
        if class_revision == u32::MAX {
            return Err(Error::InvalidPlatformReply.into());
        }
        let layout = reader.read32(address, 0x0c)?;
        let kind = census::layout(layout);
        if census::bridge_class(class_revision) != (kind == census::Layout::Bridge) {
            report_oddity(census::Oddity::BridgeClass, address);
        }
        let mut misconfigured = None;
        let bridge = match kind {
            census::Layout::Endpoint => None,
            census::Layout::Bridge => {
                let buses = reader.read32(address, 0x18)?;
                misconfigured = reader.check_bridge(address, buses, admitted).err();
                Some(buses)
            }
            census::Layout::CardBus | census::Layout::Unknown(_) => {
                // Not driven: there is no CardBus driver, and Linux ignores
                // an unknown header. Left alone only while it cannot reach
                // memory or raise INTx.
                if !census::quiet(reader.read32(address, 4)?) {
                    return Err(topology::Error::ActiveUnsupportedHeader(address).into());
                }
                report_oddity(census::Oddity::UnsupportedHeader, address);
                // A CardBus bridge shares the Type 1 bus fields. With Bus
                // Master clear it forwards no memory request from behind it.
                // Buses it cannot own stay in the scan.
                let behind = if kind == census::Layout::CardBus {
                    let buses = reader.read32(address, 0x18)?;
                    reader.check_bridge(address, buses, admitted).ok().flatten()
                } else {
                    None
                };
                return Ok(Observed::Left { layout, behind });
            }
        };
        let plan = Plan::read(
            |offset, width| {
                reader
                    .read(address, offset, width)
                    .map_err(ClaimError::Mapping)
            },
            root_bus,
        )?;
        let (window, span) = reader
            .geometry
            .function(address, reader.source.platform.wire().roots())?;
        Ok(Observed::Function(Self {
            address,
            window,
            span,
            header: Header {
                identity,
                class_revision,
                layout,
                bridge,
            },
            plan: Rc::new(plan),
            misconfigured,
        }))
    }
}

fn census(
    windows: &[EcamWindow],
    reader: &mut ConfigInventory<'_>,
) -> Result<(Vec<Input>, census::LeftOut), ContainmentError> {
    let mut inputs: Vec<Input> = Vec::new();
    // What answers but is not in `inputs`, for a later inventory.
    let mut left_out = census::LeftOut::default();
    // Every bridge admitted so far with the buses it forwards, CardBus
    // bridges included (`census::admit`).
    let mut admitted: Vec<(Address, RangeInclusive<u8>)> = Vec::new();
    // Buses behind a function left undriven: a CardBus or misconfigured
    // bridge.
    let mut left: Vec<RangeInclusive<u8>> = Vec::new();
    for window in windows {
        for bus in window.start_bus..=window.end_bus {
            // A bridge's secondary bus is above its own, so every bridge
            // that forwards to `bus` is already in the census. Only
            // admitted Type 1 bridges are driven; a CardBus bridge's buses
            // are in `left`.
            let root_bus = !inputs.iter().any(|input| {
                input.address.segment == window.segment
                    && input.misconfigured.is_none()
                    && input
                        .header
                        .bridge
                        .and_then(|buses| census::forwarded(input.address.bus, buses))
                        .is_some_and(|buses| buses.contains(&bus))
            });
            // A bus a driven bridge forwards to is scanned even when a
            // bridge left undriven claims it too, so every function there
            // is stopped.
            if root_bus && left.iter().any(|buses| buses.contains(&bus)) {
                left_out
                    .bus(window.segment, bus)
                    .map_err(|_| Error::Exhausted)?;
                continue;
            }
            for device in 0..32 {
                let mut single = None;
                for function in 0..8 {
                    let address = Address {
                        segment: window.segment,
                        bus,
                        device,
                        function,
                    };
                    // Linux reads functions 1-7 only of a multifunction
                    // device. Here they are read, and dropped only when
                    // they repeat function 0: ARI and nonconforming
                    // functions can reach memory too.
                    let observed = Input::observe(reader, address, root_bus, single, &admitted)
                        .map_err(|error| error.at(address))?;
                    let layout = match observed {
                        Observed::Absent => continue,
                        Observed::Skipped => {
                            left_out.function(address).map_err(|_| Error::Exhausted)?;
                            continue;
                        }
                        Observed::Function(input) => {
                            let layout = input.header.layout;
                            let behind = input
                                .header
                                .bridge
                                .and_then(|buses| census::forwarded(address.bus, buses));
                            if let Some(buses) = behind {
                                if input.misconfigured.is_some() {
                                    left.try_reserve(1).map_err(|_| Error::Exhausted)?;
                                    left.push(buses);
                                } else {
                                    admitted.try_reserve(1).map_err(|_| Error::Exhausted)?;
                                    admitted.push((address, buses));
                                }
                            }
                            inputs.try_reserve(1).map_err(|_| Error::Exhausted)?;
                            inputs.push(input);
                            layout
                        }
                        Observed::Left { layout, behind } => {
                            left_out.function(address).map_err(|_| Error::Exhausted)?;
                            if let Some(buses) = behind {
                                admitted.try_reserve(1).map_err(|_| Error::Exhausted)?;
                                admitted.push((address, buses.clone()));
                                left.try_reserve(1).map_err(|_| Error::Exhausted)?;
                                left.push(buses);
                            }
                            layout
                        }
                    };
                    if function == 0 && !census::multifunction(layout) {
                        single = Some(address);
                    }
                }
            }
        }
    }
    Ok((inputs, left_out))
}

#[derive(Clone)]
struct Epoch {
    platform: Arc<Platform>,
    original: Retention<Reservation>,
}

impl Epoch {
    fn context(&self) -> Result<(), Error> {
        self.platform.check_mapping_control()
    }
}

// SAFETY: this projection only keeps the original aperture and slot count.
// It creates no mapping or configuration access.
unsafe impl ConfigEpoch for Epoch {
    fn check(&self) -> Result<(), Error> {
        self.context()?;
        self.original.check_admission().map_err(Error::Assignment)
    }

    fn retain(&self) -> Result<Box<dyn ConfigEpoch>, Error> {
        self.check()?;
        Ok(Box::new(self.clone()))
    }

    fn same_epoch(&self, other: &dyn ConfigEpoch) -> Result<bool, Error> {
        self.check()?;
        other.check()?;
        Ok(other
            .as_any()
            .downcast_ref::<Self>()
            .is_some_and(|other| self.original.same_assignment(&other.original)))
    }

    fn as_any(&self) -> &dyn Any {
        self
    }
}

struct Completion {
    epoch: Epoch,
    fault: FirstFault<ConfigError>,
}

fn mapping_error(error: Error) -> ConfigError {
    ConfigError::before_effect(ConfigFailure::Mapping(error))
}

// SAFETY: the preowned validator checks the original aperture in the
// accepting context and latches its first failure without reentry.
unsafe impl CompletionCheck for Completion {
    fn postvalidate(&self) -> Result<(), ConfigError> {
        let result = self.epoch.context().map_err(mapping_error);
        if let Err(error) = result {
            self.fault.latch(error);
        }
        result
    }

    fn failure(&self) -> FaultState<ConfigError> {
        self.fault.observe()
    }
}

struct SlotCustody {
    _original: Assignment<Reservation>,
    resource: Resource,
    completion: Arc<Completion>,
    address: Address,
    window: EcamWindow,
    plan: Rc<Plan>,
    driver_lines: Option<InitialPciLines>,
    /// See [`Slot::restore`].
    restore: Rc<RefCell<Vec<(u16, u8, u32)>>>,
}

impl SlotCustody {
    /// Consume the bus reset's admission of exactly this write.
    fn take_restore(&self, offset: u16, width: usize, value: u32) -> bool {
        let mut restore = self.restore.borrow_mut();
        let Some(at) = restore
            .iter()
            .position(|&admitted| admitted == (offset, width as u8, value))
        else {
            return false;
        };
        restore.remove(at);
        true
    }
}

// SAFETY: the census issues one exact slot from the original aperture
// reservation, shared by its resource backing and configuration counts.
unsafe impl ConfigCustody for SlotCustody {
    fn address(&self) -> Address {
        self.address
    }

    fn window(&self) -> EcamWindow {
        self.window
    }

    fn resource(&self) -> &Resource {
        &self.resource
    }

    fn begin(&self, operation: Operation) -> Result<ConfigOperation, ConfigError> {
        self.completion.epoch.context().map_err(mapping_error)?;
        if self.driver_lines.is_none()
            && let Operation::Write {
                offset,
                width,
                value,
            } = operation
            && !self.plan.permits(offset, width, value)
            && !self.take_restore(offset, width, value)
        {
            return Err(ConfigError::before_effect(
                ConfigFailure::UnqualifiedControl,
            ));
        }
        match self.completion.failure() {
            FaultState::Ready(error) => return Err(error),
            FaultState::Publishing => {
                return Err(ConfigError::before_effect(ConfigFailure::FaultPublishing));
            }
            FaultState::Empty => {}
        }
        let count = self
            .completion
            .epoch
            .original
            .begin_use(UseKind::Configuration)
            .map_err(|error| ConfigError::before_effect(ConfigFailure::Ownership(error)))?;
        // SAFETY: the accepted operation retains the exact slot and validator.
        Ok(unsafe { ConfigOperation::new(count.erase(), self.completion.clone()) })
    }

    fn retirement_context(&self) -> Result<(), Error> {
        self.completion.epoch.context()
    }

    fn retain_epoch(&self) -> Result<Box<dyn ConfigEpoch>, Error> {
        self.completion.epoch.retain()
    }
}

// SAFETY: the original exclusive source issues the slot once. Its only
// writes are the exact disabling values planned from the complete census.
unsafe impl PrepublicationCustody for SlotCustody {}

// SAFETY: only promote_parked can turn on driver operations. It joins
// the original parked slot and ancestor assignment to the checked blocked
// requester and original interrupt paths, preserving the BME seal.
unsafe impl DriverCustody for SlotCustody {}

impl Source {
    /// The requester IDs other than its own that the DMA of the parked
    /// function at `address` may arrive under: each a bridge above it
    /// forwards it under, as Linux's `pci_for_each_dma_alias` visits them.
    /// A remapping unit that translates the function must give each one the
    /// function's domain. The routing seal keeps them fixed.
    pub(crate) fn requester_aliases(&self, address: Address) -> Result<Vec<u16>, ContainmentError> {
        let cohort = self.pci_slots.0.try_borrow().map_err(|_| Error::Busy)?;
        let Cohort::Parked(fabric) = &*cohort else {
            return Err(Error::Unavailable.into());
        };
        let index = fabric
            .slots
            .binary_search_by_key(&address, |slot| slot.address)
            .map_err(|_| ContainmentError::from(Error::OutsideResource).at(address))?;
        fabric.requester_aliases(address, fabric.slots[index].route)
    }

    /// Take the original parked endpoint, preserving its grant and negative
    /// operation state. A refusal after taking it keeps the exact envelope in
    /// the same slot and prevents a second attempt.
    ///
    /// # Safety
    /// The caller owns the original complete remapping fabric. The check
    /// observes the exact firmware-selected unit for this segment and RID,
    /// and, on a translated machine, that unit gives the function's domain
    /// each of its [`Self::requester_aliases`]. Prior PCI writers and any
    /// legacy writer or reassignment of the topology stay excluded. This
    /// grants no endpoint stop, root binding or IRQ route.
    pub(crate) unsafe fn promote_cold_config(
        &self,
        address: Address,
        lines: &InitialPciLines,
        check_blocked: impl Fn() -> Result<(), DmaError>,
    ) -> Result<OwnedConfig, ContainmentError> {
        // SAFETY: same original exclusive, blocked requester and line paths.
        unsafe { self.promote_parked(address, lines, check_blocked, false) }
    }

    /// Restore only forwarding through one original bridge of any kind, as
    /// Linux enables any Type 1 bridge above a driven function. Keep the
    /// routing seal, configuration owner and its original blocked context.
    ///
    /// # Safety
    /// Same proof as promote_cold_config, plus the bridge and the functions
    /// on its secondary bus ([`PciScope::Bridge`]) were rechecked stopped
    /// just before this forwarding and all leaf owners remain parked
    /// with their master seals and new DMA roots blocked. The checker owns
    /// this port's original blocked context for as long as the port is kept.
    pub(crate) unsafe fn forward_cold_bridge(
        &self,
        address: Address,
        lines: &InitialPciLines,
        check_blocked: impl Fn() -> Result<(), DmaError> + 'static,
    ) -> Result<(), ContainmentError> {
        let blocked: Box<dyn Fn() -> Result<(), DmaError>> = Box::new(check_blocked);
        // SAFETY: the caller keeps the original stopped descendant cohort,
        // and this selects only the original Type 1 bridge for the exact RID.
        let mut config = unsafe { self.promote_parked(address, lines, &blocked, true) }?;
        let result = (|| {
            let command = config.try_read16(COMMAND)? | COMMAND_MASTER;
            // SAFETY: the whole original fabric is drained and held, with
            // blocked contexts, disabled leaves and no competing writers.
            // The original routing seal stays enforced by the configuration.
            unsafe { config.release_bus_forwarding_seal() }?;
            config.try_write16(COMMAND, command)?;
            check_command(
                &config,
                command,
                ConfigEffect::WriteMayHaveApplied {
                    offset: COMMAND,
                    width: 2,
                },
            )?;
            Ok::<_, ConfigError>(command)
        })();
        let (forwarding, result) = match result {
            Ok(command) => (
                Forwarding::Open {
                    config,
                    blocked,
                    command,
                },
                Ok(()),
            ),
            Err(error) => {
                let original = config.into_parts();
                original.close_admission();
                (
                    Forwarding::Refused {
                        _original: original,
                        _blocked: blocked,
                    },
                    Err(ContainmentError::from(error).at(address)),
                )
            }
        };
        let mut cohort = self.pci_slots.0.borrow_mut();
        let Cohort::Parked(fabric) = &mut *cohort else {
            unreachable!("the original parked fabric stays owned")
        };
        let index = fabric
            .slots
            .binary_search_by_key(&address, |slot| slot.address)
            .expect("forwarding keeps the original slot");
        fabric.slots[index].forwarding = Some(forwarding);
        result
    }

    /// # Safety
    /// Caller holds the matching complete fabric and excludes all old PCI
    /// writers. Bridge promotion additionally requires parked descendants.
    unsafe fn promote_parked(
        &self,
        address: Address,
        lines: &InitialPciLines,
        check_blocked: impl Fn() -> Result<(), DmaError>,
        bridge: bool,
    ) -> Result<OwnedConfig, ContainmentError> {
        let check_paths = || {
            lines.check(self)?;
            check_blocked().map_err(|_| Error::Unavailable)
        };
        check_paths().map_err(|error| ContainmentError::from(error).at(address))?;
        let (index, prepared, header, plan) = {
            let mut cohort = self.pci_slots.0.try_borrow_mut().map_err(|_| Error::Busy)?;
            let Cohort::Parked(fabric) = &mut *cohort else {
                return Err(Error::Unavailable.into());
            };
            let index = fabric
                .slots
                .binary_search_by_key(&address, |slot| slot.address)
                .map_err(|_| Error::OutsideResource)?;
            let slot = &fabric.slots[index];
            if let Some(refusal) = &slot.refusal {
                return Err(refusal.error);
            }
            if (slot.header.layout >> 16) & 0x7f != u32::from(bridge) {
                return Err(ContainmentError::from(Error::OutsideResource).at(address));
            }
            fabric.check_upstream(slot.route)?;
            let slot = &mut fabric.slots[index];
            let prepared = slot.configuration.take().ok_or(Error::Busy)?;
            (index, prepared, slot.header, slot.plan.clone())
        };
        let result = (|| {
            let mapping = match self.authority.map(
                prepared.configuration_resource().clone(),
                0,
                PAGE as usize,
                CacheAttribute::Uncached,
                Protection::ReadWrite,
            ) {
                Ok(mapping) => mapping,
                Err(error) => return Err(SlotRefusal::new(error, prepared.into_parts())),
            };
            let config = prepared
                .attach(mapping)
                .map_err(|(error, original)| SlotRefusal::new(error, original))?;
            if let Err(error) = header
                .verify(|offset| config.try_read32(offset))
                .and_then(|()| plan.check_disabled(|offset| config.try_read16(offset)))
            {
                return Err(SlotRefusal::new(error, config.into_parts()));
            }
            // SAFETY: the parked map retired, this is its sole successor,
            // and the original kept paths are checked again before changing
            // the original grant's operation policy.
            unsafe {
                config.promote(|owner: &mut SlotCustody| {
                    check_paths()?;
                    owner.driver_lines = Some(lines.clone());
                    Ok(())
                })
            }
            .map_err(|(error, original)| SlotRefusal::new(error, original))
        })();
        match result {
            Ok(config) => Ok(config),
            Err(mut refusal) => {
                refusal.error.address = Some(address);
                let error = refusal.error;
                let mut cohort = self.pci_slots.0.borrow_mut();
                let Cohort::Parked(fabric) = &mut *cohort else {
                    unreachable!("the original parked fabric stays owned")
                };
                fabric.slots[index].refusal = Some(refusal);
                Err(error)
            }
        }
    }

    /// Contain the captured fabric once and park each original configuration.
    /// The return value qualifies PCI-level controls for remapper takeover.
    ///
    /// # Safety
    /// Firmware and APs are stopped. The caller keeps all captured buses and
    /// their topology exclusive, including reset and hotplug.
    pub(crate) unsafe fn contain_pci(
        &self,
        tsc_hz: u64,
    ) -> Result<ContainedPci<'_>, ContainmentError> {
        {
            let mut cohort = self.pci_slots.0.borrow_mut();
            match &*cohort {
                Cohort::Unclaimed => *cohort = Cohort::Incomplete(Fabric::default()),
                Cohort::Incomplete(Fabric {
                    refusal: Some(refusal),
                    ..
                }) => return Err(refusal.error),
                _ => return Err(Error::Busy.into()),
            }
        }
        // SAFETY: the caller keeps the complete captured topology exclusive.
        // with_config_inventory retires the last read-only map before return.
        let (inputs, left_out) = unsafe { self.with_config_inventory(census) }?;
        let count = inputs.len();
        let mut order = Vec::new();
        order
            .try_reserve_exact(count)
            .map_err(|_| Error::Exhausted)?;
        self.prepare_fabric(inputs, left_out)?;
        order.extend(0..count);
        // Stop endpoints while their bridges still forward. Stop downstream
        // bridges before their upstream parents.
        {
            let cohort = self.pci_slots.0.borrow();
            let Cohort::Incomplete(fabric) = &*cohort else {
                unreachable!("exclusive containment owns the incomplete fabric");
            };
            order.sort_unstable_by_key(|&index| {
                let slot = &fabric.slots[index];
                (slot.header.bridge.is_some(), Reverse(slot.address.bus))
            });
        }

        for index in order {
            let (address, prepared, header, plan, misconfigured) = {
                let mut cohort = self.pci_slots.0.borrow_mut();
                let Cohort::Incomplete(fabric) = &mut *cohort else {
                    unreachable!("exclusive containment owns the incomplete fabric");
                };
                let slot = &mut fabric.slots[index];
                (
                    slot.address,
                    slot.configuration
                        .take()
                        .expect("one prepared configuration"),
                    slot.header,
                    slot.plan.clone(),
                    slot.misconfigured,
                )
            };
            // A function stopped and sealed with a capability its plan left
            // as found (`Plan::refusal`), or a bridge whose bus numbers are
            // not used, is refused alone.
            let result =
                self.contain_slot(prepared, header, &plan, tsc_hz)
                    .and_then(|parked| match (misconfigured, plan.refusal()) {
                        (Some(reason), _) => Err(SlotRefusal::left(
                            ContainmentError {
                                address: None,
                                cause: Cause::BridgeBuses {
                                    buses: header.bridge.unwrap_or(0),
                                    reason,
                                },
                                effect: ConfigEffect::None,
                            },
                            parked,
                        )),
                        (None, Some(cause)) => Err(SlotRefusal::left(cause, parked)),
                        (None, None) => Ok(parked),
                    });
            let mut cohort = self.pci_slots.0.borrow_mut();
            let Cohort::Incomplete(fabric) = &mut *cohort else {
                unreachable!("exclusive containment owns the incomplete fabric");
            };
            match result {
                Ok(parked) => fabric.slots[index].configuration = Some(parked),
                Err(mut refusal) if refusal.error.per_function() => {
                    refusal.error.address = Some(address);
                    let error = refusal.error;
                    // Reported once here: the slot keeps the refusal, and
                    // every later claim of it, or of a function below a
                    // refused bridge, is refused without another stop.
                    if refusal.master_stopped {
                        crate::output::arguments(format_args!(
                            "HOST PCI function not contained: {error}; it stays undriven\n"
                        ));
                    } else {
                        crate::output::arguments(format_args!(
                            "HOST PCI function not contained: {error}; it stays undriven and may \
                             still master DMA, so no IOMMU unit is taken and native drivers stay \
                             refused\n"
                        ));
                        fabric.unstopped_master = fabric.unstopped_master.or(Some(address));
                    }
                    fabric.slots[index].refusal = Some(refusal);
                }
                Err(mut refusal) => {
                    refusal.error.address = Some(address);
                    let error = refusal.error;
                    fabric.refusal = Some(refusal);
                    return Err(error);
                }
            }
        }
        let mut cohort = self.pci_slots.0.borrow_mut();
        let Cohort::Incomplete(fabric) = &mut *cohort else {
            unreachable!("exclusive containment owns the incomplete fabric");
        };
        *cohort = Cohort::Parked(core::mem::take(fabric));
        Ok(ContainedPci {
            source: self,
            functions: count,
        })
    }

    fn prepare_fabric(
        &self,
        mut inputs: Vec<Input>,
        left_out: census::LeftOut,
    ) -> Result<(), ContainmentError> {
        let roots = self.platform.wire().roots();
        let mut observed = Vec::new();
        observed
            .try_reserve_exact(inputs.len())
            .map_err(|_| Error::Exhausted)?;
        // A misconfigured bridge forwards nothing the topology may use.
        observed.extend(inputs.iter().map(|input| {
            topology::Function {
                address: input.address,
                bridge_buses: input
                    .header
                    .bridge
                    .filter(|_| input.misconfigured.is_none()),
            }
        }));
        let parents = topology::resolve(roots, &observed)?;
        inputs.sort_unstable_by_key(|input| input.address);

        let mut fabric = Fabric {
            left_out,
            ..Fabric::default()
        };
        fabric
            .roots
            .try_reserve_exact(roots.len())
            .map_err(|_| Error::Exhausted)?;
        fabric
            .slots
            .try_reserve_exact(inputs.len())
            .map_err(|_| Error::Exhausted)?;
        for descriptor in roots {
            let assignment = (descriptor.space == 2).then(|| {
                Assignment::from_owned(RootBus {
                    _descriptor: descriptor,
                    platform: self.platform.clone(),
                    aperture: self.aperture.parent(),
                })
            });
            fabric.roots.push(assignment);
        }
        *self.pci_slots.0.borrow_mut() = Cohort::Incomplete(fabric);
        // Secondary bus numbers exceed their upstream bus. BDF order keeps
        // each real bridge assignment before any of its children.
        let mut orphan = None;
        for ((address, route), input) in parents.into_iter().zip(inputs) {
            debug_assert_eq!(address, input.address);
            // Reported once per bus: BDF order keeps a bus's functions
            // together.
            if let topology::Parent::Root(index) = route
                && u64::from(address.bus) != roots[index].minimum
                && orphan.replace((address.segment, address.bus))
                    != Some((address.segment, address.bus))
            {
                crate::output::arguments(format_args!(
                    "HOST PCI {address} is on bus {:#04x}, which no bridge forwards to; its \
                     functions are placed under their root bridge\n",
                    address.bus
                ));
            }
            let parent = {
                let cohort = self.pci_slots.0.borrow();
                let Cohort::Incomplete(fabric) = &*cohort else {
                    unreachable!("exclusive containment owns the incomplete fabric");
                };
                match route {
                    topology::Parent::Root(index) => fabric.roots[index]
                        .as_ref()
                        .expect("resolved root grant")
                        .parent(),
                    topology::Parent::Bridge(bridge) => {
                        let at = fabric
                            .slots
                            .binary_search_by_key(&bridge, |slot| slot.address)
                            .expect("upstream slot is ordered before its children");
                        fabric.slots[at].ancestor.clone()
                    }
                }
            };
            let slot = self
                .prepare_slot(input, parent, route)
                .map_err(|error| error.at(address))?;
            let mut cohort = self.pci_slots.0.borrow_mut();
            let Cohort::Incomplete(fabric) = &mut *cohort else {
                unreachable!("exclusive containment owns the incomplete fabric");
            };
            fabric.slots.push(slot);
        }
        Ok(())
    }

    fn prepare_slot(
        &self,
        input: Input,
        parent: Parent,
        route: topology::Parent,
    ) -> Result<Slot, ContainmentError> {
        let (resource, original) =
            self.reservation_under(input.span, ResourceKind::Configuration, parent)?;
        let ancestor = original.parent();
        let close = original.admission_close();
        let restore = Rc::new(RefCell::new(Vec::new()));
        let owner = Box::new(SlotCustody {
            completion: Arc::new(Completion {
                epoch: Epoch {
                    platform: self.platform.clone(),
                    original: original.retain(),
                },
                fault: FirstFault::new(),
            }),
            _original: original,
            resource,
            address: input.address,
            window: input.window,
            plan: input.plan.clone(),
            driver_lines: None,
            restore: restore.clone(),
        });
        // SAFETY: the same affine slot backs the bounded operation and map.
        let grant = unsafe { PrepublicationGrant::from_boxed(owner, close) };
        Ok(Slot {
            address: input.address,
            configuration: Some(UnpublishedConfig::prepare(grant)),
            refusal: None,
            header: input.header,
            plan: input.plan,
            ancestor,
            route,
            misconfigured: input.misconfigured,
            forwarding: None,
            restore,
        })
    }

    fn contain_slot(
        &self,
        prepared: PreparedConfig<PrepublicationGrant>,
        header: Header,
        plan: &Plan,
        tsc_hz: u64,
    ) -> Result<PreparedConfig<PrepublicationGrant>, SlotRefusal> {
        let mapping = match self.authority.map(
            prepared.configuration_resource().clone(),
            0,
            PAGE as usize,
            CacheAttribute::Uncached,
            Protection::ReadWrite,
        ) {
            Ok(mapping) => mapping,
            Err(error) => return Err(SlotRefusal::new(error, prepared.into_parts())),
        };
        let mut config = prepared
            .attach(mapping)
            .map_err(|(error, original)| SlotRefusal::new(error, original))?;
        let effect = match header
            .verify(|offset| config.try_read32(offset))
            .and_then(|()| plan.apply(&mut config))
        {
            Ok(effect) => effect,
            Err(error) => return Err(SlotRefusal::unstopped(error, config, plan)),
        };
        let acknowledge = (|| -> Result<(), ClaimError> {
            let started = crate::apic::rdtsc();
            let limit = (tsc_hz / 1_000 * PENDING_TRANSACTIONS_MS).max(1);
            wait_for_pending_transactions(&config, || {
                crate::apic::rdtsc().wrapping_sub(started) >= limit
            })?;
            // Stopped controls read back, and Bus Master is sealed: clear,
            // or proven hard-wired on a root-bus host or ISA bridge.
            plan.seal(&mut config)?;
            if header.bridge.is_some() {
                config.seal_bus_routing()?;
            }
            header.verify(|offset| config.try_read32(offset))?;
            Ok(())
        })();
        if let Err(cause) = acknowledge {
            return Err(SlotRefusal::unstopped(
                ContainmentError {
                    address: None,
                    cause: Cause::Claim(cause),
                    effect,
                },
                config,
                plan,
            ));
        }
        // SAFETY: the only accesses were synchronous and bounded, with no
        // escaped aliases. Source keeps this original decode and assignment.
        unsafe { config.park() }.map_err(|(error, original)| {
            SlotRefusal::new(
                ContainmentError {
                    address: None,
                    cause: Cause::Claim(ClaimError::Mapping(error)),
                    effect,
                },
                original,
            )
        })
    }
}
