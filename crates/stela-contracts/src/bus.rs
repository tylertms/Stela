//! Guest-physical MMIO routing for the Apple device models.
//!
//! The system engine decodes a guest load or store that hits device memory,
//! then calls [`Bus::read`] or [`Bus::write`] with the guest PA, the access
//! width and an [`Access`] carrying the accessing CPU, the system counter value
//! and an interrupt sink. Any refusal is returned unchanged; the engine must
//! stop or fault rather than invent a value.
use super::interrupt::{CpuIndex, InterruptSink};
use crate::Refusal;
use alloc::sync::Arc;
#[cfg(not(feature = "std"))]
use alloc::{boxed::Box, vec::Vec};
use core::any::Any;
use core::sync::atomic::{AtomicU64, Ordering};

mod container;
pub use container::SharedView;

/// Access width in bytes. Apple MMIO is little-endian.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Width {
    Byte = 1,
    Half = 2,
    Word = 4,
    Double = 8,
}
impl Width {
    pub const fn bytes(self) -> u64 {
        self as u64
    }
    pub const fn from_bytes(bytes: u64) -> Option<Self> {
        match bytes {
            1 => Some(Self::Byte),
            2 => Some(Self::Half),
            4 => Some(Self::Word),
            8 => Some(Self::Double),
            _ => None,
        }
    }
    /// All-ones value of this width.
    pub const fn mask(self) -> u64 {
        match self {
            Self::Double => u64::MAX,
            width => (1 << (width.bytes() * 8)) - 1,
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Direction {
    Read,
    Write,
}

/// A refused device access. Earlier device, wire or transition effects may
/// already be held by their owners; this error alone proves no byte prefix
/// and grants no replay or source-retirement permission.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum MmioError {
    /// No device is mapped at this guest PA.
    Unmapped { address: u64, width: Width },
    /// The access is not naturally aligned.
    Misaligned { address: u64, width: Width },
    /// A device refused the access. `offset` is device-relative.
    Unsupported {
        device: &'static str,
        offset: u64,
        width: Width,
        direction: Direction,
        reason: Refusal,
    },
    /// A device refused to bring its own state, such as a clock it owns,
    /// to count `now` as the board advanced it: no guest access was made.
    Advance {
        device: &'static str,
        now: u64,
        reason: Refusal,
    },
}

/// What a guest request does to the whole machine.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Transition {
    /// The machine resets and boots again (a watchdog bite, the SMC's
    /// restart command).
    Restart,
    /// The machine powers off (the SMC's standby command).
    PowerOff,
    /// The machine cannot continue (a model reached a state it cannot run
    /// in); neither a reset nor a power-off was asked for.
    Halt,
}

/// A guest request to reset or stop the machine, surfaced instead of acting
/// on the host.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ResetRequest {
    pub device: &'static str,
    pub transition: Transition,
    pub reason: &'static str,
    /// System counter value at which the request took effect.
    pub at: u64,
}

/// Inputs and outputs of one device access or time advance.
pub struct Access<'a> {
    /// The CPU performing the access; selects per-CPU register views.
    pub cpu: CpuIndex,
    /// Current system counter value in 24 MHz ticks (`counter::FREQUENCY_HZ`).
    pub now: u64,
    pub interrupts: &'a mut dyn InterruptSink,
    reset: Option<ResetRequest>,
}
impl<'a> Access<'a> {
    pub fn new(cpu: CpuIndex, now: u64, interrupts: &'a mut dyn InterruptSink) -> Self {
        Self {
            cpu,
            now,
            interrupts,
            reset: None,
        }
    }
    /// The guest asked for a machine reset.
    pub fn request_reset(&mut self, device: &'static str, reason: &'static str) {
        self.request_transition(device, Transition::Restart, reason);
    }
    /// The guest's request takes the whole machine through `transition`;
    /// the first request of an access wins.
    pub fn request_transition(
        &mut self,
        device: &'static str,
        transition: Transition,
        reason: &'static str,
    ) {
        self.reset.get_or_insert(ResetRequest {
            device,
            transition,
            reason,
            at: self.now,
        });
    }
    /// The first reset requested during this access, if any. The engine must
    /// stop the guest and report it; the models never touch host reset paths.
    pub fn reset_request(&self) -> Option<ResetRequest> {
        self.reset
    }
}

/// Effective supply, clock, DMA-quiesce and reset inputs of one native
/// device. Retention describes that device's declared ordinary-state contract,
/// not a request to reconstruct software configuration after a destructive edge.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LifecycleState {
    pub powered: bool,
    pub clocked: bool,
    pub quiesced: bool,
    pub reset: bool,
    pub retention: bool,
}

/// A native device's complete lifecycle boundary. Capability does not
/// imply every proposed state is checked. The machine checks every affected
/// device without side effects before acknowledging the controller request,
/// then commits the checked state before any subsequent client activity.
/// A commit must not fail or generate synthetic completion of canceled work.
pub trait Lifecycle {
    fn check_lifecycle(&self, proposed: LifecycleState) -> Result<(), Refusal>;
    fn set_lifecycle(&mut self, proposed: LifecycleState, access: &mut Access<'_>);
}

/// A memory-mapped device model. Offsets are relative to the start of the
/// device's address space as placed by [`Bus::map`]. Values occupy the low
/// `width` bytes; implementations refuse widths their registers do not have.
pub trait MmioDevice: Any {
    /// Immutable lifetime contract, queried only when this owner is installed.
    /// Passive promises that `advance` is always the default no-op, including
    /// after MMIO, lifecycle and peer mutations. An idle poller must use Always.
    fn advance_policy(&self) -> AdvancePolicy {
        AdvancePolicy::Always
    }
    /// Type-erased capability: e.g. a UART keeps a generic console sink, so
    /// lifecycle dispatch must not require downcasting to its concrete type.
    /// Returning None keeps missing lifecycle support explicit.
    fn lifecycle(&mut self) -> Option<&mut dyn Lifecycle> {
        None
    }
    fn name(&self) -> &'static str;
    /// The name of the register at `offset`, for reports; `None` where the
    /// model names none.
    fn register_name(&self, _offset: u64) -> Option<&'static str> {
        None
    }
    /// Whether a read at `offset` of `width` is plain: it changes nothing in
    /// the model, and its value follows from the model's state alone, not
    /// from the time, the reading CPU or anything outside the model. The
    /// bus then leaves the model's epoch ([`Bus::plain_epoch`]) as it was,
    /// so a value read before stays the value while the epoch holds.
    fn plain_read(&self, _offset: u64, _width: Width) -> bool {
        false
    }
    fn read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError>;
    fn write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError>;
    /// Re-evaluates time-driven state at `access.now` and returns the next
    /// counter value at which this device must be advanced again, if any.
    fn advance(&mut self, _access: &mut Access<'_>) -> Result<Option<u64>, MmioError> {
        Ok(None)
    }
}

/// Whether an owner can have time-driven effects in any reachable state.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AdvancePolicy {
    Always,
    Passive,
}

/// Builds the refusal for one device register.
pub fn refuse(
    device: &'static str,
    offset: u64,
    width: Width,
    direction: Direction,
    reason: impl Into<Refusal>,
) -> MmioError {
    MmioError::Unsupported {
        device,
        offset,
        width,
        direction,
        reason: reason.into(),
    }
}

/// `device` refused to bring its own state to count `now`
/// ([`MmioError::Advance`]).
pub fn refuse_advance(device: &'static str, now: u64, reason: impl Into<Refusal>) -> MmioError {
    MmioError::Advance {
        device,
        now,
        reason: reason.into(),
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
pub struct DeviceId(usize);

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MapError {
    Empty,
    /// A model requires naturally aligned physical register words.
    Unaligned,
    Overflow,
    Overlap {
        existing: DeviceId,
    },
    UnknownDevice,
    Allocation,
    ProposedOverlap,
    /// An explicit shared view lacks complete, matching existing ownership.
    InvalidSharedView,
}

#[derive(Debug, Clone, Copy)]
struct Region {
    start: u64,
    /// Exclusive.
    end: u64,
    device: DeviceId,
    device_offset: u64,
}

/// Routes guest PA ranges to devices. One device may own several ranges, for
/// example the AIC2 core and event blocks, which are separate DeviceTree `reg`
/// entries.
#[derive(Default)]
pub struct Bus {
    devices: Vec<Box<dyn MmioDevice>>,
    /// Each device's epoch, which every access that may change it moves
    /// before it does ([`Self::plain_epoch`]).
    epochs: Vec<Arc<AtomicU64>>,
    /// Stable owner IDs in registration order; routes and aliases add no polls.
    advancing: Vec<DeviceId>,
    /// Sorted by `start`, nonoverlapping.
    regions: Vec<Region>,
    #[cfg(feature = "native-profile")]
    advance_statistics: AdvanceStatistics,
}

/// Owner inventory and time-advance work; aliases are not owners.
#[cfg(feature = "native-profile")]
#[derive(Debug, Default, Clone, Copy, PartialEq, Eq)]
pub struct AdvanceStatistics {
    pub owners: usize,
    pub active_owners: usize,
    pub passes: u64,
    pub completed_passes: u64,
    pub invocations: u64,
}

impl Bus {
    #[cfg(feature = "native-profile")]
    pub fn advance_statistics(&self) -> AdvanceStatistics {
        AdvanceStatistics {
            owners: self.devices.len(),
            active_owners: self.advancing.len(),
            ..self.advance_statistics
        }
    }
    pub fn new() -> Self {
        Self::default()
    }
    pub fn add(&mut self, device: Box<dyn MmioDevice>) -> DeviceId {
        let id = DeviceId(self.devices.len());
        if device.advance_policy() == AdvancePolicy::Always {
            self.advancing.push(id);
        }
        self.devices.push(device);
        self.epochs.push(Arc::default());
        id
    }
    /// Install one device and all its ranges atomically. Validate both
    /// existing and proposed ownership before publishing any route or device.
    /// Each binding is (guest PA, length, device offset).
    pub fn install(
        &mut self,
        device: Box<dyn MmioDevice>,
        bindings: &[(u64, u64, u64)],
    ) -> Result<DeviceId, MapError> {
        self.install_many([device], [bindings]).map(|[id]| id)
    }
    /// Publish a complete set of distinct MMIO owners atomically. A failed
    /// later model/range cannot leave an earlier member installed. This joins
    /// physical ownership only; it supplies no device state or readiness.
    pub fn install_many<const N: usize>(
        &mut self,
        devices: [Box<dyn MmioDevice>; N],
        bindings: [&[(u64, u64, u64)]; N],
    ) -> Result<[DeviceId; N], MapError> {
        if N == 0 {
            return Err(MapError::Empty);
        }
        self.devices
            .len()
            .checked_add(N)
            .ok_or(MapError::Overflow)?;
        let ids = core::array::from_fn(|index| DeviceId(self.devices.len() + index));
        let mut proposed = Vec::new();
        for (&id, bindings) in ids.iter().zip(bindings) {
            if bindings.is_empty() {
                return Err(MapError::Empty);
            }
            proposed
                .try_reserve_exact(bindings.len())
                .map_err(|_| MapError::Allocation)?;
            for &(start, len, device_offset) in bindings {
                if len == 0 {
                    return Err(MapError::Empty);
                }
                let end = start.checked_add(len).ok_or(MapError::Overflow)?;
                device_offset.checked_add(len).ok_or(MapError::Overflow)?;
                let overlaps = |region: &Region| region.start < end && start < region.end;
                if let Some(region) = self.regions.iter().find(|region| overlaps(region)) {
                    return Err(MapError::Overlap {
                        existing: region.device,
                    });
                }
                if proposed.iter().any(overlaps) {
                    return Err(MapError::ProposedOverlap);
                }
                proposed.push(Region {
                    start,
                    end,
                    device: id,
                    device_offset,
                });
            }
        }
        // All allocations precede publication. Capacity changes on failure do
        // not add observable routes or a device with partially mapped state.
        self.devices
            .try_reserve(N)
            .map_err(|_| MapError::Allocation)?;
        self.regions
            .try_reserve(proposed.len())
            .map_err(|_| MapError::Allocation)?;
        self.advancing
            .try_reserve(N)
            .map_err(|_| MapError::Allocation)?;
        for (&id, device) in ids.iter().zip(&devices) {
            if device.advance_policy() == AdvancePolicy::Always {
                self.advancing.push(id);
            }
        }
        self.epochs
            .try_reserve(N)
            .map_err(|_| MapError::Allocation)?;
        self.devices.extend(devices);
        self.epochs.extend((0..N).map(|_| Arc::default()));
        self.regions.extend(proposed);
        self.regions.sort_unstable_by_key(|region| region.start);
        Ok(ids)
    }
    /// Maps guest PAs `[start, start + len)` to device offsets
    /// `[device_offset, device_offset + len)`.
    pub fn map(
        &mut self,
        device: DeviceId,
        start: u64,
        len: u64,
        device_offset: u64,
    ) -> Result<(), MapError> {
        if device.0 >= self.devices.len() {
            return Err(MapError::UnknownDevice);
        }
        if len == 0 {
            return Err(MapError::Empty);
        }
        let end = start.checked_add(len).ok_or(MapError::Overflow)?;
        device_offset.checked_add(len).ok_or(MapError::Overflow)?;
        let index = self.free_index(start, end)?;
        self.regions.insert(
            index,
            Region {
                start,
                end,
                device,
                device_offset,
            },
        );
        Ok(())
    }
    /// Pure whole-range preflight for a new explicitly owned resource. It
    /// reserves nothing and publishes no model or aperture. Callers keep
    /// exclusive Bus access through their complete batch publication.
    pub fn check_free(&self, start: u64, len: u64) -> Result<(), MapError> {
        if len == 0 {
            return Err(MapError::Empty);
        }
        let end = start.checked_add(len).ok_or(MapError::Overflow)?;
        self.free_index(start, end).map(|_| ())
    }
    fn free_index(&self, start: u64, end: u64) -> Result<usize, MapError> {
        let index = self.regions.partition_point(|region| region.start < start);
        let overlaps = |region: &Region| region.start < end && start < region.end;
        for neighbor in [index.checked_sub(1), Some(index)].into_iter().flatten() {
            if let Some(region) = self.regions.get(neighbor).filter(|region| overlaps(region)) {
                return Err(MapError::Overlap {
                    existing: region.device,
                });
            }
        }
        Ok(index)
    }

    /// Maps the parts of guest PAs `[start, start + len)` that no device
    /// owns yet, each at its own offset from `device_offset`: a window whose
    /// span the DeviceTree shares with smaller register banks (the `sgx`
    /// window around `gfx-asc` and a fabric bridge) owns only what they leave.
    /// A window with no free byte refuses.
    pub fn map_gaps(
        &mut self,
        device: DeviceId,
        start: u64,
        len: u64,
        device_offset: u64,
    ) -> Result<(), MapError> {
        if len == 0 {
            return Err(MapError::Empty);
        }
        let end = start.checked_add(len).ok_or(MapError::Overflow)?;
        device_offset.checked_add(len).ok_or(MapError::Overflow)?;
        let owned: Vec<(u64, u64, DeviceId)> = self
            .regions
            .iter()
            .filter(|region| region.start < end && start < region.end)
            .map(|region| (region.start.max(start), region.end.min(end), region.device))
            .collect();
        let mut cursor = start;
        let mut gaps = Vec::new();
        for &(first, last, _) in &owned {
            if first > cursor {
                gaps.push((cursor, first));
            }
            cursor = cursor.max(last);
        }
        if cursor < end {
            gaps.push((cursor, end));
        }
        if gaps.is_empty() {
            return Err(MapError::Overlap {
                existing: owned[0].2,
            });
        }
        for (first, last) in gaps {
            self.map(device, first, last - first, device_offset + (first - start))?;
        }
        Ok(())
    }
    /// The device and device offset for a complete guest PA access. Mounts
    /// may inspect ownership without reading or changing a register.
    pub fn route(&self, address: u64, width: Width) -> Result<(DeviceId, u64), MmioError> {
        if !address.is_multiple_of(width.bytes()) {
            return Err(MmioError::Misaligned { address, width });
        }
        let index = self
            .regions
            .partition_point(|region| region.start <= address);
        let region = index
            .checked_sub(1)
            .map(|index| self.regions[index])
            .filter(|region| {
                address
                    .checked_add(width.bytes())
                    .is_some_and(|end| end <= region.end)
            })
            .ok_or(MmioError::Unmapped { address, width })?;
        Ok((
            region.device,
            region.device_offset + (address - region.start),
        ))
    }
    /// The name of the model mapped at `address`, for reports.
    pub fn name_at(&self, address: u64) -> Option<&'static str> {
        let (device, _) = self.route(address, Width::Byte).ok()?;
        Some(self.devices.get(device.0)?.name())
    }
    /// Move `device`'s epoch: an access that may change it follows.
    fn changing(&self, device: DeviceId) {
        if let Some(epoch) = self.epochs.get(device.0) {
            epoch.fetch_add(1, Ordering::SeqCst);
        }
    }
    /// For a plain read at `address` of `width` ([`MmioDevice::plain_read`]),
    /// its model's epoch and the count it holds now: while the epoch still
    /// holds that count, no access that may change the model has begun, so
    /// a value the read gave since stays the value. `None` for any other
    /// read.
    pub fn plain_epoch(&self, address: u64, width: Width) -> Option<(Arc<AtomicU64>, u64)> {
        let (device, offset) = self.route(address, width).ok()?;
        if !self.devices.get(device.0)?.plain_read(offset, width) {
            return None;
        }
        let epoch = self.epochs.get(device.0)?;
        Some((Arc::clone(epoch), epoch.load(Ordering::SeqCst)))
    }
    /// The name the model gives the register at `address`, for reports.
    pub fn register_name_at(&self, address: u64) -> Option<&'static str> {
        let (device, offset) = self.route(address, Width::Byte).ok()?;
        self.devices.get(device.0)?.register_name(offset)
    }
    pub fn read(
        &mut self,
        address: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError> {
        let (device, offset) = self.route(address, width)?;
        if !self.devices[device.0].plain_read(offset, width) {
            self.changing(device);
        }
        let value = self.devices[device.0].read(offset, width, access)?;
        debug_assert_eq!(value & !width.mask(), 0, "device read wider than access");
        Ok(value & width.mask())
    }
    pub fn write(
        &mut self,
        address: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        let (device, offset) = self.route(address, width)?;
        self.changing(device);
        self.devices[device.0].write(offset, width, value & width.mask(), access)
    }
    /// Advances every time-driven owner once and returns the earliest deadline.
    /// Permanently passive owners have no time effects or deadlines to observe.
    pub fn advance(&mut self, access: &mut Access<'_>) -> Result<Option<u64>, MmioError> {
        #[cfg(feature = "native-profile")]
        {
            self.advance_statistics.passes = self.advance_statistics.passes.saturating_add(1);
        }
        let mut next: Option<u64> = None;
        for index in 0..self.advancing.len() {
            let id = self.advancing[index];
            self.changing(id);
            let device = &mut self.devices[id.0];
            let advanced = device.advance(access);
            #[cfg(feature = "native-profile")]
            if advanced.is_err() {
                self.advance_statistics.invocations = self
                    .advance_statistics
                    .invocations
                    .saturating_add((index + 1) as u64);
            }
            if let Some(deadline) = advanced? {
                next = Some(next.map_or(deadline, |next| next.min(deadline)));
            }
        }
        #[cfg(feature = "native-profile")]
        {
            self.advance_statistics.completed_passes =
                self.advance_statistics.completed_passes.saturating_add(1);
            self.advance_statistics.invocations = self
                .advance_statistics
                .invocations
                .saturating_add(self.advancing.len() as u64);
        }
        Ok(next)
    }
    /// A model's state, for engine wiring such as the AMCC's write lock.
    pub fn lifecycle(&mut self, device: DeviceId) -> Option<&mut dyn Lifecycle> {
        self.changing(device);
        self.devices.get_mut(device.0)?.lifecycle()
    }

    pub fn device<T: MmioDevice>(&self, device: DeviceId) -> Option<&T> {
        let device: &dyn Any = self.devices.get(device.0)?.as_ref();
        device.downcast_ref::<T>()
    }
    /// Typed access to two different models at once, for wiring between
    /// them such as a DMA master and its IOMMU.
    pub fn pair_mut<A: MmioDevice, B: MmioDevice>(
        &mut self,
        a: DeviceId,
        b: DeviceId,
    ) -> Option<(&mut A, &mut B)> {
        self.changing(a);
        self.changing(b);
        let [a, b] = self.devices.get_disjoint_mut([a.0, b.0]).ok()?;
        let a: &mut dyn Any = a.as_mut();
        let b: &mut dyn Any = b.as_mut();
        Some((a.downcast_mut::<A>()?, b.downcast_mut::<B>()?))
    }
    /// Typed access to a model, for engine wiring such as AIC inputs, UART
    /// receive data or fast-IPI system registers.
    pub fn device_mut<T: MmioDevice>(&mut self, device: DeviceId) -> Option<&mut T> {
        self.changing(device);
        let device: &mut dyn Any = self.devices.get_mut(device.0)?.as_mut();
        device.downcast_mut::<T>()
    }
}

#[cfg(test)]
mod tests;
