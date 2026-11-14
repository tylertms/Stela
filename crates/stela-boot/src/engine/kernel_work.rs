//! BSP work continuations over the loader's separately reserved stacks.
//! Admission here creates no Linux service, ACPI capability, or cross-CPU
//! execution grant. Those producers must join this affine owner explicitly.
use super::{
    host::{HostOwner, local::context::RootIdentity},
    storage::RestoreRoot,
};
use crate::{
    handoff::{Handoff, Region},
    layout,
    native_clock::{ClockServices, DriverExecution, Installed as InstalledClock},
    native_mmio, native_smp,
};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use core::{
    arch::asm,
    cell::{Cell, UnsafeCell},
    ffi::{c_uint, c_void},
    ops::Range,
    pin::Pin,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};
use stela_boot_core::guarded_stacks::mappings;
use stela_drivers::{
    Clock,
    bus::dma::memory::MemoryError,
    platform::work::{self, Cpu, Executor, Queue, Stack, StackBacking, WorkError},
};
/// Jobs the BSP's queue admits: one for every stack of its pool, and as many
/// again waiting, which covers the firmware's protected pending events.
pub(super) const SLOTS: usize = 2 * native_smp::resources::TASK_STACKS;
mod firmware;
mod pages;
pub(super) use pages::KernelPages;
mod pin;
mod plan;
mod task_data;
pub(crate) use firmware::FirmwareContext;
use plan::Plan;
#[derive(Debug)]
pub(crate) enum Error {
    Inventory,
    /// The loader's copy of the firmware's final memory map and PCI and
    /// ACPI settings is missing, for this reason.
    Capture(alloc::string::String),
    /// The CPUID topology APIC ID observed now differs from the firmware's
    /// MP-services processor ID of the boot processor, which the loader
    /// proved equal to the same CPUID observation at capture.
    WrongCpu {
        observed: Option<u32>,
        captured: u64,
    },
    Clock,
    Geometry,
    Allocation,
    AlreadyOwned,
    Mapping(mappings::Error),
    Cache(MemoryError),
    Work(WorkError),
    Qualification,
    Image(crate::native_image::Error),
    NoKernelStacks,
    Pages(stela_linux::pages::Error),
    /// The preparation stage that refused, around its refusal.
    At(&'static str, Box<Error>),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Inventory => f.write_str("sealed loader stack inventory unavailable"),
            Self::Capture(why) => f.write_str(why),
            Self::WrongCpu { observed, captured } => write!(
                f,
                "kernel work requires the actual BSP: CPUID topology APIC ID {observed:#x?} \
                 now, firmware MP-services processor ID {captured:#x} at capture"
            ),
            Self::Clock => f.write_str("kernel work counter calibration unavailable"),
            Self::Geometry => f.write_str("kernel stack geometry is invalid"),
            Self::Allocation => f.write_str("kernel work metadata allocation failed"),
            Self::AlreadyOwned => f.write_str("kernel stack pool is already owned"),
            Self::Mapping(error) => write!(f, "kernel stack page-table qualification: {error:?}"),
            Self::Cache(error) => {
                write!(f, "kernel stack effective cache qualification: {error:?}")
            }
            Self::Work(error) => write!(f, "native continuation admission: {error:?}"),
            Self::Qualification => f.write_str("native continuation identity or stack differs"),
            Self::Image(error) => write!(f, "kernel work executable backing: {error}"),
            Self::NoKernelStacks => {
                f.write_str("actual work pool has no Linux-compatible stack slots")
            }
            Self::Pages(error) => write!(f, "Linux page catalog: {error:?}"),
            Self::At(stage, error) => write!(f, "{stage}: {error}"),
        }
    }
}
impl Error {
    /// What a user could check or report for this refusal.
    fn cause(&self) -> stela_boot_core::work_status::Cause {
        use stela_boot_core::work_status::Cause;
        match self {
            Self::Inventory
            | Self::Geometry
            | Self::AlreadyOwned
            | Self::NoKernelStacks
            | Self::Pages(_) => Cause::Records,
            Self::Capture(_) => Cause::Capture,
            Self::WrongCpu { .. } => Cause::Processor,
            Self::Clock => Cause::Clock,
            Self::Allocation => Cause::Memory,
            Self::Mapping(_) => Cause::Mapping,
            Self::Cache(_) => Cause::Cache,
            Self::Image(_) => Cause::Code,
            Self::Work(_) | Self::Qualification => Cause::Continuation,
            Self::At(_, error) => error.cause(),
        }
    }
    /// The screen's reason for this refusal, with its log detail.
    pub(crate) fn reason(&self) -> alloc::string::String {
        stela_boot_core::work_status::reason(self.cause(), &alloc::format!("{self}"))
    }
}
struct Reservation {
    identity: Arc<RootIdentity>,
    _wire: &'static native_mmio::Wire,
    _handoff: &'static Handoff,
    _image: crate::native_image::CodePin,
    allocation: native_smp::resources::AllocationClaim,
    released_stack_views: AtomicUsize,
    pin_epochs: [pin::Epoch; SLOTS],
}
/// Immutable DATA classifier retaining this exact Queue's original image,
/// allocation and native root/table custody. This is never executable authority.
pub(super) struct ReadOnlyImage {
    _reservation: Arc<Reservation>,
    ranges: Vec<Range<usize>>,
}
impl ReadOnlyImage {
    /// Authorize one complete read within a retained immutable data section.
    pub(super) fn contains_range(&self, address: usize, bytes: usize) -> bool {
        address.checked_add(bytes).is_some_and(|end| {
            self.ranges
                .iter()
                .any(|range| range.contains(&address) && end <= range.end)
        })
    }
}
// SAFETY: only KernelWork::image constructs this value under the same actual
// kept original root. The relocated allocation and every native mapping remain
// owned by Reservation; readonly_ranges admits only original immutable DATA,
// after relocation, with no later patcher, DMA/guest loan or allocator reuse.
// contains only checks stored bounds and never dereferences a caller pointer.
unsafe impl stela_drivers::platform::image::NativeReadOnlyImage for ReadOnlyImage {
    fn contains(&self, address: usize) -> bool {
        self.contains_range(address, 1)
    }
}
// SAFETY: this reservation retains the same original Handoff/LoaderTables,
// shared mapping owners and native CodePin for the kernel lifetime. Each
// borrow validates the actual entered root and effective cache attributes.
unsafe impl stela_linux::threads::OriginalKernelRoot for Reservation {
    fn pgd(
        &self,
        process: &dyn work::EnteredProcess,
    ) -> Result<core::ptr::NonNull<c_void>, WorkError> {
        process.check()?;
        let owner = &self._handoff.native_loader_tables;
        let scope = self.identity.process_root()?;
        scope.entered_original(owner)?;
        let physical = owner.root();
        let address = layout::physical(physical);
        let page = scope
            .observe_original(address)?
            .ok_or_else(|| WorkError::corrupt())?;
        if page.physical != physical || page.user || !page.writable {
            return Err(WorkError::corrupt());
        }
        crate::native_memory::cache::verify_writeback(address, physical, 4096, layout::physical(0))
            .map_err(|_| WorkError::Unavailable)?;
        process.check()?;
        core::ptr::NonNull::new(address as *mut c_void).ok_or_else(|| WorkError::corrupt())
    }
}

// SAFETY: the same retained original table graph and image owner cover the
// entered CPU and each C entry and geometry global. Publication is synchronous
// in the checked scope, after validating the effective mappings.
unsafe impl stela_linux::threads::OriginalKernelGeometry for Reservation {
    fn bind(
        &self,
        process: &dyn work::EnteredProcess,
        target: stela_linux::threads::GeometryBinding<'_>,
    ) -> Result<(), WorkError> {
        process.check()?;
        let owner = &self._handoff.native_loader_tables;
        let scope = self.identity.process_root()?;
        scope.entered_original(owner)?;

        for entry in target.entries() {
            // SAFETY: the original root and immutable image owner are held.
            let physical =
                unsafe { self._image.check_callback(entry) }.map_err(|_| WorkError::corrupt())?;
            let page = scope
                .observe_original(entry as u64)?
                .ok_or_else(|| WorkError::corrupt())?;
            if page.physical != physical || page.user || page.writable || page.nx {
                return Err(WorkError::corrupt());
            }
        }
        for address in target.storage()? {
            if !address.is_multiple_of(core::mem::align_of::<c_uint>()) {
                return Err(WorkError::corrupt());
            }
            let bytes = core::mem::size_of::<c_uint>();
            // SAFETY: the original root and immutable image owner are held.
            let physical = unsafe { self._image.writable_span(address, bytes) }
                .map_err(|_| WorkError::corrupt())?;
            let page = scope
                .observe_original(address as u64)?
                .ok_or_else(|| WorkError::corrupt())?;
            if page.physical != physical || page.user || !page.writable || !page.nx {
                return Err(WorkError::corrupt());
            }
            crate::native_memory::cache::verify_writeback(
                address as u64,
                physical,
                bytes,
                layout::physical(0),
            )
            .map_err(|_| WorkError::Unavailable)?;
        }

        process.check()?;
        scope.entered_original(owner)?;
        // SAFETY: the exact entered LoaderTables, its live paging mode and the
        // original code and writable globals have all been checked.
        unsafe { target.publish(owner.levels()) }
    }
}

struct Backing {
    _reservation: Arc<Reservation>,
    bounds: Range<usize>,
    guard: Range<usize>,
    physical: u64,
    slot: usize,
}
// SAFETY: prepare consumes one loader allocation exactly once, verifies
// every root's exact RW/NX/supervisor/WB backing and absent guards, and verifies
// effective MTRRs on the original BSP. Slots are disjoint and excluded from
// guest/DMA allocations. The physical reservation and shared window are never
// reclaimed during this boot; each backing additionally pins the CPU/
// descriptor/root owner. Runtime keeps paused/quarantined frames and backing.
unsafe impl StackBacking for Backing {
    fn bounds(&self) -> Range<usize> {
        self.bounds.clone()
    }
    fn guard(&self) -> Range<usize> {
        self.guard.clone()
    }
    unsafe fn pin_prepared_pages(
        &self,
        epoch: u64,
    ) -> Result<stela_drivers::bus::dma::memory::PinnedPages, MemoryError> {
        // SAFETY: the native preparation owns the zeroed slot. Export
        // independently excludes an older view and checks the real mapping.
        unsafe { pin::export(self, epoch) }
    }
}
impl Drop for Backing {
    fn drop(&mut self) {
        // This retires one view only. Its raw loader allocation/root mapping
        // remains owned for the whole boot; no physical reclaim is inferred.
        self._reservation
            .released_stack_views
            .fetch_add(1, Ordering::Relaxed);
    }
}
struct NativeCpu {
    reservation: Arc<Reservation>,
    clock: Clock,
}
// SAFETY: immutable RootIdentity checks CPU/GS/descriptors and the real
// root catalog before local-state access. IRQ enqueue/wake may observe
// another owned guest root, but execution switches happen only via the affine
// owner under RestoreRoot; Runtime additionally checks captured CR3/controls.
// Ticks use the executing BSP's real calibrated progressing TSC; no task migrates
// or reconfigures controls. This constructor grants no fixed-rate Linux clock or
// idle/deadline service, which needs separate clock/IRQ validation. Descriptor/local storage is permanently kept even on quarantine.
unsafe impl Cpu for NativeCpu {
    fn check_cpu(&self) -> Result<(), WorkError> {
        self.reservation.identity.check_irq_cpu()
    }
    fn in_interrupt(&self) -> bool {
        self.reservation.identity.in_interrupt()
    }
    fn save_disable(&self) -> Result<u64, WorkError> {
        self.check_cpu()?;
        let saved;
        // SAFETY: owning CPL0 CPU; exact save brackets metadata changes.
        unsafe {
            asm!("pushfq","pop {}","cli",out(reg)saved);
        }
        Ok(saved)
    }
    unsafe fn restore(&self, flags: u64) {
        if let Err(error) = self.check_cpu() {
            self.fatal(error);
        }
        // SAFETY: caller supplies this CPU's enclosing save after restoring
        // task metadata/context; no queue lock spans this restoration.
        unsafe {
            asm!("push {}","popfq",in(reg)flags);
        }
    }
    fn ticks(&self) -> u64 {
        if let Err(error) = self.check_cpu() {
            self.fatal(error);
        }
        self.clock.now_us()
    }
    fn fatal(&self, error: WorkError) -> ! {
        self.reservation.identity.fatal(error)
    }
}
/// !Send/!Sync executor: producers may eventually receive owned mailbox views,
/// never an Arc wrapping this Runtime or a foreign-CPU task-switch permission.
pub(super) struct KernelWork {
    executor: Executor<SLOTS>,
    reservation: Arc<Reservation>,
    /// The queue and an attached service domain share this same CPU owner.
    cpu: Arc<dyn Cpu>,
    native_clock: Option<ClockServices>,
    installed_clock: Option<&'static InstalledClock>,
    linux_execution: Cell<Option<NonNull<DriverExecution<'static>>>>,
    foreground: Arc<firmware::Foreground>,
    root: u64,
    busy: Cell<bool>,
    stack_pages: Option<pages::TaskPageInventory>,
}
struct ClearLinuxExecution<'a>(&'a Cell<Option<NonNull<DriverExecution<'static>>>>);

impl Drop for ClearLinuxExecution<'_> {
    fn drop(&mut self) {
        self.0.set(None);
    }
}

/// Owned closure/result storage. It is never made Send/Sync or exposed to an
/// AP; the real queue checks its BSP before the callback can touch these cells.
struct OwnedCall<F, T> {
    input: UnsafeCell<Option<F>>,
    output: UnsafeCell<Option<T>>,
    complete: AtomicBool,
}
unsafe extern "C" fn owned_call<F: FnOnce() -> T, T>(context: *mut c_void) {
    // SAFETY: call_owned admitted this exact owned box once on the real affine
    // runtime. It remains held through completion or permanent quarantine.
    let call = unsafe { &*context.cast::<OwnedCall<F, T>>() };
    // SAFETY: only this one callback takes the closure; the dispatcher waits
    // for its ticket completion before reading or freeing output.
    let Some(function) = (unsafe { &mut *call.input.get() }).take() else {
        return;
    };
    let output = function();
    // SAFETY: the same unique callback initializes output before publication.
    unsafe { *call.output.get() = Some(output) };
    call.complete.store(true, Ordering::Release);
}

impl KernelWork {
    /// # Safety
    /// Post-EBS BSP owns this immutable validated handoff and all loader
    /// allocations, before APs/producers run. Reserved stack bytes have never
    /// carried another frame or been lent to a device; source tables and all
    /// aliases remain held WB. No external root/cache writer overlaps.
    pub(super) unsafe fn prepare(
        handoff: &'static Handoff,
        owner: &mut HostOwner,
        installed_clock: Option<&'static InstalledClock>,
    ) -> Result<Self, Error> {
        let clock = installed_clock
            .map(InstalledClock::services)
            .transpose()
            .map_err(Error::Work)?;
        let stage = Cell::new("stack plan");
        // SAFETY: original caller's handoff/owner/stack lifetime contract.
        let mut work = Plan::new(handoff)
            .and_then(|plan| unsafe {
                Self::prepare_plan(handoff, owner, plan, clock.as_ref(), &stage)
            })
            .map_err(|error| Error::At(stage.get(), Box::new(error)))?;
        work.installed_clock = installed_clock;
        Ok(work)
    }
    /// # Safety
    /// Same original handoff/owner contract as prepare. A Plan describes
    /// only layout; it cannot supply allocation, root or execution permission.
    unsafe fn prepare_plan(
        handoff: &'static Handoff,
        owner: &mut HostOwner,
        plan: Plan,
        clock: Option<&ClockServices>,
        stage: &Cell<&'static str>,
    ) -> Result<Self, Error> {
        plan.validate(handoff)?;
        stage.set("firmware records");
        // SAFETY: inherited handoff/physmap lifetime and sealed capture.
        let wire = unsafe { native_mmio::captured(handoff) }.map_err(Error::Capture)?;
        stage.set("root identity");
        let identity = owner.root_identity().map_err(Error::Work)?;
        if let Some(clock) = clock {
            stage.set("service clock identity");
            clock.check_identity(&identity).map_err(Error::Work)?;
        }
        stage.set("boot processor");
        let snapshot = identity.memory_snapshot().map_err(Error::Work)?;
        // SAFETY: same immutable processor capture as native admission.
        let capture = unsafe { native_smp::firmware::captured(handoff.native_processors) }
            .map_err(|_| Error::Inventory)?;
        let bsp = capture.processors[capture.selected[0] as usize];
        // Both sides are the one CPUID topology decoding (leaf 0x1F/0xB
        // EDX, AMD 0x8000001E EAX, else leaf 1 EBX[31:24]) the loader
        // matched to this processor's MP-services ID at capture
        // (`Snapshot::matches`), at full width: only a different processor
        // differs here.
        let observed = snapshot.apic_id();
        if observed.map(u64::from) != Some(bsp.apic_id) {
            return Err(Error::WrongCpu {
                observed,
                captured: bsp.apic_id,
            });
        }
        stage.set("stack region");
        if handoff.tsc_hz < 1_000_000 {
            return Err(Error::Clock);
        }
        let region = plan.region();
        if region.physical == 0
            || region.bytes != plan.raw_bytes()
            || !wire.loader_ram(region.physical, region.bytes)
        {
            return Err(Error::Inventory);
        }
        // SAFETY: loader root, excluding task/native entry.
        let _root = unsafe { RestoreRoot::enter(handoff.root) };
        stage.set("engine image");
        // SAFETY: the same validated original handoff/loader root; source
        // allocation, guest exclusions and callback code stay pinned below.
        let image = unsafe { handoff.native_image.pin(handoff) }.map_err(Error::Image)?;
        let allocation = handoff.native_image.allocation_region();
        if !wire.loader_ram(allocation.physical, allocation.bytes) {
            return Err(Error::Image(crate::native_image::Error::NotLoaderData));
        }
        // SAFETY: original image/header aliases are kept and no writer runs.
        unsafe { image.check_callback(Self::prepare_plan as *const () as usize) }
            .map_err(Error::Image)?;
        let mut spans = Vec::new();
        spans
            .try_reserve_exact(plan.count())
            .map_err(|_| Error::Allocation)?;
        let mut previous = plan.virtual_range().start;
        let mut read = |table, index| {
            if !wire.table_owned(table) {
                return None;
            }
            // SAFETY: sealed kept LoaderData table; walker bounds
            // index to0..512 and this root exclusion admits no table writer.
            Some(unsafe {
                (layout::physical(table) as *const u64)
                    .add(index)
                    .read_volatile()
            })
        };
        stage.set("stack mappings and cache types");
        for index in 0..plan.count() {
            let (slot, physical) = plan.slot(index)?;
            mappings::absent(
                previous..slot.usable.start,
                identity.root_pages(),
                wire.levels(),
                &mut read,
            )
            .map_err(Error::Mapping)?;
            previous = slot.usable.end;
            mappings::qualify(
                &slot,
                physical,
                identity.root_pages(),
                wire.levels(),
                snapshot.pat,
                &mut read,
            )
            .map_err(Error::Mapping)?;
            // Check every physical slot, not the first MTRR interval.
            crate::native_memory::cache::verify_writeback(
                slot.usable.start,
                physical,
                (slot.usable.end - slot.usable.start) as usize,
                layout::physical(0),
            )
            .map_err(Error::Cache)?;
            spans.push((
                usize::try_from(slot.usable.start).map_err(|_| Error::Geometry)?
                    ..usize::try_from(slot.usable.end).map_err(|_| Error::Geometry)?,
                usize::try_from(slot.lower_guard.start).map_err(|_| Error::Geometry)?
                    ..usize::try_from(slot.lower_guard.end).map_err(|_| Error::Geometry)?,
                physical,
            ));
        }
        mappings::absent(
            previous..plan.virtual_range().end,
            identity.root_pages(),
            wire.levels(),
            &mut read,
        )
        .map_err(Error::Mapping)?;
        crate::native_memory::cache::verify_writeback(
            layout::physical(region.physical),
            region.physical,
            region.bytes as usize,
            layout::physical(0),
        )
        .map_err(Error::Cache)?;
        stage.set("stack claim");
        let allocation =
            handoff
                .native_task_allocations
                .claim(region)
                .map_err(|error| match error {
                    native_smp::resources::ClaimError::AlreadyOwned => Error::AlreadyOwned,
                    native_smp::resources::ClaimError::DescriptorMismatch => Error::Inventory,
                })?;
        // A failed/uncertain admission never reuses this pool.
        let reservation = Arc::new(Reservation {
            identity,
            _wire: wire,
            _handoff: handoff,
            _image: image,
            allocation,
            released_stack_views: AtomicUsize::new(0),
            pin_epochs: core::array::from_fn(|_| pin::Epoch::new()),
        });
        let cpu: Arc<dyn Cpu> = match clock {
            Some(clock) => clock.cpu(),
            None => Arc::new(NativeCpu {
                reservation: reservation.clone(),
                clock: Clock {
                    ticks_per_microsecond: handoff.tsc_hz / 1_000_000,
                },
            }),
        };
        stage.set("task pages");
        let stack_pages = (clock.is_some() || cfg!(feature = "diagnostics"))
            .then(|| pages::TaskPageInventory::from_spans(reservation.clone(), &spans))
            .transpose()?;
        let (foreground_slot, foreground_physical) = plan.slot(0)?;
        let foreground = Arc::new(firmware::Foreground {
            reservation: reservation.clone(),
            bounds: spans[0].0.clone(),
            lower: foreground_slot.lower_guard.start as usize
                ..foreground_slot.lower_guard.end as usize,
            upper: foreground_slot.upper_guard.start as usize
                ..foreground_slot.upper_guard.end as usize,
            physical: foreground_physical,
        });
        stage.set("continuation runtime");
        let mut stacks = Vec::new();
        stacks
            .try_reserve_exact(spans.len())
            .map_err(|_| Error::Allocation)?;
        if spans.len() > SLOTS {
            return Err(Error::Geometry);
        }
        for (slot, (bounds, guard, physical)) in spans.into_iter().enumerate() {
            stacks.push(
                Stack::new(Box::new(Backing {
                    _reservation: reservation.clone(),
                    bounds,
                    guard,
                    physical,
                    slot,
                }))
                .map_err(Error::Work)?,
            );
        }
        // SAFETY: real kept source/root/code ownership; callbacks cannot
        // outlive this boot, and no producer is published by construction.
        let queue = unsafe { Queue::<SLOTS>::new(cpu.clone(), reservation.clone()) }
            .map_err(Error::Work)?;
        let executor = Executor::new_with_profile(
            queue,
            stacks,
            work::ReservationPolicy::NativeFirmwareProtected {
                gpe: work::GpeQueueBudget::QUALIFICATION,
            },
            plan::profile()?,
        )
        .map_err(Error::Work)?;
        let result = Self {
            executor,
            reservation,
            cpu,
            native_clock: clock.cloned(),
            installed_clock: None,
            linux_execution: Cell::new(None),
            foreground,
            root: handoff.root,
            busy: Cell::new(false),
            stack_pages,
        };
        // Validation enters the owned-call boundary, which acquires the
        // global driver/root guard itself. No mapping walk or table borrower
        // survives here; release this non-reentrant construction guard first.
        drop(_root);
        stage.set("first foreground call");
        result.qualify_foreground()?;
        Ok(result)
    }
    /// Execute owned boot preparation in a real foreground continuation.
    /// Neither closure nor result may borrow the caller's stack. This is not
    /// asynchronous producer admission or a fixed-rate deadline-clock grant.
    /// An uncertain call keeps its whole job/runtime and closes this sync
    /// entry, so repeated callers cannot leak another job on every poll.
    pub(super) fn call_owned<F: FnOnce() -> T + 'static, T: 'static>(
        &self,
        timeout_us: u64,
        function: F,
    ) -> Result<T, Error> {
        self.reservation
            .identity
            .check_clock_root()
            .map_err(Error::Work)?;
        self.reservation
            .identity
            .check_dispatch()
            .map_err(Error::Work)?;
        let runtime = self.executor.runtime();
        match runtime.current_id() {
            Ok(_) => return Err(Error::Work(WorkError::WouldDeadlock)),
            Err(WorkError::Unavailable) => {}
            Err(error) => return Err(Error::Work(error)),
        }
        // SAFETY: loader root on this physical BSP, no native
        // or IRQ frame; every continuation was checked in this same domain.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        // This checks the original allocation/entered root link only;
        // it supplies no rate, timer, firmware-service or task admission grant.
        self.reservation
            .identity
            .check_clock_root()
            .map_err(Error::Work)?;
        let deadline = self.deadline_us(timeout_us)?;
        if self.busy.replace(true) {
            return Err(Error::Work(WorkError::Busy));
        }
        let call = Box::into_raw(Box::new(OwnedCall {
            input: UnsafeCell::new(Some(function)),
            output: UnsafeCell::new(None),
            complete: AtomicBool::new(false),
        }));
        // SAFETY: owned 'static closure/result, pinned runtime and
        // static monomorphized callback code. On error the owned box stays
        // kept; Executor also quarantines uncertain continuation frames.
        unsafe { runtime.call_foreground(owned_call::<F, T>, call.cast(), deadline) }
            .map_err(Error::Work)?;
        // SAFETY: allocation remains owned; the callback ticket completed.
        if !unsafe { &*call }.complete.load(Ordering::Acquire) {
            return Err(Error::Qualification);
        }
        // SAFETY: completed ticket excludes every callback borrower; this
        // function is the only owner allowed to reclaim this exact Rust box.
        let call = unsafe { Box::from_raw(call) };
        // SAFETY: completed callback published output and cannot run again.
        let result = unsafe { &mut *call.output.get() }
            .take()
            .ok_or(Error::Qualification)?;
        drop(call);
        self.busy.set(false);
        Ok(result)
    }
    /// Immutable native DATA from the same original relocated engine allocation
    /// and root graph retained by this Queue, never a second image producer.
    pub(super) fn image(&self) -> Result<Arc<ReadOnlyImage>, Error> {
        self.dispatch(|_| {
            // SAFETY: dispatch entered the actual retained LoaderTables root
            // on its owning BSP, excluding IRQ/native frames and table writers.
            let ranges =
                unsafe { self.reservation._image.readonly_ranges() }.map_err(Error::Image)?;
            if ranges.is_empty() {
                return Err(Error::Inventory);
            }
            Ok(Arc::new(ReadOnlyImage {
                _reservation: self.reservation.clone(),
                ranges,
            }))
        })?
    }
    fn deadline_us(&self, timeout: u64) -> Result<u64, Error> {
        let scale = if self.native_clock.is_some() {
            1_000
        } else {
            1
        };
        let ticks = timeout.checked_mul(scale).ok_or(Error::Clock)?;
        self.cpu.ticks().checked_add(ticks).ok_or(Error::Clock)
    }

    pub(super) fn linux_geometry(&self) -> Arc<dyn stela_linux::threads::OriginalKernelGeometry> {
        self.reservation.clone()
    }

    pub(super) fn linux_root(&self) -> Arc<dyn stela_linux::threads::OriginalKernelRoot> {
        self.reservation.clone()
    }

    pub(super) fn service_clock(&self) -> Result<&ClockServices, WorkError> {
        self.native_clock.as_ref().ok_or(WorkError::Unavailable)
    }

    /// Enter the original loader root and its installed IRQ execution
    /// scope for a Linux dispatcher segment.
    pub(super) fn dispatch_linux<T>(
        &'static self,
        body: impl FnOnce(Pin<&'static work::Runtime<SLOTS>>) -> Result<T, WorkError>,
    ) -> Result<T, Error> {
        let clock = self.installed_clock.ok_or(Error::Clock)?;
        let identity = &self.reservation.identity;
        identity.check_irq_cpu().map_err(Error::Work)?;
        if identity.in_interrupt() {
            return Err(Error::Work(WorkError::InterruptContext));
        }
        if self.linux_execution.get().is_some() {
            return Err(Error::Work(WorkError::Busy));
        }
        self.dispatch(|_| {
            // SAFETY: dispatch entered the kept loader root on the owning
            // BSP. This scope ends after every active frame returns or parks.
            let execution = unsafe { clock.enter_driver_execution()? };
            self.linux_execution.set(Some(NonNull::from(&execution)));
            let _clear = ClearLinuxExecution(&self.linux_execution);
            body(self.executor.runtime())
        })?
        .map_err(Error::Work)
    }

    /// Lend the actual dispatch segment for one synchronous Linux metadata leaf.
    /// # Safety
    /// The callback cannot switch or retain its process loan. An interrupt or
    /// a call outside the active dispatch segment cannot borrow the slot.
    pub(super) unsafe fn with_linux_process<T>(
        &self,
        body: impl FnOnce(&dyn work::EnteredProcess) -> T,
    ) -> Result<T, WorkError> {
        let identity = &self.reservation.identity;
        identity.check_irq_cpu()?;
        if identity.in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        let execution = self.linux_execution.get().ok_or(WorkError::Unavailable)?;
        // SAFETY: same CPU outside IRQ, and dispatch_linux clears the slot
        // before destroying the live execution. The leaf cannot suspend it.
        unsafe { execution.as_ref().with_process(body) }
    }

    /// Run `body` in the dispatcher, on the loader's root under the host
    /// drivers' lock, from anywhere the boot PE's run loop is: unlike
    /// [`Self::call_owned`], this also serves the run, whose tagged guest
    /// roots only the entered root's checks accept.
    pub(super) fn dispatch<T>(
        &self,
        body: impl FnOnce(Pin<&work::Runtime<SLOTS>>) -> T,
    ) -> Result<T, Error> {
        // SAFETY: the loader root on this physical BSP, with no native or IRQ
        // frame, which the checks below refuse.
        let _root = unsafe { RestoreRoot::enter(self.root) };
        let identity = &self.reservation.identity;
        identity.check_clock_root().map_err(Error::Work)?;
        identity.check_dispatch().map_err(Error::Work)?;
        Ok(body(self.executor.runtime()))
    }

    /// Continuation identity; dispatcher/IRQ/foreign-root callers refuse.
    pub(super) fn current_id(&self) -> Result<u64, Error> {
        self.executor
            .runtime()
            .current_id()
            .map(|id| id.get())
            .map_err(Error::Work)
    }
    /// Check the continuation and both OSL flush roles immediately before
    /// C removal. This neither captures queue boundaries nor synchronizes SCI.
    pub(super) fn check_firmware_wait(&self) -> Result<(), Error> {
        let runtime = self.executor.runtime();
        runtime.current_id().map_err(Error::Work)?;
        runtime
            .check_firmware_join(work::FirmwareQueue::Gpe)
            .map_err(Error::Work)?;
        runtime
            .check_firmware_join(work::FirmwareQueue::Notify)
            .map_err(Error::Work)
    }
    /// Run only on the real owned foreground stack, under the dispatcher's
    /// existing serialized original-root guard. No borrowed caller stack or
    /// process permission is captured or kept in a queued job.
    pub(super) fn call_firmware_owned<F, T>(&self, timeout_us: u64, function: F) -> Result<T, Error>
    where
        F: FnOnce(&super::FirmwareContext<'_>) -> T + 'static,
        T: 'static,
    {
        let stack = self.foreground.clone();
        let queue = self.executor.runtime().queue().clone();
        self.call_owned(timeout_us, move || {
            let context = FirmwareContext::enter(&stack, &queue)?;
            let output = function(&context);
            context.check()?;
            Ok(output)
        })?
    }
    fn qualify_foreground(&self) -> Result<(), Error> {
        let valid = self.call_firmware_owned(1_000_000, |context| context.id() != 0)?;
        if !valid {
            return Err(Error::Qualification);
        }
        Ok(())
    }
}
