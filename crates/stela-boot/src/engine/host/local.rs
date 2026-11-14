//! State owned by this physical host processor for its executor lifetime.
//! GS is private to Stela at CPL0, including translated execution and faults.
use super::State;
use crate::Result;
use stela_contracts::refuse;
pub(crate) mod context;
use core::{
    arch::asm,
    cell::UnsafeCell,
    marker::PhantomData,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU32, AtomicU64, AtomicUsize, Ordering},
};

#[repr(C, align(64))]
pub(crate) struct Local {
    this: *mut Local,
    /// The heap's name for this processor's cache.
    heap_core: UnsafeCell<u32>,
    /// Native assembly and the local recovery path share this one slot.
    recovery_rsp: UnsafeCell<u64>,
    handler: super::super::interrupts::Handler,
    entry_tables: *const super::super::interrupts::Tables,
    state: UnsafeCell<Option<State>>,
    interrupted: AtomicBool,
    xstate: super::super::xstate::EntryAbi,
    xstate_active: UnsafeCell<[u64; 2]>,
    helper_rsp: UnsafeCell<u64>,
    doorbells: AtomicU64,
    /// This processor's aperture shootdown slot plus one, zero when it has
    /// none, and the latest shootdown generation it has flushed.
    shootdown_slot: AtomicUsize,
    shootdown_flushed: AtomicU64,
    worker: UnsafeCell<Option<stela_engine::machine::engine::publication::WorkerId>>,
    declarations: UnsafeCell<Option<stela_engine::NativeCpuOwner>>,
    /// This processor's place in the order they became active.
    index: u32,
}

impl Local {
    pub(crate) const fn new() -> Self {
        Self {
            this: core::ptr::null_mut(),
            heap_core: UnsafeCell::new(0),
            recovery_rsp: UnsafeCell::new(0),
            handler: super::super::interrupts::interrupt,
            entry_tables: core::ptr::null(),
            state: UnsafeCell::new(None),
            interrupted: AtomicBool::new(false),
            xstate: super::super::xstate::EntryAbi::EMPTY,
            xstate_active: UnsafeCell::new([0; 2]),
            helper_rsp: UnsafeCell::new(0),
            doorbells: AtomicU64::new(0),
            shootdown_slot: AtomicUsize::new(0),
            shootdown_flushed: AtomicU64::new(0),
            worker: UnsafeCell::new(None),
            declarations: UnsafeCell::new(None),
            index: 0,
        }
    }
}

/// A CPU-local state allocation. It cannot migrate or be shared with another
/// host processor. The owner must outlive every local native call and handler.
#[must_use = "retain the physical native host owner through every call and handler"]
pub(crate) struct Owner {
    local: NonNull<Local>,
    clock_leases: alloc::sync::Arc<()>,
    root_context: alloc::sync::Weak<context::RootContext>,
    root_identity: Option<alloc::sync::Arc<context::RootIdentity>>,
    _pinned: PhantomData<*mut ()>,
}

/// Fixed root pages and the only RAM the shadow publisher may map.
/// This immutable view pins the original physical owner; it does not move it.
pub(crate) struct MmioRoots {
    roots: alloc::vec::Vec<u64>,
    ram: alloc::vec::Vec<stela_drivers::bus::mmio::owned::PhysicalSpan>,
    _lease: alloc::sync::Arc<()>,
}
impl MmioRoots {
    pub(crate) fn roots(&self) -> &[u64] {
        &self.roots
    }
    pub(crate) fn ram(&self) -> &[stela_drivers::bus::mmio::owned::PhysicalSpan] {
        &self.ram
    }
}

impl Owner {
    /// Keep immutable clock/CPU views without moving this !Send owner. The
    /// owner refuses destruction while any view can still enter it.
    pub(crate) fn retain_clock_root(
        &mut self,
    ) -> Result<alloc::sync::Arc<()>, stela_drivers::platform::work::WorkError> {
        use stela_drivers::platform::work::WorkError;
        if current() != self.local.as_ptr() {
            return Err(WorkError::WrongCpu);
        }
        if irq_active() || !native_frames_idle() {
            return Err(WorkError::Busy);
        }
        let root: u64;
        // SAFETY: read-only observation under this physical processor's owner.
        unsafe {
            asm!("mov {}, cr3", out(reg) root, options(nomem, nostack, preserves_flags));
        }
        let state = state();
        if state.native || root != state.host_root {
            return Err(WorkError::corrupt());
        }
        Ok(self.clock_leases.clone())
    }

    /// Lay the guest RAM out from `base` (`ram::place`), keeping its host
    /// backing, before a guest regime, frame or registered RAM exists: the
    /// host drivers that read a bundle's DRAM base run after `install`.
    /// Root contexts and MMIO views see only the host spans, which stay.
    pub(crate) fn place_ram(&mut self, base: u64) -> Result<()> {
        if current() != self.local.as_ptr() {
            return Err(refuse!("native Host owner migrated"));
        }
        let state = state();
        if state.native || state.regime.is_some() || state.registered_ram.is_some() {
            return Err(refuse!(
                "guest RAM is placed before the guest's memory is registered"
            ));
        }
        super::ram::place(&mut state.ram, base);
        Ok(())
    }

    /// Adopt the existing canonical owner before a guest regime or frame is
    /// live. This replaces metadata; it allocates/copies no guest RAM.
    ///
    /// # Safety
    /// This is the source-checked registered RAM for this executor. Its
    /// all-writer contract is held. Direct `SYSTEM_MODE` guest data writers
    /// must never bypass this all-writer owner, including between checked calls.
    /// Only the source-checked checked service and registered host/table
    /// publishers may touch data; returning to serial execution additionally
    /// requires the canonical reservation/event ownership join.
    pub(crate) unsafe fn register_system_ram(
        &mut self,
        ram: alloc::sync::Arc<stela_engine::memory::RegisteredSystemRam>,
    ) -> Result<()> {
        if current() != self.local.as_ptr() {
            return Err(refuse!("native Host owner migrated"));
        }
        let state = state();
        if self.root_context.strong_count() != 0
            || state.native
            || state.regime.is_some()
            || state.registered_ram.is_some()
        {
            return Err(refuse!(
                "native RAM adoption requires pristine owner-local dispatch"
            ));
        }
        let mut incoming = alloc::vec::Vec::new();
        incoming
            .try_reserve_exact(ram.placements().count())
            .map_err(|_| refuse!("canonical RAM placement allocation failed"))?;
        incoming.extend(
            ram.placements()
                .map(|(guest, host, bytes)| super::RamRange {
                    guest,
                    host,
                    bytes: bytes as u64,
                }),
        );
        // End the mutable state borrow before checking the same root
        // identity and closing old RAM-absence/publication generations.
        let _ = state;
        self.join_ram_identity(&incoming)?;
        let state = self::state();
        state.ram = incoming;
        state.registered_ram = Some(ram);
        Ok(())
    }

    /// Bind this selected physical core once, before runtime dispatch.
    pub(crate) fn bind_worker(
        &mut self,
        worker: stela_engine::machine::engine::publication::WorkerId,
        expected_apic: u32,
    ) -> Result<()> {
        if current() != self.local.as_ptr() {
            return Err(refuse!("native Host owner migrated"));
        }
        let state = state();
        if state.apic.id() != expected_apic {
            return Err(refuse!("native Host physical identity differs"));
        }
        if state.native || state.regime.is_some() {
            return Err(refuse!(
                "native Host worker binding requires pristine dispatch"
            ));
        }
        // SAFETY: this unique processor-local owner has no active native call.
        if unsafe { *self.local.as_ref().recovery_rsp.get() } != 0 {
            return Err(refuse!(
                "native Host worker binding has active recovery frame"
            ));
        }
        // SAFETY: this processor's unique owner, before any native dispatch;
        // handler code never changes the immutable runtime worker binding.
        let binding = unsafe { &mut *self.local.as_ref().worker.get() };
        if binding.is_some() {
            return Err(refuse!("native Host worker already bound"));
        }
        // SAFETY: install's owned post-EBS platform contract keeps
        // destination affinity and stable declarations through retirement.
        // BSP canonical preparation and AP Initial both arrive here only after
        // their local Host/control installation. No firmware snapshot or BSP
        // observation mints an AP lease. Hardware validation stays separate.
        let declarations = unsafe { stela_engine::NativeCpuOwner::new(worker, expected_apic) }
            .map_err(|_| refuse!("native Host CPU declaration admission refused"))?;
        // SAFETY: same unique processor-local owner, no active dispatch.
        let slot = unsafe { &mut *self.local.as_ref().declarations.get() };
        if slot.is_some() {
            return Err(refuse!("native Host CPU declarations already owned"));
        }
        *slot = Some(declarations);
        *binding = Some(worker);
        Ok(())
    }

    /// Borrow the sole host handle while keeping its processor-local owner.
    pub(crate) fn host(&mut self) -> super::BareHost<'_> {
        assert_eq!(current(), self.local.as_ptr(), "host state owner migrated");
        super::BareHost {
            _owner: PhantomData,
            _pinned: PhantomData,
        }
    }
}

const _: () = assert!(core::mem::offset_of!(Local, this) == 0);
const _: () =
    assert!(core::mem::offset_of!(Local, heap_core) == stela_boot_core::heap::CORE_RECORD);
pub(super) const RECOVERY_RSP_OFFSET: usize = core::mem::offset_of!(Local, recovery_rsp);
pub(crate) const XSTATE_IMAGE_BYTES_OFFSET: usize = core::mem::offset_of!(Local, xstate)
    + core::mem::offset_of!(super::super::xstate::EntryAbi, image_bytes);
pub(crate) const XSTATE_SUPPORTED_MASK_OFFSET: usize = core::mem::offset_of!(Local, xstate)
    + core::mem::offset_of!(super::super::xstate::EntryAbi, supported_mask);
pub(crate) const XSTATE_AVAILABLE_OFFSET: usize = core::mem::offset_of!(Local, xstate)
    + core::mem::offset_of!(super::super::xstate::EntryAbi, xsave_available);
pub(crate) const XSTATE_EXTENDED_OFFSET: usize = core::mem::offset_of!(Local, xstate)
    + core::mem::offset_of!(super::super::xstate::EntryAbi, extended);
pub(crate) const XSTATE_ACTIVE_OFFSET: usize = core::mem::offset_of!(Local, xstate_active);
pub(super) const HELPER_RSP_OFFSET: usize = core::mem::offset_of!(Local, helper_rsp);

/// Install only a destination capture before making this Local/IDT
/// active. A caller-supplied mask/size cannot create this contract.
///
/// # Safety
/// The descriptor owner keeps `owner` and the permanently pinned
/// normal/fatal IST allocations while any vector can enter this Local. It
/// checked each real stack's frame/image/handler budget and all-root mappings.
/// No returning same-IST nested entry is admitted. This does not enable state.
pub(crate) unsafe fn configure_xstate(
    local: &mut Local,
    owner: &super::super::xstate::XstateOwner,
) -> Result<(), super::super::xstate::LayoutError> {
    if !local.this.is_null() || local.xstate.image_bytes != 0 {
        return Err(super::super::xstate::LayoutError::AlreadyInstalled);
    }
    let format = owner.check()?;
    local.xstate = *owner.abi();
    local.xstate.extended = u64::from(format != super::super::xstate::SaveFormat::Fxsave);
    Ok(())
}

/// How many processors have become active.
static ACTIVE: AtomicU32 = AtomicU32::new(0);

unsafe fn set_base(base: u64) {
    // SAFETY: the executor owns this processor's GS, and its descriptor/IST
    // setup has already finished. The base remains mapped in every root.
    unsafe { crate::cpu::msr::GS_BASE.write(base) };
}

/// # Safety
/// Called once for this processor, with interrupts disabled. `local` is private,
/// pinned and mapped through every root until this processor is permanently
/// stopped; its descriptor owner keeps it after this call.
pub(crate) unsafe fn activate(
    local: &mut Local,
    handler: super::super::interrupts::Handler,
    tables: *const super::super::interrupts::Tables,
) {
    assert!(
        local.this.is_null(),
        "host processor local record already active"
    );
    assert!(
        local.xstate.image_bytes != 0,
        "processor xstate descriptor owner absent"
    );
    local.handler = handler;
    local.entry_tables = tables;
    local.index = ACTIVE.fetch_add(1, Ordering::Relaxed);
    local.this = local;
    // SAFETY: the descriptor owner pins this private record for the CPU lifetime.
    unsafe { set_base(local as *mut Local as u64) };
    // SAFETY: the pinned record at this processor's new GS base, and no
    // processor allocates before it gets here: the boot processor's first
    // is this call, and an AP's entry reaches here first.
    unsafe { crate::ALLOCATOR.tag_cpu(local.heap_core.get()) };
}

/// # Safety
/// This processor's descriptor/local record is installed, interrupts are
/// disabled, and the returned owner outlives every local native call/handler.
/// The owned post-EBS mount requires trusted firmware/any VMM to preserve
/// relevant CPU declarations and identity while this owner is live. Retire
/// before physical reset/resume, microcode or feature reconfiguration (including
/// SMT-shared changes); re-enumerate on the destination before rebinding.
pub(super) unsafe fn install(state: State) -> Owner {
    let local = NonNull::new(current()).expect("host processor local record absent");
    // SAFETY: this is the sole initialization under the current CPU owner.
    let slot = unsafe { &mut *local.as_ref().state.get() };
    assert!(
        slot.is_none(),
        "host state already installed on this processor"
    );
    *slot = Some(state);
    Owner {
        local,
        clock_leases: alloc::sync::Arc::new(()),
        root_context: alloc::sync::Weak::new(),
        root_identity: None,
        _pinned: PhantomData,
    }
}

fn current() -> *mut Local {
    let pointer: *mut Local;
    // SAFETY: the caller keeps the local Owner. Guest translations do not
    // change GS, and every engine/shadow root maps this private allocation.
    unsafe {
        asm!("mov {}, gs:[0]", out(reg) pointer, options(readonly, nostack, preserves_flags))
    };
    pointer
}

/// This processor's GS base, read through its record's self pointer rather
/// than with RDMSR. Once [`activate`] has run on it: from then on GS is the
/// record, whose `this` names it.
pub(crate) fn activated_base() -> u64 {
    current() as u64
}

/// This processor's index, dense from zero. Once [`activate`] has run on it.
/// The BSP installs its record before any AP starts.
pub(crate) fn processor() -> u32 {
    // SAFETY: the record is pinned for the processor's life, and `index` is set
    // before `this`, which `current` reads through.
    unsafe { (*current()).index }
}

pub(super) fn state() -> &'static mut State {
    installed().expect("host state installed")
}

/// This core's state, while one is installed.
pub(super) fn installed() -> Option<&'static mut State> {
    // SAFETY: execution is pinned to the Owner's host CPU. Outside native
    // code interrupts are disabled and calls never overlap; native execution
    // keeps no State reference while a fault/helper borrows it.
    unsafe { (&mut *(*current()).state.get()).as_mut() }
}

/// Current-owner slots only; numbers or copied records cannot prove
/// that a private fixture may release mapped backing after a continuation.
pub(super) fn native_frames_idle() -> bool {
    // SAFETY: as for state(): caller keeps this processor's Owner,
    // IF is clear and no frame/handler overlaps this process boundary.
    unsafe {
        let local = &*current();
        *local.recovery_rsp.get() == 0 && *local.helper_rsp.get() == 0
    }
}

/// Normal/terminal hardware-entry metadata set by the shared IDT asm.
/// IF does not identify IRQ context; process IRQ-save still reads false here.
pub(super) fn irq_active() -> bool {
    let normal: u64;
    let terminal: u64;
    // SAFETY: permanently pinned current GS record; assembly owns the slots,
    // setting before Rust entry and clearing only after full xstate restore.
    unsafe {
        asm!("mov {}, gs:[{}]", out(reg) normal, const XSTATE_ACTIVE_OFFSET,
            options(readonly, nostack, preserves_flags));
        asm!("mov {}, gs:[{}]", out(reg) terminal, const XSTATE_ACTIVE_OFFSET + 8,
            options(readonly, nostack, preserves_flags));
    }
    normal != 0 || terminal != 0
}

pub(super) fn interrupt() {
    // SAFETY: one processor owns this record for the handler's whole lifetime.
    unsafe { &(*current()).interrupted }.store(true, Ordering::Release);
}

pub(super) fn doorbell_counter() -> &'static AtomicU64 {
    // SAFETY: descriptor/local storage is permanently pinned through parked
    // NMI/MCE entry. Only the current physical processor selects this record.
    unsafe { &(*current()).doorbells }
}

/// This processor's aperture shootdown slot and the generation it flushed.
pub(super) fn shootdown() -> (&'static AtomicUsize, &'static AtomicU64) {
    // SAFETY: as in doorbell_counter.
    let local = unsafe { &*current() };
    (&local.shootdown_slot, &local.shootdown_flushed)
}

pub(super) fn doorbell() {
    doorbell_counter().fetch_add(1, Ordering::Release);
    interrupt();
}

pub(super) fn interrupted() -> bool {
    // SAFETY: as in interrupt.
    unsafe { &(*current()).interrupted }.load(Ordering::Acquire)
}

pub(super) fn clear_interrupt() {
    // SAFETY: as in interrupt; the atomic is separate from mutable host state.
    unsafe { &(*current()).interrupted }.store(false, Ordering::Relaxed);
}

impl Drop for Owner {
    fn drop(&mut self) {
        assert_eq!(current(), self.local.as_ptr(), "host state owner migrated");
        if let Some(identity) = self.root_identity.take() {
            if alloc::sync::Arc::strong_count(&identity) != 1 {
                crate::output::fatal("native host CPU/root identity retained by live providers");
            }
            drop(identity);
        }
        if alloc::sync::Arc::strong_count(&self.clock_leases) != 1 {
            crate::output::fatal("native host root retained by live clock/CPU providers");
        }
        // SAFETY: the owner is processor-local, interrupts are disabled, and
        // the caller has stopped every execution/handler using this record.
        unsafe {
            let local = self.local.as_ref();
            // Revoke before any state/arena destruction, even if a stale
            // non-Send handle kept this owner incarnation.
            if let Some(owner) = (&mut *local.declarations.get()).take() {
                drop(owner);
            }
            *local.worker.get() = None;
            let slot = &mut *local.state.get();
            let state = slot.as_mut().expect("host state owner installed");
            assert!(
                !state.native && *local.recovery_rsp.get() == 0 && *local.helper_rsp.get() == 0,
                "active native host state dropped"
            );
            state
                .apic
                .write(crate::apic::LVT_TIMER, crate::apic::LVT_MASKED);
            state.apic.write(crate::apic::INITIAL, 0);
            // Descriptor/IST/local storage stays pinned even after host state
            // retires: NMI/MCE can still need it while this processor is parked.
            slot.take();
        }
    }
}

pub(super) fn native_worker() -> Option<stela_engine::machine::engine::publication::WorkerId> {
    // SAFETY: callers keep this processor's live Owner/GS record. The worker
    // binding is set once by that owner after checking its real local APIC ID.
    unsafe { *(*current()).worker.get() }
}

/// Lend only the current GS owner's destination-local declaration handle.
pub(super) fn native_cpu_lease() -> Option<stela_engine::NativeCpuLease> {
    // SAFETY: callers keep this processor's live exclusive HostOwner;
    // retirement revokes before destroying state, and handlers never lend it.
    unsafe {
        (&*(*current()).declarations.get())
            .as_ref()
            .map(|owner| owner.lease())
    }
}

/// The original permanently pinned descriptor/IST owner of this GS.
/// This never resolves an arbitrary address supplied by a caller.
pub(crate) fn interrupt_owner() -> (
    &'static super::super::interrupts::Tables,
    super::super::interrupts::Handler,
    [u64; 2],
) {
    let pointer = current();
    assert!(!pointer.is_null(), "interrupt local owner absent");
    // SAFETY: only the installed assembly interrupt dispatch calls this while
    // its active IST slot is held. activate records the original Tables
    // allocation, which install_on pins/maps for the entire CPU lifetime.
    let local = unsafe { &*pointer };
    assert!(
        !local.entry_tables.is_null(),
        "interrupt descriptor owner absent"
    );
    // SAFETY: same original permanently pinned allocation, never user memory.
    let tables = unsafe { &*local.entry_tables };
    assert!(
        tables.owns_local(pointer),
        "interrupt GS differs from its original descriptor owner"
    );
    // SAFETY: local CPU's assembly owns these slots; dispatch has IF clear.
    (tables, local.handler, unsafe { *local.xstate_active.get() })
}
