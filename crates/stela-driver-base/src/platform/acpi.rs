//! Native x86 firmware AML via pinned ACPICA, separate from the ARM guest ACPI.
//!
//! The platform supplies owned OSL services. There is no implicit identity map,
//! interrupt routing, EC driver, scheduler, allocator, or permissive I/O fallback.
//! Every OSL callback in [`Services`] is mandatory: real allocation,
//! IRQ-save locks, counting semaphores, owned mapping leases, hardware
//! access, monotonic 100 ns time, nonzero thread IDs, deferred work and
//! draining. A fallible unsupported operation returns the exact ACPICA
//! error and a void one completes or fails closed; providers must not
//! unwind. Shutdown closes work admission, quiesces the sources and drains
//! work before ACPICA terminates; if quiescence fails, provider and core
//! state stay quarantined.
mod io_window;
pub mod processor;
pub mod resources;
use alloc::{boxed::Box, ffi::CString, string::String, sync::Arc, vec::Vec};
use core::{
    cell::{Cell, RefCell},
    ffi::{CStr, c_char, c_void},
    marker::PhantomData,
    ptr,
    sync::atomic::{AtomicBool, AtomicUsize, Ordering},
};
pub use io_window::IoWindow;

pub type Status = u32;
pub type Interrupt = unsafe extern "C" fn(*mut c_void) -> u32;
pub type Work = unsafe extern "C" fn(*mut c_void);
pub type Notify = unsafe extern "C" fn(*mut c_void, u32, *mut c_void);
pub type Region = unsafe extern "C" fn(u32, u64, u32, *mut u64, *mut c_void, *mut c_void) -> Status;
pub type RegionSetup =
    unsafe extern "C" fn(*mut c_void, u32, *mut c_void, *mut *mut c_void) -> Status;
type GpeHandler = unsafe extern "C" fn(*mut c_void, u32, *mut c_void) -> u32;

/// ACPICA's fixed events (`ACPI_EVENT_*`).
pub const EVENT_POWER_BUTTON: u32 = 2;
/// `ACPI_INTERRUPT_HANDLED`.
const INTERRUPT_HANDLED: u32 = 1;
/// `ACPI_GPE_LEVEL_TRIGGERED` and `ACPI_GPE_EDGE_TRIGGERED`.
const GPE_LEVEL: u32 = 0x08;
const GPE_EDGE: u32 = 0x00;
/// `ACPI_DEVICE_NOTIFY` and `ACPI_ALL_NOTIFY`: notification values from
/// 0x80, and every value.
const DEVICE_NOTIFY: u32 = 2;
/// The `_DSD` device-properties UUID, daffd814-6eba-4d8c-8a91-bc9bbf4aa301.
const DEVICE_PROPERTIES: [u8; 16] = [
    0x14, 0xd8, 0xff, 0xda, 0xba, 0x6e, 0x8c, 0x4d, 0x8a, 0x91, 0xbc, 0x9b, 0xbf, 0x4a, 0xa3, 0x01,
];
const ALL_NOTIFY: u32 = 3;
/// `_OSC` status DWORD bits.
const OSC_QUERY_ENABLE: u32 = 1;
const OSC_REQUEST_ERROR: u32 = 2;
const OSC_INVALID_UUID_ERROR: u32 = 4;
const OSC_INVALID_REVISION_ERROR: u32 = 8;
const OSC_CAPABILITIES_MASK_ERROR: u32 = 16;

/// Exact ACPICA status, without collapsing firmware errors into generic success.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error(pub Status);
pub const SUPPORT: Error = Error(0x000f);
pub const NOT_FOUND: Error = Error(0x0005);
pub const BAD_PARAMETER: Error = Error(0x1001);
pub const ALREADY_EXISTS: Error = Error(0x0007);
pub const LIMIT: Error = Error(0x0010);
fn checked(status: Status) -> Result<(), Error> {
    if status == 0 {
        Ok(())
    } else {
        Err(Error(status))
    }
}

/// Hardware capabilities attested by the native provider, required as a set.
/// SCI includes correct FADT routing, shared level interrupts, mask/synchronize;
/// work includes deferred IRQ-safe enqueue and draining; regions includes
/// owned EC/GPIO/serial-bus handlers installed before dependent firmware methods;
/// allocation includes all OSL allocations from interrupt/deferred contexts.
pub const CAP_SCI: u64 = 1;
pub const CAP_DEFERRED_WORK: u64 = 2;
pub const CAP_REGIONS: u64 = 4;
pub const CAP_IRQ_ALLOC: u64 = 8;
pub const FULL_CAPABILITIES: u64 = CAP_SCI | CAP_DEFERRED_WORK | CAP_REGIONS | CAP_IRQ_ALLOC;

#[repr(C)]
#[derive(Clone, Copy, Debug, Default)]
pub struct PciId {
    pub segment: u16,
    pub bus: u16,
    pub device: u16,
    pub function: u16,
}

/// Version of the complete Rust/C OSL callback table. Format validation grants
/// no hardware capability; each callback still requires its ownership.
pub const SERVICES_ABI_VERSION: u32 = 2;
pub const SERVICES_ABI_BYTES: u32 = core::mem::size_of::<Services>() as u32;

/// Owned C-ABI native service provider. All functions are mandatory.
///
/// # Safety
/// Transfer `context` ownership to [`Session::start`]. All callbacks and context
/// must remain valid until `destroy`; failed quiescence keeps them
/// permanently. Each callback must implement the corresponding ACPICA OSL contract
/// (including timeout units, IRQ flags, locking, memory attributes, map leases,
/// unique nonzero thread IDs, 100 ns monotonic timer and callback draining).
/// `allocate` returns at least 16-byte aligned storage, `free` accepts its pointers.
/// `map` owns each exact address/length lease until matching `unmap`, including
/// overlapping/repeated table mappings; firmware RAM and MMIO require appropriate
/// cache attributes and lifetime after ExitBootServices. `read_memory` and
/// `write_memory` serve hardware registers and every AML SystemMemory field
/// access, which never goes through `map`. Lock acquisition disables
/// local interrupts and restores their previous flags on release. Callbacks must
/// not panic/unwind. Unsupported fallible operations return a real failure status;
/// mandatory void operations must complete, never silently do nothing.
/// Semaphore timeout is milliseconds (`0` polls, `0xffff` waits forever), sleep
/// is milliseconds and stall is microseconds; I/O widths are bits. `execute`
/// queues work, including from SCI, without running it inline, preserving required
/// ordering (in particular FIFO Notify), with no drain deadlock on worker context.
/// `quiesce` closes this instance's work admission (new execute calls fail), masks
/// and synchronizes SCI and region callbacks, then drains all admitted work,
/// leaving all providers valid for ACPICA's remove/free calls during termination.
/// Native table deletion must occur outside active AML frames/interpreter callbacks.
/// Pre-delete quiescence excludes its current callback while synchronizing all other
/// producers/work. Failed pre-delete quiescence invokes nonreturning `fatal`.
/// `destroy` runs only after successful termination. Hardware capability bits must
/// attest real installed services, never aspirational support.
#[repr(C)]
#[derive(Clone, Copy)]
pub struct Services {
    pub abi_version: u32,
    pub abi_bytes: u32,
    pub context: *mut c_void,
    pub root: u64,
    pub capabilities: u64,
    pub allocate: unsafe extern "C" fn(*mut c_void, u64) -> *mut c_void,
    pub free: unsafe extern "C" fn(*mut c_void, *mut c_void),
    pub map: unsafe extern "C" fn(*mut c_void, u64, u64) -> *mut c_void,
    pub unmap: unsafe extern "C" fn(*mut c_void, *mut c_void, u64),
    pub create_lock: unsafe extern "C" fn(*mut c_void, *mut *mut c_void) -> Status,
    pub delete_lock: unsafe extern "C" fn(*mut c_void, *mut c_void),
    pub acquire_lock: unsafe extern "C" fn(*mut c_void, *mut c_void) -> u64,
    pub release_lock: unsafe extern "C" fn(*mut c_void, *mut c_void, u64),
    pub create_semaphore: unsafe extern "C" fn(*mut c_void, u32, u32, *mut *mut c_void) -> Status,
    pub delete_semaphore: unsafe extern "C" fn(*mut c_void, *mut c_void) -> Status,
    pub wait_semaphore: unsafe extern "C" fn(*mut c_void, *mut c_void, u32, u16) -> Status,
    pub signal_semaphore: unsafe extern "C" fn(*mut c_void, *mut c_void, u32) -> Status,
    pub install_interrupt: unsafe extern "C" fn(*mut c_void, u32, Interrupt, *mut c_void) -> Status,
    pub remove_interrupt: unsafe extern "C" fn(*mut c_void, u32, Interrupt) -> Status,
    pub thread_id: unsafe extern "C" fn(*mut c_void) -> u64,
    pub execute: unsafe extern "C" fn(*mut c_void, u32, Work, *mut c_void) -> Status,
    /// Complete SCI synchronization and the GPE then Notify entry-boundary
    /// flushes. Failure reaches a nonreturning C poison/fatal boundary.
    pub drain_work: unsafe extern "C" fn(*mut c_void) -> Status,
    pub sleep: unsafe extern "C" fn(*mut c_void, u64),
    pub stall: unsafe extern "C" fn(*mut c_void, u32),
    pub read_port: unsafe extern "C" fn(*mut c_void, u64, *mut u32, u32) -> Status,
    pub write_port: unsafe extern "C" fn(*mut c_void, u64, u32, u32) -> Status,
    pub read_memory: unsafe extern "C" fn(*mut c_void, u64, *mut u64, u32) -> Status,
    pub write_memory: unsafe extern "C" fn(*mut c_void, u64, u64, u32) -> Status,
    pub read_pci: unsafe extern "C" fn(*mut c_void, *mut PciId, u32, *mut u64, u32) -> Status,
    pub write_pci: unsafe extern "C" fn(*mut c_void, *mut PciId, u32, u64, u32) -> Status,
    pub timer: unsafe extern "C" fn(*mut c_void) -> u64,
    pub signal: unsafe extern "C" fn(*mut c_void, u32, *mut c_void) -> Status,
    pub log: unsafe extern "C" fn(*mut c_void, *const c_char, u64),
    pub flush_cache: unsafe extern "C" fn(*mut c_void),
    pub quiesce: unsafe extern "C" fn(*mut c_void) -> Status,
    pub destroy: unsafe extern "C" fn(*mut c_void),
    /// Mandatory fail-closed kernel fatal path; called when pre-delete callback
    /// quiescence fails. It must never return, unwind, or invent completion.
    pub fatal: unsafe extern "C" fn(*mut c_void, Status) -> !,
    /// Side-effect-free context/role check before C removal starts.
    /// Success is neither a durable permission nor completion of either flush.
    pub preflight_wait_events: unsafe extern "C" fn(*mut c_void) -> Status,
}

#[repr(C)]
#[derive(Clone, Copy)]
struct RawObject {
    kind: u32,
    length: u32,
    payload: u64,
    extra: u64,
}
#[repr(C)]
struct RawBuffer {
    length: u64,
    pointer: *mut c_void,
}
unsafe extern "C" {
    fn stela_acpi_start(services: *const Services, hardware: u8) -> Status;
    fn stela_acpi_stop() -> Status;
    fn stela_acpi_preflight_wait_events() -> Status;
    fn stela_acpi_enable_hardware() -> Status;
    fn stela_acpi_guard_enter(out: *mut *mut c_void, epoch: *mut u64) -> Status;
    fn stela_acpi_guard_leave(guard: *mut c_void);
    fn stela_acpi_namespace_valid() -> u8;
    fn stela_acpi_bind_node(candidate: *mut c_void, epoch: u64) -> Status;
    fn stela_acpi_retain_context(
        before_termination: unsafe extern "C" fn(*mut c_void) -> Status,
        retire: unsafe extern "C" fn(*mut c_void) -> Status,
        activate: Option<unsafe extern "C" fn(*mut c_void) -> Status>,
        context: *mut c_void,
    ) -> Status;
    fn stela_acpi_evaluate(
        handle: *mut c_void,
        path: *const c_char,
        args: *mut RawObject,
        count: u32,
        result: *mut RawBuffer,
    ) -> Status;
    fn stela_acpi_walk(
        hid: *const c_char,
        callback: unsafe extern "C" fn(*mut c_void, u32, *mut c_void, *mut *mut c_void) -> Status,
        context: *mut c_void,
    ) -> Status;
    fn stela_acpi_walk_processors(
        callback: unsafe extern "C" fn(*mut c_void, u32, *mut c_void, *mut *mut c_void) -> Status,
        context: *mut c_void,
    ) -> Status;
    fn stela_acpi_processor_id(handle: *mut c_void, id: *mut u32) -> Status;
    fn acpi_get_name(handle: *mut c_void, kind: u32, buffer: *mut RawBuffer) -> Status;
    fn acpi_get_handle(parent: *mut c_void, path: *const c_char, out: *mut *mut c_void) -> Status;
    fn acpi_os_free(pointer: *mut c_void);
    fn acpi_acquire_global_lock(timeout_ms: u16, token: *mut u32) -> Status;
    fn acpi_release_global_lock(token: u32) -> Status;
    fn acpi_install_notify_handler(
        handle: *mut c_void,
        kind: u32,
        callback: Notify,
        context: *mut c_void,
    ) -> Status;
    fn acpi_remove_notify_handler(handle: *mut c_void, kind: u32, callback: Notify) -> Status;
    fn acpi_os_wait_events_complete();
    fn acpi_get_sleep_type_data(state: u8, a: *mut u8, b: *mut u8) -> Status;
    fn acpi_enter_sleep_state_prep(state: u8) -> Status;
    fn acpi_install_fixed_event_handler(
        event: u32,
        handler: Interrupt,
        context: *mut c_void,
    ) -> Status;
    fn acpi_install_gpe_raw_handler(
        block: *mut c_void,
        number: u32,
        kind: u32,
        handler: GpeHandler,
        context: *mut c_void,
    ) -> Status;
    fn acpi_clear_gpe(block: *mut c_void, number: u32) -> Status;
    fn acpi_enable_gpe(block: *mut c_void, number: u32) -> Status;
    fn acpi_disable_gpe(block: *mut c_void, number: u32) -> Status;
    fn acpi_mark_gpe_for_wake(block: *mut c_void, number: u32) -> Status;
    fn acpi_setup_gpe_for_wake(device: *mut c_void, block: *mut c_void, number: u32) -> Status;
    fn acpi_update_all_gpes() -> Status;
    fn acpi_install_address_space_handler(
        handle: *mut c_void,
        space: u32,
        handler: Region,
        setup: RegionSetup,
        context: *mut c_void,
    ) -> Status;
    fn acpi_remove_address_space_handler(
        handle: *mut c_void,
        space: u32,
        handler: Region,
    ) -> Status;
    fn acpi_install_address_space_handler_no_reg(
        handle: *mut c_void,
        space: u8,
        handler: Region,
        setup: RegionSetup,
        context: *mut c_void,
    ) -> Status;
    #[cfg(test)]
    fn stela_acpi_layout(field: u32) -> u64;
}
const _: () = {
    assert!(size_of::<Services>() == 296);
    assert!(core::mem::offset_of!(Services, fatal) == 280);
    assert!(core::mem::offset_of!(Services, destroy) == 272);
    assert!(core::mem::offset_of!(Services, preflight_wait_events) == 288);
    assert!(size_of::<RawObject>() == 24);
    assert!(size_of::<RawBuffer>() == 16);
    assert!(size_of::<PciId>() == 8);
};
static RESERVED: AtomicBool = AtomicBool::new(false);

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Mode {
    InterpreterOnly,
    Hardware,
}

#[derive(Clone, Copy, PartialEq, Eq)]
enum HardwareState {
    InterpreterOnly,
    ActivationFailed,
    Enabled,
}

/// The single ACPICA instance. This BSP-owned facade is neither Send nor Sync;
/// the provider synchronizes asynchronous ACPICA callbacks separately.
/// Handles borrow it, and cannot survive termination.
pub struct Session {
    active: bool,
    hardware: Cell<HardwareState>,
    ec_global_locks: Arc<AtomicUsize>,
    ec_resource_quarantine: Arc<AtomicBool>,
    notifications: RefCell<Vec<*mut NotifyContext>>,
    _local: PhantomData<*mut ()>,
}
impl Session {
    /// Initialize real firmware tables and namespace. InterpreterOnly skips ACPI
    /// mode/event/FACS/device `_INI` activation; explicit AML can still access I/O
    /// and therefore must only be evaluated with appropriate provider ownership.
    ///
    /// # Safety
    /// All [`Services`] contracts must hold, including physical firmware table
    /// validity and exclusive system ownership of this interpreter. Ownership of
    /// `context` transfers even on initialization failure (except ALREADY_EXISTS).
    pub unsafe fn start(services: Services, mode: Mode) -> Result<Self, Error> {
        if RESERVED
            .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
            .is_err()
        {
            return Err(ALREADY_EXISTS);
        }
        let valid_abi = services.abi_version == SERVICES_ABI_VERSION
            && services.abi_bytes == SERVICES_ABI_BYTES;
        if !valid_abi
            || services.root == 0
            || (mode == Mode::Hardware
                && services.capabilities & FULL_CAPABILITIES != FULL_CAPABILITIES)
        {
            // SAFETY: ownership transferred by caller; no C state was initialized.
            unsafe { (services.destroy)(services.context) };
            RESERVED.store(false, Ordering::Release);
            return Err(if !valid_abi || services.root == 0 {
                BAD_PARAMETER
            } else {
                SUPPORT
            });
        }
        // SAFETY: reserved singleton and provider contract promised by caller.
        let status = unsafe { stela_acpi_start(&services, u8::from(mode == Mode::Hardware)) };
        if status != 0 {
            // SAFETY: all callbacks remain valid, including on partial init.
            if unsafe { stela_acpi_stop() } == 0 {
                RESERVED.store(false, Ordering::Release);
            }
            return Err(Error(status));
        }
        Ok(Self {
            active: true,
            hardware: Cell::new(if mode == Mode::Hardware {
                HardwareState::Enabled
            } else {
                HardwareState::InterpreterOnly
            }),
            ec_global_locks: Arc::new(AtomicUsize::new(0)),
            ec_resource_quarantine: Arc::new(AtomicBool::new(false)),
            notifications: RefCell::new(Vec::new()),
            _local: PhantomData,
        })
    }
    /// After native EC/GPIO/serial handlers are installed, activate real SCI/GPE
    /// events and firmware `_INI`/`_REG` methods. Missing services remain errors.
    /// A failure may have partially enabled hardware; shut down the session before
    /// retrying. The provider's original capability attestation must still hold.
    pub fn enable_hardware(&self) -> Result<(), Error> {
        let guard = self.guard()?;
        match self.hardware.get() {
            HardwareState::Enabled => return guard.check(),
            HardwareState::ActivationFailed => return Err(SUPPORT),
            HardwareState::InterpreterOnly => {}
        }
        // C marks activation attempted before enabling hardware. That flag is
        // not a success witness: even a partial attempt requires
        // global shutdown before retry. Publish success only after both checks.
        self.hardware.set(HardwareState::ActivationFailed);
        // SAFETY: the live session owns ACPICA and the native provider.
        checked(unsafe { stela_acpi_enable_hardware() })?;
        guard.check()?;
        self.hardware.set(HardwareState::Enabled);
        Ok(())
    }
    /// True only after this provider's C activation succeeded, while its
    /// original namespace remains valid. No caller can manufacture this witness.
    pub fn hardware_enabled(&self) -> bool {
        self.active && self.hardware.get() == HardwareState::Enabled && self.namespace_available()
    }
    /// Real ACPICA/FACS arbitration for an EC whose observed `_GLK` requires it.
    /// The EC keeps this owner and its namespace lease through byte access.
    pub fn ec_global_lock(&self) -> EcGlobalLock<'_> {
        EcGlobalLock {
            session: self,
            held: None,
        }
    }

    pub fn root(&self) -> Handle<'_> {
        Handle {
            session: self,
            raw: usize::MAX as *mut c_void,
        }
    }
    pub fn handle(&self, path: &str) -> Result<Handle<'_>, Error> {
        self.root().child(path)
    }
    /// ACPICA's device walk matches `_HID` and `_CID` and filters absent devices
    /// using real `_STA`; a missing method follows ACPICA's specification default.
    pub fn devices(&self, hid: &str) -> Result<Vec<Handle<'_>>, Error> {
        let hid = CString::new(hid).map_err(|_| BAD_PARAMETER)?;
        self.walk(hid.as_ptr())
    }
    /// Every present device in the namespace: ACPICA's device walk without
    /// an ID, which filters by `_STA` alone.
    pub fn all_devices(&self) -> Result<Vec<Handle<'_>>, Error> {
        self.walk(ptr::null())
    }
    /// A copy of the firmware's first `signature` table, or `None` when it
    /// has none, read through AcpiGetTable and released with AcpiPutTable.
    pub fn table(&self, signature: &[u8; 4]) -> Result<Option<Vec<u8>>, Error> {
        unsafe extern "C" {
            fn acpi_get_table(signature: *const c_char, instance: u32, out: *mut *mut u8)
            -> Status;
            fn acpi_put_table(table: *mut u8);
        }
        let _guard = self.guard()?;
        let name = [signature[0], signature[1], signature[2], signature[3], 0];
        let mut table = ptr::null_mut();
        // SAFETY: a NUL-terminated signature and an out pointer ACPICA fills
        // while the namespace guard is held.
        let status = unsafe { acpi_get_table(name.as_ptr().cast(), 1, &mut table) };
        if status == NOT_FOUND.0 {
            return Ok(None);
        }
        checked(status)?;
        // SAFETY: ACPICA maps the whole table, whose header gives its length
        // at byte 4, until acpi_put_table releases it.
        let bytes = unsafe {
            let length = u32::from_le_bytes(ptr::read_unaligned(table.add(4).cast::<[u8; 4]>()));
            core::slice::from_raw_parts(table, length as usize).to_vec()
        };
        // SAFETY: the table acpi_get_table returned, released once.
        unsafe { acpi_put_table(table) };
        Ok(Some(bytes))
    }
    /// Every processor the namespace declares, with its ACPI ID, as Linux's
    /// `acpi_processor_get_info` reads it: a legacy `Processor` object's
    /// ProcessorID, present per `_STA` when it has one, or an `ACPI0007`
    /// device's `_UID`. The bool is true for a device declaration.
    pub fn processors(&self) -> Result<Vec<(Handle<'_>, u32, bool)>, Error> {
        let mut found = Vec::new();
        for handle in self.collect(|callback, context| {
            // SAFETY: the walk is synchronous and `context` outlives it.
            unsafe { stela_acpi_walk_processors(callback, context) }
        })? {
            if handle.child("_STA").is_ok()
                && !matches!(handle.evaluate(Some("_STA"), &[])?, Some(Value::Integer(status)) if status & 1 != 0)
            {
                continue;
            }
            let guard = self.guard()?;
            let mut id = 0;
            // SAFETY: the bound handle names a live Processor object.
            checked(unsafe { stela_acpi_processor_id(handle.raw, &mut id) })?;
            guard.check()?;
            found.push((handle, id, false));
        }
        for handle in self.devices("ACPI0007")? {
            if handle.child("_UID").is_err() {
                continue;
            }
            match handle.evaluate(Some("_UID"), &[])? {
                Some(Value::Integer(uid)) => {
                    if let Ok(uid) = u32::try_from(uid) {
                        found.push((handle, uid, true));
                    }
                }
                _ => continue,
            }
        }
        Ok(found)
    }
    fn walk(&self, hid: *const c_char) -> Result<Vec<Handle<'_>>, Error> {
        self.collect(|callback, context| {
            // SAFETY: the walk is synchronous and `context` outlives it.
            unsafe { stela_acpi_walk(hid, callback, context) }
        })
    }
    /// The handles a synchronous ACPICA walk reports to its callback.
    fn collect(
        &self,
        walk: impl FnOnce(
            unsafe extern "C" fn(*mut c_void, u32, *mut c_void, *mut *mut c_void) -> Status,
            *mut c_void,
        ) -> Status,
    ) -> Result<Vec<Handle<'_>>, Error> {
        let guard = self.guard()?;
        let mut raw = Vec::<*mut c_void>::new();
        unsafe extern "C" fn collect(
            handle: *mut c_void,
            _: u32,
            context: *mut c_void,
            _: *mut *mut c_void,
        ) -> Status {
            // SAFETY: the synchronous walk owns this Vec.
            unsafe { &mut *context.cast::<Vec<*mut c_void>>() }.push(handle);
            0
        }
        checked(walk(collect, (&mut raw as *mut Vec<*mut c_void>).cast()))?;
        guard.check()?;
        raw.into_iter()
            .map(|raw| {
                guard.bind(raw)?;
                Ok(Handle { session: self, raw })
            })
            .collect()
    }
    /// SLP_TYPa and SLP_TYPb of sleep `state`, as ACPICA evaluates `\_Sx`.
    pub fn sleep_types(&self, state: u8) -> Result<(u8, u8), Error> {
        let guard = self.guard()?;
        let (mut a, mut b) = (0, 0);
        // SAFETY: the live session owns ACPICA; both outputs are writable.
        checked(unsafe { acpi_get_sleep_type_data(state, &mut a, &mut b) })?;
        guard.check()?;
        Ok((a, b))
    }
    /// Prepare to enter sleep `state` as Linux's `acpi_sleep_prepare` does with
    /// `acpi_enter_sleep_state_prep`: `\_PTS` and the `\_SST` indicator. Entering
    /// the state, its PM1 or sleep-control writes, stays the caller's.
    pub fn prepare_sleep(&self, state: u8) -> Result<(), Error> {
        let guard = self.guard()?;
        // SAFETY: the live session owns ACPICA.
        checked(unsafe { acpi_enter_sleep_state_prep(state) })?;
        guard.check()
    }
    /// A namespace guard once hardware is activated: GPE blocks and fixed
    /// events exist only from then on.
    fn events(&self) -> Result<NamespaceGuard<'_>, Error> {
        if self.hardware.get() != HardwareState::Enabled {
            return Err(SUPPORT);
        }
        self.guard()
    }
    /// Linux `acpi_wakeup_gpe_init`: `device`'s `_PRW` GPE wakes the system
    /// and stays disabled while it runs (`acpi_setup_gpe_for_wake`); a
    /// button's is only marked for wake (`acpi_mark_gpe_for_wake`), as its
    /// driver may also take it at runtime ([`Self::enable_gpe`]). Either way
    /// [`Self::update_all_gpes`] leaves it alone.
    pub fn setup_wake_gpe(
        &self,
        device: Handle<'_>,
        gpe: Gpe<'_>,
        button: bool,
    ) -> Result<(), Error> {
        let guard = self.events()?;
        let block = gpe.block.map_or(ptr::null_mut(), |block| block.raw);
        // SAFETY: handles of this live session, under its namespace guard.
        checked(unsafe {
            if button {
                acpi_mark_gpe_for_wake(block, gpe.number)
            } else {
                acpi_setup_gpe_for_wake(device.raw, block, gpe.number)
            }
        })?;
        guard.check()
    }
    /// `acpi_enable_gpe`, for a GPE a driver takes while the system runs,
    /// such as a control-method button's.
    pub fn enable_gpe(&self, gpe: Gpe<'_>) -> Result<(), Error> {
        let guard = self.events()?;
        let block = gpe.block.map_or(ptr::null_mut(), |block| block.raw);
        // SAFETY: as in `setup_wake_gpe`.
        checked(unsafe { acpi_enable_gpe(block, gpe.number) })?;
        guard.check()
    }
    /// Linux `acpi_update_all_gpes` after its namespace scan: enable every
    /// GPE with an `_Lxx` or `_Exx` method that no `_PRW` claims for wake.
    /// ACPICA's SCI handler then runs the method as deferred work.
    pub fn update_all_gpes(&self) -> Result<(), Error> {
        let guard = self.events()?;
        // SAFETY: the live session owns ACPICA's GPE blocks.
        checked(unsafe { acpi_update_all_gpes() })?;
        guard.check()
    }
    /// As Linux's EC driver takes its GPE: a raw handler for GPE `number` of
    /// the FADT's blocks (`acpi_install_gpe_raw_handler`) that clears its
    /// status and sets `flag` for the driver that services the event. The raw
    /// handler disables this GPE before publishing the flag; after draining
    /// the device, the owner must call [`Self::enable_gpe`] once to resume it.
    /// Other SCI contributors remain enabled while deferred device work runs.
    pub fn flag_gpe(
        &self,
        number: u32,
        edge: bool,
        flag: &'static AtomicBool,
    ) -> Result<(), Error> {
        let guard = self.events()?;
        let kind = if edge { GPE_EDGE } else { GPE_LEVEL };
        let context = ptr::from_ref(flag).cast_mut().cast();
        // SAFETY: `flag` lives for good; the handler only clears and flags.
        checked(unsafe {
            acpi_install_gpe_raw_handler(ptr::null_mut(), number, kind, flag_gpe, context)
        })?;
        // SAFETY: the GPE whose handler was just installed.
        checked(unsafe { acpi_enable_gpe(ptr::null_mut(), number) })?;
        guard.check()
    }
    /// `acpi_install_fixed_event_handler`, as Linux's button driver takes
    /// the fixed power button: ACPICA enables fixed `event`, and its SCI
    /// handler clears the event's status and sets `flag`.
    pub fn flag_fixed_event(&self, event: u32, flag: &'static AtomicBool) -> Result<(), Error> {
        let guard = self.events()?;
        let context = ptr::from_ref(flag).cast_mut().cast();
        // SAFETY: `flag` lives for good; the handler only flags.
        checked(unsafe { acpi_install_fixed_event_handler(event, flag_event, context) })?;
        guard.check()
    }
    /// Every notification the firmware sends, a system one or a device's,
    /// with the path of the object it names, as Linux's `acpi_bus_notify`
    /// and the drivers see them together; for reporting those no driver
    /// takes. The callback runs as deferred work.
    pub fn notify_all(
        &self,
        callback: impl Fn(Option<String>, u32) + Send + Sync + 'static,
    ) -> Result<Notification<'_>, Error> {
        self.root()
            .install_notify(ALL_NOTIFY, NotifyCallback::Device(Box::new(callback)))
    }
    /// False after native table deletion. The whole facade then returns AE_SUPPORT;
    /// native driver rebinding into a changed namespace is not implemented.
    pub fn namespace_available(&self) -> bool {
        // SAFETY: this live session owns the global native interpreter.
        unsafe { stela_acpi_namespace_valid() != 0 }
    }
    fn guard(&self) -> Result<NamespaceGuard<'_>, Error> {
        let mut raw = ptr::null_mut();
        let mut epoch = 0;
        // SAFETY: borrowed session keeps C/provider state through guard drop.
        checked(unsafe { stela_acpi_guard_enter(&mut raw, &mut epoch) })?;
        Ok(NamespaceGuard {
            raw,
            epoch,
            session: self,
        })
    }
    fn release_notification(&self, context: *mut NotifyContext) {
        let mut contexts = self.notifications.borrow_mut();
        let index = contexts
            .iter()
            .position(|p| *p == context)
            .expect("owned notification");
        contexts.swap_remove(index);
        // SAFETY: handler removal and work draining completed before this call.
        unsafe {
            drop(Box::from_raw(context));
        }
    }
    /// Quiesce callbacks before freeing ACPICA state. A failure quarantines the
    /// interpreter/provider permanently and prevents another session being opened.
    pub fn shutdown(mut self) -> Result<(), Error> {
        self.close()
    }
    fn close(&mut self) -> Result<(), Error> {
        if !self.active {
            return Ok(());
        }
        if self.ec_global_locks.load(Ordering::Acquire) != 0
            || self.ec_resource_quarantine.load(Ordering::Acquire)
        {
            // An unreleased FACS mutex or an unregistered native resource with
            // failed retirement still references this provider. Keep both.
            return Err(SUPPORT);
        }
        self.active = false;
        self.hardware.set(HardwareState::ActivationFailed);
        // SAFETY: this session owns the globally reserved instance.
        let result = checked(unsafe { stela_acpi_stop() });
        if result.is_ok() {
            // The core/provider are stopped; includes callbacks kept after
            // unload or failed Notify removal. Failed quiescence leaks them safely.
            for context in self.notifications.get_mut().drain(..) {
                // SAFETY: the core is stopped, so no callback can still use the context.
                unsafe {
                    drop(Box::from_raw(context));
                }
            }
            RESERVED.store(false, Ordering::Release);
        }
        result
    }
}
impl Drop for Session {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

struct NamespaceGuard<'a> {
    raw: *mut c_void,
    epoch: u64,
    session: &'a Session,
}
impl NamespaceGuard<'_> {
    fn check(&self) -> Result<(), Error> {
        if self.session.namespace_available() {
            Ok(())
        } else {
            Err(SUPPORT)
        }
    }
    fn bind(&self, candidate: *mut c_void) -> Result<(), Error> {
        self.check()?;
        // SAFETY: namespace read lease protects the locked tree walk;
        // candidate itself is compared only and never dereferenced speculatively.
        checked(unsafe { stela_acpi_bind_node(candidate, self.epoch) })?;
        self.check()
    }
}
impl Drop for NamespaceGuard<'_> {
    fn drop(&mut self) {
        // SAFETY: this guard entered the namespace lease that it now leaves.
        unsafe {
            stela_acpi_guard_leave(self.raw);
        }
    }
}

/// An EC firmware lock, backed by ACPICA's global-lock API.
/// Failed release keeps its token/namespace lease; it never retries implicitly.
pub struct EcGlobalLock<'a> {
    session: &'a Session,
    held: Option<(u32, NamespaceGuard<'a>)>,
}

impl super::ec::FirmwareLock for EcGlobalLock<'_> {
    fn acquire(&mut self, timeout_us: u64) -> Result<(), super::ec::LockError> {
        if self.held.is_some() {
            return Err(super::ec::LockError);
        }
        // ACPICA uses milliseconds; 0xffff means forever and is not bounded.
        let milliseconds = timeout_us.div_ceil(1_000);
        if milliseconds == 0 || milliseconds >= u64::from(u16::MAX) {
            return Err(super::ec::LockError);
        }
        let guard = self.session.guard().map_err(|_| super::ec::LockError)?;
        let mut token = 0;
        // SAFETY: the session and namespace read lease keep ACPICA;
        // only the real API creates this FACS/global-mutex ownership token.
        checked(unsafe { acpi_acquire_global_lock(milliseconds as u16, &mut token) })
            .map_err(|_| super::ec::LockError)?;
        self.held = Some((token, guard));
        // Native service callbacks can nest; count the current ownership after
        // the real acquisition instead of overwriting a pre-call snapshot.
        self.session
            .ec_global_locks
            .try_update(Ordering::AcqRel, Ordering::Acquire, |count| {
                count.checked_add(1)
            })
            .map_err(|_| super::ec::LockError)?;
        if self.held.as_ref().unwrap().1.check().is_err() {
            // A native callback may have invalidated namespace admission while
            // acquisition waited. Release firmware ownership, never
            // authorize EC I/O from the stale namespace observation.
            self.release()?;
            return Err(super::ec::LockError);
        }
        Ok(())
    }

    fn release(&mut self) -> Result<(), super::ec::LockError> {
        let Some((token, _guard)) = self.held.as_ref() else {
            return Err(super::ec::LockError);
        };
        // SAFETY: token belongs to this live session and is still acquired.
        // Cleanup remains necessary even if namespace admission was invalidated.
        checked(unsafe { acpi_release_global_lock(*token) }).map_err(|_| super::ec::LockError)?;
        self.held = None;
        self.session.ec_global_locks.fetch_sub(1, Ordering::AcqRel);
        Ok(())
    }
}

impl Drop for EcGlobalLock<'_> {
    fn drop(&mut self) {
        if let Some((_token, guard)) = self.held.take() {
            // An uncertain/unreleased firmware lock cannot silently free its
            // namespace owner. Ec reports and quarantines release failures.
            core::mem::forget(guard);
        }
    }
}

/// A general-purpose event: its number in the FADT's GPE blocks, or in a
/// GPE block device's.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Gpe<'a> {
    pub block: Option<Handle<'a>>,
    pub number: u32,
}

#[derive(Clone, Copy)]
pub struct Handle<'a> {
    session: &'a Session,
    raw: *mut c_void,
}
impl core::fmt::Debug for Handle<'_> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_tuple("AcpiHandle").field(&self.raw).finish()
    }
}
impl PartialEq for Handle<'_> {
    fn eq(&self, other: &Self) -> bool {
        core::ptr::eq(self.session, other.session) && self.raw == other.raw
    }
}
impl Eq for Handle<'_> {}
/// Exact ACPICA external object type code, including values unknown here.
#[repr(transparent)]
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObjectType(u32);
impl ObjectType {
    pub const INTEGER: Self = Self(1);
    pub const DEVICE: Self = Self(6);
    pub const POWER: Self = Self(11);

    pub const fn from_raw(raw: u32) -> Self {
        Self(raw)
    }
    pub const fn raw(self) -> u32 {
        self.0
    }
}
/// Namespace reference and its external ACPICA ActualType. The handle still
/// borrows its single Session; this metadata grants no independent ownership.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Reference<'a> {
    pub handle: Handle<'a>,
    pub actual_type: ObjectType,
}
/// Why an `_OSC` handshake failed.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Osc {
    /// The object has no `_OSC`.
    Absent,
    /// Its result is not a buffer the length of the capabilities.
    InvalidReturn,
    /// The firmware refused the request: its status DWORD.
    Refused(u32),
}

/// `acpi_osc_error_check`: a query fails on any defined error bit but a
/// masked capability, which only says some were not granted; a request
/// fails only on an unrecognized UUID, since the firmware acted on the rest.
fn osc_result(status: u32, returned: Vec<u32>) -> Result<Vec<u32>, Osc> {
    let mut errors = returned[0]
        & (OSC_REQUEST_ERROR
            | OSC_INVALID_UUID_ERROR
            | OSC_INVALID_REVISION_ERROR
            | OSC_CAPABILITIES_MASK_ERROR);
    let query = status & OSC_QUERY_ENABLE != 0;
    if query {
        errors &= !OSC_CAPABILITIES_MASK_ERROR;
    }
    if errors != 0 && (query || errors & OSC_INVALID_UUID_ERROR != 0) {
        return Err(Osc::Refused(returned[0]));
    }
    Ok(returned)
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Value<'a> {
    Integer(u64),
    String(Vec<u8>),
    Buffer(Vec<u8>),
    Package(Vec<Value<'a>>),
    Reference(Reference<'a>),
    Uninitialized,
}
pub enum Argument<'a> {
    Integer(u64),
    String(&'a CStr),
    Buffer(&'a [u8]),
    Package(&'a [Argument<'a>]),
    Reference(Reference<'a>),
}
struct Arguments {
    objects: Vec<RawObject>,
    children: Vec<Arguments>,
}
impl Arguments {
    fn new(args: &[Argument<'_>], session: &Session, depth: u32) -> Result<Self, Error> {
        if depth > 32 || args.len() > 65536 {
            return Err(LIMIT);
        }
        let mut result = Self {
            objects: Vec::with_capacity(args.len()),
            children: Vec::new(),
        };
        for arg in args {
            let (kind, length, payload) = match arg {
                Argument::Integer(n) => (1, 0, *n),
                Argument::String(text) => (
                    2,
                    u32::try_from(text.to_bytes().len()).map_err(|_| LIMIT)?,
                    text.as_ptr() as u64,
                ),
                Argument::Buffer(bytes) => (
                    3,
                    u32::try_from(bytes.len()).map_err(|_| LIMIT)?,
                    bytes.as_ptr() as u64,
                ),
                Argument::Package(items) => {
                    let child = Self::new(items, session, depth + 1)?;
                    let entry = (4, child.objects.len() as u32, child.objects.as_ptr() as u64);
                    result.children.push(child);
                    entry
                }
                Argument::Reference(reference) => {
                    if !ptr::eq(session, reference.handle.session) {
                        return Err(BAD_PARAMETER);
                    }
                    (
                        0x14,
                        reference.actual_type.raw(),
                        reference.handle.raw as u64,
                    )
                }
            };
            result.objects.push(RawObject {
                kind,
                length,
                payload,
                extra: 0,
            });
        }
        Ok(result)
    }
}
impl<'a> Handle<'a> {
    pub(crate) fn ec_scope(self, root: bool) -> Self {
        if root { self.session.root() } else { self }
    }
    pub(crate) fn ec_lock_count(self) -> Arc<AtomicUsize> {
        self.session.ec_global_locks.clone()
    }
    pub(crate) fn ec_resource_quarantine(self) -> Arc<AtomicBool> {
        self.session.ec_resource_quarantine.clone()
    }
    /// Keep callback storage through core termination, including failed
    /// removal. The finalizer runs before the original OSL provider is destroyed.
    ///
    /// # Safety
    /// A successful call transfers ownership of context to the core finalizer.
    /// before_termination runs after real provider quiescence, closes native
    /// admission and must refuse while ownership still requires the live core.
    /// The finalizer must not enter AML or unwind, and must retire provider
    /// resources before freeing its storage. Failed calls transfer no ownership.
    /// An optional activation callback runs after FACS/SCI/GLK setup
    /// and before device _INI. It must be idempotent after explicit attachment,
    /// keep its namespace identity and propagate any activation failure.
    /// Activation must not register more kept contexts or borrow a movable
    /// Rust Session address.
    pub(crate) unsafe fn retain_ec_context(
        self,
        before_termination: unsafe extern "C" fn(*mut c_void) -> Status,
        retire: unsafe extern "C" fn(*mut c_void) -> Status,
        activate: Option<unsafe extern "C" fn(*mut c_void) -> Status>,
        context: *mut c_void,
    ) -> Result<(), Error> {
        let guard = self.session.guard()?;
        guard.bind(self.raw)?;
        // SAFETY: caller transfers the context on C success.
        checked(unsafe { stela_acpi_retain_context(before_termination, retire, activate, context) })
    }
    pub(crate) fn ec_activation_target(self) -> Result<*mut c_void, Error> {
        let guard = self.session.guard()?;
        guard.bind(self.raw)?;
        Ok(self.raw)
    }
    /// Match Linux EC setup: root handler coverage must not connect unrelated
    /// devices' _REG methods. The EC subtree is connected separately.
    pub(crate) unsafe fn install_ec_region(
        self,
        handler: Region,
        setup: RegionSetup,
        context: *mut c_void,
    ) -> Result<(), Error> {
        let guard = self.session.guard()?;
        guard.bind(self.raw)?;
        // SAFETY: live namespace lease validates the exact scope; caller keeps
        // the callback/context until removal or core termination.
        checked(unsafe {
            acpi_install_address_space_handler_no_reg(self.raw, 3, handler, setup, context)
        })?;
        guard.check()
    }
    /// Read the owning provider's successful activation and namespace witness.
    pub fn hardware_active(self) -> bool {
        self.session.hardware_enabled()
    }
    /// Resolve a namespace handle to its absolute ACPI path.
    pub fn path(self) -> Result<String, Error> {
        let guard = self.session.guard()?;
        // SAFETY: handle belongs to this live session, under its guard.
        let path = unsafe { name(self.raw) }?;
        guard.check()?;
        Ok(path)
    }
    /// The GPE this device's `_PRW` names, as Linux's
    /// `acpi_bus_extract_wakeup_device_power_package` reads it: an integer,
    /// a GPE of the FADT's blocks, or a GPE block device's reference and
    /// index. `None` without `_PRW`.
    pub fn wake_gpe(self) -> Result<Option<Gpe<'a>>, Error> {
        let prw = match self.child("_PRW") {
            Ok(prw) => prw,
            Err(error) if error == NOT_FOUND => return Ok(None),
            Err(error) => return Err(error),
        };
        let Some(Value::Package(elements)) = prw.evaluate(None, &[])? else {
            return Err(BAD_PARAMETER);
        };
        if elements.len() < 2 || !matches!(elements[1], Value::Integer(0..=5)) {
            return Err(BAD_PARAMETER);
        }
        let number = |value: &Value<'a>| match value {
            Value::Integer(number) => u32::try_from(*number).map_err(|_| BAD_PARAMETER),
            _ => Err(BAD_PARAMETER),
        };
        let gpe = match elements.first() {
            Some(Value::Package(pair)) => match pair.as_slice() {
                [Value::Reference(block), index] => Gpe {
                    block: Some(block.handle),
                    number: number(index)?,
                },
                _ => return Err(BAD_PARAMETER),
            },
            Some(value) => Gpe {
                block: None,
                number: number(value)?,
            },
            None => return Err(BAD_PARAMETER),
        };
        Ok(Some(gpe))
    }

    pub fn child(self, path: &str) -> Result<Self, Error> {
        let guard = self.session.guard()?;
        let path = CString::new(path).map_err(|_| BAD_PARAMETER)?;
        let mut raw = ptr::null_mut();
        // SAFETY: handles belong to the live borrowed instance, output is writable.
        checked(unsafe { acpi_get_handle(self.raw, path.as_ptr(), &mut raw) })?;
        guard.bind(raw)?;
        Ok(Self {
            session: self.session,
            raw,
        })
    }
    /// Execute AML while discarding its result in ACPICA itself. This preserves
    /// NULL-output evaluation semantics even when the returned object has no
    /// supported external representation. Errors and namespace invalidation
    /// still propagate; the method and all side effects execute once.
    pub fn invoke(self, path: Option<&str>, args: &[Argument<'_>]) -> Result<(), Error> {
        let guard = self.session.guard()?;
        guard.bind(self.raw)?;
        let path = path
            .map(CString::new)
            .transpose()
            .map_err(|_| BAD_PARAMETER)?;
        let mut args = Arguments::new(args, self.session, 0)?;
        // SAFETY: the guard binds this live namespace target. Every argument
        // buffer remains owned/borrowed here for the call. A NULL result is the
        // ACPICA API's discard request, not an invented return object.
        checked(unsafe {
            stela_acpi_evaluate(
                self.raw,
                path.as_ref().map_or(ptr::null(), |p| p.as_ptr()),
                args.objects.as_mut_ptr(),
                args.objects.len() as u32,
                ptr::null_mut(),
            )
        })?;
        guard.check()
    }
    pub fn evaluate(
        self,
        path: Option<&str>,
        args: &[Argument<'_>],
    ) -> Result<Option<Value<'a>>, Error> {
        let guard = self.session.guard()?;
        let path = path
            .map(CString::new)
            .transpose()
            .map_err(|_| BAD_PARAMETER)?;
        let mut args = Arguments::new(args, self.session, 0)?;
        let mut result = RawBuffer {
            length: u64::MAX,
            pointer: ptr::null_mut(),
        };
        // SAFETY: arguments and all nested buffers are held during evaluation.
        let status = unsafe {
            stela_acpi_evaluate(
                self.raw,
                path.as_ref().map_or(ptr::null(), |p| p.as_ptr()),
                args.objects.as_mut_ptr(),
                args.objects.len() as u32,
                &mut result,
            )
        };
        struct Free(*mut c_void);
        impl Drop for Free {
            fn drop(&mut self) {
                if !self.0.is_null() {
                    // SAFETY: ACPICA allocated this result buffer for the caller to free once.
                    unsafe { acpi_os_free(self.0) };
                }
            }
        }
        let _free = Free(result.pointer);
        checked(status)?;
        guard.check()?;
        if result.pointer.is_null() {
            return Ok(None);
        }
        let start = result.pointer as usize;
        let end = start
            .checked_add(usize::try_from(result.length).map_err(|_| LIMIT)?)
            .ok_or(LIMIT)?;
        // SAFETY: successful ACPICA result is an owned external-object allocation.
        unsafe {
            read_value(
                result.pointer.cast(),
                start,
                end,
                self.session,
                &guard,
                &mut 65536,
                0,
            )
            .map(Some)
        }
    }
    /// The `_DSD` device property `name`, as Linux's `acpi_dev_get_property`
    /// finds it: in the first well-formed device-properties set that names
    /// it. None when the device has no `_DSD` or no such property; a set
    /// whose entries are not (string, value) pairs is skipped and a malformed
    /// UUID/set pair ends the search, as `acpi_extract_properties` does.
    pub fn property(self, name: &str) -> Result<Option<Value<'a>>, Error> {
        if self.child("_DSD").is_err() {
            return Ok(None);
        }
        let Some(Value::Package(dsd)) = self.evaluate(Some("_DSD"), &[])? else {
            return Ok(None);
        };
        let mut entries = dsd.into_iter();
        while let (Some(uuid), Some(set)) = (entries.next(), entries.next()) {
            let (Value::Buffer(uuid), Value::Package(set)) = (uuid, set) else {
                break;
            };
            if uuid != DEVICE_PROPERTIES
                || !set.iter().all(|property| {
                    matches!(property, Value::Package(pair)
                        if pair.len() == 2 && matches!(pair[0], Value::String(_)))
                })
            {
                continue;
            }
            for property in set {
                let Value::Package(mut pair) = property else {
                    continue;
                };
                let value = pair.pop();
                if matches!(&pair[0], Value::String(key) if key == name.as_bytes()) {
                    return Ok(value);
                }
            }
        }
        Ok(None)
    }
    /// Evaluate the specification UCSI `_DSM` with an empty package Arg3.
    pub fn dsm(
        self,
        uuid: &[u8; 16],
        revision: u64,
        function: u64,
    ) -> Result<Option<Value<'a>>, Error> {
        self.evaluate(
            Some("_DSM"),
            &[
                Argument::Buffer(uuid),
                Argument::Integer(revision),
                Argument::Integer(function),
                Argument::Package(&[]),
            ],
        )
    }
    /// Linux's `acpi_run_osc`: `_OSC` with `capabilities` as its DWORDs, the
    /// first being the status/query DWORD, and the DWORDs the firmware
    /// returns.
    pub fn osc(
        self,
        uuid: &[u8; 16],
        revision: u64,
        capabilities: &[u32],
    ) -> Result<Result<Vec<u32>, Osc>, Error> {
        if capabilities.len() < 2 {
            return Err(BAD_PARAMETER);
        }
        if self.child("_OSC").is_err() {
            return Ok(Err(Osc::Absent));
        }
        let bytes: Vec<u8> = capabilities
            .iter()
            .flat_map(|dword| dword.to_le_bytes())
            .collect();
        let returned = self.evaluate(
            Some("_OSC"),
            &[
                Argument::Buffer(uuid),
                Argument::Integer(revision),
                Argument::Integer(capabilities.len() as u64),
                Argument::Buffer(&bytes),
            ],
        )?;
        Ok(match returned {
            Some(Value::Buffer(returned)) if returned.len() == bytes.len() => osc_result(
                capabilities[0],
                returned
                    .as_chunks::<4>()
                    .0
                    .iter()
                    .map(|dword| u32::from_le_bytes(*dword))
                    .collect(),
            ),
            _ => Err(Osc::InvalidReturn),
        })
    }
    /// Install a true ACPICA device Notify callback. Callback work must be short,
    /// nonblocking and enqueue driver work; the provider supplies deferred dispatch.
    pub fn notify(
        self,
        callback: impl Fn(u32) + Send + Sync + 'static,
    ) -> Result<Notification<'a>, Error> {
        self.install_notify(DEVICE_NOTIFY, NotifyCallback::Value(Box::new(callback)))
    }
    fn install_notify(
        self,
        kind: u32,
        callback: NotifyCallback,
    ) -> Result<Notification<'a>, Error> {
        let guard = self.session.guard()?;
        let context = Box::new(NotifyContext {
            enabled: AtomicBool::new(true),
            kind,
            callback,
        });
        let context = Box::into_raw(context);
        // SAFETY: owned stable context is held until removal and drain.
        let status = unsafe {
            acpi_install_notify_handler(self.raw, kind, notify_trampoline, context.cast())
        };
        if status != 0 {
            // SAFETY: installation failed, so this is the only reference to the box.
            unsafe {
                drop(Box::from_raw(context));
            }
            return Err(Error(status));
        }
        self.session.notifications.borrow_mut().push(context);
        if let Err(error) = guard.check() {
            // SAFETY: the context was just registered with this session, which keeps it.
            unsafe { &*context }.enabled.store(false, Ordering::Release);
            return Err(error);
        }
        Ok(Notification {
            handle: self,
            context: Some(context),
        })
    }
    /// Install an owned native region implementation (e.g. EmbeddedControl 3).
    ///
    /// # Safety
    /// Context/callbacks must remain valid until removal and event drain; implement
    /// ACPICA region setup/activation/deactivation, ownership and I/O. This
    /// never silently substitutes zeros for an unsupported EC/GPIO/serial region.
    pub unsafe fn install_region(
        self,
        space: u32,
        handler: Region,
        setup: RegionSetup,
        context: *mut c_void,
    ) -> Result<(), Error> {
        let guard = self.session.guard()?;
        // SAFETY: covered by this function's own safety contract.
        checked(unsafe {
            acpi_install_address_space_handler(self.raw, space, handler, setup, context)
        })?;
        guard.check()
    }
    /// # Safety
    /// Must match an installed handler and keep its context through this call;
    /// provider must synchronize external region callbacks before freeing context.
    pub unsafe fn remove_region(self, space: u32, handler: Region) -> Result<(), Error> {
        let guard = self.session.guard()?;
        // SAFETY: this live namespace lease precedes removal. The provider
        // checks the real current role without mutating C registration state.
        checked(unsafe { stela_acpi_preflight_wait_events() })?;
        // SAFETY: covered by this function's own safety contract.
        checked(unsafe { acpi_remove_address_space_handler(self.raw, space, handler) })?;
        // SAFETY: called from process context with no namespace lock held by this thread.
        unsafe { acpi_os_wait_events_complete() };
        guard.check()
    }
}
/// The absolute path of a node of the live namespace.
///
/// # Safety
/// `raw` is a node of the live namespace, which the caller keeps live.
unsafe fn name(raw: *mut c_void) -> Result<String, Error> {
    let mut result = RawBuffer {
        length: u64::MAX,
        pointer: ptr::null_mut(),
    };
    // SAFETY: the caller's live node; ACPICA owns the result.
    let status = unsafe { acpi_get_name(raw, 0, &mut result) };
    struct Free(*mut c_void);
    impl Drop for Free {
        fn drop(&mut self) {
            if !self.0.is_null() {
                // SAFETY: ACPICA allocated this result buffer for the caller to free once.
                unsafe { acpi_os_free(self.0) };
            }
        }
    }
    let _free = Free(result.pointer);
    checked(status)?;
    if result.pointer.is_null() {
        return Err(BAD_PARAMETER);
    }
    let length = usize::try_from(result.length).map_err(|_| LIMIT)?;
    if length > 65536 {
        return Err(LIMIT);
    }
    // SAFETY: successful ACPICA output owns exactly buffer.length bytes.
    let bytes = unsafe { core::slice::from_raw_parts(result.pointer.cast::<u8>(), length) };
    let text = CStr::from_bytes_until_nul(bytes).map_err(|_| BAD_PARAMETER)?;
    String::from_utf8(text.to_bytes().to_vec()).map_err(|_| BAD_PARAMETER)
}
struct NotifyContext {
    enabled: AtomicBool,
    /// `ACPI_DEVICE_NOTIFY` or `ACPI_ALL_NOTIFY`.
    kind: u32,
    callback: NotifyCallback,
}
enum NotifyCallback {
    Value(Box<dyn Fn(u32) + Send + Sync>),
    /// With the notifying object's path, for a handler of every object.
    Device(Box<dyn Fn(Option<String>, u32) + Send + Sync>),
}
/// A fixed event's handler: the flag set.
unsafe extern "C" fn flag_event(context: *mut c_void) -> u32 {
    // SAFETY: `flag_fixed_event` passed a flag that lives for good.
    unsafe { &*context.cast::<AtomicBool>() }.store(true, Ordering::Release);
    INTERRUPT_HANDLED
}
/// A raw GPE handler keeps a disabled device source until its owner drains
/// deferred transactions and explicitly resumes it. This permits the parent
/// SCI to service FACS/global-lock events while a device worker waits.
unsafe extern "C" fn flag_gpe(block: *mut c_void, number: u32, context: *mut c_void) -> u32 {
    // SAFETY: ACPICA dispatches its own GPE, its GPE lock released; this raw
    // registration owns its sole runtime enable reference until resumption.
    let disabled = unsafe { acpi_disable_gpe(block, number) };
    // SAFETY: the same live GPE registration owns its status register; clear
    // only after its runtime enable reference was successfully removed.
    if disabled != 0 || unsafe { acpi_clear_gpe(block, number) } != 0 {
        return 0;
    }
    // SAFETY: `flag_gpe` passed a flag that lives for good.
    unsafe { &*context.cast::<AtomicBool>() }.store(true, Ordering::Release);
    INTERRUPT_HANDLED
}
unsafe extern "C" fn notify_trampoline(object: *mut c_void, value: u32, context: *mut c_void) {
    // SAFETY: registration owns context through handler removal + draining.
    let context = unsafe { &*context.cast::<NotifyContext>() };
    // UNLOAD invalidates clients before deletion; context storage is still owned
    // through quiescence/core termination even when the node itself is gone.
    // SAFETY: ACPICA calls this trampoline only inside the live core.
    if context.enabled.load(Ordering::Acquire) && unsafe { stela_acpi_namespace_valid() != 0 } {
        match &context.callback {
            NotifyCallback::Value(callback) => callback(value),
            // SAFETY: the object ACPICA dispatches for, in its live namespace.
            NotifyCallback::Device(callback) => callback(unsafe { name(object) }.ok(), value),
        }
    }
}
pub struct Notification<'a> {
    handle: Handle<'a>,
    context: Option<*mut NotifyContext>,
}
impl Notification<'_> {
    pub fn remove(mut self) -> Result<(), Error> {
        self.close()
    }
    // Test an ACPICA removal error after another native actor removed
    // the registration. The Rust owner remains outstanding.
    #[cfg(any(test, feature = "test-support"))]
    pub fn remove_from_core_without_retiring_for_test(&self) -> Result<(), Error> {
        let guard = self.handle.session.guard()?;
        // SAFETY: the registration's context lives until global teardown.
        let kind = self
            .context
            .map_or(DEVICE_NOTIFY, |context| unsafe { &*context }.kind);
        // SAFETY: immediate provider role check precedes the test's real C removal.
        checked(unsafe { stela_acpi_preflight_wait_events() })?;
        // SAFETY: the live node and callback belong to this registration.
        checked(unsafe { acpi_remove_notify_handler(self.handle.raw, kind, notify_trampoline) })?;
        // SAFETY: test process context; drain the real provider work queue.
        unsafe { acpi_os_wait_events_complete() };
        guard.check()
    }
    fn close(&mut self) -> Result<(), Error> {
        let Some(context) = self.context.take() else {
            return Ok(());
        };
        // SAFETY: the session keeps every registered context until global teardown.
        let kind = unsafe { &*context }.kind;
        let result = (|| {
            let guard = self.handle.session.guard()?;
            // SAFETY: role eligibility is checked before ACPICA removes
            // this registration. Refusal keeps its callback in Session.
            checked(unsafe { stela_acpi_preflight_wait_events() })?;
            // Only a live namespace lease may pass this node into ACPICA.
            // SAFETY: the namespace guard keeps this node live.
            checked(unsafe {
                acpi_remove_notify_handler(self.handle.raw, kind, notify_trampoline)
            })?;
            // SAFETY: called from process context with no namespace lock held by this thread.
            unsafe {
                acpi_os_wait_events_complete();
            }
            guard.check()
        })();
        if result.is_ok() {
            self.handle.session.release_notification(context);
        } else {
            // Keep callback storage in Session until successful global teardown;
            // neither stale removal nor an error may free a still registered box.
            // SAFETY: the session keeps every registered context until global teardown.
            unsafe { &*context }.enabled.store(false, Ordering::Release);
        }
        result
    }
}
impl Drop for Notification<'_> {
    fn drop(&mut self) {
        let _ = self.close();
    }
}

unsafe fn read_value<'a>(
    object: *const RawObject,
    start: usize,
    end: usize,
    session: &'a Session,
    guard: &NamespaceGuard<'_>,
    budget: &mut usize,
    depth: u32,
) -> Result<Value<'a>, Error> {
    if depth > 32 || *budget == 0 {
        return Err(LIMIT);
    }
    *budget -= 1;
    fn range(p: usize, bytes: usize, start: usize, end: usize) -> Result<(), Error> {
        if p < start || p.checked_add(bytes).is_none_or(|n| n > end) {
            Err(BAD_PARAMETER)
        } else {
            Ok(())
        }
    }
    range(object as usize, size_of::<RawObject>(), start, end)?;
    // ACPICA packs the external buffer; unaligned reads avoid platform assumptions.
    // SAFETY: `range` checked the object lies in the result buffer.
    let raw = unsafe { object.read_unaligned() };
    match raw.kind {
        0 => Ok(Value::Uninitialized),
        1 => Ok(Value::Integer(raw.payload)),
        2 | 3 => {
            let n = raw.length as usize;
            let bytes = if n == 0 {
                Vec::new()
            } else {
                range(raw.payload as usize, n, start, end)?;
                // SAFETY: `range` checked these bytes lie in the result buffer.
                unsafe { core::slice::from_raw_parts(raw.payload as *const u8, n) }.to_vec()
            };
            Ok(if raw.kind == 2 {
                Value::String(bytes)
            } else {
                Value::Buffer(bytes)
            })
        }
        4 => {
            let count = raw.length as usize;
            if count > *budget {
                return Err(LIMIT);
            }
            if count != 0 {
                range(
                    raw.payload as usize,
                    count.checked_mul(size_of::<RawObject>()).ok_or(LIMIT)?,
                    start,
                    end,
                )?;
            }
            let mut values = Vec::with_capacity(count);
            for n in 0..count {
                // SAFETY: `range` checked the whole element array lies in the result buffer.
                values.push(unsafe {
                    read_value(
                        (raw.payload as *const RawObject).add(n),
                        start,
                        end,
                        session,
                        guard,
                        budget,
                        depth + 1,
                    )
                }?);
            }
            Ok(Value::Package(values))
        }
        0x14 => {
            guard.bind(raw.payload as *mut c_void)?;
            Ok(Value::Reference(Reference {
                handle: Handle {
                    session,
                    raw: raw.payload as *mut c_void,
                },
                actual_type: ObjectType::from_raw(raw.length),
            }))
        }
        _ => Err(SUPPORT),
    }
}

#[cfg(any(test, feature = "test-support"))]
pub mod test_support;
#[cfg(test)]
mod tests;
