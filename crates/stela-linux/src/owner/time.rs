// SPDX-License-Identifier: GPL-2.0-or-later
//! Actual source-owned physical deadlines kept independently of C park loans.
use super::*;
use core::mem::ManuallyDrop;
/// # Safety
/// The object owns a genuine minimum from the actual installed source and
/// same scheduler clock/CPU. Methods revalidate source epoch/current role.
/// Neither nominal time nor a copied Hz/sample grants readiness. It retains
/// original source/provider/code roots until explicit valid-context release.
/// All methods are nonblocking and hold no native/process loan after return.
/// Test-only implementations own isolated counter state, never native grants.
pub unsafe trait MinimumDeadline {
    fn ready(&self) -> Result<bool, WorkError>;
    fn wake_ns(&self) -> Result<i64, WorkError>;
    /// Positive conservative ceil such that a new minimum of n*quantum_us on
    /// this same source cannot precede the original remaining counter target.
    fn remaining_quanta_ceil(&self, quantum_us: u64) -> Result<u64, WorkError>;
}
#[repr(C)]
pub(super) struct Header {
    ready: unsafe extern "C" fn(*mut Header) -> bool,
    wake_ns: unsafe extern "C" fn(*mut Header) -> i64,
    remaining: unsafe extern "C" fn(*mut Header, u64) -> u64,
    release: unsafe extern "C" fn(*mut Header),
}
#[repr(C)]
struct Owned {
    header: Header,
    source: ManuallyDrop<Box<dyn MinimumDeadline>>,
    owner: *const RawOwner,
    task: *mut c_void,
}
const _: () = assert!(size_of::<Header>() == 32);
unsafe extern "C" {
    fn stela_linux_owner() -> *const RawOwner;
    fn stela_linux_minimum_task(owner: *const RawOwner) -> *mut c_void;
}
// WorkError currently has exactly thirteen payload-free variants. This
// bijection records the complete actual cause without a formatter, allocator,
// process lock, C call or arbitrary Platform.log callback in a failure path.
struct FirstMinimumFault(core::sync::atomic::AtomicU8);
impl FirstMinimumFault {
    const fn new() -> Self {
        Self(core::sync::atomic::AtomicU8::new(0))
    }
    fn record(&self, error: WorkError) {
        let code = match error {
            WorkError::Unavailable => 1,
            WorkError::WrongCpu => 2,
            WorkError::InterruptContext => 3,
            WorkError::Busy => 4,
            WorkError::Full => 5,
            WorkError::Closed => 6,
            WorkError::Exhausted => 7,
            WorkError::InvalidStack => 8,
            WorkError::InvalidTicket => 9,
            WorkError::WouldDeadlock => 10,
            WorkError::TimedOut => 11,
            WorkError::CorruptContext => 12,
            WorkError::Pending => 13,
        };
        let _ = self.0.compare_exchange(
            0,
            code,
            core::sync::atomic::Ordering::AcqRel,
            core::sync::atomic::Ordering::Acquire,
        );
    }
    fn get(&self) -> Option<WorkError> {
        match self.0.load(core::sync::atomic::Ordering::Acquire) {
            0 => None,
            1 => Some(WorkError::Unavailable),
            2 => Some(WorkError::WrongCpu),
            3 => Some(WorkError::InterruptContext),
            4 => Some(WorkError::Busy),
            5 => Some(WorkError::Full),
            6 => Some(WorkError::Closed),
            7 => Some(WorkError::Exhausted),
            8 => Some(WorkError::InvalidStack),
            9 => Some(WorkError::InvalidTicket),
            10 => Some(WorkError::WouldDeadlock),
            11 => Some(WorkError::TimedOut),
            12 => Some(WorkError::CorruptContext),
            13 => Some(WorkError::Pending),
            _ => unreachable!("private write-once minimum fault encoding"),
        }
    }
}
static FIRST_MINIMUM_FAULT: FirstMinimumFault = FirstMinimumFault::new();
/// Complete first minimum/source wake failure, retained as diagnostic metadata.
/// This observation supplies no current, source, IRQ, readiness or retry grant.
/// There is intentionally no clear/reset operation.
pub fn minimum_failure() -> Option<WorkError> {
    FIRST_MINIMUM_FAULT.get()
}
pub(super) fn fatal(error: WorkError) -> ! {
    FIRST_MINIMUM_FAULT.record(error);
    // SAFETY: unconditional terminal selected-source stop; no callback, unwind,
    // heap access, native loan release or owner/source Drop can follow failure.
    unsafe { core::arch::asm!("ud2", options(noreturn)) }
}
unsafe fn current(context: *mut c_void) -> (*const RawOwner, *mut c_void) {
    // SAFETY: original entered C callback; require_current authenticates source.
    let owner = unsafe { stela_linux_owner() };
    // SAFETY: C checks that this owner is installed and authenticates current.
    let task = unsafe { stela_linux_minimum_task(owner) };
    // SAFETY: on successful authentication the installed owner is live.
    let matches = !owner.is_null() && unsafe { (*owner).context == context };
    if !matches {
        fatal(WorkError::corrupt());
    }
    (owner, task)
}
pub(super) unsafe fn delay<P: Platform>(context: *mut c_void, minimum: u64) {
    // SAFETY: callback context belongs to the exact retained owner.
    let _identity = unsafe { current(context) };
    // SAFETY: this typed callback uses the authenticated owner’s retained context.
    if let Err(error) = unsafe { platform::<P>(context) }.minimum_delay_us(minimum) {
        fatal(error);
    }
}
pub(super) unsafe fn begin<P: Platform>(context: *mut c_void, minimum: u64) -> *mut Header {
    // SAFETY: callback context belongs to the exact retained owner.
    let (owner, task) = unsafe { current(context) };
    // SAFETY: this typed callback uses the authenticated owner’s retained context.
    let source = match unsafe { platform::<P>(context) }.minimum_deadline_us(minimum) {
        Ok(source) => source,
        Err(error) => fatal(error),
    };
    let value = Box::new(Owned {
        header: Header {
            ready,
            wake_ns,
            remaining,
            release,
        },
        source: ManuallyDrop::new(source),
        owner,
        task,
    });
    Box::into_raw(value).cast()
}
pub(super) unsafe fn timer_changed<P: Platform>(context: *mut c_void) {
    // SAFETY: no native/current/process loan survives this bounded callback.
    let _identity = unsafe { current(context) };
    // SAFETY: this typed callback uses the authenticated owner’s retained context.
    if let Err(error) = unsafe { platform::<P>(context) }.timer_changed() {
        // Queue mutation already happened. Never roll it back or return a false
        // completed arm/removal; the actual owner and pending queue stay kept.
        fatal(error);
    }
}
unsafe fn owned<'a>(pointer: *mut Header) -> &'a Owned {
    if pointer.is_null() {
        fatal(WorkError::InvalidTicket);
    }
    // SAFETY: private original constructor returns this exact live allocation;
    // C owns it until explicit release, including every parked stack frame.
    let value = unsafe { &*pointer.cast::<Owned>() };
    // SAFETY: the original C callback calls with a live deadline. Check its
    // retained owner against the installed owner, then authenticate current.
    let matches = unsafe {
        stela_linux_owner() == value.owner && stela_linux_minimum_task(value.owner) == value.task
    };
    if !matches {
        fatal(WorkError::corrupt());
    }
    value
}
unsafe extern "C" fn ready(pointer: *mut Header) -> bool {
    // SAFETY: C retains the deadline until its sole joined release.
    match unsafe { owned(pointer) }.source.ready() {
        Ok(value) => value,
        Err(error) => fatal(error),
    }
}
unsafe extern "C" fn wake_ns(pointer: *mut Header) -> i64 {
    // SAFETY: C retains the deadline until its sole joined release.
    match unsafe { owned(pointer) }.source.wake_ns() {
        Ok(value) if (0..i64::MAX).contains(&value) => value,
        Ok(_) => fatal(WorkError::Exhausted),
        Err(error) => fatal(error),
    }
}
unsafe extern "C" fn remaining(pointer: *mut Header, quantum_us: u64) -> u64 {
    if quantum_us == 0 {
        fatal(WorkError::InvalidTicket);
    }
    // SAFETY: C retains the deadline until its sole joined release.
    match unsafe { owned(pointer) }
        .source
        .remaining_quanta_ceil(quantum_us)
    {
        Ok(value) => value,
        Err(error) => fatal(error),
    }
}
unsafe extern "C" fn release(pointer: *mut Header) {
    // SAFETY: C still owns the deadline through its sole release callback.
    let value = unsafe { owned(pointer) };
    if let Err(error) = value.source.ready() {
        fatal(error);
    }
    // SAFETY: correct original current; C joined its timer before this sole
    // release. Removing a deadline subreference proves no kernel/root teardown.
    let mut value = unsafe { Box::from_raw(pointer.cast::<Owned>()) };
    // SAFETY: the source is initialized and this is its sole extraction.
    let source = unsafe { ManuallyDrop::take(&mut value.source) };
    drop(source);
}

#[cfg(test)]
mod tests;
