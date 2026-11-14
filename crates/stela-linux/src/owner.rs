//! The owner of a Linux environment: the table of `stela_linux_services` Linux
//! code calls back through (`compat/linux/stela/linux.h`), over a [`Platform`].
//!
//! This is the whole boundary. A platform answers in Rust types and negative
//! errnos; the table's trampolines are the only code here that sees the C ABI,
//! and nothing else in this crate needs to know how a service reaches hardware.
use alloc::boxed::Box;
use stela_driver_base::platform::work::WorkError;
mod realtime;
mod time;
use core::{
    ffi::{CStr, c_char, c_int, c_uint, c_void},
    marker::PhantomData,
    pin::Pin,
    ptr, slice,
};
pub use realtime::{RealtimeOrigin, RealtimeRead};
pub use time::{MinimumDeadline, minimum_failure};

/// A negative Linux errno.
pub type Errno = i32;

/// IRQ enable failure with an explicit original no-publication proof.
/// Retained is the safe default: errno alone never classifies hardware effects.
#[derive(Debug, Eq, PartialEq)]
pub struct IrqEnableError {
    errno: Errno,
    unexposed: bool,
}
impl IrqEnableError {
    pub const fn retained(errno: Errno) -> Self {
        Self {
            errno,
            unexposed: false,
        }
    }
    /// # Safety
    /// This exact original attempt published no native route/callback producer
    /// able to reach the already-linked new C action. Actual returned native
    /// owners were canceled in their original context; any issued service
    /// epoch was closed/joined. A status code or cleared bit proves none of this.
    pub unsafe fn unexposed(errno: Errno) -> Self {
        Self {
            errno,
            unexposed: true,
        }
    }
    pub const fn errno(self) -> Errno {
        self.errno
    }
}

/// What Linux code needs of the machine it runs on. Each method's contract is
/// its entry in `stela_linux_services`.
pub trait Platform {
    /// Monotonic nanoseconds.
    fn now_ns(&self) -> i64;
    /// Elapsed nanoseconds from this Linux instance's boot epoch, including
    /// suspend. A no-suspend source must own continuity from that epoch.
    /// Read the retained source without allocation, sleeping or switching.
    fn boottime_ns(&self) -> Result<u64, WorkError> {
        Err(WorkError::Unavailable)
    }
    /// Map a sample from this platform's monotonic clock into UTC. The
    /// platform retains its correlation and multigrain floor for this owner.
    fn realtime_at(&self, _ns: i64, _read: RealtimeRead) -> Option<(i64, u32)> {
        None
    }
    /// Calibrated x86 TSC frequency for Linux boot-time BogoMIPS scaling.
    /// Read retained metadata without blocking or reentering Linux.
    /// This observation grants no physical minimum or deadline authority.
    fn calibrated_tsc_hz(&self) -> Option<core::num::NonZeroU64> {
        None
    }
    /// Actual source-owned atomic minimum. No sleep, allocation, nominal
    /// frequency conversion or success fallback is permitted.
    fn minimum_delay_us(&self, minimum: u64) -> Result<(), WorkError>;
    /// Create ONE owned relative physical deadline on the same actual clock
    /// as now_ns/service timers. It retains source/provider/code across park;
    /// no borrowed execution/process/queue scope may escape this call.
    fn minimum_deadline_us(&self, minimum: u64) -> Result<Box<dyn MinimumDeadline>, WorkError>;
    /// Notify the same retained service of a real C timer queue change or a
    /// posted source softirq. This IRQ-safe callback wakes service inspection.
    fn timer_changed(&self) -> Result<(), WorkError>;
    /// One formatted kernel message at its printk level.
    fn log(&self, level: i32, message: &CStr);
    /// A `WARN` that fired, with its condition when Linux records one.
    fn warn(&self, file: &CStr, line: i32, condition: Option<&CStr>);
    /// An operation Stela does not provide; the caller continues degraded.
    fn unsupported(&self, operation: &CStr);
    /// A terminal stop of Linux code, already reported through [`Self::log`]
    /// at `LOGLEVEL_EMERG`: an owner with a stop of its own names it here and
    /// does not return. The C caller traps when this returns, as by default.
    fn fatal(&self, _message: &CStr) {}
    /// Bytes from an already seeded generator. Must not sleep or allocate.
    fn random_bytes(&self, bytes: &mut [u8]);
    /// Mix auxiliary device input into the same retained stream. False means
    /// this owner lacks an input provider. No entropy is credited by this call.
    fn mix_random(&self, _bytes: &[u8]) -> bool {
        false
    }
    /// Power the machine off.
    fn shutdown(&self, force: bool);
    /// Map page-aligned device memory uncached, or write-combining where its
    /// stores may merge.
    fn map_device(&self, physical: u64, size: usize, write_combining: bool) -> Option<*mut c_void>;
    /// # Safety
    /// `address` came from [`Self::map_device`] and nothing uses it any more.
    unsafe fn unmap_device(&self, address: *mut c_void);
    /// Map `physical` pages contiguously, write-back.
    fn map_pages(&self, physical: &[u64]) -> Option<*mut c_void>;
    /// # Safety
    /// `address` came from [`Self::map_pages`] for `count` pages and nothing
    /// uses it any more.
    unsafe fn unmap_pages(&self, address: *mut c_void, count: usize);
    /// Route `irq` to the service, which calls `stela_linux_irq_run`.
    fn irq_enable(&self, irq: u32) -> Result<(), IrqEnableError>;
    /// Reversibly mask this source. Old queued delivery may still reach its
    /// retained service epoch; this grants no action/endpoint retirement.
    fn irq_disable(&self, irq: u32) -> Result<(), Errno>;
    /// Retire the exact action after the C registry joined its action threads.
    /// Return only after root/CPU and actual service-epoch joining; errors
    /// retain every owner and C must not return/reclaim its request or dev_id.
    fn irq_retire(&self, irq: u32) -> Result<(), Errno>;
    /// # Safety
    /// `to` is writable for `size` bytes; `from` is an address in the running
    /// client. Returns the bytes left uncopied at a fault.
    unsafe fn copy_from_user(&self, to: *mut u8, from: *const u8, size: usize) -> usize;
    /// # Safety
    /// `from` is readable for `size` bytes; `to` is an address in the running
    /// client. Returns the bytes left uncopied at a fault.
    unsafe fn copy_to_user(&self, to: *mut u8, from: *const u8, size: usize) -> usize;
    /// Read kernel memory through the owner's bounded, fault-safe mapping.
    ///
    /// # Safety
    /// `to` is writable for `size` bytes. The owner must validate the source
    /// range before reading, and may leave only a copied prefix on a fault.
    unsafe fn copy_from_kernel_nofault(
        &self,
        _to: *mut u8,
        _from: *const u8,
        _size: usize,
    ) -> Result<(), Errno> {
        Err(-95)
    }
    /// Observe the whole free frames within one registered ordinary RAM extent.
    /// This is a process-context snapshot. It must not allocate, transfer
    /// ownership, or change the active Linux lease.
    fn available_page_frames(&self, _start: usize, _pages: usize) -> Result<usize, Errno> {
        Err(-95)
    }
    /// Install one page in client `space`. `prot` holds x86 page table bits.
    fn map_user(&self, space: *mut c_void, address: u64, pfn: u64, prot: u64) -> Result<(), Errno>;
    /// Remove a client's pages and confirm that stale translations cannot
    /// reach them. An error retains the Linux mapping and backing.
    fn unmap_user(&self, space: *mut c_void, address: u64, size: usize) -> Result<(), Errno>;
}

/// `struct stela_linux_services`, in that header's order.
#[repr(C)]
struct Services {
    now_ns: unsafe extern "C" fn(*mut c_void) -> i64,
    minimum_delay_us: unsafe extern "C" fn(*mut c_void, u64),
    minimum_deadline_us: unsafe extern "C" fn(*mut c_void, u64) -> *mut time::Header,
    timer_changed: unsafe extern "C" fn(*mut c_void),
    log: unsafe extern "C" fn(*mut c_void, c_int, *const c_char),
    warn: unsafe extern "C" fn(*mut c_void, *const c_char, c_int, *const c_char),
    unsupported: unsafe extern "C" fn(*mut c_void, *const c_char),
    random_bytes: unsafe extern "C" fn(*mut c_void, *mut c_void, usize),
    shutdown: unsafe extern "C" fn(*mut c_void, bool),
    map_device: unsafe extern "C" fn(*mut c_void, u64, usize, bool) -> *mut c_void,
    unmap_device: unsafe extern "C" fn(*mut c_void, *mut c_void),
    map_pages: unsafe extern "C" fn(*mut c_void, *const u64, usize) -> *mut c_void,
    unmap_pages: unsafe extern "C" fn(*mut c_void, *mut c_void, usize),
    irq_enable: unsafe extern "C" fn(*mut c_void, c_uint, *mut bool) -> c_int,
    irq_disable: unsafe extern "C" fn(*mut c_void, c_uint) -> c_int,
    irq_retire: unsafe extern "C" fn(*mut c_void, c_uint) -> c_int,
    copy_from_user: unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_void, u64) -> u64,
    copy_to_user: unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_void, u64) -> u64,
    map_user: unsafe extern "C" fn(*mut c_void, *mut c_void, u64, u64, u64) -> c_int,
    unmap_user: unsafe extern "C" fn(*mut c_void, *mut c_void, u64, usize) -> c_int,
    calibrated_tsc_hz: unsafe extern "C" fn(*mut c_void) -> u64,
    copy_from_kernel_nofault:
        unsafe extern "C" fn(*mut c_void, *mut c_void, *const c_void, usize) -> c_int,
    available_page_frames: unsafe extern "C" fn(*mut c_void, usize, usize, *mut usize) -> c_int,
    realtime_at: unsafe extern "C" fn(*mut c_void, i64, u32, u32, *mut i64, *mut u32) -> bool,
    mix_random: unsafe extern "C" fn(*mut c_void, *const c_void, usize) -> bool,
    boottime_ns: unsafe extern "C" fn(*mut c_void, *mut i64) -> bool,
    fatal: unsafe extern "C" fn(*mut c_void, *const c_char),
}

const _: () = assert!(size_of::<Services>() == 216);
const _: () = {
    assert!(core::mem::offset_of!(Services, now_ns) == 0);
    assert!(core::mem::offset_of!(Services, minimum_delay_us) == 8);
    assert!(core::mem::offset_of!(Services, minimum_deadline_us) == 16);
    assert!(core::mem::offset_of!(Services, timer_changed) == 24);
    assert!(core::mem::offset_of!(Services, log) == 32);
    assert!(core::mem::offset_of!(Services, warn) == 40);
    assert!(core::mem::offset_of!(Services, unsupported) == 48);
    assert!(core::mem::offset_of!(Services, random_bytes) == 56);
    assert!(core::mem::offset_of!(Services, shutdown) == 64);
    assert!(core::mem::offset_of!(Services, map_device) == 72);
    assert!(core::mem::offset_of!(Services, unmap_device) == 80);
    assert!(core::mem::offset_of!(Services, map_pages) == 88);
    assert!(core::mem::offset_of!(Services, unmap_pages) == 96);
    assert!(core::mem::offset_of!(Services, irq_enable) == 104);
    assert!(core::mem::offset_of!(Services, irq_disable) == 112);
    assert!(core::mem::offset_of!(Services, irq_retire) == 120);
    assert!(core::mem::offset_of!(Services, copy_from_user) == 128);
    assert!(core::mem::offset_of!(Services, copy_to_user) == 136);
    assert!(core::mem::offset_of!(Services, map_user) == 144);
    assert!(core::mem::offset_of!(Services, unmap_user) == 152);
    assert!(core::mem::offset_of!(Services, calibrated_tsc_hz) == 160);
    assert!(core::mem::offset_of!(Services, copy_from_kernel_nofault) == 168);
    assert!(core::mem::offset_of!(Services, available_page_frames) == 176);
    assert!(core::mem::offset_of!(Services, realtime_at) == 184);
    assert!(core::mem::offset_of!(Services, mix_random) == 192);
    assert!(core::mem::offset_of!(Services, boottime_ns) == 200);
    assert!(core::mem::offset_of!(Services, fatal) == 208);
};

/// `struct stela_linux_owner`.
#[repr(C)]
pub struct RawOwner {
    services: *const Services,
    context: *mut c_void,
}

const _: () = assert!(size_of::<RawOwner>() == 16);

/// # Safety
/// `context` is the `P` an [`Owner`] gave Linux, alive while Linux runs.
unsafe fn platform<'a, P: Platform>(context: *mut c_void) -> &'a P {
    // SAFETY: the caller's contract.
    unsafe { &*context.cast::<P>() }
}

/// # Safety
/// `text` is Linux's, NUL-terminated and live for the call.
unsafe fn text<'a>(text: *const c_char) -> &'a CStr {
    // SAFETY: the caller's contract.
    unsafe { CStr::from_ptr(text) }
}

fn status(result: Result<(), Errno>) -> c_int {
    result.err().unwrap_or(0)
}

/// The trampolines' safety is that of the table: Linux calls each with the
/// context [`Owner::new`] gave it and the arguments `linux.h` documents.
mod trampolines {
    use super::*;

    pub unsafe extern "C" fn now_ns<P: Platform>(context: *mut c_void) -> i64 {
        // SAFETY: see the module.
        unsafe { platform::<P>(context) }.now_ns()
    }

    pub unsafe extern "C" fn boottime_ns<P: Platform>(context: *mut c_void, now: *mut i64) -> bool {
        // SAFETY: see the module. The owner lends its original boot clock.
        let Ok(sample) = unsafe { platform::<P>(context) }.boottime_ns() else {
            return false;
        };
        // KTIME_MAX is a sentinel, never an actual time sample.
        let Ok(sample) = i64::try_from(sample) else {
            return false;
        };
        if sample == i64::MAX {
            return false;
        }
        // SAFETY: C lends one writable output, left unchanged on refusal.
        unsafe { now.write(sample) };
        true
    }

    pub unsafe extern "C" fn realtime_at<P: Platform>(
        context: *mut c_void,
        ns: i64,
        mode: u32,
        tick_ns: u32,
        seconds: *mut i64,
        nanoseconds: *mut u32,
    ) -> bool {
        let read = match (mode, core::num::NonZeroU32::new(tick_ns)) {
            (0, None) => RealtimeRead::Fine,
            (1, Some(tick)) => RealtimeRead::CoarseMultigrain(tick),
            (2, None) => RealtimeRead::FineMultigrain,
            _ => return false,
        };
        // SAFETY: see the module.
        let Some((sec, nanos)) = (unsafe { platform::<P>(context) }).realtime_at(ns, read) else {
            return false;
        };
        if nanos >= 1_000_000_000 {
            return false;
        }
        // SAFETY: both output pointers address independent writable C locals.
        unsafe {
            seconds.write(sec);
            nanoseconds.write(nanos);
        }
        true
    }

    pub unsafe extern "C" fn calibrated_tsc_hz<P: Platform>(context: *mut c_void) -> u64 {
        // SAFETY: see the module.
        unsafe { platform::<P>(context) }
            .calibrated_tsc_hz()
            .map_or(0, core::num::NonZeroU64::get)
    }

    pub unsafe extern "C" fn minimum_delay_us<P: Platform>(context: *mut c_void, minimum: u64) {
        // SAFETY: same exact source-owned Platform callback context.
        unsafe { time::delay::<P>(context, minimum) };
    }
    pub unsafe extern "C" fn minimum_deadline_us<P: Platform>(
        context: *mut c_void,
        minimum: u64,
    ) -> *mut time::Header {
        // SAFETY: returned object owns its source graph, never this Rust borrow.
        unsafe { time::begin::<P>(context, minimum) }
    }

    pub unsafe extern "C" fn timer_changed<P: Platform>(context: *mut c_void) {
        // SAFETY: original actual-current Platform callback, no borrowed permit.
        unsafe { time::timer_changed::<P>(context) };
    }

    pub unsafe extern "C" fn log<P: Platform>(
        context: *mut c_void,
        level: c_int,
        message: *const c_char,
    ) {
        // SAFETY: see the module.
        unsafe { platform::<P>(context).log(level, text(message)) };
    }

    pub unsafe extern "C" fn warn<P: Platform>(
        context: *mut c_void,
        file: *const c_char,
        line: c_int,
        condition: *const c_char,
    ) {
        // SAFETY: see the module; a condition is optional.
        unsafe {
            let condition = (!condition.is_null()).then(|| text(condition));
            platform::<P>(context).warn(text(file), line, condition);
        }
    }

    pub unsafe extern "C" fn fatal<P: Platform>(context: *mut c_void, message: *const c_char) {
        // SAFETY: see the module.
        unsafe { platform::<P>(context).fatal(text(message)) };
    }

    pub unsafe extern "C" fn unsupported<P: Platform>(
        context: *mut c_void,
        operation: *const c_char,
    ) {
        // SAFETY: see the module.
        unsafe { platform::<P>(context).unsupported(text(operation)) };
    }

    pub unsafe extern "C" fn random_bytes<P: Platform>(
        context: *mut c_void,
        bytes: *mut c_void,
        size: usize,
    ) {
        // SAFETY: see the module; `bytes` is writable for `size`.
        unsafe {
            platform::<P>(context).random_bytes(slice::from_raw_parts_mut(bytes.cast(), size));
        }
    }

    pub unsafe extern "C" fn mix_random<P: Platform>(
        context: *mut c_void,
        bytes: *const c_void,
        size: usize,
    ) -> bool {
        let input = if size == 0 {
            &[]
        } else {
            // SAFETY: the C caller validated the range and lends the device bytes for this call.
            unsafe { slice::from_raw_parts(bytes.cast(), size) }
        };
        // SAFETY: see the module. The C caller serializes access to this owner.
        unsafe { platform::<P>(context) }.mix_random(input)
    }

    pub unsafe extern "C" fn shutdown<P: Platform>(context: *mut c_void, force: bool) {
        // SAFETY: see the module.
        unsafe { platform::<P>(context) }.shutdown(force);
    }

    pub unsafe extern "C" fn map_device<P: Platform>(
        context: *mut c_void,
        physical: u64,
        size: usize,
        write_combining: bool,
    ) -> *mut c_void {
        // SAFETY: see the module.
        unsafe { platform::<P>(context) }
            .map_device(physical, size, write_combining)
            .unwrap_or(ptr::null_mut())
    }

    pub unsafe extern "C" fn unmap_device<P: Platform>(context: *mut c_void, address: *mut c_void) {
        // SAFETY: see the module; Linux passes what map_device returned.
        unsafe { platform::<P>(context).unmap_device(address) };
    }

    pub unsafe extern "C" fn map_pages<P: Platform>(
        context: *mut c_void,
        physical: *const u64,
        count: usize,
    ) -> *mut c_void {
        // SAFETY: see the module; `physical` holds `count` addresses.
        unsafe {
            platform::<P>(context)
                .map_pages(slice::from_raw_parts(physical, count))
                .unwrap_or(ptr::null_mut())
        }
    }

    pub unsafe extern "C" fn unmap_pages<P: Platform>(
        context: *mut c_void,
        address: *mut c_void,
        count: usize,
    ) {
        // SAFETY: see the module; Linux passes what map_pages returned.
        unsafe { platform::<P>(context).unmap_pages(address, count) };
    }

    pub unsafe extern "C" fn irq_enable<P: Platform>(
        context: *mut c_void,
        irq: c_uint,
        unexposed: *mut bool,
    ) -> c_int {
        if unexposed.is_null() {
            return -22;
        }
        // SAFETY: C supplies this writable result for the immediate call. It
        // starts conservative even if a later native callback aborts/panics.
        unsafe { unexposed.write(false) };
        // SAFETY: original entered owner and its classified native attempt.
        match unsafe { platform::<P>(context) }.irq_enable(irq) {
            Ok(()) => 0,
            Err(error) => {
                // SAFETY: the same immediate writable C result checked above.
                unsafe { unexposed.write(error.unexposed) };
                error.errno
            }
        }
    }

    pub unsafe extern "C" fn irq_disable<P: Platform>(context: *mut c_void, irq: c_uint) -> c_int {
        // SAFETY: see the module.
        status(unsafe { platform::<P>(context) }.irq_disable(irq))
    }
    pub unsafe extern "C" fn irq_retire<P: Platform>(context: *mut c_void, irq: c_uint) -> c_int {
        // SAFETY: same exact entered owner's C registry/action join contract.
        status(unsafe { platform::<P>(context) }.irq_retire(irq))
    }

    pub unsafe extern "C" fn copy_from_user<P: Platform>(
        context: *mut c_void,
        to: *mut c_void,
        from: *const c_void,
        size: u64,
    ) -> u64 {
        // SAFETY: see the module.
        let left =
            unsafe { platform::<P>(context).copy_from_user(to.cast(), from.cast(), size as usize) };
        left as u64
    }

    pub unsafe extern "C" fn copy_to_user<P: Platform>(
        context: *mut c_void,
        to: *mut c_void,
        from: *const c_void,
        size: u64,
    ) -> u64 {
        // SAFETY: see the module.
        let left =
            unsafe { platform::<P>(context).copy_to_user(to.cast(), from.cast(), size as usize) };
        left as u64
    }

    pub unsafe extern "C" fn copy_from_kernel_nofault<P: Platform>(
        context: *mut c_void,
        to: *mut c_void,
        from: *const c_void,
        size: usize,
    ) -> c_int {
        // SAFETY: C authenticates this owner and retains a writable destination.
        status(unsafe {
            platform::<P>(context).copy_from_kernel_nofault(to.cast(), from.cast(), size)
        })
    }

    pub unsafe extern "C" fn available_page_frames<P: Platform>(
        context: *mut c_void,
        start: usize,
        pages: usize,
        available: *mut usize,
    ) -> c_int {
        // SAFETY: C keeps the canonical owner and a writable stack output.
        match unsafe { platform::<P>(context) }.available_page_frames(start, pages) {
            Ok(count) => {
                // SAFETY: output is valid through this call, and written once.
                unsafe { available.write(count) };
                0
            }
            Err(error) => status(Err(error)),
        }
    }

    pub unsafe extern "C" fn map_user<P: Platform>(
        context: *mut c_void,
        space: *mut c_void,
        address: u64,
        pfn: u64,
        prot: u64,
    ) -> c_int {
        // SAFETY: see the module.
        status(unsafe { platform::<P>(context) }.map_user(space, address, pfn, prot))
    }

    pub unsafe extern "C" fn unmap_user<P: Platform>(
        context: *mut c_void,
        space: *mut c_void,
        address: u64,
        size: usize,
    ) -> c_int {
        // SAFETY: see the module.
        status(unsafe { platform::<P>(context) }.unmap_user(space, address, size))
    }
}

/// A platform and the table Linux reaches it through. Pinned, because Linux
/// keeps the addresses of both.
pub struct Owner<P: Platform> {
    services: Services,
    raw: RawOwner,
    platform: P,
    _pin: core::marker::PhantomPinned,
}

unsafe extern "C" {
    fn stela_linux_enter(owner: *const RawOwner);
    fn stela_linux_exit();
}

/// Linux code may run until this drops. Tasks stay entered across their
/// switches, so one entry lasts as long as its task's work.
pub struct Entered(PhantomData<*mut ()>);

impl Entered {
    /// # Safety
    /// On a task of the owner's environment, with `owner` alive past the drop.
    pub unsafe fn new(owner: *const RawOwner) -> Self {
        // SAFETY: the caller's contract.
        unsafe { stela_linux_enter(owner) };
        Self(PhantomData)
    }
}

impl Drop for Entered {
    fn drop(&mut self) {
        // SAFETY: `new` entered, on this task.
        unsafe { stela_linux_exit() };
    }
}

impl<P: Platform> Owner<P> {
    pub fn new(platform: P) -> Pin<Box<Self>> {
        use trampolines as t;
        let mut owner = Box::new(Self {
            services: Services {
                now_ns: t::now_ns::<P>,
                boottime_ns: t::boottime_ns::<P>,
                realtime_at: t::realtime_at::<P>,
                minimum_delay_us: t::minimum_delay_us::<P>,
                minimum_deadline_us: t::minimum_deadline_us::<P>,
                timer_changed: t::timer_changed::<P>,
                log: t::log::<P>,
                warn: t::warn::<P>,
                unsupported: t::unsupported::<P>,
                fatal: t::fatal::<P>,
                random_bytes: t::random_bytes::<P>,
                mix_random: t::mix_random::<P>,
                shutdown: t::shutdown::<P>,
                map_device: t::map_device::<P>,
                unmap_device: t::unmap_device::<P>,
                map_pages: t::map_pages::<P>,
                unmap_pages: t::unmap_pages::<P>,
                irq_enable: t::irq_enable::<P>,
                irq_disable: t::irq_disable::<P>,
                irq_retire: t::irq_retire::<P>,
                copy_from_user: t::copy_from_user::<P>,
                copy_from_kernel_nofault: t::copy_from_kernel_nofault::<P>,
                available_page_frames: t::available_page_frames::<P>,
                copy_to_user: t::copy_to_user::<P>,
                map_user: t::map_user::<P>,
                unmap_user: t::unmap_user::<P>,
                calibrated_tsc_hz: t::calibrated_tsc_hz::<P>,
            },
            raw: RawOwner {
                services: ptr::null(),
                context: ptr::null_mut(),
            },
            platform,
            _pin: core::marker::PhantomPinned,
        });
        owner.raw.services = &raw const owner.services;
        owner.raw.context = (&raw mut owner.platform).cast();
        Box::into_pin(owner)
    }

    /// What `stela_linux_enter` takes.
    pub fn raw(&self) -> *const RawOwner {
        &raw const self.raw
    }

    pub fn platform(&self) -> &P {
        &self.platform
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use core::cell::{Cell, RefCell};
    use std::{string::String, vec::Vec};
    extern crate std;

    #[derive(Default)]
    struct Fake {
        tsc_hz: Cell<Option<core::num::NonZeroU64>>,
        events: RefCell<Vec<String>>,
        fatal_log: Option<std::path::PathBuf>,
        /// Where an owner with a stop of its own records it, then exits.
        stop: Option<std::path::PathBuf>,
    }

    impl Fake {
        fn note(&self, event: String) {
            self.events.borrow_mut().push(event);
        }
    }

    impl Platform for Fake {
        fn calibrated_tsc_hz(&self) -> Option<core::num::NonZeroU64> {
            self.tsc_hz.get()
        }

        fn now_ns(&self) -> i64 {
            42
        }
        fn minimum_delay_us(&self, minimum: u64) -> Result<(), WorkError> {
            let _ = minimum;
            Err(WorkError::Unavailable)
        }
        fn minimum_deadline_us(&self, _: u64) -> Result<Box<dyn MinimumDeadline>, WorkError> {
            // This service-table mock has no actual relative time source.
            // Negative admission is explicit; it never manufactures readiness.
            Err(WorkError::Unavailable)
        }
        fn timer_changed(&self) -> Result<(), WorkError> {
            Err(WorkError::Unavailable) // no actual Environment in this Bare table fixture
        }
        fn log(&self, level: i32, message: &CStr) {
            if let Some(path) = &self.fatal_log {
                std::fs::write(path, b"logger reached").unwrap();
            }
            self.note(std::format!("log {level} {}", message.to_str().unwrap()));
        }
        fn warn(&self, file: &CStr, line: i32, condition: Option<&CStr>) {
            self.note(std::format!("warn {file:?} {line} {condition:?}"));
        }
        fn unsupported(&self, operation: &CStr) {
            self.note(std::format!("unsupported {operation:?}"));
        }
        fn fatal(&self, message: &CStr) {
            if let Some(path) = &self.stop {
                std::fs::write(path, message.to_bytes()).unwrap();
                std::process::exit(17);
            }
        }
        fn random_bytes(&self, bytes: &mut [u8]) {
            bytes.fill(7);
        }
        fn shutdown(&self, force: bool) {
            self.note(std::format!("shutdown {force}"));
        }
        fn map_device(
            &self,
            physical: u64,
            size: usize,
            write_combining: bool,
        ) -> Option<*mut c_void> {
            (size != 0).then_some((physical | u64::from(write_combining)) as *mut c_void)
        }
        unsafe fn unmap_device(&self, address: *mut c_void) {
            self.note(std::format!("unmap device {:#x}", address as usize));
        }
        fn map_pages(&self, physical: &[u64]) -> Option<*mut c_void> {
            physical.first().map(|&page| page as *mut c_void)
        }
        unsafe fn unmap_pages(&self, address: *mut c_void, count: usize) {
            self.note(std::format!("unmap pages {:#x} {count}", address as usize));
        }
        fn irq_enable(&self, irq: u32) -> Result<(), IrqEnableError> {
            if irq == 0 {
                // SAFETY: this exact model refused before every effect.
                Err(unsafe { IrqEnableError::unexposed(-22) })
            } else {
                Ok(())
            }
        }
        fn irq_disable(&self, irq: u32) -> Result<(), Errno> {
            self.note(std::format!("irq off {irq}"));
            Ok(())
        }
        fn irq_retire(&self, irq: u32) -> Result<(), Errno> {
            self.note(std::format!("irq retire {irq}"));
            Ok(())
        }
        unsafe fn copy_from_user(&self, to: *mut u8, from: *const u8, size: usize) -> usize {
            // SAFETY: the tests pass valid buffers.
            unsafe { ptr::copy_nonoverlapping(from, to, size) };
            0
        }
        unsafe fn copy_to_user(&self, _: *mut u8, _: *const u8, size: usize) -> usize {
            size
        }
        fn map_user(&self, _: *mut c_void, _: u64, pfn: u64, _: u64) -> Result<(), Errno> {
            if pfn == 0 { Err(-14) } else { Ok(()) }
        }
        fn unmap_user(&self, space: *mut c_void, address: u64, size: usize) -> Result<(), Errno> {
            self.note(std::format!(
                "unmap user {:#x} {address:#x} {size}",
                space as usize
            ));
            Ok(())
        }
    }

    #[cfg(all(unix, target_arch = "x86_64"))]
    #[test]
    fn minimum_failure_terminal_component_does_not_log_or_drop() {
        use std::os::unix::process::ExitStatusExt;
        const CHILD: &str = "STELA_MINIMUM_FATAL_COMPONENT";
        if let Some(directory) = std::env::var_os(CHILD) {
            let directory = std::path::PathBuf::from(directory);
            let logged = directory.join("logged");
            let owner = Owner::new(Fake {
                fatal_log: Some(logged.clone()),
                ..Fake::default()
            });
            // SAFETY: isolated diagnostic-table component process, one exact
            // retained owner, balanced hosted Bare C entry. This does NOT call
            // require_current, a minimum provider, timer delivery or native IO.
            unsafe { stela_linux_enter(owner.raw()) };
            unsafe extern "C" {
                fn stela_linux_owner() -> *const RawOwner;
            }
            // SAFETY: this exact diagnostic owner remains entered and retained.
            assert_eq!(unsafe { stela_linux_owner() }, owner.raw());
            // SAFETY: invoke this exact live retained table/context to prove
            // its diagnostic logger is connected before the terminal probe.
            unsafe {
                ((*(*owner.raw()).services).log)((*owner.raw()).context, 0, c"probe".as_ptr())
            };
            assert!(logged.exists());
            std::fs::remove_file(&logged).unwrap();
            struct DropWitness(std::path::PathBuf);
            impl Drop for DropWitness {
                fn drop(&mut self) {
                    std::fs::write(&self.0, b"dropped").unwrap();
                }
            }
            let _witness = DropWitness(directory.join("dropped"));
            std::fs::write(directory.join("armed"), b"exact diagnostic owner entered").unwrap();
            time::fatal(WorkError::Closed);
        }
        let directory = std::env::temp_dir().join(std::format!(
            "stela-minimum-fatal-{}-{}",
            std::process::id(),
            std::time::SystemTime::now()
                .duration_since(std::time::UNIX_EPOCH)
                .unwrap()
                .as_nanos()
        ));
        std::fs::create_dir(&directory).unwrap();
        let status = std::process::Command::new(std::env::current_exe().unwrap())
            .args([
                "--exact",
                "owner::tests::minimum_failure_terminal_component_does_not_log_or_drop",
                "--nocapture",
            ])
            .env(CHILD, &directory)
            .status()
            .unwrap();
        assert_eq!(status.signal(), Some(4)); // actual UD2, not Rust panic/unwind
        assert!(directory.join("armed").exists());
        assert!(!directory.join("logged").exists());
        assert!(!directory.join("dropped").exists());
        std::fs::remove_dir_all(directory).unwrap();
    }

    #[cfg(all(unix, target_arch = "x86_64"))]
    #[test]
    fn a_linux_fatal_reaches_the_owners_own_stop_after_its_report() {
        use std::os::unix::process::ExitStatusExt;
        const CHILD: &str = "STELA_OWNER_FATAL_STOP";
        const TEST: &str =
            "owner::tests::a_linux_fatal_reaches_the_owners_own_stop_after_its_report";
        unsafe extern "C" {
            fn stela_linux_fatal(message: *const c_char) -> !;
        }
        if let Some(directory) = std::env::var_os(CHILD) {
            let directory = std::path::PathBuf::from(directory);
            let stop = directory.join("stop");
            let owner = Owner::new(Fake {
                fatal_log: Some(directory.join("logged")),
                stop: (!stop.exists()).then_some(stop),
                ..Fake::default()
            });
            // SAFETY: isolated process, one retained owner, balanced entry.
            unsafe { stela_linux_enter(owner.raw()) };
            // SAFETY: the entered owner stays retained; this never returns.
            unsafe { stela_linux_fatal(c"IRQ action retirement unavailable".as_ptr()) };
        }
        let run = |directory: &std::path::Path| {
            std::process::Command::new(std::env::current_exe().unwrap())
                .args(["--exact", TEST, "--nocapture"])
                .env(CHILD, directory)
                .status()
                .unwrap()
        };
        let directory = tempfile::tempdir().unwrap();
        // An owner with a stop of its own ends the process there: no trap.
        let status = run(directory.path());
        assert_eq!(status.code(), Some(17), "{status:?}");
        assert!(directory.path().join("logged").exists());
        assert_eq!(
            std::fs::read_to_string(directory.path().join("stop")).unwrap(),
            "IRQ action retirement unavailable"
        );
        // An owner whose stop returns still traps after the report.
        std::fs::remove_file(directory.path().join("logged")).unwrap();
        let status = run(directory.path());
        assert_eq!(status.signal(), Some(libc::SIGILL), "{status:?}");
        assert!(directory.path().join("logged").exists());
    }

    #[test]
    fn ordinary_services_reach_platform_and_minima_require_actual_current() {
        let owner = Owner::new(Fake::default());
        // SAFETY: the raw owner stays valid while `owner` lives, and each call
        // follows its `linux.h` contract with live local buffers.
        unsafe {
            let raw = &*owner.raw();
            let (s, context) = (&*raw.services, raw.context);
            assert_eq!((s.now_ns)(context), 42);
            assert_eq!((s.calibrated_tsc_hz)(context), 0);
            owner
                .platform()
                .tsc_hz
                .set(core::num::NonZeroU64::new(3_200_000_000));
            assert_eq!((s.calibrated_tsc_hz)(context), 3_200_000_000);
            // This Bare table fixture cannot invoke the actual-current minimum
            // callback. The separate producer tests own their counter model.
            assert!((s.minimum_delay_us as usize) != 0);
            assert!((s.minimum_deadline_us as usize) != 0);

            (s.log)(context, 4, c"hello".as_ptr());
            (s.warn)(context, c"a.c".as_ptr(), 9, ptr::null());
            (s.warn)(context, c"a.c".as_ptr(), 9, c"x".as_ptr());
            (s.unsupported)(context, c"swap".as_ptr());
            let mut bytes = [0u8; 4];
            (s.random_bytes)(context, bytes.as_mut_ptr().cast(), 4);
            assert_eq!(bytes, [7; 4]);
            (s.shutdown)(context, false);

            assert_eq!((s.map_device)(context, 0x1000, 0, false), ptr::null_mut());
            assert_eq!((s.map_device)(context, 0x1000, 4096, true) as usize, 0x1001);
            let device = (s.map_device)(context, 0x1000, 4096, false);
            assert_eq!(device as usize, 0x1000);
            (s.unmap_device)(context, device);
            assert_eq!((s.map_pages)(context, [].as_ptr(), 0), ptr::null_mut());
            let pages = (s.map_pages)(context, [0x2000u64, 0x9000].as_ptr(), 2);
            assert_eq!(pages as usize, 0x2000);
            (s.unmap_pages)(context, pages, 2);

            let mut unexposed = false;
            assert_eq!((s.irq_enable)(context, 0, &mut unexposed), -22);
            assert!(unexposed);
            assert_eq!((s.irq_enable)(context, 5, &mut unexposed), 0);
            assert!(!unexposed);
            assert_eq!((s.irq_disable)(context, 5), 0);

            let source = [1u8, 2];
            let mut target = [0u8; 2];
            assert_eq!(
                (s.copy_from_user)(
                    context,
                    target.as_mut_ptr().cast(),
                    source.as_ptr().cast(),
                    2
                ),
                0
            );
            assert_eq!(target, source);
            assert_eq!(
                (s.copy_to_user)(
                    context,
                    target.as_mut_ptr().cast(),
                    source.as_ptr().cast(),
                    2
                ),
                2
            );
            assert_eq!((s.map_user)(context, 8 as *mut c_void, 0x4000, 0, 0), -14);
            assert_eq!((s.map_user)(context, 8 as *mut c_void, 0x4000, 1, 0), 0);
            assert_eq!((s.unmap_user)(context, 8 as *mut c_void, 0x4000, 4096), 0);
        }
        assert_eq!(
            *owner.platform().events.borrow(),
            [
                "log 4 hello",
                "warn \"a.c\" 9 None",
                "warn \"a.c\" 9 Some(\"x\")",
                "unsupported \"swap\"",
                "shutdown false",
                "unmap device 0x1000",
                "unmap pages 0x2000 2",
                "irq off 5",
                "unmap user 0x8 0x4000 4096",
            ]
        );
    }
}
