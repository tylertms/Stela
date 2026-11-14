//! Private bounded loan from the actual service's owned Domain::Batch.
use super::*;
use core::marker::PhantomData;
unsafe extern "C" {
    fn stela_linux_service_run(delivery: *const c_void) -> i64;
    fn stela_linux_require_current(owner: *const RawOwner) -> i32;
}
/// Not an entry/IRQ/counter or data-write grant. Constructor requires an actual
/// nonforgeable batch and same reserved native/source service current.
pub(super) struct TimerDelivery<'a> {
    owner: *const RawOwner,
    context: *const c_void,
    batch: *const c_void,
    check: unsafe fn(*const c_void, *const c_void) -> Result<(), WorkError>,
    _environment: PhantomData<&'a ()>,
    _local: PhantomData<*mut ()>,
}
impl<'a> TimerDelivery<'a> {
    pub(super) fn new<const N: usize>(
        environment: &'a Environment<N>,
        batch: &'a Batch,
    ) -> Result<Self, WorkError> {
        let delivery = Self {
            owner: environment.owner,
            context: core::ptr::from_ref(environment).cast(),
            batch: core::ptr::from_ref(batch).cast(),
            check: check_native::<N>,
            _environment: PhantomData,
            _local: PhantomData,
        };
        delivery.validate(environment.owner)?;
        Ok(delivery)
    }

    #[cfg(all(
        feature = "hosted-runtime",
        target_os = "linux",
        target_arch = "x86_64"
    ))]
    pub(super) fn hosted<const N: usize>(
        owner: &'a RawOwner,
        runtime: Pin<&'a stela_driver_base::platform::work::kernel_domain::HostedRuntime<N>>,
        batch: &'a stela_driver_base::platform::work::kernel_domain::HostedBatch,
    ) -> Result<Self, WorkError> {
        let delivery = Self {
            owner,
            context: core::ptr::from_ref(runtime.get_ref()).cast(),
            batch: core::ptr::from_ref(batch).cast(),
            check: check_hosted::<N>,
            _environment: PhantomData,
            _local: PhantomData,
        };
        delivery.validate(owner)?;
        Ok(delivery)
    }

    fn validate(&self, owner: *const RawOwner) -> Result<(), WorkError> {
        if owner.is_null() || owner != self.owner {
            return Err(WorkError::InvalidTicket);
        }
        // SAFETY: the private constructor paired these exact live operands.
        unsafe { (self.check)(self.context, self.batch) }?;
        // SAFETY: no callback or switch between runtime and source validation.
        if unsafe { stela_linux_require_current(owner) } != 0 {
            return Err(WorkError::Unavailable);
        }
        Ok(())
    }
}
unsafe fn check_native<const N: usize>(
    context: *const c_void,
    batch: *const c_void,
) -> Result<(), WorkError> {
    // SAFETY: the native constructor borrows these exact live operands.
    let (environment, batch) =
        unsafe { (&*context.cast::<Environment<N>>(), &*batch.cast::<Batch>()) };
    environment
        .domain
        .validate_batch(environment.runtime(), batch)
}

#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
unsafe fn check_hosted<const N: usize>(
    context: *const c_void,
    batch: *const c_void,
) -> Result<(), WorkError> {
    use stela_driver_base::platform::work::kernel_domain::{HostedBatch, HostedRuntime};
    // SAFETY: the hosted constructor borrows these exact live operands.
    let (runtime, batch) = unsafe {
        (
            &*context.cast::<HostedRuntime<N>>(),
            &*batch.cast::<HostedBatch>(),
        )
    };
    runtime.validate_service_batch(batch)
}

pub(super) unsafe fn run(delivery: &TimerDelivery<'_>) -> Option<u64> {
    // SAFETY: caller preserves genuine current, actual batch and bounded loan
    // through this original C operation; no sleeping/reentry/token escape.
    let deadline = unsafe { stela_linux_service_run(core::ptr::from_ref(delivery).cast()) };
    if deadline == i64::MAX {
        None
    } else {
        Some(u64::try_from(deadline).unwrap_or_else(|_| fatal(WorkError::corrupt())))
    }
}
static FIRST_FAILURE: core::sync::atomic::AtomicU8 = core::sync::atomic::AtomicU8::new(0);
pub(super) fn failure() -> Option<WorkError> {
    match FIRST_FAILURE.load(Ordering::Acquire) {
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
        _ => Some(WorkError::corrupt()),
    }
}
pub(super) fn fatal(error: WorkError) -> ! {
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
    let _ = FIRST_FAILURE.compare_exchange(0, code, Ordering::AcqRel, Ordering::Acquire);
    // SAFETY: a void source callback cannot return over uncertain timer
    // effects/batch ownership. Preserve all mounted graphs before any return.
    unsafe { core::arch::asm!("ud2", options(noreturn)) }
}
/// # Safety
/// C passes ONLY the original live scoped pointer lent by run above, without
/// copying/retaining/reconstructing its contents. The opaque C declaration has
/// no constructor. No callback or wait may outlive the original run call.
#[unsafe(no_mangle)]
unsafe extern "C" fn stela_linux_timer_delivery_check(
    delivery: *const c_void,
    owner: *const RawOwner,
) -> i32 {
    if delivery.is_null() || owner.is_null() {
        return -22;
    }
    // SAFETY: actual private source ABI's same live scoped Rust loan.
    let delivery = unsafe { &*delivery.cast::<TimerDelivery<'_>>() };
    match delivery.validate(owner) {
        Ok(()) => 0,
        Err(WorkError::InvalidTicket | WorkError::CorruptContext) => -22,
        Err(WorkError::Closed) => -108,
        Err(_) => -11,
    }
}
