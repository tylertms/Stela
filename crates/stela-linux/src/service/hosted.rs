//! The original service loop on the permanent hosted Initial stack.
use super::*;
use core::{mem::ManuallyDrop, ptr};
use stela_driver_base::platform::work::kernel_domain::{HostedBatch, HostedRuntime};

struct Service<'a, const N: usize> {
    owner: &'a RawOwner,
    runtime: Pin<&'a HostedRuntime<N>>,
    batch: Option<HostedBatch>,
}

/// Run the original timer service on this mounted Initial's reserved stack.
///
/// # Safety
/// Enter from the mounted Initial's active source scope after original
/// resource initialization, with no entered owner guard or live batch.
/// The caller retains the same runtime, source owner and callback graph
/// for the life of the service.
pub unsafe fn run_hosted_service<const N: usize>(
    owner: &RawOwner,
    runtime: Pin<&HostedRuntime<N>>,
) -> ! {
    if !runtime
        .service_current()
        .unwrap_or_else(|error| timer::fatal(error))
    {
        timer::fatal(WorkError::Unavailable);
    }
    let mut service = Service {
        owner,
        runtime,
        batch: None,
    };
    let mut call = Call {
        owner,
        context: ptr::from_mut(&mut service).cast(),
        begin: begin::<N>,
        next_irq: no_native_irq,
        complete_irq: unexpected_irq,
        drain: no_deferred_leaves,
        finish: finish::<N>,
        done: permanent,
    };
    // SAFETY: C owns entry and parking in the actual Initial's active scope.
    // The call and its batch slot live on this stack through every suspension.
    unsafe { stela_linux_service_loop(ptr::from_mut(&mut call).cast()) };
    timer::fatal(WorkError::corrupt());
}

unsafe extern "C" fn begin<const N: usize>(context: *mut c_void) -> bool {
    // SAFETY: the original loop lends this slot to one callback at a time.
    let service = unsafe { &mut *context.cast::<Service<'_, N>>() };
    if service.batch.is_some() {
        timer::fatal(WorkError::corrupt());
    }
    service.batch = service
        .runtime
        .begin_service_batch()
        .unwrap_or_else(|error| timer::fatal(error));
    service.batch.is_some()
}

unsafe extern "C" fn finish<const N: usize>(context: *mut c_void) {
    let (owner, runtime, batch) = {
        // SAFETY: the exact begin slot is exclusively lent to this callback.
        let service = unsafe { &mut *context.cast::<Service<'_, N>>() };
        let batch = service
            .batch
            .take()
            .unwrap_or_else(|| timer::fatal(WorkError::corrupt()));
        (service.owner, service.runtime, ManuallyDrop::new(batch))
    };
    // SAFETY: exact current, retained owner and captured batch. No service
    // slot borrow spans delivery of the original callbacks.
    let deadline = unsafe { run_hosted_timers(owner, runtime, &batch) }
        .unwrap_or_else(|error| timer::fatal(error));
    // SAFETY: original callbacks returned and this is their real next deadline.
    unsafe { runtime.finish_service_batch(ManuallyDrop::into_inner(batch), deadline) }
        .unwrap_or_else(|error| timer::fatal(error));
    // SAFETY: still on this mounted Initial, after the timer capture returned.
    if unsafe { crate::task::hosted_service_progress(owner, runtime) }.is_err() {
        timer::fatal(WorkError::corrupt());
    }
}

unsafe extern "C" fn no_native_irq(_: *mut c_void) -> c_int {
    // Hosted source has no native line owner.
    -1
}

unsafe extern "C" fn unexpected_irq(_: *mut c_void) {
    timer::fatal(WorkError::corrupt());
}

unsafe extern "C" fn no_deferred_leaves(_: *mut c_void) {}

unsafe extern "C" fn permanent(_: *mut c_void) -> bool {
    false
}
