//! Private C wait-frame owner, created by the target's concrete binding callback.
//! Source retains actual task usage + a record LIFETIME pin independently.
use super::*;
use alloc::boxed::Box;
use stela_driver_base::platform::work::{InactiveObservation, InactiveTarget};

trait Retention: Send + Sync {
    fn native(&self) -> Result<InactiveTarget, WorkError>;
}
impl<const N: usize> Retention for TargetTask<N> {
    fn native(&self) -> Result<InactiveTarget, WorkError> {
        self.wake.retain_inactive()
    }
}

struct InactiveOwner {
    target: ManuallyDrop<Box<dyn Retention>>,
    native: ManuallyDrop<Option<InactiveTarget>>,
    caller: Option<(usize, ContextId, u64)>,
    error: Option<WorkError>,
}

fn errno(error: WorkError) -> i32 {
    match error {
        WorkError::Closed => -116,
        WorkError::Busy => -16,
        WorkError::Full => -28,
        WorkError::Unavailable => -95,
        _ => -22,
    }
}

impl<const N: usize> TaskBinding<N> {
    pub(super) unsafe extern "C" fn retain_inactive(
        binding: *mut c_void,
        sequence: u64,
        out: *mut *mut c_void,
    ) -> i32 {
        if binding.is_null() || out.is_null() {
            return -22;
        }
        // SAFETY: target's OWN installed TaskOps callback receives its actual
        // Arc-backed binding loan. C holds that original record's lifetime pin
        // and task usage reference. No caller-N cast or pointer adoption occurs.
        let binding_ref = unsafe { &*binding.cast::<Self>() };
        if !binding_ref.published.load(Ordering::Acquire)
            || binding_ref.detached.load(Ordering::Acquire)
            || sequence == 0
        {
            return -116;
        }
        // SAFETY: publication initialized target; same actual UP process has no
        // concurrent arrive mutation. The immutable handle captures this job only.
        let wake = unsafe { (*binding_ref.target.get()).assume_init_ref() }.clone();
        if wake.sequence() != sequence {
            return -116;
        }
        // SAFETY: C's installed binding is borrowed from this SAME retained Arc.
        // This increments its real owner before reconstructing that one new ref.
        unsafe { Arc::increment_strong_count(binding.cast::<Self>()) };
        // SAFETY: precisely the increment above, never a fresh raw-task constructor.
        let binding = unsafe { Arc::from_raw(binding.cast::<Self>()) };
        let target = TargetTask {
            binding: ManuallyDrop::new(binding),
            wake: ManuallyDrop::new(wake),
        };
        // Park the WHOLE concrete target and opaque shell before process/provider
        // checks. A callback failure/unwind cannot dismantle the retained graph.
        let mut owner = ManuallyDrop::new(Box::new(InactiveOwner {
            target: ManuallyDrop::new(Box::new(target)),
            native: ManuallyDrop::new(None),
            caller: None,
            error: None,
        }));
        // SAFETY: out is the actual private wait frame's writable opaque slot.
        unsafe { *out = core::ptr::from_mut(&mut **owner).cast() };
        let result = owner.target.native();
        match result {
            Ok(native) => {
                *owner.native = Some(native);
                // SAFETY is not needed for transfer: no ordinary Drop was enabled.
                let _ = Box::into_raw(ManuallyDrop::into_inner(owner));
                0
            }
            Err(error) => {
                owner.error = Some(error);
                let _ = Box::into_raw(ManuallyDrop::into_inner(owner));
                errno(error)
            }
        }
    }

    pub(super) unsafe extern "C" fn observe_inactive(
        scope: *mut c_void,
        opaque: *mut c_void,
        force_wait: bool,
    ) -> i32 {
        if scope.is_null() || opaque.is_null() {
            return -22;
        }
        // SAFETY: actual run_scoped frame + privately retained opaque owner;
        // neither pointer/borrow escapes this bounded, nonwaiting callback.
        let runtime = unsafe { &*scope.cast::<Scope<'_, N>>() }.runtime;
        // SAFETY: ONLY target.retain_inactive creates this nongeneric shell.
        let owner = unsafe { &mut *opaque.cast::<InactiveOwner>() };
        let result = (|| {
            if let Some(error) = owner.error {
                return Err(error);
            }
            let waiter = runtime.wake_handle()?;
            let identity = (
                core::ptr::from_ref(&*runtime) as usize,
                waiter.context_id(),
                waiter.sequence(),
            );
            if owner.caller.is_some_and(|caller| caller != identity) {
                return Err(WorkError::Unavailable);
            }
            owner.caller = Some(identity);
            let native = owner.native.as_ref().ok_or_else(|| WorkError::corrupt())?;
            runtime.observe_inactive(native, &waiter, force_wait)
        })();
        match result {
            Ok(InactiveObservation::Inactive) => 1,
            Ok(InactiveObservation::Subscribed) => 0,
            Err(error) => {
                owner.error = Some(error);
                errno(error)
            }
        }
    }

    pub(super) unsafe extern "C" fn end_inactive(scope: *mut c_void, opaque: *mut c_void) -> i32 {
        if scope.is_null() || opaque.is_null() {
            return -22;
        }
        // SAFETY: same retained source scope and private opaque owner as observe.
        let runtime = unsafe { &*scope.cast::<Scope<'_, N>>() }.runtime;
        // SAFETY: actual original wait frame alone owns this retained allocation.
        let owner = unsafe { &mut *opaque.cast::<InactiveOwner>() };
        let result = (|| {
            if let Some(error) = owner.error {
                return Err(error);
            }
            let waiter = runtime.wake_handle()?;
            let identity = (
                core::ptr::from_ref(&*runtime) as usize,
                waiter.context_id(),
                waiter.sequence(),
            );
            if owner.caller.is_some_and(|caller| caller != identity) {
                return Err(WorkError::Unavailable);
            }
            runtime.cancel_inactive(&waiter)?;
            owner
                .native
                .as_ref()
                .ok_or_else(|| WorkError::corrupt())?
                .check_process()
        })();
        if let Err(error) = result {
            owner.error = Some(error);
            return errno(error);
        }
        // Keep the whole shell parked until all potentially fallible release
        // callbacks return. If one unwinds the actual owner allocation survives.
        // SAFETY: unique C frame transfers its actual allocation only after cancel.
        let mut owner = ManuallyDrop::new(unsafe { Box::from_raw(opaque.cast::<InactiveOwner>()) });
        // SAFETY: actual same process, no registered waiter, C still pins source.
        unsafe {
            ManuallyDrop::drop(&mut owner.target);
            ManuallyDrop::drop(&mut owner.native);
        }
        drop(ManuallyDrop::into_inner(owner));
        0
    }
}
