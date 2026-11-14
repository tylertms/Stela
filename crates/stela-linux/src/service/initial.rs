//! Private permanent original Initial startup, not full-root readiness.
use super::*;
use crate::task::ResourceBootstrapError;
use core::mem::ManuallyDrop;

/// One real bootstrap operation mounted by KernelOwner. It keeps no owning
/// Kernel/Environment/Runtime/Queue edge. The canonical mount retains the
/// borrowed kernel separately for the entire permanent service lifetime.
///
/// # Safety
/// Implementations must keep their borrowed kernel and broker valid for the
/// lifetime of the mounted service and use only that mount's source context.
pub(crate) unsafe trait Bootstrap<const N: usize> {
    /// Actual canonical Initial current, with no native metadata/row/entered
    /// borrow held. Original helper bodies may really park and resume.
    unsafe fn run(&self, binding: &Arc<TaskBinding<N>>) -> Result<(), ResourceBootstrapError>;
    /// Consume only real recorded broker events outside every passive leaf.
    unsafe fn service(&self);
}

pub(crate) struct Initial<const N: usize> {
    binding: ManuallyDrop<Arc<TaskBinding<N>>>,
    body: ManuallyDrop<Box<dyn Bootstrap<N>>>,
    phase: AtomicU8,
    failure: UnsafeCell<Option<ResourceBootstrapError>>,
}
impl<const N: usize> Initial<N> {
    pub(crate) fn new(binding: Arc<TaskBinding<N>>, body: Box<dyn Bootstrap<N>>) -> Self {
        // Retain both roots before validation or any later foreign/native call.
        let binding = ManuallyDrop::new(binding);
        let body = ManuallyDrop::new(body);
        if !binding.original_initial() {
            timer::fatal(WorkError::corrupt());
        }
        Self {
            binding,
            body,
            phase: AtomicU8::new(0),
            failure: UnsafeCell::new(None),
        }
    }
    pub(crate) fn binding(&self) -> &Arc<TaskBinding<N>> {
        &self.binding
    }
    pub(crate) fn bootstrap(&self) {
        match self
            .phase
            .compare_exchange(0, 1, Ordering::AcqRel, Ordering::Acquire)
        {
            Ok(_) => {}
            Err(2) => return,
            Err(_) => timer::fatal(WorkError::corrupt()),
        }
        // SAFETY: only actual begin on this kept original Initial calls here;
        // no affine metadata/EnteredProcess/row borrow spans original C.
        match unsafe { self.body.run(&self.binding) } {
            Ok(()) => self.phase.store(2, Ordering::Release),
            Err(error) => {
                // SAFETY: the successful phase transition grants the sole
                // write. Readers cannot access the error until phase 3.
                unsafe { *self.failure.get() = Some(error) };
                self.phase.store(3, Ordering::Release);
                // SAFETY: UD2 terminates this path without calling into C.
                unsafe { core::arch::asm!("ud2", options(noreturn)) }
            }
        }
    }
    pub(crate) fn service(&self) {
        if self.phase.load(Ordering::Acquire) != 2 {
            timer::fatal(WorkError::corrupt());
        }
        // SAFETY: called by the exact Initial begin after original bootstrap,
        // before any passive leaf/IRQ/timer delivery; no native row/loan held.
        unsafe { self.body.service() };
    }
    /// Diagnostic only; no resource, child or joined FullReady authority.
    pub(crate) fn failure(&self) -> Option<ResourceBootstrapError> {
        if self.phase.load(Ordering::Acquire) != 3 {
            return None;
        }
        // SAFETY: release phase3 publishes this immutable terminal diagnostic.
        unsafe { *self.failure.get() }
    }
}
