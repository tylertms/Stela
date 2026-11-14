//! Actual permanent Initial service producer; no bare-task fallback.
use super::*;
use crate::{
    service::initial::{Bootstrap, Initial},
    task::{InitialFailure, ResourceBootstrapError, TaskBinding},
};

pub enum InitialStartError {
    Initial(InitialFailure),
    Service(WorkError),
}
impl core::fmt::Debug for InitialStartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Self::Initial(failure) => f.debug_tuple("Initial").field(&failure.error).finish(),
            Self::Service(error) => f.debug_tuple("Service").field(error).finish(),
        }
    }
}
impl core::fmt::Display for InitialStartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "original Initial startup: {self:?}")
    }
}
impl core::error::Error for InitialStartError {}
struct SourceBootstrap<
    P: crate::threads::KernelThreadPlatform + 'static,
    Q: crate::pages::Provider + 'static,
    const N: usize,
> {
    kernel: core::ptr::NonNull<KernelOwner<P, N>>,
    threads: &'static crate::task::runtime::TaskRuntime<P, Q, N>,
}
// SAFETY: the unsafe pinned start contract keeps this SAME actual kernel;
// raw borrowed pointer avoids a Kernel→Environment→Kernel owning cycle. No
// CPU/IO/current grant is derived from the pointer. Each C entry verifies it.
unsafe impl<
    P: crate::threads::KernelThreadPlatform + 'static,
    Q: crate::pages::Provider + 'static,
    const N: usize,
> Bootstrap<N> for SourceBootstrap<P, Q, N>
{
    unsafe fn run(&self, binding: &Arc<TaskBinding<N>>) -> Result<(), ResourceBootstrapError> {
        // SAFETY: exact pinned permanent mount supplied by start_initial.
        let kernel = unsafe { self.kernel.as_ref() };
        // SAFETY: the bootstrap caller holds the canonical Initial context
        // without native metadata loans, and the kernel mount stays pinned.
        unsafe { binding.initialize_resources(kernel) }?;
        // SAFETY: resource initialization succeeded on this same Initial
        // continuation, with no transient metadata loan spanning the call.
        unsafe { binding.start_threads(kernel) }
    }
    unsafe fn service(&self) {
        // SAFETY: the bootstrap service contract provides canonical source
        // context outside passive leaves. The original broker stays mounted.
        unsafe { self.threads.service_delivery() }
    }
}
impl<P: crate::threads::KernelThreadPlatform + 'static, const N: usize> KernelOwner<P, N> {
    /// Start the one genuine original init_task on the native Service lane.
    /// This accepts native service publication, not PID1/kthreadd/WQ execution,
    /// joined SRCU probes, required periodic tick or external FullReady.
    ///
    /// # Safety
    /// Keep this SAME pinned kernel, native Runtime/Domain, code/root/heap and
    /// page inventory permanently mounted (including errors/abandonment).
    /// The exact actual root argument belongs to this kernel's same physical
    /// source. Heap callbacks must already be registered before C task claim;
    /// same-owner canonical page inventory and native scheduler Domain, fork
    /// broker, CPU topology/time/timer/IRQ providers must genuinely exist before
    /// the first service step can execute original resource/clone helpers.
    /// Caller publishes the exact Environment to the platform before dispatch.
    /// Every dispatcher segment uses a fresh real native execution guard with
    /// all transient loans ended before physical return. No borrowed permit,
    /// queue/row/RefCell guard may span this permanent service's original waits.
    #[expect(
        clippy::result_large_err,
        reason = "startup refusal returns retained ownership without another allocation"
    )]
    pub unsafe fn start_initial<Q: crate::pages::Provider + 'static>(
        self: Pin<&Self>,
        root: Arc<dyn crate::threads::OriginalKernelRoot>,
        threads: &'static crate::task::runtime::TaskRuntime<P, Q, N>,
    ) -> Result<(), InitialStartError> {
        if !threads.matches_kernel(self.get_ref()) {
            return Err(InitialStartError::Service(WorkError::corrupt()));
        }
        self.environment()
            .preflight_initial()
            .map_err(InitialStartError::Service)?;
        // SAFETY: caller supplies exact original root and bounded actual
        // dispatcher metadata context. Constructor retains claim on refusal.
        let binding = unsafe { TaskBinding::new_initial(self.get_ref(), root) }
            .map_err(InitialStartError::Initial)?;
        let startup = Initial::new(
            binding,
            Box::new(SourceBootstrap {
                kernel: core::ptr::NonNull::from(self.get_ref()),
                threads,
            }),
        );
        // SAFETY: startup graph is retained before native publication and the
        // caller keeps this same pinned permanent mount even on error.
        unsafe { self.environment().start(startup) }.map_err(InitialStartError::Service)
    }
}
