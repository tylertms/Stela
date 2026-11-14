//! Actual stopped metric publication, then genuine source-process task put.
use super::*;
/// Default abandonment retains actual native token, source task reference,
/// code/root/heap backing and sink. No numeric completion/error mints this.
pub(crate) struct StoppedTaskData {
    sink: ManuallyDrop<Arc<SourceData>>,
    stopped: ManuallyDrop<StoppedPublication>,
    source_released: bool,
}
impl<const N: usize> TaskBinding<N> {
    /// Exact base actual physical-return/IRQ-reader joined token, with no
    /// borrowed native permit escaping in the returned affine owner.
    pub(crate) fn stop_data(
        &self,
        runtime: Pin<&Runtime<N>>,
    ) -> Result<StoppedTaskData, WorkError> {
        let sink = self.source_data.as_ref().ok_or(WorkError::Unavailable)?;
        let stopped = ManuallyDrop::new(runtime.take_stopped_publication(self.first()?)?);
        let erased: Arc<dyn ExecutionPublication> = sink.clone();
        if !stopped.matches(&erased) || !sink.matches(stopped.identity()) {
            return Err(WorkError::corrupt()); // actual stopped owner retained
        }
        Ok(StoppedTaskData {
            sink: ManuallyDrop::new(sink.clone()),
            stopped,
            source_released: false,
        })
    }
}
impl StoppedTaskData {
    /// # Safety
    /// Genuine source supervisor process, no native permit, row/request lock,
    /// registry reader or RefCell borrow across original put/destructor code.
    pub(crate) unsafe fn release_source<P: crate::threads::KernelThreadPlatform, const N: usize>(
        self,
        kernel: &crate::kernel::KernelOwner<P, N>,
    ) -> Result<Self, (Self, ResourceBootstrapError)> {
        let mut this = ManuallyDrop::new(self);
        if this.source_released
            || this.sink.owner != kernel.raw()
            || this.sink.phase.load(Ordering::Acquire) != ACTIVE
            || !this.sink.matches(this.stopped.identity())
        {
            return Err((
                ManuallyDrop::into_inner(this),
                ResourceBootstrapError::Work(WorkError::corrupt()),
            ));
        }
        let identity = this.stopped.identity();
        // C compares exact retained identity, never constructs a native stack
        // or current from these metadata. No TaskContext/stack view is minted.
        let view = DataIdentity {
            queue_id: identity.queue_id(),
            context_id: identity.context_id(),
            sequence: identity.sequence(),
        };
        // SAFETY: genuine source process plus independently held exact native
        // stopped/reader receipt; no native permit spans original C disposal.
        let value =
            unsafe { stela_linux_task_data_release(this.sink.lease.as_ptr(), kernel.raw(), &view) };
        if value != 0 {
            return Err((
                ManuallyDrop::into_inner(this),
                ResourceBootstrapError::Native(value),
            ));
        }
        // Actual C source reference release and callback tail returned. No
        // ordinary fallible postcheck can conceal its accepted effects.
        this.source_released = true;
        this.sink.phase.store(SOURCE_RELEASED, Ordering::Release);
        Ok(ManuallyDrop::into_inner(this))
    }
    /// # Safety
    /// Source release above completed and its actual C callback tail joined.
    /// Fresh bounded native metadata scope only, no original C body/wait.
    pub(crate) unsafe fn finish(
        self,
        entered: &dyn EnteredProcess,
    ) -> Result<(), (Self, WorkError)> {
        let mut this = ManuallyDrop::new(self);
        if !this.source_released || this.sink.phase.load(Ordering::Acquire) != SOURCE_RELEASED {
            return Err((ManuallyDrop::into_inner(this), WorkError::Pending));
        }
        // SAFETY: affine stopped owner is transferred once before its callback.
        let stopped = unsafe { ManuallyDrop::take(&mut this.stopped) };
        // SAFETY: real source ref and C callback tail returned above; this is
        // a fresh bounded process metadata scope, not an owning suspended loan.
        match unsafe { stopped.finish(entered) } {
            Ok(()) => {
                this.sink.dispose_backing();
                // SAFETY: actual native and C writer/read/ref tails all joined.
                unsafe { ManuallyDrop::drop(&mut this.sink) };
                Ok(())
            }
            Err((stopped, error)) => {
                this.stopped = ManuallyDrop::new(stopped);
                Err((ManuallyDrop::into_inner(this), error))
            }
        }
    }
}
