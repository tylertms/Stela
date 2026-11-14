//! Qualified stack backing survives publication until successor-side release.
use super::*;
mod dormant;
pub use dormant::DormantGeneralStack;
/// Reconstructed native frame, original excluded worker/slot and physical pin.
/// No borrowed process permission is stored or kept across suspension.
#[must_use = "publish or cancel; Drop retains the pin and closes the escrow"]
pub struct OwnedPinnedGeneral<'runtime, const N: usize> {
    escrow: OwnedGeneralEscrow<'runtime, N>,
    pin: Option<ManuallyDrop<PinnedPages>>,
    epoch: u64,
}
/// Exact native assignment with its original backing pin. Linux stack/task,
/// descriptor/PID/reaper and RCU lifetimes remain separately owned obligations.
#[must_use = "release only after native completion/detach and foreign retirement"]
pub struct PublishedGeneralStack<'runtime, const N: usize> {
    runtime: Pin<&'runtime Runtime<N>>,
    worker: usize,
    assigned: AssignedGeneral,
    nonce: u64,
    epoch: u64,
    queue: ManuallyDrop<Arc<Queue<N>>>,
    pin: Option<ManuallyDrop<PinnedPages>>,
}
impl<'runtime, const N: usize> PinnedGeneralStackPreparation<'runtime, N> {
    /// Restore Runtime's bootstrap words while retaining the excluded worker
    /// and backing. This publishes no callback or foreign task/current object.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal returns the exact cold backing owner without allocation"
    )]
    pub fn reconstruct(
        mut self,
        entered: &dyn EnteredProcess,
    ) -> Result<OwnedPinnedGeneral<'runtime, N>, (Self, WorkError)> {
        if self.prepared.pin.is_none() {
            return Err((self, WorkError::corrupt()));
        }
        if let Err(error) = self.prepared.reseed(entered, false) {
            return Err((self, error));
        }
        if let Err(error) = self
            .prepared
            .check_preparation(entered)
            .and_then(|()| entered.check())
        {
            return Err((self, error));
        }
        let GeneralStackPreparation {
            escrow, pin, epoch, ..
        } = self.prepared;
        Ok(OwnedPinnedGeneral {
            escrow,
            pin,
            epoch: epoch.unwrap(),
        })
    }
}
impl<const N: usize> OwnedPinnedGeneral<'_, N> {
    /// Synchronous descriptor setup inspects this kept pin under fresh matching
    /// creator permission. Addresses alone grant no foreign task ownership.
    #[cfg(feature = "linux-kernel")]
    pub fn inspect_pin<T>(
        &self,
        entered: &dyn EnteredProcess,
        inspect: impl FnOnce(&PinnedPages) -> T,
    ) -> Result<T, WorkError> {
        self.check(entered)?;
        let result = inspect(self.pin.as_ref().ok_or(WorkError::Pending)?);
        self.check(entered)?;
        Ok(result)
    }
    fn check(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        self.check_identity(entered, true)
    }
    fn check_identity(
        &self,
        entered: &dyn EnteredProcess,
        needs_pin: bool,
    ) -> Result<(), WorkError> {
        let owner = &self.escrow;
        owner.current(entered)?;
        let flags = owner.queue.cpu.save_disable()?;
        let result = (|| {
            owner.current(entered)?;
            let mut guard = owner.queue.lock()?;
            let state = guard.state();
            if state.poisoned || owner.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            owner.validate(state)?;
            // SAFETY: same actual excluded native allocation epoch.
            let task = unsafe { &*owner.runtime.task(owner.worker) };
            if self.epoch == 0 || task.stack_epoch != self.epoch {
                return Err(WorkError::corrupt());
            }
            if needs_pin && self.pin.is_none() {
                return Err(WorkError::Pending);
            }
            Ok(())
        })();
        // SAFETY: enclosing flags after all queue/task borrows have ended.
        unsafe { owner.queue.cpu.restore(flags) };
        result
    }
    /// Cancel native exclusion after the foreign producer has removed every
    /// unpublished descriptor/reference. Refusal permanently keeps the pin.
    ///
    /// # Safety
    /// No foreign pointer/reference or prospective callback can use this stack.
    #[expect(
        clippy::result_large_err,
        reason = "Cancellation keeps the actual pin/epoch on refusal"
    )]
    pub unsafe fn cancel(mut self, entered: &dyn EnteredProcess) -> Result<(), (Self, WorkError)> {
        // A previous post-destructor refusal deliberately keeps pin=None; only
        // actual native identity/epoch, not check(pin), gates its retry.
        if let Err(error) = self
            .check_identity(entered, false)
            .and_then(|()| entered.check())
        {
            return Err((self, error));
        }
        if self.pin.as_ref().is_some_and(|pin| pin.has_other_views()) {
            return Err((self, WorkError::Pending));
        }
        if let Some(pin) = self.pin.take() {
            drop(ManuallyDrop::into_inner(pin));
        }
        // Rebuild AFTER actual data-owner Drop, which may run external code.
        if let Err(error) =
            self.escrow
                .reset_prepared_frame(entered, self.epoch, TaskState::Escrow, true)
        {
            return Err((self, error));
        }
        let Self { escrow, pin, epoch } = self;
        match escrow.cancel(entered) {
            Ok(()) => Ok(()),
            Err((escrow, error)) => Err((Self { escrow, pin, epoch }, error)),
        }
    }
}
impl<'runtime, const N: usize> OwnedPinnedGeneral<'runtime, N> {
    /// Publish the native callback while retaining backing beyond completion.
    /// Failure returns all original pinned worker/slot ownership.
    ///
    /// # Safety
    /// As OwnedGeneralEscrow::commit. Foreign resource/PID publication shares
    /// this nonblocking CPU exclusion, with all fallible steps before visible
    /// publication. Keep callback/context/code before entering the publisher,
    /// including an unknown post-publication CPU restore/return-tail unwind.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal returns the exact pinned worker/slot without implicit resource release"
    )]
    pub unsafe fn commit(
        self,
        entered: &dyn EnteredProcess,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<PublishedGeneralStack<'runtime, N>, (Self, WorkError)> {
        // SAFETY: ordinary generic callback publication, never TASK_NEW child.
        unsafe { self.publish_mode(entered, callback, context, None) }
    }
    /// Internal shared ownership transfer. Dormant is a genuinely separate
    /// native scheduler state, not an errno/flag interpreted as C completion.
    #[expect(
        clippy::result_large_err,
        reason = "exact pinned affine owner on refusal"
    )]
    unsafe fn publish_mode(
        self,
        entered: &dyn EnteredProcess,
        callback: Callback,
        context: *mut c_void,
        dormant: Option<scheduling::Snapshot>,
    ) -> Result<PublishedGeneralStack<'runtime, N>, (Self, WorkError)> {
        if let Err(error) = self.check(entered) {
            return Err((self, error));
        }
        let queue = ManuallyDrop::new(self.escrow.queue.clone());
        if queue
            .published_stack_pins
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_add(1))
            .is_err()
        {
            drop(ManuallyDrop::into_inner(queue));
            return Err((self, WorkError::Exhausted));
        }
        let Self { escrow, pin, epoch } = self;
        let runtime = escrow.runtime;
        let worker = escrow.worker;
        let nonce = escrow.nonce;
        // SAFETY: caller keeps and coordinates the complete publication graph;
        // this ManuallyDrop pin survives an uncertain outer restore tail.
        match unsafe { escrow.commit_pinned(entered, epoch, dormant, callback, context) } {
            Ok(assigned) => Ok(PublishedGeneralStack {
                runtime,
                worker,
                assigned,
                nonce,
                epoch,
                queue,
                pin,
            }),
            Err((escrow, error)) => {
                if queue.published_stack_pins.fetch_sub(1, Ordering::AcqRel) == 0 {
                    queue.cpu.fatal(WorkError::corrupt());
                }
                drop(ManuallyDrop::into_inner(queue));
                Err((Self { escrow, pin, epoch }, error))
            }
        }
    }
}
impl<const N: usize> PublishedGeneralStack<'_, N> {
    pub fn assigned(&self) -> AssignedGeneral {
        self.assigned
    }
    fn successor(&self, release: bool) -> Result<(), WorkError> {
        let queue = &self.queue;
        self.runtime.dispatcher_successor()?;
        if !queue.completed(self.assigned.ticket)? {
            return Err(WorkError::Pending);
        }
        let flags = queue.cpu.save_disable()?;
        let result = (|| {
            self.runtime.dispatcher_successor()?;
            // No Task borrow spans the real Queue lock/save operation. The
            // original queue guard records an actual release, never calls a
            // sink here, and restores only the nested IF0 state on Drop.
            let mut guard = queue.lock()?;
            let state = guard.state();
            // SAFETY: exact constructor-kept worker on the checked CPU at IF0.
            // Its private nonce excludes all stack reuse, including bare work.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            if task.id != self.assigned.context
                || task.state != TaskState::Retained
                || task.stack_pin_nonce != self.nonce
                || task.job.is_some()
                || task.binding.is_some()
                || task.draining
                || task.signal.sequence.load(Ordering::Acquire) != 0
                || task.stack_epoch != self.epoch
                || task.completed_sequence != self.assigned.ticket.sequence
                || task.returned_sequence != self.assigned.ticket.sequence
            {
                return Err(WorkError::Busy);
            }
            if queue.published_stack_pins.load(Ordering::Acquire) == 0 {
                return Err(WorkError::corrupt());
            }
            if release {
                let shape = crate::platform::work::ReleasedStackGeometry::of(
                    &task.stack,
                    !task.ever_entered,
                );
                // The physical pin was actually destroyed before this fresh
                // successor check. Fallible bounded note precedes mutation;
                // notification waits for the outer dispatcher/IF tail.
                state.resources.stack_released(shape)?;
                task.stack_retired_epoch = self.epoch;
                task.stack_pin_nonce = 0;
                task.state = TaskState::Idle;
                if queue.published_stack_pins.fetch_sub(1, Ordering::AcqRel) == 0 {
                    queue.cpu.fatal(WorkError::corrupt());
                }
            }
            Ok(())
        })();
        // SAFETY: exact enclosing flags, after all task borrows have ended.
        unsafe { queue.cpu.restore(flags) };
        result
    }
    /// Release only on the successor dispatcher after the exact native callback
    /// completed and its binding detached. Drop/forget retain the queue/code,
    /// complete Runtime and physical pin, and exclude this worker from reuse.
    ///
    /// # Safety
    /// Separately join original C stack/task/RCU/PID/reaper references, aliases,
    /// descriptors and final callbacks. Native completion does not prove those
    /// retirements. Untracked raw/C/DMA users must have retired; counted pin or
    /// mapping views may remain and will refuse Pending before any disposal.
    /// For successful release every actual old user ended. Keep the original native
    /// Runtime/root/code and qualified process CPU through the complete release.
    pub unsafe fn release(mut self) -> Result<(), (Self, WorkError)> {
        if let Err(error) = self.successor(false) {
            return Err((self, error));
        }
        if self.pin.as_ref().is_some_and(|pin| pin.has_other_views()) {
            return Err((self, WorkError::Pending));
        }
        if let Some(pin) = self.pin.take() {
            // Keep the private worker nonce, full queue and retirement counter
            // until the backing destructor returns. Unwind retains all of them.
            drop(ManuallyDrop::into_inner(pin));
        }
        // A destructor may call external code: inspect fresh CPU/dispatcher and
        // exact native ownership again before releasing the excluded worker.
        // A refused postcheck retains the owner with pin=None; retry cannot
        // drop that completed backing view a second time.
        if let Err(error) = self.successor(true) {
            return Err((self, error));
        }
        let Self { queue, .. } = self;
        drop(ManuallyDrop::into_inner(queue));
        Ok(())
    }
}

impl<const N: usize> core::fmt::Debug for OwnedPinnedGeneral<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("OwnedPinnedGeneral")
            .field("epoch", &self.epoch)
            .finish_non_exhaustive()
    }
}
impl<const N: usize> core::fmt::Debug for PinnedGeneralStackPreparation<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PinnedGeneralStackPreparation")
            .field("epoch", &self.prepared.epoch)
            .finish_non_exhaustive()
    }
}

impl<const N: usize> core::fmt::Debug for PublishedGeneralStack<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("PublishedGeneralStack")
            .field("epoch", &self.epoch)
            .finish_non_exhaustive()
    }
}

impl<const N: usize> PublishedGeneralStack<'_, N> {
    /// Actual native physical-return receipt observation while the same affine
    /// owner keeps this worker Retained and all stale wakes closed. This can be
    /// inspected from another real source task; it is not C/RCU/page retirement
    /// or new CPU/root permission. Final pin disposal rechecks actual successor.
    pub fn physically_returned(&self) -> Result<bool, WorkError> {
        self.queue.process()?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            let _guard = self.queue.lock()?;
            // SAFETY: same actual Runtime worker/nonce at IF0, immutable kept
            // vector. No Task reference spans restore or foreign callbacks.
            let task = unsafe { &*self.runtime.task(self.worker) };
            if task.id != self.assigned.context
                || task.stack_pin_nonce != self.nonce
                || task.stack_epoch != self.epoch
            {
                return Err(WorkError::corrupt());
            }
            Ok(task.state == TaskState::Retained
                && task.job.is_none()
                && task.signal.sequence.load(Ordering::Acquire) == 0
                && task.completed_sequence == self.assigned.ticket.sequence
                && task.returned_sequence == self.assigned.ticket.sequence)
        })();
        // SAFETY: exact saved flags after all task/queue borrows ended.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
}
