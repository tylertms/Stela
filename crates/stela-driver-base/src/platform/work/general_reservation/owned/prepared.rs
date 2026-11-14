//! One owned preparation for first and genuinely retired native stack epochs.
use super::*;
mod pinned;
pub use pinned::{
    DormantGeneralStack, OwnedPinnedGeneral, PinnedGeneralStackPreparation, PublishedGeneralStack,
    StackPinError,
};
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum PreparationMode {
    Unstarted,
    Returned,
}
/// Owns one exclusive, zeroed usable mapping and its actual source epoch.
/// Drop/forget retain the whole Runtime and any physical view. No borrowed
/// entered permission, raw C descriptor or new current is stored here.
#[must_use = "publish/cancel under fresh authority; abandonment keeps the owner"]
pub struct GeneralStackPreparation<'runtime, const N: usize> {
    escrow: OwnedGeneralEscrow<'runtime, N>,
    pin: Option<ManuallyDrop<PinnedPages>>,
    epoch: Option<u64>,
    mode: Option<PreparationMode>,
    zeroed: bool,
    frame_ready: bool,
}
impl<const N: usize> core::fmt::Debug for GeneralStackPreparation<'_, N> {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("GeneralStackPreparation")
            .field("epoch", &self.epoch)
            .field("mode", &self.mode)
            .finish_non_exhaustive()
    }
}
impl<'runtime, const N: usize> OwnedGeneralEscrow<'runtime, N> {
    /// Retire only this reserved idle/unbound native continuation, then zero its
    /// usable mapping. Private entry history chooses the mode; it is never reset.
    /// No C task, stack descriptor, callback or current is published.
    ///
    /// # Safety
    /// All original old C/task/stack/page/descriptor/DMA/alias/RCU users really
    /// retired and their producers/held references joined. No outstanding raw
    /// view or foreign frame may use these bytes. A completed ticket, empty
    /// binding, refcount snapshot or epoch scalar alone cannot prove this.
    /// The old native idle trampoline is the only remaining frame; its physical
    /// return was recorded by the actual off-all-owned-stacks dispatcher.
    /// Keep original RAM/root/code/mapping owners and do not switch/reenter
    /// during this bounded initialization. This operation abandons that old
    /// internal trampoline, never a live C/Rust callback or affine destructor.
    #[expect(
        clippy::result_large_err,
        reason = "Failure returns exact original native exclusion"
    )]
    pub unsafe fn prepare_stack(
        self,
        entered: &dyn EnteredProcess,
    ) -> Result<
        GeneralStackPreparation<'runtime, N>,
        (GeneralStackPreparation<'runtime, N>, WorkError),
    > {
        // The owner exists BEFORE any external restore. Even a transferred,
        // not-yet-zeroed Preparing frame has an exact retaining error owner.
        let mut prepared = GeneralStackPreparation {
            escrow: self,
            pin: None,
            epoch: None,
            mode: None,
            zeroed: false,
            frame_ready: true,
        };
        let range = match prepared.escrow.begin_preparation(
            entered,
            &mut prepared.epoch,
            &mut prepared.mode,
            &mut prepared.frame_ready,
        ) {
            Ok(range) => range,
            Err(error) => return Err((prepared, error)),
        };
        if let Err(error) = prepared
            .check_preparation(entered)
            .and_then(|()| entered.check())
        {
            return Err((prepared, error));
        }
        // SAFETY: fresh actual scope AFTER every restore, exact native epoch
        // excludes old/current execution and all foreign users really retired.
        // No callback/switch/provider runs between this check and bounded write.
        unsafe { core::ptr::write_bytes(range.start as *mut u8, 0, range.len()) };
        prepared.zeroed = true;
        Ok(prepared)
    }
    fn begin_preparation(
        &self,
        entered: &dyn EnteredProcess,
        epoch_owner: &mut Option<u64>,
        mode_owner: &mut Option<PreparationMode>,
        frame_ready: &mut bool,
    ) -> Result<Range<usize>, WorkError> {
        self.current(entered)?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            self.current(entered)?;
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned || self.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            self.validate(state)?;
            // SAFETY: exact private reserved worker/nonce at IF0; no peer runs.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            if task.stack_epoch != task.stack_retired_epoch {
                return Err(WorkError::Pending);
            }
            let mode = if task.ever_entered {
                if task.completed_sequence == 0 || task.returned_sequence != task.completed_sequence
                {
                    return Err(WorkError::Pending);
                }
                PreparationMode::Returned
            } else {
                PreparationMode::Unstarted
            };
            let epoch = task
                .stack_epoch
                .checked_add(1)
                .ok_or(WorkError::Exhausted)?;
            let range = task.stack.bounds.clone();
            // Every failure precedes destructive frame ownership transfer.
            *epoch_owner = Some(epoch);
            *mode_owner = Some(mode);
            *frame_ready = false;
            task.stack_epoch = epoch;
            task.state = TaskState::Preparing;
            Ok(range)
        })();
        // SAFETY: exact outer flags after all locks and task borrows ended.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
    fn reset_prepared_frame(
        &self,
        entered: &dyn EnteredProcess,
        epoch: u64,
        expected: TaskState,
        retire: bool,
    ) -> Result<(), WorkError> {
        self.current(entered)?;
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            self.current(entered)?;
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if state.poisoned || self.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            self.validate_state(state, expected)?;
            // SAFETY: genuine owned preparation on stopped excluded backing.
            let task = unsafe { &mut *self.runtime.task(self.worker) };
            if task.stack_epoch != epoch
                || epoch == 0
                || (task.ever_entered
                    && (task.completed_sequence == 0
                        || task.returned_sequence != task.completed_sequence))
            {
                return Err(WorkError::corrupt());
            }
            let runtime = self.runtime.get_ref() as *const Runtime<N> as usize;
            // SAFETY: source preparation owns zeroed usable RAM, no old C or
            // callback frame/user survives, only Frame::start seeds controls.
            unsafe {
                task.frame
                    .start(task.stack.bounds.end, runtime, self.worker, entry::<N>)
            };
            if retire {
                task.stack_retired_epoch = epoch;
            }
            task.state = TaskState::Escrow;
            Ok(())
        })();
        // SAFETY: matching enclosing flags; all metadata borrows ended.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }
}
impl<const N: usize> GeneralStackPreparation<'_, N> {
    /// Observation only, never allocation/current/page-record authority.
    pub fn epoch(&self) -> Option<u64> {
        self.epoch
    }
    fn check_preparation(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
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
            owner.validate_state(
                state,
                if self.frame_ready {
                    TaskState::Escrow
                } else {
                    TaskState::Preparing
                },
            )?;
            // SAFETY: exact private epoch and immutable stopped backing history.
            let task = unsafe { &*owner.runtime.task(owner.worker) };
            match (self.epoch, self.mode) {
                (Some(epoch), Some(mode))
                    if epoch != 0
                        && task.stack_epoch == epoch
                        && task.ever_entered == (mode == PreparationMode::Returned) => {}
                (None, None) if self.frame_ready && !self.zeroed && self.pin.is_none() => {}
                _ => return Err(WorkError::corrupt()),
            }
            Ok(())
        })();
        // SAFETY: exact enclosing flags after no surviving metadata borrow.
        unsafe { owner.queue.cpu.restore(flags) };
        result
    }
    fn reseed(&mut self, entered: &dyn EnteredProcess, retire: bool) -> Result<(), WorkError> {
        self.check_preparation(entered)?;
        let epoch = self.epoch.ok_or(WorkError::Pending)?;
        if !retire && !self.zeroed {
            return Err(WorkError::Pending);
        }
        self.escrow.reset_prepared_frame(
            entered,
            epoch,
            if self.frame_ready {
                TaskState::Escrow
            } else {
                TaskState::Preparing
            },
            retire,
        )?;
        self.frame_ready = true;
        Ok(())
    }
    /// Remove only this original unpublished preparation after all external
    /// descriptor/page/alias users ended. Refusal returns the exact owner.
    ///
    /// # Safety
    /// Independently join all original C/page/descriptor/alias users of any
    /// inspected/exported pin. Never infer this from this source epoch.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal retains exact preparation/pin"
    )]
    pub unsafe fn cancel(mut self, entered: &dyn EnteredProcess) -> Result<(), (Self, WorkError)> {
        if let Err(error) = self.check_preparation(entered) {
            return Err((self, error));
        }
        if self.pin.as_ref().is_some_and(|pin| pin.has_other_views()) {
            return Err((self, WorkError::Pending));
        }
        if let Some(pin) = self.pin.take() {
            // The excluded native frame and queue remain owned across actual
            // final data-view Drop, including panic. No Idle/event yet.
            drop(ManuallyDrop::into_inner(pin));
        }
        if self.epoch.is_some()
            && let Err(error) = self.reseed(entered, true)
        {
            return Err((self, error));
        }
        let Self {
            escrow,
            pin,
            epoch,
            mode,
            zeroed,
            frame_ready,
        } = self;
        debug_assert!(pin.is_none());
        match escrow.cancel(entered) {
            Ok(()) => Ok(()),
            Err((escrow, error)) => Err((
                Self {
                    escrow,
                    pin,
                    epoch,
                    mode,
                    zeroed,
                    frame_ready,
                },
                error,
            )),
        }
    }
}
