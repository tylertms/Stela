//! Opaque real backing-pin retention, with no address or C descriptor access.
use super::*;
use crate::bus::dma::memory::CacheAttribute;
mod publication;
pub use publication::{DormantGeneralStack, OwnedPinnedGeneral, PublishedGeneralStack};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum StackPinError {
    Work(WorkError),
    Memory(MemoryError),
}
impl core::fmt::Display for StackPinError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "prepared stack backing pin: {self:?}")
    }
}
impl core::error::Error for StackPinError {}

/// Owns the original excluded native preparation and mapping/allocation
/// pin. It offers no page/address/C descriptor getter and grants no current,
/// task or stack loan. No borrowed entered permission survives the operation.
/// Drop/forget keeps the pin and original runtime ownership; cancellation
/// releases only after fresh matching permission restores the real frame.
#[must_use = "cancel under fresh matching authority; Drop retains and closes the owner"]
pub struct PinnedGeneralStackPreparation<'runtime, const N: usize> {
    prepared: GeneralStackPreparation<'runtime, N>,
}
impl<const N: usize> PinnedGeneralStackPreparation<'_, N> {
    // Catalog transport for the Linux kernel services: borrowed metadata is consumed
    // immediately while this owning value keeps the pin and excluded frame.
    #[cfg(feature = "linux-kernel")]
    pub fn inspect_pin<T>(
        &self,
        entered: &dyn EnteredProcess,
        inspect: impl FnOnce(&PinnedPages) -> T,
    ) -> Result<T, WorkError> {
        self.check_pin(entered)?;
        Ok(inspect(
            self.prepared
                .pin
                .as_ref()
                .ok_or_else(|| WorkError::corrupt())?,
        ))
    }
    #[cfg(feature = "linux-kernel")]
    pub fn check_pin(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        self.prepared.pin_source(entered).map(|_| ())
    }
    /// Bounded exact native cleanup identity, including pin=None after actual
    /// data Drop. This grants no byte/pin/current or retirement permission.
    #[cfg(feature = "linux-kernel")]
    pub fn check_cancel(&self, entered: &dyn EnteredProcess) -> Result<(), WorkError> {
        self.prepared.check_preparation(entered)
    }
    /// # Safety
    /// All original unpublished C/page/descriptor/alias users really retired.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal retains exact prepared pin"
    )]
    pub unsafe fn cancel(self, entered: &dyn EnteredProcess) -> Result<(), (Self, WorkError)> {
        // SAFETY: caller separately joins original users, not inferred from epoch.
        unsafe { self.prepared.cancel(entered) }
            .map_err(|(prepared, error)| (Self { prepared }, error))
    }
    /// Actual native preparation epoch; no page/current/task permission follows.
    pub fn epoch(&self) -> u64 {
        self.prepared.epoch.unwrap()
    }
}
impl<'runtime, const N: usize> GeneralStackPreparation<'runtime, N> {
    fn pin_source<'a>(
        &'a self,
        entered: &dyn EnteredProcess,
    ) -> Result<(&'a (dyn StackBacking + 'static), Range<usize>), WorkError> {
        self.check_preparation(entered)?;
        if !self.zeroed || self.frame_ready {
            return Err(WorkError::Pending);
        }
        let owner = &self.escrow;
        owner.current(entered)?;
        let flags = owner.queue.cpu.save_disable()?;
        let result = (|| {
            owner.current(entered)?;
            let mut guard = owner.queue.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned || owner.queue.abandoned.load(Ordering::Acquire) {
                return Err(WorkError::Closed);
            }
            owner.validate_state(state, TaskState::Preparing)?;
            // SAFETY: exact creator/nonce/state checked at IF0. This immutable
            // boxed backing stays held by the owned Runtime lifetime;
            // Preparing excludes entry/reuse and retirement is count-blocked.
            let task = unsafe { &*owner.runtime.task(owner.worker) };
            if Some(task.stack_epoch) != self.epoch
                || task.stack_pin_nonce != 0
                || (task.ever_entered && task.returned_sequence != task.completed_sequence)
            {
                return Err(WorkError::corrupt());
            }
            Ok((&*task.stack.backing, task.stack.bounds.clone()))
        })();
        // SAFETY: enclosing flags; no queue/task borrow or lock survives.
        // Exporter metadata work happens only after this restoration.
        unsafe { owner.queue.cpu.restore(flags) };
        result
    }

    /// Keep usable-page ownership without exposing bytes or a C stack.
    /// Every refusal returns this same original prepared owner. A successful but
    /// refused exported pin remains held for fresh cancellation; retry cannot
    /// accumulate another pin. Stale/drop/forget never releases it implicitly.
    #[expect(
        clippy::result_large_err,
        reason = "Refusal returns the exact pinned runtime/prepared owner and any acquired physical pin without allocating or losing ownership"
    )]
    pub fn pin(
        mut self,
        entered: &dyn EnteredProcess,
    ) -> Result<PinnedGeneralStackPreparation<'runtime, N>, (Self, StackPinError)> {
        let (source, range) = match self.pin_source(entered) {
            Ok((source, range)) => (source as *const dyn StackBacking, range),
            Err(error) => return Err((self, StackPinError::Work(error))),
        };
        if self.pin.is_some() {
            return Err((self, StackPinError::Work(WorkError::Busy)));
        }
        // SAFETY: actual exclusive native epoch and zeroing were
        // established by the original prepared frame owner. The boxed source is
        // kept, and the exporter cannot park/reenter/change this context.
        let exported = unsafe { (&*source).pin_prepared_pages(self.epoch.unwrap()) };
        let validation = match exported {
            Ok(pin) => {
                self.pin = Some(ManuallyDrop::new(pin));
                Self::validate_pin(self.pin.as_ref().unwrap(), &range)
            }
            Err(error) => Err(error),
        };
        // Always recheck scope/creator/nonce after exporter and all pin
        // metadata queries, even on failure. No label or caller epoch suffices.
        let post = self.pin_source(entered).and_then(|(after, bounds)| {
            if !core::ptr::eq(source, after as *const dyn StackBacking) || bounds != range {
                Err(WorkError::corrupt())
            } else {
                Ok(())
            }
        });
        if let Err(error) = post {
            return Err((self, StackPinError::Work(error)));
        }
        if let Err(error) = validation {
            return Err((self, StackPinError::Memory(error)));
        }
        Ok(PinnedGeneralStackPreparation { prepared: self })
    }

    fn validate_pin(pin: &PinnedPages, range: &Range<usize>) -> Result<(), MemoryError> {
        if pin.page_shift() != 12 || pin.physical_pages().len() != range.len() / 4096 {
            return Err(MemoryError::InvalidPageList);
        }
        if pin.bytes() != range.len() || pin.logical_bytes() != range.len() {
            return Err(MemoryError::InvalidSize);
        }
        if pin.cache_attribute() != CacheAttribute::WriteBack {
            return Err(MemoryError::CacheAttributeUnavailable);
        }
        let cpu = pin
            .cpu_mapping(CacheAttribute::WriteBack)?
            .address(0, range.len())?;
        if cpu.as_ptr() as usize != range.start {
            return Err(MemoryError::OutsideReservation);
        }
        Ok(())
    }
}
