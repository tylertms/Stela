//! Authenticated IRQ observation, separate from counter and data-write authority.
use super::*;
/// A scoped borrowed observation of an actual installed entry event. It never
/// steals/duplicates the affine carrier needed by EOI and native finalization.
/// # Safety
/// verify independently checks the fresh sealed entry, actual CPU/root/clock
/// origin and captured interrupted RSP. Retained epochs/ranges/IDs are not
/// entry or data-write permission. The implementation cannot retain the loan.
pub unsafe trait InterruptedExecution {
    fn verify(
        &self,
        cpu: &Arc<dyn Cpu>,
        clock: &Arc<dyn service::Clock>,
    ) -> Result<usize, WorkError>;
}
/// Preallocated read view, containing no Runtime/Task pointer or Board borrow.
#[must_use = "keep the real native entry/counter/data owners until ingress joins"]
pub struct ExecutionTap {
    pub(super) view: ManuallyDrop<Arc<ExecutionView>>,
}
impl Clone for ExecutionTap {
    fn clone(&self) -> Self {
        Self {
            view: self.view.clone(),
        }
    }
}
impl ExecutionTap {
    /// Sample a genuine current frame. This counts no scheduler ticks and calls
    /// no C code; it only publishes nominal execution through an independently
    /// qualified source data lease. A refused entry/data root writes nothing.
    pub fn publish_irq(&self, entry: &dyn InterruptedExecution) -> Result<bool, WorkError> {
        let view = &self.view;
        view.cpu.check_cpu()?;
        if view.closed.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        let clock = view.clock()?;
        let rsp = entry.verify(&view.cpu, clock)?;
        let index = view.active.load(Ordering::Acquire);
        if index == NONE {
            return Ok(false);
        }
        let slot = view.slots.get(index).ok_or_else(|| WorkError::corrupt())?;
        if !slot.running.load(Ordering::Acquire) || !slot.bounds.contains(&rsp) {
            return Ok(false);
        }
        view.fence
            .readers
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_add(1))
            .map_err(|_| WorkError::Exhausted)?;
        let result = (|| {
            let now = clock.now_ns()?;
            let sample = view.sample_at(index, now)?;
            // SAFETY: scoped authentic same-CPU entry and retained immutable
            // slot; native process writers run at IF0 and cannot overlap IRQ.
            let target = unsafe { &*slot.target.get() };
            let Some(target) = target else {
                return Ok(false);
            };
            // SAFETY: source lease independently qualifies actual data mapping
            // for this entry; refusal must touch no C/source field.
            unsafe {
                target.publish_irq(entry, &sample)?;
            }
            Ok(true)
        })();
        view.fence.readers.fetch_sub(1, Ordering::AcqRel);
        result
    }
}
impl Drop for ExecutionTap {
    fn drop(&mut self) {
        if self.view.cpu.check_cpu().is_ok() && !self.view.cpu.in_interrupt() {
            // SAFETY: process-only release of this read-view handle; source
            // targets are separately ManuallyDrop-kept by their registration.
            unsafe {
                ManuallyDrop::drop(&mut self.view);
            }
        } else {
            quarantine();
        }
    }
}
