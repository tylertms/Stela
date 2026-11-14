//! Preallocated execution shadows; IRQ readers never borrow the scheduling Board.
use super::*;
use core::sync::atomic::AtomicUsize;

pub(in crate::platform::work) struct Fence {
    pub(in crate::platform::work) targets: AtomicUsize,
    pub(in crate::platform::work) readers: AtomicUsize,
}
impl Fence {
    pub(in crate::platform::work) fn new() -> Self {
        Self {
            targets: AtomicUsize::new(0),
            readers: AtomicUsize::new(0),
        }
    }
    pub(in crate::platform::work) fn quiet(&self) -> bool {
        self.targets.load(Ordering::Acquire) == 0 && self.readers.load(Ordering::Acquire) == 0
    }
}
pub(in crate::platform::work) struct Slot {
    pub(in crate::platform::work) context: u64,
    pub(in crate::platform::work) bounds: Range<usize>,
    pub(in crate::platform::work) sequence: AtomicU64,
    pub(in crate::platform::work) total: AtomicU64,
    pub(in crate::platform::work) through: AtomicU64,
    pub(in crate::platform::work) segment: AtomicU64,
    pub(in crate::platform::work) running: AtomicBool,
    pub(in crate::platform::work) stopped: AtomicBool,
    pub(in crate::platform::work) target:
        UnsafeCell<Option<ManuallyDrop<Arc<dyn ExecutionPublication>>>>,
}
pub(in crate::platform::work) struct ExecutionView {
    pub(in crate::platform::work) cpu: Arc<dyn Cpu>,
    pub(in crate::platform::work) queue: u64,
    pub(in crate::platform::work) clock: ManuallyDrop<UnsafeCell<Option<Arc<dyn service::Clock>>>>,
    pub(in crate::platform::work) slots: ManuallyDrop<Vec<Slot>>,
    pub(in crate::platform::work) active: AtomicUsize,
    pub(in crate::platform::work) closed: AtomicBool,
    pub(in crate::platform::work) fence: Arc<Fence>,
}
// SAFETY: affine CPU and actual IRQ exclusion serialize writers. IRQ readers
// inspect only immutable geometry/atomics and preallocated kept data leases;
// no Runtime/Task pointer, Board borrow or final owner release crosses CPUs.
unsafe impl Send for ExecutionView {}
// SAFETY: as Send; production entry proof independently checks actual same CPU.
unsafe impl Sync for ExecutionView {}
impl ExecutionView {
    pub(in crate::platform::work) fn new<const N: usize>(
        queue: &Queue<N>,
        tasks: &[Task],
    ) -> Result<Arc<Self>, WorkError> {
        let mut slots = Vec::new();
        slots
            .try_reserve_exact(tasks.len())
            .map_err(|_| WorkError::Full)?;
        for task in tasks {
            slots.push(Slot {
                context: task.id.get(),
                bounds: task.stack.bounds.clone(),
                sequence: AtomicU64::new(0),
                total: AtomicU64::new(0),
                through: AtomicU64::new(0),
                segment: AtomicU64::new(0),
                running: AtomicBool::new(false),
                stopped: AtomicBool::new(false),
                target: UnsafeCell::new(None),
            });
        }
        Ok(Arc::new(Self {
            cpu: Arc::clone(&queue.cpu),
            queue: queue.id,
            clock: ManuallyDrop::new(UnsafeCell::new(None)),
            slots: ManuallyDrop::new(slots),
            active: AtomicUsize::new(NONE),
            closed: AtomicBool::new(false),
            fence: queue.execution_fence.clone(),
        }))
    }
    pub(in crate::platform::work) fn set_clock(
        &self,
        clock: Arc<dyn service::Clock>,
    ) -> Result<(), WorkError> {
        self.cpu.check_cpu()?;
        if self.cpu.in_interrupt() || !Arc::ptr_eq(clock.cpu(), &self.cpu) {
            return Err(WorkError::WrongCpu);
        }
        // SAFETY: caller owns first domain publication before any job/reader.
        let stored = unsafe { &mut *self.clock.get() };
        if stored.is_some() {
            return Err(WorkError::Busy);
        }
        *stored = Some(clock);
        Ok(())
    }
    pub(in crate::platform::work) fn clock(&self) -> Result<&Arc<dyn service::Clock>, WorkError> {
        // SAFETY: same CPU; the clock is installed once before jobs/IRQ capture
        // and stays ManuallyDrop-kept through all external tap handles.
        unsafe { &*self.clock.get() }
            .as_ref()
            .ok_or(WorkError::Unavailable)
    }
    pub(in crate::platform::work) fn open(
        &self,
        index: usize,
        sample: ExecutionSample,
    ) -> Result<(), WorkError> {
        let slot = self.slots.get(index).ok_or_else(|| WorkError::corrupt())?;
        if self.closed.load(Ordering::Acquire)
            || sample.identity.queue != self.queue
            || sample.identity.context != slot.context
            || sample.identity.sequence == 0
            || self.active.load(Ordering::Acquire) != NONE
        {
            return Err(WorkError::corrupt());
        }
        let previous = slot.sequence.load(Ordering::Acquire);
        if previous != sample.identity.sequence && self.has_target(index) {
            return Err(WorkError::Pending);
        }
        slot.sequence
            .store(sample.identity.sequence, Ordering::Relaxed);
        slot.total.store(sample.total_ns, Ordering::Relaxed);
        slot.through
            .store(sample.accounted_through_ns, Ordering::Relaxed);
        slot.segment
            .store(sample.segment_started_ns, Ordering::Relaxed);
        slot.stopped.store(false, Ordering::Relaxed);
        slot.running.store(true, Ordering::Release);
        self.active.store(index, Ordering::Release);
        Ok(())
    }
    pub(in crate::platform::work) fn identity(
        &self,
        index: usize,
    ) -> Result<ExecutionIdentity, WorkError> {
        let slot = self.slots.get(index).ok_or_else(|| WorkError::corrupt())?;
        let sequence = slot.sequence.load(Ordering::Acquire);
        if sequence == 0 {
            return Err(WorkError::Unavailable);
        }
        Ok(ExecutionIdentity::new(self.queue, slot.context, sequence))
    }
    pub(in crate::platform::work) fn sample_at(
        &self,
        index: usize,
        now: u64,
    ) -> Result<ExecutionSample, WorkError> {
        let slot = self.slots.get(index).ok_or_else(|| WorkError::corrupt())?;
        let identity = self.identity(index)?;
        let stored = slot.total.load(Ordering::Relaxed);
        let through = slot.through.load(Ordering::Relaxed);
        let (total, stamp) = if slot.running.load(Ordering::Acquire) {
            (
                stored
                    .checked_add(
                        now.checked_sub(through)
                            .ok_or_else(|| WorkError::corrupt())?,
                    )
                    .ok_or(WorkError::Exhausted)?,
                now,
            )
        } else {
            (stored, through)
        };
        Ok(ExecutionSample::new(
            identity,
            total,
            stamp,
            slot.segment.load(Ordering::Relaxed),
        ))
    }
    pub(in crate::platform::work) fn publish_process(
        &self,
        index: usize,
        sample: &ExecutionSample,
    ) {
        // SAFETY: actual CPU/IF0 and exact native slot; no scheduling Board
        // reference is live here. Arc/target storage stays owned by this slot.
        let target = unsafe { &*self.slots[index].target.get() };
        if let Some(target) = target {
            // SAFETY: successful attach initialized its qualified actual data
            // lease; source contract permits only infallible aligned RAM stores.
            unsafe { target.publish(sample) };
        }
    }
    pub(in crate::platform::work) fn pause(
        &self,
        index: usize,
        sample: &ExecutionSample,
    ) -> Result<(), WorkError> {
        if self.active.load(Ordering::Acquire) != index || self.identity(index)? != sample.identity
        {
            return Err(WorkError::corrupt());
        }
        let slot = &self.slots[index];
        slot.total.store(sample.total_ns, Ordering::Relaxed);
        slot.through
            .store(sample.accounted_through_ns, Ordering::Relaxed);
        slot.running.store(false, Ordering::Release);
        self.active.store(NONE, Ordering::Release);
        Ok(())
    }
    pub(in crate::platform::work) fn prepare_target(
        &self,
        index: usize,
        target: Arc<dyn ExecutionPublication>,
    ) -> Result<(), (Arc<dyn ExecutionPublication>, WorkError)> {
        if self.has_target(index) {
            return Err((target, WorkError::Pending));
        }
        if self
            .fence
            .targets
            .try_update(Ordering::AcqRel, Ordering::Acquire, |n| n.checked_add(1))
            .is_err()
        {
            return Err((target, WorkError::Exhausted));
        }
        // SAFETY: real process/IF0; slot has no old reader or target. No drop.
        unsafe {
            *self.slots[index].target.get() = Some(ManuallyDrop::new(target));
        }
        Ok(())
    }
    pub(in crate::platform::work) fn has_target(&self, index: usize) -> bool {
        // SAFETY: caller is actual CPU under native IF0, or an IRQ reader of
        // this exact immutable-in-entry slot. No mutable access overlaps.
        unsafe { (&*self.slots[index].target.get()).is_some() }
    }
    pub(in crate::platform::work) fn abort_target(
        &self,
        index: usize,
    ) -> Option<Arc<dyn ExecutionPublication>> {
        // SAFETY: source attach refused transactionally at IF0; no publication
        // or IRQ target was enabled, take moves owner for process-side drop.
        let target = unsafe { (&mut *self.slots[index].target.get()).take() };
        target.map(|target| {
            self.fence.targets.fetch_sub(1, Ordering::AcqRel);
            ManuallyDrop::into_inner(target)
        })
    }
    pub(in crate::platform::work) fn stop(
        &self,
        index: usize,
        sample: &ExecutionSample,
    ) -> Result<(), WorkError> {
        self.pause(index, sample)?;
        self.slots[index].stopped.store(true, Ordering::Release);
        Ok(())
    }
    pub(in crate::platform::work) fn take_stopped(
        &self,
        index: usize,
        identity: ExecutionIdentity,
    ) -> Result<StoppedPublication, WorkError> {
        let slot = self.slots.get(index).ok_or_else(|| WorkError::corrupt())?;
        if self.identity(index)? != identity
            || !slot.stopped.load(Ordering::Acquire)
            || slot.running.load(Ordering::Acquire)
            || self.active.load(Ordering::Acquire) == index
            || self.fence.readers.load(Ordering::Acquire) != 0
        {
            return Err(WorkError::Pending);
        }
        // SAFETY: actual physical-return caller proved stopped row and joined
        // IRQ readers at IF0; data/code owner moves without any final release.
        let target = unsafe { (&mut *slot.target.get()).take() }.ok_or(WorkError::Unavailable)?;
        self.fence.targets.fetch_sub(1, Ordering::AcqRel);
        Ok(StoppedPublication {
            cpu: self.cpu.clone(),
            target,
            identity,
        })
    }
}
impl Drop for ExecutionView {
    fn drop(&mut self) {
        self.closed.store(true, Ordering::Release);
        if self.fence.quiet() && self.cpu.check_cpu().is_ok() && !self.cpu.in_interrupt() {
            // SAFETY: no live target/reader remains; no source ref is released
            // implicitly, and only the retained generic clock/slot storage drops.
            unsafe {
                ManuallyDrop::drop(&mut self.clock);
                ManuallyDrop::drop(&mut self.slots);
            }
        } else {
            quarantine();
        }
    }
}
