//! Temporal inactivity of one retained assignment on the actual native UP CPU.
//! No source task pointer, C callback, timer or retirement grant lives here.
use super::*;

pub(super) struct PhysicalRun {
    assignment: AtomicU64,
    entered: AtomicU64,
    departed: AtomicU64,
}
impl PhysicalRun {
    pub(super) const fn new() -> Self {
        Self {
            assignment: AtomicU64::new(0),
            entered: AtomicU64::new(0),
            departed: AtomicU64::new(0),
        }
    }
    fn enter(&self, sequence: u64) -> Result<(), WorkError> {
        let old = self.entered.load(Ordering::Acquire);
        if sequence == 0 || old != self.departed.load(Ordering::Acquire) {
            return Err(WorkError::corrupt());
        }
        let next = old.checked_add(1).ok_or(WorkError::Exhausted)?;
        self.assignment.store(sequence, Ordering::Relaxed);
        self.entered.store(next, Ordering::Release);
        Ok(())
    }
    fn depart(&self) -> Result<u64, WorkError> {
        let run = self.entered.load(Ordering::Acquire);
        let sequence = self.assignment.load(Ordering::Acquire);
        if run == 0 || sequence == 0 || self.departed.load(Ordering::Acquire) >= run {
            return Err(WorkError::corrupt());
        }
        self.departed.store(run, Ordering::Release);
        Ok(sequence)
    }
    fn returned(&self, sequence: u64) -> bool {
        let run = self.entered.load(Ordering::Acquire);
        run != 0
            && self.assignment.load(Ordering::Acquire) == sequence
            && self.departed.load(Ordering::Acquire) == run
    }
}

/// Only a temporal native observation; source state and original task lifetime
/// require their independent original pi_lock/usage owner. It never retires a task.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum InactiveObservation {
    Inactive,
    Subscribed,
}

trait QueueLifetime: Send + Sync {
    fn check_process(&self) -> Result<(), WorkError>;
    fn address(&self) -> *const ();
}
impl<const N: usize> QueueLifetime for Queue<N> {
    fn check_process(&self) -> Result<(), WorkError> {
        self.process()
    }
    fn address(&self) -> *const () {
        core::ptr::from_ref(self).cast()
    }
}

/// Concrete queue-type erasure retains the SAME actual queue/signal, without
/// casting a foreign binding to the caller's const-generic Runtime type.
/// Only a live WakeHandle can construct it; no pointer/number constructor.
pub struct InactiveTarget {
    queue: ManuallyDrop<Arc<dyn QueueLifetime>>,
    signal: ManuallyDrop<Arc<WakeSignal>>,
    id: ContextId,
    sequence: u64,
}
impl InactiveTarget {
    pub fn check_process(&self) -> Result<(), WorkError> {
        self.queue.check_process()
    }
}
impl Drop for InactiveTarget {
    fn drop(&mut self) {
        if self.check_process().is_ok() {
            // SAFETY: actual retained queue's process CPU; no native hook owns this view.
            unsafe {
                ManuallyDrop::drop(&mut self.signal);
                ManuallyDrop::drop(&mut self.queue);
            }
        } else {
            quarantine();
        }
    }
}
impl<const N: usize> WakeHandle<N> {
    pub fn retain_inactive(&self) -> Result<InactiveTarget, WorkError> {
        if self.closed()? {
            return Err(WorkError::Closed);
        }
        let queue: Arc<Queue<N>> = Arc::clone(&self.queue);
        let queue: Arc<dyn QueueLifetime> = queue;
        Ok(InactiveTarget {
            queue: ManuallyDrop::new(queue),
            signal: ManuallyDrop::new(Arc::clone(&self.signal)),
            id: self.id,
            sequence: self.sequence,
        })
    }
}

/// Queue -> signal only. No Queue/Runtime/C/task/resource-owner backedge.
pub(super) struct Subscriber {
    target: ContextId,
    target_sequence: u64,
    waiter: ContextId,
    waiter_sequence: u64,
    signal: Arc<WakeSignal>,
}

/// Wake/departure notification only; no source state is read or changed here.
pub(super) fn notify<const N: usize>(
    state: &mut State<N>,
    id: ContextId,
    sequence: u64,
) -> Result<bool, WorkError> {
    let State {
        inactive,
        scheduling,
        ..
    } = state;
    let mut notified = false;
    for row in inactive.iter().flatten() {
        if row.target != id || row.target_sequence != sequence {
            continue;
        }
        if row.signal.sequence.load(Ordering::Acquire) != row.waiter_sequence {
            return Err(WorkError::corrupt());
        }
        scheduling.wake(row.waiter, row.waiter_sequence)?;
        row.signal.pending.store(true, Ordering::Release);
        notified = true;
    }
    Ok(notified)
}

impl<const N: usize> Runtime<N> {
    /// IF0 immediately before the one actual dispatcher-to-worker switch.
    pub(super) fn physical_enter(self: Pin<&Self>, index: usize) -> Result<(), WorkError> {
        // SAFETY: dispatch owns this index at IF0; this bounded leaf never switches.
        let task = unsafe { &*self.task(index) };
        let sequence = task.job.ok_or_else(|| WorkError::corrupt())?.1.sequence;
        if task.state != TaskState::Running
            || self.current.get() != index
            || task.signal.sequence.load(Ordering::Acquire) != sequence
        {
            return Err(WorkError::corrupt());
        }
        task.signal.physical.enter(sequence)
    }

    /// Called by returned_stack after it checks the actual dispatcher successor.
    /// No C/provider callback, allocation, switch or final ref release occurs.
    pub(super) fn physical_departure(
        self: Pin<&Self>,
        id: ContextId,
        signal: &WakeSignal,
    ) -> Result<(), WorkError> {
        let sequence = signal.physical.depart()?;
        let mut guard = self.queue.lock()?;
        if notify(guard.state(), id, sequence)? {
            self.queue.inspection.store(true, Ordering::Release);
        }
        Ok(())
    }

    /// Observe one other retained assignment and, if needed, arm its physical
    /// departure event before source code parks. The caller separately holds
    /// the actual source pi_lock/IF exclusion through its original state/cookie
    /// snapshot. `force_wait` only vetoes success (saved-state/source-queued).
    /// Neither this method nor its return promises future absence of wakes.
    pub fn observe_inactive(
        self: Pin<&Self>,
        target: &InactiveTarget,
        waiter: &WakeHandle<N>,
        force_wait: bool,
    ) -> Result<InactiveObservation, WorkError> {
        self.queue.process()?;
        if Arc::as_ptr(&self.queue).cast::<()>() != target.queue.address()
            || !Arc::ptr_eq(&self.queue, &waiter.queue)
            || target.id == waiter.id
        {
            return Err(WorkError::Unavailable);
        }
        if self.current_id()? != waiter.id {
            return Err(WorkError::Unavailable);
        }
        // Retain before IF0/queue lock; clone is nonallocating and cannot be final.
        let signal = Arc::clone(&waiter.signal);
        let result = (|| {
            let mut guard = self.queue.lock()?;
            if self.queue.active.id.load(Ordering::Acquire) != waiter.id.get()
                || waiter.signal.sequence.load(Ordering::Acquire) != waiter.sequence
                || target.signal.sequence.load(Ordering::Acquire) != target.sequence
            {
                return Err(WorkError::Closed);
            }
            let state = guard.state();
            if state
                .inactive
                .iter()
                .flatten()
                .any(|row| row.waiter == waiter.id)
            {
                return Err(WorkError::Busy);
            }
            let activity = state.scheduling.activity(target.id, target.sequence)?;
            if !force_wait
                && activity == scheduling::Activity::Blocked
                && !target.signal.pending.load(Ordering::Acquire)
                && target.signal.physical.returned(target.sequence)
            {
                return Ok(InactiveObservation::Inactive);
            }
            let slot = state
                .inactive
                .iter_mut()
                .find(|row| row.is_none())
                .ok_or(WorkError::Full)?;
            *slot = Some(Subscriber {
                target: target.id,
                target_sequence: target.sequence,
                waiter: waiter.id,
                waiter_sequence: waiter.sequence,
                signal: Arc::clone(&signal),
            });
            Ok(InactiveObservation::Subscribed)
        })();
        // Guard is gone. Current process/actual caller retains both signal roots.
        drop(signal);
        result
    }

    /// Remove only the actual current waiter's registration. Its retained
    /// signal leaves the queue lock before release. Never a target retirement.
    pub fn cancel_inactive(self: Pin<&Self>, waiter: &WakeHandle<N>) -> Result<(), WorkError> {
        self.queue.process()?;
        if !Arc::ptr_eq(&self.queue, &waiter.queue) || self.current_id()? != waiter.id {
            return Err(WorkError::Unavailable);
        }
        let removed = {
            let mut guard = self.queue.lock()?;
            let state = guard.state();
            if waiter.signal.sequence.load(Ordering::Acquire) != waiter.sequence {
                return Err(WorkError::Closed);
            }
            let slot = state.inactive.iter_mut().find(|slot| {
                slot.as_ref().is_some_and(|row| {
                    row.waiter == waiter.id && row.waiter_sequence == waiter.sequence
                })
            });
            slot.and_then(Option::take)
        };
        drop(removed);
        Ok(())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn run_generation_refuses_old_departure_reuse_and_overflow() {
        // Pure transition controls do not construct a native current/root/witness.
        let physical = PhysicalRun::new();
        assert!(!physical.returned(7));
        physical.enter(7).unwrap();
        assert!(!physical.returned(7));
        assert_eq!(physical.enter(7), Err(WorkError::CorruptContext));
        assert_eq!(physical.depart(), Ok(7));
        assert!(physical.returned(7));
        physical.enter(7).unwrap();
        assert!(!physical.returned(7));
        assert_eq!(physical.depart(), Ok(7));
        physical.enter(8).unwrap();
        assert!(!physical.returned(7));
        assert!(!physical.returned(8));
        assert_eq!(physical.depart(), Ok(8));
        assert!(physical.returned(8));
        physical.entered.store(u64::MAX, Ordering::Release);
        physical.departed.store(u64::MAX, Ordering::Release);
        assert_eq!(physical.enter(9), Err(WorkError::Exhausted));
        assert_eq!(physical.assignment.load(Ordering::Acquire), 8);
    }
}
