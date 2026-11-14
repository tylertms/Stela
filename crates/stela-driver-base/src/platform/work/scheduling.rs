//! Cooperative NORMAL/FIFO ordering for the optional kernel domain.
//! NORMAL nice uses Linux weights over measured running nanoseconds at
//! cooperative boundaries; no native preemption, CFS timeslice, RR quantum or
//! DEADLINE scheduling is inferred from an integer rank. Original Linux PI
//! commits use a captured job handle; default Runtime dispatch stays round-robin.
use super::{ContextId, LeaveReason, WakeHandle, WorkError};
use alloc::sync::Arc;
use core::sync::atomic::Ordering;

const NORMAL: u8 = 120;
const MIDDLE: u64 = 1 << 63;

/// Native scheduler state at one guarded snapshot, not a no-wake/death proof.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Activity {
    Dormant,
    Ready,
    Running,
    Blocked,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum BasePolicy {
    Normal,
    NormalNice(i8),
    Fifo(u8),
}
impl BasePolicy {
    pub fn priority(self) -> Result<u8, WorkError> {
        match self {
            Self::Normal => Ok(NORMAL),
            Self::NormalNice(nice @ -20..=19) => Ok((120i16 + i16::from(nice)) as u8),
            Self::NormalNice(_) => Err(WorkError::Unavailable),
            Self::Fifo(q @ 1..=99) => Ok(99 - q),
            Self::Fifo(_) => Err(WorkError::Unavailable),
        }
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Snapshot {
    pub base: BasePolicy,
    pub effective: u8,
}
impl Snapshot {
    const NORMAL: Self = Self {
        base: BasePolicy::Normal,
        effective: NORMAL,
    };
    fn validate(self) -> Result<(), WorkError> {
        let base = self.base.priority()?;
        if self.effective == 99 || self.effective > base {
            return Err(WorkError::Unavailable);
        }
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum State {
    Reserved { history: (u64, u128) },
    Dormant,
    Ready,
    Running,
    Blocked,
}
#[derive(Clone, Copy)]
struct Row {
    id: ContextId,
    sequence: u64,
    service: bool,
    rank: Snapshot,
    state: State,
    order: u64,
    virtual_ns: u128,
    started_ns: u64,
    execution_ns: u64,
    segment_started_ns: u64,
}
/// At most32 continuation records, embedded in the Queue.
/// Only its existing IRQ-save lock accesses this state; no task/C pointer,
/// provider callback, allocation or final ref release is invoked here. The
/// clock root is cloned under the lock and sampled only outside switch hooks.
pub(super) struct Board {
    pub(super) enabled: bool,
    pub(super) clock: Option<Arc<dyn super::service::Clock>>,
    fair_normal: bool,
    min_virtual_ns: u128,
    normal_history: [(u64, u128); super::MAX_CONTINUATIONS],
    rows: [Option<Row>; super::MAX_CONTINUATIONS],
    head: u64,
    tail: u64,
    reserved_tail: u64,
    running_row: Option<usize>,
}
impl Board {
    pub(super) fn new() -> Self {
        Self {
            enabled: false,
            clock: None,
            fair_normal: false,
            min_virtual_ns: 0,
            normal_history: [(0, 0); super::MAX_CONTINUATIONS],
            rows: [None; super::MAX_CONTINUATIONS],
            head: MIDDLE,
            tail: MIDDLE,
            reserved_tail: 0,
            running_row: None,
        }
    }
    fn normal_floor(&mut self) -> u128 {
        if let Some(floor) = self
            .rows
            .iter()
            .flatten()
            .filter(|row| {
                row.rank.effective >= 100 && matches!(row.state, State::Ready | State::Running)
            })
            .map(|row| row.virtual_ns)
            .min()
        {
            self.min_virtual_ns = self.min_virtual_ns.max(floor);
        }
        self.min_virtual_ns
    }
    pub(super) fn account(&mut self, now: u64) -> Result<(), WorkError> {
        // Original Linux sched_prio_to_weight for nice -20..19.
        const WEIGHTS: [u32; 40] = [
            88761, 71755, 56483, 46273, 36291, 29154, 23254, 18705, 14949, 11916, 9548, 7620, 6100,
            4904, 3906, 3121, 2501, 1991, 1586, 1277, 1024, 820, 655, 526, 423, 335, 272, 215, 172,
            137, 110, 87, 70, 56, 45, 36, 29, 23, 18, 15,
        ];
        if let Some(index) = self.running_row {
            let row = self.rows[index]
                .as_mut()
                .ok_or_else(|| WorkError::corrupt())?;
            if row.state != State::Running {
                return Err(WorkError::corrupt());
            }
            let elapsed = now
                .checked_sub(row.started_ns)
                .ok_or_else(|| WorkError::corrupt())?;
            let execution = row
                .execution_ns
                .checked_add(elapsed)
                .ok_or(WorkError::Exhausted)?;
            let virtual_ns = if row.rank.effective >= 100 {
                let base = row.rank.base.priority()?;
                let weight = WEIGHTS[usize::from(base - 100)];
                row.virtual_ns
                    .checked_add((u128::from(elapsed) << 32) * 1024 / u128::from(weight))
                    .ok_or(WorkError::Exhausted)?
            } else {
                row.virtual_ns
            };
            // All arithmetic checks precede this one actual running-row commit.
            row.execution_ns = execution;
            row.virtual_ns = virtual_ns;
            row.started_ns = now;
        }
        self.normal_floor();
        Ok(())
    }
    fn index(&self, id: ContextId, sequence: u64) -> Result<usize, WorkError> {
        self.rows
            .iter()
            .position(|row| row.is_some_and(|row| row.id == id && row.sequence == sequence))
            .ok_or(WorkError::Closed)
    }
    fn order(&mut self, head: bool) -> Result<u64, WorkError> {
        if head {
            let value = self.head.checked_sub(1).ok_or(WorkError::Exhausted)?;
            self.head = value;
            Ok(value)
        } else {
            let value = self.tail.checked_add(1).ok_or(WorkError::Exhausted)?;
            value
                .checked_add(self.reserved_tail)
                .ok_or(WorkError::Exhausted)?;
            self.tail = value;
            Ok(value)
        }
    }
    pub(super) fn insert(
        &mut self,
        id: ContextId,
        sequence: u64,
        service: bool,
    ) -> Result<(), WorkError> {
        self.insert_mode(id, sequence, service, None, false)
    }
    #[cfg(test)]
    pub(super) fn insert_dormant(
        &mut self,
        id: ContextId,
        sequence: u64,
        rank: Snapshot,
    ) -> Result<(), WorkError> {
        self.insert_mode(id, sequence, false, Some(rank), false)
    }
    pub(super) fn reserve_child(
        &mut self,
        id: ContextId,
        sequence: u64,
        rank: Snapshot,
    ) -> Result<(), WorkError> {
        self.insert_mode(id, sequence, false, Some(rank), true)
    }

    pub(super) fn reserved_child(
        &self,
        id: ContextId,
        sequence: u64,
        rank: Snapshot,
    ) -> Result<usize, WorkError> {
        let slot = self.index(id, sequence)?;
        let row = self.rows[slot].ok_or_else(|| WorkError::corrupt())?;
        if !matches!(row.state, State::Reserved { .. })
            || row.rank != rank
            || self.reserved_tail == 0
        {
            return Err(WorkError::corrupt());
        }
        Ok(slot)
    }

    pub(super) fn publish_reserved_child(&mut self, slot: usize) {
        let row = self.rows[slot].as_mut().unwrap();
        row.state = State::Dormant;
        self.fair_normal |= matches!(row.rank.base, BasePolicy::NormalNice(nice) if nice != 0);
    }

    pub(super) fn cancel_reserved_child(&mut self, slot: usize) {
        let row = self.rows[slot].take().unwrap();
        let State::Reserved { history } = row.state else {
            unreachable!("reserved child changed while locked");
        };
        let entry = self
            .normal_history
            .iter_mut()
            .find(|entry| entry.0 == row.id.get())
            .unwrap();
        *entry = history;
        self.reserved_tail -= 1;
    }

    fn insert_mode(
        &mut self,
        id: ContextId,
        sequence: u64,
        service: bool,
        dormant: Option<Snapshot>,
        reserve: bool,
    ) -> Result<(), WorkError> {
        if let Some(rank) = dormant
            && rank.effective != rank.base.priority()?
        {
            return Err(WorkError::Unavailable);
        }
        if !self.enabled {
            return if dormant.is_some() {
                Err(WorkError::Unavailable)
            } else {
                Ok(())
            };
        }
        if sequence == 0 || self.rows.iter().flatten().any(|row| row.id == id) {
            return Err(WorkError::corrupt());
        }
        let slot = self
            .rows
            .iter()
            .position(Option::is_none)
            .ok_or(WorkError::Full)?;
        let history = self
            .normal_history
            .iter()
            .position(|(context, _)| *context == id.get())
            .or_else(|| {
                self.normal_history
                    .iter()
                    .position(|(context, _)| *context == 0)
            })
            .ok_or(WorkError::Full)?;
        let previous_history = self.normal_history[history];
        let virtual_ns = previous_history.1.max(self.normal_floor());
        let (order, reserved_tail) = if dormant.is_some() {
            let reserved = self
                .reserved_tail
                .checked_add(1)
                .ok_or(WorkError::Exhausted)?;
            self.tail
                .checked_add(reserved)
                .ok_or(WorkError::Exhausted)?;
            (0, reserved)
        } else {
            (self.order(false)?, self.reserved_tail)
        };
        if !reserve {
            self.fair_normal |= matches!(dormant, Some(Snapshot { base: BasePolicy::NormalNice(nice), .. }) if nice != 0);
        }
        self.reserved_tail = reserved_tail;
        self.normal_history[history] = (id.get(), virtual_ns);
        self.rows[slot] = Some(Row {
            id,
            sequence,
            service,
            rank: dormant.unwrap_or(Snapshot::NORMAL),
            state: if reserve {
                State::Reserved {
                    history: previous_history,
                }
            } else if dormant.is_some() {
                State::Dormant
            } else {
                State::Ready
            },
            order,
            virtual_ns,
            started_ns: 0,
            execution_ns: 0,
            segment_started_ns: 0,
        });
        Ok(())
    }
    pub(super) fn activate(&mut self, id: ContextId, sequence: u64) -> Result<(), WorkError> {
        if !self.enabled {
            return Err(WorkError::Unavailable);
        }
        let slot = self.index(id, sequence)?;
        if self.rows[slot].as_ref().unwrap().state != State::Dormant || self.reserved_tail == 0 {
            return Err(WorkError::corrupt());
        }
        // A fresh ready order was reserved before C publication. Every other
        // tail stamp preserves this credit; activation consumes no capacity.
        let order = self
            .tail
            .checked_add(1)
            .ok_or_else(|| WorkError::corrupt())?;
        let remaining = self.reserved_tail - 1;
        order
            .checked_add(remaining)
            .ok_or_else(|| WorkError::corrupt())?;
        let floor = self.normal_floor();
        let row = self.rows[slot].as_mut().unwrap();
        row.virtual_ns = row.virtual_ns.max(floor);
        row.order = order;
        row.state = State::Ready;
        self.tail = order;
        self.reserved_tail = remaining;
        Ok(())
    }
    pub(super) fn wake(&mut self, id: ContextId, sequence: u64) -> Result<(), WorkError> {
        if !self.enabled {
            return Ok(());
        }
        let slot = self.index(id, sequence)?;
        if self.rows[slot].as_ref().unwrap().state == State::Blocked {
            let order = self.order(false)?;
            let floor = self.normal_floor();
            let row = self.rows[slot].as_mut().unwrap();
            row.virtual_ns = row.virtual_ns.max(floor);
            row.order = order;
            row.state = State::Ready;
        }
        Ok(())
    }
    pub(super) fn running(
        &mut self,
        id: ContextId,
        sequence: u64,
        now: u64,
    ) -> Result<(), WorkError> {
        if !self.enabled {
            return Ok(());
        }
        if self.running_row.is_some() {
            return Err(WorkError::corrupt());
        }
        let slot = self.index(id, sequence)?;
        let row = self.rows[slot].as_mut().unwrap();
        if row.state != State::Ready {
            return Err(WorkError::corrupt());
        }
        if now < row.started_ns {
            return Err(WorkError::corrupt());
        }
        row.state = State::Running;
        row.started_ns = now;
        row.segment_started_ns = now;
        self.running_row = Some(slot);
        Ok(())
    }
    pub(super) fn can_suspend(
        &self,
        id: ContextId,
        sequence: u64,
        reason: LeaveReason,
    ) -> Result<(), WorkError> {
        if !self.enabled {
            return Ok(());
        }
        let slot = self.index(id, sequence)?;
        if self.rows[slot].as_ref().unwrap().state != State::Running {
            return Err(WorkError::corrupt());
        }
        if reason == LeaveReason::Yield
            && self
                .tail
                .checked_add(1)
                .and_then(|next| next.checked_add(self.reserved_tail))
                .is_none()
        {
            return Err(WorkError::Exhausted);
        }
        Ok(())
    }
    pub(super) fn suspend(
        &mut self,
        id: ContextId,
        sequence: u64,
        reason: LeaveReason,
    ) -> Result<(), WorkError> {
        if !self.enabled {
            return Ok(());
        }
        let slot = self.index(id, sequence)?;
        if self.running_row != Some(slot)
            || self.rows[slot].as_ref().unwrap().state != State::Running
        {
            return Err(WorkError::corrupt());
        }
        let order = if reason == LeaveReason::Yield {
            Some(self.order(false)?)
        } else {
            None
        };
        let row = self.rows[slot].as_mut().unwrap();
        if let Some(order) = order {
            row.order = order;
        }
        row.state = if reason == LeaveReason::Blocked {
            State::Blocked
        } else {
            State::Ready
        };
        self.running_row = None;
        self.normal_floor();
        Ok(())
    }
    pub(super) fn remove(&mut self, id: ContextId, sequence: u64) -> Result<(), WorkError> {
        self.remove_mode(id, sequence, false)
    }
    pub(super) fn remove_terminal(
        &mut self,
        id: ContextId,
        sequence: u64,
    ) -> Result<(), WorkError> {
        self.remove_mode(id, sequence, true)
    }
    fn remove_mode(
        &mut self,
        id: ContextId,
        sequence: u64,
        terminal: bool,
    ) -> Result<(), WorkError> {
        if !self.enabled {
            return Ok(());
        }
        let slot = self.index(id, sequence)?;
        let row = self.rows[slot].unwrap();
        if self.running_row != Some(slot)
            || (!terminal && row.rank != Snapshot::NORMAL)
            || row.state != State::Running
        {
            return Err(WorkError::Busy);
        }
        // Physical continuations keep scheduler service debt across jobs, just
        // as their Linux task metadata survives an idle worker's completion.
        let history = self
            .normal_history
            .iter_mut()
            .find(|(context, _)| *context == id.get())
            .ok_or_else(|| WorkError::corrupt())?;
        history.1 = row.virtual_ns;
        self.rows[slot] = None;
        self.running_row = None;
        self.normal_floor();
        Ok(())
    }
    pub(super) fn activity(&self, id: ContextId, sequence: u64) -> Result<Activity, WorkError> {
        if !self.enabled {
            return Err(WorkError::Unavailable);
        }
        Ok(match self.rows[self.index(id, sequence)?].unwrap().state {
            State::Reserved { .. } => return Err(WorkError::Pending),
            State::Dormant => Activity::Dormant,
            State::Ready => Activity::Ready,
            State::Running => Activity::Running,
            State::Blocked => Activity::Blocked,
        })
    }
    pub(super) fn execution(
        &self,
        id: ContextId,
        sequence: u64,
        now: u64,
    ) -> Result<(u64, u64, u64), WorkError> {
        if !self.enabled {
            return Err(WorkError::Unavailable);
        }
        let index = self.index(id, sequence)?;
        let row = self.rows[index].as_ref().unwrap();
        if row.state == State::Running {
            if self.running_row != Some(index) {
                return Err(WorkError::corrupt());
            }
            let elapsed = now
                .checked_sub(row.started_ns)
                .ok_or_else(|| WorkError::corrupt())?;
            Ok((
                row.execution_ns
                    .checked_add(elapsed)
                    .ok_or(WorkError::Exhausted)?,
                now,
                row.segment_started_ns,
            ))
        } else {
            Ok((row.execution_ns, row.started_ns, row.segment_started_ns))
        }
    }
    pub(super) fn snapshot(&self, id: ContextId, sequence: u64) -> Result<Snapshot, WorkError> {
        if !self.enabled {
            return Err(WorkError::Unavailable);
        }
        Ok(self.rows[self.index(id, sequence)?].unwrap().rank)
    }
    pub(super) fn update(
        &mut self,
        id: ContextId,
        sequence: u64,
        before: Snapshot,
        after: Snapshot,
        head: bool,
        now: Option<u64>,
    ) -> Result<(), WorkError> {
        after.validate()?;
        let slot = self.index(id, sequence)?;
        let row = self.rows[slot].unwrap();
        if !self.enabled || row.rank != before || matches!(row.state, State::Reserved { .. }) {
            return Err(WorkError::InvalidTicket);
        }
        if row.service && after.base != BasePolicy::Normal {
            return Err(WorkError::Unavailable);
        }
        // This is the original RT->RT deboost placement, not explicit yield.
        if head && !(before.effective < after.effective && after.effective < 100) {
            return Err(WorkError::Unavailable);
        }
        if let Some(now) = now {
            self.account(now)?;
        }
        self.fair_normal |= matches!(after.base, BasePolicy::NormalNice(nice) if nice != 0);
        let changed = before.effective != after.effective;
        let order = if changed && matches!(row.state, State::Ready | State::Running) {
            Some(self.order(head)?)
        } else {
            None
        };
        let row = self.rows[slot].as_mut().unwrap();
        if let Some(order) = order {
            row.order = order;
        }
        row.rank = after;
        self.normal_floor();
        Ok(())
    }
    /// Both candidates are eligible Runtime tasks. Equal NORMAL keeps
    /// the caller's round-robin scan order; equal effective RT uses ready order.
    pub(super) fn precedes(
        &self,
        candidate: (ContextId, u64),
        selected: (ContextId, u64),
    ) -> Result<bool, WorkError> {
        let a = self.rows[self.index(candidate.0, candidate.1)?].unwrap();
        let b = self.rows[self.index(selected.0, selected.1)?].unwrap();
        if a.state != State::Ready || b.state != State::Ready {
            return Err(WorkError::corrupt());
        }
        if self.fair_normal && a.rank.effective >= 100 && b.rank.effective >= 100 {
            return Ok(a.virtual_ns < b.virtual_ns
                || (a.virtual_ns == b.virtual_ns && a.rank.effective < b.rank.effective));
        }
        Ok(a.rank.effective < b.rank.effective
            || (a.rank.effective == b.rank.effective
                && a.rank.effective < 100
                && a.order < b.order))
    }
    pub(super) fn quiet(&self) -> bool {
        self.rows.iter().all(Option::is_none)
    }
}

/// Immutable queue/context/job generation. It contains no Runtime or C
/// task pointer. A stale handle cannot mutate a new job on the reused stack.
#[derive(Clone)]
pub struct SchedulingHandle<const N: usize> {
    pub(super) wake: WakeHandle<N>,
}

impl<const N: usize> WakeHandle<N> {
    /// Capture the scheduling view of this retained job. Unlike renewing a
    /// wake handle, this may name another task on the same checked queue: PI
    /// updates the owner of a lock while its waiter is running.
    pub fn scheduling_handle(&self) -> Result<SchedulingHandle<N>, WorkError> {
        let handle = SchedulingHandle { wake: self.clone() };
        handle.snapshot()?;
        Ok(handle)
    }
}
impl<const N: usize> SchedulingHandle<N> {
    fn live(&self) -> Result<(), WorkError> {
        self.wake.queue.current_id()?; // caller CPU/RSP, same Queue
        if self.wake.closed()? {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    pub fn snapshot(&self) -> Result<Snapshot, WorkError> {
        self.live()?;
        self.wake
            .queue
            .lock()?
            .state()
            .scheduling
            .snapshot(self.wake.id, self.wake.sequence)
    }
    /// Observe the actual Board row, including a generic IRQ wake before the
    /// dispatcher's Task.state scan. Closed generation is never inactive/dead.
    pub fn activity(&self) -> Result<Activity, WorkError> {
        self.live()?;
        self.wake
            .queue
            .lock()?
            .state()
            .scheduling
            .activity(self.wake.id, self.wake.sequence)
    }
    /// Measured unweighted native execution; blocked time adds no elapsed.
    pub fn execution(&self) -> Result<super::ExecutionSample, WorkError> {
        self.live()?;
        let flags = self.wake.queue.cpu.save_disable()?;
        let result = (|| {
            let now = self.wake.queue.scheduling_now()?;
            let mut guard = self.wake.queue.lock()?;
            let (total, through, segment) =
                guard
                    .state()
                    .scheduling
                    .execution(self.wake.id, self.wake.sequence, now)?;
            Ok(super::ExecutionSample::new(
                super::ExecutionIdentity::new(
                    self.wake.queue.id,
                    self.wake.id.get(),
                    self.wake.sequence,
                ),
                total,
                through,
                segment,
            ))
        })();
        // SAFETY: exact flags after all native row/queue borrows ended.
        unsafe { self.wake.queue.cpu.restore(flags) };
        result
    }
    /// # Safety
    /// The same serialized original Linux task/PI-lock mutation supplies these
    /// exact old/new admitted policies and effective priorities. C commits its
    /// corresponding priority and borrowed donor before releasing that lease.
    /// Keep donor/task/waiter lifetimes; rank is not an ownership grant.
    pub unsafe fn update(
        &self,
        before: Snapshot,
        after: Snapshot,
        head: bool,
    ) -> Result<(), WorkError> {
        self.live()?;
        // Completion already sampled and charged in process context before
        // the IF0 binding hook. That hook may only normalize primitive ranks.
        let now = if self.wake.queue.completing.load(Ordering::Acquire) {
            None
        } else {
            Some(self.wake.queue.scheduling_now()?)
        };
        self.wake.queue.lock()?.state().scheduling.update(
            self.wake.id,
            self.wake.sequence,
            before,
            after,
            head,
            now,
        )?;
        self.wake.queue.inspection.store(true, Ordering::Release);
        Ok(())
    }
}

impl<const N: usize> super::Queue<N> {
    pub(super) fn scheduling_now(&self) -> Result<u64, WorkError> {
        let clock = self.lock()?.state().scheduling.clock.clone();
        clock.map_or(Ok(0), |clock| clock.now_ns())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn idle_normal_set_keeps_a_monotonic_virtual_service_floor() {
        let mut board = Board::new();
        board.enabled = true;
        board.insert(ContextId(1), 1, false).unwrap();
        board.running(ContextId(1), 1, 0).unwrap();
        board.account(1_000_000).unwrap();
        board
            .suspend(ContextId(1), 1, LeaveReason::Blocked)
            .unwrap();
        let floor = board.min_virtual_ns;
        assert!(floor > 0);
        board.insert(ContextId(2), 2, false).unwrap();
        let fresh = board.rows[board.index(ContextId(2), 2).unwrap()].unwrap();
        assert_eq!(fresh.virtual_ns, floor);
        board.wake(ContextId(1), 1).unwrap();
        assert_eq!(
            board.rows[board.index(ContextId(1), 1).unwrap()]
                .unwrap()
                .virtual_ns,
            floor
        );
    }
    #[test]
    fn completed_worker_keeps_cpu_debt_when_its_stack_is_recreated() {
        let mut board = Board::new();
        board.enabled = true;
        let high = ContextId(1);
        let ordinary = ContextId(2);
        board.insert(high, 1, false).unwrap();
        board.insert(ordinary, 2, false).unwrap();
        let nice = Snapshot {
            base: BasePolicy::NormalNice(-20),
            effective: 100,
        };
        board
            .update(high, 1, Snapshot::NORMAL, nice, false, Some(0))
            .unwrap();
        board.running(high, 1, 0).unwrap();
        board
            .update(high, 1, nice, Snapshot::NORMAL, false, Some(88_761))
            .unwrap();
        board.remove(high, 1).unwrap();
        board.insert(high, 3, false).unwrap();
        board
            .update(high, 3, Snapshot::NORMAL, nice, false, Some(88_761))
            .unwrap();
        assert!(board.precedes((ordinary, 2), (high, 3)).unwrap());
    }
    #[test]
    fn normal_nice_measures_elapsed_cpu_and_does_not_starve_ordinary_work() {
        let mut board = Board::new();
        board.enabled = true;
        let high = ContextId(1);
        let ordinary = ContextId(2);
        board.insert(high, 1, false).unwrap();
        board.insert(ordinary, 2, false).unwrap();
        let nice = Snapshot {
            base: BasePolicy::NormalNice(-20),
            effective: 100,
        };
        board
            .update(high, 1, Snapshot::NORMAL, nice, false, Some(0))
            .unwrap();
        assert!(board.precedes((high, 1), (ordinary, 2)).unwrap());
        // One long highpri run incurs the same weighted debt as many short
        // yields: weighting counts nanoseconds, not the number of boundaries.
        board.running(high, 1, 0).unwrap();
        board.account(88_761).unwrap();
        board.suspend(high, 1, LeaveReason::Yield).unwrap();
        assert!(board.precedes((ordinary, 2), (high, 1)).unwrap());
        board.running(ordinary, 2, 88_761).unwrap();
        board.account(89_785).unwrap();
        board.suspend(ordinary, 2, LeaveReason::Yield).unwrap();
        let a = board.rows[board.index(high, 1).unwrap()].unwrap();
        let b = board.rows[board.index(ordinary, 2).unwrap()].unwrap();
        assert_eq!(a.virtual_ns, b.virtual_ns);
        // FIFO donation still outranks either NORMAL nice base; deboost restores
        // the same accumulated NORMAL service rather than granting a free run.
        let boosted = Snapshot {
            base: nice.base,
            effective: 10,
        };
        board
            .update(high, 1, nice, boosted, false, Some(89_785))
            .unwrap();
        assert!(board.precedes((high, 1), (ordinary, 2)).unwrap());
        board
            .update(high, 1, boosted, nice, false, Some(89_785))
            .unwrap();
        assert_eq!(
            board.rows[board.index(high, 1).unwrap()]
                .unwrap()
                .virtual_ns,
            a.virtual_ns
        );
        assert_eq!(
            BasePolicy::NormalNice(-21).priority(),
            Err(WorkError::Unavailable)
        );
        assert_eq!(
            BasePolicy::NormalNice(20).priority(),
            Err(WorkError::Unavailable)
        );
    }
}

#[cfg(test)]
mod child_tests;

#[cfg(test)]
mod accounting_tests;
