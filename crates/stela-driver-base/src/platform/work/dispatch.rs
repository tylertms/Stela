//! Cooperative dispatch: yields, suspension, one continuation step and
//! foreground calls.

use super::*;

impl<const N: usize> Runtime<N> {
    /// Suspend the current continuation, returning only when resumed on
    /// this same stack and ContextId. It never calls a queued callback
    /// recursively on the lock-owning caller stack.
    pub fn yield_now(self: Pin<&Self>) -> Result<(), WorkError> {
        self.suspend(LeaveReason::Yield).map(|_| ())
    }

    /// Involuntary cooperative boundary, distinct from explicit yield-to-tail.
    /// Default round-robin stays unchanged; future FIFO ordering consumes this
    /// distinct reason instead of treating it as an explicit yield.
    pub fn reschedule(self: Pin<&Self>) -> Result<(), WorkError> {
        self.suspend(LeaveReason::Reschedule).map(|_| ())
    }

    /// Process/current check precedes kept clock/provider entry.
    pub fn reschedule_requested(self: Pin<&Self>) -> Result<bool, WorkError> {
        self.current_id()?;
        let Some(domain) = self.domain() else {
            return Ok(self.step_bound_reached());
        };
        if self.service_current()? {
            return Ok(false);
        }
        // Capture before inspection. A later wake must survive this query.
        let requested = self.queue.inspection.swap(false, Ordering::AcqRel);
        Ok(domain.inspect_and_wake()? || requested || self.step_bound_reached())
    }

    /// Bound the steps of this dispatcher's caller: while `leave` is set and
    /// returns true, each reschedule point of a running continuation (where
    /// [`Self::reschedule_requested`] is asked) suspends it, so the caller's
    /// [`Self::step`] returns there instead of when the continuation parks.
    /// `None` removes the bound. The caller sets it on the dispatcher, around
    /// its own steps; a continuation never sees a bound of another caller.
    pub fn bound_steps(self: Pin<&Self>, leave: Option<fn() -> bool>) {
        self.step_bound.set(leave);
    }

    fn step_bound_reached(&self) -> bool {
        self.step_bound.get().is_some_and(|leave| leave())
    }

    /// Suspend once only if the foreign predicate is still blocked.
    /// Linux-owned process PREPARE must precede this call. Skipped/error leaves
    /// its original entered lease intact; its wrapper clears prepared state.
    pub fn park_if_blocked(self: Pin<&Self>) -> Result<ParkOutcome, WorkError> {
        self.suspend(LeaveReason::Blocked)
    }

    pub(super) fn suspend(self: Pin<&Self>, reason: LeaveReason) -> Result<ParkOutcome, WorkError> {
        self.current_id()?;
        if self.pending_reservations.get() != 0 {
            return Err(WorkError::Busy);
        }
        let index = self.current.get();
        let flags = self.queue.cpu.save_disable()?;
        let now = match self.queue.scheduling_now() {
            Ok(now) => now,
            Err(error) => {
                // SAFETY: clock refused before any departure or row mutation.
                unsafe { self.queue.cpu.restore(flags) };
                return Err(error);
            }
        };
        let view = self.context_view(index, true);
        let admission = {
            // SAFETY: checked current task and IF0; borrow ends before switch.
            let binding = unsafe { &(*self.task(index)).binding };
            (|| {
                if reason == LeaveReason::Blocked {
                    let binding = binding.as_ref().ok_or(WorkError::Unavailable)?;
                    // SAFETY: IF0 pure predicate; process PREPARE is provider-owned.
                    let still_blocked = unsafe { binding.still_blocked(&view)? };
                    // SAFETY: this field is immutable at IF0; the short
                    // borrow ends here before foreign leave or a context switch.
                    let signal = unsafe { &(*self.task(index)).signal };
                    let pending = signal.pending.swap(false, Ordering::AcqRel);
                    if !still_blocked || pending {
                        return Ok(false);
                    }
                }
                // Pure bounded preflight before foreign leave commits. Native
                // IRQ delivery is excluded; exhaustion after a hook's explicit
                // generic peer wake remains a terminal transition failure.
                {
                    let mut guard = self.queue.lock()?;
                    guard.state().scheduling.can_suspend(
                        ContextId(view.context_id),
                        view.sequence,
                        reason,
                    )?;
                    // Charge before a foreign leave hook can wake a sleeper;
                    // its placement must observe the current CPU service floor.
                    guard.state().scheduling.account(now)?;
                }
                self.execution_publish_at(index, now)?;
                if let Some(binding) = binding {
                    // SAFETY: transactional prepared commit only, no lock/alloc/drop/switch.
                    unsafe { binding.leave(&view, reason)? };
                }
                Ok(true)
            })()
        };
        match admission {
            Ok(true) => {}
            other => {
                // SAFETY: no departure committed; original task/current/lease intact.
                unsafe { self.queue.cpu.restore(flags) };
                return other.map(|_| ParkOutcome::Skipped);
            }
        }
        // Foreign departure committed. A failed sidecar commit is terminal;
        // it cannot return through a now-suspended foreign owner/lease.
        let transition = (|| {
            self.queue.lock()?.state().scheduling.suspend(
                ContextId(view.context_id),
                view.sequence,
                reason,
            )
        })();
        if let Err(error) = transition {
            self.queue.cpu.fatal(error);
        }
        if let Err(error) = self.execution_pause_at(index, now) {
            self.queue.cpu.fatal(error);
        }
        // SAFETY: running task is exclusively owned, with IF disabled.
        unsafe {
            (*self.task(index)).state = if reason == LeaveReason::Blocked {
                TaskState::Blocked
            } else {
                TaskState::Runnable
            };
        }
        self.current.set(NONE);
        self.queue.active.clear();
        // SAFETY: initialized frames; no Task reference or queue lock spans switch.
        unsafe {
            context::switch(
                core::ptr::addr_of_mut!((*self.task(index)).frame),
                self.dispatch.get(),
            )
        };
        self.arrive(index, false);
        // SAFETY: restored foreign and native metadata match this real resumed stack.
        unsafe { self.queue.cpu.restore(flags) };
        Ok(ParkOutcome::Resumed)
    }

    /// # Safety
    /// Checked taskless dispatcher at IF0; all kept task frames are stopped.
    pub(super) unsafe fn next_firmware(&self, count: usize) -> Option<usize> {
        for offset in 0..count {
            let index = (self.firmware_cursor.get() + offset) % count;
            // SAFETY: bounded immutable task vector, stopped under dispatcher IF0.
            let task = unsafe { &*self.task(index) };
            if self.layout.role(task.lane).is_none()
                || !matches!(task.state, TaskState::Ready | TaskState::Runnable)
            {
                continue;
            }
            if task.state == TaskState::Ready && matches!(task.lane, Lane::Notify | Lane::Portal) {
                let sequence = match task.job {
                    Some((_, job)) => job.sequence,
                    None => self.queue.cpu.fatal(WorkError::CorruptContext),
                };
                let earlier = (0..count).any(|peer| {
                    // SAFETY: same stopped task vector; this read cannot switch or invoke work.
                    let other = unsafe { &*self.task(peer) };
                    other.lane == task.lane
                        && other.state == TaskState::Ready
                        && other.job.is_some_and(|(_, job)| job.sequence < sequence)
                });
                if earlier {
                    continue;
                }
            }
            return Some(index);
        }
        None
    }

    /// Run at most one task continuation. No callback is inline here:
    /// RIP/RSP switch onto its distinct owned stack precedes callback.
    /// Caller is the main dispatcher outside locks/IRQ/native guest execution.
    /// Callback contexts must be provider-owned stable objects, not escaped
    /// references into this dispatcher stack. Queue ownership covers them.
    pub fn step(self: Pin<&Self>) -> Result<bool, WorkError> {
        let entry = step_counts::now();
        self.queue.process()?;
        if self.current.get() != NONE {
            return Err(WorkError::WouldDeadlock);
        }
        if self.retired.get() || self.queue.abandoned.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        // Deliver only recorded real credit releases, in the actual taskless
        // dispatcher before deadline inspection. Empty passes invoke no sink.
        self.publish_general_resources()?;
        let due_service = if let Some(domain) = self.domain() {
            if !self.service_started.get() {
                return Err(WorkError::Unavailable);
            }
            match domain.inspect_and_wake() {
                Ok(due) => due,
                Err(error) => self.queue.cpu.fatal(error),
            }
        } else {
            false
        };
        let flags = self.queue.cpu.save_disable()?;
        // One lock spans the whole scan: the dispatcher holds IF0 on the
        // queue's CPU, so no producer can change the slots between tasks,
        // and taking it per task only repeated the CPU checks.
        let mut guard = match self.queue.lock() {
            Ok(guard) => guard,
            Err(error) => {
                // SAFETY: `flags` came from the save_disable above, and no lock is held.
                unsafe { self.queue.cpu.restore(flags) };
                return Err(error);
            }
        };
        let mut waiting = guard.state().waiting_lanes();
        // SAFETY: the task list's length never changes after construction.
        let count = unsafe { (*self.tasks.get()).len() };
        let mut selected = None;
        let has_firmware = self.layout.role(Lane::Firmware).is_some();
        for n in 0..count {
            let i = (if due_service { 2 } else { self.cursor.get() } + n) % count;
            // SAFETY: no task runs while the dispatcher steps, so this is the only
            // reference to task `i`.
            let task = unsafe { &mut (*self.task(i)) };
            if task.state == TaskState::Idle {
                match guard.state().select_waiting_job(
                    &mut waiting,
                    task.lane,
                    stack_requirement::Geometry::of(&task.stack),
                    self.queue.abandoned.load(Ordering::Acquire),
                ) {
                    Ok(Some(job)) => {
                        let inserted = guard.state().scheduling.insert(
                            task.id,
                            job.1.sequence,
                            task.lane == Lane::Service,
                        );
                        if let Err(error) = inserted {
                            self.queue.cpu.fatal(error);
                        }
                        task.assign(job, flags);
                    }
                    Ok(None) => {}
                    Err(e) => {
                        drop(guard);
                        // SAFETY: `flags` came from the save_disable above, and
                        // the queue lock was released just above.
                        unsafe { self.queue.cpu.restore(flags) };
                        return Err(e);
                    }
                }
            }
            if task.state == TaskState::Blocked && task.signal.pending.swap(false, Ordering::AcqRel)
            {
                // A valid generic wake can be posted by a leave hook after
                // the final pending check but before block commit.
                // Earlier IRQ publication already moved the row; duplicate
                // wake preserves its original ready order.
                let ready = task
                    .job
                    .ok_or(WorkError::CorruptContext)
                    .and_then(|job| guard.state().scheduling.wake(task.id, job.1.sequence));
                if let Err(error) = ready {
                    self.queue.cpu.fatal(error);
                }
                task.state = TaskState::Runnable;
            }
            if matches!(task.state, TaskState::Ready | TaskState::Runnable) {
                if self.layout.role(task.lane).is_some() {
                    continue;
                }
                if self.domain().is_none() {
                    if selected.is_none() {
                        selected = Some(i);
                    }
                    if !has_firmware {
                        break; // Preserve the legacy Runtime's exact RR behavior.
                    }
                    continue;
                }
                let precedes = if due_service && i == 2 {
                    true
                } else if due_service && selected == Some(2) {
                    false
                } else if let Some(prior) = selected {
                    // SAFETY: dispatcher IF0, no task or callback is running.
                    let prior = unsafe { &*self.task(prior) };
                    let result = (|| {
                        guard.state().scheduling.precedes(
                            (
                                task.id,
                                task.job.ok_or(WorkError::CorruptContext)?.1.sequence,
                            ),
                            (
                                prior.id,
                                prior.job.ok_or(WorkError::CorruptContext)?.1.sequence,
                            ),
                        )
                    })();
                    match result {
                        Ok(value) => value,
                        Err(error) => self.queue.cpu.fatal(error),
                    }
                } else {
                    true
                };
                if precedes {
                    selected = Some(i);
                }
            }
        }
        drop(guard);
        // SAFETY: the checked dispatcher still holds IF0; all task
        // frames are stopped and no mutable task borrow survives the scan.
        let firmware = unsafe { self.next_firmware(count) };
        // Firmware is outside Linux FIFO ranks. At most one firmware turn
        // separates eligible non-firmware turns; an idle peer set permits it to
        // continue. Due Linux deadline/IRQ work keeps its existing precedence.
        if let Some(index) = firmware
            && !(due_service && selected == Some(2))
            && (self.firmware_turn.get() || selected.is_none())
        {
            selected = Some(index);
        }
        let Some(i) = selected else {
            // SAFETY: `flags` came from the save_disable above, and no lock is held.
            unsafe { self.queue.cpu.restore(flags) };
            let counts = &step_counts::STEP_COUNTS;
            step_counts::add(&counts.empty, step_counts::now().wrapping_sub(entry));
            step_counts::add(&counts.empty_steps, 1);
            return Ok(false);
        };
        if !(due_service && i == 2) {
            self.firmware_turn.set(Some(i) != firmware);
            if Some(i) == firmware {
                self.firmware_cursor.set((i + 1) % count);
            }
            self.cursor.set((i + 1) % count);
        }
        // SAFETY: immutable selected job at IF0; no borrow spans switch.
        let task = unsafe { &*self.task(i) };
        // Start measurement after task selection, not at dispatcher entry.
        let now = match self.queue.scheduling_now() {
            Ok(now) => now,
            Err(error) => {
                // SAFETY: no Running row or foreign arrival committed yet.
                unsafe { self.queue.cpu.restore(flags) };
                return Err(error);
            }
        };
        let running = (|| {
            self.queue.lock()?.state().scheduling.running(
                task.id,
                task.job.ok_or(WorkError::CorruptContext)?.1.sequence,
                now,
            )
        })();
        if let Err(error) = running {
            self.queue.cpu.fatal(error);
        }
        if self.domain().is_some() {
            // The Running transition already committed. Every subsequent
            // invariant failure retains fatally rather than returning a partly
            // dispatched row as a normal refused step.
            let opened = (|| {
                let sequence = task.job.ok_or(WorkError::CorruptContext)?.1.sequence;
                let (total, through, segment) = self
                    .queue
                    .lock()?
                    .state()
                    .scheduling
                    .execution(task.id, sequence, now)?;
                let sample = ExecutionSample::new(
                    ExecutionIdentity::new(self.queue.id, task.id.get(), sequence),
                    total,
                    through,
                    segment,
                );
                self.execution.open(i, sample)?;
                self.execution.publish_process(i, &sample);
                Ok::<(), WorkError>(())
            })();
            opened.unwrap_or_else(|error| self.queue.cpu.fatal(error));
        }
        // SAFETY: task `i` is idle or waiting, and IRQs are disabled.
        unsafe {
            (*self.task(i)).state = TaskState::Running;
        }
        self.current.set(i);
        // SAFETY: task `i` is valid, and IRQs are disabled.
        unsafe {
            let task = &*self.task(i);
            self.queue.active.publish(task.id, &task.stack.bounds);
        }
        #[cfg(feature = "linux-kernel")]
        self.physical_enter(i)
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        let switched = step_counts::now();
        // SAFETY: both frames are held, pinned and initialized; IRQs stay disabled
        // across the switch, and no lock or borrow spans it.
        unsafe {
            context::switch(
                self.dispatch.get(),
                core::ptr::addr_of!((*self.task(i)).frame),
            )
        };
        let returned = step_counts::now();
        if self.current.get() != NONE || self.queue.active.id.load(Ordering::Acquire) != 0 {
            self.queue.cpu.fatal(WorkError::CorruptContext);
        }
        // Original do_task_dead never returns through its C/Rust callback.
        // Only this real architectural successor may acknowledge its tail.
        self.terminal_returned(i);
        // Record only after real native switch return, before external outer
        // restore can unwind. This does not retire foreign C/page/alias users.
        if let Err(error) = self.returned_stack(i) {
            self.queue.cpu.fatal(error);
        }
        // SAFETY: the dispatcher resumed with `flags` from its own save_disable.
        unsafe { self.queue.cpu.restore(flags) };
        // The completing/parked frame is stopped, Task/active metadata is clear,
        // and exact outer IF restoration finished before any observer executes.
        let published = self.publish_general_resources();
        let counts = &step_counts::STEP_COUNTS;
        step_counts::add(&counts.select, switched.wrapping_sub(entry));
        step_counts::add(&counts.body, returned.wrapping_sub(switched));
        step_counts::add(&counts.after, step_counts::now().wrapping_sub(returned));
        published?;
        Ok(true)
    }

    /// One real dispatch, or one owned idle/event wait followed by dispatch.
    /// Due inspection only wakes; all service callbacks run on its real stack.
    pub fn step_or_idle(self: Pin<&Self>) -> Result<bool, WorkError> {
        if self.step()? {
            return Ok(true);
        }
        let Some(domain) = self.domain() else {
            return Ok(false);
        };
        if let Err(error) = domain.idle() {
            self.queue.cpu.fatal(error);
        }
        self.step()
    }

    /// Foreground AML/driver calls receive a real distinct task/stack identity.
    /// Dispatcher/root/IRQ callers never receive an invented ACPICA thread ID.
    /// # Safety
    /// Callback/context ownership and bounded-stack/no-unwind rules match execute.
    /// No caller stack-local reference may escape into the job. Timeout or
    /// Pending does not cancel or release admitted work: its provider-
    /// owned context remains held until completion or permanent quarantine.
    pub unsafe fn call_foreground(
        self: Pin<&Self>,
        callback: Callback,
        ctx: *mut c_void,
        deadline: u64,
    ) -> Result<(), WorkError> {
        self.queue.process()?;
        if self.current.get() != NONE {
            return Err(WorkError::WouldDeadlock);
        }
        let ticket = self.queue.enqueue_lane(Lane::Foreground, callback, ctx)?;
        while !self.queue.completed(ticket)? {
            if self.queue.cpu.ticks() >= deadline {
                return Err(WorkError::TimedOut);
            }
            if !self.step()? {
                return Err(WorkError::Pending);
            }
        }
        Ok(())
    }

    /// # Safety
    /// The checked dispatcher/current task owns this Runtime, the queue lock
    /// holds IF0, all peers are suspended, and no task borrow spans this call.
    pub(super) unsafe fn peer_progress(&self, current: usize, state: &State<N>) -> bool {
        if current == NONE {
            return true;
        }
        let mut queued_lanes = [false; queue_state::LANES];
        for job in state.jobs.iter().flatten().filter(|job| !job.running) {
            queued_lanes[queue_state::lane_index(job.lane)] = true;
        }
        // SAFETY: caller holds the same-CPU IF0 queue guard and all peer
        // frames are stopped. This read neither switches nor keeps references.
        unsafe {
            (&*self.tasks.get()).iter().enumerate().any(|(i, task)| {
                i != current
                    && match task.state {
                        TaskState::Ready | TaskState::Runnable => true,
                        TaskState::Blocked => task.signal.pending.load(Ordering::Acquire),
                        TaskState::Idle => queued_lanes[queue_state::lane_index(task.lane)],
                        TaskState::Running
                        | TaskState::Reserved
                        | TaskState::Escrow
                        | TaskState::Preparing
                        | TaskState::Dormant
                        | TaskState::Terminating
                        | TaskState::Retained => false,
                    }
            })
        }
    }
}
