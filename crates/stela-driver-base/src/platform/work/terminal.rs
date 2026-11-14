//! Nonreturning original task exit and its genuine native successor join.
use super::*;

impl<const N: usize> Runtime<N> {
    /// Leave original do_task_dead without returning through dying foreign
    /// frames. Every original resource/reaper/RCU root remains independently
    /// retained; completed native work is not source object retirement.
    ///
    /// # Safety
    /// The original source has committed this exact child to terminal exit.
    /// Its binding prepared actual source terminal state, and all skipped
    /// frames contain no live owning Rust guards/destructors/native process
    /// permit, queue/RefCell borrow or foreign return obligation. Original C
    /// terminal cleanup is explicitly owned by a real retained source reaper.
    pub unsafe fn terminate_current(self: Pin<&Self>) -> ! {
        let fail = |error| self.queue.cpu.fatal(error);
        self.current_id()
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        let index = self.current.get();
        // SAFETY: actual current index/RSP checked above; pure same-CPU read.
        let task = unsafe { &*self.task(index) };
        if task.lane != Lane::General
            || task.stack_pin_nonce == 0
            || task.binding.is_none()
            || task.owned_escrows != 0
            || self.pending_reservations.get() != 0
        {
            fail(WorkError::Busy);
        }
        let _flags = self
            .queue
            .cpu
            .save_disable()
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        let now = self
            .queue
            .scheduling_now()
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        self.queue
            .lock()
            .and_then(|mut guard| guard.state().scheduling.account(now))
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        self.execution_publish_at(index, now)
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        let view = self.context_view(index, true);
        self.queue.completing.store(true, Ordering::Release);
        // SAFETY: exact checked dying source task and callback-scoped view at
        // IF0; all checks precede terminal source departure, no lock spans hook.
        unsafe {
            (*self.task(index))
                .binding
                .as_ref()
                .unwrap()
                .terminal_prepare(&view)
        }
        .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        self.queue.completing.store(false, Ordering::Release);
        self.execution_stop_at(index, now)
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));

        // SAFETY: exact dying current at IF0; no callback/Drop/borrow survives
        // these mutations or the architectural switch. Saved frame is retained
        // permanently until separate old-C/stack/page/alias/RCU joins permit zero.
        unsafe {
            (*self.task(index)).state = TaskState::Terminating;
        }
        self.current.set(NONE);
        self.queue.active.clear();
        // SAFETY: both native frames belong to this kept Runtime. The saved
        // dying frame is never eligible for ordinary scheduling or return.
        unsafe {
            context::switch(
                core::ptr::addr_of_mut!((*self.task(index)).frame),
                self.dispatch.get(),
            );
        }
        fail(WorkError::corrupt())
    }
    pub(super) fn terminal_returned(self: Pin<&Self>, index: usize) {
        // SAFETY: called only by actual dispatcher immediately after switch.
        if unsafe { (*self.task(index)).state } != TaskState::Terminating {
            return;
        }
        let fail = |error| self.queue.cpu.fatal(error);
        self.dispatcher_successor()
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        let view = self.context_view(index, false);
        // SAFETY: exact retained terminal binding, actual off-all-stacks
        // successor at IF0, no original process/current is manufactured here.
        unsafe {
            (*self.task(index))
                .binding
                .as_ref()
                .unwrap()
                .terminal_successor(&view)
        }
        .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        self.dispatcher_successor()
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        self.queue
            .lock()
            .and_then(|mut guard| {
                guard
                    .state()
                    .scheduling
                    .remove_terminal(ContextId(view.context_id), view.sequence)
            })
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        {
            // SAFETY: actual successor retains stopped task; no reference spans
            // queue operations or later mutation. Close exact old wake generation.
            let signal = unsafe { &(*self.task(index)).signal };
            signal.sequence.store(0, Ordering::Release);
        }
        // SAFETY: same checked kept terminal task, no code can alter its job.
        let (slot, job) =
            unsafe { (*self.task(index)).job }.unwrap_or_else(|| fail(WorkError::corrupt()));
        self.queue
            .finish(slot, job.sequence, None)
            .unwrap_or_else(|error| self.queue.cpu.fatal(error));
        // SAFETY: source successor acknowledged only metadata/notification;
        // stack/binding/page/task/RCU users remain retained, not reclaimed here.
        unsafe {
            let task = &mut *self.task(index);
            task.job = None;
            task.completed_sequence = job.sequence;
            task.state = TaskState::Retained;
        }
    }
}
