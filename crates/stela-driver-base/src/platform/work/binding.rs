//! The executing job's identity: current-role checks, stack bindings and wake
//! and scheduling handles.

use super::*;

impl<const N: usize> Runtime<N> {
    pub(super) unsafe fn task(&self, index: usize) -> *mut Task {
        // SAFETY: Immutable Vec allocation: take no Rust reference across a switch.
        unsafe { (&mut *self.tasks.get()).as_mut_ptr().add(index) }
    }

    /// Actual same-CPU successor, outside every constructor-owned worker.
    /// Completion/current metadata cleared on a worker is not this proof.
    pub(super) fn dispatcher_successor(self: Pin<&Self>) -> Result<(), WorkError> {
        self.queue.process()?;
        if self.current.get() != NONE
            || self.queue.completing.load(Ordering::Acquire)
            || self.queue.active.id.load(Ordering::Acquire) != 0
            || self.queue.active.start.load(Ordering::Acquire) != 0
            || self.queue.active.end.load(Ordering::Acquire) != 0
        {
            return Err(WorkError::Busy);
        }
        let rsp = context::stack_pointer();
        // SAFETY: actual affine CPU; constructor ranges are immutable and kept.
        if unsafe { &*self.tasks.get() }
            .iter()
            .any(|task| task.stack.bounds.contains(&rsp))
        {
            return Err(WorkError::Busy);
        }
        Ok(())
    }
    /// Record a completed callback's PHYSICAL return, not a guessed counter.
    /// Caller is the dispatcher immediately after its actual context switch.
    pub(super) fn returned_stack(self: Pin<&Self>, index: usize) -> Result<(), WorkError> {
        self.dispatcher_successor()?;
        // SAFETY: real checked successor/IF0; no Task borrow spans a callback.
        let task = unsafe { &mut *self.task(index) };
        if matches!(task.state, TaskState::Idle | TaskState::Retained) {
            if task.job.is_some()
                || task.signal.sequence.load(Ordering::Acquire) != 0
                || task.completed_sequence == 0
                || task.returned_sequence > task.completed_sequence
            {
                return Err(WorkError::corrupt());
            }
            task.returned_sequence = task.completed_sequence;
        }
        #[cfg(feature = "linux-kernel")]
        self.physical_departure(task.id, &task.signal)?;
        Ok(())
    }

    pub fn queue(&self) -> &Arc<Queue<N>> {
        &self.queue
    }

    pub(super) fn domain(&self) -> Option<&Arc<service::Domain<N>>> {
        // SAFETY: same-CPU access; one setup publication precedes all jobs.
        unsafe { (&*self.service_domain.get()).as_ref() }
    }

    /// Check CPU/current/RSP before identifying the firmware continuation.
    pub fn firmware_current(&self) -> Result<bool, WorkError> {
        self.current_id()?;
        // SAFETY: current_id validated the current index, state and kept stack.
        Ok(unsafe { (*self.task(self.current.get())).lane } == Lane::Firmware)
    }

    /// Constructor role after validating CPU, current job and real RSP.
    /// The tag is observation, never an execution or native service capability.
    pub fn firmware_role_current(&self) -> Result<Option<FirmwareRole>, WorkError> {
        self.current_id()?;
        // SAFETY: current_id validated the current index/state/kept stack.
        Ok(self
            .layout
            .role(unsafe { (*self.task(self.current.get())).lane }))
    }

    /// Inspect the constructor-owned reserved Service stack geometry on the
    /// actual process CPU. No address, task/current, source registration,
    /// allocation, release event or future execution permission is returned.
    pub fn service_supports(&self, required: StackRequirement) -> Result<bool, WorkError> {
        self.queue.process()?;
        if self.retired.get() {
            return Err(WorkError::Closed);
        }
        // SAFETY: immutable constructor vector on its actual CPU; no callback,
        // switch or mutation occurs during this bounded geometry observation.
        let task = unsafe { &*self.tasks.get() }
            .get(2)
            .ok_or(WorkError::Unavailable)?;
        Ok(task.lane == Lane::Service
            && required.accepts(stack_requirement::Geometry::of(&task.stack)))
    }

    /// Check CPU/current/RSP before identifying the reserved service.
    pub fn service_current(&self) -> Result<bool, WorkError> {
        self.current_id()?;
        // SAFETY: current_id validated the continuation.
        let task = unsafe { &*self.task(self.current.get()) };
        Ok(task.signal.service)
    }

    pub fn current_id(&self) -> Result<ContextId, WorkError> {
        let id = self.queue.current_id()?;
        let i = self.current.get();
        if i == NONE {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: `i` is the current task's valid index; no switch occurs while this
        // borrow lives.
        let task = unsafe { &(*self.task(i)) };
        if task.state != TaskState::Running
            || task.id != id
            || !task.stack.bounds.contains(&context::stack_pointer())
        {
            return Err(WorkError::corrupt());
        }
        Ok(task.id)
    }

    pub(super) fn context_view(&self, index: usize, active: bool) -> TaskContext {
        // SAFETY: callers check CPU/exclusion and a valid index.
        let task = unsafe { &*self.task(index) };
        TaskContext {
            queue_id: self.queue.id,
            context_id: task.id.get(),
            sequence: task.job.map_or(0, |(_, job)| job.sequence),
            stack_start: task.stack.bounds.start,
            stack_end: task.stack.bounds.end,
            guard_start: task.stack.guard.start,
            guard_end: task.stack.guard.end,
            stack_pointer: if active {
                context::stack_pointer()
            } else {
                task.frame.saved_stack_pointer()
            },
        }
    }

    pub fn current_context(&self) -> Result<TaskContext, WorkError> {
        self.current_id()?;
        Ok(self.context_view(self.current.get(), true))
    }

    /// Attach a preallocated binding on this executing stack.
    /// No hook runs on a dispatcher/IRQ stack. Failure does not install the
    /// binding. This call consumes the supplied Arc; keep a separate clone
    /// before calling if it must be reused after failure. Process-context
    /// failure drops that Arc after restoring any saved IRQ flags. IRQ/foreign-
    /// CPU rejection quarantines the supplied roots instead of releasing them.
    pub fn bind_current(
        self: Pin<&Self>,
        binding: Arc<dyn SwitchBinding>,
    ) -> Result<WakeHandle<N>, WorkError> {
        if let Err(error) = self.current_id() {
            if self.queue.process().is_err() {
                // Rejection on IRQ/foreign CPU is not final owner-release
                // permission. Preserve the supplied binding's roots there.
                core::mem::forget(binding);
                quarantine();
            }
            return Err(error);
        }
        let flags = self.queue.cpu.save_disable()?;
        let index = self.current.get();
        let mut rejected_publication = None;
        let result = (|| {
            // SAFETY: current is validated and IF remains disabled throughout.
            if unsafe { (*self.task(index)).binding.is_some() } {
                return Err(WorkError::Busy);
            }
            let view = self.context_view(index, true);
            // SAFETY: current task is held and IF0 protects its job assignment.
            let task = unsafe { &*self.task(index) };
            if view.sequence == 0 || task.signal.sequence.load(Ordering::Acquire) != view.sequence {
                return Err(WorkError::corrupt());
            }
            let publication = binding.execution_publication(&view)?;
            let metric_sample = if let Some(publication) = publication {
                if let Err((publication, error)) = self.execution.prepare_target(index, publication)
                {
                    rejected_publication = Some(publication);
                    return Err(error);
                }
                let measured = (|| {
                    let now = self.execution.clock()?.now_ns()?;
                    self.execution.sample_at(index, now)
                })();
                match measured {
                    Ok(sample) => Some(sample),
                    Err(error) => {
                        rejected_publication = self.execution.abort_target(index);
                        return Err(error);
                    }
                }
            } else {
                None
            };
            // SAFETY: transactional source attach, with preallocated data lease
            // claimed in its successful nofallible tail. No Board borrow lives.
            if let Err(error) = unsafe { binding.attach(&view) } {
                rejected_publication = self.execution.abort_target(index);
                return Err(error);
            }
            if let Some(sample) = metric_sample {
                self.execution.publish_process(index, &sample);
            }

            let handle = WakeHandle {
                queue: ManuallyDrop::new(self.queue.clone()),
                signal: ManuallyDrop::new(task.signal.clone()),
                id: task.id,
                sequence: view.sequence,
            };
            // SAFETY: exclusive checked CPU; prior immutable borrow's last use is above.
            unsafe {
                (*self.task(index)).binding = Some(binding.clone());
            }
            Ok(handle)
        })();
        // SAFETY: no hook/lock is live and flags came from save_disable above.
        unsafe { self.queue.cpu.restore(flags) };
        drop(rejected_publication);
        result
    }

    pub fn wake_handle(self: Pin<&Self>) -> Result<WakeHandle<N>, WorkError> {
        self.current_id()?;
        // SAFETY: current task is validated; this method never switches.
        let task = unsafe { &*self.task(self.current.get()) };
        if task.binding.is_none() {
            return Err(WorkError::Unavailable);
        }
        let sequence = task.job.ok_or_else(|| WorkError::corrupt())?.1.sequence;
        if sequence == 0 || task.signal.sequence.load(Ordering::Acquire) != sequence {
            return Err(WorkError::corrupt());
        }
        Ok(WakeHandle {
            queue: ManuallyDrop::new(self.queue.clone()),
            signal: ManuallyDrop::new(task.signal.clone()),
            id: task.id,
            sequence,
        })
    }

    /// Capture scheduling permission for this job in the optional domain.
    /// The handle keeps only immutable queue/signal/generation metadata.
    pub fn scheduling_handle(
        self: Pin<&Self>,
    ) -> Result<scheduling::SchedulingHandle<N>, WorkError> {
        self.current_id()?;
        if self.domain().is_none() || self.firmware_role_current()?.is_some() {
            return Err(WorkError::Unavailable);
        }
        let wake = self.wake_handle()?;
        let handle = scheduling::SchedulingHandle { wake };
        handle.snapshot()?;
        Ok(handle)
    }

    pub(super) fn arrive(&self, index: usize, new_job: bool) {
        self.valid();
        let view = self.context_view(index, true);
        // SAFETY: current task on its real destination stack at IF0.
        if let Some(binding) = unsafe { &(*self.task(index)).binding } {
            // SAFETY: no lock/switch borrow; the hook restores foreign state.
            let result = unsafe { binding.arrive(&view, new_job) };
            if let Err(error) = result {
                self.queue.cpu.fatal(error);
            }
        }
    }

    /// # Safety
    /// Real foreign producers/task/wake references are quiescent. The kept
    /// binding validates this at detach; copied metadata is not lifetime proof.
    /// Returned owner is released only in process context after IF restoration.
    pub unsafe fn detach_binding(
        self: Pin<&Self>,
        id: ContextId,
    ) -> Result<Arc<dyn SwitchBinding>, WorkError> {
        self.queue.process()?;
        if self.current.get() != NONE {
            return Err(WorkError::WouldDeadlock);
        }
        let flags = self.queue.cpu.save_disable()?;
        let result = (|| {
            // SAFETY: dispatcher owns all task state with IF=0; vector never resizes.
            let index = unsafe { (&*self.tasks.get()).iter().position(|task| task.id == id) }
                .ok_or(WorkError::Unavailable)?;
            // SAFETY: index comes from this vector.
            let task = unsafe { &*self.task(index) };
            if !matches!(task.state, TaskState::Idle | TaskState::Retained) || task.job.is_some() {
                return Err(WorkError::Pending);
            }
            if self.execution.has_target(index) {
                return Err(WorkError::Pending);
            }
            let binding = task.binding.as_ref().ok_or(WorkError::Unavailable)?;
            let view = self.context_view(index, false);
            if !(view.stack_start..view.stack_end).contains(&view.stack_pointer) {
                return Err(WorkError::corrupt());
            }
            // SAFETY: unsafe caller supplies real quiescence; failure is transactional.
            unsafe { binding.detach(&view)? };
            // SAFETY: no task/hook is running; take moves owner without dropping it.
            if task.state == TaskState::Retained && task.stack_pin_nonce == 0 {
                let shape = (task.lane == Lane::General)
                    .then(|| ReleasedStackGeometry::of(&task.stack, !task.ever_entered));
                // Detach already committed. A later ledger failure must retain
                // fatally, never return an ordinary transactional refusal.
                if let Some(shape) = shape {
                    self.queue
                        .lock()
                        .and_then(|mut guard| guard.state().resources.stack_released(shape))
                        .unwrap_or_else(|error| self.queue.cpu.fatal(error));
                }
                // SAFETY: exact stopped unbound/pinless worker at IF0; no old
                // target remains after actual stopped-publication removal.
                unsafe {
                    (*self.task(index)).state = TaskState::Idle;
                }
            }
            // SAFETY: exact stopped worker at IF0; foreign detach committed
            // and prior immutable task/binding borrows ended. Move the Arc
            // without dropping it or calling foreign code under this scope.
            match unsafe { (*self.task(index)).binding.take() } {
                Some(binding) => Ok(binding),
                None => self.queue.cpu.fatal(WorkError::corrupt()),
            }
        })();
        // SAFETY: no owner is dropped by moving the Result; no queue lock is held.
        unsafe { self.queue.cpu.restore(flags) };
        result
    }

    pub(super) fn valid(&self) {
        if let Err(e) = self.current_id() {
            self.queue.cpu.fatal(e)
        }
    }
}
