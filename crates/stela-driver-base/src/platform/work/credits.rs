//! Actual General resource release events, never periodic availability polls.
//! The queue retains the sink; the sink must not retain this Queue or Runtime.
use super::stack_requirement::ReleasedStackGeometry;
use super::*;

/// A bounded coalesced batch of ACTUAL releases since prior notification.
/// Values carry geometry/eligibility observations only, never a stack pin,
/// callback admission, queue capacity promise or C-publication receipt.
#[derive(Clone, Copy)]
pub struct GeneralRelease {
    queue_credit: bool,
    stacks: [Option<ReleasedStackGeometry>; MAX_CONTINUATIONS * 2],
}
impl GeneralRelease {
    fn empty() -> Self {
        Self {
            queue_credit: false,
            stacks: [None; MAX_CONTINUATIONS * 2],
        }
    }
    pub fn queue_credit(&self) -> bool {
        self.queue_credit
    }
    pub fn stacks(&self) -> impl Iterator<Item = ReleasedStackGeometry> + '_ {
        self.stacks.iter().copied().flatten()
    }
    fn with_stack(&mut self, shape: ReleasedStackGeometry) -> Result<(), WorkError> {
        if self.stacks.iter().flatten().any(|actual| *actual == shape) {
            return Ok(());
        }
        let slot = self
            .stacks
            .iter_mut()
            .find(|slot| slot.is_none())
            .ok_or(WorkError::Full)?;
        *slot = Some(shape);
        Ok(())
    }
}

/// Bounded IRQ-safe publication into a separately retained process consumer.
///
/// # Safety
/// Code/root/context stays covered by this Queue's actual original owner.
/// released only records epoch and raises an already-owned wake token; it must
/// not allocate, switch, enter C, run work inline, reenter this Queue or release
/// an affine owner. The sink retains no Queue/Runtime/Task or code-pin backedge.
/// drained is an allocation-free observation, true only after the actual
/// process consumer completed this event; no callback-presence readiness.
/// Abandonment keeps uncertain resources. Its methods must not unwind.
pub unsafe trait ResourceObserver: Send + Sync {
    fn released(&self, epoch: u64, actual: &GeneralRelease) -> Result<(), WorkError>;
    fn drained(&self, epoch: u64) -> Result<bool, WorkError>;
}
pub struct ObserveFailure {
    pub error: WorkError,
    _observer: ManuallyDrop<Arc<dyn ResourceObserver>>,
}
impl core::fmt::Debug for ObserveFailure {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.debug_struct("ObserveFailure")
            .field("error", &self.error)
            .finish_non_exhaustive()
    }
}
struct Observed {
    id: u64,
    observer: Arc<dyn ResourceObserver>,
    epoch: u64,
    pending: GeneralRelease,
    sent: u64,
    closing: Option<u64>,
    poisoned: bool,
}
pub(super) struct Resources {
    next: u64,
    current: Option<Observed>,
}
impl Resources {
    pub(super) fn new() -> Self {
        Self {
            next: 1,
            current: None,
        }
    }
    /// Private ledger only. Actual notification waits until all Task metadata
    /// and the outer IF restore complete in the taskless native dispatcher.
    pub(super) fn released(&mut self) -> Result<(), WorkError> {
        self.note(true, None)
    }
    /// Private note BEFORE actual successful Retained -> Idle, with immutable
    /// geometry + entry eligibility from that SAME original Stack. No callback.
    pub(super) fn stack_released(&mut self, shape: ReleasedStackGeometry) -> Result<(), WorkError> {
        self.note(false, Some(shape))
    }
    /// Exact unpublished reservation rollback releases both its admission slot
    /// and physical worker. One transactional note precedes BOTH mutations.
    pub(super) fn reservation_released(
        &mut self,
        shape: ReleasedStackGeometry,
    ) -> Result<(), WorkError> {
        self.note(true, Some(shape))
    }
    fn note(
        &mut self,
        credit: bool,
        shape: Option<ReleasedStackGeometry>,
    ) -> Result<(), WorkError> {
        if let Some(observer) = self
            .current
            .as_mut()
            .filter(|observer| observer.closing.is_none())
        {
            let next = observer.epoch.checked_add(1).ok_or(WorkError::Exhausted)?;
            // In place, not through a copy: the batch spans every stack, and
            // this runs on a releasing task's own small stack. `with_stack`
            // changes nothing when it fails, so the event stays transactional.
            if let Some(shape) = shape {
                observer.pending.with_stack(shape)?;
            }
            observer.pending.queue_credit |= credit;
            observer.epoch = next;
        }
        Ok(())
    }
    pub(super) fn attached(&self) -> bool {
        self.current.is_some()
    }
}
/// This registration owns the actual Queue, but the sink owns no registration.
/// Default Drop retains the Queue/observer/source graph, never unregistering
/// under a stale CPU/IRQ context or mistaking cancellation for an event join.
pub struct ResourceRegistration<const N: usize> {
    queue: ManuallyDrop<Arc<Queue<N>>>,
    id: u64,
    live: bool,
    _local: PhantomData<alloc::rc::Rc<()>>,
}
impl<const N: usize> Drop for ResourceRegistration<N> {
    fn drop(&mut self) {
        if self.live {
            quarantine();
        }
    }
}
impl<const N: usize> Queue<N> {
    fn resource_dispatcher(&self) -> Result<(), WorkError> {
        self.process()?;
        if self.resource_busy.load(Ordering::Acquire) {
            return Err(WorkError::Pending);
        }
        if self.active.id.load(Ordering::Acquire) != 0 || self.completing.load(Ordering::Acquire) {
            return Err(WorkError::WouldDeadlock);
        }
        if self.retired.load(Ordering::Acquire) || self.abandoned.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        Ok(())
    }
    /// # Safety
    /// The actual original Queue owner retains this observer's compiled code
    /// and roots. Observer is fresh for this exact registration, with no stale
    /// epoch/consumer state, and obeys its IRQ-safe, no-cycle contract. Register in
    /// the actual taskless dispatcher before consumer/producers are published.
    /// The actual event consumer executes only on this same original Queue.
    unsafe fn observe_general_resources(
        self: &Arc<Self>,
        observer: Arc<dyn ResourceObserver>,
    ) -> Result<ResourceRegistration<N>, ObserveFailure> {
        let mut observer = ManuallyDrop::new(observer);
        let result = (|| {
            self.resource_dispatcher()?;
            let mut guard = self.lock()?;
            let state = guard.state();
            if state.closed || state.poisoned {
                return Err(WorkError::Closed);
            }
            if state.resources.current.is_some() {
                return Err(WorkError::Busy);
            }
            let id = state.resources.next;
            let next = id.checked_add(1).ok_or(WorkError::Exhausted)?;
            // SAFETY: ownership moves once into the real protected slot. No
            // callback occurs locked; no prior observer is replaced or dropped.
            let source = unsafe { ManuallyDrop::take(&mut observer) };
            state.resources.next = next;
            state.resources.current = Some(Observed {
                id,
                observer: source,
                epoch: 0,
                pending: GeneralRelease::empty(),
                sent: 0,
                closing: None,
                poisoned: false,
            });
            self.resource_observed.store(true, Ordering::Release);
            Ok(id)
        })();
        match result {
            Ok(id) => Ok(ResourceRegistration {
                queue: ManuallyDrop::new(self.clone()),
                id,
                live: true,
                _local: PhantomData,
            }),
            Err(error) => Err(ObserveFailure {
                error,
                _observer: observer,
            }),
        }
    }
    /// Publish only a real recorded General credit event, outside queue locks,
    /// any running Task/complete metadata and the outer IF restoration. A pass
    /// with no event does nothing; timer inspection never creates an epoch.
    fn publish_general_resources(&self) -> Result<(), WorkError> {
        if !self.resource_observed.load(Ordering::Acquire) {
            return Ok(());
        }
        self.resource_dispatcher()?;
        let captured = {
            let mut guard = self.lock()?;
            let Some(observer) = guard.state().resources.current.as_mut() else {
                return Ok(());
            };
            if observer.poisoned {
                return Err(WorkError::Pending);
            }
            let epoch = observer.closing.unwrap_or(observer.epoch);
            if epoch == observer.sent {
                return Ok(());
            }
            self.resource_busy.store(true, Ordering::Release);
            // Guard BEFORE its IRQ restore tail; panic must not release roots.
            (
                observer.id,
                epoch,
                observer.pending,
                ManuallyDrop::new(observer.observer.clone()),
            )
        };
        let (id, epoch, actual, mut observer) = captured;
        let result = observer.released(epoch, &actual);
        {
            let mut guard = self.lock()?;
            let Some(current) = guard
                .state()
                .resources
                .current
                .as_mut()
                .filter(|current| current.id == id)
            else {
                return Err(WorkError::corrupt());
            };
            match result {
                Ok(()) => {
                    // Same actual taskless CPU, no queue reentry/switch from
                    // bounded sink callback: no later event can be erased here.
                    current.sent = epoch;
                    current.pending = GeneralRelease::empty();
                }
                Err(_) => current.poisoned = true,
            }
        }
        self.resource_busy.store(false, Ordering::Release);
        // SAFETY: callback returned; actual original metadata lock and IF
        // restore tail completed. No callback/source owner is released locked.
        drop(unsafe { ManuallyDrop::take(&mut observer) });
        result
    }
}
impl<const N: usize> Runtime<N> {
    /// Actual architectural dispatcher boundary. Clearing current/active on
    /// the completing stack does not itself prove the context switch returned.
    fn resource_dispatcher(self: Pin<&Self>) -> Result<(), WorkError> {
        self.queue.resource_dispatcher()?;
        self.dispatcher_successor().map_err(|error| {
            if error == WorkError::Busy {
                WorkError::WouldDeadlock
            } else {
                error
            }
        })
    }
    /// Register one exact native Runtime event source, before publishing any
    /// process consumer. The sink retains no Runtime/Queue/code-pin backedge.
    ///
    /// # Safety
    /// Same retained actual code/root and fresh no-cycle observer contract as
    /// ResourceObserver. Runtime/stack owners stay kept through explicit join;
    /// its retirement already refuses an attached source registration.
    pub unsafe fn observe_general_resources(
        self: Pin<&Self>,
        observer: Arc<dyn ResourceObserver>,
    ) -> Result<ResourceRegistration<N>, ObserveFailure> {
        if let Err(error) = self.resource_dispatcher() {
            return Err(ObserveFailure {
                error,
                _observer: ManuallyDrop::new(observer),
            });
        }
        // SAFETY: exact original Runtime/Queue was checked; caller retains the
        // actual code/root/source graph and fresh bounded observer above.
        unsafe { self.queue.observe_general_resources(observer) }
    }
    /// Publish only after physical return to this Runtime's actual dispatcher,
    /// outside every Task, metadata/with_process borrow and outer IF tail.
    /// No event means no callback; timer inspection creates no availability.
    pub fn publish_general_resources(self: Pin<&Self>) -> Result<(), WorkError> {
        if !self.queue.resource_observed.load(Ordering::Acquire) {
            return Ok(());
        }
        self.resource_dispatcher()?;
        self.queue.publish_general_resources()
    }
}
impl<const N: usize> ResourceRegistration<N> {
    /// Close future event capture, then publish the exact final target. The
    /// consumer still lives and must process it before retirement can succeed.
    /// This is no stop/worker/DMA/native-root retirement acknowledgement.
    pub fn close(&mut self, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        if !self.live {
            return Err(WorkError::Closed);
        }
        if !Arc::ptr_eq(&self.queue, runtime.queue()) {
            return Err(WorkError::InvalidTicket);
        }
        runtime.resource_dispatcher()?;
        {
            let mut guard = self.queue.lock()?;
            let Some(observer) = guard
                .state()
                .resources
                .current
                .as_mut()
                .filter(|observer| observer.id == self.id)
            else {
                return Err(WorkError::corrupt());
            };
            if observer.poisoned {
                return Err(WorkError::Pending);
            }
            observer.closing.get_or_insert(observer.epoch);
        }
        runtime.publish_general_resources()
    }
    /// Remove only after both actual native notifications and the exact
    /// retained process consumer event join. A taskless dispatcher proves
    /// that no C take/complete callback's return tail is currently executing;
    /// a sleeping event callback remains active in the sink and refuses.
    pub fn retire(&mut self, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        if !self.live {
            return Err(WorkError::Closed);
        }
        if !Arc::ptr_eq(&self.queue, runtime.queue()) {
            return Err(WorkError::InvalidTicket);
        }
        runtime.resource_dispatcher()?;
        let (target, mut source) = {
            let mut guard = self.queue.lock()?;
            let Some(observer) = guard
                .state()
                .resources
                .current
                .as_mut()
                .filter(|observer| observer.id == self.id)
            else {
                return Err(WorkError::corrupt());
            };
            let target = observer.closing.ok_or(WorkError::Busy)?;
            if observer.poisoned || observer.sent != target {
                return Err(WorkError::Pending);
            }
            self.queue.resource_busy.store(true, Ordering::Release);
            (target, ManuallyDrop::new(observer.observer.clone()))
        };
        let drained = source.drained(target);
        let (mut removed, result) = {
            let mut guard = self.queue.lock()?;
            let current = &mut guard.state().resources.current;
            let Some(observer) = current.as_mut().filter(|observer| observer.id == self.id) else {
                return Err(WorkError::corrupt());
            };
            match drained {
                Ok(true) => (ManuallyDrop::new(current.take()), Ok(())),
                Ok(false) => (ManuallyDrop::new(None), Err(WorkError::Pending)),
                Err(error) => {
                    observer.poisoned = true;
                    (ManuallyDrop::new(None), Err(error))
                }
            }
        };
        if result.is_ok() {
            self.queue.resource_observed.store(false, Ordering::Release);
        }
        self.queue.resource_busy.store(false, Ordering::Release);
        // SAFETY: normal observer return and native lock/IF restoration. Even
        // a known pending event releases this temporary clone only outside locks.
        drop(unsafe { ManuallyDrop::take(&mut source) });
        // SAFETY: removed is Some only after the actual source/consumer join.
        drop(unsafe { ManuallyDrop::take(&mut removed) });
        result?;
        self.live = false;
        // SAFETY: unique successful retirement; later Drop touches only live.
        unsafe { ManuallyDrop::drop(&mut self.queue) };
        Ok(())
    }
}
