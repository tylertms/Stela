//! Bounded native deferred work with real kept task continuations.
//!
//! This service is pumped by its owning physical CPU. It does not install a
//! scheduler interrupt, invent an ACPICA provider, or claim autonomous wakeup.
//! Admission never invokes a callback, allocates or releases kept owners.
//! Process/deferred context owns construction, pumping, retirement and every
//! final owner release. The pump preserves incoming IF exactly; it never enables
//! interrupt delivery to manufacture progress. A future SCI/ACPICA adapter must
//! supply a dispatcher policy that permits hardware IRQ progress and
//! main-owned interrupt metadata, plus IRQ-safe allocation/synchronizers.
//! This module alone does not check that provider.
use crate::bus::dma::memory::{MemoryError, PinnedPages};
#[cfg(target_arch = "x86_64")]
use alloc::vec::Vec;
use alloc::{boxed::Box, sync::Arc};
#[cfg(target_arch = "x86_64")]
use core::sync::atomic::AtomicUsize;
use core::{
    any::Any,
    cell::UnsafeCell,
    ffi::c_void,
    mem::ManuallyDrop,
    ops::Range,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
#[cfg(target_arch = "x86_64")]
use core::{
    cell::Cell,
    marker::{PhantomData, PhantomPinned},
    pin::Pin,
};

#[cfg(target_arch = "x86_64")]
mod accounting;
#[cfg(target_arch = "x86_64")]
mod binding;
#[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
mod inactive;
#[cfg(target_arch = "x86_64")]
pub use accounting::{
    ExecutionIdentity, ExecutionPublication, ExecutionSample, ExecutionTap, InterruptedExecution,
    StoppedPublication,
};
#[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
pub use inactive::{InactiveObservation, InactiveTarget};
#[cfg(target_arch = "x86_64")]
mod context;
#[cfg(target_arch = "x86_64")]
mod dispatch;
#[cfg(target_arch = "x86_64")]
mod lifetime;
mod queue_state;
#[cfg(target_arch = "x86_64")]
pub mod scheduling;
#[cfg(target_arch = "x86_64")]
pub mod service;
#[cfg(target_arch = "x86_64")]
pub mod step_counts;
#[cfg(target_arch = "x86_64")]
mod terminal;

#[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
pub mod kernel_domain;

#[cfg(target_arch = "x86_64")]
mod credits;
#[cfg(target_arch = "x86_64")]
mod general_reservation;
#[cfg(target_arch = "x86_64")]
pub use credits::{GeneralRelease, ObserveFailure, ResourceObserver, ResourceRegistration};
mod reservation;
mod stack_requirement;
#[cfg(target_arch = "x86_64")]
pub use general_reservation::{
    AssignedGeneral, CancelledReservation, DormantGeneralStack, EnteredProcess, GeneralBlocker,
    GeneralRefusal, GeneralReservation, GeneralStackPreparation, OwnedGeneralEscrow,
    OwnedPinnedGeneral, OwnedTransferFailure, PinnedGeneralStackPreparation, PublishedGeneralStack,
    StackPinError,
};
pub use stack_requirement::{ReleasedStackGeometry, StackProfile, StackRequirement};
#[cfg(all(any(test, feature = "test-support"), unix, target_arch = "x86_64"))]
pub mod test_support;
use reservation::Layout;
pub use reservation::{FirmwareQueue, FirmwareRole, GpeQueueBudget, ReservationPolicy};

pub type Callback = unsafe extern "C" fn(*mut c_void);
pub type RetainedOwner = Arc<dyn Any + Send + Sync>;
#[cfg(target_arch = "x86_64")]
const NONE: usize = usize::MAX;
#[cfg(target_arch = "x86_64")]
static NEXT_CONTEXT: AtomicU64 = AtomicU64::new(1);
static NEXT_QUEUE: AtomicU64 = AtomicU64::new(1);
static QUARANTINED: AtomicU64 = AtomicU64::new(0);

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum WorkError {
    Unavailable,
    WrongCpu,
    InterruptContext,
    Busy,
    Full,
    Closed,
    Exhausted,
    InvalidStack,
    InvalidTicket,
    WouldDeadlock,
    TimedOut,
    CorruptContext,
    Pending,
}

/// A plain account for the boot log and screen.
impl core::fmt::Display for WorkError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        f.write_str(match self {
            Self::Unavailable => "the work service is unavailable",
            Self::WrongCpu => "the work ran on the wrong processor",
            Self::InterruptContext => "the work was asked for from an interrupt",
            Self::Busy => "the work service is busy",
            Self::Full => "the work queue is full",
            Self::Closed => "the work queue is closed",
            Self::Exhausted => "the work service has no room left",
            Self::InvalidStack => "a work stack is invalid",
            Self::InvalidTicket => "a work ticket is invalid",
            Self::WouldDeadlock => "the work would wait on itself",
            Self::TimedOut => "the work timed out",
            Self::CorruptContext => "a work context is corrupt",
            Self::Pending => "the work has not finished",
        })
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Kind {
    General,
    Notify,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[cfg_attr(
    not(target_arch = "x86_64"),
    expect(dead_code, reason = "only the x86-64 native work lanes use it")
)]
enum Lane {
    Foreground,
    General,
    Notify,
    Service,
    Firmware,
    Portal,
    Gpe,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ContextId(u64);
impl ContextId {
    pub fn get(self) -> u64 {
        self.0
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ticket {
    queue: u64,
    sequence: u64,
}

/// Main's processor/context adapter. There are no no-op defaults.
///
/// # Safety
/// All methods operate at CPL0 on one checked physical CPU, with native
/// interrupt-safe compiler code (no unprotected red zone), a stable main-owned
/// GS/FS/root context, and no task migration or root/segment changes by work.
/// `save_disable` checks
/// affinity then saves/disables local IRQs with compiler/memory ordering;
/// `restore` restores those exact flags on that CPU. `in_interrupt` uses real
/// main-owned IRQ context metadata, not IF or a made-up task ID. `ticks` is a
/// progressing monotonic clock on this CPU, including while IRQs are disabled.
/// The adapter, callback code and roots stay held through quarantine.
/// `fatal` must not return/unwind. Operations never silently migrate contexts.
pub unsafe trait Cpu: Send + Sync {
    fn check_cpu(&self) -> Result<(), WorkError>;
    fn in_interrupt(&self) -> bool;
    fn save_disable(&self) -> Result<u64, WorkError>;
    /// # Safety
    /// Flags come from this same CPU's enclosing save_disable. Task metadata
    /// and hardware SP must already describe the resumed context; no queue
    /// lock spans this restoration or a context switch.
    unsafe fn restore(&self, flags: u64);
    fn ticks(&self) -> u64;
    fn fatal(&self, error: WorkError) -> !;
}

/// The exclusively owned task RAM, guard and all-root mapping lease.
///
/// # Safety
/// Bounds are immutable page-aligned writable WB native RAM, exclusively owned
/// for this task's stack, with no DMA/other mutable owner and no preexisting
/// live execution frames when admitted to a new executor. `guard` is an
/// unmapped page-aligned preceding range in every execution root, never
/// an address label. The entire RAM/code/root/guard lease remains valid until
/// Drop, including suspended frames and uncertain executor failure. Drop may
/// reclaim it only after the context and every producer/IRQ/work user stopped.
pub unsafe trait StackBacking: Send + Sync {
    fn bounds(&self) -> Range<usize>;
    fn guard(&self) -> Range<usize>;

    /// Export this actual zeroed native stack allocation epoch.
    /// No raw address, epoch scalar or caller label substitutes for the real
    /// original RAM/mapping/root/code owner. No cold compatibility fallback.
    ///
    /// # Safety
    /// Exact native preparation excludes execution and has zeroed the whole
    /// usable mapping after original C/task/page/alias/RCU users retired and
    /// an architectural successor proved old native return. epoch is its
    /// checked nonzero per-worker generation. The provider independently
    /// refuses overlapping old epoch/views and creates fresh per-epoch backing
    /// ownership/page records; never return a cached old pin. All metadata
    /// operations are nonblocking with no queue reentry/root change/parking.
    /// Keep actual RAM/guard/mappings for Runtime lifetime even when the final
    /// epoch view drops; that drop must not invalidate its reconstructed frame.
    unsafe fn pin_prepared_pages(&self, _epoch: u64) -> Result<PinnedPages, MemoryError> {
        Err(MemoryError::Unavailable)
    }
}
#[cfg_attr(
    not(target_arch = "x86_64"),
    expect(dead_code, reason = "only the x86-64 native work lanes use it")
)]
pub struct Stack {
    backing: Box<dyn StackBacking>,
    bounds: Range<usize>,
    guard: Range<usize>,
}
impl Stack {
    pub fn new(backing: Box<dyn StackBacking>) -> Result<Self, WorkError> {
        let bounds = backing.bounds();
        let guard = backing.guard();
        if !bounds.start.is_multiple_of(4096)
            || !bounds.end.is_multiple_of(4096)
            || bounds
                .end
                .checked_sub(bounds.start)
                .is_none_or(|n| n < 16 * 1024 || n > isize::MAX as usize)
            || guard.end != bounds.start
            || !guard.start.is_multiple_of(4096)
            || guard.end.checked_sub(guard.start).is_none_or(|n| n < 4096)
        {
            return Err(WorkError::InvalidStack);
        }
        Ok(Self {
            backing,
            bounds,
            guard,
        })
    }
    pub fn bounds(&self) -> Range<usize> {
        self.bounds.clone()
    }
}

#[derive(Clone, Copy)]
#[cfg_attr(
    not(target_arch = "x86_64"),
    expect(dead_code, reason = "only the x86-64 native work lanes use it")
)]
struct Job {
    callback: Callback,
    context: usize,
    lane: Lane,
    sequence: u64,
    running: bool,
    requirement: Option<StackRequirement>,
}
struct State<const N: usize> {
    #[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
    inactive: [Option<inactive::Subscriber>; MAX_CONTINUATIONS],
    jobs: [Option<Job>; N],
    reservations: [u64; N],
    #[cfg(target_arch = "x86_64")]
    next_reservation: u64,
    layout: Layout,
    stack_profile: Option<StackProfile>,
    stack_catalog: stack_requirement::Catalog,
    next: u64,
    closed: bool,
    poisoned: bool,
    #[cfg(target_arch = "x86_64")]
    scheduling: scheduling::Board,
    #[cfg(target_arch = "x86_64")]
    resources: credits::Resources,
}
/// No Runtime/Task pointer crosses into this IRQ-safe read view. A single
/// executor binds these identities to its stack ranges.
#[cfg(target_arch = "x86_64")]
struct ActiveContext {
    id: AtomicU64,
    start: AtomicUsize,
    end: AtomicUsize,
}
#[cfg(target_arch = "x86_64")]
impl ActiveContext {
    fn new() -> Self {
        Self {
            id: AtomicU64::new(0),
            start: AtomicUsize::new(0),
            end: AtomicUsize::new(0),
        }
    }
    // Both methods require the checked CPU with IF clear. No queue lock or
    // borrowed Task state survives the subsequent hardware stack transition.
    fn publish(&self, id: ContextId, bounds: &Range<usize>) {
        self.start.store(bounds.start, Ordering::Relaxed);
        self.end.store(bounds.end, Ordering::Relaxed);
        self.id.store(id.get(), Ordering::Release);
    }
    fn clear(&self) {
        self.id.store(0, Ordering::Release);
        self.start.store(0, Ordering::Relaxed);
        self.end.store(0, Ordering::Relaxed);
    }
}

/// Shared preallocated admission slots. Every active/queued callback is covered
/// by the provider/root owner kept at construction. Callbacks and
/// their dynamic contexts must obey unsafe enqueue's lifetime contract.
pub struct Queue<const N: usize> {
    id: u64,
    abandoned: AtomicBool,
    #[cfg(target_arch = "x86_64")]
    executor_claimed: AtomicBool,
    #[cfg(target_arch = "x86_64")]
    active: ActiveContext,
    cpu: ManuallyDrop<Arc<dyn Cpu>>,
    owner: ManuallyDrop<RetainedOwner>,
    state: ManuallyDrop<UnsafeCell<State<N>>>,
    held: AtomicBool,
    #[cfg_attr(
        not(target_arch = "x86_64"),
        expect(dead_code, reason = "only the x86-64 native work lanes use it")
    )]
    inspection: AtomicBool,
    #[cfg(target_arch = "x86_64")]
    completing: AtomicBool,
    retired: AtomicBool,
    #[cfg(target_arch = "x86_64")]
    published_stack_pins: AtomicUsize,
    #[cfg(target_arch = "x86_64")]
    execution_fence: Arc<accounting::Fence>,
    #[cfg(target_arch = "x86_64")]
    resource_busy: AtomicBool,
    #[cfg(target_arch = "x86_64")]
    resource_observed: AtomicBool,
}
// SAFETY: same-CPU IRQ-save exclusion protects all state; foreign CPUs fail
// before touching it. A bounded try-lock refuses unexpected/NMI reentry. No
// callback, allocation, context switch or kept-owner Drop occurs locked.
unsafe impl<const N: usize> Send for Queue<N> {}
// SAFETY: as for Send above.
unsafe impl<const N: usize> Sync for Queue<N> {}
struct Guard<'a, const N: usize> {
    queue: &'a Queue<N>,
    flags: u64,
}
impl<const N: usize> Guard<'_, N> {
    fn state(&mut self) -> &mut State<N> {
        // SAFETY: a Guard exists only while `held` is set with IRQs disabled, so this
        // is the sole reference to the state.
        unsafe { &mut *self.queue.state.get() }
    }
}
impl<const N: usize> Drop for Guard<'_, N> {
    fn drop(&mut self) {
        self.queue.held.store(false, Ordering::Release);
        // SAFETY: `flags` came from this CPU's save_disable in `lock`, and the lock is
        // released above.
        unsafe { self.queue.cpu.restore(self.flags) };
    }
}
impl<const N: usize> Queue<N> {
    fn execution_quiet(&self) -> bool {
        #[cfg(target_arch = "x86_64")]
        {
            self.execution_fence.quiet()
        }
        #[cfg(not(target_arch = "x86_64"))]
        {
            true
        }
    }
    fn stack_pins_quiet(&self) -> bool {
        #[cfg(target_arch = "x86_64")]
        {
            self.published_stack_pins.load(Ordering::Acquire) == 0
        }
        #[cfg(not(target_arch = "x86_64"))]
        {
            true
        }
    }
    /// # Safety
    /// `owner` is the provider/context/code/root lease for all jobs,
    /// until real producer quiescence and retirement, never a dummy.
    /// Every IRQ/producer keeps a Queue Arc through its full source
    /// lifetime; an unowned raw queue pointer never checks. Creation and
    /// final Arc/owner release occur in process/deferred context.
    pub unsafe fn new(cpu: Arc<dyn Cpu>, owner: RetainedOwner) -> Result<Arc<Self>, WorkError> {
        cpu.check_cpu()?;
        if cpu.in_interrupt() {
            return Err(WorkError::InterruptContext);
        }
        if N == 0 || N > 4096 {
            return Err(WorkError::Unavailable);
        }
        let id = NEXT_QUEUE
            .try_update(Ordering::Relaxed, Ordering::Relaxed, |n| n.checked_add(1))
            .map_err(|_| WorkError::Exhausted)?;
        Ok(Arc::new(Self {
            id,
            abandoned: AtomicBool::new(false),
            #[cfg(target_arch = "x86_64")]
            executor_claimed: AtomicBool::new(false),
            #[cfg(target_arch = "x86_64")]
            active: ActiveContext::new(),
            cpu: ManuallyDrop::new(cpu),
            owner: ManuallyDrop::new(owner),
            state: ManuallyDrop::new(UnsafeCell::new(State::new())),
            held: AtomicBool::new(false),
            inspection: AtomicBool::new(false),
            #[cfg(target_arch = "x86_64")]
            completing: AtomicBool::new(false),
            retired: AtomicBool::new(false),
            #[cfg(target_arch = "x86_64")]
            published_stack_pins: AtomicUsize::new(0),
            #[cfg(target_arch = "x86_64")]
            execution_fence: Arc::new(accounting::Fence::new()),
            #[cfg(target_arch = "x86_64")]
            resource_busy: AtomicBool::new(false),
            #[cfg(target_arch = "x86_64")]
            resource_observed: AtomicBool::new(false),
        }))
    }
    fn lock(&self) -> Result<Guard<'_, N>, WorkError> {
        #[cfg(target_arch = "x86_64")]
        step_counts::add(&step_counts::STEP_COUNTS.locks, 1);
        let flags = self.cpu.save_disable()?;
        if self
            .held
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            // SAFETY: `flags` came from the save_disable above, and no lock is held.
            unsafe { self.cpu.restore(flags) };
            return Err(WorkError::Busy);
        }
        Ok(Guard { queue: self, flags })
    }
    fn process(&self) -> Result<(), WorkError> {
        self.cpu.check_cpu()?;
        if self.cpu.in_interrupt() {
            Err(WorkError::InterruptContext)
        } else {
            Ok(())
        }
    }
    /// Read the executing owned work context, never a Runtime/Task
    /// pointer. CPU/IRQ metadata is checked before active metadata or
    /// hardware RSP. Arbitrary dispatcher/ISR stacks never receive an ID.
    /// The one executor is permanently bound to this unique queue lifetime;
    /// retired/abandoned views refuse even if a consumer the Arc.
    #[cfg(target_arch = "x86_64")]
    pub fn current_id(&self) -> Result<ContextId, WorkError> {
        self.process()?;
        if self.retired.load(Ordering::Acquire) || self.abandoned.load(Ordering::Acquire) {
            return Err(WorkError::Closed);
        }
        let id = self.active.id.load(Ordering::Acquire);
        if id == 0 {
            return Err(WorkError::Unavailable);
        }
        let bounds =
            self.active.start.load(Ordering::Relaxed)..self.active.end.load(Ordering::Relaxed);
        if !bounds.contains(&context::stack_pointer()) {
            return Err(WorkError::Unavailable);
        }
        Ok(ContextId(id))
    }

    fn enqueue_lane(
        &self,
        lane: Lane,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<Ticket, WorkError> {
        self.enqueue_requirement(lane, callback, context, None)
    }
    fn enqueue_requirement(
        &self,
        lane: Lane,
        callback: Callback,
        context: *mut c_void,
        requested: Option<StackRequirement>,
    ) -> Result<Ticket, WorkError> {
        let mut guard = self.lock()?;
        let sequence = guard.state().enqueue(
            lane,
            callback,
            context,
            requested,
            self.abandoned.load(Ordering::Acquire),
        )?;
        Ok(Ticket {
            queue: self.id,
            sequence,
        })
    }
    /// IRQ-safe admission. It performs no callback/allocator/ref release.
    ///
    /// # Safety
    /// Callback/context stay valid until completion; the owner
    /// covers them and any code/resources they can reach. Accepted jobs may
    /// suspend on their real task stacks. They must not unwind, overflow the
    /// owned stack, return holding locks, or free the provider/executor. An
    /// error means this job was not admitted and will never be invoked.
    pub unsafe fn execute(
        &self,
        kind: Kind,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<Ticket, WorkError> {
        self.enqueue_lane(
            match kind {
                Kind::General => Lane::General,
                Kind::Notify => Lane::Notify,
            },
            callback,
            context,
        )
    }
    /// Queue General work with explicit geometry in a mixed-stack profile.
    /// This does not reserve an idle worker or implement thread creation.
    ///
    /// # Safety
    /// As for execute: callback/code/context remain held through
    /// completion or quarantine, and the body fits its requested usable extent.
    /// Rejection transfers nothing; a descriptor is no native/current permission.
    pub unsafe fn execute_general_with_requirement(
        &self,
        requirement: StackRequirement,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<Ticket, WorkError> {
        self.enqueue_requirement(Lane::General, callback, context, Some(requirement))
    }
    /// Claim one of the named policy's two portal continuations before
    /// accepting the callback/context. Full means neither idle worker is free;
    /// no request is hidden behind two blocked ordinary AML evaluations.
    ///
    /// # Safety
    /// The provider/request keeps callback, context and every reachable
    /// owner until completion or quarantine, as for [`Self::execute`].
    /// Rejection transfers nothing. This supplies no ACPICA or native entry grant.
    pub unsafe fn execute_portal(
        &self,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<Ticket, WorkError> {
        self.enqueue_lane(Lane::Portal, callback, context)
    }
    /// Admit serial GPE work into its independent reserved lane. Spare ordinary
    /// admission slots may hold pending GPE jobs; ordinary stacks never run them.
    ///
    /// # Safety
    /// The provider keeps the callback/context through completion or
    /// quarantine. On error it still owns them and no callback was admitted.
    pub unsafe fn execute_gpe(
        &self,
        callback: Callback,
        context: *mut c_void,
    ) -> Result<Ticket, WorkError> {
        self.enqueue_lane(Lane::Gpe, callback, context)
    }
    pub fn close(&self) -> Result<(), WorkError> {
        let mut g = self.lock()?;
        g.state().closed = true;
        Ok(())
    }

    pub fn completed(&self, ticket: Ticket) -> Result<bool, WorkError> {
        let mut guard = self.lock()?;
        if ticket.queue != self.id {
            return Err(WorkError::InvalidTicket);
        }
        guard.state().completed(ticket.sequence)
    }
    #[cfg(target_arch = "x86_64")]
    fn finish(
        &self,
        slot: usize,
        sequence: u64,
        released_stack: Option<ReleasedStackGeometry>,
    ) -> Result<(), WorkError> {
        self.lock()?
            .state()
            .finish_job(slot, sequence, released_stack)
    }
    /// # Safety
    /// External IRQ/driver/firmware/work producers have stopped and cannot
    /// concurrently enqueue or resume an executor. Keep the owner,
    /// CPU/process permission and backing through Runtime retirement.
    /// Jobs, active/suspended callbacks and reservation markers are
    /// checked refusal conditions: Pending/Closed does not retire or release
    /// them. Mere close is not producer synchronization, and queue retirement
    /// alone is not a task-stack or terminal C lifecycle proof.
    pub unsafe fn retire(&self) -> Result<(), WorkError> {
        self.process()?;
        let mut g = self.lock()?;
        let state = g.state();
        #[cfg(target_arch = "x86_64")]
        if !state.scheduling.quiet()
            || state.resources.attached()
            || self.resource_busy.load(Ordering::Acquire)
        {
            return Err(WorkError::Pending);
        }
        #[cfg(all(target_arch = "x86_64", feature = "linux-kernel"))]
        if state.inactive.iter().any(Option::is_some) {
            return Err(WorkError::Pending);
        }
        if !self.stack_pins_quiet()
            || !self.execution_quiet()
            || !state.closed
            || state.poisoned
            || self.abandoned.load(Ordering::Acquire)
            || state.jobs.iter().any(Option::is_some)
            || state.reservations.iter().any(|nonce| *nonce != 0)
        {
            return Err(WorkError::Pending);
        }
        #[cfg(target_arch = "x86_64")]
        {
            if self.active.id.load(Ordering::Acquire) != 0 {
                return Err(WorkError::Pending);
            }
            self.active.clear(); // Guard still holds the real CPU's IF clear.
        }
        self.retired.store(true, Ordering::Release);
        Ok(())
    }
}
impl<const N: usize> Drop for Queue<N> {
    fn drop(&mut self) {
        if self.retired.load(Ordering::Acquire) && self.stack_pins_quiet() && self.execution_quiet()
        {
            // SAFETY: retirement proved no producer or executor still uses these fields.
            unsafe {
                ManuallyDrop::drop(&mut self.state);
                ManuallyDrop::drop(&mut self.owner);
                ManuallyDrop::drop(&mut self.cpu)
            };
        } else {
            quarantine();
        }
    }
}
fn quarantine() {
    let _ = QUARANTINED.try_update(Ordering::Relaxed, Ordering::Relaxed, |n| {
        Some(n.saturating_add(1))
    });
}
pub fn quarantined_owners() -> u64 {
    QUARANTINED.load(Ordering::Relaxed)
}

/// A view of the real continuation, never permission to dereference
/// its stack. Active hooks receive RSP; idle detach receives saved RSP.
#[cfg(target_arch = "x86_64")]
#[repr(C)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct TaskContext {
    pub queue_id: u64,
    pub context_id: u64,
    pub sequence: u64,
    pub stack_start: usize,
    pub stack_end: usize,
    pub guard_start: usize,
    pub guard_end: usize,
    pub stack_pointer: usize,
}
#[cfg(target_arch = "x86_64")]
const _: () = {
    assert!(size_of::<TaskContext>() == 64);
    assert!(core::mem::offset_of!(TaskContext, sequence) == 16);
    assert!(core::mem::offset_of!(TaskContext, stack_pointer) == 56);
};
#[cfg(target_arch = "x86_64")]
#[repr(u32)]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum LeaveReason {
    Yield = 0,
    Blocked = 1,
    Reschedule = 2,
}
#[cfg(target_arch = "x86_64")]
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ParkOutcome {
    Skipped,
    Resumed,
}

/// Foreign execution state for one continuation.
///
/// # Safety
/// This object keeps every callback/code/context/root it reaches until real
/// quiescent detach and process-context final release. Hooks never unwind,
/// allocate, release owners, wait, reenter Runtime or enable hardware IRQs.
/// `attach`, `still_blocked`, `leave`, `arrive`, `complete` run on the
/// context stack, checked CPU, with IF=0 and no queue lock. `detach` instead
/// runs on its taskless dispatcher for an idle task; it never invents current.
/// Returned attach/leave/detach failure is transactional. `still_blocked` only
/// reads the condition and validates prepared suspension. Linux-owned
/// wrappers prepare/drain in process context BEFORE generic park/yield; hooks
/// never manufacture an owner exit or callback drain at IF=0. `leave` commits
/// only prepared state/lease suspension and clears foreign current. `arrive`
/// restores that state/current on the destination stack before IF restoration;
/// `new_job` starts a clean reused worker lease with its new existing ticket.
/// `complete` validates owner already EXITED/quiescent and clears current with
/// no drain/drop. Arrive/completion errors are fatal, never returned through an
/// inconsistent foreign context. Detach validates stopped producers and all
/// real foreign wait/task references; its final owner is released AFTER IF
/// restoration. TaskContext references are callback-scoped: copy primitive
/// metadata only, never keep the view pointer. Its ranges are observations,
/// not an independent stack/root lifetime lease.
#[cfg(target_arch = "x86_64")]
pub unsafe trait SwitchBinding: Send + Sync {
    /// # Safety
    /// Called on the current stack at IF0 with all binding roots kept.
    /// Preallocated noncyclic source field lease, prepared before attach. All
    /// fallible metric checks precede foreign attach; successful attach must
    /// claim/initialize this lease in its nofallible tail before returning Ok.
    /// Generic contexts may return None; a full source task must require Some.
    fn execution_publication(
        &self,
        _context: &TaskContext,
    ) -> Result<Option<Arc<dyn ExecutionPublication>>, WorkError> {
        Ok(None)
    }
    /// # Safety
    /// Called on the current stack at IF0 with all binding roots kept.
    unsafe fn attach(&self, context: &TaskContext) -> Result<(), WorkError>;
    /// # Safety
    /// Called after process PREPARE on the current stack at IF0; pure check.
    unsafe fn still_blocked(&self, context: &TaskContext) -> Result<bool, WorkError>;
    /// # Safety
    /// Called on the outgoing stack at IF0 with prepared state; transactional error.
    unsafe fn leave(&self, context: &TaskContext, reason: LeaveReason) -> Result<(), WorkError>;
    /// # Safety
    /// Called on the destination stack at IF0 before any hardware IF restore.
    unsafe fn arrive(&self, context: &TaskContext, new_job: bool) -> Result<(), WorkError>;
    /// # Safety
    /// Called on the completing stack at IF0 after foreign owner already exited.
    unsafe fn complete(&self, context: &TaskContext) -> Result<(), WorkError>;
    /// Commit an already-prepared original terminal departure on this dying
    /// current stack at IF0. No wait/allocator/Drop/reentry is allowed. Failure
    /// is fatal retention, never a return through original do_task_dead.
    /// # Safety
    /// Exact live native current, callback-scoped view, all roots retained.
    unsafe fn terminal_prepare(&self, _context: &TaskContext) -> Result<(), WorkError> {
        Err(WorkError::Unavailable)
    }
    /// Join that exact departure on the actual off-all-worker-stacks successor
    /// at IF0. Only bounded identity/state/notification metadata may change;
    /// original source reaper/RCU/resource disposal stays in a real process task.
    /// # Safety
    /// Core proved the actual architectural successor and retained this binding.
    unsafe fn terminal_successor(&self, _context: &TaskContext) -> Result<(), WorkError> {
        Err(WorkError::Unavailable)
    }
    /// # Safety
    /// Called on the idle taskless dispatcher at IF0 after foreign quiescence.
    unsafe fn detach(&self, context: &TaskContext) -> Result<(), WorkError>;
}

#[cfg(target_arch = "x86_64")]
struct WakeSignal {
    #[cfg(feature = "linux-kernel")]
    physical: inactive::PhysicalRun,
    service: bool,
    sequence: AtomicU64,
    pending: AtomicBool,
}
/// IRQ-safe kept read/admission view. No Runtime, Task, foreign task or
/// stack pointer is stored here. A completed job's handle never authorizes a
/// future job on the same continuation; stale views may outlive retired stacks.
#[cfg(target_arch = "x86_64")]
#[derive(Clone)]
pub struct WakeHandle<const N: usize> {
    queue: ManuallyDrop<Arc<Queue<N>>>,
    signal: ManuallyDrop<Arc<WakeSignal>>,
    id: ContextId,
    sequence: u64,
}
#[cfg(target_arch = "x86_64")]
impl<const N: usize> WakeHandle<N> {
    pub fn context_id(&self) -> ContextId {
        self.id
    }
    pub fn sequence(&self) -> u64 {
        self.sequence
    }
    pub(crate) fn closed(&self) -> Result<bool, WorkError> {
        self.queue.process()?;
        Ok(self.queue.retired.load(Ordering::Acquire)
            || self.queue.abandoned.load(Ordering::Acquire)
            || self.signal.sequence.load(Ordering::Acquire) != self.sequence)
    }
    /// Immutable constructor role, not a live task or entry grant.
    #[cfg(feature = "linux-kernel")]
    pub fn service_reserved(&self) -> Result<bool, WorkError> {
        self.queue.process()?;
        Ok(self.signal.service)
    }
    /// Checked same-CPU process/IRQ admission only; never resume inline or
    /// change the foreign condition. Close permits wakes for admitted jobs.
    /// Release/acquire pending plus final IF exclusion covers wake at park.
    /// Check process-context owner release on this queue's
    /// CPU, including after job close. This grants no current task/RSP, live
    /// sequence, Linux entry or native producer permission and signals nothing.
    #[cfg(feature = "linux-kernel")]
    pub fn check_process(&self) -> Result<(), WorkError> {
        self.queue.process()
    }
    pub fn wake(&self) -> Result<(), WorkError> {
        let mut guard = self.queue.lock()?;
        if guard.state().poisoned
            || self.queue.retired.load(Ordering::Acquire)
            || self.queue.abandoned.load(Ordering::Acquire)
            || self.signal.sequence.load(Ordering::Acquire) != self.sequence
        {
            return Err(WorkError::Closed);
        }
        // Real blocked->ready ordering is committed before wake publication;
        // duplicate/running wakes preserve equal-FIFO order. No C/task pointer.
        guard.state().scheduling.wake(self.id, self.sequence)?;
        #[cfg(feature = "linux-kernel")]
        inactive::notify(guard.state(), self.id, self.sequence)?;
        self.queue.inspection.store(true, Ordering::Release);
        self.signal.pending.store(true, Ordering::Release);
        Ok(())
    }
    /// Refresh only from this same executing continuation. Reused-job
    /// arrive can replace an old handle without a Runtime pointer. Runtime's
    /// active binding keeps Queue/signal, so such replacement decrements only
    /// nonfinal Arc references; it cannot release any owner/code/stack backing.
    pub fn renew_current(&self) -> Result<Self, WorkError> {
        if self.queue.current_id()? != self.id {
            return Err(WorkError::Unavailable);
        }
        let sequence = self.signal.sequence.load(Ordering::Acquire);
        if sequence == 0 {
            return Err(WorkError::Closed);
        }
        Ok(Self {
            queue: self.queue.clone(),
            signal: self.signal.clone(),
            id: self.id,
            sequence,
        })
    }
}

#[cfg(target_arch = "x86_64")]
impl<const N: usize> Drop for WakeHandle<N> {
    fn drop(&mut self) {
        if self.queue.process().is_ok() {
            // SAFETY: process context on the checked CPU. Runtime keeps
            // nonfinal refs during attach/arrive replacement; stale handles
            // may finally release an already-retired queue only here.
            unsafe {
                ManuallyDrop::drop(&mut self.signal);
                ManuallyDrop::drop(&mut self.queue);
            }
        } else {
            // Destruction is not IRQ/foreign-CPU reclamation permission. Keep
            // both allocations/roots rather than freeing a final owner there.
            quarantine();
        }
    }
}
#[cfg(target_arch = "x86_64")]
mod continuation;
#[cfg(target_arch = "x86_64")]
use continuation::{Continuation, TaskState};
#[cfg(target_arch = "x86_64")]
type Task = Continuation<Stack, u64, Arc<dyn SwitchBinding>>;
/// The most stacks an executor holds: the eight role stacks and its General
/// workers. A Linux kthread, a driver's ioctl worker and each thread a driver
/// such as NVK starts takes a General stack for its life.
pub const MAX_CONTINUATIONS: usize = 128;
/// The role stacks before the first General one: foreground, Notify, the
/// Linux service, firmware control, GPE, two portals and Notify.
pub const ROLE_STACKS: usize = 8;

/// CPU-affine pinned executor. Task zero is a real foreground continuation,
/// task one is serial Notify; an optional Linux service reserves task two and
/// firmware reserves task three. Remaining stacks run general work. The dispatcher
/// itself has no invented thread ID. All switching happens with IF=0.
#[cfg(target_arch = "x86_64")]
pub struct Runtime<const N: usize> {
    queue: Arc<Queue<N>>,
    layout: Layout,
    tasks: ManuallyDrop<UnsafeCell<Vec<Task>>>,
    dispatch: UnsafeCell<context::Frame>,
    current: Cell<usize>,
    cursor: Cell<usize>,
    retired: Cell<bool>,
    pending_reservations: Cell<usize>,
    owned_escrows: Cell<usize>,
    service_started: Cell<bool>,
    firmware_started: Cell<bool>,
    firmware_turn: Cell<bool>,
    firmware_cursor: Cell<usize>,
    service_domain: ManuallyDrop<UnsafeCell<Option<Arc<service::Domain<N>>>>>,
    execution: Arc<accounting::ExecutionView>,
    /// The dispatcher's caller's bound on its steps ([`Runtime::bound_steps`]).
    step_bound: Cell<Option<fn() -> bool>>,
    _local: PhantomData<*mut ()>,
    _pinned: PhantomPinned,
}
#[cfg(target_arch = "x86_64")]
impl<const N: usize> Runtime<N> {
    fn validate_stacks(queue: &Queue<N>, stacks: &[Stack]) -> Result<(), WorkError> {
        Self::validate_stack_limit(queue, stacks, 16)
    }
    fn validate_stack_limit(
        queue: &Queue<N>,
        stacks: &[Stack],
        limit: usize,
    ) -> Result<(), WorkError> {
        queue.process()?;
        if stacks.len() < 4 || stacks.len() > limit {
            return Err(WorkError::Unavailable);
        }
        for (i, a) in stacks.iter().enumerate() {
            for b in &stacks[..i] {
                if a.bounds.start < b.bounds.end && b.bounds.start < a.bounds.end {
                    return Err(WorkError::InvalidStack);
                }
                if a.backing.guard().start < b.bounds.end && b.bounds.start < a.backing.guard().end
                    || b.backing.guard().start < a.bounds.end
                        && a.bounds.start < b.backing.guard().end
                {
                    return Err(WorkError::InvalidStack);
                }
            }
        }
        let rsp = context::stack_pointer();
        if stacks.iter().any(|s| s.bounds.contains(&rsp)) {
            return Err(WorkError::InvalidStack);
        }
        Ok(())
    }
    fn new(queue: Arc<Queue<N>>, stacks: Vec<Stack>) -> Result<Pin<Box<Self>>, WorkError> {
        Self::validate_stacks(&queue, &stacks)?;
        // SAFETY: the executor runs on the checked native CPL0 CPU.
        let controls = unsafe { context::controls()? };
        Self::with_controls(queue, stacks, controls)
    }
    fn with_controls(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        controls: context::Controls,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        Self::with_service_controls(queue, stacks, controls, false)
    }
    fn with_service_controls(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        controls: context::Controls,
        reserve_service: bool,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        Self::with_reserved_controls(queue, stacks, controls, reserve_service, false)
    }
    fn with_reserved_controls(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        controls: context::Controls,
        reserve_service: bool,
        reserve_firmware: bool,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        let layout = match (reserve_service, reserve_firmware) {
            (false, false) => Layout::Ordinary,
            (true, false) => Layout::LinuxService,
            (true, true) => Layout::FirmwareControl,
            (false, true) => return Err(WorkError::Unavailable),
        };
        Self::with_layout_controls(queue, stacks, controls, layout)
    }
    fn with_layout_controls(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        controls: context::Controls,
        layout: Layout,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        Self::with_profile_controls(queue, stacks, controls, layout, None)
    }
    fn with_profile_controls(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        controls: context::Controls,
        layout: Layout,
        profile: Option<StackProfile>,
    ) -> Result<Pin<Box<Self>>, WorkError> {
        layout.validate(stacks.len(), N)?;
        let catalog = match profile {
            Some(profile) => stack_requirement::Catalog::from_actual(&stacks, profile, layout)?,
            None => stack_requirement::Catalog::empty(),
        };
        let mut tasks = Vec::new();
        tasks
            .try_reserve_exact(stacks.len())
            .map_err(|_| WorkError::Full)?;
        for (index, stack) in stacks.into_iter().enumerate() {
            tasks.push(Task::new(layout.lane(index), stack, controls, 0)?);
        }
        let execution = accounting::ExecutionView::new(&queue, &tasks)?;
        {
            let mut admission = queue.lock()?;
            let state = admission.state();
            // Native worker permits require an empty pre-admission queue: a
            // legacy Notify job in an ordinary slot otherwise bypasses the two
            // real-worker limit. Legacy constructors keep their prior contract.
            if state.reservations.iter().any(|nonce| *nonce != 0)
                || (layout.firmware_roles() && state.jobs.iter().any(Option::is_some))
                || state.jobs[N - layout.slots()..].iter().any(Option::is_some)
            {
                return Err(WorkError::Busy);
            }
            if queue
                .executor_claimed
                .compare_exchange(false, true, Ordering::AcqRel, Ordering::Acquire)
                .is_err()
            {
                return Err(WorkError::Busy);
            }
            // The real constructor atomically reserves its admission slots.
            // Existing queued jobs are never moved, discarded or reinterpreted.
            state.layout = layout;
            state.stack_profile = profile;
            state.stack_catalog = catalog;
        }
        let executor = Box::pin(Self {
            queue,
            execution,
            layout,
            tasks: ManuallyDrop::new(UnsafeCell::new(tasks)),
            dispatch: UnsafeCell::new(context::Frame::new(controls)),
            current: Cell::new(NONE),
            cursor: Cell::new(0),
            retired: Cell::new(false),
            pending_reservations: Cell::new(0),
            owned_escrows: Cell::new(0),
            service_started: Cell::new(false),
            firmware_started: Cell::new(false),
            firmware_turn: Cell::new(true),
            firmware_cursor: Cell::new(0),
            service_domain: ManuallyDrop::new(UnsafeCell::new(None)),
            step_bound: Cell::new(None),
            _local: PhantomData,
            _pinned: PhantomPinned,
        });
        let pointer = executor.as_ref().get_ref() as *const Self as usize;
        // SAFETY: the executor is not yet shared, and each frame starts on its own
        // stack.
        unsafe {
            for (i, task) in (&mut *executor.tasks.get()).iter_mut().enumerate() {
                task.frame
                    .start(task.stack.bounds.end, pointer, i, entry::<N>);
            }
        }
        Ok(executor)
    }
}
#[cfg(target_arch = "x86_64")]
unsafe extern "sysv64" fn entry<const N: usize>(pointer: usize, index: usize) -> ! {
    // SAFETY: covered by this function's own safety contract.
    let executor = unsafe { &*(pointer as *const Runtime<N>) };
    executor.valid();
    // SAFETY: the constructed trampoline just entered this kept
    // worker at IF0. The monotonic history changes only here, before hooks or
    // callbacks; an idle suspended entry frame can never masquerade as cold.
    unsafe { (*executor.task(index)).ever_entered = true };
    loop {
        executor.valid();
        // SAFETY: covered by this function's own safety contract.
        let (slot, job, flags) = unsafe {
            let t = &(*executor.task(index));
            let (slot, job) = t.job.expect("assigned task");
            (slot, job, t.flags)
        };
        executor.arrive(index, true);
        // SAFETY: covered by this function's own safety contract.
        unsafe { executor.queue.cpu.restore(flags) };
        // SAFETY: covered by this function's own safety contract.
        unsafe { (job.callback)(job.context as *mut c_void) };
        executor.valid();
        // SAFETY: this completing frame owns its immutable task index;
        // only its entered creator can alter the per-job escrow count. Peer
        // callbacks may complete while another creator keeps an owned escrow.
        let owned_escrows = unsafe { (*executor.task(index)).owned_escrows };
        if executor.pending_reservations.get() != 0 || owned_escrows != 0 {
            executor.queue.abandoned.store(true, Ordering::Release);
            quarantine();
            executor.queue.cpu.fatal(WorkError::Pending);
        }
        let _flags = match executor.queue.cpu.save_disable() {
            Ok(f) => f,
            Err(e) => executor.queue.cpu.fatal(e),
        };
        let now = executor
            .queue
            .scheduling_now()
            .unwrap_or_else(|error| executor.queue.cpu.fatal(error));
        executor
            .queue
            .lock()
            .and_then(|mut guard| guard.state().scheduling.account(now))
            .unwrap_or_else(|error| executor.queue.cpu.fatal(error));
        if let Err(error) = executor.execution_publish_at(index, now) {
            executor.queue.cpu.fatal(error);
        }
        executor.queue.completing.store(true, Ordering::Release);
        let view = executor.context_view(index, true);
        // SAFETY: completing task on its own stack at IF0.
        if let Some(binding) = unsafe { &(*executor.task(index)).binding } {
            // SAFETY: owner already exited; no lock/switch borrow spans this hook.
            let result = unsafe { binding.complete(&view) };
            if let Err(error) = result {
                executor.queue.cpu.fatal(error);
            }
        }
        executor.queue.completing.store(false, Ordering::Release);
        if let Err(error) = executor.execution_stop_at(index, now) {
            executor.queue.cpu.fatal(error);
        }

        let removed = (|| {
            executor
                .queue
                .lock()?
                .state()
                .scheduling
                .remove(ContextId(view.context_id), view.sequence)
        })();
        if let Err(error) = removed {
            executor.queue.cpu.fatal(error);
        }
        {
            // SAFETY: signal field; this borrow ends before later Task
            // mutation. Close this exact job so old wake handles remain inert.
            let signal = unsafe { &(*executor.task(index)).signal };
            signal.sequence.store(0, Ordering::Release);
        }
        // SAFETY: exact current constructor-owned Task at IF0. Only a
        // General stack WITHOUT a published C pin becomes reusable on return;
        // its actual architectural successor gates event delivery separately.
        let released_stack = unsafe {
            let task = &*executor.task(index);
            (task.lane == Lane::General
                && task.stack_pin_nonce == 0
                && !executor.execution.has_target(index))
            .then(|| ReleasedStackGeometry::of(&task.stack, !task.ever_entered))
        };
        if let Err(e) = executor.queue.finish(slot, job.sequence, released_stack) {
            executor.queue.cpu.fatal(e);
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            let t = &mut (*executor.task(index));
            t.job = None;
            t.completed_sequence = job.sequence;
            t.state = if t.stack_pin_nonce == 0 && !executor.execution.has_target(index) {
                TaskState::Idle
            } else {
                TaskState::Retained
            };
        }
        executor.current.set(NONE);
        executor.queue.active.clear();
        // SAFETY: covered by this function's own safety contract.
        unsafe {
            context::switch(
                core::ptr::addr_of_mut!((*executor.task(index)).frame),
                executor.dispatch.get(),
            )
        };
        // Next job resumes a real continuation on this same owned task stack.
    }
}
#[cfg(target_arch = "x86_64")]
impl<const N: usize> Drop for Runtime<N> {
    fn drop(&mut self) {
        self.execution.closed.store(true, Ordering::Release);
        if self.retired.get() && self.owned_escrows.get() == 0 && self.queue.stack_pins_quiet() {
            // SAFETY: retirement proved no task frame or stack is still live.
            unsafe { ManuallyDrop::drop(&mut self.tasks) };
            // SAFETY: successful retirement also required domain release.
            unsafe { ManuallyDrop::drop(&mut self.service_domain) };
        } else {
            quarantine();
        }
    }
}

#[cfg(target_arch = "x86_64")]
struct DrainFlag {
    task: *mut bool,
}
#[cfg(target_arch = "x86_64")]
impl Drop for DrainFlag {
    fn drop(&mut self) {
        // SAFETY: the flag belongs to a task the executor keeps past this drain.
        unsafe {
            self.task.write(false);
        }
    }
}

/// Sole owning constructor for a runtime; its bare pinned allocation is never
/// publicly returned. Construction and final release require process/deferred
/// context on the checked CPU, including the last Arc release of providers.
/// Owns the complete pinned runtime allocation. An uncertain Drop keeps the
/// allocation itself, not just stack pages containing pointers back to it.
#[cfg(target_arch = "x86_64")]
pub struct Executor<const N: usize> {
    runtime: ManuallyDrop<Pin<Box<Runtime<N>>>>,
}
#[cfg(target_arch = "x86_64")]
impl<const N: usize> Executor<N> {
    pub fn new(queue: Arc<Queue<N>>, stacks: Vec<Stack>) -> Result<Self, WorkError> {
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::new(queue, stacks)?),
        })
    }
    /// Reserve one service continuation in addition to Foreground,
    /// Notify and at least two General continuations. Start/bind the service
    /// explicitly before admitting dependent Linux operations.
    pub fn new_with_service(queue: Arc<Queue<N>>, stacks: Vec<Stack>) -> Result<Self, WorkError> {
        Runtime::<N>::validate_stacks(&queue, &stacks)?;
        if stacks.len() < 5 {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: same checked native construction boundary as new().
        let controls = unsafe { context::controls()? };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_service_controls(
                queue, stacks, controls, true,
            )?),
        })
    }
    /// Reserve independent Linux and firmware continuations at indices two
    /// and three, alongside Foreground, Notify and at least two General tasks.
    /// Reserve two queue admission slots as well (N must be at least three).
    /// Preexisting jobs in either final slot reject construction with Busy;
    /// no queued work moves or disappears. Guarded stacks and CPU ownership
    /// are validated as in new(). Starting grants no foreign service capability.
    pub fn new_with_services(queue: Arc<Queue<N>>, stacks: Vec<Stack>) -> Result<Self, WorkError> {
        Runtime::<N>::validate_stacks(&queue, &stacks)?;
        if stacks.len() < 6 {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: same checked native construction boundary as new().
        let controls = unsafe { context::controls()? };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_reserved_controls(
                queue, stacks, controls, true, true,
            )?),
        })
    }
    /// Apply a named bounded validation/prototype reservation policy. The layout reserves
    /// real workers and admission slots; it supplies no native IRQ/AML/provider
    /// capability. The queue must be empty (Busy otherwise), so every admitted
    /// Portal/Notify job first claims an idle-worker permit. Existing
    /// constructors keep their original layouts/contracts.
    pub fn new_with_policy(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        policy: ReservationPolicy,
    ) -> Result<Self, WorkError> {
        let layout = policy.layout();
        Runtime::<N>::validate_stack_limit(&queue, &stacks, layout.stack_limit())?;
        layout.validate(stacks.len(), N)?;
        // SAFETY: same checked native construction boundary as new().
        let controls = unsafe { context::controls()? };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_layout_controls(
                queue, stacks, controls, layout,
            )?),
        })
    }
    /// Opt into the mixed stack geometry of a protected firmware pool.
    /// Empty-queue validation is transactional. Descriptors supply no native
    /// mapping/current/kernel-thread/IRQ permission.
    pub fn new_with_profile(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        policy: ReservationPolicy,
        profile: StackProfile,
    ) -> Result<Self, WorkError> {
        let layout = policy.layout();
        Runtime::<N>::validate_stack_limit(&queue, &stacks, layout.stack_limit())?;
        layout.validate(stacks.len(), N)?;
        // SAFETY: same checked native construction boundary as new_with_policy.
        let controls = unsafe { context::controls()? };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_profile_controls(
                queue,
                stacks,
                controls,
                layout,
                Some(profile),
            )?),
        })
    }
    #[cfg(all(any(test, feature = "test-support"), unix))]
    pub fn new_hosted_profile_for_test(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        policy: ReservationPolicy,
        profile: StackProfile,
    ) -> Result<Self, WorkError> {
        let layout = policy.layout();
        Runtime::<N>::validate_stack_limit(&queue, &stacks, layout.stack_limit())?;
        layout.validate(stacks.len(), N)?;
        // SAFETY: unprivileged control snapshot and owned guarded mmap stacks only.
        let controls = unsafe { context::read_controls() };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_profile_controls(
                queue,
                stacks,
                controls,
                layout,
                Some(profile),
            )?),
        })
    }
    /// Hosted continuation mechanism test only. StackBacking must own
    /// real mapped guarded stacks; Cpu is an explicit hosted model. This never
    /// checks CPL0/IRQ/native roots or registers a native producer.
    #[cfg(all(any(test, feature = "test-support"), unix))]
    pub fn new_hosted_for_test(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
    ) -> Result<Self, WorkError> {
        Runtime::<N>::validate_stacks(&queue, &stacks)?;
        // SAFETY: unprivileged read only, used solely by hosted stack tests.
        let controls = unsafe { context::read_controls() };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_controls(queue, stacks, controls)?),
        })
    }
    #[cfg(all(any(test, feature = "test-support"), feature = "linux-kernel"))]
    pub fn new_hosted_service_for_test(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
    ) -> Result<Self, WorkError> {
        Runtime::<N>::validate_stacks(&queue, &stacks)?;
        // SAFETY: hosted stack mechanism only, as new_hosted_for_test.
        let controls = unsafe { context::read_controls() };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_service_controls(
                queue, stacks, controls, true,
            )?),
        })
    }
    #[cfg(all(any(test, feature = "test-support"), unix))]
    pub fn new_hosted_policy_for_test(
        queue: Arc<Queue<N>>,
        stacks: Vec<Stack>,
        policy: ReservationPolicy,
    ) -> Result<Self, WorkError> {
        let layout = policy.layout();
        Runtime::<N>::validate_stack_limit(&queue, &stacks, layout.stack_limit())?;
        layout.validate(stacks.len(), N)?;
        // SAFETY: unprivileged real control snapshot for owned guarded hosted stacks only.
        let controls = unsafe { context::read_controls() };
        Ok(Self {
            runtime: ManuallyDrop::new(Runtime::with_layout_controls(
                queue, stacks, controls, layout,
            )?),
        })
    }
    /// The returned pinned runtime is what a native provider
    /// binds into its context. Do not keep a pointer to this movable wrapper.
    pub fn runtime(&self) -> Pin<&Runtime<N>> {
        self.runtime.as_ref()
    }
}
#[cfg(target_arch = "x86_64")]
impl<const N: usize> Drop for Executor<N> {
    fn drop(&mut self) {
        let process = self.runtime.queue.process().is_ok();
        if process
            && self.runtime.retired.get()
            && self.runtime.owned_escrows.get() == 0
            && self.runtime.queue.stack_pins_quiet()
        {
            // SAFETY: the runtime retired, so no task or provider can resume it.
            unsafe {
                ManuallyDrop::drop(&mut self.runtime);
            }
        } else {
            self.runtime.queue.abandoned.store(true, Ordering::Release);
            if process && let Ok(flags) = self.runtime.queue.cpu.save_disable() {
                self.runtime.queue.active.clear();
                // SAFETY: `flags` came from the save_disable just above.
                unsafe { self.runtime.queue.cpu.restore(flags) };
            }
            // If affinity/context is unchecked, closing is atomic and the
            // complete allocation remains held; no foreign CPU mutates
            // active metadata or releases process-affine owners.
            quarantine();
            // Keep the Runtime allocation, queue/provider owners, task
            // frames and real stack/root leases even after a timeout/failed drain.
        }
    }
}

#[cfg(test)]
mod tests;
