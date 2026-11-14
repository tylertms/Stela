//! Linux task metadata on kept cooperative continuations.
//!
//! This binds original C task_struct/current and the entered service lease.
//! The normal/WW and original RT mutex adapters supply wake/signal/I/O effects.
//! NORMAL nice, FIFO and inherited ranks commit against retained job identities
//! on the same checked queue; native IRQ delivery remains the service's owner.
mod dispatch;
mod factory;
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
mod hosted;
mod inactive;
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
pub(crate) use hosted::service_progress as hosted_service_progress;
#[cfg(all(
    feature = "hosted-runtime",
    target_os = "linux",
    target_arch = "x86_64"
))]
pub use hosted::{
    HostedStackReservation, HostedTaskBinding, initialize_hosted_resources,
    reserve_hosted_child_stack, start_hosted_threads, step_hosted,
};
mod native;
pub mod runtime;
use alloc::sync::Arc;
mod data;
use core::{
    cell::UnsafeCell,
    ffi::c_void,
    mem::{ManuallyDrop, MaybeUninit},
    pin::Pin,
    ptr::NonNull,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
pub(crate) use data::StoppedTaskData;
use stela_driver_base::platform::work::scheduling::{BasePolicy, Snapshot};
use stela_driver_base::platform::work::{
    ContextId, LeaveReason, ParkOutcome, Runtime, SwitchBinding, TaskContext, WakeHandle, WorkError,
};

#[repr(C)]
struct Rank {
    policy: u32,
    realtime: u32,
    effective: i32,
    nice: i32,
}

impl Rank {
    fn snapshot(&self) -> Result<Snapshot, WorkError> {
        let base = match (self.policy, self.realtime) {
            (0, 0) if self.nice == 0 => BasePolicy::Normal,
            (0, 0) if (-20..=19).contains(&self.nice) => BasePolicy::NormalNice(self.nice as i8),
            (1, priority @ 1..=99) => BasePolicy::Fifo(priority as u8),
            _ => return Err(WorkError::Unavailable),
        };
        if !(-20..=19).contains(&self.nice) {
            return Err(WorkError::Unavailable);
        }
        Ok(Snapshot {
            base,
            effective: self
                .effective
                .try_into()
                .map_err(|_| WorkError::Unavailable)?,
        })
    }
}

#[repr(C)]
struct TaskOps {
    is_current: unsafe extern "C" fn(*mut c_void, *const TaskContext) -> bool,
    wake_target: unsafe extern "C" fn(*mut c_void, u64) -> i32,
    service_reserved: unsafe extern "C" fn(*mut c_void) -> bool,
    rank: unsafe extern "C" fn(*mut c_void, u64, *const Rank, *const Rank, bool) -> i32,
    retain_inactive: Option<unsafe extern "C" fn(*mut c_void, u64, *mut *mut c_void) -> i32>,
}
#[repr(C)]
struct ScopeOps {
    park: unsafe extern "C" fn(*mut c_void, bool) -> i32,
    yield_now: unsafe extern "C" fn(*mut c_void) -> i32,
    reschedule: unsafe extern "C" fn(*mut c_void) -> i32,
    reschedule_requested: unsafe extern "C" fn(*mut c_void) -> bool,
    terminate: unsafe extern "C" fn(*mut c_void) -> !,
    observe_inactive: Option<unsafe extern "C" fn(*mut c_void, *mut c_void, bool) -> i32>,
    end_inactive: Option<unsafe extern "C" fn(*mut c_void, *mut c_void) -> i32>,
    account_current: Option<unsafe extern "C" fn(*mut c_void, *mut u64) -> i32>,
}
const _: () = {
    assert!(core::mem::size_of::<TaskContext>() == 64);
    assert!(core::mem::size_of::<TaskOps>() == 40);
    assert!(core::mem::offset_of!(TaskOps, is_current) == 0);
    assert!(core::mem::offset_of!(TaskOps, wake_target) == 8);
    assert!(core::mem::offset_of!(TaskOps, service_reserved) == 16);
    assert!(core::mem::offset_of!(TaskOps, rank) == 24);
    assert!(core::mem::offset_of!(TaskOps, retain_inactive) == 32);
    assert!(core::mem::size_of::<ScopeOps>() == 64);
    assert!(core::mem::offset_of!(ScopeOps, park) == 0);
    assert!(core::mem::offset_of!(ScopeOps, yield_now) == 8);
    assert!(core::mem::offset_of!(ScopeOps, reschedule) == 16);
    assert!(core::mem::offset_of!(ScopeOps, reschedule_requested) == 24);
    assert!(core::mem::offset_of!(ScopeOps, terminate) == 32);
    assert!(core::mem::offset_of!(ScopeOps, observe_inactive) == 40);
    assert!(core::mem::offset_of!(ScopeOps, end_inactive) == 48);
    assert!(core::mem::offset_of!(ScopeOps, account_current) == 56);
};
#[repr(C)]
struct CTask {
    _opaque: [u8; 0],
}
/// Opaque synchronous birth from the actual original allocator source. Only
/// the native source callback receives it; no definition/raw-task constructor.
#[repr(C)]
pub(crate) struct OriginalBirth {
    _opaque: [u8; 0],
}
unsafe extern "C" {
    fn stela_linux_task_catalog_capacity() -> u32;
    fn stela_linux_task_stack_bytes() -> usize;
    fn stela_linux_task_stack_alignment() -> usize;
    fn stela_linux_task_stack_guard() -> usize;
    fn stela_linux_task_allocate() -> *mut CTask;
    fn stela_linux_task_initial_allocate(
        owner: *const crate::owner::RawOwner,
        pgd: *mut c_void,
        result: *mut *mut CTask,
    ) -> i32;
    fn stela_linux_task_child_allocate(
        owner: *const crate::owner::RawOwner,
        birth: *mut OriginalBirth,
        result: *mut *mut CTask,
    ) -> i32;
    fn stela_linux_task_initialize_resources(
        task: *mut CTask,
        owner: *const crate::owner::RawOwner,
    ) -> i32;
    fn stela_linux_task_start_threads(
        task: *mut CTask,
        owner: *const crate::owner::RawOwner,
    ) -> i32;
    fn stela_linux_task_reap_source(
        task: *mut CTask,
        owner: *const crate::owner::RawOwner,
        generation: u64,
    ) -> i32;
    fn stela_linux_task_join_source(
        task: *mut CTask,
        owner: *const crate::owner::RawOwner,
        generation: u64,
    ) -> i32;
    fn stela_linux_task_release_original(
        task: *mut CTask,
        owner: *const crate::owner::RawOwner,
        generation: u64,
    ) -> i32;
    fn stela_linux_task_attach(
        task: *mut CTask,
        context: *const TaskContext,
        ops: *const TaskOps,
        binding: *mut c_void,
    ) -> i32;
    fn stela_linux_task_still_blocked(task: *mut CTask, context: *const TaskContext) -> i32;
    fn stela_linux_task_leave(task: *mut CTask, context: *const TaskContext, reason: u32) -> i32;
    fn stela_linux_task_arrive(task: *mut CTask, context: *const TaskContext, new_job: bool)
    -> i32;
    fn stela_linux_task_complete(task: *mut CTask, context: *const TaskContext) -> i32;
    fn stela_linux_task_terminal_prepare(task: *mut CTask, context: *const TaskContext) -> i32;
    fn stela_linux_task_terminal_successor(task: *mut CTask, context: *const TaskContext) -> i32;
    fn stela_linux_task_detach(task: *mut CTask, context: *const TaskContext) -> i32;
    fn stela_linux_task_release(task: *mut CTask);
    fn stela_linux_task_scope_begin(
        task: *mut CTask,
        ops: *const ScopeOps,
        scope: *mut c_void,
    ) -> i32;
    fn stela_linux_task_scope_end(task: *mut CTask) -> i32;
    fn stela_linux_task_signal(task: *mut CTask, expected_sequence: u64, kind: u32) -> i32;
}

/// Geometry from the compiled Linux configuration, for resource planning.
/// It does not attest mappings, CPU/current permission or a native provider.
pub fn stack_requirement() -> Result<stela_driver_base::platform::work::StackRequirement, WorkError>
{
    // SAFETY: these three C functions return immutable compiled constants only;
    // they inspect no task, entered owner, mutable catalog or native pointer.
    let (bytes, alignment, guard) = unsafe {
        (
            stela_linux_task_stack_bytes(),
            stela_linux_task_stack_alignment(),
            stela_linux_task_stack_guard(),
        )
    };
    stela_driver_base::platform::work::StackRequirement::exact(bytes, alignment, guard)
}

static QUARANTINED: AtomicU64 = AtomicU64::new(0);

/// Metadata kept because its final release was not explicitly checked.
/// This counts no native/GPU validation and never reclaims uncertain tasks.
pub fn quarantined() -> u64 {
    QUARANTINED.load(Ordering::Relaxed)
}

/// A refused/partial initial claim retains the actual root allocation/code.
/// No reset, C initial-object finalization or mount-lifetime release is proved.
pub struct InitialFailure {
    pub error: ResourceBootstrapError,
    _data_backing: Option<ManuallyDrop<Arc<dyn crate::threads::TaskDataBacking>>>,
    _data_preparation: Option<data::PreparationFailure>,
    _root: ManuallyDrop<Arc<dyn crate::threads::OriginalKernelRoot>>,
    _execution: ManuallyDrop<crate::kernel::ExecutionPin>,
}
/// Failure retains original callback/code/root scope; this is NOT a native
/// cancellation/no-publication receipt and cannot authorize source cleanup.
pub(crate) struct ChildMetadataFailure {
    _error: ResourceBootstrapError,
    _data_backing: Option<ManuallyDrop<Arc<dyn crate::threads::TaskDataBacking>>>,
    _data_preparation: Option<data::PreparationFailure>,
    _execution: ManuallyDrop<crate::kernel::ExecutionPin>,
}

/// Original C initialization status, not a completion/admission receipt.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResourceBootstrapError {
    Work(WorkError),
    Native(i32),
}

/// Keeps the C task independently of a per-enter borrowed service owner.
/// The first immutable waker validates CPU/IRQ/RSP before C state access;
/// renew_current obtains the existing job ticket on stack reuse.
pub struct TaskBinding<const N: usize> {
    task: NonNull<CTask>,
    first: UnsafeCell<MaybeUninit<WakeHandle<N>>>,
    target: UnsafeCell<MaybeUninit<WakeHandle<N>>>,
    published: AtomicBool,
    detached: AtomicBool,
    released: bool,
    initial_root: Option<ManuallyDrop<Arc<dyn crate::threads::OriginalKernelRoot>>>,
    _source_execution: Option<ManuallyDrop<crate::kernel::ExecutionPin>>,
    source_data: Option<Arc<data::SourceData>>,
}
// SAFETY: no foreign CPU mutates the C task or first waker. Runtime
// identity is checked before C entry; publication is single-writer release/
// acquire and the immutable waker is held until safe detach/release.
unsafe impl<const N: usize> Send for TaskBinding<N> {}
// SAFETY: same single checked CPU contract; IRQ wake views contain no C task.
unsafe impl<const N: usize> Sync for TaskBinding<N> {}

fn status(value: i32) -> Result<(), WorkError> {
    match value {
        0 => Ok(()),
        -16 => Err(WorkError::Busy),
        -28 => Err(WorkError::Full),
        _ => Err(WorkError::corrupt()),
    }
}
impl<const N: usize> TaskBinding<N> {
    const OPS: TaskOps = TaskOps {
        is_current: Self::current,
        wake_target: Self::wake_target,
        service_reserved: Self::service_reserved,
        rank: Self::rank,
        retain_inactive: Some(Self::retain_inactive),
    };
    const SCOPE_OPS: ScopeOps = ScopeOps {
        park: Self::park,
        yield_now: Self::yield_now,
        reschedule: Self::reschedule,
        reschedule_requested: Self::reschedule_requested,
        terminate: Self::terminate,
        observe_inactive: Some(Self::observe_inactive),
        end_inactive: Some(Self::end_inactive),
        account_current: Some(Self::account_current),
    };

    /// Allocate original task metadata before entering a Linux owner.
    ///
    /// # Safety
    /// Call in checked process context on the Runtime CPU, with the canonical
    /// heap initialized. Every binding using the common !SMP C archive must stay
    /// on that SAME checked physical CPU for its lifetime; keep all
    /// CPU/code/root permission separately. No native producer is established
    /// by this allocation. C metadata failure is fallible; Arc allocation uses
    /// the image's existing global allocation-error policy.
    pub unsafe fn new() -> Result<Arc<Self>, WorkError> {
        // SAFETY: immutable compiled C storage-ceiling observation, with no task/owner access.
        if unsafe { stela_linux_task_catalog_capacity() } as usize
            != stela_driver_base::platform::work::MAX_CONTINUATIONS
        {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: caller supplies canonical heap/process permission.
        let task = NonNull::new(unsafe { stela_linux_task_allocate() }).ok_or(WorkError::Full)?;
        Ok(Arc::new(Self {
            task,
            first: UnsafeCell::new(MaybeUninit::uninit()),
            target: UnsafeCell::new(MaybeUninit::uninit()),
            published: AtomicBool::new(false),
            detached: AtomicBool::new(false),
            released: false,
            initial_root: None,
            _source_execution: None,
            source_data: None,
        }))
    }

    /// Claim the exact original init_task and its retained native initial-mm
    /// root. This is explicit pre-lifecycle identity, not initialized resource,
    /// supervisor/kthreadd, SRCU or original-kthread readiness.
    ///
    /// # Safety
    /// Cold canonical KernelOwner, before any previous task/WQ/body admission.
    /// Actual native CPU/root/code and fixed canonical metadata heap are kept;
    /// heap allocation here is a bounded native leaf with no root change,
    /// callback reentry, continuation switch or original resource wait. The
    /// caller keeps this kernel for the mount, including failure. Root backing
    /// comes from the exact original root owner, never a numeric CR3/address.
    /// Bind this record only to the actual reserved persistent Service job;
    /// it cannot complete/detach/release as a bare reusable continuation.
    #[expect(
        clippy::result_large_err,
        reason = "startup refusal returns retained ownership without another allocation"
    )]
    pub unsafe fn new_initial<P: crate::threads::KernelThreadPlatform>(
        kernel: &crate::kernel::KernelOwner<P, N>,
        root: Arc<dyn crate::threads::OriginalKernelRoot>,
    ) -> Result<Arc<Self>, InitialFailure> {
        let root = ManuallyDrop::new(root);
        let execution = ManuallyDrop::new(kernel.execution());
        let backing = match kernel.platform().task_data_backing() {
            Ok(backing) => ManuallyDrop::new(backing),
            Err(error) => {
                return Err(InitialFailure {
                    error: ResourceBootstrapError::Work(error),
                    _data_backing: None,
                    _data_preparation: None,
                    _root: root,
                    _execution: execution,
                });
            }
        };
        let mut data_preparation = ManuallyDrop::new(None);
        // SAFETY: one fresh actual segment, bounded source metadata only;
        // actual backing is kept before every C claim/accepted projection.
        let result = unsafe {
            kernel.platform().with_process(|process| {
                let pgd = root.pgd(process).map_err(ResourceBootstrapError::Work)?;
                let mut raw = core::ptr::null_mut();
                let value = stela_linux_task_initial_allocate(kernel.raw(), pgd.as_ptr(), &mut raw);
                if value != 0 {
                    return Err(ResourceBootstrapError::Native(value));
                }
                let task = NonNull::new(raw)
                    .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
                let source_data = match data::SourceData::prepare(
                    task,
                    kernel.raw(),
                    crate::threads::TaskDataStorage::InitialImage,
                    &backing,
                ) {
                    Ok(data) => data,
                    Err(failure) => {
                        let error = failure.error;
                        *data_preparation = Some(failure);
                        return Err(error);
                    }
                };
                Ok((task, source_data))
            })
        };
        let result = match result {
            Ok(result) => result,
            Err(error) => Err(ResourceBootstrapError::Work(error)),
        };
        let (task, source_data) = match result {
            Ok(pair) => pair,
            Err(error) => {
                return Err(InitialFailure {
                    error,
                    _data_backing: Some(backing),
                    _data_preparation: ManuallyDrop::into_inner(data_preparation),
                    _root: root,
                    _execution: execution,
                });
            }
        };
        drop(ManuallyDrop::into_inner(backing));
        Ok(Arc::new(Self {
            task,
            first: UnsafeCell::new(MaybeUninit::uninit()),
            target: UnsafeCell::new(MaybeUninit::uninit()),
            published: AtomicBool::new(false),
            detached: AtomicBool::new(false),
            released: false,
            initial_root: Some(root),
            _source_execution: Some(execution),
            source_data: Some(source_data),
        }))
    }

    /// Consume one opaque actual original allocation birth into metadata.
    /// It grants no child/current, native stack or runnable publication.
    ///
    /// # Safety
    /// Called only by this kernel's original dup_task_struct stack-allocation
    /// callback, with the synchronous genuine source birth pointer. Never
    /// fabricate/retain that constructor-frame pointer, pass an arbitrary task
    /// descriptor or enter bare. Caller keeps source/kernel and native Runtime
    /// mount control; one bounded metadata operation only, no resource waits.
    /// Native refusal/cancellation/exposure ownership remains independently
    /// required before original C can return an allocation/fork error.
    pub(crate) unsafe fn new_child<P: crate::threads::KernelThreadPlatform>(
        kernel: &crate::kernel::KernelOwner<P, N>,
        birth: NonNull<OriginalBirth>,
    ) -> Result<Arc<Self>, ChildMetadataFailure> {
        let execution = ManuallyDrop::new(kernel.execution());
        let backing = match kernel.platform().task_data_backing() {
            Ok(backing) => ManuallyDrop::new(backing),
            Err(error) => {
                return Err(ChildMetadataFailure {
                    _error: ResourceBootstrapError::Work(error),
                    _data_backing: None,
                    _data_preparation: None,
                    _execution: execution,
                });
            }
        };
        let mut data_preparation = ManuallyDrop::new(None);
        // SAFETY: one fresh actual segment, bounded source metadata only;
        // actual backing is kept before every C claim/accepted projection.
        let result = unsafe {
            kernel.platform().with_process(|_process| {
                let mut raw = core::ptr::null_mut();
                let value = stela_linux_task_child_allocate(kernel.raw(), birth.as_ptr(), &mut raw);
                if value != 0 {
                    return Err(ResourceBootstrapError::Native(value));
                }
                let task = NonNull::new(raw)
                    .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
                let source_data = match data::SourceData::prepare(
                    task,
                    kernel.raw(),
                    crate::threads::TaskDataStorage::AllocatedTask,
                    &backing,
                ) {
                    Ok(data) => data,
                    Err(failure) => {
                        let error = failure.error;
                        *data_preparation = Some(failure);
                        return Err(error);
                    }
                };
                Ok((task, source_data))
            })
        };
        let result = match result {
            Ok(result) => result,
            Err(error) => Err(ResourceBootstrapError::Work(error)),
        };
        let (task, source_data) = match result {
            Ok(pair) => pair,
            Err(error) => {
                return Err(ChildMetadataFailure {
                    _error: error,
                    _data_backing: Some(backing),
                    _data_preparation: ManuallyDrop::into_inner(data_preparation),
                    _execution: execution,
                });
            }
        };
        drop(ManuallyDrop::into_inner(backing));
        Ok(Arc::new(Self {
            task,
            first: UnsafeCell::new(MaybeUninit::uninit()),
            target: UnsafeCell::new(MaybeUninit::uninit()),
            published: AtomicBool::new(false),
            detached: AtomicBool::new(false),
            released: false,
            initial_root: None,
            _source_execution: Some(execution),
            source_data: Some(source_data),
        }))
    }

    /// Private structural observation for the canonical startup owner only.
    /// The actual C claim, native Service-only bind and source current checks
    /// independently authenticate every operation; this is no current grant.
    pub(crate) fn original_initial(&self) -> bool {
        self.initial_root.is_some() && self._source_execution.is_some()
    }

    /// Execute selected original resource roots on this actual original
    /// Initial Service task. This is NOT full runtime/workqueue/SRCU Ready.
    ///
    /// # Safety
    /// Inside this binding's real run_scoped canonical owner, after permanent
    /// native heap/image/canonical page catalogue/CPU topology/time/timer/IRQ
    /// metadata producers are genuinely installed and retained. No transient
    /// EnteredProcess/RefCell borrow or native queue lock may span original C
    /// resource code: it may allocate, synchronize or park. Source providers
    /// must be the exact original helpers, not absent-symbol success stubs.
    /// Any partial refusal/unwind retains this binding/kernel/root/code graph;
    /// original global initialization is one-shot and cannot be retried/reset.
    pub unsafe fn initialize_resources<P: crate::threads::KernelThreadPlatform>(
        &self,
        kernel: &crate::kernel::KernelOwner<P, N>,
    ) -> Result<(), ResourceBootstrapError> {
        if self.initial_root.is_none() {
            return Err(ResourceBootstrapError::Work(WorkError::Unavailable));
        }
        // SAFETY: caller provides this actual Initial current/scope and all
        // real native roots. C independently authenticates exact binding and
        // canonical owner before any original helper or phase publication.
        let value =
            unsafe { stela_linux_task_initialize_resources(self.task.as_ptr(), kernel.raw()) };
        if value == 0 {
            Ok(())
        } else {
            Err(ResourceBootstrapError::Native(value))
        }
    }

    /// Create genuine source PID1 supervisor and PID2 kthreadd through original
    /// copy_process and the SAME native child factory. Publication is distinct
    /// from their actual execution, original WQ progress or joined FullReady.
    /// # Safety
    /// This exact Initial is current in its retained source scope after real
    /// initialize_resources. The actual native scheduler Domain/fork factory,
    /// original root/provider/code graph and permanent supervisor request owner
    /// are installed before calling. No transient EnteredProcess/RefCell/queue
    /// borrow may span original clone/completion code or resource waits.
    pub unsafe fn start_threads<P: crate::threads::KernelThreadPlatform>(
        &self,
        kernel: &crate::kernel::KernelOwner<P, N>,
    ) -> Result<(), ResourceBootstrapError> {
        if self.initial_root.is_none() {
            return Err(ResourceBootstrapError::Work(WorkError::Unavailable));
        }
        // SAFETY: caller supplies genuine Initial and installed typed producer;
        // C independently verifies source current before original effects.
        match unsafe { stela_linux_task_start_threads(self.task.as_ptr(), kernel.raw()) } {
            0 => Ok(()),
            value => Err(ResourceBootstrapError::Native(value)),
        }
    }

    /// # Safety
    /// Private published-child caller checked this EXACT native/page owner's
    /// genuine physical-return receipt; source supervisor is a different real
    /// original current. No transient native permit or RefCell borrow spans
    /// this potentially waiting C source disposal. Observer callbacks only
    /// record/wake and may not borrow/mutate the owned Child being disposed.
    unsafe fn reap_source<P: crate::threads::KernelThreadPlatform>(
        &self,
        kernel: &crate::kernel::KernelOwner<P, N>,
        generation: u64,
    ) -> Result<(), ResourceBootstrapError> {
        // SAFETY: exact actual native receipt and real source context above.
        match unsafe { stela_linux_task_reap_source(self.task.as_ptr(), kernel.raw(), generation) }
        {
            0 => Ok(()),
            value => Err(ResourceBootstrapError::Native(value)),
        }
    }
    /// # Safety
    /// Source scheduler references released through reap_source, actual source
    /// supervisor/current, no transient native borrow over original rcu_barrier.
    /// All original producer/code owners remain retained. EAGAIN means actual
    /// external source refs still exist; it grants no descriptor/pin retirement.
    unsafe fn join_source<P: crate::threads::KernelThreadPlatform>(
        &self,
        kernel: &crate::kernel::KernelOwner<P, N>,
        generation: u64,
    ) -> Result<(), ResourceBootstrapError> {
        // SAFETY: caller provides true source context and exact retained record.
        match unsafe { stela_linux_task_join_source(self.task.as_ptr(), kernel.raw(), generation) }
        {
            0 => Ok(()),
            value => Err(ResourceBootstrapError::Native(value)),
        }
    }

    /// # Safety
    /// Consuming caller owns the private actual native/page completed-cancel or
    /// final-release receipt, and has joined every C/native callback tail,
    /// producer/target and binding. Actual original source current now owns this
    /// final bounded metadata free; no physical stack permission is inferred.
    unsafe fn release_original<P: crate::threads::KernelThreadPlatform>(
        self: Arc<Self>,
        kernel: &crate::kernel::KernelOwner<P, N>,
        generation: u64,
    ) -> Result<(), Arc<Self>> {
        if self.initial_root.is_some()
            || self._source_execution.is_none()
            || (self.published.load(Ordering::Acquire) && !self.detached.load(Ordering::Acquire))
        {
            return Err(self);
        }
        if let Some(data) = &self.source_data {
            if !self.published.load(Ordering::Acquire) {
                // SAFETY: existing private affine NoPublication caller receipt,
                // not !published/Arc counts, is the actual native proof.
                if unsafe { data.close_unpublished(kernel.raw()) }.is_err() {
                    return Err(self);
                }
            } else if !data.is_finished() {
                return Err(self); // real stopped/source-ref tails still owed
            }
        }
        let mut this = Arc::try_unwrap(self)?;
        // SAFETY: unique metadata + caller's actual affine release/closure
        // receipt; C independently refuses every live source/callback phase.
        let value = unsafe {
            stela_linux_task_release_original(this.task.as_ptr(), kernel.raw(), generation)
        };
        if value != 0 {
            return Err(Arc::new(this));
        }
        this.released = true;
        // Free completed C metadata BEFORE releasing its actual same-Queue
        // executable/root retention; KernelOwner itself remains mounted.
        if let Some(mut execution) = this._source_execution.take() {
            // SAFETY: real producer/source/native tails and C free returned.
            unsafe { ManuallyDrop::drop(&mut execution) };
        }
        Ok(())
    }

    /// Attach only to this current continuation, before Linux entry.
    /// Initial waker publication completes before any original C task service.
    pub fn bind_current(self: &Arc<Self>, runtime: Pin<&Runtime<N>>) -> Result<(), WorkError> {
        if self.published.load(Ordering::Acquire) {
            return Err(WorkError::Busy);
        }
        if self.initial_root.is_some() && !runtime.service_current()? {
            return Err(WorkError::Unavailable);
        }
        let handle = runtime.bind_current(self.clone())?;
        // SAFETY: one successful Runtime attachment owns this same-CPU initial
        // publication. Runtime keeps this binding before returning the handle.
        unsafe {
            (*self.first.get()).write(handle.clone());
            (*self.target.get()).write(handle);
        };
        self.published.store(true, Ordering::Release);
        Ok(())
    }

    fn first(&self) -> Result<&WakeHandle<N>, WorkError> {
        if !self.published.load(Ordering::Acquire) || self.detached.load(Ordering::Acquire) {
            return Err(WorkError::Unavailable);
        }
        // SAFETY: acquired once-only initialization, immutable through detach.
        Ok(unsafe { (*self.first.get()).assume_init_ref() })
    }
    pub fn context_id(&self) -> Result<ContextId, WorkError> {
        Ok(self.first()?.context_id())
    }
    /// Real current job's immutable wake view. An older IRQ copy remains closed
    /// after completion and cannot authorize a later job on the reused stack.
    pub fn current_wake(&self) -> Result<WakeHandle<N>, WorkError> {
        self.first()?.renew_current()
    }
    /// Capture this current job, keeping metadata independently of
    /// original C usage refs. An old target may outlive clean job completion;
    /// its expected sequence can never signal or inspect a future job.
    pub fn target_current(self: &Arc<Self>) -> Result<TargetTask<N>, WorkError> {
        let wake = self.current_wake()?;
        Ok(TargetTask {
            binding: ManuallyDrop::new(self.clone()),
            wake: ManuallyDrop::new(wake),
        })
    }

    unsafe extern "C" fn wake_target(binding: *mut c_void, expected: u64) -> i32 {
        // SAFETY: C validated an entered same-CPU caller and kept
        // target reference. It cannot run a callback/switch or free this Arc.
        let this = unsafe { &*binding.cast::<Self>() };
        if !this.published.load(Ordering::Acquire) || this.detached.load(Ordering::Acquire) {
            return -116; // ESTALE
        }
        // SAFETY: same-CPU exclusion and acquired initial publication; only an
        // new-job arrive updates this field, before C can target it.
        let handle = unsafe { (*this.target.get()).assume_init_ref() };
        if expected == 0 || handle.sequence() != expected {
            return -116;
        }
        match handle.wake() {
            Ok(()) => 0,
            Err(WorkError::Busy) => -16,
            Err(WorkError::Closed) => -116,
            Err(_) => -1,
        }
    }
    unsafe extern "C" fn service_reserved(binding: *mut c_void) -> bool {
        // SAFETY: C kept this binding and validated the target generation.
        // The first immutable signal owns the constructor's stack role,
        // even after an ordinary task reused its stack for a later job.
        let this = unsafe { &*binding.cast::<Self>() };
        if let Ok(first) = this.first()
            && let Ok(reserved) = first.service_reserved()
        {
            return reserved;
        }
        // An invalid role provider cannot silently authorize signal mutation.
        // SAFETY: fail closed before the C caller's first signal/ref effect.
        unsafe { core::arch::asm!("ud2", options(noreturn)) };
    }

    unsafe extern "C" fn rank(
        binding: *mut c_void,
        expected: u64,
        before: *const Rank,
        after: *const Rank,
        head: bool,
    ) -> i32 {
        // SAFETY: C retains this binding through its validated task generation;
        // no switch or final drop can occur while its PI lock is held.
        let this = unsafe { &*binding.cast::<Self>() };
        if !this.published.load(Ordering::Acquire)
            || this.detached.load(Ordering::Acquire)
            || expected == 0
        {
            return -116;
        }
        // SAFETY: initial publication and every new-job arrive initialize the
        // target before C can use its same-CPU, serialized task metadata.
        let target = unsafe { (*this.target.get()).assume_init_ref() };
        if target.sequence() != expected || before.is_null() || after.is_null() {
            return -116;
        }
        let result = (|| {
            let handle = target.scheduling_handle()?;
            // SAFETY: the live C call provides two initialized, immutable
            // primitive records; their borrows end before returning to C.
            let (before, after) = unsafe { ((&*before).snapshot()?, (&*after).snapshot()?) };
            // SAFETY: C owns the original task's serialized priority/PI-lock
            // mutation and commits matching C fields before releasing it.
            unsafe { handle.update(before, after, head) }
        })();
        match result {
            Ok(()) => 0,
            Err(WorkError::Closed) => -116,
            Err(WorkError::Unavailable) => -95,
            Err(_) => -1,
        }
    }

    fn validate(&self, context: &TaskContext) -> Result<(), WorkError> {
        let handle = self.current_wake()?;
        if handle.context_id().get() != context.context_id || handle.sequence() != context.sequence
        {
            return Err(WorkError::corrupt());
        }
        // Runtime keeps Queue/signal while attached: this temporary handle's
        // drop is necessarily nonfinal, including IF=0 arrive callbacks.
        Ok(())
    }
    unsafe extern "C" fn current(binding: *mut c_void, context: *const TaskContext) -> bool {
        // SAFETY: Arc receiver and callback-scoped readable view.
        let this = unsafe { &*binding.cast::<Self>() };
        // Validate CPU/IRQ/RSP independently BEFORE dereferencing any
        // mutable C task/context field. ops/binding stay immutable and kept
        // throughout attached lifetime, including taskless/IRQ rejection.
        let Ok(handle) = this.current_wake() else {
            return false;
        };
        // SAFETY: same-CPU current has now been established. C supplies
        // copied primitive metadata, not a temporary Runtime view.
        let view = unsafe { &*context };
        handle.context_id().get() == view.context_id && handle.sequence() == view.sequence
    }

    /// Run C with a shared Runtime capability on this stack.
    ///
    /// # Safety
    /// `body`/`context` remain valid through every real suspension; C may not
    /// unwind, escape the capability, leave a Linux owner/critical state live,
    /// or retire the Runtime/stack/binding. Keep shared owner roots and use
    /// the checked C lease for mutations. An exclusive Rust owner/context
    /// borrow may not span park if another continuation can access that owner.
    pub unsafe fn run_scoped(
        self: &Arc<Self>,
        runtime: Pin<&Runtime<N>>,
        body: unsafe extern "C" fn(*mut c_void),
        context: *mut c_void,
    ) -> Result<(), WorkError> {
        let view = runtime.current_context()?;
        self.validate(&view)?;
        let scope = Scope { runtime };
        // SAFETY: validated actual native current and caller's scoped body.
        unsafe { run_scope(self.task, &Self::SCOPE_OPS, scope, body, context) }
    }
    unsafe extern "C" fn terminate(scope: *mut c_void) -> ! {
        // SAFETY: run_scoped retained this actual pinned shared Runtime scope.
        // Only genuine original do_task_dead reaches here; skipped wrappers
        // own no Rust data/guards/process permit. Core invokes distinct source
        // prepare then physically switches and joins on actual dispatcher.
        unsafe { (&*scope.cast::<Scope<'_, N>>()).runtime.terminate_current() }
    }
    unsafe extern "C" fn park(scope: *mut c_void, service_idle: bool) -> i32 {
        // SAFETY: run_scoped keeps this pinned shared Runtime borrow until C
        // returns, including the suspended stack and borrowed owners.
        let runtime = unsafe { &*scope.cast::<Scope<'_, N>>() }.runtime;
        if service_idle && runtime.service_current() != Ok(true) {
            return -1;
        }
        // The native Domain already signals its reserved continuation.
        match runtime.park_if_blocked() {
            Ok(ParkOutcome::Skipped) => 0,
            Ok(ParkOutcome::Resumed) => 1,
            Err(WorkError::Busy) => -16,
            Err(_) => -22,
        }
    }
    unsafe extern "C" fn yield_now(scope: *mut c_void) -> i32 {
        // SAFETY: same shared scoped capability as park.
        match unsafe { &*scope.cast::<Scope<'_, N>>() }
            .runtime
            .yield_now()
        {
            Ok(()) => 0,
            Err(WorkError::Busy) => -16,
            Err(_) => -22,
        }
    }

    unsafe extern "C" fn reschedule(scope: *mut c_void) -> i32 {
        // SAFETY: same pinned shared Runtime scope; C proved a process
        // preparation or its private outside-owner scope-end preparation.
        match unsafe { &*scope.cast::<Scope<'_, N>>() }
            .runtime
            .reschedule()
        {
            Ok(()) => 0,
            Err(WorkError::Busy) => -16,
            Err(_) => -22,
        }
    }
    unsafe extern "C" fn reschedule_requested(scope: *mut c_void) -> bool {
        // SAFETY: C checks current, critical state and scope before this
        // bounded clock/query callback. It invokes no C delivery or switch.
        match unsafe { &*scope.cast::<Scope<'_, N>>() }
            .runtime
            .reschedule_requested()
        {
            Ok(requested) => requested,
            Err(_) => {
                // SAFETY: invalid/closed providers cannot become a false query.
                unsafe { core::arch::asm!("ud2", options(noreturn)) };
            }
        }
    }

    unsafe extern "C" fn account_current(scope: *mut c_void, total_ns: *mut u64) -> i32 {
        // SAFETY: the entered C frame keeps its exact current Runtime scope and output alive.
        let scope = unsafe { &*scope.cast::<Scope<'_, N>>() };
        match scope.runtime.execution_current() {
            Ok(sample) => {
                // SAFETY: the C caller lends one writable result for this synchronous call.
                unsafe { total_ns.write(sample.total_ns()) };
                0
            }
            Err(WorkError::Unavailable) => -95,
            Err(_) => -1,
        }
    }

    /// Release a pristine unattached task, or after Runtime detach restored flags.
    ///
    /// # Safety
    /// Producer/waiter/task/code users have stopped, on the checked
    /// process CPU outside IRQ and Linux entry, after native flags restoration.
    /// Detach's unsafe quiescence promise is real, not inferred from Arc counts.
    pub unsafe fn release_quiescent(self: Arc<Self>) -> Result<(), Arc<Self>> {
        if self.initial_root.is_some() || self._source_execution.is_some() {
            return Err(self); // Original source records have distinct affine finalization.
        }
        if self.published.load(Ordering::Acquire) && !self.detached.load(Ordering::Acquire) {
            return Err(self);
        }
        let mut this = Arc::try_unwrap(self)?;
        // SAFETY: explicit caller proof plus successful unique detached ownership.
        unsafe { stela_linux_task_release(this.task.as_ptr()) };
        this.released = true;
        Ok(())
    }
}
/// Run one scoped C body on an independently validated bound continuation.
///
/// # Safety
/// The caller holds current and retains the task, body and all borrowed
/// providers across any suspension. The body must return with its C scope
/// quiescent and may not unwind or let the capability escape.
unsafe fn run_scope<S>(
    task: NonNull<CTask>,
    ops: &'static ScopeOps,
    scope: S,
    body: unsafe extern "C" fn(*mut c_void),
    context: *mut c_void,
) -> Result<(), WorkError> {
    let scope = core::pin::pin!(scope);
    let pointer = core::ptr::from_ref(scope.as_ref().get_ref())
        .cast_mut()
        .cast();
    // SAFETY: exact validated task and stack-local, pinned capability.
    status(unsafe { stela_linux_task_scope_begin(task.as_ptr(), ops, pointer) })?;
    // SAFETY: caller retains the body/context and forbids unwind or escape.
    unsafe { body(context) };
    // SAFETY: the same original source body returned on its own stack.
    if unsafe { stela_linux_task_scope_end(task.as_ptr()) } != 0 {
        // SAFETY: retain the live scope and all owners on uncertain C exit.
        unsafe { core::arch::asm!("ud2", options(noreturn)) };
    }
    Ok(())
}

struct Scope<'a, const N: usize> {
    runtime: Pin<&'a Runtime<N>>,
}

// SAFETY: Runtime owns real current/stack/IF exclusion and keeps this Arc.
// Each C hook is transactional before leave or bounded after committed arrival;
// no drain/allocator/final release occurs in any hook. C copies view metadata.
unsafe impl<const N: usize> SwitchBinding for TaskBinding<N> {
    fn execution_publication(
        &self,
        context: &TaskContext,
    ) -> Result<Option<Arc<dyn stela_driver_base::platform::work::ExecutionPublication>>, WorkError>
    {
        match &self.source_data {
            Some(data) => data.configure(context).map(Some),
            None if self._source_execution.is_none() && self.initial_root.is_none() => Ok(None),
            None => Err(WorkError::Unavailable),
        }
    }

    unsafe fn attach(&self, context: &TaskContext) -> Result<(), WorkError> {
        if self.published.load(Ordering::Acquire) || self.detached.load(Ordering::Acquire) {
            return Err(WorkError::Busy);
        }
        if let Some(data) = &self.source_data {
            data.attachable(context)?;
        }
        // SAFETY: initial stack hook precedes initial WakeHandle publication.
        status(unsafe {
            stela_linux_task_attach(
                self.task.as_ptr(),
                context,
                &Self::OPS,
                self as *const Self as *mut c_void,
            )
        })?;
        if let Some(data) = &self.source_data {
            data.attached(context);
        }
        Ok(())
    }
    unsafe fn still_blocked(&self, context: &TaskContext) -> Result<bool, WorkError> {
        self.validate(context)?;
        // SAFETY: current CPU/RSP/ticket checked before C state inspection.
        match unsafe { stela_linux_task_still_blocked(self.task.as_ptr(), context) } {
            0 => Ok(false),
            1 => Ok(true),
            -16 => Err(WorkError::Busy),
            _ => Err(WorkError::corrupt()),
        }
    }
    unsafe fn leave(&self, context: &TaskContext, reason: LeaveReason) -> Result<(), WorkError> {
        self.validate(context)?;
        // SAFETY: outgoing stack/IF0, C requires prepared process snapshot.
        status(unsafe { stela_linux_task_leave(self.task.as_ptr(), context, reason as u32) })
    }
    unsafe fn arrive(&self, context: &TaskContext, new_job: bool) -> Result<(), WorkError> {
        self.validate(context)?;
        if new_job {
            let next = self.current_wake()?;
            // SAFETY: same-CPU destination stack/IF0, no foreign C or
            // IRQ task-pointer reader. The mutable field borrow ends before
            // any hook. Runtime keeps Queue/signal so old Drop is nonfinal.
            let previous =
                unsafe { core::ptr::replace((*self.target.get()).assume_init_mut(), next) };
            drop(previous);
        }
        // SAFETY: resumed/new job stack and existing ticket before IF restore.
        status(unsafe { stela_linux_task_arrive(self.task.as_ptr(), context, new_job) })
    }
    unsafe fn complete(&self, context: &TaskContext) -> Result<(), WorkError> {
        self.validate(context)?;
        // SAFETY: outgoing real stack; C refuses unless process exit/scope ended.
        status(unsafe { stela_linux_task_complete(self.task.as_ptr(), context) })
    }
    unsafe fn terminal_prepare(&self, context: &TaskContext) -> Result<(), WorkError> {
        self.validate(context)?;
        // SAFETY: same real dying current/IF0 and retained original child;
        // resource fields may be gone, no ordinary current is manufactured.
        status(unsafe { stela_linux_task_terminal_prepare(self.task.as_ptr(), context) })
    }
    unsafe fn terminal_successor(&self, context: &TaskContext) -> Result<(), WorkError> {
        self.first()?.check_process()?;
        if self.first()?.context_id().get() != context.context_id {
            return Err(WorkError::corrupt());
        }
        // SAFETY: Core supplied true off-all-stacks native successor/IF0 and
        // exact retained source binding; C only records closure/notification.
        status(unsafe { stela_linux_task_terminal_successor(self.task.as_ptr(), context) })
    }
    unsafe fn detach(&self, context: &TaskContext) -> Result<(), WorkError> {
        // No current validation is invented on the idle dispatcher. Runtime
        // already checks CPU/IF0/quiescence and supplies saved RSP.
        if self.first()?.context_id().get() != context.context_id {
            return Err(WorkError::corrupt());
        }
        // SAFETY: unsafe caller's producer quiescence and idle hook.
        status(unsafe { stela_linux_task_detach(self.task.as_ptr(), context) })?;
        self.detached.store(true, Ordering::Release);
        Ok(())
    }
}
impl<const N: usize> Drop for TaskBinding<N> {
    fn drop(&mut self) {
        if !self.released {
            // No uncertain Drop can become C metadata release permission. A live
            // Runtime still keeps an Arc; safely detached but unconsumed task
            // metadata is held and observable here.
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
        if self.published.load(Ordering::Acquire) {
            // SAFETY: exclusive Self destruction and acquired single publication.
            // WakeHandle itself quarantines final roots on IRQ/foreign CPU Drop.
            unsafe {
                (*self.target.get()).assume_init_drop();
                (*self.first.get()).assume_init_drop();
            };
        }
    }
}

/// The two observable kernel-wait cancellation classes. These post real
/// original SIGUSR1/SIGKILL pending bits; no POSIX process termination, siginfo
/// delivery, realtime queue or guest signal ABI is implied.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u32)]
pub enum WaitSignal {
    Interrupt = 1,
    Fatal = 2,
}

/// Owns metadata and an immutable expected job, not a permanent C usage ref.
/// Completion can close the job while this handle is held. Only the plain
/// WakeHandle is an IRQ signal view; TargetTask's C operations are process-only.
pub struct TargetTask<const N: usize> {
    binding: ManuallyDrop<Arc<TaskBinding<N>>>,
    wake: ManuallyDrop<WakeHandle<N>>,
}
impl<const N: usize> Clone for TargetTask<N> {
    fn clone(&self) -> Self {
        Self {
            binding: self.binding.clone(),
            wake: self.wake.clone(),
        }
    }
}
impl<const N: usize> TargetTask<N> {
    pub fn context_id(&self) -> ContextId {
        self.wake.context_id()
    }
    pub fn sequence(&self) -> u64 {
        self.wake.sequence()
    }
    /// Post to the captured job only. Closed/reused jobs return Closed before
    /// future C state is accessed; keeping this target is not a Linux task ref.
    ///
    /// # Safety
    /// Call on the real same-CPU entered Linux process continuation described
    /// by `caller`, with shared service roots kept. Do not call from IRQ,
    /// IF0 switch hooks, a dispatcher, or outside run_scoped/Linux entry.
    /// Target wait cancellation returns Busy once its body has returned (or
    /// no C body is active); generic immutable wakes remain valid.
    pub unsafe fn signal(
        &self,
        caller: Pin<&Runtime<N>>,
        kind: WaitSignal,
    ) -> Result<(), WorkError> {
        self.wake.check_process()?;
        caller.current_context()?;
        // SAFETY: immutable expected generation plus separately kept C
        // metadata; caller proves entered Linux process permission.
        match unsafe {
            stela_linux_task_signal(self.binding.task.as_ptr(), self.sequence(), kind as u32)
        } {
            0 => Ok(()),
            -116 => Err(WorkError::Closed),
            -16 => Err(WorkError::Busy),
            -95 => Err(WorkError::Unavailable),
            _ => Err(WorkError::corrupt()),
        }
    }
}
impl<const N: usize> Drop for TargetTask<N> {
    fn drop(&mut self) {
        if self.wake.check_process().is_ok() {
            // SAFETY: checked process CPU. Native callers keep the normal
            // flags-restored release boundary; no switch hook owns a TargetTask.
            // Keep the immutable wake owner until after binding destruction.
            unsafe {
                ManuallyDrop::drop(&mut self.binding);
                ManuallyDrop::drop(&mut self.wake);
            }
        } else {
            // Never let an old producer's final owner release run in native
            // IRQ/foreign-CPU context. Keep BOTH metadata and provider roots.
            QUARANTINED.fetch_add(1, Ordering::Relaxed);
        }
    }
}

#[cfg(all(
    test,
    feature = "hosted-test-support",
    target_arch = "x86_64",
    any(target_os = "linux", target_os = "macos")
))]
pub(crate) mod tests;
