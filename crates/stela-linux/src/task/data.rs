//! Genuine source task-ref lease on independently retained allocation backing.
use super::*;
use crate::{
    owner::RawOwner,
    threads::{TaskDataBacking, TaskDataStorage},
};
use core::sync::atomic::AtomicU8;
use stela_driver_base::platform::work::{
    EnteredProcess, ExecutionIdentity, ExecutionPublication, ExecutionSample, StoppedPublication,
};
mod stop;
pub(crate) use stop::StoppedTaskData;
const PREPARED: u8 = 0;
const CONFIGURED: u8 = 1;
const CONFIGURING: u8 = 5;
const ACTIVE: u8 = 2;
const SOURCE_RELEASED: u8 = 3;
const FINISHED: u8 = 4;
#[repr(C)]
pub(super) struct CData {
    _opaque: [u8; 0],
}
#[repr(C)]
pub(super) struct DataIdentity {
    pub(super) queue_id: u64,
    pub(super) context_id: u64,
    pub(super) sequence: u64,
}
const _: () = assert!(core::mem::size_of::<DataIdentity>() == 24);
#[repr(C)]
pub(super) struct DataView {
    pub(super) lease: *mut CData,
    pub(super) total: *mut u64,
    pub(super) system: *mut u64,
}
const _: () = assert!(core::mem::size_of::<DataView>() == 24);
unsafe extern "C" {
    pub(super) fn stela_linux_task_data_prepare(
        task: *mut CTask,
        owner: *const RawOwner,
        view: *mut DataView,
    ) -> i32;
    pub(super) fn stela_linux_task_data_release(
        lease: *mut CData,
        owner: *const RawOwner,
        view: *const DataIdentity,
    ) -> i32;
}
/// Exact prepared C lease/backing owner on every normal preparation refusal.
/// Never a native/source no-exposure receipt or ordinary clean errno.
pub(super) struct PreparationFailure {
    pub(super) error: ResourceBootstrapError,
    _prepared: ManuallyDrop<Preparation>,
}
struct Preparation {
    _task: NonNull<CTask>,
    _owner: *const RawOwner,
    view: DataView,
    _c_returned_success: bool,
    backing: ManuallyDrop<Arc<dyn TaskDataBacking>>,
}
/// This object has NO Queue/Runtime/Kernel/Platform/Binding ownership edge.
/// C inline lease keeps no Rust reference; its actual original task reference
/// is acquired only by accepted foreign attach. Unclosed Drop retains backing.
pub(super) struct SourceData {
    owner: *const RawOwner,
    lease: NonNull<CData>,
    total: NonNull<u64>,
    system: NonNull<u64>,
    backing: UnsafeCell<ManuallyDrop<Option<Arc<dyn TaskDataBacking>>>>,
    phase: AtomicU8,
    queue: AtomicU64,
    context: AtomicU64,
    sequence: AtomicU64,
}
// SAFETY: publication is only through actual native identity/root admission.
// Stored pointers name genuine C fields under a separately held task ref;
// atomics serialize one-shot lifecycle. IRQ publishing is explicitly refused.
unsafe impl Send for SourceData {}
// SAFETY: same contract; immutable field addresses, atomic lifecycle, and
// exclusive final backing take only after actual native/C writer-tail joins.
unsafe impl Sync for SourceData {}
fn fatal() -> ! {
    // SAFETY: stop before an invalid source identity could write C storage.
    unsafe { core::arch::asm!("ud2", options(noreturn)) }
}
impl SourceData {
    /// # Safety
    /// Exact actual source constructor under checked same-owner native metadata
    /// scope/root; no original allocation wait or source reentry. Backing and
    /// kernel/code owners are already held BEFORE this external C operation.
    pub(super) unsafe fn prepare(
        task: NonNull<CTask>,
        owner: *const RawOwner,
        storage: TaskDataStorage,
        backing: &Arc<dyn TaskDataBacking>,
    ) -> Result<Arc<Self>, PreparationFailure> {
        // Before ANY foreign C call, including acceptance followed by range
        // rejection or a panic in a backing implementation. Exact C output
        // and original record stay in this retained affine preparation owner.
        let mut prepared = ManuallyDrop::new(Preparation {
            _task: task,
            _owner: owner,
            view: DataView {
                lease: core::ptr::null_mut(),
                total: core::ptr::null_mut(),
                system: core::ptr::null_mut(),
            },
            _c_returned_success: false,
            backing: ManuallyDrop::new(backing.clone()),
        });
        let result = (|| {
            // SAFETY: exact constructor/retained same native segment contract;
            // prepared owner already keeps every accepted output/backing.
            let value =
                unsafe { stela_linux_task_data_prepare(task.as_ptr(), owner, &mut prepared.view) };
            prepared._c_returned_success = value == 0;
            if value != 0 {
                return Err(ResourceBootstrapError::Native(value));
            }
            let lease = NonNull::new(prepared.view.lease)
                .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
            let total = NonNull::new(prepared.view.total)
                .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
            let system = NonNull::new(prepared.view.system)
                .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
            for field in [total, system] {
                let first = field.as_ptr().addr();
                let end = first
                    .checked_add(core::mem::size_of::<u64>())
                    .ok_or_else(|| ResourceBootstrapError::Work(WorkError::corrupt()))?;
                if !first.is_multiple_of(core::mem::align_of::<u64>())
                    || !backing.contains(storage, first..end)
                {
                    return Err(ResourceBootstrapError::Work(WorkError::Unavailable));
                }
            }
            if total == system {
                return Err(ResourceBootstrapError::Work(WorkError::corrupt()));
            }
            Ok((lease, total, system))
        })();
        let (lease, total, system) = match result {
            Ok(fields) => fields,
            Err(error) => {
                return Err(PreparationFailure {
                    error,
                    _prepared: prepared,
                });
            }
        };
        // SAFETY: successful exact preparation transfers its actual backing
        // once into the real source sink; no fallible external check follows.
        let backing = unsafe { ManuallyDrop::take(&mut prepared.backing) };
        Ok(Arc::new(Self {
            owner,
            lease,
            total,
            system,
            backing: UnsafeCell::new(ManuallyDrop::new(Some(backing))),
            phase: AtomicU8::new(PREPARED),
            queue: AtomicU64::new(0),
            context: AtomicU64::new(0),
            sequence: AtomicU64::new(0),
        }))
    }

    fn matches(&self, identity: ExecutionIdentity) -> bool {
        self.queue.load(Ordering::Acquire) == identity.queue_id()
            && self.context.load(Ordering::Acquire) == identity.context_id()
            && self.sequence.load(Ordering::Acquire) == identity.sequence()
            && identity.sequence() != 0
    }
    fn matches_context(&self, view: &TaskContext) -> bool {
        self.queue.load(Ordering::Acquire) == view.queue_id
            && self.context.load(Ordering::Acquire) == view.context_id
            && self.sequence.load(Ordering::Acquire) == view.sequence
            && view.sequence != 0
    }
    pub(super) fn configure(
        self: &Arc<Self>,
        view: &TaskContext,
    ) -> Result<Arc<dyn ExecutionPublication>, WorkError> {
        if view.queue_id == 0 || view.context_id == 0 || view.sequence == 0 {
            return Err(WorkError::InvalidTicket);
        }
        match self.phase.load(Ordering::Acquire) {
            PREPARED => {
                self.phase
                    .compare_exchange(PREPARED, CONFIGURING, Ordering::AcqRel, Ordering::Acquire)
                    .map_err(|_| WorkError::Busy)?;
                // Called only by the real same-CPU native pre-attach boundary,
                // BEFORE this sink is published to Board/IRQ readers.
                self.queue.store(view.queue_id, Ordering::Relaxed);
                self.context.store(view.context_id, Ordering::Relaxed);
                self.sequence.store(view.sequence, Ordering::Relaxed);
                self.phase.store(CONFIGURED, Ordering::Release);
            }
            CONFIGURED if self.matches_context(view) => {}
            _ => return Err(WorkError::Busy),
        }
        Ok(self.clone())
    }
    pub(super) fn attachable(&self, view: &TaskContext) -> Result<(), WorkError> {
        if self.phase.load(Ordering::Acquire) != CONFIGURED || !self.matches_context(view) {
            return Err(WorkError::corrupt());
        }
        Ok(())
    }
    /// Accepted actual C attach already acquired original get_task_struct.
    /// Infallible noalloc tail, never a late ordinary Err hiding publication.
    pub(super) fn attached(&self, view: &TaskContext) {
        if self.attachable(view).is_err() {
            fatal();
        }
        self.phase.store(ACTIVE, Ordering::Release);
    }
    /// Caller holds actual private native NoPublication and joined original
    /// source unwind/free tails; phase and C flags only reject mistakes.
    pub(super) unsafe fn close_unpublished(
        &self,
        owner: *const RawOwner,
    ) -> Result<(), ResourceBootstrapError> {
        if self.owner == owner && self.phase.load(Ordering::Acquire) == FINISHED {
            return Ok(()); // same genuine completed close, metadata retry only
        }
        if self.owner != owner
            || !matches!(self.phase.load(Ordering::Acquire), PREPARED | CONFIGURED)
        {
            return Err(ResourceBootstrapError::Work(WorkError::corrupt()));
        }
        // SAFETY: caller owns actual NoPublication plus joined source tails;
        // C independently rejects every claimed/live foreign attachment.
        let value =
            unsafe { stela_linux_task_data_release(self.lease.as_ptr(), owner, core::ptr::null()) };
        if value != 0 {
            return Err(ResourceBootstrapError::Native(value));
        }
        self.dispose_backing();
        Ok(())
    }
    pub(super) fn is_finished(&self) -> bool {
        self.phase.load(Ordering::Acquire) == FINISHED
    }
    fn dispose_backing(&self) {
        // SAFETY: sole real stop/cancel control after C and native/IRQ joins.
        let backing = unsafe {
            (&mut *self
                .backing
                .get()
                .cast::<Option<Arc<dyn TaskDataBacking>>>())
                .take()
        };
        self.phase.store(FINISHED, Ordering::Release);
        drop(backing);
    }
}
// SAFETY: prepared by genuine source record/actual backing; accepted attach
// owns original task ref. Core certifies exact current mapped native process
// root/CPU/IF0. Only aligned bounded volatile field stores occur; no C call,
// allocation, owner drop, scheduling or borrowed native epoch is used here.
unsafe impl ExecutionPublication for SourceData {
    unsafe fn publish(&self, sample: &ExecutionSample) {
        if self.phase.load(Ordering::Acquire) != ACTIVE || !self.matches(sample.identity()) {
            fatal();
        }
        // SAFETY: genuine attached source ref, exact native identity and the
        // actual mapped process-root contract; only aligned u64 stores.
        unsafe {
            // The source sink is attached only to the Initial task or a kernel thread.
            self.system.as_ptr().write_volatile(sample.total_ns());
            self.total.as_ptr().write_volatile(sample.total_ns());
        }
    }
    // Required genuine IRQ data-write qualification is NOT supplied by
    // CounterEpoch/EntryEpoch or this lifetime backing. Default Unavailable
    // is intentional; full IRQ/WQ tick readiness remains an unmet dependency.
}
