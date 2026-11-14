//! One actual original-child native/page owner, never another task scheduler.
//! Source code must keep each value before every external C/native publisher.
use super::{ChildMetadataFailure, OriginalBirth, TaskBinding};
use crate::{
    kernel::KernelOwner,
    pages::{self, Catalog, Provider},
    threads::KernelThreadPlatform,
};
use alloc::sync::Arc;
use core::{
    ffi::c_void,
    mem::{self, ManuallyDrop},
    pin::Pin,
    ptr::NonNull,
};
use stela_driver_base::platform::work::{
    AssignedGeneral, Callback, CancelledReservation, EnteredProcess, GeneralRefusal,
    GeneralStackPreparation, OwnedGeneralEscrow, OwnedTransferFailure,
    PinnedGeneralStackPreparation, Runtime, StackPinError, WorkError, scheduling,
};
mod cancel;
mod prefix;
mod publication;
pub(crate) use cancel::NoPublication;
pub(crate) use publication::RetiredMetadata;

unsafe extern "C" {
    fn stela_linux_task_child_identity(
        task: *mut super::CTask,
        owner: *const crate::owner::RawOwner,
        target: *mut *mut c_void,
        generation: *mut u64,
    ) -> i32;
    fn stela_linux_task_child_install_stack(
        task: *mut super::CTask,
        owner: *const crate::owner::RawOwner,
        base: *mut c_void,
        bytes: usize,
    );
    fn stela_linux_task_child_source_unwound(
        task: *mut super::CTask,
        owner: *const crate::owner::RawOwner,
        generation: u64,
    ) -> i32;
}

/// A checked view of the SAME canonical owner/Runtime/catalogue. It creates no
/// Environment, task domain, clock, physical memory or code authority.
pub(crate) struct Factory<
    'kernel,
    'catalog,
    'runtime,
    K: KernelThreadPlatform,
    P: Provider,
    const N: usize,
> {
    kernel: &'kernel KernelOwner<K, N>,
    runtime: Pin<&'runtime Runtime<N>>,
    catalog: &'catalog Catalog<P>,
}
impl<'kernel, 'catalog, 'runtime, K: KernelThreadPlatform, P: Provider, const N: usize>
    Factory<'kernel, 'catalog, 'runtime, K, P, N>
{
    /// # Safety
    /// Checked native dispatcher metadata phase before Initial, or genuine
    /// source current on the canonical CPU; native allocator, code/root,
    /// mixed stack backing, catalogue and scheduling domain are genuinely kept.
    /// No other owner may reuse a kernel stack without full original task/page/
    /// alias/RCU retirement. The mount control outlives all abandoned FFI graphs.
    pub(crate) unsafe fn new(
        kernel: &'kernel KernelOwner<K, N>,
        runtime: Pin<&'runtime Runtime<N>>,
        catalog: &'catalog Catalog<P>,
    ) -> Result<Self, Error> {
        if !kernel.matches_runtime(runtime) {
            return Err(Error::Work(WorkError::corrupt()));
        }
        if !catalog.matches_kernel(kernel) {
            return Err(Error::Catalog(pages::Error::InvalidReservation));
        }
        // Pointer equality is only a negative consistency check, never Ready.
        // Same canonical Catalog is an explicit actual-owner input contract;
        // real descriptor operations independently validate its stored source
        // owner/generation. Never run a diagnostic allocation probe as admission.
        Ok(Self {
            kernel,
            runtime,
            catalog,
        })
    }

    /// # Safety
    /// Synchronous genuine original dup_task_struct allocation callback only.
    /// The opaque birth came from its successful real allocation factory; no
    /// caller task pointer, guessed generation or descriptor can create it.
    /// The source constructor remains retained until explicit source free.
    pub(crate) unsafe fn begin<'factory>(
        &'factory self,
        birth: NonNull<OriginalBirth>,
    ) -> Result<Child<'factory, 'kernel, 'catalog, 'runtime, K, P, N>, BeginFailure<N>> {
        // SAFETY: actual source constructor; new_child takes the genuine token
        // in one bounded with_process and retains the same Queue ExecutionPin.
        let binding =
            unsafe { TaskBinding::new_child(self.kernel, birth) }.map_err(|metadata| {
                BeginFailure::Metadata {
                    _metadata: metadata,
                }
            })?;
        let binding = ManuallyDrop::new(binding);
        let mut target = core::ptr::null_mut();
        let mut generation = 0;
        // Keep metadata/code BEFORE the external identity observation. Its
        // refusal/unwind is not a no-publication/cancellation receipt.
        // SAFETY: exact retained record, same canonical original creator.
        let value = unsafe {
            stela_linux_task_child_identity(
                binding.task.as_ptr(),
                self.kernel.raw(),
                &mut target,
                &mut generation,
            )
        };
        let identity = match (value, NonNull::new(target), generation) {
            (0, Some(target), generation) if generation != 0 => Identity { target, generation },
            (value, _, _) => {
                return Err(BeginFailure::Identity {
                    _binding: binding,
                    _error: if value == 0 {
                        Error::Work(WorkError::corrupt())
                    } else {
                        Error::Source(value)
                    },
                });
            }
        };
        Ok(Child {
            factory: self,
            binding,
            identity,
            state: ManuallyDrop::new(State::Born),
        })
    }
}

#[derive(Debug)]
pub(crate) enum Error {
    Admission(GeneralRefusal),
    Work(WorkError),
    Pin(StackPinError),
    Pages(pages::AdmissionError),
    Catalog(pages::Error),
    Source(i32),
    WrongPhase,
    PublicationUncertain,
}
/// Missing metadata/identity never becomes no-publication by its errno.
/// All ambiguous source metadata and original execution roots stay retained.
pub(crate) enum BeginFailure<const N: usize> {
    Metadata {
        _metadata: ChildMetadataFailure,
    },
    Identity {
        _binding: ManuallyDrop<Arc<TaskBinding<N>>>,
        _error: Error,
    },
}

#[derive(Clone, Copy)]
struct Identity {
    target: NonNull<c_void>,
    generation: u64,
}
#[derive(Clone, Copy, Eq, PartialEq)]
enum Exposure {
    Unlent,
    Lending,
    Lent,
}

enum State<'catalog, 'runtime, P: Provider, const N: usize> {
    Born,
    Owned(OwnedGeneralEscrow<'runtime, N>),
    Preparing(GeneralStackPreparation<'runtime, N>),
    Pinned(PinnedGeneralStackPreparation<'runtime, N>),
    Admission(pages::StackAdmission<'catalog, 'runtime, P, N>),
    Prepared {
        lease: pages::PreparedChildStackLease<'catalog, 'runtime, P, N>,
        exposure: Exposure,
    },
    Dormant(pages::DormantChildStackLease<'catalog, 'runtime, P, N>),
    Published(pages::PublishedChildStackLease<'catalog, 'runtime, P, N>),
    // Exact borrow-free native exclusion on transfer failure. Native rollback
    // is proved only by its affine CancelledReservation, never the diagnostic.
    TransferFailed(OwnedTransferFailure<'runtime, N>),
    Transitioning,
}

/// Owns the true source metadata and every native/page phase. No borrowed
/// EnteredProcess, C constructor-frame pointer or RefCell guard is stored.
/// Default Drop/forget retains the graph rather than reclaiming uncertain FFI.
#[must_use = "cancel unpublished source or join source/native retirement explicitly"]
pub(crate) struct Child<
    'factory,
    'kernel,
    'catalog,
    'runtime,
    K: KernelThreadPlatform,
    P: Provider,
    const N: usize,
> {
    factory: &'factory Factory<'kernel, 'catalog, 'runtime, K, P, N>,
    binding: ManuallyDrop<Arc<TaskBinding<N>>>,
    identity: Identity,
    state: ManuallyDrop<State<'catalog, 'runtime, P, N>>,
}
pub(crate) struct Failure<
    'factory,
    'kernel,
    'catalog,
    'runtime,
    K: KernelThreadPlatform,
    P: Provider,
    const N: usize,
> {
    pub(crate) error: Error,
    pub(crate) child: Child<'factory, 'kernel, 'catalog, 'runtime, K, P, N>,
}
impl<'factory, 'kernel, 'catalog, 'runtime, K: KernelThreadPlatform, P: Provider, const N: usize>
    Child<'factory, 'kernel, 'catalog, 'runtime, K, P, N>
{
    pub(crate) fn generation(&self) -> u64 {
        self.identity.generation
    }
    pub(crate) fn binding(&self) -> &Arc<TaskBinding<N>> {
        &self.binding
    }
    fn check_born_creator(&self, entered: &dyn EnteredProcess) -> Result<(), Error> {
        entered.check().map_err(Error::Work)?;
        let mut target = core::ptr::null_mut();
        let mut generation = 0;
        // SAFETY: SAME retained genuine C record; this observational validator
        // independently checks canonical actual source creator before storage.
        let value = unsafe {
            stela_linux_task_child_identity(
                self.binding.task.as_ptr(),
                self.factory.kernel.raw(),
                &mut target,
                &mut generation,
            )
        };
        if value != 0 {
            return Err(Error::Source(value));
        }
        if target != self.identity.target.as_ptr() || generation != self.identity.generation {
            return Err(Error::Work(WorkError::corrupt()));
        }
        Ok(())
    }
    fn take_state(&mut self) -> State<'catalog, 'runtime, P, N> {
        mem::replace(&mut *self.state, State::Transitioning)
    }
    fn keep(&mut self, state: State<'catalog, 'runtime, P, N>) {
        *self.state = state;
    }
    fn take_binding(&mut self) -> ManuallyDrop<Arc<TaskBinding<N>>> {
        // SAFETY: called once only after successful native ownership removal;
        // the enclosing Child is ManuallyDrop and will never touch this field again.
        ManuallyDrop::new(unsafe { ManuallyDrop::take(&mut self.binding) })
    }
    fn fail(self, error: Error) -> Failure<'factory, 'kernel, 'catalog, 'runtime, K, P, N> {
        Failure { error, child: self }
    }
}
