//! Original Tiny SRCU probes through the one retained original-kthread runtime.
//! Callback return and observed status never acknowledge complete task join.
use crate::{
    kernel::KernelOwner,
    owner::{Platform, RawOwner},
    task::runtime::{
        JoinedJob, ManagedJob, Outcome, Pending, Status as ManagedStatus, SubmitError, TaskRuntime,
    },
    threads::KernelThreadPlatform,
};
use alloc::{boxed::Box, rc::Rc, sync::Arc};
use core::{cell::Cell, mem::ManuallyDrop, pin::Pin};
use stela_driver_base::platform::work::WorkError;
mod job;
unsafe extern "C" {
    fn stela_linux_srcu_bootstrap(owner: *const RawOwner) -> i32;
    fn stela_linux_srcu_bootstrap_joined(owner: *const RawOwner) -> i32;
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    Work(WorkError),
    Native(i32),
    ManagedOutcome(Outcome),
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Stela Linux SRCU bootstrap: {self:?}")
    }
}
impl core::error::Error for Error {}
#[derive(Debug)]
pub enum StartError {
    Work(WorkError),
    Submission(SubmitError),
}
impl core::fmt::Display for StartError {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "Stela Linux SRCU bootstrap start: {self:?}")
    }
}
impl core::error::Error for StartError {}
type ProbeJob = Pin<Box<dyn ManagedJob>>;
type StartFailureParts<P, const N: usize> = (StartError, Arc<KernelOwner<P, N>>, Option<ProbeJob>);

/// Exact rejected owned request and canonical kernel. Default abandonment
/// retains both; no status/errno is converted into a no-publication receipt.
pub struct StartFailure<P: Platform, const N: usize> {
    pub error: StartError,
    kernel: ManuallyDrop<Arc<KernelOwner<P, N>>>,
    rejected: ManuallyDrop<Option<ProbeJob>>,
}
impl<P: Platform, const N: usize> StartFailure<P, N> {
    pub fn into_parts(mut self) -> StartFailureParts<P, N> {
        // SAFETY: consuming the exact returned/never-submitted owned request;
        // this extracts no accepted pending graph or fabricated native receipt.
        let kernel = unsafe { ManuallyDrop::take(&mut self.kernel) };
        // SAFETY: same affine extraction of this exact rejected owned job.
        let rejected = unsafe { ManuallyDrop::take(&mut self.rejected) };
        (self.error, kernel, rejected)
    }
}
enum State {
    Pending(Pending),
    Joined(JoinedJob),
}
/// Read-only progress. Only consuming Bootstrap::join obtains the constructor-
/// private JoinedJob; even Pending.joined does not grant public SRCU admission.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Status {
    Pending(ManagedStatus),
    Joined {
        outcome: Outcome,
        result: Option<i32>,
    },
}
/// One actual original-child request. Abandonment keeps its owned job, source
/// producers, actual native/C/stack/page/RCU graph and canonical kernel alive.
pub struct Bootstrap<P: KernelThreadPlatform + 'static, const N: usize> {
    kernel: ManuallyDrop<Arc<KernelOwner<P, N>>>,
    result: ManuallyDrop<Rc<Cell<Option<i32>>>>,
    state: ManuallyDrop<State>,
}
pub struct JoinFailure<P: KernelThreadPlatform + 'static, const N: usize> {
    pub error: Error,
    pub bootstrap: Bootstrap<P, N>,
}
/// Joined original worker/clock probes on the one canonical kernel. This does
/// not grant IRQ/DMA permission or prove later resource/worker availability.
pub struct Ready<P: Platform, const N: usize> {
    kernel: Arc<KernelOwner<P, N>>,
}
impl<P: Platform, const N: usize> Ready<P, N> {
    pub fn kernel(&self) -> &Arc<KernelOwner<P, N>> {
        &self.kernel
    }
}
impl<P: KernelThreadPlatform + 'static, const N: usize> Bootstrap<P, N> {
    /// Submit one owned job through the SAME original-kthread producer. It
    /// runs already entered on its authentic original child and joins original
    /// timer/normal/high replacement probes before returning from its C body.
    /// # Safety
    /// Actual CPU-local runtime control under a fresh qualified source/native
    /// segment, outside IRQ and incompatible native borrows. Keep real kernel,
    /// task-runtime/platform/code/root owners for their prescribed lifetime,
    /// including failure/abandonment. No borrowed process permit spans C waits.
    pub unsafe fn start<Q: crate::pages::Provider + 'static>(
        kernel: Arc<KernelOwner<P, N>>,
        runtime: &TaskRuntime<P, Q, N>,
    ) -> Result<Self, StartFailure<P, N>> {
        // Retain before the publisher or any potentially unwinding allocation.
        let kernel = ManuallyDrop::new(kernel);
        if !runtime.matches_kernel(&kernel) {
            return Err(StartFailure {
                error: StartError::Work(WorkError::InvalidTicket),
                kernel,
                rejected: ManuallyDrop::new(None),
            });
        }
        let mut result = ManuallyDrop::new(Rc::new(Cell::new(None)));
        let owned = job::new(Arc::clone(&kernel), Rc::clone(&result));
        // SAFETY: genuine same-owner producer owns the pinned job before source
        // creation. Pending retains every accepted/uncertain publication tail.
        let pending = match unsafe { runtime.submit(owned) } {
            Ok(pending) => pending,
            Err(rejected) => {
                // The actual returned job remains owned, not inferred clean
                // from its error. Its result allocation stays held by that job.
                // SAFETY: local result clone only; returned owned job retains it.
                drop(unsafe { ManuallyDrop::take(&mut result) });
                return Err(StartFailure {
                    error: StartError::Submission(rejected.error),
                    kernel,
                    rejected: ManuallyDrop::new(Some(rejected.job)),
                });
            }
        };
        Ok(Self {
            kernel,
            result,
            state: ManuallyDrop::new(State::Pending(pending)),
        })
    }
    pub fn status(&self) -> Status {
        match &*self.state {
            State::Pending(pending) => Status::Pending(pending.status()),
            State::Joined(joined) => Status::Joined {
                outcome: joined.outcome(),
                result: self.result.get(),
            },
        }
    }
    /// Consume the real request join before acknowledging public SRCU. A
    /// successful C result, callback_returned or joined observation alone is
    /// insufficient. Failed/incomplete joins retain their exact accepted graph.
    /// # Safety
    /// Actual same-CPU dispatcher/control context outside IRQ, native leaf/row/
    /// RefCell loans and source C frames. Original PID1 has already performed
    /// all C/source reference disposal before managed JOINED; try_join does
    /// only Rust owner extraction after its actual remaining reader/ref joins.
    pub unsafe fn join(mut self) -> Result<Ready<P, N>, JoinFailure<P, N>> {
        // SAFETY: one affine state owner, replaced before every ordinary return;
        // unwind retains the ManuallyDrop kernel/result and pending job graph.
        let state = unsafe { ManuallyDrop::take(&mut self.state) };
        let joined = match state {
            State::Joined(joined) => joined,
            // SAFETY: caller supplies qualified same-CPU dispatcher context;
            // the real broker already performed original C/source disposal.
            State::Pending(pending) => match unsafe { pending.try_join() } {
                Ok(joined) => joined,
                Err(pending) => {
                    self.state = ManuallyDrop::new(State::Pending(pending));
                    return Err(JoinFailure {
                        error: Error::Work(WorkError::Pending),
                        bootstrap: self,
                    });
                }
            },
        };
        let outcome = joined.outcome();
        self.state = ManuallyDrop::new(State::Joined(joined));
        if outcome != Outcome::CallbackReturned {
            return Err(JoinFailure {
                error: Error::ManagedOutcome(outcome),
                bootstrap: self,
            });
        }
        match self.result.get() {
            Some(0) => {}
            Some(error) => {
                return Err(JoinFailure {
                    error: Error::Native(error),
                    bootstrap: self,
                });
            }
            None => {
                return Err(JoinFailure {
                    error: Error::Work(WorkError::corrupt()),
                    bootstrap: self,
                });
            }
        }
        // SAFETY: exact private JoinedJob + expected returned body/result prove
        // all producer/native/source/stack/page/RCU and callback tails joined.
        // This atomic ABI enters no source task and invokes no callback.
        let status = unsafe { stela_linux_srcu_bootstrap_joined(self.kernel.raw()) };
        if status != 0 {
            return Err(JoinFailure {
                error: Error::Native(status),
                bootstrap: self,
            });
        }
        // SAFETY: same genuine joined job extracted once; it holds only Rust
        // kernel/result owners. No source ref destructor needs invented current.
        let State::Joined(joined) = (unsafe { ManuallyDrop::take(&mut self.state) }) else {
            unreachable!()
        };
        drop(joined.into_job());
        // SAFETY: actual owned job and tails joined. The kernel remains retained.
        drop(unsafe { ManuallyDrop::take(&mut self.result) });
        // SAFETY: transfer the sole retained Bootstrap kernel field to Ready.
        let kernel = unsafe { ManuallyDrop::take(&mut self.kernel) };
        Ok(Ready { kernel })
    }
}
