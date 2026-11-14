//! `GetBytes` calls finished outside the turn.
//!
//! A texture only host image copies reach lies in device memory the host
//! maps uncached, which a CPU read crosses at about a megabyte a second.
//! Its call's turn starts a GPU copy of the next band into host-cached
//! memory ([`crate::metal::device::resources::ReadBack`]) and answers the
//! call `Pending`. The call made again finds the copy complete from the
//! published completion, holding no turn, and copies the band to its caller
//! outside every lock; a band after it starts in the call's next turn. No
//! other process's call waits for the copy or for the CPU's.
use super::{
    POLL_UNAVAILABLE, SharedService, Spin, answer, answer_pending, read_call, read_record,
};
use crate::metal::{
    device::resources::ReadBack,
    service::{Service, StartedReadBack, wire::Status},
    shader::Compiler,
    vulkan::backend::Backend,
};
use alloc::{sync::Arc, vec::Vec};
use core::{
    num::NonZeroU64,
    sync::atomic::{AtomicU32, AtomicU64, Ordering},
};
use stela_contracts::gate::{CallerFault, CallerMemory};

/// More calls than wait for their read-backs at once: past this the oldest
/// is forgotten, and made again it reads from its first band.
pub(super) const WAITING: usize = 32;

/// The read-backs of calls answered `Pending`, and those copied out, whose
/// memory the next turn frees.
pub(super) struct ReadBacks {
    waiting: Spin<Vec<Waiting>>,
    spent: Spin<Vec<(u64, ReadBack)>>,
    pub(super) generation: Arc<Generation>,
}

impl ReadBacks {
    pub(super) fn new() -> Self {
        Self {
            waiting: Spin::new(Vec::new()),
            spent: Spin::new(Vec::new()),
            generation: Arc::new(Generation::default()),
        }
    }
}

/// A `GetBytes` call answered `Pending`, by its connection, record,
/// operation and request, which the frontend calls again with.
struct Waiting {
    key: (u64, u64, u32, u64),
    /// The device's [`Generation`] when the read-back started.
    epoch: u64,
    state: State,
}

enum State {
    /// The band's copy runs or ran.
    Copying(StartedReadBack),
    /// The bands before this one were copied to the caller: the next turn
    /// starts its copy.
    From(usize),
}

/// The generation of the device read-backs come from, and the processors
/// copying out of one now. A device reopened after a loss ends the
/// generation before the lost device goes, waiting for those processors,
/// so no copy outlives the memory, and nothing of it is freed on its
/// successor.
#[derive(Default)]
pub(crate) struct Generation {
    epoch: AtomicU64,
    readers: AtomicU32,
}

impl Generation {
    pub(super) fn current(&self) -> u64 {
        self.epoch.load(Ordering::SeqCst)
    }

    /// The device of this generation goes: once no processor copies out of
    /// its memory.
    pub(crate) fn end(&self) {
        self.epoch.fetch_add(1, Ordering::SeqCst);
        while self.readers.load(Ordering::SeqCst) != 0 {
            core::hint::spin_loop();
        }
    }

    /// Runs `body` unless the device of generation `epoch` went.
    fn read<R>(&self, epoch: u64, body: impl FnOnce() -> R) -> Option<R> {
        self.readers.fetch_add(1, Ordering::SeqCst);
        let result = (self.epoch.load(Ordering::SeqCst) == epoch).then(body);
        self.readers.fetch_sub(1, Ordering::SeqCst);
        result
    }
}

/// What a call's read-back answered before its turn.
pub(super) enum Step {
    /// The call has none: its turn serves it.
    Not,
    /// The call was answered.
    Answered(Status),
    /// The call's bands before this one reached its caller: its turn
    /// starts the copy of this one.
    From(usize),
}

impl<B: Backend, C: Compiler + 'static> SharedService<B, C> {
    /// Finishes the read-back of the call at `record` when its copy
    /// completed, outside every lock, or answers it `Pending` when the copy
    /// runs. A call with no read-back is left to its turn.
    pub(super) fn read_back_step(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
    ) -> Result<Step, CallerFault> {
        if self.read_backs.waiting.with(|waiting| waiting.is_empty()) {
            return Ok(Step::Not);
        }
        let Some(call) = read_call(record, caller) else {
            return Ok(Step::Not);
        };
        let key = (
            connection.get(),
            record,
            call.operation,
            call.request.address,
        );
        let found = self.read_backs.waiting.with(|waiting| {
            let at = waiting.iter().position(|entry| entry.key == key)?;
            Some(waiting.remove(at))
        });
        let Some(waiting) = found else {
            return Ok(Step::Not);
        };
        let started = match waiting.state {
            State::From(band) => return Ok(Step::From(band)),
            State::Copying(started) => started,
        };
        let epoch = waiting.epoch;
        match self.read_back_done(epoch, &started.read_back) {
            Some(true) => {}
            Some(false) => {
                self.wait(key, epoch, State::Copying(started));
                answer_pending(record, call, caller)?;
                return Ok(Step::Answered(Status::Pending));
            }
            // The device was lost or replaced: the call's turn answers it.
            None => {
                self.retire_read_back(epoch, started.read_back);
                return Ok(Step::Not);
            }
        }
        let span = started.span;
        let copied = self.read_backs.generation.read(epoch, || {
            started
                .read_back
                .write_out(|offset, bytes| caller.write(span.address.wrapping_add(offset), bytes))
        });
        let status = match copied {
            None => {
                self.retire_read_back(epoch, started.read_back);
                return Ok(Step::Not);
            }
            Some(Ok(())) => {
                let (next, last) = (started.band + 1, started.last);
                self.retire_read_back(epoch, started.read_back);
                if !last {
                    return Ok(Step::From(next));
                }
                Status::Ok
            }
            // The frontend maps the span and calls again: the copy stays.
            Some(Err(CallerFault::Denied { .. })) => {
                self.wait(key, epoch, State::Copying(started));
                Status::Unmapped
            }
            Some(Err(fault)) => {
                self.retire_read_back(epoch, started.read_back);
                match fault {
                    CallerFault::Unsupported(_) => Status::Unsupported,
                    _ => Status::Malformed,
                }
            }
        };
        answer(record, call, status, caller)?;
        Ok(Step::Answered(status))
    }

    /// Whether `read_back`'s copy completed: from the published completion
    /// when it shows, else from a turn's poll unless a call holds the
    /// service. `None` once its device was lost or replaced.
    fn read_back_done(&self, epoch: u64, read_back: &ReadBack) -> Option<bool> {
        if self.read_backs.generation.current() != epoch {
            return None;
        }
        self.publish_marker();
        let reached = self.reached.load(Ordering::Acquire);
        if reached != POLL_UNAVAILABLE && reached >= read_back.serial() {
            return Some(true);
        }
        // A waiter publishes it, with no driver query here.
        if reached != POLL_UNAVAILABLE && self.publishable.load(Ordering::Acquire) {
            return Some(false);
        }
        let Some(mut turn) = self.try_turn() else {
            return Some(false);
        };
        if self.read_backs.generation.current() != epoch {
            return None;
        }
        let mut done = Ok(false);
        (self.host.run)(self.host.stack, &mut || {
            done = turn.device().read_back_complete(read_back);
        });
        drop(turn);
        done.ok()
    }

    /// Keeps the call of `key` waiting in `state`: past [`WAITING`] calls
    /// the oldest is forgotten.
    fn wait(&self, key: (u64, u64, u32, u64), epoch: u64, state: State) {
        let forgotten = self.read_backs.waiting.with(|waiting| {
            let forgotten = (waiting.len() >= WAITING).then(|| waiting.remove(0));
            waiting.push(Waiting { key, epoch, state });
            forgotten
        });
        if let Some(Waiting {
            epoch,
            state: State::Copying(started),
            ..
        }) = forgotten
        {
            self.retire_read_back(epoch, started.read_back);
        }
    }

    /// Keeps the read-back the call at `record` started in its turn, which
    /// answered it `Pending`, of the device of generation `epoch`.
    pub(super) fn started_read_back(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
        started: StartedReadBack,
        epoch: u64,
    ) {
        match read_record(record, caller) {
            Ok(call) => self.wait(
                (
                    connection.get(),
                    record,
                    call.operation,
                    call.request.address,
                ),
                epoch,
                State::Copying(started),
            ),
            Err(_) => self.retire_read_back(epoch, started.read_back),
        }
    }

    /// The bands before `band` of the call at `record` reached its caller,
    /// which a turn did not serve: the call made again starts at `band`.
    pub(super) fn resume_read_back(
        &self,
        connection: NonZeroU64,
        record: u64,
        caller: &mut dyn CallerMemory,
        band: usize,
    ) {
        if let Some(call) = read_call(record, caller) {
            let key = (
                connection.get(),
                record,
                call.operation,
                call.request.address,
            );
            let epoch = self.read_backs.generation.current();
            self.wait(key, epoch, State::From(band));
        }
    }

    fn retire_read_back(&self, epoch: u64, read_back: ReadBack) {
        self.read_backs
            .spent
            .with(|spent| spent.push((epoch, read_back)));
    }

    /// Forgets `connection`'s read-backs: it closed.
    pub(super) fn forget_read_backs(&self, connection: NonZeroU64) {
        let mut forgotten = Vec::new();
        self.read_backs.waiting.with(|waiting| {
            waiting.retain_mut(|entry| {
                let keep = entry.key.0 != connection.get();
                if !keep {
                    let state = core::mem::replace(&mut entry.state, State::From(0));
                    if let State::Copying(started) = state {
                        forgotten.push((entry.epoch, started.read_back));
                    }
                }
                keep
            });
        });
        self.read_backs.spent.with(|spent| spent.extend(forgotten));
    }

    /// Frees the memory of read-backs copied out or forgotten, in a turn on
    /// the service's stack. Those of a device since replaced went with it.
    pub(super) fn free_read_backs(&self, service: &mut Service<B, C>) {
        let spent = self.read_backs.spent.with(core::mem::take);
        let epoch = self.read_backs.generation.current();
        for (made, read_back) in spent {
            if made == epoch {
                service.device().end_read_back(read_back);
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::Generation;

    #[test]
    fn no_copy_outlives_its_generation() {
        let generation = Generation::default();
        let epoch = generation.current();
        assert_eq!(generation.read(epoch, || 7), Some(7));
        generation.end();
        assert_eq!(generation.read(epoch, || 7), None, "its device went");
        assert_eq!(generation.read(generation.current(), || 7), Some(7));
    }
}
