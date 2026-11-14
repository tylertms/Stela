//! One device shared by several clients: the locked disk, merged requests and
//! per-client windows.

use super::*;

/// One disk several owners share, each through its own [`Window`]: the
/// disk's request queue. The owners may run on different processors at once
/// (a guest's storage on any PE, the host's NOR and log stores on another),
/// so each reaches the disk under its lock, which no one holds while the
/// device works on queued commands: the owners' commands wait in order, the
/// device runs as many as it has room for, and whichever owner next looks
/// for a result collects every completed one and starts those waiting
/// ([`Window::finish`]). Consecutive waiting reads, or writes, of adjoining
/// blocks go to the device as one command, as Linux's block layer merges
/// them.
#[derive(Clone)]
pub struct Shared(pub(super) Arc<Disk>);

pub(super) struct Disk {
    pub(super) held: AtomicBool,
    pub(super) profile_callbacks: AtomicBool,
    pub(super) activity: profile::Activity,
    pub(super) device: UnsafeCell<Box<dyn BlockDevice>>,
    pub(super) requests: UnsafeCell<Requests>,
}

/// The owners' commands, by ticket: those waiting in order with the
/// counter when each was submitted, those the device runs and the results
/// no owner has taken yet; and the disk's accounting, with the counter
/// when the device last went from idle to running a command.
#[derive(Default)]
pub(super) struct Requests {
    pub(super) waiting: VecDeque<(u32, Command, u64)>,
    pub(super) running: Vec<Merged>,
    pub(super) finished: BTreeMap<u32, Finished>,
    pub(super) next: u32,
    pub(super) stats: Stats,
    pub(super) busy_since: u64,
}

/// A device command and its first ticket and read length. Additional tickets
/// carry their offset and length in the read's data. What the accounting
/// counts the command as, and when its first ticket was submitted.
pub(super) struct Merged {
    pub(super) tag: u32,
    pub(super) first: (u32, usize),
    pub(super) more: Vec<(u32, usize, usize)>,
    pub(super) accounted: (Direction, u64),
    pub(super) submitted: u64,
}

// SAFETY: a device reaches the disk only through its own state and the
// controller's registers, not through anything of the processor that made
// it, and `Shared::lock` lets one processor at a time reach it.
unsafe impl Send for Disk {}
// SAFETY: as for `Send`; every access holds the lock.
unsafe impl Sync for Disk {}

/// The disk, held until dropped.
pub(super) struct Held<'a>(pub(super) &'a Disk);

impl Deref for Held<'_> {
    type Target = Box<dyn BlockDevice>;
    fn deref(&self) -> &Self::Target {
        // SAFETY: the lock is held, so no other reference to the device lives.
        unsafe { &*self.0.device.get() }
    }
}

impl DerefMut for Held<'_> {
    fn deref_mut(&mut self) -> &mut Self::Target {
        // SAFETY: as for `deref`.
        unsafe { &mut *self.0.device.get() }
    }
}

impl Drop for Held<'_> {
    fn drop(&mut self) {
        self.0.held.store(false, Ordering::Release);
    }
}

/// `next` joined to the end of `command` when it continues it, within the
/// device's transfer limit: both reads, or both writes asking the same
/// durability. The joined length of `next` in a read's data, or `None`.
pub(super) fn merge(
    command: &mut Command,
    next: &Command,
    block: usize,
    limit: usize,
) -> Option<usize> {
    let limit = if limit == 0 { usize::MAX } else { limit };
    match (command, next) {
        (
            Command::Read { lba, bytes },
            Command::Read {
                lba: at,
                bytes: more,
            },
        ) if *lba + (*bytes / block) as u64 == *at && *bytes + more <= limit => {
            *bytes += more;
            Some(*more)
        }
        (
            Command::Write { lba, data, fua },
            Command::Write {
                lba: at,
                data: more,
                fua: also,
            },
        ) if fua == also
            && *lba + (data.len() / block) as u64 == *at
            && data.len() + more.len() <= limit =>
        {
            data.extend_from_slice(more);
            Some(0)
        }
        _ => None,
    }
}

/// Where the last of a merged command's requests ends in its data: each
/// request is its ticket, offset and bytes.
fn offset_end(requests: &[(u32, usize, usize)]) -> usize {
    requests
        .iter()
        .map(|&(_, offset, bytes)| offset + bytes)
        .max()
        .unwrap_or(0)
}

impl Held<'_> {
    pub(super) fn requests(&mut self) -> &mut Requests {
        // SAFETY: the lock is held; the requests are apart from the device.
        unsafe { &mut *self.0.requests.get() }
    }

    /// Whether no command waits or runs: a transfer may reach the device
    /// directly.
    pub(super) fn idle(&mut self) -> bool {
        let requests = self.requests();
        requests.running.is_empty() && requests.waiting.is_empty()
    }

    /// Queue `command` and start what the device has room for; its ticket.
    /// Completions are collected only when the device has no room, which
    /// they may free: a start needs no look at them. The device is told of
    /// what started unless `more` commands follow at once, when the caller
    /// tells it ([`Self::commit`]).
    pub(super) fn submit(&mut self, command: Command, more: bool) -> u32 {
        let requests = self.requests();
        let ticket = requests.next;
        requests.next = requests.next.wrapping_add(1);
        requests
            .waiting
            .push_back((ticket, command, crate::Clock::ticks()));
        self.step(false);
        if !self.requests().waiting.is_empty() {
            self.step(true);
        }
        if !more {
            self.commit();
        }
        ticket
    }

    /// Tell the device of every command started since it was last told
    /// ([`BlockDevice::commit`]).
    pub(super) fn commit(&mut self) {
        BlockDevice::commit(&mut **self);
    }

    /// Count a command `accounted` as, with `merges` requests joined into
    /// it, submitted at `submitted` and completed at `now`.
    pub(super) fn account(
        &mut self,
        (direction, bytes): (Direction, u64),
        merges: u64,
        submitted: u64,
        now: u64,
    ) {
        let counts = &mut self.requests().stats.directions[direction as usize];
        counts.ios += 1;
        counts.merges += merges;
        counts.bytes += bytes;
        counts.ticks += now.wrapping_sub(submitted);
    }

    /// Run `run` on the device, which runs nothing else meanwhile, as one
    /// command `accounted` as and submitted at `submitted`.
    pub(super) fn direct<T>(
        &mut self,
        accounted: (Direction, u64),
        submitted: u64,
        run: impl FnOnce(&mut Box<dyn BlockDevice>) -> T,
    ) -> T {
        let observing = self.0.profile_callbacks.load(Ordering::Relaxed);
        let started = crate::Clock::ticks();
        let disk = self.0;
        let _active = observing.then(|| disk.activity.direct.enter(started));
        let result = run(self);
        let now = crate::Clock::ticks();
        let stats = &mut self.requests().stats;
        stats.busy += now.wrapping_sub(started);
        stats.deepest = stats.deepest.max(1);
        if observing {
            self.requests()
                .stats
                .callbacks
                .get_or_insert_default()
                .direct
                .record(now.wrapping_sub(started), profile::Work::default());
        }
        self.account(accounted, 0, submitted, now);
        result
    }

    /// The disk's accounting, its device's time running a command so far
    /// included.
    pub(super) fn stats(&mut self) -> Stats {
        let requests = self.requests();
        let mut stats = requests.stats;
        if !requests.running.is_empty() {
            stats.busy += crate::Clock::ticks().wrapping_sub(requests.busy_since);
        }
        stats
    }

    /// Collect the device's completed commands, then start those waiting
    /// while it has room. A device that takes no queued commands runs each
    /// here, its caller waiting.
    pub(super) fn advance(&mut self) {
        self.step(true);
        self.commit();
    }

    /// Start the waiting commands, collecting completions first when `reap`.
    fn step(&mut self, reap: bool) {
        let observing = self.0.profile_callbacks.load(Ordering::Relaxed);
        let started = observing.then(crate::Clock::ticks);
        let disk = self.0;
        let _active = started.map(|ticks| disk.activity.advance.enter(ticks));
        let mut work = profile::Work::default();
        if reap {
            self.advance_inner(&mut work);
        } else {
            self.start_waiting(&mut work);
        }
        if let Some(started) = started {
            let elapsed = crate::Clock::ticks().wrapping_sub(started);
            self.requests()
                .stats
                .callbacks
                .get_or_insert_default()
                .advance
                .record(elapsed, work);
        }
    }

    pub(super) fn advance_inner(&mut self, work: &mut profile::Work) {
        self.reap(work);
        self.start_waiting(work);
    }

    /// Collect the device's completed commands into `finished`. A device
    /// that collects all of them in one finish is reached once; the other
    /// results are taken from what that finish collected.
    fn reap(&mut self, work: &mut profile::Work) {
        let together = self.reaps_together();
        let mut looked = false;
        let mut index = 0;
        while index < self.requests().running.len() {
            let tag = self.requests().running[index].tag;
            let result = if together && looked {
                self.finished(tag)
            } else {
                looked = true;
                self.finish(tag)
            };
            let Some(result) = result else {
                index += 1;
                continue;
            };
            work.finishes = work.finishes.saturating_add(1);
            let merged = self.requests().running.swap_remove(index);
            let now = crate::Clock::ticks();
            let merges = merged.more.len() as u64;
            self.account(merged.accounted, merges, merged.submitted, now);
            let requests = self.requests();
            if requests.running.is_empty() {
                requests.stats.busy += now.wrapping_sub(requests.busy_since);
            }
            let finished = &mut self.requests().finished;
            let (ticket, bytes) = merged.first;
            if merged.more.is_empty() {
                let result = result.and_then(|data| match data {
                    Some(data) if data.len() < bytes => {
                        Err(BlockError::Protocol("a read returned too little data"))
                    }
                    Some(mut data) => {
                        data.truncate(bytes);
                        Ok(Some(data))
                    }
                    None => Ok(None),
                });
                finished.insert(ticket, result);
                continue;
            }
            let tickets: Vec<_> = core::iter::once((ticket, 0, bytes))
                .chain(merged.more)
                .collect();
            match result {
                // Each request's part, split off the end so the first keeps
                // the command's buffer: only the later parts are copied.
                Ok(Some(mut data)) if data.len() >= offset_end(&tickets) => {
                    data.truncate(offset_end(&tickets));
                    for &(ticket, offset, _) in tickets[1..].iter().rev() {
                        let part = data.split_off(offset);
                        finished.insert(ticket, Ok(Some(part)));
                    }
                    finished.insert(tickets[0].0, Ok(Some(data)));
                }
                Ok(Some(_)) => {
                    for &(ticket, ..) in &tickets {
                        let short = BlockError::Protocol("a read returned too little data");
                        finished.insert(ticket, Err(short));
                    }
                }
                other => {
                    for &(ticket, ..) in &tickets {
                        finished.insert(ticket, other.clone());
                    }
                }
            }
        }
    }

    /// Start the waiting commands, in order, while the device has room. A
    /// device that takes no queued commands runs each here, its caller
    /// waiting.
    fn start_waiting(&mut self, work: &mut profile::Work) {
        if self.requests().waiting.is_empty() {
            return;
        }
        let queued = self.depth() > 0;
        let (block, features) = {
            let identity = self.identity();
            (identity.block_size as usize, identity.features)
        };
        loop {
            if queued && !self.room() {
                return;
            }
            let Some((ticket, mut command, submitted)) = self.requests().waiting.pop_front() else {
                return;
            };
            if !queued {
                let accounted = command.accounted(block);
                let result = self.direct(accounted, submitted, |device| device.execute(command));
                self.requests().finished.insert(ticket, result);
                continue;
            }
            let first = match &command {
                Command::Read { bytes, .. } => *bytes,
                _ => 0,
            };
            let read = matches!(command, Command::Read { .. });
            let limit = features.transfer_limit(read);
            let mut more = Vec::new();
            let mut offset = first;
            while let Some((next, following, _)) = self.requests().waiting.front() {
                let Some(bytes) = merge(&mut command, following, block, limit) else {
                    break;
                };
                more.push((*next, offset, bytes));
                offset += bytes;
                self.requests().waiting.pop_front();
            }
            let accounted = command.accounted(block);
            work.starts = work.starts.saturating_add(1);
            // The device is told of the batch once its caller commits.
            match self.start_more(command) {
                Ok(tag) => {
                    let requests = self.requests();
                    if requests.running.is_empty() {
                        requests.busy_since = crate::Clock::ticks();
                    }
                    requests.running.push(Merged {
                        tag,
                        first: (ticket, first),
                        more,
                        accounted,
                        submitted,
                    });
                    requests.stats.deepest = requests.stats.deepest.max(requests.running.len());
                }
                Err(error) => {
                    for ticket in
                        core::iter::once(ticket).chain(more.into_iter().map(|(id, ..)| id))
                    {
                        self.requests().finished.insert(ticket, Err(error.clone()));
                    }
                }
            }
        }
    }

    /// Ticket `ticket`'s result, once it has one.
    pub(super) fn take(&mut self, ticket: u32) -> Option<Finished> {
        self.advance();
        self.requests().finished.remove(&ticket)
    }
}

impl Shared {
    pub fn new(device: Box<dyn BlockDevice>) -> Self {
        Self(Arc::new(Disk {
            held: AtomicBool::new(false),
            profile_callbacks: AtomicBool::new(false),
            activity: profile::Activity::default(),
            device: UnsafeCell::new(device),
            requests: UnsafeCell::new(Requests::default()),
        }))
    }

    /// Observe callback wall time from subsequent operations. Default off;
    /// disabling keeps earlier observations and adds no timing reads.
    pub fn set_callback_profile(&self, enabled: bool) {
        self.0.profile_callbacks.store(enabled, Ordering::Relaxed);
    }

    /// Bounded observations without taking the disk's lock or touching a driver.
    /// Enabled samples bracket independent generation/start reads with counters;
    /// disabling adds no counter reads. None means profiling is disabled.
    /// Activity never grants completion, DMA or quiescence permission.
    pub fn callback_activity(&self) -> Option<CallbackActivity> {
        if !self.0.profile_callbacks.load(Ordering::Relaxed) {
            return None;
        }
        let observed_before_ticks = crate::Clock::ticks();
        let advance = self.0.activity.advance.snapshot();
        let direct = self.0.activity.direct.snapshot();
        let observed_after_ticks = crate::Clock::ticks();
        Some(CallbackActivity {
            observed_before_ticks,
            observed_after_ticks,
            advance,
            direct,
        })
    }

    /// Callback observations with one try-lock and no counter read.
    /// None means disabled, not yet observed, or currently held by an owner.
    /// This snapshot grants no DMA, mapping or quiescence permission.
    pub fn callback_stats(&self) -> Option<CallbackStats> {
        if !self.0.profile_callbacks.load(Ordering::Relaxed) {
            return None;
        }
        self.try_lock()?.requests().stats.callbacks
    }

    /// The disk, unless another owner is using it now: an owner that may
    /// not wait, as a service turn, which holds its processor inside one
    /// exit, comes back later. The holder may itself wait for a lock its
    /// device needs, such as the host drivers'.
    pub(super) fn try_lock(&self) -> Option<Held<'_>> {
        self.0
            .held
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok()
            .then(|| Held(&self.0))
    }

    /// Whether another owner is using the disk now.
    pub(super) fn held(&self) -> bool {
        self.0.held.load(Ordering::Relaxed)
    }

    /// The disk, once no other owner is using it.
    pub(super) fn lock(&self) -> Held<'_> {
        while self
            .0
            .held
            .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
        {
            crate::spinning();
        }
        Held(&self.0)
    }

    /// Run `command` to completion, the caller waiting: at once while no
    /// queued command is ahead of it, else in its turn.
    pub(super) fn run(&self, command: Command) -> Finished {
        let ticket = {
            let mut disk = self.lock();
            if disk.idle() {
                let accounted = command.accounted(disk.identity().block_size as usize);
                return disk.direct(accounted, crate::Clock::ticks(), |device| {
                    device.execute(command)
                });
            }
            disk.submit(command, false)
        };
        loop {
            if let Some(result) = self.lock().take(ticket) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// The disk's I/O accounting since it was shared.
    pub fn stats(&self) -> Stats {
        self.lock().stats()
    }

    /// Whether `other` shares this same disk.
    pub fn same(&self, other: &Self) -> bool {
        Arc::ptr_eq(&self.0, &other.0)
    }

    /// Shut the disk down, as [`BlockDevice::shutdown`]; once for all its
    /// windows, after every queued command.
    pub fn shutdown(&self, power_off: bool) -> Result<(), BlockError> {
        let mut disk = self.lock();
        while !disk.idle() {
            disk.advance();
            core::hint::spin_loop();
        }
        disk.shutdown(power_off)
    }

    /// Every block, as a device of its own.
    pub fn whole(&self) -> Window {
        let identity = self.lock().identity().clone();
        Window {
            disk: self.clone(),
            first: 0,
            identity,
            started: Vec::new(),
            uncommitted: false,
        }
    }

    /// The `blocks` blocks from `first`, as a device of their own.
    pub fn window(&self, first: u64, blocks: u64) -> Result<Window, BlockError> {
        let mut identity = self.lock().identity().clone();
        match first.checked_add(blocks) {
            Some(end) if end <= identity.blocks => (),
            _ => return Err(range(&identity, first, blocks)),
        }
        identity.blocks = blocks;
        Ok(Window {
            disk: self.clone(),
            first,
            identity,
            started: Vec::new(),
            uncommitted: false,
        })
    }

    /// The blocks of `runs`, each a first block and a count, in order, as
    /// one device: a file's clusters where they lie on the disk.
    pub fn runs(&self, runs: Vec<(u64, u64)>) -> Result<Runs, BlockError> {
        let mut identity = self.lock().identity().clone();
        let mut blocks = 0u64;
        for &(first, count) in &runs {
            match first.checked_add(count) {
                Some(end) if end <= identity.blocks => blocks += count,
                _ => return Err(range(&identity, first, count)),
            }
        }
        identity.blocks = blocks;
        Ok(Runs {
            disk: self.clone(),
            runs,
            identity,
            started: Vec::new(),
            next: 0,
            uncommitted: false,
        })
    }
}

/// The commands a [`Window`] keeps in its disk's queue at most.
pub(super) const WINDOW_DEPTH: usize = 64;

/// A run of blocks of a [`Shared`] disk.
///
/// A look never waits for the disk while another owner uses it, as that
/// owner may itself wait long for a lock its device needs (the host
/// drivers'): [`BlockDevice::finish`] and [`BlockDevice::finished`] answer
/// nothing yet, [`BlockDevice::room`] no room, [`BlockDevice::posted`] that
/// a finish may find a result, and a commit is made by the next call that
/// takes the disk. A start, once room said the disk was free, takes it.
pub struct Window {
    pub(super) disk: Shared,
    pub(super) first: u64,
    pub(super) identity: Identity,
    /// The tickets of this window's started commands not yet finished.
    pub(super) started: Vec<u32>,
    /// A commit put off while another owner held the disk.
    pub(super) uncommitted: bool,
}

impl Window {
    /// The disk this window is part of.
    pub fn shared(&self) -> &Shared {
        &self.disk
    }

    pub(super) fn run(&self, command: Command) -> Result<(), BlockError> {
        let command = command.within(&self.identity, self.first)?;
        self.disk.run(command).map(drop)
    }
}

impl Window {
    /// Queue `command` on the disk; its ticket. The device is told of it at
    /// once unless `more` follow ([`BlockDevice::commit`]).
    fn submit(&mut self, command: Command, more: bool) -> Result<u32, BlockError> {
        let command = command.within(&self.identity, self.first)?;
        // Only the window's own depth refuses. Another owner that took the
        // disk since `room` answered is waited for: a start refused for it
        // would fail a store whose caller had found room.
        if self.started.len() >= WINDOW_DEPTH {
            return Err(BlockError::Protocol("the disk window queue is full"));
        }
        let ticket = self.disk.lock().submit(command, more);
        // Without more to follow it told the device of every start.
        self.uncommitted &= more;
        self.started.push(ticket);
        Ok(ticket)
    }
}

impl BlockDevice for Window {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        let mut disk = self.disk.lock();
        if disk.idle() {
            let accounted = (Direction::Read, buffer.len() as u64);
            let lba = self.first + lba;
            return disk.direct(accounted, crate::Clock::ticks(), |device| {
                device.read(lba, buffer)
            });
        }
        drop(disk);
        let command = Command::Read {
            lba: self.first + lba,
            bytes: buffer.len(),
        };
        let data = self.disk.run(command)?;
        buffer.copy_from_slice(&data.ok_or(BlockError::Protocol("a read returned no data"))?);
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.identity.check_write(lba, data.len())?;
        let mut disk = self.disk.lock();
        if disk.idle() {
            let accounted = (Direction::Write, data.len() as u64);
            let lba = self.first + lba;
            return disk.direct(accounted, crate::Clock::ticks(), |device| {
                device.write(lba, data)
            });
        }
        drop(disk);
        self.run(Command::Write {
            lba,
            data: data.to_vec(),
            fua: false,
        })
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.run(Command::Flush)
    }

    fn write_fua(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.run(Command::Write {
            lba,
            data: data.to_vec(),
            fua: true,
        })
    }

    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        self.run(Command::discard(lba, blocks))
    }

    fn write_zeroes(&mut self, lba: u64, blocks: u64, unmap: bool) -> Result<(), BlockError> {
        self.run(Command::WriteZeroes { lba, blocks, unmap })
    }

    /// Every window takes queued commands: its disk runs them in turn.
    fn depth(&self) -> usize {
        WINDOW_DEPTH
    }

    fn room(&self) -> bool {
        self.started.len() < WINDOW_DEPTH && !self.disk.held()
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        self.submit(command, false)
    }

    fn start_more(&mut self, command: Command) -> Result<u32, BlockError> {
        self.submit(command, true)
    }

    fn commit(&mut self) {
        match self.disk.try_lock() {
            Some(mut disk) => disk.commit(),
            None => self.uncommitted = true,
        }
    }

    /// Takes the disk's completions and commits what waits.
    fn finish(&mut self, tag: u32) -> Option<Finished> {
        let finished = {
            let mut disk = self.disk.try_lock()?;
            self.uncommitted = false;
            disk.take(tag)
        }?;
        self.started.retain(|&ticket| ticket != tag);
        Some(finished)
    }

    /// A finish collects every completed command of the disk.
    fn reaps_together(&self) -> bool {
        true
    }

    fn finished(&mut self, tag: u32) -> Option<Finished> {
        let finished = self.disk.try_lock()?.requests().finished.remove(&tag)?;
        self.started.retain(|&ticket| ticket != tag);
        Some(finished)
    }

    /// One of this window's commands has a result, or the disk has posted
    /// a completion: another window's results never count.
    fn posted(&self) -> bool {
        let Some(mut disk) = self.disk.try_lock() else {
            return true;
        };
        let finished = &disk.requests().finished;
        self.started
            .iter()
            .any(|ticket| finished.contains_key(ticket))
            || disk.posted()
    }

    fn signals(&self) -> bool {
        self.disk.lock().signals()
    }
}
