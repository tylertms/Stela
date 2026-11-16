//! The host disk under ANS's namespaces.
//!
//! A host transfer takes as long as the host's storage does. Each I/O
//! command's transfer is submitted where the guest rings the doorbell, as
//! many at once as the guest queues, through the block layer
//! ([`Submission`]), which runs them on the disk as it has room. The
//! controller completes each command once the disk has completed its
//! transfer, in whatever order that happens, as a real controller completes
//! commands once its media has. It looks once the disk has posted a
//! completion. A disk that signals its completions wakes the core that
//! looks, so it looks otherwise only for a lost signal; for one that does
//! not, it looks after half the disk's recent latency, then more often
//! (Linux's hybrid polling). Nothing waits for the disk and nothing wakes
//! while no transfer runs. A disk that runs nothing queued executes each
//! command on the calling PE. One poll admits at most 64 commands, so larger
//! transfers keep their partial progress until later polls. The boot stages
//! the guest's disk: there a submission and a look only queue commands and
//! take results, and the host I/O runs outside the devices' lock.
//!
//! A mounting APFS deallocates tens of thousands of ranges, which already
//! read as zeros ([`super::namespace::Medium::deallocate`]), so a Deallocate
//! completes at once and the disk hears of them in the background, as
//! Linux's `fstrim` tells it in batches: [`Discards`] keeps them merged and
//! sends a batch once guest transfers have paused for [`DISCARD_QUIET`]: a
//! disk runs a large deallocation for milliseconds, and a read queued behind
//! it waits that long. A background batch is one command of as many ranges
//! as the disk takes (256 on NVMe, as Linux's limit), so a read that comes
//! while it runs waits for one command at most, and the next batch waits for
//! the next pause. A flush that waits for discards sends them at once and
//! keeps [`DISCARD_COMMANDS`] commands on the disk, starting the next as
//! soon as one completes rather than once a whole group has: one slow
//! deallocation does not hold back the rest.
//! Discards never reorder past
//! writes. A write takes its blocks back from those waiting and waits out a
//! batch it overlaps, and a flush waits for all of them where the guest was
//! told deallocated blocks stay zeros, as a controller's cache holds them.
use alloc::{boxed::Box, collections::BTreeMap, format, string::String, vec::Vec};
use stela_drivers::block::{
    BlockDevice, BlockError, Extent, Finished, Identity, Request, Submission,
};

/// The latency assumed before any transfer completed: 100 us of the 24 MHz
/// counter.
const FIRST_LATENCY: u64 = 2_400;
/// The shortest wait between two looks: 10 us.
const SHORTEST_POLL: u64 = 240;
/// How long a signalling disk's transfer may go unsignalled before a look
/// finds a lost completion: 1 ms.
const LOST_SIGNAL: u64 = 24_000;

/// The discard commands kept on the disk while a transfer waits for
/// discards or a shutdown hurries them, each one batch.
const DISCARD_COMMANDS: usize = 16;
/// How long guest transfers must have paused before a discard batch goes to
/// the disk: 10 ms.
const DISCARD_QUIET: u64 = 240_000;

/// Deallocated blocks the disk has not heard of.
#[derive(Default)]
struct Discards {
    /// Merged runs of host blocks: each one's first block and its end.
    waiting: BTreeMap<u64, u64>,
    /// The batches on the disk and their runs: at most one in the
    /// background, [`DISCARD_COMMANDS`] while urgent.
    running: Vec<(Submission, Vec<Extent>)>,
    /// Transfers waiting for discards: each one's key, when it was
    /// submitted and its request.
    held: Vec<(u32, u64, Request)>,
    /// Why the disk refused a batch. Discards are hints, so it hears no
    /// more; a flush has nothing to wait for.
    failed: Option<BlockError>,
    /// When a guest transfer last started or completed, if one has.
    active: Option<u64>,
    /// The controller's shutdown waits for these: batches go at once,
    /// [`DISCARD_COMMANDS`] on the disk, as for a waiting transfer.
    hurried: bool,
    /// Runs a shutdown gave up on, which the disk never heard of.
    abandoned: u64,
}

impl Discards {
    /// Merge `start..end` into the waiting runs.
    fn add(&mut self, mut start: u64, mut end: u64) {
        if let Some((&first, &last)) = self.waiting.range(..=start).next_back()
            && last >= start
        {
            start = first;
            end = end.max(last);
        }
        while let Some((&first, &last)) = self.waiting.range(start..=end).next() {
            end = end.max(last);
            self.waiting.remove(&first);
        }
        self.waiting.insert(start, end);
    }

    /// A write takes `start..end` back from the waiting runs. Whether it
    /// overlaps a running batch, which it must wait out.
    fn written(&mut self, start: u64, end: u64) -> bool {
        let before = self
            .waiting
            .range(..start)
            .next_back()
            .filter(|&(_, &last)| last > start)
            .map(|(&first, _)| first);
        if let Some(first) = before {
            let last = self.waiting.insert(first, start).expect("a waiting run");
            if last > end {
                self.waiting.insert(end, last);
            }
        }
        while let Some((&first, &last)) = self.waiting.range(start..end).next() {
            self.waiting.remove(&first);
            if last > end {
                self.waiting.insert(end, last);
            }
        }
        self.running.iter().any(|(_, extents)| {
            extents
                .iter()
                .any(|extent| extent.lba < end && extent.lba + extent.blocks > start)
        })
    }

    /// Up to `most` waiting runs for the disk, first blocks first.
    fn batch(&mut self, most: usize) -> Vec<Extent> {
        let mut extents = Vec::new();
        while extents.len() < most
            && let Some((lba, end)) = self.waiting.pop_first()
        {
            extents.push(Extent {
                lba,
                blocks: end - lba,
            });
        }
        extents
    }

    fn outstanding(&self) -> bool {
        !self.waiting.is_empty() || !self.running.is_empty()
    }

    /// Whether a transfer waits for discards or a shutdown hurries them.
    fn urgent(&self) -> bool {
        !self.held.is_empty() || self.hurried
    }

    /// The batches the disk may run at once: one in the background, which
    /// a guest read may come to wait behind, and [`DISCARD_COMMANDS`] while
    /// urgent.
    fn batches(&self) -> usize {
        if self.urgent() { DISCARD_COMMANDS } else { 1 }
    }

    /// When the next batch may go at counter `now`, at the earliest: at once
    /// while a transfer waits for discards, else once guest transfers have
    /// paused for [`DISCARD_QUIET`].
    fn due(&self, now: u64) -> u64 {
        match self.active {
            Some(active) if self.held.is_empty() && !self.hurried => {
                active.saturating_add(DISCARD_QUIET)
            }
            _ => now,
        }
    }
}

pub(super) struct HostDisk {
    device: Box<dyn BlockDevice>,
    /// The running transfers: each one's key, when it was submitted in
    /// counter ticks, and its commands.
    running: Vec<(u32, u64, Submission)>,
    /// What a transfer took, a moving average in counter ticks.
    latency: u64,
    discards: Discards,
}

impl HostDisk {
    pub(super) fn new(device: Box<dyn BlockDevice>) -> Self {
        Self {
            device,
            running: Vec::new(),
            latency: FIRST_LATENCY,
            discards: Discards::default(),
        }
    }

    pub(super) fn identity(&self) -> &Identity {
        self.device.identity()
    }

    /// The host disk has a volatile write cache, so a write is durable
    /// only after a flush.
    pub(super) fn write_cache(&self) -> bool {
        self.identity().features.write_cache
    }

    /// Deallocating host blocks leaves them reading as zeros for good: the
    /// disk's Write Zeroes may deallocate.
    pub(super) fn deallocates_zeroed(&self) -> bool {
        self.identity()
            .features
            .write_zeroes
            .is_some_and(|zeroes| zeroes.unmap)
    }

    #[cfg(test)]
    pub(super) fn read(&mut self, address: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.device.read(address, buffer)
    }
    #[cfg(test)]
    pub(super) fn write(&mut self, address: u64, data: &[u8]) -> Result<(), BlockError> {
        self.device.write(address, data)
    }

    /// Tell the disk to deallocate `blocks` blocks from `lba`, in the
    /// background, if it takes that and has not refused it.
    pub(super) fn discard(&mut self, lba: u64, blocks: u64) {
        if self.identity().features.discard.is_some() && self.discards.failed.is_none() {
            self.discards.add(lba, lba + blocks);
        }
    }

    /// Send the waiting discards without a pause in transfers while
    /// `hurried`: the controller's shutdown waits for them.
    pub(super) fn hurry_discards(&mut self, hurried: bool) {
        self.discards.hurried = hurried;
    }

    /// Forget the discards not yet sent, and say how many runs they were.
    /// Discards are hints (NVMe 1.4 section 6.7.1.1), and the disk keeps
    /// those blocks' old contents. The batches on the disk still complete.
    pub(super) fn abandon_discards(&mut self) -> usize {
        let runs = self.discards.waiting.len();
        self.discards.waiting.clear();
        self.discards.abandoned += runs as u64;
        runs
    }

    /// Whether background discards remain: they need [`Self::poll`] though
    /// no command waits for them.
    pub(super) fn discarding(&self) -> bool {
        self.discards.outstanding()
    }

    /// Whether `request` must wait for discards. A write takes its blocks
    /// back whatever the answer.
    fn held(&mut self, request: &Request) -> bool {
        match request {
            Request::Write { lba, data, .. } => {
                let block = u64::from(self.identity().block_size);
                self.discards
                    .written(*lba, *lba + data.len() as u64 / block)
            }
            Request::WriteZeroes { lba, blocks, .. } => self.discards.written(*lba, *lba + *blocks),
            Request::Flush => self.discards.outstanding() && self.deallocates_zeroed(),
            _ => false,
        }
    }

    /// Submit `request` under `key` at counter `now`: its result if the
    /// first poll finishes it, else `None` until
    /// [`Self::poll`] gives it. Synchronous providers can also yield after
    /// the block layer's command admission limit.
    pub(super) fn submit(&mut self, key: u32, request: Request, now: u64) -> Option<Finished> {
        if self.held(&request) {
            self.discards.held.push((key, now, request));
            return None;
        }
        self.discards.active = Some(now);
        let mut submission = match Submission::new(request, self.device.identity()) {
            Ok(submission) => submission,
            Err(error) => return Some(Err(error)),
        };
        if let Some(done) = submission.poll(&mut *self.device) {
            return Some(done);
        }
        self.running.push((key, now, submission));
        None
    }

    /// The transfers the disk has completed by counter `now`, by key; never
    /// waits.
    pub(super) fn poll(&mut self, now: u64) -> Vec<(u32, Finished)> {
        let mut done = Vec::new();
        let mut index = 0;
        while index < self.running.len() {
            let Some(finished) = self.running[index].2.poll(&mut *self.device) else {
                index += 1;
                continue;
            };
            let (key, started, _) = self.running.swap_remove(index);
            let took = now.saturating_sub(started);
            self.latency = (self.latency * 7 + took) / 8;
            self.discards.active = Some(now);
            done.push((key, finished));
        }
        self.advance_discards(&mut done, now);
        done
    }

    /// Start discard batches and reap those the disk completed: in the
    /// background one, once guest transfers paused (`Discards::due`) and
    /// none runs; while urgent, a new one in each free slot of
    /// [`DISCARD_COMMANDS`], so a batch waits for no other. Then resubmit
    /// the transfers that waited: a write beside the running batches goes,
    /// a flush waits on until none remains.
    fn advance_discards(&mut self, done: &mut Vec<(u32, Finished)>, now: u64) {
        loop {
            self.start_discards(now);
            let before = self.discards.running.len();
            let mut index = 0;
            while index < self.discards.running.len() {
                let Some(result) = self.discards.running[index].0.poll(&mut *self.device) else {
                    index += 1;
                    continue;
                };
                self.discards.running.swap_remove(index);
                if let Err(error) = result {
                    self.refuse_discards(error);
                }
            }
            // A slot an urgent batch freed takes the next at once; with a
            // disk that runs commands synchronously this ends once none
            // waits.
            let freed = self.discards.running.len() < before;
            if !(freed && self.discards.urgent() && !self.discards.waiting.is_empty()) {
                break;
            }
        }
        for (key, started, request) in core::mem::take(&mut self.discards.held) {
            if let Some(finished) = self.submit(key, request, started) {
                done.push((key, finished));
            }
        }
    }

    /// Start the discard batches due at counter `now`: each one command of
    /// as many ranges as the disk takes.
    fn start_discards(&mut self, now: u64) {
        let Some(limits) = self.identity().features.discard else {
            return;
        };
        if !self.discards.urgent() && (!self.running.is_empty() || now < self.discards.due(now)) {
            return;
        }
        while self.discards.running.len() < self.discards.batches()
            && self.discards.failed.is_none()
        {
            let extents = self.discards.batch(limits.ranges.max(1));
            if extents.is_empty() {
                return;
            }
            let request = Request::Discard {
                extents: extents.clone(),
            };
            match Submission::new(request, self.device.identity()) {
                Ok(submission) => self.discards.running.push((submission, extents)),
                Err(error) => self.refuse_discards(error),
            }
        }
    }

    /// The disk refused a discard: name why ([`Self::diagnostic`]) and
    /// send no more.
    fn refuse_discards(&mut self, error: BlockError) {
        self.discards.failed = Some(error);
        self.discards.waiting.clear();
    }

    /// Whether a running transfer's completion is posted, ready for
    /// [`Self::poll`]; a look at memory alone.
    pub(super) fn posted(&self) -> bool {
        (!self.running.is_empty() || !self.discards.running.is_empty()) && self.device.posted()
    }

    /// The running transfers at counter `now`, oldest first, each as
    /// key/age in counter ticks/commands the disk runs for it (none while
    /// it waits for room), then the disk's latency and readiness.
    pub(super) fn diagnostic(&self, now: u64) -> String {
        let mut running: Vec<_> = self.running.iter().collect();
        running.sort_unstable_by_key(|(_, started, _)| *started);
        let oldest = running
            .iter()
            .take(super::DIAGNOSTIC_COMMANDS)
            .map(|(key, started, submission)| {
                let age = now.saturating_sub(*started);
                format!("{key:#x}/{age}/{}", submission.started())
            })
            .collect::<Vec<_>>()
            .join(",");
        format!(
            "running={} oldest=[{oldest}] latency={} room={} posted={} signals={} \
             discards={}+{} held={} abandoned={} refused={:?}",
            self.running.len(),
            self.latency,
            self.device.room(),
            self.device.posted(),
            self.device.signals(),
            self.discards.waiting.len(),
            self.discards
                .running
                .iter()
                .map(|(_, runs)| runs.len())
                .sum::<usize>(),
            self.discards.held.len(),
            self.discards.abandoned,
            self.discards.failed,
        )
    }

    /// When to look for the running transfers' completions next, at counter
    /// `now`: half the latency after the oldest began, then more often; for
    /// a disk that signals them, only once a signal is overdue.
    pub(super) fn next_look(&self, now: u64) -> u64 {
        // Only discards wait for the pause that sends them.
        if self.running.is_empty() && self.discards.running.is_empty() {
            let due = self.discards.due(now);
            if due > now {
                return due;
            }
        }
        let oldest = self.running.iter().map(|(_, started, _)| *started).min();
        if self.device.signals() {
            let overdue = (self.latency * 4).max(LOST_SIGNAL);
            let due = oldest.unwrap_or(now).saturating_add(overdue);
            return if now < due {
                due
            } else {
                now.saturating_add(LOST_SIGNAL)
            };
        }
        let half = oldest.unwrap_or(now).saturating_add(self.latency / 2);
        if now < half {
            half
        } else {
            now.saturating_add((self.latency / 8).max(SHORTEST_POLL))
        }
    }
}

#[cfg(test)]
mod tests;
