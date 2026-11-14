// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2011-2014, Intel Corporation.
//! The I/O queue: publishing tagged commands, collecting completions, deadlines
//! and aborts.

use super::*;

/// Borrowed only during a synchronous poll. The controller's tags and DMA
/// descriptors never retain the caller's buffer.
struct ReadDestination<'a> {
    tag: usize,
    bytes: &'a mut [u8],
}

impl Controller {
    /// Before the CPU takes back memory an admin command's data completion
    /// released: an owned controller's completion counts only while its
    /// registers answer without a fatal status, which a removed hierarchy or
    /// a failed engine would not.
    pub(super) fn trust_completion(&self) -> Result<(), BlockError> {
        match self.owned {
            Some(_) => self.check_fatal(),
            None => Ok(()),
        }
    }

    /// The error for an I/O completion whose entry changed under its read:
    /// the controller's own, when its status says it is removed or fatally
    /// failed, otherwise a corrupt queue.
    fn invalid_completion(&self) -> BlockError {
        self.stray_completion()
            .err()
            .unwrap_or(BlockError::Device { status: 0xffff })
    }

    /// A completion that names no outstanding command, which Linux's
    /// `nvme_handle_cqe` reports as an invalid ID and passes over: it is
    /// passed over while the controller's status is good, each one counted
    /// as a [`parts::Part::Status`] read. A command it should have ended
    /// still runs to its deadline, whose Abort and reset recover it as
    /// Linux's timeout does. A removed or fatally failed controller's status
    /// is the error.
    pub(super) fn stray_completion(&self) -> Result<(), BlockError> {
        parts::timed(parts::Part::Status, || self.check_fatal())
    }

    /// Page `index` of those tag `tag` holds.
    pub(super) fn tag_page(&mut self, tag: usize, index: usize) -> &mut Buffer {
        &mut self.pool.pages[usize::from(self.held[tag][index])]
    }

    /// Give the controller tag `tag`'s pages.
    fn give_tag(&mut self, tag: usize) -> Result<(), BlockError> {
        for &page in &self.held[tag] {
            self.pool.pages[usize::from(page)].give()?;
        }
        Ok(())
    }

    /// Take tag `tag`'s pages back from the controller.
    ///
    /// # Safety
    /// The controller completed the tag's command, or acknowledged that it
    /// stopped.
    unsafe fn take_tag(&mut self, tag: usize) -> Result<(), BlockError> {
        for &page in &self.held[tag] {
            // SAFETY: as the caller establishes.
            unsafe { self.pool.pages[usize::from(page)].take()? };
        }
        Ok(())
    }

    /// Copy the first `bytes` of tag `tag`'s data pages into `into`.
    fn gather(&self, tag: usize, into: &mut [u8]) {
        for (part, &page) in into.chunks_mut(PAGE).zip(&self.held[tag]) {
            part.copy_from_slice(&self.pool.pages[usize::from(page)].bytes()[..part.len()]);
        }
    }

    /// The first `bytes` of tag `tag`'s data pages, copied once into a new
    /// buffer, which is not filled first.
    fn gathered(&self, tag: usize, bytes: usize) -> Vec<u8> {
        let mut data = Vec::with_capacity(bytes);
        for &page in &self.held[tag] {
            let left = bytes - data.len();
            if left == 0 {
                break;
            }
            let page = self.pool.pages[usize::from(page)].bytes();
            data.extend_from_slice(&page[..left.min(page.len())]);
        }
        data
    }

    /// The pool's pages back from tag `tag`, whose command ended for good.
    pub(super) fn release(&mut self, tag: usize) {
        let mut pages = core::mem::take(&mut self.held[tag]);
        self.pool.give_back(&mut pages);
        self.held[tag] = pages;
    }

    /// Whether command identifier `id` names a command still outstanding.
    pub(super) fn outstanding(&self, id: u16) -> bool {
        self.events.request == Some(id)
            || self.aborts.contains(&id)
            || self
                .tags
                .iter()
                .any(|tag| matches!(tag, Tag::Running { id: running, .. } if *running == id))
            || matches!(
                self.recovery,
                Some(Recovery::Deleting { id: step, .. } | Recovery::Admin { id: step, .. })
                    if step == id
            )
    }

    /// Publish `command` on queue `io`, giving the controller the memory
    /// `data` names, and ring its doorbell; its identifier, one no
    /// outstanding command holds.
    pub(super) fn publish(
        &mut self,
        io: bool,
        command: [u32; 16],
        data: Data,
    ) -> Result<u16, BlockError> {
        self.publish_with(io, command, data, true)
    }

    /// [`Self::publish`], an I/O command's doorbell left owed unless `ring`
    /// ([`Self::ring_submissions`]).
    fn publish_with(
        &mut self,
        io: bool,
        mut command: [u32; 16],
        data: Data,
        ring: bool,
    ) -> Result<u16, BlockError> {
        let stride = self.doorbell_stride;
        loop {
            self.command_id = self.command_id.wrapping_add(1);
            if !self.outstanding(self.command_id) {
                break;
            }
        }
        let id = self.command_id;
        match data {
            Data::None => {}
            Data::Admin => self.admin_data.give()?,
            Data::Tag(tag) => self.give_tag(tag)?,
        }
        let registers = &self.registers;
        let queue = if io { &mut self.io } else { &mut self.admin };
        // SAFETY: the controller only reads submission entries, and the tail
        // entry is none it has yet to fetch: fewer commands are outstanding
        // than the queue holds.
        unsafe { queue.submissions.resume_writes() };
        let base = usize::from(queue.tail) * 64;
        command[0] |= u32::from(id) << 16;
        queue.submissions.write_submission(base, command);
        queue.submissions.give()?;
        queue.tail = queue.next(queue.tail);
        if !io {
            fence(Ordering::SeqCst);
            registers.write32(
                DOORBELLS + 2 * usize::from(queue.id) * stride,
                u32::from(queue.tail),
            )?;
            return Ok(id);
        }
        self.owed = true;
        if ring {
            self.ring_submissions()?;
        }
        Ok(id)
    }

    /// Ring the I/O submission doorbell at the queue's tail if a command
    /// published since it last rang, telling the controller of every one.
    pub(super) fn ring_submissions(&mut self) -> Result<(), BlockError> {
        if !self.owed {
            return Ok(());
        }
        self.owed = false;
        // Every entry written before the tail the controller reads.
        fence(Ordering::SeqCst);
        let doorbell = DOORBELLS + 2 * usize::from(self.io.id) * self.doorbell_stride;
        let tail = u32::from(self.io.tail);
        parts::timed(parts::Part::Doorbell, || {
            self.registers.write32(doorbell, tail)
        })
    }

    /// Ring an owed I/O submission doorbell; a failure fails the commands
    /// whose doorbell it was, and the controller.
    pub(super) fn commit(&mut self) {
        if let Err(error) = self.ring_submissions() {
            self.poison(error);
            self.fail_all();
        }
    }

    /// The completion at queue `io`'s head, once the controller has posted
    /// it: its identifier, admin result (zero for I/O) and status. Its
    /// submission queue identifier and head are not checked, as Linux's
    /// `nvme_handle_cqe` reads neither: controllers report the head when
    /// they make the entry, which need not follow the order they post in.
    pub(super) fn posted(&mut self, io: bool) -> Result<Option<(u16, u32, u32)>, BlockError> {
        let queue = if io { &mut self.io } else { &mut self.admin };
        let entry = usize::from(queue.head) * 16;
        if (queue.completions.coherent(entry + 12)? >> 16 & 1 == 1) != queue.phase {
            return Ok(None);
        }
        // The domain checks the coherent observation; this barrier orders
        // later CQ words after the device's final phase/status publication.
        fence(Ordering::SeqCst);
        // Read/write, flush, discard and write-zeroes completions use only
        // status. Skip the domain-checked DW0 access on the I/O queue.
        let result = if io {
            0
        } else {
            queue.completions.coherent(entry)?
        };
        let tail_word = queue.completions.coherent(entry + 12)?;
        if ((tail_word >> 16 & 1) == 1) != queue.phase {
            return Err(BlockError::Device { status: 0xffff });
        }
        queue.head = queue.next(queue.head);
        if queue.head == 0 {
            queue.phase = !queue.phase;
        }
        Ok(Some((tail_word as u16, result, tail_word >> 17 & 0x7fff)))
    }

    /// Whether a running command's completion is posted at the I/O
    /// queue's head, or one is done and not yet taken: memory alone.
    pub(super) fn posted_completion(&self) -> bool {
        let mut running = false;
        for tag in &self.tags {
            match tag {
                Tag::Done(_) => return true,
                Tag::Running { .. } => running = true,
                _ => {}
            }
        }
        let entry = usize::from(self.io.head) * 16;
        running
            && self
                .io
                .completions
                .coherent(entry + 12)
                .is_ok_and(|word| (word >> 16 & 1 == 1) == self.io.phase)
    }

    /// Ring queue `io`'s completion doorbell at its head.
    pub(super) fn ring_completion(&mut self, io: bool) -> Result<(), BlockError> {
        let stride = self.doorbell_stride;
        let queue = if io { &self.io } else { &self.admin };
        self.registers.write32(
            DOORBELLS + (2 * usize::from(queue.id) + 1) * stride,
            u32::from(queue.head),
        )
    }

    /// Whether a command runs, waits to run again or the controller resets.
    pub(super) fn busy(&self) -> bool {
        self.recovery.is_some()
            || self.tags.iter().any(|tag| {
                matches!(
                    tag,
                    Tag::Running { .. } | Tag::Waiting { .. } | Tag::Delayed { .. }
                )
            })
    }

    /// Wait for every queued command, keeping each result for its tag, and
    /// for a reset to end: a command of a waiting caller's own follows.
    /// Every running command and reset step has a deadline, and retries are
    /// bounded, so this ends.
    pub(super) fn settle(&mut self) -> Result<(), BlockError> {
        loop {
            self.advance();
            if let Some(error) = &self.fault {
                return Err(error.clone());
            }
            if !self.busy() {
                return Ok(());
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// The I/O command for `command` with tag `tag`'s pages, its data
    /// copied in.
    pub(super) fn io_command(
        &mut self,
        tag: usize,
        namespace: u32,
        block: usize,
        command: &Command,
    ) -> Result<[u32; 16], BlockError> {
        let mut words = [0u32; 16];
        words[1] = namespace;
        match command {
            Command::Read { lba, bytes } => {
                let pointers = self.data_pointers(tag, *bytes)?;
                Ok(transfer_command(
                    IO_READ,
                    namespace,
                    *lba,
                    (bytes / block) as u32,
                    pointers,
                    0,
                ))
            }
            Command::Write { lba, data, fua } => {
                for (index, part) in data.chunks(PAGE).enumerate() {
                    self.tag_page(tag, index).bytes_mut()[..part.len()].copy_from_slice(part);
                }
                let pointers = self.data_pointers(tag, data.len())?;
                let access = if *fua { FORCE_UNIT_ACCESS } else { 0 };
                let blocks = (data.len() / block) as u32;
                Ok(transfer_command(
                    IO_WRITE, namespace, *lba, blocks, pointers, access,
                ))
            }
            Command::Flush => {
                words[0] = u32::from(IO_FLUSH);
                Ok(words)
            }
            Command::Discard { extents } => {
                let bounce = self.tag_page(tag, 0);
                // Each range: context attributes, length, starting LBA.
                for (index, extent) in extents.iter().enumerate() {
                    let count = u32::try_from(extent.blocks)
                        .map_err(|_| BlockError::Unsupported("an NVMe range past 2^32 blocks"))?;
                    let range = index * RANGE_BYTES;
                    bounce.write(range, 0u32);
                    bounce.write(range + 4, count);
                    bounce.write(range + 8, extent.lba);
                }
                let address = bounce.address()?;
                words[0] = u32::from(IO_DATASET_MANAGEMENT);
                words[6] = address as u32;
                words[7] = (address >> 32) as u32;
                words[10] = extents.len() as u32 - 1;
                words[11] = ATTRIBUTE_DEALLOCATE;
                Ok(words)
            }
            Command::WriteZeroes { lba, blocks, unmap } => {
                let count = u16::try_from(blocks.wrapping_sub(1))
                    .map_err(|_| BlockError::Unsupported("Write Zeroes past 65536 blocks"))?;
                words[0] = u32::from(IO_WRITE_ZEROES);
                words[10] = *lba as u32;
                words[11] = (*lba >> 32) as u32;
                words[12] = u32::from(count) | if *unmap { DEALLOCATE } else { 0 };
                Ok(words)
            }
        }
    }

    /// Whether [`Self::start`] answers now: a tag and the pages of the
    /// largest command are free and no reset runs, or the controller failed
    /// or shut down, which it refuses at once rather than leaving the
    /// command waiting for room it never gets.
    pub(super) fn room(&self) -> bool {
        self.fault.is_some()
            || self.shut_down
            || (self.recovery.is_none()
                && self.pool.free.len() >= pages_for(self.max_transfer)
                && self.tags.iter().any(|tag| matches!(tag, Tag::Free)))
    }

    /// Run `command` of `namespace` under a free tag; the tag.
    pub(super) fn start(
        &mut self,
        namespace: u32,
        block: usize,
        command: Command,
    ) -> Result<u32, BlockError> {
        self.start_with(namespace, block, command, true)
    }

    /// [`Self::start`], its doorbell left owed unless `ring`.
    pub(super) fn start_with(
        &mut self,
        namespace: u32,
        block: usize,
        command: Command,
        ring: bool,
    ) -> Result<u32, BlockError> {
        self.ready()?;
        let tag = self
            .tags
            .iter()
            .position(|tag| matches!(tag, Tag::Free))
            .filter(|_| self.recovery.is_none())
            .ok_or(BlockError::Protocol("every NVMe tag is in use"))?;
        debug_assert!(self.held[tag].is_empty());
        self.held[tag] = self
            .pool
            .take(command_pages(&command))
            .ok_or(BlockError::Protocol("every NVMe bounce page is in use"))?;
        self.submit_with(tag, namespace, block, command, 0, ring);
        Ok(tag as u32)
    }

    /// Run `command` of `namespace` once every queued command ended, and
    /// wait for it. A fatal controller is reset before it is sent, as
    /// Linux's timeout handling would reset it.
    pub(super) fn run(
        &mut self,
        namespace: u32,
        block: usize,
        command: Command,
        mut destination: Option<&mut [u8]>,
    ) -> Finished {
        self.ready()?;
        self.settle()?;
        match self.registers.read32(STATUS)? {
            REMOVED => return Err(self.poison(removed())),
            status if self.fails(status) => {
                self.reset();
                self.settle()?;
            }
            _ => {}
        }
        let tag = self.start(namespace, block, command)? as usize;
        loop {
            if let Some(result) = self.finish_into(tag, destination.as_deref_mut()) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// Publish `command` under `tag`, which holds no other.
    pub(super) fn submit(
        &mut self,
        tag: usize,
        namespace: u32,
        block: usize,
        command: Command,
        retries: u8,
    ) {
        self.submit_with(tag, namespace, block, command, retries, true);
    }

    /// [`Self::submit`], its doorbell left owed unless `ring`.
    fn submit_with(
        &mut self,
        tag: usize,
        namespace: u32,
        block: usize,
        command: Command,
        retries: u8,
        ring: bool,
    ) {
        // A command whose deadline cannot be kept is not sent; the deadline
        // runs from its doorbell.
        let published = self.deadline(self.command_timeout_us).and_then(|_| {
            let words = self.io_command(tag, namespace, block, &command)?;
            let id = self.publish_with(true, words, Data::Tag(tag), ring)?;
            Ok((id, self.deadline(self.command_timeout_us)?))
        });
        self.tags[tag] = match published {
            Ok((id, deadline)) => Tag::Running {
                id,
                namespace,
                block,
                command,
                deadline,
                retries,
                aborted: false,
            },
            Err(error) => Tag::Done(Err(self.poison(error))),
        };
    }

    /// Collect every completion the controller has posted, run delayed
    /// retries, check the running commands' deadlines and move a reset on;
    /// never waits.
    pub(super) fn advance(&mut self) {
        self.advance_into(None);
    }

    fn advance_into(&mut self, destination: Option<ReadDestination<'_>>) {
        parts::timed(parts::Part::Advance, || self.advance_timed(destination));
    }

    fn advance_timed(&mut self, mut destination: Option<ReadDestination<'_>>) {
        // A command whose doorbell a caller left owed must not wait for its
        // deadline: a look at the queue rings it.
        if self.recovery.is_none() {
            self.commit();
        }
        if self.fault.is_some() {
            self.fail_all();
            return;
        }
        if self.recovery.is_some() {
            if let Err(error) = self.recover() {
                self.poison(error);
                self.fail_all();
            }
            return;
        }
        // Abort and event completions, which the I/O commands do not wait for.
        if (self.events.request.is_some() || !self.aborts.is_empty())
            && let Err(error) = self.admin_completion(None)
        {
            self.poison(error);
            self.fail_all();
            return;
        }
        // A controller posts completions only for running commands.
        let running = self
            .tags
            .iter()
            .any(|tag| matches!(tag, Tag::Running { .. }));
        let mut reaped = false;
        loop {
            if !running {
                break;
            }
            let (head, phase) = (self.io.head, self.io.phase);
            // A valid completion is trusted as Linux's nvme_handle_cqe trusts
            // it, with no register read: the controller's status is read
            // where Linux reads it, at a command's timeout, a reset, an admin
            // command's data, and a completion that names no command.
            match self.posted(true) {
                Ok(None) => break,
                Ok(Some((id, _, status))) => {
                    if !self.complete(id, status, &mut destination)
                        && let Err(error) = self.stray_completion()
                    {
                        // A removed or failed controller's entry stays
                        // unretired.
                        (self.io.head, self.io.phase) = (head, phase);
                        self.poison(error);
                        self.fail_all();
                        return;
                    }
                    reaped = true;
                }
                Err(BlockError::Device { status: 0xffff }) => {
                    let error = self.invalid_completion();
                    self.poison(error);
                    self.fail_all();
                    return;
                }
                Err(error) => {
                    self.poison(error);
                    self.fail_all();
                    return;
                }
            }
        }
        if reaped && let Err(error) = parts::timed(parts::Part::Ring, || self.ring_completion(true))
        {
            self.poison(error);
            self.fail_all();
            return;
        }
        // One clock reading serves every retry delay and command deadline.
        let now = crate::Clock::ticks();
        let mut expired = self.retry_and_find_expired(now);
        while expired != 0 {
            let tag = expired.trailing_zeros() as usize;
            expired &= expired - 1;
            let Tag::Running { id, aborted, .. } = &self.tags[tag] else {
                unreachable!();
            };
            if *aborted {
                self.reset();
                return;
            }
            let id = *id;
            match self.abort(tag, id) {
                Ok(true) => {}
                Ok(false) => {
                    self.reset();
                    return;
                }
                Err(error) => {
                    self.poison(error);
                    self.fail_all();
                    return;
                }
            }
        }
    }

    /// Run all due retries before issuing any Abort. The same counter
    /// snapshot can identify expired running tags during the retry pass.
    fn retry_and_find_expired(&mut self, now: u64) -> u64 {
        const _: () = assert!(DEPTH <= u64::BITS as usize);

        let mut expired = 0;
        for tag in 0..self.tags.len() {
            let deadline = match &self.tags[tag] {
                Tag::Running { deadline, .. } => *deadline,
                Tag::Delayed { until, .. } if until.passed_at(now) => {
                    let Tag::Delayed {
                        namespace,
                        block,
                        command,
                        retries,
                        ..
                    } = core::mem::take(&mut self.tags[tag])
                    else {
                        unreachable!();
                    };
                    self.submit(tag, namespace, block, command, retries);
                    let Tag::Running { deadline, .. } = self.tags[tag] else {
                        continue;
                    };
                    deadline
                }
                _ => continue,
            };
            if deadline.passed_at(now) {
                expired |= 1 << tag;
            }
        }
        expired
    }

    /// Command `id` completed with `status`: its tag takes the result, or,
    /// for an error without Do Not Retry and retries left, runs it again
    /// after the delay its Command Retry Delay selects
    /// (`nvme_decide_disposition`). Whether a running command matched: no
    /// two outstanding commands share an identifier ([`Self::publish`]), so
    /// the identifier names the command, as Linux's request tag does.
    fn complete(
        &mut self,
        id: u16,
        status: u32,
        destination: &mut Option<ReadDestination<'_>>,
    ) -> bool {
        let Some(tag) = self
            .tags
            .iter()
            .position(|tag| matches!(tag, Tag::Running { id: running, .. } if *running == id))
        else {
            return false;
        };
        let Tag::Running {
            namespace,
            block,
            command,
            retries,
            ..
        } = core::mem::take(&mut self.tags[tag])
        else {
            unreachable!();
        };
        // SAFETY: the controller completed this exact command; its memory
        // is the CPU's again.
        let taken = unsafe { self.take_tag(tag) };
        let retry = status != 0 && status & STATUS_DO_NOT_RETRY == 0 && retries < RETRIES;
        // A retry keeps its pages; a page the CPU could not take back stays
        // out of the pool.
        let ended = taken.is_ok() && !retry;
        self.tags[tag] = match (taken, status, command) {
            (Err(error), ..) => Tag::Done(Err(self.poison(error))),
            (Ok(()), _, command) if retry => {
                let delay = match (status & STATUS_RETRY_DELAY) >> 11 {
                    0 => 0,
                    index => u64::from(self.retry_delays[index as usize - 1]) * RETRY_DELAY_UNIT_US,
                };
                match self.deadline(delay) {
                    Ok(until) => Tag::Delayed {
                        namespace,
                        block,
                        command,
                        retries: retries + 1,
                        until,
                    },
                    Err(error) => Tag::Done(Err(error)),
                }
            }
            (Ok(()), 0, Command::Read { bytes, .. }) => {
                if let Some(read) = destination.as_mut().filter(|read| read.tag == tag) {
                    // The pages go back to the pool once this completion is
                    // reaped, even before its result is taken.
                    self.gather(tag, read.bytes);
                    Tag::Done(Ok(None))
                } else {
                    Tag::Done(Ok(Some(self.gathered(tag, bytes))))
                }
            }
            (Ok(()), 0, _) => Tag::Done(Ok(None)),
            (Ok(()), status, _) => Tag::Done(Err(BlockError::Device { status })),
        };
        if ended {
            self.release(tag);
        }
        true
    }

    /// Tag `tag`'s result once its command completed; never waits.
    pub(super) fn finish(&mut self, tag: usize) -> Option<Finished> {
        self.finish_into(tag, None)
    }

    /// Tag `tag`'s result if a completion already reaped gave it; no look at
    /// the queues.
    pub(super) fn finished(&mut self, tag: usize) -> Option<Finished> {
        if !matches!(self.tags.get(tag)?, Tag::Done(_)) {
            return None;
        }
        match core::mem::take(&mut self.tags[tag]) {
            Tag::Done(result) => Some(result),
            _ => unreachable!(),
        }
    }

    pub(super) fn finish_into(
        &mut self,
        tag: usize,
        destination: Option<&mut [u8]>,
    ) -> Option<Finished> {
        // An earlier poll can reap several completions. Only a tag whose
        // result is still pending needs another pass over the queues.
        if !matches!(self.tags.get(tag), Some(Tag::Done(_))) {
            self.advance_into(destination.map(|bytes| ReadDestination { tag, bytes }));
        }
        if !matches!(self.tags.get(tag)?, Tag::Done(_)) {
            return None;
        }
        match core::mem::take(&mut self.tags[tag]) {
            Tag::Done(result) => Some(result),
            _ => unreachable!(),
        }
    }

    /// The PRP entries for `bytes` in tag `tag`'s pages: the first page,
    /// then the second or the PRP list naming every later one (NVMe Base 2.1
    /// §4.3.1), each page whole and aligned.
    pub(super) fn data_pointers(
        &mut self,
        tag: usize,
        bytes: usize,
    ) -> Result<(u64, u64), BlockError> {
        let first = self.tag_page(tag, 0).address()?;
        match bytes.div_ceil(PAGE) {
            0 | 1 => Ok((first, 0)),
            2 => Ok((first, self.tag_page(tag, 1).address()?)),
            pages => {
                for index in 1..pages {
                    let address = self.tag_page(tag, index).address()?;
                    self.tag_page(tag, pages).write(8 * (index - 1), address);
                }
                Ok((first, self.tag_page(tag, pages).address()?))
            }
        }
    }

    pub(super) fn deadline(&self, us: u64) -> Result<crate::Deadline, BlockError> {
        self.clock
            .try_deadline(us)
            .map_err(|_| BlockError::Unsupported("NVMe requires a representable deadline"))
    }

    /// Linux `nvme_timeout` for an I/O command's first deadline: abort it
    /// and give it another, unless the controller is fatally failed, which
    /// is reset at once (`false`).
    pub(super) fn abort(&mut self, tag: usize, id: u16) -> Result<bool, BlockError> {
        match self.registers.read32(STATUS)? {
            REMOVED => return Err(removed()),
            status if status & FATAL != 0 => return Ok(false),
            _ => {}
        }
        let mut command = [0u32; 16];
        command[0] = u32::from(ADMIN_ABORT);
        command[10] = u32::from(id) << 16 | u32::from(self.io.id);
        let abort = self.publish(false, command, Data::None)?;
        self.aborts.push(abort);
        let renewed = self.deadline(self.command_timeout_us)?;
        if let Tag::Running {
            deadline, aborted, ..
        } = &mut self.tags[tag]
        {
            (*deadline, *aborted) = (renewed, true);
        }
        Ok(true)
    }

    /// End every command with the controller's fault.
    pub(super) fn fail_all(&mut self) {
        let error = self.fault.clone().unwrap_or(BlockError::Timeout);
        for tag in &mut self.tags {
            if matches!(
                tag,
                Tag::Running { .. } | Tag::Waiting { .. } | Tag::Delayed { .. }
            ) {
                *tag = Tag::Done(Err(error.clone()));
            }
        }
        self.recovery = None;
    }
}
