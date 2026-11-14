//! virtio-blk block devices with one polled split virtqueue, as QEMU's
//! `virtio-blk-pci` presents them.
use crate::{
    Clock, Deadline, Handoff,
    block::{
        BlockDevice, BlockError, Command, DiscardLimits, Extent, Features, Finished, Identity,
        Rotation, Transport, WriteZeroes, ascii,
    },
    bus::{
        dma::{DmaAllocator, DmaBuffer},
        mmio::Mmio,
        pci::Function,
        virtio::{AVAIL, DESCRIPTOR_NEXT, DESCRIPTOR_WRITE, Modern, TIMEOUT_US, USED, VENDOR},
    },
};
use alloc::{format, vec::Vec};
use core::sync::atomic::{Ordering, fence};

const MODERN_BLOCK: u16 = 0x1042;
const TRANSITIONAL_BLOCK: u16 = 0x1001;

const FEATURE_READ_ONLY: u64 = 1 << 5;
const FEATURE_BLOCK_SIZE: u64 = 1 << 6;
const FEATURE_FLUSH: u64 = 1 << 9;
const FEATURE_DISCARD: u64 = 1 << 13;
const FEATURE_WRITE_ZEROES: u64 = 1 << 14;

/// Configuration space: capacity in sectors, and the logical block size.
const CONFIG_CAPACITY: usize = 0x00;
const CONFIG_BLOCK_SIZE: usize = 0x14;
/// The most sectors one discard, and one Write Zeroes, may take, and
/// whether Write Zeroes may deallocate (virtio 1.2 §5.2.4).
const CONFIG_MAX_DISCARD: usize = 0x24;
const CONFIG_MAX_WRITE_ZEROES: usize = 0x30;
const CONFIG_WRITE_ZEROES_UNMAP: usize = 0x38;

/// `virtio_blk_req`: type, reserved, sector, then the device's status byte
/// after the data (virtio 1.2 §5.2.6).
const REQUEST_HEADER: u32 = 16;
const REQUEST_TYPE: usize = 0;
const REQUEST_RESERVED: usize = 4;
const REQUEST_SECTOR: usize = 8;
const REQUEST_STATUS: usize = 16;
/// A status byte the device has not written yet.
const STATUS_PENDING: u8 = 0xff;
/// Linux `VIRTIO_BLK_ID_BYTES`.
const ID_BYTES: usize = 20;

const REQUEST_IN: u32 = 0;
const REQUEST_OUT: u32 = 1;
const REQUEST_FLUSH: u32 = 4;
const REQUEST_GET_ID: u32 = 8;
const REQUEST_DISCARD: u32 = 11;
const REQUEST_WRITE_ZEROES: u32 = 13;
/// `virtio_blk_discard_write_zeroes`: sector, sector count, then flags
/// whose bit 0 lets Write Zeroes deallocate.
const SEGMENT_BYTES: usize = 16;
const SEGMENT_UNMAP: u32 = 1;

const QUEUE_ENTRIES: u16 = 8;
const BOUNCE_BYTES: usize = 64 * 1024;
const SECTOR: u64 = 512;

pub fn matches(function: &Function) -> bool {
    matches_identity(&function.identity())
}

/// Candidate selection from copied identity DATA; owned attachment still
/// revalidates configuration and platform requirements.
pub fn matches_identity(function: &crate::bus::pci::Identity) -> bool {
    function.vendor == VENDOR
        && (function.device == MODERN_BLOCK || function.device == TRANSITIONAL_BLOCK)
}

pub struct VirtioBlk {
    device: Mmio,
    notify: Mmio,
    queue: DmaBuffer,
    request: DmaBuffer,
    bounce: DmaBuffer,
    entries: u16,
    next: u16,
    sectors_per_block: u64,
    identity: Identity,
    clock: Clock,
    /// A request timed out and the device may still own its buffers; none
    /// is reused until a reset this driver does not perform.
    fault: Option<BlockError>,
    /// The block command started on the queue, and the next one's tag.
    running: Option<Running>,
    next_tag: u32,
}

/// A request offered to the device, until the used index reaches
/// `expected`.
#[derive(Clone, Copy)]
struct Posted {
    expected: u16,
    deadline: Deadline,
}

/// A started block command, run a bounce buffer's worth at a time.
struct Running {
    tag: u32,
    command: Command,
    /// Bytes done; a flush counts one once complete.
    done: usize,
    data: Vec<u8>,
    /// The posted request, and its data bytes.
    posted: Option<(Posted, usize)>,
    result: Option<Finished>,
}

impl VirtioBlk {
    pub fn new(
        function: &mut Function,
        handoff: &Handoff,
        dma: &mut dyn DmaAllocator,
        minimum: &(impl crate::bus::pci::power::Timer + ?Sized),
    ) -> Result<Self, BlockError> {
        minimum.check().map_err(BlockError::PciPower)?;
        let transport = Modern::new(function, handoff, minimum)?;
        let accepted = transport.negotiate(
            FEATURE_READ_ONLY
                | FEATURE_BLOCK_SIZE
                | FEATURE_FLUSH
                | FEATURE_DISCARD
                | FEATURE_WRITE_ZEROES,
        )?;
        let queue = dma.allocate(4096, 4096)?;
        let (entries, notify) = transport.queue(handoff, 0, QUEUE_ENTRIES, &queue)?;
        transport.ready();
        let device = transport.device;

        let block_size = match accepted & FEATURE_BLOCK_SIZE {
            0 => SECTOR as u32,
            _ => device.read32(CONFIG_BLOCK_SIZE),
        };
        if block_size < SECTOR as u32
            || !block_size.is_power_of_two()
            || block_size as usize > BOUNCE_BYTES
        {
            return Err(BlockError::Unsupported("virtio block size"));
        }
        let sectors_per_block = u64::from(block_size) / SECTOR;
        // Linux `virtblk_probe`: a limit of zero is none, and one command
        // counts at most a 32-bit number of sectors.
        let blocks = |offset| {
            let sectors = match device.read32(offset) {
                0 => u32::MAX,
                sectors => sectors,
            };
            (u64::from(sectors) / sectors_per_block).max(1)
        };
        let features = Features {
            write_cache: accepted & FEATURE_FLUSH != 0,
            fua: false,
            max_transfer: BOUNCE_BYTES,
            max_read_transfer: None,
            discard: (accepted & FEATURE_DISCARD != 0)
                .then(|| DiscardLimits::single(blocks(CONFIG_MAX_DISCARD))),
            write_zeroes: (accepted & FEATURE_WRITE_ZEROES != 0).then(|| WriteZeroes {
                blocks: blocks(CONFIG_MAX_WRITE_ZEROES),
                unmap: device.read8(CONFIG_WRITE_ZEROES_UNMAP) != 0,
            }),
            // virtio-blk has no rotation field.
            rotation: Rotation::Unreported,
        };
        let mut driver = Self {
            device,
            notify,
            queue,
            request: dma.allocate(4096, 4096)?,
            bounce: dma.allocate(BOUNCE_BYTES, 4096)?,
            entries,
            next: 0,
            sectors_per_block,
            identity: Identity {
                transport: Transport::VirtioBlk,
                location: format!("{}", function.address),
                model: format!("virtio-blk {:04x}:{:04x}", function.vendor, function.device),
                serial: alloc::string::String::new(),
                blocks: 0,
                block_size,
                read_only: accepted & FEATURE_READ_ONLY != 0,
                features,
            },
            clock: handoff.clock,
            fault: None,
            running: None,
            next_tag: 0,
        };
        driver.identity.blocks = driver.device.read64(CONFIG_CAPACITY) / sectors_per_block;
        driver.identity.serial = match driver.submit(REQUEST_GET_ID, 0, ID_BYTES, true) {
            Ok(()) => ascii(&driver.bounce.bytes()[..ID_BYTES]),
            Err(BlockError::Device { .. }) => alloc::string::String::new(),
            Err(error) => return Err(error),
        };
        Ok(driver)
    }

    /// Run one request of `bytes` data bytes through the bounce buffer and
    /// wait for the device to return it, after the started block command.
    fn submit(
        &mut self,
        kind: u32,
        sector: u64,
        bytes: usize,
        device_writes: bool,
    ) -> Result<(), BlockError> {
        while self
            .running
            .as_ref()
            .is_some_and(|running| running.result.is_none())
        {
            self.run();
            crate::waiting();
            core::hint::spin_loop();
        }
        let posted = self.post(kind, sector, bytes, device_writes)?;
        loop {
            if let Some(result) = self.poll(posted) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// Offer one request of `bytes` data bytes through the bounce buffer.
    fn post(
        &mut self,
        kind: u32,
        sector: u64,
        bytes: usize,
        device_writes: bool,
    ) -> Result<Posted, BlockError> {
        if let Some(fault) = &self.fault {
            return Err(fault.clone());
        }
        let deadline = self
            .clock
            .try_deadline(TIMEOUT_US)
            .map_err(|_| BlockError::Timeout)?;
        let request = self.request.bus_address();
        self.request.write(REQUEST_TYPE, kind);
        self.request.write(REQUEST_RESERVED, 0u32);
        self.request.write(REQUEST_SECTOR, sector);
        self.request.write(REQUEST_STATUS, STATUS_PENDING);
        let mut chain = [(request, REQUEST_HEADER, 0u16); 3];
        let mut length = 1;
        if bytes > 0 {
            let flags = if device_writes { DESCRIPTOR_WRITE } else { 0 };
            chain[1] = (self.bounce.bus_address(), bytes as u32, flags);
            length = 2;
        }
        chain[length] = (request + u64::from(REQUEST_HEADER), 1, DESCRIPTOR_WRITE);
        for (index, (address, len, flags)) in chain[..=length].iter().enumerate() {
            let next = if index < length { DESCRIPTOR_NEXT } else { 0 };
            let base = 16 * index;
            self.queue.write(base, *address);
            self.queue.write(base + 8, *len);
            self.queue.write(base + 12, flags | next);
            self.queue.write(base + 14, index as u16 + 1);
        }
        let slot = AVAIL + 4 + 2 * usize::from(self.next % self.entries);
        self.queue.write(slot, 0u16);
        fence(Ordering::SeqCst);
        self.next = self.next.wrapping_add(1);
        self.queue.write(AVAIL + 2, self.next);
        fence(Ordering::SeqCst);
        self.notify.write16(0, 0);
        Ok(Posted {
            expected: self.next,
            deadline,
        })
    }

    /// Whether the device has returned the posted request: memory alone.
    fn returned(&self, posted: Posted) -> bool {
        self.queue.read::<u16>(USED + 2) == posted.expected
    }

    /// The posted request's result once the device returned it or its time
    /// ran out; never waits.
    fn poll(&mut self, posted: Posted) -> Option<Result<(), BlockError>> {
        if !self.returned(posted) {
            if !posted.deadline.passed() {
                return None;
            }
            if !self.returned(posted) {
                self.fault = Some(BlockError::Timeout);
                return Some(Err(BlockError::Timeout));
            }
        }
        fence(Ordering::SeqCst);
        // The chain always starts at descriptor 0; any other head was never
        // offered (Linux `virtqueue_get_buf` marks the queue broken).
        let used = USED + 4 + 8 * usize::from(posted.expected.wrapping_sub(1) % self.entries);
        if self.queue.read::<u32>(used) != 0 {
            let error = BlockError::Device { status: u32::MAX };
            self.fault = Some(error.clone());
            return Some(Err(error));
        }
        Some(match self.request.read::<u8>(REQUEST_STATUS) {
            0 => Ok(()),
            status => Err(BlockError::Device {
                status: u32::from(status),
            }),
        })
    }

    /// Move the started block command on without waiting, by one device
    /// round trip at most: take the posted request's result if the device
    /// returned it, and post the next. A call never chains requests, even
    /// when the device returns each at its notify (an emulated one that
    /// runs the request in the notifying CPU's exit): the caller holds the
    /// host drivers' lock, which other disks' commands wait for.
    fn run(&mut self) {
        let Some(mut running) = self.running.take() else {
            return;
        };
        let mut offered = false;
        while running.result.is_none() {
            let Some((posted, bytes)) = running.posted else {
                if offered && Self::posts_more(&running) {
                    break;
                }
                match self.next_request(&running) {
                    Ok(Some(posted)) => {
                        offered = true;
                        running.posted = Some(posted);
                    }
                    Ok(None) => {
                        let data = core::mem::take(&mut running.data);
                        let read = matches!(running.command, Command::Read { .. });
                        running.result = Some(Ok(read.then_some(data)));
                    }
                    Err(error) => running.result = Some(Err(error)),
                }
                continue;
            };
            match self.poll(posted) {
                None => break,
                Some(Err(error)) => running.result = Some(Err(error)),
                Some(Ok(())) => {
                    if let Command::Read { .. } = running.command {
                        running
                            .data
                            .extend_from_slice(&self.bounce.bytes()[..bytes]);
                    }
                    running.done += bytes.max(1);
                    running.posted = None;
                }
            }
        }
        self.running = Some(running);
    }

    /// Whether `running` has a part left to post ([`Self::next_request`]).
    fn posts_more(running: &Running) -> bool {
        let done = running.done;
        match &running.command {
            Command::Read { bytes, .. } => done < *bytes,
            Command::Write {
                data, fua: false, ..
            } => done < data.len(),
            Command::Flush | Command::Discard { .. } | Command::WriteZeroes { .. } => done == 0,
            Command::Write { fua: true, .. } => false,
        }
    }

    /// Post the next part of `running`'s command, or `None` once all of it
    /// is done.
    fn next_request(&mut self, running: &Running) -> Result<Option<(Posted, usize)>, BlockError> {
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_blocks() as usize * block;
        let (done, sectors_per_block) = (running.done, self.sectors_per_block);
        let first = |lba: u64| (lba + (done / block) as u64) * sectors_per_block;
        let (kind, sector, bytes, device_writes) = match &running.command {
            Command::Read { lba, bytes } if done < *bytes => {
                (REQUEST_IN, first(*lba), (bytes - done).min(chunk), true)
            }
            Command::Write {
                lba,
                data,
                fua: false,
            } if done < data.len() => {
                let part = (data.len() - done).min(chunk);
                let sector = first(*lba);
                self.bounce.bytes_mut()[..part].copy_from_slice(&data[done..done + part]);
                (REQUEST_OUT, sector, part, false)
            }
            Command::Flush if done == 0 => (REQUEST_FLUSH, 0, 0, false),
            Command::Discard { extents } if done == 0 => {
                let &[Extent { lba, blocks }] = extents.as_slice() else {
                    return Err(BlockError::Unsupported("one discard segment a command"));
                };
                self.stage(lba, blocks, 0)?;
                (REQUEST_DISCARD, 0, SEGMENT_BYTES, false)
            }
            Command::WriteZeroes { lba, blocks, unmap } if done == 0 => {
                let flags = if *unmap { SEGMENT_UNMAP } else { 0 };
                self.stage(*lba, *blocks, flags)?;
                (REQUEST_WRITE_ZEROES, 0, SEGMENT_BYTES, false)
            }
            Command::Write { fua: true, .. } => {
                return Err(BlockError::Unsupported(
                    "the device takes no Force Unit Access",
                ));
            }
            _ => return Ok(None),
        };
        let posted = self.post(kind, sector, bytes, device_writes)?;
        Ok(Some((posted, bytes)))
    }

    /// Discard or Write Zeroes `blocks` blocks from `lba`: one segment.
    fn segment(&mut self, kind: u32, lba: u64, blocks: u64, flags: u32) -> Result<(), BlockError> {
        self.stage(lba, blocks, flags)?;
        self.submit(kind, 0, SEGMENT_BYTES, false)
    }

    /// Stage the segment for `blocks` blocks from `lba` in the bounce buffer.
    fn stage(&mut self, lba: u64, blocks: u64, flags: u32) -> Result<(), BlockError> {
        let sectors = u32::try_from(blocks * self.sectors_per_block)
            .map_err(|_| BlockError::Unsupported("a virtio segment past 2^32 sectors"))?;
        self.bounce.write(0, lba * self.sectors_per_block);
        self.bounce.write(8, sectors);
        self.bounce.write(12, flags);
        Ok(())
    }

    fn chunk_blocks(&self) -> u64 {
        (BOUNCE_BYTES / self.identity.block_size as usize) as u64
    }
}

impl BlockDevice for VirtioBlk {
    fn identity(&self) -> &Identity {
        &self.identity
    }

    fn read(&mut self, lba: u64, buffer: &mut [u8]) -> Result<(), BlockError> {
        self.identity.check(lba, buffer.len())?;
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_blocks() as usize * block;
        for (index, part) in buffer.chunks_mut(chunk).enumerate() {
            let first = lba + (index * chunk / block) as u64;
            self.submit(REQUEST_IN, first * self.sectors_per_block, part.len(), true)?;
            part.copy_from_slice(&self.bounce.bytes()[..part.len()]);
        }
        Ok(())
    }

    fn write(&mut self, lba: u64, data: &[u8]) -> Result<(), BlockError> {
        self.identity.check_write(lba, data.len())?;
        let block = self.identity.block_size as usize;
        let chunk = self.chunk_blocks() as usize * block;
        for (index, part) in data.chunks(chunk).enumerate() {
            let first = lba + (index * chunk / block) as u64;
            self.bounce.bytes_mut()[..part.len()].copy_from_slice(part);
            self.submit(
                REQUEST_OUT,
                first * self.sectors_per_block,
                part.len(),
                false,
            )?;
        }
        Ok(())
    }

    fn flush_cache(&mut self) -> Result<(), BlockError> {
        self.submit(REQUEST_FLUSH, 0, 0, false)
    }

    fn discard(&mut self, lba: u64, blocks: u64) -> Result<(), BlockError> {
        self.segment(REQUEST_DISCARD, lba, blocks, 0)
    }

    fn write_zeroes(&mut self, lba: u64, blocks: u64, unmap: bool) -> Result<(), BlockError> {
        let flags = if unmap { SEGMENT_UNMAP } else { 0 };
        self.segment(REQUEST_WRITE_ZEROES, lba, blocks, flags)
    }

    /// One request at a time, started under the caller's lock and returned
    /// at a later look ([`Self::finish`]).
    fn depth(&self) -> usize {
        1
    }

    fn room(&self) -> bool {
        self.running.is_none()
    }

    fn start(&mut self, command: Command) -> Result<u32, BlockError> {
        if let Some(fault) = &self.fault {
            return Err(fault.clone());
        }
        let command = command.within(&self.identity, 0)?;
        if self.running.is_some() {
            return Err(BlockError::Protocol(
                "a virtio-blk queue runs one command at a time",
            ));
        }
        let tag = self.next_tag;
        self.next_tag = tag.wrapping_add(1);
        self.running = Some(Running {
            tag,
            command,
            done: 0,
            data: Vec::new(),
            posted: None,
            result: None,
        });
        self.run();
        Ok(tag)
    }

    fn finish(&mut self, tag: u32) -> Option<Finished> {
        if self.running.as_ref()?.tag != tag {
            return None;
        }
        self.run();
        let result = self.running.as_mut()?.result.take()?;
        self.running = None;
        Some(result)
    }

    /// A command with a part left to post, which a look posts, counts as
    /// posted too.
    fn posted(&self) -> bool {
        self.running
            .as_ref()
            .is_some_and(|running| match running.posted {
                _ if running.result.is_some() => true,
                Some((posted, _)) => self.returned(posted),
                None => true,
            })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::TestRegion;
    use alloc::{string::String, vec};

    /// A queue whose device returns each request at its notify, as an
    /// emulated one does in the notifying CPU's exit: the notify register
    /// lies on the request's status byte, so notifying writes the status
    /// "done", and the used index starts where the first request's ends.
    fn returning_at_notify(region: &mut TestRegion, registers: &mut [u64]) -> VirtioBlk {
        let request = region.allocate(4096, 4096).unwrap();
        // SAFETY: the fixture owns both allocations past the driver's use;
        // no device reaches them.
        let (device, notify) = unsafe {
            (
                Mmio::new(registers.as_mut_ptr() as u64, size_of_val(registers)),
                Mmio::new(request.bus_address() + REQUEST_STATUS as u64, 2),
            )
        };
        VirtioBlk {
            device,
            notify,
            queue: region.allocate(4096, 4096).unwrap(),
            request,
            bounce: region.allocate(BOUNCE_BYTES, 4096).unwrap(),
            entries: QUEUE_ENTRIES,
            // The first request's used index is then 0, as the zeroed queue
            // already says.
            next: u16::MAX,
            sectors_per_block: 1,
            identity: Identity {
                transport: Transport::VirtioBlk,
                location: String::from("fixture"),
                model: String::from("virtio-blk"),
                serial: String::new(),
                blocks: 1 << 20,
                block_size: SECTOR as u32,
                read_only: false,
                features: Features {
                    max_transfer: BOUNCE_BYTES,
                    ..Features::default()
                },
            },
            clock: Clock {
                ticks_per_microsecond: 1000,
            },
            fault: None,
            running: None,
            next_tag: 0,
        }
    }

    /// The device has returned `count` requests in all.
    fn returned(disk: &mut VirtioBlk, count: u16) {
        disk.queue.write(USED + 2, count.wrapping_sub(1));
    }

    /// A look moves a command by one device round trip at most, even when
    /// the device returns each request at once: it takes the returned
    /// result and posts one request, never a chain of them, and says the
    /// rest is ready for the next look.
    #[test]
    fn a_look_posts_one_request_however_soon_the_device_returns_it() {
        let mut region = TestRegion::new(256 * 1024);
        let mut registers = vec![0u64; 16].into_boxed_slice();
        let mut disk = returning_at_notify(&mut region, &mut registers);
        let tag = disk
            .start(Command::Write {
                lba: 0,
                data: vec![0x5a; 3 * BOUNCE_BYTES],
                fua: false,
            })
            .unwrap();
        assert_eq!(disk.next, 0, "the start posted one request");
        assert_eq!(disk.running.as_ref().unwrap().done, BOUNCE_BYTES);
        assert!(disk.posted(), "the next part waits for a look");

        assert_eq!(disk.finish(tag), None);
        assert_eq!(disk.next, 1, "the look posted one request");
        assert!(!disk.posted(), "the device has not returned it");

        returned(&mut disk, 2);
        assert!(disk.posted());
        assert_eq!(disk.finish(tag), None);
        assert_eq!(disk.next, 2, "the look took the result and posted one");

        returned(&mut disk, 3);
        assert_eq!(disk.finish(tag), Some(Ok(None)));
        assert_eq!(disk.next, 2, "nothing was left to post");
        assert!(!disk.posted());
    }
}
