//! virtio-net PCI devices (virtio 1.2 section 5.1), as QEMU's
//! `virtio-net-pci` presents them, as a raw Ethernet [`Link`] that sends
//! the handoff's completion signal for received frames and link changes
//! when the function has MSI-X.
//!
//! The driver negotiates only VERSION_1, MAC and STATUS: no checksum or
//! segmentation offload, no mergeable receive buffers and no control
//! queue, so every frame is one buffer behind the 12-byte
//! `virtio_net_hdr` (section 5.1.6, `num_buffers` included under
//! VERSION_1) with all-zero offload fields. A device without a MAC
//! address refuses: the stack needs a stable station address.
use super::{HEADER as ETHERNET_HEADER, Link, LinkError, MAX_FRAME, Mac, check_frame};
use crate::{
    Clock, Handoff,
    block::BlockError,
    bus::{
        dma::{DmaAllocator, DmaBuffer},
        mmio::Mmio,
        pci::Function,
        virtio::{AVAIL, DESCRIPTOR_WRITE, Modern, USED, VENDOR},
    },
};
use alloc::{format, vec::Vec};
use core::sync::atomic::{Ordering, fence};

const MODERN_NET: u16 = 0x1041;
const TRANSITIONAL_NET: u16 = 0x1000;
const FEATURE_MAC: u64 = 1 << 5;
const FEATURE_STATUS: u64 = 1 << 16;
const CONFIG_MAC: usize = 0;
const CONFIG_STATUS: usize = 6;
const LINK_UP: u16 = 1;
const RECEIVE: u16 = 0;
const TRANSMIT: u16 = 1;
/// Receive buffers the device may fill between two reads, at most: the
/// guest network reads the adapter once per host service, about every
/// 4 ms, and a device out of buffers holds or drops what arrives (QEMU's
/// tap queue), so 16 capped a TCP stream into the guest near 6 MB/s. 256
/// is QEMU's default `rx_queue_size`; a smaller device gets its own size.
const RECEIVE_ENTRIES: u16 = 256;
/// Frames in flight to the device at most before a send waits.
const TRANSMIT_ENTRIES: u16 = 64;
/// Split used-ring flag, since the driver does not negotiate EVENT_IDX.
const USED_NO_NOTIFY: u16 = 1;
/// `virtio_net_hdr` under VERSION_1.
const HEADER: usize = 12;
const BUFFER: usize = 2048;
/// The shortest and longest receive lengths that carry one whole frame
/// (Linux `receive_small` drops others as length errors).
const SHORTEST_RECEIVE: usize = HEADER + ETHERNET_HEADER;
const LONGEST_RECEIVE: usize = HEADER + MAX_FRAME;
const TRANSMIT_WAIT_US: u64 = 100_000;

/// Where a split queue of `entries` keeps its driver and device rings, and
/// the bytes it spans: the descriptors at 0, the driver ring after them
/// (2-byte aligned) and the device ring after that (4-byte aligned, virtio
/// 1.2 section 2.7). Up to 16 entries this is [`Modern::queue`]'s layout.
const fn layout(entries: u16) -> (usize, usize, usize) {
    let entries = entries as usize;
    let driver = if 16 * entries > AVAIL {
        16 * entries
    } else {
        AVAIL
    };
    let device = (driver + 6 + 2 * entries + 3) & !3;
    let device = if device > USED { device } else { USED };
    let bytes = (device + 6 + 8 * entries + 4095) & !4095;
    (driver, device, bytes)
}

pub fn matches(function: &Function) -> bool {
    matches_identity(&function.identity())
}

/// Candidate selection from copied identity DATA; owned attachment still
/// revalidates configuration and platform requirements.
pub fn matches_identity(function: &crate::bus::pci::Identity) -> bool {
    function.vendor == VENDOR
        && (function.device == MODERN_NET || function.device == TRANSITIONAL_NET)
}

/// One split virtqueue and its buffers, one per descriptor.
struct Queue {
    ring: DmaBuffer,
    buffers: DmaBuffer,
    notify: Mmio,
    entries: u16,
    /// The driver and device rings' offsets in `ring` ([`layout`]).
    driver: usize,
    device: usize,
    /// The used index consumed so far, and the driver ring's index.
    used: u16,
    available: u16,
}

impl Queue {
    fn new(
        transport: &Modern,
        handoff: &Handoff,
        index: u16,
        entries: u16,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        // A device that takes fewer entries leaves the rings room to spare.
        let (driver, device, bytes) = layout(entries);
        let ring = dma.allocate(bytes, 4096)?;
        let (entries, notify) =
            transport.queue_at(handoff, index, entries, &ring, driver, device)?;
        let buffers = dma.allocate(usize::from(entries) * BUFFER, 4096)?;
        Ok(Self {
            ring,
            buffers,
            notify,
            entries,
            driver,
            device,
            used: 0,
            available: 0,
        })
    }

    fn describe(&mut self, id: u16, length: usize, flags: u16) {
        let base = 16 * usize::from(id);
        let address = self.buffers.bus_address() + (usize::from(id) * BUFFER) as u64;
        self.ring.write(base, address);
        self.ring.write(base + 8, length as u32);
        self.ring.write(base + 12, flags);
        self.ring.write(base + 14, 0u16);
    }

    /// Add one descriptor to the driver ring without publishing its index.
    fn stage(&mut self, id: u16) {
        let slot = self.driver + 4 + 2 * usize::from(self.available % self.entries);
        self.ring.write(slot, id);
        self.available = self.available.wrapping_add(1);
    }

    fn publish(&mut self) {
        fence(Ordering::SeqCst);
        self.ring.write(self.driver + 2, self.available);
        // Expose the new index before checking whether the device needs a
        // kick. A device rearming notifications can then see the new work.
        fence(Ordering::SeqCst);
        if self.ring.read::<u16>(self.device) & USED_NO_NOTIFY == 0 {
            self.notify.write16(0, 0);
        }
    }

    fn offer(&mut self, id: u16) {
        self.stage(id);
        self.publish();
    }

    fn prime_receive(&mut self) {
        for id in 0..self.entries {
            // These descriptors always name the same writable buffers.
            self.describe(id, BUFFER, DESCRIPTOR_WRITE);
            self.stage(id);
        }
        self.publish();
    }

    /// The next buffer the device returned: its descriptor and length. A
    /// descriptor the driver never offered, or a length past its buffer,
    /// breaks the queue, as Linux `virtqueue_get_buf` does.
    fn take(&mut self) -> Result<Option<(u16, usize)>, Broken> {
        let produced: u16 = self.ring.read(self.device + 2);
        if self.used == produced {
            return Ok(None);
        }
        // The device publishes the index after writing the used entry and
        // buffer. Order those reads after observing a completed descriptor.
        fence(Ordering::SeqCst);
        let slot = self.device + 4 + 8 * usize::from(self.used % self.entries);
        let id = self.ring.read::<u32>(slot);
        let length = self.ring.read::<u32>(slot + 4) as usize;
        self.used = self.used.wrapping_add(1);
        match u16::try_from(id) {
            Ok(id) if id < self.entries && length <= BUFFER => Ok(Some((id, length))),
            _ => Err(Broken),
        }
    }

    fn buffer(&mut self, id: u16) -> &mut [u8] {
        let start = usize::from(id) * BUFFER;
        &mut self.buffers.bytes_mut()[start..start + BUFFER]
    }
}

/// The device returned a descriptor or length it was never given.
struct Broken;

pub struct VirtioNet {
    device: Mmio,
    receive: Queue,
    transmit: Queue,
    /// Transmit descriptors the device has handed back.
    free: Vec<u16>,
    /// The device broke a queue; it carries no more frames.
    broken: bool,
    mac: Mac,
    status: bool,
    clock: Clock,
    pub location: alloc::string::String,
    /// Whether the device sends the handoff's completion signal.
    signalled: bool,
}

impl VirtioNet {
    pub fn new(
        function: &mut Function,
        handoff: &Handoff,
        dma: &mut dyn DmaAllocator,
        minimum: &dyn crate::bus::pci::power::Timer,
    ) -> Result<Self, BlockError> {
        let mut transport = Modern::new(function, handoff, minimum)?;
        if let Some(message) = handoff.completion {
            transport.signal(function, handoff, message, &[RECEIVE])?;
        }
        let accepted = transport.negotiate(FEATURE_MAC | FEATURE_STATUS)?;
        if accepted & FEATURE_MAC == 0 {
            return Err(BlockError::Unsupported(
                "virtio-net device without a MAC address",
            ));
        }
        let receive = Queue::new(&transport, handoff, RECEIVE, RECEIVE_ENTRIES, dma)?;
        let transmit = Queue::new(&transport, handoff, TRANSMIT, TRANSMIT_ENTRIES, dma)?;
        let signalled = transport.signals();
        let device = transport.device;
        let mac = Mac(core::array::from_fn(|n| device.read8(CONFIG_MAC + n)));
        let mut net = Self {
            device,
            free: (0..transmit.entries).collect(),
            broken: false,
            receive,
            transmit,
            mac,
            status: accepted & FEATURE_STATUS != 0,
            clock: handoff.clock,
            location: format!("{}", function.address),
            signalled,
        };
        transport.ready();
        net.receive.prime_receive();
        Ok(net)
    }

    /// Take back every transmit descriptor the device returned; one it
    /// returns twice breaks the queue.
    fn reclaim(&mut self) -> Result<(), Broken> {
        while let Some((id, _)) = self.transmit.take()? {
            if self.free.contains(&id) {
                return Err(Broken);
            }
            self.free.push(id);
        }
        Ok(())
    }

    fn usable(&self) -> Result<(), LinkError> {
        match self.broken {
            true => Err(LinkError::Adapter(format!(
                "virtio-net {} returned a buffer it was never given",
                self.location
            ))),
            false => Ok(()),
        }
    }

    fn break_on<T>(&mut self, result: Result<T, Broken>) -> Result<T, LinkError> {
        result.map_err(|Broken| {
            self.broken = true;
            self.usable().unwrap_err()
        })
    }

    /// Consume a valid DMA buffer before reposting its descriptor.
    fn receive_frame<T>(&mut self, copy: impl FnOnce(&[u8]) -> T) -> Result<Option<T>, LinkError> {
        self.usable()?;
        loop {
            let taken = self.receive.take();
            let Some((id, length)) = self.break_on(taken)? else {
                return Ok(None);
            };
            if (SHORTEST_RECEIVE..=LONGEST_RECEIVE).contains(&length) {
                let frame = copy(&self.receive.buffer(id)[HEADER..length]);
                self.receive.offer(id);
                return Ok(Some(frame));
            }
            // A runt or oversized buffer is dropped and the next one read.
            self.receive.offer(id);
        }
    }
}

impl Link for VirtioNet {
    fn mac(&self) -> Mac {
        self.mac
    }

    fn carrier(&mut self) -> bool {
        !self.status || self.device.read16(CONFIG_STATUS) & LINK_UP != 0
    }

    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        check_frame(frame)?;
        self.usable()?;
        let reclaimed = self.reclaim();
        self.break_on(reclaimed)?;
        if self.free.is_empty() {
            let deadline = self.clock.deadline(TRANSMIT_WAIT_US);
            while self.free.is_empty() && !deadline.passed() {
                let reclaimed = self.reclaim();
                self.break_on(reclaimed)?;
                crate::waiting();
            }
        }
        let id = self.free.pop().ok_or_else(|| {
            LinkError::Adapter(format!(
                "virtio-net {} transmit queue stalled",
                self.location
            ))
        })?;
        let buffer = self.transmit.buffer(id);
        buffer[..HEADER].fill(0);
        buffer[HEADER..HEADER + frame.len()].copy_from_slice(frame);
        self.transmit.describe(id, HEADER + frame.len(), 0);
        self.transmit.offer(id);
        Ok(())
    }

    fn signals(&self) -> bool {
        self.signalled
    }

    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.receive_frame(<[u8]>::to_vec)
    }

    fn receive_reusing(&mut self, frame: &mut Vec<u8>) -> Result<bool, LinkError> {
        self.receive_frame(|bytes| {
            frame.clear();
            if frame.capacity() < MAX_FRAME {
                frame.reserve_exact(MAX_FRAME);
            }
            frame.extend_from_slice(bytes);
        })
        .map(|received| received.is_some())
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::RegionAllocator;
    use alloc::boxed::Box;

    const ENTRIES: u16 = 4;

    struct Fixture {
        net: VirtioNet,
        _registers: Box<[u64]>,
        _memory: Vec<u64>,
    }

    impl Fixture {
        fn new() -> Self {
            let mut registers = vec![0u64; 64].into_boxed_slice();
            let mut memory = vec![0u64; 8192];
            let start = memory.as_mut_ptr() as u64;
            // SAFETY: the fixture owns both stable allocations for every
            // access the driver makes; no device is involved.
            let (mmio, mut dma) = unsafe {
                (
                    Mmio::new(registers.as_mut_ptr() as u64, size_of_val(&registers[..])),
                    RegionAllocator::new(start..start + (memory.len() * 8) as u64),
                )
            };
            let mut queue = || Queue {
                ring: dma.allocate(4096, 4096).unwrap(),
                buffers: dma.allocate(usize::from(ENTRIES) * BUFFER, 4096).unwrap(),
                notify: mmio,
                entries: ENTRIES,
                driver: AVAIL,
                device: USED,
                used: 0,
                available: 0,
            };
            let (mut receive, transmit) = (queue(), queue());
            receive.prime_receive();
            Self {
                net: VirtioNet {
                    device: mmio,
                    receive,
                    transmit,
                    free: (0..ENTRIES).collect(),
                    broken: false,
                    mac: Mac([2, 0, 0, 0, 0, 1]),
                    status: false,
                    clock: Clock {
                        ticks_per_microsecond: 0,
                    },
                    location: "fixture".into(),
                    signalled: false,
                },
                _registers: registers,
                _memory: memory,
            }
        }
    }

    /// The device hands back descriptor `id` holding `length` bytes.
    fn give_back(queue: &mut Queue, produced: &mut u16, id: u32, length: u32) {
        let slot = USED + 4 + 8 * usize::from(*produced % queue.entries);
        queue.ring.write(slot, id);
        queue.ring.write(slot + 4, length);
        *produced = produced.wrapping_add(1);
        queue.ring.write(USED + 2, *produced);
    }

    #[test]
    fn receive_reposts_fixed_descriptors_across_ring_and_index_wrap() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        assert_eq!(net.receive.ring.read::<u16>(AVAIL + 2), ENTRIES);
        for id in 0..ENTRIES {
            let slot = usize::from(id);
            assert_eq!(net.receive.ring.read::<u16>(AVAIL + 4 + 2 * slot), id);
            assert_eq!(
                net.receive.ring.read::<u64>(16 * slot),
                net.receive.buffers.bus_address() + (slot * BUFFER) as u64
            );
            assert_eq!(net.receive.ring.read::<u32>(16 * slot + 8), BUFFER as u32);
            assert_eq!(
                net.receive.ring.read::<u16>(16 * slot + 12),
                DESCRIPTOR_WRITE
            );
            assert_eq!(net.receive.ring.read::<u16>(16 * slot + 14), 0);
        }
        let descriptors = net.receive.ring.bytes()[..16 * usize::from(ENTRIES)].to_vec();

        net.receive.available = u16::MAX;
        net.receive.ring.write(AVAIL + 2, u16::MAX);
        net.receive.used = u16::MAX;
        let mut produced = u16::MAX;
        net.receive.buffer(2)[HEADER..HEADER + 60].fill(0x44);
        give_back(&mut net.receive, &mut produced, 2, (HEADER + 60) as u32);
        let received = net.receive().unwrap().unwrap();
        assert_eq!(net.receive.ring.read::<u16>(AVAIL + 2), 0);
        assert_eq!(
            net.receive
                .ring
                .read::<u16>(AVAIL + 4 + 2 * usize::from(ENTRIES - 1)),
            2
        );
        net.receive.buffer(2)[HEADER..HEADER + 60].fill(0x77);
        assert_eq!(received, [0x44; 60]);
        assert_eq!(net.receive(), Ok(None));
        assert_eq!(&net.receive.ring.bytes()[..descriptors.len()], descriptors);
    }

    #[test]
    fn receive_reposts_across_wrap_and_rings_after_the_device_rearms() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        net.receive.available = u16::MAX;
        net.receive.ring.write(AVAIL + 2, u16::MAX);
        net.receive.used = u16::MAX;
        let mut produced = u16::MAX;
        for (id, flags, kick) in [(0u16, USED_NO_NOTIFY, false), (1, 0, true), (2, 0, true)] {
            net.receive.ring.write(USED, flags);
            net.receive.notify.write16(0, u16::MAX);
            net.receive.buffer(id)[HEADER..HEADER + 60].fill(id as u8);
            give_back(
                &mut net.receive,
                &mut produced,
                u32::from(id),
                (HEADER + 60) as u32,
            );
            assert_eq!(net.receive(), Ok(Some(vec![id as u8; 60])));
            assert_eq!(net.receive.ring.read::<u16>(AVAIL + 2), id);
            assert_eq!(net.receive.used, produced);
            let slot = usize::from(id.wrapping_sub(1) % ENTRIES);
            assert_eq!(net.receive.ring.read::<u16>(AVAIL + 4 + 2 * slot), id);
            assert_eq!(
                net.receive.notify.read16(0),
                if kick { 0 } else { u16::MAX }
            );
        }
    }

    #[test]
    fn transmit_publishes_even_while_the_device_suppresses_kicks() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        net.transmit.notify.write16(0, u16::MAX);
        net.transmit.ring.write(USED, USED_NO_NOTIFY);
        net.transmit(&[0x5a; 60]).unwrap();
        assert_eq!(net.transmit.ring.read::<u16>(AVAIL + 2), 1);
        assert_eq!(net.transmit.ring.read::<u16>(AVAIL + 4), 3);
        assert_eq!(net.transmit.notify.read16(0), u16::MAX);
        assert_eq!(&net.transmit.buffer(3)[HEADER..HEADER + 60], &[0x5a; 60]);

        net.transmit.ring.write(USED, 0u16);
        net.transmit(&[0x6b; 60]).unwrap();
        assert_eq!(net.transmit.ring.read::<u16>(AVAIL + 2), 2);
        assert_eq!(net.transmit.ring.read::<u16>(AVAIL + 6), 2);
        assert_eq!(net.transmit.notify.read16(0), 0);
    }

    #[test]
    fn reusable_receive_preserves_storage_and_empty_or_error_output() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        let mut produced = 0;
        let mut frame = vec![0x52; 17];
        assert_eq!(net.receive_reusing(&mut frame), Ok(false));
        assert_eq!(frame, [0x52; 17]);

        let mut storage = None;
        for (index, length) in [60, MAX_FRAME, ETHERNET_HEADER].into_iter().enumerate() {
            let id = index as u16;
            let byte = 0xa0 + index as u8;
            net.receive.buffer(id)[HEADER..HEADER + length].fill(byte);
            give_back(
                &mut net.receive,
                &mut produced,
                u32::from(id),
                (HEADER + length) as u32,
            );
            assert_eq!(net.receive_reusing(&mut frame), Ok(true));
            assert_eq!(frame.len(), length);
            assert!(frame.iter().all(|&value| value == byte));
            assert_eq!(net.receive.available, ENTRIES + id + 1);
            assert_eq!(*storage.get_or_insert(frame.as_ptr()), frame.as_ptr());
        }

        give_back(&mut net.receive, &mut produced, 3, (HEADER + 13) as u32);
        assert_eq!(net.receive_reusing(&mut frame), Ok(false));
        assert_eq!(net.receive.available, 2 * ENTRIES);
        assert_eq!(frame, [0xa2; ETHERNET_HEADER]);
        give_back(&mut net.receive, &mut produced, u32::from(ENTRIES), 64);
        assert!(net.receive_reusing(&mut frame).is_err());
        assert!(net.receive_reusing(&mut frame).is_err());
        assert_eq!(frame, [0xa2; ETHERNET_HEADER]);
    }

    #[test]
    fn bad_length_frames_are_dropped() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        let mut produced = 0;
        give_back(&mut net.receive, &mut produced, 0, (HEADER + 13) as u32);
        give_back(
            &mut net.receive,
            &mut produced,
            1,
            (HEADER + MAX_FRAME + 1) as u32,
        );
        net.receive.buffer(2)[HEADER..HEADER + 60].fill(0x5a);
        give_back(&mut net.receive, &mut produced, 2, (HEADER + 60) as u32);
        assert_eq!(net.receive(), Ok(Some(vec![0x5a; 60])));
        assert_eq!(net.receive(), Ok(None));
        // Every buffer, dropped or delivered, went back to the device.
        assert_eq!(net.receive.available, ENTRIES + 3);
    }

    #[test]
    fn a_descriptor_never_offered_breaks_the_device() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        let mut produced = 0;
        give_back(&mut net.receive, &mut produced, u32::from(ENTRIES), 64);
        assert!(net.receive().is_err());
        assert!(net.broken);
        assert!(net.transmit(&[0; 60]).is_err());
        assert!(net.receive().is_err());
    }

    #[test]
    fn a_transmit_descriptor_returned_twice_breaks_the_device() {
        let mut fixture = Fixture::new();
        let net = &mut fixture.net;
        net.transmit(&[1; 60]).unwrap();
        let sent = 3; // `free` hands out its last descriptor first
        let mut produced = 0;
        give_back(&mut net.transmit, &mut produced, sent, 0);
        give_back(&mut net.transmit, &mut produced, sent, 0);
        assert!(net.transmit(&[1; 60]).is_err());
        assert!(net.broken);
    }

    /// Small queues keep the transport's fixed layout; the receive queue's
    /// rings follow its 4 KiB of descriptors without overlapping.
    #[test]
    fn queue_layouts_keep_their_parts_apart() {
        assert_eq!(layout(ENTRIES), (AVAIL, USED, 4096));
        assert_eq!(layout(16), (AVAIL, USED, 4096));
        for entries in [32u16, TRANSMIT_ENTRIES, RECEIVE_ENTRIES, 1024] {
            let (driver, device, bytes) = layout(entries);
            let n = usize::from(entries);
            assert_eq!(driver, 16 * n);
            assert!(driver + 6 + 2 * n <= device);
            assert_eq!(device % 4, 0);
            assert!(device + 6 + 8 * n <= bytes);
            assert_eq!(bytes % 4096, 0);
        }
        assert_eq!(layout(RECEIVE_ENTRIES), (4096, 4616, 8192));
    }

    #[test]
    fn frames_without_an_ethernet_header_are_refused() {
        let mut fixture = Fixture::new();
        assert_eq!(
            fixture.net.transmit(&[0; 13]),
            Err(LinkError::Invalid("frame shorter than its header"))
        );
        assert_eq!(fixture.net.transmit(&[0; 14]), Ok(()));
    }
}
