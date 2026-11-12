// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2016 Felix Fietkau <nbd@nbd.name>
// Copyright (C) 2020-2023 MediaTek Inc.
//! Raw non-WED/non-RRO WFDMA receive buffers. DDONE transfers ownership;
//! fragments preserve hardware length/drop/checksum observations for the MCU
//! or 802.11 decoder, and never claim an associated or authenticated packet.
use super::{Allocation, DESC_BYTES, DONE, Error, LAST, QueueIo};
use crate::bus::dma::{
    mapping::{Direction, DmaDomain},
    memory::PageSource,
};
use alloc::{sync::Arc, vec::Vec};
use core::sync::atomic::{Ordering, fence};
const BUFFER_BYTES: usize = 2048;
const MAX_COUNT: usize = 2048;

struct Slot {
    data: Allocation,
    posted: bool,
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Fragment {
    pub bytes: Vec<u8>,
    pub more: bool,
    pub drop: bool,
    pub control: u32,
    pub buffer1: u32,
    pub info: u32,
}
pub struct RxRing<Q: QueueIo> {
    io: Option<Q>,
    descriptors: Allocation,
    slots: Vec<Slot>,
    head: usize,
    tail: usize,
    poisoned: bool,
    released: bool,
}
impl<Q: QueueIo> RxRing<Q> {
    /// # Safety
    /// Same coherent-WB requester/BAR/domain and stopped-engine requirements as
    /// `TxRing::install`. WED/RRO and hardware header translation are disabled;
    /// the queue uses ordinary 16-byte descriptors and 2048-byte native buffers.
    /// Count need not be a power of two: the donor's data queues use 1536 entries.
    pub unsafe fn install(
        io: Q,
        source: &dyn PageSource,
        domain: Arc<dyn DmaDomain>,
        count: usize,
    ) -> Result<Self, Error> {
        if !(4..=MAX_COUNT).contains(&count) {
            return Err(Error::InvalidSize);
        }
        let descriptors = Allocation::new(
            source,
            domain.clone(),
            count * DESC_BYTES,
            Direction::Bidirectional,
        )?;
        let mut slots = Vec::with_capacity(count);
        for i in 0..count {
            descriptors.write32(i * DESC_BYTES + 4, DONE)?;
            slots.push(Slot {
                data: Allocation::new(source, domain.clone(), BUFFER_BYTES, Direction::FromDevice)?,
                posted: false,
            });
        }
        let address = descriptors.address()?;
        let mut ring = Self {
            io: Some(io),
            descriptors,
            slots,
            head: 0,
            tail: 0,
            poisoned: false,
            released: false,
        };
        // Queue base publication makes all descriptor bytes potentially reachable.
        ring.descriptors
            .dma
            .sync_for_device(0..count * DESC_BYTES)?;
        ring.descriptors.published = true;
        ring.io
            .as_mut()
            .expect("retained queue")
            .configure(address, count as u16)?;
        if ring.io.as_mut().expect("retained queue").device_index()? != 0 {
            return Err(Error::InvalidDeviceIndex);
        }
        for _ in 0..count - 1 {
            ring.post_head()?;
        }
        fence(Ordering::SeqCst);
        ring.io
            .as_mut()
            .expect("retained queue")
            .publish(ring.head as u16)?;
        Ok(ring)
    }
    fn post_head(&mut self) -> Result<(), Error> {
        let slot = &mut self.slots[self.head];
        if slot.posted {
            return Err(Error::OwnershipMismatch);
        }
        slot.data.dma.sync_for_device(0..BUFFER_BYTES)?;
        slot.data.published = true;
        let at = self.head * DESC_BYTES;
        self.descriptors.write32(at, slot.data.address()?)?;
        self.descriptors.write32(at + 8, 0)?;
        self.descriptors.write32(at + 12, 0)?;
        self.descriptors
            .write32(at + 4, (BUFFER_BYTES as u32) << 16)?;
        self.descriptors.dma.sync_for_device(at..at + DESC_BYTES)?;
        slot.posted = true;
        self.head = (self.head + 1) % self.slots.len();
        Ok(())
    }
    pub fn is_poisoned(&self) -> bool {
        self.poisoned
    }
    pub fn poll(&mut self) -> Result<Option<Fragment>, Error> {
        if self.poisoned || self.released {
            return Err(Error::Poisoned);
        }
        let result = self.poll_inner();
        if result.is_err() {
            self.poisoned = true
        }
        result
    }
    fn poll_inner(&mut self) -> Result<Option<Fragment>, Error> {
        if !self.slots[self.tail].posted {
            return Err(Error::OwnershipMismatch);
        }
        let at = self.tail * DESC_BYTES;
        // Checked-coherent WB volatile completion load: no Rust slice aliases device-written bytes.
        let control = self.descriptors.read32(at + 4)?;
        if control & DONE == 0 {
            return Ok(None);
        }
        fence(Ordering::SeqCst);
        // SAFETY: real DDONE transferred this exact descriptor to the CPU.
        unsafe { self.descriptors.dma.sync_for_cpu(at..at + DESC_BYTES)? };
        let length = ((control >> 16) & 0x3fff) as usize;
        // mt76_dma_rx_process drops a buffer whose length does not fit it,
        // and VER_MASK belongs to WED/RRO formats this ring never carries:
        // either is a dropped, empty fragment, and the descriptor is
        // reposted as any other. No byte of it is interpreted.
        let malformed = length == 0 || length > BUFFER_BYTES || control & (1 << 7) != 0;
        let buffer1 = self.descriptors.read32(at + 8)?;
        let info = self.descriptors.read32(at + 12)?;
        let slot = &mut self.slots[self.tail];
        // SAFETY: observed real DDONE on the checked-coherent descriptor transfers this payload
        // to the CPU. Reposting happens only after all bytes have been copied.
        unsafe { slot.data.dma.sync_for_cpu(0..BUFFER_BYTES)? };
        let length = if malformed { 0 } else { length };
        let mut bytes = vec![0u8; length];
        // SAFETY: the buffer is CPU-owned after the sync above, and `pointer` bounds
        // the copy to `length` bytes.
        unsafe {
            core::ptr::copy_nonoverlapping(
                slot.data.pointer(0, length)?,
                bytes.as_mut_ptr(),
                length,
            )
        };
        slot.posted = false;
        self.tail = (self.tail + 1) % self.slots.len();
        // Keep one unposted gap. Refill the prior head rather than immediately
        // returning the just-completed descriptor to hardware out of ring order.
        self.post_head()?;
        fence(Ordering::SeqCst);
        self.io
            .as_mut()
            .expect("retained queue")
            .publish(self.head as u16)?;
        Ok(Some(Fragment {
            bytes,
            more: control & LAST == 0,
            drop: malformed || control & ((1 << 12) | (1 << 14)) != 0,
            control,
            buffer1,
            info,
        }))
    }
    /// # Safety
    /// The owning controller has acknowledged WFDMA idle/reset, blocked all
    /// submissions and drained IRQ/deferred workers. Every current/stale DMA
    /// access to descriptors and payloads has ended, as for TX teardown.
    pub unsafe fn release_after_quiesce(&mut self) -> Result<(), Error> {
        self.poisoned = true;
        for slot in &mut self.slots {
            // SAFETY: covered by this function's own safety contract.
            unsafe { slot.data.release()? };
            slot.posted = false;
        }
        // SAFETY: covered by this function's own safety contract.
        unsafe { self.descriptors.release()? };
        self.released = true;
        Ok(())
    }
}

impl<Q: QueueIo> Drop for RxRing<Q> {
    fn drop(&mut self) {
        if self.descriptors.published && !self.released {
            super::retain_queue(self.io.take());
        }
    }
}

/// Bounded receive-chain assembly, preserving any fragment's hardware-drop
/// indication. It never clears a failed chain into a successful-looking frame.
/// A chain longer than its bounds is dropped whole, as mt76 frees an
/// over-long `rx_head`, and assembly goes on with the next chain.
#[derive(Default)]
pub struct Assembly {
    bytes: Vec<u8>,
    descriptors: Vec<Descriptor>,
    drop: bool,
    /// The current chain outgrew its bounds: its parts are counted, not kept.
    overflowed: bool,
    fragments: usize,
    poisoned: bool,
}
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct Frame {
    pub bytes: Vec<u8>,
    pub descriptors: Vec<Descriptor>,
    pub drop: bool,
    pub fragments: usize,
}
/// All DMA descriptor observations and their byte boundaries remain
/// attached to assembled RX bytes for checksum/drop/protocol consumers.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Descriptor {
    pub length: usize,
    pub control: u32,
    pub buffer1: u32,
    pub info: u32,
}
impl Assembly {
    /// CPU assembly ownership only; this is not hardware/IRQ quiescence.
    pub(in crate::wifi::mediatek) fn is_idle(&self) -> bool {
        self.bytes.is_empty() && self.descriptors.is_empty() && !self.poisoned
    }
    pub fn push(&mut self, fragment: Fragment) -> Result<Option<Frame>, Error> {
        if self.poisoned {
            return Err(Error::Poisoned);
        }
        self.fragments += 1;
        // A dropped part carries no bytes; an empty or oversized one that is
        // not marked dropped is dropped here.
        self.drop |=
            fragment.drop || fragment.bytes.is_empty() || fragment.bytes.len() > BUFFER_BYTES;
        if !self.overflowed
            && (self.descriptors.len() == 8 || self.bytes.len() + fragment.bytes.len() > 16 * 1024)
        {
            self.overflowed = true;
            self.drop = true;
            self.bytes = Vec::new();
            self.descriptors = Vec::new();
        }
        if !self.overflowed && fragment.bytes.len() <= BUFFER_BYTES {
            self.bytes.extend_from_slice(&fragment.bytes);
            self.descriptors.push(Descriptor {
                length: fragment.bytes.len(),
                control: fragment.control,
                buffer1: fragment.buffer1,
                info: fragment.info,
            });
        }
        if fragment.more {
            return Ok(None);
        }
        let frame = Frame {
            bytes: core::mem::take(&mut self.bytes),
            drop: self.drop,
            fragments: self.fragments,
            descriptors: core::mem::take(&mut self.descriptors),
        };
        self.drop = false;
        self.overflowed = false;
        self.fragments = 0;
        Ok(Some(frame))
    }
}

#[cfg(test)]
pub(crate) mod tests {
    extern crate std;
    use super::super::tests::{Domain, Io, Source, State};
    use super::*;
    use std::sync::Mutex;
    fn setup(count: usize) -> (RxRing<Io>, Arc<Mutex<State>>) {
        let state = Arc::new(Mutex::new(State::default()));
        let domain: Arc<dyn DmaDomain> = Arc::new(Domain(state.lock().unwrap().memory.clone()));
        // SAFETY: exclusive, idle queue in the stateful device model.
        let ring = unsafe { RxRing::install(Io(state.clone()), &Source, domain, count) }.unwrap();
        (ring, state)
    }
    pub(crate) fn device_receive(state: &Arc<Mutex<State>>, bytes: &[u8], last: bool, flags: u32) {
        let mut s = state.lock().unwrap();
        assert_ne!(s.index, s.head);
        let descriptor =
            s.memory.lock().unwrap().mappings[&s.base].1 + s.index as usize * DESC_BYTES;
        let ptr = descriptor as *mut u32;
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
        // accesses it concurrently.
        let address = u64::from(u32::from_le(unsafe { ptr.read_volatile() }));
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
        // accesses it concurrently.
        let control = u32::from_le(unsafe { ptr.add(1).read_volatile() });
        assert_eq!(control & DONE, 0);
        assert_eq!(control >> 16, BUFFER_BYTES as u32);
        let payload = s.memory.lock().unwrap().mappings[&address].1 as *mut u8;
        assert_ne!(payload as u64, address);
        assert!(bytes.len() <= BUFFER_BYTES);
        // Device writes the IOVA-backed payload before publishing DDONE.
        // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and nothing
        // accesses it concurrently.
        unsafe {
            core::ptr::copy_nonoverlapping(bytes.as_ptr(), payload, bytes.len());
            ptr.add(2).write_volatile(0x4321u32.to_le());
            ptr.add(3).write_volatile(0x1234u32.to_le());
            ptr.add(1).write_volatile(
                (DONE | if last { LAST } else { 0 } | ((bytes.len() as u32) << 16) | flags).to_le(),
            );
        }
        s.index = (s.index + 1) % s.count;
    }

    /// Publish a complete packet through buffer-sized DDONE fragments.
    /// The final descriptor alone terminates the native receive chain.
    pub(crate) fn device_receive_frame(
        state: &Arc<Mutex<State>>,
        bytes: &[u8],
        flags: u32,
    ) -> usize {
        assert!(!bytes.is_empty());
        let count = bytes.len().div_ceil(BUFFER_BYTES);
        let mut fragments = bytes.chunks(BUFFER_BYTES).peekable();
        while let Some(fragment) = fragments.next() {
            device_receive(state, fragment, fragments.peek().is_none(), flags);
        }
        count
    }
    #[test]
    fn receive_ddone_payload_and_gap_refill_wrap_non_power_of_two_rings() {
        if crate::test_process::isolate_model() {
            return;
        }
        let (mut ring, state) = setup(7);
        for i in 0..70u8 {
            assert_eq!(ring.poll().unwrap(), None);
            let expected = vec![i; 2048 - usize::from(i)];
            device_receive(&state, &expected, true, 0);
            let frame = ring.poll().unwrap().unwrap();
            assert_eq!(frame.bytes, expected);
            assert!(!frame.more);
            assert!(!frame.drop);
            assert_eq!(frame.buffer1, 0x4321);
            assert_eq!(frame.info, 0x1234);
        }
        assert_eq!(
            state
                .lock()
                .unwrap()
                .memory
                .lock()
                .unwrap()
                .syncs
                .iter()
                .filter(|v| v.0)
                .count(),
            140
        );
        // SAFETY: explicit model stop ends every receive-buffer device access.
        unsafe { ring.release_after_quiesce() }.unwrap();
        drop(ring);
        assert_eq!(state.lock().unwrap().queue_drops, 1);
        assert!(
            state
                .lock()
                .unwrap()
                .memory
                .lock()
                .unwrap()
                .mappings
                .is_empty()
        );
    }
    #[test]
    fn corrupt_length_and_wed_format_are_dropped_and_the_ring_goes_on() {
        if crate::test_process::isolate_model() {
            return;
        }
        for ctrl in [
            DONE | LAST,
            DONE | LAST | (2049 << 16),
            DONE | LAST | (40 << 16) | (1 << 7),
        ] {
            let (mut ring, state) = setup(4);
            let pointer = {
                let s = state.lock().unwrap();
                s.memory.lock().unwrap().mappings[&s.base].1 + 4
            };
            // SAFETY: the pointer addresses live memory this test's model owns, in bounds, and
            // nothing accesses it concurrently.
            unsafe { (pointer as *mut u32).write_volatile(ctrl.to_le()) };
            state.lock().unwrap().index = 1;
            let dropped = ring.poll().unwrap().unwrap();
            assert!(dropped.drop && dropped.bytes.is_empty() && !dropped.more);
            assert!(!ring.is_poisoned());
            // The next descriptor is received as ever.
            device_receive(&state, &[0x5a; 300], true, 0);
            let frame = ring.poll().unwrap().unwrap();
            assert_eq!(frame.bytes, [0x5a; 300]);
            assert!(!frame.drop);
            // SAFETY: no device continues accessing this stopped model queue.
            unsafe { ring.release_after_quiesce() }.unwrap();
        }
    }
    #[test]
    fn failed_rearm_keeps_queue_and_all_published_buffers_until_quiescence() {
        if crate::test_process::isolate_model() {
            return;
        }
        let (mut ring, state) = setup(4);
        device_receive(&state, &[0x5a; 200], true, 0);
        state.lock().unwrap().fail_publish = true;
        assert_eq!(ring.poll(), Err(Error::RegisterFailure));
        assert!(ring.is_poisoned());
        assert_eq!(
            state.lock().unwrap().memory.lock().unwrap().mappings.len(),
            5
        );
        assert_eq!(state.lock().unwrap().queue_drops, 0);
        // SAFETY: modeled device stopped before all mappings are removed.
        unsafe { ring.release_after_quiesce() }.unwrap();
        drop(ring);
        assert!(
            state
                .lock()
                .unwrap()
                .memory
                .lock()
                .unwrap()
                .mappings
                .is_empty()
        );
        assert_eq!(state.lock().unwrap().queue_drops, 1);
    }
    #[test]
    fn active_drop_keeps_owned_requester_and_all_posted_receive_pins() {
        if crate::test_process::isolate_model() {
            return;
        }
        let (ring, state) = setup(4);
        drop(ring);
        assert_eq!(
            state.lock().unwrap().memory.lock().unwrap().mappings.len(),
            4
        ); // descriptor + 3 posted
        assert_eq!(state.lock().unwrap().queue_drops, 0);
        device_receive(&state, &[0xa5; 100], true, 0);
    }
    #[test]
    fn fragment_assembly_bounds_the_whole_chain() {
        if crate::test_process::isolate_model() {
            return;
        }
        let fragment = |more, drop| Fragment {
            bytes: vec![0x71; 2048],
            more,
            drop,
            control: 0,
            buffer1: 0,
            info: 0,
        };
        let mut a = Assembly::default();
        assert_eq!(a.push(fragment(true, false)).unwrap(), None);
        assert_eq!(a.push(fragment(true, true)).unwrap(), None);
        let f = a.push(fragment(false, false)).unwrap().unwrap();
        assert_eq!(f.bytes, [0x71; 6144]);
        assert_eq!(f.fragments, 3);
        assert!(f.drop);
        let f = a.push(fragment(false, false)).unwrap().unwrap();
        assert!(!f.drop);
        assert_eq!(f.fragments, 1);
        // An over-long chain is dropped whole; the next one assembles.
        for _ in 0..8 {
            assert_eq!(a.push(fragment(true, false)).unwrap(), None);
        }
        let f = a.push(fragment(false, false)).unwrap().unwrap();
        assert!(f.drop && f.bytes.is_empty() && f.descriptors.is_empty());
        assert_eq!(f.fragments, 9);
        let f = a.push(fragment(false, false)).unwrap().unwrap();
        assert_eq!((f.drop, f.bytes.len(), f.fragments), (false, 2048, 1));
        // A dropped empty part drops its chain without its bytes.
        let empty = Fragment {
            bytes: Vec::new(),
            ..fragment(true, true)
        };
        assert_eq!(a.push(empty).unwrap(), None);
        let f = a.push(fragment(false, false)).unwrap().unwrap();
        assert!(f.drop);
        assert_eq!((f.bytes.len(), f.fragments), (2048, 2));
    }
    #[test]
    fn donor_1536_entry_data_queue_is_supported_without_power_of_two_assumptions() {
        if crate::test_process::isolate_model() {
            return;
        }
        let (mut ring, state) = setup(1536);
        assert_eq!(state.lock().unwrap().head, 1535);
        device_receive(&state, &[0x39; 2048], true, 0);
        assert_eq!(ring.poll().unwrap().unwrap().bytes, [0x39; 2048]);
        // SAFETY: the complete modeled receive engine is stopped.
        unsafe { ring.release_after_quiesce() }.unwrap();
        assert!(
            state
                .lock()
                .unwrap()
                .memory
                .lock()
                .unwrap()
                .mappings
                .is_empty()
        );
    }
    #[test]
    fn every_completed_descriptor_payload_and_rearm_visibility_failure_keeps_ownership() {
        if crate::test_process::isolate_model() {
            return;
        }
        for fail_at in 8..=11 {
            let (mut ring, state) = setup(4);
            state.lock().unwrap().memory.lock().unwrap().fail_sync_at = Some(fail_at);
            device_receive(&state, &[0x42; 200], true, 0);
            assert_eq!(
                ring.poll(),
                Err(Error::Dma(
                    crate::bus::dma::mapping::DmaError::VisibilityFailed
                ))
            );
            assert!(ring.is_poisoned());
            assert_eq!(state.lock().unwrap().queue_drops, 0);
            assert_eq!(
                state.lock().unwrap().memory.lock().unwrap().mappings.len(),
                5
            );
            // SAFETY: explicit model stop permits quiescent invalidation.
            unsafe { ring.release_after_quiesce() }.unwrap();
            drop(ring);
            assert!(
                state
                    .lock()
                    .unwrap()
                    .memory
                    .lock()
                    .unwrap()
                    .mappings
                    .is_empty()
            );
            assert_eq!(state.lock().unwrap().queue_drops, 1);
        }
    }
}
