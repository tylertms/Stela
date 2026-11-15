// SPDX-License-Identifier: GPL-2.0-only
// Copyright (c) 2026 Stela contributors
//! Native descriptor ownership. DMA storage survives every error until reboot.
use super::{LinkError, MAX_FRAME, MIN_FRAME, registers::*};
use crate::{
    block::BlockError,
    bus::dma::{DmaAllocator, DmaBuffer},
    ring::write_transmit_frame,
};
#[cfg(test)]
use alloc::vec::Vec;
use core::sync::atomic::{Ordering, fence};

pub(super) struct Ring {
    pub descriptors: DmaBuffer,
    buffers: DmaBuffer,
    next: usize,
    pub packets: u64,
    pub discarded: u64,
}
impl Ring {
    pub fn new(dma: &mut dyn DmaAllocator, wide: bool) -> Result<Self, BlockError> {
        let descriptors = dma.allocate(DESCRIPTORS * DESCRIPTOR, 256)?;
        let buffers = dma.allocate(DESCRIPTORS * BUFFER, 256)?;
        for allocation in [&descriptors, &buffers] {
            if !wide && !fits32(allocation.bus_address(), allocation.len()) {
                return Err(BlockError::Unsupported(
                    "RTL8169/8168B requires DMA below 4 GiB",
                ));
            }
        }
        Ok(Self {
            descriptors,
            buffers,
            next: 0,
            packets: 0,
            discarded: 0,
        })
    }
    /// Call only before DMA starts, or after a completed controller reset.
    pub fn initialize(&mut self, receive: bool) {
        self.next = 0;
        for index in 0..DESCRIPTORS {
            let offset = index * DESCRIPTOR;
            self.descriptors.write(
                offset + 8,
                self.buffers.bus_address() + (index * BUFFER) as u64,
            );
            self.descriptors.write(offset + 4, 0u32);
            self.descriptors.write(offset, end(index));
            if receive {
                self.offer(index);
            }
        }
    }
    fn offer(&mut self, index: usize) {
        self.descriptors.write(index * DESCRIPTOR + 4, 0u32);
        fence(Ordering::Release);
        self.descriptors
            .write(index * DESCRIPTOR, OWN | end(index) | BUFFER as u32);
    }
    pub fn available(&self) -> bool {
        self.descriptors.read::<u32>(self.next * DESCRIPTOR) & OWN == 0
    }
    pub fn next_status(&self) -> u32 {
        self.descriptors.read(self.next * DESCRIPTOR)
    }
    pub fn owned(&self) -> usize {
        (0..DESCRIPTORS)
            .filter(|index| self.descriptors.read::<u32>(index * DESCRIPTOR) & OWN != 0)
            .count()
    }
    pub fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        if !(14..=MAX_FRAME).contains(&frame.len()) {
            return Err(LinkError::Invalid("frame length is outside 14..=1514"));
        }
        if !self.available() {
            return Err(LinkError::Busy);
        }
        fence(Ordering::Acquire);
        let length = frame.len().max(MIN_FRAME);
        let data = &mut self.buffers.bytes_mut()[self.next * BUFFER..self.next * BUFFER + length];
        write_transmit_frame(data, frame, length);
        let offset = self.next * DESCRIPTOR;
        self.descriptors.write(offset + 4, 0u32);
        // Publish OWN last. Never rewrite a device-owned descriptor or buffer.
        fence(Ordering::Release);
        self.descriptors
            .write(offset, OWN | end(self.next) | FIRST | LAST | length as u32);
        self.next = (self.next + 1) % DESCRIPTORS;
        self.packets = self.packets.saturating_add(1);
        Ok(())
    }
    #[cfg(test)]
    pub fn receive(&mut self) -> Option<Vec<u8>> {
        self.receive_with(<[u8]>::to_vec)
    }
    /// Completes the next receive slot with `frame` and a four-byte FCS,
    /// as the controller would.
    #[cfg(test)]
    pub fn deliver(&mut self, frame: &[u8]) {
        let start = self.next * BUFFER;
        let bytes = self.buffers.bytes_mut();
        bytes[start..start + frame.len()].copy_from_slice(frame);
        bytes[start + frame.len()..start + frame.len() + 4].fill(0xcd);
        self.descriptors.write(
            self.next * DESCRIPTOR,
            FIRST | LAST | end(self.next) | (frame.len() + 4) as u32,
        );
    }
    /// The next good frame, passed to `take` while its buffer is still
    /// held, before the slot goes back to the controller.
    pub fn receive_with<T>(&mut self, mut take: impl FnMut(&[u8]) -> T) -> Option<T> {
        for _ in 0..DESCRIPTORS {
            let status = self.descriptors.read::<u32>(self.next * DESCRIPTOR);
            if status & OWN != 0 {
                return None;
            }
            fence(Ordering::Acquire);
            let frame = received_length(status).map(|length| {
                take(&self.buffers.bytes()[self.next * BUFFER..self.next * BUFFER + length])
            });
            if frame.is_some() {
                self.packets = self.packets.saturating_add(1);
            } else {
                self.discarded = self.discarded.saturating_add(1);
            }
            self.offer(self.next);
            self.next = (self.next + 1) % DESCRIPTORS;
            if frame.is_some() {
                return frame;
            }
        }
        None
    }
}
/// A completed receive descriptor's frame length without the FCS, as Linux
/// `rtl8169_rx_interrupt` decodes it: none with an error bit, none for a
/// fragment (a frame over one buffer), and none outside 14..=1514 bytes.
fn received_length(status: u32) -> Option<usize> {
    let length = (status & LENGTH) as usize;
    (status & RX_ERRORS == 0
        && status & (FIRST | LAST) == FIRST | LAST
        && (18..=MAX_FRAME + 4).contains(&length))
    .then(|| length - 4)
}
fn end(index: usize) -> u32 {
    if index == DESCRIPTORS - 1 { END } else { 0 }
}
fn fits32(address: u64, length: usize) -> bool {
    address
        .checked_add(length as u64)
        .is_some_and(|end| end <= 1u64 << 32)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::TestRegion;

    #[test]
    fn dma_limit_checks_entire_range_and_overflow() {
        assert!(fits32(0xffff_f000, 4096));
        assert!(!fits32(0xffff_f000, 4097));
        assert!(!fits32(1u64 << 32, 1));
        assert!(!fits32(u64::MAX - 3, 8));
    }
    #[test]
    fn receive_status_decodes_length_fragments_and_errors() {
        let whole = FIRST | LAST;
        assert_eq!(received_length(whole | 64), Some(60));
        assert_eq!(received_length(whole | 18), Some(14));
        assert_eq!(
            received_length(whole | (MAX_FRAME as u32 + 4)),
            Some(MAX_FRAME)
        );
        assert_eq!(received_length(whole | 17), None);
        assert_eq!(received_length(whole | (MAX_FRAME as u32 + 5)), None);
        // Checksum results, the ring end and the address-match bits above
        // the 14-bit length change neither the length nor the verdict.
        let above = (7 << 14) | (1 << 17) | (1 << 18) | (7 << 25) | END;
        assert_eq!(received_length(whole | above | 64), Some(60));
        // RWT, RES, RUNT and CRC each drop the frame.
        for bit in 19..=22 {
            assert_eq!(received_length(whole | (1 << bit) | 64), None, "{bit}");
        }
        // Fragments of a frame over one buffer are dropped, as Linux does.
        for fragment in [FIRST, LAST, 0] {
            assert_eq!(received_length(fragment | 64), None);
        }
    }
    #[test]
    fn device_ownership_ring_wrap_padding_fcs_and_corruption_boundaries() {
        // Both rings, plus room for alignment.
        let mut dma = TestRegion::new((2 * DESCRIPTORS * (DESCRIPTOR + BUFFER) + 4096) / 8 * 8);
        let mut tx = Ring::new(&mut dma, true).unwrap();
        let mut rx = Ring::new(&mut dma, true).unwrap();
        tx.initialize(false);
        rx.initialize(true);
        // Each slot sees every length, and the ring wraps.
        for pass in 0..4 {
            for index in 0..DESCRIPTORS {
                let length = [14, 59, 60, MAX_FRAME][(pass + index) % 4];
                let frame: Vec<u8> = (0..length)
                    .map(|p| (p * 19 + pass * 7 + index) as u8)
                    .collect();
                tx.transmit(&frame).unwrap();
                let offset = index * DESCRIPTOR;
                let status = tx.descriptors.read::<u32>(offset);
                assert_eq!(
                    status,
                    OWN | end(index) | FIRST | LAST | length.max(60) as u32
                );
                let address = tx.descriptors.read::<u64>(offset + 8);
                assert_eq!(address, tx.buffers.bus_address() + (index * BUFFER) as u64);
                let packet = &tx.buffers.bytes()[index * BUFFER..index * BUFFER + length.max(60)];
                assert_eq!(&packet[..length], frame);
                assert!(packet[length..].iter().all(|b| *b == 0));
                // Independent device completion: strip OWN, leave all remaining bits.
                tx.descriptors.write(offset, status & !OWN);
                let wirelen = length + 4;
                rx.buffers.bytes_mut()[index * BUFFER..index * BUFFER + length]
                    .copy_from_slice(&frame);
                rx.buffers.bytes_mut()[index * BUFFER + length..index * BUFFER + wirelen]
                    .fill(0xcd);
                rx.descriptors
                    .write(offset, FIRST | LAST | end(index) | wirelen as u32);
                assert_eq!(rx.receive().unwrap(), frame);
                assert_eq!(
                    rx.descriptors.read::<u32>(offset),
                    OWN | end(index) | BUFFER as u32
                );
            }
        }
        for _ in 0..DESCRIPTORS {
            tx.transmit(&[0xab; 14]).unwrap();
        }
        let original = tx.buffers.bytes().to_vec();
        let descriptors = tx.descriptors.bytes().to_vec();
        assert!(tx.transmit(&[0xcd; 14]).is_err());
        assert_eq!(tx.buffers.bytes(), original);
        assert_eq!(tx.descriptors.bytes(), descriptors);
        assert!(tx.transmit(&[0; 13]).is_err());
        assert!(tx.transmit(&[0; MAX_FRAME + 1]).is_err());
        for bad in [
            0u32,
            3,
            17,
            MAX_FRAME as u32 + 5,
            LENGTH,
            FIRST | 18,
            LAST | 18,
            FIRST | LAST | 18 | RX_ERRORS,
        ] {
            let index = rx.next;
            rx.descriptors.write(index * DESCRIPTOR, bad | end(index));
            assert_eq!(rx.receive(), None);
            assert_eq!(
                rx.descriptors.read::<u32>(index * DESCRIPTOR),
                OWN | end(index) | BUFFER as u32
            );
        }
        // All bad descriptors are bounded to one pass; receive cannot spin forever.
        for index in 0..DESCRIPTORS {
            rx.descriptors.write(index * DESCRIPTOR, end(index));
        }
        assert_eq!(rx.receive(), None);
    }
}
