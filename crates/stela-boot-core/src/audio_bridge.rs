//! Host cyclic DMA ring scheduler and PCM converter for bridging guest USB
//! Audio Class 2.0 (48 kHz stereo MSB-aligned 32-bit) streams into a live HDA
//! 16-bit stereo playback ring buffer.
//!
//! Pure, `no_std` state machine so it can be thoroughly tested on the host
//! while running inside the UEFI boot engine (`stela-boot`).

/// Sample rate shared by `stela_soc::usb::audio` and the HDA playback stream.
pub const RATE: u32 = 48_000;
/// Stereo channel count.
pub const CHANNELS: usize = 2;
/// Bytes per interleaved 16-bit stereo frame (`2 channels * 2 bytes`).
pub const FRAME_BYTES: usize = 4;
/// Bytes per HDA BDL period (4 KiB = 1,024 stereo frames = 21.33 ms).
pub const PERIOD_BYTES: usize = 4096;
/// Number of periods in the cyclic HDA DMA ring (8 * 4 KiB = 32 KiB = 170.67 ms).
pub const PERIODS: usize = 8;
/// Total cyclic DMA ring buffer size in bytes.
pub const RING_BYTES: usize = PERIOD_BYTES * PERIODS;
/// Target playback lead ahead of hardware LPIB when starting from silence (~21.3 ms).
pub const TARGET_LEAD_BYTES: usize = PERIOD_BYTES;
/// Maximum buffered audio kept ahead of hardware LPIB (~149.3 ms), leaving one
/// full guard period between the write cursor and hardware playback head.
pub const MAX_LEAD_BYTES: usize = RING_BYTES - PERIOD_BYTES;
/// Wall-clock gap above which hardware may have lapped the cyclic ring (~140 ms).
pub const STALL_US: u64 = 140_000;

/// Converts MSB-aligned 32-bit stereo frames (as returned by
/// `stela_soc::usb::audio::Audio::take`) into interleaved signed 16-bit
/// little-endian stereo PCM bytes.
///
/// Returns the number of PCM bytes written into `out` (always a multiple of 4).
pub fn frames_to_s16le(frames: &[[i32; CHANNELS]], out: &mut [u8]) -> usize {
    let count = frames.len().min(out.len() / FRAME_BYTES);
    for (frame, dst) in frames[..count]
        .iter()
        .zip(out[..count * FRAME_BYTES].chunks_exact_mut(FRAME_BYTES))
    {
        let left = (frame[0] >> 16) as i16;
        let right = (frame[1] >> 16) as i16;
        let [l0, l1] = left.to_le_bytes();
        let [r0, r1] = right.to_le_bytes();
        dst[0] = l0;
        dst[1] = l1;
        dst[2] = r0;
        dst[3] = r1;
    }
    count * FRAME_BYTES
}

/// One contiguous ring-buffer transfer segment (never wraps past `ring_bytes`).
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Segment {
    pub ring_offset: usize,
    pub src_offset: usize,
    pub len: usize,
}

/// Up to two contiguous ring-buffer segments representing a possibly wrapped
/// copy or zero-scrub range.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct SpanPlan {
    count: usize,
    segments: [Segment; 2],
}

impl SpanPlan {
    pub fn as_slice(&self) -> &[Segment] {
        &self.segments[..self.count]
    }

    pub fn is_empty(&self) -> bool {
        self.count == 0
    }

    pub fn total_bytes(&self) -> usize {
        self.as_slice().iter().map(|s| s.len).sum()
    }

    fn push(&mut self, segment: Segment) {
        if segment.len == 0 {
            return;
        }
        debug_assert!(self.count < 2);
        self.segments[self.count] = segment;
        self.count += 1;
    }

    fn wrapped(ring_bytes: usize, start: usize, len: usize) -> Self {
        let mut plan = Self::default();
        if len == 0 || ring_bytes == 0 {
            return plan;
        }
        let clamped = len.min(ring_bytes);
        let first = clamped.min(ring_bytes - start);
        plan.push(Segment {
            ring_offset: start,
            src_offset: 0,
            len: first,
        });
        let second = clamped - first;
        if second > 0 {
            plan.push(Segment {
                ring_offset: 0,
                src_offset: first,
                len: second,
            });
        }
        plan
    }
}

/// Diagnostic counters for the host audio bridge.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct BridgeCounters {
    pub written_bytes: u64,
    pub dropped_bytes: u64,
    pub underruns: u64,
}

/// Tracks hardware playback progress (`LPIB`) around a cyclic HDA DMA buffer,
/// schedules trailing silence scrubs so played audio never loops, and schedules
/// wrapped PCM writes safely ahead of the hardware read pointer.
pub struct RingBridge {
    ring_bytes: usize,
    lead_bytes: usize,
    max_lead_bytes: usize,
    last_hw_pos: usize,
    write_cursor: usize,
    buffered: usize,
    dirty_remaining: usize,
    last_poll_us: Option<u64>,
    counters: BridgeCounters,
}

impl RingBridge {
    pub fn new(ring_bytes: usize, lead_bytes: usize) -> Self {
        let ring_bytes = (ring_bytes & !3).max(FRAME_BYTES * 4);
        let lead_bytes = (lead_bytes & !3).clamp(FRAME_BYTES, ring_bytes / 2);
        let max_lead_bytes = ring_bytes.saturating_sub(lead_bytes).max(lead_bytes);
        Self {
            ring_bytes,
            lead_bytes,
            max_lead_bytes,
            last_hw_pos: 0,
            write_cursor: lead_bytes % ring_bytes,
            buffered: 0,
            dirty_remaining: 0,
            last_poll_us: None,
            counters: BridgeCounters::default(),
        }
    }

    pub fn counters(&self) -> BridgeCounters {
        self.counters
    }

    pub fn buffered_bytes(&self) -> usize {
        self.buffered
    }

    /// Observe the hardware's current playback byte coordinate (`LPIB`) and
    /// monotonic timestamp (`now_us`).
    ///
    /// Returns zero-scrub segments covering already-played bytes that still
    /// contain non-zero PCM so the cyclic ring falls completely silent once
    /// the guest stops streaming.
    pub fn advance_and_plan_scrub(&mut self, hw_pos_raw: u32, now_us: u64) -> SpanPlan {
        let hw_pos = ((hw_pos_raw as usize) % self.ring_bytes) & !3;
        let Some(prev_us) = self.last_poll_us else {
            self.last_poll_us = Some(now_us);
            self.last_hw_pos = hw_pos;
            return SpanPlan::default();
        };
        let elapsed = now_us.saturating_sub(prev_us);
        self.last_poll_us = Some(now_us);
        if elapsed >= STALL_US {
            self.last_hw_pos = hw_pos;
            if self.buffered > 0 {
                self.counters.underruns += 1;
            }
            self.buffered = 0;
            if self.dirty_remaining > 0 {
                self.dirty_remaining = 0;
                return SpanPlan::wrapped(self.ring_bytes, 0, self.ring_bytes);
            }
            return SpanPlan::default();
        }
        let prev = self.last_hw_pos;
        let advanced = (hw_pos + self.ring_bytes - prev) % self.ring_bytes;
        self.last_hw_pos = hw_pos;
        if advanced > 0 && self.buffered > 0 {
            if advanced >= self.buffered {
                self.buffered = 0;
            } else {
                self.buffered -= advanced;
            }
        }
        if self.dirty_remaining > 0 && advanced > 0 {
            self.dirty_remaining = self.dirty_remaining.saturating_sub(advanced);
            return SpanPlan::wrapped(self.ring_bytes, prev, advanced);
        }
        SpanPlan::default()
    }

    /// Plan wrapped writes for `pcm_bytes` of fresh 16-bit stereo PCM ahead of
    /// the hardware read pointer.
    pub fn plan_write(&mut self, pcm_bytes: usize) -> SpanPlan {
        let pcm_bytes = pcm_bytes & !3;
        if pcm_bytes == 0 {
            return SpanPlan::default();
        }
        if self.buffered == 0 {
            self.write_cursor = (self.last_hw_pos + self.lead_bytes) % self.ring_bytes;
            self.buffered = self.lead_bytes;
        }
        let room = self.max_lead_bytes.saturating_sub(self.buffered) & !3;
        let to_write = pcm_bytes.min(room);
        self.counters.dropped_bytes += (pcm_bytes - to_write) as u64;
        if to_write == 0 {
            return SpanPlan::default();
        }
        let plan = SpanPlan::wrapped(self.ring_bytes, self.write_cursor, to_write);
        self.write_cursor = (self.write_cursor + to_write) % self.ring_bytes;
        self.buffered += to_write;
        self.dirty_remaining = self.ring_bytes;
        self.counters.written_bytes += to_write as u64;
        plan
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn converts_msb_aligned_32bit_stereo_to_s16le() {
        let frames = [
            [0x7fff_0000u32 as i32, 0x8000_0000u32 as i32],
            [0x1234_5600u32 as i32, -0x0100_0000i32],
        ];
        let mut out = [0u8; 16];
        let n = frames_to_s16le(&frames, &mut out);
        assert_eq!(n, 8);
        assert_eq!(i16::from_le_bytes([out[0], out[1]]), i16::MAX);
        assert_eq!(i16::from_le_bytes([out[2], out[3]]), i16::MIN);
        assert_eq!(i16::from_le_bytes([out[4], out[5]]), 0x1234);
        assert_eq!(i16::from_le_bytes([out[6], out[7]]), -0x0100);
    }

    #[test]
    fn steady_streaming_wraps_cleanly_and_scrubs_behind_hw() {
        let mut ring = [0u8; RING_BYTES];
        let mut bridge = RingBridge::new(RING_BYTES, TARGET_LEAD_BYTES);

        // Initial silent poll at hw=0.
        assert!(bridge.advance_and_plan_scrub(0, 1_000).is_empty());

        // Write 960 bytes (5 ms) of non-zero audio.
        let src = [0x42u8; 960];
        let write = bridge.plan_write(src.len());
        assert_eq!(write.total_bytes(), 960);
        assert_eq!(write.as_slice()[0].ring_offset, TARGET_LEAD_BYTES);
        for seg in write.as_slice() {
            ring[seg.ring_offset..seg.ring_offset + seg.len]
                .copy_from_slice(&src[seg.src_offset..seg.src_offset + seg.len]);
        }

        // Advance hardware all the way around the ring in 4096-byte steps with no more writes.
        let mut hw = 0u32;
        let mut t = 1_000u64;
        for _ in 0..PERIODS {
            hw = (hw + PERIOD_BYTES as u32) % RING_BYTES as u32;
            t += 21_000;
            let scrub = bridge.advance_and_plan_scrub(hw, t);
            for seg in scrub.as_slice() {
                ring[seg.ring_offset..seg.ring_offset + seg.len].fill(0);
            }
        }
        // Entire ring must now be scrubbed back to pure silence so sound never loops.
        assert!(ring.iter().all(|&b| b == 0));
        // Further idle advances produce no unnecessary scrub writes.
        assert!(
            bridge
                .advance_and_plan_scrub(PERIOD_BYTES as u32, t + 21_000)
                .is_empty()
        );
    }

    #[test]
    fn write_splits_cleanly_across_ring_end() {
        let mut bridge = RingBridge::new(RING_BYTES, TARGET_LEAD_BYTES);
        // Hardware near end so write_cursor starts 256 bytes before RING_BYTES.
        let hw = (RING_BYTES - TARGET_LEAD_BYTES - 256) as u32;
        assert!(bridge.advance_and_plan_scrub(hw, 0).is_empty());

        let write = bridge.plan_write(1024);
        assert_eq!(write.as_slice().len(), 2);
        assert_eq!(
            write.as_slice()[0],
            Segment {
                ring_offset: RING_BYTES - 256,
                src_offset: 0,
                len: 256,
            }
        );
        assert_eq!(
            write.as_slice()[1],
            Segment {
                ring_offset: 0,
                src_offset: 256,
                len: 768,
            }
        );
    }

    #[test]
    fn stall_recovery_scrubs_full_ring_and_counts_underrun() {
        let mut bridge = RingBridge::new(RING_BYTES, TARGET_LEAD_BYTES);
        assert!(bridge.advance_and_plan_scrub(0, 10_000).is_empty());
        assert_eq!(bridge.plan_write(1024).total_bytes(), 1024);

        // Simulate a 200 ms stall (exceeds STALL_US).
        let scrub = bridge.advance_and_plan_scrub(8192, 210_000);
        assert_eq!(scrub.total_bytes(), RING_BYTES);
        assert_eq!(bridge.buffered_bytes(), 0);
        assert_eq!(bridge.counters().underruns, 1);

        // Next write resyncs cleanly TARGET_LEAD_BYTES ahead of current hw_pos (8192).
        let next = bridge.plan_write(512);
        assert_eq!(next.as_slice()[0].ring_offset, 8192 + TARGET_LEAD_BYTES);
    }

    #[test]
    fn caps_writes_to_max_lead_without_overwriting_unplayed_audio() {
        let mut bridge = RingBridge::new(RING_BYTES, TARGET_LEAD_BYTES);
        assert!(bridge.advance_and_plan_scrub(0, 0).is_empty());
        // First write snaps buffered to TARGET_LEAD_BYTES (4096), leaving MAX_LEAD_BYTES - 4096 = 24576 bytes.
        let plan = bridge.plan_write(40_000);
        assert_eq!(plan.total_bytes(), MAX_LEAD_BYTES - TARGET_LEAD_BYTES);
        assert_eq!(
            bridge.counters().dropped_bytes,
            (40_000 - (MAX_LEAD_BYTES - TARGET_LEAD_BYTES)) as u64
        );
    }
}
