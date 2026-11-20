//! Live host HD Audio playback bridge behind the guest's virtual USB Audio
//! Class 2.0 speaker (`stela_soc::usb::audio`).
//!
//! Picks the first discovered native HDA controller with a usable analog
//! output route (preferring a physically plugged-in jack when jack sense is
//! available, falling back to firmware Line-Out / Speaker ordering and minimal
//! transport-only codecs such as QEMU's `hda-duplex`).
//!
//! During each host service turn, stereo 48 kHz frames are drained from the
//! guest's USB audio ring outside the HDA lock, converted to 16-bit signed
//! little-endian PCM, and written ahead of the hardware LPIB playback cursor
//! under `RestoreRoot::try_enter` (so audio never stalls other cores waiting
//! for the host drivers' lock). Already-played bytes are scrubbed back to zero
//! behind LPIB so short sounds never loop on the next cyclic lap.
use super::{output, storage::RestoreRoot};
use alloc::{string::String, vec, vec::Vec};
use stela_boot_core::audio_bridge::{self, CHANNELS, FRAME_BYTES, RingBridge, TARGET_LEAD_BYTES};
use stela_drivers::{
    Clock,
    probe::native::hda::{Host as HdaHost, PlaybackInfo},
};
use stela_soc::machine::{AppleDevices, UsbAudioStatus};

/// Maximum stereo frames drained per service turn (1,024 frames = 21.33 ms).
pub(super) const TURN_FRAMES: usize = 1024;

/// Static silence slice used to scrub played DMA regions without heap allocation.
static ZERO_CHUNK: [u8; 4096] = [0u8; 4096];

pub struct HostAudio {
    host: Option<HdaHost>,
    unused: Vec<HdaHost>,
    location: String,
    #[allow(dead_code)]
    info: PlaybackInfo,
    root: u64,
    clock: Clock,
    bridge: RingBridge,
    failed: bool,
    last_bits: u8,
    last_muted: bool,
    last_volume: i16,
    reported_written: u64,
    pcm_scratch: Vec<u8>,
}

impl Drop for HostAudio {
    fn drop(&mut self) {
        let mut host = self.host.take();
        let unused = core::mem::take(&mut self.unused);
        super::network::in_loader_root(self.root, || {
            if let Some(host) = host.as_mut() {
                let _ = host.stop_playback();
            }
            // Keep HDA controller DMA/MMIO owners alive for the boot just as
            // storage.rs retains other native device owners.
            if let Some(host) = host {
                core::mem::forget(host);
            }
            Vec::leak(unused);
        });
    }
}

impl HostAudio {
    /// Start live 48 kHz 16-bit stereo analog playback on the first discovered
    /// HDA controller that exposes a usable analog output route.
    ///
    /// Must be called inside the loader's identity root (during `storage::open`).
    pub fn first(hosts: Vec<HdaHost>, root: u64, clock: Clock) -> Option<Self> {
        let mut active: Option<(HdaHost, PlaybackInfo)> = None;
        let mut unused = Vec::new();
        for mut host in hosts {
            if active.is_some() {
                output::arguments(format_args!("HOST AUDIO unused {}\n", host.location));
                unused.push(host);
                continue;
            }
            match host.start_playback() {
                Ok(info) => {
                    output::arguments(format_args!(
                        "HOST AUDIO guest uses {} codec={} pin={:#x} dac={:#x} device={:?} \
                         jack={} ring={} transport_only={}\n",
                        host.location,
                        info.codec,
                        info.pin,
                        info.converter,
                        info.device,
                        info.jack_detected,
                        info.ring_bytes,
                        info.transport_only,
                    ));
                    active = Some((host, info));
                }
                Err(error) => {
                    output::arguments(format_args!(
                        "HOST AUDIO skipped {}: {error:?}\n",
                        host.location
                    ));
                    unused.push(host);
                }
            }
        }
        let Some((host, info)) = active else {
            Vec::leak(unused);
            return None;
        };
        let location = host.location.clone();
        let bridge = RingBridge::new(info.ring_bytes, TARGET_LEAD_BYTES);
        Some(Self {
            host: Some(host),
            unused,
            location,
            info,
            root,
            clock,
            bridge,
            failed: false,
            last_bits: 0,
            last_muted: false,
            last_volume: 0,
            reported_written: 0,
            pcm_scratch: vec![0u8; TURN_FRAMES * FRAME_BYTES],
        })
    }

    /// Drain up to `out.len()` waiting stereo frames and status from the
    /// guest's virtual USB audio speaker under the devices lock.
    pub fn queued(
        devices: &mut AppleDevices,
        out: &mut [[i32; CHANNELS]],
    ) -> (usize, Option<UsbAudioStatus>) {
        let count = devices.audio_take(out);
        let status = devices.audio_status();
        (count, status)
    }

    fn fail(&mut self, what: &str, error: impl core::fmt::Debug) {
        if !self.failed {
            output::arguments(format_args!(
                "HOST AUDIO FAILED {} {what}: {error:?}\n",
                self.location
            ));
            self.failed = true;
        }
    }

    /// Pump guest PCM frames into the live HDA cyclic playback ring and scrub
    /// already-played bytes back to silence behind hardware LPIB.
    pub fn pump(&mut self, frames: &[[i32; CHANNELS]], status: Option<UsbAudioStatus>) {
        if let Some(st) = status
            && (st.bits != self.last_bits
                || st.muted != self.last_muted
                || st.volume != self.last_volume)
        {
            self.last_bits = st.bits;
            self.last_muted = st.muted;
            self.last_volume = st.volume;
            output::arguments(format_args!(
                "GUEST AUDIO bits={} muted={} volume_256db={} received={} overflowed={} split={}\n",
                st.bits,
                st.muted,
                st.volume,
                st.counters.received,
                st.counters.overflowed,
                st.counters.split,
            ));
        }
        if self.failed {
            return;
        }
        // SAFETY: the loader root `storage::open` captured maps the HDA BAR0
        // and coherent DMA ring for the entire boot.
        let Some(_root) = (unsafe { RestoreRoot::try_enter(self.root) }) else {
            return;
        };
        let Some(host) = self.host.as_mut() else {
            return;
        };
        let hw_status = match host.playback_status() {
            Ok(status) => status,
            Err(error) => return self.fail("status", error),
        };
        if !hw_status.running {
            return self.fail("stream stopped", "running=false");
        }
        let now_us = self.clock.now_us();
        let scrub = self
            .bridge
            .advance_and_plan_scrub(hw_status.position_bytes, now_us);
        for seg in scrub.as_slice() {
            let mut offset = seg.ring_offset;
            let mut remaining = seg.len;
            while remaining > 0 {
                let chunk = remaining.min(ZERO_CHUNK.len());
                if let Err(error) = host.write_playback(offset, &ZERO_CHUNK[..chunk]) {
                    return self.fail("scrub", error);
                }
                offset += chunk;
                remaining -= chunk;
            }
        }
        if !frames.is_empty() {
            let pcm_len = audio_bridge::frames_to_s16le(frames, &mut self.pcm_scratch);
            let write = self.bridge.plan_write(pcm_len);
            for seg in write.as_slice() {
                let slice = &self.pcm_scratch[seg.src_offset..seg.src_offset + seg.len];
                if let Err(error) = host.write_playback(seg.ring_offset, slice) {
                    return self.fail("write", error);
                }
            }
        }
    }

    /// Periodically log host playback counters once guest audio has streamed.
    pub fn report(&mut self) {
        let c = self.bridge.counters();
        if c.written_bytes > self.reported_written {
            self.reported_written = c.written_bytes;
            output::arguments(format_args!(
                "HOST AUDIO {} written_bytes={} dropped_bytes={} underruns={} buffered={}\n",
                self.location,
                c.written_bytes,
                c.dropped_bytes,
                c.underruns,
                self.bridge.buffered_bytes(),
            ));
        }
    }

    /// Stop active HDA playback DMA before machine shutdown or restart.
    pub fn stop(&mut self) {
        if let Some(host) = self.host.as_mut() {
            super::network::in_loader_root(self.root, || {
                let _ = host.stop_playback();
            });
        }
    }
}
