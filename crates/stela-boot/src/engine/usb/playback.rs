//! Moves bounded batches from the guest UAC2 speaker to an available USB DAC.
use super::{RestoreRoot, output};
use crate::engine::{host, parallel::Board};
use alloc::{collections::VecDeque, vec, vec::Vec};
use core::sync::atomic::{AtomicU64, Ordering};
use stela_boot_core::audio_pcm::Pcm;
use stela_drivers::{
    audio::Direction,
    usb::audio::{Error, UsbAudio},
};
use stela_soc::usb::audio::RATE;

const FRAMES_PER_MS: usize = RATE as usize / 1000;
const TARGET: usize = 40 * FRAMES_PER_MS;
const PREFILL: usize = 20 * FRAMES_PER_MS;
const LOW: usize = 15 * FRAMES_PER_MS;
const BATCH: usize = 10 * FRAMES_PER_MS;
const MAX_PCM: usize = TARGET * 2 * 3;

// Shared clock, since any PE may take a service turn. Zero disables requests.
static NEXT: AtomicU64 = AtomicU64::new(0);

pub(in crate::engine) fn due() -> bool {
    let next = NEXT.load(Ordering::Relaxed);
    next != 0 && host::fresh_counter() >= next
}

pub(super) struct Playback {
    outputs: VecDeque<Output>,
    samples: Vec<[i32; 2]>,
    pcm: Vec<u8>,
    pending: usize,
    running: bool,
    next: u64,
    corrected: u64,
    underruns: u64,
    reported: bool,
}

struct Output {
    device: UsbAudio,
    alternate: usize,
    format: Pcm,
}

impl Output {
    fn of(device: UsbAudio) -> Option<Self> {
        let formats = &device.binding().formats;
        for (format, bits) in [(Pcm::I16, 16), (Pcm::I24, 24)] {
            let alternate = formats.iter().position(|f| {
                f.direction == Direction::Playback
                    && f.channels == 2
                    && f.bits == bits
                    && usize::from(f.subslot) == format.sample_bytes()
                    && f.power_domain.is_none()
                    && f.endpoint_rates
                        .as_ref()
                        .is_none_or(|rates| rates.ranges.iter().any(|range| range.contains(RATE)))
            });
            if let Some(alternate) = alternate {
                return Some(Self {
                    device,
                    alternate,
                    format,
                });
            }
        }
        None
    }
}

impl Playback {
    /// Called while the host driver root is active, so incompatible owners
    /// drop with their controller and DMA storage mapped.
    pub(super) fn select(
        devices: Vec<UsbAudio>,
        inventory: &[stela_drivers::usb::xhci::Device],
    ) -> Option<Self> {
        let mut outputs: VecDeque<_> = devices.into_iter().filter_map(Output::of).collect();
        let selected = outputs.front().map(|output| output.device.location());
        for device in inventory.iter().filter(|device| {
            device.configurations.iter().any(|configuration| {
                stela_drivers::usb::descriptor::interfaces(configuration)
                    .iter()
                    .any(|interface| interface.class == 1 && matches!(interface.subclass, 1 | 2))
            })
        }) {
            let status = if selected == Some(device.location.as_str()) {
                " (selected)"
            } else if outputs
                .iter()
                .any(|output| output.device.location() == device.location)
            {
                " (compatible USB)"
            } else {
                " (not supported)"
            };
            output::arguments(format_args!(
                "AUDIO OUTPUT {}{status}\n",
                device.name().escape_debug()
            ));
        }
        if let Some(primary) = outputs.pop_front() {
            let name = primary.device.product_name();
            // A fallback with another name would leave the guest's existing
            // USB product descriptor pointing at the wrong hardware.
            outputs.retain(|output| output.device.product_name() == name);
            outputs.push_front(primary);
        }
        (!outputs.is_empty()).then(|| Self {
            outputs,
            samples: vec![[0; 2]; TARGET + 1],
            pcm: vec![0; MAX_PCM],
            pending: 0,
            running: false,
            next: 0,
            corrected: 0,
            underruns: 0,
            reported: false,
        })
    }

    pub(super) fn guest_name(&self) -> Option<&str> {
        self.outputs
            .front()
            .map(|output| output.device.product_name())
    }

    fn again(&mut self, now: u64, millis: u64) {
        self.next = now + host::COUNTER_HZ / 1000 * millis;
        NEXT.store(if self.running { self.next } else { 0 }, Ordering::Relaxed);
    }

    pub(super) fn step(&mut self, root: u64, board: &Board, boot: bool) {
        let now = host::fresh_counter();
        if now < self.next {
            return;
        }
        let Some(output) = self.outputs.front_mut() else {
            return;
        };
        let buffered = output.device.buffered_playback_frames();
        let Some(active) = board.try_with(|board| {
            let speaker = board.usb_audio()?;
            if !speaker.streaming() {
                self.pending = 0;
                if speaker.queued() != 0 {
                    speaker.take(&mut self.samples);
                }
                return Some(false);
            }
            if self.pending != 0 || (!self.running && (!boot || speaker.queued() < PREFILL)) {
                return Some(true);
            }
            let queued = speaker.queued();
            let frames = if self.running {
                TARGET.saturating_sub(buffered).min(BATCH)
            } else {
                queued.min(TARGET)
            };
            // Correct small clock differences in sufficiently large batches.
            // A real shortage uses silence only when the DAC is close to empty.
            let can_correct = self.running
                && frames >= 2 * FRAMES_PER_MS
                && now.wrapping_sub(self.corrected) >= host::COUNTER_HZ / 100;
            let wanted = if can_correct && queued > TARGET {
                frames + 1
            } else if can_correct && queued >= frames - 1 && queued < PREFILL {
                frames - 1
            } else {
                frames
            };
            let consumed = speaker.take(&mut self.samples[..wanted]);
            let stretch = wanted != frames && consumed == wanted;
            if stretch {
                self.corrected = now;
            }
            let rendered = if !self.running || buffered < LOW || stretch {
                frames
            } else {
                consumed.min(frames)
            };
            self.pending = rendered * 2 * output.format.sample_bytes();
            output.format.render(
                &self.samples[..consumed],
                &mut self.pcm[..self.pending],
                stretch,
            );
            Some(true)
        }) else {
            self.again(now, 2);
            return;
        };
        let Some(active) = active else {
            return;
        };
        if (!active && !self.running) || (!boot && (active != self.running)) {
            self.again(now, if active { 4 } else { 20 });
            return;
        }
        if active && !self.running && self.pending == 0 {
            self.again(now, 4);
            return;
        }
        // The guest board lock has been released before any USB driver call.
        // SAFETY: this is the live loader root kept by HostUsb.
        let Some(root_guard) = (unsafe { RestoreRoot::try_enter(root) }) else {
            self.again(now, 2);
            return;
        };
        let result = if !active {
            self.running = false;
            output.device.stop_playback()
        } else if !self.running {
            let result = output.device.start_playback(
                output.alternate,
                RATE,
                &self.pcm[..self.pending],
                false,
            );
            self.running = result.is_ok();
            result
        } else {
            output
                .device
                .write_playback(&self.pcm[..self.pending])
                .and_then(|()| output.device.poll().map(|_| ()))
        };
        self.pending = 0;
        let bits = output.format.sample_bytes() * 8;
        let failed = !matches!(result, Ok(()) | Err(Error::Underrun));
        let name = (failed || (self.running && !self.reported))
            .then(|| alloc::string::String::from(output.device.product_name()));
        if failed {
            // Owners may need mapped controller registers while dropping.
            drop(self.outputs.pop_front());
            self.running = false;
            self.corrected = 0;
            self.underruns = 0;
            self.reported = false;
        }
        drop(root_guard);
        let name = name.as_deref().unwrap_or("USB audio");
        match result {
            Ok(()) => {
                if self.running && !self.reported {
                    self.reported = true;
                    output::arguments(format_args!(
                        "AUDIO USB playback started rate={RATE} bits={bits} channels=2 device={name:?}\n",
                    ));
                }
                self.again(now, if self.running { 2 } else { 20 });
            }
            Err(Error::Underrun) => {
                // The driver retired the stream before reporting an underrun.
                self.running = false;
                self.underruns += 1;
                if self.underruns.is_power_of_two() {
                    output::arguments(format_args!(
                        "AUDIO USB playback underruns={}\n",
                        self.underruns
                    ));
                }
                self.again(now, 10);
            }
            Err(error) => {
                output::arguments(format_args!(
                    "AUDIO USB playback unavailable device={name:?}: {error:?}\n"
                ));
                self.again(now, 4);
            }
        }
    }

    pub(super) fn stop(&mut self, root: u64) {
        NEXT.store(0, Ordering::Relaxed);
        if self.outputs.is_empty() {
            return;
        }
        // SAFETY: the service is stopping, with no host driver guard held
        // and the same live loader root as its normal work.
        let _root = unsafe { RestoreRoot::enter(root) };
        if self.running
            && let Some(output) = self.outputs.front_mut()
        {
            if let Err(error) = output.device.stop_playback() {
                output::arguments(format_args!("AUDIO USB playback stop: {error:?}\n"));
            }
        }
        self.outputs.clear();
        self.running = false;
    }
}
