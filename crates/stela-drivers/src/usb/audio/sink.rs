// SPDX-License-Identifier: GPL-2.0-or-later
//! A continuous 48 kHz stereo output on a USB audio owner, for PCM that
//! arrives in bursts from another clock (a guest's audio device).
//!
//! The sink picks the owner's first playback format that runs at 48 kHz,
//! preferring two channels, then more, then one, and 24-bit samples, then
//! 16-bit, then 32-bit. Frames come in as 32-bit samples MSB-aligned; each is
//! written in the format's subslot with its low bits cleared below the bit
//! resolution. Stereo fills the first two channels and leaves the rest
//! silent; a mono format takes the average.
//!
//! What the owner holds (queued PCM and packets in flight) stays between
//! [`LOW_FRAMES`] and [`HIGH_FRAMES`]: frames that would pass the high mark
//! are dropped, and below the low mark silence brings it to
//! [`REFILL_FRAMES`], both counted. The two clocks drift apart slowly, so
//! this costs an occasional frame, never a stopped stream. A stream starts
//! with the first frames and stops once its source stops and what it holds
//! has played. An underrun or packet failure stops the stream, counted, and
//! the next frames start it again; [`MAX_START_FAILURES`] failed starts in a
//! row, or a poisoned owner, end the sink, reported once.
//!
//! Every call is bounded: it moves at most what the owner can hold and never
//! waits for the device.
use super::{AudioDevice, Error, Format, Transport};
use crate::audio::Direction;
use alloc::vec::Vec;

/// The rate the sink plays.
pub const RATE: u32 = 48_000;
/// Frames per millisecond.
const MS: u64 = RATE as u64 / 1000;
/// The least the owner should hold before silence is added: 40 ms, so a
/// service 50 ms late still finds packets queued.
pub const LOW_FRAMES: u64 = 40 * MS;
/// What silence fills up to: 60 ms.
pub const REFILL_FRAMES: u64 = 60 * MS;
/// The most the owner holds; frames past it are dropped: 120 ms.
pub const HIGH_FRAMES: u64 = 120 * MS;
/// Failed starts in a row that end the sink.
pub const MAX_START_FAILURES: u32 = 3;

/// The sink's counts since it was made, in frames unless named otherwise.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Counters {
    /// Frames from the source written to the owner.
    pub played: u64,
    /// Silence written because the owner ran low.
    pub silence: u64,
    /// Source frames dropped at the high mark.
    pub dropped: u64,
    /// Streams the owner stopped by underrunning.
    pub underruns: u64,
    /// Streams stopped by another owner error.
    pub faults: u64,
    pub starts: u64,
}

/// A change the owner of the sink reports.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Note {
    /// A stream started on this format.
    Started {
        interface: u8,
        alternate: u8,
        channels: u8,
        bits: u8,
    },
    /// The source stopped and its frames played.
    Stopped,
    /// The stream stopped with this error; the next frames restart it.
    Interrupted(Error),
    /// The sink ended with this error and drops what comes.
    Failed(Error),
}

pub struct Sink<T: Transport> {
    device: AudioDevice<T>,
    /// Playback format indices, in preference order.
    candidates: Vec<usize>,
    /// The running stream's format index and format.
    running: Option<(usize, Format)>,
    /// Frames written and the owner's played count when the stream began.
    written: u64,
    base: u64,
    start_failures: u32,
    failed: bool,
    counters: Counters,
    notes: Vec<Note>,
    scratch: Vec<u8>,
}

fn preference(format: &Format) -> (u8, u8) {
    let channels = match format.channels {
        2 => 0,
        c if c > 2 => 1,
        _ => 2,
    };
    let bits = match format.bits {
        24 => 0,
        16 => 1,
        32 => 2,
        _ => 3,
    };
    (channels, bits)
}

/// Append `frame` in `format`'s layout.
pub(crate) fn encode(frame: [i32; 2], format: &Format, out: &mut Vec<u8>) {
    let subslot = usize::from(format.subslot).clamp(1, 4);
    let bits = u32::from(format.bits).clamp(1, 32);
    let mask = (!0u32 << (32 - bits)) as i32;
    let mono = (frame[0] >> 1) + (frame[1] >> 1);
    let channels = usize::from(format.channels);
    for channel in 0..channels {
        let sample = match channels {
            1 => mono,
            _ => frame.get(channel).copied().unwrap_or(0),
        };
        out.extend_from_slice(&(sample & mask).to_le_bytes()[4 - subslot..]);
    }
}

impl<T: Transport> Sink<T> {
    /// A sink on `device`; `None` when it has no playback format.
    pub fn new(device: AudioDevice<T>) -> Option<Self> {
        let formats = &device.binding().formats;
        let mut candidates: Vec<usize> = (0..formats.len())
            .filter(|&n| formats[n].direction == Direction::Playback && formats[n].channels > 0)
            .collect();
        if candidates.is_empty() {
            return None;
        }
        candidates.sort_by_key(|&n| preference(&formats[n]));
        Some(Self {
            device,
            candidates,
            running: None,
            written: 0,
            base: 0,
            start_failures: 0,
            failed: false,
            counters: Counters::default(),
            notes: Vec::new(),
            scratch: Vec::new(),
        })
    }

    pub fn device(&self) -> &AudioDevice<T> {
        &self.device
    }

    #[cfg(test)]
    pub(crate) fn device_mut(&mut self) -> &mut AudioDevice<T> {
        &mut self.device
    }

    pub fn counters(&self) -> Counters {
        self.counters
    }

    pub fn running(&self) -> bool {
        self.running.is_some()
    }

    /// Whether the sink ended; it then drops everything.
    pub fn failed(&self) -> bool {
        self.failed
    }

    /// Changes since the last call, oldest first.
    pub fn take_notes(&mut self) -> Vec<Note> {
        core::mem::take(&mut self.notes)
    }

    /// Frames the owner holds: written and not yet played.
    fn held(&self) -> u64 {
        let played = self
            .device
            .progress()
            .playback_frames
            .saturating_sub(self.base);
        self.written.saturating_sub(played)
    }

    fn fail(&mut self, error: Error) {
        self.failed = true;
        self.running = None;
        let _ = self.device.shutdown();
        self.notes.push(Note::Failed(error));
    }

    /// The stream stopped with `error`; the owner has shut it down.
    fn interrupted(&mut self, error: Error) {
        self.running = None;
        if error == Error::Poisoned || self.device.last_fault == Some(Error::Poisoned) {
            return self.fail(error);
        }
        if error == Error::Underrun {
            self.counters.underruns += 1;
        } else {
            self.counters.faults += 1;
        }
        self.notes.push(Note::Interrupted(error));
    }

    fn start(&mut self) {
        let mut last = Error::Format;
        for &index in &self.candidates {
            let format = self.device.binding().formats[index].clone();
            self.scratch.clear();
            for _ in 0..REFILL_FRAMES {
                encode([0, 0], &format, &mut self.scratch);
            }
            match self
                .device
                .start_playback(index, RATE, &self.scratch, false)
            {
                Ok(()) => {
                    self.base = self.device.progress().playback_frames;
                    self.written = REFILL_FRAMES;
                    self.counters.silence += REFILL_FRAMES;
                    self.counters.starts += 1;
                    self.start_failures = 0;
                    self.notes.push(Note::Started {
                        interface: format.interface,
                        alternate: format.alternate,
                        channels: format.channels,
                        bits: format.bits,
                    });
                    self.running = Some((index, format));
                    return;
                }
                // Another format may take the rate or the endpoint.
                Err(error @ (Error::RateUnsupported | Error::Busy | Error::Format)) => last = error,
                Err(error) => {
                    last = error;
                    break;
                }
            }
        }
        if last == Error::Poisoned {
            return self.fail(last);
        }
        self.start_failures += 1;
        if self.start_failures >= MAX_START_FAILURES {
            self.fail(last);
        }
    }

    fn write(&mut self, frames: &[[i32; 2]]) -> Result<(), Error> {
        let Some((_, format)) = &self.running else {
            return Ok(());
        };
        self.scratch.clear();
        for &frame in frames {
            encode(frame, format, &mut self.scratch);
        }
        self.device.write_playback(&self.scratch)?;
        self.written += frames.len() as u64;
        Ok(())
    }

    /// One service: `frames` from the source, and whether the source still
    /// streams. Plays what fits, keeps the owner fed and polls it once.
    pub fn push(&mut self, frames: &[[i32; 2]], streaming: bool) {
        if self.failed {
            self.counters.dropped += frames.len() as u64;
            return;
        }
        if self.running.is_some()
            && let Err(error) = self.device.poll()
        {
            // A source that stopped and whose frames all played.
            if error == Error::Underrun && !streaming && frames.is_empty() {
                self.running = None;
                self.notes.push(Note::Stopped);
                return;
            }
            self.interrupted(error);
            if self.failed {
                self.counters.dropped += frames.len() as u64;
                return;
            }
        }
        if self.running.is_none() {
            if frames.is_empty() {
                return;
            }
            self.start();
            if self.running.is_none() {
                self.counters.dropped += frames.len() as u64;
                return;
            }
        }
        let room = HIGH_FRAMES.saturating_sub(self.held()) as usize;
        let (take, rest) = frames.split_at(frames.len().min(room));
        self.counters.dropped += rest.len() as u64;
        if let Err(error) = self.write(take) {
            return self.stop_with(error);
        }
        self.counters.played += take.len() as u64;
        let held = self.held();
        if !streaming && frames.is_empty() {
            if held < LOW_FRAMES {
                match self.device.stop_playback() {
                    Ok(()) => {
                        self.running = None;
                        self.notes.push(Note::Stopped);
                    }
                    Err(error) => self.fail(error),
                }
            }
            return;
        }
        if held < LOW_FRAMES {
            let silence = REFILL_FRAMES - held;
            if let Err(error) = self.write(&alloc::vec![[0, 0]; silence as usize]) {
                return self.stop_with(error);
            }
            self.counters.silence += silence;
        }
        // Hand what was written to the device's free packets now.
        if let Err(error) = self.device.poll() {
            self.interrupted(error);
        }
    }

    fn stop_with(&mut self, error: Error) {
        let _ = self.device.stop_playback();
        self.interrupted(error);
    }
}
