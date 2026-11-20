//! A native high-speed USB audio output: a USB Audio Class 2.0 speaker
//! whose PCM the machine owner plays on a host audio device.
//!
//! Evidence: USB Audio Class 2.0 (Audio Device Class Definition, release
//! 2.0): the Interface Association (4.6), the class-specific AC interface
//! header (4.7.2), Clock Source (4.7.2.1), Input and Output Terminals
//! (4.7.2.4-5), Feature Unit (4.7.2.8), the AS interface and isochronous
//! endpoint descriptors (4.9.2, 4.10.1.2), the CUR and RANGE requests and
//! their parameter blocks (5.2.1-5.2.3), the Clock Source frequency and
//! validity controls (5.2.5.1.1-2) and the Feature Unit mute and volume
//! controls (5.2.5.7.1-2); Frmts20 2.3.1 for Type I PCM (samples MSB first
//! in little-endian subslots). macOS binds its own class driver to a
//! UAC2 function, so this needs no Apple-specific audio hardware.
//!
//! The function is one stereo output: a fixed internal 48 kHz clock (ID 1,
//! frequency and validity read-only), a USB streaming input terminal (ID
//! 2), a feature unit with master mute and volume (ID 3) and a speaker
//! output terminal (ID 4). There is no input terminal, so the host sees no
//! capture device. Interface 1 streams: alternate setting 1 carries 16-bit
//! and setting 2 24-bit samples (3-byte subslots), each on isochronous OUT
//! endpoint 1, adaptive: the host sends at its own clock's rate and the
//! owner absorbs the difference. Each 1 ms service interval carries up to
//! one frame more than the nominal 48.
//!
//! Received frames go to a ring of [`RING_FRAMES`] stereo frames, kept as
//! 32-bit samples MSB-aligned. A packet arriving at a full ring drops the
//! oldest frames, counted, so the output never falls further behind the
//! guest than the ring holds. The owner takes frames with [`Audio::take`],
//! which applies mute and volume. A packet that splits a frame loses the
//! split frame's bytes, counted.
//!
//! The clock frequency request accepts SET CUR of 48 kHz although the
//! control is read-only, as some hosts write it regardless; any other
//! frequency STALLs. Volume is -60 dB to 0 dB in 0.5 dB steps; a value
//! outside the range is clamped and one between steps rounded down.
use super::{
    BusPeer, Function, Reply, Token,
    control::{
        self, CLASS, CLEAR_FEATURE, Control, DEVICE, DEVICE_QUALIFIER, Descriptors, ENDPOINT,
        ENDPOINT_HALT, GET_INTERFACE, GET_STATUS, INTERFACE, Outcome, RECIPIENT_ENDPOINT,
        RECIPIENT_INTERFACE, Requests, SET_FEATURE, SET_INTERFACE, STANDARD, Setup, Stall,
    },
};
use crate::Result;
use alloc::{collections::VecDeque, string::String, vec, vec::Vec};

/// The only sample rate.
pub const RATE: u32 = 48_000;
pub const CHANNELS: usize = 2;
/// Stereo frames the ring holds: 200 ms.
pub const RING_FRAMES: usize = RATE as usize / 5;
/// Frames received since the owner last took any that make the function
/// ask for a host service: 5 ms.
const DUE_FRAMES: usize = RATE as usize / 200;

const STREAM: u8 = 0x01;
const CONTROL_INTERFACE: u16 = 0;
const STREAM_INTERFACE: u16 = 1;
const CLOCK: u8 = 1;
const INPUT: u8 = 2;
const FEATURE: u8 = 3;
const OUTPUT: u8 = 4;

// Class-specific descriptor types and subtypes.
const CS_INTERFACE: u8 = 0x24;
const CS_ENDPOINT: u8 = 0x25;
const HEADER: u8 = 0x01;
const INPUT_TERMINAL: u8 = 0x02;
const OUTPUT_TERMINAL: u8 = 0x03;
const FEATURE_UNIT: u8 = 0x06;
const CLOCK_SOURCE: u8 = 0x0a;
const AS_GENERAL: u8 = 0x01;
const FORMAT_TYPE: u8 = 0x02;
const EP_GENERAL: u8 = 0x01;

// Requests and control selectors.
const CUR: u8 = 0x01;
const RANGE: u8 = 0x02;
const SAM_FREQ: u8 = 0x01;
const CLOCK_VALID: u8 = 0x02;
const MUTE: u8 = 0x01;
const VOLUME: u8 = 0x02;

/// Volume range in 1/256 dB.
pub const VOLUME_MIN: i16 = -60 * 256;
pub const VOLUME_MAX: i16 = 0;
const VOLUME_STEP: i16 = 128;
/// The gain of one 0.5 dB step down, 10^(-0.5/20), in Q30.
const STEP_GAIN: i64 = 1_013_677_647;
const UNITY: i64 = 1 << 30;

/// The streaming settings: (alternate, subslot bytes, bit resolution).
const SETTINGS: [(u8, u8, u8); 2] = [(1, 2, 16), (2, 3, 24)];
/// Frames one service interval carries at most: the nominal 48 and one.
const PACKET_FRAMES: usize = RATE as usize / 1000 + 1;

/// Counters for the owner's diagnostics.
#[derive(Debug, Clone, Copy, Default, PartialEq, Eq)]
pub struct Counters {
    /// Frames the guest sent.
    pub received: u64,
    /// Frames dropped at a full ring.
    pub overflowed: u64,
    /// Bytes of packets that split a frame, dropped.
    pub split: u64,
}

/// What the owner reports about the stream.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Status {
    /// The selected streaming setting's bit resolution, 0 while idle.
    pub bits: u8,
    pub muted: bool,
    /// 1/256 dB.
    pub volume: i16,
    pub counters: Counters,
}

pub struct Audio {
    control: Control,
    state: State,
}

struct State {
    alternate: u8,
    muted: bool,
    volume: i16,
    /// The volume's gain in Q30.
    gain: i64,
    ring: VecDeque<[i32; CHANNELS]>,
    /// Frames received since the owner last took frames.
    fresh: usize,
    due: bool,
    counters: Counters,
}

pub const DEFAULT_NAME: &str = "Stela USB Audio";

fn descriptors(name: String) -> Descriptors {
    let device = [
        18,
        DEVICE,
        0x00,
        0x02,
        0xef, // miscellaneous: functions described by interface associations
        0x02,
        0x01,
        control::EP0_PACKET as u8,
        0x09,
        0x12, // vendor 0x1209 (pid.codes)
        0x03,
        0x00, // its test product ID
        0x00,
        0x01,
        1,
        2,
        3,
        1,
    ];
    // The AC interface's class-specific descriptors, after its header.
    let clock = [8, CS_INTERFACE, CLOCK_SOURCE, CLOCK, 0x01, 0x05, 0, 0];
    let input = [
        17,
        CS_INTERFACE,
        INPUT_TERMINAL,
        INPUT,
        0x01,
        0x01, // USB streaming
        0,
        CLOCK,
        CHANNELS as u8,
        0x03, // front left, front right
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    // Master mute and volume read-write; no per-channel controls.
    let feature = [
        18,
        CS_INTERFACE,
        FEATURE_UNIT,
        FEATURE,
        INPUT,
        0x0f,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
        0,
    ];
    let output = [
        12,
        CS_INTERFACE,
        OUTPUT_TERMINAL,
        OUTPUT,
        0x01,
        0x03, // speaker
        0,
        FEATURE,
        CLOCK,
        0,
        0,
        0,
    ];
    let total = (9 + clock.len() + input.len() + feature.len() + output.len()) as u16;
    let [total_low, total_high] = total.to_le_bytes();
    // bcdADC 2.00, category desktop speaker.
    let header = [
        9,
        CS_INTERFACE,
        HEADER,
        0x00,
        0x02,
        0x01,
        total_low,
        total_high,
        0,
    ];
    let configuration = |interval: u8| {
        let mut streams: Vec<Vec<u8>> = vec![vec![9, INTERFACE, 1, 0, 0, 1, 2, 0x20, 0]];
        for (alternate, subslot, bits) in SETTINGS {
            let [low, high] =
                ((PACKET_FRAMES * CHANNELS * usize::from(subslot)) as u16).to_le_bytes();
            streams.push(vec![9, INTERFACE, 1, alternate, 1, 1, 2, 0x20, 0]);
            streams.push(vec![
                16,
                CS_INTERFACE,
                AS_GENERAL,
                INPUT,
                0,
                1, // format type I
                1, // PCM
                0,
                0,
                0,
                CHANNELS as u8,
                0x03,
                0,
                0,
                0,
                0,
            ]);
            streams.push(vec![6, CS_INTERFACE, FORMAT_TYPE, 1, subslot, bits]);
            // Isochronous, adaptive, data.
            streams.push(vec![7, ENDPOINT, STREAM, 0x09, low, high, interval]);
            streams.push(vec![8, CS_ENDPOINT, EP_GENERAL, 0, 0, 0, 0, 0]);
        }
        let mut parts: Vec<&[u8]> = vec![
            // One association of the control and streaming interfaces.
            &[8, 0x0b, 0, 2, 1, 0, 0x20, 0],
            &[9, INTERFACE, 0, 0, 0, 1, 1, 0x20, 0],
            &header,
            &clock,
            &input,
            &feature,
            &output,
        ];
        parts.extend(streams.iter().map(Vec::as_slice));
        control::configuration(1, 0, &parts)
    };
    let qualifier = [
        10,
        DEVICE_QUALIFIER,
        0x00,
        0x02,
        0xef,
        0x02,
        0x01,
        control::EP0_PACKET as u8,
        1,
        0,
    ];
    Descriptors {
        device,
        // High speed: 2^(4-1) microframes, 1 ms; full speed: one frame.
        configuration: configuration(4),
        other_speed: Some((qualifier, configuration(1))),
        strings: vec![String::from("Stela"), name, String::from("0001")],
    }
}

/// The gain of `volume` (1/256 dB, in range) in Q30.
fn gain(volume: i16) -> i64 {
    let steps = (VOLUME_MAX - volume) / VOLUME_STEP;
    (0..steps).fold(UNITY, |gain, _| (gain * STEP_GAIN) >> 30)
}

/// A RANGE parameter block of one subrange (5.2.1): 2-byte or 4-byte
/// fields by `width`.
fn range(width: usize, min: i32, max: i32, resolution: i32) -> Vec<u8> {
    let mut bytes = 1u16.to_le_bytes().to_vec();
    for value in [min, max, resolution] {
        bytes.extend_from_slice(&value.to_le_bytes()[..width]);
    }
    bytes
}

impl Default for Audio {
    fn default() -> Self {
        Self::new()
    }
}

impl Audio {
    pub fn new() -> Self {
        Self::named(String::from(DEFAULT_NAME))
    }

    pub fn named(name: String) -> Self {
        Self {
            control: Control::new(descriptors(name)),
            state: State {
                alternate: 0,
                muted: false,
                volume: VOLUME_MAX,
                gain: UNITY,
                ring: VecDeque::with_capacity(RING_FRAMES),
                fresh: 0,
                due: false,
                counters: Counters::default(),
            },
        }
    }

    /// The selected setting's (subslot bytes, bit resolution), while the
    /// guest streams.
    fn setting(&self) -> Option<(u8, u8)> {
        let alternate = self.state.alternate;
        SETTINGS
            .iter()
            .find(|(a, ..)| *a == alternate && self.control.configuration() != 0)
            .map(|&(_, subslot, bits)| (subslot, bits))
    }

    /// Whether the guest has a streaming setting selected.
    pub fn streaming(&self) -> bool {
        self.setting().is_some()
    }

    pub fn status(&self) -> Status {
        Status {
            bits: self.setting().map_or(0, |(_, bits)| bits),
            muted: self.state.muted,
            volume: self.state.volume,
            counters: self.state.counters,
        }
    }

    /// Frames waiting for the owner.
    pub fn queued(&self) -> usize {
        self.state.ring.len()
    }

    /// Moves up to `out.len()` frames to `out`, mute and volume applied;
    /// the number moved.
    pub fn take(&mut self, out: &mut [[i32; CHANNELS]]) -> usize {
        let state = &mut self.state;
        let n = out.len().min(state.ring.len());
        let gain = if state.muted { 0 } else { state.gain };
        for (slot, frame) in out.iter_mut().zip(state.ring.drain(..n)) {
            *slot = frame.map(|sample| ((i64::from(sample) * gain) >> 30) as i32);
        }
        state.fresh = 0;
        n
    }

    /// Whether enough frames arrived since the owner last took any that it
    /// should run soon; cleared by asking.
    pub fn take_due(&mut self) -> bool {
        core::mem::take(&mut self.state.due)
    }
}

impl State {
    fn select(&mut self, alternate: u8) {
        self.alternate = alternate;
    }

    /// One isochronous OUT packet of `subslot`-byte samples.
    fn receive(&mut self, packet: &[u8], subslot: usize) {
        let frame = subslot * CHANNELS;
        let (frames, rest) = packet.split_at(packet.len() / frame * frame);
        self.counters.split += rest.len() as u64;
        for bytes in frames.chunks_exact(frame) {
            if self.ring.len() == RING_FRAMES {
                self.ring.pop_front();
                self.counters.overflowed += 1;
            }
            let mut samples = [0i32; CHANNELS];
            for (sample, raw) in samples.iter_mut().zip(bytes.chunks_exact(subslot)) {
                // MSB-aligned in 32 bits: the subslot's bytes on top.
                let mut word = [0u8; 4];
                word[4 - subslot..].copy_from_slice(raw);
                *sample = i32::from_le_bytes(word);
            }
            self.ring.push_back(samples);
        }
        let count = frames.len() / frame;
        self.counters.received += count as u64;
        self.fresh += count;
        if self.fresh >= DUE_FRAMES {
            self.due = true;
        }
    }

    /// A class request to an entity of the control interface.
    fn entity(&mut self, setup: Setup, data: &[u8]) -> Result<Outcome, Stall> {
        let entity = (setup.index >> 8) as u8;
        let (selector, channel) = ((setup.value >> 8) as u8, setup.value as u8);
        let read = setup.device_to_host();
        match (entity, selector, channel, setup.request, read) {
            (CLOCK, SAM_FREQ, 0, CUR, true) => Ok(Outcome::Data(RATE.to_le_bytes().to_vec())),
            (CLOCK, SAM_FREQ, 0, RANGE, true) => {
                Ok(Outcome::Data(range(4, RATE as i32, RATE as i32, 0)))
            }
            (CLOCK, SAM_FREQ, 0, CUR, false) if data == RATE.to_le_bytes() => Ok(Outcome::Done),
            (CLOCK, CLOCK_VALID, 0, CUR, true) => Ok(Outcome::Data(vec![1])),
            (FEATURE, MUTE, 0, CUR, true) => Ok(Outcome::Data(vec![u8::from(self.muted)])),
            (FEATURE, MUTE, 0, CUR, false) if data.len() == 1 => {
                self.muted = data[0] != 0;
                Ok(Outcome::Done)
            }
            (FEATURE, VOLUME, 0, CUR, true) => {
                Ok(Outcome::Data(self.volume.to_le_bytes().to_vec()))
            }
            (FEATURE, VOLUME, 0, RANGE, true) => Ok(Outcome::Data(range(
                2,
                i32::from(VOLUME_MIN),
                i32::from(VOLUME_MAX),
                i32::from(VOLUME_STEP),
            ))),
            (FEATURE, VOLUME, 0, CUR, false) if data.len() == 2 => {
                let asked = i16::from_le_bytes([data[0], data[1]]).clamp(VOLUME_MIN, VOLUME_MAX);
                // Round down to a step: toward the minimum.
                let steps = (i32::from(VOLUME_MAX) - i32::from(asked) + i32::from(VOLUME_STEP) - 1)
                    / i32::from(VOLUME_STEP);
                self.volume = (i32::from(VOLUME_MAX) - steps * i32::from(VOLUME_STEP)) as i16;
                self.gain = gain(self.volume);
                Ok(Outcome::Done)
            }
            _ => Err(Stall),
        }
    }
}

impl Requests for State {
    fn configure(&mut self, _value: u8) -> Result<(), Stall> {
        self.alternate = 0;
        self.ring.clear();
        self.fresh = 0;
        self.due = false;
        Ok(())
    }

    fn request(&mut self, setup: Setup, data: &[u8]) -> Result<Outcome, Stall> {
        let read = setup.device_to_host();
        match (setup.kind(), setup.recipient(), setup.request, read) {
            (CLASS, RECIPIENT_INTERFACE, ..) if setup.index & 0xff == CONTROL_INTERFACE => {
                self.entity(setup, data)
            }
            (STANDARD, RECIPIENT_INTERFACE, GET_STATUS, true)
                if setup.index <= STREAM_INTERFACE =>
            {
                Ok(Outcome::Data(vec![0, 0]))
            }
            (STANDARD, RECIPIENT_INTERFACE, GET_INTERFACE, true) => match setup.index {
                CONTROL_INTERFACE => Ok(Outcome::Data(vec![0])),
                STREAM_INTERFACE => Ok(Outcome::Data(vec![self.alternate])),
                _ => Err(Stall),
            },
            (STANDARD, RECIPIENT_INTERFACE, SET_INTERFACE, false) => {
                match (setup.index, setup.value) {
                    (CONTROL_INTERFACE, 0) => Ok(Outcome::Done),
                    (STREAM_INTERFACE, alternate @ 0..=2) => {
                        self.select(alternate as u8);
                        Ok(Outcome::Done)
                    }
                    _ => Err(Stall),
                }
            }
            // An isochronous endpoint has no halt; the requests succeed.
            (STANDARD, RECIPIENT_ENDPOINT, GET_STATUS, true)
                if setup.index == u16::from(STREAM) =>
            {
                Ok(Outcome::Data(vec![0, 0]))
            }
            (STANDARD, RECIPIENT_ENDPOINT, CLEAR_FEATURE | SET_FEATURE, false)
                if setup.index == u16::from(STREAM) && setup.value == ENDPOINT_HALT =>
            {
                Ok(Outcome::Done)
            }
            _ => Err(Stall),
        }
    }
}

impl BusPeer for Audio {
    fn reset(&mut self) -> Result<()> {
        self.control.reset(&mut self.state)?;
        let _ = self.state.configure(0);
        Ok(())
    }

    fn packet(&mut self, endpoint: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
        if endpoint == 0 {
            return self.control.packet(token, data, &mut self.state);
        }
        Ok(match (endpoint, token, self.setting()) {
            (STREAM, Token::Out, Some((subslot, _))) => {
                self.state.receive(data, usize::from(subslot));
                Reply::Ack(data.len())
            }
            _ => Reply::Stall,
        })
    }
}

impl Function for Audio {
    fn set_time(&mut self, _now: u64) {}

    fn ready_at(&self, _endpoint: u8) -> Option<u64> {
        None
    }
}

#[cfg(test)]
mod tests;
