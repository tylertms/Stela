//! The continuous output sink on the in-memory UAC2 peer.
use super::*;
use crate::usb::audio::sink::{
    Counters, HIGH_FRAMES, MAX_START_FAILURES, Note, REFILL_FRAMES, Sink, encode,
};

fn sink(dma: &mut TestRegion) -> Sink<Model> {
    Sink::new(AudioDevice::new(Model::new(), binding(), dma).unwrap()).unwrap()
}

#[test]
fn frames_are_encoded_in_the_formats_subslot_and_channels() {
    let mut format = binding().formats[0].clone();
    let frame = [0x1234_5678, -0x0102_0304];
    let mut out = Vec::new();
    encode(frame, &format, &mut out);
    assert_eq!(out, [0x34, 0x12, 0xfd, 0xfe]);
    (format.subslot, format.bits) = (3, 24);
    out.clear();
    encode(frame, &format, &mut out);
    assert_eq!(out, [0x56, 0x34, 0x12, 0xfc, 0xfd, 0xfe]);
    // 24 bits in 4-byte subslots: the low byte cleared.
    (format.subslot, format.bits, format.channels) = (4, 24, 4);
    out.clear();
    encode(frame, &format, &mut out);
    assert_eq!(
        out,
        [
            0, 0x56, 0x34, 0x12, 0, 0xfc, 0xfd, 0xfe, 0, 0, 0, 0, 0, 0, 0, 0
        ]
    );
    (format.subslot, format.bits, format.channels) = (2, 16, 1);
    out.clear();
    encode([0x4000_0000, 0x2000_0000], &format, &mut out);
    assert_eq!(out, [0x00, 0x30]);
}

#[test]
fn a_stream_starts_with_silence_plays_frames_and_stops_after_its_source() {
    let mut dma = TestRegion::new(80 * 1024 * 8);
    let mut s = sink(&mut dma);
    s.push(&[], false);
    assert!(!s.running());
    // The device plays nothing while held, then everything at once.
    s.device_mut().transport.hold = true;
    let frames: Vec<[i32; 2]> = (0..96).map(|n| [n << 16, -(n << 16)]).collect();
    s.push(&frames, true);
    assert_eq!(
        s.take_notes(),
        [Note::Started {
            interface: 1,
            alternate: 1,
            channels: 2,
            bits: 16
        }]
    );
    s.device_mut().transport.hold = false;
    s.push(&[], true);
    let output = &s.device_mut().transport.output;
    // The prefill's silence, then the frames in order.
    let start = REFILL_FRAMES as usize * 4;
    assert!(output[..start].iter().all(|b| *b == 0));
    for (n, chunk) in output[start..start + 96 * 4].chunks(4).enumerate() {
        let n = n as i16;
        assert_eq!(chunk, [n.to_le_bytes(), (-n).to_le_bytes()].concat());
    }
    // Everything played at once is a gap no queue covers: the stream
    // underran, counted, and the next frames start it again.
    let counters = s.counters();
    assert_eq!((counters.played, counters.dropped), (96, 0));
    assert_eq!(counters.underruns, 1);
    assert!(matches!(
        s.take_notes()[..],
        [Note::Interrupted(Error::Underrun)]
    ));
    assert!(!s.running());
    s.device_mut().transport.hold = true;
    s.push(&frames, true);
    assert!(s.running());
    // The source stops; once what is held has played, so does the stream.
    s.push(&[], false);
    assert!(s.running());
    s.device_mut().transport.hold = false;
    s.push(&[], false);
    assert!(!s.running());
    assert_eq!(s.take_notes().last(), Some(&Note::Stopped));
    assert_eq!(s.counters().underruns, 1);
    assert!(s.device_mut().transport.pending.is_empty());
}

#[test]
fn frames_past_the_high_mark_are_dropped() {
    let mut dma = TestRegion::new(80 * 1024 * 8);
    let mut s = sink(&mut dma);
    s.device_mut().transport.hold = true;
    // 200 ms at once while nothing plays.
    let frames = vec![[1 << 16, 1 << 16]; 9600];
    s.push(&frames, true);
    let counters = s.counters();
    assert_eq!(counters.played, HIGH_FRAMES - REFILL_FRAMES);
    assert_eq!(counters.dropped, 9600 - counters.played);
    s.push(&frames[..48], true);
    assert_eq!(s.counters().dropped, counters.dropped + 48);
}

#[test]
fn an_interrupted_stream_restarts_and_a_poisoned_owner_ends_the_sink() {
    let mut dma = TestRegion::new(80 * 1024 * 8);
    let mut s = sink(&mut dma);
    s.device_mut().transport.hold = true;
    s.push(&[[0, 0]; 48], true);
    s.take_notes();
    s.device_mut().transport.hold = false;
    s.device_mut().transport.transport_fault = true;
    s.push(&[[0, 0]; 48], true);
    // Each poll that fails stops the stream; frames start it again.
    let notes = s.take_notes();
    assert!(
        matches!(notes[0], Note::Interrupted(Error::Transport(_))),
        "{notes:?}"
    );
    assert!(!s.failed());
    assert!(s.counters().faults >= 1);
    s.device_mut().transport.transport_fault = false;
    s.device_mut().transport.hold = true;
    s.push(&[[0, 0]; 48], true);
    assert!(s.running());
    s.device_mut().transport.hold = false;
    // A failed cancel poisons the owner: the sink ends and drops frames.
    s.device_mut().transport.transport_fault = true;
    s.device_mut().transport.cancel_fail = true;
    s.push(&[[0, 0]; 48], true);
    assert!(s.failed());
    assert!(matches!(s.take_notes().last(), Some(Note::Failed(_))));
    let before = s.counters();
    s.push(&[[0, 0]; 10], true);
    assert_eq!(
        s.counters(),
        Counters {
            dropped: before.dropped + 10,
            ..before
        }
    );
}

#[test]
fn a_rate_no_format_takes_fails_after_repeated_starts() {
    let mut dma = TestRegion::new(80 * 1024 * 8);
    let mut s = sink(&mut dma);
    // The clock offers 8 kHz to 16 kHz only.
    let mut range = vec![1, 0];
    for value in [8000u32, 16_000, 8000] {
        range.extend_from_slice(&value.to_le_bytes());
    }
    let model = &mut s.device_mut().transport;
    model
        .overrides
        .insert(setup(0xa1, 2, 0x100, 0xa00, 2), vec![1, 0]);
    model
        .overrides
        .insert(setup(0xa1, 2, 0x100, 0xa00, 14), range);
    for _ in 0..MAX_START_FAILURES {
        s.push(&[[0, 0]; 48], true);
    }
    assert!(s.failed());
    assert_eq!(s.take_notes(), [Note::Failed(Error::RateUnsupported)]);
    assert_eq!(s.counters().dropped, 48 * u64::from(MAX_START_FAILURES));
}
