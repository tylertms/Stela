use super::*;
use crate::{
    dwc3::xhci::tests::host,
    usb::{
        Speed,
        control::{
            GET_DESCRIPTOR, SET_ADDRESS, SET_CONFIGURATION,
            tests::{setup, transfer},
        },
        hub::{Hub, Ports},
    },
};
use stela_drivers::{audio::Direction, usb::audio::Sync as StreamSync};

fn configured() -> Audio {
    let mut a = Audio::new();
    transfer(&mut a, setup(0, SET_ADDRESS, 4, 0, 0), &[]).unwrap();
    transfer(&mut a, setup(0, SET_CONFIGURATION, 1, 0, 0), &[]).unwrap();
    a
}

/// A class request to entity `entity` of the control interface.
fn entity(request_type: u8, request: u8, selector: u8, entity: u8, length: u16) -> [u8; 8] {
    setup(
        request_type,
        request,
        u16::from(selector) << 8,
        u16::from(entity) << 8,
        length,
    )
}

fn stream(a: &mut Audio, alternate: u16) {
    transfer(a, setup(0x01, SET_INTERFACE, alternate, 1, 0), &[]).unwrap();
}

#[test]
fn named_speaker_publishes_the_physical_product_string() {
    let name = "FIFINE Headset";
    let mut audio = Audio::named(name.into());
    let descriptor = transfer(
        &mut audio,
        setup(0x80, GET_DESCRIPTOR, 0x0302, 0x0409, 255),
        &[],
    )
    .unwrap();
    assert_eq!(descriptor[1], 3);
    let text: Vec<_> = descriptor[2..]
        .chunks_exact(2)
        .map(|unit| u16::from_le_bytes([unit[0], unit[1]]))
        .collect();
    assert!(text.into_iter().eq(name.encode_utf16()));
}

#[test]
fn descriptors_describe_one_stereo_uac2_output_without_capture() {
    let mut a = configured();
    let config = transfer(&mut a, setup(0x80, GET_DESCRIPTOR, 0x200, 0, 512), &[]).unwrap();
    assert_eq!(
        usize::from(u16::from_le_bytes([config[2], config[3]])),
        config.len()
    );
    assert_eq!(config[4], 2, "control and streaming interfaces");
    // Every descriptor's length walks the configuration exactly.
    let mut at = 0;
    let mut terminals = Vec::new();
    while at < config.len() {
        let d = &config[at..at + usize::from(config[at])];
        if d[1] == INTERFACE && d[2] == 1 {
            // The streaming interface: its subtypes mean other things.
            break;
        }
        if d[1] == CS_INTERFACE && matches!(d[2], INPUT_TERMINAL | OUTPUT_TERMINAL) && d.len() >= 6
        {
            terminals.push((d[2], u16::from_le_bytes([d[4], d[5]])));
        }
        at += d.len();
    }
    let mut rest = at;
    while rest < config.len() {
        rest += usize::from(config[rest]);
    }
    assert_eq!(rest, config.len());
    // USB streaming in, speaker out: no microphone or line input.
    assert_eq!(
        terminals,
        [(INPUT_TERMINAL, 0x0101), (OUTPUT_TERMINAL, 0x0301)]
    );
    let other = transfer(&mut a, setup(0x80, GET_DESCRIPTOR, 0x700, 0, 512), &[]).unwrap();
    assert_eq!(other.len(), config.len());
}

#[test]
fn clock_and_feature_unit_requests_answer_as_uac2_defines() {
    let mut a = configured();
    assert_eq!(
        transfer(&mut a, entity(0xa1, CUR, SAM_FREQ, CLOCK, 4), &[]).unwrap(),
        48_000u32.to_le_bytes()
    );
    // A host reads the subrange count first, then the whole block.
    assert_eq!(
        transfer(&mut a, entity(0xa1, RANGE, SAM_FREQ, CLOCK, 2), &[]).unwrap(),
        [1, 0]
    );
    let block = transfer(&mut a, entity(0xa1, RANGE, SAM_FREQ, CLOCK, 14), &[]).unwrap();
    assert_eq!(&block[2..6], 48_000u32.to_le_bytes());
    assert_eq!(&block[6..10], 48_000u32.to_le_bytes());
    assert_eq!(
        transfer(&mut a, entity(0xa1, CUR, CLOCK_VALID, CLOCK, 1), &[]).unwrap(),
        [1]
    );
    transfer(
        &mut a,
        entity(0x21, CUR, SAM_FREQ, CLOCK, 4),
        &48_000u32.to_le_bytes(),
    )
    .unwrap();
    assert!(
        transfer(
            &mut a,
            entity(0x21, CUR, SAM_FREQ, CLOCK, 4),
            &44_100u32.to_le_bytes()
        )
        .is_err()
    );
    // Volume: range, clamping and rounding toward the minimum.
    assert_eq!(
        transfer(&mut a, entity(0xa1, RANGE, VOLUME, FEATURE, 8), &[]).unwrap(),
        [1, 0, 0x00, 0xc4, 0, 0, 0x80, 0]
    );
    for (asked, kept) in [
        (-6 * 256, -6 * 256),
        (-77, -128),
        (100, 0),
        (-30_000, -60 * 256),
    ] {
        transfer(
            &mut a,
            entity(0x21, CUR, VOLUME, FEATURE, 2),
            &i16::to_le_bytes(asked),
        )
        .unwrap();
        assert_eq!(
            transfer(&mut a, entity(0xa1, CUR, VOLUME, FEATURE, 2), &[]).unwrap(),
            i16::to_le_bytes(kept)
        );
    }
    transfer(&mut a, entity(0x21, CUR, MUTE, FEATURE, 1), &[1]).unwrap();
    assert_eq!(
        transfer(&mut a, entity(0xa1, CUR, MUTE, FEATURE, 1), &[]).unwrap(),
        [1]
    );
    // Per-channel controls, unknown entities and selectors STALL.
    assert!(transfer(&mut a, setup(0xa1, CUR, 0x0201, 0x0300, 2), &[]).is_err());
    assert!(transfer(&mut a, entity(0xa1, CUR, VOLUME, 9, 2), &[]).is_err());
    assert!(transfer(&mut a, entity(0xa1, CUR, 7, FEATURE, 2), &[]).is_err());
}

#[test]
fn streamed_samples_reach_the_ring_msb_aligned_with_gain_applied() {
    let mut a = configured();
    // No setting selected: the endpoint does not exist.
    assert_eq!(a.packet(STREAM, Token::Out, &mut [0; 4]), Ok(Reply::Stall));
    stream(&mut a, 1);
    assert!(a.streaming());
    let mut packet = [0x34, 0x12, 0xcc, 0xed, 0xff, 0x7f, 0x00, 0x80];
    assert_eq!(a.packet(STREAM, Token::Out, &mut packet), Ok(Reply::Ack(8)));
    let mut out = [[0; 2]; 4];
    assert_eq!(a.take(&mut out), 2);
    assert_eq!(out[0], [0x1234_0000, 0xedcc_0000u32 as i32]);
    assert_eq!(out[1], [0x7fff_0000, i32::MIN]);
    stream(&mut a, 2);
    let mut packet = [0x56, 0x34, 0x12, 0x00, 0x00, 0x80, 0xff];
    a.packet(STREAM, Token::Out, &mut packet).unwrap();
    assert_eq!(a.take(&mut out), 1);
    assert_eq!(out[0], [0x1234_5600, i32::MIN]);
    assert_eq!(a.status().counters.split, 1);
    // -6 dB halves the amplitude to within the step table's rounding.
    transfer(
        &mut a,
        entity(0x21, CUR, VOLUME, FEATURE, 2),
        &i16::to_le_bytes(-6 * 256),
    )
    .unwrap();
    let mut packet = [0, 0, 0x40, 0, 0, 0x40];
    a.packet(STREAM, Token::Out, &mut packet).unwrap();
    a.take(&mut out);
    let half = f64::from(out[0][0]) / f64::from(0x4000_0000);
    assert!((half - 0.501).abs() < 0.001, "{half}");
    transfer(&mut a, entity(0x21, CUR, MUTE, FEATURE, 1), &[1]).unwrap();
    a.packet(STREAM, Token::Out, &mut packet).unwrap();
    a.take(&mut out);
    assert_eq!(out[0], [0, 0]);
}

#[test]
fn a_full_ring_drops_its_oldest_frames_and_asks_for_service() {
    let mut a = configured();
    stream(&mut a, 1);
    let mut first = true;
    // 300 ms of 1 ms packets, frame n carrying n.
    for packet in 0..300u32 {
        let mut bytes = Vec::new();
        for n in packet * 48..packet * 48 + 48 {
            bytes.extend_from_slice(&(n as u16).to_le_bytes());
            bytes.extend_from_slice(&(n as u16).to_le_bytes());
        }
        a.packet(STREAM, Token::Out, &mut bytes).unwrap();
        if first {
            // One 1 ms packet is under the 5 ms that asks for service.
            assert!(!a.take_due());
            first = false;
        }
    }
    assert!(a.take_due());
    assert!(!a.take_due());
    assert_eq!(a.queued(), RING_FRAMES);
    let status = a.status();
    assert_eq!(status.counters.received, 300 * 48);
    assert_eq!(status.counters.overflowed, (300 * 48 - RING_FRAMES) as u64);
    let mut out = [[0; 2]; 1];
    a.take(&mut out);
    let oldest = (300 * 48 - RING_FRAMES) as u16;
    assert_eq!(out[0][0], i32::from(oldest as i16) << 16);
    // Leaving the setting keeps what is queued; reconfiguring drops it.
    stream(&mut a, 0);
    assert!(!a.streaming());
    assert_eq!(a.queued(), RING_FRAMES - 1);
    transfer(&mut a, setup(0, SET_CONFIGURATION, 1, 0, 0), &[]).unwrap();
    assert_eq!(a.queued(), 0);
}

struct One(Audio);
impl Ports for One {
    fn count(&self) -> usize {
        1
    }
    fn speed(&self, _: usize) -> Speed {
        Speed::High
    }
    fn function(&mut self, _: usize) -> &mut dyn Function {
        &mut self.0
    }
    fn function_ref(&self, _: usize) -> &dyn Function {
        &self.0
    }
}

/// The host's own UAC2 class driver binds the function behind the native
/// hub on the modeled xHCI and plays PCM through its isochronous endpoint:
/// the descriptors, the class requests and the isochronous OUT path.
#[test]
fn a_uac2_host_driver_plays_through_the_isochronous_endpoint() {
    let hub = Hub::new(One(Audio::new()), "HUB").unwrap();
    let mut bench = host::bench(hub);
    let mut found = host::run(&mut bench);
    assert!(found.failures.is_empty(), "{:?}", found.failures);
    assert_eq!(found.audio.len(), 1, "{:?}", found.unused);
    let mut audio = found.audio.remove(0);
    let formats: Vec<_> = audio
        .binding()
        .formats
        .iter()
        .map(|f| (f.direction, f.channels, f.subslot, f.bits, f.sync.clone()))
        .collect();
    assert_eq!(
        formats,
        [
            (Direction::Playback, 2, 2, 16, StreamSync::Adaptive),
            (Direction::Playback, 2, 3, 24, StreamSync::Adaptive),
        ]
    );
    for (index, subslot) in [(1usize, 3usize), (0, 2)] {
        // 100 ms of a ramp, each sample's subslot bytes distinct.
        let frames = 4800usize;
        let mut pcm = Vec::new();
        for n in 0..frames * 2 {
            let value = (n as u32).wrapping_mul(2_654_435_761);
            pcm.extend_from_slice(&value.to_le_bytes()[..subslot]);
        }
        audio.start_playback(index, RATE, &pcm, true).unwrap();
        assert_eq!(audio.buffered_playback_frames(), frames);
        let mut drained = false;
        for _ in 0..20_000 {
            if audio.poll().unwrap().playback_drained {
                drained = true;
                break;
            }
        }
        assert!(drained, "{:?}", audio.progress());
        assert_eq!(audio.buffered_playback_frames(), 0);
        let model = &mut bench.model.borrow_mut().function.ports.0;
        // A finite stream ends by selecting the zero-bandwidth setting.
        assert!(!model.streaming());
        let mut out = vec![[0i32; 2]; frames + 1];
        assert_eq!(model.take(&mut out), frames);
        for (n, frame) in out[..frames].iter().enumerate() {
            for (c, sample) in frame.iter().enumerate() {
                let value = ((n * 2 + c) as u32).wrapping_mul(2_654_435_761);
                let mut word = [0u8; 4];
                word[4 - subslot..].copy_from_slice(&value.to_le_bytes()[..subslot]);
                assert_eq!(*sample, i32::from_le_bytes(word), "frame {n}");
            }
        }
        assert_eq!(model.status().counters.overflowed, 0);
    }
    // The host's mute and volume reach the feature unit.
    audio.set_mute(FEATURE, 0, true).unwrap();
    audio.set_volume(FEATURE, 0, -12 * 256).unwrap();
    let status = bench.model.borrow().function.ports.0.status();
    assert!(status.muted);
    assert_eq!(status.volume, -12 * 256);
}
