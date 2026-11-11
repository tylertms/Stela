// SPDX-License-Identifier: GPL-2.0-or-later
//! Representative standard UAC2 descriptors, not a dump of the physical PC.
use super::*;
mod badd;
mod mixer;
mod sink;
mod uac1;
mod uac3;
use crate::usb::audio::*;
use crate::{
    bus::dma::TestRegion,
    usb::descriptor::{interfaces, setup},
};
use alloc::{collections::BTreeMap, vec::Vec};

fn append(v: &mut Vec<u8>, d: &[u8]) {
    v.extend_from_slice(d);
}
pub(crate) fn configuration() -> Vec<u8> {
    let mut v = vec![9, 2, 0, 0, 3, 1, 0, 0x80, 50];
    append(&mut v, &[8, 11, 0, 3, 1, 0, 0x20, 0]);
    append(&mut v, &[9, 4, 0, 0, 0, 1, 1, 0x20, 0]);
    let start = v.len();
    append(&mut v, &[9, 0x24, 1, 0, 2, 1, 0, 0, 0]);
    append(&mut v, &[8, 0x24, 10, 10, 3, 7, 0, 0]); // rw frequency + ro validity
    append(
        &mut v,
        &[17, 0x24, 2, 1, 1, 1, 0, 10, 2, 3, 0, 0, 0, 0, 0, 0, 0],
    );
    append(
        &mut v,
        &[18, 0x24, 6, 2, 1, 15, 0, 0, 0, 15, 0, 0, 0, 15, 0, 0, 0, 0],
    );
    append(&mut v, &[12, 0x24, 3, 3, 1, 3, 0, 2, 10, 0, 0, 0]);
    append(
        &mut v,
        &[17, 0x24, 2, 4, 1, 2, 0, 10, 2, 3, 0, 0, 0, 0, 0, 0, 0],
    );
    append(&mut v, &[12, 0x24, 3, 5, 1, 1, 0, 4, 10, 0, 0, 0]);
    let total = (v.len() - start) as u16;
    v[start + 6..start + 8].copy_from_slice(&total.to_le_bytes());
    for (i, terminal, address, attrs) in [(1, 1, 1, 0x0d), (2, 5, 0x82, 0x05)] {
        append(&mut v, &[9, 4, i, 0, 0, 1, 2, 0x20, 0]);
        append(&mut v, &[9, 4, i, 1, 1, 1, 2, 0x20, 0]);
        append(
            &mut v,
            &[16, 0x24, 1, terminal, 0, 1, 1, 0, 0, 0, 2, 3, 0, 0, 0, 0],
        );
        append(&mut v, &[6, 0x24, 2, 1, 2, 16]);
        append(&mut v, &[7, 5, address, attrs, 0xc0, 0, 4]);
        append(&mut v, &[8, 0x25, 1, 0, 0, 0, 0, 0]);
    }
    let total = v.len() as u16;
    v[2..4].copy_from_slice(&total.to_le_bytes());
    v
}
fn binding() -> Binding {
    let c = configuration();
    let all = interfaces(&c);
    Binding::parse(&all[0], &all, &c, 3).unwrap()
}

#[test]
fn runtime_descriptors_select_graph_and_pcm_not_product_nodes() {
    let b = binding();
    assert_eq!(b.formats.len(), 2);
    assert_eq!(b.formats[0].frame_bytes(), 4);
    assert_eq!(b.graph.terminal_clock(5, Direction::Capture), Ok(10));
    assert_eq!(b.settings(), vec![(0, 0), (1, 0), (2, 0)]);
    let mut c = configuration();
    c.pop();
    let a = interfaces(&c);
    assert!(Binding::parse(&a[0], &a, &c, 3).is_err());
    let mut c = configuration();
    let n = c
        .windows(8)
        .position(|w| w == [8, 0x24, 10, 10, 3, 7, 0, 0])
        .unwrap();
    c[n + 3] = 1;
    let a = interfaces(&c);
    assert!(Binding::parse(&a[0], &a, &c, 3).is_err());
}

#[derive(Default)]
struct Model {
    overrides: BTreeMap<[u8; 8], Vec<u8>>,
    selection: u8,
    rate: u32,
    time: u64,
    valid: bool,
    muted: bool,
    volume: i16,
    active: Vec<u8>,
    pending: BTreeMap<u64, (Endpoint, u64, usize)>,
    next: u64,
    output: Vec<u8>,
    submitted: Vec<(u8, usize)>,
    alternates: Vec<(u8, u8)>,
    pitches: Vec<[u8; 8]>,
    cancel_fail: bool,
    packet_fault: Option<PacketStatus>,
    transport_fault: bool,
    hold: bool,
}
impl Model {
    fn new() -> Self {
        Self {
            selection: 1,
            rate: 48_000,
            valid: true,
            ..Self::default()
        }
    }
}
impl Control for Model {
    fn now_us(&self) -> u64 {
        self.time
    }
    fn relax(&mut self) {
        self.time += 1000;
    }
    fn control(&mut self, r: [u8; 8], out: &[u8]) -> Result<Vec<u8>, BlockError> {
        if let Some(reply) = self.overrides.get(&r) {
            return Ok(reply.clone());
        }
        let entity = r[5];
        let sel = r[3];
        let len = u16::from_le_bytes([r[6], r[7]]) as usize;
        if r[0] == 1 && r[1] == 11 {
            self.alternates.push((r[4], r[2]));
            return Ok(Vec::new());
        }
        if r[0] == 0x22 {
            assert_eq!(out, [1]);
            self.pitches.push(r);
            return Ok(Vec::new());
        }
        assert_eq!(r[4], 0);
        assert_eq!(r[2], 0);
        if r[0] == 0x21 {
            match (entity, sel) {
                (11 | 14 | 16, 1) => self.selection = out[0],
                (10, 1) => self.rate = u32::from_le_bytes(out.try_into().unwrap()),
                (2, 1) => self.muted = out[0] != 0,
                (2, 2) => self.volume = i16::from_le_bytes(out.try_into().unwrap()),
                _ => panic!("unexpected set {r:?}"),
            };
            return Ok(Vec::new());
        }
        assert_eq!(r[0], 0xa1);
        let data = match (entity, sel, r[1]) {
            (11 | 14 | 16, 1, 1) => vec![self.selection],
            (10, 1, 1) => self.rate.to_le_bytes().to_vec(),
            (10, 2, 1) => vec![u8::from(self.valid)],
            (10, 1, 2) => {
                let mut v = 1u16.to_le_bytes().to_vec();
                v.extend_from_slice(&44_100u32.to_le_bytes());
                v.extend_from_slice(&96_000u32.to_le_bytes());
                v.extend_from_slice(&100u32.to_le_bytes());
                v
            }
            (2, 1, 1) => vec![u8::from(self.muted)],
            (2, 2, 1) => self.volume.to_le_bytes().to_vec(),
            (2, 2, 2) => vec![1, 0, 0, 0xc0, 0, 0, 0, 1],
            _ => panic!("unexpected get {r:?}"),
        };
        Ok(data[..len.min(data.len())].to_vec())
    }
}
impl Transport for Model {
    // Independent in-memory device peer; it owns no native attachment.
    fn validate(&self) -> Result<(), BlockError> {
        Ok(())
    }
    type Ticket = u64;
    fn preflight(&mut self, eps: &[Endpoint]) -> Result<(), BlockError> {
        if eps.iter().any(|e| self.active.contains(&e.address)) {
            return Err(BlockError::Unsupported("foreign endpoint owner"));
        }
        Ok(())
    }
    fn configure(&mut self, _: Direction, eps: &[Endpoint]) -> Result<(), BlockError> {
        for e in eps {
            assert!(!self.active.contains(&e.address));
            self.active.push(e.address);
        }
        Ok(())
    }
    fn deconfigure(&mut self, _: Direction, eps: &[Endpoint]) -> Result<(), BlockError> {
        if self.cancel_fail {
            return Err(BlockError::Timeout);
        }
        for e in eps {
            self.active.retain(|a| *a != e.address);
            self.pending.retain(|_, (ep, _, _)| ep.address != e.address);
        }
        Ok(())
    }
    fn submit(&mut self, e: &Endpoint, p: u64, n: usize) -> Result<u64, BlockError> {
        assert!(self.active.contains(&e.address));
        assert!(n <= 3072);
        assert_eq!(p % 64, 0);
        self.submitted.push((e.address, n));
        self.next += 1;
        self.pending.insert(self.next, (*e, p, n));
        Ok(self.next)
    }
    fn completed(&mut self, t: &u64) -> Option<Result<Completion, BlockError>> {
        if self.hold {
            return None;
        }
        if self.transport_fault {
            return Some(Err(BlockError::Timeout));
        }
        let (e, p, n) = self.pending.remove(t).expect("ticket consumed once");
        // SAFETY: the class's live DMA buffer remains owned by this ticket until this terminal completion.
        let data = unsafe { core::slice::from_raw_parts_mut(p as *mut u8, n) };
        if e.attributes & 0x30 == 0x10 {
            data[..4].copy_from_slice(&(6u32 << 16).to_le_bytes());
        } else if e.is_in() {
            data.fill(*t as u8);
        } else {
            self.output.extend_from_slice(data);
        }
        Some(Ok(Completion {
            bytes: n,
            status: self.packet_fault.take().unwrap_or(PacketStatus::Success),
        }))
    }
}
fn run(f: impl FnOnce(&mut AudioDevice<Model>)) {
    let mut dma = TestRegion::new(80 * 1024 * 8);
    let mut a = AudioDevice::new(Model::new(), binding(), &mut dma).unwrap();
    f(&mut a);
}
const CURRENT_RATE: [u8; 8] = [0xa1, 1, 0, 1, 0, 10, 4, 0];
const SET_RATE: [u8; 8] = [0x21, 1, 0, 1, 0, 10, 4, 0];
const VALIDITY: [u8; 8] = [0xa1, 1, 0, 2, 0, 10, 1, 0];
/// Answer both halves of the clock-10 range read with these triplets.
fn ranges(m: &mut Model, triplets: &[[u32; 3]]) {
    let mut reply = (triplets.len() as u16).to_le_bytes().to_vec();
    for value in triplets.iter().flatten() {
        reply.extend_from_slice(&value.to_le_bytes());
    }
    m.overrides
        .insert(setup(0xa1, 2, 0x100, 0xa00, 2), reply[..2].to_vec());
    m.overrides
        .insert(setup(0xa1, 2, 0x100, 0xa00, reply.len() as u16), reply);
}
fn set_clock_controls(graph: &mut Graph, id: u8, bits: u8) {
    for entity in &mut graph.entities {
        if let Entity::ClockSource {
            id: source,
            controls,
            ..
        } = entity
            && *source == id
        {
            *controls = bits;
        }
    }
}
#[test]
fn rates_are_set_and_validated_as_linux_set_sample_rate_v2v3_does() {
    let mut g = binding().graph;
    let mut m = Model::new();
    let c = g.resolve_clock(&mut m, 10).unwrap();
    assert_eq!(
        c,
        ResolvedClock {
            terminal: 10,
            source: 10
        }
    );
    let Rates::Table(table) = g.rates(&mut m, c).unwrap() else {
        panic!("discrete ranges announce a table");
    };
    assert_eq!((table.len(), table[0], table[519]), (520, 44_100, 96_000));
    assert!(!g.rates(&mut m, c).unwrap().contains(44_101));
    assert_eq!(g.set_rate(&mut m, c, 44_100), Ok(44_100));
    assert_eq!(m.rate, 44_100);
    g.set_mute(&mut m, 2, 0, true).unwrap();
    let range = g.volume_range(&mut m, 2, 0).unwrap();
    assert_eq!(
        range,
        MixerRange {
            min: -16384,
            max: 0,
            res: 256,
            db: Some((-6400, 0))
        }
    );
    assert_eq!(m.volume, 0);
    g.set_volume(&mut m, 2, 0, range, -256).unwrap();
    assert_eq!(m.volume, -256);
    // get_abs_value: the step at or below, clamped to the range.
    g.set_volume(&mut m, 2, 0, range, -257).unwrap();
    assert_eq!(m.volume, -512);
    g.set_volume(&mut m, 2, 0, range, i16::MIN).unwrap();
    assert_eq!(m.volume, -16384);
    assert_eq!(setup(0xa1, 1, 0x100, 0xa00, 4), CURRENT_RATE);

    // A clock that keeps running elsewhere is reported, not refused.
    m.overrides
        .insert(CURRENT_RATE, 48_000u32.to_le_bytes().to_vec());
    assert_eq!(g.set_rate(&mut m, c, 44_100), Ok(48_000));
    // An unreadable frequency counts as 0 and the previous rate stands.
    m.overrides.insert(CURRENT_RATE, vec![0; 3]);
    assert_eq!(g.current_rate(&mut m, c), 0);
    assert_eq!(g.set_rate(&mut m, c, 48_000), Ok(0));
    m.overrides.clear();
    // A refused write fails the change.
    m.overrides.insert(SET_RATE, vec![0]);
    assert_eq!(g.set_rate(&mut m, c, 96_000), Err(Error::ControlLength));
    m.overrides.clear();
    // A read-only frequency is left as it runs.
    set_clock_controls(&mut g, 10, 0b0101);
    assert_eq!(g.set_rate(&mut m, c, 96_000), Ok(48_000));
    assert_eq!(m.rate, 48_000);
    set_clock_controls(&mut g, 10, 0b0111);

    // Validity is one read: no settling wait.
    m.valid = false;
    assert_eq!(g.valid_clock(&mut m, c), Err(Error::ClockInvalid));
    assert_eq!(g.set_rate(&mut m, c, 44_100), Err(Error::ClockInvalid));
    assert_eq!(m.time, 0);
    // The UAC2 quirk: an internal single-rate source wired to the terminal.
    ranges(&mut m, &[[48_000, 48_000, 0]]);
    assert_eq!(g.rates(&mut m, c), Ok(Rates::Table(vec![48_000])));
    assert_eq!(g.valid_clock(&mut m, c), Ok(()));
    m.overrides.clear();
    // Without a readable validity control the clock is assumed valid.
    set_clock_controls(&mut g, 10, 0b0011);
    assert_eq!(g.valid_clock(&mut m, c), Ok(()));
    set_clock_controls(&mut g, 10, 0b0111);
    m.valid = true;
    m.overrides.insert(VALIDITY, vec![]);
    assert_eq!(g.valid_clock(&mut m, c), Err(Error::ClockInvalid));
}
#[test]
fn sample_rate_ranges_parse_like_linux() {
    let g = binding().graph;
    let mut m = Model::new();
    let c = g.resolve_clock(&mut m, 10).unwrap();
    // Negative as a signed int, then max below min: both skipped.
    ranges(
        &mut m,
        &[
            [0x8000_0000, 0x8000_0000, 0],
            [48_000, 44_100, 0],
            [8_000, 192_000, 1],
        ],
    );
    assert_eq!(
        g.rates(&mut m, c),
        Ok(Rates::Continuous {
            min: 8_000,
            max: 192_000
        })
    );
    assert!(g.rates(&mut m, c).unwrap().contains(12_345));
    // Linux stops at MAX_NR_RATES.
    ranges(&mut m, &[[1, 4_000, 2]]);
    let Rates::Table(table) = g.rates(&mut m, c).unwrap() else {
        panic!("resolution 2 is discrete");
    };
    assert_eq!((table.len(), table[1023]), (1024, 2_047));
}
#[test]
fn a_running_rate_mismatch_is_recorded_and_streaming_continues() {
    run(|a| {
        a.transport
            .overrides
            .insert(CURRENT_RATE, 44_100u32.to_le_bytes().to_vec());
        a.start_capture(1, 48000).unwrap();
        assert_eq!(a.clock_mismatch, Some((48_000, 44_100)));
        a.poll().unwrap();
        a.stop_capture().unwrap();
        a.transport.overrides.clear();
        a.start_capture(1, 48000).unwrap();
        assert_eq!(a.clock_mismatch, None);
        a.stop_capture().unwrap();
        assert_eq!(
            a.start_playback(0, 47_950, &[0; 24], true),
            Err(Error::RateUnsupported)
        );
        a.shutdown().unwrap();
    });
}
#[test]
fn finite_pcm_is_exact_across_many_packet_ring_wraps_and_restart() {
    run(|a| {
        let pcm: Vec<u8> = (0..65536).map(|i| (i * 17) as u8).collect();
        a.start_playback(0, 48000, &pcm, true).unwrap();
        for _ in 0..200 {
            if a.poll().unwrap().playback_drained {
                break;
            }
        }
        assert!(a.progress().playback_drained);
        assert_eq!(a.transport.output, pcm);
        assert!(a.transport.pending.is_empty());
        assert_eq!(a.transport.alternates.last(), Some(&(1, 0)));
        a.start_playback(0, 48000, &pcm[..24], true).unwrap();
        a.poll().unwrap();
        assert_eq!(a.progress().playback_frames, 16390);
    });
}
#[test]
fn duplex_capture_uses_packet_bytes_and_shared_clock_is_not_retuned() {
    run(|a| {
        a.start_capture(1, 48000).unwrap();
        a.start_playback(0, 48000, &vec![7; 24000], true).unwrap();
        a.poll().unwrap();
        assert_eq!(a.progress().capture_frames, 3072);
        let mut pcm = vec![0; 6144];
        assert_eq!(a.read_capture(&mut pcm).unwrap(), 6144);
        assert_eq!(&pcm[..192], &[1; 192]);
        assert_eq!(&pcm[192..384], &[2; 192]);
        assert_eq!(a.start_capture(1, 44100), Err(Error::Busy));
        a.stop_playback().unwrap();
        assert!(a.capture.is_some());
        a.stop_capture().unwrap();
        assert!(a.transport.pending.is_empty());
    });
}
#[test]
fn packet_errors_lose_one_packet_and_failed_cancel_keeps_dma_quarantined() {
    run(|a| {
        a.start_capture(1, 48000).unwrap();
        a.transport.packet_fault = Some(PacketStatus::MissedService);
        a.poll().unwrap();
        assert_eq!(a.progress().packet_errors, 1);
        a.shutdown().unwrap();
        assert!(a.transport.pending.is_empty());
        a.start_capture(1, 48000).unwrap();
        a.transport.transport_fault = true;
        a.transport.cancel_fail = true;
        assert_eq!(a.poll(), Err(Error::Poisoned));
        assert!(!a.transport.pending.is_empty());
        assert_eq!(a.start_capture(1, 48000), Err(Error::Poisoned));
        a.transport.transport_fault = false;
        a.transport.cancel_fail = false;
        a.shutdown().unwrap();
        assert!(a.transport.pending.is_empty());
    });
}
#[test]
fn continuous_partial_packet_starvation_is_explicit_and_capture_fifo_wraps() {
    run(|a| {
        a.start_playback(0, 48000, &[1; 28], false).unwrap();
        assert_eq!(a.poll(), Err(Error::Underrun));
        a.start_capture(1, 48000).unwrap();
        for _ in 0..3 {
            a.poll().unwrap();
            let mut out = vec![0; 6144];
            assert_eq!(a.read_capture(&mut out).unwrap(), 6144);
            for (n, p) in out.chunks(192).enumerate() {
                assert!(p.iter().all(|v| *v == p[0]), "packet{n}");
            }
        }
        a.shutdown().unwrap();
    });
}

#[test]
fn explicit_feedback_updates_packetizer_and_stale_feedback_stops() {
    run(|a| {
        let fb = Endpoint {
            address: 0x83,
            attributes: 0x11,
            max_packet: 4,
            interval: 4,
            companion: None,
        };
        a.binding.formats[0].sync = Sync::Explicit(fb);
        a.start_playback(0, 48000, &vec![9; 24000], false).unwrap();
        assert_eq!(a.poll().unwrap().feedback_updates, 64);
        a.transport.hold = true;
        a.transport.time = 100001;
        assert_eq!(a.poll(), Err(Error::Feedback));
        assert!(a.transport.pending.is_empty());
    });
}
#[test]
fn implicit_feedback_uses_capture_packet_frames_and_keeps_requested_capture() {
    run(|a| {
        let c = a.binding.formats[1].clone();
        a.binding.formats[0].sync = Sync::Implicit {
            interface: c.interface,
            alternate: c.alternate,
            endpoint: c.data,
            frame_bytes: c.frame_bytes(),
        };
        a.start_playback(0, 48000, &vec![3; 12288], true).unwrap();
        assert_eq!(a.progress().queued_playback, 0);
        a.start_capture(1, 48000).unwrap();
        for _ in 0..4 {
            a.poll().unwrap();
        }
        assert!(a.progress().playback_drained);
        assert_eq!(a.transport.output, vec![3; 12288]);
        assert!(a.capture.as_ref().unwrap().requested);
        assert!(!a.capture.as_ref().unwrap().implicit);
        a.stop_capture().unwrap();
        assert!(a.transport.pending.is_empty());
    });
}

#[test]
fn clock_paths_are_found_like_linux_uac_clock_find_source() {
    let mut graph = binding().graph;
    graph.entities.extend([
        Entity::ClockSelector {
            id: 11,
            sources: vec![10],
            controls: 3,
        },
        Entity::ClockMultiplier {
            id: 12,
            source: 11,
            controls: 5,
        },
        Entity::Selector {
            id: 14,
            sources: vec![1, 4],
            controls: 3,
        },
        Entity::ClockSource {
            id: 15,
            attributes: 1,
            controls: 0b0011,
        },
        Entity::ClockSelector {
            id: 16,
            sources: vec![10, 15],
            controls: 3,
        },
    ]);
    let mut m = Model::new();
    graph.signal_selection(&mut m, 14, 2).unwrap();
    assert_eq!(m.selection, 2);
    // Multipliers pass through unread; a one-input selector is not read
    // but its pin is written.
    m.selection = 0;
    let c = graph.resolve_clock(&mut m, 12).unwrap();
    assert_eq!(
        c,
        ResolvedClock {
            terminal: 12,
            source: 10
        }
    );
    assert_eq!(m.selection, 1);
    assert_eq!(graph.set_rate(&mut m, c, 88_200), Ok(88_200));
    assert_eq!(m.rate, 88_200);
    // An invalid current input moves a writable selector to a valid one.
    m.valid = false;
    assert_eq!(graph.resolve_clock(&mut m, 16).unwrap().source, 15);
    assert_eq!(m.selection, 2);
    // An illegal selection is replaced by the first valid input.
    m.valid = true;
    m.selection = 7;
    assert_eq!(graph.resolve_clock(&mut m, 16).unwrap().source, 10);
    assert_eq!(m.selection, 1);
    // A read-only selector keeps its invalid input, unvalidated, for the
    // rate change to validate.
    if let Some(Entity::ClockSelector { controls, .. }) = graph
        .entities
        .iter_mut()
        .find(|entity| matches!(entity, Entity::ClockSelector { id: 16, .. }))
    {
        *controls = 1;
    }
    m.valid = false;
    let c = graph.resolve_clock(&mut m, 16).unwrap();
    assert_eq!(c.source, 10);
    assert_eq!(graph.set_rate(&mut m, c, 48_000), Err(Error::ClockInvalid));
    // A one-input selector's failed write is a firmware bug Linux
    // ignores; another selector's leaves no usable path.
    m.valid = true;
    m.overrides.insert(setup(0xa1, 1, 0x100, 0xb00, 1), vec![]);
    assert_eq!(graph.resolve_clock(&mut m, 12).unwrap().source, 10);
    if let Some(Entity::ClockSelector { controls, .. }) = graph
        .entities
        .iter_mut()
        .find(|entity| matches!(entity, Entity::ClockSelector { id: 16, .. }))
    {
        *controls = 3;
    }
    m.overrides.insert(setup(0xa1, 1, 0x100, 0x1000, 1), vec![]);
    assert_eq!(graph.resolve_clock(&mut m, 16), Err(Error::ClockInvalid));
}
#[test]
fn raw_endpoint_counts_reserved_usage_and_implicit_association_are_checked() {
    let mut c = configuration();
    let n = c
        .windows(7)
        .position(|w| w == [7, 5, 1, 13, 192, 0, 4])
        .unwrap();
    c[n + 3] = 0x35;
    let all = interfaces(&c);
    assert!(Binding::parse(&all[0], &all, &c, 3).is_err());
    let mut c = configuration();
    let n = c
        .windows(9)
        .position(|w| w == [9, 4, 1, 1, 1, 1, 2, 32, 0])
        .unwrap();
    c[n + 4] = 2;
    let all = interfaces(&c);
    assert!(Binding::parse(&all[0], &all, &c, 3).is_err());
    let mut c = configuration();
    let n = c
        .windows(7)
        .position(|w| w == [7, 5, 1, 13, 192, 0, 4])
        .unwrap();
    c[n + 3] = 5;
    let n = c
        .windows(7)
        .position(|w| w == [7, 5, 0x82, 5, 192, 0, 4])
        .unwrap();
    c[n + 3] = 0x25;
    let all = interfaces(&c);
    let b = Binding::parse(&all[0], &all, &c, 3).unwrap();
    assert!(matches!(
        b.formats[0].sync,
        Sync::Implicit {
            interface: 2,
            alternate: 1,
            ..
        }
    ));
    c[n + 6] = 3;
    let all = interfaces(&c);
    let b = Binding::parse(&all[0], &all, &c, 3).unwrap();
    assert_eq!(b.formats.len(), 1);
    assert!(!b.unsupported.is_empty());
}

#[test]
fn missing_capture_or_synchronous_output_completions_fail_and_cancel() {
    run(|a| {
        a.start_capture(1, 48000).unwrap();
        a.transport.hold = true;
        a.transport.time = 1_000_001;
        assert_eq!(
            a.poll(),
            Err(Error::Packet("USB audio capture made no progress"))
        );
        assert!(a.transport.pending.is_empty());
        a.transport.time = 0;
        a.transport.hold = false;
        a.start_playback(0, 48000, &vec![0; 6144], true).unwrap();
        a.transport.hold = true;
        a.transport.time = 1_000_001;
        assert_eq!(
            a.poll(),
            Err(Error::Packet("USB audio playback made no progress"))
        );
        assert!(a.transport.pending.is_empty());
    });
}

#[test]
fn foreign_endpoint_refusal_never_stages_cleanup_or_changes_alternate() {
    run(|a| {
        a.transport.active.push(1);
        let before = a.transport.alternates.clone();
        assert_eq!(
            a.start_playback(0, 48000, &[0; 192], true),
            Err(Error::Transport(BlockError::Unsupported(
                "foreign endpoint owner"
            )))
        );
        assert_eq!(a.transport.active, vec![1]);
        assert_eq!(a.transport.alternates, before);
        assert!(a.playback.is_none());
        assert!(a.output.endpoint.is_none());
    });
}
#[test]
fn explicit_feedback_cannot_alias_active_capture_in_either_start_order() {
    run(|a| {
        let mut fb = a.binding.formats[1].data;
        fb.attributes = 0x11;
        fb.max_packet = 4;
        a.binding.formats[0].sync = Sync::Explicit(fb);
        a.start_capture(1, 48000).unwrap();
        assert_eq!(
            a.start_playback(0, 48000, &[0; 192], true),
            Err(Error::Busy)
        );
        assert!(a.capture.is_some());
        a.stop_capture().unwrap();
        a.start_playback(0, 48000, &[0; 192], true).unwrap();
        assert_eq!(a.start_capture(1, 48000), Err(Error::Busy));
        assert!(a.playback.is_some());
        a.stop_playback().unwrap();
    });
}
#[test]
fn descriptor_audio_alternate_cannot_claim_foreign_class_dci() {
    let mut c = configuration();
    c[4] = 4;
    c.extend_from_slice(&[9, 4, 3, 0, 1, 3, 1, 1, 0, 7, 5, 1, 3, 8, 0, 1]);
    let n = c.len() as u16;
    c[2..4].copy_from_slice(&n.to_le_bytes());
    let all = interfaces(&c);
    assert_eq!(
        Binding::parse(&all[0], &all, &c, 3),
        Err(Error::Descriptor(
            "audio alternate aliases a foreign interface endpoint"
        ))
    );
}

#[test]
fn mixer_values_are_restored_like_snd_usb_mixer_resume() {
    let mut state = None;
    run(|a| {
        a.set_mute(2, 0, true).unwrap();
        // The step at or below is what is cached and restored.
        a.set_volume(2, 0, -300).unwrap();
        assert_eq!(a.transport.volume, -512);
        state = Some(a.mixer_state());
    });
    // The fresh owner resume publishes starts from device defaults.
    run(|a| {
        let state = state.unwrap();
        a.restore_mixer(state.clone()).unwrap();
        assert!(a.transport.muted);
        assert_eq!(a.transport.volume, -512);
        // The old ranges are adopted, not probed again.
        assert_eq!(a.mixer_state(), state);
        assert_eq!(a.volume_range(2, 0).unwrap().res, 256);
    });
}

#[test]
fn pitch_control_is_enabled_after_the_rate_like_snd_usb_init_pitch() {
    let mut c = configuration();
    let at = c.windows(3).position(|w| w == [8, 0x25, 1]).unwrap();
    c[at + 4] = 1;
    let all = interfaces(&c);
    let binding = Binding::parse(&all[0], &all, &c, 3).unwrap();
    assert_eq!(binding.formats[0].pitch, Some((0x0100, 0)));
    run(|a| {
        a.binding.formats[0].pitch = Some((0x0100, 0));
        a.start_playback(0, 48000, &[0; 192], true).unwrap();
        assert_eq!(a.transport.pitches, [[0x22, 1, 0, 1, 0, 0, 1, 0]]);
    });
}

#[test]
fn max_packets_only_refuses_short_finite_tail_until_producer_completes_packet() {
    run(|a| {
        a.binding.formats[0].fill_max = true;
        assert_eq!(
            a.start_playback(0, 48000, &[1; 24], true),
            Err(Error::Format)
        );
        assert!(a.transport.active.is_empty());
        assert!(a.transport.alternates.is_empty());
        let pcm = vec![1; 6168];
        a.start_playback(0, 48000, &pcm, false).unwrap();
        assert_eq!(a.finish_playback(), Err(Error::Format));
        assert!(!a.playback.as_ref().unwrap().finite);
        a.write_playback(&[2; 168]).unwrap();
        a.finish_playback().unwrap();
        for _ in 0..3 {
            a.poll().unwrap();
        }
        assert!(a.progress().playback_drained);
        let mut expected = pcm;
        expected.extend_from_slice(&[2; 168]);
        assert_eq!(a.transport.output, expected);
        assert_eq!(a.transport.output.len() % 192, 0);
    });
}

#[test]
fn fractional_low_rate_preserves_zero_length_service_intervals() {
    run(|a| {
        a.binding.formats[0].service_microframes = 1;
        a.binding.formats[0].data.interval = 1;
        let mut range = vec![1, 0];
        range.extend_from_slice(&4000u32.to_le_bytes());
        range.extend_from_slice(&4000u32.to_le_bytes());
        range.extend_from_slice(&0u32.to_le_bytes());
        a.transport
            .overrides
            .insert(setup(0xa1, 2, 0x100, 0xa00, 2), vec![1, 0]);
        a.transport
            .overrides
            .insert(setup(0xa1, 2, 0x100, 0xa00, 14), range);
        let pcm = [1, 2, 3, 4, 5, 6, 7, 8];
        a.start_playback(0, 4000, &pcm, true).unwrap();
        assert_eq!(a.transport.submitted, vec![(1, 0), (1, 4), (1, 0), (1, 4)]);
        assert_eq!(a.progress().queued_playback, 4);
        a.poll().unwrap();
        assert!(a.progress().playback_drained);
        assert_eq!(a.progress().playback_frames, 2);
        assert_eq!(a.transport.output, pcm);
    });
}

/// A lane's queue covers a 64 ms service gap where the transfer and event
/// rings allow it: 64 packets at 1 ms, the 192-packet cap at 125 us, the
/// pool's size for large packets; no slot crosses a 64 KiB boundary.
#[test]
fn lanes_queue_enough_service_intervals_for_a_late_poll() {
    let mut dma = TestRegion::new(512 * 1024);
    let mut lane: Lane<u64> = Lane::new(&mut dma, POOL_BYTES).unwrap();
    for (bytes, microframes, packets) in [(196, 8, 64), (168, 1, 192), (4096, 8, 32), (3072, 1, 42)]
    {
        lane.arm(bytes, microframes).unwrap();
        assert_eq!(lane.packets.len(), packets, "{bytes} every {microframes}");
        for packet in &lane.packets {
            let start = lane.pool.physical() + packet.offset as u64;
            assert_eq!(start % 64, 0);
            assert_eq!(
                start / TD_BOUNDARY,
                (start + lane.slot as u64 - 1) / TD_BOUNDARY
            );
            assert!(packet.offset + lane.slot <= lane.pool.len());
        }
    }
    assert_eq!(lane.arm(POOL_BYTES + 1, 8), Err(Error::Format));
}

/// A continuous stream at 1 ms intervals has 64 ms queued, so a 50 ms
/// pause in polling does not starve it.
#[test]
fn a_continuous_stream_survives_a_fifty_millisecond_poll_gap() {
    run(|a| {
        // 200 ms of 2-channel 16-bit PCM.
        a.start_playback(0, 48000, &vec![5; 38400], false).unwrap();
        assert_eq!(a.progress().queued_playback, 64);
        a.transport.time += 50_000;
        let progress = a.poll().unwrap();
        assert_eq!(progress.playback_frames, 64 * 48);
        assert_eq!(progress.queued_playback, 64);
    });
}
