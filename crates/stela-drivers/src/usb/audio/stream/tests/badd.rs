// SPDX-License-Identifier: GPL-2.0-only
//! BADD device model: physical power registers settle only after recovery time;
//! real alternate selection, terminal DMA retirement and PCM use the shared bus model.
use super::*;

fn configuration(profile: u8, playback: u8, capture: u8, speed: u8) -> Vec<u8> {
    let count = 1 + u8::from(playback != 0) + u8::from(capture != 0);
    let mut c = vec![9, 2, 0, 0, count, 1, 0, 0x80, 50];
    append(&mut c, &[8, 11, 0, count, 1, profile, 0x30, 0]);
    append(&mut c, &[9, 4, 0, 0, 0, 1, 1, 0x30, 0]);
    let mut interface = 1;
    for (channels, address) in [(playback, 1), (capture, 0x82)] {
        if channels == 0 {
            continue;
        }
        append(&mut c, &[9, 4, interface, 0, 0, 1, 2, 0x30, 0]);
        append(&mut c, &[9, 4, interface, 1, 1, 1, 2, 0x30, 0]);
        let packet = u16::from(channels) * 96;
        append(
            &mut c,
            &[
                7,
                5,
                address,
                0x0d,
                packet as u8,
                (packet >> 8) as u8,
                if speed == 3 { 4 } else { 1 },
            ],
        );
        interface += 1;
    }
    total(&mut c);
    c
}
fn total(c: &mut [u8]) {
    let n = c.len() as u16;
    c[2..4].copy_from_slice(&n.to_le_bytes());
}
fn parse(c: &[u8], speed: u8) -> Result<Binding, Error> {
    let all = interfaces(c);
    Binding::parse(&all[0], &all, c, speed)
}
struct BasicDevice {
    inner: Model,
    states: [u8; 2],
    settle: [u64; 2],
    writes: Vec<(u8, u8, u64)>,
    gains: BTreeMap<(u8, u8), i16>,
    mutes: BTreeMap<u8, bool>,
    fail_get: bool,
    fail_domain: u8,
    short_get: bool,
    fail_set: bool,
    ignore_wake: bool,
    ignore_idle: bool,
    inserted: bool,
}
impl BasicDevice {
    fn new() -> Self {
        Self {
            inner: Model::new(),
            states: [2, 1],
            settle: [0; 2],
            writes: Vec::new(),
            gains: BTreeMap::new(),
            mutes: BTreeMap::new(),
            fail_get: false,
            fail_domain: 0,
            short_get: false,
            fail_set: false,
            ignore_wake: false,
            ignore_idle: false,
            inserted: true,
        }
    }
    fn domain(owner: Direction) -> usize {
        if owner == Direction::Playback { 0 } else { 1 }
    }
    fn powered(&self, owner: Direction) {
        let d = Self::domain(owner);
        assert_eq!(self.states[d], 0, "DMA or alternate before real D0");
        assert!(
            self.inner.time >= self.settle[d],
            "DMA before power recovery deadline"
        );
    }
}
impl Control for BasicDevice {
    fn now_us(&self) -> u64 {
        self.inner.now_us()
    }
    fn relax(&mut self) {
        self.inner.relax();
    }
    fn control(&mut self, r: [u8; 8], out: &[u8]) -> Result<Vec<u8>, BlockError> {
        if r[0] == 1 && r[1] == 11 {
            let direction = if r[4] == 1 {
                Direction::Playback
            } else {
                Direction::Capture
            };
            if r[2] != 0 {
                self.powered(direction);
            } else {
                assert!(
                    self.inner.pending.values().all(|(e, _, _)| {
                        if direction == Direction::Playback {
                            !matches!(e.address, 1 | 0x83)
                        } else {
                            e.address != 0x82
                        }
                    }),
                    "alternate zero before packet retirement"
                );
            }
            return self.inner.control(r, out);
        }
        assert_eq!(r[4], 0);
        if matches!(r[5], 10 | 11) {
            assert_eq!((r[1], r[2], r[3], r[6], r[7]), (1, 0, 2, 1, 0));
            let index = usize::from(r[5] - 10);
            if r[0] == 0xa1 {
                if self.fail_get || self.fail_domain == r[5] {
                    return Err(BlockError::Timeout);
                }
                if self.short_get {
                    return Ok(Vec::new());
                }
                return Ok(vec![self.states[index]]);
            }
            assert_eq!(r[0], 0x21);
            assert_eq!(out.len(), 1);
            self.writes.push((r[5], out[0], self.inner.time));
            if self.fail_set {
                return Err(BlockError::Timeout);
            }
            if out[0] == 0 {
                let delay = match self.states[index] {
                    0 => 0,
                    1 => 30_000,
                    2 => 300_000,
                    _ => panic!("write from reserved power state"),
                };
                if !self.ignore_wake {
                    self.states[index] = 0;
                    self.settle[index] = self.inner.time + delay;
                }
            } else {
                assert_eq!(out[0], 1);
                let address = if index == 0 { 1 } else { 0x82 };
                assert!(!self.inner.active.contains(&address));
                assert!(
                    self.inner
                        .pending
                        .values()
                        .all(|(e, _, _)| e.address != address)
                );
                assert_eq!(
                    self.inner
                        .alternates
                        .iter()
                        .rev()
                        .find(|(i, _)| *i == (index + 1) as u8),
                    Some(&((index + 1) as u8, 0))
                );
                if !self.ignore_idle {
                    self.states[index] = 1;
                }
            }
            return Ok(Vec::new());
        }
        assert_ne!(r[5], 9, "fixed BADD clock is not a programmable UAC2 clock");
        if matches!(r[5], 3 | 4) {
            assert_eq!((r[0], r[1], r[2], r[3], r[6], r[7]), (0xa1, 1, 0, 1, 2, 0));
            return Ok(vec![1, u8::from(self.inserted)]);
        }
        assert!(matches!(r[5], 2 | 5 | 7));
        if r[3] == 1 {
            assert_eq!(r[2], 0);
            if r[0] == 0x21 {
                self.mutes.insert(r[5], out[0] != 0);
                return Ok(Vec::new());
            }
            return Ok(vec![u8::from(*self.mutes.get(&r[5]).unwrap_or(&false))]);
        }
        assert_eq!(r[3], 2);
        assert!((1..=2).contains(&r[2]));
        if r[0] == 0x21 {
            self.gains
                .insert((r[5], r[2]), i16::from_le_bytes(out.try_into().unwrap()));
            return Ok(Vec::new());
        }
        let reply = if r[1] == 2 {
            vec![1, 0, 0, 0xc0, 0, 0, 0, 1]
        } else {
            self.gains
                .get(&(r[5], r[2]))
                .copied()
                .unwrap_or(0)
                .to_le_bytes()
                .to_vec()
        };
        let n = usize::from(u16::from_le_bytes([r[6], r[7]]));
        Ok(reply[..reply.len().min(n)].to_vec())
    }
}
impl Transport for BasicDevice {
    // Independent in-memory device peer; it owns no native attachment.
    fn validate(&self) -> Result<(), BlockError> {
        Ok(())
    }
    type Ticket = u64;
    fn preflight(&mut self, e: &[Endpoint]) -> Result<(), BlockError> {
        self.inner.preflight(e)
    }
    fn configure(&mut self, o: Direction, e: &[Endpoint]) -> Result<(), BlockError> {
        self.powered(o);
        self.inner.configure(o, e)
    }
    fn deconfigure(&mut self, o: Direction, e: &[Endpoint]) -> Result<(), BlockError> {
        self.inner.deconfigure(o, e)
    }
    fn submit(&mut self, e: &Endpoint, p: u64, n: usize) -> Result<u64, BlockError> {
        self.powered(if e.is_in() && e.attributes & 0x30 != 0x10 {
            Direction::Capture
        } else {
            Direction::Playback
        });
        self.inner.submit(e, p, n)
    }
    fn completed(&mut self, t: &u64) -> Option<Result<Completion, BlockError>> {
        self.inner.completed(t)
    }
}
fn run(f: impl FnOnce(&mut AudioDevice<BasicDevice>)) {
    let mut dma = TestRegion::new(256 * 1024 * 8);
    let binding = parse(&configuration(0x25, 2, 1, 3), 3).unwrap();
    let mut audio = AudioDevice::new(BasicDevice::new(), binding, &mut dma).unwrap();
    f(&mut audio);
}
#[test]
fn basic_profiles_and_packet_encodings_are_selected_from_the_owning_iad() {
    for speed in [1, 3] {
        for (profile, playback, capture) in [
            (0x20, 1, 2),
            (0x20, 2, 0),
            (0x21, 2, 0),
            (0x22, 1, 0),
            (0x23, 0, 2),
            (0x24, 2, 1),
            (0x25, 2, 1),
            (0x26, 1, 1),
        ] {
            let b = parse(&configuration(profile, playback, capture, speed), speed).unwrap();
            assert_eq!(b.basic_profile.map(|p| p as u8), Some(profile));
            assert_eq!(
                b.formats.len(),
                usize::from(playback != 0) + usize::from(capture != 0)
            );
            assert!(
                b.formats
                    .iter()
                    .all(|f| f.service_microframes == 8 && f.endpoint_rates.is_none())
            );
        }
    }
    for (packet, channels, bytes, asynchronous) in [
        (0x60u16, 1, 2, false),
        (0x62, 1, 2, true),
        (0x90, 1, 3, false),
        (0x93, 1, 3, true),
        (0xc0, 2, 2, false),
        (0xc4, 2, 2, true),
        (0x120, 2, 3, false),
        (0x126, 2, 3, true),
    ] {
        let mut c = configuration(0x23, 0, 1, 3);
        let at = c.windows(2).position(|d| d == [7, 5]).unwrap();
        c[at + 3] = if asynchronous { 5 } else { 13 };
        c[at + 4..at + 6].copy_from_slice(&packet.to_le_bytes());
        let b = parse(&c, 3).unwrap();
        assert_eq!(
            (b.formats[0].channels, b.formats[0].subslot),
            (channels, bytes)
        );
    }
    for (profile, p, c) in [
        (0x21, 1, 0),
        (0x22, 2, 1),
        (0x23, 1, 1),
        (0x24, 2, 2),
        (0x25, 1, 1),
        (0x26, 2, 1),
    ] {
        assert!(parse(&configuration(profile, p, c, 3), 3).is_err());
    }
}
#[test]
fn badd_rejects_foreign_ownership_descriptor_mismatches_and_wrong_intervals() {
    let base = configuration(0x25, 2, 1, 3);
    for change in 0..8 {
        let mut c = base.clone();
        let ep = c.windows(2).position(|d| d == [7, 5]).unwrap();
        match change {
            0 => {
                c.drain(9..17);
            } // no owning IAD
            1 => c[14] = 1,     // generic UAC3 needs real high-capability queries
            2 => c[ep + 6] = 1, // capacity is for a millisecond, not a microframe
            3 => c[ep + 3] = 5, // async with sync packet encoding
            4 => {
                c.splice(ep..ep, [3, 0x24, 1]);
            } // BADD has no CS descriptors
            5 => c[12] = 2,     // association escapes capture
            6 => {
                c.splice(17..17, [8, 11, 1, 1, 1, 0x23, 0x30, 0]);
            } // overlapping IAD
            7 => {
                // foreign HID interface aliases the live playback DCI
                c[4] += 1;
                c.extend_from_slice(&[9, 4, 8, 0, 1, 3, 0, 0, 0, 7, 5, 1, 3, 8, 0, 1]);
            }
            _ => unreachable!(),
        }
        total(&mut c);
        assert!(parse(&c, 3).is_err(), "accepted malformed case {change}");
    }
}
#[test]
fn badd_duplex_pcm_observes_real_power_recovery_and_profile_controls() {
    run(|a| {
        assert_eq!(
            a.start_playback(0, 44_100, &[0; 192], true),
            Err(Error::RateUnsupported)
        );
        assert!(a.transport.writes.is_empty());
        a.set_mute(2, 0, true).unwrap();
        a.set_volume(2, 1, -512).unwrap();
        a.set_volume(5, 1, -256).unwrap();
        a.set_mute(7, 0, true).unwrap();
        a.set_volume(7, 1, -768).unwrap();
        assert!(a.set_volume(2, 0, -256).is_err());
        assert!(a.set_mute(2, 1, true).is_err());
        assert!(a.set_mixer_volume(8, 1, 1, -256).is_err());
        assert_eq!(a.connector_present(3), Ok(true));
        a.transport.inserted = false;
        assert_eq!(a.connector_present(4), Ok(false));
        let pcm: Vec<u8> = (0..6144).map(|i| i as u8).collect();
        a.start_playback(0, 48000, &pcm, true).unwrap();
        assert_eq!(a.transport.inner.time, 330_000);
        a.start_capture(1, 48000).unwrap();
        assert_eq!(a.transport.inner.time, 330_000);
        for _ in 0..3 {
            a.poll().unwrap();
            if a.progress.playback_drained {
                break;
            }
        }
        assert_eq!(a.transport.inner.output, pcm);
        assert!(a.progress.capture_frames > 0);
        let mut captured = [0u8; 192];
        assert_eq!(a.read_capture(&mut captured), Ok(192));
        assert!(captured.iter().any(|b| *b != 0));
        a.stop_playback().unwrap();
        assert_eq!(a.transport.states, [0, 0]);
        a.stop_capture().unwrap();
        assert_eq!(a.transport.states, [1, 1]);
        assert!(a.transport.inner.pending.is_empty());
    });
}
#[test]
fn badd_power_failures_never_publish_dma_or_erase_failed_cleanup_ownership() {
    for mode in 0..6 {
        run(|a| {
            match mode {
                0 => a.transport.fail_get = true,
                1 => a.transport.short_get = true,
                2 => a.transport.states[0] = 3,
                3 => a.transport.fail_set = true,
                4 => a.transport.ignore_wake = true,
                5 => {
                    a.transport.ignore_wake = true;
                    a.transport.ignore_idle = true;
                }
                _ => unreachable!(),
            }
            assert!(a.start_playback(0, 48000, &[0; 192], true).is_err());
            assert!(a.transport.inner.pending.is_empty());
            assert!(a.transport.inner.submitted.is_empty());
            if mode != 4 {
                assert_eq!(a.write_playback(&[0; 4]), Err(Error::Poisoned));
            }
            a.transport.fail_get = false;
            a.transport.short_get = false;
            a.transport.fail_set = false;
            a.transport.ignore_wake = false;
            a.transport.ignore_idle = false;
            if mode == 2 {
                a.transport.states[0] = 1;
            }
            a.shutdown().unwrap();
            assert!(a.playback.is_none());
        });
    }
    run(|a| {
        // Playback wakes, then the sibling power observation fails. Neither
        // stream may publish DMA; keep both domains until real recovery.
        a.transport.fail_domain = 11;
        assert_eq!(
            a.start_playback(0, 48000, &[0; 192], true),
            Err(Error::Poisoned)
        );
        assert_eq!(a.transport.states[0], 0);
        assert_eq!(a.power_owners.len(), 2);
        assert!(a.transport.inner.submitted.is_empty());
        a.transport.fail_domain = 0;
        a.shutdown().unwrap();
        assert!(a.power_owners.is_empty());
        assert_eq!(a.transport.states, [1, 1]);
    });
    run(|a| {
        a.transport.inner.hold = true;
        a.start_playback(0, 48000, &[0; 6144], false).unwrap();
        let pending = a.transport.inner.pending.len();
        a.transport.inner.cancel_fail = true;
        assert_eq!(a.stop_playback(), Err(Error::Poisoned));
        assert_eq!(a.transport.inner.pending.len(), pending);
        assert_eq!(a.transport.states[0], 0);
        assert!(
            !a.transport
                .writes
                .iter()
                .any(|(id, s, _)| (*id, *s) == (10, 1))
        );
        a.transport.inner.cancel_fail = false;
        a.transport.ignore_idle = true;
        assert_eq!(a.shutdown(), Err(Error::Poisoned));
        assert!(!a.power_owners.is_empty()); // keep failed power ownership after DMA retirement
        a.transport.ignore_idle = false;
        a.shutdown().unwrap();
        assert_eq!(a.transport.states[0], 1);
    });
}

#[test]
fn badd_standard_mono_alternate_and_explicit_feedback_stream_real_pcm() {
    run(|a| {
        let mut c = configuration(0x25, 2, 1, 3);
        let second = c
            .windows(9)
            .position(|d| d == [9, 4, 2, 0, 0, 1, 2, 0x30, 0])
            .unwrap();
        c.splice(
            second..second,
            [9, 4, 1, 2, 1, 1, 2, 0x30, 0, 7, 5, 1, 13, 96, 0, 4],
        );
        total(&mut c);
        a.binding = parse(&c, 3).unwrap();
        assert_eq!(a.binding.formats[1].channels, 1);
        a.start_playback(1, 48000, &[0x35; 3072], true).unwrap();
        a.poll().unwrap();
        assert_eq!(a.transport.inner.output, vec![0x35; 3072]);
        a.stop_playback().unwrap();
        // Async stereo with the standard 4-byte high-speed feedback endpoint.
        c = configuration(0x25, 2, 1, 3);
        let primary = c
            .windows(9)
            .position(|d| d == [9, 4, 1, 1, 1, 1, 2, 0x30, 0])
            .unwrap();
        c[primary + 4] = 2;
        let ep = primary + 9;
        c[ep + 3] = 5;
        c[ep + 4] = 0xc4;
        c.splice(ep + 7..ep + 7, [7, 5, 0x83, 0x11, 4, 0, 4]);
        total(&mut c);
        a.binding = parse(&c, 3).unwrap();
        assert!(matches!(a.binding.formats[0].sync, Sync::Explicit(_)));
        a.transport.inner.output.clear();
        a.start_playback(0, 48000, &[0x72; 24576], true).unwrap();
        a.start_capture(1, 48000).unwrap();
        a.poll().unwrap();
        assert!(a.progress.feedback_updates > 0);
        assert_eq!(a.transport.inner.output, vec![0x72; 12288]);
        a.stop_capture().unwrap(); // keep the duplex power session awake while playback remains live
        assert_eq!(a.transport.states, [0, 0]);
        a.stop_playback().unwrap();
    });
    run(|a| {
        a.binding = parse(&configuration(0x25, 2, 1, 1), 1).unwrap();
        a.start_playback(0, 48000, &[0x49; 6144], true).unwrap();
        a.poll().unwrap();
        assert_eq!(a.transport.inner.output, vec![0x49; 6144]);
        a.shutdown().unwrap();
    });
}

#[test]
fn badd_rejected_capture_request_preserves_live_playback_power_and_dma() {
    run(|a| {
        a.transport.inner.hold = true;
        a.start_playback(0, 48000, &[0x36; 12288], false).unwrap();
        let pending = a.transport.inner.pending.len();
        let alternates = a.transport.inner.alternates.clone();
        let power_writes = a.transport.writes.clone();
        let now = a.transport.inner.time;
        assert_eq!(a.start_capture(1, 44100), Err(Error::RateUnsupported));
        assert!(a.playback.is_some());
        assert_eq!(a.transport.inner.pending.len(), pending);
        assert_eq!(a.transport.inner.alternates, alternates);
        assert_eq!(a.transport.writes, power_writes);
        assert_eq!(a.transport.inner.time, now);
        assert_eq!(a.transport.states, [0, 0]);
        // A foreign capture DCI refusal is also wholly prepublication. It may
        // not stop or deconfigure either the sibling or that foreign owner.
        a.transport.inner.active.push(0x82);
        assert!(a.start_capture(1, 48000).is_err());
        assert!(a.playback.is_some());
        assert_eq!(a.transport.inner.pending.len(), pending);
        assert_eq!(a.transport.inner.alternates, alternates);
        assert_eq!(a.transport.writes, power_writes);
        assert!(a.transport.inner.active.contains(&0x82));
        assert_eq!(a.write_playback(&[0; 4]), Ok(()));
        a.transport.inner.active.retain(|e| *e != 0x82);
        a.shutdown().unwrap();
    });
}

#[test]
fn badd_failed_live_power_permission_check_retires_or_quarantines_the_session() {
    for mode in 0..3 {
        run(|a| {
            a.transport.inner.hold = true;
            a.start_playback(0, 48000, &[0x61; 12288], false).unwrap();
            assert!(!a.transport.inner.pending.is_empty());
            match mode {
                0 => a.transport.states[0] = 1,
                1 => a.transport.short_get = true,
                2 => a.transport.fail_get = true,
                _ => unreachable!(),
            }
            assert!(a.start_capture(1, 48000).is_err());
            assert!(a.transport.inner.pending.is_empty());
            // A completely acknowledged cleanup may return to idle; otherwise
            // power ownership remains poisoned. Neither permits producer use.
            assert!(matches!(
                a.write_playback(&[0; 4]),
                Err(Error::NotRunning | Error::Poisoned)
            ));
            if mode != 0 {
                assert!(!a.power_owners.is_empty());
            }
            a.transport.short_get = false;
            a.transport.fail_get = false;
            a.shutdown().unwrap();
            assert!(a.power_owners.is_empty());
        });
    }
}
