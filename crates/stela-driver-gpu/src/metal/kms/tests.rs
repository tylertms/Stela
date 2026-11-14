use super::*;
use alloc::{vec, vec::Vec};

mod outputs;
mod scanout;

fn mode(width: u16, height: u16, kind: u32) -> ModeInfo {
    ModeInfo {
        hdisplay: width,
        vdisplay: height,
        kind,
        ..ModeInfo::default()
    }
}

#[test]
fn a_modes_refresh_is_its_clock_over_its_totals() {
    // CVT-RB 2560x1440 at 144 Hz: 2720 by 1525 at 597.31 MHz.
    let fast = ModeInfo {
        clock: 597_310,
        timing_h: [2608, 2640, 2720, 0],
        timing_v: [1443, 1448, 1525, 0],
        vrefresh: 144,
        ..mode(2560, 1440, 0)
    };
    assert_eq!(refresh(&fast), 144_000);
    let untimed = ModeInfo {
        vrefresh: 75,
        ..mode(1280, 800, 0)
    };
    assert_eq!(refresh(&untimed), 75_000);
    assert_eq!(refresh(&mode(1280, 800, 0)), 60_000);
}

/// A GPU of two connectors and two controllers: connector 50 on encoder 61,
/// which reaches both controllers, and connector 51, connected with
/// `modes`, on encoder 60, which reaches the second. Controller 40 runs
/// only connector 50 and 41 only 51.
struct Card {
    modes: Vec<ModeInfo>,
    possible_crtcs: u32,
    /// The mode and framebuffer controller 41 was set to.
    set: Option<(ModeInfo, u32)>,
    /// Requests to answer `EBUSY` before a flip on controller 41 succeeds.
    busy: u32,
    /// The first connector's modes, whether it has a display, and what its
    /// encoder reaches.
    first: Vec<ModeInfo>,
    first_connected: bool,
    first_crtcs: u32,
    /// The first connector lists encoder 60, the second's.
    shared_encoder: bool,
    /// The connector the driver read back as lit, with a current encoder.
    lit: u32,
    /// Controller 40's mode and framebuffer, its flips, and the requests to
    /// answer `EBUSY` before one succeeds.
    set_40: Option<(ModeInfo, u32)>,
    flips_40: Vec<u32>,
    busy_40: u32,
    /// A controller whose mode sets are refused.
    refused_crtc: Option<u32>,
    flip_error: Option<i64>,
    /// Whether a request completes on the processor making it.
    immediate: bool,
    /// Flips requested, whether the engine took them or was busy.
    flip_requests: u32,
    flips: Vec<u32>,
    next: u32,
    memory: Vec<Vec<u8>>,
    pitches: Vec<u32>,
    invalid_buffer: Option<(u32, u64)>,
    resource_changes: usize,
    mode_changes: usize,
    /// Whether the second connector has a display.
    connected: bool,
    /// Connector requests that probed: those that list no modes.
    probes: u32,
    /// Controller mode sets, the ones that turned it off included.
    mode_sets: u32,
    /// What the next mode set answers instead of setting the mode.
    set_error: Option<i64>,
    /// Mappings to refuse.
    map_refusals: u32,
    /// Dumb buffers and framebuffers given back, and mappings removed.
    destroyed: Vec<u32>,
    removed: Vec<u32>,
    unmapped: usize,
}

impl Card {
    fn new(modes: Vec<ModeInfo>) -> Self {
        Self {
            modes,
            possible_crtcs: 0b10,
            set: None,
            busy: 0,
            first: Vec::new(),
            first_connected: false,
            first_crtcs: 0b11,
            shared_encoder: false,
            lit: 0,
            set_40: None,
            flips_40: Vec::new(),
            busy_40: 0,
            refused_crtc: None,
            flip_error: None,
            immediate: true,
            flip_requests: 0,
            flips: Vec::new(),
            next: 100,
            memory: Vec::new(),
            pitches: Vec::new(),
            invalid_buffer: None,
            resource_changes: 0,
            mode_changes: 0,
            connected: true,
            probes: 0,
            mode_sets: 0,
            set_error: None,
            map_refusals: 0,
            destroyed: Vec::new(),
            removed: Vec::new(),
            unmapped: 0,
        }
    }

    fn id(&mut self) -> u32 {
        self.next += 1;
        self.next
    }
}

// SAFETY: each mapping owns a separate Vec whose allocation stays put in
// `memory` until the card is dropped, after the borrowing display.
unsafe impl Drm for &mut Card {
    fn control(&mut self, request: u32, argument: *mut c_void) -> i64 {
        // SAFETY: each arm reads the record its request carries.
        unsafe {
            match request {
                GET_RESOURCES => {
                    let resources = &mut *argument.cast::<CardResources>();
                    if resources.crtc_count != 0 {
                        let crtcs = resources.crtcs as *mut u32;
                        crtcs.write(40);
                        crtcs.add(1).write(41);
                        let connectors = resources.connectors as *mut u32;
                        connectors.write(50);
                        connectors.add(1).write(51);
                    }
                    resources.crtc_count = 2;
                    resources.connector_count = 2;
                    if resources.crtcs != 0 && self.resource_changes != 0 {
                        self.resource_changes -= 1;
                        resources.connector_count = 3;
                    }
                }
                GET_CONNECTOR => {
                    let connector = &mut *argument.cast::<Connector>();
                    let (connected, listed, encoder) = match connector.id {
                        50 => (
                            self.first_connected,
                            &self.first,
                            if self.shared_encoder { 60 } else { 61 },
                        ),
                        _ => (self.connected, &self.modes, 60),
                    };
                    connector.connection = if connected { CONNECTED } else { 2 };
                    connector.encoder = if connector.id == self.lit { encoder } else { 0 };
                    let modes: &[ModeInfo] = if connected { listed } else { &[] };
                    self.probes += u32::from(connector.mode_count == 0);
                    // Linux fills a list only when all of it fits.
                    if connector.mode_count as usize >= modes.len() && connector.mode_count != 0 {
                        let list = connector.modes as *mut ModeInfo;
                        ptr::copy_nonoverlapping(modes.as_ptr(), list, modes.len());
                    }
                    if connector.encoder_count != 0 {
                        (connector.encoders as *mut u32).write(encoder);
                    }
                    connector.mode_count = modes.len() as u32;
                    connector.encoder_count = 1;
                    if connected && connector.modes != 0 && self.mode_changes != 0 {
                        self.mode_changes -= 1;
                        connector.mode_count += 1;
                    }
                }
                GET_ENCODER => {
                    let encoder = &mut *argument.cast::<Encoder>();
                    encoder.possible_crtcs = match encoder.id {
                        60 => self.possible_crtcs,
                        61 => self.first_crtcs,
                        other => panic!("encoder {other}"),
                    };
                }
                CREATE_DUMB => {
                    let dumb = &mut *argument.cast::<CreateDumb>();
                    dumb.pitch = self
                        .pitches
                        .get(self.memory.len())
                        .copied()
                        .unwrap_or(dumb.width * BYTES_PER_PIXEL + 16);
                    dumb.size = u64::from(dumb.pitch) * u64::from(dumb.height);
                    if let Some((pitch, size)) = self.invalid_buffer {
                        dumb.pitch = pitch;
                        dumb.size = size;
                    }
                    dumb.handle = self.id();
                }
                ADD_FB2 => {
                    let framebuffer = &mut *argument.cast::<FramebufferCommand>();
                    assert_eq!(framebuffer.format, XRGB8888);
                    assert_ne!(framebuffer.pitches[0], 0);
                    framebuffer.id = self.id();
                }
                MAP_DUMB => (*argument.cast::<MapDumb>()).offset = 0x1000,
                SET_CRTC => {
                    let crtc = &*argument.cast::<Crtc>();
                    let (runs, set) = match crtc.id {
                        40 => (50, &mut self.set_40),
                        41 => (51, &mut self.set),
                        other => panic!("controller {other}"),
                    };
                    self.mode_sets += 1;
                    if let Some(error) = self.set_error.take() {
                        return error;
                    }
                    if self.refused_crtc == Some(crtc.id) {
                        return -22;
                    }
                    *set = (crtc.connector_count != 0).then(|| {
                        assert_eq!((crtc.connectors as *const u32).read(), runs);
                        (crtc.mode, crtc.framebuffer)
                    });
                }
                PAGE_FLIP => {
                    assert!(self.immediate, "a flip requested where it must wait");
                    self.flip_requests += 1;
                    if let Some(error) = self.flip_error.take() {
                        return error;
                    }
                    let flip = *argument.cast::<PageFlip>();
                    let (busy, flips) = match flip.crtc {
                        40 => (&mut self.busy_40, &mut self.flips_40),
                        41 => (&mut self.busy, &mut self.flips),
                        other => panic!("controller {other}"),
                    };
                    if *busy != 0 {
                        *busy -= 1;
                        return BUSY;
                    }
                    flips.push(flip.framebuffer);
                }
                REMOVE_FB => self.removed.push(*argument.cast::<u32>()),
                DESTROY_DUMB => self
                    .destroyed
                    .push((*argument.cast::<DestroyDumb>()).handle),
                other => panic!("request {other:#x}"),
            }
        }
        0
    }

    fn unmap(&mut self, pixels: NonNull<u8>, bytes: usize) {
        assert!(
            self.memory
                .iter()
                .any(|memory| memory.as_ptr() == pixels.as_ptr() && memory.len() == bytes)
        );
        self.unmapped += 1;
    }

    fn map(&mut self, _: u64, bytes: usize) -> Option<NonNull<u8>> {
        if self.map_refusals != 0 {
            self.map_refusals -= 1;
            return None;
        }
        self.memory.push(vec![0xa5; bytes]);
        NonNull::new(self.memory.last_mut()?.as_mut_ptr())
    }

    fn immediate(&self) -> bool {
        self.immediate
    }
}

#[test]
fn requests_match_linux() {
    assert_eq!(GET_RESOURCES, 0xc040_64a0);
    assert_eq!(GET_CONNECTOR, 0xc050_64a7);
    assert_eq!(GET_ENCODER, 0xc014_64a6);
    assert_eq!(SET_CRTC, 0xc068_64a2);
    assert_eq!(PAGE_FLIP, 0xc018_64b0);
    assert_eq!(CREATE_DUMB, 0xc020_64b2);
    assert_eq!(MAP_DUMB, 0xc010_64b3);
    assert_eq!(ADD_FB2, 0xc068_64b8);
    assert_eq!(REMOVE_FB, 0xc004_64af);
    assert_eq!(DESTROY_DUMB, 0xc004_64b4);
}

#[test]
fn the_firmwares_size_wins_then_the_preferred_mode() {
    let modes = [
        mode(1280, 720, 0),
        mode(3840, 2160, PREFERRED),
        mode(1920, 1080, 0),
    ];
    let any: Fits<'_> = &|_, _, _| true;
    assert_eq!(
        choose(&modes, Wanted::Size((1920, 1080)), any),
        Some(modes[2])
    );
    assert_eq!(
        choose(&modes, Wanted::Size((800, 600)), any),
        Some(modes[1])
    );
    assert_eq!(
        choose(&modes[..1], Wanted::Size((800, 600)), any),
        Some(modes[0])
    );
    assert_eq!(choose(&[], Wanted::Size((800, 600)), any), None);
}

/// Before the guest's mode is set the monitor's own mode wins: its fastest
/// of the preferred size that fits, then the firmware's size.
#[test]
fn a_display_taken_at_start_runs_the_monitors_own_mode() {
    let at = |width, height, kind, vrefresh| ModeInfo {
        vrefresh,
        ..mode(width, height, kind)
    };
    let modes = [
        at(3840, 2160, PREFERRED, 60),
        at(3840, 2160, 0, 144),
        at(1920, 1080, 0, 240),
        at(1920, 1080, 0, 60),
    ];
    let any: Fits<'_> = &|_, _, _| true;
    let firmware = Wanted::Native((1920, 1080));
    assert_eq!(choose(&modes, firmware, any), Some(modes[0]));
    assert_eq!(
        choose(&modes, Wanted::Size((1920, 1080)), any),
        Some(modes[2])
    );
    // The native size cannot be described at all: the firmware's size.
    let small: Fits<'_> = &|width, _, _| width < 3840;
    assert_eq!(choose(&modes, firmware, small), Some(modes[2]));
    // Its listed-first rate cannot: the next of the native size.
    let fast = [modes[1], at(3840, 2160, PREFERRED, 60), modes[2]];
    let slow: Fits<'_> = &|_, _, refresh| refresh < 100_000;
    assert_eq!(choose(&fast, firmware, slow), Some(fast[1]));
    // No preferred mode: the firmware's size, then the first.
    assert_eq!(choose(&modes[2..], firmware, any), Some(modes[2]));
    assert_eq!(
        choose(&modes[2..], Wanted::Native((800, 600)), any),
        Some(modes[2])
    );
}

/// A monitor lists its fastest mode of a size first. One the guest cannot
/// be told of is passed over for the next of that size; with none of that
/// size, the preferred one that fits, then any; and with no mode that fits
/// at all, the choice is as without the test.
#[test]
fn a_mode_the_guest_cannot_be_told_of_is_passed_over() {
    let at = |width, height, kind, vrefresh| ModeInfo {
        vrefresh,
        ..mode(width, height, kind)
    };
    let modes = [
        at(2560, 1440, PREFERRED, 180),
        at(2560, 1440, 0, 144),
        at(2560, 1440, 0, 60),
        at(1920, 1080, 0, 240),
        at(1920, 1080, 0, 60),
    ];
    let below_165: Fits<'_> = &|_, _, refresh| refresh < 165_000;
    assert_eq!(
        choose(&modes, Wanted::Size((2560, 1440)), below_165),
        Some(modes[1])
    );
    assert_eq!(
        choose(&modes, Wanted::Size((1920, 1080)), below_165),
        Some(modes[4])
    );
    // No mode of that size: the preferred does not fit, so the first that does.
    assert_eq!(
        choose(&modes, Wanted::Size((800, 600)), below_165),
        Some(modes[1])
    );
    let none: Fits<'_> = &|_, _, _| false;
    assert_eq!(
        choose(&modes, Wanted::Size((1920, 1080)), none),
        Some(modes[3])
    );
    assert_eq!(
        choose(&modes, Wanted::Size((800, 600)), none),
        Some(modes[0])
    );
}

#[test]
fn the_connected_output_runs_on_a_controller_it_can_reach() {
    let modes = vec![mode(4, 2, PREFERRED)];
    let mut card = Card::new(modes.clone());
    let front = {
        let display = Display::take(&mut card, (4, 2)).unwrap();
        assert_eq!(display.size(), (4, 2));
        display.buffers[0].framebuffer
    };
    assert_eq!(card.set, Some((modes[0], front)));
}

#[test]
fn the_survey_names_every_connector_with_what_the_driver_detected() {
    let mut card = Card::new(vec![mode(4, 2, PREFERRED)]);
    let seen = survey(&mut &mut card).unwrap();
    assert_eq!(seen.len(), 2);
    assert_eq!(seen.iter().filter(|seen| seen.connected).count(), 1);
    let connected = seen.iter().find(|seen| seen.connected).unwrap();
    assert_eq!(
        alloc::format!("{connected}"),
        "Unknown-0 connected modes=1 encoders=1"
    );
    let other = seen.iter().find(|seen| !seen.connected).unwrap();
    assert_eq!(
        alloc::format!("{other}"),
        "Unknown-0 not connected modes=0 encoders=1"
    );
}

#[test]
fn an_output_with_no_mode_or_controller_is_not_taken() {
    let mut card = Card::new(Vec::new());
    assert_eq!(
        Display::take(&mut card, (4, 2)).err(),
        Some(Error::NoOutput)
    );
    let mut card = Card::new(vec![mode(4, 2, 0)]);
    card.possible_crtcs = 0;
    assert_eq!(
        Display::take(&mut card, (4, 2)).err(),
        Some(Error::NoOutput)
    );
}

#[test]
fn a_busy_engine_keeps_the_flip_queued_for_a_later_flush_without_waiting() {
    let mut card = Card::new(vec![mode(2, 2, PREFERRED)]);
    card.busy = 2;
    let (first, second, flips);
    {
        let mut display = Display::take(&mut card, (2, 2)).unwrap();
        let (front, back, spare) = (
            display.buffers[0].framebuffer,
            display.buffers[1].framebuffer,
            display.buffers[2].framebuffer,
        );
        let rows: Vec<u8> = (0..16).collect();
        display.present(&rows).unwrap();
        // The engine was busy: nothing waits, the frame stays queued.
        assert!(display.queued());
        assert_eq!(display.buffers[0].framebuffer, front);
        display.flush().unwrap();
        assert!(display.queued());
        display.flush().unwrap();
        assert!(!display.queued());
        assert_eq!(display.buffers[0].framebuffer, spare);
        display.present(&rows).unwrap();
        assert_eq!(display.buffers[0].framebuffer, back);
        // Nothing queued: a flush requests nothing.
        display.flush().unwrap();
        assert_eq!(display.drm.flip_requests, 4);
        flips = [spare, back];
        first = rows[..8].to_vec();
        second = rows[8..].to_vec();
    }
    assert_eq!(card.flips, flips);
    // The rows land `pitch` apart.
    let filled = &card.memory[2];
    assert_eq!(&filled[..8], first);
    assert_eq!(&filled[24..32], second);
}

#[test]
fn a_present_where_a_request_must_wait_only_fills_and_queues() {
    let mut card = Card::new(vec![mode(2, 2, PREFERRED)]);
    let spare;
    {
        let mut display = Display::take(&mut card, (2, 2)).unwrap();
        spare = display.buffers[2].framebuffer;
        // Another processor than the one that serves requests: the card
        // asserts that no flip is requested.
        display.drm.immediate = false;
        display.present(&[1; 16]).unwrap();
        assert!(display.queued());
        display.flush().unwrap();
        // A newer frame replaces the queued one in the same hidden buffer.
        display.present(&[2; 16]).unwrap();
        assert_eq!(display.buffers[2].framebuffer, spare);
        assert_eq!(display.drm.flip_requests, 0);
        // On the serving processor the flush requests one flip.
        display.drm.immediate = true;
        display.flush().unwrap();
        assert!(!display.queued());
        assert_eq!(display.buffers[0].framebuffer, spare);
    }
    assert_eq!(card.flips, [spare]);
    assert_eq!(&card.memory[2][..8], &[2; 8]);
}

#[test]
fn each_buffer_is_filled_with_its_own_stale_rows_and_the_damage() {
    let mut card = Card::new(vec![mode(2, 8, PREFERRED)]);
    let mut display = Display::take(&mut card, (2, 8)).unwrap();
    let mut asked = Vec::new();
    for damage in [None, Some(0..1), Some(1..2), Some(2..3), Some(2..3)] {
        let mut rows = Vec::new();
        display
            .prepare_rows(damage, |y, row| {
                rows.push(y);
                row.fill(y as u8);
                Ok(())
            })
            .unwrap()
            .present()
            .unwrap();
        asked.push(rows);
    }
    let every: Vec<usize> = (0..8).collect();
    // The first frame and the two buffers that held only black need every
    // row; then each buffer needs what changed since its own last frame.
    assert_eq!(asked[..3], [every.clone(), every.clone(), every]);
    assert_eq!(asked[3], [0, 1, 2]);
    assert_eq!(asked[4], [1, 2]);
}

#[test]
fn a_detected_display_is_read_without_a_probe() {
    let mut card = Card::new(vec![mode(4, 2, PREFERRED), mode(2, 2, 0)]);
    assert_eq!(detected(&mut &mut card), Ok(true));
    card.connected = false;
    assert_eq!(detected(&mut &mut card), Ok(false));
    assert_eq!(card.probes, 0);
}

#[test]
fn an_output_that_returns_gets_a_full_mode_set_on_the_buffer_on_screen() {
    let modes = vec![mode(2, 2, PREFERRED)];
    let mut card = Card::new(modes.clone());
    let front;
    {
        let mut display = Display::take(&mut card, (2, 2)).unwrap();
        front = display.buffers[0].framebuffer;
        let unchanged = Watched::default();
        assert_eq!(display.watch(), Ok(unchanged));
        display.drm.connected = false;
        let lost = Watched {
            lost: 1,
            ..unchanged
        };
        assert_eq!(display.watch(), Ok(lost));
        assert_eq!(display.watch(), Ok(unchanged));
        // Where a request must wait, nothing is read.
        display.drm.connected = true;
        display.drm.immediate = false;
        assert_eq!(display.watch(), Ok(unchanged));
        display.drm.immediate = true;
        let probes = display.drm.probes;
        let restored = Watched {
            restored: 1,
            ..unchanged
        };
        assert_eq!(display.watch(), Ok(restored));
        assert_eq!(display.drm.probes, probes);
        assert_eq!(display.watch(), Ok(unchanged));
    }
    // The first set, then off and on again.
    assert_eq!(card.mode_sets, 3);
    assert_eq!(card.set, Some((modes[0], front)));
}

#[test]
fn a_display_that_is_not_taken_gives_its_buffers_back() {
    // The mode set is refused: all three buffers go.
    let mut third = Card::new(vec![mode(1920, 1080, PREFERRED)]);
    third.set_error = Some(-22);
    assert!(Display::take(&mut third, (1920, 1080)).is_err());
    assert_eq!(third.destroyed.len(), 3);
    assert_eq!(third.removed.len(), 3);
    assert_eq!(third.unmapped, 3);
    assert_eq!(third.set, None);

    // A refused mapping: the buffer and its framebuffer go, with no mapping.
    let mut card = Card::new(vec![mode(1920, 1080, PREFERRED)]);
    card.map_refusals = 1;
    assert_eq!(
        Display::take(&mut card, (1920, 1080)).err(),
        Some(Error::Map)
    );
    assert_eq!(card.destroyed.len(), 1);
    assert_eq!(card.removed.len(), 1);
    assert_eq!(card.unmapped, 0);

    // A buffer of the wrong shape has no framebuffer yet.
    let mut shaped = Card::new(vec![mode(1920, 1080, PREFERRED)]);
    shaped.invalid_buffer = Some((4, 4));
    assert_eq!(
        Display::take(&mut shaped, (1920, 1080)).err(),
        Some(Error::InvalidBuffer)
    );
    assert_eq!(shaped.destroyed.len(), 1);
    assert!(shaped.removed.is_empty());

    // The next look takes the display.
    assert!(Display::take(&mut card, (1920, 1080)).is_ok());
}
