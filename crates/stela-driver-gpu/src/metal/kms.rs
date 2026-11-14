//! The host display over DRM's mode setting, once a GPU's driver owns it.
//!
//! Binding a GPU ends what the firmware left: its framebuffer sits behind a
//! BAR the driver remaps, and its display engine is the driver's to program.
//! [`Display`] scans out one of three linear buffers of video memory, each a
//! DRM dumb buffer: the one a flip last named, the one before it, which shows
//! until that flip lands, and the one a frame fills.
//!
//! Several connected outputs show the same picture, as Linux's fbdev helper
//! clones them: one size that most of them run ([`plan`]), each output on its
//! own controller in its own mode of that size, all scanning out the same
//! buffers. The guest has one display, so an output with no mode of that size
//! stays dark; no scaler exists. The output the firmware lit, which the driver
//! reads back as the connector's current encoder, wins ties, then DRM's
//! order. The first output paces the flips; the others follow it to the
//! newest buffer, and one that lags a frame may show a frame being filled.
//!
//! NVIDIA's display engine reads only video memory, so a guest surface in
//! guest RAM cannot be scanned out in place: frames copy in, top to bottom,
//! and only the rows a buffer lacks since it last held a frame.
//!
//! A present never waits. Its flip is queued and requested at once only
//! where a request completes without another processor ([`Drm::immediate`]):
//! a present runs under the board's device lock, and a request from any
//! other processor waits for the boot processor, which may itself be
//! waiting for that lock. A queued flip is requested by a later
//! [`Display::flush`] on the boot processor. A flip the engine has not
//! finished answers `EBUSY` and stays queued for the next flush; a newer
//! frame meanwhile replaces the queued one in the same hidden buffer.
//!
//! Display sleep: [`Display::set_power`] turns every shown output's controller
//! off, as Linux's fbdev helper does for a blanked console, and back on in its
//! mode on the buffer on screen; like a flip, the request waits for a
//! processor where it completes at once ([`Display::flush`]).
//!
//! Hot-plug: a connector's state as the driver last detected it, which its
//! hot-plug interrupt keeps current, is read with no probe ([`detected`]).
//! An output that returns after it was seen disconnected gets its mode set
//! again ([`Display::watch`]), as Linux's fbdev helper does on a hot-plug
//! event, and one connected after the take joins the picture if it can.
use alloc::vec::Vec;
use core::ffi::c_void;
use core::ops::Range;
use core::ptr::{self, NonNull};
use stela_contracts::{Refusal, refuse};

/// `_IOWR('d', number, T)`.
const fn request<T>(number: u32) -> u32 {
    3 << 30 | (size_of::<T>() as u32) << 16 | 0x64 << 8 | number
}

const GET_RESOURCES: u32 = request::<CardResources>(0xa0);
const SET_CRTC: u32 = request::<Crtc>(0xa2);
const GET_ENCODER: u32 = request::<Encoder>(0xa6);
const GET_CONNECTOR: u32 = request::<Connector>(0xa7);
const PAGE_FLIP: u32 = request::<PageFlip>(0xb0);
const CREATE_DUMB: u32 = request::<CreateDumb>(0xb2);
const MAP_DUMB: u32 = request::<MapDumb>(0xb3);
const ADD_FB2: u32 = request::<FramebufferCommand>(0xb8);
const REMOVE_FB: u32 = request::<u32>(0xaf);
const DESTROY_DUMB: u32 = request::<DestroyDumb>(0xb4);

/// `DRM_MODE_CONNECTED`.
const CONNECTED: u32 = 1;
/// `DRM_MODE_TYPE_PREFERRED`.
const PREFERRED: u32 = 1 << 3;
/// `DRM_FORMAT_XRGB8888`: blue in the low byte of the little-endian pixel.
const XRGB8888: u32 = u32::from_le_bytes(*b"XR24");
const BYTES_PER_PIXEL: u32 = 4;
/// `EBUSY`, as a negative result.
const BUSY: i64 = -16;
/// Hotplug may change a list between its size query and its data query.
const ENUMERATION_ATTEMPTS: usize = 8;

/// `struct drm_mode_card_res`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct CardResources {
    framebuffers: u64,
    crtcs: u64,
    connectors: u64,
    encoders: u64,
    framebuffer_count: u32,
    crtc_count: u32,
    connector_count: u32,
    encoder_count: u32,
    limits: [u32; 4],
}

/// `struct drm_mode_modeinfo`.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
#[repr(C)]
pub struct ModeInfo {
    clock: u32,
    hdisplay: u16,
    timing_h: [u16; 4],
    vdisplay: u16,
    timing_v: [u16; 4],
    vrefresh: u32,
    flags: u32,
    kind: u32,
    name: [[u8; 8]; 4],
}

/// `struct drm_mode_get_connector`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct Connector {
    encoders: u64,
    modes: u64,
    properties: u64,
    property_values: u64,
    mode_count: u32,
    property_count: u32,
    encoder_count: u32,
    encoder: u32,
    id: u32,
    kind: u32,
    kind_id: u32,
    connection: u32,
    size_mm: [u32; 2],
    subpixel: u32,
    pad: u32,
}

/// `struct drm_mode_get_encoder`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct Encoder {
    id: u32,
    kind: u32,
    crtc: u32,
    possible_crtcs: u32,
    possible_clones: u32,
}

/// `struct drm_mode_crtc`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct Crtc {
    connectors: u64,
    connector_count: u32,
    id: u32,
    framebuffer: u32,
    x: u32,
    y: u32,
    gamma_size: u32,
    mode_valid: u32,
    mode: ModeInfo,
}

/// `struct drm_mode_create_dumb`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct CreateDumb {
    height: u32,
    width: u32,
    bits_per_pixel: u32,
    flags: u32,
    handle: u32,
    pitch: u32,
    size: u64,
}

/// `struct drm_mode_destroy_dumb`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct DestroyDumb {
    handle: u32,
}

/// `struct drm_mode_map_dumb`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct MapDumb {
    handle: u32,
    pad: u32,
    offset: u64,
}

/// `struct drm_mode_fb_cmd2`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct FramebufferCommand {
    id: u32,
    width: u32,
    height: u32,
    format: u32,
    flags: u32,
    handles: [u32; 4],
    pitches: [u32; 4],
    offsets: [u32; 4],
    modifiers: [u64; 4],
}

/// `struct drm_mode_crtc_page_flip`.
#[derive(Clone, Copy, Debug, Default)]
#[repr(C)]
struct PageFlip {
    crtc: u32,
    framebuffer: u32,
    flags: u32,
    reserved: u32,
    user_data: u64,
}

/// What stops the display.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Error {
    /// No output is connected, or none has a mode or a controller to run it.
    NoOutput,
    /// A request failed, with its negative errno.
    Request(i64),
    /// A buffer could not be mapped.
    Map,
    /// Hotplug kept changing the connector or controller lists.
    Changed,
    /// A returned scanout allocation cannot hold the selected mode.
    InvalidBuffer,
    /// A frame must contain every pixel of the selected mode exactly once.
    FrameSize { expected: usize, actual: usize },
    /// Storage for the mode or resource lists could not be allocated.
    Allocation,
}

/// A DRM primary node of the bound GPU.
///
/// # Safety
/// Successful mappings must be writable for the requested extent and stay
/// mapped at the same address while this node's display uses them. Distinct
/// dumb buffers must not alias. Ioctls must honor the UAPI's pointer/count
/// bounds and report the allocation's actual pitch and size.
pub unsafe trait Drm {
    /// The ioctl `request` on `argument`: zero or a negative errno.
    fn control(&mut self, request: u32, argument: *mut c_void) -> i64;
    /// Maps `bytes` at `offset`, what `MAP_DUMB` answered.
    fn map(&mut self, offset: u64, bytes: usize) -> Option<NonNull<u8>>;
    /// Removes the mapping of `bytes` that [`Drm::map`] gave at `pixels`.
    fn unmap(&mut self, pixels: NonNull<u8>, bytes: usize);
    /// Whether a request made now completes without waiting for another
    /// processor. A display queues what it cannot request now.
    fn immediate(&self) -> bool;
}

/// A scanned-out buffer.
#[derive(Debug)]
struct Buffer {
    framebuffer: u32,
    pixels: NonNull<u8>,
    pitch: usize,
    /// The dumb buffer's handle and mapped bytes, to give it back.
    handle: u32,
    bytes: usize,
}

/// One output showing the picture: its controller, connector and mode.
#[derive(Debug)]
struct Head {
    crtc: u32,
    connector: u32,
    /// The connector's encoders, which no other head may use.
    encoders: Vec<u32>,
    mode: ModeInfo,
    /// The connector was last read disconnected ([`Display::watch`]).
    lost: bool,
    /// Restores tried since the connector was last read disconnected.
    tries: u8,
    /// Not the first head: its flip to the newest buffer is still to be
    /// requested.
    behind: bool,
}

/// The outputs a [`Display`] runs.
#[derive(Debug)]
pub struct Display<D> {
    drm: D,
    width: u32,
    height: u32,
    /// The mode's refresh in millihertz.
    refresh: u32,
    /// The monitor's image size in millimetres, when its EDID gave one.
    size_mm: Option<(u32, u32)>,
    /// What the engine reads, then the one the next frame fills.
    /// The newest frame, the one before it and the one that shows neither.
    buffers: [Buffer; 3],
    /// For each buffer, the rows that differ from the newest frame: what a
    /// frame filling it must write besides its own damage.
    stale: [Range<usize>; 3],
    /// The hidden buffer holds a frame whose flip is not requested yet.
    queued: bool,
    /// The row a producer fills before it is copied to video memory.
    row: Vec<u8>,
    /// The outputs showing the picture; the first paces the flips and
    /// gives the refresh.
    heads: Vec<Head>,
    /// Every controller of the card.
    crtcs: Vec<u32>,
    /// The connectors not shown, each with whether it was read connected
    /// and tried since it was last read disconnected.
    others: Vec<(u32, bool)>,
    /// Every shown output's controller is off: display sleep.
    dark: bool,
    /// The power [`Self::set_power`] asked for, not yet requested.
    power: Option<bool>,
}

/// Looks at which a shown output connected again is set once more, before
/// it is left dark until it is connected again: a monitor whose EDID is
/// read late, or whose link trains at the second set.
const RESTORE_TRIES: u8 = 3;

/// What [`Display::watch`] read of the connectors since the last read.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Watched {
    /// Shown outputs disconnected.
    pub lost: u32,
    /// Shown outputs connected again, their modes set again.
    pub restored: u32,
    /// Shown outputs connected again, now left dark: the monitor lists no
    /// mode of the display's size.
    pub modeless: u32,
    /// Shown outputs connected again, now left dark: the driver refused
    /// the mode set.
    pub refused: u32,
    /// The first request the driver refused in this read, other outputs
    /// read and set after it.
    pub error: Option<Error>,
    /// Outputs connected that now show the picture too.
    pub joined: u32,
    /// Outputs connected that cannot show it: no mode of its size, or no
    /// free controller or encoder.
    pub unshown: u32,
}

// SAFETY: the buffers' mappings go with the display, which one PE uses at a
// time through the board's lock.
unsafe impl<D: Send> Send for Display<D> {}

/// What [`Display::restore`] did.
enum Restore {
    /// The output runs the picture again.
    Set,
    /// The probe read it disconnected.
    Gone,
    /// It lists no mode of the display's size; its controller is untouched.
    NoMode,
}

/// A complete frame in the hidden buffer. Dropping it leaves the display
/// unchanged. No other frame can be prepared before this one is released.
pub struct PreparedFrame<'a, D>(&'a mut Display<D>);

impl<D: Drm> PreparedFrame<'_, D> {
    /// Queue the prepared frame for the next vertical blank.
    pub fn present(self) -> Result<(), Error> {
        self.0.queue()
    }
}

/// The smallest range of rows holding both `a` and `b`.
fn union(a: Range<usize>, b: Range<usize>) -> Range<usize> {
    match (a.is_empty(), b.is_empty()) {
        (true, _) => b,
        (_, true) => a,
        _ => a.start.min(b.start)..a.end.max(b.end),
    }
}

/// `mode`'s refresh in millihertz: its pixel clock (kHz) over its totals,
/// rounded, as `drm_mode_vrefresh` computes it in hertz; its `vrefresh`
/// when the totals or clock are missing, and 60 Hz without either.
fn refresh(mode: &ModeInfo) -> u32 {
    let total = u64::from(mode.timing_h[2]) * u64::from(mode.timing_v[2]);
    let timed = (total != 0)
        .then(|| (u64::from(mode.clock) * 1_000_000 + total / 2) / total)
        .and_then(|refresh| u32::try_from(refresh).ok());
    [timed, Some(mode.vrefresh.saturating_mul(1000))]
        .into_iter()
        .flatten()
        .find(|&refresh| refresh != 0)
        .unwrap_or(60_000)
}

fn call<D: Drm, T>(drm: &mut D, request: u32, argument: &mut T) -> Result<(), Error> {
    match drm.control(request, ptr::from_mut(argument).cast()) {
        0 => Ok(()),
        error => Err(Error::Request(error)),
    }
}

/// A list a second `control` fills, which Linux does only whole, once the
/// first says how long it is.
fn list<T: Copy + Default>(count: u32) -> Result<Vec<T>, Error> {
    let mut values = Vec::new();
    values
        .try_reserve_exact(count as usize)
        .map_err(|_| Error::Allocation)?;
    values.resize(count as usize, T::default());
    Ok(values)
}

/// A connector: its state, its modes and its encoders.
struct Output {
    connected: bool,
    /// The image size the connector's EDID reports, in millimetres.
    size_mm: [u32; 2],
    modes: Vec<ModeInfo>,
    encoders: Vec<u32>,
    /// `DRM_MODE_CONNECTOR_*` and the connector's number among its kind.
    kind: u32,
    index: u32,
    /// The connector has a current encoder: a controller runs it, as the
    /// driver read back what the firmware lit before any mode set here.
    lit: bool,
}

/// One connector as the driver reports it, for the report of a GPU whose
/// display was not taken.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Seen {
    kind: u32,
    index: u32,
    /// Whether the driver detected a display on it.
    pub connected: bool,
    /// How many modes and encoders it lists.
    pub modes: usize,
    pub encoders: usize,
}

impl core::fmt::Display for Seen {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        // Linux's `drm_connector_enum_list` names.
        const KINDS: [&str; 21] = [
            "Unknown",
            "VGA",
            "DVI-I",
            "DVI-D",
            "DVI-A",
            "Composite",
            "SVIDEO",
            "LVDS",
            "Component",
            "DIN",
            "DP",
            "HDMI-A",
            "HDMI-B",
            "TV",
            "eDP",
            "Virtual",
            "DSI",
            "DPI",
            "Writeback",
            "SPI",
            "USB",
        ];
        let kind = KINDS.get(self.kind as usize).copied().unwrap_or(KINDS[0]);
        let state = match self.connected {
            true => "connected",
            false => "not connected",
        };
        write!(
            f,
            "{kind}-{} {state} modes={} encoders={}",
            self.index, self.modes, self.encoders
        )
    }
}

/// Every connector of `drm`, each detected anew as Linux's forced probe does.
pub fn survey<D: Drm>(drm: &mut D) -> Result<Vec<Seen>, Error> {
    let (_, connectors) = resources(drm)?;
    connectors
        .iter()
        .map(|&id| {
            output(drm, id).map(|found| Seen {
                kind: found.kind,
                index: found.index,
                connected: found.connected,
                modes: found.modes.len(),
                encoders: found.encoders.len(),
            })
        })
        .collect()
}

/// Whether connector `id` has a display as the driver last detected it.
/// Linux probes a connector only for a request that lists no modes
/// (`drm_mode_getconnector`), so this one has room for one mode, which
/// Linux fills only when it is the whole list.
fn detected_on<D: Drm>(drm: &mut D, id: u32) -> Result<bool, Error> {
    let mut mode = ModeInfo::default();
    let mut connector = Connector {
        id,
        modes: ptr::from_mut(&mut mode) as u64,
        mode_count: 1,
        ..Connector::default()
    };
    call(drm, GET_CONNECTOR, &mut connector)?;
    Ok(connector.connection == CONNECTED)
}

/// Whether any connector of `drm` has a display as its driver last
/// detected it, with no probe: cheap enough to ask often, and current where
/// hot-plug interrupts arrive.
pub fn detected<D: Drm>(drm: &mut D) -> Result<bool, Error> {
    let (_, connectors) = resources(drm)?;
    for id in connectors {
        if detected_on(drm, id)? {
            return Ok(true);
        }
    }
    Ok(false)
}

fn output<D: Drm>(drm: &mut D, id: u32) -> Result<Output, Error> {
    for _ in 0..ENUMERATION_ATTEMPTS {
        let mut sized = Connector {
            id,
            ..Connector::default()
        };
        call(drm, GET_CONNECTOR, &mut sized)?;
        let mut modes = list::<ModeInfo>(sized.mode_count)?;
        let mut encoders = list::<u32>(sized.encoder_count)?;
        let mut described = Connector {
            id,
            modes: modes.as_mut_ptr() as u64,
            mode_count: modes.len() as u32,
            encoders: encoders.as_mut_ptr() as u64,
            encoder_count: encoders.len() as u32,
            ..Connector::default()
        };
        call(drm, GET_CONNECTOR, &mut described)?;
        if described.mode_count as usize > modes.len()
            || described.encoder_count as usize > encoders.len()
        {
            continue;
        }
        modes.truncate(described.mode_count as usize);
        encoders.truncate(described.encoder_count as usize);
        return Ok(Output {
            connected: described.connection == CONNECTED,
            size_mm: described.size_mm,
            modes,
            encoders,
            kind: described.kind,
            index: described.kind_id,
            lit: described.encoder != 0,
        });
    }
    Err(Error::Changed)
}

/// The controllers one of `encoders` can drive, a bit for each of the
/// first 32 of the card's `crtcs`.
fn reach<D: Drm>(drm: &mut D, encoders: &[u32], crtcs: &[u32]) -> Result<u32, Error> {
    let present = match crtcs.len() {
        count if count >= u32::BITS as usize => u32::MAX,
        count => (1 << count) - 1,
    };
    let mut reach = 0;
    for &id in encoders {
        let mut encoder = Encoder {
            id,
            ..Encoder::default()
        };
        call(drm, GET_ENCODER, &mut encoder)?;
        reach |= encoder.possible_crtcs;
    }
    Ok(reach & present)
}

/// The first of `crtcs` in `reach` that no head runs.
fn free_crtc(reach: u32, crtcs: &[u32], heads: &[Head]) -> Option<u32> {
    crtcs
        .iter()
        .take(u32::BITS as usize)
        .enumerate()
        .find(|&(index, crtc)| {
            reach >> index & 1 != 0 && heads.iter().all(|head| head.crtc != *crtc)
        })
        .map(|(_, &crtc)| crtc)
}

/// Whether an output of `encoders` can join `heads` without taking an
/// encoder one of them may use: the driver would move it, and that output
/// would go dark.
fn encoders_free(encoders: &[u32], heads: &[Head]) -> bool {
    heads
        .iter()
        .all(|head| head.encoders.iter().all(|used| !encoders.contains(used)))
}

/// Whether whoever shows the output's picture to the guest can describe a
/// mode of this width and height at this refresh in millihertz.
pub type Fits<'a> = &'a dyn Fn(u32, u32, u32) -> bool;

/// Which of an output's modes to take.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Wanted {
    /// The monitor's own preferred mode, its native one, else a mode of
    /// this size: for a display taken before the guest's mode is set. A
    /// firmware often shows a 3840x2160 panel in 1920x1080.
    Native((u32, u32)),
    /// A mode of this size, else the preferred one: the guest's mode is
    /// set, and only that size is shown.
    Size((u32, u32)),
}

fn size_of_mode(mode: &ModeInfo) -> (u32, u32) {
    (u32::from(mode.hdisplay), u32::from(mode.vdisplay))
}

/// `modes`' mode of `size` that `fits`, else its first of that size. Linux
/// lists the faster of each size first, so this is the size's fastest mode
/// the guest can be told of. A monitor's fastest mode may not fit, as
/// 2560x1440 at 180 Hz.
fn mode_of(modes: &[ModeInfo], size: (u32, u32), fits: Fits<'_>) -> Option<ModeInfo> {
    let mut sized = modes.iter().filter(|mode| size_of_mode(mode) == size);
    let fitting = sized
        .clone()
        .find(|mode| fits(size.0, size.1, refresh(mode)));
    fitting.or_else(|| sized.next()).copied()
}

/// The size the connected `outputs`, each its modes, show the picture in,
/// and which of them gives the mode the guest is told of.
///
/// The sizes in order: for [`Wanted::Native`] each output's preferred size,
/// then the firmware's, then every listed size; for [`Wanted::Size`] that
/// size first. A size with a mode the guest can be told of beats one
/// without. Then [`Wanted::Size`]'s size wins, as only it is shown; for
/// [`Wanted::Native`] the size most outputs run wins, so every monitor
/// shows the picture where they share a size, as Linux's fbdev helper
/// clones in a common mode. Earlier sizes win ties. The output that gives
/// the mode is the first with a fitting mode of the size, else the first
/// with one at all.
fn plan(outputs: &[&[ModeInfo]], wanted: Wanted, fits: Fits<'_>) -> Option<((u32, u32), usize)> {
    let preferred = outputs.iter().filter_map(|modes| {
        let native = modes.iter().find(|mode| mode.kind & PREFERRED != 0)?;
        Some(size_of_mode(native))
    });
    let listed = outputs
        .iter()
        .flat_map(|modes| modes.iter().map(size_of_mode));
    let sizes: Vec<(u32, u32)> = match wanted {
        Wanted::Native(firmware) => preferred.chain([firmware]).chain(listed).collect(),
        Wanted::Size(size) => [size].into_iter().chain(preferred).chain(listed).collect(),
    };
    let fitting = |modes: &[ModeInfo], size: (u32, u32)| {
        modes
            .iter()
            .any(|mode| size_of_mode(mode) == size && fits(size.0, size.1, refresh(mode)))
    };
    let covered = |size: (u32, u32)| {
        outputs
            .iter()
            .filter(|modes| modes.iter().any(|mode| size_of_mode(mode) == size))
            .count()
    };
    let (_, &size) = sizes
        .iter()
        .enumerate()
        .filter(|&(_, &size)| covered(size) != 0)
        .max_by_key(|&(order, &size)| {
            let told = outputs.iter().any(|modes| fitting(modes, size));
            let rank = match wanted {
                Wanted::Native(_) => (told, false, covered(size)),
                Wanted::Size(only) => (told, size == only, 0),
            };
            (rank, core::cmp::Reverse(order))
        })?;
    let gives = outputs
        .iter()
        .position(|modes| fitting(modes, size))
        .or_else(|| {
            outputs
                .iter()
                .position(|modes| modes.iter().any(|mode| size_of_mode(mode) == size))
        })?;
    Some((size, gives))
}

/// The mode one output with `modes` runs: [`plan`] for it alone.
#[cfg(test)]
fn choose(modes: &[ModeInfo], wanted: Wanted, fits: Fits<'_>) -> Option<ModeInfo> {
    let (size, _) = plan(&[modes], wanted, fits)?;
    mode_of(modes, size, fits)
}

/// One buffer of the mode's size, with its framebuffer and mapping. A
/// buffer that cannot be finished is given back: a display that is not
/// taken is looked for again, and each look would keep its buffers.
fn buffer<D: Drm>(drm: &mut D, width: u32, height: u32) -> Result<Buffer, Error> {
    let mut dumb = CreateDumb {
        width,
        height,
        bits_per_pixel: BYTES_PER_PIXEL * 8,
        ..CreateDumb::default()
    };
    call(drm, CREATE_DUMB, &mut dumb)?;
    let mut framebuffer = None;
    let finished = finish(drm, width, height, &dumb, &mut framebuffer);
    if finished.is_err() {
        release(drm, dumb.handle, framebuffer, None);
    }
    finished
}

/// The framebuffer and mapping of the dumb buffer `dumb`; `framebuffer` is
/// the one made, for [`buffer`] to give back when a later request fails.
fn finish<D: Drm>(
    drm: &mut D,
    width: u32,
    height: u32,
    dumb: &CreateDumb,
    made: &mut Option<u32>,
) -> Result<Buffer, Error> {
    let row = width
        .checked_mul(BYTES_PER_PIXEL)
        .ok_or(Error::InvalidBuffer)?;
    let bytes = usize::try_from(dumb.size).map_err(|_| Error::InvalidBuffer)?;
    if width == 0
        || height == 0
        || dumb.pitch < row
        || dumb.size < u64::from(dumb.pitch) * u64::from(height)
    {
        return Err(Error::InvalidBuffer);
    }
    let mut framebuffer = FramebufferCommand {
        width,
        height,
        format: XRGB8888,
        ..FramebufferCommand::default()
    };
    framebuffer.handles[0] = dumb.handle;
    framebuffer.pitches[0] = dumb.pitch;
    call(drm, ADD_FB2, &mut framebuffer)?;
    *made = Some(framebuffer.id);
    let mut map = MapDumb {
        handle: dumb.handle,
        ..MapDumb::default()
    };
    call(drm, MAP_DUMB, &mut map)?;
    let pixels = drm.map(map.offset, bytes).ok_or(Error::Map)?;
    // SAFETY: the Drm mapping contract covers `bytes` writable bytes; the
    // buffer has not been given to the display engine yet.
    unsafe { ptr::write_bytes(pixels.as_ptr(), 0, bytes) };
    Ok(Buffer {
        framebuffer: framebuffer.id,
        pixels,
        pitch: dumb.pitch as usize,
        handle: dumb.handle,
        bytes,
    })
}

/// [`release`] each of `buffers`, none of which the controller shows.
fn release_all<D: Drm>(drm: &mut D, buffers: &[Buffer]) {
    for buffer in buffers {
        release(
            drm,
            buffer.handle,
            Some(buffer.framebuffer),
            Some((buffer.pixels, buffer.bytes)),
        );
    }
}

/// Give a dumb buffer back: its mapping, its framebuffer and the buffer
/// itself, in the order Linux needs. A refusal leaves what it names with
/// the node, whose close frees it.
fn release<D: Drm>(
    drm: &mut D,
    handle: u32,
    framebuffer: Option<u32>,
    mapping: Option<(NonNull<u8>, usize)>,
) {
    if let Some((pixels, bytes)) = mapping {
        drm.unmap(pixels, bytes);
    }
    if let Some(mut id) = framebuffer {
        let _ = call(drm, REMOVE_FB, &mut id);
    }
    let _ = call(drm, DESTROY_DUMB, &mut DestroyDumb { handle });
}

fn resources<D: Drm>(drm: &mut D) -> Result<(Vec<u32>, Vec<u32>), Error> {
    for _ in 0..ENUMERATION_ATTEMPTS {
        let mut sized = CardResources::default();
        call(drm, GET_RESOURCES, &mut sized)?;
        let mut crtcs = list::<u32>(sized.crtc_count)?;
        let mut connectors = list::<u32>(sized.connector_count)?;
        let mut described = CardResources {
            crtcs: crtcs.as_mut_ptr() as u64,
            crtc_count: crtcs.len() as u32,
            connectors: connectors.as_mut_ptr() as u64,
            connector_count: connectors.len() as u32,
            ..CardResources::default()
        };
        call(drm, GET_RESOURCES, &mut described)?;
        if described.crtc_count as usize > crtcs.len()
            || described.connector_count as usize > connectors.len()
        {
            continue;
        }
        crtcs.truncate(described.crtc_count as usize);
        connectors.truncate(described.connector_count as usize);
        return Ok((crtcs, connectors));
    }
    Err(Error::Changed)
}

impl<D: Drm> Display<D> {
    /// Takes the connected outputs of `drm`, showing a black frame in the
    /// mode of `wanted` pixels where they have one, as the firmware's did.
    pub fn take(drm: D, wanted: (u32, u32)) -> Result<Self, Error> {
        Self::take_fitting(drm, Wanted::Size(wanted), &|_, _, _| true)
    }

    /// [`Self::take`] in the size [`plan`] picks for [`Wanted`] and `fits`.
    /// Every connected output with a mode of that size, a controller no
    /// other output runs and encoders of its own shows the picture; the
    /// output that gives the guest's mode is set first, and its refusal
    /// stops the take. A refusal of another leaves it dark.
    pub fn take_fitting(mut drm: D, wanted: Wanted, fits: Fits<'_>) -> Result<Self, Error> {
        let (crtcs, connectors) = resources(&mut drm)?;
        let mut usable = Vec::new();
        let mut others = Vec::new();
        for &id in &connectors {
            let found = output(&mut drm, id)?;
            let reach = match found.connected && !found.modes.is_empty() {
                true => reach(&mut drm, &found.encoders, &crtcs)?,
                false => 0,
            };
            match reach {
                0 => others.push((id, found.connected)),
                _ => usable.push((id, found, reach)),
            }
        }
        // What the firmware lit first, then DRM's order.
        usable.sort_by_key(|(_, found, _)| !found.lit);
        let modes: Vec<&[ModeInfo]> = usable.iter().map(|(_, found, _)| &*found.modes).collect();
        let Some((size, first)) = plan(&modes, wanted, fits) else {
            return Err(Error::NoOutput);
        };
        let first = usable.remove(first);
        let [width, height] = first.1.size_mm;
        let mut heads = Vec::new();
        for (id, found, reach) in core::iter::once(first).chain(usable) {
            let fitting: Fits<'_> = match heads.is_empty() {
                true => fits,
                false => &|_, _, _| true,
            };
            let crtc =
                free_crtc(reach, &crtcs, &heads).filter(|_| encoders_free(&found.encoders, &heads));
            match (mode_of(&found.modes, size, fitting), crtc) {
                (Some(mode), Some(crtc)) => heads.push(Head {
                    crtc,
                    connector: id,
                    encoders: found.encoders,
                    mode,
                    lost: false,
                    tries: 0,
                    behind: false,
                }),
                _ => others.push((id, true)),
            }
        }
        let display = Self::run(drm, heads, crtcs, others)?;
        Ok(Self {
            size_mm: (width != 0 && height != 0).then_some((width, height)),
            ..display
        })
    }

    /// Set the first of `heads`, then the others, on the first of three
    /// new buffers of the first's size.
    fn run(
        mut drm: D,
        mut heads: Vec<Head>,
        crtcs: Vec<u32>,
        mut others: Vec<(u32, bool)>,
    ) -> Result<Self, Error> {
        let Some(mode) = heads.first().map(|head| head.mode) else {
            return Err(Error::NoOutput);
        };
        let (width, height) = size_of_mode(&mode);
        let mut made = Vec::new();
        for _ in 0..3 {
            match buffer(&mut drm, width, height) {
                Ok(buffer) => made.push(buffer),
                Err(error) => {
                    release_all(&mut drm, &made);
                    return Err(error);
                }
            }
        }
        let Ok(buffers) = <[Buffer; 3]>::try_from(made) else {
            return Err(Error::Allocation);
        };
        let rest = heads.split_off(1);
        let mut display = Self {
            drm,
            width,
            height,
            refresh: refresh(&mode),
            size_mm: None,
            buffers,
            // Each buffer holds black, and the front the console's text.
            stale: [0..height as usize, 0..height as usize, 0..height as usize],
            queued: false,
            row: Vec::new(),
            heads,
            crtcs,
            others: Vec::new(),
            dark: false,
            power: None,
        };
        if let Err(error) = display.set_mode(0) {
            release_all(&mut display.drm, &display.buffers);
            return Err(error);
        }
        for head in rest {
            display.heads.push(head);
            let last = display.heads.len() - 1;
            if display.set_mode(last).is_err() {
                let head = display.heads.remove(last);
                others.push((head.connector, true));
            }
        }
        display.others = others;
        Ok(display)
    }

    /// Run head `index`'s controller in its mode on its connector, showing
    /// the buffer on screen.
    fn set_mode(&mut self, index: usize) -> Result<(), Error> {
        let head = &self.heads[index];
        let mut connectors = head.connector;
        let mut set = Crtc {
            connectors: ptr::from_mut(&mut connectors) as u64,
            connector_count: 1,
            id: head.crtc,
            framebuffer: self.buffers[0].framebuffer,
            mode_valid: 1,
            mode: head.mode,
            ..Crtc::default()
        };
        call(&mut self.drm, SET_CRTC, &mut set)
    }

    /// Turn head `index`'s controller off.
    fn crtc_off(&mut self, index: usize) -> Result<(), Error> {
        let mut off = Crtc {
            id: self.heads[index].crtc,
            ..Crtc::default()
        };
        call(&mut self.drm, SET_CRTC, &mut off)
    }

    /// Turn every shown output off (`false`), as display sleep does, or on
    /// again (`true`) in its mode on the buffer on screen, the last frame
    /// flipped to. Where a request must wait, this is left to the next
    /// [`Self::flush`]. While off, frames still fill the hidden buffer and
    /// queue, and the first flush after the outputs are on again flips the
    /// newest. A refused request is answered; the other outputs are still
    /// set, and an output that cannot be lit again is left lost for the
    /// next [`Self::watch`] to set once more.
    pub fn set_power(&mut self, on: bool) -> Result<(), Error> {
        self.power = Some(on);
        self.apply_power()
    }

    fn apply_power(&mut self) -> Result<(), Error> {
        let Some(on) = self.power else {
            return Ok(());
        };
        if !self.drm.immediate() {
            return Ok(());
        }
        self.power = None;
        if on != self.dark {
            return Ok(());
        }
        self.dark = !on;
        let mut applied = Ok(());
        for index in 0..self.heads.len() {
            if self.heads[index].lost {
                continue;
            }
            let result = match on {
                true => self.set_mode(index),
                false => self.crtc_off(index),
            };
            if let Err(error) = result {
                if on {
                    self.heads[index].lost = true;
                    self.heads[index].tries = 0;
                }
                if applied.is_ok() {
                    applied = Err(error);
                }
            }
        }
        // Every lit output shows the buffer on screen now.
        for head in &mut self.heads {
            head.behind = false;
        }
        applied
    }

    /// Read the connectors as the driver last detected them, with no probe,
    /// where a request completes now.
    ///
    /// A shown output connected again after a read found it disconnected
    /// is probed, as the monitor may be another one, and set in a mode of
    /// the display's size from its new list: the mode it ran where the list
    /// still has it, else one of that size at the same refresh, else the
    /// size's first. Its controller is turned off first: a full mode set,
    /// which trains a DisplayPort link anew, where an unchanged one would
    /// change nothing. With no mode of the size the controller is not
    /// touched. Either failure leaves it lost, tried at the next reads up
    /// to [`RESTORE_TRIES`] looks, then counted once and left dark until it
    /// is connected again.
    ///
    /// An output not shown, connected since it was last read disconnected
    /// or since the take, is probed once and joins the picture in its mode
    /// of the display's size on a free controller, else stays dark until it
    /// is connected again.
    ///
    /// A refused request stops only the output it was for: the others are
    /// still read and set, and the first refusal is in [`Watched::error`].
    pub fn watch(&mut self) -> Watched {
        let mut watched = Watched::default();
        if !self.drm.immediate() {
            return watched;
        }
        for index in 0..self.heads.len() {
            let connected = match detected_on(&mut self.drm, self.heads[index].connector) {
                Ok(connected) => connected,
                Err(error) => {
                    watched.error.get_or_insert(error);
                    continue;
                }
            };
            let head = &mut self.heads[index];
            match (connected, head.lost) {
                (false, false) => {
                    head.lost = true;
                    head.tries = 0;
                    watched.lost += 1;
                }
                (false, true) => head.tries = 0,
                // While the outputs are off, a returning one is lit with
                // the others ([`Self::set_power`]), and set here after.
                (true, true) if !self.dark && head.tries < RESTORE_TRIES => {
                    head.tries += 1;
                    let last = head.tries == RESTORE_TRIES;
                    match self.restore(index, &mut watched) {
                        Ok(Restore::Set) => {
                            self.heads[index].lost = false;
                            watched.restored += 1;
                        }
                        // Read disconnected after all: the next read that
                        // finds it connected tries again.
                        Ok(Restore::Gone) => self.heads[index].tries = 0,
                        Ok(Restore::NoMode) => watched.modeless += u32::from(last),
                        Err(error) => {
                            watched.error.get_or_insert(error);
                            watched.refused += u32::from(last);
                        }
                    }
                }
                _ => {}
            }
        }
        let mut index = 0;
        while let Some(&(id, tried)) = self.others.get(index) {
            index += 1;
            match detected_on(&mut self.drm, id) {
                Ok(true) => {}
                Ok(false) => {
                    self.others[index - 1].1 = false;
                    continue;
                }
                Err(error) => {
                    watched.error.get_or_insert(error);
                    continue;
                }
            }
            if tried || self.dark {
                continue;
            }
            self.others[index - 1].1 = true;
            match self.join(id) {
                Ok(true) => {
                    self.others.remove(index - 1);
                    index -= 1;
                    watched.joined += 1;
                }
                Ok(false) => watched.unshown += 1,
                Err(error) => {
                    watched.error.get_or_insert(error);
                    watched.unshown += 1;
                }
            }
        }
        watched
    }

    /// Set head `index` again on its connector as its driver reads it now
    /// ([`Self::watch`]).
    fn restore(&mut self, index: usize, watched: &mut Watched) -> Result<Restore, Error> {
        let found = output(&mut self.drm, self.heads[index].connector)?;
        if !found.connected {
            return Ok(Restore::Gone);
        }
        let ran = self.heads[index].mode;
        let size = (self.width, self.height);
        let mode = found
            .modes
            .iter()
            .find(|&&mode| mode == ran)
            .or_else(|| {
                found
                    .modes
                    .iter()
                    .find(|mode| size_of_mode(mode) == size && refresh(mode) == refresh(&ran))
            })
            .copied()
            .or_else(|| mode_of(&found.modes, size, &|_, _, _| true));
        let Some(mode) = mode else {
            return Ok(Restore::NoMode);
        };
        self.heads[index].mode = mode;
        // A refused off still leaves the set to try.
        if let Err(error) = self.crtc_off(index) {
            watched.error.get_or_insert(error);
        }
        self.set_mode(index)?;
        Ok(Restore::Set)
    }

    /// Show the picture on connector `id` as well, where it has a mode of
    /// the display's size, a free controller and encoders of its own.
    fn join(&mut self, id: u32) -> Result<bool, Error> {
        let found = output(&mut self.drm, id)?;
        let reach = reach(&mut self.drm, &found.encoders, &self.crtcs)?;
        let mode = mode_of(&found.modes, (self.width, self.height), &|_, _, _| true);
        let crtc = free_crtc(reach, &self.crtcs, &self.heads)
            .filter(|_| encoders_free(&found.encoders, &self.heads));
        let (Some(mode), Some(crtc), true) = (mode, crtc, found.connected) else {
            return Ok(false);
        };
        self.heads.push(Head {
            crtc,
            connector: id,
            encoders: found.encoders,
            mode,
            lost: false,
            tries: 0,
            behind: false,
        });
        let last = self.heads.len() - 1;
        if self.set_mode(last).is_err() {
            self.heads.remove(last);
            return Ok(false);
        }
        Ok(true)
    }

    /// Give the outputs back: each controller turned off, then the buffers,
    /// their mappings and framebuffers, and the node, for its owner to
    /// close. A refused request leaves what it names with the node, whose
    /// close frees it.
    pub fn release(mut self) -> D {
        for index in 0..self.heads.len() {
            let _ = self.crtc_off(index);
        }
        release_all(&mut self.drm, &self.buffers);
        self.drm
    }

    /// How many outputs show the picture.
    pub fn outputs(&self) -> usize {
        self.heads.len()
    }

    /// How many connected outputs do not show it: no mode of its size, or
    /// no free controller or encoder, or a refused mode set.
    pub fn unshown(&self) -> usize {
        self.others.iter().filter(|(_, tried)| *tried).count()
    }

    /// Pixels across and down.
    pub fn size(&self) -> (u32, u32) {
        (self.width, self.height)
    }

    /// The mode's refresh in millihertz.
    pub fn refresh_millihertz(&self) -> u32 {
        self.refresh
    }

    /// The monitor's image size in millimetres, when its EDID gave one.
    pub fn size_mm(&self) -> Option<(u32, u32)> {
        self.size_mm
    }

    /// The buffer on screen and the bytes between its rows, for what draws
    /// before the first frame.
    pub fn front(&self) -> (NonNull<u8>, usize) {
        (self.buffers[0].pixels, self.buffers[0].pitch)
    }

    /// Shows `rows`, blue-low pixels of [`Self::size`] top to bottom, at the
    /// next vertical blank.
    pub fn present(&mut self, rows: &[u8]) -> Result<(), Error> {
        let row = self.width as usize * BYTES_PER_PIXEL as usize;
        let expected = row * self.height as usize;
        if rows.len() != expected {
            return Err(Error::FrameSize {
                expected,
                actual: rows.len(),
            });
        }
        let every = 0..self.height as usize;
        for stale in &mut self.stale {
            *stale = every.clone();
        }
        for (y, line) in rows.chunks_exact(row).enumerate() {
            self.copy_hidden_row(y, line);
        }
        self.stale[2] = 0..0;
        self.queue()
    }

    /// Fill the hidden buffer one row at a time: the rows `damage` names
    /// (`None`: every row) and those the buffer lacks since it last held a
    /// frame. The producer works in ordinary RAM so inspecting or changing
    /// pixels never reads VRAM. A failed row leaves the current scanout and
    /// buffer order unchanged.
    pub fn prepare_rows(
        &mut self,
        damage: Option<Range<usize>>,
        mut fill: impl FnMut(usize, &mut [u8]) -> Result<(), Refusal>,
    ) -> Result<PreparedFrame<'_, D>, Refusal> {
        let height = self.height as usize;
        let damage = damage.map_or(0..height, |rows| {
            rows.start.min(height)..rows.end.min(height)
        });
        for stale in &mut self.stale {
            *stale = union(stale.clone(), damage.clone());
        }
        let bytes = self.width as usize * BYTES_PER_PIXEL as usize;
        let mut row = core::mem::take(&mut self.row);
        if row.len() != bytes {
            row.clear();
            row.try_reserve_exact(bytes)
                .map_err(|_| refuse!("could not allocate a scanout row"))?;
            row.resize(bytes, 0);
        }
        let mut filled = Ok(());
        for y in self.stale[2].clone() {
            filled = fill(y, &mut row);
            if filled.is_err() {
                break;
            }
            self.copy_hidden_row(y, &row);
        }
        self.row = row;
        filled?;
        self.stale[2] = 0..0;
        Ok(PreparedFrame(self))
    }

    /// Whether a frame waits in the hidden buffer for its flip to be
    /// requested, or an output that follows the first waits for its flip
    /// to the newest buffer ([`Self::flush`]).
    pub fn queued(&self) -> bool {
        self.queued || self.power.is_some() || self.heads.iter().any(|head| head.behind)
    }

    /// Request the queued frame's flip on the first output, if this
    /// processor may now, then each other output's flip to the newest
    /// buffer. A flip the engine is still busy with stays queued for the
    /// next flush. A refused flip drops the request, reported here: on the
    /// first output the frame stays in the hidden buffer and shows with the
    /// next one; another output keeps its picture until the next frame.
    pub fn flush(&mut self) -> Result<(), Error> {
        if !self.queued() || !self.drm.immediate() {
            return Ok(());
        }
        let mut flushed = self.apply_power();
        // An output that is off takes no flip: the newest frame waits.
        if self.dark {
            return flushed;
        }
        if self.queued {
            match self.flip(0, self.buffers[2].framebuffer) {
                0 => {
                    self.queued = false;
                    self.buffers.rotate_right(1);
                    self.stale.rotate_right(1);
                    for head in &mut self.heads[1..] {
                        head.behind = true;
                    }
                }
                BUSY => {}
                error => {
                    self.queued = false;
                    flushed = Err(Error::Request(error));
                }
            }
        }
        for index in 1..self.heads.len() {
            if !self.heads[index].behind {
                continue;
            }
            match self.flip(index, self.buffers[0].framebuffer) {
                BUSY => {}
                result => {
                    self.heads[index].behind = false;
                    if result != 0 && flushed.is_ok() {
                        flushed = Err(Error::Request(result));
                    }
                }
            }
        }
        flushed
    }

    /// Request head `index`'s flip to `framebuffer`: zero or a negative
    /// errno.
    fn flip(&mut self, index: usize, framebuffer: u32) -> i64 {
        let mut flip = PageFlip {
            crtc: self.heads[index].crtc,
            framebuffer,
            ..PageFlip::default()
        };
        self.drm.control(PAGE_FLIP, ptr::from_mut(&mut flip).cast())
    }

    fn queue(&mut self) -> Result<(), Error> {
        self.queued = true;
        self.flush()
    }

    fn copy_hidden_row(&mut self, y: usize, row: &[u8]) {
        let target = &self.buffers[2];
        // SAFETY: callers give one full row and a valid row index. The
        // mapping's pitch holds that row, and the engine reads a different
        // buffer.
        unsafe {
            ptr::copy_nonoverlapping(
                row.as_ptr(),
                target.pixels.as_ptr().add(y * target.pitch),
                row.len(),
            );
        }
    }
}

#[cfg(test)]
mod tests;
