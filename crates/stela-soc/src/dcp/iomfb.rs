//! The IOMFB method service of the modeled DCP for one fixed, connected SDR
//! monitor: the replies to the methods the AP's IOMobileFramebuffer stack
//! calls during boot, power, mode set and swap, and the callbacks the
//! coprocessor makes. Every call is first checked against its interface row
//! ([`super::abi`]); a tag outside the interface and a call whose sizes differ
//! from its row refuse by name. A method of the interface the model does not
//! serve must still be answered, as a refusal would leave the AP's call open
//! for good. It is reported once (an `UNMODELED` line through
//! [`Methods::take_unmodeled`]) and answered degraded ([`Degraded`]): a
//! method whose AP stub reads a link status gets the protocol's
//! `kIOReturnUnsupported` ([`abi::LINK_UNSUPPORTED`]), its in-out argument
//! returned as it came; any other answers zeros, which is no model of it.
//!
//! Each method's first call is logged; every call only under swap tracing
//! (`--display-trace`), and a refused call always, before its refusal.
//!
//! The boot sequence follows Asahi Linux's AP side of the same protocol
//! (`drivers/gpu/drm/apple/iomfb_template.c`: `boot_1` .. `boot_done`,
//! `iomfb_poweron`, the swap callbacks) and m1n1's method signatures
//! (`proxyclient/m1n1/fw/dcp/ipc.py`), with this interface's tags and sizes.
//! The mode, hotplug and swap layouts are the 26A434 AP's own (kernel
//! fileset offsets, cited per method).
//!
//! `A401 start_signal` nests `D120 start_hardware_boot` into the AP before it
//! answers, as the firmware does (the AP's D120 handler,
//! `UnifiedPipeline2::start_hardware_boot` 0x2a5d7cc, calls
//! `set_create_DFB`, `do_create_default_frame_buffer`, the config frame size
//! methods, `flush_supportsPower`, `apt_supported`, `late_init_signal`,
//! `headless` and `setDisplayRefreshProperties` back, which re-enter
//! [`Methods::call`]). The model inherits the boot framebuffer rather than
//! allocating a new default one: `A454 do_create_default_frame_buffer`
//! answers 0. The pipe's config frame size starts unset (`A103` answers
//! 0 x 0), so the AP's `start_hardware_boot` sets it to the display size
//! (`A104`) and programs it (`A105`).
//!
//! Once started, the coprocessor publishes its one mode on the IOMFB service
//! (`D126`..`D128 setDCPAVPropStart/Chunk/End` with the keys `TimingElements`
//! and `ColorElements`, [`super::modes`]) and then reports the monitor
//! connected (`D575 hotPlug_notify_gated`, state 1). Its callbacks leave one
//! at a time, each after the AP answered the previous one. The mode set is
//! `A364 getVideoDataForIDs` then `A413 set_digital_out_mode` with the
//! published IDs.
//!
//! A swap is `A406 swap_start_dcp` (the coprocessor assigns the swap id)
//! then `A407 swap_submit_dcp` ([`super::swap`]). The submitted layers
//! update the four layer slots; the visible layers are queued as a
//! [`Frame`]; the coprocessor reads their pixels through the display pipe's
//! DART, composes and presents them and only then completes the swap with
//! `D594 swap_complete_ap_gated` ([`super::Coprocessor::present`]).
//!
//! Timing follows a display controller's: the monitor refreshes on a fixed
//! grid of vertical blanks ([`Methods::refresh`]), and a swap is latched at
//! one. At each vblank the swaps whose GPU fences have all signaled, taken
//! in submission order up to the first still waiting, are latched together:
//! the newest is shown, since its layer state already holds every earlier
//! one's, and all of them complete after it, in order. At most one frame is
//! presented per refresh, however late the model looks or however many
//! swaps a slow frame held back, so a backlog clears at one vblank rather
//! than one per device step. An earlier swap of the batch completes as
//! displayed unless a later one replaced every layer it set and none of its
//! surfaces is on screen: then it completes as not displayed and the AP
//! releases its surfaces at once (`SWAP_COMPLETE_DISPLAYED`). Every swap the
//! AP submitted is completed; none is dropped. The AP bounds what it queues
//! itself: in the runs measured, WindowServer kept at most two of its own
//! swaps open, while its cursor client (layer 1 only, one surface moved)
//! kept submitting behind a fenced frame, 19 deep at most, until it
//! cleared. The vsync callback
//! `D588` carries the vblank's own counter value, not the time the model
//! looked.
//!
//! Composition (Inferred: the coprocessor's compositor is undocumented):
//! layers 0..3 stack bottom to top over opaque black, each a source
//! rectangle of its surface scaled nearest-neighbour into its destination
//! rectangle, clipped to the screen, and blended source-over with the
//! premultiplied flag the AP sets per surface. This agrees with Linux
//! (`plane.c`), which marks premultiplied only its formats without alpha
//! (XRGB, XBGR, XBGR2101010) and straight its ARGB ones: premultiplied over
//! black, an X byte of any value shows the colour unchanged.
use super::super::scanout::{self, Mode, Scanout};
use super::Log;
use super::abi::{self, Fields, IO_RETURN_SUCCESS, Interface, LINK_UNSUPPORTED, Method, Packed};
use super::avipc::DisplayState;
use super::compose::{Compose, Composition, Pages};
use super::link::SHARED_VARIABLES;
use super::modes::{self, Advertised};
use super::rpc::{Callback, Service, Step};
use super::swap::{self, Layer, Submission};
use crate::Result;
use crate::handoff::Handoff;
use alloc::boxed::Box;
use alloc::collections::{BTreeSet, VecDeque};
use alloc::format;
use alloc::rc::Rc;
use alloc::string::String;
use alloc::string::ToString;
use alloc::sync::Arc;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
use core::cell::RefCell;
use stela_contracts::Refusal;
use stela_contracts::refuse;

/// The classes whose methods the model answers.
const AP: &str = "IOMobileFramebufferAP";
const UP: &str = "UnifiedPipeline2";
const H13P: &str = "UPPipeAP_H13P";
const PIPE2: &str = "UPPipe2";
const POWER: &str = "DCPPowerManager";

/// The most display-log entries kept between drains.
const LOG_CAPACITY: usize = 4096;
/// The leading call-input bytes a log entry keeps.
const LOGGED_INPUT: usize = 64;
/// How many `swap_submit_dcp` inputs are logged whole.
const DUMPED_SWAPS: u32 = 2;
/// How many new layer shapes are logged ([`Log::Placement`]).
const PLACEMENTS: u32 = 64;
/// How many distinct swap surface layouts are reported and remembered; one
/// line says when later ones are left out.
const LAYOUTS: usize = 64;
/// How many degraded answers, by method and parameter, are reported and
/// remembered; one line says when later ones are left out.
const UNMODELED_REPORTS: usize = 256;
/// The submitted swaps the model holds for presentation: past this many a
/// submit refuses. The deepest queue measured was 32.
const QUEUED_SWAPS: usize = 256;
/// Report progress once this many swaps have reached the scanout.
const PRESENT_SUMMARY_INTERVAL: u64 = 256;
/// The started swaps kept for submission. A client that dies between
/// `swap_start_dcp` and its submit leaves its swap started for good, so the
/// oldest is forgotten past this many (inferred: no swap ID ever revives).
const STARTED_SWAPS: usize = 16;
/// How long a swap may wait for its surfaces' GPU fences before it is shown
/// as it stands: the AP normally names each fenced surface within a frame or
/// two of its GPU work completing.
const FENCE_TIMEOUT: u64 = 2 * COUNTER_HZ;
/// `D594 swap_complete_ap_gated`'s input starts with the swap ID. Its
/// statistics pointer is absent. The fixed eight-blob storage remains zero
/// with count zero; the original dispatcher passes that array unconditionally
/// and has no independent blob-null flag. The sole null flag is at +0x72c;
/// +0x72d..+0x72f is padding in the complete 0x730-byte record.
const SWAP_COMPLETE: [u8; 4] = *b"D594";
/// `swap_notify_gated(u64, u64, u64)`: the refresh tick the AP forwards to every
/// registered client, with no reply. It fills in the other two values itself
/// when they are zero, and Core Animation stops presenting without it.
const SWAP_NOTIFY: [u8; 4] = *b"D588";
/// The ticks per second of the counter refreshes are timed by.
const COUNTER_HZ: u64 = crate::counter::FREQUENCY_HZ;

/// The counter ticks each pacing report covers.
const PACING_WINDOW: u64 = 10 * COUNTER_HZ;

/// How swaps reached the scanout over a window of the counter: their rate,
/// the gaps between them, how many waited at once and whether any left out of
/// submission order.
#[derive(Clone, Debug, Default, PartialEq, Eq)]
pub struct Pacing {
    /// Counter ticks the window covers: from the vblank that opened it to
    /// the one that closed it, at least [`PACING_WINDOW`].
    pub ticks: u64,
    /// Frames presented (one per vblank at most) and swaps submitted in the
    /// window.
    pub presented: u32,
    pub submitted: u32,
    /// The shortest, total and longest gaps between the vblanks of
    /// successive presents, in ticks: whole refresh periods.
    pub shortest: u64,
    pub total_gap: u64,
    pub longest: u64,
    /// Gaps longer than two refresh periods: frames the display repeated.
    pub stalls: u32,
    /// The most swaps waiting at a present, the presented one included.
    pub deepest: usize,
    /// Swaps presented after one the AP submitted later.
    pub reordered: u32,
    /// Swaps presented before the GPU completed the work submitted ahead of
    /// them, which may still have been drawing their surfaces.
    pub ahead: u32,
    /// Swaps that waited for a surface's GPU fence, and those shown after
    /// [`FENCE_TIMEOUT`] without one.
    pub fence_waits: u32,
    pub fence_timeouts: u32,
    /// Method calls the AP made in the window, logged or not.
    pub calls: u32,
    /// Calls answered with the protocol's unsupported status or with
    /// zeros: a method that repeats per frame shows here.
    pub unmodeled: u32,
    /// `swap_set_color_matrix` calls.
    pub color_matrix: u32,
    /// Swaps completed at a vblank with a newer one shown in their place.
    pub merged: u32,
    /// How long after its vblank each present ran, in ticks: the total and
    /// the longest. A present is due at its vblank; the model acts when
    /// the counter is next looked at.
    pub late_total: u64,
    pub late_longest: u64,
    /// The gaps measured: presents after the first one the display made.
    pub gaps: u32,
    /// Presents that moved a layer: one whose destination kept its size at
    /// another place, as a pointer's does ([`moved_presents`]).
    pub moved: u32,
    /// Where a swap's time goes, in counter ticks, so a slow frame rate can
    /// be told apart from a late display: `fence`, from its submit to the
    /// notice of its last GPU fence (swaps that waited for one); `notice`,
    /// from the time `io_fence_notify` names (the AP's clock) to the model
    /// receiving it; `latch`, from ready (fences done, or submitted
    /// unfenced) to the present that showed or merged it; `complete`, from
    /// that present to the AP's answer to its `swap_complete_ap_gated`;
    /// `total`, from submit to that answer.
    pub fence: Latency,
    pub notice: Latency,
    pub latch: Latency,
    pub complete: Latency,
    pub total: Latency,
    /// Vsyncs (`D588`) the notification clients answered, each from its
    /// vblank to the answer: the frame clock Core Animation paces by.
    pub vsync: Latency,
}

/// A count of intervals, their total and the longest, in counter ticks.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Latency {
    pub count: u32,
    pub total: u64,
    pub longest: u64,
}

impl Latency {
    fn add(&mut self, ticks: u64) {
        self.count = self.count.saturating_add(1);
        self.total = self.total.saturating_add(ticks);
        self.longest = self.longest.max(ticks);
    }
}

impl core::fmt::Display for Latency {
    /// `count:average/longest`, in milliseconds.
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let ms = |ticks: u64| ticks as f64 * 1000.0 / COUNTER_HZ as f64;
        write!(
            f,
            "{}:{:.1}/{:.1}",
            self.count,
            ms(self.total) / f64::from(self.count.max(1)),
            ms(self.longest)
        )
    }
}

/// Presents that moved a layer since boot, for the host to time against its
/// own clock: how often the pointer moves on screen, and how long after a
/// host input.
static MOVED: core::sync::atomic::AtomicU64 = core::sync::atomic::AtomicU64::new(0);

/// Presents that moved a layer since boot ([`Pacing::moved`]).
pub fn moved_presents() -> u64 {
    MOVED.load(core::sync::atomic::Ordering::Acquire)
}

/// Whether `next` shows a layer of `shown` at another place with the same
/// size: a moved pointer, or any layer moved whole.
fn moved(
    shown: &[Option<scanout::Layer>; swap::LAYERS],
    next: &[Option<scanout::Layer>; swap::LAYERS],
) -> bool {
    shown.iter().zip(next).any(|pair| match pair {
        (Some(shown), Some(next)) => {
            let (a, b) = (shown.destination, next.destination);
            (a.width, a.height) == (b.width, b.height) && (a.x, a.y) != (b.x, b.y)
        }
        _ => false,
    })
}

impl Pacing {
    fn gap(&mut self, gap: u64, refresh: u64) {
        self.shortest = if self.gaps == 0 {
            gap
        } else {
            self.shortest.min(gap)
        };
        self.gaps += 1;
        self.total_gap += gap;
        self.longest = self.longest.max(gap);
        self.stalls += u32::from(gap > 2 * refresh);
    }
}

impl core::fmt::Display for Pacing {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        let ms = |ticks: u64| ticks as f64 * 1000.0 / COUNTER_HZ as f64;
        write!(
            f,
            "DCP PACING seconds={:.1} presented={} submitted={} merged={} per_second={:.1} gap_ms={:.1}/{:.1}/{:.1} stalls={} late_ms={:.1}/{:.1} deepest={} reordered={} ahead_of_gpu={} fence_waits={} fence_timeouts={} calls={} unmodeled={} color_matrix={} moved={} swap_ms=n:avg/max fence={} notice={} latch={} complete={} total={} vsync={}",
            ms(self.ticks) / 1000.0,
            self.presented,
            self.submitted,
            self.merged,
            if self.ticks == 0 {
                0.0
            } else {
                f64::from(self.presented) * 1000.0 / ms(self.ticks)
            },
            ms(self.shortest),
            ms(self.total_gap) / f64::from(self.gaps.max(1)),
            ms(self.longest),
            self.stalls,
            ms(self.late_total) / f64::from(self.presented.max(1)),
            ms(self.late_longest),
            self.deepest,
            self.reordered,
            self.ahead,
            self.fence_waits,
            self.fence_timeouts,
            self.calls,
            self.unmodeled,
            self.color_matrix,
            self.moved,
            self.fence,
            self.notice,
            self.latch,
            self.complete,
            self.total,
            self.vsync,
        )
    }
}

/// How a method the model does not serve in full was answered.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Degraded {
    /// No reply layout is recorded for the method: zeros, which a status
    /// word would read as success.
    Zeros,
    /// The protocol's error reply, `kIOReturnUnsupported`
    /// ([`abi::LINK_UNSUPPORTED`]).
    Unsupported,
    /// Success with the value ignored, because the AP's caller retries a
    /// failure forever; the text says what is shown instead.
    Ignored(&'static str),
}

impl core::fmt::Display for Degraded {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self {
            Degraded::Zeros => {
                write!(f, "answered with zeros; no reply layout is recorded")
            }
            Degraded::Unsupported => {
                write!(f, "answered kIOReturnUnsupported (link status 2)")
            }
            Degraded::Ignored(shown) => {
                write!(
                    f,
                    "answered success, value ignored (the caller retries a failure forever): {shown}"
                )
            }
        }
    }
}

/// The refresh period of `refresh` millihertz in counter ticks, rounded.
fn refresh_ticks(refresh: u32) -> u64 {
    let refresh = u64::from(refresh.max(1));
    (COUNTER_HZ * 1000 + refresh / 2) / refresh
}
/// Original T6000 `D594_callback__` (container file 0x3daec14) reads this
/// flag at VA 0xfffffe000adb2cec and passes nullptr for bit0=1. The AP
/// skips absent statistics at 0xfffffe000ad9cb68 and still completes the
/// presented swap. A present record copies its timestamp at +0x19.
const SWAP_COMPLETE_DATA_NULL: usize = 0x72c;
/// J473's `D594_callback__` (0x3d89f14) passes bit 0 of +4 as
/// `swap_complete_ap_gated`'s (0x3d73c9c) bool: the swap reached the
/// display. Only then does the AP record it as displayed
/// (`updateDisplayedDataFromPendingSwap_gated` 0x3d7344c), releasing the
/// surfaces it replaces. Otherwise it releases the swap's own surfaces at
/// once, while a layer it left unchanged still shows them.
const SWAP_COMPLETE_DISPLAYED: usize = 4;
/// The property-publishing callbacks (`UnifiedPipeline_RemoteCalls::
/// D126_callback__` 0x2a71628, `D127` 0x2a71730, `D128` 0x2a718b4).
const PROP_START: [u8; 4] = *b"D126";
const PROP_CHUNK: [u8; 4] = *b"D127";
const PROP_END: [u8; 4] = *b"D128";
/// `D127`'s chunk: `data[0x1000]` then `u32 offset`, `u32 length`.
const CHUNK_BYTES: usize = 0x1000;
/// `D128`'s key: `char[0x40]`, NUL-terminated by the AP.
const KEY_BYTES: usize = 0x40;
/// `IOMobileFramebuffer_RemoteCalls::D575_callback__` (0x3d88bb8):
/// `hotPlug_notify_gated(u64 state, IOMFB_TiledDisplayInfo* (0x4b bytes,
/// in-out), bool)`.
const HOTPLUG: [u8; 4] = *b"D575";
const TILED_DISPLAY_INFO: usize = 0x4b;
/// `hotPlug_notify_gated`'s plugged state (`IOMobileFramebufferShim::
/// hotPlug_notify_gated` 0x2a613c0 takes 0 as unplugged).
const PLUGGED: u64 = 1;
/// `get_digital_out_state`'s state for a connected display:
/// `restored_external` uses an external display only when it reads 1
/// ("Will use display %s").
const DIGITAL_OUT_CONNECTED: u32 = 1;
/// `set_matrix`'s `u64[3][3]` holds 1.0 as 1 << 32 (Inferred: the AP's
/// boot call sends that diagonal and zeros elsewhere, the identity it opens
/// every display with).
const MATRIX_ONE: u64 = 1 << 32;
/// `IOMFBColorFixedMatrix`, the 0x78 bytes `swap_set_color_matrix` carries
/// (AP stub 0xfffffe000ada4718): 15 values in the same 1 << 32 fixed point.
/// `IOMFB::MatrixHandler::set` (26B123 0xfffffe0009ae8b38) reads +0x00 and
/// +0x60 as two offset triples and +0x18 as the 3x3 coefficients, row-major.
const COLOR_MATRIX_VALUES: usize = 15;
const COLOR_MATRIX_FIRST_COEFFICIENT: usize = 3;
/// `IOMFBColorMatrixFunction`s with a matrix location: `AppleDisplayPipe::
/// swap_set_color_matrix_gated` (0xfffffe0009aafc68) maps 0, 1, 2 to
/// locations 2, 1, 3 and answers `kIOReturnUnsupported` past them.
const COLOR_MATRIX_FUNCTIONS: u32 = 3;
/// `setBrightnessCorrection`'s uncorrected value, 1.0 in 16.16: Apple's
/// on-SoC panel driver (`AppleCLCD::setBrightnessCorrection_gated`, 26B123
/// VA 0xfffffe0009a93174) refuses any larger value with
/// `kIOReturnBadArgument`, and the AP sends it at boot.
const BRIGHTNESS_UNCORRECTED: u32 = 0x10000;

/// A submitted swap waiting for its pixels: its id and its layers, none for
/// a swap that only clears to the background.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Frame {
    pub swap_id: u32,
    /// The IOMFB client that started the swap (`swap_start_dcp`'s u64).
    pub client: u64,
    /// The GPU's last submitted command buffer when the swap was submitted.
    pub gpu_work: u64,
    /// Layers whose surface the GPU may still be drawing, each waiting for
    /// `io_fence_notify` to name its surface, and the layers' surface IDs.
    pub fenced: u32,
    pub surface_ids: [u32; swap::LAYERS],
    /// The counter when the display first found this swap waiting on a
    /// fence.
    pub waiting_since: Option<u64>,
    /// The counter when the AP submitted the swap, and when it became ready
    /// to show: at submit when nothing was fenced, else at the last fence's
    /// notice (or the fence timeout).
    pub submitted_at: u64,
    pub ready_at: Option<u64>,
    /// The layer slots after this swap: what each shows, if anything.
    pub planes: [Option<scanout::Layer>; swap::LAYERS],
    /// The slots this swap set, whether to a surface or to nothing.
    pub updated: u32,
}

impl Frame {
    /// The visible layers, bottom to top.
    pub fn layers(&self) -> Vec<scanout::Layer> {
        self.planes.iter().flatten().copied().collect()
    }
}

/// A callback the coprocessor makes on its own initiative.
struct Outgoing {
    tag: [u8; 4],
    input: Vec<u8>,
    output: u32,
}

/// The spin-loop hints a call that drops swaps waits for a compose in
/// flight ([`Methods::settle_compose`]): far longer than a compose takes
/// (milliseconds), short enough that the devices' lock is not held for good.
const SETTLE_SPINS: u64 = 1 << 26;

/// What a vblank latched ([`Methods::batch`]): its counter and the count
/// at the look that took it, the swaps it shows (the oldest `count`), the
/// newest one's layer slots and layers, and the rows that may change.
struct Batch {
    now: u64,
    due: u64,
    count: usize,
    planes: [Option<scanout::Layer>; swap::LAYERS],
    layers: Vec<scanout::Layer>,
    damage: Option<core::ops::Range<usize>>,
}

/// The IOMFB methods of the one connected display, presenting to a host
/// scanout.
pub struct Methods {
    /// The booted build's method interface: which tag names which method.
    interface: &'static Interface,
    /// The host scanout, `None` while a compose outside the devices' lock
    /// has it ([`super::compose`]).
    scanout: Option<Box<dyn Scanout>>,
    /// The scanout's fixed mode.
    mode: Mode,
    /// The storage of the last composed frame, for the next
    /// ([`scanout::present`]); taken with the scanout.
    frame: scanout::Composed,
    /// The present whose compose runs outside the devices' lock, and what
    /// it hands back.
    composing: Option<(Batch, Arc<Handoff<Composition>>)>,
    /// A compose read past its resolved pages: the next present is made in
    /// place.
    in_place: bool,
    /// A compose outside the lock outlasted [`SETTLE_SPINS`] (reported once).
    compose_stall_reported: bool,
    /// The layer slots on screen since the last present, `None` before it.
    shown: Option<[Option<scanout::Layer>; swap::LAYERS]>,
    /// Slots that swaps dropped without a present (an abort or a shutdown)
    /// had set: their surfaces may differ from what is on screen.
    unshown: u32,
    /// The one mode, or why the scanout's mode cannot be advertised.
    advertised: Result<Advertised>,
    /// Why the last call refused, with the tag and its interface row.
    last_refusal: Option<String>,
    /// The interface's methods answered without a model, each reported once
    /// per method and, where the method names one, per parameter.
    unmodeled: BTreeSet<([u8; 4], Option<u32>)>,
    /// Whether one past [`UNMODELED_REPORTS`] was seen, which is reported
    /// once.
    unlisted_unmodeled: bool,
    /// Those reports, not yet taken ([`Self::take_unmodeled`]).
    reported: Vec<String>,
    /// The methods called so far: each one's first call is logged.
    called: BTreeSet<[u8; 4]>,
    /// The swap surface layouts seen so far, each reported once
    /// ([`LAYOUTS`] at most).
    layouts: BTreeSet<swap::SurfaceLayout>,
    /// Whether a layout past [`LAYOUTS`] was seen, which is reported once.
    unlisted_layouts: bool,
    /// The layer shapes logged so far ([`PLACEMENTS`] at most).
    placements: u32,
    /// The current call, when its log entry was left out; a refusal logs it.
    pending_call: Option<Log>,
    /// Whether the AP has any notification client (`A033`).
    notified: bool,
    /// The counter value of the next vblank; 0 before the display's first
    /// look, which starts the grid.
    next_refresh: u64,
    /// The monitor's refresh period, in counter ticks.
    refresh_ticks: u64,
    /// The last vblank the display passed whose latch is still open: a
    /// ready swap may be presented for it. A present or a vblank with
    /// nothing ready closes it.
    latched: Option<u64>,
    /// The vsync waiting to leave for the AP's notification clients: its
    /// vblank's counter value. A later one replaces it, as a display drops
    /// the tick it could not deliver.
    refresh: Option<u64>,
    /// `set_color_remap_mode`'s value, which reads back; 0 is IOMFB's normal
    /// mode, and the scanout applies no other.
    color_remap_mode: u32,
    log: VecDeque<Log>,
    dropped: u64,
    dumped: u32,
    /// Whether to report each swap's calls, present and completion.
    trace_swaps: bool,
    /// Swaps whose pixels reached the host since this display was placed.
    presented: u64,
    /// This window's pacing, the vblank that opened it and the last
    /// present's vblank and swap.
    pacing: Pacing,
    window_start: Option<u64>,
    last_present: Option<(u64, u32)>,
    /// The monitor's power and sleep state, shared with its IOAV controller.
    display: Rc<RefCell<DisplayState>>,
    /// Whether the scanout's monitor is lit, as last set; it follows the
    /// display's power and sleep state ([`Self::apply_power`]).
    sink_powered: bool,
    /// The scanout refused to turn its monitor off or on, which is
    /// reported once.
    sink_power_refused: bool,
    /// Open IOMFB clients (`first_client_open` minus `last_client_close`).
    clients: u32,
    /// The mode set by `set_digital_out_mode`, `(color, timing)`.
    digital_out: Option<(u32, u32)>,
    /// The pipe's config frame size, `(width, height)`; 0 x 0 until the AP
    /// sets it.
    config_frame: (u32, u32),
    next_swap: u32,
    /// Swaps `swap_start_dcp` opened and not yet submitted, oldest first,
    /// with the client that started each.
    started: VecDeque<(u32, u64)>,
    /// Fences `io_fence_notify` reported for started swaps not yet submitted,
    /// as `(swap id, surface id)`.
    early_fences: VecDeque<(u32, u32)>,
    /// A swap was shown without its fence, which is reported once.
    fence_timeout_reported: bool,
    /// The four layer slots after the last submitted swap.
    planes: [Option<Layer>; swap::LAYERS],
    frames: VecDeque<Frame>,
    /// Callbacks to make, in order.
    outgoing: VecDeque<Outgoing>,
    /// Swaps latched at a vblank, whose completion callback is due, and
    /// whether each reached the display.
    completed: VecDeque<(u32, bool)>,
    /// For each queued completion, in the same order: its swap and the
    /// counter when it was submitted and presented ([`Pacing::complete`]).
    completion_times: VecDeque<(u32, u64, u64)>,
    /// A callback is out and not answered yet.
    in_flight: bool,
    /// The AP shut the link down (`link_shutdown_signal`).
    shut_down: bool,
    /// The newest counter value the model was given: a call, notice or
    /// answer is timed by the link service that carried it.
    now: u64,
}

impl Methods {
    pub(super) fn check_reset(&self) -> Result<()> {
        if !self.started.is_empty()
            || !self.frames.is_empty()
            || !self.outgoing.is_empty()
            || !self.completed.is_empty()
            || self.in_flight
            || self.display.borrow().hotplug_pending()
        {
            return Err(refuse!(
                "DCP physical reset with owned scanout or callback work"
            ));
        }
        Ok(())
    }
    pub(super) fn reset(&mut self) {
        // The fixed physical monitor and host scanout survive a coprocessor
        // reset. Native clients/mode/layer state are volatile. The caller
        // preflights all frame and callback ownership before reaching here.
        self.clients = 0;
        self.digital_out = None;
        self.config_frame = (0, 0);
        self.next_swap = 1;
        self.started.clear();
        self.early_fences.clear();
        self.planes = [None; swap::LAYERS];
        self.shut_down = false;
    }
    /// Resume the link after the coprocessor's sleep-to-wake edge.
    /// Firmware 0x187658 clears its quiescence byte and releases stopped
    /// streams; the shared region, mode and client state remain allocated.
    /// No callbacks or swaps discarded by A500 are recreated here.
    pub(super) fn resume(&mut self) {
        self.shut_down = false;
    }

    pub fn new(scanout: Box<dyn Scanout>, interface: &'static Interface) -> Self {
        // Configured model power, not an observation of Apple's boot state.
        Self::with_display_state(
            scanout,
            Rc::new(RefCell::new(DisplayState::new(true))),
            interface,
        )
    }

    pub fn with_display_state(
        scanout: Box<dyn Scanout>,
        display: Rc<RefCell<DisplayState>>,
        interface: &'static Interface,
    ) -> Self {
        let refresh = scanout.refresh_millihertz();
        // A refresh that cannot be advertised leaves macOS without a
        // display, and it stops a minute later: the size is published at
        // the usual rate instead, at which macOS then draws.
        let mut reported = Vec::new();
        let advertised = Advertised::sized(scanout.mode(), refresh, scanout.physical_size_mm())
            .or_else(|refusal| {
                let usual = super::super::scanout::DEFAULT_REFRESH_MILLIHERTZ;
                reported.push(format!(
                    "IOMFB a {refresh} mHz display is advertised at {usual} mHz: {refusal}"
                ));
                Advertised::sized(scanout.mode(), usual, scanout.physical_size_mm())
            });
        Self {
            interface,
            mode: scanout.mode(),
            scanout: Some(scanout),
            frame: scanout::Composed::default(),
            composing: None,
            in_place: false,
            compose_stall_reported: false,
            shown: None,
            unshown: 0,
            advertised,
            last_refusal: None,
            unmodeled: BTreeSet::new(),
            unlisted_unmodeled: false,
            layouts: BTreeSet::new(),
            unlisted_layouts: false,
            placements: 0,
            reported,
            called: BTreeSet::new(),
            pending_call: None,
            notified: false,
            next_refresh: 0,
            refresh_ticks: refresh_ticks(refresh),
            latched: None,
            refresh: None,
            color_remap_mode: 0,
            log: VecDeque::new(),
            dropped: 0,
            dumped: 0,
            trace_swaps: true,
            presented: 0,
            pacing: Pacing::default(),
            window_start: None,
            last_present: None,
            display,
            sink_powered: true,
            sink_power_refused: false,
            clients: 0,
            digital_out: None,
            config_frame: (0, 0),
            next_swap: 1,
            started: VecDeque::new(),
            early_fences: VecDeque::new(),
            fence_timeout_reported: false,
            planes: [None; swap::LAYERS],
            frames: VecDeque::new(),
            outgoing: VecDeque::new(),
            completed: VecDeque::new(),
            completion_times: VecDeque::new(),
            in_flight: false,
            shut_down: false,
            now: 0,
        }
    }

    pub(super) fn display_state(&self) -> Rc<RefCell<DisplayState>> {
        self.display.clone()
    }

    /// The fixed mode the one connected display runs.
    pub fn mode(&self) -> Mode {
        self.mode
    }

    /// The last `setPowerState` the AP asked for.
    #[cfg(test)]
    pub fn powered(&self) -> Option<bool> {
        self.display.borrow().powered
    }

    /// The `(color, timing)` IDs of the last `set_digital_out_mode`.
    #[cfg(test)]
    pub fn digital_out(&self) -> Option<(u32, u32)> {
        self.digital_out
    }

    /// The last refusal's full text.
    #[cfg(test)]
    pub fn last_refusal(&self) -> Option<&str> {
        self.last_refusal.as_deref()
    }

    /// Choose per-swap tracing or bounded presentation reports.
    pub(super) fn set_swap_tracing(&mut self, enabled: bool) {
        self.trace_swaps = enabled;
    }

    fn trace_swap(&self, swap_id: u32) -> bool {
        self.trace_swaps || swap_id <= DUMPED_SWAPS
    }

    /// The next display-log entry.
    pub fn take_log(&mut self) -> Option<Log> {
        self.log.pop_front().or_else(|| {
            (self.dropped != 0).then(|| Log::Dropped(core::mem::take(&mut self.dropped)))
        })
    }

    pub(super) fn record_log(&mut self, entry: Log) {
        if self.log.len() < LOG_CAPACITY {
            self.log.push_back(entry);
        } else {
            self.dropped += 1;
        }
    }

    /// The next submitted swap waiting for its pixels.
    pub fn frame_pending(&self) -> bool {
        !self.frames.is_empty()
    }

    /// How many swaps, in submission order from the oldest, can be shown at
    /// counter `now`: each has its surfaces' GPU work finished, or has
    /// waited [`FENCE_TIMEOUT`] for it. Swaps are shown in order, so a ready
    /// swap waits behind one still drawing.
    fn ready_swaps(&mut self, now: u64) -> usize {
        let mut ready = 0;
        let mut blocked = false;
        let mut timed_out = Vec::new();
        // Every fenced swap's wait starts when the display first sees it,
        // behind a blocked one or not.
        for frame in &mut self.frames {
            if frame.fenced != 0 {
                let since = *frame.waiting_since.get_or_insert(now);
                if !blocked && now.saturating_sub(since) >= FENCE_TIMEOUT {
                    // Degraded: the surface may still be drawing. Show it
                    // rather than freeze the display, and say so once.
                    timed_out.push((frame.swap_id, frame.fenced));
                    frame.fenced = 0;
                    if frame.ready_at.is_none() {
                        frame.ready_at = Some(now);
                    }
                }
            }
            blocked |= frame.fenced != 0;
            ready += usize::from(!blocked);
        }
        for (swap_id, fenced) in timed_out {
            self.pacing.fence_timeouts += 1;
            if !self.fence_timeout_reported {
                self.fence_timeout_reported = true;
                self.record_log(Log::FenceTimeout {
                    swap_id,
                    fenced,
                    seconds: FENCE_TIMEOUT / COUNTER_HZ,
                });
            }
        }
        ready
    }

    /// Whether the vblank the display last passed latches a swap at counter
    /// `now`: one is ready ([`Self::ready_swaps`]) and the latch is still
    /// open. A vblank that finds none ready closes its latch: the monitor
    /// repeats its frame, and a swap that becomes ready later waits for the
    /// next vblank.
    pub fn frame_ready(&mut self, now: u64) -> bool {
        if self.latched.is_none() {
            return false;
        }
        if self.ready_swaps(now) != 0 {
            return true;
        }
        self.latched = None;
        false
    }

    /// `io_fence_notify`: the GPU finished `surface` for `swap_id`. A queued
    /// swap stops waiting for each layer that shows it; one only started
    /// keeps the notice for its submission.
    fn fence_notified(&mut self, swap_id: u32, surface: u32) {
        if let Some(frame) = self
            .frames
            .iter_mut()
            .find(|frame| frame.swap_id == swap_id)
        {
            for (index, &id) in frame.surface_ids.iter().enumerate() {
                if id == surface {
                    frame.fenced &= !(1 << index);
                }
            }
            if frame.fenced == 0 && frame.ready_at.is_none() {
                frame.ready_at = Some(self.now);
                self.pacing
                    .fence
                    .add(self.now.saturating_sub(frame.submitted_at));
            }
        } else if self.started.iter().any(|&(id, _)| id == swap_id) {
            if self.early_fences.len() == STARTED_SWAPS {
                self.early_fences.pop_front();
            }
            self.early_fences.push_back((swap_id, surface));
        }
    }

    /// The counter at which the display next needs a look: its next
    /// vblank, while the AP's notification clients listen for it, a swap
    /// waits for one or the scanout holds work for a later flush.
    pub fn refresh_deadline(&self) -> Option<u64> {
        let wanted = self.notified
            || (!self.frames.is_empty() && self.display.borrow().active())
            || self.display.borrow().active() != self.sink_powered
            || self
                .scanout
                .as_ref()
                .is_some_and(|scanout| scanout.queued());
        (wanted && !self.shut_down).then_some(self.next_refresh)
    }

    /// Advance the display to counter `now`: flush what the scanout queued,
    /// and once a vblank is passed, open its latch if a swap is ready and
    /// queue its vsync for the notification clients. Vblanks keep their phase: a late look
    /// takes the last one it passed and the next falls on the same grid,
    /// so the rate holds however late the counter is read. A look more than
    /// a period late skips the vblanks between, as nothing was presented or
    /// told of them. The first look starts the grid.
    pub fn refresh(&mut self, now: u64) {
        self.observe(now);
        if self.shut_down {
            return;
        }
        self.apply_power();
        if let Some(scanout) = self.scanout.as_mut()
            && scanout.queued()
            && let Err(reason) = scanout.flush()
        {
            self.refusal(format!("DCP scanout refused a queued frame: {reason}"));
        }
        if self.next_refresh != 0 && now < self.next_refresh {
            return;
        }
        let due = if self.next_refresh == 0 {
            now
        } else {
            let behind = (now - self.next_refresh) / self.refresh_ticks;
            self.next_refresh + behind * self.refresh_ticks
        };
        self.next_refresh = due + self.refresh_ticks;
        // The latch takes only what is ready at its vblank.
        self.latched = Some(due);
        self.frame_ready(now);
        if self.notified {
            self.refresh = Some(due);
        }
        self.close_window(due);
    }

    /// Turn the scanout's monitor off or on as the display's power and sleep
    /// state now ask, from `setPowerState`, DCPAVIPC's `SetPower` or its
    /// display sleep: macOS's display sleep then darkens the monitor, and a
    /// wake shows the last frame again before the next swap. While a compose
    /// has the scanout this waits for the next look. A sink that refuses is
    /// reported once, and its monitor keeps the last frame lit.
    fn apply_power(&mut self) {
        let active = self.display.borrow().active();
        if active == self.sink_powered {
            return;
        }
        let Some(scanout) = self.scanout.as_mut() else {
            return;
        };
        self.sink_powered = active;
        if let Err(reason) = scanout.set_power(active)
            && !core::mem::replace(&mut self.sink_power_refused, true)
        {
            self.reported.push(format!(
                "IOMFB display {}: {reason}; the monitor stays as it was",
                if active { "wake" } else { "sleep" }
            ));
        }
    }

    /// The counter reads `now`: what the calls, notices and answers the link
    /// carries next are timed by. The counter never goes back.
    pub(super) fn observe(&mut self, now: u64) {
        self.now = self.now.max(now);
    }

    /// Open the pacing window at vblank `due`, or close it once it covers
    /// [`PACING_WINDOW`] and report it, if anything was submitted or shown.
    fn close_window(&mut self, due: u64) {
        let Some(start) = self.window_start else {
            self.window_start = Some(due);
            return;
        };
        let ticks = due.saturating_sub(start);
        if ticks < PACING_WINDOW {
            return;
        }
        let pacing = core::mem::take(&mut self.pacing);
        if pacing.presented != 0 || pacing.submitted != 0 {
            self.record_log(Log::Pacing(Pacing { ticks, ..pacing }));
        }
        self.window_start = Some(due);
    }

    /// Count a present of `swap_id` for vblank `due` at counter `now`, with
    /// `waiting` swaps queued.
    fn pace(&mut self, now: u64, due: u64, swap_id: u32, waiting: usize) {
        if let Some((last, last_swap)) = self.last_present {
            self.pacing
                .gap(due.saturating_sub(last), self.refresh_ticks);
            self.pacing.reordered += u32::from(swap_id < last_swap);
        }
        self.last_present = Some((due, swap_id));
        self.pacing.presented += 1;
        self.pacing.deepest = self.pacing.deepest.max(waiting);
        let late = now.saturating_sub(due);
        self.pacing.late_total += late;
        self.pacing.late_longest = self.pacing.late_longest.max(late);
    }

    /// Present at counter `now` the swaps the last vblank latched: read the
    /// newest one's layers through `read` (DVA, bytes; false when
    /// unreachable), hand the rows its damage covers to the scanout and
    /// queue the completion of every latched swap, oldest first. A swap is
    /// never completed before the pixels it shows are consumed; a failed
    /// present keeps every swap and the latch for a retry.
    pub fn present(&mut self, now: u64, read: impl FnMut(u64, &mut [u8]) -> bool) -> Result<()> {
        let Some(batch) = self.batch(now) else {
            return Ok(());
        };
        self.in_place = false;
        let scanout = self
            .scanout
            .as_mut()
            .expect("no batch while a compose has the scanout");
        scanout::present(
            scanout.as_mut(),
            &batch.layers,
            batch.damage.clone(),
            read,
            &mut self.frame,
        )?;
        // Both the surface DMA and host sink can refuse. The submitted swaps
        // own their surfaces until presentation succeeds, or the AP explicitly
        // aborts/shuts down; neither failure consumes them or queues D594.
        self.presented(batch);
        Ok(())
    }

    /// What the vblank the display last passed latches at counter `now`,
    /// if a swap is ready and no compose is in flight: the swaps it shows,
    /// the newest one's layers and the rows that may change.
    fn batch(&mut self, now: u64) -> Option<Batch> {
        if !self.display.borrow().active() || self.composing.is_some() {
            return None;
        }
        self.refresh(now);
        if !self.frame_ready(now) {
            return None;
        }
        let due = self.latched.expect("a ready frame has an open latch");
        let count = self.ready_swaps(now);
        let newest = &self.frames[count - 1];
        let planes = newest.planes;
        let layers = newest.layers();
        let updated = self
            .frames
            .iter()
            .take(count)
            .fold(self.unshown, |slots, frame| slots | frame.updated);
        let damage = self
            .shown
            .map(|shown| damage(&shown, &planes, updated, self.mode().height));
        Some(Batch {
            now,
            due,
            count,
            planes,
            layers,
            damage,
        })
    }

    /// Present at counter `now` as [`Self::present`] does, composing
    /// outside the devices' lock ([`super::compose`]): the compose to run,
    /// once every page the layers reach resolves (`resolve`, in pages of
    /// `page` bytes). `None` with nothing to present, a compose in flight,
    /// or a page that does not resolve; [`Self::present`] then presents
    /// what is latched in place.
    pub fn compose(
        &mut self,
        now: u64,
        page: u64,
        resolve: impl FnMut(u64) -> core::result::Result<u64, stela_contracts::dma::DmaError>,
    ) -> Option<Compose> {
        if self.composing.is_some() || self.scanout.is_none() || self.in_place {
            return None;
        }
        // Read-only until it is certain the compose leaves: the batch is
        // taken again in place otherwise.
        let probe = self.batch(now)?;
        let pages = Pages::resolve(&probe.layers, page, resolve)?;
        let scanout = self.scanout.take().expect("checked above");
        let composed = core::mem::take(&mut self.frame);
        let (compose, done) = Compose::new(
            scanout,
            composed,
            probe.layers.clone(),
            probe.damage.clone(),
            pages,
        );
        self.composing = Some((probe, done));
        Some(compose)
    }

    /// Wait for a compose in flight and finish its present, before a call
    /// that drops queued swaps acts on them: they are then as a present in
    /// place would have left them. A present's refusal is recorded as the
    /// call's. A compose that does not end within [`SETTLE_SPINS`] (its PE
    /// runs it as soon as it leaves the devices' lock, so none should) is
    /// reported once and refuses the call, which keeps the swaps.
    pub fn settle_compose(&mut self) -> Result<()> {
        self.settle_compose_within(SETTLE_SPINS)
    }

    fn settle_compose_within(&mut self, spins: u64) -> Result<()> {
        if let Err(reason) = self.finish_compose(spins) {
            self.refusal(format!("DCP present refused: {reason}"));
        }
        if !self.composing() {
            return Ok(());
        }
        if !core::mem::replace(&mut self.compose_stall_reported, true) {
            self.refusal("DCP compose outside the devices' lock did not end in time".to_string());
        }
        Err(refuse!(
            "a compose outside the devices' lock is still in flight"
        ))
    }

    /// Whether a compose outside the devices' lock is in flight.
    pub fn composing(&self) -> bool {
        self.composing.is_some()
    }

    /// Take back what a finished compose left, if it did: the scanout and
    /// its rows, then the present finished as in place. `Ok(true)` when it
    /// presented, `Ok(false)` when nothing finished or a read missed the
    /// resolved pages (the latch stays open for a present in place), and
    /// the present's refusal. A compose in flight is waited for at most
    /// `spins` spin-loop hints.
    pub fn finish_compose(&mut self, spins: u64) -> Result<bool> {
        let Some((_, done)) = &self.composing else {
            return Ok(false);
        };
        let finished = done.wait(spins);
        let Some(Composition {
            sink,
            composed,
            result,
            missed,
        }) = finished
        else {
            return Ok(false);
        };
        let (batch, _) = self.composing.take().expect("checked above");
        self.scanout = Some(sink);
        self.frame = composed;
        if missed {
            self.in_place = true;
            return Ok(false);
        }
        result?;
        self.presented(batch);
        Ok(true)
    }

    /// Finish the present of `batch`, whose rows the scanout now holds:
    /// complete its swaps, oldest first, and pace by it.
    fn presented(&mut self, batch: Batch) {
        let Batch {
            now,
            due,
            count,
            planes,
            layers,
            damage: _,
        } = batch;
        // A vblank passed during a compose opened a new latch: it stays.
        if self.latched == Some(due) {
            self.latched = None;
        }
        if self
            .shown
            .as_ref()
            .is_some_and(|shown| moved(shown, &planes))
        {
            self.pacing.moved += 1;
            MOVED.fetch_add(1, core::sync::atomic::Ordering::Release);
        }
        self.shown = Some(planes);
        self.unshown = 0;
        let waiting = self.frames.len();
        let batch: Vec<Frame> = self.frames.drain(..count).collect();
        let (shown, merged) = batch.split_last().expect("a latched batch is not empty");
        for frame in &batch {
            let ready = frame.ready_at.unwrap_or(now);
            self.pacing.latch.add(now.saturating_sub(ready));
        }
        for (index, frame) in merged.iter().enumerate() {
            let later = batch[index + 1..]
                .iter()
                .fold(0, |slots, later| slots | later.updated);
            let on_screen = frame
                .planes
                .iter()
                .enumerate()
                .filter(|&(slot, _)| frame.updated & 1 << slot != 0)
                .filter_map(|(_, layer)| layer.as_ref())
                .any(|own| layers.iter().any(|layer| layer.surface == own.surface));
            // Replaced in every slot it set, with none of its surfaces on
            // screen: the AP may take its surfaces back at once.
            let displayed = frame.updated & !later != 0 || on_screen;
            self.completed.push_back((frame.swap_id, displayed));
            self.completion_times
                .push_back((frame.swap_id, frame.submitted_at, now));
            self.pacing.merged += 1;
            if self.trace_swap(frame.swap_id) {
                self.record_log(Log::Merged {
                    swap_id: frame.swap_id,
                    shown: shown.swap_id,
                    displayed,
                });
            }
        }
        self.presented = self.presented.saturating_add(1);
        self.pace(now, due, shown.swap_id, waiting);
        self.pacing.ahead += u32::from(super::gpu::completed() < shown.gpu_work);
        if self.trace_swap(shown.swap_id) {
            self.record_log(Log::Presented {
                swap_id: shown.swap_id,
                surfaces: layers.iter().map(|layer| layer.surface).collect(),
            });
        }
        if !self.trace_swaps && self.presented.is_multiple_of(PRESENT_SUMMARY_INTERVAL) {
            self.record_log(Log::PresentSummary {
                total: self.presented,
                last_swap: shown.swap_id,
            });
        }
        self.completed.push_back((shown.swap_id, true));
        self.completion_times
            .push_back((shown.swap_id, shown.submitted_at, now));
    }

    fn refusal(&mut self, text: String) {
        self.record_log(Log::Refused(text.clone()));
        self.last_refusal = Some(text);
    }

    /// Record and return a named refusal for method `tag`.
    fn refuse(&mut self, tag: [u8; 4], reason: Refusal) -> Result<Step<Self>> {
        // The refused call itself, if its entry was left out.
        if let Some(call) = self.pending_call.take() {
            self.record_log(call);
        }
        let row = self
            .interface
            .method(&tag)
            .map(|row| {
                format!(
                    " {}::{} (in {:#x}, out {:#x})",
                    row.class, row.name, row.input, row.output
                )
            })
            .unwrap_or_default();
        self.refusal(format!("DCP IOMFB {}{row}: {reason}", abi::tag_str(&tag)));
        Err(reason)
    }

    /// Record and return a named refusal of the AP's answer to callback `tag`.
    fn refuse_answer(&mut self, tag: [u8; 4], reason: Refusal) -> Result<()> {
        let name = abi::callback(&tag).map_or("", |row| row.name);
        self.refusal(format!(
            "DCP IOMFB callback {} {name}: {reason}",
            abi::tag_str(&tag)
        ));
        Err(reason)
    }

    /// Report method `tag`, answered `answer` without a model, once per
    /// `detail` (a parameter the method names, where one tells requests
    /// apart), in the display log and as an `UNMODELED` line.
    fn report_unmodeled(&mut self, tag: [u8; 4], answer: Degraded, detail: Option<u32>) {
        if self.unmodeled.len() >= UNMODELED_REPORTS && !self.unmodeled.contains(&(tag, detail)) {
            if !self.unlisted_unmodeled {
                self.unlisted_unmodeled = true;
                self.reported.push(format!(
                    "IOMFB unmodeled answers after the first {UNMODELED_REPORTS} are not listed"
                ));
            }
            return;
        }
        if self.unmodeled.insert((tag, detail)) {
            let name = self.name(&tag);
            let entry = Log::Unmodeled {
                tag,
                name,
                answer,
                detail,
            };
            // The display-log line without its `DCP ` lead; the machine names
            // the coprocessor.
            let line = entry.to_string();
            self.reported.push(format!(
                "IOMFB {}",
                line.strip_prefix("DCP ").unwrap_or(&line)
            ));
            self.record_log(entry);
        }
    }

    /// The methods answered degraded for the first time since the last
    /// call, one line each.
    pub fn take_unmodeled(&mut self) -> Vec<String> {
        core::mem::take(&mut self.reported)
    }

    /// Answer a status method the model cannot carry out with the
    /// protocol's error reply, reported once per `detail`.
    fn unsupported(
        &mut self,
        row: &Method,
        input: &[u8],
        output: u32,
        detail: Option<u32>,
    ) -> Result<Step<Self>> {
        let Some(reply) = abi::status_reply(row) else {
            return self.refuse(
                row.tag,
                refuse!("DCP method has no recorded status word for its error reply"),
            );
        };
        let packed = abi::status_output(reply, input, output, LINK_UNSUPPORTED);
        if packed.is_ok() {
            self.pacing.unmodeled = self.pacing.unmodeled.saturating_add(1);
            self.report_unmodeled(row.tag, Degraded::Unsupported, detail);
        }
        self.reply(row.tag, packed)
    }

    /// A method of the interface with no model: the error reply where its
    /// status word is recorded, else zeros.
    fn unmodeled_answer(&mut self, row: &Method, input: &[u8], output: u32) -> Result<Step<Self>> {
        if abi::status_reply(row).is_some() {
            return self.unsupported(row, input, output, None);
        }
        self.pacing.unmodeled = self.pacing.unmodeled.saturating_add(1);
        self.report_unmodeled(row.tag, Degraded::Zeros, None);
        Ok(Step::Return(alloc::vec![0; output as usize]))
    }

    /// The interface's name for method `tag`, empty off the interface.
    fn name(&self, tag: &[u8; 4]) -> &'static str {
        self.interface.method(tag).map_or("", |row| row.name)
    }

    fn callback_log(&mut self, tag: [u8; 4], input: usize, output: u32) {
        self.record_log(Log::Callback { tag, input, output });
    }

    fn reply(&mut self, tag: [u8; 4], packed: Result<Vec<u8>>) -> Result<Step<Self>> {
        match packed {
            Ok(bytes) => Ok(Step::Return(bytes)),
            Err(reason) => self.refuse(tag, reason),
        }
    }

    /// Queue a callback, sized by its interface row.
    fn queue(&mut self, tag: [u8; 4], input: Packed) -> Result<()> {
        let row = abi::callback(&tag).ok_or(refuse!("DCP callback is not on the interface"))?;
        let (Some(length), Some(output)) = (row.input, row.output) else {
            return Err(refuse!("DCP callback sizes are not recorded"));
        };
        let input = input.finish(length)?;
        self.outgoing.push_back(Outgoing { tag, input, output });
        Ok(())
    }

    /// Publish the one mode and the monitor's identity on the IOMFB service,
    /// then report the monitor plugged. WindowServer leaves a monitor that does
    /// not say what it is dark, and composites to a virtual display instead.
    fn publish(&mut self) -> Result<()> {
        let advertised = match &self.advertised {
            Ok(advertised) => advertised,
            Err(reason) => {
                let reason = reason.clone();
                return self.refuse_answer(PROP_START, reason);
            }
        };
        let properties = [
            ("TimingElements", advertised.timing_elements()),
            ("ColorElements", advertised.color_elements()),
            ("DisplayAttributes", advertised.display_attributes()),
            ("Transport", Advertised::transport()),
            ("IOMFBUUID", Advertised::uuid()),
        ];
        for (key, xml) in properties {
            // setDCPAVPropEnd NUL-terminates the buffer's last byte and
            // hands the whole buffer to OSUnserializeXML.
            let mut bytes = xml.into_bytes();
            bytes.push(0);
            let length =
                u32::try_from(bytes.len()).map_err(|_| refuse!("DCP property is too long"))?;
            self.queue(PROP_START, Packed::new().u32(length))?;
            for (index, chunk) in bytes.chunks(CHUNK_BYTES).enumerate() {
                let mut data = [0u8; CHUNK_BYTES];
                data[..chunk.len()].copy_from_slice(chunk);
                self.queue(
                    PROP_CHUNK,
                    Packed::new()
                        .bytes(&data)
                        .u32((index * CHUNK_BYTES) as u32)
                        .u32(chunk.len() as u32),
                )?;
            }
            let mut name = [0u8; KEY_BYTES];
            name[..key.len()].copy_from_slice(key.as_bytes());
            self.queue(PROP_END, Packed::new().bytes(&name))?;
        }
        // A single, untiled display: a zero IOMFB_TiledDisplayInfo, present
        // (its null flag clear); the trailing bool is not read by
        // IOMobileFramebufferShim::hotPlug_notify_gated.
        self.queue_hotplug()
    }

    fn queue_hotplug(&mut self) -> Result<()> {
        self.queue(
            HOTPLUG,
            Packed::new()
                .u64(PLUGGED)
                .bytes(&[0; TILED_DISPLAY_INFO])
                .bool(false)
                .bool(false),
        )
    }
}

impl Service for Methods {
    fn call(&mut self, tag: [u8; 4], input: &[u8], output: u32) -> Result<Step<Self>> {
        let row = self.interface.method(&tag);
        let swap = row.is_some_and(|row| {
            row.class == AP && matches!(row.name, "swap_start_dcp" | "swap_submit_dcp")
        });
        self.pacing.calls = self.pacing.calls.saturating_add(1);
        let first = self.called.insert(tag);
        let logged = if swap {
            self.trace_swap(self.started.back().map_or(self.next_swap, |&(id, _)| id))
        } else {
            self.trace_swaps || first
        };
        let entry = Log::Call {
            tag,
            name: row.map_or("", |row| row.name),
            input: input[..input.len().min(LOGGED_INPUT)].to_vec(),
            output,
        };
        self.pending_call = None;
        if logged {
            self.record_log(entry);
        } else {
            self.pending_call = Some(entry);
        }
        let Some(row) = row else {
            return self.refuse(tag, refuse!("tag is not a method of this DCP interface"));
        };
        if row.input as usize != input.len() || row.output != output {
            return self.refuse(
                tag,
                refuse!("call sizes differ from the interface's record"),
            );
        }
        if self.shut_down {
            return self.refuse(
                tag,
                refuse!(
                    "the AP shut the DCP link down; an IOP sleep-to-wake transition is required"
                ),
            );
        }
        let mut fields = Fields::new(input);
        match (row.class, row.name) {
            // start_signal: nest D120 start_hardware_boot, report its bool,
            // then publish the mode and the hotplug.
            (AP, "start_signal") => {
                self.callback_log(*b"D120", 0, 4);
                Ok(Step::Call {
                    tag: *b"D120",
                    input: Vec::new(),
                    output: 4,
                    then: Box::new(|methods, answer| {
                        methods.record_log(Log::Answer {
                            tag: *b"D120",
                            output: answer.to_vec(),
                        });
                        let ok = answer.first().is_some_and(|byte| byte & 1 != 0);
                        if ok {
                            methods.publish()?;
                        }
                        Ok(Step::Return(bool_reply(ok)))
                    }),
                })
            }
            // late_init_signal(bool) -> bool (Asahi boot_4, true on >= 13.2).
            (H13P, "late_init_signal") => {
                let packed = Packed::new().bool(true).finish(output);
                self.reply(tag, packed)
            }
            // update_notify_clients_dcp(u32 clients[27]): a count of clients
            // for each notification. Every one is told of the refresh.
            (H13P, "update_notify_clients_dcp") => {
                self.notified = input.iter().any(|&byte| byte != 0);
                Ok(Step::Return(Vec::new()))
            }
            // setup_video_limits(): the fixed mode has no limits to set up.
            (H13P, "setup_video_limits") => Ok(Step::Return(Vec::new())),
            // set_color_remap_mode(mode) -> link status (AP stub
            // 0xfffffe000ada3348). The scanout applies only mode 0; another
            // mode is unsupported and the mode reads back unchanged.
            (AP, "set_color_remap_mode") => {
                let mode = fields.u32()?;
                if mode != 0 {
                    return self.unsupported(row, input, output, Some(mode));
                }
                self.color_remap_mode = mode;
                Ok(Step::Return(IO_RETURN_SUCCESS.to_le_bytes().to_vec()))
            }
            // set_matrix(u32 location, const u64 (*)[3][3]) -> link status:
            // the scanout applies the identity, so only it is carried out.
            (AP, "set_matrix") => {
                let location = fields.u32()?;
                let mut identity = true;
                for index in 0..9 {
                    let one = if index % 4 == 0 { MATRIX_ONE } else { 0 };
                    identity &= fields.u64()? == one;
                }
                // The matrix pointer's null flag.
                identity &= !fields.bool()?;
                if !identity {
                    return self.unsupported(row, input, output, Some(location));
                }
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // swap_set_color_matrix(IOMFBColorFixedMatrix*, function, u32)
            // -> the matrix back, then the link status. Core Animation sends
            // its accessibility matrix (function 2) in each frame until a
            // call succeeds. It names no layer: one matrix for the pipe
            // (Inferred: after blending). The scanout applies none, so only
            // the identity is carried out.
            (AP, "swap_set_color_matrix") => {
                self.pacing.color_matrix = self.pacing.color_matrix.saturating_add(1);
                if first {
                    self.record_log(Log::Payload {
                        tag,
                        name: row.name,
                        bytes: input.to_vec(),
                    });
                }
                let mut values = [0u64; COLOR_MATRIX_VALUES];
                for value in &mut values {
                    *value = fields.u64()?;
                }
                let function = fields.u32()?;
                fields.u32()?;
                // A null matrix: the AP's fill, which nothing reads.
                let null = fields.bool()?;
                let identity = values.iter().enumerate().all(|(index, &value)| {
                    let coefficient = index.wrapping_sub(COLOR_MATRIX_FIRST_COEFFICIENT);
                    let one = coefficient < 9 && coefficient % 4 == 0;
                    value == if one { MATRIX_ONE } else { 0 }
                });
                if function >= COLOR_MATRIX_FUNCTIONS || !(null || identity) {
                    // The matrix goes with its one report.
                    if !first && !self.unmodeled.contains(&(tag, Some(function))) {
                        self.record_log(Log::Payload {
                            tag,
                            name: row.name,
                            bytes: input.to_vec(),
                        });
                    }
                    return self.unsupported(row, input, output, Some(function));
                }
                let Some(reply) = abi::status_reply(row) else {
                    return self.refuse(
                        tag,
                        refuse!("DCP method has no recorded status word for its reply"),
                    );
                };
                let packed = abi::status_output(reply, input, output, IO_RETURN_SUCCESS);
                self.reply(tag, packed)
            }
            // setBrightnessCorrection(u32 16.16) -> link status: only the
            // uncorrected value is what the scanout shows.
            (AP, "setBrightnessCorrection") => {
                if fields.u32()? != BRIGHTNESS_UNCORRECTED {
                    return self.unsupported(row, input, output, None);
                }
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // enable_disable_video_power_savings(u32 enable) -> link status:
            // the display never saves power, so only disabling is carried out.
            (AP, "enable_disable_video_power_savings") => {
                let enable = fields.u32()?;
                if enable != 0 {
                    return self.unsupported(row, input, output, Some(enable));
                }
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // get_color_remap_mode(*mode) -> {mode, status}: the pointer's
            // value and a null flag go in (IOMobileFramebufferAP 0xad7c358),
            // which the answer does not depend on.
            (AP, "get_color_remap_mode") => {
                let mut bytes = self.color_remap_mode.to_le_bytes().to_vec();
                bytes.extend_from_slice(&IO_RETURN_SUCCESS.to_le_bytes());
                Ok(Step::Return(bytes))
            }
            // set_parameter_dcp(param, value[4], count) -> u32. Asahi zeroes
            // adaptive sync, and WindowServer sets others of unknown meaning as
            // it opens the display, retrying a failure forever. Zero leaves the
            // one fixed mode as it is; anything else is a recorded gap.
            (AP, "set_parameter_dcp") => {
                let parameter = fields.u32()?;
                let mut values = [0u64; 4];
                for value in &mut values {
                    *value = fields.u64()?;
                }
                // Past `count`, the AP leaves its stack's bytes.
                let count = fields.u32()? as usize;
                let set = values.get(..count).unwrap_or(&values);
                if set.iter().any(|&value| value != 0) {
                    self.report_unmodeled(
                        tag,
                        Degraded::Ignored("the fixed mode runs as it is"),
                        Some(parameter),
                    );
                }
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // setPowerState(state, ...) -> {u32, u32 result}: 1 on, 0 off
            // (Asahi dcp_on_set_power_state / last_client_closed_poff).
            (AP, "setPowerState") => {
                let state = fields.u64()?;
                self.display.borrow_mut().powered = Some(match state {
                    0 => false,
                    1 => true,
                    _ => {
                        return self.refuse(
                            tag,
                            refuse!("setPowerState names a state other than off or on"),
                        );
                    }
                });
                self.apply_power();
                let packed = Packed::new().u32(0).u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // manage_properties(bool) (UnifiedPipeline2 0x2a52a8c; the
            // shim's HPD path passes "plugged"): the properties are the
            // ones already published; withdrawing them is not modeled.
            (UP, "manage_properties") => {
                if !fields.bool()? {
                    return self.refuse(
                        tag,
                        refuse!("manage_properties withdraws the published modes, which the fixed monitor never does"),
                    );
                }
                Ok(Step::Return(Vec::new()))
            }
            // getVideoDataForIDs(IOAVVideoTimingData*, IOAVVideoColorData*,
            // u32 timing, u32 color, bool* dual pipe) -> bool (UnifiedPipeline2
            // 0x2a52ae4; the shim passes the timing ID before the color ID).
            (UP, "getVideoDataForIDs") => {
                let timing_in = fields.bytes(modes::TIMING_BYTES)?;
                let color_in = fields.bytes(modes::COLOR_BYTES)?;
                let timing_id = fields.u32()?;
                let color_id = fields.u32()?;
                let found = self
                    .advertised
                    .as_ref()
                    .ok()
                    .and_then(|advertised| advertised.video_data(timing_id, color_id));
                let packed = match found {
                    Some((timing, color)) => Packed::new()
                        .bytes(timing)
                        .bytes(color)
                        .bool(false)
                        .bool(true),
                    // Unknown IDs: the records are left as passed, false.
                    None => Packed::new()
                        .bytes(timing_in)
                        .bytes(color_in)
                        .bool(false)
                        .bool(false),
                };
                let packed = packed.finish(output);
                self.reply(tag, packed)
            }
            // set_digital_out_mode(color id, timing id) -> link status
            // (IOMobileFramebufferAP 0x3d77a64).
            (AP, "set_digital_out_mode") => {
                let color = fields.u32()?;
                let timing = fields.u32()?;
                if (color, timing) != (modes::COLOR_ID, modes::TIMING_ID) {
                    return self.refuse(
                        tag,
                        refuse!("set_digital_out_mode names a mode the DCP never advertised"),
                    );
                }
                self.digital_out = Some((color, timing));
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // get_digital_out_state(u32* state) -> link status
            // (IOMobileFramebufferAP 0x3d77ad4): the monitor is connected.
            (AP, "get_digital_out_state") => {
                let packed = Packed::new()
                    .u32(DIGITAL_OUT_CONNECTED)
                    .u32(IO_RETURN_SUCCESS)
                    .finish(output);
                self.reply(tag, packed)
            }
            // UPPipe2::get_config_frame_size(u32* width, u32* height)
            // (0x2a6b264): both values, then a null flag per pointer.
            // `UnifiedPipeline2::start_hardware_boot` (0x2a5d840) treats a
            // zero dimension as unset and configures the display size
            // (Inferred: a freshly started pipe has none configured).
            (PIPE2, "get_config_frame_size") => {
                fields.u32()?;
                fields.u32()?;
                let (width, height) = self.config_frame;
                let packed = Packed::new().u32(width).u32(height).finish(output);
                self.reply(tag, packed)
            }
            // set_config_frame_size(u32 width, u32 height) (0x2a6b33c).
            (PIPE2, "set_config_frame_size") => {
                self.config_frame = (fields.u32()?, fields.u32()?);
                Ok(Step::Return(Vec::new()))
            }
            // program_config_frame_size() (0x2a6b39c): the pipe scans out
            // the one mode, so only its size can be programmed.
            (PIPE2, "program_config_frame_size") => {
                let mode = self.mode();
                if self.config_frame != (mode.width, mode.height) {
                    return self.refuse(
                        tag,
                        refuse!(
                            "program_config_frame_size programs a frame size other than the mode's"
                        ),
                    );
                }
                Ok(Step::Return(Vec::new()))
            }
            // first_client_open() / last_client_close_dcp(out u32).
            (AP, "first_client_open") => {
                self.clients += 1;
                Ok(Step::Return(Vec::new()))
            }
            (AP, "last_client_close_dcp") => {
                self.clients = self.clients.saturating_sub(1);
                let packed = Packed::new().u32(0).finish(output);
                self.reply(tag, packed)
            }
            // swap_begin(): no arguments and no result.
            (AP, "swap_begin") => Ok(Step::Return(Vec::new())),
            // swap_start_dcp(u32* swap_id, u64 client) -> {swap_id, u32}.
            // Each IOMFB client starts its own swaps, and the record each
            // submission carries names the swap, so starts need not alternate
            // with submits.
            (AP, "swap_start_dcp") => {
                fields.u32()?;
                let client = fields.u64()?;
                let id = self.next_swap;
                let packed = Packed::new().u32(id).u32(IO_RETURN_SUCCESS).finish(output);
                let reply = self.reply(tag, packed)?;
                self.next_swap = self.next_swap.wrapping_add(1).max(1);
                if self.started.len() == STARTED_SWAPS {
                    self.started.pop_front();
                }
                self.started.push_back((id, client));
                Ok(reply)
            }
            (AP, "swap_submit_dcp") => self.submit(tag, input, output),
            // io_fence_notify(u32 swap_id, u32 surface_id, u64 time,
            // IOMFBStatus) -> IOMFBStatus: `IOMobileFramebufferAP::
            // io_fence_callback` (0x3d96b68) sends it when the GPU fence of a
            // surface `swap_apply_fences` left pending signals, after the
            // swap was submitted. A failed fence also ends the wait: the AP
            // reports it, and the surface is shown as it stands.
            (AP, "io_fence_notify") => {
                let swap_id = fields.u32()?;
                let surface = fields.u32()?;
                // The AP's time of the signal; its clock is this counter
                // only if the guest's timebase has no offset, so a time
                // ahead of the model or older than the fence timeout is
                // not counted.
                let signaled = fields.u64()?;
                let behind = self.now.wrapping_sub(signaled);
                if signaled != 0 && signaled <= self.now && behind < FENCE_TIMEOUT {
                    self.pacing.notice.add(behind);
                }
                self.fence_notified(swap_id, surface);
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // abort_swaps_dcp(u64 client) -> link status: drop that client's
            // open and queued swaps. `IOMobileFramebufferAP::abort_swaps`
            // (26B123 VA 0xfffffe000ad9275c) passes the user client it
            // aborts, the value `swap_start_dcp` carries for its swaps.
            (AP, "abort_swaps_dcp") => {
                let client = fields.u64()?;
                if let Err(reason) = self.settle_compose() {
                    return self.refuse(tag, reason);
                }
                let aborted: Vec<u32> = self
                    .started
                    .iter()
                    .filter(|&&(_, owner)| owner == client)
                    .map(|&(id, _)| id)
                    .collect();
                self.started.retain(|&(_, owner)| owner != client);
                self.early_fences
                    .retain(|(swap_id, _)| !aborted.contains(swap_id));
                let mut unshown = self.unshown;
                self.frames.retain(|frame| {
                    let kept = frame.client != client;
                    if !kept {
                        unshown |= frame.updated;
                    }
                    kept
                });
                self.unshown = unshown;
                let packed = Packed::new().u32(IO_RETURN_SUCCESS).finish(output);
                self.reply(tag, packed)
            }
            // link_shutdown_signal(bool hibernate, bool): the AP's
            // DCPPowerManager::set_power_state (0x3d852c0) sends it for
            // power state 0 once the link is idle, then waits for its own
            // streams and cuts the DCP's power and DART. Methods resume
            // only on the coprocessor's sleep-to-wake edge.
            // The firmware (restore 0x191390, normal 0x191594) reads only
            // the first byte. 1 also saves the pipe for hibernation
            // (0x191434, 0x1884a4). Otherwise it stops the pipe (0x27c99c),
            // tears its AP link down (0x1871f0) and answers nothing. The
            // model's pipe stops: no queued swap is presented or completed
            // and no callback follows.
            (POWER, "link_shutdown_signal") => {
                if fields.u8()? == 1 {
                    return self.refuse(
                        tag,
                        refuse!("link_shutdown_signal saves the pipe for hibernation, which is not modeled"),
                    );
                }
                if let Err(reason) = self.settle_compose() {
                    return self.refuse(tag, reason);
                }
                self.shut_down = true;
                self.started.clear();
                self.early_fences.clear();
                self.unshown = self
                    .frames
                    .drain(..)
                    .fold(self.unshown, |slots, frame| slots | frame.updated);
                self.outgoing.clear();
                self.completed.clear();
                self.completion_times.clear();
                Ok(Step::Return(Vec::new()))
            }
            _ => match answer(row, input, output, self.mode()) {
                Ok(Some(bytes)) => Ok(Step::Return(bytes)),
                // A refusal here would leave the AP's call open for good: it
                // waits, and the link retries the call. It is answered
                // degraded instead.
                Ok(None) => self.unmodeled_answer(row, input, output),
                Err(reason) => self.refuse(tag, reason),
            },
        }
    }

    /// `fControllerPowerState`: the display's controller runs while the
    /// monitor is powered and awake and the link is up. The AP refuses
    /// every swap wait for an external display while it reads 0.
    fn shared_variables(&self) -> [u32; SHARED_VARIABLES] {
        let powered = !self.shut_down && self.display.borrow().active();
        [u32::from(powered), 0, 0]
    }

    fn callback_pending(&self) -> bool {
        !self.in_flight
            && (!(self.outgoing.is_empty() && self.completed.is_empty())
                || self.refresh.is_some()
                || (!self.shut_down && self.display.borrow().hotplug_pending()))
    }

    fn take_callback(&mut self) -> Option<Callback<Self>> {
        if self.in_flight {
            return None;
        }
        if !self.shut_down && self.display.borrow_mut().take_hotplug_request() {
            self.queue_hotplug()
                .expect("the recorded fixed-monitor HPD callback has a valid size");
        }
        if let Some(counter) = self.refresh.take_if(|_| self.outgoing.is_empty()) {
            self.in_flight = true;
            let mut input = alloc::vec![0u8; size_of::<[u64; 3]>()];
            input[..8].copy_from_slice(&counter.to_le_bytes());
            return Some(Callback {
                tag: SWAP_NOTIFY,
                input,
                output: 0,
                then: Box::new(move |methods, _| {
                    methods.in_flight = false;
                    let waited = methods.now.saturating_sub(counter);
                    methods.pacing.vsync.add(waited);
                    Ok(())
                }),
            });
        }
        if let Some(Outgoing { tag, input, output }) = self.outgoing.pop_front() {
            self.in_flight = true;
            self.callback_log(tag, input.len(), output);
            return Some(Callback {
                tag,
                input,
                output,
                then: Box::new(move |methods, answer| {
                    methods.in_flight = false;
                    methods.record_log(Log::Answer {
                        tag,
                        output: answer.to_vec(),
                    });
                    // The setDCPAVProp handlers answer a bool: false is
                    // "setDCPAVPropStart not called or failed", "invalid
                    // chunk requested", "OSUnserializeXML failed" or
                    // "setProperty failed".
                    if tag != HOTPLUG && !answer.first().is_some_and(|byte| byte & 1 != 0) {
                        methods.outgoing.clear();
                        return methods
                            .refuse_answer(tag, refuse!("the AP refused the published mode"));
                    }
                    Ok(())
                }),
            });
        }
        let (swap_id, displayed) = self.completed.pop_front()?;
        let times = self
            .completion_times
            .pop_front()
            .filter(|&(id, _, _)| id == swap_id);
        let length = abi::callback(&SWAP_COMPLETE)
            .and_then(|row| row.input)
            .expect("swap_complete_ap_gated's input size is recorded")
            as usize;
        let mut input = alloc::vec![0u8; length];
        input[0..4].copy_from_slice(&swap_id.to_le_bytes());
        // Only latched swaps complete; one shown in its place may not have
        // been displayed.
        input[SWAP_COMPLETE_DISPLAYED] = u8::from(displayed);
        input[SWAP_COMPLETE_DATA_NULL] = 1;
        self.in_flight = true;
        if self.trace_swap(swap_id) {
            self.callback_log(SWAP_COMPLETE, length, 0);
        }
        Some(Callback {
            tag: SWAP_COMPLETE,
            input,
            output: 0,
            then: Box::new(move |methods, _| {
                methods.in_flight = false;
                if let Some((_, submitted, presented)) = times {
                    let now = methods.now;
                    methods.pacing.complete.add(now.saturating_sub(presented));
                    methods.pacing.total.add(now.saturating_sub(submitted));
                }
                Ok(())
            }),
        })
    }
}

impl Methods {
    /// `swap_submit_dcp`: update the layer slots, queue the visible layers
    /// (none is a background clear) and answer `{bool, u32, link status}`.
    fn submit(&mut self, tag: [u8; 4], input: &[u8], output: u32) -> Result<Step<Self>> {
        if self.dumped < DUMPED_SWAPS {
            self.dumped += 1;
            self.record_log(Log::Payload {
                tag,
                name: self.name(&tag),
                bytes: input.to_vec(),
            });
        }
        let submission = match Submission::parse(input) {
            Ok(submission) => submission,
            Err(reason) => return self.refuse(tag, reason),
        };
        // Each new surface layout once, before the model shows or refuses it.
        for layer in submission.layers.iter().flatten() {
            let layout = layer.record.layout;
            if self.layouts.len() < LAYOUTS {
                if self.layouts.insert(layout) {
                    self.reported.push(layout.to_string());
                }
            } else if !self.unlisted_layouts && !self.layouts.contains(&layout) {
                self.unlisted_layouts = true;
                self.reported.push(format!(
                    "SURFACE layouts after the first {LAYOUTS} are not listed"
                ));
            }
        }
        let swap_id = submission.swap_id;
        let Some(start) = self.started.iter().position(|&(id, _)| id == swap_id) else {
            return self.refuse(
                tag,
                refuse!("swap_submit_dcp names a swap that is not started"),
            );
        };
        if self.frames.len() >= QUEUED_SWAPS {
            return self.refuse(
                tag,
                refuse!("swap_submit_dcp queues more swaps than the model holds"),
            );
        }
        if self.trace_swap(swap_id) {
            self.record_log(Log::Submitted {
                swap_id,
                enabled: submission.enabled,
                fenced: submission.fenced(),
                layers: submission.layers.map(|layer| layer.map(|layer| layer.dva)),
            });
        }
        let planes = submission.apply(&self.planes);
        let shown = match visible(&planes) {
            Ok(shown) => shown,
            Err(reason) => return self.refuse(tag, reason),
        };
        // Uncovered regions show the background, which scanout composes as
        // black. Every recorded WindowServer swap sends 0 or 0xff000000,
        // including its first, layerless one, so the high byte is read as
        // alpha (or unused) and black at any alpha is exact. The channel
        // order of any other color is not recorded; WindowServer retries a
        // failed swap forever, so it is a reported gap, shown black.
        if !swap::black_background(submission.background) {
            self.report_unmodeled(
                tag,
                Degraded::Ignored("the background is shown black"),
                None,
            );
        }
        let packed = Packed::new()
            .bool(false)
            .u32(0)
            .u32(IO_RETURN_SUCCESS)
            .finish(output);
        let reply = self.reply(tag, packed)?;
        // Commit the accepted swap only after every fallible preflight succeeds.
        let (_, client) = self.started.remove(start).expect("found started swap");
        self.log_placements(swap_id, &planes);
        self.planes = planes;
        let mut fenced = submission.fenced();
        self.early_fences.retain(|&(swap, surface)| {
            if swap != swap_id {
                return true;
            }
            for (index, &id) in submission.surface_ids.iter().enumerate() {
                if id == surface {
                    fenced &= !(1 << index);
                }
            }
            false
        });
        self.pacing.fence_waits += u32::from(fenced != 0);
        self.frames.push_back(Frame {
            swap_id,
            client,
            gpu_work: super::gpu::submitted(),
            fenced,
            surface_ids: submission.surface_ids,
            waiting_since: None,
            submitted_at: self.now,
            ready_at: (fenced == 0).then_some(self.now),
            planes: shown,
            updated: if submission.enabled & swap::REMOVE_LAYERS != 0 {
                (1 << swap::LAYERS) - 1
            } else {
                submission.enabled & ((1 << swap::LAYERS) - 1)
            },
        });
        self.pacing.submitted += 1;
        Ok(reply)
    }
}

impl Methods {
    /// Log each slot whose surface size, source rectangle or destination
    /// size `planes` changes, the first [`PLACEMENTS`] times: what the guest
    /// asks the scaler for. A destination that only moves (the cursor) is
    /// not a new shape.
    fn log_placements(&mut self, swap_id: u32, planes: &[Option<Layer>; swap::LAYERS]) {
        let shape = |layer: &Option<Layer>| {
            layer.map(|layer| {
                (
                    (layer.record.width, layer.record.height),
                    layer.source,
                    (layer.destination.width, layer.destination.height),
                )
            })
        };
        for (index, layer) in planes.iter().enumerate() {
            let Some(next) = layer else { continue };
            if self.placements >= PLACEMENTS || shape(layer) == shape(&self.planes[index]) {
                continue;
            }
            self.placements += 1;
            self.record_log(Log::Placement {
                swap_id,
                layer: index,
                surface: (next.record.width, next.record.height),
                source: next.source,
                destination: next.destination,
            });
        }
    }
}

/// What each slot shows, bottom to top, over a black background. A layer
/// whose surface or rectangles the model cannot show refuses by name.
fn visible(
    planes: &[Option<Layer>; swap::LAYERS],
) -> Result<[Option<scanout::Layer>; swap::LAYERS]> {
    let mut shown = [None; swap::LAYERS];
    for (shown, layer) in shown.iter_mut().zip(planes) {
        if let Some(layer) = layer {
            *shown = Some(scanout::Layer::new(
                layer.record.surface(layer.dva)?,
                layer.source,
                layer.destination,
                layer.record.premultiplied,
            )?);
        }
    }
    Ok(shown)
}

/// The rows of a `height`-row screen that differ between slots `before` and
/// `after`, where swaps set the slots in `updated` in between: those rows
/// a changed or set slot covered before or covers after. A slot no swap set
/// shows the same surface, which nothing draws in while it is on screen
/// (the AP releases a surface only once a later swap replaced it).
fn damage(
    before: &[Option<scanout::Layer>; swap::LAYERS],
    after: &[Option<scanout::Layer>; swap::LAYERS],
    updated: u32,
    height: u32,
) -> core::ops::Range<usize> {
    let mut rows = 0..0;
    for (slot, (before, after)) in before.iter().zip(after).enumerate() {
        if updated & 1 << slot == 0 && before == after {
            continue;
        }
        for layer in [before, after].into_iter().flatten() {
            let rect = layer.destination;
            let start = rect.y.min(height) as usize;
            let end = (u64::from(rect.y) + u64::from(rect.height)).min(height.into()) as usize;
            rows = scanout::union(rows, start..end);
        }
    }
    rows
}

/// A boolean DCP reply (the low bit set is true).
fn bool_reply(ok: bool) -> Vec<u8> {
    u32::from(ok).to_le_bytes().to_vec()
}

/// Answer a stateless query for the fixed `mode`, or `None` for a method with
/// no model. The output must be exactly the length the AP's header allocated
/// (the interface's size); a mismatch refuses rather than truncates.
pub fn answer(row: &Method, input: &[u8], output: u32, mode: Mode) -> Result<Option<Vec<u8>>> {
    let out = output as usize;
    let sized = |bytes: Vec<u8>| -> Result<Option<Vec<u8>>> {
        if bytes.len() == out {
            Ok(Some(bytes))
        } else {
            Err(refuse!(
                "DCP method output size differs from the AP's header"
            ))
        }
    };
    let width = mode.width.to_le_bytes().to_vec();
    let height = mode.height.to_le_bytes().to_vec();
    match (row.class, row.name) {
        // IOMobileFramebufferAP display geometry (out=4 each).
        (AP, "display_width") => sized(width),
        (AP, "display_height") => sized(height),
        // UnifiedPipeline2 aliases: displayHeight (A350), displayWidth (A351).
        (UP, "displayHeight") => sized(height),
        (UP, "displayWidth") => sized(width),
        // get_display_size(flags) -> {width, height} (in=4, out=8).
        (AP, "get_display_size") => {
            let mut bytes = mode.width.to_le_bytes().to_vec();
            bytes.extend_from_slice(&mode.height.to_le_bytes());
            sized(bytes)
        }
        // is_external: the one HDMI display is external.
        (UP, "is_external") => sized(bool_reply(true)),
        // headless: a display is connected, so not headless (0).
        (UP, "headless") => sized(bool_reply(false)),
        // is_main_display: the one connected display is main.
        (AP, "is_main_display") => sized(bool_reply(true)),
        // get_framebuffer_id: the single framebuffer's id.
        (AP, "get_framebuffer_id") => sized(1u32.to_le_bytes().to_vec()),
        // set_display_device(index) -> link status, not bool. The AP
        // wrapper (0x3d77988) passes this word to iomfb_status_from_link
        // (0x3d9cf30): 0 succeeds; 1 maps to kIOReturnError (0xe00002bc).
        // Original T6000 restore FW 0x26aed0 ignores the index and returns
        // success for its external trait (+0x4305); init 0x256310..2563e8
        // sets that trait for types 2/3 and publishes "external".
        (AP, "set_display_device") => sized(IO_RETURN_SUCCESS.to_le_bytes().to_vec()),
        // do_create_default_frame_buffer: the DCP does NOT create one; the
        // boot framebuffer is inherited, so this is 0 and the AP skips the
        // A036/A037 default-framebuffer-info path.
        (AP, "do_create_default_frame_buffer") => sized(bool_reply(false)),
        // set_create_DFB / stop_signal / free_signal: no output.
        (UP, "set_create_DFB") | (AP, "stop_signal") | (AP, "free_signal") => sized(Vec::new()),
        // flush_supportsPower(bool): no output.
        (AP, "flush_supportsPower") => sized(Vec::new()),
        // setDisplayRefreshProperties: success (out=4).
        (AP, "setDisplayRefreshProperties") => sized(bool_reply(true)),
        // apt_supported (UPPipeAP_H13P::late_init 0x2a4e98c): false. The
        // firmware answers pipe word +0x42c == 1 (restore 0x2bcad0, normal
        // 0x2bf884), which its init sets only to 3 or 0 (0x2c84b0, 0x2c8724).
        (H13P, "apt_supported") => sized(bool_reply(false)),
        // setSharedEventMemBlockSize(u32, u32) (UnifiedPipeline2::start
        // 0x2a5e02c): no output; the firmware only records the sizes in its
        // shared-event block (restore 0x1becd8, normal 0x1bf20c).
        (AP, "setSharedEventMemBlockSize") => sized(Vec::new()),
        // isKeepOnScreen (swapping_client_did_start 0x3d67534): false, the
        // firmware's byte +0x4304, which nothing sets (restore 0x1c8f30).
        (AP, "isKeepOnScreen") => sized(bool_reply(false)),
        // get_shared_disp_state (get_ap_shared_disp_state 0x3d6247c, asked
        // once AppleClamshellState exists): false, the pipe's byte +0x2ba8,
        // which nothing sets (restore 0x2c01d0, normal 0x2c2fb4).
        (AP, "get_shared_disp_state") => sized(bool_reply(false)),
        // remove_gain_maps (0x3d7a6a8): this monitor creates no gain map, so
        // the client's record state in the first eight bytes returns as it
        // came, valid byte +5 included, with success in the word at +8, which
        // the AP returns.
        (AP, "remove_gain_maps") => {
            let mut bytes = input.to_vec();
            if let Some(result) = bytes.get_mut(8..12) {
                result.copy_from_slice(&IO_RETURN_SUCCESS.to_le_bytes());
            }
            sized(bytes)
        }
        _ => Ok(None),
    }
}

#[cfg(test)]
mod tests;
