//! The guest's display on the firmware's framebuffer.
//!
//! iBoot hands XNU a linear framebuffer through `boot_args.Video` and the
//! DeviceTree's `/vram`; XNU's boot console and verbose output draw into it.
//! The mount gives the guest one of the screen mode's size (the mode
//! `console::mode` chose, which the DCP model can advertise), 32 bits per
//! pixel, at the top of guest DRAM, outside the range the kernel manages
//! (`stela_formats::boot_bundle::Machine`), and shows only the frames the
//! modeled DCP presents. XNU's 32-bit pixels are x8r8g8b8 with blue in the
//! low byte (public XNU `osfmk/console/video_console.c`), GOP's BGR order;
//! an RGB or channel-mask mode re-encodes each pixel.
use crate::Result;
use crate::console::{self, Framebuffer};
use crate::output;
use alloc::vec::Vec;
#[cfg(feature = "nvk")]
use alloc::{boxed::Box, vec};
use core::ops::Range;
use core::sync::atomic::{AtomicBool, AtomicU64, Ordering};
use stela_boot_core::staged_frame::StagedFrame;
use stela_contracts::refuse;
use stela_formats::boot_bundle;
use stela_soc::machine::{DisplayLog, Scanout, ScanoutFormat, ScanoutMode};
use stela_soc::scanout::union;

#[cfg(feature = "nvk")]
use super::gpu::client::{present_took, timed_present as timed};
/// Counts a present that took `ticks`; the GPU's reports count presents
/// only with a GPU.
#[cfg(not(feature = "nvk"))]
fn present_took(_ticks: u64) {}

/// Whether the DCP model has presented a swap: the first one takes the
/// screen from Stela's console.
static DCP_PRESENTED: AtomicBool = AtomicBool::new(false);

/// An install keeps the host console visible, including the guest UART.
/// Set before any guest PE runs, so framebuffer writers never race it.
static INSTALL_CONSOLE: AtomicBool = AtomicBool::new(false);

pub(super) fn install_console() {
    INSTALL_CONSOLE.store(true, Ordering::Release);
    output::arguments(format_args!(
        "DISPLAY install diagnostics v7: guest UART and restore heartbeat stay visible\n"
    ));
}

/// The host scanout the modeled DCP presents completed swaps to: the UEFI GOP
/// framebuffer at its own mode. The DCP model composes each swap's layers in
/// this scanout's pixel order (blue-low, which `Framebuffer::show_row` writes,
/// re-encoding it for an RGB or channel-mask mode), so a frame arrives here
/// as rows ready for the firmware's mode.
///
/// The model presents under the devices' lock, and video memory is slow to
/// write (a hypervisor that tracks its writes takes a fault for each page),
/// so a present only fills the frame kept in RAM
/// ([`stela_boot_core::staged_frame`]): its rows reach the screen from the
/// PEs' work between slices ([`flush`]), outside that lock, mostly from
/// waiting PEs and in bounded steps from busy ones other than the boot PE.
///
/// Where the display has two frames (a virtual display's DISPI registers), the rows go
/// into the hidden one, which is then shown, and only the rows that frame
/// lacks since it last held a frame.
pub struct GopScanout {
    mode: ScanoutMode,
    refresh_millihertz: u32,
    size_mm: Option<(u32, u32)>,
}

/// The screen and the frame kept for it ([`GopScanout`]).
struct Staged {
    screen: Framebuffer,
    frame: StagedFrame,
    /// For each of the screen's two frames, the rows that differ from the
    /// kept frame as the screen last took it ([`StagedFrame::take`]).
    stale: [Range<usize>; 2],
    /// Whether the hidden frame is being brought up to the kept one, since
    /// when, and the ticks that has taken so far.
    writing: bool,
    started: u64,
    spent: u64,
    /// The row copied before it is written to video memory.
    row: Vec<u8>,
    /// Whether a refused frame was reported.
    refused: bool,
}

/// The one screen's [`Staged`], which the model's presents and the PEs'
/// flushes take in turn.
struct StagedScreen {
    held: AtomicBool,
    staged: core::cell::UnsafeCell<Option<Staged>>,
}

// SAFETY: `held` admits one user of `staged` at a time ([`with_staged`]).
unsafe impl Sync for StagedScreen {}

static STAGED: StagedScreen = StagedScreen {
    held: AtomicBool::new(false),
    staged: core::cell::UnsafeCell::new(None),
};

/// Run `f` on the staged screen, after any other processor's use of it: a
/// present fills rows in RAM and a flush writes [`FLUSH_ROWS`] rows at a
/// time, so a wait is a fraction of a millisecond.
fn with_staged<R>(f: impl FnOnce(&mut Option<Staged>) -> R) -> R {
    while STAGED
        .held
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        core::hint::spin_loop();
    }
    // SAFETY: `held` excludes every other reference until it is released.
    let result = f(unsafe { &mut *STAGED.staged.get() });
    STAGED.held.store(false, Ordering::Release);
    result
}

/// The rows of video memory one turn at the staged screen writes: what a
/// present under the devices' lock, or a waiting PE's interrupt, may wait
/// for, and what a busy PE gives the screen between two slices.
const FLUSH_ROWS: usize = 64;

/// A busy PE gives the screen at most one part in this of its time
/// ([`Turn::Busy`]).
pub const BUSY_SHARE: u64 = 16;

/// How long a frame may stay due with no PE taking it before the boot PE
/// does, and how long one may take to write before the next PE to come
/// finishes it whole: about a frame at 60 Hz, so a frame being written
/// reaches the screen promptly whoever started it.
const LATE_MICROS: u64 = 16_000;

/// The host counter's ticks per second, which the run tells before its
/// first frame: the firmware's own record of it is gone by then. Until it
/// is told, every pending frame counts as late.
static TICK_RATE: AtomicU64 = AtomicU64::new(0);

/// The host counter runs at `hz` ticks per second.
pub fn set_tick_rate(hz: u64) {
    TICK_RATE.store(hz, Ordering::Relaxed);
}

/// Who gives the screen its rows ([`flush`]).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Turn {
    /// The one processor, with guest code to run: frames as their share
    /// allows ([`StagedFrame::due`]), each whole.
    Alone,
    /// The boot PE, which the GPUs' work and the devices also need: the
    /// first frame, which takes the screen from the console, and frames no
    /// other PE took or finished in time.
    Home,
    /// Another PE between two slices: one step of [`FLUSH_ROWS`] rows, of a
    /// frame whose share allows it, within its own [`BUSY_SHARE`].
    Busy,
    /// A PE that waits for an interrupt: until one comes for it. It has no
    /// guest code to run, so the screen then costs the guest nothing.
    Idle,
}

/// Give the screen the rows its kept frame changed, holding no lock of the
/// devices', as `turn` does; the ticks it took.
pub fn flush(turn: Turn) -> u64 {
    if INSTALL_CONSOLE.load(Ordering::Acquire) {
        return 0;
    }
    let begun = super::host::rdtsc();
    loop {
        if STAGED.held.load(Ordering::Relaxed) {
            break;
        }
        let more = with_staged(|staged| {
            let staged = staged.as_mut()?;
            let started = super::host::rdtsc();
            let stepped = staged.step(turn, started);
            let finished = super::host::rdtsc();
            staged.spent += finished.wrapping_sub(started);
            match stepped {
                Ok(Step::More { whole }) => Some(whole),
                Ok(Step::Idle) => None,
                Ok(Step::Shown) => {
                    let spent = core::mem::take(&mut staged.spent);
                    present_took(spent);
                    staged.frame.spent(spent, finished);
                    None
                }
                Err(reason) => {
                    // The frame stays hidden; a later present tries again.
                    staged.writing = false;
                    staged.spent = 0;
                    if !core::mem::replace(&mut staged.refused, true) {
                        output::arguments(format_args!("DISPLAY refused a frame: {reason}\n"));
                    }
                    None
                }
            }
        });
        let Some(whole) = more else {
            break;
        };
        let stop = match turn {
            Turn::Busy => !whole,
            Turn::Idle => !whole && super::host::interrupted(),
            Turn::Alone | Turn::Home => false,
        };
        if stop {
            break;
        }
    }
    super::host::rdtsc().wrapping_sub(begun)
}

/// What a turn at the staged screen left ([`Staged::step`]).
enum Step {
    /// Nothing to write, or not this PE's to write now.
    Idle,
    /// Rows written; the hidden frame lacks more, which must be written
    /// `whole` now, the frame being late.
    More { whole: bool },
    /// The hidden frame was completed and shown.
    Shown,
}

impl Staged {
    /// Write the next [`FLUSH_ROWS`] rows the hidden frame lacks, and show
    /// it once it lacks none. A display with one frame takes the kept frame
    /// whole. A frame starts with the rows changed until then; those that
    /// change while it is written wait for the next, so a frame ends.
    fn step(&mut self, turn: Turn, now: u64) -> Result<Step> {
        let late = TICK_RATE.load(Ordering::Relaxed) / 1000 * LATE_MICROS / 1000;
        if !self.writing {
            let first = !DCP_PRESENTED.load(Ordering::Relaxed);
            let may = match turn {
                Turn::Alone => self.frame.due(now),
                // Only the boot PE takes the screen from the console.
                Turn::Home => (first && self.frame.due(now)) || self.frame.overdue(now, late),
                Turn::Busy => !first && self.frame.due(now),
                Turn::Idle => !first && self.frame.pending(),
            };
            if !may {
                return Ok(Step::Idle);
            }
            self.writing = true;
            self.started = now;
            let pending = self.frame.take();
            for stale in &mut self.stale {
                *stale = union(stale.clone(), pending.clone());
            }
        } else if turn == Turn::Home && now.wrapping_sub(self.started) < late {
            // Another PE writes it.
            return Ok(Step::Idle);
        }
        let (width, height) = self.screen.resolution();
        let row_bytes = width * 4;
        let frame = &self.frame;
        let mut fill = |y: usize, row: &mut [u8]| {
            row[..row_bytes].copy_from_slice(frame.row(y));
            Ok(())
        };
        let Some(hidden) = self.screen.hidden_half() else {
            // One frame in video memory, which holds what both would.
            if !self.screen.show_frame(self.frame.frame(), take_screen) {
                return Err(refuse!("firmware framebuffer access was refused"));
            }
            self.stale = [0..0, 0..0];
            self.writing = false;
            return Ok(Step::Shown);
        };
        self.row.resize(row_bytes, 0);
        let lacking = self.stale[hidden].start..self.stale[hidden].end.min(height);
        let rows = lacking.start..lacking.end.min(lacking.start + FLUSH_ROWS);
        self.screen
            .fill_rows(hidden, rows.clone(), &mut self.row, &mut fill)?;
        self.stale[hidden] = rows.end..lacking.end;
        if !self.stale[hidden].is_empty() {
            return Ok(Step::More {
                whole: now.wrapping_sub(self.started) >= late,
            });
        }
        self.stale[hidden] = 0..0;
        self.screen
            .show_half(hidden, &mut self.row, &mut fill, take_screen)?;
        self.writing = false;
        Ok(Step::Shown)
    }
}

/// Take the screen from Stela's console for the DCP's first frame.
fn take_screen() -> bool {
    if !DCP_PRESENTED.load(Ordering::Relaxed) {
        if !console::release() {
            return false;
        }
        DCP_PRESENTED.store(true, Ordering::Relaxed);
    }
    true
}

/// Print one display-log entry: what the guest sent the DCP model. Each
/// line starts `DCP `.
pub fn print_log(entry: &DisplayLog) {
    output::arguments(format_args!("{entry}\n"));
}

/// `--scanout-mirror`: any scanout, with its newest frame kept at one address
/// that the boot log names. A debugger reads it from there, as QEMU's `memsave`
/// does; a frame's rows are blue-low `width` pixels top to bottom.
/// Its presents are timed for the GPU's reports.
#[cfg(feature = "nvk")]
pub struct MirroredScanout {
    scanout: Box<dyn Scanout>,
    frame: Box<[u8]>,
}

#[cfg(feature = "nvk")]
impl MirroredScanout {
    pub fn new(scanout: Box<dyn Scanout>) -> Self {
        Self {
            scanout,
            frame: Box::default(),
        }
    }

    /// Size the mirror for frames of `bytes`, naming its address when it
    /// moves; whether it is new.
    fn fit(&mut self, bytes: usize) -> bool {
        if self.frame.len() == bytes {
            return false;
        }
        self.frame = vec![0; bytes].into_boxed_slice();
        let mode = self.mode();
        output::arguments(format_args!(
            "DISPLAY mirror address={:#x} bytes={:#x} mode={}x{}\n",
            self.frame.as_ptr() as usize,
            bytes,
            mode.width,
            mode.height,
        ));
        true
    }
}

#[cfg(feature = "nvk")]
impl Scanout for MirroredScanout {
    fn mode(&self) -> ScanoutMode {
        self.scanout.mode()
    }

    fn refresh_millihertz(&self) -> u32 {
        self.scanout.refresh_millihertz()
    }

    fn physical_size_mm(&self) -> Option<(u32, u32)> {
        self.scanout.physical_size_mm()
    }

    fn present(&mut self, rows: &[u8]) -> Result<()> {
        timed(|| {
            self.fit(rows.len());
            self.frame.copy_from_slice(rows);
            self.scanout.present(rows)
        })
    }

    /// The rows the scanout takes from the composer are copied as they pass,
    /// so a frame costs the mirror only the rows it changed, as it costs the
    /// scanout, rather than a whole-frame composition and copy.
    fn try_present_rows(
        &mut self,
        damage: Option<Range<usize>>,
        fill: &mut dyn FnMut(usize, &mut [u8]) -> Result<()>,
    ) -> Result<bool> {
        let mode = self.mode();
        let row_bytes = mode.width as usize * 4;
        // A new mirror holds no frame yet: every row is needed.
        let damage = if self.fit(row_bytes * mode.height as usize) {
            None
        } else {
            damage
        };
        let frame = &mut self.frame;
        let scanout = &mut self.scanout;
        timed(|| {
            scanout.try_present_rows(damage, &mut |y, row| {
                fill(y, row)?;
                if let (Some(kept), Some(row)) = (
                    frame.get_mut(y * row_bytes..(y + 1) * row_bytes),
                    row.get(..row_bytes),
                ) {
                    kept.copy_from_slice(row);
                }
                Ok(())
            })
        })
    }

    fn queued(&self) -> bool {
        self.scanout.queued()
    }

    fn flush(&mut self) -> Result<()> {
        timed(|| self.scanout.flush())
    }
}

impl GopScanout {
    /// A scanout over `screen`, already relocated to the engine's physmap.
    /// With `mirror` (`--scanout-mirror`) the boot log names the address of
    /// its kept frame, which a debugger reads as QEMU's `memsave` does: its
    /// rows are blue-low `width` pixels top to bottom.
    pub fn new(screen: Framebuffer, mirror: bool) -> Self {
        let (width, height) = screen.resolution();
        let scanout = Self {
            mode: ScanoutMode {
                width: width as u32,
                height: height as u32,
                // show_row takes blue-low pixels and re-encodes them for the mode.
                format: ScanoutFormat::Bgra8888,
            },
            refresh_millihertz: screen.refresh_millihertz(),
            size_mm: screen.size_mm(),
        };
        let frame = StagedFrame::new(width * 4, height);
        if mirror {
            output::arguments(format_args!(
                "DISPLAY mirror address={:#x} bytes={:#x} mode={width}x{height}\n",
                frame.address(),
                frame.bytes(),
            ));
        }
        // Both of the screen's frames hold the console's text, not a
        // presented frame, and the kept frame nothing to show yet.
        let mut frame = frame;
        frame.take();
        with_staged(|staged| {
            *staged = Some(Staged {
                screen,
                frame,
                stale: [0..height, 0..height],
                writing: false,
                started: 0,
                spent: 0,
                row: Vec::new(),
                refused: false,
            });
        });
        scanout
    }
}

impl Scanout for GopScanout {
    fn mode(&self) -> ScanoutMode {
        self.mode
    }

    fn refresh_millihertz(&self) -> u32 {
        self.refresh_millihertz
    }

    fn physical_size_mm(&self) -> Option<(u32, u32)> {
        self.size_mm
    }

    fn present(&mut self, rows: &[u8]) -> Result<()> {
        let frame = self.mode.width as usize * 4 * self.mode.height as usize;
        if rows.len() < frame {
            return Err(refuse!("presented frame is smaller than the scanout mode"));
        }
        with_staged(|staged| match staged {
            Some(staged) => {
                staged.frame.store(rows);
                Ok(())
            }
            None => Err(refuse!("the firmware framebuffer has no staged frame")),
        })
    }

    fn try_present_rows(
        &mut self,
        damage: Option<Range<usize>>,
        fill: &mut dyn FnMut(usize, &mut [u8]) -> Result<()>,
    ) -> Result<bool> {
        with_staged(|staged| match staged {
            Some(staged) => staged.frame.fill(damage, fill).map(|()| true),
            None => Ok(false),
        })
    }
}

const BYTES_PER_PIXEL: u64 = 4;
/// The guest framebuffer is whole 16 KiB pages.
const PAGE: u64 = 0x4000;

/// The kernel's boot framebuffer for a `width` by `height` screen, ending at
/// `dram_end`. Nothing copies it to the screen: the kernel's boot console
/// and verbose text there reach the boot log through the debug UART, and
/// Stela's own console keeps the screen until the DCP presents macOS's
/// first frame ([`GopScanout`]).
pub fn boot_framebuffer(
    (width, height): (usize, usize),
    dram_end: u64,
) -> Option<boot_bundle::Framebuffer> {
    let (width, height) = (width as u64, height as u64);
    let row_bytes = width * BYTES_PER_PIXEL;
    let bytes = (row_bytes * height).next_multiple_of(PAGE);
    Some(boot_bundle::Framebuffer {
        guest_pa: dram_end.checked_sub(bytes)?,
        bytes,
        row_bytes,
        width,
        height,
        bits_per_pixel: BYTES_PER_PIXEL * 8,
    })
}
