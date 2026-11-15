//! What the screen says while Stela starts: one line per step, in the style
//! of a Linux boot, while every detail goes to the UART and the boot log
//! ([`output`]). A running step's line shows a moving marker, its time so
//! far and its progress, with lines such as macOS's console output under
//! it ([`stream`]), and its result replaces them when it ends:
//!
//! ```text
//!  [  ok  ]  Read Install macOS (484 MB, 12 s)
//!  [ ***  ]  Starting the macOS installer (46 s)  installer: fdr create
//!            AppleNVMe: controller ready
//! ```
//!
//! A finished line too long for the screen wraps under its text.
//!
//! [`tick`] moves the marker, at most eight frames a second: the firmware's
//! timer calls it while boot services run, and the host drivers' waits and
//! the engine's slices after. Finished lines are kept as [`history`], which
//! the loader hands to the system engine so its screen carries on from them
//! ([`replay`]). Each step's start and end store the boot logs, so a log
//! read after a hang ends at the step that hung.

use crate::{
    console::{self, Color},
    output,
};
use alloc::{format, string::String, vec::Vec};
use core::{
    arch::x86_64::_rdtsc,
    fmt::{self, Write as _},
};
use stela_boot_core::screen::Exclusive;

/// Frames of the running marker: three cells moving across six and back.
const FRAMES: usize = 14;
const MARKER: usize = 6;
/// The finished lines kept for the system engine's screen.
pub const HISTORY_BYTES: usize = 4096;
/// The most lines [`stream`] shows under the running step.
pub const STREAM_LINES: usize = 40;
/// The width of a step's mark and the gap after it, where its text starts:
/// info and streamed lines start there too.
const TEXT_INDENT: &str = "           ";
/// A running step's detail held this long is logged with its time
/// ([`State::detail_ends`]).
const DETAIL_LOG_MILLIS: u64 = 250;

/// Text formatted on the stack: drawing a timer tick allocates nothing.
struct StackText<const N: usize> {
    bytes: [u8; N],
    len: usize,
}

type Short = StackText<32>;

impl<const N: usize> Default for StackText<N> {
    fn default() -> Self {
        Self {
            bytes: [0; N],
            len: 0,
        }
    }
}

impl<const N: usize> StackText<N> {
    fn as_str(&self) -> &str {
        core::str::from_utf8(&self.bytes[..self.len]).unwrap_or_default()
    }
}

impl<const N: usize> fmt::Write for StackText<N> {
    fn write_str(&mut self, text: &str) -> fmt::Result {
        let end = self.len + text.len();
        if end > self.bytes.len() {
            return Err(fmt::Error);
        }
        self.bytes[self.len..end].copy_from_slice(text.as_bytes());
        self.len = end;
        Ok(())
    }
}

struct Running {
    title: String,
    detail: String,
    /// When `detail` last changed, on the TSC.
    detail_started: u64,
    percentage: Option<u8>,
    started: u64,
    frame: usize,
    drawn: u64,
}

struct State {
    tsc_hz: u64,
    running: Option<Running>,
    /// The lines under the running step, oldest first, and whether the
    /// screen shows them yet.
    stream: Vec<String>,
    streamed: bool,
    /// Finished lines, each its mark's letter then its text.
    history: String,
}

/// The boot PE's reports and the host drivers' waits on any processor
/// ([`tick`]) reach the state, one at a time: a marker frame due while it
/// is in use is skipped, and a report waits at most [`REPORT_WAIT_SPINS`]
/// tries for its turn, so a step's end or a failure line is not lost to
/// another processor's tick.
static STATE: Exclusive<State> = Exclusive::new(State {
    tsc_hz: 0,
    running: None,
    stream: Vec::new(),
    streamed: true,
    history: String::new(),
});

fn now() -> u64 {
    // SAFETY: RDTSC has no memory effects.
    unsafe { _rdtsc() }
}

#[derive(Clone, Copy)]
enum Mark {
    Ok,
    Warn,
    Failed,
    Info,
}

impl Mark {
    fn letter(self) -> char {
        match self {
            Mark::Ok => 'o',
            Mark::Warn => 'w',
            Mark::Failed => 'f',
            Mark::Info => 'i',
        }
    }

    fn from_letter(letter: char) -> Option<Self> {
        Some(match letter {
            'o' => Mark::Ok,
            'w' => Mark::Warn,
            'f' => Mark::Failed,
            'i' => Mark::Info,
            _ => return None,
        })
    }

    fn draw(self, text: &str) {
        let (label, color) = match self {
            Mark::Ok => ("  ok  ", Color::Good),
            Mark::Warn => (" warn ", Color::Warn),
            Mark::Failed => (" fail ", Color::Fatal),
            Mark::Info => {
                console::rewrite_wrapped(
                    &[(TEXT_INDENT, Color::Text), (text, Color::Text)],
                    TEXT_INDENT.len(),
                );
                console::write(&[("\n", Color::Text)]);
                return;
            }
        };
        console::rewrite_wrapped(
            &[
                (" [", Color::Dim),
                (label, color),
                ("]  ", Color::Dim),
                (text, Color::Text),
            ],
            TEXT_INDENT.len(),
        );
        console::write(&[("\n", Color::Text)]);
    }
}

impl State {
    /// Whole seconds since TSC `since`; none before the rate is known.
    fn seconds(&self, since: u64) -> u64 {
        match self.tsc_hz {
            0 => 0,
            hz => (now() - since) / hz,
        }
    }

    /// Milliseconds since TSC `since`; none before the rate is known.
    fn millis(&self, since: u64) -> u64 {
        match self.tsc_hz {
            0 => 0,
            hz => (now() - since) / (hz / 1000).max(1),
        }
    }

    /// The running step's detail gives way to `next` (empty at the step's
    /// end): one that held for [`DETAIL_LOG_MILLIS`] or more is logged with
    /// its time, such as a device a driver took long to start, which the
    /// step's own time does not single out.
    fn detail_ends(&mut self, next: &str) {
        let Some(running) = self.running.as_ref() else {
            return;
        };
        if running.detail == next {
            return;
        }
        let millis = self.millis(running.detail_started);
        if !running.detail.is_empty() && millis >= DETAIL_LOG_MILLIS {
            output::arguments(format_args!("STEP DETAIL {} ms={millis}\n", running.detail));
        }
        if let Some(running) = self.running.as_mut() {
            running.detail_started = now();
        }
    }

    fn draw_running(&mut self) {
        let Some(running) = self.running.as_ref() else {
            return;
        };
        let step = running.frame % FRAMES;
        let head = if step < FRAMES / 2 {
            step
        } else {
            FRAMES - step
        };
        let mut marker = [b' '; MARKER];
        for (cell, byte) in marker.iter_mut().enumerate() {
            if cell <= head && head < cell + 3 {
                *byte = b'*';
            }
        }
        let mut percentage = Short::default();
        if let Some(value) = running.percentage {
            let _ = write!(percentage, " ({value}%)");
        }
        let marker = core::str::from_utf8(&marker).unwrap_or("      ");
        let seconds = self.seconds(running.started);
        let mut elapsed = Short::default();
        if seconds != 0 {
            let _ = write!(elapsed, " ({seconds} s)");
        }
        let gap = if running.detail.is_empty() { "" } else { "  " };
        console::rewrite(&[
            (" [", Color::Dim),
            (marker, Color::Busy),
            ("]  ", Color::Dim),
            (&running.title, Color::Text),
            (elapsed.as_str(), Color::Dim),
            (gap, Color::Text),
            (&running.detail, Color::Text),
            (percentage.as_str(), Color::Busy),
        ]);
        if !self.streamed {
            console::below(&self.stream, TEXT_INDENT.len(), Color::Dim);
            self.streamed = true;
        }
    }

    fn finish(&mut self, mark: Mark, result: &str) {
        self.detail_ends("");
        // A step a warning already ended has nothing left to end; a
        // failure is shown on its own.
        let Some(running) = self.running.take() else {
            if matches!(mark, Mark::Failed) {
                self.line(mark, result);
            }
            return;
        };
        let seconds = self.seconds(running.started);
        let millis = self.millis(running.started);
        let mut text = running.title;
        // A failed step keeps what it was on, such as the PCI function a
        // probe stopped in: before the engine resumes the boot log, the
        // screen shows it whatever becomes of the stop's record.
        if matches!(mark, Mark::Failed) && !running.detail.is_empty() {
            text.push_str(" at ");
            text.push_str(&running.detail);
        }
        match (result.is_empty(), seconds) {
            (true, 0) => {}
            (true, seconds) => text.push_str(&format!(" ({seconds} s)")),
            (false, 0) => text.push_str(&format!(" ({result})")),
            (false, seconds) => text.push_str(&format!(" ({result}, {seconds} s)")),
        }
        // The log's line carries the exact time, which the screen's rounds.
        output::arguments(format_args!("STEP {} {text} ms={millis}\n", mark.letter()));
        self.line(mark, &text);
    }

    fn line(&mut self, mark: Mark, text: &str) {
        if !self.stream.is_empty() {
            self.stream.clear();
            console::below(&[], 0, Color::Dim);
        }
        mark.draw(text);
        let line = format!("{}{text}\n", mark.letter());
        while self.history.len() + line.len() > HISTORY_BYTES {
            let Some(end) = self.history.find('\n') else {
                self.history.clear();
                break;
            };
            self.history.drain(..=end);
        }
        if line.len() <= HISTORY_BYTES {
            self.history.push_str(&line);
        }
        // The running step's line, which the finished one took, follows it.
        self.draw_running();
    }
}

/// Tries a report makes for the state before it is skipped: far longer
/// than another processor's frame takes, and still short when the holder
/// is this processor further up its stack.
const REPORT_WAIT_SPINS: u64 = 1 << 22;

fn with<R>(f: impl FnOnce(&mut State) -> R) -> Option<R> {
    let mut spins = 0u64;
    STATE.with_until(
        || {
            spins += 1;
            spins >= REPORT_WAIT_SPINS
        },
        f,
    )
}

/// The time-stamp counter's rate, for the times steps show.
pub fn set_clock(tsc_hz: u64) {
    with(|state| state.tsc_hz = tsc_hz);
    output::set_clock(tsc_hz);
}

/// Start a step; one still running ends first, as done.
pub fn begin(title: &str) {
    with(|state| {
        if state.running.is_some() {
            state.finish(Mark::Ok, "");
        }
        // Seconds since the PC started, which the serial port's lines lack.
        let millis = state.millis(0);
        output::arguments(format_args!(
            "STEP {title} t={}.{:03}\n",
            millis / 1000,
            millis % 1000
        ));
        state.running = Some(Running {
            title: String::from(title),
            detail: String::new(),
            detail_started: now(),
            percentage: None,
            started: now(),
            frame: 0,
            drawn: now(),
        });
        state.draw_running();
    });
    output::flush_log();
}

/// Report the EBS handoff without allocating or touching firmware services.
/// The ticker must already be stopped. Unlike `progress`, this does not
/// grow the running step's String, or require a running step to exist.
/// The console updates only its existing cell buffers and framebuffer.
pub fn handoff_progress(detail: fmt::Arguments<'_>) {
    let mut text = StackText::<160>::default();
    let _ = text.write_fmt(detail);
    // A later failure must not attribute this phase to a stale pre-EBS
    // detail. Clearing existing storage cannot allocate.
    with(|state| {
        if let Some(running) = state.running.as_mut() {
            running.detail.clear();
            running.percentage = None;
        }
    });
    output::arguments(format_args!("HOST HANDOFF {}\n", text.as_str()));
    console::rewrite(&[(" [ ***  ]  ", Color::Busy), (text.as_str(), Color::Text)]);
}

/// Show how far the running step has come, such as `182 of 484 MB`.
pub fn progress(detail: &str) {
    with(|state| {
        state.detail_ends(detail);
        if let Some(running) = state.running.as_mut() {
            running.detail.clear();
            running.detail.push_str(detail);
            running.percentage = None;
            state.draw_running();
        }
    });
}

/// Show a percentage for the finite operation named by `detail`, such
/// as reading one file with a known byte length. This measures only that
/// operation, never an estimate of total boot progress. An unknown/invalid
/// total keeps the ordinary moving marker and omits the percentage.
pub fn progress_fraction(detail: &str, completed: u64, total: u64) {
    with(|state| {
        state.detail_ends(detail);
        if let Some(running) = state.running.as_mut() {
            running.detail.clear();
            running.detail.push_str(detail);
            running.percentage = stela_boot_core::screen::percentage(completed, total);
            state.draw_running();
        }
    });
}

/// Move the running step's marker, at most eight frames a second, and send
/// the serial port what it takes now (`output::pump_serial`). Safe from the
/// firmware timer's callback: it allocates nothing and skips a frame while
/// the state or the console is in use.
pub fn tick() {
    output::pump_serial();
    STATE.try_with(|state| {
        let frame = state.tsc_hz / 8;
        let Some(running) = state.running.as_mut().filter(|_| frame > 0) else {
            return;
        };
        if now() - running.drawn < frame {
            return;
        }
        running.drawn = now();
        running.frame += 1;
        state.draw_running();
    });
}

/// Show `lines`, such as macOS's console output, under the running step,
/// the newest lowest; the next [`tick`] draws them, and the step's end
/// clears them.
pub fn stream<'a>(lines: impl IntoIterator<Item = &'a str>) {
    with(|state| {
        if state.running.is_none() {
            return;
        }
        for line in lines {
            if state.stream.len() == STREAM_LINES {
                state.stream.remove(0);
            }
            state.stream.push(String::from(line.trim_end()));
            state.streamed = false;
        }
    });
}

/// Show `action` as a step counting down `seconds`, and wait them out.
pub fn countdown(action: &str, seconds: u64) {
    let mut hz = 0;
    with(|state| hz = state.tsc_hz);
    begin(action);
    let start = now();
    let mut left = u64::MAX;
    while now() - start < seconds * hz {
        let remaining = seconds - (now() - start) / hz.max(1);
        if remaining != left {
            left = remaining;
            progress(&format!("in {left} s"));
        }
        tick();
        core::hint::spin_loop();
    }
    done("");
}

/// End the running step with `result`, which may be empty.
pub fn done(result: &str) {
    with(|state| state.finish(Mark::Ok, result));
    output::flush_log();
}

/// End the running step as having gone wrong in a way that lets the boot
/// go on.
pub fn warn(result: &str) {
    with(|state| state.finish(Mark::Warn, result));
    output::flush_log();
}

/// End the running step as failed, or show `result` as a failure.
pub fn fail(result: &str) {
    with(|state| state.finish(Mark::Failed, result));
    output::flush_log();
}

/// A line of its own, under the step lines.
pub fn info(text: &str) {
    with(|state| state.line(Mark::Info, text));
}

/// A warning line of its own, with no step to end, as for what an earlier
/// start left.
pub fn warning(text: &str) {
    with(|state| state.line(Mark::Warn, text));
}

/// Draw with Stela's console on `screen` from now on, starting with the
/// finished lines so far and the running step.
pub fn take_screen(screen: Option<console::Framebuffer>) {
    let Some(screen) = screen else {
        return;
    };
    console::install(screen);
    with(|state| {
        for line in state.history.lines() {
            let mut characters = line.chars();
            if let Some(mark) = characters.next().and_then(Mark::from_letter) {
                mark.draw(characters.as_str());
            }
        }
        state.draw_running();
    });
}

/// The finished lines so far.
pub fn history(into: &mut [u8; HISTORY_BYTES]) -> usize {
    with(|state| {
        let bytes = state.history.len();
        into[..bytes].copy_from_slice(state.history.as_bytes());
        bytes
    })
    .unwrap_or(0)
}

/// Draw the loader's finished lines again, as the system engine's screen
/// starts.
pub fn replay(history: &[u8]) {
    let text = core::str::from_utf8(history).unwrap_or_default();
    for line in text.lines() {
        let mut characters = line.chars();
        if let Some(mark) = characters.next().and_then(Mark::from_letter) {
            with(|state| state.line(mark, characters.as_str()));
        }
    }
}
