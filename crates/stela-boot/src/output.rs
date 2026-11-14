//! The mount's detailed output: every line goes to the UART and, from the
//! very first line, into a bounded record for the persistent boot logs
//! (`pc::boot_log`): the USB's `EFI/stela/boot.log` ([`crate::log_file`])
//! and the machine state's, once `--nor` opens it. [`add_log`] adds one,
//! which starts with everything the record still holds, and [`flush_log`]
//! stores what each has not: at every step and at every stop, which
//! [`terminate`] reaches. Between engine slices [`flush_due`] stores the
//! machine state's log every [`MACHINE_FLUSH_SECONDS`], as ext4 commits its
//! journal, and the USB's, slow flash the guest waits on, only every
//! [`USB_FLUSH_SECONDS`]. A stop while the USB's log is not kept, as between
//! the end of the firmware's boot services and the start of Stela's own
//! driver for the stick, leaves the record's newest lines in the PC's
//! firmware instead ([`crate::stop_record`]). The screen shows the curated
//! steps of [`status`](crate::status) instead, and a fatal error or panic, which
//! reach it before the console exists through the firmware's.
use crate::{console, serial::Serial};
use alloc::{boxed::Box, vec::Vec};
use core::{
    cell::RefCell,
    convert::Infallible,
    fmt::{self, Write},
    mem::MaybeUninit,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use stela_drivers::block::{BlockDevice, BlockError, Submission};
use stela_runtime::pc::boot_log::{PENDING_BYTES, Pending, Unkept, Writer};

mod terminal;
pub(crate) use terminal::{Callers, Code, panic, print_stack_at};

/// One engine-wide value. The mount runs on one processor and its handlers
/// only print; every access is a `try_borrow`, so output that arrives while
/// the value is in use (a line printed during a flush) is skipped instead of
/// re-entering it. Values other processors reach too are reached only under
/// a lock: the record under [`WRITING`], the guest's lines under the board's.
pub(crate) struct Global<T>(pub(crate) RefCell<T>);
// SAFETY: one processor at a time reaches each value (above), and no access
// blocks or assumes the value is free.
unsafe impl<T> Sync for Global<T> {}

/// A processor writes the record and the serial port. Others wait for it at
/// most [`WRITING_WAIT_MICROS`], longer than a long line takes at the serial
/// port's speed, then write to the serial port alone: the holder may be this
/// processor itself, interrupted or failing inside a write.
static WRITING: AtomicBool = AtomicBool::new(false);
const WRITING_WAIT_MICROS: u64 = 250_000;
/// How long a log store that may not wait ([`flush_due`]) waits for the
/// record: the copy of its new bytes goes to the next store instead.
const STORE_COPY_WAIT_MICROS: u64 = 1000;
/// Spins to wait while the counter's rate is not known yet.
const WRITING_WAIT_SPINS: u64 = 1 << 22;

/// Run `f` while no other processor writes the record; `None` when the
/// wait ran out.
fn exclusive<R>(f: impl FnOnce() -> R) -> Option<R> {
    exclusive_within(WRITING_WAIT_MICROS, f)
}

/// [`exclusive`], waiting `micros` at most.
fn exclusive_within<R>(micros: u64, f: impl FnOnce() -> R) -> Option<R> {
    let hz = TSC_HZ.load(Ordering::Relaxed);
    let started = rdtsc();
    let mut spins = 0u64;
    while WRITING
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        spins += 1;
        let waited = match hz {
            0 => spins >= WRITING_WAIT_SPINS,
            hz => rdtsc().wrapping_sub(started) >= hz / 1_000_000 * micros,
        };
        if waited {
            return None;
        }
        core::hint::spin_loop();
    }
    let result = f();
    WRITING.store(false, Ordering::Release);
    Some(result)
}

/// Output for the serial port, queued from the first line. The port takes
/// 16 bytes per transmit FIFO, each burst a port access a hypervisor traps
/// and a real port drains at its baud rate (about 11.5 KB/s), so output
/// sent where it is printed would hold the boot, or the PE it runs, for
/// seconds. A writer sends only what the port takes at once; the rest goes
/// out at later lines, at the marker's ticks, and from PEs that wait for an
/// interrupt anyway ([`send_queued`]). Until [`queue_serial`] the queue is
/// a fixed ring in the image ([`EARLY_RING`]), since the heap may not exist
/// yet. Reached only under [`WRITING`], which covers copies into and out of
/// the queue and never waits for the port.
static SERIAL_QUEUE: Global<Option<Queue>> = Global(RefCell::new(None));
const SERIAL_QUEUE_BYTES: usize = 8 << 20;
/// The ring the serial port's output waits in before [`queue_serial`].
const EARLY_RING_BYTES: usize = 64 << 10;
struct EarlyRing(core::cell::UnsafeCell<[MaybeUninit<u8>; EARLY_RING_BYTES]>);
// SAFETY: taken once ([`EARLY_TAKEN`]), then reached only as the queue's
// storage, under [`WRITING`].
unsafe impl Sync for EarlyRing {}
static EARLY_RING: EarlyRing = EarlyRing(core::cell::UnsafeCell::new(
    [MaybeUninit::uninit(); EARLY_RING_BYTES],
));
static EARLY_TAKEN: AtomicBool = AtomicBool::new(false);
/// The queue is gone ([`unqueue_serial`]): the boot is stopping or has left
/// this image's boot services, and later lines go to the port directly.
static SERIAL_DIRECT: AtomicBool = AtomicBool::new(false);
/// How long a stop drains the queue before it judges the port's speed.
const DRAIN_PROBE_MILLIS: u64 = 100;
/// A port that takes less than this many bytes a second is a real UART,
/// which a stop gives only the queue's newest [`DRAIN_TAIL_BYTES`]; a
/// hypervisor's port takes the whole queue.
const DRAIN_FAST_BYTES_PER_SECOND: u64 = 1 << 20;
/// About a third of a second at 115200 baud.
const DRAIN_TAIL_BYTES: usize = 4 << 10;
/// Whether the queue holds bytes, read without the lock.
static SERIAL_QUEUED: AtomicBool = AtomicBool::new(false);
/// One processor at a time feeds the port from the queue, so its bursts
/// neither interleave nor overrun the FIFO it found empty; the others do
/// not wait for it.
static SENDING: AtomicBool = AtomicBool::new(false);
/// The bytes a 16550's transmit FIFO takes once it is empty: a burst, each
/// line feed sent as two ([`Queue::take_burst`]), fills it in one string
/// output, which a hypervisor takes in one exit.
const SERIAL_FIFO: usize = 16;
/// Once other processors drain the queue ([`share_draining`]), a writer
/// only queues: under a hypervisor every burst is two exits to the host's
/// device model, each of which can wait there for the model's lock as long
/// as the host takes to refresh its display, and a writer holds its PE,
/// and any lock it holds, meanwhile.
static SHARED_DRAINING: AtomicBool = AtomicBool::new(false);
/// Bursts a draining processor sends at once ([`drain_some`]): a few
/// hundred bytes, a few hundred microseconds under a hypervisor.
const DRAIN_BURSTS: usize = 32;

/// A ring of bytes not yet sent.
struct Queue {
    bytes: &'static mut [MaybeUninit<u8>],
    head: usize,
    len: usize,
    /// Bytes left out while the queue was full, not reported yet: the
    /// record and the logs keep them.
    dropped: usize,
}

/// The note that bytes were left out, formatted without the heap.
fn dropped_note(bytes: usize) -> Text<96> {
    let mut note = Text::default();
    let _ = write!(
        note,
        "\n[serial: {bytes} bytes dropped; the boot log keeps them]\n"
    );
    note
}

impl Queue {
    /// The ring in the image; `None` once taken.
    fn early() -> Option<Self> {
        if EARLY_TAKEN.swap(true, Ordering::AcqRel) {
            return None;
        }
        // SAFETY: taken once, above; the ring lives as long as the image.
        let bytes = unsafe { &mut *EARLY_RING.0.get() };
        Some(Self {
            bytes,
            head: 0,
            len: 0,
            dropped: 0,
        })
    }

    /// Queue `data`, or count it as dropped while the queue cannot take it
    /// whole: a writer never waits for the port. The first output that
    /// fits again begins with a note of how much was dropped.
    fn push(&mut self, data: &[u8]) {
        if self.dropped != 0 {
            let note = dropped_note(self.dropped);
            if self.room() < note.as_bytes().len() + data.len() {
                self.dropped += data.len();
                return;
            }
            self.dropped = 0;
            self.copy(note.as_bytes());
        }
        if self.room() < data.len() {
            self.dropped = data.len();
            return;
        }
        self.copy(data);
    }

    fn room(&self) -> usize {
        self.bytes.len() - self.len
    }

    /// Append `data`, which fits.
    fn copy(&mut self, data: &[u8]) {
        let size = self.bytes.len();
        let tail = (self.head + self.len) % size;
        let first = data.len().min(size - tail);
        self.bytes[tail..tail + first].write_copy_of_slice(&data[..first]);
        self.bytes[..data.len() - first].write_copy_of_slice(&data[first..]);
        self.len += data.len();
        SERIAL_QUEUED.store(true, Ordering::Release);
    }

    /// Move the oldest bytes into `out` as far as they fit in one FIFO
    /// with each line feed sent as two; how many.
    fn take_burst(&mut self, out: &mut [u8; SERIAL_FIFO]) -> usize {
        let mut count = 0;
        let mut fifo = 0;
        while count < self.len {
            // SAFETY: only the occupied range is read, as in `take`.
            let byte = unsafe { self.bytes[(self.head + count) % self.bytes.len()].assume_init() };
            let width = if byte == b'\n' { 2 } else { 1 };
            if fifo + width > SERIAL_FIFO {
                break;
            }
            out[count] = byte;
            fifo += width;
            count += 1;
        }
        self.head = (self.head + count) % self.bytes.len();
        self.len -= count;
        if self.len == 0 {
            self.head = 0;
        }
        SERIAL_QUEUED.store(self.len != 0, Ordering::Release);
        count
    }

    /// Move the oldest bytes into `out`, as many as fit; how many.
    fn take(&mut self, out: &mut [u8]) -> usize {
        let count = out.len().min(self.len);
        for slot in &mut out[..count] {
            // SAFETY: only the occupied range is read; copy initialized it before increasing len.
            *slot = unsafe { self.bytes[self.head].assume_init() };
            self.head = (self.head + 1) % self.bytes.len();
        }
        self.len -= count;
        if self.len == 0 {
            // Reuse the touched prefix after a drain instead of walking through unused pages.
            self.head = 0;
        }
        SERIAL_QUEUED.store(self.len != 0, Ordering::Release);
        count
    }

    /// Leave out all but the newest `keep` bytes; how many were left out.
    fn keep_newest(&mut self, keep: usize) -> usize {
        let skipped = self.len.saturating_sub(keep);
        self.head = (self.head + skipped) % self.bytes.len();
        self.len -= skipped;
        skipped
    }

    /// Send what the port takes now, never waiting for it; nothing while
    /// another processor sends or once processors drain the queue.
    fn send_now(&mut self) {
        if self.len == 0
            || SHARED_DRAINING.load(Ordering::Relaxed)
            || SENDING
                .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
                .is_err()
        {
            return;
        }
        let mut burst = [0u8; SERIAL_FIFO];
        while self.len != 0 && crate::serial::ready() {
            let taken = self.take_burst(&mut burst);
            crate::serial::burst(&burst[..taken]);
        }
        SENDING.store(false, Ordering::Release);
    }

    /// Send the queue as the boot stops, bounded in time: a port that drains
    /// slowly, a real UART, gets only the newest [`DRAIN_TAIL_BYTES`] after
    /// [`DRAIN_PROBE_MILLIS`], after a note of how many were left out.
    fn drain(&mut self) {
        // A port that stops taking bytes this long is given up on, as
        // `serial::send` gives up on one byte.
        const STUCK_SPINS: u32 = 1 << 20;
        let hz = TSC_HZ.load(Ordering::Relaxed);
        let started = rdtsc();
        let mut sent = 0u64;
        let mut tail = None;
        let mut stuck = 0;
        let mut burst = [0u8; SERIAL_FIFO];
        while self.len != 0 && stuck < STUCK_SPINS {
            let now = rdtsc();
            let elapsed = now.wrapping_sub(started);
            let probed = hz != 0 && elapsed >= hz / 1000 * DRAIN_PROBE_MILLIS;
            match tail {
                None if probed
                    && sent.saturating_mul(hz)
                        < DRAIN_FAST_BYTES_PER_SECOND.saturating_mul(elapsed) =>
                {
                    tail = Some(now);
                    let skipped = self.keep_newest(DRAIN_TAIL_BYTES);
                    if skipped != 0 {
                        Serial.write_bytes(dropped_note(skipped).as_bytes());
                    }
                    continue;
                }
                // The tail takes a third of a second at 115200 baud.
                Some(since) if now.wrapping_sub(since) >= hz => break,
                _ => {}
            }
            if crate::serial::ready() {
                stuck = 0;
                let taken = self.take_burst(&mut burst);
                crate::serial::burst(&burst[..taken]);
                sent += taken as u64;
            } else {
                stuck += 1;
                core::hint::spin_loop();
            }
        }
        SERIAL_QUEUED.store(false, Ordering::Release);
    }
}

/// Queue the serial port's output in a large ring from now on, for
/// [`send_queued`], with what the image's ring still holds.
pub fn queue_serial() {
    let mut queue = Queue {
        bytes: Box::leak(Box::new_uninit_slice(SERIAL_QUEUE_BYTES)),
        head: 0,
        len: 0,
        dropped: 0,
    };
    let _ = exclusive(|| {
        if let Ok(mut slot) = SERIAL_QUEUE.0.try_borrow_mut() {
            if let Some(mut early) = slot.take() {
                let mut chunk = [0u8; 256];
                loop {
                    let taken = early.take(&mut chunk);
                    if taken == 0 {
                        break;
                    }
                    queue.push(&chunk[..taken]);
                }
                queue.dropped += early.dropped;
            }
            *slot = Some(queue);
        }
    });
}

/// Send what the serial queue holds as far as the port takes it now, never
/// waiting for the port or another processor: safe from the firmware
/// timer's callback.
pub fn pump_serial() {
    if !SERIAL_QUEUED.load(Ordering::Acquire)
        || WRITING
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
    {
        return;
    }
    if let Ok(mut slot) = SERIAL_QUEUE.0.try_borrow_mut()
        && let Some(queue) = slot.as_mut()
    {
        queue.send_now();
    }
    WRITING.store(false, Ordering::Release);
}

/// Send a burst of queued bytes if the port takes it now without waiting;
/// whether this processor should send more. Never waits for another
/// processor: while one sends, the others leave the port to it.
pub fn send_queued() -> bool {
    if !SERIAL_QUEUED.load(Ordering::Acquire)
        || SENDING
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_err()
    {
        return false;
    }
    if crate::serial::ready() {
        let mut burst = [0u8; SERIAL_FIFO];
        let taken = exclusive(|| {
            SERIAL_QUEUE
                .0
                .try_borrow_mut()
                .ok()
                .and_then(|mut slot| slot.as_mut().map(|queue| queue.take_burst(&mut burst)))
                .unwrap_or(0)
        })
        .unwrap_or(0);
        crate::serial::burst(&burst[..taken]);
    }
    SENDING.store(false, Ordering::Release);
    SERIAL_QUEUED.load(Ordering::Acquire)
}

/// From now on a writer only queues, and the processors that call
/// [`drain_some`] or wait send the queue.
pub fn share_draining() {
    SHARED_DRAINING.store(true, Ordering::Relaxed);
}

/// Send up to [`DRAIN_BURSTS`] bursts of queued output, as far as the port
/// takes them now; never waits for the port or another processor.
pub fn drain_some() {
    for _ in 0..DRAIN_BURSTS {
        if !send_queued() {
            break;
        }
    }
}

/// Queue `bytes` for the serial port and send what it takes now; after
/// [`unqueue_serial`], send them here. Under [`WRITING`].
fn serial(bytes: &[u8]) {
    if !SERIAL_DIRECT.load(Ordering::Relaxed)
        && let Ok(mut slot) = SERIAL_QUEUE.0.try_borrow_mut()
    {
        if slot.is_none() {
            *slot = Queue::early();
        }
        if let Some(queue) = slot.as_mut() {
            queue.push(bytes);
            queue.send_now();
            return;
        }
    }
    Serial.write_bytes(bytes);
}

/// Send what the serial queue holds here, bounded in time ([`Queue::drain`]),
/// and stop queueing: the boot is stopping, or this image is leaving boot
/// services, and no PE may wait again.
fn unqueue_serial() {
    let _ = exclusive(|| {
        SERIAL_DIRECT.store(true, Ordering::Relaxed);
        if let Ok(mut slot) = SERIAL_QUEUE.0.try_borrow_mut()
            && let Some(mut queue) = slot.take()
        {
            queue.drain();
        }
    });
}

/// The newest output bytes since the first line, which every log reads
/// from its own position.
static RECORD: Global<Pending<[u8; PENDING_BYTES]>> =
    Global(RefCell::new(Pending::new([0; PENDING_BYTES])));

/// An open log, the blocks that hold it and its position in the record.
struct Sink {
    writer: Writer,
    device: Box<dyn BlockDevice>,
    kind: Log,
    stored: u64,
    /// The counter at its last store.
    flushed: u64,
    /// Its stores so far and the counter ticks they took.
    stores: u64,
    ticks: u64,
    /// The counter when the store in progress began.
    began: Option<u64>,
    /// The store's request in progress on the device.
    request: Option<Submission>,
}

impl Sink {
    /// Move the store in progress on: collect the running request's result
    /// and start the next; with a counter deadline `wait`, until the store
    /// is done or the deadline passed. Whether it is done: a device that
    /// never completes leaves the store in progress at the deadline.
    fn advance(&mut self, wait: Option<u64>) -> Result<bool, BlockError> {
        loop {
            if let Some(request) = self.request.as_mut() {
                let Some(finished) = request.poll(&mut *self.device) else {
                    match wait {
                        Some(deadline) if !passed(deadline) => {
                            core::hint::spin_loop();
                            continue;
                        }
                        _ => return Ok(false),
                    }
                };
                self.request = None;
                self.writer.completed(finished)?;
            }
            let Some(request) = self.writer.next_request() else {
                if let Some(began) = self.began.take() {
                    self.flushed = rdtsc();
                    self.stores += 1;
                    self.ticks += self.flushed.wrapping_sub(began);
                }
                return Ok(true);
            };
            match Submission::new(request, self.device.identity()) {
                Ok(request) => self.request = Some(request),
                Err(error) => {
                    let _ = self.writer.completed(Err(error.clone()));
                    return Err(error);
                }
            }
        }
    }
}

/// The longest a store that waits for the logs (at a stop, or the USB's log
/// before a device starts) waits for their disks: a disk that never
/// completes a command must not keep the PC at the stop. On expiry it is
/// reported once and the stop goes on.
const STORE_WAIT_MILLIS: u64 = 2000;

/// The TSC rate taken before it is measured, for [`STORE_WAIT_MILLIS`]: no
/// PC's is higher, so the wait is at least that long.
const UNMEASURED_TSC_HZ: u64 = 6_000_000_000;

/// Whether the counter passed `deadline`.
fn passed(deadline: u64) -> bool {
    rdtsc().wrapping_sub(deadline) < u64::MAX / 2
}

/// Seconds between the machine state's log stores while the engine runs:
/// ext4's default journal commit interval. A stop stores at once.
const MACHINE_FLUSH_SECONDS: u64 = 5;
/// Seconds between the USB log's stores while the engine runs.
const USB_FLUSH_SECONDS: u64 = 10;

/// The BSP's counter on any core (`engine::host::rdtsc`): log times and
/// flush deadlines are compared across cores.
fn rdtsc() -> u64 {
    crate::engine::host::rdtsc()
}

pub use stela_boot_core::log_sinks::Log;

/// The logs lost this boot, each reported once: reached only while
/// holding [`HELD`].
static LOST: Global<stela_boot_core::log_sinks::Lost> =
    Global(RefCell::new(stela_boot_core::log_sinks::Lost::new()));

/// Report that `log` could not be opened or stored: once per place, in the
/// log and on the screen. The boot goes on without it: a diagnostic copy of
/// the output never stops the machine.
pub fn log_lost(log: Log, error: impl fmt::Display) {
    arguments(format_args!(
        "BOOT LOG refusal: {} log: {error}; closed for this boot\n",
        log.name()
    ));
    // Never waits: another processor may hold the logs for good.
    let notice = with_sinks(false, |_| LOST.0.borrow_mut().lose(log)).flatten();
    if let Some(notice) = notice {
        crate::status::info(notice);
    }
}

/// The open logs, which any processor may flush or report: reached only
/// while holding [`HELD`].
static SINKS: Global<Vec<Sink>> = Global(RefCell::new(Vec::new()));
/// A processor holds [`SINKS`]: a fatal or panic raised inside a flush
/// halts without flushing again.
static HELD: AtomicBool = AtomicBool::new(false);

/// Run `f` on the open logs once no other processor holds them, or `None`
/// at once when one does and `wait` is false.
fn with_sinks<R>(wait: bool, f: impl FnOnce(&mut Vec<Sink>) -> R) -> Option<R> {
    while HELD
        .compare_exchange_weak(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        if !wait {
            return None;
        }
        core::hint::spin_loop();
    }
    let result = SINKS.0.try_borrow_mut().ok().map(|mut sinks| f(&mut sinks));
    HELD.store(false, Ordering::Release);
    result
}
/// Boot services still run, so the firmware's console can show a fatal
/// error before Stela's own console takes the screen.
static FIRMWARE_CONSOLE: AtomicBool = AtomicBool::new(true);
/// `--host-check` shows its report itself, so every line reaches the screen.
static RAW_SCREEN: AtomicBool = AtomicBool::new(false);
/// The time-stamp counter's rate, once known, which stamps each line the
/// logs keep with the seconds since the PC started.
static TSC_HZ: AtomicU64 = AtomicU64::new(0);
/// The record's next byte starts a line.
static LINE_START: AtomicBool = AtomicBool::new(true);

/// Stamp the logs' lines from now on with the time from the counter running
/// at `tsc_hz`.
pub fn set_clock(tsc_hz: u64) {
    TSC_HZ.store(tsc_hz, Ordering::Relaxed);
}

/// `[  12.345] `: the seconds since the PC started, as the kernel's own
/// lines show them; nothing before the counter's rate is known.
fn stamp(record: &mut Pending<[u8; PENDING_BYTES]>) {
    let hz = TSC_HZ.load(Ordering::Relaxed);
    if hz == 0 {
        return;
    }
    let millis = rdtsc() / (hz / 1000).max(1);
    let mut text = Text::<24>::default();
    let _ = write!(text, "[{:5}.{:03}] ", millis / 1000, millis % 1000);
    record.push(text.as_bytes());
}

/// Text formatted without the heap, such as a line's stamp.
struct Text<const N: usize> {
    bytes: [u8; N],
    length: usize,
}

impl<const N: usize> Default for Text<N> {
    fn default() -> Self {
        Self {
            bytes: [0; N],
            length: 0,
        }
    }
}

impl<const N: usize> Text<N> {
    fn as_bytes(&self) -> &[u8] {
        &self.bytes[..self.length]
    }
}

impl<const N: usize> Write for Text<N> {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        let end = self.length + value.len();
        self.bytes
            .get_mut(self.length..end)
            .ok_or(fmt::Error)?
            .copy_from_slice(value.as_bytes());
        self.length = end;
        Ok(())
    }
}

/// Stop the marker's firmware timer, store the logs and close those the
/// firmware holds, then stop writing to the firmware's console: called just before boot services end, which
/// take both with them.
pub fn leave_boot_services() {
    crate::firmware::stop_ticker();
    // The engine's copy of this image has a queue of its own: send this
    // one's now, bounded, and the loader's last lines directly.
    unqueue_serial();
    flush_log();
    with_sinks(true, |sinks| {
        let firmware = |sink: &Sink| sink.kind == Log::Firmware;
        if let Some(stored) = sinks
            .iter()
            .filter(|sink| firmware(sink))
            .map(|sink| sink.stored)
            .max()
        {
            UNKEPT_FROM.store(stored, Ordering::Relaxed);
        }
        sinks.retain(|sink| !firmware(sink))
    });
    disable_firmware_console();
}

/// The record's position after the last byte the firmware's log file took
/// before it closed: later lines are on the stick only once Stela's own
/// drivers write the log on, and a stop before that keeps them in the
/// firmware ([`keep_stop_record`]).
static UNKEPT_FROM: AtomicU64 = AtomicU64::new(0);

/// Copy the lines no log stored since the firmware's log closed into
/// `into`, the newest whole lines that fit (`loader_lines`), for the engine
/// to put at the head of its own record ([`carry_loader_lines`]); the
/// bytes used.
pub fn unkept_lines(into: &mut [u8]) -> usize {
    use stela_boot_core::loader_lines::{Carry, first};
    exclusive(|| {
        let Ok(record) = RECORD.0.try_borrow() else {
            return 0;
        };
        let unkept = UNKEPT_FROM.load(Ordering::Relaxed);
        let from = first(unkept, record.pushed(), into.len());
        let mut carry = Carry::new(into, from - unkept);
        let Ok(_) = record.copy_since(from, |bytes| {
            carry.push(bytes);
            Ok::<_, Infallible>(())
        });
        carry.finish()
    })
    .unwrap_or(0)
}

/// Put the loader's lines that no log stored ([`unkept_lines`]) at the
/// head of this image's record, with their stamps, for every log it opens
/// to store first. The serial port had them from the loader.
pub fn carry_loader_lines(lines: &[u8]) {
    if lines.is_empty() {
        return;
    }
    let _ = exclusive(|| {
        if let Ok(mut record) = RECORD.0.try_borrow_mut() {
            record.push(lines);
            LINE_START.store(lines.ends_with(b"\n"), Ordering::Relaxed);
        }
    });
}

/// Leave the newest lines in the PC's firmware (`stop_record::keep`), for
/// the next start to copy into the log and to say this one stopped: those
/// the USB's log does not hold, or a stop's inside the host drivers, whose
/// screen may be dead; whether the firmware holds them. The lines end with the stop's reason, which
/// the caller has printed; `reason` stands for them where the record
/// cannot be read, as in a stop raised while a line was being written.
fn keep_stop_record(reason: Option<&str>) -> bool {
    use stela_boot_core::stop_record::TEXT_BYTES;
    crate::stop_record::keep(|lines| {
        let _ = exclusive(|| {
            let Ok(record) = RECORD.0.try_borrow() else {
                return;
            };
            // One byte more than the record holds, which says whether its
            // first byte starts a line.
            let newest = record.pushed().saturating_sub(TEXT_BYTES as u64 + 1);
            let from = UNKEPT_FROM.load(Ordering::Relaxed).max(newest);
            let Ok(_) = record.copy_since(from, |bytes| {
                lines.push(bytes);
                Ok::<_, Infallible>(())
            });
        });
        if let Some(reason) = reason.filter(|_| lines.text().is_empty()) {
            for part in ["STELA FATAL ", reason, "\n"] {
                lines.push(part.as_bytes());
            }
        }
    })
}

/// The relocated engine has its own statics, not the loader image's updated
/// console flag. Revoke this image's firmware access before any engine error
/// can consult the expired system table. This makes no firmware calls and
/// needs neither the engine heap nor a valid handoff.
pub fn disable_firmware_console() {
    FIRMWARE_CONSOLE.store(false, Ordering::Relaxed);
}

/// Send every line to the screen as well, for a report that is all detail.
#[cfg(feature = "diagnostics")]
pub fn raw_screen() {
    RAW_SCREEN.store(true, Ordering::Relaxed);
}

/// Write `text` to the firmware's console while boot services run and the
/// firmware has one.
fn firmware_console(text: &str) {
    let console = || {
        uefi::table::system_table_raw()
            // SAFETY: the firmware's system table, valid while boot services run.
            .is_some_and(|table| unsafe { !table.as_ref().stdout.is_null() })
    };
    if FIRMWARE_CONSOLE.load(Ordering::Relaxed) && console() {
        uefi::system::with_stdout(|out| {
            let _ = out.write_str(text);
        });
    }
}

struct Output;
impl Write for Output {
    fn write_str(&mut self, value: &str) -> fmt::Result {
        serial(value.as_bytes());
        if RAW_SCREEN.load(Ordering::Relaxed) {
            console::write(&[(value, console::Color::Text)]);
        }
        if let Ok(mut record) = RECORD.0.try_borrow_mut() {
            for line in value.split_inclusive('\n') {
                if LINE_START.load(Ordering::Relaxed) {
                    stamp(&mut record);
                }
                record.push(line.as_bytes());
                LINE_START.store(line.ends_with('\n'), Ordering::Relaxed);
            }
        }
        Ok(())
    }
}

/// Why [`add_log`] did not keep a log.
pub enum Unopened {
    /// The device refused the first store.
    Store(BlockError),
    /// The device took the first store and does not hold it.
    Unkept(Unkept),
}

impl fmt::Display for Unopened {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Store(error) => write!(f, "{error}"),
            Self::Unkept(unkept) => write!(f, "{unkept}"),
        }
    }
}

/// Keep the output in `writer`'s log on `device` from now on, starting with
/// everything since the first line that the record still holds. The first
/// store is read back before the log counts as open.
pub fn add_log(
    mut writer: Writer,
    mut device: Box<dyn BlockDevice>,
    kind: Log,
) -> Result<(), Unopened> {
    let stored = exclusive(|| match RECORD.0.try_borrow() {
        Ok(record) => {
            let Ok(reached) = record.copy_since(0, |bytes| {
                writer.append(bytes);
                Ok::<_, Infallible>(())
            });
            reached
        }
        Err(_) => 0,
    })
    .unwrap_or(0);
    writer.flush(&mut *device).map_err(Unopened::Store)?;
    writer.verify(&mut *device).map_err(Unopened::Unkept)?;
    with_sinks(true, |sinks| {
        sinks.push(Sink {
            writer,
            device,
            kind,
            stored,
            flushed: rdtsc(),
            stores: 0,
            ticks: 0,
            began: None,
            request: None,
        })
    });
    Ok(())
}

/// Store what every open log lacks and close them all, before the disks
/// they live on are shut down; later output reaches the serial port only.
pub fn close_logs() {
    flush_log();
    with_sinks(true, Vec::clear);
}

/// Store in every open log the output it does not hold yet, waiting for
/// the stores. A log that fails is reported and closed for this boot
/// ([`log_lost`]). Output printed during the flush
/// is not recorded.
pub fn flush_log() {
    flush_where(|_| true, true);
}

/// Store the USB's log now, waiting for the store: what the device start
/// does before each device's driver runs, so the line naming it is on the
/// stick first.
pub fn store_usb_log() {
    flush_where(|sink| sink.kind == Log::Usb, true);
}

/// Begin storing the logs whose store is due, the machine state's every
/// [`MACHINE_FLUSH_SECONDS`] and the USB's every [`USB_FLUSH_SECONDS`], and
/// move every store in progress on; never waits for a device
/// ([`advance_logs`]).
pub fn flush_due() {
    let hz = TSC_HZ.load(Ordering::Relaxed);
    let now = rdtsc();
    flush_where(
        |sink| {
            let seconds = match sink.kind {
                Log::Usb => USB_FLUSH_SECONDS,
                _ => MACHINE_FLUSH_SECONDS,
            };
            now.wrapping_sub(sink.flushed) >= seconds * hz
        },
        false,
    );
}

/// Move the log stores in progress on; nothing to do while none runs.
pub fn advance_logs() {
    if STORING.load(Ordering::Acquire) {
        flush_where(|_| false, false);
    }
}

/// A log store is in progress ([`advance_logs`]).
static STORING: AtomicBool = AtomicBool::new(false);
/// The last store that waited could not reach the logs: this processor is
/// inside the host drivers, or another holds the logs. A stop that finds
/// it set has lines the USB's log does not hold ([`stopped_text`]).
static UNSTORED: AtomicBool = AtomicBool::new(false);

fn flush_where(due: impl Fn(&Sink) -> bool, wait: bool) {
    // The interrupted driver call holds the disks' lock and may be partway
    // through a request of its own. The GPU's start holds it outside every
    // storage driver's call and stores the USB's log itself, which a stop
    // there waits for: no other log is open then.
    if crate::engine::inside_host_drivers() {
        if wait && crate::engine::store_usb_log_in_gpu_start() {
            return;
        }
        static SAID: AtomicBool = AtomicBool::new(false);
        // The device start stores the USB's log itself between devices.
        if !crate::engine::starting_host_devices() && !SAID.swap(true, Ordering::Relaxed) {
            arguments(format_args!(
                "BOOT LOG not stored: this processor stopped inside the host drivers\n"
            ));
        }
        if wait {
            UNSTORED.store(true, Ordering::Relaxed);
        }
        return;
    }
    let mut failed = Vec::new();
    let mut late = Vec::new();
    // One deadline for every log's store.
    let started = rdtsc();
    let deadline = wait.then(|| {
        let hz = match TSC_HZ.load(Ordering::Relaxed) {
            0 => UNMEASURED_TSC_HZ,
            hz => hz,
        };
        started.wrapping_add(hz / 1000 * STORE_WAIT_MILLIS)
    });
    let reached = with_sinks(false, |sinks| {
        // A log whose store is in progress takes new bytes at its next one.
        let is_due = |sink: &Sink| !sink.writer.storing() && due(sink);
        // Take the record's new bytes while no processor adds to it, then
        // store them without holding the others' output up. Polling in-flight
        // writes needs neither the record nor a list of sinks to prepare.
        if sinks.iter().any(&is_due) {
            let bound = if wait {
                WRITING_WAIT_MICROS
            } else {
                STORE_COPY_WAIT_MICROS
            };
            let _ = exclusive_within(bound, || {
                let Ok(record) = RECORD.0.try_borrow() else {
                    return;
                };
                for sink in sinks.iter_mut().filter(|sink| is_due(sink)) {
                    let Ok(reached) = record.copy_since(sink.stored, |bytes| {
                        sink.writer.append(bytes);
                        Ok::<_, Infallible>(())
                    });
                    sink.stored = reached;
                }
            });
        }
        let mut index = 0;
        while index < sinks.len() {
            let sink = &mut sinks[index];
            if is_due(sink) && sink.writer.begin_store() {
                sink.began = Some(rdtsc());
            }
            match sink.advance(deadline) {
                Ok(done) => {
                    if wait && !done {
                        late.push(sink.kind);
                    }
                    index += 1;
                }
                Err(error) => failed.push((sinks.remove(index).kind, error)),
            }
        }
        STORING.store(
            sinks.iter().any(|sink| sink.writer.storing()),
            Ordering::Release,
        );
    });
    if wait {
        UNSTORED.store(reached.is_none() || !late.is_empty(), Ordering::Relaxed);
    }
    for (kind, error) in failed {
        log_lost(kind, error);
    }
    // Reported once a boot: its store stays in progress and goes on at the
    // next turn, if any.
    static LATE: AtomicBool = AtomicBool::new(false);
    if let Some(kind) = late.first()
        && !LATE.swap(true, Ordering::Relaxed)
    {
        arguments(format_args!(
            "BOOT LOG refusal: the {} log's store timed out after {} ms; going on without it\n",
            kind.name(),
            millis_since(started)
        ));
    }
}

/// The counter now, for [`millis_since`].
pub fn now() -> u64 {
    rdtsc()
}

/// Milliseconds since counter value `start`; 0 before the rate is known.
pub fn millis_since(start: u64) -> u64 {
    match TSC_HZ.load(Ordering::Relaxed) {
        0 => 0,
        hz => rdtsc().wrapping_sub(start) / (hz / 1000).max(1),
    }
}

/// Report each open log's stores so far and the milliseconds they took.
pub fn report_log_stores() {
    let hz = TSC_HZ.load(Ordering::Relaxed).max(1);
    let Some(costs) = with_sinks(false, |sinks| {
        sinks
            .iter()
            .map(|sink| (sink.kind, sink.stores, sink.ticks * 1000 / hz))
            .collect::<Vec<(Log, u64, u64)>>()
    }) else {
        return;
    };
    for (kind, stores, millis) in costs {
        let name = kind.name();
        arguments(format_args!(
            "BOOT LOG {name} stores={stores} ms={millis}\n"
        ));
    }
}

pub fn arguments(args: fmt::Arguments<'_>) {
    if exclusive(|| Output.write_fmt(args)).is_none() {
        let _ = Serial.write_fmt(args);
    }
}

/// Report `code` on the hypervisor exit port, which may end a virtual
/// machine's run there; a PC without a hypervisor gets no write, and
/// execution continues.
pub fn debug_exit(code: u32) {
    // The run may end here: whatever waits for the serial port goes first.
    unqueue_serial();
    terminal::exit(code);
}

/// Whether CPUID reports a hypervisor, whose exit port takes stop codes.
pub fn hypervisor_exit_port() -> bool {
    terminal::exit_port().is_some()
}

/// Report `code` on the hypervisor exit port, then halt. On a PC the screen
/// keeps the output until the machine is powered off.
pub fn terminate(code: u32) -> ! {
    arguments(format_args!(
        "STELA STOPPED code={code:#x}; power off to continue\n"
    ));
    // Why it stopped is at the end of what is queued, and the log's USB disk
    // may be what stopped.
    unqueue_serial();
    flush_log();
    halt(code)
}

/// Stop before allocator initialization. UART/record formatting and UEFI's
/// stack-buffered text writer need no Rust allocation; neither log flushing nor
/// the allocating status/panic presentation path is entered here. No log is
/// open yet, so the lines go to the firmware's variables, which need no
/// allocation either, for the next start to copy into the log
/// (`stop_record::carry`). Call only while firmware boot services and its
/// system table remain live.
pub(crate) fn early_fatal(message: &str) -> ! {
    arguments(format_args!("\nSTELA FATAL {message}\n"));
    firmware_console("\nStela stopped: ");
    firmware_console(message);
    firmware_console("\n");
    crate::stop_record::arm(crate::stop_record::Services::capture_without_heap(), None);
    keep_stop_record(Some(message));
    arguments(format_args!(
        "STELA STOPPED code=0x11; power off to continue\n"
    ));
    halt(0x11)
}

fn halt(code: u32) -> ! {
    debug_exit(code);
    // After the debug-exit attempt no task resumes; the stop answers only
    // the PC's power button.
    crate::stop::wait()
}

/// Show `message` as the reason the mount stopped: as a failed step on
/// Stela's console, followed by `next`, or on the firmware's before it
/// exists.
fn stopped(message: &str, next: &dyn Fn() -> &'static str) {
    console::reclaim();
    if console::active() && !RAW_SCREEN.load(Ordering::Relaxed) {
        crate::status::fail(message);
        crate::status::info(next());
    } else if FIRMWARE_CONSOLE.load(Ordering::Relaxed) {
        firmware_console("\nStela stopped: ");
        firmware_console(message);
        firmware_console("\n");
    }
}

/// What a stopped boot tells the user to do next: where its log is, and how
/// to read it on another computer.
pub const STOPPED: &str = "Stela stopped. Its log is on this USB: on another computer, plug in \
    this USB and run `stela log`, which saves it to log.txt; include that file in a report. Turn \
    the PC off to try again; Ctrl+Alt+Delete on a laptop's own keyboard restarts it.";

/// What a stop whose last lines the USB's log does not hold, as one before
/// the engine writes the log again, tells the user once the PC's firmware
/// holds them ([`keep_stop_record`]): another start of Stela on this PC
/// moves them into the log.
pub const STOPPED_RECORDED: &str = "Stela stopped before it could write this part of its log \
    to the USB. This PC keeps its last lines: start Stela on this PC once more, which copies them \
    into the log on this USB. Then, on another computer, plug in this USB and run `stela log`, \
    which saves it to log.txt; include that file in a report. Turn the PC off to try again; \
    Ctrl+Alt+Delete on a laptop's own keyboard restarts it.";

/// What such a stop tells the user when the firmware did not take the
/// lines: the screen is all that is kept of this part of the run.
pub const STOPPED_UNLOGGED: &str = "Stela stopped before it could write this part of its log \
    to the USB: this screen is the only record of it. Take a photo of it for a report. Turn the \
    PC off to try again; Ctrl+Alt+Delete on a laptop's own keyboard restarts it.";

/// Whether the USB's log is open and none of its stores failed this boot.
/// Never waits: a stop raised inside a log store finds the logs held, and
/// a log whose store was cut short is not claimed.
pub fn usb_log_kept() -> bool {
    with_sinks(false, |sinks| {
        let lost = LOST.0.borrow();
        sinks.iter().any(|sink| sink.kind != Log::Machine)
            && !lost.is_lost(Log::Usb)
            && !lost.is_lost(Log::Firmware)
    })
    .unwrap_or(false)
}

/// [`STOPPED`] while the USB's log is kept ([`usb_log_kept`]) and took the
/// last store: a log that was closed as lost is never claimed, nor one a
/// stop inside the host drivers could not store its last lines in. Else
/// those lines go to the PC's firmware here, once a boot, and the text says
/// whether it holds them: [`STOPPED_RECORDED`], or [`STOPPED_UNLOGGED`].
/// Called only as the boot stops, after the last store, so a store that
/// fails then counts.
pub fn stopped_text() -> &'static str {
    stop_advice(None)
}

/// [`stopped_text`] for a stop whose reason is `reason`.
fn stop_advice(reason: Option<&str>) -> &'static str {
    if usb_log_kept() && !UNSTORED.load(Ordering::Relaxed) {
        STOPPED
    } else if keep_stop_record(reason) {
        STOPPED_RECORDED
    } else {
        STOPPED_UNLOGGED
    }
}

pub fn fatal(message: impl core::fmt::Display) -> ! {
    let message = alloc::string::ToString::to_string(&message);
    arguments(format_args!("\nSTELA FATAL {message}\n"));
    // The reason reaches the port before any later step can stop for good.
    unqueue_serial();
    crate::engine::stop_gpus_for_fatal();
    // Inside the host drivers no store goes through them, or only the USB's
    // log the GPU's start drives. The lines go to the stick where they can
    // and to the PC's firmware in any case, before the screen is drawn: the
    // start's reset of the GPU may have left none, and the next start then
    // shows that this one stopped, and why (`stop_record::carry`).
    if crate::engine::inside_host_drivers() {
        flush_log();
        keep_stop_record(Some(&message));
    }
    // The reason is on the screen before the last store, which may be what
    // fails, and the advice after it says whether the log was kept.
    stopped(&message, &|| {
        flush_log();
        stop_advice(Some(&message))
    });
    // A stop with no screen of Stela's own leaves its record too.
    flush_log();
    stop_advice(Some(&message));
    terminate(0x11)
}

/// Report `message` as the reason the mount stops through an orderly
/// power-off the caller has requested (`engine::service`), which brings the
/// GPUs to rest and unloads their firmware first, unlike [`fatal`].
#[cfg(feature = "nvk")]
pub fn stopping(message: &str) {
    arguments(format_args!("\nSTELA STOP {message}\n"));
    unqueue_serial();
    stopped(message, &|| "Stela stopped. The PC turns off.");
}
