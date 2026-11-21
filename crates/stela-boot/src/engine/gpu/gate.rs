//! The Metal service over NVK, which the guest reaches through StelaGPU's gate,
//!
//! It opens on the boot CPU once nouveau has bound the GPUs, before the guest's
//! DeviceTree is final, so the tree has a gate only with a device behind it,
//! and one for each GPU that opened.
//!
//! Each service runs on a stack of its own, deep enough for a compile. Its
//! lock means one is enough; opening comes before any caller. A second
//! stack takes the compilations the calls defer, one at a time under the
//! compiler's own lock ([`run_job`]): a call that needs one is answered
//! not ready, and its guest thread sleeps and calls again.
use super::{
    BOOT_CPU, Bound, Installed, PAGE, client, installed, look_again, pump_waiting, slice,
    stack::{self, Stack},
};
use crate::{engine::host, output};
use alloc::{boxed::Box, format, string::String, sync::Arc, vec::Vec};
use core::{
    hint,
    num::NonZeroU64,
    ptr::{self, NonNull},
    sync::atomic::{AtomicBool, AtomicPtr, AtomicU32, AtomicU64, Ordering},
};
use stela_boot_core::gpu_loss::{self, SilentLines};
use stela_contracts::gate::{BundleReport, GateService, MemoryRange, window};
pub(in crate::engine) use stela_drivers::gpu::metal::deferred::Taker;
use stela_drivers::gpu::metal::{
    compiler::{self, AirCompiler},
    deferred::{busy_pause_after_slice, busy_pause_millis},
    device::{lifecycle::QuiesceStatus, pass_times},
    error::Error,
    native::{self, ExportedMemory, Loader, MemoryExporter, NativeBackend, pipeline_store},
    open::{Attachment, FIRST_SUBMISSION_NS, Gpu, OpenError, PcieLink, open, reopen},
    service::{
        JobSlice, REOPEN_FAILED, Served, ServiceHost, SharedService, identity::DeviceEpoch,
        wire::Status,
    },
    timing::{ExitWatch, Totals},
    vulkan::physical::PciBusInfo,
};
use stela_drivers::gpu::stack::{ResetState, ResetWait};
use stela_linux::{
    client::Run,
    function::{dma_quarantine_status, quarantined_functions},
};
use stela_loader::{apple_device_tree::DeviceTree, stela_gpu};

type Shared = SharedService<NativeBackend, AirCompiler>;

/// A process's main stack, as big as the deepest compile LLVM bounds.
pub(super) const STACK_BYTES: usize = 16 << 20;

/// How long a GPU's accepted work may go without progress as the PC stops:
/// with the guest stopped, an event it waits on will never signal.
const STOP_STALL_SECONDS: u64 = 5;
/// How long the stop waits for each GPU's GSP to unload.
const STOP_UNLOAD_SECONDS: u64 = 10;

/// The services for the kernel's interrupts, once all are open.
static SERVICES: AtomicPtr<Vec<Arc<Shared>>> = AtomicPtr::new(ptr::null_mut());
/// The bound GPU, by its index (its DRM minor), of each service, in the
/// services' order.
static SERVED: AtomicPtr<Vec<usize>> = AtomicPtr::new(ptr::null_mut());

/// The compile stacks ([`open_gpu`]), whose jobs [`run`] runs with the
/// shootdown vector open; a GPU past these runs its jobs masked.
static COMPILE_STACKS: [AtomicU64; 8] = [const { AtomicU64::new(0) }; 8];

fn run(stack: u64, body: &mut dyn FnMut()) {
    let job = COMPILE_STACKS
        .iter()
        .any(|compile| compile.load(Ordering::Relaxed) == stack);
    // SAFETY: a service's stack lives for the boot, and only the opener, a
    // turn's holder or the compile lock's holder is here.
    let mut on_stack = || unsafe { stack::on_stack(stack, &mut *body) };
    if job {
        // SAFETY: a job runs where long host work is allowed, from a run
        // loop or a guest call, holding no lock the doorbell's or the
        // shootdown's handler takes; it leaves IF and CR8 as they were.
        unsafe { crate::native_mmio::shootdown::job(on_stack) }
    } else {
        on_stack()
    }
}

/// A moment of a wait with no bound: on the boot processor it pumps, as the
/// holder may wait for Linux, which only runs there; one Linux step can take
/// tens of milliseconds. It resumes a job waiting between slices, which may
/// hold what this waits for: the compile lock, or a lock of Mesa's.
pub(super) fn relax() {
    // What this waits for may be retiring a mapping, which waits for this.
    crate::native_mmio::shootdown::service();
    slice::help();
    if host::processor() == BOOT_CPU.load(Ordering::Relaxed) {
        pump_waiting();
    }
    hint::spin_loop();
}

/// A moment of a frontend call's bounded wait for the turn
/// (`ServiceHost::relax_within`): it never pumps, as one Linux step can
/// outlast the bound many times over. On the boot processor, while a call is
/// posted to Linux, the wait ends at once: the holder may wait for that call,
/// which the boot processor serves only once it returns to its run loop.
fn relax_within() -> bool {
    crate::native_mmio::shootdown::service();
    if host::processor() == BOOT_CPU.load(Ordering::Relaxed) && client::posted_waiting() {
        return false;
    }
    hint::spin_loop();
    true
}

/// A table of the texts a log line has shown, and a count of the distinct
/// ones it had no room left to hold.
struct Seen<const N: usize> {
    hashes: [AtomicU64; N],
    unheld: AtomicU64,
}

impl<const N: usize> Seen<N> {
    const fn new() -> Self {
        Self {
            hashes: [const { AtomicU64::new(0) }; N],
            unheld: AtomicU64::new(0),
        }
    }

    /// Whether to show `text`: the first time, by its FNV-1a hash, since a
    /// text is built per call and only its bytes identify it. Once the table
    /// fills, a text it cannot hold is shown only at each power-of-two count
    /// of such texts, which the [`Unheld`] it answers names, so a flood of distinct
    /// texts stays bounded.
    fn first_time(&self, text: &str) -> Option<Unheld> {
        let hash = text.bytes().fold(0xcbf2_9ce4_8422_2325, |hash, byte| {
            (hash ^ u64::from(byte)).wrapping_mul(0x100_0000_01b3)
        });
        for slot in &self.hashes {
            match slot.compare_exchange(0, hash, Ordering::Relaxed, Ordering::Relaxed) {
                Ok(_) => return Some(Unheld(0)),
                Err(known) if known == hash => return None,
                Err(_) => {}
            }
        }
        let unheld = self.unheld.fetch_add(1, Ordering::Relaxed) + 1;
        unheld.is_power_of_two().then_some(Unheld(unheld))
    }
}

/// How many distinct texts a full [`Seen`] has counted, shown after a line.
struct Unheld(u64);

impl core::fmt::Display for Unheld {
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        match self.0 {
            0 => Ok(()),
            count => write!(formatter, " ({count} distinct past the log's table)"),
        }
    }
}

/// The time stamp before which no busy processor takes a job
/// ([`busy_pause_millis`]).
static NEXT_BUSY_JOB: AtomicU64 = AtomicU64::new(0);

/// How long a frontend call waits for the turn before it is answered
/// `Pending` and made again (`SharedService::set_wait_bound`): its PE takes
/// no guest interrupt while it waits, and XNU panics when a thread cannot be
/// switched off its CPU within half a second.
const WAIT_BOUND_MILLIS: u64 = 2;

/// About how long each part of the boot CPU's look at a service takes at
/// most (`SharedService::set_look_budget`): the destruction of what completed
/// work released, and the upkeep of the frees and semaphores the turns left,
/// done where a host kernel call needs no posting. The boot PE's guest waits
/// meanwhile; one look destroying a burst of releases stalled it 99-300 ms.
const LOOK_MICROS: u64 = 1000;

/// About how long one `NewLibrary` run reads and digests its library before
/// its turn (`SharedService::set_library_budget`): libraries of 100 MB and
/// more held one exit and every other call's turn for up to 81 ms while
/// they were copied and compared whole.
const LIBRARY_MICROS: u64 = 2000;

/// A single gate exit this long or longer is reported once a boot
/// (`GPU LONG EXIT`): 100 ms.
const LONG_EXIT_MILLIS: u64 = 100;

/// The exits since the last report, checked against the target
/// (`GPU EXIT … check=ok|over`).
static EXITS: ExitWatch = ExitWatch::new();

/// A turn held this long or longer is reported once for each operation and
/// phase: no turn should come near it.
const LONG_TURN_MILLIS: u64 = 50;

/// One host step of a deferred job this long or longer is reported, each
/// one longer than any before: its processor's guest took no interrupt
/// meanwhile, and XNU panics when a processor stays out of its guest for
/// about half a second. A slice should end within a few milliseconds.
const LONG_JOB_MILLIS: u64 = 100;

/// The longest step of a deferred job reported so far, in TSC ticks.
static LONGEST_JOB: AtomicU64 = AtomicU64::new(0);

/// Runs a slice of a deferred compilation, between two slices of a
/// processor's guest and never inside a guest's call: a slice ends at the
/// job's first allocation, wait or host call past [`slice::SLICE_MICROS`],
/// so a job of seconds holds no processor out of its guest for long
/// (`slice`). A processor with no guest work to do takes a job first.
/// Off the boot processor a slice runs with interrupts on; on it with only
/// the doorbell and the TLB shootdown open, so a shootdown another
/// processor sends is acknowledged at once rather than after the slice.
/// Whether one ran.
pub(in crate::engine) fn run_job(taker: Taker) -> bool {
    let Some(gpu) = installed() else {
        return false;
    };
    let millis = gpu.tsc_hz / 1000;
    let now = host::rdtsc();
    if taker == Taker::Busy && now < NEXT_BUSY_JOB.load(Ordering::Relaxed) {
        return false;
    }
    let older = millis * taker.after_millis();
    let resume_after = millis * taker.resume_after_millis();
    let mut ran = None;
    for service in shared().iter() {
        if !service.jobs_waiting() {
            continue;
        }
        let run = || {
            if taker == Taker::Boot {
                service.run_job(older, resume_after)
            } else {
                // SAFETY: an AP's host work between slices, which holds nothing
                // its interrupts' handlers take; they save the vector state.
                unsafe { with_interrupts(|| service.run_job(older, resume_after)) }
            }
        };
        if let Some(step) = measured_job(service.job_between_slices(), run) {
            note_long_step(step.ticks, millis, taker);
            ran = Some(step);
            break;
        }
    }
    if taker == Taker::Busy {
        // After a slice, busy processors leave the next to a waiting one
        // for as long as it took; with none due, they look again soon, so
        // that the spacing never adds to a job's wait.
        let pause = match ran {
            Some(step) => busy_pause_after_slice(step.ticks, millis),
            None => millis * busy_pause_millis(false),
        };
        NEXT_BUSY_JOB.store(host::rdtsc().wrapping_add(pause), Ordering::Relaxed);
    }
    ran.is_some()
}

/// Reports a job's host step of `ticks`, each one longer than any before
/// once it reaches [`LONG_JOB_MILLIS`].
fn note_long_step(ticks: u64, millis: u64, taker: Taker) {
    if ticks >= LONG_JOB_MILLIS * millis && LONGEST_JOB.fetch_max(ticks, Ordering::Relaxed) < ticks
    {
        output::arguments(format_args!(
            "GPU LONG JOB ms={} taker={taker:?} pe={}: its guest took no interrupt meanwhile\n",
            ticks / millis.max(1),
            host::processor()
        ));
    }
}

/// Resumes a job that waits between slices, for a wait outside every job
/// ([`slice::help`]); never starts one.
pub(super) fn resume_job() {
    for service in shared().iter() {
        if service.job_between_slices() {
            if let Some(gpu) = installed() {
                let millis = gpu.tsc_hz / 1000;
                if let Some(step) = measured_job(true, || service.run_job(u64::MAX, 0)) {
                    note_long_step(step.ticks, millis, Taker::Waiting);
                }
            }
            return;
        }
    }
}

/// A host call of the guest call served on this stack was withdrawn, as no
/// task took it in time: whether that guest call, a turn of the service
/// whose stack this is, will be made again if it fails
/// ([`SharedService::defer_for_wait`]).
pub(super) fn defer_for_wait() -> bool {
    let pointer = stack::pointer();
    shared()
        .iter()
        .find(|service| {
            let top = service.stack();
            pointer < top && top - pointer < STACK_BYTES as u64
        })
        .is_some_and(|service| service.defer_for_wait())
}

/// The C library's core for the jobs on the compile stack at `top`: one
/// past every processor's from the top of the processors' range, one for
/// each compile stack, so that a job keeps its thread-local values on
/// whichever processor resumes it ([`slice::job_core`]).
pub(super) fn job_core(top: u64) -> core::ffi::c_uint {
    let index = COMPILE_STACKS
        .iter()
        .position(|stack| stack.load(Ordering::Relaxed) == top)
        .unwrap_or(COMPILE_STACKS.len());
    (slice::LIBC_PROCESSORS - 1 - index) as core::ffi::c_uint
}

/// Whether a busy processor may look for a job ([`Taker::Busy`]) now: a
/// load, so that a busy processor's slice pays for no more.
pub(in crate::engine) fn busy_job_due() -> bool {
    host::rdtsc() >= NEXT_BUSY_JOB.load(Ordering::Relaxed)
}

/// The most the heap's carved bytes rose during one job since the last
/// `GPU JOBS` line, and the carved bytes that job started from.
static JOB_RISE: AtomicU64 = AtomicU64::new(0);
static JOB_RISE_FROM: AtomicU64 = AtomicU64::new(0);
/// Set while a job is measured, from its first slice to its end.
static MEASURING: AtomicBool = AtomicBool::new(false);
/// Set while the measured job waits between slices, keeping the heap's mark.
static MEASURED_BETWEEN: AtomicBool = AtomicBool::new(false);
/// The carved bytes the measured job started from.
static MEASURED_FROM: AtomicU64 = AtomicU64::new(0);

/// How far the heap's carved bytes rose above where the measured job began,
/// while one is measured: the heap a compile in that job cost, which the
/// pipeline store keeps with what it compiled. Zero outside a measured job.
pub(in crate::engine) fn job_heap_rise() -> u64 {
    if !MEASURING.load(Ordering::Acquire) {
        return 0;
    }
    (crate::ALLOCATOR.marked_peak() as u64).saturating_sub(MEASURED_FROM.load(Ordering::Relaxed))
}

/// Runs `run`, and for a job it began, keeps how far the heap's carved
/// bytes rose above where they were until the job ended: a compile's own
/// peak, as jobs run one at a time, with what other processors carved
/// meanwhile. One job is measured at a time, as the heap keeps one mark.
/// `between`: a job waits between slices, which `run` may resume.
fn measured_job(between: bool, run: impl FnOnce() -> Option<JobSlice>) -> Option<JobSlice> {
    let measure = !between
        && MEASURING
            .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
            .is_ok();
    if measure {
        MEASURED_FROM.store(crate::ALLOCATOR.mark() as u64, Ordering::Relaxed);
    }
    let step = run();
    let began = measure && step.is_some_and(|step| step.started);
    let ended = step.is_some_and(|step| step.finished)
        && (began || MEASURED_BETWEEN.swap(false, Ordering::AcqRel));
    if ended {
        let from = MEASURED_FROM.load(Ordering::Relaxed);
        let rise = (crate::ALLOCATOR.marked_peak() as u64).saturating_sub(from);
        if rise > JOB_RISE.load(Ordering::Relaxed) {
            JOB_RISE.store(rise, Ordering::Relaxed);
            JOB_RISE_FROM.store(from, Ordering::Relaxed);
        }
        MEASURING.store(false, Ordering::Release);
    } else if began {
        MEASURED_BETWEEN.store(true, Ordering::Release);
    } else if measure {
        MEASURING.store(false, Ordering::Release);
    }
    step
}

/// Runs `body` with this processor's interrupts on.
///
/// # Safety
/// Nothing `body`'s caller holds is taken by an interrupt handler.
unsafe fn with_interrupts<R>(body: impl FnOnce() -> R) -> R {
    // SAFETY: the caller's.
    unsafe { core::arch::asm!("sti", options(nomem, nostack)) };
    let result = body();
    // SAFETY: back to how the caller ran.
    unsafe { core::arch::asm!("cli", options(nomem, nostack)) };
    result
}

/// Logs why the service refused a guest's call, once for each reason.
fn report(reason: &str) {
    static SEEN: Seen<512> = Seen::new();
    if let Some(unheld) = SEEN.first_time(reason) {
        output::arguments(format_args!("GPU REFUSED {reason}{unheld}\n"));
    }
    // A reopen that failed may leave macOS no GPU: the next pump decides,
    // outside the client's call.
    if reason.starts_with(REOPEN_FAILED) {
        LOSS_DUE.store(true, Ordering::Release);
    }
}

/// A lost device's reopen failed since the pump last looked ([`report`]).
static LOSS_DUE: AtomicBool = AtomicBool::new(false);

/// The kernel drivers' lines that a GPU stopped answering, each GPU's
/// first, for the reason of a stop that follows.
static SILENT: SilentLines<4, 320> = SilentLines::new();

/// A line a kernel driver printed: one that says its GPU stopped answering
/// is kept for [`watch_losses`].
pub(in crate::engine) fn driver_line(line: &str) {
    SILENT.observe(line);
    // What the port above reads when its GPU went silent or came back.
    if let Some(event) = gpu_loss::link_event(line)
        && let Some(gpu) = installed()
        && let Some(bound) = gpu
            .start
            .bound
            .iter()
            .find(|bound| line.contains(&bound.name))
    {
        super::link::log_port_at(&bound.name, event, bound.port.as_ref());
    }
}

/// End the boot once no GPU can serve macOS again: every GPU that serves
/// Metal is lost and did not reopen ([`gpu_loss::verdict`]). Until then each
/// new WindowServer is refused and dies, on a frozen screen. The stop is
/// the one a boot without a GPU makes, with which GPU was lost and what
/// its driver said of it. A reopen that gives a device changes nothing.
/// Each look follows a reopen that failed, so a GPU whose driver said
/// nothing is judged by reopens tried across its reset's time.
pub(super) fn watch_losses() {
    if !LOSS_DUE.swap(false, Ordering::AcqRel) {
        return;
    }
    let Some(gpu) = installed() else {
        return;
    };
    let mut losses = Vec::new();
    for (index, service) in shared().iter().enumerate() {
        let Some(loss) = service.loss() else {
            // A call holds the service: the next pump looks again.
            LOSS_DUE.store(true, Ordering::Release);
            return;
        };
        let name = served_gpu(index)
            .and_then(|bound| gpu.start.bound.get(bound))
            .map_or("(unnamed)", |bound| bound.name.as_str());
        losses.push((name, loss, SILENT.of(name)));
    }
    let millis = (gpu.tsc_hz / 1000).max(1);
    let gpus: Vec<gpu_loss::Gpu<'_>> = losses
        .iter()
        .map(|(name, loss, silent)| gpu_loss::Gpu {
            name,
            lost: loss.lost,
            failed_reopens: loss.failed_reopens,
            failing_ms: loss.failing_ticks / millis,
            error: loss.error.as_deref(),
            silent: silent.as_deref(),
        })
        .collect();
    if let Some(reason) = gpu_loss::verdict(&gpus) {
        output::fatal(reason);
    }
}

/// The accelerator connections open now on every service, for the guest
/// session's line; `None` while a call holds a service.
pub(in crate::engine) fn open_connections() -> Option<usize> {
    shared()
        .iter()
        .map(|service| service.memory().map(|memory| memory.connections.len()))
        .sum()
}

/// Logs each distinct error or warning NVK reports, such as why a call failed
/// with only a generic result: in a job too, where no slice ends in it.
fn driver_message(text: &str) {
    static SEEN: Seen<128> = Seen::new();
    if let Some(unheld) = SEEN.first_time(text) {
        slice::without_yields(|| {
            output::arguments(format_args!("GPU DRIVER {text}{unheld}\n"));
            report_peaks();
        });
    }
}

/// Logs what a guest call made or met, such as a render pipeline, once for
/// each distinct note, with the heap's live bytes, its size and its
/// high-water mark after it: a compile that raised the mark shows it against
/// the note before. An integrated GPU's memory note names its share of RAM
/// itself; the heap's figures weigh it only where it has no share and its
/// memory is the heap's.
fn note(text: &str) {
    static SEEN: Seen<512> = Seen::new();
    if let Some(unheld) = SEEN.first_time(text) {
        let heap = crate::ALLOCATOR.usage();
        output::arguments(format_args!(
            "GPU {text} heap={}M/{}M peak={}M{unheld}\n",
            heap.live >> 20,
            heap.total >> 20,
            heap.peak >> 20
        ));
        report_peaks();
    }
}

/// Stacks of the allocations that raised the heap's high-water mark, each
/// the carved bytes then and the image offsets of code on the stack.
const PEAK_SLOTS: usize = 16;
const PEAK_WORDS: usize = 24;
/// The stack words searched for them.
const PEAK_SCANNED: usize = 512;
static PEAKS: [[AtomicU64; PEAK_WORDS + 1]; PEAK_SLOTS] =
    [const { [const { AtomicU64::new(0) }; PEAK_WORDS + 1] }; PEAK_SLOTS];
static PEAKS_TAKEN: AtomicU32 = AtomicU32::new(0);
static PEAKS_SHOWN: AtomicU32 = AtomicU32::new(0);

/// The heap's peak hook: keeps the allocating stack's code offsets. Runs
/// under the heap's lock, so it only reads the stack and stores words; the
/// stack may end within a page of its pointer, as a fresh job's does.
pub(super) fn peak_passed(carved: usize) {
    let slot = PEAKS_TAKEN.load(Ordering::Relaxed) as usize;
    if slot >= PEAK_SLOTS {
        return;
    }
    let mut offsets = [0; PEAK_WORDS];
    let kept = crate::engine::large_blocks::stack_code_offsets(PEAK_SCANNED, &mut offsets);
    let words = &PEAKS[slot];
    words[0].store(carved as u64, Ordering::Relaxed);
    for (word, &offset) in words[1..].iter().zip(&offsets[..kept]) {
        word.store(offset, Ordering::Relaxed);
    }
    PEAKS_TAKEN.store(slot as u32 + 1, Ordering::Release);
}

/// Logs the peak stacks taken since the last call, and once that the
/// slots filled, after which higher peaks show only in `BEAT peak=`.
fn report_peaks() {
    use core::fmt::Write;
    let taken = PEAKS_TAKEN.load(Ordering::Acquire);
    let shown = PEAKS_SHOWN.swap(taken, Ordering::AcqRel);
    if shown < taken && taken as usize == PEAK_SLOTS {
        output::arguments(format_args!(
            "HEAP PEAK the {PEAK_SLOTS} stack slots are full; later peaks are not traced\n"
        ));
    }
    for words in &PEAKS[shown as usize..taken as usize] {
        let mut line = format!(
            "HEAP PEAK carved={}M stack",
            words[0].load(Ordering::Relaxed) >> 20
        );
        for word in &words[1..] {
            let offset = word.load(Ordering::Relaxed);
            if offset != 0 {
                let _ = write!(line, " {offset:#x}");
            }
        }
        output::arguments(format_args!("{line}\n"));
    }
}

/// A guest call waits for a job no processor started: a PE whose guest
/// sleeps waiting for an interrupt is called to start it now. A sleeping PE
/// wakes only for its guest or the host's service, so a queued job waited
/// for the next PE that stopped to wait, or 20 ms for a busy one
/// (`Taker::Busy`), while the guest's call slept and asked again.
fn job_waiting() {
    super::call_idle_core();
}

/// What the kext found of the Metal plugin, the bundle whose code this boot's
/// trust cache lists. A multi-user boot's Metal clients cannot start without
/// it. A missing plugin, whose directories a kext cannot create, is repaired
/// once by a single-user boot the engine drives (`bundle_repair`): this boot
/// restarts into it. Otherwise the boot stops with the remedy rather than let
/// WindowServer fail. The stop and the restart are orderly, not fatal: the
/// GPUs' firmware is running by now, and only the boot CPU's service,
/// outside Linux's callbacks, unloads it before the reset that would
/// otherwise take down the card's link.
fn bundle(report: BundleReport) {
    use super::super::bundle_repair;
    use stela_boot_core::bundle_repair::{Decision, Report};
    let decided = bundle_repair::decide(match report {
        BundleReport::Current => Report::Current,
        BundleReport::Updated => Report::Updated,
        BundleReport::Missing => Report::Missing,
        BundleReport::Failed => Report::Failed,
    });
    match (decided, report) {
        (Decision::Continue(_), BundleReport::Updated) => {
            output::arguments(format_args!(
                "GPU StelaGPU.bundle updated to this stela.efi's\n"
            ));
        }
        (Decision::Continue(_), _) => {
            output::arguments(format_args!("GPU StelaGPU.bundle is this stela.efi's\n"));
        }
        (Decision::Log, _) => {
            output::arguments(format_args!(
                "GPU StelaGPU.bundle is not installed ({report:?}); this single-user boot may install it\n"
            ));
        }
        (Decision::Repair, _) => {
            output::arguments(format_args!(
                "GPU StelaGPU.bundle is not installed and a kext cannot create its \
                 directories; restarting once into a single-user boot that creates them\n"
            ));
            super::super::service::request_restart();
        }
        (Decision::StopMissing { attempted }, _) => stop(&bundle_repair::missing(attempted)),
        (Decision::StopFailed, _) => stop(
            "the StelaGPU kext could not install StelaGPU.bundle; its StelaGPU lines above say why",
        ),
    }

    fn stop(reason: &str) {
        output::stopping(reason);
        super::super::service::request_shutdown(true);
    }
}

/// The guest calls one operation's gate served since the last report.
struct Calls {
    /// The operation plus one; zero while the slot is free.
    key: AtomicU32,
    count: AtomicU64,
    /// TSC ticks holding the service, the longest one call held it, and
    /// waiting for another call's turn.
    held: AtomicU64,
    longest: AtomicU64,
    waited: AtomicU64,
    /// Calls answered `Unmapped`, which the frontend repeats.
    unmapped: AtomicU64,
    /// Requests the calls posted to the host's Linux environment, and the
    /// ticks they waited for the boot CPU to serve them.
    posted: AtomicU64,
    posted_waited: AtomicU64,
    /// The longest one wait for the turn, the ticks compiling outside the
    /// lock, and the compilations that were.
    longest_wait: AtomicU64,
    compiled: AtomicU64,
    deferrals: AtomicU64,
}

/// More slots than the protocol has operations.
static CALLS: [Calls; 96] = [const {
    Calls {
        key: AtomicU32::new(0),
        count: AtomicU64::new(0),
        held: AtomicU64::new(0),
        longest: AtomicU64::new(0),
        waited: AtomicU64::new(0),
        unmapped: AtomicU64::new(0),
        posted: AtomicU64::new(0),
        posted_waited: AtomicU64::new(0),
        longest_wait: AtomicU64::new(0),
        compiled: AtomicU64::new(0),
        deferrals: AtomicU64::new(0),
    }
}; 96];

/// The phases of the calls timed by phase since the last report.
static PHASES: Totals = Totals::new();

/// A timed call of a second over this or longer shows a line of its own:
/// 100 ms.
const SLOW_CALL_DIVISOR: u64 = 10;

/// Calls whose record the service could not read, answered by the gate's
/// fault alone (`GPU CALLS unreached=`): the frontend makes each again.
static UNREACHED: AtomicU64 = AtomicU64::new(0);

/// Counts a call the gate served: the PE that rang runs no guest code until
/// it returns, so a call that holds the service long is guest time lost.
fn served(served: Served) {
    report_long_turn(&served);
    note_exit(&served);
    // A record out of reach names no operation and leaves the call closed.
    if served.operation == 0 && served.status == Status::Closed {
        UNREACHED.fetch_add(1, Ordering::Relaxed);
    }
    if let Some(call) = served.phases {
        let tsc_hz = installed().map_or(0, |gpu| gpu.tsc_hz);
        if PHASES.add(&call, tsc_hz / SLOW_CALL_DIVISOR) {
            output::arguments(format_args!("{}\n", call.slow_line(tsc_hz)));
        }
    }
    // The display compares each swap with the GPU's progress.
    if let Some((submitted, completed)) = served.progress {
        stela_soc::dcp::gpu::note(submitted, completed);
    }
    // The PE ran nothing but this call since the last one it served.
    let (posted, posted_waited) = client::take_mine();
    let key = served.operation.wrapping_add(1);
    // Slots fill in order and stay claimed: the first that is free or
    // already this operation's is its own.
    let Some(calls) = CALLS.iter().find(|calls| {
        match calls
            .key
            .compare_exchange(0, key, Ordering::Relaxed, Ordering::Relaxed)
        {
            Ok(_) => true,
            Err(claimed) => claimed == key,
        }
    }) else {
        return;
    };
    calls.count.fetch_add(1, Ordering::Relaxed);
    calls.held.fetch_add(served.held, Ordering::Relaxed);
    calls.longest.fetch_max(served.held, Ordering::Relaxed);
    calls.waited.fetch_add(served.waited, Ordering::Relaxed);
    if served.status == Status::Unmapped {
        calls.unmapped.fetch_add(1, Ordering::Relaxed);
    }
    calls.posted.fetch_add(posted, Ordering::Relaxed);
    calls
        .posted_waited
        .fetch_add(posted_waited, Ordering::Relaxed);
    calls
        .longest_wait
        .fetch_max(served.longest_wait, Ordering::Relaxed);
    calls.compiled.fetch_add(served.compiled, Ordering::Relaxed);
    calls
        .deferrals
        .fetch_add(u64::from(served.deferrals), Ordering::Relaxed);
}

/// Counts each exit against the target, and says once a boot when one
/// reached [`LONG_EXIT_MILLIS`]: `GPU LONG EXIT operation=… status=…
/// waited_us=… held_us=…`.
fn note_exit(served: &Served) {
    static REPORTED: AtomicBool = AtomicBool::new(false);
    let exit = served.waited.wrapping_add(served.held);
    let tsc_hz = installed().map_or(0, |gpu| gpu.tsc_hz);
    EXITS.note(exit, served.operation, served.gave_up, tsc_hz);
    if tsc_hz == 0
        || exit < tsc_hz / 1000 * LONG_EXIT_MILLIS
        || REPORTED.swap(true, Ordering::Relaxed)
    {
        return;
    }
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz);
    output::arguments(format_args!(
        "GPU LONG EXIT operation={:#x} status={:?} waited_us={} held_us={} (once a boot)\n",
        served.operation,
        served.status,
        micros(served.waited),
        micros(served.held),
    ));
}

/// `GPU LONG TURN operation=… phase=… held_us=…`, once for each operation
/// and phase whose turn reached [`LONG_TURN_MILLIS`], and each time for a
/// phase that copies through a texture's memory
/// (`CallPhases::reported_each_time`): the phase is the timed operation's
/// longest, `untimed` for an operation timed by no phase.
fn report_long_turn(served: &Served) {
    static SEEN: Seen<128> = Seen::new();
    let tsc_hz = installed().map_or(0, |gpu| gpu.tsc_hz);
    if tsc_hz == 0 || served.held < tsc_hz / 1000 * LONG_TURN_MILLIS {
        return;
    }
    let phase = served
        .phases
        .as_ref()
        .map_or("untimed", |phases| phases.longest_phase().0);
    let key = format!("{:#x} {phase}", served.operation);
    let each_time = served
        .phases
        .as_ref()
        .is_some_and(|phases| phases.reported_each_time());
    let shown = match each_time {
        true => Some(Unheld(0)),
        false => SEEN.first_time(&key),
    };
    if let Some(unheld) = shown {
        let detail = served
            .phases
            .as_ref()
            .map(|phases| format!("{}", phases.detail))
            .unwrap_or_default();
        output::arguments(format_args!(
            "GPU LONG TURN operation={:#x} phase={phase} held_us={}{detail}{unheld}\n",
            served.operation,
            u128::from(served.held) * 1_000_000 / u128::from(tsc_hz),
        ));
    }
}

/// What a device holds: its allocation with the part waiting for submitted
/// work and the part of that only held work keeps (now and its peak), that
/// work, the resources automatic heaps answered nil for (no
/// room / room only in pieces), its memory blocks with the resources placed in them and
/// those in memory of their own, its upload chunks, the allocations that ran out of
/// memory, the command buffers whose commit failed (in all / for want of memory), and
/// the connections counting the most
/// bytes, each as
/// id:objects/MiB. Nothing while a call holds the service.
fn report_memory(service: &Shared) {
    const SHOWN: usize = 4;
    let Some(mut memory) = service.memory() else {
        return;
    };
    let mib = |bytes: u64| bytes >> 20;
    let device = memory.device;
    output::arguments(format_args!(
        "GPU MEMORY allocated={}M deferred={}/{}M pinned={}/{}M/{}M in_flight={} completed={}/{} recycled={}/{}M reused={}/{} heap_nils={}/{} blocks={}/{}M placed={}/{}M own={} chunks={}/{}M oom={} dropped={}/{} connections={}",
        mib(device.allocated),
        device.deferred,
        mib(device.deferred_bytes),
        device.pinned,
        mib(device.pinned_bytes),
        mib(device.peak_pinned_bytes),
        device.in_flight,
        device.completed,
        device.submitted,
        device.recycled.textures,
        mib(device.recycled.bytes),
        device.recycled.hits,
        device.recycled.hits + device.recycled.misses,
        device.heap_refusals.full,
        device.heap_refusals.fragmented,
        device.blocks.blocks,
        mib(device.blocks.block_bytes),
        device.blocks.placed,
        mib(device.blocks.placed_bytes),
        device.blocks.own,
        device.chunks.chunks,
        mib(device.chunks.bytes),
        device.out_of_memory,
        memory.dropped.command_buffers,
        memory.dropped.out_of_memory,
        memory.connections.len(),
    ));
    memory
        .connections
        .sort_unstable_by_key(|connection| core::cmp::Reverse(connection.allocated));
    for connection in memory.connections.iter().take(SHOWN) {
        output::arguments(format_args!(
            " {}:{}/{}M",
            connection.id,
            connection.objects,
            mib(connection.allocated)
        ));
    }
    output::arguments(format_args!("\n"));
}

/// The operations whose calls held the service longest since the last
/// report: each as count/held/longest/waited/unmapped/posted/posted_waited,
/// microseconds between.
pub(in crate::engine) fn report_calls(tsc_hz: u64) {
    const SHOWN: usize = 8;
    // Peaks a compile or allocation raised with no GPU line after it.
    report_peaks();
    shared().iter().for_each(|service| report_memory(service));
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    // How long submitted work took to be seen complete: the GPU's time plus
    // the wait for its interrupt and a look at the device.
    for service in shared().iter() {
        let (count, total, longest) = service.take_latency();
        if let Some(average) = total.checked_div(count) {
            output::arguments(format_args!(
                "GPU LATENCY completed={count} average_us={} longest_us={}\n",
                micros(average),
                micros(longest),
            ));
        }
    }
    // Which part of that latency was waiting for the GPU to start (behind
    // earlier work, or on the host's way to the GPU), the GPU's own time,
    // and waiting for the look after it finished.
    for service in shared().iter() {
        let Some(times) = service.gpu_times() else {
            continue;
        };
        if times.timed == 0 && times.waiting == 0 {
            continue;
        }
        let average = |total: u64, count: u64| total.checked_div(count).unwrap_or(0);
        output::arguments(format_args!(
            "GPU TIMES timed={} busy_us={} average_us={} longest_us={}@{}:{} idle_us={} \
             overlap_us={} placed={} queued_us={}/{} behind_us={} noticed_us={}/{} waiting={} \
             span_us={}@{}:{}/{}:{} longest_submits={}\n",
            times.timed,
            times.busy_ns / 1000,
            average(times.busy_ns, times.timed) / 1000,
            times.longest_ns / 1000,
            times.longest_serial,
            times.longest_entries,
            times.idle_ns / 1000,
            times.overlap_ns / 1000,
            times.placed,
            micros(average(times.queued_ticks, times.placed)),
            micros(times.longest_queued_ticks),
            micros(average(times.behind_ticks, times.placed)),
            micros(average(times.noticed_ticks, times.placed)),
            micros(times.longest_noticed_ticks),
            times.waiting,
            times.span_ns / 1000,
            times.span_serial,
            times.span_index,
            times.span_count,
            times.span_entries,
            times.longest_submits,
        ));
    }
    // With `--profile`: the GPU's own time per pass, conversion and copy.
    if pass_times::pass_profile() {
        static UNTIMED: AtomicBool = AtomicBool::new(false);
        for service in shared().iter() {
            if service.has_timestamps() == Some(false) && !UNTIMED.swap(true, Ordering::Relaxed) {
                output::arguments(format_args!(
                    "GPU PASSES unavailable: the device has no timestamp queries\n"
                ));
            }
            if let Some(passes) = service.pass_times()
                && passes.command_buffers != 0
            {
                output::arguments(format_args!("{passes}\n"));
            }
        }
    }
    // The display's presents, and how long they held the boot CPU while a
    // posted call waited for it.
    let [presents, total, longest, on_boot, blocked] = client::take_presents();
    if presents != 0 {
        output::arguments(format_args!(
            "GPU PRESENTS count={presents} on_boot={on_boot} total_us={} longest_us={} \
             blocking_us={}\n",
            micros(total),
            micros(longest),
            micros(blocked),
        ));
    }
    // Pipelines created through the pipeline store's cache: found there, or
    // compiled by the driver, and the driver's time for each.
    let store = pipeline_store::take_lookups();
    if store.hits != 0 || store.misses != 0 {
        output::arguments(format_args!(
            "GPU STORE hits={} misses={} hit_us={} miss_us={} miss_longest_us={}\n",
            store.hits,
            store.misses,
            store.hit_ns / 1000,
            store.miss_ns / 1000,
            store.miss_longest_ns / 1000,
        ));
    }
    // Pipeline work in turns: the jobs turns left, with what making them
    // (each stage's NIR copied for its job) took, and the pipelines turns
    // created themselves.
    let [
        [left, copied, copy_longest],
        [created, create, create_longest],
        [looked, look, look_longest],
    ] = native::take_pipeline_turn_work();
    if left != 0 || created != 0 || looked != 0 {
        output::arguments(format_args!(
            "GPU PIPELINE TURNS jobs_left={left} copy_us={} copy_longest_us={} created={created} \
             create_us={} create_longest_us={} archive_looks={looked} look_us={} \
             look_longest_us={}\n",
            micros(copied),
            micros(copy_longest),
            micros(create),
            micros(create_longest),
            micros(look),
            micros(look_longest),
        ));
    }
    // Where the time of the slowest operations went, phase by phase.
    for taken in PHASES.take() {
        output::arguments(format_args!("{}\n", taken.line(tsc_hz)));
    }
    for service in shared().iter() {
        if let Some(line) = service.compiler_report() {
            output::arguments(format_args!("{line}\n"));
        }
        // The compilations calls deferred, which ran outside every turn.
        let jobs = service.take_job_report();
        if jobs.ran != 0 || jobs.deferred != 0 {
            output::arguments(format_args!(
                "GPU JOBS ran={} total_us={} longest_us={} slices={} longest_slice_us={} \
                 longest_queued_us={} deferred={} repeated={} waiting={} heap_rise={}M from={}M \
                 queued_mean_us={} resumed={} resumed_mean_us={} resumed_longest_us={}\n",
                jobs.ran,
                micros(jobs.ticks),
                micros(jobs.longest),
                jobs.slices,
                micros(jobs.longest_slice),
                micros(jobs.longest_queued),
                jobs.deferred,
                jobs.repeated,
                service.jobs_waiting(),
                JOB_RISE.swap(0, Ordering::Relaxed) >> 20,
                JOB_RISE_FROM.load(Ordering::Relaxed) >> 20,
                micros(jobs.queued / jobs.ran.max(1)),
                jobs.resumed,
                micros(jobs.resumed_after / jobs.resumed.max(1)),
                micros(jobs.longest_resumed_after),
            ));
        }
    }
    report_lock(tsc_hz);
    // Every beat with exits says whether each stayed under the target.
    if let Some(exits) = EXITS.take() {
        output::arguments(format_args!("{}\n", exits.line(tsc_hz)));
    }
    let mut taken: Vec<_> = CALLS
        .iter()
        .filter_map(|calls| {
            let key = calls.key.load(Ordering::Relaxed);
            let count = calls.count.swap(0, Ordering::Relaxed);
            (key != 0 && count != 0).then(|| {
                (
                    key - 1,
                    count,
                    calls.held.swap(0, Ordering::Relaxed),
                    calls.longest.swap(0, Ordering::Relaxed),
                    calls.waited.swap(0, Ordering::Relaxed),
                    calls.unmapped.swap(0, Ordering::Relaxed),
                    calls.posted.swap(0, Ordering::Relaxed),
                    calls.posted_waited.swap(0, Ordering::Relaxed),
                )
            })
        })
        .collect();
    if let Some(pool) = client::pool_report(tsc_hz) {
        output::arguments(format_args!("{pool}\n"));
    }
    let posted = client::take_posted();
    if taken.is_empty() && posted.is_empty() {
        return;
    }
    taken.sort_unstable_by_key(|&(_, _, held, ..)| core::cmp::Reverse(held));
    let held: u64 = taken.iter().map(|&(_, _, held, ..)| held).sum();
    output::arguments(format_args!(
        "GPU CALLS held_us={} unreached={} \
         operation:count/held_us/longest_us/waited_us/unmapped/posted/posted_us",
        micros(held),
        UNREACHED.swap(0, Ordering::Relaxed),
    ));
    for &(operation, count, held, longest, waited, unmapped, posted, posted_waited) in
        taken.iter().take(SHOWN)
    {
        output::arguments(format_args!(
            " {operation:#x}:{count}/{}/{}/{}/{unmapped}/{posted}/{}",
            micros(held),
            micros(longest),
            micros(waited),
            micros(posted_waited),
        ));
    }
    // The host's Linux work the calls asked the boot CPU for, by ioctl.
    let (shootdowns, shot, ranged, nmis, [owed, held]) =
        crate::native_mmio::shootdown::take_waits();
    output::arguments(format_args!(
        "\nGPU POSTED shootdowns={shootdowns}/{}us ranged={ranged} nmis={nmis} \
         owed_nmis={owed}/{held} request:count/waited_us",
        micros(shot)
    ));
    for &(request, count, waited) in posted.iter().take(SHOWN) {
        output::arguments(format_args!(
            " {}:{count}/{}",
            stela_boot_core::gpu_requests::Request(request),
            micros(waited)
        ));
    }
    // How long they waited for the boot CPU to take them.
    output::arguments(format_args!("\nGPU TAKEN request:count/us"));
    for &(request, count, taken) in client::take_taken().iter().take(SHOWN) {
        output::arguments(format_args!(
            " {}:{count}/{}",
            stela_boot_core::gpu_requests::Request(request),
            micros(taken)
        ));
    }
    // How long they waited for a worker to take them.
    output::arguments(format_args!("\nGPU QUEUED request:count/us"));
    for &(request, count, queued) in client::take_queued().iter().take(SHOWN) {
        output::arguments(format_args!(
            " {}:{count}/{}",
            stela_boot_core::gpu_requests::Request(request),
            micros(queued)
        ));
    }
    // What performing them took the service task, waits for the GPU included.
    output::arguments(format_args!("\nGPU SERVED request:count/us"));
    for &(request, count, took) in client::take_served().iter().take(SHOWN) {
        output::arguments(format_args!(
            " {}:{count}/{}",
            stela_boot_core::gpu_requests::Request(request),
            micros(took)
        ));
    }
    output::arguments(format_args!("\n"));
}

/// `GPU LOCK operation:held_us/waited_us/longest_wait_us/compiled_us/compiles`:
/// for each operation that held or waited for the service's turn since the
/// last report, the turn's time, the waits for it, the longest one wait,
/// and the compilations it ran outside the turn. Reads before
/// [`report_calls`]'s `GPU CALLS`, which takes the counts.
fn report_lock(tsc_hz: u64) {
    const SHOWN: usize = 12;
    let micros = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
    let mut rows: Vec<_> = CALLS
        .iter()
        .filter_map(|calls| {
            let key = calls.key.load(Ordering::Relaxed);
            let longest_wait = calls.longest_wait.swap(0, Ordering::Relaxed);
            let compiled = calls.compiled.swap(0, Ordering::Relaxed);
            let deferrals = calls.deferrals.swap(0, Ordering::Relaxed);
            let held = calls.held.load(Ordering::Relaxed);
            let waited = calls.waited.load(Ordering::Relaxed);
            (key != 0 && (held | waited | compiled) != 0).then_some((
                key - 1,
                held,
                waited,
                longest_wait,
                compiled,
                deferrals,
            ))
        })
        .collect();
    if rows.is_empty() {
        return;
    }
    rows.sort_unstable_by_key(|&(_, held, waited, ..)| core::cmp::Reverse(held + waited));
    let mut line =
        String::from("GPU LOCK operation:held_us/waited_us/longest_wait_us/compiled_us/compiles");
    for &(operation, held, waited, longest, compiled, deferrals) in rows.iter().take(SHOWN) {
        line.push_str(&format!(
            " {operation:#x}:{}/{}/{}/{}/{deferrals}",
            micros(held),
            micros(waited),
            micros(longest),
            micros(compiled),
        ));
    }
    output::arguments(format_args!("{line}\n"));
}

/// The host pages behind whole pages of guest RAM.
fn guest_pages(ranges: &[MemoryRange]) -> Result<Vec<Run>, Error> {
    const NOT_RAM: Error = Error::Invalid("imported memory is whole pages of guest RAM");
    let mut runs: Vec<Run> = Vec::new();
    for range in ranges {
        if !(range.address | range.bytes).is_multiple_of(PAGE) {
            return Err(NOT_RAM);
        }
        let whole = host::guest_runs(range.address, range.bytes, |physical, bytes| {
            match runs.last_mut() {
                Some(last) if last.physical + last.bytes == physical => last.bytes += bytes,
                _ => runs.push(Run { physical, bytes }),
            }
        });
        if !whole {
            return Err(NOT_RAM);
        }
    }
    Ok(runs)
}

/// Guest RAM the kext wired, for the Vulkan driver to import. The GPU reaches it through
/// the host pages behind it.
struct Exporter;

impl MemoryExporter for Exporter {
    fn export(&self, ranges: &[MemoryRange]) -> Result<ExportedMemory, Error> {
        let runs = guest_pages(ranges)?;
        let (fd, pointer) = client::export(&runs).ok_or(Error::OutOfMemory)?;
        let bytes = runs.iter().map(|run| run.bytes).sum();
        Ok(ExportedMemory { fd, pointer, bytes })
    }

    fn close(&self, exported: ExportedMemory) {
        client::close_descriptor(exported.fd);
        self.unmap(exported);
    }

    fn unmap(&self, exported: ExportedMemory) {
        client::unmap_export(exported.pointer.as_ptr() as u64, exported.bytes);
    }
}

fn service(
    gpu: &Gpu,
    stack: u64,
    watch: Option<client::ResetWatch>,
) -> Result<Arc<Shared>, OpenError> {
    let driver = gpu.stack.vulkan.as_ref().ok_or(Error::Invalid(
        "the GPU's stack drives displays only: it has no Vulkan driver",
    ))?;
    let compiler = |physical| {
        compiler::Service::linked(core::array::from_fn(|stage| {
            // SAFETY: the driver's own physical device, whose options last
            // as long.
            let options = unsafe { (driver.nir_options)(physical, stage as u32) };
            NonNull::new(options.cast_mut()).expect("the Vulkan driver has NIR options per stage")
        }))
    };
    // An import's driver work after its turn may wait for Linux in `relax`.
    let loader: Loader = driver.loader(driver_message, relax);
    // SAFETY: the stack's linked Vulkan driver and the compiler, which last
    // for the boot.
    let service = unsafe {
        open(
            loader,
            Box::new(Exporter),
            compiler,
            gpu,
            DeviceEpoch::from_registered(NonZeroU64::MIN),
        )
    }?;
    let gpu = *gpu;
    // A lost device's channels are gone, so the next client gets a new one,
    // opened on the service's stack as the first was.
    let service = service.with_reopen(Box::new(move || {
        // Not while the kernel driver still resets the GPU once the wait
        // below ended: that reopen fails, as any that cannot open.
        if let Some(refused) = watch
            .as_ref()
            .and_then(client::ResetWatch::state)
            .and_then(ResetState::refuses_reopen)
        {
            return Err(Error::Unsupported(refused));
        }
        // SAFETY: as above.
        unsafe { reopen(loader, Box::new(Exporter), compiler, &gpu) }
    }));
    // While the kernel driver resets the GPU, a reopen waits, at most
    // `RESET_WAIT_SECONDS`: the connection that asked reports why, and the
    // next one asks again. A wait is no failed reopen.
    let service = match watch {
        Some(watch) => {
            let mut wait = ResetWait::default();
            service.with_reopen_wait(Box::new(move || {
                let hz = installed().map_or(0, |gpu| gpu.tsc_hz);
                wait.wait(watch.state(), host::rdtsc(), hz)
            }))
        }
        None => service,
    };
    // A driver that loses its device only with a GPU its kernel driver
    // reset, whose release would wait for that reset's end, keeps the lost
    // device unreleased, as an open whose first submission hung leaves it.
    // Its reopened device serves once an empty submission on it signals
    // within the bound the first open's check has, looked at without
    // waiting in a call: one whose GPU hangs again fails the reopen.
    let service = if driver.keep_lost {
        let service = service.keep_lost_devices();
        match installed().map(|gpu| gpu.tsc_hz).filter(|&hz| hz != 0) {
            Some(hz) => service.check_reopened(
                (u128::from(hz) * u128::from(FIRST_SUBMISSION_NS) / 1_000_000_000) as u64,
            ),
            None => service,
        }
    } else {
        service
    };
    Ok(Arc::new(SharedService::new(
        service,
        ServiceHost {
            relax,
            relax_within,
            stack,
            run,
            report,
            note,
            ticks: host::rdtsc,
            served,
            wake: look_again,
            bundle,
            job_waiting,
        },
    )))
}

/// Open a service over each bound GPU that [`metal_devices`] chooses, the
/// PC's Metal device first. What stops one is reported, and the Mac boots
/// without Metal on that GPU. When none of them opens, the drawing GPUs it
/// left display-only serve instead, with one line each; those that stay
/// display-only are reported once, with the GPU that draws for Metal.
///
/// [`metal_devices`]: stela_drivers::gpu::stack::metal_devices
pub(in crate::engine) fn open_service() {
    let Some(gpu) = installed() else {
        return;
    };
    let bound = &gpu.start.bound;
    let chosen = metal_choice(bound, super::display::taken_index());
    let mut constructed = false;
    let mut open = |indices: &[usize]| -> Vec<(usize, Arc<Shared>)> {
        indices
            .iter()
            .filter_map(|&index| {
                open_gpu(gpu, index, &bound[index], &mut constructed)
                    .map(|service| (index, service))
            })
            .collect()
    };
    let mut opened = open(&chosen.serve);
    if let Some(&(serving, _)) = opened.first() {
        for &index in &chosen.display_only {
            output::arguments(format_args!(
                "GPU {}: shows displays only: GPU {}, a discrete GPU, draws for Metal\n",
                bound[index].name, bound[serving].name
            ));
        }
    } else {
        opened = open(&chosen.display_only);
        for &(index, _) in &opened {
            output::arguments(format_args!(
                "GPU {}: draws for Metal, as no discrete GPU opened a Metal device\n",
                bound[index].name
            ));
        }
    }
    let (served, services): (Vec<_>, Vec<_>) = opened.into_iter().unzip();
    SERVED.store(Box::into_raw(Box::new(served)), Ordering::Release);
    SERVICES.store(Box::into_raw(Box::new(services)), Ordering::Release);
}

/// The bound GPU, by its index, that service `service` runs on: services
/// follow [`metal_choice`]'s order, which puts the GPU driving the display
/// first and leaves out failed GPUs and, while a chosen GPU opened,
/// display-only ones, so it is not the claims' order.
pub(super) fn served_gpu(service: usize) -> Option<usize> {
    // SAFETY: published once, before any service exists, and leaked.
    unsafe { SERVED.load(Ordering::Acquire).as_ref() }
        .and_then(|served| served.get(service).copied())
}

/// Which bound GPUs serve Metal devices, given the one whose output took
/// the connected display.
pub(super) fn metal_choice(
    bound: &[Bound],
    display: Option<usize>,
) -> stela_drivers::gpu::stack::MetalDevices {
    let candidates: Vec<_> = bound
        .iter()
        .enumerate()
        .map(|(index, bound)| stela_drivers::gpu::stack::Candidate {
            draws: bound.stack.draws(),
            integrated: integrated(index, bound),
            drives_display: display == Some(index),
        })
        .collect();
    stela_drivers::gpu::stack::metal_devices(&candidates)
}

/// Whether the bound GPU at `index` is integrated: its kernel driver's
/// answer for the bound device where it gives one, else its table's. A
/// driver that decides during its probe and does not answer leaves the
/// kind unknown, and the GPU counts as integrated, so a discrete GPU takes
/// the Metal device first; that is reported once per GPU.
fn integrated(index: usize, bound: &Bound) -> bool {
    /// The bound GPUs, by index, whose unanswered query was reported.
    static REPORTED: AtomicU64 = AtomicU64::new(0);
    let Some(query) = bound.stack.kernel.probed_integrated else {
        return bound.integrated.unwrap_or(false);
    };
    if let Some(integrated) = client::query(index as u32, query) {
        return integrated;
    }
    let integrated = bound.integrated.unwrap_or(true);
    let bit = 1u64 << index.min(63);
    if REPORTED.fetch_or(bit, Ordering::Relaxed) & bit == 0 {
        output::arguments(format_args!(
            "GPU {}: {} did not say whether it is integrated, so it counts as {}\n",
            bound.name,
            bound.stack.kernel.driver,
            if integrated { "integrated" } else { "discrete" }
        ));
    }
    integrated
}

/// Runs the image's C constructors once, on the stack of whoever opens first.
fn construct(constructed: &mut bool) {
    if !core::mem::replace(constructed, true) {
        // SAFETY: the one call, before any other of the C's.
        unsafe { stela_mesa::stela_constructors() };
        // The drivers' pipeline cache UUID is every Stela build's; their
        // archives' hash names this build in the guest's archive files.
        stela_drivers::gpu::metal::device::pipelines::set_build_identity(
            stela_mesa::COMPILER_SHA256.as_bytes(),
        );
    }
}

/// The service of `bound`, the bound GPU at `index`, whose failure is reported.
fn open_gpu(
    gpu: &Installed,
    index: usize,
    bound: &Bound,
    constructed: &mut bool,
) -> Option<Arc<Shared>> {
    let refuse = |reason: &str| {
        crate::output::arguments(format_args!("GPU {}: {reason}\n", bound.name));
        None
    };
    if !bound.stack.draws() {
        // Its kernel driver shows frames; `display` takes its outputs.
        return None;
    }
    // Read before the GPU's first work, for the guest's `StelaHost`, with
    // what the driver says of the chip, for the summary below: once the GPU
    // hung, a file's close may wait for the driver's reset.
    let facts = client::remember_memory(index as u32, bound.stack);
    let shares_memory = integrated(index, bound);
    let link = match PcieLink::admitted(bound.link, shares_memory) {
        Ok(link) => link,
        Err(reason) => return refuse(reason),
    };
    let Some(chip) = client::query(index as u32, bound.stack.kernel.chip) else {
        return refuse("no chipset reported");
    };
    let Some(stack) = Stack::map(gpu.window, STACK_BYTES) else {
        return refuse("no memory for its service");
    };
    let top = stack.top();
    // It serves for the boot.
    core::mem::forget(stack);
    let target = Gpu {
        stack: bound.stack,
        pci: PciBusInfo {
            domain: bound.drm.segment.into(),
            bus: bound.drm.bus.into(),
            device: bound.drm.slot.into(),
            function: bound.drm.function.into(),
        },
        chip,
        // A GPU in a Thunderbolt enclosure reaches Stela as the function
        // firmware enumerated, like any other in a slot.
        attachment: Attachment {
            link,
            external: false,
            location_number: index as u64,
        },
    };
    // Made before the first submission, so a reset it caused shows on it.
    let driver_name = bound.stack.kernel.driver;
    let watch = bound.stack.kernel.reset.as_ref().and_then(|reset| {
        let watch = client::ResetWatch::new(index as u32, reset);
        if watch.is_none() {
            output::arguments(format_args!(
                "GPU {}: no {driver_name} context to watch for resets\n",
                bound.name
            ));
        }
        watch
    });
    let mut opened = None;
    run(top, &mut || {
        construct(constructed);
        opened = Some(service(&target, top, watch));
    });
    match opened? {
        Ok(service) => {
            if let Some(state @ (ResetState::Reset | ResetState::Resetting)) =
                watch.as_ref().and_then(client::ResetWatch::state)
            {
                // The first submission hung the GPU and the driver's recovery
                // took it: its fence was signaled by the reset, not the GPU.
                // Every later file close and submission waits on that reset,
                // which on Navi 21 has not been seen to end. The service is
                // kept unreleased, as its release closes the driver's file.
                output::arguments(format_args!(
                    "GPU {}: {driver_name} reset the GPU during its first submission{}; \
                     the Mac boots without Metal on it\n",
                    bound.name,
                    if state == ResetState::Resetting {
                        ", reset still in progress"
                    } else {
                        ""
                    },
                ));
                core::mem::forget(service);
                // The reset unmapped the display's buffers too.
                super::display::abandon();
                summarize(bound, &facts, shares_memory, None);
                return None;
            }
            output::arguments(format_args!(
                "GPU {}: Metal service open at DRM minor {index}\n",
                bound.name
            ));
            summarize(bound, &facts, shares_memory, Some(&*service));
            if let Some(shortfall) = service.family_shortfall() {
                output::arguments(format_args!(
                    "GPU {}: {shortfall}; macOS runs on Apple GPUs of both families\n",
                    bound.name
                ));
            }
            // Without a second stack, every compilation stays in its turn.
            service.set_wait_bound(gpu.tsc_hz / 1000 * WAIT_BOUND_MILLIS);
            service.set_look_budget(gpu.tsc_hz / 1_000_000 * LOOK_MICROS);
            service.set_library_budget(gpu.tsc_hz / 1_000_000 * LIBRARY_MICROS);
            if let Some(compiling) = Stack::map(gpu.window, STACK_BYTES) {
                if let Some(free) = COMPILE_STACKS
                    .iter()
                    .find(|compile| compile.load(Ordering::Relaxed) == 0)
                {
                    free.store(compiling.top(), Ordering::Relaxed);
                }
                service.set_compile_stack(compiling.top());
                // Jobs run in slices there; their allocations end them,
                // but never while they hold a driver's spin lock.
                stela_drivers::gpu::metal::deferred::set_lock_hooks(
                    slice::enter_lock,
                    slice::leave_lock,
                );
                service.set_job_slicer(slice::start);
                stela_boot_core::heap::set_allocation_hook(slice::allocation_point);
                core::mem::forget(compiling);
                // Imports' exports and driver imports run after their turn
                // on a stack of their own; without one they stay in it. So
                // do the frees and semaphores the boot CPU's look keeps up.
                if let Some(outside) = Stack::map(gpu.window, STACK_BYTES) {
                    service.set_outside_stack(outside.top());
                    core::mem::forget(outside);
                }
            } else {
                output::arguments(format_args!(
                    "GPU {}: no memory for a compile stack; compilations hold the service\n",
                    bound.name
                ));
            }
            Some(service)
        }
        Err(error) => {
            let refused = refuse(&format!("Metal could not open: {error:?}"));
            summarize(bound, &facts, shares_memory, None);
            refused
        }
    }
}

/// The one `GPU SUMMARY` line of `bound` as its Metal service opened, or
/// did not: what a triager reads first of a GPU that fails on a PC nobody
/// here can test. `facts` is what its kernel driver said of the chip
/// ([`client::describe`]); the heaps are the Vulkan driver's, where its
/// device opened.
fn summarize(bound: &Bound, facts: &str, integrated: bool, service: Option<&Shared>) {
    let (working_set, heaps) = match service.and_then(|service| service.memory_plan()) {
        Some((working_set, heaps)) => {
            let mut text = String::new();
            for heap in heaps {
                let kind = if heap.device_local { "local" } else { "host" };
                text.push_str(&format!(" {}M/{kind}", heap.size >> 20));
            }
            (Some(working_set), text)
        }
        None => (None, String::new()),
    };
    let heap = crate::ALLOCATOR.usage();
    // Only an integrated GPU works in a share: the probe of one speaks of
    // an integrated GPU, and a discrete GPU's pages are held to the plan's
    // bound alone, which is no share of its own.
    let share = integrated
        .then(stela_drivers::gpu::metal::device::share::probe)
        .flatten()
        .and_then(|probe| probe())
        .map(|state| (state.held, state.limit));
    let pages = match stela_linux::pages::held_and_limit() {
        (held, limit) if !integrated && limit != 0 => {
            Some((held as u64 * PAGE, limit as u64 * PAGE))
        }
        _ => None,
    };
    let summary = stela_boot_core::gpu_triage::Summary {
        address: &bound.name,
        vendor: bound.drm.vendor,
        device: bound.drm.device,
        revision: bound.drm.revision,
        market: &bound.market,
        driver: bound.stack.kernel.driver,
        facts,
        integrated,
        working_set,
        heaps: &heaps,
        share,
        pages,
        heap_live: heap.live,
        heap_total: heap.total,
    };
    output::arguments(format_args!("{summary}\n"));
}

/// `--profile`: every device times its command buffers' passes apart and
/// the report shows `GPU PASSES`.
pub(in crate::engine) fn profile_passes() {
    pass_times::set_pass_profile(true);
}

pub(super) fn shared() -> &'static [Arc<Shared>] {
    // SAFETY: the published services are leaked, so they live for the boot.
    unsafe { SERVICES.load(Ordering::Acquire).as_ref() }.map_or(&[], Vec::as_slice)
}

/// Tell the services a GPU interrupted: its MSI says nothing of which.
pub(super) fn interrupted() {
    shared().iter().for_each(|service| service.interrupted());
}

/// Waits on the completion timeline of each device that has not yet given
/// it, as a newly opened or reopened device has, or whose waiter left by
/// itself, which the device then exports again: completed work is then
/// looked for at once.
pub(super) fn watch_completions() {
    use super::waiter::{self, Due};
    for (index, service) in shared().iter().enumerate() {
        match waiter::due(index) {
            // A call holding the service leaves it due for the next look.
            Due::Restart if service.rearm_completion() => {
                let count = waiter::restarted(index);
                output::arguments(format_args!(
                    "GPU {index}: completion waiter started again ({count})\n"
                ));
            }
            Due::GiveUp => output::arguments(format_args!(
                "GPU {index}: completion waiter left too often; completions are found by \
                 looks\n"
            )),
            Due::Restart | Due::Nothing => {}
        }
        if let Some((fd, timeline)) = service.take_completion() {
            waiter::start(fd, index, timeline);
        }
    }
}

/// Whether service `index` has submitted work not yet published complete.
pub(super) fn awaits_completion(index: usize) -> bool {
    shared()
        .get(index)
        .is_some_and(|service| service.awaits_completion())
}

/// Service `index`'s completion timeline `timeline` reached `value`.
pub(super) fn completed_through(index: usize, timeline: u64, value: u64) {
    if let Some(service) = shared().get(index) {
        service.completed_through(timeline, value);
    }
    // The display's swap gate reads this between GPU calls, so a waiting
    // frame is released by the GPU's own completion, not by the guest's
    // next call.
    stela_soc::dcp::gpu::note_completed(value);
}

/// Bring every GPU to rest before the PC resets or turns off, on the boot CPU
/// outside Linux's callbacks. Admission closes for good and accepted work
/// finishes; `close_gates` then releases what the guest held of each GPU that
/// proved idle, answering the imports still in reach. Whatever could not be
/// proven stays alive and is reported: the transition itself ends the DMA.
pub(in crate::engine) fn stop(close_gates: impl FnOnce() -> u64) {
    let Some(gpu) = installed() else {
        return;
    };
    // From here Linux runs on the boot CPU only, which owns what the stop
    // releases.
    super::close_moves();
    let services = shared();
    let ms = |since: u64| host::rdtsc().wrapping_sub(since) / (gpu.tsc_hz / 1000).max(1);
    let began = host::rdtsc();
    services.iter().for_each(|service| service.begin_quiesce());
    for (index, service) in services.iter().enumerate() {
        let status = settle(service, gpu.tsc_hz);
        output::arguments(format_args!("GPU {index} at the stop: {status:?}\n"));
    }
    let quiesce_ms = ms(began);
    let closing = host::rdtsc();
    let reachable = close_gates();
    let close_ms = ms(closing);
    let quarantine = dma_quarantine_status();
    output::arguments(format_args!(
        "GPU stopped: {reachable} imports in reach, {} DMA mappings of {} bytes and {} functions quarantined\n",
        quarantine.mappings,
        quarantine.bytes,
        quarantined_functions(),
    ));
    super::waiter::stop();
    let unloading = host::rdtsc();
    let interrupts = super::interrupts();
    let done = client::shutdown(STOP_UNLOAD_SECONDS);
    // Whether the GPU still interrupted while its driver unloaded, and
    // whether Linux's timers still ran, which its message waits sleep on.
    output::arguments(format_args!(
        "GPU UNLOAD waited_ms={} done={done} quiesce_ms={quiesce_ms} close_ms={close_ms} \
         interrupts={} timer_failure={:?}\n",
        ms(unloading),
        super::interrupts().wrapping_sub(interrupts),
        stela_linux::service::timer_delivery_failure(),
    ));
    if done {
        output::arguments(format_args!("GPU drivers stopped\n"));
    } else {
        output::arguments(format_args!(
            "GPU drivers still stopping after {STOP_UNLOAD_SECONDS} s for each GPU\n"
        ));
        // The calls still running, the stop among them, and for how long.
        if let Some(line) = client::pool_report(gpu.tsc_hz) {
            output::arguments(format_args!("{line}\n"));
        }
    }
}

/// Unload the GPUs' firmware before a fatal stop ends the machine: the reset
/// that follows takes down the link of a card whose GSP still runs, until a
/// power cycle, and a card whose driver did not stop it (amdgpu's PSP ring,
/// its video decoder) fails its next start. A stop raised inside Linux, on
/// the processor that holds the host drivers' lock, cannot run Linux again
/// and leaves the firmware running, and says so; any other stop unloads, on
/// the boot CPU only where every bound GPU's driver unloads from a stop
/// there (`Kernel::unloads_on_boot_cpu`). Once.
pub(in crate::engine) fn stop_for_fatal() {
    use client::StopUnload;
    static STOPPING: AtomicBool = AtomicBool::new(false);
    let Some(gpu) = installed() else {
        return;
    };
    if STOPPING.swap(true, Ordering::AcqRel) {
        return;
    }
    // On the boot CPU, the stopped work's own state is all around the
    // unload: a driver not shown to unload there keeps the firmware
    // running, as every stop there left it.
    if host::processor() == BOOT_CPU.load(Ordering::Relaxed)
        && let Some(driver) = stela_drivers::gpu::stack::kept_from_boot_cpu_unload(
            gpu.start.bound.iter().map(|bound| &bound.stack.kernel),
        )
    {
        output::arguments(format_args!(
            "GPU firmware left running: the stop is on the CPU that unloads it, and {driver}'s \
             unload is not run from a stop there\n"
        ));
        return;
    }
    use crate::engine::storage;
    // Which of the three put the stop inside Linux, which cannot run again
    // under it.
    let inside = if storage::holds_host_drivers() {
        Some("this processor holds the host drivers' lock")
    } else if storage::host_drivers_held_here() {
        Some("a fault stopped this processor inside the host drivers")
    } else if host::moved_here() {
        Some("this processor runs a process's Linux work in its place")
    } else {
        None
    };
    if let Some(why) = inside {
        output::arguments(format_args!(
            "GPU firmware left running: the stop is inside Linux, which unloads it ({why})\n"
        ));
        return;
    }
    match client::shutdown_for_fatal(STOP_UNLOAD_SECONDS) {
        StopUnload::Stopped => {
            output::arguments(format_args!("GPU drivers stopped before the fatal stop\n"));
        }
        StopUnload::Stopping => output::arguments(format_args!(
            "GPU drivers still stopping after {STOP_UNLOAD_SECONDS} s for each GPU; the firmware may still run\n"
        )),
        StopUnload::NotRun(Some(error)) => output::arguments(format_args!(
            "GPU firmware left running: Linux could not run for the unload ({error:?})\n"
        )),
        StopUnload::NotRun(None) => output::arguments(format_args!(
            "GPU firmware left running: another processor kept the host drivers' lock past \
             the unload's deadline\n"
        )),
    }
}

/// Step `service` until accepted work ends or stops moving.
fn settle(service: &Shared, tsc_hz: u64) -> QuiesceStatus {
    let (mut last, mut moved) = (None, host::rdtsc());
    loop {
        let status = service.quiesce_step();
        let QuiesceStatus::Pending { completed, .. } = status else {
            return status;
        };
        if last != Some(completed) {
            (last, moved) = (Some(completed), host::rdtsc());
        } else if host::rdtsc() - moved >= STOP_STALL_SECONDS * tsc_hz {
            return status;
        }
        relax();
    }
}

/// Add each service's gate node to `tree`, for what the board mounts there.
pub(in crate::engine) fn place(
    tree: &mut DeviceTree,
) -> Vec<(stela_gpu::Placement, Arc<dyn GateService>)> {
    let mut gates = Vec::new();
    for (index, service) in shared().iter().enumerate() {
        match stela_gpu::place_gpu_gate(tree, index, window::BYTES) {
            Ok(placement) => gates.push((placement, service.clone() as Arc<dyn GateService>)),
            Err(error) => {
                crate::status::warn(&format!("GPU {index}'s gate has no place: {error}"));
            }
        }
    }
    gates
}
