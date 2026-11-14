//! The host's service between guest slices: the restore host, the host's
//! inputs, network and NOR, its power button and the log stores. The guest
//! storage's host transfers need none: its controller submits each one and
//! completes it once the disk has (`ans::host`).
//!
//! Guest time passes while a PE's core serves the host, and another PE may
//! spin on a lock the served PE's guest holds; XNU panics when a spinlock
//! is held past its timeout. With every PE on a host core of its own, a PE
//! that waits for an interrupt serves the host ([`serve`]), as it holds no
//! such lock, and a device that leaves bytes for the host wakes one that
//! sleeps (`Hub::request_service`). The boot PE serves whole only while it
//! waits for an interrupt itself, or always when the PEs run serialized.
//! Once no PE has served for [`STARVED_MILLIS`] it takes one bounded step
//! between its slices ([`step`]): the parts in turn, until one ends past
//! [`STEP_MICROS`], the next step going on from the part after it. The
//! shared count stops at what its guest code earned while it serves
//! (`clock`), so a step stops every PE's time for its length.
//!
//! A PE that serves answers no interrupt and runs no timer of its own, and
//! another core's XNU times its answers. So every turn runs inside
//! `Machine::serve_host`, which keeps its PE reachable: a wake that brings
//! it an interrupt kicks its core and marks the shared count, as its own
//! timer's deadline does, and the count holds a stall limit past that if
//! the PE stays out. The turn looks for such guest work before each part
//! after its first (`GuestWatch`, `service_watch::Turn`) and leaves the
//! rest to a later turn, so the guest waits at most for the part that
//! runs; a part that kept it waiting past [`LATE_MILLIS`] is named in the
//! `HOST SERVICE SLOW` line with what waited.
use super::{bluetooth, host, input, network, nor, output, parallel, power, restore, usb};
use alloc::{format, string::String, vec::Vec};
use core::{
    cell::UnsafeCell,
    sync::atomic::{AtomicBool, AtomicU64, Ordering},
};
use stela_boot_core::{
    service_watch::{SlowParts, Turn, late_worth},
    shutdown::{Mode as Shutdown, Requests as ShutdownRequests},
};
use stela_engine::machine::engine::GuestWatch;

static SHUTDOWN: ShutdownRequests = ShutdownRequests::new();

/// Queue a native driver's power request without reentering host services
/// from a Linux callback. Installation of the service does not clear it.
pub(super) fn request_shutdown(force: bool) {
    SHUTDOWN.request(if force {
        Shutdown::Forced
    } else {
        Shutdown::Orderly
    });
}

/// Stop the guest and restart the PC: what NVRAM records at the stop
/// chooses the next boot: the Metal plugin repair's.
pub(super) fn request_restart() {
    SHUTDOWN.request(Shutdown::Restart);
}

pub(super) fn shutdown_pending() -> bool {
    SHUTDOWN.pending()
}

/// How long the host may go unserved before the boot PE serves it: a PE's
/// wait returns to its run loop at least this often
/// (`WAIT_POLLS_PER_SECOND`), so with any PE idle it is served first. A
/// device's request with no PE asleep to take it (bytes for the host while
/// every PE runs) waits [`REQUESTED_MILLIS`] at most.
const STARVED_MILLIS: u64 = 50;
const REQUESTED_MILLIS: u64 = 2;

/// How long a step a PE that runs guest code takes ([`step`]) may go on:
/// about what a slice that ran no blocks takes from its guest code. A part
/// that takes longer still ends.
const STEP_MICROS: u64 = 1000;

/// Seconds between the service's own reports.
const REPORT_SECONDS: u64 = 10;

/// Seconds between stores of the PMU upcount's anchor (`nor::HostNor`),
/// after the first, which goes once the guest's time has started, and
/// before the one at the stop. One 4 KiB write each. The next boot's
/// upcount is the anchor's plus the wall time since it, so with a host wall
/// time a lost last store costs nothing; without one the upcount goes on
/// from the last stored anchor and loses at most this interval.
const ANCHOR_SECONDS: u64 = 600;

/// The longest a part that calls the host drivers (the logs, network,
/// Bluetooth and NOR) is left to a later turn because another core holds
/// the drivers' lock: a turn does not wait for the GPUs' pump, which can
/// hold it for milliseconds, while that part ran recently.
const DEFERRED_MILLIS: u64 = 20;

/// The parts a turn runs, in order: the inputs first, as a pointer's motion
/// is what the guest's user waits on.
const INPUTS: usize = 0;
const LOGS: usize = 1;
const NETWORK: usize = 2;
const BLUETOOTH: usize = 3;
const NOR: usize = 4;
const RESTORE: usize = 5;
const TURN_PARTS: usize = 6;

/// The host interfaces the service owns for the whole run.
pub(super) struct Service {
    devices: parallel::Board,
    restore: Option<restore::Experiment>,
    input: Option<input::HostInput>,
    usb: Option<usb::HostUsb>,
    bluetooth: Option<bluetooth::HostBluetooth>,
    network: Option<network::GuestNetwork>,
    nor: Option<nor::HostNor>,
    power: power::HostPowerControl,
    tsc_hz: u64,
    log_flushed: u64,
    reported: u64,
    /// When the RTC anchor was last stored, `None` before the first.
    anchored: Option<u64>,
    /// The part the next service starts at: after the last one a step
    /// ended on, else the first.
    next: usize,
    /// When each part last ran, for [`DEFERRED_MILLIS`].
    ran: [u64; TURN_PARTS],
    #[cfg(feature = "diagnostics")]
    input_peak: Option<InputTurn>,
}

/// The host's keyboards and pointers, the USB controllers new ones arrive
/// on, and the Bluetooth controller behind the guest's.
pub(super) struct Peripherals {
    pub input: Option<input::HostInput>,
    pub usb: Option<usb::HostUsb>,
    pub bluetooth: Option<bluetooth::HostBluetooth>,
}

impl Service {
    pub(super) fn new(
        devices: parallel::Board,
        restore: Option<restore::Experiment>,
        Peripherals {
            input,
            usb,
            bluetooth,
        }: Peripherals,
        network: Option<network::GuestNetwork>,
        nor: Option<nor::HostNor>,
        power: power::HostPowerControl,
        tsc_hz: u64,
    ) -> Self {
        let now = host::rdtsc();
        Self {
            devices,
            restore,
            input,
            usb,
            bluetooth,
            network,
            nor,
            power,
            tsc_hz,
            log_flushed: now,
            reported: now,
            anchored: None,
            next: 0,
            ran: [now; TURN_PARTS],
            #[cfg(feature = "diagnostics")]
            input_peak: None,
        }
    }

    pub(super) fn power(&self) -> &power::HostPowerControl {
        &self.power
    }

    /// Store what the guest committed since the last service, the caller
    /// waiting: at the stop, before the PC restarts or turns off.
    pub(super) fn stop(&mut self) {
        if let Some(nor) = self.nor.as_mut() {
            let repair = super::bundle_repair::change_pending();
            let (mut dirty, anchor, image) = self.devices.with(|devices| {
                (
                    nor::HostNor::dirty(devices),
                    devices.rtc_anchor(),
                    repair.then(|| devices.nor_image()).flatten(),
                )
            });
            // The automatic plugin repair's record, on top of what the
            // guest committed: the guest no longer writes its NOR.
            if let Some(mut image) = image {
                dirty.extend(super::bundle_repair::at_stop(&mut image));
            }
            nor.store(dirty);
            if let Some(anchor) = anchor {
                nor.keep_anchor(anchor);
            }
            nor.finish();
        }
        #[cfg(feature = "nvk")]
        super::gpu::gate::stop(|| self.devices.serve(|devices| devices.stop_gpu_gates()));
    }

    /// One service: its quick parts first, the restore's copy of what it
    /// read ahead to the guest among them, then the restore's read ahead,
    /// verification and answers and another copy, which a waiting PE leaves
    /// to a later service once an interrupt comes for it (`boot` serves them
    /// whole); whether it ran the parts to the last.
    fn serve(&mut self, boot: bool, step: bool, watch: &GuestWatch) -> bool {
        if let Some(mode) = SHUTDOWN.take(boot) {
            output::arguments(format_args!("POWER native shutdown: {mode:?}\n"));
            match mode {
                Shutdown::Forced => {
                    self.stop();
                    self.power.power_off();
                }
                Shutdown::Restart => {
                    self.stop();
                    self.power.restart();
                }
                Shutdown::Orderly => self
                    .devices
                    .serve(|devices| self.power.request_shutdown(devices)),
            }
        }
        let Self {
            devices,
            restore,
            input,
            usb,
            bluetooth,
            network,
            nor,
            power,
            tsc_hz,
            log_flushed,
            reported,
            anchored,
            next,
            ran,
            #[cfg(feature = "diagnostics")]
            input_peak,
        } = self;
        let mut lap = Lap::new(*tsc_hz, boot, step, watch);
        let now = host::rdtsc();
        // The guest console's input, which its device model takes under
        // the devices' lock and so cannot wait for the host's port there.
        super::feed_console_input();
        let fed = host::rdtsc();
        // A service runs the parts from where the last one stopped on: a
        // step until one ends past its time; any turn, once a part ran,
        // until guest work comes for its PE (an interrupt for it or its own
        // timer), and a waiting PE's until an interrupt comes for its core,
        // as whoever sent it times the answer and the count holds for it
        // only a stall limit (`clock`, `GuestWatch`).
        let from = core::mem::take(next);
        let limit = *tsc_hz / 1_000_000 * STEP_MICROS;
        let deferred = *tsc_hz / 1000 * DEFERRED_MILLIS;
        let mut turn = Turn::new(from, now, step.then_some(limit));
        let guest = || watch.pending(host::fresh_counter()).is_some();
        // Whether `part` runs in this turn; `drivers` when it calls the host
        // drivers, whose lock another core may hold: such a part is left
        // to a later turn without ending this one.
        let mut due = |part: usize, drivers: bool, turn: &mut Turn| {
            if drivers
                && super::storage::host_drivers_taken()
                && now.wrapping_sub(ran[part]) < deferred
            {
                return false;
            }
            if !turn.due(part, host::rdtsc(), !boot && host::interrupted(), guest()) {
                return false;
            }
            ran[part] = now;
            true
        };
        #[cfg(feature = "diagnostics")]
        let mut input_turn = InputTurn {
            start: lap.started,
            boot,
            ..InputTurn::default()
        };
        let inputs_due = due(INPUTS, false, &mut turn);
        if let Some(usb) = usb.as_mut().filter(|_| inputs_due) {
            let arrived = usb.poll();
            if let Some(input) = input.as_mut() {
                input.add(arrived);
            }
            // The part's time holds the console's input before it: the
            // host's port is read outside every lock.
            let console = fed.wrapping_sub(now);
            let (hz, split) = (*tsc_hz, usb.split());
            lap.part_of(Part::Usb, |took| {
                format!(
                    " console={}{}",
                    u128::from(console) * 1_000_000 / u128::from(hz.max(1)),
                    split.line(took.saturating_sub(console), hz),
                )
            });
        }
        if let Some(input) = input.as_mut().filter(|_| inputs_due) {
            #[cfg(feature = "diagnostics")]
            {
                input_turn.usb_done = host::rdtsc();
            }
            let inputs = input.read(
                #[cfg(feature = "diagnostics")]
                &mut input_turn,
            );
            #[cfg(feature = "diagnostics")]
            {
                input_turn.read_done = host::rdtsc();
                input_turn.reports = inputs.len();
            }
            if inputs.iter().any(|input| {
                matches!(
                    input,
                    stela_drivers::input::Input::Move { .. }
                        | stela_drivers::input::Input::Motion { .. }
                )
            }) {
                super::cursor::motion(host::rdtsc());
            }
            if !inputs.is_empty() {
                devices.serve(|devices| {
                    #[cfg(feature = "diagnostics")]
                    {
                        input_turn.delivery_entered = host::rdtsc();
                    }
                    input.deliver_all(devices, inputs);
                });
            }
            lap.part(Part::Input);
            #[cfg(feature = "diagnostics")]
            {
                input_turn.end = lap.started;
                if input_peak
                    .as_ref()
                    .is_none_or(|prior| input_turn.duration() > prior.duration())
                {
                    *input_peak = Some(input_turn);
                }
            }
        }
        if due(LOGS, true, &mut turn) {
            // The logs' disks are looked at without waiting for the host
            // drivers: a store another processor's driver call holds up
            // goes on at a later turn.
            let ((), drivers) = super::storage::without_waiting(*tsc_hz, || {
                if now.wrapping_sub(*log_flushed) >= *tsc_hz {
                    output::flush_due();
                    *log_flushed = host::rdtsc();
                } else {
                    output::advance_logs();
                }
            });
            lap.part_of(Part::Log, |_| drivers);
        }
        if let Some(network) = network.as_mut().filter(|_| due(NETWORK, true, &mut turn)) {
            // The host adapter's sends and receives wait outside the
            // devices' lock, which every PE's device accesses take.
            let (frames, status, room) = devices.with(|devices| {
                let frames = network.queued(devices);
                (frames, devices.network_status(), devices.network_room())
            });
            network.status(status);
            let reusable = network.transmit(frames);
            network.receive(room);
            devices.serve(|devices| {
                if let Some(frame) = reusable {
                    devices.network_recycle(frame);
                }
                network.deliver(devices);
            });
            lap.part(Part::Network);
        }
        if let Some(bluetooth) = bluetooth.as_mut().filter(|_| due(BLUETOOTH, true, &mut turn)) {
            let (packets, room, status) = devices.with(|devices| bluetooth.queued(devices));
            bluetooth.status(status);
            bluetooth.transmit(packets);
            let received = if room {
                bluetooth.receive()
            } else {
                Vec::new()
            };
            if !received.is_empty() {
                devices.serve(|devices| bluetooth::HostBluetooth::deliver(devices, received));
            }
            lap.part(Part::Bluetooth);
        }
        if due(NOR, true, &mut turn) {
            let mut drivers = String::new();
            if let Some(nor) = nor {
                let due =
                    anchored.is_none_or(|at| now.wrapping_sub(at) >= ANCHOR_SECONDS * *tsc_hz);
                let (dirty, anchor) = devices.with(|devices| {
                    let anchor = due.then(|| devices.rtc_anchor()).flatten();
                    (nor::HostNor::dirty(devices), anchor)
                });
                if anchor.is_some() {
                    *anchored = Some(now);
                }
                // As the logs' stores: never waiting for the host drivers.
                let ((), line) = super::storage::without_waiting(*tsc_hz, || {
                    nor.store(dirty);
                    if let Some(anchor) = anchor {
                        nor.keep_anchor(anchor);
                    }
                });
                drivers = line;
            }
            if power.pressed() {
                devices.serve(|devices| power.press(devices));
            }
            lap.part_of(Part::Nor, |_| drivers);
        }
        if let Some(experiment) = restore.as_mut().filter(|_| due(RESTORE, false, &mut turn)) {
            // What was read ahead goes to the guest first: a service is
            // usually due because the guest's window opened, and the read
            // ahead can take milliseconds. The source is then read, and the
            // guest's requests answered, outside the devices' lock, which
            // every PE's device accesses take, and the answers and new
            // chunks copied in turn.
            experiment.pump();
            devices.serve(|devices| experiment.poll(devices));
            lap.part(Part::Restore);
            // The read ahead and its work wait for a later turn once guest
            // work came for this PE, or, on a waiting PE, an interrupt.
            if turn.goes_on(host::rdtsc(), !boot && host::interrupted(), guest()) {
                experiment.prefetch();
                let prefetched = host::rdtsc();
                experiment.work();
                let (hz, work) = (*tsc_hz, host::rdtsc().wrapping_sub(prefetched));
                lap.part_of(Part::Read, |took| {
                    let us = |ticks: u64| u128::from(ticks) * 1_000_000 / u128::from(hz.max(1));
                    format!(
                        " prefetch_us={} work_us={}",
                        us(took.saturating_sub(work)),
                        us(work)
                    )
                });
                devices.serve(|devices| experiment.poll(devices));
                lap.part(Part::Restore);
            }
        }
        if now.wrapping_sub(*reported) >= REPORT_SECONDS * *tsc_hz {
            if let Some(line) = restore
                .as_ref()
                .and_then(|experiment| experiment.asr_report())
            {
                output::arguments(format_args!("{line}\n"));
            }
            if let Some(network) = network.as_ref() {
                network.report();
            }
            power.disks().report(*tsc_hz);
            *reported = now;
        }
        match turn.ended() {
            Some((part, _)) => {
                *next = part;
                false
            }
            None => true,
        }
    }
}

/// The parts of a service, timed for [`report`].
#[derive(Clone, Copy)]
enum Part {
    /// The restore source's read ahead.
    Read,
    Restore,
    /// The USB hosts' topology and attachment poll.
    Usb,
    Input,
    Network,
    Bluetooth,
    /// The NOR's stores and the power button.
    Nor,
    Log,
}

const PARTS: usize = 8;
const NAMES: [&str; PARTS] = [
    "read",
    "restore",
    "usb",
    "input",
    "network",
    "bluetooth",
    "nor",
    "log",
];

/// The longest each part took since the last [`report`], in counter ticks.
static LONGEST: [AtomicU64; PARTS] = [const { AtomicU64::new(0) }; PARTS];
/// The time each part took in all since the last [`report`], in counter ticks.
static TOTAL: [AtomicU64; PARTS] = [const { AtomicU64::new(0) }; PARTS];
/// Services since the last report: by a waiting PE and by the boot PE.
static SERVICES: [AtomicU64; 2] = [const { AtomicU64::new(0) }; 2];

/// The parts that held a PE for [`SLOW_MILLIS`], each reported in a line of
/// its own a few times a boot at most ([`SlowParts`]).
static SLOW: SlowParts<PARTS> = SlowParts::new();

/// How long one part of one service turn may take before it is named in a
/// line of its own: a fifth of what XNU's watchdog allows one exit.
const SLOW_MILLIS: u64 = 100;

/// The parts that kept guest work waiting for their PE past [`LATE_MILLIS`],
/// each reported a few times a boot at most, apart from the slow ones.
static LATE: SlowParts<PARTS> = SlowParts::new();

/// How long guest work (an interrupt or timer for the serving PE) may wait
/// for one part before the part is named although it was not slow: the
/// stall limit after which the shared count holds for that PE (`clock`).
const LATE_MILLIS: u64 = 50;

/// When the running part started, and this core's posted pumps' ticks then
/// (`gpu::posted_ticks_here`), in a turn of the kind [`serve`] was asked for,
/// on the PE `watch` looks after.
struct Lap<'a> {
    started: u64,
    pumped: u64,
    tsc_hz: u64,
    boot: bool,
    step: bool,
    watch: &'a GuestWatch,
}

impl<'a> Lap<'a> {
    fn new(tsc_hz: u64, boot: bool, step: bool, watch: &'a GuestWatch) -> Self {
        Self {
            started: host::rdtsc(),
            pumped: super::gpu::posted_ticks_here(),
            tsc_hz,
            boot,
            step,
            watch,
        }
    }

    fn part(&mut self, part: Part) {
        self.part_of(part, |_| String::new());
    }

    /// The running part ended; `detail` says where its time went, given
    /// the ticks it took, when the part is reported as slow.
    fn part_of(&mut self, part: Part, detail: impl FnOnce(u64) -> String) {
        let now = host::rdtsc();
        // The posted pumps a part ran between its device polls are the
        // pump's time (`GPU PUMP`), not the part's.
        let pumped = super::gpu::posted_ticks_here().wrapping_sub(self.pumped);
        let took = now.wrapping_sub(self.started).saturating_sub(pumped);
        LONGEST[part as usize].fetch_max(took, Ordering::Relaxed);
        TOTAL[part as usize].fetch_add(took, Ordering::Relaxed);
        // The guest work that waited for this PE while the part ran: the
        // part is named when it was slow, or when it kept that work waiting
        // past the stall limit, with what waited and how long.
        let waited = self.watch.waited(host::fresh_counter());
        let slow = SLOW.worth(part as usize, took, self.tsc_hz / 1000 * SLOW_MILLIS);
        let late = late_worth(
            &LATE,
            part as usize,
            waited.and_then(|(_, ticks)| ticks),
            host::COUNTER_HZ / 1000 * LATE_MILLIS,
        );
        if slow || late {
            let guest = match waited {
                None => String::from(" guest=-"),
                Some((pending, None)) => format!(" guest={}", pending.name()),
                Some((pending, Some(ticks))) => format!(
                    " guest={} late_us={}",
                    pending.name(),
                    u128::from(ticks) * 1_000_000 / u128::from(host::COUNTER_HZ)
                ),
            };
            output::arguments(format_args!(
                "HOST SERVICE SLOW part={} us={} boot={} step={} pe={}{guest}{}\n",
                NAMES[part as usize],
                u128::from(took) * 1_000_000 / u128::from(self.tsc_hz.max(1)),
                self.boot,
                self.step,
                self.watch.pe(),
                detail(took),
            ));
        }
        // A posted GPU call waits for the boot CPU, not for the remaining
        // parts; the pump's time is no part's.
        super::gpu::pump_posted();
        *self = Self::new(self.tsc_hz, self.boot, self.step, self.watch);
    }
}

/// A single completed turn, so its phases cannot come from different PEs
/// or different services. The outer service owner serializes access.
#[cfg(feature = "diagnostics")]
#[derive(Default)]
pub(super) struct InputTurn {
    start: u64,
    usb_done: u64,
    pub(super) led_done: u64,
    read_done: u64,
    delivery_entered: u64,
    end: u64,
    pub(super) led_changed: bool,
    pub(super) devices: usize,
    reports: usize,
    boot: bool,
}

#[cfg(feature = "diagnostics")]
impl InputTurn {
    fn duration(&self) -> u64 {
        self.end.wrapping_sub(self.start)
    }

    fn report(&self, tsc_hz: u64) -> String {
        let us = |ticks| u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1));
        let delivery_entry = if self.reports == 0 {
            0
        } else {
            self.delivery_entered.wrapping_sub(self.read_done)
        };
        format!(
            " input_turn_us={} input_turn_boot={} input_usb_us={} \
             input_led_us={} input_led_changed={} input_read_us={} \
             input_devices={} input_delivery_us={} \
             input_delivery_entry_us={} input_reports={}",
            us(self.duration()),
            self.boot,
            us(self.usb_done.wrapping_sub(self.start)),
            us(self.led_done.wrapping_sub(self.usb_done)),
            self.led_changed,
            us(self.read_done.wrapping_sub(self.led_done)),
            self.devices,
            us(self.end.wrapping_sub(self.read_done)),
            us(delivery_entry),
            self.reports,
        )
    }
}

/// The service, which one core at a time holds.
struct Slot {
    held: AtomicBool,
    service: UnsafeCell<Option<Service>>,
}

// SAFETY: only the core that set `held` reaches the service, so the state
// its parts share through `Rc` or `RefCell` is never touched by two cores
// at once; their devices are reached through their controllers' registers
// and locks, from any core.
unsafe impl Sync for Slot {}

static SLOT: Slot = Slot {
    held: AtomicBool::new(false),
    service: UnsafeCell::new(None),
};

/// The counter when the host was last served.
static SERVED: AtomicU64 = AtomicU64::new(0);

/// Run `f` on the service while this core holds it; `None` while another
/// does.
fn hold<R>(f: impl FnOnce(&mut Option<Service>) -> R) -> Option<R> {
    if SLOT
        .held
        .compare_exchange(false, true, Ordering::Acquire, Ordering::Relaxed)
        .is_err()
    {
        return None;
    }
    // SAFETY: `held` gives this core the only reference.
    let result = f(unsafe { &mut *SLOT.service.get() });
    SLOT.held.store(false, Ordering::Release);
    Some(result)
}

/// Give the service its interfaces for the run.
pub(super) fn install(service: Service) {
    let mut service = Some(service);
    while hold(|slot| *slot = service.take()).is_none() {
        core::hint::spin_loop();
    }
}

/// Serve the host unless another core is serving it; `boot` when the boot
/// PE serves it. A `step` serves only for about [`STEP_MICROS`]: its core
/// runs guest code that no other core can, and every PE's time waits for it.
/// The turn ends at its next part once `watch` finds guest work for the
/// serving PE (`Machine::guest_watch`), called within `Machine::serve_host`.
pub(super) fn serve(boot: bool, step: bool, watch: &GuestWatch) {
    // Give the boot processor the next turn when it must perform a native
    // shutdown; an AP may not consume that request or enter firmware for it.
    if !boot && SHUTDOWN.pending() {
        return;
    }
    // An interrupt from now on is one the waiting PE should take instead.
    if !boot {
        host::clear_interrupted();
    }
    let served = hold(|slot| {
        let Some(service) = slot else {
            return false;
        };
        service.serve(boot, step, watch)
    });
    // A step that left parts for the next keeps the host starved, so the
    // next step follows at the next slice.
    if served == Some(true) {
        SERVED.store(host::rdtsc(), Ordering::Release);
        SERVICES[usize::from(boot)].fetch_add(1, Ordering::Relaxed);
    }
}

/// Whether no PE has served the host for [`STARVED_MILLIS`], or for
/// [`REQUESTED_MILLIS`] with a device's request `requested`.
pub(super) fn starved(tsc_hz: u64, requested: bool) -> bool {
    if SHUTDOWN.pending() {
        return true;
    }
    let millis = if requested {
        REQUESTED_MILLIS
    } else {
        STARVED_MILLIS
    };
    host::rdtsc().wrapping_sub(SERVED.load(Ordering::Acquire)) >= tsc_hz / 1000 * millis
}

/// The service, back from whichever core serves it, for the run's end.
pub(super) fn take() -> Option<Service> {
    loop {
        if let Some(service) = hold(Option::take) {
            return service;
        }
        core::hint::spin_loop();
    }
}

/// The services since the last report and the longest and total time each
/// part took.
pub(super) fn report(tsc_hz: u64) -> String {
    let services = SERVICES
        .each_ref()
        .map(|count| count.swap(0, Ordering::Relaxed));
    // Time cores waited for the host drivers, in total and at most, and the
    // longest hold with where it was taken.
    let (waited, longest, held, holder) = super::storage::host_drivers_waited(tsc_hz);
    let holder = holder.map_or(String::from("-"), |site| {
        format!("{}:{}", site.file(), site.line())
    });
    let mut line = format!(
        "services waiting={} boot={} host_drivers_waited_us={waited}/{longest} host_drivers_held_us={held}@{holder} undelivered_doorbells={} longest_us",
        services[0],
        services[1],
        host::undelivered_doorbells(),
    );
    for (name, longest) in NAMES.iter().zip(&LONGEST) {
        let ticks = longest.swap(0, Ordering::Relaxed);
        line.push_str(&format!(
            " {name}={}",
            u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1))
        ));
    }
    line.push_str(" total_us");
    for (name, total) in NAMES.iter().zip(&TOTAL) {
        let ticks = total.swap(0, Ordering::Relaxed);
        line.push_str(&format!(
            " {name}={}",
            u128::from(ticks) * 1_000_000 / u128::from(tsc_hz.max(1))
        ));
    }
    // An in-flight service keeps its sample for the next report. Reporting
    // never waits for the owner or holds it while formatting.
    #[cfg(feature = "diagnostics")]
    if let Some(Some(turn)) = hold(|slot| slot.as_mut()?.input_peak.take()) {
        line.push_str(&turn.report(tsc_hz));
    }
    line
}
