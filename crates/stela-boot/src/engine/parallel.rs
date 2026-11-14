//! Every guest PE on its own host core (the default). The boot processor's
//! machine keeps the boot PE and serves resets and DMA; each other PE runs
//! a machine of its own on the AP its worker binding names, from the first
//! time a device starts it. Whichever PE waits for an interrupt serves the
//! host (`service`). The machines share guest
//! RAM, the board's devices and a hub (`stela_engine::machine::engine::
//! parallel`).
use super::{SLICE, WAIT_POLLS_PER_SECOND, host, output};
use crate::Result;
use crate::{apic::Apic, native_smp::executor::Executors};
use alloc::{boxed::Box, sync::Arc, vec::Vec};
use stela_boot_core::staged_frame::Share;
use stela_contracts::refuse;
use stela_engine::machine::Cpu;
use stela_engine::machine::Host;
use stela_engine::machine::Machine;
use stela_engine::machine::Stop;
use stela_engine::machine::ctrr;
use stela_engine::machine::engine::Kick;
use stela_engine::machine::engine::SharedDevices;
pub(crate) use stela_engine::machine::engine::SourcePhase;
use stela_engine::machine::engine::WorkerProfile;
use stela_engine::machine::engine::publication::WorkerBinding;
pub(crate) use stela_engine::machine::engine::{Counts, RunBudget};
use stela_engine::machine::exception::Pstate;
use stela_soc::machine::AppleDevices;

/// Engine heap bytes per byte of kept block buffers and index/FIFO metadata
/// shared by the PEs (`translations::share`).
pub(crate) const SHARED_CODE_SHARE: usize = 8;

/// The board's devices as every PE's machine reaches them.
pub(crate) type Board = SharedDevices<AppleDevices>;

/// Rings the doorbell of the AP that runs each PE.
pub(crate) struct Doorbells {
    /// APIC IDs by PE.
    destinations: Vec<Option<u32>>,
}

impl Doorbells {
    /// `apic` gives the APIC ID of each worker slot.
    pub(crate) fn new(
        pes: usize,
        bindings: &[WorkerBinding],
        apic: impl Fn(usize) -> Option<u32>,
    ) -> Self {
        let mut destinations = alloc::vec![None; pes];
        for binding in bindings {
            if let Some(destination) = destinations.get_mut(binding.pe) {
                *destination = apic(binding.worker.0);
            }
        }
        Self { destinations }
    }
}

impl Kick for Doorbells {
    /// A store to the core's doorbell when it waits on one with MWAIT
    /// (`native_idle::Doorbell`), else its interrupt. Neither waits for the
    /// target.
    fn kick(&self, pe: usize) {
        if let Some(&Some(destination)) = self.destinations.get(pe)
            && !crate::native_idle::doorbell(destination).is_some_and(|doorbell| doorbell.ring())
        {
            host::ring(destination);
        }
    }

    fn ticks(&self) -> u64 {
        host::fresh_counter()
    }

    fn count(&self) -> Option<u64> {
        Some(host::fresh_counter())
    }

    fn released(&self) -> bool {
        super::storage::released()
    }

    fn release_due(&self) -> bool {
        super::storage::release_due()
    }

    fn release_inline(&self) -> bool {
        super::storage::release_inline()
    }

    fn signal_due(&self) -> bool {
        super::storage::signal_due()
    }

    fn signal_step(&self) -> bool {
        super::storage::signal_step()
    }
}

/// What every PE's machine shares with the home machine's settings.
pub(crate) struct Settings {
    pub traces: Vec<u64>,
    pub watches: Vec<(u64, u64)>,
    /// `--profile`: time each engine phase, as the home machine does.
    pub step_timing: bool,
}

/// Give each AP the PE its worker binding names, to run on a machine of its
/// own once a device starts it. `home` already entered parallel and holds
/// the serial workers.
pub(crate) fn spawn<H: Host>(
    home: &Machine<H, Board>,
    bindings: &[WorkerBinding],
    executors: &Executors,
    sender: &Apic,
    tsc_hz: u64,
    settings: &Settings,
) -> Result<()> {
    let mut profile = None;
    for binding in bindings {
        if binding.pe == home.processor() {
            continue;
        }
        let processor = executors
            .processors()
            .iter()
            .find(|processor| processor.slot() == binding.worker.0)
            .ok_or(refuse!("a PE's worker has no AP"))?;
        let pe = Pe {
            pe: binding.pe,
            devices: home.devices.worker(),
            core_locks: home.cpu.core_locks.clone(),
            profile: profile.get_or_insert_with(|| home.worker_profile()).clone(),
            traces: settings.traces.clone(),
            watches: settings.watches.clone(),
            step_timing: settings.step_timing,
        };
        processor.spawn(Box::new(move |owner| pe.run(owner)), sender, tsc_hz)?;
    }
    output::arguments(format_args!(
        "PARALLEL {} PEs, each on its own host core\n",
        bindings.len()
    ));
    // Every PE's guest time passes while the serial port takes a long
    // report; idle PEs send it instead.
    output::queue_serial();
    Ok(())
}

/// One PE's machine, before its first start.
struct Pe {
    pe: usize,
    devices: Board,
    core_locks: Arc<ctrr::CoreLocks>,
    profile: WorkerProfile,
    traces: Vec<u64>,
    watches: Vec<(u64, u64)>,
    step_timing: bool,
}

impl Pe {
    fn run(self, owner: &mut host::HostOwner) {
        let Self {
            pe,
            devices,
            core_locks,
            profile,
            traces,
            watches,
            step_timing,
        } = self;
        // This core's driver calls keep the engine's TLB entries.
        let _tagged = super::storage::TaggedRoots::enable();
        let hub = Arc::clone(devices.hub());
        let address = {
            let mut host = owner.host();
            hub.first_start(pe, || host.wait_for_interrupt())
        };
        let cpu = Cpu::of(profile.chip(), address, Pstate::reset(2));
        // This core's own allocations, which other PEs' leave alone.
        let own = || crate::ALLOCATOR.own().unwrap_or(0);
        let before = own();
        if pe == 0 {
            host::chain_check::host_branch(&mut owner.host());
        }
        if step_timing {
            host::chain_check::run(pe, &mut owner.host());
        }
        let mut machine = profile
            .start(cpu, owner.host(), devices, pe)
            .unwrap_or_else(|error| output::fatal(error));
        machine.set_step_timing(step_timing);
        if step_timing {
            host::pmu::start(pe);
        }
        let observing = !traces.is_empty() || !watches.is_empty();
        if !traces.is_empty() {
            machine.set_traces(&traces);
        }
        if !watches.is_empty() {
            machine.set_watches(&watches);
        }
        drop((traces, watches));
        output::arguments(format_args!(
            "PARALLEL PE {pe} machine takes {} KiB of heap\n",
            own().saturating_sub(before).max(0) >> 10
        ));
        // Waits return to this loop, which publishes the counts.
        machine.set_wait_limit(Some(host::COUNTER_HZ / WAIT_POLLS_PER_SECOND));
        if let Err(stop) = machine.start_parallel(Arc::clone(&hub), core_locks, address) {
            stopped(pe, stop);
        }
        output::arguments(format_args!(
            "PARALLEL PE {pe} starts at {address:#x} count={}\n",
            if machine.count_reads_natively() {
                "native"
            } else {
                "still"
            }
        ));
        // The serial port's queue goes out from here between slices, so the
        // home PE, which holds the devices and the GPUs' work, does not pay
        // for its own reports.
        output::share_draining();
        #[cfg(feature = "native-profile")]
        let mut stored_report = None;
        // This PE's part of giving the screen its frames while it runs.
        let mut screen = Share::new(super::display::BUSY_SHARE);
        loop {
            let limit = machine.statistics.blocks + SLICE;
            let stop = machine.run_with_budget(RunBudget {
                blocks: limit,
                host_ticks: Some(host::COUNTER_HZ / WAIT_POLLS_PER_SECOND),
            });
            #[cfg(feature = "native-profile")]
            if step_timing {
                let counter = host::fresh_counter();
                if stored_report.is_none_or(|before| {
                    counter.wrapping_sub(before) >= super::run::BEAT_SECONDS * host::COUNTER_HZ
                }) {
                    report_stored_pages(&machine, counter);
                    stored_report = Some(counter);
                }
            }
            // Like the home PE, drain outside guest execution and before a
            // terminal stop. Retention is bounded by TRACE_HITS/WATCH_HITS;
            // output uses the shared bounded serial queue, never an ISR.
            // The host's work between slices runs outside this PE's run: it
            // uses none of its translations, and the serial port and host
            // devices it reaches can take milliseconds (a VM's port I/O
            // waits on the hypervisor), which no other PE's maintenance
            // broadcast waits for. The PE reads its mail before it runs.
            super::run::host_work(&mut machine, |machine| {
                if observing {
                    report_diagnostics(machine);
                }
                // What this PE answered degraded: the home PE reports its
                // own and the devices' coprocessor requests.
                super::reports::unmodeled_report(machine, Vec::new());
            });
            match stop {
                // A PE that waits for an interrupt serves the host meanwhile,
                // gives the screen the frames the DCP model presented and a
                // staged guest disk its host I/O.
                stop if machine.yielded_for_interrupt(&stop) => {
                    super::run::host_work(&mut machine, |machine| {
                        let watch = machine.guest_watch();
                        super::service::serve(false, false, &watch);
                        super::display::flush(super::display::Turn::Idle);
                        if super::storage::serve_staged(true) {
                            machine.devices.call_home();
                        }
                        // The translation pack's writes, a bounded step.
                        super::translations::step();
                        // A GPU compilation a guest call deferred.
                        #[cfg(feature = "nvk")]
                        super::gpu::gate::run_job(super::gpu::gate::Taker::Waiting);
                    })
                }
                // A busy PE gives the screen one bounded step within its
                // share, so that the boot PE need not.
                Stop::Budget | Stop::Waiting => {
                    let now = host::rdtsc();
                    if screen.allows(now) {
                        let spent = super::run::host_work(&mut machine, |_| {
                            super::display::flush(super::display::Turn::Busy)
                        });
                        screen.spent(spent, now.wrapping_add(spent));
                    }
                    // A GPU compilation no waiting PE took for long, with
                    // every guest CPU busy: taken between two slices of user
                    // code, where the guest holds no lock and no kernel
                    // watchdog counts, rather than inside the guest call that
                    // waits for it.
                    #[cfg(feature = "nvk")]
                    if machine.cpu.pstate.el == 0 && super::gpu::gate::busy_job_due() {
                        super::run::host_work(&mut machine, |_| {
                            super::gpu::gate::run_job(super::gpu::gate::Taker::Busy)
                        });
                    }
                }
                stop => stopped(pe, stop),
            }
            hub.publish(pe, machine.counts());
            // Whether the boot processor's Linux step returns.
            super::step_watchdog::look();
            if machine.step_timing().is_some() {
                hub.add_system_exits(pe, &machine.take_system_exits());
                if machine.has_hot_samples() {
                    hub.add_hot_samples(|hot| machine.take_hot_samples(hot));
                }
            }
            super::run::host_work(&mut machine, |_| output::drain_some());
        }
    }
}

/// Owner-local counts sampled after native execution has returned.
#[cfg(feature = "native-profile")]
pub(super) fn report_stored_pages<H: Host>(machine: &Machine<H, Board>, counter: u64) {
    let pe = machine.processor();
    let (pages, budget, peak) = machine.stored_page_occupancy();
    output::arguments(format_args!(
        "PE STORED pe={pe} counter={counter:#x} pages={pages} budget={budget} peak={peak} page_bytes={}\n",
        stela_engine::translations::PAGE_BYTES,
    ));
}

/// The home and secondary PEs' records, including the PE that made each hit.
/// Each machine keeps at most TRACE_HITS and WATCH_HITS between drains;
/// their last records report any overflow rather than hiding lost hits.
pub(super) fn report_diagnostics<H: Host>(machine: &mut Machine<H, Board>) {
    for hit in machine.take_trace_hits() {
        output::arguments(format_args!(
            "TRACE pc={:#x} pe={} el={} x0-x3={:#x} {:#x} {:#x} {:#x} lr={:#x} dropped={}\n",
            hit.pc,
            hit.pe,
            hit.el,
            hit.arguments[0],
            hit.arguments[1],
            hit.arguments[2],
            hit.arguments[3],
            hit.lr,
            hit.dropped
        ));
    }
    for hit in machine.take_watch_hits() {
        output::arguments(format_args!(
            "WATCH pc={:#x} pe={} {} pa={:#x} bytes={} value={:#x}{} dropped={}\n",
            hit.pc,
            hit.pe,
            if hit.write { "write" } else { "read" },
            hit.physical,
            hit.bytes,
            hit.value,
            if hit.completed { "" } else { " refused" },
            hit.dropped
        ));
    }
}

/// Bounded, explicitly stale observations copied by each owning Machine.
/// Neither the report time nor another PE's publication makes them current.
pub(super) fn report_source_observations<H: Host>(home: &Machine<H, Board>) {
    let hub = home.devices.hub();
    if !hub.source_observations_enabled() {
        return;
    }
    output::arguments(format_args!(
        "PE SOURCE last-materialized=true coherent=false\n"
    ));
    for pe in 0..hub.pes() {
        match hub.source_observation(pe) {
            Some(observed) => {
                let source = observed.source;
                output::arguments(format_args!(
                    "PE SOURCE pe={pe} sequence={} observed_host_counter={:#x} phase={:?} pc={:#x} el={} daif={:#x} sp={:#x} fp={:#x} lr={:#x}\n",
                    observed.sequence,
                    source.host_counter,
                    source.phase,
                    source.pc,
                    source.el,
                    source.daif,
                    source.sp,
                    source.fp,
                    source.lr
                ));
            }
            None => output::arguments(format_args!(
                "PE SOURCE pe={pe} unavailable=no-stable-owner-observation\n"
            )),
        }
    }
    output::arguments(format_args!(
        "PE BROADCAST retained=true coherent-across-pes=false counter_hz={} fields=started/completed/recipients/elapsed/max/polls\n",
        host::COUNTER_HZ
    ));
    for pe in 0..hub.pes() {
        match hub.broadcast_profile(pe) {
            Some(profile) => {
                let [tlb, code, power, completion] = profile.kinds;
                output::arguments(format_args!(
                    "PE BROADCAST pe={pe} tlb={}/{}/{}/{}/{}/{} code={}/{}/{}/{}/{}/{} power={}/{}/{}/{}/{}/{} completion={}/{}/{}/{}/{}/{} active={:?} active_start={:#x} dropped={}\n",
                    tlb.started,
                    tlb.completed,
                    tlb.recipients,
                    tlb.elapsed_ticks,
                    tlb.maximum_ticks,
                    tlb.polls,
                    code.started,
                    code.completed,
                    code.recipients,
                    code.elapsed_ticks,
                    code.maximum_ticks,
                    code.polls,
                    power.started,
                    power.completed,
                    power.recipients,
                    power.elapsed_ticks,
                    power.maximum_ticks,
                    power.polls,
                    completion.started,
                    completion.completed,
                    completion.recipients,
                    completion.elapsed_ticks,
                    completion.maximum_ticks,
                    completion.polls,
                    profile.active,
                    profile.active_start,
                    profile.dropped_updates
                ));
            }
            None => output::arguments(format_args!(
                "PE BROADCAST pe={pe} unavailable=no-stable-observation\n"
            )),
        }
    }
}

/// A PE's machine stopped, which only the home machine may: the board stops.
fn stopped(pe: usize, stop: Stop) -> ! {
    output::fatal(format_args!(
        "PE {pe} on its own host core stopped: {stop:?}"
    ))
}

/// Each PE's counts: the home machine's own, and those every other PE's
/// machine last published; one when the PEs run serialized.
pub(crate) fn counts<H: Host>(home: &Machine<H, Board>) -> Vec<Counts> {
    let own = home.counts();
    if !home.runs_parallel() {
        return alloc::vec![own];
    }
    let hub = home.devices.hub();
    (0..hub.pes())
        .map(|pe| {
            if pe == home.processor() {
                own
            } else {
                hub.published(pe)
            }
        })
        .collect()
}

/// The hot-block samples of every PE since the last call, under step
/// timing: the home machine's own and those the others added
/// (`Hub::add_hot_samples`).
pub(crate) fn hot_samples<H: Host>(
    home: &mut Machine<H, Board>,
) -> stela_engine::machine::engine::step_timing::HotBlocks {
    let mut hot = if home.runs_parallel() {
        home.devices.hub().take_hot_samples()
    } else {
        Default::default()
    };
    home.take_hot_samples(&mut hot);
    hot
}

/// The System exits of every PE since the last call, under step timing:
/// the home machine's own and those the others added
/// (`Hub::add_system_exits`).
pub(crate) fn system_exits<H: Host>(
    home: &mut Machine<H, Board>,
) -> stela_engine::machine::engine::step_timing::SystemExits {
    let mut exits = home.take_system_exits();
    if home.runs_parallel() {
        let hub = home.devices.hub();
        for pe in (0..hub.pes()).filter(|&pe| pe != home.processor()) {
            exits.merge(&hub.take_system_exits(pe));
        }
    }
    exits
}
