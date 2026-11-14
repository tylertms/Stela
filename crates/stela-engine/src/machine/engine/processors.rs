//! The board's PEs. One runs on [`Machine::cpu`]; each other one is off,
//! preempted at the end of its time slice, or waiting in WFI or WFE. The
//! scheduler gives each PE that can run a time slice in turn, of at most
//! [`QUANTUM`] blocks and [`QUANTUM_HZ`]⁻¹ seconds of the system counter, and
//! waits on the host when none can.
//!
//! The PEs share the system counter but not the host: a PE preempted while
//! it holds a spinlock keeps it until its next turn, while the others spin
//! against the kernel's timeout. XNU 26A434 panics once a ticket spinlock
//! stays held for 3,000,000 counts (125 ms at 24 MHz, `lock_ticket.c`), so
//! a slice bounded in counter time keeps the wait for a preempted holder,
//! at most one slice of every other PE, short of it on every board, and the
//! counter itself advances no faster than the PEs progress (`clock`).
//!
//! Native code writes guest RAM directly, so the engine cannot see the store
//! by one PE that clears another PE's global Exclusives monitor (Arm
//! `ClearExclusiveByAddress`) as it happens; `Machine::observe_writes`
//! finds it between runs, and a PE that resumes after another ran has lost
//! its reservation.
//!
//! A WFI in CYC_OVRD's power-down mode turns its core off: the core loses
//! its state and comes back only by leaving reset at its reset vector
//! (m1n1 `deep_wfi`, `src/utils_asm.S`, which saves its context first for
//! that reason). XNU 26A434 powers its cores down this way for system
//! sleep (`_ml_arm_sleep`, file 0x4d6f404) and to take a core offline
//! (file 0x4d6ea84), both through `arm64_prepare_for_sleep` (file
//! 0x4b99170). Its argument 0 first sets CYC_OVRD's IRQ and FIQ modes to
//! disable, and the core then leaves reset only when a device starts it
//! again; sleep passes 1, which keeps them. An interrupt line the core's
//! CYC_OVRD still enables wakes it: the board's device lines, since the
//! core's own timers, performance monitor and vGIC went down with it. Its
//! reset vector is the board's (`Devices::reset_vector`), and the board
//! learns of each power-down and power-up (`Devices::processor_power`).
pub mod publication;
mod retained;
mod transaction;
use super::{
    Devices, Host, Machine, RequirementKind, Start, Step, Stop, Wait, WaitKind, interrupt_pending,
    system, timer,
};
use crate::machine::{
    cpu::{Cpu, RESERVATION_GRANULE},
    ctrr,
    exception::Pstate,
    mmu,
    registers::CpuProfile,
    registers::Lines,
};
use alloc::{boxed::Box, vec::Vec};
pub use retained::{
    PreparedProcessorPower, ProcessorBinding, ProcessorChange, ProcessorPowerError, Processors,
};
#[cfg(test)]
use stela_contracts::processor::ProcessorTarget;
use transaction::NativeProcessorState;
pub use transaction::{ProcessorWrite, SettledProcessorWrite};

/// Blocks a PE runs before the next PE that can run gets its turn.
pub(super) const QUANTUM: u64 = 1 << 14;
/// The inverse of the longest time slice in seconds of the system counter:
/// 2 ms, so the seven other PEs of an eight-PE board hold a preempted
/// spinlock holder off for at most 14 ms.
pub(super) const QUANTUM_HZ: u64 = 500;

/// The inverse of the longest a waiting PE polls before it halts: 200 us,
/// Linux's `guest_halt_poll_ns`.
pub(super) const IDLE_POLL_HZ: u64 = 5000;
/// XNU xnu-12377.121.6 machine_routines.c:108,1251 groups the original
/// DeviceTree `reg` by Aff1 (proc_reg.h:1059-1061). Aff2 is a core type,
/// not part of the physical cluster, including the M6's mixed P/M cluster.
pub(super) const PHYSICAL_CLUSTER_MASK: u64 = 0xff00;

pub(super) enum Peer {
    /// The PE on [`Machine::cpu`].
    Running,
    /// Held in reset until a device starts it.
    Off,
    Parked(Box<Parked>),
    /// Run by another machine on its own host core (`parallel`).
    Remote,
}

pub(super) struct Parked {
    cpu: Cpu,
    event_sample: Option<u64>,
    waiting: Option<Wait>,
    /// Its mirrored-read cells (`MIRRORED`) as it left them: their values
    /// follow from its own System registers and PSTATE, which no other PE
    /// changes, so it takes them back when it runs again.
    mirrors: super::Mirrors,
}

/// The selected existing PE runs; the others are off. A profile without
/// processors has one PE. The constructor validates the selected index.
pub(super) fn boot(profile: &CpuProfile, boot_processor: usize) -> Vec<Peer> {
    let mut peers: Vec<Peer> = (0..profile.processors.len().max(1))
        .map(|_| Peer::Off)
        .collect();
    peers[boot_processor] = Peer::Running;
    peers
}

impl Parked {
    pub(super) fn native_ready(&self) -> bool {
        self.waiting.is_none()
    }
    pub(super) fn cpu(&self) -> &Cpu {
        &self.cpu
    }
    /// Its state, to change: its mirrored-read cells close.
    pub(super) fn cpu_mut(&mut self) -> &mut Cpu {
        self.mirrors = super::Mirrors::default();
        &mut self.cpu
    }
}

/// Whether a PE waiting in `waiting` is powered down.
fn down(waiting: Option<Wait>) -> bool {
    waiting.is_some_and(|wait| wait.down)
}

/// A PE as a report shows it.
#[derive(Debug, Clone, Copy)]
pub enum PeerState<'a> {
    Running,
    Off,
    /// Powered down by a WFI in CYC_OVRD's power-down mode; its state is
    /// lost.
    Down,
    /// Out of reset but not running, including a clock/quiesce
    /// pause: its state, and whether it waits in WFI (`Some(true)`) or WFE
    /// (`Some(false)`).
    Parked {
        cpu: &'a Cpu,
        wfi: Option<bool>,
    },
    /// Running on its own machine ([`super::parallel`]).
    Remote,
}

impl<H: Host, D: Devices> Machine<H, D> {
    /// The progress a serialized clock paces this PE by.
    pub(super) fn work(&self) -> u64 {
        self.statistics.blocks
    }

    /// Checked kept participants, separate from the live/parked Cpu
    /// ownership slots. An unbound inventory grants no PMGR support credit.
    pub fn processors(&self) -> &Processors {
        &self.processor_power
    }

    /// Bind positively identified owners with an explicit handoff state.
    /// Every binding is checked before any participant or CPU changes.
    pub fn bind_processor_power(
        &mut self,
        bindings: &[ProcessorBinding],
    ) -> Result<(), ProcessorPowerError> {
        check_owners(&self.processor_power, self.current, &self.peers)?;
        let plan = self.processor_power.prepare_bindings(bindings)?;
        self.settle_processor_power(plan)
    }

    pub fn prepare_processor_power(
        &self,
        changes: &[ProcessorChange],
    ) -> Result<PreparedProcessorPower, ProcessorPowerError> {
        check_owners(&self.processor_power, self.current, &self.peers)?;
        self.processor_power.prepare(changes)
    }

    /// Commit at one instruction boundary, before a controller acknowledges
    /// its request. Stale/replaced plans refuse before any CPU or clock effect.
    /// No production PMGR resolver/bridge is enabled by this independent API.
    pub fn settle_processor_power(
        &mut self,
        plan: PreparedProcessorPower,
    ) -> Result<(), ProcessorPowerError> {
        self.processor_write()
            .settle(plan)
            .map(|_settled: SettledProcessorWrite<'_>| ())
    }

    /// Borrow the scheduler and native-dispatch owners for one complete
    /// synchronous transaction. Its consuming apply cannot queue work for later.
    pub fn processor_write(&mut self) -> ProcessorWrite<'_> {
        self.processor_write_parts().1
    }

    /// Split the device receiver from the execution owners without a
    /// move, copied snapshot, unsafe alias or a borrow of the complete Machine.
    pub(super) fn processor_write_parts(&mut self) -> (&mut D, ProcessorWrite<'_>) {
        let processors = ProcessorWrite {
            processors: &mut self.processor_power,
            publication: &mut self.native_publication,
            current: self.current,
            cpu: &mut self.cpu,
            peers: &mut self.peers,
            waiting: self.waiting,
            clock: &mut self.clock,
            slice_end: &mut self.slice_end,
            native: NativeProcessorState {
                host: &mut self.host,
                statistics: &mut self.statistics,
                translation_generation: &mut self.translation_generation,
                timers: &mut self.timers,
                floating_point: &mut self.floating_point,
                memory_checks: &mut self.memory_checks,
                native_system: &mut self.native_system,
                debug: &mut self.debug,
                pointer: &mut self.pointer,
                mirrors: &mut self.mirrors,
                device_lines: &mut self.device_lines,
                watch: &mut self.watch,
                tag_checks: &mut self.tag_checks,
                prediction_context: &mut self.prediction_context,
            },
            forwarded: Some(&mut self.forwarded_power),
        };
        (&mut self.devices, processors)
    }

    pub(super) fn processor_paused(&self, pe: usize) -> bool {
        self.processor_power
            .paused(pe)
            .expect("one retained slot per PE")
    }

    /// The number of the running PE.
    pub fn processor(&self) -> usize {
        self.current
    }

    /// Completed guest instructions across kept PE incarnations, including
    /// stopped-service terminals and powered-down CPUs. No allocation or PE
    /// switching occurs. Each CPU's internal u64 retirement counter must not
    /// wrap within one incarnation; this is a performance counter, not PMU.
    pub fn retired_instructions(&self) -> u128 {
        self.peers
            .iter()
            .fold(self.statistics.replaced_retired, |total, peer| {
                let retired = match peer {
                    Peer::Running => self.cpu.retired,
                    Peer::Parked(parked) => parked.cpu.retired,
                    Peer::Off | Peer::Remote => 0,
                };
                total.saturating_add(u128::from(retired))
            })
    }

    /// Every PE's state, in PE order.
    pub fn peer_states(&self) -> Vec<PeerState<'_>> {
        self.peers
            .iter()
            .map(|peer| match peer {
                Peer::Running if down(self.waiting) => PeerState::Down,
                Peer::Running if self.processor_paused(self.current) => PeerState::Parked {
                    cpu: &self.cpu,
                    wfi: self.waiting.map(|wait| wait.kind == WaitKind::Interrupt),
                },
                Peer::Running => PeerState::Running,
                Peer::Off => PeerState::Off,
                Peer::Remote => PeerState::Remote,
                Peer::Parked(parked) if down(parked.waiting) => PeerState::Down,
                Peer::Parked(parked) => PeerState::Parked {
                    cpu: &parked.cpu,
                    wfi: parked.waiting.map(|wait| wait.kind == WaitKind::Interrupt),
                },
            })
            .collect()
    }

    /// The execution PC a board debug sampler observes for a powered PE.
    pub(super) fn processor_pc(&self, cpu: usize) -> Option<u64> {
        match self.peers.get(cpu)? {
            Peer::Running if !down(self.waiting) => Some(self.cpu.pc),
            Peer::Parked(parked) if !down(parked.waiting) => Some(parked.cpu.pc),
            _ => None,
        }
    }

    /// Release the PEs devices started, then keep the running PE until its
    /// time slice ends or it waits. Otherwise run the next PE that can, round
    /// robin from the running one, or wait on the host until one can (or,
    /// with idle skip, leap the counter to the next deadline); with nothing
    /// left to wake any PE the machine is idle. A wait that reaches the
    /// run's wait limit, follows a device's bytes for the host, or that only
    /// the host's inputs can end, returns to the host.
    pub(super) fn schedule(&mut self) -> Step {
        self.schedule_with_deadline(None)
    }

    /// Whether the host called for this PE's core (`Hub::call_host`).
    fn take_host_call(&self) -> bool {
        self.parallel
            .as_ref()
            .is_some_and(|hub| hub.take_host_call(self.current))
    }

    pub(super) fn schedule_with_deadline(
        &mut self,
        run_deadline: Option<super::budget::Deadline>,
    ) -> Step {
        self.mark_progress();
        if self.parallel.is_some() {
            self.forward_power()?;
            self.read_mail()?;
        }
        self.take_starts()?;
        if self.take_host_call() {
            return Err(Stop::Waiting);
        }
        if !self.processor_paused(self.current)
            && self.waiting.is_none()
            && self.statistics.blocks < self.slice_end
            && (self.peers.len() < 2 || self.host.counter() < self.slice_deadline)
        {
            return Ok(());
        }
        let work = self.work();
        self.clock.end_slice(|| self.host.counter(), work);
        let paused = self.processor_paused(self.current);
        if paused || self.waiting.is_some() {
            // Its guest waits: no stall of its holds the count (`clock`).
            let phases = self.clock_phases();
            let retired = self.cpu.retired;
            self.clock.idle(|| self.host.counter(), retired, phases);
        }
        if paused || self.waiting.is_some() && !self.polls_monitor() {
            // Waiting or paused, the PE has time to spare, so its timers
            // signal as soon as they are due. One polling a monitor spins on
            // a lock and stays busy.
            self.clock.wait(|| self.host.counter());
        }
        if let Some(step) = self.short_event_wait(run_deadline) {
            return step;
        }
        let mut armed = false;
        loop {
            let pass = self.host.counter();
            if run_deadline.is_some_and(|deadline| deadline.remaining(pass).is_none()) {
                if armed {
                    self.host.arm_timer(None);
                }
                return Err(Stop::Budget);
            }
            if self.parallel.is_some() {
                self.read_mail()?;
            }
            if self.take_host_call() {
                if armed {
                    self.host.arm_timer(None);
                }
                return Err(Stop::Waiting);
            }
            self.observe_writes();
            // The PEs that run may all be stopped, with nobody else to see.
            self.clock.look(|| self.host.counter());
            let now = self.now();
            self.poll_devices(now)?;
            // A device can start a PE while every PE waits; the start reads
            // the count, so the rest of the pass reads it after.
            self.take_starts()?;
            let now = self.now();
            let (current, count) = (self.current, self.peers.len());
            if let Some(next) = (1..=count)
                .map(|offset| (current + offset) % count)
                .find(|&pe| self.ready(pe, now))
            {
                if armed {
                    self.host.arm_timer(None);
                }
                if next != self.current {
                    self.switch_to(next);
                }
                self.slice_end = self.statistics.blocks + QUANTUM;
                self.slice_deadline =
                    self.host.counter() + self.profile.counter_frequency / QUANTUM_HZ;
                let share = self.runnable();
                let work = self.work();
                self.clock.start_slice(|| self.host.counter(), work, share);
                let (retired, phases) = (self.cpu.retired, self.clock_phases());
                self.clock.busy(|| self.host.counter(), retired, phases);
                return self.wake();
            }
            // In parallel, only a PE that waits for an interrupt serves the
            // host: one in WFE may hold a ticket another PE hands it. A host
            // interrupt that ended the last halt may have brought host work
            // (`Host::host_work_due`), which the halt would otherwise sleep
            // through until its own deadline.
            if (self.parallel.is_none() || self.waits_for_interrupt())
                && (self.devices.take_host_service() || self.host.host_work_due())
            {
                if armed {
                    self.host.arm_timer(None);
                }
                return Err(Stop::Waiting);
            }
            let deadline = self.next_deadline(now);
            // Another parallel PE can send an event or start this PE. A
            // single-PE hub has the same wake sources as a serialized board.
            let remote_pe = self.parallel.as_ref().is_some_and(|hub| hub.pes() > 1);
            if deadline.is_none() && !remote_pe && !self.devices.waits_on_host() {
                return Err(Stop::Idle);
            }
            if let (Some(deadline), true) = (deadline, self.idle_skip) {
                self.statistics.waits += 1;
                let host = self.host.counter();
                self.clock.skip_to(host, deadline);
                continue;
            }
            let limit = self
                .wait_limit
                .map(|limit| pass.saturating_add(limit.saturating_sub(self.waited)));
            let deadline = deadline.map(|deadline| self.clock.wake_at(deadline, pass));
            let until = match (deadline, limit) {
                (_, Some(limit)) if pass >= limit => None,
                (Some(deadline), Some(limit)) => Some(deadline.min(limit)),
                (deadline, limit) => deadline.or(limit),
            };
            let Some(until) = until else {
                if armed {
                    self.host.arm_timer(None);
                }
                return Err(Stop::Waiting);
            };
            if self.polls_monitor() {
                // A safe point: the host may be waiting for this PE.
                self.host.relax();
                // The whole pass is the poll's.
                self.statistics.spin_ticks += self.host.counter().wrapping_sub(pass);
            } else {
                self.statistics.waits += 1;
                let until = match run_deadline {
                    Some(deadline) => {
                        let Some(until) = deadline.timer(self.host.counter(), until) else {
                            if armed {
                                self.host.arm_timer(None);
                            }
                            return Err(Stop::Budget);
                        };
                        until
                    }
                    None => until,
                };
                let slept = self.host.counter();
                if !self.poll_for_wake(slept, until) {
                    let (blocked, halted) = self.outside_run(|machine| {
                        let halt = machine.host.counter();
                        machine.host.arm_timer(Some(until));
                        machine.sleep();
                        let woke = machine.host.counter();
                        (woke.wrapping_sub(slept), woke.wrapping_sub(halt))
                    });
                    self.statistics.halt_ticks += halted;
                    armed = true;
                    self.tune_idle_poll(blocked);
                }
                self.statistics.sleep_ticks += self.host.counter().wrapping_sub(slept);
            }
            self.waited = self
                .waited
                .saturating_add(self.host.counter().saturating_sub(pass));
        }
    }

    /// Whether the running PE waits for an interrupt, or is powered down,
    /// so holds no lock another PE spins on.
    pub fn waits_for_interrupt(&self) -> bool {
        self.waiting
            .is_some_and(|wait| wait.kind == WaitKind::Interrupt || wait.down)
    }

    /// A scheduling yield that leaves this PE waiting for an interrupt,
    /// including a host-time budget that expires during the wait.
    pub fn yielded_for_interrupt(&self, stop: &Stop) -> bool {
        matches!(stop, Stop::Budget | Stop::Waiting) && self.waits_for_interrupt()
    }

    pub(super) fn take_starts(&mut self) -> Step {
        while let Some(start) = self.devices.take_start() {
            self.start(start)?;
        }
        Ok(())
    }

    /// The PEs that share the round with the running one, itself included:
    /// those out of reset and not waiting for an interrupt. A PE in WFE
    /// spins, so it shares the round.
    pub(super) fn runnable(&self) -> u64 {
        let parked = self.peers.iter().enumerate().filter(|(pe, peer)| {
            if self.processor_paused(*pe) {
                return false;
            }
            matches!(peer, Peer::Parked(parked)
                if parked.waiting.is_none_or(|wait| wait.kind != WaitKind::Interrupt))
        });
        u64::from(!self.processor_paused(self.current)) + parked.count() as u64
    }

    /// Parked PE `pe`.
    fn parked(&self, pe: usize) -> &Parked {
        let Peer::Parked(parked) = &self.peers[pe] else {
            unreachable!("PE {pe} is parked");
        };
        parked
    }

    /// Whether PE `pe` can run: it is running or preempted, or a WFI it waits
    /// in sees an interrupt pending (Arm `AArch64_InterruptPending`, masked or
    /// not), or a WFE sees that or its event register set, including by an
    /// event stream edge, or the local timeout of a WFIT or WFET expired. A
    /// powered-down PE wakes when a device line its CYC_OVRD enables
    /// asserts.
    pub(super) fn ready(&mut self, pe: usize, now: u64) -> bool {
        if self.processor_paused(pe) {
            return false;
        }
        if pe == self.current {
            return match self.waiting {
                None => true,
                Some(Wait { down: true, .. }) => {
                    let device = self.device_lines_of(pe, now);
                    self.cpu.wakes_for(device)
                }
                Some(Wait { kind, timeout, .. }) => {
                    if kind == WaitKind::Event {
                        self.sample_events(now);
                    }
                    (kind == WaitKind::Event && self.cpu.event_is_set())
                        || timeout.is_some_and(|timeout| timeout.expired(now))
                        || self.interrupt_pending_at(now)
                }
            };
        }
        let Peer::Parked(parked) = &mut self.peers[pe] else {
            return false;
        };
        let Some(Wait {
            kind,
            timeout,
            down,
        }) = parked.waiting
        else {
            return true;
        };
        if down {
            let device = self.device_lines_of(pe, now);
            return self.parked(pe).cpu.wakes_for(device);
        }
        if timeout.is_some_and(|timeout| timeout.expired(now)) {
            return true;
        }
        if kind == WaitKind::Event {
            if let Some(previous) = parked.event_sample
                && timer::event(&parked.cpu, previous, now)
            {
                parked.cpu.signal_event();
            }
            parked.event_sample = Some(now);
            if parked.cpu.event_is_set() {
                return true;
            }
        }
        let device = self.device_lines_of(pe, now);
        let parked = self.parked(pe);
        let timer = timer::deadline(&parked.cpu, || now).is_some_and(|deadline| now >= deadline)
            || super::monitor_asserted(&parked.cpu, now);
        let lines = parked.cpu.core_lines(Lines {
            irq: device.irq,
            fiq: device.fiq || timer,
        });
        interrupt_pending(&parked.cpu, lines)
    }

    /// Complete the wait the running PE woke from. A WFE or WFET that no
    /// interrupt woke consumes the event register (`Machine::wait`). A PE
    /// that woke from power-down leaves reset instead.
    pub(super) fn wake(&mut self) -> Step {
        if self.waiting.is_some_and(|waiting| waiting.down) {
            let pe = self.current;
            let Some(address) = self.devices.reset_vector(pe) else {
                return Err(self.unsupported(RequirementKind::Native(
                    "a PE woke from CYC_OVRD's power-down mode, but the board holds no reset vector for it",
                )));
            };
            return self.restart(pe, address);
        }
        let Some(Wait { kind, .. }) = self.waiting.take() else {
            return Ok(());
        };
        self.sync_monitor();
        if kind == WaitKind::Event && !self.interrupt_pending() {
            self.cpu.consume_event();
        }
        self.next()
    }

    /// The earliest counter value at which a waiting PE's timer fires, its
    /// event stream sets its event register, its local timeout expires, or a
    /// device changes state. A powered-down PE has none of its own.
    pub(super) fn next_deadline(&mut self, now: u64) -> Option<u64> {
        let streams = |cpu: &Cpu, waiting: Option<Wait>| {
            (waiting.map(|wait| wait.kind) == Some(WaitKind::Event))
                .then(|| timer::next_event(cpu, now))
                .flatten()
        };
        let timeout = |waiting: Option<Wait>| {
            waiting.and_then(|wait| wait.timeout).map(|timeout| {
                if timeout.expired(now) {
                    now
                } else {
                    timeout.deadline(now)
                }
            })
        };
        let mut earliest = None;
        let mut consider = |deadline| {
            if let Some(deadline) = deadline {
                earliest = Some(earliest.map_or(deadline, |prior: u64| prior.min(deadline)));
            }
        };
        if !self.processor_paused(self.current) && !down(self.waiting) {
            consider(self.timer_deadline());
            consider(self.cpu.pmu.as_ref().and_then(|monitor| monitor.deadline()));
            consider(streams(&self.cpu, self.waiting));
            consider(timeout(self.waiting));
        }
        for (pe, peer) in self.peers.iter().enumerate() {
            if let Peer::Parked(parked) = peer
                && !self.processor_paused(pe)
                && !down(parked.waiting)
            {
                consider(timer::deadline(&parked.cpu, || now));
                consider(
                    parked
                        .cpu
                        .pmu
                        .as_ref()
                        .and_then(|monitor| monitor.deadline()),
                );
                consider(streams(&parked.cpu, parked.waiting));
                consider(timeout(parked.waiting));
            }
        }
        consider(self.devices_deadline(now));
        consider(self.devices.dma_deadline());
        earliest
    }

    /// The devices' next deadline at `now`. The one the running PE's cached
    /// lines came with still holds while no other machine's call reached
    /// the devices since ([`Devices::generation`]): their advance with time
    /// gave it, and nothing changes them before it but such a call. That
    /// saves the home PE a second advance of every device in each wait pass.
    fn devices_deadline(&mut self, now: u64) -> Option<u64> {
        if let (Some((_, until, pe)), Some(expected)) = (self.device_lines, self.device_generation)
            && pe == self.current
            && now < until
            && self.devices.generation() == Some(expected)
        {
            return (until != u64::MAX).then_some(until);
        }
        self.devices.deadline(now)
    }

    /// Run PE `next` in place of the running one, which parks with its state.
    pub(super) fn switch_to(&mut self, next: usize) {
        let Peer::Parked(incoming) = core::mem::replace(&mut self.peers[next], Peer::Running)
        else {
            unreachable!("only a parked PE is ready to run");
        };
        let incoming = *incoming;
        let inputs = [&incoming.cpu, &self.cpu].map(Cpu::translation_inputs);
        let upper = inputs.map(|(controls, hcr)| {
            let controls = controls.map(|controls| mmu::Controls {
                ttbr0: 0,
                ..controls
            });
            (controls, hcr)
        });
        let outgoing = Parked {
            cpu: core::mem::replace(&mut self.cpu, incoming.cpu),
            event_sample: core::mem::replace(&mut self.event_sample, incoming.event_sample),
            waiting: core::mem::replace(&mut self.waiting, incoming.waiting),
            mirrors: *self.mirrors,
        };
        self.peers[self.current] = Peer::Parked(Box::new(outgoing));
        self.current = next;
        self.statistics.switches += 1;
        // The switch changes no device: what derives from the PE goes, the
        // devices' lines and deadline stay, and the incoming PE's mirrored
        // reads reopen as it left them.
        let device_lines = self.device_lines;
        // Another PE ran since the incoming one's exclusive load and may
        // have written its granule, even back to the same bytes: its
        // store-exclusive fails, as Arm permits of any (`observe_writes`).
        self.cpu.monitor = None;
        self.cpu.state.exclusive_bytes = 0;
        // The fetch and jump caches and the host's shadow tables follow the
        // incoming PE's translation regime. One that translates alike keeps
        // them: the tables and TLBIs are shared, and what derives from its
        // other System registers is recomputed. One whose TTBR0s alone
        // differ changes the lower half's translations, as a TTBR0 write,
        // and what derives from the other System registers, as they are
        // another PE's.
        if inputs[0] == inputs[1] {
            self.sync_regime();
        } else if upper[0] == upper[1] {
            self.derived_changed();
            self.table_base_changed();
        } else {
            self.controls_changed();
        }
        self.device_lines = device_lines;
        *self.mirrors = incoming.mirrors;
        self.mirrors.transition(self.mirror_state());
    }

    /// Clear the Exclusives monitor of every PE whose reserved granule
    /// another observer has written since its exclusive load, which
    /// generates an event: the WFE wake-up event of a cleared global monitor
    /// (Arm `AArch64_ExclusiveMonitorsStatus`, `ClearExclusiveByAddress`).
    /// One PE runs at a time and only between runs can another PE's (or a
    /// device's) write reach a reservation, so it is observed then, as the
    /// granule's bytes changing. A write that leaves them as they were
    /// generates no event: a WFE that waits for the value to change sees
    /// none. It cannot let a store-exclusive pass, because a PE that
    /// resumes after another ran has lost its reservation (`switch_to`).
    fn observe_writes(&mut self) {
        let parked = self.peers.iter_mut().filter_map(|peer| match peer {
            Peer::Parked(parked) => Some(&mut parked.cpu),
            Peer::Running | Peer::Off | Peer::Remote => None,
        });
        for cpu in core::iter::once(&mut self.cpu).chain(parked) {
            let Some(reservation) = cpu.monitor else {
                continue;
            };
            let mut bytes = [0; RESERVATION_GRANULE as usize];
            if !self.host.read_ram(reservation.granule, &mut bytes) || bytes != reservation.bytes {
                cpu.monitor = None;
                cpu.signal_event();
            }
        }
    }

    /// SEV signals an event to every PE and SEVL to this one (Arm
    /// `SendEvent`, `SendEventLocal`). A PE held in reset or powered down
    /// takes none.
    pub(super) fn send_event(&mut self, local: bool) {
        self.cpu.signal_event();
        if local {
            return;
        }
        if let Some(hub) = &self.parallel {
            hub.event(self.current);
        }
        for peer in &mut self.peers {
            if let Peer::Parked(parked) = peer
                && !down(parked.waiting)
            {
                parked.cpu.signal_event();
            }
        }
    }

    /// The running PE's WFI in CYC_OVRD's power-down mode turned its core
    /// off: its reservation goes with its state, and the board learns of it.
    pub(super) fn power_down(&mut self) {
        self.cpu.monitor = None;
        self.cpu.state.exclusive_bytes = 0;
        self.device_lines = None;
        self.devices.processor_power(self.current, false);
    }

    /// Release PE `start.cpu` from reset at `start.address` in EL2h, the
    /// highest implemented EL, with the System registers' reset values and
    /// its own identification (Arm `TakeReset`). A powered-down PE can be
    /// started like one held in reset.
    pub(super) fn start(&mut self, start: Start) -> Step {
        let held = match self.peers.get(start.cpu) {
            Some(Peer::Remote) => {
                let hub = self
                    .parallel
                    .as_ref()
                    .expect("a remote PE runs in parallel");
                hub.start(start.cpu, start.address);
                return Ok(());
            }
            Some(Peer::Off) => true,
            Some(Peer::Running) => down(self.waiting),
            Some(Peer::Parked(parked)) => down(parked.waiting),
            None => {
                return Err(self.unsupported(RequirementKind::Native(
                    "a device started a PE the CPU profile does not have",
                )));
            }
        };
        if !held {
            return Err(self.unsupported(RequirementKind::Native(
                "a device started a PE that is neither held in reset nor powered down",
            )));
        }
        self.restart(start.cpu, start.address)
    }

    /// The running PE's write of RMR_EL2.RR requested a Warm reset (Arm
    /// RMR_EL2), which it takes as the write completes: Arm leaves when to
    /// the reset logic, and software waits for it in WFI. The PE leaves reset
    /// at its reset vector (RVBAR_EL2) in EL2h, the highest implemented EL.
    /// The System register fields with a Warm reset take their reset
    /// values; the others keep theirs: the debug state, the OS lock, the
    /// breakpoints and watchpoints, the debug communications channel and
    /// claim tags, and RMR_EL2.AA64, which only a Cold reset sets. The
    /// activity monitors keep counting, as only their own reset resets them.
    /// The general-purpose and SIMD registers, UNKNOWN after the reset, read
    /// 0, the Exclusives monitor clears, and the PE's Apple
    /// IMPLEMENTATION DEFINED state (its registers, performance monitor and
    /// AMX unit) restarts as on its other resets (inference: no 26A434
    /// component requests a Warm reset). The cluster's shared registers stay.
    pub(super) fn warm_reset(&mut self) -> Step {
        let pe = self.current;
        let Some(address) = self.devices.reset_vector(pe) else {
            return Err(self.unsupported(RequirementKind::Native(
                "RMR_EL2.RR requested a Warm reset, but the board holds no reset vector for the PE",
            )));
        };
        let mut cpu = Cpu::of(self.profile.chip, address, Pstate::reset(2));
        system::identify(&self.profile, pe, &mut cpu, &mut Vec::new());
        if self.native_publication.is_some() || self.cpu.has_canonical_memory() {
            self.prepare_native_reset(pe, &mut cpu).map_err(|_| {
                self.unsupported(RequirementKind::Native(
                    "native warm reset requires the actual held physical roster and canonical RAM",
                ))
            })?;
            self.cpu.clear_exclusive().map_err(|_| {
                self.unsupported(RequirementKind::Native("native warm reset monitor refused"))
            })?;
        }
        let old = core::mem::replace(&mut self.cpu, cpu);
        let cpu = &mut self.cpu;
        cpu.registers.keep_over_warm_reset(&old.registers);
        cpu.dcc = old.dcc;
        cpu.claim = old.claim;
        cpu.amu = old.amu;
        cpu.retired = old.retired;
        cpu.ctrr = old.ctrr;
        cpu.ctrr_core = ctrr::Core::handed_off(&old.core_locks);
        cpu.core_locks = old.core_locks;
        cpu.apple.impdef.join(&old.apple.impdef);
        cpu.memory_lock = old.memory_lock;
        self.waiting = None;
        self.event_sample = None;
        self.device_lines = None;
        self.translations_changed();
        Ok(())
    }

    /// PE `pe` leaves reset at `address`.
    pub(super) fn restart(&mut self, pe: usize, address: u64) -> Step {
        let mut cpu = Cpu::of(self.profile.chip, address, Pstate::reset(2));
        // Every PE has the registers of PE 0, the profile's missing ones
        // included.
        system::identify(&self.profile, pe, &mut cpu, &mut Vec::new());
        if self.native_publication.is_some() || self.cpu.has_canonical_memory() {
            self.prepare_native_reset(pe, &mut cpu).map_err(|_| {
                self.unsupported(RequirementKind::Native(
                    "native start/reset requires the actual held physical roster and canonical RAM",
                ))
            })?;
            let clear = match &mut self.peers[pe] {
                Peer::Running if pe == self.current => self.cpu.clear_exclusive(),
                Peer::Parked(parked) => parked.cpu.clear_exclusive(),
                Peer::Off | Peer::Remote => Ok(()),
                _ => unreachable!("reset preflight checked exact PE owners"),
            };
            clear.map_err(|_| {
                self.unsupported(RequirementKind::Native(
                    "native reset monitor retirement refused",
                ))
            })?;
        }
        // The PEs of a physical cluster (MPIDR_EL1 Aff1) share its CTRR
        // registers while one of them is powered; the memory controller's
        // lock binds every PE.
        let cluster = |other: usize| {
            self.profile
                .processors
                .get(other)
                .map(|identity| identity.mpidr & PHYSICAL_CLUSTER_MASK)
        };
        let shared = (0..self.peers.len())
            .filter(|&other| other != pe && cluster(other) == cluster(pe))
            .find_map(|other| match &self.peers[other] {
                Peer::Running if !down(self.waiting) => Some(&self.cpu),
                Peer::Parked(parked) if !down(parked.waiting) => Some(&parked.cpu),
                Peer::Running | Peer::Parked(_) | Peer::Off | Peer::Remote => None,
            });
        // Its cluster-scoped implementation-defined registers too
        // (`impdef::Scope::Cluster`). A new cluster's CTRR registers, and
        // every PE's own, start from what iBoot left (`ctrr::CoreLocks`).
        cpu.core_locks = self.cpu.core_locks.clone();
        cpu.ctrr_core = ctrr::Core::handed_off(&cpu.core_locks);
        // In parallel, the cluster's state outlives its PEs on the hub.
        if let Some(hub) = &self.parallel {
            let key = cluster(pe).unwrap_or(0);
            let cluster = hub.cluster(key, || super::parallel::Cluster {
                ctrr: crate::machine::shared::Shared::new(ctrr::Cluster::handed_off(
                    &cpu.core_locks,
                )),
                impdef: cpu.apple.impdef.cluster_registers(),
            });
            cpu.ctrr = cluster.ctrr;
            cpu.apple.impdef.join_cluster(cluster.impdef);
        } else if let Some(peer) = shared {
            cpu.ctrr = peer.ctrr.clone();
            cpu.apple.impdef.join(&peer.apple.impdef);
        } else {
            cpu.ctrr =
                crate::machine::shared::Shared::new(ctrr::Cluster::handed_off(&cpu.core_locks));
        }
        cpu.memory_lock = self.cpu.memory_lock;
        let now = self.now();
        pause_cpu(&mut cpu, None, now, self.processor_paused(pe));
        // Its SIQ_CFG_EL1 restarts at 0, which accepts IRQs.
        self.devices.irq_steering(pe, false, now);
        let replaced = match &self.peers[pe] {
            Peer::Running => self.cpu.retired,
            Peer::Parked(parked) => parked.cpu.retired,
            Peer::Off | Peer::Remote => 0,
        };
        self.statistics.replaced_retired = self
            .statistics
            .replaced_retired
            .saturating_add(u128::from(replaced));
        if pe == self.current {
            self.cpu = cpu;
            self.waiting = None;
            self.event_sample = None;
            self.device_lines = None;
            self.translations_changed();
        } else {
            self.peers[pe] = Peer::Parked(Box::new(Parked {
                cpu,
                event_sample: None,
                waiting: None,
                mirrors: super::Mirrors::default(),
            }));
        }
        self.devices.processor_power(pe, true);
        Ok(())
    }
}

/// Check the scheduler ownership before a checked participant plan can
/// mutate its inventory, the clock or any live/parked Cpu. An off PE has no
/// invented Cpu to settle; exactly one slot must refer to the live Cpu.
fn check_owners(
    processors: &Processors,
    current: usize,
    peers: &[Peer],
) -> Result<(), ProcessorPowerError> {
    if peers.len() != processors.count() {
        return Err(ProcessorPowerError::OwnerCount {
            expected: processors.count(),
            actual: peers.len(),
        });
    }
    if !matches!(peers.get(current), Some(Peer::Running)) {
        return Err(ProcessorPowerError::MissingLiveOwner(current));
    }
    if let Some(pe) = peers
        .iter()
        .enumerate()
        .find_map(|(pe, peer)| (pe != current && matches!(peer, Peer::Running)).then_some(pe))
    {
        return Err(ProcessorPowerError::UnexpectedLiveOwner(pe));
    }
    Ok(())
}

/// Preserve architectural state and pending status while stopping only the
/// declared core accounting. The system counter and timer comparators remain
/// global. Local reservations can be lost at any time (as at a PE switch).
fn pause_cpu(cpu: &mut Cpu, waiting: Option<Wait>, now: u64, paused: bool) {
    let waiting = paused || waiting.is_some();
    let context = cpu.pmu_context(waiting);
    if let Some(pmu) = cpu.pmu.as_mut() {
        pmu.sync(now, cpu.retired, context);
    }
    if let Some(amu) = cpu.amu.as_mut() {
        amu.sync(now, cpu.retired, waiting);
    }
}

#[cfg(test)]
mod tests;
