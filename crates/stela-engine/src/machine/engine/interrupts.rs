//! Interrupt lines, monitors, event streams and WFI/WFE waits of the running
//! PE.

use super::*;

/// More instructions than one native run retires: [`CHAIN_LIMIT`] chained
/// blocks of at most [`block::MAX_BLOCK_INSTRUCTIONS`], each with a thunk
/// that completes one more.
pub(super) const NATIVE_RUN_INSTRUCTIONS: u64 =
    (CHAIN_LIMIT + 1) * (block::MAX_BLOCK_INSTRUCTIONS as u64 + 1);

impl<H: Host, D: Devices> Machine<H, D> {
    /// The system counter's count (`clock`).
    pub(super) fn now(&mut self) -> u64 {
        self.clock.read(|| self.host.counter())
    }

    pub(super) fn lines(&mut self) -> Lines {
        self.counted_lines().0
    }

    /// [`Self::lines`] and the count they were read at, 0 where no answer
    /// could change with the count.
    fn counted_lines(&mut self) -> (Lines, u64) {
        // The count is read only when an answer can change with it alone: a
        // device answer with a deadline, a timer's or a PMI's. Every exit
        // asks, and a raw validation PE has none of them.
        let Some((_, devices, pe)) = self.device_lines else {
            let now = self.now();
            return (self.lines_at(now), now);
        };
        let timer = self.timer_deadline();
        let monitor = self.cpu.pmu.as_ref().and_then(|monitor| monitor.deadline());
        let now = if devices == u64::MAX && timer.is_none() && monitor.is_none() {
            0
        } else {
            let change = devices
                .min(timer.unwrap_or(u64::MAX))
                .min(monitor.unwrap_or(u64::MAX));
            if pe == self.current && self.clock.below(change, || self.host.counter()) {
                self.clock.last()
            } else {
                self.now()
            }
        };
        let device = self.device_lines_of(self.current, now);
        (self.merge_lines(device, now, timer), now)
    }

    /// The count before which time alone leaves the running PE's lines as
    /// they were read at `now`: the devices' next deadline, and a timer's or
    /// the PMI's that has not passed (a timer that signals goes on signalling
    /// until a System register write). 0 when no such count holds for a whole
    /// native run: the PMI may be asserted, or instructions alone may raise
    /// it before a run could end.
    fn lines_steady_until(&mut self, now: u64) -> u64 {
        let devices = match self.device_lines {
            Some((_, until, pe)) if pe == self.current => until,
            _ => return 0,
        };
        let timer = self
            .timer_deadline()
            .filter(|&deadline| deadline > now)
            .unwrap_or(u64::MAX);
        let monitor = match &self.cpu.pmu {
            None => u64::MAX,
            Some(monitor) => {
                let retired = self.cpu.retired;
                if monitor.deadline().is_some_and(|deadline| deadline <= now)
                    || monitor.instruction_deadline().is_some_and(|deadline| {
                        deadline.saturating_sub(retired) <= NATIVE_RUN_INSTRUCTIONS
                    })
                {
                    return 0;
                }
                monitor.deadline().unwrap_or(u64::MAX)
            }
        };
        devices.min(timer).min(monitor)
    }

    /// Let native code read ISR_EL1 from [`GuestState::pending`] while the
    /// lines it was set from hold (`Mirrors::interrupt_status`): at EL2,
    /// where the register reads the physical lines alone, until the count
    /// [`Self::lines_steady_until`] gives or a line change moves the
    /// interrupt epoch past the one this step's lines reflect. Native code
    /// itself changes no line: device accesses and every System register
    /// write that could exit, and every step sets it again before native
    /// code runs.
    fn refresh_interrupt_status(&mut self, now: u64) {
        // EL0 of the host takes its interrupts at EL2 as EL2 does: a native
        // ERET or GEXIT may unmask there too (`lines_until`). ISR_EL1 reads
        // stay EL2's.
        let pstate = self.cpu.pstate;
        let host_user = pstate.el == 0 && !pstate.guarded && self.cpu.in_host();
        // A set IPI_SR is the PE's to clear natively ([`Devices::own_fiq`]),
        // which no epoch reports: ISR_EL1 then reads in the engine.
        let deadline = if (pstate.el == 2 || host_user) && !self.devices.own_fiq(self.current) {
            self.lines_steady_until(now)
        } else {
            0
        };
        self.mirrors.interrupt_status = register_read::Cell {
            readable: if pstate.el == 2 { deadline } else { 0 },
            value: self.chain.acknowledged_interrupt(),
        };
        self.mirrors.lines_until = deadline;
    }

    /// The lines the devices drive into PE `pe` at count `now`: the cached
    /// ones, or the latched pins of another PE, until a device changes or
    /// the devices' deadline passes; else the devices are brought up to
    /// `now` and asked. The FIQ the PE's own handle drives (an Apple fast
    /// IPI's IPI_SR) is read live at every look and never cached
    /// ([`Devices::own_fiq`]): native code may set or clear it between.
    pub(super) fn device_lines_of(&mut self, pe: usize, now: u64) -> Lines {
        // Read first: a shared board's first read of it takes the PE's
        // handle under the lock, which moves the count the cached lines
        // keep (`Devices::generation`).
        let own = self.devices.own_fiq(pe);
        let mut lines = self.cached_device_lines(pe, now);
        lines.fiq |= own;
        lines
    }

    /// [`Self::device_lines_of`] without the PE's own FIQ: what the cache
    /// holds.
    fn cached_device_lines(&mut self, pe: usize, now: u64) -> Lines {
        match self.device_lines {
            Some((lines, until, cached)) if now < until && cached == pe => lines,
            Some((_, until, _))
                if now < until
                    && let Some(lines) = self.devices.pins(pe) =>
            {
                self.device_lines = Some((lines, until, pe));
                lines
            }
            _ => {
                #[cfg(feature = "native-profile")]
                let profile_started = self
                    .host
                    .native_timing_work_start(NativeTimingWork::DeviceLines);
                let started = self.timing_start();
                let generation = self.devices.generation();
                let (lines, until) = self.devices.lines_and_deadline(pe, now);
                // Its own call moved the count once (`Devices::generation`).
                self.device_generation = generation.map(|generation| generation.wrapping_add(1));
                let until = until.unwrap_or(u64::MAX);
                self.statistics.line_queries += 1;
                self.statistics.line_ticks += self.timing_since(started);
                #[cfg(feature = "native-profile")]
                if let Some(started) = profile_started {
                    self.host
                        .native_timing_work_end(NativeTimingWork::DeviceLines, started);
                }
                self.device_lines = Some((lines, until, pe));
                lines
            }
        }
    }

    /// The running PE's interrupt lines at count `now`, which no count the
    /// devices were already given exceeds: a caller that holds a count
    /// asks at it, so the devices never see time run backwards.
    pub(super) fn lines_at(&mut self, now: u64) -> Lines {
        let device = self.device_lines_of(self.current, now);
        // A timer signals exactly when the count reaches its deadline, which
        // only System register writes change.
        let timer = self.timer_deadline();
        self.merge_lines(device, now, timer)
    }

    fn merge_lines(&self, device: Lines, now: u64, timer: Option<u64>) -> Lines {
        self.cpu.core_lines(Lines {
            irq: device.irq,
            // Apple cores signal the generic timers and the PMI as FIQ.
            fiq: device.fiq
                || timer.is_some_and(|deadline| now >= deadline)
                || monitor_asserted(&self.cpu, now),
        })
    }

    /// Bring the running PE's performance monitor and activity monitors up
    /// to now and continue counting in its current EL and wait state. Every
    /// change of either passes here: exception entry and return through
    /// `sync_regime`, and the start and end of a wait.
    pub(super) fn sync_monitor(&mut self) {
        let context = self
            .cpu
            .pmu_context(self.waiting.is_some() || self.processor_paused(self.current));
        let retired = self.cpu.retired;
        let pmu_timed = self.cpu.pmu.as_ref().map(|monitor| monitor.timed());
        let amu_timed = self.cpu.amu.as_ref().map(|monitors| monitors.timed());
        let aon_controls = self
            .cpu
            .aon
            .as_ref()
            .map(|_| self.cpu.apple.impdef.aon_controls());
        let aon_counting = self.cpu.aon.as_ref().is_some_and(|aon| aon.counting())
            || aon_controls.is_some_and(|controls| controls.enabled);
        if pmu_timed.is_none() && amu_timed.is_none() && !aon_counting {
            return;
        }
        // Without a cycle counter nothing counts time, so the count need not
        // be read.
        let now = if pmu_timed == Some(true) || amu_timed == Some(true) || aon_counting {
            self.now()
        } else {
            0
        };
        if let Some(monitor) = self.cpu.pmu.as_mut() {
            monitor.sync(now, retired, context);
        }
        if let Some(monitors) = self.cpu.amu.as_mut() {
            monitors.sync(now, retired, context.waiting);
        }
        if let (Some(aon), Some(controls)) = (self.cpu.aon.as_mut(), aon_controls) {
            aon.sync(now, retired, context.waiting, controls);
            self.publish_core_counts();
        }
    }

    /// Add the CLPC core performance counts since the last call to the
    /// board's storage of each counter (`machine::aon`), which the PE's
    /// native reads and other PEs' window loads read.
    fn publish_core_counts(&mut self) {
        let Some(aon) = self.cpu.aon.as_mut() else {
            return;
        };
        let counts = aon.take();
        for (count, encoding) in counts.into_iter().zip(crate::machine::impdef::AON_COUNTERS) {
            if count == 0 {
                continue;
            }
            let Some(address) = self.devices.register_storage(self.current, encoding) else {
                continue;
            };
            // SAFETY: `Devices::register_storage` gives the address of the
            // aligned 64-bit word the register reads, valid for the board's
            // life (`LocalRegisters::storage`); the board keeps each counter
            // as an atomic word (`platform::apple::core_counters`), which
            // other PEs load concurrently, so it is accessed atomically.
            let word = unsafe { core::sync::atomic::AtomicU64::from_ptr(address as *mut u64) };
            word.fetch_add(count, core::sync::atomic::Ordering::Relaxed);
        }
    }

    /// Complete the instruction at `pc` in the engine, counting it as
    /// retired unless it takes a synchronous exception at its own address
    /// (`Machine::take`).
    pub(super) fn completing(&mut self, pc: u64, complete: impl FnOnce(&mut Self) -> Step) -> Step {
        self.cpu.retired += 1;
        self.completing = Some(pc);
        let step = complete(self);
        self.completing = None;
        step
    }

    /// Count the instructions of block `index` before `pc` as retired.
    pub(super) fn retire_before(&mut self, index: usize, pc: u64) {
        self.cpu.retired += (pc - self.code.blocks[index].pc) / INSTRUCTION;
    }

    /// Whether an interrupt routed to `target` is taken now (Arm ARM D1.3.6;
    /// QEMU `arm_excp_unmasked`): never when it targets a lower EL. Its
    /// PSTATE mask applies at the same EL, to EL1 from EL0, and to EL2 from
    /// EL0 in the host (HCR_EL2.{E2H,TGE} {1,1}; E2H is RES1 on Apple
    /// cores); EL2 from EL0 or EL1 otherwise ignores it.
    pub(super) fn taken(&self, target: u8, mask: u8) -> bool {
        let el = self.cpu.pstate.el;
        let maskable = target == el || target == 1 || self.cpu.in_host();
        target >= el && (!maskable || self.cpu.pstate.daif & mask == 0)
    }

    /// Take a pending interrupt before the next instruction: the physical
    /// lines routed by HCR_EL2.{TGE,IMO,FMO}, then the virtual IRQ, FIQ and
    /// SError that HCR_EL2.{VI,VF,VSE} hold for EL1 and EL0 outside the host.
    pub(super) fn deliver_interrupts(&mut self) -> bool {
        let (lines, now) = self.counted_lines();
        let virtual_pending = if self.cpu.pstate.el == 2 {
            0
        } else {
            self.cpu.virtual_pending_bits()
        };
        self.cpu.state.pending = if virtual_pending != 0 {
            u32::from(DAIF_A | DAIF_I | DAIF_F)
        } else {
            u32::from(if lines.irq { DAIF_I } else { 0 } | if lines.fiq { DAIF_F } else { 0 })
        };
        self.refresh_interrupt_status(now);
        if !lines.irq && !lines.fiq && virtual_pending == 0 {
            return false;
        }
        let mut exception = None;
        for (vector, pending, mask) in [
            (Vector::Fiq, lines.fiq, DAIF_F),
            (Vector::Irq, lines.irq, DAIF_I),
        ] {
            let target = self.cpu.interrupt_target(vector);
            if pending && self.taken(target, mask) {
                exception = Some((vector, target, 0));
                break;
            }
        }
        if exception.is_none() && virtual_pending != 0 && !self.cpu.in_host() {
            for (vector, routing, virtual_bit, mask) in [
                (Vector::Fiq, HCR_FMO, HCR_VF, DAIF_F),
                (Vector::Irq, HCR_IMO, HCR_VI, DAIF_I),
                (Vector::SError, HCR_AMO, HCR_VSE, DAIF_A),
            ] {
                if self.cpu.hcr(routing)
                    && virtual_pending & (1 << virtual_bit) != 0
                    && self.cpu.pstate.daif & mask == 0
                {
                    let syndrome = if vector == Vector::SError {
                        // ESR_EL1 takes VSESR_EL2.{IDS,ISS} with EC 0x2F.
                        super::super::exception::syndrome(
                            Class::SError,
                            self.cpu.read(SystemRegister::VSESR_EL2) as u32,
                        )
                    } else {
                        0
                    };
                    exception = Some((vector, 1, syndrome));
                    break;
                }
            }
        }
        let Some((vector, target, syndrome)) = exception else {
            return false;
        };
        if vector == Vector::SError {
            // The virtual SError is taken once: HCR_EL2.VSE clears.
            let hcr = self.cpu.read(SystemRegister::HCR_EL2) & !(1 << HCR_VSE);
            self.cpu.registers.set(SystemRegister::HCR_EL2, 0, hcr);
        }
        self.statistics.interrupts += 1;
        if vector != Vector::SError
            && let Some(timing) = &mut self.step_timing
        {
            timing.levels.interrupt_taken();
        }
        if vector == Vector::Fiq {
            let now = self.clock.last();
            if self
                .timer_deadline()
                .is_some_and(|deadline| now >= deadline)
            {
                self.clock.timer_taken(now, self.cpu.retired);
            }
        }
        self.take(Exception {
            vector,
            syndrome,
            fault_address: None,
            return_address: self.cpu.pc,
            target: Some(target),
        });
        true
    }

    /// Apply Arm `GenericCounterTick`'s event-stream tests to the counts since
    /// the last sample. Sampling before the event register is read and before
    /// every write that changes a stream gives each tick the controls it had.
    pub(crate) fn sample_events(&mut self, now: u64) {
        // An already pending event absorbs every stream edge through this
        // sample. The running PE is the sole consumer, but the sample still
        // advances so the next WFE cannot see one of those edges again.
        if let Some(previous) = self.event_sample.replace(now)
            && !self.cpu.event_is_set()
            && timer::event(&self.cpu, previous, now)
        {
            self.cpu.signal_event();
        }
    }

    /// The earliest count at which the running PE's timers signal: their
    /// own deadline, kept, held back while a busy PE has not had its share
    /// of work since the last timer interrupt (`clock::TIMER_WORK`).
    /// The deadline is read from the registers' storage whenever
    /// native code changed what it was computed from.
    pub(super) fn timer_deadline(&mut self) -> Option<u64> {
        let registers = native_timer_registers(&self.cpu);
        if registers != self.timer_registers {
            self.timer_registers = registers;
            self.timers = None;
        }
        // A timer with an offset is read at the count now, once for each
        // change of the timers: the answer holds as the count advances
        // (`timer::deadline`). Without one no count is read.
        let deadline = match self.timers {
            Some(deadline) => deadline,
            None => {
                let (clock, host) = (&mut self.clock, &mut self.host);
                let deadline = timer::deadline(&self.cpu, || clock.read(|| host.counter()));
                *self.timers.insert(deadline)
            }
        };
        self.clock.timer_due(deadline)
    }

    /// WFI and WFE (Arm `Hint_WFI`, `Hint_WFE` and `AArch64_CheckForWFxTrap`).
    /// WFE with the event register set clears it and completes, and WFI with
    /// an interrupt pending, masked or not, completes; neither then traps.
    /// WFET and WFIT (FEAT_WFxT) also complete once their local timeout has
    /// expired: the virtual count reaches Xt (Arm `Hint_WFET`, `Hint_WFIT`).
    /// Otherwise SCTLR_ELx.nTWI/nTWE trap them from EL0 to EL1, or to EL2 in
    /// the host, and HCR_EL2.TWI/TWE trap them from EL0 and EL1 outside the
    /// host to EL2, with ISS.TI the form and, for the timeout forms, RV set
    /// and the register in ISS[9:5] (Arm `AArch64_WFxTrap`). On a chip
    /// without FEAT_WFxT the timeout forms are UNDEFINED. An event stream
    /// event wakes WFE and, by this implementation's choice of Arm's
    /// IMPLEMENTATION DEFINED option, clears the event register. A WFE that
    /// completes at once still yields the rest of the time slice to the
    /// other PEs (`processors`), as a spin-wait that waits for another PE
    /// would on hardware where they run at the same time; only timing
    /// changes. A WFE that waits, with other PEs on cores of their own,
    /// lets native WFE in the state spin briefly for its event register
    /// before it comes here ([`EVENT_WAITS`]).
    pub(super) fn wait(&mut self, _word: u32, kind: WaitKind, register: Option<u8>) -> Step {
        const HCR_TWI: u32 = 13;
        const HCR_TWE: u32 = 14;
        if register.is_some() {
            match self.cpu.registers.implemented(Feature::FEAT_WFxT) {
                Ok(true) => {}
                Ok(false) => return self.undefined(),
                Err(error) => {
                    return Err(self.unsupported(RequirementKind::Native(error.reason())));
                }
            }
        }
        let timeout = register.map(|register| {
            let limit = if register == 31 {
                0
            } else {
                self.cpu.state.x[usize::from(register)]
            };
            // The virtual count of the timer bank the regime reads
            // (`timer::agt`); a scaled bank's limit becomes a system count.
            let bank = timer::agt::selected(&self.cpu);
            let offset = if self.cpu.regime() == crate::machine::cpu::TranslationRegime::El20 {
                0
            } else {
                timer::value(&self.cpu, bank, SystemRegister::CNTVOFF_EL2)
            };
            if timer::scaled(&self.cpu, bank) {
                Timeout {
                    limit: timer::system_count(&self.cpu, bank, limit.wrapping_add(offset)),
                    offset: 0,
                }
            } else {
                Timeout { limit, offset }
            }
        });
        let event_sample = match kind {
            WaitKind::Event => {
                let now = self.now();
                self.sample_events(now);
                let at_once = self.cpu.consume_event();
                if let Some(timing) = &mut self.step_timing {
                    timing.levels.wait_for_event(at_once);
                }
                if at_once {
                    // The WFE of a spin-wait ends the PE's time slice: the PE
                    // it waits for runs next instead of after the slice.
                    self.slice_end = self.statistics.blocks;
                    // A WFE that finds an event set completes before any
                    // trap check, as native code completes it then
                    // ([`Mirrors::events`]).
                    return self.next();
                }
                Some(now)
            }
            WaitKind::Interrupt => {
                let at_once = self.interrupt_pending();
                if let Some(timing) = &mut self.step_timing {
                    timing.levels.wait_for_interrupt(at_once);
                }
                if at_once {
                    return self.next();
                }
                None
            }
        };
        if timeout
            .is_some_and(|timeout| timeout.expired(event_sample.unwrap_or_else(|| self.now())))
        {
            return self.next();
        }
        let el = self.cpu.pstate.el;
        let (enable_bit, hcr_trap, form) = match kind {
            WaitKind::Interrupt => (1u64 << 16, HCR_TWI, 0b00),
            WaitKind::Event => (1u64 << 18, HCR_TWE, 0b01),
        };
        // ISS.TI: 0b00 WFI, 0b01 WFE, 0b10 WFIT, 0b11 WFET; RV and the
        // register for the timeout forms.
        let iss = TRAPPED_AARCH64
            | form
            | register.map_or(0, |register| 0b10 | 1 << 2 | u32::from(register) << 5);
        if el == 0 && self.cpu.regime_sctlr() & enable_bit == 0 {
            self.take(Exception::synchronous(Class::WaitTrap, iss, self.cpu.pc));
            return Ok(());
        }
        if el != 2 && !self.cpu.in_host() && self.cpu.hcr(hcr_trap) {
            self.take(Exception::synchronous(Class::WaitTrap, iss, self.cpu.pc).to(2));
            return Ok(());
        }
        // A WFE that waits here may spin natively while other PEs run on
        // cores of their own to set its event register ([`EVENT_WAITS`]);
        // serialized PEs set none while this one runs.
        if kind == WaitKind::Event && register.is_none() && self.parallel.is_some() {
            self.mirrors.transition(self.mirror_state());
            self.mirrors[EVENT_WAITS].readable = 1;
        }
        // CYC_OVRD's power-down mode turns the core off and loses its state
        // (`processors`). A WFIT's local timeout is the core's own, so it
        // could not end that wait.
        let down = kind == WaitKind::Interrupt
            && self.cpu.wfi_mode() == registers::CYC_OVRD_WFI_POWER_DOWN;
        if down && self.cpu.has_canonical_memory() {
            self.require_native_lifecycle_hold().map_err(|_| {
                self.unsupported(RequirementKind::Native(
                    "native power-down requires the actual held physical roster",
                ))
            })?;
        }
        if down && timeout.is_some() {
            return Err(self.unsupported(RequirementKind::Native(
                "WFIT with CYC_OVRD's power-down mode (a powered-down core has no local timeout)",
            )));
        }
        if down && self.cpu.has_canonical_memory() {
            self.cpu.clear_exclusive().map_err(|_| {
                self.unsupported(RequirementKind::Native("native power-down monitor refused"))
            })?;
        }
        // The PE waits until the scheduler finds it woken
        // (`processors::Peer::ready`) and completes the instruction.
        self.waiting = Some(Wait {
            kind,
            timeout,
            down,
        });
        self.sync_monitor();
        if down {
            self.power_down();
        }
        Ok(())
    }

    /// Take what native WFEs counted of their spins
    /// ([`Mirrors::spin`](super::mirrors::Mirrors::spin)).
    pub(super) fn take_event_spins(&mut self) {
        let spin = core::mem::take(&mut self.mirrors.spin);
        if let Some(timing) = &mut self.step_timing {
            timing.levels.event_spins(spin);
        }
    }

    /// Arm `AArch64_InterruptPending` for the running PE.
    pub(super) fn interrupt_pending(&mut self) -> bool {
        let lines = self.lines();
        interrupt_pending(&self.cpu, lines)
    }

    /// [`Self::interrupt_pending`] at count `now` ([`Self::lines_at`]).
    pub(super) fn interrupt_pending_at(&mut self, now: u64) -> bool {
        let lines = self.lines_at(now);
        interrupt_pending(&self.cpu, lines)
    }
}

/// The timer registers native code writes ([`TIMER_COMPARE`]), as the
/// engine's cached timer deadline depends on them.
fn native_timer_registers(cpu: &Cpu) -> [u64; 2] {
    [
        cpu.registers.get(SystemRegister::CNTHV_CVAL_EL2, 0),
        cpu.registers.get(SystemRegister::CNTHV_CTL_EL2, 0),
    ]
}

#[cfg(test)]
mod core_count_tests {
    use super::super::tests::{FakeHost, board};
    use super::super::*;
    use crate::machine::aon::{CYCLES, CoreActivity, INSTRUCTIONS};
    use crate::machine::impdef;
    use core::sync::atomic::{AtomicU64, Ordering::Relaxed};

    /// A board whose PE's first two core performance counters are words
    /// of its own, as `platform::apple::core_counters` keeps them.
    struct Counters(Box<[AtomicU64; 2]>);

    impl Devices for Counters {
        fn read(&mut self, _: usize, _: u64, _: u8, _: u64) -> Result<u64, DeviceError> {
            Err(DeviceError::NoDevice)
        }
        fn write(&mut self, _: usize, _: u64, _: u8, _: u64, _: u64) -> Result<(), DeviceError> {
            Err(DeviceError::NoDevice)
        }
        fn lines(&mut self, _: usize, _: u64) -> Lines {
            Lines::default()
        }
        fn register_storage(&mut self, _: usize, encoding: Encoding) -> Option<u64> {
            let slot = impdef::AON_COUNTERS[..2]
                .iter()
                .position(|&counter| counter == encoding)?;
            Some(self.0[slot].as_ptr() as u64)
        }
    }

    fn write(machine: &mut Machine<FakeHost, Counters>, encoding: Encoding, value: u64) {
        let entry = impdef::lookup(encoding, crate::machine::test_chip()).unwrap();
        machine.cpu.apple.impdef.write(entry, value).unwrap();
    }

    #[test]
    fn monitor_updates_add_core_counts_to_the_boards_counters() {
        let counters = Counters(Box::new([AtomicU64::new(7), AtomicU64::new(0)]));
        let mut machine = board(&[], counters, Vec::new());
        machine.cpu.aon = CoreActivity::new(24_000_000 * 100, 24_000_000);
        machine.sync_monitor();
        let read = |machine: &Machine<FakeHost, Counters>| {
            [
                machine.devices.0[0].load(Relaxed),
                machine.devices.0[1].load(Relaxed),
            ]
        };
        assert_eq!(read(&machine), [7, 0], "nothing counts before the enable");
        write(&mut machine, impdef::AON_SELECT_0, CYCLES);
        write(&mut machine, impdef::AON_SELECT_1, INSTRUCTIONS);
        write(&mut machine, impdef::AON_ENABLE, 1);
        machine.host.now = 1000;
        machine.cpu.retired = 50;
        machine.sync_monitor();
        let enabled = read(&machine);
        machine.host.now = 2000;
        machine.cpu.retired = 80;
        machine.sync_monitor();
        let later = read(&machine);
        assert!(
            later[0] > enabled[0],
            "cycles grow with time: {enabled:?} {later:?}"
        );
        assert_eq!(later[1] - enabled[1], 30, "instructions as retired");
        write(&mut machine, impdef::AON_ENABLE, 0);
        machine.sync_monitor();
        let stopped = read(&machine);
        machine.host.now = 3000;
        machine.cpu.retired = 90;
        machine.sync_monitor();
        assert_eq!(
            read(&machine),
            stopped,
            "clearing the enable stops counting"
        );
    }
}
