//! PMGR power states, lifecycle resources and the CPU, fabric and SoC power
//! banks.

use super::*;
use crate::Result;
use stela_contracts::bus::Direction;
use stela_contracts::refuse;

struct T6020UnmodeledPmgr;

impl MmioDevice for T6020UnmodeledPmgr {
    fn name(&self) -> &'static str {
        "t6020-unmodeled-pmgr"
    }

    fn read(&mut self, offset: u64, width: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        Err(stela_contracts::bus::refuse(
            self.name(),
            offset,
            width,
            Direction::Read,
            "PMGR register effects are not modeled",
        ))
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        _: u64,
        _: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        Err(stela_contracts::bus::refuse(
            self.name(),
            offset,
            width,
            Direction::Write,
            "PMGR register effects are not modeled",
        ))
    }
}

impl AppleDevices {
    /// The PMGR power-state registers at `registers`, in `blocks` of guest
    /// PA. Each starts active under the existing boot assumption; this is
    /// not a checked transfer of the original firmware's control words.
    pub fn add_power_states(&mut self, blocks: &[Range], registers: &[u64]) -> Result<()> {
        if self.pmgr.is_some() {
            return Err(refuse!("PMGR lifecycle owner is already bound"));
        }
        let states = registers.iter().map(|&address| PowerState {
            address,
            initial: ACTIVE,
        });
        let model = Pmgr::new(states).map_err(|error| refuse!("PMGR states: {error}", ?error))?;
        // The model's offsets are guest PAs. Publish all banks together.
        let bindings: Vec<_> = blocks
            .iter()
            .map(|block| (block.physical, block.bytes, block.physical))
            .collect();
        let device = self
            .bus
            .install(Box::new(model), &bindings)
            .map_err(|error| refuse!("PMGR placement: {error}", ?error))?;
        self.pmgr = Some(device);
        Ok(())
    }

    /// Register a model's checked lifecycle boundary against its
    /// canonical first physical aperture. Several real banks may serve one
    /// hardware owner; a bank must not receive conflicting duplicate inputs.
    pub fn register_power_device(
        &mut self,
        physical: u64,
        device: DeviceId,
        retention: bool,
    ) -> Result<()> {
        if self.power_configured
            || self
                .power_devices
                .iter()
                .any(|entry| entry.device == device)
        {
            return Err(refuse!(
                "native lifecycle registration is duplicate or already bound"
            ));
        }
        if self.bus.lifecycle(device).is_none() {
            return Err(refuse!(
                "native lifecycle at {physical:#x} has no actual model hook",
                physical
            ));
        }
        self.power_devices.push(PowerDevice {
            physical,
            device,
            retention,
        });
        Ok(())
    }

    /// Account for every original resource before connecting lifecycle.
    /// Mapper views have the same hardware owner as their DART. Other
    /// hardware needs its own real sinks. Validate all links before enabling
    /// any register control.
    pub fn connect_power_resources(
        &mut self,
        states: &stela_soc_data::board::PowerStates,
        darts: &[Vec<DartId>],
        retention: &[bool],
    ) -> Result<()> {
        if self.power_configured || darts.len() != retention.len() {
            return Err(refuse!(
                "PMGR binding is duplicate or has mismatched DART groups"
            ));
        }
        let pmgr = self
            .pmgr
            .and_then(|id| self.bus.device::<Pmgr>(id))
            .ok_or(refuse!("PMGR resources have no mapped power-state model"))?;
        let complete = LifecycleSinks {
            clock: true,
            power: true,
            quiesce: true,
            reset: true,
        };
        let mut devices = Vec::new();
        for instances in darts {
            if instances.is_empty() {
                return Err(refuse!("PMGR binding has an empty DART group"));
            }
            for dart in instances {
                if self.bus.device::<Dart>(dart.0).is_none() || devices.contains(&dart.0) {
                    return Err(refuse!("PMGR binding is duplicate or not a DART"));
                }
                devices.push(dart.0);
            }
        }
        let power_requirements = states.power_requirements()?;
        let mut consumers = Vec::new();
        for (id, resource) in states.resources.iter().enumerate() {
            let connected = resource.owner.is_some_and(|owner| match owner {
                PowerOwner::Mmio(physical) => self
                    .power_devices
                    .iter()
                    .any(|entry| entry.physical == physical),
                _ => true,
            });
            if connected && resource.admission == coverage::Admission::Unavailable {
                return Err(refuse!(
                    "{name} is both unavailable and placed",
                    name = resource.name
                ));
            }
            if resource.owner.is_some_and(|owner| match owner {
                PowerOwner::Dart(index) => index >= darts.len(),
                PowerOwner::Usb(index) => index >= self.usb_masters.len(),
                PowerOwner::Phy(index) => index >= self.phys.len(),
                PowerOwner::Mmio(_) => false,
            }) {
                return Err(refuse!(
                    "{name} names an unplaced hardware lifecycle owner",
                    name = resource.name
                ));
            }
            consumers.push(Consumer {
                id,
                name: &resource.name,
                clocks: resource.activated_clocks(),
                power: &power_requirements[id],
                admission: resource.admission,
                sinks: if connected {
                    complete
                } else {
                    LifecycleSinks::default()
                },
            });
        }
        let coverage = coverage::analyze(&states.domains, &consumers)
            .map_err(|error| refuse!("PMGR resource coverage: {error}", ?error))?;
        if coverage.len() != states.registers.len()
            || coverage.iter().any(|record| {
                !states.registers.contains(&record.address)
                    || pmgr.domain_state(record.address).is_none()
            })
        {
            return Err(refuse!(
                "PMGR resource graph differs from mapped power states"
            ));
        }
        let mut clients = darts
            .iter()
            .enumerate()
            .flat_map(|(index, instances)| {
                instances
                    .iter()
                    .map(move |dart| (PowerOwner::Dart(index), dart.0, retention[index]))
            })
            .collect::<Vec<_>>();
        clients.extend(
            self.usb_masters
                .iter()
                .enumerate()
                .map(|(index, master)| (PowerOwner::Usb(index), master.device, master.retention)),
        );
        clients.extend(
            self.phys
                .iter()
                .enumerate()
                .map(|(index, phy)| (PowerOwner::Phy(index), phy.device, phy.retention)),
        );
        clients.extend(self.power_devices.iter().map(|entry| {
            (
                PowerOwner::Mmio(entry.physical),
                entry.device,
                entry.retention,
            )
        }));
        let mut links = Vec::new();
        for (owner, device, retention) in clients {
            let domains = coverage
                .iter()
                .filter(|record| {
                    record
                        .consumers
                        .iter()
                        .any(|consumer| states.resources[consumer.id].owner == Some(owner))
                })
                .map(|record| {
                    (
                        record.address,
                        pmgr.domain_state(record.address).expect("validated domain"),
                    )
                })
                .collect::<Vec<_>>();
            let resources = states
                .resources
                .iter()
                .filter(|resource| resource.owner == Some(owner))
                .map(|resource| resource.name.as_str())
                .collect::<Vec<_>>()
                .join(", ");
            links.push(PowerClient {
                resources,
                device,
                owner,
                domains,
                retention,
            });
        }
        // A domain no inventoried resource names is controller-only: it
        // completes its own state machine and has no client to deliver to.
        let refusals = coverage
            .iter()
            .filter(|record| !record.bindable() && !record.controller_only())
            .map(|record| {
                let names = record
                    .consumers
                    .iter()
                    .filter(|consumer| {
                        consumer.admission == coverage::Admission::Required
                            && !consumer.sinks.complete()
                    })
                    .map(|consumer| consumer.name.as_str())
                    .collect::<Vec<_>>();
                let identities = states
                    .domains
                    .iter()
                    .filter(|domain| domain.address == Some(record.address))
                    .map(|domain| {
                        format!(
                            "{:?} (die {} ID {:#x})",
                            domain.name, domain.id.die, domain.id.device
                        )
                    })
                    .collect::<Vec<_>>()
                    .join(", ");
                (
                    record.address,
                    format!(
                        "{identities}: lifecycle sinks missing for {}",
                        names.join(", ")
                    ),
                )
            })
            .collect();
        let pmgr = self
            .bus
            .device_mut::<Pmgr>(self.pmgr.expect("validated PMGR"))
            .expect("PMGR type");
        for record in coverage {
            let result = if record.bindable() || record.controller_only() {
                pmgr.bind_lifecycle(record.address)
            } else {
                pmgr.refuse_lifecycle(record.address)
            };
            result.map_err(|error| refuse!("PMGR resource binding: {error}", ?error))?;
        }
        self.power_clients = links;
        self.power_refusals = refusals;
        self.power_configured = true;
        Ok(())
    }

    /// Resource names for domains whose consumers still lack sinks.
    pub fn power_refusals(&self) -> &[(u64, String)] {
        &self.power_refusals
    }

    /// A CPU cluster's PMGR registers at `base`: its register window, with
    /// the DVMR tables the boot firmware programmed, and its `pstates`
    /// performance-state slots.
    pub fn add_cpu_cluster(
        &mut self,
        base: u64,
        dvmr: &[(u64, u64)],
        pstates: usize,
        counter_capacity: usize,
        mapped: bool,
        events: &[u8],
    ) -> Result<()> {
        let mut model = Cluster::new(dvmr, pstates)
            .and_then(|c| c.with_counters(counter_capacity, mapped, events))?;
        // Only the P cluster's counters are mapped; only it shares with the ANE.
        if mapped {
            model = model.with_ane_states(stela_soc_data::board::ANE_CLOCKS_MHZ.len());
        }
        let table = model.pstate_bytes();
        let shared = model.share_words();
        let device = self.bus.add(Box::new(model));
        for (offset, bytes) in [
            (cluster::ACC_OFFSET, cluster::ACC_BYTES),
            (cluster::PSTATE_OFFSET, table),
            (cluster::SCRATCH[0], 8),
            (cluster::SCRATCH[1], 8),
        ] {
            self.bus
                .map(device, base + offset, bytes, offset)
                .map_err(|error| refuse!("CPU cluster placement: {error}", ?error))?;
        }
        // Every shared word lies in one of the windows just mapped.
        for (offset, words, index) in shared {
            self.passive.add(base + offset, &words, index);
        }
        Ok(())
    }

    /// CPM throttlers and restore command in the DT-validated CPM window.
    pub fn add_cpm_power(&mut self, physical: u64, performance: bool) -> Result<()> {
        let model = Cpm::new(performance);
        let offsets = model.offsets();
        let passive: Vec<_> = model
            .passive_words()
            .map(|(offset, words, index)| (offset, Arc::clone(words), index))
            .collect();
        let device = self.bus.add(Box::new(model));
        for offset in offsets {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(offset)
                        .ok_or(refuse!("CPM address overflows"))?,
                    8,
                    offset,
                )
                .map_err(|e| refuse!("CPM power placement: {e}", ?e))?;
        }
        // Every mapped word is in the window, so its address does not wrap.
        for (offset, words, index) in passive {
            self.passive.add(physical + offset, &words, index);
        }
        Ok(())
    }

    /// One fabric bridge; only described configuration and command words map.
    pub fn add_bridge(
        &mut self,
        physical: u64,
        fields: &[(u64, u32)],
        header: [u8; 4],
        configs: &[[u8; 36]],
        axi: Option<[u8; 4]>,
    ) -> Result<()> {
        let model = Bridge::new(fields, header, configs, axi)?;
        let spans = model.spans();
        let device = self.bus.add(Box::new(model));
        for (o, n) in spans {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(o)
                        .ok_or(refuse!("bridge address overflows"))?,
                    n,
                    o,
                )
                .map_err(|e| refuse!("bridge placement: {e}", ?e))?;
        }
        Ok(())
    }

    /// SOC DPE sample aliases and enable words, each mapped sparsely.
    pub fn add_soc_dpe(&mut self, physical: u64, samples: &[(u64, bool)]) -> Result<()> {
        let model = Dpe::new(samples)?;
        let spans = model.spans();
        let device = self.bus.add(Box::new(model));
        for (o, n) in spans {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(o)
                        .ok_or(refuse!("SOC DPE address overflows"))?,
                    n,
                    o,
                )
                .map_err(|e| refuse!("SOC DPE placement: {e}", ?e))?;
        }
        Ok(())
    }

    /// A typed SOC configuration bank at its DT `reg` address.
    pub fn add_soc_power(
        &mut self,
        physical: u64,
        kind: stela_soc_data::power::Kind,
    ) -> Result<()> {
        let model = Power::new(kind);
        let device = self.bus.add(Box::new(model));
        for offset in kind.offsets() {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(offset)
                        .ok_or(refuse!("SOC power address overflows"))?,
                    4,
                    offset,
                )
                .map_err(|e| refuse!("SOC power placement: {e}", ?e))?;
        }
        Ok(())
    }

    /// The eight active-low AOP wake masks at the board's `reg` bank +8,
    /// with the source number of the SMC's wake of the AP when the board
    /// names it.
    pub fn add_wake_masks(
        &mut self,
        physical: u64,
        valid: [u32; wake::WORDS],
        smc_wake: Option<u8>,
    ) -> Result<()> {
        let device = self.bus.add(Box::new(WakeMasks::new(valid)));
        self.sleep.wake_masks = Some((device, smc_wake));
        self.bus
            .map(device, physical, wake::BYTES, 0)
            .map_err(|error| refuse!("AOP wake mask placement: {error}", ?error))
    }

    /// One version-1 PMGR performance bank, at its `perf-regs` global word.
    pub fn add_performance_counters(&mut self, physical: u64, selectors: &[u16]) -> Result<()> {
        let model = PerfCounters::new(selectors)?;
        let spans = model.spans();
        let device = self.bus.add(Box::new(model));
        for (offset, bytes) in spans {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(offset)
                        .ok_or(refuse!("PMGR performance address overflows"))?,
                    bytes,
                    offset,
                )
                .map_err(|error| refuse!("PMGR performance placement: {error}", ?error))?;
        }
        Ok(())
    }

    /// A CPU/CPM energy bank at its absolute `hw-dpe-reg` address. Map only
    /// the two accumulators and, for CPM, its separate configuration word.
    pub fn add_cpu_energy(&mut self, physical: u64, cpm: bool) -> Result<()> {
        let model = Energy::new(cpm);
        let passive: Vec<_> = model
            .passive_words()
            .map(|(offset, words, index)| (offset, Arc::clone(words), index))
            .collect();
        let device = self.bus.add(Box::new(model));
        self.bus
            .map(device, physical, energy::COUNTER_BYTES, 0)
            .map_err(|error| refuse!("CPU energy placement: {error}", ?error))?;
        if cpm {
            self.bus
                .map(
                    device,
                    physical
                        .checked_add(energy::CONTROL_OFFSET)
                        .ok_or(refuse!("CPM energy configuration address overflows"))?,
                    8,
                    energy::CONTROL_OFFSET,
                )
                .map_err(|error| refuse!("CPM energy configuration placement: {error}", ?error))?;
        }
        // The words are mapped, so their addresses do not wrap.
        for (offset, words, index) in passive {
            self.passive.add(physical + offset, &words, index);
        }
        Ok(())
    }

    /// The always-on eFuse block at `physical`.
    pub fn add_aon_fuses(&mut self, physical: u64) -> Result<()> {
        let device = self.bus.add(Box::new(AonFuses));
        self.bus
            .map(device, physical, fuse::FUSE_BYTES, 0)
            .map_err(|error| refuse!("eFuse placement: {error}", ?error))
    }

    /// Place only the three T6020 IDS words, not the surrounding PMGR bank.
    pub fn add_rail_calibration(&mut self, physical: u64) -> Result<()> {
        let device = self.bus.add(Box::new(fuse::RailCalibration));
        self.bus
            .map(device, physical, 12, 0)
            .map_err(|error| refuse!("IDS calibration placement: {error}", ?error))
    }

    /// Place just the qualified 64-bit status/weight words. The virtual
    /// complexes have no physical transition pending and no DVMR limit.
    pub fn add_t6020_perf_boot(&mut self, words: &[u64]) -> Result<()> {
        let bindings: Vec<_> = words.iter().map(|&physical| (physical, 8, 0)).collect();
        self.bus
            .install(Box::new(fuse::T6020PerfBoot), &bindings)
            .map_err(|error| refuse!("T6020 PMGR boot register placement: {error}", ?error))?;
        Ok(())
    }

    /// Retain the positively located PMGR bus targets after other devices
    /// mount. Unimplemented accesses still refuse; no effects are retired.
    pub fn add_t6020_unmodeled_pmgr(&mut self, ranges: &[(u64, u64)]) -> Result<()> {
        if ranges.is_empty() {
            return Ok(());
        }
        let device = self.bus.add(Box::new(T6020UnmodeledPmgr));
        for &(base, bytes) in ranges {
            match self.bus.map_gaps(device, base, bytes, base) {
                Ok(()) | Err(MapError::Overlap { .. }) => {}
                Err(error) => return Err(refuse!("T6020 unmodeled PMGR window: {error}", ?error)),
            }
        }
        Ok(())
    }

    pub(super) fn settle_phys(&mut self, cpu: CpuIndex, now: u64) {
        for master in &mut self.usb_masters {
            let state = self
                .bus
                .device::<AtcPhy>(master.phy)
                .expect("USB retains actual PHY")
                .usb2_state();
            let mut access = Access::new(cpu, now, &mut self.latch);
            let changed = self
                .bus
                .device_mut::<Dwc3>(master.device)
                .expect("USB retains actual core")
                .set_phy_state(
                    state.controller_reset,
                    state.ready,
                    state.clocked,
                    state.quiesced,
                    &mut access,
                );
            master.phy_changed |= changed;
            self.service
                .retain_reset(&mut self.reset, reset_reason(&access));
        }
    }

    pub(super) fn settle_lifecycle(&mut self, cpu: CpuIndex, now: u64) {
        self.settle_power(cpu, now);
        self.settle_phys(cpu, now);
    }

    /// Validate every affected bank before a PMGR request can change
    /// its register or any client. A missing functional device is not created
    /// by controller-state completion. Partial capability failures are atomic.
    pub(super) fn check_power_request(
        &mut self,
        physical: u64,
        value: u64,
    ) -> Result<(), DeviceError> {
        let Some(pmgr) = self.pmgr.and_then(|id| self.bus.device::<Pmgr>(id)) else {
            return Ok(());
        };
        if pmgr.domain_state(physical).is_none() {
            return Ok(());
        }
        let transition = pmgr.preview(physical, value as u32).map_err(|reason| {
            let resources = self
                .power_refusals
                .iter()
                .find(|(address, _)| *address == physical)
                .map(|(_, resources)| resources.as_str())
                .unwrap_or("request validation");
            let control = pmgr.control_word(physical).expect("described PMGR domain");
            DeviceError::Refused(
                refuse!(
                    "PMGR domain {physical:#x} ({resources}) current control {control:#010x}, \
                     requested {value:#010x}",
                    physical,
                    resources,
                    control,
                    value
                )
                .because(reason),
            )
        })?;
        if transition.before == transition.after {
            return Ok(());
        }
        for link in &self.power_clients {
            if !matches!(link.owner, PowerOwner::Mmio(_))
                || !link.domains.iter().any(|(address, _)| *address == physical)
            {
                continue;
            }
            let states = link
                .domains
                .iter()
                .map(|(address, state)| {
                    if *address == physical {
                        transition.after
                    } else {
                        *state
                    }
                })
                .collect::<Vec<_>>();
            let proposed = LifecycleState {
                powered: states.iter().all(|state| state.power != POWER_GATED),
                clocked: states.iter().all(|state| state.power == ACTIVE),
                quiesced: states.iter().any(|state| state.disabled),
                reset: states.iter().any(|state| state.reset),
                retention: link.retention,
            };
            self.bus
                .lifecycle(link.device)
                .expect("binding checked actual lifecycle hook")
                .check_lifecycle(proposed)
                .map_err(|reason| {
                    DeviceError::Refused(
                        refuse!(
                            "PMGR domain {physical:#x} ({resources})",
                            physical,
                            resources = link.resources
                        )
                        .because(reason),
                    )
                })?;
        }
        Ok(())
    }

    /// Replay each edge: off/on or reset assertion/release must still
    /// discard ordinary state even if the final image is active again.
    /// Infallible after binding, this runs before the next client effect,
    /// including after a bus/service operation returned a partial failure.
    pub(super) fn settle_power(&mut self, cpu: CpuIndex, now: u64) {
        let Some(pmgr) = self.pmgr else {
            return;
        };
        // Taking none would move PMGR's epoch at every entry for nothing
        // (`Bus::plain_epoch`).
        if !self
            .bus
            .device::<Pmgr>(pmgr)
            .is_some_and(Pmgr::has_transitions)
        {
            return;
        }
        let transitions = self
            .bus
            .device_mut::<Pmgr>(pmgr)
            .expect("PMGR type")
            .take_transitions();
        for transition in transitions {
            for link in &mut self.power_clients {
                let Some((_, state)) = link
                    .domains
                    .iter_mut()
                    .find(|(address, _)| *address == transition.address)
                else {
                    continue;
                };
                debug_assert_eq!(*state, transition.before, "PMGR lifecycle edge order");
                *state = transition.after;
                let powered = link
                    .domains
                    .iter()
                    .all(|(_, state)| state.power != POWER_GATED);
                let clocked = link.domains.iter().all(|(_, state)| state.power == ACTIVE);
                let quiesced = link.domains.iter().any(|(_, state)| state.disabled);
                let reset = link.domains.iter().any(|(_, state)| state.reset);
                let mut access = Access::new(cpu, now, &mut self.latch);
                match link.owner {
                    PowerOwner::Dart(_) => {
                        let dart = self
                            .bus
                            .device_mut::<Dart>(link.device)
                            .expect("binding checked DART");
                        dart.set_quiesced(quiesced);
                        dart.set_clock(clocked);
                        dart.set_reset(reset, &mut access);
                        dart.set_power(powered, link.retention, &mut access);
                    }
                    PowerOwner::Usb(_) => {
                        let usb = self
                            .bus
                            .device_mut::<Dwc3>(link.device)
                            .expect("binding checked USB controller");
                        usb.set_quiesced(quiesced);
                        usb.set_clock(clocked, &access);
                        usb.set_reset(reset, &mut access);
                        usb.set_power(powered, link.retention, &mut access);
                    }
                    PowerOwner::Mmio(_) => {
                        let state = LifecycleState {
                            powered,
                            clocked,
                            quiesced,
                            reset,
                            retention: link.retention,
                        };
                        self.bus
                            .lifecycle(link.device)
                            .expect("binding checked actual lifecycle hook")
                            .set_lifecycle(state, &mut access);
                    }
                    PowerOwner::Phy(_) => {
                        let phy = self
                            .bus
                            .device_mut::<AtcPhy>(link.device)
                            .expect("binding checked PHY");
                        phy.set_quiesced(quiesced);
                        phy.set_clock(clocked);
                        phy.set_reset(reset);
                        phy.set_power(powered, link.retention);
                    }
                }
                self.service
                    .retain_reset(&mut self.reset, reset_reason(&access));
            }
            self.settle_phys(cpu, now);
            self.service.lifecycle.push(transition);
        }
    }
}
