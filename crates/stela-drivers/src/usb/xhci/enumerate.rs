//! Enumeration (USB 2.0 9.1, 11): root and hub ports are debounced, reset
//! and addressed, and each device's descriptors read.
use super::*;

/// A port's debounce: whether a device stayed connected, and how long
/// the port took to settle in microseconds.
type Debounced = Result<(bool, u64), BlockError>;

/// Linux `hub_port_debounce`: sample a port until its connection holds
/// steady and say whether a device is there. `sample` returns the port's
/// connection and connect change, clearing a change it reports.
pub(super) fn debounce(
    minimum: Minimum,
    sample: impl FnMut() -> Result<(bool, bool), BlockError>,
) -> Result<bool, BlockError> {
    minimum.check()?;
    debounce_with_delay(sample, || minimum.wait_us(DEBOUNCE_STEP_US))
}

fn debounce_with_delay(
    mut sample: impl FnMut() -> Result<(bool, bool), BlockError>,
    mut delay: impl FnMut() -> Result<(), BlockError>,
) -> Result<bool, BlockError> {
    let mut debounce = Debounce::default();
    let mut total = 0;
    loop {
        let (connected, changed) = sample()?;
        if let Some(connected) = debounce.observe(connected, changed) {
            return Ok(connected);
        }
        if total >= DEBOUNCE_TIMEOUT_US {
            return Err(BlockError::Unsupported("USB port connection never settled"));
        }
        delay()?;
        total += DEBOUNCE_STEP_US;
    }
}

/// What is left of [`CONNECT_SETTLE_US`] once the ports have been powered
/// for `waited_us`.
fn settle_left(waited_us: u64) -> u64 {
    CONNECT_SETTLE_US.saturating_sub(waited_us)
}

/// Linux `hub_port_debounce` for several ports at once, as its hub threads
/// debounce in parallel: every unsettled port is sampled at each step, so the
/// whole set settles in one window. A port with no connection and no connect
/// change is empty once it stays so for `settle_us` from the first sample:
/// right after a controller reset a USB 3 link retrains and a USB 2 device
/// signals its connection again, which Linux finds later as a port change.
/// With `settle_us` 0 such a port is empty at its first sample. Each result
/// is the port's connection and the microseconds it took to settle.
fn debounce_ports_with_delay(
    count: usize,
    settle_us: u64,
    mut sample: impl FnMut(usize) -> Result<(bool, bool), BlockError>,
    mut delay: impl FnMut() -> Result<(), BlockError>,
) -> Vec<Debounced> {
    let mut results: Vec<Option<Debounced>> = Vec::new();
    // Each unsettled port, its debounce, and whether it has shown nothing yet.
    let mut pending = Vec::new();
    for port in 0..count {
        match sample(port) {
            Ok((false, false)) if settle_us == 0 => results.push(Some(Ok((false, 0)))),
            Ok((false, false)) => {
                pending.push((port, Debounce::default(), true));
                results.push(None);
            }
            Ok((connected, changed)) => {
                let mut debounce = Debounce::default();
                debounce.observe(connected, changed);
                pending.push((port, debounce, false));
                results.push(None);
            }
            Err(error) => results.push(Some(Err(error))),
        }
    }
    let mut total = 0;
    while !pending.is_empty() {
        let stop = if total >= DEBOUNCE_TIMEOUT_US {
            Some(BlockError::Unsupported("USB port connection never settled"))
        } else {
            delay().err()
        };
        if let Some(error) = stop {
            for (port, ..) in pending.drain(..) {
                results[port] = Some(Err(error.clone()));
            }
            break;
        }
        total += DEBOUNCE_STEP_US;
        pending.retain_mut(|(port, debounce, quiet)| {
            let settled = match sample(*port) {
                Ok((false, false)) if *quiet => (total >= settle_us).then_some(Ok((false, total))),
                Ok((connected, changed)) => {
                    *quiet = false;
                    debounce
                        .observe(connected, changed)
                        .map(|connected| Ok((connected, total)))
                }
                Err(error) => Some(Err(error)),
            };
            let done = settled.is_some();
            if done {
                results[*port] = settled;
            }
            !done
        });
    }
    results
        .into_iter()
        .map(|result| result.expect("every port settled, failed or timed out"))
        .collect()
}

/// How one root port fared in the first enumeration, for boot-time
/// diagnostics: `port=<n> connected=<bool> debounce_ms=<ms> reset_ms=<ms>
/// enumerate_ms=<ms>`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PortProbe {
    pub port: u8,
    pub connected: bool,
    /// From the first sample until the connection held steady.
    pub debounce_us: u64,
    /// Port resets, every try included.
    pub reset_us: u64,
    /// Addressing, descriptors and configuration, every try included.
    pub enumerate_us: u64,
}

impl core::fmt::Display for PortProbe {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "port={} connected={} debounce_ms={} reset_ms={} enumerate_ms={}",
            self.port,
            self.connected,
            self.debounce_us / 1000,
            self.reset_us / 1000,
            self.enumerate_us / 1000
        )
    }
}

#[derive(Default)]
pub(super) struct Debounce {
    connection: Option<bool>,
    stable: u64,
}

impl Debounce {
    /// A notification between timed samples can invalidate stability, but
    /// cannot contribute a sample to the interval.
    pub(super) fn restart_on_change(&mut self, connected: bool, changed: bool) -> bool {
        if !changed && self.connection == Some(connected) {
            return false;
        }
        self.stable = 0;
        self.connection = Some(connected);
        true
    }

    /// Timed observations are at least one debounce step apart.
    pub(super) fn observe(&mut self, connected: bool, changed: bool) -> Option<bool> {
        if self.restart_on_change(connected, changed) {
            return None;
        }
        self.stable += DEBOUNCE_STEP_US;
        (self.stable >= DEBOUNCE_STABLE_US).then_some(connected)
    }
}

/// The status change endpoint of a hub's interface in `alternate`.
pub(super) fn hub_status_endpoint(configuration: &[u8], alternate: u8) -> Option<Endpoint> {
    interfaces(configuration)
        .into_iter()
        .find(|interface| interface.class == HUB_CLASS && interface.alternate == alternate)
        .and_then(|interface| interface.endpoints.first().copied())
}

/// A hub child's route string: the parent's, with this tier's port, which
/// xHCI holds as 15 at most (Linux `usb_alloc_dev`).
pub(super) fn child_route(parent: u32, tier: u8, port: u8) -> u32 {
    parent | u32::from(port.min(MAX_ROUTE_PORT)) << (4 * tier)
}

impl Controller {
    /// Enables a slot and addresses the device at `attachment`.
    #[cfg(test)]
    #[expect(
        clippy::result_large_err,
        reason = "This test-only return keeps the shared typed source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn address(
        &mut self,
        attachment: Attachment,
        dma: &mut dyn DmaAllocator,
    ) -> Result<u8, (String, BlockError)> {
        self.address_parent(attachment, None, dma)
    }

    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn address_parent(
        &mut self,
        attachment: Attachment,
        parent_id: Option<SlotId>,
        dma: &mut dyn DmaAllocator,
    ) -> Result<u8, (String, BlockError)> {
        during(
            "validate original USB parent",
            self.validate_attachment_parent(attachment, parent_id),
        )?;
        let generation = during("allocate device generation", identity::next_generation())?;
        let slot = during("Enable Slot", self.fresh_slot())?;
        if let Err(error) = self.reserve_current(slot, generation, attachment, parent_id) {
            let _ = self.disable_slot(slot, reset::Ownership::Vacant);
            return Err(("reserve USB upstream current".into(), error));
        }
        let context = self.context();
        let allocation = format!("slot {slot} allocate contexts and EP0 ring");
        let slot_resources::Resources {
            output,
            mut input,
            control,
            ring,
        } = during(
            allocation,
            self.address_resources(slot, attachment, parent_id, dma),
        )?;
        let max_packet: u32 = match attachment.speed {
            LOW_SPEED | FULL_SPEED => 8,
            HIGH_SPEED => 64,
            _ => 512,
        };
        let (tt_slot, tt_port) = attachment.translator.unwrap_or((0, 0));
        let slot_context = [
            attachment.route | u32::from(attachment.speed) << 20 | 1 << 27,
            u32::from(attachment.root_port) << 16,
            u32::from(tt_slot) | u32::from(tt_port) << 8,
            0,
        ];
        input.write(4, 0b11u32);
        for (n, word) in slot_context.iter().enumerate() {
            input.write(context + 4 * n, *word);
        }
        let ep0 = 2 * context;
        input.write(ep0 + 4, 3 << 1 | CONTROL_TYPE << 3 | max_packet << 16);
        input.write(ep0 + 8, ring.dequeue_pointer());
        input.write(ep0 + 16, 8u32);
        let mut endpoints: [Option<Transfer>; 32] = Default::default();
        endpoints[1] = Some(Transfer {
            ring,
            streams: None,
            iso: None,
            is_in: false,
            poisoned: false,
            enabled: true,
            in_flight: 0,
            stopped: false,
        });
        let input_address = input.device_address();
        // A failed Address Device may leave the output context untouched.
        // Publish its CPU-owned zeros before DCBAA exposes the device pointer.
        during("publish initial device context", output.sync_dirty())?;
        self.dcbaa
            .write(usize::from(slot) * 8, output.device_address());
        self.slots[usize::from(slot)] = Some(Slot {
            generation,
            parent_id,
            inventory: None,
            consumer: identity::Consumer::Unclaimed,
            suspended: false,
            attachment,
            retiring: true,
            input,
            _output: output,
            context: slot_context,
            endpoints,
            retired_rings: Default::default(),
            control,
            speed: attachment.speed,
            current_configuration: Some(0),
        });
        let addressed = self.command(
            input_address,
            0,
            ADDRESS_DEVICE << 10 | u32::from(slot) << 24,
        );
        if let Err(error) = addressed {
            let _ = self.disable_slot(slot, reset::Ownership::Vacant);
            return Err((format!("slot {slot} Address Device"), error));
        }
        // Address Device sends SET_ADDRESS with BSR=0. The xHC does not
        // time the device's recovery interval (xHCI 1.2b 4.6.5); USB 2.0
        // 9.2.6.3 permits it to ignore Setup packets for another 2 ms.
        during(
            "USB address recovery",
            self.minimum_wait(ADDRESS_RECOVERY_US),
        )?;
        self.slot(slot).unwrap().retiring = false;
        Ok(slot)
    }

    /// Changes EP0's max packet size (Evaluate Context, 4.6.7).
    fn set_control_packet(&mut self, slot: u8, max_packet: u16) -> Result<(), BlockError> {
        self.clear_input(slot)?;
        let context = self.context();
        let input_address = {
            let state = self.slot(slot)?;
            state.input.write(4, 0b10u32);
            state
                .input
                .write(2 * context + 4, u32::from(max_packet) << 16);
            state.input.device_address()
        };
        self.command(
            input_address,
            0,
            EVALUATE_CONTEXT << 10 | u32::from(slot) << 24,
        )?;
        Ok(())
    }

    /// Every device on the root ports and behind their USB 2.0 hubs; a
    /// device that fails to enumerate is reported and skipped.
    pub fn enumerate(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Vec<Device> {
        self.enumerate_timed(dma, failures, &mut Vec::new())
    }

    /// [`Self::enumerate`], recording how long each root port took. The
    /// ports are powered together, debounced together and empty ones skipped
    /// at once; only connected ports are reset and enumerated, one at a time.
    pub fn enumerate_timed(
        &mut self,
        dma: &mut dyn DmaAllocator,
        failures: &mut Vec<(String, BlockError)>,
        probes: &mut Vec<PortProbe>,
    ) -> Vec<Device> {
        let mut devices = Vec::new();
        let location = |c: &Self, port: u8| format!("{} port {port}", c.location);
        let settled = match self.debounce_roots() {
            Ok(settled) => settled,
            Err(error) => {
                failures.push((format!("{} root ports", self.location), error));
                return devices;
            }
        };
        for (index, settled) in settled.into_iter().enumerate() {
            let port = index as u8 + 1;
            let (connected, debounce_us) = match settled {
                Ok(settled) => settled,
                Err(error) => {
                    failures.push((format!("{} debounce", location(self, port)), error));
                    continue;
                }
            };
            let mut probe = PortProbe {
                port,
                connected,
                debounce_us,
                reset_us: 0,
                enumerate_us: 0,
            };
            if connected {
                let location = location(self, port);
                crate::working(&location);
                let clock = self.clock;
                let start = clock.now_us();
                let mut reset_us = 0;
                let reset = |c: &mut Self| {
                    let start = clock.now_us();
                    let speed = c.root_reset(port);
                    reset_us += clock.now_us().saturating_sub(start);
                    speed.map(|speed| {
                        speed.map(|speed| Attachment {
                            root_port: port,
                            route: 0,
                            tier: 0,
                            speed,
                            translator: None,
                            parent: None,
                        })
                    })
                };
                self.connect(location, None, reset, dma, &mut devices, failures);
                probe.reset_us = reset_us;
                probe.enumerate_us = clock
                    .now_us()
                    .saturating_sub(start)
                    .saturating_sub(reset_us);
            }
            probes.push(probe);
        }
        devices
    }

    /// Power every unpowered root port and wait once for them all. The
    /// connect settle time runs from the first call that finds them all
    /// powered, or from a later one that had to power one again.
    pub(super) fn power_roots(&mut self) -> Result<(), BlockError> {
        self.check_minimum()?;
        let mut powered = false;
        for port in 1..=self.max_ports {
            self.validate_current_graph(port)?;
            if self.registers.read32(self.port_offset(port)) & PP == 0 {
                self.write_port(port, PP);
                powered = true;
            }
        }
        if powered {
            self.minimum_wait(PORT_POWER_US)?;
        }
        if powered || self.settle_from.is_none() {
            self.settle_from = Some(self.clock.now_us());
        }
        Ok(())
    }

    /// Power the root ports ([`Self::power_roots`]), then debounce every
    /// port together. What is left of the connect settle time since the
    /// ports were powered is waited here, once for them all.
    fn debounce_roots(&mut self) -> Result<Vec<Debounced>, BlockError> {
        self.power_roots()?;
        let waited = self
            .clock
            .now_us()
            .saturating_sub(self.settle_from.unwrap_or(u64::MAX));
        let minimum = self.minimum.clone();
        minimum.check()?;
        let settled = debounce_ports_with_delay(
            usize::from(self.max_ports),
            settle_left(waited),
            |index| Ok(self.sample_root(index as u8 + 1)),
            || minimum.wait_us(DEBOUNCE_STEP_US),
        );
        self.minimum_status(Ok(settled))
    }

    /// A root port's connection and connect change, clearing the change.
    fn sample_root(&mut self, port: u8) -> (bool, bool) {
        let portsc = self.registers.read32(self.port_offset(port));
        if portsc & CSC != 0 {
            self.write_port(port, CSC);
        }
        (
            portsc & CCS != 0 || self.warm_reset_required(port, portsc),
            portsc & CSC != 0,
        )
    }

    /// Linux `hub_port_warm_reset_required`, with Cold Attach Status read as
    /// `xhci_hub_report_usb3_link_state` reads it: a powered USB 3 root port
    /// whose link is in SS.Inactive or Compliance Mode, or that saw a device
    /// it could not train (CAS, xHCI 1.2 4.19.8), holds a device only a warm
    /// reset brings up, whether or not it shows a connection.
    pub(super) fn warm_reset_required(&self, port: u8, portsc: u32) -> bool {
        let link = portsc >> PLS_SHIFT & PLS_FIELD;
        self.port_major.get(usize::from(port.wrapping_sub(1))) == Some(&3)
            && portsc & PP != 0
            && (portsc & CAS != 0 || matches!(link, LINK_INACTIVE | LINK_COMPLIANCE))
    }

    /// Linux `hub_port_connect`: reset a port and enumerate its device, up
    /// to four times; only the last failure is reported.
    pub(super) fn connect(
        &mut self,
        location: String,
        parent_id: Option<SlotId>,
        mut reset: impl FnMut(&mut Self) -> Result<Option<Attachment>, BlockError>,
        dma: &mut dyn DmaAllocator,
        devices: &mut Vec<Device>,
        failures: &mut Vec<(String, BlockError)>,
    ) {
        let mut failure = None;
        for _ in 0..PORT_INIT_TRIES {
            if let Some(id) = parent_id
                && let Err(error) = self.validate_topology_id(id)
            {
                failures.push((format!("{location} original parent generation"), error));
                return;
            }
            let attachment = match reset(self) {
                Ok(Some(attachment)) => attachment,
                Ok(None) => return,
                Err(error) => {
                    failures.push((format!("{location} enable/reset port"), error));
                    return;
                }
            };
            match self.identify_parent(attachment, parent_id, location.clone(), dma) {
                Ok(device) => {
                    return self.adopt(device, attachment, location, dma, devices, failures);
                }
                Err(error) => failure = Some((attachment, error)),
            }
        }
        if let Some((attachment, (operation, error))) = failure {
            let at = self.port_offset(attachment.root_port);
            let portsc = self.registers.read32(at);
            failures.push((
                format!(
                    "{location} speed={} root-portsc={portsc:#010x} {operation}",
                    attachment.speed
                ),
                error,
            ));
        }
    }

    /// A root port's device speed once the port is enabled, or `None` with
    /// nothing connected.
    pub(super) fn root_port(&mut self, port: u8) -> Result<Option<u8>, BlockError> {
        self.check_minimum()?;
        self.validate_current_graph(port)?;
        let at = self.port_offset(port);
        if self.registers.read32(at) & PP == 0 {
            self.write_port(port, PP);
            self.minimum_wait(PORT_POWER_US)?;
        }
        let connected = debounce(self.minimum.clone(), || Ok(self.sample_root(port)));
        let connected = self.minimum_status(connected)?;
        if !connected {
            return Ok(None);
        }
        self.root_reset(port)
    }

    /// Reset a debounced root port and return its device's speed once the
    /// port is enabled, or `None` once nothing is connected.
    pub(super) fn root_reset(&mut self, port: u8) -> Result<Option<u8>, BlockError> {
        self.check_minimum()?;
        self.validate_current_graph(port)?;
        let at = self.port_offset(port);
        let clock = self.clock;
        let portsc = self.registers.read32(at);
        let warm = self.warm_reset_required(port, portsc);
        if portsc & CCS == 0 && !warm {
            return Ok(None);
        }
        // Clear the remaining changes, so later changes are this driver's.
        self.write_port(port, portsc & PORT_CHANGES);
        let usb3 = self.port_major[usize::from(port - 1)] == 3;
        if warm {
            // Linux `hub_port_reset` warm-resets a SuperSpeed link stuck in
            // SS.Inactive or Compliance Mode, or one that never trained.
            self.write_port(port, WPR);
            if !wait(&mut *self.registers, clock, PORT_RESET_US, |r| {
                r.read32(at) & (WRC | PRC) == WRC | PRC
            }) {
                return Err(BlockError::Timeout);
            }
            self.write_port(port, WRC | PRC);
            self.minimum_wait(RESET_RECOVERY_US)?;
        } else if !usb3 {
            self.write_port(port, PR);
            let clock = self.clock;
            if !wait(&mut *self.registers, clock, PORT_RESET_US, |r| {
                r.read32(at) & PRC != 0
            }) {
                return Err(BlockError::Timeout);
            }
            self.write_port(port, PRC);
            self.minimum_wait(RESET_RECOVERY_US)?;
        } else {
            let clock = self.clock;
            if !wait(&mut *self.registers, clock, PORT_RESET_US, |r| {
                r.read32(at) & PED != 0
            }) {
                return Err(BlockError::Timeout);
            }
        }
        let portsc = self.registers.read32(at);
        // Linux `hub_port_reset` returns -ENOTCONN for a warm reset that
        // leaves no connection, and the port stays disabled: it is empty,
        // not a failure. The warm reset cleared CAS, so the next look does
        // not reset it again.
        if warm && portsc & CCS == 0 {
            return Ok(None);
        }
        if portsc & PED == 0 {
            return Err(BlockError::Unsupported("root port did not enable"));
        }
        if !usb3 || warm {
            self.current_port_reset(None, port)?;
        }
        Ok(Some((portsc >> PORT_SPEED_SHIFT & PORT_SPEED_FIELD) as u8))
    }

    /// Choose an enumerated device's configuration and bring up a hub's
    /// ports; the device joins the inventory either way.
    /// `usb_choose_configuration` for a freshly identified device. A failed
    /// selection leaves the descriptor inventory with no configuration.
    pub(super) fn choose(
        &mut self,
        device: &mut Device,
        location: &str,
        failures: &mut Vec<(String, BlockError)>,
    ) -> bool {
        // After boot no network adapter starts, so its chip is not asked
        // either.
        if !self.after_boot
            && let Err(error) = self.preflight_vendor_ethernet(device)
        {
            failures.push((format!("{location} RTL815x configuration preflight"), error));
        }
        if let Err(why) = configuration::select_with_controller(device, Some(self)) {
            device.configuration.clear();
            failures.push((
                format!("{location} {} choose configuration", device.name()),
                BlockError::Unsupported(why),
            ));
            return false;
        }
        true
    }

    fn adopt(
        &mut self,
        mut device: Device,
        attachment: Attachment,
        location: String,
        dma: &mut dyn DmaAllocator,
        devices: &mut Vec<Device>,
        failures: &mut Vec<(String, BlockError)>,
    ) {
        if self.choose(&mut device, &location, failures)
            && device.descriptor[4] == HUB_CLASS
            && let Err(error) = self.hub(&device, attachment, dma, devices, failures)
        {
            failures.push((format!("{location} {} enumerate hub", device.name()), error));
        }
        self.initial_usb2_lpm(&device, attachment);
        // Selection failures still preserve the entire descriptor
        // inventory for diagnostics and future generic forwarding.
        if let Ok(slot) = self.slot(device.slot) {
            slot.inventory = Some(device.clone());
        }
        devices.push(device);
    }

    pub(crate) fn preflight_vendor_ethernet(
        &mut self,
        device: &mut Device,
    ) -> Result<(), BlockError> {
        if ethernet::matches(&device.descriptor) {
            use ethernet::rtl815x::Revision;
            let bytes = self.device(device.id)?.control(Revision::request(), &[])?;
            let revision = Revision::from_register(&bytes)?;
            device.vendor_ethernet_revision = Some(revision.0);
            device.vendor_ethernet_tcr = Some(u32::from_le_bytes(bytes.try_into().unwrap()));
        }
        Ok(())
    }

    /// Address a new device and read its descriptors. A device that fails
    /// gives its slot back.
    #[cfg(test)]
    #[expect(
        clippy::result_large_err,
        reason = "This test-only return keeps the shared typed source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn identify(
        &mut self,
        attachment: Attachment,
        location: String,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Device, (String, BlockError)> {
        self.identify_parent(attachment, None, location, dma)
    }

    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn identify_parent(
        &mut self,
        attachment: Attachment,
        parent_id: Option<SlotId>,
        location: String,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Device, (String, BlockError)> {
        let slot = self.address_parent(attachment, parent_id, dma)?;
        let described = self.describe(slot, attachment, location);
        if described.is_err() {
            // Best effort: a slot that will not disable stays enabled.
            let _ = self.disable_slot(slot, reset::Ownership::Vacant);
        }
        if let Ok(device) = &described {
            self.slot(slot).unwrap().inventory = Some(device.clone());
        }
        described
    }

    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn describe(
        &mut self,
        slot: u8,
        attachment: Attachment,
        location: String,
    ) -> Result<Device, (String, BlockError)> {
        let first = self.enumeration_descriptor(slot, DEVICE_DESCRIPTOR, 0, 8)?;
        if first.len() < 8 {
            return during(
                format!("slot {slot} device descriptor header"),
                Err(BlockError::Unsupported("short device descriptor")),
            );
        }
        let ep0 = u16::from(first[7]);
        let expected = match attachment.speed {
            LOW_SPEED | FULL_SPEED => 8,
            HIGH_SPEED => 64,
            _ => 512,
        };
        if attachment.speed == FULL_SPEED && ep0 != expected && [8, 16, 32, 64].contains(&ep0) {
            during(
                format!("slot {slot} Evaluate Context EP0 max_packet={ep0}"),
                self.set_control_packet(slot, ep0),
            )?;
        }
        let bytes = self.enumeration_descriptor(slot, DEVICE_DESCRIPTOR, 0, 18)?;
        let descriptor: [u8; 18] = during(
            format!("slot {slot} full device descriptor"),
            bytes
                .get(..18)
                .and_then(|b| b.try_into().ok())
                .ok_or(BlockError::Unsupported("short device descriptor")),
        )?;
        let identity = format!(
            "{:04x}:{:04x}",
            u16::from_le_bytes([descriptor[8], descriptor[9]]),
            u16::from_le_bytes([descriptor[10], descriptor[11]])
        );
        if descriptor[0] != 18 || descriptor[1] != DEVICE_DESCRIPTOR || descriptor[17] == 0 {
            return during(
                format!("{identity} slot {slot} device descriptor"),
                Err(BlockError::Unsupported(
                    "invalid USB device descriptor/configuration count",
                )),
            );
        }
        let configurations = self
            .configurations(slot, descriptor[17])
            .map_err(|(operation, error)| (format!("{identity} {operation}"), error))?;
        let language = if descriptor[14..17].iter().any(|index| *index != 0) {
            self.language(slot)
        } else {
            None
        };
        let mut string =
            |index| language.and_then(|language| self.string_owned(slot, language, index));
        let manufacturer = string(descriptor[14]);
        let product = string(descriptor[15]);
        let serial = string(descriptor[16]);
        let device = Device {
            id: self
                .device_id(slot)
                .map_err(|error| ("identify device generation".into(), error))?,
            vendor_ethernet_revision: None,
            vendor_ethernet_tcr: None,
            slot,
            speed: attachment.speed,
            location,
            descriptor,
            configurations,
            configuration: Vec::new(),
            manufacturer,
            product,
            serial,
            language,
        };
        Ok(device)
    }

    /// Read every descriptor index, keeping the complete original byte streams.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn configurations(
        &mut self,
        slot: u8,
        count: u8,
    ) -> Result<Vec<Vec<u8>>, (String, BlockError)> {
        if count == 0 {
            return during(
                format!("slot {slot} configuration inventory"),
                Err(BlockError::Unsupported("USB device has no configurations")),
            );
        }
        let mut configurations = Vec::new();
        let mut inventory_bytes = 0;
        for index in 0..count {
            let bytes = self.configuration(slot, index)?;
            inventory_bytes += bytes.len();
            if inventory_bytes > CONFIGURATION_INVENTORY_BYTES {
                return during(
                    format!("slot {slot} configuration {index}"),
                    Err(BlockError::Unsupported(
                        "USB configuration inventory exceeds 64 KiB",
                    )),
                );
            }
            // A device may answer two indices with one configuration value.
            // Linux keeps both and a value selects the first; so does this.
            configurations.push(bytes);
        }
        self.current_descriptions(slot, &configurations)
            .map_err(|error| ("retain USB configuration current descriptors".into(), error))?;
        Ok(configurations)
    }

    /// Configuration descriptor `index`, whole.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn configuration(
        &mut self,
        slot: u8,
        index: u8,
    ) -> Result<Vec<u8>, (String, BlockError)> {
        let head = self.enumeration_descriptor(slot, CONFIGURATION_DESCRIPTOR, index, 9)?;
        if head.len() < 9 {
            return during(
                format!("slot {slot} configuration {index} header"),
                Err(BlockError::Unsupported("short configuration descriptor")),
            );
        }
        let total = u16::from_le_bytes([head[2], head[3]]);
        if head[0] != 9
            || head[1] != CONFIGURATION_DESCRIPTOR
            || total < 9
            || usize::from(total) > CONTROL_BYTES
        {
            return during(
                format!("slot {slot} configuration {index} length={total}"),
                Err(BlockError::Unsupported(
                    "invalid or oversized USB configuration descriptor",
                )),
            );
        }
        let bytes = self.enumeration_descriptor(slot, CONFIGURATION_DESCRIPTOR, index, total)?;
        during(
            format!(
                "slot {slot} configuration {index} value={} validate",
                head[5]
            ),
            if bytes[..9] != head[..9] {
                Err(BlockError::Unsupported(
                    "USB configuration header changed during enumeration",
                ))
            } else {
                Ok(())
            },
        )?;
        Ok(bytes)
    }

    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn enumeration_descriptor(
        &mut self,
        slot: u8,
        kind: u8,
        index: u8,
        length: u16,
    ) -> Result<Vec<u8>, (String, BlockError)> {
        during(
            format!("slot {slot} EP0 GET_DESCRIPTOR type={kind} index={index} length={length}"),
            self.descriptor(slot, kind, index, length),
        )
    }

    pub(super) fn hub(
        &mut self,
        device: &Device,
        attachment: Attachment,
        dma: &mut dyn DmaAllocator,
        devices: &mut Vec<Device>,
        failures: &mut Vec<(String, BlockError)>,
    ) -> Result<(), BlockError> {
        if self.validate_topology_id(device.id)? != device.slot {
            return Err(BlockError::Unsupported(
                "USB hub original descriptor slot changed",
            ));
        }
        if self.slot(device.slot)?.attachment != attachment {
            return Err(BlockError::Unsupported(
                "USB hub attachment is not its retained owner",
            ));
        }
        if matches!(attachment.speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
            return self.superspeed_hub(device, attachment, dma, devices, failures);
        }
        if attachment.tier + 1 > MAX_TIERS {
            return Err(BlockError::Unsupported("hub more than five tiers deep"));
        }
        let slot = device.slot;
        // get_hub_descriptor: three tries for a reply holding the fixed
        // fields and the port count.
        let mut bytes = Vec::new();
        for _ in 0..3 {
            bytes = self.control_owned(
                slot,
                setup(
                    DEVICE_TO_HOST | TYPE_CLASS,
                    GET_DESCRIPTOR,
                    u16::from(HUB_DESCRIPTOR) << 8,
                    0,
                    hub::descriptor::Usb2Descriptor::REQUEST,
                ),
                &[],
            )?;
            if bytes.len() >= hub::descriptor::Usb2Descriptor::MINIMUM {
                break;
            }
        }
        if bytes.len() < hub::descriptor::Usb2Descriptor::MINIMUM {
            return Err(BlockError::Unsupported("short USB2 hub descriptor"));
        }
        let original = device
            .configurations
            .iter()
            .find(|configuration| {
                configuration.get(5).copied() == Some(device.configuration_value())
            })
            .ok_or(BlockError::Unsupported(
                "USB2 hub selected original configuration absent",
            ))?;
        let endpoint = hub::usb2_status_endpoint(original, device.speed, 0, bytes[2])?;
        let descriptor = hub::descriptor::Usb2Descriptor::new(&bytes, &endpoint)?;
        let ports = descriptor.ports;
        let tt_think = if attachment.speed == HIGH_SPEED {
            descriptor.tt_think
        } else {
            0
        };
        self.set_configuration_owned(slot, device.configuration_value())?;
        // Device status describes the configuration the device acknowledged.
        let self_powered = self.hub_power_source(slot)?;
        self.install_hub_current(slot, ports, descriptor.controller_ma, self_powered)?;
        self.observe_hub_current(slot)?;
        // Linux `hub_configure`: a multi-TT hub (device protocol 2) gets one
        // TT per port through interface 0's alternate setting 1; a refusal
        // leaves it a single-TT hub.
        let multi = device.descriptor[6] == HUB_MULTI_TT
            && self
                .control_owned(
                    slot,
                    setup(
                        RECIPIENT_INTERFACE,
                        SET_INTERFACE,
                        u16::from(MULTI_TT_ALTERNATE),
                        0,
                        0,
                    ),
                    &[],
                )
                .is_ok();
        let alternate = if multi { MULTI_TT_ALTERNATE } else { 0 };
        // SET_INTERFACE success is physical permission for this setting. If its
        // original descriptor cannot supply a complete status bitmap, refuse
        // before Configure Endpoint, downstream power or child publication.
        let endpoint = hub::usb2_status_endpoint(original, device.speed, alternate, ports)?;
        self.configure_owned(slot, &[endpoint], Some((ports, tt_think, multi)), dma)?;
        let mut powered_ports = Vec::new();
        for port in 1..=ports {
            if let Err(error) = self.reserve_port_current(slot, port) {
                failures.push((
                    format!("{}.{port} reserve initial USB current", device.location),
                    error,
                ));
                continue;
            }
            self.control_owned(
                slot,
                setup(
                    TYPE_CLASS | RECIPIENT_OTHER,
                    SET_FEATURE,
                    PORT_POWER,
                    u16::from(port),
                    0,
                ),
                &[],
            )?;
            powered_ports.push(port);
        }
        self.minimum_wait(descriptor.power_good_us)?;
        for port in powered_ports {
            let location = format!("{}.{port}", device.location);
            crate::working(&location);
            let reset = |c: &mut Self| {
                c.hub_port_for(device.id, port).map(|speed| {
                    speed.map(|speed| Attachment {
                        root_port: attachment.root_port,
                        route: child_route(attachment.route, attachment.tier, port),
                        tier: attachment.tier + 1,
                        speed,
                        parent: Some((slot, port)),
                        translator: match (speed, attachment.speed) {
                            (LOW_SPEED | FULL_SPEED, HIGH_SPEED) => Some((slot, port)),
                            _ => attachment.translator,
                        },
                    })
                })
            };
            self.connect(location, Some(device.id), reset, dma, devices, failures);
        }
        Ok(())
    }

    pub(super) fn hub_port_status_for(
        &mut self,
        id: SlotId,
        port: u8,
    ) -> Result<(u16, u16), BlockError> {
        let slot = self.validate_topology_id(id)?;
        self.hub_port_status_owned(slot, port)
    }

    #[cfg(test)]
    pub(super) fn hub_port_status(
        &mut self,
        id: impl DeviceRef,
        port: u8,
    ) -> Result<(u16, u16), BlockError> {
        let slot = id.current_slot(self)?;
        self.hub_port_status_owned(slot, port)
    }

    pub(super) fn hub_port_status_owned(
        &mut self,
        slot: u8,
        port: u8,
    ) -> Result<(u16, u16), BlockError> {
        let status = self.control_owned(
            slot,
            setup(
                DEVICE_TO_HOST | TYPE_CLASS | RECIPIENT_OTHER,
                GET_STATUS,
                0,
                u16::from(port),
                4,
            ),
            &[],
        )?;
        if status.len() != 4 {
            return Err(BlockError::Unsupported("short hub port status"));
        }
        hub::validate_port_status(&status, self.slot(slot)?.attachment.speed)?;
        Ok((
            u16::from_le_bytes([status[0], status[1]]),
            u16::from_le_bytes([status[2], status[3]]),
        ))
    }
}

#[cfg(test)]
mod tests;
