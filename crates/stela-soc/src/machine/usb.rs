//! Type-C connectors, USB controllers and the peripherals and network the host
//! attaches through them.

use super::*;
use crate::Result;
use stela_contracts::refuse;

/// The reason of the power-off a held power button makes.
pub const POWER_BUTTON_HELD: &str = "smc: power button held: system power-off";

impl AppleDevices {
    /// The shared cable state of one Type-C connector.
    pub(super) fn typec_port(&mut self, number: u32) -> typec::Port {
        if let Some((_, port)) = self.typec_ports.iter().find(|(n, _)| *n == number) {
            return port.clone();
        }
        let port = typec::Port::new();
        self.typec_ports.push((number, port.clone()));
        port
    }

    /// Place canonical PHY regions once. ADT aliases inside the core resolve
    /// by physical address; no length concatenation or duplicate storage.
    pub fn add_atc_phy(&mut self, placement: &stela_soc_data::board::TypecPhy) -> Result<()> {
        if placement.phandle == 0
            || placement.port == 0
            || self.phys.iter().any(|phy| phy.phandle == placement.phandle)
        {
            return Err(refuse!(
                "ATC PHY has an invalid/duplicate phandle or invalid connector"
            ));
        }
        let id = self.bus.add(Box::new(AtcPhy::new()));
        let pipe = placement.pipe.map(|pipe| (pipe, atc_phy::PIPE));
        for ((physical, bytes), offset) in [
            (placement.usb2, atc_phy::USB2),
            (placement.core, 0),
            (placement.events, atc_phy::EVENTS),
        ]
        .into_iter()
        .chain(pipe)
        {
            self.bus
                .map(id, physical, bytes, offset)
                .map_err(|error| refuse!("ATC PHY placement: {error}", ?error))?;
        }
        self.phys.push(PhyBinding {
            phandle: placement.phandle,
            port: placement.port,
            device: id,
            retention: placement.retention,
            pipe: placement.pipe,
            events: placement.events,
        });
        Ok(())
    }

    /// Place the core once, with reg[2]/reg[8] sharing reg[0]'s bytes.
    /// Platform windows keep distinct refusing offsets; reg[3] and reg[4]
    /// are the pipehandler and core event block, which the ATC PHY placement
    /// owns (`atc-phy-parent`). IRQ indices 0..3
    /// are from AppleUSBXDCI::goOnBus (KC 0x42f5338/5380/53c8/5d24),
    /// passed unchanged to EventHandler::withParameters; initializePlatformSettings
    /// at 0x4338e74 chooses four handlers. The fifth ADT IRQ is not an event ring.
    pub fn add_usb(
        &mut self,
        placement: &stela_soc_data::board::UsbController,
        dart: DartId,
    ) -> Result<UsbId> {
        self.validate_native_dart("USB", dart, placement.stream as usize)?;
        let binding = self
            .phys
            .iter()
            .find(|phy| phy.phandle == placement.phy && phy.port == placement.port)
            .ok_or_else(|| {
                refuse!(
                    "USB atc-phy-parent {phy:#x} has no placed PHY on connector {port}",
                    phy = placement.phy,
                    port = placement.port
                )
            })?;
        if binding.pipe != Some(placement.ranges[3]) || binding.events != placement.ranges[4] {
            return Err(refuse!(
                "USB PHY lacks the matching physical pipehandler reset/event windows"
            ));
        }
        let phy = binding.device;
        let state = self
            .bus
            .device::<AtcPhy>(phy)
            .expect("placed PHY type")
            .usb2_state();
        let connector = self.typec_port(placement.port);
        let irqs = core::array::from_fn(|n| AicInput::new(0, placement.irqs[n] as u16));
        let mut model = Dwc3::new(irqs);
        model.set_phy_state(
            state.controller_reset,
            state.ready,
            state.clocked,
            state.quiesced,
            &mut Access::new(0, 0, &mut self.latch),
        );
        let device = self.bus.add(Box::new(model));
        let (physical, bytes) = placement.ranges[0];
        self.bus
            .map(device, physical, bytes, 0)
            .map_err(|e| refuse!("DWC3 placement: {e}", ?e))?;
        for index in [1, 5, 6, 7] {
            let (physical, bytes) = placement.ranges[index];
            self.bus
                .map(
                    device,
                    physical,
                    bytes,
                    dwc3::PLATFORM_WINDOW_BASE + index as u64 * dwc3::PLATFORM_WINDOW_STRIDE,
                )
                .map_err(|e| refuse!("USB platform window {index}: {e}", index, ?e))?;
        }
        self.usb_masters.push(UsbMaster {
            device,
            phy,
            phy_changed: true,
            dart: dart.0,
            stream: placement.stream as usize,
            session: UsbSession::new(),
            port: UsbPort::of(&connector),
            number: placement.port,
            connector,
            peripheral: None,
            next_poll: 0,
            attached: false,
            epoch: 0,
            retention: placement.retention,
            session_reset: false,
        });
        Ok(UsbId(device))
    }

    /// A cable change on one Type-C connector, as the external peer makes
    /// it. The connector's HPM dies and USB controller observe the same state.
    pub fn set_connection(
        &mut self,
        connector: u32,
        connection: Option<typec::Attached>,
    ) -> Result<()> {
        self.typec_port(connector).set_connection(connection)?;
        Ok(())
    }

    /// Cables a native high-speed hub to Type-C connector `connector`,
    /// this machine being the downstream-facing port that sources VBUS: the
    /// HPM reports the attach and the controller's host personality finds
    /// the hub on its USB2 port. The keyboard and pointer sit on the hub's
    /// first port, and `cables` names the USB Ethernet adapter, Bluetooth
    /// controller and mass storage devices on the next ones.
    pub fn attach_input(&mut self, connector: u32, cables: UsbCables) -> Result<()> {
        let UsbCables {
            disks,
            network,
            bluetooth,
            audio,
        } = cables;
        let extras = usize::from(network.is_some())
            + usize::from(bluetooth.is_some())
            + usize::from(audio.is_some());
        if 1 + extras + disks.len() > hub::MAX_PORTS {
            return Err(refuse!("the native hub has seven ports"));
        }
        let network = network
            .map(|mac| Ecm::with_model(mac, crate::usb::ecm::Model::Ncm))
            .transpose()?;
        let disks = disks
            .into_iter()
            .map(MassStorage::of)
            .collect::<Result<Vec<_>, _>>()?;
        let master = self
            .usb_masters
            .iter_mut()
            .find(|master| master.number == connector)
            .ok_or(refuse!("no USB controller on that connector"))?;
        let attachments = Attachments {
            input: Peripheral::new(),
            network,
            bluetooth: bluetooth.map(|kind| match kind {
                UsbBluetooth::Host => Hci::new(),
                UsbBluetooth::Model => Hci::modeled(crate::usb::hci::model::address(self.ecid)),
            }),
            audio: audio.map(crate::usb::audio::Audio::named),
            disks,
        };
        master.peripheral = Some(Hub::new(attachments, "0001")?);
        master.connector.set_connection(Some(typec::Attached {
            orientation: typec::Orientation::Normal,
            role: typec::DataRole::Dfp,
            source: true,
            vbus: true,
            speed: typec::Usb2Speed::High,
        }))?;
        Ok(())
    }

    pub fn usb_audio(&mut self) -> Option<&mut crate::usb::audio::Audio> {
        self.usb_masters
            .iter_mut()
            .find_map(|master| master.peripheral.as_mut()?.ports.audio.as_mut())
    }

    pub(super) fn usb_network(&mut self) -> Result<&mut Ecm> {
        self.usb_masters
            .iter_mut()
            .find_map(|master| master.peripheral.as_mut()?.ports.network.as_mut())
            .ok_or(refuse!("no USB Ethernet adapter is attached"))
    }

    /// A frame from the host network for the guest's USB Ethernet adapter;
    /// dropped as the adapter's filter and queue decide.
    pub fn network_deliver(&mut self, frame: &[u8]) -> Result<()> {
        self.usb_network()?.deliver(frame);
        Ok(())
    }

    /// Hand the USB Ethernet adapter an owned frame from the host link.
    pub fn network_deliver_owned(&mut self, frame: Vec<u8>) -> Result<()> {
        self.usb_network()?.deliver_owned(frame);
        Ok(())
    }

    fn usb_bluetooth(&mut self) -> Option<&mut Hci> {
        self.usb_masters
            .iter_mut()
            .find_map(|master| master.peripheral.as_mut()?.ports.bluetooth.as_mut())
    }

    /// An event from the host's Bluetooth controller for the guest's.
    pub fn bluetooth_event(&mut self, packet: &[u8]) {
        if let Some(hci) = self.usb_bluetooth() {
            hci.deliver_event(packet);
        }
    }

    /// ACL data from the host's Bluetooth controller for the guest's.
    pub fn bluetooth_acl(&mut self, packet: &[u8]) {
        if let Some(hci) = self.usb_bluetooth() {
            hci.deliver_acl(packet);
        }
    }

    /// Whether the guest's Bluetooth controller has room for a packet from
    /// the host's; false without one.
    pub fn bluetooth_room(&mut self) -> bool {
        self.usb_bluetooth().is_some_and(|hci| hci.has_room())
    }

    /// The next packet the guest sent for the host's Bluetooth controller.
    pub fn bluetooth_take(&mut self) -> Option<Outbound> {
        self.usb_bluetooth()?.take()
    }

    /// Opcodes the modeled controller answered Unknown HCI Command, each
    /// once.
    pub fn bluetooth_unknown(&mut self) -> Vec<u16> {
        self.usb_bluetooth()
            .map(|hci| hci.take_unknown())
            .unwrap_or_default()
    }

    /// The Bluetooth function's trace for the owner's log since it was
    /// last taken: configurations, resets and the first commands after each
    /// configuration.
    pub fn bluetooth_trace(&mut self) -> Vec<UsbBluetoothTrace> {
        self.usb_bluetooth()
            .map(|hci| hci.take_trace())
            .unwrap_or_default()
    }

    /// Whether the guest configured its Bluetooth controller, and the
    /// controller's counters; `None` without one.
    pub fn bluetooth_status(&mut self) -> Option<(bool, UsbBluetoothCounters)> {
        let hci = self.usb_bluetooth()?;
        Some((hci.up(), hci.counters()))
    }

    /// Whether the guest has its USB Ethernet link up, and the adapter's
    /// counters; `None` without an adapter.
    pub fn network_status(&mut self) -> Option<(bool, UsbNetworkCounters)> {
        let network = self.usb_network().ok()?;
        Some((network.up(), network.counters()))
    }

    /// The carrier of the host link the guest's USB Ethernet adapter is
    /// bridged to, which the guest sees as its cable (`Ecm::set_carrier`).
    pub fn network_carrier(&mut self, carrier: bool) {
        if let Ok(network) = self.usb_network() {
            network.set_carrier(carrier);
        }
    }

    /// Whether the guest's USB Ethernet adapter has room for a frame from
    /// the host network; false without an adapter.
    pub fn network_room(&mut self) -> bool {
        self.usb_network().is_ok_and(|network| network.has_room())
    }

    /// The next frame the guest sent through its USB Ethernet adapter.
    pub fn network_take(&mut self) -> Option<Vec<u8>> {
        self.usb_network().ok()?.take()
    }

    /// Return the host link's finished frame storage to the USB assembler.
    pub fn network_recycle(&mut self, frame: Vec<u8>) {
        if let Ok(network) = self.usb_network() {
            network.recycle_outbound(frame);
        }
    }

    /// The keyboard LEDs the guest set on the attached peripheral, Num Lock
    /// in bit 0 as input LED codes number them.
    pub fn keyboard_leds(&self) -> Option<u8> {
        self.usb_masters
            .iter()
            .find_map(|master| master.peripheral.as_ref())
            .map(|peripheral| peripheral.ports.input.leds())
    }

    /// One host input change for the attached peripheral.
    pub fn input(&mut self, input: Input) -> Result<()> {
        self.usb_masters
            .iter_mut()
            .find_map(|master| master.peripheral.as_mut())
            .ok_or(refuse!("no input peripheral is attached"))?
            .ports
            .input
            .input(input)
    }

    /// The machine's power button went down or up. The SMC reports it to
    /// the AP as a HID button notification once the AP has enabled them;
    /// while the SoC sleeps a press is a wake source instead (`sleep`).
    pub fn power_button(&mut self, pressed: bool) -> Result<()> {
        if self.smc()?.power_button(pressed) {
            self.wake(smc::WakeSource::PowerButton);
        }
        Ok(())
    }

    /// The board's power button held: the SMC cuts the board's power, whatever
    /// the AP does. That takes no message from the AP, whose next mailbox
    /// write a stuck guest never makes, so the machine turns off at once.
    pub fn power_button_held(&mut self) -> Result<()> {
        self.smc()?;
        if self.reset.is_none() {
            self.reset = Some(Reset {
                transition: Transition::PowerOff,
                reason: String::from(POWER_BUTTON_HELD),
            });
        }
        Ok(())
    }

    pub fn usb_client(&mut self, id: UsbId) -> Result<UsbClient<'_>> {
        let master = self
            .usb_masters
            .iter_mut()
            .find(|master| master.device == id.0)
            .ok_or(refuse!("unknown USB controller"))?;
        let reset = self
            .bus
            .device_mut::<Dwc3>(master.device)
            .expect("USB master owns DWC3")
            .take_session_reset();
        master.cancel_session(reset)?;
        Ok(UsbClient(&mut master.session))
    }

    /// Each USB controller's connector, whether a host session is attached
    /// and, while one cannot be, why: for the boot log.
    pub fn usb_diagnostics(&self) -> Vec<String> {
        self.usb_masters
            .iter()
            .map(|master| {
                let controller = self.bus.device::<Dwc3>(master.device).expect("USB core");
                let phy = self.bus.device::<AtcPhy>(master.phy).expect("USB PHY");
                // Only a port the machine is a device on waits for a host.
                let why = match controller.unavailable() {
                    _ if master.peripheral.is_some() => String::new(),
                    Some(why) if !controller.phy_ready() => {
                        format!(" unavailable={why}: {}", phy.usb2_unready().unwrap_or("?"))
                    }
                    Some(why) => format!(" unavailable={why}"),
                    None => String::new(),
                };
                let host = controller
                    .host_summary()
                    .map_or_else(String::new, |host| format!(" host=[{host}]"));
                let hub = master
                    .peripheral
                    .as_ref()
                    .map_or_else(String::new, |hub| format!(" hub=[{}]", hub.summary()));
                // The USB Ethernet adapter's frames since it was made: what
                // its bulk transfers and their events carry.
                let net = master
                    .peripheral
                    .as_ref()
                    .and_then(|hub| hub.ports.network.as_ref())
                    .map_or_else(String::new, |ecm| {
                        let c = ecm.counters();
                        format!(
                            " net=[model={:?} to_guest={} to_host={} filtered={} overflowed={} \
                             oversized={} malformed={}]",
                            ecm.model(),
                            c.to_guest,
                            c.to_host,
                            c.filtered,
                            c.overflowed,
                            c.oversized,
                            c.malformed
                        )
                    });
                format!(
                    "port={} attached={}{why}{host}{hub}{net}",
                    master.number,
                    master.attached || master.peripheral.is_some()
                )
            })
            .collect()
    }

    pub fn take_usb_log(&mut self) -> Option<UsbLog> {
        self.usb_masters
            .iter_mut()
            .find_map(|master| master.session.take_log())
    }

    /// Each USB controller's recent bulk and interrupt transfer steps, with
    /// its port: what the core did before a peer reported lost data.
    pub fn usb_transfer_history(&self) -> Vec<(u32, String)> {
        self.usb_masters
            .iter()
            .flat_map(|master| {
                let controller = self.bus.device::<Dwc3>(master.device).expect("USB core");
                controller
                    .transfer_history()
                    .map(move |line| (master.number, line))
            })
            .collect()
    }

    /// The next line of a USB controller's host trace, with its port.
    pub fn take_usb_host_trace(&mut self) -> Option<(u32, String)> {
        self.usb_masters.iter().find_map(|master| {
            self.bus
                .device_mut::<Dwc3>(master.device)
                .and_then(Dwc3::take_host_trace)
                .map(|line| (master.number, line))
        })
    }
}
