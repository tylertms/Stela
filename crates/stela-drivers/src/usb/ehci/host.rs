//! The root ports of an attached EHCI controller and the hubs behind them,
//! each device brought up as Linux `hub_port_init` brings one up and its
//! Bulk-Only mass storage interface bound to [`storage`](crate::usb::storage).
//! A hub, such as the rate-matching hub behind every root port of Intel's
//! 5 to 8 series EHCI controllers, is configured and its ports debounced
//! together, reset and enumerated as Linux `hub_events` does, five tiers
//! deep. Full- and low-speed devices behind a high-speed hub run as split
//! transactions; on a root port they go to the companion controller, which
//! has no driver here.
use super::{
    Controller, DeviceId, Error, Speed,
    hub::{C_PORT_CONNECTION, PORT_CONNECTION, PortStatus},
    input,
};
use crate::{
    Clock,
    block::BlockError,
    usb::{
        descriptor::{
            CONFIGURATION_DESCRIPTOR, DEVICE_DESCRIPTOR, DEVICE_TO_HOST, GET_DESCRIPTOR, Interface,
            STRING_DESCRIPTOR, TYPE_CLASS, interfaces, setup,
        },
        storage::{self, UsbDisk, UsbStorage, settle},
    },
};
use alloc::{format, rc::Rc, string::String, vec::Vec};
use core::cell::RefCell;

/// An attached controller, shared by the storage transports on its roots.
pub type Shared = Rc<RefCell<Controller>>;

/// Why a root's device is left to the companion controller.
const COMPANION: &str =
    "full- or low-speed device left to the EHCI companion controller, which has no driver here";
/// Why an enumerated device is left unbound.
const NO_DRIVER: &str =
    "device without a Bulk-Only mass storage, boot keyboard or boot mouse interface, or hub";
/// The hub class, and the hub descriptor's type.
const HUB_CLASS: u8 = 9;
const HUB_DESCRIPTOR: u8 = 0x29;
/// Linux `hub_port_debounce`: sample every 25 ms until the connection held
/// 100 ms, for at most 2 s.
const DEBOUNCE_STEP_US: u64 = 25_000;
const DEBOUNCE_STABLE_US: u64 = 100_000;
const DEBOUNCE_TIMEOUT_US: u64 = 2_000_000;

/// What enumeration found on one controller's roots.
#[derive(Default)]
pub struct Found {
    pub storage: Vec<UsbStorage>,
    pub disks: Vec<UsbDisk>,
    /// Devices left without a driver here, and why.
    pub unused: Vec<(String, &'static str)>,
    pub failures: Vec<(String, BlockError)>,
    /// Boot keyboards and mice.
    pub inputs: Vec<input::BootInput>,
    /// Bulk-Only interfaces configured and waiting out their settle time:
    /// see [`enumerate_unsettled`].
    pub(crate) settling: Vec<settle::Settling>,
}

impl Found {
    /// The Bulk-Only interfaces still settling, for a caller that waits
    /// once for those of every controller it started ([`settle::settle`]).
    pub fn take_settling(&mut self) -> Vec<settle::Settling> {
        core::mem::take(&mut self.settling)
    }
}

/// A root's device, reset, addressed and described.
pub(in crate::usb) struct Addressed {
    pub id: DeviceId,
    pub descriptor: [u8; 18],
}

/// Linux `hub_port_init` for a high-speed root device: reset, SET_ADDRESS to
/// the lowest free address, then the device descriptor. A failure after the
/// reset disables the root, so its address0 owner never blocks another.
pub(in crate::usb) fn address(controller: &Shared, port: u8) -> Result<Addressed, Error> {
    let mut c = controller.borrow_mut();
    let id = c.reset_port(port)?;
    match describe(&mut c, &id) {
        Ok(descriptor) => Ok(Addressed { id, descriptor }),
        Err(error) => {
            let _ = c.disable_port(&id);
            Err(error)
        }
    }
}

/// [`address`] for the device on `port` of `hub`: the hub port's reset,
/// then the same steps. None when the port has nothing connected after the
/// reset. A failure after the reset disables the hub port.
pub(in crate::usb) fn address_child(
    controller: &Shared,
    hub: &DeviceId,
    port: u8,
) -> Result<Option<Addressed>, Error> {
    let mut c = controller.borrow_mut();
    let Some(id) = c.reset_hub_port(hub, port)? else {
        return Ok(None);
    };
    match describe(&mut c, &id) {
        Ok(descriptor) => Ok(Some(Addressed { id, descriptor })),
        Err(error) => {
            c.disable_hub_port(hub, port)?;
            Err(error)
        }
    }
}

/// Reset `id`'s port, on the root or its hub, and address the device again,
/// as `usb_reset_device` does.
pub(in crate::usb) fn readdress(controller: &Shared, id: &DeviceId) -> Result<Addressed, Error> {
    // A root device's port needs no live identity: its reset mints one.
    let hub = if id.is_root() {
        None
    } else {
        controller.borrow().hub_port_of(id)?
    };
    match hub {
        None => address(controller, id.root_port()),
        Some((hub, port)) => address_child(controller, &hub, port)?.ok_or(Error::Stale),
    }
}

/// SET_ADDRESS to the lowest free address, then the device descriptor. A
/// device below high speed gives its EP0 packet size in the descriptor's
/// first eight bytes first, as Linux's old enumeration scheme reads it.
fn describe(c: &mut Controller, id: &DeviceId) -> Result<[u8; 18], Error> {
    let address = c.unused_address().ok_or(Error::Exhausted)?;
    c.set_address(id, address)?;
    let request = |length| {
        setup(
            DEVICE_TO_HOST,
            GET_DESCRIPTOR,
            u16::from(DEVICE_DESCRIPTOR) << 8,
            0,
            length,
        )
    };
    if c.speed(id)? != Speed::High {
        let head = c.control_in(id, request(8))?;
        if head.len() != 8 || head[1] != DEVICE_DESCRIPTOR {
            return Err(Error::Invalid);
        }
        c.set_control_packet(id, head[7])?;
    }
    let descriptor: [u8; 18] = c
        .control_in(id, request(18))?
        .try_into()
        .map_err(|_| Error::Invalid)?;
    if descriptor[0] != 18 || descriptor[1] != DEVICE_DESCRIPTOR || descriptor[17] == 0 {
        return Err(Error::Invalid);
    }
    Ok(descriptor)
}

/// The configuration descriptor at `index`, whole.
pub(in crate::usb) fn configuration(
    controller: &mut Controller,
    id: &DeviceId,
    index: u8,
) -> Result<Vec<u8>, Error> {
    let value = u16::from(CONFIGURATION_DESCRIPTOR) << 8 | u16::from(index);
    let header = controller.control_in(id, setup(DEVICE_TO_HOST, GET_DESCRIPTOR, value, 0, 9))?;
    if header.len() != 9 || header[1] != CONFIGURATION_DESCRIPTOR {
        return Err(Error::Invalid);
    }
    let length = u16::from_le_bytes([header[2], header[3]]);
    let bytes =
        controller.control_in(id, setup(DEVICE_TO_HOST, GET_DESCRIPTOR, value, 0, length))?;
    if bytes.len() != usize::from(length) || bytes.get(..9) != Some(header.as_slice()) {
        return Err(Error::Invalid);
    }
    Ok(bytes)
}

/// String descriptor `index` in the device's first language, as Linux
/// `usb_string` reads it; `None` for index zero or an unreadable string.
fn string(controller: &mut Controller, id: &DeviceId, index: u8) -> Option<String> {
    if index == 0 {
        return None;
    }
    let read = |controller: &mut Controller, index: u8, language: u16| {
        let value = u16::from(STRING_DESCRIPTOR) << 8 | u16::from(index);
        controller
            .control_in(
                id,
                setup(DEVICE_TO_HOST, GET_DESCRIPTOR, value, language, 255),
            )
            .ok()
            .filter(|bytes| bytes.len() >= 2 && bytes[1] == STRING_DESCRIPTOR)
    };
    let languages = read(controller, 0, 0)?;
    let language = u16::from_le_bytes([*languages.get(2)?, *languages.get(3)?]);
    let bytes = read(controller, index, language)?;
    let length = usize::from(bytes[0]).min(bytes.len());
    let units = bytes[2..length]
        .as_chunks::<2>()
        .0
        .iter()
        .map(|unit| u16::from_le_bytes(*unit));
    let text: String = char::decode_utf16(units)
        .map(|c| c.unwrap_or(char::REPLACEMENT_CHARACTER))
        .collect();
    (!text.is_empty()).then_some(text)
}

/// Bring up every connected root of `controller` and bind its storage,
/// waiting out the Bulk-Only settle time.
pub fn enumerate(controller: &Shared, location: &str, clock: Clock) -> Found {
    let mut found = enumerate_unsettled(controller, location, clock);
    let mut settling = found.take_settling();
    let settled = settle::settle(&mut settling, None, true);
    found.storage.extend(settled.storage);
    found.disks.extend(settled.disks);
    found.failures.extend(settled.failures);
    found
}

/// [`enumerate`] with the Bulk-Only interfaces left settling, each from
/// its own configuration, for the caller to discover with every other
/// controller's ([`settle::settle`]), so the waits overlap.
pub fn enumerate_unsettled(controller: &Shared, location: &str, clock: Clock) -> Found {
    let mut found = Found::default();
    let ports = controller.borrow().ports();
    for port in 1..=ports {
        let at = format!("{location} port {port}");
        match address(controller, port) {
            Ok(addressed) => bind(controller, addressed, at, clock, 0, &mut found),
            // Nothing is connected.
            Err(Error::Stale) => {}
            Err(Error::Unsupported(
                "EHCI FS/LS companion traffic" | "EHCI companion-owned root",
            )) => found.unused.push((at, COMPANION)),
            Err(error) => found.failures.push((at, storage::ehci_error(error))),
        }
    }
    found
}

/// What a device's configuration is bound to.
enum Prepared {
    /// With the end of its settle time, taken at its configuration.
    Storage(
        Interface,
        Option<String>,
        Vec<super::BulkPipe>,
        Option<crate::Deadline>,
    ),
    Input(Vec<Interface>),
}

/// Bind the first Bulk-Only SCSI interface of configuration zero, else its
/// boot keyboard and mouse interfaces, or enumerate a hub's ports. `tier` counts the hubs above the device.
fn bind(
    controller: &Shared,
    addressed: Addressed,
    at: String,
    clock: Clock,
    tier: usize,
    found: &mut Found,
) {
    if addressed.descriptor[4] == HUB_CLASS {
        return hub(controller, addressed.id, &at, clock, tier, found);
    }
    let Addressed { id, descriptor } = addressed;
    let prepared = (|| {
        let mut c = controller.borrow_mut();
        let configuration = configuration(&mut c, &id, 0)?;
        let found = interfaces(&configuration);
        let Some(setting) = found
            .iter()
            .find(|i| i.alternate == 0 && storage::bulk_only(i))
            .cloned()
        else {
            let boot: Vec<_> = found.into_iter().filter(input::is_boot).collect();
            if boot.is_empty() {
                return Ok(None);
            }
            c.configure(&id, 0)?;
            return Ok(Some(Prepared::Input(boot)));
        };
        let serial = string(&mut c, &id, descriptor[16]);
        let pipes = c.configure(&id, 0)?;
        let configured = settle::configured(clock);
        Ok(Some(Prepared::Storage(setting, serial, pipes, configured)))
    })();
    match prepared {
        Ok(Some(Prepared::Input(interfaces))) => {
            for interface in interfaces {
                let at = format!("{at} interface {}", interface.number);
                let bound = input::BootInput::new(
                    at.clone(),
                    controller.clone(),
                    id.clone(),
                    &interface,
                    clock,
                );
                match bound {
                    Ok(bound) => found.inputs.push(bound),
                    Err(error) => found.failures.push((at, storage::ehci_error(error))),
                }
            }
        }
        Ok(Some(Prepared::Storage(setting, serial, pipes, configured))) => {
            let serial = serial.unwrap_or_else(|| at.clone());
            let io = storage::EhciBulk::new(controller.clone(), id, pipes, clock, descriptor);
            let interface = settle::Configured::Ehci {
                io,
                setting,
                serial,
            };
            found
                .settling
                .push(settle::Settling::new(configured, at, interface));
        }
        Ok(None) => found.unused.push((at, NO_DRIVER)),
        Err(error) => found.failures.push((at, storage::ehci_error(error))),
    }
}

/// Linux `hub_activate` and `hub_events` for one hub: configure it, power
/// its ports, debounce the connected ones together, then reset and
/// enumerate each in turn.
fn hub(controller: &Shared, id: DeviceId, at: &str, clock: Clock, tier: usize, found: &mut Found) {
    let ports = (|| {
        let mut c = controller.borrow_mut();
        c.configure(&id, 0)?;
        let request = setup(
            DEVICE_TO_HOST | TYPE_CLASS,
            GET_DESCRIPTOR,
            u16::from(HUB_DESCRIPTOR) << 8,
            0,
            71,
        );
        let descriptor = c.control_in(&id, request)?;
        if descriptor.len() < 7 || descriptor[1] != HUB_DESCRIPTOR || descriptor[2] == 0 {
            return Err(Error::Invalid);
        }
        let ports = descriptor[2];
        for port in 1..=ports {
            c.power_hub_port(&id, port)?;
        }
        // Linux `hub_power_on_good_delay`: at least 100 ms below a root.
        c.wait_minimum((u64::from(descriptor[5]) * 2_000).max(100_000))?;
        Ok(ports)
    })();
    let ports = match ports {
        Ok(ports) => ports,
        Err(error) => {
            found
                .failures
                .push((format!("{at} hub"), storage::ehci_error(error)));
            return;
        }
    };
    let connected = match debounce(&mut controller.borrow_mut(), &id, ports) {
        Ok(connected) => connected,
        Err(error) => {
            found
                .failures
                .push((format!("{at} hub"), storage::ehci_error(error)));
            return;
        }
    };
    for port in connected {
        let at = format!("{at} hub port {port}");
        match address_child(controller, &id, port) {
            Ok(Some(addressed)) => bind(controller, addressed, at, clock, tier + 1, found),
            Ok(None) => {}
            Err(error) => found.failures.push((at, storage::ehci_error(error))),
        }
    }
}

/// Linux `hub_port_debounce` for every port of `hub` at once: the ports
/// whose connection held [`DEBOUNCE_STABLE_US`], each connection change
/// acknowledged as it is seen. A port that never settles is left alone.
fn debounce(c: &mut Controller, hub: &DeviceId, ports: u8) -> Result<Vec<u8>, Error> {
    // How long each port's connection has held; None while disconnected.
    let mut held = alloc::vec![None::<u64>; usize::from(ports)];
    let (mut waited, mut step) = (0, 0);
    loop {
        for port in 1..=ports {
            let PortStatus { status, change } = c.hub_port_status(hub, port)?;
            let connected = status & PORT_CONNECTION != 0;
            let held = &mut held[usize::from(port - 1)];
            // wPortChange bit 0 is C_PORT_CONNECTION.
            if change & 1 != 0 {
                c.clear_hub_port_change(hub, port, C_PORT_CONNECTION)?;
                *held = connected.then_some(0);
            } else {
                *held = connected.then(|| held.map_or(0, |time| time + step));
            }
        }
        if held
            .iter()
            .flatten()
            .all(|time| *time >= DEBOUNCE_STABLE_US)
            || waited >= DEBOUNCE_TIMEOUT_US
        {
            break;
        }
        c.wait_minimum(DEBOUNCE_STEP_US)?;
        waited += DEBOUNCE_STEP_US;
        step = DEBOUNCE_STEP_US;
    }
    Ok((1..=ports)
        .filter(|port| held[usize::from(port - 1)].is_some_and(|time| time >= DEBOUNCE_STABLE_US))
        .collect())
}
