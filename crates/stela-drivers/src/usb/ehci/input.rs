//! Boot-protocol keyboards and mice on an EHCI controller, as Linux usbkbd
//! and usbmouse drive them: the interface is switched to the boot protocol,
//! its idle rate set to zero, and its interrupt IN endpoint read for the
//! HID 1.11 Appendix B reports. Behind an Intel rate-matching hub these are
//! the low- and full-speed split transactions the controller runs through
//! the hub's transaction translator.
//!
//! The controller runs one transfer at a time. A waiting interrupt transfer
//! gives way to any control or bulk transfer and is submitted again at the
//! next poll; a second keyboard or mouse gets its turn when the first one's
//! transfer gives way after eight of its periods. Neither loses a report:
//! the device holds a report until it is read, and a split transaction under
//! way finishes before its transfer is cancelled.
use super::{Controller, DeviceId, Error, InterruptPipe, TransferId, host::Shared};
use crate::{
    Clock,
    block::BlockError,
    input::Input,
    usb::{
        descriptor::{Interface, setup},
        hid::{Translator, events::Event},
    },
};
use alloc::{string::String, vec::Vec};

const SET_REPORT: u8 = 0x09;
const SET_IDLE: u8 = 0x0a;
const SET_PROTOCOL: u8 = 0x0b;
/// Class request to an interface, host to device.
const TO_INTERFACE: u8 = 0x21;
/// The boot interface subclass and its keyboard and mouse protocols.
pub const BOOT: u8 = 1;
pub const KEYBOARD: u8 = 1;
pub const MOUSE: u8 = 2;
/// Boot reports fit eight bytes (HID 1.11 Appendix B).
const BOOT_REPORT_BYTES: u16 = 8;
/// A waiting transfer is cancelled and submitted again after eight of its
/// endpoint's periods, at least this long, so a second keyboard or mouse on
/// the controller gets its turn, and well inside its timeout, which would
/// fail the controller.
const MINIMUM_WINDOW_US: u64 = 32_000;
const TRANSFER_TIMEOUT_US: u64 = 5_000_000;
/// Reports one poll reads at most, so a chattering device cannot hold it.
const REPORTS_PER_POLL: usize = 4;

/// HID 1.11 Appendix B.1: the boot keyboard's report descriptor.
pub(crate) const KEYBOARD_REPORT: [u8; 63] = [
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, 0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, 0x15, 0x00, 0x25, 0x01,
    0x75, 0x01, 0x95, 0x08, 0x81, 0x02, 0x95, 0x01, 0x75, 0x08, 0x81, 0x01, 0x95, 0x05, 0x75, 0x01,
    0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, 0x95, 0x01, 0x75, 0x03, 0x91, 0x01, 0x95, 0x06,
    0x75, 0x08, 0x15, 0x00, 0x25, 0x65, 0x05, 0x07, 0x19, 0x00, 0x29, 0x65, 0x81, 0x00, 0xc0,
];
/// HID 1.11 Appendix B.2: the boot mouse's report descriptor.
pub(crate) const MOUSE_REPORT: [u8; 50] = [
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, 0x09, 0x01, 0xa1, 0x00, 0x05, 0x09, 0x19, 0x01, 0x29, 0x03,
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, 0x95, 0x01, 0x75, 0x05, 0x81, 0x01,
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x02, 0x81, 0x06,
    0xc0, 0xc0,
];

/// Whether `interface` is a boot keyboard or mouse this driver reads.
pub fn is_boot(interface: &Interface) -> bool {
    interface.alternate == 0
        && interface.class == crate::usb::hid::CLASS
        && interface.subclass == BOOT
        && matches!(interface.protocol, KEYBOARD | MOUSE)
        && interface
            .endpoints
            .iter()
            .any(|endpoint| endpoint.kind() == 3 && endpoint.is_in())
}

/// One boot keyboard or mouse interface.
pub struct BootInput {
    /// The interface's location, for the boot log.
    pub name: String,
    controller: Shared,
    id: DeviceId,
    interface: u8,
    pipe: InterruptPipe,
    /// The bytes one read asks for: the endpoint's packet, at most a boot
    /// report.
    bytes: usize,
    translator: Translator,
    /// The waiting transfer and when it was submitted.
    pending: Option<(TransferId, u64)>,
    /// How long a transfer waits before it gives way.
    window_us: u64,
    clock: Clock,
    failed: bool,
}

impl BootInput {
    /// Bind `interface` of the configured device `id`: the boot protocol,
    /// then idle rate zero, which a device may refuse, as Linux ignores.
    pub fn new(
        name: String,
        controller: Shared,
        id: DeviceId,
        interface: &Interface,
        clock: Clock,
    ) -> Result<Self, Error> {
        let endpoint = interface
            .endpoints
            .iter()
            .find(|endpoint| endpoint.kind() == 3 && endpoint.is_in())
            .ok_or(Error::Invalid)?;
        let report: &[u8] = if interface.protocol == KEYBOARD {
            &KEYBOARD_REPORT
        } else {
            &MOUSE_REPORT
        };
        let translator = Translator::describe(report).map_err(|_| Error::Invalid)?;
        let (pipe, pipe_interval) = {
            let mut c = controller.borrow_mut();
            let pipe = c
                .interrupt_pipes(&id)?
                .into_iter()
                .find(|pipe| pipe.endpoint_address() == endpoint.address)
                .ok_or(Error::Invalid)?;
            let interval = pipe.interval_microframes();
            let index = u16::from(interface.number);
            c.control_out(&id, setup(TO_INTERFACE, SET_PROTOCOL, 0, index, 0), &[])?;
            match c.control_out(&id, setup(TO_INTERFACE, SET_IDLE, 0, index, 0), &[]) {
                Err(error) if !error.stalled() => return Err(error),
                _ => {}
            }
            (pipe, interval)
        };
        Ok(Self {
            name,
            controller,
            id,
            interface: interface.number,
            pipe,
            bytes: usize::from((endpoint.max_packet & 0x7ff).min(BOOT_REPORT_BYTES)),
            translator,
            pending: None,
            window_us: (u64::from(pipe_interval) * 125 * 8).max(MINIMUM_WINDOW_US),
            clock,
            failed: false,
        })
    }

    /// The input changes of the reports that arrived since the last poll.
    /// A controller busy with another transfer is read at a later poll.
    pub fn poll(&mut self) -> Vec<Input> {
        let mut inputs = Vec::new();
        if self.failed {
            return inputs;
        }
        let controller = self.controller.clone();
        let mut c = controller.borrow_mut();
        for _ in 0..REPORTS_PER_POLL {
            match self.read(&mut c) {
                Ok(Some(report)) => {
                    let report = &report[..report.len().min(self.translator.input_bytes())];
                    let mut events: Vec<Event> = Vec::new();
                    self.translator.translate(report, &mut events, &mut inputs);
                }
                Ok(None) => break,
                Err(_) => {
                    self.failed = true;
                    inputs.extend(self.translator.release().0);
                    break;
                }
            }
        }
        inputs
    }

    /// One report, if one came in; the transfer that waits for it is
    /// submitted, renewed or resubmitted after a preemption as needed.
    fn read(&mut self, c: &mut Controller) -> Result<Option<Vec<u8>>, Error> {
        let now = self.clock.now_us();
        let (ticket, submitted) = match self.pending.take() {
            Some(pending) => pending,
            None => match c.submit_interrupt_in(&self.pipe, self.bytes, TRANSFER_TIMEOUT_US) {
                Ok(ticket) => (ticket, now),
                // Another interrupt transfer has its turn.
                Err(Error::Busy) => return Ok(None),
                Err(error) => return Err(error),
            },
        };
        match c.poll(&ticket) {
            Ok(Some(completion)) => Ok(Some(completion.data)),
            // Give way to another interrupt endpoint on the controller.
            Ok(None) if now.saturating_sub(submitted) >= self.window_us => Ok(c
                .cancel_interrupt(&ticket)?
                .map(|completion| completion.data)),
            Ok(None) => {
                self.pending = Some((ticket, submitted));
                Ok(None)
            }
            // Preempted, while the device is still there.
            Err(Error::Stale) if c.validate(&self.id).is_ok() => Ok(None),
            Err(error) if error.stalled() => {
                c.clear_interrupt_halt(&self.pipe)?;
                Ok(None)
            }
            Err(error) => Err(error),
        }
    }

    /// Show `leds` (input LED codes as bits) on a keyboard, through its
    /// output report, as Linux `usb_kbd_event` sends it.
    pub fn set_leds(&mut self, leds: u16) -> Result<(), BlockError> {
        let Some((id, report)) = self.translator.led_report(leds) else {
            return Ok(());
        };
        let length = u16::try_from(report.len())
            .map_err(|_| BlockError::Unsupported("HID LED report too long"))?;
        self.controller
            .borrow_mut()
            .control_out(
                &self.id,
                setup(
                    TO_INTERFACE,
                    SET_REPORT,
                    0x0200 | u16::from(id),
                    u16::from(self.interface),
                    length,
                ),
                &report,
            )
            .map_err(crate::usb::storage::ehci_error)
    }

    /// The last report the device sent, as it arrived.
    pub fn last_report(&self) -> &[u8] {
        self.translator.last_report()
    }

    /// Whether the device stopped and is no longer read.
    pub fn failed(&self) -> bool {
        self.failed
    }
}
