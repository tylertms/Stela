//! A native full-speed USB keyboard, absolute pointer and mouse: one
//! composite device with three HID 1.11 interfaces, as a host port's
//! [`Function`].
//!
//! Evidence: USB 2.0 chapter 9 (standard requests and device states) and
//! Device Class Definition for HID 1.11 (class descriptors, requests,
//! boot protocol, Appendix B.1 keyboard and B.2 mouse reports). Interface
//! 0 is a boot keyboard (subclass 1, protocol 1) whose report is the boot
//! format in both protocols: modifiers, a reserved byte and a six-key
//! array, with an output report of five LEDs. Interface 1 is a
//! report-protocol pointer with three buttons, X and Y absolute over
//! 0..=0x7fff and a relative wheel: host tablets, whose positions macOS
//! takes as they are. Interface 2 is a boot mouse (subclass 1, protocol 2)
//! with three buttons and relative X, Y and wheel over -127..=127: host
//! mice, whose counts macOS accelerates as it does a Mac's own mouse. Its
//! report protocol report is the boot report followed by the wheel byte;
//! in the boot protocol it sends the three boot bytes. Motion beyond one
//! report's range splits across reports, and queued motion of unchanged
//! buttons coalesces. Usage values are those of HID Usage Tables 1.4
//! (Keyboard/Keypad page 07h); host sources translate their own codes into
//! them.
//!
//! The device identifies itself as Apple's USB Pro Keyboard (ANSI), vendor
//! 0x05AC and product 0x0204: macOS knows the layout of an Apple keyboard by
//! its IDs, and asks for any other's with Keyboard Setup Assistant on first
//! boot. Its keyboard ID, 0xCC, is outside the ranges where macOS makes F12
//! eject, as an unlisted Apple product's 0xC6 is not, and the report has no
//! Fn usage, so the F-keys stay F-keys. It is a self-powered device without
//! remote wakeup, so a host never suspends it waiting on a wake it cannot
//! signal. Requests a full-speed HID device need not support (device
//! qualifier, other-speed configuration, test mode, remote wakeup,
//! SYNCH_FRAME, alternate settings) answer with a request-error STALL.
//! Transactions to an endpoint of an unconfigured device also STALL: the
//! bus boundary has no no-response outcome.
//!
//! Each input change queues one report. Queues are bounded; when one is
//! full the newest report replaces its last entry, so the final state
//! always reaches the host. The device reports changes alone: a nonzero
//! SET_IDLE rate STALLs, which HID 1.11 7.2.4 lets a device do for a rate it
//! does not keep, as repeating an unchanged report tells the host nothing
//! and wakes its driver every time.
use super::{BusPeer, Function, Reply, Token, control::Stall};
use crate::Result;
use alloc::{collections::VecDeque, vec::Vec};
use stela_contracts::refuse;
use stela_drivers::input::{Input, POINTER_MAX};

const EP0_PACKET: usize = 64;
const KEYBOARD_ENDPOINT: u8 = 0x81;
const POINTER_ENDPOINT: u8 = 0x82;
const MOUSE_ENDPOINT: u8 = 0x83;
const KEYBOARD_REPORT: usize = 8;
const POINTER_REPORT: usize = 7;
const MOUSE_REPORT: usize = 5;
/// The boot mouse report: buttons, X and Y (HID 1.11 B.2).
const BOOT_MOUSE_REPORT: usize = 3;
const INTERFACES: usize = 3;
const KEYBOARD: usize = 0;
const POINTER: usize = 1;
const MOUSE: usize = 2;
/// One relative report's range per axis.
const MOTION_MAX: i16 = 127;
const QUEUE: usize = 64;
/// Keys the boot report reports before it reports rollover.
const ROLLOVER: usize = 6;
const ERROR_ROLLOVER: u8 = 1;
/// Keys held at once that the device tracks.
const MAX_HELD: usize = 32;

const GET_STATUS: u8 = 0;
const CLEAR_FEATURE: u8 = 1;
const SET_FEATURE: u8 = 3;
const SET_ADDRESS: u8 = 5;
const GET_DESCRIPTOR: u8 = 6;
const GET_CONFIGURATION: u8 = 8;
const SET_CONFIGURATION: u8 = 9;
const GET_INTERFACE: u8 = 10;
const SET_INTERFACE: u8 = 11;
const GET_REPORT: u8 = 1;
const GET_IDLE: u8 = 2;
const GET_PROTOCOL: u8 = 3;
const SET_REPORT: u8 = 9;
const SET_IDLE: u8 = 0x0a;
const SET_PROTOCOL: u8 = 0x0b;
const ENDPOINT_HALT: u16 = 0;
const DEVICE: u8 = 1;
const CONFIGURATION: u8 = 2;
const STRING: u8 = 3;
const HID: u8 = 0x21;
const REPORT: u8 = 0x22;
const INPUT_REPORT: u8 = 1;
const OUTPUT_REPORT: u8 = 2;
const BOOT_PROTOCOL: u8 = 0;
const REPORT_PROTOCOL: u8 = 1;
const ENGLISH: u16 = 0x0409;

const DEVICE_DESCRIPTOR: [u8; 18] = [
    18,
    DEVICE,
    0x00,
    0x02, // USB 2.0
    0,
    0,
    0, // class per interface
    EP0_PACKET as u8,
    0xac,
    0x05, // vendor 0x05ac
    0x04,
    0x02, // product 0x0204
    0x00,
    0x01, // release 1.00
    1,
    2,
    0, // manufacturer, product, no serial
    1,
];

const KEYBOARD_REPORT_DESCRIPTOR: [u8; 64] = [
    0x05, 0x01, 0x09, 0x06, 0xa1, 0x01, // Generic Desktop, Keyboard, Application
    0x05, 0x07, 0x19, 0xe0, 0x29, 0xe7, // modifier usages E0h..E7h
    0x15, 0x00, 0x25, 0x01, 0x75, 0x01, 0x95, 0x08, 0x81, 0x02, // 8 bits
    0x95, 0x01, 0x75, 0x08, 0x81, 0x01, // reserved byte
    0x95, 0x05, 0x75, 0x01, 0x05, 0x08, 0x19, 0x01, 0x29, 0x05, 0x91, 0x02, // five LEDs
    0x95, 0x01, 0x75, 0x03, 0x91, 0x01, // LED padding
    0x95, 0x06, 0x75, 0x08, 0x15, 0x00, 0x26, 0xdd, 0x00, // six keys 0..DDh
    0x05, 0x07, 0x19, 0x00, 0x29, 0xdd, 0x81, 0x00, // key array
    0xc0,
];

const POINTER_REPORT_DESCRIPTOR: [u8; 89] = [
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, // Generic Desktop, Mouse, Application
    0x09, 0x01, 0xa1, 0x00, // Pointer, Physical
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03, // buttons 1..3
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, //
    0x95, 0x01, 0x75, 0x05, 0x81, 0x01, // padding
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, // X, Y
    0x15, 0x00, 0x26, 0xff, 0x7f, 0x35, 0x00, 0x46, 0xff, 0x7f, //
    0x75, 0x10, 0x95, 0x02, 0x81, 0x02, // absolute
    0x05, 0x01, 0x09, 0x38, // wheel
    0x15, 0x81, 0x25, 0x7f, 0x35, 0x00, 0x45, 0x00, //
    0x75, 0x08, 0x95, 0x01, 0x81, 0x06, // relative
    0x05, 0x0c, 0x0a, 0x38, 0x02, // AC Pan
    0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x01, 0x81, 0x06, // relative
    0xc0, 0xc0,
];

/// The boot mouse report (HID 1.11 B.2) with wheel and AC Pan bytes after it.
const MOUSE_REPORT_DESCRIPTOR: [u8; 61] = [
    0x05, 0x01, 0x09, 0x02, 0xa1, 0x01, // Generic Desktop, Mouse, Application
    0x09, 0x01, 0xa1, 0x00, // Pointer, Physical
    0x05, 0x09, 0x19, 0x01, 0x29, 0x03, // buttons 1..3
    0x15, 0x00, 0x25, 0x01, 0x95, 0x03, 0x75, 0x01, 0x81, 0x02, //
    0x95, 0x01, 0x75, 0x05, 0x81, 0x01, // padding
    0x05, 0x01, 0x09, 0x30, 0x09, 0x31, 0x09, 0x38, // X, Y, wheel
    0x15, 0x81, 0x25, 0x7f, 0x75, 0x08, 0x95, 0x03, 0x81, 0x06, // relative
    0x05, 0x0c, 0x0a, 0x38, 0x02, // AC Pan
    0x95, 0x01, 0x81, 0x06, // relative
    0xc0, 0xc0,
];

const fn hid_descriptor(report: usize) -> [u8; 9] {
    [
        9,
        HID,
        0x11,
        0x01, // HID 1.11
        0,
        1,
        REPORT,
        report as u8,
        (report >> 8) as u8,
    ]
}

/// Each interface's interface, HID and endpoint descriptors.
const INTERFACE_BYTES: usize = 9 + 9 + 7;
const CONFIGURATION_BYTES: usize = 9 + INTERFACES * INTERFACE_BYTES;

fn configuration_descriptor() -> [u8; CONFIGURATION_BYTES] {
    let mut bytes = [0; CONFIGURATION_BYTES];
    let parts: [&[u8]; 10] = [
        &[
            9,
            CONFIGURATION,
            CONFIGURATION_BYTES as u8,
            0,
            INTERFACES as u8,
            1,    // bConfigurationValue
            0,    // no string
            0xc0, // self-powered, no remote wakeup
            0,
        ],
        &[9, 4, KEYBOARD as u8, 0, 1, 3, 1, 1, 0], // boot keyboard
        &hid_descriptor(KEYBOARD_REPORT_DESCRIPTOR.len()),
        &[7, 5, KEYBOARD_ENDPOINT, 3, KEYBOARD_REPORT as u8, 0, 1],
        &[9, 4, POINTER as u8, 0, 1, 3, 0, 0, 0], // report-protocol pointer
        &hid_descriptor(POINTER_REPORT_DESCRIPTOR.len()),
        &[7, 5, POINTER_ENDPOINT, 3, POINTER_REPORT as u8, 0, 1],
        &[9, 4, MOUSE as u8, 0, 1, 3, 1, 2, 0], // boot mouse
        &hid_descriptor(MOUSE_REPORT_DESCRIPTOR.len()),
        &[7, 5, MOUSE_ENDPOINT, 3, MOUSE_REPORT as u8, 0, 1],
    ];
    let mut at = 0;
    for part in parts {
        bytes[at..at + part.len()].copy_from_slice(part);
        at += part.len();
    }
    bytes
}

fn string(text: &str) -> Vec<u8> {
    let mut bytes = alloc::vec![0, STRING];
    bytes.extend(text.encode_utf16().flat_map(u16::to_le_bytes));
    bytes[0] = bytes.len() as u8;
    bytes
}

#[derive(Clone)]
struct Interface {
    protocol: u8,
    halted: bool,
    queue: VecDeque<Vec<u8>>,
}

impl Interface {
    fn new() -> Self {
        Self {
            protocol: REPORT_PROTOCOL,
            halted: false,
            queue: VecDeque::new(),
        }
    }

    fn push(&mut self, report: Vec<u8>) {
        if self.queue.len() == QUEUE {
            self.queue.pop_back();
        }
        self.queue.push_back(report);
    }
}

#[derive(Clone)]
enum Control {
    Idle,
    /// IN data stage: the reply, the bytes sent and whether the final
    /// (short or zero-length) packet has gone.
    In {
        data: Vec<u8>,
        sent: usize,
        length: usize,
        done: bool,
    },
    Out {
        setup: [u8; 8],
        data: Vec<u8>,
    },
    /// The status stage of a request that succeeded, with its deferred
    /// effect (SET_ADDRESS takes effect after its status stage).
    Status(Option<u8>),
    Stalled,
}

/// The device's bus state and its host's view of keys and pointer.
#[derive(Clone)]
pub struct Peripheral {
    address: u8,
    configuration: u8,
    control: Control,
    interfaces: [Interface; INTERFACES],
    /// The LEDs the host last set: Num, Caps, Scroll, Compose, Kana lock
    /// as bits 0..4.
    leds: u8,
    modifiers: u8,
    held: Vec<u8>,
    buttons: u8,
    x: u16,
    y: u16,
    mouse_buttons: u8,
}

/// The interfaces after configuration.
fn interfaces() -> [Interface; INTERFACES] {
    core::array::from_fn(|_| Interface::new())
}

impl Default for Peripheral {
    fn default() -> Self {
        Self::new()
    }
}

impl Peripheral {
    pub fn new() -> Self {
        Self {
            address: 0,
            configuration: 0,
            control: Control::Idle,
            interfaces: interfaces(),
            leds: 0,
            modifiers: 0,
            held: Vec::new(),
            buttons: 0,
            x: 0,
            y: 0,
            mouse_buttons: 0,
        }
    }

    /// The keyboard LEDs the host last set, Num Lock in bit 0 through Kana
    /// in bit 4, as the boot keyboard's output report carries them.
    pub fn leds(&self) -> u8 {
        self.leds
    }

    /// Applies one input change, queuing the report it produces. Reports
    /// queue only while configured; a host that has not configured the
    /// device reads the current state with GET_REPORT.
    pub fn input(&mut self, input: Input) -> Result<()> {
        match input {
            Input::Key { usage, pressed } => {
                if let 0xe0..=0xe7 = usage {
                    let bit = 1 << (usage - 0xe0);
                    self.modifiers = if pressed {
                        self.modifiers | bit
                    } else {
                        self.modifiers & !bit
                    };
                } else if (0x04..=0xdd).contains(&usage) {
                    let position = self.held.iter().position(|&held| held == usage);
                    match (pressed, position) {
                        (true, None) if self.held.len() == MAX_HELD => {
                            return Err(refuse!("too many keys held at once"));
                        }
                        (true, None) => self.held.push(usage),
                        (false, Some(n)) => {
                            self.held.remove(n);
                        }
                        _ => return Ok(()),
                    }
                } else {
                    return Err(refuse!("not a Keyboard/Keypad usage this keyboard reports"));
                }
                let report = self.keyboard_report().to_vec();
                self.queue(KEYBOARD, report);
            }
            Input::Move { x, y } => {
                if x > POINTER_MAX || y > POINTER_MAX {
                    return Err(refuse!("pointer position outside its logical range"));
                }
                (self.x, self.y) = (x, y);
                // Motion coalesces into a queued report of the same buttons.
                let report = self.pointer_report(0, 0);
                let queue = &mut self.interfaces[POINTER].queue;
                match queue.back_mut() {
                    Some(last) if last[0] == report[0] && last[5] == 0 && last[6] == 0 => {
                        last.copy_from_slice(&report)
                    }
                    _ => self.queue(POINTER, report.to_vec()),
                }
            }
            Input::Buttons(buttons) => {
                if buttons & !7 != 0 {
                    return Err(refuse!("the pointer has three buttons"));
                }
                self.buttons = buttons;
                let report = self.pointer_report(0, 0).to_vec();
                self.queue(POINTER, report);
            }
            Input::Wheel(detents) => {
                if detents == i8::MIN {
                    return Err(refuse!("wheel motion outside its logical range"));
                }
                let report = self.pointer_report(detents, 0).to_vec();
                self.queue(POINTER, report);
            }
            Input::HWheel(detents) => {
                if detents == i8::MIN {
                    return Err(refuse!("wheel motion outside its logical range"));
                }
                let report = self.pointer_report(0, detents).to_vec();
                self.queue(POINTER, report);
            }
            Input::Motion { dx, dy } => self.motion(dx, dy),
            Input::MouseButtons(buttons) => {
                if buttons & !7 != 0 {
                    return Err(refuse!("the mouse has three buttons"));
                }
                self.mouse_buttons = buttons;
                let report = self.mouse_report(0, 0, 0, 0).to_vec();
                self.queue(MOUSE, report);
            }
            Input::MouseWheel(detents) => {
                if detents == i8::MIN {
                    return Err(refuse!("wheel motion outside its logical range"));
                }
                let report = self.mouse_report(0, 0, detents, 0).to_vec();
                self.queue(MOUSE, report);
            }
            Input::MouseHWheel(detents) => {
                if detents == i8::MIN {
                    return Err(refuse!("wheel motion outside its logical range"));
                }
                let report = self.mouse_report(0, 0, 0, detents).to_vec();
                self.queue(MOUSE, report);
            }
        }
        Ok(())
    }

    /// Queues mouse motion: first into a queued report of the same
    /// buttons without wheel motion, as far as its range allows, then in
    /// reports of at most [`MOTION_MAX`] counts per axis.
    fn motion(&mut self, mut dx: i16, mut dy: i16) {
        if self.configuration == 0 {
            return;
        }
        let buttons = self.mouse_buttons;
        let queue = &mut self.interfaces[MOUSE].queue;
        if let Some(last) = queue.back_mut()
            && last[0] == buttons
            && last[3] == 0
            && last[4] == 0
        {
            for (byte, delta) in [(1, &mut dx), (2, &mut dy)] {
                let held = i16::from(last[byte] as i8);
                let merged = (held + *delta).clamp(-MOTION_MAX, MOTION_MAX);
                *delta -= merged - held;
                last[byte] = merged as i8 as u8;
            }
        }
        while dx != 0 || dy != 0 {
            let step_x = dx.clamp(-MOTION_MAX, MOTION_MAX);
            let step_y = dy.clamp(-MOTION_MAX, MOTION_MAX);
            (dx, dy) = (dx - step_x, dy - step_y);
            let report = self.mouse_report(step_x as i8, step_y as i8, 0, 0).to_vec();
            self.interfaces[MOUSE].push(report);
        }
    }

    fn queue(&mut self, interface: usize, report: Vec<u8>) {
        if self.configuration != 0 {
            self.interfaces[interface].push(report);
        }
    }

    fn keyboard_report(&self) -> [u8; KEYBOARD_REPORT] {
        let mut report = [0; KEYBOARD_REPORT];
        report[0] = self.modifiers;
        if self.held.len() > ROLLOVER {
            report[2..].fill(ERROR_ROLLOVER);
        } else {
            report[2..2 + self.held.len()].copy_from_slice(&self.held);
        }
        report
    }

    fn pointer_report(&self, wheel: i8, pan: i8) -> [u8; POINTER_REPORT] {
        let [x0, x1] = self.x.to_le_bytes();
        let [y0, y1] = self.y.to_le_bytes();
        [self.buttons, x0, x1, y0, y1, wheel as u8, pan as u8]
    }

    fn mouse_report(&self, dx: i8, dy: i8, wheel: i8, pan: i8) -> [u8; MOUSE_REPORT] {
        [
            self.mouse_buttons,
            dx as u8,
            dy as u8,
            wheel as u8,
            pan as u8,
        ]
    }

    /// The interface's report now: relative fields report no motion.
    fn current(&self, interface: usize) -> Vec<u8> {
        match interface {
            KEYBOARD => self.keyboard_report().to_vec(),
            POINTER => self.pointer_report(0, 0).to_vec(),
            _ => self.mouse_report(0, 0, 0, 0).to_vec(),
        }
    }

    fn interface_of(endpoint: u8) -> Option<usize> {
        match endpoint {
            KEYBOARD_ENDPOINT => Some(KEYBOARD),
            POINTER_ENDPOINT => Some(POINTER),
            MOUSE_ENDPOINT => Some(MOUSE),
            _ => None,
        }
    }

    /// The boot-subclass interfaces, which take GET/SET_PROTOCOL.
    fn boot(interface: usize) -> bool {
        matches!(interface, KEYBOARD | MOUSE)
    }

    /// Evaluates a request whose data stage, if any, is complete. `Err`
    /// is a request error: the device STALLs the rest of the transfer.
    fn request(&mut self, setup: [u8; 8], data: &[u8]) -> Result<Outcome, Stall> {
        let kind = setup[0];
        let request = setup[1];
        let value = u16::from_le_bytes([setup[2], setup[3]]);
        let index = u16::from_le_bytes([setup[4], setup[5]]);
        let configured = self.configuration != 0;
        let interface = usize::from(index as u8);
        let reply = |bytes: &[u8]| Ok(Outcome::Data(bytes.to_vec()));
        match (kind, request) {
            (0x80, GET_STATUS) => reply(&[1, 0]), // self-powered
            (0x81, GET_STATUS) if configured && interface < INTERFACES => reply(&[0, 0]),
            (0x82, GET_STATUS) => match index as u8 {
                0 | 0x80 => reply(&[0, 0]),
                ep if configured => {
                    let n = Self::interface_of(ep).ok_or(Stall)?;
                    reply(&[u8::from(self.interfaces[n].halted), 0])
                }
                _ => Err(Stall),
            },
            (0x02, CLEAR_FEATURE | SET_FEATURE) if value == ENDPOINT_HALT => match index as u8 {
                0 | 0x80 => Ok(Outcome::Done(None)),
                ep if configured => {
                    let n = Self::interface_of(ep).ok_or(Stall)?;
                    self.interfaces[n].halted = request == SET_FEATURE;
                    Ok(Outcome::Done(None))
                }
                _ => Err(Stall),
            },
            (0x00, SET_ADDRESS) if value <= 127 && index == 0 && !configured => {
                Ok(Outcome::Done(Some(value as u8)))
            }
            (0x80, GET_DESCRIPTOR) => match ((value >> 8) as u8, value as u8) {
                (DEVICE, 0) => reply(&DEVICE_DESCRIPTOR),
                (CONFIGURATION, 0) => reply(&configuration_descriptor()),
                (STRING, 0) => reply(&[4, STRING, 0x09, 0x04]),
                (STRING, 1) if index == ENGLISH => reply(&string("Stela")),
                (STRING, 2) if index == ENGLISH => reply(&string("Stela Keyboard and Pointers")),
                _ => Err(Stall),
            },
            (0x81, GET_DESCRIPTOR) if interface < INTERFACES => {
                match ((value >> 8) as u8, value as u8) {
                    (HID, 0) => {
                        let start = 9 + interface * INTERFACE_BYTES + 9;
                        reply(&configuration_descriptor()[start..start + 9])
                    }
                    (REPORT, 0) => reply(match interface {
                        KEYBOARD => &KEYBOARD_REPORT_DESCRIPTOR[..],
                        POINTER => &POINTER_REPORT_DESCRIPTOR[..],
                        _ => &MOUSE_REPORT_DESCRIPTOR[..],
                    }),
                    _ => Err(Stall),
                }
            }
            (0x80, GET_CONFIGURATION) => reply(&[self.configuration]),
            (0x00, SET_CONFIGURATION) if value <= 1 && self.address != 0 => {
                self.configuration = value as u8;
                self.interfaces = interfaces();
                Ok(Outcome::Done(None))
            }
            (0x81, GET_INTERFACE) if configured && interface < INTERFACES => reply(&[0]),
            (0x01, SET_INTERFACE) if configured && interface < INTERFACES && value == 0 => {
                self.interfaces[interface].halted = false;
                Ok(Outcome::Done(None))
            }
            (0xa1, GET_REPORT)
                if interface < INTERFACES && value == u16::from(INPUT_REPORT) << 8 =>
            {
                reply(&self.report(interface, self.current(interface)))
            }
            (0x21, SET_REPORT)
                if interface == 0 && value == u16::from(OUTPUT_REPORT) << 8 && data.len() == 1 =>
            {
                self.leds = data[0] & 0x1f;
                Ok(Outcome::Done(None))
            }
            (0xa1, GET_IDLE) if interface < INTERFACES && value == 0 => reply(&[0]),
            (0x21, SET_IDLE) if interface < INTERFACES && value == 0 => Ok(Outcome::Done(None)),
            (0xa1, GET_PROTOCOL) if Self::boot(interface) && value == 0 => {
                reply(&[self.interfaces[interface].protocol])
            }
            (0x21, SET_PROTOCOL)
                if Self::boot(interface)
                    && matches!(value as u8, BOOT_PROTOCOL | REPORT_PROTOCOL) =>
            {
                self.interfaces[interface].protocol = value as u8;
                Ok(Outcome::Done(None))
            }
            _ => Err(Stall),
        }
    }

    fn setup(&mut self, setup: [u8; 8]) {
        let length = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
        self.control = if setup[0] & 0x80 == 0 && length != 0 {
            Control::Out {
                setup,
                data: Vec::new(),
            }
        } else {
            match self.request(setup, &[]) {
                Ok(Outcome::Data(data)) if setup[0] & 0x80 != 0 => Control::In {
                    data,
                    sent: 0,
                    length,
                    done: length == 0,
                },
                Ok(Outcome::Done(effect)) if length == 0 => Control::Status(effect),
                _ => Control::Stalled,
            }
        };
    }

    fn control_in(&mut self, buffer: &mut [u8]) -> Reply {
        match &mut self.control {
            Control::In {
                data,
                sent,
                length,
                done,
            } if !*done => {
                let end = data.len().min(*length);
                let n = (end - *sent).min(EP0_PACKET).min(buffer.len());
                buffer[..n].copy_from_slice(&data[*sent..*sent + n]);
                *sent += n;
                *done = n < EP0_PACKET || *sent == *length;
                Reply::Ack(n)
            }
            Control::Status(effect) => {
                if let Some(address) = *effect {
                    self.address = address;
                }
                self.control = Control::Idle;
                Reply::Ack(0)
            }
            _ => {
                self.control = Control::Stalled;
                Reply::Stall
            }
        }
    }

    fn control_out(&mut self, data: &[u8]) -> Reply {
        match &mut self.control {
            // The host's status stage, after all or part of the IN data.
            Control::In { .. } if data.is_empty() => {
                self.control = Control::Idle;
                Reply::Ack(0)
            }
            Control::Out {
                setup,
                data: so_far,
            } => {
                so_far.extend_from_slice(data);
                let length = usize::from(u16::from_le_bytes([setup[6], setup[7]]));
                if so_far.len() < length && data.len() == EP0_PACKET {
                    return Reply::Ack(data.len());
                }
                let (setup, so_far) = (*setup, core::mem::take(so_far));
                self.control = match self.request(setup, &so_far) {
                    Ok(Outcome::Done(effect)) if so_far.len() == length => Control::Status(effect),
                    _ => Control::Stalled,
                };
                Reply::Ack(data.len())
            }
            _ => {
                self.control = Control::Stalled;
                Reply::Stall
            }
        }
    }

    fn interrupt_in(&mut self, endpoint: u8, buffer: &mut [u8]) -> Reply {
        let Some(n) = Self::interface_of(endpoint).filter(|_| self.configuration != 0) else {
            return Reply::Stall;
        };
        if self.interfaces[n].halted {
            return Reply::Stall;
        }
        let Some(report) = self.interfaces[n].queue.pop_front() else {
            return Reply::Nak;
        };
        let report = self.report(n, report);
        let length = report.len().min(buffer.len());
        buffer[..length].copy_from_slice(&report[..length]);
        Reply::Ack(length)
    }

    /// `report` as interface `n` sends it in its current protocol: the
    /// boot mouse report is the first three bytes.
    fn report(&self, n: usize, mut report: Vec<u8>) -> Vec<u8> {
        if n == MOUSE && self.interfaces[n].protocol == BOOT_PROTOCOL {
            report.truncate(BOOT_MOUSE_REPORT);
        }
        report
    }
}

/// A request's outcome before its status stage.
enum Outcome {
    Data(Vec<u8>),
    Done(Option<u8>),
}

impl BusPeer for Peripheral {
    /// A bus reset returns the device to its Default state; held keys and
    /// buttons remain held.
    fn reset(&mut self) -> Result<()> {
        self.address = 0;
        self.configuration = 0;
        self.control = Control::Idle;
        self.interfaces = interfaces();
        Ok(())
    }

    fn packet(&mut self, endpoint: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
        Ok(match (endpoint, token) {
            (0, Token::Setup) => {
                let setup = data
                    .try_into()
                    .map_err(|_| refuse!("a SETUP packet is 8 bytes"))?;
                self.setup(setup);
                Reply::Ack(8)
            }
            (_, Token::Setup) => return Err(refuse!("SETUP to a non-control endpoint")),
            (0, Token::In) => self.control_in(data),
            (0, Token::Out) => self.control_out(data),
            (_, Token::In) => self.interrupt_in(endpoint, data),
            // No OUT endpoint: the LED report arrives through SET_REPORT.
            (_, Token::Out) => Reply::Stall,
        })
    }
}

impl Function for Peripheral {
    fn set_time(&mut self, _: u64) {}

    fn ready_at(&self, endpoint: u8) -> Option<u64> {
        let n = Self::interface_of(endpoint)?;
        if self.configuration == 0 || self.interfaces[n].halted {
            return None;
        }
        (!self.interfaces[n].queue.is_empty()).then_some(0)
    }
}

#[cfg(test)]
mod tests;
