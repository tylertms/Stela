// SPDX-License-Identifier: GPL-2.0-only
//! Polled PC i8042, raw-set-2 AT keyboards and ordinary/IntelliMouse/Explorer PS/2 mice.
//!
//! Linux `i8042.c`, `libps2.c`, `atkbd.c` and `psmouse-base.c` are the protocol
//! references; see NOTICE. Vojtech Pavlik
//! (1999–2002) and Dmitry Torokhov (2003–2004) keep their upstream copyrights.
//!
//! The platform must establish ACPI/PnP presence, remove USB legacy emulation,
//! mask both IRQ routes and provide exclusive controller ownership. This engine
//! leaves IRQs and set-1 translation disabled. It does not probe unadvertised
//! ports, active multiplexers or DMI quirks. [`synaptics`] and [`elantech`] supply
//! absolute touchpad profiles under this same controller, selected explicitly
//! or by [`Controller::new_probing`]'s psmouse protocol probe. Every
//! command is acknowledged; a failed/uncertain command quarantines this owner.
use super::{
    Input,
    observation::{
        Batch, Change, Description, Format, InitialState, Item, NativePoll, Origin, Payload,
        Producer, Source, Status, StopCause,
    },
};
use crate::{Clock, bus::i2c::Timer};
use alloc::{collections::VecDeque, rc::Rc, vec::Vec};
use core::cell::RefCell;

pub mod elantech;
mod keyboard;
mod mouse;
pub mod synaptics;

const DATA_PORT: u16 = 0x60;
const COMMAND_PORT: u16 = 0x64;
const OUTPUT_FULL: u8 = 1;
const INPUT_FULL: u8 = 1 << 1;
const AUX_DATA: u8 = 1 << 5;
const FRAME_ERRORS: u8 = 0xc0;
const IRQ_BITS: u8 = 3;
const KEYBOARD_DISABLED: u8 = 1 << 4;
const MOUSE_DISABLED: u8 = 1 << 5;
const TRANSLATION: u8 = 1 << 6;
const READ_CONFIG: u8 = 0x20;
const WRITE_CONFIG: u8 = 0x60;
const SELF_TEST: u8 = 0xaa;
const TEST_KEYBOARD: u8 = 0xab;
const DISABLE_KEYBOARD: u8 = 0xad;
const ENABLE_KEYBOARD: u8 = 0xae;
const TEST_MOUSE: u8 = 0xa9;
const DISABLE_MOUSE: u8 = 0xa7;
const ENABLE_MOUSE: u8 = 0xa8;
const SEND_MOUSE: u8 = 0xd4;
/// The next data byte comes back as if the auxiliary port sent it.
const AUX_LOOP: u8 = 0xd3;
/// The byte Linux's i8042_check_aux loops back.
const LOOP_BYTE: u8 = 0x5a;
const ACK: u8 = 0xfa;
const RESEND: u8 = 0xfe;
const RESET: u8 = 0xff;
const BAT_OK: u8 = 0xaa;
const ENABLE_REPORTING: u8 = 0xf4;
const DISABLE_REPORTING: u8 = 0xf5;
const DEFAULTS: u8 = 0xf6;
const GET_ID: u8 = 0xf2;
const SET_RATE: u8 = 0xf3;
const SET_SCAN: u8 = 0xf0;
const SET_LEDS: u8 = 0xed;
const SCALE_ONE_TO_ONE: u8 = 0xe6;
const SET_RESOLUTION: u8 = 0xe8;
const GET_INFO: u8 = 0xe9;
const QUEUE_LIMIT: usize = 64;
const MAX_ATTEMPTS: usize = 2;

/// Status bits that stand rather than follow each byte: the system flag
/// and the keyboard's inhibit switch (set when not inhibited).
const STANDING_STATUS: u8 = 0x14;

/// The bits of the status register that stand, for telling a change from
/// the flow of bytes; all ones, a controller that does not answer, kept.
pub fn standing_status(status: u8) -> u8 {
    if status == u8::MAX {
        status
    } else {
        status & STANDING_STATUS
    }
}

/// `status XX (...), configuration XX (...)`: the controller's status
/// register and configuration byte in words, for the boot log.
pub fn describe_controller(status: Option<u8>, config: Option<u8>) -> alloc::string::String {
    use alloc::format;
    let on = |set: bool| if set { "on" } else { "off" };
    let status = match status {
        None => alloc::string::String::from("status not read yet"),
        Some(u8::MAX) => alloc::string::String::from("status ff (no controller answers)"),
        Some(status) => format!(
            "status {status:02x} (system flag {}, keyboard {}, last byte's timeout {} and parity \
             error {})",
            on(status & 0x04 != 0),
            if status & 0x10 != 0 {
                "not inhibited"
            } else {
                "inhibited"
            },
            on(status & 0x40 != 0),
            on(status & 0x80 != 0),
        ),
    };
    let config = match config {
        None => alloc::string::String::from("configuration not read"),
        Some(config) => format!(
            "configuration {config:02x} (keyboard port {}, mouse port {}, translation {}, \
             keyboard interrupt {}, mouse interrupt {})",
            on(config & KEYBOARD_DISABLED == 0),
            on(config & MOUSE_DISABLED == 0),
            on(config & TRANSLATION != 0),
            on(config & 1 != 0),
            on(config & 2 != 0),
        ),
    };
    format!("{status}, {config}")
}

/// Which way keyboard bytes reach the decoder, for the boot log: translated
/// to set 1 as the firmware left the controller, or untranslated set 2.
pub fn describe_keyboard_path(translated: Option<bool>) -> &'static str {
    match translated {
        Some(true) => "translation kept as the firmware left it, keys decoded as set 1",
        Some(false) => "untranslated as the firmware left it, keys decoded as set 2",
        None => "keyboard path not chosen yet",
    }
}

/// The SMBIOS chassis types Linux's `atkbd_is_portable_device` lists:
/// portable, laptop, notebook, sub-notebook, convertible and detachable.
const PORTABLE_CHASSIS: [u8; 6] = [8, 9, 10, 14, 31, 32];

/// How a start decided a keyboard is behind the keyboard port, as Linux's
/// `atkbd_probe` decides it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum KeyboardPresence {
    /// It answered GETID with a keyboard's two ID bytes.
    Id,
    /// GETID failed, and it acknowledged Set LEDs.
    Leds,
    /// No GETID was sent: the port is translated and the firmware reports
    /// a portable chassis of this SMBIOS type, where Linux sends none.
    Assumed { chassis_type: u8 },
}

/// The keyboard presence check a start took, for the boot log.
pub fn describe_presence(presence: Option<KeyboardPresence>) -> alloc::string::String {
    use alloc::{format, string::String};
    match presence {
        Some(KeyboardPresence::Id) => String::from("keyboard found by its GETID answer"),
        Some(KeyboardPresence::Leds) => {
            String::from("keyboard found by Set LEDs after GETID failed")
        }
        Some(KeyboardPresence::Assumed { chassis_type }) => format!(
            "keyboard taken as present without GETID, a translated port in portable chassis \
             type {chassis_type}"
        ),
        None => String::from("no keyboard presence check"),
    }
}

/// Ordered, exclusively owned byte operations on the i8042 register pair.
pub trait Registers {
    fn status(&mut self) -> u8;
    fn read_data(&mut self) -> u8;
    fn write_command(&mut self, value: u8);
    fn write_data(&mut self, value: u8);
}

/// The native PC register pair. Construction performs no hardware access.
pub struct PortIo(());

impl PortIo {
    /// # Safety
    /// The platform established that these are the enabled i8042 data/status
    /// resources, not an EC or absent decoder. Caller has I/O privilege and
    /// exclusive ownership, with USB legacy/SMM interception removed and IRQ1/
    /// IRQ12 masked. No firmware, other driver or interrupt handler may access
    /// either register until this owner has been explicitly stopped.
    pub unsafe fn claim() -> Self {
        Self(())
    }

    fn read(port: u16) -> u8 {
        // SAFETY: claim establishes privilege and exclusive decoded resources.
        unsafe { stela_driver_base::bus::port::read8(port) }
    }
    fn write(port: u16, value: u8) {
        // SAFETY: claim establishes privilege and exclusive decoded resources.
        unsafe { stela_driver_base::bus::port::write8(port, value) }
    }
}
impl Registers for PortIo {
    fn status(&mut self) -> u8 {
        Self::read(COMMAND_PORT)
    }
    fn read_data(&mut self) -> u8 {
        Self::read(DATA_PORT)
    }
    fn write_command(&mut self, value: u8) {
        Self::write(COMMAND_PORT, value);
    }
    fn write_data(&mut self, value: u8) {
        Self::write(DATA_PORT, value);
    }
}

#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum MouseProtocol {
    #[default]
    Standard,
    IntelliMouse,
    Explorer,
}

#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub keyboard: bool,
    pub mouse: bool,
    /// Probe IntelliMouse (200/100/80) then Explorer (200/200/80), each
    /// followed by a GETID. An ordinary ID0 mouse stays three-byte.
    /// The separate Explorer4.0 horizontal-mode sequence is not enabled.
    pub wheel: bool,
    /// Reset the controller and keyboard before negotiation. Native x86
    /// firmware handoff leaves this off, as Linux's i8042/atkbd defaults do;
    /// some embedded laptop keyboards do not complete these reset commands.
    pub reset_on_start: bool,
    /// Query F0 00 after acknowledged Set 2 selection. Some laptop embedded
    /// keyboards ignore this optional query; native startup leaves it off.
    pub query_scan_set: bool,
    /// The SMBIOS chassis type the firmware reports, as Linux's DMI
    /// chassis type reads it. A translated keyboard in a portable chassis
    /// is not sent GETID, as Linux's `atkbd_skip_getid` decides.
    pub chassis_type: Option<u8>,
    pub io_timeout_us: u64,
    pub command_timeout_us: u64,
    pub reset_timeout_us: u64,
    /// An incomplete keyboard prefix or mouse packet cannot keep framing
    /// indefinitely. Expiration is reported; no invented input is emitted.
    pub packet_timeout_us: u64,
}
impl Default for Config {
    fn default() -> Self {
        Self {
            keyboard: true,
            mouse: true,
            wheel: true,
            reset_on_start: true,
            query_scan_set: true,
            chassis_type: None,
            io_timeout_us: 500_000,
            command_timeout_us: 200_000,
            reset_timeout_us: 1_000_000,
            packet_timeout_us: 500_000,
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Port {
    Keyboard,
    Mouse,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Phase {
    InputReady,
    Response,
    Drain,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum ErrorKind {
    Clock(crate::ClockError),
    InvalidConfiguration,
    NotInitialized,
    AlreadyInitialized,
    NeedsReset,
    NoController,
    Timeout(Phase),
    ControllerReply {
        command: u8,
        received: u8,
    },
    Frame {
        port: Port,
        status: u8,
    },
    DeviceReply {
        port: Port,
        received: u8,
    },
    ResendLimit {
        port: Port,
        byte: u8,
    },
    UnsupportedMouseId(u8),
    /// Enhanced horizontal/6-bit modes were not negotiated by this engine.
    MousePacketMode(u8),
    UnsupportedScanSet(u8),
    QueueFull,
    UnsupportedScanCode(u16),
    KeyboardSequence,
    PacketTimeout(Port),
    MouseSync(u8),
    DeviceReset(Port),
    Synaptics(synaptics::Error),
    Elantech(elantech::Error),
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Recovery {
    NotAttempted,
    PortsDisabled,
    Failed,
    /// Only this port's device failed: its port is disabled and the other
    /// port goes on, as Linux's atkbd and psmouse are separate devices.
    PortDisabled(Port),
    /// The controller and both devices go on without the failed command.
    Continued,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Error {
    pub kind: ErrorKind,
    /// Last wire command before failure, preserved across containment.
    pub command: Option<Command>,
    pub recovery: Recovery,
    /// Reply or framed-error byte consumed before failure, including a
    /// late ACK. This is an observation, not proof of accepted acknowledgement.
    /// Recovery register accesses cannot replace this original observation.
    pub response: Option<ObservedResponse>,
    pub recovery_error: Option<ErrorKind>,
    /// Separate containment observation, never relabeled as the primary
    /// reply or copied from it when containment performed no read.
    pub recovery_response: Option<ObservedResponse>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Command {
    Controller(u8),
    Device { port: Port, byte: u8 },
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct ObservedResponse {
    pub port: Port,
    pub data: u8,
}
impl ErrorKind {
    /// A device's own answer to one command, or its silence: the controller
    /// still works, and Linux's atkbd goes on without that command.
    fn device_refusal(self) -> bool {
        matches!(
            self,
            Self::Timeout(Phase::Response)
                | Self::ResendLimit { .. }
                | Self::DeviceReply { .. }
                | Self::DeviceReset(Port::Keyboard)
        )
    }
    /// A byte that broke a keyboard sequence or a mouse packet, or a packet
    /// that never finished: Linux's atkbd and psmouse drop it, resynchronize
    /// on the next byte and keep the device.
    fn framing(self) -> bool {
        matches!(
            self,
            Self::KeyboardSequence
                | Self::PacketTimeout(_)
                | Self::MouseSync(_)
                | Self::MousePacketMode(_)
                | Self::DeviceReset(Port::Keyboard)
                | Self::Synaptics(synaptics::Error::Packet { .. })
                | Self::Elantech(elantech::Error::Packet)
        )
    }
    /// The port whose device alone failed, when the other port can go on.
    fn port(self, command: Option<Command>) -> Option<Port> {
        match self {
            Self::Timeout(Phase::Response) => match command? {
                Command::Device { port, .. } => Some(port),
                Command::Controller(SEND_MOUSE) => Some(Port::Mouse),
                Command::Controller(_) => None,
            },
            Self::DeviceReply { port, .. }
            | Self::ResendLimit { port, .. }
            | Self::DeviceReset(port)
            | Self::PacketTimeout(port) => Some(port),
            Self::Frame {
                port: Port::Mouse, ..
            }
            | Self::UnsupportedMouseId(_)
            | Self::MousePacketMode(_)
            | Self::MouseSync(_)
            | Self::Synaptics(_)
            | Self::Elantech(_) => Some(Port::Mouse),
            Self::UnsupportedScanSet(_) | Self::KeyboardSequence => Some(Port::Keyboard),
            _ => None,
        }
    }
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(
            f,
            "PS/2 {:?} at {:?} (recovery {:?})",
            self.kind, self.command, self.recovery
        )
    }
}
impl From<ErrorKind> for Error {
    fn from(kind: ErrorKind) -> Self {
        Self {
            kind,
            command: None,
            recovery: Recovery::NotAttempted,
            response: None,
            recovery_error: None,
            recovery_response: None,
        }
    }
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum State {
    Fresh,
    Initializing,
    Ready,
    Stopped,
    Faulted,
}
#[derive(Clone, Copy)]
struct Byte {
    port: Port,
    data: u8,
    received_us: u64,
}

/// A controller shared with the SMBus companion of its mouse port's touchpad.
pub type Shared = Rc<RefCell<Controller>>;

/// The SMBus companion Linux uses by default for the touchpad on the mouse
/// port, as `synaptics_setup_intertouch` and `elantech_setup_smbus` choose it.
/// The InterTouch platform data (clickpad, top buttons, trackstick buttons)
/// shapes Linux's input properties, which the mouse emulation does not use.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Companion {
    /// Synaptics InterTouch: RMI4 at 0x2c.
    Rmi,
    /// An Elantech IC with SMBus Host Notify: ELAN at 0x15.
    Elan,
}

impl Companion {
    /// The companion's seven-bit SMBus address.
    pub fn address(self) -> u8 {
        match self {
            Self::Rmi => 0x2c,
            Self::Elan => 0x15,
        }
    }
}

/// Owns the controller for both ports, including cross-port bytes received
/// while the sibling executes a command. No IRQ worker or firmware user runs
/// concurrently with this polled engine.
pub struct Controller<R = PortIo, T = Clock> {
    registers: R,
    timer: T,
    clock_origin: u64,
    config: Config,
    state: State,
    controller_config: Option<u8>,
    input_known: bool,
    discarded_before_start: u64,
    pending: VecDeque<Byte>,
    events: Vec<Input>,
    keyboard: keyboard::Keyboard,
    keyboard_observation: Option<Producer>,
    pending_observations: Batch,
    mouse: mouse::Mouse,
    synaptics: Option<synaptics::Engine>,
    elantech: Option<elantech::Engine>,
    /// The mouse port's `PNP:` identifier when its touchpad protocol is
    /// probed at the first initialization.
    probe: Option<Probe>,
    /// The mouse port is handed to its SMBus companion, and its bytes are
    /// dropped, as `psmouse_smbus_process_byte` drops them.
    companion: bool,
    observed_response: Option<ObservedResponse>,
    last_command: Option<Command>,
    /// Commands a keyboard did not acknowledge that its start went on
    /// without, as Linux's atkbd goes on without them.
    refused: Vec<Command>,
    /// Mouse bytes dropped for their parity or timeout bit, and bytes that
    /// broke a keyboard sequence or mouse packet and were dropped.
    framing_errors: u64,
    /// Bytes read from the data port, by the port the status named:
    /// keyboard, then mouse.
    bytes_read: [u64; 2],
    /// The last status read, `None` before the first.
    last_status: Option<u8>,
    /// The configuration byte the firmware left, as the first start read it.
    firmware_config: Option<u8>,
    /// How the last start found the keyboard, `None` before it did.
    keyboard_presence: Option<KeyboardPresence>,
}

/// The psmouse protocol probe the first initialization runs on the mouse port.
struct Probe {
    firmware_id: Option<alloc::string::String>,
}
impl<R: Registers, T: Timer> Controller<R, T> {
    /// Validates configuration and keeps the timer origin; call `initialize`
    /// to touch device hardware. The same stable clock/context must survive the
    /// owner, with its origin observed within the timer's full counter period.
    pub fn new(registers: R, timer: T, config: Config) -> Result<Self, Error> {
        if (!config.keyboard && !config.mouse)
            || config.io_timeout_us == 0
            || config.command_timeout_us == 0
            || config.reset_timeout_us == 0
            || config.packet_timeout_us == 0
        {
            return Err(ErrorKind::InvalidConfiguration.into());
        }
        timer
            .check_duration(
                config
                    .io_timeout_us
                    .max(config.command_timeout_us)
                    .max(config.reset_timeout_us)
                    .max(config.packet_timeout_us),
            )
            .map_err(ErrorKind::Clock)?;
        let clock_origin = timer.start();
        Ok(Self {
            registers,
            timer,
            clock_origin,
            config,
            state: State::Fresh,
            controller_config: None,
            input_known: false,
            discarded_before_start: 0,
            pending: VecDeque::new(),
            events: Vec::new(),
            keyboard: keyboard::Keyboard::default(),
            keyboard_observation: None,
            pending_observations: Batch::default(),
            mouse: mouse::Mouse::default(),
            synaptics: None,
            elantech: None,
            probe: None,
            companion: false,
            observed_response: None,
            last_command: None,
            refused: Vec::new(),
            framing_errors: 0,
            bytes_read: [0; 2],
            last_status: None,
            firmware_config: None,
            keyboard_presence: None,
        })
    }
    /// Select the mouse port's protocol as Linux `psmouse_extensions` does
    /// for the protocols this owner drives: a Synaptics identity selects
    /// [`synaptics`] at 80 packets a second, else an Elantech knock and
    /// firmware signature select [`elantech`], else the mouse is reset and
    /// the IntelliMouse/Explorer probe runs. `firmware_id` is the port's
    /// `PNP: <_HID> <_CID>...` identifier, which selects Synaptics ForcePad
    /// and board profiles. The protocol stays selected across restarts.
    pub fn new_probing(
        registers: R,
        timer: T,
        config: Config,
        firmware_id: Option<&str>,
    ) -> Result<Self, Error> {
        let mut controller = Self::new(registers, timer, config)?;
        if config.mouse {
            // The longest touchpad command wait, Elantech's retry delay.
            controller
                .timer
                .check_duration(500_000)
                .map_err(ErrorKind::Clock)?;
            controller.probe = Some(Probe {
                firmware_id: firmware_id.map(alloc::string::String::from),
            });
        }
        Ok(controller)
    }
    pub fn is_ready(&self) -> bool {
        self.state == State::Ready
    }
    pub fn needs_reset(&self) -> bool {
        self.state == State::Faulted
    }
    /// Pre-reset bytes have unknown protocol/scan-set provenance and are not
    /// guest input. Count them rather than decoding them under the new mode.
    pub fn discarded_before_start(&self) -> u64 {
        self.discarded_before_start
    }
    /// Queued port/data/receipt-time observations, including bytes kept
    /// during failed commands and containment. Reading this does not consume
    /// input or grant permission to resume a faulted controller. Receipt times
    /// are microseconds from this owner's clock origin.
    pub fn retained_bytes(&self) -> impl Iterator<Item = (Port, u8, u64)> + '_ {
        self.pending
            .iter()
            .map(|byte| (byte.port, byte.data, byte.received_us))
    }
    /// Whether the keyboard port is configured.
    pub fn has_keyboard(&self) -> bool {
        self.config.keyboard
    }
    pub fn has_wheel(&self) -> bool {
        self.synaptics.is_none()
            && self.elantech.is_none()
            && self.mouse.protocol() != MouseProtocol::Standard
    }
    pub fn mouse_protocol(&self) -> Option<MouseProtocol> {
        (self.is_ready()
            && self.config.mouse
            && self.synaptics.is_none()
            && self.elantech.is_none())
        .then(|| self.mouse.protocol())
    }

    fn expired(&self, start: u64, limit: u64) -> bool {
        self.timer.elapsed_us(start) >= limit
    }
    fn timestamp_us(&self) -> u64 {
        self.timer.elapsed_us(self.clock_origin)
    }
    fn status(&mut self) -> Result<u8, ErrorKind> {
        let status = self.registers.status();
        self.last_status = Some(status);
        if status == u8::MAX {
            Err(ErrorKind::NoController)
        } else {
            Ok(status)
        }
    }
    fn write_ready(&mut self) -> Result<(), ErrorKind> {
        let start = self.timer.start();
        loop {
            let status = self.status()?;
            if self.expired(start, self.config.io_timeout_us) {
                return Err(ErrorKind::Timeout(Phase::InputReady));
            }
            if status & INPUT_FULL == 0 {
                return Ok(());
            }
            self.timer.relax();
        }
    }
    fn command(&mut self, command: u8) -> Result<(), ErrorKind> {
        self.last_command = Some(Command::Controller(command));
        self.write_ready()?;
        self.registers.write_command(command);
        Ok(())
    }
    fn write_data(&mut self, value: u8) -> Result<(), ErrorKind> {
        self.write_ready()?;
        self.registers.write_data(value);
        Ok(())
    }
    fn read_byte(&mut self) -> Result<Option<Byte>, ErrorKind> {
        self.read_byte_for(None)
    }
    /// The next byte either port sent. A keyboard byte whose status has the
    /// parity or timeout bit is taken as it is, as Linux's atkbd takes it on
    /// x86; a mouse byte with either bit is dropped, as psmouse drops it.
    /// A mouse port that sends nothing but such bytes fails after a queue's
    /// worth of them.
    fn read_byte_for(&mut self, expected: Option<Port>) -> Result<Option<Byte>, ErrorKind> {
        let mut framed = 0;
        for _ in 0..QUEUE_LIMIT {
            let status = self.status()?;
            if status & OUTPUT_FULL == 0 {
                return Ok(None);
            }
            let port = if status & AUX_DATA == 0 {
                Port::Keyboard
            } else {
                Port::Mouse
            };
            let dropped = port == Port::Mouse && status & FRAME_ERRORS != 0;
            // A sibling byte needs queue space. Do not consume it to learn
            // the queue is full, including during subsequent fault
            // containment. Expected replies have their own observation slot.
            if self.pending.len() >= QUEUE_LIMIT && expected != Some(port) && !dropped {
                return Err(ErrorKind::QueueFull);
            }
            let data = self.registers.read_data();
            let counted = &mut self.bytes_read[usize::from(port == Port::Mouse)];
            *counted = counted.saturating_add(1);
            if dropped {
                framed = status;
                self.observed_response = Some(ObservedResponse { port, data });
                self.framing_errors = self.framing_errors.saturating_add(1);
                continue;
            }
            return Ok(Some(Byte {
                port,
                data,
                received_us: self.timestamp_us(),
            }));
        }
        Err(ErrorKind::Frame {
            port: Port::Mouse,
            status: framed,
        })
    }
    fn queue(&mut self, byte: Byte) -> Result<(), ErrorKind> {
        if !self.input_known {
            self.discarded_before_start = self.discarded_before_start.saturating_add(1);
            return Ok(());
        }
        if self.pending.len() == QUEUE_LIMIT {
            return Err(ErrorKind::QueueFull);
        }
        self.pending.push_back(byte);
        Ok(())
    }
    fn response(&mut self, port: Port, start: u64, timeout: u64) -> Result<u8, ErrorKind> {
        self.observed_response = None;
        loop {
            if let Some(byte) = self.read_byte_for(Some(port))? {
                if byte.port == port {
                    self.observed_response = Some(ObservedResponse {
                        port,
                        data: byte.data,
                    });
                    if self.expired(start, timeout) {
                        return Err(ErrorKind::Timeout(Phase::Response));
                    }
                    return Ok(byte.data);
                }
                self.queue(byte)?;
            }
            if self.expired(start, timeout) {
                return Err(ErrorKind::Timeout(Phase::Response));
            }
            self.timer.relax();
        }
    }
    fn controller_response(&mut self, command: u8) -> Result<u8, ErrorKind> {
        self.observed_response = None;
        self.command(command)?;
        self.response(
            Port::Keyboard,
            self.timer.start(),
            self.config.io_timeout_us,
        )
    }
    /// The configuration byte once two reads in a row agree, as Linux's
    /// i8042_controller_init reads it, at most ten reads 50 us apart.
    fn stable_config(&mut self) -> Result<u8, ErrorKind> {
        let mut last = self.controller_response(READ_CONFIG)?;
        for _ in 1..10 {
            let start = self.timer.start();
            while !self.expired(start, 50) {
                self.timer.relax();
            }
            let config = self.controller_response(READ_CONFIG)?;
            if config == last {
                return Ok(config);
            }
            last = config;
        }
        Err(ErrorKind::ControllerReply {
            command: READ_CONFIG,
            received: last,
        })
    }
    fn set_config(&mut self, value: u8) -> Result<(), ErrorKind> {
        self.command(WRITE_CONFIG)?;
        self.write_data(value)?;
        self.write_ready()?;
        // Inhibited keyboards cannot produce another byte; remove any older
        // pre-handoff data before a controller response uses the same lane.
        if value & KEYBOARD_DISABLED != 0 {
            self.collect_output()?;
        }
        let mut actual = self.controller_response(READ_CONFIG)?;
        let mask = IRQ_BITS | KEYBOARD_DISABLED | MOUSE_DISABLED | TRANSLATION;
        if actual == ACK && actual & mask != value & mask {
            // Some embedded controllers acknowledge the CTR write with an ACK
            // that comes ahead of the CTR (Lenovo IdeaPad Pro 5 16AKP10).
            actual = self.response(Port::Keyboard, self.timer.start(), self.config.io_timeout_us)?;
        }
        if actual & mask != value & mask {
            return Err(ErrorKind::ControllerReply {
                command: WRITE_CONFIG,
                received: actual,
            });
        }
        self.controller_config = Some(actual);
        Ok(())
    }
    fn expect_controller(&mut self, command: u8, expected: u8) -> Result<(), ErrorKind> {
        let actual = self.controller_response(command)?;
        if actual != expected {
            return Err(ErrorKind::ControllerReply {
                command,
                received: actual,
            });
        }
        Ok(())
    }
    fn send_byte(
        &mut self,
        port: Port,
        value: u8,
        timeout: u64,
        id_without_ack: bool,
    ) -> Result<Option<u8>, ErrorKind> {
        self.observed_response = None;
        let start = self.timer.start();
        for _ in 0..MAX_ATTEMPTS {
            if port == Port::Mouse {
                self.command(SEND_MOUSE)?;
            }
            self.last_command = Some(Command::Device { port, byte: value });
            self.write_data(value)?;
            loop {
                let received = self.response(port, start, timeout)?;
                if port == Port::Mouse
                    && self.input_known
                    && (self
                        .synaptics
                        .as_ref()
                        .is_some_and(|engine| engine.input_byte(received))
                        || self
                            .elantech
                            .as_ref()
                            .is_some_and(|engine| engine.input_byte(received)))
                {
                    self.decode(Byte {
                        port,
                        data: received,
                        received_us: self.timestamp_us(),
                    })?;
                    if self.expired(start, timeout) {
                        return Err(ErrorKind::Timeout(Phase::Response));
                    }
                    continue;
                }
                match received {
                    ACK => return Ok(None),
                    RESEND => break,
                    0xfc | 0xfd => return Err(ErrorKind::DeviceReply { port, received }),
                    0 | 3 | 4 if id_without_ack => return Ok(Some(received)),
                    BAT_OK if port == Port::Keyboard => return Err(ErrorKind::DeviceReset(port)),
                    _ => self.queue(Byte {
                        port,
                        data: received,
                        received_us: self.timestamp_us(),
                    })?,
                }
                if self.expired(start, timeout) {
                    return Err(ErrorKind::Timeout(Phase::Response));
                }
                self.timer.relax();
            }
        }
        Err(ErrorKind::ResendLimit { port, byte: value })
    }
    fn device_command(&mut self, port: Port, value: u8, arguments: &[u8]) -> Result<(), ErrorKind> {
        // Pending packet decoding can fail before send_byte starts. An earlier
        // successful command's ACK is not a response to this new operation.
        self.observed_response = None;
        if port == Port::Mouse
            && self.input_known
            && (self.synaptics.is_some() || self.elantech.is_some())
        {
            // Sibling commands can keep a mouse prefix. Establish its
            // position before distinguishing reply FA from coordinate data FA.
            while let Some(byte) = self.pending.pop_front() {
                self.decode(byte)?;
            }
        }
        self.send_byte(port, value, self.config.command_timeout_us, false)?;
        for &argument in arguments {
            self.send_byte(port, argument, self.config.command_timeout_us, false)?;
        }
        Ok(())
    }
    fn sliced_command(&mut self, value: u8) -> Result<(), ErrorKind> {
        self.device_command(Port::Mouse, SCALE_ONE_TO_ONE, &[])?;
        for shift in [6, 4, 2, 0] {
            self.device_command(Port::Mouse, SET_RESOLUTION, &[(value >> shift) & 3])?;
        }
        Ok(())
    }
    /// The three bytes GETINFO returns: a mouse's status, or a touchpad
    /// query's answer after its sliced command.
    fn info(&mut self) -> Result<[u8; 3], ErrorKind> {
        self.device_command(Port::Mouse, GET_INFO, &[])?;
        let start = self.timer.start();
        let mut bytes = [0; 3];
        for byte in &mut bytes {
            *byte = self.response(Port::Mouse, start, self.config.command_timeout_us)?;
        }
        Ok(bytes)
    }
    /// Linux `synaptics_detect`, then `elantech_detect`; a mouse that is
    /// neither is reset, as `psmouse_extensions` resets it after its probes.
    fn select_touchpad(&mut self) -> Result<(), ErrorKind> {
        self.sliced_command(0)?;
        if self.info()?[1] == 0x47 {
            let firmware_id = self.probe.as_ref().and_then(|p| p.firmware_id.as_deref());
            self.synaptics = Some(synaptics::Engine::for_firmware(true, firmware_id));
            return Ok(());
        }
        self.device_command(Port::Mouse, DEFAULTS, &[])?;
        self.device_command(Port::Mouse, DISABLE_REPORTING, &[])?;
        for _ in 0..3 {
            self.device_command(Port::Mouse, SCALE_ONE_TO_ONE, &[])?;
        }
        if elantech::knocked(self.info()?) {
            self.sliced_command(1)?;
            if elantech::signature_valid(self.info()?) {
                self.elantech = Some(elantech::Engine::default());
                return Ok(());
            }
        }
        self.reset_mouse()
    }
    /// Reset the mouse and leave it at its defaults with reporting disabled.
    /// As Linux `psmouse_reset`, the reset fails only when neither the
    /// self-test result nor the ID that follows it is an ordinary mouse's,
    /// and a mouse that sends its self-test result without the ID passes.
    fn reset_mouse(&mut self) -> Result<(), ErrorKind> {
        self.send_byte(Port::Mouse, RESET, self.config.reset_timeout_us, false)?;
        let bat = self.response(
            Port::Mouse,
            self.timer.start(),
            self.config.reset_timeout_us,
        )?;
        let id = match self.response(
            Port::Mouse,
            self.timer.start(),
            self.config.command_timeout_us,
        ) {
            Ok(id) => Some(id),
            Err(ErrorKind::Timeout(Phase::Response)) => None,
            Err(kind) => return Err(kind),
        };
        if bat != BAT_OK && id != Some(0) {
            return Err(match id {
                Some(id) => ErrorKind::UnsupportedMouseId(id),
                None => ErrorKind::DeviceReply {
                    port: Port::Mouse,
                    received: bat,
                },
            });
        }
        self.device_command(Port::Mouse, DEFAULTS, &[])?;
        self.device_command(Port::Mouse, DISABLE_REPORTING, &[])
    }
    fn mouse_id(&mut self) -> Result<u8, ErrorKind> {
        match self.send_byte(Port::Mouse, GET_ID, self.config.command_timeout_us, true)? {
            Some(id) => Ok(id),
            None => self.response(
                Port::Mouse,
                self.timer.start(),
                self.config.command_timeout_us,
            ),
        }
    }
    fn reset_device(&mut self, port: Port) -> Result<(), ErrorKind> {
        self.send_byte(port, RESET, self.config.reset_timeout_us, false)?;
        let received = self.response(port, self.timer.start(), self.config.reset_timeout_us)?;
        if received != BAT_OK {
            return Err(ErrorKind::DeviceReply { port, received });
        }
        Ok(())
    }
    fn initialize_inner(&mut self) -> Result<(), ErrorKind> {
        self.keyboard_presence = None;
        self.command(DISABLE_KEYBOARD)?;
        self.command(DISABLE_MOUSE)?;
        // Discard only pre-handoff bytes while both ports are disabled. A
        // perpetually asserted OBF is a failure, not a successful flush.
        let start = self.timer.start();
        let mut drained = 0;
        while self.read_byte()?.is_some() {
            drained += 1;
            self.discarded_before_start = self.discarded_before_start.saturating_add(1);
            if drained == QUEUE_LIMIT || self.expired(start, self.config.io_timeout_us) {
                return Err(ErrorKind::Timeout(Phase::Drain));
            }
            self.timer.relax();
        }
        let config = self.stable_config()?;
        self.controller_config = Some(config);
        let firmware = *self.firmware_config.get_or_insert(config);
        // Translation stays as the firmware left it, as Linux's
        // i8042_controller_init leaves it by default: clearing it "can
        // confuse notebook BIOSes". The keyboard's bytes are decoded as the
        // set the controller then hands over.
        let config = (config & !TRANSLATION) | (firmware & TRANSLATION);
        self.keyboard.set_translated(config & TRANSLATION != 0);
        self.set_config((config & !IRQ_BITS) | KEYBOARD_DISABLED | MOUSE_DISABLED)?;
        if self.config.reset_on_start {
            self.expect_controller(SELF_TEST, 0x55)?;
            // Self-test can reset CTR on real controllers; reapply/read it back.
            self.set_config((config & !IRQ_BITS) | KEYBOARD_DISABLED | MOUSE_DISABLED)?;
        }
        // Linux tests neither port at a native start. Its auxiliary port
        // check loops a byte back through the controller first and runs the
        // port test only when that fails, "because it's common for chipsets
        // to return error on perfectly functioning AUX ports"; the test's 0,
        // FA and FF are then taken.
        if self.config.keyboard && self.config.reset_on_start {
            self.expect_controller(TEST_KEYBOARD, 0)?;
        }
        if self.config.mouse && !self.aux_loop()? {
            let received = self.controller_response(TEST_MOUSE)?;
            if !matches!(received, 0 | ACK | 0xff) {
                return Err(ErrorKind::ControllerReply {
                    command: TEST_MOUSE,
                    received,
                });
            }
        }
        let enabled = (config & !(IRQ_BITS | KEYBOARD_DISABLED | MOUSE_DISABLED))
            | if self.config.keyboard {
                0
            } else {
                KEYBOARD_DISABLED
            }
            | if self.config.mouse { 0 } else { MOUSE_DISABLED };
        if self.config.keyboard {
            self.start_keyboard()?;
        }
        if self.config.mouse {
            self.command(ENABLE_MOUSE)?;
            self.device_command(Port::Mouse, DISABLE_REPORTING, &[])?;
            self.reset_mouse()?;
            if self.probe.is_some() && self.synaptics.is_none() && self.elantech.is_none() {
                self.select_touchpad()?;
            }
            self.set_up_mouse()?;
        }
        // Both configured devices have now acknowledged reporting disable and
        // their new protocol is known. Only now is the keyboard response lane
        // safe for final CTR readback with the keyboard clock running.
        self.set_config(enabled)?;
        self.input_known = true;
        if self.config.mouse {
            self.device_command(Port::Mouse, ENABLE_REPORTING, &[])?;
        }
        if self.config.keyboard {
            // atkbd_activate's failure is reported, and the keyboard kept.
            self.optional_keyboard(ENABLE_REPORTING, &[])?;
        }
        Ok(())
    }
    /// Linux's AUX_LOOP in i8042_check_aux: whether the byte written for the
    /// auxiliary port's output comes back as that port's. The first byte
    /// either port gives is the answer, as Linux reads it.
    fn aux_loop(&mut self) -> Result<bool, ErrorKind> {
        self.command(AUX_LOOP)?;
        self.write_data(LOOP_BYTE)?;
        let start = self.timer.start();
        loop {
            if let Some(byte) = self.read_byte()? {
                return Ok(byte.port == Port::Mouse && byte.data == LOOP_BYTE);
            }
            if self.expired(start, self.config.io_timeout_us) {
                return Ok(false);
            }
            self.timer.relax();
        }
    }
    /// Bring up the keyboard as Linux `atkbd_connect` does on x86: no reset
    /// unless asked; GETID, or Set LEDs when GETID fails, to prove a
    /// keyboard is there, and neither on a translated port in a portable
    /// chassis, as `atkbd_skip_getid` decides; and the rest (reporting
    /// disable, Set 2 selection, LEDs, typematic) tried and gone on without
    /// when the keyboard does not acknowledge them. Behind a translating
    /// controller no scan-set command is sent, as Linux's `atkbd_select_set`
    /// sends none there. Untranslated, Set 2 selection is tried, and a
    /// keyboard that refuses it is taken to report Set 2, the set firmware
    /// and Linux both leave it in.
    fn start_keyboard(&mut self) -> Result<(), ErrorKind> {
        self.command(ENABLE_KEYBOARD)?;
        // Firmware may have left this keyboard reporting an unknown scan
        // set. Quiesce it first, and never interpret those bytes.
        self.optional_keyboard(DISABLE_REPORTING, &[])?;
        if self.config.reset_on_start {
            self.reset_device(Port::Keyboard)?;
            self.device_command(Port::Keyboard, DISABLE_REPORTING, &[])?;
        }
        let translated = self.controller_config.unwrap_or(0) & TRANSLATION != 0;
        let portable = self
            .config
            .chassis_type
            .filter(|chassis_type| translated && PORTABLE_CHASSIS.contains(chassis_type));
        let presence = match portable {
            Some(chassis_type) => KeyboardPresence::Assumed { chassis_type },
            None if self.keyboard_id()? => KeyboardPresence::Id,
            None => {
                self.device_command(Port::Keyboard, SET_LEDS, &[0])?;
                KeyboardPresence::Leds
            }
        };
        self.keyboard_presence = Some(presence);
        let leds_set = presence == KeyboardPresence::Leds;
        if !translated {
            self.optional_keyboard(SET_SCAN, &[2])?;
        }
        if self.config.query_scan_set && !translated {
            self.device_command(Port::Keyboard, SET_SCAN, &[0])?;
            let scan = self.response(
                Port::Keyboard,
                self.timer.start(),
                self.config.command_timeout_us,
            )?;
            if scan != 2 {
                return Err(ErrorKind::UnsupportedScanSet(scan));
            }
        }
        // Linux's atkbd_reset_state: LEDs off, then typematic rate and delay
        // at their fastest, unless the LEDs were refused.
        let leds = leds_set || self.optional_keyboard(SET_LEDS, &[0])?;
        if leds {
            self.optional_keyboard(SET_RATE, &[0])?;
        }
        Ok(())
    }
    /// A keyboard command whose refusal Linux's atkbd goes on after: the
    /// refused command is kept for [`Self::refused_commands`], and the start
    /// goes on. A controller failure still fails.
    fn optional_keyboard(&mut self, value: u8, arguments: &[u8]) -> Result<bool, ErrorKind> {
        match self.device_command(Port::Keyboard, value, arguments) {
            Ok(()) => Ok(true),
            Err(kind) if kind.device_refusal() => {
                let command = Command::Device {
                    port: Port::Keyboard,
                    byte: value,
                };
                if !self.refused.contains(&command) {
                    self.refused.push(command);
                }
                Ok(false)
            }
            Err(kind) => Err(kind),
        }
    }
    /// Linux `atkbd_probe`'s GETID: whether it succeeded with a keyboard's
    /// ID. GETID fails, for the Set LEDs check, when it is not acknowledged
    /// or an ID byte does not come. A first byte that `ps2_is_keyboard_id`
    /// does not list ends GETID with a device that is no keyboard, which
    /// Linux's atkbd does not take.
    fn keyboard_id(&mut self) -> Result<bool, ErrorKind> {
        match self.device_command(Port::Keyboard, GET_ID, &[]) {
            Ok(()) => {}
            Err(kind) if kind.device_refusal() => return Ok(false),
            Err(kind) => return Err(kind),
        }
        let timeout = self.config.command_timeout_us;
        let first = match self.response(Port::Keyboard, self.timer.start(), timeout) {
            Ok(first) => first,
            Err(ErrorKind::Timeout(Phase::Response)) => return Ok(false),
            Err(kind) => return Err(kind),
        };
        if !matches!(first, 0xab | 0xac | 0x2b | 0x5d | 0x60 | 0x47) {
            return Err(ErrorKind::DeviceReply {
                port: Port::Keyboard,
                received: first,
            });
        }
        match self.response(Port::Keyboard, self.timer.start(), timeout) {
            Ok(_) => Ok(true),
            Err(ErrorKind::Timeout(Phase::Response)) => Ok(false),
            Err(kind) => Err(kind),
        }
    }
    /// How the last start found the keyboard.
    pub fn keyboard_presence(&self) -> Option<KeyboardPresence> {
        self.keyboard_presence
    }
    /// Keyboard commands the keyboard did not acknowledge at its start, which
    /// the start went on without, as Linux's atkbd goes on without them.
    pub fn refused_commands(&self) -> &[Command] {
        &self.refused
    }
    /// Mouse bytes dropped for their parity or timeout bit, and bytes that
    /// broke a keyboard sequence or mouse packet and were dropped to
    /// resynchronize.
    pub fn framing_errors(&self) -> u64 {
        self.framing_errors
    }
    /// Bytes read from the data port since the controller was taken, by
    /// the port its status named: keyboard, then mouse, dropped ones
    /// included.
    pub fn bytes_read(&self) -> [u64; 2] {
        self.bytes_read
    }
    /// The last status register read, and the configuration byte the
    /// controller last held as this driver read or wrote it; `None` before
    /// either.
    pub fn controller_state(&self) -> (Option<u8>, Option<u8>) {
        (self.last_status, self.controller_config)
    }
    /// The configuration byte the firmware left, as the first start read it
    /// once it had disabled both ports.
    pub fn firmware_config(&self) -> Option<u8> {
        self.firmware_config
    }
    /// Whether the keyboard's bytes arrive translated to set 1, as the
    /// firmware left the controller; `None` before the first start read
    /// its configuration.
    pub fn translated(&self) -> Option<bool> {
        self.firmware_config.map(|config| config & TRANSLATION != 0)
    }
    /// Set up the mouse's selected protocol, its reporting still disabled.
    fn set_up_mouse(&mut self) -> Result<(), ErrorKind> {
        if self.synaptics.is_some() {
            return self.initialize_synaptics();
        }
        if self.elantech.is_some() {
            self.initialize_elantech()?;
            self.begin_elantech_rate_change();
            self.device_command(Port::Mouse, SET_RATE, &[100])?;
            self.restore_elantech_rate_mode()?;
            self.device_command(Port::Mouse, SET_RESOLUTION, &[3])?;
            return self.device_command(Port::Mouse, SCALE_ONE_TO_ONE, &[]);
        }
        self.device_command(Port::Mouse, SCALE_ONE_TO_ONE, &[])?;
        self.device_command(Port::Mouse, SET_RESOLUTION, &[2])?;
        if self.config.wheel {
            for rate in [200, 100, 80] {
                self.device_command(Port::Mouse, SET_RATE, &[rate])?;
            }
            let id = self.mouse_id()?;
            if !matches!(id, 0 | 3 | 4) {
                return Err(ErrorKind::UnsupportedMouseId(id));
            }
            for rate in [200, 200, 80] {
                self.device_command(Port::Mouse, SET_RATE, &[rate])?;
            }
            let protocol = match self.mouse_id()? {
                0 => MouseProtocol::Standard,
                3 => MouseProtocol::IntelliMouse,
                4 => MouseProtocol::Explorer,
                id => return Err(ErrorKind::UnsupportedMouseId(id)),
            };
            self.mouse.set_protocol(protocol);
        }
        self.device_command(Port::Mouse, SET_RATE, &[100])
    }
    /// The SMBus companion Linux uses by default for the touchpad on the
    /// mouse port, if the selected protocol has one.
    pub fn smbus_companion(&self) -> Option<Companion> {
        if let Some(engine) = &self.synaptics {
            return engine.intertouch();
        }
        let firmware = self.elantech_capabilities()?.firmware;
        // Linux's i2c_blacklist_pnp_ids is empty.
        elantech::host_notify(firmware).then_some(Companion::Elan)
    }
    /// Hand the mouse port to its SMBus companion, as Linux
    /// `psmouse_smbus_init` leaves it after the protocol probe: the device is
    /// reset to its defaults with reporting disabled, its held buttons are
    /// released, and its bytes are dropped from then on. Handing over a port
    /// already handed over does nothing.
    pub fn hand_mouse_to_companion(&mut self) -> Result<(), Error> {
        self.ready()?;
        if !self.config.mouse {
            return Err(ErrorKind::InvalidConfiguration.into());
        }
        if self.companion {
            return Ok(());
        }
        self.observed_response = None;
        let result = (|| {
            self.device_command(Port::Mouse, DISABLE_REPORTING, &[])?;
            self.companion = true;
            self.reset_mouse()
        })();
        result.map_err(|kind| self.recover(kind))?;
        self.mouse.release(&mut self.events);
        if let Some(engine) = &mut self.synaptics {
            engine.release(&mut self.events);
        }
        if let Some(engine) = &mut self.elantech {
            engine.release(&mut self.events);
        }
        Ok(())
    }
    /// Take the mouse port back from its SMBus companion: the selected
    /// protocol is set up again and reporting enabled, as Linux's rescan
    /// restores a PS/2 device whose companion is gone.
    pub fn reclaim_mouse(&mut self) -> Result<(), Error> {
        self.ready()?;
        if !self.companion {
            return Ok(());
        }
        self.observed_response = None;
        let result = (|| {
            self.reset_mouse()?;
            self.companion = false;
            self.set_up_mouse()?;
            self.device_command(Port::Mouse, ENABLE_REPORTING, &[])
        })();
        result.map_err(|kind| self.recover(kind))
    }
    /// Reset, test, initialize and enable only the configured attached ports.
    /// Ownership remains in this object on failure; a faulted object cannot
    /// retry without a platform-established reset and fresh ownership claim.
    pub fn initialize(&mut self) -> Result<(), Error> {
        if self.state == State::Ready {
            return Err(ErrorKind::AlreadyInitialized.into());
        }
        if !matches!(self.state, State::Fresh | State::Stopped) {
            return Err(ErrorKind::NeedsReset.into());
        }
        self.state = State::Initializing;
        self.observed_response = None;
        self.input_known = false;
        self.refused.clear();
        match self.initialize_inner() {
            Ok(()) => {
                self.state = State::Ready;
                if self.config.keyboard {
                    self.keyboard.begin_observation();
                    self.keyboard_observation = Some(Producer::new(
                        Description {
                            origin: Origin::Ps2Keyboard,
                            format: Format::ScanSet2,
                        },
                        InitialState::unknown(),
                    ));
                    if let Some(producer) = &self.keyboard_observation {
                        self.pending_observations
                            .items
                            .push(Item::Change(Change::Added(producer.source())));
                    }
                }
                Ok(())
            }
            Err(kind) => Err(self.fail(kind)),
        }
    }
    fn collect_output(&mut self) -> Result<(), ErrorKind> {
        // Controller replies share the keyboard data lane. Keep all older
        // input before asking for CTR; otherwise a key byte could be mistaken
        // for a successful controller acknowledgement.
        for _ in 0..QUEUE_LIMIT {
            match self.read_byte()? {
                Some(byte) => self.queue(byte)?,
                None => break,
            }
        }
        if self.status()? & OUTPUT_FULL != 0 {
            return Err(ErrorKind::QueueFull);
        }
        Ok(())
    }
    fn disable_ports(&mut self) -> Result<(), ErrorKind> {
        self.command(DISABLE_KEYBOARD)?;
        self.command(DISABLE_MOUSE)?;
        self.write_ready()?;
        self.collect_output()?;
        let config = match self.controller_config {
            Some(value) => value,
            None => self.controller_response(READ_CONFIG)?,
        };
        self.set_config((config & !IRQ_BITS) | KEYBOARD_DISABLED | MOUSE_DISABLED)
    }
    /// Contain a failure while running: a failure of one port's device
    /// disables that port alone while the other is configured; anything
    /// else stops both.
    fn recover(&mut self, kind: ErrorKind) -> Error {
        let command = self.last_command;
        let response = self.observed_response.take();
        if let Some(port) = kind.port(command) {
            let other = match port {
                Port::Keyboard => self.config.mouse,
                Port::Mouse => self.config.keyboard,
            };
            if other {
                return match self.disable_port(port) {
                    Ok(()) => Error {
                        command,
                        recovery: Recovery::PortDisabled(port),
                        response,
                        ..kind.into()
                    },
                    Err(containment) => {
                        let mut error = self.fail(containment);
                        error.kind = kind;
                        error.command = command;
                        error.response = response;
                        error.recovery_error = Some(containment);
                        error.recovery = Recovery::Failed;
                        error
                    }
                };
            }
        }
        self.observed_response = response;
        self.fail(kind)
    }
    /// Whether `kind` from a byte of `port` broke only a keyboard sequence
    /// or a mouse packet: the byte or packet is then dropped and the port
    /// reads on from its next byte, as Linux's atkbd and psmouse
    /// resynchronize. Nothing is reported for it but the count.
    fn resynchronized(&mut self, kind: ErrorKind, port: Port) -> bool {
        if !kind.framing() {
            return false;
        }
        let port = match kind {
            ErrorKind::PacketTimeout(port) | ErrorKind::DeviceReset(port) => port,
            ErrorKind::KeyboardSequence => Port::Keyboard,
            _ => port,
        };
        self.resync(port);
        self.framing_errors = self.framing_errors.saturating_add(1);
        true
    }
    /// Drop `port`'s packet or sequence in progress.
    fn resync(&mut self, port: Port) {
        match port {
            Port::Keyboard => self.keyboard.reset_prefix(),
            Port::Mouse => {
                if let Some(engine) = &mut self.synaptics {
                    engine.resync();
                } else if let Some(engine) = &mut self.elantech {
                    engine.resync();
                } else {
                    self.mouse.resync();
                }
            }
        }
    }
    /// Stop `port`'s clock, release what its device held and read on with
    /// the other port alone. The controller command sets the port's disable
    /// bit itself, so no configuration readback shares the keyboard's lane
    /// with keys.
    fn disable_port(&mut self, port: Port) -> Result<(), ErrorKind> {
        let (command, bit) = match port {
            Port::Keyboard => (DISABLE_KEYBOARD, KEYBOARD_DISABLED),
            Port::Mouse => (DISABLE_MOUSE, MOUSE_DISABLED),
        };
        self.command(command)?;
        self.write_ready()?;
        if let Some(config) = &mut self.controller_config {
            *config |= bit;
        }
        self.pending.retain(|byte| byte.port != port);
        match port {
            Port::Keyboard => {
                self.capture_keyboard_observation();
                self.stop_keyboard_observation(StopCause::TransportFailed);
                self.keyboard.release(&mut self.events);
                self.config.keyboard = false;
            }
            Port::Mouse => {
                self.mouse.release(&mut self.events);
                if let Some(engine) = &mut self.synaptics {
                    engine.release(&mut self.events);
                }
                if let Some(engine) = &mut self.elantech {
                    engine.release(&mut self.events);
                }
                self.config.mouse = false;
            }
        }
        Ok(())
    }
    fn fail(&mut self, kind: ErrorKind) -> Error {
        self.state = State::Faulted;
        self.capture_keyboard_observation();
        self.stop_keyboard_observation(StopCause::TransportFailed);
        let response = self.observed_response;
        let command = self.last_command;
        self.observed_response = None;
        let containment = self.disable_ports();
        let recovery = if containment.is_ok() {
            Recovery::PortsDisabled
        } else {
            Recovery::Failed
        };
        Error {
            kind,
            command,
            recovery,
            response,
            recovery_error: containment.err(),
            recovery_response: if containment.is_err() {
                self.observed_response
            } else {
                None
            },
        }
    }
    fn ready(&self) -> Result<(), Error> {
        match self.state {
            State::Ready => Ok(()),
            State::Faulted => Err(ErrorKind::NeedsReset.into()),
            _ => Err(ErrorKind::NotInitialized.into()),
        }
    }
    /// Set Scroll/Num/Caps LEDs (bits 0/1/2). Interleaved input is held.
    pub fn set_leds(&mut self, leds: u8) -> Result<(), Error> {
        self.ready()?;
        if leds & !7 != 0 {
            return Err(ErrorKind::InvalidConfiguration.into());
        }
        if !self.config.keyboard {
            return Err(ErrorKind::NotInitialized.into());
        }
        // Linux's atkbd goes on after a Set LEDs the keyboard refused.
        self.device_command(Port::Keyboard, SET_LEDS, &[leds])
            .map_err(|kind| {
                if kind.device_refusal() {
                    Error {
                        command: self.last_command,
                        recovery: Recovery::Continued,
                        ..kind.into()
                    }
                } else {
                    self.fail(kind)
                }
            })
    }
    fn decode(&mut self, byte: Byte) -> Result<(), ErrorKind> {
        match byte.port {
            Port::Keyboard if self.config.keyboard => self.keyboard.feed(
                byte.data,
                byte.received_us,
                self.config.packet_timeout_us,
                &mut self.events,
                self.keyboard_observation
                    .as_ref()
                    .is_some_and(|producer| producer.source().status() == Status::Producing),
            ),
            Port::Mouse if self.config.mouse && self.companion => Ok(()),
            Port::Mouse if self.config.mouse && self.synaptics.is_some() => {
                self.synaptics.as_mut().unwrap().feed(
                    byte.data,
                    byte.received_us,
                    self.config.packet_timeout_us,
                    &mut self.events,
                )
            }
            Port::Mouse if self.config.mouse && self.elantech.is_some() => {
                self.elantech.as_mut().unwrap().feed(
                    byte.data,
                    byte.received_us,
                    self.config.packet_timeout_us,
                    &mut self.events,
                )
            }
            Port::Mouse if self.config.mouse => self.mouse.feed(
                byte.data,
                byte.received_us,
                self.config.packet_timeout_us,
                &mut self.events,
            ),
            // A byte from a port this owner disabled, sent before the
            // controller stopped its clock.
            _ => {
                self.framing_errors = self.framing_errors.saturating_add(1);
                Ok(())
            }
        }
    }
    /// Decode at most 64 hardware bytes per poll. Prefixes/partial packets stay
    /// with their port. On error, already decoded input remains available
    /// through `drain_events`; it is never silently discarded with the error.
    pub fn poll(&mut self) -> Result<Vec<Input>, Error> {
        self.close_observation_admission();
        let result = self.poll_inner();
        drop(self.keyboard.drain_observed());
        result
    }

    pub fn observation_sources(&self) -> Vec<Source> {
        self.keyboard_observation
            .as_ref()
            .map(Producer::source)
            .into_iter()
            .collect()
    }

    /// Poll the SAME controller once. Completed physical scan prefixes survive a
    /// later byte error alongside the unchanged guest decoded prefix.
    pub fn poll_observed(&mut self) -> NativePoll<Error> {
        let result = self.poll_inner();
        let stop = self.needs_reset().then_some(StopCause::TransportFailed);
        self.observed_result(result, stop)
    }

    fn capture_keyboard_observation(&mut self) {
        let bytes = self.keyboard.drain_observed();
        let Some(producer) = &mut self.keyboard_observation else {
            return;
        };
        if !bytes.is_empty() && producer.source().status() == Status::Producing {
            self.pending_observations
                .publish(producer, Payload::ScanSet2 { bytes });
        }
    }

    fn stop_keyboard_observation(&mut self, cause: StopCause) {
        if let Some(producer) = &mut self.keyboard_observation
            && let Some(change) = producer.stop(cause)
        {
            self.pending_observations.items.push(Item::Change(change));
        }
    }

    fn observed_result(
        &mut self,
        result: Result<Vec<Input>, Error>,
        stop: Option<StopCause>,
    ) -> NativePoll<Error> {
        let mut polled = NativePoll::default();
        match result {
            Ok(inputs) => polled.inputs = inputs,
            Err(error) => {
                polled.inputs = self.drain_events();
                polled.record_error(error);
            }
        }
        self.capture_keyboard_observation();
        if let Some(cause) = stop {
            self.stop_keyboard_observation(cause);
        }
        polled.observations = core::mem::take(&mut self.pending_observations);
        polled
    }

    /// Close observation only, draining decoded facts before the stop frontier.
    /// No hardware poll/stop or Linux callback/current loan occurs here.
    fn close_observation_admission(&mut self) {
        self.capture_keyboard_observation();
        self.stop_keyboard_observation(StopCause::ObservationClosed);
    }

    pub fn close_observation(&mut self) -> Batch {
        self.close_observation_admission();
        core::mem::take(&mut self.pending_observations)
    }

    fn partial_input(&self) -> bool {
        self.keyboard.prefix()
            || if let Some(engine) = &self.synaptics {
                engine.partial()
            } else if let Some(engine) = &self.elantech {
                engine.partial()
            } else {
                self.mouse.partial()
            }
    }

    fn poll_inner(&mut self) -> Result<Vec<Input>, Error> {
        self.ready()?;
        self.observed_response = None;
        for _ in 0..QUEUE_LIMIT {
            if self
                .synaptics
                .as_ref()
                .is_some_and(|engine| engine.backpressured())
                || self
                    .elantech
                    .as_ref()
                    .is_some_and(|engine| engine.backpressured())
            {
                break;
            }
            let byte = match self.pending.pop_front() {
                Some(byte) => Some(byte),
                None => self.read_byte().map_err(|kind| self.recover(kind))?,
            };
            let Some(byte) = byte else {
                // The port already reported no output. Only an incomplete
                // packet needs another status check before expiring it.
                if !self.partial_input() {
                    return Ok(self.drain_events());
                }
                break;
            };
            if let Err(kind) = self.decode(byte) {
                // A key outside the declared HID usage map is explicit but
                // does not corrupt either port's framing or stop known keys.
                if matches!(kind, ErrorKind::UnsupportedScanCode(_)) {
                    return Err(kind.into());
                }
                if self.resynchronized(kind, byte.port) {
                    continue;
                }
                return Err(self.recover(kind));
            }
        }
        // A silent device must not keep an incomplete prefix indefinitely.
        // Do not expire a frame while complete bytes remain queued to decode.
        if self.pending.is_empty()
            && self.status().map_err(|kind| self.fail(kind))? & OUTPUT_FULL == 0
        {
            let now = self.timestamp_us();
            let result = self
                .keyboard
                .expire(now, self.config.packet_timeout_us)
                .and_then(|()| {
                    if let Some(engine) = &self.synaptics {
                        engine.expire(now, self.config.packet_timeout_us)
                    } else if let Some(engine) = &mut self.elantech {
                        engine.expire(now, self.config.packet_timeout_us)
                    } else {
                        self.mouse.expire(now, self.config.packet_timeout_us)
                    }
                });
            if let Err(kind) = result
                && !self.resynchronized(kind, Port::Mouse)
            {
                return Err(self.recover(kind));
            }
        }
        Ok(self.drain_events())
    }
    pub fn drain_events(&mut self) -> Vec<Input> {
        core::mem::take(&mut self.events)
    }
    /// Release every published held key/button without inventing motion. Also
    /// used after an explicit fault so consumers can retire stale held input.
    pub fn release_inputs(&mut self) -> Vec<Input> {
        self.keyboard.release(&mut self.events);
        self.mouse.release(&mut self.events);
        if let Some(engine) = &mut self.synaptics {
            engine.release(&mut self.events);
        }
        if let Some(engine) = &mut self.elantech {
            engine.release(&mut self.events);
        }
        self.drain_events()
    }
    /// Acknowledge reporting disable on both configured devices, then prove
    /// clock/IRQ disable by reading CTR. Failure quarantines this owner.
    pub fn stop(&mut self) -> Result<Vec<Input>, Error> {
        self.close_observation_admission();
        let result = self.stop_inner();
        drop(self.keyboard.drain_observed());
        result
    }

    pub fn stop_observed(&mut self) -> NativePoll<Error> {
        let result = self.stop_inner();
        let cause = if result.is_ok() {
            StopCause::ObservationClosed
        } else {
            StopCause::TransportFailed
        };
        self.observed_result(result, Some(cause))
    }

    fn stop_inner(&mut self) -> Result<Vec<Input>, Error> {
        self.ready()?;
        self.observed_response = None;
        let result = (|| {
            if self.config.keyboard {
                self.device_command(Port::Keyboard, DISABLE_REPORTING, &[])?;
            }
            if self.config.mouse && !self.stop_synaptics_mouse_inner()? {
                self.device_command(Port::Mouse, DISABLE_REPORTING, &[])?;
            }
            self.disable_ports()?;
            while let Some(byte) = self.pending.pop_front() {
                self.decode(byte)?;
            }
            Ok(())
        })();
        match result {
            Ok(()) => {
                self.capture_keyboard_observation();
                self.stop_keyboard_observation(StopCause::ObservationClosed);
                self.state = State::Stopped;
                Ok(self.release_inputs())
            }
            Err(kind) => Err(self.fail(kind)),
        }
    }
}

#[cfg(test)]
mod tests;
