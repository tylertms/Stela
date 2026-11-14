//! The DCPAVIPC controller service above AFKEPV2's interface transport.
//!
//! Original restore firmware `ipad14dcp_restore` registers the 16 records
//! at VA 0x70ec78 from 0x93a4c. Their handlers are 0x103e48..0x104014
//! (common commands) and 0x93ae0..0x93e44 (controller commands). Firmware
//! 0x102aa4 and AP kernel file 0x336a200 independently validate the same
//! 64-byte envelope. Firmware 0x104338 returns the mutated message through
//! the AFK command response, preserving its header. Addresses here are
//! firmware VAs unless explicitly marked kernel file offsets.
//!
//! The advertised power capability is backed by shared display state:
//! IOMFB power requests and controller power requests change the same
//! scanout gate. This fixed monitor has no virtual-device mode or CRC
//! acquisition; those operations explicitly refuse instead of inventing
//! device data. The original event-log controls themselves answer
//! `kIOReturnUnsupported` (0x103e48..0x103f84).
use crate::Result;
use alloc::rc::Rc;
#[cfg(not(feature = "std"))]
use alloc::vec::Vec;
use alloc::{format, string::String};
use core::cell::RefCell;
use stela_contracts::refuse;

pub const SERVICE_NAME: &str = "dcpav-controller-epic";
/// Original named endpoint config2ff148..2ff1cc and mailbox mask20e8c.
pub(super) const ENDPOINT: u8 = 0x24;
/// The sole native publisher uses the original odd allocator's first free ID.
pub(super) const INTERFACE: u16 = 1;
pub(super) fn descriptor(name: String) -> super::afk::interface::Descriptor {
    let properties = format!("<dict><key>EPICName</key><string>{SERVICE_NAME}</string><key>EPICUnit</key><integer size=\"32\">0</integer><key>EPICLocation</key><string>External</string><key>EPICProviderClass</key><string>AppleDCPDPTXController</string><key>interface-name</key><string>{name}</string><key>interface-id</key><integer size=\"32\">{INTERFACE}</integer></dict>\0").into_bytes();
    super::afk::interface::Descriptor {
        number: INTERFACE,
        name,
        //102bf0's four EPIC fields and interface-name, plus AFK publication's
        // interface-id at36008. SupportsCRC=false omits RequiresCommandDelegate
        // (93930); the command family explicitly refuses CRC acquisition.
        properties,
    }
}

impl super::afk::interface::Service for Controller {
    fn handle(&mut self, packet_type: u8, payload: &[u8]) -> Result<(i32, Vec<u8>)> {
        Controller::handle(self, packet_type, payload)
    }
    fn transport_closed(&mut self) {
        Controller::transport_closed(self);
    }
}

/// Firmware 0x10423c dispatches this AFK command type to DCPAVIPC.
pub const COMMAND_PACKET_TYPE: u8 = 0xc0;
const HEADER_BYTES: usize = 0x40;
const MAGIC: u32 = 0x6970_6378;
const VARIABLE_LENGTH: u16 = 1;
/// The original validator's error, distinct from command-level IOReturn.
const INVALID_MESSAGE: i32 = 0xe000_0001u32 as i32;
const SUCCESS: u32 = 0;
const UNSUPPORTED: u32 = 0xe000_02c7;
const VALUE: usize = HEADER_BYTES + 0x10;

/// Command order and sizes are the original firmware's complete table.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u32)]
enum Command {
    ClearEventLog,
    SetEventLogMaxCapacity,
    SetEventLogCommandMask,
    SetEventLogEventMask,
    Open,
    Close,
    SetPower,
    GetPower,
    WakeDisplay,
    SleepDisplay,
    ForceHotPlugDetect,
    SetVirtualDeviceMode,
    GetParticipatesPowerManagement,
    GetDefaultPowerState,
    GetVirtualDeviceBootMode,
    GetCrcData,
}

const COMMANDS: [(Command, usize); 16] = [
    (Command::ClearEventLog, 0x20),
    (Command::SetEventLogMaxCapacity, 0x20),
    (Command::SetEventLogCommandMask, 0x20),
    (Command::SetEventLogEventMask, 0x20),
    (Command::Open, 0x20),
    (Command::Close, 0x10),
    (Command::SetPower, 0x20),
    (Command::GetPower, 0x20),
    (Command::WakeDisplay, 0x10),
    (Command::SleepDisplay, 0x10),
    (Command::ForceHotPlugDetect, 0x10),
    (Command::SetVirtualDeviceMode, 0x20),
    (Command::GetParticipatesPowerManagement, 0x20),
    (Command::GetDefaultPowerState, 0x20),
    (Command::GetVirtualDeviceBootMode, 0x20),
    (Command::GetCrcData, 0x30),
];

/// The fixed monitor state, shared by IOMFB and its IOAV controller.
#[derive(Debug)]
pub struct DisplayState {
    /// Configured initial model power, not an Apple boot-state observation.
    default_power: bool,
    /// None until an explicit power request.
    pub(super) powered: Option<bool>,
    pub(super) asleep: bool,
    hotplug_requested: bool,
}

impl DisplayState {
    pub fn new(default_power: bool) -> Self {
        Self {
            default_power,
            powered: None,
            asleep: false,
            hotplug_requested: false,
        }
    }

    /// Current output follows requests, or its explicitly configured default.
    pub(super) fn active(&self) -> bool {
        self.powered.unwrap_or(self.default_power) && !self.asleep
    }

    pub(super) fn take_hotplug_request(&mut self) -> bool {
        core::mem::take(&mut self.hotplug_requested)
    }

    pub(super) fn hotplug_pending(&self) -> bool {
        self.hotplug_requested
    }
}

/// One controller interface session. Interface transport owns correlation,
/// request buffers, fragmentation and completion delivery.
pub struct Controller {
    display: Rc<RefCell<DisplayState>>,
    opened: bool,
}

impl Controller {
    pub fn new(display: Rc<RefCell<DisplayState>>) -> Self {
        Self {
            display,
            opened: false,
        }
    }

    /// Transport termination closes the client; it does not power off the
    /// physical display. IOP sleep does not call this operation.
    pub fn transport_closed(&mut self) {
        self.opened = false;
    }

    pub fn handle(&mut self, packet_type: u8, payload: &[u8]) -> Result<(i32, Vec<u8>)> {
        if packet_type != COMMAND_PACKET_TYPE {
            return Err(refuse!(
                "DCPAVIPC controller packet type is not command 0xc0"
            ));
        }
        let Some(command) = validate(payload) else {
            return Ok((INVALID_MESSAGE, payload.to_vec()));
        };
        let mut reply = payload.to_vec();
        let value = || u64::from_le_bytes(payload[VALUE..VALUE + 8].try_into().unwrap());
        match command {
            Command::ClearEventLog
            | Command::SetEventLogMaxCapacity
            | Command::SetEventLogCommandMask
            | Command::SetEventLogEventMask => write_u32(&mut reply, HEADER_BYTES, UNSUPPORTED),
            Command::Open => {
                self.opened = true;
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
                write_u64(&mut reply, VALUE, 1);
            }
            Command::Close => {
                self.opened = false;
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
            }
            Command::SetPower => {
                self.display.borrow_mut().powered = Some(value() != 0);
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
            }
            Command::GetPower => {
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
                let display = self.display.borrow();
                write_u64(
                    &mut reply,
                    VALUE,
                    u64::from(display.powered.unwrap_or(display.default_power)),
                );
            }
            Command::WakeDisplay | Command::SleepDisplay => {
                self.display.borrow_mut().asleep = command == Command::SleepDisplay;
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
            }
            Command::ForceHotPlugDetect => {
                self.display.borrow_mut().hotplug_requested = true;
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
            }
            Command::SetVirtualDeviceMode => {
                if value() != 0 {
                    return Err(refuse!(
                        "DCPAVIPC SetVirtualDeviceMode requires an unmodeled virtual monitor"
                    ));
                }
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
            }
            Command::GetParticipatesPowerManagement => {
                // Backed by power control and the shared scanout gate.
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
                write_u64(&mut reply, VALUE, 1);
            }
            Command::GetDefaultPowerState => {
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
                write_u64(
                    &mut reply,
                    VALUE,
                    u64::from(self.display.borrow().default_power),
                );
            }
            Command::GetVirtualDeviceBootMode => {
                write_u32(&mut reply, HEADER_BYTES, SUCCESS);
                write_u64(&mut reply, VALUE, 0);
            }
            Command::GetCrcData => {
                return Err(refuse!(
                    "DCPAVIPC GetCRCData requires unmodeled display CRC acquisition"
                ));
            }
        }
        Ok((0, reply))
    }
}

/// Bytes 0..2 and the unused header fields are opaque in both original
/// validators: they are not an invented protocol version. Preserve them.
fn validate(bytes: &[u8]) -> Option<Command> {
    if bytes.len() < HEADER_BYTES {
        return None;
    }
    let word = |offset| u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap());
    if word(12) != MAGIC || usize::try_from(word(8)).ok()? != bytes.len() - HEADER_BYTES {
        return None;
    }
    let &(command, expected) = COMMANDS.get(usize::try_from(word(4)).ok()?)?;
    let flags = u16::from_le_bytes(bytes[2..4].try_into().unwrap());
    let actual = bytes.len() - HEADER_BYTES;
    if flags & VARIABLE_LENGTH == 0 && actual != expected || actual < expected {
        return None;
    }
    Some(command)
}

fn write_u32(bytes: &mut [u8], offset: usize, value: u32) {
    bytes[offset..offset + 4].copy_from_slice(&value.to_le_bytes());
}

fn write_u64(bytes: &mut [u8], offset: usize, value: u64) {
    bytes[offset..offset + 8].copy_from_slice(&value.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    fn request(command: u32, bytes: usize) -> Vec<u8> {
        let mut request = alloc::vec![0xa5; HEADER_BYTES + bytes];
        request[..4].fill(0);
        write_u32(&mut request, 4, command);
        write_u32(&mut request, 8, bytes as u32);
        write_u32(&mut request, 12, MAGIC);
        request
    }

    fn controller() -> Controller {
        Controller::new(Rc::new(RefCell::new(DisplayState::new(true))))
    }

    fn call(controller: &mut Controller, command: u32, bytes: usize, value: u64) -> Vec<u8> {
        let mut request = request(command, bytes);
        if bytes >= 0x20 {
            write_u64(&mut request, VALUE, value);
        }
        let (status, reply) = controller.handle(COMMAND_PACKET_TYPE, &request).unwrap();
        assert_eq!(status, 0);
        assert_eq!(&reply[..HEADER_BYTES], &request[..HEADER_BYTES]);
        reply
    }

    fn word(bytes: &[u8], offset: usize) -> u32 {
        u32::from_le_bytes(bytes[offset..offset + 4].try_into().unwrap())
    }

    #[test]
    fn capabilities_and_power_share_state() {
        let mut controller = controller();
        for (command, expected) in [(12, 1), (13, 1), (14, 0), (7, 1)] {
            assert_eq!(
                word(&call(&mut controller, command, 0x20, 0), VALUE),
                expected
            );
        }
        assert_eq!(word(&call(&mut controller, 6, 0x20, 0), HEADER_BYTES), 0);
        assert!(!controller.display.borrow().active());
        assert_eq!(word(&call(&mut controller, 7, 0x20, 0), VALUE), 0);
        assert_eq!(word(&call(&mut controller, 13, 0x20, 0), VALUE), 1);
        call(&mut controller, 6, 0x20, u64::MAX);
        assert!(controller.display.borrow().active());
        call(&mut controller, 9, 0x10, 0);
        assert!(!controller.display.borrow().active());
        call(&mut controller, 8, 0x10, 0);
        assert!(controller.display.borrow().active());
    }

    #[test]
    fn default_power_is_explicit_configuration_and_wake_does_not_override_it() {
        let mut controller = Controller::new(Rc::new(RefCell::new(DisplayState::new(false))));
        for command in [7, 13] {
            assert_eq!(word(&call(&mut controller, command, 0x20, 0), VALUE), 0);
        }
        call(&mut controller, 9, 0x10, 0);
        call(&mut controller, 8, 0x10, 0);
        assert!(!controller.display.borrow().active());
        call(&mut controller, 6, 0x20, 1);
        assert!(controller.display.borrow().active());
        assert_eq!(word(&call(&mut controller, 13, 0x20, 0), VALUE), 0);
    }

    #[test]
    fn open_close_and_transport_loss_do_not_invent_power_changes() {
        let mut controller = controller();
        assert_eq!(word(&call(&mut controller, 4, 0x20, 0), VALUE), 1);
        assert!(controller.opened);
        controller.transport_closed();
        assert!(!controller.opened && controller.display.borrow().active());
        call(&mut controller, 4, 0x20, 0);
        call(&mut controller, 5, 0x10, 0);
        assert!(!controller.opened && controller.display.borrow().active());
        call(&mut controller, 10, 0x10, 0);
        assert!(controller.display.borrow_mut().take_hotplug_request());
        assert!(!controller.display.borrow_mut().take_hotplug_request());
    }

    #[test]
    fn unsupported_operations_and_original_event_log_errors_are_explicit() {
        let mut controller = controller();
        for command in 0..4 {
            assert_eq!(
                word(&call(&mut controller, command, 0x20, 0), HEADER_BYTES),
                UNSUPPORTED
            );
        }
        assert_eq!(word(&call(&mut controller, 11, 0x20, 0), HEADER_BYTES), 0);
        let mut virtual_request = request(11, 0x20);
        write_u64(&mut virtual_request, VALUE, 1);
        assert!(
            controller
                .handle(COMMAND_PACKET_TYPE, &virtual_request)
                .unwrap_err()
                .to_string()
                .contains("SetVirtualDeviceMode")
        );
        assert!(
            controller
                .handle(COMMAND_PACKET_TYPE, &request(15, 0x30))
                .unwrap_err()
                .to_string()
                .contains("GetCRCData")
        );
        assert!(controller.display.borrow().active());
    }

    #[test]
    fn validation_covers_all_table_sizes_and_variable_length_without_side_effects() {
        // Independent original table: 0..15, not generated from COMMANDS.
        let sizes = [
            32, 32, 32, 32, 32, 16, 32, 32, 16, 16, 16, 32, 32, 32, 32, 48,
        ];
        for (command, size) in sizes.into_iter().enumerate() {
            let mut request = request(command as u32, size);
            assert!(validate(&request).is_some());
            request.extend_from_slice(&[0x9a; 16]);
            write_u32(&mut request, 8, (size + 16) as u32);
            assert!(validate(&request).is_none());
            request[2] = 1;
            assert!(validate(&request).is_some());
            request.truncate(HEADER_BYTES + size - 1);
            write_u32(&mut request, 8, (size - 1) as u32);
            assert!(validate(&request).is_none());
        }
        let mut controller = controller();
        let mut bad_magic = request(6, 32);
        bad_magic[12] ^= 1;
        let mut bad_length = request(6, 32);
        write_u32(&mut bad_length, 8, 64);
        for bytes in [alloc::vec![0; 63], request(16, 32), bad_magic, bad_length] {
            let original = bytes.clone();
            assert_eq!(
                controller.handle(COMMAND_PACKET_TYPE, &bytes).unwrap(),
                (INVALID_MESSAGE, original)
            );
        }
        assert!(controller.display.borrow().active() && !controller.opened);
        assert!(
            controller
                .handle(0, &request(7, 32))
                .unwrap_err()
                .to_string()
                .contains("packet type")
        );
    }

    #[test]
    fn response_preserves_opaque_header_fields_and_variable_tails() {
        let mut bytes = request(7, 0x30);
        bytes[..2].copy_from_slice(&[0x34, 0x12]);
        bytes[2..4].copy_from_slice(&0x8001u16.to_le_bytes());
        let (status, reply) = controller().handle(COMMAND_PACKET_TYPE, &bytes).unwrap();
        assert_eq!(status, 0);
        assert_eq!(&reply[..HEADER_BYTES], &bytes[..HEADER_BYTES]);
        assert_eq!(&reply[HEADER_BYTES + 0x20..], &[0xa5; 16]);
        assert_eq!(&reply[0x44..0x50], &[0xa5; 12]);
        assert_eq!(&reply[0x50..0x58], &1u64.to_le_bytes());
    }
}
