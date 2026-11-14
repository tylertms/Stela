//! Native controller fixture: consumes command/context/EP0/normal TRBs.
use super::*;
use alloc::collections::{BTreeMap, VecDeque};

mod bot;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(in super::super) enum Fault {
    None,
    Power,
    PowerAbsent,
    Reset,
    ResetForever,
    Disconnect,
    NoEnable,
    Suspended,
    UnknownSpeed,
    HotThenWarm,
    Inactive,
    WarmForever,
    ShortStatus,
    ClearChange,
    LinkNotU0,
    ConfigError,
    ConnectionChange,
    ResetConnectionBounce,
    FirstResetDisabled,
    FirstResetNotU0,
    OverCurrentChange,
    NoResetChange,
    SuspendIgnored,
    ResumeIgnored,
    ResetRecoveryLoss,
}
#[derive(Clone)]
pub(in super::super) struct PortModel {
    pub fault: Fault,
    pub child_speed: u8,
    pub status: u16,
    pub change: u16,
    pub extended: u32,
    pub connection_samples: VecDeque<(bool, bool)>,
    pub reset_count: usize,
    pending: VecDeque<(u16, u16)>,
}
impl PortModel {
    pub fn connected() -> Self {
        Self {
            fault: Fault::None,
            child_speed: SUPER_SPEED,
            status: 3,
            change: 1,
            extended: 0x11,
            connection_samples: VecDeque::new(),
            reset_count: 0,
            pending: VecDeque::new(),
        }
    }
}
/// USB 3.0 Table 9-8 FUNCTION_SUSPEND options, kept per configured
/// interface. Pinned Linux ch9.h puts LP/RW in wIndex bits 8/9; hub.c's
/// usb_disable_remote_wakeup sends both clear through SET_FEATURE, not CLEAR.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub(in super::super) struct FunctionSuspend {
    pub low_power: bool,
    pub remote_wake: bool,
}

#[derive(Clone)]
pub(in super::super) struct Node {
    pub speed: u8,
    pub ports: Option<Vec<PortModel>>,
    pub bos: Option<Vec<u8>>,
    pub descriptor: Option<Vec<u8>>,
    pub configuration_override: Option<Vec<u8>>,
    pub configured: bool,
    pub functions: BTreeMap<u8, FunctionSuspend>,
    pub multi_tt: bool,
    active_alternate: u8,
    pub depth: Option<u16>,
    pub self_powered: bool,
    pub unconfigured_self_powered: Option<bool>,
    pub max_power: u8,
    pub interface_class: Option<u8>,
    pub controller_current: u8,
    pub hub_status: u16,
    pub hub_change: u16,
    pub after_power_status: Option<u16>,
    bot: Option<bot::Disk>,
    pub hid: Option<HidPeer>,
}

/// A boot keyboard (protocol 1) or mouse (2): its interrupt IN endpoint
/// answers only with queued reports, as a real device NAKs until a change.
#[derive(Clone, Default)]
pub(in super::super) struct HidPeer {
    pub protocol: u8,
    pub reports: VecDeque<Vec<u8>>,
}
impl Node {
    pub fn hub(speed: u8, ports: usize) -> Self {
        Self {
            speed,
            ports: Some(
                (0..ports)
                    .map(|_| {
                        let mut port = PortModel::connected();
                        port.child_speed = speed;
                        port
                    })
                    .collect(),
            ),
            bos: None,
            descriptor: None,
            configuration_override: None,
            configured: false,
            functions: BTreeMap::new(),
            multi_tt: false,
            active_alternate: 0,
            depth: None,
            self_powered: true,
            unconfigured_self_powered: None,
            max_power: 0,
            interface_class: None,
            controller_current: 0,
            hub_status: 0,
            hub_change: 0,
            after_power_status: None,
            bot: None,
            hid: None,
        }
    }
    pub fn leaf(speed: u8) -> Self {
        Self {
            speed,
            ports: None,
            bos: None,
            descriptor: None,
            configuration_override: None,
            configured: false,
            functions: BTreeMap::new(),
            multi_tt: false,
            active_alternate: 0,
            depth: None,
            self_powered: true,
            unconfigured_self_powered: None,
            max_power: 0,
            interface_class: None,
            controller_current: 0,
            hub_status: 0,
            hub_change: 0,
            after_power_status: None,
            bot: None,
            hid: None,
        }
    }
    /// A full-speed boot keyboard (`protocol` 1) or mouse (2).
    pub fn hid(protocol: u8) -> Self {
        Self {
            hid: Some(HidPeer {
                protocol,
                reports: VecDeque::new(),
            }),
            ..Self::leaf(FULL_SPEED)
        }
    }
    fn report_descriptor(&self) -> &'static [u8] {
        match self.hid.as_ref().map(|hid| hid.protocol) {
            Some(1) => &crate::usb::ehci::input::KEYBOARD_REPORT,
            _ => &crate::usb::ehci::input::MOUSE_REPORT,
        }
    }
    pub fn bot_disk(speed: u8) -> Self {
        Self {
            bot: Some(bot::Disk::new()),
            ..Self::leaf(speed)
        }
    }
    pub fn bot_commands(&self) -> &[(u32, u8)] {
        &self.bot.as_ref().expect("BOT peer required").commands
    }
    /// Hang the BOT peer until a port reset readdresses it
    /// ([`bot::Disk::wedged`]).
    pub fn wedge_bot(&mut self) {
        self.bot.as_mut().expect("BOT peer required").wedged = true;
    }
    pub fn bot_wedged(&self) -> bool {
        self.bot.as_ref().expect("BOT peer required").wedged
    }
    fn device(&self) -> Vec<u8> {
        let mut out = vec![
            18, 1, 0, 3, 0, 0, 0, 9, 0x34, 0x12, 0x78, 0x56, 0, 1, 0, 0, 0, 1,
        ];
        if self.ports.is_some() {
            out[4] = HUB_CLASS;
            out[6] = if self.speed >= SUPER_SPEED {
                3
            } else if self.multi_tt {
                2
            } else {
                1
            };
        }
        if self.speed < SUPER_SPEED {
            out[2..4].copy_from_slice(&0x0200u16.to_le_bytes());
            out[7] = 64;
        }
        if self.bos.is_some() {
            out[2..4].copy_from_slice(&0x0310u16.to_le_bytes());
        }
        out
    }
    pub fn configuration(&self) -> Vec<u8> {
        if let Some(configuration) = &self.configuration_override {
            return configuration.clone();
        }
        let hub = self.ports.is_some();
        if let Some(hid) = &self.hid {
            let report = self.report_descriptor().len() as u8;
            let mut out = vec![9, 2, 0, 0, 1, 1, 0, 0xa0, 50];
            out.extend_from_slice(&[9, 4, 0, 0, 1, 3, 1, hid.protocol, 0]);
            out.extend_from_slice(&[9, 0x21, 0x11, 1, 0, 1, 0x22, report, 0]);
            out.extend_from_slice(&[7, 5, 0x81, 3, 8, 0, 10]);
            let length = out.len() as u16;
            out[2..4].copy_from_slice(&length.to_le_bytes());
            return out;
        }
        let mut out = vec![
            9,
            2,
            0,
            0,
            1,
            1,
            0,
            0x80 | if self.self_powered { 0x40 } else { 0 },
            self.max_power,
            9,
            4,
            0,
            0,
            if hub { 1 } else { 2 },
            if hub {
                9
            } else {
                self.interface_class.unwrap_or(8)
            },
            if hub { 0 } else { 6 },
            if hub { 0 } else { 0x50 },
            0,
        ];
        if hub {
            out.extend_from_slice(&[7, 5, 0x81, 3, if self.multi_tt { 4 } else { 2 }, 0, 8]);
            if self.multi_tt {
                out.extend_from_slice(&[9, 4, 0, 1, 1, 9, 0, 2, 0, 7, 5, 0x82, 3, 4, 0, 8]);
            }
            if self.speed >= SUPER_SPEED {
                out.extend_from_slice(&[6, 0x30, 0, 0, 2, 0]);
            }
        } else {
            for address in [0x81, 2] {
                let packet = if self.speed >= SUPER_SPEED {
                    1024u16
                } else if self.speed == HIGH_SPEED {
                    512u16
                } else {
                    64u16
                };
                out.extend_from_slice(&[7, 5, address, 2, packet as u8, (packet >> 8) as u8, 0]);
                if self.speed >= SUPER_SPEED {
                    out.extend_from_slice(&[6, 0x30, 0, 0, 0, 0]);
                }
            }
        }
        let length = out.len() as u16;
        out[2..4].copy_from_slice(&length.to_le_bytes());
        out
    }
    fn hub_descriptor(&self) -> Vec<u8> {
        self.descriptor.clone().unwrap_or_else(|| {
            if self.speed >= SUPER_SPEED {
                vec![
                    12,
                    0x2a,
                    self.ports.as_ref().unwrap().len() as u8,
                    1,
                    0,
                    1,
                    self.controller_current,
                    0,
                    0,
                    0,
                    0,
                    0,
                ]
            } else {
                let ports = self.ports.as_ref().unwrap().len();
                let bitmap = (ports + 1).div_ceil(8);
                let mut bytes = vec![
                    (7 + 2 * bitmap) as u8,
                    0x29,
                    ports as u8,
                    0x61,
                    0,
                    1,
                    self.controller_current,
                ];
                bytes.resize(7 + bitmap, 0);
                bytes.resize(7 + 2 * bitmap, 0xff);
                bytes
            }
        })
    }
}

struct Cursor {
    at: u64,
    cycle: u32,
}
impl Cursor {
    fn new(pointer: u64) -> Self {
        Self {
            at: pointer & !0xf,
            cycle: pointer as u32 & 1,
        }
    }
    fn next(&mut self) -> (u64, u64, u32, u32) {
        let mut at = self.at;
        let mut control = read32(at + 12);
        if control >> 10 & 0x3f == LINK {
            assert_eq!(control & CYCLE, self.cycle);
            self.at = read64(at) & !0xf;
            if control & TOGGLE != 0 {
                self.cycle ^= 1;
            }
            at = self.at;
            control = read32(at + 12);
        }
        assert_eq!(
            control & CYCLE,
            self.cycle,
            "controller consumed unpublished TRB"
        );
        self.at += 16;
        (at, read64(at), read32(at + 8), control)
    }
}
fn read32(at: u64) -> u32 {
    assert!(
        at >= 65536 && at.is_multiple_of(4),
        "invalid fixture DMA word {at:#x}"
    );
    // SAFETY: caller uses only kept fixture DMA fields.
    unsafe { (at as *const u32).read_volatile() }
}
fn read64(at: u64) -> u64 {
    assert!(
        at >= 65536 && at.is_multiple_of(8),
        "invalid fixture DMA pointer {at:#x}"
    );
    // SAFETY: caller uses only kept fixture DMA fields.
    unsafe { (at as *const u64).read_volatile() }
}
fn write32(at: u64, value: u32) {
    // SAFETY: caller has the fixture's device-side ownership of this DMA.
    unsafe { (at as *mut u32).write_volatile(value) }
}
pub(in super::super) struct Hardware {
    pub nodes: BTreeMap<u32, Node>,
    pub requests: Vec<(u8, u32, [u8; 8])>,
    pub request_times: Vec<u64>,
    pub effect_fail: Option<(u8, [u8; 8])>,
    pub effect_silence: Option<(u8, [u8; 8])>,
    pub root_loss_after: Option<[u8; 8]>,
    pub contexts: Vec<(u8, u32, [u32; 4])>,
    pub commands: Vec<(u32, u8)>,
    pub fail: Option<(u32, u8)>,
    pub silent: Option<(u32, u8)>,
    /// The device at this route answers this many more requests, then none,
    /// and an Address Device for it fails as a controller's does for a
    /// device that does not answer SET_ADDRESS.
    pub answers: Option<(u32, usize)>,
    /// A silenced command, keeping CRCR.CRR set until it is aborted.
    stuck: Option<u64>,
    pub outgoing: Vec<Vec<u8>>,
    pub loss_after_child_route: Option<u32>,
    pub silent_configuration: Option<u8>,
    pub failed_configuration: Option<u8>,
    pub root_status: Option<u32>,
    /// A warm reset of the root port leaves its link untrained.
    pub root_untrainable: bool,
    /// A wedged BOT peer on the root port drops its link to SS.Inactive,
    /// showing a connection or not, as a device whose firmware hung may;
    /// then only a warm reset retrains it.
    pub wedge_drops_root_link: Option<bool>,
    pub notification_override: Option<u32>,
    pending_interrupts: BTreeMap<(u8, u8), (u64, u64, usize)>,
    /// Bulk TDs a wedged BOT peer never completes; a Stop Endpoint ends them.
    withheld: BTreeMap<(u8, u8), u64>,
    pub mmio_reads: usize,
    pub mmio_writes: usize,
    registers: BTreeMap<usize, u32>,
    running: bool,
    context_bytes: usize,
    command: Option<Cursor>,
    endpoints: BTreeMap<(u8, u8), Cursor>,
    outputs: BTreeMap<u8, u64>,
    routes: BTreeMap<u8, u32>,
    event_base: u64,
    events: usize,
    next_slot: u8,
    /// Disabled IDs, which Enable Slot hands out again, lowest first, once
    /// every ID up to the enabled slot count has been used.
    free_slots: alloc::collections::BTreeSet<u8>,
    pub(crate) slot_limit: u8,
}
impl Hardware {
    pub fn new(context_bytes: usize, nodes: BTreeMap<u32, Node>) -> Self {
        Self {
            nodes,
            requests: Vec::new(),
            request_times: Vec::new(),
            effect_fail: None,
            effect_silence: None,
            root_loss_after: None,
            contexts: Vec::new(),
            commands: Vec::new(),
            fail: None,
            silent: None,
            answers: None,
            stuck: None,
            outgoing: Vec::new(),
            loss_after_child_route: None,
            silent_configuration: None,
            failed_configuration: None,
            root_status: None,
            root_untrainable: false,
            wedge_drops_root_link: None,
            notification_override: None,
            pending_interrupts: BTreeMap::new(),
            withheld: BTreeMap::new(),
            mmio_reads: 0,
            mmio_writes: 0,
            registers: BTreeMap::new(),
            running: false,
            context_bytes,
            command: None,
            endpoints: BTreeMap::new(),
            outputs: BTreeMap::new(),
            routes: BTreeMap::new(),
            event_base: 0,
            events: 0,
            next_slot: 1,
            free_slots: Default::default(),
            slot_limit: 32,
        }
    }
    pub fn notify_root(&mut self) {
        self.notify_root_code(SUCCESS);
    }
    pub fn notify_root_code(&mut self, code: u32) {
        self.event(1 << 24, code, reset::PORT_STATUS_CHANGE, 0, 0, 0);
    }
    fn register64(&self, at: usize) -> u64 {
        u64::from(*self.registers.get(&at).unwrap_or(&0))
            | u64::from(*self.registers.get(&(at + 4)).unwrap_or(&0)) << 32
    }
    fn event(&mut self, parameter: u64, code: u32, kind: u32, slot: u8, dci: u8, residual: usize) {
        let at = self.event_base + ((self.events % RING_TRBS) * TRB) as u64;
        // SAFETY: event_base came from the published ERST allocation.
        unsafe {
            (at as *mut u64).write_volatile(parameter);
        }
        write32(at + 8, code << 24 | residual as u32);
        write32(
            at + 12,
            u32::from(slot) << 24
                | u32::from(dci) << 16
                | kind << 10
                | u32::from((self.events / RING_TRBS).is_multiple_of(2)),
        );
        self.events += 1;
    }
    /// A command abort: the stuck command completes aborted, then the ring stops.
    fn abort(&mut self) {
        if let Some(at) = self.stuck.take() {
            self.event(at, COMMAND_ABORTED, COMMAND_COMPLETION, 0, 0, 0);
            self.event(0, COMMAND_RING_STOPPED, COMMAND_COMPLETION, 0, 0, 0);
        }
    }
    fn command(&mut self) {
        let (at, parameter, _, control) = self.command.as_mut().unwrap().next();
        let kind = control >> 10 & 0x3f;
        let mut slot = (control >> 24) as u8;
        let dci = (control >> 16 & 31) as u8;
        self.commands.push((kind, slot));
        if self.silent == Some((kind, slot)) {
            self.stuck = Some(at);
            return;
        }
        let route = (kind == ADDRESS_DEVICE)
            .then(|| read32(parameter + self.context_bytes as u64) & 0xfffff);
        let code = if self.fail == Some((kind, slot)) {
            11
        } else if route.is_some()
            && self
                .answers
                .is_some_and(|(r, left)| Some(r) == route && left == 0)
        {
            transfer::USB_TRANSACTION_ERROR
        } else {
            SUCCESS
        };
        if code == SUCCESS {
            match kind {
                ENABLE_SLOT => {
                    slot = if self.next_slot <= self.slot_limit {
                        self.next_slot += 1;
                        self.next_slot - 1
                    } else {
                        self.free_slots.pop_first().unwrap_or(0)
                    };
                }
                ADDRESS_DEVICE | CONFIGURE_ENDPOINT | EVALUATE_CONTEXT => {
                    let output = read64(self.register64(0x40 + DCBAAP) + u64::from(slot) * 8);
                    self.outputs.insert(slot, output);
                    let previous_address = read32(output + 12) & 0x7f;
                    let add = read32(parameter + 4);
                    if kind == CONFIGURE_ENDPOINT {
                        let drop = read32(parameter);
                        for dci in 2..32 {
                            if drop & (1 << dci) != 0 {
                                self.endpoints.remove(&(slot, dci as u8));
                                write32(output + (dci * self.context_bytes) as u64, 0);
                            }
                        }
                    }
                    for dci in 0..32 {
                        if add & (1 << dci) == 0 {
                            continue;
                        }
                        let input = parameter + ((dci + 1) * self.context_bytes) as u64;
                        let target = output + (dci * self.context_bytes) as u64;
                        if kind == EVALUATE_CONTEXT && dci > 0 {
                            // Evaluate EP0 changes max packet only; unlike
                            // Configure it does not replace a dequeue pointer.
                            write32(
                                target + 4,
                                read32(target + 4) & 0xffff | read32(input + 4) & 0xffff_0000,
                            );
                            continue;
                        }
                        // SAFETY: copied complete kept xHCI contexts are disjoint.
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                input as *const u8,
                                target as *mut u8,
                                self.context_bytes,
                            );
                        }
                        if dci > 0 {
                            write32(target, read32(target) & !7 | ENDPOINT_RUNNING);
                            self.endpoints
                                .insert((slot, dci as u8), Cursor::new(read64(target + 8)));
                        }
                    }
                    // Hardware owns the USB address field across Configure
                    // Endpoint/Evaluate Context; input context zero is not a
                    // request to clear the address assigned by Address Device.
                    let address = if kind == ADDRESS_DEVICE {
                        u32::from(slot) + 32
                    } else {
                        previous_address
                    };
                    write32(output + 12, read32(output + 12) & !0x7f | address);
                    let words = [
                        read32(output),
                        read32(output + 4),
                        read32(output + 8),
                        read32(output + 12),
                    ];
                    let route = words[0] & 0xfffff;
                    self.routes.insert(slot, route);
                    // Only a port reset readdresses a device: it ends a
                    // wedged BOT peer's hang.
                    if kind == ADDRESS_DEVICE
                        && let Some(disk) = self
                            .nodes
                            .get_mut(&route)
                            .and_then(|node| node.bot.as_mut())
                    {
                        disk.wedged = false;
                        disk.reset();
                    }
                    self.contexts.push((slot, kind, words));
                    if kind == ADDRESS_DEVICE && self.loss_after_child_route == Some(route) {
                        self.nodes.get_mut(&0).unwrap().hub_status = 1;
                    }
                }
                reset::DISABLE_SLOT => {
                    self.free_slots.insert(slot);
                    assert!(self.pending_interrupts.keys().all(|(s, _)| *s != slot));
                    assert!(self.withheld.keys().all(|(s, _)| *s != slot));
                    for &(s, dci) in self.endpoints.keys() {
                        if s == slot {
                            assert_eq!(
                                read32(
                                    self.outputs[&slot]
                                        + u64::from(dci) * self.context_bytes as u64
                                ) & 7,
                                ENDPOINT_STOPPED,
                                "Disable Slot before acknowledged endpoint retirement"
                            );
                        }
                    }
                    self.endpoints.retain(|(s, _), _| *s != slot);
                }
                STOP_ENDPOINT | RESET_ENDPOINT => {
                    if let Some((trb, _, _)) = self.pending_interrupts.remove(&(slot, dci)) {
                        // Stopped transfer event precedes Stop ACK.
                        self.event(trb, 26, TRANSFER_EVENT, slot, dci, 0);
                    }
                    if let Some(trb) = self.withheld.remove(&(slot, dci)) {
                        self.event(trb, 26, TRANSFER_EVENT, slot, dci, 0);
                    }
                    let output = self.outputs[&slot] + u64::from(dci) * self.context_bytes as u64;
                    write32(output, read32(output) & !7 | ENDPOINT_STOPPED);
                }
                SET_DEQUEUE => {
                    self.endpoints.insert((slot, dci), Cursor::new(parameter));
                }
                _ => panic!("unexpected native hub command {kind}"),
            }
        }
        self.event(at, code, COMMAND_COMPLETION, slot, 0, 0);
    }
    fn request(&mut self, slot: u8, request: [u8; 8]) -> Result<Vec<u8>, u32> {
        let route = self.routes[&slot];
        self.requests.push((slot, route, request));
        self.request_times.push(
            Clock {
                ticks_per_microsecond: 1,
            }
            .now_us(),
        );
        let node = self
            .nodes
            .get_mut(&route)
            .expect("no fixture at published child route");
        let value = u16::from_le_bytes([request[2], request[3]]);
        let index = u16::from_le_bytes([request[4], request[5]]);
        let length = u16::from_le_bytes([request[6], request[7]]);
        match (request[0], request[1]) {
            (0x81, GET_DESCRIPTOR) if node.hid.is_some() && value == 0x2200 => {
                Ok(node.report_descriptor().to_vec())
            }
            // SET_IDLE, SET_PROTOCOL.
            (0x21, 0x0a | 0x0b) if node.hid.is_some() => Ok(Vec::new()),
            (0xa1, 0xfe) if node.bot.is_some() => {
                assert_eq!(request, setup(0xa1, 0xfe, 0, 0, 1));
                Ok(vec![0])
            }
            (0x21, 0xff) if node.bot.is_some() => {
                assert_eq!(request, setup(0x21, 0xff, 0, 0, 0));
                if node.bot.as_ref().unwrap().wedged {
                    return Err(6);
                }
                node.bot.as_mut().unwrap().reset();
                Ok(Vec::new())
            }
            (RECIPIENT_INTERFACE, SET_FEATURE) => {
                // Strict peer: only configured SuperSpeed interfaces,
                // selector zero, no data, and the two defined option bits.
                // A malformed request must STALL without changing any function.
                if node.speed < SUPER_SPEED
                    || !node.configured
                    || value != 0
                    || length != 0
                    || index & 0xfc00 != 0
                {
                    return Err(6);
                }
                let function = node.functions.get_mut(&(index as u8)).ok_or(6u32)?;
                *function = FunctionSuspend {
                    low_power: index & 0x0100 != 0,
                    remote_wake: index & 0x0200 != 0,
                };
                Ok(Vec::new())
            }
            (0x80, GET_STATUS) => Ok(vec![
                u8::from(if node.configured {
                    node.self_powered
                } else {
                    node.unconfigured_self_powered.unwrap_or(node.self_powered)
                }),
                0,
            ]),
            (0xa0, GET_STATUS) => {
                assert!(node.configured);
                let powered = node
                    .ports
                    .as_ref()
                    .unwrap()
                    .iter()
                    .all(|port| port.status & 1 << 9 != 0);
                let status = if powered {
                    node.after_power_status.unwrap_or(node.hub_status)
                } else {
                    node.hub_status
                };
                let mut bytes = Vec::new();
                bytes.extend_from_slice(&status.to_le_bytes());
                bytes.extend_from_slice(&node.hub_change.to_le_bytes());
                Ok(bytes)
            }
            (0x20, CLEAR_FEATURE) => {
                assert!(value <= 1);
                node.hub_change &= !(1 << value);
                Ok(Vec::new())
            }
            (0x80, GET_DESCRIPTOR) => match (value >> 8) as u8 {
                DEVICE_DESCRIPTOR => Ok(node.device()),
                CONFIGURATION_DESCRIPTOR => Ok(node.configuration()),
                15 => Ok(node.bos.clone().unwrap_or_else(|| vec![5, 15, 5, 0, 0])),
                _ => Err(6),
            },
            (0xa0, GET_DESCRIPTOR) => {
                assert_eq!(
                    value >> 8,
                    if node.speed >= SUPER_SPEED {
                        0x2a
                    } else {
                        0x29
                    }
                );
                Ok(node.hub_descriptor())
            }
            (RECIPIENT_INTERFACE, SET_INTERFACE) if node.multi_tt && index == 0 && value == 1 => {
                node.active_alternate = 1;
                Ok(Vec::new())
            }
            (0, SET_CONFIGURATION) => {
                assert!(value <= 1);
                if self.failed_configuration == Some(slot) {
                    return Err(6);
                }
                node.configured = value != 0;
                // Acknowledged SET_CONFIGURATION selects the descriptor
                // interfaces and resets each function's options. Alternate
                // settings do not create additional functions.
                node.functions.clear();
                if node.configured && node.speed >= SUPER_SPEED {
                    for interface in interfaces(&node.configuration()) {
                        if interface.alternate == 0 {
                            node.functions
                                .insert(interface.number, FunctionSuspend::default());
                        }
                    }
                }
                Ok(Vec::new())
            }
            (0x20, 12) => {
                assert!(node.configured);
                assert_eq!(
                    value,
                    (0..5).filter(|tier| route >> (tier * 4) & 15 != 0).count() as u16
                );
                node.depth = Some(value);
                Ok(Vec::new())
            }
            (0x23, 8) => {
                assert!(node.configured && node.speed == HIGH_SPEED);
                assert!((1..=node.ports.as_ref().unwrap().len() as u16).contains(&index));
                assert!(matches!(value >> 11 & 3, 0 | 2));
                assert_ne!(value >> 4 & 0x7f, 0);
                Ok(Vec::new())
            }
            (0x23, SET_FEATURE | CLEAR_FEATURE) | (0xa3, GET_STATUS) => {
                let ss = node.speed >= SUPER_SPEED;
                let configured = node.configured;
                let depth = node.depth;
                let port = &mut node.ports.as_mut().unwrap()[usize::from(index & 0xff) - 1];
                match (request[0], request[1]) {
                    (0x23, SET_FEATURE) if value == PORT_POWER => {
                        assert!(configured);
                        if ss {
                            assert!(depth.is_some());
                        }
                        if port.fault == Fault::Power {
                            return Err(6);
                        }
                        if port.fault != Fault::PowerAbsent {
                            port.status |= if ss { 1 << 9 } else { 1 << 8 };
                        }
                    }
                    (0x23, SET_FEATURE) if value == 5 && ss => {
                        assert!(configured && depth.is_some());
                        let link = index >> 8;
                        assert!(matches!(link, 0 | 3));
                        if !(link == 3 && port.fault == Fault::SuspendIgnored
                            || link == 0 && port.fault == Fault::ResumeIgnored)
                        {
                            port.status = port.status & !(15 << 5) | link << 5;
                            port.change |= 1 << 6;
                        }
                    }
                    (0x23, SET_FEATURE) if value == 2 && !ss => {
                        if port.fault != Fault::SuspendIgnored {
                            port.status |= 1 << 2;
                        }
                    }
                    (0x23, CLEAR_FEATURE) if value == 2 && !ss => {
                        if port.fault != Fault::ResumeIgnored {
                            port.status &= !(1 << 2);
                            port.change |= 1 << 2;
                        }
                    }
                    (0x23, SET_FEATURE) if value == PORT_RESET || value == 28 => {
                        assert!(configured);
                        if ss {
                            assert!(depth.is_some());
                        }
                        assert!(
                            port.connection_samples.is_empty(),
                            "reset before connection settled"
                        );
                        if port.fault == Fault::Reset {
                            return Err(6);
                        }
                        let power = if ss { 1 << 9 } else { 1 << 8 };
                        assert_ne!(port.status & power, 0);
                        if ss && matches!(port.status & 0x1e0, 0xc0 | 0x140) {
                            assert_eq!(value, 28);
                        }
                        port.reset_count += 1;
                        let mut change = if value == 28 { 1 << 5 } else { 1 << 4 };
                        port.status = (port.status | 1 << 4) & !2;
                        let mut status = power | 3;
                        if !ss {
                            status |= match port.child_speed {
                                LOW_SPEED => 1 << 9,
                                HIGH_SPEED => 1 << 10,
                                _ => 0,
                            };
                        }
                        match port.fault {
                            Fault::ResetForever => status |= 1 << 4,
                            Fault::Disconnect => status = power,
                            Fault::NoEnable => status &= !2,
                            Fault::FirstResetDisabled if port.reset_count == 1 => status &= !2,
                            Fault::FirstResetNotU0 if port.reset_count == 1 => status |= 5 << 5,
                            Fault::HotThenWarm if value == PORT_RESET => status |= 0xc0,
                            Fault::WarmForever => status |= 0x140,
                            Fault::LinkNotU0 => status |= 1 << 5,
                            Fault::ConfigError => change |= 1 << 7,
                            Fault::ConnectionChange => change |= 1,
                            Fault::OverCurrentChange => change |= 1 << 3,
                            Fault::NoResetChange => change = 0,
                            _ => {}
                        }
                        if port.fault == Fault::ResetConnectionBounce {
                            port.pending.push_back((power | 1 << 4, 0));
                            port.pending.push_back((power, change | 1));
                            change |= 1;
                        }
                        port.pending.push_back((status, change));
                        if port.fault == Fault::ResetRecoveryLoss {
                            port.pending.push_back((power, 1));
                        }
                    }
                    (0x23, CLEAR_FEATURE) => {
                        if port.fault == Fault::ClearChange {
                            return Err(6);
                        }
                        let bit = match value {
                            16 => 1,
                            17 if !ss => 1 << 1,
                            18 if !ss => 1 << 2,
                            23 if !ss => 1 << 5,
                            19 => 1 << 3,
                            20 => 1 << 4,
                            29 if ss => 1 << 5,
                            25 if ss => 1 << 6,
                            26 if ss => 1 << 7,
                            _ => panic!("USB2/reserved change selector used on SS port {value}"),
                        };
                        port.change &= !bit;
                    }
                    (0xa3, GET_STATUS) => {
                        if let Some((connected, changed)) = port.connection_samples.pop_front() {
                            port.status = (port.status & !1) | u16::from(connected);
                            port.change |= u16::from(changed);
                        }
                        if let Some((status, change)) = port.pending.pop_front() {
                            port.status = status;
                            port.change |= change;
                        }
                        let mut bytes = Vec::new();
                        bytes.extend_from_slice(&port.status.to_le_bytes());
                        bytes.extend_from_slice(&port.change.to_le_bytes());
                        if value == 2 {
                            bytes.extend_from_slice(&port.extended.to_le_bytes());
                        }
                        if port.fault == Fault::ShortStatus {
                            bytes.pop();
                        }
                        return Ok(bytes);
                    }
                    _ => panic!("unexpected hub request {request:?}"),
                }
                Ok(Vec::new())
            }
            _ => Err(6),
        }
    }
    fn transfer(&mut self, slot: u8, dci: u8) {
        // A doorbell restarts an acknowledged stopped ring. A completed TD
        // leaves it Running, so recovery must Stop EP0 again before Disable.
        let output = self.outputs[&slot] + u64::from(dci) * self.context_bytes as u64;
        write32(output, read32(output) & !7 | ENDPOINT_RUNNING);
        if dci == 1 {
            let cursor = self.endpoints.get_mut(&(slot, dci)).unwrap();
            let (_, parameter, _, setup_control) = cursor.next();
            assert_eq!(setup_control >> 10 & 0x3f, SETUP);
            let request = parameter.to_le_bytes();
            let requested = usize::from(u16::from_le_bytes([request[6], request[7]]));
            let mut data = None;
            if requested > 0 {
                let td = cursor.next();
                assert_eq!(td.3 >> 10 & 0x3f, DATA);
                data = Some(td);
            }
            let (status, _, _, control) = cursor.next();
            assert_eq!(control >> 10 & 0x3f, STATUS);
            if let Some((route, left)) = &mut self.answers
                && self.routes.get(&slot) == Some(route)
            {
                if *left == 0 {
                    return;
                }
                *left -= 1;
            }
            let mut reply = self.request(slot, request);
            if self.root_loss_after == Some(request) {
                self.root_status = Some(PP);
                self.root_loss_after = None;
            }
            if self.effect_fail == Some((slot, request)) {
                self.effect_fail = None;
                reply = Err(6);
            }
            if self.effect_silence == Some((slot, request)) {
                self.effect_silence = None;
                return;
            }
            match reply {
                Ok(mut reply) => {
                    if request[0] == 0
                        && request[1] == SET_CONFIGURATION
                        && self.silent_configuration == Some(slot)
                    {
                        return;
                    }
                    if let Some((address, buffer, length, _)) = data {
                        reply.truncate(requested);
                        assert_eq!(length as usize, requested);
                        // SAFETY: DATA TRB gives this device-side live DMA range.
                        unsafe {
                            core::ptr::copy_nonoverlapping(
                                reply.as_ptr(),
                                buffer as *mut u8,
                                reply.len(),
                            );
                        }
                        if reply.len() < requested {
                            self.event(
                                address,
                                SHORT_PACKET,
                                TRANSFER_EVENT,
                                slot,
                                dci,
                                requested - reply.len(),
                            );
                        }
                    }
                    self.event(status, SUCCESS, TRANSFER_EVENT, slot, dci, 0);
                }
                Err(code) => {
                    let output = self.outputs[&slot] + u64::from(dci) * self.context_bytes as u64;
                    write32(output, read32(output) & !7 | ENDPOINT_HALTED);
                    self.event(status, code, TRANSFER_EVENT, slot, dci, 0);
                }
            }
        } else {
            let (trb, buffer, length, control) =
                self.endpoints.get_mut(&(slot, dci)).unwrap().next();
            assert_eq!(control >> 10 & 0x3f, NORMAL);
            if self.nodes[&self.routes[&slot]].hid.is_some() {
                assert_eq!(dci, 3);
                assert!(
                    self.pending_interrupts
                        .insert((slot, dci), (trb, buffer, length as usize))
                        .is_none()
                );
                self.complete_interrupts();
                return;
            }
            if self.nodes[&self.routes[&slot]].ports.is_some() {
                assert_eq!(
                    dci,
                    if self.nodes[&self.routes[&slot]].active_alternate == 1 {
                        5
                    } else {
                        3
                    }
                );
                assert!(
                    self.pending_interrupts
                        .insert((slot, dci), (trb, buffer, length as usize))
                        .is_none()
                );
                self.complete_interrupts();
                return;
            }
            if let Some(disk) = self
                .nodes
                .get_mut(&self.routes[&slot])
                .unwrap()
                .bot
                .as_mut()
            {
                if disk.wedged {
                    assert!(self.withheld.insert((slot, dci), trb).is_none());
                    if self.routes[&slot] == 0
                        && let Some(connected) = self.wedge_drops_root_link
                        && let Some(status) = self.root_status.as_mut()
                    {
                        // PLS Inactive with Port Link State Change.
                        *status = *status & !(CCS | PED | PLS)
                            | u32::from(connected)
                            | LINK_INACTIVE << PLS_SHIFT
                            | 1 << 22;
                    }
                    return;
                }
                disk.transfer(dci & 1 != 0, buffer, length as usize);
            } else if dci & 1 != 0 {
                // SAFETY: published IN payload belongs to this fixture.
                unsafe {
                    core::ptr::write_bytes(buffer as *mut u8, slot, length as usize);
                }
            } else {
                // SAFETY: published OUT payload belongs to this fixture.
                self.outgoing.push(
                    unsafe { core::slice::from_raw_parts(buffer as *const u8, length as usize) }
                        .to_vec(),
                );
            }
            self.event(trb, SUCCESS, TRANSFER_EVENT, slot, dci, 0);
        }
    }
}
impl Hardware {
    fn complete_interrupts(&mut self) {
        let mut completed = Vec::new();
        let mut reports = Vec::new();
        for (&(slot, dci), &(trb, buffer, length)) in &self.pending_interrupts {
            let node = &self.nodes[&self.routes[&slot]];
            if let Some(hid) = &node.hid {
                if !hid.reports.is_empty() {
                    reports.push((slot, dci, trb, buffer, length));
                }
                continue;
            }
            let mut bits = u32::from(node.hub_change != 0);
            for (port, state) in node.ports.as_ref().unwrap().iter().enumerate() {
                if state.change != 0 {
                    bits |= 1 << (port + 1);
                }
            }
            let bits = self.notification_override.unwrap_or(bits);
            if bits == 0 {
                continue;
            } // Real hub NAKs while no change is pending.
            let bytes = bits.to_le_bytes();
            // SAFETY: only the published interrupt TD owns this device write.
            unsafe {
                core::ptr::copy_nonoverlapping(bytes.as_ptr(), buffer as *mut u8, length);
            }
            completed.push((slot, dci, trb));
        }
        for (slot, dci, trb) in completed {
            self.pending_interrupts.remove(&(slot, dci));
            self.event(trb, SUCCESS, TRANSFER_EVENT, slot, dci, 0);
        }
        for (slot, dci, trb, buffer, length) in reports {
            let route = self.routes[&slot];
            let hid = self.nodes.get_mut(&route).unwrap().hid.as_mut().unwrap();
            let report = hid.reports.pop_front().unwrap();
            let n = report.len().min(length);
            // SAFETY: only the published interrupt TD owns this device write.
            unsafe { core::ptr::copy_nonoverlapping(report.as_ptr(), buffer as *mut u8, n) };
            self.pending_interrupts.remove(&(slot, dci));
            self.event(
                trb,
                if n < length { SHORT_PACKET } else { SUCCESS },
                TRANSFER_EVENT,
                slot,
                dci,
                length - n,
            );
        }
    }
}
pub(in super::super) struct RegistersModel(pub Rc<RefCell<Hardware>>);
impl Registers for RegistersModel {
    fn read32(&mut self, at: usize) -> u32 {
        let mut state = self.0.borrow_mut();
        state.mmio_reads += 1;
        match at {
            0 => 0x0110_0040,
            HCSPARAMS1 => 1 << 24 | 32,
            HCCPARAMS1 => 1 | 0x20 << 16 | if state.context_bytes == 64 { 4 } else { 0 },
            DBOFF => 0x1000,
            RTSOFF => 0x800,
            0x40 => {
                if state.running {
                    RUN
                } else {
                    0
                }
            }
            0x44 => {
                state.complete_interrupts();
                if state.running { 0 } else { HCH }
            }
            0x48 => 1,
            0x58 if state.stuck.is_some() => CRCR_RUNNING,
            0x80 => {
                (if state.nodes[&0].speed >= SUPER_SPEED {
                    3
                } else {
                    2
                }) << 24
                    | SUPPORTED_PROTOCOL
            }
            0x88 => 1 | 1 << 8,
            0x440 => {
                // Hubs answer their interrupt TDs while the host samples ports.
                state.complete_interrupts();
                state
                    .root_status
                    .unwrap_or(PP | CCS | PED | PRC | u32::from(state.nodes[&0].speed) << 10)
            }
            _ => *state.registers.get(&at).unwrap_or(&0),
        }
    }
    fn write32(&mut self, at: usize, value: u32) {
        let mut state = self.0.borrow_mut();
        state.mmio_writes += 1;
        state.registers.insert(at, value);
        if at == 0x40 + CRCR && value & CRCR_ABORT != 0 {
            state.abort();
        }
        if at == 0x440 {
            let actual_speed = state.nodes[&0].speed;
            let untrainable = state.root_untrainable;
            if let Some(status) = state.root_status.as_mut() {
                *status &= !(value & PORT_CHANGES);
                if value & PR != 0 && *status >> PLS_SHIFT & PLS_FIELD == LINK_INACTIVE {
                    // A hot reset cannot retrain an SS.Inactive link: it
                    // completes with the port still disabled.
                    *status = *status & !PR | PRC;
                } else if value & PR != 0 {
                    // The device handshake determines speed when reset enables
                    // the port; its pre-reset speed field may be undefined.
                    *status = *status & !(PR | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT))
                        | PED
                        | PRC
                        | u32::from(actual_speed) << PORT_SPEED_SHIFT;
                }
                if value & WPR != 0 {
                    // A warm reset clears Cold Attach Status and retrains
                    // the link to U0 (xHCI 1.2 4.19.5.1).
                    let trained = if untrainable {
                        0
                    } else {
                        CCS | PED | u32::from(actual_speed) << PORT_SPEED_SHIFT
                    };
                    *status = *status
                        & !(CAS | CCS | PED | PLS | (PORT_SPEED_FIELD << PORT_SPEED_SHIFT))
                        | WRC
                        | PRC
                        | trained;
                }
            }
            if value & (PR | WPR) != 0 {
                state.event(1 << 24, SUCCESS, reset::PORT_STATUS_CHANGE, 0, 0, 0);
            }
        }
        if at == 0x40 {
            state.running = value & RUN != 0;
            if state.running {
                state.command = Some(Cursor::new(state.register64(0x40 + CRCR)));
                state.event_base = read64(state.register64(0x800 + ERSTBA));
            }
        }
        if at == 0x1000 {
            state.command();
        } else if (0x1004..0x1084).contains(&at) {
            state.transfer(((at - 0x1000) / 4) as u8, (value & 255) as u8);
        }
    }
}

#[cfg(test)]
mod function_suspend_tests {
    use super::*;

    fn peer(speed: u8) -> Hardware {
        let mut hardware = Hardware::new(64, BTreeMap::from([(0, Node::leaf(speed))]));
        hardware.routes.insert(1, 0);
        hardware
    }

    #[test]
    fn function_suspend_requires_configured_superspeed_interface_and_preserves_other_functions() {
        let mut hardware = peer(SUPER_SPEED);
        let configuration = vec![
            9, 2, 36, 0, 2, 1, 0, 0x80, 0, 9, 4, 0, 0, 0, 0xff, 0, 0, 0, 9, 4, 7, 0, 0, 0xff, 0, 0,
            0, 9, 4, 7, 1, 0, 0xff, 0, 0, 0,
        ];
        hardware.nodes.get_mut(&0).unwrap().configuration_override = Some(configuration);
        assert_eq!(
            hardware.request(1, setup(1, SET_FEATURE, 0, 0x0307, 0)),
            Err(6)
        );
        assert!(hardware.nodes[&0].functions.is_empty());
        hardware
            .request(1, setup(0, SET_CONFIGURATION, 1, 0, 0))
            .unwrap();
        assert_eq!(hardware.nodes[&0].functions.len(), 2);
        for (options, expected) in [
            (
                0x0300,
                FunctionSuspend {
                    low_power: true,
                    remote_wake: true,
                },
            ),
            (
                0x0100,
                FunctionSuspend {
                    low_power: true,
                    remote_wake: false,
                },
            ),
            (
                0x0200,
                FunctionSuspend {
                    low_power: false,
                    remote_wake: true,
                },
            ),
            (0, FunctionSuspend::default()),
        ] {
            hardware
                .request(1, setup(1, SET_FEATURE, 0, options | 7, 0))
                .unwrap();
            assert_eq!(hardware.nodes[&0].functions[&7], expected);
            assert_eq!(hardware.nodes[&0].functions[&0], FunctionSuspend::default());
        }
        hardware
            .request(1, setup(1, SET_FEATURE, 0, 0x0307, 0))
            .unwrap();
        hardware
            .request(1, setup(0, SET_CONFIGURATION, 1, 0, 0))
            .unwrap();
        assert_eq!(hardware.nodes[&0].functions[&7], FunctionSuspend::default());
        hardware
            .request(1, setup(0, SET_CONFIGURATION, 0, 0, 0))
            .unwrap();
        assert!(hardware.nodes[&0].functions.is_empty());
        assert_eq!(hardware.request(1, setup(1, SET_FEATURE, 0, 7, 0)), Err(6));
    }

    #[test]
    fn invalid_function_suspend_requests_stall_without_mutating_configured_state() {
        for speed in [HIGH_SPEED, SUPER_SPEED] {
            let mut hardware = peer(speed);
            hardware
                .request(1, setup(0, SET_CONFIGURATION, 1, 0, 0))
                .unwrap();
            if speed >= SUPER_SPEED {
                hardware
                    .request(1, setup(1, SET_FEATURE, 0, 0x0300, 0))
                    .unwrap();
            }
            let before = hardware.nodes[&0].functions.clone();
            let mut invalid = vec![
                setup(1, SET_FEATURE, 1, 0, 0),
                setup(1, SET_FEATURE, 0, 1, 0),
                setup(1, SET_FEATURE, 0, 0x0400, 0),
                setup(1, SET_FEATURE, 0, 0, 1),
                setup(1, CLEAR_FEATURE, 0, 0, 0),
            ];
            if speed < SUPER_SPEED {
                invalid.push(setup(1, SET_FEATURE, 0, 0, 0));
            }
            for request in invalid {
                assert_eq!(hardware.request(1, request), Err(6), "{request:?}");
                assert_eq!(hardware.nodes[&0].functions, before);
            }
        }
    }
}
