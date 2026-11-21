//! Bounded native USB2 high-speed host. USB 2.0 chapter 9 setup packets are
//! sent to the device's EP0; all descriptors are guest bytes. The mux interface
//! tuple ff/fe/02 is from AppleUSBDeviceMux::start, KC file 0x321fc60..cb8,
//! independently matched by usbmuxd 3ded00c `src/usb.c` descriptor discovery.
use super::{BusPeer, MAX_PACKET, Reply, Token};
use crate::Result;
use alloc::{collections::VecDeque, vec, vec::Vec};
use stela_contracts::refuse;

const EP0_PACKET: usize = 64;
const MAX_DESCRIPTOR: usize = 8192;
const MAX_CONFIGURATIONS: u8 = 8;
const MAX_TRANSFERS: usize = 8;
pub const MAX_TRANSFER: usize = 65536;
const REQUEST_TIMEOUT: u64 = 5 * crate::counter::FREQUENCY_HZ;
pub(crate) const CONTROL_TIMEOUT: &str = "USB control request timed out";
const RESET_RECOVERY: u64 = crate::counter::FREQUENCY_HZ / 100;
const GET_DESCRIPTOR: u8 = 6;
const SET_ADDRESS: u8 = 5;
const SET_CONFIGURATION: u8 = 9;
const DEVICE: u8 = 1;
const CONFIGURATION: u8 = 2;
const STRING: u8 = 3;
const BOS: u8 = 15;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Setup {
    pub request_type: u8,
    pub request: u8,
    pub value: u16,
    pub index: u16,
    pub length: u16,
}
impl Setup {
    pub fn bytes(self) -> [u8; 8] {
        let value = self.value.to_le_bytes();
        let index = self.index.to_le_bytes();
        let length = self.length.to_le_bytes();
        [
            self.request_type,
            self.request,
            value[0],
            value[1],
            index[0],
            index[1],
            length[0],
            length[1],
        ]
    }
    fn descriptor(kind: u8, index: u8, language: u16, length: u16) -> Self {
        Self {
            request_type: 0x80,
            request: GET_DESCRIPTOR,
            value: u16::from(kind) << 8 | u16::from(index),
            index: language,
            length,
        }
    }
    fn set(request: u8, value: u16) -> Self {
        Self {
            request_type: 0,
            request,
            value,
            index: 0,
            length: 0,
        }
    }
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Descriptor {
    pub kind: u8,
    pub index: u8,
    pub language: u16,
    pub bytes: Vec<u8>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct MuxEndpoints {
    pub configuration: u8,
    pub interface: u8,
    pub input: u8,
    pub output: u8,
    pub input_packet: u16,
    pub output_packet: u16,
}

/// Strict walk of one configuration; endpoint numbers are never constants.
/// Alternate settings other than zero require SET_INTERFACE and stay explicit.
pub fn mux_endpoints(bytes: &[u8]) -> Result<Option<MuxEndpoints>> {
    if bytes.len() < 9
        || bytes[0] != 9
        || bytes[1] != CONFIGURATION
        || usize::from(u16::from_le_bytes([bytes[2], bytes[3]])) != bytes.len()
        || bytes[5] == 0
    {
        return Err(refuse!("invalid USB configuration header or total length"));
    }
    let mut found = None;
    let mut interface = None;
    let mut input = None;
    let mut output = None;
    let mut addresses = 0u32;
    let mut offset = 9;
    while offset < bytes.len() {
        let size = usize::from(bytes[offset]);
        if size < 2 || size > bytes.len() - offset {
            return Err(refuse!("truncated or zero-length USB descriptor"));
        }
        let descriptor = &bytes[offset..offset + size];
        match descriptor[1] {
            4 => {
                if size != 9 {
                    return Err(refuse!("invalid USB interface descriptor length"));
                }
                if interface.is_some() && (input.is_none() || output.is_none()) {
                    return Err(refuse!("mux interface lacks opposite bulk endpoints"));
                }
                interface = if descriptor[3] == 0 && descriptor[5..8] == [0xff, 0xfe, 2] {
                    if descriptor[4] != 2 {
                        return Err(refuse!("mux interface requires two endpoints"));
                    }
                    Some(descriptor[2])
                } else {
                    None
                };
                input = None;
                output = None;
            }
            5 if interface.is_some() => {
                if size != 7 || descriptor[3] != 2 {
                    return Err(refuse!("mux endpoint is not a USB2 bulk endpoint"));
                }
                let address = descriptor[2];
                let packet = u16::from_le_bytes([descriptor[4], descriptor[5]]);
                if address & 0x70 != 0 || address & 0xf == 0 || packet != MAX_PACKET as u16 {
                    return Err(refuse!(
                        "invalid high-speed bulk endpoint address or max packet"
                    ));
                }
                let bit = u32::from(address & 0xf) * 2 + u32::from(address & 0x80 != 0);
                if addresses & (1 << bit) != 0 {
                    return Err(refuse!("duplicate USB mux endpoint address"));
                }
                addresses |= 1 << bit;
                let slot = if address & 0x80 != 0 {
                    &mut input
                } else {
                    &mut output
                };
                if slot.replace((address, packet)).is_some() {
                    return Err(refuse!("duplicate USB mux endpoint direction"));
                }
                if let (Some((input, input_packet)), Some((output, output_packet))) =
                    (input, output)
                {
                    if found.is_some() {
                        return Err(refuse!("ambiguous USB mux interfaces"));
                    }
                    found = Some(MuxEndpoints {
                        configuration: bytes[5],
                        interface: interface.unwrap(),
                        input,
                        output,
                        input_packet,
                        output_packet,
                    });
                }
            }
            _ => {}
        }
        offset += size;
    }
    if interface.is_some() && (input.is_none() || output.is_none()) {
        return Err(refuse!("mux interface lacks opposite bulk endpoints"));
    }
    Ok(found)
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Stage {
    Setup,
    Data,
    Status,
}
struct Control {
    setup: Setup,
    stage: Stage,
    data: Vec<u8>,
    deadline: u64,
}
impl Control {
    fn new(setup: Setup, now: u64) -> Self {
        Self {
            setup,
            stage: Stage::Setup,
            data: Vec::new(),
            deadline: now.saturating_add(REQUEST_TIMEOUT),
        }
    }
    fn step(&mut self, peer: &mut impl BusPeer, now: u64) -> Result<bool> {
        if now >= self.deadline {
            return Err(refuse!(
                CONTROL_TIMEOUT,
                stage = ?self.stage,
                request_type = self.setup.request_type,
                request = self.setup.request,
                value = self.setup.value,
                index = self.setup.index,
                length = self.setup.length,
                received = self.data.len(),
            ));
        }
        let (endpoint, token, mut packet) = match self.stage {
            Stage::Setup => (0, Token::Setup, self.setup.bytes().to_vec()),
            Stage::Data => (
                0x80,
                Token::In,
                vec![0; EP0_PACKET.min(usize::from(self.setup.length) - self.data.len())],
            ),
            Stage::Status => {
                if self.setup.request_type & 0x80 != 0 {
                    (0, Token::Out, Vec::new())
                } else {
                    (0x80, Token::In, Vec::new())
                }
            }
        };
        let bytes = match peer.packet(endpoint, token, &mut packet)? {
            Reply::Nak => return Ok(false),
            Reply::Stall => return Err(refuse!("USB control request stalled")),
            Reply::Ack(bytes) => bytes,
        };
        if bytes > packet.len() {
            return Err(refuse!("USB peer returned an oversized control packet"));
        }
        match self.stage {
            Stage::Setup => {
                if bytes != 8 {
                    return Err(refuse!("USB setup acknowledgement is not eight bytes"));
                }
                if self.setup.length == 0 {
                    self.stage = Stage::Status;
                } else if self.setup.request_type & 0x80 == 0 {
                    return Err(refuse!("USB host control OUT data is not implemented"));
                } else {
                    self.stage = Stage::Data;
                }
            }
            Stage::Data => {
                self.data.extend_from_slice(&packet[..bytes]);
                if bytes < EP0_PACKET || self.data.len() == usize::from(self.setup.length) {
                    self.stage = Stage::Status;
                }
            }
            Stage::Status => {
                if bytes != 0 {
                    return Err(refuse!("USB status acknowledgement is not a ZLP"));
                }
                return Ok(true);
            }
        }
        Ok(false)
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
enum Enumerate {
    Detached,
    Reset,
    Recover(u64),
    Device,
    Address,
    ConfigHeader(u8),
    Config(u8, u16),
    Languages,
    Strings(usize),
    BosHeader,
    Bos(u16),
    Configure,
    Ready,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum TransferStatus {
    Complete,
    Stall,
    Cancelled,
}
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Completion {
    pub id: u64,
    pub status: TransferStatus,
    pub bytes: Vec<u8>,
    pub transferred: usize,
}
struct Bulk {
    id: u64,
    input: bool,
    data: Vec<u8>,
    length: usize,
    done: usize,
    zlp: bool,
}

pub struct Host {
    state: Enumerate,
    generation: Option<u64>,
    control: Option<Control>,
    descriptors: Vec<Descriptor>,
    endpoints: Option<MuxEndpoints>,
    configurations: u8,
    strings: Vec<u8>,
    language: u16,
    bos: bool,
    next_id: u64,
    bulk: VecDeque<Bulk>,
    completions: VecDeque<Completion>,
    prefer_input: bool,
}
impl Default for Host {
    fn default() -> Self {
        Self::new()
    }
}
impl Host {
    pub fn new() -> Self {
        Self {
            state: Enumerate::Detached,
            generation: None,
            control: None,
            descriptors: Vec::new(),
            endpoints: None,
            configurations: 0,
            strings: Vec::new(),
            language: 0,
            bos: false,
            next_id: 1,
            bulk: VecDeque::new(),
            completions: VecDeque::new(),
            prefer_input: false,
        }
    }
    pub fn attach(&mut self, generation: u64) {
        if self.generation == Some(generation) {
            return;
        }
        self.detach();
        self.generation = Some(generation);
        self.state = Enumerate::Reset;
    }
    pub fn detach(&mut self) {
        for transfer in self.bulk.drain(..) {
            self.completions.push_back(Completion {
                id: transfer.id,
                status: TransferStatus::Cancelled,
                bytes: transfer.data,
                transferred: transfer.done,
            });
        }
        self.control = None;
        self.generation = None;
        self.endpoints = None;
        self.descriptors.clear();
        self.strings.clear();
        self.bos = false;
        self.state = Enumerate::Detached;
    }
    pub fn ready(&self) -> bool {
        self.state == Enumerate::Ready
    }
    pub fn descriptors(&self) -> &[Descriptor] {
        &self.descriptors
    }
    pub fn completion(&mut self) -> Option<Completion> {
        self.completions.pop_front()
    }
    /// A bulk OUT of `data`, which waits for the device as long as it takes:
    /// usbmuxd submits its transfers with no timeout.
    pub fn submit_out(&mut self, data: &[u8], terminate_with_zlp: bool) -> Result<u64> {
        if data.len() > MAX_TRANSFER {
            return Err(refuse!("USB host transfer bound exceeded"));
        }
        self.submit_out_owned(data.to_vec(), terminate_with_zlp)
    }
    /// The same transfer, keeping an existing frame buffer through completion.
    pub(super) fn submit_out_owned(
        &mut self,
        data: Vec<u8>,
        terminate_with_zlp: bool,
    ) -> Result<u64> {
        let length = data.len();
        self.submit(false, data, length, terminate_with_zlp)
    }
    /// A bulk IN of up to `length` bytes, pending until the device sends.
    pub fn submit_in(&mut self, length: usize) -> Result<u64> {
        self.submit(true, Vec::new(), length, false)
    }
    fn submit(&mut self, input: bool, data: Vec<u8>, length: usize, zlp: bool) -> Result<u64> {
        if !self.ready() {
            return Err(refuse!("USB host is not configured"));
        }
        if length > MAX_TRANSFER || self.bulk.len() + self.completions.len() >= MAX_TRANSFERS {
            return Err(refuse!("USB host transfer bound exceeded"));
        }
        let id = self.next_id;
        self.next_id = id
            .checked_add(1)
            .ok_or(refuse!("USB transfer identifier exhausted"))?;
        self.bulk.push_back(Bulk {
            id,
            input,
            data,
            length,
            done: 0,
            zlp,
        });
        Ok(id)
    }
    /// At most one packet per call. The caller supplies the monotonic guest
    /// 24 MHz counter and runs guest code between NAKs; this never does so.
    /// Whether another call at the same `now` can move a packet: not once
    /// every pending bulk transfer NAKed, none is pending, or the port waits
    /// out its reset, so a caller with packets to spend stops there.
    pub fn advance(&mut self, peer: &mut impl BusPeer, now: u64) -> Result<bool> {
        match self.state {
            Enumerate::Detached => return Ok(false),
            Enumerate::Reset => {
                peer.reset()?;
                self.state = Enumerate::Recover(now.saturating_add(RESET_RECOVERY));
                return Ok(false);
            }
            Enumerate::Recover(deadline) => {
                let recovered = now >= deadline;
                if recovered {
                    self.state = Enumerate::Device;
                }
                return Ok(recovered);
            }
            Enumerate::Ready => return self.bulk_step(peer),
            _ => {}
        }
        if self.control.is_none() {
            let setup = match self.state {
                Enumerate::Device => Setup::descriptor(DEVICE, 0, 0, 18),
                Enumerate::Address => Setup::set(SET_ADDRESS, 1),
                Enumerate::ConfigHeader(n) => Setup::descriptor(CONFIGURATION, n, 0, 9),
                Enumerate::Config(n, length) => Setup::descriptor(CONFIGURATION, n, 0, length),
                Enumerate::Languages => Setup::descriptor(STRING, 0, 0, 255),
                Enumerate::Strings(n) => {
                    Setup::descriptor(STRING, self.strings[n], self.language, 255)
                }
                Enumerate::BosHeader => Setup::descriptor(BOS, 0, 0, 5),
                Enumerate::Bos(length) => Setup::descriptor(BOS, 0, 0, length),
                Enumerate::Configure => Setup::set(
                    SET_CONFIGURATION,
                    u16::from(
                        self.endpoints
                            .ok_or(refuse!("USB device has no mux interface"))?
                            .configuration,
                    ),
                ),
                _ => unreachable!(),
            };
            self.control = Some(Control::new(setup, now));
        }
        if !self.control.as_mut().unwrap().step(peer, now)? {
            return Ok(true);
        }
        let request = self.control.take().unwrap();
        let bytes = request.data;
        self.state = match self.state {
            Enumerate::Device => {
                if bytes.len() != 18
                    || bytes[..2] != [18, DEVICE]
                    || bytes[7] != EP0_PACKET as u8
                    || bytes[17] == 0
                    || bytes[17] > MAX_CONFIGURATIONS
                {
                    return Err(refuse!("invalid high-speed device descriptor"));
                }
                self.configurations = bytes[17];
                self.bos = u16::from_le_bytes([bytes[2], bytes[3]]) >= 0x0201;
                for &index in &bytes[14..17] {
                    if index != 0 && !self.strings.contains(&index) {
                        self.strings.push(index);
                    }
                }
                self.record(DEVICE, 0, bytes);
                Enumerate::Address
            }
            Enumerate::Address => Enumerate::ConfigHeader(0),
            Enumerate::ConfigHeader(n) => {
                if bytes.len() != 9 || bytes[..2] != [9, CONFIGURATION] {
                    return Err(refuse!("invalid USB configuration prefix"));
                }
                let length = u16::from_le_bytes([bytes[2], bytes[3]]);
                if usize::from(length) > MAX_DESCRIPTOR || length < 9 {
                    return Err(refuse!("USB configuration length bound exceeded"));
                }
                Enumerate::Config(n, length)
            }
            Enumerate::Config(n, _) => {
                if let Some(endpoints) = mux_endpoints(&bytes)?
                    && self.endpoints.is_none()
                {
                    self.endpoints = Some(endpoints);
                }
                self.record(CONFIGURATION, n, bytes);
                if n + 1 < self.configurations {
                    Enumerate::ConfigHeader(n + 1)
                } else if !self.strings.is_empty() {
                    Enumerate::Languages
                } else {
                    self.after_strings()
                }
            }
            Enumerate::Languages => {
                if bytes.len() < 4
                    || !bytes.len().is_multiple_of(2)
                    || usize::from(bytes[0]) != bytes.len()
                    || bytes[1] != STRING
                {
                    return Err(refuse!("invalid USB language descriptor"));
                }
                self.language = u16::from_le_bytes([bytes[2], bytes[3]]);
                self.record(STRING, 0, bytes);
                Enumerate::Strings(0)
            }
            Enumerate::Strings(n) => {
                if bytes.len() < 2
                    || !bytes.len().is_multiple_of(2)
                    || usize::from(bytes[0]) != bytes.len()
                    || bytes[1] != STRING
                {
                    return Err(refuse!("invalid USB string descriptor"));
                }
                self.record(STRING, self.strings[n], bytes);
                if n + 1 < self.strings.len() {
                    Enumerate::Strings(n + 1)
                } else {
                    self.after_strings()
                }
            }
            Enumerate::BosHeader => {
                if bytes.len() != 5 || bytes[..2] != [5, BOS] {
                    return Err(refuse!("invalid USB BOS prefix"));
                }
                let length = u16::from_le_bytes([bytes[2], bytes[3]]);
                if length < 5 || usize::from(length) > MAX_DESCRIPTOR {
                    return Err(refuse!("USB BOS length bound exceeded"));
                }
                Enumerate::Bos(length)
            }
            Enumerate::Bos(length) => {
                if bytes.len() != usize::from(length) || bytes[..2] != [5, BOS] {
                    return Err(refuse!("invalid USB BOS total length"));
                }
                self.record(BOS, 0, bytes);
                Enumerate::Configure
            }
            Enumerate::Configure => Enumerate::Ready,
            _ => unreachable!(),
        };
        Ok(true)
    }
    fn record(&mut self, kind: u8, index: u8, bytes: Vec<u8>) {
        self.descriptors.push(Descriptor {
            kind,
            index,
            language: if kind == STRING && index != 0 {
                self.language
            } else {
                0
            },
            bytes,
        });
    }
    fn after_strings(&self) -> Enumerate {
        if self.bos {
            Enumerate::BosHeader
        } else {
            Enumerate::Configure
        }
    }
    /// One packet of the first transfer in the preferred direction, or of
    /// the other direction's first when that NAKs or none is pending: a NAK
    /// takes the bus a token's time, not a packet's. Whether one moved.
    fn bulk_step(&mut self, peer: &mut impl BusPeer) -> Result<bool> {
        for input in [self.prefer_input, !self.prefer_input] {
            let Some(position) = self
                .bulk
                .iter()
                .position(|transfer| transfer.input == input)
            else {
                continue;
            };
            let transfer = &mut self.bulk[position];
            let endpoints = self.endpoints.unwrap();
            let (endpoint, token) = if transfer.input {
                (endpoints.input, Token::In)
            } else {
                (endpoints.output, Token::Out)
            };
            let length = MAX_PACKET.min(transfer.length - transfer.done);
            let mut packet = [0; MAX_PACKET];
            if !transfer.input {
                packet[..length]
                    .copy_from_slice(&transfer.data[transfer.done..transfer.done + length]);
            }
            let reply = peer.packet(endpoint, token, &mut packet[..length])?;
            let status = match reply {
                Reply::Nak => continue,
                Reply::Stall => Some(TransferStatus::Stall),
                Reply::Ack(bytes) => {
                    if bytes > length || (!transfer.input && bytes != length) {
                        return Err(refuse!(
                            "USB peer returned an invalid bulk acknowledgement length"
                        ));
                    }
                    if transfer.input {
                        transfer.data.extend_from_slice(&packet[..bytes]);
                    }
                    transfer.done += bytes;
                    (bytes < MAX_PACKET
                        || (transfer.done == transfer.length && (transfer.input || !transfer.zlp)))
                        .then_some(TransferStatus::Complete)
                }
            };
            self.prefer_input = !transfer.input;
            if let Some(status) = status {
                let transfer = self.bulk.remove(position).unwrap();
                self.completions.push_back(Completion {
                    id: transfer.id,
                    status,
                    bytes: transfer.data,
                    transferred: transfer.done,
                });
            }
            return Ok(true);
        }
        Ok(false)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn configuration(input: u8, output: u8) -> Vec<u8> {
        vec![
            9, 2, 32, 0, 1, 7, 0, 0x80, 50, 9, 4, 3, 0, 2, 0xff, 0xfe, 2, 0, 7, 5, input, 2, 0, 2,
            0, 7, 5, output, 2, 0, 2, 0,
        ]
    }
    #[test]
    fn discovers_guest_configuration_and_refuses_malformed_descriptors() {
        let config = configuration(0x85, 9);
        assert_eq!(
            mux_endpoints(&config),
            Ok(Some(MuxEndpoints {
                configuration: 7,
                interface: 3,
                input: 0x85,
                output: 9,
                input_packet: 512,
                output_packet: 512
            }))
        );
        for cut in 0..config.len() {
            assert!(mux_endpoints(&config[..cut]).is_err());
        }
        let mut bad = config.clone();
        bad[18] = 0;
        assert!(mux_endpoints(&bad).is_err());
        let mut bad = config.clone();
        bad[27] = 0x86;
        assert!(mux_endpoints(&bad).is_err());
        let mut bad = config;
        bad[22] = 64;
        assert!(mux_endpoints(&bad).is_err());
    }
    struct Fixture {
        pending: Vec<u8>,
        requests: Vec<Setup>,
        nak: bool,
    }
    impl BusPeer for Fixture {
        fn reset(&mut self) -> Result<()> {
            self.pending.clear();
            Ok(())
        }
        fn packet(&mut self, _: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
            if self.nak {
                self.nak = false;
                return Ok(Reply::Nak);
            }
            self.nak = true;
            if token == Token::Setup {
                let setup = Setup {
                    request_type: data[0],
                    request: data[1],
                    value: u16::from_le_bytes([data[2], data[3]]),
                    index: u16::from_le_bytes([data[4], data[5]]),
                    length: u16::from_le_bytes([data[6], data[7]]),
                };
                self.pending = match (setup.request, (setup.value >> 8) as u8) {
                    (GET_DESCRIPTOR, DEVICE) => {
                        vec![18, 1, 0, 2, 0, 0, 0, 64, 0x34, 0x12, 0, 1, 0, 1, 0, 0, 0, 1]
                    }
                    (GET_DESCRIPTOR, CONFIGURATION) => configuration(0x85, 9),
                    _ => Vec::new(),
                };
                self.pending.truncate(usize::from(setup.length));
                self.requests.push(setup);
                Ok(Reply::Ack(8))
            } else if token == Token::In {
                let bytes = data.len().min(self.pending.len());
                data[..bytes].copy_from_slice(&self.pending[..bytes]);
                self.pending.drain(..bytes);
                Ok(Reply::Ack(bytes))
            } else {
                Ok(Reply::Ack(data.len()))
            }
        }
    }
    #[test]
    fn enumerates_with_naks_and_reenumerates_without_stale_transfers() {
        let mut peer = Fixture {
            pending: Vec::new(),
            requests: Vec::new(),
            nak: false,
        };
        let mut host = Host::new();
        host.attach(1);
        for n in 0..100 {
            host.advance(&mut peer, n * 24_000).unwrap();
        }
        assert!(host.ready());
        assert_eq!(host.descriptors.len(), 2);
        assert_eq!(
            peer.requests.iter().map(|s| s.request).collect::<Vec<_>>(),
            [6, 5, 6, 6, 9]
        );
        assert_eq!(peer.requests.last().unwrap().value, 7);
        let old = host.submit_in(512).unwrap();
        host.attach(2);
        assert_eq!(
            host.completion().unwrap(),
            Completion {
                id: old,
                status: TransferStatus::Cancelled,
                bytes: vec![],
                transferred: 0
            }
        );
        for n in 0..100 {
            host.advance(&mut peer, n * 24_000).unwrap();
        }
        assert!(host.ready());
        assert!(host.submit_in(512).unwrap() > old);
    }
    #[test]
    fn bounds_submissions_waits_out_naks_and_zero_length_packets() {
        let mut host = Host::new();
        host.state = Enumerate::Ready;
        host.endpoints = mux_endpoints(&configuration(0x85, 9)).unwrap();
        let mut peer = Fixture {
            pending: vec![],
            requests: vec![],
            nak: false,
        };
        let id = host.submit_out(&vec![0; 512], true).unwrap();
        for _ in 0..4 {
            host.advance(&mut peer, 0).unwrap();
        }
        let done = host.completion().unwrap();
        assert_eq!(done.id, id);
        assert_eq!(done.transferred, 512);
        assert!(host.submit_in(MAX_TRANSFER + 1).is_err());
        for _ in 0..MAX_TRANSFERS {
            host.submit_in(512).unwrap();
        }
        assert!(host.submit_in(512).is_err());
        // A device that NAKs keeps its transfers pending, however long.
        peer.nak = true;
        host.advance(&mut peer, 100 * REQUEST_TIMEOUT).unwrap();
        assert!(host.completion().is_none());
    }
    #[test]
    fn a_naking_input_costs_the_output_no_packet() {
        struct OutOnly(usize);
        impl BusPeer for OutOnly {
            fn reset(&mut self) -> Result<()> {
                Ok(())
            }
            fn packet(&mut self, _: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
                if token == Token::In {
                    return Ok(Reply::Nak);
                }
                self.0 += 1;
                Ok(Reply::Ack(data.len()))
            }
        }
        let mut host = Host::new();
        host.state = Enumerate::Ready;
        host.endpoints = mux_endpoints(&configuration(0x85, 9)).unwrap();
        let mut peer = OutOnly(0);
        host.submit_in(512).unwrap();
        let id = host.submit_out(&[0; 1024], false).unwrap();
        // The IN, preferred in turn, NAKs; each call still moves an OUT packet.
        assert!(host.advance(&mut peer, 0).unwrap());
        assert!(host.advance(&mut peer, 0).unwrap());
        assert_eq!(peer.0, 2);
        let done = host.completion().unwrap();
        assert_eq!((done.id, done.transferred), (id, 1024));
        // With only the NAKing IN pending, nothing moves.
        assert!(!host.advance(&mut peer, 0).unwrap());
        assert_eq!(peer.0, 2);
    }

    #[test]
    fn a_failed_bulk_packet_preserves_progress_and_queue_order() {
        struct Peer {
            fail: bool,
            seen: Vec<u8>,
        }
        impl BusPeer for Peer {
            fn reset(&mut self) -> Result<()> {
                Ok(())
            }
            fn packet(&mut self, _: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
                assert_eq!(token, Token::Out);
                if self.fail {
                    return Err(refuse!("test USB peer failure"));
                }
                self.seen.push(data[0]);
                Ok(Reply::Ack(data.len()))
            }
        }
        let mut host = Host::new();
        host.state = Enumerate::Ready;
        host.endpoints = mux_endpoints(&configuration(0x85, 9)).unwrap();
        let mut bytes = [1; 1024];
        bytes[512..].fill(2);
        let first = host.submit_out(&bytes, false).unwrap();
        let second = host.submit_out(&[3; 17], false).unwrap();
        let mut peer = Peer {
            fail: false,
            seen: Vec::new(),
        };
        assert!(host.advance(&mut peer, 0).unwrap());
        peer.fail = true;
        assert!(host.advance(&mut peer, 0).is_err());
        assert!(host.completion().is_none());
        peer.fail = false;
        assert!(host.advance(&mut peer, 0).unwrap());
        assert_eq!(host.completion().unwrap().id, first);
        assert!(host.advance(&mut peer, 0).unwrap());
        assert_eq!(host.completion().unwrap().id, second);
        assert_eq!(peer.seen, [1, 2, 3]);
    }
}
