// SPDX-License-Identifier: GPL-2.0-or-later AND GPL-2.0-only
// NCM mode negotiation adapted from Linux cdc_ncm.c:
// Copyright (C) ST-Ericsson 2010-2012
// Original author: Hans Petter Selasky <hans.petter.selasky@stericsson.com>
//! USB CDC Ethernet adapters as host network links: the Ethernet Control
//! Model (CDC ECM 1.2) and the Network Control Model (CDC NCM 1.0), the two
//! standard USB Ethernet classes.
//!
//! The communications interface (class 2, subclass 6 for ECM or 0x0d for
//! NCM) names its data interface in its Union descriptor and its station
//! address in the Ethernet Networking descriptor's iMACAddress string.
//! Attach configures the data interface's alternate setting with two bulk
//! endpoints, selects it (SET_INTERFACE), and sets the packet filter to
//! directed, broadcast and all-multicast frames (ECM 6.2.4; NCM only when
//! its functional descriptor's bmNetworkCapabilities offers the request).
//!
//! ECM moves one frame per bulk transfer: each received frame is one bulk
//! IN transfer ended by a short packet (3.3.1); each sent frame is one bulk
//! OUT transfer. NCM moves transfer blocks (`ntb`): while the data
//! interface is still at alternate setting 0, attach reads the function's
//! NTB parameters (GetNtbParameters), selects CRC-off and NTB16 when those
//! controls are supported, and caps its IN blocks at
//! [`NCM_IN_BLOCK`] bytes (SetNtbInputSize); each IN block may carry several
//! frames, and each sent frame is one OUT block. Either way a transfer that
//! is a multiple of the endpoint's packet size is followed by a zero-length
//! packet, except an NCM block of exactly the function's largest OUT size.
//! NetworkConnection notifications on the interrupt endpoint set the
//! carrier (ECM 6.3.1, and alike in NCM); until one arrives the link is
//! taken to be up. Frames above the 1514-byte Ethernet maximum are refused.
//!
//! As usbnet keeps its URBs, one OUT transfer and one IN transfer stay in
//! flight across link calls, and no call waits for the device: a frame
//! sent while the last is in flight is refused as busy, and a failed
//! transfer's endpoint reset runs a command or poll per call.
mod ntb;

use super::descriptor::{
    DEVICE_TO_HOST, Endpoint, Interface, RECIPIENT_INTERFACE, SET_INTERFACE, TYPE_CLASS, setup,
};
use super::xhci::{Device, Operation, Plan, Shared, SlotId, TransferTicket};
use crate::{
    Deadline,
    block::BlockError,
    bus::dma::{DmaAllocator, DmaBuffer},
    net::{Link, LinkError, MAX_FRAME, Mac},
};
use alloc::{collections::VecDeque, format, string::String, vec::Vec};

pub const COMMUNICATIONS: u8 = 2;
/// The Ethernet Control Model subclass.
pub const ECM: u8 = 6;
/// The Network Control Model subclass.
pub const NCM: u8 = 0x0d;
/// Class requests and notifications address the communications interface.
const CLASS_INTERFACE: u8 = TYPE_CLASS | RECIPIENT_INTERFACE;
pub(crate) const DATA_CLASS: u8 = 10;
const CS_INTERFACE: u8 = 0x24;
const UNION: u8 = 0x06;
const ETHERNET: u8 = 0x0f;
/// The NCM functional descriptor's subtype.
const NCM_FUNCTIONAL: u8 = 0x1a;
/// bmNetworkCapabilities: SetEthernetPacketFilter is supported.
const NCM_PACKET_FILTER: u8 = 1 << 0;
/// bmNetworkCapabilities: CRC mode can be selected.
const NCM_CRC_MODE: u8 = 1 << 4;
/// bmNetworkCapabilities: SetNtbInputSize takes 8 bytes, the size and
/// wNtbInMaxDatagrams.
const NCM_INPUT_SIZE_8: u8 = 1 << 5;
const SET_ETHERNET_PACKET_FILTER: u8 = 0x43;
const GET_NTB_PARAMETERS: u8 = 0x80;
const SET_NTB_FORMAT: u8 = 0x84;
const SET_NTB_INPUT_SIZE: u8 = 0x86;
const SET_CRC_MODE: u8 = 0x8a;
const NTB16_FORMAT: u16 = 0;
const CRC_NOT_APPENDED: u16 = 0;
/// Directed, broadcast and all-multicast frames.
const FILTER: u16 = 0x04 | 0x08 | 0x02;
const NETWORK_CONNECTION: u8 = 0x00;
/// An ECM frame's DMA buffer.
const FRAME_BYTES: usize = 2048;
/// The largest NCM IN block this driver takes: room for ten full frames.
pub const NCM_IN_BLOCK: usize = 16 * 1024;
/// How long one OUT transfer may stay outstanding before it is cancelled.
const SEND_US: u64 = 1_000_000;
/// xHCI's Incompatible Device Error, which only retiring the slot ends.
const INCOMPATIBLE_DEVICE: u32 = 22;

/// The CDC Ethernet model an adapter speaks.
#[derive(Clone, Copy, Debug)]
enum Model {
    Ecm,
    /// With its functional descriptor's bmNetworkCapabilities.
    Ncm(u8),
}

/// How frames travel on the data interface.
enum Framing {
    Ecm,
    /// NTB16 blocks, with the next OUT block's wSequence.
    Ncm(ntb::Parameters, u16),
}

/// The interfaces and endpoints one adapter uses.
#[derive(Clone, Debug)]
pub struct Binding {
    model: Model,
    control: u8,
    data: u8,
    alternate: u8,
    mac_string: u8,
    input: Endpoint,
    output: Endpoint,
    notify: Option<Endpoint>,
}

impl Binding {
    /// The data interface and alternate setting selected by this binding.
    pub(super) fn data_setting(&self) -> (u8, u8) {
        (self.data, self.alternate)
    }

    pub(super) fn endpoints(&self) -> Vec<Endpoint> {
        let mut endpoints = alloc::vec![self.input, self.output];
        endpoints.extend(self.notify);
        endpoints
    }
}

pub(super) fn plan(interface: &Interface, all: &[Interface]) -> Result<Plan, &'static str> {
    let descriptor = |kind: u8, bytes: usize| {
        interface
            .extra
            .iter()
            .find(|d| d.len() >= bytes && d[1] == CS_INTERFACE && d[2] == kind)
    };
    let model = match interface.subclass {
        ECM => Model::Ecm,
        NCM => Model::Ncm(
            descriptor(NCM_FUNCTIONAL, 6)
                .ok_or("NCM interface without its functional descriptor")?[5],
        ),
        _ => return Err("not a CDC Ethernet interface"),
    };
    let union = descriptor(UNION, 5).ok_or("CDC Ethernet interface without a Union descriptor")?;
    let ethernet = descriptor(ETHERNET, 13)
        .ok_or("CDC Ethernet interface without an Ethernet Networking descriptor")?;
    let data = union[4];
    let setting = all
        .iter()
        .find(|i| i.number == data && i.class == DATA_CLASS && i.endpoints.len() >= 2)
        .ok_or("CDC Ethernet data interface without a bulk setting")?;
    let bulk = |is_in: bool| {
        setting
            .endpoints
            .iter()
            .copied()
            .find(|e| e.kind() == 2 && e.is_in() == is_in)
    };
    let (Some(input), Some(output)) = (bulk(true), bulk(false)) else {
        return Err("CDC Ethernet data interface without two bulk endpoints");
    };
    let notify = interface
        .endpoints
        .iter()
        .copied()
        .find(|e| e.kind() == 3 && e.is_in());
    Ok(Plan::Ethernet(Binding {
        model,
        control: interface.number,
        data,
        alternate: setting.alternate,
        mac_string: ethernet[3],
        input,
        output,
        notify,
    }))
}

fn parse_mac(text: &str) -> Option<Mac> {
    if text.len() != 12 {
        return None;
    }
    let mut mac = [0u8; 6];
    for (n, byte) in mac.iter_mut().enumerate() {
        *byte = u8::from_str_radix(text.get(2 * n..2 * n + 2)?, 16).ok()?;
    }
    Some(Mac(mac))
}

/// The OUT transfer the controller still owns: its ticket, its length, when
/// it times out, and whether a zero-length packet follows it.
struct Sending {
    ticket: TransferTicket,
    bytes: usize,
    deadline: Deadline,
    zero_length: bool,
}

/// What an endpoint reset under way is for.
enum Reset {
    /// A failed IN transfer, dropped once its endpoint is reset.
    Input,
    /// A failed or timed-out OUT transfer, reported once it is cancelled.
    Output(BlockError),
}

pub struct UsbNet {
    controller: Shared,
    id: SlotId,
    binding: Binding,
    framing: Framing,
    mac: Mac,
    receive: DmaBuffer,
    /// The received frames of an NCM block not yet taken.
    pending: VecDeque<Vec<u8>>,
    send: DmaBuffer,
    status: DmaBuffer,
    queued: Option<TransferTicket>,
    /// The send buffer is the controller's until this ends or its
    /// cancellation is acknowledged.
    sending: Option<Sending>,
    /// An endpoint reset, a command or poll per link call.
    resetting: Option<(Operation, Reset)>,
    /// A failed OUT transfer's error, once its endpoint is reset, for the
    /// next transmit to report.
    sent_error: Option<BlockError>,
    notify_queued: Option<TransferTicket>,
    connected: Option<bool>,
    /// Receive transfers that failed, blocks that did not decode and frames
    /// no Ethernet link carries, dropped as usbnet counts them in
    /// `rx_errors`.
    rx_dropped: u64,
    pub location: String,
}

impl UsbNet {
    /// Receive transfers, blocks and frames dropped as damaged. Saturates.
    pub fn rx_dropped(&self) -> u64 {
        self.rx_dropped
    }
    pub(super) fn new(
        controller: Shared,
        device: &Device,
        binding: Binding,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        let (mac, framing) = {
            let mut host = controller.borrow_mut();
            let mut c = host.device(device.id)?;
            let text = device
                .language
                .and_then(|language| c.string(language, binding.mac_string))
                .ok_or(BlockError::Unsupported(
                    "CDC Ethernet adapter without a MAC address string",
                ))?;
            let mac = parse_mac(&text)
                .ok_or(BlockError::Unsupported("CDC Ethernet MAC address string"))?;
            let control = u16::from(binding.control);
            // NCM's mode and size requests precede the data setting.
            let framing = match binding.model {
                Model::Ecm => Framing::Ecm,
                Model::Ncm(capabilities) => {
                    let request = setup(
                        DEVICE_TO_HOST | CLASS_INTERFACE,
                        GET_NTB_PARAMETERS,
                        0,
                        control,
                        ntb::PARAMETER_BYTES as u16,
                    );
                    let reply = c.control(request, &[])?;
                    let mut parameters =
                        ntb::Parameters::parse(&reply).map_err(BlockError::Unsupported)?;
                    if capabilities & NCM_CRC_MODE != 0 {
                        let request =
                            setup(CLASS_INTERFACE, SET_CRC_MODE, CRC_NOT_APPENDED, control, 0);
                        c.control(request, &[])?;
                    }
                    // NTB16 is the reset default. Like Linux cdc_ncm_init,
                    // explicitly select it when both formats are offered.
                    if parameters.supports_ntb32 {
                        let request =
                            setup(CLASS_INTERFACE, SET_NTB_FORMAT, NTB16_FORMAT, control, 0);
                        c.control(request, &[])?;
                    }
                    if parameters.in_max > NCM_IN_BLOCK {
                        parameters.in_max = NCM_IN_BLOCK;
                        let mut size = (NCM_IN_BLOCK as u32).to_le_bytes().to_vec();
                        if capabilities & NCM_INPUT_SIZE_8 != 0 {
                            // wNtbInMaxDatagrams 0: no limit.
                            size.extend_from_slice(&[0; 4]);
                        }
                        let request = setup(
                            CLASS_INTERFACE,
                            SET_NTB_INPUT_SIZE,
                            0,
                            control,
                            size.len() as u16,
                        );
                        c.control(request, &size)?;
                    }
                    Framing::Ncm(parameters, 0)
                }
            };
            (mac, framing)
        };
        let (receive_bytes, send_bytes) = match &framing {
            Framing::Ecm => (FRAME_BYTES, FRAME_BYTES),
            Framing::Ncm(parameters, _) => (
                parameters.in_max,
                parameters
                    .encoded_len(MAX_FRAME)
                    .ok_or(BlockError::Unsupported(
                        "NCM OUT parameters cannot fit the maximum Ethernet frame",
                    ))?,
            ),
        };
        // Both one-TD buffers must stay inside one 64 KiB xHCI window. The
        // validated NTB16 OUT size is below 64 KiB, and IN is capped at 16 KiB;
        // next-power-of-two alignment therefore contains every transfer.
        let receive_align = receive_bytes
            .checked_next_power_of_two()
            .ok_or(BlockError::NoDmaMemory)?;
        let send_align = send_bytes
            .checked_next_power_of_two()
            .ok_or(BlockError::NoDmaMemory)?;
        let receive = dma.allocate(receive_bytes, receive_align)?;
        let send = dma.allocate(send_bytes, send_align)?;
        let status = dma.allocate(64, 64)?;
        // Do not activate the data setting if any bounded DMA allocation failed.
        {
            let mut host = controller.borrow_mut();
            let mut c = host.device(device.id)?;
            let control = u16::from(binding.control);
            if binding.alternate != 0 {
                let request = setup(
                    1,
                    SET_INTERFACE,
                    u16::from(binding.alternate),
                    u16::from(binding.data),
                    0,
                );
                c.control(request, &[])?;
            }
            let filter = match binding.model {
                Model::Ecm => true,
                Model::Ncm(capabilities) => capabilities & NCM_PACKET_FILTER != 0,
            };
            if filter {
                let request = setup(
                    CLASS_INTERFACE,
                    SET_ETHERNET_PACKET_FILTER,
                    FILTER,
                    control,
                    0,
                );
                c.control(request, &[])?;
            }
        }
        Ok(Self {
            controller,
            id: device.id,
            binding,
            framing,
            mac,
            receive,
            pending: VecDeque::new(),
            send,
            status,
            queued: None,
            sending: None,
            resetting: None,
            sent_error: None,
            notify_queued: None,
            connected: None,
            rx_dropped: 0,
            location: format!("{} ({})", device.location, device.name()),
        })
    }

    /// Takes any NetworkConnection notifications that arrived.
    fn notifications(&mut self) -> Result<(), BlockError> {
        let mut host = self.controller.borrow_mut();
        let mut c = host.device(self.id)?;
        let Some(notify) = self.binding.notify else {
            return Ok(());
        };
        let length = usize::from(notify.max_packet & 0x7ff).min(64);
        for _ in 0..4 {
            if self.notify_queued.is_none() {
                self.notify_queued =
                    Some(c.submit_ticket(notify.dci(), self.status.physical(), length)?);
            }
            let Some(result) = c.completed_ticket(self.notify_queued.as_ref().unwrap()) else {
                return Ok(());
            };
            self.notify_queued = None;
            if let Ok(bytes) = result
                && bytes >= 8
            {
                let header = &self.status.bytes()[..8];
                if header[0] == DEVICE_TO_HOST | CLASS_INTERFACE && header[1] == NETWORK_CONNECTION
                {
                    self.connected = Some(header[2] != 0);
                }
            }
        }
        Ok(())
    }
}

fn link_error(error: BlockError) -> LinkError {
    LinkError::Adapter(format!("USB Ethernet: {error}"))
}

impl UsbNet {
    /// One look at the endpoint reset under way: whether none runs any more.
    /// A reset that fails leaves its endpoint refusing every transfer.
    fn advance(&mut self) -> Result<bool, BlockError> {
        let Some((operation, _)) = &mut self.resetting else {
            return Ok(true);
        };
        let result = {
            let mut host = self.controller.borrow_mut();
            host.device(self.id)?.poll(operation)
        };
        let Some(result) = result else {
            return Ok(false);
        };
        let (_, reset) = self.resetting.take().unwrap();
        result?;
        match reset {
            Reset::Input => self.rx_dropped = self.rx_dropped.saturating_add(1),
            Reset::Output(error) => {
                self.sending = None;
                self.sent_error = Some(error);
            }
        }
        Ok(true)
    }

    /// Whether the send buffer is free: no OUT transfer in flight, or the
    /// one in flight completed, after which a due zero-length packet is
    /// sent. A failed transfer is reported once its endpoint's TDs are
    /// cancelled, and one past its deadline once its cancellation is
    /// acknowledged; until then the buffer stays the controller's.
    fn sent(&mut self) -> Result<bool, LinkError> {
        if let Some(error) = self.sent_error.take() {
            return Err(link_error(error));
        }
        let output = self.binding.output;
        loop {
            let Some(sending) = &self.sending else {
                return Ok(true);
            };
            let mut host = self.controller.borrow_mut();
            let clock = host.clock;
            let mut c = host.device(self.id).map_err(link_error)?;
            let error = match c.completed_ticket(&sending.ticket) {
                Some(Ok(actual)) if actual != sending.bytes => {
                    self.sending = None;
                    return Err(LinkError::Adapter(String::from(
                        "short USB Ethernet OUT transfer",
                    )));
                }
                Some(Ok(_)) if sending.zero_length => {
                    let deadline = clock.try_deadline(SEND_US).map_err(|_| {
                        LinkError::Adapter(String::from("USB Ethernet send clock unavailable"))
                    })?;
                    let ticket = c
                        .submit_ticket(output.dci(), self.send.physical(), 0)
                        .map_err(link_error)?;
                    self.sending = Some(Sending {
                        ticket,
                        bytes: 0,
                        deadline,
                        zero_length: false,
                    });
                    continue;
                }
                Some(Ok(_)) => {
                    self.sending = None;
                    return Ok(true);
                }
                // Only retiring the slot ends this; no cancellation can.
                Some(Err(
                    error @ BlockError::Device {
                        status: INCOMPATIBLE_DEVICE,
                    },
                )) => {
                    self.sending = None;
                    return Err(link_error(error));
                }
                Some(Err(error)) => error,
                None if sending.deadline.passed() => BlockError::Timeout,
                None => return Ok(false),
            };
            let operation = c.start_cancel(output.dci()).map_err(link_error)?;
            drop(host);
            self.resetting = Some((operation, Reset::Output(error)));
            self.advance().map_err(link_error)?;
            return match self.sent_error.take() {
                Some(error) => Err(link_error(error)),
                None => Ok(false),
            };
        }
    }
}

impl Link for UsbNet {
    fn mac(&self) -> Mac {
        self.mac
    }

    fn carrier(&mut self) -> bool {
        self.advance().is_ok() && self.notifications().is_ok() && self.connected.unwrap_or(true)
    }

    /// Queue `frame` and return without waiting for the controller, as
    /// usbnet_start_xmit submits its URB. While the previous frame, or an
    /// endpoint reset, is still under way the frame is refused as busy.
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        let idle = self.advance().map_err(link_error)?;
        if frame.len() > MAX_FRAME {
            return Err(LinkError::Adapter(String::from(
                "frame above the Ethernet maximum",
            )));
        }
        if !idle || !self.sent()? {
            return Err(LinkError::Busy);
        }
        let mut host = self.controller.borrow_mut();
        let clock = host.clock;
        let mut c = host.device(self.id).map_err(link_error)?;
        c.endpoint_ready(self.binding.output.dci())
            .map_err(link_error)?;
        let deadline = clock
            .try_deadline(SEND_US)
            .map_err(|_| LinkError::Adapter(String::from("USB Ethernet send clock unavailable")))?;
        // The length of the one OUT transfer, and the NCM block size that
        // needs no zero-length packet after it.
        let (length, whole) = match &mut self.framing {
            Framing::Ecm => {
                self.send.bytes_mut()[..frame.len()].copy_from_slice(frame);
                (frame.len(), None)
            }
            Framing::Ncm(parameters, sequence) => {
                let block = parameters
                    .encode(*sequence, frame)
                    .filter(|block| block.len() <= self.send.len())
                    .ok_or_else(|| {
                        LinkError::Adapter(String::from("frame above the NCM OUT block size"))
                    })?;
                *sequence = sequence.wrapping_add(1);
                self.send.bytes_mut()[..block.len()].copy_from_slice(&block);
                (block.len(), Some(parameters.out_max))
            }
        };
        let output = self.binding.output;
        let packet = usize::from(output.max_packet & 0x7ff).max(1);
        let ticket = c
            .submit_ticket(output.dci(), self.send.physical(), length)
            .map_err(link_error)?;
        drop(host);
        self.sending = Some(Sending {
            ticket,
            bytes: length,
            deadline,
            zero_length: length.is_multiple_of(packet) && whole != Some(length),
        });
        // A controller that already finished reports the outcome now.
        self.sent().map(|_| ())
    }

    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        if !self.advance().map_err(link_error)? {
            return Ok(None);
        }
        let mut host = self.controller.borrow_mut();
        let mut c = host.device(self.id).map_err(link_error)?;
        if let Some(frame) = self.pending.pop_front() {
            return Ok(Some(frame));
        }
        let input = self.binding.input;
        let capacity = self.receive.bytes().len();
        if self.queued.is_none() {
            self.queued = Some(
                c.submit_ticket(input.dci(), self.receive.physical(), capacity)
                    .map_err(link_error)?,
            );
        }
        let Some(result) = c.completed_ticket(self.queued.as_ref().unwrap()) else {
            return Ok(None);
        };
        self.queued = None;
        let bytes = match result {
            Ok(bytes) => &self.receive.bytes()[..bytes],
            Err(_) => {
                // usbnet's rx_complete: a failed transfer is counted and
                // dropped once its endpoint is reset; an endpoint that
                // cannot be reset is a dead device.
                let operation = c.start_clear(&input).map_err(link_error)?;
                drop(host);
                self.resetting = Some((operation, Reset::Input));
                self.advance().map_err(link_error)?;
                return Ok(None);
            }
        };
        // A malformed block, like an oversized frame, is dropped as a NIC
        // drops a frame that fails its checks.
        let frames = match self.framing {
            Framing::Ecm => Some(alloc::vec![bytes]),
            Framing::Ncm(..) => ntb::decode(bytes).ok(),
        };
        let Some(frames) = frames else {
            self.rx_dropped = self.rx_dropped.saturating_add(1);
            return Ok(None);
        };
        for frame in frames {
            // An empty transfer is the zero-length packet that ends a frame
            // of whole packets, not a damaged frame.
            if frame.len() > MAX_FRAME {
                self.rx_dropped = self.rx_dropped.saturating_add(1);
            } else if !frame.is_empty() {
                self.pending.push_back(frame.to_vec());
            }
        }
        Ok(self.pending.pop_front())
    }
}

#[cfg(test)]
pub(crate) mod tests;
