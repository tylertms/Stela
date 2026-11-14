// SPDX-License-Identifier: GPL-2.0-or-later
// Linux HID reference: Copyright (c) 1999 Andreas Gal;
// 2000-2005 Vojtech Pavlik; 2005 Michael Haboustak for Concept2, Inc;
// 2006-2012 Jiri Kosina; 2007-2008 Oliver Neukum.
// Source identities and adaptation scope: NOTICE.
//! Native USB HID report transport and descriptor-checked input.
//!
//! The original report descriptor and every received report keep IDs,
//! collections, usages and values through `poll_hid`, including consumer keys,
//! gamepads, extra buttons and multitouch. No contact/axis semantics or receiver
//! identity is synthesized. Delimiter sets select their primary usage branch;
//! vendor protocols require explicit additional support. Existing `poll` exposes
//! the legacy keyboard/pointer input vocabulary and standard boot fallback for
//! boot-subclass interfaces.
//! EP0 controls use descriptor lengths and status-stage acknowledgements;
//! endpoint stop/restart shares the controller's DMA retirement permission.
mod control;
pub mod descriptor;
mod error;
pub mod events;
mod lifecycle;
pub mod receiver;
mod report;
mod touch;
pub mod translate;
mod usages;

pub use descriptor::{DecodeError, Descriptor, Frame, ReportType};
pub use error::{ParseError, ParseErrorKind, Reader, UsageError};
pub use touch::{TouchDiscard, TouchFrame, TouchPoll};
pub use translate::{Translated, Translator};

use super::descriptor::{
    DEVICE_TO_HOST, Endpoint, GET_DESCRIPTOR, Interface, RECIPIENT_INTERFACE, TYPE_CLASS, setup,
};
use super::xhci::{Controller, Device, Plan, STALL, Shared, SlotId};
use crate::{
    block::BlockError,
    bus::dma::{DmaAllocator, DmaBuffer},
    input::Input,
};
use alloc::{format, string::String, vec::Vec};
use report::{Keys, Pointer, Reports};

pub const CLASS: u8 = 3;
const BOOT: u8 = 1;
const KEYBOARD: u8 = 1;
const MOUSE: u8 = 2;
const GET_REPORT: u8 = 0x01;
const SET_REPORT: u8 = 0x09;
const SET_IDLE: u8 = 0x0a;
const MAX_DESCRIPTOR_BYTES: usize = 4096;
const SET_PROTOCOL: u8 = 0x0b;
const HID_DESCRIPTOR: u8 = 0x21;
const REPORT_DESCRIPTOR: u8 = 0x22;
/// Boot reports fit eight bytes (HID 1.11 Appendix B); Linux `usbmouse`
/// caps its transfer there.
const BOOT_REPORT_BYTES: usize = 8;
/// An interrupt endpoint's wMaxPacketSize payload bits.
const MAX_PACKET_SIZE: u16 = 0x7ff;
/// Linux `hid_io_error`: retry after 13 ms, doubling to at least 100 ms,
/// and give up after a second of failures.
const FIRST_RETRY_MS: u64 = 13;
const LONGEST_RETRY_MS: u64 = 100;
const RETRY_WINDOW_US: u64 = 1_000_000;
const MICROSECONDS_PER_MILLISECOND: u64 = 1_000;
pub(super) const LED_PAGE: u32 = 0x08;
/// Main item bit 1: a Variable field, one value per usage.
pub(super) const VARIABLE: u32 = 2;
/// LED-page usages by input LED code, as `hidinput_configure_usage` maps
/// them: Num Lock, Caps Lock, Scroll Lock, Compose, Kana, Stand-By, System
/// Suspend, Mute, Generic Indicator, Message Waiting, External Power.
pub(super) const LED_USAGES: [u16; 11] = [
    0x01, 0x02, 0x03, 0x04, 0x05, 0x27, 0x4c, 0x09, 0x4b, 0x19, 0x4d,
];
/// include/linux/usb.h USB_CTRL_SET_TIMEOUT, the `usb_interrupt_msg` bound
/// `usbhid_output_report` uses.
const OUTPUT_TIMEOUT_US: u64 = 5_000_000;
const BOOT_PROTOCOL: u16 = 0;
const REPORT_PROTOCOL: u16 = 1;
/// The Keyboard/Keypad usages the native keyboard reports.
pub(super) const KEY_USAGES: core::ops::RangeInclusive<u32> = 0x04..=0xdd;
pub(super) const MODIFIER_USAGES: core::ops::RangeInclusive<u32> = 0xe0..=0xe7;

pub(super) fn plan(interface: &Interface) -> Result<Plan, &'static str> {
    let endpoint =
        interrupt(interface, true).ok_or("HID interface without an interrupt IN endpoint")?;
    Ok(Plan::Hid(endpoint, interrupt(interface, false)))
}

/// `usbhid_start`: the first interrupt endpoint in each direction.
fn interrupt(interface: &Interface, input: bool) -> Option<Endpoint> {
    interface
        .endpoints
        .iter()
        .copied()
        .find(|e| e.kind() == 3 && e.is_in() == input)
}

/// Fetch the exact descriptor named by this interface's HID descriptor.
async fn read_descriptor(
    interface: &Interface,
    control: &mut impl AsyncFnMut([u8; 8]) -> Result<Vec<u8>, BlockError>,
) -> Result<Vec<u8>, BlockError> {
    let length = descriptor_length(interface).map_err(BlockError::Unsupported)?;
    let request = setup(
        DEVICE_TO_HOST | RECIPIENT_INTERFACE,
        GET_DESCRIPTOR,
        u16::from(REPORT_DESCRIPTOR) << 8,
        u16::from(interface.number),
        length,
    );
    let bytes = control(request).await?;
    if bytes.len() != usize::from(length) {
        return Err(BlockError::Protocol("short USB HID report descriptor"));
    }
    Ok(bytes)
}

fn descriptor_length(interface: &Interface) -> Result<u16, &'static str> {
    let descriptor = interface
        .extra
        .iter()
        .find(|d| d.get(1) == Some(&HID_DESCRIPTOR))
        .ok_or("HID interface without a HID descriptor")?;
    if descriptor.len() < 6
        || usize::from(descriptor[0]) != descriptor.len()
        || descriptor.len() != 6 + usize::from(descriptor[5]) * 3
    {
        return Err("malformed USB HID descriptor");
    }
    let mut reports = descriptor[6..]
        .as_chunks::<3>()
        .0
        .iter()
        .filter(|entry| entry[0] == REPORT_DESCRIPTOR);
    let entry = reports
        .next()
        .ok_or("HID interface without a report descriptor")?;
    if reports.next().is_some() {
        return Err("multiple HID report descriptors on one interface");
    }
    let length = u16::from_le_bytes([entry[1], entry[2]]);
    if length == 0 || usize::from(length) > MAX_DESCRIPTOR_BYTES {
        return Err("USB HID report descriptor length outside 1..4096");
    }
    Ok(length)
}

/// Why an interface is left unused.
pub const NO_REPORTS: &str = "HID interface without an input report";

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum State {
    Running,
    Stopped,
    /// A transport/cancellation failed. Keep the TD and DMA until the shared
    /// controller can acknowledge retirement.
    Faulted,
}

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum ReceivedReport {
    Data(Frame),
    /// A standard boot-protocol report; the original descriptor could not be
    /// used. `Poll::inputs` carries the Appendix B keyboard/mouse events.
    Boot(Vec<u8>),
    Rejected {
        bytes: Vec<u8>,
        error: DecodeError,
    },
}

/// One bounded service pass: native compatibility inputs and every received
/// descriptor-checked HID report. Frontends must consume `reports` to expose
/// consumer controls, extra buttons, gamepads and touch to the guest.
#[derive(Debug, Default)]
pub struct Poll {
    pub inputs: Vec<Input>,
    pub events: Vec<events::Event>,
    pub reports: Vec<ReceivedReport>,
}

/// The protocol and layout to read a boot interface with: its report
/// descriptor's, in the report protocol, when that is `usable` (describes
/// the keyboard or pointer the interface is), else `boot` in the boot
/// protocol.
fn boot_interface(
    described: Result<Reports, BlockError>,
    boot: Reports,
    usable: impl Fn(&Reports) -> bool,
) -> (u16, Reports) {
    match described {
        Ok(reports) if usable(&reports) => (REPORT_PROTOCOL, reports),
        _ => (BOOT_PROTOCOL, boot),
    }
}

/// How [`UsbInput::negotiate`] chose to read an interface.
pub(in crate::usb) struct Negotiated {
    chosen: u16,
    reports: Reports,
    descriptor: Option<Descriptor>,
    report_descriptor: Vec<u8>,
}

/// Failed transfers being retried, as Linux `hid_io_error` retries them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
struct Retry {
    delay_ms: u64,
    /// When the next transfer may go out, and when retrying stops.
    at_us: u64,
    stop_us: u64,
}

impl Retry {
    fn first(now_us: u64) -> Self {
        Self {
            delay_ms: FIRST_RETRY_MS,
            at_us: now_us + FIRST_RETRY_MS * MICROSECONDS_PER_MILLISECOND,
            stop_us: now_us + RETRY_WINDOW_US,
        }
    }

    /// The next retry after another failure, or none once retrying has run
    /// a second without a report.
    fn next(self, now_us: u64) -> Option<Self> {
        if now_us > self.stop_us {
            return None;
        }
        let delay_ms = match self.delay_ms < LONGEST_RETRY_MS {
            true => self.delay_ms * 2,
            false => self.delay_ms,
        };
        Some(Self {
            delay_ms,
            at_us: now_us + delay_ms * MICROSECONDS_PER_MILLISECOND,
            ..self
        })
    }
}

/// One HID interface the host polls.
pub struct UsbInput {
    controller: Shared,
    id: SlotId,
    interface: u8,
    report_descriptor: Vec<u8>,
    state: State,
    /// The original first transport failure, never replaced by a later error.
    error: Option<BlockError>,
    endpoint: Endpoint,
    buffer: DmaBuffer,
    /// Bytes one interrupt transfer asks for: the longest input report.
    length: usize,
    /// The interrupt OUT endpoint and a buffer for the longest output
    /// report, when the interface has both.
    output: Option<(Endpoint, DmaBuffer)>,
    /// The queued TD, if one is out.
    queued: Option<u64>,
    /// Transfer errors being retried.
    retry: Option<Retry>,
    translator: Translator,
    /// Completed output stays with this owner across later controller calls.
    pending: TouchPoll,
    touch: Option<touch::Assembler>,
    receiver: Option<receiver::Link>,
    /// The first transfer failure, reported once.
    failed: bool,
    pub name: String,
}

impl UsbInput {
    pub(super) fn new(
        controller: Shared,
        device: &Device,
        interface: &Interface,
        endpoint: Endpoint,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Option<Self>, BlockError> {
        let negotiated = {
            let mut c = controller.borrow_mut();
            stela_driver_base::stepped::block_on(Self::negotiate(interface, async |request| {
                c.device(device.id)?.control(request, &[])
            }))?
        };
        Self::start(controller, device, interface, endpoint, negotiated, dma)
    }

    /// The class requests that choose how an interface is read, each a
    /// control transfer `control` makes: its report descriptor, then
    /// SET_PROTOCOL for a boot interface and SET_IDLE.
    pub(in crate::usb) async fn negotiate(
        interface: &Interface,
        mut control: impl AsyncFnMut([u8; 8]) -> Result<Vec<u8>, BlockError>,
    ) -> Result<Negotiated, BlockError> {
        let index = u16::from(interface.number);
        // SET_IDLE reports on change only; a device may refuse.
        let idle = setup(TYPE_CLASS | RECIPIENT_INTERFACE, SET_IDLE, 0, index, 0);
        // The generic layout is authoritative for all report kinds. Boot
        // fallback is allowed only for a boot subclass interface.
        let raw = read_descriptor(interface, &mut control).await;
        let described = raw.as_ref().map_err(Clone::clone).and_then(|bytes| {
            let descriptor = Descriptor::parse(bytes)
                .map_err(|error| BlockError::Unsupported(error.message()))?;
            let reports =
                report::parse(bytes).map_err(|error| BlockError::Unsupported(error.message()))?;
            Ok((descriptor, reports))
        });
        let legacy = described
            .as_ref()
            .map(|(_, reports)| reports.clone())
            .map_err(Clone::clone);
        let boot = match (interface.subclass, interface.protocol) {
            (BOOT, KEYBOARD) => Some(boot_interface(
                legacy.clone(),
                Reports {
                    keys: Some(Keys::boot()),
                    pointer: None,
                    input_bytes: BOOT_REPORT_BYTES,
                },
                |reports| reports.keys.is_some(),
            )),
            (BOOT, MOUSE) => Some(boot_interface(
                legacy.clone(),
                Reports {
                    keys: None,
                    pointer: Some(Pointer::boot()),
                    input_bytes: BOOT_REPORT_BYTES,
                },
                |reports| reports.pointer.is_some(),
            )),
            _ => None,
        };
        let negotiated = match boot {
            Some((chosen, reports)) => {
                let request = setup(
                    TYPE_CLASS | RECIPIENT_INTERFACE,
                    SET_PROTOCOL,
                    chosen,
                    index,
                    0,
                );
                // Linux usbhid sends no SET_PROTOCOL: report protocol is the
                // default after reset, so a device refusing to confirm it
                // still works. Boot fallback depends on the switch.
                match control(request).await {
                    Ok(_) => {}
                    Err(BlockError::Device { status: STALL }) if chosen == REPORT_PROTOCOL => {}
                    Err(error) => return Err(error),
                }
                match control(idle).await {
                    Ok(_) | Err(BlockError::Device { status: STALL }) => {}
                    Err(error) => return Err(error),
                }
                let descriptor = if chosen == REPORT_PROTOCOL {
                    Some(described?.0)
                } else {
                    None
                };
                Negotiated {
                    chosen,
                    reports,
                    descriptor,
                    report_descriptor: raw.unwrap_or_default(),
                }
            }
            None => {
                let (descriptor, reports) = described?;
                // SET_PROTOCOL is only defined for boot-subclass devices.
                // SET_IDLE is optional for an ordinary report interface.
                match control(idle).await {
                    Ok(_) | Err(BlockError::Device { status: STALL }) => {}
                    Err(error) => return Err(error),
                }
                Negotiated {
                    chosen: REPORT_PROTOCOL,
                    reports,
                    descriptor: Some(descriptor),
                    report_descriptor: raw?,
                }
            }
        };
        Ok(negotiated)
    }

    /// The input owner [`Self::negotiate`] chose, with its transfer buffers.
    pub(in crate::usb) fn start(
        controller: Shared,
        device: &Device,
        interface: &Interface,
        endpoint: Endpoint,
        negotiated: Negotiated,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Option<Self>, BlockError> {
        let Negotiated {
            chosen,
            reports,
            descriptor,
            report_descriptor,
        } = negotiated;
        if chosen == REPORT_PROTOCOL && descriptor.as_ref().is_none_or(|d| d.input_bytes() == 0) {
            return Ok(None);
        }
        if let Some(pointer) = &reports.pointer
            && pointer.x.map(|f| f.relative) != pointer.y.map(|f| f.relative)
        {
            return Err(BlockError::Unsupported(
                "HID pointer with one absolute and one relative axis",
            ));
        }
        let length = match chosen {
            REPORT_PROTOCOL => descriptor.as_ref().unwrap().input_bytes(),
            _ => usize::from(endpoint.max_packet & MAX_PACKET_SIZE).min(BOOT_REPORT_BYTES),
        };
        if length == 0 {
            return Err(BlockError::Unsupported("HID input report without data"));
        }
        let buffer = dma.allocate(length, length.next_power_of_two())?;
        let longest = descriptor.as_ref().map_or(0, |descriptor| {
            descriptor
                .reports
                .iter()
                .filter(|layout| layout.kind == ReportType::Output)
                .map(|layout| layout.wire_bytes())
                .max()
                .unwrap_or(0)
        });
        let output = match interrupt(interface, false) {
            Some(out) if longest != 0 => {
                Some((out, dma.allocate(longest, longest.next_power_of_two())?))
            }
            _ => None,
        };
        Ok(Some(Self {
            controller,
            id: device.id,
            interface: interface.number,
            report_descriptor,
            state: State::Running,
            error: None,
            endpoint,
            buffer,
            length,
            output,
            queued: None,
            retry: None,
            translator: Translator::new(descriptor, reports),
            pending: TouchPoll::default(),
            touch: None,
            receiver: None,
            failed: false,
            name: format!("{} ({})", device.name(), device.location),
        }))
    }

    /// Legacy keyboard/pointer service. Use `poll_hid` for the complete report
    /// stream; this adapter exposes only the native `Input` vocabulary.
    pub fn poll(&mut self) -> Vec<Input> {
        self.poll_hid().inputs
    }

    /// Service the owned interrupt-IN endpoint, bounded to 16 reports.
    pub fn poll_hid(&mut self) -> Poll {
        self.poll_touch_hid().hid
    }

    /// Same HID service pass, with opt-in complete hybrid frames. Use
    /// this method to consume assembled output; `poll_hid` keeps raw output.
    pub fn poll_touch_hid(&mut self) -> TouchPoll {
        let gone = self.controller.borrow().validate_device_id(self.id).err();
        if let Some(error) = gone {
            self.invalidate_touch();
            // Linux input_disconnect_device: a device that went away, as
            // behind a KVM switch, releases what it held, once.
            if self.state == State::Running {
                let (releases, events) = self.translator.release();
                self.pending.hid.inputs.extend(releases);
                self.pending.hid.events.extend(events);
                self.fault(error);
            }
        }
        let now = self.controller.borrow().clock.now_us();
        if self.state != State::Running || self.retry.is_some_and(|retry| now < retry.at_us) {
            return self.drain_pending();
        }
        for _ in 0..16 {
            let length = self.length;
            let trb = match self.queued {
                Some(trb) => trb,
                None => {
                    let submitted =
                        self.controller
                            .borrow_mut()
                            .device(self.id)
                            .and_then(|mut device| {
                                device.submit(self.endpoint.dci(), self.buffer.physical(), length)
                            });
                    match submitted {
                        Ok(trb) => {
                            self.queued = Some(trb);
                            trb
                        }
                        Err(error) => {
                            self.fault(error);
                            break;
                        }
                    }
                }
            };
            let result = match self.controller.borrow_mut().device(self.id) {
                Ok(mut device) => device.completed(self.endpoint.dci(), trb, length),
                Err(error) => Some(Err(error)),
            };
            let Some(result) = result else {
                break;
            };
            match result {
                Ok(bytes) => {
                    self.queued = None;
                    self.retry = None;
                    let bytes = self.buffer.bytes()[..bytes].to_vec();
                    let events_at = self.pending.hid.events.len();
                    let translated = self.translator.translate(
                        &bytes,
                        &mut self.pending.hid.events,
                        &mut self.pending.hid.inputs,
                    );
                    match translated {
                        Translated::Data(frame) => {
                            if let Some(assembly) = &mut self.touch {
                                for event in &self.pending.hid.events[events_at..] {
                                    if let events::Event::Touch(packet) = event {
                                        assembly.update(
                                            &frame,
                                            packet,
                                            &mut self.pending.frames,
                                            &mut self.pending.discarded,
                                        );
                                    }
                                }
                            }
                            self.pending.hid.reports.push(ReceivedReport::Data(frame));
                            self.receiver_observe(&bytes);
                        }
                        Translated::Rejected(error) => {
                            self.pending
                                .hid
                                .reports
                                .push(ReceivedReport::Rejected { bytes, error });
                            self.invalidate_touch();
                            self.invalidate_receiver(receiver::Error::RejectedReport);
                        }
                        Translated::Boot => {
                            self.pending.hid.reports.push(ReceivedReport::Boot(bytes))
                        }
                    }
                    continue;
                }
                Err(error) => {
                    self.error.get_or_insert_with(|| error.clone());
                    self.invalidate_touch();
                    self.invalidate_receiver(receiver::Error::Transport);
                    let recovered =
                        self.controller
                            .borrow_mut()
                            .device(self.id)
                            .and_then(|mut device| {
                                if error == (BlockError::Device { status: STALL }) {
                                    device.clear(&self.endpoint)
                                } else {
                                    device.reset_endpoint(&self.endpoint)
                                }
                            });
                    if recovered.is_ok() {
                        self.queued = None;
                    }
                    self.retry = match self.retry {
                        None => Some(Retry::first(now)),
                        Some(retry) => retry.next(now),
                    };
                    if let Err(error) = recovered {
                        self.fault(error);
                    } else if self.retry.is_none() {
                        self.fault(error);
                    }
                    break;
                }
            }
        }
        self.receiver_progress();
        self.drain_pending()
    }

    /// Take completed output without hardware access. After an interrupted
    /// service pass this preserves its prefix; it does not cancel a transfer,
    /// reset the endpoint or grant DMA retirement/retry permission.
    pub fn drain_pending(&mut self) -> TouchPoll {
        core::mem::take(&mut self.pending)
    }

    fn fault(&mut self, error: BlockError) {
        self.error.get_or_insert(error);
        self.failed = true;
        self.state = State::Faulted;
        self.invalidate_touch();
        self.invalidate_receiver(receiver::Error::Transport);
    }

    /// The last report received, as it arrived, for diagnostics.
    pub fn last_report(&self) -> &[u8] {
        self.translator.last_report()
    }

    /// Keyboard reports holding usages the native keyboard cannot report.
    pub fn dropped(&self) -> u64 {
        self.translator.dropped()
    }

    /// Whether polling stopped on a transfer failure.
    pub fn failed(&self) -> bool {
        self.failed
    }
}

#[cfg(test)]
mod tests;
