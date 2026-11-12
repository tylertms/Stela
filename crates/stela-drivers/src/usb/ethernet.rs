//! Vendor USB Ethernet backends. Candidate interfaces become usable links only
//! after exact chip identification and revision-specific initialization.
//! RTL8152 01/02/07, RTL8153A 03/04/05/06, RTL8153B 08/09, RTL8153C 14
//! and RTL8156A 10/11 and RTL8156B 12/13/15 have distinct implementations.
//! Revision15 follows the donor's gigabit capability policy. ASIX AX88179/178A
//! uses its own control protocol and packet framing behind the shared DMA path.
mod asix;
pub mod firmware;
mod framing;
mod identity;
mod minimum;
mod registers;
use minimum::MinimumControl;
pub mod rtl815x;

use super::descriptor::{Endpoint, Interface};
use super::xhci::{Device, DeviceAccess, Operation, Shared, SlotId, TransferTicket};
use crate::{
    Deadline,
    block::BlockError,
    bus::dma::{DmaAllocator, DmaBuffer},
    net::{Link, LinkError, Mac},
};
use alloc::{collections::VecDeque, format, rc::Rc, string::String, vec::Vec};
#[cfg(test)]
pub(crate) use asix::LegacyModel;
#[cfg(test)]
pub(crate) use asix::model::Hardware as AsixModel;
use core::{cell::Cell, future::Future, task::Poll};
use stela_driver_base::stepped::{Stepped, block_on, until, yield_now};
#[cfg(test)]
pub(crate) fn asix_legacy_ids() -> Vec<(u16, u16)> {
    asix::legacy_ids()
        .iter()
        .map(|entry| (entry.0, entry.1))
        .collect()
}
use core::sync::atomic::{Ordering, fence};
pub use firmware::Error;
#[cfg(test)]
pub(crate) use identity::USB_IDS;
pub use identity::matches;
use identity::policy;
use rtl815x::{Chip, Revision};

/// Whether a vendor Ethernet backend's ID table lists the device. Only listed
/// devices are adapters; other vendor-class interfaces, such as LED or sensor
/// controllers, belong to no driver here and get no chip identification.
pub fn listed(descriptor: &[u8; 18]) -> bool {
    matches(descriptor) || asix::listed(descriptor)
}

#[cfg(test)]
pub(crate) fn asix_ids() -> Vec<(u16, u16)> {
    asix::ids().iter().map(|entry| (entry.0, entry.1)).collect()
}

/// Endpoint-zero requests and the waits between them, as futures: the chip
/// code written once over them runs to completion at attach ([`block_on`])
/// or a step per runtime call. Success includes the exact transferred byte
/// count; short replies/writes are errors, never invented register values.
pub trait Control {
    fn control(
        &mut self,
        request: [u8; 8],
        output: &[u8],
    ) -> impl Future<Output = Result<Vec<u8>, BlockError>>;
    /// Nominal polling cadence only; never authorizes a hardware minimum.
    fn delay_us(&mut self, microseconds: u64) -> impl Future<Output = ()>;
    /// Required physical quiet interval from this actual device's owner.
    fn minimum_wait(&mut self, microseconds: u64) -> impl Future<Output = Result<(), BlockError>>;
}

#[derive(Clone, Debug)]
pub struct Binding {
    input: Endpoint,
    output: Endpoint,
    kind: Kind,
    setting: (u8, u8),
}

#[derive(Clone, Copy, Debug)]
enum Kind {
    Realtek(Revision),
    Asix(asix::Identity),
}

impl Binding {
    pub(super) fn new(device: &Device, interface: &Interface) -> Result<Self, &'static str> {
        let kind = if let Some(identity) = asix::identify(&device.descriptor, interface) {
            if matches!(identity, asix::Identity::Ax88772 { .. })
                && !matches!(
                    device.speed,
                    super::xhci::FULL_SPEED | super::xhci::HIGH_SPEED
                )
            {
                return Err("AX88772 requires USB full or high speed");
            }
            Kind::Asix(identity)
        } else {
            let revision = device
                .vendor_ethernet_revision
                .map(Revision)
                .ok_or("USB Ethernet chip revision not identified")?;
            if !matches(&device.descriptor) || !revision.implemented() {
                return Err("RTL815x chip revision initialization not implemented");
            }
            if interface.class != 0xff || interface.alternate != 0 || interface.number != 0 {
                return Err("RTL815x unsupported vendor interface setting");
            }
            Kind::Realtek(revision)
        };
        let input: Vec<_> = interface
            .endpoints
            .iter()
            .copied()
            .filter(|e| e.kind() == 2 && e.is_in())
            .collect();
        let output: Vec<_> = interface
            .endpoints
            .iter()
            .copied()
            .filter(|e| e.kind() == 2 && !e.is_in())
            .collect();
        if input.len() != 1 || output.len() != 1 {
            return Err("USB vendor Ethernet requires unique bulk IN/OUT endpoints");
        }
        // rtl_check_vendor_ok fixes the controller's RX/TX pipe numbers.
        // Carrier is polled, so the unused interrupt endpoint is not required.
        if matches!(kind, Kind::Realtek(_))
            && (input[0].address != 0x81 || output[0].address != 0x02)
        {
            return Err("RTL815x requires RX endpoint1 and TX endpoint2");
        }
        Ok(Self {
            input: input[0],
            output: output[0],
            kind,
            setting: (interface.number, interface.alternate),
        })
    }

    pub(super) fn setting(&self) -> (u8, u8) {
        self.setting
    }

    pub(super) fn observed_vendor(&self) -> bool {
        matches!(self.kind, Kind::Realtek(_))
    }
    pub(super) fn asix_candidate(&self) -> bool {
        matches!(self.kind, Kind::Asix(_))
    }

    pub(super) fn endpoints(&self) -> Vec<Endpoint> {
        vec![self.input, self.output]
    }
}

pub(super) struct HostControl {
    pub(super) controller: Shared,
    pub(super) id: SlotId,
    /// Whether a wait ends the step until it passes rather than holding the
    /// processor through it: off while attach runs to completion, on for
    /// the link's work in steps.
    pub(super) stepped: Rc<Cell<bool>>,
}

impl Control for HostControl {
    async fn control(&mut self, request: [u8; 8], output: &[u8]) -> Result<Vec<u8>, BlockError> {
        operation(&self.controller, self.id, |c| {
            if !c.connected()? {
                return Err(BlockError::Unsupported("USB Ethernet disconnected"));
            }
            c.start_control(request, output)
        })
        .await
    }
    async fn delay_us(&mut self, microseconds: u64) {
        let clock = self.controller.borrow().clock;
        if self.stepped.get() {
            until(clock.deadline(microseconds)).await;
        } else {
            clock.delay(microseconds);
        }
    }
    async fn minimum_wait(&mut self, microseconds: u64) -> Result<(), BlockError> {
        let interval = {
            let mut controller = self.controller.borrow_mut();
            controller.device(self.id)?.connected()?;
            if !self.stepped.get() {
                return controller.minimum_wait(microseconds);
            }
            controller.begin_minimum(microseconds)?
        };
        let Some(interval) = interval else {
            // A timer that keeps no interval waits in full; the step ends
            // after it.
            self.controller.borrow_mut().minimum_wait(microseconds)?;
            yield_now().await;
            return Ok(());
        };
        while !self
            .controller
            .borrow_mut()
            .minimum_elapsed(interval.as_ref())?
        {
            yield_now().await;
        }
        Ok(())
    }
}

/// An endpoint operation `start` begins on the device, looked at once per
/// step until it ends; the controller is borrowed only within a step.
pub(super) async fn operation(
    controller: &Shared,
    id: SlotId,
    start: impl FnOnce(&mut DeviceAccess<'_>) -> Result<Operation, BlockError>,
) -> Result<Vec<u8>, BlockError> {
    let mut operation = start(&mut controller.borrow_mut().device(id)?)?;
    loop {
        if let Some(result) = controller.borrow_mut().device(id)?.poll(&mut operation) {
            return result;
        }
        yield_now().await;
    }
}

enum Backend {
    Realtek(Chip<HostControl>),
    Asix(asix::Device<HostControl>),
}
impl Backend {
    fn receive_bytes(&self) -> usize {
        match self {
            Self::Realtek(chip) => chip.receive_bytes(),
            Self::Asix(chip) => chip.receive_bytes(),
        }
    }
    async fn carrier(&mut self) -> Result<bool, BlockError> {
        match self {
            Self::Realtek(chip) => chip.carrier().await,
            Self::Asix(chip) => chip.carrier().await,
        }
    }
    async fn link_down(&mut self) -> Result<(), BlockError> {
        match self {
            Self::Realtek(c) => c.stop().await,
            Self::Asix(c) => c.link_down().await,
        }
    }
    async fn stop(&mut self) -> Result<(), BlockError> {
        match self {
            Self::Realtek(chip) => chip.stop().await,
            Self::Asix(chip) => chip.stop().await,
        }
    }
    fn encode(&self, frame: &[u8], output: &mut [u8], packet: u16) -> Result<usize, &'static str> {
        match self {
            Self::Realtek(_) => framing::encode(frame, output),
            Self::Asix(chip) => chip.encode(frame, output, packet),
        }
    }
    fn decode(&mut self, bytes: &[u8]) -> Result<Vec<Vec<u8>>, &'static str> {
        match self {
            Self::Realtek(_) => framing::decode(bytes),
            Self::Asix(chip) => chip.decode(bytes),
        }
    }
}

/// How long one transmit TD may stay outstanding before it is cancelled.
const TRANSMIT_US: u64 = 1_000_000;
/// How often the PHY is asked for the link. Linux's drivers learn of link
/// changes from the adapter's interrupt endpoint, polled at its bInterval;
/// a tenth of a second keeps about that latency without a control transfer
/// on every link call.
const LINK_POLL_US: u64 = 100_000;

/// The transmit TD, which holds the send buffer.
enum Sending {
    /// The controller owns it: its ticket, its length and when it times out.
    InFlight {
        ticket: TransferTicket,
        bytes: usize,
        deadline: Deadline,
    },
    /// It failed with this error, reported once its endpoint's reset, which
    /// work in steps runs, acknowledged the buffer free.
    Failed(BlockError),
}

/// What work in steps does.
enum Work {
    /// The carrier check: the PHY's link read, and the MAC set up for a new
    /// link.
    Carrier,
    /// The lost link: both bulk endpoints cancelled, then the MAC stopped.
    LinkDown,
    /// A failed receive transfer's endpoint reset.
    ClearInput,
    /// The transmit TD's endpoint reset after a STALL, or its cancellation
    /// past its deadline.
    ResetOutput,
}

/// Device work run a step per link call, each step ending at the first
/// control transfer, command or wait not yet complete, so no call holds the
/// processor for the seconds a slow device's transfers may take. It holds
/// the chip until it ends; meanwhile receive finds nothing and transmit is
/// busy.
struct Running {
    work: Work,
    task: Stepped<'static, Finished>,
}
struct Finished {
    chip: Backend,
    result: Result<bool, BlockError>,
}

/// Cancel both bulk endpoints' outstanding TDs, a command or poll per step.
async fn quiesce(controller: &Shared, id: SlotId, endpoints: [u8; 2]) -> Result<(), BlockError> {
    for dci in endpoints {
        operation(controller, id, |c| c.start_cancel(dci)).await?;
    }
    Ok(())
}

/// One receive TD and one transmit TD, each in flight across link calls, as
/// usbnet keeps its URBs; DMA allocations persist for the boot even after
/// failed cancellation, as required by xHCI ownership.
pub struct UsbEthernet {
    controller: Shared,
    id: SlotId,
    binding: Binding,
    /// The chip, which work in steps holds while it runs.
    chip: Option<Backend>,
    running: Option<Running>,
    /// When the next carrier check is due; none when it is due now.
    link_due: Option<Deadline>,
    mac: Mac,
    receive: DmaBuffer,
    send: DmaBuffer,
    queued: Option<TransferTicket>,
    /// The transmit TD; the send buffer is the controller's until it
    /// completes or its cancellation is acknowledged.
    sending: Option<Sending>,
    pending: VecDeque<Vec<u8>>,
    carrier: bool,
    closed: bool,
    stopped: bool,
    pub firmware: Option<firmware::Receipt>,
    /// Why the chip runs without its firmware patch: the file was missing
    /// or failed validation, and r8152 then starts the chip unpatched.
    pub firmware_skipped: Option<firmware::Error>,
    /// Receive transfers that failed and aggregates that did not decode,
    /// dropped as r8152 and the ASIX drivers count them in `rx_errors`.
    rx_dropped: u64,
    pub location: String,
}

impl UsbEthernet {
    /// Receive transfers and aggregates dropped as damaged. Saturates.
    pub fn rx_dropped(&self) -> u64 {
        self.rx_dropped
    }
    pub fn driver(&self) -> &'static str {
        match self.binding.kind {
            Kind::Realtek(_) => "usb-rtl815x",
            Kind::Asix(_) => "usb-asix",
        }
    }

    pub(super) fn new(
        controller: Shared,
        device: &Device,
        binding: Binding,
        dma: &mut dyn DmaAllocator,
        provider: &dyn crate::firmware::Provider,
    ) -> Result<Self, Error> {
        controller.borrow_mut().device(device.id)?;
        if binding.setting.1 != 0 {
            controller.borrow_mut().device(device.id)?.control(
                super::descriptor::setup(
                    1,
                    super::descriptor::SET_INTERFACE,
                    u16::from(binding.setting.1),
                    u16::from(binding.setting.0),
                    0,
                ),
                &[],
            )?;
        }
        let stepped = Rc::new(Cell::new(false));
        let io = HostControl {
            controller: controller.clone(),
            id: device.id,
            stepped: stepped.clone(),
        };
        let (chip, mac, firmware, family, firmware_skipped) = match binding.kind {
            Kind::Realtek(revision) => {
                // rtl8152_request_firmware: a missing or invalid patch is
                // reported and the chip starts without it.
                let (firmware, skipped) =
                    match firmware::load(provider, &device.descriptor, revision) {
                        Ok(firmware) => (firmware, None),
                        Err(error) => (None, Some(error)),
                    };
                let (chip, mac) = block_on(Chip::attach_with_policy(
                    io,
                    revision,
                    device.speed,
                    firmware.as_ref().map(|(_, parsed)| parsed),
                    policy(device),
                ))?;
                let family = if revision.is_8153a() {
                    "RTL8153A"
                } else if revision.is_8153b() {
                    "RTL8153B"
                } else if revision.is_8153c() {
                    "RTL8153C"
                } else if revision.is_8156a() {
                    "RTL8156A"
                } else if revision.0 == 0x7420 {
                    "RTL815x tcr-7420 (gigabit)"
                } else if revision.is_8156b() {
                    "RTL8156B"
                } else {
                    "RTL8152"
                };
                let receipt = firmware.map(|(image, _)| firmware::Receipt::from_image(image));
                let family = match skipped {
                    Some(_) => format!("{family} without its firmware patch"),
                    None => String::from(family),
                };
                (Backend::Realtek(chip), mac, receipt, family, skipped)
            }
            Kind::Asix(identity) => {
                let (chip, mac) = block_on(asix::Device::attach(io, identity))?;
                let family = match identity {
                    asix::Identity::Ax88179 => "AX88179",
                    asix::Identity::Ax88178a => "AX88178A",
                    asix::Identity::Ax88772 { .. } => "AX88772/A/B",
                };
                let register = if matches!(identity, asix::Identity::Ax88772 { .. }) {
                    "smsr"
                } else {
                    "general"
                };
                let family = format!(
                    "{family} {register}-{:02x} phy-{:08x}",
                    chip.identity().0,
                    chip.identity().1
                );
                (Backend::Asix(chip), mac, None, family, None)
            }
        };
        // Attach ran to completion; the link's work runs in steps.
        stepped.set(true);
        let receive_bytes = chip.receive_bytes();
        Ok(Self {
            controller,
            id: device.id,
            binding,
            chip: Some(chip),
            running: None,
            link_due: None,
            mac,
            receive: dma.allocate(receive_bytes, receive_bytes.next_power_of_two())?,
            send: dma.allocate(2048, 2048)?,
            queued: None,
            sending: None,
            pending: VecDeque::new(),
            carrier: false,
            closed: false,
            stopped: false,
            firmware,
            firmware_skipped,
            rx_dropped: 0,
            location: format!("{} {family}", device.location),
        })
    }

    fn open(&mut self) -> Result<(), BlockError> {
        if self.closed {
            return Err(BlockError::Unsupported("USB Ethernet closed"));
        }
        if !self.controller.borrow_mut().device(self.id)?.connected()? {
            self.closed = true;
            return Err(BlockError::Unsupported("USB Ethernet disconnected"));
        }
        Ok(())
    }

    /// Hand the chip to `work`, which runs `run` on it a step per call.
    fn begin<F>(&mut self, work: Work, run: impl FnOnce(Backend) -> F)
    where
        F: Future<Output = Finished> + 'static,
    {
        let chip = self.chip.take().expect("no work holds the chip");
        self.running = Some(Running {
            work,
            task: Stepped::new(run(chip)),
        });
    }

    fn begin_carrier(&mut self) {
        self.begin(Work::Carrier, |mut chip| async move {
            let result = chip.carrier().await;
            Finished { chip, result }
        });
    }

    fn begin_link_down(&mut self) {
        let (controller, id) = (self.controller.clone(), self.id);
        let endpoints = [self.binding.input.dci(), self.binding.output.dci()];
        self.begin(Work::LinkDown, move |mut chip| async move {
            let result = async {
                quiesce(&controller, id, endpoints).await?;
                chip.link_down().await
            }
            .await;
            Finished {
                chip,
                result: result.map(|()| false),
            }
        });
    }

    /// Reset one bulk endpoint's host side, with CLEAR_FEATURE(ENDPOINT_HALT)
    /// for a STALL (`clear`), or only cancel its TD.
    fn begin_reset(&mut self, work: Work, endpoint: Endpoint, clear: bool) {
        let (controller, id) = (self.controller.clone(), self.id);
        self.begin(work, move |chip| async move {
            let result = operation(&controller, id, |c| match clear {
                true => c.start_clear(&endpoint),
                false => c.start_cancel(endpoint.dci()),
            })
            .await;
            Finished {
                chip,
                result: result.map(|_| true),
            }
        });
    }

    /// One step of the work under way, and of any work its end starts:
    /// pending while it runs, otherwise the first failure. A failure closes
    /// the link, as the chip and endpoint it leaves cannot be trusted.
    fn advance(&mut self) -> Poll<Result<(), BlockError>> {
        while let Some(running) = &mut self.running {
            let Poll::Ready(finished) = running.task.step() else {
                return Poll::Pending;
            };
            let work = self.running.take().unwrap().work;
            self.chip = Some(finished.chip);
            if let Err(error) = finished
                .result
                .and_then(|linked| self.finished(work, linked))
            {
                self.closed = true;
                self.carrier = false;
                return Poll::Ready(Err(error));
            }
        }
        Poll::Ready(Ok(()))
    }

    /// What ended work leaves behind.
    fn finished(&mut self, work: Work, linked: bool) -> Result<(), BlockError> {
        match work {
            Work::Carrier => {
                // A clock that keeps no deadline checks at every call.
                self.link_due = self
                    .controller
                    .borrow()
                    .clock
                    .try_deadline(LINK_POLL_US)
                    .ok();
                if self.carrier && !linked {
                    self.carrier = false;
                    self.begin_link_down();
                } else {
                    self.carrier = linked;
                }
            }
            Work::LinkDown => {
                self.queued = None;
                self.sending = None;
                self.pending.clear();
            }
            Work::ClearInput => self.rx_dropped = self.rx_dropped.saturating_add(1),
            Work::ResetOutput => {}
        }
        Ok(())
    }

    /// The work under way run to its end in this call, as teardown may.
    fn finish(&mut self) -> Result<(), BlockError> {
        loop {
            if let Poll::Ready(result) = self.advance() {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// A step of the work under way, and a carrier check when one is due;
    /// work that ended in this call had its step, so a due check starts at
    /// the next.
    fn update_carrier(&mut self) -> Result<(), BlockError> {
        self.open()?;
        let busy = self.running.is_some();
        match self.advance() {
            Poll::Pending => return Ok(()),
            Poll::Ready(result) => result?,
        }
        if !busy && self.link_due.as_ref().is_none_or(Deadline::passed) {
            self.begin_carrier();
            if let Poll::Ready(result) = self.advance() {
                result?;
            }
        }
        Ok(())
    }

    /// Stop requires acknowledged cancellation of both bulk endpoints before
    /// the device reset; failed stops remain closed and never reuse DMA.
    /// Work under way ends first. It runs to completion, as teardown may.
    pub fn stop(&mut self) -> Result<(), BlockError> {
        self.controller.borrow_mut().device(self.id)?;
        if self.stopped {
            return Ok(());
        }
        self.closed = true;
        // Work that failed left a chip or endpoint that refuses the stop.
        let _ = self.finish();
        self.carrier = false;
        let endpoints = [self.binding.input.dci(), self.binding.output.dci()];
        block_on(quiesce(&self.controller, self.id, endpoints))?;
        self.queued = None;
        self.sending = None;
        self.pending.clear();
        let chip = self.chip.as_mut().expect("no work holds the chip");
        block_on(chip.stop())?;
        self.stopped = true;
        Ok(())
    }
}

impl Drop for UsbEthernet {
    fn drop(&mut self) {
        // Failure cannot release boot-lifetime DMA or claim device quiescence.
        let _ = self.stop();
    }
}

fn link_error(error: BlockError) -> LinkError {
    LinkError::Adapter(format!("USB Ethernet: {error}"))
}

impl UsbEthernet {
    /// Whether the send buffer is free: no transmit TD in flight, or the one
    /// in flight completed. A failed TD is reported once its endpoint is
    /// reset, and one past its deadline once its cancellation is
    /// acknowledged, both as work in steps; until then the buffer stays the
    /// controller's.
    fn sent(&mut self) -> Result<bool, LinkError> {
        let (ticket, bytes, deadline) = match &self.sending {
            None => return Ok(true),
            Some(Sending::InFlight {
                ticket,
                bytes,
                deadline,
            }) => (ticket, *bytes, deadline),
            // No work runs: its reset ended.
            Some(Sending::Failed(_)) => return self.reset_sent(),
        };
        let result = {
            let mut controller = self.controller.borrow_mut();
            let mut c = controller.device(self.id).map_err(link_error)?;
            c.completed_ticket(ticket)
        };
        let (error, clear) = match result {
            Some(Ok(actual)) => {
                self.sending = None;
                if actual != bytes {
                    return Err(LinkError::Adapter("USB Ethernet short TX transfer".into()));
                }
                return Ok(true);
            }
            // A STALL completes the request but the halted hardware can
            // still reference its old TD until Reset/Set Dequeue succeeds.
            Some(Err(error)) => (error, true),
            None if deadline.passed() => (BlockError::Timeout, false),
            None => return Ok(false),
        };
        self.sending = Some(Sending::Failed(error));
        self.begin_reset(Work::ResetOutput, self.binding.output, clear);
        if let Poll::Ready(result) = self.advance() {
            result.map_err(link_error)?;
        }
        if self.running.is_some() {
            return Ok(false);
        }
        self.reset_sent()
    }

    /// The failed transmit TD's error, its endpoint reset.
    fn reset_sent(&mut self) -> Result<bool, LinkError> {
        match self.sending.take() {
            Some(Sending::Failed(error)) => Err(link_error(error)),
            _ => unreachable!("only a failed TD is reset"),
        }
    }
}

impl Link for UsbEthernet {
    fn mac(&self) -> Mac {
        self.mac
    }
    fn carrier(&mut self) -> bool {
        self.update_carrier().is_ok() && self.carrier
    }
    /// Queue `frame` and return without waiting for the controller, as
    /// usbnet_start_xmit submits its URB. While the previous frame is still
    /// in flight the ring is full, and while work in steps holds the chip
    /// it is busy: the frame is refused as busy at once.
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.update_carrier().map_err(link_error)?;
        if !self.carrier {
            return Err(LinkError::Adapter("USB Ethernet carrier down".into()));
        }
        if self.running.is_some() || !self.sent()? {
            return Err(LinkError::Busy);
        }
        let deadline = self
            .controller
            .borrow()
            .clock
            .try_deadline(TRANSMIT_US)
            .map_err(|_| LinkError::Adapter("USB Ethernet transmit clock unavailable".into()))?;
        let mut controller = self.controller.borrow_mut();
        let mut c = controller.device(self.id).map_err(link_error)?;
        c.endpoint_ready(self.binding.output.dci())
            .map_err(link_error)?;
        let bytes = self
            .chip
            .as_ref()
            .expect("no work holds the chip")
            .encode(frame, self.send.bytes_mut(), self.binding.output.max_packet)
            .map_err(|why| LinkError::Adapter(why.into()))?;
        fence(Ordering::Release);
        // The vendor protocol carries its own lengths; Linux submits no ZLP.
        let ticket = c
            .submit_ticket(self.binding.output.dci(), self.send.physical(), bytes)
            .map_err(link_error)?;
        drop(controller);
        self.sending = Some(Sending::InFlight {
            ticket,
            bytes,
            deadline,
        });
        // A controller that already finished reports the outcome now.
        self.sent().map(|_| ())
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.update_carrier().map_err(link_error)?;
        if !self.carrier || self.running.is_some() {
            return Ok(None);
        }
        if let Some(frame) = self.pending.pop_front() {
            return Ok(Some(frame));
        }
        let endpoint = self.binding.input;
        let result = {
            let mut controller = self.controller.borrow_mut();
            let mut c = controller.device(self.id).map_err(link_error)?;
            if self.queued.is_none() {
                let token = c
                    .submit_ticket(endpoint.dci(), self.receive.physical(), self.receive.len())
                    .map_err(link_error)?;
                self.queued = Some(token);
            }
            c.completed_ticket(self.queued.as_ref().unwrap())
        };
        let Some(result) = result else {
            return Ok(None);
        };
        self.queued = None;
        let actual = match result {
            Ok(bytes) => bytes,
            Err(_) => {
                // A failed transfer (a STALL, babble or transaction error)
                // is counted and dropped once its endpoint is reset, as
                // r8152 and usbnet resubmit; an endpoint that cannot be
                // reset is a dead device.
                self.begin_reset(Work::ClearInput, endpoint, true);
                if let Poll::Ready(result) = self.advance() {
                    result.map_err(link_error)?;
                }
                return Ok(None);
            }
        };
        if actual > self.receive.len() {
            return Err(LinkError::Adapter(
                "USB Ethernet RX completion exceeds DMA".into(),
            ));
        }
        fence(Ordering::Acquire);
        // An aggregate that fails its checks is dropped whole, as a NIC
        // drops a frame that fails its own.
        let bytes = &self.receive.bytes()[..actual];
        match self
            .chip
            .as_mut()
            .expect("no work holds the chip")
            .decode(bytes)
        {
            Ok(frames) => self.pending.extend(frames),
            Err(_) => self.rx_dropped = self.rx_dropped.saturating_add(1),
        }
        Ok(self.pending.pop_front())
    }
}

#[cfg(test)]
pub(crate) mod tests {
    pub(crate) mod generation;
    pub(crate) mod stepped;
    use super::*;

    impl UsbEthernet {
        /// The carrier once the work under way, and any work it starts,
        /// ended.
        pub(crate) fn settled_carrier(&mut self) -> bool {
            for _ in 0..100_000 {
                let up = self.carrier();
                if self.running.is_none() {
                    return up;
                }
            }
            panic!("link work never ended");
        }

        /// The carrier a check due now finds, once its work ended.
        pub(crate) fn checked_carrier(&mut self) -> bool {
            let clock = self.controller.borrow().clock;
            crate::test_clock::advance(clock, LINK_POLL_US);
            self.settled_carrier()
        }
    }

    #[test]
    #[ignore = "oracle: STELA_RTL815X_IDS_ORACLE"]
    fn unchanged_linux_rtl815x_exact_usb_candidates_and_flags() {
        extern crate std;
        use sha2::{Digest, Sha256};
        let ids =
            std::fs::read_to_string(std::env::var("STELA_RTL815X_IDS_ORACLE").unwrap()).unwrap();
        assert_eq!(
            format!("{:x}", Sha256::digest(ids.as_bytes())),
            "e722f13b67f558590bbe36bfa80ca12174007f5f8a2a661a2f0d84bfd8b3aab1"
        );
        let expected: Vec<_> = ids
            .lines()
            .map(|line| {
                let (vid, pid) = line.split_once(' ').unwrap();
                (
                    u16::from_str_radix(vid, 16).unwrap(),
                    u16::from_str_radix(pid, 16).unwrap(),
                )
            })
            .collect();
        assert_eq!(USB_IDS, expected);
        for (vid, pid) in expected {
            let mut descriptor = [0; 18];
            descriptor[8..10].copy_from_slice(&vid.to_le_bytes());
            descriptor[10..12].copy_from_slice(&pid.to_le_bytes());
            assert!(matches(&descriptor));
            descriptor[8..10].copy_from_slice(&(vid ^ 0x8000).to_le_bytes());
            assert!(!matches(&descriptor));
            descriptor[8..10].copy_from_slice(&vid.to_le_bytes());
            descriptor[10..12].copy_from_slice(&0xffffu16.to_le_bytes());
            assert!(!matches(&descriptor));
        }
    }
}
