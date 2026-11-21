//! Synopsys DWC3 USB2 device-mode core, independently testable through DMA.
//!
//! Evidence: Linux a0ff06b3 `drivers/usb/dwc3/{core.h,gadget.c,ep0.c}`;
//! AppleUSBXDCI::sendEndpointCommand KC file 0x42ecd50 uses PAR0/PAR1/PAR2
//! at Apple offsets 0x808/804/800 and command at 0x80c. Apple's reg[2]
//! is the reg[0] aperture plus 0xc000, not a second register bank.
//! AppleUSBXDCIEventHandler::initRegisters (0x43063d4, writes at
//! 0x43065c0..0x43066dc) programs four-byte events into a 0x800-byte ring.
//! TI SPRUHJ7A tables 2-24..34 and 2-53 specify event status/parameters
//! and short-packet actions; non-stream completions have stream parameter 0.
//!
//! Stela handoff choice: disabled device, no connection, zero software
//! configuration, no transfers, four masked empty event buffers. This is
//! neither a captured iBoot image nor an assertion of Apple's reset values.
//! Identification is the declared DWC3 3.00a programming profile (0x5533300a),
//! 32 physical endpoints, four interrupt buffers, UTMI high-speed only, no
//! streams/hibernation/OTG/SuperSpeed. Endpoints are control EP0, bulk and
//! interrupt. The global configuration `AppleUSBXDCI::initDeviceController`
//! (0x42f97f0), `goOnBus` (0x42f4ef8) and `updateLPMPolicy` (0x43037f4)
//! program — GUCTL1, GUID, GTXFIFOPRIDEV, the DCFG speed maximum and LPM
//! capability, the DCTL LPM and U1/U2 fields, the wakeup and suspend event
//! enables — is held where no modeled link observes it (the field
//! constants say why); other fields, offsets and widths refuse.
//!
//! GCTL.PRTCAPDIR selects the dual-role core's host (1) or device (2)
//! personality. `AppleT8112USBXHCI::start` read-modify-writes it to host at
//! KC file 0x4457644 whatever the port's role; the device driver selects
//! device again when it powers the core. Host mode idles the device
//! personality like a core reset; OTG and the reserved selection refuse.
//! The host personality's registers (`xhci`) occupy the core's first
//! 4 KiB; host interrupter n shares event buffer n's wire, and a GCTL core
//! soft reset resets both personalities. The Apple registers the host
//! driver programs around the core are `apple`'s.
mod apple;
mod regs;
pub mod xhci;

use crate::Result;
use alloc::{collections::VecDeque, string::String};
use regs::*;
use stela_contracts::Refusal;
use stela_contracts::refuse;
use {
    super::typec::Usb2Speed, stela_contracts::bus::Access, stela_contracts::bus::Direction,
    stela_contracts::bus::MmioDevice, stela_contracts::bus::MmioError, stela_contracts::bus::Width,
    stela_contracts::bus::refuse, stela_contracts::interrupt::AicInput,
    stela_contracts::interrupt::Wire,
};
use {
    crate::usb::BusPeer, crate::usb::Function, crate::usb::MAX_PACKET, crate::usb::Reply,
    crate::usb::Token, stela_contracts::dma::DmaError, stela_contracts::dma::Port,
};

const NAME: &str = "dwc3-device";
pub const PLATFORM_WINDOW_BASE: u64 = 0x20000;
pub const PLATFORM_WINDOW_STRIDE: u64 = 0x4000;
/// GUCTL1 fields the device driver sets (Linux `core.h` names):
/// DEV_L1_EXIT_BY_HW (24, `updateLPMPolicy`), DEV_FORCE_20_CLK_FOR_30_CLK
/// (26, `goOnBus`) and DEV_DECOUPLE_L1L2_EVT (31, `initDeviceController`,
/// KC 0x42fa004). They shape LPM exit, the SuperSpeed clock source and
/// suspend event reporting, none of which a high-speed link that never
/// enters L1 or suspend observes; they are held only.
const GUCTL1_FIELDS: u32 = 1 << 24 | 1 << 26 | 1 << 31;
/// DCTL: run, soft reset, the LPM NYET threshold (23:20) and the
/// SuperSpeed U1/U2 accept/initiate enables (12:9), which no high-speed
/// link uses. Link state change requests, test modes, save/restore and
/// hibernation refuse.
const DCTL_FIELDS: u32 = RUN | RESET | 0xf << 20 | 0xf << 9;
/// DEVTEN: disconnect, reset, connect done, wakeup, suspend (EOPF) and
/// command complete. The host peer never suspends or resumes the bus, so
/// the wakeup and suspend events never occur.
const DEVTEN_FIELDS: u32 = 1 | 1 << 1 | 1 << 2 | 1 << 4 | 1 << 6 | 1 << 10;
/// DCFG: device speed (2:0), address, receive buffer count and LPM
/// capability. A SuperSpeed (4) or SuperSpeedPlus (5) maximum connects at
/// high speed, as on a core with no SuperSpeed link; full and low speed
/// refuse. With LPM capable the core would answer LPM tokens, which the
/// host peer never sends.
const DCFG_FIELDS: u32 = 7 | 0x7f << 3 | 0x1f << 17 | 1 << 22;
const MAX_PENDING_EVENTS: usize = 256;
const MAX_LINKS_PER_PACKET: usize = 8;
/// Bulk and interrupt transfer steps kept for diagnosis (`transfer_history`).
const HISTORY_STEPS: usize = 48;
const FIFO_WORDS: u32 = 4096;
const CORE_RESET: u32 = 1 << 11;
const HALTED: u32 = 1 << 22;
const RX_EMPTY: u32 = 1 << 17;
/// EP0's largest configurable packet. Before Connect Done reports the
/// link speed, drivers configure EP0 for SuperSpeed's 512 bytes (Linux
/// `dwc3_gadget_ep0_desc`, `AppleUSBXDCI::initDeviceController` step 11),
/// then modify it to the connected speed's 64. The host peer's own
/// high-speed packets bound every transaction in between.
const MAX_PACKET_CONTROL: usize = 512;
/// DEPCFG parameter 0 burst size (25:22). Bursts are SuperSpeed-only; a
/// high-speed link never observes them, so a bulk endpoint's value is
/// only. Apple's driver sets 15 on every bulk endpoint.
const BURST: u32 = 0xf << 22;
const CMD_FAILURE: u32 = 1 << 12;
const PRTCAP: u32 = 3 << 12;
const PRTCAP_HOST: u32 = 1 << 12;
const PRTCAP_DEVICE: u32 = 2 << 12;

/// Minimal port boundary until the independent Type-C model lands. A new
/// connection generation is a detach: the driver ends its own transfers.
/// The provider must derive readiness from its PHY/role lifecycle.
pub trait PortState {
    fn generation(&self) -> u64;
    fn high_speed_device_ready(&self) -> bool;
}

#[derive(Clone, Copy)]
struct Ring {
    address: u64,
    size: u32,
    masked: bool,
    count: u32,
    producer: u32,
    asserted: bool,
}
impl Ring {
    const fn new() -> Self {
        Self {
            address: 0,
            size: 0,
            masked: true,
            count: 0,
            producer: 0,
            asserted: false,
        }
    }
}

#[derive(Clone, Copy, Default)]
struct Endpoint {
    parameters: [u32; 3],
    command: u32,
    configured: bool,
    resource: bool,
    stalled: bool,
    ring: usize,
    packet: usize,
    events: u32,
    active: Option<Transfer>,
    not_ready: bool,
    /// The transfer stopped at a TRB software did not own the hardware
    /// for: the core fetches it again only after Update or Start Transfer
    /// (DWC3 databook "Update Transfer"), so software may write the TRB's
    /// words in any order before it issues the command.
    parked: bool,
}

#[derive(Clone, Copy)]
struct Transfer {
    trb: u64,
    done: u32,
    /// The leading bytes of an OUT TRB's payload as the core wrote them,
    /// read back at completion: software owns the buffer only after.
    head: [u8; HEAD_BYTES],
}

/// [`Transfer::head`]'s length: a usbmux v2 and TCP header fits.
const HEAD_BYTES: usize = 36;

impl Transfer {
    fn at(trb: u64) -> Self {
        Self {
            trb,
            done: 0,
            head: [0; HEAD_BYTES],
        }
    }
}

/// A step of a non-control endpoint's transfer, kept so a peer that
/// reports lost or misordered data can be diagnosed from the ring's view.
#[derive(Clone, Copy)]
enum Step {
    /// Start (6), Update (7) or End (8) Transfer, with Start's TRB.
    Command { ep: usize, command: u32, trb: u64 },
    /// A completed TRB: its buffer, size and status, where the core
    /// resumes after it (past owned links), and whether it raised an event.
    Completed {
        ep: usize,
        trb: u64,
        buffer: u64,
        size: u32,
        done: u32,
        ctrl: u32,
        resume: Option<u64>,
        event: bool,
    },
    /// An OUT TRB's buffer whose leading payload bytes changed while the
    /// core owned it: what the core wrote and what it read back.
    Clobbered {
        ep: usize,
        trb: u64,
        buffer: u64,
        wrote: [u8; HEAD_BYTES],
        found: [u8; HEAD_BYTES],
    },
}

#[derive(Clone, Copy)]
struct Event {
    ring: usize,
    value: u32,
}

/// Independent ordinary-domain controls. Clock/kept power pauses keep
/// configuration; quiesce inhibits DMA while MMIO remains available.
#[derive(Clone, Copy)]
struct Lifecycle {
    powered: bool,
    clocked: bool,
    quiesced: bool,
    reset: bool,
}

/// A controller DMA that failed: its I/O address, length and direction,
/// and for a payload the endpoint and the TRB, at its address, it served.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DmaFault {
    pub iova: u64,
    pub bytes: usize,
    pub write: bool,
    pub error: DmaError,
    pub payload: Option<(usize, u64, [u8; TRB_BYTES as usize])>,
}

impl DmaFault {
    fn new(iova: u64, bytes: usize, write: bool, error: DmaError) -> Self {
        Self {
            iova,
            bytes,
            write,
            error,
            payload: None,
        }
    }
}

pub struct Dwc3 {
    lifecycle: Lifecycle,
    phy_reset: bool,
    phy_ready: bool,
    phy_quiesced: bool,
    phy_clocked: bool,
    pending_phy_disconnect: bool,
    session_reset: bool,
    irqs: [AicInput; RINGS],
    rings: [Ring; RINGS],
    endpoints: [Endpoint; ENDPOINTS],
    pending: VecDeque<Event>,
    gctl: u32,
    usb2: u32,
    usb3: u32,
    tx_fifo: [u32; ENDPOINTS / 2],
    /// GTXFIFOPRIDEV: one DMA priority bit per TX FIFO. The model serves
    /// one transfer at a time, so priority orders nothing.
    tx_priority: u32,
    guid: u32,
    guctl1: u32,
    rx_fifo: u32,
    dcfg: u32,
    dctl: u32,
    devten: u32,
    enabled: u32,
    generic_parameter: u32,
    generic_command: u32,
    port_generation: u64,
    port_ready: bool,
    connected: bool,
    /// The DMA failure that poisoned the controller until software reset.
    poisoned: Option<DmaFault>,
    host: xhci::Xhci,
    apple: apple::Registers,
    history: VecDeque<Step>,
}

impl Dwc3 {
    /// IRQs are event-buffer wires, supplied by placement, never inferred from
    /// the event number. Production Apple routing still requires validation.
    pub fn new(irqs: [AicInput; RINGS]) -> Self {
        Self {
            lifecycle: Lifecycle {
                powered: true,
                clocked: true,
                quiesced: false,
                reset: false,
            },
            phy_reset: false,
            phy_ready: false,
            phy_quiesced: false,
            phy_clocked: true,
            pending_phy_disconnect: false,
            session_reset: false,
            irqs,
            rings: [Ring::new(); RINGS],
            endpoints: [Endpoint::default(); ENDPOINTS],
            pending: VecDeque::new(),
            gctl: PRTCAP_DEVICE,
            usb2: 0,
            usb3: 0,
            tx_fifo: [0; ENDPOINTS / 2],
            rx_fifo: 0,
            tx_priority: 0,
            guid: 0,
            guctl1: 0,
            dcfg: 0,
            dctl: 0,
            devten: 0,
            enabled: 0,
            generic_parameter: 0,
            generic_command: 0,
            port_generation: 0,
            port_ready: false,
            connected: false,
            poisoned: None,
            host: xhci::Xhci::new(),
            apple: apple::Registers::default(),
            history: VecDeque::with_capacity(HISTORY_STEPS),
        }
    }

    fn record(&mut self, step: Step) {
        if self.history.len() == HISTORY_STEPS {
            self.history.pop_front();
        }
        self.history.push_back(step);
    }

    /// The last non-control transfer steps, oldest first, for a peer that
    /// reports lost or misordered data.
    pub fn transfer_history(&self) -> impl Iterator<Item = String> + '_ {
        self.history.iter().map(|step| match *step {
            Step::Command { ep, command, trb } => {
                let name = match command {
                    6 => "start",
                    7 => "update",
                    _ => "end",
                };
                format!("ep={ep} {name} trb={trb:#x}")
            }
            Step::Completed {
                ep,
                trb,
                buffer,
                size,
                done,
                ctrl,
                resume,
                event,
            } => format!(
                "ep={ep} completed trb={trb:#x} buffer={buffer:#x} size={size} done={done} ctrl={ctrl:#x} event={event} resume={}",
                resume.map_or_else(|| String::from("none"), |trb| format!("{trb:#x}"))
            ),
            Step::Clobbered {
                ep,
                trb,
                buffer,
                wrote,
                found,
            } => format!(
                "ep={ep} clobbered trb={trb:#x} buffer={buffer:#x} wrote={wrote:02x?} found={found:02x?}"
            ),
        })
    }

    fn mmio_ready(&self) -> Result<()> {
        self.mmio_ready_at(None)
    }

    /// Whether the AP register at `offset` (any, for `None`) is accessible.
    /// `AppleT8112USBXDCI::powerOn` (26A434 KC 0x43096e8) read-modify-writes
    /// GUSB2PHYCFG and GUSB3PIPECTL, clearing SUSPHY, at 0x430a380 and
    /// 0x430a5a4, before it clears the pipehandler's FORCE_CLAMP (0x430a774)
    /// and sets its RESET_N (0x430a91c): those two PHY configuration words
    /// complete, and keep what is written, while the physical ATC PHY reset
    /// is held. The drivers release that reset before core initialization
    /// and establish no other register's access while it is held, which
    /// stays unchecked, not a claim that physical reset makes the rest
    /// of MMIO inaccessible.
    fn mmio_ready_at(&self, offset: Option<u64>) -> Result<()> {
        if !self.lifecycle.powered {
            return Err(refuse!("DWC3 power domain is off"));
        }
        if !self.lifecycle.clocked {
            return Err(refuse!("DWC3 clock is gated"));
        }
        if self.lifecycle.reset {
            return Err(refuse!("DWC3 reset is asserted"));
        }
        if self.phy_reset && !matches!(offset, Some(GUSB2PHYCFG | GUSB3PIPECTL)) {
            return Err(refuse!(
                "DWC3 MMIO while physical ATC PHY reset is held is unqualified"
            ));
        }
        Ok(())
    }

    fn activity_ready(&self) -> Result<()> {
        self.mmio_ready()?;
        if self.lifecycle.quiesced {
            return Err(refuse!("DWC3 DMA is quiesced"));
        }
        if !self.phy_clocked {
            return Err(refuse!("ATC PHY clock is gated"));
        }
        if self.phy_quiesced {
            return Err(refuse!("ATC PHY DMA is quiesced"));
        }
        Ok(())
    }

    pub fn lifecycle_active(&self) -> bool {
        self.activity_ready().is_ok()
    }

    fn clock_running(&self) -> bool {
        self.lifecycle.powered
            && self.lifecycle.clocked
            && !self.lifecycle.reset
            && !self.phy_reset
            && self.phy_clocked
    }

    /// Inhibit both personalities without losing their rings or configuration.
    pub fn set_clock(&mut self, enabled: bool, access: &Access<'_>) {
        let before = self.clock_running();
        self.lifecycle.clocked = enabled;
        let after = self.clock_running();
        if before != after {
            self.host.set_clock(after, access.now);
        }
    }

    /// No DMA work runs concurrently with a machine boundary. Non-kept
    /// supply loss cancels both personalities using the ordinary reset image;
    /// already committed guest RAM bytes are never undone or completed again.
    pub fn set_power(&mut self, powered: bool, retention: bool, access: &mut Access<'_>) {
        let before = self.clock_running();
        if self.lifecycle.powered && !powered && !retention {
            self.reset(access);
        }
        self.lifecycle.powered = powered;
        let after = self.clock_running();
        if before != after {
            self.host.set_clock(after, access.now);
        }
    }

    pub fn set_quiesced(&mut self, quiesced: bool) {
        self.lifecycle.quiesced = quiesced;
    }

    pub fn set_reset(&mut self, asserted: bool, access: &mut Access<'_>) {
        let before = self.clock_running();
        let reset_before = self.lifecycle.reset || self.phy_reset;
        self.lifecycle.reset = asserted;
        if !reset_before && (self.lifecycle.reset || self.phy_reset) {
            self.reset(access);
        }
        let after = self.clock_running();
        if before != after {
            self.host.set_clock(after, access.now);
        }
    }

    /// Independent physical outputs from the PHY. Reset sources are
    /// ORed; neither a PMGR release nor a PHY release can clear the other.
    pub fn set_phy_state(
        &mut self,
        reset: bool,
        ready: bool,
        clocked: bool,
        quiesced: bool,
        access: &mut Access<'_>,
    ) -> bool {
        let changed = (
            self.phy_reset,
            self.phy_ready,
            self.phy_clocked,
            self.phy_quiesced,
        ) != (reset, ready, clocked, quiesced);
        let before = self.clock_running();
        let reset_before = self.lifecycle.reset || self.phy_reset;
        if self.phy_ready && !ready {
            // Keep an electrical falling edge until service, even if software
            // immediately restores the PHY before the next DMA boundary.
            self.pending_phy_disconnect = true;
            self.session_reset = true;
        }
        self.phy_reset = reset;
        self.phy_ready = ready;
        self.phy_clocked = clocked;
        self.phy_quiesced = quiesced;
        if !reset_before && (self.lifecycle.reset || self.phy_reset) {
            self.reset(access);
        }
        let after = self.clock_running();
        if before != after {
            self.host.set_clock(after, access.now);
        }
        changed
    }

    pub fn phy_ready(&self) -> bool {
        self.phy_ready
    }

    /// The adapter cancels its external peer session before further service.
    /// It keeps its own retry flag if that cancellation reports failure.
    pub fn take_session_reset(&mut self) -> bool {
        core::mem::take(&mut self.session_reset)
    }

    /// Whether the adapter's next service has work whatever the time: a
    /// session to cancel, a PHY disconnection to report or device events
    /// to post.
    pub fn service_due(&self) -> bool {
        self.session_reset || self.pending_phy_disconnect || !self.pending.is_empty()
    }

    fn error(offset: u64, direction: Direction, reason: impl Into<Refusal>) -> MmioError {
        refuse(NAME, offset, Width::Word, direction, reason)
    }

    fn queue(&mut self, ring: usize, value: u32) -> Result<()> {
        if self.pending.len() == MAX_PENDING_EVENTS {
            return Err(refuse!("DWC3 pending event capacity exhausted"));
        }
        self.pending.push_back(Event { ring, value });
        Ok(())
    }

    fn device_event(&mut self, kind: u32) -> Result<()> {
        if self.devten & (1 << kind) != 0 {
            self.queue(0, device_event(kind))?;
        }
        Ok(())
    }

    /// A core soft reset or a switch away from device mode: the device
    /// controller loses its transfers, transfer resources and queued events.
    fn release(&mut self) {
        for ep in &mut self.endpoints {
            ep.active = None;
            ep.parked = false;
            ep.resource = false;
            ep.stalled = false;
            ep.not_ready = false;
        }
        self.pending.clear();
        self.session_reset = true;
        self.dcfg &= !(0x7f << 3);
        self.connected = false;
    }

    /// A bus event (USB reset, detach, PHY reset, Run/Stop clear) leaves
    /// endpoint state to software: transfer resources stay assigned until
    /// Start Configuration, started transfers until End Transfer, and the
    /// driver itself zeroes DCFG.DevAddr and clears halts on USB reset
    /// (DWC3 databook "Initialization on USB Reset"; Linux
    /// `dwc3_gadget_reset_interrupt`). Only XferNotReady reporting rearms.
    fn bus_event(&mut self) {
        for ep in &mut self.endpoints {
            ep.not_ready = false;
        }
    }

    /// Whether [`Self::sync_port`] with `state` would change nothing: no
    /// PHY disconnection to report and the port's generation and readiness
    /// already `state`'s.
    pub fn port_synced(&self, state: &impl PortState) -> bool {
        !self.pending_phy_disconnect
            && self.port_generation == state.generation()
            && self.port_ready == state.high_speed_device_ready()
    }

    pub fn sync_port(&mut self, state: &impl PortState) -> Result<()> {
        self.activity_ready()?;
        if self.pending_phy_disconnect {
            if self.connected {
                self.device_event(0)?;
                self.bus_event();
                self.connected = false;
            }
            self.host.attach(None);
            self.pending_phy_disconnect = false;
        }
        let ready = state.high_speed_device_ready();
        if self.port_generation != state.generation() || self.port_ready != ready {
            if self.connected {
                self.device_event(0)?;
            }
            self.bus_event();
            self.connected = false;
            self.port_generation = state.generation();
            self.port_ready = ready;
        }
        Ok(())
    }

    /// The DMA failure that poisoned the controller, if one did.
    pub fn dma_fault(&self) -> Option<DmaFault> {
        self.poisoned.clone()
    }

    /// Publish bounded queued events via this device's DART stream, before
    /// asserting a wire. Full rings backpressure without overwriting entries.
    /// An uncertain DMA failure poisons the controller until software reset;
    /// no payload or completion is retried behind the owner's back.
    pub fn flush(&mut self, port: &mut Port<'_, '_>) -> Result<()> {
        self.activity_ready()?;
        if self.poisoned.is_some() {
            return Err(refuse!("DWC3 requires reset after a DMA failure"));
        }
        for _ in 0..self.pending.len() {
            let Some(event) = self.pending.front().copied() else {
                break;
            };
            let ring = &mut self.rings[event.ring];
            if ring.size == 0 || ring.count + EVENT_BYTES > ring.size {
                let blocked = self.pending.pop_front().unwrap();
                self.pending.push_back(blocked);
                continue;
            }
            let address = ring
                .address
                .checked_add(u64::from(ring.producer))
                .ok_or(refuse!("DWC3 event address overflow"))?;
            if let Err(error) = port.write(address, &event.value.to_le_bytes()) {
                self.poisoned = Some(DmaFault::new(
                    address,
                    event.value.to_le_bytes().len(),
                    true,
                    error,
                ));
                return Err(refuse!("DWC3 event DMA fault; reset required"));
            }
            ring.producer = (ring.producer + EVENT_BYTES) % ring.size;
            ring.count += EVENT_BYTES;
            self.pending.pop_front();
        }
        self.interrupts(port.access);
        Ok(())
    }

    /// Host personality work (a doorbell, events to post) waits for DMA.
    pub fn host_pending(&self) -> bool {
        self.lifecycle_active() && self.host.pending()
    }

    /// The system count at which the host personality next has work of
    /// its own: an event to post, or `function`'s endpoint to poll.
    pub fn host_deadline(&self, function: Option<&(dyn Function + '_)>) -> Option<u64> {
        if self.lifecycle_active() {
            self.host.deadline(function)
        } else {
            None
        }
    }

    /// The USB2 function [`Self::attach_host_function`] last cabled.
    pub fn host_function(&self) -> Option<Usb2Speed> {
        self.host.attached()
    }

    /// Cables a USB2 function of `speed` to the host personality's root
    /// port, or nothing.
    pub fn attach_host_function(&mut self, speed: Option<Usb2Speed>) -> Result<()> {
        self.activity_ready()?;
        self.host.attach(speed);
        Ok(())
    }

    /// The host personality's next trace line, for the boot log.
    pub fn take_host_trace(&mut self) -> Option<String> {
        self.host.take_trace()
    }

    /// The host personality's registers and event rings, while it runs or
    /// holds events.
    pub fn host_summary(&self) -> Option<String> {
        let wires: [bool; RINGS] = core::array::from_fn(|n| self.rings[n].asserted);
        self.host
            .summary()
            .map(|summary| alloc::format!("{summary} wires={wires:?}"))
    }

    /// Runs the host personality's commands and transfers, `function`
    /// being what is cabled to its port, and posts its events.
    pub fn service_host(
        &mut self,
        port: &mut Port<'_, '_>,
        function: Option<&mut (dyn Function + '_)>,
    ) -> Result<()> {
        self.activity_ready()?;
        if self.poisoned.is_some() {
            return Err(refuse!("DWC3 requires reset after a DMA failure"));
        }
        self.host.service(port, function)?;
        self.interrupts(port.access);
        Ok(())
    }

    fn interrupts(&mut self, access: &mut Access<'_>) {
        for (n, (ring, irq)) in self.rings.iter_mut().zip(self.irqs).enumerate() {
            let level = ring.count != 0 && !ring.masked || self.host.interrupt(n);
            if ring.asserted != level {
                access.interrupts.set_level(Wire::Aic(irq), level);
                ring.asserted = level;
            }
        }
    }

    fn reset(&mut self, access: &mut Access<'_>) {
        for irq in self.irqs {
            access.interrupts.set_level(Wire::Aic(irq), false);
        }
        let mut host = core::mem::take(&mut self.host);
        host.reset("DWC3 core reset");
        let generation = self.port_generation;
        let ready = self.port_ready;
        let lifecycle = self.lifecycle;
        let phy = (
            self.phy_reset,
            self.phy_ready,
            self.phy_clocked,
            self.phy_quiesced,
        );
        *self = Self::new(self.irqs);
        self.port_generation = generation;
        self.port_ready = ready;
        self.lifecycle = lifecycle;
        (
            self.phy_reset,
            self.phy_ready,
            self.phy_clocked,
            self.phy_quiesced,
        ) = phy;
        self.host = host;
        self.session_reset = true;
    }

    fn endpoint_command(&mut self, n: usize, value: u32) -> Result<()> {
        let command = value & 0xf;
        if !matches!(command, 1 | 2 | 4..=9) {
            return Err(refuse!("unmodeled DWC3 endpoint command"));
        }
        if value & CMDACT == 0 || value & !(0x7f_0000 | 0x800 | CMDACT | CMDIOC | 0xf) != 0 {
            return Err(refuse!(
                "DWC3 endpoint command reserved bits or CMDACT clear"
            ));
        }
        if command != 8 && value & (1 << 11) != 0 {
            return Err(refuse!(
                "DWC3 endpoint priority or clear-pending request is not modeled"
            ));
        }
        if command == 8 && value & (1 << 11) == 0 {
            return Err(refuse!("DWC3 End Transfer requires ForceRM"));
        }
        if value & CMDIOC != 0 && self.pending.len() == MAX_PENDING_EVENTS {
            return Err(refuse!("DWC3 pending command event capacity exhausted"));
        }
        let ep = &mut self.endpoints[n];
        let [p2, p1, p0] = ep.parameters;
        let mut failed = false;
        let mut result = value & !(CMDACT | 0x7f_0000);
        match command {
            1 => {
                let kind = (p0 >> 1) & 3;
                let packet = ((p0 >> 3) & 0x7ff) as usize;
                let ring = (p1 & 0x1f) as usize;
                // No stream/isochronous state can be discarded. An
                // interrupt endpoint's bInterval-1 (16..23) sets its
                // service interval, which only the host's polling observes.
                let interval = if kind == INTERRUPT { 0xff << 16 } else { 0 };
                let burst = if kind == 2 { BURST } else { 0 };
                if p2 != 0
                    || p0 & !((3 << 1) | (0x7ff << 3) | (0x1f << 17) | burst | (3 << 30)) != 0
                    || p0 >> 30 == 1
                    || p0 >> 30 == 3
                    || p1
                        & !((0x1f << 25)
                            | CFG_COMPLETE
                            | CFG_PROGRESS
                            | CFG_NOT_READY
                            | 0x1f
                            | interval)
                        != 0
                {
                    return Err(refuse!(
                        "DWC3 endpoint streams, bursts or reserved configuration"
                    ));
                }
                if !matches!(kind, 0 | 2 | INTERRUPT) || (kind == 0) != (n < 2) {
                    return Err(refuse!(
                        "DWC3 models control EP0, bulk and interrupt endpoints only"
                    ));
                }
                if packet == 0
                    || packet
                        > if n < 2 {
                            MAX_PACKET_CONTROL
                        } else {
                            MAX_PACKET
                        }
                    || ring >= RINGS
                    || (p1 >> 25) & 0x1f != n as u32
                    // Connect Done modifies EP0 with its SETUP transfer
                    // started (Linux `dwc3_gadget_conndone_interrupt`);
                    // only initialization requires an idle endpoint.
                    || (ep.active.is_some() && p0 >> 30 != 2)
                {
                    failed = true;
                } else {
                    ep.configured = true;
                    ep.ring = ring;
                    ep.packet = packet;
                    ep.events = p1 & (CFG_COMPLETE | CFG_PROGRESS | CFG_NOT_READY);
                }
            }
            2 => {
                if p0 != 1 || p1 != 0 || p2 != 0 {
                    return Err(refuse!("DWC3 requires one transfer resource per endpoint"));
                }
                if ep.configured && ep.active.is_none() {
                    ep.resource = true;
                } else {
                    failed = true;
                }
            }
            4 | 5 => {
                if n < 2 && (n != 0 || command == 5) {
                    return Err(refuse!(
                        "DWC3 control stall is OUT-only and clears on SETUP"
                    ));
                }
                if !ep.configured {
                    failed = true;
                } else {
                    ep.stalled = command == 4;
                }
            }
            6 => {
                if value & (0x7f << 16 | 1 << 11) != 0 {
                    return Err(refuse!(
                        "DWC3 non-stream start requires zero command parameter/priority"
                    ));
                }
                if p2 != 0 {
                    return Err(refuse!("DWC3 transfer streams are not modeled"));
                }
                let address = u64::from(p1) | u64::from(p0) << 32;
                if !ep.resource || ep.active.is_some() || !address.is_multiple_of(TRB_BYTES) {
                    failed = true;
                } else {
                    ep.active = Some(Transfer::at(address));
                    ep.not_ready = false;
                    ep.parked = false;
                    result |= 1 << 16;
                }
            }
            7 | 8 => {
                if (value >> 16) & 0x7f != 1 || ep.active.is_none() {
                    failed = true;
                } else if command == 8 {
                    ep.active = None;
                    ep.not_ready = false;
                    ep.parked = false;
                } else {
                    result |= 1 << 16;
                    ep.not_ready = false;
                    ep.parked = false;
                }
            }
            9 => {
                let first = ((value >> 16) & 0x7f) as usize;
                if n != 0 || !matches!(first, 0 | 2) {
                    failed = true;
                } else if first == 0 && value & CMDIOC != 0 {
                    return Err(refuse!("DWC3 initial Start Configuration requires polling"));
                } else if self.endpoints[first..].iter().any(|ep| ep.active.is_some()) {
                    failed = true;
                } else {
                    for ep in &mut self.endpoints[first..] {
                        ep.resource = false;
                    }
                }
            }
            _ => unreachable!(),
        }
        if failed {
            result |= CMD_FAILURE;
        } else if n >= 2 && matches!(command, 6..=8) {
            let trb = if command == 6 {
                u64::from(p1) | u64::from(p0) << 32
            } else {
                0
            };
            self.record(Step::Command {
                ep: n,
                command,
                trb,
            });
        }
        self.endpoints[n].command = result;
        self.endpoints[n].not_ready = false;
        if value & CMDIOC != 0 {
            self.queue(
                self.endpoints[n].ring,
                ep_event(
                    n,
                    CMD_COMPLETE,
                    u32::from(failed),
                    command << 8 | ((result >> 16) & 0x7f),
                ),
            )?;
        }
        Ok(())
    }

    pub fn available(&self) -> bool {
        self.unavailable().is_none()
    }

    /// Capture both control endpoints and their interrupt rings at a host timeout.
    pub(crate) fn control_timeout(&self, port: u32, cause: Refusal) -> Refusal {
        let mut report = refuse!(
            "DWC3 control state",
            port,
            address = (self.dcfg >> 3) & 0x7f,
            enabled = self.enabled & 3,
            pending_events = self.pending.len()
        );
        for (name, ep) in ["ep0_out", "ep0_in"].into_iter().zip(&self.endpoints[..2]) {
            let ring = &self.rings[ep.ring];
            report = report.display(
                name,
                &format_args!(
                    "configured={} trb={:x?} parked={} events={:#x} ring={} count={} masked={} irq={}",
                    ep.configured,
                    ep.active.map(|transfer| transfer.trb),
                    ep.parked,
                    ep.events,
                    ep.ring,
                    ring.count,
                    ring.masked,
                    ring.asserted
                ),
            );
        }
        report.because(cause)
    }

    /// Why a host cannot reach the device personality now, if it cannot:
    /// the first unmet condition of [`Self::available`].
    pub fn unavailable(&self) -> Option<Refusal> {
        if let Err(why) = self.activity_ready() {
            return Some(why);
        }
        Some(Refusal::new(if !self.phy_ready {
            "ATC PHY USB2 is not ready"
        } else if self.pending_phy_disconnect {
            "a PHY disconnect is pending service"
        } else if !self.port_ready {
            "the Type-C port is not ready"
        } else if self.dctl & RUN == 0 {
            "DCTL.RUN is clear"
        } else if self.gctl & (CORE_RESET | PRTCAP) != PRTCAP_DEVICE {
            "GCTL is not device mode out of core reset"
        } else if self.usb2 & (1 << 31) != 0 {
            "GUSB2PHYCFG.PHYSOFTRST is set"
        } else {
            return None;
        }))
    }

    /// A transaction adapter whose borrow keeps the DMA stream.
    pub fn peer<'a, 'b, 'c>(&'a mut self, dma: &'a mut Port<'b, 'c>) -> Peer<'a, 'b, 'c> {
        Peer {
            controller: self,
            dma,
        }
    }

    fn not_ready(&mut self, n: usize, token: Token, bytes: usize, active: bool) -> Result<Reply> {
        let ep = self.endpoints[n];
        if !ep.not_ready
            && ep.events & CFG_NOT_READY != 0
            && self.pending.len() < MAX_PENDING_EVENTS
        {
            let phase = if n >= 2 || token == Token::Setup {
                0
            } else if bytes == 0 {
                2
            } else {
                1
            };
            self.queue(
                ep.ring,
                ep_event(n, XFER_NOT_READY, phase | if active { 8 } else { 0 }, 0),
            )?;
            self.endpoints[n].not_ready = true;
        }
        Ok(Reply::Nak)
    }

    /// The TRB at `address`, or past the owned link TRBs it leads to.
    fn fetch(
        &mut self,
        port: &mut Port<'_, '_>,
        mut address: u64,
    ) -> Result<(u64, [u8; TRB_BYTES as usize])> {
        let mut trb = [0u8; TRB_BYTES as usize];
        for _ in 0..=MAX_LINKS_PER_PACKET {
            if let Err(error) = port.read(address, &mut trb) {
                self.poisoned = Some(DmaFault::new(address, trb.len(), false, error));
                return Err(refuse!("DWC3 TRB DMA fault; reset required"));
            }
            let ctrl = u32::from_le_bytes(trb[12..16].try_into().unwrap());
            if ctrl & HWO == 0 || (ctrl >> 4) & 0x3f != 8 {
                return Ok((address, trb));
            }
            if ctrl & !(HWO | (0x3f << 4)) != 0 || trb[8..12] != [0; 4] {
                return Err(refuse!("DWC3 link TRB has unmodeled flags or size"));
            }
            address = u64::from_le_bytes(trb[..8].try_into().unwrap());
            if !address.is_multiple_of(TRB_BYTES) {
                return Err(refuse!("DWC3 unaligned link TRB"));
            }
        }
        Err(refuse!("DWC3 cyclic or excessive TRB links"))
    }

    fn packet(
        &mut self,
        n: usize,
        token: Token,
        data: &mut [u8],
        port: &mut Port<'_, '_>,
    ) -> Result<Reply> {
        self.activity_ready()?;
        if self.poisoned.is_some() {
            return Err(refuse!("DWC3 requires reset after a DMA failure"));
        }
        if !self.connected || !self.available() {
            return Ok(Reply::Nak);
        }
        if n >= ENDPOINTS || self.enabled & (1 << n) == 0 {
            return Ok(Reply::Nak);
        }
        let mut ep = self.endpoints[n];
        if !ep.configured {
            return Ok(Reply::Nak);
        }
        if ep.stalled && token != Token::Setup {
            return Ok(Reply::Stall);
        }
        let Some(mut transfer) = ep.active else {
            return self.not_ready(n, token, data.len(), false);
        };
        if ep.parked {
            return Ok(Reply::Nak);
        }
        if token == Token::Setup && self.endpoints[1].active.is_some() {
            return Err(refuse!("DWC3 superseding control SETUP is not modeled"));
        }
        if self.pending.len() == MAX_PENDING_EVENTS {
            return Ok(Reply::Nak);
        }
        let (address, trb) = self.fetch(port, transfer.trb)?;
        transfer.trb = address;
        if u32::from_le_bytes(trb[12..16].try_into().unwrap()) & HWO == 0 {
            self.endpoints[n].active = Some(transfer);
            self.endpoints[n].parked = true;
            return self.not_ready(n, token, data.len(), true);
        }
        let buffer = u64::from_le_bytes(trb[..8].try_into().unwrap());
        let size = u32::from_le_bytes(trb[8..12].try_into().unwrap());
        let ctrl = u32::from_le_bytes(trb[12..16].try_into().unwrap());
        let kind = (ctrl >> 4) & 0x3f;
        if ctrl & !(HWO | LST | CHN | CSP | ISP | IOC | (0x3f << 4)) != 0 || size & !LENGTH != 0 {
            return Err(refuse!("DWC3 unmodeled TRB flags, streams or status"));
        }
        if n < 2 {
            if !matches!(kind, 2..=5)
                || (token == Token::Setup) != (kind == 2)
                || (kind == 2 && (n != 0 || data.len() != 8 || size != 8))
                || (matches!(kind, 3 | 4) && !data.is_empty())
            {
                return Err(refuse!("DWC3 control token/TRB stage mismatch"));
            }
        } else if kind != 1 {
            return Err(refuse!("DWC3 bulk requires a normal TRB"));
        }
        let remaining = (size & LENGTH)
            .checked_sub(transfer.done)
            .ok_or(refuse!("DWC3 changed active TRB size"))? as usize;
        let packet = if token == Token::Setup { 8 } else { ep.packet };
        if data.len() > packet {
            return Err(refuse!("DWC3 transaction exceeds endpoint max packet"));
        }
        let bytes = if token == Token::In {
            remaining.min(data.len())
        } else {
            data.len()
        };
        if bytes > remaining {
            return Err(refuse!("DWC3 OUT packet exceeds TRB capacity"));
        }
        let address = buffer
            .checked_add(u64::from(transfer.done))
            .ok_or(refuse!("DWC3 payload address overflow"))?;
        let short_packet = token == Token::Out && !matches!(kind, 3 | 4) && bytes < packet;
        if short_packet && ctrl & (CSP | CHN) == CSP | CHN {
            return Err(refuse!(
                "DWC3 short-packet chain fast-forward is not modeled"
            ));
        }
        let next_trb = transfer
            .trb
            .checked_add(TRB_BYTES)
            .ok_or(refuse!("DWC3 TRB list address overflow"))?;
        let result = if token == Token::In {
            port.read(address, &mut data[..bytes])
        } else {
            port.write(address, &data[..bytes])
        };
        if let Err(error) = result {
            let write = token != Token::In;
            self.poisoned = Some(DmaFault {
                payload: Some((n, transfer.trb, trb)),
                ..DmaFault::new(address, bytes, write, error)
            });
            return Err(refuse!("DWC3 payload DMA fault; reset required"));
        }
        let start = transfer.done as usize;
        if token == Token::Out && n >= 2 && start < HEAD_BYTES {
            let end = (start + bytes).min(HEAD_BYTES);
            transfer.head[start..end].copy_from_slice(&data[..end - start]);
        }
        transfer.done += bytes as u32;
        let short = short_packet;
        let complete = transfer.done == size & LENGTH || short;
        if complete {
            // Software must leave a buffer alone while the core owns its
            // TRB; one it changed meanwhile is recorded for diagnosis.
            let kept = (transfer.done as usize).min(HEAD_BYTES);
            let mut found = [0; HEAD_BYTES];
            if token == Token::Out
                && n >= 2
                && kept != 0
                && port.read(buffer, &mut found[..kept]).is_ok()
                && found[..kept] != transfer.head[..kept]
            {
                self.record(Step::Clobbered {
                    ep: n,
                    trb: transfer.trb,
                    buffer,
                    wrote: transfer.head,
                    found,
                });
            }
            let residual = (size & LENGTH) - transfer.done;
            let mut completion = [0; 8];
            completion[..4].copy_from_slice(&residual.to_le_bytes());
            completion[4..].copy_from_slice(&(ctrl & !HWO).to_le_bytes());
            if let Err(error) = port.write(transfer.trb + 8, &completion) {
                self.poisoned = Some(DmaFault::new(
                    transfer.trb + 8,
                    completion.len(),
                    true,
                    error,
                ));
                return Err(refuse!("DWC3 TRB completion DMA fault; reset required"));
            }
            let last = ctrl & LST != 0 || (short && ctrl & CSP == 0);
            let event_kind = if last { XFER_COMPLETE } else { XFER_PROGRESS };
            let event_enable = if last { CFG_COMPLETE } else { CFG_PROGRESS };
            let event = ep.events & event_enable != 0
                && (last || ctrl & IOC != 0 || (short && ctrl & ISP != 0));
            if event {
                let status = if short { EV_SHORT } else { 0 }
                    | if ctrl & IOC != 0 { EV_IOC } else { 0 }
                    | if ctrl & LST != 0 { EV_LST } else { 0 };
                self.queue(ep.ring, ep_event(n, event_kind, status, 0))?;
            }
            // The core prefetches past a completed TRB, so a link is
            // followed while its block still belongs to this ring: Apple's
            // driver returns a finished block to a pool shared by all
            // endpoints as soon as its event arrives.
            ep.active = if last {
                None
            } else {
                Some(Transfer::at(self.fetch(port, next_trb)?.0))
            };
            if n >= 2 {
                self.record(Step::Completed {
                    ep: n,
                    trb: transfer.trb,
                    buffer,
                    size: size & LENGTH,
                    done: transfer.done,
                    ctrl,
                    resume: ep.active.map(|transfer| transfer.trb),
                    event,
                });
            }
        } else {
            ep.active = Some(transfer);
        }
        if token == Token::Setup {
            ep.stalled = false;
            self.endpoints[1].stalled = false;
        }
        ep.not_ready = false;
        self.endpoints[n] = ep;
        Ok(Reply::Ack(bytes))
    }
}

pub struct Peer<'a, 'b, 'c> {
    controller: &'a mut Dwc3,
    dma: &'a mut Port<'b, 'c>,
}
impl BusPeer for Peer<'_, '_, '_> {
    fn reset(&mut self) -> Result<()> {
        let controller = &mut self.controller;
        if !controller.available() {
            return Err(refuse!(
                "DWC3 port or controller is not ready for USB2 reset"
            ));
        }
        controller.bus_event();
        controller.connected = true;
        controller.device_event(1)?;
        controller.device_event(2)?;
        Ok(())
    }
    fn packet(&mut self, endpoint: u8, token: Token, data: &mut [u8]) -> Result<Reply> {
        if endpoint & 0x70 != 0 || (token == Token::In) != (endpoint & 0x80 != 0) {
            return Err(refuse!(
                "invalid USB endpoint direction or reserved address bits"
            ));
        }
        let n = usize::from(endpoint & 0xf) * 2 + usize::from(token == Token::In);
        self.controller.packet(n, token, data, self.dma)
    }
}

impl MmioDevice for Dwc3 {
    fn name(&self) -> &'static str {
        NAME
    }
    fn read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError> {
        self.mmio_ready_at(Some(offset))
            .map_err(|why| refuse(NAME, offset, width, Direction::Read, why))?;
        if offset < xhci::BYTES {
            return self
                .host
                .read(offset, width.bytes(), access.now)
                .map_err(|why| refuse(NAME, offset, width, Direction::Read, why));
        }
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(refuse(
                NAME,
                offset,
                width,
                Direction::Read,
                "DWC3 registers require aligned 32-bit accesses",
            ));
        }
        let value = match offset {
            GUCTL1 => self.guctl1,
            GUID => self.guid,
            GTXFIFOPRIDEV => self.tx_priority,
            GCTL => self.gctl,
            GSNPSID => 0x5533_300a,
            GHWPARAMS => (64 << 24) | (32 << 16) | (32 << 8),
            0xc144 => (RINGS as u32) << 15,
            0xc148 | 0xc150 | 0xc154 | 0xc158 => 0,
            0xc14c => (16 << 18) | (32 << 12) | (1 << 2),
            0xc15c => FIFO_WORDS | FIFO_WORDS << 16,
            GUSB2PHYCFG => self.usb2,
            GUSB3PIPECTL => self.usb3,
            GRXFIFOSIZ => self.rx_fifo,
            DCFG => self.dcfg,
            DCTL => self.dctl,
            DEVTEN => self.devten,
            DSTS => {
                RX_EMPTY
                    | if self.dctl & RUN == 0 { HALTED } else { 0 }
                    | if self.connected { 0 } else { 5 << 18 }
            }
            DGCMDPAR => self.generic_parameter,
            DGCMD => self.generic_command,
            DALEPENA => self.enabled,
            at if (GTXFIFOSIZ..GTXFIFOSIZ + 4 * (ENDPOINTS as u64 / 2)).contains(&at) => {
                self.tx_fifo[((at - GTXFIFOSIZ) / 4) as usize]
            }
            at if (GEVNT..GEVNT + STRIDE * RINGS as u64).contains(&at) => {
                let ring = self.rings[((at - GEVNT) / STRIDE) as usize];
                match (at - GEVNT) % STRIDE {
                    0 => ring.address as u32,
                    4 => (ring.address >> 32) as u32,
                    8 => ring.size | if ring.masked { INTMASK } else { 0 },
                    12 => ring.count,
                    _ => unreachable!(),
                }
            }
            at if (DEP..DEP + STRIDE * ENDPOINTS as u64).contains(&at) => {
                let ep = self.endpoints[((at - DEP) / STRIDE) as usize];
                match (at - DEP) % STRIDE {
                    12 => ep.command,
                    field => ep.parameters[(field / 4) as usize],
                }
            }
            at if apple::Registers::claims(at) => self
                .apple
                .read(at)
                .map_err(|why| Self::error(offset, Direction::Read, why))?,
            _ => {
                return Err(Self::error(
                    offset,
                    Direction::Read,
                    "unmodeled DWC3/Apple register",
                ));
            }
        };
        Ok(u64::from(value))
    }
    fn write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        self.mmio_ready_at(Some(offset))
            .map_err(|why| refuse(NAME, offset, width, Direction::Write, why))?;
        if offset < xhci::BYTES {
            let result = self.host.write(offset, width.bytes(), value, access.now);
            // This register bank has its own clock. Software RUN/STOP while
            // the PHY clock is off cannot restart the external bus clock.
            if !self.clock_running() {
                self.host.set_clock(false, access.now);
            }
            result.map_err(|why| refuse(NAME, offset, width, Direction::Write, why))?;
            self.interrupts(access);
            return Ok(());
        }
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(refuse(
                NAME,
                offset,
                width,
                Direction::Write,
                "DWC3 registers require aligned 32-bit accesses",
            ));
        }
        let value = value as u32;
        let err = |why: Refusal| Self::error(offset, Direction::Write, why);
        match offset {
            GCTL => {
                // GBLHIBERNATIONEN (bit 1), which AppleT8112USBXHCI::reset
                // sets, has no effect on a core built without hibernation
                // (GHWPARAMS1.EN_PWROPT): it is held only.
                const GCTL_CONFIG: u32 =
                    0xfff8_0000 | (1 << 16) | PRTCAP | CORE_RESET | (1 << 10) | (3 << 6) | 0x1f;
                if value & !GCTL_CONFIG != 0
                    || !matches!(value & PRTCAP, PRTCAP_HOST | PRTCAP_DEVICE)
                    || value & (3 << 6) != 0
                    || value & (3 << 4) != 0
                {
                    return Err(err(refuse!(
                        "DWC3 requires host or device mode, bus clock and documented configuration"
                    )));
                }
                if value & CORE_RESET != 0 || value & PRTCAP != PRTCAP_DEVICE {
                    self.release();
                }
                if value & CORE_RESET != 0 {
                    self.host.reset("GCTL core soft reset");
                    self.interrupts(access);
                }
                self.gctl = value;
            }
            GUSB2PHYCFG => {
                const USB2_CONFIG: u32 =
                    (3 << 30) | (1 << 17) | (0xf << 10) | (3 << 8) | (1 << 6) | (1 << 3);
                if value & !USB2_CONFIG != 0 {
                    return Err(err(refuse!(
                        "DWC3 requires documented UTMI PHY configuration"
                    )));
                }
                if value & (1 << 31) != 0 {
                    self.bus_event();
                    self.connected = false;
                }
                self.usb2 = value;
            }
            GUSB3PIPECTL => {
                if value & !(1 << 31 | 1 << 17) != 0 {
                    return Err(err(refuse!("DWC3 has no SuperSpeed PHY")));
                }
                self.usb3 = value;
            }
            DCFG => {
                if value & !DCFG_FIELDS != 0 || !matches!(value & 7, 0 | 4 | 5) {
                    return Err(err(refuse!(
                        "DWC3 connects at high speed only, without streams"
                    )));
                }
                self.dcfg = value;
            }
            DCTL => {
                if value & !DCTL_FIELDS != 0 {
                    return Err(err(refuse!(
                        "DWC3 link state change, test or save/restore request is not modeled"
                    )));
                }
                if value & RESET != 0 {
                    self.reset(access);
                } else {
                    if value & RUN == 0 {
                        self.bus_event();
                        self.connected = false;
                    }
                    self.dctl = value;
                }
            }
            DEVTEN => {
                if value & !DEVTEN_FIELDS != 0 {
                    return Err(err(refuse!("DWC3 unmodeled device event enable")));
                }
                self.devten = value;
            }
            DALEPENA => self.enabled = value,
            DGCMDPAR => self.generic_parameter = value,
            DGCMD => {
                if value & CMDACT == 0 || value & !(CMDACT | CMDIOC | 0xff) != 0 {
                    return Err(err(refuse!("invalid DWC3 generic command encoding")));
                }
                if value & 0xff != 0x0a {
                    return Err(err(refuse!("only DWC3 all-FIFO flush is modeled")));
                }
                if self.endpoints.iter().any(|ep| ep.active.is_some()) {
                    return Err(err(refuse!(
                        "DWC3 active FIFO flush requires transfer cancellation"
                    )));
                }
                if value & CMDIOC != 0 && self.devten & (1 << 10) != 0 {
                    self.queue(0, device_event(10)).map_err(err)?;
                }
                // DGCMD command type and IOC are write-only (SPRUHJ7A table 3-120).
                self.generic_command = 0;
            }
            GRXFIFOSIZ => {
                if value > FIFO_WORDS {
                    return Err(err(refuse!("DWC3 RX FIFO exceeds declared RAM")));
                }
                self.rx_fifo = value;
            }
            at if (GTXFIFOSIZ..GTXFIFOSIZ + 4 * (ENDPOINTS as u64 / 2)).contains(&at) => {
                if (value >> 16) + (value & 0xffff) > FIFO_WORDS {
                    return Err(err(refuse!("DWC3 TX FIFO exceeds declared RAM")));
                }
                self.tx_fifo[((at - GTXFIFOSIZ) / 4) as usize] = value;
            }
            at if (GEVNT..GEVNT + STRIDE * RINGS as u64).contains(&at) => {
                let index = ((at - GEVNT) / STRIDE) as usize;
                let ring = &mut self.rings[index];
                match (at - GEVNT) % STRIDE {
                    field @ (0 | 4) => {
                        if ring.count != 0 || self.pending.iter().any(|event| event.ring == index) {
                            return Err(err(refuse!(
                                "cannot relocate a DWC3 ring with pending events"
                            )));
                        }
                        if field == 0 {
                            if value & 3 != 0 {
                                return Err(err(refuse!("DWC3 event ring must be word aligned")));
                            }
                            ring.address = ring.address & !0xffff_ffff | u64::from(value);
                        } else {
                            ring.address = ring.address & 0xffff_ffff | u64::from(value) << 32;
                        }
                        ring.producer = 0;
                    }
                    8 => {
                        if value & !(INTMASK | 0xfffc) != 0 {
                            return Err(err(refuse!(
                                "DWC3 event size must be a multiple of four; reserved bits"
                            )));
                        }
                        let size = value & 0xfffc;
                        if size != ring.size
                            && (ring.count != 0
                                || self.pending.iter().any(|event| event.ring == index))
                        {
                            return Err(err(refuse!(
                                "cannot resize a DWC3 ring with pending events"
                            )));
                        }
                        if size != ring.size {
                            ring.producer = 0;
                        }
                        ring.size = size;
                        ring.masked = value & INTMASK != 0;
                    }
                    12 => {
                        if value & !0xfffc != 0 || value > ring.count {
                            return Err(err(refuse!(
                                "DWC3 event acknowledgement exceeds pending byte count or is unaligned"
                            )));
                        }
                        ring.count -= value;
                    }
                    _ => unreachable!(),
                }
                self.interrupts(access);
            }
            at if (DEP..DEP + STRIDE * ENDPOINTS as u64).contains(&at) => {
                let index = ((at - DEP) / STRIDE) as usize;
                match (at - DEP) % STRIDE {
                    12 => self.endpoint_command(index, value).map_err(err)?,
                    field => self.endpoints[index].parameters[(field / 4) as usize] = value,
                }
            }
            GUCTL1 => {
                if value & !GUCTL1_FIELDS != 0 {
                    return Err(err(refuse!("unmodeled DWC3 GUCTL1 fields")));
                }
                self.guctl1 = value;
            }
            // A software-defined identification word.
            GUID => self.guid = value,
            GTXFIFOPRIDEV => {
                if value >> (ENDPOINTS / 2) != 0 {
                    return Err(err(refuse!(
                        "DWC3 TX FIFO priority names a FIFO the core lacks"
                    )));
                }
                self.tx_priority = value;
            }
            at if apple::Registers::claims(at) => self.apple.write(at, value).map_err(err)?,
            _ => return Err(err(refuse!("read-only or unmodeled DWC3/Apple register"))),
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
