//! Serialized SCSI-transparent Bulk-Only Transport for all LUNs of one interface.
// SPDX-License-Identifier: GPL-2.0-only AND GPL-2.0-or-later
// Hardware protocol reference: Linux drivers/usb/storage/transport.c,
// Copyright 1999-2002 Matthew Dharm; 2000 David L. Brown, Jr.;
// 2000 Stephen J. Gowdy; 2002 Alan Stern; 1999 Michael Gee.
// Block-zero ordering: Linux sd.c, Drew Eckhardt 1992; Eric Youngdale
// 1993-1995, 1999. Lifecycle uses Stela's shared xHCI permission.
use super::super::descriptor::{DEVICE_TO_HOST, Interface, RECIPIENT_INTERFACE, TYPE_CLASS};
use super::super::xhci::{Device, Operation};
use super::{Data, Endpoint, MAX_TD, Shared, copy_read, scsi, setup};
mod ehci;
mod quirks;
use crate::{
    block::{BlockError, Sense},
    bus::{
        dma::{DmaAllocator, DmaBuffer},
        pci::power::Interval,
    },
};
use alloc::{boxed::Box, rc::Rc, vec::Vec};
use core::{
    cell::Cell,
    ops::{Deref, DerefMut},
};
pub(in crate::usb) use ehci::{EhciBulk, block as ehci_error};

const RESET: u8 = 0xff;
const GET_MAX_LUN: u8 = 0xfe;
const CBW_SIGNATURE: u32 = 0x4342_5355;
const CSW_SIGNATURE: u32 = 0x5342_5355;
/// bCSWStatus: Command Failed (BOT 5.2).
const CSW_FAILED: u8 = 1;
const COMMAND_US: u64 = 30_000_000;
/// A command's time while the interface's first scan runs. A device that
/// never answers its first command is reset and probed again, as the SCSI
/// error handler does, and that must fit, after the controller's own start
/// and enumeration, inside the start budget of the function it is on: with
/// [`COMMAND_US`] the budget passed first, and the reset never ran.
pub(super) const SCAN_COMMAND_US: u64 = 10_000_000;
const RESET_RECOVERY_US: u64 = 6_000_000;
const MAX_LUN: u8 = 15;
/// xHCI completion code 22: the slot must be retired, not cancelled.
const INCOMPATIBLE_DEVICE: u32 = 22;
const PROTOCOL_ERROR: BlockError = BlockError::Protocol("malformed Bulk-Only wrapper or transfer");
const SHORT_WRITE: BlockError = BlockError::Protocol("short Bulk-Only write");

pub(super) fn supported(device: &Device) -> Result<(), &'static str> {
    quirks::Flags::from_device(device).map(|_| ())
}

/// BOT's CPU view of transport buffers. Only a direct-DMA transport
/// has a bus address to pass to its controller.
pub(super) trait Buffer {
    fn bytes(&self) -> &[u8];
    fn bytes_mut(&mut self) -> &mut [u8];
}

impl Buffer for DmaBuffer {
    fn bytes(&self) -> &[u8] {
        self.bytes()
    }

    fn bytes_mut(&mut self) -> &mut [u8] {
        self.bytes_mut()
    }
}

impl Buffer for Vec<u8> {
    fn bytes(&self) -> &[u8] {
        self.as_slice()
    }

    fn bytes_mut(&mut self) -> &mut [u8] {
        self.as_mut_slice()
    }
}

/// Private seam: every native transfer returns only after completion or
/// xHCI cancellation. An error may leave poison; ready must expose it before
/// the CPU touches either transport buffer again. No hosted USB API is used.
pub(super) trait Io {
    type Buffer: Buffer;

    fn ready(&mut self) -> Result<(), BlockError>;
    fn control(&mut self, request: [u8; 8]) -> Result<Vec<u8>, BlockError>;
    fn clear(&mut self, endpoint: Endpoint) -> Result<(), BlockError>;
    fn bulk(
        &mut self,
        endpoint: Endpoint,
        buffer: &mut Self::Buffer,
        length: usize,
    ) -> Result<usize, BlockError>;
    /// Isolate the device after its transport was poisoned.
    fn quarantine(&mut self);
    fn delay(&self, micros: u64);
    fn minimum_wait(&mut self, micros: u64) -> Result<(), BlockError>;
    /// Start a minimum interval of `micros` that [`Self::minimum_elapsed`]
    /// polls; `None` where the timer has none, and the caller waits with
    /// [`Self::minimum_wait`].
    fn begin_minimum(&mut self, _micros: u64) -> Result<Option<Box<dyn Interval>>, BlockError> {
        Ok(None)
    }
    /// Whether a [`Self::begin_minimum`] interval has passed; never waits.
    fn minimum_elapsed(&mut self, interval: &dyn Interval) -> Result<bool, BlockError> {
        interval.elapsed().map_err(BlockError::PciPower)
    }
    /// The transport's clock, in microseconds.
    fn now_us(&self) -> u64;
    /// Whether the host controller sends the handoff's completion signal.
    fn signals(&self) -> bool {
        false
    }
    /// Whether a submitted transfer may have completed, for [`Self::reap`];
    /// a look at memory alone.
    fn posted(&self) -> bool {
        false
    }
    /// Submit a bulk transfer and return; [`Self::reap`] gives its length
    /// once complete. By default it runs at once.
    fn submit(
        &mut self,
        endpoint: Endpoint,
        buffer: &mut Self::Buffer,
        length: usize,
    ) -> Result<Pending, BlockError> {
        Ok(Pending::Done(self.bulk(endpoint, buffer, length)))
    }
    /// A submitted transfer's length once complete; never waits. A failed
    /// or timed-out one has had its endpoint's queue cancelled, as a
    /// failed [`Self::bulk`] has.
    fn reap(&mut self, pending: &mut Pending) -> Option<Result<usize, BlockError>> {
        match pending {
            Pending::Done(result) => Some(result.clone()),
            Pending::Ticket { .. } | Pending::Cancelling(..) => Some(Err(BlockError::Protocol(
                "a BOT transfer ticket on a transport without tickets",
            ))),
        }
    }
    /// Start a control transfer; [`Self::look`] gives its result. By
    /// default it runs at once.
    fn begin_control(&mut self, request: [u8; 8]) -> Request {
        Request::Done(self.control(request))
    }
    /// Start clearing a halt; [`Self::look`] gives its result. By default
    /// it runs at once.
    fn begin_clear(&mut self, endpoint: Endpoint) -> Request {
        Request::Done(self.clear(endpoint).map(|()| Vec::new()))
    }
    /// A started request's result once it ended; never waits.
    fn look(&mut self, request: &mut Request) -> Option<Result<Vec<u8>, BlockError>> {
        match request {
            Request::Done(result) => Some(result.clone()),
            Request::Operation(_) => Some(Err(BlockError::Protocol(
                "an xHCI operation on a transport without them",
            ))),
        }
    }
    /// The port reset and rebind after a failed class reset of `bot`
    /// ([`super::Scsi::reset_device`]); none where the host has no such step.
    #[allow(
        clippy::type_complexity,
        reason = "the replacement transport, or the failed operation and its exact cause"
    )]
    fn reset_device(
        _bot: &mut Bot<Self>,
        _dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Bot<Self>, (alloc::string::String, BlockError)>>
    where
        Self: Sized,
    {
        None
    }
    /// Whether [`Self::reset_device`] needs fresh rings from its allocator,
    /// as a device behind a hub does.
    fn reset_needs_allocation(&self) -> bool {
        false
    }
}

/// A control transfer or halt clear ([`Io::begin_control`], [`Io::begin_clear`]).
pub(super) enum Request {
    Done(Result<Vec<u8>, BlockError>),
    Operation(Operation),
}

impl Request {
    fn started(operation: Result<Operation, BlockError>) -> Self {
        operation.map_or_else(|error| Self::Done(Err(error)), Self::Operation)
    }
}

/// A submitted bulk transfer ([`Io::submit`]).
#[expect(
    clippy::large_enum_variant,
    reason = "Cancellation keeps the in-flight operation and exact failure in one allocation-free state transition."
)]
pub(super) enum Pending {
    Done(Result<usize, BlockError>),
    Ticket {
        ticket: super::super::xhci::TransferTicket,
        dci: u8,
        deadline: crate::Deadline,
        status_due: Option<crate::Deadline>,
    },
    /// The endpoint's cancellation after a failure, which it then reports.
    Cancelling(Operation, BlockError),
}

pub(super) struct Native {
    pub controller: Shared,
    pub id: super::super::xhci::SlotId,
    pub endpoints: [Endpoint; 2],
    /// A command's time: [`SCAN_COMMAND_US`] until the first scan ends,
    /// then [`COMMAND_US`].
    pub command_us: Rc<Cell<u64>>,
}

/// The command time a new interface starts with, and its owner's handle to
/// raise it by [`scanned`].
pub(super) fn scan_command_time() -> Rc<Cell<u64>> {
    Rc::new(Cell::new(SCAN_COMMAND_US))
}

/// The first scan ended: later commands get the full time.
pub(super) fn scanned(command_us: &Cell<u64>) {
    command_us.set(COMMAND_US);
}

impl Io for Native {
    type Buffer = DmaBuffer;

    fn reset_device(
        bot: &mut Bot<Self>,
        dma: &mut dyn DmaAllocator,
    ) -> Option<Result<Bot<Self>, (alloc::string::String, BlockError)>> {
        Some(bot.reset_and_rebind(dma))
    }

    fn reset_needs_allocation(&self) -> bool {
        self.controller
            .try_borrow()
            .is_ok_and(|controller| controller.reset_needs_allocation(self.id))
    }

    fn signals(&self) -> bool {
        self.controller
            .try_borrow()
            .is_ok_and(|controller| controller.signals())
    }
    fn posted(&self) -> bool {
        self.controller
            .try_borrow()
            .is_ok_and(|controller| controller.posted_for(self.id))
    }
    fn ready(&mut self) -> Result<(), BlockError> {
        self.controller
            .borrow_mut()
            .device(self.id)?
            .endpoints_ready(&[1, self.endpoints[0].dci(), self.endpoints[1].dci()])
    }
    fn control(&mut self, request: [u8; 8]) -> Result<Vec<u8>, BlockError> {
        self.controller
            .borrow_mut()
            .device(self.id)?
            .control(request, &[])
    }
    fn clear(&mut self, endpoint: Endpoint) -> Result<(), BlockError> {
        self.controller
            .borrow_mut()
            .device(self.id)?
            .clear(&endpoint)
    }
    fn bulk(
        &mut self,
        endpoint: Endpoint,
        buffer: &mut Self::Buffer,
        length: usize,
    ) -> Result<usize, BlockError> {
        self.controller.borrow_mut().device(self.id)?.transfer(
            endpoint.dci(),
            buffer.physical(),
            length,
            self.command_us.get(),
        )
    }
    fn quarantine(&mut self) {
        let mut controller = self.controller.borrow_mut();
        if let Ok(mut device) = controller.device(self.id) {
            device.quarantine();
        }
    }
    fn delay(&self, micros: u64) {
        let clock = self.controller.borrow().clock;
        clock.delay(micros);
    }
    fn minimum_wait(&mut self, micros: u64) -> Result<(), BlockError> {
        let mut controller = self.controller.borrow_mut();
        controller.device(self.id)?.connected()?;
        controller.minimum_wait(micros)
    }
    fn begin_minimum(&mut self, micros: u64) -> Result<Option<Box<dyn Interval>>, BlockError> {
        let mut controller = self.controller.borrow_mut();
        controller.device(self.id)?.connected()?;
        controller.begin_minimum(micros)
    }
    fn minimum_elapsed(&mut self, interval: &dyn Interval) -> Result<bool, BlockError> {
        self.controller.borrow_mut().minimum_elapsed(interval)
    }
    fn now_us(&self) -> u64 {
        self.controller.borrow().clock.now_us()
    }
    fn submit(
        &mut self,
        endpoint: Endpoint,
        buffer: &mut Self::Buffer,
        length: usize,
    ) -> Result<Pending, BlockError> {
        let buffer = buffer.physical();
        let mut controller = self.controller.borrow_mut();
        let deadline = controller
            .clock
            .try_deadline(self.command_us.get())
            .map_err(|_| BlockError::Unsupported("xHCI transfer timeout clock unavailable"))?;
        let dci = endpoint.dci();
        let ticket = controller
            .device(self.id)?
            .submit_ticket(dci, buffer, length)?;
        Ok(Pending::Ticket {
            ticket,
            dci,
            deadline,
            status_due: None,
        })
    }
    fn begin_control(&mut self, request: [u8; 8]) -> Request {
        let mut controller = self.controller.borrow_mut();
        Request::started(
            controller
                .device(self.id)
                .and_then(|mut device| device.start_control(request, &[])),
        )
    }
    fn begin_clear(&mut self, endpoint: Endpoint) -> Request {
        let mut controller = self.controller.borrow_mut();
        Request::started(
            controller
                .device(self.id)
                .and_then(|mut device| device.start_clear(&endpoint)),
        )
    }
    fn look(&mut self, request: &mut Request) -> Option<Result<Vec<u8>, BlockError>> {
        let operation = match request {
            Request::Done(result) => return Some(result.clone()),
            Request::Operation(operation) => operation,
        };
        let mut controller = self.controller.borrow_mut();
        match controller.device(self.id) {
            Ok(mut device) => device.poll(operation),
            Err(error) => Some(Err(error)),
        }
    }
    /// As the xHCI's own waited transfer: a failure or a timeout cancels
    /// the endpoint's queue before the buffer returns to the CPU.
    fn reap(&mut self, pending: &mut Pending) -> Option<Result<usize, BlockError>> {
        let mut controller = self.controller.borrow_mut();
        let mut device = match controller.device(self.id) {
            Ok(device) => device,
            Err(error) => return Some(Err(error)),
        };
        if let Pending::Ticket {
            ticket,
            dci,
            deadline,
            status_due,
        } = pending
        {
            let error = match device.poll_ticket_until(ticket, deadline, status_due) {
                Some(Ok(length)) => return Some(Ok(length)),
                Some(Err(error)) => {
                    let incompatible = matches!(
                        error,
                        BlockError::Device {
                            status: INCOMPATIBLE_DEVICE
                        }
                    );
                    if incompatible {
                        return Some(Err(error));
                    }
                    error
                }
                None => return None,
            };
            match device.start_cancel(*dci) {
                Ok(operation) => *pending = Pending::Cancelling(operation, error),
                Err(cancel) => return Some(Err(cancel)),
            }
        }
        match pending {
            Pending::Done(result) => Some(result.clone()),
            Pending::Cancelling(operation, error) => {
                Some(device.poll(operation)?.and(Err(error.clone())))
            }
            Pending::Ticket { .. } => unreachable!(),
        }
    }
}

/// The unique owner can move to a fresh transport only after whole-device
/// retirement and successful native reconstruction. Old media aliases
/// keep their poisoned Bot, whose buffer slots are then permanently empty.
/// The replacement takes the original allocation intact.
struct TransportBuffer<B>(Option<B>);
impl<B> TransportBuffer<B> {
    fn new(buffer: B) -> Self {
        Self(Some(buffer))
    }
    fn present(&self) -> bool {
        self.0.is_some()
    }
    fn transfer(&mut self) -> Self {
        Self(Some(self.0.take().expect("retained BOT buffer")))
    }
}
impl<B> Deref for TransportBuffer<B> {
    type Target = B;
    fn deref(&self) -> &B {
        self.0.as_ref().expect("retained BOT buffer")
    }
}
impl<B> DerefMut for TransportBuffer<B> {
    fn deref_mut(&mut self) -> &mut B {
        self.0.as_mut().expect("retained BOT buffer")
    }
}

pub(super) struct Bot<I: Io> {
    pub io: I,
    interface: u8,
    endpoints: [Endpoint; 2],
    buffer: TransportBuffer<I::Buffer>,
    wrapper: TransportBuffer<I::Buffer>,
    tag: u32,
    max_lun: u8,
    poisoned: bool,
    minimum_error: Option<BlockError>,
    flags: quirks::Flags,
    initial_read: bool,
    attachment: Option<(Device, Interface)>,
    reset_generation: u64,
    /// A reset recovery under way ([`Self::recover`]).
    recovery: Option<Recovery>,
    /// The command begun and not yet ended ([`Self::begin`]).
    begun: Option<Begun>,
    /// The data a failed command's data phase moved in, until the next
    /// command: sd keeps it when the sense is RECOVERED ERROR.
    failed_data: Option<Vec<u8>>,
}

/// A command between its wrapper and its status, one transfer at a time,
/// none waited for ([`Bot::begin`]).
struct Begun {
    lun: u8,
    opcode: u8,
    is_in: bool,
    length: usize,
    out: Option<Vec<u8>>,
    received: Vec<u8>,
    moved: usize,
    /// The status stage met a zero-length packet, or a stall.
    zero: bool,
    stalled: bool,
    stage: Stage,
    /// The command is the REQUEST SENSE that answers another's result.
    sensing: Option<Sensing>,
}

/// A reset recovery's progress, as usb_stor_reset_common. Request completion
/// is polled; the physical settling phase uses the fallible actual owner.
enum Recovery {
    /// The class reset request.
    Resetting(Request),
    /// Nominal scheduling time, with the timer's own interval for the
    /// complete physical minimum where it keeps one: polled, never waited
    /// for. Without one, an actual owner must still supply that minimum,
    /// waiting, before any halt clear.
    Settling(u64, Option<Box<dyn Interval>>),
    /// A halt's clear: the input's, or with its result the output's.
    Clearing(Request, Option<Result<Vec<u8>, BlockError>>),
}

/// What a REQUEST SENSE answers (Linux `usb_stor_invoke_transport`).
#[derive(Clone, Copy)]
enum Sensing {
    /// The command reported CHECK CONDITION: its auto-sense.
    Failed,
    /// A SYNCHRONIZE CACHE the device's quirk leaves without status.
    Synchronized,
}

enum Stage {
    /// The CBW, of this many bytes.
    Wrapper(Pending, usize),
    /// A data transfer of this many bytes.
    Data(Pending, usize),
    Status(Pending),
    /// A stalled pipe's clear, then the status stage, submitted again when
    /// it was the status stage that stalled.
    Clearing(Request, bool),
}

/// What a step of a begun command left.
#[expect(
    clippy::large_enum_variant,
    reason = "Descriptor progress carries the full pending stage or exact completion cause without allocating."
)]
enum Next {
    Wait,
    Stage(Stage),
    Done(Result<Vec<u8>, BlockError>),
}

impl<I: Io<Buffer = DmaBuffer>> Bot<I> {
    #[cfg(test)]
    pub fn new(
        io: I,
        interface: u8,
        endpoints: [Endpoint; 2],
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        Self::with_flags(io, interface, endpoints, dma, quirks::Flags::default())
    }
    fn with_flags(
        mut io: I,
        interface: u8,
        endpoints: [Endpoint; 2],
        dma: &mut dyn DmaAllocator,
        flags: quirks::Flags,
    ) -> Result<Self, BlockError> {
        let max_lun = Self::startup(&mut io, interface, flags)?;
        Ok(Self::with_buffers(
            io,
            interface,
            endpoints,
            TransportBuffer::new(dma.allocate(MAX_TD, MAX_TD)?),
            TransportBuffer::new(dma.allocate(64, 64)?),
            flags,
            max_lun,
        ))
    }
}

impl<I: Io> Bot<I> {
    /// Perform transport admission before committing any moved buffers.
    fn startup(io: &mut I, interface: u8, flags: quirks::Flags) -> Result<u8, BlockError> {
        io.ready()?;
        let request = setup(
            DEVICE_TO_HOST | TYPE_CLASS | RECIPIENT_INTERFACE,
            GET_MAX_LUN,
            0,
            u16::from(interface),
            1,
        );
        Ok(if flags.single_lun() {
            0
        } else {
            match io.control(request) {
                Ok(bytes) if bytes.len() == 1 && bytes[0] <= MAX_LUN => bytes[0],
                // Linux `usb_stor_Bulk_max_lun`: devices stall this request, send
                // nothing, time out or answer nonsense; all mean one LUN. EP0
                // must still be usable afterwards.
                _ => {
                    io.ready()?;
                    0
                }
            }
        })
    }
    fn with_buffers(
        io: I,
        interface: u8,
        endpoints: [Endpoint; 2],
        buffer: TransportBuffer<I::Buffer>,
        wrapper: TransportBuffer<I::Buffer>,
        flags: quirks::Flags,
        max_lun: u8,
    ) -> Self {
        Self {
            io,
            interface,
            endpoints,
            buffer,
            wrapper,
            tag: 0,
            max_lun,
            poisoned: false,
            minimum_error: None,
            flags,
            initial_read: flags.initial_read(),
            attachment: None,
            reset_generation: 0,
            recovery: None,
            begun: None,
            failed_data: None,
        }
    }
    /// A transport over `io` that takes this one's buffers, cleared, once a
    /// reset retired every request of this one. This one stays poisoned, so
    /// its aliases cannot inspect or submit the buffers again.
    fn moved_to(&mut self, io: I, interface: u8, flags: quirks::Flags, max_lun: u8) -> Self {
        self.poisoned = true;
        let mut buffer = self.buffer.transfer();
        let mut wrapper = self.wrapper.transfer();
        buffer.bytes_mut().fill(0);
        wrapper.bytes_mut().fill(0);
        Self::with_buffers(
            io,
            interface,
            self.endpoints,
            buffer,
            wrapper,
            flags,
            max_lun,
        )
    }
    pub fn max_lun(&self) -> u8 {
        self.max_lun
    }
    /// Poison the transport and isolate its device.
    pub fn quarantine(&mut self) {
        self.poisoned = true;
        self.io.quarantine();
    }
    pub fn poisoned(&self) -> bool {
        self.poisoned
    }
    pub fn now_us(&self) -> u64 {
        self.io.now_us()
    }
    pub fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        let result = self.io.minimum_wait(micros);
        if let Err(error) = &result {
            // Keep every begun transfer, recovery request, and buffer before
            // returning a result to any class completion callback.
            self.minimum_error = Some(error.clone());
            self.poisoned = true;
        }
        result
    }
    /// What the last command's data phase moved in before it failed.
    pub fn take_failed_data(&mut self) -> Option<Vec<u8>> {
        self.failed_data.take()
    }

    pub fn generation(&self) -> u64 {
        self.reset_generation
    }
    pub fn write_protect(&self) -> bool {
        self.flags.write_protect()
    }
    pub fn write_cache(&self) -> bool {
        self.flags.write_cache()
    }
    pub fn capacity16_first(&self) -> bool {
        self.flags.capacity16_first()
    }
    pub fn capacity(&self, blocks: u64) -> Option<u64> {
        self.flags.capacity(blocks)
    }
    pub fn max_transfer_bytes(&self) -> usize {
        self.flags.max_transfer_bytes()
    }
    pub fn read_before_mode_sense(&mut self, lun: u8, block_size: usize) -> Result<(), BlockError> {
        let mut cdb = [0; 10];
        cdb[0] = scsi::READ_10;
        cdb[8] = 1;
        if self.initial_read {
            // Linux INITIAL_READ10 discards the first result. This workaround
            // runs only while probing metadata before publication, never on a
            // caller's data command or after an observed media change.
            self.initial_read = false;
            match self.command(lun, &cdb, Data::In(block_size)) {
                Err(BlockError::Sense(sense)) if super::media::changed(&sense, true) => {
                    return Err(BlockError::Sense(sense));
                }
                Err(error) if self.poisoned => return Err(error),
                _ => {}
            }
        }
        let bytes = self.command(lun, &cdb, Data::In(block_size))?;
        if bytes.len() != block_size {
            return Err(BlockError::Protocol("short USB block-zero read"));
        }
        Ok(())
    }

    fn ready(&mut self) -> Result<(), BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        if self.poisoned || !self.buffer.present() || !self.wrapper.present() {
            return Err(BlockError::Unsupported(
                "USB storage interface requires rebind",
            ));
        }
        if let Err(error) = self.io.ready() {
            if matches!(error, BlockError::PciPower(_)) {
                self.minimum_error = Some(error.clone());
            }
            self.poisoned = true;
            return Err(error);
        }
        Ok(())
    }
    /// Reset and halt clearing affect *every* LUN. All acknowledgements must
    /// succeed to reuse the shared transport. Unresolved DMA stays in permanent
    /// boot allocations, and the entire interface remains quarantined.
    ///
    /// Acknowledged class reset is not device-side recovery completion: as
    /// usb_stor_reset_common, the device settles for HZ*6 before either
    /// CLEAR_FEATURE, which controller readiness alone cannot replace. No
    /// command is admitted until the genuine minimum is supplied. Polling may
    /// defer that blocking owner operation until the nominal deadline. The
    /// transport takes no command until
    /// both halts are cleared ([`Self::recovering`]).
    fn recover(&mut self) {
        if let Err(error) = self.io.ready() {
            if matches!(error, BlockError::PciPower(_)) {
                self.minimum_error = Some(error);
            }
            self.poisoned = true;
            return;
        }
        let request = self.io.begin_control(setup(
            TYPE_CLASS | RECIPIENT_INTERFACE,
            RESET,
            0,
            u16::from(self.interface),
            0,
        ));
        self.recovery = Some(Recovery::Resetting(request));
        self.recovering();
    }

    /// Whether a reset recovery is under way, moved on as far as it goes.
    fn recovering(&mut self) -> bool {
        loop {
            let Some(recovery) = &mut self.recovery else {
                return false;
            };
            match recovery {
                Recovery::Resetting(request) => {
                    let Some(result) = self.io.look(request) else {
                        return true;
                    };
                    if result.is_ok() {
                        // The acknowledged reset ends every published unit's media.
                        let settled = self.io.now_us().saturating_add(RESET_RECOVERY_US);
                        let interval = match self.io.begin_minimum(RESET_RECOVERY_US) {
                            Ok(interval) => interval,
                            Err(error) => {
                                self.minimum_error = Some(error);
                                self.poisoned = true;
                                return false;
                            }
                        };
                        self.recovery = Some(Recovery::Settling(settled, interval));
                        self.reset_generation = self.reset_generation.wrapping_add(1);
                    } else {
                        self.recovery = None;
                        self.poisoned = true;
                    }
                }
                Recovery::Settling(settled, interval) => {
                    let settled = *settled;
                    if self.minimum_error.is_some() {
                        return false;
                    }
                    if self.io.now_us() < settled {
                        return true;
                    }
                    let measured = interval.take();
                    if let Some(interval) = &measured {
                        match self.io.minimum_elapsed(interval.as_ref()) {
                            Ok(true) => {}
                            Ok(false) => {
                                self.recovery = Some(Recovery::Settling(settled, measured));
                                return true;
                            }
                            Err(error) => {
                                self.minimum_error = Some(error);
                                self.poisoned = true;
                                return false;
                            }
                        }
                    }
                    if let Err(error) = self.io.ready() {
                        if matches!(error, BlockError::PciPower(_)) {
                            self.minimum_error = Some(error);
                            self.poisoned = true;
                            return false;
                        }
                        self.recovery = None;
                        self.poisoned = true;
                        continue;
                    }
                    // The nominal deadline only schedules work. Without the
                    // timer's interval a genuine owner now supplies the
                    // entire conservative minimum even if numeric clock time
                    // already advanced past the deadline.
                    if measured.is_none() && self.delay(RESET_RECOVERY_US).is_err() {
                        return false;
                    }
                    let request = self.io.begin_clear(self.endpoints[0]);
                    self.recovery = Some(Recovery::Clearing(request, None));
                }
                Recovery::Clearing(request, input) => {
                    let Some(result) = self.io.look(request) else {
                        return true;
                    };
                    match input.take() {
                        // Both halts are cleared whatever the first did.
                        None => {
                            let request = self.io.begin_clear(self.endpoints[1]);
                            self.recovery = Some(Recovery::Clearing(request, Some(result)));
                        }
                        Some(input) => {
                            self.recovery = None;
                            if input.and(result).and_then(|_| self.io.ready()).is_err() {
                                self.poisoned = true;
                            }
                        }
                    }
                }
            }
        }
    }
    fn failed<T>(&mut self, error: BlockError) -> Result<T, BlockError> {
        self.recover();
        Err(self.minimum_error.clone().unwrap_or(error))
    }

    /// Run `cdb` on `lun`, the caller waiting, for a reset recovery's
    /// settling first.
    pub fn command(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
        self.command_wait(lun, cdb, data, None)
    }

    /// For a single data transfer, its DMA buffer holds the payload until
    /// the separate status transfer completes. The caller's slice stays on
    /// the waiting stack and is filled only after the CSW validates it.
    pub fn read_into(&mut self, lun: u8, cdb: &[u8], output: &mut [u8]) -> Result<(), BlockError> {
        let data = Data::In(output.len());
        if output.is_empty() || output.len() > MAX_TD {
            return copy_read(output, &self.command(lun, cdb, data)?);
        }
        self.command_wait(lun, cdb, data, Some(output)).map(|_| ())
    }

    fn command_wait(
        &mut self,
        lun: u8,
        cdb: &[u8],
        data: Data<'_>,
        output: Option<&mut [u8]>,
    ) -> Result<Vec<u8>, BlockError> {
        while self.recovering() {
            if let Some(Recovery::Settling(settled, _)) = self.recovery {
                self.io.delay(settled.saturating_sub(self.io.now_us()));
            } else {
                crate::waiting();
                core::hint::spin_loop();
            }
        }
        let result = self.begin(lun, cdb, data).and_then(|()| self.wait(output));
        // The waiting caller sees the generation a failure's reset left.
        while matches!(self.recovery, Some(Recovery::Resetting(_))) && self.recovering() {
            crate::waiting();
            core::hint::spin_loop();
        }
        self.minimum_error.clone().map_or(result, Err)
    }

    /// The begun command's result, the caller waiting for its transfers.
    fn wait(&mut self, mut output: Option<&mut [u8]>) -> Result<Vec<u8>, BlockError> {
        loop {
            if let Some(result) = self.end_with_output(output.as_deref_mut(), false) {
                return result;
            }
            crate::waiting();
            core::hint::spin_loop();
        }
    }

    /// Whether a begun command has not ended, or a reset recovery settles.
    pub fn busy(&mut self) -> bool {
        self.begun.is_some() || self.recovering()
    }

    /// Begin `cdb` on `lun` with its data stage and return once its CBW is
    /// submitted; [`Self::end`] gives the result.
    pub fn begin(&mut self, lun: u8, cdb: &[u8], data: Data<'_>) -> Result<(), BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        if self.begun.is_some() || self.recovering() {
            return Err(BlockError::Protocol(
                "BOT runs one command at a time, none while its reset settles",
            ));
        }
        if lun > self.max_lun || cdb.is_empty() || cdb.len() > 16 || !self.flags.permits(cdb) {
            return Err(BlockError::Unsupported(
                "invalid USB storage LUN or command",
            ));
        }
        self.failed_data = None;
        let (length, is_in, out) = match data {
            Data::None => (0, false, None),
            Data::In(n) => (n, true, None),
            Data::Out(b) => (b.len(), false, Some(b.to_vec())),
        };
        if length > self.max_transfer_bytes() {
            return Err(BlockError::Unsupported(
                "USB storage transfer above device bound",
            ));
        }
        self.ready()?;
        self.tag = self.tag.wrapping_add(1);
        let wrapper = cbw(self.tag, lun, cdb, length, is_in);
        self.wrapper.bytes_mut()[..31].copy_from_slice(&wrapper);
        // BULK32 pads the native transfer; stale wrapper bytes cannot
        // leak through the extra donor-defined byte.
        let cbw_length = self.flags.cbw_len();
        self.wrapper.bytes_mut()[31] = 0;
        let pending = match self
            .io
            .submit(self.endpoints[1], &mut self.wrapper, cbw_length)
        {
            Ok(pending) => pending,
            Err(error) => return self.failed(error),
        };
        self.begun = Some(Begun {
            lun,
            opcode: cdb[0],
            is_in,
            length,
            out,
            received: Vec::new(),
            moved: 0,
            zero: false,
            stalled: false,
            stage: Stage::Wrapper(pending, cbw_length),
            sensing: None,
        });
        Ok(())
    }

    /// The begun command's data once its status arrived, with same-LUN
    /// auto-sense after a failure, which runs as the command did; never
    /// waits for a transfer. A call moves the command by one device round
    /// trip at most: it takes a completed transfer and submits at most one,
    /// even when the device completes each at its doorbell, as an emulated
    /// one may in the submitting CPU's exit; the caller may hold a lock
    /// other disks' commands wait for.
    pub fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        self.end_with_output(None, true)
    }

    /// [`Self::end`], moving on through every transfer already complete
    /// unless `bounded`, for a caller that waits for the command anyway.
    fn end_with_output(
        &mut self,
        output: Option<&mut [u8]>,
        bounded: bool,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        if let Some(error) = &self.minimum_error {
            return self.begun.as_ref().map(|_| Err(error.clone()));
        }
        let mut begun = self.begun.take()?;
        // Auto-sense can resume on a later poll of the original read.
        // Its response belongs to the sense parser, not the read's buffer.
        let mut output = output.filter(|_| begun.sensing.is_none());
        let result = loop {
            match self.advance(&mut begun, output.as_deref_mut()) {
                Next::Wait => {
                    self.begun = Some(begun);
                    return None;
                }
                // Each new stage submitted a transfer.
                Next::Stage(stage) if bounded => {
                    begun.stage = stage;
                    self.begun = Some(begun);
                    return None;
                }
                Next::Stage(stage) => begun.stage = stage,
                Next::Done(result) => break result,
            }
        };
        if let Some(error) = &self.minimum_error {
            let error = error.clone();
            self.begun = Some(begun);
            return Some(Err(error));
        }
        match begun.sensing {
            Some(Sensing::Failed) => {
                let sense = result.ok().and_then(|bytes| Sense::parse(&bytes));
                return Some(Err(
                    sense.map_or(BlockError::CheckCondition, BlockError::Sense)
                ));
            }
            Some(Sensing::Synchronized) => {
                return Some(result.and_then(|bytes| {
                    let sense = Sense::parse(&bytes).ok_or(PROTOCOL_ERROR)?;
                    if sense.key != 0 {
                        return Err(BlockError::Sense(sense));
                    }
                    Ok(Vec::new())
                }));
            }
            None => {}
        }
        let (lun, opcode) = (begun.lun, begun.opcode);
        // SCSI auto-sense must use the failing command's LUN, and cannot race
        // a different unit's command while the interface is borrowed.
        let sensing = if result == Err(BlockError::CheckCondition) {
            (opcode != scsi::REQUEST_SENSE).then_some(Sensing::Failed)
        } else {
            (result.is_ok()
                && opcode == scsi::SYNCHRONIZE_CACHE_10
                && self.flags.sense_after_sync())
            .then_some(Sensing::Synchronized)
        };
        let Some(sensing) = sensing else {
            return Some(result);
        };
        // The failed command's data outlives the sense command.
        let moved = self.failed_data.take();
        let begun = self.begin(lun, &scsi::request_sense(), Data::In(scsi::SENSE_BYTES));
        self.failed_data = moved;
        if let Err(error) = begun {
            return Some(Err(match sensing {
                Sensing::Failed => BlockError::CheckCondition,
                Sensing::Synchronized => error,
            }));
        }
        self.begun.as_mut().unwrap().sensing = Some(sensing);
        if bounded {
            return None;
        }
        self.end_with_output(None, false)
    }

    /// Move the begun command on as far as its completed transfers allow.
    fn advance(&mut self, begun: &mut Begun, output: Option<&mut [u8]>) -> Next {
        match &mut begun.stage {
            Stage::Wrapper(pending, cbw_length) => {
                let cbw_length = *cbw_length;
                let Some(result) = self.io.reap(pending) else {
                    return Next::Wait;
                };
                match result {
                    Ok(n) if n == cbw_length => {}
                    Ok(_) => return Next::Done(self.failed(PROTOCOL_ERROR)),
                    Err(error) => return Next::Done(self.failed(error)),
                }
                if self.flags.go_slow()
                    && let Err(error) = self.delay(125)
                {
                    return Next::Done(Err(error));
                }
                self.data_or_status(begun, output)
            }
            Stage::Data(pending, piece) => {
                let piece = *piece;
                let Some(result) = self.io.reap(pending) else {
                    return Next::Wait;
                };
                let endpoint = self.endpoints[usize::from(!begun.is_in)];
                match result {
                    Ok(n) if n <= piece => {
                        if begun.is_in && (output.is_none() || n != begun.length) {
                            begun.received.extend_from_slice(&self.buffer.bytes()[..n]);
                        }
                        begun.moved += n;
                        if n < piece {
                            return self.status(begun, output);
                        }
                        self.data_or_status(begun, output)
                    }
                    Ok(_) => Next::Done(self.failed(PROTOCOL_ERROR)),
                    Err(BlockError::Device {
                        status: super::STALL,
                    }) => Next::Stage(Stage::Clearing(self.io.begin_clear(endpoint), false)),
                    Err(error) => Next::Done(self.failed(error)),
                }
            }
            Stage::Status(pending) => {
                let Some(result) = self.io.reap(pending) else {
                    return Next::Wait;
                };
                match result {
                    Ok(13) => Next::Done(self.conclude(begun, output)),
                    Ok(0) if !begun.zero && !begun.stalled => {
                        begun.zero = true;
                        self.submit_status()
                    }
                    Err(BlockError::Device {
                        status: super::STALL,
                    }) if !begun.stalled => {
                        begun.stalled = true;
                        // As Linux's usb_stor_bulk_transfer_buf: the halt is
                        // cleared at once and the status read again; no reset
                        // ran, so nothing settles.
                        let request = self.io.begin_clear(self.endpoints[0]);
                        Next::Stage(Stage::Clearing(request, true))
                    }
                    Ok(_) => Next::Done(self.failed(PROTOCOL_ERROR)),
                    Err(error) => Next::Done(self.failed(error)),
                }
            }
            Stage::Clearing(request, resubmit) => {
                let resubmit = *resubmit;
                match self.io.look(request) {
                    None => Next::Wait,
                    Some(Err(error)) => Next::Done(self.failed(error)),
                    Some(Ok(_)) if resubmit => self.submit_status(),
                    Some(Ok(_)) => self.status(begun, output),
                }
            }
        }
    }

    /// A data stage longer than one transfer descriptor moves as
    /// consecutive ones of `MAX_TD` bytes, each a whole number of packets,
    /// as Linux's scatter-gather list does, until the device ends it short
    /// or stalls.
    fn data_or_status(&mut self, begun: &mut Begun, output: Option<&mut [u8]>) -> Next {
        if begun.moved >= begun.length {
            return self.status(begun, output);
        }
        let piece = (begun.length - begun.moved).min(MAX_TD);
        // No uncertain CBW transfer can be followed by a CPU data write.
        if let Some(bytes) = &begun.out {
            self.buffer.bytes_mut()[..piece]
                .copy_from_slice(&bytes[begun.moved..begun.moved + piece]);
        }
        let endpoint = self.endpoints[usize::from(!begun.is_in)];
        match self.io.submit(endpoint, &mut self.buffer, piece) {
            Ok(pending) => Next::Stage(Stage::Data(pending, piece)),
            Err(error) => Next::Done(self.failed(error)),
        }
    }

    /// Some bridges skip the data phase and send the CSW directly. Keep
    /// status bytes out of returned media and do not wait for a second CSW.
    fn status(&mut self, begun: &mut Begun, output: Option<&mut [u8]>) -> Next {
        let skipped = begun.is_in
            && begun.moved == 13
            && begun.length > 13
            && u32::from_le_bytes(begun.received[..4].try_into().unwrap()) == CSW_SIGNATURE;
        if skipped {
            self.wrapper.bytes_mut()[..13].copy_from_slice(&begun.received[..13]);
            begun.moved = 0;
            return Next::Done(self.conclude(begun, output));
        }
        self.submit_status()
    }

    fn submit_status(&mut self) -> Next {
        match self.io.submit(self.endpoints[0], &mut self.wrapper, 13) {
            Ok(pending) => Next::Stage(Stage::Status(pending)),
            Err(error) => Next::Done(self.failed(error)),
        }
    }

    /// The command's result from its CSW.
    fn conclude(
        &mut self,
        begun: &mut Begun,
        output: Option<&mut [u8]>,
    ) -> Result<Vec<u8>, BlockError> {
        let (opcode, is_in, length, moved) = (begun.opcode, begun.is_in, begun.length, begun.moved);
        let mut received = core::mem::take(&mut begun.received);
        let bytes = &self.wrapper.bytes()[..13];
        let residue = u32::from_le_bytes(bytes[8..12].try_into().unwrap()) as usize;
        let status = bytes[12];
        if u32::from_le_bytes(bytes[..4].try_into().unwrap()) != CSW_SIGNATURE
            || (!self.flags.ignore_tag()
                && u32::from_le_bytes(bytes[4..8].try_into().unwrap()) != self.tag)
            || status > 1
        {
            return self.failed(PROTOCOL_ERROR);
        }
        if status == CSW_FAILED {
            if is_in && begun.sensing.is_none() {
                // REQUEST SENSE reuses the DMA buffer. Preserve a full
                // transfer for the block layer's RECOVERED ERROR handling.
                if output.is_some() && moved == length {
                    received.extend_from_slice(&self.buffer.bytes()[..moved]);
                }
                self.failed_data = Some(received);
            }
            return Err(BlockError::CheckCondition);
        }
        // transport.c detects bogus residue only from completely received,
        // successful fixed-size identification replies; no bytes are invented.
        if is_in
            && moved == length
            && residue != 0
            && ((opcode == scsi::INQUIRY && length == 36)
                || (opcode == scsi::READ_CAPACITY_10 && length == 8))
        {
            self.flags.learn_residue();
        }
        if !self.flags.ignore_residue() && residue > length {
            return self.failed(PROTOCOL_ERROR);
        }
        let residue = if self.flags.ignore_residue() {
            0
        } else {
            residue
        };
        // A successful CSW does not turn a partial block write into success.
        if !is_in && (moved != length || residue != 0) {
            return Err(SHORT_WRITE);
        }
        let count = moved.min(length - residue);
        if let Some(output) = output {
            copy_read(output, &self.buffer.bytes()[..count])?;
        }
        received.truncate(count);
        Ok(received)
    }
}

impl Bot<Native> {
    pub fn native(
        io: Native,
        device: &Device,
        setting: &Interface,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        let flags = quirks::Flags::from_device(device).map_err(BlockError::Unsupported)?;
        let endpoints = io.endpoints;
        let mut bot = Self::with_flags(io, setting.number, endpoints, dma, flags)?;
        bot.attachment = Some((device.clone(), setting.clone()));
        Ok(bot)
    }
    pub fn device(&self) -> &Device {
        &self.attachment.as_ref().expect("native BOT attachment").0
    }
    pub fn interface(&self) -> u8 {
        self.interface
    }
    /// Reconstruct the attachment while keeping its two transport
    /// allocations. Root-leaf reset also reuses the controller's retired
    /// resources; downstream reset keeps the existing allocation behavior.
    /// Every fallible reconstruction step precedes moving either buffer owner.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub fn reset_and_rebind(
        &mut self,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, (alloc::string::String, BlockError)> {
        self.poisoned = true;
        let fail = |why| {
            (
                alloc::string::String::from("BOT reset/rebind"),
                BlockError::Unsupported(why),
            )
        };
        if !self.buffer.present() || !self.wrapper.present() {
            return Err(fail("BOT transport DMA owners already transferred"));
        }
        let (old, old_setting) = self
            .attachment
            .as_ref()
            .ok_or_else(|| fail("BOT has no retained native attachment"))?;
        let interfaces = super::super::descriptor::interfaces(&old.configuration);
        if old.configuration.get(4) != Some(&1)
            || interfaces.is_empty()
            || interfaces.iter().any(|i| i.number != self.interface)
        {
            return Err(fail(
                "BOT reset requires exclusive ownership of a single-interface device",
            ));
        }
        let controller = self.io.controller.clone();
        // SAFETY: this sole interface owns both bulk endpoints and was
        // permanently poisoned before retiring the complete physical device.
        let mut fresh = unsafe {
            controller
                .borrow_mut()
                .reset_and_reuse_root(old, &self.endpoints, dma)?
        };
        let id = fresh.id;
        let result = (|| {
            if fresh.descriptor != old.descriptor
                || fresh.configurations != old.configurations
                || fresh.serial != old.serial
                || fresh.speed != old.speed
            {
                return Err(fail("USB device identity/descriptors changed across reset"));
            }
            fresh.configuration = fresh
                .configurations
                .iter()
                .find(|bytes| bytes.get(5) == Some(&old.configuration_value()))
                .cloned()
                .ok_or_else(|| fail("USB reset lost the selected configuration"))?;
            let setting = super::super::descriptor::interfaces(&fresh.configuration)
                .into_iter()
                .find(|i| {
                    i.number == self.interface
                        && i.alternate == old_setting.alternate
                        && i.class == super::CLASS
                        && i.subclass == super::SCSI
                        && i.protocol == super::BULK_ONLY
                })
                .ok_or_else(|| fail("USB reset lost the selected BOT alternate"))?;
            controller
                .borrow_mut()
                .device(id)
                .and_then(|mut device| device.set_configuration(fresh.configuration_value()))
                .map_err(|e| {
                    (
                        alloc::string::String::from("BOT reset restore configuration"),
                        e,
                    )
                })?;
            if setting.alternate != 0 {
                controller
                    .borrow_mut()
                    .device(id)
                    .and_then(|mut device| {
                        device.control(
                            setup(
                                1,
                                super::super::descriptor::SET_INTERFACE,
                                u16::from(setting.alternate),
                                u16::from(setting.number),
                                0,
                            ),
                            &[],
                        )
                    })
                    .map_err(|e| {
                        (
                            alloc::string::String::from("BOT reset restore alternate"),
                            e,
                        )
                    })?;
            }
            controller
                .borrow_mut()
                .device(id)
                .and_then(|mut device| device.configure(&self.endpoints, None, dma))
                .map_err(|e| {
                    (
                        alloc::string::String::from("BOT reset reconstruct endpoints"),
                        e,
                    )
                })?;
            let flags = quirks::Flags::from_device(&fresh).map_err(fail)?;
            let mut io = Native {
                controller: controller.clone(),
                id,
                endpoints: self.endpoints,
                command_us: self.io.command_us.clone(),
            };
            let max_lun = Self::startup(&mut io, setting.number, flags).map_err(|e| {
                (
                    alloc::string::String::from("BOT reset reconstruct transport"),
                    e,
                )
            })?;
            io.ready().map_err(|e| {
                (
                    alloc::string::String::from("BOT reset final transport readiness"),
                    e,
                )
            })?;
            // Every fallible operation has completed. Whole-device retirement
            // acknowledged all old endpoint DMA, and fresh configuration and
            // endpoint commands have acknowledged the replacement. No request
            // in the new transport refers to either application buffer yet.
            // Move the unique owners; old aliases stay poisoned and
            // cannot inspect or submit these addresses again.
            let mut buffer = self.buffer.transfer();
            let mut wrapper = self.wrapper.transfer();
            buffer.bytes_mut().fill(0);
            wrapper.bytes_mut().fill(0);
            let mut bot = Self::with_buffers(
                io,
                setting.number,
                self.endpoints,
                buffer,
                wrapper,
                flags,
                max_lun,
            );
            bot.attachment = Some((fresh, setting));
            Ok(bot)
        })();
        if result.is_err()
            && let Ok(mut device) = controller.borrow_mut().device(id)
        {
            device.quarantine();
        }
        result
    }
}

fn cbw(tag: u32, lun: u8, cdb: &[u8], length: usize, is_in: bool) -> [u8; 31] {
    let mut bytes = [0; 31];
    bytes[..4].copy_from_slice(&CBW_SIGNATURE.to_le_bytes());
    bytes[4..8].copy_from_slice(&tag.to_le_bytes());
    bytes[8..12].copy_from_slice(&(length as u32).to_le_bytes());
    bytes[12] = if is_in { 0x80 } else { 0 };
    bytes[13] = lun;
    bytes[14] = cdb.len() as u8;
    bytes[15..15 + cdb.len()].copy_from_slice(cdb);
    bytes
}
#[cfg(test)]
mod tests;
