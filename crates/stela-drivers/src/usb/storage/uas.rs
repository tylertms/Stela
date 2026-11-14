//! Native USB Attached SCSI command/data/status transport.
//!
//! USB2 uses READ/WRITE_READY handshakes; SuperSpeed uses the canonical
//! xHCI primary stream service and opaque transfer tickets. The same serialized
//! CDB owner keeps every contributor until completion or acknowledged drain.
// SPDX-License-Identifier: GPL-2.0-only
// Linux uas.c reference: Hans de Goede/Red Hat 2013-2016;
// Matthew Wilcox/Intel Corp 2010; Sarah Sharp/Intel Corp 2010.
use super::{Data, Device, DmaAllocator, Endpoint, MAX_TD, Shared, copy_read};
use crate::{
    block::{BlockError, Sense},
    bus::dma::DmaBuffer,
};
use alloc::vec::Vec;

mod abort;
mod binding;
mod quirks;
pub use abort::{AbortCommandResult, AbortTaskOutcome};
pub use binding::Binding;

/// Shared with alternate selection; readiness still requires allocation.
pub const REQUESTED_STREAMS: u16 = 2;

const COMMAND_IU: u8 = 1;
const STATUS_IU: u8 = 3;
const RESPONSE_IU: u8 = 4;
const READ_READY_IU: u8 = 6;
const WRITE_READY_IU: u8 = 7;
const TIMEOUT_US: u64 = 30_000_000;
const IU_BUFFER_BYTES: usize = 512;
const MAX_CDB: usize = 260;
const PROTOCOL_ERROR: BlockError = BlockError::Protocol("malformed UAS information unit");

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
enum Pipe {
    Command,
    Status,
    Input,
    Output,
}
impl Pipe {
    fn index(self) -> usize {
        self as usize
    }
}

/// Opaque eight-byte SCSI logical-unit address, kept exactly as reported.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq, PartialOrd, Ord)]
pub struct Lun([u8; 8]);
impl Lun {
    pub fn from_bytes(bytes: [u8; 8]) -> Self {
        Self(bytes)
    }
    pub(super) fn sequential(number: u16) -> Self {
        let mut bytes = [0; 8];
        bytes[0..2].copy_from_slice(&number.to_be_bytes());
        Self(bytes)
    }
    pub fn bytes(self) -> [u8; 8] {
        self.0
    }
}
impl core::fmt::Display for Lun {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        for byte in self.0 {
            write!(f, "{byte:02x}")?;
        }
        Ok(())
    }
}

/// The class owner keeps buffers allocated for the entire boot, including
/// uncertain DMA. A failed command quarantines this complete interface; host
/// TD cancellation alone cannot establish that the device retired its UAS tag.
pub struct Host {
    engine: Engine<Native>,
    flags: quirks::Flags,
    device: Device,
    binding: Binding,
    minimum_error: Option<BlockError>,
}
impl Host {
    /// Whether the host controller sends the handoff's completion signal.
    pub fn signals(&self) -> bool {
        self.engine.io.signals()
    }

    /// Whether a running transfer may have completed; a look at memory.
    pub fn posted(&self) -> bool {
        self.engine.io.posted()
    }

    /// # Safety
    /// `device`, binding and controller must name the same physical attachment
    /// and configuration/reset epoch. The caller owns the interface exclusively
    /// and has not configured or submitted to any of these endpoint contexts.
    /// This constructor owns SET_INTERFACE and endpoint/stream setup.
    /// No firmware/other class owner may drive the interface.
    pub unsafe fn new(
        controller: Shared,
        device: &Device,
        binding: Binding,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, BlockError> {
        let io = Self::configure_native(controller, device, &binding, dma)?;
        let engine = Engine::new(io, dma)?;
        Ok(Self {
            engine,
            flags: binding.flags,
            device: device.clone(),
            binding,
            minimum_error: None,
        })
    }
    /// Establish the alternate and endpoint/stream contexts before
    /// acquiring or transferring any transport DMA owner.
    fn configure_native(
        controller: Shared,
        device: &Device,
        binding: &Binding,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Native, BlockError> {
        use super::super::xhci::{FULL_SPEED, HIGH_SPEED, SUPER_SPEED, SUPER_SPEED_PLUS};
        if !matches!(
            device.speed,
            FULL_SPEED | HIGH_SPEED | SUPER_SPEED | SUPER_SPEED_PLUS
        ) {
            return Err(BlockError::Unsupported("UAS device speed"));
        }
        if controller
            .borrow_mut()
            .device(device.id)?
            .current_configuration()?
            != Some(binding.configuration_value())
        {
            return Err(BlockError::Unsupported(
                "UAS configuration is not acknowledged",
            ));
        }
        if binding.alternate() != 0 {
            controller.borrow_mut().device(device.id)?.control(
                super::setup(
                    0x01,
                    super::super::descriptor::SET_INTERFACE,
                    u16::from(binding.alternate()),
                    u16::from(binding.interface()),
                    0,
                ),
                &[],
            )?;
        }
        let endpoints = binding.endpoints();
        let streams = if matches!(device.speed, SUPER_SPEED | SUPER_SPEED_PLUS) {
            // This is the real allocation/configuration ACK, not a capability
            // advertisement or a caller-supplied claim that streams exist.
            Some(
                controller
                    .borrow_mut()
                    .device(device.id)?
                    .configure_streams(
                        &endpoints,
                        &binding.stream_endpoints(),
                        REQUESTED_STREAMS,
                        dma,
                    )?,
            )
        } else {
            controller
                .borrow_mut()
                .device(device.id)?
                .configure(&endpoints, None, dma)?;
            None
        };
        Ok(Native {
            controller,
            id: device.id,
            endpoints,
            streams,
        })
    }
    /// Permanently retire this owner and construct a new owner after a
    /// whole-device USB reset. Existing Rc disk aliases never become usable.
    /// Keep the three transport allocations; root-leaf reset also reuses
    /// retired controller resources. Downstream reset keeps its existing
    /// controller-allocation behavior until subtree reuse is supported.
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    pub(super) fn reset_and_rebind(
        &mut self,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, (alloc::string::String, BlockError)> {
        let controller = self.engine.io.controller.clone();
        let port = controller.borrow().root_port_of(self.device.id);
        self.reset_and_rebind_once(dma)
            .map_err(|(operation, error)| {
                // The port as the failed step left it.
                match port.map(|port| controller.borrow_mut().root_port_status(port)) {
                    Some(status) => (alloc::format!("{operation} ({status})"), error),
                    None => (operation, error),
                }
            })
    }
    #[expect(
        clippy::result_large_err,
        reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
    )]
    fn reset_and_rebind_once(
        &mut self,
        dma: &mut dyn DmaAllocator,
    ) -> Result<Self, (alloc::string::String, BlockError)> {
        self.engine.poisoned = true;
        let fail = |why| {
            (
                alloc::string::String::from("UAS reset/rebind"),
                BlockError::Unsupported(why),
            )
        };
        if !self.engine.buffers_present() {
            return Err(fail("UAS transport DMA owners already transferred"));
        }
        let interfaces = super::super::descriptor::interfaces(&self.device.configuration);
        if self.device.configuration.get(4) != Some(&1)
            || interfaces.is_empty()
            || interfaces
                .iter()
                .any(|interface| interface.number != self.binding.interface())
        {
            return Err(fail(
                "UAS reset requires exclusive ownership of a single-interface device",
            ));
        }
        let controller = self.engine.io.controller.clone();
        // SAFETY: this sole interface owns exactly all configured class DCIs,
        // and its shared owner was permanently poisoned before retirement.
        let mut fresh = unsafe {
            controller.borrow_mut().reset_and_reuse_root(
                &self.device,
                &self.binding.endpoints(),
                dma,
            )?
        };
        let id = fresh.id;
        let result = (|| {
            if fresh.descriptor != self.device.descriptor
                || fresh.configurations != self.device.configurations
                || fresh.serial != self.device.serial
                || fresh.speed != self.device.speed
            {
                return Err(fail("USB device identity/descriptors changed across reset"));
            }
            fresh.configuration = fresh
                .configurations
                .iter()
                .find(|bytes| bytes.get(5) == Some(&self.binding.configuration_value()))
                .cloned()
                .ok_or_else(|| fail("USB reset lost the selected configuration"))?;
            let interface = super::super::descriptor::interfaces(&fresh.configuration)
                .into_iter()
                .find(|interface| {
                    interface.number == self.binding.interface()
                        && interface.alternate == self.binding.alternate()
                })
                .ok_or_else(|| fail("USB reset lost the selected UAS alternate"))?;
            let binding = Binding::new(&fresh, &interface).map_err(fail)?;
            controller
                .borrow_mut()
                .device(id)
                .and_then(|mut device| device.set_configuration(fresh.configuration_value()))
                .map_err(|error| {
                    (
                        alloc::string::String::from("UAS reset restore configuration"),
                        error,
                    )
                })?;
            let mut io = Self::configure_native(controller.clone(), &fresh, &binding, dma)
                .map_err(|error| {
                    (
                        alloc::string::String::from("UAS reset reconstruct endpoints"),
                        error,
                    )
                })?;
            if io.streams() == Some(0) {
                return Err(fail("UAS has no usable stream IDs"));
            }
            io.ready().map_err(|error| {
                (
                    alloc::string::String::from("UAS reset final transport readiness"),
                    error,
                )
            })?;
            // Whole-device retirement and every fallible reconstruction
            // step precede transferring any of the three unique allocations.
            // Old aliases keep their permanently poisoned, empty engine.
            let mut command = self.engine.command.transfer();
            let mut status = self.engine.status.transfer();
            let mut data = self.engine.data.transfer();
            command.bytes_mut().fill(0);
            status.bytes_mut().fill(0);
            data.bytes_mut().fill(0);
            Ok(Self {
                engine: Engine {
                    io,
                    command,
                    status,
                    data,
                    pending: core::array::from_fn(|_| None),
                    touched: [false; 4],
                    tag: 0,
                    poisoned: false,
                    active: None,
                    failed_data: None,
                },
                flags: binding.flags,
                device: fresh,
                binding,
                minimum_error: None,
            })
        })();
        if result.is_err()
            && let Ok(mut device) = controller.borrow_mut().device(id)
        {
            device.quarantine();
        }
        result
    }
    pub(super) fn device(&self) -> &Device {
        &self.device
    }
    pub(super) fn interface(&self) -> u8 {
        self.binding.interface()
    }
    pub(super) fn quarantine(&mut self) {
        self.engine.poisoned = true;
        let mut controller = self.engine.io.controller.borrow_mut();
        if let Ok(mut device) = controller.device(self.engine.io.id) {
            device.quarantine();
        }
    }

    /// Execute a CDB against an eight-byte SCSI LUN address. A check
    /// condition uses the returned Sense IU, not an invented REQUEST SENSE.
    pub fn command(
        &mut self,
        lun: [u8; 8],
        cdb: &[u8],
        data: Data<'_>,
    ) -> Result<Vec<u8>, BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        execute(&mut self.engine, self.flags, lun, cdb, data)
    }

    pub(super) fn read_into(
        &mut self,
        lun: [u8; 8],
        cdb: &[u8],
        output: &mut [u8],
    ) -> Result<(), BlockError> {
        let data = Data::In(output.len());
        admit(self.flags, cdb, &data)?;
        self.engine.read_into(lun, cdb, output)
    }

    /// Begin a CDB as [`Self::command`] runs it, without waiting for any
    /// of its transfers; [`Self::end`] gives its result.
    pub(super) fn begin(
        &mut self,
        lun: [u8; 8],
        cdb: &[u8],
        data: Data<'_>,
    ) -> Result<(), BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        admit(self.flags, cdb, &data)?;
        self.engine.begin(lun, cdb, data)
    }
    /// The begun command's result once it ended; never waits.
    pub(super) fn end(&mut self) -> Option<Result<Vec<u8>, BlockError>> {
        if let Some(error) = &self.minimum_error {
            return self.engine.active.as_ref().map(|_| Err(error.clone()));
        }
        self.engine.poll(None, &mut None)
    }
    /// Whether a begun command has not ended.
    pub(super) fn busy(&self) -> bool {
        self.engine.active.is_some()
    }

    pub(super) fn delay(&mut self, micros: u64) -> Result<(), BlockError> {
        if let Some(error) = &self.minimum_error {
            return Err(error.clone());
        }
        let result = (|| {
            let mut controller = self.engine.io.controller.borrow_mut();
            controller.device(self.device.id)?.connected()?;
            controller.minimum_wait(micros)
        })();
        if let Err(error) = &result {
            self.minimum_error = Some(error.clone());
            // Do not cancel/reuse tags or take pending contributors after a
            // failed physical source. Native owner/DMA custody remains held.
            self.engine.poisoned = true;
        }
        result
    }
    pub(super) fn now_us(&self) -> u64 {
        self.engine.io.now()
    }
    /// Linux `uas_sdev_configure`: an ALWAYS_SYNC enclosure skips both mode
    /// pages and is assumed writable with its write cache on.
    pub(super) fn write_protect_page(&self) -> Option<u8> {
        (!self.flags.always_sync()).then_some(super::scsi::ALL_PAGES)
    }
    /// Linux `uas_sdev_configure` sets `broken_fua` for a BROKEN_FUA
    /// enclosure, which then takes no FUA write.
    pub(super) fn fua_permitted(&self) -> bool {
        self.flags.permits(&super::scsi::write_10_fua())
    }
    pub(super) fn caching(&self) -> super::Caching {
        match self.flags.always_sync() {
            true => super::Caching::WriteBack,
            false => super::Caching::Query,
        }
    }
    /// `uas_sdev_configure` sets `read_before_ms` for every UAS unit. Some
    /// bridges otherwise return generic write-protect/cache mode information.
    /// This metadata read is never a retry of a caller's failed block command.
    pub(super) fn read_before_mode_sense(
        &mut self,
        lun: [u8; 8],
        block_size: usize,
    ) -> Result<(), BlockError> {
        read_block_zero(&mut self.engine, self.flags, lun, block_size)
    }
    pub(super) fn max_transfer_bytes(&self) -> usize {
        self.flags.max_transfer_bytes()
    }
    pub(super) fn report_luns_permitted(&self) -> bool {
        self.flags.report_luns_permitted()
    }
    pub fn requires_rebind(&self) -> bool {
        self.engine.poisoned
    }
    /// Whether [`Self::reset_and_rebind`] needs fresh rings from its
    /// allocator, as a device behind a hub does.
    pub(super) fn reset_needs_allocation(&self) -> bool {
        self.engine
            .io
            .controller
            .try_borrow()
            .is_ok_and(|controller| controller.reset_needs_allocation(self.device.id))
    }
    /// What the last command's data phase moved in before it failed.
    pub(super) fn take_failed_data(&mut self) -> Option<Vec<u8>> {
        self.engine.failed_data.take()
    }
}

fn execute<I: Io>(
    engine: &mut Engine<I>,
    flags: quirks::Flags,
    lun: [u8; 8],
    cdb: &[u8],
    data: Data<'_>,
) -> Result<Vec<u8>, BlockError> {
    admit(flags, cdb, &data)?;
    engine.command(lun, cdb, data)
}

/// The enclosure's quirks take `cdb` and its transfer.
fn admit(flags: quirks::Flags, cdb: &[u8], data: &Data<'_>) -> Result<(), BlockError> {
    if !flags.permits(cdb) {
        return Err(BlockError::Unsupported(
            "CDB requires an unsupported enclosure operation",
        ));
    }
    let length = match data {
        Data::None => 0,
        Data::In(bytes) => *bytes,
        Data::Out(bytes) => bytes.len(),
    };
    if length > flags.max_transfer_bytes() {
        return Err(BlockError::Unsupported("UAS enclosure transfer limit"));
    }
    Ok(())
}

/// Linux `sd_read_block_zero` uses READ(10) when the ordinary block path uses
/// ten-byte CDBs. Stela propagates failure rather than querying mode pages after
/// uncertain transport/media change. No retry, write, or data publication occurs.
fn read_block_zero<I: Io>(
    engine: &mut Engine<I>,
    flags: quirks::Flags,
    lun: [u8; 8],
    block_size: usize,
) -> Result<(), BlockError> {
    if block_size < 512 || !block_size.is_power_of_two() || block_size > MAX_TD {
        return Err(BlockError::Unsupported("UAS block-zero probe geometry"));
    }
    let mut cdb = [0; 10];
    cdb[0] = super::scsi::READ_10;
    cdb[8] = 1;
    let bytes = execute(engine, flags, lun, &cdb, Data::In(block_size))?;
    if bytes.len() != block_size {
        return Err(BlockError::Protocol("short UAS block-zero probe"));
    }
    Ok(())
}

trait Io {
    type Ticket;
    fn ready(&mut self) -> Result<(), BlockError>;
    /// Whether the host controller sends the handoff's completion signal.
    fn signals(&self) -> bool {
        false
    }
    /// Whether a submitted transfer may have completed; a look at memory.
    fn posted(&self) -> bool {
        false
    }
    fn streams(&self) -> Option<u16> {
        None
    }
    fn submit(
        &mut self,
        pipe: Pipe,
        tag: u16,
        address: u64,
        length: usize,
    ) -> Result<Self::Ticket, BlockError>;
    fn completed(&mut self, ticket: &Self::Ticket) -> Option<Result<usize, BlockError>>;
    fn cancel(&mut self, pipe: Pipe) -> Result<(), BlockError>;
    fn now(&self) -> u64;
    fn pause(&self);
}
struct Native {
    controller: Shared,
    id: super::super::xhci::SlotId,
    endpoints: [Endpoint; 4],
    streams: Option<u16>,
}
enum NativeTicket {
    Ordinary { pipe: Pipe, trb: u64, length: usize },
    Stream(super::super::xhci::StreamTransfer),
}
impl Io for Native {
    type Ticket = NativeTicket;
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
        let [command, status, input, output] = self.endpoints;
        self.controller
            .borrow_mut()
            .device(self.id)?
            .endpoints_ready(&[1, command.dci(), status.dci(), input.dci(), output.dci()])
    }
    fn streams(&self) -> Option<u16> {
        self.streams
    }
    fn submit(
        &mut self,
        pipe: Pipe,
        tag: u16,
        address: u64,
        length: usize,
    ) -> Result<NativeTicket, BlockError> {
        let dci = self.endpoints[pipe.index()].dci();
        let mut controller = self.controller.borrow_mut();
        let mut device = controller.device(self.id)?;
        if self.streams.is_some() && pipe != Pipe::Command {
            device
                .submit_stream(dci, tag, address, length)
                .map(NativeTicket::Stream)
        } else {
            device
                .submit(dci, address, length)
                .map(|trb| NativeTicket::Ordinary { pipe, trb, length })
        }
    }
    fn completed(&mut self, ticket: &NativeTicket) -> Option<Result<usize, BlockError>> {
        let mut controller = self.controller.borrow_mut();
        let mut device = match controller.device(self.id) {
            Ok(device) => device,
            Err(error) => return Some(Err(error)),
        };
        match ticket {
            NativeTicket::Stream(ticket) => device.completed_stream(ticket),
            NativeTicket::Ordinary { pipe, trb, length } => {
                device.completed(self.endpoints[pipe.index()].dci(), *trb, *length)
            }
        }
    }
    fn cancel(&mut self, pipe: Pipe) -> Result<(), BlockError> {
        self.controller
            .borrow_mut()
            .device(self.id)?
            .cancel(self.endpoints[pipe.index()].dci())
    }
    fn now(&self) -> u64 {
        self.controller.borrow().clock.now_us()
    }
    fn pause(&self) {
        let clock = self.controller.borrow().clock;
        clock.delay(1);
    }
}

/// A unique allocation can move only after acknowledged device retirement.
/// Empty slots belong to permanently poisoned old engines, never dummy buffers.
struct TransportBuffer(Option<DmaBuffer>);
impl TransportBuffer {
    fn new(buffer: DmaBuffer) -> Self {
        Self(Some(buffer))
    }
    fn present(&self) -> bool {
        self.0.is_some()
    }
    fn transfer(&mut self) -> Self {
        Self(Some(self.0.take().expect("retained UAS DMA owner")))
    }
}
impl core::ops::Deref for TransportBuffer {
    type Target = DmaBuffer;
    fn deref(&self) -> &DmaBuffer {
        self.0.as_ref().expect("retained UAS DMA owner")
    }
}
impl core::ops::DerefMut for TransportBuffer {
    fn deref_mut(&mut self) -> &mut DmaBuffer {
        self.0.as_mut().expect("retained UAS DMA owner")
    }
}

struct Engine<I: Io> {
    io: I,
    command: TransportBuffer,
    status: TransportBuffer,
    data: TransportBuffer,
    pending: [Option<I::Ticket>; 4],
    touched: [bool; 4],
    tag: u16,
    /// The begun command, until [`Engine::poll`] returns its result.
    active: Option<Active>,
    poisoned: bool,
    /// Input data that completed before the Sense IU failed its command,
    /// until the next command: sd keeps it when the sense is RECOVERED
    /// ERROR.
    failed_data: Option<Vec<u8>>,
}
impl<I: Io> Engine<I> {
    fn new(mut io: I, dma: &mut dyn DmaAllocator) -> Result<Self, BlockError> {
        if io.streams() == Some(0) {
            return Err(BlockError::Unsupported("UAS has no usable stream IDs"));
        }
        io.ready()?;
        Ok(Self {
            io,
            command: TransportBuffer::new(dma.allocate(IU_BUFFER_BYTES, IU_BUFFER_BYTES)?),
            status: TransportBuffer::new(dma.allocate(IU_BUFFER_BYTES, IU_BUFFER_BYTES)?),
            data: TransportBuffer::new(dma.allocate(MAX_TD, MAX_TD)?),
            pending: core::array::from_fn(|_| None),
            touched: [false; 4],
            tag: 0,
            active: None,
            poisoned: false,
            failed_data: None,
        })
    }
    fn buffers_present(&self) -> bool {
        self.command.present() && self.status.present() && self.data.present()
    }
    fn post(&mut self, pipe: Pipe, address: u64, length: usize) -> Result<(), BlockError> {
        let index = pipe.index();
        if self.pending[index].is_some() {
            return Err(BlockError::Unsupported(
                "UAS pipe already has a pending transfer",
            ));
        }
        // A failed submit may have published a TD before losing its receipt.
        // Include it in terminal cancellation even without a returned ticket.
        self.touched[index] = true;
        self.pending[index] = Some(self.io.submit(pipe, self.tag, address, length)?);
        Ok(())
    }
    fn failure<T>(&mut self, error: BlockError) -> Result<T, BlockError> {
        self.poisoned = true;
        for pipe in [Pipe::Command, Pipe::Status, Pipe::Input, Pipe::Output] {
            if self.touched[pipe.index()] {
                // Keep the owner and all allocations even on cancellation
                // failure. There is no successful-looking tag reuse afterward.
                let _ = self.io.cancel(pipe);
            }
        }
        Err(error)
    }
    fn command(&mut self, lun: [u8; 8], cdb: &[u8], data: Data<'_>) -> Result<Vec<u8>, BlockError> {
        self.command_inner(lun, cdb, data, None, &mut None)
    }
    fn command_inner(
        &mut self,
        lun: [u8; 8],
        cdb: &[u8],
        data: Data<'_>,
        abort_timeout: Option<u64>,
        abort: &mut Option<AbortTaskOutcome>,
    ) -> Result<Vec<u8>, BlockError> {
        let received = self.wait(lun, cdb, data, abort_timeout, abort)?;
        Ok(self.received_data(received).to_vec())
    }
    fn read_into(&mut self, lun: [u8; 8], cdb: &[u8], output: &mut [u8]) -> Result<(), BlockError> {
        // The caller's slice stays on the stack. Only a retired command
        // with successful status can give the CPU its received data.
        let received = self.wait(lun, cdb, Data::In(output.len()), None, &mut None)?;
        copy_read(output, self.received_data(received))
    }
    fn received_data(&self, received: Option<usize>) -> &[u8] {
        received.map_or(&[], |bytes| &self.data.bytes()[..bytes])
    }
    fn wait(
        &mut self,
        lun: [u8; 8],
        cdb: &[u8],
        data: Data<'_>,
        abort_timeout: Option<u64>,
        abort: &mut Option<AbortTaskOutcome>,
    ) -> Result<Option<usize>, BlockError> {
        self.begin(lun, cdb, data)?;
        loop {
            if let Some(result) = self.poll_completion(abort_timeout, abort) {
                return result;
            }
            self.io.pause();
        }
    }
    /// Post `cdb`'s status, data and command requests without waiting for
    /// any of them; [`Self::poll`] moves the command on.
    fn begin(&mut self, lun: [u8; 8], cdb: &[u8], data: Data<'_>) -> Result<(), BlockError> {
        if self.poisoned || !self.buffers_present() {
            return Err(BlockError::Unsupported("UAS interface requires rebind"));
        }
        if self.active.is_some() {
            return Err(BlockError::Protocol("a UAS command is already running"));
        }
        let (length, input) = match &data {
            Data::None => (0, false),
            Data::In(n) => (*n, true),
            Data::Out(b) => (b.len(), false),
        };
        if cdb.is_empty() || cdb.len() > MAX_CDB || length > MAX_TD {
            return Err(BlockError::Unsupported("UAS CDB or transfer length"));
        }
        // Readiness can fail because topology retirement, a controller fault,
        // or an earlier endpoint error invalidated this attachment. Fence the
        // entire class owner before media code observes the failure. No DMA
        // buffer may be rewritten and no later command may reuse this epoch.
        self.touched = [false; 4];
        if let Err(error) = self.io.ready() {
            return self.failure(error);
        }
        let streams = self.io.streams();
        let maximum = streams.unwrap_or(u16::MAX);
        self.tag = if self.tag >= maximum { 1 } else { self.tag + 1 };
        let command = command_iu(self.tag, lun, cdb);
        self.command.bytes_mut()[..command.len()].copy_from_slice(&command);
        self.status.bytes_mut().fill(0);
        if let Data::Out(bytes) = data {
            self.data.bytes_mut()[..length].copy_from_slice(bytes);
        }
        self.touched = [false; 4];
        if let Err(error) = self.post(Pipe::Status, self.status.physical(), IU_BUFFER_BYTES) {
            return self.failure(error);
        }
        // USB3 has no READY handshake. Queue the stream-owned status
        // and data requests before publishing the ordinary command IU.
        if streams.is_some()
            && length != 0
            && let Err(error) = self.post(
                if input { Pipe::Input } else { Pipe::Output },
                self.data.physical(),
                length,
            )
        {
            return self.failure(error);
        }
        if let Err(error) = self.post(Pipe::Command, self.command.physical(), command.len()) {
            return self.failure(error);
        }
        self.active = Some(Active {
            lun,
            command: command.len(),
            length,
            input,
            streams,
            started: self.io.now(),
            ready: false,
            moved: if length == 0 { Some(0) } else { None },
            terminal: None,
        });
        Ok(())
    }
    /// The begun command's result once it ended, failed or timed out; never
    /// waits. `abort_timeout` replaces the command timeout with one that
    /// aborts the task, as error handling's ABORT TASK does.
    fn poll(
        &mut self,
        abort_timeout: Option<u64>,
        abort: &mut Option<AbortTaskOutcome>,
    ) -> Option<Result<Vec<u8>, BlockError>> {
        self.poll_completion(abort_timeout, abort)
            .map(|result| result.map(|received| self.received_data(received).to_vec()))
    }
    /// The byte count of a retired successful input command. Writes and
    /// no-data commands have no input to expose.
    fn poll_completion(
        &mut self,
        abort_timeout: Option<u64>,
        abort: &mut Option<AbortTaskOutcome>,
    ) -> Option<Result<Option<usize>, BlockError>> {
        let mut active = self.active.take()?;
        let result = self.advance(&mut active, abort_timeout, abort);
        if result.is_none() {
            self.active = Some(active);
        }
        result
    }
    fn advance(
        &mut self,
        active: &mut Active,
        abort_timeout: Option<u64>,
        abort: &mut Option<AbortTaskOutcome>,
    ) -> Option<Result<Option<usize>, BlockError>> {
        let (length, input, streams) = (active.length, active.input, active.streams);
        for pipe in [Pipe::Command, Pipe::Status, Pipe::Input, Pipe::Output] {
            let index = pipe.index();
            let completion = self.pending[index]
                .as_ref()
                .and_then(|ticket| self.io.completed(ticket));
            let Some(completion) = completion else {
                continue;
            };
            self.pending[index] = None;
            let count = match completion {
                Ok(count) => count,
                Err(error) => return Some(self.failure(error)),
            };
            match pipe {
                Pipe::Command if count != active.command => {
                    return Some(self.failure(PROTOCOL_ERROR));
                }
                Pipe::Command => {}
                Pipe::Input | Pipe::Output if count <= length => active.moved = Some(count),
                Pipe::Input | Pipe::Output => {
                    return Some(self.failure(PROTOCOL_ERROR));
                }
                Pipe::Status => {
                    let frame = match self
                        .status
                        .bytes()
                        .get(..count)
                        .and_then(|bytes| status_iu(bytes, self.tag))
                    {
                        Some(frame) => frame,
                        None => {
                            return Some(self.failure(PROTOCOL_ERROR));
                        }
                    };
                    match frame {
                        Frame::Ready(direction)
                            if streams.is_none()
                                && !active.ready
                                && length != 0
                                && direction == input =>
                        {
                            active.ready = true;
                            // Repost the status request before data; a fast
                            // device may finish its CDB as soon as data ends.
                            if let Err(error) =
                                self.post(Pipe::Status, self.status.physical(), IU_BUFFER_BYTES)
                            {
                                return Some(self.failure(error));
                            }
                            if let Err(error) = self.post(
                                if input { Pipe::Input } else { Pipe::Output },
                                self.data.physical(),
                                length,
                            ) {
                                return Some(self.failure(error));
                            }
                        }
                        Frame::Ready(_) => {
                            return Some(self.failure(PROTOCOL_ERROR));
                        }
                        Frame::Done(status) => active.terminal = Some(status),
                    }
                }
            }
        }
        if let Some(status) = &active.terminal {
            if status.code != 0 {
                if input && self.pending[Pipe::Input.index()].is_none() {
                    self.failed_data = active
                        .moved
                        .map(|moved| self.data.bytes()[..moved].to_vec());
                }
                // Device-terminal error may precede host data completion.
                // Drain each outstanding contributor before any CPU
                // buffer access or later command is permitted.
                for pipe in [Pipe::Command, Pipe::Input, Pipe::Output] {
                    if self.pending[pipe.index()].is_some() {
                        if let Err(error) = self.io.cancel(pipe) {
                            return Some(self.failure(error));
                        }
                        self.pending[pipe.index()] = None;
                    }
                }
                return Some(Err(match status.response {
                    Some(_) => BlockError::Protocol("UAS Response IU refused the command"),
                    None => Sense::parse(&status.sense)
                        .map_or(BlockError::CheckCondition, BlockError::Sense),
                }));
            }
            if self.pending.iter().all(Option::is_none) {
                let Some(moved) = active.moved else {
                    return Some(self.failure(PROTOCOL_ERROR));
                };
                if !input && moved != length {
                    return Some(Err(BlockError::Protocol("short UAS write")));
                }
                return Some(Ok(input.then_some(moved)));
            }
        }
        if self.io.now().wrapping_sub(active.started) >= abort_timeout.unwrap_or(TIMEOUT_US) {
            if abort_timeout.is_some() {
                // This task and both IU buffers remain owned throughout
                // recovery. No later command may reuse this attachment.
                self.poisoned = true;
                *abort = Some(if active.terminal.is_some() {
                    AbortTaskOutcome::NotAcknowledged(BlockError::Unsupported(
                        "UAS task already returned terminal status",
                    ))
                } else {
                    self.abort_task(active.lun)
                });
                return Some(Err(BlockError::Timeout));
            }
            return Some(self.failure(BlockError::Timeout));
        }
        None
    }
}

/// A begun command: what [`Engine::poll`] has seen of it so far.
struct Active {
    lun: [u8; 8],
    /// The Command IU's bytes.
    command: usize,
    length: usize,
    input: bool,
    streams: Option<u16>,
    started: u64,
    /// The READ or WRITE READY handshake arrived.
    ready: bool,
    moved: Option<usize>,
    terminal: Option<Status>,
}

fn command_iu(tag: u16, lun: [u8; 8], cdb: &[u8]) -> Vec<u8> {
    let extra = cdb.len().saturating_sub(16).next_multiple_of(4);
    let mut bytes = vec![0; 32 + extra];
    bytes[0] = COMMAND_IU;
    bytes[2..4].copy_from_slice(&tag.to_be_bytes());
    bytes[6] = extra as u8;
    bytes[8..16].copy_from_slice(&lun);
    bytes[16..16 + cdb.len()].copy_from_slice(cdb);
    bytes
}
struct Status {
    code: u8,
    sense: Vec<u8>,
    response: Option<u8>,
}
enum Frame {
    Ready(bool),
    Done(Status),
}
fn status_iu(bytes: &[u8], tag: u16) -> Option<Frame> {
    if bytes.len() < 4 || u16::from_be_bytes(bytes[2..4].try_into().ok()?) != tag {
        return None;
    }
    match bytes[0] {
        READ_READY_IU if bytes.len() == 4 => Some(Frame::Ready(true)),
        WRITE_READY_IU if bytes.len() == 4 => Some(Frame::Ready(false)),
        STATUS_IU if bytes.len() >= 16 => {
            // Like Linux uas_sense, a sense length that disagrees with the
            // transfer is clamped to the bytes that arrived.
            let length = usize::from(u16::from_be_bytes(bytes[14..16].try_into().ok()?));
            let end = bytes.len().min(16 + length);
            Some(Frame::Done(Status {
                code: bytes[6],
                sense: bytes[16..end].to_vec(),
                response: None,
            }))
        }
        RESPONSE_IU if bytes.len() == 8 => Some(Frame::Done(Status {
            code: if bytes[7] == 8 { 0 } else { 0xff },
            sense: Vec::new(),
            response: Some(bytes[7]),
        })),
        _ => None,
    }
}

#[cfg(test)]
mod tests;
