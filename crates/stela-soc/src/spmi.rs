//! Apple SPMI Gen1 to Gen4 controller (`aapl,spmi`, source-selected `gen`), a MIPI SPMI bus
//! master and the interrupt controller of its slaves.
//!
//! The offsets below are Gen1's and Gen2's. Gen3 keeps them with its fault
//! latches and counters elsewhere in bank B and an arbiter bank in `reg[2]`;
//! Gen4 moves bank A, the status bits and the fault FIFO and adds a FIFO
//! depth register. `config` documents both from their handlers.
//!
//! Evidence (26A434 KC VAs, unslid): `AppleGen2SPMIHandler::initHandler`
//! 0xfffffe0009dc757c loads the register offsets; `fifoQueuesEmpty`
//! 0xfffffe0009dcb4fc, `queuePop` 0xfffffe0009dcbe64,
//! `extendedReadWriteCommand` 0xfffffe0009dcfec0..0xfffffe0009dd000c and
//! `resetQueueBase` 0xfffffe0009dceb50 use them. Bank A is `reg[0]`, bank B
//! `reg[1]`; both are 32-bit registers.
//!
//! - A+0x00 STATUS: bit 24 response FIFO empty, bit 25 response FIFO full,
//!   bit 8 request FIFO empty. Queue depths are byte counts, not word counts.
//! - A+0x04 request FIFO: a header word, then a write's payload words.
//! - A+0x08 response FIFO: one word per read.
//! - A+0x20 + 4g / A+0x60 + 4g: interrupt enable / write-1-clear status of
//!   vector group g (0..=8), bit `vector & 31`.
//! - A+0xa0 control: both generations save, zero and restore it. The Gen1
//!   declaration has zero control; other control-field effects refuse.
//! - A+0xa4 queue reset: writing 1 flushes both FIFOs; `resetQueueBase`
//!   then requires STATUS to read exactly 0x01000100.
//! - B+0 status and B+8 pop belong to the independent fault FIFO. Its clean
//!   handoff is empty (bit24), and an empty pop refuses. Gen2 has B+0x110..11c
//!   fault latches and 17 counters at B+0x200..224/B+0x25c..274. Gen1 uses
//!   16 exact offsets in a separate reg[2]; see `config`.
//!
//! A header is `address << 16 | last << 15 | sid << 8 | opcode`, the opcode
//! carrying the byte count minus one: extended write 0x00 and read 0x20
//! (16-byte, 8-bit address), extended-long write 0x30 and read 0x38 (8-byte,
//! 16-bit address), the MIPI SPMI opcodes. The driver pops one response per
//! request chunk and requires its low twelve bits to echo the request
//! (`getHeaderMatch` masks 0xfff). A write response's bit 15 is the slave's
//! ACK (NACK fails 0xe00002ed). A read response carries one parity-good bit
//! per byte at bits 16 and up, then the data in little-endian words; a NACKed
//! read is modeled with its parity bits clear and zero data, which the driver
//! reports as a parity failure (0xe00002bc). The last chunk raises local
//! configured response vector (DT interrupts[0]: Gen1 J314s192, Gen2
//! J473256), which the driver polls or takes through the AIC line.
//!
//! Scalar Register Read/Write and Register0 Write use one request word.
//! Original 26A434 J613/T8122 `kernelcache.release.mac15g` SHA256
//! 8210f3645ea16c2481bcaf285ead56eaf2e591bb0d124fe2c01f596c6361ded3.
//! `registerReadWriteCommand` at VA9d5c768
//! validates a five-bit address and one byte, then packs address at 20:16
//! and the write byte at 31:24 (9d5c9a0..9d5c9f0). Opcodes 0x40/0x60
//! come from its table at 772ff48. `performPayloadlessTransaction` packs
//! Register0 Write opcode 0x80 and its seven-bit byte at 31:24
//! (9d5d07c..9d5d0b4). The normal response FIFO owns the scalar read byte.
//!
//! Stela handoff: idle, empty FIFOs, every vector masked and clear, no
//! faults. The model executes a command when its last payload word arrives.
//! It has no faults to latch or count, so those registers read zero.
//!
//! Lifecycle pauses require no partial request and every vector
//! masked. Masked responses/pending latches and attached PMIC state remain
//! owned; physical reset/rail loss and armed-source wake behavior refuse.
//! Quiesce permits nonmutating reads, while a clock-stop MMIO access remains
//! unchecked. Slave autonomous time is independent of controller gating.
//! Armed autonomous capture through clock/power loss remains an explicit board
//! requirement; attaching a slave does not claim that wire. The debug queue
//! windows (A+0xb0..0xb8; Gen3 A+0xbc; Gen4 A+0x800..0x810), the Gen3 and
//! Gen4 arbiter reset (`reg[2]` + 0x20, which the driver pulses only when a
//! queue reset leaves the FIFOs non-empty, never here), nonzero control on
//! every generation but Gen2, command types outside the extended/scalar register
//! and four payloadless forms, unchecked slave command effects, and request
//! or response overflow refuse.
use crate::Result;
use stela_contracts::Refusal;
use stela_contracts::refuse;

#[cfg(test)]
use stela_soc_data::spmi::BANK_B;
#[cfg(test)]
use stela_soc_data::spmi::BANK_BYTES;
#[cfg(test)]
use stela_soc_data::spmi::BANK_C;
use stela_soc_data::spmi::{Configuration, Profile};

use alloc::{boxed::Box, collections::VecDeque, vec::Vec};
use {
    stela_contracts::bus::Access, stela_contracts::bus::Direction, stela_contracts::bus::Lifecycle,
    stela_contracts::bus::LifecycleState, stela_contracts::bus::MmioDevice,
    stela_contracts::bus::MmioError, stela_contracts::bus::Width, stela_contracts::bus::refuse,
    stela_contracts::bus::refuse_advance, stela_contracts::interrupt::AicInput,
    stela_contracts::interrupt::Wire,
};

const NAME: &str = "SPMI";
const GROUPS: usize = 9;
const LAST: u32 = 1 << 15;
const ACK: u32 = 1 << 15;
const REGISTER_WRITE: u32 = 0x40;
const REGISTER_READ: u32 = 0x60;
const REGISTER_ZERO_WRITE: u32 = 0x80;
const REGISTER_ADDRESS_MASK: u32 = 0x1f;
const REGISTER_ZERO_DATA_MASK: u8 = 0x7f;

/// A slave's refusal to complete a transaction.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum SlaveError {
    /// The slave answers NACK, a bus result the driver reports.
    Nack,
    /// The transaction needs behavior the model does not have; the machine
    /// stops with this reason.
    Unsupported(Refusal),
}

/// Payloadless MIPI commands recovered from the original AP command table
/// (26A434 mac15g VA772ff48[8..12]). Dispatch is not an implementation of
/// the attached slave's reset, supply or low-power effects.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[repr(u8)]
pub enum Command {
    Reset = 0x10,
    Sleep = 0x11,
    Shutdown = 0x12,
    Wakeup = 0x13,
}

/// One SPMI slave on a controller, addressed by its slave ID. Accesses
/// cover consecutive register addresses. A failed access changes nothing.
pub trait SpmiSlave {
    fn read(&mut self, address: u16, data: &mut [u8], now: u64) -> Result<(), SlaveError>;
    fn write(&mut self, address: u16, data: &[u8], now: u64) -> Result<(), SlaveError>;
    /// Execute a slave-owned command. Unqualified effects must not ACK.
    fn command(&mut self, _command: Command, _now: u64) -> Result<(), SlaveError> {
        Err(SlaveError::Unsupported(refuse!(
            "SPMI slave payloadless command is unqualified"
        )))
    }
    /// Bring time-driven state up to `now`; the next counter value at which
    /// the slave changes state by itself, if any.
    /// Validate an autonomous time step before any attached slave advances.
    /// Callers must complete this preflight before calling `advance`.
    fn check_time(&self, _now: u64) -> Result<(), SlaveError> {
        Ok(())
    }
    /// Slave output. None means this slave has no checked output contract.
    fn interrupt_level(&self) -> Option<bool> {
        None
    }
    /// Indexed output, in the slave's source-declared order. The legacy
    /// single-output boundary is output zero; other wires require an owner.
    fn interrupt_level_at(&self, output: u8) -> Option<bool> {
        if output == 0 {
            self.interrupt_level()
        } else {
            None
        }
    }
    /// Could an autonomous output change while the master is paused? Unknown
    /// slaves conservatively keep ownership; a masked local vector is irrelevant.
    fn interrupt_armed(&self) -> bool {
        true
    }
    fn advance(&mut self, _now: u64) -> Option<u64> {
        None
    }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
enum Kind {
    Write,
    Read,
    Command(Command),
}

/// A decoded request chunk.
#[derive(Clone, Copy, Debug)]
struct Chunk {
    header: u32,
    kind: Kind,
    sid: u8,
    address: u16,
    bytes: usize,
    /// Scalar writes carry their byte in the request's high byte, not a
    /// following FIFO word (26A434 mac15g AppleSPMI, VA9d5c9c8/9d5d0a8).
    inline: Option<u8>,
}

impl Chunk {
    fn decode(header: u32) -> Result<Self> {
        let opcode = header & 0xff;
        if header & 0x7000 != 0 {
            return Err(refuse!("SPMI request header reserved bits"));
        }
        let (kind, address, bytes, inline) = match opcode {
            REGISTER_WRITE | REGISTER_READ => {
                let address = (header >> 16) & 0xff;
                if address > REGISTER_ADDRESS_MASK || (opcode == REGISTER_READ && header >> 24 != 0)
                {
                    return Err(refuse!(
                        "SPMI scalar register access has reserved address/data bits"
                    ));
                }
                (
                    if opcode == REGISTER_READ {
                        Kind::Read
                    } else {
                        Kind::Write
                    },
                    address as u16,
                    1,
                    (opcode == REGISTER_WRITE).then_some((header >> 24) as u8),
                )
            }
            0x10..=0x13 => {
                if header >> 16 != 0 {
                    return Err(refuse!(
                        "SPMI payloadless command carries address or data bits"
                    ));
                }
                let command = match opcode {
                    0x10 => Command::Reset,
                    0x11 => Command::Sleep,
                    0x12 => Command::Shutdown,
                    0x13 => Command::Wakeup,
                    _ => unreachable!(),
                };
                (Kind::Command(command), 0, 0, None)
            }
            REGISTER_ZERO_WRITE => {
                let data = (header >> 24) as u8;
                if header & 0x00ff_0000 != 0 || data & !REGISTER_ZERO_DATA_MASK != 0 {
                    return Err(refuse!(
                        "SPMI register-zero write has reserved address/data bits"
                    ));
                }
                (Kind::Write, 0, 1, Some(data))
            }
            _ => {
                let address = (header >> 16) as u16;
                let (kind, bytes, long) = match opcode {
                    0x00..=0x0f => (Kind::Write, (opcode & 0xf) + 1, false),
                    0x20..=0x2f => (Kind::Read, (opcode & 0xf) + 1, false),
                    0x30..=0x37 => (Kind::Write, (opcode & 7) + 1, true),
                    0x38..=0x3f => (Kind::Read, (opcode & 7) + 1, true),
                    _ => {
                        return Err(refuse!(
                            "SPMI command other than a qualified register access"
                        ));
                    }
                };
                if !long && address > 0xff {
                    return Err(refuse!("SPMI extended access with a 16-bit address"));
                }
                (kind, address, bytes as usize, None)
            }
        };
        Ok(Self {
            header,
            kind,
            sid: ((header >> 8) & 0xf) as u8,
            address,
            bytes,
            inline,
        })
    }
    /// The AP matches scalar replies with mask 0xfe0 and register-zero
    /// replies with mask 0xf80 (26A434 mac15g table VA772ff5c). The low
    /// bits carry the MIPI command's five-bit address or seven-bit data.
    fn response_header(&self) -> u32 {
        let opcode = self.header & 0xff;
        match opcode {
            REGISTER_WRITE | REGISTER_READ => {
                (self.header & 0xf00) | opcode | u32::from(self.address)
            }
            REGISTER_ZERO_WRITE => {
                (self.header & 0xf00) | opcode | u32::from(self.inline.expect("scalar write"))
            }
            _ => self.header & 0xfff,
        }
    }
    fn words(&self) -> usize {
        self.bytes.div_ceil(4)
    }
}

pub struct Spmi {
    configuration: Configuration,
    lifecycle: LifecycleState,
    irq: AicInput,
    slaves: Vec<(u8, Box<dyn SpmiSlave>)>,
    inputs: Vec<(u8, u8, u16, bool)>,
    enable: [u32; GROUPS],
    pending: [u32; GROUPS],
    control: u32,
    /// A write chunk waiting for its payload words.
    partial: Option<(Chunk, Vec<u32>)>,
    responses: VecDeque<u32>,
}

impl Spmi {
    #[cfg(test)]
    pub fn new(irq: AicInput) -> Self {
        Self::with_configuration(irq, Configuration::GEN2)
    }
    pub fn with_configuration(irq: AicInput, configuration: Configuration) -> Self {
        Self {
            configuration,
            lifecycle: LifecycleState {
                powered: true,
                clocked: true,
                quiesced: false,
                reset: false,
                retention: true,
            },
            irq,
            slaves: Vec::new(),
            inputs: Vec::new(),
            enable: [0; GROUPS],
            pending: [0; GROUPS],
            control: 0,
            partial: None,
            responses: VecDeque::new(),
        }
    }
    /// Attaches a slave at its four-bit ID; an unoccupied ID NACKs.
    pub fn attach(&mut self, sid: u8, slave: Box<dyn SpmiSlave>) -> Result<()> {
        if sid > 0xf || self.slaves.iter().any(|(id, _)| *id == sid) {
            return Err(refuse!("invalid or duplicate SPMI slave ID"));
        }
        self.slaves.push((sid, slave));
        Ok(())
    }
    /// Attach a deasserted slave output to a source-declared edge vector.
    /// This samples the initial level; attaching cannot manufacture an edge.
    pub fn attach_with_interrupt(
        &mut self,
        sid: u8,
        vector: u16,
        slave: Box<dyn SpmiSlave>,
    ) -> Result<()> {
        self.attach_with_interrupts(sid, &[(0, vector)], slave)
    }
    /// Attach every source-declared output atomically. Output indices select
    /// independent slave levels; vector order never invents semantics.
    pub fn attach_with_interrupts(
        &mut self,
        sid: u8,
        outputs: &[(u8, u16)],
        slave: Box<dyn SpmiSlave>,
    ) -> Result<()> {
        if outputs.is_empty() {
            return Err(refuse!("SPMI interrupt attachment has no outputs"));
        }
        for (index, &(output, vector)) in outputs.iter().enumerate() {
            if usize::from(vector) >= GROUPS * 32
                || u32::from(vector) == self.configuration.response_vector()
                || self.inputs.iter().any(|(_, _, input, _)| *input == vector)
                || outputs[..index]
                    .iter()
                    .any(|&(previous, input)| previous == output || input == vector)
            {
                return Err(refuse!(
                    "invalid, response-aliasing or duplicate SPMI input vector/output"
                ));
            }
            if slave.interrupt_level_at(output) != Some(false) {
                return Err(refuse!(
                    "SPMI input requires an actual qualified deasserted output"
                ));
            }
        }
        self.attach(sid, slave)?;
        self.inputs.extend(
            outputs
                .iter()
                .map(|&(output, vector)| (sid, output, vector, false)),
        );
        Ok(())
    }
    fn capture_inputs(&mut self) {
        for (sid, output, vector, previous) in &mut self.inputs {
            let slave = &self
                .slaves
                .iter()
                .find(|(id, _)| id == sid)
                .expect("attached input owner")
                .1;
            let level = slave
                .interrupt_level_at(*output)
                .expect("qualified input owner");
            if level && !*previous {
                self.pending[usize::from(*vector / 32)] |= 1 << (*vector % 32);
            }
            *previous = level;
        }
    }
    fn status(&self) -> u32 {
        let layout = self.configuration.profile().layout();
        (if self.responses.is_empty() {
            layout.response_empty
        } else {
            0
        }) | (if self.responses.len() == self.configuration.response_words() {
            layout.response_full
        } else {
            0
        }) | if self.partial.is_none() {
            layout.request_empty
        } else {
            0
        }
    }
    fn update_irq(&self, access: &mut Access<'_>) {
        let asserted = self
            .enable
            .iter()
            .zip(&self.pending)
            .any(|(e, p)| e & p != 0);
        access.interrupts.set_level(Wire::Aic(self.irq), asserted);
    }
    fn request(&mut self, word: u32, now: u64) -> Result<()> {
        if let Some((chunk, payload)) = &self.partial {
            let mut words = payload.clone();
            words.push(word);
            if words.len() < chunk.words() {
                self.partial.as_mut().expect("borrowed partial").1 = words;
                return Ok(());
            }
            let chunk = *chunk;
            // A refused final word is not consumed. In particular a full
            // response FIFO or unsupported slave cannot discard its earlier
            // accepted payload or repeat a completed slave effect on retry.
            self.execute(chunk, &words, now)?;
            self.partial = None;
            Ok(())
        } else {
            let chunk = Chunk::decode(word)?;
            if chunk.kind == Kind::Write && chunk.inline.is_none() {
                self.partial = Some((chunk, Vec::with_capacity(chunk.words())));
                Ok(())
            } else {
                self.execute(chunk, &[], now)
            }
        }
    }
    fn execute(&mut self, chunk: Chunk, payload: &[u32], now: u64) -> Result<()> {
        let words = 1 + if chunk.kind == Kind::Read {
            chunk.words()
        } else {
            0
        };
        if self.responses.len() + words > self.configuration.response_words() {
            return Err(refuse!("SPMI response FIFO overflow"));
        }
        let slave = self
            .slaves
            .iter_mut()
            .find(|(id, _)| *id == chunk.sid)
            .map(|(_, slave)| slave);
        let echo = chunk.response_header();
        match chunk.kind {
            Kind::Write => {
                let bytes: Vec<u8> = match chunk.inline {
                    Some(byte) => alloc::vec![byte],
                    None => payload
                        .iter()
                        .flat_map(|word| word.to_le_bytes())
                        .take(chunk.bytes)
                        .collect(),
                };
                let result = match slave {
                    Some(slave) => slave.write(chunk.address, &bytes, now),
                    None => Err(SlaveError::Nack),
                };
                match result {
                    Ok(()) => self.responses.push_back(echo | ACK),
                    Err(SlaveError::Nack) => self.responses.push_back(echo),
                    Err(SlaveError::Unsupported(reason)) => return Err(reason),
                }
            }
            Kind::Command(command) => {
                let result = match slave {
                    Some(slave) => slave.command(command, now),
                    None => Err(SlaveError::Nack),
                };
                match result {
                    Ok(()) => self.responses.push_back(echo | ACK),
                    Err(SlaveError::Nack) => self.responses.push_back(echo),
                    Err(SlaveError::Unsupported(reason)) => return Err(reason),
                }
            }
            Kind::Read => {
                let mut bytes = [0u8; 16];
                let data = &mut bytes[..chunk.bytes];
                let result = match slave {
                    Some(slave) => slave.read(chunk.address, data, now),
                    None => Err(SlaveError::Nack),
                };
                let parity = match result {
                    Ok(()) => ((1u32 << chunk.bytes) - 1) << 16 | ACK,
                    Err(SlaveError::Nack) => {
                        data.fill(0);
                        0
                    }
                    Err(SlaveError::Unsupported(reason)) => return Err(reason),
                };
                self.responses.push_back(echo | parity);
                for word in data.chunks(4) {
                    let mut le = [0u8; 4];
                    le[..word.len()].copy_from_slice(word);
                    self.responses.push_back(u32::from_le_bytes(le));
                }
            }
        }
        self.capture_inputs();
        if chunk.header & LAST != 0 {
            let vector = self.configuration.response_vector();
            self.pending[(vector >> 5) as usize] |= 1 << (vector & 31);
        }
        Ok(())
    }
}

impl Lifecycle for Spmi {
    fn check_lifecycle(&self, proposed: LifecycleState) -> Result<()> {
        if !proposed.powered || proposed.reset {
            return Err(refuse!(
                "SPMI physical reset/supply retention is unqualified; queue reset is a separate software operation"
            ));
        }
        if !proposed.clocked || proposed.quiesced {
            if self.inputs.iter().any(|(sid, _, _, _)| {
                self.slaves
                    .iter()
                    .any(|(id, slave)| id == sid && slave.interrupt_armed())
            }) {
                return Err(refuse!(
                    "SPMI pause with an autonomous routed slave has unqualified capture/wake semantics"
                ));
            }
            if self.partial.is_some() {
                return Err(refuse!(
                    "SPMI pause while a partial request owns the transaction is unqualified"
                ));
            }
            if self.enable.iter().any(|&enabled| enabled != 0) {
                return Err(refuse!(
                    "SPMI pause with an armed interrupt has unqualified wake/capture semantics"
                ));
            }
        }
        Ok(())
    }
    fn set_lifecycle(&mut self, proposed: LifecycleState, _: &mut Access<'_>) {
        self.lifecycle = proposed;
    }
}

impl MmioDevice for Spmi {
    fn lifecycle(&mut self) -> Option<&mut dyn Lifecycle> {
        Some(self)
    }
    fn name(&self) -> &'static str {
        NAME
    }
    fn read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError> {
        let err = |reason: Refusal| refuse(NAME, offset, width, Direction::Read, reason);
        if !self.lifecycle.clocked {
            return Err(err(refuse!(
                "SPMI MMIO while the physical clock is stopped is unqualified"
            )));
        }
        let layout = self.configuration.profile().layout();
        if self.lifecycle.quiesced && offset == layout.response {
            return Err(err(refuse!(
                "SPMI response ownership cannot advance while quiesced"
            )));
        }
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(err(refuse!(
                "SPMI registers require aligned 32-bit accesses"
            )));
        }
        let value = match offset {
            at if at == layout.status => self.status(),
            at if at == layout.response => {
                let word = self
                    .responses
                    .pop_front()
                    .ok_or_else(|| err(refuse!("SPMI response FIFO read while empty")))?;
                self.update_irq(access);
                word
            }
            at if (layout.enable..layout.enable + 4 * GROUPS as u64).contains(&at) => {
                self.enable[((at - layout.enable) / 4) as usize]
            }
            at if (layout.pending..layout.pending + 4 * GROUPS as u64).contains(&at) => {
                self.pending[((at - layout.pending) / 4) as usize]
            }
            at if at == layout.control => self.control,
            at if at == layout.queue_reset => 0,
            at if Some(at) == layout.depth => self.configuration.depth_register(),
            at if at == layout.fault_status => layout.fault_empty,
            at if at == layout.fault_response => {
                return Err(err(refuse!("SPMI fault FIFO read while empty")));
            }
            at if layout
                .latches
                .is_some_and(|(start, end)| (start..end).contains(&at)) =>
            {
                0
            }
            at if self.configuration.profile().counter(at) => 0,
            _ => return Err(err(refuse!("unmodeled SPMI register"))),
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
        let err = |reason: Refusal| refuse(NAME, offset, width, Direction::Write, reason);
        if !self.lifecycle.clocked || self.lifecycle.quiesced {
            return Err(err(refuse!("SPMI controller transactions are paused")));
        }
        if width != Width::Word || !offset.is_multiple_of(4) {
            return Err(err(refuse!(
                "SPMI registers require aligned 32-bit accesses"
            )));
        }
        let value = value as u32;
        let profile = self.configuration.profile();
        let layout = profile.layout();
        match offset {
            at if at == layout.request => self.request(value, access.now).map_err(err)?,
            at if (layout.enable..layout.enable + 4 * GROUPS as u64).contains(&at) => {
                self.enable[((at - layout.enable) / 4) as usize] = value;
            }
            at if (layout.pending..layout.pending + 4 * GROUPS as u64).contains(&at) => {
                self.pending[((at - layout.pending) / 4) as usize] &= !value;
            }
            at if at == layout.control => {
                // Gen2's driver saves, zeroes and restores control; the
                // other generations' fields have no recovered effects.
                if profile != Profile::Gen2 && value != 0 {
                    return Err(err(refuse!(
                        "SPMI nonzero control-field effects are unqualified"
                    )));
                }
                self.control = value;
            }
            at if at == layout.queue_reset => {
                if value != 1 {
                    return Err(err(refuse!("SPMI queue reset other than bit 0")));
                }
                self.partial = None;
                self.responses.clear();
            }
            // Read-then-write-back acknowledges latched faults; none latch.
            at if layout
                .latches
                .is_some_and(|(start, end)| (start..end).contains(&at)) =>
            {
                if value != 0 {
                    return Err(err(refuse!(
                        "SPMI fault latch acknowledge of an unlatched fault"
                    )));
                }
            }
            _ => return Err(err(refuse!("unmodeled SPMI register"))),
        }
        self.update_irq(access);
        Ok(())
    }
    fn advance(&mut self, access: &mut Access<'_>) -> Result<Option<u64>, MmioError> {
        // Slaves such as the PMIC own independent clocks, rails and timers.
        // Gating the SPMI master cannot suspend their autonomous time.
        // A refusal here is no guest transfer: the slave's own clock
        // cannot be at the count the board advanced to.
        for (_, slave) in &self.slaves {
            slave.check_time(access.now).map_err(|error| {
                refuse_advance(
                    NAME,
                    access.now,
                    match error {
                        SlaveError::Unsupported(reason) => reason,
                        SlaveError::Nack => refuse!("SPMI autonomous slave refused time"),
                    },
                )
            })?;
        }
        let deadline = self
            .slaves
            .iter_mut()
            .filter_map(|(_, slave)| slave.advance(access.now))
            .min();
        self.capture_inputs();
        self.update_irq(access);
        Ok(deadline)
    }
}

#[cfg(test)]
mod tests;
