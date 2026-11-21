// SPDX-License-Identifier: GPL-2.0-or-later
//
// Linux DesignWare I2C master FIFO and abort handling adapted to a polled Rust
// controller at a804c5742bff224d4cb4d25dd383554d598be32b:
// drivers/i2c/busses/i2c-designware-master.c
// SHA-256 e73845fed46c45702ada39d6dfb30ed7c48c71990cf4e68e20456b57f55bb9ae
// drivers/i2c/busses/i2c-designware-core.h
// SHA-256 31cc6f6e90ea42adfdcf2b695e105f479a3a9c7be64d48484d805448070d1626
//
// Synopsys DesignWare I2C adapter driver (master only).
// Based on the TI DAVINCI I2C adapter driver.
// Copyright (C) 2006 Texas Instruments.
// Copyright (C) 2007 MontaVista Software Inc.
// Copyright (C) 2009 Provigent Ltd.
//
//! Register-level master engine. Tests run this same engine against a controller
//! model, not a replacement implementation of the bus transaction API.
use super::{
    AbortKind, Address, Bus, Clock, Error, ErrorKind, IoError, Operation, Phase, Recovery, Timer,
};
use crate::bus::mmio::Mmio;
use alloc::boxed::Box;
use core::mem::ManuallyDrop;
mod ownership;
pub use ownership::{Access, AccessResult, Failure, Progress};

const CON: usize = 0x00;
const TAR: usize = 0x04;
const DATA_CMD: usize = 0x10;
const SS_SCL_HCNT: usize = 0x14;
const SS_SCL_LCNT: usize = 0x18;
const FS_SCL_HCNT: usize = 0x1c;
const FS_SCL_LCNT: usize = 0x20;
const INTR_MASK: usize = 0x30;
const RAW_INTR_STAT: usize = 0x34;
const RX_TL: usize = 0x38;
const TX_TL: usize = 0x3c;
const CLR_INTR: usize = 0x40;
const CLR_TX_ABRT: usize = 0x54;
const CLR_STOP_DET: usize = 0x60;
const ENABLE: usize = 0x6c;
const STATUS: usize = 0x70;
const TXFLR: usize = 0x74;
const RXFLR: usize = 0x78;
const SDA_HOLD: usize = 0x7c;
const TX_ABRT_SOURCE: usize = 0x80;
const DMA_CR: usize = 0x88;
const SMBUS_INTR_MASK: usize = 0xcc;
const ENABLE_STATUS: usize = 0x9c;
const COMP_PARAM_1: usize = 0xf4;
const COMP_VERSION: usize = 0xf8;
const COMP_TYPE: usize = 0xfc;

const COMPONENT_TYPE: u32 = 0x4457_0140;
const SDA_HOLD_VERSION: u32 = 0x3131_312a;
const PARAM_ENCODED: u32 = 1 << 7;
const PARAM_WIDTH_32: u32 = 2;
const CON_MASTER: u32 = 1;
const CON_STANDARD: u32 = 1 << 1;
const CON_FAST: u32 = 2 << 1;
const CON_TEN_BIT: u32 = 1 << 4;
const CON_RESTART: u32 = 1 << 5;
const CON_SLAVE_DISABLE: u32 = 1 << 6;
const TAR_TEN_BIT: u32 = 1 << 12;
const CMD_READ: u32 = 1 << 8;
const CMD_STOP: u32 = 1 << 9;
const CMD_RESTART: u32 = 1 << 10;
const ENABLE_ON: u32 = 1;
const ENABLE_ABORT: u32 = 1 << 1;
const STATUS_ACTIVE: u32 = 1;
const STATUS_TX_EMPTY: u32 = 1 << 2;
const STATUS_MASTER_ACTIVE: u32 = 1 << 5;
const INTR_RX_UNDER: u32 = 1;
const INTR_RX_OVER: u32 = 1 << 1;
const INTR_TX_OVER: u32 = 1 << 3;
const INTR_TX_ABRT: u32 = 1 << 6;
const INTR_STOP_DET: u32 = 1 << 9;
const INTR_FIFO_ERROR: u32 = INTR_RX_UNDER | INTR_RX_OVER | INTR_TX_OVER;
const ABRT_ADDRESS_NACK: u32 = 0b111;
const ABRT_DATA_NACK: u32 = 1 << 3;
const ABRT_ARBITRATION_LOST: u32 = 1 << 12;

/// Exclusive access to a conventional DesignWare register block. An
/// implementation must preserve ordered, volatile 32-bit register semantics.
/// The trait is public so independent models can exercise the driver.
pub trait Registers {
    fn read32(&mut self, offset: usize) -> Result<u32, IoError>;
    fn write32(&mut self, offset: usize, value: u32) -> Result<(), IoError>;
}

/// A uniquely owned, identity-mapped, uncached DesignWare register window.
pub struct MmioRegisters(Mmio);

impl MmioRegisters {
    /// # Safety
    ///
    /// `base..base + 0x100` must be mapped uncached, aligned to four bytes,
    /// owned exclusively for this object's lifetime, and backed by a powered
    /// DesignWare controller. The platform must provide clocks and reset first.
    /// Neither firmware, an IRQ handler nor another CPU may access the window.
    pub unsafe fn new(base: u64) -> Self {
        // SAFETY: covered by this function's own safety contract.
        Self(unsafe { Mmio::new(base, 0x100) })
    }
}

impl Registers for MmioRegisters {
    fn read32(&mut self, offset: usize) -> Result<u32, IoError> {
        Ok(self.0.read32(offset))
    }

    fn write32(&mut self, offset: usize, value: u32) -> Result<(), IoError> {
        self.0.write32(offset, value);
        Ok(())
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Speed {
    Standard,
    Fast,
}

impl Speed {
    fn control(self) -> u32 {
        match self {
            Self::Standard => CON_STANDARD,
            Self::Fast => CON_FAST,
        }
    }
}

/// Register counts already calculated or read from the platform firmware for
/// the controller input clock and the board's rise/fall times. This is not a
/// bus frequency in Hz; guessing counts can violate a peripheral's timing.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Timing {
    pub speed: Speed,
    pub scl_high: u16,
    pub scl_low: u16,
    /// SDA_HOLD's TX count (low 16 bits) and RX count (bits 16–23). `None`
    /// keeps the firmware value; programming requires core version >= 1.11*.
    pub sda_hold: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Config {
    pub timing: Timing,
    /// Total bound for the transaction, including bus acquisition and STOP.
    pub timeout_us: u64,
    /// Additional bound for disable/abort on initialization or error cleanup.
    pub recovery_timeout_us: u64,
}

/// Master-mode engine using PIO FIFOs, with interrupts and DMA disabled.
/// Standard/fast mode and 7-/10-bit unicast addressing are implemented.
/// High-speed mode, target mode, SMBus quick/PEC and GPIO bus clear are not.
pub struct DesignWare<R = MmioRegisters, T = Clock> {
    state: ManuallyDrop<Box<State<R, T>>>,
}

struct State<R, T> {
    registers: R,
    timer: T,
    config: Config,
    tx_depth: usize,
    rx_depth: usize,
    control: u32,
    ready: bool,
    progress: Progress,
    active: bool,
    failure: Option<Error>,
    inflight_error: Option<Error>,
}

impl<R: Registers, T: Timer> State<R, T> {
    fn initialize(&mut self) -> Result<(), Error> {
        let config = self.config;
        let result = (|| {
            if config.timeout_us == 0
                || config.recovery_timeout_us == 0
                || self.timer.check_duration(config.timeout_us).is_err()
                || self
                    .timer
                    .check_duration(config.recovery_timeout_us)
                    .is_err()
                || config.timing.scl_high < 6
                || config.timing.scl_low < 8
                || config
                    .timing
                    .sda_hold
                    .is_some_and(|hold| hold > 0x00ff_ffff)
            {
                return Err(ErrorKind::InvalidConfiguration);
            }
            let component = self.read(COMP_TYPE)?;
            if component != COMPONENT_TYPE {
                return Err(ErrorKind::ComponentType(component));
            }
            let parameters = self.read(COMP_PARAM_1)?;
            self.tx_depth = (((parameters >> 16) & 0xff) + 1) as usize;
            self.rx_depth = (((parameters >> 8) & 0xff) + 1) as usize;
            let max_speed = (parameters >> 2) & 3;
            if parameters & PARAM_ENCODED == 0
                || parameters & 3 != PARAM_WIDTH_32
                || self.tx_depth < 2
                || self.rx_depth < 2
                || max_speed == 0
                || (config.timing.speed == Speed::Fast && max_speed < 2)
            {
                return Err(ErrorKind::ComponentParameters(parameters));
            }
            if config.timing.sda_hold.is_some() {
                let version = self.read(COMP_VERSION)?;
                if version < SDA_HOLD_VERSION {
                    return Err(ErrorKind::UnsupportedVersion(version));
                }
            }
            self.control =
                CON_MASTER | CON_RESTART | CON_SLAVE_DISABLE | config.timing.speed.control();
            if !self.quiesce()? {
                return Err(ErrorKind::Timeout(Phase::Disable));
            }
            self.write(INTR_MASK, 0)?;
            self.write(SMBUS_INTR_MASK, 0)?;
            self.write(DMA_CR, 0)?;
            self.write(CON, self.control)?;
            self.write(RX_TL, 0)?;
            self.write(TX_TL, 0)?;
            let (high, low) = match config.timing.speed {
                Speed::Standard => (SS_SCL_HCNT, SS_SCL_LCNT),
                Speed::Fast => (FS_SCL_HCNT, FS_SCL_LCNT),
            };
            self.write(high, u32::from(config.timing.scl_high))?;
            self.write(low, u32::from(config.timing.scl_low))?;
            if let Some(hold) = config.timing.sda_hold {
                self.write(SDA_HOLD, hold)?;
            }
            if self.read(high)? != u32::from(config.timing.scl_high)
                || self.read(low)? != u32::from(config.timing.scl_low)
            {
                return Err(ErrorKind::TimingRejected);
            }
            if let Some(hold) = config.timing.sda_hold
                && self.read(SDA_HOLD)? != hold
            {
                return Err(ErrorKind::TimingRejected);
            }
            self.ready = true;
            Ok(())
        })();
        result.map_err(|kind| {
            self.failure.unwrap_or_else(|| Error {
                kind,
                recovery: if kind == ErrorKind::Timeout(Phase::Disable) {
                    Recovery::Failed
                } else {
                    Recovery::NotAttempted
                },
            })
        })
    }

    fn expired(&self, start: u64, timeout: u64) -> bool {
        self.timer.elapsed_us(start) >= timeout
    }

    fn wait_until(
        &mut self,
        start: u64,
        timeout: u64,
        mut ready: impl FnMut(&mut Self) -> Result<bool, ErrorKind>,
    ) -> Result<bool, ErrorKind> {
        loop {
            let observed = ready(self)?;
            if self.expired(start, timeout) {
                return Ok(false);
            }
            if observed {
                return Ok(true);
            }
            self.timer.relax();
        }
    }

    /// Abort releases a master holding SCL on an empty TX FIFO; disable then
    /// flushes any unread FIFO entries. Never spin forever on wedged hardware.
    fn quiesce(&mut self) -> Result<bool, ErrorKind> {
        let start = self.timer.start();
        if self.read(ENABLE_STATUS)? & ENABLE_ON != 0
            && self.read(STATUS)? & STATUS_MASTER_ACTIVE != 0
        {
            self.write(ENABLE, ENABLE_ON | ENABLE_ABORT)?;
            let _aborted = self.wait_until(start, self.config.recovery_timeout_us, |state| {
                Ok(state.read(ENABLE)? & ENABLE_ABORT == 0)
            })?;
        }
        self.write(ENABLE, 0)?;
        let mut disabled = self.wait_until(start, self.config.recovery_timeout_us, |state| {
            Ok(state.read(ENABLE_STATUS)? & ENABLE_ON == 0)
        })?;
        if disabled {
            self.write(INTR_MASK, 0)?;
            // Firmware may leave IC_SMBUS enabled independently of the
            // ordinary interrupt mask. No SMBus interrupt is serviced here.
            self.write(SMBUS_INTR_MASK, 0)?;
            self.write(DMA_CR, 0)?;
            self.read(CLR_INTR)?;
            disabled = !self.expired(start, self.config.recovery_timeout_us);
        }
        self.ready = disabled;
        Ok(disabled)
    }

    fn faults(&mut self) -> Result<u32, ErrorKind> {
        let interrupts = self.read(RAW_INTR_STAT)?;
        if interrupts & INTR_TX_ABRT != 0 {
            // CLR_INTR/CLR_TX_ABRT also erase the reason. Capture it first.
            let source = self.read(TX_ABRT_SOURCE)?;
            let kind = if source & ABRT_ARBITRATION_LOST != 0 {
                AbortKind::ArbitrationLost
            } else if source & ABRT_ADDRESS_NACK != 0 {
                AbortKind::AddressNack
            } else if source & ABRT_DATA_NACK != 0 {
                AbortKind::DataNack
            } else {
                AbortKind::Other
            };
            let primary = ErrorKind::Aborted { kind, source };
            self.inflight_error = Some(Error {
                kind: primary,
                recovery: Recovery::Interrupted,
            });
            self.read(CLR_TX_ABRT)?;
            self.inflight_error = None;
            return Err(primary);
        }
        if interrupts & INTR_FIFO_ERROR != 0 {
            return Err(ErrorKind::Fifo { interrupts });
        }
        Ok(interrupts)
    }

    fn transaction_deadline(&self, start: u64, phase: Phase) -> Result<(), ErrorKind> {
        if self.expired(start, self.config.timeout_us) {
            Err(ErrorKind::Timeout(phase))
        } else {
            Ok(())
        }
    }

    fn transfer(
        &mut self,
        address: Address,
        operations: &mut [Operation<'_>],
    ) -> Result<(), ErrorKind> {
        let start = self.timer.start();
        if !self.wait_until(start, self.config.timeout_us, |state| {
            Ok(state.read(STATUS)? & STATUS_ACTIVE == 0)
        })? {
            return Err(ErrorKind::Timeout(Phase::BusIdle));
        }
        self.read(CLR_INTR)?;
        let (target, address_control) = match address {
            Address::SevenBit(address) => (u32::from(address), 0),
            Address::TenBit(address) => (u32::from(address) | TAR_TEN_BIT, CON_TEN_BIT),
        };
        self.write(CON, self.control | address_control)?;
        self.write(TAR, target)?;
        self.transaction_deadline(start, Phase::Enable)?;
        self.write(ENABLE, ENABLE_ON)?;
        if !self.wait_until(start, self.config.timeout_us, |state| {
            Ok(state.read(ENABLE_STATUS)? & ENABLE_ON != 0)
        })? {
            return Err(ErrorKind::Timeout(Phase::Enable));
        }

        let final_message = operations.len() - 1;
        for (message, operation) in operations.iter_mut().enumerate() {
            let (mut queued, mut received) = (0, 0);
            let count = operation.len();
            loop {
                let interrupts = self.faults()?;
                self.transaction_deadline(start, Phase::Transfer)?;
                if interrupts & INTR_STOP_DET != 0 && message == 0 && queued == 0 {
                    // No command is on the wire yet, so this STOP is not ours
                    // (seen on an AMD I2C touchpad right after enable). Drop it.
                    self.read(CLR_STOP_DET)?;
                    continue;
                }
                if interrupts & INTR_STOP_DET != 0 {
                    // A STOP cannot belong to our final byte while more
                    // commands remain in the hardware's TX FIFO. Merely
                    // having queued every message does not prove completion.
                    if message != final_message || queued != count || self.read(TXFLR)? != 0 {
                        return Err(ErrorKind::UnexpectedStop);
                    }
                }
                let available = self.read(RXFLR)? as usize;
                self.transaction_deadline(start, Phase::Transfer)?;
                if available > self.rx_depth {
                    return Err(ErrorKind::UnexpectedReceive);
                }
                if available != 0 {
                    match operation {
                        Operation::Read(bytes) if available <= queued - received => {
                            for byte in &mut bytes[received..received + available] {
                                let observed = self.read(DATA_CMD)? as u8;
                                self.transaction_deadline(start, Phase::Transfer)?;
                                *byte = observed;
                            }
                            received += available;
                        }
                        _ => return Err(ErrorKind::UnexpectedReceive),
                    }
                }
                let pending = self.read(TXFLR)? as usize;
                if pending > self.tx_depth {
                    return Err(ErrorKind::Fifo {
                        interrupts: INTR_TX_OVER,
                    });
                }
                let room = self.tx_depth - pending;
                let is_read = matches!(operation, Operation::Read(_));
                // Every queued read will eventually occupy RX FIFO space,
                // including requests still in the TX FIFO or on the wire.
                let read_room = if is_read {
                    self.rx_depth - (queued - received)
                } else {
                    usize::MAX
                };
                let issue = (count - queued).min(room).min(read_room);
                for _ in 0..issue {
                    let mut command = match operation {
                        Operation::Write(bytes) => u32::from(bytes[queued]),
                        Operation::Read(_) => CMD_READ,
                    };
                    if message != 0 && queued == 0 {
                        command |= CMD_RESTART;
                    }
                    if message == final_message && queued == count - 1 {
                        command |= CMD_STOP;
                    }
                    // No target/status/payload observation may spend the
                    // remaining budget and still permit a later wire command.
                    self.transaction_deadline(start, Phase::Transfer)?;
                    self.write(DATA_CMD, command)?;
                    queued += 1;
                }
                if queued == count && (!is_read || received == count) {
                    break;
                }
                if self.expired(start, self.config.timeout_us) {
                    return Err(ErrorKind::Timeout(Phase::Transfer));
                }
                self.timer.relax();
            }
        }
        loop {
            let interrupts = self.faults()?;
            if interrupts & INTR_STOP_DET != 0 && self.read(TXFLR)? != 0 {
                return Err(ErrorKind::UnexpectedStop);
            }
            let status = self.read(STATUS)?;
            self.transaction_deadline(start, Phase::Stop)?;
            if interrupts & INTR_STOP_DET != 0
                && status & (STATUS_TX_EMPTY | STATUS_ACTIVE) == STATUS_TX_EMPTY
            {
                let remaining = self.read(RXFLR)?;
                self.transaction_deadline(start, Phase::Stop)?;
                if remaining != 0 {
                    return Err(ErrorKind::UnexpectedReceive);
                }
                self.read(CLR_STOP_DET)?;
                self.transaction_deadline(start, Phase::Stop)?;
                return Ok(());
            }
            if self.expired(start, self.config.timeout_us) {
                return Err(ErrorKind::Timeout(Phase::Stop));
            }
            self.timer.relax();
        }
    }
}

#[cfg(test)]
mod tests;
