// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! Register access for rtw89 chips over a PCI BAR window, with the masked
//! read-modify-write and polling helpers that rtw89 builds on
//! `rtw89_read*`/`rtw89_write*` and `read_poll_timeout`. Unlike rtw88's
//! `Io`, nothing here is tied to one chip.
use crate::{Clock, bus::mmio::Mmio};
use alloc::sync::Arc;
use core::{any::Any, fmt, ptr::NonNull};

/// Smallest window: the S8 indirect watchdog registers sit at 0x40000.
pub const MIN_WINDOW_BYTES: usize = 0x4_0004;

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum AxError {
    /// `mask` bits of `register` never reached the wanted value in time.
    Timeout {
        register: u32,
        mask: u32,
    },
    /// The register read all ones: the device is gone or not responding.
    AllOnes {
        register: u32,
    },
    /// Unaligned, past the window, or the window is unusable or retired.
    OutOfWindow {
        register: u32,
    },
    /// Uncalibrated, retired or non-advancing time source.
    Clock,
    /// The firmware download refused or failed; the message says why.
    Firmware(&'static str),
    /// The WCPU never reached the download state waited on: `what` names the
    /// state, and `fw_ctrl` is the whole `R_AX_WCPU_FW_CTRL` (0x01E0) read
    /// once the wait expired, as `rtw89_fw_dl_fail_dump` (`fw.c:1943`)
    /// logs it.
    FirmwareStatus {
        what: &'static str,
        fw_ctrl: u32,
    },
    /// The firmware file itself is missing or unusable; the message names it.
    FirmwareFile(&'static str),
    /// The PCI function is not the chip this bring-up drives; the message
    /// says what it is instead.
    Identity(&'static str),
    /// The rings or the CH12 queue failed; the message says how.
    Dma(&'static str),
    SecureBootUnsupported,
    /// The efuse could not be parsed (`-EINVAL` in `efuse.c`).
    Efuse(&'static str),
    /// `rtw89_mac_reset_pwr_state_ax`: the MAC is already powered on (-EBUSY).
    AlreadyPowered,
    /// `R_AX_IC_PWR_STATE[9:8]` read a state in which the chip cannot be
    /// taken as stopped: 2 (`MAC_AX_MAC_LPS`, its firmware alive) or 3
    /// (undefined).
    MacPowerState(u8),
}

impl fmt::Display for AxError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Timeout { register, mask } => {
                write!(
                    f,
                    "timeout waiting on register {register:#x} mask {mask:#x}"
                )
            }
            Self::AllOnes { register } => write!(f, "register {register:#x} read all ones"),
            Self::OutOfWindow { register } => {
                write!(f, "register {register:#x} is outside the usable window")
            }
            Self::Clock => f.write_str("time source unavailable or not advancing"),
            // The bring-up step already names the subsystem, so these print
            // their message alone.
            Self::Firmware(what)
            | Self::FirmwareFile(what)
            | Self::Identity(what)
            | Self::Dma(what)
            | Self::Efuse(what) => f.write_str(what),
            Self::FirmwareStatus { what, fw_ctrl } => write!(f, "{what} (0x01E0={fw_ctrl:#010x})"),
            Self::SecureBootUnsupported => f.write_str("secure-boot firmware is unsupported"),
            Self::AlreadyPowered => f.write_str("MAC has already powered on"),
            Self::MacPowerState(2) => f.write_str(
                "the MAC is in firmware-managed low power (LPS), so its firmware may still run; \
                 Bus Master stays off; power the machine fully off (not restart) and boot again",
            ),
            Self::MacPowerState(state) => write!(
                f,
                "the MAC power state reads {state}, which rtw89 does not define; Bus Master \
                 stays off"
            ),
        }
    }
}

/// Exact register access plus delays and a monotonic clock. Reads are
/// volatile observations and stores complete in program order; `delay_us`
/// waits at least the requested time.
pub trait AxIo {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError>;
    fn read16(&mut self, reg: u32) -> Result<u16, AxError>;
    fn read32(&mut self, reg: u32) -> Result<u32, AxError>;
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError>;
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError>;
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError>;
    fn delay_us(&mut self, us: u64);
    /// Monotonic microseconds from the same clock `delay_us` waits on.
    fn now_us(&mut self) -> Result<u64, AxError>;
}

macro_rules! masked {
    ($t:ty, $read:ident, $write:ident, $set:ident, $clear:ident, $field:ident) => {
        /// `rtw89_write*_set`: set `bits`, keeping the rest.
        pub fn $set<I: AxIo + ?Sized>(io: &mut I, reg: u32, bits: $t) -> Result<(), AxError> {
            let old = io.$read(reg)?;
            io.$write(reg, old | bits)
        }
        /// `rtw89_write*_clr`: clear `bits`, keeping the rest.
        pub fn $clear<I: AxIo + ?Sized>(io: &mut I, reg: u32, bits: $t) -> Result<(), AxError> {
            let old = io.$read(reg)?;
            io.$write(reg, old & !bits)
        }
        /// `rtw89_write*_mask`: replace the `mask` field with `value`, which
        /// is shifted up to the mask's lowest set bit and cut to the mask.
        pub fn $field<I: AxIo + ?Sized>(
            io: &mut I,
            reg: u32,
            mask: $t,
            value: $t,
        ) -> Result<(), AxError> {
            let shift = mask.trailing_zeros() % <$t>::BITS;
            let old = io.$read(reg)?;
            io.$write(reg, (old & !mask) | ((value << shift) & mask))
        }
    };
}
masked!(u8, read8, write8, set8, clear8, field8);
masked!(u16, read16, write16, set16, clear16, field16);
masked!(u32, read32, write32, set32, clear32, field32);

/// Poll until `reg & mask == want`, sampling every `interval_us` for up to
/// `timeout_us` of wall time (`read_poll_timeout`: one last sample after the
/// deadline). A register reading all ones is refused at once. An unchecked
/// stalled clock is stopped by an independent sample-count guard.
pub fn poll32<I: AxIo + ?Sized>(
    io: &mut I,
    reg: u32,
    mask: u32,
    want: u32,
    interval_us: u64,
    timeout_us: u64,
) -> Result<(), AxError> {
    poll32_value(io, reg, mask, want, interval_us, timeout_us).map(|_| ())
}

/// One `poll32` sample: the register, refused if it reads all ones.
fn sample32<I: AxIo + ?Sized>(io: &mut I, reg: u32, wants_all_ones: bool) -> Result<u32, AxError> {
    let value = io.read32(reg)?;
    if value == u32::MAX && !wants_all_ones {
        return Err(AxError::AllOnes { register: reg });
    }
    Ok(value)
}

/// `poll32`, returning the register value that satisfied the wait, as
/// `read_poll_timeout` leaves it in its `val` argument.
pub fn poll32_value<I: AxIo + ?Sized>(
    io: &mut I,
    reg: u32,
    mask: u32,
    want: u32,
    interval_us: u64,
    timeout_us: u64,
) -> Result<u32, AxError> {
    let wants_all_ones = mask == u32::MAX && want == u32::MAX;
    match poll_sample(
        io,
        interval_us,
        timeout_us,
        |io| sample32(io, reg, wants_all_ones),
        |value| value & mask == want,
    )? {
        Polled::Met(value) => Ok(value),
        Polled::Expired(_) => Err(AxError::Timeout {
            register: reg,
            mask,
        }),
    }
}

/// `poll32_value` for a poll Linux writes as `read_poll_timeout_atomic`:
/// budgeted by iterations (`poll_sample_atomic`), not wall time.
pub fn poll32_value_atomic<I: AxIo + ?Sized>(
    io: &mut I,
    reg: u32,
    mask: u32,
    want: u32,
    delay_us: u64,
    timeout_us: u64,
    delay_before: bool,
) -> Result<u32, AxError> {
    let wants_all_ones = mask == u32::MAX && want == u32::MAX;
    match poll_sample_atomic(
        io,
        delay_us,
        timeout_us,
        delay_before,
        |io| sample32(io, reg, wants_all_ones),
        |value| value & mask == want,
    )? {
        Polled::Met(value) => Ok(value),
        Polled::Expired(_) => Err(AxError::Timeout {
            register: reg,
            mask,
        }),
    }
}

/// `poll32_value_atomic` with no delay before the first sample, keeping
/// only the outcome.
pub fn poll32_atomic<I: AxIo + ?Sized>(
    io: &mut I,
    reg: u32,
    mask: u32,
    want: u32,
    delay_us: u64,
    timeout_us: u64,
) -> Result<(), AxError> {
    poll32_value_atomic(io, reg, mask, want, delay_us, timeout_us, false).map(|_| ())
}

/// How a `poll_sample` wait ended.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Polled<T> {
    /// The sample that satisfied the condition.
    Met(T),
    /// The timeout passed; the last sample, as `read_poll_timeout` leaves
    /// it in `val` for a caller that decodes a failure status.
    Expired(T),
}

/// The `read_poll_timeout` loop under every poll: take a `sample` every
/// `interval_us` until `done` accepts one or `timeout_us` of wall time has
/// passed (one last sample after the deadline). `sample` may refuse a value
/// with an error, which ends the wait at once. An unchecked stalled clock is
/// stopped by an independent sample-count guard.
pub fn poll_sample<I: AxIo + ?Sized, T: Copy>(
    io: &mut I,
    interval_us: u64,
    timeout_us: u64,
    mut sample: impl FnMut(&mut I) -> Result<T, AxError>,
    done: impl Fn(T) -> bool,
) -> Result<Polled<T>, AxError> {
    let start = io.now_us()?;
    let end = start.checked_add(timeout_us).ok_or(AxError::Clock)?;
    let guard = (timeout_us / interval_us.max(1))
        .saturating_add(3)
        .saturating_mul(2);
    let mut last = start;
    for _ in 0..guard {
        let now = io.now_us()?;
        if now < last {
            return Err(AxError::Clock);
        }
        last = now;
        let value = sample(io)?;
        if done(value) {
            return Ok(Polled::Met(value));
        }
        if now > end {
            return Ok(Polled::Expired(value));
        }
        io.delay_us(interval_us);
        if io.now_us()? <= now {
            return Err(AxError::Clock);
        }
    }
    Err(AxError::Clock)
}

/// `poll_timeout_us_atomic` (`iopoll.h:85-118`): the budget of every poll
/// Linux writes as `read_poll_timeout_atomic`. It reads no clock. `left_ns`
/// starts at `timeout_us * 1000`; the optional `delay_before` wait and each
/// failed sample's `delay_us(delay_us)` subtract `delay_us * 1000`, and every
/// failed sample also subtracts 1 ns (the `cpu_relax()` branch). The wait is
/// over once `left_ns < 0`, after one last sample. A register read costs real
/// time on PCIe, so this waits longer than `timeout_us` of wall time. Sample
/// counts: `timeout_us * 1000 / (delay_us * 1000 + 1) + 2` (1001 for 1 us
/// over 1000 us; `timeout_us * 1000 + 2` with no delay). The loop bound is
/// exactly that count, so the guard never cuts a Linux-length wait short.
/// `timeout_us == 0` expires after two samples here, where Linux never times
/// out; no caller passes 0.
pub fn poll_sample_atomic<I: AxIo + ?Sized, T: Copy>(
    io: &mut I,
    delay_us: u64,
    timeout_us: u64,
    delay_before: bool,
    mut sample: impl FnMut(&mut I) -> Result<T, AxError>,
    done: impl Fn(T) -> bool,
) -> Result<Polled<T>, AxError> {
    let delay_ns = i128::from(delay_us) * 1000;
    let mut left_ns = i128::from(timeout_us) * 1000;
    if delay_before && delay_us != 0 {
        io.delay_us(delay_us);
        left_ns -= delay_ns;
    }
    let bound = (left_ns.max(0) / (delay_ns + 1)).saturating_add(2);
    let mut taken: i128 = 0;
    while taken < bound {
        taken += 1;
        let expired = left_ns < 0;
        let value = sample(io)?;
        if done(value) {
            return Ok(Polled::Met(value));
        }
        if expired {
            return Ok(Polled::Expired(value));
        }
        if delay_us != 0 {
            io.delay_us(delay_us);
            left_ns -= delay_ns;
        }
        left_ns -= 1;
    }
    Err(AxError::Clock)
}

/// Real volatile MMIO over the device's BAR. The owner (BAR claim and
/// mapping) lives as long as the window: whoever keeps the device after a
/// failure keeps the window, and so the owner, with it.
pub struct NativeAxIo {
    registers: Mmio,
    clock: Clock,
    bytes: usize,
    _owner: Arc<dyn Any>,
}

impl NativeAxIo {
    /// # Safety
    /// `window` is the exclusive uncached BAR of the owned rtw89 requester,
    /// mapped for at least `bytes`, and `owner` keeps that claim and mapping
    /// alive. Safe `write32` can program ring addresses, so DMA domains used
    /// with these registers bind the same requester, as the `pci::pre_init`
    /// contract requires, and the caller writes no DMA address through them
    /// outside that contract. No other worker touches or resets the device
    /// through another path while the window is in use, and `clock` is
    /// calibrated for the CPU that drives it.
    pub unsafe fn new(
        window: NonNull<u8>,
        bytes: usize,
        owner: Arc<dyn Any>,
        clock: Clock,
    ) -> Result<Self, AxError> {
        if bytes < MIN_WINDOW_BYTES || window.as_ptr().addr() & 3 != 0 {
            return Err(AxError::OutOfWindow { register: 0 });
        }
        if clock.ticks_per_microsecond == 0 {
            return Err(AxError::Clock);
        }
        Ok(Self {
            // SAFETY: the constructor's exclusive UC window contract.
            registers: unsafe { Mmio::new(window.as_ptr() as u64, bytes) },
            clock,
            bytes,
            _owner: owner,
        })
    }

    fn check(&self, reg: u32, width: usize) -> Result<usize, AxError> {
        let at = reg as usize;
        if !at.is_multiple_of(width) || at.checked_add(width).is_none_or(|end| end > self.bytes) {
            return Err(AxError::OutOfWindow { register: reg });
        }
        Ok(at)
    }
}

impl AxIo for NativeAxIo {
    fn read8(&mut self, reg: u32) -> Result<u8, AxError> {
        Ok(self.registers.read8(self.check(reg, 1)?))
    }
    fn read16(&mut self, reg: u32) -> Result<u16, AxError> {
        Ok(self.registers.read16(self.check(reg, 2)?))
    }
    fn read32(&mut self, reg: u32) -> Result<u32, AxError> {
        Ok(self.registers.read32(self.check(reg, 4)?))
    }
    fn write8(&mut self, reg: u32, v: u8) -> Result<(), AxError> {
        self.registers.write8(self.check(reg, 1)?, v);
        Ok(())
    }
    fn write16(&mut self, reg: u32, v: u16) -> Result<(), AxError> {
        self.registers.write16(self.check(reg, 2)?, v);
        Ok(())
    }
    fn write32(&mut self, reg: u32, v: u32) -> Result<(), AxError> {
        self.registers.write32(self.check(reg, 4)?, v);
        Ok(())
    }
    fn delay_us(&mut self, us: u64) {
        self.clock.delay(us);
    }
    fn now_us(&mut self) -> Result<u64, AxError> {
        Ok(self.clock.now_us())
    }
}
