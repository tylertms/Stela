// SPDX-License-Identifier: BSD-3-Clause-Clear
// Copyright (C) 2020-2023 MediaTek Inc.
//! Source-backed PCIe ownership, WFDMA stop and firmware-state polling. These
//! are register observations, not a firmware epoch or DMA/IRQ fence.
use super::{
    identity::Chip,
    registers::{self, BarIo, Registers},
};
use stela_driver_base::stepped::until;

mod cold_start;
pub mod dma;
pub mod mac;

// Register names from mt76 `mt792x_regs.h` and `mt7921/regs.h`.
const MT_PCIE_MAC_INT_ENABLE: u32 = 0x10188;
const MT_WFDMA0_RST: u32 = 0xd4100;
const MT_MCU_CMD: u32 = 0xd41f0;
const MT_MCU2HOST_SW_INT_ENA: u32 = 0xd41f4;
const MT_WFDMA0_HOST_INT_STA: u32 = 0xd4200;
const MT_WFDMA0_HOST_INT_ENA: u32 = 0xd4204;
const MT_WFDMA0_GLO_CFG: u32 = 0xd4208;
const MT_WFDMA0_RST_DTX_PTR: u32 = 0xd420c;
const MT_WFDMA0_RST_DRX_PTR: u32 = 0xd4280;
const MT_WFDMA0_INT_RX_PRI: u32 = 0xd4298;
const MT_WFDMA0_INT_TX_PRI: u32 = 0xd429c;
const MT_WFDMA0_GLO_CFG_EXT0: u32 = 0xd42b0;
const MT_WFDMA0_PRI_DLY_INT_CFG0: u32 = 0xd42f0;
const MT_TX_RING_BASE: u32 = 0xd4300;
const MT_RX_RING_BASE: u32 = 0xd4500;
const MT_WFDMA0_TX_RING0_EXT_CTRL: u32 = 0xd4600;
const MT_WFDMA0_RX_RING0_EXT_CTRL: u32 = 0xd4680;
const MT_WFDMA_DUMMY_CR: u32 = 0x54000120;
const MT_WFDMA_NEED_REINIT: u32 = 1 << 1;
const MT_UWFDMA0_GLO_CFG_EXT1: u32 = 0x7c0242b4;
const MT_DMASHDL_SW_CONTROL: u32 = 0x7c026004;
const MT_CONN_ON_LPCTL: u32 = 0x7c060010;
const MT_HW_BOUND: u32 = 0x70010020;
const MT_HW_CHIPID: u32 = 0x70010200;
const MT_HW_REV: u32 = 0x70010204;
const MT_CONN_ON_MISC: u32 = 0x7c0600f0;
const MT_HW_EMI_CTL: u32 = 0x18011100;
const EMI_SLEEP_PROTECTION: u32 = 1 << 1;
const DISABLE: u32 = 1 | (1 << 2) | (1 << 15) | (1 << 28) | (1 << 27) | (1 << 21);
const BUSY: u32 = (1 << 1) | (1 << 3);

/// The registers of WFDMA0 TX ring `n`: base, size, CPU and DMA index.
pub(in crate::wifi::mediatek) const fn tx_ring(n: u32) -> u32 {
    MT_TX_RING_BASE + 16 * n
}
/// The registers of WFDMA0 RX ring `n`: base, size, CPU and DMA index.
pub(in crate::wifi::mediatek) const fn rx_ring(n: u32) -> u32 {
    MT_RX_RING_BASE + 16 * n
}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error {
    ClockUnavailable,
    NotDriverOwned,
    InitialResetRefused,
    DmaNotStopped,
    UnsupportedDmaMode(u32),
    Readback {
        address: u32,
        mask: u32,
        expected: u32,
        actual: u32,
    },
    Register(registers::Error),
    Unavailable {
        address: u32,
    },
    Timeout {
        address: u32,
        mask: u32,
        expected: u32,
        last: u32,
    },
    Poisoned,
}
impl From<registers::Error> for Error {
    fn from(e: registers::Error) -> Self {
        Self::Register(e)
    }
}
impl core::fmt::Display for Error {
    fn fmt(&self, f: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {
        write!(f, "MediaTek controller: {self:?}")
    }
}
impl core::error::Error for Error {}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Observation {
    pub address: u32,
    pub value: u32,
    pub reads: u32,
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DriverOwned {
    pub attempts: u8,
    pub status: Observation,
}
/// The donor's DMA-disable sequence reached idle and applied its DMASHDL/reset
/// settings. This receipt is not proof that PCIe writes, IRQ handlers/deferred
/// work or firmware processing have drained; it cannot authorize ring unmap.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct DmaStopped {
    pub status: Observation,
    pub reset_applied: bool,
}

pub struct Control<I: BarIo> {
    registers: Registers<I>,
    pub(super) clock: crate::Clock,
    driver_owned: bool,
    dma_stopped: bool,
    poisoned: bool,
    initial_reset_attempted: bool,
    #[cfg(test)]
    polls: alloc::vec::Vec<(u32, u32, u32, u64)>,
}
impl<I: BarIo> Control<I> {
    /// # Safety
    /// The clock is calibrated for this native execution CPU and the
    /// register file belongs to this exclusive live requester. The caller
    /// serializes all ring/MCU/IRQ/power users against these global ownership
    /// and engine changes, and keeps DMA pins and requester ownership
    /// until a separately established complete hardware/CPU drain. This object
    /// neither claims the PCI function nor constructs a fresh firmware epoch.
    pub unsafe fn new(registers: Registers<I>, clock: crate::Clock) -> Result<Self, Error> {
        if clock.ticks_per_microsecond == 0 {
            return Err(Error::ClockUnavailable);
        }
        Ok(Self {
            registers,
            clock,
            driver_owned: false,
            dma_stopped: false,
            poisoned: false,
            initial_reset_attempted: false,
            #[cfg(test)]
            polls: alloc::vec::Vec::new(),
        })
    }
    pub fn is_poisoned(&self) -> bool {
        self.poisoned || self.registers.is_poisoned()
    }
    fn guard(&self, require_owned: bool) -> Result<(), Error> {
        if self.is_poisoned() {
            return Err(Error::Poisoned);
        }
        if require_owned && !self.driver_owned {
            return Err(Error::NotDriverOwned);
        }
        Ok(())
    }
    fn finish<T>(&mut self, result: Result<T, Error>) -> Result<T, Error> {
        if result.is_err() {
            self.driver_owned = false;
            self.dma_stopped = false;
            self.poisoned = true;
        }
        result
    }
    fn read_control(&mut self, address: u32) -> Result<u32, Error> {
        let value = self.registers.read32(address)?;
        // Only mandatory power/DMA control observations use this check. It is
        // never applied to arbitrary firmware data registers or memory.
        if value == u32::MAX {
            return Err(Error::Unavailable { address });
        }
        Ok(value)
    }
    fn modify(&mut self, address: u32, mask: u32, value: u32) -> Result<(), Error> {
        let old = self.read_control(address)?;
        self.registers.write32(address, (old & !mask) | value)?;
        Ok(())
    }
    /// One register read a step until `address` reads `expected` under
    /// `mask` or `timeout_us` passes.
    async fn poll(
        &mut self,
        address: u32,
        mask: u32,
        expected: u32,
        timeout_us: u64,
    ) -> Result<Observation, Error> {
        #[cfg(test)]
        self.polls.push((address, mask, expected, timeout_us));
        let deadline = self.clock.deadline(timeout_us);
        let mut reads = 0u32;
        loop {
            let value = self.read_control(address)?;
            reads = reads.saturating_add(1);
            if value & mask == expected {
                return Ok(Observation {
                    address,
                    value,
                    reads,
                });
            }
            if deadline.passed() {
                return Err(Error::Timeout {
                    address,
                    mask,
                    expected,
                    last: value,
                });
            }
            // mt76_poll_msec_tick(..., tick=1). State transitions are read
            // from the device after each delay, never inferred by time.
            until(self.clock.deadline(1000)).await;
        }
    }
    /// `mt792xe_mcu_fw_pmctrl`: hand the function to its firmware, retrying
    /// the MT_CONN_ON_LPCTL request through at most ten 50 ms polls.
    pub async fn firmware_own(&mut self) -> Result<Observation, Error> {
        self.guard(false)?;
        self.driver_owned = false;
        self.dma_stopped = false;
        let result = async {
            for attempts in 1..=10 {
                self.registers.write32(MT_CONN_ON_LPCTL, 1 << 0)?;
                match self.poll(MT_CONN_ON_LPCTL, 1 << 2, 1 << 2, 50_000).await {
                    Ok(status) => return Ok(status),
                    Err(Error::Timeout { .. }) if attempts < 10 => {}
                    Err(error) => return Err(error),
                }
            }
            unreachable!("the final ownership timeout returns above")
        }
        .await;
        self.finish(result)
    }
    /// The registers `mt7925_pci_probe` identifies the chip by once the
    /// driver owns it: MT_HW_CHIPID, MT_HW_REV and, for MT7921, MT_HW_BOUND,
    /// raw. The remap window is restored afterwards.
    pub fn identity_registers(&mut self) -> Result<(u32, u32, Option<u32>), Error> {
        self.guard(true)?;
        let result = (|| {
            let chip = self.registers.read32(MT_HW_CHIPID)?;
            let revision = self.registers.read32(MT_HW_REV)?;
            let bound = if self.registers.chip() == Chip::Mt7921 {
                Some(self.registers.read32(MT_HW_BOUND)?)
            } else {
                None
            };
            self.registers.restore_remap()?;
            Ok((chip, revision, bound))
        })();
        self.finish(result)
    }
    /// The common MT792xe MT_CONN_ON_LPCTL request retries at most ten 50ms polls. ASPM
    /// requires a minimum 2ms wake delay before each status read.
    pub async fn driver_own(&mut self, aspm_supported: bool) -> Result<DriverOwned, Error> {
        self.guard(false)?;
        self.driver_owned = false;
        self.dma_stopped = false;
        let result = async {
            for attempts in 1..=10 {
                self.registers.write32(MT_CONN_ON_LPCTL, 1 << 1)?;
                if aspm_supported {
                    until(self.clock.deadline(2000)).await;
                }
                match self.poll(MT_CONN_ON_LPCTL, 1 << 2, 0, 50_000).await {
                    Ok(status) => return Ok(DriverOwned { attempts, status }),
                    Err(Error::Timeout { .. }) if attempts < 10 => {}
                    Err(error) => return Err(error),
                }
            }
            unreachable!("the final ownership timeout returns above")
        }
        .await;
        let receipt = self.finish(result)?;
        self.driver_owned = true;
        Ok(receipt)
    }
    /// Stop WLAN WFDMA0 and optionally toggle only its DMASHDL/logic reset.
    /// This does not issue a WLAN subsystem/combo reset, reset Bluetooth,
    /// release any mapping, or claim a fresh MCU response-sequencing epoch.
    pub async fn stop_dma(&mut self, force: bool) -> Result<DmaStopped, Error> {
        self.guard(true)?;
        self.dma_stopped = false;
        let result = async {
            self.modify(MT_WFDMA0_GLO_CFG, DISABLE, 0)?;
            let status = self.poll(MT_WFDMA0_GLO_CFG, BUSY, 0, 100_000).await?;
            if status.value & DISABLE != 0 {
                return Err(Error::Readback {
                    address: MT_WFDMA0_GLO_CFG,
                    mask: DISABLE,
                    expected: 0,
                    actual: status.value,
                });
            }
            self.modify(MT_WFDMA0_GLO_CFG_EXT0, 1 << 6, 0)?;
            self.modify(MT_DMASHDL_SW_CONTROL, 0, 1 << 28)?;
            if force {
                self.modify(MT_WFDMA0_RST, (1 << 4) | (1 << 5), 0)?;
                self.modify(MT_WFDMA0_RST, 0, (1 << 4) | (1 << 5))?;
            }
            // Push every final posted control write before returning. Idle
            // remains an observation, not a universal transaction-drain fence.
            self.read_control(MT_WFDMA0_RST)?;
            Ok(DmaStopped {
                status,
                reset_applied: force,
            })
        }
        .await;
        let receipt = self.finish(result)?;
        self.dma_stopped = true;
        Ok(receipt)
    }
    /// `mt7925_pci_probe`'s `MT_HW_EMI_CTL` sleep protection before its
    /// WLAN subsystem reset, so the reset cannot cut an EMI access short.
    pub fn enable_sleep_protection(&mut self) -> Result<(), Error> {
        self.guard(true)?;
        let result = self.modify(MT_HW_EMI_CTL, EMI_SLEEP_PROTECTION, EMI_SLEEP_PROTECTION);
        self.finish(result)
    }
    /// Poll the pre-download power state for 1 s. Linux warns on a timeout
    /// and downloads anyway, so a timeout is `None`, not a failure, and
    /// leaves the control usable. This does not itself restart the MCU.
    pub async fn poll_firmware_power_on(&mut self) -> Result<Option<Observation>, Error> {
        self.guard(true)?;
        match self.poll(MT_CONN_ON_MISC, 7, 1, 1_000_000).await {
            Ok(observation) => Ok(Some(observation)),
            Err(Error::Timeout { .. }) => Ok(None),
            Err(error) => self.finish(Err(error)),
        }
    }
    /// Observe N9-ready bits after the caller's download/start exchange.
    /// Even success proves only this register state, not which firmware was
    /// loaded, a new command epoch, calibration, regulatory setup or association.
    pub async fn poll_n9_ready(&mut self) -> Result<Observation, Error> {
        self.guard(true)?;
        let result = self.poll(MT_CONN_ON_MISC, 3, 3, 1_500_000).await;
        self.finish(result)
    }
}

#[cfg(test)]
mod tests;
