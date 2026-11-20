// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2022  Realtek Corporation
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE (PCIe) XTAL SI access and MAC power on/off, from
//! `rtw89_mac_write_xtal_si_ax` / `rtw89_mac_read_xtal_si_ax` (`mac.c:7195`),
//! `rtw8852c_pwr_on_func` / `rtw8852c_pwr_off_func` (`rtw8852c.c:327-529`),
//! `rtw89_mac_power_switch` / `rtw89_mac_pwr_on` (`mac.c:1521-1600`) and
//! `rtw89_mac_reset_pwr_state_ax` (`mac.c:4104`). Only the PCIe branches are
//! ported. The only coex scoreboard write here is the power-off's
//! `MAC_AX_NOTIFY_PWR_MAJOR` (`power_switch_off`); the power-on's
//! `MAC_AX_NOTIFY_TP_MAJOR` and its efuse read belong to the caller. This
//! module does not handle USB or SDIO, an unplugged device, or the vcore
//! reset of thermal protection.
use super::io::{AxError, AxIo, clear8, clear32, field32, poll32, set8, set32};

// reg.h / mac.h (pinned rtw89), PCIe AX.
const R_AX_SYS_ISO_CTRL: u32 = 0x0000;
const R_AX_SYS_FUNC_EN: u32 = 0x0002;
const R_AX_SYS_PW_CTRL: u32 = 0x0004;
const R_AX_SYS_SWR_CTRL1: u32 = 0x0010;
const R_AX_SYS_ADIE_PAD_PWR_CTRL: u32 = 0x0018;
const R_AX_AFE_CTRL1: u32 = 0x0024;
const R_AX_SYS_SDIO_CTRL: u32 = 0x0070;
const R_AX_SYS_ISO_CTRL_EXTEND: u32 = 0x0080;
const R_AX_SCOREBOARD: u32 = 0x00AC;
const R_AX_PLATFORM_ENABLE: u32 = 0x0088;
const R_AX_WLLPS_CTRL: u32 = 0x0090;
const R_AX_PMC_DBG_CTRL2: u32 = 0x00CC;
const R_AX_SYS_STATUS1: u32 = 0x00F4;
const R_AX_SPS_DIG_ON_CTRL0: u32 = 0x0200;
const R_AX_LDO_AON_CTRL0: u32 = 0x0218;
const R_AX_WLAN_XTAL_SI_CTRL: u32 = 0x0270;
const R_AX_LED1_FUNC_SEL: u32 = 0x02DC;
const R_AX_GPIO0_15_EECS_EESK_LED1_PULL_LOW_EN: u32 = 0x02E4;
const R_AX_WLRF_CTRL: u32 = 0x02F0;
const R_AX_IC_PWR_STATE: u32 = 0x03F0;
const R_AX_DMAC_FUNC_EN: u32 = 0x8400;
const R_AX_CMAC_FUNC_EN: u32 = 0xC000;

const fn bit(n: u32) -> u32 {
    1 << n
}

// R_AX_SYS_PW_CTRL.
const B_AX_AFSM_WLSUS_EN: u32 = bit(11);
const B_AX_AFSM_PCIE_SUS_EN: u32 = bit(12);
const B_AX_DIS_WLBT_PDNSUSEN_SOPC: u32 = bit(18);
const B_AX_APDM_HPDN: u32 = bit(15);
const B_AX_APFM_SWLPS: u32 = bit(10);
const B_AX_RDY_SYSPWR: u32 = bit(17);
const B_AX_EN_WLON: u32 = bit(16);
const B_AX_APFN_ONMAC: u32 = bit(8);
const B_AX_APFM_OFFMAC: u32 = bit(9);
const B_AX_XTAL_OFF_A_DIE: u32 = bit(22);
// Others.
const B_AX_PAD_HCI_SEL_V2_MASK: u32 = 0x38; // GENMASK(5, 3)
const MAC_AX_HCI_SEL_PCIE_USB: u32 = 3;
const B_AX_PD_REGU_L: u32 = bit(16);
const B_AX_DIS_WLBT_LPSEN_LOPC: u32 = bit(1);
const B_AX_OCP_L1_MASK: u32 = 0xE000; // GENMASK(15, 13)
const B_AX_REG_ZCDC_H_MASK: u32 = 0x6_0000; // GENMASK(18, 17)
const B_AX_PLATFORM_EN: u8 = 1;
const B_AX_PCIE_CALIB_EN_V1: u32 = bit(12);
const B_AX_CMAC1_FEN: u32 = bit(30);
const B_AX_R_SYM_ISO_CMAC12PP: u32 = bit(5);
// B_AX_R_SYM_WLCMAC1_{P4,P3,P2,P1,}_PC_EN: bits 4..0.
const WLCMAC1_PC_EN_ALL: u32 = 0x1F;
const B_AX_SYM_PADPDN_WL_PTA_1P3: u32 = bit(6);
const B_AX_SYM_PADPDN_WL_RFC_1P3: u32 = bit(5);
const B_AX_SYSON_DIS_PMCR_AX_WRMSK: u32 = bit(2);
const B_AX_ISO_EB2CORE: u32 = bit(8);
const B_AX_PWC_EV2EF_B15: u32 = bit(15);
const B_AX_PWC_EV2EF_B14: u32 = bit(14);
// EECS | EESK | LED1 pull-low enables: bits 16, 17, 18.
const PULL_LOW_EN_ALL: u32 = bit(16) | bit(17) | bit(18);
// MAC_FUNC_EN .. H_AXIDMA_EN: bits 30 down to 14.
const DMAC_FUNC_EN_ALL: u32 = 0x7FFF_C000;
// CMAC_EN, TXEN, RXEN (30..28), FORCE_CMACREG_GCKEN (15), PHYINTF..RMAC (5..0).
const CMAC_FUNC_EN_ALL: u32 = 0x7000_0000 | bit(15) | 0x3F;
const B_AX_PINMUX_EESK_FUNC_SEL_V1_MASK: u32 = 0x0F00_0000; // GENMASK(27, 24)
const PINMUX_EESK_FUNC_SEL_BT_LOG: u32 = 1;
const B_AX_AFC_AFEDIG: u32 = bit(17);
const B_AX_FEN_BB_GLB_RSTN: u8 = 1 << 1;
const B_AX_FEN_BBRSTB: u8 = 1 << 0;
const B_AX_R_SYM_FEN_WLBBGLB_1: u32 = bit(17);
const B_AX_R_SYM_FEN_WLBBFUN_1: u32 = bit(16);
const SW_LPS_OPTION: u32 = 0x0001_A0B2;
const B_AX_SYM_CTRL_SPS_PWMFREQ: u32 = bit(10);
const B_AX_WL_XTAL_SI_CMD_POLL: u32 = bit(31);
const B_AX_WLMAC_PWR_STE_MASK: u32 = 0x300; // GENMASK(9, 8)
const MAC_AX_MAC_ON: u32 = 1;
const MAC_AX_NOTIFY_PWR_MAJOR: u8 = 0x80;
const XTAL_SI_NORMAL_WRITE: u32 = 0;
const XTAL_SI_NORMAL_READ: u32 = 1;

// mac.h XTAL SI offsets and bits.
const XTAL_SI_XTAL_XMD_2: u8 = 0x24;
const XTAL_SI_LDO_LPS: u8 = 0x70; // GENMASK(6, 4)
const XTAL_SI_XTAL_XMD_4: u8 = 0x26;
const XTAL_SI_LPS_CAP: u8 = 0x0F; // GENMASK(3, 0)
const XTAL_SI_WL_RFC_S0: u8 = 0x80;
const XTAL_SI_RF00: u8 = 1 << 0;
const XTAL_SI_WL_RFC_S1: u8 = 0x81;
const XTAL_SI_RF10: u8 = 1 << 0;
const XTAL_SI_ANAPAR_WL: u8 = 0x90;
const XTAL_SI_SRAM2RFC: u8 = 1 << 7;
const XTAL_SI_GND_SHDN_WL: u8 = 1 << 6;
const XTAL_SI_SHDN_WL: u8 = 1 << 5;
const XTAL_SI_RFC2RF: u8 = 1 << 4;
const XTAL_SI_OFF_EI: u8 = 1 << 3;
const XTAL_SI_OFF_WEI: u8 = 1 << 2;
const XTAL_SI_PON_EI: u8 = 1 << 1;
const XTAL_SI_PON_WEI: u8 = 1 << 0;

/// `read_poll_timeout` values of the power sequences (1 ms, 20 ms) and of
/// XTAL SI (50 us, 50 ms).
const PWR_POLL_INTERVAL_US: u64 = 1_000;
const PWR_POLL_TIMEOUT_US: u64 = 20_000;
const XTAL_POLL_INTERVAL_US: u64 = 50;
const XTAL_POLL_TIMEOUT_US: u64 = 50_000;

fn xtal_si_command<I: AxIo + ?Sized>(io: &mut I, command: u32) -> Result<(), AxError> {
    io.write32(R_AX_WLAN_XTAL_SI_CTRL, command)?;
    poll32(
        io,
        R_AX_WLAN_XTAL_SI_CTRL,
        B_AX_WL_XTAL_SI_CMD_POLL,
        0,
        XTAL_POLL_INTERVAL_US,
        XTAL_POLL_TIMEOUT_US,
    )
}

/// `rtw89_mac_write_xtal_si_ax`: write `value` under `mask` at XTAL SI
/// `offset`. Linux only warns when the polled word does not echo the write;
/// that check has no effect on the sequence and is not ported.
pub fn xtal_si_write<I: AxIo + ?Sized>(
    io: &mut I,
    offset: u8,
    value: u8,
    mask: u8,
) -> Result<(), AxError> {
    xtal_si_command(
        io,
        u32::from(offset)
            | u32::from(value) << 8
            | u32::from(mask) << 16
            | XTAL_SI_NORMAL_WRITE << 24
            | B_AX_WL_XTAL_SI_CMD_POLL,
    )
}

/// `rtw89_mac_read_xtal_si_ax`: read the byte at XTAL SI `offset`.
pub fn xtal_si_read<I: AxIo + ?Sized>(io: &mut I, offset: u8) -> Result<u8, AxError> {
    io.write32(
        R_AX_WLAN_XTAL_SI_CTRL,
        u32::from(offset) | XTAL_SI_NORMAL_READ << 24 | B_AX_WL_XTAL_SI_CMD_POLL,
    )?;
    poll32(
        io,
        R_AX_WLAN_XTAL_SI_CTRL,
        B_AX_WL_XTAL_SI_CMD_POLL,
        0,
        XTAL_POLL_INTERVAL_US,
        XTAL_POLL_TIMEOUT_US,
    )?;
    // The poll's last sample is the result word; Linux reuses it.
    Ok((io.read32(R_AX_WLAN_XTAL_SI_CTRL)? >> 8) as u8)
}

/// `rtw89_mac_reset_pwr_state_ax` (PCIe): refuse if the MAC is already on.
fn reset_pwr_state<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let state = (io.read32(R_AX_IC_PWR_STATE)? & B_AX_WLMAC_PWR_STE_MASK) >> 8;
    if state == MAC_AX_MAC_ON {
        return Err(AxError::AlreadyPowered);
    }
    Ok(())
}

/// `MAC_AX_MAC_OFF` (`reg.h:332`).
pub(crate) const MAC_AX_MAC_OFF: u8 = 0;
/// `MAC_AX_MAC_LPS` (`reg.h:334`): firmware-managed power save, WCPU alive.
pub(crate) const MAC_AX_MAC_LPS: u8 = 2;
/// `MAC_AX_MAC_ON` (`reg.h:333`), as a field value.
pub(crate) const MAC_AX_MAC_ON_STATE: u8 = MAC_AX_MAC_ON as u8;

/// The MAC power state field, `R_AX_IC_PWR_STATE[9:8]`, that
/// `rtw89_mac_reset_pwr_state_ax` reads. A register that reads all ones
/// (the device is not answering) is an error, not a state.
pub(crate) fn mac_state<I: AxIo + ?Sized>(io: &mut I) -> Result<u8, AxError> {
    let state = io.read32(R_AX_IC_PWR_STATE)?;
    if state == u32::MAX {
        return Err(AxError::AllOnes {
            register: R_AX_IC_PWR_STATE,
        });
    }
    Ok(((state & B_AX_WLMAC_PWR_STE_MASK) >> 8) as u8)
}

/// `rtw89_mac_power_switch(true)` for PCIe: the already-on check, then
/// `rtw8852c_pwr_on_func`. No scoreboard or efuse work (those follow in the
/// caller).
fn power_on_once<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    reset_pwr_state(io)?;
    pwr_on_func(io)
}

/// `rtw89_mac_power_switch(false)`: `rtw8852c_pwr_off_func`, then, only if it
/// succeeded, `rtw89_mac_update_scoreboard(MAC_AX_NOTIFY_PWR_MAJOR)`, i.e.
/// write8 `R_AX_SCOREBOARD + 3` = 0x80 (`mac.c:1580`, `btc_sb` of rtw8852c).
pub(crate) fn power_switch_off<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    power_off(io)?;
    io.write8(R_AX_SCOREBOARD + 3, MAC_AX_NOTIFY_PWR_MAJOR)
}

/// Both failures of a [`power_on`] whose retry failed too. Linux returns only
/// the retry's; the first is kept as the likelier root cause.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct PowerOnError {
    pub first: AxError,
    pub retry: AxError,
}

/// `rtw89_mac_pwr_on`: on any failure, power off (its result ignored) and
/// try the power switch once more.
pub fn power_on<I: AxIo + ?Sized>(io: &mut I) -> Result<(), PowerOnError> {
    let Err(first) = power_on_once(io) else {
        return Ok(());
    };
    let _ = power_switch_off(io);
    power_on_once(io).map_err(|retry| PowerOnError { first, retry })
}

/// `rtw8852c_pwr_on_func` (`rtw8852c.c:327-451`), PCIe.
fn pwr_on_func<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    let hci_sel = (io.read32(R_AX_SYS_STATUS1)? & B_AX_PAD_HCI_SEL_V2_MASK) >> 3;
    if hci_sel == MAC_AX_HCI_SEL_PCIE_USB {
        set32(io, R_AX_LDO_AON_CTRL0, B_AX_PD_REGU_L)?;
    }

    clear32(
        io,
        R_AX_SYS_PW_CTRL,
        B_AX_AFSM_WLSUS_EN | B_AX_AFSM_PCIE_SUS_EN,
    )?;
    set32(io, R_AX_SYS_PW_CTRL, B_AX_DIS_WLBT_PDNSUSEN_SOPC)?;
    set32(io, R_AX_WLLPS_CTRL, B_AX_DIS_WLBT_LPSEN_LOPC)?;
    clear32(io, R_AX_SYS_PW_CTRL, B_AX_APDM_HPDN)?;
    clear32(io, R_AX_SYS_PW_CTRL, B_AX_APFM_SWLPS)?;

    field32(io, R_AX_SPS_DIG_ON_CTRL0, B_AX_OCP_L1_MASK, 0x7)?;

    poll32(
        io,
        R_AX_SYS_PW_CTRL,
        B_AX_RDY_SYSPWR,
        B_AX_RDY_SYSPWR,
        PWR_POLL_INTERVAL_US,
        PWR_POLL_TIMEOUT_US,
    )?;

    set32(io, R_AX_SYS_PW_CTRL, B_AX_EN_WLON)?;
    set32(io, R_AX_SYS_PW_CTRL, B_AX_APFN_ONMAC)?;

    poll32(
        io,
        R_AX_SYS_PW_CTRL,
        B_AX_APFN_ONMAC,
        0,
        PWR_POLL_INTERVAL_US,
        PWR_POLL_TIMEOUT_US,
    )?;

    set8(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;
    clear8(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;
    set8(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;
    clear8(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;

    set8(io, R_AX_PLATFORM_ENABLE, B_AX_PLATFORM_EN)?;

    clear32(io, R_AX_SYS_SDIO_CTRL, B_AX_PCIE_CALIB_EN_V1)?;

    clear32(io, R_AX_SYS_ISO_CTRL_EXTEND, B_AX_CMAC1_FEN)?;
    set32(io, R_AX_SYS_ISO_CTRL_EXTEND, B_AX_R_SYM_ISO_CMAC12PP)?;
    clear32(io, R_AX_AFE_CTRL1, WLCMAC1_PC_EN_ALL)?;
    set32(io, R_AX_SYS_ADIE_PAD_PWR_CTRL, B_AX_SYM_PADPDN_WL_PTA_1P3)?;

    xtal_si_write(
        io,
        XTAL_SI_ANAPAR_WL,
        XTAL_SI_GND_SHDN_WL,
        XTAL_SI_GND_SHDN_WL,
    )?;

    set32(io, R_AX_SYS_ADIE_PAD_PWR_CTRL, B_AX_SYM_PADPDN_WL_RFC_1P3)?;

    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_SHDN_WL, XTAL_SI_SHDN_WL)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_OFF_WEI, XTAL_SI_OFF_WEI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_OFF_EI, XTAL_SI_OFF_EI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_RFC2RF)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_PON_WEI, XTAL_SI_PON_WEI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_PON_EI, XTAL_SI_PON_EI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_SRAM2RFC)?;
    xtal_si_write(io, XTAL_SI_XTAL_XMD_2, 0x10, XTAL_SI_LDO_LPS)?;
    xtal_si_write(io, XTAL_SI_XTAL_XMD_4, 0, XTAL_SI_LPS_CAP)?;

    set32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;
    set32(io, R_AX_SYS_ISO_CTRL, B_AX_ISO_EB2CORE)?;
    clear32(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B15)?;

    io.delay_us(1000);

    clear32(io, R_AX_SYS_ISO_CTRL, B_AX_PWC_EV2EF_B14)?;
    clear32(io, R_AX_PMC_DBG_CTRL2, B_AX_SYSON_DIS_PMCR_AX_WRMSK)?;

    set32(
        io,
        R_AX_GPIO0_15_EECS_EESK_LED1_PULL_LOW_EN,
        PULL_LOW_EN_ALL,
    )?;

    set32(io, R_AX_DMAC_FUNC_EN, DMAC_FUNC_EN_ALL)?;
    set32(io, R_AX_CMAC_FUNC_EN, CMAC_FUNC_EN_ALL)?;

    field32(
        io,
        R_AX_LED1_FUNC_SEL,
        B_AX_PINMUX_EESK_FUNC_SEL_V1_MASK,
        PINMUX_EESK_FUNC_SEL_BT_LOG,
    )
}

/// `rtw89_mac_power_switch(false)` without its tail: `rtw8852c_pwr_off_func`
/// (`rtw8852c.c:453-529`) only. Linux then notifies BT coex through the
/// scoreboard (`power_switch_off` does that); a caller powering off on its
/// own must append `write8(0x00AF, 0x80)`. An unplugged device would use
/// another function in Linux; that path does not exist here.
pub(crate) fn power_off<I: AxIo + ?Sized>(io: &mut I) -> Result<(), AxError> {
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_RFC2RF, XTAL_SI_RFC2RF)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_OFF_EI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_OFF_WEI)?;
    xtal_si_write(io, XTAL_SI_WL_RFC_S0, 0, XTAL_SI_RF00)?;
    xtal_si_write(io, XTAL_SI_WL_RFC_S1, 0, XTAL_SI_RF10)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, XTAL_SI_SRAM2RFC, XTAL_SI_SRAM2RFC)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_PON_EI)?;
    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_PON_WEI)?;

    set32(io, R_AX_SYS_PW_CTRL, B_AX_EN_WLON)?;
    clear32(io, R_AX_WLRF_CTRL, B_AX_AFC_AFEDIG)?;
    clear8(io, R_AX_SYS_FUNC_EN, B_AX_FEN_BB_GLB_RSTN | B_AX_FEN_BBRSTB)?;
    clear32(
        io,
        R_AX_SYS_ISO_CTRL_EXTEND,
        B_AX_R_SYM_FEN_WLBBGLB_1 | B_AX_R_SYM_FEN_WLBBFUN_1,
    )?;
    clear32(io, R_AX_SYS_ADIE_PAD_PWR_CTRL, B_AX_SYM_PADPDN_WL_RFC_1P3)?;

    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_SHDN_WL)?;

    clear32(io, R_AX_SYS_ADIE_PAD_PWR_CTRL, B_AX_SYM_PADPDN_WL_PTA_1P3)?;

    xtal_si_write(io, XTAL_SI_ANAPAR_WL, 0, XTAL_SI_GND_SHDN_WL)?;

    set32(io, R_AX_SYS_PW_CTRL, B_AX_APFM_OFFMAC)?;

    poll32(
        io,
        R_AX_SYS_PW_CTRL,
        B_AX_APFM_OFFMAC,
        0,
        PWR_POLL_INTERVAL_US,
        PWR_POLL_TIMEOUT_US,
    )?;

    io.write32(R_AX_WLLPS_CTRL, SW_LPS_OPTION)?;

    clear32(io, R_AX_SYS_PW_CTRL, B_AX_XTAL_OFF_A_DIE)?;
    set32(io, R_AX_SYS_SWR_CTRL1, B_AX_SYM_CTRL_SPS_PWMFREQ)?;
    field32(io, R_AX_SPS_DIG_ON_CTRL0, B_AX_REG_ZCDC_H_MASK, 0x3)?;

    set32(io, R_AX_SYS_PW_CTRL, B_AX_APFM_SWLPS)
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::wifi::realtek::ax::tests::model::{Op, RegisterModel};
    use alloc::{boxed::Box, vec, vec::Vec};

    /// Hardware behaviour the sequences wait on: RDY_SYSPWR (0x0004 bit17)
    /// reads set, APFN_ONMAC (bit8) and APFM_OFFMAC (bit9) self-clear, and
    /// every XTAL SI command (0x0270 bit31) completes at once.
    fn settled() -> RegisterModel {
        let mut m = RegisterModel::new();
        m.on_read(
            0x0004,
            Box::new(|m: &mut RegisterModel| (m.get32(0x0004) | 1 << 17) & !(1 << 8 | 1 << 9)),
        );
        m.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
        );
        m
    }

    fn xtal(offset: u32, value: u32, mask: u32) -> Op {
        Op::W32(0x0270, 1 << 31 | mask << 16 | value << 8 | offset)
    }

    /// `rtw8852c_pwr_off_func`, `rtw8852c.c:453-529`, against `settled()`
    /// (0x0004 reads back with bit17 set).
    fn off_trace() -> Vec<Op> {
        vec![
            // 458-482: XTAL SI ANAPAR_WL/WL_RFC_S0/S1 power-down words.
            xtal(0x90, 0x10, 0x10),
            xtal(0x90, 0, 0x08),
            xtal(0x90, 0, 0x04),
            xtal(0x80, 0, 0x01),
            xtal(0x81, 0, 0x01),
            xtal(0x90, 0x80, 0x80),
            xtal(0x90, 0, 0x02),
            xtal(0x90, 0, 0x01),
            // 485: write32_set SYS_PW_CTRL EN_WLON.
            Op::W32(0x0004, 0x0003_0000),
            // 486: write32_clr WLRF_CTRL AFC_AFEDIG.
            Op::W32(0x02F0, 0),
            // 487: write8_clr SYS_FUNC_EN FEN_BB_GLB_RSTN | FEN_BBRSTB.
            Op::W8(0x0002, 0),
            // 488-489: write32_clr SYS_ISO_CTRL_EXTEND WLBBGLB_1 | WLBBFUN_1.
            Op::W32(0x0080, 0),
            // 490: write32_clr ADIE_PAD_PWR_CTRL PADPDN_WL_RFC_1P3.
            Op::W32(0x0018, 0),
            // 492: XTAL SI SHDN_WL cleared.
            xtal(0x90, 0, 0x20),
            // 496: write32_clr ADIE_PAD_PWR_CTRL PADPDN_WL_PTA_1P3.
            Op::W32(0x0018, 0),
            // 498: XTAL SI GND_SHDN_WL cleared.
            xtal(0x90, 0, 0x40),
            // 502: write32_set SYS_PW_CTRL APFM_OFFMAC (poll 504-505).
            Op::W32(0x0004, 0x0003_0200),
            // 509-510: write32 WLLPS_CTRL = SW_LPS_OPTION (PCIe).
            Op::W32(0x0090, 0x0001_A0B2),
            // 514: write32_clr SYS_PW_CTRL XTAL_OFF_A_DIE.
            Op::W32(0x0004, 0x0003_0000),
            // 515: write32_set SYS_SWR_CTRL1 SYM_CTRL_SPS_PWMFREQ.
            Op::W32(0x0010, 0x0400),
            // 516-517: write32_mask SPS_DIG_ON_CTRL0 REG_ZCDC_H = 3.
            Op::W32(0x0200, 0x0006_0000),
            // 519-520: write32_set SYS_PW_CTRL APFM_SWLPS (PCIe).
            Op::W32(0x0004, 0x0003_0400),
        ]
    }

    #[test]
    fn xtal_si_write_encodes_command_and_polls_bit31() {
        let mut m = settled();
        xtal_si_write(&mut m, 0x90, 0x40, 0x40).unwrap();
        // mac.c:7195-7203: offset | data << 8 | mask << 16 | mode 0 | bit31.
        assert_eq!(m.writes(), vec![Op::W32(0x0270, 0x8040_4090)]);
        assert_eq!(m.trace.last(), Some(&Op::R32(0x0270)));
    }

    #[test]
    fn xtal_si_read_uses_mode1_and_returns_data_byte() {
        let mut m = RegisterModel::new();
        m.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| (m.get32(0x0270) & 0x00FF_00FF) | 0xA5 << 8),
        );
        // mac.c:7227: mode 1 at [25:24], data comes back in [15:8].
        assert_eq!(xtal_si_read(&mut m, 0x41).unwrap(), 0xA5);
        assert_eq!(m.writes(), vec![Op::W32(0x0270, 0x8100_0041)]);
    }

    #[test]
    fn xtal_si_times_out() {
        let mut m = RegisterModel::new(); // bit31 never clears
        assert_eq!(
            xtal_si_write(&mut m, 0x90, 1, 1),
            Err(AxError::Timeout {
                register: 0x270,
                mask: 0x8000_0000
            })
        );
        assert_eq!(
            xtal_si_read(&mut m, 0x41),
            Err(AxError::Timeout {
                register: 0x270,
                mask: 0x8000_0000
            })
        );
        // 50 us interval (mac.c:7207).
        assert!(m.trace.contains(&Op::Delay(50)));
    }

    #[test]
    fn power_on_matches_linux_sequence() {
        let mut m = settled();
        m.set32(0x00F4, 3 << 3); // PAD_HCI_SEL_V2 == PCIE_USB
        power_on(&mut m).unwrap();
        let expected = vec![
            // rtw8852c.c:332-335: HCI sel 3 -> write32_set LDO_AON_CTRL0 PD_REGU_L.
            Op::W32(0x0218, 0x0001_0000),
            // 337-338: write32_clr SYS_PW_CTRL AFSM_WLSUS_EN | AFSM_PCIE_SUS_EN.
            Op::W32(0x0004, 0x0002_0000),
            // 339: write32_set SYS_PW_CTRL DIS_WLBT_PDNSUSEN_SOPC.
            Op::W32(0x0004, 0x0006_0000),
            // 340: write32_set WLLPS_CTRL DIS_WLBT_LPSEN_LOPC.
            Op::W32(0x0090, 0x0002),
            // 341: write32_clr SYS_PW_CTRL APDM_HPDN.
            Op::W32(0x0004, 0x0006_0000),
            // 342: write32_clr SYS_PW_CTRL APFM_SWLPS.
            Op::W32(0x0004, 0x0006_0000),
            // 344-345: write32_mask SPS_DIG_ON_CTRL0 OCP_L1 = 7 (poll RDY_SYSPWR 347-348).
            Op::W32(0x0200, 0xE000),
            // 352: write32_set SYS_PW_CTRL EN_WLON.
            Op::W32(0x0004, 0x0007_0000),
            // 353: write32_set SYS_PW_CTRL APFN_ONMAC (poll 355-356).
            Op::W32(0x0004, 0x0007_0100),
            // 360-365: write8 set/clr/set/clr/set PLATFORM_EN.
            Op::W8(0x0088, 1),
            Op::W8(0x0088, 0),
            Op::W8(0x0088, 1),
            Op::W8(0x0088, 0),
            Op::W8(0x0088, 1),
            // 367-368: PCIe: write32_clr SYS_SDIO_CTRL PCIE_CALIB_EN_V1.
            Op::W32(0x0070, 0),
            // 370: write32_clr SYS_ISO_CTRL_EXTEND CMAC1_FEN.
            Op::W32(0x0080, 0),
            // 371: write32_set SYS_ISO_CTRL_EXTEND R_SYM_ISO_CMAC12PP.
            Op::W32(0x0080, 0x20),
            // 372-376: write32_clr AFE_CTRL1 WLCMAC1_P4..PC_EN.
            Op::W32(0x0024, 0),
            // 377: write32_set ADIE_PAD_PWR_CTRL PADPDN_WL_PTA_1P3.
            Op::W32(0x0018, 0x40),
            // 379-380: XTAL SI ANAPAR_WL GND_SHDN_WL.
            xtal(0x90, 0x40, 0x40),
            // 384: write32_set ADIE_PAD_PWR_CTRL PADPDN_WL_RFC_1P3.
            Op::W32(0x0018, 0x60),
            // 386-410: XTAL SI ANAPAR_WL SHDN_WL, OFF_WEI, OFF_EI, RFC2RF=0,
            // PON_WEI, PON_EI, SRAM2RFC=0.
            xtal(0x90, 0x20, 0x20),
            xtal(0x90, 0x04, 0x04),
            xtal(0x90, 0x08, 0x08),
            xtal(0x90, 0, 0x10),
            xtal(0x90, 0x01, 0x01),
            xtal(0x90, 0x02, 0x02),
            xtal(0x90, 0, 0x80),
            // 412-416: XTAL SI XMD_2 = 0x10 under LDO_LPS (0x70), XMD_4 = 0 under LPS_CAP (0x0F).
            xtal(0x24, 0x10, 0x70),
            xtal(0x26, 0, 0x0F),
            // 419: write32_set PMC_DBG_CTRL2 SYSON_DIS_PMCR_AX_WRMSK.
            Op::W32(0x00CC, 0x04),
            // 420: write32_set SYS_ISO_CTRL ISO_EB2CORE.
            Op::W32(0x0000, 0x0100),
            // 421: write32_clr SYS_ISO_CTRL PWC_EV2EF_B15.
            Op::W32(0x0000, 0x0100),
            // 423: fsleep(1000).
            Op::Delay(1000),
            // 425: write32_clr SYS_ISO_CTRL PWC_EV2EF_B14.
            Op::W32(0x0000, 0x0100),
            // 426: write32_clr PMC_DBG_CTRL2 SYSON_DIS_PMCR_AX_WRMSK.
            Op::W32(0x00CC, 0),
            // 428-431: PCIe: write32_set GPIO0_15_EECS_EESK_LED1_PULL_LOW_EN bits 16-18.
            Op::W32(0x02E4, 0x0007_0000),
            // 433-439: write32_set DMAC_FUNC_EN (bits 30..14).
            Op::W32(0x8400, 0x7FFF_C000),
            // 441-445: write32_set CMAC_FUNC_EN.
            Op::W32(0xC000, 0x7000_803F),
            // 447-448: write32_mask LED1_FUNC_SEL PINMUX_EESK_FUNC_SEL_V1 = BT_LOG (1).
            Op::W32(0x02DC, 0x0100_0000),
        ];
        assert_eq!(m.writes(), expected);
    }

    #[test]
    fn power_on_skips_regu_write_when_hci_sel_is_not_pcie_usb() {
        let mut m = settled();
        power_on(&mut m).unwrap();
        assert!(!m.trace.iter().any(|op| matches!(op, Op::W32(0x0218, _))));
        assert_eq!(m.writes()[0], Op::W32(0x0004, 0x0002_0000));
        // The scoreboard write belongs to the caller (ruling R1).
        assert!(!m.trace.iter().any(|op| matches!(op, Op::W8(0x00AF, _))));
    }

    #[test]
    fn power_on_waits_with_linux_poll_intervals() {
        let mut m = RegisterModel::new();
        // RDY_SYSPWR never comes up: 1 ms interval, 20 ms timeout.
        assert_eq!(
            power_on_once(&mut m),
            Err(AxError::Timeout {
                register: 0x0004,
                mask: 1 << 17
            })
        );
        assert!(m.trace.contains(&Op::Delay(1000)));
        let waited: u64 = m
            .trace
            .iter()
            .filter_map(|op| {
                if let Op::Delay(d) = op {
                    Some(*d)
                } else {
                    None
                }
            })
            .sum();
        assert!((20_000..=22_000).contains(&waited), "{waited}");
    }

    #[test]
    fn power_on_from_already_on_powers_off_then_retries_once() {
        let mut m = settled();
        let mut reads = 0;
        m.on_read(
            0x03F0,
            Box::new(move |_m: &mut RegisterModel| {
                reads += 1;
                if reads == 1 { 0x100 } else { 0 }
            }),
        );
        power_on(&mut m).unwrap();
        let w = m.writes();
        // Power-off first, then the whole power-on.
        let off = off_trace();
        assert_eq!(w[..off.len()], off[..]);
        // mac.c:1580: power_switch(false) tells BT coex WL went down.
        assert_eq!(w[off.len()], Op::W8(0x00AF, 0x80));
        // The retry starts from the off sequence's leftover SYS_PW_CTRL (APFM_SWLPS set).
        assert_eq!(w[off.len() + 1], Op::W32(0x0004, 0x0003_0400));
        assert_eq!(w.last(), Some(&Op::W32(0x02DC, 0x0100_0000)));
        let dmac = Op::W32(0x8400, 0x7FFF_C000);
        assert_eq!(w.iter().filter(|op| **op == dmac).count(), 1);
    }

    #[test]
    fn power_on_still_on_after_retry_is_an_error() {
        let mut m = settled();
        m.on_read(0x03F0, Box::new(|_m: &mut RegisterModel| 0x100));
        assert_eq!(
            power_on(&mut m),
            Err(PowerOnError {
                first: AxError::AlreadyPowered,
                retry: AxError::AlreadyPowered,
            })
        );
        // Exactly one power-off ran; the power-on body never started.
        let mut expected = off_trace();
        expected.push(Op::W8(0x00AF, 0x80));
        assert_eq!(m.writes(), expected);
    }

    #[test]
    fn power_on_keeps_the_first_error_when_the_retry_fails_differently() {
        // The MAC reads on at first; after the power-off it reads off, but
        // RDY_SYSPWR never comes up.
        let mut m = RegisterModel::new();
        m.on_read(
            0x0004,
            Box::new(|m: &mut RegisterModel| m.get32(0x0004) & !(1 << 17 | 1 << 8 | 1 << 9)),
        );
        m.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
        );
        let mut reads = 0;
        m.on_read(
            0x03F0,
            Box::new(move |_m: &mut RegisterModel| {
                reads += 1;
                if reads == 1 { 0x100 } else { 0 }
            }),
        );
        assert_eq!(
            power_on(&mut m),
            Err(PowerOnError {
                first: AxError::AlreadyPowered,
                retry: AxError::Timeout {
                    register: 0x0004,
                    mask: 1 << 17
                },
            })
        );
    }

    #[test]
    fn power_on_failure_midway_powers_off_then_retries() {
        // rtw89_mac_pwr_on retries after any power_switch failure, and
        // ignores a failing power-off.
        let mut m = settled();
        m.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| {
                let v = m.get32(0x0270);
                if m.now < 60_000 {
                    v | 0x8000_0000
                } else {
                    v & 0x7FFF_FFFF
                }
            }),
        );
        power_on(&mut m).unwrap();
        assert_eq!(m.writes().last(), Some(&Op::W32(0x02DC, 0x0100_0000)));
    }

    #[test]
    fn power_off_matches_linux_sequence() {
        let mut m = settled();
        power_off(&mut m).unwrap();
        assert_eq!(m.writes(), off_trace());
    }

    #[test]
    fn power_off_times_out_if_offmac_never_clears() {
        let mut m = RegisterModel::new();
        m.on_read(
            0x0270,
            Box::new(|m: &mut RegisterModel| m.get32(0x0270) & 0x7FFF_FFFF),
        );
        assert_eq!(
            power_off(&mut m),
            Err(AxError::Timeout {
                register: 0x0004,
                mask: 1 << 9
            })
        );
    }
}
