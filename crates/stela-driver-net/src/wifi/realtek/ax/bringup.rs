// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE bring-up to running firmware, in the order and with the failure
//! handling of `rtw89_chip_info_setup` (`core.c:7383`): `rtw89_mac_pwr_on`
//! (`mac.c:1588`) with the tail of `rtw89_mac_power_switch(true)`
//! (`mac.c:1521`: `efuse_read_fw_secure`, then the `MAC_AX_NOTIFY_TP_MAJOR`
//! scoreboard), then `rtw89_chip_efuse_info_setup` (`core.c:7274`), i.e.
//! `rtw89_mac_partial_init` (`mac.c:4307`: MAC pre-init, PCI pre-init,
//! `rtw89_fw_download` with its 5-try loop, `fw.c:2043`) and
//! `rtw89_parse_efuse_map_ax` (`efuse.c:281`) for the MAC address, and
//! finally `rtw89_mac_pwr_off` (`mac.c:1603`), which Linux runs on success
//! and on every failure after power-on.
//!
//! This module does not grant a usable radio: the chip is powered off again
//! before `bring_up` returns, as Linux's probe leaves it, and nothing past
//! the firmware's ready status (phycap, BB/RF, H2C, C2H, scan) is ported.
//! It does not support secure-boot chips, and it does not claim the MAC
//! address read from the efuse is valid.
use super::efuse::{self, SecureBoot};
use super::io::{AxError, AxIo};
use super::pci::Rings;
use super::{dmac, fwdl, pci, power};
use crate::bus::dma::{mapping::DmaDomain, memory::PageSource};
use crate::wifi::realtek::firmware::AxImage;
use alloc::{sync::Arc, vec};
use core::fmt;

/// `R_AX_SCOREBOARD` (`reg.h:156`), the 8852C `btc_sb` register
/// (`rtw8852c.c:3287`); `rtw89_mac_update_scoreboard` writes byte 3.
const R_AX_SCOREBOARD: u32 = 0x00AC;
/// `MAC_AX_NOTIFY_TP_MAJOR` (`reg.h:161`); `power::power_switch_off` writes
/// `MAC_AX_NOTIFY_PWR_MAJOR`.
const MAC_AX_NOTIFY_TP_MAJOR: u8 = 0x81;
/// `rtw89_fw_download`: `for (retry = 0; retry < 5; retry++)` (`fw.c:2048`).
const FWDL_TRIES: u8 = 5;
/// `rtw89_mac_pwr_on` (`mac.c:1588`): one power switch and one retry.
const POWER_ON_TRIES: u8 = 2;

/// An RTL8852CE whose firmware reported `RTW89_FWDL_WCPU_FW_INIT_RDY`.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Running {
    /// The chip cut (`enum rtw89_cv`, 0 = CAV) the bring-up was given.
    pub cut: u8,
    /// The downloaded image's version, major.minor.sub.index.
    pub firmware: (u8, u8, u8, u8),
    /// The efuse MAC address (logical map 0x400).
    pub mac: [u8; 6],
    /// Always false: a secure-boot chip is refused.
    pub secure_boot: bool,
}

impl fmt::Display for Running {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        let (a, b, c, d) = self.firmware;
        let m = self.mac;
        write!(
            f,
            "RTL8852CE firmware running: cut {}, firmware {a}.{b}.{c}.{d}, \
             MAC {:02x}:{:02x}:{:02x}:{:02x}:{:02x}:{:02x}, secure boot {}",
            self.cut,
            m[0],
            m[1],
            m[2],
            m[3],
            m[4],
            m[5],
            if self.secure_boot { "on" } else { "off" },
        )
    }
}

/// The step that failed and why.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct BringUpError {
    /// One of `"power on"`, `"secure boot"`, `"dma rings"`,
    /// `"mac pre-init"`, `"pci pre-init"`, `"firmware download"`, `"efuse"`;
    /// the probe that opens the register window, stages the firmware and
    /// runs [`quiesce`] before [`bring_up`] adds `"register window"`,
    /// `"identity"`, `"firmware"` and `"quiesce"`.
    pub step: &'static str,
    /// The step's last failure.
    pub error: AxError,
    /// What the step's retries ran into before `error`, kept because the
    /// first failure is the likelier root cause.
    pub earlier: Earlier,
}

/// What a retried bring-up step ran into before the error it reports.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub enum Earlier {
    /// The step was not retried, or its first attempt failed the same way.
    #[default]
    Nothing,
    /// The first of `tries` attempts failed with `error`, which the last
    /// attempt's error differs from.
    First { tries: u8, error: AxError },
    /// A `"pci pre-init"` re-run inside the firmware download: download
    /// attempt `attempt` (from 1) failed with `stopped`, which left CH12
    /// stopped, so the next attempt re-ran `pci::pre_init` first. `first`
    /// is attempt 1's error, when that was an earlier, different one.
    Ch12Stopped {
        attempt: u8,
        stopped: AxError,
        first: Option<AxError>,
    },
}

impl Earlier {
    /// `First`, when `first` differs from `last`.
    fn first_of(tries: u8, first: AxError, last: AxError) -> Self {
        if first == last {
            Self::Nothing
        } else {
            Self::First {
                tries,
                error: first,
            }
        }
    }
}

impl BringUpError {
    /// `step` failed with `error`, with nothing earlier.
    pub const fn new(step: &'static str, error: AxError) -> Self {
        Self {
            step,
            error,
            earlier: Earlier::Nothing,
        }
    }
}

impl fmt::Display for BringUpError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "RTL8852CE bring-up: {}: {}", self.step, self.error)?;
        match self.earlier {
            Earlier::Nothing => Ok(()),
            Earlier::First { tries, error } => write!(f, " (first of {tries} attempts: {error})"),
            Earlier::Ch12Stopped {
                attempt,
                stopped,
                first,
            } => {
                write!(
                    f,
                    " (re-run after firmware download attempt {attempt} of {FWDL_TRIES} failed \
                     and stopped CH12: {stopped}"
                )?;
                if let Some(first) = first {
                    write!(f, "; first of {FWDL_TRIES} attempts: {first}")?;
                }
                f.write_str(")")
            }
        }
    }
}

fn at(step: &'static str) -> impl FnOnce(AxError) -> BringUpError {
    move |error| BringUpError::new(step, error)
}

/// Bring an RTL8852CE of chip cut `cut` up to running firmware `image`, and
/// read its MAC address, then power it off again. Steps, in Linux's order:
///
/// 1. `"power on"`: `power::power_on`, with Linux's own power-off and
///    single retry (`rtw89_mac_pwr_on`). When the retry fails too, the error
///    keeps both tries' failures.
/// 2. `"secure boot"`: the efuse secure-boot bytes, then the
///    `MAC_AX_NOTIFY_TP_MAJOR` scoreboard write that closes
///    `rtw89_mac_power_switch(true)`. A secure-boot chip is refused with
///    `SecureBootUnsupported`.
/// 3. `"dma rings"`: `Rings::install`.
/// 4. `"mac pre-init"`: `dmac::pre_init`.
/// 5. `"pci pre-init"`: `pci::pre_init`.
/// 6. `"firmware download"`: up to five `fwdl::download` attempts
///    (`fw.c:2043`), the last error returned, with the first when they
///    differ. An attempt whose CH12 submit failed leaves CH12 stopped; the
///    next attempt then runs `pci::pre_init` again first (a failure there is
///    reported as `"pci pre-init"`, with the download error that stopped
///    CH12 and, when earlier and different, the first one).
/// 7. `"efuse"`: the whole physical map, its logical map and the MAC.
///
/// Success and every failure end as `rtw89_chip_info_setup` does after a
/// power-on, in `rtw89_mac_pwr_off`: `power::power_off`, then, only if it
/// succeeded, the `MAC_AX_NOTIFY_PWR_MAJOR` scoreboard write. Linux ignores
/// the result of that power-off, and so does this. A failed `"power on"`
/// ends the same way, so the chip is left off; Linux returns from it
/// without, but its retry ran the same power-off between the two tries.
///
/// On success the firmware result is reported with the `Rings` that DMA was
/// started on. The caller must keep them (dropping them leaves every mapping
/// quarantined, never returned): the power-off does not prove the engine
/// stopped reading them. On a failure after the rings were programmed, they
/// are dropped after the power-off and stay quarantined the same way.
///
/// # Safety
/// The contracts of `Rings::install`, `pci::pre_init` and `Ch12::submit`:
/// `io` is the exclusive register window of exactly the RTL8852C PCI
/// requester that `domain` isolates; `source` yields real coherent WB DMA
/// RAM for that requester, with NoSnoop and ATS disabled; one worker owns
/// the device and its queues, and no other engine, IRQ or reset worker
/// touches them.
pub unsafe fn bring_up<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
    cut: u8,
    image: &AxImage<'_>,
) -> Result<(Running, Rings), BringUpError> {
    if let Err(error) = power::power_on(io) {
        power_down(io);
        return Err(BringUpError {
            step: "power on",
            error: error.retry,
            earlier: Earlier::first_of(POWER_ON_TRIES, error.first, error.retry),
        });
    }
    // SAFETY: the caller's `Rings::install` contract, forwarded unchanged.
    let mut rings = match unsafe { prepare(io, source, domain) } {
        Ok(rings) => rings,
        Err(error) => {
            power_down(io);
            return Err(error);
        }
    };
    // SAFETY: the caller's `pci::pre_init` and `Ch12::submit` contracts;
    // `rings` were installed on `domain` for the requester behind `io`.
    let result = unsafe { start(io, cut, &mut rings, image) };
    power_down(io);
    // On a failure `rings` drop here, after the power-off.
    Ok((result?, rings))
}

/// `rtw89_mac_pwr_off` (`mac.c:1603`): `rtw89_mac_power_switch(false)`,
/// whose result Linux ignores. `rtw89_mac_set_vcore_reset` does nothing
/// until thermal protection set a voltage level, which nothing here does.
fn power_down<I: AxIo + ?Sized>(io: &mut I) {
    // The scoreboard write (mac.c:1580) follows only a successful
    // pwr_off_func.
    let _ = power::power_switch_off(io);
}

/// Stop an RTL8852CE that firmware or an earlier driver left running,
/// before its requester is taken over, by its MAC power state
/// (`R_AX_IC_PWR_STATE[9:8]`, `reg.h:332-334`):
/// - 0, `MAC_AX_MAC_OFF`: nothing runs; nothing is written.
/// - 1, `MAC_AX_MAC_ON`: the PCIe WPDMA is stopped and the AXI master and
///   TX/RX HCI DMA disabled ([`pci::stop_dma`], the start of
///   `rtw89_pci_ops_mac_pre_init_ax`), then `rtw89_mac_power_switch(false)`
///   runs `rtw8852c_pwr_off_func` (XTAL SI power-down commands, then
///   `B_AX_APFM_OFFMAC` set and polled clear, then the PCIe low-power
///   option), then writes the `MAC_AX_NOTIFY_PWR_MAJOR` scoreboard.
/// - 2, `MAC_AX_MAC_LPS`, firmware-managed power save with the WCPU alive
///   and its HCI DMA path live, and 3, undefined: refused with
///   `MacPowerState`, nothing written. Leaving LPS is the firmware's
///   handshake, which is not ported, so the chip is never taken as stopped.
///
/// Returns whether the MAC was on. This proves nothing about DMA the device
/// issued before, which the caller excludes (Bus Master clear).
pub fn quiesce<I: AxIo + ?Sized>(io: &mut I) -> Result<bool, AxError> {
    match power::mac_state(io)? {
        power::MAC_AX_MAC_OFF => Ok(false),
        power::MAC_AX_MAC_ON_STATE => {
            pci::stop_dma(io)?;
            power::power_switch_off(io)?;
            Ok(true)
        }
        state => {
            debug_assert!(state == power::MAC_AX_MAC_LPS || state == 3);
            Err(AxError::MacPowerState(state))
        }
    }
}

/// The tail of `rtw89_mac_power_switch(true)` (`mac.c:1557-1570`), then the
/// ring memory.
///
/// # Safety
/// The `Rings::install` contract.
unsafe fn prepare<I: AxIo + ?Sized>(
    io: &mut I,
    source: &dyn PageSource,
    domain: Arc<dyn DmaDomain>,
) -> Result<Rings, BringUpError> {
    // efuse_read_fw_secure (mac.c:1560), then rtw89_mac_update_scoreboard(
    // MAC_AX_NOTIFY_TP_MAJOR) (mac.c:1569). Linux only warns on an
    // unreadable secure-boot efuse and fails later, in
    // rtw89_parse_efuse_map_ax; here an unread state is not taken for "off".
    let secure = efuse::secure_boot(io).map_err(at("secure boot"))?;
    io.write8(R_AX_SCOREBOARD + 3, MAC_AX_NOTIFY_TP_MAJOR)
        .map_err(at("secure boot"))?;
    // The non-secure download path is the only one ported (fwdl.rs).
    if let SecureBoot::On { .. } = secure {
        return Err(at("secure boot")(AxError::SecureBootUnsupported));
    }
    // SAFETY: the caller's `Rings::install` contract.
    unsafe { Rings::install(source, domain) }.map_err(at("dma rings"))
}

/// `rtw89_chip_efuse_info_setup` (`core.c:7274`) up to the MAC address:
/// `rtw89_mac_partial_init` (`mac.c:4307`), then the efuse map.
///
/// # Safety
/// The `pci::pre_init` and `Ch12::submit` contracts for `io` and `rings`.
unsafe fn start<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
) -> Result<Running, BringUpError> {
    // rtw89_mac_ctrl_hci_dma_trx and rtw89_mac_dmac_pre_init, then the
    // HCI's mac_pre_init. rtw89_mac_fwdl_preconfig is NULL for AX
    // (mac.c:7516).
    dmac::pre_init(io).map_err(at("mac pre-init"))?;
    // SAFETY: the caller's `pci::pre_init` contract for these rings.
    unsafe { pci::pre_init(io, cut, rings) }.map_err(at("pci pre-init"))?;
    // SAFETY: the caller's contracts; `pci::pre_init` just programmed
    // `rings` through this `io`.
    unsafe { download(io, cut, rings, image) }?;

    // rtw89_parse_efuse_map_ax (efuse.c:281): the DDV physical map from 0,
    // its logical map, then rtw8852c_read_efuse's MAC (rtw8852c.c:648). The
    // R_AX_SYS_WL_EFUSE_CTRL autoload check only warns and is not ported.
    let mut physical = vec![0u8; efuse::PHYSICAL_EFUSE_SIZE];
    efuse::read_physical(io, 0, &mut physical).map_err(at("efuse"))?;
    let logical = efuse::logical_map(&physical).map_err(at("efuse"))?;
    Ok(Running {
        cut,
        firmware: image.version(),
        mac: efuse::mac_address(&logical),
        secure_boot: false,
    })
}

/// `rtw89_fw_download(RTW89_FW_NORMAL)` (`fw.c:2043`): up to five attempts
/// of `__rtw89_fw_download`, the last error returned, with the first when it
/// differs. A failed CH12 submit stops the queue (Linux's queue has no such
/// state), so the next attempt restarts it with `pci::pre_init` first; if
/// that fails, it is reported with the download error that stopped CH12.
///
/// # Safety
/// The `pci::pre_init` and `Ch12::submit` contracts for `io` and `rings`.
unsafe fn download<I: AxIo + ?Sized>(
    io: &mut I,
    cut: u8,
    rings: &mut Rings,
    image: &AxImage<'_>,
) -> Result<(), BringUpError> {
    // SAFETY: the caller's `Ch12::submit` contract; `pci::pre_init`
    // programmed `rings` through this `io`.
    let first = match unsafe { fwdl::download(io, &mut rings.ch12(), image) } {
        Ok(()) => return Ok(()),
        Err(error) => error,
    };
    let mut last = first;
    for attempt in 2..=FWDL_TRIES {
        if rings.ch12_stopped() {
            // SAFETY: the caller's `pci::pre_init` contract for these rings.
            if let Err(error) = unsafe { pci::pre_init(io, cut, rings) } {
                return Err(BringUpError {
                    step: "pci pre-init",
                    error,
                    earlier: Earlier::Ch12Stopped {
                        attempt: attempt - 1,
                        stopped: last,
                        first: (attempt > 2 && first != last).then_some(first),
                    },
                });
            }
        }
        // SAFETY: as for the first attempt; a stopped CH12 was restarted by
        // `pci::pre_init` on these rings just above.
        match unsafe { fwdl::download(io, &mut rings.ch12(), image) } {
            Ok(()) => return Ok(()),
            Err(error) => last = error,
        }
    }
    Err(BringUpError {
        step: "firmware download",
        error: last,
        earlier: Earlier::first_of(FWDL_TRIES, first, last),
    })
}

#[cfg(test)]
mod tests;
