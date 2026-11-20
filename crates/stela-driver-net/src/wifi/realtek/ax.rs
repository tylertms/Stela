// SPDX-License-Identifier: GPL-2.0 OR BSD-3-Clause
// Copyright(c) 2019-2020  Realtek Corporation
//! RTL8852CE (rtw89, the "AX" generation) bring-up to running firmware: its
//! register access, with masked and polling helpers that mirror rtw89's
//! `rtw89_read*`/`rtw89_write*`, `rtw89_write*_set`/`_clr`/`_mask` and
//! `read_poll_timeout` uses from pinned rtw89/core.h; MAC power on and off;
//! the efuse; the DMAC and PCIe pre-init with its rings; and the firmware
//! download, which [`bring_up`] puts in Linux's order before powering the
//! chip off again. It grants no station, no RF or baseband setup, and no
//! interrupts: nothing past the firmware's ready status runs.
pub mod bringup;
pub mod dmac;
pub mod efuse;
pub mod fwdl;
pub mod io;
pub mod pci;

pub mod power;
#[cfg(test)]
pub(crate) mod tests;

pub use bringup::{BringUpError, Earlier, Running, bring_up, quiesce};
pub use fwdl::download;
pub use io::{
    AxError, AxIo, NativeAxIo, Polled, clear8, clear16, clear32, field8, field16, field32,
    poll_sample, poll_sample_atomic, poll32, poll32_atomic, poll32_value, poll32_value_atomic,
    set8, set16, set32,
};
// `pci::pre_init` and `dmac::pre_init` are reached by their module only.
pub use pci::{Ch12, Rings, encode_txbd};
pub use power::{power_on, xtal_si_read, xtal_si_write};
