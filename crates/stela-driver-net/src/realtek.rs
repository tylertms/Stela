// SPDX-License-Identifier: GPL-2.0-only AND GPL-2.0-or-later
// Generic PHY link sampling from Linux phy_device.c (GPL-2.0-or-later):
// Copyright (c) 2004 Freescale Semiconductor, Inc.
// Author: Andy Fleming
// Copyright (c) 2002 ShuChen <shuchen@realtek.com.tw>
// Copyright (c) 2003-2007 Francois Romieu <romieu@fr.zoreil.com>
// Copyright (c) a lot of people too. Please respect their work.
// Copyright (c) 2026 Stela contributors
//! Realtek copper Ethernet, one queue without VLAN/checksum/TSO offloads.
//!
//! Native Rust adaptation of Linux r8169 at
//! `a804c5742bff224d4cb4d25dd383554d598be32b`, including its ordered MAC
//! revision table, analogue PHY sequences and firmware-script instruction set.
//! See NOTICE and `LICENSES/GPL-2.0-only.txt`.
//!
//! This slice implements MAC versions 02..06 (8169S/SB/SC, 8110S/SB/SC),
//! 17..26 (8168B/C/CP/D, 8111B/C/CP/D), 28/31 (8168DP/8111DP), 32..36 (8168E/E-VL/F,
//! 8111E/E-VL/F), 38 (RTL8411), 40/42 (8168G/8111G), 44 (RTL8411B) and
//! 46 (8168H/8111H and 8168M), 51 (8168EP/8111EP) and 52 (8168FP/8117).
//! PHY IDs `001c:c800` (internal FE/GE) and `001c:c912` (RTL8211B) are accepted only
//! with copper-gigabit capabilities. Versions 25/26 require their exact
//! `rtl_nic/rtl8168d-1.fw`/`rtl_nic/rtl8168d-2.fw` resource and matching PHY
//! firmware-readiness sentinel; 34 requires `rtl_nic/rtl8168e-3.fw`.
//! Versions 32/33/35/36/38/40/42/44/46 require exact revision firmware and the
//! internal FE/GE PHY; external PHY initialization is outside these new paths.
//! E1/E2 return an observable warning: optional EEE cleanup is not performed
//! and EEE advertisement state is unverified. External `.fw` image bytes are
//! never embedded or copied.
//!
//! MAC40/42/44 use checked PHY/MAC OCP, explicit shared FIFO ownership, and
//! their exact PHY/start paths; MAC44 keeps the donor's licensed RX fix.
//! MAC46 adds real ADC-bias calibration and saw-count timer programming;
//! EEE advertisement and MAC TX-LPI are cleared, including on link-up.
//! MAC51/52 use the exact EP/FP OOB ERI namespace and bounded CMAC ownership.
//! MAC52 XID54a requires MAC-only `rtl_nic/rtl8168fp-3.fw` after MAC tuning;
//! XID54b and MAC51 require none. Their PHY/EPHY/start paths stay distinct.
//! MAC61/63/64/65/66/70 (RTL8125/8126) require a checked BAR through 0x604a, their exact
//! revision firmware, a validated internal NBASE PHY and real 2.5G
//! capability. Their distinct FIFO/IRQ/doorbell/RX/EPHY and legacy descriptor
//! setup precede DMA. Carrier uses polled PHY BMCR/BMSR with a 10 ms
//! cache; EEE is disabled; advertising is capped to PHY and MAC capability.
//! MAC28/31 use the DP MDIO gate and bounded initial DASH driver-start
//! notification/acknowledgement. Their integrated RTL8211B identity erratum
//! is an observable classification correction; MDIO returns values.
//! Failed DP command ports require reset/reinitialization, without retries.
//! RTL9151AS keeps the raw extended XID, verifies the larger 0x60b4 aperture,
//! and binds firmware to the full primary/extended identity; unlisted values fail.
//! Versions 18 and later with a completion signal in the handoff send it
//! from one MSI-X or MSI vector when a frame arrives or the link changes,
//! as rtl_alloc_irq allows; earlier versions are polled.
//! RTL8139, 100Mbps-only chips, ancient unsupported version 01, external PHY
//! variants, SFP/TBI, DASH stop/release and power lifecycle, suspend/resume,
//! and jumbo frames are
//! outside this slice. This is not a whole-family hardware validation.
//!
//! Drop stops the chip and resets its MAC, as rtl8169_down does; DASH
//! firmware is not told the driver stopped (its EP/FP stop waits seconds).
//! DMA buffers live until reboot when that reset is not observed, and on
//! startup/transmit errors. OWN is the sole ownership transfer; timed-out
//! transmit slots are never overwritten.

use super::{Link, LinkError, MAX_FRAME, MIN_FRAME, Mac, check_frame};
use crate::{
    Handoff,
    block::BlockError,
    bus::{
        dma::DmaAllocator,
        mmio::Mmio,
        pci::{self, ConfigSpace, Function},
    },
    firmware as resources, registers as map_registers,
};
use alloc::{format, string::String, vec::Vec};
use core::{
    fmt,
    mem::ManuallyDrop,
    sync::atomic::{Ordering, fence},
};
mod firmware;
mod hardware;
mod phy;
mod recovery;
mod registers;
mod revision;
mod ring;
mod start;
use firmware::Script;
use hardware::Hardware;
use registers::*;
pub use revision::{PCI_DEVICES, Revision};
use ring::Ring;
const NBASE_LINK_POLL_US: u64 = 10_000;
// Firmware handoff may leave a working MAC behind an undelivered MSI.
// Keep receiving on MAC46 even when its completion vector never wakes us.
const H_RECEIVE_POLL_US: u64 = 10_000;
/// The events that send the completion signal: rtl8169's RTL_EVENT_NAPI_RX
/// and its link change.
const SIGNALLED_EVENTS: u16 = RX_OK | RX_ERROR | LINK_CHANGE;

#[derive(Clone, Debug, PartialEq, Eq)]
pub enum Error {
    Device(BlockError),
    Firmware(resources::Error),
    Script(&'static str),
    UnknownMac(u32),
    UnknownExtendedMac { primary: u16, extended: u32 },
    UnsupportedRevision(Revision),
    UnsupportedPhy(u32),
}
impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Device(e) => e.fmt(f),
            Self::Firmware(e) => e.fmt(f),
            Self::Script(e) => write!(f, "Realtek firmware/PHY: {e}"),
            Self::UnknownMac(u32::MAX) => {
                f.write_str("Realtek registers read all ones: the chip does not answer")
            }
            Self::UnknownMac(tx) => write!(
                f,
                "unknown Realtek chip revision XID {:03x} (TxConfig {tx:#010x})",
                (tx >> 20) & 0xfcf
            ),
            Self::UnknownExtendedMac { primary, extended } => write!(
                f,
                "unknown Realtek MAC XID {primary:03x} extended {extended:08x} (classification {:08x})",
                extended & 0x7fff_ffff
            ),
            Self::UnsupportedRevision(r) => {
                write!(f, "{} MAC version {} XID {:03x}", r.name, r.version, r.xid)?;
                if let Some(raw) = r.extended_xid {
                    write!(f, " extended {raw:08x}")?;
                }
                write!(f, " startup is not implemented")
            }
            Self::UnsupportedPhy(id) => write!(
                f,
                "Realtek copper PHY {id:#010x} startup is not implemented"
            ),
        }
    }
}
impl core::error::Error for Error {}
impl From<BlockError> for Error {
    fn from(e: BlockError) -> Self {
        Self::Device(e)
    }
}
impl From<resources::Error> for Error {
    fn from(e: resources::Error) -> Self {
        Self::Firmware(e)
    }
}
fn block_error(e: BlockError) -> Error {
    e.into()
}

/// Candidate PCI IDs; the constructor additionally requires an exact supported
/// MAC revision and copper PHY. PCI-ID recognition alone does not claim support.
pub fn matches<C: ConfigSpace>(function: &Function<C>) -> bool {
    matches_identity(&function.identity())
}

/// Candidate selection from copied identity DATA; owned attachment still
/// revalidates configuration and platform requirements.
pub fn matches_identity(function: &crate::bus::pci::Identity) -> bool {
    function.vendor == 0x10ec
        && revision::pci_candidate(function.device)
        && function.class == [2, 0, 0]
}
trait Registers {
    fn read8(&self, offset: usize) -> u8;
    fn read16(&self, offset: usize) -> u16;
    fn read32(&self, offset: usize) -> u32;
    fn write8(&self, offset: usize, value: u8);
    fn write16(&self, offset: usize, value: u16);
    fn write32(&self, offset: usize, value: u32);
}
impl Registers for Mmio {
    fn read8(&self, o: usize) -> u8 {
        self.read8(o)
    }
    fn read16(&self, o: usize) -> u16 {
        self.read16(o)
    }
    fn read32(&self, o: usize) -> u32 {
        self.read32(o)
    }
    fn write8(&self, o: usize, v: u8) {
        self.write8(o, v)
    }
    fn write16(&self, o: usize, v: u16) {
        self.write16(o, v)
    }
    fn write32(&self, o: usize, v: u32) {
        self.write32(o, v)
    }
}
trait Timer {
    fn now(&self) -> u64;
    #[track_caller]
    fn delay(&self, microseconds: u64) -> Result<(), BlockError>;
}
impl Timer for crate::timing::Timing {
    fn now(&self) -> u64 {
        self.now_us()
    }
    #[track_caller]
    fn delay(&self, us: u64) -> Result<(), BlockError> {
        crate::timing::Timing::delay(self, us)
    }
}

/// A tolerated, observable limitation of otherwise completed initialization.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Warning {
    /// MAC32/33's optional donor EEE cleanup has no C45 bus implementation.
    /// The optional cleanup is not performed; EEE advertisement is unverified.
    EeeAdvertisementDisableUnavailable,
    /// Exact MAC28/31 integrated RTL8211B reports the generic FE/GE ID.
    /// The raw register read remains available; only driver classification changes.
    DpPhyIdentityCorrected { reported: u32, selected: u32 },
    /// The chip's firmware file was not supplied, or was refused: as Linux
    /// r8169 does, the chip starts without it, on its built-in parameters.
    FirmwareSkipped { name: &'static str, supplied: bool },
    /// An RTL8168D PHY's readiness sentinel differed, so its firmware was
    /// skipped, as Linux `rtl8168d_apply_firmware_cond` skips it.
    PhyNotReadyForFirmware { readiness: u16 },
}
impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::DpPhyIdentityCorrected { reported, selected } => write!(f,
                "RTL8168DP PHY identity erratum: reported {reported:08x}, selected {selected:08x} RTL8211B initialization"),
            Self::EeeAdvertisementDisableUnavailable => f.write_str(
                "optional EEE advertisement cleanup not performed: C45 access unsupported; EEE advertisement state unverified",
            ),
            Self::FirmwareSkipped { name, supplied } => write!(
                f,
                "firmware \"{name}\" {}: started without it, on the chip's built-in parameters, as Linux does",
                if *supplied { "refused" } else { "not supplied" }
            ),
            Self::PhyNotReadyForFirmware { readiness } => write!(
                f,
                "RTL8168D PHY not ready for firmware (sentinel {readiness:04x}): started without it, as Linux does"
            ),
        }
    }
}

pub struct Realtek {
    device: Mmio,
    clock: crate::timing::Timing,
    receive: ManuallyDrop<Ring>,
    transmit: ManuallyDrop<Ring>,
    mac: Mac,
    link: RuntimeLink,
    revision: Revision,
    warnings: Vec<Warning>,
    /// The PCI function number, which selects per-function OCP state.
    function: u8,
    pub location: String,
    /// Whether the function's vector sends the handoff's completion signal.
    signalled: bool,
    receive_watch: recovery::ReceiveWatch,
}
impl Realtek {
    /// Read-only failure snapshot; submitted TX frames are not proof of delivery.
    pub fn diagnostics(&self) -> String {
        let irq = if self.revision.nbase() {
            self.device.read32(NBASE_INTERRUPT_STATUS)
        } else {
            u32::from(self.device.read16(INTERRUPT_STATUS))
        };
        format!(
            "Realtek link={} tx={} pending={} rx={} dropped={} cmd={:#04x} phy={:#04x} irq={irq:#06x} rxdesc={:#010x} poll={}ms recoveries={} rxowned={} rxcfg={:#010x}",
            if self.device.read8(PHY_STATUS) & LINK_STATUS != 0 {
                "up"
            } else {
                "down"
            },
            self.transmit.packets,
            self.transmit.owned(),
            self.receive.packets,
            self.receive.discarded,
            self.device.read8(COMMAND),
            self.device.read8(PHY_STATUS),
            self.receive.next_status(),
            if self.revision.version == 46 { 10 } else { 0 },
            self.receive_watch.restarts,
            self.receive.owned(),
            self.device.read32(RX_CONFIG),
        )
    }

    /// What the start tolerated, each said once.
    pub fn warnings(&self) -> &[Warning] {
        &self.warnings
    }

    pub fn revision(&self) -> Revision {
        self.revision
    }

    /// Initialize one exclusively owned physical function after ExitBootServices.
    /// The provider's exact binding is `driver=realtek`, lowercase PCI identity
    /// (e.g. `10ec:8168`), `revision=xid-2c8`, `board=None`. XID contains every
    /// TxConfig bit used by Linux's version table, even bits its mask ignores.
    /// The image is borrowed only for this call; its allocation is never copied.
    ///
    /// # Safety
    /// Caller exclusively owns the controller and its segment's PCI
    /// configuration. Handoff ECAM/MMIO and DMA resources must be mapped
    /// uncached/device-accessible machine ranges. Firmware and other drivers
    /// must not access them, including during BAR sizing and upstream ASPM edits.
    pub unsafe fn new(
        function: &mut Function,
        handoff: &Handoff,
        dma: &mut dyn DmaAllocator,
        provider: &dyn resources::Provider,
        minimum: alloc::rc::Rc<dyn crate::bus::pci::power::Timer>,
    ) -> Result<Self, Error> {
        let clock = crate::timing::Timing::checked(handoff.clock, minimum)?;
        if !matches(function) {
            return Err(
                BlockError::Unsupported("not a supported Realtek PCI Ethernet candidate").into(),
            );
        }
        let pcie_link_control = prepare_pcie_link(function, |function| {
            // SAFETY: caller grants exclusive PCI segment access and valid ECAM.
            unsafe { pci::disable_aspm_link(function, handoff) }
        })?;
        function.power_up(clock.minimum())?;
        let bars = function.bars()?;
        let bar = bars
            .into_iter()
            .flatten()
            .find(|b| matches!(b, pci::Bar::Memory { .. }));
        let device = map_registers(handoff, bar, 0, 256)?;
        function.enable_polled()?;
        let txconfig = device.read32(TX_CONFIG);
        // A window panics on an access past its length, so each read goes
        // through one that covers it: the extended identity word, then
        // every register the detected revision's start and runtime reach.
        let device = widen(handoff, bar, device, identity_aperture(txconfig))?;
        let revision = detect_revision(&device, txconfig, |bytes| {
            map_registers(handoff, bar, 0, bytes).map(|_| ())
        })?;
        let device = widen(handoff, bar, device, register_aperture(revision))?;
        admit_revision(function.device, pcie_link_control, revision)?;
        if tbi_mode(revision, device.read8(PHY_STATUS)) {
            return Err(BlockError::Unsupported("Realtek TBI/fiber mode").into());
        }
        // rtl_alloc_irq keeps versions before 18 on INTx, which stays off.
        let signalled = match handoff.completion {
            Some(message) if revision.version >= 18 => {
                function.enable_message(handoff, &bars, message)?
            }
            _ => false,
        };
        let subsystem = function.config().read32(0x2c)?;
        let mut hardware = Hardware::new(&device, &clock, revision, function.address.function);
        if revision.nbase() && hardware.mac_ocp_read(0xd006)? & 0xff == 7 {
            return Err(BlockError::Unsupported("Realtek SFP mode").into());
        }
        hardware.initialize_modern()?;
        hardware.reset()?;
        hardware.dp_start()?;
        hardware.ep_start()?;
        if revision.version <= 6 {
            function.config().write16(0x0c, 0x4008)?; // Linux cache-line/latency values, narrow config write.
            device.write8(0x82, 1);
        }
        let mac = read_station(&hardware)?;
        let identity = format!("{:04x}:{:04x}", function.vendor, function.device);
        let mut warnings = Vec::new();
        let script = optional_firmware(provider, &identity, revision, &mut warnings);
        warnings.extend(hardware.configure_phy(script.as_ref(), subsystem == 0xe000_1458)?);
        hardware.reset()?; // Linux resets the MAC again after PHY initialization.
        hardware.configure_mac(script.as_ref())?;
        if revision.version == 28 {
            // The exact DP4 start branch enables CLKREQ after EPHY setup.
            // ASPM link states remain disabled by the shared PCI boundary.
            let offset = pcie_link_control.ok_or(BlockError::Unsupported(
                "Realtek DP without PCIe link control",
            ))?;
            clock_request(function.config(), offset, true)?;
        }
        // Nothing has a descriptor address yet. Allocation faults cannot start DMA.
        let mut net = Self {
            device,
            clock,
            receive: ManuallyDrop::new(Ring::new(dma, revision.version >= 18)?),
            transmit: ManuallyDrop::new(Ring::new(dma, revision.version >= 18)?),
            mac,
            link: RuntimeLink::default(),
            revision,
            warnings,
            function: function.address.function,
            location: format!(
                "{} {} XID {:03x}",
                function.address, revision.name, revision.xid
            ),
            signalled,
            receive_watch: recovery::ReceiveWatch::default(),
        };
        net.receive.initialize(true);
        net.transmit.initialize(false);
        let hardware = Hardware::new(&net.device, &net.clock, revision, function.address.function);
        if let Err(error) = publish(&hardware, &net.receive, &net.transmit, mac) {
            // Buffers remain boot-owned even if a disconnected/stuck device fails reset.
            let _ =
                Hardware::new(&net.device, &net.clock, revision, function.address.function).reset();
            return Err(error.into());
        }
        if net.signalled {
            hardware.interrupts_on(SIGNALLED_EVENTS);
        }
        Ok(net)
    }

    fn update_link(&mut self) -> Result<bool, LinkError> {
        let hardware = Hardware::new(&self.device, &self.clock, self.revision, self.function);
        self.link.update(&hardware).map_err(|error| {
            if matches!(error, BlockError::PciPower(_)) {
                return LinkError::Pci(error);
            }
            LinkError::Adapter(format!("{} link configuration: {error}", self.location))
        })
    }
}

/// Link-programming errors leave command completion and filter state uncertain.
/// There is no retry or in-place recovery: only a fresh constructor after the
/// caller's explicit device reset/reinitialization can regain ownership.
#[derive(Default)]
struct RuntimeLink {
    status: Option<u8>,
    failure: Option<BlockError>,
    last_phy_poll: Option<u64>,
}
impl RuntimeLink {
    fn update<R: Registers, T: Timer>(
        &mut self,
        hw: &Hardware<'_, R, T>,
    ) -> Result<bool, BlockError> {
        if let Some(error) = &self.failure {
            return Err(error.clone());
        }
        if hw.revision.nbase()
            && self
                .last_phy_poll
                .is_some_and(|last| hw.timer.now().wrapping_sub(last) < NBASE_LINK_POLL_US)
        {
            return Ok(self.status.is_some_and(|status| status & LINK_STATUS != 0));
        }
        let observed: Result<u8, BlockError> = if hw.revision.nbase() {
            // genphy_update_link's polled BMCR/BMSR sequence preserves a
            // latch-low drop while up, and retries a low sample while down.
            // A 10 ms cache keeps PHY polling out of the per-packet fast path.
            (|| {
                let read = |reg| {
                    let value = hw.phy_read(reg)?;
                    if value == u16::MAX {
                        Err(BlockError::Device { status: 65535 })
                    } else {
                        Ok(value)
                    }
                };
                if read(PHY_CONTROL)? & BMCR_RESTART_AUTONEGOTIATION != 0 {
                    return Ok(0);
                }
                let previous_up = self.status.is_some_and(|status| status & LINK_STATUS != 0);
                let mut status = read(MII_STATUS)?;
                if !previous_up && status & BMSR_LINK == 0 {
                    status = read(MII_STATUS)?;
                }
                // This driver always configures autonegotiation: an old
                // link indication must not outlive a cleared AN-complete bit.
                let up = BMSR_LINK | BMSR_AUTONEGOTIATION_COMPLETE;
                Ok(if status & up == up { LINK_STATUS } else { 0 })
            })()
        } else {
            Ok(hw.registers.read8(PHY_STATUS))
        };
        let status = match observed {
            Ok(status) => status,
            Err(error) => {
                self.failure = Some(error.clone());
                return Err(error);
            }
        };
        let result = if status == u8::MAX {
            Err(BlockError::Device { status: 255 })
        } else if self.status != Some(status)
            && status & LINK_STATUS != 0
            && matches!(hw.revision.version, 34..=36 | 38 | 46 | 51 | 52 | 61 | 63..=66 | 70)
        {
            link_patch(hw, status)
        } else {
            Ok(())
        };
        if let Err(error) = result {
            self.failure = Some(error.clone());
            return Err(error);
        }
        self.status = Some(status);
        if hw.revision.nbase() {
            self.last_phy_poll = Some(hw.timer.now());
        }
        Ok(status & LINK_STATUS != 0)
    }
}

fn clock_request<C: ConfigSpace>(
    config: &mut C,
    link_control: u16,
    enabled: bool,
) -> Result<(), BlockError> {
    let value = config.read16(link_control)?;
    if value == u16::MAX {
        return Err(BlockError::Device {
            status: u32::from(value),
        });
    }
    config.write16(
        link_control,
        (value & !(1 << 8)) | (u16::from(enabled) << 8),
    )?;
    Ok(())
}

fn prepare_pcie_link<C: ConfigSpace>(
    function: &mut Function<C>,
    disable_aspm: impl FnOnce(&mut Function<C>) -> Result<(), BlockError>,
) -> Result<Option<u16>, BlockError> {
    let invalid = || BlockError::Unsupported("invalid Realtek PCIe capabilities");
    let mut control = None;
    for entry in crate::bus::pci::capability::capabilities(function.config()) {
        let entry = entry?;
        if entry.id == 0x10 {
            let offset = entry.offset + 0x10;
            if offset + 2 > 0x100 || control.replace(offset).is_some() {
                return Err(invalid());
            }
        }
    }
    if let Some(offset) = control {
        // The shared ASPM boundary validates link type and topology before
        // either MMIO decoding or bus mastering is enabled by this constructor.
        disable_aspm(function)?;
        clock_request(function.config(), offset, false)?;
    }
    Ok(control)
}

/// Whether the chip runs its TBI (fiber) interface, which this driver does
/// not: PHYstatus bit 7 says so only on the 8169 family, versions 02..06.
/// Older Linux read it as TBI only there (`rtl_tbi_enabled`), and the pinned
/// source has dropped TBI altogether. 8168 chips set the bit with their
/// copper PHY (an ASUS board's RTL8111H does), so it is no refusal there.
fn tbi_mode(revision: Revision, phy_status: u8) -> bool {
    revision.version <= 6 && phy_status & TBI_ENABLE != 0
}

fn admit_revision(device: u16, pcie_control: Option<u16>, revision: Revision) -> Result<(), Error> {
    if !revision::pci_family(device, revision) || !revision.implemented() {
        return Err(Error::UnsupportedRevision(revision));
    }
    if revision.version > 6 && pcie_control.is_none() {
        return Err(
            BlockError::Unsupported("Realtek PCIe revision without PCIe capability").into(),
        );
    }
    Ok(())
}

/// The registers the identity reads span: the first 256 bytes, and through
/// the extended word where TxConfig says the chip has one.
fn identity_aperture(txconfig: u32) -> u64 {
    if revision::needs_extended(txconfig) {
        EXTENDED_REGISTER_BYTES
    } else {
        256
    }
}

/// `device`, or a window of `bytes` over the same BAR when it is shorter.
/// The BAR must cover and map the wider range.
fn widen(
    handoff: &Handoff,
    bar: Option<pci::Bar>,
    device: Mmio,
    bytes: u64,
) -> Result<Mmio, BlockError> {
    if bytes <= device.len() as u64 {
        return Ok(device);
    }
    map_registers(handoff, bar, 0, bytes)
}

/// The registers a revision's start and runtime span.
fn register_aperture(revision: Revision) -> u64 {
    if revision.nbase() {
        NBASE_REGISTER_BYTES
    } else {
        256
    }
}

fn kick_transmit(registers: &impl Registers, revision: Revision) {
    if revision.nbase() {
        registers.write16(NBASE_TX_POLL, 1);
    } else {
        registers.write8(TX_POLL, POLL_NORMAL);
    }
}

fn detect_revision<R: Registers>(
    registers: &R,
    txconfig: u32,
    require_range: impl FnOnce(u64) -> Result<(), BlockError>,
) -> Result<Revision, Error> {
    if !revision::needs_extended(txconfig) {
        let revision = revision::detect(txconfig).ok_or(Error::UnknownMac(txconfig))?;
        if revision.nbase() {
            // PCI aliases do not imply a layout: validate the detected NBASE
            // range before returning a revision that can reach extended I/O.
            require_range(register_aperture(revision))?;
        }
        return Ok(revision);
    }
    // The first BAR must cover and map the extended word before I/O.
    require_range(EXTENDED_REGISTER_BYTES)?;
    let raw = registers.read32(TX_CONFIG_V2);
    if raw == u32::MAX {
        return Err(BlockError::Device { status: raw }.into());
    }
    revision::detect_extended(txconfig, raw).ok_or(Error::UnknownExtendedMac {
        primary: ((txconfig >> 20) & 0xfcf) as u16,
        extended: raw,
    })
}

/// The `revision` label of a firmware target: the chip's XID, and its
/// extended word where it has one.
fn target_revision(revision: Revision) -> String {
    match revision.extended_xid {
        Some(raw) => format!("xid-{:03x}-ext-{raw:08x}", revision.xid),
        None => format!("xid-{:03x}", revision.xid),
    }
}

/// Every firmware request an admitted chip can make, as its PCI device,
/// the file and the target's `revision` label: the requests setup's pinned
/// files must answer.
pub fn firmware_requests() -> impl Iterator<Item = (u16, &'static str, String)> {
    PCI_DEVICES.iter().flat_map(|&device| {
        revision::with_firmware()
            .filter(move |&revision| revision::pci_family(device, revision))
            .filter_map(move |revision| {
                Some((device, revision.firmware?, target_revision(revision)))
            })
    })
}

fn load_firmware<'a>(
    provider: &'a dyn resources::Provider,
    hardware: &str,
    revision: Revision,
) -> Result<Option<resources::Image<'a>>, Error> {
    let xid = target_revision(revision);
    revision
        .firmware
        .map(|name| {
            provider
                .load(resources::Requirement {
                    name,
                    target: resources::Target {
                        driver: "realtek",
                        hardware,
                        revision: &xid,
                        board: None,
                    },
                    minimum_bytes: 4,
                })
                .map_err(Error::from)
        })
        .transpose()
}

/// Linux `rtl_request_firmware`: every chip's file is optional. One that is
/// missing or fails its checks is left out, said once in `warnings`, and
/// the chip starts on its built-in parameters. A file that passes is
/// parsed here once and its script returned.
fn optional_firmware<'a>(
    provider: &'a dyn resources::Provider,
    hardware: &str,
    revision: Revision,
    warnings: &mut Vec<Warning>,
) -> Option<Script<'a>> {
    let supplied = match load_firmware(provider, hardware, revision) {
        Ok(None) => return None,
        Ok(Some(image)) => match Script::parse(image.bytes()) {
            Ok(script) => return Some(script),
            Err(_) => true,
        },
        Err(error) => !matches!(
            error,
            Error::Firmware(resources::Error {
                kind: resources::ErrorKind::Missing,
                ..
            })
        ),
    };
    warnings.extend(
        revision
            .firmware
            .map(|name| Warning::FirmwareSkipped { name, supplied }),
    );
    None
}

/// Linux `rtl_link_chg_patch`: per-speed ERI FIFO thresholds and EEE state
/// at link-up. The donor gives these vendor offsets and values no names.
fn link_patch<R: Registers, T: Timer>(
    hw: &Hardware<'_, R, T>,
    status: u8,
) -> Result<(), BlockError> {
    if hw.revision.nbase() {
        hw.mac_ocp_modify(0xe040, 3, 0)?;
        if hw.revision.version == 61 {
            hw.mac_ocp_modify(0xeb62, 6, 0)?;
        }
        return Ok(());
    }
    if matches!(hw.revision.version, 46 | 51 | 52) {
        // EEE stays disabled, including stale MAC TX-LPI state at link-up.
        hw.registers.write8(0x1b, hw.mmio8(0x1b)? & !7);
        return hw.eri_modify(0x1b0, 3, 0);
    }
    let (first, mut second) = match status & 0x1c {
        0x10 => (0x11, 0x05),
        0x08 => (0x1f, 0x05),
        0x04 => (0x1f, 0x3f),
        _ => {
            return Err(BlockError::Unsupported(
                "Realtek link reports an unknown copper speed",
            ));
        }
    };
    if matches!(hw.revision.version, 35 | 36) && status & 0x1c != 0x10 {
        second = 0x3f;
    }
    hw.eri_write(0x1bc, 15, first)?;
    hw.eri_write(0x1dc, 15, second)?;
    if matches!(hw.revision.version, 34 | 38) {
        hw.eri_modify(0xdc, 1, 0)?;
        hw.eri_modify(0xdc, 0, 1)?;
    }
    Ok(())
}
fn read_station<R: Registers, T: Timer>(hw: &Hardware<'_, R, T>) -> Result<Mac, BlockError> {
    if hw.revision.nbase()
        && let Some(mac) = station(
            hw.mmio32(NBASE_MAC0_BACKUP)?,
            hw.mmio32(NBASE_MAC0_BACKUP + 4)?,
        )
    {
        return Ok(mac);
    }
    if matches!(
        hw.revision.version,
        35 | 36 | 38 | 40 | 42 | 44 | 46 | 51 | 52
    ) && let Some(mac) = station(hw.eri_read(0xe0)?, hw.eri_read(0xe4)?)
    {
        return Ok(mac);
    }
    station(hw.registers.read32(MAC0), hw.registers.read32(MAC4)).ok_or(BlockError::Unsupported(
        "Realtek without a valid station MAC",
    ))
}
fn station(low: u32, high: u32) -> Option<Mac> {
    let a = low.to_le_bytes();
    let b = high.to_le_bytes();
    let mac = Mac([a[0], a[1], a[2], a[3], b[0], b[1]]);
    (!mac.is_multicast() && mac.0 != [0; 6]).then_some(mac)
}
/// RxConfig's FIFO, DMA and fetch bits per MAC version, from Linux
/// `rtl_init_rxcfg`, without its `RX_MULTI_EN` (bit 14).
///
/// Linux sets that bit on versions 18..24, 34..38 and 40..53. It lets the
/// chip fetch several receive descriptors at once, and Linux gives each
/// descriptor its own 16 KiB buffer, where this driver packs its buffers
/// 2 KiB apart in one allocation. With this layout an RTL8168H (version 46)
/// with the bit set wrote frames into the buffers of other descriptors: the
/// descriptors it returned were complete and error-free, but their own
/// buffers stayed zero, so every frame arrived empty. With the bit clear it
/// received every frame. The bit is only a fetch optimization, so it stays
/// clear on every version rather than on that one.
fn receive_config(version: u8) -> u32 {
    match version {
        2..=17 => RX_FIFO_THRESHOLD | RX_DMA_BURST,
        40..=53 => RX_128_INTERRUPTS | RX_DMA_BURST | RX_EARLY_OFF,
        61 => RX_FETCH_8125 | RX_DMA_BURST,
        // Bit 11 is RX_PAUSE_SLOT_ON on these: the same bit as RX_EARLY_OFF.
        63..=66 | 70 => RX_FETCH_8125 | RX_DMA_BURST | RX_EARLY_OFF,
        _ => RX_128_INTERRUPTS | RX_DMA_BURST,
    }
}

fn publish<R: Registers, T: Timer>(
    hw: &Hardware<'_, R, T>,
    rx: &Ring,
    tx: &Ring,
    mac: Mac,
) -> Result<(), BlockError> {
    let r = hw.registers;
    let revision = hw.revision;
    r.write8(CONFIG_LOCK, CONFIG_UNLOCKED);
    let result: Result<(), BlockError> = (|| {
        let low = u32::from_le_bytes(mac.0[..4].try_into().expect("MAC low word"));
        let high = u32::from(u16::from_le_bytes([mac.0[4], mac.0[5]]));
        r.write32(MAC4, high);
        r.read8(COMMAND);
        r.write32(MAC0, low);
        r.read8(COMMAND);
        if revision.version == 34 {
            // Linux rtl_rar_exgmac_set: both EXGMAC address representations,
            // including reserved bytes which may keep stale firmware state.
            hw.eri_write(0xe0, 15, low)?;
            hw.eri_write(0xe4, 15, high)?;
            hw.eri_write(
                0xf0,
                15,
                u32::from(u16::from_le_bytes([mac.0[0], mac.0[1]])) << 16,
            )?;
            hw.eri_write(
                0xf4,
                15,
                u32::from_le_bytes(mac.0[2..].try_into().expect("MAC upper word")),
            )?;
        }
        // Linux requires high-before-low descriptor base writes on some systems.
        r.write32(TX_HIGH, (tx.descriptors.physical() >> 32) as u32);
        r.write32(TX_LOW, tx.descriptors.physical() as u32);
        r.write32(RX_HIGH, (rx.descriptors.physical() >> 32) as u32);
        r.write32(RX_LOW, rx.descriptors.physical() as u32);
        Ok(())
    })();
    r.write8(CONFIG_LOCK, CONFIG_LOCKED);
    result?;
    r.read8(COMMAND);
    fence(Ordering::SeqCst);
    r.write8(COMMAND, ENABLE);
    // No promiscuous, error, runt or VLAN-stripping acceptance.
    r.write32(RX_CONFIG, receive_config(revision.version) | RX_ACCEPT);
    r.write32(MAR0, u32::MAX);
    r.write32(MAR0 + 4, u32::MAX);
    let auto_fifo = match (34..=52).contains(&revision.version) {
        true => TX_AUTO_FIFO,
        false => 0,
    };
    r.write32(TX_CONFIG, TX_DMA_BURST | TX_INTER_FRAME_GAP | auto_fifo);
    hw.interrupts_off();
    let command = r.read8(COMMAND);
    if command == u8::MAX {
        return Err(BlockError::Device { status: 255 });
    }
    if command & ENABLE != ENABLE {
        return Err(BlockError::Timeout);
    }
    Ok(())
}
impl Drop for Realtek {
    /// Linux `rtl8169_down`'s register sequence ([`Hardware::stop`]). The
    /// rings are freed only once the MAC reset completed; otherwise they
    /// stay allocated. A function that left the bus reads all ones and
    /// gets no write.
    ///
    /// This drives the controller's registers, so the owner drops it where
    /// its MMIO window is mapped; anywhere else the first read faults.
    fn drop(&mut self) {
        if self.device.read32(TX_CONFIG) == u32::MAX {
            return;
        }
        let hardware = Hardware::new(&self.device, &self.clock, self.revision, self.function);
        if hardware.stop().is_ok() {
            // SAFETY: the completed MAC reset stopped every descriptor and
            // buffer access, and neither ring is used after this.
            unsafe {
                ManuallyDrop::drop(&mut self.receive);
                ManuallyDrop::drop(&mut self.transmit);
            }
        }
    }
}

impl Link for Realtek {
    fn mac(&self) -> Mac {
        self.mac
    }
    fn carrier(&mut self) -> bool {
        self.update_link().unwrap_or(false)
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        check_frame(frame)?;
        // Linux stops the queue while the carrier is off; queued descriptors
        // stay with the controller and leave once the link returns.
        if !self.update_link()? {
            return Err(LinkError::NoCarrier);
        }
        if !self.transmit.available() {
            return Err(LinkError::Busy);
        }
        self.transmit.transmit(frame)?;
        fence(Ordering::SeqCst);
        kick_transmit(&self.device, self.revision);
        Ok(())
    }
    fn signals(&self) -> bool {
        self.signalled
    }
    fn due_us(&self) -> Option<u64> {
        (self.revision.version == 46).then(|| self.clock.now_us().saturating_add(H_RECEIVE_POLL_US))
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.receive_frame(<[u8]>::to_vec)
    }
    /// The stack's receive path: the frame is copied into its reused
    /// buffer, so a received frame allocates nothing.
    fn receive_reusing(&mut self, frame: &mut Vec<u8>) -> Result<bool, LinkError> {
        self.receive_frame(|bytes| {
            frame.clear();
            frame.extend_from_slice(bytes);
        })
        .map(|received| received.is_some())
    }
}

impl Realtek {
    /// The next good frame in the receive ring, passed to `take` while its
    /// buffer is still held.
    fn receive_frame<T>(
        &mut self,
        mut take: impl FnMut(&[u8]) -> T,
    ) -> Result<Option<T>, LinkError> {
        let carrier = self.update_link()?;
        let frame = self.receive.receive_with(&mut take);
        if frame.is_some() {
            return Ok(frame);
        }
        // rtl8169_interrupt acknowledges every event it read (rtl_ack_events
        // of the whole status), so the next frame or link change raises the
        // vector again. Among them are the receive ring's exhaustion and the
        // receive FIFO's overflow: an RTL8168H (XID 541) that had them left
        // pending received nothing more after a download's first burst. A
        // frame that arrived before the acknowledgement is in the ring.
        let hardware = Hardware::new(&self.device, &self.clock, self.revision, self.function);
        if hardware.acknowledge() & LINK_CHANGE != 0 {
            self.link.last_phy_poll = None;
            self.update_link()?;
        }
        let frame = self.receive.receive_with(take);
        // A full ring can accumulate while the loader is busy starting CPUs or
        // preparing the restore. A link-up and completed TX do not establish RX
        // progress. Recover only MAC46, after draining RX and completed TX, and
        // never rewrite live DMA until a bounded MAC reset has succeeded.
        if frame.is_none() && self.revision.version == 46 && carrier {
            match self.receive_watch.check(
                self.clock.now_us(),
                self.receive.packets.saturating_add(self.receive.discarded),
                self.transmit.packets,
                self.transmit.owned(),
            ) {
                recovery::Decision::Wait => {}
                recovery::Decision::Restart => {
                    let mut hardware =
                        Hardware::new(&self.device, &self.clock, self.revision, self.function);
                    recovery::restart(
                        &mut hardware,
                        &mut self.receive,
                        &mut self.transmit,
                        self.mac,
                        self.signalled,
                    )
                    .map_err(|error| {
                        LinkError::Adapter(format!("{} receive recovery: {error}", self.location,))
                    })?;
                    self.link.last_phy_poll = None;
                }
                recovery::Decision::Exhausted => {
                    return Err(LinkError::Adapter(format!(
                        "{} receive stalled after three MAC restarts: {}",
                        self.location,
                        self.diagnostics(),
                    )));
                }
            }
        }
        Ok(frame)
    }
}

#[cfg(test)]
mod tests;

impl From<pci::ConfigError> for Error {
    fn from(error: pci::ConfigError) -> Self {
        Self::Device(BlockError::Config(error))
    }
}
impl From<pci::CapabilityError> for Error {
    fn from(error: pci::CapabilityError) -> Self {
        Self::Device(BlockError::PciCapability(error))
    }
}

#[cfg(test)]
use crate::Clock;
