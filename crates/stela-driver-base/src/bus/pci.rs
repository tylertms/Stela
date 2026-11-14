//! PCI configuration space through ECAM: enumeration, BAR decoding and
//! sizing, capabilities and interrupt discovery. Memory decoding and bus
//! mastering are independent of interrupt policy. The current boot drivers
//! use [`Function::enable_polled`]; interrupt-driven ports must arrange
//! delivery with the mount before enabling their interrupt sources.
use super::mmio::Mmio;
use crate::block::BlockError;
use alloc::vec::Vec;
use core::{
    fmt,
    ops::{Range, RangeInclusive},
};

mod access;
mod inventory;
mod kept;
mod transaction;
pub use access::{ConfigEffect, ConfigError, ConfigFailure};
pub use inventory::{
    EnumerationError, EnumerationFailure, InventoryError, InventoryFailure, InventoryReleaseFailure,
};
mod aspm;
pub mod assignment;
pub mod bus_reset;
pub mod capability;
pub mod hotplug;
pub mod msi;
pub mod msix;
pub mod native;
pub mod owned;
pub mod power;
pub mod regs;
pub mod requester;
pub use aspm::{
    aspm_control, disable_aspm_l1_2_link, disable_aspm_link, link_control_word, link_speed,
    native as native_aspm,
};
pub use capability::{
    Capabilities, Capability, CapabilityError, ExtendedCapabilities, ExtendedCapability, Layout,
};

/// MSI-X Message Control: enable, and mask every vector.
const MSIX_ENABLE: u16 = 1 << 15;
const MSIX_FUNCTION_MASK: u16 = 1 << 14;
/// MSI Message Control: enable, and Multiple Message Enable.
const MSI_ENABLE: u16 = 1 << 0;
const MSI_MULTIPLE_ENABLE: u16 = 0b111 << 4;
/// An MSI-X table entry's bytes.
const MSIX_ENTRY_BYTES: u64 = 16;
/// Memory BAR type field: 64-bit (PCI 3.0 §6.2.5.1).
const BAR_MEMORY_64: u32 = 2 << 1;
const BAR_MEMORY_TYPE: u32 = 3 << 1;
const BAR_PREFETCHABLE: u32 = 1 << 3;
const BAR_IO: u32 = 1;
const BARS: usize = 6;

/// Power Management Control/Status (PCI PM 1.2 §3.2.4).
const PM_CONTROL: u16 = 4;
const PM_STATE: u16 = 3;
const PM_NO_SOFT_RESET: u16 = 1 << 3;
const PM_STATUS: u16 = 1 << 15;
/// D3hot and D2 exit delays (PCI PM 1.2 Table 5-1; Linux `PCI_PM_D3HOT_WAIT`,
/// `PCI_PM_D2_DELAY`).
const D3HOT_DELAY_US: u64 = 10_000;
const D2_DELAY_US: u64 = 200;
const D2: u16 = 2;
const D3HOT: u16 = 3;

pub const CAPABILITY_POWER: u8 = 0x01;
pub const CAPABILITY_MSI: u8 = 0x05;
pub const CAPABILITY_VENDOR: u8 = 0x09;
pub const CAPABILITY_MSIX: u8 = 0x11;
pub const CAPABILITY_EXPRESS: u8 = 0x10;

const HEADER_BRIDGE: u8 = 1;
const SECONDARY_BUS: u16 = 0x19;
/// The PCI Express capability's flags, device control 2 and their fields.
const EXPRESS_FLAGS: u16 = 2;
const EXPRESS_TYPE_SHIFT: u16 = 4;
const EXPRESS_ROOT_PORT: u16 = 4;
const EXPRESS_DOWNSTREAM: u16 = 6;
const EXPRESS_DEVICE_CONTROL_2: u16 = 0x28;
const ARI_FORWARDING: u16 = 1 << 5;
/// What `pcie_flr` and `pci_save_pcie_state` read and write of the same
/// capability.
const EXPRESS_DEVICE_CAPABILITIES: u16 = 4;
const EXPRESS_FLR_CAPABLE: u32 = 1 << 28;
const EXPRESS_DEVICE_CONTROL: u16 = 8;
const EXPRESS_INITIATE_FLR: u16 = 1 << 15;
const EXPRESS_DEVICE_STATUS: u16 = 0xa;
const EXPRESS_TRANSACTIONS_PENDING: u16 = 1 << 5;
const EXPRESS_LINK_CONTROL: u16 = 0x10;
const EXPRESS_LINK_CONTROL_2: u16 = 0x30;
/// What a reset restores of the header after its command register.
const HEADER_REST: u16 = 0x0c;
const HEADER_REST_WORDS: usize = 13;
/// `pcie_flr`'s wait after starting a reset, the first wait of
/// `pci_wait_for_pending` and `PCIE_RESET_READY_POLL_MS`.
const FLR_WAIT_US: u64 = 100_000;
const FLR_READY_POLL_US: u64 = 1_000;
const FLR_READY_US: u64 = 60_000_000;
/// The ARI extended capability's ID.
const EXTENDED_ARI: u16 = 0x0e;
/// The Vendor ID a function still initializing answers with Configuration
/// RRS Software Visibility on.
const VENDOR_NOT_READY: u16 = 0x0001;

/// A function-local reset [`Function::reset`] performs, in Linux
/// `pci_reset_function`'s order.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum ResetMethod {
    /// PCI Express Function Level Reset.
    FunctionLevel,
    /// D3hot to D0 with No_Soft_Reset clear (Linux `pci_pm_reset`).
    PowerManagement,
}

#[derive(Clone, Copy)]
enum Method {
    FunctionLevel { express: u16 },
    PowerManagement { power: u16 },
}
impl Method {
    fn public(self) -> ResetMethod {
        match self {
            Self::FunctionLevel { .. } => ResetMethod::FunctionLevel,
            Self::PowerManagement { .. } => ResetMethod::PowerManagement,
        }
    }
}

/// What Linux's `pci_dev_save_and_disable` keeps of a function before a
/// reset and `pci_dev_restore` writes back after it: the header after its
/// command register, the command register, and the PCI Express device and
/// link controls.
struct Saved {
    header: [u32; HEADER_REST_WORDS],
    command: u16,
    controls: Option<[(u16, u16); 4]>,
}
impl Saved {
    /// Save the function's state, then leave only INTx disabled in its
    /// command register.
    fn save_and_disable<C: ConfigSpace>(
        config: &mut C,
        express: Option<u16>,
    ) -> Result<Self, ConfigError> {
        let saved = Self::save(config, express)?;
        saved.disable(config)?;
        Ok(saved)
    }
    /// Leave only INTx disabled in the command register.
    fn disable<C: ConfigSpace>(&self, config: &mut C) -> Result<(), ConfigError> {
        config.write16(regs::COMMAND, regs::COMMAND_INTX_DISABLE)
    }
    /// Save the function's state.
    fn save<C: ConfigSpace>(config: &C, express: Option<u16>) -> Result<Self, ConfigError> {
        let mut header = [0; HEADER_REST_WORDS];
        for (word, value) in header.iter_mut().enumerate() {
            *value = config.read32(HEADER_REST + word as u16 * 4)?;
        }
        let command = config.read16(regs::COMMAND)?;
        let controls = match express {
            Some(express) => {
                let mut controls = [(0, 0); 4];
                for (index, control) in [
                    EXPRESS_DEVICE_CONTROL,
                    EXPRESS_LINK_CONTROL,
                    EXPRESS_DEVICE_CONTROL_2,
                    EXPRESS_LINK_CONTROL_2,
                ]
                .into_iter()
                .enumerate()
                {
                    controls[index] = (express + control, config.read16(express + control)?);
                }
                Some(controls)
            }
            None => None,
        };
        Ok(Self {
            header,
            command,
            controls,
        })
    }
    /// The writes [`Self::restore`] makes, in order, each an offset, a width
    /// in bytes and a value: the command register last.
    fn writes(&self) -> Vec<(u16, u8, u32)> {
        let header = self
            .header
            .into_iter()
            .enumerate()
            .map(|(word, value)| (HEADER_REST + word as u16 * 4, 4, value));
        let controls = self
            .controls
            .into_iter()
            .flatten()
            .map(|(control, value)| (control, 2, u32::from(value)));
        header
            .chain(controls)
            .chain([(regs::COMMAND, 2, u32::from(self.command))])
            .collect()
    }
    /// Write the saved state back.
    fn restore<C: ConfigSpace>(&self, config: &mut C) -> Result<(), ConfigError> {
        for (offset, width, value) in self.writes() {
            match width {
                4 => config.write32(offset, value)?,
                _ => config.write16(offset, value as u16)?,
            }
        }
        Ok(())
    }
}

/// The PCI Express capability's offset, from the complete checked list.
fn express_offset<C: ConfigSpace>(config: &C) -> Result<Option<u16>, CapabilityError> {
    let mut express = None;
    for entry in capability::capabilities(config) {
        let entry = entry?;
        if entry.id == CAPABILITY_EXPRESS && express.is_none() {
            express = Some(entry.offset);
        }
    }
    Ok(express)
}

/// Linux `pci_dev_wait`: poll the Vendor ID, waits doubling from 1 ms, until
/// the function answers with neither all-ones nor the Configuration RRS
/// value. False once `limit` microseconds have passed without an answer.
fn wait_ready<C: ConfigSpace>(
    config: &C,
    delay: &mut impl FnMut(u64) -> Result<(), BlockError>,
    limit: u64,
) -> Result<bool, BlockError> {
    let mut poll = FLR_READY_POLL_US;
    let mut waited = 0;
    while matches!(config.read16(regs::VENDOR_ID)?, 0xffff | VENDOR_NOT_READY) {
        if waited >= limit {
            return Ok(false);
        }
        delay(poll)?;
        waited += poll;
        poll *= 2;
    }
    Ok(true)
}

/// One ECAM window of the ACPI MCFG table.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct EcamWindow {
    pub base: u64,
    pub segment: u16,
    pub start_bus: u8,
    pub end_bus: u8,
}

impl EcamWindow {
    /// The configuration bytes this window decodes.
    pub fn range(&self) -> Range<u64> {
        let start = self.base + (u64::from(self.start_bus) << 20);
        start..self.base + ((u64::from(self.end_bus) + 1) << 20)
    }

    /// The configuration space of one function in this window.
    ///
    /// # Safety
    /// The window is mapped, and the caller excludes every other accessor of
    /// the function while it holds the configuration.
    pub unsafe fn config(&self, bus: u8, device: u8, function: u8) -> EcamConfig {
        let offset =
            (u64::from(bus) << 20) | (u64::from(device) << 15) | (u64::from(function) << 12);
        // SAFETY: forwarded.
        unsafe { EcamConfig::raw(self.base + offset) }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq, PartialOrd, Ord)]
pub struct Address {
    pub segment: u16,
    pub bus: u8,
    pub device: u8,
    pub function: u8,
}

impl Address {
    /// The address its [`Display`](fmt::Display) form writes,
    /// `ssss:bb:dd.f`.
    pub fn parse(text: &str) -> Option<Self> {
        let (segment, rest) = text.split_once(':')?;
        let (bus, rest) = rest.split_once(':')?;
        let (device, function) = rest.split_once('.')?;
        let address = Self {
            segment: u16::from_str_radix(segment, 16).ok()?,
            bus: u8::from_str_radix(bus, 16).ok()?,
            device: u8::from_str_radix(device, 16).ok()?,
            function: function.parse().ok()?,
        };
        (address.device < 32 && address.function < 8).then_some(address)
    }
}

impl fmt::Display for Address {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(
            f,
            "{:04x}:{:02x}:{:02x}.{}",
            self.segment, self.bus, self.device, self.function
        )
    }
}

/// Exact-width access to one function's current configuration permission.
/// Access/ownership failures are distinct from successful hardware all-ones.
/// Native narrow writes must never replay adjacent W1C status through RMW.
/// No operation may replace denial with a sentinel, success or a panic.
pub trait ConfigSpace {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError>;
    fn read16(&self, offset: u16) -> Result<u16, ConfigError>;
    fn read32(&self, offset: u16) -> Result<u32, ConfigError>;
    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError>;
    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError>;
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError>;

    fn capabilities(&self) -> Capabilities<capability::Config<'_, Self>>
    where
        Self: Sized,
    {
        capability::capabilities(self)
    }

    fn extended_capabilities(&self) -> ExtendedCapabilities<capability::Config<'_, Self>>
    where
        Self: Sized,
    {
        capability::extended_capabilities(self)
    }
}

/// Linux `pci_power_up` over any configuration transport: a function left
/// in D1-D3hot is brought to D0 with the whole PMCSR written as zero (PME
/// generation off; a pending PME_Status, write-one-to-clear, stays pending),
/// the state's exit delay waited, and the BARs a D3hot soft reset cleared
/// written back. A function without a power management capability is
/// always in D0. [`Function::power_up`] and drivers that own their
/// configuration transport share this.
pub fn power_up_config<C: ConfigSpace>(
    config: &mut C,
    mut delay: impl FnMut(u64) -> Result<(), BlockError>,
) -> Result<(), BlockError> {
    let Some(offset) = capability::capabilities(&*config)
        .find_map(|entry| match entry {
            Ok(entry) if entry.id == CAPABILITY_POWER => Some(Ok(entry.offset)),
            Ok(_) => None,
            Err(error) => Some(Err(error)),
        })
        .transpose()?
    else {
        return Ok(());
    };
    let mut config = transaction::Transaction::new(config);
    let control = offset + PM_CONTROL;
    let value = config.read16(control)?;
    if value == u16::MAX {
        return Err(BlockError::Unsupported("PCI function does not answer"));
    }
    let state = value & PM_STATE;
    if state == 0 {
        return Ok(());
    }
    let mut bars = [0; BARS];
    for (index, bar) in bars.iter_mut().enumerate() {
        *bar = config.read32(regs::BASE_ADDRESS_0 + 4 * index as u16)?;
    }
    config.write16(control, 0)?;
    delay(match state {
        D3HOT => D3HOT_DELAY_US,
        D2 => D2_DELAY_US,
        _ => 0,
    })?;
    if config.read16(control)? & PM_STATE != 0 {
        return Err(BlockError::Unsupported("PCI function did not enter D0"));
    }
    if state == D3HOT
        && value & PM_NO_SOFT_RESET == 0
        && config.read8(regs::HEADER_TYPE)? & 0x7f == 0
    {
        for (index, bar) in bars.into_iter().enumerate() {
            config.write32(regs::BASE_ADDRESS_0 + 4 * index as u16, bar)?;
        }
    }
    Ok(())
}

/// A function's 4 KiB of ECAM configuration space.
/// A context-bound config transport. Owned and kept-owned arms delegate
/// every effect to the original mapping owner; no copied pointer bypass exists.
pub struct EcamConfig {
    transport: EcamTransport,
    _context: core::marker::PhantomData<*mut ()>,
}
enum EcamTransport {
    Raw(Mmio),
    Owned(owned::OwnedConfig),
    Retained(&'static mut owned::OwnedConfig),
}
impl EcamConfig {
    pub fn from_owned(config: owned::OwnedConfig) -> Self {
        Self {
            transport: EcamTransport::Owned(config),
            _context: core::marker::PhantomData,
        }
    }
    /// Borrow exactly one whole-boot configuration owner. The exclusive
    /// borrow cannot outlive or duplicate its original resource grant.
    pub fn from_retained(config: &'static mut owned::OwnedConfig) -> Self {
        Self {
            transport: EcamTransport::Retained(config),
            _context: core::marker::PhantomData,
        }
    }
    pub fn into_owned(self) -> Result<owned::OwnedConfig, Self> {
        match self.transport {
            EcamTransport::Owned(config) => Ok(config),
            _ => Err(self),
        }
    }
    unsafe fn raw(base: u64) -> Self {
        Self {
            // SAFETY: caller owns this live mapped UC ECAM function
            // and excludes every competing accessor for the transport life.
            transport: EcamTransport::Raw(unsafe { Mmio::new(base, 4096) }),
            _context: core::marker::PhantomData,
        }
    }
}
fn ecam_offset(offset: u16, width: usize) -> Result<usize, ConfigError> {
    let start = usize::from(offset);
    if !start.is_multiple_of(width) || start.checked_add(width).is_none_or(|end| end > 4096) {
        return Err(ConfigError::before_effect(ConfigFailure::Access {
            offset,
            width,
        }));
    }
    Ok(start)
}
impl ConfigSpace for EcamConfig {
    fn read8(&self, offset: u16) -> Result<u8, ConfigError> {
        match &self.transport {
            EcamTransport::Raw(registers) => Ok(registers.read8(ecam_offset(offset, 1)?)),
            EcamTransport::Owned(config) => config.read8(offset),
            EcamTransport::Retained(config) => config.read8(offset),
        }
    }
    fn write8(&mut self, offset: u16, value: u8) -> Result<(), ConfigError> {
        match &mut self.transport {
            EcamTransport::Raw(registers) => {
                registers.write8(ecam_offset(offset, 1)?, value);
                Ok(())
            }
            EcamTransport::Owned(config) => config.write8(offset, value),
            EcamTransport::Retained(config) => config.write8(offset, value),
        }
    }

    fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
        match &self.transport {
            EcamTransport::Raw(registers) => Ok(registers.read16(ecam_offset(offset, 2)?)),
            EcamTransport::Owned(config) => config.read16(offset),
            EcamTransport::Retained(config) => config.read16(offset),
        }
    }
    fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
        match &mut self.transport {
            EcamTransport::Raw(registers) => {
                registers.write16(ecam_offset(offset, 2)?, value);
                Ok(())
            }
            EcamTransport::Owned(config) => config.write16(offset, value),
            EcamTransport::Retained(config) => config.write16(offset, value),
        }
    }

    fn read32(&self, offset: u16) -> Result<u32, ConfigError> {
        match &self.transport {
            EcamTransport::Raw(registers) => Ok(registers.read32(ecam_offset(offset, 4)?)),
            EcamTransport::Owned(config) => config.read32(offset),
            EcamTransport::Retained(config) => config.read32(offset),
        }
    }
    fn write32(&mut self, offset: u16, value: u32) -> Result<(), ConfigError> {
        match &mut self.transport {
            EcamTransport::Raw(registers) => {
                registers.write32(ecam_offset(offset, 4)?, value);
                Ok(())
            }
            EcamTransport::Owned(config) => config.write32(offset, value),
            EcamTransport::Retained(config) => config.write32(offset, value),
        }
    }
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Bar {
    Memory {
        address: u64,
        size: u64,
        prefetchable: bool,
        wide: bool,
    },
    Io {
        port: u32,
        size: u32,
    },
}

impl Bar {
    /// The bytes a memory BAR decodes; none for I/O or a range past 2^64.
    pub fn memory(&self) -> Option<Range<u64>> {
        match *self {
            Self::Memory { address, size, .. } => Some(address..address.checked_add(size)?),
            Self::Io { .. } => None,
        }
    }
}

/// The size a BAR decodes: its lowest writable address bit, as Linux
/// `pci_size` takes it, so read-only upper address bits cannot inflate it.
fn bar_size(writable: u64) -> u64 {
    writable.isolate_lowest_one()
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Msi {
    pub offset: u8,
    pub vectors: u8,
    pub wide: bool,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct MsiX {
    pub offset: u8,
    pub vectors: u16,
    pub table_bar: u8,
    pub table_offset: u32,
    pub pending_bar: u8,
    pub pending_offset: u32,
}

/// The message-signalled interrupts a function offers. Discovery alone
/// neither programs nor enables interrupt delivery.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
pub struct Interrupts {
    pub msi: Option<Msi>,
    pub msix: Option<MsiX>,
}

/// Copied discovery observation, never proof of current configuration,
/// mapping, controller or reset ownership. Revalidate after owned attachment.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Identity {
    pub address: Address,
    pub vendor: u16,
    pub device: u16,
    pub class: [u8; 3],
}

/// One present PCI function.
pub struct Function<C = EcamConfig> {
    pub address: Address,
    pub vendor: u16,
    pub device: u16,
    /// Base class, subclass and programming interface.
    pub class: [u8; 3],
    config: C,
}
impl<C> fmt::Debug for Function<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("Function")
            .field("address", &self.address)
            .field("vendor", &self.vendor)
            .field("device", &self.device)
            .field("class", &self.class)
            .finish_non_exhaustive()
    }
}
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum FunctionError {
    Absent,
    ConfigurationRetry,
    Access(ConfigError),
}
/// The exact configuration owner returns on absence, CRS or access failure.
pub struct FunctionRefusal<C> {
    pub reason: FunctionError,
    pub config: C,
}
impl<C> fmt::Debug for FunctionRefusal<C> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.debug_struct("FunctionRefusal")
            .field("reason", &self.reason)
            .finish_non_exhaustive()
    }
}

impl<C> Function<C> {
    /// Move the same configuration through an owning phase conversion without
    /// new reads. Refusal returns its original configuration and identity data.
    /// The callback retains custody across its own effects and unwind; observed
    /// identity data grants no resource or access permission.
    pub fn try_map_config<D, E>(
        self,
        transform: impl FnOnce(C) -> Result<D, (E, C)>,
    ) -> Result<Function<D>, (E, Self)> {
        let Function {
            address,
            vendor,
            device,
            class,
            config,
        } = self;
        match transform(config) {
            Ok(config) => Ok(Function {
                address,
                vendor,
                device,
                class,
                config,
            }),
            Err((error, config)) => Err((
                error,
                Function {
                    address,
                    vendor,
                    device,
                    class,
                    config,
                },
            )),
        }
    }

    /// Copy the already observed fields without additional config reads.
    /// This data grants no configuration or takeover capability.
    pub fn identity(&self) -> Identity {
        Identity {
            address: self.address,
            vendor: self.vendor,
            device: self.device,
            class: self.class,
        }
    }
}

impl<C: ConfigSpace> Function<C> {
    /// Return the config owner on absent/CRS responses or access denial. No device
    /// quiescence or mapping-retirement permission follows from either value.
    pub fn new(address: Address, config: C) -> Result<Self, FunctionRefusal<C>> {
        let mut retained = kept::Kept::new(config);
        let observed = (|| {
            let config = retained.get_mut();
            let id = config
                .read32(regs::VENDOR_ID)
                .map_err(FunctionError::Access)?;
            if id as u16 == u16::MAX {
                return Err(FunctionError::Absent);
            }
            if id as u16 == VENDOR_NOT_READY {
                return Err(FunctionError::ConfigurationRetry);
            }
            let class = config
                .read32(regs::CLASS_REVISION)
                .map_err(FunctionError::Access)?;
            Ok((id, class))
        })();
        let config = retained.take();
        match observed {
            Ok((id, class)) => Ok(Self {
                address,
                vendor: id as u16,
                device: (id >> 16) as u16,
                class: [(class >> 24) as u8, (class >> 16) as u8, (class >> 8) as u8],
                config,
            }),
            Err(reason) => Err(FunctionRefusal { reason, config }),
        }
    }

    pub fn config(&mut self) -> &mut C {
        &mut self.config
    }

    pub(crate) fn config_ref(&self) -> &C {
        &self.config
    }

    /// Transfer the original configuration owner without creating another
    /// access path or ending its mapping/resource lifetime.
    pub fn into_config(self) -> C {
        self.config
    }

    fn header_type(&self) -> Result<u8, ConfigError> {
        Ok(self.config.read8(regs::HEADER_TYPE)? & 0x7f)
    }

    /// Decode and size every BAR. Failure can leave decoding disabled or a
    /// probe value published; retain this owner and do not retry as unchanged.
    pub fn bars(&mut self) -> Result<[Option<Bar>; BARS], ConfigError> {
        let mut bars = [None; BARS];
        if self.header_type()? != 0 {
            return Ok(bars);
        }
        let mut config = transaction::Transaction::new(&mut self.config);
        let command = config.read16(regs::COMMAND)?;
        config.write16(
            regs::COMMAND,
            command & !(regs::COMMAND_IO | regs::COMMAND_MEMORY),
        )?;
        let mut index = 0;
        while index < BARS {
            let offset = regs::BASE_ADDRESS_0 + 4 * index as u16;
            let (low, low_mask) = Self::size_register(&mut config, offset)?;
            if low & BAR_IO != 0 {
                let size = bar_size(u64::from(low_mask & !3));
                if size != 0 {
                    bars[index] = Some(Bar::Io {
                        port: low & !3,
                        size: size as u32,
                    });
                }
                index += 1;
                continue;
            }
            let wide = low & BAR_MEMORY_TYPE == BAR_MEMORY_64;
            if wide && index + 1 == BARS {
                break;
            }
            let (high, high_mask) = if wide {
                Self::size_register(&mut config, offset + 4)?
            } else {
                (0, 0)
            };
            let size = bar_size(u64::from(high_mask) << 32 | u64::from(low_mask & !15));
            if size != 0 {
                bars[index] = Some(Bar::Memory {
                    address: u64::from(high) << 32 | u64::from(low & !15),
                    size,
                    prefetchable: low & BAR_PREFETCHABLE != 0,
                    wide,
                });
            }
            index += if wide { 2 } else { 1 };
        }
        config.write16(regs::COMMAND, command)?;
        Ok(bars)
    }
    fn size_register(
        config: &mut impl ConfigSpace,
        offset: u16,
    ) -> Result<(u32, u32), ConfigError> {
        let value = config.read32(offset)?;
        config.write32(offset, u32::MAX)?;
        let mask = config.read32(offset)?;
        config.write32(offset, value)?;
        Ok((value, mask))
    }
    /// Bring the function to D0 as Linux `pci_power_up` does: clear the whole
    /// PMCSR (PME_Status is write-one-to-clear and stays pending), wait the
    /// state's exit delay, and restore the BARs a D3hot soft reset cleared.
    /// Functions without a power management capability are always in D0.
    pub fn power_up(&mut self, timer: &(impl power::Timer + ?Sized)) -> Result<(), BlockError> {
        timer.check()?;
        self.power_up_with(|microseconds| timer.wait_us(microseconds).map_err(BlockError::from))
    }
    fn power_up_with(
        &mut self,
        delay: impl FnMut(u64) -> Result<(), BlockError>,
    ) -> Result<(), BlockError> {
        power_up_config(&mut self.config, delay)
    }
    /// Reset this function alone, as Linux's `pcie_flr` inside
    /// `pci_dev_save_and_disable` and `pci_dev_restore`: requests stopped,
    /// their completions waited for, the reset, then the header, device and
    /// link controls back as they were. MSI and MSI-X come back off: the
    /// caller enables them again and rewrites a table in its BAR. False when
    /// the function has no such reset, having touched nothing. The caller's
    /// own use of the function has ended; this sleeps at least 100 ms.
    pub fn function_level_reset(
        &mut self,
        timer: &(impl power::Timer + ?Sized),
    ) -> Result<bool, BlockError> {
        // A nominal rate or capability snapshot cannot replace a real minimum
        // delay. Refuse before even the first configuration observation.
        timer.check()?;
        self.function_level_reset_with(|microseconds| {
            timer.wait_us(microseconds).map_err(BlockError::from)
        })
    }
    fn function_level_reset_with(
        &mut self,
        delay: impl FnMut(u64) -> Result<(), BlockError>,
    ) -> Result<bool, BlockError> {
        match self.method()? {
            Some(method @ Method::FunctionLevel { .. }) => {
                self.reset_by(method, delay)?;
                Ok(true)
            }
            _ => Ok(false),
        }
    }
    /// The function-local reset Linux `pci_reset_function` would pick, from
    /// the ones offered here: FLR, then a D3hot to D0 power management reset
    /// when the function is in D0 and No_Soft_Reset is clear. None when it
    /// has neither. Reads only.
    pub fn reset_method(&self) -> Result<Option<ResetMethod>, BlockError> {
        Ok(self.method()?.map(Method::public))
    }
    /// Reset this function alone by [`Self::reset_method`]'s method, with
    /// [`Self::function_level_reset`]'s save, wait and restore. None when the
    /// function has no such reset, having touched nothing.
    pub fn reset(
        &mut self,
        timer: &(impl power::Timer + ?Sized),
    ) -> Result<Option<ResetMethod>, BlockError> {
        timer.check()?;
        self.reset_with(|microseconds| timer.wait_us(microseconds).map_err(BlockError::from))
    }
    fn reset_with(
        &mut self,
        delay: impl FnMut(u64) -> Result<(), BlockError>,
    ) -> Result<Option<ResetMethod>, BlockError> {
        let Some(method) = self.method()? else {
            return Ok(None);
        };
        self.reset_by(method, delay)?;
        Ok(Some(method.public()))
    }
    fn method(&self) -> Result<Option<Method>, BlockError> {
        let capabilities = self.capabilities()?;
        let find = |wanted| {
            capabilities
                .iter()
                .find(|&&(_, id)| id == wanted)
                .map(|&(offset, _)| u16::from(offset))
        };
        if let Some(express) = find(CAPABILITY_EXPRESS) {
            let flags = self.config.read32(express + EXPRESS_DEVICE_CAPABILITIES)?;
            if flags == u32::MAX {
                return Err(BlockError::Unsupported("PCI function does not answer"));
            }
            if flags & EXPRESS_FLR_CAPABLE != 0 {
                return Ok(Some(Method::FunctionLevel { express }));
            }
        }
        if let Some(power) = find(CAPABILITY_POWER) {
            let control = self.config.read16(power + PM_CONTROL)?;
            if control == u16::MAX {
                return Err(BlockError::Unsupported("PCI function does not answer"));
            }
            // `pci_pm_reset` refuses a function outside D0 or one whose
            // D3hot exit keeps its state.
            if control & (PM_STATE | PM_NO_SOFT_RESET) == 0 {
                return Ok(Some(Method::PowerManagement { power }));
            }
        }
        Ok(None)
    }
    /// `pci_dev_save_and_disable`, the reset, `pci_dev_wait` and
    /// `pci_dev_restore`, for either method.
    fn reset_by(
        &mut self,
        method: Method,
        mut delay: impl FnMut(u64) -> Result<(), BlockError>,
    ) -> Result<(), BlockError> {
        let express = self.express_offset()?;
        let mut config = transaction::Transaction::new(&mut self.config);
        let saved = Saved::save_and_disable(&mut config, express)?;
        match method {
            Method::FunctionLevel { express } => {
                for attempt in 0..3 {
                    let status = config.read16(express + EXPRESS_DEVICE_STATUS)?;
                    if status & EXPRESS_TRANSACTIONS_PENDING == 0 {
                        break;
                    }
                    delay(FLR_WAIT_US << attempt)?;
                }
                let control = express + EXPRESS_DEVICE_CONTROL;
                let original = config.read16(control)?;
                config.write16(control, original | EXPRESS_INITIATE_FLR)?;
                delay(FLR_WAIT_US)?;
            }
            Method::PowerManagement { power } => {
                // `pci_pm_reset`: D3hot, its delay, D0, the delay again. A
                // pending PME_Status (write-one-to-clear) stays pending.
                let control = power + PM_CONTROL;
                let original = config.read16(control)? & !(PM_STATE | PM_STATUS);
                config.write16(control, original | D3HOT)?;
                delay(D3HOT_DELAY_US)?;
                config.write16(control, original)?;
                delay(D3HOT_DELAY_US)?;
            }
        }
        if !wait_ready(&config, &mut delay, FLR_READY_US)? {
            return Err(BlockError::Unsupported(
                "PCI function did not return from its reset",
            ));
        }
        if let Method::PowerManagement { power } = method
            && config.read16(power + PM_CONTROL)? & PM_STATE != 0
        {
            return Err(BlockError::Unsupported("PCI function did not enter D0"));
        }
        saved.restore(&mut config)?;
        Ok(())
    }
    /// The PCI Express capability's offset, from the complete checked list.
    fn express_offset(&self) -> Result<Option<u16>, CapabilityError> {
        express_offset(&self.config)
    }
    /// Complete checked conventional capability inventory; malformed/access
    /// failures are distinct from an absent capability.
    pub fn capabilities(&self) -> Result<Vec<(u8, u8)>, CapabilityError> {
        capability::capabilities(&self.config)
            .map(|entry| entry.map(|entry| (entry.offset as u8, entry.id)))
            .collect()
    }
    pub fn interrupts(&self) -> Result<Interrupts, CapabilityError> {
        let mut interrupts = Interrupts::default();
        for (offset, id) in self.capabilities()? {
            let base = u16::from(offset);
            let control =
                self.config
                    .read16(base + 2)
                    .map_err(|error| CapabilityError::Config {
                        offset: base + 2,
                        error,
                    })?;
            match id {
                CAPABILITY_MSI => {
                    interrupts.msi = Some(Msi {
                        offset,
                        vectors: 1 << ((control >> 1) & 7).min(5),
                        wide: control & (1 << 7) != 0,
                    })
                }
                CAPABILITY_MSIX => {
                    let table =
                        self.config
                            .read32(base + 4)
                            .map_err(|error| CapabilityError::Config {
                                offset: base + 4,
                                error,
                            })?;
                    let pending =
                        self.config
                            .read32(base + 8)
                            .map_err(|error| CapabilityError::Config {
                                offset: base + 8,
                                error,
                            })?;
                    interrupts.msix = Some(MsiX {
                        offset,
                        vectors: (control & 0x7ff) + 1,
                        table_bar: (table & 7) as u8,
                        table_offset: table & !7,
                        pending_bar: (pending & 7) as u8,
                        pending_offset: pending & !7,
                    });
                }
                _ => {}
            }
        }
        Ok(interrupts)
    }
    /// Narrow Command update and exact readback. Failure retains this owner;
    /// no denied write is hidden behind a successful old register value.
    pub fn set_bus_master(&mut self, enabled: bool) -> Result<(), ConfigError> {
        let mut config = transaction::Transaction::new(&mut self.config);
        let previous = config.read16(regs::COMMAND)?;
        let desired = if enabled {
            previous | regs::COMMAND_MASTER
        } else {
            previous & !regs::COMMAND_MASTER
        };
        config.write16(regs::COMMAND, desired)?;
        let observed = config.read16(regs::COMMAND)?;
        if observed != desired {
            return Err(ConfigError::write_may_have_applied(
                regs::COMMAND,
                2,
                ConfigFailure::Readback {
                    offset: regs::COMMAND,
                    width: 2,
                    expected: u32::from(desired),
                    observed: u32::from(observed),
                },
            ));
        }
        Ok(())
    }
    pub fn enable_memory_and_bus_master(&mut self) -> Result<(), ConfigError> {
        let mut config = transaction::Transaction::new(&mut self.config);
        let command = config.read16(regs::COMMAND)?;
        config.write16(
            regs::COMMAND,
            command | regs::COMMAND_MEMORY | regs::COMMAND_MASTER,
        )
    }
    /// Enable a polling driver. A failed prefix stays contained with its owner;
    /// no later MSI/MSI-X/command write is issued after the first access denial.
    pub fn enable_polled(&mut self) -> Result<(), BlockError> {
        self.enable_polled_with(regs::COMMAND_MASTER)
    }
    /// [`Self::enable_polled`] with bus mastering left as it is, off after
    /// the device start: for a driver that first stops an engine the
    /// firmware left running, then calls [`Self::enable_bus_master`].
    pub fn enable_polled_decode(&mut self) -> Result<(), BlockError> {
        self.enable_polled_with(0)
    }
    pub fn enable_bus_master(&mut self) -> Result<(), BlockError> {
        let mut config = transaction::Transaction::new(&mut self.config);
        let command = config.read16(regs::COMMAND)?;
        config.write16(regs::COMMAND, command | regs::COMMAND_MASTER)?;
        Ok(())
    }
    fn enable_polled_with(&mut self, master: u16) -> Result<(), BlockError> {
        let interrupts = self.interrupts()?;
        let mut config = transaction::Transaction::new(&mut self.config);
        if let Some(msi) = interrupts.msi {
            let offset = u16::from(msi.offset) + 2;
            let control = config.read16(offset)?;
            config.write16(offset, control & !MSI_ENABLE)?;
        }
        if let Some(msix) = interrupts.msix {
            let offset = u16::from(msix.offset) + 2;
            let control = config.read16(offset)?;
            config.write16(offset, control & !MSIX_ENABLE)?;
        }
        let command = config.read16(regs::COMMAND)?;
        config.write16(
            regs::COMMAND,
            command | regs::COMMAND_MEMORY | master | regs::COMMAND_INTX_DISABLE,
        )?;
        Ok(())
    }
    /// Give up a device whose driver stopped getting answers: MSI and MSI-X
    /// off, INTx disabled and bus mastering off, so it can neither write
    /// memory nor interrupt. Memory decoding stays as it was.
    pub fn abandon(&mut self) -> Result<(), BlockError> {
        let interrupts = self.interrupts()?;
        let mut config = transaction::Transaction::new(&mut self.config);
        if let Some(msi) = interrupts.msi {
            let offset = u16::from(msi.offset) + 2;
            let control = config.read16(offset)?;
            config.write16(offset, control & !MSI_ENABLE)?;
        }
        if let Some(msix) = interrupts.msix {
            let offset = u16::from(msix.offset) + 2;
            let control = config.read16(offset)?;
            config.write16(offset, control & !MSIX_ENABLE)?;
        }
        let command = config.read16(regs::COMMAND)?;
        config.write16(
            regs::COMMAND,
            command & !regs::COMMAND_MASTER | regs::COMMAND_INTX_DISABLE,
        )?;
        Ok(())
    }
    /// Keep all vectors masked while programming. A failed callback retains
    /// that masked original state; it cannot authorize cleanup effects or retry.
    pub fn enable_msix<T>(
        &mut self,
        msix: MsiX,
        program: impl FnOnce() -> Result<T, BlockError>,
    ) -> Result<T, BlockError> {
        let control = u16::from(msix.offset) + 2;
        let mut config = transaction::Transaction::new(&mut self.config);
        let value = config.read16(control)?;
        config.write16(control, value | MSIX_ENABLE | MSIX_FUNCTION_MASK)?;
        let programmed = program()?;
        let value = config.read16(control)?;
        config.write16(control, value & !MSIX_FUNCTION_MASK)?;
        Ok(programmed)
    }
    pub fn enable_msi(
        &mut self,
        msi: Msi,
        message: crate::bus::irq::MsiMessage,
    ) -> Result<(), ConfigError> {
        let control = u16::from(msi.offset) + 2;
        let mut config = transaction::Transaction::new(&mut self.config);
        let value = config.read16(control)?;
        config.write16(control, value & !(MSI_ENABLE | MSI_MULTIPLE_ENABLE))?;
        config.write32(u16::from(msi.offset) + 4, message.address as u32)?;
        let data = if msi.wide {
            config.write32(u16::from(msi.offset) + 8, (message.address >> 32) as u32)?;
            u16::from(msi.offset) + 12
        } else {
            u16::from(msi.offset) + 8
        };
        config.write16(data, message.data as u16)?;
        let value = config.read16(control)?;
        config.write16(control, (value & !MSI_MULTIPLE_ENABLE) | MSI_ENABLE)?;
        Ok(())
    }
    /// Program one supported message mechanism. Mapping failures do not become
    /// capability absence; all original access/resource owners remain required.
    pub fn enable_message(
        &mut self,
        handoff: &crate::Handoff,
        bars: &[Option<Bar>],
        message: crate::bus::irq::MsiMessage,
    ) -> Result<bool, BlockError> {
        let interrupts = self.interrupts()?;
        if let Some(msix) = interrupts.msix {
            let bar = bars.get(usize::from(msix.table_bar)).copied().flatten();
            let table =
                crate::registers(handoff, bar, u64::from(msix.table_offset), MSIX_ENTRY_BYTES)?;
            self.enable_msix(msix, || {
                table.write32(0, message.address as u32);
                table.write32(4, (message.address >> 32) as u32);
                table.write32(8, message.data);
                table.write32(12, 0);
                Ok(())
            })?;
            return Ok(true);
        }
        if let Some(msi) = interrupts.msi {
            self.enable_msi(msi, message)?;
            return Ok(true);
        }
        Ok(false)
    }
}
/// How a bus's functions are found, from the bridge above it.
#[derive(Clone, Copy, PartialEq, Eq)]
enum BusScan {
    /// Every device number: a root bus or below a conventional bridge.
    All,
    /// Device 0 alone, below a PCI Express root or downstream port, whose
    /// link has one device. Some devices answer at every device number
    /// there, as Linux's `only_one_child` notes.
    OneChild,
    /// Device 0's functions by their ARI chain, below a port forwarding ARI.
    Ari,
}

/// A capability list's own malformation, as opposed to a configuration
/// read that failed: Linux's `__pci_find_next_cap_ttl` stops at such a link
/// and keeps what it found before it.
fn malformed(error: &CapabilityError) -> bool {
    !matches!(error, CapabilityError::Config { .. })
}

impl<C: ConfigSpace> Function<C> {
    /// The extended capability `id` found before the list ends or turns out
    /// malformed. Only a failed configuration read is an error.
    fn extended_capability(&self, id: u16) -> Result<Option<u16>, CapabilityError> {
        for entry in capability::extended_capabilities(&self.config) {
            match entry {
                Ok(entry) if entry.id == id => return Ok(Some(entry.offset)),
                Ok(_) => {}
                Err(error) if malformed(&error) => return Ok(None),
                Err(error) => return Err(error),
            }
        }
        Ok(None)
    }
    /// The PCI Express capability found before the standard list ends or
    /// turns out malformed. Only a failed configuration read is an error.
    fn express_capability(&self) -> Result<Option<u16>, CapabilityError> {
        for entry in capability::capabilities(&self.config) {
            match entry {
                Ok(entry) if entry.id == CAPABILITY_EXPRESS => return Ok(Some(entry.offset)),
                Ok(_) => {}
                Err(error) if malformed(&error) => return Ok(None),
                Err(error) => return Err(error),
            }
        }
        Ok(None)
    }
    /// A bridge's secondary bus and how to scan it. A malformed capability
    /// list hides the port type, so the bus is scanned as below a
    /// conventional bridge: every device number, as Linux does without a
    /// PCI Express capability. The function's driver meets the same list
    /// again and reports it; the rest of the segment is still found.
    fn secondary(&self) -> Result<Option<(u8, BusScan)>, CapabilityError> {
        let read8 = |offset| {
            self.config
                .read8(offset)
                .map_err(|error| CapabilityError::Config { offset, error })
        };
        let read16 = |offset| {
            self.config
                .read16(offset)
                .map_err(|error| CapabilityError::Config { offset, error })
        };
        if read8(regs::HEADER_TYPE)? & 0x7f != HEADER_BRIDGE {
            return Ok(None);
        }
        let bus = read8(SECONDARY_BUS)?;
        let express = self.express_capability()?;
        let scan = match express {
            Some(express) => {
                let kind = (read16(express + EXPRESS_FLAGS)? >> EXPRESS_TYPE_SHIFT) & 0xf;
                let control = read16(express + EXPRESS_DEVICE_CONTROL_2)?;
                match kind {
                    EXPRESS_ROOT_PORT | EXPRESS_DOWNSTREAM if control & ARI_FORWARDING != 0 => {
                        BusScan::Ari
                    }
                    EXPRESS_ROOT_PORT | EXPRESS_DOWNSTREAM => BusScan::OneChild,
                    _ => BusScan::All,
                }
            }
            None => BusScan::All,
        };
        Ok(Some((bus, scan)))
    }
}
/// The MCFG's windows narrowed to host bridges: for each root bridge, given
/// as its segment and its bus range (the bus-number descriptor of its
/// `_CRS`), the part of its segment's window that decodes the range, from
/// its root bus. Bridges no window decodes are left out.
pub fn rooted(windows: &[EcamWindow], bridges: &[(u16, RangeInclusive<u8>)]) -> Vec<EcamWindow> {
    bridges
        .iter()
        .filter_map(|(segment, buses)| {
            let window = windows.iter().find(|window| {
                window.segment == *segment
                    && (window.start_bus..=window.end_bus).contains(buses.start())
            })?;
            Some(EcamWindow {
                start_bus: *buses.start(),
                end_bus: window.end_bus.min(*buses.end()),
                ..*window
            })
        })
        .collect()
}

#[cfg(test)]
mod tests;
