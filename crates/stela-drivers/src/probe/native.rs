//! PCI functions whose drivers attach through [`NativeServices`]: each takes
//! a stopped requester, its DMA domain and owned BAR mappings from the mount.
use super::{Error, NativeServices, ServiceError};
#[cfg(feature = "native-acpi")]
use crate::net::wifi;
use crate::{
    Clock,
    bluetooth::{bcm4377, intel_pcie},
    bus::{
        dma::{DomainAllocator, mapping::DmaDomain},
        mmio::owned::{CacheAttribute, Mapping, Protection},
        pci::{Address, Bar, ConfigSpace, Function, msix::MappedBar, owned::OwnedConfig},
        vmd,
    },
    net::{
        Link, LinkError, Mac,
        aqtion::{self, MappedRegisters},
        tg3,
        wifi::{mediatek, realtek as realtek_wifi},
    },
    storage::nvme,
    usb::ehci,
};
use alloc::{boxed::Box, rc::Rc, string::String, sync::Arc, vec::Vec};
use core::{any::Any, cell::RefCell};

pub mod hda;
pub mod usb4;

/// A Broadcom Tigon3 adapter over owned BAR0 and APE mappings.
pub type Tg3 = tg3::Packet<tg3::io::Mmio<OwnedConfig>, Rc<dyn crate::bus::pci::power::Timer>>;

/// A link driver and the PCI location it was found at.
pub struct Located<L> {
    pub location: String,
    pub link: L,
}

impl<L: Link> Link for Located<L> {
    fn mac(&self) -> Mac {
        self.link.mac()
    }
    fn carrier(&mut self) -> bool {
        self.link.carrier()
    }
    fn carrier_wait_us(&self) -> u64 {
        self.link.carrier_wait_us()
    }
    fn signals(&self) -> bool {
        self.link.signals()
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        self.link.transmit(frame)
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        self.link.receive()
    }
}

/// The drivers here that attach through [`NativeServices`].
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Kind {
    Tg3,
    Aqtion,
    Vmd,
    /// AMD Sensor Fusion Hub v2.
    Sfh,
    /// Intel Integrated Sensor Hub.
    Ish,
    /// An Intel Wi-Fi function, which `iwl_pci_probe` identifies further.
    IntelWifi,
    /// A MediaTek Wi-Fi function; only MT7925 attaches.
    MediatekWifi,
    /// A Realtek Wi-Fi function; only RTL8822C attaches.
    RealtekWifi,
    /// An Intel PCIe Bluetooth function, as `btintel_pcie_table` lists them.
    IntelBluetooth,
    /// A Broadcom PCIe Bluetooth function, as `bcm4377_devid_table` lists
    /// them.
    Bcm4377,
    /// An HD Audio controller the bounded identity table admits.
    Hda,
    /// A USB4/Thunderbolt host interface, as `pci.c::nhi_ids` lists them.
    Usb4,
}

impl Kind {
    pub(super) fn of(function: &Function) -> Option<Self> {
        Self::of_identity(&function.identity())
    }

    /// [`Self::of`] from a function's copied header fields.
    pub(super) fn of_identity(function: &crate::bus::pci::Identity) -> Option<Self> {
        if vmd::Profile::identify(function.vendor, function.device).is_ok() {
            Some(Self::Vmd)
        } else if tg3::candidate(function.vendor, function.device, function.class) {
            Some(Self::Tg3)
        } else if aqtion::Profile::identify(function.vendor, function.device).is_some()
            && function.class == [2, 0, 0]
        {
            Some(Self::Aqtion)
        } else if (function.vendor, function.device) == (0x1022, 0x15e4) {
            Some(Self::Sfh)
        } else if (function.vendor, function.device) == (0x8086, 0x9d35) {
            Some(Self::Ish)
        } else if function.vendor == 0x8086 && function.class[..2] == [2, 0x80] {
            Some(Self::IntelWifi)
        } else if function.vendor == 0x14c3 && function.class[..2] == [2, 0x80] {
            Some(Self::MediatekWifi)
        } else if function.vendor == 0x10ec
            && function.class[..2] == [2, 0x80]
            && realtek_wifi::identity::RTL8822C_DEVICES.contains(&function.device)
        {
            Some(Self::RealtekWifi)
        } else if function.vendor == 0x8086 && intel_pcie::DEVICE_IDS.contains(&function.device) {
            Some(Self::IntelBluetooth)
        } else if function.vendor == 0x14e4
            && function.class[..2] == [2, 0x80]
            && bcm4377::Chip::of_device(function.device).is_some()
        {
            Some(Self::Bcm4377)
        } else if crate::audio::hda::pci::identify_identity(function).is_ok() {
            Some(Self::Hda)
        } else if crate::bus::thunderbolt::controller::Profile::identify(
            function.vendor,
            function.device,
            u32::from_be_bytes([0, function.class[0], function.class[1], function.class[2]]),
        )
        .is_ok()
        {
            Some(Self::Usb4)
        } else {
            None
        }
    }

    /// The driver this kind attaches with, as the boot log names it.
    pub(super) fn driver(self) -> &'static str {
        match self {
            Self::Tg3 => "tg3 Ethernet",
            Self::Aqtion => "atlantic Ethernet",
            Self::Vmd => "Intel VMD storage",
            Self::Sfh => "AMD sensor hub",
            Self::Ish => "Intel sensor hub",
            Self::IntelWifi => "iwlwifi Wi-Fi (with --wifi)",
            Self::MediatekWifi => "mt7925 Wi-Fi (with --wifi)",
            Self::RealtekWifi => "rtw88 Wi-Fi (with --wifi)",
            Self::IntelBluetooth => "Intel PCIe Bluetooth",
            Self::Bcm4377 => "bcm4377 Bluetooth",
            Self::Hda => "HD Audio",
            Self::Usb4 => "USB4/Thunderbolt NHI",
        }
    }
}

/// A PCI bridge the scan passed.
#[derive(Clone, Copy, Debug)]
pub(super) struct Bridge {
    pub(super) address: Address,
    pub(super) secondary: u8,
    /// The ASPM Control field of its PCIe Link Control.
    pub(super) aspm: u16,
}

/// The bridge whose secondary bus is `bus`: Linux's `bus->self`.
fn upstream(bridges: &[Bridge], segment: u16, bus: u8) -> Option<&Bridge> {
    bridges.iter().find(|bridge| {
        bridge.address.segment == segment && bridge.secondary == bus && bridge.address.bus < bus
    })
}

/// `mt76_pci_aspm_supported`: whether firmware enabled ASPM on the
/// function's link, at either end.
pub(super) fn aspm_supported<C: ConfigSpace>(
    function: &Function<C>,
    bridges: &[Bridge],
) -> Result<bool, crate::block::BlockError> {
    let address = function.address;
    Ok(crate::bus::pci::aspm_control(function)? != 0
        || upstream(bridges, address.segment, address.bus).is_some_and(|parent| parent.aspm != 0))
}

fn mediatek_identity<C: ConfigSpace>(
    function: &mut Function<C>,
) -> Result<mediatek::identity::PciIdentity, crate::bus::pci::ConfigError> {
    let (vendor, device) = (function.vendor, function.device);
    let config = function.config();
    Ok(mediatek::identity::PciIdentity {
        vendor,
        device,
        subsystem_vendor: config.read16(0x2c)?,
        subsystem_device: config.read16(0x2e)?,
        revision: config.read8(0x08)?,
    })
}

/// A function claimed from the mount, before any device effect.
struct Claimed {
    function: Function<OwnedConfig>,
    domain: Arc<dyn DmaDomain>,
    bars: [Option<Bar>; 6],
}

#[expect(
    clippy::result_large_err,
    reason = "The shared error preserves exact source and cleanup causes without allocating on propagation."
)]
fn claim(services: &dyn NativeServices, address: Address) -> Result<Claimed, Failure> {
    // The Wi-Fi drivers set Bus Master before they reset their engines, so
    // they produce no quiescence receipt yet: the function stays unmastered,
    // and the seal refuses that write.
    let requester = super::services::claim_unmastered(services, address)
        .map_err(|error| (Error::Native(error), None))?;
    let (requester, binding) = match requester.into_dma_binding() {
        Ok(value) => value,
        Err((error, owner)) => {
            return Err(keep_error(Error::Native(ServiceError::Claim(error)), owner));
        }
    };
    let domain = match services.domain(binding) {
        Ok(domain) => domain,
        Err(error) => return Err(keep_error(Error::Native(error), requester)),
    };
    let mut function = requester.into_function();
    let bars = match function.bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, (function, domain))),
    };
    Ok(Claimed {
        function,
        domain,
        bars,
    })
}

fn map(
    services: &dyn NativeServices,
    address: Address,
    bars: &[Option<Bar>; 6],
    index: usize,
) -> Result<Mapping, ServiceError> {
    match bars[index] {
        Some(bar @ Bar::Memory { .. }) => services.map_bar(address, bar),
        _ => Err(ServiceError::Refused("missing memory BAR")),
    }
}

/// What a failed attach leaves: the error and any owner that must outlive
/// the device because it may hold mappings or DMA.
pub(super) type Failure = (Error, Option<Box<dyn Any>>);

fn keep_error(error: impl Into<Error>, owner: impl Any) -> Failure {
    (error.into(), Some(Box::new(owner)))
}

/// Attach an EHCI host controller with its own requester, BAR0 and DMA arena.
///
/// # Safety
/// [`NativeServices`]' contract holds for `services`, `clock` is the
/// handoff's calibrated clock, and every companion controller of its roots
/// completed the legacy handoff. This path takes the EHCI BIOS semaphore
/// through the original requester and register owners.
#[expect(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn ehci(
    services: &dyn NativeServices,
    address: Address,
    clock: Clock,
) -> Result<ehci::host::Shared, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    let refused = |error: ServiceError| (Error::Native(error), None);
    let mut requester = services.prepare_requester(address).map_err(refused)?;
    let bars = match requester.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, requester)),
    };
    let registers = match map(services, address, &bars, 0) {
        Ok(registers) => registers,
        Err(error) => return Err(keep_error(Error::Native(error), (requester, minimum))),
    };
    let resources = ehci::NativeResources {
        requester,
        registers,
        memory: services.pages(),
        clock,
        minimum,
    };
    // SAFETY: paired original configuration/root/BAR owners and masked/drained
    // IRQ/work exclusion from the cold mount grant. The caller owns the
    // companions. EHCI proves BIOS release and engine HALT, then drains the
    // original root before binding the domain or allocating its arena.
    match unsafe { ehci::attach(resources) } {
        Ok((controller, _)) => Ok(Rc::new(RefCell::new(controller))),
        Err(failure) => Err((Error::Ehci(failure.error), Some(Box::new(failure)))),
    }
}

/// Attach a Tigon3 adapter: BAR0 holds the MAC registers, BAR2 the APE
/// of a chip that has one.
///
/// # Safety
/// [`NativeServices`]' contract holds for `services`, and `clock` is the
/// handoff's calibrated clock.
#[expect(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn tg3(
    services: &dyn NativeServices,
    address: Address,
) -> Result<Located<Tg3>, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    let refused = |error: ServiceError| (Error::Native(error), None);
    let mut requester = services.prepare_requester(address).map_err(refused)?;
    let bars = match requester.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, requester)),
    };
    let mac = match map(services, address, &bars, 0) {
        Ok(mac) => mac,
        Err(error) => return Err(keep_error(Error::Native(error), (requester, minimum))),
    };
    // Only a chip with an APE has BAR 2 (Linux maps it for those alone).
    let ape = if tg3::ape(requester.function_mut().device) {
        match map(services, address, &bars, 2) {
            Ok(ape) => Some(ape),
            Err(error) => return Err(keep_error(Error::Native(error), (requester, mac, minimum))),
        }
    } else {
        None
    };
    let window = |mapping: &Mapping| {
        let bytes = mapping.bytes()?;
        Ok((mapping.address(0, bytes)?, bytes))
    };
    let ((mac_window, mac_bytes), ape_window) =
        match window(&mac).and_then(|mac| Ok((mac, ape.as_ref().map(window).transpose()?))) {
            Ok(windows) => windows,
            Err(error) => {
                return Err(keep_error(
                    Error::Native(ServiceError::Mapping(error)),
                    (requester, mac, ape, minimum),
                ));
            }
        };
    let windows: Arc<dyn Any> = Arc::new((mac, ape));
    // SAFETY: original preparation keeps configuration/root/inherited backing
    // and prior IRQ/work exclusions. Both UC mappings belong to that function.
    let registers = unsafe {
        tg3::io::Cold::from_owned_windows(requester, mac_window, mac_bytes, ape_window, windows)
    }
    .map_err(|(error, requester, windows)| {
        (
            Error::Tg3(error),
            Some(Box::new((requester, windows)) as Box<dyn Any>),
        )
    })?;
    // SAFETY: the cold registers meet their exclusive native ownership
    // contract, and this is the handoff's calibrated clock/context.
    let hardware = unsafe { tg3::Hardware::new(registers, minimum.clone()) };
    let registers = hardware.quiesce_cold().map_err(|(error, hardware)| {
        (Error::Tg3(error), Some(Box::new(hardware) as Box<dyn Any>))
    })?;
    // SAFETY: quiesce_cold acknowledged this original function's vendor
    // engines and BME stop. No IRQ/work or competing config/reset owner exists.
    // Binding first drains the old root while all original windows stay kept.
    let bound = unsafe { registers.bind() }.map_err(|failure| {
        (
            Error::Tg3(tg3::Error::Preparation(failure.error)),
            Some(Box::new(failure) as Box<dyn Any>),
        )
    })?;
    // SAFETY: finalization preserved these exact register owners and supplies
    // their isolated domain/unique attachment. No DMA allocation occurred yet.
    let hardware = unsafe { tg3::Hardware::new(bound.registers, minimum.clone()) };
    // SAFETY: original-root drain and current-epoch binding completed; IRQ/work
    // exclusions stay held. Packet retirement unmaps rings before detaching.
    match unsafe { tg3::Packet::attach_bound(hardware, services.pages(), bound.attachment) } {
        Ok(link) => Ok(Located {
            location: alloc::format!("{address}"),
            link,
        }),
        Err((error, link)) => Err((Error::Tg3(error), Some(Box::new(link)))),
    }
}

/// Attach an Aquantia AQtion adapter over its BAR0 registers.
///
/// # Safety
/// As for [`tg3`]. Original configuration/root/backing and prior firmware,
/// IRQ/work and bypass exclusions stay held through cold preparation.
#[expect(
    clippy::result_large_err,
    reason = "failed native attachment returns the original cold or controller owner"
)]
pub(super) unsafe fn aqtion(
    services: &dyn NativeServices,
    address: Address,
) -> Result<Located<aqtion::native::Attached>, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    let refused = |error: ServiceError| (Error::Native(error), None);
    let mut requester = services.prepare_requester(address).map_err(refused)?;
    let command = match requester
        .function_mut()
        .config()
        .read16(crate::bus::pci::regs::COMMAND)
    {
        Ok(command) => command,
        Err(error) => return Err(keep_error(error, requester)),
    };
    if command == u16::MAX || command & crate::bus::pci::regs::COMMAND_MASTER != 0 {
        return Err((
            Error::Aqtion(aqtion::native::Error::BusMaster(command)),
            Some(Box::new(requester)),
        ));
    }
    // Establish D0 before sizing or mapping the BAR, with bus mastering gated.
    if let Err(error) = requester.function_mut().power_up(minimum.as_ref()) {
        return Err((
            Error::Aqtion(aqtion::native::Error::Power(error)),
            Some(Box::new(requester)),
        ));
    }
    let bars = match requester.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, requester)),
    };
    let registers = map(services, address, &bars, 0).map_err(refused)?;
    let registers = match MappedRegisters::new(registers) {
        Ok(registers) => registers,
        Err((error, mapping)) => {
            return Err((
                Error::Aqtion(aqtion::native::Error::Device(error)),
                Some(Box::new((requester, mapping))),
            ));
        }
    };
    let inputs = aqtion::native::Inputs {
        requester,
        registers,
        timer: minimum,
    };
    // SAFETY: paired original function/root/BAR and calibrated process timer;
    // no Stela IRQ/work or competing writer exists. The driver resets firmware,
    // stops every queue and drains the old root before binding or allocating.
    match unsafe { aqtion::native::attach(inputs, services.pages()) } {
        Ok(link) => Ok(Located {
            location: alloc::format!("{address}"),
            link,
        }),
        Err(failure) => Err((
            Error::Aqtion(failure.error.clone()),
            Some(Box::new(failure)),
        )),
    }
}

/// An Intel AX200/AX210 adapter over its owned BAR0, reading its ACPI
/// companion.
#[cfg(feature = "native-acpi")]
pub type IntelWifi = wifi::intel::link::Station<
    wifi::intel::boot::NativeBootIo,
    wifi::intel::radio::bios::service::native::NativeBios,
>;

/// The owners an MT7925 station keeps: its claimed function and BAR0.
pub type MediatekOwner = (Function<OwnedConfig>, Mapping);

/// An MT7925 station over its owned function and BAR0.
pub type MediatekWifi = mediatek::station::Wifi<MediatekOwner>;

/// Attach a MediaTek Wi-Fi function: BAR0 holds its registers and remap
/// windows.
///
/// # Safety
/// As for [`tg3`]; `inputs` describes this function's own link and ACPI
/// companion.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn mediatek_wifi(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    mut inputs: mediatek::attach::Inputs,
) -> Result<Located<mediatek::attach::Attached<MediatekOwner>>, Failure> {
    let Claimed {
        mut function,
        domain,
        bars,
    } = claim(services, address)?;
    let pci = match mediatek_identity(&mut function) {
        Ok(pci) => pci,
        Err(error) => return Err(keep_error(error, (function, domain))),
    };
    let registers = match map(services, address, &bars, 0) {
        Ok(registers) => registers,
        Err(error) => return Err(keep_error(Error::Native(error), (function, domain))),
    };
    let (base, bytes) = match registers
        .bytes()
        .and_then(|bytes| Ok((registers.address(0, bytes)?, bytes)))
    {
        Ok(window) => window,
        Err(error) => {
            return Err(keep_error(
                Error::Native(ServiceError::Mapping(error)),
                (function, domain, registers),
            ));
        }
    };
    let configured = (|| -> Result<bool, Error> {
        let interrupt = function.interrupts()?.msi;
        function.enable_polled()?;
        match (completion, interrupt) {
            (Some(message), Some(msi)) => {
                function.enable_msi(msi, message)?;
                Ok(true)
            }
            _ => Ok(false),
        }
    })();
    inputs.vector = match configured {
        Ok(vector) => vector,
        Err(error) => return Err(keep_error(error, (function, domain, registers))),
    };
    #[expect(
        clippy::arc_with_non_send_sync,
        reason = "The serialized BAR and queue retainers require Arc<dyn Any>; this PCI custody remains !Send and !Sync."
    )]
    let owner = Arc::new((function, registers));
    let kept = Box::new(owner.clone()) as Box<dyn Any>;
    // SAFETY: the mount claimed the stopped function, mapped its BAR0
    // uncached and supplied an isolating domain; `owner` keeps the function
    // and mapping, and its interrupts are off.
    match unsafe { mediatek::attach::attach(pci, base.cast(), bytes, owner, domain, inputs) } {
        Ok(attached) => Ok(Located {
            location: alloc::format!("{address}"),
            link: attached,
        }),
        Err(error) => Err((Error::MediatekWifi(error), Some(kept))),
    }
}

/// An RTL8822C station on its hardware.
pub type RealtekWifi = realtek_wifi::station::Wifi<realtek_wifi::hardware::Hardware>;

/// Attach an RTL8822C: `rtw_pci_io_mapping` finds its registers in BAR2.
///
/// # Safety
/// As for [`tg3`].
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn realtek_wifi(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    mut inputs: realtek_wifi::attach::Inputs,
) -> Result<Located<RealtekWifi>, Failure> {
    let Claimed {
        mut function,
        domain,
        bars,
    } = claim(services, address)?;
    let registers = match map(services, address, &bars, 2) {
        Ok(registers) => registers,
        Err(error) => return Err(keep_error(Error::Native(error), (function, domain))),
    };
    let (base, bytes) = match registers
        .bytes()
        .and_then(|bytes| Ok((registers.address(0, bytes)?, bytes)))
    {
        Ok(window) => window,
        Err(error) => {
            return Err(keep_error(
                Error::Native(ServiceError::Mapping(error)),
                (function, domain, registers),
            ));
        }
    };
    let configured = (|| -> Result<_, Error> {
        let link_control = crate::bus::pci::link_control_word(&function)?;
        let config = function.config();
        let identity = (
            config.read16(0x2c)?,
            config.read16(0x2e)?,
            config.read8(0x08)?,
        );
        let interrupt = function.interrupts()?.msi;
        function.enable_polled()?;
        let vector = match (completion, interrupt) {
            (Some(message), Some(msi)) => {
                function.enable_msi(msi, message)?;
                true
            }
            _ => false,
        };
        Ok((link_control, identity, vector))
    })();
    let (link_control, (subsystem_vendor, subsystem_device, pci_revision), vector) =
        match configured {
            Ok(configured) => configured,
            Err(error) => return Err(keep_error(error, (function, domain, registers))),
        };
    inputs.link_control = link_control;
    inputs.vector = vector;
    let observed = realtek_wifi::identity::Observed {
        vendor: function.vendor,
        device: function.device,
        subsystem_vendor,
        subsystem_device,
        pci_revision,
        // SAFETY: same owned mapped BAR2, whose first page holds SYS_CFG1.
        sys_cfg1: unsafe { crate::bus::mmio::Mmio::new(base.as_ptr() as u64, 4096) }.read32(0xf0),
    };
    #[expect(
        clippy::arc_with_non_send_sync,
        reason = "The serialized Realtek backend retains Arc<dyn Any>; this PCI custody remains !Send and !Sync."
    )]
    let owner: Arc<dyn Any> = Arc::new((function, registers));
    let kept = Box::new(owner.clone()) as Box<dyn Any>;
    // SAFETY: the mount claimed the stopped function, mapped its BAR2 uncached
    // and supplied an isolating domain; `owner` keeps the function and mapping,
    // and the device's interrupt masks are off until the station starts.
    match unsafe {
        realtek_wifi::attach::attach(base.cast(), bytes, owner, domain, observed, inputs)
    } {
        Ok(link) => Ok(Located {
            location: alloc::format!("{address}"),
            link,
        }),
        Err(error) => Err((Error::RealtekWifi(error), Some(kept))),
    }
}

/// Attach an Intel Wi-Fi function: BAR0 holds its CSRs.
///
/// # Safety
/// As for [`tg3`]; `inputs.bios` reads this function's own ACPI companion.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
#[cfg(feature = "native-acpi")]
pub(super) unsafe fn intel_wifi<B: wifi::intel::radio::bios::service::BiosIo>(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    inputs: wifi::intel::link::Inputs<B>,
) -> Result<Located<wifi::intel::link::Station<wifi::intel::boot::NativeBootIo, B>>, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    use wifi::intel::attach::{AttachError, Pci, attach};
    let Claimed {
        mut function,
        domain,
        bars,
    } = claim(services, address)?;
    let registers = match map(services, address, &bars, 0) {
        Ok(registers) => registers,
        Err(error) => return Err(keep_error(Error::Native(error), (function, domain))),
    };
    let window = match registers.address(0, 16384) {
        Ok(window) => window,
        Err(error) => {
            return Err(keep_error(
                Error::Native(ServiceError::Mapping(error)),
                (function, domain, registers),
            ));
        }
    };
    let configured = (|| -> Result<_, Error> {
        let vendor = function.vendor;
        let device = function.device;
        let link_speed = crate::bus::pci::link_speed(&function)?;
        let config = function.config();
        let pci = Pci {
            vendor,
            device,
            subsystem_vendor: config.read16(0x2c)?,
            subsystem_device: config.read16(0x2e)?,
            revision: config.read8(0x08)?,
            link_speed,
            vector: false,
        };
        let interrupt = function.interrupts()?.msix;
        function.enable_polled()?;
        let vector = match (completion, interrupt) {
            (Some(message), Some(msix)) if msix.table_bar == 0 => {
                let table = registers
                    .address(msix.table_offset as usize, 16)
                    .map_err(|error| Error::Native(ServiceError::Mapping(error)))?;
                // SAFETY: this function's owned uncached MSI-X table stays kept.
                let table = unsafe { crate::bus::mmio::Mmio::new(table.as_ptr() as u64, 16) };
                let vector = IntelVector {
                    msix,
                    table,
                    message,
                };
                vector.enable(&mut function)?;
                Some(vector)
            }
            _ => None,
        };
        Ok((
            Pci {
                vector: vector.is_some(),
                ..pci
            },
            vector,
        ))
    })();
    let (pci, vector) = match configured {
        Ok(configured) => configured,
        Err(error) => return Err(keep_error(error, (function, domain, registers))),
    };
    let owner: Rc<dyn wifi::intel::boot::FunctionOwner> = Rc::new(IntelOwner {
        function: RefCell::new(function),
        _registers: registers,
        vector,
        minimum: minimum.clone(),
    });
    let kept = |owner: &Rc<dyn wifi::intel::boot::FunctionOwner>| {
        Some(Box::new(owner.clone()) as Box<dyn Any>)
    };
    // SAFETY: the mount claimed the stopped function and mapped its BAR0
    // uncached; `owner` keeps both, and its interrupts are off.
    let io = unsafe {
        wifi::intel::boot::NativeBootIo::from_window(window.cast(), owner.clone(), minimum.clone())
    }
    .map_err(|error| (Error::IntelWifi(AttachError::Boot(error)), kept(&owner)))?;
    // SAFETY: the mount's domain isolates this requester, and the caller's
    // contract covers `inputs`.
    match unsafe { attach(io, pci, domain, inputs) } {
        Ok(link) => Ok(Located {
            location: alloc::format!("{address}"),
            link,
        }),
        Err(error) => Err((Error::IntelWifi(error), kept(&owner))),
    }
}

/// The MSI-X vector an Intel radio's completions arrive on.
#[cfg(feature = "native-acpi")]
struct IntelVector {
    msix: crate::bus::pci::MsiX,
    table: crate::bus::mmio::Mmio,
    message: crate::bus::irq::MsiMessage,
}

#[cfg(feature = "native-acpi")]
impl IntelVector {
    fn enable(&self, function: &mut Function<OwnedConfig>) -> Result<(), crate::block::BlockError> {
        function.enable_msix(self.msix, || {
            self.table.write32(0, self.message.address as u32);
            self.table.write32(4, (self.message.address >> 32) as u32);
            self.table.write32(8, self.message.data);
            self.table.write32(12, 0);
            Ok(())
        })
    }
}

/// An Intel radio's function, with everything that outlives its users.
#[cfg(feature = "native-acpi")]
struct IntelOwner {
    function: RefCell<Function<OwnedConfig>>,
    _registers: Mapping,
    vector: Option<IntelVector>,
    minimum: Rc<dyn crate::bus::pci::power::Timer>,
}

#[cfg(feature = "native-acpi")]
impl wifi::intel::boot::FunctionOwner for IntelOwner {
    /// `pci_reset_function`, of its methods the one Intel radios have, with
    /// the vector back as `pci_restore_state` puts it.
    fn reset(&self) -> Result<bool, crate::block::BlockError> {
        let mut function = self.function.borrow_mut();
        let reset = function.function_level_reset(self.minimum.as_ref())?;
        if reset && let Some(vector) = &self.vector {
            vector.enable(&mut function)?;
        }
        Ok(reset)
    }
}

/// The ACPI companion of the function at `address`, as Linux's
/// `acpi_pci_find_companion` finds it: from the root bridge whose `_SEG` and
/// `_BBN` name its root bus, the child whose `_ADR` names each bridge on the
/// way down, then the function. `bridges` lists each PCI bridge with its
/// secondary bus.
#[cfg(feature = "native-acpi")]
pub(super) fn companion(
    session: &'static crate::platform::acpi::Session,
    address: Address,
    bridges: &[Bridge],
) -> Option<crate::platform::acpi::Handle<'static>> {
    use crate::platform::acpi::Handle;
    let parent = |handle: Handle<'static>| -> Option<String> {
        let path = handle.path().ok()?;
        path.rsplit_once('.')
            .map(|(parent, _)| String::from(parent))
    };
    let mut chain = alloc::vec![(address, false)];
    let mut bus = address.bus;
    while let Some(bridge) = upstream(bridges, address.segment, bus) {
        chain.push((bridge.address, true));
        bus = bridge.address.bus;
    }
    let devices = session.all_devices().ok()?;
    let (mut current, ..) = roots(session)
        .into_iter()
        .find(|&(_, segment, root)| segment == address.segment && root == bus)?;
    for (address, bridge) in chain.into_iter().rev() {
        let path = current.path().ok()?;
        let adr = (u64::from(address.device) << 16) | u64::from(address.function);
        // acpi_find_child_device and find_child_checks: an enabled device
        // without _HID wins; one without _STA or with a _HID only scores
        // lower; a bridge's companion needs children.
        let mut best: Option<(u8, Handle<'static>)> = None;
        for &device in &devices {
            if parent(device).as_deref() != Some(path.as_str())
                || integer(device, "_ADR") != Some(adr)
            {
                continue;
            }
            if bridge {
                let own = device.path().ok()?;
                if !devices
                    .iter()
                    .any(|child| parent(*child).as_deref() == Some(own.as_str()))
                {
                    continue;
                }
            }
            let score = if device.child("_STA").is_err() {
                1
            } else if integer(device, "_STA").is_none_or(|status| status & 2 == 0) {
                continue;
            } else if device.child("_HID").is_ok() {
                1
            } else {
                3
            };
            if best.is_none_or(|(kept, _)| score > kept) {
                best = Some((score, device));
            }
            if score == 3 {
                break;
            }
        }
        current = best?.1;
    }
    Some(current)
}

/// An Intel VMD host's first NVMe controller, the devices behind it no
/// driver here takes, and the hierarchy owner that hands back the parent's
/// owners when the controller retires.
pub struct Vmd {
    pub nvme: Option<nvme::Attached>,
    pub others: Vec<vmd::Child>,
    pub owner: Box<dyn Any>,
}

/// Attach the NVMe controller behind an Intel VMD host: BAR0 holds the
/// children's configuration space, BAR2 and BAR4 their memory windows.
///
/// # Safety
/// As for [`tg3`]; every device behind the host is stopped as well.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn vmd(
    services: &dyn NativeServices,
    address: Address,
    clock: Clock,
    platform: &crate::firmware::smbios::SystemIdentity<'_>,
) -> Result<Vmd, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    let refused = |error: ServiceError| (Error::Native(error), None);
    let failed = |error: vmd::Error, owner: Box<dyn Any>| (Error::Vmd(error), Some(owner));
    let mut preparing = services.prepare_requester(address).map_err(refused)?;
    let function = preparing.function_mut();
    let profile = match vmd::Profile::identify(function.vendor, function.device) {
        Ok(profile) => profile,
        Err(error) => return Err(keep_error(Error::Vmd(error), preparing)),
    };
    let bars = match preparing.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, preparing)),
    };
    // SAFETY: VMD has no engine of its own: every request it issues is a
    // child's, forwarded upstream under its requester ID. Its Bus Master bit
    // has been clear since the mount parked it, the preparation's seal keeps
    // it so, and finalization releases the seal without setting the bit.
    // Only the hierarchy owner sets it again, after discovery stopped every
    // child firmware left mastering (refusing one that does not stop),
    // preparation cleared every child's Bus Master and the NVMe driver
    // disabled its controller. The cold grant excludes prior
    // firmware, IRQ and work users; no Stela producer exists yet. The BARs
    // were sized with decode as the mount left it, and finalization rechecks
    // the stopped state after the original root's drain.
    let bound = match unsafe { preparing.acknowledge_quiescence() }.finalize() {
        Ok(bound) => bound,
        Err(failure) => {
            return Err((Error::Preparation(failure.error), Some(Box::new(failure))));
        }
    };
    let domain = bound.attachment.domain();
    let (requester, attachment) = (bound.requester, bound.attachment);
    let reserve = |index: usize| match bars[index] {
        Some(bar @ Bar::Memory { .. }) => services.bar_resource(address, bar),
        _ => Err(ServiceError::Refused("missing memory BAR")),
    };
    // A failure here keeps the stopped requester and its attachment, as
    // every later failure keeps them with the host.
    let reserved = (|| {
        let bars = [reserve(0)?, reserve(2)?, reserve(4)?];
        let authority = services.authority()?;
        let map = |resource: &crate::bus::mmio::owned::Resource, window: core::ops::Range<u64>| {
            authority
                .map(
                    resource.clone(),
                    window.start as usize,
                    (window.end - window.start) as usize,
                    CacheAttribute::Uncached,
                    Protection::ReadWrite,
                )
                .map_err(ServiceError::Mapping)
        };
        let configuration = map(&bars[0], 0..bars[0].physical_span().bytes())?;
        let shadow = profile
            .shadow_window()
            .map(|window| map(&bars[2], window))
            .transpose()?;
        Ok::<_, ServiceError>((bars, authority, configuration, shadow))
    })();
    let (bars, authority, configuration, shadow) = match reserved {
        Ok(reserved) => reserved,
        Err(error) => return Err(keep_error(Error::Native(error), (requester, attachment))),
    };
    let resources = vmd::NativeResources {
        requester,
        bars,
        cfgbar: configuration,
        shadow,
        msix: None,
    };
    // SAFETY: the mount reserved and mapped these BARs for this stopped host,
    // and no other owner reaches the hierarchy behind it.
    let registers = match unsafe { vmd::NativeRegisters::claim(resources) } {
        Ok(registers) => registers,
        Err((error, resources)) => return Err(failed(error, Box::new((resources, attachment)))),
    };
    let mut host = match vmd::Host::initialize(registers) {
        Ok(host) => host,
        Err((error, registers)) => {
            return Err(keep_error(Error::Vmd(error), (registers, attachment)));
        }
    };
    let mut others = match host.scan() {
        Ok(children) => children,
        Err(error) => return Err(failed(error, Box::new((host, attachment)))),
    };
    let Some(index) = others.iter().position(|child| child.class == [1, 8, 2]) else {
        return Ok(Vmd {
            nvme: None,
            others,
            owner: Box::new((host, attachment)),
        });
    };
    let child = others.remove(index);
    let quirks = nvme::Quirks::for_device(child.vendor, child.device)
        | nvme::Quirks::for_platform(child.vendor, child.device, platform);
    let bytes = match host.memory_bar_bytes(child.address) {
        Ok(bytes) => bytes,
        Err(error) => return Err(failed(error, Box::new((host, attachment)))),
    };
    // SAFETY: `bytes` is the child's firmware-assigned BAR0, sized with its
    // decode off, and the stopped hierarchy has no other owner.
    let bar = match unsafe { vmd::endpoint::FirmwareBar::claim(&mut host, child.address, bytes) } {
        Ok(bar) => bar,
        Err(error) => return Err(failed(error, Box::new((host, attachment)))),
    };
    let label = alloc::format!(
        "{address} VMD {:02x}:{:02x}.{}",
        child.address.bus,
        child.address.device,
        child.address.function
    );
    // SAFETY: the host's own attached domain covers its requester, under
    // which every child's request reaches the root; the mount's permission
    // issued these mappings, and the hierarchy is stopped.
    match unsafe {
        crate::storage::nvme::vmd::attach(
            host,
            bar,
            domain,
            authority,
            services.pages(),
            clock,
            minimum.clone(),
            label,
            quirks,
        )
    } {
        Ok((attached, hierarchy)) => Ok(Vmd {
            nvme: Some(attached),
            others,
            owner: Box::new((hierarchy, attachment)),
        }),
        Err((error, hierarchy)) => Err((
            Error::Device(error),
            Some(Box::new((hierarchy, attachment))),
        )),
    }
}

/// An Intel PCIe Bluetooth transport over its owned BAR0, whose MSI-X table
/// entry 0 delivers its interrupts.
pub type IntelBluetooth =
    intel_pcie::native::lifetime::OwnedTransport<MappedBar, intel_pcie::native::MappedRegisters>;

/// A Broadcom BCM4377-family transport over its owned BAR0 and BAR2, whose
/// MSI delivers its interrupts.
pub type Bcm4377 = bcm4377::Transport<bcm4377::native::NativePlatform>;

/// An attached PCIe Bluetooth controller and its one firmware initialization
/// attempt. A kept success is not radio or guest readiness.
pub struct PcieHci {
    pub location: String,
    pub controller: PcieController,
}

#[allow(
    clippy::large_enum_variant,
    reason = "one controller per attached function, kept for the boot"
)]
pub enum PcieController {
    Intel {
        transport: IntelBluetooth,
        initialization: Result<intel_pcie::firmware::Report, intel_pcie::firmware::Error>,
    },
    Broadcom {
        transport: Bcm4377,
        initialization: Result<bcm4377::Report, bcm4377::Error>,
    },
}

/// The root buses of `segment`, as the ACPI host bridges' `_SEG` and `_BBN`
/// declare them.
#[cfg(feature = "native-acpi")]
pub(super) fn root_buses(
    session: &'static crate::platform::acpi::Session,
    segment: u16,
) -> Vec<crate::bus::pci::native_aspm::RootBus> {
    use crate::bus::pci::native_aspm::RootBus;
    roots(session)
        .into_iter()
        .filter(|&(_, root_segment, _)| root_segment == segment)
        .map(|(_, segment, bus)| RootBus { segment, bus })
        .collect()
}

/// Each ACPI PCI host bridge with its `_SEG` and `_BBN`, zero when absent.
#[cfg(feature = "native-acpi")]
fn roots(
    session: &'static crate::platform::acpi::Session,
) -> Vec<(crate::platform::acpi::Handle<'static>, u16, u8)> {
    let mut roots = session.devices("PNP0A08").unwrap_or_default();
    roots.extend(session.devices("PNP0A03").unwrap_or_default());
    roots
        .into_iter()
        .filter_map(|root| {
            let segment = u16::try_from(integer(root, "_SEG").unwrap_or(0)).ok()?;
            let bus = u8::try_from(integer(root, "_BBN").unwrap_or(0)).ok()?;
            Some((root, segment, bus))
        })
        .collect()
}

/// The integer `name` evaluates to on `handle`, when it has one.
#[cfg(feature = "native-acpi")]
fn integer(handle: crate::platform::acpi::Handle<'static>, name: &str) -> Option<u64> {
    use crate::platform::acpi::Value;
    handle.child(name).ok()?;
    match handle.evaluate(Some(name), &[]).ok()? {
        Some(Value::Integer(value)) => Some(value),
        _ => None,
    }
}

/// Attach a Broadcom PCIe Bluetooth function, as `bcm4377_probe` does:
/// BAR0 and BAR2 hold its registers, `topology` its link's bridges and
/// `board` the board its firmware is chosen for.
///
/// # Safety
/// As for [`tg3`]; `topology` holds the mount's bridges of the function's
/// segment and its root buses, and no other owner sets their link policy.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn bcm4377(
    services: &dyn NativeServices,
    address: Address,
    topology: crate::bus::pci::native_aspm::Topology<OwnedConfig>,
    board: bcm4377::Board,
    clock: Clock,
) -> Result<Bcm4377, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    use bcm4377::native::{
        Error as NativeError, Inputs, MappedRegisters, NativeClock, NativePlatform, cold,
    };
    let refused = |error: ServiceError| (Error::Native(error), None);
    let failed =
        |error: NativeError, kept: Box<dyn Any>| (Error::Bcm4377(error.into()), Some(kept));
    // SAFETY: the handoff's clock is the calibrated invariant TSC.
    let clock =
        unsafe { NativeClock::new(clock) }.map_err(|error| (Error::Bcm4377(error.into()), None))?;
    let interrupts = services.interrupts().map_err(refused)?;
    let authority = services.authority().map_err(refused)?;
    let mut requester = services.prepare_requester(address).map_err(refused)?;
    // Restore D0 before BAR sizing under the original configuration/root owner.
    if let Err(error) = requester.function_mut().power_up(minimum.as_ref()) {
        return Err(failed(
            NativeError::Power(error),
            Box::new((requester, topology)),
        ));
    }
    let bars = match requester.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, (requester, topology))),
    };
    let bar0 = match map(services, address, &bars, 0) {
        Ok(mapping) => mapping,
        Err(error) => return Err((Error::Native(error), Some(Box::new((requester, topology))))),
    };
    let bar2 = match map(services, address, &bars, 2) {
        Ok(mapping) => mapping,
        Err(error) => {
            return Err((
                Error::Native(error),
                Some(Box::new((requester, topology, bar0))),
            ));
        }
    };
    let spans = match (bar0.physical_span(), bar2.physical_span()) {
        (Ok(bar0), Ok(bar2)) => [bar0, bar2],
        (Err(error), _) | (_, Err(error)) => {
            return Err(failed(
                NativeError::Mapping(error),
                Box::new((requester, topology, bar0, bar2)),
            ));
        }
    };
    // SAFETY: the mount mapped exactly BAR0 and BAR2 uncached, once each.
    let registers = match unsafe { MappedRegisters::new(spans, [bar0, bar2]) } {
        Ok(registers) => registers,
        Err((error, mappings)) => {
            return Err(failed(error, Box::new((requester, topology, mappings))));
        }
    };
    let cold_inputs = cold::Inputs {
        requester,
        topology,
        registers,
        clock,
        minimum: minimum.clone(),
    };
    // SAFETY: original requester/BAR/root/backing are held, prior firmware,
    // IRQ/work, bypass and competing PCI writers excluded. No private DMA yet.
    let cold = unsafe { cold::Cold::new(cold_inputs) }
        .map_err(|(error, inputs)| failed(error, Box::new(inputs)))?;
    let bound = match cold.quiesce_and_bind() {
        Ok(bound) => bound,
        Err(failure) => return Err(failed(failure.error.clone(), Box::new(failure))),
    };
    let inputs = Inputs {
        function: bound.function,
        retained: Arc::new(authority),
        topology: bound.topology,
        registers: bound.registers,
        attachment: bound.attachment,
        pages: Box::new(services.pages()),
        board,
        // An x86 board's firmware node carries none of the device-tree
        // calibration or address properties Apple silicon boards do.
        properties: Vec::new(),
        clock: bound.clock,
        minimum: bound.minimum,
        interrupts,
    };
    // SAFETY: vendor FLR/D0/BAR restore, original-root drain and current-epoch
    // binding completed under the same owners. Constructor performs no reset;
    // only its subsequent transport preparation enables the new domain's BME.
    let platform = unsafe { NativePlatform::new(inputs) }
        .map_err(|(error, inputs)| failed(error, Box::new(inputs)))?;
    platform
        .into_transport()
        .map_err(|error| (Error::Bcm4377(error), None))
}

/// Attach an Intel PCIe Bluetooth function and start its ROM transport, as
/// `btintel_pcie_probe` does before downloading firmware. Its reset is the
/// function's own, so it needs no coordination with the Wi-Fi function of the
/// same module, as in Linux.
///
/// # Safety
/// As for [`tg3`].
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
pub(super) unsafe fn intel_bluetooth(
    services: &dyn NativeServices,
    address: Address,
    clock: Clock,
) -> Result<IntelBluetooth, Failure> {
    let minimum = services
        .minimum_delay()
        .map_err(|error| (Error::Native(error), None))?;
    use intel_pcie::native::{self as native, Inputs, NativePlatform, cold, lifetime};
    let refused = |error: ServiceError| (Error::Native(error), None);
    let failed =
        |error: intel_pcie::Error, kept: Box<dyn Any>| (Error::IntelBluetooth(error), Some(kept));
    let mut requester = services.prepare_requester(address).map_err(refused)?;
    // Power restoration precedes sizing/MMIO, with the original root/backing
    // still held and requester DMA gated by preparation.
    if let Err(error) = requester.function_mut().power_up(minimum.as_ref()) {
        return Err(failed(
            native::Error::Power(error).into(),
            Box::new(requester),
        ));
    }
    let bars = match requester.function_mut().bars() {
        Ok(bars) => bars,
        Err(error) => return Err(keep_error(error, requester)),
    };
    let interrupts = services.interrupts().map_err(refused)?;
    let authority = services.authority().map_err(refused)?;
    let Some(bar @ Bar::Memory { .. }) = bars[0] else {
        return Err(refused(ServiceError::Refused("missing memory BAR")));
    };
    let mapping = services.map_bar(address, bar).map_err(refused)?;
    let full = match mapping.physical_span() {
        Ok(span) => span,
        Err(error) => {
            return Err(failed(
                native::Error::Mapping(error).into(),
                Box::new((requester, mapping)),
            ));
        }
    };
    // SAFETY: same original complete UC BAR0. This table view is not used
    // until after cold reset, original-root drain and private attachment.
    let table = match unsafe { MappedBar::new(full, [Some(&mapping), None]) } {
        Ok(table) => table,
        Err(error) => {
            return Err(failed(
                native::Error::Msix(error).into(),
                Box::new((requester, mapping)),
            ));
        }
    };
    // SAFETY: original exclusively owned BAR0, with the inactive table view
    // kept locally. The cold owner preserves this mapping through every failure.
    let registers = match unsafe { native::MappedRegisters::new(full, mapping) } {
        Ok(registers) => registers,
        Err((error, mapping)) => {
            return Err(failed(error.into(), Box::new((requester, mapping, table))));
        }
    };
    let inputs = cold::Inputs {
        requester,
        registers,
        clock,
        minimum: minimum.clone(),
    };
    // SAFETY: the original function is powered D0; prior IRQ/work/firmware,
    // competing writers and bypass are excluded by the mount. Clock/backing
    // remain the original provider's. No private allocation or IRQ exists yet.
    let cold = match unsafe { cold::Cold::new(inputs) } {
        Ok(cold) => cold,
        Err((error, inputs)) => return Err(failed(error, Box::new((inputs, table)))),
    };
    let mut bound = match cold.quiesce_and_bind() {
        Ok(bound) => bound,
        Err(failure) => {
            let error = failure.error.clone();
            return Err(failed(error, Box::new((failure, table))));
        }
    };
    let mut function = bound.requester.into_function();
    let retention = match function.config().retain_epoch() {
        Ok(retention) => retention,
        Err(error) => {
            return Err(failed(
                native::Error::Mapping(error).into(),
                Box::new((function, bound.registers, bound.attachment)),
            ));
        }
    };
    match bound.attachment.matches_config(function.config()) {
        Ok(true) => {}
        Ok(false) => {
            return Err(failed(
                native::Error::Requirement("same original BT config allocation").into(),
                Box::new((function, bound.registers, bound.attachment, retention)),
            ));
        }
        Err(error) => {
            return Err(failed(
                native::Error::Dma {
                    operation: lifetime::DmaOperation::Drain,
                    source: error,
                }
                .into(),
                Box::new((function, bound.registers, bound.attachment, retention)),
            ));
        }
    }
    // Vendor disconnect/reset, original-root drain and current-epoch binding
    // are complete. BME now permits only the subsequently allocated new domain.
    if let Err(error) = function.enable_polled() {
        return Err(keep_error(
            error,
            (
                function,
                bound.registers,
                bound.attachment,
                retention,
                table,
            ),
        ));
    }
    let mut msix = [const { None }; 6];
    msix[0] = Some(table);
    let msi_route = match bound.attachment.irq_route() {
        Ok(route) => route,
        Err(error) => {
            return Err(failed(
                native::Error::Dma {
                    operation: lifetime::DmaOperation::Drain,
                    source: error,
                }
                .into(),
                Box::new((function, msix, bound.registers, bound.attachment, retention)),
            ));
        }
    };
    let inputs = Inputs {
        function,
        retained: Arc::new(authority),
        bars: msix,
        registers: bound.registers,
        clock,
        minimum: minimum.clone(),
        interrupts,
        domain: bound.attachment.domain(),
        msi_route,
    };
    // SAFETY: same original stopped function/root, acknowledged memory/BME,
    // coherent domain, routed IRQ provider and checked UC/MSI-X owners.
    let platform = match unsafe { NativePlatform::new(inputs) } {
        Ok(platform) => platform,
        Err((error, inputs)) => {
            return Err(failed(
                error.into(),
                Box::new((inputs, bound.attachment, retention)),
            ));
        }
    };
    // SAFETY: receipt finalized this exact requester's snooped coherent domain,
    // with all old backing held through its original completion; DMA is 64-bit.
    let allocator =
        unsafe { DomainAllocator::new(services.pages(), bound.attachment.domain(), u64::MAX) };
    let inputs = lifetime::Inputs {
        platform,
        allocator,
        attachment: bound.attachment,
        retention,
    };
    // SAFETY: these are the same original function, configuration seal, unique
    // attachment and allocator. The native owner retains them through teardown.
    let mut transport = unsafe { lifetime::OwnedTransport::new(inputs) }
        .map_err(|failure| failed(failure.error.clone(), Box::new(failure)))?;
    match transport.start() {
        Ok(()) => Ok(transport),
        Err(error) => Err(failed(error, Box::new(transport))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::probe::services::tests::Refusing;

    fn address() -> Address {
        Address {
            segment: 0,
            bus: 3,
            device: 0,
            function: 0,
        }
    }

    #[test]
    fn missing_minimum_refuses_before_any_native_preparation() {
        let services = Refusing::default();
        let clock = Clock {
            ticks_per_microsecond: 1000,
        };
        // SAFETY: all physical services refuse; no device capability exists.
        let error = unsafe { ehci(&services, address(), clock) };
        assert!(matches!(
            error,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert!(services.requests.borrow().is_empty());
        assert!(services.preparations.borrow().is_empty());
    }
    #[test]
    fn ehci_requests_original_preparation_without_a_stopped_or_bound_grant() {
        let services = Refusing::minimum_only();
        let clock = Clock {
            ticks_per_microsecond: 1000,
        };
        // SAFETY: refusing services transfer no owner or hardware access.
        let result = unsafe { ehci(&services, address(), clock) };
        assert!(matches!(
            result,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert_eq!(*services.requests.borrow(), [address()]);
        assert_eq!(*services.preparations.borrow(), [address()]);
    }

    #[test]
    fn a_refused_claim_attaches_nothing_and_keeps_no_owner() {
        let services = Refusing::minimum_only();
        let clock = Clock {
            ticks_per_microsecond: 1000,
        };
        // SAFETY: the refusing services transfer no owner.
        let tg3 = unsafe { tg3(&services, address()) };
        assert!(matches!(
            tg3,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert_eq!(*services.preparations.borrow(), [address()]);
        // SAFETY: as above.
        let aqtion = unsafe { aqtion(&services, address()) };
        assert!(matches!(
            aqtion,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert_eq!(*services.preparations.borrow(), [address(); 2]);
        // SAFETY: as above.
        let vmd = unsafe { vmd(&services, address(), clock, &Default::default()) };
        assert!(matches!(
            vmd,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        // VMD attaches through the cold preparation the policy plans for it.
        assert_eq!(*services.preparations.borrow(), [address(); 3]);
        // SAFETY: as above.
        let bluetooth = unsafe { intel_bluetooth(&services, address(), clock) };
        assert!(matches!(
            bluetooth,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert_eq!(*services.requests.borrow(), [address(); 4]);
    }

    #[test]
    fn a_broadcom_function_without_interrupts_claims_nothing() {
        let services = Refusing::minimum_only();
        let clock = Clock {
            ticks_per_microsecond: 1000,
        };
        // SAFETY: an empty topology owns no bridge, and the refusing services
        // transfer no owner.
        let topology =
            unsafe { crate::bus::pci::native_aspm::Topology::new(Vec::new(), Vec::new()) };
        let board = bcm4377::Board::from_dmi("Apple Inc.", "MacBookAir9,1").unwrap();
        // SAFETY: as above.
        let bluetooth = unsafe { bcm4377(&services, address(), topology, board, clock) };
        assert!(matches!(
            bluetooth,
            Err((Error::Native(ServiceError::Unavailable), None))
        ));
        assert!(services.requests.borrow().is_empty());
    }

    #[test]
    fn broadcom_bluetooth_is_named_by_its_device_table() {
        assert_eq!(
            bcm4377::Chip::of_device(0x5fa0),
            Some(bcm4377::Chip::Bcm4377)
        );
        assert_eq!(
            bcm4377::Chip::of_device(0x5f72),
            Some(bcm4377::Chip::Bcm4388)
        );
        assert_eq!(bcm4377::Chip::of_device(0x4377), None);
    }
}
