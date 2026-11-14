//! The PCI scan that brings up every driver: each function of every mapped
//! ECAM window goes to the driver of its class, and what each driver found
//! or refused is collected in [`Devices`].
use crate::{
    Clock, Handoff, PlatformCaptures,
    block::{BlockDevice, BlockError},
    bus::{
        dma::DmaAllocator,
        pci::{Address, EcamWindow, Function},
    },
    gpu,
    input::{self, InputDevice, virtio::VirtioInput},
    net::{
        self, Link, LinkError, Mac,
        alx::Alx,
        atl1c::Atl1c,
        e1000e::{E1000e, pch::cold::attach as pch},
        igb::Igb,
        igc::Igc,
        realtek::{self, Realtek},
        virtio::VirtioNet,
    },
    storage::{ahci, nvme, rtsx, sdhci, virtio},
    usb::{self, bluetooth::UsbHci, cdc::UsbNet, ethernet::UsbEthernet, xhci},
};
use alloc::{boxed::Box, rc::Rc, string::String, sync::Arc, vec::Vec};
use core::fmt;
use stela_driver_base::budget;

#[cfg(feature = "native-acpi")]
mod companion;
#[cfg(feature = "native-acpi")]
mod i2c;
#[cfg(feature = "native-acpi")]
mod power;
#[cfg(feature = "native-acpi")]
pub use power::{EmbeddedControllerInstallation, Power, install_embedded_controller};
pub mod native;
#[cfg(any(test, feature = "native-acpi"))]
mod platform;
pub mod services;
#[cfg(feature = "native-acpi")]
pub mod smbus;
#[cfg(feature = "native-acpi")]
pub mod typec;
use crate::net::wifi::network::Network;
pub use services::{NativeServices, ServiceError};

/// A controller refusal keeps its original typed details for boot diagnostics.
/// In particular, firmware names and integrity/target errors are never replaced
/// by a generic unsupported-device message.
#[derive(Debug)]
pub enum Error {
    Enumeration(crate::bus::pci::EnumerationError),
    Device(BlockError),
    Realtek(realtek::Error),
    Pch(pch::AttachError),
    /// The mount's [`NativeServices`] refused or lack a resource.
    Native(ServiceError),
    /// The original root refused a driver's quiescence receipt.
    Preparation(crate::bus::pci::requester::preparation::Error),
    Vmd(crate::bus::vmd::Error),
    Tg3(net::tg3::Error),
    Aqtion(net::aqtion::native::Error),
    Sfh(input::sfh::Error),
    SfhCold(input::sfh::cold::native::NativeError),
    Ish(input::ish::Error),
    IshCold(input::ish::cold::native::Error),
    IntelWifi(net::wifi::intel::attach::AttachError),
    MediatekWifi(net::wifi::mediatek::attach::AttachError),
    RealtekWifi(net::wifi::realtek::attach::AttachError),
    IntelBluetooth(crate::bluetooth::intel_pcie::Error),
    Bcm4377(crate::bluetooth::bcm4377::Error),
    Ehci(usb::ehci::Error),
    HdaPci(crate::audio::hda::pci::AttachError),
    Hda(crate::audio::Error),
    HdaNative(native::hda::RetirementError),
    Usb4(native::usb4::Refusal),
    /// A driver here claimed the function and its start recorded no
    /// device, no failure and no reason: the probe names it, so no claimed
    /// device ends up silently absent.
    Unrecorded,
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Device(error) => error.fmt(f),
            Self::Enumeration(error) => write!(f, "PCI enumeration: {error}"),
            Self::Realtek(error) => error.fmt(f),
            Self::Pch(error) => write!(f, "e1000e: {error}"),
            Self::Native(error) => error.fmt(f),
            Self::Preparation(error) => write!(f, "cold requester preparation: {error}"),
            Self::Vmd(error) => write!(f, "VMD: {error}"),
            Self::Tg3(error) => write!(f, "tg3: {error}"),
            Self::Aqtion(error) => write!(f, "atlantic: {error}"),
            Self::Sfh(error) => write!(f, "AMD SFH: {error:?}"),
            Self::SfhCold(error) => write!(f, "AMD SFH original-owner preparation: {error:?}"),
            Self::Ish(error) => write!(f, "Intel ISH: {error:?}"),
            Self::IshCold(error) => write!(f, "Intel ISH original-owner preparation: {error:?}"),
            Self::IntelWifi(error) => write!(f, "iwlwifi: {error}"),
            Self::MediatekWifi(error) => write!(f, "mt76: {error}"),
            Self::RealtekWifi(error) => write!(f, "rtw88: {error}"),
            Self::IntelBluetooth(error) => write!(f, "btintel_pcie: {error}"),
            Self::Bcm4377(error) => write!(f, "hci_bcm4377: {error}"),
            Self::Ehci(error) => write!(f, "EHCI: {error}"),
            Self::HdaPci(error) => write!(f, "HD Audio PCI: {error:?}"),
            Self::Hda(error) => write!(f, "HD Audio: {error:?}"),
            Self::HdaNative(error) => write!(f, "HD Audio native: {error:?}"),
            Self::Usb4(refusal) => write!(f, "USB4 host: {refusal}"),
            Self::Unrecorded => {
                f.write_str("its driver started and recorded no device and no failure")
            }
        }
    }
}

impl From<BlockError> for Error {
    fn from(error: BlockError) -> Self {
        Self::Device(error)
    }
}

impl From<crate::bus::pci::ConfigError> for Error {
    fn from(error: crate::bus::pci::ConfigError) -> Self {
        Self::Device(BlockError::Config(error))
    }
}
impl From<crate::bus::pci::CapabilityError> for Error {
    fn from(error: crate::bus::pci::CapabilityError) -> Self {
        Self::Device(BlockError::PciCapability(error))
    }
}

impl From<pch::AttachError> for Error {
    fn from(error: pch::AttachError) -> Self {
        Self::Pch(error)
    }
}

impl From<realtek::Error> for Error {
    fn from(error: realtek::Error) -> Self {
        Self::Realtek(error)
    }
}

/// Why a device or controller outside the PCI drivers' attach was not brought
/// up or handed over.
#[derive(Clone, Debug, PartialEq, Eq)]
pub enum PlatformError {
    Config(crate::bus::pci::ConfigError),
    PciCapability(crate::bus::pci::CapabilityError),
    /// A USB host controller could not be taken from firmware.
    Handoff(usb::handoff::Error),
    /// Walking the ACPI namespace or decoding a resource template failed.
    #[cfg(feature = "native-acpi")]
    Acpi(crate::platform::acpi::resources::Error),
    /// The description or the machine state rules attachment out.
    Refused(&'static str),
    Ps2(input::ps2::Error),
    Native(ServiceError),
    I2c(crate::bus::i2c::Error),
    Lpss(crate::bus::lpss::Error),
    LpssAttach(crate::bus::lpss::AttachError),
    HidI2c(input::hid_i2c::Error),
    /// The HID report descriptor could not be interpreted as input.
    HidDescriptor(usb::hid::ParseError),
    Touchpad(input::touchpad::Error),
    #[cfg(feature = "native-acpi")]
    Gpio(crate::bus::gpio::native::Error),
    /// The embedded controller's operation regions could not be installed.
    #[cfg(feature = "native-acpi")]
    Ec(crate::platform::ec::binding::Failure),
    /// The embedded controller's description could not be read.
    #[cfg(feature = "native-acpi")]
    EcDiscovery(crate::platform::ec::discovery::native::DiscoveryError),
    #[cfg(feature = "native-acpi")]
    Battery(crate::platform::battery::Error),
    Smbus(crate::bus::smbus::Error),
    #[cfg(feature = "native-acpi")]
    Ucsi(crate::usb::typec::ucsi::acpi::attachment::Error),
    /// Finding a device or evaluating its method failed.
    #[cfg(feature = "native-acpi")]
    AcpiMethod(crate::platform::acpi::Error),
}

impl fmt::Display for PlatformError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Handoff(error) => error.fmt(f),
            Self::Config(error) => error.fmt(f),
            Self::PciCapability(error) => write!(f, "PCI capability: {error:?}"),
            #[cfg(feature = "native-acpi")]
            Self::Acpi(error) => write!(f, "ACPI: {error:?}"),
            Self::Refused(why) => f.write_str(why),
            Self::Ps2(error) => error.fmt(f),
            Self::Native(error) => error.fmt(f),
            Self::I2c(error) => write!(f, "I2C controller: {error:?}"),
            Self::Lpss(error) => write!(f, "LPSS: {error:?}"),
            Self::LpssAttach(error) => write!(f, "LPSS I2C attach: {error:?}"),
            Self::HidI2c(error) => error.fmt(f),
            Self::HidDescriptor(error) => error.fmt(f),
            Self::Touchpad(error) => error.fmt(f),
            #[cfg(feature = "native-acpi")]
            Self::Gpio(error) => write!(f, "GPIO controller: {error:?}"),
            #[cfg(feature = "native-acpi")]
            Self::Ec(error) => write!(f, "embedded controller: {error:?}"),
            #[cfg(feature = "native-acpi")]
            Self::EcDiscovery(error) => write!(f, "embedded controller description: {error:?}"),
            #[cfg(feature = "native-acpi")]
            Self::Battery(error) => write!(f, "battery: {error:?}"),
            Self::Smbus(error) => error.fmt(f),
            #[cfg(feature = "native-acpi")]
            Self::Ucsi(error) => write!(f, "UCSI: {error:?}"),
            #[cfg(feature = "native-acpi")]
            Self::AcpiMethod(error) => write!(f, "ACPI method: {error:?}"),
        }
    }
}

/// Nonfatal diagnostics keep controller-specific values through discovery.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Warning {
    Message(&'static str),
    /// An xHCI controller stopped answering: what it was doing, and whether
    /// it was reset and attached again or this was the second attachment.
    XhciDied(Option<xhci::Death>, bool),
    Realtek(realtek::Warning),
    /// An Intel xHCI took its switchable ports from EHCI.
    IntelPortSwitch(usb::handoff::Switched),
    /// An NVMe controller runs with Linux's quirks for its PCI ID, model or
    /// board.
    NvmeQuirks(nvme::Quirks),
    /// An NVMe controller's namespaces were scanned one ID at a time in
    /// place of its refused or garbage active namespace list.
    NvmeScan(nvme::SequentialScan),
}

impl fmt::Display for Warning {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Message(message) => f.write_str(message),
            Self::XhciDied(death, again) => {
                f.write_str(if *again {
                    XHCI_DIED_AGAIN
                } else {
                    XHCI_RESTARTED
                })?;
                match death {
                    Some(death) => write!(f, " ({death})"),
                    None => Ok(()),
                }
            }
            Self::Realtek(warning) => write!(f, "Realtek {warning}"),
            Self::IntelPortSwitch(switched) => write!(
                f,
                "Intel xHCI took its ports from EHCI: SuperSpeed {:#x}, USB 2 {:#x}",
                switched.superspeed, switched.usb2
            ),
            Self::NvmeQuirks(quirks) => f.write_str(&quirks.describe()),
            Self::NvmeScan(scan) => write!(f, "{scan}"),
        }
    }
}

/// A host network adapter, as a raw Ethernet link.
pub enum Adapter {
    Virtio(VirtioNet),
    Igc(Igc),
    E1000e(E1000e),
    /// An Intel 82579, I217/I218 or I219 integrated MAC.
    Pch(Box<pch::Pch>),
    Igb(Igb),
    Alx(Alx),
    Atl1c(Atl1c),
    Realtek(Realtek),
    Usb(UsbNet),
    UsbVendor(UsbEthernet),
    Tg3(Box<native::Located<native::Tg3>>),
    Aqtion(Box<native::Located<net::aqtion::native::Attached>>),
    #[cfg(feature = "native-acpi")]
    IntelWifi(Box<native::Located<native::IntelWifi>>),
    MediatekWifi(Box<native::Located<native::MediatekWifi>>),
    RealtekWifi(Box<native::Located<native::RealtekWifi>>),
}

impl Adapter {
    /// Stable implementation name for diagnostics and driver verification.
    pub fn driver(&self) -> &'static str {
        match self {
            Self::Virtio(_) => "virtio-net",
            Self::Igc(_) => "igc",
            Self::E1000e(_) | Self::Pch(_) => "e1000e",
            Self::Igb(_) => "igb",
            Self::Alx(_) => "alx",
            Self::Atl1c(_) => "atl1c",
            Self::Realtek(_) => "realtek",
            Self::Usb(_) => "usb-cdc",
            Self::Tg3(_) => "tg3",
            Self::Aqtion(_) => "atlantic",
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(_) => "iwlwifi",
            Self::MediatekWifi(_) => "mt7925e",
            Self::RealtekWifi(_) => "rtw88_8822ce",
            Self::UsbVendor(device) => device.driver(),
        }
    }

    /// Whether the adapter is a Wi-Fi radio rather than a wired port.
    pub fn wireless(&self) -> bool {
        match self {
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(_) => true,
            Self::MediatekWifi(_) | Self::RealtekWifi(_) => true,
            _ => false,
        }
    }

    /// Where the adapter is, for logs.
    pub fn location(&self) -> &str {
        match self {
            Self::Virtio(device) => &device.location,
            Self::Igc(device) => &device.location,
            Self::E1000e(device) => &device.location,
            Self::Pch(device) => &device.location,
            Self::Igb(device) => &device.location,
            Self::Alx(device) => &device.location,
            Self::Atl1c(device) => &device.location,
            Self::Realtek(device) => &device.location,
            Self::Usb(device) => &device.location,
            Self::UsbVendor(device) => &device.location,
            Self::Tg3(device) => &device.location,
            Self::Aqtion(device) => &device.location,
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => &device.location,
            Self::MediatekWifi(device) => &device.location,
            Self::RealtekWifi(device) => &device.location,
        }
    }
}

impl Link for Adapter {
    fn mac(&self) -> Mac {
        match self {
            Self::Virtio(device) => device.mac(),
            Self::Igc(device) => device.mac(),
            Self::E1000e(device) => device.mac(),
            Self::Pch(device) => device.mac(),
            Self::Igb(device) => device.mac(),
            Self::Alx(device) => device.mac(),
            Self::Atl1c(device) => device.mac(),
            Self::Realtek(device) => device.mac(),
            Self::Usb(device) => device.mac(),
            Self::UsbVendor(device) => device.mac(),
            Self::Tg3(device) => device.mac(),
            Self::Aqtion(device) => device.mac(),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.mac(),
            Self::MediatekWifi(device) => device.mac(),
            Self::RealtekWifi(device) => device.mac(),
        }
    }
    fn carrier(&mut self) -> bool {
        match self {
            Self::Virtio(device) => device.carrier(),
            Self::Igc(device) => device.carrier(),
            Self::E1000e(device) => device.carrier(),
            Self::Pch(device) => device.carrier(),
            Self::Igb(device) => device.carrier(),
            Self::Alx(device) => device.carrier(),
            Self::Atl1c(device) => device.carrier(),
            Self::Realtek(device) => device.carrier(),
            Self::Usb(device) => device.carrier(),
            Self::UsbVendor(device) => device.carrier(),
            Self::Tg3(device) => device.carrier(),
            Self::Aqtion(device) => device.carrier(),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.carrier(),
            Self::MediatekWifi(device) => device.carrier(),
            Self::RealtekWifi(device) => device.carrier(),
        }
    }
    fn carrier_wait_us(&self) -> u64 {
        match self {
            Self::Virtio(device) => device.carrier_wait_us(),
            Self::Igc(device) => device.carrier_wait_us(),
            Self::E1000e(device) => device.carrier_wait_us(),
            Self::Pch(device) => device.carrier_wait_us(),
            Self::Igb(device) => device.carrier_wait_us(),
            Self::Alx(device) => device.carrier_wait_us(),
            Self::Atl1c(device) => device.carrier_wait_us(),
            Self::Realtek(device) => device.carrier_wait_us(),
            Self::Usb(device) => device.carrier_wait_us(),
            Self::UsbVendor(device) => device.carrier_wait_us(),
            Self::Tg3(device) => device.carrier_wait_us(),
            Self::Aqtion(device) => device.carrier_wait_us(),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.carrier_wait_us(),
            Self::MediatekWifi(device) => device.carrier_wait_us(),
            Self::RealtekWifi(device) => device.carrier_wait_us(),
        }
    }
    fn signals(&self) -> bool {
        match self {
            Self::Virtio(device) => device.signals(),
            Self::Igc(device) => device.signals(),
            Self::E1000e(device) => device.signals(),
            Self::Pch(device) => device.signals(),
            Self::Igb(device) => device.signals(),
            Self::Alx(device) => device.signals(),
            Self::Atl1c(device) => device.signals(),
            Self::Realtek(device) => device.signals(),
            Self::Usb(device) => device.signals(),
            Self::UsbVendor(device) => device.signals(),
            Self::Tg3(device) => device.signals(),
            Self::Aqtion(device) => device.signals(),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.signals(),
            Self::MediatekWifi(device) => device.signals(),
            Self::RealtekWifi(device) => device.signals(),
        }
    }
    fn due_us(&self) -> Option<u64> {
        match self {
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.due_us(),
            Self::MediatekWifi(device) => device.due_us(),
            Self::RealtekWifi(device) => device.due_us(),
            _ => None,
        }
    }
    fn transmit(&mut self, frame: &[u8]) -> Result<(), LinkError> {
        match self {
            Self::Virtio(device) => device.transmit(frame),
            Self::Igc(device) => device.transmit(frame),
            Self::E1000e(device) => device.transmit(frame),
            Self::Pch(device) => device.transmit(frame),
            Self::Igb(device) => device.transmit(frame),
            Self::Alx(device) => device.transmit(frame),
            Self::Atl1c(device) => device.transmit(frame),
            Self::Realtek(device) => device.transmit(frame),
            Self::Usb(device) => device.transmit(frame),
            Self::UsbVendor(device) => device.transmit(frame),
            Self::Tg3(device) => device.transmit(frame),
            Self::Aqtion(device) => device.transmit(frame),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.transmit(frame),
            Self::MediatekWifi(device) => device.transmit(frame),
            Self::RealtekWifi(device) => device.transmit(frame),
        }
    }
    fn receive(&mut self) -> Result<Option<Vec<u8>>, LinkError> {
        match self {
            Self::Virtio(device) => device.receive(),
            Self::Igc(device) => device.receive(),
            Self::E1000e(device) => device.receive(),
            Self::Pch(device) => device.receive(),
            Self::Igb(device) => device.receive(),
            Self::Alx(device) => device.receive(),
            Self::Atl1c(device) => device.receive(),
            Self::Realtek(device) => device.receive(),
            Self::Usb(device) => device.receive(),
            Self::UsbVendor(device) => device.receive(),
            Self::Tg3(device) => device.receive(),
            Self::Aqtion(device) => device.receive(),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.receive(),
            Self::MediatekWifi(device) => device.receive(),
            Self::RealtekWifi(device) => device.receive(),
        }
    }
    fn receive_reusing(&mut self, frame: &mut Vec<u8>) -> Result<bool, LinkError> {
        match self {
            Self::Virtio(device) => device.receive_reusing(frame),
            Self::Igc(device) => device.receive_reusing(frame),
            Self::E1000e(device) => device.receive_reusing(frame),
            Self::Pch(device) => device.receive_reusing(frame),
            Self::Igb(device) => device.receive_reusing(frame),
            Self::Alx(device) => device.receive_reusing(frame),
            Self::Atl1c(device) => device.receive_reusing(frame),
            Self::Realtek(device) => device.receive_reusing(frame),
            Self::Usb(device) => device.receive_reusing(frame),
            Self::UsbVendor(device) => device.receive_reusing(frame),
            Self::Tg3(device) => device.receive_reusing(frame),
            Self::Aqtion(device) => device.receive_reusing(frame),
            #[cfg(feature = "native-acpi")]
            Self::IntelWifi(device) => device.receive_reusing(frame),
            Self::MediatekWifi(device) => device.receive_reusing(frame),
            Self::RealtekWifi(device) => device.receive_reusing(frame),
        }
    }
}

/// Whether a driver here may attach `function`, or probe hands it over from
/// firmware (every USB host controller), including the HD Audio check: its
/// memory BARs must be mapped uncached before ExitBootServices.
pub fn claims(function: &Function) -> bool {
    claims_identity(&function.identity())
}

/// Candidate selection from copied firmware/native header DATA only. This
/// grants neither resource ownership nor permission to size or attach BARs.
pub fn claims_identity(function: &crate::bus::pci::Identity) -> bool {
    virtio::matches_identity(function)
        || input::virtio::matches_identity(function)
        || net::virtio::matches_identity(function)
        || net::igc::matches_identity(function)
        || net::e1000e::matches_identity(function)
        || pch::matches_identity(function)
        || net::igb::matches_identity(function)
        || net::alx::matches_identity(function)
        || net::atl1c::matches_identity(function)
        || realtek::matches_identity(function)
        || crate::audio::hda::pci::identify_identity(function).is_ok()
        || function.class == nvme::CLASS
        || ahci::matches_identity(function)
        || sdhci::matches_identity(function)
        || rtsx::matches_identity(function)
        || function.class == xhci::CLASS
        || usb::handoff::Kind::of(function.class).is_some()
}

/// Why the disks behind storage controller `function` are out of reach of
/// Stela's drivers, for the disk chooser to refuse them before boot
/// services end rather than the boot failing after; `None` when a driver
/// here takes the controller, or it is no storage controller.
pub fn disk_refusal(function: &crate::bus::pci::Identity) -> Option<&'static str> {
    if function.class[0] != 0x01
        || claims_identity(function)
        || native::Kind::of_identity(function).is_some()
    {
        return None;
    }
    Some(match function.class[1] {
        0x04 => "Its controller is in RAID mode: set SATA to AHCI in firmware setup",
        _ => "Stela has no driver for its controller",
    })
}

/// What [`probe_with_provider`] will do with `function`, for a boot log
/// line written before boot services end: the driver that attaches it, or
/// why none does. `displays` are the slots whose function 0 is a display
/// controller. Options decided after the census (a Wi-Fi network, the
/// mount's native services) are named where they apply.
pub fn plan(
    function: &crate::bus::pci::Identity,
    displays: &[Address],
) -> alloc::borrow::Cow<'static, str> {
    use alloc::borrow::Cow;
    let class = function.class;
    if class[..2] == [2, 0x80]
        && let Some(why) = crate::net::wifi::unsupported::reason(function.vendor, function.device)
    {
        return Cow::Owned(alloc::format!("unsupported: {why}"));
    }
    if class[0] == 0x03 {
        return Cow::Borrowed("GPU");
    }
    #[cfg(feature = "native-acpi")]
    if smbus::matches_identity(function) {
        return Cow::Borrowed("i801 SMBus");
    }
    let handoff = match usb::handoff::Kind::of(class) {
        Some(usb::handoff::Kind::Ehci) => return Cow::Borrowed("EHCI USB"),
        Some(_) if class != xhci::CLASS => {
            return Cow::Borrowed(
                "taken from firmware, not driven: UHCI or OHCI USB host controller",
            );
        }
        Some(_) => true,
        None => false,
    };
    if let Some(kind) = native::Kind::of_identity(function) {
        return Cow::Borrowed(kind.driver());
    }
    let driver = if virtio::matches_identity(function) {
        "virtio-blk"
    } else if class == nvme::CLASS {
        "NVMe"
    } else if handoff {
        "xHCI USB"
    } else if net::virtio::matches_identity(function) {
        "virtio-net"
    } else if net::igc::matches_identity(function) {
        "igc Ethernet"
    } else if net::e1000e::matches_identity(function) {
        "e1000e Ethernet"
    } else if pch::matches_identity(function) {
        "Intel PCH Ethernet"
    } else if net::igb::matches_identity(function) {
        "igb Ethernet"
    } else if net::alx::matches_identity(function) {
        "alx Ethernet"
    } else if net::atl1c::matches_identity(function) {
        "atl1c Ethernet"
    } else if realtek::matches_identity(function) {
        "r8169 Ethernet"
    } else if input::virtio::matches_identity(function) {
        "virtio-input"
    } else if rtsx::matches_identity(function) {
        "Realtek card reader"
    } else if sdhci::matches_identity(function) {
        "SDHCI card reader"
    } else if ahci::matches_identity(function) {
        "AHCI SATA"
    } else if class[..2] == [6, 4] {
        "PCI bridge"
    } else if let Some(why) = unclaimed(class, function.address, displays) {
        return Cow::Owned(alloc::format!("unsupported: {why}"));
    } else {
        "not driven"
    };
    Cow::Borrowed(driver)
}

/// An HD Audio function beside a display controller's function 0 is that
/// GPU's HDMI/DP audio, as Linux `get_bound_vga` finds it. No HDMI/DP codec
/// profile here serves one outside the admitted identities.
const DISPLAY_AUDIO: &str = "HDMI/DP audio controller of a GPU outside the supported HD Audio identities (its outputs carry no audio)";

/// Why no driver here takes `class` at `address`, given the slots whose
/// function 0 is a display controller.
fn unclaimed(class: [u8; 3], address: Address, displays: &[Address]) -> Option<&'static str> {
    let beside_display = displays.iter().any(|display| {
        (display.segment, display.bus, display.device)
            == (address.segment, address.bus, address.device)
    });
    if class[..2] == [0x04, 0x03] && address.function != 0 && beside_display {
        return Some(DISPLAY_AUDIO);
    }
    unsupported(class)
}

/// Why no driver here takes a function of a device class the host needs,
/// or none for classes outside this crate's scope.
fn unsupported(class: [u8; 3]) -> Option<&'static str> {
    match class {
        [0x01, 0x01, _] => Some("IDE storage controller"),
        [0x01, 0x04, _] => {
            Some("RAID controller, not supported; set the SATA mode to AHCI in firmware setup")
        }
        [0x01, 0x06, _] => Some("SATA controller without the AHCI interface"),
        [0x01, 0x08, _] => Some("non-NVMe non-volatile memory controller"),
        [0x01, ..] => Some("storage controller"),
        [0x02, 0x00, _] => Some("Ethernet controller whose ID no driver here takes"),
        [0x02, 0x80, _] => {
            Some("Wi-Fi controller other than Intel's, MediaTek's MT7925 and Realtek's RTL8822C")
        }
        [0x02, ..] => Some("network controller whose ID no driver here takes"),
        // Admitted identities attach through the mount's native services.
        [0x04, 0x03, _] => Some("HD Audio controller outside the supported identities"),
        // EHCI attaches through the mount's native services.
        [0x0c, 0x03, 0x20] => None,
        [0x0c, 0x03, _] => {
            Some("UHCI or OHCI USB host controller (taken from firmware, not driven)")
        }
        // Listed NHIs attach through the mount's native services.
        [0x0c, 0x0a, _] => Some("USB4/Thunderbolt host outside the supported NHI identities"),
        [0x0c, 0x05, _] => Some(
            "SMBus host other than an Intel i801 under native ACPI (no Host Notify for touchpad companions)",
        ),
        [0x08, 0x05, _] => Some("SD host controller without standard SDHCI interface"),
        [0x0d, 0x11, _] => Some("PCIe Bluetooth controller other than Intel's"),
        _ => None,
    }
}

/// Every block and input device the drivers brought up, and every
/// controller they could not.
#[derive(Default)]
pub struct Devices {
    /// Native UAC2 playback/capture owners, awaiting explicit runtime PCM use.
    pub audio: Vec<crate::usb::audio::UsbAudio>,
    pub disks: Vec<Box<dyn BlockDevice>>,
    /// Display controllers, inventoried without changing firmware scanout.
    /// These are not initialized accelerators; each records its missing port.
    pub gpus: Vec<gpu::Adapter>,
    /// Host keyboards and pointers: virtio-input devices, USB HID devices
    /// behind xHCI controllers, and the i8042 ports the ACPI namespace
    /// lists, each polled through [`InputDevice::poll`].
    pub inputs: Vec<InputDevice>,
    /// Complete descriptor inventories, including hubs and unbound devices.
    /// Retention does not grant hardware ownership or guest forwarding.
    pub usb_devices: Vec<xhci::Device>,
    /// Explicitly polled USB topology owners, including empty controllers.
    pub usb_hosts: Vec<xhci::Host>,
    /// Attached EHCI controllers, which their roots' storage transports share.
    pub ehci_hosts: Vec<usb::ehci::host::Shared>,
    /// HD Audio controllers and their bound codecs.
    pub hda_hosts: Vec<native::hda::Host>,
    /// USB4/Thunderbolt hosts whose firmware connection manager answered.
    pub usb4_hosts: Vec<native::usb4::Host>,
    /// Native SDHCI socket owners, including empty/removable card sockets.
    pub sd_hosts: Vec<sdhci::Host>,
    /// Native Realtek socket owners, kept for empty/removable SD/MMC cards.
    pub rtsx_hosts: Vec<rtsx::Host>,
    /// Actual AHCI owners retained for lifecycle and failure retirement.
    pub ahci_hosts: Vec<ahci::Host>,
    /// Storage class owners remain available for explicit media rescan.
    pub usb_storage: Vec<crate::usb::storage::UsbStorage>,
    /// USB HCI transports with kept device-specific initialization results.
    /// Historical evidence does not establish radio or guest readiness.
    pub bluetooth: Vec<UsbHci>,
    /// PCIe HCI transports with their kept firmware initialization result.
    pub pcie_bluetooth: Vec<native::PcieHci>,
    /// Raw firmware uploaders awaiting USB removal/fresh enumeration.
    pub bluetooth_bootloaders: Vec<crate::usb::bluetooth::ath3k::UsbBootloader>,
    /// Exact pre-EBS board-variable observation, including unavailable diagnostics.
    pub bluetooth_platform: crate::usb::bluetooth::intel::DsbrCapture,
    /// Network adapters, as raw Ethernet links.
    pub nics: Vec<Adapter>,
    /// Every network-class PCI function the scan found (class 02), started
    /// or not, for [`Devices::network_account`].
    pub network_functions: Vec<crate::bus::pci::Identity>,
    /// The regulatory database, once probe found a Wi-Fi adapter (a PCI
    /// network controller of class 02/80) while a network is configured:
    /// `Err` reports once that it is missing, and no Wi-Fi adapter then
    /// scans or transmits.
    pub regulatory: Option<
        Result<
            crate::net::wifi::regulatory::database::Database,
            crate::net::wifi::regulatory::database::LoadError,
        >,
    >,
    pub failures: Vec<(Address, Error)>,
    /// The PCI class of every function the scan found, which names the
    /// subsystem of a failure whose error kind does not, such as an audio
    /// controller the mount refused.
    pub classes: Vec<(Address, [u8; 3])>,
    /// Devices behind an attached controller that failed on their own, such
    /// as AHCI ports and NVMe namespaces, by location.
    pub device_failures: Vec<(String, BlockError)>,
    /// Nonfatal initialization diagnostics, including adapters not selected.
    pub warnings: Vec<(Address, Warning)>,
    /// USB devices that failed to enumerate or bind, by location.
    pub usb_failures: Vec<(String, BlockError)>,
    /// Vendor initialization failures, preserving exact firmware requests.
    pub usb_vendor_failures: Vec<(String, crate::usb::ethernet::Error)>,
    /// Vendor adapters that started without their optional firmware patch,
    /// and why the patch was not applied, by location.
    pub usb_vendor_warnings: Vec<(String, crate::usb::ethernet::Error)>,
    /// USB interfaces no driver here takes, and why.
    pub usb_unused: Vec<(String, &'static str)>,
    /// How long each xHCI root port took at probe, for the boot log's
    /// `XHCI PROBE <pci> <probe>` lines.
    pub usb_port_probes: Vec<(Address, xhci::PortProbe)>,
    /// Devices and controllers outside the PCI drivers' attach that were not
    /// brought up or handed over, by location, such as
    /// `0000:00:1d.0 EHCI handoff`.
    pub platform_failures: Vec<(String, PlatformError)>,
    /// Objects that must stay alive while the devices they set up exist,
    /// including attaches that failed, such as the Intel VMD hierarchy behind
    /// its NVMe controller and a failed controller's mappings and DMA domain.
    pub owners: Vec<Box<dyn core::any::Any>>,
    /// The batteries, AC adapter and lid the ACPI namespace lists.
    #[cfg(feature = "native-acpi")]
    pub power: Power,
    /// Intel i801 SMBus hosts, shared with the touchpad companions on them.
    #[cfg(feature = "native-acpi")]
    pub smbus_hosts: Vec<smbus::Host>,
    /// Devices outside PCI that the platform describes but no driver here
    /// attaches, by location, and why; reported once, not as failures.
    pub platform_unsupported: Vec<(String, &'static str)>,
    /// ECAM windows that were not mapped, and so not scanned.
    pub unmapped: Vec<EcamWindow>,
    /// Whether the mount supplied [`NativeServices`]. Drivers that attach
    /// through owned mappings, requester DMA domains or the ACPI namespace
    /// need them.
    pub native_services: bool,
    /// Storage, network, audio and USB functions no driver here takes, with
    /// their class and why.
    pub unsupported: Vec<(Address, [u8; 3], &'static str)>,
    /// PCI bridges found with bus mastering off, which the scan turned on
    /// so the functions below them reach memory and the interrupt
    /// controllers.
    pub bridges_reopened: Vec<Address>,
    /// Functions whose start ran past [`DEVICE_START_US`]: their waits then
    /// ended at once, the driver gave up, and the function was left with
    /// bus mastering and interrupts off where the probe still held it.
    pub abandoned: Vec<(Address, budget::Expiry)>,
}

/// The longest one PCI function's start may take. Linux's own per-step
/// bounds are smaller: xHCI commands 5 s (`XHCI_CMD_DEFAULT_TIMEOUT`), USB
/// control transfers 5 s (`USB_CTRL_GET_TIMEOUT`), AHCI port reset 1 s,
/// the NVMe admin queue 60 s but CC.RDY by CAP.TO, which drives already
/// started by the firmware meet in a few seconds. A device that answers
/// starts far inside this; one that stops answering is given up here even
/// when a driver's per-step bounds would add up to more.
pub const DEVICE_START_US: u64 = 30_000_000;

/// Said when an xHCI controller died as its devices started and was reset
/// and attached a second time.
const XHCI_RESTARTED: &str = "xHCI: the controller stopped answering while its devices started; \
                              Stela reset it and looked for its devices again";

/// Said when the second attachment of an xHCI controller died as well.
const XHCI_DIED_AGAIN: &str = "xHCI: the controller stopped answering again after its reset; \
                               its devices are out of reach for this boot";

/// Why a Wi-Fi adapter is left alone.
const NO_WIFI_NETWORK: &str = "Wi-Fi adapter stays off: no --wifi network";

/// The function a driver here claimed whose start is being accounted for,
/// with the count of outcomes recorded before it started.
#[derive(Default)]
struct Claimed(Option<(Address, usize)>);

impl Claimed {
    fn begin(&mut self, address: Address, recorded: usize) {
        self.0 = Some((address, recorded));
    }

    /// The start succeeded, whatever it recorded: a controller with no
    /// disk, such as an NVMe controller with no active namespace, is no
    /// failure, as Linux reports none.
    fn started(&mut self) {
        self.0 = None;
    }

    /// Once its start has ended: a start that recorded no outcome is named
    /// a failure, so no claimed function is silently absent.
    fn settle(&mut self, recorded: usize, failures: &mut Vec<(Address, Error)>) {
        if let Some((address, before)) = self.0.take()
            && recorded == before
        {
            failures.push((address, Error::Unrecorded));
        }
    }
}

/// Whether the scan starts `function` through a driver here, and so must
/// record its outcome: not a USB4 host whose software connection manager
/// came up before the scan (`already`), which that start recorded.
fn scan_starts(function: &crate::bus::pci::Identity, already: &[Address]) -> bool {
    match native::Kind::of_identity(function) {
        Some(native::Kind::Usb4) => !already.contains(&function.address),
        Some(_) => true,
        None => claims_identity(function),
    }
}

/// One function's start under its [`DEVICE_START_US`] budget; a passed
/// budget is recorded as the function's abandonment when the start ends.
struct DeviceStart<'a> {
    address: Address,
    budget: Option<budget::Budget>,
    abandoned: &'a mut Vec<(Address, budget::Expiry)>,
}

impl<'a> DeviceStart<'a> {
    fn begin(
        address: Address,
        clock: Clock,
        abandoned: &'a mut Vec<(Address, budget::Expiry)>,
    ) -> Self {
        Self {
            address,
            budget: budget::Budget::start(clock, DEVICE_START_US).ok(),
            abandoned,
        }
    }

    /// A fresh budget for a second start of the same function, as a
    /// controller reset and attached again gets: the first start's budget,
    /// which its failure may have used up, would end every wait of the
    /// second at once.
    fn renew(&mut self, clock: Clock) {
        // Dropped first, so the new budget replaces what the old replaced.
        self.budget = None;
        self.budget = budget::Budget::start(clock, DEVICE_START_US).ok();
    }

    /// Turn off a function whose budget passed while the probe still holds
    /// it, so a device left half started neither writes memory nor
    /// interrupts.
    fn quiesce<C: crate::bus::pci::ConfigSpace>(&self, function: &mut Function<C>) {
        if self.budget.as_ref().is_some_and(|b| b.expired().is_some()) {
            let _ = function.abandon();
        }
    }
}

impl Drop for DeviceStart<'_> {
    fn drop(&mut self) {
        if let Some(expiry) = self.budget.as_ref().and_then(budget::Budget::expired) {
            self.abandoned.push((self.address, expiry));
        }
    }
}

impl Devices {
    /// How many outcomes the PCI functions' starts have recorded so far:
    /// devices, hosts, failures and reasons.
    fn recorded(&self) -> usize {
        self.disks.len()
            + self.gpus.len()
            + self.inputs.len()
            + self.audio.len()
            + self.usb_devices.len()
            + self.usb_hosts.len()
            + self.ehci_hosts.len()
            + self.hda_hosts.len()
            + self.usb4_hosts.len()
            + self.sd_hosts.len()
            + self.rtsx_hosts.len()
            + self.ahci_hosts.len()
            + self.bluetooth.len()
            + self.pcie_bluetooth.len()
            + self.nics.len()
            + self.failures.len()
            + self.device_failures.len()
            + self.usb_failures.len()
            + self.platform_failures.len()
            + self.unsupported.len()
            // A VMD host whose hierarchy holds no NVMe controller keeps
            // only its owner, as Linux reports no error.
            + self.owners.len()
    }

    /// The host subsystem the failure of the function at `address` belongs
    /// to: the one its error's kind names, or else the one the function's
    /// PCI class names, so a refusal common to every driver, such as the
    /// mount's, is filed with its own kind of device and not with the disks.
    pub fn subsystem(&self, address: Address, error: &Error) -> &'static str {
        let class = self
            .classes
            .iter()
            .find(|(found, _)| *found == address)
            .map(|(_, class)| *class);
        let by_class = match class {
            Some([0x01, ..]) => "storage",
            Some([0x02, 0x80, _]) => "Wi-Fi",
            Some([0x02, ..]) => "network",
            Some([0x03, ..]) => "GPU",
            Some([0x04, ..]) => "audio",
            Some([0x0c, 0x03, _]) => "USB",
            Some([0x0d, 0x11, _]) => "Bluetooth",
            _ => "device",
        };
        match error {
            Error::Enumeration(_) => "PCI",
            Error::Vmd(_) => "storage",
            // The r8169 Ethernet driver's; a Realtek card reader fails as a
            // device.
            Error::Realtek(_) | Error::Tg3(_) | Error::Aqtion(_) | Error::Pch(_) => "network",
            Error::IntelWifi(_) | Error::MediatekWifi(_) | Error::RealtekWifi(_) => "Wi-Fi",
            Error::IntelBluetooth(_) | Error::Bcm4377(_) => "Bluetooth",
            Error::Sfh(_) | Error::SfhCold(_) | Error::Ish(_) | Error::IshCold(_) => "sensors",
            Error::Ehci(_) => "USB",
            Error::HdaPci(_) | Error::Hda(_) | Error::HdaNative(_) => "audio",
            Error::Usb4(_) => "USB4",
            _ => by_class,
        }
    }

    /// Take up what an xHCI controller brought, at its start or later
    /// through a port change: disks, inputs, links, audio and Bluetooth,
    /// failures and the owners that keep them.
    pub fn absorb_usb(
        &mut self,
        address: Option<Address>,
        found: xhci::Found,
        firmware: &dyn crate::firmware::Provider,
    ) {
        self.disks.extend(
            found
                .disks
                .into_iter()
                .map(|d| Box::new(d) as Box<dyn BlockDevice>),
        );
        self.inputs.extend(
            found
                .inputs
                .into_iter()
                .map(|input| InputDevice::Usb(Box::new(input))),
        );
        self.usb_devices.extend(found.devices);
        self.usb_hosts.extend(found.hosts);
        self.usb_storage.extend(found.storage);
        self.audio.extend(found.audio);
        self.bluetooth_bootloaders.extend(found.bootloaders);
        for mut device in found.bluetooth {
            // The kept result owns its evidence/errors;
            // no catalog borrow or firmware bytes escape.
            let _ = device
                .initialize_with_provider(firmware, self.bluetooth_platform.platform_inputs());
            self.bluetooth.push(device);
        }
        self.nics.extend(found.links.into_iter().map(Adapter::Usb));
        self.usb_vendor_warnings
            .extend(found.vendor_links.iter().filter_map(|link| {
                let error = link.firmware_skipped.clone()?;
                Some((link.location.clone(), error))
            }));
        self.nics
            .extend(found.vendor_links.into_iter().map(Adapter::UsbVendor));
        self.usb_failures.extend(found.failures);
        self.usb_vendor_failures.extend(found.vendor_failures);
        self.usb_unused.extend(found.unused);
        if let Some(address) = address {
            self.usb_port_probes
                .extend(found.probes.into_iter().map(|probe| (address, probe)));
        }
    }

    /// Failures from attempted native vendor initialization, USB and PCIe,
    /// and USB controllers no vendor setup here takes, each named once: no
    /// consumer runs them, so they stay off.
    pub fn bluetooth_failures(&self) -> impl Iterator<Item = (&str, &dyn fmt::Display)> {
        let usb = self.bluetooth.iter().filter_map(|device| {
            let Err(error) = device.initialization()? else {
                return None;
            };
            Some((
                device.binding().device().location.as_str(),
                error as &dyn fmt::Display,
            ))
        });
        let pcie = self.pcie_bluetooth.iter().filter_map(|hci| {
            let error: &dyn fmt::Display = match &hci.controller {
                native::PcieController::Intel { initialization, .. } => {
                    initialization.as_ref().err()?
                }
                native::PcieController::Broadcom { initialization, .. } => {
                    initialization.as_ref().err()?
                }
            };
            Some((hci.location.as_str(), error))
        });
        usb.chain(pcie)
    }

    /// One plain line for each network adapter the drivers found and did
    /// not start, with its PCI address, IDs and why, or the one line that
    /// says none answered at all. Started adapters have none.
    pub fn network_account(&self) -> Vec<String> {
        let bluetooth: Vec<&str> = self
            .pcie_bluetooth
            .iter()
            .filter_map(|hci| hci.location.split(' ').next())
            .collect();
        self.network_lines(&bluetooth)
    }

    /// [`Self::network_account`], with the PCIe Bluetooth functions that
    /// started given by their addresses.
    fn network_lines(&self, bluetooth: &[&str]) -> Vec<String> {
        let mut lines = Vec::new();
        for function in &self.network_functions {
            let address = function.address;
            let at = alloc::format!("{address}");
            let started = self
                .nics
                .iter()
                .any(|nic| nic.location().split(' ').next() == Some(at.as_str()));
            let failure = self.failures.iter().find(|(found, _)| *found == address);
            // Broadcom's PCIe Bluetooth shares class 02/80 with Wi-Fi, started
            // or not; it is no network adapter.
            if bluetooth.contains(&at.as_str())
                || matches!(
                    failure,
                    Some((_, Error::Bcm4377(_) | Error::IntelBluetooth(_)))
                )
            {
                continue;
            }
            let abandoned = self.abandoned.iter().any(|(found, _)| *found == address);
            // A wait that ended for the budget only repeats the abandonment.
            let failure = failure.filter(|(_, error)| {
                !(abandoned
                    && alloc::format!("{}", plain(error))
                        .contains(crate::bus::pci::power::START_BUDGET))
            });
            let why = match (failure, abandoned) {
                (Some((_, error)), false) => alloc::format!("{}", plain(error)),
                (Some((_, error)), true) => alloc::format!(
                    "{} (its start took over {} s and was given up)",
                    plain(error),
                    DEVICE_START_US / 1_000_000
                ),
                (None, true) => alloc::format!(
                    "its start took over {} s and was given up",
                    DEVICE_START_US / 1_000_000
                ),
                (None, false) if started => continue,
                (None, false) => match self
                    .unsupported
                    .iter()
                    .find(|(found, ..)| *found == address)
                {
                    Some((_, _, why)) => String::from(*why),
                    None => String::from("no driver started it"),
                },
            };
            let kind = match function.class[1] {
                0x00 => "Ethernet",
                0x80 => "Wi-Fi",
                _ => "network controller",
            };
            lines.push(alloc::format!(
                "{at} {:04x}:{:04x} {kind}: {why}",
                function.vendor,
                function.device
            ));
        }
        lines.extend(
            self.usb_vendor_failures
                .iter()
                .map(|(location, error)| alloc::format!("USB {location} Ethernet: {error}")),
        );
        if lines.is_empty() && self.nics.is_empty() {
            lines.push(String::from(
                "no network-class device answered on PCI, and no USB network adapter was found",
            ));
        }
        lines
    }
}

/// The screen line for a started adapter that never got a link: a wired
/// port is told to check its cable, a Wi-Fi radio that it did not join.
pub fn no_link(location: &str, wireless: bool) -> String {
    if wireless {
        alloc::format!("{location}: not connected to the Wi-Fi network.")
    } else {
        alloc::format!(
            "{location}: no link. Check that a network cable is plugged in and that the other end \
             is on."
        )
    }
}

/// A network adapter's failure as its account line shows it: the line
/// already names the adapter by its IDs and kind, so no driver's name leads
/// the reason, for any wired or Wi-Fi driver.
fn plain(error: &Error) -> impl fmt::Display + '_ {
    struct Plain<'a>(&'a Error);
    impl fmt::Display for Plain<'_> {
        fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
            match self.0 {
                Error::Pch(error) => error.fmt(f),
                Error::Tg3(error) => error.fmt(f),
                Error::Aqtion(error) => error.fmt(f),
                Error::IntelWifi(error) => error.fmt(f),
                Error::MediatekWifi(error) => error.fmt(f),
                Error::RealtekWifi(error) => error.fmt(f),
                error => error.fmt(f),
            }
        }
    }
    Plain(error)
}

/// Enumerate PCI and bring up every virtio-blk, NVMe and AHCI controller,
/// every virtio-input device, every virtio-net and supported Intel/Realtek Ethernet adapter,
/// and every xHCI
/// controller with the USB disks, keyboards, pointers and Ethernet
/// adapters on it.
///
/// # Safety
///
/// `handoff` must describe the running machine after ExitBootServices,
/// with its ranges mapped as it says, and nothing else may drive these
/// controllers afterwards.
pub unsafe fn probe(
    handoff: &Handoff,
    captures: PlatformCaptures,
    dma: &mut dyn DmaAllocator,
) -> Devices {
    let firmware = handoff_provider(handoff);
    // SAFETY: the caller granted the complete machine/controller contract.
    unsafe { probe_with_provider(handoff, captures, dma, firmware, None, None) }
}

/// The handoff's firmware catalog as the provider [`probe_with_provider`]
/// takes. Attached devices keep borrowing their firmware for as long as
/// they run, so the catalog lives for the rest of the boot.
pub fn handoff_provider(handoff: &Handoff) -> &'static crate::firmware::HandoffProvider<'static> {
    let catalog = handoff
        .firmware
        .clone()
        // SAFETY: the reference count is never given back, so the catalog
        // stays allocated for every later borrow.
        .map(|catalog| unsafe { &*Arc::into_raw(catalog) });
    Box::leak(Box::new(crate::firmware::HandoffProvider::new(catalog)))
}

/// What every Intel Wi-Fi attach in one probe shares.
struct WifiInputs<'a> {
    clock: Clock,
    observations: net::wifi::intel::platform::Observations,
    firmware: &'static dyn crate::firmware::Provider,
    network: &'a Network,
}

/// Attach the Intel Wi-Fi function at `address` for `wifi.network`, reading
/// its ACPI companion, once the regulatory database loaded.
///
/// # Safety
/// [`NativeServices`]' contract holds for `services`, and nothing else
/// reaches the function.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
unsafe fn intel_wifi(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    wifi: &WifiInputs<'_>,
    regulatory: &Option<
        Result<
            crate::net::wifi::regulatory::database::Database,
            crate::net::wifi::regulatory::database::LoadError,
        >,
    >,
    bridges: &[native::Bridge],
) -> Result<Adapter, native::Failure> {
    use net::wifi::intel::attach::AttachError;
    let Some(Ok(database)) = regulatory else {
        return Err((Error::IntelWifi(AttachError::NoRegulatoryDatabase), None));
    };
    let WifiInputs {
        clock,
        observations,
        firmware,
        network,
    } = *wifi;
    #[cfg(feature = "native-acpi")]
    {
        let session = services.acpi();
        let companion = session.and_then(|session| native::companion(session, address, bridges));
        // SAFETY: `companion` is this function's own, and the handoff's
        // captured variables are this boot's.
        let bios = unsafe {
            net::wifi::intel::radio::bios::service::native::NativeBios::new(
                session,
                companion,
                firmware,
                observations,
            )
        };
        let inputs = net::wifi::intel::link::Inputs {
            firmware,
            observations,
            pages: services.pages(),
            clock,
            bios,
            database: database.clone(),
            network: network.clone(),
            entropy: services
                .entropy()
                .map_err(|error| (Error::Native(error), None))?,
        };
        // SAFETY: forwarded from the caller.
        unsafe { native::intel_wifi(services, address, completion, inputs) }
            .map(|link| Adapter::IntelWifi(Box::new(link)))
    }
    #[cfg(not(feature = "native-acpi"))]
    {
        let _ = (
            services,
            address,
            completion,
            clock,
            observations,
            firmware,
            network,
            database,
            bridges,
        );
        Err((Error::IntelWifi(AttachError::NoNamespace), None))
    }
}

/// Attach the MediaTek Wi-Fi function at `address` for `wifi.network`, its
/// SAR tables read from its ACPI companion, once the regulatory database
/// loaded. `aspm` is whether firmware enabled ASPM on its link.
///
/// # Safety
/// As for [`intel_wifi`].
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
unsafe fn mediatek_wifi(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    wifi: &WifiInputs<'_>,
    regulatory: &Option<
        Result<
            crate::net::wifi::regulatory::database::Database,
            crate::net::wifi::regulatory::database::LoadError,
        >,
    >,
    bridges: &[native::Bridge],
    aspm: bool,
) -> Result<
    native::Located<net::wifi::mediatek::attach::Attached<native::MediatekOwner>>,
    native::Failure,
> {
    use net::wifi::mediatek::attach::AttachError;
    let Some(Ok(database)) = regulatory else {
        return Err((Error::MediatekWifi(AttachError::NoRegulatoryDatabase), None));
    };
    let entropy = services
        .entropy()
        .map_err(|error| (Error::Native(error), None))?;
    #[cfg(feature = "native-acpi")]
    let sar = net::wifi::mediatek::acpi::read(
        services
            .acpi()
            .and_then(|session| native::companion(session, address, bridges)),
    );
    #[cfg(not(feature = "native-acpi"))]
    let sar = {
        let _ = bridges;
        net::wifi::mediatek::acpi::Sar::default()
    };
    let inputs = net::wifi::mediatek::attach::Inputs {
        firmware: wifi.firmware,
        pages: services.pages(),
        clock: wifi.clock,
        vector: false,
        aspm,
        database: database.clone(),
        network: wifi.network.clone(),
        sar,
        entropy,
    };
    // SAFETY: forwarded from the caller.
    unsafe { native::mediatek_wifi(services, address, completion, inputs) }
}

/// Attach the RTL8822C at `address` for `wifi.network`, once the regulatory
/// database loaded.
///
/// # Safety
/// As for [`intel_wifi`].
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
unsafe fn realtek_wifi(
    services: &dyn NativeServices,
    address: Address,
    completion: Option<crate::bus::irq::MsiMessage>,
    wifi: &WifiInputs<'_>,
    regulatory: &Option<
        Result<
            crate::net::wifi::regulatory::database::Database,
            crate::net::wifi::regulatory::database::LoadError,
        >,
    >,
) -> Result<native::Located<native::RealtekWifi>, native::Failure> {
    use net::wifi::realtek::attach::{AttachError, Inputs};
    let Some(Ok(database)) = regulatory else {
        return Err((Error::RealtekWifi(AttachError::NoRegulatoryDatabase), None));
    };
    let inputs = Inputs {
        firmware: wifi.firmware,
        pages: services.pages(),
        clock: wifi.clock,
        database: database.clone(),
        network: wifi.network.clone(),
        entropy: services
            .entropy()
            .map_err(|error| (Error::Native(error), None))?,
        vector: false,
        link_control: 0,
    };
    // SAFETY: forwarded from the caller.
    unsafe { native::realtek_wifi(services, address, completion, inputs) }
}

/// Attach the Broadcom PCIe Bluetooth function at `address` with its
/// segment's bridges, for the board SMBIOS names, as `bcm4377_probe_dmi`
/// selects it.
///
/// # Safety
/// As for [`intel_wifi`]; `bridges` lists every bridge of the segment.
#[allow(
    clippy::result_large_err,
    reason = "a failed attach hands back the owners that must outlive the device"
)]
unsafe fn bcm4377(
    services: &dyn NativeServices,
    address: Address,
    bridges: &[native::Bridge],
    firmware: &'static dyn crate::firmware::Provider,
    clock: Clock,
) -> Result<native::Bcm4377, native::Failure> {
    use crate::bluetooth::bcm4377::{Board, Error as Bcm4377Error};
    use crate::firmware::smbios::Observation;
    let refused = |error: ServiceError| (Error::Native(error), None);
    let text = |observation| match observation {
        Observation::Value(bytes) => core::str::from_utf8(bytes).ok(),
        _ => None,
    };
    let board = crate::firmware::smbios::load(firmware)
        .ok()
        .and_then(|snapshot| {
            let identity = snapshot.identity;
            Board::from_dmi(text(identity.system_vendor)?, text(identity.product_name)?).ok()
        })
        .ok_or((
            Error::Bcm4377(Bcm4377Error::Unsupported(
                "an SMBIOS system the source's DMI board table lists",
            )),
            None,
        ))?;
    #[cfg(feature = "native-acpi")]
    let roots = services
        .acpi()
        .map(|session| native::root_buses(session, address.segment))
        .unwrap_or_default();
    #[cfg(not(feature = "native-acpi"))]
    let roots = Vec::new();
    if roots.is_empty() {
        return Err(refused(ServiceError::Refused(
            "no ACPI host bridge declares the segment's root buses",
        )));
    }
    let mut owned = Vec::new();
    for bridge in bridges
        .iter()
        .filter(|bridge| bridge.address.segment == address.segment)
    {
        owned.push(services.bridge(bridge.address).map_err(refused)?);
    }
    // SAFETY: these are the mount's owners of every bridge the scan found
    // in the segment and the root buses its ACPI host bridges declare; no
    // other driver sets their link policy.
    let topology = unsafe { crate::bus::pci::native_aspm::Topology::new(owned, roots) };
    // SAFETY: forwarded from the caller.
    unsafe { native::bcm4377(services, address, topology, board, clock) }
}

/// Enumerate and attach devices using an explicit exact firmware provider.
/// A loader can observe consumer requirements while delegating the original
/// lookup result, without reconstructing device identities outside drivers.
/// This performs the same hardware operations and reports the same failures as
/// [`probe`]; a missing resource still refuses that controller's initialization.
/// `services` supplies the native owners some drivers attach through.
/// `wifi` is the network the mount's configuration names; without one a
/// Wi-Fi adapter is unsupported and needs no Wi-Fi firmware or regulatory
/// data.
///
/// # Safety
/// The machine, mappings, DMA and exclusive post-ExitBootServices ownership
/// requirements of [`probe`] apply. The provider must preserve its documented
/// exact identity and integrity contract for every resource it returns. It
/// lives for the rest of the boot because attached devices keep borrowing
/// the firmware it returned.
pub unsafe fn probe_with_provider(
    handoff: &Handoff,
    captures: PlatformCaptures,
    dma: &mut dyn DmaAllocator,
    firmware: &'static dyn crate::firmware::Provider,
    services: Option<&dyn NativeServices>,
    wifi: Option<&Network>,
) -> Devices {
    // SAFETY: forwarded.
    unsafe {
        probe_observed(
            handoff,
            captures,
            dma,
            firmware,
            services,
            wifi,
            &mut Unobserved,
        )
    }
}

/// What the mount does around the PCI functions' starts.
pub trait Observer {
    /// The function to start before every other, such as the controller
    /// of the disk Stela started from.
    fn first(&self) -> Option<Address> {
        None
    }
    /// Once [`Self::first`]'s start has ended, before any other starts.
    fn first_started(&mut self, _devices: &mut Devices) {}
    /// Once a function is named as starting, right before its driver runs,
    /// between two drivers' calls: whether its driver runs.
    fn starting(&mut self, _function: &crate::bus::pci::Identity) -> Start {
        Start::Run
    }
}

/// Whether a function's driver runs, as [`Observer::starting`] decides.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Start {
    Run,
    /// Left untouched and recorded as not driven, for this reason.
    Skip(&'static str),
}

/// No observer.
pub struct Unobserved;
impl Observer for Unobserved {}

/// `scanned` with the function at `first` alone in a leading group, the
/// others in their scan order; whether it was found.
fn first_of(scanned: Vec<Vec<Function>>, first: Option<Address>) -> (Vec<Vec<Function>>, bool) {
    let Some(first) = first else {
        return (scanned, false);
    };
    let mut leading = Vec::new();
    let mut rest = Vec::with_capacity(scanned.len() + 1);
    for found in scanned {
        let (taken, others): (Vec<_>, Vec<_>) = found
            .into_iter()
            .partition(|function| function.address == first);
        leading.extend(taken);
        rest.push(others);
    }
    let found = !leading.is_empty();
    if found {
        rest.insert(0, leading);
    }
    (rest, found)
}

/// [`probe_with_provider`], with `observer` told of each start.
///
/// # Safety
/// As for [`probe_with_provider`]; `observer` drives no device the probe
/// holds while it is called, beyond those [`Observer::first_started`] hands
/// it.
pub unsafe fn probe_observed(
    handoff: &Handoff,
    captures: PlatformCaptures,
    dma: &mut dyn DmaAllocator,
    firmware: &'static dyn crate::firmware::Provider,
    services: Option<&dyn NativeServices>,
    wifi: Option<&Network>,
    observer: &mut dyn Observer,
) -> Devices {
    let mut devices = Devices {
        bluetooth_platform: captures.bluetooth,
        native_services: services.is_some(),
        ..Devices::default()
    };
    let wifi_inputs = wifi.map(|network| WifiInputs {
        clock: handoff.clock,
        observations: captures.wifi,
        firmware,
        network,
    });
    // Broadcom PCIe Bluetooth functions, attached once the scan has found
    // every bridge of their segment.
    let mut broadcom_bluetooth = Vec::new();
    let legacy_pci_writes = services.is_none_or(NativeServices::legacy_pci_writes);
    let mut ehci_hosts = Vec::new();
    let mut usb_handoff_failures = Vec::new();
    // USB4 hosts with a software connection manager come up first: the scan
    // below then finds what they tunnel, as it finds any other function.
    let software_usb4 = services.map_or_else(Vec::new, |services| {
        // SAFETY: the caller owns the machine, and no driver has started.
        unsafe { native::usb4::bring_up(services, handoff, &mut devices) }
    });
    let mut usb4_hosts = Vec::new();
    // Each PCI bridge and its secondary bus, for ACPI companion lookups.
    let mut bridges = Vec::new();
    // i801 SMBus hosts, attached under ACPI arbitration once the embedded
    // controller is.
    #[cfg(feature = "native-acpi")]
    let mut smbus_functions = Vec::new();
    let minimum_result = match services {
        Some(services) => services.minimum_delay(),
        None => Err(ServiceError::Unavailable),
    };
    // Every window is enumerated before any driver runs, as Linux finds
    // every device before its fixups: the Intel xHCI port switch asks
    // whether an Intel EHCI exists anywhere.
    let (scanned, pci_scan_complete) = scan(handoff, &mut devices);
    let mut abandoned = Vec::new();
    let intel_ehci = scanned.iter().flatten().any(|function| {
        function.vendor == 0x8086
            && usb::handoff::Kind::of(function.class) == Some(usb::handoff::Kind::Ehci)
    });
    let displays: Vec<Address> = scanned
        .iter()
        .flatten()
        .filter(|function| function.class[0] == 0x03 && function.address.function == 0)
        .map(|function| function.address)
        .collect();
    // Everything a function's start can record, here or in a later attach.
    macro_rules! recorded {
        () => {{
            #[cfg(feature = "native-acpi")]
            let smbus = smbus_functions.len();
            #[cfg(not(feature = "native-acpi"))]
            let smbus = 0;
            devices.recorded()
                + smbus
                + ehci_hosts.len()
                + usb_handoff_failures.len()
                + broadcom_bluetooth.len()
                + usb4_hosts.len()
        }};
    }
    // xHCI controllers started in the scan, whose devices are enumerated
    // after it.
    let mut xhci_started = Vec::new();
    let mut claimed = Claimed::default();
    // A driver that enables a function needs every bridge above it to
    // forward the function's DMA and message interrupts upstream: Linux's
    // `pci_enable_bridge`. A mount that stopped the bridges and did not
    // reopen them itself leaves that to this probe, which opens them all
    // before any function starts, the observer's first one included.
    let mut scanned = scanned;
    if legacy_pci_writes {
        for function in scanned.iter_mut().flatten() {
            if function.class[..2] != [6, 4] {
                continue;
            }
            let address = function.address;
            let opened = (|| -> Result<bool, BlockError> {
                use crate::bus::pci::{ConfigSpace, regs};
                if function.config().read16(regs::COMMAND)? & regs::COMMAND_MASTER != 0 {
                    return Ok(false);
                }
                function.enable_memory_and_bus_master()?;
                Ok(true)
            })();
            match opened {
                Ok(true) => devices.bridges_reopened.push(address),
                Ok(false) => {}
                Err(error) => devices.failures.push((address, error.into())),
            }
        }
    }
    let (scanned, mut first_pending) = first_of(scanned, observer.first());
    let mut seen = 0usize;
    for found in scanned {
        for mut function in found {
            claimed.settle(recorded!(), &mut devices.failures);
            if seen > 0 && core::mem::take(&mut first_pending) {
                observer.first_started(&mut devices);
            }
            seen += 1;
            let address = function.address;
            devices.classes.push((address, function.class));
            if scan_starts(&function.identity(), &software_usb4) {
                claimed.begin(address, recorded!());
            }
            if function.class[0] == 0x02 {
                devices.network_functions.push(function.identity());
            }
            // A network adapter joins the links, or its failure is recorded.
            macro_rules! attach_link {
                ($attached:expr, $variant:path) => {{
                    match $attached {
                        Ok(device) => devices.nics.push($variant(device)),
                        Err(error) => devices.failures.push((address, error.into())),
                    }
                    continue;
                }};
            }
            macro_rules! minimum_for_function {
                () => {
                    match &minimum_result {
                        Ok(minimum) => minimum,
                        Err(error) => {
                            devices.failures.push((address, Error::Native(*error)));
                            devices.owners.push(Box::new(function));
                            continue;
                        }
                    }
                };
            }
            if function.class[..2] == [6, 4] {
                use crate::bus::pci::ConfigSpace;
                let checked = (|| -> Result<_, BlockError> {
                    Ok(native::Bridge {
                        address,
                        secondary: function.config().read8(0x19)?,
                        aspm: crate::bus::pci::aspm_control(&function)?,
                    })
                })();
                match checked {
                    Ok(bridge) => bridges.push(bridge),
                    Err(error) => {
                        devices.failures.push((address, error.into()));
                        devices.owners.push(Box::new(function));
                        continue;
                    }
                }
            }
            if function.class[..2] == [2, 0x80] {
                // A chip Linux drives and no driver here does is named once,
                // with its Linux driver, and left untouched.
                if let Some(why) =
                    crate::net::wifi::unsupported::reason(function.vendor, function.device)
                {
                    devices.unsupported.push((address, function.class, why));
                    continue;
                }
                if wifi.is_none()
                    && matches!(
                        native::Kind::of(&function),
                        None | Some(
                            native::Kind::IntelWifi
                                | native::Kind::MediatekWifi
                                | native::Kind::RealtekWifi
                        )
                    )
                {
                    // No Wi-Fi hardware is touched without a network. Other
                    // functions of this class, such as Broadcom Bluetooth,
                    // still attach.
                    devices
                        .unsupported
                        .push((address, function.class, NO_WIFI_NETWORK));
                    continue;
                }
                if wifi.is_some() && devices.regulatory.is_none() {
                    devices.regulatory =
                        Some(crate::net::wifi::regulatory::database::load(firmware));
                }
            }
            if let Some(adapter) = gpu::Adapter::discover(&function) {
                devices.gpus.push(adapter);
                continue;
            }
            crate::working(&alloc::format!("PCI {address}"));
            if let Start::Skip(why) = observer.starting(&function.identity()) {
                devices.unsupported.push((address, function.class, why));
                continue;
            }
            let mut start = DeviceStart::begin(address, handoff.clock, &mut abandoned);
            #[cfg(feature = "native-acpi")]
            if smbus::matches(&function) {
                smbus_functions.push(address);
                continue;
            }
            if let Some(kind) = usb::handoff::Kind::of(function.class) {
                let inputs = UsbHandoff {
                    handoff,
                    minimum: &minimum_result,
                    legacy_pci_writes,
                    intel_ehci,
                };
                let Some(handed) = usb_handoff(
                    function,
                    kind,
                    &inputs,
                    &mut ehci_hosts,
                    &mut usb_handoff_failures,
                    &mut devices,
                ) else {
                    continue;
                };
                function = handed;
            }
            let result = if let Some(kind) = native::Kind::of(&function) {
                let Some(services) = services else {
                    devices
                        .failures
                        .push((address, Error::Native(ServiceError::Unavailable)));
                    continue;
                };
                match kind {
                    // Attached once the scan has found every bridge of the
                    // segment.
                    native::Kind::Bcm4377 => broadcom_bluetooth.push(address),
                    native::Kind::Usb4 => {
                        if !software_usb4.contains(&address) {
                            usb4_hosts.push(address);
                        }
                    }
                    // SAFETY: the caller grants `services`' contract, and
                    // this loop leaves `function` alone once it is claimed.
                    _ => unsafe {
                        attach_native(
                            kind,
                            function,
                            services,
                            &NativeInputs {
                                handoff,
                                firmware,
                                minimum: &minimum_result,
                                wifi: wifi_inputs.as_ref(),
                                bridges: &bridges,
                            },
                            &mut devices,
                        )
                    },
                }
                continue;
            } else if !legacy_pci_writes {
                if function.class[..2] != [6, 4] {
                    devices.unsupported.push((
                        address,
                        function.class,
                        "legacy PCI writes are excluded by the blocked requester fabric",
                    ));
                }
                continue;
            } else if virtio::matches(&function) {
                let minimum = minimum_for_function!().clone();
                let attached =
                    virtio::VirtioBlk::new(&mut function, handoff, dma, minimum.as_ref());
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attached.map(|device| vec![Box::new(device) as Box<dyn BlockDevice>])
            } else if function.class == nvme::CLASS {
                let minimum = minimum_for_function!().clone();
                // Linux's board quirks read the SMBIOS system and baseboard.
                let snapshot = crate::firmware::smbios::load(firmware).ok();
                let platform = snapshot
                    .as_ref()
                    .map(|snapshot| snapshot.identity)
                    .unwrap_or_default();
                let attached =
                    nvme::attach(&mut function, handoff, dma, minimum.as_ref(), &platform);
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attached.map(|attached| {
                    if !attached.quirks.is_empty() {
                        devices
                            .warnings
                            .push((address, Warning::NvmeQuirks(attached.quirks)));
                    }
                    if let Some(scan) = attached.sequential {
                        devices.warnings.push((address, Warning::NvmeScan(scan)));
                    }
                    devices
                        .device_failures
                        .extend(attached.failures.into_iter().map(|(namespace, error)| {
                            (alloc::format!("{address} namespace {namespace}"), error)
                        }));
                    attached
                        .namespaces
                        .into_iter()
                        .map(|device| Box::new(device) as Box<dyn BlockDevice>)
                        .collect()
                })
            } else if function.class == xhci::CLASS {
                let minimum = minimum_for_function!().clone();
                // Every controller but the boot's own is started here and
                // its devices are enumerated after the scan: its ports
                // settle while the other devices start, as Linux's hub
                // threads settle beside its other drivers.
                if observer.first() != Some(address) {
                    match xhci::begin_with_provider(
                        &mut function,
                        handoff,
                        dma,
                        firmware,
                        minimum.clone(),
                    ) {
                        Ok(started) => {
                            claimed.started();
                            drop(start);
                            xhci_started.push((function, minimum, started));
                        }
                        Err(error) => {
                            start.quiesce(&mut function);
                            devices.failures.push((address, error.into()));
                        }
                    }
                    continue;
                }
                let attached = xhci::attach_with_provider(
                    &mut function,
                    handoff,
                    dma,
                    firmware,
                    minimum.clone(),
                );
                absorb_xhci(
                    attached,
                    &mut function,
                    &XhciInputs {
                        handoff,
                        firmware,
                        minimum: &minimum,
                    },
                    dma,
                    &mut start,
                    &mut devices,
                );
                continue;
            } else if net::virtio::matches(&function) {
                let minimum = minimum_for_function!().clone();
                let attached = VirtioNet::new(&mut function, handoff, dma, minimum.as_ref());
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Virtio);
            } else if net::igc::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: the caller grants exclusive post-EBS controller,
                // segment configuration and firmware/ASPM policy ownership.
                let attached = unsafe { Igc::new(&mut function, handoff, dma, minimum.clone()) };
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Igc);
            } else if net::e1000e::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: the caller granted exclusive ownership of all mapped PCI
                // functions and controllers after ExitBootServices.
                let attached = unsafe { E1000e::new(&mut function, handoff, dma, minimum.clone()) };
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::E1000e);
            } else if pch::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // The MTP/PTP cohorts read Linux's K1 board quirk from the
                // SMBIOS system fields; without them those cohorts refuse.
                let snapshot = crate::firmware::smbios::load(firmware).ok();
                let host = snapshot
                    .as_ref()
                    .map(|snapshot| pch::host_identity(&snapshot.identity))
                    .unwrap_or_default();
                // SAFETY: as for e1000e; the adapter keeps the function, its
                // configuration and both register windows to the end of boot.
                let attached =
                    unsafe { pch::Pch::attach(function, handoff, dma, minimum.clone(), host) };
                if let Ok(device) = &attached
                    && device.nvm_repair_left_out()
                {
                    devices.warnings.push((
                        address,
                        Warning::Message(
                            "e1000e: the NVM checksum valid bit is clear; Linux would set it and \
                             rewrite the checksum in flash, Stela runs the adapter without \
                             writing the NVM",
                        ),
                    ));
                }
                if let Ok(device) = &attached
                    && device.ulp_exit_failed()
                {
                    devices.warnings.push((
                        address,
                        Warning::Message(
                            "e1000e: the PHY did not leave its low-power state when asked; \
                             Stela went on to its PHY checks, as Linux does",
                        ),
                    ));
                }
                attach_link!(attached.map(Box::new), Adapter::Pch);
            } else if net::igb::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: as for e1000e, the caller granted exclusive ownership
                // of the mapped PCI functions and their controllers.
                let attached = unsafe { Igb::new(&mut function, handoff, dma, minimum.clone()) };
                if let Ok(device) = &attached {
                    if device.warnings.nvm_autoload_timeout {
                        devices.warnings.push((
                            address,
                            Warning::Message(
                                "igb NVM autoload completion timed out; NVM contents validated",
                            ),
                        ));
                    }
                    if device.warnings.phy_configuration_timeout {
                        devices.warnings.push((
                            address,
                            Warning::Message("igb PHY configuration completion timed out; PHY transactions validated"),
                        ));
                    }
                }
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Igb);
            } else if net::alx::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: as for e1000e, the caller granted exclusive ownership
                // of the mapped PCI functions and their controllers.
                let attached = unsafe { Alx::new(&mut function, handoff, dma, minimum.clone()) };
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Alx);
            } else if net::atl1c::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: as for alx, the caller granted exclusive ownership
                // of the mapped PCI functions and their controllers.
                let attached = unsafe { Atl1c::new(&mut function, handoff, dma, minimum.clone()) };
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Atl1c);
            } else if realtek::matches(&function) {
                let minimum = minimum_for_function!().clone();
                // SAFETY: the caller granted exclusive post-EBS ownership of
                // mapped PCI/MMIO and DMA, including upstream ASPM configuration.
                let attached =
                    unsafe { Realtek::new(&mut function, handoff, dma, firmware, minimum.clone()) };
                if let Ok(device) = &attached {
                    devices.warnings.extend(
                        device
                            .warnings()
                            .iter()
                            .map(|warning| (address, Warning::Realtek(*warning))),
                    );
                }
                if attached.is_err() {
                    start.quiesce(&mut function);
                }
                attach_link!(attached, Adapter::Realtek);
            } else if input::virtio::matches(&function) {
                let minimum = minimum_for_function!().clone();
                match VirtioInput::new(&mut function, handoff, dma, minimum.as_ref()) {
                    Ok(device) => devices.inputs.push(InputDevice::Virtio(Box::new(device))),
                    Err(error) => devices.failures.push((address, error.into())),
                }
                continue;
            } else if rtsx::matches(&function) {
                let minimum = minimum_for_function!().clone();
                let attached = match rtsx::attach(function, handoff, minimum.clone()) {
                    Ok(attached) => attached,
                    Err(failure) => {
                        devices
                            .failures
                            .push((address, Error::Device(failure.error.clone())));
                        devices.owners.push(Box::new(failure));
                        continue;
                    }
                };
                Ok({
                    devices.rtsx_hosts.push(attached.host);
                    if let Some(error) = attached.failure {
                        devices
                            .device_failures
                            .push((alloc::format!("{address} RTSX SD slot"), error));
                    }
                    attached
                        .card
                        .into_iter()
                        .map(|card| Box::new(card) as Box<dyn BlockDevice>)
                        .collect()
                })
            } else if sdhci::matches(&function) {
                let minimum = minimum_for_function!().clone();
                let attached = match sdhci::attach(function, handoff, dma, minimum.clone()) {
                    Ok(attached) => attached,
                    Err(failure) => {
                        devices
                            .failures
                            .push((address, Error::Device(failure.error.clone())));
                        devices.owners.push(Box::new(failure));
                        continue;
                    }
                };
                Ok({
                    for host in attached.failed_hosts {
                        devices.owners.push(Box::new(host));
                    }
                    for slot in attached.failed_slots {
                        devices.owners.push(Box::new(slot));
                    }
                    devices.sd_hosts.extend(attached.hosts);
                    devices.device_failures.extend(
                        attached.failures.into_iter().map(|(slot, error)| {
                            (alloc::format!("{address} SD slot {slot}"), error)
                        }),
                    );
                    attached
                        .cards
                        .into_iter()
                        .map(|card| Box::new(card) as Box<dyn BlockDevice>)
                        .collect()
                })
            } else if ahci::matches(&function) {
                let minimum = minimum_for_function!().clone();
                let attached = match ahci::attach(function, handoff, dma, minimum.clone()) {
                    Ok(attached) => attached,
                    Err(failure) => {
                        devices
                            .failures
                            .push((address, Error::Device(failure.error.clone())));
                        devices.owners.push(Box::new(failure));
                        continue;
                    }
                };
                Ok({
                    if attached.remapped_nvme != 0 {
                        devices
                            .warnings
                            .push((address, Warning::Message(ahci::REMAPPED_NVME)));
                    }
                    devices.ahci_hosts.push(attached.host);
                    devices.device_failures.extend(
                        attached
                            .failures
                            .into_iter()
                            .map(|(port, error)| (alloc::format!("{address} port {port}"), error)),
                    );
                    attached
                        .disks
                        .into_iter()
                        .map(|device| Box::new(device) as Box<dyn BlockDevice>)
                        .collect()
                })
            } else {
                if let Some(why) = unclaimed(function.class, address, &displays) {
                    devices.unsupported.push((address, function.class, why));
                }
                continue;
            };
            match result {
                Ok(found) => {
                    claimed.started();
                    devices.disks.extend(found);
                }
                Err(error) => devices.failures.push((address, error.into())),
            }
        }
    }
    claimed.settle(recorded!(), &mut devices.failures);
    if first_pending {
        observer.first_started(&mut devices);
    }
    for (mut function, minimum, started) in xhci_started {
        let address = function.address;
        crate::working(&alloc::format!("PCI {address}"));
        // Named again, so a start that never ends is this controller's.
        if let Start::Skip(why) = observer.starting(&function.identity()) {
            devices.unsupported.push((address, function.class, why));
            core::mem::forget(started);
            continue;
        }
        let mut start = DeviceStart::begin(address, handoff.clock, &mut abandoned);
        absorb_xhci(
            Ok(started.finish(dma, firmware)),
            &mut function,
            &XhciInputs {
                handoff,
                firmware,
                minimum: &minimum,
            },
            dma,
            &mut start,
            &mut devices,
        );
    }
    devices.abandoned = abandoned;
    let companions_released = pci_scan_complete
        && !usb_handoff_failures
            .iter()
            .any(|kind| matches!(kind, usb::handoff::Kind::Uhci | usb::handoff::Kind::Ohci));
    attach_ehci_hosts(
        ehci_hosts,
        companions_released,
        services,
        handoff,
        &mut devices,
    );
    attach_usb4_hosts(usb4_hosts, services, &bridges, handoff, &mut devices);
    attach_broadcom_bluetooth(
        broadcom_bluetooth,
        services,
        &bridges,
        firmware,
        handoff,
        &mut devices,
    );
    // After every USB host left firmware, as Linux's i8042 probe runs after
    // its PCI fixups.
    #[cfg(feature = "native-acpi")]
    attach_platform(services, smbus_functions, &bridges, handoff, &mut devices);
    devices
}

/// What an xHCI controller's attachment needs again when it is retried.
struct XhciInputs<'a> {
    handoff: &'a Handoff,
    firmware: &'static dyn crate::firmware::Provider,
    minimum: &'a Rc<dyn crate::bus::pci::power::Timer>,
}

/// Record an xHCI controller's attachment. A controller that stopped
/// answering while its devices started has lost every device below it, the
/// boot stick and the keyboard among them. Its reset starts it again, so
/// it is attached once more. The first attachment's owners are kept, never
/// dropped: their rings stay allocated and they touch no register again.
fn absorb_xhci(
    mut attached: Result<xhci::Found, BlockError>,
    function: &mut Function,
    inputs: &XhciInputs<'_>,
    dma: &mut dyn DmaAllocator,
    start: &mut DeviceStart<'_>,
    devices: &mut Devices,
) {
    let address = function.address;
    if let Ok(found) = &attached
        && found.controller_died()
    {
        devices
            .warnings
            .push((address, Warning::XhciDied(found.controller_death(), false)));
        if let Ok(found) = core::mem::replace(
            &mut attached,
            Err(BlockError::Unsupported("xHCI host controller died")),
        ) {
            core::mem::forget(found);
        }
        start.renew(inputs.handoff.clock);
        attached = xhci::attach_with_provider(
            function,
            inputs.handoff,
            dma,
            inputs.firmware,
            inputs.minimum.clone(),
        );
        if let Ok(found) = &attached
            && found.controller_died()
        {
            devices
                .warnings
                .push((address, Warning::XhciDied(found.controller_death(), true)));
        }
    }
    match attached {
        Ok(found) => devices.absorb_usb(Some(address), found, inputs.firmware),
        Err(error) => {
            start.quiesce(function);
            devices.failures.push((address, error.into()));
        }
    }
}

/// What the USB hosts' firmware handoffs in one scan share.
struct UsbHandoff<'a> {
    handoff: &'a Handoff,
    minimum: &'a Result<Rc<dyn crate::bus::pci::power::Timer>, ServiceError>,
    /// Whether the mount allows the legacy PCI writes the handoff makes.
    legacy_pci_writes: bool,
    /// Whether the scan found an Intel EHCI anywhere.
    intel_ehci: bool,
}

/// Take a USB host of `kind` from firmware before any driver binds, as
/// Linux's early PCI fixup does, and return it for its driver. EHCI hosts
/// go to `ehci_hosts` for their native attach. `None` when the function
/// goes no further in this scan.
fn usb_handoff(
    mut function: Function,
    kind: usb::handoff::Kind,
    inputs: &UsbHandoff<'_>,
    ehci_hosts: &mut Vec<Address>,
    usb_handoff_failures: &mut Vec<usb::handoff::Kind>,
    devices: &mut Devices,
) -> Option<Function> {
    let address = function.address;
    // EHCI's native attach takes the controller through its own
    // owners when legacy writes are excluded.
    if kind == usb::handoff::Kind::Ehci && !inputs.legacy_pci_writes {
        ehci_hosts.push(address);
        return None;
    }
    if !inputs.legacy_pci_writes {
        usb_handoff_failures.push(kind);
        devices.failures.push((
            address,
            Error::Native(ServiceError::Refused("legacy USB PCI handoff unavailable")),
        ));
        return None;
    }
    let minimum = match inputs.minimum {
        Ok(minimum) => minimum,
        Err(error) => {
            usb_handoff_failures.push(kind);
            devices.failures.push((address, Error::Native(*error)));
            devices.owners.push(Box::new(function));
            return None;
        }
    };
    // SAFETY: the caller owns the machine and no driver has bound
    // this controller yet, as Linux's early PCI fixup runs. Every
    // host, EHCI included, leaves firmware here, so its SMIs are
    // off even when its driver later fails to attach.
    match unsafe { usb::handoff::early(&mut function, inputs.handoff, minimum.as_ref()) } {
        Some(Ok(handed)) if handed.forced => devices.warnings.push((
            address,
            Warning::Message(
                "USB host firmware handoff timed out; ownership taken with its SMIs off",
            ),
        )),
        Some(Err(error)) => {
            devices.platform_failures.push((
                alloc::format!("{address} {} handoff", kind.name()),
                PlatformError::Handoff(error),
            ));
            // EHCI's native attach retries the handoff through
            // its own requester and BAR mapping.
            if kind == usb::handoff::Kind::Ehci {
                ehci_hosts.push(address);
                return None;
            }
            usb_handoff_failures.push(kind);
            devices.owners.push(Box::new(function));
            return None;
        }
        _ => {}
    }
    if kind == usb::handoff::Kind::Ehci {
        ehci_hosts.push(address);
        return None;
    }
    // `quirk_usb_handoff_xhci` ends with the Intel port switch,
    // before the xHCI driver binds.
    match usb::handoff::switch_intel_ports(&mut function, inputs.intel_ehci) {
        Ok(Some(switched)) => devices
            .warnings
            .push((address, Warning::IntelPortSwitch(switched))),
        Ok(None) => {}
        Err(error) => devices.platform_failures.push((
            alloc::format!("{address} xHCI port switch"),
            PlatformError::Handoff(error),
        )),
    }
    Some(function)
}

/// What every native attach in one scan shares.
struct NativeInputs<'a> {
    handoff: &'a Handoff,
    firmware: &'static dyn crate::firmware::Provider,
    minimum: &'a Result<Rc<dyn crate::bus::pci::power::Timer>, ServiceError>,
    wifi: Option<&'a WifiInputs<'a>>,
    /// Each PCI bridge the scan has found so far, for ACPI companion lookups.
    bridges: &'a [native::Bridge],
}

/// Attach `function` through the native driver of `kind`, recording the
/// device, or the failure and the owners it keeps, in `devices`.
///
/// # Safety
/// [`NativeServices`]' contract holds for `services`, and nothing else
/// reaches the function.
unsafe fn attach_native(
    kind: native::Kind,
    function: Function,
    services: &dyn NativeServices,
    inputs: &NativeInputs<'_>,
    devices: &mut Devices,
) {
    let address = function.address;
    let attached = match kind {
        // Both hubs need a native notification owner that does
        // not exist yet, so their engines are left as firmware
        // left them.
        native::Kind::Sfh => {
            devices.unsupported.push((
                address,
                function.class,
                "AMD Sensor Fusion Hub: no native notification and mode owner yet",
            ));
            return;
        }
        native::Kind::Ish => {
            devices.unsupported.push((
                    address,
                    function.class,
                    "Intel Integrated Sensor Hub: no native notification owner or firmware DMA address reset yet",
                ));
            return;
        }
        native::Kind::IntelBluetooth => {
            // SAFETY: as for tg3 below.
            match unsafe { native::intel_bluetooth(services, address, inputs.handoff.clock) } {
                Ok(mut transport) => {
                    let initialization = transport.initialize_firmware(
                        inputs.firmware,
                        devices.bluetooth_platform.platform_inputs(),
                    );
                    devices.pcie_bluetooth.push(native::PcieHci {
                        location: alloc::format!("{address}"),
                        controller: native::PcieController::Intel {
                            transport,
                            initialization,
                        },
                    });
                }
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        // The scan defers these until it has found every bridge of the
        // segment (`attach_broadcom_bluetooth`, `attach_usb4_hosts`).
        native::Kind::Bcm4377 | native::Kind::Usb4 => return,
        native::Kind::Hda => {
            let minimum = match inputs.minimum {
                Ok(minimum) => minimum,
                Err(error) => {
                    devices.failures.push((address, Error::Native(*error)));
                    devices.owners.push(Box::new(function));
                    return;
                }
            };
            // SAFETY: as for tg3 below; only conventional
            // controllers, which no firmware agent drives, match.
            match unsafe {
                native::hda::attach(services, address, inputs.handoff.clock, minimum.clone())
            } {
                Ok(host) => devices.hda_hosts.push(host),
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        native::Kind::IntelWifi => {
            let Some(wifi) = inputs.wifi else {
                devices
                    .unsupported
                    .push((address, function.class, NO_WIFI_NETWORK));
                return;
            };
            // SAFETY: as for tg3 below; the companion is this
            // function's own.
            match unsafe {
                intel_wifi(
                    services,
                    address,
                    inputs.handoff.completion,
                    wifi,
                    &devices.regulatory,
                    inputs.bridges,
                )
            } {
                Ok(link) => devices.nics.push(link),
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        native::Kind::MediatekWifi => {
            let Some(wifi) = inputs.wifi else {
                devices
                    .unsupported
                    .push((address, function.class, NO_WIFI_NETWORK));
                return;
            };
            let aspm = match native::aspm_supported(&function, inputs.bridges) {
                Ok(aspm) => aspm,
                Err(error) => {
                    devices.failures.push((address, error.into()));
                    devices.owners.push(Box::new(function));
                    return;
                }
            };
            // SAFETY: as for tg3 below; the companion is this
            // function's own.
            match unsafe {
                mediatek_wifi(
                    services,
                    address,
                    inputs.handoff.completion,
                    wifi,
                    &devices.regulatory,
                    inputs.bridges,
                    aspm,
                )
            } {
                Ok(native::Located { location, link }) => {
                    if link.firmware_domain_unused {
                        devices.warnings.push((
                                address,
                                Warning::Message(
                                    "MT7925 firmware regulatory domain left unused; regulatory.db's country is programmed",
                                ),
                            ));
                    }
                    devices
                        .nics
                        .push(Adapter::MediatekWifi(Box::new(native::Located {
                            location,
                            link: link.station,
                        })));
                }
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        native::Kind::RealtekWifi => {
            let Some(wifi) = inputs.wifi else {
                devices
                    .unsupported
                    .push((address, function.class, NO_WIFI_NETWORK));
                return;
            };
            // SAFETY: as for tg3 below.
            match unsafe {
                realtek_wifi(
                    services,
                    address,
                    inputs.handoff.completion,
                    wifi,
                    &devices.regulatory,
                )
            } {
                Ok(link) => {
                    devices.nics.push(Adapter::RealtekWifi(Box::new(link)));
                }
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        native::Kind::Vmd => {
            let snapshot = crate::firmware::smbios::load(inputs.firmware).ok();
            let platform = snapshot
                .as_ref()
                .map(|snapshot| snapshot.identity)
                .unwrap_or_default();
            // SAFETY: as for tg3 below; the host's hierarchy is
            // stopped with it.
            match unsafe { native::vmd(services, address, inputs.handoff.clock, &platform) } {
                Ok(found) => {
                    for child in &found.others {
                        let location = alloc::format!(
                            "{address} VMD {:02x}:{:02x}.{}",
                            child.address.bus,
                            child.address.device,
                            child.address.function
                        );
                        devices.device_failures.push((
                            location,
                            BlockError::Unsupported(
                                "device behind VMD other than its first NVMe controller",
                            ),
                        ));
                    }
                    if let Some(attached) = found.nvme {
                        if !attached.quirks.is_empty() {
                            devices
                                .warnings
                                .push((address, Warning::NvmeQuirks(attached.quirks)));
                        }
                        if let Some(scan) = attached.sequential {
                            devices.warnings.push((address, Warning::NvmeScan(scan)));
                        }
                        devices
                            .device_failures
                            .extend(attached.failures.into_iter().map(|(namespace, error)| {
                                (alloc::format!("{address} VMD namespace {namespace}"), error)
                            }));
                        devices.disks.extend(
                            attached
                                .namespaces
                                .into_iter()
                                .map(|disk| Box::new(disk) as Box<dyn BlockDevice>),
                        );
                    }
                    devices.owners.push(found.owner);
                }
                Err((error, kept)) => {
                    devices.failures.push((address, error));
                    devices.owners.extend(kept);
                }
            }
            return;
        }
        native::Kind::Tg3 => {
            // SAFETY: the caller grants `services`' contract, and
            // this loop leaves `function` alone once it is claimed.
            let link = unsafe { native::tg3(services, address) };
            link.map(|link| Adapter::Tg3(Box::new(link)))
        }
        native::Kind::Aqtion => {
            // SAFETY: as for tg3.
            let link = unsafe { native::aqtion(services, address) };
            link.map(|link| Adapter::Aqtion(Box::new(link)))
        }
    };
    match attached {
        Ok(adapter) => devices.nics.push(adapter),
        Err((error, kept)) => {
            devices.failures.push((address, error));
            devices.owners.extend(kept);
        }
    }
}

/// Enumerate every mapped ECAM window, and whether every window was
/// scanned. Unmapped windows and failed enumerations go to `devices`.
fn scan(handoff: &Handoff, devices: &mut Devices) -> (Vec<Vec<Function>>, bool) {
    let mut pci_scan_complete = true;
    let mut scanned = Vec::new();
    for window in &handoff.ecam {
        if !handoff.is_mapped(&window.range()) {
            devices.unmapped.push(*window);
            pci_scan_complete = false;
            continue;
        }
        // SAFETY: mapped as checked; the caller owns the machine.
        match unsafe { Function::enumerate(window) } {
            Ok(found) => scanned.push(found),
            Err(failure) => {
                pci_scan_complete = false;
                devices
                    .failures
                    .push((failure.address(), Error::Enumeration(failure.error)));
                devices.owners.push(Box::new(failure));
            }
        }
    }
    (scanned, pci_scan_complete)
}

/// Attach each EHCI host the scan found, once every UHCI/OHCI companion
/// has left firmware, with the USB devices on it.
fn attach_ehci_hosts(
    ehci_hosts: Vec<Address>,
    companions_released: bool,
    services: Option<&dyn NativeServices>,
    handoff: &Handoff,
    devices: &mut Devices,
) {
    for address in ehci_hosts {
        let attached = match services {
            Some(services) if companions_released => {
                // SAFETY: the completed scan has run the UHCI/OHCI handoff
                // for every discovered companion. The native attach owns
                // EHCI's BIOS handoff through its original requester and BAR.
                unsafe { native::ehci(services, address, handoff.clock) }
            }
            Some(_) => Err((
                Error::Native(ServiceError::Refused("EHCI companion handoff incomplete")),
                None,
            )),
            None => Err((Error::Native(ServiceError::Unavailable), None)),
        };
        let controller = match attached {
            Ok(attached) => attached,
            Err((error, kept)) => {
                devices.failures.push((address, error));
                devices.owners.extend(kept);
                continue;
            }
        };
        let found = usb::ehci::host::enumerate(
            &controller,
            &alloc::format!("{address} EHCI"),
            handoff.clock,
        );
        devices.disks.extend(
            found
                .disks
                .into_iter()
                .map(|disk| Box::new(disk) as Box<dyn BlockDevice>),
        );
        devices.usb_storage.extend(found.storage);
        devices.usb_failures.extend(found.failures);
        devices.usb_unused.extend(found.unused);
        devices.inputs.extend(
            found
                .inputs
                .into_iter()
                .map(|input| InputDevice::Ehci(Box::new(input))),
        );
        devices.ehci_hosts.push(controller);
    }
}

/// Attach each USB4 host left to the firmware connection manager, once the
/// scan has found every bridge of its segment.
fn attach_usb4_hosts(
    usb4_hosts: Vec<Address>,
    services: Option<&dyn NativeServices>,
    bridges: &[native::Bridge],
    handoff: &Handoff,
    devices: &mut Devices,
) {
    for address in usb4_hosts {
        let Some(services) = services else {
            devices
                .failures
                .push((address, Error::Native(ServiceError::Unavailable)));
            continue;
        };
        // SAFETY: as for tg3; the scan found every bridge of the segment.
        match unsafe { native::usb4::attach(services, address, bridges, handoff.clock, false) } {
            Ok(host) => devices.usb4_hosts.push(host),
            Err((error, kept)) => {
                devices.failures.push((address, error));
                devices.owners.extend(kept);
            }
        }
    }
}

/// Attach each Broadcom PCIe Bluetooth function, once the scan has found
/// every bridge of its segment, and load its firmware.
fn attach_broadcom_bluetooth(
    broadcom_bluetooth: Vec<Address>,
    services: Option<&dyn NativeServices>,
    bridges: &[native::Bridge],
    firmware: &'static dyn crate::firmware::Provider,
    handoff: &Handoff,
    devices: &mut Devices,
) {
    for address in broadcom_bluetooth {
        let Some(services) = services else {
            devices
                .failures
                .push((address, Error::Native(ServiceError::Unavailable)));
            continue;
        };
        // SAFETY: as for tg3; the scan found every bridge of the segment.
        match unsafe { bcm4377(services, address, bridges, firmware, handoff.clock) } {
            Ok(mut transport) => {
                let initialization = transport.initialize(firmware);
                devices.pcie_bluetooth.push(native::PcieHci {
                    location: alloc::format!("{address}"),
                    controller: native::PcieController::Broadcom {
                        transport,
                        initialization,
                    },
                });
            }
            Err((error, kept)) => {
                devices.failures.push((address, error));
                devices.owners.extend(kept);
            }
        }
    }
}

/// The devices the ACPI namespace lists: the embedded controller first,
/// whose fields other devices' methods may read, then the i801 SMBus hosts,
/// the i8042, I2C, power and Type-C. Without an ACPI session each SMBus host
/// is reported unavailable.
#[cfg(feature = "native-acpi")]
fn attach_platform(
    services: Option<&dyn NativeServices>,
    mut smbus_functions: Vec<Address>,
    bridges: &[native::Bridge],
    handoff: &Handoff,
    devices: &mut Devices,
) {
    if let Some((services, session)) =
        services.and_then(|services| services.acpi().map(|session| (services, session)))
    {
        // Before any other device's methods, which may read EC fields.
        let ec = match services.embedded_controller() {
            Ok(Some(installed)) => {
                devices.owners.push(Box::new(installed.clone()));
                Some(installed)
            }
            Ok(None) => power::attach_ec(session, devices),
            // The mount installs no EC while ACPI hardware is not active,
            // and reports its own failed install. Say so once, and only
            // when the namespace lists an EC.
            Err(ServiceError::Unavailable) => {
                if session
                    .devices(services::ids::EMBEDDED_CONTROLLER[0])
                    .is_ok_and(|found| !found.is_empty())
                {
                    devices.platform_unsupported.push((
                        String::from("embedded controller"),
                        "not attached: ACPI hardware is not active, or the mount's install failed",
                    ));
                }
                None
            }
            Err(error) => {
                devices.platform_failures.push((
                    String::from("embedded controller"),
                    PlatformError::Native(error),
                ));
                None
            }
        };
        for address in core::mem::take(&mut smbus_functions) {
            // SAFETY: the mount grants each i801's I/O window to probe.
            match unsafe { smbus::attach(services, session, address, bridges, handoff.clock) } {
                Ok(host) => devices.smbus_hosts.push(host),
                Err(error) => devices
                    .platform_failures
                    .push((alloc::format!("{address} SMBus"), error)),
            }
        }
        // Linux never ties the i8042 to the USB handoffs: a host that
        // firmware kept is reported on its own, and the controller's
        // self-test finds ports that USB legacy emulation still traps.
        let smbus_hosts = devices.smbus_hosts.clone();
        platform::attach_i8042(session, handoff.clock, &smbus_hosts, devices);
        i2c::attach(services, session, handoff.clock, devices);
        power::attach_power(session, devices);
        typec::attach(services, session, ec.as_ref(), handoff.clock, devices);
    }
    for address in smbus_functions {
        devices.platform_failures.push((
            alloc::format!("{address} SMBus"),
            PlatformError::Native(ServiceError::Unavailable),
        ));
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// The boot of issue 18: the mount refused both HD Audio controllers
    /// before their driver ran, and they are filed as audio, not with the
    /// devices that may hold a disk.
    #[test]
    fn a_refusal_common_to_every_driver_is_filed_by_the_functions_class() {
        use crate::bus::pci::requester::{ClaimError, FunctionRole};
        let audio = identity(0, 3, 0x8086, 0xa2f0, 0x04_03_00);
        let unknown = identity(1, 0, 0x1234, 0x5678, 0xff_00_00);
        let devices = Devices {
            classes: alloc::vec![
                (audio.address, audio.class),
                (unknown.address, unknown.class)
            ],
            ..Devices::default()
        };
        let refused = Error::Native(ServiceError::Claim(ClaimError::UnsupportedRole(
            FunctionRole::Unverified,
        )));
        assert_eq!(devices.subsystem(audio.address, &refused), "audio");
        assert_eq!(devices.subsystem(unknown.address, &refused), "device");
        // An error that names its driver keeps that driver's subsystem.
        let vmd = Error::Vmd(crate::bus::vmd::Error::UnsupportedDevice);
        assert_eq!(devices.subsystem(audio.address, &vmd), "storage");
    }

    /// The boot of issue 18: an audio controller's failure stays out of the
    /// network account, and each adapter's line reads as plain words.
    #[test]
    fn the_network_account_leaves_audio_out_and_reads_plainly() {
        use crate::bus::pci::requester::{ClaimError, FunctionRole};
        let audio = identity(0, 3, 0x8086, 0xa2f0, 0x04_03_00);
        let ethernet = identity(0, 6, 0x8086, 0x15b8, 0x02_00_00);
        let wifi = identity(4, 0, 0x8086, 0x2725, 0x02_80_00);
        let mut devices = Devices {
            network_functions: alloc::vec![ethernet, wifi],
            ..Devices::default()
        };
        let phy = crate::net::e1000e::pch::Error::from(crate::net::e1000e::pch::Fault::MdicTimeout);
        devices.failures = alloc::vec![
            (
                audio.address,
                Error::Native(ServiceError::Claim(ClaimError::UnsupportedRole(
                    FunctionRole::Unverified
                )))
            ),
            (
                ethernet.address,
                Error::Pch(pch::AttachError::Controller(
                    crate::net::e1000e::pch::cold::Error::Phy(phy)
                ))
            ),
            (
                wifi.address,
                Error::IntelWifi(net::wifi::intel::attach::AttachError::NoPinnedFirmware {
                    family: net::wifi::intel::firmware::Family::Ax210,
                    name: "iwlwifi-ty-a0-gf-a0-89.ucode",
                })
            ),
        ];
        let lines = devices.network_account();
        assert_eq!(lines.len(), 2, "{lines:#?}");
        assert!(lines[0].starts_with("0000:00:00.6 8086:15b8 Ethernet: "));
        assert!(lines[1].starts_with("0000:04:00.0 8086:2725 Wi-Fi: "));
        // No driver's name leads a reason, wired or Wi-Fi alike.
        for line in &lines {
            let reason = line.split(": ").nth(1).unwrap();
            assert!(
                !["e1000e", "iwlwifi", "tg3", "atlantic", "r8169"]
                    .iter()
                    .any(|driver| reason.starts_with(driver)),
                "{line}"
            );
        }
        for line in &lines {
            assert!(
                !line.contains('{') && !line.contains("Unverified") && !line.contains("Error"),
                "{line}"
            );
        }
    }

    /// The cable advice is for a wired port only, never a Wi-Fi link.
    #[test]
    fn the_cable_advice_is_for_a_wired_port_only() {
        assert_eq!(
            no_link("0000:00:1f.6", false),
            "0000:00:1f.6: no link. Check that a network cable is plugged in and that the other \
             end is on."
        );
        assert_eq!(
            no_link("0000:04:00.0", true),
            "0000:04:00.0: not connected to the Wi-Fi network."
        );
    }

    /// A function whose device never answers spends its whole budget and
    /// is recorded abandoned, with the wait it stopped at; the next
    /// function then starts with a fresh budget and answers normally.
    #[test]
    fn a_dead_function_is_abandoned_and_the_next_one_still_starts() {
        let _clock = crate::test_clock::virtualize();
        let clock = Clock {
            ticks_per_microsecond: 1,
        };
        let at = |bus| Address {
            segment: 0,
            bus,
            device: 0,
            function: 0,
        };
        let mut abandoned = Vec::new();
        let started = Clock::ticks();
        {
            let _start = DeviceStart::begin(at(0x0b), clock, &mut abandoned);
            // A driver step whose own bound is a minute: the budget ends it.
            assert!(!clock.wait(60_000_000, || false));
        }
        assert!(Clock::ticks() - started < DEVICE_START_US + 1_000);
        {
            let _start = DeviceStart::begin(at(0x0c), clock, &mut abandoned);
            assert!(!clock.deadline(1_000_000).passed());
            assert!(clock.wait(1_000_000, || true));
        }
        assert_eq!(abandoned.len(), 1);
        let (address, expiry) = abandoned[0];
        assert_eq!(address, at(0x0b));
        assert_eq!(expiry.budget_us, DEVICE_START_US);
        assert!(expiry.at.unwrap().file().ends_with("probe.rs"));
    }
    use crate::firmware::{
        self, Catalog, HandoffProvider, Provenance, Provider, Requirement, Resource, Target,
    };
    use alloc::{format, string::ToString};
    use sha2::{Digest, Sha256};

    fn identity(
        bus: u8,
        function: u8,
        vendor: u16,
        device: u16,
        class: u32,
    ) -> crate::bus::pci::Identity {
        crate::bus::pci::Identity {
            address: Address {
                segment: 0,
                bus,
                device: 0,
                function,
            },
            vendor,
            device,
            class: [(class >> 16) as u8, (class >> 8) as u8, class as u8],
        }
    }

    /// Every network-class function a boot found and did not start has its
    /// line, whatever stopped it, and a boot with none says so.
    #[test]
    fn the_network_account_names_each_adapter_that_did_not_start() {
        let none = Devices::default();
        assert_eq!(
            none.network_account(),
            ["no network-class device answered on PCI, and no USB network adapter was found"]
        );
        let r8168 = identity(3, 0, 0x10ec, 0x8168, 0x02_00_00);
        let unknown = identity(4, 0, 0x8086, 0x1234, 0x02_00_00);
        let wifi = identity(5, 0, 0x8086, 0xa370, 0x02_80_00);
        let silent = identity(6, 0, 0x14e4, 0x1686, 0x02_00_00);
        let mut devices = Devices {
            network_functions: alloc::vec![r8168, unknown, wifi, silent],
            ..Devices::default()
        };
        devices.failures.push((
            r8168.address,
            Error::Realtek(realtek::Error::Firmware(firmware::Error {
                name: firmware::ErrorName::Available("rtl_nic/rtl8168g-2.fw".into()),
                kind: firmware::ErrorKind::Missing,
            })),
        ));
        devices.unsupported.push((
            unknown.address,
            unknown.class,
            unsupported(unknown.class).unwrap(),
        ));
        devices.unsupported.push((
            wifi.address,
            wifi.class,
            "Intel Wireless-AC 9000 series (Linux iwlwifi): this generation has no driver here",
        ));
        assert_eq!(
            devices.network_account(),
            [
                "0000:03:00.0 10ec:8168 Ethernet: firmware \"rtl_nic/rtl8168g-2.fw\": not supplied",
                "0000:04:00.0 8086:1234 Ethernet: Ethernet controller whose ID no driver here takes",
                "0000:05:00.0 8086:a370 Wi-Fi: Intel Wireless-AC 9000 series (Linux iwlwifi): this \
                 generation has no driver here",
                "0000:06:00.0 14e4:1686 Ethernet: no driver started it",
            ]
        );
        // A start past its budget is named with what it ended in.
        devices.abandoned.push((
            r8168.address,
            budget::Expiry {
                budget_us: DEVICE_START_US,
                at: None,
            },
        ));
        assert!(
            devices.network_account()[0].ends_with("(its start took over 30 s and was given up)")
        );
        // A failure that is the budget's own ending says it once.
        devices.failures[0].1 = Error::Device(BlockError::PciPower(
            crate::bus::pci::power::Error::StartBudget,
        ));
        assert_eq!(
            devices.network_account()[0],
            "0000:03:00.0 10ec:8168 Ethernet: its start took over 30 s and was given up"
        );
        // A started Broadcom PCIe Bluetooth function shares Wi-Fi's class
        // and is no network adapter.
        let bluetooth = identity(7, 0, 0x14e4, 0x5fa0, 0x02_80_00);
        let devices = Devices {
            network_functions: alloc::vec![bluetooth],
            ..Devices::default()
        };
        let at = alloc::format!("{}", bluetooth.address);
        let named = |lines: Vec<String>| lines.iter().any(|line| line.contains("14e4:5fa0"));
        assert!(!named(devices.network_lines(&[at.as_str()])));
        assert!(named(devices.network_lines(&[])));
    }

    /// Each wired driver's failure, as the network account shows it, reads
    /// as words: no Rust debug text reaches the screen.
    #[test]
    fn wired_driver_failures_read_as_words() {
        use crate::bus::dma::memory::MemoryError;
        let failures = [
            Error::Tg3(net::tg3::Error::MdioTimeout),
            Error::Tg3(net::tg3::Error::Memory(MemoryError::OutOfMemory)),
            Error::Aqtion(net::aqtion::native::Error::Device(
                net::aqtion::Error::Firmware { status: 0x10 },
            )),
            Error::Ehci(usb::ehci::Error::Timeout),
            Error::Device(BlockError::Irq(crate::bus::irq::IrqError::Exhausted)),
        ];
        for error in failures {
            let text = alloc::format!("{error}");
            assert!(
                !text.contains('{') && !text.contains('(') || text.contains("(status"),
                "{text}"
            );
            assert!(!text.contains("Error") && !text.contains("::"), "{text}");
        }
    }

    #[test]
    fn plan_names_the_driver_of_common_desktop_functions() {
        let gpu = identity(1, 0, 0x10de, 0x2882, 0x03_00_00);
        let displays = [gpu.address];
        for (function, expected) in [
            (identity(0x0b, 0, 0x1022, 0x149c, 0x0c_03_30), "xHCI USB"),
            (identity(0x0c, 0, 0x1022, 0x7901, 0x01_06_01), "AHCI SATA"),
            (identity(0x04, 0, 0x144d, 0xa808, 0x01_08_02), "NVMe"),
            (gpu, "GPU"),
            (identity(0x00, 1, 0x1022, 0x1483, 0x06_04_00), "PCI bridge"),
            (identity(0x00, 0, 0x1022, 0x1480, 0x06_00_00), "not driven"),
            (
                identity(0x05, 0, 0x1106, 0x0571, 0x01_01_80),
                "unsupported: IDE storage controller",
            ),
        ] {
            assert_eq!(
                plan(&function, &displays),
                expected,
                "{:?}",
                function.address
            );
        }
        // The GPU's own HDMI audio function, beside its function 0.
        let hdmi = identity(1, 1, 0x1234, 0x0001, 0x04_03_00);
        assert!(plan(&hdmi, &displays).starts_with("unsupported: HDMI/DP audio"));
        assert!(plan(&hdmi, &[]).starts_with("unsupported: HD Audio controller"));
    }

    #[test]
    fn plan_names_a_driver_exactly_for_the_functions_claimed() {
        let functions = [
            identity(0x0b, 0, 0x1022, 0x149c, 0x0c_03_30),
            identity(0x0b, 1, 0x1022, 0x43d5, 0x0c_03_30),
            identity(0x0c, 0, 0x1022, 0x7901, 0x01_06_01),
            identity(0x04, 0, 0x144d, 0xa808, 0x01_08_02),
            identity(0x0d, 0, 0x10ec, 0x8168, 0x02_00_00),
            identity(0x0e, 0, 0x8086, 0x15f3, 0x02_00_00),
            identity(0x0f, 0, 0x1022, 0x1487, 0x04_03_00),
            identity(0x10, 0, 0x1af4, 0x1042, 0x01_00_00),
            identity(0x11, 0, 0x8086, 0x1e2d, 0x0c_03_20),
            identity(0x00, 0, 0x1022, 0x1480, 0x06_00_00),
            identity(0x12, 0, 0x1234, 0x5678, 0x0c_03_10),
        ];
        for function in functions {
            let named = plan(&function, &[]);
            // A USB host taken from firmware has its registers mapped too.
            let driven = !named.starts_with("unsupported") && named != "not driven";
            assert_eq!(
                driven,
                claims_identity(&function),
                "{:?} {named}",
                function.address
            );
        }
    }

    #[test]
    fn ehci_uses_native_preparation_only_after_companion_handoff() {
        for companion_refused in [false, true] {
            let mut ecam = alloc::vec![u32::MAX; (1 << 20) / 4];
            let mut function = |device: usize, class: u32| {
                let start = device * (1 << 15) / 4;
                let page = &mut ecam[start..start + 4096 / 4];
                page.fill(0);
                page[0] = 0x1234_4321;
                page[2] = class;
            };
            function(0, 0x0c03_2000);
            if companion_refused {
                // The companion's BAR is absent or unmapped. The scan
                // discovers it after EHCI, and it cannot reach register I/O.
                function(1, 0x0c03_1000);
            }
            let window = EcamWindow {
                base: ecam.as_mut_ptr() as u64,
                segment: 0,
                start_bus: 0,
                end_bus: 0,
            };
            let handoff = Handoff {
                ecam: alloc::vec![window],
                mapped: alloc::vec![window.range()],
                dma: 0..0,
                clock: Clock {
                    ticks_per_microsecond: 1,
                },
                firmware: None,
                completion: None,
            };
            let services = services::tests::Refusing::minimum_only();
            // SAFETY: the empty region can allocate no DMA pages. Only the
            // synthetic ECAM buffer is mapped, and native claims all refuse.
            let mut dma = unsafe { crate::bus::dma::RegionAllocator::new(0..0) };
            // SAFETY: ECAM is the live fixture above. Its legacy BARs are
            // unmapped and the services refuse all native device access.
            let devices = unsafe {
                probe_with_provider(
                    &handoff,
                    PlatformCaptures::default(),
                    &mut dma,
                    handoff_provider(&handoff),
                    Some(&services),
                    None,
                )
            };
            let address = Address {
                segment: 0,
                bus: 0,
                device: 0,
                function: 0,
            };
            assert!(devices.ehci_hosts.is_empty());
            assert_eq!(ecam[1], 0);
            assert_eq!(ecam[4], 0);
            // The early handoff ran first and found the BAR unmapped; the
            // native attach still tries its own mapping.
            assert!(
                matches!(
                    devices.platform_failures.as_slice(),
                    [(name, PlatformError::Handoff(usb::handoff::Error::Unmapped)), ..]
                        if name.ends_with("EHCI handoff")
                ),
                "{:?}",
                devices.platform_failures
            );
            if companion_refused {
                assert!(services.preparations.borrow().is_empty());
                assert!(matches!(
                    devices.failures.as_slice(),
                    [(found, Error::Native(ServiceError::Refused("EHCI companion handoff incomplete")))]
                        if *found == address
                ));
            } else {
                assert_eq!(*services.preparations.borrow(), [address]);
                assert!(matches!(
                    devices.failures.as_slice(),
                    [(found, Error::Native(ServiceError::Unavailable))] if *found == address
                ));
            }
        }
    }

    /// A claimed function whose start recorded nothing is a named failure;
    /// one that recorded anything is left to that record.
    #[test]
    fn a_claimed_start_that_records_nothing_is_a_failure() {
        let address = |device| Address {
            segment: 0,
            bus: 7,
            device,
            function: 0,
        };
        let mut failures = Vec::new();
        let mut claimed = Claimed::default();
        claimed.settle(3, &mut failures);
        assert!(failures.is_empty());
        claimed.begin(address(0), 3);
        claimed.settle(4, &mut failures);
        assert!(failures.is_empty());
        claimed.begin(address(1), 4);
        claimed.settle(4, &mut failures);
        assert!(matches!(
            failures.as_slice(),
            [(found, Error::Unrecorded)] if *found == address(1)
        ));
        assert_eq!(
            failures[0].1.to_string(),
            "its driver started and recorded no device and no failure"
        );
        // Settled once only.
        claimed.settle(4, &mut failures);
        assert_eq!(failures.len(), 1);
        // An NVMe controller with no active namespace started and found no
        // disk: no failure, as Linux reports none.
        claimed.begin(address(2), 4);
        claimed.started();
        claimed.settle(4, &mut failures);
        assert_eq!(failures.len(), 1);
    }

    /// A USB4 host whose software connection manager came up before the
    /// scan was recorded then; the scan neither starts nor claims it again.
    #[test]
    fn a_usb4_host_brought_up_before_the_scan_is_not_claimed_again() {
        // An Intel Thunderbolt 4 host interface, as `nhi_ids` lists it.
        let host = identity(0x00, 0, 0x8086, 0x9a1b, 0x0c_03_40);
        assert_eq!(native::Kind::of_identity(&host), Some(native::Kind::Usb4));
        assert!(scan_starts(&host, &[]));
        assert!(!scan_starts(&host, &[host.address]));
        // Every other claimed function is started by the scan.
        let nvme = identity(0x04, 0, 0x144d, 0xa808, 0x01_08_02);
        assert!(scan_starts(&nvme, &[nvme.address]));
        assert!(!scan_starts(
            &identity(0x05, 0, 0x1234, 0x5678, 0x08_80_00),
            &[]
        ));
    }

    /// The observer's first function starts before every other, its end is
    /// told before any other starts, and every start is told once, after
    /// its name and before its driver.
    #[test]
    fn the_observers_first_function_starts_first_and_each_start_is_told() {
        #[derive(Default)]
        struct Told {
            first: Option<Address>,
            events: Vec<(&'static str, u8)>,
        }
        impl Observer for Told {
            fn first(&self) -> Option<Address> {
                self.first
            }
            fn first_started(&mut self, _: &mut Devices) {
                self.events.push(("first started", 0xff));
            }
            fn starting(&mut self, function: &crate::bus::pci::Identity) -> Start {
                self.events.push(("starting", function.address.device));
                if function.address.device == 1 {
                    Start::Skip("skipped here")
                } else {
                    Start::Run
                }
            }
        }
        let mut ecam = alloc::vec![u32::MAX; (1 << 20) / 4];
        for (device, class) in [(0, 0x0108_0200), (1, 0x0200_0000), (2, 0x0c03_3000)] {
            let start = device * (1 << 15) / 4;
            let page = &mut ecam[start..start + 4096 / 4];
            page.fill(0);
            page[0] = 0x5678_1234;
            page[2] = class;
        }
        let window = EcamWindow {
            base: ecam.as_mut_ptr() as u64,
            segment: 0,
            start_bus: 0,
            end_bus: 0,
        };
        let handoff = Handoff {
            ecam: alloc::vec![window],
            mapped: alloc::vec![window.range()],
            dma: 0..0,
            clock: Clock {
                ticks_per_microsecond: 1,
            },
            firmware: None,
            completion: None,
        };
        let address = |device| Address {
            segment: 0,
            bus: 0,
            device,
            function: 0,
        };
        for (first, expected) in [
            (
                Some(address(2)),
                alloc::vec![
                    ("starting", 2),
                    ("first started", 0xff),
                    ("starting", 0),
                    ("starting", 1)
                ],
            ),
            // A first function the scan did not find changes nothing.
            (
                Some(address(9)),
                alloc::vec![("starting", 0), ("starting", 1), ("starting", 2)],
            ),
            (
                None,
                alloc::vec![("starting", 0), ("starting", 1), ("starting", 2)],
            ),
        ] {
            let services = services::tests::Refusing::minimum_only();
            // SAFETY: as in the EHCI fixture: no DMA pages, only the
            // synthetic ECAM mapped, and every native grant refused.
            let mut dma = unsafe { crate::bus::dma::RegionAllocator::new(0..0) };
            let mut told = Told {
                first,
                ..Told::default()
            };
            // SAFETY: as above.
            let result = unsafe {
                probe_observed(
                    &handoff,
                    PlatformCaptures::default(),
                    &mut dma,
                    handoff_provider(&handoff),
                    Some(&services),
                    None,
                    &mut told,
                )
            };
            assert_eq!(told.events, expected, "{first:?}");
            // A skipped function is left untouched and recorded as not
            // driven, and the claim on it is settled by that record.
            let devices = result;
            assert!(
                devices
                    .unsupported
                    .iter()
                    .any(|(at, _, why)| *at == address(1) && *why == "skipped here"),
                "{first:?}"
            );
            assert!(
                devices.failures.iter().all(|(at, _)| *at != address(1)),
                "{first:?}"
            );
        }
    }

    /// A bridge found with bus mastering off forwards nothing upstream from
    /// the functions below it; the scan turns it on, as Linux's
    /// `pci_enable_bridge` does, and says so. One that forwards is left.
    #[test]
    fn a_bridge_with_bus_mastering_off_is_reopened_for_the_functions_below_it() {
        /// Whether the bridge forwarded as each function's start was told.
        struct Forwarding {
            command: *const u32,
            first: Address,
            seen: Vec<bool>,
        }
        impl Observer for Forwarding {
            fn first(&self) -> Option<Address> {
                Some(self.first)
            }
            fn starting(&mut self, _: &crate::bus::pci::Identity) -> Start {
                // SAFETY: the test's own ECAM words, alive for the probe.
                self.seen
                    .push(unsafe { self.command.read_volatile() } & 4 != 0);
                Start::Skip("left alone")
            }
        }
        let mut ecam = alloc::vec![u32::MAX; (1 << 20) / 4];
        for (device, command, class, header) in [
            (0, 0x0002, 0x0604_0000, 0x0001_0000),
            (1, 0x0006, 0x0604_0000, 0x0001_0000),
            (2, 0x0002, 0x0c03_3000, 0),
        ] {
            let start = device * (1 << 15) / 4;
            let page = &mut ecam[start..start + 4096 / 4];
            page.fill(0);
            page[0] = 0x5678_1234;
            page[1] = command;
            page[2] = class;
            page[3] = header;
        }
        let window = EcamWindow {
            base: ecam.as_mut_ptr() as u64,
            segment: 0,
            start_bus: 0,
            end_bus: 0,
        };
        let handoff = Handoff {
            ecam: alloc::vec![window],
            mapped: alloc::vec![window.range()],
            dma: 0..0,
            clock: Clock {
                ticks_per_microsecond: 1,
            },
            firmware: None,
            completion: None,
        };
        let address = |device| Address {
            segment: 0,
            bus: 0,
            device,
            function: 0,
        };
        let services = services::tests::Refusing::minimum_only();
        // The boot's own controller starts first, ahead of the scan's order:
        // the bridge above it forwards by then.
        let mut observer = Forwarding {
            command: &raw const ecam[1],
            first: address(2),
            seen: Vec::new(),
        };
        // SAFETY: no DMA pages, only the synthetic ECAM mapped, and every
        // native grant refused.
        let mut dma = unsafe { crate::bus::dma::RegionAllocator::new(0..0) };
        // SAFETY: as above.
        let devices = unsafe {
            probe_observed(
                &handoff,
                PlatformCaptures::default(),
                &mut dma,
                handoff_provider(&handoff),
                Some(&services),
                None,
                &mut observer,
            )
        };
        assert_eq!(devices.bridges_reopened, [address(0)]);
        assert!(devices.failures.is_empty());
        assert!(!observer.seen.is_empty() && observer.seen.iter().all(|&forwarded| forwarded));
        assert_eq!(ecam[1] & 0xffff, 0x0006);
        assert_eq!(ecam[(1 << 15) / 4 + 1] & 0xffff, 0x0006);
    }

    /// Every function a driver here claims ends its probe with a record:
    /// storage, network, audio, USB and native starts that all refuse.
    #[test]
    fn every_claimed_function_ends_its_probe_recorded() {
        let mut ecam = alloc::vec![u32::MAX; (1 << 20) / 4];
        let functions = [
            (0x144d_a808, 0x0108_0200),
            (0x10ec_8125, 0x0200_0000),
            (0x8086_a352, 0x0106_0100),
            (0x1022_15b6, 0x0c03_3000),
            (0x8086_a348, 0x0403_0000),
            (0x14e4_1686, 0x0200_0000),
            (0x1af4_1042, 0x0100_0000),
            (0x1234_5678, 0x0880_0000),
        ];
        for (device, (id, class)) in functions.iter().enumerate() {
            let start = device * (1 << 15) / 4;
            let page = &mut ecam[start..start + 4096 / 4];
            page.fill(0);
            // Vendor in the low half, device in the high half.
            page[0] = (id & 0xffff) << 16 | id >> 16;
            page[2] = *class;
        }
        let window = EcamWindow {
            base: ecam.as_mut_ptr() as u64,
            segment: 0,
            start_bus: 0,
            end_bus: 0,
        };
        let handoff = Handoff {
            ecam: alloc::vec![window],
            mapped: alloc::vec![window.range()],
            dma: 0..0,
            clock: Clock {
                ticks_per_microsecond: 1,
            },
            firmware: None,
            completion: None,
        };
        let services = services::tests::Refusing::minimum_only();
        // SAFETY: as in the EHCI fixture above: no DMA pages, only the
        // synthetic ECAM mapped, and every native grant refused.
        let mut dma = unsafe { crate::bus::dma::RegionAllocator::new(0..0) };
        // SAFETY: as above.
        let devices = unsafe {
            probe_with_provider(
                &handoff,
                PlatformCaptures::default(),
                &mut dma,
                handoff_provider(&handoff),
                Some(&services),
                None,
            )
        };
        for (device, (id, class)) in functions.iter().enumerate() {
            let address = Address {
                segment: 0,
                bus: 0,
                device: device as u8,
                function: 0,
            };
            let named = format!("{address} ");
            let failed = devices.failures.iter().any(|(at, _)| *at == address)
                || devices
                    .platform_failures
                    .iter()
                    .any(|(at, _)| at.starts_with(&named));
            let unsupported = devices.unsupported.iter().any(|(at, ..)| *at == address);
            if *class == 0x0880_0000 {
                assert!(!failed && !unsupported, "{id:08x}");
            } else {
                assert!(failed || unsupported, "{id:08x} {:?}", devices.failures);
            }
        }
        assert!(
            !devices
                .failures
                .iter()
                .any(|(_, error)| matches!(error, Error::Unrecorded)),
            "{:?}",
            devices.failures
        );
    }

    /// The chooser refuses a disk whose controller no driver here takes,
    /// naming RAID mode, and offers every disk a driver reaches.
    #[test]
    fn disks_behind_undriven_storage_controllers_are_refused_before_the_choice() {
        let identity = |vendor, device, class| crate::bus::pci::Identity {
            address: Address {
                segment: 0,
                bus: 0,
                device: 0x17,
                function: 0,
            },
            vendor,
            device,
            class,
        };
        // A RAID-mode controller outside the AHCI table, and an IDE one.
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x1234, [0x01, 0x04, 0x00])),
            Some("Its controller is in RAID mode: set SATA to AHCI in firmware setup")
        );
        assert_eq!(
            disk_refusal(&identity(0x1b4b, 0x9230, [0x01, 0x04, 0x00])),
            Some("Its controller is in RAID mode: set SATA to AHCI in firmware setup")
        );
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x7ae2, [0x01, 0x01, 0x8a])),
            Some("Stela has no driver for its controller")
        );
        // AHCI, Intel RAID-mode SATA the AHCI table lists, NVMe, VMD.
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x7ae2, [0x01, 0x06, 0x01])),
            None
        );
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x2822, [0x01, 0x04, 0x00])),
            None
        );
        assert_eq!(
            disk_refusal(&identity(0x144d, 0xa808, [0x01, 0x08, 0x02])),
            None
        );
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x467f, [0x01, 0x04, 0x00])),
            None
        );
        // xHCI: USB disks are refused by their own checks, not here.
        assert_eq!(
            disk_refusal(&identity(0x8086, 0x7ae0, [0x0c, 0x03, 0x30])),
            None
        );
    }

    #[test]
    fn unclaimed_host_classes_are_reported_and_others_ignored() {
        assert_eq!(
            unsupported([0x02, 0x80, 0x00]),
            Some("Wi-Fi controller other than Intel's, MediaTek's MT7925 and Realtek's RTL8822C")
        );
        assert_eq!(
            unsupported([0x01, 0x04, 0x00]),
            Some("RAID controller, not supported; set the SATA mode to AHCI in firmware setup")
        );
        assert_eq!(
            unsupported([0x0c, 0x03, 0x10]),
            Some("UHCI or OHCI USB host controller (taken from firmware, not driven)")
        );
        assert_eq!(unsupported([0x0c, 0x03, 0x20]), None); // EHCI: native
        assert_eq!(
            unsupported([0x08, 0x05, 0x02]),
            Some("SD host controller without standard SDHCI interface")
        );
        assert!(unsupported([0x0c, 0x05, 0x00]).is_some_and(|why| why.contains("i801")));
        assert_eq!(unsupported([0x06, 0x04, 0x00]), None); // PCI bridge
        assert_eq!(unsupported([0x03, 0x00, 0x00]), None); // display: GPU inventory
    }

    #[test]
    fn hd_audio_beside_a_display_function_is_reported_as_display_audio() {
        let at = |bus, device, function| Address {
            segment: 0,
            bus,
            device,
            function,
        };
        let displays = [at(0x0d, 0, 0)];
        let audio = [0x04, 0x03, 0x00];
        assert_eq!(
            unclaimed(audio, at(0x0d, 0, 1), &displays),
            Some(DISPLAY_AUDIO)
        );
        // A chipset or board HD Audio controller keeps the general reason.
        for address in [at(0x0d, 1, 1), at(0x0e, 0, 1), at(0x0d, 0, 0)] {
            assert_eq!(
                unclaimed(audio, address, &displays),
                Some("HD Audio controller outside the supported identities")
            );
        }
        // Other classes beside a GPU are unaffected.
        assert_eq!(
            unclaimed([0x0c, 0x03, 0x10], at(0x0d, 0, 3), &displays),
            unsupported([0x0c, 0x03, 0x10])
        );
    }

    #[test]
    fn wifi_without_regulatory_data_never_claims_the_adapter() {
        let services = services::tests::Refusing::default();
        let handoff = Handoff {
            ecam: Vec::new(),
            mapped: Vec::new(),
            dma: 0..0,
            clock: crate::Clock {
                ticks_per_microsecond: 1,
            },
            firmware: None,
            completion: None,
        };
        let firmware = handoff_provider(&handoff);
        let network = Network {
            ssid: b"test".to_vec(),
            secret: crate::net::wifi::network::Secret::parse(b"test passphrase").unwrap(),
            country: None,
        };
        let address = Address {
            segment: 0,
            bus: 1,
            device: 0,
            function: 0,
        };
        // SAFETY: the refusing services cannot grant hardware access.
        let result = unsafe {
            intel_wifi(
                &services,
                address,
                None,
                &WifiInputs {
                    clock: handoff.clock,
                    observations: Default::default(),
                    firmware,
                    network: &network,
                },
                &None,
                &[],
            )
        };
        assert!(matches!(
            result,
            Err((
                Error::IntelWifi(net::wifi::intel::attach::AttachError::NoRegulatoryDatabase),
                None
            ))
        ));
        // SAFETY: as above.
        let result = unsafe {
            mediatek_wifi(
                &services,
                address,
                None,
                &WifiInputs {
                    clock: handoff.clock,
                    observations: Default::default(),
                    firmware,
                    network: &network,
                },
                &None,
                &[],
                false,
            )
        };
        assert!(matches!(
            result,
            Err((
                Error::MediatekWifi(net::wifi::mediatek::attach::AttachError::NoRegulatoryDatabase),
                None
            ))
        ));
        assert!(services.requests.borrow().is_empty());
    }

    #[test]
    fn a_board_smbios_does_not_name_attaches_no_broadcom_bluetooth() {
        let services = services::tests::Refusing::default();
        let handoff = Handoff {
            ecam: Vec::new(),
            mapped: Vec::new(),
            dma: 0..0,
            clock: crate::Clock {
                ticks_per_microsecond: 1,
            },
            firmware: None,
            completion: None,
        };
        let address = Address {
            segment: 0,
            bus: 2,
            device: 0,
            function: 0,
        };
        // SAFETY: the refusing services cannot grant hardware access.
        let result = unsafe {
            bcm4377(
                &services,
                address,
                &[],
                handoff_provider(&handoff),
                handoff.clock,
            )
        };
        assert!(matches!(
            result,
            Err((
                Error::Bcm4377(crate::bluetooth::bcm4377::Error::Unsupported(_)),
                None
            ))
        ));
        assert!(services.requests.borrow().is_empty());
    }

    #[test]
    fn probe_diagnostics_preserve_exact_firmware_refusals() {
        let requirement = Requirement {
            name: "rtl_nic/rtl8168e-3.fw",
            target: Target {
                driver: "realtek",
                hardware: "10ec:8168",
                revision: "xid-2c8",
                board: None,
            },
            minimum_bytes: 4,
        };
        let missing = HandoffProvider::new(None).load(requirement).unwrap_err();
        let catalog = Catalog::from_handoff(vec![Resource {
            name: requirement.name,
            target: Target {
                revision: "xid-2c9",
                ..requirement.target
            },
            provenance: Provenance {
                source: "synthetic-fixtures",
                revision: "v1",
                member: "fixtures/abcd.bin",
                license: "CC0-1.0",
            },
            expected_length: 4,
            expected_sha256: Sha256::digest(b"abcd").into(),
            bytes: b"abcd".to_vec(),
        }])
        .unwrap();
        let wrong_target = HandoffProvider::new(Some(&catalog))
            .load(requirement)
            .unwrap_err();
        assert_eq!(wrong_target.kind, firmware::ErrorKind::WrongTarget);
        let address = Address {
            segment: 0,
            bus: 1,
            device: 2,
            function: 0,
        };
        let mut devices = Devices::default();
        for original in [missing, wrong_target] {
            let error = realtek::Error::Firmware(original.clone());
            devices.failures.push((address, error.into()));
            let (location, error) = devices.failures.last().unwrap();
            assert_eq!(*location, address);
            assert!(
                matches!(error, Error::Realtek(realtek::Error::Firmware(kept)) if *kept == original)
            );
            assert_eq!(format!("{error}"), format!("{original}"));
            assert!(format!("{error}").contains(requirement.name));
        }
        let device: Error = BlockError::Device { status: 0x1234 }.into();
        assert_eq!(format!("{device}"), "device status 0x1234");
    }
}
