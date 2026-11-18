//! NVIDIA GPUs under Stela's Metal device: nouveau with GSP firmware in the
//! kernel and NVK as the Vulkan driver.
//!
//! Hardware access, firmware, memory management, command submission and
//! shader code generation stay with nouveau, NVK and NAK; this module only
//! admits a device and supplies the facts Vulkan does not report, as the
//! NVIDIA stack's part of [`open`](super::open).
use super::{
    abi::DeviceLocation,
    capabilities::Platform,
    open::{self, Admission, Attachment},
    vulkan::{DriverId, PhysicalDeviceType, physical::Identity},
};
use alloc::string::String;

pub const VENDOR_ID: u32 = 0x10de;

pub use crate::nouveau_chip::{Architecture, CHIPS, Chip};

/// The first Turing chipset; nouveau's card type for 0x160-0x16f is TU100.
const FIRST_GSP_CHIPSET: u16 = 0x160;

/// Why a Vulkan device is not admitted as an NVIDIA Metal device.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Refusal {
    NotNvidia {
        vendor_id: u32,
    },
    /// Only NVK is integrated with Stela's nouveau winsys.
    NotNvk(DriverId),
    /// Chips before Turing have no GSP firmware path.
    PreTuring {
        chipset: u16,
    },
    /// nouveau does not name this chipset.
    UnknownChipset {
        chipset: u16,
    },
    /// Tegra parts share system memory and have no PCI function.
    NotDiscrete(PhysicalDeviceType),
}

impl Chip {
    /// Admits the Vulkan device `identity` whose kernel driver reports
    /// `chipset` (`NOUVEAU_GETPARAM_CHIPSET_ID`).
    pub fn admit(identity: &Identity, chipset: u16) -> Result<&'static Chip, Refusal> {
        if identity.vendor_id != VENDOR_ID {
            return Err(Refusal::NotNvidia {
                vendor_id: identity.vendor_id,
            });
        }
        if identity.driver_id != DriverId::MesaNvk {
            return Err(Refusal::NotNvk(identity.driver_id));
        }
        if identity.device_type != PhysicalDeviceType::DiscreteGpu {
            return Err(Refusal::NotDiscrete(identity.device_type));
        }
        const PASCAL_CHIPSETS: [u16; 5] = [0x132, 0x134, 0x136, 0x137, 0x138];
        if chipset < FIRST_GSP_CHIPSET && !PASCAL_CHIPSETS.contains(&chipset) {
            return Err(Refusal::PreTuring { chipset });
        }
        Self::find(chipset).ok_or(Refusal::UnknownChipset { chipset })
    }
}

/// NVK's explicit linear layouts need 32-byte row pitches on Maxwell B and
/// later (NIL `Image::new_linear`).
const LINEAR_ROW_ALIGNMENT: u64 = 32;
/// NIL aligns linear images of an explicit row pitch to 32 bytes on Maxwell B
/// and later, and NVK binds an image without a PTE kind at its memory's
/// address plus the offset (`nvk_image_plane_bind`), mapping no pages of its
/// own.
const LINEAR_OFFSET_ALIGNMENT: u64 = 32;
/// nouveau's sparse binding uses the GPU's 64 KiB big pages, matching
/// Vulkan's standard sparse block shapes.
const SPARSE_PAGE_BYTES: u64 = 64 << 10;

impl Chip {
    pub fn platform(&self, attachment: Attachment) -> Platform {
        Platform {
            architecture: String::from(self.name),
            location: if attachment.external {
                DeviceLocation::External
            } else {
                DeviceLocation::Slot
            },
            location_number: attachment.location_number,
            max_transfer_rate: attachment.link.bytes_per_second(),
            headless: !self.display,
            removable: attachment.external,
            linear_row_alignment: LINEAR_ROW_ALIGNMENT,
            linear_offset_alignment: LINEAR_OFFSET_ALIGNMENT,
            sparse_page_bytes: SPARSE_PAGE_BYTES,
        }
    }
}

/// The NVIDIA stack's admission: nouveau's chipset must be one of [`CHIPS`]
/// and the Vulkan device NVK's.
pub const ADMISSION: Admission = Admission { admit };

fn admit(
    identity: &Identity,
    chip: u32,
    attachment: Attachment,
) -> Result<Platform, open::Refusal> {
    let chipset = u16::try_from(chip).unwrap_or(u16::MAX);
    Chip::admit(identity, chipset)
        .map(|chip| chip.platform(attachment))
        .map_err(open::Refusal::Nvidia)
}

/// NVK as linked into the image, with its driver-internal calls
/// (`compat/mesa/nir_options.c`, `compat/mesa/submit_thread.c`).
#[cfg(all(feature = "metal-linked", not(test)))]
pub const DRIVER: super::native::Driver = {
    use super::native::{Driver, Refusals, vk};
    use core::ffi::{c_char, c_void};
    unsafe extern "system" {
        fn vk_icdGetInstanceProcAddr(
            instance: vk::Instance,
            name: *const c_char,
        ) -> vk::PFN_vkVoidFunction;
    }
    unsafe extern "C" {
        fn stela_nvk_nir_options(physical: vk::PhysicalDevice, stage: u32) -> *const c_void;
        fn stela_nvk_submit_thread(queue: vk::Queue) -> vk::Result;
        fn stela_nvk_submit_failure(
            queue: vk::Queue,
            semaphore: vk::Semaphore,
            value: u64,
        ) -> vk::Result;
        fn stela_nvk_forget_failures(queue: vk::Queue) -> u32;
        fn stela_nvk_count_failures(queue: vk::Queue, outstanding: *mut u32);
    }
    Driver {
        get_instance_proc_addr: vk_icdGetInstanceProcAddr,
        nir_options: stela_nvk_nir_options,
        // A commit's submission runs where Linux does, not on its PE.
        submit_thread: Some(stela_nvk_submit_thread),
        // A refused submission fails its command buffer, not the device.
        refusals: Some(Refusals {
            take: stela_nvk_submit_failure,
            forget: stela_nvk_forget_failures,
            count: stela_nvk_count_failures,
        }),
        keep_lost: false,
    }
};

#[cfg(test)]
pub(crate) mod fixture;

#[cfg(test)]
mod tests;
