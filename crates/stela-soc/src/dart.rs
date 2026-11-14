//! Native Apple DART generation selection. Chip placement selects an explicit
//! register/table profile; a compatible string is never a generation alias.
//! All bus masters keep the same [`Dart`] DMA and lifecycle boundary.
use crate::Result;
use stela_contracts::refuse;
use stela_soc_data::dart::{DAPF_BYTES, TRANSLATION_WINDOW_LIMIT};
pub(crate) mod descriptor;
pub mod gen3;
mod irq;
mod legacy;
mod legacy_umms;
pub mod smmu;
mod t8110;

use alloc::vec::Vec;
pub use irq::SharedIrq;
pub use smmu::SmmuBlock;
use stela_contracts::interrupt::AicInput;
use {
    stela_contracts::bus::Access, stela_contracts::bus::MmioDevice,
    stela_contracts::bus::MmioError, stela_contracts::bus::Width,
};
use {stela_contracts::dma::DmaError, stela_contracts::dma::Iommu, stela_contracts::dma::Memory};
/// Internal bus offsets of the independently placed APF and UMMS views,
/// above the largest translation window any profile admits.
pub const DAPF_OFFSET: u64 = TRANSLATION_WINDOW_LIMIT;
pub const SMMU_OFFSET: u64 = DAPF_OFFSET + DAPF_BYTES;
pub const SMMU_BYTES: u64 = 0x4000;

/// A t8110 translation fault's ERROR code; legacy generations use their
/// separately encoded error bits.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Fault {
    NoTtbr = 1,
    /// A missing first-level entry of a revision 2 FOUR_LEVEL walk.
    NoPgd = 1 << 1,
    NoPmd = 1 << 2,
    NoPte = 1 << 3,
    Write = 1 << 4,
    Read = 1 << 5,
    Region = 1 << 29,
}

use stela_soc_data::dart::Profile;

enum Model {
    Legacy(legacy::Dart),
    T8110(t8110::Dart),
}

pub struct Dart {
    profile: Profile,
    model: Model,
}

impl Dart {
    /// Join a physical input's wired-OR owner before mapping this translator.
    /// An active source cannot be rebound without losing its old assertion.
    pub fn bind_shared_irq(&mut self, irq: &SharedIrq) -> Result<()> {
        match &mut self.model {
            Model::Legacy(model) => model.bind_shared_irq(irq),
            Model::T8110(model) => model.bind_shared_irq(irq),
        }
    }

    /// The existing cacheless revision-1 t8110 profile.
    pub fn new(streams: usize, irq: Option<AicInput>) -> Self {
        Self {
            profile: Profile::T8110,
            model: Model::T8110(t8110::Dart::new(streams, irq)),
        }
    }

    /// Construct the native generation's register/table format. Original
    /// per-chip placement and optional interfaces require separate validation.
    pub fn for_profile(profile: Profile, streams: usize, irq: Option<AicInput>) -> Result<Self> {
        if streams == 0 || streams > profile.stream_limit() {
            return Err(refuse!(
                "DART stream count is outside the selected native profile"
            ));
        }
        Ok(Self {
            profile,
            model: match profile {
                Profile::T8110 | Profile::T8110Rev20 | Profile::T8110Rev22 => Model::T8110(
                    t8110::Dart::for_version(t8110::version(profile), streams, irq),
                ),
                Profile::T8020 | Profile::T6000 => {
                    Model::Legacy(legacy::Dart::new(profile, streams, irq))
                }
            },
        })
    }

    /// Construction profile, kept independently of optional views.
    pub fn profile(&self) -> Profile {
        self.profile
    }

    /// Constructed incoming-SID capacity, independent of enable/configuration
    /// and lifecycle. Native masters validate their owner before mapping.
    pub fn stream_capacity(&self) -> usize {
        match &self.model {
            Model::Legacy(model) => model.stream_capacity(),
            Model::T8110(model) => model.stream_capacity(),
        }
    }

    /// Independent TRAD windows sharing one level-sensitive fault wire.
    pub fn group(streams: usize, irq: Option<AicInput>, count: usize) -> Vec<Self> {
        assert!(count != 0, "a DART group needs a translation instance");
        t8110::Dart::group(streams, irq, count)
            .into_iter()
            .map(|model| Self {
                profile: Profile::T8110,
                model: Model::T8110(model),
            })
            .collect()
    }

    pub fn group_for_profile(
        profile: Profile,
        streams: usize,
        irq: Option<AicInput>,
        count: usize,
    ) -> Result<Vec<Self>> {
        if count == 0 {
            return Err(refuse!("a DART group needs a translation instance"));
        }
        Self::for_profile(profile, streams, irq)?;
        Ok(match profile {
            Profile::T8110 | Profile::T8110Rev20 | Profile::T8110Rev22 => {
                t8110::Dart::group_for_version(t8110::version(profile), streams, irq, count)
                    .into_iter()
                    .map(|model| Self {
                        profile,
                        model: Model::T8110(model),
                    })
                    .collect()
            }
            Profile::T8020 | Profile::T6000 => legacy::Dart::group(profile, streams, irq, count)
                .into_iter()
                .map(|model| Self {
                    profile,
                    model: Model::Legacy(model),
                })
                .collect(),
        })
    }

    pub fn with_apf(mut self, slices: usize) -> Self {
        self.model = match self.model {
            Model::T8110(model) => Model::T8110(model.with_apf(slices)),
            Model::Legacy(model) => Model::Legacy(model.with_apf(slices)),
        };
        self
    }

    #[cfg(test)]
    pub fn with_smmu(mut self) -> Result<Self> {
        self.enable_smmu()?;
        Ok(self)
    }

    /// Whether this generation implements a native paired UMMS view. This
    /// does not check optional hardware capture/performance/configuration
    /// forms: unsupported accesses still refuse inside the selected family.
    /// The query never attaches a view or changes a device's state.
    pub fn supports_smmu(&self) -> bool {
        match &self.model {
            Model::T8110(_) => true,
            Model::Legacy(model) => model.supports_smmu(),
        }
    }

    /// Validate support and uniqueness before placing any physical window.
    #[cfg(test)]
    pub fn can_attach_smmu(&self) -> bool {
        self.supports_smmu() && !self.has_smmu()
    }

    /// Attach only a supported, unique view. A refused attachment leaves the
    /// model unchanged; machine placement must validate before mapping MMIO.
    pub fn enable_smmu(&mut self) -> Result<()> {
        match &mut self.model {
            Model::T8110(model) => {
                if model.enable_smmu() {
                    Ok(())
                } else {
                    Err(refuse!("DART already has a paired SMMU instance"))
                }
            }
            Model::Legacy(model) => model.enable_smmu(),
        }
    }

    /// Whether this instance already has a supported paired capture view.
    pub fn has_smmu(&self) -> bool {
        match &self.model {
            Model::T8110(model) => model.has_smmu(),
            Model::Legacy(model) => model.has_smmu(),
        }
    }

    pub fn set_power(&mut self, powered: bool, retention: bool, access: &mut Access<'_>) {
        match &mut self.model {
            Model::T8110(model) => model.set_power(powered, retention, access),
            Model::Legacy(model) => model.set_power(powered, retention, access),
        }
    }

    pub fn set_quiesced(&mut self, quiesced: bool) {
        match &mut self.model {
            Model::T8110(model) => model.set_quiesced(quiesced),
            Model::Legacy(model) => model.set_quiesced(quiesced),
        }
    }

    /// Gate the functional clock without resetting any register state. A
    /// clock-disabled device cannot perform MMIO or DMA; power is independent.
    pub fn set_clock(&mut self, enabled: bool) {
        match &mut self.model {
            Model::T8110(model) => model.set_clock(enabled),
            Model::Legacy(model) => model.set_clock(enabled),
        }
    }

    pub fn set_reset(&mut self, asserted: bool, access: &mut Access<'_>) {
        match &mut self.model {
            Model::T8110(model) => model.set_reset(asserted, access),
            Model::Legacy(model) => model.set_reset(asserted, access),
        }
    }
}

impl Dart {
    /// The physical address `stream` reaches at `iova` for the bytes up to
    /// `last` (in the same page), as [`Iommu::translate`] finds it, but
    /// without its effects: a fault is neither recorded nor signalled. What
    /// a copy outside the devices' lock reads by; a page that does not
    /// resolve is left to `translate`, under the lock.
    pub fn resolve(
        &self,
        memory: &mut dyn Memory,
        stream: usize,
        iova: u64,
        last: u64,
        write: bool,
    ) -> Result<u64, DmaError> {
        match &self.model {
            Model::T8110(model) => model.walk(memory, stream, iova, last, write),
            Model::Legacy(model) => model.walk(memory, stream, iova, last, write),
        }
    }
}

impl Iommu for Dart {
    fn page(&self) -> u64 {
        match &self.model {
            Model::T8110(model) => model.page(),
            Model::Legacy(model) => model.page(),
        }
    }

    fn translate(
        &mut self,
        memory: &mut dyn Memory,
        stream: usize,
        iova: u64,
        last: u64,
        write: bool,
        access: &mut Access<'_>,
    ) -> Result<u64, DmaError> {
        match &mut self.model {
            Model::T8110(model) => model.translate(memory, stream, iova, last, write, access),
            Model::Legacy(model) => model.translate(memory, stream, iova, last, write, access),
        }
    }
}

impl MmioDevice for Dart {
    fn advance_policy(&self) -> super::bus::AdvancePolicy {
        super::bus::AdvancePolicy::Passive
    }
    fn name(&self) -> &'static str {
        match &self.model {
            Model::T8110(model) => model.name(),
            Model::Legacy(model) => model.name(),
        }
    }

    fn read(
        &mut self,
        offset: u64,
        width: Width,
        access: &mut Access<'_>,
    ) -> Result<u64, MmioError> {
        match &mut self.model {
            Model::T8110(model) => model.read(offset, width, access),
            Model::Legacy(model) => model.read(offset, width, access),
        }
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        value: u64,
        access: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        match &mut self.model {
            Model::T8110(model) => model.write(offset, width, value, access),
            Model::Legacy(model) => model.write(offset, width, value, access),
        }
    }
}

#[cfg(test)]
mod tests;
