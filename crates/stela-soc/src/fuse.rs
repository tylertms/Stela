//! The always-on eFuse block (`/arm-io/pmgr` `reg[36]` on J473AP, also mapped
//! by `apcie` and the `atc-phy` nodes): per-die calibration the factory burns.
//!
//! 26A434 J473AP `AppleT8112PMGR::getVoltageRailIDS` reads, through
//! `ApplePMGR::readPMGRAONFuseReg`, the leakage current (IDS) of three
//! voltage rails, scaled to amperes by the driver:
//!
//! - rail 0: `0x4c4` bits 28:18;
//! - rail 7: `0x4d8` bits 31:19;
//! - rail 12: `0x4cc` bits 29:21.
//!
//! `AGXAcceleratorG14G::populatePowerEstimationConfigData` (KC file
//! 0x1b6a1f0..0x1b6a3b0) reads the GPU's power-estimation calibration. When
//! a 128-bit configuration it holds has ten bits set it takes `0x4e0`:`0x4dc`
//! bits 42:30 and `0x4b0` bits 23:15, otherwise `0x4dc` bits 29:17 and `0x4e0`
//! bits 19:11 (field tables at file 0x13c618 and 0x13c640). It stores
//! float(first + 2) and a table scale times (second + 1) in the firmware's
//! power-estimation configuration, with no division or validity check.
//!
//! The drivers use these fields only in power estimates. An emulated die has
//! no leakage or power characteristic to calibrate, so each field reads 0:
//! the estimates count the smallest calibrated values rather than invented
//! ones. The registers' other bits are fields no reader of this kernel is
//! known to take, so they read 0 as well, and every other register refuses.
//! Fuses are read-only.
//!
//! `T8112CLPC::isRunningOnSilicon` (0xfffffe0009f78cf0) reads `0x80c` and
//! takes bits 31:28 of 0 for a real die, so 0 there is the silicon answer.
use {
    stela_contracts::bus::Access, stela_contracts::bus::Direction,
    stela_contracts::bus::MmioDevice, stela_contracts::bus::MmioError, stela_contracts::bus::Width,
    stela_contracts::bus::refuse,
};

const NAME: &str = "aon-fuse";

/// The calibration registers a model describes.
const DESCRIBED: [u64; 7] = [0x4b0, 0x4c4, 0x4cc, 0x4d8, 0x4dc, 0x4e0, 0x80c];

/// The block's span.
pub const FUSE_BYTES: u64 = 0x4000;

#[derive(Debug, Clone, Copy, Default)]
pub struct AonFuses;

/// T6020 IDS fields are used for the guest's power estimates. A cacheless
/// frontend has no physical die leakage to calibrate, as with AonFuses.
pub(crate) struct RailCalibration;

impl MmioDevice for RailCalibration {
    fn name(&self) -> &'static str {
        "t6020-rail-calibration"
    }

    fn read(&mut self, offset: u64, width: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        if width == Width::Word && matches!(offset, 0 | 4 | 8) {
            Ok(0)
        } else {
            Err(refuse(
                self.name(),
                offset,
                width,
                Direction::Read,
                "not a described IDS word",
            ))
        }
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        _: u64,
        _: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        Err(refuse(
            self.name(),
            offset,
            width,
            Direction::Write,
            "IDS calibration is read-only",
        ))
    }
}

pub(crate) struct T6020PerfBoot;

impl MmioDevice for T6020PerfBoot {
    fn name(&self) -> &'static str {
        "t6020-perf-boot"
    }

    fn read(&mut self, offset: u64, width: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        if offset == 0 && width == Width::Double {
            Ok(0)
        } else {
            Err(refuse(
                self.name(),
                offset,
                width,
                Direction::Read,
                "not a 64-bit boot status/weight read",
            ))
        }
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        _: u64,
        _: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        Err(refuse(
            self.name(),
            offset,
            width,
            Direction::Write,
            "performance-control effects are not modeled",
        ))
    }
}

impl MmioDevice for AonFuses {
    fn name(&self) -> &'static str {
        NAME
    }

    fn read(&mut self, offset: u64, width: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        if width == Width::Word && DESCRIBED.contains(&offset) {
            Ok(0)
        } else {
            Err(refuse(
                NAME,
                offset,
                width,
                Direction::Read,
                "eFuse register without described meaning",
            ))
        }
    }

    fn write(
        &mut self,
        offset: u64,
        width: Width,
        _: u64,
        _: &mut Access<'_>,
    ) -> Result<(), MmioError> {
        Err(refuse(
            NAME,
            offset,
            width,
            Direction::Write,
            "eFuses are read-only",
        ))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use stela_contracts::interrupt::InterruptLatch;

    #[test]
    fn the_rail_leakage_and_gpu_power_fields_read_uncalibrated() {
        let mut latch = InterruptLatch::new();
        let mut access = Access::new(0, 0, &mut latch);
        let mut fuses = AonFuses;
        for offset in DESCRIBED {
            assert_eq!(fuses.read(offset, Width::Word, &mut access), Ok(0));
        }
        assert!(fuses.read(0x4c8, Width::Word, &mut access).is_err());
        assert!(fuses.read(0x4c4, Width::Double, &mut access).is_err());
        assert!(fuses.write(0x4c4, Width::Word, 1, &mut access).is_err());
    }
}
