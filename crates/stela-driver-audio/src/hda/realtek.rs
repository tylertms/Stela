// SPDX-License-Identifier: GPL-2.0-only
//! Codec-common Realtek PLL/EAPD and silicon-revision controls adapted from Linux
//! sound/hda/codecs/realtek/{realtek,alc882,alc662,alc269}.c at
//! a804c5742bff224d4cb4d25dd383554d598be32b (GPL-2.0-or-later).
//! This explicitly scoped setup does not select OEM fixups or establish board,
//! GPIO, companion-amplifier, headset or speaker readiness. See NOTICE.
use super::{CodecGraph, Controller, Registers, Verb};
use crate::{Error, Timer};
use alloc::vec::Vec;

mod alc245;
mod family;
pub use family::RealtekFamily;

/// Whether the managed ALC882/ALC662-family initialization owns this codec.
pub(super) fn managed(id: u32, revision: u32) -> bool {
    family::family(id, revision).is_ok()
}
mod oem;
pub use oem::RealtekOem;
mod laptop;
mod managed;
pub(super) use managed::State;
pub use managed::{AmplifierLevel, RealtekCaptureGain, RealtekReport};
mod headphone;
pub use headphone::RealtekHeadphoneReport;
mod dual_headphone;
pub use dual_headphone::{RealtekDualHeadphoneReport, RealtekHeadphoneRoute};

const PROCESSING: u8 = 0x20;
const GET_PARAMETER: u16 = 0xf00;
#[derive(Clone, Copy)]
struct Profile {
    pll: Option<(u16, u16)>,
    eapd: Option<(u16, u16)>,
}
fn profile(id: u32) -> Result<Profile, Error> {
    let (pll, eapd) = match id {
        0x10ec_0887 => (Some((0x0a, 1 << 10)), None),
        0x10ec_0892 | 0x10ec_0897 => (Some((0x04, 1 << 15)), Some((0x07, 1 << 5))),
        0x10ec_0900 | 0x10ec_1220 => (None, Some((0x07, 1 << 1))),
        // The donor's alc882_probe default includes this ALC1220 alias, unlike
        // 1220 itself: keep its PLL clear in addition to common EAPD control.
        0x10ec_1168 => (Some((0x0a, 1 << 10)), Some((0x07, 1 << 1))),
        _ => return Err(Error::UnsupportedTopology),
    };
    Ok(Profile { pll, eapd })
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct CoefficientChange {
    pub index: u16,
    pub previous: u16,
    pub current: u16,
}
/// An observation of codec-common controls only, not a board-readiness token.
/// AFG subsystem ID is reported raw; no board identity is inferred from it.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekCommonReport {
    pub codec_id: u32,
    pub revision_id: u32,
    pub subsystem_id: u32,
    pub coefficients: Vec<CoefficientChange>,
}
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Alc269Variant {
    Va,
    Vb,
    Vc,
    Vd,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct Alc269CommonReport {
    pub common: RealtekCommonReport,
    pub coefficient_zero: u16,
    pub variant: Alc269Variant,
}

impl<R: Registers, T: Timer> Controller<R, T> {
    fn realtek_parameter(&mut self, codec: u8, node: u8, parameter: u8) -> Result<u32, Error> {
        self.minimum_guard()?;
        self.verb(Verb::short(codec, node, GET_PARAMETER, parameter)?)
    }
    fn coefficient_index(&mut self, codec: u8, index: u16) -> Result<(), Error> {
        self.minimum_guard()?;
        self.verb(Verb::long(codec, PROCESSING, 5, index)?)?;
        if self.verb(Verb::long(codec, PROCESSING, 13, 0)?)? != u32::from(index) {
            return Err(Error::CoefficientRejected {
                node: PROCESSING,
                index,
            });
        }
        Ok(())
    }
    fn read_coefficient(&mut self, codec: u8, index: u16) -> Result<u16, Error> {
        self.minimum_guard()?;
        self.coefficient_index(codec, index)?;
        u16::try_from(self.verb(Verb::long(codec, PROCESSING, 12, 0)?)?).map_err(|_| {
            Error::CoefficientRejected {
                node: PROCESSING,
                index,
            }
        })
    }
    fn write_coefficient_observed(
        &mut self,
        codec: u8,
        index: u16,
        previous: u16,
        current: u16,
    ) -> Result<CoefficientChange, Error> {
        self.minimum_guard()?;
        self.coefficient_index(codec, index)?;
        self.verb(Verb::long(codec, PROCESSING, 4, current)?)?;
        self.coefficient_index(codec, index)?;
        if self.verb(Verb::long(codec, PROCESSING, 12, 0)?)? != u32::from(current) {
            return Err(Error::CoefficientRejected {
                node: PROCESSING,
                index,
            });
        }
        Ok(CoefficientChange {
            index,
            previous,
            current,
        })
    }
    fn update_coefficient(
        &mut self,
        codec: u8,
        index: u16,
        mask: u16,
        set: u16,
    ) -> Result<CoefficientChange, Error> {
        self.minimum_guard()?;
        let previous = self.read_coefficient(codec, index)?;
        let current = (previous & !mask) | set;
        if current == previous {
            return Ok(CoefficientChange {
                index,
                previous,
                current,
            });
        }
        self.write_coefficient_observed(codec, index, previous, current)
    }
    fn clear_coefficient_bits(
        &mut self,
        codec: u8,
        index: u16,
        mask: u16,
    ) -> Result<CoefficientChange, Error> {
        self.minimum_guard()?;
        self.update_coefficient(codec, index, mask, 0)
    }

    fn realtek_common_guard(&self, graph: &CodecGraph) -> Result<(), Error> {
        self.minimum_guard()?;
        if graph.token != self.token {
            return Err(Error::InvalidCodecGraph);
        }
        if self.routing_failed {
            return Err(Error::NeedsReset);
        }
        self.require_routing_stopped()?;
        if self
            .converter_bindings
            .iter()
            .any(|binding| binding.codec == graph.address)
            || self.manages_realtek_codec(graph.address)
            || self.manages_cirrus_codec(graph.address)
            || self.manages_conexant_codec(graph.address)
            || self.manages_sigmatel_codec(graph.address)
        {
            return Err(Error::RouteInUse);
        }
        if self
            .routes
            .iter()
            .any(|(_, route)| route.codec_address() == graph.address)
            || self
                .codec_controls
                .iter()
                .any(|(_, codec, _)| *codec == graph.address)
        {
            return Err(Error::RouteInUse);
        }
        Ok(())
    }
    fn realtek_common_identity(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<(u32, u32, u32, bool), Error> {
        self.minimum_guard()?;
        let group = graph
            .groups
            .iter()
            .find(|group| group.node == 1 && group.function_type & 255 == 1)
            .ok_or(Error::InvalidCodecGraph)?;
        if group
            .power_states
            .is_none_or(|value| value == u32::MAX || value & 1 == 0)
        {
            return Err(Error::UnsupportedTopology);
        }
        let codec_id = self.realtek_parameter(graph.address, 0, 0)?;
        let revision_id = self.realtek_parameter(graph.address, 0, 2)?;
        if codec_id != graph.vendor_id
            || revision_id != graph.revision_id
            || revision_id == u32::MAX
        {
            return Err(Error::InvalidCodecGraph);
        }
        // These profiles use their standard AFG1 processing node20.
        let function = self.realtek_parameter(graph.address, 1, 5)?;
        let power = self.realtek_parameter(graph.address, 1, 0x0f)?;
        let nodes = self.realtek_parameter(graph.address, 1, 4)?;
        let first = (nodes >> 16) & 255;
        let count = nodes & 255;
        if function == u32::MAX
            || function & 255 != 1
            || power == u32::MAX
            || power & 1 == 0
            || first < 2
            || first > u32::from(PROCESSING)
            || count == 0
            || first + count > 128
            || first + count <= u32::from(PROCESSING)
        {
            return Err(Error::InvalidCodecGraph);
        }
        let caps = self.realtek_parameter(graph.address, PROCESSING, 9)?;
        if caps == u32::MAX || (caps >> 20) & 15 != 15 || caps & (1 << 6) == 0 {
            return Err(Error::UnsupportedTopology);
        }
        let vendor_power = if caps & (1 << 10) != 0 {
            let value = self.realtek_parameter(graph.address, PROCESSING, 0x0f)?;
            let effective = if value == 0 { power } else { value };
            if effective == u32::MAX || effective & 1 == 0 {
                return Err(Error::UnsupportedTopology);
            }
            true
        } else {
            false
        };
        let subsystem_id = self.verb(Verb::short(graph.address, 1, 0xf20, 0)?)?;
        Ok((codec_id, revision_id, subsystem_id, vendor_power))
    }

    /// Apply only exact codec-common desktop PLL/EAPD coefficient masks.
    /// Live identity/revision/AFG/vendor-node bounds are checked again before
    /// writes, so editing public graph metadata cannot select a different chip.
    /// Every stream must be stopped, none poisoned, and this codec must have no
    /// reserved route. D0 and every written coefficient are read back. Failure
    /// after programming begins blocks typed routing/start until reinitialize.
    /// This does not enable EAPD pins, choose PinOptions or apply an OEM fixup.
    /// ALC2xx depop/headset/revision procedures are unsupported here.
    pub fn initialize_realtek_common(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<RealtekCommonReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        let selected = profile(graph.vendor_id)?;
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, PROCESSING)?;
            }
            let mut coefficients = Vec::new();
            for (index, mask) in [selected.pll, selected.eapd].into_iter().flatten() {
                coefficients.push(self.clear_coefficient_bits(graph.address, index, mask)?);
            }
            Ok(RealtekCommonReport {
                codec_id,
                revision_id,
                subsystem_id,
                coefficients,
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
    /// Exact ALC269 common silicon-revision setup. This remains separate from
    /// desktop setup and from board/GPIO, headphone-route and depop lifecycle
    /// policy. Unknown variant nibbles fail rather than inheriting VA controls.
    pub fn initialize_realtek_alc269_common(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<Alc269CommonReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        if graph.vendor_id != 0x10ec_0269 {
            return Err(Error::UnsupportedTopology);
        }
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        // alc269_probe replaces the ordinary init hook when node51 identifies
        // an ALC5505 DSP. No companion/DSP owner is supplied by this entry;
        // refuse that and opaque nonzero replies before power/index writes.
        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, PROCESSING)?;
            }
            let coefficient_zero = self.read_coefficient(graph.address, 0)?;
            let variant = match coefficient_zero & 0x00f0 {
                0x00 => Alc269Variant::Va,
                0x10 => Alc269Variant::Vb,
                0x20 => Alc269Variant::Vc,
                0x30 => Alc269Variant::Vd,
                _ => return Err(Error::UnsupportedTopology),
            };
            let mut coefficients = Vec::new();
            match variant {
                Alc269Variant::Va | Alc269Variant::Vc => {
                    // VA registers a PLL bit15 clear; VC uses the same index/
                    // mask for its EAPD verb-control selection, without a PLL fix.
                    coefficients.push(self.clear_coefficient_bits(graph.address, 4, 1 << 15)?);
                }
                Alc269Variant::Vd => {
                    coefficients.push(self.clear_coefficient_bits(graph.address, 0x10, 1 << 9)?);
                }
                Alc269Variant::Vb => {
                    let revision = coefficient_zero & 255;
                    // alc269_fill_coef's exact early-VB literal values.
                    if revision < 0x15 || revision == 0x16 {
                        for (index, value) in [
                            (0x0f, 0x960b),
                            (0x0e, if revision == 0x16 { 0x8814 } else { 0x8817 }),
                        ] {
                            let old = self.read_coefficient(graph.address, index)?;
                            // Preserve the donor's unconditional literal write.
                            coefficients.push(self.write_coefficient_observed(
                                graph.address,
                                index,
                                old,
                                value,
                            )?);
                        }
                    }
                    if revision == 0x18 {
                        for (index, field, expected, bit) in [
                            (0x0d, 0x0c00, 0x0400, 1 << 10),
                            (0x17, 0x01c0, 0x0100, 1 << 7),
                        ] {
                            let old = self.read_coefficient(graph.address, index)?;
                            if old & field != expected {
                                // These are OR operations in the pinned source,
                                // not normalized field replacements.
                                coefficients.push(self.write_coefficient_observed(
                                    graph.address,
                                    index,
                                    old,
                                    old | bit,
                                )?);
                            }
                        }
                    }
                    coefficients.push(self.update_coefficient(graph.address, 4, 0, 1 << 11)?);
                    coefficients.push(self.update_coefficient(graph.address, 0x0d, 0, 1 << 14)?);
                }
            }
            Ok(Alc269CommonReport {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                coefficient_zero,
                variant,
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
}
