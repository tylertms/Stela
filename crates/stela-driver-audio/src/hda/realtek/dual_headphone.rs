// SPDX-License-Identifier: GPL-2.0-only
//! ALC285/295 held-D0 startup adapted from Linux alc269.c::{alc225_init,
//! alc285_hp_init,alc269_probe} and realtek.c::alc_fill_eapd_coef at
//! a804c5742bff224d4cb4d25dd383554d598be32b (GPL-2.0-or-later).
//! The fixed second pin is an observed/control endpoint, not an invented DAC
//! route. No OEM wiring, GPIO/companion amplifier or ULP policy is inferred.
use super::{CoefficientChange, RealtekCommonReport, headphone::DepopControl};
use crate::{
    Direction, Error, Phase, Timer,
    hda::{
        AnalogRoute, CodecGraph, Controller, PinOptions, PinSense, Registers, StreamId, Verb,
        WidgetKind,
    },
};
use alloc::vec::Vec;

/// A route chosen by the caller, including electrical options.
#[derive(Clone, Copy, Debug)]
pub struct RealtekHeadphoneRoute<'a> {
    pub route: &'a AnalogRoute,
    pub stream: StreamId,
    pub channel: u8,
    pub options: PinOptions,
}

/// Startup observations only. Array order is selected primary pin, fixed pin16;
/// those may be the same node. Only explicit supplied routes carry PCM.
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekDualHeadphoneReport {
    pub common: RealtekCommonReport,
    /// Coefficient0 used for ALC285 variant selection; absent for295.
    pub coefficient_zero: Option<u16>,
    pub boot_depop_performed: bool,
    pub headphone_pins: [u8; 2],
    pub initial_senses: [PinSense; 2],
}

impl<R: Registers, T: Timer> Controller<R, T> {
    // Node58 is fixed in the donor. Only its three sequencer indices
    // enter this private helper; no public arbitrary vendor-NID API is added.
    fn sequencer_index(&mut self, codec: u8, index: u16) -> Result<(), Error> {
        self.minimum_guard()?;
        if !matches!(index, 0 | 3 | 0x0f) {
            return Err(Error::InvalidConfiguration);
        }
        self.verb(Verb::long(codec, 0x58, 5, index)?)?;
        if self.verb(Verb::long(codec, 0x58, 13, 0)?)? != u32::from(index) {
            return Err(Error::CoefficientRejected { node: 0x58, index });
        }
        Ok(())
    }
    fn sequencer_read(&mut self, codec: u8, index: u16) -> Result<u16, Error> {
        self.minimum_guard()?;
        self.sequencer_index(codec, index)?;
        u16::try_from(self.verb(Verb::long(codec, 0x58, 12, 0)?)?)
            .map_err(|_| Error::CoefficientRejected { node: 0x58, index })
    }
    fn sequencer_write(&mut self, codec: u8, index: u16, value: u16) -> Result<(), Error> {
        self.minimum_guard()?;
        self.sequencer_index(codec, index)?;
        self.verb(Verb::long(codec, 0x58, 4, value)?)?;
        if self.sequencer_read(codec, index)? != value {
            return Err(Error::CoefficientRejected { node: 0x58, index });
        }
        Ok(())
    }
    fn boot_headphone_depop(
        &mut self,
        codec: u8,
        pin: u8,
    ) -> Result<Vec<CoefficientChange>, Error> {
        self.minimum_guard()?;
        self.sequencer_write(codec, 0, 0x1888)?;
        let mut changes = alloc::vec![self.update_coefficient(codec, 0x4a, 1 << 15, 1 << 15)?];
        let saved38 = self.read_coefficient(codec, 0x38)?;
        let saved0d = self.read_coefficient(codec, 0x0d)?;
        let saved36 = self.read_coefficient(codec, 0x36)?;
        changes.push(self.clear_coefficient_bits(codec, 0x38, 1 << 4)?);
        changes.push(self.clear_coefficient_bits(codec, 0x0d, 0x110)?);
        changes.push(self.update_coefficient(codec, 0x67, 0xf000, 0x3000)?);
        self.verb(Verb::long(codec, pin, 3, 0xb080)?)?;
        for channel in [0xa000, 0x8000] {
            if self.verb(Verb::long(codec, pin, 11, channel)?)? != 0x80 {
                return Err(Error::RouteRejected);
            }
        }
        self.delay(130_000)?;
        changes.push(self.update_coefficient(codec, 0x36, 1 << 14, 1 << 14)?);
        changes.push(self.clear_coefficient_bits(codec, 0x36, 1 << 13)?);
        self.route_control(codec, pin, 0x707, 0xf07, 0)?;
        self.delay(10_000)?;
        let previous = self.read_coefficient(codec, 0x67)?;
        changes.push(self.write_coefficient_observed(codec, 0x67, previous, 0)?);
        self.sequencer_write(codec, 0, 0x7880)?;
        self.sequencer_write(codec, 0x0f, 0xf049)?;
        let bias = self.sequencer_read(codec, 3)?;
        self.sequencer_write(codec, 3, (bias & !0xf0) | 0xc0)?;
        self.sequencer_index(codec, 0)?;
        self.verb(Verb::long(codec, 0x58, 4, 0xf888)?)?;
        // Bit15 is self-clearing: a completed first read is valid. Prior exact
        // sequencer transactions established the bank/index write contract.
        let mut result = self.sequencer_read(codec, 0)?;
        for _ in 0..20 {
            if result & 0x8000 == 0 {
                break;
            }
            self.delay(50_000)?;
            result = self.sequencer_read(codec, 0)?;
        }
        if result & 0x8000 != 0 {
            return Err(Error::Timeout(Phase::CodecDepop));
        }
        self.sequencer_write(codec, 0, result)?;
        // The donor ORs each full saved value after clearing its mask. Keep
        // that operation, rather than silently narrowing it to saved & mask.
        changes.push(self.update_coefficient(codec, 0x38, 1 << 4, saved38)?);
        changes.push(self.update_coefficient(codec, 0x0d, 0x110, saved0d)?);
        changes.push(self.update_coefficient(codec, 0x36, 3 << 13, saved36)?);
        self.delay(50_000)?;
        changes.push(self.clear_coefficient_bits(codec, 0x4a, 1 << 15)?);
        Ok(changes)
    }

    fn fixed_second_pin_preflight(&mut self, graph: &CodecGraph) -> Result<(bool, u32), Error> {
        self.minimum_guard()?;
        let pin = graph
            .groups
            .iter()
            .find(|g| g.node == 1)
            .and_then(|g| g.widgets.iter().find(|w| w.node == 0x16))
            .ok_or(Error::UnsupportedTopology)?;
        let nodes = self.realtek_parameter(graph.address, 1, 4)?;
        let first = (nodes >> 16) & 255;
        if first > 0x16 || first + (nodes & 255) <= 0x16 {
            return Err(Error::InvalidCodecGraph);
        }
        let caps = self.realtek_parameter(graph.address, 0x16, 9)?;
        let pin_caps = self.realtek_parameter(graph.address, 0x16, 0x0c)?;
        let defaults = self.verb(Verb::short(graph.address, 0x16, 0xf1c, 0)?)?;
        if caps == u32::MAX
            || pin_caps == u32::MAX
            || caps & (1 << 9) != 0
            || (caps >> 20) & 15 != 4
            || pin.kind != WidgetKind::Pin
            || caps != pin.capabilities
            || pin.pin_capabilities != Some(pin_caps)
            || pin.pin_default != Some(defaults)
        {
            return Err(Error::InvalidCodecGraph);
        }
        let power = caps & (1 << 10) != 0;
        if power {
            let raw = self.realtek_parameter(graph.address, 0x16, 0x0f)?;
            let states = if raw == 0 {
                self.realtek_parameter(graph.address, 1, 0x0f)?
            } else {
                raw
            };
            if states == u32::MAX || states & 1 == 0 {
                return Err(Error::UnsupportedTopology);
            }
        }
        Ok((power, pin_caps))
    }

    /// Exact ALC285/295 held-D0 startup with one primary PCM route.
    /// Fixed pin16 is always measured, including firmware NoConnection pins;
    /// positive presence requires standard output/amp capabilities.
    /// Its pin/amp controls are reserved without inventing a second DAC/path.
    /// An optional real pin16 route may use a separate disjoint stream/path.
    /// Coefficient0 selects ALC285's donor boot-depop variant. ALC295 always
    /// performs it; every call is a fresh startup, never a suspend/ULP resume.
    /// Partial failures and noncompleted depop require acknowledged reset.
    pub fn configure_realtek_alc285_295_headphones(
        &mut self,
        graph: &CodecGraph,
        primary: RealtekHeadphoneRoute<'_>,
        secondary: Option<RealtekHeadphoneRoute<'_>>,
    ) -> Result<RealtekDualHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;
        if !matches!(graph.vendor_id, 0x10ec_0285 | 0x10ec_0295) {
            return Err(Error::UnsupportedTopology);
        }
        // All token/stream/path conflicts precede live codec access.
        self.preflight_analog_route(
            graph,
            primary.route,
            primary.stream,
            primary.channel,
            primary.options,
        )?;
        if let Some(second) = secondary {
            self.preflight_analog_route(
                graph,
                second.route,
                second.stream,
                second.channel,
                second.options,
            )?;
            if second.route.pin() != 0x16
                || second.route.direction() != Direction::Playback
                || second.route.group_node() != 1
                || second.channel != 0
                || second.stream == primary.stream
                || primary
                    .route
                    .nodes()
                    .iter()
                    .any(|node| second.route.nodes().contains(node))
            {
                return Err(Error::RouteInUse);
            }
        }
        let pin_power = self.headphone_preflight(
            graph,
            primary.route,
            primary.stream,
            primary.channel,
            primary.options,
        )?;
        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }
        let (second_power, second_caps) = self.fixed_second_pin_preflight(graph)?;
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }
            if pin_power {
                self.power_d0(graph.address, primary.route.pin())?;
            }
            if second_power && primary.route.pin() != 0x16 {
                self.power_d0(graph.address, 0x16)?;
            }
            let coefficient_zero = if codec_id == 0x10ec_0285 {
                Some(self.read_coefficient(graph.address, 0)?)
            } else {
                None
            };
            let mut coefficients = Vec::new();
            if codec_id == 0x10ec_0295 {
                coefficients.push(self.update_coefficient(graph.address, 0x67, 0xf000, 0x3000)?);
            }
            coefficients.push(self.clear_coefficient_bits(graph.address, 0x36, 1 << 13)?);
            coefficients.push(self.clear_coefficient_bits(graph.address, 0x10, 1 << 9)?);
            let boot_depop_performed = coefficient_zero.is_none_or(|value| value & 0x10 == 0);
            if boot_depop_performed {
                coefficients.extend(self.boot_headphone_depop(graph.address, primary.route.pin())?);
            }
            self.delay(30_000)?;
            let first = self.poll_pin_sense(graph, primary.route.pin())?;
            // Donor raw standard sensing is not replaced with a synthetic
            // absence just because firmware reports NoConnection/no-presence.
            if second_caps & 2 != 0 {
                self.verb(Verb::short(graph.address, 0x16, 0x709, 0)?)?;
            }
            let raw = self.verb(Verb::short(graph.address, 0x16, 0xf09, 0)?)?;
            let code = raw & 0x7fff_ffff;
            let second = PinSense {
                raw,
                present: raw & (1 << 31) != 0,
                impedance_code: (second_caps & 1 != 0 && code != 0x7fff_ffff).then_some(code),
            };
            if second.present {
                let pin = graph.widget(0x16).ok_or(Error::InvalidCodecGraph)?;
                if second_caps & 0x14 != 0x14 || pin.channels() != 2 || pin.capabilities & 4 == 0 {
                    return Err(Error::UnsupportedTopology);
                }
                let amp = self.realtek_parameter(
                    graph.address,
                    if pin.capabilities & 8 != 0 { 0x16 } else { 1 },
                    0x12,
                )?;
                if amp == u32::MAX || pin.output_amplifier != Some(amp) {
                    return Err(Error::InvalidCodecGraph);
                }
            }
            if first.present || second.present {
                self.delay(2_000)?;
                self.depop_control(graph.address, DepopControl::HeadphonePower, 1)?;
                for (pin, sense) in [(primary.route.pin(), first), (0x16, second)] {
                    if sense.present {
                        self.route_control(graph.address, pin, 0x707, 0xf07, 0x40)?;
                    }
                }
                self.delay(75_000)?;
                for (pin, sense) in [(primary.route.pin(), first), (0x16, second)] {
                    if sense.present {
                        self.headphone_unmute_zero(graph.address, pin)?;
                    }
                }
                self.delay(75_000)?;
                coefficients.push(self.clear_coefficient_bits(graph.address, 0x4a, 3 << 10)?);
                self.depop_control(graph.address, DepopControl::HeadphonePower, 4)?;
            }
            self.configure_analog_route(
                graph,
                primary.route,
                primary.stream,
                primary.channel,
                primary.options,
            )?;
            if let Some(second) = secondary {
                self.configure_analog_route(
                    graph,
                    second.route,
                    second.stream,
                    second.channel,
                    second.options,
                )?;
            } else if !primary.route.nodes().contains(&0x16) {
                self.codec_controls
                    .push((primary.stream, graph.address, 0x16));
            }
            Ok(RealtekDualHeadphoneReport {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                coefficient_zero,
                boot_depop_performed,
                headphone_pins: [primary.route.pin(), 0x16],
                initial_senses: [first, second],
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
}

// Dedicated Acer ALC245 startup based on the Linux alc225_init sequence.
impl<R: Registers, T: Timer> Controller<R, T> {
    pub fn configure_realtek_alc245_headphones(
        &mut self,
        graph: &CodecGraph,
        primary: RealtekHeadphoneRoute<'_>,
        secondary: Option<RealtekHeadphoneRoute<'_>>,
    ) -> Result<RealtekDualHeadphoneReport, Error> {
        self.minimum_guard()?;
        self.realtek_common_guard(graph)?;

        if graph.vendor_id != super::alc245::ALC245_VENDOR_ID {
            return Err(Error::UnsupportedTopology);
        }

        self.preflight_analog_route(
            graph,
            primary.route,
            primary.stream,
            primary.channel,
            primary.options,
        )?;

        if let Some(second) = secondary {
            self.preflight_analog_route(
                graph,
                second.route,
                second.stream,
                second.channel,
                second.options,
            )?;

            if second.route.pin() != 0x16
                || second.route.direction() != Direction::Playback
                || second.route.group_node() != 1
                || second.channel != 0
                || second.stream == primary.stream
                || primary
                    .route
                    .nodes()
                    .iter()
                    .any(|node| second.route.nodes().contains(node))
            {
                return Err(Error::RouteInUse);
            }
        }

        let pin_power = self.headphone_preflight(
            graph,
            primary.route,
            primary.stream,
            primary.channel,
            primary.options,
        )?;

        let (codec_id, revision_id, subsystem_id, vendor_power) =
            self.realtek_common_identity(graph)?;
        let identity = super::alc245::Alc245Identity::detect(codec_id, revision_id)
            .ok_or(Error::UnsupportedTopology)?;

        if codec_id != 0x10ec_0245 || super::alc245::board_profile(subsystem_id).is_none() {
            return Err(Error::UnsupportedTopology);
        }

        if self.realtek_parameter(graph.address, 0x51, 0)? != 0 {
            return Err(Error::UnsupportedTopology);
        }

        let (second_power, second_caps) = self.fixed_second_pin_preflight(graph)?;

        let result = (|| {
            self.power_d0(graph.address, 1)?;

            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }

            if pin_power {
                self.power_d0(graph.address, primary.route.pin())?;
            }

            if second_power && primary.route.pin() != 0x16 {
                self.power_d0(graph.address, 0x16)?;
            }

            let coefficient_zero = self.read_coefficient(graph.address, 0)?;

            if identity.silicon_variant(coefficient_zero)
                != super::alc245::Alc245SiliconVariant::Alc245
            {
                return Err(Error::UnsupportedTopology);
            }

            self.delay(30_000)?;

            let first = self.poll_pin_sense(graph, primary.route.pin())?;

            if second_caps & 2 != 0 {
                self.verb(Verb::short(graph.address, 0x16, 0x709, 0)?)?;
            }

            let raw = self.verb(Verb::short(graph.address, 0x16, 0xf09, 0)?)?;

            let code = raw & 0x7fff_ffff;

            let second = PinSense {
                raw,
                present: raw & (1 << 31) != 0,
                impedance_code: (second_caps & 1 != 0 && code != 0x7fff_ffff).then_some(code),
            };

            if second.present {
                let pin = graph.widget(0x16).ok_or(Error::InvalidCodecGraph)?;

                if second_caps & 0x14 != 0x14 || pin.channels() != 2 || pin.capabilities & 4 == 0 {
                    return Err(Error::UnsupportedTopology);
                }

                let amp = self.realtek_parameter(
                    graph.address,
                    if pin.capabilities & 8 != 0 { 0x16 } else { 1 },
                    0x12,
                )?;

                if amp == u32::MAX || pin.output_amplifier != Some(amp) {
                    return Err(Error::InvalidCodecGraph);
                }
            }

            if first.present || second.present {
                self.delay(2_000)?;
            }

            // Node 0x57, coefficient 0x04: low-power mode.
            self.depop_control(graph.address, DepopControl::HeadphonePower, 1)?;

            for (pin, present) in [(primary.route.pin(), first.present), (0x16, second.present)] {
                if present {
                    self.verb(Verb::long(graph.address, pin, 3, 0xb080)?)?;

                    for channel in [0xa000, 0x8000] {
                        if self.verb(Verb::long(graph.address, pin, 11, channel)?)? != 0x80 {
                            return Err(Error::RouteRejected);
                        }
                    }
                }
            }

            if first.present || second.present {
                self.delay(85_000)?;
            }

            if first.present {
                self.route_control(graph.address, primary.route.pin(), 0x707, 0xf07, 0x40)?;
            }

            if second.present {
                self.route_control(graph.address, 0x16, 0x707, 0xf07, 0x40)?;
            }

            if first.present || second.present {
                self.delay(100_000)?;
            }

            let mut coefficients = alloc::vec::Vec::new();

            coefficients.push(self.clear_coefficient_bits(graph.address, 0x4a, 3 << 10)?);

            // Node 0x57, coefficient 0x04: restore high-power mode.
            self.depop_control(graph.address, DepopControl::HeadphonePower, 4)?;

            self.configure_analog_route(
                graph,
                primary.route,
                primary.stream,
                primary.channel,
                primary.options,
            )?;

            if let Some(second) = secondary {
                self.configure_analog_route(
                    graph,
                    second.route,
                    second.stream,
                    second.channel,
                    second.options,
                )?;
            } else if !primary.route.nodes().contains(&0x16) {
                self.codec_controls
                    .push((primary.stream, graph.address, 0x16));
            }

            Ok(RealtekDualHeadphoneReport {
                common: RealtekCommonReport {
                    codec_id,
                    revision_id,
                    subsystem_id,
                    coefficients,
                },
                coefficient_zero: Some(coefficient_zero),
                boot_depop_performed: false,
                headphone_pins: [primary.route.pin(), 0x16],
                initial_senses: [first, second],
            })
        })();

        if result.is_err() {
            self.routing_failed = true;
        }

        result
    }
}
