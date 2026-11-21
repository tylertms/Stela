// SPDX-License-Identifier: GPL-2.0-only
//! Realtek analog ownership. Family initialization and 200ms EAPD
//! shutup adapt Linux realtek.c/alc662.c/alc882.c at
//! a804c5742bff224d4cb4d25dd383554d598be32b (GPL-2.0-or-later).
use super::{
    CoefficientChange, RealtekCommonReport,
    family::{self, RealtekFamily},
    oem::{self, RealtekOem},
};
use crate::{
    Error, Timer,
    hda::{
        AnalogRoute, CodecGraph, Controller, PinOptions, PinSense, Registers, StreamId, Verb,
        Widget, WidgetKind, routing::RoutePolicy,
    },
};
use alloc::vec::Vec;

#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekReport {
    pub common: RealtekCommonReport,
    pub family: RealtekFamily,
    pub oem: RealtekOem,
    pub coefficient_zero: u16,
    /// Source front/headphone EAPD endpoints found in the live graph.
    pub eapd_pins: Vec<u8>,
    /// Retained-controller PCI observation; never inferred from AFG f20.
    pub pci_subsystem: Option<u32>,
}

#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct AmplifierLevel {
    /// Hardware gain step, not a percentage or an inferred dB value.
    pub gain: u8,
    pub muted: bool,
}
#[derive(Clone, Debug, PartialEq, Eq)]
pub struct RealtekCaptureGain {
    pub input_index: u8,
    /// Inherited/overridden input amplifier capabilities.
    pub capabilities: u32,
    /// Left, then optional right amplifier channel. A mono mixer with
    /// one stereo source has stereo input amplifiers, as in the Linux path.
    pub levels: Vec<AmplifierLevel>,
}

#[derive(Clone)]
pub(in crate::hda) struct State {
    address: u8,
    graph: CodecGraph,
    subsystem: u32,
    pci_subsystem: Option<u32>,
    oem: RealtekOem,
    coefficients: Vec<CoefficientChange>,
    eapd_pins: Vec<u8>,
    laptop: Option<(u8, u16)>,
    hidden: Vec<(u8, u16, u16)>,
    active: bool,
    quiet: bool,
    authority_lost: bool,
}

fn same_graph(a: &CodecGraph, b: &CodecGraph) -> bool {
    a.token == b.token
        && a.address == b.address
        && a.vendor_id == b.vendor_id
        && a.revision_id == b.revision_id
        && a.groups.len() == b.groups.len()
        && a.groups.iter().zip(&b.groups).all(|(a, b)| {
            a.node == b.node
                && a.function_type == b.function_type
                && a.power_states == b.power_states
                && a.pcm.rates_and_bits == b.pcm.rates_and_bits
                && a.pcm.stream_formats == b.pcm.stream_formats
                && a.widgets.len() == b.widgets.len()
                && a.widgets.iter().zip(&b.widgets).all(|(a, b)| {
                    a.node == b.node
                        && a.kind == b.kind
                        && a.capabilities == b.capabilities
                        && a.connections == b.connections
                        && a.input_amplifier == b.input_amplifier
                        && a.output_amplifier == b.output_amplifier
                        && a.power_states == b.power_states
                        && a.pin_capabilities == b.pin_capabilities
                        && a.pin_default == b.pin_default
                        && a.pcm.rates_and_bits == b.pcm.rates_and_bits
                        && a.pcm.stream_formats == b.pcm.stream_formats
                })
        })
}

impl<R: Registers, T: Timer> Controller<R, T> {
    pub(in crate::hda) fn manages_realtek_codec(&self, address: u8) -> bool {
        self.realtek_codecs
            .iter()
            .any(|state| state.address == address)
    }
    fn realtek_guard(&self, graph: &CodecGraph) -> Result<(), Error> {
        self.minimum_guard()?;
        if graph.token != self.token {
            return Err(Error::InvalidCodecGraph);
        }
        if self.stopped {
            return Err(Error::Shutdown);
        }
        if self.routing_failed || self.lifecycle_failed || self.command_poisoned {
            return Err(Error::NeedsReset);
        }
        Ok(())
    }
    fn realtek_graph_authority(&mut self, graph: &CodecGraph) -> Result<(), Error> {
        self.minimum_guard()?;
        // Full live re-enumeration prevents public cached defaults, amp, power,
        // PCM or connection fields from manufacturing electrical permission.
        if !same_graph(graph, &self.enumerate_codec(graph.address)?) {
            return Err(Error::InvalidCodecGraph);
        }
        Ok(())
    }
    fn realtek_hidden_read(&mut self, address: u8, node: u8, index: u16) -> Result<u16, Error> {
        self.minimum_guard()?;
        self.verb(Verb::long(address, node, 5, index)?)?;
        if self.verb(Verb::long(address, node, 13, 0)?)? != u32::from(index) {
            return Err(Error::CoefficientRejected { node, index });
        }
        u16::try_from(self.verb(Verb::long(address, node, 12, 0)?)?)
            .map_err(|_| Error::CoefficientRejected { node, index })
    }
    fn realtek_active(&mut self, graph: &CodecGraph) -> Result<State, Error> {
        self.minimum_guard()?;
        self.realtek_guard(graph)?;
        let state = self
            .realtek_codecs
            .iter()
            .find(|s| s.address == graph.address)
            .cloned()
            .ok_or(Error::NeedsReset)?;
        if !state.active || state.quiet || state.authority_lost {
            return Err(Error::NeedsReset);
        }
        let result = (|| {
            if !same_graph(graph, &state.graph) {
                return Err(Error::InvalidCodecGraph);
            }
            self.realtek_graph_authority(graph)?;
            let (_, _, subsystem, _) = self.realtek_common_identity(graph)?;
            if subsystem != state.subsystem
                || (state.oem != RealtekOem::Firmware
                    && self.registers.codec_pci_subsystem()? != state.pci_subsystem)
            {
                return Err(Error::InvalidCodecGraph);
            }
            self.realtek_oem_preflight(graph, state.oem)?;
            for &(node, index, value) in &state.hidden {
                if self.realtek_hidden_read(graph.address, node, index)? != value {
                    return Err(Error::CoefficientRejected { node, index });
                }
            }
            for change in &state.coefficients {
                if self.read_coefficient(graph.address, change.index)? != change.current {
                    return Err(Error::CoefficientRejected {
                        node: 0x20,
                        index: change.index,
                    });
                }
            }
            Ok(())
        })();
        if let Err(error) = result {
            self.routing_failed = true;
            if let Some(owned) = self
                .realtek_codecs
                .iter_mut()
                .find(|s| s.address == graph.address)
            {
                owned.authority_lost = true;
            }
            return Err(error);
        }
        Ok(state)
    }
    /// Whether the controller's original PCI subsystem selects an OEM
    /// procedure for `graph`, as Linux's fixup table does.
    pub(in crate::hda) fn realtek_oem_selected(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<bool, Error> {
        self.minimum_guard()?;
        Ok(self
            .registers
            .codec_pci_subsystem()?
            .is_some_and(|subsystem| super::oem::select(graph.vendor_id, subsystem).is_ok()))
    }
    /// Cold source-checked ALC882/ALC662-family initialization and kept
    /// held-D0 ownership. Every GPIO/companion-amp policy remains unselected.
    /// Unknown boards use only their firmware analog routes; this is
    /// not a claim that their speakers or headset wiring are ready.
    pub fn initialize_realtek_codec(&mut self, graph: &CodecGraph) -> Result<RealtekReport, Error> {
        self.minimum_guard()?;
        self.initialize_realtek_owned(graph, false)
    }
    /// Select only an original PCI-subsystem-bound ALC1220 X570/P950 procedure.
    /// AFG subsystem IDs and caller assertions cannot select these fixups.
    /// The donor's fixed 0c connection must also be present on real silicon.
    pub fn initialize_realtek_oem_codec(
        &mut self,
        graph: &CodecGraph,
    ) -> Result<RealtekReport, Error> {
        self.minimum_guard()?;
        self.initialize_realtek_owned(graph, true)
    }
    fn initialize_realtek_owned(
        &mut self,
        graph: &CodecGraph,
        require_oem: bool,
    ) -> Result<RealtekReport, Error> {
        self.minimum_guard()?;
        self.realtek_guard(graph)?;
        self.realtek_common_guard(graph)?;
        let family = family::family(graph.vendor_id, graph.revision_id)?;
        self.realtek_graph_authority(graph)?;
        let (_, _, subsystem, vendor_power) = self.realtek_common_identity(graph)?;
        if subsystem == u32::MAX {
            return Err(Error::InvalidCodecGraph);
        }
        let pci_subsystem = if require_oem {
            self.registers.codec_pci_subsystem()?
        } else {
            None
        };
        let oem = if require_oem {
            oem::select(
                graph.vendor_id,
                pci_subsystem.ok_or(Error::UnsupportedTopology)?,
            )?
        } else {
            RealtekOem::Firmware
        };
        self.realtek_oem_preflight(graph, oem)?;
        let eapd_pins: Vec<_> = [0x0f, 0x10, 0x14, 0x15, 0x17]
            .into_iter()
            .filter(|node| {
                graph
                    .widget(*node)
                    .is_some_and(|w| w.pin_capabilities.is_some_and(|caps| caps & (1 << 16) != 0))
            })
            .collect();
        let index = self.realtek_codecs.len();
        // Keep ownership before the first power/index/data mutation. Partial
        // failure never lets a different backend replace this codec address.
        self.realtek_codecs.push(State {
            address: graph.address,
            graph: graph.clone(),
            subsystem,
            pci_subsystem,
            oem,
            coefficients: Vec::new(),
            eapd_pins: eapd_pins.clone(),
            laptop: None,
            hidden: Vec::new(),
            active: false,
            quiet: false,
            authority_lost: false,
        });
        let result = (|| {
            self.power_d0(graph.address, 1)?;
            if vendor_power {
                self.power_d0(graph.address, 0x20)?;
            }
            let zero = self.read_coefficient(graph.address, 0)?;
            let mut coefficients =
                self.realtek_family_coefficients(graph.address, graph.vendor_id, family, zero)?;
            for pin in &eapd_pins {
                self.route_control(graph.address, *pin, 0x70c, 0xf0c, 2)?;
            }
            coefficients.extend(self.realtek_default_amplifier_coefficient(
                graph.address,
                graph.vendor_id,
                zero,
            )?);
            coefficients.extend(self.realtek_oem_coefficients(graph.address, oem)?);
            Ok(RealtekReport {
                common: RealtekCommonReport {
                    codec_id: graph.vendor_id,
                    revision_id: graph.revision_id,
                    subsystem_id: subsystem,
                    coefficients,
                },
                family,
                oem,
                coefficient_zero: zero,
                eapd_pins,
                pci_subsystem,
            })
        })();
        match &result {
            Ok(report) => {
                // Reports preserve transaction order; active validation keeps
                // the final observed value when the same index occurs twice.
                let state = &mut self.realtek_codecs[index];
                for change in &report.common.coefficients {
                    if let Some(old) = state
                        .coefficients
                        .iter_mut()
                        .find(|c| c.index == change.index)
                    {
                        *old = *change;
                    } else {
                        state.coefficients.push(*change);
                    }
                }
                state.active = true;
            }
            Err(_) => self.routing_failed = true,
        }
        result
    }
    /// Compose the source-checked existing startup procedures with kept
    /// ownership and the matching timed shutdown. ALC269's variant procedure
    /// uses its exact init hook rather than the default 30ms headphone hook.
    /// Optional pin16 PCM is accepted only by ALC285/295. No ULP, headset keys,
    /// CTIA/OMTP, OEM inversion or companion amplifier is selected.
    pub fn configure_realtek_laptop_headphones(
        &mut self,
        graph: &CodecGraph,
        primary: super::RealtekHeadphoneRoute<'_>,
        secondary: Option<super::RealtekHeadphoneRoute<'_>>,
    ) -> Result<RealtekReport, Error> {
        self.minimum_guard()?;
        self.realtek_guard(graph)?;
        self.realtek_common_guard(graph)?;
        if !matches!(
            graph.vendor_id,
            0x10ec_0269
                | 0x10ec_0255
                | 0x10ec_0256
                | 0x10ec_0257
                | 0x10ec_0285
                | 0x10ec_0295
                | 0x10ec_0298
        ) || (secondary.is_some() && !matches!(graph.vendor_id, 0x10ec_0285 | 0x10ec_0295))
        {
            return Err(Error::UnsupportedTopology);
        }
        self.headphone_preflight(
            graph,
            primary.route,
            primary.stream,
            primary.channel,
            primary.options,
        )?;
        self.realtek_graph_authority(graph)?;
        let (_, _, subsystem, _) = self.realtek_common_identity(graph)?;
        if subsystem == u32::MAX {
            return Err(Error::InvalidCodecGraph);
        }
        let eapd_pins: Vec<_> = [0x0f, 0x10, 0x14, 0x15, 0x17]
            .into_iter()
            .filter(|node| {
                graph
                    .widget(*node)
                    .is_some_and(|w| w.pin_capabilities.is_some_and(|caps| caps & (1 << 16) != 0))
            })
            .collect();
        if matches!(graph.vendor_id, 0x10ec_0285 | 0x10ec_0295) {
            let second = graph.widget(0x16).ok_or(Error::UnsupportedTopology)?;
            if second.channels() != 2
                || second
                    .output_amplifier
                    .is_none_or(|caps| caps == u32::MAX || caps & (1 << 31) == 0)
            {
                return Err(Error::UnsupportedTopology);
            }
        }
        // This local owner exists before startup mutates hardware. Exclusive
        // &mut self prevents interleaving while the earlier startup routines
        // reserve their paths. It is inserted on success AND failure.
        let mut state = State {
            address: graph.address,
            graph: graph.clone(),
            subsystem,
            pci_subsystem: None,
            oem: RealtekOem::Firmware,
            coefficients: Vec::new(),
            eapd_pins: eapd_pins.clone(),
            laptop: Some((primary.route.pin(), 0)),
            hidden: Vec::new(),
            active: false,
            quiet: false,
            authority_lost: false,
        };
        let result = (|| {
            let common = match graph.vendor_id {
                0x10ec_0245 => {
                    self.configure_realtek_alc245_headphones(graph, primary, secondary)?
                        .common
                }
                0x10ec_0269 => {
                    let report = self.initialize_realtek_alc269_common(graph)?;
                    self.configure_analog_route(
                        graph,
                        primary.route,
                        primary.stream,
                        primary.channel,
                        primary.options,
                    )?;
                    report.common
                }
                0x10ec_0255 => {
                    self.configure_realtek_alc255_headphone(
                        graph,
                        primary.route,
                        primary.stream,
                        primary.channel,
                        primary.options,
                    )?
                    .common
                }
                0x10ec_0256 | 0x10ec_0257 => {
                    self.configure_realtek_alc256_headphone(
                        graph,
                        primary.route,
                        primary.stream,
                        primary.channel,
                        primary.options,
                    )?
                    .common
                }
                0x10ec_0298 => {
                    self.configure_realtek_alc298_headphone(
                        graph,
                        primary.route,
                        primary.stream,
                        primary.channel,
                        primary.options,
                    )?
                    .common
                }
                _ => {
                    self.configure_realtek_alc285_295_headphones(graph, primary, secondary)?
                        .common
                }
            };
            for pin in &eapd_pins {
                self.route_control(graph.address, *pin, 0x70c, 0xf0c, 2)?;
            }
            let zero = self.read_coefficient(graph.address, 0)?;
            if matches!(
                graph.vendor_id,
                0x10ec_0255 | 0x10ec_0256 | 0x10ec_0257 | 0x10ec_0285 | 0x10ec_0295
            ) {
                let value = self.realtek_hidden_read(graph.address, 0x57, 4)?;
                state.hidden.push((0x57, 4, value));
            }
            if matches!(graph.vendor_id, 0x10ec_0255..=0x10ec_0257) {
                let value = self.realtek_hidden_read(graph.address, 0x53, 2)?;
                state.hidden.push((0x53, 2, value));
            }
            if matches!(graph.vendor_id, 0x10ec_0285 | 0x10ec_0295) {
                for index in [0, 3, 0x0f] {
                    let value = self.realtek_hidden_read(graph.address, 0x58, index)?;
                    state.hidden.push((0x58, index, value));
                }
            }
            Ok(RealtekReport {
                common,
                family: if graph.vendor_id == 0x10ec_0245 {
                    RealtekFamily::Alc245
                } else {
                    RealtekFamily::Alc269
                },
                oem: RealtekOem::Firmware,
                coefficient_zero: zero,
                eapd_pins,
                pci_subsystem: None,
            })
        })();
        if let Ok(report) = &result {
            state.laptop = Some((primary.route.pin(), report.coefficient_zero));
            for change in &report.common.coefficients {
                if let Some(old) = state
                    .coefficients
                    .iter_mut()
                    .find(|c| c.index == change.index)
                {
                    *old = *change;
                } else {
                    state.coefficients.push(*change);
                }
            }
            state.active = true;
        } else {
            self.routing_failed = true;
        }
        self.realtek_codecs.push(state);
        result
    }
    pub fn configure_realtek_route(
        &mut self,
        graph: &CodecGraph,
        route: &AnalogRoute,
        id: StreamId,
        channel: u8,
        options: PinOptions,
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        self.realtek_guard(graph)?;
        self.preflight_analog_route(graph, route, id, channel, options)?;
        let state = self.realtek_active(graph)?;
        oem::validate_route(state.oem, route)?;
        self.configure_analog_route_policy(
            &state.graph,
            route,
            id,
            channel,
            options,
            // alc_alloc_spec sets single_adc_amp for every managed family.
            // The selector still uses the route connection, but ADC gain is
            // always input amplifier zero.
            RoutePolicy::SingleAdc,
        )
    }
    pub fn poll_realtek_pin_sense(
        &mut self,
        graph: &CodecGraph,
        pin: u8,
    ) -> Result<PinSense, Error> {
        self.minimum_guard()?;
        self.realtek_active(graph)?;
        let result = self.poll_pin_sense_policy(graph, pin, true);
        if result
            .as_ref()
            .is_err_and(|e| *e != Error::UnsupportedTopology && *e != Error::InvalidCodecGraph)
        {
            self.routing_failed = true;
        }
        result
    }
    /// Change a reserved playback-path output amplifier, including
    /// while PCM runs. Gain is an advertised hardware step, not a percentage.
    /// Both stereo channels and every write readback are checked; unsupported
    /// hardware mute never becomes an invented mute bit or discarded effect.
    pub fn set_realtek_output_gain(
        &mut self,
        graph: &CodecGraph,
        node: u8,
        gain: u8,
        muted: bool,
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        let state = self.realtek_active(graph)?;
        if !self.routes.iter().any(|(_, route)| {
            route.codec_address() == graph.address
                && route.direction() == crate::Direction::Playback
                && route.nodes().contains(&node)
        }) {
            return Err(Error::RouteInUse);
        }
        let widget = state.graph.widget(node).ok_or(Error::InvalidCodecGraph)?;
        let caps = widget.output_amplifier.ok_or(Error::UnsupportedTopology)?;
        let steps = ((caps >> 8) & 0x7f) as u8;
        if caps == u32::MAX || widget.channels() > 2 || (muted && caps & (1 << 31) == 0) {
            return Err(Error::UnsupportedTopology);
        }
        if gain > steps {
            return Err(Error::InvalidConfiguration);
        }
        let value = u16::from(gain) | if muted { 0x80 } else { 0 };
        let channels = if widget.channels() == 2 {
            0x3000
        } else {
            0x2000
        };
        let result = (|| {
            self.verb(Verb::long(
                graph.address,
                node,
                3,
                0x8000 | channels | value,
            )?)?;
            for channel in 0..widget.channels() {
                let get = if channel == 0 { 0xa000 } else { 0x8000 };
                let actual = self.verb(Verb::long(graph.address, node, 11, get)?)?;
                if actual == u32::MAX || actual & 0xff != u32::from(value) {
                    return Err(Error::RouteRejected);
                }
            }
            Ok(())
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
    fn realtek_capture_amplifier(
        &mut self,
        graph: &CodecGraph,
        stream: StreamId,
        node: u8,
    ) -> Result<(Widget, u8, u8), Error> {
        self.minimum_guard()?;
        self.realtek_guard(graph)?;
        // An old/foreign stream or unreserved node must not trigger even the
        // live codec-permission reads, much less change an amplifier.
        if self.stream_direction(stream)? != crate::Direction::Capture {
            return Err(Error::InvalidStream);
        }
        let route = self
            .routes
            .iter()
            .find(|(owner, route)| {
                *owner == stream
                    && route.codec_address() == graph.address
                    && route.direction() == crate::Direction::Capture
                    && route.nodes().contains(&node)
            })
            .map(|(_, route)| route.clone())
            .ok_or(Error::RouteInUse)?;
        let state = self.realtek_active(graph)?;
        let widget = state
            .graph
            .widget(node)
            .cloned()
            .ok_or(Error::InvalidCodecGraph)?;
        let caps = widget.input_amplifier.ok_or(Error::UnsupportedTopology)?;
        if caps == u32::MAX
            || widget.channels() > 2
            || !matches!(
                widget.kind,
                WidgetKind::Input | WidgetKind::Mixer | WidgetKind::Selector | WidgetKind::Pin
            )
        {
            return Err(Error::UnsupportedTopology);
        }
        let offset = route.nodes().iter().position(|n| *n == node).unwrap();
        let index = if let Some(source) = route.nodes().get(offset + 1) {
            widget
                .connections
                .iter()
                .position(|n| n == source)
                .ok_or(Error::InvalidCodecGraph)?
        } else {
            0
        };
        if index > 15 {
            return Err(Error::UnsupportedTopology);
        }
        // A selector that no longer names this path is a lost routing
        // epoch. Never adjust an inactive source and call it capture volume.
        if offset + 1 < route.nodes().len()
            && widget.kind != WidgetKind::Mixer
            && widget.connections.len() > 1
        {
            let selected = self.verb(Verb::short(graph.address, node, 0xf01, 0)?)?;
            if selected != index as u32 {
                self.routing_failed = true;
                return Err(Error::RouteRejected);
            }
        }
        let channels = if widget.channels() == 1
            && widget.kind == WidgetKind::Mixer
            && widget.connections.len() == 1
            && state
                .graph
                .widget(widget.connections[0])
                .is_some_and(|source| source.capabilities & 1 != 0)
        {
            2
        } else {
            widget.channels()
        };
        let amplifier_index = if widget.kind == WidgetKind::Input {
            0
        } else {
            index as u8
        };
        Ok((widget, amplifier_index, channels))
    }
    /// Read the selected capture-path amplifier, including while PCM
    /// runs. Only an initialized live codec and this stream's reserved input
    /// path authorize the index. Other mixer inputs remain isolated.
    pub fn realtek_capture_gain(
        &mut self,
        graph: &CodecGraph,
        stream: StreamId,
        node: u8,
    ) -> Result<RealtekCaptureGain, Error> {
        self.minimum_guard()?;
        let (widget, index, channels) = self.realtek_capture_amplifier(graph, stream, node)?;
        let caps = widget.input_amplifier.unwrap();
        let result = (|| {
            let mut levels = Vec::with_capacity(usize::from(channels));
            for channel in 0..channels {
                let actual = self.verb(Verb::long(
                    graph.address,
                    node,
                    11,
                    if channel == 0 { 0x2000 } else { 0 } | u16::from(index),
                )?)?;
                if actual > 255
                    || (actual & 0x7f) > ((caps >> 8) & 0x7f)
                    || (actual & 0x80 != 0 && caps & (1 << 31) == 0)
                {
                    return Err(Error::RouteRejected);
                }
                levels.push(AmplifierLevel {
                    gain: (actual & 0x7f) as u8,
                    muted: actual & 0x80 != 0,
                });
            }
            Ok(RealtekCaptureGain {
                input_index: index,
                capabilities: caps,
                levels,
            })
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
    /// Set a reserved input amplifier's gain/mute on both channels and
    /// acknowledge each channel independently. Unsupported mute never becomes
    /// an invented bit or a silent minimum-gain substitute.
    pub fn set_realtek_capture_gain(
        &mut self,
        graph: &CodecGraph,
        stream: StreamId,
        node: u8,
        gain: u8,
        muted: bool,
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        let (widget, index, channels) = self.realtek_capture_amplifier(graph, stream, node)?;
        let caps = widget.input_amplifier.unwrap();
        if gain > ((caps >> 8) & 0x7f) as u8 {
            return Err(Error::InvalidConfiguration);
        }
        if muted && caps & (1 << 31) == 0 {
            return Err(Error::UnsupportedTopology);
        }
        let value = u16::from(gain) | if muted { 0x80 } else { 0 };
        let channel_bits = if channels == 2 { 0x3000 } else { 0x2000 };
        let result = (|| {
            self.verb(Verb::long(
                graph.address,
                node,
                3,
                0x4000 | channel_bits | u16::from(index) << 8 | value,
            )?)?;
            for channel in 0..channels {
                let actual = self.verb(Verb::long(
                    graph.address,
                    node,
                    11,
                    if channel == 0 { 0x2000 } else { 0 } | u16::from(index),
                )?)?;
                if actual != u32::from(value) {
                    return Err(Error::RouteRejected);
                }
            }
            Ok(())
        })();
        if result.is_err() {
            self.routing_failed = true;
        }
        result
    }
    pub(in crate::hda) fn shutdown_realtek_codecs(&mut self) -> Result<(), Error> {
        self.minimum_guard()?;
        for index in 0..self.realtek_codecs.len() {
            let state = self.realtek_codecs[index].clone();
            if state.quiet {
                continue;
            }
            if state.authority_lost || self.command_poisoned || !state.active {
                return Err(Error::NeedsReset);
            }
            // Do not drive an address whose silicon/board identity changed.
            self.realtek_graph_authority(&state.graph)?;
            let (_, _, subsystem, _) = self.realtek_common_identity(&state.graph)?;
            if subsystem != state.subsystem
                || (state.oem != RealtekOem::Firmware
                    && self.registers.codec_pci_subsystem()? != state.pci_subsystem)
            {
                return Err(Error::InvalidCodecGraph);
            }
            if let Some((primary, zero)) = state.laptop {
                self.realtek_laptop_shutup(&state.graph, primary, zero, &state.eapd_pins)?;
                self.realtek_codecs[index].active = false;
                self.realtek_codecs[index].quiet = true;
                continue;
            }
            for pin in &state.eapd_pins {
                self.route_control(state.address, *pin, 0x70c, 0xf0c, 0)?;
            }
            if family::family(state.graph.vendor_id, state.graph.revision_id)?
                == RealtekFamily::Alc662
            {
                self.delay(200_000)?;
            }
            for widget in state.graph.groups.iter().flat_map(|group| &group.widgets) {
                if widget.pin_capabilities.is_some() {
                    self.route_control(state.address, widget.node, 0x707, 0xf07, 0)?;
                }
            }
            self.realtek_codecs[index].active = false;
            self.realtek_codecs[index].quiet = true;
        }
        Ok(())
    }
}
