// SPDX-License-Identifier: GPL-2.0-only
//! Permanent-shutdown (bus.shutdown) Realtek laptop depop from Linux
//! alc269.c::{alc269_shutup,alc256_shutup,alc225_shutup,alc_default_shutup}
//! and realtek.c::alc_shutup_pins at a804c5742bff224d4cb4d25dd383554d598be32b.
//! GPL-2.0-or-later donor, combined adaptation GPL-2.0-only.
use super::headphone::DepopControl;
use crate::{
    Error, Timer,
    hda::{CodecGraph, Controller, Registers, Verb},
};

impl<R: Registers, T: Timer> Controller<R, T> {
    fn realtek_shutdown_mute(&mut self, address: u8, pin: u8) -> Result<(), Error> {
        self.minimum_guard()?;
        self.verb(Verb::long(address, pin, 3, 0xb080)?)?;
        for channel in [0xa000, 0x8000] {
            if self.verb(Verb::long(address, pin, 11, channel)?)? != 0x80 {
                return Err(Error::RouteRejected);
            }
        }
        Ok(())
    }
    pub(super) fn realtek_laptop_shutup(
        &mut self,
        graph: &CodecGraph,
        primary: u8,
        zero: u16,
        eapd: &[u8],
    ) -> Result<(), Error> {
        self.minimum_guard()?;
        let address = graph.address;
        let id = graph.vendor_id;
        if id == 0x10ec_0269 {
            if zero & 0xf0 == 0x10 {
                self.clear_coefficient_bits(address, 4, 1 << 11)?;
                if zero & 255 == 0x18 {
                    self.delay(150_000)?;
                }
            }
        } else if matches!(id, 0x10ec_0285 | 0x10ec_0295) {
            let first = self.poll_pin_sense_policy(graph, primary, true)?.present;
            let caps = graph
                .widget(0x16)
                .and_then(|w| w.pin_capabilities)
                .ok_or(Error::InvalidCodecGraph)?;
            if caps & 2 != 0 {
                self.verb(Verb::short(address, 0x16, 0x709, 0)?)?;
            }
            let second = self.verb(Verb::short(address, 0x16, 0xf09, 0)?)? & (1 << 31) != 0;
            if first || second {
                // This profile never enables headset-key unsolicited controls;
                // the donor's has_hs_key conditional therefore has no effects.
                self.update_coefficient(address, 0x4a, 0, 3 << 10)?;
                self.delay(2_000)?;
                for (pin, present) in [(primary, first), (0x16, second)] {
                    if present {
                        self.realtek_shutdown_mute(address, pin)?;
                    }
                }
                self.delay(75_000)?;
                for (pin, present) in [(primary, first), (0x16, second)] {
                    if present {
                        self.route_control(address, pin, 0x707, 0xf07, 0)?;
                    }
                }
                self.delay(75_000)?;
                self.clear_coefficient_bits(address, 0x4a, 3 << 10)?;
            }
        } else if id == 0x10ec_0245 {
            let first = self.poll_pin_sense_policy(graph, primary, true)?.present;
            let second = graph
                .widget(0x16)
                .and_then(|widget| widget.pin_capabilities)
                .is_some()
                && self.poll_pin_sense_policy(graph, 0x16, true)?.present;

            if first || second {
                self.update_coefficient(address, 0x4a, 0, 3 << 10)?;
                self.delay(2_000)?;
                if first {
                    self.realtek_shutdown_mute(address, primary)?;
                }
                if second {
                    self.realtek_shutdown_mute(address, 0x16)?;
                }
                self.delay(75_000)?;
                if first {
                    self.route_control(address, primary, 0x707, 0xf07, 0)?;
                }
                if second {
                    self.route_control(address, 0x16, 0x707, 0xf07, 0)?;
                }
                self.delay(75_000)?;
                self.clear_coefficient_bits(address, 0x4a, 3 << 10)?;
            }
        } else {
            if matches!(id, 0x10ec_0255..=0x10ec_0257) {
                self.depop_control(address, DepopControl::HeadphonePower, 1)?;
                // Source en_3kpull_low defaults true; ALC257 explicitly clears
                // it to keep cold-boot headset microphone detection.
                if id != 0x10ec_0257 {
                    self.update_coefficient(address, 0x46, 0, 3 << 12)?;
                }
            }
            if self.poll_pin_sense_policy(graph, primary, true)?.present {
                self.delay(2_000)?;
                self.realtek_shutdown_mute(address, primary)?;
                self.delay(75_000)?;
                self.route_control(address, primary, 0x707, 0xf07, 0)?;
                self.delay(75_000)?;
            }
        }
        if id != 0x10ec_0269 {
            for pin in eapd {
                self.route_control(address, *pin, 0x70c, 0xf0c, 0)?;
            }
        }
        // alc_headset_mic_no_shutup returns early when bus.shutdown is true.
        // Keep microphone/jack defaults on these codecs during permanent
        // shutdown. Primary headphone mute/clear above remains mandatory.
        if matches!(id, 0x10ec_0269 | 0x10ec_0255) {
            for widget in graph.groups.iter().flat_map(|group| &group.widgets) {
                if widget.pin_capabilities.is_some() {
                    self.route_control(address, widget.node, 0x707, 0xf07, 0)?;
                }
            }
        }
        Ok(())
    }
}
