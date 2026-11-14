// SPDX-License-Identifier: GPL-2.0-only
//! What each host input device holds down, from the [`Input`] changes it
//! gave, whatever its transport: when a device stops (unplugged, a transfer
//! that failed past its retries, a controller reset), the guest gets a
//! release for every key and button it held, as Linux's input core sends
//! when a device goes away. Without them the guest's keyboard keeps the key
//! held and repeats it, and its pointer keeps a drag going.
use super::Input;
use alloc::vec::Vec;

/// The keys and buttons one device holds down.
#[derive(Clone, Copy, Debug, Default, PartialEq, Eq)]
struct Held {
    /// Keyboard/Keypad page usages, one bit each.
    keys: [u64; 4],
    /// The pointer's buttons ([`Input::Buttons`]) and the mouse's
    /// ([`Input::MouseButtons`]), as the device last reported them.
    buttons: u8,
    mouse_buttons: u8,
}

impl Held {
    fn key(usage: u8) -> (usize, u64) {
        (usize::from(usage / 64), 1 << (usage % 64))
    }

    fn note(&mut self, input: Input) {
        match input {
            Input::Key { usage, pressed } => {
                let (word, bit) = Self::key(usage);
                if pressed {
                    self.keys[word] |= bit;
                } else {
                    self.keys[word] &= !bit;
                }
            }
            Input::Buttons(buttons) => self.buttons = buttons,
            Input::MouseButtons(buttons) => self.mouse_buttons = buttons,
            _ => {}
        }
    }

    /// Releases for everything held, in usage order.
    fn releases(&self, out: &mut Vec<Input>) {
        for (word, &bits) in self.keys.iter().enumerate() {
            let mut left = bits;
            while left != 0 {
                let usage = (word * 64) as u8 + left.trailing_zeros() as u8;
                out.push(Input::Key {
                    usage,
                    pressed: false,
                });
                left &= left - 1;
            }
        }
        if self.buttons != 0 {
            out.push(Input::Buttons(0));
        }
        if self.mouse_buttons != 0 {
            out.push(Input::MouseButtons(0));
        }
    }
}

/// The held state of every host input device, by an identity the caller
/// gives each device for as long as it is read.
#[derive(Debug, Default)]
pub struct HeldInputs {
    devices: Vec<(u64, Held)>,
}

impl HeldInputs {
    pub const fn new() -> Self {
        Self {
            devices: Vec::new(),
        }
    }

    fn held(&mut self, device: u64) -> &mut Held {
        let index = match self.devices.iter().position(|(id, _)| *id == device) {
            Some(index) => index,
            None => {
                self.devices.push((device, Held::default()));
                self.devices.len() - 1
            }
        };
        &mut self.devices[index].1
    }

    /// Device `device` gave `inputs[from..]`: note what it holds.
    pub fn polled(&mut self, device: u64, inputs: &mut Vec<Input>, from: usize) {
        let held = self.held(device);
        for &input in &inputs[from..] {
            held.note(input);
        }
    }

    /// Device `device` stopped: append a release for every key and button
    /// it held to `out`, and forget it. Once for each device; a device that
    /// held nothing adds nothing.
    pub fn stopped(&mut self, device: u64, out: &mut Vec<Input>) {
        if let Some(index) = self.devices.iter().position(|(id, _)| *id == device) {
            let (_, held) = self.devices.remove(index);
            held.releases(out);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn key(usage: u8, pressed: bool) -> Input {
        Input::Key { usage, pressed }
    }

    #[test]
    fn a_device_that_stops_releases_what_it_held() {
        let mut held = HeldInputs::new();
        let mut inputs = alloc::vec![key(0x04, true), key(0xe1, true), key(0x05, true)];
        held.polled(0, &mut inputs, 0);
        inputs.push(key(0x05, false));
        held.polled(0, &mut inputs, 3);
        let mut out = Vec::new();
        held.stopped(0, &mut out);
        assert_eq!(out, [key(0x04, false), key(0xe1, false)]);
        // Once.
        out.clear();
        held.stopped(0, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn a_pointer_that_stops_lets_its_buttons_go() {
        let mut held = HeldInputs::new();
        let mut inputs = alloc::vec![
            Input::MouseButtons(1),
            Input::Motion { dx: 3, dy: 0 },
            Input::Buttons(2),
            Input::Move { x: 5, y: 5 },
        ];
        held.polled(7, &mut inputs, 0);
        let mut out = Vec::new();
        held.stopped(7, &mut out);
        assert_eq!(out, [Input::Buttons(0), Input::MouseButtons(0)]);
        // A pointer with nothing held releases nothing.
        let mut inputs = alloc::vec![Input::MouseButtons(1), Input::MouseButtons(0)];
        held.polled(8, &mut inputs, 0);
        out.clear();
        held.stopped(8, &mut out);
        assert!(out.is_empty());
    }

    #[test]
    fn devices_are_kept_apart() {
        let mut held = HeldInputs::new();
        let mut first = alloc::vec![key(0x04, true)];
        held.polled(1, &mut first, 0);
        let mut second = alloc::vec![key(0x1d, true)];
        held.polled(2, &mut second, 0);
        let mut out = Vec::new();
        held.stopped(2, &mut out);
        assert_eq!(out, [key(0x1d, false)]);
        out.clear();
        held.stopped(1, &mut out);
        assert_eq!(out, [key(0x04, false)]);
    }
}
