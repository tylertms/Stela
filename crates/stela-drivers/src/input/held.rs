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

    /// Whether a device other than the one at `index` holds key `usage`.
    fn others_hold(&self, index: usize, usage: u8) -> bool {
        let (word, bit) = Held::key(usage);
        self.devices
            .iter()
            .enumerate()
            .any(|(other, (_, held))| other != index && held.keys[word] & bit != 0)
    }

    /// The pointer's and the mouse's buttons all devices hold.
    fn buttons(&self) -> (u8, u8) {
        self.devices
            .iter()
            .fold((0, 0), |(buttons, mouse), (_, held)| {
                (buttons | held.buttons, mouse | held.mouse_buttons)
            })
    }

    /// Device `device` gave `inputs[from..]`: note what it holds, and make
    /// those inputs what the guest's one keyboard and pointer should see of
    /// all devices together, as Linux and macOS keep each device's state
    /// and join them: a button mask is every device's buttons, and a key
    /// another device holds neither goes down again nor comes up. With no
    /// other device holding anything they stay as they are.
    pub fn polled(&mut self, device: u64, inputs: &mut Vec<Input>, from: usize) {
        self.held(device);
        let index = self
            .devices
            .iter()
            .position(|(id, _)| *id == device)
            .expect("the device was just added");
        let alone = self
            .devices
            .iter()
            .enumerate()
            .all(|(other, (_, held))| other == index || *held == Held::default());
        if alone {
            let held = &mut self.devices[index].1;
            for &input in &inputs[from..] {
                held.note(input);
            }
            return;
        }
        let gave: Vec<Input> = inputs.drain(from..).collect();
        for input in gave {
            self.devices[index].1.note(input);
            match input {
                Input::Key { usage, .. } if self.others_hold(index, usage) => {}
                Input::Buttons(_) => inputs.push(Input::Buttons(self.buttons().0)),
                Input::MouseButtons(_) => inputs.push(Input::MouseButtons(self.buttons().1)),
                input => inputs.push(input),
            }
        }
    }

    /// Device `device` stopped: append to `out` a release of every key it
    /// held that no other device holds, and the button masks the other
    /// devices leave, and forget it. Once for each device; a device that
    /// held nothing adds nothing.
    pub fn stopped(&mut self, device: u64, out: &mut Vec<Input>) {
        let Some(index) = self.devices.iter().position(|(id, _)| *id == device) else {
            return;
        };
        let (_, mut held) = self.devices.remove(index);
        for (word, keys) in held.keys.iter_mut().enumerate() {
            for (_, other) in &self.devices {
                *keys &= !other.keys[word];
            }
        }
        let (buttons, mouse) = self.buttons();
        let mut releases = Vec::new();
        held.releases(&mut releases);
        out.extend(releases.into_iter().map(|release| match release {
            Input::Buttons(_) => Input::Buttons(buttons),
            Input::MouseButtons(_) => Input::MouseButtons(mouse),
            release => release,
        }));
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

    #[test]
    fn a_report_of_one_pointer_keeps_the_buttons_another_holds() {
        let mut held = HeldInputs::new();
        let mut mouse = alloc::vec![Input::MouseButtons(1)];
        held.polled(0, &mut mouse, 0);
        assert_eq!(mouse, [Input::MouseButtons(1)]);
        // The touchpad's report with no button keeps the mouse's drag.
        let mut touchpad = alloc::vec![Input::Motion { dx: 1, dy: 1 }, Input::MouseButtons(0)];
        held.polled(1, &mut touchpad, 0);
        assert_eq!(
            touchpad,
            [Input::Motion { dx: 1, dy: 1 }, Input::MouseButtons(1)]
        );
        // Its own button joins the mouse's.
        let mut touchpad = alloc::vec![Input::MouseButtons(2)];
        held.polled(1, &mut touchpad, 0);
        assert_eq!(touchpad, [Input::MouseButtons(3)]);
        // The mouse stops: the touchpad's button stays down.
        let mut out = Vec::new();
        held.stopped(0, &mut out);
        assert_eq!(out, [Input::MouseButtons(2)]);
    }

    #[test]
    fn a_key_two_keyboards_hold_comes_up_when_both_let_go() {
        let mut held = HeldInputs::new();
        let mut first = alloc::vec![key(0xe1, true)];
        held.polled(0, &mut first, 0);
        assert_eq!(first, [key(0xe1, true)]);
        let mut second = alloc::vec![key(0xe1, true), key(0x04, true)];
        held.polled(1, &mut second, 0);
        assert_eq!(second, [key(0x04, true)], "Shift is down already");
        let mut second = alloc::vec![key(0xe1, false)];
        held.polled(1, &mut second, 0);
        assert!(second.is_empty(), "the first keyboard still holds Shift");
        // The second keyboard stops: its A comes up, not Shift.
        let mut out = Vec::new();
        let mut first = alloc::vec![key(0x04, true)];
        held.polled(0, &mut first, 0);
        assert!(first.is_empty(), "A is down already");
        held.stopped(1, &mut out);
        assert!(out.is_empty(), "the first keyboard holds A and Shift");
        let mut first = alloc::vec![key(0xe1, false), key(0x04, false)];
        held.polled(0, &mut first, 0);
        assert_eq!(first, [key(0xe1, false), key(0x04, false)]);
    }

    #[test]
    fn inputs_kept_before_the_device_s_own_stay() {
        let mut held = HeldInputs::new();
        let mut inputs = alloc::vec![Input::MouseButtons(1)];
        held.polled(0, &mut inputs, 0);
        inputs.push(Input::MouseButtons(0));
        held.polled(1, &mut inputs, 1);
        assert_eq!(inputs, [Input::MouseButtons(1), Input::MouseButtons(1)]);
    }
}
