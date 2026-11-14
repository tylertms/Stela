//! Host keyboards and pointers, plus SFH sensors ([`sfh`]). Pointer/keyboard
//! changes are read as [`Input`] changes for the native
//! USB peripheral: virtio-input ([`virtio`]) and i8042 PS/2 ([`ps2`]). USB HID devices are in
//! [`usb::hid`](crate::usb::hid); ELAN and Cypress touchpads read as a mouse in
//! [`touchpad`]. [`InputDevice`] polls any of them.
pub mod cyapa;
mod device;
pub mod elan;
mod held;
pub mod hid_i2c;
pub mod ish;
pub mod observation;
pub mod ps2;
pub mod rmi;
pub mod sfh;
pub mod touchpad;
pub mod virtio;

pub use device::{InputDevice, InputError};
pub use held::HeldInputs;

/// What masking an I2C touchpad's interrupt after a failed operation
/// achieved, as Cypress and Elan drivers report with their errors.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum TouchpadRecovery {
    NotAttempted,
    IrqMasked,
    MaskFailed,
}

impl TouchpadRecovery {
    /// Mask and drain `interrupt` after a failure.
    pub fn mask(interrupt: &mut impl hid_i2c::Interrupt) -> Self {
        if interrupt.mask_and_drain().is_ok() {
            Self::IrqMasked
        } else {
            Self::MaskFailed
        }
    }
}

/// The largest absolute pointer coordinate on either axis.
pub const POINTER_MAX: u16 = 0x7fff;

/// One host input change, in HID usages, the pointer's absolute range and
/// the mouse's counts.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Input {
    /// A Keyboard/Keypad page usage: 04h..DDh, or a modifier E0h..E7h.
    Key { usage: u8, pressed: bool },
    /// The pointer's position, each axis 0..=[`POINTER_MAX`].
    Move { x: u16, y: u16 },
    /// The pointer's buttons 1..3 held, as bits 0..2.
    Buttons(u8),
    /// Pointer wheel detents, positive away from the user.
    Wheel(i8),
    /// Pointer horizontal wheel (AC Pan) detents, positive right.
    HWheel(i8),
    /// Mouse motion in counts, positive right and down.
    Motion { dx: i16, dy: i16 },
    /// Mouse buttons 1..5 held, as bits 0..4: left, right, middle, side,
    /// extra. Consumers with a narrower descriptor must refuse extra buttons.
    MouseButtons(u8),
    /// Mouse wheel detents, positive away from the user.
    MouseWheel(i8),
    /// Mouse horizontal wheel (AC Pan) detents, positive right.
    MouseHWheel(i8),
}

#[cfg(test)]
mod tests;
