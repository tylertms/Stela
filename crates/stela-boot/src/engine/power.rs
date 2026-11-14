//! The host machine's power: the guest's restart and power-off become the
//! host's, through its ACPI fixed hardware (`stela_runtime::pc::power`),
//! and the host's power button becomes the guest's.
//!
//! Each transition first writes the stop code to the hypervisor exit port
//! under a hypervisor ([`stela_boot_core::exit_port`]), so a virtual
//! machine's run can end with that code; a PC without a hypervisor gets no
//! write, and the transition follows.
//!
//! - Restart: the FADT's reset register, then the 8042 keyboard
//!   controller's reset pulse (command 0xfe, the PC/AT reset line), then a
//!   triple fault, each given half a second.
//! - Power-off: the firmware's `\_PTS(5)` through its AML ([`super::aml`]),
//!   then the S5 soft-off sequence with the sleep types its `\_S5`
//!   evaluates, every GPE disabled and every wake status cleared first. A
//!   hardware-reduced platform, or one
//!   without S5, uses the firmware's `ResetSystem` instead, as Linux does,
//!   and S5 that does not take effect falls back to it. When neither takes
//!   effect, the mount says so and halts; it never reports a power-off it
//!   did not make.
//! - Power button: the fixed-feature button's PWRBTN_STS, polled between
//!   engine slices, where the firmware already runs in ACPI mode (SCI_EN):
//!   the mount enables PWRBTN_EN and clears the status it reads. The status
//!   records a press only, so the guest gets a press and a release. Firmware
//!   in legacy mode keeps the button, and the thermal, fan and battery
//!   events its SMM handles, which an OS that took them would have to
//!   service.
use super::host;
use crate::output;
use core::arch::asm;
use stela_boot_core::power_button::{FORCE_WINDOW_MILLIS, Press, Presses};
use stela_runtime::pc::power::{HostPower, POWER_BUTTON, SCI_ENABLE};
use stela_soc::machine::AppleDevices;

/// The stop codes the hypervisor exit port reports.
pub const RESTART_CODE: u32 = 0x15;
pub const POWER_OFF_CODE: u32 = 0x17;

const KEYBOARD_STATUS: u16 = 0x64;
const KEYBOARD_INPUT_FULL: u8 = 1 << 1;
const KEYBOARD_PULSE_RESET: u8 = 0xfe;

pub(super) unsafe fn out8(port: u16, value: u8) {
    // SAFETY: the caller's port.
    unsafe { asm!("out dx, al", in("dx") port, in("al") value, options(nostack, preserves_flags)) }
}

pub(super) unsafe fn in8(port: u16) -> u8 {
    let value;
    // SAFETY: the caller's port.
    unsafe { asm!("in al, dx", in("dx") port, out("al") value, options(nostack, preserves_flags)) }
    value
}

pub(super) unsafe fn out16(port: u16, value: u16) {
    // SAFETY: the caller's port.
    unsafe { asm!("out dx, ax", in("dx") port, in("ax") value, options(nostack, preserves_flags)) }
}

pub(super) unsafe fn in16(port: u16) -> u16 {
    let value;
    // SAFETY: the caller's port.
    unsafe { asm!("in ax, dx", in("dx") port, out("ax") value, options(nostack, preserves_flags)) }
    value
}

pub(super) unsafe fn out32(port: u16, value: u32) {
    // SAFETY: the caller's port.
    unsafe {
        asm!("out dx, eax", in("dx") port, in("eax") value, options(nostack, preserves_flags))
    }
}

pub(super) unsafe fn in32(port: u16) -> u32 {
    let value;
    // SAFETY: the caller's port.
    unsafe {
        asm!("in eax, dx", in("dx") port, out("eax") value, options(nostack, preserves_flags))
    }
    value
}

/// Spin until `done` or `milliseconds` pass.
fn wait(tsc_hz: u64, milliseconds: u64, mut done: impl FnMut() -> bool) -> bool {
    let start = host::rdtsc();
    let ticks = tsc_hz / 1000 * milliseconds;
    while host::rdtsc() - start < ticks {
        if done() {
            return true;
        }
        core::hint::spin_loop();
    }
    done()
}

/// `EFI_RESET_TYPE`'s EfiResetShutdown (UEFI 2.10 section 8.5.1).
const EFI_RESET_SHUTDOWN: u32 = 2;

/// The firmware's `ResetSystem` runtime service, whose regions the loader
/// keeps mapped by identity in its root.
#[derive(Clone, Copy)]
pub struct EfiReset {
    service: unsafe extern "efiapi" fn(u32, usize, usize, *const u8),
    root: u64,
}

impl EfiReset {
    /// `None` when the firmware has no `ResetSystem`.
    pub fn new(service: u64, root: u64) -> Option<Self> {
        (service != 0).then(|| Self {
            // SAFETY: the firmware's ResetSystem entry the loader read from
            // its runtime services table; code addresses fit a pointer.
            service: unsafe {
                core::mem::transmute::<usize, unsafe extern "efiapi" fn(u32, usize, usize, *const u8)>(
                    service as usize,
                )
            },
            root,
        })
    }

    /// Power off through the firmware, as Linux's `efi_power_off`; returns
    /// only when it did not.
    fn power_off(self) {
        output::arguments(format_args!("POWER powering off through the firmware\n"));
        output::flush_log();
        // SAFETY: the loader root maps the firmware's runtime regions by
        // identity, as it left them without SetVirtualAddressMap;
        // interrupts stay masked and the host drivers' lock is held.
        unsafe {
            let _root = super::storage::RestoreRoot::enter(self.root);
            (self.service)(EFI_RESET_SHUTDOWN, 0, 0, core::ptr::null());
        }
    }
}

pub struct HostPowerControl {
    power: Option<HostPower>,
    /// The firmware's AML, which prepares S5 and evaluates its sleep types.
    acpi: Option<&'static super::aml::HostAcpi>,
    efi: Option<EfiReset>,
    tsc_hz: u64,
    /// The host disks, told before either transition.
    disks: super::storage::HostDisks,
    /// The power button is read.
    button: bool,
    /// The guest refused the button once; reported once.
    refused: bool,
    /// The last press that may still be pending: a second one soon after
    /// stands for a hold ([`stela_boot_core::power_button`]).
    presses: Presses,
    /// When the PM1 status registers were last read.
    looked: u64,
}

/// How often the PM1 status registers are read for the power button. Its
/// status bit stays set until written, so no press is lost between reads,
/// and each read is a port access a hypervisor takes to its device model
/// and the boot PE would make at every service turn.
const BUTTON_MILLIS: u64 = 50;

impl HostPowerControl {
    /// The host's power hardware the loader found, with the fixed power
    /// button armed when there is one.
    pub fn new(
        power: Option<HostPower>,
        acpi: Option<&'static super::aml::HostAcpi>,
        efi: Option<EfiReset>,
        tsc_hz: u64,
        disks: super::storage::HostDisks,
    ) -> Self {
        let mut control = Self {
            power,
            acpi,
            efi,
            tsc_hz,
            disks,
            button: false,
            refused: false,
            presses: Presses::new(),
            looked: 0,
        };
        control.button = control.arm_button();
        control
    }

    pub fn disks(&self) -> &super::storage::HostDisks {
        &self.disks
    }

    fn arm_button(&self) -> bool {
        if self.acpi.is_some_and(|acpi| acpi.hardware_active()) {
            return true;
        }
        let Some(power) = self.power.as_ref().filter(|power| power.fixed_power_button) else {
            output::arguments(format_args!(
                "POWER host button: no fixed-feature power button\n"
            ));
            return false;
        };
        let pm1a = power.pm1[0].expect("a fixed button has PM1a");
        // SAFETY: the FADT's PM1a control block.
        if unsafe { in16(pm1a.control) } & SCI_ENABLE == 0 {
            output::arguments(format_args!(
                "POWER host button: the firmware runs in legacy mode and keeps the button\n"
            ));
            return false;
        }
        for block in power.blocks() {
            let enable = power.enable_port(block);
            // SAFETY: the FADT's PM1 event blocks; a status bit clears on one.
            unsafe {
                out16(block.event, POWER_BUTTON);
                out16(enable, in16(enable) | POWER_BUTTON);
            }
        }
        output::arguments(format_args!(
            "POWER host button: fixed-feature at PM1a_EVT {:#x}\n",
            pm1a.event
        ));
        true
    }

    /// Give the guest a press and a release of its power button, for a
    /// press of the host's ([`Self::pressed`]). A host button has no hold a
    /// PC reports, so a second press within a few seconds, with the guest
    /// not asleep since, stands for one: the Mac turns off however the
    /// guest fares, and the run's translations are kept as at any power-off.
    /// Any later press, or one after a sleep, is a new request.
    pub fn press(&mut self, devices: &mut AppleDevices) {
        // The service moves between cores: the corrected count, not a core's
        // raw TSC.
        let window = host::COUNTER_HZ / 1000 * FORCE_WINDOW_MILLIS;
        let press = self.presses.press(
            host::fresh_counter(),
            window,
            devices.sleeps(),
            devices.asleep(),
        );
        let result = if press == Press::ForceOff {
            output::arguments(format_args!(
                "POWER host button pressed twice within {FORCE_WINDOW_MILLIS} ms: held\n"
            ));
            devices.power_button_held()
        } else {
            devices
                .power_button(true)
                .and_then(|()| devices.power_button(false))
        };
        if let Err(error) = result
            && !core::mem::replace(&mut self.refused, true)
        {
            output::arguments(format_args!("POWER host button refused: {error}\n"));
        }
    }

    /// Request a clean guest shutdown. Repeated native requests remain button
    /// pulses; only an explicit forced request bypasses the guest's exit.
    pub fn request_shutdown(&mut self, devices: &mut AppleDevices) {
        let result = devices
            .power_button(true)
            .and_then(|()| devices.power_button(false));
        if let Err(error) = result
            && !core::mem::replace(&mut self.refused, true)
        {
            output::arguments(format_args!("POWER orderly shutdown refused: {error}\n"));
        }
    }

    /// Whether the host's power button was pressed since the last call.
    pub fn pressed(&mut self) -> bool {
        if let Some(acpi) = self.acpi.filter(|acpi| acpi.hardware_active()) {
            return acpi.take_power_button();
        }
        let Some(power) = self.power.as_ref().filter(|_| self.button) else {
            return false;
        };
        let now = host::rdtsc();
        if now.wrapping_sub(self.looked) < self.tsc_hz / 1000 * BUTTON_MILLIS {
            return false;
        }
        self.looked = now;
        let pressed = power.take_power_button(&mut crate::stop::Io);
        if pressed {
            output::arguments(format_args!("POWER host button pressed\n"));
        }
        pressed
    }

    /// Restart the host machine.
    pub fn restart(&self) -> ! {
        match crate::stop_record::return_at_restart() {
            Some(entry) => output::arguments(format_args!(
                "POWER the firmware starts its entry Boot{entry:04X} again after the restart\n"
            )),
            None => output::arguments(format_args!(
                "POWER the firmware chooses what starts after the restart: it named no entry \
                 for this start, or would not take one for the next\n"
            )),
        }
        output::arguments(format_args!("STELA RESTART code={RESTART_CODE:#x}\n"));
        output::close_logs();
        self.disks.shutdown(false);
        output::debug_exit(RESTART_CODE);
        match self.power.as_ref().and_then(|power| power.reset) {
            Some((register, value)) => {
                // SAFETY: the FADT's reset register; the machine resets.
                unsafe { crate::stop::write_reset(register, value) };
                wait(self.tsc_hz, 500, || false);
                output::arguments(format_args!(
                    "POWER the ACPI reset register did not reset the machine\n"
                ));
            }
            None => output::arguments(format_args!("POWER no ACPI reset register\n")),
        }
        // SAFETY: the 8042's command port, when one answers.
        unsafe {
            if in8(KEYBOARD_STATUS) != 0xff {
                wait(self.tsc_hz, 100, || {
                    in8(KEYBOARD_STATUS) & KEYBOARD_INPUT_FULL == 0
                });
                out8(KEYBOARD_STATUS, KEYBOARD_PULSE_RESET);
                wait(self.tsc_hz, 500, || false);
            }
        }
        output::arguments(format_args!("POWER restarting by triple fault\n"));
        output::flush_log();
        let empty = [0u16; 5];
        // SAFETY: an empty IDT makes the breakpoint a triple fault, which
        // resets the processor.
        unsafe {
            asm!("lidt [{}]", "int3", in(reg) &empty, options(noreturn));
        }
    }

    /// Power the host machine off.
    pub fn power_off(&self) -> ! {
        output::arguments(format_args!("STELA POWER-OFF code={POWER_OFF_CODE:#x}\n"));
        output::close_logs();
        self.disks.shutdown(true);
        output::debug_exit(POWER_OFF_CODE);
        // Linux's acpi_power_off_prepare: `\_PTS(5)`, and S5's sleep types
        // as the firmware's AML evaluates them, over those the mount read
        // from a literal `\_S5`.
        let evaluated = self.acpi.and_then(super::aml::HostAcpi::prepare_soft_off);
        let soft_off = self
            .power
            .as_ref()
            .and_then(|power| Some((power, evaluated.or(power.soft_off)?)));
        // Linux's `efi_poweroff_required`: a hardware-reduced platform, or
        // one without S5, powers off through the firmware.
        let reduced = self
            .power
            .as_ref()
            .is_some_and(|power| power.sleep_registers.is_some());
        if let Some(efi) = self.efi
            && (soft_off.is_none() || reduced)
        {
            efi.power_off();
        }
        let Some((power, (a, b))) = soft_off else {
            output::fatal("the host has no ACPI S5 soft-off sequence or firmware power-off")
        };
        power.enter_soft_off(&mut crate::stop::Io, (a, b));
        wait(self.tsc_hz, 1000, || false);
        output::arguments(format_args!("POWER the host did not power off at S5\n"));
        if let Some(efi) = self.efi {
            efi.power_off();
        }
        output::fatal("the host did not power off at S5; its firmware may need _PTS")
    }
}
