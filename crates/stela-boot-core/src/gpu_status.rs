//! What the boot screen says when no GPU started: which NVIDIA GPU was
//! refused and why, in words a user can act on from another computer.
use alloc::{format, string::String, vec::Vec};
use stela_drivers::gpu::{Adapter, Vendor};
use stela_loader::host_firmware::recipes::nvidia_chip;

/// The NVIDIA generations Stela's GPU driver runs (Pascal, Turing, Ampere, Ada and
/// Blackwell: every chip `recipes::nvidia_chip` names), as people know them.
pub const SUPPORTED: &str = "an NVIDIA GeForce GTX 10, 16 or RTX 20, 30, 40 or 50 series GPU";

/// The boot screen's line when the NVIDIA GPU started with no screen
/// connected and macOS is shown on the screen the PC started on, which
/// another display adapter drives (a laptop with no display mux).
pub const SHOWN_BY_COPY: &str = "No screen is connected to the NVIDIA GPU, so Stela copies each \
    frame of macOS to the screen the PC started on. The picture is slower than on a screen \
    connected to the GPU.";

/// Why an NVIDIA GPU did not start.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Reason {
    /// The USB holds no firmware for its chip.
    MissingFirmware,
    /// The USB holds its chip's firmware, and reading it before the
    /// firmware's exit failed.
    FirmwareReadFailed,
    /// The USB holds its chip's firmware, and it was not read: the GPU
    /// answered only after the firmware's exit, as a GPU the firmware left
    /// without power does once Stela powers its port.
    FirmwareNotRead,
    /// Anything else; the boot log names it.
    Other,
}

/// The start of the screen line for a GPU whose files are on the USB and
/// not in memory.
const HOLDS: &str = "the USB holds the firmware for";
/// Its end when reading them before the firmware's exit failed.
const READ_FAILED: &str = ", but reading it from the USB failed. The boot log on this USB \
    says why; `stela repair` on another computer writes the files again";
/// Its end when they were not read because the GPU answered only after the
/// firmware's exit.
const NOT_READ: &str = ", but it was not read: the GPU answered only after the PC's firmware \
    handed over to Stela. The boot log on this USB says why";

/// The screen line for the GPU `device` of `vendor` at PCI `address` that
/// did not start for `reason`, `why` the refusal's own words (empty without
/// any): a screen is often all a report brings.
pub fn refused(vendor: Vendor, address: &str, device: u16, reason: Reason, why: &str) -> String {
    if vendor != Vendor::Nvidia {
        // A GPU of another maker that its driver's own table admitted.
        let gpu = format!(
            "the {} GPU at {address} (device {device:04x})",
            maker(vendor)
        );
        return match (reason, why.is_empty()) {
            (Reason::MissingFirmware, _) => format!(
                "the USB has no firmware for {gpu}. On another computer, plug in this USB and \
                 run `stela repair`"
            ),
            (Reason::FirmwareReadFailed, _) => format!("{HOLDS} {gpu}{READ_FAILED}"),
            (Reason::FirmwareNotRead, _) => format!("{HOLDS} {gpu}{NOT_READ}"),
            (Reason::Other, true) => {
                format!("{gpu} did not start; the boot log on this USB says why")
            }
            (Reason::Other, false) => {
                format!("{gpu} did not start: {why}. The boot log on this USB has the rest")
            }
        };
    }
    match (nvidia_chip(device), reason) {
        (None, _) => format!(
            "the NVIDIA GPU at {address} (device {device:04x}) is not supported: Stela needs \
             {SUPPORTED}"
        ),
        (Some(chip), Reason::MissingFirmware) => format!(
            "the USB has no firmware for this PC's NVIDIA {chip} GPU. On another computer, plug \
             in this USB and run `stela repair`"
        ),
        (Some(chip), Reason::FirmwareReadFailed) => {
            format!("{HOLDS} this PC's NVIDIA {chip} GPU at {address}{READ_FAILED}")
        }
        (Some(chip), Reason::FirmwareNotRead) => {
            format!("{HOLDS} this PC's NVIDIA {chip} GPU at {address}{NOT_READ}")
        }
        (Some(chip), Reason::Other) if why.is_empty() => format!(
            "the NVIDIA {chip} GPU at {address} did not start; the boot log on this USB says why"
        ),
        (Some(chip), Reason::Other) => format!(
            "the NVIDIA {chip} GPU at {address} did not start: {why}. The boot log on this USB \
             has the rest"
        ),
    }
}

/// The screen line for the NVIDIA GPUs among `adapters` that were claimed
/// but did not start, `error` naming the step that ended the start.
pub fn failed(adapters: &[Adapter], error: &str) -> String {
    let named: Vec<String> = adapters
        .iter()
        .filter(|found| found.vendor == Vendor::Nvidia)
        .map(|found| match nvidia_chip(found.device) {
            Some(chip) => format!(
                "NVIDIA {chip} GPU at {} (device {:04x})",
                found.address, found.device
            ),
            None => format!(
                "NVIDIA GPU at {} (device {:04x})",
                found.address, found.device
            ),
        })
        .collect();
    // Without an NVIDIA GPU, the other adapters a driver was started for.
    let others: Vec<String> = adapters
        .iter()
        .filter(|found| found.vendor != Vendor::Nvidia)
        .map(|found| {
            format!(
                "{} GPU at {} (device {:04x})",
                maker(found.vendor),
                found.address,
                found.device
            )
        })
        .collect();
    let gpu = match (named.is_empty(), others.is_empty()) {
        (false, _) => named.join(" and "),
        (true, false) => others.join(" and "),
        (true, true) => String::from("NVIDIA GPU"),
    };
    format!(
        "the {gpu} could not start: {error}. macOS starts without it; the boot log on this USB \
         says why"
    )
}

/// The screen line for the claimed GPU at PCI `address` that its Linux
/// driver `driver` did not bind. `dark` when the firmware's picture was on
/// it: the claim's reset ended that picture, so its display shows nothing
/// and only another adapter's screen, or the log, carries this line.
pub fn unbound(address: &str, driver: &str, dark: bool) -> String {
    format!(
        "the GPU at {address} did not start: its Linux driver {driver} did not bind it.{} The \
         boot log on this USB says why",
        if dark {
            " The screen the PC started on is wired to it and stays dark."
        } else {
            ""
        }
    )
}

/// The reason the boot stops when Linux's GPU driver stops for good, `stop`
/// naming what it could not do. `failed_start` is the first error Linux
/// reported while the GPUs started, when the stop came before the start
/// finished: it says why the GPU did not start. `firmware` is what the
/// GPU's own firmware said about that start ([`GspStart::failure`]).
pub fn driver_stopped(stop: &str, failed_start: Option<&str>, firmware: Option<&str>) -> String {
    match (failed_start, firmware) {
        (Some(error), Some(firmware)) => format!(
            "the NVIDIA GPU did not start ({error}). {firmware}. Stela cannot yet finish the GPU \
             driver's cleanup after that ({stop})"
        ),
        (Some(error), None) => format!(
            "the NVIDIA GPU did not start ({error}), and Stela cannot yet finish the GPU \
             driver's cleanup after that ({stop})"
        ),
        (None, _) => format!("the GPU driver stopped: {stop}"),
    }
}

/// `NV_VGPU_MSG_EVENT_GSP_INIT_DONE`: GSP-RM's report that its own start
/// ended (nouveau `rm/r535/nvrm/msgfn.h` and `rm/r570/nvrm/msgfn.h`).
const GSP_INIT_DONE: u32 = 0x1001;
/// `NV_VGPU_MSG_EVENT_RECOVERY_ACTION` (r570 `msgfn.h`): GSP-RM changed the
/// recovery the GPU needs, which it does when it marks the GPU for a reset.
/// nouveau handles no such event and prints it.
const RECOVERY_ACTION: u32 = 0x1022;

/// What GSP-RM, the firmware an NVIDIA GPU runs its driver core on, said
/// about its start, read from the lines nouveau prints: `gsp: RM version:
/// <v>` names the firmware nouveau loaded (`nvkm/subdev/gsp/base.c`), and
/// `gsp:msg fn:<n> ... res:0x<r>` is a reply GSP-RM ended with result `r`,
/// which nouveau prints before it turns it into -EINVAL
/// (`r535_gsp_msg_recv`). nouveau's own error for a failed start is then
/// only "gsp: init failed, -22".
#[derive(Debug, Clone, Default, PartialEq, Eq)]
pub struct GspStart {
    version: Option<String>,
    /// The nonzero NV_STATUS GSP-RM ended its start with.
    init_result: Option<u32>,
    /// Whether GSP-RM raised RECOVERY_ACTION before it ended its start.
    recovery: bool,
}

impl GspStart {
    /// Take one line Linux printed, without its log level.
    pub fn observe(&mut self, line: &str) {
        if let Some((_, version)) = line.split_once("gsp: RM version: ") {
            self.version
                .get_or_insert_with(|| String::from(version.trim()));
            return;
        }
        let Some((_, reply)) = line.split_once("gsp:msg ") else {
            return;
        };
        let field = |name: &str| {
            reply
                .split_ascii_whitespace()
                .find_map(|word| word.strip_prefix(name))
        };
        let function = field("fn:").and_then(|value| value.parse::<u32>().ok());
        let result = field("res:")
            .and_then(|value| value.strip_prefix("0x"))
            .and_then(|value| u32::from_str_radix(value, 16).ok());
        match (function, result) {
            (Some(GSP_INIT_DONE), Some(result)) if result != 0 => {
                self.init_result.get_or_insert(result);
            }
            (Some(RECOVERY_ACTION), _) if self.init_result.is_none() => self.recovery = true,
            _ => {}
        }
    }

    /// The words for a start GSP-RM itself ended with an error, or None
    /// when it reported none. They name the step: GSP-RM's own
    /// initialisation, which nouveau waits for as GSP_INIT_DONE once GSP-RM
    /// booted (`r535_gsp_init`, after the FSP or booter started it) and
    /// before it asks GSP-RM for anything.
    pub fn failure(&self) -> Option<String> {
        let result = self.init_result?;
        let firmware = match &self.version {
            Some(version) => format!("GSP-RM {version}"),
            None => String::from("GSP-RM"),
        };
        let status = match nv_status(result) {
            Some((name, meaning)) => {
                format!("{name} ({result:#x}): {meaning} inside that firmware")
            }
            None => format!("NV_STATUS {result:#x}"),
        };
        let recovery = match self.recovery {
            true => ", after it had raised RECOVERY_ACTION, its mark that the GPU needs a reset",
            false => "",
        };
        Some(format!(
            "The GPU's own firmware, {firmware}, ended its initialisation (GSP_INIT_DONE, right \
             after it booted) with {status}{recovery}"
        ))
    }
}

/// The name and meaning of an NV_STATUS code GSP-RM's start can end with
/// (NVIDIA's `nvstatuscodes.h`).
fn nv_status(code: u32) -> Option<(&'static str, &'static str)> {
    Some(match code {
        0x0f => ("NV_ERR_GPU_IS_LOST", "the GPU stopped answering on the bus"),
        0x40 => (
            "NV_ERR_INVALID_STATE",
            "a step found the GPU in a state it did not expect",
        ),
        0x51 => ("NV_ERR_NO_MEMORY", "a step ran out of memory"),
        0x56 => (
            "NV_ERR_NOT_SUPPORTED",
            "a step is not supported on this GPU",
        ),
        0x62 => ("NV_ERR_RESET_REQUIRED", "a step asked for a GPU reset"),
        0x65 => ("NV_ERR_TIMEOUT", "a step timed out"),
        0x70 => ("NV_ERR_FLCN_ERROR", "a microcontroller halted"),
        _ => return None,
    })
}

/// The memory BARs a reset kept, as the claim's reset line names them:
/// `BAR0 16 MiB, BAR1 16 GiB`, each `(index, bytes)`. A BAR the size of the
/// GPU's memory means firmware enabled Resizable BAR, which a reset puts
/// back to its default size unless it is restored.
pub fn bar_sizes(bars: &[(usize, u64)]) -> String {
    let named: Vec<String> = bars
        .iter()
        .map(|&(index, bytes)| match bytes {
            bytes if bytes >= 1 << 30 && bytes.is_multiple_of(1 << 30) => {
                format!("BAR{index} {} GiB", bytes >> 30)
            }
            bytes if bytes >= 1 << 20 && bytes.is_multiple_of(1 << 20) => {
                format!("BAR{index} {} MiB", bytes >> 20)
            }
            bytes => format!("BAR{index} {bytes:#x} bytes"),
        })
        .collect();
    named.join(", ")
}

/// How the screen names a display adapter: its maker, PCI address and
/// device ID.
pub fn adapter(adapter: &Adapter) -> String {
    format!(
        "the {} display adapter at {} (device {:04x})",
        maker(adapter.vendor),
        adapter.address,
        adapter.device
    )
}

fn maker(vendor: Vendor) -> String {
    match vendor {
        Vendor::Nvidia => String::from("NVIDIA"),
        Vendor::Amd => String::from("AMD"),
        Vendor::Intel => String::from("Intel"),
        Vendor::Other(id) => format!("PCI vendor {id:04x}"),
    }
}

/// The line for a display adapter Stela's GPU driver does not run, and why.
pub fn unused(found: &Adapter) -> String {
    format!(
        "{} is not used: {}",
        adapter(found),
        found.unavailable_reason()
    )
}

/// The screen line for a PC without an NVIDIA GPU, naming the display
/// adapters it has instead.
pub fn none(adapters: &[Adapter]) -> String {
    let found: Vec<String> = adapters
        .iter()
        .filter(|found| found.vendor != Vendor::Nvidia)
        .map(adapter)
        .collect();
    match found.is_empty() {
        true => format!("this PC has no NVIDIA GPU: Stela needs {SUPPORTED}"),
        false => format!(
            "this PC has no NVIDIA GPU, only {}: Stela needs {SUPPORTED}",
            found.join(" and ")
        ),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn a_gpu_of_another_maker_is_named_by_its_maker() {
        assert_eq!(
            refused(Vendor::Intel, "00:02.0", 0x9a49, Reason::Other, ""),
            "the Intel GPU at 00:02.0 (device 9a49) did not start; the boot log on this USB \
             says why"
        );
        let missing = refused(
            Vendor::Intel,
            "00:02.0",
            0x9a49,
            Reason::MissingFirmware,
            "",
        );
        assert!(
            missing.starts_with("the USB has no firmware for the Intel GPU at 00:02.0"),
            "{missing}"
        );
        assert!(missing.ends_with("run `stela repair`"), "{missing}");
        let named = refused(Vendor::Intel, "00:02.0", 0x9a49, Reason::Other, "no reset");
        assert!(
            named.contains("did not start: no reset. The boot log"),
            "{named}"
        );
    }

    #[test]
    fn an_unbound_gpu_says_when_its_screen_stays_dark() {
        assert_eq!(
            unbound("00:02.0", "xe", false),
            "the GPU at 00:02.0 did not start: its Linux driver xe did not bind it. The boot \
             log on this USB says why"
        );
        let dark = unbound("00:02.0", "xe", true);
        assert!(
            dark.contains("xe did not bind it. The screen the PC started on is wired to it"),
            "{dark}"
        );
    }

    #[test]
    fn a_refused_gpu_is_named_with_its_fix() {
        let missing = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x2484,
            Reason::MissingFirmware,
            "",
        );
        assert!(missing.contains("NVIDIA ga104 GPU"), "{missing}");
        assert!(missing.ends_with("run `stela repair`"), "{missing}");
        // The files are on the USB: no `stela repair` for a GPU that
        // answered late, and the USB is not said to lack them.
        let late = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x28a0,
            Reason::FirmwareNotRead,
            "",
        );
        assert_eq!(
            late,
            "the USB holds the firmware for this PC's NVIDIA ad107 GPU at 01:00.0, but it was \
             not read: the GPU answered only after the PC's firmware handed over to Stela. The \
             boot log on this USB says why"
        );
        let failed = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x28a0,
            Reason::FirmwareReadFailed,
            "",
        );
        assert!(
            failed.starts_with(
                "the USB holds the firmware for this PC's NVIDIA ad107 GPU at 01:00.0, but \
                 reading it from the USB failed."
            ),
            "{failed}"
        );
        let amd = refused(Vendor::Amd, "03:00.0", 0x7550, Reason::FirmwareNotRead, "");
        assert!(
            amd.starts_with("the USB holds the firmware for the AMD GPU at 03:00.0 (device 7550)"),
            "{amd}"
        );
        let other = refused(Vendor::Nvidia, "03:00.0", 0x2b85, Reason::Other, "");
        assert_eq!(
            other,
            "the NVIDIA gb202 GPU at 03:00.0 did not start; the boot log on this USB says why"
        );
        // The refusal's words reach the screen, which is often all a report
        // brings.
        let named = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x28a0,
            Reason::Other,
            "the GPU's register BAR does not decode",
        );
        assert_eq!(
            named,
            "the NVIDIA ad107 GPU at 01:00.0 did not start: the GPU's register BAR does not \
             decode. The boot log on this USB has the rest"
        );
    }

    #[test]
    fn an_older_gpu_or_none_names_what_is_supported() {
        // A GeForce GTX 980 (Maxwell, gm204) is unsupported.
        let maxwell = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x13c0,
            Reason::MissingFirmware,
            "",
        );
        assert_eq!(
            maxwell,
            "the NVIDIA GPU at 01:00.0 (device 13c0) is not supported: Stela needs an NVIDIA \
             GeForce GTX 10, 16 or RTX 20, 30, 40 or 50 series GPU"
        );
        // A GeForce GTX 1060 (Pascal, gp106) is supported.
        let pascal = refused(
            Vendor::Nvidia,
            "01:00.0",
            0x1c03,
            Reason::MissingFirmware,
            "",
        );
        assert_eq!(
            pascal,
            "the USB has no firmware for this PC's NVIDIA gp106 GPU. On another computer, plug \
             in this USB and run `stela repair`"
        );
        assert!(none(&[]).starts_with("this PC has no NVIDIA GPU: Stela needs"));
    }

    #[test]
    fn a_pc_with_only_other_display_adapters_names_them() {
        use stela_drivers::bus::pci::Address;
        let at = |bus, vendor, device| Adapter {
            address: Address {
                segment: 0,
                bus,
                device: 0,
                function: 0,
            },
            vendor,
            device,
            class: [3, 0, 0],
        };
        let amd = at(3, Vendor::Amd, 0x744c);
        let intel = at(0, Vendor::Intel, 0xa780);
        assert_eq!(
            none(&[amd, intel]),
            "this PC has no NVIDIA GPU, only the AMD display adapter at 0000:03:00.0 (device \
             744c) and the Intel display adapter at 0000:00:00.0 (device a780): Stela needs an \
             NVIDIA GeForce GTX 10, 16 or RTX 20, 30, 40 or 50 series GPU"
        );
        assert_eq!(
            unused(&amd),
            "the AMD display adapter at 0000:03:00.0 (device 744c) is not used: AMDGPU is not \
             ported to Stela"
        );
        assert!(adapter(&at(1, Vendor::Other(0x1af4), 0x1050)).contains("PCI vendor 1af4"));
    }

    #[test]
    fn a_driver_stop_after_a_failed_start_names_the_first_error() {
        assert_eq!(
            driver_stopped(
                "IRQ action retirement unavailable; request retained",
                Some("nouveau 0000:01:00.0: gsp: init failed, -22"),
                None
            ),
            "the NVIDIA GPU did not start (nouveau 0000:01:00.0: gsp: init failed, -22), and \
             Stela cannot yet finish the GPU driver's cleanup after that (IRQ action retirement \
             unavailable; request retained)"
        );
        assert_eq!(driver_stopped("x", None, None), "the GPU driver stopped: x");
        assert_eq!(
            driver_stopped("x", None, Some("y")),
            "the GPU driver stopped: x"
        );
    }

    #[test]
    fn a_start_gsp_rm_ended_itself_names_the_firmware_and_its_result() {
        // The lines of an RTX 5060 (GB206) whose GSP-RM timed out in its
        // own start, in the order nouveau printed them.
        let mut start = GspStart::default();
        for line in [
            "nouveau 0000:01:00.0: NVIDIA GB206 (1b6000a1)",
            "nouveau 0000:01:00.0: gsp: RM version: 570.144",
            "nouveau 0000:01:00.0: gsp:msg fn:4130 len:0x28/0x8 res:0x0 resp:0x0",
            "6msg: 00000000: 00 00 00 00 00 00 00 00                          ........",
        ] {
            start.observe(line);
        }
        assert_eq!(start.failure(), None);
        start.observe("nouveau 0000:01:00.0: gsp:msg fn:4097 len:0x20/0x0 res:0x65 resp:0x65");
        // A later reply's error does not replace the start's.
        start.observe("nouveau 0000:01:00.0: gsp:msg fn:4097 len:0x20/0x0 res:0x51 resp:0x0");
        let failure = start.failure().unwrap();
        assert_eq!(
            failure,
            "The GPU's own firmware, GSP-RM 570.144, ended its initialisation (GSP_INIT_DONE, \
             right after it booted) with NV_ERR_TIMEOUT (0x65): a step timed out inside that \
             firmware, after it had raised RECOVERY_ACTION, its mark that the GPU needs a reset"
        );
        assert_eq!(
            driver_stopped(
                "IRQ action retirement unavailable; request retained",
                Some("nouveau 0000:01:00.0: gsp: init failed, -22"),
                Some(&failure)
            ),
            "the NVIDIA GPU did not start (nouveau 0000:01:00.0: gsp: init failed, -22). The \
             GPU's own firmware, GSP-RM 570.144, ended its initialisation (GSP_INIT_DONE, right \
             after it booted) with NV_ERR_TIMEOUT (0x65): a step timed out inside that firmware, \
             after it had raised RECOVERY_ACTION, its mark that the GPU needs a reset. Stela \
             cannot yet finish the GPU driver's cleanup after that (IRQ action retirement \
             unavailable; request retained)"
        );

        // An error reply to any other call is not the start's result, and
        // an unknown code is named by its number.
        let mut other = GspStart::default();
        other.observe("nouveau 0000:01:00.0: gsp:msg fn:76 len:0x20/0x0 res:0x56 resp:0x0");
        assert_eq!(other.failure(), None);
        other.observe("nouveau 0000:01:00.0: gsp:msg fn:4097 len:0x20/0x0 res:0x1234 resp:0x0");
        // A recovery action after the start's end is not part of it.
        other.observe("nouveau 0000:01:00.0: gsp:msg fn:4130 len:0x28/0x8 res:0x0 resp:0x0");
        assert_eq!(
            other.failure().unwrap(),
            "The GPU's own firmware, GSP-RM, ended its initialisation (GSP_INIT_DONE, right after \
             it booted) with NV_STATUS 0x1234"
        );
    }

    #[test]
    fn the_bars_a_reset_kept_are_named_with_their_sizes() {
        assert_eq!(
            bar_sizes(&[(0, 64 << 20), (1, 16 << 30), (3, 32 << 20), (5, 0x80)]),
            "BAR0 64 MiB, BAR1 16 GiB, BAR3 32 MiB, BAR5 0x80 bytes"
        );
        assert_eq!(bar_sizes(&[]), "");
    }

    #[test]
    fn a_gpu_that_failed_to_start_is_named_with_its_step() {
        use stela_drivers::bus::pci::Address;
        let at = |bus, vendor, device| Adapter {
            address: Address {
                segment: 0,
                bus,
                device: 0,
                function: 0,
            },
            vendor,
            device,
            class: [3, 0, 0],
        };
        // An RTX 4060 (AD107) beside an Intel iGPU.
        let found = [at(0, Vendor::Intel, 0xa780), at(1, Vendor::Nvidia, 0x2882)];
        assert_eq!(
            failed(&found, "GPU startup exceeded its deadline"),
            "the NVIDIA ad107 GPU at 0000:01:00.0 (device 2882) could not start: GPU startup \
             exceeded its deadline. macOS starts without it; the boot log on this USB says why"
        );
        assert!(failed(&[], "x").starts_with("the NVIDIA GPU could not start: x."));
        // Without an NVIDIA GPU, the adapters a driver was started for.
        assert!(
            failed(&found[..1], "x")
                .starts_with("the Intel GPU at 0000:00:00.0 (device a780) could not start: x."),
        );
    }
}
