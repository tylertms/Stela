//! USB 2.0 hardware link power management (L1) for devices on USB 2 root
//! ports, as Linux `xhci_update_device`, `hub_set_initial_usb2_lpm_policy`
//! and `xhci_set_usb2_hardware_lpm` do. USB 3 U1/U2 needs
//! `XHCI_LPM_SUPPORT`, which Linux sets for no AMD host.
use super::*;

/// The USB 2 Supported Protocol's hardware LPM and BESL capabilities.
const PROTOCOL_HLC: u32 = 1 << 19;
const PROTOCOL_BLC: u32 = 1 << 20;
/// PORTPMSC and PORTHLPMC, after PORTSC.
const PORTPMSC: usize = 0x4;
const PORTHLPMC: usize = 0xc;
const PMSC_RWE: u32 = 1 << 3;
const PMSC_HIRD: u32 = 0xf << 4;
const PMSC_L1DS: u32 = 0xff << 8;
const PMSC_HLE: u32 = 1 << 16;
/// The USB 2.0 Extension's bmAttributes.
const LPM_SUPPORT: u32 = 1 << 1;
const BESL_SUPPORT: u32 = 1 << 2;
const BESL_BASELINE_VALID: u32 = 1 << 3;
const BESL_DEEP_VALID: u32 = 1 << 4;
/// `XHCI_L1_TIMEOUT` and `XHCI_DEFAULT_BESL`.
const L1_TIMEOUT_US: u32 = 512;
const DEFAULT_BESL: u8 = 4;
/// `xhci_besl_encoding`, in microseconds.
const BESL_ENCODING: [u16; 16] = [
    125, 150, 200, 300, 400, 500, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000,
];
const BOS_DESCRIPTOR: u8 = 15;
const DEVICE_CAPABILITY: u8 = 16;
const USB2_EXTENSION: u8 = 2;
const U0_WAIT_US: u64 = 10_000;
/// `USB_QUIRK_NO_LPM` devices in Linux's `usb_quirk_list`.
const NO_LPM: &[(u16, u16)] = &[
    (0x045e, 0x07c6),
    (0x046d, 0x0825),
    (0x046d, 0x086c),
    (0x04e8, 0x6200),
    (0x04f3, 0x0381),
    (0x05e3, 0x0612),
    (0x05e3, 0x0616),
    (0x067b, 0x2731),
    (0x0781, 0x5583),
    (0x0781, 0x5591),
    (0x0781, 0x55ae),
    (0x0853, 0x011b),
    (0x0951, 0x1666),
    (0x0930, 0x1408),
    (0x0bda, 0x0487),
    (0x0bda, 0x8153),
    (0x0fce, 0x0dde),
    (0x1532, 0x0e05),
    (0x17ef, 0x720c),
    (0x17ef, 0x721e),
    (0x17ef, 0xa387),
    (0x17ef, 0xa391),
    (0x17ef, 0xa392),
    (0x1de1, 0xc102),
    (0x1edb, 0xbd3b),
    (0x1edb, 0xbd4f),
    (0x1f75, 0x0917),
    (0x2109, 0x0711),
    (0x2109, 0x2817),
    (0x2386, 0x3114),
    (0x2386, 0x3119),
    (0x2386, 0x350e),
    (0x2ca3, 0x0031),
    (0x2ce3, 0x9563),
    (0x413c, 0xb062),
];

/// The host's USB 2 hardware LPM support and what is enabled per root port.
#[derive(Clone, Debug, Default)]
pub(super) struct Host {
    /// HCIVERSION, HCSPARAMS3's U2 device exit latency, and each root
    /// port's USB 2 Supported Protocol port word (zero on others).
    version: u16,
    u2_latency: u32,
    protocol: Vec<u32>,
    enabled: Vec<Option<Usb2Lpm>>,
}

impl Host {
    pub(super) fn new(version: u16, u2_latency: u32, protocol: Vec<u32>) -> Self {
        Self {
            version,
            u2_latency,
            enabled: vec![None; protocol.len()],
            protocol,
        }
    }

    /// `hw_lpm_support` with `xhci_check_usb2_port_capability`.
    fn port(&self, port: u8) -> u32 {
        match self.version >= 0x100 {
            true => self
                .protocol
                .get(usize::from(port) - 1)
                .copied()
                .unwrap_or(0),
            false => 0,
        }
    }
}

/// One device's L1 parameters.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) struct Usb2Lpm {
    slot: u8,
    /// The USB 2.0 Extension's bmAttributes.
    attributes: u32,
    /// `usb2_hw_lpm_besl_capable`: the port takes BESL rather than HIRD.
    besl_capable: bool,
}

impl Usb2Lpm {
    /// `xhci_calculate_hird_besl` for a HIRD port.
    fn hird(self, u2_latency: u32) -> u8 {
        let (host, device) = if self.attributes & BESL_SUPPORT != 0 {
            let host = BESL_ENCODING
                .iter()
                .position(|&latency| u32::from(latency) >= u2_latency)
                .unwrap_or(16);
            (host as u32, self.preferred_besl().unwrap_or(0))
        } else if u2_latency <= 50 {
            (0, 0)
        } else {
            ((u2_latency - 51) / 75 + 1, 0)
        };
        (host + u32::from(device)).min(15) as u8
    }

    /// The device's baseline BESL, else its deep BESL.
    fn preferred_besl(self) -> Option<u8> {
        let field = |shift: u32| (self.attributes >> shift & 0xf) as u8;
        if self.attributes & BESL_BASELINE_VALID != 0 {
            Some(field(8))
        } else if self.attributes & BESL_DEEP_VALID != 0 {
            Some(field(12))
        } else {
            None
        }
    }

    /// The BESL a BESL port uses: the device's baseline, else
    /// `XHCI_DEFAULT_BESL`.
    fn besl(self) -> u8 {
        let baseline = BESL_SUPPORT | BESL_BASELINE_VALID;
        match self.attributes & baseline == baseline {
            true => (self.attributes >> 8 & 0xf) as u8,
            false => DEFAULT_BESL,
        }
    }

    /// `xhci_calculate_usb2_hw_lpm_params`: BESLD, the L1 timeout in 256 µs
    /// steps and HIRDM.
    fn hlpmc(self) -> u32 {
        let (besld, hirdm) = match self.attributes & BESL_DEEP_VALID != 0 {
            true => (self.attributes >> 12 & 0xf, 1),
            false => (0, 0),
        };
        besld << 10 | (L1_TIMEOUT_US / 256) << 2 | hirdm
    }
}

impl Controller {
    /// `hub_port_init`'s BOS read and `hub_set_initial_usb2_lpm_policy`:
    /// hardware LPM for a non-hub high- or full-speed device on a USB 2 root
    /// port with HLC whose USB 2.0 Extension supports LPM and BESL. Linux
    /// also enables a hardwired port without BESL; that connect type comes
    /// from ACPI `_UPC`, which this driver does not have. Failures leave LPM
    /// off, as Linux ignores them.
    pub(super) fn initial_usb2_lpm(&mut self, device: &Device, attachment: Attachment) {
        block_on(Self::initial_usb2_lpm_with(
            &Access::blocking(self, None),
            device,
            attachment,
        ))
    }

    pub(super) async fn initial_usb2_lpm_with(
        a: &Access<'_>,
        device: &Device,
        attachment: Attachment,
    ) {
        let port = attachment.root_port;
        let protocol = a.with(|c| {
            if let Some(enabled) = c.lpm.enabled.get_mut(usize::from(port) - 1) {
                *enabled = None;
            }
            c.lpm.port(port)
        });
        let bcd = u16::from_le_bytes([device.descriptor[2], device.descriptor[3]]);
        let id = (
            u16::from_le_bytes([device.descriptor[8], device.descriptor[9]]),
            u16::from_le_bytes([device.descriptor[10], device.descriptor[11]]),
        );
        if attachment.route != 0
            || device.descriptor[4] == HUB_CLASS
            || !matches!(attachment.speed, HIGH_SPEED | FULL_SPEED)
            || protocol & PROTOCOL_HLC == 0
            || bcd < 0x0201
            || NO_LPM.contains(&id)
        {
            return;
        }
        let Some(attributes) = Self::usb2_extension_with(a, device.slot).await else {
            return;
        };
        if attributes & (LPM_SUPPORT | BESL_SUPPORT) != LPM_SUPPORT | BESL_SUPPORT {
            return;
        }
        let lpm = Usb2Lpm {
            slot: device.slot,
            attributes,
            besl_capable: protocol & PROTOCOL_BLC != 0,
        };
        if lpm.besl_capable {
            let latency = BESL_ENCODING[usize::from(lpm.besl())];
            if Self::max_exit_latency_with(a, lpm.slot, latency)
                .await
                .is_err()
            {
                return;
            }
        }
        a.with(|c| {
            c.enable_usb2_lpm(port, lpm);
            c.lpm.enabled[usize::from(port) - 1] = Some(lpm);
        });
    }

    /// `usb_disable_usb2_hardware_lpm` before a root device's reset.
    pub(super) fn disable_usb2_lpm(&mut self, port: u8) -> Result<(), BlockError> {
        let index = usize::from(port) - 1;
        match self.lpm.enabled.get_mut(index).and_then(Option::take) {
            Some(lpm) => self.set_usb2_lpm(port, lpm, false),
            None => Ok(()),
        }
    }

    /// `usb_get_bos_descriptor`'s USB 2.0 Extension bmAttributes; a device
    /// without a readable BOS has none.
    async fn usb2_extension_with(a: &Access<'_>, slot: u8) -> Option<u32> {
        let header = Self::descriptor_with(a, slot, BOS_DESCRIPTOR, 0, 5)
            .await
            .ok()?;
        if header.len() < 5 || header[1] != BOS_DESCRIPTOR {
            return None;
        }
        let total = u16::from_le_bytes([header[2], header[3]]);
        let bytes = Self::descriptor_with(a, slot, BOS_DESCRIPTOR, 0, total)
            .await
            .ok()?;
        let mut at = usize::from(header[0]).max(5);
        while let Some(&length) = bytes.get(at) {
            let length = usize::from(length);
            if length < 3 || at + length > bytes.len() {
                return None;
            }
            if bytes[at + 1] == DEVICE_CAPABILITY && bytes[at + 2] == USB2_EXTENSION && length >= 7
            {
                return Some(u32::from_le_bytes(
                    bytes[at + 3..at + 7].try_into().unwrap(),
                ));
            }
            at += length;
        }
        None
    }

    /// `xhci_set_usb2_hardware_lpm`. A BESL port first takes the exit
    /// latency through Evaluate Context and BESLD, the L1 timeout and HIRDM
    /// in PORTHLPMC; then PORTPMSC gets the HIRD or BESL, remote wake and
    /// the L1 device slot before HLE. Disabling clears them, and a BESL port
    /// drops the exit latency and waits for U0.
    fn set_usb2_lpm(&mut self, port: u8, lpm: Usb2Lpm, enable: bool) -> Result<(), BlockError> {
        let pmsc = self.port_offset(port) + PORTPMSC;
        let mut value = self.registers.read32(pmsc);
        if enable {
            if lpm.besl_capable {
                self.max_exit_latency(lpm.slot, BESL_ENCODING[usize::from(lpm.besl())])?;
            }
            self.enable_usb2_lpm(port, lpm);
        } else {
            value &= !(PMSC_HLE | PMSC_RWE | PMSC_HIRD | PMSC_L1DS);
            self.registers.write32(pmsc, value);
            self.registers.read32(pmsc);
            if lpm.besl_capable {
                self.max_exit_latency(lpm.slot, 0)?;
                let at = self.port_offset(port);
                let clock = self.clock;
                let registers = &mut self.registers;
                let _ = clock.wait(U0_WAIT_US, || registers.read32(at) & PLS == 0);
            }
        }
        Ok(())
    }

    /// [`Self::set_usb2_lpm`]'s registers once a BESL port's exit latency
    /// is in.
    fn enable_usb2_lpm(&mut self, port: u8, lpm: Usb2Lpm) {
        let pmsc = self.port_offset(port) + PORTPMSC;
        let hlpmc = self.port_offset(port) + PORTHLPMC;
        let mut value = self.registers.read32(pmsc);
        let hird = if lpm.besl_capable {
            self.registers.write32(hlpmc, lpm.hlpmc());
            self.registers.read32(hlpmc);
            lpm.besl()
        } else {
            lpm.hird(self.lpm.u2_latency)
        };
        value &= !PMSC_HIRD;
        value |= u32::from(hird) << 4 | PMSC_RWE | u32::from(lpm.slot) << 8;
        self.registers.write32(pmsc, value);
        let value = self.registers.read32(pmsc);
        self.registers.write32(pmsc, value | PMSC_HLE);
        self.registers.read32(pmsc);
    }

    /// `xhci_change_max_exit_latency`: the slot context's Max Exit Latency
    /// through Evaluate Context.
    fn max_exit_latency(&mut self, slot: u8, latency: u16) -> Result<(), BlockError> {
        block_on(Self::max_exit_latency_with(
            &Access::blocking(self, None),
            slot,
            latency,
        ))
    }

    async fn max_exit_latency_with(
        a: &Access<'_>,
        slot: u8,
        latency: u16,
    ) -> Result<(), BlockError> {
        let input_address = a.with(|c| {
            c.clear_input(slot)?;
            let context = c.context();
            let state = c.slot(slot)?;
            let mut words = state.context;
            words[1] = words[1] & !0xffff | u32::from(latency);
            state.input.write(4, 1u32);
            for (index, word) in words.iter().enumerate() {
                state.input.write(context + 4 * index, *word);
            }
            Ok::<_, BlockError>(state.input.device_address())
        })?;
        a.command(
            input_address,
            0,
            EVALUATE_CONTEXT << 10 | u32::from(slot) << 24,
        )
        .await?;
        a.with(|c| {
            let state = c.slot(slot)?;
            state.context[1] = state.context[1] & !0xffff | u32::from(latency);
            Ok(())
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::dma::RegionAllocator;
    use alloc::collections::BTreeMap;

    #[test]
    fn root_devices_get_hardware_lpm_like_xhci_set_usb2_hardware_lpm() {
        let mut memory = vec![0u64; 512 * 1024];
        let start = memory.as_mut_ptr() as u64;
        // SAFETY: all hardware DMA allocations remain live through the test.
        let mut dma = unsafe { RegionAllocator::new(start..start + memory.len() as u64 * 8) };
        // A USB 2.0 Extension with LPM, BESL and a baseline BESL of 2.
        let mut leaf = hub::model::Node::leaf(HIGH_SPEED);
        leaf.bos = Some(vec![5, 15, 12, 0, 1, 7, 16, 2, 0x0e, 0x02, 0, 0]);
        let (mut controller, _) = hub::tests::controller(&mut dma, 64, BTreeMap::from([(0, leaf)]));
        controller.lpm = Host::new(0x110, 0, vec![PROTOCOL_HLC | PROTOCOL_BLC]);
        let mut failures = Vec::new();
        let devices = controller.enumerate(&mut dma, &mut failures);
        assert!(failures.is_empty(), "{failures:?}");
        let slot = devices[0].slot;
        // The baseline BESL's exit latency, then BESL, remote wake, the slot
        // and HLE; the 512 µs L1 timeout.
        assert_eq!(controller.slot(slot).unwrap().context[1] & 0xffff, 200);
        let at = controller.port_offset(1);
        assert_eq!(
            controller.registers.read32(at + PORTPMSC),
            2 << 4 | PMSC_RWE | u32::from(slot) << 8 | PMSC_HLE
        );
        assert_eq!(controller.registers.read32(at + PORTHLPMC), 2 << 2);
        drop(memory);
    }

    #[test]
    fn l1_parameters_follow_xhci_calculate_hird_besl_and_hw_lpm_params() {
        let lpm = |attributes| Usb2Lpm {
            slot: 3,
            attributes,
            besl_capable: false,
        };
        // HIRD from the host's U2 exit latency alone.
        assert_eq!(lpm(LPM_SUPPORT).hird(50), 0);
        assert_eq!(lpm(LPM_SUPPORT).hird(200), 2);
        // BESL: the host's encoding plus the device's baseline, at most 15.
        let baseline = LPM_SUPPORT | BESL_SUPPORT | BESL_BASELINE_VALID | 3 << 8;
        assert_eq!(lpm(baseline).hird(200), 2 + 3);
        assert_eq!(lpm(baseline).hird(20_000), 15);
        assert_eq!(lpm(baseline).besl(), 3);
        assert_eq!(lpm(LPM_SUPPORT | BESL_SUPPORT).besl(), DEFAULT_BESL);
        // BESLD and HIRDM from a deep BESL; the 512 µs L1 timeout.
        let deep = LPM_SUPPORT | BESL_SUPPORT | BESL_DEEP_VALID | 6 << 12;
        assert_eq!(lpm(deep).hlpmc(), 6 << 10 | 2 << 2 | 1);
        assert_eq!(lpm(LPM_SUPPORT).hlpmc(), 2 << 2);
    }
}
