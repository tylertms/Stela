// SPDX-License-Identifier: GPL-2.0-only
//! Firmware-to-native handoff. Prefer the NVMe controller reset, as Linux
//! nvme_pci_configure_admin_queue does; PCIe FLR is a bounded fallback, not
//! an unconditional initialization step. No namespace command runs here.
use super::*;
use crate::bus::pci::{ConfigSpace, power::Timer, regs};

pub(super) fn prepare(
    function: &mut Function,
    registers: &RegisterIo,
    clock: Clock,
    minimum: &(impl Timer + ?Sized),
    allow_flr: bool,
) -> Result<(), BlockError> {
    report_pci(function, "firmware");
    function.enable_polled()?;
    verify_command(function.config().read16(regs::COMMAND)?)?;
    let stopped = stop(registers, clock);
    match stopped {
        Ok(()) => stela_driver_base::working(
            "NVMe takeover v12: controller reset acknowledged; FLR not needed",
        ),
        Err(error) if allow_flr && error == BlockError::Timeout => {
            stela_driver_base::working(
                "NVMe takeover v12: controller reset timed out; trying PCIe FLR once",
            );
            if !function.function_level_reset(minimum)? {
                return Err(error);
            }
            function.enable_polled()?;
            verify_command(function.config().read16(regs::COMMAND)?)?;
            stop(registers, clock)?;
        }
        Err(error) => return Err(error),
    }
    report_pci(function, "prepared");
    Ok(())
}

fn verify_command(command: u16) -> Result<(), BlockError> {
    let required = regs::COMMAND_MEMORY | regs::COMMAND_MASTER;
    if command & required != required {
        return Err(BlockError::Protocol(
            "NVMe PCI memory/bus mastering not enabled",
        ));
    }
    Ok(())
}

pub(super) fn stop(registers: &RegisterIo, clock: Clock) -> Result<(), BlockError> {
    let previous = registers.read32(CONFIGURATION)?;
    let status = registers.read32(STATUS)?;
    let cap = registers.read64(CAPABILITIES)?;
    stela_driver_base::working(&format!(
        "NVMe handoff v12 cc={previous:#x} csts={status:#x} cap={cap:#x} vs={:?} aqa={:?} asq={:?} acq={:?}",
        registers.read32(VERSION),
        registers.read32(ADMIN_ATTRIBUTES),
        registers.read64(ADMIN_SUBMISSION),
        registers.read64(ADMIN_COMPLETION)
    ));
    if status == REMOVED || previous == REMOVED || cap == u64::MAX {
        return Err(removed());
    }
    registers.write32(CONFIGURATION, previous & !(ENABLE | SHUTDOWN_NOTIFICATION))?;
    let mut error = None;
    let ready = clock.wait(ready_wait_us(cap >> 24 & 0xff), || {
        match registers.read32(STATUS) {
            Ok(REMOVED) => {
                error = Some(removed());
                true
            }
            Ok(status) => status & READY == 0,
            Err(failure) => {
                error = Some(failure);
                true
            }
        }
    });
    if let Some(error) = error {
        return Err(error);
    }
    if ready {
        Ok(())
    } else {
        Err(BlockError::Timeout)
    }
}

/// Read-only snapshots: never clear W1C PCI/PCIe/AER status. The firmware
/// baseline and failed attachment remain distinguishable in the USB log.
pub(super) fn report_pci(function: &mut Function, phase: &str) {
    let address = function.address;
    let config = function.config();
    stela_driver_base::working(&format!(
        "NVMe PCI v12 {phase} pci={address} command={:?} status={:?} bar0={:?} bar1={:?}",
        config.read16(regs::COMMAND),
        config.read16(regs::STATUS),
        config.read32(regs::BASE_ADDRESS_0),
        config.read32(regs::BASE_ADDRESS_0 + 4)
    ));
    for entry in config.capabilities() {
        let Ok(entry) = entry else {
            break;
        };
        let at = entry.offset;
        if entry.id == 0x10 {
            stela_driver_base::working(&format!(
                "NVMe PCIe v12 {phase} devctl={:?} devsta={:?} linkctl={:?} linksta={:?}",
                config.read16(at + 8),
                config.read16(at + 10),
                config.read16(at + 16),
                config.read16(at + 18)
            ));
        } else if entry.id == 1 {
            stela_driver_base::working(&format!(
                "NVMe PM v12 {phase} pmcsr={:?}",
                config.read16(at + 4)
            ));
        }
    }
    for entry in config.extended_capabilities() {
        let Ok(entry) = entry else {
            break;
        };
        if entry.id == 1 {
            let at = entry.offset;
            stela_driver_base::working(&format!(
                "NVMe AER v12 {phase} uncorrectable={:?} mask={:?} severity={:?} correctable={:?} header={:?}",
                config.read32(at + 4),
                config.read32(at + 8),
                config.read32(at + 12),
                config.read32(at + 16),
                [
                    config.read32(at + 28),
                    config.read32(at + 32),
                    config.read32(at + 36),
                    config.read32(at + 40)
                ]
            ));
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn dma_enable_requires_both_memory_decode_and_bus_mastering() {
        for command in 0u16..8 {
            assert_eq!(verify_command(command).is_ok(), command & 6 == 6);
        }
    }
}
