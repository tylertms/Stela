// SPDX-License-Identifier: GPL-2.0-or-later
//! Legacy NV3 takeover must also open upstream DMA forwarding. Linux's
//! pci_enable_device_flags recursively enables bridges and bus mastering.
//! Only the ancestors of the affected endpoint are changed; no bridge reset,
//! BAR/window reassignment, status clearing or native ownership bypass occurs.
use super::*;
use crate::bus::pci::{ConfigSpace, regs};

#[derive(Clone, Copy)]
struct Bridge {
    address: Address,
    primary: u8,
    secondary: u8,
    subordinate: u8,
}

fn route(target: Address, bridges: &[Bridge]) -> Result<Vec<Address>, BlockError> {
    let mut bus = target.bus;
    let mut path = Vec::new();
    // This compatibility path requires a complete route to bus zero. A
    // nonzero root/ambiguous topology is refused rather than guessed.
    while bus != 0 {
        let mut candidates = bridges
            .iter()
            .filter(|b| b.address.segment == target.segment && b.secondary == bus);
        let bridge = candidates
            .next()
            .ok_or(BlockError::Protocol("NVMe upstream bridge missing"))?;
        if candidates.next().is_some()
            || bridge.primary != bridge.address.bus
            || bridge.secondary > bridge.subordinate
            || target.bus > bridge.subordinate
            || path.contains(&bridge.address)
        {
            return Err(BlockError::Protocol(
                "NVMe upstream bridge topology invalid",
            ));
        }
        path.push(bridge.address);
        bus = bridge.address.bus;
    }
    path.reverse();
    Ok(path)
}

fn enable<C: ConfigSpace>(config: &mut C) -> Result<(u16, u16), BlockError> {
    let before = config.read16(regs::COMMAND)?;
    if before == u16::MAX {
        return Err(BlockError::Protocol("NVMe upstream bridge removed"));
    }
    let desired = before | regs::COMMAND_MEMORY | regs::COMMAND_MASTER;
    if desired != before {
        // Never use a dword RMW: the adjacent status is W1C evidence.
        config.write16(regs::COMMAND, desired)?;
    }
    let after = config.read16(regs::COMMAND)?;
    if after != desired {
        return Err(BlockError::Protocol(
            "NVMe upstream DMA enable readback failed",
        ));
    }
    Ok((before, after))
}

pub(super) fn prepare(scanned: &mut [Vec<Function>]) -> Vec<(Address, BlockError)> {
    let targets: Vec<_> = scanned
        .iter()
        .flatten()
        .filter(|f| f.vendor == 0x2646 && f.device == 0x5027 && f.class == [1, 8, 2])
        .map(|f| f.address)
        .collect();
    let mut failures = Vec::new();
    for target in targets {
        let prepared = (|| {
            let mut bridges = Vec::new();
            for function in scanned
                .iter_mut()
                .flatten()
                .filter(|f| f.address.segment == target.segment && f.class[..2] == [6, 4])
            {
                let address = function.address;
                let config = function.config();
                bridges.push(Bridge {
                    address,
                    primary: config.read8(0x18)?,
                    secondary: config.read8(0x19)?,
                    subordinate: config.read8(0x1a)?,
                });
            }
            let path = route(target, &bridges)?;
            stela_driver_base::working(&alloc::format!(
                "NVMe upstream v13 target={target} bridges={path:?}"
            ));
            for address in path {
                let bridge = scanned
                    .iter_mut()
                    .flatten()
                    .find(|f| f.address == address)
                    .ok_or(BlockError::Protocol("NVMe upstream bridge disappeared"))?;
                let config = bridge.config();
                let status = config.read16(regs::STATUS)?;
                stela_driver_base::working(&alloc::format!(
                    "NVMe upstream v13 before bridge={address} command={:?} status={status:#x} secondary_status={:?}",
                    config.read16(regs::COMMAND),
                    config.read16(0x1e)
                ));
                let (before, after) = enable(config)?;
                stela_driver_base::working(&alloc::format!(
                    "NVMe upstream v13 enabled bridge={address} command={before:#x}->{after:#x} status={:?}",
                    config.read16(regs::STATUS)
                ));
            }
            Ok::<(), BlockError>(())
        })();
        if let Err(error) = prepared {
            stela_driver_base::working(&alloc::format!(
                "NVMe upstream v13 refused target={target}: {error}"
            ));
            failures.push((target, error));
        }
    }
    failures
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::bus::pci::ConfigError;
    fn at(segment: u16, bus: u8, device: u8) -> Address {
        Address {
            segment,
            bus,
            device,
            function: 0,
        }
    }
    fn bridge(bus: u8, secondary: u8, subordinate: u8) -> Bridge {
        Bridge {
            address: at(0, bus, 1),
            primary: bus,
            secondary,
            subordinate,
        }
    }
    #[test]
    fn route_is_root_first_and_excludes_siblings_and_other_segments() {
        let mut other = bridge(0, 3, 3);
        other.address.segment = 1;
        assert_eq!(
            route(
                at(0, 3, 0),
                &[bridge(1, 3, 3), bridge(0, 1, 3), bridge(0, 2, 2), other]
            )
            .unwrap(),
            [at(0, 0, 1), at(0, 1, 1)]
        );
    }
    #[test]
    fn malformed_missing_ambiguous_and_cyclic_paths_are_refused() {
        assert!(route(at(0, 3, 0), &[]).is_err());
        assert!(route(at(0, 3, 0), &[bridge(0, 3, 3), bridge(0, 3, 3)]).is_err());
        assert!(route(at(0, 3, 0), &[bridge(0, 3, 2)]).is_err());
        assert!(route(at(0, 3, 0), &[bridge(3, 3, 3)]).is_err());
        let mut wrong = bridge(0, 3, 3);
        wrong.primary = 2;
        assert!(route(at(0, 3, 0), &[wrong]).is_err());
    }
    struct Config {
        command: u16,
        ignore_write: bool,
        writes: usize,
    }
    impl ConfigSpace for Config {
        fn read16(&self, offset: u16) -> Result<u16, ConfigError> {
            assert_eq!(offset, regs::COMMAND);
            Ok(self.command)
        }
        fn write16(&mut self, offset: u16, value: u16) -> Result<(), ConfigError> {
            assert_eq!(offset, regs::COMMAND);
            self.writes += 1;
            if !self.ignore_write {
                self.command = value;
            }
            Ok(())
        }
        fn read8(&self, _: u16) -> Result<u8, ConfigError> {
            panic!("unexpected width")
        }
        fn read32(&self, _: u16) -> Result<u32, ConfigError> {
            panic!("unexpected width")
        }
        fn write8(&mut self, _: u16, _: u8) -> Result<(), ConfigError> {
            panic!("unexpected width")
        }
        fn write32(&mut self, _: u16, _: u32) -> Result<(), ConfigError> {
            panic!("must not replay W1C status")
        }
    }
    #[test]
    fn enables_both_gates_preserves_bits_and_avoids_unnecessary_writes() {
        let mut c = Config {
            command: 0x402,
            ignore_write: false,
            writes: 0,
        };
        assert_eq!(enable(&mut c).unwrap(), (0x402, 0x406));
        assert_eq!(c.writes, 1);
        enable(&mut c).unwrap();
        assert_eq!(c.writes, 1);
    }
    #[test]
    fn lost_write_and_removed_bridge_are_not_success() {
        let mut c = Config {
            command: 0x402,
            ignore_write: true,
            writes: 0,
        };
        assert!(enable(&mut c).is_err());
        c.command = 0xffff;
        c.writes = 0;
        assert!(enable(&mut c).is_err());
        assert_eq!(c.writes, 0);
    }
}
