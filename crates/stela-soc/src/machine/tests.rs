use super::*;
use crate::Result;
use stela_contracts::refuse;

/// The J473 `/arm-io/aes`: version 3, one key-disable word.
fn j473_aes() -> stela_soc_data::aes::Config {
    stela_soc_data::aes::Config {
        version: 3,
        address_width: stela_soc_data::aes::DEFAULT_ADDRESS_WIDTH,
        key_disable: stela_soc_data::aes::KeyDisable {
            bytes: 0x8000,
            word: 0,
            keys: [None; 3],
        },
    }
}
use alloc::{rc::Rc, vec::Vec};
use core::cell::RefCell;
use {
    stela_contracts::bus::MmioDevice, stela_contracts::dma::Iommu,
    stela_contracts::dma::test_support::Ram,
};

#[test]
fn a_board_entry_given_an_earlier_count_is_named_in_the_stop_reason() {
    let d = AppleDevices::new();
    d.observe_time(10, "lines");
    assert_eq!(d.rtc_regression(), "");
    d.observe_time(5, "deadline");
    d.observe_time(3, "read");
    assert_eq!(d.rtc_regression(), " (deadline at 0x5 after lines at 0xa)");
}

#[test]
fn t6020_pmgr_refusals_cover_only_gaps_and_keep_existing_owners() {
    let mut devices = AppleDevices::new();
    devices.add_rail_calibration(0x1010).unwrap();
    let rail = devices.bus.route(0x1010, Width::Word).unwrap();
    devices.add_t6020_unmodeled_pmgr(&[(0x1010, 0x20)]).unwrap();
    assert_eq!(devices.bus.route(0x1010, Width::Word).unwrap(), rail);
    assert_eq!(devices.read(0, 0x1010, 4, 0), Ok(0));
    let unsupported = devices.read(0, 0x1020, 4, 0);
    assert!(matches!(unsupported, Err(DeviceError::Refused(_))));
    assert_eq!(devices.read(0, 0x1030, 4, 0), Err(DeviceError::NoDevice));
}

#[test]
fn native_dart_mounts_program_the_selected_register_bank_and_transfer_real_dma_bytes() {
    use {
        stela_contracts::dma::Memory as _, stela_contracts::dma::Port,
        stela_contracts::dma::test_support::Ram,
    };
    const ROOT: u64 = 0x8_0000_0000;
    const PAGE: u64 = 0x4000;
    for profile in [
        stela_soc_data::dart::Profile::T8020,
        stela_soc_data::dart::Profile::T6000,
        stela_soc_data::dart::Profile::T8110,
    ] {
        let mut d = AppleDevices::new();
        let ranges = [
            Range {
                physical: 0x2_7000_0000,
                bytes: PAGE,
            },
            Range {
                physical: 0x2_7008_0000,
                bytes: PAGE,
            },
        ];
        let ids = d
            .add_dart_group_for_profile(profile, &ranges, 16, None, None)
            .unwrap();
        let mut ram = Ram::new(ROOT, (4 * PAGE) as usize);
        let descriptor = |physical: u64| {
            if profile == stela_soc_data::dart::Profile::T8020 {
                physical | 1
            } else {
                physical >> 4 | 1
            }
        };
        ram.put_u64(ROOT + 8, descriptor(ROOT + PAGE));
        ram.put_u64(ROOT + PAGE + 8, descriptor(ROOT + 2 * PAGE) | 0xfff << 40);
        let (enabled, tcr, ttbr, ttbr_value, translate) =
            if profile == stela_soc_data::dart::Profile::T8110 {
                (0xc00, 0x1004, 0x1404, ROOT >> 12 | 1, 1)
            } else {
                (0xfc, 0x104, 0x210, ROOT >> 12 | 1 << 31, 1 << 7)
            };
        for (offset, value) in [(enabled, 2), (tcr, translate), (ttbr, ttbr_value)] {
            d.write(0, ranges[0].physical + offset, 4, value, 0)
                .unwrap();
        }
        assert_eq!(d.read(0, ranges[0].physical + tcr, 4, 0), Ok(translate));
        assert_eq!(d.read(0, ranges[1].physical + tcr, 4, 0), Ok(0));
        let mut a = Access::new(0, 0, &mut d.latch);
        let mut port = Port {
            iommu: d.bus.device_mut::<Dart>(ids[0].0).unwrap(),
            stream: 1,
            memory: &mut ram,
            access: &mut a,
        };
        port.write(0x0200_4007, &[2, 4, 6, 8]).unwrap();
        let mut actual = [0; 4];
        assert!(ram.read(ROOT + 2 * PAGE + 7, &mut actual));
        assert_eq!(actual, [2, 4, 6, 8]);
    }
}

#[test]
fn revision_2_2_mounts_whole_windows_beside_the_filter_and_capture_views() {
    let mut d = AppleDevices::new();
    let ranges = [
        Range {
            physical: 0x2_7000_0000,
            bytes: 0x20000,
        },
        Range {
            physical: 0x2_7004_0000,
            bytes: 0xc000,
        },
    ];
    // A window selects its register file; neither revision mounts the
    // other's, nor a window without a recovered register file.
    for (profile, bytes) in [
        (stela_soc_data::dart::Profile::T8110, 0xc000),
        (stela_soc_data::dart::Profile::T8110Rev22, 0x4000),
        (stela_soc_data::dart::Profile::T8110Rev22, 0x8000),
        (stela_soc_data::dart::Profile::T8110Rev22, 0x24000),
    ] {
        let range = Range {
            physical: ranges[0].physical,
            bytes,
        };
        assert!(
            d.add_dart_group_for_profile(profile, &[range], 16, None, None)
                .is_err()
        );
        assert!(d.read(0, range.physical, 4, 0).is_err());
    }
    let apf = 0x2_7002_0000;
    let ids = d
        .add_dart_group_for_profile(
            stela_soc_data::dart::Profile::T8110Rev22,
            &ranges,
            16,
            None,
            Some((apf, 16)),
        )
        .unwrap();
    let smmu = Range {
        physical: 0x2_7005_4000,
        bytes: 0x4000,
    };
    d.add_dart_smmu(ids[1], smmu).unwrap();
    for range in ranges {
        assert_eq!(d.read(0, range.physical + 8, 4, 0), Ok(0x2a2a_0202));
        assert_eq!(d.read(0, range.physical + 0x4000, 4, 0), Ok(0));
        assert!(d.read(0, range.physical + 0x100, 4, 0).is_err());
    }
    assert!(d.read(0, ranges[0].physical + 0xc000, 4, 0).is_err());
    assert!(d.read(0, ranges[0].physical + 0x1fffc, 4, 0).is_err());
    // The filter's slices stay reachable at their own window.
    d.write(0, apf + 8, 4, 0x1234_0000, 0).unwrap();
    assert_eq!(d.read(0, apf + 8, 4, 0), Ok(0x1234_0000));
    assert!(d.read(0, smmu.physical, 4, 0).is_err());
}

#[test]
fn unsupported_legacy_mount_features_refuse_before_any_aperture_is_published() {
    let mut d = AppleDevices::new();
    let range = Range {
        physical: 0x2_7000_0000,
        bytes: 0x4000,
    };
    for (streams, apf) in [(64, None), (16, Some((0x2_7010_0000, 1)))] {
        assert!(
            d.add_dart_group_for_profile(
                stela_soc_data::dart::Profile::T8020,
                &[range],
                streams,
                None,
                apf
            )
            .is_err()
        );
        assert!(d.read(0, range.physical, 4, 0).is_err());
    }
    for invalid in [
        Range {
            physical: range.physical,
            bytes: 0,
        },
        Range {
            physical: range.physical,
            bytes: 0x8000,
        },
        Range {
            physical: range.physical + 4,
            bytes: 0x4000,
        },
        Range {
            physical: !0x3fff,
            bytes: 0x4000,
        },
    ] {
        assert!(
            d.add_dart_group_for_profile(
                stela_soc_data::dart::Profile::T6000,
                &[invalid],
                16,
                None,
                None
            )
            .is_err()
        );
        assert!(d.read(0, range.physical, 4, 0).is_err());
    }
    assert!(
        d.add_dart_group_for_profile(
            stela_soc_data::dart::Profile::T8020,
            &[range],
            0,
            None,
            None
        )
        .is_err()
    );
    assert!(
        d.add_dart_group_for_profile(stela_soc_data::dart::Profile::T8020, &[], 16, None, None)
            .is_err()
    );
    let ids = d
        .add_dart_group_for_profile(
            stela_soc_data::dart::Profile::T8020,
            &[range],
            16,
            None,
            None,
        )
        .unwrap();
    let smmu = Range {
        physical: 0x2_7020_0000,
        bytes: 0x4000,
    };
    assert!(d.add_dart_smmu(ids[0], smmu).is_err());
    assert!(d.read(0, smmu.physical, 4, 0).is_err());
}

#[test]
fn coresight_components_keep_independent_locks_and_do_not_claim_debug_controls() {
    let mut d = AppleDevices::new();
    let bases = [0x2_1001_0000, 0x2_1101_0000];
    for base in bases {
        d.add_coresight_lock(base).unwrap();
        assert_eq!(d.read(0, base + coresight::STATUS, 4, 0), Ok(3));
    }
    d.write(0, bases[0] + coresight::ACCESS, 4, 0xc5acce55, 0)
        .unwrap();
    assert_eq!(d.read(0, bases[0] + coresight::STATUS, 4, 0), Ok(1));
    assert_eq!(d.read(0, bases[1] + coresight::STATUS, 4, 0), Ok(3));
    assert!(d.read(0, bases[0], 4, 0).is_err());
    assert!(d.write(0, bases[0] + coresight::ACCESS, 8, 0, 0).is_err());
}

fn powered_dart(retention: bool, supplies: usize) -> (AppleDevices, DartId, u64, u64) {
    powered_dart_with(retention, supplies, |_| {})
}

/// A declared synthetic physical supply for mounted native banks.
/// Tests choose retention explicitly; production constructors do not infer
/// it from an RTKit sleep capability.
fn connect_native_power(
    d: &mut AppleDevices,
    supply: u64,
    resources: &[(&str, u64)],
    retained: bool,
) {
    use stela_soc_data::board::PowerResource;
    let gate = coverage::DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![coverage::Domain {
            name: String::new(),
            id: gate,
            address: Some(supply),
            parents: [None; 4],
        }],
        resources
            .iter()
            .map(|&(name, physical)| PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: name.into(),
                clocks: vec![gate],
                power: vec![],
                owner: Some(PowerOwner::Mmio(physical)),
                admission: coverage::Admission::Required,
            })
            .collect(),
    );
    for device in &mut d.power_devices {
        device.retention = retained;
    }
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &[supply],
    )
    .unwrap();
    d.connect_power_resources(&states, &[], &[]).unwrap();
    assert!(d.power_refusals().is_empty());
}

fn powered_dart_with(
    retention: bool,
    supplies: usize,
    edit: impl FnOnce(&mut stela_soc_data::board::PowerStates),
) -> (AppleDevices, DartId, u64, u64) {
    use coverage::{Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let mut d = AppleDevices::new();
    let supply = 0x2_3b70_0100;
    let base = 0x2_3500_4000;
    let registers = (0..supplies)
        .map(|index| supply + index as u64 * 8)
        .collect::<Vec<_>>();
    let ids = (0..supplies)
        .map(|index| DomainId {
            die: 0,
            device: index as u16 + 1,
        })
        .collect::<Vec<_>>();
    let mut states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, supplies as u64 * 8)],
        registers.clone(),
        registers
            .iter()
            .zip(&ids)
            .map(|(&address, &id)| Domain {
                name: String::new(),
                id,
                address: Some(address),
                parents: [None; 4],
            })
            .collect(),
        vec![PowerResource {
            clock_role: stela_soc_data::board::ClockRole::Consumer,
            name: "/arm-io/dart".into(),
            clocks: ids,
            power: Vec::new(),
            owner: Some(PowerOwner::Dart(0)),
            admission: coverage::Admission::Required,
        }],
    );
    edit(&mut states);
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: supplies as u64 * 8,
        }],
        &registers,
    )
    .unwrap();
    let dart = d
        .add_dart(
            Range {
                physical: base,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    d.connect_power_resources(&states, &[vec![dart]], &[retention])
        .unwrap();
    (d, dart, supply, base)
}

fn powered_serial_controllers() -> (AppleDevices, Console, u64, [u64; 3]) {
    use coverage::{Admission, Domain, DomainId};
    use stela_soc_data::board::{Device, I2cController, PowerResource, SpiNor};
    let mut d = AppleDevices::new();
    let supply = 0x2_3b70_0100;
    let bases = [UART, 0x2_3501_0000, 0x2_3510_4000];
    let id = DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![Domain {
            name: String::new(),
            id,
            address: Some(supply),
            parents: [None; 4],
        }],
        bases
            .iter()
            .zip(["uart", "i2c", "spi"])
            .map(|(&physical, name)| PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: name.into(),
                clocks: vec![id],
                power: vec![],
                owner: Some(PowerOwner::Mmio(physical)),
                admission: Admission::Required,
            })
            .collect(),
    );
    let console = Console::default();
    d.add_uart(
        console.clone(),
        &[Range {
            physical: bases[0],
            bytes: 0x4000,
        }],
        None,
    )
    .unwrap();
    d.add_i2c(
        &I2cController {
            device: Device {
                physical: bases[1],
                bytes: 0x1000,
                irq: 761,
            },
            targets: vec![],
        },
        &|_| None,
    )
    .unwrap();
    d.add_spi_nor(&SpiNor {
        device: Device {
            physical: bases[2],
            bytes: 0x1000,
            irq: 749,
        },
        image: stela_soc_data::nor::Image::fresh(),
        select: None,
    })
    .unwrap();
    assert_eq!(
        d.power_devices.len(),
        3,
        "actual constructors register each bank once"
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &[supply],
    )
    .unwrap();
    d.connect_power_resources(&states, &[], &[]).unwrap();
    assert!(d.power_refusals().is_empty());
    (d, console, supply, bases)
}

#[test]
fn the_host_copies_the_nor_image_only_when_a_nor_exists() {
    let (mut d, ..) = powered_serial_controllers();
    assert_eq!(d.nor_image(), Some(stela_soc_data::nor::Image::fresh()));
    assert_eq!(AppleDevices::new().nor_image(), None);
}

#[test]
fn pmgr_serial_constructor_hooks_gate_mmio_and_preserve_kept_configuration() {
    let (mut d, console, supply, [uart, i2c, spi]) = powered_serial_controllers();
    for (address, value) in [(uart, 3), (i2c + 0x30, 0x1234), (spi + 0x30, 7)] {
        d.write(0, address, 4, value, 0).unwrap();
    }
    d.write(0, uart + 0x20, 4, u64::from(b'a'), 0).unwrap();
    for client in &mut d.power_clients {
        client.retention = true; // Explicit synthetic policy, not a J473 retention claim.
    }
    for request in [4, 0] {
        d.write(0, supply, 4, request, 0).unwrap();
        for address in [uart, i2c + 0x30, spi + 0x30] {
            assert!(d.read(0, address, 4, 0).is_err());
        }
        d.write(0, supply, 4, 15, 0).unwrap();
        for (address, value) in [(uart, 3), (i2c + 0x30, 0x1234), (spi + 0x30, 7)] {
            assert_eq!(d.read(0, address, 4, 0), Ok(value));
        }
    }
    assert_eq!(*console.0.borrow(), b"a");
}

#[test]
fn pmgr_serial_reset_preflights_nor_before_clearing_other_controllers() {
    let (mut d, console, supply, [uart, i2c, spi]) = powered_serial_controllers();
    d.write(0, uart + 0x20, 4, u64::from(b'a'), 0).unwrap();
    d.write(0, spi + 0x0c, 4, 0, 0).unwrap(); // Internal CS low.
    d.write(0, spi + 0x4c, 4, 1, 0).unwrap();
    d.write(0, spi + 0x10, 4, 3, 0).unwrap(); // READ, missing address.
    d.write(0, spi, 4, 1, 0).unwrap();
    d.write(0, supply, 4, 0x40f, 0).unwrap();
    let before = d.read(0, supply, 4, 0).unwrap();
    assert!(matches!(d.write(0, supply, 4, 0x8000040f, 0),
            Err(DeviceError::Refused(reason)) if reason.to_string().contains("incomplete NOR")));
    assert_eq!(d.read(0, supply, 4, 0), Ok(before));
    assert_eq!(d.read(0, uart, 4, 0), Ok(3));
    assert_eq!(d.read(0, i2c + 0x1c, 4, 0), Ok(0));
    d.write(0, supply, 4, 15, 0).unwrap();
    d.write(0, spi + 0x4c, 4, 3, 0).unwrap();
    for _ in 0..3 {
        d.write(0, spi + 0x10, 4, 0, 0).unwrap();
    }
    // Closing the now-complete command allows a shared reset.
    d.write(0, spi + 0x0c, 4, 2, 0).unwrap();
    for request in [0x40f, 0x8000040f, 0x40f, 15] {
        d.write(0, supply, 4, request, 0).unwrap();
    }
    for address in [uart, i2c + 0x1c, spi] {
        assert!(
            d.read(0, address, 4, 0).is_err(),
            "unknown reset image at {address:#x}"
        );
    }
    assert_eq!(*console.0.borrow(), b"a");
}

/// A checked native bank fixture shares a mounted DART's supply.
/// Its refusal models an explicitly unchecked destructive transition,
/// without claiming another physical controller implementation.
struct CheckedPowerBank {
    reject: bool,
    committed: Vec<LifecycleState>,
}
impl stela_contracts::bus::Lifecycle for CheckedPowerBank {
    fn check_lifecycle(&self, state: LifecycleState) -> Result<()> {
        if self.reject && (!state.powered || state.reset) {
            Err(refuse!(
                "native bank destructive transition remains unqualified"
            ))
        } else {
            Ok(())
        }
    }
    fn set_lifecycle(&mut self, state: LifecycleState, _: &mut Access<'_>) {
        self.committed.push(state);
    }
}
impl MmioDevice for CheckedPowerBank {
    fn lifecycle(&mut self) -> Option<&mut dyn stela_contracts::bus::Lifecycle> {
        Some(self)
    }
    fn name(&self) -> &'static str {
        "checked-power-bank"
    }
    fn read(&mut self, _: u64, _: Width, _: &mut Access<'_>) -> Result<u64, MmioError> {
        Ok(self.committed.len() as u64)
    }
    fn write(&mut self, _: u64, _: Width, _: u64, _: &mut Access<'_>) -> Result<(), MmioError> {
        Ok(())
    }
}

#[test]
fn repeated_resource_names_do_not_cross_connect_native_power_domains() {
    use coverage::{Admission, Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let mut d = AppleDevices::new();
    let supply = 0x2_3b70_0100;
    let native = [0x2_3600_0000, 0x2_3601_0000];
    let registers = [supply, supply + 8];
    let ids = [1, 2].map(|device| DomainId { die: 0, device });
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 16)],
        registers.into(),
        registers
            .into_iter()
            .zip(ids)
            .map(|(address, id)| Domain {
                name: String::new(),
                id,
                address: Some(address),
                parents: [None; 4],
            })
            .collect(),
        native
            .into_iter()
            .zip(ids)
            .map(|(physical, id)| PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "/arm-io/repeated".into(),
                clocks: vec![id],
                power: vec![],
                owner: Some(PowerOwner::Mmio(physical)),
                admission: Admission::Required,
            })
            .collect(),
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 16,
        }],
        &registers,
    )
    .unwrap();
    for physical in native {
        let bank = d.bus.add(Box::new(CheckedPowerBank {
            reject: false,
            committed: vec![],
        }));
        d.bus.map(bank, physical, 4, 0).unwrap();
        d.register_power_device(physical, bank, true).unwrap();
    }
    d.connect_power_resources(&states, &[], &[]).unwrap();
    assert_eq!(d.power_clients[0].domains.len(), 1);
    assert_eq!(d.power_clients[0].domains[0].0, supply);
    assert_eq!(d.power_clients[1].domains.len(), 1);
    assert_eq!(d.power_clients[1].domains[0].0, supply + 8);
    d.write(0, supply, 4, 0x3f4, 0).unwrap();
    assert_eq!(d.read(0, native[0], 4, 0), Ok(1));
    assert_eq!(d.read(0, native[1], 4, 0), Ok(0));
    d.write(0, supply + 8, 4, 0x3f4, 0).unwrap();
    assert_eq!(d.read(0, native[0], 4, 0), Ok(1));
    assert_eq!(d.read(0, native[1], 4, 0), Ok(1));
}

#[test]
fn pmgr_all_native_banks_preflight_before_register_or_dart_effects() {
    use coverage::{Admission, Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let mut d = AppleDevices::new();
    let supply = 0x2_3b70_0100;
    let base = 0x2_3500_4000;
    let canonical = 0x2_3600_0000;
    let gate = DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![Domain {
            name: String::new(),
            id: gate,
            address: Some(supply),
            parents: [None; 4],
        }],
        [
            ("/arm-io/dart", PowerOwner::Dart(0)),
            ("/arm-io/checked-native", PowerOwner::Mmio(canonical)),
        ]
        .map(|(name, owner)| PowerResource {
            clock_role: stela_soc_data::board::ClockRole::Consumer,
            name: name.into(),
            clocks: vec![gate],
            power: vec![],
            owner: Some(owner),
            admission: Admission::Required,
        })
        .into(),
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &[supply],
    )
    .unwrap();
    let dart = d
        .add_dart(
            Range {
                physical: base,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let first = d.bus.add(Box::new(CheckedPowerBank {
        reject: false,
        committed: vec![],
    }));
    let second = d.bus.add(Box::new(CheckedPowerBank {
        reject: true,
        committed: vec![],
    }));
    d.register_power_device(canonical, first, false).unwrap();
    d.register_power_device(canonical, second, false).unwrap();
    assert!(d.register_power_device(canonical, first, false).is_err());
    let pmgr = d.pmgr.unwrap();
    assert!(d.register_power_device(canonical, pmgr, false).is_err());
    d.connect_power_resources(&states, &[vec![dart]], &[false])
        .unwrap();
    assert!(d.register_power_device(canonical, second, false).is_err());
    d.write(0, base + 0x1000, 4, 1, 0).unwrap();
    let before = d.read(0, supply, 4, 0).unwrap();
    let failure = d.write(0, supply, 4, 0x3f0, 0).unwrap_err();
    assert!(
        matches!(failure, DeviceError::Refused(reason) if reason.to_string().contains("checked-native") && reason.to_string().contains("unqualified"))
    );
    assert_eq!(d.read(0, supply, 4, 0), Ok(before));
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(1));
    assert!(
        d.bus
            .device_mut::<Pmgr>(pmgr)
            .unwrap()
            .take_transitions()
            .is_empty()
    );
    for bank in [first, second] {
        assert!(
            d.bus
                .device::<CheckedPowerBank>(bank)
                .unwrap()
                .committed
                .is_empty()
        );
    }
    // No lifecycle change: configuration and width validation do not ask
    // a native bank to check an unrelated destructive transition.
    d.write(0, supply, 4, 0x1000f, 0).unwrap();
    assert!(d.write(0, supply, 8, 0, 0).is_err());
    for request in [4, 15, 0x40f] {
        d.write(0, supply, 4, request, 0).unwrap();
    }
    let before = d.read(0, supply, 4, 0).unwrap();
    assert!(d.write(0, supply, 4, 0x8000040f, 0).is_err());
    assert_eq!(d.read(0, supply, 4, 0), Ok(before));
    d.bus.device_mut::<CheckedPowerBank>(second).unwrap().reject = false;
    for request in [0x8000040f, 0x40f, 15, 0, 15] {
        d.write(0, supply, 4, request, 0).unwrap();
    }
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
    for bank in [first, second] {
        let edges = &d.bus.device::<CheckedPowerBank>(bank).unwrap().committed;
        assert_eq!(edges.len(), 8);
        assert!(!edges[0].clocked && edges[0].powered);
        assert!(edges[1].clocked);
        assert!(edges[2].quiesced);
        assert!(edges[3].reset);
        assert!(!edges[4].reset && edges[4].quiesced);
        assert!(!edges[5].quiesced);
        assert!(!edges[6].powered);
        assert!(edges[7].powered);
    }
}

#[test]
fn pmgr_power_gates_reset_dart_tables_but_clock_gates_and_retention_preserve_them() {
    for retention in [false, true] {
        let (mut d, _, supply, base) = powered_dart(retention, 1);
        d.write(0, base + 0x1000, 4, 1, 0).unwrap();
        d.write(0, supply, 4, 4, 0).unwrap();
        assert!(d.read(0, base + 0x1000, 4, 0).is_err());
        d.write(0, supply, 4, 15, 0).unwrap();
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(1));
        d.write(0, supply, 4, 0, 0).unwrap();
        assert!(d.read(0, base + 0x1000, 4, 0).is_err());
        d.write(0, supply, 4, 15, 0).unwrap();
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(u64::from(retention)));
    }
}

#[test]
fn paired_smmu_shares_dart_power_and_reset() {
    let (mut d, owner, supply, base) = powered_dart(false, 1);
    let smmu = 0x2_7130_0000;
    d.add_dart_smmu(
        owner,
        Range {
            physical: smmu,
            bytes: dart::SMMU_BYTES,
        },
    )
    .unwrap();
    assert_eq!(d.read(0, smmu, 4, 0), Ok(0));
    assert!(d.read(0, smmu, 8, 0).is_err());
    assert!(d.read(0, smmu + 0x2000, 4, 0).is_err());
    d.write(0, supply, 4, 0, 0).unwrap();
    assert!(d.read(0, base, 4, 0).is_err());
    assert!(d.read(0, smmu, 4, 0).is_err());
    d.write(0, supply, 4, 15, 0).unwrap();
    assert_eq!(d.read(0, smmu, 4, 0), Ok(0));
    for reset in [true, false] {
        let mut access = Access::new(0, 0, &mut d.latch);
        d.bus
            .device_mut::<Dart>(owner.0)
            .unwrap()
            .set_reset(reset, &mut access);
        assert_eq!(d.read(0, smmu, 4, 0).is_err(), reset);
        assert_eq!(d.read(0, base, 4, 0).is_err(), reset);
    }
}

#[test]
fn mounted_t6000_groups_preserve_each_fault_on_the_same_physical_wire() {
    use stela_contracts::interrupt::Wire;
    let mut d = AppleDevices::new();
    let input = AicInput::new(0, 873);
    d.add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 16))
        .unwrap();
    let mut ids = Vec::new();
    for base in [0x2_8930_4000, 0x2_8930_c000] {
        ids.push(
            d.add_dart_group_for_profile(
                stela_soc_data::dart::Profile::T6000,
                &[Range {
                    physical: base,
                    bytes: stela_soc_data::dart::DAPF_BYTES,
                }],
                16,
                Some(input),
                None,
            )
            .unwrap()[0],
        );
    }
    assert_eq!(d.dart_irqs.len(), 1);
    let mut ram = Ram::new(0, 0x4000);
    for id in &ids {
        let mut access = Access::new(0, 0, &mut d.latch);
        assert!(
            d.bus
                .device_mut::<Dart>(id.0)
                .unwrap()
                .translate(&mut ram, 0, 0, 0, false, &mut access)
                .is_err()
        );
    }
    assert!(d.latch.is_asserted(Wire::Aic(input)));
    d.write(0, 0x2_8930_4040, 4, 1, 0).unwrap();
    assert!(d.latch.is_asserted(Wire::Aic(input)));
    d.write(0, 0x2_8930_c040, 4, 1, 0).unwrap();
    assert!(!d.latch.is_asserted(Wire::Aic(input)));
}

#[test]
fn t6000_paired_umms_shares_reset_and_invalidates_only_unknown_configuration() {
    let mut d = AppleDevices::new();
    let base = 0x2_8930_4000;
    let umms = 0x2_8930_0000;
    let id = d
        .add_dart_group_for_profile(
            stela_soc_data::dart::Profile::T6000,
            &[Range {
                physical: base,
                bytes: stela_soc_data::dart::DAPF_BYTES,
            }],
            16,
            None,
            None,
        )
        .unwrap()[0];
    d.add_dart_smmu(
        id,
        Range {
            physical: umms,
            bytes: dart::SMMU_BYTES,
        },
    )
    .unwrap();
    assert_eq!(d.read(0, umms, 4, 0), Ok(0));
    assert_eq!(d.read(0, umms + 0x20, 4, 0), Ok(0));
    d.write(0, umms + 0x20, 4, 1 << 31, 0).unwrap();
    {
        let mut a = Access::new(0, 0, &mut d.latch);
        d.bus
            .device_mut::<Dart>(id.0)
            .unwrap()
            .set_reset(true, &mut a);
    }
    assert!(d.read(0, base, 4, 0).is_err());
    assert!(d.read(0, umms, 4, 0).is_err());
    {
        let mut a = Access::new(0, 0, &mut d.latch);
        d.bus
            .device_mut::<Dart>(id.0)
            .unwrap()
            .set_reset(false, &mut a);
    }
    assert_eq!(d.read(0, umms, 4, 0), Ok(0));
    assert!(d.read(0, umms + 0x20, 4, 0).is_err());
    d.write(0, umms + 0x20, 4, 0, 0).unwrap();
    assert_eq!(d.read(0, umms + 0x20, 4, 0), Ok(0));
    assert!(d.read(0, umms + 0x2000, 4, 0).is_err());
}

#[test]
fn failed_smmu_placement_does_not_attach_or_publish_a_view() {
    let mut d = AppleDevices::new();
    let base = 0x2_3500_4000;
    let smmu = 0x2_7130_0000;
    {
        let profile = stela_soc_data::dart::Profile::T8020;
        let legacy = d
            .bus
            .add(Box::new(Dart::for_profile(profile, 16, None).unwrap()));
        assert!(
            d.add_dart_smmu(
                DartId(legacy),
                Range {
                    physical: smmu,
                    bytes: dart::SMMU_BYTES,
                }
            )
            .unwrap_err()
            .to_string()
            .contains("no supported paired SMMU")
        );
        assert!(!d.bus.device::<Dart>(legacy).unwrap().has_smmu());
        assert!(d.read(0, smmu, 4, 0).is_err());
    }
    let owner = d
        .add_dart(
            Range {
                physical: base,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    for range in [
        Range {
            physical: smmu + 4,
            bytes: dart::SMMU_BYTES,
        },
        Range {
            physical: smmu,
            bytes: dart::SMMU_BYTES / 2,
        },
        Range {
            physical: base,
            bytes: dart::SMMU_BYTES,
        },
        Range {
            physical: !(dart::SMMU_BYTES - 1),
            bytes: dart::SMMU_BYTES,
        },
    ] {
        assert!(d.add_dart_smmu(owner, range).is_err());
        assert!(!d.bus.device::<Dart>(owner.0).unwrap().has_smmu());
        assert!(d.read(0, smmu, 4, 0).is_err());
    }
    d.add_dart_smmu(
        owner,
        Range {
            physical: smmu,
            bytes: dart::SMMU_BYTES,
        },
    )
    .unwrap();
    let other = smmu + dart::SMMU_BYTES;
    assert!(
        d.add_dart_smmu(
            owner,
            Range {
                physical: other,
                bytes: dart::SMMU_BYTES
            }
        )
        .is_err()
    );
    assert_eq!(d.read(0, smmu, 4, 0), Ok(0));
    assert!(d.read(0, other, 4, 0).is_err());
}

#[test]
fn pmgr_replays_queued_off_on_and_reset_edges_before_the_next_client_access() {
    for reset in [false, true] {
        let (mut d, _, supply, base) = powered_dart(reset, 1);
        d.write(0, base + 0x1000, 4, 1, 0).unwrap();
        let writes = if reset {
            vec![0x40f, 0x8000_040f, 0x40f, 0xf]
        } else {
            vec![0, 15]
        };
        let mut access = Access::new(0, 0, &mut d.latch);
        let pmgr = d.bus.device_mut::<Pmgr>(d.pmgr.unwrap()).unwrap();
        for value in writes {
            pmgr.write(supply, Width::Word, value, &mut access).unwrap();
        }
        assert_eq!(pmgr.domain_state(supply).unwrap().power, ACTIVE);
        // The failed access must still deliver earlier accepted edges.
        assert!(d.write(0, base + 1, 4, 0, 0).is_err());
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
        assert!(
            d.bus
                .device_mut::<Pmgr>(d.pmgr.unwrap())
                .unwrap()
                .take_transitions()
                .is_empty()
        );
    }
}

#[test]
fn pmgr_unavailable_shared_consumers_keep_native_lifecycle_effects_in_all_directions() {
    use stela_soc_data::board::PowerResource;
    for retention in [false, true] {
        let (mut d, _, supply, base) = powered_dart_with(retention, 1, |states| {
            states.resources.push(PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "/arm-io/unavailable".into(),
                clocks: states.resources[0].clocks.clone(),
                power: vec![],
                owner: None,
                admission: coverage::Admission::Unavailable,
            });
        });
        assert!(d.power_refusals().is_empty());
        d.write(0, base + 0x1000, 4, 1, 0).unwrap();
        d.write(0, supply, 4, 0x3f4, 0).unwrap(); // Read-only nibble is ignored.
        assert!(d.read(0, base + 0x1000, 4, 0).is_err());
        d.write(0, supply, 4, 15, 0).unwrap();
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(1));
        d.write(0, supply, 4, 0x3f0, 0).unwrap(); // Original global sweep encoding.
        assert!(d.read(0, base + 0x1000, 4, 0).is_err());
        assert_eq!(
            d.bus.device::<Pmgr>(d.pmgr.unwrap()).unwrap().state(supply),
            Some(POWER_GATED)
        );
        d.write(0, supply, 4, 15, 0).unwrap();
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(u64::from(retention)));
        d.write(0, base + 0x1000, 4, 1, 0).unwrap();
        for value in [0x40f, 0x8000040f, 0x40f, 15] {
            d.write(0, supply, 4, value, 0).unwrap();
        }
        assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
        assert_eq!(d.read(0, 0x2_3501_0000, 4, 0), Err(DeviceError::NoDevice));
    }
}

#[test]
fn pmgr_domains_no_resource_names_complete_controller_state_with_no_client() {
    // J473AP ISP_SYS: the board exposes no ISP, and XNU power-gates it.
    let supply = 0x2_3b70_0438;
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![coverage::Domain {
            name: "ISP_SYS".into(),
            id: coverage::DomainId {
                die: 0,
                device: 0x7a,
            },
            address: Some(supply),
            parents: [None; 4],
        }],
        Vec::new(),
    );
    let mut d = AppleDevices::new();
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &states.registers,
    )
    .unwrap();
    d.connect_power_resources(&states, &[], &[]).unwrap();
    assert!(d.power_refusals().is_empty() && d.power_clients.is_empty());
    d.write(0, supply, 4, 0x3f0, 0).unwrap();
    assert_eq!(d.read(0, supply, 4, 0), Ok(0x100));
    for value in [0x10f, 0x40f, 0x8000_040f, 0x40f, 15] {
        d.write(0, supply, 4, value, 0).unwrap();
    }
    assert_eq!(d.read(0, supply, 4, 0), Ok(0xff));
}

#[test]
fn pmgr_unavailable_only_domains_model_controller_state_without_functional_device_credit() {
    use stela_soc_data::board::PowerResource;
    let supply = 0x2_3b70_8020;
    let id = coverage::DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![coverage::Domain {
            name: String::new(),
            id,
            address: Some(supply),
            parents: [None; 4],
        }],
        vec![PowerResource {
            clock_role: stela_soc_data::board::ClockRole::Consumer,
            name: "/arm-io/unavailable".into(),
            clocks: vec![id],
            power: vec![],
            owner: None,
            admission: coverage::Admission::Unavailable,
        }],
    );
    let mut d = AppleDevices::new();
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &states.registers,
    )
    .unwrap();
    d.connect_power_resources(&states, &[], &[]).unwrap();
    for state in [0u8, 4, 15, 4, 0, 15] {
        d.write(0, supply, 4, 0x3f0 | u64::from(state), 0).unwrap();
        assert_eq!(
            d.bus.device::<Pmgr>(d.pmgr.unwrap()).unwrap().state(supply),
            Some(state)
        );
    }
    for value in [0x40f, 0x8000040f, 0x40f, 15] {
        d.write(0, supply, 4, value, 0).unwrap();
    }
    assert!(d.power_clients.is_empty());
    assert_eq!(d.read(0, 0x2_3501_0000, 4, 0), Err(DeviceError::NoDevice));
}

#[test]
fn pmgr_multi_supply_inhibits_activity_until_every_clock_and_reset_is_released() {
    let (mut d, dart, supply, base) = powered_dart(true, 2);
    d.write(0, base + 0x1000, 4, 1, 0).unwrap();
    d.write(0, supply, 4, 4, 0).unwrap();
    d.write(0, supply + 8, 4, 4, 0).unwrap();
    d.write(0, supply, 4, 15, 0).unwrap();
    assert!(d.read(0, base + 0x1000, 4, 0).is_err());
    d.write(0, supply + 8, 4, 15, 0).unwrap();
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(1));
    d.write(0, supply, 4, 0x40f, 0).unwrap();
    let mut access = Access::new(0, 0, &mut d.latch);
    assert!(
        d.bus
            .device_mut::<Dart>(dart.0)
            .unwrap()
            .translate(&mut Ram::new(0, 0x1000), 0, 0, 0, false, &mut access)
            .is_err()
    );
    d.write(0, supply, 4, 0x8000_040f, 0).unwrap();
    d.write(0, supply + 8, 4, 0x8000_040f, 0).unwrap();
    d.write(0, supply, 4, 15, 0).unwrap();
    assert!(d.read(0, base + 0x1000, 4, 0).is_err());
    d.write(0, supply + 8, 4, 15, 0).unwrap();
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
}

#[test]
fn pmgr_shared_parents_and_aliases_refuse_only_the_uncovered_hardware_domains() {
    use coverage::{Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let id = |device| DomainId { die: 0, device };
    let (mut d, _, supply, base) = powered_dart_with(false, 3, |states| {
        states.domains[1].parents = [Some(id(1)), None, None, None];
        states.domains.push(Domain {
            name: String::new(),
            id: id(4),
            address: states.domains[1].address,
            parents: [Some(id(3)), None, None, None],
        });
        states.resources[0].clocks = vec![id(2)];
        for (name, device) in [("controller", 1), ("firmware-endpoint", 3)] {
            states.resources.push(PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: name.into(),
                clocks: vec![],
                power: vec![id(device)],
                owner: None,
                admission: coverage::Admission::Required,
            });
        }
    });
    assert_eq!(d.power_refusals().len(), 2);
    d.write(0, base + 0x1000, 4, 1, 0).unwrap();
    for (address, consumer) in [(supply, "controller"), (supply + 16, "firmware-endpoint")] {
        for value in [0, 4, 0x40f, 0x8000_040f] {
            let error = d.write(0, address, 4, value, 0).unwrap_err();
            let message = format!("{error:?}");
            assert!(message.contains(consumer));
            assert!(message.contains("current control 0x000000ff"));
            assert!(message.contains(&format!("requested {value:#010x}")));
            assert_eq!(d.read(0, address, 4, 0), Ok(0xff));
            assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(1));
        }
    }
    // The exclusive child remains connected even though its two alias
    // parents have unrelated unsupported hardware consumers.
    d.write(0, supply + 8, 4, 0x40f, 0).unwrap();
    d.write(0, supply + 8, 4, 0x8000_040f, 0).unwrap();
    d.write(0, supply + 8, 4, 15, 0).unwrap();
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
}

#[test]
fn pmgr_accepted_edges_survive_a_later_wire_delivery_failure() {
    use {stela_contracts::interrupt::InterruptSink, stela_contracts::interrupt::Wire};
    let (mut d, _, supply, base) = powered_dart(false, 1);
    d.write(0, base + 0x1000, 4, 1, 0).unwrap();
    d.latch.set_level(Wire::Aic(AicInput::new(0, 1)), true);
    // PMGR accepts the edge and DART clears first; the unrelated AIC
    // delivery then refuses. No later client may see pre-gate state.
    assert!(d.write(0, supply, 4, 0, 0).is_err());
    assert_eq!(
        d.bus
            .device::<Pmgr>(d.pmgr.unwrap())
            .unwrap()
            .domain_state(supply)
            .unwrap()
            .power,
        POWER_GATED
    );
    assert_eq!(d.pending_service().unwrap().lifecycle().len(), 1);
    assert!(d.write(0, supply, 4, 15, 1).is_err());
    assert_eq!(d.pending_service().unwrap().lifecycle().len(), 2);
    // Supplying the missing owner can resume the edge;
    // neither the earlier power-off nor the later power-on is replayed.
    d.add_aic(0x7000_0000, 32, AicLayout::V1).unwrap();
    d.settle(0, 2, None).unwrap();
    assert!(d.pending_service().is_none());
    assert_eq!(d.read(0, base + 0x1000, 4, 0), Ok(0));
}

#[test]
fn watchdog_routes_separate_side_words_without_claiming_intervening_addresses() {
    let mut d = AppleDevices::new();
    let base = 0x2_3d2b_0000;
    let side = [base + 0xc218, base + 0x8008, base + 0x800c, base + 0x8020];
    let trigger = 0x2_1002_0304;
    d.add_watchdog_with_side(
        Range {
            physical: base,
            bytes: 0x4000,
        },
        Some(&stela_soc_data::board::WatchdogSide {
            words: side.to_vec(),
            triggers: alloc::vec![(trigger, 0xf, 1)],
        }),
        None,
        0,
    )
    .unwrap();
    assert_eq!(d.read(0, side[0], 4, 0), Ok(0));
    d.write(0, side[0], 4, 1, 0).unwrap();
    assert_eq!(d.read(0, side[0], 4, 0), Ok(1));
    d.write(0, side[1], 4, 0, 0).unwrap();
    assert!(d.read(0, side[1], 4, 0).is_err());
    d.write(0, side[2], 4, 0x10, 0).unwrap();
    assert_eq!(d.read(0, side[2], 4, 0), Ok(0x10));
    assert_eq!(d.read(0, side[3], 4, 0), Ok(0));
    assert!(d.read(0, side[0] + 4, 4, 0).is_err());
    assert!(d.read(0, base + 0x5000, 4, 0).is_err());
    d.write(0, trigger, 4, 1, 0).unwrap();
    assert_eq!(d.read(0, trigger, 4, 0), Ok(1));
    assert!(d.read(0, trigger + 4, 4, 0).is_err());
}

#[test]
fn watchdog_shares_declared_flags_and_refuses_unqualified_aliases_before_mapping() {
    let base = 0x2_922b_0000;
    let flags = base + 0x8020;
    let words = [flags, base + 0x802c, base + 0x8008, flags];
    let trigger = 0x2_1002_0304;
    let range = Range {
        physical: base,
        bytes: 0x4000,
    };
    let side = stela_soc_data::board::WatchdogSide {
        words: words.to_vec(),
        triggers: alloc::vec![(trigger, 0xf, 1), (trigger, 0xf, 1)],
    };
    let mut d = AppleDevices::new();
    d.add_watchdog_with_side(range, Some(&side), None, 0)
        .unwrap();
    d.write(0, flags, 4, 0xabcd_0001, 0).unwrap();
    let old = d.read(0, flags, 4, 0).unwrap();
    d.write(0, flags, 4, old | 4, 0).unwrap();
    assert_eq!(d.read(0, flags, 4, 0), Ok(0xabcd_0005));
    assert_eq!(d.read(0, words[2], 4, 0), Ok(0));
    assert!(d.read(0, words[1], 4, 0).is_err());
    assert_eq!(d.read(0, base, 4, 0), Ok(0));
    d.write(0, trigger, 4, 1, 0).unwrap();
    assert_eq!(d.read(0, trigger, 4, 0), Ok(1));
    assert!(d.read(0, flags + 4, 4, 0).is_err());

    let mut bad_cases = Vec::new();
    let mut bad = side.clone();
    bad.words[1] = flags;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words[2] = flags;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words[0] = base;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words[0] += 1;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words[0] = u64::MAX - 3;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words.push(base + 0x9000);
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.words.truncate(1);
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.triggers[1].1 = 3;
    bad_cases.push(bad);
    let mut bad = side.clone();
    bad.triggers[0].0 = flags;
    bad_cases.push(bad);
    for bad in bad_cases {
        let mut d = AppleDevices::new();
        assert!(
            d.add_watchdog_with_side(range, Some(&bad), None, 0)
                .is_err()
        );
        assert!(d.read(0, base, 4, 0).is_err(), "no primary view published");
        assert!(d.read(0, flags, 4, 0).is_err(), "no side view published");
    }
}

#[test]
fn implementation_windows_share_lockdown_energy_and_cpm_without_overlap() {
    let mut d = AppleDevices::new();
    let cpm = 0x2_11e4_0000;
    let cpu = 0x2_1105_0000;
    d.add_lockdown(
        "cpm-impl-reg",
        0x9000,
        Range {
            physical: cpm,
            bytes: 0x10000,
        },
        None,
    )
    .unwrap();
    d.add_cpm_power(cpm, true).unwrap();
    d.add_cpu_energy(cpm + 0x8000, true).unwrap();
    d.add_lockdown(
        "cpu-impl-reg",
        0x5000,
        Range {
            physical: cpu,
            bytes: 0x10000,
        },
        Some((0, 0x8000_0000)),
    )
    .unwrap();
    d.add_cpu_energy(cpu + 0x9000, false).unwrap();
    assert_eq!(d.read(0, cpm + 0x9008, 8, 0), Ok(0));
    assert_eq!(d.read(0, cpu, 8, 0), Ok(0x8000_0001));
    d.write(0, cpm + 0x250, 8, 1 << 63, 0).unwrap();
    assert_eq!(d.read(0, cpm + 0x250, 8, 0), Ok(1 << 63));
    d.write(0, cpm + 0x40f8, 8, 1, 0).unwrap();
    assert!(d.read(0, cpm + 0x40f8, 8, 0).is_err());
    assert!(d.read(0, cpm + 0x1234, 8, 0).is_err());
}

#[test]
fn external_gpio_cs_delimits_nor_commands_across_controller_transfers() {
    use stela_soc_data::board::{Device, GpioBank, SpiNor};
    const SPI: u64 = 0x10000;
    const GPIO: u64 = 0x20000;
    const PIN: u64 = GPIO + 49 * 4;
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x100000, 1024, AicLayout::aic2(0x8000, 0))
        .unwrap();
    let bank = GpioBank {
        profile: stela_soc_data::gpio::Profile::T8101,
        no_restore_pins: Vec::new(),
        physical: GPIO,
        bytes: 0x1000,
        pins: 213,
        groups: 7,
        phandle: 106,
        lines: (199..206).collect(),
        i2c_lines: Vec::new(),
    };
    devices.add_gpio(&bank, &[49], &[]).unwrap();
    devices
        .add_spi_nor(&SpiNor {
            device: Device {
                physical: SPI,
                bytes: 0x4000,
                irq: 749,
            },
            image: stela_soc_data::nor::Image::fresh(),
            select: Some(49),
        })
        .unwrap();
    fn transfer(devices: &mut AppleDevices, bytes: &[u8]) -> Vec<u8> {
        devices.write(0, SPI, 4, 0, 0).unwrap();
        devices.write(0, SPI, 4, 12, 0).unwrap();
        devices
            .write(0, SPI + 0x34, 4, bytes.len() as u64, 0)
            .unwrap();
        devices
            .write(0, SPI + 0x4c, 4, bytes.len() as u64, 0)
            .unwrap();
        for &b in bytes {
            devices.write(0, SPI + 0x10, 4, u64::from(b), 0).unwrap();
        }
        devices.write(0, SPI, 4, 1, 0).unwrap();
        (0..bytes.len())
            .map(|_| devices.read(0, SPI + 0x20, 4, 0).unwrap() as u8)
            .collect()
    }
    devices.write(0, PIN, 4, 2, 0).unwrap();
    transfer(&mut devices, &[0x06]);
    devices.write(0, PIN, 4, 3, 0).unwrap();
    devices.write(0, PIN, 4, 2, 0).unwrap();
    transfer(&mut devices, &[0x02, 0, 0, 0]);
    transfer(&mut devices, &[0x42, 0x24]);
    devices.write(0, PIN, 4, 3, 0).unwrap();
    devices.write(0, PIN, 4, 2, 0).unwrap();
    transfer(&mut devices, &[0x03, 0, 0, 0]);
    assert_eq!(transfer(&mut devices, &[0xff, 0xff]), [0x42, 0x24]);
    devices.write(0, PIN, 4, 3, 0).unwrap();
    // An incomplete address cannot release CS or mutate the GPIO word.
    devices.write(0, PIN, 4, 2, 0).unwrap();
    transfer(&mut devices, &[0x20, 0]);
    assert!(devices.write(0, PIN, 4, 3, 0).is_err());
    assert_eq!(devices.read(0, PIN, 4, 0), Ok(2));
}

#[test]
fn ans_dma_and_sart_idle_share_the_nvme_aperture_without_aliasing() {
    use crate::ans;
    use stela_contracts::dma::test_support::Ram;
    use stela_drivers::block::RamDisk;
    use stela_soc_data::board::AnsController;
    const NVME: u64 = 0x2_7bcc_0000;
    const SART: u64 = 0x2_7bc5_0000;
    const WRAPPER: u64 = 0x2_7740_0000;
    const VDMA: u64 = 0x2_7b10_0000;
    const RAM: u64 = 0x8_0000_0000;
    const SUPPLY: u64 = 0x2_3b70_0428;
    let placement = AnsController {
        geometry: stela_soc_data::ans::QueueGeometry::DEFAULT,
        wrapper: (WRAPPER, 0x6c000),
        nvme: (NVME, 0x60000),
        vdma: Some((VDMA, 0x44000)),
        sart: (SART, 0xc000),
        sart_layout: stela_soc_data::sart::Layout::V3,
        sart_idle: NVME + 0x13e8,
        sart_canary: None,
        sart_exclusive_bounds: false,
        interrupts: [718, 717, 720, 719],
        completion_irq: 724,
        namespaces: alloc::vec![stela_soc_data::ans::Description {
            id: 1,
            kind: 1,
            blocks: 0
        }],
        clock_gates: alloc::vec![coverage::DomainId {
            die: 0,
            device: 125
        }],
        power_gates: alloc::vec![coverage::DomainId {
            die: 0,
            device: 125
        }],
        quiesced: false,
    };
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    devices
        .add_ans(
            &placement,
            Box::new(RamDisk::new("backing", 16, 4096)),
            stela_formats::builds::DEFAULT,
        )
        .unwrap();
    assert_eq!(devices.power_devices.len(), 4);
    assert!(
        devices
            .power_devices
            .iter()
            .filter(|bank| bank.physical != 0x2_3b0c_0000)
            .all(|bank| bank.physical == WRAPPER)
    );
    connect_native_power(&mut devices, SUPPLY, &[("/arm-io/ans", WRAPPER)], true);
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    // The roll call lists 0x20, which RTBuddy publishes as the
    // ANS2Endpoint1 service the NVMe driver waits for.
    walk_rtkit_start(&mut devices, WRAPPER);
    // The ASC has no owned mailbox work and checks native cold reset;
    // the later NVMe bank must veto before either it or PMGR changes.
    let pmgr_before = devices.read(0, SUPPLY, 4, 0).unwrap();
    devices.write(0, SUPPLY, 4, 0x40f, 0).unwrap();
    let quiesced = devices.read(0, SUPPLY, 4, 0).unwrap();
    let failure = devices.write(0, SUPPLY, 4, 0x8000_040f, 0).unwrap_err();
    assert!(
        matches!(failure, DeviceError::Refused(reason) if reason.to_string().contains("ANS physical reset"))
    );
    assert_eq!(devices.read(0, SUPPLY, 4, 0), Ok(quiesced));
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    devices.write(0, SUPPLY, 4, 15, 0).unwrap();
    assert_eq!(devices.read(0, SUPPLY, 4, 0), Ok(pmgr_before));
    devices.write(0, WRAPPER + 0x8800, 8, 1, 0).unwrap();
    // A request the ANS firmware does not implement is reported once.
    devices.write(0, WRAPPER + 0x8808, 8, 0x20, 0).unwrap();
    assert_eq!(devices.take_unmodeled().len(), 1);
    devices.write(0, NVME + 0x13e8, 4, 1, 0).unwrap();
    assert_eq!(devices.read(0, NVME + 0x13e8, 4, 0), Ok(1));
    assert_eq!(devices.read(0, NVME + 0x1300, 4, 0), Ok(0xde71_ce55));
    assert!(devices.read(0, NVME + 0x13ec, 4, 0).is_err());
    for (at, value) in [
        (SART + 0x40, RAM / stela_soc_data::sart::PAGE),
        (SART + 0x80, 16),
        (SART, 0xff),
        (NVME + 0x24, 0x0001_0001),
        (NVME + 0x28, RAM),
        (NVME + 0x30, RAM + 0x1000),
        (NVME + 0x28108, RAM + 0x2000),
        (NVME + 0x24908, 1),
        (NVME + 0x14, 0x0046_0001),
    ] {
        let width = if [NVME + 0x28, NVME + 0x30, NVME + 0x28108].contains(&at) {
            8
        } else {
            4
        };
        devices.write(0, at, width, value, 0).unwrap();
    }
    let mut ram = Ram::new(RAM, 0x10000);
    let mut command = [0; 64];
    command[0] = 6;
    command[24..32].copy_from_slice(&(RAM + 0x4000).to_le_bytes());
    command[40] = 1;
    let mut tcb = [0; 128];
    tcb[1] = 1;
    tcb[24..32].copy_from_slice(&(RAM + 0x4000).to_le_bytes());
    assert!(Memory::write(&mut ram, RAM, &command));
    assert!(Memory::write(&mut ram, RAM + 0x2000, &tcb));
    devices.write(0, NVME + 0x2490c, 4, 0, 0).unwrap();
    // A real accepted command is frozen through each gate.
    for gate in [4, 0x40f, 0] {
        devices.write(0, SUPPLY, 4, gate, 0).unwrap();
        devices.transfer(0, &mut ram, 0).unwrap();
        let mut empty = [1; 16];
        assert!(Memory::read(&mut ram, RAM + 0x1000, &mut empty));
        assert_eq!(empty, [0; 16]);
        devices.write(0, SUPPLY, 4, 15, 0).unwrap();
    }
    devices.transfer(0, &mut ram, 0).unwrap();
    let mut completion = [0; 16];
    assert!(Memory::read(&mut ram, RAM + 0x1000, &mut completion));
    assert_eq!(completion[14], 1);
    let mut model = [0; 10];
    assert!(Memory::read(&mut ram, RAM + 0x4000 + 24, &mut model));
    assert_eq!(&model, b"Stela ANS2");
    // The completed tag's VDMA cache entry holds no DMA permissions, the
    // state SPTM's UNMAP_PAGES requires; other words have no model.
    let entry = VDMA + ans::VDMA_CACHE_OFFSET;
    assert_eq!(devices.read(0, entry, 4, 0), Ok(0));
    assert_eq!(devices.read(0, entry + 63 * 32, 4, 0), Ok(0));
    assert!(devices.read(0, entry + 4, 4, 0).is_err());
    assert!(devices.write(0, entry, 4, 0, 0).is_err());
    assert!(
        devices
            .read(0, entry + ans::VDMA_CACHE_BYTES, 4, 0)
            .is_err()
    );
}

#[test]
fn an_aux_gpio_bank_reads_the_lines_its_board_drives() {
    use stela_soc_data::board::{AuxGpioBank, GpioBank};
    const BANK: u64 = 0x2_4a82_0000;
    const HPD: u32 = 0x31;
    let aux = AuxGpioBank {
        name: "aop-gpio".into(),
        bank: GpioBank {
            profile: stela_soc_data::gpio::Profile::T8101,
            no_restore_pins: Vec::new(),
            physical: BANK,
            bytes: 0x4000,
            pins: 0x36,
            groups: 7,
            phandle: 0x6b,
            lines: alloc::vec![0x12d, 0x12e, 0x12f, 0x130, 0x131, 0x132, 0x133],
            i2c_lines: Vec::new(),
        },
        client_pins: alloc::vec![0x28, HPD],
    };
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    devices.add_aux_gpio(&aux).unwrap();
    assert!(devices.add_aux_gpio(&aux).is_err());
    let hpd = devices.gpio_line(0x6b, HPD).unwrap();
    assert!(devices.gpio_line(0x6b, 0x30).is_err());
    assert!(devices.gpio_line(0x6c, HPD).is_err());
    // An interrupt-disabled input with its receiver on reads the net.
    let pin = BANK + 4 * u64::from(HPD);
    devices.write(0, pin, 4, 0xe | 1 << 9, 0).unwrap();
    assert_eq!(devices.read(0, pin, 4, 0).unwrap() & 1, 1);
    hpd.pull_low(true);
    assert_eq!(devices.read(0, pin, 4, 0).unwrap() & 1, 0);
}

/// Every message waiting in an ASC wrapper's outbox, as (endpoint,
/// payload).
fn mailbox_drain(devices: &mut AppleDevices, wrapper: u64) -> Vec<(u64, u64)> {
    let mut messages = Vec::new();
    while devices.read(0, wrapper + 0x8114, 4, 0).unwrap() & 1 << 17 == 0 {
        let payload = devices.read(0, wrapper + 0x8830, 8, 0).unwrap();
        let endpoint = devices.read(0, wrapper + 0x8838, 8, 0).unwrap() & 0xff;
        messages.push((endpoint, payload));
    }
    messages
}

/// The AP side of the RTKit start-up handshake a restore boot logged
/// for a coprocessor listing one application endpoint 0x20, word for
/// word, up to the client driver's STARTEP of it, except that group 0
/// now also lists the crashlog endpoint. The coprocessor must end
/// enumeration with the power-on announcement the AP waits for.
#[test]
fn a_quiesced_ans2_nub_waits_for_the_ap_wake_before_its_hello() {
    use stela_drivers::block::RamDisk;
    use stela_soc_data::board::AnsController;
    const NVME: u64 = 0x2_7bcc_0000;
    const SART: u64 = 0x2_7bc5_0000;
    const WRAPPER: u64 = 0x2_7740_0000;
    const VDMA: u64 = 0x2_7b10_0000;
    const SUPPLY: u64 = 0x2_3b70_0428;
    let placement = AnsController {
        geometry: stela_soc_data::ans::QueueGeometry::DEFAULT,
        wrapper: (WRAPPER, 0x6c000),
        nvme: (NVME, 0x60000),
        vdma: Some((VDMA, 0x44000)),
        sart: (SART, 0xc000),
        sart_layout: stela_soc_data::sart::Layout::V3,
        sart_idle: NVME + 0x13e8,
        sart_canary: None,
        sart_exclusive_bounds: false,
        interrupts: [718, 717, 720, 719],
        completion_irq: 724,
        namespaces: alloc::vec![stela_soc_data::ans::Description {
            id: 1,
            kind: 1,
            blocks: 0
        }],
        clock_gates: alloc::vec![coverage::DomainId {
            die: 0,
            device: 125
        }],
        power_gates: alloc::vec![coverage::DomainId {
            die: 0,
            device: 125
        }],
        quiesced: true,
    };
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    devices
        .add_ans(
            &placement,
            Box::new(RamDisk::new("backing", 16, 4096)),
            stela_formats::builds::DEFAULT,
        )
        .unwrap();

    connect_native_power(&mut devices, SUPPLY, &[("/arm-io/ans", WRAPPER)], true);
    // The boot chain quiesced it: no HELLO until the AP's IOP power 0x220.
    assert!(mailbox_drain(&mut devices, WRAPPER).is_empty());
    devices
        .write(0, WRAPPER + 0x8800, 8, 0x0060_0000_0000_0220, 0)
        .unwrap();
    devices.write(0, WRAPPER + 0x8808, 8, 0, 0).unwrap();
    assert_eq!(
        mailbox_drain(&mut devices, WRAPPER),
        [(0, 0x0010_0000_000c_000c)]
    );
}

fn walk_rtkit_start(devices: &mut AppleDevices, wrapper: u64) {
    let send = |devices: &mut AppleDevices, payload| {
        devices.write(0, wrapper + 0x8800, 8, payload, 0).unwrap();
        devices.write(0, wrapper + 0x8808, 8, 0, 0).unwrap();
        mailbox_drain(devices, wrapper)
    };
    assert_eq!(
        mailbox_drain(devices, wrapper),
        [(0, 0x0010_0000_000c_000c)]
    );
    assert_eq!(
        send(devices, 0x0020_0000_000c_000c),
        [(0, 0x0080_0000_0000_0003)]
    );
    assert_eq!(
        send(devices, 0x0080_0000_0000_0001),
        [(0, 0x0088_0001_0000_0001)]
    );
    assert_eq!(
        send(devices, 0x0088_0001_0000_0000),
        [(0, 0x0070_0000_0000_0020)]
    );
    assert!(send(devices, 0x0050_0020_0000_0002).is_empty());
}

#[test]
fn pmp_places_a_running_user_managed_wrapper() {
    use stela_soc_data::board::PmpController;
    const WRAPPER: u64 = 0x2_3bc0_0000;
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    devices
        .add_pmp(
            &PmpController {
                wrapper: (WRAPPER, 0x6c000),
                interrupts: [0x275, 0x274, 0x277, 0x276],
                quiesced: false,
                mapper: None,
            },
            stela_formats::builds::DEFAULT,
        )
        .unwrap();
    assert_eq!(devices.power_devices.len(), 2);
    assert_eq!(devices.power_devices[1].physical, WRAPPER);
    let supply = 0x2_3b70_0410;
    connect_native_power(&mut devices, supply, &[("/arm-io/pmp", WRAPPER)], false);
    // Its committed HELLO survives a real clock gate without restarting
    // native firmware or inventing an additional message on release.
    devices.write(0, supply, 4, 4, 0).unwrap();
    assert!(devices.read(0, WRAPPER + 0x8830, 8, 0).is_err());
    devices.write(0, supply, 4, 15, 0).unwrap();
    // Started and running; its HELLO waits in the outbox.
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    assert_eq!(devices.coprocessor_diagnostics().len(), 1);
    assert!(devices.coprocessor_diagnostics()[0].starts_with("PMP "));
    // A user-power-managed nub is still awaited to running.
    walk_rtkit_start(&mut devices, WRAPPER);
}

#[test]
fn gfx_native_control_cannot_bind_over_another_mounted_bank() {
    use stela_soc_data::agx::generation;
    use stela_soc_data::board::GfxController;
    const WRAPPER: u64 = 0x2_0640_0000;
    const CONTROL: u64 = WRAPPER + 0x14_0120;
    let placement = GfxController {
        gpu: generation::by_chip(0x8112).unwrap(),
        wrapper: (WRAPPER, 0x6c000),
        interrupts: [0x2c6, 0x2c5, 0x2c8, 0x2c7],
        sgx: vec![(0x2_0400_0000, 0x400_0000, 0)],
        ttbat: 0x8_0000_0000,
        shared_l2: 0x8_0000_8000,
        handoff: 0x8_000c_c000,
    };
    // Reject a complete foreign word and a foreign upper half. Declared
    // SGX geometry alone cannot check either physical bus layout.
    for (base, bytes) in [(CONTROL, 8), (CONTROL + 4, 4)] {
        let mut devices = AppleDevices::new();
        let foreign = devices.bus.add(Box::new(CheckedPowerBank {
            reject: true,
            committed: Vec::new(),
        }));
        devices.bus.map(foreign, base, bytes, 0).unwrap();
        assert_eq!(
            devices.add_gfx(&placement, stela_formats::builds::DEFAULT),
            Err(refuse!(
                "GFX native ASC control is not wholly owned by its mounted SGX bank"
            ))
        );
        assert!(devices.gfx_masters.is_empty());
        assert_eq!(devices.bus.route(base, Width::Word), Ok((foreign, 0)));
        assert!(
            devices
                .bus
                .device::<CheckedPowerBank>(foreign)
                .unwrap()
                .committed
                .is_empty()
        );
    }
}

#[test]
fn pro_max_and_ultra_gfx_keep_their_sparse_original_banks() {
    use stela_soc_data::agx::generation;
    use stela_soc_data::board::GfxController;
    // 26A434 original ADTs: J314s/c, J375c/d, J414s/c and J180d.
    // arm-io/ranges translates the child bases by 0x2_0000_0000.
    // G13X/G14X use their own callback programs, not G14G's private
    // wrapper+0x140120 word.
    const SGX: u64 = 0x4_0400_0000;
    const WRAPPER: u64 = 0x4_0640_0000;
    for chip in [0x6000, 0x6001, 0x6002, 0x6020, 0x6021, 0x6022] {
        let info_bytes = if chip < 0x6020 { 0x20000 } else { 0x16c000 };
        let placement = GfxController {
            gpu: generation::by_chip(chip).unwrap(),
            wrapper: (WRAPPER, 0x6c000),
            interrupts: [0x2c6, 0x2c5, 0x2c8, 0x2c7],
            sgx: vec![(SGX, 0x100000, 0), (SGX + 0xd00000, info_bytes, 0xd00000)],
            ttbat: 0x100_0000_0000,
            shared_l2: 0x100_0000_8000,
            handoff: 0x8_000c_c000,
        };
        let mut devices = AppleDevices::new();
        devices
            .add_gfx(&placement, stela_formats::builds::DEFAULT)
            .unwrap();
        let (asc, block) = devices.gfx_masters[0];
        assert_eq!(
            devices.bus.route(WRAPPER + 0x8110, Width::Word),
            Ok((asc, 0x8110))
        );
        assert_eq!(
            devices.bus.route(SGX + 0xd04000, Width::Word),
            Ok((block, 0xd04000))
        );
        assert!(devices.read(0, SGX + 0xd04000, 4, 0).is_ok());
        // No invented SGX aperture or successful control-word effect.
        assert!(
            devices
                .bus
                .route(WRAPPER + 0x14_0120, Width::Double)
                .is_err()
        );
        assert!(devices.bus.route(SGX + 0x200000, Width::Word).is_err());
    }
}

#[test]
fn gfx_firmware_reads_its_initdata_through_the_uat_the_sgx_mmu_roots() {
    use crate::agx::{firmware, uat as table};
    use stela_contracts::dma::test_support::Ram;
    use stela_soc_data::agx::generation;
    use stela_soc_data::board::GfxController;
    const WRAPPER: u64 = 0x2_0640_0000;
    const SGX: u64 = 0x2_0400_0000;
    const RAM: u64 = 0x8_0000_0000;
    const SUPPLY: u64 = 0x2_3b70_0430;
    let placement = GfxController {
        gpu: generation::by_chip(0x8112).unwrap(),
        wrapper: (WRAPPER, 0x6c000),
        interrupts: [0x2c6, 0x2c5, 0x2c8, 0x2c7],
        sgx: vec![(SGX, 0x400_0000, 0)],
        ttbat: RAM,
        shared_l2: RAM + 0x8000,
        handoff: RAM + 0xc000,
    };
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    devices
        .add_gfx(&placement, stela_formats::builds::DEFAULT)
        .unwrap();
    assert_eq!(devices.power_devices.len(), 3);
    assert_eq!(devices.power_devices[1].physical, WRAPPER);
    assert_eq!(devices.power_devices[2].physical, SGX);
    connect_native_power(
        &mut devices,
        SUPPLY,
        &[("/arm-io/gfx-asc", WRAPPER), ("/arm-io/sgx", SGX)],
        true,
    );
    // The coprocessor is running and the sgx identity reads through the bus.
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    assert_eq!(devices.read(0, SGX + 0xd0_4000, 4, 0).unwrap() >> 24, 5);
    assert!(devices.read(0, SGX + 0x10, 4, 0).is_err());
    // The wrapper inside the sgx window stays the ASC's.
    assert!(devices.read(0, WRAPPER + 0x8110, 4, 0).is_ok());
    let (_, block) = devices.gfx_masters[0];
    assert_eq!(
        devices.bus.route(WRAPPER + 0x14_0120, Width::Double),
        Ok((block, 0x254_0120))
    );

    // The TTBAT, context 0's empty TTBR1 root, and one firmware page of
    // the shared kernel window through the shared L2, as SPTM leaves
    // them.
    let mut ram = Ram::new(RAM, 0x4000 * 5);
    let va = 0xffff_ffa0_0000_0000_u64;
    let (_, idx) = table::indices(va);
    let [ttbr0, ttbr1] = table::encode_gptbat(0, 0, RAM + 0x4000);
    ram.put_u64(RAM, ttbr0 & !1);
    ram.put_u64(RAM + 8, ttbr1);
    ram.put_u64(
        RAM + 0x8000 + idx[1] as u64 * 8,
        table::encode_table(RAM + 0xc000),
    );
    let flags = table::Flags {
        access: table::Access::Firmware,
        memory: table::MemoryAttr::Uncached,
        global: true,
        pxn: false,
        uxn: true,
        gpu_access: true,
    };
    ram.put_u64(
        RAM + 0xc000 + idx[2] as u64 * 8,
        table::encode_pte(RAM + 0x1_0000, &flags),
    );
    // The kext's root: magic, the +0x28 words and its object pointers.
    ram.put_u64(RAM + 0x1_0000, generation::MAGIC_G13_G14G);
    ram.put_u64(RAM + 0x1_0028, 1 << 32);
    for field in [0x08, 0x18, 0x20, 0xa8, 0xb0, 0xb8, 0xc0] {
        ram.put_u64(RAM + 0x1_0000 + field, va + 0x1000);
    }
    // The boot stage handed the MMU off enabled at the TTBAT, so
    // setupMMUConfig returns at once.
    assert_eq!(devices.read(0, SGX + 0xd0_8000, 4, 0), Ok(1));
    assert_eq!(devices.read(0, SGX + 0xd0_802c, 4, 0), Ok(RAM >> 14));

    // RTKit start-up over the mailbox.
    let send = |devices: &mut AppleDevices, endpoint: u64, payload: u64| {
        devices.write(0, WRAPPER + 0x8800, 8, payload, 0).unwrap();
        devices.write(0, WRAPPER + 0x8808, 8, endpoint, 0).unwrap();
    };
    let receive = |devices: &mut AppleDevices| {
        let payload = devices.read(0, WRAPPER + 0x8830, 8, 0).unwrap();
        let endpoint = devices.read(0, WRAPPER + 0x8838, 8, 0).unwrap() & 0xff;
        (endpoint, payload)
    };
    let mgmt = |kind: u64, fields: u64| kind << 52 | fields;
    assert_eq!(receive(&mut devices), (0, mgmt(1, 12 | 12 << 16)));
    send(&mut devices, 0, mgmt(2, 12 | 12 << 16));
    // Group 0: the management and crashlog endpoints.
    assert_eq!(receive(&mut devices), (0, mgmt(8, 3)));
    send(&mut devices, 0, mgmt(8, 3));
    // 0x20 and 0x21 are bits 0 and 1 of the last group, 1.
    assert_eq!(receive(&mut devices), (0, mgmt(8, 1 << 51 | 1 << 32 | 3)));
    send(&mut devices, 0, mgmt(8, 1 << 51 | 1 << 32 | 3));
    assert_eq!(receive(&mut devices), (0, mgmt(7, 0x20)));
    for endpoint in [0x20, 0x21] {
        send(&mut devices, 0, mgmt(5, endpoint << 32 | 2));
    }
    devices.write(0, SUPPLY, 4, 0x40f, 0).unwrap();
    let before = devices.read(0, SUPPLY, 4, 0).unwrap();
    let failure = devices.write(0, SUPPLY, 4, 0x8000_040f, 0).unwrap_err();
    assert!(
        matches!(failure, DeviceError::Refused(reason) if reason.to_string().contains("SGX physical reset"))
    );
    assert_eq!(devices.read(0, SUPPLY, 4, 0), Ok(before));
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    devices.write(0, SUPPLY, 4, 15, 0).unwrap();
    // notifyFirmwareStarted's initdata message, then the firmware's read.
    send(&mut devices, 0x20, 0x81 << 48 | (va & ((1 << 44) - 1)));
    for gate in [4, 0x40f, 0] {
        devices.write(0, SUPPLY, 4, gate, 0).unwrap();
        devices.transfer(0, &mut ram, 0).unwrap();
        assert!(
            !devices
                .bus
                .device::<Asc>(devices.gfx_masters[0].0)
                .unwrap()
                .firmware::<firmware::Coprocessor>()
                .unwrap()
                .protocol()
                .with(|state| state.initdata_checked)
        );
        devices.write(0, SUPPLY, 4, 15, 0).unwrap();
    }
    devices.transfer(0, &mut ram, 0).unwrap();
    let checked = devices
        .bus
        .device::<Asc>(devices.gfx_masters[0].0)
        .and_then(|asc| asc.firmware::<firmware::Coprocessor>())
        .map(|gfx| gfx.protocol().with(|state| state.initdata_checked));
    assert_eq!(checked, Some(true));
}

#[test]
fn smc_places_a_running_wrapper_its_shared_page_and_watchdog() {
    use stela_soc_data::board::SmcController;
    const WRAPPER: u64 = 0x2_3e40_0000;
    const WATCHDOG: u64 = 0x2_3e81_0000;
    const REGION: u64 = 0x2_3fe0_0000;
    let placement = SmcController {
        wrapper: (WRAPPER, 0x6c000),
        interrupts: [0x1f4, 0x1f3, 0x1f6, 0x1f5],
        watchdog: Some((WATCHDOG, 0x14)),
        region: (REGION, 0x10_0000),
        gpio_keys: alloc::vec![smc::code(b"gP16")],
        quiesced: true,
        wake_bits: stela_soc_data::smc::WakeBits::default(),
    };
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    assert!(devices.power_button(true).is_err());
    devices
        .add_smc(&placement, stela_formats::builds::DEFAULT)
        .unwrap();
    // The press waits in the key endpoint until the AP enables
    // notifications.
    devices.power_button(true).unwrap();
    devices.power_button(false).unwrap();
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    // A held button turns the machine off with no message from the AP.
    devices.power_button_held().unwrap();
    let off = devices.take_reset().unwrap();
    assert_eq!(off.transition, Transition::PowerOff);
    assert_eq!(off.reason, POWER_BUTTON_HELD);
    assert_eq!(devices.take_reset(), None);
    devices.write(0, REGION + 8, 8, 0x1122_3344, 0).unwrap();
    assert_eq!(devices.read(0, REGION + 8, 8, 0), Ok(0x1122_3344));
    // The coprocessor's own memory past the shared page has no model.
    assert!(devices.read(0, REGION + smc::SRAM_BYTES, 4, 0).is_err());
    assert_eq!(devices.read(0, WATCHDOG + 0xc, 4, 0), Ok(0));
    devices.write(0, WATCHDOG + 0xc, 4, 0, 0).unwrap();
    assert_eq!(devices.coprocessor_diagnostics().len(), 1);
}

#[test]
fn the_kept_rtc_anchor_is_the_pmu_upcount_at_the_latest_board_entry() {
    let hz = crate::counter::FREQUENCY_HZ;
    let wall = stela_soc_data::pmu::WallClock {
        unix_seconds: 1_792_000_000,
        counter: 100,
    };
    let loaded = pmu::RtcAnchor {
        ticks: 77 * pmu::RTC_HZ,
        unix_seconds: Some(wall.unix_seconds - 60),
    };
    let epoch = pmu::RtcEpoch::anchored(100, Some(wall), Some(loaded));
    assert_eq!(epoch.ticks, 137 * pmu::RTC_HZ);
    let mut devices = AppleDevices::new();
    devices.pmu_clocks.push(pmu::RtcClock::new(epoch).unwrap());
    // No origin kept, or no board entry yet: nothing to keep.
    assert_eq!(devices.rtc_anchor(), None);
    devices.keep_rtc_origin(Some(wall), Some(loaded));
    assert_eq!(devices.rtc_anchor(), None);
    devices.observe_time(100 + 5 * hz, "test");
    assert_eq!(
        devices.rtc_anchor(),
        Some(pmu::RtcAnchor {
            ticks: 142 * pmu::RTC_HZ,
            unix_seconds: Some(wall.unix_seconds + 5),
        })
    );
    // The next boot, 100 s later on the host's clock, continues from it.
    let next = pmu::RtcEpoch::anchored(
        0,
        Some(stela_soc_data::pmu::WallClock {
            unix_seconds: wall.unix_seconds + 105,
            counter: 0,
        }),
        devices.rtc_anchor(),
    );
    assert_eq!(next.ticks, 242 * pmu::RTC_HZ);
}

#[test]
fn rtc_publication_precedes_every_timed_service_and_refuses_stale_effects() {
    let epoch = pmu::RtcEpoch {
        counter: 100,
        ticks: 1234,
    };
    let clock = pmu::RtcClock::new(epoch).unwrap();
    let mut devices = AppleDevices::new();
    devices.rtc_clock = Some(clock.clone());
    let timer = 0x2_922b_0000;
    devices
        .add_watchdog(
            &[Range {
                physical: timer,
                bytes: 0x4000,
            }],
            None,
            epoch.counter,
        )
        .unwrap();
    assert!(clock.published_ticks().is_err());
    devices.write(0, timer + 4, 4, 50, epoch.counter).unwrap();
    assert_eq!(clock.published_ticks(), Ok(epoch.ticks));
    let hz = crate::counter::FREQUENCY_HZ;
    let ticks = |seconds| epoch.ticks + seconds * pmu::RTC_HZ;
    devices.read(0, timer + 4, 4, epoch.counter + hz).unwrap();
    assert_eq!(clock.published_ticks(), Ok(ticks(1)));
    let mut ram = Ram::new(0x800000000, 0x4000);
    devices
        .transfer(0, &mut ram, epoch.counter + 2 * hz)
        .unwrap();
    assert_eq!(clock.published_ticks(), Ok(ticks(2)));
    assert_eq!(
        devices.system_register(
            0,
            Encoding::new(3, 0, 0, 0, 0),
            None,
            epoch.counter + 3 * hz
        ),
        None
    );
    assert_eq!(clock.published_ticks(), Ok(ticks(3)));
    devices.lines(0, epoch.counter + 4 * hz);
    assert_eq!(clock.published_ticks(), Ok(ticks(4)));
    devices.deadline(epoch.counter + 5 * hz);
    assert_eq!(clock.published_ticks(), Ok(ticks(5)));
    // An old counter cannot change the timer or replace the last sample.
    assert!(devices.write(0, timer + 4, 4, 99, epoch.counter).is_err());
    assert_eq!(clock.published_ticks(), Ok(ticks(5)));
    assert_eq!(
        devices.read(0, timer + 4, 4, epoch.counter + 5 * hz),
        Ok(50)
    );
    // Lines/deadline cannot return Result; they keep an explicit halt.
    devices.lines(0, epoch.counter);
    let reset = devices.reset.take().unwrap();
    assert_eq!(reset.transition, Transition::Halt);
    assert!(reset.reason.contains("RTC"));
    devices.deadline(epoch.counter);
    assert_eq!(devices.reset.take().unwrap().transition, Transition::Halt);
}

#[test]
fn native_dma_constructors_preflight_every_owner_before_publication() {
    const DART: u64 = 0x235004000;
    const AES: u64 = 0x23500c000;
    const WRAPPER: u64 = 0x271c00000;
    struct UnusedScreen;
    impl Scanout for UnusedScreen {
        fn mode(&self) -> ScanoutMode {
            panic!("DCP used scanout before owner preflight")
        }
        fn present(&mut self, _: &[u8]) -> Result<()> {
            unreachable!()
        }
    }
    fn refuse(d: &mut AppleDevices, dart: DartId, input: usize) {
        assert!(
            d.add_aes(
                &[
                    Range {
                        physical: AES,
                        bytes: aes::BLOCK_BYTES
                    },
                    Range {
                        physical: 0x23d2dc000,
                        bytes: 0x8000
                    }
                ],
                j473_aes(),
                None,
                (dart, input),
                None
            )
            .unwrap_err()
            .to_string()
            .contains("DART")
        );
        assert!(
            d.add_dcpext(
                DcpextNub {
                    wrapper: Range {
                        physical: WRAPPER,
                        bytes: 0x6c000
                    },
                    interrupts: [1, 2, 3, 4],
                    quiesced: false,
                    role: "DCPEXT",
                    identity: stela_soc_data::dcp::ExternalIdentity::Unnumbered
                },
                Box::new(UnusedScreen),
                DcpextDma {
                    link: (dart, input),
                    window: None,
                    surface: None
                },
                stela_formats::builds::DEFAULT
            )
            .unwrap_err()
            .to_string()
            .contains("DART")
        );
        let mut usb = usb_placement();
        usb.stream = input as u32;
        assert!(
            d.add_usb(&usb, dart)
                .unwrap_err()
                .to_string()
                .contains("DART")
        );
        assert!(d.masters.is_empty() && d.dcp_masters.is_empty() && d.usb_masters.is_empty());
        for address in [AES, WRAPPER + 0x8114, usb.ranges[0].0 + 0x20] {
            assert!(d.read(0, address, 4, 0).is_err());
        }
    }
    for profile in [
        stela_soc_data::dart::Profile::T6000,
        stela_soc_data::dart::Profile::T8110,
    ] {
        let mut other = AppleDevices::new();
        let absent = other
            .add_dart(
                Range {
                    physical: DART,
                    bytes: 0x4000,
                },
                2,
                None,
                None,
            )
            .unwrap();
        let mut d = AppleDevices::new();
        refuse(&mut d, absent, 1);
        d.add_aic(0x23b0c0000, 1152, AicLayout::aic2(0x8000, 0))
            .unwrap();
        let (Controller::V1(id) | Controller::V2(id)) = d.aic.unwrap();
        refuse(&mut d, DartId(id), 1);
        let owner = d
            .add_dart_group_for_profile(
                profile,
                &[Range {
                    physical: DART,
                    bytes: 0x4000,
                }],
                2,
                None,
                None,
            )
            .unwrap()[0];
        // Validate constructed capacity2, not the generation's limit.
        refuse(&mut d, owner, 2);
        refuse(&mut d, owner, u32::MAX as usize);
        assert!(
            d.add_dcpext(
                DcpextNub {
                    wrapper: Range {
                        physical: WRAPPER,
                        bytes: 0x6c000
                    },
                    interrupts: [1, 2, 3, 4],
                    quiesced: false,
                    role: "DCPEXT",
                    identity: stela_soc_data::dcp::ExternalIdentity::Unnumbered
                },
                Box::new(UnusedScreen),
                DcpextDma {
                    link: (owner, 1),
                    window: None,
                    surface: Some((owner, 2))
                },
                stela_formats::builds::DEFAULT
            )
            .unwrap_err()
            .to_string()
            .contains("DCP surface")
        );
        assert!(d.coprocessors.is_empty() && d.dcp_masters.is_empty());
        assert!(d.read(0, WRAPPER + 0x8114, 4, 0).is_err());
        // Every refusal left the apertures available for a valid native retry.
        d.add_aes(
            &[
                Range {
                    physical: AES,
                    bytes: aes::BLOCK_BYTES,
                },
                Range {
                    physical: 0x23d2dc000,
                    bytes: 0x8000,
                },
            ],
            j473_aes(),
            None,
            (owner, 1),
            None,
        )
        .unwrap();
        assert_eq!(d.masters[0].stream, 1);
        let _phy = add_usb_phy(&mut d);
        d.add_usb(&usb_placement(), owner).unwrap();
        assert_eq!(d.usb_masters[0].stream, 1);
    }
}

#[test]
fn mounted_dcpext_kept_pm_gates_suppress_real_surface_dma_and_scanout() {
    use crate::dcp::{link, swap};
    struct Screen(Rc<RefCell<usize>>);
    impl Scanout for Screen {
        fn mode(&self) -> ScanoutMode {
            ScanoutMode {
                width: 1,
                height: 1,
                format: ScanoutFormat::Bgra8888,
            }
        }
        fn present(&mut self, rows: &[u8]) -> Result<()> {
            // The first swap's pixel, then the second's.
            let frame = *self.0.borrow();
            assert_eq!(rows, [[1, 2, 3, 255], [4, 5, 6, 255]][frame]);
            *self.0.borrow_mut() += 1;
            Ok(())
        }
    }
    struct CountMemory {
        ram: Ram,
        reads: usize,
        writes: usize,
    }
    impl Memory for CountMemory {
        fn read(&mut self, address: u64, bytes: &mut [u8]) -> bool {
            self.reads += 1;
            self.ram.read(address, bytes)
        }
        fn write(&mut self, address: u64, bytes: &[u8]) -> bool {
            self.writes += 1;
            self.ram.write(address, bytes)
        }
    }
    const WRAPPER: u64 = 0x2_71c0_0000;
    const DART: u64 = 0x2_7130_c000;
    const RAM: u64 = 0x8_0000_0000;
    const REGION: u64 = RAM + 0x4000;
    const PIXELS: u64 = RAM + 0xd0000;
    const SUPPLY: u64 = 0x2_3b70_0380;
    fn send(d: &mut AppleDevices, endpoint: u64, payload: u64) {
        d.write(0, WRAPPER + 0x8800, 8, payload, 0).unwrap();
        d.write(0, WRAPPER + 0x8808, 8, endpoint, 0).unwrap();
    }
    fn call(
        d: &mut AppleDevices,
        memory: &mut CountMemory,
        tag: &[u8; 4],
        input: &[u8],
        output: u32,
        now: u64,
    ) -> Vec<(u64, u64)> {
        let stream = link::Stream {
            owner: link::Owner::Ap,
            index: 1,
        };
        let header = link::Header {
            tag: *tag,
            input: input.len() as u32,
            output,
        };
        let at = REGION + stream.calls();
        assert!(memory.ram.write(at, &header.bytes()));
        assert!(memory.ram.write(at + 12, input));
        send(
            d,
            0x37,
            link::Word::Rpc(link::Rpc {
                answer: false,
                stream,
                opens: true,
                offset: 0,
                length: header.length().unwrap(),
                token: 0,
            })
            .encode(),
        );
        d.transfer(0, memory, now).unwrap();
        let reply = mailbox_drain(d, WRAPPER);
        assert!(!reply.is_empty());
        assert!(reply.iter().all(|&(endpoint, _)| endpoint == 0x37));
        reply
    }
    // 60 Hz on the 24 MHz counter.
    const PERIOD: u64 = 400_000;
    let mut d = AppleDevices::new();
    d.add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    let dart = d
        .add_dart(
            Range {
                physical: DART,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let frames = Rc::new(RefCell::new(0));
    d.add_dcpext(
        DcpextNub {
            wrapper: Range {
                physical: WRAPPER,
                bytes: 0x6c000,
            },
            interrupts: [0x243, 0x242, 0x245, 0x244],
            quiesced: false,
            role: "DCPEXT",
            identity: stela_soc_data::dcp::ExternalIdentity::Unnumbered,
        },
        Box::new(Screen(frames.clone())),
        DcpextDma {
            link: (dart, 5),
            window: Some(RAM),
            surface: Some((dart, 5)),
        },
        // The build whose DCP tags this test sends.
        stela_formats::builds::find("26A434").unwrap(),
    )
    .unwrap();
    connect_native_power(&mut d, SUPPLY, &[("/arm-io/dcpext", WRAPPER)], true);
    d.write(0, DART + 0x1014, 4, 6, 0).unwrap();
    d.write(0, DART + 0xc00, 4, 1 << 5, 0).unwrap();
    assert_eq!(mailbox_drain(&mut d, WRAPPER).len(), 1);
    for payload in [
        2 << 52 | 12 | 12 << 16,
        8 << 52 | 1,
        8 << 52 | 1 << 51 | 1 << 32,
        5 << 52 | 0x37 << 32 | 2,
    ] {
        send(&mut d, 0, payload);
        mailbox_drain(&mut d, WRAPPER);
    }
    let mut memory = CountMemory {
        ram: Ram::new(RAM, 0x100000),
        reads: 0,
        writes: 0,
    };
    assert!(memory.ram.write(REGION + 12, &4u32.to_le_bytes()));
    send(&mut d, 0x37, link::Word::Region { dva: 0x4000 }.encode());
    d.transfer(0, &mut memory, 0).unwrap();
    assert_eq!(mailbox_drain(&mut d, WRAPPER).len(), 1);
    // The display's first look, at 0, starts its vblank grid.
    call(&mut d, &mut memory, b"A406", &[0; 0x10], 8, 0);
    assert!(memory.ram.write(PIXELS, &[1, 2, 3, 255]));
    call(
        &mut d,
        &mut memory,
        b"A407",
        &swap::tests::submission(1, &[(0, 1, 1, PIXELS)]),
        0xc,
        1,
    );
    // The swap waits for the next vblank: until it, a write that reaches
    // none of the DCP's devices leaves the DCP's step out.
    assert!(!d.dcp_due(0, PERIOD - 1));
    assert!(d.dcp_due(0, PERIOD));
    for gate in [4, 0x40f, 0] {
        d.write(0, SUPPLY, 4, gate, 0).unwrap();
        let before = (memory.reads, memory.writes);
        d.transfer(0, &mut memory, PERIOD).unwrap();
        assert_eq!((memory.reads, memory.writes), before);
        assert_eq!(*frames.borrow(), 0);
        assert!(
            d.bus
                .device::<Asc>(d.dcp_masters[0].asc)
                .unwrap()
                .firmware::<dcp::Coprocessor>()
                .unwrap()
                .frame_pending()
        );
        d.write(0, SUPPLY, 4, 15, 0).unwrap();
    }
    // The swap waits for a vblank: none is passed between them.
    d.transfer(0, &mut memory, PERIOD - 1).unwrap();
    assert_eq!(*frames.borrow(), 0);
    d.transfer(0, &mut memory, 2 * PERIOD).unwrap();
    assert_eq!(*frames.borrow(), 1);
    assert!(
        !d.bus
            .device::<Asc>(d.dcp_masters[0].asc)
            .unwrap()
            .firmware::<dcp::Coprocessor>()
            .unwrap()
            .frame_pending()
    );
    d.transfer(0, &mut memory, 2 * PERIOD).unwrap();
    assert_eq!(*frames.borrow(), 1);
    mailbox_drain(&mut d, WRAPPER);
    // The AP answers the completion; a swap submitted in the step of a
    // vblank is latched at it, in that step: its completion's call leaves
    // with the submit's reply.
    call(&mut d, &mut memory, b"A406", &[0; 0x10], 8, 2 * PERIOD + 1);
    // Other pixels: a swap whose rows are the last frame's presents nothing.
    assert!(memory.ram.write(PIXELS, &[4, 5, 6, 255]));
    call(
        &mut d,
        &mut memory,
        b"A407",
        &swap::tests::submission(2, &[(0, 1, 1, PIXELS)]),
        0xc,
        3 * PERIOD,
    );
    assert_eq!(*frames.borrow(), 2);
}

#[test]
fn dcpext_enumerates_its_rtkit_and_the_display_regions_refuse() {
    struct Screen;
    impl Scanout for Screen {
        fn mode(&self) -> ScanoutMode {
            ScanoutMode {
                width: 1920,
                height: 1080,
                format: ScanoutFormat::Bgra8888,
            }
        }
        fn present(&mut self, _rows: &[u8]) -> Result<()> {
            Ok(())
        }
    }
    const WRAPPER: u64 = 0x2_71c0_0000;
    const DISPEXT: u64 = 0x2_7000_0000;
    let mut devices = AppleDevices::new();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 0))
        .unwrap();
    let dart = devices
        .add_dart(
            Range {
                physical: 0x2_7130_c000,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    devices
        .add_dcpext(
            DcpextNub {
                wrapper: Range {
                    physical: WRAPPER,
                    bytes: 0x6c000,
                },
                interrupts: [0x242, 0x242, 0x245, 0x244],
                quiesced: true,
                role: "DCPEXT",
                identity: stela_soc_data::dcp::ExternalIdentity::Unnumbered,
            },
            Box::new(Screen),
            DcpextDma {
                link: (dart, 5),
                window: Some(0x8_0000_0000),
                surface: None,
            },
            stela_formats::builds::DEFAULT,
        )
        .unwrap();
    devices
        .add_display(&[(
            "dispext0",
            Range {
                physical: DISPEXT,
                bytes: 0x61c000,
            },
        )])
        .unwrap();
    // The ASC wrapper runs (RUN bit set by the boot-chain handoff).
    assert_eq!(devices.read(0, WRAPPER + 0x44, 4, 0), Ok(1 << 4));
    assert_eq!(devices.coprocessor_diagnostics().len(), 1);
    // A display register has no modeled meaning and refuses.
    assert!(devices.read(0, DISPEXT, 4, 0).is_err());
}

#[test]
fn energy_banks_route_only_their_described_words() {
    let mut devices = AppleDevices::new();
    let cpm = 0x2_11e4_8000;
    let core = 0x2_1005_9000;
    devices.add_cpu_energy(cpm, true).unwrap();
    devices.add_cpu_energy(core, false).unwrap();
    assert_eq!(devices.read(0, cpm, 8, 0), Ok(0));
    devices
        .write(0, cpm + 8, 8, 0x1234_5678_ffff_0000, 0)
        .unwrap();
    assert_eq!(devices.read(0, cpm + 8, 8, 99), Ok(0x1234_5678_ffff_0000));
    assert_eq!(devices.read(0, core + 8, 8, 99), Ok(0));
    assert_eq!(devices.read(0, cpm + energy::CONTROL_OFFSET, 8, 0), Ok(0));
    devices
        .write(0, cpm + energy::CONTROL_OFFSET, 8, 0xa800_0000_0000_0000, 0)
        .unwrap();
    assert_eq!(
        devices.read(0, cpm + energy::CONTROL_OFFSET, 8, 0),
        Ok(0xa800_0000_0000_0000)
    );
    assert!(devices.read(0, cpm + 16, 8, 0).is_err());
    assert!(
        devices
            .read(0, core + energy::CONTROL_OFFSET, 8, 0)
            .is_err()
    );
}

/// A PE's sampler counters are one value through its own MRS and MSR
/// and through any PE's access to its `cpu-impl-reg` window.
#[test]
fn a_pes_sampler_counters_share_its_registers_and_window() {
    let mut devices = AppleDevices::new();
    let layout = stela_soc_data::core_counters::layout(0x8112).unwrap();
    let blocks = [0x2_1005_0000, 0x2_1015_0000];
    for (cpu, physical) in blocks.into_iter().enumerate() {
        devices.add_core_counters(cpu, physical, layout).unwrap();
    }
    let aon_2 = Encoding::new(3, 7, 15, 4, 3);
    let stall_1 = Encoding::new(3, 1, 15, 9, 3);
    assert_eq!(devices.system_register(1, aon_2, Some(77), 0), Some(Ok(0)));
    // PE 0 samples PE 1's counter remotely; its own is untouched.
    assert_eq!(devices.read(0, blocks[1] + 0x4020, 8, 0), Ok(77));
    assert_eq!(devices.read(0, blocks[0] + 0x4020, 8, 0), Ok(0));
    devices.write(0, blocks[1] + 0x6038, 8, 1 << 47, 0).unwrap();
    assert_eq!(
        devices.system_register(1, stall_1, None, 0),
        Some(Ok(1 << 47))
    );
    assert_eq!(devices.system_register(0, stall_1, None, 0), Some(Ok(0)));
    // A register the T8112 lacks is not the board's.
    let aon_3 = Encoding::new(3, 7, 15, 6, 3);
    assert_eq!(devices.system_register(1, aon_3, None, 0), None);
    assert!(devices.read(0, blocks[1] + 0x4030, 8, 0).is_err());
}

#[test]
fn performance_banks_map_sparse_words_without_claiming_holes() {
    let mut devices = AppleDevices::new();
    let base = 0x2_3d2b_4000;
    devices.add_performance_counters(base, &[2, 7]).unwrap();
    assert_eq!(devices.read(0, base, 4, 0), Ok(0));
    devices.write(0, base, 4, 1, 0).unwrap();
    devices.write(0, base + 0x120, 4, 3, 0).unwrap();
    devices.write(0, base + 0x128, 4, 0x1234, 0).unwrap();
    devices.write(0, base + 0x12c, 4, 0x5678, 0).unwrap();
    assert_eq!(devices.read(0, base + 0x120, 4, 0), Ok(3));
    assert_eq!(devices.read(0, base + 0x128, 4, 0), Ok(0x1234));
    assert_eq!(devices.read(0, base + 0x12c, 4, 0), Ok(0x5678));
    assert!(devices.read(0, base + 0x124, 4, 0).is_err());
    assert!(devices.read(0, base + 0x130, 4, 0).is_err());
    assert!(devices.read(0, base + 0x128, 8, 0).is_err());
}

#[test]
fn aop_masks_route_at_the_bank_plus_eight() {
    let mut devices = AppleDevices::new();
    let base = 0x2_3d00_4008;
    let mut valid = [u32::MAX; wake::WORDS];
    valid[7] = 0x7fff_ffff;
    devices.add_wake_masks(base, valid, None).unwrap();
    for (word, &mask) in valid.iter().enumerate() {
        let physical = base + word as u64 * 4;
        assert_eq!(devices.read(0, physical, 4, 0), Ok(u64::from(mask)));
        devices.write(0, physical, 4, u64::from(mask), 0).unwrap();
        assert_eq!(devices.read(0, physical, 4, 0), Ok(u64::from(mask)));
    }
    assert!(devices.read(0, base - 4, 4, 0).is_err());
    assert!(devices.read(0, base + wake::BYTES, 4, 0).is_err());
}

#[derive(Clone, Default)]
struct Console(Rc<RefCell<Vec<u8>>>);
impl ConsoleSink for Console {
    fn transmit(&mut self, byte: u8) -> Result<(), ConsoleBusy> {
        self.0.borrow_mut().push(byte);
        Ok(())
    }
}

const UART: u64 = 0x2_3520_0000;

/// XNU's fast-IPI send and acknowledge between T8112 PEs, through the
/// board's System registers: IPI_RR_GLOBAL_EL1 to cluster 1 CPU 0, then
/// IPI_SR_EL1 on that PE; other encodings are not the board's.
#[test]
fn fast_ipis_raise_and_clear_the_target_pes_fiq() {
    let request = Encoding::new(3, 5, 15, 0, 1);
    let status = Encoding::new(3, 5, 15, 1, 1);
    let mut devices = AppleDevices::new();
    devices.add_fast_ipi(&[0x8000_0000, 0x8000_0001, 0x8000_0100, 0x8000_0101]);
    assert_eq!(
        devices.system_register(0, request, Some(1 << 16), 0),
        Some(Ok(0))
    );
    assert!(devices.own_fiq(2) && !devices.own_fiq(0));
    assert!(
        !devices.lines(2, 0).fiq,
        "the lines leave IPI_SR to own_fiq"
    );
    assert_eq!(devices.system_register(2, status, None, 0), Some(Ok(1)));
    let local = devices.local_registers(2).unwrap();
    assert_eq!(local.register(status, None), Some(1));
    assert!(local.fiq());
    let word = local.storage(status).unwrap() as *const u64;
    // SAFETY: the board keeps the word for its life.
    assert_eq!(unsafe { word.read_volatile() }, 1);
    assert_eq!(devices.system_register(2, status, Some(1), 0), Some(Ok(0)));
    assert!(!devices.own_fiq(2) && !local.fiq());
    assert_eq!(local.register(status, None), Some(0));
    // The same send and acknowledgement without the devices' lock: the
    // sender's handle names the PE to wake, the target's clears its word.
    let sender = devices.local_registers(0).unwrap();
    assert_eq!(sender.send(request, 1 << 16), Some(Some(2)));
    assert_eq!(sender.send(request, 1 << 16), Some(None), "already pending");
    assert!(devices.own_fiq(2) && local.fiq());
    assert_eq!(devices.latched_pins(2).map(|pins| pins.fiq), Some(false));
    assert_eq!(local.register(status, Some(1)), Some(0));
    assert!(!devices.own_fiq(2));
    assert!(devices.take_signalled(2), "the locked send above");
    assert!(!devices.take_signalled(2) && !devices.take_signalled(0));
    assert_eq!(sender.send(status, 1), None, "IPI_SR is the PE's own");
    assert_eq!(
        devices.system_register(0, Encoding::new(3, 5, 15, 2, 0), None, 0),
        None
    );
    assert!(matches!(
        devices.system_register(0, request, Some(1 << 16 | 3 << 28), 0),
        Some(Err(_))
    ));
}

/// An IPI_SR write completes on the PE's own word: one without PENDING
/// changes nothing, and one with PENDING clears it and with it the PE's
/// FIQ. A field the model refuses reaches the board.
#[test]
fn ipi_status_writes_complete_on_the_pes_word() {
    const START: u64 = 0x2_3b73_4000;
    let request = Encoding::new(3, 5, 15, 0, 0);
    let status = Encoding::new(3, 5, 15, 1, 1);
    let mut devices = AppleDevices::new();
    devices.add_fast_ipi(&[0x8000_0000, 0x8000_0001]);
    let local = devices.local_registers(1).unwrap();
    assert_eq!(local.register(status, Some(1)), Some(0), "nothing pending");
    assert_eq!(local.register(status, Some(0)), Some(0));
    assert_eq!(local.register(status, Some(2)), None, "the model refuses");
    assert_eq!(devices.system_register(0, request, Some(1), 0), Some(Ok(0)));
    assert!(devices.own_fiq(1));
    assert_eq!(local.register(status, Some(0)), Some(0));
    assert!(devices.own_fiq(1));
    assert_eq!(local.register(status, Some(1)), Some(0), "clears PENDING");
    assert!(!devices.own_fiq(1));
    // Reports name the model at an address.
    devices.add_cpu_start(START, 1, &[0x8000_0000]).unwrap();
    assert_eq!(devices.device_name(START + 4), Some("cpu-start"));
    assert_eq!(devices.device_name(0x10), None);
}

#[test]
fn a_start_bit_releases_its_pe_at_the_locked_rvbar() {
    const START: u64 = 0x2_3b73_4000;
    const ENTRY: u64 = 0x8_0300_c388;
    let mpidrs = [0x8000_0000, 0x8000_0001, 0x8000_0100, 0x8000_0101];
    let mut devices = AppleDevices::new();
    for pe in 0..4 {
        let range = Range {
            physical: 0x2_1005_0000 + 0x10_0000 * pe as u64,
            bytes: 0x9010,
        };
        devices
            .add_lockdown("cpu-impl-reg", 0x5000, range, Some((pe, ENTRY)))
            .unwrap();
    }
    devices.add_cpu_start(START, 2, &mpidrs).unwrap();
    // The kernel's sequence for cluster 1 core 1: its startup bit, then
    // both clusters' start words.
    devices.write(0, START + 4, 4, 1 << 5, 0).unwrap();
    assert_eq!(devices.take_start(), None);
    devices.write(0, START + 8, 4, 0, 0).unwrap();
    devices.write(0, START + 0xc, 4, 1 << 1, 0).unwrap();
    assert_eq!(
        devices.take_start(),
        Some(Start {
            cpu: 3,
            address: 0x8_0300_c000
        })
    );
    assert_eq!(devices.take_start(), None);
    // RVBAR_EL2 reads the same reset address.
    assert_eq!(devices.reset_vector(3), Some(0x8_0300_c000));
    // A core the board does not have refuses.
    assert!(devices.write(0, START + 0xc, 4, 1 << 2, 0).is_err());
}

#[test]
fn processor_startup_words_bind_once_without_legacy_lockdown() {
    const BASE: u64 = 0x2_1005_0000;
    const ENTRY: u64 = 0x100_0300_c000;
    let mut devices = AppleDevices::new();
    let range = Range {
        physical: BASE,
        bytes: 0x9008,
    };
    devices.add_processor_registers(0, range, ENTRY).unwrap();
    assert_eq!(devices.read(0, BASE, 8, 0), Ok(ENTRY | 1));
    assert_eq!(devices.read(0, BASE + lockdown::CPU_STS, 8, 0), Ok(1));
    devices.set_processor_running(0, false);
    assert_eq!(devices.read(0, BASE + lockdown::CPU_STS, 8, 0), Ok(0));
    assert_eq!(devices.locked_reset_vector(0), Some(ENTRY));
    devices
        .write(0, BASE + lockdown::CORE_THRTL_CFG2, 8, 0xb2 << 56, 0)
        .unwrap();
    assert_eq!(
        devices.read(0, BASE + lockdown::CORE_THRTL_CFG2, 8, 0),
        Ok(0xb2 << 56)
    );
    for offset in [0x5000, 0x5008, 0x9000, 0x9008] {
        assert!(devices.bus.route(BASE + offset, Width::Double).is_err());
    }
    assert!(matches!(
        devices.add_processor_registers(0, range, ENTRY),
        Err(MapError::Overlap { .. })
    ));
}

#[test]
fn cpu_status_initially_runs_only_the_selected_handoff_pe() {
    const BASE: u64 = 0x2_1005_0000;
    const ENTRY: u64 = 0x100_0300_c000;
    for legacy in [false, true] {
        let mut devices = AppleDevices::for_boot_processor(2);
        for pe in 0..4 {
            let range = Range {
                physical: BASE + pe as u64 * 0x10_0000,
                bytes: 0x9010,
            };
            if legacy {
                devices
                    .add_lockdown("cpu-impl-reg", 0x5000, range, Some((pe, ENTRY)))
                    .unwrap();
            } else {
                devices.add_processor_registers(pe, range, ENTRY).unwrap();
            }
            assert_eq!(
                devices.read(2, range.physical + lockdown::CPU_STS, 8, 0),
                Ok(u64::from(pe == 2))
            );
            assert_eq!(devices.locked_reset_vector(pe), Some(ENTRY));
        }
        devices.processor_power(2, false);
        assert_eq!(
            devices.read(2, BASE + 0x20_0000 + lockdown::CPU_STS, 8, 0),
            Ok(0)
        );
    }
}

#[test]
fn aic_default_delivery_uses_the_selected_boot_pe_and_keeps_every_pe_served() {
    const AIC: u64 = 0x2_3b0c_0000;
    for (layout, irq_config, event) in [
        (AicLayout::aic2(0x8000, 0), 0x2000, 0x8000),
        (AicLayout::aic3(0x40000, 0), 0x10000, 0x40000),
    ] {
        let mut devices = AppleDevices::for_boot_processor(2);
        devices
            .add_aic_for_processors(AIC, 1152, layout, 4)
            .unwrap();
        let software = AIC + irq_config + 4 * u64::from(stela_soc_data::aic::AIC2_MAX_IRQ);
        devices.write(2, AIC + 0x14, 4, 1, 0).unwrap();
        devices.write(2, software, 4, 1 << 7, 0).unwrap();
        // MASK_CLR follows SW_SET, SW_CLR and MASK_SET, each MAX_IRQ/8 bytes.
        devices.write(2, software + 3 * 512, 4, 1 << 7, 0).unwrap();
        for pe in [0, 1, 3] {
            assert!(!devices.lines(pe, 0).irq);
            assert_eq!(devices.read(pe, AIC + event, 4, 0), Ok(0));
        }
        assert!(devices.lines(2, 0).irq);
        assert_eq!(devices.read(2, AIC + event, 4, 0), Ok(1 << 16 | 7));
        assert!(!devices.lines(2, 0).irq);
    }
    let mut devices = AppleDevices::for_boot_processor(4);
    for cpus in [0, 4, 65] {
        assert!(
            devices
                .add_aic_for_processors(AIC, 1152, AicLayout::aic2(0x8000, 0), cpus)
                .is_err()
        );
        assert!(devices.aic.is_none());
        assert!(devices.bus.route(AIC, Width::Word).is_err());
    }
    devices
        .add_aic_for_processors(AIC, 1152, AicLayout::aic2(0x8000, 0), 5)
        .unwrap();
}

#[test]
fn processor_startup_word_installation_is_atomic_for_every_conflict() {
    const BASE: u64 = 0x2_1005_0000;
    let range = Range {
        physical: BASE,
        bytes: 0x9008,
    };
    for conflict in lockdown::PE_WORDS {
        let mut devices = AppleDevices::new();
        let foreign = devices.bus.add(Box::new(CheckedPowerBank {
            reject: true,
            committed: Vec::new(),
        }));
        devices.bus.map(foreign, BASE + conflict, 8, 0).unwrap();
        assert!(
            devices
                .add_processor_registers(2, range, 0x100_0300_c000)
                .is_err()
        );
        assert!(devices.reset_vectors.is_empty());
        assert_eq!(
            devices.bus.route(BASE + conflict, Width::Double),
            Ok((foreign, 0))
        );
        for offset in lockdown::PE_WORDS
            .into_iter()
            .filter(|offset| *offset != conflict)
        {
            assert!(devices.bus.route(BASE + offset, Width::Double).is_err());
        }
    }
    let mut devices = AppleDevices::new();
    for bad in [
        Range {
            physical: BASE + 1,
            bytes: 0x9008,
        },
        Range {
            physical: BASE,
            bytes: lockdown::CORE_THRTL_CFG2 + 4,
        },
        Range {
            physical: u64::MAX - 7,
            bytes: 0x9008,
        },
    ] {
        assert!(
            devices
                .add_processor_registers(0, bad, 0x100_0300_c000)
                .is_err()
        );
        assert!(devices.reset_vectors.is_empty());
        assert!(devices.bus.route(BASE, Width::Double).is_err());
    }
}

#[test]
fn lockdown_maps_only_its_registers_beside_the_energy_accumulators() {
    // J473AP cpu0's `cpu-impl-reg` and cluster 0's `cpm-impl-reg`, with
    // their accumulators at +0x9000 and +0x8000.
    let (cpu, cpm) = (0x2_1005_0000, 0x2_10e4_0000);
    let mut devices = AppleDevices::new();
    for (name, physical, control, reset) in [
        ("cpu-impl-reg", cpu, 0x5000, Some((0, 0x8_0300_c388))),
        ("cpm-impl-reg", cpm, 0x9000, None),
    ] {
        let range = Range {
            physical,
            bytes: 0x9010,
        };
        devices.add_lockdown(name, control, range, reset).unwrap();
    }
    devices
        .add_cpu_energy(cpu + stela_soc_data::energy::CORE_OFFSET, false)
        .unwrap();
    devices.add_cpu_energy(cpm + 0x8000, true).unwrap();
    assert_eq!(devices.read(0, cpu, 8, 0), Ok(0x8_0300_c001));
    assert_eq!(devices.read(0, cpu + 0x5008, 8, 0), Ok(0));
    assert_eq!(devices.read(0, cpm + 0x9000, 8, 0), Ok(0));
    assert_eq!(devices.read(0, cpu + 0x9008, 8, 0), Ok(0));
    // The boot PE runs: XNU's cpu_exit_wait polls CPU_STS until 0, which
    // the PE's power-down gives it.
    assert_ne!(devices.read(0, cpu + lockdown::CPU_STS, 8, 0), Ok(0));
    devices.processor_power(0, false);
    assert_eq!(devices.read(0, cpu + lockdown::CPU_STS, 8, 0), Ok(0));
    assert_eq!(
        devices.read(0, cpu + lockdown::CORE_THRTL_CFG2, 8, 0),
        Ok(0)
    );
    // The rest of each block reaches no model.
    assert!(devices.read(0, cpu + 0x108, 8, 0).is_err());
    assert!(devices.read(0, cpm, 8, 0).is_err());
    assert!(devices.read(0, cpm + lockdown::CPU_STS, 8, 0).is_err());
}

#[test]
fn routes_the_console_and_refuses_other_registers() {
    let console = Console::default();
    let mut devices = AppleDevices::new();
    devices
        .add_uart(
            console.clone(),
            &[Range {
                physical: UART,
                bytes: 0x4000,
            }],
            None,
        )
        .unwrap();
    // UTRSTAT reports an empty transmitter; UTXH sends one byte.
    assert_eq!(devices.read(0, UART + 0x10, 4, 0).unwrap() & 6, 6);
    for byte in *b"ok" {
        devices
            .write(0, UART + 0x20, 4, u64::from(byte), 0)
            .unwrap();
    }
    assert_eq!(console.0.borrow().as_slice(), b"ok");
    assert_eq!(
        devices.read(0, 0x2_0000_0000, 4, 0),
        Err(DeviceError::NoDevice)
    );
    assert!(matches!(
        devices.read(0, UART + 0x3c, 4, 0),
        Err(DeviceError::Refused(reason)) if reason.to_string().contains("UVER")
    ));
    assert!(!devices.lines(0, 0).irq);
    assert_eq!(devices.take_reset(), None);
}

#[test]
fn places_an_aic2_with_derived_counts_and_routes_uart_wires_to_it() {
    let mut devices = AppleDevices::new();
    devices
        .add_uart(
            Console::default(),
            &[Range {
                physical: UART,
                bytes: 0x4000,
            }],
            Some(AicInput::new(0, 739)),
        )
        .unwrap();
    devices
        .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x8000, 16))
        .unwrap();
    // CAP0 (offset 4) reports NR_IRQ in bits 15:0.
    assert_eq!(devices.read(0, 0x2_3b0c_0004, 4, 0).unwrap() & 0xffff, 1152);
    assert!(
        devices
            .add_aic(0x2_3b0c_0000, 1152, AicLayout::aic2(0x10, 16))
            .is_err()
    );
}

/// J274AP (M1): `aic,1` at 0x2_3b10_0000 with 896 inputs, the UART on 605.
#[test]
fn places_an_aic1_and_routes_uart_wires_to_it() {
    const AIC: u64 = 0x2_3b10_0000;
    let mut devices = AppleDevices::new();
    devices
        .add_uart(
            Console::default(),
            &[Range {
                physical: UART,
                bytes: 0x4000,
            }],
            Some(AicInput::new(0, 605)),
        )
        .unwrap();
    devices.add_aic(AIC, 896, AicLayout::V1).unwrap();
    // INFO (offset 4) reports NR_IRQ in bits 15:0, and WHOAMI cpu0.
    assert_eq!(devices.read(0, AIC + 0x4, 4, 0).unwrap() & 0xffff, 896);
    assert_eq!(devices.read(0, AIC + 0x2000, 4, 0).unwrap(), 0);
    // Target the UART line at cpu0 and unmask it.
    devices.write(0, AIC + 0x3000 + 4 * 605, 4, 1, 0).unwrap();
    devices
        .write(0, AIC + 0x4180 + 4 * (605 / 32), 4, 1 << (605 % 32), 0)
        .unwrap();
    // XNU `apple_uart_enable_irq`: with the TX trigger at 0 and UCON's
    // TX threshold interrupt enabled, sending a byte raises input 605.
    let ufcon = devices.read(0, UART + 0x8, 4, 0).unwrap();
    devices
        .write(0, UART + 0x8, 4, ufcon & !(0b11 << 6), 0)
        .unwrap();
    let ucon = devices.read(0, UART + 0x4, 4, 0).unwrap();
    devices.write(0, UART + 0x4, 4, ucon | 1 << 13, 0).unwrap();
    assert!(!devices.lines(0, 0).irq);
    devices
        .write(0, UART + 0x20, 4, u64::from(b'x'), 0)
        .unwrap();
    assert!(devices.lines(0, 0).irq);
    assert_eq!(
        devices.read(0, AIC + AIC1_EVENT, 4, 0).unwrap(),
        1 << 16 | 605
    );
    assert!(!devices.lines(0, 0).irq, "the event read masked the IRQ");
    assert!(matches!(
        devices.read(0, AIC + 0x5008, 4, 0),
        Err(DeviceError::Refused(reason)) if reason.to_string().contains("per-CPU")
    ));
}

/// The AES engine's DMA runs in the write that starts it, through its
/// DART stream: here a bypassed one, ECB with the FIPS-197 C.1 key.
#[test]
fn aes_writes_move_data_through_the_dart_stream() {
    use stela_contracts::dma::test_support::Ram;
    const DART: u64 = 0x2_3500_4000;
    const AES: u64 = 0x2_3500_c000;
    const RAM: u64 = 0x8_0000_0000;
    let mut devices = AppleDevices::new();
    let dart = devices
        .add_dart(
            Range {
                physical: DART,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let ranges = [
        Range {
            physical: AES,
            bytes: 0x4000,
        },
        Range {
            physical: 0x2_3d2d_c000,
            bytes: 0x8000,
        },
    ];
    devices
        .add_aes(&ranges, j473_aes(), None, (dart, 1), None)
        .unwrap();
    let mut ram = Ram::new(RAM, 0x1000);
    let plain: Vec<u8> = (0..16).map(|byte| byte * 0x11).collect();
    assert!(Memory::write(&mut ram, RAM, &plain));
    // Stream 1 enabled and bypassed: TCR BYPASS_DART | BYPASS_DAPF.
    for (offset, value) in [(0xc00, 0b10), (0x1004, 0b110)] {
        devices.write(0, DART + offset, 4, value, 0).unwrap();
    }
    let key: Vec<u64> = (0..4)
        .map(|word| u64::from(0x0302_0100_u32 + 0x0404_0404 * word))
        .collect();
    let command = [0x1010_0000]
        .into_iter()
        .chain(key)
        .chain([0x5000_0010, 0x0008_0008, 0, 0x100]);
    devices.write(0, AES + 0x8, 4, 1, 0).unwrap();
    for word in command {
        devices.write(0, AES + 0x200, 4, word, 0).unwrap();
        devices.transfer(0, &mut ram, 0).unwrap();
    }
    let mut out = [0; 16];
    assert!(Memory::read(&mut ram, RAM + 0x100, &mut out));
    assert_eq!(
        out,
        [
            0x69, 0xc4, 0xe0, 0xd8, 0x6a, 0x7b, 0x04, 0x30, 0xd8, 0xcd, 0xb7, 0x80, 0x70, 0xb4,
            0xc5, 0x5a
        ]
    );
    // The FIFO drained: EMPTY with LEVEL 0.
    assert_eq!(devices.read(0, AES + 0x24, 4, 0).unwrap() & 0xff02, 0b10);
}

/// A J773g-shaped version 5 engine owns its DKey range third; its key
/// words follow the ADT offsets.
#[test]
fn version_5_aes_places_its_dkey_range_after_the_key_disable_words() {
    const DART: u64 = 0x1_ad00_4000;
    const AES: u64 = 0x1_ad00_c000;
    const DISABLE: u64 = 0x1_882d_c000;
    const DKEY: u64 = 0x1_8807_8000;
    let mut devices = AppleDevices::new();
    let dart = devices
        .add_dart(
            Range {
                physical: DART,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let config = stela_soc_data::aes::Config {
        version: 5,
        address_width: 42,
        key_disable: stela_soc_data::aes::KeyDisable {
            bytes: 0x8000,
            word: 0,
            keys: [Some(0x40), Some(0x44), Some(0x48)],
        },
    };
    let ranges = [(AES, 0x4000), (DISABLE, 0x8000), (DKEY, 0x4000)]
        .map(|(physical, bytes)| Range { physical, bytes });
    assert!(
        devices
            .add_aes(&ranges[..2], config, None, (dart, 1), None)
            .unwrap_err()
            .to_string()
            .contains("has 3 register ranges, not 2")
    );
    devices
        .add_aes(&ranges, config, None, (dart, 1), None)
        .unwrap();
    assert_eq!(devices.read(0, DISABLE, 4, 0).unwrap(), 0xf);
    assert_eq!(devices.read(0, DISABLE + 0x40, 4, 0).unwrap(), 0);
    assert!(devices.write(0, DKEY + 4, 4, 1, 0).is_err());
}
#[test]
fn sep_native_nonzero_routes_read_firmware_and_preflight_owner_capacity() {
    use crate::sep::message::{Frame, ROM};
    const DART: u64 = 0x25d2c0000;
    const SEP: u64 = 0x25e400000;
    const AUX: u64 = 0x25e050000;
    const RAM: u64 = 0x800000000;
    let ranges = [
        Range {
            physical: SEP,
            bytes: 0x6c000,
        },
        Range {
            physical: AUX,
            bytes: 0x60000,
        },
    ];
    let add = |d: &mut AppleDevices, owner, input| {
        d.add_sep(
            ranges,
            asc::Lines::default(),
            (owner, input),
            Box::new(|b: &mut [u8]| {
                b.fill(0xa5);
                Ok(())
            }),
            SepProvisioning {
                security: None,
                ecid: 0xa5a5_0006,
                kexts: None,
                observe: false,
                state: sep::state::Location::Volatile,
                recovery_policy_nonce: None,
            },
        )
    };
    for (profile, input, context) in [
        (stela_soc_data::dart::Profile::T8110, 4, 4),
        (stela_soc_data::dart::Profile::T6000, 5, 4),
    ] {
        let mut unrelated = AppleDevices::new();
        let absent = unrelated
            .add_dart(
                Range {
                    physical: DART,
                    bytes: 0x4000,
                },
                16,
                None,
                None,
            )
            .unwrap();
        let mut d = AppleDevices::new();
        assert!(
            add(&mut d, absent, input)
                .unwrap_err()
                .to_string()
                .contains("not a mounted DART")
        );
        d.add_aic(0x23b100000, 16, AicLayout::V1).unwrap();
        let (Controller::V1(id) | Controller::V2(id)) = d.aic.unwrap();
        let foreign = DartId(id);
        assert!(
            add(&mut d, foreign, input)
                .unwrap_err()
                .to_string()
                .contains("not a mounted DART")
        );
        let owner = d
            .add_dart_group_for_profile(
                profile,
                &[Range {
                    physical: DART,
                    bytes: 0x4000,
                }],
                16,
                None,
                None,
            )
            .unwrap()[0];
        assert!(
            add(&mut d, owner, 16)
                .unwrap_err()
                .to_string()
                .contains("capacity")
        );
        assert!(
            add(&mut d, owner, usize::MAX)
                .unwrap_err()
                .to_string()
                .contains("capacity")
        );
        assert!(d.sep_masters.is_empty());
        assert!(d.read(0, SEP + 0x8114, 4, 0).is_err());
        add(&mut d, owner, input).unwrap();
        assert_eq!(d.sep_masters[0].stream, input);
        d.write(0, SEP + 0x8114, 4, 1, 0).unwrap();
        let regs = if profile == stela_soc_data::dart::Profile::T8110 {
            [
                (0xc00, 1u64 << context),
                (0x1000 + 4 * context, 6),
                (0x80, 0),
            ]
        } else {
            // Source5 -> context4, all other sources identity. Only
            // context4 is enabled/bypassed; keep input5 independently.
            [
                (0xfc, 1u64 << context),
                (0x100 + 4 * context, 1 << 8),
                (0x84, 0x07060404),
            ]
        };
        for (offset, value) in regs {
            if profile == stela_soc_data::dart::Profile::T6000 || offset != 0x80 {
                d.write(0, DART + offset, 4, value, 0).unwrap();
            }
        }
        let send = |d: &mut AppleDevices, ram: &mut Ram, opcode, data| {
            let frame = Frame {
                service: ROM,
                tag: 1,
                opcode,
                parameter: 0,
                data,
            };
            d.write(0, SEP + 0x8800, 8, frame.mailbox().payload, 0)
                .unwrap();
            d.write(0, SEP + 0x8808, 8, 0, 0).unwrap();
            d.transfer(0, ram, 0).unwrap();
            let reply = Frame::from_payload(d.read(0, SEP + 0x8830, 8, 0).unwrap());
            d.read(0, SEP + 0x8838, 8, 0).unwrap();
            reply
        };
        let mut ram = Ram::new(RAM, 0x8000);
        let image = stela_formats::img4::test_support::im4p(b"sepi", b"stela-sepos");
        ram.bytes[..image.len()].copy_from_slice(&image);
        assert_eq!(send(&mut d, &mut ram, 5, 0).opcode, 105);
        assert_eq!(send(&mut d, &mut ram, 6, (RAM >> 12) as u32).opcode, 106);
    }
}

#[test]
fn sep_rom_accepts_firmware_under_stela_policy_and_reaches_os_active() {
    use {
        crate::sep::message, crate::sep::message::Frame, crate::sep::message::ROM,
        stela_contracts::dma::test_support::Ram,
    };
    const DART: u64 = 0x25d2c0000;
    const SEP: u64 = 0x25e400000;
    const AUX: u64 = 0x25e050000;
    const RAM: u64 = 0x800000000;

    // Build a SEP mapped on dart-sep stream 0, with its outbox enabled.
    fn board() -> AppleDevices {
        let mut devices = AppleDevices::new();
        let dart = devices
            .add_dart(
                Range {
                    physical: DART,
                    bytes: 0x4000,
                },
                1,
                None,
                None,
            )
            .unwrap();
        devices
            .add_sep(
                [
                    Range {
                        physical: SEP,
                        bytes: 0x6c000,
                    },
                    Range {
                        physical: AUX,
                        bytes: 0x60000,
                    },
                ],
                asc::Lines::default(),
                (dart, 0),
                Box::new(|b: &mut [u8]| {
                    b.fill(0xa5);
                    Ok(())
                }),
                SepProvisioning {
                    security: None,
                    ecid: 0xa5a5_0006,
                    kexts: None,
                    observe: false,
                    state: sep::state::Location::Volatile,
                    recovery_policy_nonce: None,
                },
            )
            .unwrap();
        devices.write(0, SEP + 0x8114, 4, 1, 0).unwrap(); // enable outbox
        devices
    }

    fn send(devices: &mut AppleDevices, ram: &mut Ram, frame: Frame) -> Result<(), DeviceError> {
        devices
            .write(0, SEP + 0x8800, 8, frame.mailbox().payload, 0)
            .unwrap();
        let written = devices.write(0, SEP + 0x8808, 8, 0, 0);
        written.and_then(|()| devices.transfer(0, ram, 0))
    }

    fn recv(devices: &mut AppleDevices) -> Frame {
        let payload = devices.read(0, SEP + 0x8830, 8, 0).unwrap();
        devices.read(0, SEP + 0x8838, 8, 0).unwrap();
        Frame::from_payload(payload)
    }

    // The real IPSW SEP payload is a bare encrypted `sepi`; Stela accepts
    // it under its own policy and boots its native sepOS.
    let image = stela_formats::img4::test_support::im4p(b"sepi", b"stela-sepos");
    let mut devices = board();
    let mut ram = Ram::new(RAM, 0x8000);
    ram.bytes[..image.len()].copy_from_slice(&image);
    // Map the delivered container into the SEP's dart-sep stream.
    devices.write(0, DART + 0xc00, 4, 1, 0).unwrap();
    devices.write(0, DART + 0x1000, 4, 6, 0).unwrap(); // TCR BYPASS_DART|BYPASS_DAPF

    send(
        &mut devices,
        &mut ram,
        Frame {
            service: ROM,
            tag: 1,
            opcode: 5,
            parameter: 0,
            data: 0,
        },
    )
    .unwrap();
    assert_eq!(recv(&mut devices).opcode, 105, "BootTz0 ack");
    send(
        &mut devices,
        &mut ram,
        Frame {
            service: ROM,
            tag: 1,
            opcode: 6,
            parameter: 0,
            data: (RAM >> 12) as u32,
        },
    )
    .unwrap();
    assert_eq!(recv(&mut devices).opcode, 106, "SEP accepted IMG4");
    // The AP maps a real, separate directory through dart-sep. This
    // fixture advertises no channels, so its first entry is the required
    // terminator. OS active follows successful directory DMA.
    ram.bytes[0x4000..0x4004].copy_from_slice(&u32::from_be_bytes(*b"null").to_le_bytes());
    send(
        &mut devices,
        &mut ram,
        Frame {
            service: message::BOOT,
            tag: 0,
            opcode: 4,
            parameter: 0,
            data: ((RAM + 0x4000) >> 12) as u32,
        },
    )
    .unwrap();
    let active = recv(&mut devices);
    assert_eq!((active.service, active.tag, active.opcode), (0, 0, 21));

    // A malformed container refuses with no boot acknowledgement.
    let mut devices = board();
    let mut ram = Ram::new(RAM, 0x8000);
    ram.bytes[..8].copy_from_slice(b"\x30\x06\x16\x04IM4P");
    devices.write(0, DART + 0xc00, 4, 1, 0).unwrap();
    devices.write(0, DART + 0x1000, 4, 6, 0).unwrap(); // TCR BYPASS_DART|BYPASS_DAPF
    send(
        &mut devices,
        &mut ram,
        Frame {
            service: ROM,
            tag: 1,
            opcode: 5,
            parameter: 0,
            data: 0,
        },
    )
    .unwrap();
    recv(&mut devices);
    let refusal = send(
        &mut devices,
        &mut ram,
        Frame {
            service: ROM,
            tag: 1,
            opcode: 6,
            parameter: 0,
            data: (RAM >> 12) as u32,
        },
    );
    assert!(
        matches!(refusal, Err(DeviceError::TransferRefused(reason)) if reason.to_string().contains("payload type is malformed"))
    );
}
fn usb_placement() -> stela_soc_data::board::UsbController {
    const BASE: u64 = 0x382280000;
    stela_soc_data::board::UsbController {
        ranges: [
            (BASE, 0x11800),
            (0x382200000, 0x4000),
            (BASE + 0xc000, 0x5800),
            (0x382a84000, 0x4000),
            (0x382800000, 0x4000),
            (0x382a80000, 0x4000),
            (0x382000000, 0x4000),
            (0x382080000, 0x4000),
            (BASE + 0xd000, 0x4000),
        ],
        irqs: [1031, 1032, 1033, 1034, 990],
        iommu: 0,
        phandle: Some(0xa5),
        mapper: 0xa4,
        mapper_context: 1,
        stream: 1,
        port: 1,
        retention: false,
        phy: 0xa2,
    }
}

fn add_usb_phy(d: &mut AppleDevices) -> stela_soc_data::board::TypecPhy {
    let placement = stela_soc_data::board::TypecPhy {
        phandle: 0xa2,
        port: 1,
        retention: true,
        views: Vec::new(),
        usb2: (0x382a90000, stela_soc_data::atc_phy::WINDOW_BYTES),
        core: (0x383000000, stela_soc_data::atc_phy::CORE_BYTES),
        pipe: Some((0x382a84000, stela_soc_data::atc_phy::WINDOW_BYTES)),
        events: (0x382800000, stela_soc_data::atc_phy::WINDOW_BYTES),
    };
    d.add_atc_phy(&placement).unwrap();
    placement
}

fn release_usb_phy(d: &mut AppleDevices, phy: &stela_soc_data::board::TypecPhy) {
    // USB2 RUN, reset/SIDDQ release, core supply/reset release,
    // and pipehandler controller RESET_N without clamp.
    for (address, value) in [
        (phy.usb2.0, 2),
        (phy.usb2.0 + 4, 4),
        (phy.core.0 + 0x20000, 3 | 8 | 16),
        (phy.core.0 + 0x20008, 1),
        (phy.pipe.unwrap().0 + 0x1c, 1),
    ] {
        d.write(0, address, 4, value, 0).unwrap();
    }
}

#[test]
fn mounted_usb_dma_service_failure_is_protected_from_survey_substitution() {
    let mut d = AppleDevices::new();
    let dart = d
        .add_dart(
            Range {
                physical: 0x382f00000,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let phy = add_usb_phy(&mut d);
    let placement = usb_placement();
    d.add_usb(&placement, dart).unwrap();
    release_usb_phy(&mut d, &phy);
    let base = placement.ranges[0].0;
    let table = 0x800008000u64;
    let events = 0x800009000u64;
    let mut ram = Ram::new(0x800000000, 0x10000);
    ram.put_u64(table, events);
    ram.put_u64(table + 8, 16);
    d.write(0, base + 0x468, 4, 1, 0).unwrap();
    d.write(0, base + 0x470, 4, table as u32 as u64, 0).unwrap();
    d.write(0, base + 0x474, 4, table >> 32, 0).unwrap();
    // The placed stream is still disabled. Loading this real
    // event segment table must fail through DART, never survey-success.
    for now in [0, 10] {
        let error = d.transfer(0, &mut ram, now).unwrap_err();
        assert!(
            matches!(error, DeviceError::TransferRefused(ref reason) if reason.to_string().starts_with("USB device advance:"))
        );
    }
    let mut untouched = [0; 16];
    assert!(ram.read(events, &mut untouched));
    assert_eq!(untouched, [0; 16]);
}

#[test]
fn phy_clock_and_kept_supply_freeze_usb_time_while_quiesce_does_not() {
    use coverage::{Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let mut d = AppleDevices::new();
    let dart = d
        .add_dart(
            Range {
                physical: 0x382f00000,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let phy = add_usb_phy(&mut d);
    let placement = usb_placement();
    d.add_usb(&placement, dart).unwrap();
    release_usb_phy(&mut d, &phy);
    let supply = 0x23d280100;
    let id = DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![Domain {
            name: String::new(),
            id,
            address: Some(supply),
            parents: [None; 4],
        }],
        vec![PowerResource {
            clock_role: stela_soc_data::board::ClockRole::Consumer,
            name: "/arm-io/atc-phy0".into(),
            clocks: vec![id],
            power: vec![],
            owner: Some(PowerOwner::Phy(0)),
            admission: coverage::Admission::Required,
        }],
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &states.registers,
    )
    .unwrap();
    d.connect_power_resources(&states, &[vec![dart]], &[false])
        .unwrap();
    let base = placement.ranges[0].0;
    d.write(0, base + 0x20, 4, 1, 0).unwrap();
    assert_eq!(d.read(0, base + 0x440, 4, 15_000), Ok(5));
    d.write(0, supply, 4, 4, 15_000).unwrap();
    d.write(0, base + 0x20, 4, 1, 800_000).unwrap();
    assert!(d.read(0, phy.usb2.0, 4, 900_000).is_err());
    assert_eq!(d.read(0, base + 0x440, 4, 900_000), Ok(5));
    d.write(0, supply, 4, 15, 900_000).unwrap();
    d.write(0, supply, 4, 0x40f, 900_000).unwrap();
    assert_eq!(d.read(0, base + 0x440, 4, 915_000), Ok(10));
    assert!(
        !d.bus
            .device::<Dwc3>(d.usb_masters[0].device)
            .unwrap()
            .lifecycle_active()
    );
    d.write(0, supply, 4, 15, 915_000).unwrap();
    d.write(0, supply, 4, 0, 915_000).unwrap();
    assert_eq!(d.read(0, base + 0x440, 4, 1_915_000), Ok(10));
    d.write(0, supply, 4, 15, 1_915_000).unwrap();
    assert_eq!(d.read(0, base + 0x440, 4, 1_930_000), Ok(15));
}

#[test]
fn phy_phandles_distinguish_standalone_views_on_the_same_connector() {
    let mut devices = AppleDevices::new();
    let first = add_usb_phy(&mut devices);
    let mut auxiliary = first.clone();
    auxiliary.phandle += 1;
    auxiliary.pipe = None;
    // Original 26A434 J414s/J416s/J474s have four ATC PHYs: PHY3
    // repeats port 1, but no USB DRD names its distinct phandle.
    const OTHER_APERTURE: u64 = 0x1_0000_0000;
    auxiliary.usb2.0 += OTHER_APERTURE;
    auxiliary.core.0 += OTHER_APERTURE;
    auxiliary.events.0 += OTHER_APERTURE;
    devices.add_atc_phy(&auxiliary).unwrap();
    assert_eq!(devices.phys.len(), 2);
    assert!(
        devices
            .add_atc_phy(&auxiliary)
            .unwrap_err()
            .to_string()
            .contains("phandle")
    );
    assert_eq!(devices.phys.len(), 2);
    let dart = devices
        .add_dart(
            Range {
                physical: 0x382f00000,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let mut placement = usb_placement();
    placement.phy = auxiliary.phandle;
    assert!(
        devices
            .add_usb(&placement, dart)
            .unwrap_err()
            .to_string()
            .contains("pipehandler")
    );
    placement.phy = first.phandle;
    devices.add_usb(&placement, dart).unwrap();
    assert_eq!(devices.usb_masters[0].phy, devices.phys[0].device);
}

#[test]
fn usb_requires_phy_binding_and_kept_phy_pauses_preserve_sessions() {
    use coverage::{Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let mut d = AppleDevices::new();
    let base_dart = 0x382f00000;
    let dart = d
        .add_dart(
            Range {
                physical: base_dart,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let placement = usb_placement();
    assert!(
        d.add_usb(&placement, dart)
            .unwrap_err()
            .to_string()
            .contains("no placed PHY")
    );
    let phy = add_usb_phy(&mut d);
    // An unbound PHY with no physical reset aperture is a supported
    // standalone topology, but cannot silently hold a USB core forever.
    let saved_pipe = d.phys[0].pipe.take();
    assert!(
        d.add_usb(&placement, dart)
            .unwrap_err()
            .to_string()
            .contains("pipehandler")
    );
    d.phys[0].pipe = saved_pipe;
    d.add_usb(&placement, dart).unwrap();
    let base = placement.ranges[0].0;
    assert!(d.read(0, base + 0xc704, 4, 0).is_err());
    release_usb_phy(&mut d, &phy);
    let supply = 0x23d280100;
    let id = DomainId { die: 0, device: 1 };
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 4)],
        vec![supply],
        vec![Domain {
            name: String::new(),
            id,
            address: Some(supply),
            parents: [None; 4],
        }],
        vec![PowerResource {
            clock_role: stela_soc_data::board::ClockRole::Consumer,
            name: "/arm-io/atc-phy0".into(),
            clocks: vec![id],
            power: vec![],
            owner: Some(PowerOwner::Phy(0)),
            admission: coverage::Admission::Required,
        }],
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 4,
        }],
        &states.registers,
    )
    .unwrap();
    d.connect_power_resources(&states, &[vec![dart]], &[false])
        .unwrap();
    assert!(d.power_refusals().is_empty());
    d.write(0, base_dart + 0xc00, 4, 2, 0).unwrap();
    d.write(0, base_dart + 0x1004, 4, 6, 0).unwrap();
    let cable = typec::Attached {
        orientation: typec::Orientation::Normal,
        role: typec::DataRole::Ufp,
        source: false,
        vbus: true,
        speed: typec::Usb2Speed::High,
    };
    d.set_connection(1, Some(cable)).unwrap();
    d.write(0, base + 0xc704, 4, 1 << 31, 0).unwrap();
    let mut ram = Ram::new(0x800000000, 0x10000);
    d.transfer(0, &mut ram, 0).unwrap();
    assert_eq!(d.usb_masters[0].epoch, 1);
    for (tick, value) in [(10, 4), (40, 0), (70, 0x40f)] {
        // clock gate, kept off, quiesce
        d.write(0, supply, 4, value, tick).unwrap();
        d.transfer(0, &mut ram, tick + 10).unwrap();
        assert!(d.usb_masters[0].attached);
        assert_eq!(d.usb_masters[0].epoch, 1);
        assert_eq!(d.read(0, base + 0xc704, 4, tick + 10), Ok(1 << 31));
        d.write(0, supply, 4, 15, tick + 20).unwrap();
        d.transfer(0, &mut ram, tick + 20).unwrap();
    }
    // A rapid electrical isolate/release must still cancel the old
    // native session even when final-ready is true at service time.
    d.write(0, phy.usb2.0, 4, 2 | 4, 100).unwrap();
    d.write(0, phy.usb2.0, 4, 2, 100).unwrap();
    d.transfer(0, &mut ram, 100).unwrap();
    assert_eq!(d.usb_masters[0].epoch, 2);
    assert_eq!(d.typec_port(1).snapshot().connection, Some(cable));
    // PHY PMGR reset reaches the pipehandler/controller bank.
    for value in [0x40f, 0x8000040f, 0x40f, 15] {
        d.write(0, supply, 4, value, 110).unwrap();
    }
    assert!(d.read(0, base + 0xc704, 4, 110).is_err());
    d.transfer(0, &mut ram, 110).unwrap();
    assert!(!d.usb_masters[0].attached);
    release_usb_phy(&mut d, &phy);
    assert_eq!(d.read(0, base + 0xc704, 4, 110), Ok(0));
    d.write(0, base + 0xc704, 4, 1 << 31, 120).unwrap();
    d.transfer(0, &mut ram, 120).unwrap();
    assert_eq!(d.usb_masters[0].epoch, 3);
}

#[test]
fn pmgr_shared_usb_leaf_pauses_both_clients_and_cancels_sessions_only_on_reset_or_detach() {
    use coverage::{Domain, DomainId};
    use stela_soc_data::board::PowerResource;
    let id = |device| DomainId { die: 0, device };
    let mut d = AppleDevices::new();
    d.add_aic(0x3b0c0000, 1100, AicLayout::aic2(0x8000, 0))
        .unwrap();
    let dart_base = 0x382f00000;
    let dart = d
        .add_dart(
            Range {
                physical: dart_base,
                bytes: 0x4000,
            },
            16,
            None,
            None,
        )
        .unwrap();
    let phy = add_usb_phy(&mut d);
    let placement = usb_placement();
    let usb = d.add_usb(&placement, dart).unwrap();
    release_usb_phy(&mut d, &phy);
    let base = placement.ranges[0].0;
    let supply = 0x23d280098;
    let states = stela_soc_data::board::fixture_support::power_states_fixture(
        vec![(supply, 12)],
        vec![supply, supply + 8],
        vec![
            Domain {
                name: String::new(),
                id: id(0x210),
                address: None,
                parents: [Some(id(0x108)), None, None, None],
            },
            Domain {
                name: String::new(),
                id: id(0x108),
                address: Some(supply),
                parents: [Some(id(2)), None, None, None],
            },
            Domain {
                name: String::new(),
                id: id(2),
                address: Some(supply + 8),
                parents: [None; 4],
            },
        ],
        vec![
            PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "dart-usb0".into(),
                clocks: vec![id(0x210)],
                power: vec![],
                owner: Some(PowerOwner::Dart(0)),
                admission: coverage::Admission::Required,
            },
            PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "usb-drd0".into(),
                clocks: vec![id(0x108)],
                power: vec![],
                owner: Some(PowerOwner::Usb(0)),
                admission: coverage::Admission::Required,
            },
            PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "usb-drd0/port-hs".into(),
                clocks: vec![id(0x108)],
                power: vec![],
                owner: Some(PowerOwner::Usb(0)),
                admission: coverage::Admission::Required,
            },
            PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "acio0".into(),
                clocks: vec![id(2)],
                power: vec![],
                owner: None,
                admission: coverage::Admission::Required,
            },
            PowerResource {
                clock_role: stela_soc_data::board::ClockRole::Consumer,
                name: "atc-phy0".into(),
                clocks: vec![],
                power: vec![],
                owner: Some(PowerOwner::Phy(0)),
                admission: coverage::Admission::Required,
            },
        ],
    );
    d.add_power_states(
        &[Range {
            physical: supply,
            bytes: 12,
        }],
        &states.registers,
    )
    .unwrap();
    d.connect_power_resources(&states, &[vec![dart]], &[false])
        .unwrap();
    assert_eq!(d.power_refusals().len(), 1);
    assert!(d.write(0, supply + 8, 4, 0, 0).is_err());
    let host = typec::Attached {
        orientation: typec::Orientation::Normal,
        role: typec::DataRole::Ufp,
        source: false,
        vbus: true,
        speed: typec::Usb2Speed::High,
    };
    d.set_connection(1, Some(host)).unwrap();
    let restore_dart = |d: &mut AppleDevices| {
        d.write(0, dart_base + 0xc00, 4, 2, 0).unwrap();
        d.write(0, dart_base + 0x1004, 4, 6, 0).unwrap();
    };
    restore_dart(&mut d);
    d.write(0, base + 0xc704, 4, 1 << 31, 0).unwrap(); // DCTL RUN
    let mut ram = Ram::new(0x800000000, 0x10000);
    d.transfer(0, &mut ram, 0).unwrap();
    assert!(d.usb_masters[0].attached);
    assert_eq!(d.usb_masters[0].epoch, 1);
    d.write(0, supply, 4, 4, 10).unwrap();
    assert!(d.read(0, base + 0xc704, 4, 10).is_err());
    assert!(d.read(0, dart_base + 0x1004, 4, 10).is_err());
    assert_eq!(d.dma_deadline(), None);
    d.transfer(0, &mut ram, 20).unwrap();
    assert!(d.usb_masters[0].attached);
    assert_eq!(d.usb_masters[0].epoch, 1);
    d.write(0, supply, 4, 15, 30).unwrap();
    d.transfer(0, &mut ram, 30).unwrap();
    assert_eq!(d.usb_masters[0].epoch, 1);
    for value in [0x40f, 0x8000040f, 0x40f, 15] {
        d.write(0, supply, 4, value, 40).unwrap();
    }
    assert_eq!(d.typec_port(1).snapshot().connection, Some(host));
    restore_dart(&mut d);
    d.transfer(0, &mut ram, 50).unwrap();
    assert!(!d.usb_masters[0].attached);
    assert_eq!(d.read(0, base + 0xc704, 4, 50), Ok(0));
    d.write(0, base + 0xc704, 4, 1 << 31, 60).unwrap();
    d.transfer(0, &mut ram, 60).unwrap();
    assert!(d.usb_masters[0].attached);
    assert_eq!(d.usb_masters[0].epoch, 2);
    // Cable removal cancels the connection even while the clock is off.
    d.write(0, supply, 4, 4, 70).unwrap();
    d.set_connection(1, None).unwrap();
    d.transfer(0, &mut ram, 70).unwrap();
    assert!(!d.usb_masters[0].attached);
    d.set_connection(1, Some(host)).unwrap();
    d.transfer(0, &mut ram, 80).unwrap();
    d.write(0, supply, 4, 15, 90).unwrap();
    d.transfer(0, &mut ram, 90).unwrap();
    assert!(d.usb_masters[0].attached);
    assert_eq!(d.usb_masters[0].epoch, 3);
    assert_eq!(
        d.usb_client(usb).unwrap().mux_state(),
        MuxState::Negotiating
    );
}

#[test]
fn a_late_usb_poll_moves_the_packets_of_the_microframes_it_missed() {
    let frame = USB_POLL_TICKS;
    assert_eq!(usb_poll(frame, frame), (13, 2 * frame));
    assert_eq!(usb_poll(2 * frame - 1, frame), (13, 2 * frame));
    assert_eq!(usb_poll(3 * frame + 7, frame), (3 * 13, 4 * frame));
    // A frame's at most, as for a session's first poll.
    assert_eq!(usb_poll(100 * frame + 7, frame), (8 * 13, 101 * frame));
    assert_eq!(usb_poll(100 * frame, 0), (8 * 13, 101 * frame));

    // A polling interval just under two microframes must still service
    // each elapsed microframe. Resetting the phase loses almost half.
    let mut due = frame;
    let mut packets = 0;
    for poll in 1..=100 {
        let now = poll * (2 * frame - 1);
        let (budget, next) = usb_poll(now, due);
        packets += budget;
        due = next;
    }
    assert_eq!(packets, 199 * 13);
    assert_eq!(due, 200 * frame);
}

#[test]
fn usb_core_aliases_platform_refusals_and_port_driven_polling() {
    const BASE: u64 = 0x382280000;
    let mut devices = AppleDevices::new();
    let dart = devices
        .add_dart(
            Range {
                physical: 0x382f00000,
                bytes: 0x4000,
            },
            16,
            Some(AicInput::new(0, 1035)),
            None,
        )
        .unwrap();
    let phy = add_usb_phy(&mut devices);
    let placement = usb_placement();
    let id = devices.add_usb(&placement, dart).unwrap();
    release_usb_phy(&mut devices, &phy);
    assert_eq!(devices.read(0, BASE + 0xc120, 4, 0), Ok(0x5533_300a));
    devices
        .write(0, placement.ranges[2].0 + 0x700, 4, 5 << 3, 0)
        .unwrap();
    assert_eq!(devices.read(0, BASE + 0xc700, 4, 0), Ok(5 << 3));
    // DRD_USB31_CFG's RAM status and the power block's request are
    // modeled; the other platform words refuse.
    for index in [1, 5] {
        assert_eq!(devices.read(0, placement.ranges[index].0, 4, 0), Ok(0));
    }
    for index in [6, 7, 8] {
        assert!(matches!(
            devices.read(0, placement.ranges[index].0, 4, 0),
            Err(DeviceError::Refused(_))
        ));
    }
    // The pipehandler and core event block are the ATC PHY's.
    assert_eq!(devices.read(0, placement.ranges[3].0 + 0x1c, 4, 0), Ok(1));
    assert_eq!(devices.read(0, placement.ranges[4].0, 4, 0), Ok(0));
    devices.transfer(0, &mut Ram::new(0, 0x1000), 0).unwrap();
    assert_eq!(devices.dma_deadline(), None);
    let host = typec::Attached {
        orientation: typec::Orientation::Normal,
        role: typec::DataRole::Ufp,
        source: false,
        vbus: true,
        speed: typec::Usb2Speed::High,
    };
    devices.set_connection(1, Some(host)).unwrap();
    assert_eq!(devices.dma_deadline(), Some(0));
    let mut ram = stela_contracts::dma::test_support::Ram::new(0x800000000, 0x10000);
    devices.transfer(0, &mut ram, 0).unwrap();
    assert_eq!(devices.dma_deadline(), Some(USB_POLL_TICKS));
    assert!(devices.usb_client(id).unwrap().open(62078).is_err());
    // Without VBUS the controller is not ready; the change detaches.
    let unpowered = typec::Attached {
        vbus: false,
        ..host
    };
    devices.set_connection(1, Some(unpowered)).unwrap();
    assert_eq!(devices.dma_deadline(), Some(0));
    devices.transfer(0, &mut ram, USB_POLL_TICKS).unwrap();
    assert_eq!(devices.dma_deadline(), None);
}

#[test]
fn a_gate_store_is_served_in_its_own_pes_step() {
    extern crate std;

    use core::num::NonZeroU64;
    use stela_contracts::gate::{CallerFault, CallerMemory};
    const GATE: u64 = 0x5_0000_0000;
    #[derive(Default)]
    struct Service(std::sync::Mutex<Vec<(u64, u64)>>);
    impl stela_contracts::gate::GateService for Service {
        fn connect(&self) -> Result<NonZeroU64, &'static str> {
            Ok(NonZeroU64::MIN)
        }
        fn disconnect(&self, _: NonZeroU64) {}
        fn serve(
            &self,
            connection: NonZeroU64,
            record: u64,
            _: &mut dyn CallerMemory,
        ) -> Result<(), CallerFault> {
            self.0.lock().unwrap().push((connection.get(), record));
            Ok(())
        }
        fn import(
            &self,
            _: NonZeroU64,
            _: &[stela_contracts::gate::MemoryRange],
            _: NonZeroU64,
        ) -> Result<NonZeroU64, &'static str> {
            Err("no memory")
        }
        fn retired(&self) -> Option<NonZeroU64> {
            None
        }
        fn quiesced(&self) -> bool {
            false
        }
        fn signaled(&self) -> bool {
            false
        }
        fn bundle(&self, _: stela_contracts::gate::BundleReport) {}
    }
    struct Caller;
    impl CallerMemory for Caller {
        fn read(&mut self, address: u64, _: &mut [u8]) -> Result<(), CallerFault> {
            Err(CallerFault::Denied { address })
        }
        fn write(&mut self, address: u64, _: &[u8]) -> Result<(), CallerFault> {
            Err(CallerFault::Denied { address })
        }
    }
    let service = alloc::sync::Arc::new(Service::default());
    let mut d = AppleDevices::new();
    let irq = AicInput::new(0, 700);
    d.add_gpu_gate(GATE, irq, service.clone()).unwrap();
    assert!(
        d.add_gpu_gate(GATE, irq, service.clone()).is_err(),
        "a gate's window is its own"
    );
    let second = GATE + stela_contracts::gate::window::BYTES;
    d.add_gpu_gate(second, AicInput::new(0, 701), service.clone())
        .unwrap();
    // A call finishes at the gate that took it.
    d.write(3, second, 8, 0, 0).unwrap();
    let outcome = d.take_call(3).unwrap().run(&mut Caller);
    assert_eq!(outcome.window, 1);
    d.finish_call(outcome);
    assert_eq!(d.read(3, second + 8, 8, 0).unwrap(), 1);
    assert_eq!(d.read(3, GATE + 8, 8, 0).unwrap(), 0);
    d.write(0, GATE, 8, 0, 0).unwrap();
    let outcome = d.take_call(0).unwrap().run(&mut Caller);
    d.finish_call(outcome);
    let page = d.read(0, GATE + 8, 8, 0).unwrap();
    assert_eq!(page, 1);
    d.write(
        2,
        GATE + page * stela_contracts::gate::window::PAGE,
        8,
        0x1234,
        0,
    )
    .unwrap();
    assert!(d.take_call(1).is_none());
    let outcome = d.take_call(2).unwrap().run(&mut Caller);
    d.finish_call(outcome);
    assert_eq!(*service.0.lock().unwrap(), [(1, 0x1234)]);
}

mod data_integration;
mod passive;
mod pc_sampler;
