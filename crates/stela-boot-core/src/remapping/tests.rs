//! Complete topology and firmware remapping policy regressions.
use super::*;
use alloc::{collections::BTreeMap, vec};

/// A function's requester, class, multifunction bit and bridge buses.
type Function = (u16, [u8; 3], bool, Option<(u8, u8)>);

/// A config space of `(rid, offset) -> dword`.
fn space(functions: &[Function]) -> BTreeMap<(u16, u16), u32> {
    let mut space = BTreeMap::new();
    for &(rid, class, multi, bridge) in functions {
        space.insert((rid, VENDOR), 0x1234_8086);
        space.insert(
            (rid, CLASS),
            u32::from(class[0]) << 24 | u32::from(class[1]) << 16 | u32::from(class[2]) << 8,
        );
        let kind = u32::from(bridge.is_some()) | if multi { 0x80 } else { 0 };
        space.insert((rid, HEADER), kind << 16);
        if let Some((secondary, subordinate)) = bridge {
            space.insert(
                (rid, BUSES),
                u32::from(secondary) << 8 | u32::from(subordinate) << 16,
            );
        }
    }
    space
}

fn inventory() -> Vec<Present> {
    let space = space(&[
        (0x0000, [6, 0, 0], false, None),
        (0x0010, [3, 0, 0], false, None),
        (0x00a0, [0x0c, 3, 0x30], true, None),
        (0x00a1, [5, 0, 0], true, None),
        (0x00e0, [6, 4, 0], false, Some((1, 2))),
        (0x0100, [6, 4, 0], false, Some((2, 2))),
        (0x0200, [1, 8, 2], false, None),
    ]);
    enumerate(&[0], |rid, offset| space.get(&(rid, offset)).copied())
}

#[test]
fn window_geometry_rejects_empty_overflow_and_out_of_width_ranges() {
    assert_eq!(
        window_range(&(0x12345..0x12346), 48),
        Some(0x12000..0x13000)
    );
    let limit = 1u64 << 52;
    assert_eq!(
        window_range(&(limit - 1..limit), 52),
        Some(limit - 4096..limit)
    );
    for range in [
        0..0,
        0x1000..0x1000,
        0..u64::MAX,
        u64::MAX - 4096..u64::MAX,
        limit..limit + 1,
    ] {
        assert_eq!(window_range(&range, 52), None);
    }
    for bits in [0, 31, 53, 64] {
        assert_eq!(window_range(&(0x1000..0x2000), bits), None);
    }
}

#[test]
fn enumeration_follows_bridges_and_multifunction_devices() {
    let found = inventory();
    let rids: Vec<u16> = found.iter().map(|function| function.rid).collect();
    assert_eq!(
        rids,
        [0x0000, 0x0010, 0x00a0, 0x00a1, 0x00e0, 0x0100, 0x0200]
    );
    assert!(found[1].display());
    assert!(found[2].usb());
    assert_eq!(found[4].bridge, Some((1, 2)));
}

#[test]
fn disjoint_ecam_windows_share_one_complete_segment_inventory() {
    use crate::dmar::{Drhd, Remapping};
    let windows = [
        EcamWindow {
            base: 0x8000_0000,
            segment: 0,
            start_bus: 1,
            end_bus: 1,
        },
        EcamWindow {
            base: 0x9000_0000,
            segment: 0,
            start_bus: 7,
            end_bus: 7,
        },
        EcamWindow {
            base: 0xa000_0000,
            segment: 3,
            start_bus: 1,
            end_bus: 1,
        },
    ];
    let first = space(&[(0x0100, [1, 8, 2], false, None)]);
    let second = space(&[(0x0700, [2, 0, 0], false, None)]);
    let other = space(&[(0x0100, [0x0c, 3, 0x30], false, None)]);
    let inventories = enumerate_segments(&windows, |window, rid, offset| {
        assert!((window.start_bus..=window.end_bus).contains(&((rid >> 8) as u8)));
        let config = match window.base {
            0x8000_0000 => &first,
            0x9000_0000 => &second,
            0xa000_0000 => &other,
            _ => panic!("foreign ECAM window"),
        };
        config.get(&(rid, offset)).copied()
    });
    assert_eq!(inventories.len(), 2);
    assert_eq!(inventories[0].0, 0);
    assert_eq!(
        inventories[0].1.iter().map(|f| f.rid).collect::<Vec<_>>(),
        [0x0100, 0x0700]
    );
    assert_eq!(inventories[1].0, 3);
    assert_eq!(inventories[1].1[0].rid, 0x0100);
    assert!(inventories[1].1[0].usb());
    let remapping = Remapping {
        physical_bits: 39,
        flags: 0,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![Drhd {
            base: 0xfed9_0000,
            bytes: 4096,
            segment: 0,
            include_all: true,
            scopes: vec![],
            sources: vec![],
        }],
        reserved: vec![],
    };
    assert_eq!(
        plan(&remapping, 0, &inventories[0].1, &[]),
        [(0, Ok(vec![0x0100, 0x0700]))]
    );
}

/// Two laptops' fabrics: functions that read vendor ID 0000 beside a
/// device's function 0 (00:00.3, 00:03.2 to 00:03.4), an idle CardBus
/// header (00:00.5), a repeat of a single-function device's function 0
/// (00:04.1) and a function behind a bridge whose buses are not used
/// (05:00.0). The containment census leaves each out; the inventory after
/// it reads them as absent and finds exactly the functions the fabric
/// holds, where each one was refused as a function containment did not
/// hold and the GPU with it.
#[test]
fn the_inventory_after_containment_passes_over_what_the_census_left_out() {
    use crate::ecam::census::LeftOut;
    use stela_drivers::bus::pci::Address;
    let windows = [EcamWindow {
        base: 0xe000_0000,
        segment: 0,
        start_bus: 0,
        end_bus: 5,
    }];
    let mut config = space(&[
        (0x0000, [6, 0, 0], true, None),
        (0x0008, [6, 4, 0], false, Some((1, 1))),
        (0x0018, [6, 0, 0], true, None),
        (0x0020, [6, 0, 0], false, None),
        (0x0028, [6, 4, 0], false, Some((5, 5))),
        (0x0100, [3, 0, 0], false, None),
        (0x0500, [2, 0, 0], false, None),
    ]);
    for rid in [0x0003, 0x001a, 0x001b, 0x001c] {
        config.insert((rid, VENDOR), 0);
        config.insert((rid, CLASS), 0);
        config.insert((rid, HEADER), 0);
    }
    config.insert((0x0005, VENDOR), 0x1234_8086);
    config.insert((0x0005, CLASS), 0x0607_0000);
    config.insert((0x0005, HEADER), 2 << 16);
    config.insert((0x0021, VENDOR), 0x1234_8086);
    config.insert((0x0021, CLASS), 0x0600_0000);
    config.insert((0x0021, HEADER), 0);
    let at = |rid: u16| Address {
        segment: 0,
        bus: (rid >> 8) as u8,
        device: (rid >> 3) as u8 & 0x1f,
        function: rid as u8 & 7,
    };
    let mut left_out = LeftOut::default();
    for rid in [0x0003, 0x0005, 0x001a, 0x001b, 0x001c, 0x0021] {
        left_out.function(at(rid)).unwrap();
    }
    left_out.bus(0, 5).unwrap();
    // What the boot's inventory read does with the census's record: a
    // function the fabric does not hold refuses.
    let held = [0x0000, 0x0008, 0x0018, 0x0020, 0x0028, 0x0100];
    let inventories = try_enumerate_segments(&windows, |_, rid, offset| {
        if left_out.holds(at(rid)) {
            return Ok(None);
        }
        let value = config.get(&(rid, offset)).copied();
        if offset == VENDOR && value.is_some() && !held.contains(&rid) {
            return Err(rid);
        }
        Ok(value)
    });
    let found = inventories.map(|segments| {
        segments[0]
            .1
            .iter()
            .map(|function| function.rid)
            .collect::<Vec<_>>()
    });
    assert_eq!(found, Ok(held.to_vec()));
    // Without the record a function left out refuses the inventory: the
    // last bus is read first.
    let refused = try_enumerate_segments(&windows, |_, rid, offset| {
        let value = config.get(&(rid, offset)).copied();
        if offset == VENDOR && value.is_some() && !held.contains(&rid) {
            return Err(rid);
        }
        Ok(value)
    });
    assert_eq!(refused, Err(0x0500));
}

#[test]
fn a_failed_configuration_read_refuses_the_entire_segment_census() {
    let windows = [EcamWindow {
        base: 0x8000_0000,
        segment: 0,
        start_bus: 1,
        end_bus: 1,
    }];
    let mut reads = Vec::new();
    let result = try_enumerate_segments(&windows, |_, rid, offset| {
        reads.push((rid, offset));
        match (rid, offset) {
            (0x0100 | 0x0101, VENDOR) => Ok(Some(0x0001_1234)),
            (0x0100, CLASS | HEADER) => Ok(Some(0)),
            (0x0101, CLASS) => Err("configuration mapping lost"),
            _ => panic!("continued reading after the failed access"),
        }
    });
    assert_eq!(result, Err("configuration mapping lost"));
    assert_eq!(
        reads,
        [
            (0x0100, VENDOR),
            (0x0100, CLASS),
            (0x0100, HEADER),
            (0x0101, VENDOR),
            (0x0101, CLASS),
        ]
    );
}

#[test]
fn scopes_resolve_endpoints_paths_and_bridge_subtrees() {
    let present = inventory();
    let endpoint = Scope {
        bridge: false,
        bus: 0,
        path: vec![(0x1c, 0), (0, 0), (0, 0)],
    };
    assert_eq!(endpoint.resolve(&present), [0x0200]);
    let bridge = Scope {
        bridge: true,
        bus: 0,
        path: vec![(0x1c, 0)],
    };
    assert_eq!(bridge.resolve(&present), [0x00e0, 0x0100, 0x0200]);
    let absent = Scope {
        bridge: false,
        bus: 0,
        path: vec![(5, 0)],
    };
    assert!(absent.resolve(&present).is_empty());
}

#[test]
fn an_include_all_unit_covers_what_listed_units_leave() {
    use crate::dmar::{Drhd, Remapping, Rmrr};
    let present = inventory();
    let unit = |include_all, scopes| Drhd {
        base: 0xfed9_0000,
        bytes: 4096,
        segment: 0,
        include_all,
        scopes,
        sources: vec![],
    };
    let graphics = Scope {
        bridge: false,
        bus: 0,
        path: vec![(2, 0)],
    };
    let mut remapping = Remapping {
        physical_bits: 39,
        flags: 0,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![unit(true, vec![]), unit(false, vec![graphics])],
        reserved: vec![Rmrr {
            segment: 0,
            base: 0x7d00_0000,
            limit: 0x7d01_ffff,
            scopes: vec![Scope {
                bridge: false,
                bus: 0,
                path: vec![(0x14, 0)],
            }],
            sources: vec![],
        }],
    };
    let decisions = plan(&remapping, 0, &present, &[]);
    assert_eq!(decisions[0], (1, Err(Leave::Display(0x0010))));
    assert_eq!(
        decisions[1],
        (0, Ok(vec![0x0000, 0x00a0, 0x00a1, 0x00e0, 0x0100, 0x0200]))
    );
    assert!(plan(&remapping, 1, &present, &[]).is_empty());
    remapping.reserved[0].scopes[0].path = vec![(0x1c, 0), (0, 0), (0, 0)];
    assert_eq!(
        plan(&remapping, 0, &present, &[])[1],
        (0, Err(Leave::Reserved(0x0200)))
    );
}

#[test]
fn a_claimed_display_may_keep_its_reserved_ranges_but_other_classes_may_not() {
    let present = inventory();
    let all = remainder(&present, &[]);
    assert_eq!(
        decide(&all, &[0x0010], &present, &[0x0010]),
        Ok(all.clone())
    );
    assert_eq!(
        decide(&all, &[0x0010], &present, &[]),
        Err(Leave::Display(0x0010))
    );
    assert_eq!(
        decide(&all, &[0x0200], &present, &[0x0010, 0x0200]),
        Err(Leave::Reserved(0x0200))
    );
}

#[test]
fn displays_and_non_usb_reservations_leave_a_unit_untranslated() {
    let present = inventory();
    let all = remainder(&present, &[]);
    assert_eq!(
        decide(&all, &[], &present, &[]),
        Err(Leave::Display(0x0010))
    );
    assert_eq!(decide(&all, &[], &present, &[0x0010]), Ok(all.clone()));
    let rest = remainder(&present, &[0x0010]);
    assert_eq!(decide(&rest, &[0x00a0], &present, &[]), Ok(rest.clone()));
    assert_eq!(
        decide(&rest, &[0x0200], &present, &[]),
        Err(Leave::Reserved(0x0200))
    );
    // A reservation for a requester another unit covers does not count.
    assert_eq!(
        decide(&[0x0200], &[0x0010], &present, &[]),
        Ok(vec![0x0200])
    );
    assert_eq!(decide(&[], &[], &present, &[]), Err(Leave::Empty));
}

fn drhd(include_all: bool, scopes: Vec<Scope>) -> crate::dmar::Drhd {
    crate::dmar::Drhd {
        base: 0xfed90000,
        bytes: 4096,
        segment: 0,
        include_all,
        scopes,
        sources: vec![],
    }
}
#[test]
fn ambiguous_explicit_units_and_duplicate_include_all_units_are_not_taken() {
    let present = inventory();
    let endpoint = Scope {
        bridge: false,
        bus: 0,
        path: vec![(0x1c, 0), (0, 0), (0, 0)],
    };
    for units in [
        vec![
            drhd(false, vec![endpoint.clone()]),
            drhd(false, vec![endpoint]),
        ],
        vec![drhd(true, vec![]), drhd(true, vec![])],
    ] {
        let remapping = crate::dmar::Remapping {
            physical_bits: 39,
            flags: 0,
            additional_structures: vec![],
            skipped: vec![],
            units,
            reserved: vec![],
        };
        assert_eq!(
            route(&remapping, 0, &present, 0x200),
            Err(Leave::Ambiguous(0x200))
        );
        assert!(
            plan(&remapping, 0, &present, &[])
                .iter()
                .all(|(_, decision)| matches!(decision, Err(Leave::Ambiguous(_))))
        );
    }
    let remapping = crate::dmar::Remapping {
        physical_bits: 39,
        flags: 0,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![drhd(
            false,
            vec![
                Scope {
                    bridge: false,
                    bus: 2,
                    path: vec![(0, 0)]
                };
                2
            ],
        )],
        reserved: vec![],
    };
    assert_eq!(route(&remapping, 0, &present, 0x200), Ok(Some(0)));
    assert_eq!(plan(&remapping, 0, &present, &[]), [(0, Ok(vec![0x200]))]);
}
#[test]
fn an_arriving_child_uses_its_explicit_unit_even_when_the_bridge_uses_include_all() {
    let mut present = inventory();
    present.retain(|function| function.rid != 0x200);
    let remapping = crate::dmar::Remapping {
        physical_bits: 39,
        flags: 0,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![
            drhd(true, vec![]),
            drhd(
                false,
                vec![Scope {
                    bridge: false,
                    bus: 0,
                    path: vec![(0x1c, 0), (0, 0), (0, 0)],
                }],
            ),
        ],
        reserved: vec![],
    };
    assert_eq!(route(&remapping, 0, &present, 0xe0), Ok(Some(0)));
    assert_eq!(route(&remapping, 0, &present, 0x200), Ok(None));
    present.push(Present {
        rid: 0x200,
        class: [1, 8, 2],
        bridge: None,
    });
    assert_eq!(route(&remapping, 0, &present, 0x200), Ok(Some(1)));
    assert_eq!(route(&remapping, 1, &present, 0x200), Ok(None));
    assert_eq!(route(&remapping, 0, &present, 0x208), Ok(None));
}

#[test]
fn future_isolation_includes_unresolved_child_scopes_and_excludes_unrelated_endpoints() {
    let mut present = inventory();
    present.retain(|entry| entry.rid != 0x200);
    let mut remapping = crate::dmar::Remapping {
        physical_bits: 39,
        flags: 0,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![
            drhd(true, vec![]),
            drhd(
                false,
                vec![Scope {
                    bridge: false,
                    bus: 0,
                    path: vec![(0x1c, 0), (0, 0), (0, 0)],
                }],
            ),
            drhd(
                false,
                vec![Scope {
                    bridge: false,
                    bus: 0,
                    path: vec![(2, 0)],
                }],
            ),
        ],
        reserved: vec![],
    };
    assert_eq!(
        subtree_routes(&remapping, 0, &present, 0xe0),
        Some(vec![1, 0])
    );
    assert_eq!(subtree_routes(&remapping, 0, &present, 0x200), None);
    assert_eq!(subtree_routes(&remapping, 1, &present, 0xe0), Some(vec![]));
    // An explicit bridge scope covers the entire subtree, so unrelated
    // untranslated include-all/display units cannot route its children.
    remapping.units[1].scopes = vec![Scope {
        bridge: true,
        bus: 0,
        path: vec![(0x1c, 0)],
    }];
    assert_eq!(subtree_routes(&remapping, 0, &present, 0xe0), Some(vec![1]));
    // An endpoint scope naming only the bridge says nothing about its children.
    remapping.units[1].scopes[0].bridge = false;
    assert_eq!(subtree_routes(&remapping, 0, &present, 0xe0), Some(vec![0]));
}

#[test]
fn inventory_cannot_omit_a_pf_because_function_zero_or_its_multifunction_bit_is_absent() {
    let space = space(&[
        (0x0000, [6, 0, 0], false, None),
        (0x0001, [2, 0, 0], false, None),
        (0x0009, [2, 0, 0], false, None),
    ]);
    let present = enumerate(&[0], |rid, offset| space.get(&(rid, offset)).copied());
    assert_eq!(
        present.iter().map(|entry| entry.rid).collect::<Vec<_>>(),
        [0, 1, 9]
    );
}

#[test]
fn unowned_firmware_dma_sources_block_plans_routes_and_future_isolation() {
    use crate::dmar::{Remapping, Rmrr, SourceKind, SourceScope};
    let present = inventory();
    let source = |kind| SourceScope {
        kind,
        flags: 0,
        reserved: 0,
        enumeration_id: 9,
        bus: 0,
        path: vec![(0x1f, 0)],
    };
    let baseline = Remapping {
        physical_bits: 39,
        flags: 1,
        additional_structures: vec![],
        skipped: vec![],
        units: vec![drhd(true, vec![])],
        reserved: vec![],
    };
    for kind in [SourceKind::IoApic, SourceKind::Hpet] {
        let mut remapping = baseline.clone();
        remapping.units[0].sources.push(source(kind));
        assert!(plan(&remapping, 0, &present, &[0x10])[0].1.is_ok());
    }
    for kind in [SourceKind::Namespace, SourceKind::Reserved(0x7f)] {
        for in_reserved in [false, true] {
            let mut remapping = baseline.clone();
            if in_reserved {
                remapping.reserved.push(Rmrr {
                    segment: 0,
                    base: 0x1000,
                    limit: 0x1fff,
                    scopes: vec![],
                    sources: vec![source(kind)],
                });
            } else {
                remapping.units[0].sources.push(source(kind));
            }
            assert_eq!(
                plan(&remapping, 0, &present, &[]),
                [(0, Err(Leave::UnsupportedSource(kind)))]
            );
            assert_eq!(
                route(&remapping, 0, &present, 0x0200),
                Err(Leave::UnsupportedSource(kind))
            );
            assert_eq!(subtree_routes(&remapping, 0, &present, 0x00e0), None);
            remapping.units[0].segment = 1;
            remapping
                .reserved
                .iter_mut()
                .for_each(|region| region.segment = 1);
            remapping.units.push(drhd(true, vec![]));
            assert_eq!(route(&remapping, 0, &present, 0x0200), Ok(Some(1)));
        }
    }
    for kind in [5, 6, 0x7f] {
        let mut remapping = baseline.clone();
        remapping.additional_structures.push(kind);
        assert_eq!(
            plan(&remapping, 0, &present, &[]),
            [(0, Err(Leave::UnsupportedStructure(kind)))]
        );
        assert_eq!(
            route(&remapping, 0, &present, 0x0200),
            Err(Leave::UnsupportedStructure(kind))
        );
        assert_eq!(subtree_routes(&remapping, 0, &present, 0x00e0), None);
    }
}

#[test]
fn a_dropped_scope_or_region_leaves_every_unit_while_ignored_bits_do_not() {
    use crate::dmar::{Remapping, Skipped};
    let present = inventory();
    let baseline = Remapping {
        physical_bits: 39,
        flags: 1,
        additional_structures: vec![],
        skipped: vec![Skipped::ReservedBits, Skipped::UnalignedRegion],
        units: vec![drhd(true, vec![])],
        reserved: vec![],
    };
    assert!(plan(&baseline, 0, &present, &[0x10])[0].1.is_ok());
    for dropped in [Skipped::UnusableScope, Skipped::UnusableRegion] {
        let mut remapping = baseline.clone();
        remapping.skipped.push(dropped);
        assert_eq!(
            plan(&remapping, 0, &present, &[0x10]),
            [(0, Err(Leave::Incomplete))]
        );
        assert_eq!(
            route(&remapping, 0, &present, 0x0200),
            Err(Leave::Incomplete)
        );
        assert_eq!(subtree_routes(&remapping, 0, &present, 0x00e0), None);
    }
}
