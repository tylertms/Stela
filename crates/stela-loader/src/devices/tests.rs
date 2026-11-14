use super::*;

fn node(name: &str, compatible: Option<&[u8]>, children: Vec<Node>) -> Node {
    let mut node = Node::named(name).unwrap();
    if let Some(value) = compatible {
        node.set_property(COMPATIBLE, value.to_vec()).unwrap();
    }
    node.children = children;
    node
}

fn function_client(name: &str, phandle_value: u32, provider: u32) -> Node {
    let mut client = phandle(node(name, None, vec![]), phandle_value);
    client
        .set_property(DEVICE_TYPE, b"amfm\0".to_vec())
        .unwrap();
    client
        .set_property(
            "function-pcie_port_control",
            [provider, four_char_code(b"PrtC"), 0x36]
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect(),
        )
        .unwrap();
    client
}

#[test]
fn a_name_matched_client_needs_the_parent_it_waits_for() {
    let audio = |parent: u32| {
        let mut client = typed(node("dp-audio0", None, vec![]), b"dp-audio0\0");
        client
            .set_property("dma-parent", parent.to_le_bytes().to_vec())
            .unwrap();
        client
    };
    let tree = |parent, provider: Option<&[u8]>| DeviceTree {
        root: node(
            "",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    phandle(node("sio-dma", provider, vec![]), 7),
                    audio(parent),
                    typed(node("dp-audio1", None, vec![]), b"dp-audio1\0"),
                ],
            )],
        ),
    };
    let mut absent = tree(7, None);
    let mut hidden = Vec::new();
    hide_unprovided_clients(&mut absent, &mut hidden);
    assert_eq!(hidden, ["/arm-io/dp-audio0", "/arm-io/dp-audio1"]);
    assert!(absent.property("/arm-io/dp-audio0", IGNORE).is_some());

    let mut present = tree(7, Some(b"sio-dma,t8112\0"));
    let mut hidden = Vec::new();
    hide_unprovided_clients(&mut present, &mut hidden);
    assert_eq!(hidden, ["/arm-io/dp-audio1"]);
    assert!(present.property("/arm-io/dp-audio0", IGNORE).is_none());
}

#[test]
fn name_matched_clients_require_an_admitted_function_provider() {
    let tree = |provider| DeviceTree {
        root: node(
            "",
            None,
            vec![function_client("radio", 0x24, 0x35), provider],
        ),
    };
    let mut hidden = tree(phandle(node("pcie", Some(b"apcie,t8112\0"), vec![]), 0x35));
    let original = hidden.node("/radio").unwrap().clone();
    assert_eq!(hide_unmodeled(&mut hidden), ["/pcie", "/radio"]);
    assert!(hidden.property("/radio", IGNORE).is_some());
    let mut retained = hidden.node("/radio").unwrap().clone();
    retained.remove_property(IGNORE);
    assert_eq!(retained, original);
    assert!(hide_unmodeled(&mut hidden).is_empty());

    let mut absent = tree(phandle(node("other", Some(b"uart-1,samsung\0"), vec![]), 7));
    assert_eq!(hide_unmodeled(&mut absent), ["/radio"]);
    // Availability follows the declared provider, never its node name.
    let mut provided = tree(phandle(
        node("device", Some(b"uart-1,samsung\0"), vec![]),
        0x35,
    ));
    assert!(hide_unmodeled(&mut provided).is_empty());
    assert!(provided.property("/radio", IGNORE).is_none());
    provided
        .node_mut("/device")
        .unwrap()
        .set_property(IGNORE, vec![])
        .unwrap();
    assert_eq!(hide_unmodeled(&mut provided), ["/radio"]);
}

#[test]
fn required_function_chains_are_proven_and_unrooted_cycles_are_hidden() {
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![
                function_client("first", 1, 2),
                function_client("second", 2, 3),
                phandle(node("provider", Some(b"uart-1,samsung\0"), vec![]), 3),
            ],
        ),
    };
    assert!(hide_unmodeled(&mut tree).is_empty());
    tree.root.children.pop();
    tree.node_mut("/second")
        .unwrap()
        .set_property(
            "function-pcie_port_control",
            [1u32, four_char_code(b"PrtC")]
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect(),
        )
        .unwrap();
    assert_eq!(hide_unmodeled(&mut tree), ["/first", "/second"]);
    assert!(tree.property("/first", IGNORE).is_some());
    assert!(tree.property("/second", IGNORE).is_some());
    assert!(hide_unmodeled(&mut tree).is_empty());
}

#[test]
fn undeclared_optional_functions_do_not_hide_clients_but_required_records_must_exist() {
    let mut optional = node("optional", None, vec![]);
    optional
        .set_property(DEVICE_TYPE, b"metadata\0".to_vec())
        .unwrap();
    optional
        .set_property("function-send_disp_clock", vec![])
        .unwrap();
    let mut required = function_client("required", 1, 2);
    required.remove_property("function-pcie_port_control");
    let mut tree = DeviceTree {
        root: node("", None, vec![optional, required]),
    };
    assert_eq!(hide_unmodeled(&mut tree), ["/required"]);
    assert!(tree.property("/optional", IGNORE).is_none());
    // A phandle alone is not a complete platform function.
    let mut malformed = function_client("bad", 3, 2);
    malformed
        .set_property("function-pcie_port_control", 2u32.to_le_bytes().to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![
                malformed,
                phandle(node("provider", Some(b"uart-1,samsung\0"), vec![]), 2),
            ],
        ),
    };
    assert_eq!(hide_unmodeled(&mut tree), ["/bad"]);
}

fn wlan(managed: bool) -> Node {
    let mut client = node("wlan", None, vec![]);
    client
        .set_property(DEVICE_TYPE, b"wlan\0".to_vec())
        .unwrap();
    if managed {
        client
            .set_property("amfm-managed-port-control", vec![])
            .unwrap();
    }
    client
}

#[test]
fn managed_wlan_requires_the_amfm_service_and_its_admitted_provider_chain() {
    // The WLAN nub is independent of /amfm, matches by name and has
    // no compatible. Put it first to exercise transitive convergence.
    let tree = |provider| DeviceTree {
        root: node(
            "",
            None,
            vec![wlan(true), function_client("amfm", 0x24, 0x35), provider],
        ),
    };
    let mut provided = tree(phandle(
        node("provider", Some(b"uart-1,samsung\0"), vec![]),
        0x35,
    ));
    assert!(hide_unmodeled(&mut provided).is_empty());
    assert!(provided.property("/wlan", IGNORE).is_none());
    let mut hidden = tree(phandle(
        node("provider", Some(b"apcie,t8112\0"), vec![]),
        0x35,
    ));
    assert_eq!(hide_unmodeled(&mut hidden), ["/provider", "/wlan", "/amfm"]);
    for path in ["/wlan", "/amfm"] {
        assert!(hidden.property(path, IGNORE).is_some());
    }
    assert!(
        hidden
            .property("/wlan", "amfm-managed-port-control")
            .is_some()
    );
    assert!(hide_unmodeled(&mut hidden).is_empty());
    provided
        .node_mut("/amfm")
        .unwrap()
        .set_property(IGNORE, vec![])
        .unwrap();
    assert_eq!(hide_unmodeled(&mut provided), ["/wlan"]);
    let mut missing = DeviceTree {
        root: node("", None, vec![wlan(true)]),
    };
    assert_eq!(hide_unmodeled(&mut missing), ["/wlan"]);
}

#[test]
fn unmarked_wlan_does_not_infer_a_pcie_requirement_from_its_hardware_class() {
    let mut direct = wlan(false);
    direct
        .set_property(
            "function-pcie_port_control",
            [0x35u32, four_char_code(b"PrtC")]
                .into_iter()
                .flat_map(u32::to_le_bytes)
                .collect(),
        )
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![
                direct,
                phandle(node("provider", Some(b"uart-1,samsung\0"), vec![]), 0x35),
            ],
        ),
    };
    assert!(hide_unmodeled(&mut tree).is_empty());
    tree.node_mut("/wlan")
        .unwrap()
        .set_property("amfm-managed-port-control", vec![])
        .unwrap();
    // The marker selects a different required interface: the direct
    // function cannot make a missing AMFM service available.
    assert_eq!(hide_unmodeled(&mut tree), ["/wlan"]);
    let mut absent = DeviceTree {
        root: node("", None, vec![wlan(false)]),
    };
    assert!(hide_unmodeled(&mut absent).is_empty());
    // The driver has an AppleOnboardSerialSync personality too. A
    // WLAN type alone does not establish which bus interface it uses.
    absent
        .node_mut("/wlan")
        .unwrap()
        .set_property(COMPATIBLE, b"uart-1,samsung\0".to_vec())
        .unwrap();
    assert!(hide_unmodeled(&mut absent).is_empty());
    assert!(absent.property("/wlan", COMPATIBLE).is_some());
}

fn typed(mut node: Node, kind: &[u8]) -> Node {
    node.set_property(DEVICE_TYPE, kind.to_vec()).unwrap();
    node
}

#[test]
fn only_hidden_audited_service_classes_retire_their_raw_tree_identity() {
    let original = typed(
        phandle(node("unavailable", Some(b"unmodeled-display\0"), vec![]), 9),
        b"display-subsystem\0",
    );
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![
                original.clone(),
                typed(
                    node("admitted", Some(board::DISPEXT.as_bytes()), vec![]),
                    b"ext-display-subsystem\0",
                ),
                typed(
                    node("unmodeled", Some(b"unknown\0"), vec![]),
                    b"unaudited-device\0",
                ),
            ],
        ),
    };
    let hidden = hide_unmodeled(&mut tree);
    assert_eq!(hidden, ["/unavailable", "/unmodeled"]);
    assert!(tree.property("/unavailable", DEVICE_TYPE).is_none());
    let mut expected = original.clone();
    expected.remove_property(COMPATIBLE);
    expected.remove_property(DEVICE_TYPE);
    // Retirement preserves names, phandles, remaining property order
    // and every other datum. The original input remains unchanged.
    assert_eq!(tree.node("/unavailable").unwrap(), &expected);
    assert_eq!(
        tree.property("/admitted", DEVICE_TYPE).unwrap().value,
        b"ext-display-subsystem\0"
    );
    assert!(tree.property("/admitted", COMPATIBLE).is_some());
    assert_eq!(
        tree.property("/unmodeled", DEVICE_TYPE).unwrap().value,
        b"unaudited-device\0"
    );
    assert!(hide_unmodeled(&mut tree).is_empty());
}

#[test]
fn identity_retirement_requires_a_hidden_ledger_entry_and_preserves_structural_metadata() {
    let mut tree = DeviceTree {
        root: typed(
            node(
                "",
                None,
                vec![
                    typed(
                        node(
                            "arm-io",
                            None,
                            vec![typed(
                                node("unavailable", None, vec![]),
                                b"ext-display-subsystem\0",
                            )],
                        ),
                        b"display-subsystem\0",
                    ),
                    node(
                        "cpus",
                        None,
                        vec![typed(node("cpu0", None, vec![]), b"display-subsystem\0")],
                    ),
                    typed(node("memory", None, vec![]), b"memory\0"),
                    typed(node("metadata", None, vec![]), b"display-subsystem\0"),
                ],
            ),
            b"display-subsystem\0",
        ),
    };
    let original = tree.clone();
    let mut ledger = vec![
        "/".into(),
        "/arm-io".into(),
        "/cpus/cpu0".into(),
        "/memory".into(),
    ];
    retire_hidden_service_identities(&mut tree, &ledger);
    assert_eq!(tree, original);
    ledger.push("/arm-io/unavailable".into());
    retire_hidden_service_identities(&mut tree, &ledger);
    assert!(tree.property("/arm-io/unavailable", DEVICE_TYPE).is_none());
    assert_eq!(
        tree.property("/metadata", DEVICE_TYPE).unwrap().value,
        b"display-subsystem\0"
    );
    assert_eq!(ledger.last().unwrap(), "/arm-io/unavailable");
    let prior = ledger.clone();
    retire_hidden_service_identities(&mut tree, &ledger);
    assert_eq!(ledger, prior);
}

#[test]
fn a_hidden_controller_binds_no_driver_and_keeps_no_client_waiting() {
    let parent = |name, phandle: u32| {
        let mut client = node(name, Some(b"usbc,cd3217\0"), vec![]);
        client
            .set_property(ACIO_PARENT, phandle.to_le_bytes().to_vec())
            .unwrap();
        client
    };
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    typed(phandle(node("acio0", None, vec![]), 7), b"acio\0"),
                    typed(node("atc0-dpphy", None, vec![]), b"AAPL,atc-dpphy\0"),
                    typed(node("atc0-dpin0", None, vec![]), b"AAPL,atc-dpin\0"),
                    typed(phandle(node("acio1", None, vec![]), 8), b"acio\0"),
                    parent("hpm0", 7),
                    parent("hpm1", 8),
                ],
            )],
        ),
    };
    let mut ledger: Vec<String> = ["/arm-io/acio0", "/arm-io/atc0-dpphy", "/arm-io/atc0-dpin0"]
        .map(String::from)
        .into();
    retire_hidden_service_identities(&mut tree, &ledger);
    retire_hidden_acio_parents(&mut tree, &mut ledger);
    for path in ledger.iter().take(3) {
        assert!(tree.property(path, DEVICE_TYPE).is_none(), "{path}");
    }
    assert!(tree.property("/arm-io/hpm0", ACIO_PARENT).is_none());
    assert_eq!(ledger.last().unwrap(), "/arm-io/hpm0 acio-parent");
    assert_eq!(ledger.len(), 4);
    // A controller a model owns keeps its identity and its clients.
    assert!(tree.property("/arm-io/acio1", DEVICE_TYPE).is_some());
    assert!(tree.property("/arm-io/hpm1", ACIO_PARENT).is_some());
}

#[test]
fn a_watchdog_without_a_visible_pmu_says_no_pmu() {
    let tree = |pmu: Option<&[u8]>| DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    node("wdt", Some(b"wdt,t8101\0wdt,s5l8960x\0"), vec![]),
                    node(
                        "nub-spmi0",
                        Some(b"aapl,spmi\0"),
                        vec![node("pmu-main", pmu, vec![])],
                    ),
                ],
            )],
        ),
    };
    // The modeled Dialog PMU stays visible and the watchdog waits for it.
    let mut modeled = tree(Some(b"pmu,spmi\0pmu,stowe\0"));
    assert_eq!(hide_unmodeled(&mut modeled), Vec::<String>::new());
    assert!(modeled.property("/arm-io/wdt", NO_PMU).is_none());
    let mut hidden = tree(Some(b"pmu,acme\0"));
    let paths = hide_unmodeled(&mut hidden);
    assert_eq!(
        paths,
        [
            "/arm-io/nub-spmi0/pmu-main",
            "/arm-io/nub-spmi0",
            "/arm-io/wdt +no-pmu"
        ]
    );
    let no_pmu = hidden.property("/arm-io/wdt", NO_PMU).unwrap();
    assert!(no_pmu.value.is_empty());
    // Already marked: nothing more to say.
    assert_eq!(hide_unmodeled(&mut hidden), Vec::<String>::new());
    // A tree without any PMU node at all is marked the same way.
    let mut absent = tree(None);
    assert_eq!(
        hide_unmodeled(&mut absent),
        ["/arm-io/nub-spmi0", "/arm-io/wdt +no-pmu"]
    );
}

fn spmi_bus(name: &str, handle: u32, children: Vec<Node>) -> Node {
    let mut bus = phandle(node(name, Some(b"aapl,spmi\0"), children), handle);
    bus.set_property(
        "reg",
        [0x1000u64.to_le_bytes(), 0x4000u64.to_le_bytes()].concat(),
    )
    .unwrap();
    // A controller's self-parented vectors do not create a consumer.
    bus.set_property("interrupt-parent", handle.to_le_bytes().to_vec())
        .unwrap();
    bus
}

#[test]
fn unused_spmi_aliases_keep_metadata_without_choosing_a_controller() {
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![
                spmi_bus("first", 10, vec![]),
                spmi_bus("alternative", 20, vec![]),
                spmi_bus(
                    "unsupported",
                    30,
                    vec![node("target", Some(b"unmodeled\0"), vec![])],
                ),
            ],
        ),
    };
    let original = tree.clone();
    assert_eq!(
        hide_unmodeled(&mut tree),
        [
            "/unsupported/target",
            "/first",
            "/alternative",
            "/unsupported"
        ]
    );
    for name in ["first", "alternative", "unsupported"] {
        let mut expected = original.root.child(name).unwrap().clone();
        expected.remove_property(COMPATIBLE);
        if let Some(target) = expected.children.first_mut() {
            target.remove_property(COMPATIBLE);
        }
        assert_eq!(tree.root.child(name).unwrap(), &expected);
    }
    assert!(hide_unmodeled(&mut tree).is_empty());
}

#[test]
fn spmi_targets_and_external_providers_keep_colliding_buses_explicit() {
    let pmu = || node("pmu", Some(b"pmu,spmi\0"), vec![]);
    let mut provider_client = node("provider-client", Some(board::UART.as_bytes()), vec![]);
    provider_client
        .set_property(
            "function-spmi_access",
            [20u32.to_le_bytes(), 1u32.to_le_bytes()].concat(),
        )
        .unwrap();
    let mut interrupt_client = node("interrupt-client", Some(board::UART.as_bytes()), vec![]);
    interrupt_client
        .set_property(
            "interrupt-parent",
            [99u32.to_le_bytes(), 30u32.to_le_bytes()].concat(),
        )
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![
                spmi_bus("with-pmu", 10, vec![pmu()]),
                spmi_bus("function-provider", 20, vec![]),
                spmi_bus("interrupt-provider", 30, vec![]),
                spmi_bus("second-target", 40, vec![pmu()]),
                provider_client,
                interrupt_client,
            ],
        ),
    };
    let original = tree.clone();
    assert!(hide_unmodeled(&mut tree).is_empty());
    // Neither equal MMIO banks nor traversal order authorizes deduplication.
    assert_eq!(tree, original);
}

#[test]
fn unavailable_clients_cannot_keep_an_unused_spmi_provider() {
    let mut unavailable = node("unavailable", Some(b"unmodeled\0"), vec![]);
    unavailable
        .set_property("function-spmi_access", 10u32.to_le_bytes().to_vec())
        .unwrap();
    let mut ignored = node("ignored", Some(board::UART.as_bytes()), vec![]);
    ignored.set_property(IGNORE, vec![]).unwrap();
    ignored
        .set_property("interrupt-parent", 10u32.to_le_bytes().to_vec())
        .unwrap();
    let mut metadata_client = node("name-matched", None, vec![]);
    metadata_client
        .set_property("function-spmi_access", 20u32.to_le_bytes().to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![
                spmi_bus("unused", 10, vec![]),
                spmi_bus("named-provider", 20, vec![]),
                unavailable,
                ignored,
                metadata_client,
            ],
        ),
    };
    assert_eq!(hide_unmodeled(&mut tree), ["/unavailable", "/unused"]);
    assert!(tree.property("/unused", COMPATIBLE).is_none());
    assert!(tree.property("/named-provider", COMPATIBLE).is_some());
}

#[test]
#[ignore = "oracle: STELA_TEST_LOADER_DT_DIR"]
fn original_m4_imac_unused_spmi_alternatives_are_all_retired() {
    let directory = std::env::var("STELA_TEST_LOADER_DT_DIR").unwrap();
    for (board, digest) in [
        (
            "j623ap",
            "b1ea7673bb96e22f2e5fcb755045c9c96de8e0fd3d3c30b7251f22d080b19f9d",
        ),
        (
            "j624ap",
            "21286eb2e7186a90106d75a38659e2081f22f514b49b884cb5e92e28d05383bc",
        ),
    ] {
        let input = std::fs::read(format!("{directory}/DeviceTree.{board}")).unwrap();
        assert_eq!(stela_formats::sha256(&input), digest);
        let mut tree = DeviceTree::parse(&input).unwrap();
        let original = tree.clone();
        let hidden = hide_unmodeled(&mut tree);
        fn metadata(mut node: Node) -> Node {
            node.remove_property(COMPATIBLE);
            node.children = node.children.into_iter().map(metadata).collect();
            node
        }
        for name in ["aop-spmi0", "aop-spmi1", "aop-spmi0-p1bu", "aop-spmi1-p1bu"] {
            let path = format!("/arm-io/{name}");
            // NFC descendants are unavailable too; ordinary admission
            // removes their compatibles before the unused-bus pass.
            let bus = tree.node(&path).unwrap();
            assert!(bus.property(COMPATIBLE).is_none());
            assert_eq!(
                metadata(bus.clone()),
                metadata(original.node(&path).unwrap().clone())
            );
            assert!(hidden.contains(&path));
        }
        for name in ["nub-spmi0", "nub-spmi1"] {
            assert!(
                tree.property(&format!("/arm-io/{name}"), COMPATIBLE)
                    .is_some()
            );
        }
    }
}

#[test]
fn role_claim_keeps_only_its_nub_and_referenced_provider() {
    let mut ans = node(
        "ans",
        Some(board::ASC.as_bytes()),
        vec![node("iop-ans-nub", Some(board::RTBUDDY.as_bytes()), vec![])],
    );
    ans.set_property("role", b"ANS2\0".to_vec()).unwrap();
    ans.set_property(IOMMU_PARENT, 142u32.to_le_bytes().to_vec())
        .unwrap();
    let mut sart = node("sart-ans", Some(board::SART.as_bytes()), vec![]);
    sart.set_property(PHANDLE, 142u32.to_le_bytes().to_vec())
        .unwrap();
    let other = node(
        "other",
        Some(board::ASC.as_bytes()),
        vec![node("nub", Some(board::RTBUDDY.as_bytes()), vec![])],
    );
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    ans,
                    sart,
                    other,
                    node("unused-sart", Some(board::SART.as_bytes()), vec![]),
                ],
            )],
        ),
    };
    assert_eq!(
        hide_unmodeled(&mut tree),
        ["/arm-io/other", "/arm-io/other/nub", "/arm-io/unused-sart"]
    );
    assert!(
        tree.property("/arm-io/ans/iop-ans-nub", COMPATIBLE)
            .is_some()
    );
    assert!(tree.property("/arm-io/sart-ans", COMPATIBLE).is_some());
    for key in [
        "pre-loaded",
        "running",
        "no-firmware-service",
        "segment-ranges",
        "segment-names",
    ] {
        assert!(
            tree.property("/arm-io/ans/iop-ans-nub", key)
                .unwrap()
                .value
                .is_empty()
        );
        assert!(tree.property("/arm-io/other/nub", key).is_none());
    }
    // The claimed ANS2 node carries its own boot-chain facts: the empty
    // register tuning lists its driver requires.
    for key in [
        "afc-aiu-ans-dual-tunables",
        "command-accelerator-tunables",
        "storage-lane-common-tunables",
        "ccu-security-interrupt-tunables",
    ] {
        assert!(tree.property("/arm-io/ans", key).unwrap().value.is_empty());
        assert!(tree.property("/arm-io/other", key).is_none());
    }
    tree.node_mut("/arm-io/ans")
        .unwrap()
        .set_property("role", b"ANS3\0".to_vec())
        .unwrap();
    assert_eq!(
        hide_unmodeled(&mut tree),
        ["/arm-io/ans", "/arm-io/ans/iop-ans-nub", "/arm-io/sart-ans"]
    );
}

/// J473AP's SMC subtree: the claimed wrapper, its native nub, the nub's
/// `smc-pmu` child and `/buttons`, whose hold button `smc-pmu` provides.
#[test]
fn the_smc_claim_keeps_its_pmu_and_the_buttons_it_provides() {
    let smc = |role: &[u8]| {
        let mut pmu = node("smc-pmu", Some(b"smc-pmu\0"), vec![]);
        pmu.set_property(PHANDLE, 0x91u32.to_le_bytes().to_vec())
            .unwrap();
        let mut smc = node(
            "smc",
            Some(board::ASC.as_bytes()),
            vec![node(
                "iop-smc-nub",
                Some(board::RTBUDDY.as_bytes()),
                vec![pmu, node("smc-socd", Some(b"smc-socd\0"), vec![])],
            )],
        );
        smc.set_property("role", role.to_vec()).unwrap();
        smc
    };
    let buttons = |provider: u32| {
        let mut buttons = node("buttons", Some(b"buttons\0"), vec![]);
        let hold = [provider, four_char_code(b"btnR"), four_char_code(b"bHLD")];
        buttons
            .set_property(
                "function-button_hold",
                hold.iter().flat_map(|cell| cell.to_le_bytes()).collect(),
            )
            .unwrap();
        buttons
    };
    let tree = |role: &[u8], provider: u32| DeviceTree {
        root: node(
            "",
            None,
            vec![node("arm-io", None, vec![smc(role)]), buttons(provider)],
        ),
    };
    let mut modeled = tree(b"SMC\0", 0x91);
    // Only the unmodeled grandchild goes.
    assert_eq!(
        hide_unmodeled(&mut modeled),
        ["/arm-io/smc/iop-smc-nub/smc-socd"]
    );
    for kept in [
        "/arm-io/smc",
        "/arm-io/smc/iop-smc-nub",
        "/arm-io/smc/iop-smc-nub/smc-pmu",
        "/buttons",
    ] {
        assert!(modeled.property(kept, COMPATIBLE).is_some(), "{kept}");
    }
    // The PMU child carries no RTKit hand-off facts of its own.
    assert!(
        modeled
            .property("/arm-io/smc/iop-smc-nub/smc-pmu", "pre-loaded")
            .is_none()
    );
    // Without the SMC claim the PMU goes, and so do the buttons it
    // provides; buttons naming a missing provider go too.
    let mut unclaimed = tree(b"PMU\0", 0x91);
    let hidden = hide_unmodeled(&mut unclaimed);
    assert_eq!(hidden.last().map(String::as_str), Some("/buttons"));
    assert!(
        hidden
            .iter()
            .any(|path| path == "/arm-io/smc/iop-smc-nub/smc-pmu")
    );
    let mut orphaned = tree(b"SMC\0", 0x42);
    assert!(hide_unmodeled(&mut orphaned).contains(&"/buttons".into()));
}

fn four_char_code(name: &[u8; 4]) -> u32 {
    u32::from_be_bytes(*name)
}

#[test]
fn a_visible_gpu_gets_the_handed_off_ttbat_published() {
    let gpu = |base: u64| {
        let mut sgx = node("sgx", Some(b"gpu,t8112".as_slice()), vec![]);
        sgx.set_property(GPU_REGION_BASE, base.to_le_bytes().to_vec())
            .unwrap();
        DeviceTree {
            root: node("", None, vec![node("arm-io", None, vec![sgx])]),
        }
    };
    let mut tree = gpu(0x8_0004_4000);
    let mut notes = Vec::new();
    publish_gpu_ttbat(&mut tree, &mut notes);
    assert_eq!(notes, ["/arm-io/sgx +gptbat-ready"]);
    assert!(
        tree.property("/arm-io/sgx", GPTBAT_READY)
            .unwrap()
            .value
            .is_empty()
    );
    assert_eq!(
        tree.property("/arm-io/sgx", TTBAT_BASE).unwrap().value,
        0x20_0011u32.to_le_bytes()
    );
    // Published once; a misaligned base gets nothing.
    notes.clear();
    publish_gpu_ttbat(&mut tree, &mut notes);
    assert!(notes.is_empty());
    let mut misaligned = gpu(0x8_0004_4100);
    publish_gpu_ttbat(&mut misaligned, &mut notes);
    assert!(misaligned.property("/arm-io/sgx", GPTBAT_READY).is_none());
    // Until the GPU is admitted the loader hides it and publishes nothing.
    let mut hidden = gpu(0x8_0004_4000);
    hide_unmodeled(&mut hidden);
    assert_eq!(
        hidden.property("/arm-io/sgx", GPTBAT_READY).is_some(),
        board::GPU_ADMITTED
    );
}

#[test]
fn a_real_gpu_hides_the_agx_model_and_nothing_else() {
    let mut asc = node(
        "gfx-asc",
        Some(b"iop,ascwrap-v4\0"),
        vec![node("iop-gfx-nub", Some(b"iop-nub,rtbuddy-v2\0"), vec![])],
    );
    asc.set_property("role", b"GFX\0".to_vec()).unwrap();
    let mut smc = node("smc", Some(b"iop,ascwrap-v4\0"), vec![]);
    smc.set_property("role", b"SMC\0".to_vec()).unwrap();
    let mut sgx = node("sgx", Some(b"gpu,t8112\0"), vec![]);
    sgx.set_property("gfx-shared-region-base", vec![1]).unwrap();
    let mapper = node("mapper-gfx-asc", Some(board::GFX_MAPPER.as_bytes()), vec![]);
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node("arm-io", None, vec![asc, smc, sgx, mapper])],
        ),
    };
    assert_eq!(
        hide_agx(&mut tree),
        [
            "/arm-io/gfx-asc",
            "/arm-io/gfx-asc/iop-gfx-nub",
            "/arm-io/sgx",
            "/arm-io/mapper-gfx-asc",
        ]
    );
    assert!(tree.property("/arm-io/sgx", COMPATIBLE).is_none());
    assert!(
        tree.property("/arm-io/sgx", "gfx-shared-region-base")
            .is_some()
    );
    assert!(tree.property("/arm-io/smc", COMPATIBLE).is_some());
}

#[test]
fn spi_requires_a_nor_slave_and_keeps_its_gpio_provider() {
    let mut gpio = node("gpio", Some(board::GPIO.as_bytes()), vec![]);
    gpio.set_property(PHANDLE, 106u32.to_le_bytes().to_vec())
        .unwrap();
    let mut spi = node(
        "spi1",
        Some(board::SPI.as_bytes()),
        vec![node(
            "spinor",
            Some(board::NOR.as_bytes()),
            vec![
                node("firmware", Some(board::BOOT_FIRMWARE.as_bytes()), vec![]),
                node("anvram", Some(board::NVRAM.as_bytes()), vec![]),
            ],
        )],
    );
    spi.set_property(
        "function-spi_cs0",
        [106u32, board::GPIO_FUNCTION, 49, 0x10001]
            .into_iter()
            .flat_map(u32::to_le_bytes)
            .collect(),
    )
    .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node(
                "arm-io",
                None,
                vec![gpio, spi, node("spi0", Some(board::SPI.as_bytes()), vec![])],
            )],
        ),
    };
    assert_eq!(hide_unmodeled(&mut tree), ["/arm-io/spi0"]);
    assert!(tree.property("/arm-io/gpio", COMPATIBLE).is_some());
    assert!(
        tree.property("/arm-io/spi1/spinor/firmware", COMPATIBLE)
            .is_some()
    );
    tree.node_mut("/arm-io/spi1").unwrap().children.clear();
    assert_eq!(hide_unmodeled(&mut tree), ["/arm-io/gpio", "/arm-io/spi1"]);
}

#[test]
fn a_visible_client_keeps_the_gpio_bank_its_interrupt_names() {
    let mut gpio = node("gpio", Some(board::GPIO.as_bytes()), vec![]);
    gpio.set_property(PHANDLE, 106u32.to_le_bytes().to_vec())
        .unwrap();
    let mut manager = node("hpmBusManager", Some(b"usbc,manager\0"), vec![]);
    manager
        .set_property("interrupt-parent", 106u32.to_le_bytes().to_vec())
        .unwrap();
    let i2c = node("i2c0", Some(b"i2c,t8101\0"), vec![manager]);
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node("arm-io", None, vec![gpio, i2c])],
        ),
    };
    assert_eq!(hide_unmodeled(&mut tree), Vec::<String>::new());
    tree.node_mut("/arm-io/i2c0").unwrap().children.clear();
    assert_eq!(hide_unmodeled(&mut tree), ["/arm-io/gpio"]);
}

#[test]
fn keeps_only_modeled_devices() {
    let mut tree = DeviceTree {
        root: secure_root(node(
            "device-tree",
            Some(b"J473AP\0AppleARM\0"),
            vec![
                node(
                    "cpus",
                    None,
                    vec![node("cpu0", Some(b"apple,blizzard\0ARM,v8\0"), vec![])],
                ),
                node(
                    "arm-io",
                    Some(b"arm-io,t8110\0"),
                    vec![
                        node("uart0", Some(b"uart-1,samsung\0"), vec![]),
                        node("wdt", Some(b"wdt,t8101\0wdt,s5l8960x\0"), vec![]),
                        node("aic", Some(b"aic,2\0"), vec![]),
                        {
                            let mut sep = node(
                                "sep",
                                Some(b"iop-sep,ascwrap-v4\0"),
                                vec![node("iop-sep-nub", Some(b"iop-nub,sep\0"), vec![])],
                            );
                            sep.set_property("role", b"SEP\0".to_vec()).unwrap();
                            sep.set_property(IOMMU_PARENT, 0x84u32.to_le_bytes().to_vec())
                                .unwrap();
                            sep
                        },
                        node(
                            "dart-sep",
                            Some(b"dart,t8110\0"),
                            vec![phandle(
                                node("mapper-sep", Some(b"iommu-mapper\0"), vec![]),
                                0x84,
                            )],
                        ),
                        aes(b"aes,s5l8960x\0"),
                        phandle(
                            node("dockchannel-uart", Some(b"aapl,dock-channels\0"), vec![]),
                            0x4d,
                        ),
                    ],
                ),
                node("buttons", Some(b"buttons\0"), vec![]),
                serial_default(0x4d),
            ],
        )),
    };
    let hidden = hide_unmodeled(&mut tree);
    assert_eq!(
        hidden,
        [
            "/arm-io/aes",
            "/arm-io/dockchannel-uart",
            "/buttons",
            "/defaults serial-device",
            "/ secure-root-prefix",
            "/arm-io/wdt +no-pmu",
        ]
    );
    for kept in [
        "/",
        "/cpus/cpu0",
        "/arm-io",
        "/arm-io/uart0",
        "/arm-io/wdt",
        "/arm-io/aic",
        // The SEP (role = SEP), its native nub and its DART are admitted.
        "/arm-io/sep",
        "/arm-io/sep/iop-sep-nub",
        "/arm-io/dart-sep",
        "/arm-io/dart-sep/mapper-sep",
    ] {
        assert!(tree.property(kept, COMPATIBLE).is_some(), "{kept}");
    }
    assert!(tree.property(DEFAULTS, SERIAL_DEVICE).is_none());
    assert!(tree.root.property(SECURE_ROOT_PREFIX).is_none());
}

fn secure_root(mut root: Node) -> Node {
    root.set_property(SECURE_ROOT_PREFIX, b"md\0".to_vec())
        .unwrap();
    root
}

#[test]
fn only_an_aes_node_with_its_compatible_is_a_visible_accelerator() {
    let mut root = node(
        "device-tree",
        None,
        vec![node("arm-io", None, vec![aes(b"aes,s8000\0")])],
    );
    assert!(visible_aes(&root));
    root.children[0].children[0].remove_property(COMPATIBLE);
    assert!(!visible_aes(&root));
    let other = node(
        "device-tree",
        None,
        vec![node("sio", Some(b"iop,ascwrap-v4\0"), vec![])],
    );
    assert!(!visible_aes(&other));
}

fn aes(compatible: &[u8]) -> Node {
    let mut aes = node("aes", Some(compatible), vec![]);
    aes.set_property(DEVICE_TYPE, b"aes\0".to_vec()).unwrap();
    aes
}

fn phandle(mut node: Node, value: u32) -> Node {
    node.set_property(PHANDLE, value.to_le_bytes().to_vec())
        .unwrap();
    node
}

fn serial_default(value: u32) -> Node {
    let mut defaults = node("defaults", None, vec![]);
    defaults
        .set_property(SERIAL_DEVICE, value.to_le_bytes().to_vec())
        .unwrap();
    defaults
}

fn mapper(name: &str, phandle_value: u32) -> Node {
    phandle(node(name, Some(b"iommu-mapper\0"), vec![]), phandle_value)
}

/// J473AP's AES engine reaches RAM through `dart-sio` `mapper-aes`
/// (phandle 0x8a): that DART and mapper stay, its other mappers and the
/// DARTs no visible device names go.
#[test]
fn darts_stay_only_for_the_streams_of_visible_devices() {
    let mut engine = aes(b"aes,s8000\0");
    engine
        .set_property(IOMMU_PARENT, 0x8a_u32.to_le_bytes().to_vec())
        .unwrap();
    let mut sio = node("sio", Some(b"iop,ascwrap-v4\0"), vec![]);
    sio.set_property(IOMMU_PARENT, 0x89_u32.to_le_bytes().to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: secure_root(node(
            "device-tree",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    node(
                        "dart-sio",
                        Some(b"dart,t8110\0"),
                        vec![mapper("mapper-sio", 0x89), mapper("mapper-aes", 0x8a)],
                    ),
                    node(
                        "dart-sep",
                        Some(b"dart,t8110\0"),
                        vec![mapper("mapper-sep", 0x90)],
                    ),
                    engine,
                    sio,
                ],
            )],
        )),
    };
    assert_eq!(
        hide_unmodeled(&mut tree),
        [
            "/arm-io/dart-sio/mapper-sio",
            "/arm-io/dart-sep",
            "/arm-io/dart-sep/mapper-sep",
            "/arm-io/sio",
        ]
    );
    for kept in [
        "/arm-io/dart-sio",
        "/arm-io/dart-sio/mapper-aes",
        "/arm-io/aes",
    ] {
        assert!(tree.property(kept, COMPATIBLE).is_some(), "{kept}");
    }
    // A visible AES engine keeps the secure root prefix.
    assert!(tree.root.property(SECURE_ROOT_PREFIX).is_some());
}

#[test]
#[ignore = "oracle: STELA_TEST_DART_DT"]
fn original_j314s_preparation_keeps_the_complete_external_display_graph() {
    let bytes = std::fs::read(std::env::var("STELA_TEST_DART_DT").unwrap()).unwrap();
    assert_eq!(
        stela_formats::sha256(&bytes),
        "44fa41bfd0fe0027fd862b8615ae956885614636625d44a59cab226d682558ad"
    );
    let mut tree = DeviceTree::parse(&bytes).unwrap();
    hide_unmodeled(&mut tree);
    for path in [
        "/arm-io/dcpext0",
        "/arm-io/dcpext0-expert",
        "/arm-io/dispext0",
        "/arm-io/dart-dcpext0",
        "/arm-io/dart-dcpext0/mapper-dcpext0",
        "/arm-io/dart-dispext0",
        "/arm-io/dart-dispext0/mapper-dispext0",
    ] {
        assert!(tree.property(path, COMPATIBLE).is_some(), "{path}");
    }
    let surface = tree.node("/arm-io/dart-dispext0").unwrap();
    assert_eq!(surface.property("instance").unwrap().value.len(), 24);
    assert!(surface.property("never-lock").unwrap().value.is_empty());
    assert!(
        surface
            .property("ignore-inconsistent-hw-locks")
            .unwrap()
            .value
            .is_empty()
    );
    assert!(tree.property("/arm-io/dcpext1", COMPATIBLE).is_none());
    assert!(tree.property("/arm-io/usb-drd0", COMPATIBLE).is_none());
}

#[test]
fn native_legacy_dart_preparation_preserves_checked_mapper_dependencies_only() {
    for compatible in [b"dart,t8020\0".as_slice(), b"dart,t6000\0"] {
        let mut engine = aes(b"aes,s8000\0");
        engine
            .set_property(IOMMU_PARENT, 0x8au32.to_le_bytes().to_vec())
            .unwrap();
        let mut stream = mapper("mapper-aes", 0x8a);
        stream
            .set_property("reg", 0u32.to_le_bytes().to_vec())
            .unwrap();
        let mut dart = node("dart-sio", Some(compatible), vec![stream]);
        for (key, bytes) in [
            ("instance", b"TRADDART\0\0\0\0".to_vec()),
            (
                "reg",
                [0x235004000u64, 0x4000]
                    .into_iter()
                    .flat_map(u64::to_le_bytes)
                    .collect(),
            ),
            ("page-size", 0x4000u32.to_le_bytes().to_vec()),
            ("sids", 1u32.to_le_bytes().to_vec()),
            ("vm-base", 0u32.to_le_bytes().to_vec()),
        ] {
            dart.set_property(key, bytes).unwrap();
        }
        let original = DeviceTree {
            root: secure_root(node(
                "device-tree",
                None,
                vec![node("arm-io", None, vec![dart, engine])],
            )),
        };
        let mut prepared = original.clone();
        hide_unmodeled(&mut prepared);
        for path in [
            "/arm-io/dart-sio",
            "/arm-io/dart-sio/mapper-aes",
            "/arm-io/aes",
        ] {
            assert!(prepared.property(path, COMPATIBLE).is_some(), "{path}");
        }
        for key in ["instance", "reg", "sids", "vm-base"] {
            assert_eq!(
                prepared.property("/arm-io/dart-sio", key),
                original.property("/arm-io/dart-sio", key)
            );
        }
        for unsupported in ["remap", "routing"] {
            let mut prepared = original.clone();
            let dart = prepared.node_mut("/arm-io/dart-sio").unwrap();
            if unsupported == "remap" {
                dart.set_property("remap", vec![0, 1, 0, 0]).unwrap();
            } else {
                dart.set_property(
                    "instance",
                    [b"TRADDART\0\0\0\0".as_slice(), b"TRADDARTBLK\0"].concat(),
                )
                .unwrap();
                dart.set_property(
                    "reg",
                    [0x235004000u64, 0x4000, 0x235008000, 0x4000]
                        .into_iter()
                        .flat_map(u64::to_le_bytes)
                        .collect(),
                )
                .unwrap();
            }
            hide_unmodeled(&mut prepared);
            for path in [
                "/arm-io/dart-sio",
                "/arm-io/dart-sio/mapper-aes",
                "/arm-io/aes",
            ] {
                assert!(
                    prepared.property(path, COMPATIBLE).is_none(),
                    "{unsupported}: {path}"
                );
            }
            assert!(prepared.property("/arm-io/dart-sio", "instance").is_some());
        }
    }
}

/// A legacy family without the required native topology/page contract
/// remains unmodeled, and its DMA clients go with it.
#[test]
fn a_device_behind_an_unmodeled_dart_is_hidden_with_it() {
    let mut engine = aes(b"aes,s8000\0");
    engine
        .set_property(IOMMU_PARENT, 0x8a_u32.to_le_bytes().to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: secure_root(node(
            "device-tree",
            None,
            vec![node(
                "arm-io",
                None,
                vec![
                    node(
                        "dart-sio",
                        Some(b"dart,t8020\0"),
                        vec![mapper("mapper-aes", 0x8a)],
                    ),
                    engine,
                ],
            )],
        )),
    };
    assert_eq!(
        hide_unmodeled(&mut tree),
        [
            "/arm-io/aes",
            "/arm-io/dart-sio",
            "/arm-io/dart-sio/mapper-aes",
            "/ secure-root-prefix",
        ]
    );
}

/// Direct AP memory registers do not master through the separate PMP
/// transport. Model-class admission supplies no native owner or lifecycle
/// permission: mount must still require the joint memory resources.
#[test]
fn direct_memory_admission_does_not_admit_unsupported_pmp_dma() {
    let mut mcc = node("mcc", Some(b"mcc,t6000\0"), vec![]);
    for (key, bytes) in [
        (
            "reg",
            [0x200000000u64, 0x1000000]
                .into_iter()
                .flat_map(u64::to_le_bytes)
                .collect(),
        ),
        ("config-data", vec![0xa5; 200]),
        ("dramcfg-data", 0xffff0000u32.to_le_bytes().to_vec()),
    ] {
        mcc.set_property(key, bytes).unwrap();
    }
    let mut nub = node("iop-pmp-nub", Some(board::RTBUDDY.as_bytes()), vec![]);
    nub.set_property("ptd-range", vec![0x5a; 40]).unwrap();
    let mut pmp = node("pmp", Some(board::ASC.as_bytes()), vec![nub]);
    pmp.set_property("role", b"PMP\0".to_vec()).unwrap();
    pmp.set_property(IOMMU_PARENT, 0xa9u32.to_le_bytes().to_vec())
        .unwrap();
    pmp.set_property("ptd-update-reg-index", 3u32.to_le_bytes().to_vec())
        .unwrap();
    let mut dart = node(
        "dart-pmp",
        Some(b"dart,t6000\0"),
        vec![mapper("mapper-pmp", 0xa9)],
    );
    dart.set_property("instance", b"FPADDART\0\0\0\0".to_vec())
        .unwrap();
    dart.set_property("page-size", 0x4000u32.to_le_bytes().to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node("arm-io", None, vec![mcc.clone(), pmp, dart])],
        ),
    };
    let original = tree.clone();
    assert_eq!(
        hide_unmodeled(&mut tree),
        [
            "/arm-io/pmp",
            "/arm-io/pmp/iop-pmp-nub",
            "/arm-io/dart-pmp",
            "/arm-io/dart-pmp/mapper-pmp"
        ]
    );
    assert_eq!(tree.node("/arm-io/mcc"), Some(&mcc));
    for path in [
        "/arm-io/pmp",
        "/arm-io/pmp/iop-pmp-nub",
        "/arm-io/dart-pmp",
        "/arm-io/dart-pmp/mapper-pmp",
    ] {
        assert!(tree.property(path, COMPATIBLE).is_none(), "{path}");
        let expected: Vec<_> = original
            .node(path)
            .unwrap()
            .properties
            .iter()
            .filter(|property| property.name() != COMPATIBLE)
            .collect();
        let actual: Vec<_> = tree.node(path).unwrap().properties.iter().collect();
        assert_eq!(actual, expected, "{path}");
    }
    // A direct-memory class does not exempt a declared DMA
    // dependency from the ordinary unsupported-provider cascade.
    let mut dependent = original;
    dependent
        .node_mut("/arm-io/mcc")
        .unwrap()
        .set_property(IOMMU_PARENT, 0xa9u32.to_le_bytes().to_vec())
        .unwrap();
    assert!(hide_unmodeled(&mut dependent).contains(&"/arm-io/mcc".into()));
    let mut unknown = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node("memory", Some(b"mcc,unknown\0"), vec![])],
        ),
    };
    assert_eq!(hide_unmodeled(&mut unknown), ["/memory"]);
}

#[test]
#[ignore = "oracle: STELA_T6000_MCC_DEVICE_TREE"]
fn original_direct_memory_preserves_source_while_pmp_dma_stays_hidden() {
    let bytes = std::fs::read(std::env::var("STELA_T6000_MCC_DEVICE_TREE").unwrap()).unwrap();
    assert_eq!(
        stela_formats::sha256(&bytes),
        "44fa41bfd0fe0027fd862b8615ae956885614636625d44a59cab226d682558ad"
    );
    let original = DeviceTree::parse(&bytes).unwrap();
    let mut prepared = original.clone();
    let hidden = hide_unmodeled(&mut prepared);
    assert_eq!(prepared.node("/arm-io/mcc"), original.node("/arm-io/mcc"));
    assert_eq!(
        prepared.property("/arm-io/mcc", COMPATIBLE).unwrap().value,
        b"mcc,t6000\0"
    );
    assert!(!hidden.contains(&"/arm-io/mcc".into()));
    for path in [
        "/arm-io/pmp",
        "/arm-io/pmp/iop-pmp-nub",
        "/arm-io/dart-pmp",
        "/arm-io/dart-pmp/mapper-pmp",
    ] {
        assert_eq!(
            hidden.iter().filter(|entry| entry.as_str() == path).count(),
            1,
            "{path}"
        );
        assert!(prepared.property(path, COMPATIBLE).is_none(), "{path}");
        let expected: Vec<_> = original
            .node(path)
            .unwrap()
            .properties
            .iter()
            .filter(|property| property.name() != COMPATIBLE)
            .collect();
        let actual: Vec<_> = prepared.node(path).unwrap().properties.iter().collect();
        assert_eq!(actual, expected, "{path}");
    }
}

#[test]
fn t6020_mcc_retains_the_compatible_needed_by_the_pmgr_function_provider() {
    let mcc = phandle(node("mcc", Some(b"mcc,t6020\0"), vec![]), 0xa3);
    let mut pmgr = node("pmgr", Some(b"pmgr,t6020\0"), vec![]);
    pmgr.set_property("function-mcc_ctrl", b"\xa3\0\0\0$meM".to_vec())
        .unwrap();
    let mut tree = DeviceTree {
        root: node(
            "",
            None,
            vec![node("arm-io", None, vec![mcc.clone(), pmgr])],
        ),
    };

    let hidden = hide_unmodeled(&mut tree);
    assert!(!hidden.iter().any(|path| path == "/arm-io/mcc"));
    assert_eq!(tree.node("/arm-io/mcc"), Some(&mcc));
}

#[test]
fn a_modeled_serial_device_stays_selected() {
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![
                node(
                    "arm-io",
                    None,
                    vec![phandle(node("uart0", Some(b"uart-1,samsung\0"), vec![]), 7)],
                ),
                serial_default(7),
            ],
        ),
    };
    assert!(hide_unmodeled(&mut tree).is_empty());
    assert!(tree.property(DEFAULTS, SERIAL_DEVICE).is_some());
}

#[test]
fn a_hidden_neural_engine_withdraws_its_capability_and_keeps_its_links() {
    let mut ane = phandle(node("ane", Some(b"ane,t8020\0"), vec![]), 0xe6);
    for (key, value) in [
        ("ane-type", 0x80u32),
        ("iommu-parent", 0xe8),
        ("clock-gates", 0x185),
        ("power-gates", 0x185),
    ] {
        ane.set_property(key, value.to_le_bytes().to_vec()).unwrap();
    }
    let ane = typed(ane, b"ane\0");
    // A device the machine models keeps the property.
    let mut uart = node("uart0", Some(board::UART.as_bytes()), vec![]);
    uart.set_property("ane-type", vec![1, 0, 0, 0]).unwrap();
    let mut tree = DeviceTree {
        root: node(
            "device-tree",
            None,
            vec![node("arm-io", None, vec![ane, uart])],
        ),
    };
    let hidden = hide_unmodeled(&mut tree);
    assert_eq!(hidden, ["/arm-io/ane", "/arm-io/ane ane-type"]);
    assert!(tree.property("/arm-io/ane", "ane-type").is_none());
    assert!(tree.property("/arm-io/ane", COMPATIBLE).is_none());
    for key in [
        "iommu-parent",
        "clock-gates",
        "power-gates",
        PHANDLE,
        DEVICE_TYPE,
    ] {
        assert!(tree.property("/arm-io/ane", key).is_some(), "{key}");
    }
    assert!(tree.property("/arm-io/uart0", "ane-type").is_some());
    // Once withdrawn, a second preparation has nothing to note.
    assert!(hide_unmodeled(&mut tree).is_empty());
}
