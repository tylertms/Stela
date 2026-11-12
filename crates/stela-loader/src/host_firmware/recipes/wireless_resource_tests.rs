//! Provisioning selectors preserve synthetic test observations and exact pins.
use super::*;
use alloc::format;

fn request(driver: &str, name: &str, hardware: &str, revision: &str) -> Request {
    Request {
        name: name.into(),
        target: super::super::requirements::Identity {
            driver: driver.into(),
            hardware: hardware.into(),
            revision: revision.into(),
            board: None,
        },
        minimum_bytes: 1,
    }
}

fn select(requests: &[Request]) -> Result<Plan, Error> {
    let mut inventory = Inventory::default();
    for request in requests {
        inventory.observe(request.view())?;
    }
    Plan::select(&inventory)
}

#[test]
fn rtl8822c_normal_and_wow_keep_each_exact_observation_and_share_the_notice() {
    for device in ["c822", "c82f"] {
        let hardware = format!("pci:10ec:{device}:subsys:1a3b:3750:rev:01");
        let requests = [
            request(
                "realtek-wifi",
                "rtw88/rtw8822c_fw.bin",
                &hardware,
                "syscfg1-00003000:cut-03",
            ),
            request(
                "realtek-wifi",
                "rtw88/rtw8822c_wow_fw.bin",
                &hardware,
                "syscfg1-00003000:cut-03",
            ),
        ];
        let plan = select(&requests).unwrap();
        assert_eq!(plan.selections().len(), 2);
        for (selected, request) in plan.selections().iter().zip(requests.iter()) {
            assert_eq!(&selected.request, request);
            assert_eq!(selected.recipe.image.member, request.name);
            assert_eq!(selected.recipe.image.revision, WIFI_FIRMWARE_REVISION);
        }
        assert_eq!(plan.artifacts().len(), 3);
        assert_eq!(plan.selections()[1].recipe.image.length, 145224);
    }
}

#[test]
fn rtl8852b_uses_its_original_cut_field_without_rtw88_test_chip_policy() {
    for device in ["b852", "b85b"] {
        for (raw, cut) in [
            (0x00000000u32, 0),
            (0x00001000, 1),
            (0x00802000, 2),
            (0x0000f000, 15),
        ] {
            let hardware = format!("pci:10ec:{device}:subsys:17aa:1234:rev:02");
            let revision = format!("syscfg1-{raw:08x}:cut-{cut:02x}");
            let request = request(
                "realtek-wifi",
                "rtw89/rtw8852b_fw-2.bin",
                &hardware,
                &revision,
            );
            let plan = select(core::slice::from_ref(&request)).unwrap();
            assert_eq!(plan.selections()[0].request, request);
            assert_eq!(plan.selections()[0].recipe.image.length, 1554594);
            assert_eq!(plan.selections()[0].recipe.license, &REALTEK_WIFI_LICENSE);
            assert_eq!(plan.artifacts().len(), 2);
        }
    }
}

#[test]
fn realtek_resources_do_not_cross_chip_names_versions_or_inconsistent_observations() {
    let valid = request(
        "realtek-wifi",
        "rtw89/rtw8852b_fw-2.bin",
        "pci:10ec:b852:subsys:17aa:1234:rev:02",
        "syscfg1-00001000:cut-01",
    );
    for (field, value) in [
        ("name", "rtw89/rtw8852b_fw-1.bin"),
        ("name", "rtw89/rtw8852b_fw.bin"),
        ("name", "rtw88/rtw8822c_fw.bin"),
        ("name", "rtw88/rtw8822c_wow_fw.bin"),
        ("hardware", "pci:10ec:c822:subsys:17aa:1234:rev:02"),
        ("hardware", "pci:8086:b852:subsys:17aa:1234:rev:02"),
        ("hardware", "pci:10ec:b85c:subsys:17aa:1234:rev:02"),
        ("hardware", "pci:10ec:b852:subsys:17aa:1234:rev:zz"),
        ("revision", "syscfg1-00001000:cut-02"),
        ("revision", "syscfg1-ffffffff:cut-0f"),
        ("revision", "observed"),
        ("driver", "wifi-intel"),
        ("board", "unbound-board"),
    ] {
        let mut changed = valid.clone();
        match field {
            "name" => changed.name = value.into(),
            "hardware" => changed.target.hardware = value.into(),
            "revision" => changed.target.revision = value.into(),
            "driver" => changed.target.driver = value.into(),
            "board" => changed.target.board = Some(value.into()),
            _ => unreachable!(),
        }
        assert!(select(&[changed]).is_err(), "{field}: {value}");
    }
    let test_silicon = request(
        "realtek-wifi",
        "rtw88/rtw8822c_wow_fw.bin",
        "pci:10ec:c822:subsys:17aa:1234:rev:02",
        "syscfg1-00801000:cut-01",
    );
    assert!(select(&[test_silicon]).is_err());
}

#[test]
fn signed_regdb_resources_keep_the_observed_global_target_and_immutable_commit() {
    let requests = [
        request("wifi-regulatory", "regulatory.db", "wireless-regdb", "v20"),
        request(
            "wifi-regulatory",
            "regulatory.db.p7s",
            "wireless-regdb",
            "v20",
        ),
    ];
    let plan = select(&requests).unwrap();
    assert_eq!(plan.selections().len(), 2);
    assert_eq!(plan.artifacts().len(), 3);
    for (selected, request) in plan.selections().iter().zip(requests.iter()) {
        assert_eq!(&selected.request, request);
        assert_eq!(selected.recipe.image.revision, WIRELESS_REGDB_REVISION);
        assert_eq!(selected.recipe.license.revision, WIRELESS_REGDB_REVISION);
    }
    assert_eq!(plan.selections()[1].recipe.image.length, 1085);
    // Existing database-only observations are not rewritten as invented requests.
    assert_eq!(select(&requests[..1]).unwrap().selections().len(), 1);
    for (driver, hardware, revision) in [
        ("wifi-intel", "wireless-regdb", "v20"),
        ("wifi-regulatory", "pci:10ec:b852", "v20"),
        ("wifi-regulatory", "wireless-regdb", "v19"),
    ] {
        let changed = request(driver, "regulatory.db.p7s", hardware, revision);
        assert!(select(&[changed]).is_err());
    }
}

#[cfg(feature = "std")]
#[test]
#[ignore = "oracle: STELA_WIRELESS_PROVISIONING_EVIDENCE"]
fn approved_downloads_match_recipe_and_license_content_pins() {
    let root =
        std::path::PathBuf::from(std::env::var("STELA_WIRELESS_PROVISIONING_EVIDENCE").unwrap());
    for (recipe, directory) in [
        (&REALTEK_WIFI[1], "linux-firmware"),
        (&REALTEK_WIFI[2], "linux-firmware"),
        (&REGULATORY[1], "wireless-regdb"),
    ] {
        let image = std::fs::read(root.join(directory).join(recipe.image.member)).unwrap();
        let license = std::fs::read(root.join(directory).join(recipe.license.member)).unwrap();
        recipe.image.verify(&image).unwrap();
        recipe.license.verify(&license).unwrap();
        let mut altered = image;
        altered[0] ^= 1;
        assert!(recipe.image.verify(&altered).is_err());
    }
}

#[test]
fn a_wifi_network_requires_the_database_and_a_mediatek_chips_two_images() {
    for (vendor, device, chip) in [
        (0x14c3, 0x7961, "7961"),
        (0x14c3, 0x0608, "7961"),
        (0x14c3, 0x7922, "7922"),
        (0x14c3, 0x0616, "7922"),
        // ITTIM's MT7922 takes MediaTek's MT7922 images.
        (0x0b48, 0x7922, "7922"),
        (0x14c3, 0x7925, "7925"),
        (0x14c3, 0x0717, "7925"),
    ] {
        let mut inventory = Inventory::default();
        require_regulatory(&mut inventory).unwrap();
        assert!(require_wifi(&mut inventory, vendor, device).unwrap());
        let plan = Plan::select(&inventory).unwrap();
        assert_eq!(plan.selections().len(), 3, "{device:#x}");
        for selection in &plan.selections()[1..] {
            assert_eq!(selection.request.target.driver, "wifi-mt792x");
            assert_eq!(
                selection.request.target.hardware,
                format!("pci:14c3:{chip}")
            );
            assert_eq!(selection.request.target.revision, "linux-firmware");
            assert_eq!(selection.recipe.image.member, selection.request.name);
        }
    }
    // Ethernet controllers, Wi-Fi chips no driver here runs, and RTL8852B,
    // whose driver loads no image, have none.
    let mut inventory = Inventory::default();
    for (vendor, device) in [
        (0x8086, 0x125c),
        (0x8086, 0x15f3),
        (0x10ec, 0x8168),
        (0x14c3, 0x7663),
        (0x8086, 0x2526),
        (0x8086, 0x24fd),
        (0x10ec, 0xb852),
        (0x10ec, 0xb822),
        (0x17cb, 0x1103),
    ] {
        assert!(!require_wifi(&mut inventory, vendor, device).unwrap());
    }
    assert!(inventory.requests().is_empty());
}

#[test]
fn a_wifi_network_requires_an_rtl8822cs_image_under_the_chips_binding() {
    for device in [0xc822, 0xc82f] {
        let mut inventory = Inventory::default();
        assert!(require_wifi(&mut inventory, 0x10ec, device).unwrap());
        let plan = Plan::select(&inventory).unwrap();
        assert_eq!(plan.selections().len(), 1);
        let selection = &plan.selections()[0];
        assert_eq!(selection.request.name, "rtw88/rtw8822c_fw.bin");
        assert_eq!(selection.request.target.driver, "realtek-wifi");
        assert_eq!(selection.request.target.hardware, "pci:10ec:c822");
        assert_eq!(selection.request.target.revision, "linux-firmware");
        assert_eq!(selection.recipe.image.member, "rtw88/rtw8822c_fw.bin");
    }
    // The chip binding holds the RTL8822C's images alone.
    for (name, hardware, revision) in [
        ("rtw89/rtw8852b_fw-2.bin", "pci:10ec:c822", "linux-firmware"),
        ("rtw88/rtw8822c_fw.bin", "pci:10ec:c822", "observed"),
        ("rtw88/rtw8822c_fw.bin", "pci:10ec:b852", "linux-firmware"),
    ] {
        assert!(select(&[request("realtek-wifi", name, hardware, revision)]).is_err());
    }
}

#[test]
fn a_wifi_network_requires_every_image_an_intel_device_can_select() {
    for (device, names) in [
        (0x2723, &["iwlwifi-cc-a0-77.ucode"][..]),
        (
            0x2725,
            &["iwlwifi-ty-a0-gf-a0-89.ucode", "iwlwifi-ty-a0-gf-a0.pnvm"][..],
        ),
        (
            0x272b,
            &["iwlwifi-gl-c0-fm-c0-c106.ucode", "iwlwifi-gl-c0-fm-c0.pnvm"][..],
        ),
        (
            0xa0f0,
            &[
                "iwlwifi-Qu-b0-hr-b0-77.ucode",
                "iwlwifi-Qu-c0-hr-b0-77.ucode",
                "iwlwifi-QuZ-a0-hr-b0-77.ucode",
            ][..],
        ),
        (
            0x51f0,
            &[
                "iwlwifi-so-a0-hr-b0-89.ucode",
                "iwlwifi-so-a0-gf-a0-89.ucode",
                "iwlwifi-so-a0-gf-a0.pnvm",
                "iwlwifi-ma-b0-gf-a0-89.ucode",
                "iwlwifi-ma-b0-gf-a0.pnvm",
            ][..],
        ),
        (
            0x7740,
            &["iwlwifi-bz-b0-fm-c0-c106.ucode", "iwlwifi-bz-b0-fm-c0.pnvm"][..],
        ),
        (0xe440, &["iwlwifi-sc-a0-fm-c0-c106.ucode"][..]),
    ] {
        let mut inventory = Inventory::default();
        assert!(require_wifi(&mut inventory, 0x8086, device).unwrap());
        let plan = Plan::select(&inventory).unwrap();
        let selected: Vec<_> = plan
            .selections()
            .iter()
            .map(|selection| {
                assert_eq!(selection.request.target.driver, "wifi-intel");
                assert_eq!(
                    selection.request.target.hardware,
                    format!("pci:8086:{device:04x}")
                );
                assert_eq!(selection.request.target.revision, "linux-firmware");
                assert_eq!(
                    selection.recipe.image.member,
                    format!("intel/iwlwifi/{}", selection.request.name)
                );
                selection.request.name.as_str()
            })
            .collect();
        assert_eq!(selected, names, "{device:04x}");
    }
    // Every device the driver lists has pinned files; each plan selects.
    let mut listed = 0;
    for device in 0..=u16::MAX {
        let mut inventory = Inventory::default();
        if require_wifi(&mut inventory, 0x8086, device).unwrap() {
            listed += 1;
            Plan::select(&inventory).unwrap();
        }
    }
    assert_eq!(listed, 26);
}
