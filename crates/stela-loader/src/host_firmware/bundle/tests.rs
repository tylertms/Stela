use super::*;
use stela_driver_core::firmware::{Provenance, Resource, Target};

#[test]
fn the_complete_bundle_holds_every_recipe_once_by_content() {
    let objects = objects();
    for recipe in recipes() {
        assert!(
            objects
                .iter()
                .any(|object| object.sha256 == recipe.image.sha256),
            "{}",
            recipe.image.member
        );
    }
    let mut digests: Vec<_> = objects.iter().map(|object| object.sha256).collect();
    digests.sort_unstable();
    digests.dedup();
    assert_eq!(digests.len(), objects.len());
    let (count, bytes) = size();
    assert_eq!(count, objects.len());
    // The RTL8168H and RTL8168M share a file, as the RTL9151A and RTL9151AS
    // do, and, with the AMD port, amdgpu's `mec` and `mec2` of Raphael and
    // of Navi 21.
    if Ports::BUILT == Ports::ALL {
        assert_eq!(count + 4, recipes().count());
        // The recipes' recorded lengths: 121 files, 184.9 MiB.
        assert_eq!((count, bytes), (121, 193_887_456), "{count} {bytes}");
    }
    if Ports::BUILT == Ports::NONE {
        assert_eq!(count + 2, recipes().count());
        // Without the AMD port: 88 files, 177.0 MiB.
        assert_eq!((count, bytes), (88, 185_584_904), "{count} {bytes}");
    }
    // The GPU's large images come last, for a volume short of room.
    assert_eq!(objects.last().unwrap().length, 64_116_464);
    let manifest = manifest(&complete()).unwrap();
    let bundle = Bundle::parse(&manifest).unwrap();
    assert_eq!(bundle.objects.len(), count);
    assert!(bundle.missing().is_empty());
    assert_eq!(bundle.notices.len(), notices().len());
    for notice in &bundle.notices {
        assert!(
            notice.member.contains("LICEN") || notice.member == "COPYING",
            "{}",
            notice.member
        );
    }
}

#[test]
fn a_shared_file_is_stored_once_and_a_partial_bundle_names_what_it_lacks() {
    let first = recipes::GSP[0].image;
    let notice = *recipes::GSP[0].license;
    let alias = Artifact {
        member: "nvidia/other/gsp/alias.bin",
        ..first
    };
    // Two names for the same content: one file.
    let mut files: Vec<(&Artifact, &Artifact)> = Vec::new();
    for artifact in [&first, &alias] {
        if !files.iter().any(|(seen, _)| seen.sha256 == artifact.sha256) {
            files.push((artifact, &notice));
        }
    }
    let bytes = manifest(&files).unwrap();
    let bundle = Bundle::parse(&bytes).unwrap();
    assert_eq!(bundle.objects.len(), 1);
    assert_eq!(bundle.missing().len(), size().0 - 1);
    // A duplicate digest, a path off its digest, or a dangling notice is
    // refused.
    let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    let object = value["objects"][0].clone();
    value["objects"].as_array_mut().unwrap().push(object);
    assert!(Bundle::parse(&serde_json::to_vec(&value).unwrap()).is_err());
    let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    value["objects"][0]["path"] = "other.bin".into();
    assert!(Bundle::parse(&serde_json::to_vec(&value).unwrap()).is_err());
    let mut value: serde_json::Value = serde_json::from_slice(&bytes).unwrap();
    value["objects"][0]["license"] = "licenses/none.txt".into();
    assert!(Bundle::parse(&serde_json::to_vec(&value).unwrap()).is_err());
}

fn members(selection: &Selection) -> Vec<&'static str> {
    let mut members: Vec<_> = selection
        .wanted
        .iter()
        .map(|(artifact, _)| artifact.member)
        .collect();
    members.sort_unstable();
    members
}

/// The USB Bluetooth and Ethernet files every boot reads, sorted.
const USB: [&str; 12] = [
    "intel/ibt-20-1-3.ddc",
    "intel/ibt-20-1-3.sfi",
    "mediatek/BT_RAM_CODE_MT7922_1_1_hdr.bin",
    "mediatek/BT_RAM_CODE_MT7961_1_2_hdr.bin",
    "mediatek/mt7925/BT_RAM_CODE_MT7925_1_1_hdr.bin",
    "rtl_nic/rtl8153a-2.fw",
    "rtl_nic/rtl8153a-3.fw",
    "rtl_nic/rtl8153a-4.fw",
    "rtl_nic/rtl8153b-2.fw",
    "rtl_nic/rtl8153c-1.fw",
    "rtl_nic/rtl8156a-2.fw",
    "rtl_nic/rtl8156b-2.fw",
];

fn pci(vendor: u16, device: u16, class: u8) -> Pci {
    Pci {
        segment: 0,
        bus: 0,
        vendor,
        device,
        class,
        bridge: None,
    }
}

/// A PCI port on bus 0 whose secondary bus is `secondary`.
fn port(secondary: u8) -> Pci {
    Pci {
        bridge: Some((secondary, secondary)),
        ..pci(0x8086, 0x7ab8, 0x06)
    }
}

#[test]
fn a_boot_selects_exactly_the_files_its_pci_functions_need() {
    // No device of a supported family: the USB Bluetooth and Ethernet files alone.
    let none = select(&[pci(0x8086, 0x125c, 0x02), pci(0x1b36, 0x000d, 0x0c)]);
    assert_eq!(members(&none), USB);
    assert!(none.unknown.is_empty());
    // An RTX 5080 (GB203) and its audio function, an MT7925 and an RTL8168.
    let pc = select(&[
        pci(0x10de, 0x2c02, 0x03),
        pci(0x10de, 0x22e9, 0x04),
        pci(0x14c3, 0x7925, 0x02),
        pci(0x10ec, 0x8168, 0x02),
    ]);
    let mut expected: Vec<&str> = USB.to_vec();
    expected.extend([
        "mediatek/mt7925/WIFI_MT7925_PATCH_MCU_1_1_hdr.bin",
        "mediatek/mt7925/WIFI_RAM_CODE_MT7925_1_1.bin",
        "nvidia/gb202/gsp/bootloader-570.211.01.bin",
        "nvidia/gb202/gsp/fmc-570.211.01.bin",
        "nvidia/gb202/gsp/gsp-570.211.01.bin",
        "regulatory.db",
        "regulatory.db.p7s",
    ]);
    expected.extend(recipes::REALTEK.iter().map(|recipe| recipe.image.member));
    expected.sort_unstable();
    expected.dedup();
    assert_eq!(members(&pc), expected);
    // A Realtek chip under an alias ID, such as a Killer E2600, reads them too.
    let killer = select(&[pci(0x10ec, 0x2600, 0x02)]);
    assert!(members(&killer).contains(&"rtl_nic/rtl8168h-2.fw"));
    assert_eq!(
        pc.wanted
            .iter()
            .find(|(artifact, _)| artifact.length == 64_116_464)
            .unwrap()
            .1,
        "pci 10de:2c02"
    );
    // About the GSP image and the small files, not the whole bundle.
    assert!(pc.bytes() < 70_000_000, "{}", pc.bytes());
    // An AX210: its image and PNVM and the database.
    let ax210 = select(&[pci(0x8086, 0x2725, 0x02)]);
    let mut expected: Vec<&str> = USB.to_vec();
    expected.extend([
        "intel/iwlwifi/iwlwifi-ty-a0-gf-a0-89.ucode",
        "intel/iwlwifi/iwlwifi-ty-a0-gf-a0.pnvm",
        "regulatory.db",
        "regulatory.db.p7s",
    ]);
    expected.sort_unstable();
    assert_eq!(members(&ax210), expected);
    // With the AMD port: Ryzen 9000 graphics, the files amdgpu asks for on it, its HDMI audio
    // function needs none, and an AMD display chip without pinned files
    // (Navi 10) is named once.
    let ryzen = select_for(
        &[
            pci(0x1002, 0x13c0, 0x03),
            pci(0x1002, 0x1640, 0x04),
            pci(0x1002, 0x731f, 0x03),
        ],
        Ports::ALL,
    );
    // The files amdgpu asks for on a GPU of the PCI device ID `device`.
    let family = |device: u16| {
        let mut inventory = Inventory::default();
        recipes::require_amdgpu(&mut inventory, device).unwrap();
        let mut names: Vec<&str> = inventory
            .requests()
            .iter()
            .map(|request| {
                recipes::AMDGPU
                    .iter()
                    .find(|recipe| recipe.image.member == request.name)
                    .unwrap()
                    .image
                    .member
            })
            .collect();
        names.sort_unstable();
        names
    };
    let raphael = family(0x13c0);
    let mut expected: Vec<&str> = USB.to_vec();
    expected.extend(raphael.iter().copied());
    expected.sort_unstable();
    // `mec2` is `mec`'s bytes, which the bundle holds once.
    let found = members(&ryzen);
    assert!(found.iter().all(|member| expected.contains(member)));
    assert_eq!(found.len(), USB.len() + raphael.len() - 1);
    assert_eq!(ryzen.unknown, ["pci 1002:731f"]);
    // An RX 9070 (Navi 48) and its audio function: amdgpu's files for Navi
    // 48 and nothing for the audio function.
    let navi48 = select_for(
        &[pci(0x1002, 0x7550, 0x03), pci(0x1002, 0xab40, 0x04)],
        Ports::ALL,
    );
    let mut expected: Vec<&str> = USB.to_vec();
    expected.extend(family(0x7550));
    expected.sort_unstable();
    assert_eq!(members(&navi48), expected);
    assert!(navi48.unknown.is_empty());
    // An RX 6800 XT (Navi 21): its files, `mec2` held once as `mec`'s bytes.
    let navi21 = select_for(
        &[pci(0x1002, 0x73bf, 0x03), pci(0x1002, 0xab28, 0x04)],
        Ports::ALL,
    );
    let found = members(&navi21);
    let files = family(0x73bf);
    assert!(
        found
            .iter()
            .all(|member| USB.contains(member) || files.contains(member))
    );
    assert_eq!(found.len(), USB.len() + files.len() - 1);
    assert!(navi21.unknown.is_empty());
    // An NVIDIA display chip no recipe covers is named once.
    let old = select(&[pci(0x10de, 0x13c0, 0x03), pci(0x10de, 0x13c0, 0x03)]);
    assert_eq!(old.unknown, ["pci 10de:13c0"]);
    assert_eq!(members(&old), USB);
}

#[test]
fn the_handoff_manifest_keeps_an_older_per_pc_manifest_beside_bundle_files() {
    let older = serde_json::to_vec(&serde_json::json!({"schema":1,"images":[{
        "name":"rtl_nic/rtl8168e-3.fw", "path":"0.bin",
        "target":{"driver":"realtek","hardware":"10ec:8168","revision":"xid-2c8","board":null},
        "provenance":{"source":"synthetic-fixture","revision":"v1","member":"one.bin","license":"CC0-1.0"},
        "length":3,"sha256":"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"}]}))
    .unwrap();
    let older = super::super::Manifest::parse(&older).unwrap();
    let bytes = manifest(&complete()).unwrap();
    let bundle = Bundle::parse(&bytes).unwrap();
    let selected: Vec<_> = bundle.objects.iter().take(2).collect();
    let handoff = handoff_manifest(Some(&older), &selected).unwrap();
    let parsed = super::super::Manifest::parse(&handoff).unwrap();
    assert_eq!(parsed.images.len(), 3);
    assert_eq!(parsed.images[0].name, "rtl_nic/rtl8168e-3.fw");
    assert_eq!(parsed.images[0].target.revision, "xid-2c8");
    assert_eq!(parsed.images[1].name, name(selected[0].sha256));
    assert_eq!(parsed.images[1].target.driver, TARGET.driver);
    // An older manifest alone still hands off as it was.
    let alone = handoff_manifest(Some(&older), &[]).unwrap();
    assert_eq!(
        super::super::Manifest::parse(&alone).unwrap().images.len(),
        1
    );
}

const ABC: &str = "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad";

static SYNTHETIC: Artifact = Artifact {
    source: "synthetic",
    revision: "v1",
    member: "regulatory.db",
    length: 3,
    sha256: ABC,
};

fn synthetic_pin(request: &Request) -> Option<&'static Artifact> {
    (request.name == "regulatory.db").then_some(&SYNTHETIC)
}

fn stored(name: &'static str, bytes: &[u8]) -> &'static Catalog {
    use stela_crypto::{Sha256, digest::Digest};
    Box::leak(Box::new(
        Catalog::from_handoff(alloc::vec![Resource {
            name,
            target: TARGET,
            provenance: Provenance {
                source: "synthetic",
                revision: "v1",
                member: "regulatory.db",
                license: "licenses/x.txt",
            },
            expected_length: bytes.len(),
            expected_sha256: Sha256::digest(bytes).into(),
            bytes: bytes.to_vec(),
        }])
        .unwrap(),
    ))
}

const REQUEST: Requirement<'static> = Requirement {
    name: "regulatory.db",
    target: Target {
        driver: "wifi-regulatory",
        hardware: "wireless-regdb",
        revision: "v20",
        board: None,
    },
    minimum_bytes: 3,
};

#[test]
fn a_drivers_exact_request_gets_the_bundle_file_its_recipe_pins() {
    let name: &'static str = Box::leak(name(ABC).into_boxed_str());
    let catalog = stored(name, b"abc");
    let provider = BundleProvider::with_pins(catalog, synthetic_pin);
    for _ in 0..2 {
        let image = provider.load(REQUEST).unwrap();
        assert_eq!(image.name(), "regulatory.db");
        assert_eq!(image.target(), REQUEST.target);
        assert_eq!(image.bytes(), b"abc");
        assert_eq!(image.provenance().member, "regulatory.db");
    }
    // Bound once, then found again.
    assert_eq!(provider.bound.borrow().len(), 1);
    // A request longer than the file is refused under its own name.
    let long = Requirement {
        minimum_bytes: 4,
        ..REQUEST
    };
    let error = BundleProvider::with_pins(catalog, synthetic_pin)
        .load(long)
        .unwrap_err();
    assert!(
        matches!(error.kind, ErrorKind::Truncated { .. }),
        "{error:?}"
    );
    assert_eq!(error.name.as_str(), Some("regulatory.db"));
    // A request no pin names stays missing.
    let other = Requirement {
        name: "vendor/unpinned.bin",
        ..REQUEST
    };
    assert_eq!(provider.load(other).unwrap_err().kind, ErrorKind::Missing);
}

#[test]
fn a_stored_file_whose_digest_is_not_the_pin_is_never_bound() {
    // The real pin for the database, and a file stored under its name whose
    // bytes are something else.
    let pin = recipes::REGULATORY[0].image;
    let name: &'static str = Box::leak(name(pin.sha256).into_boxed_str());
    let catalog = stored(name, b"not the database");
    let provider = BundleProvider::new(catalog);
    assert_eq!(
        provider
            .load(Requirement {
                minimum_bytes: 8,
                ..REQUEST
            })
            .unwrap_err()
            .kind,
        ErrorKind::Missing
    );
    assert!(provider.bound.borrow().is_empty());
}

#[test]
fn an_exact_binding_wins_over_the_bundle() {
    use stela_crypto::{Sha256, digest::Digest};
    let exact: &'static Catalog = Box::leak(Box::new(
        Catalog::from_handoff(alloc::vec![Resource {
            name: REQUEST.name,
            target: REQUEST.target,
            provenance: Provenance {
                source: "synthetic",
                revision: "v2",
                member: "regulatory.db",
                license: "CC0-1.0",
            },
            expected_length: 4,
            expected_sha256: Sha256::digest(b"abcd").into(),
            bytes: b"abcd".to_vec(),
        }])
        .unwrap(),
    ));
    let provider = BundleProvider::with_pins(exact, synthetic_pin);
    let image = provider.load(REQUEST).unwrap();
    assert_eq!(image.bytes(), b"abcd");
}

/// The AMD port named in a Cargo.toml line's feature list.
fn ports_of(line: &str) -> Ports {
    Ports {
        amd: line.contains("amdgpu-port\""),
    }
}

/// The line of `manifest` that starts with `start`.
fn line<'a>(manifest: &'a str, start: &str) -> &'a str {
    manifest
        .lines()
        .find(|line| line.starts_with(start))
        .unwrap_or_else(|| panic!("no line {start}"))
}

#[test]
fn the_boot_images_drivers_and_the_bundle_name_the_same_gpu_ports() {
    let boot = include_str!("../../../../stela-boot/Cargo.toml");
    let loader = include_str!("../../../Cargo.toml");
    let cli = include_str!("../../../../stela-cli/Cargo.toml");
    // The default image's drivers, the boot's bundle and a default stick's:
    // no AMD port in any.
    let drivers = ports_of(line(boot, "stela-drivers = "));
    assert_eq!(drivers, Ports::NONE);
    assert_eq!(ports_of(line(boot, "stela-loader = ")), drivers);
    assert_eq!(ports_of(line(loader, "default = ")), drivers);
    // An image with amdgpu reads its firmware, and `stela` built with the
    // port writes it.
    let amdgpu = line(boot, "amdgpu = ");
    assert!(amdgpu.contains("\"stela-drivers/amdgpu-port\""), "{amdgpu}");
    assert!(amdgpu.contains("\"stela-loader/amdgpu-port\""), "{amdgpu}");
    let port = line(cli, "amdgpu-port = ");
    assert!(port.contains("\"stela-loader/amdgpu-port\""), "{port}");
}

#[test]
fn a_build_without_the_amd_port_bundles_none_of_its_firmware() {
    let theirs = |member: &str| member.starts_with("amdgpu/");
    let without_port = objects_for(Ports::NONE);
    assert!(!without_port.is_empty());
    assert!(without_port.iter().all(|artifact| !theirs(artifact.member)));
    assert!(
        families_for(Ports::NONE)
            .flat_map(|(_, recipes)| recipes.iter())
            .all(|recipe| !recipe.license.member.contains("amdgpu"))
    );
    // Its GPUs select nothing; the rest is as with the port.
    let functions = [pci(0x1002, 0x13c0, 3), pci(0x10de, 0x2c02, 3)];
    let without = select_for(&functions, Ports::NONE);
    assert!(
        without
            .wanted
            .iter()
            .all(|(artifact, _)| !theirs(artifact.member))
    );
    assert!(without.unknown.is_empty());
    assert!(
        without
            .wanted
            .iter()
            .any(|(artifact, _)| artifact.member.starts_with("nvidia/"))
    );
    let with = select_for(&functions, Ports::ALL);
    assert!(
        with.wanted
            .iter()
            .any(|(artifact, _)| theirs(artifact.member))
    );
    // This build's bundle holds them exactly when it has the port.
    assert_eq!(
        objects().iter().any(|artifact| theirs(artifact.member)),
        Ports::BUILT.amd
    );
}

#[test]
fn a_pci_function_is_read_from_its_configuration_header() {
    let mut header = [0u8; 64];
    header[..4].copy_from_slice(&[0xde, 0x10, 0xa0, 0x28]);
    header[9..12].copy_from_slice(&[0x00, 0x00, 0x03]);
    let gpu = Pci::from_header(0, 1, &header).unwrap();
    assert_eq!(
        gpu,
        Pci {
            segment: 0,
            bus: 1,
            vendor: 0x10de,
            device: 0x28a0,
            class: 0x03,
            bridge: None,
        }
    );
    // A type 1 header, multi-function bit set, names its buses.
    header[0x0e] = 0x81;
    header[0x18..0x1b].copy_from_slice(&[0, 2, 5]);
    assert_eq!(
        Pci::from_header(0, 0, &header).unwrap().bridge,
        Some((2, 5))
    );
    assert_eq!(Pci::from_header(0, 0, &header[..0x1a]), None);
}

#[test]
fn a_port_with_nothing_below_it_is_where_a_gpu_can_answer_later() {
    let below = |bus: u8| Pci {
        bus,
        ..pci(0x144d, 0xa80a, 0x01)
    };
    let functions = [
        port(1),
        below(1),
        // Nothing on bus 2.
        port(2),
        // No bus number of its own: it forwards nothing yet.
        Pci {
            bridge: Some((0, 0)),
            ..port(0)
        },
        // Bus 3 holds a function only on another segment.
        port(3),
        Pci {
            segment: 1,
            ..below(3)
        },
    ];
    let empty: Vec<_> = empty_ports(&functions)
        .map(|function| function.bridge)
        .collect();
    assert_eq!(empty, [Some((2, 2)), Some((3, 3))]);
}

/// The files of each chip Stela's NVIDIA driver runs, by recipe.
fn every_nvidia_chip_file() -> Vec<&'static str> {
    let mut files = Vec::new();
    let mut chips: Vec<&str> = (0..=u16::MAX).filter_map(recipes::nvidia_chip).collect();
    chips.dedup();
    assert!(chips.contains(&"ad107"));
    for chip in chips.into_iter().filter(|c| !c.starts_with("gp")) {
        let mut inventory = Inventory::default();
        recipes::require_gsp(&mut inventory, chip).unwrap();
        for selected in recipes::Plan::select(&inventory).unwrap().selections() {
            files.push(selected.recipe.image.member);
        }
    }
    files.sort_unstable();
    files.dedup();
    files
}

#[test]
fn a_gpu_the_firmware_left_without_power_gets_every_chips_files() {
    // A hybrid laptop: the integrated GPU answers, and its discrete GPU's
    // port has nothing below it while the GPU is in D3cold.
    let functions = [pci(0x8086, 0x46a6, 0x03), port(1)];
    let mut laptop = select_for(&functions, Ports::NONE);
    assert_eq!(members(&laptop), USB);
    assert_eq!(laptop.late.len(), 1);
    let late = &laptop.late[0];
    assert_eq!(late.family, "NVIDIA GSP-RM");
    assert!(late.why.contains("one of 1 PCI ports"), "{}", late.why);
    // Every NVIDIA file: 26 files, 150.6 MiB.
    assert_eq!((late.files.len(), late.bytes()), (26, 157_944_212));
    let left = laptop.take_late(Room {
        images: super::super::MAX_IMAGES,
        bytes: super::super::MAX_TOTAL_BYTES,
    });
    assert!(left.is_empty());
    assert!(laptop.late.is_empty());
    let wanted = members(&laptop);
    for file in every_nvidia_chip_file() {
        assert!(wanted.contains(&file), "{file}");
    }
    // With the USB files and an Intel Wi-Fi card's it stays well within
    // the handoff's images and bytes.
    assert!(laptop.wanted.len() + 8 < super::super::MAX_IMAGES);
    assert!(laptop.bytes() + 16 * 1024 * 1024 < super::super::MAX_TOTAL_BYTES);
    // With the AMD port, its files too.
    let amd = select_for(&functions, Ports::ALL);
    let families: Vec<_> = amd.late.iter().map(|late| late.family).collect();
    assert_eq!(families, ["AMD GPU", "NVIDIA GSP-RM"]);
    // Without a census nothing is known: every family is late.
    let unseen = select_without_census(Ports::NONE);
    assert_eq!(members(&unseen), USB);
    assert_eq!(unseen.late.len(), 1);
    assert_eq!(unseen.late[0].files.len(), 26);
}

#[test]
fn a_visible_gpu_or_no_empty_port_reads_no_late_files() {
    // A supported NVIDIA GPU answered: its files only, beside an empty port.
    let desktop = select_for(&[pci(0x10de, 0x2c02, 0x03), port(2)], Ports::NONE);
    assert!(desktop.late.is_empty());
    // An AMD GPU with pinned files keeps the AMD family out, not NVIDIA's.
    let amd = select_for(&[pci(0x1002, 0x7550, 0x03), port(2)], Ports::ALL);
    let families: Vec<_> = amd.late.iter().map(|late| late.family).collect();
    assert_eq!(families, ["NVIDIA GSP-RM"]);
    // An NVIDIA chip no recipe covers drives nothing: a second GPU may
    // still answer late.
    let maxwell = select_for(&[pci(0x10de, 0x13c0, 0x03), port(2)], Ports::NONE);
    assert_eq!(maxwell.late.len(), 1);
    // No port with nothing below it: nothing can answer later.
    let below = Pci {
        bus: 2,
        ..pci(0x8086, 0x2725, 0x02)
    };
    let full = select_for(&[pci(0x8086, 0x46a6, 0x03), port(2), below], Ports::NONE);
    assert!(full.late.is_empty());
}

#[test]
fn the_nvidia_notice_claims_no_more_than_the_census_shows() {
    let below = |bus: u8| Pci {
        bus,
        ..pci(0x8086, 0x2725, 0x02)
    };
    let intel = pci(0x8086, 0x46a6, 0x03);
    let maxwell = pci(0x10de, 0x13c0, 0x03);
    let pascal = pci(0x10de, 0x1c03, 0x03);
    // A supported NVIDIA GPU answers: nothing to say, empty ports or not.
    assert_eq!(
        nvidia_notice(&[pci(0x10de, 0x2c02, 0x03), port(2)]),
        NvidiaNotice::Driven
    );
    assert_eq!(nvidia_notice(&[pascal, port(2)]), NvidiaNotice::Driven);
    assert_eq!(nvidia_notice(&[maxwell, pascal]), NvidiaNotice::Driven);
    // Every port has something below it: what the census shows is final.
    assert_eq!(
        nvidia_notice(&[intel, port(2), below(2)]),
        NvidiaNotice::Absent
    );
    assert_eq!(nvidia_notice(&[]), NvidiaNotice::Absent);
    assert_eq!(
        nvidia_notice(&[maxwell, port(2), below(2)]),
        NvidiaNotice::NotDriven
    );
    // A hybrid laptop whose GPU is in D3cold: its port shows nothing below
    // it, and only the engine's power-on can tell.
    assert_eq!(nvidia_notice(&[intel, port(1)]), NvidiaNotice::NotYet);
    // A second, supported GPU may still answer behind an empty port.
    assert_eq!(nvidia_notice(&[maxwell, port(2)]), NvidiaNotice::NotYet);
    // A port without a bus number of its own forwards nothing yet.
    let unnumbered = Pci {
        bridge: Some((0, 0)),
        ..port(0)
    };
    assert_eq!(nvidia_notice(&[intel, unnumbered]), NvidiaNotice::Absent);
    // A function of another class or vendor is no NVIDIA display controller.
    assert_eq!(
        nvidia_notice(&[pci(0x10de, 0x2c02, 0x04), pci(0x1002, 0x7550, 0x03)]),
        NvidiaNotice::Absent
    );
}

#[test]
fn late_files_are_read_whole_by_family_within_the_handoffs_room() {
    let functions = [pci(0x8086, 0x46a6, 0x03), port(1)];
    let usb = select(&[]);
    let mut tight = select_for(&functions, Ports::ALL);
    let amd = tight.late[0].bytes();
    let amd_files = tight.late[0].files.len();
    // Room for the AMD files but not NVIDIA's: AMD's are taken whole and
    // NVIDIA's left out whole.
    let left = tight.take_late(Room {
        images: usb.wanted.len() + amd_files + 1,
        bytes: usb.bytes() + amd + 1,
    });
    assert_eq!(left.len(), 1);
    assert_eq!(left[0].family, "NVIDIA GSP-RM");
    assert_eq!(tight.wanted.len(), usb.wanted.len() + amd_files);
    assert!(
        tight
            .wanted
            .iter()
            .all(|(artifact, _)| !artifact.member.starts_with("nvidia/"))
    );
    // One image short: nothing of a family is taken.
    let mut short = select_for(&functions, Ports::NONE);
    let files = short.late[0].files.len();
    let left = short.take_late(Room {
        images: usb.wanted.len() + files - 1,
        bytes: usize::MAX,
    });
    assert_eq!(left.len(), 1);
    assert_eq!(members(&short), USB);
}

#[test]
fn the_engine_learns_what_became_of_each_bundle_file_before_the_exit() {
    let files: Vec<_> = complete().into_iter().take(4).collect();
    let bytes = manifest(&files).unwrap();
    let bundle = Bundle::parse(&bytes).unwrap();
    let digests: Vec<&str> = files.iter().map(|(object, _)| object.sha256).collect();
    let states = [Held::Read, Held::Failed, Held::NoRoom, Held::Unread];
    let record = held_record(&bundle, |sha256| {
        states[digests.iter().position(|seen| *seen == sha256).unwrap()]
    });
    assert_eq!(record.len(), 4 * 67);
    for (digest, state) in digests.iter().zip(states) {
        assert_eq!(held(Some(&record), digest), state);
    }
    // A file the bundle does not hold, and a boot that found no bundle.
    let other = complete().last().unwrap().0.sha256;
    assert_eq!(held(Some(&record), other), Held::Lacks);
    assert_eq!(held(Some(&[]), other), Held::Lacks);
    assert_eq!(held(None, digests[0]), Held::NoBundle);
    // A damaged line is no state.
    let mut damaged = record.clone();
    damaged[65] = b'x';
    assert_eq!(held(Some(&damaged), digests[0]), Held::Lacks);
}

#[test]
fn a_gpu_lacking_files_is_told_why_in_the_order_a_user_can_act_on() {
    use Held::*;
    // Any file the USB lacks: `stela repair` is the answer.
    assert_eq!(Absent::of([Unread, Lacks, Failed]), Absent::NotOnUsb);
    assert_eq!(Absent::of([NoBundle]), Absent::NotOnUsb);
    // On the USB, and a read failed.
    assert_eq!(Absent::of([Unread, Failed]), Absent::ReadFailed);
    // On the USB and not read: the GPU answered after the firmware's exit.
    assert_eq!(Absent::of([Unread, NoRoom]), Absent::AnsweredLate);
    assert_eq!(Absent::of([Unread]), Absent::AnsweredLate);
}
