//! The devices a boot bundle presents: only those the machine models.
//!
//! The board's DeviceTree describes every device of the real SoC. SPTM's
//! bootstrap (DART instances by `compatible`, `dart-mapper` restrictions
//! through their DART's `compatible`) and the kernel's drivers bind to a
//! device through its `compatible`, and would then program hardware the
//! machine does not have. A node whose `compatible` names no modeled device
//! (`stela_soc_data::board::models`) loses that property and keeps the
//! rest, so its `reg`, phandle and children stay where software that only
//! reads them expects them. The root, `/arm-io` and each `/cpus` child keep
//! theirs: they describe the board, its I/O fabric and the PEs the engine
//! runs. darwin-vm `dt_fixup.py` (MIT, commit `edfc1943`) boots the same
//! SPTM kernels to a shell in QEMU after removing `compatible` the same way.
//!
//! A modeled native DART presents only the streams visible devices
//! use: an `iommu-mapper` child stays only while a device that keeps its
//! `compatible` names it as `iommu-parent`, and the DART itself only while
//! one of its mappers stays. SPTM initializes every DART the tree shows, and
//! XNU starts an IODARTMapper per mapper; a DART no visible device reaches
//! would only be programmed for clients the machine does not have.
//! A modeled device whose `iommu-parent` names a mapper of a DART the
//! machine does not model (including unchecked legacy filter, remap or
//! routing variants) goes too, before its streams are counted: its DMA would
//! have no translation to reach RAM through, so its driver must not bind.
//!
//! A selection of a hidden device goes with it: XNU `pe_serial.c` (`xnu-12377`)
//! takes `/defaults` `serial-device`, a phandle, as its console and panics
//! when that node has no `compatible`. On J473AP it names the unmodeled
//! `dockchannel-uart`, so the property is removed and the kernel runs without
//! a console unless the `serial-device-name` boot-arg selects one.
//!
//! So does the root's `secure-root-prefix` once no AES accelerator (a node of
//! `device_type` `aes`) stays visible. `AppleARMPE::start` (26A434 J473AP
//! kernelcache) reads it from the platform node: without it the platform
//! records the root as secure and `SecureRootName` returns at once; with it,
//! `SecureRootName`, which `IOSecureBSDRoot` calls from `bsd_init`, compares
//! the root device's name against the prefix and then sleeps until a
//! `SecureRoot` call without a result buffer consumes the verdict.
//! `IOAESAccelerator::start` makes that call to install its
//! `SecureRootCallBack`, so on J473AP (`aes,s8000`, root `md0`, prefix `md`)
//! a hidden AES engine leaves the kernel asleep before `launchd` forever.
//! Removing the prefix keeps the verdict a matching root gets (secure); only
//! the callback into the absent driver goes.
//!
//! A watchdog whose board shows no PMU says so. `AppleARMWatchdogTimer::start`
//! (26A434 J473AP kernelcache file 0x1ca4458..0x1ca4488) waits without a
//! timeout for an `IOPMU` service unless its node has `no-pmu`; a PMU
//! driver binds to a `pmu,*` node (J473AP's SPMI `pmu,spmi`). With every PMU
//! hidden, that wait never ends, `IOWatchdog` never registers and
//! `restored_external` fails its `disable_watchdog` step, so a visible
//! watchdog gets an empty `no-pmu`, the board fact the kernel tests for.
//!
//! A role-claimed coprocessor node, and each child its model owns, also gets
//! the boot-chain facts that model declares (`board::model_boot_properties`,
//! `board::ChildModel`), such as a native RTKit's preloaded state or ANS2's
//! empty register tuning lists.
//!
//! Some peripherals bind by `name`/`device_type` without a `compatible`.
//! Removing `compatible` cannot hide those clients. Declared required
//! platform-function dependencies are resolved after ordinary admission;
//! a client without its provider gets `AAPL,ignore`, which XNU's
//! `IODTFindMatchingEntries` skips. Its name, phandle and properties remain
//! available as DeviceTree data. Optional functions are not dependencies.
//!
//! A final pass also retires audited service-advertising `device_type`
//! values on hidden devices: raw DeviceTree display discovery does not
//! honor `compatible` or `AAPL,ignore`, and `IONameMatch` compares the type
//! too, so a Thunderbolt or DisplayPort driver would bind a hidden node and
//! wait in `start` for hardware that never answers, holding the registry
//! busy until kernelmanagerd's quiet wait panics. The existing admission
//! ledger keeps the hidden paths; original inputs keep the removed identity
//! bytes. Structural and unaudited type metadata remain.
//!
//! A kept client's `acio-parent` naming a hidden ACIO goes too (`PATH
//! acio-parent`): `resolvePHandleHAL` (26A434 0xfffffe000979f710) gives up
//! at once without the property but otherwise waits 60 s for a controller
//! that cannot appear. Either way the client has no USB4 tunnels.
//!
//! A hidden device also loses the capability properties user space reads
//! from the raw tree whatever binds it (`board::CAPABILITY_PROPERTIES`,
//! `PATH NAME`): `/arm-io/ane`'s `ane-type` is how libMobileGestalt answers
//! `HasAppleNeuralEngine`, so without a Neural Engine model the board says
//! it has none, as Apple's virtual machines do. Its `reg`, gates, clocks and
//! DART links stay as data, like every hidden node's.
//!
//! An SPMI controller with no admitted target or external platform-function
//! or interrupt consumer also loses `compatible`. Board templates may carry
//! alternative controllers over the same banks (26A434 J623/J624); iBoot
//! selects those by hardware straps. An unused bus needs no selection. If
//! both alternatives serve kept consumers, both remain and mounting
//! explicitly refuses their overlapping ownership rather than choosing one.
use crate::apple_device_tree::{DeviceTree, Node};
use alloc::{format, string::String, vec::Vec};
use stela_soc_data::board;

const COMPATIBLE: &str = "compatible";
const ROLE: &str = "role";
const GFX_ROLE: &[u8] = b"GFX\0";
const DEFAULTS: &str = "/defaults";
const SERIAL_DEVICE: &str = "serial-device";
const PHANDLE: &str = "AAPL,phandle";
const SECURE_ROOT_PREFIX: &str = "secure-root-prefix";
const DEVICE_TYPE: &str = "device_type";
const IOMMU_PARENT: &str = "iommu-parent";
const ACIO_PARENT: &str = "acio-parent";
const NO_PMU: &str = "no-pmu";
const IGNORE: &str = "AAPL,ignore";
/// Consumers that use these physical classes as service advertisements even
/// after `compatible` is removed: raw DeviceTree readers of the display
/// classes, and the
/// `IONameMatch` of `AppleThunderboltHALType5` and the
/// `AppleATCDP*Port` drivers, which match the type alone. Only audited class
/// identities are retired; unknown `device_type` metadata stays intact.
const SERVICE_IDENTITIES: &[&[u8]] = &[
    b"display-subsystem\0",
    b"ext-display-subsystem\0",
    b"acio\0",
    b"AAPL,atc-dpphy\0",
    b"AAPL,atc-dpin\0",
    b"AAPL,atc-dpout\0",
];
/// Required functions of name-matched hardware classes, not optional
/// callbacks inferred from arbitrary `function-*` properties. The original
/// AMFM requires its PCIe platform function. Other WLAN interfaces need
/// their active bus established before imposing a direct PCIe requirement;
/// the original driver also has a serial-provider personality.
const REQUIRED_FUNCTIONS: &[(&str, &[&str])] = &[("amfm", &["function-pcie_port_control"])];
/// Name-matched hardware classes and the property naming the provider each
/// waits for in `start`. `DCPAVAudioDMADelegate`'s `IONameMatch` lists
/// `dp-audio0` to `dp-audio9`, so run 130's kernelmanagerd quiet wait panicked
/// on those nubs with their `sio-dma` absent.
const REQUIRED_PARENTS: &[(&str, &str)] = &[("dp-audio", "dma-parent")];
/// A service-selected hardware interface replaces the direct platform
/// function, rather than making it optional. AppleOLYHAL's withProvider
/// (26A434 0xfffffe0009b465a8..0x9b465dc) selects its AMFM implementation
/// when this marker exists; its init waits for AppleMultiFunctionManager
/// (0x9b34d64..0x9b34dac), whose original IONameMatch is `amfm`.
const REQUIRED_SERVICES: &[(&str, &str, &str)] = &[("wlan", "amfm-managed-port-control", "amfm")];
/// The prefix of a button's platform function: the provider's phandle, the
/// function code and its arguments (`function-button_hold`).
const BUTTON_FUNCTION: &str = "function-button_";
/// The GPU TTBAT the boot stage hands off (`gpu-region-base`), published as
/// `AGXSecureMonitor::init` (KC file 0x1b8c5c4..0x1b8c6b4) reads it: the
/// presence of `gptbat-ready`, then `ttbat-phys-addr-base` as one 32-bit
/// page number of 16 KiB.
const GPU_REGION_BASE: &str = "gpu-region-base";
const GPTBAT_READY: &str = "gptbat-ready";
const TTBAT_BASE: &str = "ttbat-phys-addr-base";
const TTBAT_PAGE_SHIFT: u32 = 14;

/// Paths that keep their `compatible` whatever it names.
fn structural(path: &str) -> bool {
    path == "/"
        || path == "/arm-io"
        || path
            .strip_prefix("/cpus/")
            .is_some_and(|cpu| !cpu.contains('/'))
}

fn compatibles(node: &Node) -> impl Iterator<Item = &str> {
    node.property(COMPATIBLE)
        .into_iter()
        .flat_map(|property| property.value.split(|&byte| byte == 0))
        .filter_map(|name| core::str::from_utf8(name).ok())
}

fn named(node: &Node, model: &str) -> bool {
    compatibles(node).any(|name| name == model)
}

fn is_dart(node: &Node) -> bool {
    compatibles(node).any(|name| name.starts_with("dart,"))
}

fn modeled_dart(node: &Node) -> bool {
    use stela_soc_data::dart::Profile;
    let Some(profile) = node
        .property(COMPATIBLE)
        .and_then(|property| Profile::from_compatible(&property.value))
    else {
        return false;
    };
    if !compatibles(node)
        .any(|name| board::models(name, |key| node.property(key).map(|p| p.value.as_slice())))
    {
        return false;
    }
    if profile == Profile::T8110 {
        return true;
    }
    // A capture descriptor does not create another translating owner.
    let descriptors = &node
        .property("instance")
        .expect("qualified native descriptors")
        .value;
    let translations = descriptors
        .as_chunks::<12>()
        .0
        .iter()
        .filter(|d| &d[..4] == b"TRAD")
        .count();
    let mut mappers = node
        .children
        .iter()
        .filter(|child| named(child, board::IOMMU_MAPPER));
    if translations != 1 && mappers.clone().next().is_some() {
        return false;
    }
    let streams = node
        .property("sid-count")
        .and_then(|p| p.as_u32())
        .unwrap_or(16) as usize;
    let remaps = node
        .property("remap")
        .map(|p| p.value.as_slice())
        .unwrap_or_default()
        .as_chunks::<2>()
        .0
        .iter()
        .filter(|p| **p != [0, 0])
        .map(|p| (p[0], p[1]))
        .collect::<Vec<_>>();
    // The source mapper names a context, not an incoming remapped SID.
    mappers.all(|mapper| {
        let Some(context) = mapper
            .property("reg")
            .and_then(|p| <[u8; 4]>::try_from(p.value.as_slice()).ok())
            .map(u32::from_le_bytes)
        else {
            return false;
        };
        profile
            .mapper_input_stream(streams, context as usize, &remaps)
            .is_ok()
    })
}

/// A modeled device that is not part of an IOMMU.
fn modeled_client(node: &Node) -> bool {
    // Only NOR-backed controllers have a modeled SPI slave. Other SPI
    // controllers must not bind drivers for absent peripherals.
    if named(node, board::SPI) && !node.children.iter().any(|child| named(child, board::NOR)) {
        return false;
    }
    !is_dart(node)
        && !named(node, board::IOMMU_MAPPER)
        && compatibles(node)
            .any(|name| board::models(name, |key| node.property(key).map(|p| p.value.as_slice())))
}

/// The IOMMU, GPIO and interrupt-controller phandles that visible clients
/// under `node` require.
fn client_providers(node: &Node, parents: &mut Vec<u32>) {
    if modeled_client(node)
        && let Some(parent) = node.property("interrupt-parent").and_then(|p| p.as_u32())
    {
        parents.push(parent);
    }
    if modeled_client(node)
        && named(node, board::SPI)
        && let Some(phandle) = node
            .property("function-spi_cs0")
            .and_then(|p| p.value.get(..4))
    {
        parents.push(u32::from_le_bytes(phandle.try_into().expect("4 bytes")));
    }
    if modeled_client(node)
        && let Some(property) = node.property(IOMMU_PARENT)
    {
        let cells = property.value.as_chunks::<4>().0;
        parents.extend(cells.iter().map(|cell| u32::from_le_bytes(*cell)));
    }
    for child in &node.children {
        client_providers(child, parents);
    }
}

fn kept_mapper(node: &Node, parents: &[u32]) -> bool {
    named(node, board::IOMMU_MAPPER)
        && node
            .property(PHANDLE)
            .and_then(|property| property.as_u32())
            .is_some_and(|phandle| parents.contains(&phandle))
}

/// Whether the machine models `node` and it serves a visible device.
fn modeled(node: &Node, parents: &[u32]) -> bool {
    if named(node, board::GPIO) {
        node.property(PHANDLE)
            .and_then(|p| p.as_u32())
            .is_some_and(|p| parents.contains(&p))
    } else if is_dart(node) {
        modeled_dart(node)
            && node
                .children
                .iter()
                .any(|child| kept_mapper(child, parents))
    } else if named(node, board::SART) {
        node.property(PHANDLE)
            .and_then(|p| p.as_u32())
            .is_some_and(|p| parents.contains(&p))
    } else if named(node, board::IOMMU_MAPPER) {
        kept_mapper(node, parents)
    } else {
        modeled_client(node)
    }
}

/// The phandles of the `iommu-mapper` nodes under `node` whose DART the
/// machine does not model.
fn unmodeled_mappers(node: &Node, mappers: &mut Vec<u32>) {
    for child in &node.children {
        if !modeled_dart(node)
            && named(child, board::IOMMU_MAPPER)
            && let Some(phandle) = child.property(PHANDLE).and_then(|p| p.as_u32())
        {
            mappers.push(phandle);
        }
        unmodeled_mappers(child, mappers);
    }
}

/// Removes `compatible` from each modeled device under `node` whose
/// `iommu-parent` names one of `mappers`, in tree order.
fn hide_unserved_clients(node: &mut Node, path: &str, mappers: &[u32], hidden: &mut Vec<String>) {
    let unserved = modeled_client(node)
        && node.property(IOMMU_PARENT).is_some_and(|property| {
            property
                .value
                .as_chunks::<4>()
                .0
                .iter()
                .any(|cell| mappers.contains(&u32::from_le_bytes(*cell)))
        });
    if unserved {
        node.remove_property(COMPATIBLE);
        hidden.push(path.into());
    }
    for child in &mut node.children {
        let name = child.name().unwrap_or("?");
        let path = format!("{path}/{name}");
        hide_unserved_clients(child, &path, mappers, hidden);
    }
}

/// Removes `compatible` from every device the machine does not model, a
/// `/defaults serial-device` that selects one and a `secure-root-prefix`
/// that no AES accelerator can acknowledge, marks visible watchdogs
/// `no-pmu` when no PMU is visible, and returns their paths, those behind
/// an unmodeled DART first and then the rest in tree order, then the
/// selections and marks (`PATH +no-pmu`). Name-matched clients whose
/// declared required function providers are unavailable get `AAPL,ignore`
/// and an entry in that same ledger.
pub fn hide_unmodeled(tree: &mut DeviceTree) -> Vec<String> {
    let mut mappers = Vec::new();
    unmodeled_mappers(&tree.root, &mut mappers);
    let mut hidden = Vec::new();
    hide_unserved_clients(&mut tree.root, "", &mappers, &mut hidden);
    hide_incomplete_external_outputs(tree, &mut hidden);
    let mut parents = Vec::new();
    client_providers(&tree.root, &mut parents);
    visit(&mut tree.root, "", &parents, &[], &mut hidden);
    hide_unprovided_buttons(tree, &mut hidden);
    hide_unprovided_clients(tree, &mut hidden);
    hide_unused_spmi(tree, &mut hidden);
    let selected = tree
        .property(DEFAULTS, SERIAL_DEVICE)
        .and_then(|property| property.as_u32())
        .and_then(|phandle| path_of(&tree.root, "", phandle));
    if selected.is_some_and(|path| hidden.contains(&path)) {
        if let Some(defaults) = tree.node_mut(DEFAULTS) {
            defaults.remove_property(SERIAL_DEVICE);
        }
        hidden.push(format!("{DEFAULTS} {SERIAL_DEVICE}"));
    }
    if tree.root.property(SECURE_ROOT_PREFIX).is_some() && !visible_aes(&tree.root) {
        tree.root.remove_property(SECURE_ROOT_PREFIX);
        hidden.push(format!("/ {SECURE_ROOT_PREFIX}"));
    }
    if !visible(&tree.root, &|node| {
        compatibles(node).any(|name| name.starts_with("pmu,"))
    }) {
        let mut watchdogs = Vec::new();
        visible_paths(
            &tree.root,
            "",
            &|node| named(node, board::WATCHDOG),
            &mut watchdogs,
        );
        for path in watchdogs {
            let node = tree.node_mut(&path).expect("visited path");
            if node.property(NO_PMU).is_none() {
                node.set_property(NO_PMU, Vec::new())
                    .expect("empty property");
                hidden.push(format!("{path} +{NO_PMU}"));
            }
        }
    }
    publish_gpu_ttbat(tree, &mut hidden);
    retire_hidden_service_identities(tree, &hidden);
    retire_hidden_acio_parents(tree, &mut hidden);
    retire_hidden_capabilities(tree, &mut hidden);
    hidden
}

/// Drop each hidden device's raw capability claims
/// (`board::CAPABILITY_PROPERTIES`), noting `PATH NAME` in the ledger.
fn retire_hidden_capabilities(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    let mut notes = Vec::new();
    for path in hidden.iter() {
        if structural(path) {
            continue;
        }
        let Some(node) = tree.node_mut(path) else {
            continue;
        };
        if node.property(COMPATIBLE).is_some() && node.property(IGNORE).is_none() {
            continue;
        }
        for &name in board::CAPABILITY_PROPERTIES {
            if node.remove_property(name).is_some() {
                notes.push(format!("{path} {name}"));
            }
        }
    }
    hidden.extend(notes);
}

/// Hides the AGX model (`sgx`, its `gfx-asc` coprocessor and firmware mapper)
/// from a boot whose guest has a real GPU, so that StelaGPU is its only Metal
/// device, and from an installed macOS's console boot without one, which then
/// has no Metal device: the model refuses all GPU work, and macOS could
/// choose it. Returns
/// their paths for the admission ledger. The properties stay, as they do on
/// every hidden node.
pub fn hide_agx(tree: &mut DeviceTree) -> Vec<String> {
    let mut paths = Vec::new();
    visible_paths(
        &tree.root,
        "",
        &|node| {
            compatibles(node).any(|name| name == board::GFX_MAPPER || board::admitted_gpu(name))
                || node
                    .property(ROLE)
                    .is_some_and(|role| role.value == GFX_ROLE)
        },
        &mut paths,
    );
    let mut hidden = Vec::new();
    for path in paths {
        retire(
            tree.node_mut(&path).expect("visited path"),
            &path,
            &mut hidden,
        );
    }
    retire_hidden_service_identities(tree, &hidden);
    hidden
}

/// Removes `compatible` from `node` and everything under it.
fn retire(node: &mut Node, path: &str, hidden: &mut Vec<String>) {
    if node.property(COMPATIBLE).is_some() {
        node.remove_property(COMPATIBLE);
        hidden.push(path.into());
    }
    for child in &mut node.children {
        let name = child.name().unwrap_or("?");
        let path = format!("{path}/{name}");
        retire(child, &path, hidden);
    }
}

/// Keep buses only for visible targets or externally used provider
/// identities. The latter includes name-matched clients with no compatible;
/// the admission ledger and AAPL,ignore exclude unavailable consumers.
fn hide_unused_spmi(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    fn target(node: &Node) -> bool {
        node.property(IGNORE).is_none()
            && (node.property(COMPATIBLE).is_some() || node.children.iter().any(target))
    }

    fn referenced(node: &Node, path: &str, bus: &str, phandle: u32, hidden: &[String]) -> bool {
        let here = if path.is_empty() { "/" } else { path };
        if here == bus || node.property(IGNORE).is_some() {
            return false;
        }
        if !hidden.iter().any(|hidden| hidden == here)
            && node.properties.iter().any(|property| {
                if property.name().starts_with("function-") {
                    property.value.first_chunk::<4>() == Some(&phandle.to_le_bytes())
                } else if property.name() == "interrupt-parent" {
                    property
                        .value
                        .as_chunks::<4>()
                        .0
                        .contains(&phandle.to_le_bytes())
                } else {
                    false
                }
            })
        {
            return true;
        }
        node.children.iter().any(|child| {
            let path = format!("{path}/{}", child.name().unwrap_or("?"));
            referenced(child, &path, bus, phandle, hidden)
        })
    }

    let mut buses = Vec::new();
    visible_paths(&tree.root, "", &|node| named(node, board::SPMI), &mut buses);
    for path in buses {
        let bus = tree.node(&path).expect("visited SPMI path");
        if bus.children.iter().any(target)
            || bus
                .property(PHANDLE)
                .and_then(|p| p.as_u32())
                .is_some_and(|phandle| referenced(&tree.root, "", &path, phandle, hidden))
        {
            continue;
        }
        tree.node_mut(&path)
            .expect("visited SPMI path")
            .remove_property(COMPATIBLE);
        hidden.push(path);
    }
}

/// Admit the external controller, same-role expert and selected pipe as one
/// source-backed graph. IOMMU failures above cannot leave an expert advertising
/// a service for a controller that was removed. The final retirement pass also
/// removes raw display-subsystem advertisements for a hidden pipe.
fn hide_incomplete_external_outputs(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    for profile in board::EXTERNAL_OUTPUTS {
        let paths = [profile.controller, profile.expert, profile.pipe]
            .map(|name| format!("/arm-io/{name}"));
        if tree.node(&paths[0]).is_none()
            && tree.node(&paths[1]).is_none()
            && tree
                .node(&paths[2])
                .is_none_or(|n| !named(n, profile.compatible))
        {
            continue;
        }
        let controller = tree.node(&paths[0]);
        let expert = tree.node(&paths[1]);
        let pipe = tree.node(&paths[2]);
        let qualified = profile.admitted
            && controller.is_some_and(|n| {
                named(n, board::ASC)
                    && n.property("role")
                        .is_some_and(|p| p.value == profile.role_property)
                    && n.children
                        .iter()
                        .filter(|c| named(c, board::RTBUDDY))
                        .count()
                        == 1
                    && n.children
                        .iter()
                        .find(|c| named(c, board::RTBUDDY))
                        .is_some_and(|nub| {
                            profile.external_index.is_none_or(|index| {
                                nub.property("external-index").and_then(|p| p.as_u32())
                                    == Some(index)
                            })
                        })
            })
            && expert.is_some_and(|n| {
                named(n, board::DCP_EXPERT)
                    && n.property("role")
                        .is_some_and(|p| p.value == profile.role_property)
                    && n.property("join-power-plane").and_then(|p| p.as_u32()) == Some(1)
                    && n.property("reg").is_none()
                    && board::DCP_EXPERT_HARDWARE_FEATURES
                        .iter()
                        .all(|key| n.property(key).is_none())
            })
            && pipe.is_some_and(|n| {
                named(n, profile.compatible)
                    && profile
                        .external_index
                        .is_none_or(|_| n.property("external").and_then(|p| p.as_u32()) == Some(1))
            });
        if !qualified {
            for path in paths {
                if let Some(node) = tree.node_mut(&path)
                    && node.remove_property(COMPATIBLE).is_some()
                    && !hidden.contains(&path)
                {
                    hidden.push(path);
                }
            }
        }
    }
}

/// Retire a proven service-advertising class only after the admission pass
/// has established that its physical device is unavailable. The path ledger
/// stays unchanged; the original input and its source provenance keep the
/// identity bytes separately from this prepared tree. Names and phandles
/// stay usable by metadata readers; structural and unaudited types stay intact.
fn retire_hidden_service_identities(tree: &mut DeviceTree, hidden: &[String]) {
    for path in hidden {
        if structural(path) {
            continue;
        }
        let Some(node) = tree.node_mut(path) else {
            continue;
        };
        if node
            .property(DEVICE_TYPE)
            .is_some_and(|property| SERVICE_IDENTITIES.contains(&property.value.as_slice()))
        {
            node.remove_property(DEVICE_TYPE);
        }
    }
}

/// Drop each kept client's `acio-parent` naming a hidden node.
fn retire_hidden_acio_parents(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    let mut clients = Vec::new();
    visible_paths(
        &tree.root,
        "",
        &|node| node.property(ACIO_PARENT).is_some(),
        &mut clients,
    );
    for path in clients {
        let parent = tree
            .property(&path, ACIO_PARENT)
            .and_then(|property| property.as_u32())
            .and_then(|phandle| path_of(&tree.root, "", phandle));
        if parent.is_some_and(|parent| hidden.contains(&parent)) {
            let node = tree.node_mut(&path).expect("visited path");
            node.remove_property(ACIO_PARENT);
            hidden.push(format!("{path} {ACIO_PARENT}"));
        }
    }
}

fn required_service(node: &Node) -> Option<&'static str> {
    let kind = node.property(DEVICE_TYPE).and_then(|p| p.as_str());
    REQUIRED_SERVICES
        .iter()
        .find_map(|&(client, selector, service)| {
            (kind == Some(client) && node.property(selector).is_some()).then_some(service)
        })
}

fn required_functions(node: &Node) -> &'static [&'static str] {
    if required_service(node).is_some() {
        return &[];
    }
    let kind = node.property(DEVICE_TYPE).and_then(|p| p.as_str());
    REQUIRED_FUNCTIONS
        .iter()
        .find_map(|&(name, functions)| (kind == Some(name)).then_some(functions))
        .unwrap_or_default()
}

/// The phandle of the provider `node`'s class requires by property: `None`
/// for a class that names none, `Some(None)` for a node missing it.
fn required_parent(node: &Node) -> Option<Option<u32>> {
    let kind = node.property(DEVICE_TYPE).and_then(|p| p.as_str())?;
    REQUIRED_PARENTS
        .iter()
        .find(|&&(class, _)| kind.starts_with(class))
        .map(|&(_, property)| node.property(property).and_then(|p| p.as_u32()))
}

struct PlatformClient {
    path: String,
    phandle: Option<u32>,
    providers: Option<Vec<u32>>,
    /// Hardware class of the service this admitted client supplies.
    kind: Option<String>,
    service: Option<&'static str>,
    provided: bool,
}

/// Collect trusted provider roots and clients whose required dependencies
/// still need to be proven. An ignored subtree supplies no IOKit service.
fn platform_clients(
    node: &Node,
    path: &str,
    clients: &mut Vec<PlatformClient>,
    roots: &mut Vec<u32>,
) {
    if node.property(IGNORE).is_some() {
        return;
    }
    let required = required_functions(node);
    let phandle = node.property(PHANDLE).and_then(|p| p.as_u32());
    let service = required_service(node);
    let parent = required_parent(node);
    let here = if path.is_empty() { "/" } else { path };
    if (!required.is_empty() || service.is_some() || parent.is_some()) && !structural(here) {
        let providers = required
            .iter()
            .map(|name| {
                // Every platform function begins with a provider phandle
                // and a four-character operation code; trailing arguments
                // belong to that function's own protocol.
                let value = &node.property(name)?.value;
                let prefix = value.get(..8)?;
                Some(u32::from_le_bytes(prefix[..4].try_into().expect("4 bytes")))
            })
            .chain(parent)
            .collect();
        clients.push(PlatformClient {
            path: path.into(),
            phandle,
            providers,
            kind: node
                .property(DEVICE_TYPE)
                .and_then(|p| p.as_str())
                .map(String::from),
            service,
            provided: false,
        });
    } else if node.property(COMPATIBLE).is_some()
        && let Some(phandle) = phandle
    {
        roots.push(phandle);
    }
    for child in &node.children {
        let name = child.name().unwrap_or("?");
        platform_clients(child, &format!("{path}/{name}"), clients, roots);
    }
}

/// Prove required-provider and service chains from admitted roots. At most
/// one pass per client can add a provider or service, so a cycle without an
/// admitted root stays unprovided and terminates. Optional functions never
/// hide a client.
fn hide_unprovided_clients(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    let mut clients = Vec::new();
    let mut provided = Vec::new();
    platform_clients(&tree.root, "", &mut clients, &mut provided);
    let mut services = Vec::new();
    for _ in 0..clients.len() {
        let mut changed = false;
        for client in &mut clients {
            if !client.provided
                && client
                    .service
                    .is_none_or(|required| services.iter().any(|kind| kind == required))
                && client.providers.as_ref().is_some_and(|providers| {
                    providers.iter().all(|provider| provided.contains(provider))
                })
            {
                client.provided = true;
                if let Some(kind) = &client.kind {
                    services.push(kind.clone());
                }
                if let Some(phandle) = client.phandle {
                    provided.push(phandle);
                }
                changed = true;
            }
        }
        if !changed {
            break;
        }
    }
    for client in clients.into_iter().filter(|client| !client.provided) {
        tree.node_mut(&client.path)
            .expect("visited path")
            .set_property(IGNORE, Vec::new())
            .expect("empty property");
        if !hidden.contains(&client.path) {
            hidden.push(client.path);
        }
    }
}

/// Hides each visible `/buttons` node that names no `function-button_*`
/// provider, or one whose provider lost its `compatible`. AppleM68Buttons
/// reads its buttons through those platform functions (`function-button_hold`
/// = `smc-pmu`, `btnR`, `bHLD` on J473AP); with the provider hidden it would
/// bind to buttons no model drives.
fn hide_unprovided_buttons(tree: &mut DeviceTree, hidden: &mut Vec<String>) {
    let mut buttons = Vec::new();
    visible_paths(
        &tree.root,
        "",
        &|node| named(node, board::BUTTONS),
        &mut buttons,
    );
    for path in buttons {
        let node = tree.node(&path).expect("visited path");
        let providers: Vec<u32> = node
            .properties
            .iter()
            .filter(|property| property.name().starts_with(BUTTON_FUNCTION))
            .filter_map(|property| property.value.get(..4))
            .map(|cell| u32::from_le_bytes(cell.try_into().expect("4 bytes")))
            .collect();
        let provided = !providers.is_empty()
            && providers.iter().all(|&phandle| {
                path_of(&tree.root, "", phandle)
                    .and_then(|provider| tree.property(&provider, COMPATIBLE))
                    .is_some()
            });
        if !provided {
            tree.node_mut(&path)
                .expect("visited path")
                .remove_property(COMPATIBLE);
            hidden.push(path);
        }
    }
}

/// Publish on each visible GPU the TTBAT the machine hands it off with: its
/// MMU is rooted at `gpu-region-base`, as the boot stage that starts the GFX
/// firmware leaves it. Without the published table the kext roots its own,
/// which the running firmware never walks.
fn publish_gpu_ttbat(tree: &mut DeviceTree, notes: &mut Vec<String>) {
    let mut gpus = Vec::new();
    visible_paths(
        &tree.root,
        "",
        &|node| compatibles(node).any(board::admitted_gpu),
        &mut gpus,
    );
    for path in gpus {
        let node = tree.node_mut(&path).expect("visited path");
        if node.property(GPTBAT_READY).is_some() {
            continue;
        }
        let page = node
            .property(GPU_REGION_BASE)
            .and_then(|p| <[u8; 8]>::try_from(p.value.as_slice()).ok())
            .map(u64::from_le_bytes)
            .filter(|base| base.trailing_zeros() >= TTBAT_PAGE_SHIFT)
            .and_then(|base| u32::try_from(base >> TTBAT_PAGE_SHIFT).ok());
        if let Some(page) = page {
            node.set_property(GPTBAT_READY, Vec::new())
                .expect("empty property");
            node.set_property(TTBAT_BASE, page.to_le_bytes().to_vec())
                .expect("u32 property");
            notes.push(format!("{path} +{GPTBAT_READY}"));
        }
    }
}

/// Whether a node under `node` that `test` selects keeps its `compatible`.
fn visible(node: &Node, test: &dyn Fn(&Node) -> bool) -> bool {
    test(node) && node.property(COMPATIBLE).is_some()
        || node.children.iter().any(|child| visible(child, test))
}

/// The paths of the visible nodes under `node` that `test` selects.
fn visible_paths(node: &Node, path: &str, test: &dyn Fn(&Node) -> bool, paths: &mut Vec<String>) {
    if test(node) && node.property(COMPATIBLE).is_some() {
        paths.push(path.into());
    }
    for child in &node.children {
        let name = child.name().unwrap_or("?");
        visible_paths(child, &format!("{path}/{name}"), test, paths);
    }
}

/// Whether an AES accelerator under `node` keeps its `compatible`.
fn visible_aes(node: &Node) -> bool {
    let aes = node
        .property(DEVICE_TYPE)
        .is_some_and(|property| property.value.split(|&byte| byte == 0).next() == Some(b"aes"));
    aes && node.property(COMPATIBLE).is_some() || node.children.iter().any(visible_aes)
}

/// The path of the node whose phandle is `phandle`.
fn path_of(node: &Node, path: &str, phandle: u32) -> Option<String> {
    if node.property(PHANDLE).and_then(|p| p.as_u32()) == Some(phandle) {
        return Some(if path.is_empty() {
            "/".into()
        } else {
            path.into()
        });
    }
    node.children.iter().find_map(|child| {
        let name = child.name().unwrap_or("?");
        path_of(child, &format!("{path}/{name}"), phandle)
    })
}

fn visit(
    node: &mut Node,
    path: &str,
    parents: &[u32],
    inherited: &[&board::ChildModel],
    hidden: &mut Vec<String>,
) {
    let inherited_model = inherited
        .iter()
        .copied()
        .find(|model| named(node, model.compatible));
    if let Some(model) = inherited_model {
        for &(key, value) in model.boot_properties {
            node.set_property(key, value.to_vec())
                .expect("static native boot property");
        }
    }
    let own: Vec<(&str, &[u8])> = compatibles(node)
        .flat_map(|name| {
            board::model_boot_properties(name, |key| node.property(key).map(|p| p.value.as_slice()))
                .iter()
                .copied()
        })
        .collect();
    for (key, value) in own {
        node.set_property(key, value.to_vec())
            .expect("static native boot property");
    }
    let owned: Vec<&board::ChildModel> = compatibles(node)
        .flat_map(|name| {
            board::model_children(name, |key| node.property(key).map(|p| p.value.as_slice())).iter()
        })
        .chain(inherited_model.into_iter().flat_map(|model| model.children))
        .collect();
    let here = if path.is_empty() { "/" } else { path };
    if node.property(COMPATIBLE).is_some()
        && !structural(here)
        && !modeled(node, parents)
        && !inherited.iter().any(|model| named(node, model.compatible))
    {
        node.remove_property(COMPATIBLE);
        hidden.push(here.into());
    }
    for child in &mut node.children {
        let name = child.name().unwrap_or("?");
        let path = format!("{path}/{name}");
        visit(child, &path, parents, &owned, hidden);
    }
}

#[cfg(test)]
mod tests;
