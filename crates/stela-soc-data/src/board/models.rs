//! Compatible and role matching for the native models exposed in the DeviceTree.
use super::{
    AES, AICS, ASC, BOOT_FIRMWARE, BUTTONS, CLPC, DCP_EXPERT, DCPEXT_ROLE, DISPEXT, DISPEXT_T6000,
    DOCK_PORT, GFX_MAPPER, GPIO, GPIO_S5L, GPIO_T6000, HPM, HPM_MANAGER, IOMMU_MAPPER, MCC, NOR,
    NVRAM, PMGR2_IMPLEMENTATION, PMU, RTBUDDY, SEP, SEP_NUB, SMC_PMU, SOCS, SPI, SPMI, SYSCFG,
    T6000_EXTERNAL_OUTPUT_ADMITTED, UART, WATCHDOG, admitted_gpu, atc_phy_compatible,
    dart_topology, gpio_profile, i2c_compatible, memory_resources, usb_controller_compatible,
};

/// A compatible may require a distinguishing property and may own children
/// sharing a compatible with unrelated devices. Match properties exactly;
/// a missing or malformed role never broadens the claim.
pub(super) struct ModelClaim {
    pub(super) compatible: &'static str,
    pub(super) property: (&'static str, &'static [u8]),
    pub(super) children: &'static [ChildModel],
    /// Boot-chain facts the claimed node itself carries, as
    /// [`ChildModel::boot_properties`] do for a child.
    pub(super) boot_properties: &'static [(&'static str, &'static [u8])],
    /// Whether the claim admits its node; see [`GPU_ADMITTED`].
    pub(super) admitted: bool,
}

/// Whether the GPU (`sgx`, its `gfx-asc` coprocessor and firmware mapper) is
/// visible. Its kernel window resolves through SPTM's shared L2 and the
/// firmware consumes the kext's device-control ring, so
/// `AGXAcceleratorG14G::start` completes on the 26A434 J473AP kext in a
/// restore boot. GPU work commands still refuse by name until their fields
/// are confirmed.
pub const GPU_ADMITTED: bool = true;
/// A child owned by a native coprocessor model and its boot-chain facts.
/// Properties describe model state, never a guest image or a driver patch.
/// `children` are the child's own children the same model owns.
pub struct ChildModel {
    pub compatible: &'static str,
    pub boot_properties: &'static [(&'static str, &'static [u8])],
    pub children: &'static [ChildModel],
}
/// Native RTKit is already running before SPTM and uses no ARM firmware
/// memory. RTBuddy::_initConfigEDT (0x4661408..14b4) reads
/// pre-loaded/running/no-firmware-service; getSegmentMap (0x4661d68/1df8)
/// accepts empty segment-ranges/segment-names. _handlePreloadFirmware
/// (0x465a488) consumes that segment map.
const NATIVE_RTKIT_PROPERTIES: &[(&str, &[u8])] = &[
    ("pre-loaded", b""),
    ("running", b""),
    ("no-firmware-service", b""),
    ("segment-ranges", b""),
    ("segment-names", b""),
];
const NATIVE_RTKIT: ChildModel = ChildModel {
    compatible: RTBUDDY,
    boot_properties: NATIVE_RTKIT_PROPERTIES,
    children: &[],
};
/// The SMC's native nub, which also owns its `smc-pmu` child ([`SMC_PMU`]).
const SMC_RTKIT: ChildModel = ChildModel {
    compatible: RTBUDDY,
    boot_properties: NATIVE_RTKIT_PROPERTIES,
    children: &[ChildModel {
        compatible: SMC_PMU,
        boot_properties: &[],
        children: &[],
    }],
};
/// The SEP's native nub (`iop-nub,sep`, kept so `AppleSEPManager`'s
/// `A7IOPNub` personality binds). Unlike RTBuddy coprocessors, the SEP is not
/// an RTKit preload: it boots through its SEPROM mailbox handshake, so no
/// pre-loaded/running firmware-segment markers are injected.
const SEP_NUB_MODEL: ChildModel = ChildModel {
    compatible: SEP_NUB,
    boot_properties: &[],
    children: &[],
};
/// The register tunings the ANS2 driver requires before it re-powers the
/// coprocessor. `AppleANS2NVMeController::ApplyTunables` (0x3dbfc48), from
/// `PowerStateAction` (0x3dbf740), fails with 0xe00002c9 when any of these
/// `/arm-io/ans` properties is missing (0x3dbfeec..0x3dbff40). The IPSW
/// DeviceTree has none of them, so the boot chain supplies them. Each is a
/// list of 24-byte entries: register offset, width (4), mask and value.
/// The command accelerator's offsets are relative to `reg[3]` + 0x20000. The
/// others are relative to `reg[4]`, `reg[5]` and `reg[6]`
/// (`mapDeviceMemoryWithIndex` 4..6 at 0x3dbf36c..0x3dbf448).
/// `ApplyTunablesFromData` (0x3dbf914) accepts an empty list without any
/// access. Emulated die: Stela's choice. The die has no analog or fabric
/// trim to set, so each list is empty. The ECC and parity shadow lists are
/// read only when `reg[7]`, `reg[8]` or `gib-pcie-up` exist, and J473AP has none
/// of them. The `tunable-table-bundle` is unrelated: it names the restore
/// updater's NAND tables, which reach the MSP through tunnel commands
/// (`AppleNVMeUpdateUC::TunableTableDownload`, selector 14).
const ANS2_TUNABLES: &[(&str, &[u8])] = &[
    ("afc-aiu-ans-dual-tunables", b""),
    ("command-accelerator-tunables", b""),
    ("storage-lane-common-tunables", b""),
    ("ccu-security-interrupt-tunables", b""),
];
pub(super) const ROLE_MODELS: &[ModelClaim] = &[
    ModelClaim {
        compatible: ASC,
        property: ("role", b"ANS2\0"),
        children: &[NATIVE_RTKIT],
        boot_properties: ANS2_TUNABLES,
        admitted: true,
    },
    ModelClaim {
        compatible: ASC,
        property: ("role", b"SMC\0"),
        children: &[SMC_RTKIT],
        boot_properties: &[],
        admitted: true,
    },
    // The AGX GPU's coprocessor: AGXFirmwareKextRTBuddy::bootFirmware
    // (0x1ae9c88) panics unless its nub reads as iBoot-loaded.
    ModelClaim {
        compatible: ASC,
        property: ("role", b"GFX\0"),
        children: &[NATIVE_RTKIT],
        boot_properties: &[],
        admitted: GPU_ADMITTED,
    },
    // The SEP: a native model whose ASC wrapper uses `iop-sep,ascwrap-v4` and
    // whose nub is `iop-nub,sep`. Distinguished by its `role = SEP`, so it
    // never collides with the ANS2/SMC `iop,ascwrap-v4` coprocessors.
    ModelClaim {
        compatible: SEP,
        property: ("role", b"SEP\0"),
        children: &[SEP_NUB_MODEL],
        boot_properties: &[],
        admitted: true,
    },
    ModelClaim {
        compatible: ASC,
        property: ("role", b"PMP\0"),
        children: &[NATIVE_RTKIT],
        boot_properties: &[],
        admitted: true,
    },
    // The display coprocessor (`platform::apple::dcp`): its `iop-dcpext-nub`
    // reads as the native RTKit the other ASC claims present.
    ModelClaim {
        compatible: ASC,
        property: ("role", DCPEXT_ROLE),
        children: &[NATIVE_RTKIT],
        boot_properties: &[],
        admitted: true,
    },
    ModelClaim {
        compatible: ASC,
        property: ("role", b"DCPEXT0\0"),
        children: &[NATIVE_RTKIT],
        boot_properties: &[],
        admitted: T6000_EXTERNAL_OUTPUT_ADMITTED,
    },
    ModelClaim {
        compatible: DCP_EXPERT,
        property: ("role", b"DCPEXT0\0"),
        children: &[],
        boot_properties: &[],
        admitted: T6000_EXTERNAL_OUTPUT_ADMITTED,
    },
    // Its power expert ([`DCP_EXPERT`]), a register-less node that
    // `AppleDCPExpert` binds to. IOMFB waits for it before it makes a call.
    ModelClaim {
        compatible: DCP_EXPERT,
        property: ("role", DCPEXT_ROLE),
        children: &[],
        boot_properties: &[],
        admitted: true,
    },
];

/// Child compatibles claimed by a matching parent model. The loader uses
/// this relationship before changing the tree, without relying on paths.
pub fn model_children<'a>(
    name: &str,
    property: impl Fn(&str) -> Option<&'a [u8]>,
) -> &'static [ChildModel] {
    ROLE_MODELS
        .iter()
        .find(|claim| {
            claim.admitted
                && claim.compatible == name
                && property(claim.property.0) == Some(claim.property.1)
        })
        .map_or(&[], |claim| claim.children)
}

/// Boot-chain properties a matching claimed node carries itself.
pub fn model_boot_properties<'a>(
    name: &str,
    property: impl Fn(&str) -> Option<&'a [u8]>,
) -> &'static [(&'static str, &'static [u8])] {
    ROLE_MODELS
        .iter()
        .find(|claim| {
            claim.admitted
                && claim.compatible == name
                && property(claim.property.0) == Some(claim.property.1)
        })
        .map_or(&[], |claim| claim.boot_properties)
}

/// Whether a compatible and its distinguishing properties name a modeled
/// device. The loader also checks provider and parent/child relationships.
pub fn models<'a>(name: &str, property: impl Fn(&str) -> Option<&'a [u8]>) -> bool {
    [
        UART,
        WATCHDOG,
        IOMMU_MAPPER,
        AES,
        MCC,
        // Direct MCC has a native cold owner. Its descriptor must still pass
        // full source validation and mount requires that owner.
        memory_resources::COMPATIBLE,
        memory_resources::t6020::COMPATIBLE,
        SPI,
        NOR,
        BOOT_FIRMWARE,
        NVRAM,
        SYSCFG,
        GPIO,
        HPM_MANAGER,
        HPM,
        DOCK_PORT,
        SPMI,
        PMU,
        BUTTONS,
        DISPEXT,
        CLPC,
    ]
    .contains(&name)
        || i2c_compatible(name, property("compatible"))
        || usb_controller_compatible(name, property("compatible"))
        || T6000_EXTERNAL_OUTPUT_ADMITTED && name == DISPEXT_T6000
        || [GPIO_T6000, GPIO_S5L].contains(&name)
            && gpio_profile(&property) == Some(crate::gpio::Profile::S5l8960x)
        || crate::dart::Profile::from_compatible(name.as_bytes()).is_some_and(|profile| {
            property("compatible")
                .is_none_or(|bytes| crate::dart::Profile::from_compatible(bytes) == Some(profile))
                && (profile == crate::dart::Profile::T8110
                    || dart_topology::legacy_requirements(profile, &property).is_ok())
        })
        || GPU_ADMITTED && (name == GFX_MAPPER || admitted_gpu(name))
        || AICS.contains(&name)
        || atc_phy_compatible(name, property("compatible"))
        || SOCS.iter().any(|soc| soc.pmgr == name)
        || name == PMGR2_IMPLEMENTATION
        || ROLE_MODELS.iter().any(|claim| {
            claim.admitted
                && claim.compatible == name
                && property(claim.property.0) == Some(claim.property.1)
        })
}
