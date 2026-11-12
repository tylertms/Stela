//! Apple SoCs: a board's chip and PEs from its DeviceTree.
//!
//! The board's DeviceTree gives the SoC: `/chosen` `chip-id` (the identity's
//! ApChipID, which the boot bundle writes there as iBoot does), `/product`
//! `product-soc-name`, and one `/cpus` node per PE with `reg` and
//! `cluster-type`. Each SoC row pairs an ApChipID with the
//! `product-soc-name` its boards' DeviceTrees give in UniversalMac 26A434 and
//! the MIDR_EL1 part number of each cluster type (m1n1 `src/midr.h`
//! `MIDR_PART_<soc>_<core>`; for the Ultras, the parts of the Max dies they
//! join, and for the second M3 Max and the M3 Ultra an inference stated at
//! their rows). ApChipID 0x6050 has three rows, the M5 Pro, Max and Ultra,
//! which its boards' DeviceTree template names per variant. A SoC row names
//! the `machine::Chip` whose CPU profile its cores take; the A18 Pro takes
//! the M4's (its row gives the evidence). A SoC, cluster type or affinity
//! without that evidence refuses: the T6050's M cores, whose caches no
//! source gives, the M6's M cores, and a T6050 template whose fused variant
//! is not merged yet (the boot bundle merges it, `stela_loader::soc_variant`).
use crate::Result;
use crate::agx;
use crate::aic::AIC2_DIE_STRIDE;
use crate::aic::AIC2_MAX_DIES;
use crate::aic::AIC3_IRQ_CFG;
use crate::aic::AIC3_TIMESTAMPS;
use crate::aic::AicLayout;
use crate::ans;
use crate::asc;
use crate::atc_phy;
use crate::cluster;
use crate::coresight;
use crate::dcp;
use crate::energy;
use crate::fuse;
use crate::i2c;
use crate::pmgr::coverage::DomainId;
use crate::sart;
use crate::smc;
use crate::spi;
use crate::wake;
use alloc::{collections::BTreeMap, format, string::String, vec::Vec};
use stela_contracts::refuse;
use {stela_contracts::adt::Node, stela_contracts::identity::Identity};

pub mod mpidr;
pub mod t6000_pmgr;

mod apertures;
mod interrupts;
pub use interrupts::InterruptController;
use interrupts::{aic_inputs, aic_layout, aic_vectors};
mod gpio_topology;
pub use gpio_topology::{AuxGpioBank, GpioBank};
use gpio_topology::{aux_gpio_banks, gpio_bank, gpio_profile};
mod usb_topology;
pub use usb_topology::{
    I2cAddress, I2cController, TypecPhy, UsbController, atc_phy_compatible, hpm_firmware_file,
    i2c_compatible, usb_controller_compatible,
};
use usb_topology::{typec_devices, usb_controllers};
mod spmi_topology;
use spmi_topology::spmi_controllers;
pub use spmi_topology::{FirmwareMailbox, SpmiController, SpmiPmu};
mod storage_topology;
pub use storage_topology::{AesEngine, AnsController, SpiNor};
use storage_topology::{aes_engine, ans_controller, spi_nor};
mod memory_topology;
pub use memory_topology::{MemoryCache, MemoryController, PtdBank};
use memory_topology::{PTD_RANGE, memory_cache, memory_controller, memory_controller_named};
mod boot_processor;
pub use boot_processor::BootProcessor;
mod processor_registers;
pub use processor_registers::ProcessorRegisters;
mod power_registers;
pub use power_registers::{
    BridgeBank, EnergyBank, PerfBank, PmgrBank, PmgrBankRefusal, PowerBank, SocDpe, WakeBank,
};
use power_registers::{PmgrRegisters, cpu_start, pmgr_registers};
mod pmp_topology;
pub use pmp_topology::PmpController;
use pmp_topology::pmp_controller;
mod smc_topology;
pub use smc_topology::SmcController;
use smc_topology::smc_controller;
mod gfx_topology;
use gfx_topology::gfx_controller;
pub use gfx_topology::{GfxController, admitted_gpu};
mod sep_topology;
pub use sep_topology::SepDevice;
use sep_topology::sep_device;
mod display_topology;
use display_topology::dcp_device;
pub use display_topology::{DcpController, DcpMapperRoute};
mod dart_topology;
mod dma_routes;
#[cfg(test)]
use dart_topology::iommu;
use dart_topology::iommus;
pub use dma_routes::NativeDmaRoute;

mod handoff;
mod memory_resources;
pub use memory_resources::DirectMemoryCache;
pub use memory_resources::t6020::{T6020Memory, from_io as t6020_memory_source};
mod power_resources;
mod topology;
pub use handoff::{
    BOOT_CLOCK_HZ, CHIP_REVISION, Carveout, DRAM_VENDOR, DramVendor, Evidence, HANDOFF_REGION,
    Handoff, IBOOT_PAGE_BYTES, PANIC_REGION_BYTES, PANIC_REGION_ID, PANIC_WINDOW_BYTES,
    PANIC_WINDOW_MEMBERS, SOCD_TRACE_KEYS, USB_PHY_HZ, handoff,
};
#[cfg(test)]
use power_resources::PMGR_VIRTUAL;
use power_resources::power_states;
pub use power_resources::{
    ClockRole, PmgrClockProvider, PmpDomain, PowerOwner, PowerResource, PowerStates,
    ProcessorAssociation,
};
pub use topology::{KernelLimits, assigned};

mod soc;
#[cfg(any(test, feature = "test-support"))]
use soc::cache;
#[cfg(any(test, feature = "test-support"))]
use soc::rows::T6050_LIMITS;
#[cfg(test)]
use soc::rows::{AIC3_EVENT, CPU_START_T8112, T8112_CLUSTERS};
use soc::rows::{PMGR2_ARCH, PMGR2_IMPLEMENTATION, SOCS};
pub use soc::{
    CoreType, CpuCluster, Cpus, Processor, aic, board_dram_base, chip, core_types, cpus, dram_base,
    level1_caches, midr_part, name, operating_points, template_variant,
};
use soc::{WakeLayout, chip_row};
#[cfg(any(test, feature = "test-support"))]
use stela_contracts::caches::Cache;
#[cfg(test)]
use stela_contracts::{Chip, caches::Caches};
mod models;
#[cfg(test)]
use models::ROLE_MODELS;
pub use models::{ChildModel, GPU_ADMITTED, model_boot_properties, model_children, models};

/// A device's first `reg` range in guest PA and its AIC input.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Device {
    pub physical: u64,
    pub bytes: u64,
    pub irq: u32,
}

/// The lockdown registers of one CPU implementation block: the ADT `/cpus`
/// property naming the block, its guest PA and size, the offset of its
/// lockdown CONTROL, and for a `cpu-impl-reg` the PE (in `/cpus` order)
/// whose RVBAR it holds.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct LockdownBlock {
    pub name: &'static str,
    pub physical: u64,
    pub bytes: u64,
    pub control: u64,
    pub cpu: Option<usize>,
}

/// One PE's CLPC sampler counters: the PE (in `/cpus` order), its
/// `cpu-impl-reg`'s guest PA and the SoC's counters
/// (`platform::apple::core_counters`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CounterBlock {
    pub cpu: usize,
    pub physical: u64,
    pub layout: crate::core_counters::Layout,
}

/// The `compatible` of the S5L UART the engine models (`platform::apple::uart`).
pub const UART: &str = "uart-1,samsung";
/// The `compatible` of the watchdog the engine models (`platform::apple::wdt`).
pub const WATCHDOG: &str = "wdt,s5l8960x";
/// The `compatible` of the closed-loop performance controller whose kext,
/// `AppleT8112CLPC`, runs on the modeled PMGR, core counters and PTD. Its
/// perfcontrol callbacks are what place thread groups on the P-cores.
pub const CLPC: &str = "clpc,t8112";

/// The ANE domain's clocks in MHz, lowest first. iBoot reads them from the
/// chip's PLLs; these are chosen, and nothing runs at them. `CLPC::init` turns
/// each P state's ANE clock into a curve and needs two different clocks, the
/// lowest at 19 MHz or more for the curve to be positive; the curve is near 1
/// at the highest.
pub const ANE_CLOCKS_MHZ: [u32; 2] = [300, 600];
/// Capability properties user space reads from the raw DeviceTree whatever
/// binds the node, so a hidden device's copy claims hardware the machine does
/// not have. A hidden device loses them (`PATH NAME` in the admission
/// ledger). The guest's libMobileGestalt answers `AppleNeuralEngineSubtype`
/// (shared cache 0x186e13548) with an `IORegistryEntrySearchCFProperty` for
/// `ane-type` over the whole IODeviceTree plane, and `HasAppleNeuralEngine`
/// (0x186e15744) is that subtype above zero. MetalFX, MPSGraph and Core ML
/// then send work to an ANE no driver serves. Without the property they take
/// the path Apple's virtual machines take. In the 26A434 kernelcache only
/// the ANE kext reads it, and it binds by `compatible`.
pub const CAPABILITY_PROPERTIES: &[&str] = &["ane-type"];
/// The AIC versions the engine models (`platform::apple::aic1` and `aic`).
const AICS: [&str; 3] = ["aic,1", "aic,2", "aic,3"];
/// The modern DART compatible; native legacy families are selected through
/// [`crate::dart::Profile::from_compatible`], with separate optional-feature gates.
pub const DART: &str = "dart,t8110";
/// The `compatible` of a DART child naming one client's stream (its `reg`),
/// which XNU's IODARTMapper binds and clients name as `iommu-parent`.
pub const IOMMU_MAPPER: &str = "iommu-mapper";
/// The `compatible` of the SoC's PMGR (each row's `pmgr`). The PMGR
/// family models place its power states and CPU start registers (m1n1
/// `src/pmgr.c` and `src/smp.c` read them alike on every SoC), clusters,
/// eFuses, energy, performance counters, wake masks, voltage, DPE and
/// fabric bridges.
pub fn pmgr(chip_id: u32) -> Option<&'static str> {
    chip_row(chip_id).map(|soc| soc.pmgr)
}

/// The `compatible` of the AES engine the engine models
/// (`platform::apple::aes`).
pub const AES: &str = "aes,s8000";
/// The `compatible` of the memory cache controller whose registers
/// `platform::apple::amcc`, `dcs` and `ptd` model.
pub const MCC: &str = "mcc,t8112";
/// ASC wrapper and nub shared by multiple coprocessor roles.
pub const ASC: &str = "iop,ascwrap-v4";
pub const RTBUDDY: &str = "iop-nub,rtbuddy-v2";
/// The SMC nub's PMU child: `AppleSMCPMU` binds it through the
/// `AppleSMCInterface` nub that `AppleSMCInterface::startInterfaces`
/// (26A434 J473AP KC file 0x2d99550) creates for every child of the SMC's
/// RTBuddy nub except `smc-socd`. Its contract (boot and shutdown records,
/// `MBSE` system events, HID button notifications and the `pKW4`, `keyW`
/// and `btnR` platform functions other nodes route through it) is the SMC
/// key endpoint's (`platform::apple::smc`).
pub const SMC_PMU: &str = "smc-pmu";
/// The board's buttons (`/buttons`), which `AppleM68Buttons` binds. Its
/// `function-button_*` platform functions name their provider; the loader
/// keeps the node only while each provider stays visible
/// (`stela_loader::devices`).
pub const BUTTONS: &str = "buttons";

pub const SART: &str = "sart,coastguard";

/// RTBuddy nub flags under which the AP does not wait for the coprocessor's
/// power-on announcement and wakes it by AP power instead (`rtkit` module
/// comment). No placed coprocessor model follows that contract.
const UNAWAITED_NUB: [&str; 2] = ["remote-only", "automatic-rail-gating-enabled"];

/// Select the protocol endpoint by its declared compatible under an already
/// checked ASC role. Node names are labels (J314s uses iop-gfx-nub0),
/// not a second protocol discriminator. Ambiguous endpoints refuse.
fn rtbuddy_nub<'a>(asc: &Node<'a>, role: &str) -> Result<Node<'a>> {
    let children = asc
        .children()
        .ok_or_else(|| refuse!("{role} children are malformed", role))?;
    let mut nubs = children.into_iter().filter(|node| node.compatible(RTBUDDY));
    let nub = nubs
        .next()
        .ok_or_else(|| refuse!("{role} has no RTBuddy v2 nub", role))?;
    if nubs.next().is_some() {
        return Err(refuse!("{role} has ambiguous RTBuddy v2 nubs", role));
    }
    awaited_nub(&nub, role)?;
    Ok(nub)
}

/// Refuse an RTBuddy nub whose start-up the RTKit model does not carry.
fn awaited_nub(nub: &Node<'_>, role: &str) -> Result<()> {
    match UNAWAITED_NUB
        .into_iter()
        .find(|flag| nub.property(flag).is_some())
    {
        Some(flag) => Err(refuse!(
            "{role} nub is {flag}, a start-up the RTKit model does not carry",
            role,
            flag
        )),
        None => Ok(()),
    }
}

/// SPI controller compatible, byte-PIO version 1.
pub const SPI: &str = "spi-1,spimc";
/// Single-wire SPI NOR compatible.
pub const NOR: &str = "nor-flash,spi";
/// Apple CHRP NVRAM service compatible.
pub const NVRAM: &str = "nvram,chrp";
/// The boot-firmware partition exposed through the modeled SPI NOR. The
/// kernel's AppleEmbeddedSimpleSPINORFlasherDriver binds to this child and
/// performs its reads, sector erases and page writes through the parent NOR.
pub const BOOT_FIRMWARE: &str = "iboot,boot";
/// GPIO output provider compatible.
pub const GPIO: &str = "gpio,t8101";
const GPIO_T6000: &str = "gpio,t6000";
const GPIO_S5L: &str = "gpio,s5l8960x";

/// The Type-C I2C bus: controller, HPM bus manager and CD321x dies.
pub const I2C: &str = "i2c,t8101";
pub const HPM_MANAGER: &str = "usbc,manager";
pub const HPM: &str = "usbc,cd3217";
pub const RETIMER: &str = "atcrt";
pub const ATC_PHY: &str = "atc-phy,t8112";
/// A Type-C connector description (`/port-usb-c-N`): no IOKit personality
/// binds it and it has no registers. The ATC PHY, `usb-drd` and ACIO name
/// it as `function-dock_parent`, which `AppleARMFunction::withProvider`
/// (0xfffffe0008c53aac) resolves through the node's registered platform
/// nub. With the node hidden, each of those starts stalled about 30 s and
/// the xHCI then failed to find its PHY.
pub const DOCK_PORT: &str = "dock,usb-c";
/// The SPMI controllers and the Dialog PMUs on them (`spmi`, `pmu`).
pub const SPMI: &str = "aapl,spmi";
pub const PMU: &str = "pmu,spmi";
/// A NOR partition read by AppleDiagnosticDataAccessReadOnly (kernel file
/// 0x235ecb8 start, 0x235f1e8 lazy whole-region read, 0x235f788 forwarding).
pub const SYSCFG: &str = "diagnostic-data,format1";

/// SEP's wrapper and native nub, from the J473AP IPSW DeviceTree. Admitted
/// through [`model_children`] (matched by `role = SEP`), so the loader keeps the
/// sep node and its nub and `AppleSEPManager` binds and boots the native SEP.
pub const SEP: &str = "iop-sep,ascwrap-v4";
pub const SEP_NUB: &str = "iop-nub,sep";

/// The J473AP HDMI display pipeline the engine models at its AP-facing
/// protocols (`platform::apple::dcp`, `display`): the `dcpext` coprocessor (the
/// shared ASC [`ASC`] with `role` `DCPEXT`, admitted through [`model_children`])
/// and the display pipe `dispext0` ([`DISPEXT`], kept so `AppleCLCD2`, an
/// `IOMobileFramebufferAP`, matches), whose DART `dart-dispext0` the
/// coprocessor reads swap surfaces through ([`DcpController::surface`]).
///
/// `dptx-phy` ([`DPTX_PHY`]) and `display-crossbar0` ([`DISPLAY_CROSSBAR`])
/// stay hidden. Their AP drivers (`AppleT8112DPTXPort` and
/// `AppleT8112DisplayCrossbar`, both in `com.apple.driver.AppleDisplayCrossbar`;
/// the 26A434 DeviceTree-to-kext match) configure the physical link: the
/// crossbar node has no `reg` and only routes the four `dispext` pipes and
/// `scodec0` to the ATC, `lpdptx` and `dptx` endpoints, and the PHY answers
/// the DCP's DPTX-port requests (AFK endpoint 0x2a, Linux `dptxep.c`). The
/// model drives no link and announces no DPTX-port service, so no request
/// reaches either driver; the model's monitor is connected by the
/// coprocessor itself. A guest that waits for their services instead is an
/// Open row of section 8.6.
pub const DCPEXT_ROLE: &[u8] = b"DCPEXT\0";
pub const DISPEXT: &str = "dispext0,t8112";
pub const DISPEXT_T6000: &str = "dispext0,t600x";
/// One native GOP-backed external graph with checked TRAD/UMMS ownership.
/// This admits its resource contract; visible Recovery output still requires
/// original firmware/kernel execution through that graph.
pub const T6000_EXTERNAL_OUTPUT_ADMITTED: bool = true;

/// A source-checked external pipeline family, preserving native identities.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct ExternalOutput {
    pub controller: &'static str,
    pub role: &'static str,
    pub role_property: &'static [u8],
    pub expert: &'static str,
    pub pipe: &'static str,
    pub compatible: &'static str,
    pub external_index: Option<u32>,
    pub identity: dcp::ExternalIdentity,
    pub admitted: bool,
}

pub const EXTERNAL_OUTPUTS: &[ExternalOutput] = &[
    ExternalOutput {
        controller: "dcpext",
        role: "DCPEXT",
        role_property: DCPEXT_ROLE,
        expert: "dcpext-expert",
        pipe: "dispext0",
        compatible: DISPEXT,
        external_index: None,
        identity: dcp::ExternalIdentity::Unnumbered,
        admitted: true,
    },
    ExternalOutput {
        controller: "dcpext0",
        role: "DCPEXT0",
        role_property: b"DCPEXT0\0",
        expert: "dcpext0-expert",
        pipe: "dispext0",
        compatible: DISPEXT_T6000,
        external_index: Some(1),
        identity: dcp::ExternalIdentity::Numbered(0),
        admitted: T6000_EXTERNAL_OUTPUT_ADMITTED,
    },
];

pub fn external_output(role: &[u8]) -> Option<&'static ExternalOutput> {
    EXTERNAL_OUTPUTS
        .iter()
        .find(|p| p.admitted && p.role_property == role)
}

/// These features require native expert operations beyond RTKit power.
pub const DCP_EXPERT_HARDWARE_FEATURES: &[&str] = &[
    "require-pmgr-scratch",
    "require-conditional-retention",
    "require-force-wakeup",
    "require-axi2af-reset",
    "support-dynamic-sysfe-pg",
    "support-psd-self-vote",
];
/// The coprocessor's power expert, `/arm-io/dcpext-expert` (`role` `DCPEXT`,
/// `join-power-plane`, no `reg`). In the 26A434 J473AP kernelcache:
/// - `UnifiedPipeline2::start` waits in `DCPPowerManager::wait_for_dcp_expert`,
///   and `DCPEndpointV2::start` in `AppleDCPExpert::copyDCPExpert`, until an
///   `AppleDCPExpert` with the coprocessor's role is published. Without it,
///   `AppleCLCD2` never makes an IOMFB call and restore reports no display.
/// - `AppleDCPExpert::start` (KC file 0x22f4278) reads its hardware features
///   from that node: `require-pmgr-scratch`, `require-conditional-retention`,
///   `require-force-wakeup`, `require-axi2af-reset`, `support-dynamic-sysfe-pg`
///   and `support-psd-self-vote`. J473AP sets none of them, so it maps no
///   register. It waits for the role's `RTBuddyService` and joins the power
///   plane.
/// - It changes the coprocessor's power only through that RTBuddy
///   (`_changeDCPPowerStateInternal`, 0x22f8928), whose RTKit power messages
///   the coprocessor model answers.
///
/// The primary display pipe's `dcp0-expert` (`role` `DCP`) stays hidden with its
/// unmodeled coprocessor.
pub const DCP_EXPERT: &str = "dcp-expert-v1";
pub const DPTX_PHY: &str = "dptx-phy,t8112";
pub const DISPLAY_CROSSBAR: &str = "display-crossbar,t8112";

/// T8112 device controller compatible; admission stays closed until the
/// unchanged Apple platform/PHY initialization has been observed.
pub const USB_DRD: &str = "usb-drd,t8112";

/// Everything the engine models of a board, from its DeviceTree.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Board {
    /// The AP GPIO bank, when visible.
    pub gpio: Option<GpioBank>,
    /// The other visible GPIO banks, in DeviceTree order.
    pub aux_gpios: Vec<AuxGpioBank>,
    /// SPI1, its NOR image and the GPIO chip-select wire, when visible.
    pub spi_nor: Option<SpiNor>,
    /// Explicitly visible I2C controllers and their addressed targets.
    pub i2cs: Vec<I2cController>,
    /// Explicitly visible T8112 PHYs. Prepared bundles still hide them.
    pub typec_phys: Vec<TypecPhy>,
    pub cpus: Cpus,
    /// The sole running node, if this is a prepared handoff rather than a template.
    pub boot_processor: Option<BootProcessor>,
    /// `/arm-io/uart0`, a `uart-1,samsung`.
    pub uart: Device,
    /// `/arm-io/wdt`, a `wdt,s5l8960x`.
    pub watchdog: Device,
    /// Optional side registers and per-CPU trigger words of the watchdog.
    pub watchdog_side: Option<WatchdogSide>,
    /// `/arm-io/aic`, an `aic,1`, `aic,2` or `aic,3`.
    pub aic: InterruptController,
    /// Every PE's `cpu-impl-reg` and every cluster's `cpm-impl-reg`, once
    /// each, on a SoC whose lockdown offsets are known; none otherwise.
    pub lockdown: Vec<LockdownBlock>,
    /// Each PE's original implementation block, independent of legacy lockdown.
    pub processor_registers: Vec<ProcessorRegisters>,
    /// Every PE's CLPC sampler counters in its `cpu-impl-reg`, on a SoC
    /// whose counters are known.
    pub counters: Vec<CounterBlock>,
    /// CPU CoreSight regions, keeping the PE identity and aperture length.
    pub coresight: Vec<CoreSight>,
    /// The AMCC lock registers of `/chosen/lock-regs/amcc`, when iBoot
    /// described them.
    pub memory_controller: Option<MemoryController>,
    /// The separately described I/O access CTRR controller.
    pub io_access_controller: Option<MemoryController>,
    /// Every supported native DART group, preserving DT traversal order.
    pub darts: Vec<Iommu>,
    /// `/arm-io/aes`, an `aes,s8000`, when the DeviceTree keeps it.
    pub aes: Option<AesEngine>,
    /// ANS2 and its SART, if an input tree explicitly presents the device.
    /// Prepared bundles still hide its compatibles until its vendor ABI is complete.
    pub ans: Option<AnsController>,
    /// The SMC coprocessor, when the DeviceTree shows its `iop,ascwrap-v4`.
    pub smc: Option<SmcController>,
    /// The PMP coprocessor, under the same condition.
    pub pmp: Option<PmpController>,
    /// The AGX GPU, when the DeviceTree shows both `gfx-asc` and `sgx`.
    pub gfx: Option<GfxController>,
    /// SEP placement if the input tree explicitly keeps it. Production
    /// `models()` admission remains closed while sepOS boot services refuse.
    pub sep: Option<SepDevice>,
    /// DWC3 placements of checked visible USB DRD families, in DeviceTree order.
    pub usbs: Vec<UsbController>,
    /// `/arm-io/pmgr`'s power-state registers, when the DeviceTree shows the
    /// SoC's PMGR ([`pmgr`]).
    pub power_states: Option<PowerStates>,
    /// Unsupported PMGR families; these grant no register/lifecycle effects.
    pub pmgr_refusals: Vec<PmgrBankRefusal>,
    /// `/arm-io/mcc`, when the DeviceTree shows an `mcc,t8112`.
    pub memory_cache: Option<MemoryCache>,
    /// Direct T6000 resources; parsing does not grant native RAM/DSB permission.
    pub direct_memory_cache: Option<DirectMemoryCache>,
    /// `/arm-io/mcc` when it is an `mcc,t6020` (T602x MCC/AMCC/PTD); parsing
    /// grants no native RAM or cache permission.
    pub t6020_memory: Option<T6020Memory>,
    /// The CPU clusters' PMGR registers, when the DeviceTree shows the SoC's
    /// PMGR and the SoC row describes them.
    pub clusters: Vec<CpuCluster>,
    /// The always-on eFuse block, under the same conditions.
    pub aon_fuses: Option<(u64, u64)>,
    /// The T6020 PMGR voltage-rail leakage calibration words.
    pub rail_calibration: Option<u64>,
    /// Source-checked T6020 complex status and DVMR boot reads.
    pub t6020_perf_boot: Vec<u64>,
    /// T6020 PMGR physical ranges with unsupported register effects.
    pub t6020_unmodeled_pmgr: Vec<(u64, u64)>,
    /// CPU/CPM energy banks selected by `hw-dpe-reg`, in absolute guest PA.
    pub energy: Vec<EnergyBank>,
    /// SOC configuration banks, bound to translated PMGR `reg` entries.
    pub power_banks: Vec<PowerBank>,
    /// SOC DPE controls and absolute DT energy samples.
    pub soc_dpe: Option<SocDpe>,
    /// DT bridge maps, selected counter configurations and device dependencies.
    pub bridges: Vec<BridgeBank>,
    pub cluster_events: Vec<Vec<u8>>,
    /// AON/SOC/cluster performance banks and DT-selected counters.
    pub performance: Vec<PerfBank>,
    /// AOP wake masks and reporting source IDs.
    pub wake_masks: Option<WakeBank>,
    /// The CPU start registers' guest PA and the clusters they start, where
    /// the SoC row places them (`power_registers::cpu_start`).
    pub cpu_start: Option<(u64, u8)>,
    /// Visible SPMI controllers and the PMUs on them.
    pub spmis: Vec<SpmiController>,
    /// The `dcpext` display coprocessor and its register apertures, when the
    /// DeviceTree shows them (admission is closed; see [`DcpController`]).
    pub dcp: Option<DcpController>,
}

impl Board {
    /// The board firmware images its devices run, by guest file name, each
    /// once: what a boot bundle must carry for the board's devices.
    pub fn firmware_files(&self) -> Vec<String> {
        let mut files: Vec<String> = Vec::new();
        for target in self.i2cs.iter().flat_map(|i2c| &i2c.targets) {
            if target.compatible != HPM {
                continue;
            }
            if let Some(personality) = &target.firmware {
                let file = hpm_firmware_file(personality);
                if !files.contains(&file) {
                    files.push(file);
                }
            }
        }
        files
    }
}

/// A node's one `reg` range, translated through contiguous `arm-io` ranges.
fn single_range(
    node: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<(u64, u64)> {
    let pairs = apertures::registers(node).ok_or(refuse!("device reg missing"))?;
    let [(offset, bytes)] = pairs[..] else {
        return Err(refuse!("device requires one reg range"));
    };
    let physical =
        translate(offset, bytes).ok_or(refuse!("device range is not wholly translated"))?;
    Ok((physical, bytes))
}

/// The opcode of a `function-*` property naming a GPIO pin: the FourCC
/// `GPIO`, first character in the high byte, as the ADT cells store it (the
/// J473AP `/arm-io/spi1` `function-spi_cs0` bytes `4f 49 50 47`).
pub const GPIO_FUNCTION: u32 = u32::from_be_bytes(*b"GPIO");

/// One translation instance, keeping its native logical number and the
/// physical `reg` ordinal. Legacy 12-byte descriptors contain kind+owner;
/// their logical number counts translation occurrences. Modern 16-byte
/// descriptors additionally encode the logical number. Owner bytes never
/// select application behavior or DMA routing.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct DartInstance {
    pub physical: u64,
    pub bytes: u64,
    pub number: u32,
    pub reg_index: usize,
    pub owner: [u8; 8],
}

/// A DT DART group, its independently programmed TRAD windows, mapper stream
/// IDs and optional APF. Modern `sid` is a list of u32 SIDs; legacy `sids`
/// is a bitmap. Neither selects a translation instance. Modern `remap`
/// contains byte source/destination pairs, with 0/0 padding ignored
/// (0x2f11864..0x2f11938). They describe routing, not permission to bypass
/// the DART. Per-instance DMA selection remains the bus master's contract.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Iommu {
    pub profile: crate::dart::Profile,
    pub instances: Vec<DartInstance>,
    pub streams: u16,
    pub irq: u32,
    pub mappers: Vec<(u32, u32)>,
    pub apf: Option<(u64, usize)>,
    /// Separate original die-checked PMGR clock and power requirements.
    pub clock_gates: Vec<DomainId>,
    pub power_gates: Vec<DomainId>,
    /// A kept DART preserves ordinary state while its supply is gated.
    pub retention: bool,
    pub sids: Vec<u32>,
    pub remap: Vec<(u8, u8)>,
    /// The base of the I/O window its mappers allocate in (`vm-base`).
    pub vm_base: u64,
    /// Its SMMU instance's register block, an `instance` descriptor of kind
    /// `UMMS` after its TRAD instances (J473AP `dart-dispext0`).
    /// `AppleT8110DART::_smmuSetupInstance` (26A434 KC file 0x2f12b64) maps
    /// that numbered `reg` entry and records it, and `_smmuSetup` (0x2f12af0)
    /// marks the DART as having one. The kernel reads it for fault capture
    /// (`_smmuCaptureHardwareState` 0x2f1a3ec, `_smmuCaptureRegs` 0x2f1d2b4),
    /// and SPTM validates its capture count during startup (file 0xcabb8).
    /// The paired native cacheless view reports zero capture words and shares
    /// its DART's lifecycle; absent selector/data accesses refuse by name.
    pub smmu: Option<(u64, u64)>,
}

/// The GPU's firmware mapper's compatible.
pub const GFX_MAPPER: &str = "iommu-mapper,gfx";

/// The `name` of an `/arm-io` child, for messages.
fn name_of(node: &Node<'_>) -> String {
    node.property("name")
        .and_then(|name| core::str::from_utf8(name).ok())
        .map_or_else(|| "?".into(), |name| name.trim_end_matches('\0').into())
}

/// The watchdog's side registers, each one aligned word outside its timer
/// window, as `AppleARMWatchdogTimer::start` (26A434 file 0x1ca362c) maps
/// device memory 1..=4: the panic-save scratch, the mechanism command, the
/// AP watchdog configuration and the panic force-power-off flags
/// (`panic-forcepoweroff-flag-bit`); the last two are optional. `triggers`
/// are the `trigger-config` records it maps per CPU: a physical word, a
/// mask and the value it sets under that mask.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct WatchdogSide {
    pub words: Vec<u64>,
    pub triggers: Vec<(u64, u32, u32)>,
}

fn watchdog_side(
    node: &Node<'_>,
    translate: &dyn Fn(u64, u64) -> Option<u64>,
) -> Result<Option<WatchdogSide>> {
    let reg = apertures::registers(node).ok_or(refuse!("watchdog has no reg"))?;
    if reg.len() == 1 {
        return Ok(None);
    }
    let bit = |name| node.first_u32(name).is_some_and(|bit| bit < 32);
    if !(3..=5).contains(&reg.len())
        || !bit("panic-save-flag-bit")
        || (reg.len() == 5 && !bit("panic-forcepoweroff-flag-bit"))
    {
        return Err(refuse!(
            "watchdog side controls need reg[1]/reg[2], valid flag bits and at most reg[4]"
        ));
    }
    let mut words = Vec::new();
    for &(address, bytes) in &reg[1..] {
        if bytes != 4 || !address.is_multiple_of(4) {
            return Err(refuse!(
                "watchdog side register is not one aligned full32 word"
            ));
        }
        let physical = translate(address, bytes)
            .ok_or(refuse!("watchdog side register is outside arm-io ranges"))?;
        words.push(physical);
    }
    let mut triggers = Vec::new();
    if let Some(records) = node.property("trigger-config") {
        let (records, rest) = records.as_chunks::<16>();
        if !rest.is_empty() {
            return Err(refuse!("watchdog trigger-config holds a partial record"));
        }
        for record in records {
            let word = |at: usize| u32::from_le_bytes(record[at..at + 4].try_into().expect("word"));
            let physical = u64::from_le_bytes(record[..8].try_into().expect("address"));
            let (mask, value) = (word(8), word(12));
            if !physical.is_multiple_of(4) || mask == 0 || value & !mask != 0 {
                return Err(refuse!(
                    "watchdog trigger-config record is unaligned or exceeds its mask"
                ));
            }
            triggers.push((physical, mask, value));
        }
    }
    Ok(Some(WatchdogSide { words, triggers }))
}

/// A CPU's CoreSight aperture; components exist only within its DT length.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CoreSight {
    pub cpu: usize,
    pub physical: u64,
    pub bytes: u64,
}

impl CoreSight {
    /// The UTTDBG region base, only when the DT aperture contains its sampler.
    pub fn pc_sampler(&self) -> Option<u64> {
        (self.bytes >= coresight::UTT_OFFSET + coresight::PC_SAMPLE + 8)
            .then(|| self.physical.checked_add(coresight::UTT_OFFSET))
            .flatten()
    }
}

/// Validate the DT aperture before placing its lock pair and, when it fits,
/// the UTTDBG PC sampler. Preserve the owner even when other PEs lack a map.
fn place_coresight(cpu: usize, pairs: &[(u64, u64)]) -> Result<CoreSight> {
    let [(physical, bytes)] = pairs else {
        return Err(refuse!("CPU CoreSight map must describe one component"));
    };
    if *bytes < coresight::STATUS + 4
        || !physical.is_multiple_of(coresight::BYTES)
        || physical.checked_add(*bytes).is_none()
    {
        return Err(refuse!(
            "CPU CoreSight component is empty, unaligned or overflows its page"
        ));
    }
    Ok(CoreSight {
        cpu,
        physical: *physical,
        bytes: *bytes,
    })
}

/// The board a DeviceTree describes, or why the engine cannot model it: its
/// chip and PEs ([`cpus`], from `/chosen` `chip-id`, `/product`
/// `product-soc-name` and each `/cpus` node with a `cluster-type`) and the
/// devices the engine models, each of which must be `compatible` with its
/// model. An `/arm-io` child's `reg` is translated through `/arm-io`
/// `ranges`; the AIC's inputs and dies are those its vectors name
/// (`aic_vectors`, `aic_inputs`).
pub fn board(tree: &[u8]) -> Result<Board> {
    let root = Node::root_exact(tree).ok_or(refuse!("DeviceTree has no root or is malformed"))?;
    let chip_id = root
        .child("chosen")
        .and_then(|chosen| chosen.first_u32("chip-id"))
        .ok_or(refuse!("DeviceTree has no /chosen chip-id"))?;
    let soc_name = root
        .child("product")
        .and_then(|product| product.property("product-soc-name"))
        .and_then(|name| core::str::from_utf8(name).ok())
        .map(|name| name.trim_end_matches('\0'))
        .ok_or(refuse!("DeviceTree has no /product product-soc-name"))?;
    let mut found = Vec::new();
    let mut blocks = Vec::new();
    let mut debug_components = Vec::new();
    root.child("cpus")
        .ok_or(refuse!("DeviceTree has no /cpus"))?
        .visit(&mut |node| {
            if let Some(cluster_type) = node.property("cluster-type") {
                if node.property("coresight-reg").is_some() {
                    debug_components.push((found.len(), node.pairs("coresight-reg")));
                }
                found.push((
                    cluster_type.first().copied(),
                    node.first_u32("reg"),
                    node.first_u32("l2-cache-size"),
                ));
                for (index, name) in ["cpu-impl-reg", "cpm-impl-reg"].into_iter().enumerate() {
                    let pe = (index == 0).then(|| found.len() - 1);
                    blocks.push((index, name, node.pairs(name), pe));
                }
            }
        })
        .ok_or(refuse!("DeviceTree walk failed"))?;
    let coresight = debug_components
        .into_iter()
        .map(|(cpu, pairs)| {
            place_coresight(
                cpu,
                &pairs.ok_or(refuse!("CPU CoreSight reg contains partial records"))?,
            )
        })
        .collect::<Result<Vec<_>>>()?;
    let processors = found
        .into_iter()
        .map(|found| match found {
            (Some(cluster_type), Some(reg), l2_bytes) => Ok(Processor {
                cluster_type,
                reg,
                l2_bytes,
            }),
            _ => Err(refuse!("a /cpus node has no reg or an empty cluster-type")),
        })
        .collect::<Result<Vec<_>, _>>()?;
    let boot_processor = boot_processor::boot_processor(&root, &processors)?;
    let cpus = cpus(chip_id, soc_name, &processors)?;
    let processor_registers = processor_registers::processor_registers(&root)?;
    if processor_registers.len() != cpus.processors.len() {
        return Err(refuse!(
            "CPU register roster differs from processor topology"
        ));
    }
    let mut counters = Vec::new();
    if let Some(layout) = crate::core_counters::layout(chip_id) {
        let end = layout
            .windows()
            .map(|(offset, bytes)| offset + bytes)
            .max()
            .unwrap_or(0);
        for (_, name, pairs, cpu) in blocks.iter().filter(|block| block.0 == 0) {
            let (physical, bytes) = pairs
                .as_ref()
                .and_then(|pairs| pairs.first().copied())
                .ok_or_else(|| refuse!("a /cpus node has no {name}", name))?;
            if end > bytes {
                return Err(refuse!(
                    "{name} at {physical:#x} has no sampler counters",
                    name,
                    physical
                ));
            }
            counters.push(CounterBlock {
                cpu: cpu.expect("every cpu-impl-reg names its PE"),
                physical,
                layout,
            });
        }
    }
    let mut lockdown: Vec<LockdownBlock> = Vec::new();
    for (index, name, pairs, cpu) in blocks.into_iter().filter(|_| cpus.lockdown.is_some()) {
        let (physical, bytes) = pairs
            .and_then(|pairs| pairs.first().copied())
            .ok_or_else(|| refuse!("a /cpus node has no {name}", name))?;
        let control = cpus.lockdown.expect("filtered")[index];
        if control + 16 > bytes {
            return Err(refuse!(
                "{name} at {physical:#x} has no lockdown registers",
                name,
                physical
            ));
        }
        if !lockdown.iter().any(|block| block.physical == physical) {
            lockdown.push(LockdownBlock {
                name,
                physical,
                bytes,
                control,
                cpu,
            });
        }
    }
    let io = root
        .child("arm-io")
        .ok_or(refuse!("DeviceTree has no /arm-io"))?;
    let translate = |offset: u64| io.translate_range(offset, 1);
    let extent = |offset: u64, bytes: u64| io.translate_range(offset, bytes);
    let place = |name: &str, model: Option<&str>| -> Result<(Node<'_>, u64, u64)> {
        let node = io
            .child(name)
            .ok_or_else(|| refuse!("DeviceTree has no /arm-io/{name}", name))?;
        if let Some(model) = model
            && !node.compatible(model)
        {
            return Err(refuse!(
                "/arm-io/{name} is not {model}, the modeled device",
                name,
                model
            ));
        }
        let (offset, bytes) = apertures::registers(&node)
            .and_then(|reg| reg.first().copied())
            .ok_or_else(|| refuse!("/arm-io/{name} has no reg", name))?;
        let physical = extent(offset, bytes)
            .ok_or_else(|| refuse!("/arm-io/{name} extent is outside /arm-io ranges", name))?;
        Ok((node, physical, bytes))
    };
    let device = |name: &str, model: &str| -> Result<Device> {
        let (node, physical, bytes) = place(name, Some(model))?;
        let irq = node
            .first_u32("interrupts")
            .ok_or_else(|| refuse!("/arm-io/{name} has no interrupts", name))?;
        Ok(Device {
            physical,
            bytes,
            irq,
        })
    };
    let uart = device("uart0", UART)?;
    let watchdog = device("wdt", WATCHDOG)?;
    let watchdog_side = watchdog_side(&io.child("wdt").expect("device was placed"), &extent)?;
    let (version, event) = cpus
        .aic
        .ok_or(refuse!("the SoC's AIC event register offset is unknown"))?;
    let (aic, physical, _) = place("aic", Some(version))?;
    let phandle = aic
        .first_u32("AAPL,phandle")
        .ok_or(refuse!("/arm-io/aic has no phandle"))?;
    let (inputs, layout) = aic_inputs(
        &aic,
        aic_layout(&aic, version, event)?,
        &aic_vectors(&root, phandle)?,
    )?;
    let chosen = root
        .child("chosen")
        .ok_or(refuse!("DeviceTree has no /chosen"))?;
    let memory_controller = memory_controller(&chosen)?;
    let io_access_controller = memory_controller_named(&chosen, "ioa")?;
    let gpio = gpio_bank(&io, &extent, phandle)?;
    let aux_gpios = aux_gpio_banks(&root, &io, &extent, phandle)?;
    let (i2cs, typec_phys) = typec_devices(&io, &extent, phandle, gpio.as_ref())?;
    let darts = iommus(&io, &extent)?;
    let aes = aes_engine(&io, &extent, &darts)?;
    let iboot_security = root
        .child("defaults")
        .is_some_and(|defaults| defaults.property("nvme-iboot-sptm-security").is_some());
    let ans = ans_controller(&io, &extent, iboot_security)?;
    let smc = smc_controller(&root, &io, &extent)?;
    let pmp = pmp_controller(&io, &extent)?;
    let gfx = gfx_controller(&io, &extent, chip_id)?;
    let sep = sep_device(&root, &io, &extent, &darts)?;
    let usbs = usb_controllers(&io, &extent, &darts)?;
    let power_states = power_states(&root, &extent, chip_id)?;
    let PmgrRegisters {
        refusals: mut pmgr_refusals,
        cluster_events,
        bridges,
        soc_dpe,
        power_banks,
        wake_masks,
        performance,
        energy,
        clusters,
        aon_fuses,
        rail_calibration,
        t6020_perf_boot,
        t6020_unmodeled_pmgr,
    } = pmgr_registers(&io, &extent, chip_id)?;
    // The native T8112 PMP image names three independent DVFS controllers.
    // Their source-checked protocol does not establish a backend.
    if chip_id == 0x8112 && pmp.is_some() {
        pmgr_refusals.extend(
            crate::pmgr::dvfs::T8112
                .into_iter()
                .map(|descriptor| PmgrBankRefusal::from(descriptor.refusal())),
        );
    }
    let cpu_start = cpu_start(&io, &extent, chip_id, &cpus.processors)?;
    let memory_cache = memory_cache(&io, &extent, chip_id, memory_controller.as_ref())?;
    let direct_memory_cache =
        memory_resources::direct_memory_cache(&io, &extent, memory_controller.as_ref())?;
    let t6020_memory = memory_resources::t6020::from_io(&io, &extent)?;
    let spi_nor = spi_nor(&root, &io, &extent, phandle, gpio.as_ref())?;
    let spmis = spmi_controllers(&io, &extent, phandle)?;
    let dcp = dcp_device(&io, &translate, &darts)?;
    Ok(Board {
        spmis,
        dcp,
        gpio,
        aux_gpios,
        i2cs,
        typec_phys,
        cluster_events,
        bridges,
        soc_dpe,
        power_banks,
        spi_nor,
        wake_masks,
        performance,
        energy,
        cpus,
        boot_processor,
        uart,
        watchdog,
        watchdog_side,
        lockdown,
        processor_registers,
        counters,
        coresight,
        memory_controller,
        io_access_controller,
        darts,
        aes,
        ans,
        smc,
        pmp,
        gfx,
        sep,
        usbs,
        power_states,
        pmgr_refusals,
        memory_cache,
        direct_memory_cache,
        t6020_memory,
        clusters,
        aon_fuses,
        rail_calibration,
        t6020_perf_boot,
        t6020_unmodeled_pmgr,
        cpu_start,
        aic: InterruptController {
            physical,
            inputs,
            layout,
        },
    })
}

#[cfg(test)]
mod tests;

#[cfg(any(test, feature = "test-support"))]
pub mod fixture_support;
