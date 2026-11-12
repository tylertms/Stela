//! The Apple T8112 device models as the engine's
//! [`Devices`]: one [`Bus`] for the guest physical address space, an
//! [`InterruptLatch`] for the wires, AIC input forwarding and reset requests.
//! The models own every register decision; this adapter only routes accesses
//! and time, and turns each `MmioError` into an explicit refusal.
mod buses;
mod call_gates;
mod coprocessors;
mod memory_control;
mod power;
mod processors;
pub mod t6000_pmgr;
mod timekeeping;
mod transfer_times;
mod usb;
pub use transfer_times::{PartTime, TRANSFER_PARTS};

use crate::Result;
pub use crate::pmu::{RtcAnchor, RtcEpoch};
use crate::usb::{
    Function, Speed,
    ecm::Ecm,
    hci::Hci,
    hid::Peripheral,
    hub::{self, Hub},
    session::Session as UsbSession,
    storage::MassStorage,
};
pub use crate::usb::{
    ecm::Counters as UsbNetworkCounters,
    hci::{Counters as UsbBluetoothCounters, Outbound},
    host::{Descriptor as UsbDescriptor, MuxEndpoints},
    mux::{
        Backpressure as MuxBackpressure, State as MuxState, StreamId as MuxStream,
        StreamState as MuxStreamState,
    },
    session::Log as UsbLog,
};
use alloc::{boxed::Box, format, string::String, sync::Arc, vec, vec::Vec};
use core::cell::Cell;
use stela_compiler::aarch64::system::registers::Encoding;
use stela_contracts::refuse;
use stela_drivers::{block::BlockDevice, input::Input, net::Mac};
use stela_engine::machine::engine::ConsumedPair;
use stela_engine::machine::engine::DeviceError;
use stela_engine::machine::engine::Devices;
use stela_engine::machine::engine::LocalRegisters;
use stela_engine::machine::engine::Memory;
use stela_engine::machine::engine::PassiveRegisters;
use stela_engine::machine::engine::Reset;
use stela_engine::machine::engine::Start;
use stela_engine::machine::engine::Transition;
use stela_engine::machine::engine::UnlockedDma;
use stela_engine::machine::registers::Lines;
use stela_engine::system::access::AccessKind;
use stela_soc_data::board::PowerOwner;
pub use usb::POWER_BUTTON_HELD;

use crate::aes;
use crate::aes::Aes;
use crate::agx::firmware::Coprocessor as GfxCoprocessor;
use crate::agx::sgx::Sgx;
use crate::agx::uat;
use crate::agx::uat::Agent;
use crate::aic::Aic2;
use crate::aic1::Aic1;
use crate::amcc::Amcc;
use crate::ans;
use crate::ans::Ans;
use crate::asc;
use crate::asc::Asc;
use crate::atc_phy;
use crate::atc_phy::AtcPhy;
use crate::bridge::Bridge;
use crate::cluster;
use crate::cluster::Cluster;
use crate::core_counters::CoreCounters;
use crate::coresight;
use crate::coresight::PcSampler;
use crate::coresight::SoftwareLock;
use crate::cpm::Cpm;
use crate::cpustart::CpuStart;
use crate::dart;
use crate::dart::Dart;
use crate::dcp;
use crate::display as display_hw;
use crate::dpe::Dpe;
use crate::dwc3;
use crate::dwc3::Dwc3;
use crate::dwc3::PortState;
use crate::energy;
use crate::energy::Energy;
use crate::fuse;
use crate::fuse::AonFuses;
use crate::gpio::Gpio;
use crate::gpio::Net;
use crate::gpio::OpenDrain;
use crate::gpu_gate;
use crate::hpm::Firmware;
use crate::hpm::Hpm;
use crate::hpm::Summary;
use crate::i2c;
use crate::i2c::I2c;
use crate::ipi;
use crate::ipi::Affinity;
use crate::ipi::FastIpi;
use crate::lockdown;
use crate::lockdown::Lockdown;
use crate::perf::PerfCounters;
use crate::pmgr::ACTIVE;
use crate::pmgr::DomainState;
use crate::pmgr::POWER_GATED;
use crate::pmgr::Pmgr;
use crate::pmgr::PowerState;
use crate::pmp;
use crate::pmu;
use crate::pmu::Pmu;
use crate::power::Power;
use crate::rtkit::RtKit;
use crate::sart;
use crate::sart::Sart;
use crate::sep;
use crate::sep::Sep;
use crate::smc;
use crate::smc::Smc;
use crate::spi::Spi;
use crate::spmi::Spmi;
use crate::uart::S5lUart;
use crate::wake;
use crate::wake::WakeMasks;
use crate::wdt;
use crate::wdt::AppleWdt;
use stela_contracts::bus::Access;
use stela_contracts::bus::Bus;
use stela_contracts::bus::DeviceId;
use stela_contracts::bus::LifecycleState;
use stela_contracts::bus::MapError;
use stela_contracts::bus::MmioDevice;
use stela_contracts::bus::MmioError;
use stela_contracts::bus::Width;
use stela_contracts::interrupt::CpuIndex;
use stela_contracts::interrupt::InterruptLatch;
use stela_soc_data::aic::Aic2Config;
use stela_soc_data::pmgr::coverage;
use stela_soc_data::pmgr::coverage::Consumer;
use stela_soc_data::pmgr::coverage::LifecycleSinks;
use {stela_contracts::dma::Physical, stela_contracts::dma::Port};

mod canonical_reads;
mod canonical_writes;
mod display_resources;
pub mod memory_resources;
mod pmp_dma;
mod pmp_services;
pub use pmp_dma::{ExplicitRuntimeConfiguration, SourceProvenance};
mod processor_power;
mod service;
mod sleep;
mod t8112_memory;
pub use service::{AicDelivery, ServiceBatch, ServiceInput};
pub use sleep::PowerLog;
pub use t8112_memory::cold as t8112_cold;

#[cfg(test)]
use stela_soc_data::aic::AIC1_EVENT;
use stela_soc_data::aic::AicLayout;
pub use {
    crate::asc::Lines as AscLines, crate::dcp::Log as DisplayLog, crate::nor,
    crate::pmp::native as native_pmp, crate::pmp::native::services::Clock as NativePmpClock,
    crate::scanout::ChannelMasks as ScanoutMasks, crate::scanout::Mode as ScanoutMode,
    crate::scanout::PixelFormat as ScanoutFormat, crate::scanout::Scanout,
    crate::sep::Entropy as SepEntropy,
    crate::sep::image::signature::Evidence as SepSignatureEvidence,
    crate::sep::image::signature::ManifestEvidence as SepManifestEvidence,
    crate::sep::image::signature::inspect as inspect_sep_signature,
    crate::sep::image::signature::verify_manifest as verify_sep_manifest_signature,
    crate::sep::state as sep_state, crate::smc::WriteLog as SmcWriteLog, crate::typec,
    crate::uart::ConsoleBusy, crate::uart::ConsoleSink, stela_contracts::interrupt::AicInput,
};

/// One guest physical range of a device: DeviceTree `reg` translated
/// through the parent `ranges`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct Range {
    pub physical: u64,
    pub bytes: u64,
}

pub struct AppleDevices {
    bus: Bus,
    latch: InterruptLatch,
    aic: Option<Controller>,
    reset: Option<Reset>,
    service: ServiceBatch,
    memory_controller: Option<DeviceId>,
    /// DCS/PTD storage is separate from the AMCC lock controller.
    memory_publication: Option<DeviceId>,
    ipi: Option<FastIpi>,
    masters: Vec<Master>,
    coprocessors: Vec<(&'static str, DeviceId)>,
    /// The SPI controller holding the NOR die.
    nor: Option<DeviceId>,
    /// StelaGPU's call gates (`gpu_gate`), a GPU's each.
    gpu_gates: Vec<DeviceId>,
    storage_masters: Vec<(DeviceId, DeviceId)>,
    /// The GFX coprocessor's ASC and the `sgx` block whose UAT it masters
    /// memory through.
    gfx_masters: Vec<(DeviceId, DeviceId)>,
    sep_masters: Vec<SepMaster>,
    dcp_masters: Vec<DcpMaster>,
    /// A DCP presented a frame since the host's service was last asked for
    /// ([`Devices::take_host_service`]): its scanout may keep the frame for
    /// a processor with nothing else to do to put on the screen.
    frame_presented: bool,
    /// One wired-OR owner per physical DART input, across logical groups.
    dart_irqs: Vec<(AicInput, dart::SharedIrq)>,
    usb_masters: Vec<UsbMaster>,
    cpu_start: Option<CpuStarter>,
    reset_vectors: Vec<Option<DeviceId>>,
    /// Each PE's CLPC sampler counters (`platform::apple::core_counters`),
    /// by PE number.
    core_counters: Vec<Option<DeviceId>>,
    pc_samplers: Vec<(usize, u64, DeviceId)>,
    /// The SoC watchdogs, whose mechanism writes the reports show.
    watchdogs: Vec<DeviceId>,
    starts: Vec<Start>,
    selects: Vec<SelectWire>,
    /// The AP GPIO bank's base and the board nets on its input pins.
    gpio: Option<(u64, Vec<(u32, Net)>)>,
    /// The other GPIO banks' phandles and the board nets on their client
    /// pins.
    aux_gpios: Vec<(u32, Vec<(u32, Net)>)>,
    typec_ports: Vec<(u32, typec::Port)>,
    phys: Vec<PhyBinding>,
    pmgr: Option<DeviceId>,
    power_clients: Vec<PowerClient>,
    power_devices: Vec<PowerDevice>,
    power_refusals: Vec<(u64, String)>,
    power_configured: bool,
    processor_power: Option<processor_power::AssignedPower>,
    native_pmp_dma: Option<pmp_dma::PmpDma>,
    native_pmp_services: Option<pmp_services::NativePmpServices>,
    /// The SoC's sleep and wake (`sleep`).
    sleep: sleep::SocSleep,
    rtc_clock: Option<pmu::RtcClock>,
    /// Every PMU's oscillator, the RTC's among them.
    pmu_clocks: Vec<pmu::RtcClock>,
    /// The host's wall time and the machine state's anchor this boot's
    /// upcount started from, for the anchors it keeps (`rtc_anchor`).
    rtc_origin: Option<(
        Option<stela_soc_data::pmu::WallClock>,
        Option<pmu::RtcAnchor>,
    )>,
    /// The latest count a board entry was given, and the entry; the first
    /// entry given an earlier count, with it, for the stop it causes.
    time_seen: Cell<(u64, &'static str)>,
    time_regression: Cell<Option<[(u64, &'static str); 2]>>,
    /// The inline key engine the SEP provisions and ANS unwraps keys with.
    inline_keys: crate::inline_key::Engine,
    /// The board's ECID once the SEP is placed: the modeled Bluetooth
    /// controller's address derives from it.
    ecid: u64,
    /// Each transfer part's host time, while the host gives a clock.
    transfer_times: transfer_times::TransferTimes,
    /// The registers PEs read without the devices' lock.
    passive: crate::passive::PassiveMap,
    /// Each PE's last device write, as address and bytes, and the one whose
    /// DMA the current transfer runs ([`Devices::settle_write`]).
    last_write: Vec<Option<(u64, u8)>>,
    writer: Option<(u64, u8)>,
    /// The SoC's sleep when [`Devices::take_pins_changed`] last ran; `None`
    /// before its first call.
    pins_asleep: Option<bool>,
    /// Whether DMA data copies are left to [`Devices::take_unlocked`].
    defer_dma: bool,
    /// The displays' composes left since the last
    /// [`Devices::take_unlocked`] ([`dcp::compose`]).
    composes: Vec<dcp::compose::Compose>,
}

/// The host polls its USB sessions once a high-speed microframe, in which
/// a bulk pipe moves at most 13 512-byte packets (USB 2.0 Table 5-10).
const USB_POLL_TICKS: u64 = crate::counter::FREQUENCY_HZ / 8000;
const USB_PACKETS_PER_MICROFRAME: usize = 13;
/// The microframes a late poll catches up, a frame's: this bounds one poll's
/// work under the devices' lock.
const USB_BACKLOG_MICROFRAMES: u64 = 8;

/// A due poll's packet budget and the next boundary. A late poll catches
/// up at most one frame while keeping the microframe clock's phase.
fn usb_poll(now: u64, due: u64) -> (usize, u64) {
    let late = now - due;
    let microframes = (late / USB_POLL_TICKS + 1).min(USB_BACKLOG_MICROFRAMES);
    let next = now.saturating_add(USB_POLL_TICKS - late % USB_POLL_TICKS);
    (microframes as usize * USB_PACKETS_PER_MICROFRAME, next)
}
/// A native host handle, owned by this AppleDevices instance.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct UsbId(DeviceId);
struct PhyBinding {
    phandle: u32,
    port: u32,
    device: DeviceId,
    retention: bool,
    pipe: Option<(u64, u64)>,
    events: (u64, u64),
}
struct UsbMaster {
    device: DeviceId,
    phy: DeviceId,
    phy_changed: bool,
    dart: DeviceId,
    stream: usize,
    session: UsbSession,
    port: UsbPort,
    /// The Type-C connector's `port-number` and shared cable state.
    number: u32,
    connector: typec::Port,
    /// The hub cabled to the connector, with the input peripheral and the
    /// host's disks behind it, when this port is the downstream-facing
    /// (host) side.
    peripheral: Option<Hub<Attachments>>,
    next_poll: u64,
    attached: bool,
    epoch: u64,
    retention: bool,
    session_reset: bool,
}
impl UsbMaster {
    fn cancel_session(&mut self, reset: bool) -> Result<()> {
        self.session_reset |= reset;
        if self.session_reset {
            self.session.detach()?;
            self.attached = false;
            self.next_poll = 0;
            self.session_reset = false;
        }
        Ok(())
    }
}

#[derive(Clone, Copy, PartialEq, Eq)]
struct UsbPort {
    generation: u64,
    ready: bool,
}
impl UsbPort {
    /// Device-mode readiness from the connector: attached as UFP with VBUS.
    fn of(connector: &typec::Port) -> Self {
        let snapshot = connector.snapshot();
        Self {
            generation: snapshot.generation,
            ready: snapshot
                .connection
                .is_some_and(|c| c.role == typec::DataRole::Ufp && c.vbus),
        }
    }
}
/// The USB2 function a downstream-facing attach with VBUS cables to the
/// host personality, by its speed.
/// What the native hub carries besides the keyboard and pointer.
#[derive(Default)]
pub struct UsbCables {
    /// Host disks, each a USB mass storage device (at most five with a
    /// network adapter, six without).
    pub disks: Vec<Box<dyn BlockDevice>>,
    /// A USB Ethernet adapter with this station address.
    pub network: Option<Mac>,
    /// A USB Bluetooth controller: bridged to one of the host's, or the
    /// model the owner selects with [`AppleDevices::bluetooth_model`].
    pub bluetooth: bool,
}

impl UsbCables {
    /// The disks the hub has ports for beside the keyboard and pointer and,
    /// with `network` and `bluetooth`, the Ethernet adapter and Bluetooth
    /// controller.
    pub const fn disk_ports(network: bool, bluetooth: bool) -> usize {
        hub::MAX_PORTS - 1 - network as usize - bluetooth as usize
    }
}

/// The functions on the native hub's ports: the keyboard and pointer on
/// port 1, then the Ethernet adapter and Bluetooth controller when there are
/// any, then one mass storage device per host disk.
struct Attachments {
    input: Peripheral,
    network: Option<Ecm>,
    bluetooth: Option<Hci>,
    disks: Vec<MassStorage>,
}

impl hub::Ports for Attachments {
    fn count(&self) -> usize {
        1 + usize::from(self.network.is_some())
            + usize::from(self.bluetooth.is_some())
            + self.disks.len()
    }
    fn speed(&self, port: usize) -> Speed {
        if port == 0 { Speed::Full } else { Speed::High }
    }
    fn function(&mut self, port: usize) -> &mut dyn Function {
        let input = core::iter::once(&mut self.input as &mut dyn Function);
        let network = self.network.iter_mut().map(|f| f as &mut dyn Function);
        let bluetooth = self.bluetooth.iter_mut().map(|f| f as &mut dyn Function);
        let disks = self.disks.iter_mut().map(|f| f as &mut dyn Function);
        let mut functions = input.chain(network).chain(bluetooth).chain(disks);
        functions.nth(port).expect("a port of the hub's count")
    }
    fn function_ref(&self, port: usize) -> &dyn Function {
        let input = core::iter::once(&self.input as &dyn Function);
        let network = self.network.iter().map(|f| f as &dyn Function);
        let bluetooth = self.bluetooth.iter().map(|f| f as &dyn Function);
        let disks = self.disks.iter().map(|f| f as &dyn Function);
        let mut functions = input.chain(network).chain(bluetooth).chain(disks);
        functions.nth(port).expect("a port of the hub's count")
    }
}

fn host_function(connector: &typec::Port) -> Option<typec::Usb2Speed> {
    connector
        .snapshot()
        .connection
        .filter(|c| c.role == typec::DataRole::Dfp && c.vbus)
        .map(|c| c.speed)
}

impl PortState for UsbPort {
    fn generation(&self) -> u64 {
        self.generation
    }
    fn high_speed_device_ready(&self) -> bool {
        self.ready
    }
}

struct SelectWire {
    spi: DeviceId,
    physical: u64,
}

/// The CPU start registers and each PE's MPIDR_EL1 (Aff1 its cluster, Aff0
/// its core), by which a released core finds its PE.
struct CpuStarter {
    device: DeviceId,
    mpidrs: Vec<u64>,
}

/// Supplies shared by one DART, with its hardware retention property.
struct PowerDevice {
    physical: u64,
    device: DeviceId,
    retention: bool,
}

struct PowerClient {
    resources: String,
    device: DeviceId,
    owner: PowerOwner,
    domains: Vec<(u64, DomainState)>,
    retention: bool,
}

/// An AES engine and the DART stream its DMA goes through.
#[derive(Debug, Clone, Copy)]
struct Master {
    aes: DeviceId,
    dart: DeviceId,
    stream: usize,
}

/// A SEP and the DT-selected DART stream carrying its firmware DMA.
struct SepMaster {
    sep: DeviceId,
    dart: DeviceId,
    stream: usize,
}

/// The per-machine inputs the SEP binds its own trust and key state to: the
/// AP's effective security mode (shared fuses), the ECID (`/chosen`
/// `unique-chip-id`), and where its durable state lives.
pub struct SepProvisioning {
    pub security: Option<stela_soc_data::sep::SecurityMode>,
    pub ecid: u64,
    /// The kext grant its macOS LocalPolicies carry.
    pub kexts: Option<stela_formats::local_policy::Kexts>,
    /// Observe (`--sep-observe`): advertise and log `sks`/`pnon`/`scrd`.
    pub observe: bool,
    pub state: sep::state::Location,
    /// The recoveryOS policy nonce its first proposal takes, from an install
    /// bundle's staged signing answers; every other one is drawn.
    pub recovery_policy_nonce: Option<[u8; 32]>,
}

/// The DCPEXT coprocessor's ASC, the DART stream it masters its shared
/// memory through, and the display pipe's DART stream its swap surfaces are
/// mapped in, once placed.
struct DcpMaster {
    asc: DeviceId,
    dart: DeviceId,
    stream: usize,
    surface: Option<(DeviceId, usize)>,
}

/// A DART on the bus, for [`AppleDevices::add_aes`].
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct DartId(DeviceId);

/// The mapped AIC and which model it is.
#[derive(Debug, Clone, Copy)]
enum Controller {
    V1(DeviceId),
    V2(DeviceId),
}

fn describe(error: MmioError) -> DeviceError {
    match error {
        MmioError::Unmapped { .. } => DeviceError::NoDevice,
        MmioError::Misaligned { address, width } => DeviceError::Refused(refuse!(
            "misaligned {bytes}-byte device access at {address:#x}",
            bytes = width.bytes(),
            address
        )),
        MmioError::Unsupported {
            device,
            offset,
            width,
            direction,
            reason,
        } => DeviceError::Refused(
            refuse!(
                "{device} {direction} of {bytes} bytes at offset {offset:#x}",
                device,
                ?direction,
                bytes = width.bytes(),
                offset
            )
            .because(reason),
        ),
    }
}

fn describe_transfer(error: MmioError) -> DeviceError {
    match describe(error) {
        DeviceError::NoDevice => DeviceError::TransferRefused(refuse!("unmapped DMA service")),
        DeviceError::Refused(reason) | DeviceError::TransferRefused(reason) => {
            DeviceError::TransferRefused(reason)
        }
    }
}

/// Native client boundary: enqueues bytes, exposes backpressure, and never
/// executes guest code. Advance the machine between calls while connecting.
pub struct UsbClient<'a>(&'a mut UsbSession);
#[cfg(any(test, feature = "test-support"))]
impl<'a> UsbClient<'a> {
    pub fn test_session(session: &'a mut UsbSession) -> Self {
        Self(session)
    }
}
impl UsbClient<'_> {
    pub fn mux_state(&self) -> MuxState {
        self.0.mux_state()
    }
    pub fn backpressure(&self) -> MuxBackpressure {
        self.0.backpressure()
    }
    pub fn control_ready(&self) -> bool {
        self.0.control_ready()
    }
    pub fn open(&mut self, port: u16) -> Result<MuxStream> {
        self.0.open(port)
    }
    pub fn state(&self, stream: MuxStream) -> Result<MuxStreamState> {
        self.0.state(stream)
    }
    pub fn write(&mut self, stream: MuxStream, bytes: &[u8]) -> Result<usize> {
        self.0.write(stream, bytes)
    }
    pub fn read(&mut self, stream: MuxStream, bytes: &mut [u8]) -> Result<usize> {
        self.0.read(stream, bytes)
    }
    pub fn close(&mut self, stream: MuxStream) -> Result<()> {
        self.0.close(stream)
    }
    pub fn release(&mut self, stream: MuxStream) -> Result<()> {
        self.0.release(stream)
    }
    pub fn query_type(&mut self) -> Result<()> {
        self.0.query_type()
    }
}

impl Default for AppleDevices {
    fn default() -> Self {
        Self::new()
    }
}

/// A DCPEXT nub as the board describes it: its register aperture, AIC
/// interrupts, whether it starts quiesced, and its role and identity.
pub struct DcpextNub {
    pub wrapper: Range,
    pub interrupts: [u32; 4],
    pub quiesced: bool,
    pub role: &'static str,
    pub identity: stela_soc_data::dcp::ExternalIdentity,
}

/// The DART streams a DCPEXT masters: its link region's stream, the
/// `window` its firmware maps through it, and the display pipe's surface
/// stream when it is placed.
pub struct DcpextDma {
    pub link: (DartId, usize),
    pub window: Option<u64>,
    pub surface: Option<(DartId, usize)>,
}

impl AppleDevices {
    #[cfg(feature = "native-profile")]
    pub fn advance_statistics(&self) -> stela_contracts::bus::AdvanceStatistics {
        self.bus.advance_statistics()
    }

    /// The board's devices with PE zero receiving the boot handoff.
    pub fn new() -> Self {
        Self::for_boot_processor(0)
    }

    /// The board's existing PE receiving the boot handoff. PE indices keep
    /// DeviceTree order; the selected index also owns CPU_STS and SoC wake.
    /// The mount validates it against the same roster passed to the engine.
    pub fn for_boot_processor(boot_processor: usize) -> Self {
        Self {
            bus: Bus::new(),
            latch: InterruptLatch::new(),
            aic: None,
            reset: None,
            service: ServiceBatch::default(),
            memory_controller: None,
            memory_publication: None,
            ipi: None,
            masters: Vec::new(),
            storage_masters: Vec::new(),
            gfx_masters: Vec::new(),
            coprocessors: Vec::new(),
            nor: None,
            gpu_gates: Vec::new(),
            sep_masters: Vec::new(),
            dcp_masters: Vec::new(),
            frame_presented: false,
            dart_irqs: Vec::new(),
            usb_masters: Vec::new(),
            cpu_start: None,
            reset_vectors: Vec::new(),
            core_counters: Vec::new(),
            pc_samplers: Vec::new(),
            watchdogs: Vec::new(),
            starts: Vec::new(),
            selects: Vec::new(),
            gpio: None,
            aux_gpios: Vec::new(),
            typec_ports: Vec::new(),
            phys: Vec::new(),
            pmgr: None,
            power_clients: Vec::new(),
            power_devices: Vec::new(),
            power_refusals: Vec::new(),
            power_configured: false,
            processor_power: None,
            native_pmp_dma: None,
            native_pmp_services: None,
            sleep: sleep::SocSleep::for_boot_processor(boot_processor),
            rtc_clock: None,
            pmu_clocks: Vec::new(),
            rtc_origin: None,
            time_seen: Cell::new((0, "start")),
            time_regression: Cell::new(None),
            inline_keys: crate::inline_key::Engine::default(),
            ecid: 0,
            transfer_times: transfer_times::TransferTimes::default(),
            passive: crate::passive::PassiveMap::default(),
            last_write: Vec::new(),
            writer: None,
            pins_asleep: None,
            defer_dma: false,
            composes: Vec::new(),
        }
    }

    /// Note PE `cpu`'s device write, whose DMA its next
    /// [`Devices::settle_write`] runs.
    fn note_write(&mut self, cpu: usize, physical: u64, bytes: u8) {
        if self.last_write.len() <= cpu {
            self.last_write.resize(cpu + 1, None);
        }
        self.last_write[cpu] = Some((physical, bytes));
    }

    /// Time each part of every transfer ([`TRANSFER_PARTS`]) with `clock`,
    /// the host's counter, from now on; `None` stops.
    pub fn time_transfers(&mut self, clock: Option<fn() -> u64>) {
        self.transfer_times.set_clock(clock);
    }

    /// Each transfer part's calls, host ticks and longest call since the
    /// last call, by [`TRANSFER_PARTS`].
    pub fn take_transfer_times(&mut self) -> [PartTime; TRANSFER_PARTS.len()] {
        self.transfer_times.take()
    }

    /// The NOR sectors the guest changed since the last call, as (offset,
    /// bytes), for the host to persist.
    pub fn nor_dirty(&mut self) -> Vec<(usize, Vec<u8>)> {
        self.nor
            .and_then(|id| self.bus.device_mut::<Spi>(id))
            .map_or_else(Vec::new, Spi::take_dirty)
    }

    /// A copy of the NOR image as the guest left it, for the host to change
    /// NVRAM once the guest has stopped.
    pub fn nor_image(&mut self) -> Option<stela_soc_data::nor::Image> {
        self.nor
            .and_then(|id| self.bus.device_mut::<Spi>(id))
            .map(|spi| spi.image().clone())
    }

    /// Drain the `sks`/`pnon`/`scrd` traffic the SEP observed this run under
    /// `--sep-observe`, as log lines. Empty when observation is off.
    pub fn sep_observations(&mut self) -> Vec<String> {
        let ids: Vec<DeviceId> = self.sep_masters.iter().map(|master| master.sep).collect();
        let mut lines = Vec::new();
        for id in ids {
            if let Some(sep) = self.bus.device_mut::<Sep>(id) {
                lines.extend(sep.observations());
            }
        }
        lines
    }

    /// The coprocessor requests answered with an error reply for the first
    /// time since the last call, as log lines naming the coprocessor.
    pub fn take_unmodeled(&mut self) -> Vec<String> {
        let mut lines = Vec::new();
        let ids: Vec<(&'static str, DeviceId)> = self.coprocessors.clone();
        for (role, id) in ids {
            if let Some(asc) = self.bus.device_mut::<Asc>(id) {
                lines.extend(
                    asc.take_unmodeled()
                        .into_iter()
                        .map(|l| format!("{role} {l}")),
                );
            }
        }
        let ids: Vec<DeviceId> = self.sep_masters.iter().map(|master| master.sep).collect();
        for id in ids {
            if let Some(sep) = self.bus.device_mut::<Sep>(id) {
                lines.extend(sep.take_unmodeled().into_iter().map(|l| format!("SEP {l}")));
            }
        }
        lines
    }

    /// State of native RTKit coprocessors, without register side effects.
    pub fn coprocessor_diagnostics(&self) -> Vec<String> {
        self.coprocessors
            .iter()
            .map(|&(role, id)| {
                let asc = self.bus.device::<Asc>(id).expect("coprocessor wrapper");
                format!("{role} {}", asc.diagnostic())
            })
            .collect()
    }

    /// Each storage controller's owned commands and queues at counter `now`
    /// ([`Ans::diagnostic`]).
    pub fn storage_diagnostics(&self, now: u64) -> Vec<String> {
        self.storage_masters
            .iter()
            .filter_map(|&(ans, _)| Some(self.bus.device::<Ans>(ans)?.diagnostic(now)))
            .collect()
    }

    fn map(&mut self, device: DeviceId, ranges: &[Range]) -> Result<(), MapError> {
        let mut offset = 0;
        for range in ranges {
            self.bus.map(device, range.physical, range.bytes, offset)?;
            offset += range.bytes;
        }
        Ok(())
    }
}

fn width(bytes: u8) -> Result<Width, DeviceError> {
    Width::from_bytes(u64::from(bytes))
        .ok_or_else(|| DeviceError::Refused(refuse!("{bytes}-byte device access", bytes)))
}

impl AppleDevices {
    /// Bring the fast IPIs and every device to `now` for PE `cpu`, which
    /// settles every PE's pins: the next count at which one changes by
    /// itself, none when an advance was refused and the machine halts, or
    /// `Err` when the board halted on the time itself.
    fn advance_to(&mut self, cpu: usize, now: u64) -> Result<Option<u64>, ()> {
        self.observe_time(now, "advance");
        if !self.publish_rtc_time_or_halt(now) {
            return Err(());
        }
        self.settle_lifecycle(cpu, now);
        let deferred = self.ipi.as_mut().and_then(|ipi| ipi.advance(now));
        let mut access = Access::new(cpu, now, &mut self.latch);
        let advanced = self.bus.advance(&mut access);
        let reset = reset_reason(&access);
        let device = advanced.as_ref().ok().copied().flatten();
        let settled = self.settle(cpu, now, reset);
        let refused = advanced.map_err(describe).and(settled).err();
        if let Some(error) = &refused
            && self.reset.is_none()
        {
            // A model that cannot advance stops the machine like a reset
            // would, with the model's own reason.
            self.reset = Some(Reset {
                transition: Transition::Halt,
                reason: format!("device advance refused: {error:?}{}", self.rtc_regression()),
            });
        }
        self.watch_pmu();
        if refused.is_some() {
            return Ok(None);
        }
        let usb = self
            .usb_masters
            .iter()
            .filter(|master| {
                (master.port.ready || master.attached)
                    && self
                        .bus
                        .device::<Dwc3>(master.device)
                        .is_some_and(Dwc3::lifecycle_active)
            })
            .map(|master| master.next_poll);
        Ok([deferred, device].into_iter().flatten().chain(usb).min())
    }
}

fn reset_reason(access: &Access<'_>) -> Option<Reset> {
    access.reset_request().map(|request| Reset {
        transition: request.transition,
        reason: format!(
            "{}: {} at counter {}",
            request.device, request.reason, request.at
        ),
    })
}

/// What a PE reaches of the board's registers outside the devices' lock:
/// its core counters, and the fast IPIs' signals as PE `cpu`.
struct PeRegisters {
    counters: Option<CoreCounters>,
    ipi: Option<(Arc<crate::ipi::Signals>, usize)>,
}

impl PeRegisters {
    /// The fast-IPI register `encoding` names, with the signals.
    fn ipi(
        &self,
        encoding: Encoding,
    ) -> Option<(
        stela_engine::machine::fast_ipi::Register,
        &crate::ipi::Signals,
        usize,
    )> {
        let (signals, cpu) = self.ipi.as_ref()?;
        let register = stela_engine::machine::fast_ipi::Register::decode(encoding)?;
        Some((register, signals, *cpu))
    }
}

impl LocalRegisters for PeRegisters {
    /// IPI_SR reads, and IPI_SR writes of 0 or PENDING: an acknowledgement
    /// clears the PE's word, which is its FIQ from the fast IPIs
    /// ([`crate::ipi::Signals::send`]).
    fn register(&self, encoding: Encoding, write: Option<u64>) -> Option<u64> {
        use stela_engine::machine::fast_ipi::Register;
        self.counters
            .as_ref()
            .and_then(|counters| LocalRegisters::register(counters, encoding, write))
            .or_else(|| {
                let (register, signals, cpu) = self.ipi(encoding)?;
                if register != Register::Status {
                    return None;
                }
                match write {
                    None => Some(signals.pending(cpu)),
                    Some(0) => Some(0),
                    Some(value) => signals.send(cpu, register, value).map(|_| 0),
                }
            })
    }

    /// Immediate fast-IPI requests to a CPU with no deferred IPI armed.
    fn send(&self, encoding: Encoding, value: u64) -> Option<Option<usize>> {
        use stela_engine::machine::fast_ipi::Register;
        let (register, signals, cpu) = self.ipi(encoding)?;
        if register == Register::Status {
            return None;
        }
        signals.send(cpu, register, value)
    }

    fn fiq(&self) -> bool {
        self.ipi
            .as_ref()
            .is_some_and(|(signals, cpu)| signals.pending(*cpu) != 0)
    }

    fn storage(&self, encoding: Encoding) -> Option<u64> {
        use stela_engine::machine::fast_ipi::Register;
        self.counters
            .as_ref()
            .and_then(|counters| counters.storage(encoding))
            .or_else(|| {
                let (register, signals, cpu) = self.ipi(encoding)?;
                (register == Register::Status)
                    .then(|| signals.storage(cpu))
                    .flatten()
            })
    }
}

impl Devices for AppleDevices {
    fn pc_sample_target(&self, physical: u64, bytes: u8) -> Option<usize> {
        self.pc_samplers
            .iter()
            .find_map(|&(cpu, register, _)| (physical == register && bytes == 4).then_some(cpu))
    }

    fn observe_pc_sample(&mut self, physical: u64, pc: u64) {
        if let Some(&(_, _, device)) = self
            .pc_samplers
            .iter()
            .find(|&&(_, register, _)| register == physical)
        {
            self.bus
                .device_mut::<PcSampler>(device)
                .expect("registered PC sampler")
                .observe(pc);
        }
    }

    fn read(&mut self, cpu: usize, physical: u64, bytes: u8, now: u64) -> Result<u64, DeviceError> {
        self.observe_time(now, "read");
        let consumed = self.consume_scalar_read(cpu, physical, bytes, now, None)?;
        consumed.settlement?;
        Ok(consumed.value)
    }

    fn requires_canonical_read(
        &self,
        _cpu: usize,
        physical: u64,
        shape: stela_engine::machine::engine::DeviceReadShape,
    ) -> bool {
        self.read_needs_ram(physical, shape)
    }

    /// CPU PMGR registers, which settle the board's processors, and the
    /// memory controller words that drain or flush to RAM.
    fn requires_canonical_write(&self, _cpu: usize, physical: u64, bytes: u8) -> bool {
        self.is_processor_power_register(physical)
            || self.read_needs_ram(
                physical,
                stela_engine::machine::engine::DeviceReadShape::Scalar { bytes },
            )
            || width(bytes).is_ok_and(|width| self.empty_cache_flush_needs_ram(physical, width))
    }

    fn read_canonical(
        &mut self,
        cpu: usize,
        physical: u64,
        bytes: u8,
        now: u64,
        memory: &mut dyn stela_engine::machine::engine::CanonicalDeviceMemory,
    ) -> Result<stela_engine::machine::engine::ConsumedScalar, DeviceError> {
        self.observe_time(now, "read");
        self.consume_scalar_read(cpu, physical, bytes, now, Some(memory))
    }

    fn read_pair_canonical(
        &mut self,
        cpu: usize,
        physical: u64,
        bytes: u8,
        now: u64,
        _memory: &mut dyn stela_engine::machine::engine::CanonicalDeviceMemory,
    ) -> Result<ConsumedPair, DeviceError> {
        self.observe_time(now, "paired read");
        // Only the Prepared publication/status owner has a checked
        // whole-pair read. Never consume two unrelated scalar registers here.
        self.read_pair(cpu, physical, bytes, now)?.ok_or_else(|| {
            DeviceError::TransferRefused(
                "canonical pair has no qualified whole-register owner".into(),
            )
        })
    }

    fn read_pair(
        &mut self,
        cpu: usize,
        physical: u64,
        bytes: u8,
        now: u64,
    ) -> Result<Option<ConsumedPair>, DeviceError> {
        self.observe_time(now, "paired read");
        // Probe without effects: both members must belong to the same
        // joint resource at consecutive offsets, not adjacent PAs.
        let Some(width) = Width::from_bytes(u64::from(bytes)) else {
            return Ok(None);
        };
        let Ok((device, offset)) = self.bus.route(physical, width) else {
            return Ok(None);
        };
        let Some(next) = physical.checked_add(u64::from(bytes)) else {
            return Ok(None);
        };
        let Some(next_offset) = offset.checked_add(u64::from(bytes)) else {
            return Ok(None);
        };
        if self.bus.route(next, width).ok() != Some((device, next_offset)) {
            return Ok(None);
        }
        if !memory_resources::resource_for(&self.bus, device)
            .is_some_and(|owner| owner.supports_pair(offset, bytes))
        {
            return Ok(None);
        }
        self.publish_rtc_time(now)?;
        self.settle_lifecycle(cpu, now);
        let mut access = Access::new(cpu, now, &mut self.latch);
        let values = memory_resources::resource_for(&self.bus, device)
            .expect("paired read retains the prepared resource")
            .read_pair_values(offset, bytes, &mut access)
            .map_err(DeviceError::Refused)?;
        let reset = reset_reason(&access);
        let settlement = self.settle(cpu, now, reset);
        Ok(Some(ConsumedPair { values, settlement }))
    }

    fn write(
        &mut self,
        cpu: usize,
        physical: u64,
        bytes: u8,
        value: u64,
        now: u64,
    ) -> Result<(), DeviceError> {
        self.observe_time(now, "write");
        self.note_write(cpu, physical, bytes);
        use stela_engine::machine::engine::ConsumedWrite;
        match self.consume_scalar_write(cpu, physical, bytes, value, now, None)? {
            ConsumedWrite::Complete { settlement } => settlement,
            ConsumedWrite::Prefix { error, .. } | ConsumedWrite::Unknown { error } => Err(error),
        }
    }

    fn write_canonical(
        &mut self,
        cpu: usize,
        physical: u64,
        bytes: u8,
        value: u64,
        now: u64,
        owners: &mut dyn stela_engine::machine::engine::CanonicalDeviceWrite,
    ) -> Result<stela_engine::machine::engine::ConsumedWrite, DeviceError> {
        self.observe_time(now, "write");
        self.note_write(cpu, physical, bytes);
        if let Some(completed) =
            self.consume_processor_power_write(cpu, physical, bytes, value, now, owners)?
        {
            return Ok(completed);
        }
        use stela_engine::machine::engine::CanonicalDeviceMemory;
        let mut memory = owners.memory()?;
        memory.fence_cpu_memory();
        self.consume_scalar_write(cpu, physical, bytes, value, now, Some(&mut memory))
    }

    fn system_register(
        &mut self,
        cpu: usize,
        encoding: Encoding,
        write: Option<u64>,
        now: u64,
    ) -> Option<Result<u64, &'static str>> {
        self.observe_time(now, "System register");
        if !self.publish_rtc_time_or_halt(now) {
            return Some(Err("RTC counter publication refused; board halted"));
        }
        if let Some(device) = self.core_counters.get(cpu).copied().flatten()
            && let Some(value) = self
                .bus
                .device_mut::<CoreCounters>(device)
                .and_then(|counters| counters.register(encoding, write))
        {
            return Some(Ok(value));
        }
        let register = stela_engine::machine::fast_ipi::Register::decode(encoding)?;
        let ipi = self.ipi.as_mut()?;
        let kind = match write {
            Some(value) => AccessKind::Write { value },
            None => AccessKind::Read,
        };
        Some(
            ipi.access(cpu, register, kind, now)
                .map_err(|error| match error {
                    ipi::IpiError::UnknownCpu(_) => "fast IPI from a PE the board does not have",
                    ipi::IpiError::Unsupported(_, reason) => reason,
                }),
        )
    }

    fn defer_dma(&mut self) {
        self.defer_dma = true;
    }

    /// The storage masters' read copies ([`crate::ans::ReadCopy`]).
    fn take_unlocked(&mut self) -> Vec<Box<dyn UnlockedDma>> {
        let mut copies: Vec<Box<dyn UnlockedDma>> = Vec::new();
        for &(ans, _) in &self.storage_masters {
            if let Some(ans) = self.bus.device_mut::<Ans>(ans) {
                copies.extend(
                    ans.take_copies()
                        .map(|copy| Box::new(copy) as Box<dyn UnlockedDma>),
                );
            }
        }
        copies.extend(
            self.composes
                .drain(..)
                .map(|compose| Box::new(compose) as Box<dyn UnlockedDma>),
        );
        copies
    }

    fn passive_registers(&self) -> Option<Arc<dyn PassiveRegisters>> {
        (!self.passive.is_empty())
            .then(|| Arc::new(self.passive.clone()) as Arc<dyn PassiveRegisters>)
    }

    fn local_registers(&self, cpu: usize) -> Option<Arc<dyn LocalRegisters>> {
        let device = self.core_counters.get(cpu).copied().flatten();
        let counters = device.and_then(|device| self.bus.device::<CoreCounters>(device));
        let ipi = self
            .ipi
            .as_ref()
            .filter(|ipi| ipi.signals().storage(cpu).is_some())
            .map(|ipi| (ipi.signals(), cpu));
        (counters.is_some() || ipi.is_some()).then(|| {
            Arc::new(PeRegisters {
                counters: counters.cloned(),
                ipi,
            }) as Arc<dyn LocalRegisters>
        })
    }

    fn serves_register(&self, encoding: Encoding) -> bool {
        self.ipi.is_some() && stela_engine::machine::fast_ipi::Register::decode(encoding).is_some()
            || self.core_counters.iter().flatten().any(|&device| {
                self.bus
                    .device::<CoreCounters>(device)
                    .is_some_and(|counters| counters.serves(encoding))
            })
    }

    fn lines(&mut self, cpu: usize, now: u64) -> Lines {
        self.lines_and_deadline(cpu, now).0
    }

    /// The latched pins: [`Self::lines`] advances every device and settles
    /// every PE's pins at once, so another PE's need no advance until a
    /// device changes or its deadline passes. A PE's FIQ from the fast IPIs
    /// is its IPI_SR, read live ([`Self::own_fiq`]).
    fn pins(&mut self, cpu: usize) -> Option<Lines> {
        if self.sleep.asleep {
            return Some(Lines::default());
        }
        let pins = self.latch.pins(cpu);
        Some(Lines {
            irq: pins.irq,
            fiq: pins.fiq,
        })
    }

    fn fast_ipi_words(&self) -> Vec<stela_engine::machine::fast_ipi::Word> {
        self.ipi
            .as_ref()
            .map(|ipi| ipi.signals().words())
            .unwrap_or_default()
    }

    /// The PE's IPI_SR, held low while the SoC sleeps, as the latch is.
    fn own_fiq(&mut self, cpu: usize) -> bool {
        !self.sleep.asleep && self.ipi.as_ref().is_some_and(|ipi| ipi.fiq(cpu))
    }

    fn take_signalled(&mut self, cpu: usize) -> bool {
        self.ipi.as_mut().is_some_and(|ipi| ipi.take_raised(cpu))
    }

    /// The latch's changed CPUs, those the fast IPIs raised, and every CPU
    /// when the SoC's sleep changed (asleep, every latched pin reads low) or
    /// on the first call.
    fn take_pins_changed(&mut self) -> u64 {
        let asleep = Some(self.sleep.asleep);
        let slept = core::mem::replace(&mut self.pins_asleep, asleep) != asleep;
        let raised = self
            .ipi
            .as_ref()
            .map_or(0, crate::ipi::FastIpi::raised_mask);
        let changed = self.latch.take_changed_pins() | raised;
        if slept { u64::MAX } else { changed }
    }

    fn deadline(&mut self, now: u64) -> Option<u64> {
        // Time-driven state is the board's, not a PE's: the access names
        // cpu0.
        self.advance_to(0, now).ok().flatten()
    }

    fn lines_and_deadline(&mut self, cpu: usize, now: u64) -> (Lines, Option<u64>) {
        match self.advance_to(cpu, now) {
            Ok(deadline) => (self.pins(cpu).unwrap_or_default(), deadline),
            Err(()) => (Lines::default(), None),
        }
    }

    fn take_reset(&mut self) -> Option<Reset> {
        self.reset.take()
    }

    fn take_start(&mut self) -> Option<Start> {
        (!self.starts.is_empty()).then(|| self.starts.remove(0))
    }

    fn reset_vector(&self, cpu: usize) -> Option<u64> {
        self.locked_reset_vector(cpu)
    }

    fn processor_power(&mut self, cpu: usize, on: bool) {
        self.processor_powered(cpu, on);
    }

    /// An AICv2 or AICv3 stops sending IRQs to a CPU whose SIQ_CFG_EL1
    /// refuses them; an AICv1 is not documented to consult it.
    fn irq_steering(&mut self, cpu: usize, refuses: bool, now: u64) {
        self.observe_time(now, "IRQ steering");
        let Some(Controller::V2(id)) = self.aic else {
            return;
        };
        let mut access = Access::new(cpu, now, &mut self.latch);
        if let Some(aic) = self.bus.device_mut::<Aic2>(id) {
            aic.steer(cpu, refuses, &mut access);
        }
    }

    /// A sleeping SoC with no alarm due waits for the host's power button.
    fn waits_on_host(&self) -> bool {
        self.sleep.asleep
    }

    fn take_host_service(&mut self) -> bool {
        self.usb_masters
            .iter_mut()
            .fold(core::mem::take(&mut self.frame_presented), |due, master| {
                master.session.take_due() | due
            })
    }

    fn device_name(&self, physical: u64) -> Option<&'static str> {
        self.bus.name_at(physical)
    }

    fn memory_lock(&self) -> stela_engine::machine::mmu::MemoryLocks {
        let Some(device) = self.memory_controller else {
            return [None; stela_soc_data::amcc::GROUPS];
        };
        if let Some(locked) = memory_resources::resource_for(&self.bus, device)
            .and_then(|owner| owner.locked_regions())
        {
            locked
        } else if let Some(owner) = self
            .bus
            .device::<memory_resources::t8112::cache::EmptyResource>(device)
        {
            owner.locked_regions()
        } else if let Some(owner) = self
            .bus
            .device::<memory_resources::t8112::amcc::LockResource>(device)
        {
            owner.locked_regions()
        } else {
            self.bus
                .device::<Amcc>(device)
                .map_or([None; stela_soc_data::amcc::GROUPS], Amcc::locked_regions)
        }
    }

    fn dma_deadline(&self) -> Option<u64> {
        self.usb_masters
            .iter()
            .filter_map(|master| {
                let port = UsbPort::of(&master.connector);
                let controller = self.bus.device::<Dwc3>(master.device);
                if !controller.is_some_and(Dwc3::lifecycle_active) {
                    return None;
                }
                if master.phy_changed
                    || port != master.port
                    || controller.is_some_and(Dwc3::host_pending)
                {
                    return Some(0);
                }
                let poll = (port.ready || master.attached).then_some(master.next_poll);
                let function = master.peripheral.as_ref().map(|p| p as &dyn Function);
                let host = controller.and_then(|c| c.host_deadline(function));
                poll.into_iter().chain(host).min()
            })
            .chain(self.native_pmp_dma_deadline())
            .chain(self.native_pmp_services_deadline())
            // A SEP passcode derivation runs one bounded slice per step,
            // so it finishes while every PE waits.
            .chain(
                self.sep_masters
                    .iter()
                    .filter(|master| {
                        self.bus
                            .device::<Sep>(master.sep)
                            .is_some_and(Sep::deriving)
                    })
                    .map(|_| 0),
            )
            // A display's next vblank, while its notification clients pace
            // by it, a swap waits to be latched or a queued flip to be made.
            .chain(self.dcp_masters.iter().filter_map(|master| {
                self.bus
                    .device::<Asc>(master.asc)
                    .filter(|asc| asc.execution_enabled())
                    .and_then(Asc::firmware::<dcp::Coprocessor>)?
                    .refresh_deadline()
            }))
            // When to look for a storage command's host transfer.
            .chain(
                self.storage_masters
                    .iter()
                    .filter_map(|&(ans, _)| self.bus.device::<Ans>(ans)?.host_look()),
            )
            .min()
    }

    fn take_call(&mut self, cpu: usize) -> Option<stela_engine::machine::engine::GateCall> {
        self.gpu_gates.iter().find_map(|&gate| {
            self.bus
                .device_mut::<gpu_gate::Gate>(gate)
                .expect("the gate is mapped")
                .take(cpu)
        })
    }

    fn finish_call(&mut self, outcome: stela_engine::machine::engine::GateOutcome) {
        self.bus
            .device_mut::<gpu_gate::Gate>(self.gpu_gates[outcome.window])
            .expect("the gate is mapped")
            .finish(outcome.answer);
    }

    /// The entry, each master's DMA step and the wires it settles. After a
    /// device write ([`Self::settle_write`]) a master that write did not
    /// reach takes its step only when it has work ([`takes_step`]).
    fn transfer(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        let mut lap = self.transfer_times.lap();
        self.observe_time(now, "transfer");
        self.publish_rtc_time(now)?;
        self.settle_lifecycle(cpu, now);
        lap.end(&mut self.transfer_times, 0);
        // A master's refused work stops only that master: the others and the
        // devices' timers go on, and the first refusal is what is reported.
        let mut results = [const { Ok(()) }; 7];
        for (index, result) in results.iter_mut().enumerate() {
            *result = match index {
                0 => self.transfer_pmp(cpu, memory, now),
                1 => self.transfer_aes(cpu, memory, now),
                2 => self.transfer_storage(cpu, memory, now),
                3 => self.transfer_gfx(cpu, memory, now),
                4 => self.transfer_sep(cpu, memory, now),
                5 => self.transfer_dcp(cpu, memory, now),
                _ => self.transfer_usb(cpu, memory, now),
            };
            lap.end(&mut self.transfer_times, index + 1);
        }
        let settled = self.settle(cpu, now, None);
        lap.end(&mut self.transfer_times, 8);
        results.into_iter().find(Result::is_err).unwrap_or(settled)
    }

    /// As the default, with the transfer knowing PE `cpu`'s write: each
    /// master that write did not reach takes its step only when it has work
    /// ([`takes_step`]); the home machine's transfers run every step.
    fn settle_write(
        &mut self,
        cpu: usize,
        now: u64,
        memory: &mut dyn stela_engine::machine::engine::LockedMemory,
    ) -> stela_engine::machine::engine::Settled {
        self.writer = self.last_write.get_mut(cpu).and_then(Option::take);
        let lock = self.memory_lock();
        let transfer = self.transfer(cpu, memory.under(lock), now);
        self.writer = None;
        stela_engine::machine::engine::Settled {
            transfer,
            lock,
            call: self.take_call(cpu),
            after: self.memory_lock(),
        }
    }
}

/// Whether a master whose devices are `devices` takes its step in a
/// transfer after the write `written` ([`AppleDevices::written_device`]):
/// always in a transfer of the home machine's own (`None`) or after a write
/// that reached one of `devices`, and otherwise only when `due` says the
/// step has work. Each master's `due` is exact: everything its step would
/// act on, its deadlines and the state other devices' writes and the
/// transfer's entry (power, PHYs, cables) can change. A step that only acts
/// on its devices' pending work (AES, storage, GFX, SEP) is its own test.
fn takes_step(
    written: Option<Option<DeviceId>>,
    devices: &[DeviceId],
    due: impl FnOnce() -> bool,
) -> bool {
    match written {
        None => true,
        Some(device) => device.is_some_and(|device| devices.contains(&device)) || due(),
    }
}

impl AppleDevices {
    fn transfer_pmp(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        // SAFETY: the persistent native binding admits only the canonical
        // execution path. Its held cohort/fenced Host sessions and
        // exclusive synchronous device owner remain live through this step.
        unsafe {
            self.service_native_pmp_services(cpu, now)?;
            self.service_native_pmp_dma(cpu, memory, now)?;
            self.service_native_pmp_services(cpu, now)
        }
    }

    fn transfer_aes(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        for master in &self.masters {
            let (aes, dart) = self
                .bus
                .pair_mut::<Aes, Dart>(master.aes, master.dart)
                .expect("a master names an AES engine and its DART");
            if !aes.pending() {
                continue;
            }
            let mut access = Access::new(cpu, now, &mut self.latch);
            let mut port = Port {
                iommu: dart,
                stream: master.stream,
                memory,
                access: &mut access,
            };
            aes.run(&mut port).map_err(describe_transfer)?;
        }
        Ok(())
    }

    fn transfer_storage(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        for &(ans, sart) in &self.storage_masters {
            let (ans, sart) = self
                .bus
                .pair_mut::<Ans, Sart>(ans, sart)
                .expect("a storage master names ANS2 and SART");
            if self.defer_dma {
                ans.defer_copies();
            }
            if !ans.pending(now) {
                continue;
            }
            let mut access = Access::new(cpu, now, &mut self.latch);
            let mut port = Port {
                iommu: &mut Physical,
                stream: 0,
                memory,
                access: &mut access,
            };
            ans.run(&mut port, sart)
                .map_err(|error| match describe_transfer(error) {
                    DeviceError::TransferRefused(reason) => DeviceError::TransferRefused(
                        refuse!("ANS transfer", last_submission = ?ans.last_submission())
                            .because(reason),
                    ),
                    other => other,
                })?;
        }
        Ok(())
    }

    fn transfer_gfx(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        for &(asc, block) in &self.gfx_masters {
            let (asc, block) = self
                .bus
                .pair_mut::<Asc, Sgx>(asc, block)
                .expect("a GFX master names its ASC and sgx block");
            if !asc.dma_pending() {
                continue;
            }
            let mut access = Access::new(cpu, now, &mut self.latch);
            let mut view = block.uat().agent(Agent::Firmware);
            let mut port = Port {
                iommu: &mut view,
                stream: uat::FIRMWARE_CONTEXT,
                memory: &mut *memory,
                access: &mut access,
            };
            if let Err(reason) = asc.service_dma(&mut port) {
                // The walk's evidence: the last fault and the firmware
                // context's TTBAT words, as the guest left them.
                let uat = block.uat();
                let words = uat.ttbat().map(|base| {
                    let address = base + uat::FIRMWARE_CONTEXT as u64 * 16;
                    let mut bytes = [0u8; 16];
                    memory.read(address, &mut bytes).then_some((address, bytes))
                });
                // The nonzero entries of the faulting half's root table.
                let root = |words: &[u8; 16], half: usize| {
                    u64::from_le_bytes(words[half * 8..half * 8 + 8].try_into().unwrap())
                        & 0xff_ffff_c000
                };
                let half = uat
                    .last_fault()
                    .map_or(1, |fault| usize::from(fault.address >> uat::IAS & 1 != 0));
                let mut entries = Vec::new();
                if let Some(Some((_, words))) = words {
                    let table = root(&words, half);
                    for index in 0..2048u64 {
                        if entries.len() == 8 {
                            break;
                        }
                        let mut entry = [0u8; 8];
                        if memory.read(table + index * 8, &mut entry) && entry != [0; 8] {
                            entries.push((index, u64::from_le_bytes(entry)));
                        }
                    }
                }
                return Err(DeviceError::TransferRefused(
                    refuse!(
                        "GFX firmware DMA",
                        last_uat_fault = ?uat.last_fault(),
                        context_ttbat = ?words,
                        root_entries = ?entries
                    )
                    .because(reason),
                ));
            }
        }
        Ok(())
    }

    fn transfer_sep(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        for master in &self.sep_masters {
            let (sep, dart) = self
                .bus
                .pair_mut::<Sep, Dart>(master.sep, master.dart)
                .expect("a SEP master names its model and DART");
            if !sep.pending() {
                continue;
            }
            let mut access = Access::new(cpu, now, &mut self.latch);
            let mut port = Port {
                iommu: dart,
                stream: master.stream,
                memory,
                access: &mut access,
            };
            sep.transfer(&mut port).map_err(describe_transfer)?;
        }
        Ok(())
    }

    /// The DCPs' step: each one's link DMA first, so a submit or a fence
    /// notice the AP sent is seen by this step's vblank; then the display at
    /// `now` (its vblank, a present latched at it); then the link DMA again
    /// for the callbacks the vblank queued, its vsync and the completions.
    /// Its devices are its ASC, link DART and surface DART, and its work
    /// [`Self::dcp_due`] ([`takes_step`]).
    fn transfer_dcp(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        let written = self.written_device();
        for index in 0..self.dcp_masters.len() {
            let master = &self.dcp_masters[index];
            let surface = master.surface.map(|(surface, _)| surface);
            let devices = [master.asc, master.dart, surface.unwrap_or(master.dart)];
            if !takes_step(written, &devices, || self.dcp_due(index, now)) {
                continue;
            }
            self.service_dcp_link(index, cpu, memory, now)?;
            let master = &self.dcp_masters[index];
            let mut finished = Ok(false);
            let presenting = self
                .bus
                .device_mut::<Asc>(master.asc)
                .filter(|asc| asc.execution_enabled())
                .and_then(Asc::firmware_mut::<dcp::Coprocessor>)
                .is_some_and(|coprocessor| {
                    // A compose that ended finishes its present first.
                    finished = coprocessor.finish_compose();
                    coprocessor.refresh(now);
                    coprocessor.frame_ready(now)
                });
            match finished {
                Ok(true) => self.frame_presented = true,
                Ok(false) => {}
                Err(reason) => {
                    return Err(DeviceError::TransferRefused(
                        refuse!("DCPEXT swap surface").because(reason),
                    ));
                }
            }
            if presenting
                && self.defer_dma
                && let Some((surface, stream)) = master.surface
            {
                let (asc, dart) = self
                    .bus
                    .pair_mut::<Asc, Dart>(master.asc, surface)
                    .expect("a DCP master names its ASC and surface DART");
                let coprocessor = asc
                    .firmware_mut::<dcp::Coprocessor>()
                    .expect("a DCP master's ASC runs the DCP model");
                let page = stela_contracts::dma::Iommu::page(&*dart);
                let resolve = |iova: u64| {
                    let last = iova
                        .checked_add(page - 1)
                        .ok_or(stela_contracts::dma::DmaError::Memory)?;
                    dart.resolve(memory, stream, iova, last, false)
                };
                // No link service follows in this locked call: a call that
                // drops swaps would wait for a compose that runs only once
                // the lock is released. The transfer the composing PE makes
                // right after it ([`SharedDevices::run_unlocked`]) finishes
                // the present and serves the link.
                if let Some(compose) = coprocessor.compose(now, page, resolve) {
                    self.composes.push(compose);
                    continue;
                }
            }
            if presenting {
                let (surface, stream) = master.surface.ok_or_else(|| {
                    DeviceError::TransferRefused(
                        "DCPEXT swap surface: the display pipe's DART (dart-dispext0) is not placed"
                            .into(),
                    )
                })?;
                let (asc, dart) = self
                    .bus
                    .pair_mut::<Asc, Dart>(master.asc, surface)
                    .expect("a DCP master names its ASC and surface DART");
                let coprocessor = asc
                    .firmware_mut::<dcp::Coprocessor>()
                    .expect("a DCP master's ASC runs the DCP model");
                let mut access = Access::new(cpu, now, &mut self.latch);
                let mut port = Port {
                    iommu: dart,
                    stream,
                    memory,
                    access: &mut access,
                };
                coprocessor.present(now, &mut port).map_err(|reason| {
                    DeviceError::TransferRefused(refuse!("DCPEXT swap surface").because(reason))
                })?;
                self.frame_presented = true;
            }
            self.service_dcp_link(index, cpu, memory, now)?;
        }
        Ok(())
    }

    /// Run DCP `index`'s queued link DMA, if any.
    fn service_dcp_link(
        &mut self,
        index: usize,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        let master = &self.dcp_masters[index];
        let (asc, dart) = self
            .bus
            .pair_mut::<Asc, Dart>(master.asc, master.dart)
            .expect("a DCP master names its ASC and DART");
        if !asc.dma_pending() {
            return Ok(());
        }
        let mut access = Access::new(cpu, now, &mut self.latch);
        let mut port = Port {
            iommu: dart,
            stream: master.stream,
            memory,
            access: &mut access,
        };
        asc.service_dma(&mut port).map_err(|reason| {
            DeviceError::TransferRefused(refuse!("DCPEXT link DMA").because(reason))
        })
    }

    /// Whether DCP `index`'s step has work at `now`: link DMA queued, a
    /// compose to finish, or the display's look due ([`dcp::Coprocessor::
    /// refresh_deadline`]: a vblank while anyone listens, a swap waits or the
    /// scanout queued work). That deadline is what the home machine's DMA
    /// deadline holds for it, and a DCP's own writes reach its devices, so
    /// a step without either changes nothing a later one would not.
    fn dcp_due(&self, index: usize, now: u64) -> bool {
        let master = &self.dcp_masters[index];
        let Some(asc) = self.bus.device::<Asc>(master.asc) else {
            return true;
        };
        asc.dma_pending()
            || asc.execution_enabled()
                && asc
                    .firmware::<dcp::Coprocessor>()
                    .is_some_and(|coprocessor| {
                        coprocessor.composing()
                            || coprocessor.refresh_deadline().is_some_and(|due| due <= now)
                    })
    }

    /// The device the write being settled ([`Self::writer`]) reached, if
    /// one is being settled: `Some(None)` for an address no device takes.
    fn written_device(&self) -> Option<Option<DeviceId>> {
        self.writer.map(|(physical, bytes)| {
            Width::from_bytes(u64::from(bytes))
                .and_then(|width| self.bus.route(physical, width).ok())
                .map(|(device, _)| device)
        })
    }

    /// Whether USB master `master`'s step has work at `now`: a session to
    /// cancel, a new cable generation, a port or host cable the controller
    /// has not taken up (the PHY and power the transfer's entry settled can
    /// change it), and, with the controller active, its schedule's work: what
    /// the home machine's DMA deadline is for ([`Devices::dma_deadline`]),
    /// and the work a step takes whatever the time.
    fn usb_due(controller: &Dwc3, master: &UsbMaster, now: u64) -> bool {
        let connector = UsbPort::of(&master.connector);
        if master.session_reset || controller.service_due() || connector != master.port {
            return true;
        }
        if !controller.lifecycle_active() {
            return false;
        }
        let physical_port = UsbPort {
            generation: connector.generation,
            ready: connector.ready && controller.phy_ready(),
        };
        let cable = controller
            .phy_ready()
            .then(|| host_function(&master.connector))
            .flatten();
        if !controller.port_synced(&physical_port) || controller.host_function() != cable {
            return true;
        }
        if master.phy_changed
            || master.session_reset
            || controller.service_due()
            || controller.dma_fault().is_some()
            || controller.host_pending()
            || controller.available() != master.attached
        {
            return true;
        }
        let function = master.peripheral.as_ref().map(|p| p as &dyn Function);
        let poll = master.attached.then_some(master.next_poll);
        poll.into_iter()
            .chain(controller.host_deadline(function))
            .any(|due| due <= now)
    }

    /// Each USB master's step. Its devices are its core, PHY and DART, and
    /// its work [`Self::usb_due`] ([`takes_step`]); a master that skips its
    /// step keeps its session clock where the step would have.
    fn transfer_usb(
        &mut self,
        cpu: usize,
        memory: &mut dyn Memory,
        now: u64,
    ) -> Result<(), DeviceError> {
        let written = self.written_device();
        for master in &mut self.usb_masters {
            let (controller, dart) = self
                .bus
                .pair_mut::<Dwc3, Dart>(master.device, master.dart)
                .expect("USB master retains its DART");
            let devices = [master.device, master.phy, master.dart];
            if !takes_step(written, &devices, || Self::usb_due(controller, master, now)) {
                // A detached session moves nothing; its clock moves on.
                if controller.lifecycle_active() && !master.attached && now >= master.next_poll {
                    master.next_poll = usb_poll(now, master.next_poll).1;
                }
                continue;
            }
            let mut access = Access::new(cpu, now, &mut self.latch);
            let mut port = Port {
                iommu: dart,
                stream: master.stream,
                memory,
                access: &mut access,
            };
            let fail =
                |why| DeviceError::TransferRefused(refuse!("USB device advance").because(why));
            master
                .cancel_session(controller.take_session_reset())
                .map_err(fail)?;
            let connector = UsbPort::of(&master.connector);
            if connector != master.port {
                // A new cable generation cancels the previous connection's work.
                master.session.detach().map_err(fail)?;
                master.attached = false;
                master.port = connector;
                master.next_poll = 0;
            }
            // A kept clock/quiesce pause preserves the external session;
            // connector changes still cancel it, and reset/power loss above
            // invalidates it before any new USB or DMA transaction.
            if !controller.lifecycle_active() {
                continue;
            }
            let physical_port = UsbPort {
                generation: master.port.generation,
                ready: master.port.ready && controller.phy_ready(),
            };
            controller.sync_port(&physical_port).map_err(fail)?;
            controller
                .attach_host_function(
                    controller
                        .phy_ready()
                        .then(|| host_function(&master.connector))
                        .flatten(),
                )
                .map_err(fail)?;
            let function = master.peripheral.as_mut().map(|p| p as &mut dyn Function);
            controller.service_host(&mut port, function).map_err(fail)?;
            controller.flush(&mut port).map_err(fail)?;
            let ready = controller.available();
            if !ready && master.attached {
                master.session.detach().map_err(fail)?;
                master.attached = false;
            }
            if ready && !master.attached {
                master.epoch = master
                    .epoch
                    .checked_add(1)
                    .ok_or_else(|| fail(refuse!("USB connection generation exhausted")))?;
                master.session.attach(master.epoch).map_err(fail)?;
                master.attached = true;
            }
            if now >= master.next_poll {
                let (packets, next) = usb_poll(now, master.next_poll);
                master.next_poll = next;
                let advanced = (|| {
                    // Moving stops where every pending transfer NAKs.
                    for _ in 0..packets {
                        if !master
                            .session
                            .advance(&mut controller.peer(&mut port), now)?
                        {
                            break;
                        }
                    }
                    controller.flush(&mut port)
                })();
                advanced.map_err(|why| match controller.dma_fault() {
                    Some(fault) => fail(refuse!("USB DMA fault", ?fault).because(why)),
                    None if why.what() == crate::usb::host::CONTROL_TIMEOUT => {
                        fail(controller.control_timeout(master.number, why))
                    }
                    None => fail(why),
                })?;
            }
            master.phy_changed = false;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;

impl UnlockedDma for crate::ans::ReadCopy {
    fn run(self: Box<Self>, memory: &mut dyn Memory) {
        crate::ans::ReadCopy::run(*self, memory);
    }
}

impl UnlockedDma for dcp::compose::Compose {
    fn run(self: Box<Self>, memory: &mut dyn Memory) {
        dcp::compose::Compose::run(*self, memory);
    }
}
