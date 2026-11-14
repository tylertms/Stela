//! `stela.cfg`'s system-mode options: one typed record, the one parser
//! `stela.efi`'s loader and engine read it with, and the one serializer
//! `stela setup` writes it with. The option line is whitespace-separated
//! words starting with `--system`; each option is a flag or a flag and one
//! value. The spellings are part of the setup/mount contract
//! (`stela_loader::mount_interface::OPTIONS_REVISION`).
use crate::Result;
use alloc::{string::String, vec::Vec};
use core::fmt;
use stela_contracts::refuse;
pub use stela_contracts::uuid::Uuid;
use stela_loader::{
    boot_selection::{self, BootVolume},
    handoff::BootCommand,
    mount_interface, recovery,
};
use stela_runtime::pc::{gpt::Guid, storage::Target};

mod error;
mod parse;
mod validate;

pub use error::{Constraint, Error, GuidArgument, TargetArgument};

/// Apple T8112 DRAM base (Asahi Linux T8112 notes; the DeviceTree `/memory`).
pub const DRAM_BASE: u64 = 0x8_0000_0000;
const LARGE: u64 = 2 * 1024 * 1024;

pub const SYSTEM: &str = "--system";
pub const INSTALL: &str = mount_interface::INSTALL_OPTION;
pub const INSTALL_SOURCE: &str = "--install-source";
pub const VERIFY_ON_READ: &str = "--verify-on-read";
pub const UPDATE_RESTORE: &str = mount_interface::UPDATE_RESTORE_OPTION;
pub const EXTERNAL_DISKS: &str = "--external-disks";
pub const NATIVE_CORES: &str = "--native-cores";
pub const GUEST_PES: &str = "--guest-pes";
pub const MAINTENANCE_WRITE: &str = "--maintenance-write";
/// Boots the installed system to its single-user shell.
pub const SINGLE_USER: &str = "--single-user";
/// Keeps the newest presented frame at one logged address.
pub const SCANOUT_MIRROR: &str = "--scanout-mirror";
pub const WIFI: &str = "--wifi";
pub const WIFI_KEY: &str = "--wifi-key";
pub const WIFI_COUNTRY: &str = "--wifi-country";
pub const USB4_APPROVE: &str = "--usb4-approve";
/// The ESP file `--wifi` reads its key from without `--wifi-key`.
pub const DEFAULT_WIFI_KEY: &str = "\\EFI\\stela\\wifi.key";
/// IEEE 802.11 SSIDs are 1 to 32 octets.
const SSID_BYTES: core::ops::RangeInclusive<usize> = 1..=32;
pub const IPV4: &str = "--ipv4";
pub const IPV4_GATEWAY: &str = "--ipv4-gateway";
pub const DNS: &str = "--dns";
/// The most `--dns` servers, as glibc's resolver keeps (`MAXNS`).
pub const MAX_DNS_SERVERS: usize = 3;

/// A `--audio-check` mode.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum AudioCheck {
    Playback,
    Duplex,
    TransportPlayback,
    TransportDuplex,
}

impl AudioCheck {
    const NAMES: [(&'static str, Self); 4] = [
        ("playback", Self::Playback),
        ("duplex", Self::Duplex),
        ("transport-playback", Self::TransportPlayback),
        ("transport-duplex", Self::TransportDuplex),
    ];

    pub fn parse(value: &str) -> Result<Self, Error> {
        Self::NAMES
            .iter()
            .find(|(name, _)| *name == value)
            .map(|(_, mode)| *mode)
            .ok_or(Error::AudioMode)
    }

    pub fn name(self) -> &'static str {
        Self::NAMES
            .iter()
            .find(|(_, mode)| *mode == self)
            .map(|(name, _)| *name)
            .unwrap_or_default()
    }

    pub fn transport(self) -> bool {
        matches!(self, Self::TransportPlayback | Self::TransportDuplex)
    }

    pub fn duplex(self) -> bool {
        matches!(self, Self::Duplex | Self::TransportDuplex)
    }
}

/// Where a `--dump` range starts: a guest virtual address, or the stopped
/// PE's general register `xN` plus or minus an offset (`x21`, `x22+0x4000`).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Base {
    Address(u64),
    Register(usize, i64),
}

impl Base {
    /// Parse and check one option line.
    pub fn parse(text: &str) -> Result<Self, Error> {
        let Some(register) = text.strip_prefix('x') else {
            return Ok(Self::Address(number(text)?));
        };
        let (index, offset) = match register.find(['+', '-']) {
            Some(at) => {
                let magnitude = number(&register[at + 1..])? as i64;
                let sign = if register.as_bytes()[at] == b'-' {
                    -1
                } else {
                    1
                };
                (&register[..at], sign * magnitude)
            }
            None => (register, 0),
        };
        match index.parse() {
            Ok(index) if index < 31 => Ok(Self::Register(index, offset)),
            _ => Err(Error::DumpRegister),
        }
    }

    pub fn resolve(self, x: &[u64; 31]) -> u64 {
        match self {
            Self::Address(address) => address,
            Self::Register(index, offset) => x[index].wrapping_add_signed(offset),
        }
    }
}

impl fmt::Display for Base {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match *self {
            Self::Address(address) => write!(f, "{address:#x}"),
            Self::Register(index, 0) => write!(f, "x{index}"),
            Self::Register(index, offset) if offset < 0 => {
                write!(f, "x{index}-{:#x}", offset.unsigned_abs())
            }
            Self::Register(index, offset) => write!(f, "x{index}+{offset:#x}"),
        }
    }
}

/// A number in `stela.cfg`: decimal, or hexadecimal after `0x` with optional
/// `_` separators.
pub fn number(value: &str) -> Result<u64, Error> {
    match value.strip_prefix("0x") {
        Some(hex) => u64::from_str_radix(&hex.replace('_', ""), 16),
        None => value.parse(),
    }
    .map_err(|_| Error::Number)
}

/// The legacy HTTP-only host/path adapter. URLs that need a transport scheme
/// must use the original full URL; HTTPS cannot lose its security requirement.
pub fn split_url(url: &str) -> Result<(&str, &str)> {
    if !url.starts_with("http://") {
        return Err(refuse!("only http:// URLs are supported"));
    }
    let value = parse_url(url).map_err(Error::message)?;
    Ok((value.authority, value.target))
}

fn parse_url(url: &str) -> Result<stela_runtime::net::http::Url<'_>, Error> {
    // Preserve scheme/path diagnostic priority, then require the one shared
    // syntax and transport-policy parser for every accepted value.
    if !["http://", "https://"].iter().any(|prefix| {
        url.get(..prefix.len())
            .is_some_and(|part| part.eq_ignore_ascii_case(prefix))
    }) {
        return Err(Error::UrlScheme);
    }
    stela_runtime::net::http::Url::parse(url).map_err(|error| match error {
        stela_runtime::net::http::UrlError::Scheme => Error::UrlScheme,
        stela_runtime::net::http::UrlError::Path => Error::UrlPath,
        stela_runtime::net::http::UrlError::Host if url.ends_with("://") => Error::UrlPath,
        stela_runtime::net::http::UrlError::Host => Error::UrlHost,
        other => Error::Url(other),
    })
}

/// The system-mode options of one `stela.cfg` line.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct System {
    pub dram_base: u64,
    /// `--dram-base` was given; otherwise a bundle's own base applies.
    pub dram_explicit: bool,
    pub load: u64,
    pub entry: u64,
    pub x0: u64,
    pub budget: u64,
    /// Load the boot bundle the loader read instead of `source.bin`.
    pub bundle: bool,
    /// The bundle's directory on the Stela volume (`--bundle DIR`).
    pub bundle_directory: Option<String>,
    /// Build the boot bundle from the installed system on this APFS
    /// partition instead (`SERIAL:GUID`).
    pub preboot: Option<Target>,
    /// Allow SIP-protected filesystem writes for this explicit maintenance boot.
    pub maintenance_write: bool,
    /// Boot the installed system single-user, for maintenance at its console.
    pub single_user: bool,
    /// This boot is the automatic Metal plugin repair NVRAM asked for
    /// (`--bundle-repair`, never from `stela.cfg`): single-user with
    /// filesystem protection off, its console driven by the engine
    /// (`bundle_repair`).
    pub bundle_repair: bool,
    /// Keep the newest presented frame in host memory for a debugger to read:
    /// a passed-through GPU's picture otherwise leaves only through its port.
    pub scanout_mirror: bool,
    /// The NVRAM boot volume the loader chose (`--boot-volume`), naming the
    /// Preboot disk's container and volume group.
    pub boot_volume: Option<BootVolume>,
    /// NVRAM asked for a safe-mode boot (`--safe-boot`).
    pub safe_boot: bool,
    /// The entry origin kept after NVRAM's one-shot command is consumed.
    pub boot_command: Option<BootCommand>,
    /// NVRAM chose a staged macOS update (`--upgrade`): build from the
    /// Preboot disk's Update volume.
    pub upgrade: bool,
    /// NVRAM chose a recoveryOS the Preboot disk holds: its Recovery
    /// volume's paired copy (`--paired-recovery`) or its RecoveryOSContainer's
    /// System Recovery (`--system-recovery`).
    pub recovery: Option<recovery::Kind>,
    /// One host partition backing all ANS namespaces.
    pub storage: Option<Target>,
    /// Instead, this many bytes of engine memory back them, and nothing
    /// persists (`--storage-ram`).
    pub storage_ram: Option<u64>,
    /// The host partition holding the NOR flash image (`--nor`); the
    /// `--storage` partition itself keeps it in its tail.
    pub nor: Option<Target>,
    /// `--restore-study`.
    pub restore: bool,
    pub restore_source: Option<Target>,
    /// `--restore-manifest PATH` was given.
    pub restore_manifest: bool,
    pub restore_manifest_path: Option<String>,
    pub restore_identity: Option<usize>,
    /// The install medium's `payload.json` (`--install`), whose directory
    /// holds its pieces: a restore with Apple's stock options.
    pub install: Option<String>,
    /// The install medium's partition, by its unique GUID alone.
    pub install_source: Option<Guid>,
    /// Refuse a payload without recorded block hashes, which would be read
    /// whole before StartRestore (`--verify-on-read`).
    pub verify_on_read: bool,
    /// Restore with the inventory's Update identity (`--update-restore`).
    pub update_restore: bool,
    /// Use restored's development-board policy to keep and report FDR
    /// failures while allowing the erase restore to continue
    /// (`--development-restore`), a degraded mode.
    pub development_restore: bool,
    /// The disk's system was installed by a development restore, so it boots
    /// as the same development board; set from its bundle, not a flag.
    pub development_board: bool,
    /// Answer device accesses no model implements and list them
    /// (`--survey`): research runs only.
    pub survey: bool,
    /// Leap the counter over waits of every PE (`--idle-skip`): boot and
    /// install runs, not interactive use.
    pub idle_skip: bool,
    /// Cable the native USB keyboard and pointer to the last USB-C
    /// controller and feed it the host's virtio-input devices (`--input`).
    pub input: bool,
    /// Leave the guest without the Bluetooth controller `--input` cables
    /// to it (`--no-guest-bluetooth`). On a host with no Bluetooth radio
    /// (a VM, typically) the modeled controller has no radio, and Setup
    /// Assistant can stall on it; past setup, macOS wants a controller.
    pub no_guest_bluetooth: bool,
    /// Explicit native executors and guest PEs (`--native-cores N`), one per
    /// logical processor; SMT siblings count after every physical core.
    pub native_cores: Option<usize>,
    /// The source PE count (`--guest-pes N`) recorded by setup for disk
    /// boots whose verified DeviceTree is loaded after firmware exit.
    pub guest_pes: Option<usize>,
    /// Check owned AP startup/host resources, then stop before guest entry.
    pub native_bootstrap_check: bool,
    /// Bounded checked-memory kernel execution on the selected cores.
    pub native_shared_check: bool,
    /// Emit full artifact identities (`--native-artifacts`).
    pub native_artifacts: bool,
    /// Execute PEs in checked native rounds on their assigned workers
    /// (`--native-rounds`) instead of serialized chained blocks on the BSP.
    pub native_rounds: bool,
    /// Run each PE on its own host core, the default. `--serialized` runs
    /// every PE on the BSP instead, as native rounds and checks do.
    pub parallel: bool,
    /// Time the engine's phases and report them with each heartbeat
    /// (`--profile`).
    pub profile: bool,
    /// The machine's ECID for a bundle built from the Preboot volume
    /// (`--ecid HEX`, nonzero).
    pub ecid: Option<u64>,
    /// POST once to this `http://` URL at start and report the reply
    /// (`--network-check URL`).
    pub network_check: Option<String>,
    /// Check exclusively owned conventional HDA after ExitBootServices.
    pub audio_check: Option<AudioCheck>,
    /// Explicit whole-controller handoff for the audio check's physical clock.
    pub audio_own_hpet: bool,
    /// Exclusive handoff of the original IOAPICs through shutdown.
    pub native_pci_own_ioapic: bool,
    /// Whole host disks, by serial, that the guest sees as USB mass
    /// storage behind the native hub (`--usb-storage SERIAL`, repeatable).
    pub usb_storage: Vec<String>,
    /// USB4 devices that get a PCIe tunnel where no IOMMU confines their DMA
    /// (`--usb4-approve UUID`, repeatable).
    pub usb4_approved: Vec<Uuid>,
    /// Every other host disk the engine does not open itself is USB mass
    /// storage too, up to the hub's free ports (`--external-disks`).
    pub external_disks: bool,
    /// Give the guest a USB Ethernet adapter bridged to a host network
    /// adapter of its own (`--guest-network`).
    pub guest_network: bool,
    /// Join this Wi-Fi network for the mount's network (`--wifi SSID`). The
    /// line writes `%`, whitespace and control characters as `%XX`.
    pub ssid: Option<String>,
    /// The ESP file holding the network's passphrase or PSK (`--wifi-key
    /// PATH`); `DEFAULT_WIFI_KEY` when `--wifi` is set without it.
    pub wifi_key: Option<String>,
    /// The ISO 3166-1 alpha-2 regulatory country, uppercase
    /// (`--wifi-country XX`).
    pub wifi_country: Option<String>,
    /// The mount network's IPv4 address, set by hand instead of by DHCP
    /// (`--ipv4 ADDRESS/PREFIX`).
    pub ipv4: Option<StaticIpv4>,
    /// Its default router (`--ipv4-gateway ADDRESS`).
    pub ipv4_gateway: Option<core::net::Ipv4Addr>,
    /// The DNS servers, in place of DHCP's and router advertisements'
    /// (`--dns SERVER[,SERVER...]`).
    pub dns: Vec<core::net::IpAddr>,
    /// Advertise and log the SEP services whose contracts are not recovered
    /// (`--sep-observe`), each request refused by name.
    pub sep_observe: bool,
    /// Stop when a PE reaches one of these guest PCs (`--break`).
    pub breakpoints: Vec<u64>,
    /// Log, without stopping, when a PE reaches one of these guest PCs
    /// (`--trace`).
    pub traces: Vec<u64>,
    /// Log raw guarded entries, exits, exceptions and returns (`--monitor-trace`).
    pub monitor_trace: bool,
    /// Log every display swap's DCP calls, present and callback (`--display-trace`).
    pub display_trace: bool,
    /// Log every device access in these physical ranges (`--watch BASE,BYTES`).
    pub watches: Vec<(u64, u64)>,
    /// Guest virtual ranges the stop's report shows (`--dump BASE,BYTES`).
    pub dumps: Vec<(Base, u64)>,
}

impl Default for System {
    fn default() -> Self {
        Self {
            dram_base: DRAM_BASE,
            dram_explicit: false,
            load: DRAM_BASE,
            entry: DRAM_BASE,
            x0: 0,
            budget: 0,
            bundle: false,
            bundle_directory: None,
            preboot: None,
            boot_volume: None,
            safe_boot: false,
            maintenance_write: false,
            single_user: false,
            bundle_repair: false,
            scanout_mirror: false,
            boot_command: None,
            upgrade: false,
            recovery: None,
            storage: None,
            storage_ram: None,
            nor: None,
            restore: false,
            restore_source: None,
            restore_manifest: false,
            restore_manifest_path: None,
            restore_identity: None,
            install: None,
            install_source: None,
            verify_on_read: false,
            update_restore: false,
            development_restore: false,
            development_board: false,
            survey: false,
            idle_skip: false,
            input: false,
            no_guest_bluetooth: false,
            native_cores: None,
            guest_pes: None,
            native_bootstrap_check: false,
            native_shared_check: false,
            native_artifacts: false,
            native_rounds: false,
            parallel: true,
            profile: false,
            ecid: None,
            network_check: None,
            audio_check: None,
            audio_own_hpet: false,
            native_pci_own_ioapic: false,
            usb_storage: Vec::new(),
            usb4_approved: Vec::new(),
            external_disks: false,
            guest_network: false,
            ssid: None,
            wifi_key: None,
            wifi_country: None,
            ipv4: None,
            ipv4_gateway: None,
            dns: Vec::new(),
            sep_observe: false,
            breakpoints: Vec::new(),
            traces: Vec::new(),
            monitor_trace: false,
            display_trace: false,
            watches: Vec::new(),
            dumps: Vec::new(),
        }
    }
}

/// A host address and its subnet's prefix length (`--ipv4`), written
/// `ADDRESS/PREFIX`.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct StaticIpv4 {
    pub address: core::net::Ipv4Addr,
    pub prefix: u8,
}

impl StaticIpv4 {
    /// `ADDRESS/PREFIX` naming a unicast host of its subnet: a prefix of 1
    /// to 32, and below /31 neither the subnet's network nor its broadcast
    /// address.
    pub fn parse(word: &str) -> Option<Self> {
        let (address, prefix) = word.split_once('/')?;
        let address: core::net::Ipv4Addr = address.parse().ok()?;
        let prefix: u8 = prefix
            .parse()
            .ok()
            .filter(|prefix| (1..=32).contains(prefix))?;
        let ipv4 = Self { address, prefix };
        let host = u32::from(address) & !ipv4.mask();
        let usable = !address.is_unspecified()
            && !address.is_multicast()
            && !address.is_broadcast()
            && (prefix >= 31 || (host != 0 && host != !ipv4.mask()));
        usable.then_some(ipv4)
    }
    fn mask(self) -> u32 {
        u32::MAX
            .checked_shl(32 - u32::from(self.prefix))
            .unwrap_or(0)
    }
    /// Whether `other` is another host of this subnet.
    pub fn neighbors(self, other: core::net::Ipv4Addr) -> bool {
        other != self.address
            && u32::from(other) & self.mask() == u32::from(self.address) & self.mask()
    }
}

impl fmt::Display for StaticIpv4 {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{}/{}", self.address, self.prefix)
    }
}

/// A `--dns` word: 1 to [`MAX_DNS_SERVERS`] distinct addresses, IPv4 or
/// IPv6, separated by commas.
pub fn dns_servers(word: &str) -> Option<Vec<core::net::IpAddr>> {
    let mut servers = Vec::new();
    for server in word.split(',') {
        let server: core::net::IpAddr = server.parse().ok()?;
        if server.is_unspecified() || server.is_multicast() || servers.contains(&server) {
            return None;
        }
        servers.push(server);
    }
    (servers.len() <= MAX_DNS_SERVERS).then_some(servers)
}

struct Servers<'a>(&'a [core::net::IpAddr]);

impl fmt::Display for Servers<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for (index, server) in self.0.iter().enumerate() {
            if index != 0 {
                f.write_str(",")?;
            }
            write!(f, "{server}")?;
        }
        Ok(())
    }
}

/// A `--wifi` word: `%XX` escapes decoded, 1 to 32 bytes of UTF-8.
fn ssid(word: &str) -> Result<String, Error> {
    const MALFORMED: Error = Error::WifiNetwork;
    let mut bytes = Vec::with_capacity(word.len());
    let mut rest = word.as_bytes();
    while let Some((&byte, tail)) = rest.split_first() {
        rest = tail;
        if byte != b'%' {
            bytes.push(byte);
            continue;
        }
        let digits = rest.get(..2).ok_or(MALFORMED)?;
        let digits = core::str::from_utf8(digits).map_err(|_| MALFORMED)?;
        bytes.push(u8::from_str_radix(digits, 16).map_err(|_| MALFORMED)?);
        rest = &rest[2..];
    }
    if !SSID_BYTES.contains(&bytes.len()) {
        return Err(MALFORMED);
    }
    String::from_utf8(bytes).map_err(|_| MALFORMED)
}

/// An SSID as one `--wifi` word: `%`, whitespace and control characters as
/// `%XX`.
struct Ssid<'a>(&'a str);

impl fmt::Display for Ssid<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        for character in self.0.chars() {
            if character == '%' || character.is_whitespace() || character.is_control() {
                let mut bytes = [0; 4];
                for byte in character.encode_utf8(&mut bytes).bytes() {
                    write!(f, "%{byte:02X}")?;
                }
            } else {
                write!(f, "{character}")?;
            }
        }
        Ok(())
    }
}

fn flag(set: &mut bool, duplicate: Error) -> Result<(), Error> {
    if core::mem::replace(set, true) {
        return Err(duplicate);
    }
    Ok(())
}

impl System {
    /// This boot starts the macOS the Preboot disk holds, which needs a GPU;
    /// its recoveryOS and a staged update's installer show on the firmware's
    /// display, as a restore does.
    pub fn boots_macos(&self) -> bool {
        self.preboot.is_some() && self.recovery.is_none() && !self.upgrade
    }

    /// Select the exact number of host processors the loader reserves.
    /// A disk-derived count is rechecked against the live source before SIPI.
    pub fn assigned_native_cores(
        &self,
        available: usize,
        verified_bundle_pes: Option<usize>,
    ) -> crate::Result<usize> {
        use crate::topology::{assignment_count, guest_assignment_count};
        use stela_contracts::refuse;
        if self.native_bootstrap_check || self.native_shared_check {
            let count = self.native_cores.ok_or(refuse!(
                "native bootstrap check needs an explicit processor count"
            ))?;
            return assignment_count(Some(count), available);
        }
        if !self.parallel && !self.native_rounds {
            if self.native_cores.is_some_and(|count| count != 1) {
                return Err(refuse!(
                    "serialized guest execution claims exactly one native core"
                ));
            }
            return assignment_count(Some(1), available);
        }
        let source_pes = if let Some(verified) = verified_bundle_pes {
            verified
        } else if self.preboot.is_some() {
            self.native_cores.or(self.guest_pes).ok_or(refuse!(
                "Preboot CPU count is unavailable before reservation; run stela setup repair or specify --guest-pes N"
            ))?
        } else {
            1
        };
        guest_assignment_count(self.native_cores, available, source_pes)
    }

    /// Join the reservation to the verified live topology before starting APs.
    pub fn validate_native_cores(
        &self,
        available: usize,
        reserved: usize,
        source_pes: usize,
    ) -> crate::Result<()> {
        use stela_contracts::refuse;
        if self.preboot.is_some()
            && (self.parallel || self.native_rounds)
            && self.native_cores.is_none()
            && self.guest_pes != Some(source_pes)
        {
            return Err(refuse!(
                "Preboot CPU count changed; run stela setup repair or update --guest-pes N"
            ));
        }
        // Fewer cores than assigned run when firmware leaves no AP startup
        // page or an AP fails to start.
        if reserved == 0 || reserved > self.assigned_native_cores(available, Some(source_pes))? {
            return Err(refuse!(
                "verified live source and reserved native core count disagree; check --guest-pes and --native-cores"
            ));
        }
        Ok(())
    }

    /// The line starts with `--system`.
    pub fn is_system(text: &str) -> bool {
        text.split_ascii_whitespace().next() == Some(SYSTEM)
    }

    /// Parse a line the mount runs: `parse`, refusing an unreplaced `@disk`.
    pub fn parse_resolved(text: &str) -> Result<Self, Error> {
        let options = Self::parse(text)?;
        if [&options.storage, &options.nor, &options.preboot]
            .into_iter()
            .flatten()
            .any(is_chosen)
        {
            return Err(Error::Constraint(Constraint::UnresolvedDisk));
        }
        Ok(options)
    }

    /// The first authored fixture requested by these typed options. Shared
    /// parsing remains stable across mounts; production mounts reject a fixture
    /// before taking over hardware.
    pub fn diagnostic_option(&self) -> Option<&'static str> {
        [
            (self.native_bootstrap_check, "--native-bootstrap-check"),
            (self.native_shared_check, "--native-shared-check"),
            (self.audio_check.is_some(), "--audio-check"),
            (self.audio_own_hpet, "--audio-own-hpet"),
            (self.network_check.is_some(), "--network-check"),
        ]
        .into_iter()
        .find_map(|(enabled, flag)| enabled.then_some(flag))
    }

    pub fn parse(text: &str) -> Result<Self, Error> {
        parse::system(text)
    }

    /// Check complete-record constraints in their established diagnostic order.
    /// This does not apply parsing defaults or mutate the record.
    pub fn check(&self) -> Result<(), Error> {
        validate::system(self)
    }
}

struct Line<'a, 'b>(&'a mut fmt::Formatter<'b>);

impl Line<'_, '_> {
    fn flag(&mut self, name: &str, set: bool) -> fmt::Result {
        if set {
            write!(self.0, " {name}")?;
        }
        Ok(())
    }
    fn value(&mut self, name: &str, value: Option<impl fmt::Display>) -> fmt::Result {
        if let Some(value) = value {
            write!(self.0, " {name} {value}")?;
        }
        Ok(())
    }
}

struct Partition<'a>(&'a Target);

fn written(target: &Option<Target>) -> Option<Partition<'_>> {
    target.as_ref().map(Partition)
}

impl fmt::Display for Partition<'_> {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        if is_chosen(self.0) {
            return f.write_str(mount_interface::CHOSEN_DISK);
        }
        write!(f, "{}:{}", self.0.serial, self.0.partition)
    }
}

/// The partition `stela.efi`'s Install macOS chooses, written `@disk`
/// (`mount_interface::CHOSEN_DISK`). The mount replaces each `@disk` with the
/// chosen `SERIAL:GUID` before the engine reads the line
/// (`mount_interface::with_disk`).
pub fn chosen_disk() -> Target {
    Target {
        serial: String::from(mount_interface::CHOSEN_DISK),
        partition: Guid([0; 16]),
    }
}

/// `target` is `chosen_disk`.
pub fn is_chosen(target: &Target) -> bool {
    target.serial == mount_interface::CHOSEN_DISK && target.partition.is_zero()
}

fn partition(value: &str, argument: TargetArgument) -> Result<Target, Error> {
    if value == mount_interface::CHOSEN_DISK {
        return Ok(chosen_disk());
    }
    value
        .parse()
        .map_err(|source| Error::Target { argument, source })
}

struct Hex(u64);

impl fmt::Display for Hex {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#x}", self.0)
    }
}

/// An ECID as 16 hexadecimal digits, as `stela setup` prints it.
struct Ecid(u64);

impl fmt::Display for Ecid {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        write!(f, "{:#018x}", self.0)
    }
}

/// The canonical option line: `--system`, then each option that differs
/// from its default, in a fixed order. `System::parse` reads it back to an
/// equal record.
impl fmt::Display for System {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(SYSTEM)?;
        let mut line = Line(f);
        line.value("--storage", written(&self.storage))?;
        line.value("--storage-ram", self.storage_ram.map(Hex))?;
        line.value("--nor", written(&self.nor))?;
        if self.preboot.is_some() {
            line.value("--preboot", written(&self.preboot))?;
        }
        line.value("--bundle", self.bundle_directory.as_deref())?;
        line.value("--ecid", self.ecid.map(Ecid))?;
        line.value(
            boot_selection::BOOT_VOLUME_OPTION,
            self.boot_volume
                .as_ref()
                .map(|volume| String::from_utf8_lossy(&volume.encode()).into_owned()),
        )?;
        line.value(
            boot_selection::BOOT_COMMAND_OPTION,
            self.boot_command.map(|command| command as u8),
        )?;
        line.flag(boot_selection::SAFE_BOOT_OPTION, self.safe_boot)?;
        line.flag(MAINTENANCE_WRITE, self.maintenance_write)?;
        line.flag(SINGLE_USER, self.single_user)?;
        line.flag(boot_selection::BUNDLE_REPAIR_OPTION, self.bundle_repair)?;
        line.flag(SCANOUT_MIRROR, self.scanout_mirror)?;
        line.flag(boot_selection::UPGRADE_OPTION, self.upgrade)?;
        line.flag(
            boot_selection::PAIRED_RECOVERY_OPTION,
            self.recovery == Some(recovery::Kind::Paired),
        )?;
        line.flag(
            boot_selection::SYSTEM_RECOVERY_OPTION,
            self.recovery == Some(recovery::Kind::System),
        )?;
        line.flag("--restore-study", self.restore)?;
        line.value("--restore-source", written(&self.restore_source))?;
        line.value("--restore-manifest", self.restore_manifest_path.as_deref())?;
        line.value("--restore-identity", self.restore_identity)?;
        line.value(INSTALL, self.install.as_deref())?;
        line.value(INSTALL_SOURCE, self.install_source)?;
        line.flag(VERIFY_ON_READ, self.verify_on_read)?;
        line.flag(UPDATE_RESTORE, self.update_restore)?;
        line.flag("--development-restore", self.development_restore)?;
        for serial in &self.usb_storage {
            line.value("--usb-storage", Some(serial))?;
        }
        for uuid in &self.usb4_approved {
            line.value(USB4_APPROVE, Some(uuid))?;
        }
        line.flag(EXTERNAL_DISKS, self.external_disks)?;
        line.flag("--guest-network", self.guest_network)?;
        line.value(WIFI, self.ssid.as_deref().map(Ssid))?;
        line.value(
            WIFI_KEY,
            self.wifi_key
                .as_deref()
                .filter(|&path| path != DEFAULT_WIFI_KEY),
        )?;
        line.value(WIFI_COUNTRY, self.wifi_country.as_deref())?;
        line.value(IPV4, self.ipv4)?;
        line.value(IPV4_GATEWAY, self.ipv4_gateway)?;
        line.value(DNS, (!self.dns.is_empty()).then(|| Servers(&self.dns)))?;
        line.flag("--input", self.input)?;
        line.flag("--no-guest-bluetooth", self.no_guest_bluetooth)?;
        line.flag("--idle-skip", self.idle_skip)?;
        line.flag("--survey", self.survey)?;
        line.flag("--sep-observe", self.sep_observe)?;
        line.value(NATIVE_CORES, self.native_cores)?;
        line.value(GUEST_PES, self.guest_pes)?;
        line.flag("--native-rounds", self.native_rounds)?;
        line.flag("--native-artifacts", self.native_artifacts)?;
        line.flag("--native-bootstrap-check", self.native_bootstrap_check)?;
        line.flag("--native-shared-check", self.native_shared_check)?;
        let implied_serial =
            self.native_rounds || self.native_shared_check || self.native_bootstrap_check;
        line.flag("--serialized", !self.parallel && !implied_serial)?;
        line.flag("--profile", self.profile)?;
        line.flag("--monitor-trace", self.monitor_trace)?;
        line.flag("--display-trace", self.display_trace)?;
        line.value("--network-check", self.network_check.as_deref())?;
        line.value("--audio-check", self.audio_check.map(AudioCheck::name))?;
        line.flag("--audio-own-hpet", self.audio_own_hpet)?;
        line.flag("--native-pci-own-ioapic", self.native_pci_own_ioapic)?;
        if self.dram_explicit {
            line.value("--dram-base", Some(Hex(self.dram_base)))?;
        }
        for (name, value, default) in [
            ("--load", self.load, DRAM_BASE),
            ("--entry", self.entry, DRAM_BASE),
            ("--x0", self.x0, 0),
            ("--budget", self.budget, 0),
        ] {
            if value != default {
                line.value(name, Some(Hex(value)))?;
            }
        }
        for &address in &self.breakpoints {
            line.value("--break", Some(Hex(address)))?;
        }
        for &address in &self.traces {
            line.value("--trace", Some(Hex(address)))?;
        }
        for &(base, bytes) in &self.watches {
            write!(line.0, " --watch {base:#x},{bytes:#x}")?;
        }
        for &(base, bytes) in &self.dumps {
            write!(line.0, " --dump {base},{bytes:#x}")?;
        }
        Ok(())
    }
}

pub const HOST_CHECK: &str = "--host-check";
const DEFAULT_KEY_SECONDS: u64 = 60;

/// The options of a `--host-check` line: the standalone report on the PC
/// and its diagnostics.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct HostCheck {
    /// Read the NOR image from this FAT32 partition's file (`--nor-file
    /// GUID:/PATH`).
    pub nor_file: Option<(Guid, String)>,
    /// Stop after this many seconds (`--deadline`).
    pub deadline: Option<u64>,
    /// How long a prompt waits for a key (`--key-seconds`).
    pub key_seconds: u64,
}

impl Default for HostCheck {
    fn default() -> Self {
        Self {
            nor_file: None,
            deadline: None,
            key_seconds: DEFAULT_KEY_SECONDS,
        }
    }
}

fn seconds(value: Option<&str>) -> Result<u64, Error> {
    value
        .and_then(|value| value.parse().ok())
        .filter(|&seconds| seconds > 0)
        .ok_or(Error::HostSeconds)
}

impl HostCheck {
    pub fn parse(text: &str) -> Result<Self, Error> {
        let mut words = text.split_ascii_whitespace();
        if words.next() != Some(HOST_CHECK) {
            return Err(Error::HostPrefix);
        }
        let mut options = Self::default();
        while let Some(word) = words.next() {
            match word {
                "--nor-file" => {
                    let (guid, path) = words
                        .next()
                        .and_then(|value| value.split_once(':'))
                        .ok_or(Error::HostNorSpecification)?;
                    options.nor_file = Some((
                        guid.parse().map_err(|source| Error::Guid {
                            argument: GuidArgument::HostNorFile,
                            source,
                        })?,
                        String::from(path),
                    ));
                }
                "--deadline" => options.deadline = Some(seconds(words.next())?),
                "--key-seconds" => options.key_seconds = seconds(words.next())?,
                _ => return Err(Error::UnknownHostOption),
            }
        }
        Ok(options)
    }
}

impl fmt::Display for HostCheck {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        f.write_str(HOST_CHECK)?;
        let mut line = Line(f);
        if let Some((guid, path)) = &self.nor_file {
            write!(line.0, " --nor-file {guid}:{path}")?;
        }
        line.value("--deadline", self.deadline)?;
        if self.key_seconds != DEFAULT_KEY_SECONDS {
            line.value("--key-seconds", Some(self.key_seconds))?;
        }
        Ok(())
    }
}

#[cfg(test)]
mod tests;
