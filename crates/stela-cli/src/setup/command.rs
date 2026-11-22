//! `stela`'s command line. Plain `stela` looks at the plugged-in sticks and
//! offers the next step; each explicit command is what that menu runs, with
//! only the options it takes, so the parser refuses every other combination.
//! Each form becomes one [`Request`] for the action it names. Options for
//! developers and for recovering a lost stick stay hidden but working.
use super::{Action, NextBoot, Request, builds, ecid::Ecid, wifi};
use clap::{Args, Subcommand, builder::TypedValueParser};
use std::path::PathBuf;
use stela_boot_core::options::Uuid;

/// The options every command takes.
#[derive(Clone, Copy, Default)]
pub(crate) struct Global {
    pub(crate) dry_run: bool,
    pub(crate) yes: bool,
}

/// One of `stela`'s commands.
#[derive(Subcommand)]
pub(crate) enum Command {
    /// Erase a stick and make a macOS installer on it
    #[command(
        long_about = "Erase a stick and make a macOS installer on it. Without a terminal,\n\
                      erasing needs --confirm-disk."
    )]
    Create(Box<Create>),
    /// Bring a stick to this version of Stela
    #[command(
        long_about = "Replace the stick's Stela with this version and prepare its boot\n\
                      files again when needed. Downloads only the boot files it needs."
    )]
    Update(Update),
    /// Fix what is damaged or missing; finish a stopped create or update
    #[command(
        long_about = "Check the stick and put back what is damaged or missing: settings,\n\
                      boot files, macOS, firmware the last boot asked for.\n\
                      Finishes an interrupted create or update."
    )]
    Repair(Repair),
    /// Show what is on a stick and whether it is healthy; changes nothing
    #[command(
        long_about = "Show what is on the stick and whether it is healthy. Changes nothing.\n\
                      Exits 1 when something needs doing, and names the command."
    )]
    Status(Status),
    /// Change a stick's Wi-Fi, network addresses and approved USB4 devices
    #[command(long_about = "Without options, asks for each setting on a terminal.")]
    Settings(Settings),
    /// Save the boot logs for a bug report
    #[command(
        long_about = "Read every boot log on this computer's disks: the stick's and each\n\
                      install's. PATH limits it to one disk, volume, disk image or boot.log\n\
                      copy. On a terminal the logs are saved to log.txt in this folder, the\n\
                      file to attach to a report; otherwise they are printed."
    )]
    Log(Log),
    /// Remove everything Stela wrote to a stick.
    #[command(hide = true)]
    Uninstall(Stick),
    /// Change an install's NVRAM on its machine-state partition, which is
    /// on the PC's own disk.
    #[command(hide = true, subcommand)]
    Nvram(Nvram),
    /// Back up an install's machine state, or write a backup back.
    #[command(hide = true, subcommand)]
    State(State),
    /// Install an authorized local host-device firmware package
    /// (manifest.json and resources) onto the stick.
    #[command(hide = true)]
    HostFirmware {
        #[arg(value_name = "DIRECTORY")]
        directory: PathBuf,
        #[command(flatten)]
        stick: Stick,
    },
}

impl Command {
    /// The request the command line names.
    pub(crate) fn request(self, global: Global) -> Result<Request, clap::Error> {
        let mut request = match self {
            Self::Create(command) => command.request(),
            Self::Update(command) => command.request(),
            Self::Repair(command) => command.request(),
            Self::Status(command) => Request {
                full: command.full,
                ..command.stick.request(Action::Status)
            },
            Self::Settings(command) => command.request()?,
            Self::Log(command) => command.request(),
            Self::Uninstall(stick) => stick.request(Action::Uninstall),
            Self::Nvram(command) => command.request(),
            Self::State(command) => command.request(),
            Self::HostFirmware { directory, stick } => {
                stick.request(Action::HostFirmwareInstall(directory))
            }
        };
        request.dry_run = global.dry_run;
        request.yes = global.yes;
        Ok(request)
    }
}

/// A partition's name, given as is or by its device path (`/dev/disk1s2`,
/// `/dev/sdb1`), as STICK takes a whole disk.
fn partition_name(value: &str) -> Result<String, String> {
    Ok(value.strip_prefix("/dev/").unwrap_or(value).to_owned())
}

fn path() -> impl TypedValueParser<Value = PathBuf> {
    clap::builder::OsStringValueParser::new().map(PathBuf::from)
}

/// The stick a command acts on: without it, the one plugged-in stick that
/// holds Stela.
#[derive(Args)]
pub(crate) struct Stick {
    /// The stick's whole disk or one of its mounted volumes.
    /// It can be left out when only one stick fits.
    #[arg(
        value_name = "STICK",
        value_parser = path(),
        help = "The stick, when several fit",
        verbatim_doc_comment
    )]
    stick: Option<PathBuf>,
}

impl Stick {
    fn request(self, action: Action) -> Request {
        Request {
            action,
            usb: self.stick.unwrap_or_default(),
            ..Request::default()
        }
    }
}

/// The macOS a stick installs, by version (27.2), build (26B123) or
/// `latest`, as the build.
fn macos(value: &str) -> Result<String, String> {
    if value.eq_ignore_ascii_case("latest") {
        return Ok(builds::DEFAULT.build.into());
    }
    if let Some(entry) = builds::SUPPORTED
        .iter()
        .find(|entry| entry.version == value)
    {
        return Ok(entry.build.into());
    }
    if value.is_empty() || !value.chars().all(|c| c.is_ascii_alphanumeric() || c == '.') {
        return Err("a macOS version such as 27.2, a build such as 26B123, or latest".into());
    }
    Ok(value.to_ascii_uppercase())
}

/// `--wifi`: a network's name, or `none`.
#[derive(Clone)]
enum Join {
    None,
    Network(String),
}

fn join(value: &str) -> Result<Join, String> {
    if value.eq_ignore_ascii_case("none") {
        return Ok(Join::None);
    }
    wifi::ssid(value).map(Join::Network)
}

/// The Wi-Fi network `stela.efi` joins.
#[derive(Args)]
struct Wifi {
    /// Wi-Fi to join during the install, or none. The passphrase is asked
    /// for and stored in plaintext on the stick.
    #[arg(
        long,
        value_name = "SSID|none",
        value_parser = join,
        help = "Wi-Fi for the install, or none",
        verbatim_doc_comment
    )]
    wifi: Option<Join>,
    /// Read the Wi-Fi passphrase from FILE
    #[arg(long, value_name = "FILE")]
    wifi_passphrase_file: Option<PathBuf>,
    /// The country whose Wi-Fi rules apply, as an ISO 3166 code such as US
    /// or DE: this computer's region by default.
    #[arg(long, value_name = "CODE", value_parser = wifi::country, hide = true)]
    wifi_country: Option<String>,
}

impl Wifi {
    fn apply(self, request: &mut Request) {
        match self.wifi {
            Some(Join::Network(ssid)) => request.wifi = Some(ssid),
            Some(Join::None) => request.no_wifi = true,
            None => {}
        }
        request.wifi_passphrase_file = self.wifi_passphrase_file;
        request.wifi_country = self.wifi_country;
    }
}

/// A USB4 device's UUID, in the spelling the boot log prints, or `none`.
fn usb4_device(text: &str) -> Result<String, String> {
    if text.eq_ignore_ascii_case("none") {
        return Ok("none".into());
    }
    Uuid::parse(text)
        .map(|uuid| uuid.to_string())
        .ok_or_else(|| "a USB4 device is named by its UUID, as the boot log prints it".into())
}

/// Apply `--usb4-allow`: `none` alone withdraws every approval.
fn apply_usb4(request: &mut Request, devices: Vec<String>) -> Result<(), clap::Error> {
    if devices.iter().any(|device| device == "none") {
        if devices.len() > 1 {
            return Err(clap::Error::raw(
                clap::error::ErrorKind::ArgumentConflict,
                "--usb4-allow none cannot be given with a device\n",
            ));
        }
        request.no_usb4_approve = true;
    } else {
        request.usb4_approve = devices;
    }
    Ok(())
}

/// `--ipv4` or `--dns`: a value, or `auto` for DHCP's.
#[derive(Clone)]
enum Automatic<T> {
    Auto,
    Set(T),
}

fn ipv4(word: &str) -> Result<Automatic<stela_boot_core::options::StaticIpv4>, String> {
    if word.eq_ignore_ascii_case("auto") {
        return Ok(Automatic::Auto);
    }
    super::addresses::ipv4(word).map(Automatic::Set)
}

fn dns(word: &str) -> Result<Automatic<Vec<std::net::IpAddr>>, String> {
    if word.eq_ignore_ascii_case("auto") {
        return Ok(Automatic::Auto);
    }
    super::addresses::servers(word).map(|servers| Automatic::Set(servers.0))
}

/// The addresses `stela.efi`'s network uses instead of DHCP's.
#[derive(Args)]
struct Addressing {
    /// Static IPv4 address, or auto for DHCP
    #[arg(long, value_name = "ADDRESS/PREFIX|auto", value_parser = ipv4)]
    ipv4: Option<Automatic<stela_boot_core::options::StaticIpv4>>,
    /// IPv4 router, with --ipv4
    #[arg(long, value_name = "ADDRESS", requires = "ipv4")]
    gateway: Option<std::net::Ipv4Addr>,
    /// Up to 3 DNS servers, comma-separated, or auto for DHCP's
    #[arg(
        long,
        value_name = "SERVERS|auto",
        value_parser = dns,
        help = "DNS servers (up to 3), or auto"
    )]
    dns: Option<Automatic<Vec<std::net::IpAddr>>>,
}

impl Addressing {
    fn apply(self, request: &mut Request) {
        match self.ipv4 {
            Some(Automatic::Set(ipv4)) => request.ipv4 = Some(ipv4),
            Some(Automatic::Auto) => request.automatic_ipv4 = true,
            None => {}
        }
        request.ipv4_gateway = self.gateway;
        match self.dns {
            Some(Automatic::Set(servers)) => request.dns = Some(servers),
            Some(Automatic::Auto) => request.automatic_dns = true,
            None => {}
        }
    }
}

/// The same addresses, hidden: a new installer takes DHCP's unless a
/// script names them; `stela settings` changes them.
#[derive(Args)]
struct HiddenAddressing {
    #[arg(long, value_name = "ADDRESS/PREFIX", value_parser = super::addresses::ipv4, hide = true)]
    ipv4: Option<stela_boot_core::options::StaticIpv4>,
    #[arg(long, value_name = "ADDRESS", requires = "ipv4", hide = true)]
    gateway: Option<std::net::Ipv4Addr>,
    #[arg(long, value_name = "SERVERS", value_parser = super::addresses::servers, hide = true)]
    dns: Option<super::addresses::Servers>,
}

/// How long the startup choices wait.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub(super) enum Wait {
    /// As setup chooses (`super::CHOOSER_SECONDS` with several installs).
    Automatic,
    Seconds(u32),
}

fn wait(value: &str) -> Result<Wait, String> {
    if value.eq_ignore_ascii_case("auto") {
        return Ok(Wait::Automatic);
    }
    value
        .parse()
        .map(Wait::Seconds)
        .map_err(|_| "a number of seconds, or auto".into())
}

fn ecid(value: &str) -> Result<Ecid, String> {
    Ecid::parse(value).map_err(|e| e.to_string())
}

/// An install's machine state.
#[derive(Args)]
pub(crate) struct Machine {
    #[command(flatten)]
    stick: Stick,
    /// The install to act on (the default install otherwise).
    #[arg(long)]
    install: Option<String>,
    /// The install's machine-state partition, when this computer names it
    /// differently than setup found it by its GUID and disk serial.
    #[arg(long, value_parser = partition_name)]
    nor: Option<String>,
}

impl Machine {
    fn request(self, action: Action) -> Request {
        Request {
            install: self.install,
            nor: self.nor,
            ..self.stick.request(action)
        }
    }
}

/// `stela create`.
#[derive(Args)]
pub(crate) struct Create {
    /// The stick's whole disk (/dev/sdb, disk5 or PhysicalDrive2), which
    /// is erased, or one of its mounted volumes. Without it, stela asks.
    #[arg(
        value_name = "STICK",
        value_parser = path(),
        help = "The stick to erase",
        verbatim_doc_comment
    )]
    stick: Option<PathBuf>,
    /// Offline copies macOS to the stick (32 GB or more); the install
    /// needs no internet. Online downloads macOS during the install.
    /// Default: offline when it fits.
    #[arg(
        help = "The installer to make",
        verbatim_doc_comment,
        long,
        value_enum,
        value_name = "offline|online",
        hide_possible_values = true
    )]
    installer: Option<super::InstallImage>,
    /// macOS to install [default: latest tested]
    #[arg(long, value_name = "VERSION|BUILD|latest", value_parser = macos)]
    macos: Option<String>,
    /// Use this IPSW instead of downloading
    #[arg(long, value_name = "FILE")]
    ipsw: Option<PathBuf>,
    #[command(flatten)]
    wifi: Wifi,
    /// The stick's model or size, typed back: needed without a terminal,
    /// and for disks that are not USB sticks
    #[arg(
        long,
        value_name = "MODEL|SIZE",
        help = "The stick's model or size, typed back",
        verbatim_doc_comment
    )]
    confirm_disk: Option<String>,
    /// Where downloaded images are kept: Stela's per-user cache by default.
    #[arg(long, hide = true)]
    download_dir: Option<PathBuf>,
    /// Give this USB4 device a PCIe tunnel on PCs without an IOMMU that
    /// confines it.
    #[arg(long, value_name = "UUID", value_parser = usb4_device, hide = true)]
    usb4_allow: Vec<String>,
    #[command(flatten)]
    addressing: HiddenAddressing,
    /// The `stela.efi` to install instead of the one built into `stela`.
    #[arg(long, hide = true)]
    efi: Option<PathBuf>,
    /// The Mac model Stela presents: Mac14,3 (Mac mini M2), the one
    /// supported now.
    #[arg(long, hide = true)]
    model: Option<String>,
    /// The PC's partition that becomes the Mac's internal SSD, when stela
    /// runs on the PC itself; on an existing setup it adds another install.
    #[arg(long, value_parser = partition_name, hide = true)]
    target: Option<String>,
    /// A separate PC partition for the machine state instead of the
    /// target's last bytes (at least 14.7 MB).
    #[arg(long, value_parser = partition_name, requires = "target", hide = true)]
    nor: Option<String>,
    /// The serial number of the disk holding `--target` and `--nor`, when
    /// this computer cannot read it.
    #[arg(long, requires = "target", hide = true)]
    disk_serial: Option<String>,
    /// Erase the other file system a `--target` holds.
    #[arg(long, requires = "target", hide = true)]
    erase_target: bool,
    /// Write fresh machine state over bytes that are neither blank nor
    /// Stela's machine state.
    #[arg(long, requires = "target", hide = true)]
    initialize_nor: bool,
    /// The new install's name (macOS by default).
    #[arg(long, hide = true)]
    install: Option<String>,
    /// The ECID of the machine being set up, in hexadecimal: reuse a known
    /// machine's (from its old stick's EFI/stela/setup.json or a `state
    /// backup` file).
    #[arg(long, value_parser = ecid, hide = true)]
    ecid: Option<Ecid>,
    /// Give the machine a new ECID although its machine state belongs to
    /// another one.
    #[arg(long, hide = true)]
    new_ecid: bool,
    /// Have every startup choice time the engine (its `--profile`).
    #[arg(long, hide = true)]
    profile: bool,
    /// Boot the install with SIP filesystem protection off, for maintenance
    /// such as writing the system TCC database.
    #[arg(long, hide = true)]
    maintenance_write: bool,
    /// Have the install refuse an IPSW without recorded block hashes.
    #[arg(long, requires = "target", hide = true)]
    verify_on_read: bool,
    /// The bound of each translation pack `stela.efi` keeps on the stick,
    /// in bytes (64 MiB to the FAT32 file limit; 0 for none).
    #[arg(long, value_name = "BYTES", hide = true)]
    translation_store_bytes: Option<u64>,
}

impl Create {
    fn request(self) -> Request {
        let mut request = Request {
            action: Action::Create,
            usb: self.stick.unwrap_or_default(),
            install_image: self.installer,
            fetch: self.ipsw.is_none(),
            ipsw: self.ipsw,
            build: self.macos.unwrap_or_else(|| builds::DEFAULT.build.into()),
            download_dir: self.download_dir,
            efi: self.efi,
            model: self.model,
            target: self.target,
            nor: self.nor,
            disk_serial: self.disk_serial,
            erase_target: self.erase_target,
            initialize_nor: self.initialize_nor,
            install: self.install,
            ecid: self.ecid,
            new_ecid: self.new_ecid,
            profile: self.profile,
            maintenance_write: self.maintenance_write.then_some(true),
            verify_on_read: self.verify_on_read,
            translation_store_bytes: self.translation_store_bytes,
            confirm_disk: self.confirm_disk,
            usb4_approve: self
                .usb4_allow
                .into_iter()
                .filter(|device| device != "none")
                .collect(),
            ipv4: self.addressing.ipv4,
            ipv4_gateway: self.addressing.gateway,
            dns: self.addressing.dns.map(|servers| servers.0),
            ..Request::default()
        };
        self.wifi.apply(&mut request);
        request
    }
}

/// `stela update`.
#[derive(Args)]
pub(crate) struct Update {
    #[command(flatten)]
    stick: Stick,
    /// Also change the macOS the stick installs. The installed macOS is
    /// not changed; it updates itself.
    #[arg(
        long,
        value_name = "VERSION|BUILD|latest",
        value_parser = macos,
        help = "Also change the macOS it installs",
        verbatim_doc_comment
    )]
    macos: Option<String>,
    /// Use this IPSW instead of downloading
    #[arg(long, value_name = "FILE")]
    ipsw: Option<PathBuf>,
    /// Where downloaded images are kept: Stela's per-user cache by default.
    #[arg(long, hide = true)]
    download_dir: Option<PathBuf>,
    /// The `stela.efi` to install instead of the one built into `stela`.
    #[arg(long, hide = true)]
    efi: Option<PathBuf>,
}

impl Update {
    fn request(self) -> Request {
        Request {
            ipsw: self.ipsw,
            download_dir: self.download_dir,
            efi: self.efi,
            macos: self.macos,
            ..self.stick.request(Action::UpdateStela)
        }
    }
}

/// `stela repair`.
#[derive(Args)]
pub(crate) struct Repair {
    #[command(flatten)]
    stick: Stick,
    /// Read all of macOS on the stick (slow). Done by itself when the
    /// last install reported a damaged part.
    #[arg(
        long,
        help = "Read all of macOS on the stick (slow)",
        verbatim_doc_comment
    )]
    full: bool,
    /// Use this IPSW instead of downloading
    #[arg(long, value_name = "FILE")]
    ipsw: Option<PathBuf>,
    /// Read the Wi-Fi passphrase from FILE when finishing an
    /// interrupted create without a terminal
    #[arg(
        long,
        value_name = "FILE",
        help = "Passphrase file, for a stopped create",
        verbatim_doc_comment
    )]
    wifi_passphrase_file: Option<PathBuf>,
    /// Where downloaded images are kept: Stela's per-user cache by default.
    #[arg(long, hide = true)]
    download_dir: Option<PathBuf>,
    /// Erase the other file system an install's SSD partition holds.
    #[arg(long, hide = true)]
    erase_target: bool,
    /// Write fresh machine state over an install's that stela.efi would
    /// refuse.
    #[arg(long, hide = true)]
    initialize_nor: bool,
    /// The machine's ECID, in hexadecimal, when its machine state cannot
    /// name it.
    #[arg(long, value_parser = ecid, hide = true)]
    ecid: Option<Ecid>,
    /// Give the machine a new ECID although its machine state belongs to
    /// another one.
    #[arg(long, hide = true)]
    new_ecid: bool,
}

impl Repair {
    fn request(self) -> Request {
        Request {
            full: self.full,
            ipsw: self.ipsw,
            wifi_passphrase_file: self.wifi_passphrase_file,
            download_dir: self.download_dir,
            erase_target: self.erase_target,
            initialize_nor: self.initialize_nor,
            ecid: self.ecid,
            new_ecid: self.new_ecid,
            ..self.stick.request(Action::Repair)
        }
    }
}

/// `stela status`.
#[derive(Args)]
pub(crate) struct Status {
    #[command(flatten)]
    stick: Stick,
    /// Also read all of macOS on the stick (slow)
    #[arg(long)]
    full: bool,
}

/// `stela settings`.
#[derive(Args)]
pub(crate) struct Settings {
    #[command(flatten)]
    stick: Stick,
    #[command(flatten)]
    wifi: Wifi,
    #[command(flatten)]
    addressing: Addressing,
    /// Allow a USB4 device on PCs without an IOMMU that confines it;
    /// repeat for more
    #[arg(
        long,
        value_name = "UUID|none",
        value_parser = usb4_device,
        help = "Allow a USB4 device; repeat for more",
        verbatim_doc_comment
    )]
    usb4_allow: Vec<String>,
    /// Boot this install by default.
    #[arg(long, hide = true)]
    default_install: Option<String>,
    /// Remove this install from the setup; its partitions are kept.
    #[arg(long, hide = true)]
    remove_install: Option<String>,
    /// Seconds `stela.efi`'s startup choices wait (0: only when a key is
    /// held), or `auto`.
    #[arg(long, value_name = "SECONDS|auto", value_parser = wait, hide = true)]
    chooser_timeout: Option<Wait>,
    /// Turn the engine profile on or off for every startup choice.
    #[arg(long, value_name = "on|off", value_parser = clap::builder::BoolishValueParser::new(), hide = true)]
    profile: Option<bool>,
    /// Turn SIP filesystem protection off or on for an install's boot.
    #[arg(long, value_name = "on|off", value_parser = clap::builder::BoolishValueParser::new(), hide = true)]
    maintenance_write: Option<bool>,
    /// The install `--maintenance-write` changes: the default one unless
    /// named.
    #[arg(long, requires = "maintenance_write", hide = true)]
    install: Option<String>,
}

impl Settings {
    fn request(self) -> Result<Request, clap::Error> {
        let mut request = Request {
            default_install: self.default_install,
            remove_install: self.remove_install,
            chooser_timeout: self.chooser_timeout,
            profile: self.profile == Some(true),
            no_profile: self.profile == Some(false),
            maintenance_write: self.maintenance_write,
            install: self.install,
            ..self.stick.request(Action::Configure)
        };
        self.wifi.apply(&mut request);
        self.addressing.apply(&mut request);
        apply_usb4(&mut request, self.usb4_allow)?;
        Ok(request)
    }
}

/// `stela log`.
#[derive(Args)]
pub(crate) struct Log {
    /// One disk, volume, disk image or boot.log copy to read instead of
    /// every disk of this computer.
    #[arg(
        value_name = "PATH",
        value_parser = path(),
        help = "One disk, volume, image or boot.log",
        verbatim_doc_comment
    )]
    path: Option<PathBuf>,
    /// Save to FILE
    #[arg(short, long, value_name = "FILE")]
    output: Option<PathBuf>,
    /// With a mounted volume of a stick: the install to read.
    #[arg(long, requires = "path", hide = true)]
    install: Option<String>,
    /// Only the machine-state partition with this unique GUID in PATH's
    /// GPT.
    #[arg(long, conflicts_with = "offset", requires = "path", hide = true)]
    partition: Option<String>,
    /// Only the machine-state partition at this byte offset in PATH (0 for
    /// a raw partition).
    #[arg(long, value_parser = crate::args::parse_address, requires = "path", hide = true)]
    offset: Option<u64>,
    /// With --partition or --offset: the partition is a Mac's SSD partition
    /// whose last bytes hold the machine state.
    #[arg(long, hide = true)]
    in_target: bool,
}

impl Log {
    fn request(self) -> Request {
        Request {
            action: Action::BootLog,
            usb: self.path.unwrap_or_default(),
            install: self.install,
            output: self.output,
            log_place: crate::boot_log::Place {
                partition: self.partition,
                offset: self.offset,
                in_target: self.in_target,
            },
            ..Request::default()
        }
    }
}

#[derive(Subcommand)]
pub(crate) enum Nvram {
    /// Choose the next boot of the installed disk in its NVRAM.
    NextBoot {
        #[arg(value_enum)]
        choice: NextBoot,
        #[command(flatten)]
        machine: Machine,
    },
    /// Write a fresh NVRAM to the install's machine-state partition; its
    /// SEP state is kept.
    Reset {
        #[command(flatten)]
        machine: Machine,
    },
}

impl Nvram {
    fn request(self) -> Request {
        match self {
            Self::NextBoot { choice, machine } => machine.request(Action::NextBoot(choice)),
            Self::Reset { machine } => machine.request(Action::ResetNvram),
        }
    }
}

#[derive(Subcommand)]
pub(crate) enum State {
    /// Copy the install's machine state (NVRAM and SEP state, with its keys)
    /// to a new private file.
    Backup {
        #[arg(value_name = "FILE")]
        file: PathBuf,
        #[command(flatten)]
        machine: Machine,
    },
    /// Write a `state backup` file back to the install it came from.
    Restore {
        #[arg(value_name = "FILE")]
        file: PathBuf,
        #[command(flatten)]
        machine: Machine,
    },
}

impl State {
    fn request(self) -> Request {
        match self {
            Self::Backup { file, machine } => machine.request(Action::BackupState(file)),
            Self::Restore { file, machine } => machine.request(Action::RestoreState(file)),
        }
    }
}
