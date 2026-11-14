//! `stela`: prepare a USB stick to install macOS 27 onto
//! a disk of the PC and boot it, and keep it working over the installed
//! system's life.
//!
//! Setup writes the install medium (`setup::medium`) of the installer the
//! user chooses ([`InstallImage`]): Offline, the whole IPSW on the stick's
//! FAT32 data partition, which needs a stick of 32 GB or more; or Online,
//! the IPSW's inventory, from which `stela.efi` streams the IPSW from
//! Apple's servers during the install. Offline is the default where it
//! fits ([`offline_unavailable`]). It prepares each machine's restore bundle (`stela
//! boot prepare --restore`) and adds an Install macOS startup choice
//! (`stela.efi --install`), which restores macOS and its paired recoveryOS
//! and System Recovery onto the Mac's internal SSD, as a Mac's erase
//! restore does. An erase restore needs the macOS system image, so a
//! download's boot files (`restore::fetch_inputs`) only prepare the boot
//! bundles. The macOS choice
//! boots the installed system afterwards. The restore personalizes its root
//! tickets, its Cryptex1 ticket and the recoveryOS LocalPolicy with Apple's
//! signing server (TSS); setup asks for them while it prepares the bundle
//! (`boot::ticket::staged`), so an install with macOS on the stick needs no
//! network.
//!
//! One command checks the stick and the restore image without writing
//! anything, prepares the restore bundle on this computer, then copies it
//! to the stick's FAT32 data partition, which the PC's firmware can read and
//! which also holds the install medium and the translation store
//! (`stela/boot`, `stela/install` and `stela/store`), installs `stela.efi`
//! as the stick's boot loader (`efi`) with a `stela.cfg` that boots the
//! installed macOS, and records what it wrote in `EFI/stela/setup.json` on
//! the EFI partition. `--dry-run` stops after the checks; `uninstall`
//! removes everything the record names.
//!
//! Each Stela machine has its own ECID (`setup::ecid`), drawn once, kept in
//! the record and given to every bundle prepared for it. Without `--target`,
//! Install macOS asks for a disk of the PC in `stela.efi`, or a raw
//! partition of the stick is the Mac's internal SSD. With `--target`, setup
//! runs on the PC itself and names the PC's own disk (`setup::installed`):
//! one empty partition becomes the Mac's internal SSD and keeps the machine
//! state (NVRAM, the SEP's state, the boot log) in its last bytes, outside
//! what the guest sees, or `--nor` names a partition of its own for that
//! state. The USB then boots the installed macOS from that disk, or the
//! disk's own recoveryOS when NVRAM asks for it or the disk holds no system,
//! as a Mac does. Further disks become further installs of the same setup
//! (`setup::machines`), each its own machine, chosen at boot from
//! `stela.efi`'s startup choices. Setup writes no disk but the stick, except
//! fresh machine state with `--initialize-nor` and NVRAM with
//! `nvram next-boot`.
//!
//! An existing setup is updated in place (`setup::update`): `update stela`
//! installs a new `stela.efi` and prepares the boot bundles again when it
//! reads another schema or this `stela` prepares another bundle revision
//! than theirs, and `update os` a new macOS build's install medium and
//! restore bundles. Every path takes `--dry-run`. A `stela.efi`
//! whose mount interface (`stela_loader::mount_interface`) differs from this
//! `stela`'s is refused.
mod addresses;
mod assets;
mod bundle;
mod command;
mod create;
mod doctor;
mod ecid;
mod elevation;
mod fdr;
mod firmware;
mod format;
pub(crate) mod home;
pub(crate) mod host;
mod inputs;
mod installed;
mod machines;
mod medium;
#[cfg(unix)]
mod ownership;
#[cfg(unix)]
pub(crate) use ownership::hand_to_invoker;
mod prepare;
mod record;
mod release;
mod settings;
mod signatures;
mod sources;
mod state;
mod translations;
mod ui;
mod uninstall;
mod update;
mod wifi;
mod wizard;

pub(crate) use command::{Command, Global};
pub(crate) use elevation::hold_window;
pub(crate) use installed::NextBoot;
pub(crate) use prepare::{cache_dir, download_dir, ensure_room};

use crate::{efi, restore};
use anyhow::{Context, Result, bail, ensure};
use bundle::Kind;
use clap::Args;
use create::{apply_install, describe_install, plan_install, restore_bundles, setup, state_line};
use ecid::Ecid;
use host::{FileSystem, Host};
use prepare::{
    Prepared, check_space, copy_bundles, fetch, prepare_kind, prepare_restore, prepare_update,
    supported,
};
use record::{Record, publish, read_record, remove_bundles, write_record};
use serde::{Deserialize, Serialize};
use std::{
    fmt::Write as _,
    fs,
    path::{Path, PathBuf},
};
use stela_formats::builds;
use stela_loader::mount_interface::{self, Interface};
use uninstall::uninstall;

/// `relative`, spelled with `/`, below `base` with the host's own separators.
fn join_slashed(base: &Path, relative: &str) -> PathBuf {
    let mut path = base.to_path_buf();
    path.extend(relative.split('/'));
    path
}

/// The Mac models setup presents, with their names: the M2 Mac mini, the
/// board the engine is tested on. Other Macs are refused by name.
pub(crate) const SUPPORTED_MODELS: &[(&str, &str)] = &[("Mac14,3", "Mac mini M2")];
/// The Mac model Stela presents by default.
pub(crate) const PINNED_MODEL: &str = SUPPORTED_MODELS[0].0;

/// Refuse a Mac model `models` does not hold, naming the ones it does.
fn check_model(models: &[(&str, &str)], model: &str) -> Result<()> {
    if models.iter().any(|(supported, _)| *supported == model) {
        return Ok(());
    }
    let supported: Vec<String> = models
        .iter()
        .map(|(model, name)| format!("{model} ({name})"))
        .collect();
    bail!(
        "Mac model {model} is not supported yet; Stela supports {}",
        supported.join(", ")
    )
}
const RECORD: &str = "EFI/stela/setup.json";
/// A margin so no volume is filled to its last byte.
pub(super) const MARGIN: u64 = 64 * 1024 * 1024;
/// The boot loader `stela create` installs: `stela.efi` itself.
pub(super) const LOADER_PATH: &str = efi::LOADER;
/// The startup choices beside `stela.cfg`.
const CHOICES_PATH: &str = "EFI/stela/choices.cfg";
/// The startup choices' timeout with several installs; with one, the list
/// opens only when a key is held.
const CHOOSER_SECONDS: u32 = 5;
/// The startup choice that installs macOS onto a setup's one install.
const INSTALL_CHOICE: &str = "Install macOS";
/// The startup choice that updates a setup's one install in place.
const UPDATE_CHOICE: &str = "Update macOS";
thread_local! {
    /// Whether several plugged-in sticks hold Stela, so that a command a
    /// message names must name its stick.
    static SEVERAL: std::cell::Cell<bool> = const { std::cell::Cell::new(false) };
    /// The stick as the user named it on the command line, which hints
    /// repeat: never the volume path it was resolved to, which may hold
    /// the user's name.
    static NAMED: std::cell::RefCell<Option<String>> = const { std::cell::RefCell::new(None) };
}

/// `stela {words}` in backticks, as a user types it, with the stick at
/// `usb` only when several sticks hold Stela: the shortest command that
/// works.
fn fix(usb: &Path, words: &str) -> String {
    match SEVERAL.get() {
        true => {
            let stick = NAMED
                .with_borrow(Clone::clone)
                .unwrap_or_else(|| usb.display().to_string());
            format!("`stela {words} {}`", quoted(&stick))
        }
        false => format!("`stela {words}`"),
    }
}

/// How a message says to start over: the hidden `uninstall` is never
/// suggested.
const START_OVER: &str = "start over with `stela create` on the stick's whole disk, which \
    erases it";

/// `word` as a shell takes it as one argument.
fn quoted(word: &str) -> String {
    if word.is_empty() || word.contains(|c: char| c.is_whitespace() || c == '"') {
        format!("\"{}\"", word.replace('"', "\\\""))
    } else {
        word.into()
    }
}

/// What to do about an interrupted first setup on `usb`.
fn interrupted_text(record: &Record, usb: &Path) -> String {
    format!(
        "the first setup on {} was interrupted: {} finishes it (add --ipsw FILE to use a local \
         image). To start over, make a new installer on the stick with `stela create`",
        record.esp.display(),
        fix(usb, "repair")
    )
}

/// The download above which a run asks first: the macOS image or its
/// pieces ask, boot files and firmware do not.
const LARGE_DOWNLOAD: u64 = 1_000_000_000;

/// Before downloading `bytes` (`what`): ask on a terminal when it is more
/// than [`LARGE_DOWNLOAD`]. Without a terminal, a command that names the
/// download (`named`, such as `update --macos`) has consented by naming
/// it; any other needs `--yes`.
fn consent_download(
    environment: &Environment<'_>,
    yes: bool,
    named: bool,
    bytes: u64,
    what: &str,
) -> Result<()> {
    if bytes <= LARGE_DOWNLOAD || (named && !environment.ui.interactive()) {
        return Ok(());
    }
    let size = host::size_text(bytes);
    consent(
        environment,
        yes,
        &format!("This downloads {size} from Apple: {what}. Download it?"),
        &format!("this downloads {size} from Apple: {what}"),
    )
}

/// What finishing `record`'s interrupted create downloads, in bytes, when
/// its source can be found now.
fn resume_bytes(
    environment: &Environment<'_>,
    record: &Record,
    ipsw: Option<&Path>,
    download_dir: Option<&Path>,
) -> Option<u64> {
    let lookup = sources::Lookup {
        need: sources::need(record, &record.build, true)?,
        build: &record.build,
        model: &record.model,
        ipsw,
        download_dir,
        medium: record.medium_of(&record.build),
    };
    sources::locate(environment, &lookup)
        .ok()
        .map(|source| sources::download_bytes(&source))
}

/// Before an effect a terminal run asks about (`question`): accept it with
/// `--yes` (`yes`), ask on a terminal, and otherwise refuse with `refusal`,
/// naming `--yes`. Declining changes nothing. Erasing a disk never comes
/// here: it needs the disk typed back (`--confirm-disk`).
fn consent(environment: &Environment<'_>, yes: bool, question: &str, refusal: &str) -> Result<()> {
    if yes {
        return Ok(());
    }
    if !environment.ui.interactive() {
        bail!("{refusal}. Nothing was changed. Add --yes to accept.");
    }
    let accepted = match environment.ui.confirm(question) {
        Err(error) if ui::cancelled(&error) => false,
        answer => answer?,
    };
    match accepted {
        true => Ok(()),
        false => Err(Declined.into()),
    }
}

/// A question declined at a terminal: the person chose no, so the run
/// stops there, having changed nothing. It is an answer, not a failure:
/// the run closes with "Nothing was changed." and exits 0, as Quit in plain
/// `stela` does. Without a terminal a question is refused instead, naming
/// `--yes`, and the run fails.
#[derive(Debug)]
pub(super) struct Declined;

impl std::fmt::Display for Declined {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        f.write_str("nothing was changed")
    }
}

impl std::error::Error for Declined {}

/// How a run on `ui` ends: a [`Declined`] question closes with "Nothing was
/// changed." and succeeds; anything else is the run's result.
fn closed(ui: &dyn ui::Ui, result: Result<()>) -> Result<()> {
    match result {
        Err(error) if error.is::<Declined>() => ui.cancel("Nothing was changed."),
        result => result,
    }
}

/// A command that found problems: its whole report, which goes to standard
/// output, and a one-line summary, which is the error. Its `Display` is
/// both, so callers that only print the error lose nothing.
#[derive(Debug)]
pub(super) struct Reported {
    pub(super) report: String,
    pub(super) summary: String,
}

impl std::fmt::Display for Reported {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        write!(f, "{}{}", self.report, self.summary)
    }
}

impl std::error::Error for Reported {}

/// Split a command's result into what goes to standard output and the
/// error: a [`Reported`] failure prints its report and fails with its
/// summary alone.
fn split_report(result: Result<String>) -> (String, Result<()>) {
    match result {
        Ok(text) => (text, Ok(())),
        Err(error) => match error.downcast::<Reported>() {
            Ok(reported) => (reported.report, Err(anyhow::anyhow!(reported.summary))),
            Err(error) => (String::new(), Err(error)),
        },
    }
}

/// Write `report` to `path` instead of the terminal, and say where it went.
fn save_report(path: &Path, report: &str) -> Result<()> {
    fs::write(path, report).with_context(|| format!("cannot write {}", path.display()))?;
    #[cfg(unix)]
    hand_to_invoker(path);
    println!(
        "Saved the report to {}",
        crate::boot_log::shown(path).display()
    );
    Ok(())
}

/// The startup choice that restores with a bundle of `kind` on a setup
/// with one install.
fn restore_choice_name(kind: Kind) -> &'static str {
    match kind {
        Kind::Update => UPDATE_CHOICE,
        _ => INSTALL_CHOICE,
    }
}

/// The installer a setup makes, which the user sees as Offline or Online
/// ([`Self::name`], `--installer`): where the install reads the macOS
/// restore image.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize, clap::ValueEnum)]
#[serde(rename_all = "lowercase")]
pub(crate) enum InstallImage {
    // The whole IPSW on the stick; with the signing answers setup fetched,
    // the install needs no network. Records name it `full`, and those
    // written before that name `offline`. The doc line is `--help`'s.
    /// macOS is copied to the stick (32 GB or more); the install needs no internet
    #[serde(alias = "offline")]
    #[value(name = "offline")]
    Full,
    // The IPSW on Apple's servers, streamed during the install. Records
    // name it `network`, and those written before that `online`.
    /// The install downloads macOS, so the PC needs internet
    #[serde(alias = "online")]
    #[value(name = "online")]
    Network,
}

impl InstallImage {
    /// The installer's name wherever the user sees it.
    pub(super) fn name(self) -> &'static str {
        match self {
            Self::Full => "Offline",
            Self::Network => "Online",
        }
    }
}

/// The smallest USB, as the PC reports it, that an Offline installer is
/// made on. Sticks sold as 32 GB report from about 28.8 GB (26.8 GiB) to
/// 32 GB, and macOS 27.2 with its boot files and the firmware takes about
/// 26.5 GB, so every one of them passes; the plan's own check of the free
/// space against the image's size refuses the one that is too small after
/// all, with the sizes.
pub(super) const FULL_MIN: u64 = 26 * 1024 * 1024 * 1024;

/// Why an Offline installer cannot be made on a USB of `capacity` bytes (0
/// when unknown) from an image that holds all of macOS (`whole`) or only
/// its startup files, as the wizard and the refusal of `--installer
/// offline` say it after "Offline"; nothing when it can.
pub(super) fn offline_unavailable(capacity: u64, whole: bool) -> Option<String> {
    if !whole {
        return Some(
            "needs an IPSW file with all of macOS; this one has only its startup files".into(),
        );
    }
    (capacity != 0 && capacity < FULL_MIN).then(|| {
        format!(
            "needs a USB stick of 32 GB or more (this one is {})",
            host::size_text(capacity)
        )
    })
}

fn image_is_remote(ipsw: &Path, choice: InstallImage) -> Result<bool> {
    match choice {
        InstallImage::Network => Ok(true),
        InstallImage::Full => {
            ensure!(
                restore::payload::holds_system_image(ipsw)?,
                "{} holds only boot files, without the macOS system image this stick's \
                 install reads from it; pass the whole IPSW with --ipsw",
                ipsw.display()
            );
            Ok(false)
        }
    }
}

/// What one `stela` command does.
#[derive(Default)]
enum Action {
    /// Plain `stela`: look at the sticks and offer the next step.
    Home,
    /// Create a setup, or add an install to one with `--target`.
    #[default]
    Create,
    Status,
    Repair,
    /// `update`: bring the stick to this Stela, and with `--macos` (the
    /// request's `macos`) also to another macOS build.
    UpdateStela,
    /// Bring the stick to the request's `build` of macOS.
    UpdateOs,
    Uninstall,
    NextBoot(NextBoot),
    ResetNvram,
    BackupState(PathBuf),
    RestoreState(PathBuf),
    BootLog,
    Configure,
    HostFirmwareInstall(PathBuf),
}

/// One `stela` command: its action and the options it was given
/// (`setup::command` describes each).
#[derive(Default)]
pub(crate) struct Request {
    action: Action,
    usb: PathBuf,
    ipsw: Option<PathBuf>,
    fetch: bool,
    install_image: Option<InstallImage>,
    download_dir: Option<PathBuf>,
    efi: Option<PathBuf>,
    model: Option<String>,
    pub(crate) build: String,
    /// `update --macos`: the build the stick moves to after its Stela.
    macos: Option<String>,
    pub(crate) dry_run: bool,
    /// `--yes`: without a terminal, accept what a terminal run asks about,
    /// erasing aside.
    pub(crate) yes: bool,
    /// The bytes of a download plain `stela`'s choice named and the person
    /// chose: a download of that size is not asked about again, any other
    /// is.
    accepted_download: Option<u64>,
    target: Option<String>,
    nor: Option<String>,
    disk_serial: Option<String>,
    erase_target: bool,
    /// `--full`: read every piece of the install medium.
    full: bool,
    initialize_nor: bool,
    install: Option<String>,
    ecid: Option<Ecid>,
    new_ecid: bool,
    profile: bool,
    no_profile: bool,
    verify_on_read: bool,
    install_volume: Option<PathBuf>,
    default_install: Option<String>,
    remove_install: Option<String>,
    chooser_timeout: Option<command::Wait>,
    translation_store_bytes: Option<u64>,
    confirm_disk: Option<String>,
    /// The person running setup confirmed erasing the stick, as the wizard
    /// asks; otherwise a scripted erase at a terminal asks.
    confirmed: bool,
    wifi: Option<String>,
    wifi_passphrase_file: Option<PathBuf>,
    wifi_country: Option<String>,
    no_wifi: bool,
    usb4_approve: Vec<String>,
    no_usb4_approve: bool,
    /// `--wifi`'s passphrase, once read or typed.
    wifi_passphrase: Option<String>,
    ipv4: Option<stela_boot_core::options::StaticIpv4>,
    ipv4_gateway: Option<std::net::Ipv4Addr>,
    dns: Option<Vec<std::net::IpAddr>>,
    automatic_ipv4: bool,
    automatic_dns: bool,
    /// `log`: the file the report goes to instead of the terminal.
    output: Option<PathBuf>,
    /// `log`: one machine-state place in the file it reads.
    log_place: crate::boot_log::Place,
}

#[derive(Args)]
pub struct InstallMedium {
    /// The macOS 27 restore image (IPSW); a download receipt beside it is
    /// checked.
    #[arg(long)]
    ipsw: PathBuf,
    /// The Mac model whose identities the inventory lists.
    #[arg(long, default_value = PINNED_MODEL)]
    model: String,
    /// The mounted FAT32 volume to write `stela/install/<build>` to.
    #[arg(long)]
    volume: PathBuf,
}

impl InstallMedium {
    pub fn run(self) -> Result<()> {
        let host = host::System;
        let info = host.volume(&self.volume)?;
        ensure!(
            info.filesystem == FileSystem::Fat,
            "{} is {}, not FAT32",
            self.volume.display(),
            info.filesystem
        );
        let (_, build) = restore::payload::identity(&self.ipsw, &self.model)?;
        let bytes = fs::metadata(&self.ipsw)?.len();
        let piece_bytes = stela_formats::payload::PIECE_BYTES;
        medium::check_space(
            &host,
            medium::SpaceOptions {
                volume: &self.volume,
                build: &build,
                ipsw_bytes: bytes,
                piece_bytes,
            },
        )?;
        let expected = medium::Expected::of(restore::receipt(&self.ipsw)?.as_ref());
        let inventory = restore::payload::inventory(
            &self.ipsw,
            &self.model,
            restore::payload::KeyOptions {
                fetch: true,
                output: None,
                input: None,
            },
        )?;
        let written = medium::write(medium::WriteOptions {
            volume: &self.volume,
            ipsw: &self.ipsw,
            inventory,
            expected: &expected,
            piece_bytes,
        })?;
        let directory = medium::directory(&self.volume, &build);
        println!(
            "Install medium {}: {} pieces kept, {} written; payload.json SHA-256 {}",
            directory.display(),
            written.kept,
            written.wrote,
            written.manifest_sha256
        );
        println!(
            "Engine options: --install \\{}\\{} --install-source <the partition's GPT unique \
             GUID>",
            stela_formats::payload::install_directory(&build).replace('/', "\\"),
            stela_formats::payload::MANIFEST_FILE
        );
        if expected.sha1.is_none() {
            println!("warning: no download receipt; IPSW checked against its BuildManifest only");
        }
        Ok(())
    }
}

/// Writes a restore bundle for an IPSW, model and machine into a
/// directory, under the machine's personalized ticket.
type Prepare = dyn for<'a> Fn(prepare::RamdiskOptions<'a>) -> Result<()>;

/// An IPSW's restore inventory for a model.
type InventoryOf = dyn Fn(&Path, &str) -> Result<restore::payload::Inventory>;
/// The bytes of a build's boot files for a model on Apple's servers, and
/// of its whole image (`restore::inputs_size`).
type ImageSizeOf<'a> = dyn Fn(&str, &str) -> Result<(u64, u64)> + 'a;
/// Fetch what a command needs of a build's image for a model into a
/// directory: the boot files (`prepare::fetch_inputs`), or the boot files
/// of a whole image that is written onto the stick as it downloads
/// (`medium::download::Download`).
/// The members of an image `boot prepare` reads for a model's restore
/// bundle (`boot::restore_inputs_in`), from the image's directory and
/// BuildManifest wherever its bytes are.
type InputsOf<'a> =
    dyn Fn(&mut zip::ZipArchive<&mut dyn restore::ImageSource>, &str) -> Result<Vec<String>> + 'a;
type FetchImageOf<'a> = dyn Fn(&Path, &str, &str, sources::Need) -> Result<sources::Fetched> + 'a;

/// Inputs that the command line fixes and tests replace.
#[derive(Clone, Copy)]
struct Environment<'a> {
    host: &'a dyn Host,
    download: Option<&'a medium::download::Download>,
    /// `stela-dev boot prepare --restore`; tests use synthetic IPSWs, which
    /// have no kernel, and write a stand-in.
    prepare_restore: &'a Prepare,
    /// `stela-dev boot prepare --restore --update`, likewise.
    prepare_update: &'a Prepare,
    /// The install medium's inventory, with the public AEA keys fetched.
    inventory: &'a InventoryOf,
    /// The inventory of the IPSW's build on Apple's servers, which a remote
    /// medium names (`restore::remote_inventory`).
    remote_inventory: &'a InventoryOf,
    /// The install medium's piece size (`payload::PIECE_BYTES`).
    piece_bytes: u64,
    /// The mount interface this `stela` writes for.
    interface: Interface,
    /// The revision of the bundles this `stela` prepares
    /// (`stela_loader::boot_bundle::REVISION`).
    bundle_revision: u32,
    /// Where downloaded images may be, this user's cache first
    /// (`sources::image_caches`).
    image_caches: &'a [PathBuf],
    image_size: &'a ImageSizeOf<'a>,
    fetch_image: &'a FetchImageOf<'a>,
    /// Where questions are asked: the terminal, or no one.
    ui: &'a dyn ui::Ui,
    /// The Wi-Fi networks this computer sees or knows (`wifi::host_networks`).
    wifi_networks: &'a dyn Fn() -> Vec<String>,
    /// The country whose Wi-Fi rules this computer follows
    /// (`wifi::host_country`), for a network named without `--wifi-country`.
    wifi_country: &'a dyn Fn() -> Option<String>,
    bundle_inputs: &'a InputsOf<'a>,
    /// The macOS builds setup prepares (`stela_formats::builds::SUPPORTED`);
    /// tests add their synthetic images' builds.
    builds: &'a [builds::Build],
    /// The Mac models setup presents ([`SUPPORTED_MODELS`]); tests add
    /// their synthetic images' model.
    models: &'a [(&'a str, &'a str)],
}

impl Environment<'_> {
    /// The generation of the bundles this `stela` prepares.
    fn generation(&self) -> bundle::Generation {
        bundle::Generation {
            schema: self.interface.bundle_schema,
            revision: self.bundle_revision,
        }
    }
}

/// The `stela.efi` at `path`, which must be a UEFI application with this
/// `stela`'s mount interface.
fn read_efi(environment: &Environment<'_>, path: &Path) -> Result<Vec<u8>> {
    let image = fs::read(path).with_context(|| format!("cannot read {}", path.display()))?;
    ensure!(
        image.starts_with(b"MZ"),
        "{} is not a UEFI application",
        path.display()
    );
    if let Some(reason) =
        mount_interface::mismatch(environment.interface, mount_interface::find(&image))
    {
        bail!(
            "{} is not the stela.efi this stela installs: {reason}. Omit --efi to install the \
             stela.efi built into this stela.",
            path.display()
        );
    }
    Ok(image)
}

/// Whether `record`'s stela.efi is missing, is not the version the record
/// names, or reads another interface than this `stela` writes, or its boot
/// bundles are of an older revision than this `stela` prepares: the damage
/// `status` names `update` for, which `repair` then does first.
fn needs_update(environment: &Environment<'_>, record: &Record) -> bool {
    if record.bundle_revision < environment.bundle_revision {
        return true;
    }
    match fs::read(record.esp.join(LOADER_PATH)) {
        Err(_) => true,
        Ok(image) => {
            record.stela_version.as_deref() != Some(update::version_id(&image).as_str())
                || mount_interface::mismatch(environment.interface, mount_interface::find(&image))
                    .is_some()
        }
    }
}

/// Refuse to change a setup whose installed `stela.efi` reads another
/// interface than this `stela` writes; `update` replaces it first.
fn check_installed_efi(environment: &Environment<'_>, record: &Record) -> Result<()> {
    let tool = record.esp.join(LOADER_PATH);
    let image = fs::read(&tool).with_context(|| format!("cannot read {}", tool.display()))?;
    if let Some(reason) =
        mount_interface::mismatch(environment.interface, mount_interface::find(&image))
    {
        bail!(
            "the stela.efi on {} does not read what this stela writes ({reason}); install this \
             stela's stela.efi first with {}, or use the stela that installed it",
            record.esp.display(),
            fix(&record.esp, "update")
        );
    }
    Ok(())
}

/// Refuse to prepare bundles for `record` when its active ones are of
/// another revision than this `stela` prepares: `update` prepares
/// them all at this one.
fn check_bundle_revision(environment: &Environment<'_>, record: &Record) -> Result<()> {
    match revision_mismatch(environment, record) {
        Some(reason) => bail!("{reason}"),
        None => Ok(()),
    }
}

/// Why `record`'s active bundles are not of the revision this `stela`
/// prepares, with the command that brings them to it.
fn revision_mismatch(environment: &Environment<'_>, record: &Record) -> Option<String> {
    let (theirs, ours) = (record.bundle_revision, environment.bundle_revision);
    if theirs == ours {
        return None;
    }
    Some(if theirs < ours {
        format!(
            "this setup's boot bundles are revision {theirs}, and this stela prepares revision \
             {ours}: prepare them again with {}",
            fix(&record.esp, "update")
        )
    } else {
        format!(
            "this setup's boot bundles are revision {theirs}, newer than the revision {ours} \
             this stela prepares: use the stela that prepared them"
        )
    })
}

/// Whether `action` may write to a disk: every one but the two that only
/// read, `status` and `log`.
fn writes_disks(action: &Action) -> bool {
    !matches!(action, Action::Status | Action::BootLog)
}

/// Whether `action` acts on a stick that holds Stela, found when STICK is
/// not given: every one but `create`, which finds a stick to erase, and
/// `log`, which reads every disk.
fn finds_usb(action: &Action) -> bool {
    !matches!(action, Action::Home | Action::Create | Action::BootLog)
}

/// The mounted volume of each of this computer's disks that holds a Stela
/// setup, with the disk it is on: an EFI partition with the setup record,
/// or else a data volume with Stela's `stela/boot` directory. One per disk.
fn find_setups(host: &dyn Host) -> Result<Vec<(PathBuf, String)>> {
    let mut found = Vec::new();
    for disk in host.disks()? {
        if let Some(volume) = setup_volume(&disk) {
            found.push((volume, whole_disk(&disk)));
        }
    }
    Ok(found)
}

/// A whole disk as a user types it, with its name: `/dev/sdb (USB Flash
/// Drive)`.
fn whole_disk(disk: &host::Disk) -> String {
    format!("{}{} ({})", host::DISK_PREFIX, disk.identifier, disk.name)
}

/// The mounted volume of `disk` that holds a Stela setup ([`find_setups`]).
fn setup_volume(disk: &host::Disk) -> Option<PathBuf> {
    let mounts = || disk.partitions.iter().filter_map(|p| p.mount.as_ref());
    mounts()
        .find(|mount| mount.join(RECORD).is_file())
        .or_else(|| mounts().find(|mount| mount.join("stela/boot").is_dir()))
        .cloned()
}

/// The mounted volume that holds the setup of the whole disk `stick`
/// names; `stick` itself when it names no whole disk.
fn stick_volume(host: &dyn Host, stick: &Path) -> Result<PathBuf> {
    let Some(identifier) = host.whole_disk(stick)? else {
        return Ok(stick.to_path_buf());
    };
    let disk = host
        .disks()?
        .into_iter()
        .find(|disk| disk.identifier == identifier)
        .with_context(|| format!("this computer lists no disk {identifier}"))?;
    setup_volume(&disk).with_context(|| {
        format!(
            "no mounted volume of {} ({}) holds Stela: mount its {} volume, or make an \
             installer on it with `stela create {}`",
            disk.identifier,
            disk.name,
            create::USB_VOLUME,
            stick.display()
        )
    })
}

/// The command a user types for `action`, as an error names it.
fn command_words(action: &Action) -> &'static str {
    match action {
        Action::Repair => "repair",
        Action::UpdateStela | Action::UpdateOs => "update",
        Action::Configure => "settings",
        Action::Uninstall => "uninstall",
        _ => "status",
    }
}

/// The USB of the one disk that holds a setup ([`find_setups`]), with the
/// disk it is on. None or several are an error that lists them, naming
/// `stela {words}` with a stick; stela never guesses between two.
fn choose_setup(host: &dyn Host, words: &str) -> Result<(PathBuf, String)> {
    let found = find_setups(host)?;
    match found.as_slice() {
        [one] => Ok(one.clone()),
        [] => bail!(
            "no mounted volume on this computer holds Stela: plug in the stick and mount its \
             STELA volume, or make an installer with `stela create`"
        ),
        several => {
            let list: String = several
                .iter()
                .map(|(_, disk)| format!("  {disk}\n"))
                .collect();
            let first = several[0].1.split(" (").next().unwrap_or_default();
            bail!(
                "{} sticks hold Stela:\n{list}Name the one to use after the command, such as \
                 `stela {words} {}`",
                several.len(),
                quoted(first)
            )
        }
    }
}

/// Make the installer the wizard asked for, then say what comes next.
fn create_installer(environment: &Environment<'_>, request: Request) -> Result<()> {
    let ui = environment.ui;
    let dry_run = request.dry_run;
    let profile = request.profile;
    let (report, result) = split_report(request.dispatch(environment));
    if let Err(error) = result {
        print!("{report}");
        return Err(error);
    }
    if dry_run {
        print!("{report}");
        return ui.outro("Done.");
    }
    let profile = match profile {
        true => "\nProfiling is on. Keep EFI/stela/boot.log after your run.",
        false => "",
    };
    ui.note(
        "Next steps",
        &format!(
            "Boot the target PC from this stick and choose Install macOS.\n{}{profile}",
            create::network_next_step(&report)
        ),
    )?;
    ui.outro("Your installer stick is ready.")
}

impl Request {
    /// Plain `stela`: look at the sticks and offer the next step.
    pub(crate) fn home(global: Global) -> Self {
        Self {
            action: Action::Home,
            build: builds::DEFAULT.build.into(),
            fetch: true,
            dry_run: global.dry_run,
            yes: global.yes,
            ..Self::default()
        }
    }

    pub(crate) fn run(mut self) -> Result<()> {
        SEVERAL.set(find_setups(&host::System).is_ok_and(|found| found.len() > 1));
        // Without STICK, a maintenance command acts on the one stick that
        // holds a setup; a whole disk named is read through its volume.
        let mut found = None;
        if finds_usb(&self.action) {
            self.usb = match self.usb.as_os_str().is_empty() {
                true => {
                    let (volume, disk) = choose_setup(&host::System, command_words(&self.action))?;
                    found = Some(disk);
                    volume
                }
                false => {
                    NAMED.set(Some(self.usb.display().to_string()));
                    stick_volume(&host::System, &self.usb)?
                }
            };
        }
        // The rights to write disks, before anything is downloaded or
        // changed. A Windows relaunch opens its own window, which only the
        // wizard and `log`, whose raw disk reads need administrator
        // rights, want.
        let boot_log = matches!(self.action, Action::BootLog);
        let rights = elevation::rights(elevation::Situation {
            windows: cfg!(windows),
            administrator: elevation::is_administrator(),
            dry_run: self.dry_run,
            own_window: self.usb.as_os_str().is_empty() || boot_log,
            writes_disks: writes_disks(&self.action),
        });
        match rights {
            elevation::Rights::Enough => {}
            elevation::Rights::Relaunch => {
                elevation::relaunch()?;
                if boot_log {
                    eprintln!(
                        "Reading the disks needs administrator rights: the boot log opens in a \
                         new window."
                    );
                }
                return Ok(());
            }
            elevation::Rights::Refuse => bail!(elevation::REFUSED),
        }
        // Said once, by the run that has the rights.
        if let Some(disk) = found {
            eprintln!("Using the Stela stick {disk}");
        }
        // Under sudo, what this run downloads into the invoking user's
        // cache stays theirs, also when it stops with an error.
        #[cfg(unix)]
        let _handback = {
            let mut caches = vec![cache_dir()];
            caches.extend(self.download_dir.clone());
            ownership::Handback::start(&caches)
        };
        // `log` reads every disk without a path, and a disk, image or
        // copied log as a file; a mounted volume of a stick is read through
        // its setup below.
        // On a terminal the logs go to a file to attach; a pipe gets them.
        if boot_log && self.output.is_none() && std::io::IsTerminal::is_terminal(&std::io::stdout())
        {
            self.output = Some(crate::boot_log::default_output());
        }
        if boot_log && !self.usb.is_dir() {
            let place = std::mem::take(&mut self.log_place);
            return crate::boot_log::BootLog {
                file: (!self.usb.as_os_str().is_empty()).then(|| self.usb.clone()),
                output: self.output.take(),
                partition: place.partition,
                offset: place.offset,
                in_target: place.in_target,
            }
            .run();
        }
        debug_assert_eq!(
            stela_loader::boot_bundle::SCHEMA_VERSION,
            mount_interface::CURRENT.bundle_schema
        );
        let image_caches = sources::image_caches();
        let environment = Environment {
            host: &host::System,
            image_caches: &image_caches,
            bundle_inputs: &|archive, model| crate::boot::restore_inputs_in(archive, model),
            ui: ui::Terminal::or_none(),
            wifi_networks: &wifi::host_networks,
            wifi_country: &wifi::host_country,
            image_size: &|build, model| {
                let image = restore::select(&restore::catalog()?, Some(build), Some(model))?;
                restore::inputs_size(&image, model)
            },
            fetch_image: &|directory, build, model, need| match need {
                sources::Need::BootFiles => Ok(sources::Fetched {
                    path: prepare::fetch_inputs(directory, build, model)?,
                    download: None,
                }),
                sources::Need::WholeImage | sources::Need::Pieces => {
                    let image = restore::select(&restore::catalog()?, Some(build), Some(model))?;
                    let path = directory.join("boot.ipsw");
                    // Without a boot bundle to prepare, no boot file is read.
                    let boot = (need == sources::Need::WholeImage).then_some(path.as_path());
                    let download = medium::download::Download::prepare(image, model, boot)?;
                    Ok(sources::Fetched {
                        path,
                        download: Some(download),
                    })
                }
            },
            download: None,
            prepare_restore: &prepare_restore,
            prepare_update: &prepare_update,
            inventory: &|ipsw, model| {
                restore::payload::inventory(
                    ipsw,
                    model,
                    restore::payload::KeyOptions {
                        fetch: true,
                        output: None,
                        input: None,
                    },
                )
            },
            remote_inventory: &|ipsw, model| {
                let (_, build) = restore::payload::identity(ipsw, model)?;
                let image = restore::select(&restore::catalog()?, Some(&build), Some(model))?;
                restore::remote_inventory(&image, model, None)
            },
            piece_bytes: stela_formats::payload::PIECE_BYTES,
            interface: mount_interface::CURRENT,
            bundle_revision: stela_loader::boot_bundle::REVISION,
            builds: builds::SUPPORTED,
            models: SUPPORTED_MODELS,
        };
        closed(environment.ui, self.act(&environment))
    }

    /// Run this request in `environment`, once it has the rights it needs.
    fn act(mut self, environment: &Environment<'_>) -> Result<()> {
        let asks = environment.ui.interactive();
        let wizard = matches!(self.action, Action::Create) && self.usb.as_os_str().is_empty();
        // A stela without its stela.efi says so before anything is asked.
        if (wizard || matches!(self.action, Action::Home)) && self.efi.is_none() {
            let path = assets::resolve()?;
            read_efi(environment, &path)?;
            self.efi = Some(path);
        }
        match self.action {
            Action::Home => return home::run(environment, self),
            Action::Create if wizard => {
                ensure!(
                    asks,
                    "name the stick to erase without a terminal: stela create {} --confirm-disk \
                     SIZE",
                    host::DISK_EXAMPLE
                );
                let sticks: Vec<_> = home::sticks(environment)?
                    .into_iter()
                    .filter(|stick| stick.disk.bytes >= home::NETWORK_MIN)
                    .collect();
                ensure!(
                    !sticks.is_empty(),
                    "no USB stick of 8 GB or more found: plug one in"
                );
                let stick = match home::choose(environment.ui, sticks) {
                    Err(error) if ui::cancelled(&error) => None,
                    stick => stick?,
                };
                let Some(stick) = stick else {
                    return environment.ui.cancel("Nothing was changed.");
                };
                if !wizard::configure(environment, &mut self, &stick)? {
                    return Ok(());
                }
                return create_installer(environment, self);
            }
            // Plain `stela settings` on a terminal asks for each setting.
            Action::Configure if asks && !settings::named(&self) => {
                let record = read_record(environment, &self.usb)?;
                if !settings::ask(environment, &mut self, Some(&record))? {
                    return Ok(());
                }
            }
            _ => {}
        }
        let output = self.output.take();
        let (report, result) = split_report(self.dispatch(environment));
        if let Err(error) = result {
            print!("{report}");
            return Err(error);
        }
        if let Some(path) = output {
            save_report(&path, &report)?;
        } else {
            print!("{report}");
        }
        Ok(())
    }

    fn model(&self) -> &str {
        self.model.as_deref().unwrap_or(PINNED_MODEL)
    }

    /// Put this stela's stela.efi on the stick of `record`, preparing its
    /// boot files again when they are of another generation; `allowed`
    /// when this run already asked to change a newer Stela's stick, and
    /// `skips` when it reports and ends the device skips.
    fn update_stela(
        &self,
        environment: &Environment<'_>,
        record: Record,
        (allowed, skips): (bool, bool),
    ) -> Result<String> {
        let image = match &self.efi {
            Some(path) => path.clone(),
            None => assets::resolve()?,
        };
        // The update finds the restore image itself when it needs one,
        // downloading only its boot files, never the whole image. A local
        // image named for a macOS change is that build's, not the active
        // one's.
        update::update_stela(
            environment,
            record,
            update::UpdateOptions {
                usb: &self.usb,
                image_path: &image,
                ipsw: self.ipsw.as_deref().filter(|_| self.macos.is_none()),
                download_dir: self.download_dir.as_deref(),
                dry_run: self.dry_run,
                yes: self.yes || allowed,
                skips,
            },
        )
    }

    /// Bring the stick of `record` to the request's build of macOS; `named`
    /// when the command named that build, as `update --macos` does.
    fn update_os(
        &self,
        environment: &Environment<'_>,
        mut record: Record,
        named: bool,
    ) -> Result<String> {
        check_installed_efi(environment, &record)?;
        record.install_image = self.install_image.or(record.install_image);
        update::update_os(
            environment,
            record,
            update::OsOptions {
                usb: &self.usb,
                ipsw: self.ipsw.as_deref(),
                build: &self.build,
                download_dir: self.download_dir.as_deref(),
                dry_run: self.dry_run,
                yes: self.yes,
                named,
                accepted: self.accepted_download,
            },
        )
    }

    fn dispatch(mut self, environment: &Environment<'_>) -> Result<String> {
        let action = std::mem::take(&mut self.action);
        match action {
            Action::Create => return self.create(environment),
            Action::Status => return doctor::status(environment, &self.usb, self.full),
            Action::Uninstall => return uninstall(environment, &self.usb, self.dry_run),
            _ => {}
        }
        let record = match read_record(environment, &self.usb) {
            Ok(record) => record,
            // The logs need no record: its folder may be refused, or its
            // EFI partition not mounted.
            Err(error) if matches!(action, Action::BootLog) => {
                return state::raw_boot_log(environment.host, &self.usb, &error);
            }
            Err(error) => return Err(error),
        };
        // A maintenance download fetches the image of the model the setup
        // presents.
        self.model.get_or_insert_with(|| record.model.clone());
        // A USB that holds the whole macOS image keeps holding it: its
        // repairs and updates download whole images.
        self.install_image
            .get_or_insert_with(|| record.install_kind());
        // Every maintenance command works on the active build's files, so
        // what it downloads is that build; only a macOS update names
        // another.
        if !matches!(action, Action::UpdateOs) {
            self.build = record.build.clone();
        }
        // An older stela asks before it changes a newer stick; `update` and
        // a plain `repair` ask in their own words.
        let changes_stick = match &action {
            Action::Repair => !record.complete || record.replacing.is_some(),
            Action::UpdateOs | Action::Configure | Action::HostFirmwareInstall(_) => true,
            _ => false,
        };
        if changes_stick && !self.dry_run {
            release::allow_older(
                environment,
                &record,
                self.yes,
                "change it with this older stela",
            )?;
        }
        match action {
            // An interrupted first setup is finished as it began.
            Action::Repair if !record.complete => self.finish_create(environment, &record),
            // A macOS switch that removed the old copy for room goes on,
            // after what `status` names `update` for: the switch prepares
            // bundles of this stela's revision. The repair names no
            // download: a large one is asked about, unless the menu's
            // choice named its size.
            Action::Repair if record.replacing.is_some() => {
                let build = record.replacing.clone().unwrap_or_default();
                let mut text = String::new();
                let record = match needs_update(environment, &record) {
                    false => record,
                    true => {
                        // The question above asked to change a newer stick.
                        // Finishing a switch leaves the device skips.
                        text = self.update_stela(environment, record, (true, false))?;
                        if self.dry_run {
                            text.push_str(&format!(
                                "  then          the switch to macOS {build} goes on once this \
                                 stela.efi is in place\n"
                            ));
                            return Ok(text);
                        }
                        read_record(environment, &self.usb)?
                    }
                };
                text.push_str(&Request { build, ..self }.update_os(environment, record, false)?);
                Ok(text)
            }
            // What `status` names `update` for (`needs_update`) is put
            // right as `update` does it, and the repair goes on with the
            // rest.
            // The repair reports and ends the device skips; a dry run does
            // not reach it.
            Action::Repair if needs_update(environment, &record) => {
                let mut text = self.update_stela(environment, record, (false, self.dry_run))?;
                match self.dry_run {
                    true => text.push_str(
                        "  then          the repair checks the rest once this stela.efi is in place\n",
                    ),
                    false => text.push_str(&doctor::repair(environment, &self)?),
                }
                Ok(text)
            }
            Action::Repair => doctor::repair(environment, &self),
            Action::ResetNvram => state::reset_nvram(environment, &record, &self),
            Action::BackupState(path) => state::backup(environment, &record, &self, &path),
            Action::RestoreState(path) => state::restore(environment, &record, &self, &path),
            Action::BootLog => state::boot_log(environment, &record, &self),
            // The device skips end once the whole update is done.
            Action::UpdateStela => {
                let (esp, dry_run) = (record.esp.clone(), self.dry_run);
                let mut text = self.update_stela(environment, record, (false, true))?;
                if let Some(build) = self.macos.take() {
                    let next = Request {
                        action: Action::UpdateOs,
                        build,
                        ..self
                    };
                    text.push_str(&next.dispatch(environment)?);
                }
                if !dry_run {
                    doctor::end_device_skips(&esp, "stela update")?;
                }
                Ok(text)
            }
            Action::UpdateOs => self.update_os(environment, record, true),
            Action::NextBoot(next) => self.next_boot(environment, &record, next),
            Action::Configure => {
                // Without a terminal nothing is asked, so a run that names
                // no setting has nothing to write.
                ensure!(
                    environment.ui.interactive() || settings::named(&self),
                    "name a setting to change, such as --wifi SSID, --ipv4 ADDRESS/PREFIX, \
                     --dns SERVERS or --usb4-allow UUID; on a terminal, {} asks for each",
                    fix(&self.usb, "settings")
                );
                self.read_wifi_passphrase(environment.ui)?;
                machines::configure(environment, &self)
            }
            Action::HostFirmwareInstall(source) => {
                let mount_image = host_firmware_mount(environment, &record)?;
                let package =
                    stela_loader::host_firmware::Package::read_for_mount(&source, &mount_image)?;
                let (images, bytes) = package.inventory();
                let destination = record.esp.join(stela_loader::host_firmware::DIRECTORY);
                let mut report = format!(
                    "Host firmware: {images} verified resources, {bytes} bytes to {}\n",
                    destination.display()
                );
                if self.dry_run {
                    report.push_str("Dry run: nothing was changed.\n");
                } else {
                    package.publish(&destination)?;
                    report.push_str(
                        "Published host firmware manifest. Device initialization is checked at boot.\n",
                    );
                }
                Ok(report)
            }
            Action::Home | Action::Create | Action::Status | Action::Uninstall => {
                unreachable!("these run without the setup record")
            }
        }
    }

    /// Finish the interrupted first setup `record` describes, as it began:
    /// its build, model, installer, disk and network, and the pieces and
    /// bundles it wrote that are intact (`create::plan`).
    fn finish_create(mut self, environment: &Environment<'_>, record: &Record) -> Result<String> {
        let esp = record.esp.clone();
        self.resume_from(esp.clone(), record);
        // The passphrase an interrupted setup wrote already, or else the
        // one a file or the terminal gives.
        let key = esp.join(wifi::KEY_PATH);
        if self.wifi.is_some() && self.wifi_passphrase_file.is_none() && key.is_file() {
            self.wifi_passphrase = Some(wifi::read_passphrase(&key)?);
        }
        if !self.dry_run {
            let what = format!("the rest of macOS {}", record.build);
            match resume_bytes(
                environment,
                record,
                self.ipsw.as_deref(),
                self.download_dir.as_deref(),
            ) {
                Some(bytes) => consent_download(
                    environment,
                    self.yes || self.accepted_download == Some(bytes),
                    false,
                    bytes,
                    &what,
                )?,
                // Apple's server did not say its size: asked without one.
                None => consent(
                    environment,
                    self.yes,
                    &format!(
                        "This downloads {what} from Apple; its size is not known yet. Download it?"
                    ),
                    &format!("this downloads {what} from Apple, of a size not known yet"),
                )?,
            }
        }
        self.action = Action::Create;
        self.create(environment)
    }

    /// Make this request the interrupted setup `record` describes, on its
    /// EFI partition `esp`, so it resumes instead of erasing the stick
    /// again. A local IPSW given stays; otherwise the recorded build is
    /// downloaded.
    fn resume_from(&mut self, esp: PathBuf, record: &Record) {
        self.usb = esp;
        self.build = record.build.clone();
        self.model = Some(record.model.clone());
        // The installer the stopped setup was making: a full one goes on
        // from the pieces already on the stick.
        self.install_image = Some(record.install_kind());
        self.fetch = self.ipsw.is_none();
        if let [install] = &record.installs[..] {
            self.install = Some(install.name.clone());
            let disk = &install.disk;
            if !disk.is_chosen() && !disk.external_disks {
                self.target = Some(disk.apple_region.clone());
                if disk.machine_state_offset.is_none() {
                    self.nor = Some(disk.nor.clone());
                }
            }
        }
        self.wifi = record.wifi.as_ref().map(|network| network.ssid.clone());
        self.wifi_country = record
            .wifi
            .as_ref()
            .and_then(|network| network.country.clone());
        self.ipv4 = record
            .addresses
            .ipv4
            .map(|ipv4| stela_boot_core::options::StaticIpv4 {
                address: ipv4.address,
                prefix: ipv4.prefix,
            });
        self.ipv4_gateway = record.addresses.ipv4.and_then(|ipv4| ipv4.gateway);
        self.dns = (!record.addresses.dns.is_empty()).then(|| record.addresses.dns.clone());
        self.usb4_approve = record.usb4_approved.clone();
        self.profile = record.profile;
        self.verify_on_read = record.verify_on_read;
    }

    /// Create a setup, or add an install (`--target`) to an existing one.
    fn create(mut self, environment: &Environment<'_>) -> Result<String> {
        if let Some(model) = &self.model {
            check_model(environment.models, model)?;
        }
        // A first setup that was interrupted is finished by `repair`, not
        // added to.
        let configured = locate_esp(environment.host, &self.usb).is_ok_and(|esp| {
            esp.join(RECORD).is_file()
                && record::interrupted(environment.host, &esp, &self.usb).is_none()
        });
        if configured && self.target.is_some() {
            ensure!(
                self.wifi.is_none(),
                "--wifi goes with a new setup; {} changes an existing one's",
                fix(&self.usb, "settings --wifi SSID")
            );
            // Every install restores the active build, which is
            // downloaded when no --ipsw names it.
            if let Ok(record) = read_record(environment, &self.usb) {
                self.build = record.build;
            }
            return machines::add(environment, &self);
        }
        setup(environment, self)
    }

    /// Read `--wifi`'s passphrase: the wizard's, `--wifi-passphrase-file`'s,
    /// or one typed at a prompt, which a dry run skips.
    fn read_wifi_passphrase(&mut self, ui: &dyn ui::Ui) -> Result<()> {
        let Some(ssid) = &self.wifi else {
            return Ok(());
        };
        if self.wifi_passphrase.is_some() {
            return Ok(());
        }
        self.wifi_passphrase = match &self.wifi_passphrase_file {
            Some(file) => Some(wifi::read_passphrase(file)?),
            None if self.dry_run => None,
            None => Some(wifi::ask_passphrase(ui, ssid)?),
        };
        Ok(())
    }

    /// `nvram next-boot`.
    fn next_boot(
        &self,
        environment: &Environment<'_>,
        record: &Record,
        next: NextBoot,
    ) -> Result<String> {
        check_installed_efi(environment, record)?;
        let install = record.install(self.install.as_deref())?;
        let disk = &install.disk;
        let changes = installed::next_boot(environment.host, disk, next, self.dry_run)?;
        let mut text = format!(
            "NVRAM of {} (install \"{}\"'s machine state):\n",
            match disk.machine_state_offset {
                Some(offset) => format!("{} from byte {offset:#x}", disk.nor),
                None => disk.nor.clone(),
            },
            install.name
        );
        if changes.is_empty() {
            text.push_str("  nothing to change\n");
        }
        for change in changes {
            writeln!(text, "  {change}")?;
        }
        text.push_str(if self.dry_run {
            "Dry run: nothing was changed.\n"
        } else {
            "Done.\n"
        });
        Ok(text)
    }

    /// The restore image: `--ipsw`, or the build setup downloads.
    fn image(&self) -> Result<PathBuf> {
        match (&self.ipsw, self.fetch) {
            (Some(path), false) => Ok(path.clone()),
            (None, true) if self.dry_run => {
                bail!(
                    "a dry run needs the image first: pass --ipsw, or run once without \
                     --dry-run to download it"
                )
            }
            (None, true) => fetch(self),
            _ => bail!("pass --ipsw FILE"),
        }
    }
}

/// The installed `stela.efi`, which must read host firmware resources.
fn host_firmware_mount(environment: &Environment<'_>, record: &Record) -> Result<Vec<u8>> {
    check_installed_efi(environment, record)?;
    let mount_image = fs::read(record.esp.join(LOADER_PATH))?;
    ensure!(
        stela_loader::host_firmware::supports_mount(&mount_image),
        "the installed stela.efi cannot load host firmware; run {} first",
        fix(&record.esp, "update")
    );
    Ok(mount_image)
}

/// The ESP to change: the EFI partition of `usb`'s disk, which must be
/// mounted, sit on a removable disk and boot no loader but Stela's.
fn locate_esp(host: &dyn Host, usb: &Path) -> Result<PathBuf> {
    ensure!(usb.is_dir(), "{} is not a mounted volume", usb.display());
    let volume = host.volume(usb)?;
    let disks = host.disks()?;
    let disk = volume
        .disk
        .as_ref()
        .and_then(|id| disks.iter().find(|d| &d.identifier == id));
    let partition = disk.and_then(|d| d.partitions.iter().find(|p| p.content == "EFI"));
    let esp = match partition {
        Some(partition) => match &partition.mount {
            Some(mount) => mount.clone(),
            // Hosts leave a USB's EFI partition unmounted.
            None => host.mount(&partition.identifier).with_context(|| {
                format!(
                    "{} is on the same disk as EFI partition {}, which could not be mounted \
                     (setup needs {})",
                    usb.display(),
                    partition.identifier,
                    host::ADMIN
                )
            })?,
        },
        None => bail!(
            "{} is on a disk without an EFI partition. `stela create` erases a stick into one \
             when given its whole disk (such as {}), which deletes everything on it.",
            usb.display(),
            host::DISK_EXAMPLE
        ),
    };
    // A USB setup prepared owns its loader, damaged or old as it may be.
    if !esp.join(RECORD).is_file() {
        efi::check_loader(&esp)?;
    }
    if let Some(disk) = host
        .volume(&esp)?
        .disk
        .and_then(|id| disks.into_iter().find(|d| d.identifier == id))
    {
        ensure!(
            disk.erasable(),
            "{} is on {} ({}), an internal disk{}. Stela changes only a removable stick: \
             plug one in and name it.",
            esp.display(),
            disk.identifier,
            disk.name,
            if disk.system {
                " that runs this computer"
            } else {
                ""
            }
        );
    }
    Ok(esp)
}

#[cfg(test)]
mod tests;
