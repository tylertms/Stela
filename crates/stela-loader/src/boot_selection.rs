//! LLB's boot choice from NVRAM: which installed system, and which of its
//! operating systems (macOS, its paired recoveryOS or System Recovery),
//! from the variables of the machine's persistent NOR bank.
//!
//! Known, from the 26A434 J473 LLB (`Firmware/all_flash/LLB.j473.RELEASE`,
//! raw image offsets):
//!
//! - 0xd46b8 reads `alt-boot-volume` and, when present, removes it and saves
//!   NVRAM before booting it, so it chooses one boot only; otherwise it reads
//!   `boot-volume`.
//! - 0xd4744 parses either as `TYPE:PARTITION:GROUP`, three UUID texts. The
//!   first must name one of three partition types, written as `uuid_unparse`
//!   of the GPT type GUID's on-disk bytes: APFS
//!   (`EF57347C-0000-AA11-AA11-00306543ECAC`), the iBoot System Container
//!   (`61696469-0067-AA11-AA11-00306543ECAC`) or the recoveryOS container
//!   (`72766352-0079-AA11-AA11-00306543ECAC`). The third is the APFS volume
//!   group.
//! - 0x2a58 removes `one-time-boot-command` (with the `iboot-failure-reason`
//!   variables) and saves NVRAM; 0x2a74 and 0x2ab8 read `auto-boot` and
//!   `auto-boot-once`. Its boot-command table is `fsboot`, `recover`
//!   ("Paired Recovery"), `recover-once`, `recover-system` ("System
//!   Recovery"), `recover-fallback`, `device-recovery`, `upgrade`,
//!   `post-upgrade`, `fsboot-safe` and `field-diags-paired`.
//! - The 26A434 kernel's `IOSetRecoveryBoot` and iBoot (0x4464) send a
//!   machine to recovery by setting `recovery-boot-mode` with `boot-command`
//!   `recover-system` or `recover`.
//! - The `fsboot` class is `fsboot`, `fsboot-safe` and `post-upgrade` as a
//!   one-time command (LLB 0xbeb70, iBoot 0xd242c): `post-upgrade` boots
//!   macOS.
//! - `boot-command=upgrade` (0xbe93c) boots the update macOS staged: when
//!   `iboot1-precommitted=true` LLB first removes it, commits the staged
//!   iBoot to the NOR and resets (0x43d0). The upgrade itself (0x448c)
//!   removes `boot-command`, or, when `upgrade-retry` is set, removes that
//!   and sets `boot-command=recover`, so a failed retry boots the paired
//!   recoveryOS; it saves NVRAM, mounts the boot device's Update-role volume
//!   (APFS role 0xc0, mount flag 1, 0x945a0) at `/boot` and runs its
//!   `/boot/iBEC`. On a failure it sets `ota-result` to `failed to execute
//!   upgrade command from iBoot` and `ota-failure-reason` to the step
//!   (`scan SPs`, `mount upgrade partition`, `load upgrade iBEC`, `load
//!   upgrade local policy`), saves and resets (0x4770).
//! - `device-recovery` puts LLB in its recovery mode (0x2994, mode 5) and
//!   `field-diags-paired` boots the paired diagnostics.
//!
//! Inferred: the second UUID is the partition's unique GUID in the same
//! on-disk byte order as the type; `one-time-boot-command` takes precedence
//! over `boot-command`, except that `upgrade` is checked first (0x2b14, before
//! the boot-command dispatch); a set `recovery-boot-mode` without a recovery
//! command boots the paired recoveryOS (the variable stays for recoveryOS
//! to read and clear); `fsboot-safe` is a safe-mode (`-x`) macOS boot;
//! `auto-boot-once=true` overrides `auto-boot=false` once and is consumed.
//! Each variable is read from the `system` partition first, then `common`,
//! since which one LLB reads is Unknown. Variables LLB sets are written to
//! `system`.
//!
//! Stela does not run the NOR's LLB or iBoot, so a precommitted iBoot is
//! consumed with a note rather than activated; Stela takes iBEC's place in
//! an upgrade boot (`stela_loader::upgrade`).
//!
//! iBoot's recovery mode, which waits for a USB restore host, is entered by
//! `device-recovery` and by `auto-boot=false` without `auto-boot-once`
//! ([`Target::RecoveryMode`]). Known, from the 26A434 `restored_external`
//! strings ("could not set auto-boot=false", "Setting auto-boot to true to
//! return to prior OS"): a restore clears `auto-boot` while it writes the
//! disk and sets it again once it completes, so an interrupted restore
//! leaves the machine in recovery mode (the order is Inferred). Stela's
//! restore host is its Install macOS startup choice, so `stela.efi`
//! holds such a machine at its startup choices instead of booting the
//! half-restored disk.
//!
//! A recoveryOS boot reads the copy on the boot disk, as LLB does: the
//! paired recoveryOS from the boot container's Recovery volume, then System
//! Recovery from the RecoveryOSContainer ([`route`],
//! `stela_loader::recovery`). A disk that holds neither is refused: Stela has
//! no recoveryOS of its own to fall back to.
//!
//! Refused explicitly: `field-diags-paired` (a diagnostics image Stela does
//! not read).
use crate::Result;
use alloc::{format, string::String, vec::Vec};
use stela_contracts::{refuse, require};
use {stela_soc_data::nor::nvram::Partition, stela_soc_data::nor::nvram::Variables};

pub const BOOT_VOLUME: &[u8] = b"boot-volume";
pub const ALT_BOOT_VOLUME: &[u8] = b"alt-boot-volume";
pub const BOOT_COMMAND: &[u8] = b"boot-command";
pub const ONE_TIME_BOOT_COMMAND: &[u8] = b"one-time-boot-command";
pub const RECOVERY_BOOT_MODE: &[u8] = b"recovery-boot-mode";
pub const AUTO_BOOT: &[u8] = b"auto-boot";
pub const AUTO_BOOT_ONCE: &[u8] = b"auto-boot-once";
pub const UPGRADE_RETRY: &[u8] = b"upgrade-retry";
pub const IBOOT_PRECOMMITTED: &[u8] = b"iboot1-precommitted";
pub const OTA_RESULT: &[u8] = b"ota-result";
pub const OTA_FAILURE_REASON: &[u8] = b"ota-failure-reason";
/// LLB's `ota-result` when its upgrade boot fails.
pub const UPGRADE_FAILURE: &[u8] = b"failed to execute upgrade command from iBoot";
/// The variables LLB removes with `one-time-boot-command`.
pub const FAILURE_REASONS: [&[u8]; 2] = [b"iboot-failure-reason", b"iboot-failure-reason-str"];
/// Stela's own record of the automatic repair of a missing Metal plugin
/// (`stela_boot_core::bundle_repair`), in `system`: [`BUNDLE_REPAIR_PENDING`]
/// asks the next macOS boot to be the repair's single-user maintenance boot,
/// which sets it to [`BUNDLE_REPAIR_ATTEMPTED`] before it boots, so a failed
/// or interrupted repair is never retried by itself. The engine removes it
/// once a boot finds the plugin installed. [`BUNDLE_REPAIR_INSTALLED`] is
/// what a finished erase install leaves: the next macOS boot is the same
/// maintenance boot and removes the record, so a normal boot that still
/// finds the plugin missing asks for the repair as usual.
/// [`BUNDLE_REPAIR_RETRY`] is what a maintenance boot whose shell never took
/// input leaves: like a request, but taken as [`BUNDLE_REPAIR_RETRIED`],
/// after which nothing asks again by itself.
pub const BUNDLE_REPAIR: &[u8] = b"stela-bundle-repair";
pub const BUNDLE_REPAIR_PENDING: &[u8] = b"pending";
pub const BUNDLE_REPAIR_ATTEMPTED: &[u8] = b"attempted";
pub const BUNDLE_REPAIR_INSTALLED: &[u8] = b"installed";
pub const BUNDLE_REPAIR_RETRY: &[u8] = b"retry";
pub const BUNDLE_REPAIR_RETRIED: &[u8] = b"retried";
/// What the repair's maintenance boot reported at its stop, as text: the
/// step that failed and its status, or that every required step passed.
/// A boot that still finds the plugin missing names it in its stop, and
/// `stela status` shows it. Taking a new request removes it, so it always
/// belongs to the latest attempt.
pub const BUNDLE_REPAIR_RESULT: &[u8] = b"stela-bundle-repair-result";
const TRUE: &[u8] = b"true";
const FALSE: &[u8] = b"false";
/// `8-4-4-4-12` hexadecimal digits.
const UUID_TEXT_BYTES: usize = 36;
const UUID_GROUPS: [usize; 5] = [8, 4, 4, 4, 12];

/// A UUID in `uuid_t` byte order: the bytes its text names, in order.
pub type Uuid = [u8; 16];

/// Parse `uuid_parse` text: 36 characters, either case.
pub fn parse_uuid(text: &[u8]) -> Option<Uuid> {
    if text.len() != UUID_TEXT_BYTES {
        return None;
    }
    let mut uuid = [0u8; 16];
    let mut at = 0;
    let mut out = 0;
    for (index, width) in UUID_GROUPS.into_iter().enumerate() {
        if index > 0 {
            if text[at] != b'-' {
                return None;
            }
            at += 1;
        }
        for pair in text[at..at + width].chunks(2) {
            let digits = core::str::from_utf8(pair).ok()?;
            uuid[out] = u8::from_str_radix(digits, 16).ok()?;
            out += 1;
        }
        at += width;
    }
    Some(uuid)
}

/// `uuid_unparse_upper` text.
pub fn uuid_text(uuid: &Uuid) -> String {
    let mut text = String::with_capacity(UUID_TEXT_BYTES);
    let mut at = 0;
    for (index, width) in UUID_GROUPS.into_iter().enumerate() {
        if index > 0 {
            text.push('-');
        }
        for byte in &uuid[at..at + width / 2] {
            text.push_str(&format!("{byte:02X}"));
        }
        at += width / 2;
    }
    text
}

/// The partition types a boot volume may name.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VolumeType {
    /// An APFS container holding the installed system.
    Apfs,
    /// The iBoot System Container.
    SystemContainer,
    /// The recoveryOS container.
    Recovery,
}

impl VolumeType {
    const ALL: [Self; 3] = [Self::Apfs, Self::SystemContainer, Self::Recovery];

    /// The type as `boot-volume` writes it.
    pub fn text(self) -> &'static str {
        match self {
            Self::Apfs => "EF57347C-0000-AA11-AA11-00306543ECAC",
            Self::SystemContainer => "61696469-0067-AA11-AA11-00306543ECAC",
            Self::Recovery => "72766352-0079-AA11-AA11-00306543ECAC",
        }
    }

    /// The GPT type GUID's on-disk bytes.
    pub fn guid_bytes(self) -> Uuid {
        parse_uuid(self.text().as_bytes()).expect("constant UUID")
    }
}

/// A `boot-volume` or `alt-boot-volume` value.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct BootVolume {
    pub kind: VolumeType,
    /// The partition's unique GUID, on-disk bytes.
    pub partition: Uuid,
    /// The APFS volume group.
    pub volume_group: Uuid,
}

impl BootVolume {
    /// Parse `TYPE:PARTITION:GROUP`, optionally NUL-terminated.
    pub fn parse(value: &[u8]) -> Result<Self> {
        let value = value.strip_suffix(b"\0").unwrap_or(value);
        let fields: Vec<&[u8]> = value.split(|&b| b == b':').collect();
        let [kind, partition, group] = fields[..] else {
            return Err(refuse!(
                "a boot volume is TYPE:PARTITION:GROUP, not {value}",
                value = format!("{:?}", String::from_utf8_lossy(value))
            )
            .into());
        };
        let uuid = |field: &[u8], what: &str| {
            parse_uuid(field).ok_or_else(|| {
                refuse!(
                    "boot volume {what} {field} is not a UUID",
                    what,
                    field = format!("{:?}", String::from_utf8_lossy(field))
                )
            })
        };
        let kind_bytes = uuid(kind, "type")?;
        let Some(kind) = VolumeType::ALL
            .into_iter()
            .find(|t| t.guid_bytes() == kind_bytes)
        else {
            return Err(refuse!(
                "boot volume type {kind} is none of APFS, the iBoot System Container or recoveryOS",
                kind = uuid_text(&kind_bytes)
            )
            .into());
        };
        Ok(Self {
            kind,
            partition: uuid(partition, "partition")?,
            volume_group: uuid(group, "volume group")?,
        })
    }

    /// The value as `boot-volume` stores it.
    pub fn encode(&self) -> Vec<u8> {
        format!(
            "{}:{}:{}",
            self.kind.text(),
            uuid_text(&self.partition),
            uuid_text(&self.volume_group)
        )
        .into_bytes()
    }
}

/// An LLB boot command.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Command {
    Fsboot,
    FsbootSafe,
    Recover,
    RecoverOnce,
    RecoverSystem,
    RecoverFallback,
    DeviceRecovery,
    Upgrade,
    PostUpgrade,
    FieldDiagsPaired,
}

impl Command {
    const ALL: [Self; 10] = [
        Self::Fsboot,
        Self::FsbootSafe,
        Self::Recover,
        Self::RecoverOnce,
        Self::RecoverSystem,
        Self::RecoverFallback,
        Self::DeviceRecovery,
        Self::Upgrade,
        Self::PostUpgrade,
        Self::FieldDiagsPaired,
    ];

    pub fn name(self) -> &'static str {
        match self {
            Self::Fsboot => "fsboot",
            Self::FsbootSafe => "fsboot-safe",
            Self::Recover => "recover",
            Self::RecoverOnce => "recover-once",
            Self::RecoverSystem => "recover-system",
            Self::RecoverFallback => "recover-fallback",
            Self::DeviceRecovery => "device-recovery",
            Self::Upgrade => "upgrade",
            Self::PostUpgrade => "post-upgrade",
            Self::FieldDiagsPaired => "field-diags-paired",
        }
    }

    pub fn parse(value: &[u8]) -> Result<Self> {
        let value = value.strip_suffix(b"\0").unwrap_or(value);
        Self::ALL
            .into_iter()
            .find(|c| c.name().as_bytes() == value)
            .ok_or_else(|| {
                refuse!(
                    "{command} is not an LLB boot command",
                    command = format!("{:?}", String::from_utf8_lossy(value))
                )
                .into()
            })
    }
}

/// What the machine boots.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Target {
    /// The installed macOS; `safe` adds the safe-mode boot argument.
    MacOs { safe: bool },
    /// The recoveryOS paired with the installed system (its container's
    /// Recovery volume).
    PairedRecovery,
    /// System Recovery: the recoveryOS container's.
    SystemRecovery,
    /// The update ramdisk macOS staged on its container's Update volume.
    Upgrade,
    /// iBoot's recovery mode, waiting for a restore host: `auto-boot=false`
    /// or `device-recovery`. No operating system boots.
    RecoveryMode,
}

/// The boot LLB would choose, what chose it, and the one-shot variables the
/// boot consumes.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct Selection {
    pub target: Target,
    /// The original NVRAM command, kept through one-shot consumption;
    /// post-upgrade has a distinct firmware handoff despite booting macOS.
    pub command: Option<Command>,
    /// The boot volume, when NVRAM names one; otherwise the disk's only
    /// installed system is used.
    pub volume: Option<BootVolume>,
    /// Which variables decided, for the boot log.
    pub reasons: Vec<String>,
    /// Variables removed and saved before the boot.
    pub consumed: Vec<&'static [u8]>,
    /// Variables set, in `system`, after those are removed.
    pub assigned: Vec<(&'static [u8], &'static [u8])>,
    /// Effects of the boot Stela does not reproduce, for the boot log.
    pub notes: Vec<String>,
    /// The boot disk holds the paired recoveryOS (`--paired-recovery`);
    /// otherwise a paired recovery boot falls back to System Recovery.
    pub paired_on_disk: bool,
    /// The boot disk holds System Recovery in its RecoveryOSContainer
    /// (`--system-recovery`); otherwise a recoveryOS boot is refused.
    pub system_on_disk: bool,
    /// A macOS boot that is the automatic Metal plugin repair's maintenance
    /// boot (`--bundle-repair`), which [`BUNDLE_REPAIR`] asked for.
    pub bundle_repair: bool,
}

impl Selection {
    /// Whether booting this selection changes NVRAM.
    pub fn consumes(&self) -> bool {
        !self.consumed.is_empty() || !self.assigned.is_empty()
    }

    /// The boot disk holds no installed system: boot System Recovery, as
    /// the 26A434 kernel's `IOSetRecoveryBoot` asks with
    /// `boot-picker-bringup-reason=missing-boot-media` and
    /// `boot-command=recover-system` (Inferred: LLB does the same when it
    /// finds no boot volume, its `recover-fallback`).
    pub fn missing_boot_media(&mut self) {
        if matches!(self.target, Target::MacOs { .. }) {
            self.target = Target::SystemRecovery;
            // The repair waits for a macOS boot, as for any recoveryOS one.
            self.bundle_repair = false;
            self.assigned.retain(|(name, _)| *name != BUNDLE_REPAIR);
            self.consumed
                .retain(|name| ![BUNDLE_REPAIR, BUNDLE_REPAIR_RESULT].contains(name));
            self.reasons
                .push("missing-boot-media: the boot disk holds no installed system".into());
        }
    }
}

/// `name`'s value, from `system` and then `common`.
pub fn lookup<'a>(variables: &'a Variables, name: &[u8]) -> Option<&'a [u8]> {
    variables
        .get(Partition::System, name)
        .or_else(|| variables.get(Partition::Common, name))
}

fn strip(value: &[u8]) -> &[u8] {
    value.strip_suffix(b"\0").unwrap_or(value)
}

fn flag(name: &[u8], value: &[u8]) -> Result<bool> {
    match strip(value) {
        TRUE => Ok(true),
        FALSE => Ok(false),
        other => Err(refuse!(
            "{name} is {value}, neither true nor false",
            name = String::from_utf8_lossy(name).into_owned(),
            value = format!("{:?}", String::from_utf8_lossy(other))
        )
        .into()),
    }
}

fn shown(name: &[u8], value: &[u8]) -> String {
    format!(
        "{}={}",
        String::from_utf8_lossy(name),
        String::from_utf8_lossy(strip(value))
    )
}

/// Choose the boot as LLB does; refused for the boots Stela does not model.
pub fn select(variables: &Variables) -> Result<Selection> {
    let mut selection = Selection {
        target: Target::MacOs { safe: false },
        command: None,
        volume: None,
        reasons: Vec::new(),
        consumed: Vec::new(),
        assigned: Vec::new(),
        notes: Vec::new(),
        paired_on_disk: false,
        system_on_disk: false,
        bundle_repair: false,
    };
    if let Some(value) = lookup(variables, AUTO_BOOT)
        && !flag(AUTO_BOOT, value)?
    {
        match lookup(variables, AUTO_BOOT_ONCE) {
            Some(once) if flag(AUTO_BOOT_ONCE, once)? => {
                selection.reasons.push(shown(AUTO_BOOT_ONCE, once));
                selection.consumed.push(AUTO_BOOT_ONCE);
            }
            once => {
                if let Some(once) = once {
                    flag(AUTO_BOOT_ONCE, once)?;
                }
                selection.reasons.push(shown(AUTO_BOOT, value));
                selection.target = Target::RecoveryMode;
                return Ok(selection);
            }
        }
    }
    let volume = match lookup(variables, ALT_BOOT_VOLUME) {
        Some(value) => {
            selection.consumed.push(ALT_BOOT_VOLUME);
            Some((ALT_BOOT_VOLUME, value))
        }
        None => lookup(variables, BOOT_VOLUME).map(|value| (BOOT_VOLUME, value)),
    };
    if let Some((name, value)) = volume {
        selection.volume = Some(BootVolume::parse(value)?);
        selection.reasons.push(shown(name, value));
    }
    let upgrade = lookup(variables, BOOT_COMMAND)
        .is_some_and(|value| Command::parse(value).ok() == Some(Command::Upgrade));
    if upgrade {
        selection.reasons.push(shown(BOOT_COMMAND, b"upgrade"));
        if let Some(value) = lookup(variables, IBOOT_PRECOMMITTED)
            && flag(IBOOT_PRECOMMITTED, value)?
        {
            selection.consumed.push(IBOOT_PRECOMMITTED);
            selection.notes.push(
                "iboot1-precommitted: Stela does not run the NOR's iBoot, so the staged iBoot \
                 is not committed"
                    .into(),
            );
        }
        selection.consumed.push(BOOT_COMMAND);
        if lookup(variables, UPGRADE_RETRY).is_some() {
            selection.reasons.push(shown(UPGRADE_RETRY, b"set"));
            selection.consumed.push(UPGRADE_RETRY);
            selection
                .assigned
                .push((BOOT_COMMAND, Command::Recover.name().as_bytes()));
        }
        selection.target = Target::Upgrade;
        selection.command = Some(Command::Upgrade);
        check_volume(&selection)?;
        return Ok(selection);
    }
    let command = match lookup(variables, ONE_TIME_BOOT_COMMAND) {
        Some(value) => {
            selection.consumed.push(ONE_TIME_BOOT_COMMAND);
            selection.consumed.extend(FAILURE_REASONS);
            Some((ONE_TIME_BOOT_COMMAND, value))
        }
        None => lookup(variables, BOOT_COMMAND).map(|value| (BOOT_COMMAND, value)),
    };
    let command = match command {
        Some((name, value)) => {
            selection.reasons.push(shown(name, value));
            Some(Command::parse(value)?)
        }
        None => None,
    };
    let recovery_mode = lookup(variables, RECOVERY_BOOT_MODE);
    selection.command = command;
    selection.target =
        match command {
            Some(Command::Recover | Command::RecoverOnce) => Target::PairedRecovery,
            Some(Command::RecoverSystem | Command::RecoverFallback) => Target::SystemRecovery,
            Some(Command::DeviceRecovery) => Target::RecoveryMode,
            Some(Command::FieldDiagsPaired) => return Err(refuse!(
                "boot-command field-diags-paired boots the paired diagnostics, which Stela does \
             not read"
            )
            .into()),
            // A one-time `upgrade`; LLB checks only `boot-command` for it.
            Some(Command::Upgrade) => {
                return Err(refuse!(
                    "one-time-boot-command upgrade: LLB enters an upgrade only from boot-command"
                )
                .into());
            }
            _ if recovery_mode.is_some() => {
                let value = recovery_mode.expect("checked");
                selection.reasons.push(shown(RECOVERY_BOOT_MODE, value));
                Target::PairedRecovery
            }
            Some(Command::FsbootSafe) => Target::MacOs { safe: true },
            Some(Command::Fsboot | Command::PostUpgrade) | None => Target::MacOs { safe: false },
        };
    check_volume(&selection)?;
    // Only a macOS boot repairs: a recoveryOS boot leaves the request for
    // the next one.
    if matches!(selection.target, Target::MacOs { .. })
        && let Some(value) = lookup(variables, BUNDLE_REPAIR)
    {
        match strip(value) {
            BUNDLE_REPAIR_PENDING => selection
                .assigned
                .push((BUNDLE_REPAIR, BUNDLE_REPAIR_ATTEMPTED)),
            BUNDLE_REPAIR_RETRY => selection
                .assigned
                .push((BUNDLE_REPAIR, BUNDLE_REPAIR_RETRIED)),
            BUNDLE_REPAIR_INSTALLED => selection.consumed.push(BUNDLE_REPAIR),
            _ => return Ok(selection),
        }
        // The new attempt reports for itself.
        if lookup(variables, BUNDLE_REPAIR_RESULT).is_some() {
            selection.consumed.push(BUNDLE_REPAIR_RESULT);
        }
        selection.reasons.push(shown(BUNDLE_REPAIR, value));
        selection.bundle_repair = true;
    }
    Ok(selection)
}

fn check_volume(selection: &Selection) -> Result<()> {
    if let Some(volume) = &selection.volume {
        require!(
            volume.kind == VolumeType::Apfs
                || !matches!(selection.target, Target::MacOs { .. } | Target::Upgrade),
            "boot volume {partition} is not an APFS container, so it holds no macOS",
            partition = uuid_text(&volume.partition)
        );
    }
    Ok(())
}

/// Record a failed upgrade boot as LLB does before it resets: `ota-result`
/// and `ota-failure-reason`, the step that failed.
pub fn record_upgrade_failure(variables: &mut Variables, step: &str) -> Result<()> {
    for (name, value) in [
        (OTA_RESULT, UPGRADE_FAILURE),
        (OTA_FAILURE_REASON, step.as_bytes()),
    ] {
        variables.set(Partition::System, name, value)?;
    }
    Ok(())
}

/// Remove the variables `selection` consumes from whichever partition holds
/// them and set those it assigns; returns whether anything changed, when
/// the bank must be saved.
pub fn consume(variables: &mut Variables, selection: &Selection) -> Result<bool> {
    let mut changed = false;
    for name in &selection.consumed {
        for partition in [Partition::System, Partition::Common] {
            changed |= variables.remove(partition, name).is_some();
        }
    }
    for (name, value) in &selection.assigned {
        variables.set(Partition::System, name, value)?;
        changed = true;
    }
    Ok(changed)
}

/// Name `volume` as the next boot's volume: `alt-boot-volume` for one boot,
/// `boot-volume` otherwise. Written to `system`, where the macOS startup
/// disk choice is Inferred to live.
pub fn choose(variables: &mut Variables, volume: &BootVolume, once: bool) -> Result<()> {
    let name = if once { ALT_BOOT_VOLUME } else { BOOT_VOLUME };
    Ok(variables.set(Partition::System, name, &volume.encode())?)
}

/// `stela.cfg`'s installed-system option: the disk the bundle is built from.
pub const PREBOOT_OPTION: &str = "--preboot";
pub const BUNDLE_OPTION: &str = "--bundle";
/// Options `stela.efi` passes its engine: the boot volume to build from and
/// a safe-mode boot.
pub const BOOT_VOLUME_OPTION: &str = "--boot-volume";
pub const SAFE_BOOT_OPTION: &str = "--safe-boot";
/// The previous-stage entry code kept after consuming NVRAM commands.
pub const BOOT_COMMAND_OPTION: &str = "--boot-command";
/// Build the bundle from the Preboot disk's Update volume
/// (`stela_loader::upgrade`) instead of its installed system.
pub const UPGRADE_OPTION: &str = "--upgrade";
/// Build the bundle from the Preboot disk's Recovery volume, the paired
/// recoveryOS (`stela_loader::recovery`).
pub const PAIRED_RECOVERY_OPTION: &str = "--paired-recovery";
/// Build the bundle from the Preboot disk's RecoveryOSContainer, System
/// Recovery (`stela_loader::recovery`).
pub const SYSTEM_RECOVERY_OPTION: &str = "--system-recovery";
/// How [`route`]'s refusal of [`Target::RecoveryMode`] starts, so the
/// mount can tell it from other refusals and hold the machine at its
/// startup choices.
pub const RECOVERY_MODE_REFUSAL: &str = "NVRAM holds the machine in iBoot's recovery mode";
/// How [`route`]'s refusal of a recoveryOS boot of a disk without one
/// starts.
pub const NO_RECOVERY_REFUSAL: &str = "NVRAM boots recoveryOS";
/// A macOS boot that is the automatic Metal plugin repair's single-user
/// maintenance boot ([`BUNDLE_REPAIR`]).
pub const BUNDLE_REPAIR_OPTION: &str = "--bundle-repair";
/// What the engine is told only by NVRAM.
const NVRAM_OPTIONS: [&str; 7] = [
    BOOT_VOLUME_OPTION,
    SAFE_BOOT_OPTION,
    UPGRADE_OPTION,
    PAIRED_RECOVERY_OPTION,
    SYSTEM_RECOVERY_OPTION,
    BOOT_COMMAND_OPTION,
    BUNDLE_REPAIR_OPTION,
];

/// The engine options for `selection` (none when NVRAM was not read): a
/// macOS boot keeps `--preboot` and adds the boot volume and safe mode; an
/// update adds `--upgrade`, a paired recoveryOS the disk holds
/// `--paired-recovery`, and System Recovery the disk holds (also in place
/// of a paired recoveryOS it lacks) `--system-recovery`; a recoveryOS boot
/// of a disk that holds neither is refused. Recovery mode boots nothing and
/// is refused ([`Target::RecoveryMode`]). Returns the options and what the
/// boot log should say.
pub fn route(options: &str, selection: Option<&Selection>) -> Result<(String, String)> {
    let words: Vec<&str> = options.split_ascii_whitespace().collect();
    for option in NVRAM_OPTIONS {
        require!(
            !words.contains(&option),
            "{option} is chosen from NVRAM, not stela.cfg",
            option
        );
    }
    let Some(selection) = selection else {
        return Ok((words.join(" "), "NVRAM not read".into()));
    };
    let reasons = if selection.reasons.is_empty() {
        "no boot variables".into()
    } else {
        selection.reasons.join(", ")
    };
    let mut extra: Vec<String> = Vec::new();
    let on_disk =
        |extra: &mut Vec<String>, what: &str, command: crate::handoff::BootCommand| -> Result<()> {
            require!(
                words.contains(&PREBOOT_OPTION),
                "NVRAM boots {what}, which needs {PREBOOT_OPTION}",
                what,
                PREBOOT_OPTION
            );
            if let Some(volume) = &selection.volume {
                extra.push(BOOT_VOLUME_OPTION.into());
                extra.push(String::from_utf8(volume.encode()).expect("ASCII"));
            }
            extra.push(BOOT_COMMAND_OPTION.into());
            extra.push(format!("{}", command as u8));
            Ok(())
        };
    let log = match selection.target {
        Target::MacOs { safe } => {
            let command = if safe {
                crate::handoff::BootCommand::FsbootSafe
            } else if selection.command == Some(Command::PostUpgrade) {
                crate::handoff::BootCommand::PostUpgrade
            } else {
                crate::handoff::BootCommand::Fsboot
            };
            on_disk(&mut extra, "macOS", command)?;
            if safe {
                extra.push(SAFE_BOOT_OPTION.into());
            }
            if selection.bundle_repair {
                extra.push(BUNDLE_REPAIR_OPTION.into());
            }
            format!(
                "macOS{}{} ({reasons})",
                if safe { " in safe mode" } else { "" },
                if selection.bundle_repair {
                    ", single-user to repair the Metal plugin"
                } else {
                    ""
                }
            )
        }
        Target::Upgrade => {
            on_disk(
                &mut extra,
                "a macOS update",
                crate::handoff::BootCommand::Upgrade,
            )?;
            extra.push(UPGRADE_OPTION.into());
            format!("macOS update from the Update volume ({reasons})")
        }
        Target::RecoveryMode => {
            return Err(refuse!(
                "{RECOVERY_MODE_REFUSAL} ({reasons}): it waits for a restore host, as an \
             interrupted restore leaves it; choose Install macOS to restore again",
                RECOVERY_MODE_REFUSAL,
                reasons
            )
            .into());
        }
        Target::PairedRecovery if selection.paired_on_disk => {
            on_disk(
                &mut extra,
                "the paired recoveryOS",
                crate::handoff::BootCommand::Recover,
            )?;
            extra.push(PAIRED_RECOVERY_OPTION.into());
            format!("paired recoveryOS from the Recovery volume ({reasons})")
        }
        target @ (Target::PairedRecovery | Target::SystemRecovery) if selection.system_on_disk => {
            on_disk(
                &mut extra,
                "System Recovery",
                crate::handoff::BootCommand::RecoverSystem,
            )?;
            extra.push(SYSTEM_RECOVERY_OPTION.into());
            match target {
                Target::PairedRecovery => format!(
                    "System Recovery from the RecoveryOSContainer ({reasons}): the boot disk \
                     holds no paired recoveryOS"
                ),
                _ => format!("System Recovery from the RecoveryOSContainer ({reasons})"),
            }
        }
        Target::PairedRecovery | Target::SystemRecovery => {
            return Err(refuse!(
                "{NO_RECOVERY_REFUSAL} ({reasons}): the boot disk holds neither a paired \
             recoveryOS nor System Recovery; choose Install macOS to restore it",
                NO_RECOVERY_REFUSAL,
                reasons
            )
            .into());
        }
    };
    let mut text: Vec<String> = words.into_iter().map(String::from).collect();
    text.extend(extra);
    Ok((text.join(" "), log))
}

#[cfg(test)]
mod tests;
