//! Streaming application and final normalization of one system option line.
use super::*;
use core::str::SplitAsciiWhitespace;

/// Partial state is private and discarded on any error. An accepted core count
/// itself records occurrence; there is no independent seen/valid state to drift.
struct ParseState<'a> {
    options: System,
    words: SplitAsciiWhitespace<'a>,
}

pub(super) fn system(text: &str) -> Result<System, Error> {
    let mut words = text.split_ascii_whitespace();
    if words.next() != Some(SYSTEM) {
        return Err(Error::SystemPrefix);
    }
    let mut state = ParseState {
        options: System::default(),
        words,
    };
    while let Some(name) = state.words.next() {
        state.apply(name)?;
    }
    state.finish()
}

impl ParseState<'_> {
    /// Consume exactly the value words required by this spelling. Duplicate
    /// checks stay on their original side of value parsing.
    fn apply(&mut self, name: &str) -> Result<(), Error> {
        let options = &mut self.options;
        let mut value = || self.words.next();
        match name {
            "--native-artifacts" => options.native_artifacts = true,
            "--native-rounds" => options.native_rounds = true,
            "--parallel" => options.parallel = true,
            "--serialized" => options.parallel = false,
            "--profile" => options.profile = true,
            MAINTENANCE_WRITE => options.maintenance_write = true,
            SINGLE_USER => options.single_user = true,
            SCANOUT_MIRROR => options.scanout_mirror = true,
            "--monitor-trace" => options.monitor_trace = true,
            "--display-trace" => options.display_trace = true,
            "--native-shared-check" => flag(
                &mut options.native_shared_check,
                Error::DuplicateNativeShared,
            )?,
            "--native-bootstrap-check" => flag(
                &mut options.native_bootstrap_check,
                Error::DuplicateNativeBootstrap,
            )?,
            NATIVE_CORES => {
                if options.native_cores.is_some() {
                    return Err(Error::DuplicateNativeCores);
                }
                let count = value()
                    .ok_or(Error::NativeCoreCount)?
                    .parse::<usize>()
                    .map_err(|_| Error::NativeCoreCount)?;
                if !(1..=crate::topology::MAX_CORES).contains(&count) {
                    return Err(Error::NativeCoreCapacity);
                }
                options.native_cores = Some(count);
            }
            GUEST_PES => {
                if options.guest_pes.is_some() {
                    return Err(Error::DuplicateGuestPes);
                }
                let count = value()
                    .ok_or(Error::GuestPeCount)?
                    .parse::<usize>()
                    .map_err(|_| Error::GuestPeCount)?;
                if count == 0 {
                    return Err(Error::GuestPeCount);
                }
                options.guest_pes = Some(count);
            }
            "--restore-study" => options.restore = true,
            "--restore-source" => {
                let target = value()
                    .ok_or(Error::MissingRestoreSource)?
                    .parse()
                    .map_err(|source| Error::Target {
                        argument: TargetArgument::RestoreSource,
                        source,
                    })?;
                if options.restore_source.replace(target).is_some() {
                    return Err(Error::DuplicateRestoreSource);
                }
            }
            "--restore-manifest" => {
                let path = value().ok_or(Error::MissingRestoreManifest)?;
                if options.restore_manifest {
                    return Err(Error::DuplicateRestoreManifest);
                }
                options.restore_manifest = true;
                options.restore_manifest_path = Some(String::from(path));
            }
            INSTALL => {
                let path = value().ok_or(Error::MissingInstallManifest)?;
                if options.install.replace(String::from(path)).is_some() {
                    return Err(Error::DuplicateInstallMedium);
                }
            }
            INSTALL_SOURCE => {
                let guid = value()
                    .ok_or(Error::MissingInstallSource)?
                    .parse()
                    .map_err(|source| Error::Guid {
                        argument: GuidArgument::InstallSource,
                        source,
                    })?;
                if options.install_source.replace(guid).is_some() {
                    return Err(Error::DuplicateInstallSource);
                }
            }
            "--restore-identity" => {
                let index = number(value().ok_or(Error::MissingRestoreIdentity)?)?;
                let index = usize::try_from(index).map_err(|_| Error::RestoreIdentityCapacity)?;
                if options.restore_identity.replace(index).is_some() {
                    return Err(Error::DuplicateRestoreIdentity);
                }
            }
            "--bundle" => {
                let directory = value().ok_or(Error::MissingBundleDirectory)?;
                options.bundle = true;
                options.bundle_directory = Some(String::from(directory));
            }
            "--survey" => options.survey = true,
            VERIFY_ON_READ => options.verify_on_read = true,
            UPDATE_RESTORE => options.update_restore = true,
            "--development-restore" => options.development_restore = true,
            "--idle-skip" => options.idle_skip = true,
            "--input" => options.input = true,
            "--no-guest-bluetooth" => options.no_guest_bluetooth = true,
            "--usb-storage" => {
                let serial = value().ok_or(Error::MissingUsbStorage)?;
                if options.usb_storage.iter().any(|other| other == serial) {
                    return Err(Error::DuplicateUsbStorage);
                }
                options.usb_storage.push(String::from(serial));
            }
            "--guest-network" => options.guest_network = true,
            WIFI => {
                let ssid = ssid(value().ok_or(Error::MissingWifiNetwork)?)?;
                if options.ssid.replace(ssid).is_some() {
                    return Err(Error::DuplicateWifi);
                }
            }
            WIFI_KEY => {
                let path = value().ok_or(Error::MissingWifiKey)?;
                if options.wifi_key.replace(String::from(path)).is_some() {
                    return Err(Error::DuplicateWifiKey);
                }
            }
            IPV4 => {
                let ipv4 = value().and_then(StaticIpv4::parse).ok_or(Error::Ipv4)?;
                if options.ipv4.replace(ipv4).is_some() {
                    return Err(Error::DuplicateIpv4);
                }
            }
            IPV4_GATEWAY => {
                let gateway = value()
                    .and_then(|word| word.parse().ok())
                    .ok_or(Error::Ipv4Gateway)?;
                if options.ipv4_gateway.replace(gateway).is_some() {
                    return Err(Error::DuplicateIpv4Gateway);
                }
            }
            DNS => {
                let servers = value().and_then(dns_servers).ok_or(Error::Dns)?;
                if !core::mem::replace(&mut options.dns, servers).is_empty() {
                    return Err(Error::DuplicateDns);
                }
            }
            WIFI_COUNTRY => {
                let code = value().ok_or(Error::WifiCountry)?;
                if code.len() != 2 || !code.bytes().all(|byte| byte.is_ascii_alphabetic()) {
                    return Err(Error::WifiCountry);
                }
                if options
                    .wifi_country
                    .replace(code.to_ascii_uppercase())
                    .is_some()
                {
                    return Err(Error::DuplicateWifiCountry);
                }
            }
            USB4_APPROVE => {
                let uuid = value().and_then(Uuid::parse).ok_or(Error::Usb4Approve)?;
                if options.usb4_approved.contains(&uuid) {
                    return Err(Error::DuplicateUsb4Approve);
                }
                options.usb4_approved.push(uuid);
            }
            EXTERNAL_DISKS => options.external_disks = true,
            "--audio-check" => {
                if options.audio_check.is_some() {
                    return Err(Error::DuplicateAudioCheck);
                }
                let mode = AudioCheck::parse(value().ok_or(Error::MissingAudioMode)?)?;
                options.audio_check = Some(mode);
            }
            "--audio-own-hpet" => {
                flag(&mut options.audio_own_hpet, Error::DuplicateAudioHpet)?;
            }
            "--native-pci-own-ioapic" => {
                flag(
                    &mut options.native_pci_own_ioapic,
                    Error::DuplicateNativePciIoapic,
                )?;
            }
            "--network-check" => {
                let url = value().ok_or(Error::MissingNetworkUrl)?;
                parse_url(url)?;
                if options.network_check.replace(String::from(url)).is_some() {
                    return Err(Error::DuplicateNetworkCheck);
                }
            }
            "--ecid" => {
                let ecid = number(value().ok_or(Error::MissingEcid)?)?;
                if ecid == 0 || options.ecid.replace(ecid).is_some() {
                    return Err(Error::Ecid);
                }
            }
            "--sep-observe" => options.sep_observe = true,
            "--storage" => {
                let target = partition(
                    value().ok_or(Error::MissingStorage)?,
                    TargetArgument::Storage,
                )?;
                if options.storage_ram.is_some() || options.storage.replace(target).is_some() {
                    return Err(Error::AnsBacking);
                }
            }
            "--storage-ram" => {
                let bytes = number(value().ok_or(Error::MissingRamStorage)?)?;
                if options.storage.is_some() || options.storage_ram.replace(bytes).is_some() {
                    return Err(Error::AnsBacking);
                }
            }
            "--nor" => {
                let target = partition(value().ok_or(Error::MissingNor)?, TargetArgument::Nor)?;
                if options.nor.replace(target).is_some() {
                    return Err(Error::DuplicateNor);
                }
            }
            "--preboot" => {
                let target = value().ok_or(Error::MissingPreboot)?;
                options.preboot = Some(partition(target, TargetArgument::Preboot)?);
                options.bundle = true;
            }
            "--watch" => {
                let range = value().ok_or(Error::MissingWatchRange)?;
                let (base, bytes) = range.split_once(',').ok_or(Error::WatchRange)?;
                options.watches.push((number(base)?, number(bytes)?));
            }
            boot_selection::BOOT_VOLUME_OPTION => {
                let volume = BootVolume::parse(value().ok_or(Error::MissingBootVolume)?.as_bytes())
                    .map_err(|_| Error::BootVolume)?;
                if options.boot_volume.replace(volume).is_some() {
                    return Err(Error::DuplicateBootVolume);
                }
            }
            boot_selection::SAFE_BOOT_OPTION => options.safe_boot = true,
            boot_selection::BUNDLE_REPAIR_OPTION => options.bundle_repair = true,
            boot_selection::BOOT_COMMAND_OPTION => {
                let value = number(value().ok_or(Error::MissingBootCommand)?)?;
                let value = u8::try_from(value).map_err(|_| Error::BootCommandByte)?;
                let command = BootCommand::try_from(value)
                    .map_err(|source| Error::BootCommandOrigin { value, source })?;
                if options.boot_command.replace(command).is_some() {
                    return Err(Error::DuplicateBootCommand);
                }
            }
            boot_selection::UPGRADE_OPTION => options.upgrade = true,
            boot_selection::PAIRED_RECOVERY_OPTION | boot_selection::SYSTEM_RECOVERY_OPTION => {
                let kind = if name == boot_selection::PAIRED_RECOVERY_OPTION {
                    recovery::Kind::Paired
                } else {
                    recovery::Kind::System
                };
                if options.recovery.replace(kind).is_some() {
                    return Err(Error::RecoverySelection);
                }
            }
            "--dump" => {
                let range = value().ok_or(Error::MissingDumpRange)?;
                let (base, bytes) = range.split_once(',').ok_or(Error::DumpRange)?;
                options.dumps.push((Base::parse(base)?, number(bytes)?));
            }
            _ => {
                let value = number(value().ok_or(Error::MissingNumericValue)?)?;
                match name {
                    "--break" => options.breakpoints.push(value),
                    "--trace" => options.traces.push(value),
                    "--dram-base" => {
                        options.dram_base = value;
                        options.dram_explicit = true;
                    }
                    "--load" => options.load = value,
                    "--entry" => options.entry = value,
                    "--x0" => options.x0 = value,
                    "--budget" => options.budget = value,
                    _ => return Err(Error::UnknownSystemOption),
                }
            }
        }
        Ok(())
    }

    /// Normalize only after all words; validation sees this canonical state.
    fn finish(mut self) -> Result<System, Error> {
        let options = &mut self.options;
        if options.native_rounds || options.native_shared_check || options.native_bootstrap_check {
            options.parallel = false;
        }
        if options.ssid.is_some() && options.wifi_key.is_none() {
            options.wifi_key = Some(String::from(DEFAULT_WIFI_KEY));
        }
        options.check()?;
        Ok(self.options)
    }
}
