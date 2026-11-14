use super::record::{RECORD_KIND, RECORD_SCHEMA};
use super::*;
use host::{Cpu, Disk, Partition, PartitionInfo, Volume};
use std::collections::BTreeMap;
use stela_loader::mount_interface::Choices;
use stela_soc_data::nor::nvram::Variables;

const GIB: u64 = 1024 * 1024 * 1024;

fn sparse_image(path: &Path, bytes: u64) {
    let file = fs::File::create(path).unwrap();
    #[cfg(windows)]
    mark_sparse(&file);
    file.set_len(bytes).unwrap();
}

#[cfg(windows)]
fn mark_sparse(file: &fs::File) {
    use std::os::windows::io::AsRawHandle;
    use windows_sys::Win32::System::{IO::DeviceIoControl, Ioctl::FSCTL_SET_SPARSE};
    let mut returned = 0u32;
    // SAFETY: the handle is open for the file's lifetime, the control code
    // takes no buffers, and `returned` outlives the call.
    let ok = unsafe {
        DeviceIoControl(
            file.as_raw_handle() as _,
            FSCTL_SET_SPARSE,
            std::ptr::null(),
            0,
            std::ptr::null_mut(),
            0,
            &mut returned,
            std::ptr::null_mut(),
        )
    };
    assert!(ok != 0, "cannot mark the image sparse");
}

#[test]
fn partitions_are_named_alone_or_by_device_path() {
    let setup = crate::tests::parse(&[
        "create",
        "/dev/sdb",
        "--target",
        "/dev/nvme0n1p2",
        "--nor",
        "sdc3",
    ])
    .unwrap();
    assert_eq!(setup.usb, Path::new("/dev/sdb"));
    assert_eq!(setup.target.as_deref(), Some("nvme0n1p2"));
    assert_eq!(setup.nor.as_deref(), Some("sdc3"));
}

/// A computer whose volumes and disks the test declares.
pub(super) struct FakeHost {
    pub(super) volumes: BTreeMap<PathBuf, (FileSystem, u64, Option<&'static str>)>,
    pub(super) disks: Vec<Disk>,
    /// Partitions of the PC's disk, each backed by an image file.
    pub(super) partitions: BTreeMap<String, (PartitionInfo, PathBuf)>,
    pub(super) serials: BTreeMap<String, String>,
    pub(super) cpu: Cpu,
    /// The partitions `unmount` unmounted, in order.
    pub(super) unmounted: std::cell::RefCell<Vec<String>>,
    /// Where `mount` mounts a partition; others refuse.
    pub(super) mountable: BTreeMap<String, PathBuf>,
    pub(super) nvidia_gpus: std::cell::RefCell<Vec<u16>>,
    /// Bytes every volume has free beyond its declared free space, as
    /// removing files frees them.
    pub(super) freed: std::cell::Cell<u64>,
}

impl FakeHost {
    pub(super) fn new(
        volumes: BTreeMap<PathBuf, (FileSystem, u64, Option<&'static str>)>,
        disks: Vec<Disk>,
    ) -> Self {
        Self {
            volumes,
            disks,
            partitions: BTreeMap::new(),
            serials: BTreeMap::new(),
            cpu: Cpu { entropy: true },
            unmounted: Default::default(),
            mountable: BTreeMap::new(),
            nvidia_gpus: Default::default(),
            freed: Default::default(),
        }
    }

    fn image(&self, identifier: &str) -> Result<&Path> {
        self.partitions
            .get(identifier)
            .map(|(_, path)| path.as_path())
            .context("no such partition")
    }
}

impl Host for FakeHost {
    fn partition(&self, identifier: &str) -> Result<PartitionInfo> {
        let mut info = self
            .partitions
            .get(identifier)
            .map(|(info, _)| info.clone())
            .context("no such partition")?;
        if self
            .unmounted
            .borrow()
            .iter()
            .any(|done| done == identifier)
        {
            info.mount = None;
            info.container = None;
        }
        Ok(info)
    }
    fn unmount(&self, partition: &PartitionInfo) -> Result<()> {
        self.unmounted
            .borrow_mut()
            .push(partition.identifier.clone());
        Ok(())
    }
    fn mount(&self, identifier: &str) -> Result<PathBuf> {
        self.mountable
            .get(identifier)
            .cloned()
            .context("permission denied")
    }
    fn serial(&self, disk: &str) -> Result<Option<String>> {
        Ok(self.serials.get(disk).cloned())
    }
    // Images may be sparse files as large as the partition, so they
    // are read and written in place.
    fn read(&self, identifier: &str, offset: u64, length: usize) -> Result<Vec<u8>> {
        use std::io::{Read, Seek, SeekFrom};
        let mut file = fs::File::open(self.image(identifier)?)?;
        ensure!(
            offset + length as u64 <= file.metadata()?.len(),
            "read past the partition"
        );
        file.seek(SeekFrom::Start(offset))?;
        let mut bytes = vec![0; length];
        file.read_exact(&mut bytes)?;
        Ok(bytes)
    }
    fn write(&self, identifier: &str, offset: u64, data: &[u8]) -> Result<()> {
        use std::io::{Seek, SeekFrom, Write};
        let mut file = fs::OpenOptions::new()
            .write(true)
            .open(self.image(identifier)?)?;
        ensure!(
            offset + data.len() as u64 <= file.metadata()?.len(),
            "write past the partition"
        );
        file.seek(SeekFrom::Start(offset))?;
        file.write_all(data)?;
        Ok(())
    }
    fn cpu(&self) -> Result<Cpu> {
        Ok(self.cpu)
    }
    fn nvidia_gpus(&self) -> Result<Vec<u16>> {
        Ok(self.nvidia_gpus.borrow().clone())
    }
    fn volume(&self, path: &Path) -> Result<Volume> {
        let (filesystem, free_bytes, disk) = self
            .volumes
            .iter()
            .filter(|(root, _)| path.starts_with(root))
            .max_by_key(|(root, _)| root.components().count())
            .map(|(_, v)| v.clone())
            .context("no such volume")?;
        Ok(Volume {
            filesystem,
            free_bytes: free_bytes + self.freed.get(),
            disk: disk.map(str::to_owned),
        })
    }
    fn disks(&self) -> Result<Vec<Disk>> {
        Ok(self.disks.clone())
    }
}

fn disk(id: &str, contents: &[(&str, Option<&Path>)]) -> Disk {
    Disk {
        identifier: id.into(),
        name: format!("Disk {id}"),
        bytes: 128 * GIB,
        removal: match id {
            "disk5" => host::Removal::Usb,
            "disk6" => host::Removal::Typed,
            _ => host::Removal::Fixed,
        },
        system: id == "disk0",
        partitions: contents
            .iter()
            .enumerate()
            .map(|(i, (content, mount))| Partition {
                identifier: format!("{id}s{}", i + 1),
                content: (*content).into(),
                mount: mount.map(Path::to_path_buf),
            })
            .collect(),
    }
}

/// A `stela.efi` stand-in with this tree's mount interface.
/// The host firmware marker of a stela.efi before the bundle, which reads
/// exact per-PC files from the EFI partition.
fn per_pc_marker() -> [u8; 21] {
    let mut marker = stela_loader::host_firmware::MOUNT_CAPABILITY;
    marker[20] = 3;
    marker
}

fn efi_image(tag: &[u8]) -> Vec<u8> {
    let mut image = b"MZ".to_vec();
    image.extend_from_slice(tag);
    image.extend_from_slice(&mount_interface::marker(mount_interface::CURRENT));
    image
}

/// A USB (disk5) with an empty EFI partition and a data volume, HFS+ or
/// FAT, and this computer's system disk (disk0).
struct Fixture {
    _dir: tempfile::TempDir,
    esp: PathBuf,
    data: PathBuf,
    ipsw: PathBuf,
    efi: PathBuf,
    host: FakeHost,
}

fn fixture(esp_free: u64, fat_data: bool) -> Fixture {
    let dir = tempfile::tempdir().unwrap();
    let esp = dir.path().join("ESP");
    let data = dir.path().join("Data");
    fs::create_dir_all(&esp).unwrap();
    fs::create_dir_all(&data).unwrap();
    let data_filesystem = if fat_data {
        FileSystem::Fat
    } else {
        FileSystem::Hfs
    };
    let ipsw = dir.path().join("Synthetic.ipsw");
    efi::synthetic_ipsw(&ipsw).unwrap();
    let efi = dir.path().join("stela.efi");
    fs::write(&efi, efi_image(b"stela")).unwrap();
    let host = FakeHost::new(
        [
            (esp.clone(), (FileSystem::Fat, esp_free, Some("disk5"))),
            (data.clone(), (data_filesystem, 100 * GIB, Some("disk5"))),
            (
                dir.path().to_path_buf(),
                (FileSystem::Apfs, 100 * GIB, Some("disk0")),
            ),
        ]
        .into(),
        vec![
            disk("disk0", &[("EFI", None), ("Apple_APFS", None)]),
            disk("disk5", &[("EFI", Some(&esp)), ("Apple_HFS", Some(&data))]),
        ],
    );
    Fixture {
        _dir: dir,
        esp,
        data,
        ipsw,
        efi,
        host,
    }
}

fn command(f: &Fixture, usb: &Path) -> Request {
    Request {
        action: Action::Create,
        usb: usb.to_path_buf(),
        ipsw: Some(f.ipsw.clone()),
        fetch: false,
        // Most of these tests exercise a USB that holds the whole image, as
        // setups made before every install streamed it do.
        install_image: Some(InstallImage::Full),
        download_dir: None,
        efi: Some(f.efi.clone()),
        model: Some(restore::payload::SYNTHETIC_MODEL.into()),
        build: restore::payload::SYNTHETIC_BUILD.into(),
        // The smallest store: its spare is allocated at the bound, and
        // `snapshot` reads every file it compares. The default bound's
        // shrinking to fit is `translations::choose`'s to test.
        translation_store_bytes: Some(translations::MIN_PACK_BYTES),
        ..Request::default()
    }
}

/// The generation of the bundles this tree prepares.
fn current() -> bundle::Generation {
    bundle::Generation {
        schema: mount_interface::CURRENT.bundle_schema,
        revision: stela_loader::boot_bundle::REVISION,
    }
}

/// A stand-in for `boot prepare --restore`: the synthetic IPSW has no
/// kernel to prepare. The bundle's byte names the machine.
fn fake_prepare_restore(options: prepare::RamdiskOptions<'_>) -> Result<()> {
    let prepare::RamdiskOptions { ecid, output, .. } = options;
    let byte = ecid.map_or(0x5a, |e| e.0 as u8) ^ 0xa5;
    bundle::tests::write(output, current(), &[byte; 2048])
}

/// A stand-in for `boot prepare --restore --update`.
fn fake_prepare_update(options: prepare::RamdiskOptions<'_>) -> Result<()> {
    let prepare::RamdiskOptions { ecid, output, .. } = options;
    let byte = ecid.map_or(0x5a, |e| e.0 as u8) ^ 0x3c;
    bundle::tests::write(output, current(), &[byte; 1024])
}

/// The synthetic IPSW's inventory: it names no AEA member to fetch a
/// key for.
fn fake_inventory(ipsw: &Path, model: &str) -> Result<restore::payload::Inventory> {
    restore::payload::inventory(ipsw, model, restore::payload::KeyOptions::default())
}

/// The synthetic IPSW's inventory as if it were on Apple's servers.
fn fake_remote_inventory(ipsw: &Path, model: &str) -> Result<restore::payload::Inventory> {
    let layout = stela_formats::payload::Layout::Remote {
        url: "http://updates-http.example.test/Synthetic.ipsw".into(),
        etag: "\"synthetic\"".into(),
    };
    let bytes = fs::metadata(ipsw)?.len();
    restore::payload::remote_inventory(
        fs::File::open(ipsw)?,
        restore::payload::RemoteInventoryOptions {
            image: ipsw,
            bytes,
            layout,
            model,
            keys_in: None,
        },
    )
}

/// Tests reach no server, and search and write only a cache of their own.
static TEST_CACHES: std::sync::LazyLock<[PathBuf; 1]> = std::sync::LazyLock::new(|| {
    [tempfile::Builder::new()
        .prefix("stela-test-cache-")
        .tempdir()
        .unwrap()
        .keep()]
});

/// What `boot prepare` reads of the synthetic images, which have no
/// kernel: their manifest and restore ramdisk.
fn fake_bundle_inputs(
    _: &mut zip::ZipArchive<&mut dyn restore::ImageSource>,
    _: &str,
) -> Result<Vec<String>> {
    Ok(vec![
        "BuildManifest.plist".into(),
        "stelatest-ramdisk.dmg".into(),
    ])
}

fn offline_image_size(_: &str, _: &str) -> Result<(u64, u64)> {
    anyhow::bail!("offline")
}

fn offline_fetch_image(_: &Path, _: &str, _: &str, _: sources::Need) -> Result<sources::Fetched> {
    panic!("tests never download")
}

fn environment(host: &FakeHost) -> Environment<'_> {
    Environment {
        host,
        download: None,
        image_caches: &*TEST_CACHES,
        image_size: &offline_image_size,
        fetch_image: &offline_fetch_image,
        ui: &ui::NoTerminal,
        wifi_networks: &Vec::new,
        wifi_country: &|| None,
        bundle_inputs: &fake_bundle_inputs,
        prepare_restore: &fake_prepare_restore,
        prepare_update: &fake_prepare_update,
        inventory: &fake_inventory,
        remote_inventory: &fake_remote_inventory,
        piece_bytes: medium::tests::PIECE_BYTES,
        interface: mount_interface::CURRENT,
        bundle_revision: stela_loader::boot_bundle::REVISION,
        builds: SYNTHETIC_BUILDS,
        models: SYNTHETIC_MODELS,
    }
}

/// The synthetic images' model beside the supported ones.
const SYNTHETIC_MODELS: &[(&str, &str)] = &[
    (PINNED_MODEL, "Mac mini M2"),
    (restore::payload::SYNTHETIC_MODEL, "Stela test model"),
];

/// The synthetic images' builds, as supported ones.
const SYNTHETIC_BUILDS: &[builds::Build] = &[
    synthetic_build(restore::payload::SYNTHETIC_BUILD),
    synthetic_build("STELA2"),
    synthetic_build("STELA3"),
];

const fn synthetic_build(build: &'static str) -> builds::Build {
    builds::Build {
        build,
        version: "27.0",
        dcp_link_version: builds::DEFAULT.dcp_link_version,
        rtkit_version: builds::DEFAULT.rtkit_version,
    }
}

#[test]
fn preparation_rejects_unsupported_build() {
    let f = fixture(64 * GIB, true);
    let environment = Environment {
        builds: builds::SUPPORTED,
        ..environment(&f.host)
    };
    let error = setup(&environment, command(&f, &f.esp))
        .unwrap_err()
        .to_string();
    assert!(
        error.contains(&format!(
            "macOS build {} is not supported",
            restore::payload::SYNTHETIC_BUILD
        )),
        "{error}"
    );
    assert!(error.contains(builds::DEFAULT.build), "{error}");
}

#[test]
fn other_mac_models_are_refused_naming_the_supported_one() {
    let f = fixture(64 * GIB, true);
    let environment = Environment {
        models: SUPPORTED_MODELS,
        ..environment(&f.host)
    };
    for model in ["MacBookPro18,3", "Mac14,14", "Mac14,2", "Mac16,10"] {
        let mut request = command(&f, &f.esp);
        request.model = Some(model.into());
        let error = request.dispatch(&environment).unwrap_err().to_string();
        assert_eq!(
            error,
            format!("Mac model {model} is not supported yet; Stela supports Mac14,3 (Mac mini M2)")
        );
    }
    assert_eq!(PINNED_MODEL, "Mac14,3");
}

/// Stand-ins for the restore and update restore `boot prepare` of a
/// stela whose loader prepares `revision`; each bundle's byte names its
/// machine, kind and revision.
fn preparers(revision: u32) -> [Box<Prepare>; 2] {
    let of = |kind: u8, bytes: usize| -> Box<Prepare> {
        Box::new(move |options: prepare::RamdiskOptions<'_>| {
            let prepare::RamdiskOptions { ecid, output, .. } = options;
            let byte = ecid.map_or(0x5a, |e| e.0 as u8) ^ kind ^ revision as u8;
            let generation = bundle::Generation {
                revision,
                ..current()
            };
            bundle::tests::write(output, generation, &vec![byte; bytes])
        })
    };
    [of(0xa5, 2048), of(0x3c, 1024)]
}

/// The environment of a stela whose loader prepares `revision`.
fn environment_at<'a>(
    host: &'a FakeHost,
    preparers: &'a [Box<Prepare>; 2],
    revision: u32,
) -> Environment<'a> {
    Environment {
        prepare_restore: &*preparers[0],
        prepare_update: &*preparers[1],
        bundle_revision: revision,
        ..environment(host)
    }
}

/// Every file under `root`, with its bytes.
fn snapshot(root: &Path) -> BTreeMap<PathBuf, Vec<u8>> {
    let mut files = BTreeMap::new();
    let mut stack = vec![root.to_path_buf()];
    while let Some(path) = stack.pop() {
        for entry in fs::read_dir(&path).unwrap() {
            let entry = entry.unwrap();
            if entry.file_type().unwrap().is_dir() {
                stack.push(entry.path());
            } else {
                files.insert(entry.path(), fs::read(entry.path()).unwrap());
            }
        }
    }
    files
}

fn record(f: &Fixture) -> Record {
    stela_engine::read_json(&f.esp.join(RECORD)).unwrap()
}

/// The active build's restore bundle of `ecid`'s machine.
fn bundle_of(f: &Fixture, ecid: Option<Ecid>) -> PathBuf {
    let record = record(f);
    bundle::directory_of(
        &record.volume,
        bundle::Identity {
            kind: Kind::Restore,
            build: &record.build,
            ecid,
            generation: record.generation(),
        },
    )
}

#[test]
fn setup_installs_from_the_usb_and_uninstall_restores_it() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let volume = stela_volume(&f);
    let before = (snapshot(&f.esp), snapshot(&f.data), snapshot(&volume));
    let host = &f.host;

    let mut dry = command(&f, &f.esp);
    dry.dry_run = true;
    host.nvidia_gpus.replace(vec![0x2c02, 0x2c05, 0x9999]);
    let plan = setup(&environment(host), dry).unwrap();
    host.nvidia_gpus.take();
    // No firmware follows this computer's GPUs; an unsupported one is a note.
    assert!(!plan.contains("gb203"), "{plan}");
    assert!(plan.contains("NVIDIA GPU (device 9999)"), "{plan}");
    assert!(plan.contains("Dry run: nothing was changed."), "{plan}");
    assert!(plan.contains("ECID           0x"), "{plan}");
    assert!(plan.contains("gs.apple.com"), "{plan}");
    // macOS is on the USB, and the plan still says the install needs the
    // network, for the signing server alone.
    let local = create::OFFLINE_SIGNING_NOTE;
    assert!(plan.contains(local), "{plan}");
    assert!(plan.contains("restore        "), "{plan}");
    assert!(!plan.contains("recoveryOS     "), "{plan}");
    assert_eq!(
        (snapshot(&f.esp), snapshot(&f.data), snapshot(&volume)),
        before
    );

    let report = setup(&environment(host), command(&f, &f.esp)).unwrap();
    assert!(report.contains("Check it with `stela status`"), "{report}");
    assert!(!report.contains("uninstall"), "{report}");
    assert!(report.contains("pick the PC disk"), "{report}");
    let record = record(&f);
    let ecid = record.installs[0].ecid.unwrap();
    assert_ne!(ecid.0, 0);
    let options = fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert_eq!(
        options,
        format!(
            "--system --storage @disk --nor @disk --preboot @disk --ecid {ecid} \
             --external-disks --guest-network --input\n"
        )
    );
    assert!(!options.contains("--bundle"), "{options}");
    // The only bundle is the machine's restore bundle.
    assert_eq!(record.bundles.len(), 1);
    assert_eq!(record.bundles[0].kind, Kind::Restore);
    let mut sized = record.clone();
    sized.bundles[0].processors = 8;
    let line = sized.line(stela_boot_core::options::System::default());
    assert!(line.contains("--guest-pes 8"), "{line}");
    let bundle = bundle_of(&f, Some(ecid));
    assert_eq!(
        fs::read(bundle.join("ramdisk.img")).unwrap(),
        vec![ecid.0 as u8 ^ 0xa5; 2048]
    );
    assert!(bundle::verify(&bundle, &record.bundles[0].manifest_sha256).is_empty());
    assert!(!f.esp.join("EFI/stela/source.bin").exists());
    let choices = Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    let names: Vec<&str> = choices.entries.iter().map(|e| e.name.as_str()).collect();
    assert_eq!(names, ["macOS", INSTALL_CHOICE]);
    assert_eq!(
        fs::read(f.esp.join(efi::LOADER)).unwrap(),
        efi_image(b"stela")
    );
    assert!(record.complete);
    assert_eq!(record.schema_version, RECORD_SCHEMA);
    // A second setup is refused until the first is undone.
    let again = setup(&environment(host), command(&f, &f.esp));
    let again = again.unwrap_err().to_string();
    assert!(again.contains("start over with `stela create`"), "{again}");
    assert!(!again.contains("uninstall"), "{again}");
    // The only install is not removed alone.
    let error = rerun(&f, |c| {
        c.action = Action::Configure;
        c.remove_install = Some("macOS".into());
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("only one"), "{error}");

    // The firmware bundle a setup with a bundle-reading stela.efi puts on
    // the volume goes too.
    let firmware = record
        .volume
        .join(stela_loader::host_firmware::bundle::DIRECTORY);
    fs::create_dir_all(firmware.join("licenses")).unwrap();
    fs::write(firmware.join("bundle.json"), b"{}").unwrap();
    let dry = uninstall(&environment(host), &f.esp, true).unwrap();
    assert!(dry.contains("Dry run"));
    assert!(
        dry.contains(&format!("remove   {}", firmware.display())),
        "{dry}"
    );
    assert!(bundle.is_dir() && firmware.is_dir());
    let done = uninstall(&environment(host), &f.esp, false).unwrap();
    assert!(done.contains("Done."));
    assert_eq!(
        (snapshot(&f.esp), snapshot(&f.data), snapshot(&volume)),
        before
    );
}

#[test]
fn setup_rejects_efi_interface_mismatch() {
    let f = fixture(64 * GIB, false);
    fs::write(&f.efi, b"MZ without a marker").unwrap();
    let error = setup(&environment(&f.host), command(&f, &f.esp))
        .unwrap_err()
        .to_string();
    assert!(error.contains("no Stela mount interface marker"), "{error}");
    let other = mount_interface::Interface {
        options_revision: mount_interface::OPTIONS_REVISION + 1,
        ..mount_interface::CURRENT
    };
    let mut image = b"MZ".to_vec();
    image.extend_from_slice(&mount_interface::marker(other));
    fs::write(&f.efi, image).unwrap();
    let error = setup(&environment(&f.host), command(&f, &f.esp))
        .unwrap_err()
        .to_string();
    assert!(error.contains("stela.cfg option revision"), "{error}");
    assert!(!f.esp.join(RECORD).exists());
}

#[test]
fn setup_places_bundles_on_fat32_data_volume() {
    // The firmware cannot read HFS+, so the USB has no data partition.
    let f = fixture(GIB / 32, false);
    let error = setup(&environment(&f.host), command(&f, &f.data))
        .unwrap_err()
        .to_string();
    assert!(error.contains("not FAT32"), "{error}");
    assert!(
        error.contains("boot bundles and the install medium"),
        "{error}"
    );
    // A FAT data partition holds the bundle and the network install's
    // inventory under `stela/`; the EFI partition keeps only the boot
    // loader and its configuration.
    let mut f = fixture(GIB / 32, false);
    add_stela(&mut f);
    let volume = stela_volume(&f);
    let mut network = command(&f, &volume);
    network.install_image = Some(InstallImage::Network);
    setup(&environment(&f.host), network).unwrap();
    let ecid = record(&f).installs[0].ecid;
    assert!(bundle_of(&f, ecid).join("ramdisk.img").is_file());
    assert!(bundle_of(&f, ecid).starts_with(volume.join("stela/boot")));
    assert!(volume.join("stela/install").is_dir());
    assert!(!f.esp.join("stela").exists());
    assert!(f.esp.join(efi::LOADER).is_file());
    uninstall(&environment(&f.host), &volume, false).unwrap();
    assert!(!volume.join("stela").exists());
}

#[test]
fn usbs_setup_cannot_boot_from_are_explained_and_left_unchanged() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    // A USB without an EFI partition, one whose partition is not
    // mounted, and one that already boots another loader.
    let dir = tempfile::tempdir().unwrap();
    let mut host = FakeHost::new(f.host.volumes.clone(), Vec::new());
    host.volumes
        .insert(dir.path().to_path_buf(), (FileSystem::Fat, GIB, None));
    let error = setup(&environment(&host), command(&f, dir.path()))
        .unwrap_err()
        .to_string();
    assert!(error.contains("given its whole disk"), "{error}");
    let mut host = FakeHost::new(f.host.volumes.clone(), f.host.disks.clone());
    host.partitions = f.host.partitions.clone();
    // An unmounted EFI partition is mounted, or the error says why not.
    host.disks[1].partitions[0].mount = None;
    let error = setup(&environment(&host), command(&f, &f.data))
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("disk5s1, which could not be mounted"),
        "{error}"
    );
    host.mountable.insert("disk5s1".into(), f.esp.clone());
    let mut dry = command(&f, &f.data);
    dry.dry_run = true;
    let plan = setup(&environment(&host), dry).unwrap();
    assert!(plan.contains(&f.esp.display().to_string()), "{plan}");
    let loader = f.esp.join(efi::LOADER);
    fs::create_dir_all(loader.parent().unwrap()).unwrap();
    fs::write(&loader, b"MZopencore").unwrap();
    let before = snapshot(&f.esp);
    let error = setup(&environment(&f.host), command(&f, &f.esp))
        .unwrap_err()
        .to_string();
    assert!(error.contains("another loader"), "{error}");
    assert_eq!(snapshot(&f.esp), before);
    // An internal EFI partition is another system's boot partition.
    let f = fixture(64 * GIB, false);
    let mut host = FakeHost::new(f.host.volumes.clone(), f.host.disks.clone());
    host.volumes
        .insert(f.esp.clone(), (FileSystem::Fat, 64 * GIB, Some("disk0")));
    host.disks[0].partitions[0].mount = Some(f.esp.clone());
    let error = setup(&environment(&host), command(&f, &f.esp))
        .unwrap_err()
        .to_string();
    assert!(error.contains("internal disk"), "{error}");
}

#[test]
fn older_setups_are_still_read_and_undone() {
    // Schema 1, with a payload and a target.
    let f = fixture(64 * GIB, false);
    let before = snapshot(&f.esp);
    efi::add_mount(
        &f.esp,
        &fs::read(&f.efi).unwrap(),
        None,
        Some("--install-mode verify\n"),
    )
    .unwrap();
    let payload = f.esp.join("EFI/stela/images/26A434");
    let bundles = f.esp.join("EFI/stela/boot/26A434");
    for directory in [&payload, &bundles] {
        fs::create_dir_all(directory).unwrap();
        fs::write(directory.join("file"), b"x").unwrap();
    }
    let record = serde_json::json!({
        "schema_version": 1,
        "kind": RECORD_KIND,
        "build": "26A434",
        "model": "Mac14,3",
        "target": "the disk with serial number EMPTY2",
        "esp": f.esp,
        "payload_volume": f.esp,
        "payload_directory": payload,
        "bundle_directory": bundles,
        "created_directories": [
            f.esp.join("EFI/stela/images"),
            f.esp.join("EFI/stela/boot"),
        ],
        "options": "--install-mode verify\n",
        "complete": true,
    });
    stela_engine::write_json(&f.esp.join(RECORD), &record).unwrap();
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);

    // Schema 7, with a recoveryOS bundle on the USB: this stela no longer
    // reads it, and uninstalls it.
    let f = fixture(64 * GIB, false);
    let before = snapshot(&f.esp);
    efi::add_mount(&f.esp, &fs::read(&f.efi).unwrap(), None, Some("--system\n")).unwrap();
    let build = f.esp.join("EFI/stela/boot/STELA1");
    let recovery = build.join("recovery-0000000000000001");
    let restore = build.join("restore-0000000000000001");
    bundle::tests::write(&recovery, bundle::Generation::FIRST, b"old").unwrap();
    bundle::tests::write(&restore, bundle::Generation::FIRST, b"old").unwrap();
    let disk = installed::InstalledDisk {
        apple_region: REGION.into(),
        apple_region_serial: SERIAL.into(),
        apple_region_guid: REGION_GUID.into(),
        nor: NOR.into(),
        nor_serial: SERIAL.into(),
        nor_guid: NOR_GUID.into(),
        machine_state_offset: None,
        external_disks: false,
    };
    let mut install = serde_json::to_value(&disk).unwrap();
    install["name"] = "macOS".into();
    install["ecid"] = "0x1".into();
    let record = serde_json::json!({
        "schema_version": 7,
        "kind": RECORD_KIND,
        "build": "STELA1",
        "model": "Mac14,3",
        "esp": f.esp,
        "volume": f.esp,
        "bundle_directory": build,
        "created_directories": [build, f.esp.join("EFI/stela/boot")],
        "options": "--system\n",
        "complete": true,
        "recovery_builds": ["STELA1"],
        "ecid": "0x1",
        "installs": [install],
        "bundles": [
            {"build": "STELA1", "ecid": "0x1", "schema": 1, "manifest_sha256": ""},
            {"build": "STELA1", "ecid": "0x1", "schema": 1, "manifest_sha256": "",
             "kind": "restore", "personalized": true},
        ],
    });
    stela_engine::write_json(&f.esp.join(RECORD), &record).unwrap();
    let error = read_record(&environment(&f.host), &f.esp)
        .unwrap_err()
        .to_string();
    assert!(error.contains("schema 7 setup"), "{error}");
    assert!(error.contains("recoveryOS bundle"), "{error}");
    assert!(error.contains("Start over with `stela create`"), "{error}");
    let error = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(error.contains("no longer reads"), "{error}");
    let dry = uninstall(&environment(&f.host), &f.esp, true).unwrap();
    assert!(dry.contains("keep     install \"macOS\""), "{dry}");
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);

    // Schema 8, which kept the data partition's bundles, install medium
    // and translation store under `EFI/stela`: refused, and uninstalled
    // from where it wrote them.
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let volume = stela_volume(&f);
    let before = (snapshot(&f.esp), snapshot(&volume));
    efi::add_mount(&f.esp, &fs::read(&f.efi).unwrap(), None, Some("--system\n")).unwrap();
    let old = volume.join("EFI/stela");
    let build = old.join("boot/STELA1");
    bundle::tests::write(
        &build.join("restore-0000000000000001"),
        bundle::Generation::FIRST,
        b"old",
    )
    .unwrap();
    for (directory, file) in [("install/STELA1", "payload.json"), ("store", "store.cfg")] {
        fs::create_dir_all(old.join(directory)).unwrap();
        fs::write(old.join(directory).join(file), b"x").unwrap();
    }
    let mut install = serde_json::to_value(&disk).unwrap();
    install["name"] = "macOS".into();
    install["ecid"] = "0x1".into();
    let record = serde_json::json!({
        "schema_version": 8,
        "kind": RECORD_KIND,
        "build": "STELA1",
        "model": "Mac14,3",
        "esp": f.esp,
        "volume": volume,
        "bundle_directory": build,
        "created_directories": [build, old.join("boot")],
        "options": "--system\n",
        "complete": true,
        "builds": ["STELA1"],
        "installs": [install],
        "bundles": [
            {"build": "STELA1", "ecid": "0x1", "schema": 1, "manifest_sha256": "",
             "personalized": true},
        ],
        "media": [
            {"build": "STELA1", "volume": volume, "partition_guid": STELA_GUID,
             "manifest_sha256": ""},
        ],
        "translations": {"volume": volume, "pack_bytes": translations::MIN_PACK_BYTES},
    });
    stela_engine::write_json(&f.esp.join(RECORD), &record).unwrap();
    let error = read_record(&environment(&f.host), &f.esp)
        .unwrap_err()
        .to_string();
    assert!(error.contains("under EFI/"), "{error}");
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!((snapshot(&f.esp), snapshot(&volume)), before);
    assert!(!old.join("boot").exists());
}

const REGION: &str = "disk1s2";
const NOR: &str = "disk1s3";
const REGION_GUID: &str = "5E1A0000-0000-4000-8000-000000000001";
const NOR_GUID: &str = "5E1A0000-0000-4000-8000-000000000002";
const REGION2: &str = "disk2s2";
const NOR2: &str = "disk2s3";
const REGION2_GUID: &str = "5E1A0000-0000-4000-8000-000000000003";
const NOR2_GUID: &str = "5E1A0000-0000-4000-8000-000000000004";
const SERIAL: &str = "STELAPC01";
const SERIAL2: &str = "STELAPC02";
/// Bytes of each synthetic partition image: the probe reads and a NOR.
const IMAGE_BYTES: usize = stela_runtime::pc::boot_log::PARTITION_BYTES as usize;
const MIB: u64 = 1024 * 1024;

/// Give `f`'s PC a disk (`disk1`, or `disk2` with `second`) with an
/// Apple region and a NOR partition, backed by synthetic images that
/// start with `region` and `nor`.
fn add_disk(f: &mut Fixture, second: bool, region: &[u8], nor: &[u8]) {
    add_stela(f);
    let (disk, serial, partitions) = if second {
        (
            "disk2",
            SERIAL2,
            [(REGION2, REGION2_GUID, region), (NOR2, NOR2_GUID, nor)],
        )
    } else {
        (
            "disk1",
            SERIAL,
            [(REGION, REGION_GUID, region), (NOR, NOR_GUID, nor)],
        )
    };
    for ((identifier, guid, head), bytes) in partitions.into_iter().zip([100 * GIB, 256 * MIB]) {
        let path = f._dir.path().join(format!("{identifier}.img"));
        let mut image = vec![0u8; IMAGE_BYTES];
        image[..head.len()].copy_from_slice(head);
        fs::write(&path, image).unwrap();
        let info = PartitionInfo {
            identifier: identifier.into(),
            disk: disk.into(),
            guid: guid.to_ascii_lowercase(),
            bytes,
            content: "Apple_APFS".into(),
            mount: None,
            container: None,
        };
        f.host.partitions.insert(identifier.into(), (info, path));
    }
    f.host.serials.insert(disk.into(), serial.into());
}

fn add_installed_disk(f: &mut Fixture, region: &[u8], nor: &[u8]) {
    add_disk(f, false, region, nor);
}

fn installed_command(f: &Fixture) -> Request {
    let mut command = command(f, &f.esp);
    command.target = Some(REGION.into());
    command.nor = Some(NOR.into());
    command
}

/// The USB's FAT32 data partition, which holds the install medium.
const STELA_PARTITION: &str = "disk5s3";
const STELA_GUID: &str = "5E1A0000-0000-4000-8000-0000000000aa";

fn stela_volume(f: &Fixture) -> PathBuf {
    f._dir.path().join("STELA")
}

/// Give the USB its FAT32 data partition, once.
fn add_stela(f: &mut Fixture) {
    let volume = stela_volume(f);
    if volume.exists() {
        return;
    }
    fs::create_dir_all(&volume).unwrap();
    f.host
        .volumes
        .insert(volume.clone(), (FileSystem::Fat, 100 * GIB, Some("disk5")));
    let usb = f
        .host
        .disks
        .iter_mut()
        .find(|d| d.identifier == "disk5")
        .unwrap();
    usb.partitions.push(Partition {
        identifier: STELA_PARTITION.into(),
        content: "Microsoft Basic Data".into(),
        mount: Some(volume.clone()),
    });
    let info = PartitionInfo {
        identifier: STELA_PARTITION.into(),
        disk: "disk5".into(),
        guid: STELA_GUID.into(),
        bytes: 100 * GIB,
        content: "Microsoft Basic Data".into(),
        mount: Some(volume.clone()),
        container: None,
    };
    let image = f._dir.path().join("stela-partition.img");
    f.host
        .partitions
        .insert(STELA_PARTITION.into(), (info, image));
}

fn partition_bytes(f: &Fixture, identifier: &str) -> Vec<u8> {
    fs::read(f.host.image(identifier).unwrap()).unwrap()
}

fn nvram_of(f: &Fixture, nor: &str) -> Variables {
    let bytes = partition_bytes(f, nor)[..stela_soc_data::nor::CAPACITY].to_vec();
    let image = stela_soc_data::nor::Image::from_bytes(bytes).unwrap();
    Variables::parse(image.bank(image.current_bank().unwrap()).unwrap()).unwrap()
}

fn nvram(f: &Fixture) -> Variables {
    nvram_of(f, NOR)
}

/// Give the USB an unmounted raw partition, as the Mac's internal SSD.
fn add_stick_ssd(f: &mut Fixture, identifier: &str, guid: &str) {
    add_stela(f);
    let usb = f
        .host
        .disks
        .iter_mut()
        .find(|d| d.identifier == "disk5")
        .unwrap();
    usb.partitions.push(Partition {
        identifier: identifier.into(),
        content: "FFFFFFFF-FFFF-FFFF-FFFF-FFFFFFFFFFFF".into(),
        mount: None,
    });
    // Sparse, as large as the partition: the machine state is its tail.
    let path = f._dir.path().join(format!("{identifier}.img"));
    sparse_image(&path, 100 * GIB);
    let info = PartitionInfo {
        identifier: identifier.into(),
        disk: "disk5".into(),
        guid: guid.to_ascii_lowercase(),
        bytes: 100 * GIB,
        content: "FFFFFFFF-FFFF-FFFF-FFFF-FFFFFFFFFFFF".into(),
        mount: None,
        container: None,
    };
    f.host.partitions.insert(identifier.into(), (info, path));
    f.host.serials.insert("disk5".into(), "STICK01".into());
}

#[test]
fn install_maps_usb_partition_as_guest_ssd() {
    let mut f = fixture(64 * GIB, false);
    add_stick_ssd(&mut f, "disk5s4", REGION_GUID);
    let opted = command(&f, &f.esp);
    let report = setup(&environment(&f.host), opted).unwrap();
    assert!(report.contains("external drives"), "{report}");
    let record = record(&f);
    let disk = &record.installs[0].disk;
    assert_eq!(disk.apple_region, "disk5s4");
    assert!(disk.external_disks);
    let options = fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert!(
        options.contains(&format!("--storage STICK01:{REGION_GUID}")),
        "{options}"
    );
    assert!(options.contains("--external-disks"), "{options}");
    let choices = fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap();
    let install = choices
        .lines()
        .find(|line| line.starts_with(INSTALL_CHOICE))
        .unwrap();
    assert!(install.contains("--development-restore"), "{install}");

    // Two raw partitions name no one internal SSD.
    let mut f = fixture(64 * GIB, false);
    add_stick_ssd(&mut f, "disk5s4", REGION_GUID);
    add_stick_ssd(&mut f, "disk5s5", REGION2_GUID);
    let opted = command(&f, &f.esp);
    let error = setup(&environment(&f.host), opted).unwrap_err().to_string();
    assert!(error.contains("several unmounted partitions"), "{error}");
}

#[test]
fn setup_erases_usb_and_refuses_internal_disk() {
    let f = fixture(64 * GIB, true);
    let whole = |usb: &str| f.host.whole_disk(Path::new(usb)).unwrap();
    assert_eq!(whole("/dev/disk5").as_deref(), Some("disk5"));
    assert_eq!(whole("disk0").as_deref(), Some("disk0"));
    let esp = f.esp.to_str().unwrap();
    for volume in [
        esp,
        "/Volumes/STELA",
        "disk5s1",
        "disk",
        "/dev/rdisk5",
        "disk12",
    ] {
        assert_eq!(whole(volume), None, "{volume}");
    }
    let mut dry = command(&f, Path::new("disk5"));
    dry.dry_run = true;
    let report = setup(&environment(&f.host), dry).unwrap();
    assert!(report.contains("would erase disk5"), "{report}");
    let mut internal = command(&f, Path::new("disk0"));
    internal.dry_run = true;
    let error = setup(&environment(&f.host), internal)
        .unwrap_err()
        .to_string();
    assert!(error.contains("internal disk"), "{error}");
    // An SD card is erased only with its model or size typed back.
    let mut host = FakeHost::new(f.host.volumes.clone(), f.host.disks.clone());
    host.disks.push(disk("disk6", &[]));
    let mut card = command(&f, Path::new("disk6"));
    card.dry_run = true;
    let error = setup(&environment(&host), card).unwrap_err().to_string();
    assert!(error.contains("--confirm-disk"), "{error}");
    assert!(
        error.contains("disk6 (Disk disk6, 137.4 GB) is not a USB disk;"),
        "{error}"
    );
    // A USB on a computer whose own disks setup cannot tell: refused for
    // that reason alone.
    host.disks.push(Disk {
        removal: host::Removal::Unsure,
        ..disk("disk7", &[])
    });
    let mut unsure = command(&f, Path::new("disk7"));
    unsure.dry_run = true;
    let error = setup(&environment(&host), unsure).unwrap_err().to_string();
    assert!(error.contains("--confirm-disk"), "{error}");
    assert!(
        error.contains("may be the disk this computer runs from"),
        "{error}"
    );
    assert!(!error.contains("not a USB disk"), "{error}");
    for typed in ["Disk disk6", "137.4 GB"] {
        let mut card = command(&f, Path::new("disk6"));
        card.dry_run = true;
        card.confirm_disk = Some(typed.into());
        let report = setup(&environment(&host), card).unwrap();
        assert!(report.contains("would erase disk6"), "{report}");
    }
    // Without an image, the USB is left as it was: no erase is tried.
    let mut no_image = command(&f, Path::new("disk5"));
    no_image.ipsw = None;
    no_image.confirmed = true;
    let error = setup(&environment(&f.host), no_image)
        .unwrap_err()
        .to_string();
    assert!(error.contains("pass --ipsw"), "{error}");
    // Without a terminal and without the model or size typed back, a USB
    // is not erased; --yes does not change that.
    let mut unasked = command(&f, Path::new("disk5"));
    unasked.ipsw = None;
    unasked.yes = true;
    let error = setup(&environment(&f.host), unasked)
        .unwrap_err()
        .to_string();
    assert!(error.contains("without a terminal"), "{error}");
    assert!(error.contains("--confirm-disk"), "{error}");
    // On a terminal the erase is asked, and a no or Escape erases nothing.
    let before = (snapshot(&f.esp), snapshot(&f.data));
    for answer in ["no", "cancel"] {
        let person = ui::tests::Scripted::new(&[answer]);
        let mut asked = command(&f, Path::new("disk5"));
        asked.ipsw = None;
        let environment = Environment {
            ui: &person,
            ..environment(&f.host)
        };
        let error = setup(&environment, asked).unwrap_err().to_string();
        assert_eq!(error, "disk5 was not erased; nothing was changed");
        assert!(
            person.transcript().starts_with("? Erase disk5 ("),
            "{}",
            person.transcript()
        );
    }
    assert_eq!((snapshot(&f.esp), snapshot(&f.data)), before);
}

#[test]
fn preparation_preflights_signing_and_capacity() {
    let mut f = fixture(64 * GIB, true);
    let before = (snapshot(&f.esp), snapshot(&f.data));
    // Confirmed, so the preflights run without a terminal to ask at.
    let mut opted = command(&f, Path::new("disk5"));
    opted.confirmed = true;
    let fail = |_: prepare::RamdiskOptions<'_>| bail!("signing service unavailable");
    let env = Environment {
        prepare_restore: &fail,
        ..environment(&f.host)
    };
    let error = setup(&env, opted).unwrap_err();
    assert!(format!("{error:#}").contains("signing service unavailable"));
    assert_eq!((snapshot(&f.esp), snapshot(&f.data)), before);

    f.host
        .disks
        .iter_mut()
        .find(|d| d.identifier == "disk5")
        .unwrap()
        .bytes = 1;
    // Offline is refused on it first; Online meets the capacity check.
    let mut opted = command(&f, Path::new("disk5"));
    opted.confirmed = true;
    let error = setup(&environment(&f.host), opted).unwrap_err();
    assert!(
        error.to_string().starts_with("Offline needs a USB"),
        "{error:#}"
    );
    let mut opted = command(&f, Path::new("disk5"));
    opted.confirmed = true;
    opted.install_image = None;
    let error = setup(&environment(&f.host), opted).unwrap_err();
    assert!(
        error.to_string().contains("needs a USB of at least"),
        "{error:#}"
    );
    assert_eq!((snapshot(&f.esp), snapshot(&f.data)), before);
}

#[test]
fn preparation_rejects_replaced_usb() {
    let mut f = fixture(64 * GIB, true);
    f.host.serials.insert("disk5".into(), "original".into());
    let selected = wizard::Usb::from_disk(&f.host.disks[1], &f.host);
    selected.verify(&f.host).unwrap();
    f.host.serials.insert("disk5".into(), "replacement".into());
    assert!(selected.verify(&f.host).is_err());
    f.host.serials.insert("disk5".into(), "original".into());
    f.host.disks[1].system = true;
    assert!(selected.verify(&f.host).is_err());
    f.host.disks.pop();
    assert!(selected.verify(&f.host).is_err());
}

#[test]
fn targetless_setup_defers_disk_selection_to_efi() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    // The authoring computer need not have the target's CPU features.
    f.host.cpu.entropy = false;
    let opted = command(&f, &f.esp);
    let report = setup(&environment(&f.host), opted).unwrap();
    assert!(report.contains("pick the PC disk"), "{report}");
    let record = record(&f);
    assert!(record.installs[0].disk.is_chosen());
    let medium = record.active_medium().unwrap();
    assert!(!medium.remote && medium.problems().is_empty(), "{medium:?}");
    let choices = fs::read_to_string(f.esp.join("EFI/stela/choices.cfg")).unwrap();
    let install = choices
        .lines()
        .find(|line| line.starts_with(INSTALL_CHOICE))
        .unwrap();
    assert!(install.contains("--storage @disk --nor @disk"), "{install}");
    assert!(
        install.contains(&format!("--install {}", medium.firmware_manifest())),
        "{install}"
    );
    assert!(install.contains("--install-source"), "{install}");
    let mut joined = Vec::new();
    for index in 0.. {
        let piece = medium
            .directory()
            .join(stela_formats::payload::piece_name(index));
        let Ok(bytes) = fs::read(piece) else { break };
        joined.extend(bytes);
    }
    assert_eq!(joined, fs::read(&f.ipsw).unwrap());
    let options = fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert!(options.contains("--preboot @disk"), "{options}");
    // This computer never reaches the chosen disk's machine state.
    let error = rerun(&f, |c| c.action = Action::ResetNvram)
        .unwrap_err()
        .to_string();
    assert!(error.contains("chosen in stela.efi"), "{error}");
}

fn rerun(f: &Fixture, configure: impl Fn(&mut Request)) -> Result<String> {
    let mut command = command(f, &f.esp);
    command.ipsw = None;
    command.model = None;
    configure(&mut command);
    command.dispatch(&environment(&f.host))
}

#[test]
fn install_and_repair_unmount_target_filesystems() {
    let mut apfs = vec![0u8; 64];
    apfs[32..36].copy_from_slice(b"NXSB");
    let erased = |f: &Fixture| {
        fs::read(&f.host.partitions[REGION].1).unwrap()[..IMAGE_BYTES / 2]
            .iter()
            .all(|&byte| byte == 0)
    };
    // A mounted container is refused, and with --erase-target setup
    // unmounts and erases it, so the PC's OS stops mounting it.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &apfs, &[]);
    f.host.partitions.get_mut(REGION).unwrap().0.container = Some("disk9".into());
    let error = setup(&environment(&f.host), installed_command(&f))
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("--erase-target, which unmounts and erases"),
        "{error}"
    );
    let mut erase = installed_command(&f);
    erase.erase_target = true;
    setup(&environment(&f.host), erase).unwrap();
    assert_eq!(*f.host.unmounted.borrow(), [REGION]);
    assert!(erased(&f));
    // One an existing install's region gains is a problem `status`
    // names and `repair --erase-target` erases.
    let path = f.host.partitions[REGION].1.clone();
    let mut image = fs::read(&path).unwrap();
    image[..apfs.len()].copy_from_slice(&apfs);
    fs::write(&path, image).unwrap();
    let error = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(error.contains("holds another file system"), "{error}");
    let report = rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(report.contains("`stela repair --erase-target`"), "{report}");
    assert!(!erased(&f));
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.erase_target = true;
    })
    .unwrap();
    assert!(erased(&f));
}

#[test]
fn only_status_and_boot_log_leave_disks_unwritten() {
    for action in [Action::Status, Action::BootLog] {
        assert!(!writes_disks(&action));
    }
    for action in [
        Action::Create,
        Action::Repair,
        Action::UpdateStela,
        Action::UpdateOs,
        Action::Uninstall,
        Action::ResetNvram,
        Action::BackupState(PathBuf::new()),
        Action::RestoreState(PathBuf::new()),
        Action::Configure,
        Action::HostFirmwareInstall(PathBuf::new()),
    ] {
        assert!(writes_disks(&action));
    }
}

#[test]
fn the_last_word_on_the_network_follows_the_installer() {
    let run = |kind: InstallImage| {
        let mut f = fixture(64 * GIB, false);
        add_installed_disk(&mut f, &[], &[]);
        let mut command = installed_command(&f);
        command.install_image = Some(kind);
        create::network_next_step(&setup(&environment(&f.host), command).unwrap())
    };
    let network = run(InstallImage::Network);
    assert!(
        network.starts_with("Online: the install downloads macOS"),
        "{network}"
    );
    // These fake bundles hold no signing answers.
    let full = run(InstallImage::Full);
    assert!(
        full.starts_with("Offline: the install needs internet only"),
        "{full}"
    );
    let offline = format!("  note           {}\n", create::OFFLINE_NOTE);
    let offline = create::network_next_step(&offline);
    assert!(
        offline.starts_with("Offline: the install needs no internet."),
        "{offline}"
    );
}

#[test]
fn install_reads_full_or_streamed_image() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    // Network: the USB holds the inventory alone, and the Install choice
    // names no partition to read the IPSW from.
    let mut network = installed_command(&f);
    network.install_image = Some(InstallImage::Network);
    let report = setup(&environment(&f.host), network).unwrap();
    assert!(report.contains("installer      Online: "), "{report}");
    let remote = create::ONLINE_NOTE;
    assert!(report.contains(remote), "{report}");
    let written = record(&f);
    assert_eq!(written.install_image, Some(InstallImage::Network));
    let medium = written.active_medium().cloned().unwrap();
    assert!(medium.remote && medium.problems().is_empty(), "{medium:?}");
    let choices = fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap();
    let install = choices
        .lines()
        .find(|line| line.starts_with(INSTALL_CHOICE))
        .unwrap();
    assert!(
        install.contains(&format!("--install {}", medium.firmware_manifest())),
        "{install}"
    );
    assert!(!install.contains("--install-source"), "{install}");
    // Status names where a streamed image comes from, not an empty partition.
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("(macOS ")
            && status.contains(", Online, downloads during installation): intact"),
        "{status}"
    );
    assert!(status.contains(create::ONLINE_NOTE), "{status}");
    // Repair makes it full: the IPSW in pieces on the USB.
    let switch = |image| {
        rerun(&f, |c| {
            c.action = Action::Repair;
            c.ipsw = Some(f.ipsw.clone());
            c.install_image = Some(image);
        })
        .unwrap();
        record(&f).active_medium().cloned().unwrap()
    };
    let full = switch(InstallImage::Full);
    assert!(!full.remote && full.problems().is_empty(), "{full:?}");
    let first_piece = full.directory().join(stela_formats::payload::piece_name(0));
    assert!(first_piece.is_file());
    // Back to the network, the pieces go.
    let network = switch(InstallImage::Network);
    assert!(
        network.remote && network.problems().is_empty(),
        "{network:?}"
    );
    assert!(!first_piece.exists());
}

#[test]
fn full_install_requires_system_image() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let recovery = f._dir.path().join("Recovery.ipsw");
    restore::payload::trim(
        fs::File::open(&f.ipsw).unwrap(),
        restore::payload::SYNTHETIC_MODEL,
        &recovery,
        "Copying boot files",
    )
    .unwrap();
    assert!(restore::payload::holds_system_image(&f.ipsw).unwrap());
    assert!(!restore::payload::holds_system_image(&recovery).unwrap());
    let mut full = installed_command(&f);
    full.ipsw = Some(recovery.clone());
    full.install_image = Some(InstallImage::Full);
    full.dry_run = true;
    let error = format!("{:#}", setup(&environment(&f.host), full).unwrap_err());
    assert!(
        error.contains("Offline needs an IPSW file with all of macOS"),
        "{error}"
    );
    // A new setup without an image choice on this 128 GB USB, with
    // `--target` too, puts a whole image on it, and streams the install
    // from a recovery archive.
    for (ipsw, line) in [
        (recovery, "installer      Online: "),
        (f.ipsw.clone(), "installer      Offline: "),
    ] {
        let mut default = installed_command(&f);
        default.ipsw = Some(ipsw);
        default.install_image = None;
        default.dry_run = true;
        let report = setup(&environment(&f.host), default).unwrap();
        assert!(report.contains(line), "{report}");
    }
    let mut default = installed_command(&f);
    default.install_image = None;
    setup(&environment(&f.host), default).unwrap();
    let record = record(&f);
    assert_eq!(record.install_image, Some(InstallImage::Full));
    assert!(!record.active_medium().unwrap().remote);
}

#[test]
fn installed_disk_preserves_boot_choices_and_partitions() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let before = snapshot(&f.esp);
    let stela_before = snapshot(&stela_volume(&f));
    let disks = (partition_bytes(&f, REGION), partition_bytes(&f, NOR));

    let mut dry = installed_command(&f);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(plan.contains("installer      Offline: "), "{plan}");
    assert!(plan.contains(STELA_GUID), "{plan}");
    assert!(plan.contains("restore        "), "{plan}");
    assert_eq!(snapshot(&stela_volume(&f)), stela_before);
    assert!(
        plan.contains("Mac's SSD      disk1s2 (serial STELAPC01"),
        "{plan}"
    );
    assert!(plan.contains("disk1s2 is blank"), "{plan}");
    assert!(plan.contains("Dry run"), "{plan}");
    assert_eq!(snapshot(&f.esp), before);

    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let record = record(&f);
    let ecid = record.installs[0].ecid.unwrap();
    // The restore boots under the ticket Apple personalized for it.
    assert!(
        record
            .restore_bundle(Kind::Restore, Some(ecid))
            .is_some_and(|b| b.personalized)
    );
    let region = format!("{SERIAL}:{REGION_GUID}");
    let line = format!(
        "--system --storage {region} --nor {SERIAL}:{NOR_GUID} --preboot {region} \
         --ecid {ecid} --input"
    );
    assert_eq!(
        fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap(),
        format!("{line}\n")
    );
    // The startup choices hold the install and Install macOS, and open
    // on their own.
    let choices = Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    assert_eq!(choices.timeout_seconds, CHOOSER_SECONDS);
    assert_eq!(choices.entries.len(), 2);
    assert_eq!(choices.entries[0].name, "macOS");
    assert_eq!(choices.entries[0].options, line);
    // stela.efi routes a disk without an installed system to the disk's
    // System Recovery, and refuses one that holds none.
    let mut selection = stela_loader::boot_selection::select(
        &Variables::parse(&stela_soc_data::nor::nvram::empty_bank()).unwrap(),
    )
    .unwrap();
    selection.missing_boot_media();
    assert!(stela_loader::boot_selection::route(&line, Some(&selection)).is_err());
    selection.system_on_disk = true;
    let (routed, _) = stela_loader::boot_selection::route(&line, Some(&selection)).unwrap();
    assert!(routed.contains(&format!("--preboot {region}")), "{routed}");
    assert!(routed.ends_with("--system-recovery"), "{routed}");
    // Install macOS restores the install medium onto the disk
    // with the machine's restore bundle.
    let build = restore::payload::SYNTHETIC_BUILD;
    assert_eq!(choices.entries[1].name, INSTALL_CHOICE);
    assert_eq!(
        choices.entries[1].options,
        format!(
            "--system --storage {region} --nor {SERIAL}:{NOR_GUID} --bundle \
             \\stela\\boot\\{build}\\{} --ecid {ecid} \
             --install \\stela\\install\\{build}\\payload.json --install-source \
             {} --development-restore",
            bundle::leaf_of(Kind::Restore, Some(ecid), current()),
            STELA_GUID.to_ascii_uppercase()
        )
    );
    let restore_bundle = record.restore_bundle(Kind::Restore, Some(ecid)).unwrap();
    assert!(
        bundle::verify(
            &restore_bundle.directory(&record.volume),
            &restore_bundle.manifest_sha256
        )
        .is_empty()
    );
    // The medium is the IPSW in pieces on the USB's FAT32 partition.
    let medium = record.active_medium().cloned().unwrap();
    assert_eq!(
        (medium.volume.as_path(), medium.partition_guid.as_str()),
        (stela_volume(&f).as_path(), STELA_GUID)
    );
    assert!(medium.problems().is_empty());
    let mut joined = Vec::new();
    for index in 0.. {
        let piece = medium
            .directory()
            .join(stela_formats::payload::piece_name(index));
        let Ok(bytes) = fs::read(piece) else { break };
        joined.extend(bytes);
    }
    assert_eq!(joined, fs::read(&f.ipsw).unwrap());
    // Status finds a damaged piece by name.
    let first = medium
        .directory()
        .join(stela_formats::payload::piece_name(0));
    let piece = fs::read(&first).unwrap();
    fs::write(&first, &piece[1..]).unwrap();
    let status = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::Status).unwrap_err()
    );
    assert!(status.contains("ipsw.000 holds"), "{status}");
    fs::write(&first, &piece).unwrap();
    // --verify-on-read is the engine's, on the Install choice alone.
    let mut verify = record.clone();
    verify.verify_on_read = true;
    let line = verify.install_line(&verify.installs[0]).unwrap().unwrap();
    assert!(line.contains("--install-source"), "{line}");
    assert!(
        line.trim_end()
            .ends_with("--verify-on-read --development-restore"),
        "{line}"
    );
    // Setup itself writes neither partition.
    assert_eq!(
        (partition_bytes(&f, REGION), partition_bytes(&f, NOR)),
        disks
    );
    // The engine profile turns on for every choice, and off again.
    let on = rerun(&f, |c| {
        c.action = Action::Configure;
        c.profile = true;
    })
    .unwrap();
    assert!(on.contains("profile        on"), "{on}");
    let config = || fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert!(config().trim_end().ends_with("--profile"), "{}", config());
    let choices = Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    assert!(
        choices
            .entries
            .iter()
            .all(|e| e.options.ends_with("--profile"))
    );
    rerun(&f, |c| {
        c.action = Action::Configure;
        c.no_profile = true;
    })
    .unwrap();
    assert!(!config().contains("--profile"));

    // The next boot is chosen in NVRAM, which a dry run leaves alone.
    let dry = rerun(&f, |c| {
        c.action = Action::NextBoot(NextBoot::Recovery);
        c.dry_run = true;
    })
    .unwrap();
    assert!(
        dry.contains("one-time-boot-command=recover-system"),
        "{dry}"
    );
    assert_eq!(partition_bytes(&f, NOR), disks.1);
    rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Recovery)).unwrap();
    let variables = nvram(&f);
    assert_eq!(
        stela_loader::boot_selection::select(&variables)
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::SystemRecovery
    );
    rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Macos)).unwrap();
    assert_eq!(
        stela_loader::boot_selection::select(&nvram(&f))
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::MacOs { safe: false }
    );
    // A partition whose GUID is no longer the recorded one is not
    // written: its identifier now names another partition.
    let nor = partition_bytes(&f, NOR);
    f.host.partitions.get_mut(NOR).unwrap().0.guid = REGION2_GUID.into();
    let error = rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Recovery))
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("is not connected to this computer"),
        "{error}"
    );
    assert!(error.contains("run stela on the PC itself"), "{error}");
    assert_eq!(partition_bytes(&f, NOR), nor);
    f.host.partitions.get_mut(NOR).unwrap().0.guid = NOR_GUID.to_ascii_lowercase();

    // Uninstall restores the USB and keeps the installed disk and NVRAM.
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);
    assert_eq!(snapshot(&stela_volume(&f)), stela_before);
    assert!(!stela_volume(&f).join("stela").exists());
    assert_eq!(partition_bytes(&f, REGION), disks.0);
    assert_eq!(partition_bytes(&f, NOR), nor);
}

#[test]
fn installed_disks_that_would_lose_data_or_cannot_boot_are_refused() {
    let refusal = |f: &Fixture, configure: &dyn Fn(&mut Request)| {
        let mut command = installed_command(f);
        configure(&mut command);
        setup(&environment(&f.host), command)
            .unwrap_err()
            .to_string()
    };
    // An Apple region with another file system needs --erase-target.
    let mut apfs = vec![0u8; 64];
    apfs[32..36].copy_from_slice(b"NXSB");
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &apfs, &[]);
    assert!(refusal(&f, &|_| {}).contains("--erase-target"));
    let mut dry = installed_command(&f);
    dry.erase_target = true;
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(plan.contains("--erase-target erases it"), "{plan}");
    // A NOR partition with other bytes needs --initialize-nor.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], b"not a NOR image");
    assert!(refusal(&f, &|_| {}).contains("--initialize-nor"));
    let mut initialize = installed_command(&f);
    initialize.initialize_nor = true;
    setup(&environment(&f.host), initialize).unwrap();
    assert_eq!(
        stela_loader::boot_selection::select(&nvram(&f))
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::MacOs { safe: false }
    );
    // So does a SEP area after the NOR image that holds other bytes; the
    // flag blanks it and keeps the NOR image.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let capacity = stela_soc_data::nor::CAPACITY;
    let path = f.host.partitions[NOR].1.clone();
    let mut image = fs::read(&path).unwrap();
    image[capacity..capacity + 8].copy_from_slice(b"not SEP!");
    fs::write(&path, image).unwrap();
    assert!(refusal(&f, &|_| {}).contains("--initialize-nor"));
    let mut initialize = installed_command(&f);
    initialize.initialize_nor = true;
    setup(&environment(&f.host), initialize).unwrap();
    let bytes = partition_bytes(&f, NOR);
    assert!(bytes[capacity..].iter().all(|&b| b == 0));
    assert!(bytes[..capacity].iter().all(|&b| b == 0));
    // A guest disk is kept and booted.
    let mut gpt = vec![0u8; 520];
    gpt[512..520].copy_from_slice(b"EFI PART");
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &gpt, &[]);
    let mut dry = installed_command(&f);
    dry.dry_run = true;
    assert!(
        setup(&environment(&f.host), dry)
            .unwrap()
            .contains("already holds a macOS disk")
    );
    // Mounted or in-use partitions, the PC's limits and serials.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    f.host.partitions.get_mut(REGION).unwrap().0.container = Some("disk4".into());
    assert!(refusal(&f, &|_| {}).contains("APFS container disk4"));
    f.host.partitions.get_mut(REGION).unwrap().0.container = None;
    f.host.partitions.get_mut(NOR).unwrap().0.mount = Some("/Volumes/x".into());
    assert!(refusal(&f, &|_| {}).contains("is mounted"));
    f.host.partitions.get_mut(NOR).unwrap().0.mount = None;
    assert!(refusal(&f, &|c| c.nor = Some(REGION.into())).contains("name the same partition"));
    f.host.cpu.entropy = false;
    assert!(refusal(&f, &|_| {}).contains("RDSEED"));
    f.host.cpu.entropy = true;
    assert!(refusal(&f, &|c| c.disk_serial = Some("OTHER".into())).contains("reports serial"));
    f.host.serials.clear();
    assert!(refusal(&f, &|_| {}).contains("--disk-serial"));
    let mut given = installed_command(&f);
    given.disk_serial = Some(SERIAL.into());
    given.dry_run = true;
    setup(&environment(&f.host), given).unwrap();
    // Nothing was written by any refusal.
    assert!(!f.esp.join(RECORD).exists());
}

/// A SEP roots record, as the SEP model provisions one.
fn provision_sep(f: &Fixture, nor: &str) {
    let capacity = stela_soc_data::nor::CAPACITY;
    let path = f.host.partitions[nor].1.clone();
    let mut image = fs::read(&path).unwrap();
    let mut record = vec![0u8; 8 + 2 * 32];
    record[..8].copy_from_slice(b"STSEPR01");
    record[8..].fill(0x42);
    use sha2::{Digest, Sha256};
    let sum = Sha256::digest(&record);
    record.extend_from_slice(&sum);
    image[capacity..capacity + record.len()].copy_from_slice(&record);
    fs::write(&path, image).unwrap();
    assert_eq!(
        stela_soc::machine::sep_state::blank(
            &partition_bytes(f, nor)
                [capacity..capacity + stela_formats::machine_state::SEP_ROOTS_BYTES]
        ),
        Ok(false)
    );
}

#[test]
fn machine_state_of_an_earlier_machine_keeps_its_ecid() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    provision_sep(&f, NOR);
    let error = setup(&environment(&f.host), installed_command(&f))
        .unwrap_err()
        .to_string();
    assert!(error.contains("--ecid"), "{error}");
    let mut known = installed_command(&f);
    known.ecid = Some(Ecid(0x1234));
    setup(&environment(&f.host), known).unwrap();
    assert_eq!(record(&f).installs[0].ecid, Some(Ecid(0x1234)));
    assert!(bundle_of(&f, Some(Ecid(0x1234))).is_dir());
}

#[test]
fn additional_disk_gets_separate_machine_identity() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let record1 = record(&f);
    let first = record1.installs[0].ecid.unwrap();
    let kinds: Vec<Kind> = record1.bundles.iter().map(|b| b.kind).collect();
    assert_eq!(kinds, [Kind::Restore]);
    assert!(record1.active_medium().unwrap().problems().is_empty());
    let add = |target: &str, nor: &str, ipsw: bool, dry_run: bool| {
        rerun(&f, |c| {
            c.target = Some(target.into());
            c.nor = Some(nor.into());
            c.ipsw = ipsw.then(|| f.ipsw.clone());
            c.dry_run = dry_run;
        })
    };
    // The second is a new machine and needs its own bundle, prepared from
    // the USB's own copy of the image without --ipsw, or from --ipsw.
    let before = snapshot(&f.esp);
    let plan = add(REGION2, NOR2, false, true).unwrap();
    assert!(plan.contains("from this USB's copy of macOS"), "{plan}");
    assert_eq!(snapshot(&f.esp), before);
    assert!(add(REGION2, NOR2, true, true).unwrap().contains("Dry run"));
    assert_eq!(snapshot(&f.esp), before);
    add(REGION2, NOR2, true, false).unwrap();
    let record2 = record(&f);
    let second = record2.installs[1].ecid.unwrap();
    assert_ne!(second, first);
    assert_eq!(record2.installs[1].name, "macOS 2");
    assert_eq!(
        fs::read(bundle_of(&f, Some(second)).join("ramdisk.img")).unwrap(),
        vec![second.0 as u8 ^ 0xa5; 2048]
    );
    // The same partitions are not added twice.
    assert!(
        add(REGION2, NOR2, true, false)
            .unwrap_err()
            .to_string()
            .contains("already belongs")
    );
    let choices =
        || Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    assert_eq!(choices().timeout_seconds, CHOOSER_SECONDS);
    let names: Vec<String> = choices().entries.into_iter().map(|e| e.name).collect();
    assert_eq!(
        names,
        [
            "macOS",
            "macOS - Install macOS",
            "macOS 2",
            "macOS 2 - Install macOS"
        ]
    );
    // Each install restores with its own machine's restore bundle.
    let restore2 = bundle::directory_of(
        &record2.volume,
        bundle::Identity {
            kind: Kind::Restore,
            build: &record2.build,
            ecid: Some(second),
            generation: record2.generation(),
        },
    );
    assert!(choices().entries[3].options.contains(&format!(
        "--bundle \\stela\\boot\\{}\\restore-{}",
        record2.build,
        second.digits()
    )));
    // The default moves, and stela.cfg with it.
    rerun(&f, |c| {
        c.action = Action::Configure;
        c.default_install = Some("macOS 2".into());
    })
    .unwrap();
    assert_eq!(choices().entries[0].name, "macOS 2");
    assert!(
        fs::read_to_string(f.esp.join("EFI/stela/stela.cfg"))
            .unwrap()
            .contains(&format!("{SERIAL2}:{NOR2_GUID}"))
    );
    // Each install's next boot is its own NVRAM.
    rerun(&f, |c| {
        c.action = Action::NextBoot(NextBoot::Recovery);
        c.install = Some("macOS".into());
    })
    .unwrap();
    assert_eq!(
        stela_loader::boot_selection::select(&nvram_of(&f, NOR))
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::SystemRecovery
    );
    assert!(partition_bytes(&f, NOR2).iter().all(|&b| b == 0));
    // A model other than the setup's is refused for an install.
    let mut f2 = rerun(&f, |c| {
        c.target = Some(REGION2.into());
        c.nor = Some(NOR2.into());
        c.model = Some(PINNED_MODEL.into());
    })
    .unwrap_err()
    .to_string();
    assert!(f2.contains("same model"), "{f2}");
    // Removing an install keeps its partitions and drops its bundle.
    let nor2 = partition_bytes(&f, NOR2);
    f2 = rerun(&f, |c| {
        c.action = Action::Configure;
        c.remove_install = Some("macOS 2".into());
    })
    .unwrap();
    assert!(f2.contains(&format!("ECID {second}")), "{f2}");
    assert!(!bundle_of(&f, Some(second)).exists());
    assert!(!restore2.exists());
    assert_eq!(partition_bytes(&f, NOR2), nor2);
    // The list still opens on its own while the USB can install macOS.
    assert_eq!(choices().timeout_seconds, CHOOSER_SECONDS);
    assert_eq!(choices().entries[1].name, INSTALL_CHOICE);
    assert_eq!(record(&f).installs.len(), 1);
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert!(!f.esp.join("EFI/stela").exists());
}

#[test]
fn translation_store_defaults_to_the_firmware_pack_limit() {
    let f = fixture(64 * GIB, true);
    let (store, _) = translations::choose(&f.host, &[&f.data], |_| 0, None).unwrap();
    assert_eq!(store.unwrap().pack_bytes, GIB);

    let (store, _) = translations::choose(&f.host, &[&f.data], |_| 0, Some(2 * GIB)).unwrap();
    assert_eq!(store.unwrap().pack_bytes, 2 * GIB);
}

#[test]
fn same_revision_update_preserves_options() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let mut first = command(&f, &f.esp);
    first.translation_store_bytes = Some(translations::MIN_PACK_BYTES);
    setup(&environment(&f.host), first).unwrap();
    let store = record(&f).translations.unwrap();
    let spare = store.directory().join(translations::SPARE);
    fs::remove_file(&spare).unwrap();
    let pack = translations::tests::write_pack(
        &store.directory(),
        [4; 32],
        translations::tests::SMALL_PACK,
    );
    let pack_bytes = fs::read(&pack).unwrap();
    let options = f.esp.join(efi::OPTIONS);
    let custom = b"--system --watch 0x1000,0x10\n";
    fs::write(&options, custom).unwrap();
    let before_record = fs::read(f.esp.join(RECORD)).unwrap();
    let before_efi = fs::read(f.esp.join(efi::LOADER)).unwrap();
    let update = |dry_run| {
        rerun(&f, |c| {
            c.action = Action::UpdateStela;
            c.dry_run = dry_run;
        })
    };
    assert!(update(true).unwrap().contains("Dry run"));
    assert!(!spare.exists());
    assert_eq!(fs::read(&options).unwrap(), custom);
    assert!(update(false).unwrap().contains("prepare the cache"));
    assert_eq!(fs::metadata(&spare).unwrap().len(), store.pack_bytes);
    assert_eq!(fs::read(&pack).unwrap(), pack_bytes);
    assert_eq!(fs::read(&options).unwrap(), custom);
    assert_eq!(fs::read(f.esp.join(RECORD)).unwrap(), before_record);
    assert_eq!(fs::read(f.esp.join(efi::LOADER)).unwrap(), before_efi);
    // Retry after a partial allocation repairs the spare without an update.
    fs::write(&spare, [0; 64]).unwrap();
    fs::remove_file(store.directory().join(translations::CONFIG)).unwrap();
    update(false).unwrap();
    assert!(store.config_intact() && store.spare_ready());
    assert_eq!(fs::read(&pack).unwrap(), pack_bytes);
    assert_eq!(fs::read(&options).unwrap(), custom);
    assert_eq!(fs::read(f.esp.join(RECORD)).unwrap(), before_record);
    assert!(update(false).unwrap().contains("already installed"));
}

#[test]
fn status_notes_the_spare_the_first_boot_took_and_succeeds() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let store = record(&f).translations.unwrap();
    let spare = store.directory().join(translations::SPARE);
    // The first boot renames the spare into its translator's pack.
    let pack = store.directory().join(translations::pack_name(&[7; 32]));
    fs::rename(&spare, &pack).unwrap();
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
    let note = status
        .lines()
        .find(|line| line.contains(&store.spare_repair()))
        .unwrap();
    assert!(note.trim_start().starts_with("note"), "{status}");
    assert!(note.contains("`stela repair`"), "{status}");
    // Repair prepares the next spare and keeps the pack.
    rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(store.spare_ready() && pack.is_file());
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(!status.contains(&store.spare_repair()), "{status}");
    // A spare missing beside no pack, or a short one, is still a problem.
    fs::remove_file(&pack).unwrap();
    fs::remove_file(&spare).unwrap();
    let error = rerun(&f, |c| c.action = Action::Status).unwrap_err();
    assert!(format!("{error:#}").contains("1 problem"), "{error:#}");
    fs::write(&pack, [0; 64]).unwrap();
    fs::write(&spare, [0; 64]).unwrap();
    assert!(rerun(&f, |c| c.action = Action::Status).is_err());
}

#[test]
fn status_notes_a_stale_chosen_disk_without_a_problem() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    assert!(record(&f).installs[0].disk.is_chosen());
    const GUID: &str = "5e1a0000-0000-4000-8000-0000000000c1";
    f.host.serials.insert("disk0".into(), "HOSTDISK".into());
    f.host.partitions.insert(
        "disk0s2".into(),
        (
            PartitionInfo {
                identifier: "disk0s2".into(),
                disk: "disk0".into(),
                guid: GUID.into(),
                bytes: 100 * GIB,
                content: "Apple_APFS".into(),
                mount: None,
                container: None,
            },
            f._dir.path().join("disk0s2.img"),
        ),
    );
    let disk_cfg = f.esp.join("EFI/stela/disk.cfg");
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(!status.contains("chosen disk"), "{status}");
    for (chosen, words) in [
        (
            "GONE:".to_owned() + GUID,
            "no disk on this PC has the chosen serial",
        ),
        (
            "HOSTDISK:5E1A0000-0000-4000-8000-0000000000c2".into(),
            "no longer has the chosen partition",
        ),
    ] {
        fs::write(&disk_cfg, format!("{chosen}\n")).unwrap();
        let status = rerun(&f, |c| c.action = Action::Status).unwrap();
        assert!(status.contains("No problems found."), "{status}");
        let note = status.lines().find(|line| line.contains(words)).unwrap();
        assert!(note.trim_start().starts_with("note"), "{status}");
        assert!(
            note.contains("Install macOS will ask for a disk again"),
            "{status}"
        );
    }
    fs::write(
        &disk_cfg,
        format!("HOSTDISK:{}\n", GUID.to_ascii_uppercase()),
    )
    .unwrap();
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("the chosen disk is in this computer"),
        "{status}"
    );
    assert!(!status.contains("will ask for a disk again"), "{status}");
}

#[test]
fn cache_spare_failure_blocks_activation_until_retry() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let mut first = command(&f, &f.esp);
    first.translation_store_bytes = Some(translations::MIN_PACK_BYTES);
    setup(&environment(&f.host), first).unwrap();
    let store = record(&f).translations.unwrap();
    let spare = store.directory().join(translations::SPARE);
    let pack = translations::tests::write_pack(
        &store.directory(),
        [4; 32],
        translations::tests::SMALL_PACK,
    );
    let pack_bytes = fs::read(&pack).unwrap();
    let active = f.esp.join(efi::LOADER);
    let old_efi = fs::read(&active).unwrap();
    let config = fs::read(f.esp.join(efi::OPTIONS)).unwrap();
    let new = f._dir.path().join("new.efi");
    let new_efi = efi_image(b"stela-2");
    fs::write(&new, &new_efi).unwrap();
    let update = || {
        rerun(&f, |c| {
            c.action = Action::UpdateStela;
            c.efi = Some(new.clone());
        })
    };
    fs::remove_file(&spare).unwrap();
    fs::create_dir(&spare).unwrap();
    assert!(update().unwrap_err().to_string().contains("cannot create"));
    assert_eq!(fs::read(&active).unwrap(), old_efi);
    assert_eq!(fs::read(f.esp.join(efi::OPTIONS)).unwrap(), config);
    assert_eq!(fs::read(&pack).unwrap(), pack_bytes);
    fs::remove_dir(&spare).unwrap();
    update().unwrap();
    assert_eq!(fs::read(&active).unwrap(), new_efi);
    assert!(store.spare_ready());
    assert_eq!(fs::read(&pack).unwrap(), pack_bytes);
}

#[test]
fn stela_update_with_disabled_cache_creates_no_store() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let mut first = command(&f, &f.esp);
    first.translation_store_bytes = Some(0);
    setup(&environment(&f.host), first).unwrap();
    assert!(record(&f).translations.is_none());
    let new = f._dir.path().join("new.efi");
    fs::write(&new, efi_image(b"stela-2")).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(new.clone());
    })
    .unwrap();
    assert!(!f.esp.join(translations::DIRECTORY).exists());
    assert!(record(&f).translations.is_none());
    assert_eq!(
        fs::read(f.esp.join(efi::LOADER)).unwrap(),
        efi_image(b"stela-2")
    );
}

#[test]
fn stela_update_replaces_the_loader_and_keeps_the_newest_pack() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let before = snapshot(&f.esp);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let tool = f.esp.join(efi::LOADER);
    let config = fs::read(f.esp.join(efi::OPTIONS)).unwrap();
    let new = f._dir.path().join("new.efi");
    fs::write(&new, efi_image(b"stela-2")).unwrap();
    let update = |dry_run: bool| {
        rerun(&f, |c| {
            c.action = Action::UpdateStela;
            c.efi = Some(new.clone());
            c.dry_run = dry_run;
        })
    };
    assert!(update(true).unwrap().contains("Dry run"));
    assert_eq!(fs::read(&tool).unwrap(), efi_image(b"stela"));
    // Packs of four translators and a damaged one: the update keeps the
    // newest intact one.
    let packs = record(&f).translations.unwrap().directory();
    assert!(packs.join(translations::CONFIG).is_file());
    let written: Vec<PathBuf> = (0..4u8)
        .map(|n| {
            let path =
                translations::tests::write_pack(&packs, [n; 32], translations::tests::SMALL_PACK);
            fs::File::options()
                .write(true)
                .open(&path)
                .unwrap()
                .set_modified(
                    std::time::SystemTime::UNIX_EPOCH
                        + std::time::Duration::from_secs(1000 + u64::from(n)),
                )
                .unwrap();
            path
        })
        .collect();
    fs::write(packs.join(translations::pack_name(&[0xab; 32])), b"STLT").unwrap();
    let report = update(false).unwrap();
    assert!(
        report.contains("new pack for the new stela.efi"),
        "{report}"
    );
    assert!(
        report.contains(&written[0].display().to_string()),
        "{report}"
    );
    assert!(written[..3].iter().all(|path| !path.exists()) && written[3].exists());
    assert_eq!(
        fs::read_dir(&packs).unwrap().count(),
        4,
        "store.cfg, one pack, the spare and the pipeline cache"
    );
    assert!(packs.join(translations::SPARE).exists());
    assert!(packs.join(translations::PIPELINES).exists());
    assert_eq!(fs::read(&tool).unwrap(), efi_image(b"stela-2"));
    // Only the loader changed: stela.cfg stays.
    assert_eq!(fs::read(f.esp.join(efi::OPTIONS)).unwrap(), config);
    assert!(update(false).unwrap().contains("already installed"));
    // A stela.efi of another interface is refused.
    fs::write(&new, b"MZ old build").unwrap();
    assert!(
        update(false)
            .unwrap_err()
            .to_string()
            .contains("not the stela.efi")
    );
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);
}

#[test]
fn schema_update_rebuilds_bundles() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let next_schema = record(&f).bundle_schema.checked_add(1).unwrap();
    let ecid = record(&f).installs[0].ecid;
    let old = bundle_of(&f, ecid);
    // The Install macOS choice names the restore bundle.
    let options = || fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap();
    // A later stela and stela.efi read the next schema.
    let newer = mount_interface::Interface {
        bundle_schema: next_schema,
        ..mount_interface::CURRENT
    };
    let next_generation = bundle::Generation {
        schema: next_schema,
        ..current()
    };
    let prepare = move |options: prepare::RamdiskOptions<'_>| {
        let prepare::RamdiskOptions { ecid, output, .. } = options;
        bundle::tests::write(
            output,
            next_generation,
            &[ecid.map_or(0, |e| e.0 as u8) ^ 0xff; 64],
        )
    };
    let environment = Environment {
        prepare_restore: &prepare,
        interface: newer,
        ..environment(&f.host)
    };
    let new = f._dir.path().join("new.efi");
    let mut image = b"MZstela-3".to_vec();
    image.extend_from_slice(&mount_interface::marker(newer));
    fs::write(&new, image).unwrap();
    // The USB holds the image, which the update reads without --ipsw.
    let mut command = command(&f, &f.esp);
    command.ipsw = None;
    command.model = None;
    command.action = Action::UpdateStela;
    command.efi = Some(new.clone());
    command.dispatch(&environment).unwrap();
    let record2 = record(&f);
    assert_eq!(record2.bundle_schema, next_schema);
    let migrated = bundle_of(&f, ecid);
    let leaf = bundle::leaf_of(Kind::Restore, ecid, next_generation);
    assert!(
        migrated.ends_with(&leaf) && leaf.contains(&format!("-s{next_schema}")),
        "{leaf}"
    );
    assert!(
        options()
            .split_ascii_whitespace()
            .any(|word| word.ends_with(&leaf)),
        "{}",
        options()
    );
    assert!(!old.exists());
}

#[test]
fn revision_update_rebuilds_bundles() {
    // A setup with an install, made before setups recorded revisions.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let (older, newer) = (preparers(0), preparers(1));
    let old = environment_at(&f.host, &older, 0);
    setup(&old, installed_command(&f)).unwrap();
    let path = f.esp.join(RECORD);
    let mut json: serde_json::Value = stela_engine::read_json(&path).unwrap();
    json.as_object_mut().unwrap().remove("bundle_revision");
    for kept in json["bundles"].as_array_mut().unwrap() {
        kept.as_object_mut().unwrap().remove("revision");
    }
    stela_engine::write_json(&path, &json).unwrap();
    let first = record(&f);
    assert_eq!(first.bundle_revision, 0);
    let ecid = first.installs[0].ecid;
    let choices = || fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap();
    let directories = |revision: u32| -> Vec<PathBuf> {
        let generation = bundle::Generation {
            revision,
            ..current()
        };
        vec![bundle::directory_of(
            &first.volume,
            bundle::Identity {
                kind: Kind::Restore,
                build: &first.build,
                ecid,
                generation,
            },
        )]
    };
    // A later stela prepares revision 1, with the same stela.efi.
    let new = environment_at(&f.host, &newer, 1);
    let cache = f._dir.path().join("cache");
    let run = |configure: &dyn Fn(&mut Request)| {
        let mut command = command(&f, &f.esp);
        command.ipsw = None;
        command.model = None;
        command.download_dir = Some(cache.clone());
        configure(&mut command);
        command.dispatch(&new)
    };
    let status = run(&|c| c.action = Action::Status).unwrap_err().to_string();
    assert!(
        status.contains("boot bundles are revision 0, and this stela prepares revision 1"),
        "{status}"
    );
    assert!(!status.contains("--fetch"), "{status}");
    assert!(status.contains("`stela update`"), "{status}");
    // Other changes that prepare bundles wait for the update.
    let repair = run(&|c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    });
    assert!(
        repair.unwrap_err().to_string().contains("`stela update`"),
        "the repair is refused"
    );
    let update = |ipsw: bool, dry_run: bool| {
        run(&|c| {
            c.action = Action::UpdateStela;
            c.ipsw = ipsw.then(|| f.ipsw.clone());
            c.dry_run = dry_run;
        })
    };
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    // With --ipsw, a dry run prepares from it; without, it names the USB's
    // own copy of the image, which the update reads; neither downloads
    // (this environment's downloads panic) nor changes anything.
    let plan = update(true, true).unwrap();
    assert!(!plan.contains("is already installed"), "{plan}");
    assert!(plan.contains(&f.ipsw.display().to_string()), "{plan}");
    for directory in directories(1) {
        assert!(plan.contains(&directory.display().to_string()), "{plan}");
    }
    let plan = update(false, true).unwrap();
    assert!(plan.contains("from this USB's copy of macOS"), "{plan}");
    assert!(plan.contains("nothing is downloaded"), "{plan}");
    for directory in directories(1) {
        assert!(plan.contains(&directory.display().to_string()), "{plan}");
    }
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // A copy that does not read back as written is refused before the USB
    // changes.
    let piece = medium::directory(&first.volume, &first.build).join("ipsw.000");
    let good = fs::read(&piece).unwrap();
    let mut bad = good.clone();
    bad[0] ^= 1;
    fs::write(&piece, &bad).unwrap();
    let before_bad = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let error = format!("{:#}", update(false, false).unwrap_err());
    assert!(error.contains("reading "), "{error}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before_bad);
    fs::write(&piece, &good).unwrap();
    // The update itself needs no image and no network: the stick holds
    // the image. What it read from the USB is not kept.
    update(false, false).unwrap();
    assert!(fs::read_dir(&cache).unwrap().next().is_none());
    let updated = record(&f);
    assert_eq!(updated.bundle_revision, 1);
    assert_eq!(updated.stela_version, first.stela_version);
    for (old, new) in directories(0).iter().zip(directories(1)) {
        assert_eq!(bundle::generation(&new).unwrap().revision, 1);
        assert!(!old.exists());
    }
    let [restore] = &directories(1)[..] else {
        unreachable!()
    };
    let leaf = restore.file_name().unwrap().to_string_lossy().into_owned();
    assert!(choices().contains(&leaf), "{}", choices());
    assert!(
        run(&|c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
    // Bundles of the current revision are left alone: a new stela.efi
    // needs no restore image, and the same one is no update.
    assert!(update(false, false).unwrap().contains("already installed"));
    let new_efi = f._dir.path().join("new.efi");
    fs::write(&new_efi, efi_image(b"stela-2")).unwrap();
    run(&|c| {
        c.action = Action::UpdateStela;
        c.efi = Some(new_efi.clone());
    })
    .unwrap();
    assert_eq!(record(&f).bundles, updated.bundles);
}

/// A setup at revision 0 whose install streams the image from Apple's
/// servers, so the USB holds no copy of it, and the environment of a stela
/// that prepares revision 1.
fn network_setup(f: &mut Fixture, older: &[Box<Prepare>; 2]) {
    add_stela(f);
    let mut network = command(f, &stela_volume(f));
    network.install_image = Some(InstallImage::Network);
    setup(&environment_at(&f.host, older, 0), network).unwrap();
    assert!(record(f).active_medium().unwrap().remote);
}

/// Remove the boot files a network setup keeps on its USB.
fn drop_inputs(f: &Fixture) {
    let record = record(f);
    fs::remove_file(inputs::path(&record.volume, &record.build)).unwrap();
}

#[test]
fn revision_update_of_a_network_setup_prepares_from_the_boot_files_it_keeps() {
    let mut f = fixture(64 * GIB, false);
    let (older, newer) = (preparers(0), preparers(1));
    network_setup(&mut f, &older);
    let kept = {
        let record = record(&f);
        inputs::path(&record.volume, &record.build)
    };
    assert!(kept.is_file(), "the setup keeps the boot files");
    // No image, no cache, no network (downloads panic here).
    let environment = environment_at(&f.host, &newer, 1);
    let cache = f._dir.path().join("cache");
    let mut dry = update_request(&f, &cache);
    dry.dry_run = true;
    let plan = dry.dispatch(&environment).unwrap();
    assert!(plan.contains("kept on this USB"), "{plan}");
    update_request(&f, &cache).dispatch(&environment).unwrap();
    assert_eq!(record(&f).bundle_revision, 1);
    assert!(kept.is_file());
}

fn update_request(f: &Fixture, cache: &Path) -> Request {
    let mut command = command(f, &stela_volume(f));
    command.action = Action::UpdateStela;
    command.ipsw = None;
    command.model = None;
    command.download_dir = Some(cache.to_path_buf());
    // Another build than the setup's: every maintenance command works on
    // the setup's own build.
    command.build = "STELA2".into();
    command
}

#[test]
fn revision_update_without_any_image_fails_before_changing_anything() {
    let mut f = fixture(64 * GIB, false);
    let (older, newer) = (preparers(0), preparers(1));
    network_setup(&mut f, &older);
    drop_inputs(&f);
    let cache = f._dir.path().join("cache");
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    // No image, no cache, no network: refused with what to do, the USB
    // as it was.
    let error = update_request(&f, &cache)
        .dispatch(&environment_at(&f.host, &newer, 1))
        .unwrap_err()
        .to_string();
    assert!(
        error.contains(&format!("schema {} revision 1", current().schema)),
        "{error}"
    );
    assert!(error.contains("--ipsw FILE"), "{error}");
    assert!(error.contains("streams it from Apple"), "{error}");
    assert!(error.contains(&cache.display().to_string()), "{error}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
}

#[test]
fn revision_update_uses_a_downloaded_image_in_any_cache() {
    let mut f = fixture(64 * GIB, false);
    let (older, newer) = (preparers(0), preparers(1));
    network_setup(&mut f, &older);
    drop_inputs(&f);
    // The boot archive an earlier run downloaded, in a cache other than
    // --download-dir's, as root's is to a user's run.
    let other = f._dir.path().join("root-cache");
    fs::create_dir_all(&other).unwrap();
    let archive = other.join(format!(
        "Recovery_{}_{}.ipsw",
        restore::payload::SYNTHETIC_BUILD,
        restore::payload::SYNTHETIC_MODEL.replace(',', "_")
    ));
    restore::payload::trim(
        fs::File::open(&f.ipsw).unwrap(),
        restore::payload::SYNTHETIC_MODEL,
        &archive,
        "Copying boot files",
    )
    .unwrap();
    let caches = [other.clone()];
    let environment = Environment {
        image_caches: &caches,
        ..environment_at(&f.host, &newer, 1)
    };
    let cache = f._dir.path().join("cache");
    let mut dry = update_request(&f, &cache);
    dry.dry_run = true;
    let plan = dry.dispatch(&environment).unwrap();
    assert!(plan.contains(&archive.display().to_string()), "{plan}");
    update_request(&f, &cache).dispatch(&environment).unwrap();
    assert_eq!(record(&f).bundle_revision, 1);
    assert!(archive.is_file());
}

#[test]
fn revision_update_downloads_only_the_boot_files_saying_their_size() {
    let mut f = fixture(64 * GIB, false);
    let (older, newer) = (preparers(0), preparers(1));
    network_setup(&mut f, &older);
    drop_inputs(&f);
    let ipsw = f.ipsw.clone();
    let fetched = std::cell::Cell::new(0);
    let fetch = |directory: &Path,
                 build: &str,
                 model: &str,
                 need: sources::Need|
     -> Result<sources::Fetched> {
        fetched.set(fetched.get() + 1);
        assert_eq!(
            (build, need),
            (restore::payload::SYNTHETIC_BUILD, sources::Need::BootFiles)
        );
        fs::create_dir_all(directory)?;
        let path = directory.join(prepare::recovery_name(build, model));
        restore::payload::trim(fs::File::open(&ipsw)?, model, &path, "Copying boot files")?;
        Ok(sources::Fetched {
            path,
            download: None,
        })
    };
    let size = |build: &str, _: &str| -> Result<(u64, u64)> {
        assert_eq!(build, restore::payload::SYNTHETIC_BUILD);
        Ok((2_200_000_000, 27_000_000_000))
    };
    let environment = Environment {
        image_size: &size,
        fetch_image: &fetch,
        ..environment_at(&f.host, &newer, 1)
    };
    let cache = f._dir.path().join("cache");
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let mut dry = update_request(&f, &cache);
    dry.dry_run = true;
    let plan = dry.dispatch(&environment).unwrap();
    assert!(plan.contains("download 2.2 GB of the macOS"), "{plan}");
    assert!(plan.contains("27.0 GB"), "{plan}");
    assert!(plan.contains("--ipsw FILE"), "{plan}");
    assert_eq!(fetched.get(), 0, "a dry run downloads nothing");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    update_request(&f, &cache).dispatch(&environment).unwrap();
    assert_eq!(fetched.get(), 1);
    assert_eq!(record(&f).bundle_revision, 1);
    // The next update finds that download and fetches nothing.
    let newest = preparers(2);
    let environment = Environment {
        image_size: &size,
        fetch_image: &fetch,
        ..environment_at(&f.host, &newest, 2)
    };
    update_request(&f, &cache).dispatch(&environment).unwrap();
    assert_eq!((fetched.get(), record(&f).bundle_revision), (1, 2));
}

#[test]
fn repair_puts_a_damaged_bundle_back_from_the_usbs_own_image() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let ecid = record(&f).installs[0].ecid;
    let ramdisk = bundle_of(&f, ecid).join("ramdisk.img");
    fs::write(&ramdisk, b"damaged").unwrap();
    // Status names the command that works: repair, no flag.
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(problems.contains("`stela repair`"), "{problems}");
    assert!(!problems.contains("--fetch"), "{problems}");
    // No image, no network: the USB holds the image.
    let plan = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(plan.contains("from this USB's copy of macOS"), "{plan}");
    rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(
        rerun(&f, |c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
}

/// The boot files read from the USB's full install medium are the members
/// a download fetches, not the recovery set.
#[test]
fn boot_files_read_from_the_usb_are_the_members_a_download_fetches() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let record = record(&f);
    let medium = record.active_medium().cloned().unwrap();
    let lookup = sources::Lookup {
        need: sources::Need::BootFiles,
        build: &record.build,
        model: &record.model,
        ipsw: None,
        download_dir: Some(&f._dir.path().join("cache")),
        medium: Some(&medium),
    };
    let environment = environment(&f.host);
    let source = sources::Source::Medium(medium.clone());
    let obtained = sources::obtain(&environment, &source, &lookup).unwrap();
    let archive = zip::ZipArchive::new(fs::File::open(&obtained.path).unwrap()).unwrap();
    assert_eq!(archive.comment(), restore::payload::INPUTS_COMMENT);
    let mut held: Vec<String> = archive.file_names().map(str::to_owned).collect();
    held.sort_unstable();
    let mut image = fs::File::open(&f.ipsw).unwrap();
    let mut whole = zip::ZipArchive::new(&mut image as &mut dyn restore::ImageSource).unwrap();
    assert_eq!(held, fake_bundle_inputs(&mut whole, &record.model).unwrap());
    assert!(!restore::payload::holds_system_image(&obtained.path).unwrap());
}

#[test]
fn status_advises_update_stela_while_bundles_are_of_another_revision() {
    let mut f = fixture(64 * GIB, false);
    let (older, newer) = (preparers(0), preparers(1));
    network_setup(&mut f, &older);
    let ecid = record(&f).installs[0].ecid;
    fs::write(bundle_of(&f, ecid).join("ramdisk.img"), b"damaged").unwrap();
    let environment = environment_at(&f.host, &newer, 1);
    let mut status = update_request(&f, &f._dir.path().join("cache"));
    status.action = Action::Status;
    let problems = status.dispatch(&environment).unwrap_err().to_string();
    assert!(problems.contains("`stela update`"), "{problems}");
    assert!(!problems.contains("`stela repair`"), "{problems}");
    // Repair refuses before downloading anything (downloads panic here).
    let mut repair = update_request(&f, &f._dir.path().join("cache"));
    repair.action = Action::Repair;
    let error = repair.dispatch(&environment).unwrap_err().to_string();
    assert!(error.contains("`stela update`"), "{error}");
}

#[test]
fn os_update_checks_everything_before_any_download() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let os = |build: &str, environment: &Environment<'_>| {
        let mut request = update_request(&f, &f._dir.path().join("cache"));
        request.action = Action::UpdateOs;
        request.build = build.into();
        request.dispatch(environment)
    };
    // The setup's own build: nothing to do, and no download (they panic).
    let done = os(restore::payload::SYNTHETIC_BUILD, &environment(&f.host)).unwrap();
    assert!(done.contains("already the build"), "{done}");
    assert!(done.contains("nothing to update"), "{done}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // Another build, offline and nowhere on this computer: refused with
    // what to do.
    let error = os("STELA2", &environment(&f.host)).unwrap_err().to_string();
    assert!(error.contains("--ipsw FILE"), "{error}");
    assert!(error.contains("the whole image"), "{error}");
    // A whole image larger than the USB's free space: refused before the
    // download, with the sizes.
    let huge = |_: &str, _: &str| -> Result<(u64, u64)> { Ok((GIB, 1000 * GIB)) };
    let environment = Environment {
        image_size: &huge,
        ..environment(&f.host)
    };
    let error = format!("{:#}", os("STELA2", &environment).unwrap_err());
    assert!(error.contains("GB free"), "{error}");
    assert!(error.contains("would not make enough room"), "{error}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
}

#[test]
fn a_new_setup_takes_an_image_this_computer_holds_instead_of_downloading() {
    for installer in [InstallImage::Full, InstallImage::Network] {
        let mut f = fixture(64 * GIB, false);
        add_stela(&mut f);
        let cache = f._dir.path().join("cache");
        fs::create_dir_all(&cache).unwrap();
        let cached = cache.join(format!(
            "UniversalMac_0_{}_Restore.ipsw",
            restore::payload::SYNTHETIC_BUILD
        ));
        fs::copy(&f.ipsw, &cached).unwrap();
        let mut request = command(&f, &f.esp);
        request.ipsw = None;
        request.fetch = true;
        request.download_dir = Some(cache.clone());
        request.install_image = Some(installer);
        let text = setup(&environment(&f.host), request).unwrap();
        assert!(
            text.contains(&format!("restore image  {}", cached.display())),
            "{text}"
        );
        assert_eq!(record(&f).install_kind(), installer);
    }
}

#[test]
fn a_piece_an_install_found_damaged_is_named_by_status_and_put_right_by_repair() {
    use stela_formats::payload::pieces::DAMAGED_LOG_LINE;
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let medium = record(&f).active_medium().cloned().unwrap();
    let first = medium
        .directory()
        .join(stela_formats::payload::piece_name(0));
    let good = fs::read(&first).unwrap();
    // One byte changed, the size kept: only reading the piece finds it.
    let mut bad = good.clone();
    bad[7] ^= 1;
    fs::write(&first, &bad).unwrap();
    let status = || rerun(&f, |c| c.action = Action::Status);
    let clean = status().unwrap();
    assert!(
        clean.contains("every block is checked as the install reads it"),
        "{clean}"
    );
    // Without the image or the network, repair refuses before it changes
    // anything: the damaged piece stays rather than going missing.
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let offline = || {
        let mut request = update_request(&f, &f._dir.path().join("empty"));
        request.action = Action::Repair;
        request.full = true;
        request
    };
    let error = format!(
        "{:#}",
        offline().dispatch(&environment(&f.host)).unwrap_err()
    );
    assert!(error.contains("needs the macOS"), "{error}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // With Apple's servers reachable, the dry run names the download of
    // that piece alone.
    let whole = fs::metadata(&f.ipsw).unwrap().len();
    let size = |_: &str, _: &str| -> Result<(u64, u64)> { Ok((2_200_000_000, whole)) };
    let mut request = offline();
    request.dry_run = true;
    let plan = request
        .dispatch(&Environment {
            image_size: &size,
            ..environment(&f.host)
        })
        .unwrap();
    let piece = host::bytes_text(good.len() as u64);
    assert!(
        plan.contains(&format!("download {piece} from Apple")),
        "{plan}"
    );
    assert!(plan.contains("this USB's copy lacks"), "{plan}");
    // The image, downloaded earlier.
    let cache = f._dir.path().join("cache");
    fs::create_dir_all(&cache).unwrap();
    fs::copy(
        &f.ipsw,
        cache.join(format!(
            "UniversalMac_0_{}_Restore.ipsw",
            restore::payload::SYNTHETIC_BUILD
        )),
    )
    .unwrap();
    let repair = |verify: bool, dry_run: bool| {
        let mut request = update_request(&f, &cache);
        request.action = Action::Repair;
        request.full = verify;
        request.dry_run = dry_run;
        request.dispatch(&environment(&f.host)).unwrap()
    };
    // Asked to, repair reads every piece, and names what it finds.
    let plan = repair(true, true);
    assert!(
        plan.contains("ipsw.000 damaged: would be removed"),
        "{plan}"
    );
    assert!(plan.contains(&format!("({piece} to write)")), "{plan}");
    assert_eq!(fs::read(&first).unwrap(), bad);
    // An install that found the block: status names the piece and repair.
    let line = format!("[   12.000] {DAMAGED_LOG_LINE}ipsw.000 block 0\n");
    fs::write(
        f.esp.join(efi::BOOT_LOG),
        crate::boot_log::tests::region(&[&line]),
    )
    .unwrap();
    let problems = format!("{:#}", status().unwrap_err());
    assert!(
        problems.contains("ipsw.000 failed its block check"),
        "{problems}"
    );
    assert!(problems.contains("`stela repair`"), "{problems}");
    // Repair finds it by itself, and writes it again from the image.
    let done = repair(false, false);
    assert!(done.contains("ipsw.000 damaged: removed"), "{done}");
    assert_eq!(fs::read(&first).unwrap(), good);
    // The log still names it; it is put right since.
    assert!(status().unwrap().contains("No problems found."));
    let again = repair(true, false);
    assert!(
        again.contains("every piece read and checked: intact"),
        "{again}"
    );
    // A later install that finds a damaged block is named again.
    fs::write(
        f.esp.join(efi::BOOT_LOG),
        crate::boot_log::tests::region(&[&line, &line]),
    )
    .unwrap();
    let problems = format!("{:#}", status().unwrap_err());
    assert!(
        problems.contains("ipsw.000 failed its block check"),
        "{problems}"
    );
}

#[test]
fn repair_of_a_short_piece_downloads_that_piece_alone_saying_so() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let medium = record(&f).active_medium().cloned().unwrap();
    let first = medium
        .directory()
        .join(stela_formats::payload::piece_name(0));
    let piece = fs::read(&first).unwrap();
    fs::write(&first, &piece[1..]).unwrap();
    let whole = fs::metadata(&f.ipsw).unwrap().len();
    let boot = 2_200_000_000;
    let size = |_: &str, _: &str| -> Result<(u64, u64)> { Ok((boot, whole)) };
    // No bundle needs preparing: the download is asked for the pieces
    // alone, never the boot files.
    let fetch = |_: &Path, _: &str, _: &str, need: sources::Need| -> Result<sources::Fetched> {
        anyhow::bail!("asked for {need:?}")
    };
    let environment = Environment {
        image_size: &size,
        fetch_image: &fetch,
        ..environment(&f.host)
    };
    let repair = |ipsw: Option<&Path>, dry_run: bool| {
        let mut request = update_request(&f, &f._dir.path().join("cache"));
        request.action = Action::Repair;
        request.dry_run = dry_run;
        request.ipsw = ipsw.map(Path::to_path_buf);
        request.dispatch(&environment)
    };
    let plan = repair(None, true).unwrap();
    let total = host::bytes_text(piece.len() as u64);
    assert!(
        plan.contains(&format!("download {total} from Apple")),
        "{plan}"
    );
    assert!(plan.contains("this USB's copy lacks"), "{plan}");
    assert!(plan.contains("checked and kept"), "{plan}");
    assert!(!plan.contains("boot files"), "{plan}");
    let error = repair(None, false).unwrap_err().to_string();
    assert!(error.contains("asked for Pieces"), "{error}");
    // From an image on this computer it writes that piece alone.
    let plan = repair(Some(&f.ipsw), true).unwrap();
    let piece_text = host::bytes_text(piece.len() as u64);
    assert!(plan.contains(&format!("({piece_text} to write)")), "{plan}");
}

/// A full-installer setup whose USB has room for one copy of macOS only,
/// and the newer build's image.
fn one_copy_setup() -> (Fixture, PathBuf, u64) {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    f.host.volumes.get_mut(&stela_volume(&f)).unwrap().1 = MARGIN + 16 * 1024;
    let old = record(&f).active_medium().unwrap().bytes_on_volume();
    (f, newer, old)
}

fn os_update(f: &Fixture, ipsw: &Path) -> Request {
    let mut request = update_request(f, &f._dir.path().join("cache"));
    request.action = Action::UpdateOs;
    request.ipsw = Some(ipsw.to_path_buf());
    request
}

#[test]
fn os_update_with_room_for_one_copy_replaces_it_only_after_asking() {
    let (f, newer, old_bytes) = one_copy_setup();
    let old = record(&f).active_medium().unwrap().directory();
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    // Without a terminal: refused before anything changes, saying why.
    let error = format!(
        "{:#}",
        os_update(&f, &newer)
            .dispatch(&environment(&f.host))
            .unwrap_err()
    );
    assert!(error.contains("Add --yes"), "{error}");
    assert!(error.contains("room for one copy"), "{error}");
    // A dry run says it would ask, and asks nothing.
    let never = ui::tests::Scripted::new(&[]);
    let mut dry = os_update(&f, &newer);
    dry.dry_run = true;
    let plan = dry
        .dispatch(&Environment {
            ui: &never,
            ..environment(&f.host)
        })
        .unwrap();
    assert!(never.shown.borrow().is_empty());
    assert!(plan.contains("remove first"), "{plan}");
    assert!(plan.contains(&old.display().to_string()), "{plan}");
    // A no keeps everything.
    let no = ui::tests::Scripted::new(&["no"]);
    let error = os_update(&f, &newer)
        .dispatch(&Environment {
            ui: &no,
            ..environment(&f.host)
        })
        .unwrap_err()
        .to_string();
    assert_eq!(error, "nothing was changed");
    let asked = no.transcript();
    assert!(asked.contains("cannot install macOS"), "{asked}");
    assert!(asked.contains("keeps starting"), "{asked}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // A yes replaces the copy, whose removal frees its bytes.
    let yes = ui::tests::Scripted::with_hook(&["yes"], |_| f.host.freed.set(old_bytes));
    os_update(&f, &newer)
        .dispatch(&Environment {
            ui: &yes,
            ..environment(&f.host)
        })
        .unwrap();
    assert!(yes.done());
    let updated = record(&f);
    assert_eq!(
        (updated.build.as_str(), updated.replacing.as_deref()),
        ("STELA2", None)
    );
    assert!(!old.exists());
    assert!(updated.active_medium().unwrap().problems().is_empty());
}

#[test]
fn an_interrupted_copy_replacement_is_named_and_finished_by_the_same_update() {
    let (f, newer, old_bytes) = one_copy_setup();
    // Stopped after the old copy went and before the new one was complete.
    let mut stopped = record(&f);
    let old = stopped.active_medium().unwrap().clone();
    stopped.replacing = Some("STELA2".into());
    stopped.media.clear();
    publish(&mut stopped).unwrap();
    write_record(&f.esp, &stopped).unwrap();
    fs::remove_dir_all(old.directory()).unwrap();
    f.host.freed.set(old_bytes);
    let status = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(status.contains("cannot install macOS"), "{status}");
    assert!(
        status.contains(&format!("{} finishes it", fix(&f.esp, "repair"))),
        "{status}"
    );
    // Repair goes on with the switch, asking nothing: the old copy is gone.
    let mut repair = os_update(&f, &newer);
    repair.action = Action::Repair;
    repair.dispatch(&environment(&f.host)).unwrap();
    let finished = record(&f);
    assert_eq!(
        (finished.build.as_str(), finished.replacing.as_deref()),
        ("STELA2", None)
    );
    assert!(
        rerun(&f, |c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
}

#[test]
fn stela_updates_remove_the_previous_bundles() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let preparers: Vec<_> = (0..4).map(preparers).collect();
    setup(
        &environment_at(&f.host, &preparers[0], 0),
        command(&f, &f.esp),
    )
    .unwrap();
    let ecid = record(&f).installs[0].ecid;
    let directory = |revision: u32| {
        let record = record(&f);
        let generation = bundle::Generation {
            revision,
            ..current()
        };
        bundle::directory_of(
            &record.volume,
            bundle::Identity {
                kind: Kind::Restore,
                build: &record.build,
                ecid,
                generation,
            },
        )
    };
    for revision in 1..4 {
        let image = f._dir.path().join(format!("stela-{revision}.efi"));
        fs::write(&image, efi_image(format!("stela-{revision}").as_bytes())).unwrap();
        let mut command = command(&f, &f.esp);
        command.model = None;
        command.action = Action::UpdateStela;
        command.efi = Some(image);
        let environment = environment_at(&f.host, &preparers[revision as usize], revision);
        command.dispatch(&environment).unwrap();
    }
    for revision in 0..3 {
        assert!(!directory(revision).exists());
    }
    assert!(directory(3).join(bundle::MANIFEST).is_file());
    assert_eq!(record(&f).bundles.len(), 1);
}

#[test]
fn interrupted_os_update_finishes_when_run_again() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let before = snapshot(&f.esp);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let ecid = record(&f).installs[0].ecid;
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    let update = |dry_run: bool| {
        rerun(&f, |c| {
            c.action = Action::UpdateOs;
            c.ipsw = Some(newer.clone());
            c.dry_run = dry_run;
        })
    };
    // The Install macOS choice names the build's restore bundle.
    let options = || fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap();
    let first = options();
    assert!(update(true).unwrap().contains("Dry run"));
    assert_eq!(options(), first);
    let leaf = bundle::leaf_of(Kind::Restore, ecid, current());
    let partial = stela_volume(&f).join(format!("stela/boot/STELA2/{leaf}.partial"));
    fs::create_dir_all(&partial).unwrap();
    fs::write(partial.join("partial"), b"x").unwrap();
    let report = update(false).unwrap();
    assert!(!partial.exists());
    assert!(report.contains("choose Install macOS"), "{report}");
    assert!(options().contains("\\boot\\STELA2\\restore-"));
    assert!(
        stela_volume(&f)
            .join(format!("stela/boot/STELA2/{leaf}/bundle.json"))
            .is_file()
    );
    let old = bundle::directory_of(
        &stela_volume(&f),
        bundle::Identity {
            kind: Kind::Restore,
            build: restore::payload::SYNTHETIC_BUILD,
            ecid,
            generation: current(),
        },
    );
    assert!(!old.exists());
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);
}

/// Copy the tree at `from` to `to`.
fn copy_tree(from: &Path, to: &Path) {
    fs::create_dir_all(to).unwrap();
    for entry in fs::read_dir(from).unwrap() {
        let entry = entry.unwrap();
        let path = entry.path();
        if path.is_dir() {
            copy_tree(&path, &to.join(entry.file_name()));
        } else {
            fs::copy(&path, to.join(entry.file_name())).unwrap();
        }
    }
}

#[test]
fn stela_update_stopped_after_its_loader_finishes_when_run_again() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let (older, newer) = (preparers(0), preparers(1));
    setup(&environment_at(&f.host, &older, 0), installed_command(&f)).unwrap();
    let old = record(&f);
    let ecid = old.installs[0].ecid;
    let old_bundle = bundle_of(&f, ecid);
    let saved = f._dir.path().join("saved-bundle");
    copy_tree(&old_bundle, &saved);
    let options = f.esp.join(efi::OPTIONS);
    let (old_options, old_choices) = (
        fs::read(&options).unwrap(),
        fs::read(f.esp.join(CHOICES_PATH)).unwrap(),
    );
    let new_efi = f._dir.path().join("new.efi");
    fs::write(&new_efi, efi_image(b"stela-2")).unwrap();
    let new = environment_at(&f.host, &newer, 1);
    let run = |configure: &dyn Fn(&mut Request)| {
        let mut command = command(&f, &f.esp);
        command.ipsw = None;
        command.model = None;
        configure(&mut command);
        command.dispatch(&new)
    };
    let update = |ipsw: bool, dry_run: bool| {
        run(&|c| {
            c.action = Action::UpdateStela;
            c.efi = Some(new_efi.clone());
            c.ipsw = ipsw.then(|| f.ipsw.clone());
            c.dry_run = dry_run;
        })
    };
    update(true, false).unwrap();
    let done = record(&f);
    // A stop after the new loader went in and before stela.cfg: the record
    // names the old version and both generations' bundles.
    let mut stopped = done.clone();
    stopped.stela_version = old.stela_version.clone();
    stopped.bundles.extend(old.bundles.iter().cloned());
    write_record(&f.esp, &stopped).unwrap();
    fs::write(&options, &old_options).unwrap();
    fs::write(f.esp.join(CHOICES_PATH), &old_choices).unwrap();
    copy_tree(&saved, &old_bundle);
    let status = run(&|c| c.action = Action::Status).unwrap_err().to_string();
    assert!(
        status.contains("not the version setup installed"),
        "{status}"
    );
    assert!(status.contains("left by an interrupted update"), "{status}");
    // A dry run changes nothing; running the update again finishes it
    // without the restore image.
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    assert!(update(false, true).unwrap().contains("Dry run"));
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    let report = update(false, false).unwrap();
    assert!(report.contains("its update is finished"), "{report}");
    assert!(!old_bundle.exists());
    assert_eq!(record(&f), done);
    assert_eq!(fs::read_to_string(&options).unwrap(), done.options);
    let status = run(&|c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
    assert!(update(false, false).unwrap().contains("already installed"));
}

#[test]
fn os_update_leftovers_are_removed_by_running_it_again_or_repair() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let old = record(&f);
    let ecid = old.installs[0].ecid;
    let (old_bundle, old_medium) = (
        bundle_of(&f, ecid),
        old.active_medium().unwrap().directory(),
    );
    let saved = f._dir.path().join("saved");
    copy_tree(&old_bundle, &saved.join("bundle"));
    copy_tree(&old_medium, &saved.join("medium"));
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    let update = |dry_run: bool| {
        rerun(&f, |c| {
            c.action = Action::UpdateOs;
            c.ipsw = Some(newer.clone());
            c.dry_run = dry_run;
        })
    };
    update(false).unwrap();
    let done = record(&f);
    // A stop after the new build became the active one and before the old
    // build's files were removed.
    let stop = || {
        let mut stopped = done.clone();
        stopped.bundles.extend(old.bundles.iter().cloned());
        stopped.media.extend(old.media.iter().cloned());
        write_record(&f.esp, &stopped).unwrap();
        copy_tree(&saved.join("bundle"), &old_bundle);
        copy_tree(&saved.join("medium"), &old_medium);
    };
    let status = || rerun(&f, |c| c.action = Action::Status);
    stop();
    let problems = status().unwrap_err().to_string();
    assert!(
        problems.contains("left by an interrupted update"),
        "{problems}"
    );
    let report = update(false).unwrap();
    assert!(report.contains("earlier builds"), "{report}");
    assert!(!old_bundle.exists() && !old_medium.exists());
    assert_eq!(record(&f), done);
    assert!(status().unwrap().contains("No problems found."));
    // The same build again, with nothing left over, has nothing to do.
    assert!(update(false).unwrap().contains("nothing to update"));
    assert_eq!(record(&f), done);
    stop();
    let plan = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(plan.contains("interrupted update"), "{plan}");
    assert!(old_bundle.exists() && old_medium.exists());
    rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(!old_bundle.exists() && !old_medium.exists());
    assert_eq!(record(&f), done);
    assert!(status().unwrap().contains("No problems found."));
}

/// A first setup stopped while it copied the install medium: the record is
/// incomplete and names the medium without its digest, its manifest is gone,
/// its last piece is short, and the loader and options are not written yet.
/// Returns the medium's first and last pieces.
fn interrupt_setup(f: &Fixture) -> (PathBuf, PathBuf) {
    let mut stopped = record(f);
    stopped.complete = false;
    let medium = &mut stopped.media[0];
    medium.manifest_sha256 = String::new();
    let directory = medium.directory();
    write_record(&f.esp, &stopped).unwrap();
    fs::remove_file(directory.join(stela_formats::payload::MANIFEST_FILE)).unwrap();
    let mut pieces: Vec<PathBuf> = fs::read_dir(&directory)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            path.file_name()
                .unwrap()
                .to_string_lossy()
                .starts_with("ipsw.")
        })
        .collect();
    pieces.sort();
    assert!(pieces.len() >= 2, "{pieces:?}");
    let last = pieces.last().unwrap().clone();
    let bytes = fs::read(&last).unwrap();
    fs::write(&last, &bytes[..bytes.len() / 2]).unwrap();
    for path in [efi::LOADER, efi::OPTIONS, CHOICES_PATH] {
        fs::remove_file(f.esp.join(path)).unwrap();
    }
    (pieces[0].clone(), last)
}

fn set_old(path: &Path) {
    fs::File::options()
        .write(true)
        .open(path)
        .unwrap()
        .set_modified(std::time::SystemTime::UNIX_EPOCH)
        .unwrap();
}

fn is_old(path: &Path) -> bool {
    fs::metadata(path).unwrap().modified().unwrap() == std::time::SystemTime::UNIX_EPOCH
}

#[test]
fn interrupted_first_setup_resumes_keeping_intact_files() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let before = snapshot(&f.esp);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let done = record(&f);
    let ecid = done.installs[0].ecid;
    let manifest = bundle_of(&f, ecid).join(bundle::MANIFEST);
    let (first, last) = interrupt_setup(&f);
    let last_bytes = fs::metadata(&last).unwrap().len();
    set_old(&first);
    set_old(&manifest);
    let status = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(status.contains("was interrupted: "), "{status}");
    // The parts it had not written are named as its, not as other fixes.
    assert!(status.contains("1 problem:"), "{status}");
    assert_eq!(status.matches("repair").count(), 1, "{status}");
    // The same command resumes it: a dry run changes nothing.
    let mut dry = installed_command(&f);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(plan.contains("resumes the setup interrupted"), "{plan}");
    assert!(plan.contains("boot files are intact and kept"), "{plan}");
    assert!(!f.esp.join(efi::LOADER).exists());
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    // The intact piece and bundle were kept, the short piece written again.
    assert!(is_old(&first) && is_old(&manifest));
    assert!(fs::metadata(&last).unwrap().len() > last_bytes);
    let resumed = record(&f);
    assert!(resumed.complete);
    assert_eq!(resumed.installs, done.installs);
    assert_eq!(resumed.media, done.media);
    assert_eq!(resumed.bundles, done.bundles);
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
    // A finished setup is not resumed.
    let again = setup(&environment(&f.host), installed_command(&f))
        .unwrap_err()
        .to_string();
    assert!(again.contains("is already set up"), "{again}");
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(snapshot(&f.esp), before);
    assert!(!stela_volume(&f).join("stela").exists());
}

#[test]
fn interrupted_first_setup_of_another_build_or_target_is_refused() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    interrupt_setup(&f);
    let stopped = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    let mut other = installed_command(&f);
    other.ipsw = Some(newer);
    other.target = Some(REGION2.into());
    other.nor = Some(NOR2.into());
    let error = setup(&environment(&f.host), other).unwrap_err().to_string();
    assert!(
        error.contains("holds an interrupted setup that differs from this one"),
        "{error}"
    );
    assert!(error.contains("this one STELA2"), "{error}");
    assert!(error.contains("target"), "{error}");
    assert!(error.contains("start over with `stela create`"), "{error}");
    let mut renamed = installed_command(&f);
    renamed.install = Some("Other".into());
    let error = setup(&environment(&f.host), renamed)
        .unwrap_err()
        .to_string();
    assert!(error.contains("install \"macOS\""), "{error}");
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), stopped);
    // A different ECID is refused as for any machine.
    let mut ecid = installed_command(&f);
    ecid.ecid = Some(Ecid::parse("0x1234").unwrap());
    let error = setup(&environment(&f.host), ecid).unwrap_err().to_string();
    assert!(error.contains("this machine's ECID is"), "{error}");
}

#[test]
fn interrupted_first_setup_prepares_a_damaged_bundle_again() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let ecid = record(&f).installs[0].ecid;
    let bundle = bundle_of(&f, ecid);
    let ramdisk = bundle.join("ramdisk.img");
    let good = fs::read(&ramdisk).unwrap();
    interrupt_setup(&f);
    // A copy that stopped part way: a short file, and the next copy's
    // partial sibling.
    fs::write(&ramdisk, &good[..good.len() / 2]).unwrap();
    let partial = bundle.with_file_name(format!(
        "{}.partial",
        bundle.file_name().unwrap().to_string_lossy()
    ));
    fs::create_dir_all(&partial).unwrap();
    fs::write(partial.join("ramdisk.img"), b"x").unwrap();
    let plan = {
        let mut dry = installed_command(&f);
        dry.dry_run = true;
        setup(&environment(&f.host), dry).unwrap()
    };
    assert!(!plan.contains("intact and kept"), "{plan}");
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    assert_eq!(fs::read(&ramdisk).unwrap(), good);
    assert!(!partial.exists());
    assert_eq!(record(&f).installs[0].ecid, ecid);
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
}

/// Rewrite the record on the USB as another computer would have written it,
/// with the USB's partitions mounted at `old_esp` and `old_volume`.
fn written_elsewhere(f: &Fixture, old_esp: &Path, old_volume: &Path) {
    let path = f.esp.join(RECORD);
    let text = fs::read_to_string(&path)
        .unwrap()
        .replace(
            stela_volume(f).to_str().unwrap(),
            old_volume.to_str().unwrap(),
        )
        .replace(f.esp.to_str().unwrap(), old_esp.to_str().unwrap());
    fs::write(&path, text).unwrap();
}

#[test]
fn an_interrupted_setup_follows_the_usb_to_where_it_is_mounted_now() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let done = record(&f);
    let old = f._dir.path().join("elsewhere");
    let (old_esp, old_volume) = (old.join("EFI"), old.join("STELA"));
    // Another USB's files where this one was mounted, which nothing touches.
    for directory in [
        bundle::BOOT_DIRECTORY,
        "stela/install",
        translations::DIRECTORY,
    ] {
        fs::create_dir_all(old_volume.join(directory).join(&done.build)).unwrap();
        fs::write(old_volume.join(directory).join("other"), b"other").unwrap();
    }
    let decoy = snapshot(&old);
    interrupt_setup(&f);
    written_elsewhere(&f, &old_esp, &old_volume);
    let mut dry = installed_command(&f);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(plan.contains("resumes the setup interrupted"), "{plan}");
    assert!(!plan.contains(old.to_str().unwrap()), "{plan}");
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let resumed = record(&f);
    assert!(resumed.complete);
    assert_eq!(resumed.volume, stela_volume(&f));
    assert_eq!(resumed.media, done.media);
    // Undone from another mount, it removes this USB's files only.
    interrupt_setup(&f);
    written_elsewhere(&f, &old_esp, &old_volume);
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert!(!stela_volume(&f).join("stela").exists());
    assert!(!f.esp.join(RECORD).exists());
    assert_eq!(snapshot(&old), decoy);
}

/// The install medium's pieces in the active build's directory, in order.
fn medium_pieces(f: &Fixture) -> Vec<PathBuf> {
    let directory = record(f).active_medium().unwrap().directory();
    let mut pieces: Vec<PathBuf> = fs::read_dir(&directory)
        .unwrap()
        .map(|entry| entry.unwrap().path())
        .filter(|path| {
            let name = path.file_name().unwrap().to_string_lossy().into_owned();
            name.starts_with("ipsw.") && !name.ends_with(".partial")
        })
        .collect();
    pieces.sort();
    pieces
}

/// Run the interrupted setup again as `stela repair` does: no
/// installer choice, which comes from the record.
fn resume_setup(f: &Fixture) -> String {
    let mut again = installed_command(f);
    again.install_image = None;
    setup(&environment(&f.host), again).unwrap()
}

#[test]
fn an_interrupted_full_setup_resumes_as_a_full_install_and_keeps_its_pieces() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    // An older stela's setup that copied the whole image and stopped part
    // way; its record names the image choice by its first name.
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let (first, last) = interrupt_setup(&f);
    let path = f.esp.join(RECORD);
    let text = fs::read_to_string(&path).unwrap();
    assert!(text.contains("\"install_image\": \"full\""), "{text}");
    fs::write(&path, text.replace("\"full\"", "\"offline\"")).unwrap();
    let pieces = medium_pieces(&f);
    for piece in &pieces {
        set_old(piece);
    }
    let report = resume_setup(&f);
    assert!(report.contains("Done."), "{report}");
    // The last piece is short, so it is not counted as on the USB.
    let found = format!(
        "continuing the Offline installer: {} parts of macOS already on the USB",
        pieces.len() - 1
    );
    assert!(report.contains(&found), "{report}");
    let resumed = record(&f);
    assert!(resumed.complete);
    assert_eq!(resumed.install_image, Some(InstallImage::Full));
    let medium = resumed.active_medium().unwrap();
    assert!(!medium.remote && medium.problems().is_empty(), "{medium:?}");
    // Every whole piece is kept; only the short one is written again.
    assert_eq!(medium_pieces(&f), pieces);
    for piece in &pieces {
        assert_eq!(is_old(piece), *piece != last, "{}", piece.display());
    }
    assert!(first.exists());
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
}

#[test]
fn an_older_stelas_interrupted_setup_is_resumed_with_new_bundles_and_its_pieces() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let (older, newer) = (preparers(0), preparers(1));
    setup(&environment_at(&f.host, &older, 0), installed_command(&f)).unwrap();
    let (_, last) = interrupt_setup(&f);
    let stopped = record(&f);
    let ecid = stopped.installs[0].ecid;
    let old_bundle = bundle_of(&f, ecid);
    assert!(old_bundle.is_dir());
    let pieces = medium_pieces(&f);
    for piece in &pieces {
        set_old(piece);
    }
    // A later stela prepares revision 1: the same command finishes it.
    let new = environment_at(&f.host, &newer, 1);
    let mut again = installed_command(&f);
    again.install_image = None;
    let report = setup(&new, again).unwrap();
    assert!(report.contains("Done."), "{report}");
    assert!(report.contains("an older stela's"), "{report}");
    let resumed = record(&f);
    assert!(resumed.complete);
    assert_eq!(resumed.bundle_revision, 1);
    assert!(resumed.bundles.iter().all(|b| b.revision == 1));
    assert!(!old_bundle.exists());
    assert!(bundle_of(&f, ecid).is_dir());
    assert_eq!(resumed.installs[0].ecid, ecid);
    for piece in &pieces {
        assert_eq!(is_old(piece), *piece != last, "{}", piece.display());
    }
    let mut status = installed_command(&f);
    status.action = Action::Status;
    let status = status.dispatch(&new).unwrap();
    assert!(status.contains("No problems found."), "{status}");
}

#[test]
fn a_resumed_full_setup_writes_again_only_its_damaged_piece() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let (first, last) = interrupt_setup(&f);
    let pieces = medium_pieces(&f);
    // The first piece keeps its size but not its bytes.
    let bytes = fs::read(&first).unwrap();
    fs::write(&first, vec![0xa5; bytes.len()]).unwrap();
    for piece in &pieces {
        set_old(piece);
    }
    let report = resume_setup(&f);
    assert!(report.contains("Done."), "{report}");
    assert_eq!(fs::read(&first).unwrap(), bytes);
    for piece in &pieces {
        let written = *piece == first || *piece == last;
        assert_eq!(is_old(piece), !written, "{}", piece.display());
    }
    assert!(record(&f).active_medium().unwrap().problems().is_empty());
}

#[test]
fn an_interrupted_setup_from_before_the_choice_was_recorded_resumes_by_its_medium() {
    for remote in [false, true] {
        let mut f = fixture(64 * GIB, false);
        add_installed_disk(&mut f, &[], &[]);
        let mut first = installed_command(&f);
        if remote {
            first.install_image = Some(InstallImage::Network);
        }
        setup(&environment(&f.host), first).unwrap();
        let mut stopped = record(&f);
        stopped.complete = false;
        stopped.install_image = None;
        write_record(&f.esp, &stopped).unwrap();
        for path in [efi::LOADER, efi::OPTIONS, CHOICES_PATH] {
            fs::remove_file(f.esp.join(path)).unwrap();
        }
        let text = fs::read_to_string(f.esp.join(RECORD)).unwrap();
        assert!(!text.contains("install_image"), "{text}");
        let report = resume_setup(&f);
        assert!(report.contains("Done."), "{report}");
        let resumed = record(&f);
        let kind = match remote {
            true => InstallImage::Network,
            false => InstallImage::Full,
        };
        assert_eq!(resumed.install_image, Some(kind));
        let medium = resumed.active_medium().unwrap();
        assert!(medium.remote == remote && medium.problems().is_empty());
    }
}

#[test]
fn an_interrupted_network_setup_resumes_as_a_network_install() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let mut network = installed_command(&f);
    network.install_image = Some(InstallImage::Network);
    setup(&environment(&f.host), network).unwrap();
    let mut stopped = record(&f);
    stopped.complete = false;
    write_record(&f.esp, &stopped).unwrap();
    for path in [efi::LOADER, efi::OPTIONS, CHOICES_PATH] {
        fs::remove_file(f.esp.join(path)).unwrap();
    }
    assert_eq!(stopped.resume_summary(), "continuing the Online installer");
    let report = resume_setup(&f);
    assert!(
        report.contains("continuing the Online installer"),
        "{report}"
    );
    let resumed = record(&f);
    assert!(resumed.complete);
    assert_eq!(resumed.install_image, Some(InstallImage::Network));
    assert!(resumed.active_medium().unwrap().remote);
}

#[test]
fn interrupted_disk_add_is_finished_by_repair_initialize_nor() {
    use stela_runtime::pc::boot_log::{LOG_BYTES, OFFSET};
    use stela_soc_data::nor::{CAPACITY, Image};
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    rerun(&f, |c| {
        c.target = Some(REGION2.into());
        c.nor = Some(NOR2.into());
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    // `--target --initialize-nor` stopped after recording the install and
    // before its bundle and its machine state: the NOR partition still
    // holds another program's bytes, and stela.efi never booted it.
    let mut stopped = record(&f);
    let second = stopped.installs[1].ecid;
    stopped.bundles.retain(|b| b.ecid != second);
    write_record(&f.esp, &stopped).unwrap();
    let foreign = vec![0xa5; CAPACITY + stela_formats::machine_state::SEP_BYTES as usize];
    f.host.write(NOR2, 0, &foreign).unwrap();
    f.host
        .write(NOR2, OFFSET, &vec![0; LOG_BYTES as usize])
        .unwrap();
    let state = || f.host.read(NOR2, 0, foreign.len()).unwrap();
    let status = || rerun(&f, |c| c.action = Action::Status);
    let problems = status().unwrap_err().to_string();
    assert!(problems.contains("never booted"), "{problems}");
    assert!(
        problems.contains("`stela repair --initialize-nor`"),
        "{problems}"
    );
    assert!(!problems.contains("state restore"), "{problems}");
    assert!(problems.contains("`stela repair`"), "{problems}");
    // Without the flag, repair names it and leaves the partition alone.
    let repair = |initialize: bool, dry_run: bool| {
        rerun(&f, |c| {
            c.action = Action::Repair;
            c.ipsw = Some(f.ipsw.clone());
            c.initialize_nor = initialize;
            c.dry_run = dry_run;
        })
    };
    let report = repair(false, false).unwrap();
    assert!(
        report.contains("`stela repair --initialize-nor`"),
        "{report}"
    );
    assert_eq!(state()[..], foreign[..]);
    let plan = repair(true, true).unwrap();
    assert!(plan.contains("a blank SEP area"), "{plan}");
    assert_eq!(state()[..], foreign[..]);
    repair(true, false).unwrap();
    let written = state();
    assert_eq!(&written[..CAPACITY], Image::fresh().bytes());
    assert!(written[CAPACITY..].iter().all(|&b| b == 0));
    let report = status().unwrap();
    assert!(report.contains("No problems found."), "{report}");

    // A machine that booted may keep its keys there: never blanked.
    f.host.write(NOR2, CAPACITY as u64, &[0xa5; 4096]).unwrap();
    let region = crate::boot_log::tests::region(&["NOR persistent fresh=true\n"]);
    f.host.write(NOR2, OFFSET, &region).unwrap();
    let problems = status().unwrap_err().to_string();
    assert!(
        problems.contains("If you have a backup, restore it with"),
        "{problems}"
    );
    assert!(
        problems.contains("--remove-install \"macOS 2\""),
        "{problems}"
    );
    let before = state();
    let error = repair(true, false).unwrap_err().to_string();
    assert!(error.contains("may hold its keys"), "{error}");
    assert_eq!(state(), before);
}

#[test]
fn records_with_the_retired_signing_opt_in_are_still_read() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let path = f.esp.join(RECORD);
    let mut json: serde_json::Value = stela_engine::read_json(&path).unwrap();
    json["tss"] = serde_json::Value::Bool(true);
    stela_engine::write_json(&path, &json).unwrap();
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
    // A bundle an older setup prepared under the global ticket is missing.
    let mut global = record(&f);
    global.bundles[0].personalized = false;
    write_record(&f.esp, &global).unwrap();
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(problems.contains("`stela repair`"), "{problems}");
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    assert!(record(&f).bundles.iter().all(|b| b.personalized));
    assert!(
        rerun(&f, |c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
}

#[test]
fn installed_os_update_preserves_user_data() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let ecid = record(&f).installs[0].ecid.unwrap();
    assert_eq!(record(&f).media.len(), 1);
    assert!(
        rerun(&f, |c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
    let disks = (partition_bytes(&f, REGION), partition_bytes(&f, NOR));
    let choices =
        || Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    let names: Vec<String> = choices().entries.into_iter().map(|e| e.name).collect();
    assert_eq!(names, ["macOS", INSTALL_CHOICE]);
    let first = restore::payload::SYNTHETIC_BUILD;
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    let update_to = |ipsw: &Path, dry_run: bool| {
        rerun(&f, |c| {
            c.action = Action::UpdateOs;
            c.ipsw = Some(ipsw.to_path_buf());
            c.dry_run = dry_run;
        })
    };
    let before = snapshot(&f.esp);
    let plan = update_to(&newer, true).unwrap();
    assert!(plan.contains("installer      Offline: "), "{plan}");
    assert!(plan.contains("remove"), "{plan}");
    assert_eq!(snapshot(&f.esp), before);
    update_to(&newer, false).unwrap();
    let record1 = record(&f);
    assert_eq!(record1.build, "STELA2");
    let media: Vec<&str> = record1.media.iter().map(|m| m.build.as_str()).collect();
    assert_eq!(media, ["STELA2"], "the old medium is removed");
    assert!(record1.active_medium().unwrap().problems().is_empty());
    assert!(record1.restore_bundle(Kind::Restore, Some(ecid)).is_some());
    for entry in &choices().entries[1..2] {
        assert!(
            entry.options.contains("\\boot\\STELA2\\")
                && entry.options.contains("\\install\\STELA2\\payload.json"),
            "{}",
            entry.options
        );
    }
    // The update writes no disk of the PC.
    assert_eq!(
        (partition_bytes(&f, REGION), partition_bytes(&f, NOR)),
        disks
    );
    let newest = f._dir.path().join("Newest.ipsw");
    restore::payload::synthetic_ipsw_of(&newest, "STELA3").unwrap();
    update_to(&newest, false).unwrap();
    let media: Vec<String> = record(&f).media.into_iter().map(|m| m.build).collect();
    assert_eq!(media, ["STELA3"]);
    assert!(!medium::directory(&stela_volume(&f), first).exists());
    assert!(
        rerun(&f, |c| c.action = Action::Status)
            .unwrap()
            .contains("No problems found.")
    );
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert!(!f.esp.join("EFI/stela").exists());
    assert!(!stela_volume(&f).join("stela/install").exists());
}

#[test]
fn repair_restores_recorded_state() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let status = || rerun(&f, |c| c.action = Action::Status);
    let report = status().unwrap();
    assert!(report.contains("No problems found."), "{report}");
    assert!(report.contains("Install \"macOS\" (default)"), "{report}");
    assert!(report.contains("ECID           0x"), "{report}");
    assert!(report.contains("no installed macOS yet"), "{report}");
    assert!(report.contains("NVRAM          blank"), "{report}");
    assert!(report.contains("SEP state      blank"), "{report}");
    assert!(report.contains("intact"), "{report}");
    // Status changes nothing.
    let before = snapshot(&f.esp);
    status().unwrap();
    assert_eq!(snapshot(&f.esp), before);

    // A damaged bundle, loader and stela.cfg are found ...
    let ecid = record(&f).installs[0].ecid;
    let ramdisk = bundle_of(&f, ecid).join("ramdisk.img");
    fs::write(&ramdisk, [0u8; 10]).unwrap();
    fs::write(f.esp.join(LOADER_PATH), b"MZ damaged").unwrap();
    fs::write(f.esp.join("EFI/stela/stela.cfg"), b"--system\n").unwrap();
    let error = status().unwrap_err().to_string();
    assert!(error.contains("problems:"), "{error}");
    assert!(error.contains("ramdisk.img"), "{error}");
    assert!(error.contains("not the version setup installed"), "{error}");
    assert!(error.contains("stela.cfg differs"), "{error}");
    // ... an update puts the loader back ...
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(f.efi.clone());
    })
    .unwrap();
    fs::write(f.esp.join("EFI/stela/stela.cfg"), b"--system\n").unwrap();
    // ... and the bundle is prepared again from the USB's own copy of the
    // image, or from --ipsw.
    let damaged = snapshot(&f.esp);
    let plan = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(plan.contains("from this USB's copy of macOS"), "{plan}");
    assert_eq!(snapshot(&f.esp), damaged);
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
        c.dry_run = true;
    })
    .unwrap();
    assert_eq!(snapshot(&f.esp), damaged);
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    // The bundle is the machine's own again, with its ECID.
    assert_eq!(record(&f).installs[0].ecid, ecid);
    assert_eq!(
        fs::read(&ramdisk).unwrap(),
        vec![ecid.unwrap().0 as u8 ^ 0xa5; 2048]
    );
    assert_eq!(snapshot(&f.esp), before);
    status().unwrap();

    // A machine an older setup made without an ECID gets one.
    let mut old = record(&f);
    old.installs[0].ecid = None;
    write_record(&f.esp, &old).unwrap();
    let error = status().unwrap_err().to_string();
    assert!(error.contains("has no ECID"), "{error}");
    assert!(error.contains("`stela repair`"), "{error}");
    assert!(!error.contains("--fetch"), "{error}");
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    let repaired = record(&f);
    let new = repaired.installs[0].ecid.unwrap();
    assert!(bundle_of(&f, Some(new)).join("bundle.json").is_file());
    status().unwrap();
}

#[test]
fn a_record_spells_a_path_one_way_however_it_was_built() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let path = f.esp.join(RECORD);
    let written = fs::read(&path).unwrap();
    // The same directory with the separator this system does not write, as
    // joining `stela/boot` whole leaves it on Windows.
    let mut made = record(&f);
    let (own, other) = match std::path::MAIN_SEPARATOR {
        '/' => ("/", "\\"),
        _ => ("\\", "/"),
    };
    let below = made.bundle_directory.strip_prefix(&made.volume).unwrap();
    let below = below.to_str().unwrap().replace(own, other);
    assert!(below.contains(other), "{below}");
    made.bundle_directory = format!("{}{other}{below}", made.volume.display()).into();
    write_record(&f.esp, &made).unwrap();
    assert_eq!(fs::read(&path).unwrap(), written);
}

#[test]
fn maintenance_writes_only_machine_state() {
    let mut f = fixture(64 * GIB, false);
    // An installed macOS: the Apple region holds the guest's own GPT,
    // and its containers' bytes after it stand for the user's data.
    let mut region = vec![0u8; 64 * 1024];
    region[512..520].copy_from_slice(b"EFI PART");
    for (n, byte) in region[1024..].iter_mut().enumerate() {
        *byte = (n % 251) as u8;
    }
    add_installed_disk(&mut f, &region, &[]);
    // A partition of another system on the same disk.
    let foreign = f._dir.path().join("disk1s4.img");
    fs::write(&foreign, vec![0xa5; IMAGE_BYTES]).unwrap();
    f.host.partitions.insert(
        "disk1s4".into(),
        (
            PartitionInfo {
                identifier: "disk1s4".into(),
                disk: "disk1".into(),
                guid: "5e1a0000-0000-4000-8000-0000000000ff".into(),
                bytes: 50 * GIB,
                content: "Apple_HFS".into(),
                mount: None,
                container: None,
            },
            foreign.clone(),
        ),
    );
    let disks = || {
        (
            partition_bytes(&f, REGION),
            partition_bytes(&f, NOR),
            fs::read(&foreign).unwrap(),
            snapshot(&f.data),
        )
    };
    let before = disks();
    let report = setup(&environment(&f.host), installed_command(&f)).unwrap();
    assert!(report.contains("macOS disk"), "{report}");
    assert_eq!(disks(), before, "setup");
    let new = f._dir.path().join("new.efi");
    fs::write(&new, efi_image(b"stela-2")).unwrap();
    let newer = f._dir.path().join("Newer.ipsw");
    restore::payload::synthetic_ipsw_of(&newer, "STELA2").unwrap();
    type Step<'a> = (&'a str, &'a dyn Fn(&mut Request));
    let steps: [Step<'_>; 5] = [
        ("status", &|c| c.action = Action::Status),
        ("update-stela", &|c| {
            c.action = Action::UpdateStela;
            c.efi = Some(new.clone());
        }),
        ("update-os", &|c| {
            c.action = Action::UpdateOs;
            c.ipsw = Some(newer.clone());
        }),
        ("repair", &|c| {
            c.action = Action::Repair;
            c.ipsw = Some(newer.clone());
        }),
        ("status", &|c| c.action = Action::Status),
    ];
    for (name, step) in steps {
        rerun(&f, step).unwrap_or_else(|error| panic!("{name}: {error:#}"));
        assert_eq!(disks(), before, "{name}");
    }
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert_eq!(disks(), before, "uninstall");
}

#[test]
fn repair_recovers_interrupted_bundle_rename() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let before = snapshot(&f.esp);
    let directory = bundle_of(&f, record(&f).installs[0].ecid);
    // The stop came after the old bundle was set aside, before the new
    // copy was renamed into place.
    let (partial, previous) = bundle::siblings(&directory).unwrap();
    fs::rename(&directory, &previous).unwrap();
    fs::create_dir_all(&partial).unwrap();
    fs::write(partial.join("ramdisk.img"), b"half").unwrap();
    let error = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(error.contains("set aside"), "{error}");
    assert!(error.contains("stopped part way"), "{error}");
    // A dry run says so and changes nothing, and needs no restore image.
    let interrupted = snapshot(&f.esp);
    let dry = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(dry.contains("would put right"), "{dry}");
    assert_eq!(snapshot(&f.esp), interrupted);
    let report = rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(report.contains("put back"), "{report}");
    assert!(!partial.exists() && !previous.exists());
    assert_eq!(snapshot(&f.esp), before);
    rerun(&f, |c| c.action = Action::Status).unwrap();
}

#[test]
fn machine_state_roundtrip_requires_identity() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    provision_sep(&f, NOR);
    rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Recovery)).unwrap();
    let capacity = stela_soc_data::nor::CAPACITY;
    let sep = |f: &Fixture| partition_bytes(f, NOR)[capacity..].to_vec();
    let sep_before = sep(&f);

    // A reset finds the partition by its GUID; a dry run writes nothing.
    let unchanged = partition_bytes(&f, NOR);
    let plan = rerun(&f, |c| {
        c.action = Action::ResetNvram;
        c.dry_run = true;
    })
    .unwrap();
    assert!(plan.contains("disk1s3"), "{plan}");
    assert_eq!(partition_bytes(&f, NOR), unchanged);
    assert_eq!(sep(&f), sep_before);
    let error = rerun(&f, |c| {
        c.action = Action::ResetNvram;
        c.nor = Some(NOR2.into());
    })
    .unwrap_err()
    .to_string();
    assert!(
        error.contains("not this install's machine-state partition"),
        "{error}"
    );
    let state = partition_bytes(&f, NOR);
    rerun(&f, |c| {
        c.action = Action::ResetNvram;
        c.nor = Some(NOR.into());
        c.dry_run = true;
    })
    .unwrap();
    assert_eq!(partition_bytes(&f, NOR), state);
    // Back up before the reset.
    let file = f._dir.path().join("machine.state");
    rerun(&f, |c| c.action = Action::BackupState(file.clone())).unwrap();
    #[cfg(unix)]
    assert_eq!(
        std::os::unix::fs::PermissionsExt::mode(&fs::metadata(&file).unwrap().permissions())
            & 0o777,
        0o600
    );
    assert!(
        rerun(&f, |c| c.action = Action::BackupState(file.clone()))
            .unwrap_err()
            .to_string()
            .contains("exists")
    );
    rerun(&f, |c| {
        c.action = Action::ResetNvram;
        c.nor = Some(NOR.into());
    })
    .unwrap();
    assert_eq!(
        stela_loader::boot_selection::select(&nvram(&f))
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::MacOs { safe: false }
    );
    assert_eq!(sep(&f), sep_before);
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("SEP state      provisioned"), "{status}");
    assert!(status.contains("next boot      macOS"), "{status}");

    // The backup restores only to its own machine.
    let mut second = installed_command(&f);
    second.target = Some(REGION2.into());
    second.nor = Some(NOR2.into());
    second.install = Some("Other".into());
    second.model = None;
    second.dispatch(&environment(&f.host)).unwrap();
    let error = rerun(&f, |c| {
        c.action = Action::RestoreState(file.clone());
        c.install = Some("Other".into());
        c.nor = Some(NOR2.into());
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("restored only to that machine"), "{error}");
    let damaged = f._dir.path().join("damaged.state");
    let mut bytes = fs::read(&file).unwrap();
    *bytes.last_mut().unwrap() ^= 1;
    fs::write(&damaged, bytes).unwrap();
    let error = rerun(&f, |c| {
        c.action = Action::RestoreState(damaged.clone());
        c.nor = Some(NOR.into());
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("damaged"), "{error}");
    rerun(&f, |c| {
        c.action = Action::RestoreState(file.clone());
        c.nor = Some(NOR.into());
    })
    .unwrap();
    assert_eq!(partition_bytes(&f, NOR), state);
    assert_eq!(
        stela_loader::boot_selection::select(&nvram(&f))
            .unwrap()
            .target,
        stela_loader::boot_selection::Target::SystemRecovery
    );
    // --nor alone names nothing to do.
    assert!(rerun(&f, |c| c.nor = Some(NOR.into())).is_err());
}

#[test]
fn boot_log_prints_the_install_s_log_and_names_a_missing_one() {
    use stela_runtime::pc::boot_log::{LOG_BYTES, OFFSET, PARTITION_BYTES};
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    let plan = setup(&environment(&f.host), installed_command(&f)).unwrap();
    assert!(plan.contains("keeps the boot log"), "{plan}");
    let path = f.host.image(NOR).unwrap().to_path_buf();
    let mut bytes = fs::read(&path).unwrap();
    bytes.resize(PARTITION_BYTES as usize, 0);
    fs::write(&path, &bytes).unwrap();
    let error = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::BootLog).unwrap_err()
    );
    assert!(error.contains("the log region is blank"), "{error}");

    let region = crate::boot_log::tests::region(&["NOR persistent fresh=true\n"]);
    bytes[OFFSET as usize..(OFFSET + LOG_BYTES) as usize].copy_from_slice(&region);
    fs::write(&path, &bytes).unwrap();
    let state = partition_bytes(&f, NOR);
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(
        text.contains("(disk1s3): Stela boot log: 1 boots, 47 bytes\n"),
        "{text}"
    );
    assert!(
        text.ends_with("=== STELA BOOT 1 ===\nNOR persistent fresh=true\n"),
        "{text}"
    );
    assert_eq!(partition_bytes(&f, NOR), state);
    // The USB's own log, which setup preallocates, comes first.
    let usb = crate::boot_log::tests::region(&["CHOICE 3 (Install macOS)\n"]);
    fs::write(f.esp.join(efi::BOOT_LOG), &usb).unwrap();
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(text.starts_with("Boot log on the USB ("), "{text}");
    assert!(text.contains("CHOICE 3 (Install macOS)\n"), "{text}");
    assert!(
        text.ends_with("=== STELA BOOT 1 ===\nNOR persistent fresh=true\n"),
        "{text}"
    );
    // The log is optional: a smaller partition is noted, not refused,
    // and the USB's log still prints.
    f.host.partitions.get_mut(NOR).unwrap().0.bytes = PARTITION_BYTES - 1;
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(text.starts_with("Boot log on the USB ("), "{text}");
    assert!(text.ends_with("without one\n"), "{text}");
    fs::write(f.esp.join(efi::BOOT_LOG), vec![0; LOG_BYTES as usize]).unwrap();
    let error = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::BootLog).unwrap_err()
    );
    assert!(
        error.contains("has no boot log: the log region is blank\n"),
        "{error}"
    );
    assert!(error.ends_with("without one"), "{error}");
    let mut again = installed_command(&f);
    again.target = Some(REGION2.into());
    again.nor = Some(NOR2.into());
    add_disk(&mut f, true, &[], &[]);
    f.host.partitions.get_mut(NOR2).unwrap().0.bytes = PARTITION_BYTES - 1;
    again.install = Some("Other".into());
    again.model = None;
    again.dry_run = true;
    let plan = again.dispatch(&environment(&f.host)).unwrap();
    assert!(plan.contains("has no room for the boot log"), "{plan}");
    assert!(
        plan.contains("has no room for the board devices' flash"),
        "{plan}"
    );
}

/// The Apple region of a one-partition install: 100 GiB, a sparse image.
const SPLIT_BYTES: u64 = 100 * GIB;

/// Give `f`'s PC a disk (`disk1`, or `disk2` with `second`) whose Apple
/// region alone is set up, as a sparse image of the whole partition
/// that starts with `head`.
fn add_split_disk(f: &mut Fixture, second: bool, head: &[u8]) {
    add_stela(f);
    let (disk, serial, identifier, guid) = if second {
        ("disk2", SERIAL2, REGION2, REGION2_GUID)
    } else {
        ("disk1", SERIAL, REGION, REGION_GUID)
    };
    let path = f._dir.path().join(format!("{identifier}.img"));
    sparse_image(&path, SPLIT_BYTES);
    let info = PartitionInfo {
        identifier: identifier.into(),
        disk: disk.into(),
        guid: guid.to_ascii_lowercase(),
        bytes: SPLIT_BYTES,
        content: "Apple_APFS".into(),
        mount: None,
        container: None,
    };
    f.host.partitions.insert(identifier.into(), (info, path));
    f.host.serials.insert(disk.into(), serial.into());
    f.host.write(identifier, 0, head).unwrap();
}

/// Where the region's machine state starts.
fn tail() -> u64 {
    stela_runtime::pc::machine_state::split(SPLIT_BYTES)
        .unwrap()
        .state_offset()
}

fn split_command(f: &Fixture) -> Request {
    let mut command = command(f, &f.esp);
    command.target = Some(REGION.into());
    command
}

/// The region's machine state: its NOR image and SEP area.
fn state_of(f: &Fixture, identifier: &str) -> Vec<u8> {
    let bytes = stela_soc_data::nor::CAPACITY + stela_formats::machine_state::SEP_BYTES as usize;
    f.host.read(identifier, tail(), bytes).unwrap()
}

fn tail_nvram(f: &Fixture) -> Variables {
    let capacity = stela_soc_data::nor::CAPACITY;
    let image =
        stela_soc_data::nor::Image::from_bytes(state_of(f, REGION)[..capacity].to_vec()).unwrap();
    Variables::parse(image.bank(image.current_bank().unwrap()).unwrap()).unwrap()
}

fn next_target(variables: &Variables) -> stela_loader::boot_selection::Target {
    stela_loader::boot_selection::select(variables)
        .unwrap()
        .target
}

/// A guest disk's primary GPT header at LBA 1 of 512-byte blocks, whose
/// backup header is at `backup`.
fn guest_gpt(backup: u64) -> Vec<u8> {
    let mut head = vec![0u8; 1024];
    head[512..520].copy_from_slice(b"EFI PART");
    head[512 + 32..512 + 40].copy_from_slice(&backup.to_le_bytes());
    head[512 + 48..512 + 56].copy_from_slice(&(backup - 33).to_le_bytes());
    head
}

#[test]
fn machine_state_tail_stays_outside_guest_region() {
    use stela_soc_data::nor::CAPACITY;
    let mut f = fixture(64 * GIB, false);
    add_split_disk(&mut f, false, &[]);
    let mut dry = split_command(&f);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(
        plan.contains(&format!(
            "machine state  the tail of disk1s2 from byte {:#x}",
            tail()
        )),
        "{plan}"
    );
    assert!(
        plan.contains(&format!(
            "the Mac's SSD is the {} before it",
            host::bytes_text(tail())
        )),
        "{plan}"
    );
    assert!(plan.contains("keeps the boot log"), "{plan}");
    assert!(!f.esp.join(RECORD).exists());

    setup(&environment(&f.host), split_command(&f)).unwrap();
    let record = record(&f);
    let disk = &record.installs[0].disk;
    assert_eq!(disk.machine_state_offset, Some(tail()));
    assert_eq!(
        (
            disk.nor.as_str(),
            disk.nor_serial.as_str(),
            disk.nor_guid.as_str()
        ),
        (REGION, SERIAL, REGION_GUID)
    );
    // The record keeps the choice, and the boot line names the region as
    // both the SSD and the machine state.
    let json = fs::read_to_string(f.esp.join(RECORD)).unwrap();
    assert!(
        json.contains(&format!("\"machine_state_offset\": {}", tail())),
        "{json}"
    );
    let ecid = record.installs[0].ecid.unwrap();
    let region = format!("{SERIAL}:{REGION_GUID}");
    let restore = format!(
        "\\stela\\boot\\{}\\{}",
        restore::payload::SYNTHETIC_BUILD,
        bundle::leaf_of(Kind::Restore, Some(ecid), current())
    );
    assert_eq!(
        fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap(),
        format!(
            "--system --storage {region} --nor {region} --preboot {region} --ecid {ecid} --input\n"
        )
    );
    let choices = Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    assert!(
        choices.entries[1].options.starts_with(&format!(
            "--system --storage {region} --nor {region} --bundle {restore} --ecid {ecid} "
        )),
        "{}",
        choices.entries[1].options
    );
    // Setup itself wrote neither the SSD nor the machine state.
    assert!(
        f.host
            .read(REGION, 0, 1 << 20)
            .unwrap()
            .iter()
            .all(|&b| b == 0)
    );
    assert!(state_of(&f, REGION).iter().all(|&b| b == 0));

    // NVRAM is the tail's: the next boot is chosen there.
    rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Recovery)).unwrap();
    assert_eq!(
        next_target(&tail_nvram(&f)),
        stela_loader::boot_selection::Target::SystemRecovery
    );
    assert!(
        f.host
            .read(REGION, 0, 1 << 20)
            .unwrap()
            .iter()
            .all(|&b| b == 0)
    );
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains(&format!(
            "machine state  the tail of disk1s2 from byte {:#x}",
            tail()
        )),
        "{status}"
    );
    assert!(
        status.contains("next boot      System Recovery"),
        "{status}"
    );
    assert!(status.contains("SEP state      blank"), "{status}");
    assert!(status.contains("No problems found."), "{status}");

    // A provisioned SEP area in the tail backs up, survives a reset and
    // restores, each command addressing the tail.
    let mut roots = vec![0u8; 8 + 2 * 32];
    roots[..8].copy_from_slice(b"STSEPR01");
    roots[8..].fill(0x42);
    use sha2::{Digest, Sha256};
    let sum = Sha256::digest(&roots);
    roots.extend_from_slice(&sum);
    f.host
        .write(REGION, tail() + CAPACITY as u64, &roots)
        .unwrap();
    let state = state_of(&f, REGION);
    let file = f._dir.path().join("split.state");
    rerun(&f, |c| c.action = Action::BackupState(file.clone())).unwrap();
    // Its SSD partition's tail is found by the partition's GUID.
    let found = rerun(&f, |c| c.action = Action::ResetNvram).unwrap();
    assert!(
        found.contains(&format!("disk1s2 from byte {:#x} (partition", tail())),
        "{found}"
    );
    let reset = rerun(&f, |c| {
        c.action = Action::ResetNvram;
        c.nor = Some(REGION.into());
    })
    .unwrap();
    assert!(
        reset.contains(&format!("disk1s2 from byte {:#x} (partition", tail())),
        "{reset}"
    );
    assert_eq!(
        next_target(&tail_nvram(&f)),
        stela_loader::boot_selection::Target::MacOs { safe: false }
    );
    assert_eq!(state_of(&f, REGION)[CAPACITY..], state[CAPACITY..]);
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("SEP state      provisioned"), "{status}");
    rerun(&f, |c| {
        c.action = Action::RestoreState(file.clone());
        c.nor = Some(REGION.into());
    })
    .unwrap();
    assert_eq!(state_of(&f, REGION), state);
    assert_eq!(
        next_target(&tail_nvram(&f)),
        stela_loader::boot_selection::Target::SystemRecovery
    );

    // The boot log is read from the tail too.
    let error = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::BootLog).unwrap_err()
    );
    assert!(error.contains("the log region is blank"), "{error}");
    let log = crate::boot_log::tests::region(&["MACHINE STATE in the target's tail\n"]);
    f.host
        .write(REGION, tail() + stela_runtime::pc::boot_log::OFFSET, &log)
        .unwrap();
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(
        text.contains(&format!(
            "(disk1s2 from byte {:#x}): Stela boot log: 1 boots",
            tail()
        )),
        "{text}"
    );
    assert!(
        text.ends_with("=== STELA BOOT 1 ===\nMACHINE STATE in the target's tail\n"),
        "{text}"
    );
    assert!(
        f.host
            .read(REGION, 0, 1 << 20)
            .unwrap()
            .iter()
            .all(|&b| b == 0)
    );

    // A resized partition moves the tail: nothing is written there.
    f.host.partitions.get_mut(REGION).unwrap().0.bytes = SPLIT_BYTES - (1 << 20);
    let error = rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Macos))
        .unwrap_err()
        .to_string();
    assert!(error.contains("was resized"), "{error}");
    let error = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(error.contains("was resized"), "{error}");
    f.host.partitions.get_mut(REGION).unwrap().0.bytes = SPLIT_BYTES;
    assert_eq!(state_of(&f, REGION), state);

    let removed = uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert!(
        removed.contains("install \"macOS\": macOS, its NVRAM and SEP state stay on disk1s2"),
        "{removed}"
    );
    assert_eq!(state_of(&f, REGION), state);
}

fn split_refusal(f: &Fixture, configure: impl Fn(&mut Request)) -> String {
    let mut command = split_command(f);
    configure(&mut command);
    setup(&environment(&f.host), command)
        .unwrap_err()
        .to_string()
}

#[test]
fn tail_initialization_preserves_guest_region() {
    // A tail holding other bytes needs --initialize-nor, which writes a
    // fresh image there and leaves the SSD alone.
    let mut f = fixture(64 * GIB, false);
    add_split_disk(&mut f, false, &[]);
    f.host.write(REGION, tail(), b"not a NOR image").unwrap();
    let error = split_refusal(&f, |_| {});
    assert!(error.contains("disk1s2's machine state"), "{error}");
    assert!(error.contains("--initialize-nor"), "{error}");
    let mut initialize = split_command(&f);
    initialize.initialize_nor = true;
    setup(&environment(&f.host), initialize).unwrap();
    assert_eq!(
        next_target(&tail_nvram(&f)),
        stela_loader::boot_selection::Target::MacOs { safe: false }
    );
    assert!(
        f.host
            .read(REGION, 0, 1 << 20)
            .unwrap()
            .iter()
            .all(|&b| b == 0)
    );
}

#[test]
fn guest_gpt_must_fit_before_state_tail() {
    // A guest disk that spans the whole partition is not converted; one
    // inside the SSD, laid out by an earlier one-partition install, is
    // kept.
    let blocks = SPLIT_BYTES / 512;
    let mut f = fixture(64 * GIB, false);
    add_split_disk(&mut f, false, &guest_gpt(blocks - 1));
    let error = split_refusal(&f, |c| c.erase_target = true);
    assert!(error.contains("is not converted"), "{error}");
    assert!(error.contains("--nor"), "{error}");
    let mut f = fixture(64 * GIB, false);
    add_split_disk(&mut f, false, &guest_gpt(tail() / 512 - 1));
    let mut dry = split_command(&f);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(plan.contains("already holds a macOS disk"), "{plan}");
}

#[test]
fn state_tail_requires_guest_capacity() {
    // The SSD before the tail must still hold 60 GB: a 60 GB partition
    // falls short by its tail.
    let mut f = fixture(64 * GIB, false);
    add_split_disk(&mut f, false, &[]);
    f.host.partitions.get_mut(REGION).unwrap().0.bytes = 60_000_000_000;
    let error = split_refusal(&f, |_| {});
    assert!(error.contains("before the machine state"), "{error}");
    assert!(
        error.contains(
            "holds 59.9 GB before the machine state in its last bytes; macOS needs \
             at least 60.0 GB"
        ),
        "{error}"
    );
}

#[test]
fn separate_state_partition_is_not_converted() {
    // An install with a machine-state partition of its own keeps it: its
    // Apple region is not added again as a one-partition install.
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let before = snapshot(&f.esp);
    let error = rerun(&f, |c| {
        c.target = Some(REGION.into());
        c.install = Some("Again".into());
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("not converted"), "{error}");
    assert_eq!(snapshot(&f.esp), before);
    // Its separate partition still holds its machine state.
    let record = record(&f);
    assert_eq!(record.installs[0].disk.machine_state_offset, None);
    assert!(
        record
            .options
            .contains(&format!("--nor {SERIAL}:{NOR_GUID}")),
        "{}",
        record.options
    );

    // A further disk can be a one-partition install beside it.
    add_split_disk(&mut f, true, &[]);
    let added = rerun(&f, |c| {
        c.target = Some(REGION2.into());
        c.install = Some("Other".into());
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    assert!(
        added.contains("machine state  the tail of disk2s2"),
        "{added}"
    );
    let record = self::record(&f);
    assert_eq!(record.installs[1].disk.machine_state_offset, Some(tail()));
    let choices = Choices::parse(&fs::read_to_string(f.esp.join(CHOICES_PATH)).unwrap()).unwrap();
    let region2 = format!("{SERIAL2}:{REGION2_GUID}");
    assert!(
        choices.entries[2]
            .options
            .contains(&format!("--storage {region2} --nor {region2}")),
        "{}",
        choices.entries[2].options
    );
    // Its Install choice restores onto the region with the tail kept.
    assert_eq!(choices.entries[3].name, "Other - Install macOS");
    for entry in &choices.entries[3..4] {
        assert!(
            entry
                .options
                .contains(&format!("--storage {region2} --nor {region2} --bundle")),
            "{}",
            entry.options
        );
    }
}
#[test]
fn static_addresses_apply_to_all_boot_choices_until_made_automatic() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let mut request = command(&f, &f.esp);
    request.ipv4 = stela_boot_core::options::StaticIpv4::parse("192.168.1.20/24");
    request.ipv4_gateway = Some("192.168.1.1".parse().unwrap());
    request.dns = Some(vec!["1.1.1.1".parse().unwrap()]);
    let report = setup(&environment(&f.host), request).unwrap();
    assert!(
        report.contains("192.168.1.20/24 via 192.168.1.1, not from DHCP"),
        "{report}"
    );
    let options = || fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert!(
        options().contains("--ipv4 192.168.1.20/24 --ipv4-gateway 192.168.1.1 --dns 1.1.1.1"),
        "{}",
        options()
    );
    // A router outside the subnet is refused and changes nothing.
    assert!(
        rerun(&f, |c| {
            c.action = Action::Configure;
            c.ipv4 = stela_boot_core::options::StaticIpv4::parse("10.0.0.5/24");
            c.ipv4_gateway = Some("10.0.1.1".parse().unwrap());
        })
        .is_err()
    );
    assert!(
        options().contains("--ipv4 192.168.1.20/24"),
        "{}",
        options()
    );
    rerun(&f, |c| {
        c.action = Action::Configure;
        c.automatic_ipv4 = true;
        c.automatic_dns = true;
    })
    .unwrap();
    assert!(
        !options().contains("--ipv4") && !options().contains("--dns"),
        "{}",
        options()
    );
}

#[test]
fn usb4_approvals_apply_to_all_boot_choices_until_withdrawn() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let uuid = "01234567-89ab-cdef-0123-456789abcdef";
    let mut request = command(&f, &f.esp);
    request.usb4_approve = vec![uuid.into()];
    setup(&environment(&f.host), request).unwrap();
    let options = || fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert!(
        options().contains(&format!("--usb4-approve {uuid}")),
        "{}",
        options()
    );
    let report = rerun(&f, |c| {
        c.action = Action::Configure;
        c.no_usb4_approve = true;
    })
    .unwrap();
    assert!(report.contains("only with an IOMMU"), "{report}");
    assert!(!options().contains("--usb4-approve"), "{}", options());
}

#[test]
fn wifi_configuration_applies_to_all_boot_choices() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let mut joined = command(&f, &f.esp);
    joined.wifi = Some("Home Net".into());
    joined.wifi_country = Some("DE".into());
    joined.wifi_passphrase = Some("correct horse".into());
    let report = setup(&environment(&f.host), joined).unwrap();
    assert!(
        report.contains("joins \"Home Net\" under DE's rules"),
        "{report}"
    );
    let key = f.esp.join(wifi::KEY_PATH);
    let options = || fs::read_to_string(f.esp.join("EFI/stela/stela.cfg")).unwrap();
    assert_eq!(fs::read_to_string(&key).unwrap(), "correct horse");
    assert!(
        options().contains("--wifi Home%20Net --wifi-country DE"),
        "{}",
        options()
    );
    assert!(!options().contains("--wifi-key"), "{}", options());
    rerun(&f, |c| c.action = Action::Status).unwrap();
    // Off on the existing setup, and its key file goes too.
    rerun(&f, |c| {
        c.action = Action::Configure;
        c.no_wifi = true;
    })
    .unwrap();
    assert!(
        !key.exists() && !options().contains("--wifi"),
        "{}",
        options()
    );
    rerun(&f, |c| {
        c.action = Action::Configure;
        c.wifi = Some("Office".into());
        c.wifi_passphrase = Some("0123456789".into());
    })
    .unwrap();
    assert_eq!(fs::read_to_string(&key).unwrap(), "0123456789");
    assert!(options().contains("--wifi Office"), "{}", options());
    uninstall(&environment(&f.host), &f.esp, false).unwrap();
    assert!(!key.exists());
}

#[test]
fn legacy_operations_reject_oversized_firmware() {
    use sha2::{Digest, Sha256};
    use stela_loader::host_firmware::{DIRECTORY, MOUNT_CAPABILITY, Package};
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let legacy = f._dir.path().join("firmware-v1.efi");
    let mut marker = MOUNT_CAPABILITY;
    marker[20] = 1;
    let mut image = efi_image(b"firmware-v1");
    image.extend_from_slice(&marker);
    fs::write(&legacy, image).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(legacy.clone());
    })
    .unwrap();
    let source = f._dir.path().join("large-package");
    fs::create_dir(&source).unwrap();
    let bytes = vec![0x5a; 16 * 1024 * 1024 + 1];
    fs::write(source.join("data.bin"), &bytes).unwrap();
    fs::write(source.join("manifest.json"), serde_json::to_vec(&serde_json::json!({
        "schema":1,"images":[{"name":"fixture/large.bin","path":"data.bin",
        "length":bytes.len(),"sha256":hex::encode(Sha256::digest(&bytes)),
        "target":{"driver":"fixture","hardware":"synthetic","revision":"v1","board":null},
        "provenance":{"source":"synthetic","revision":"v1","member":"fixture/large.bin","license":"CC0-1.0"}}]
    })).unwrap()).unwrap();
    drop(bytes);
    let before = snapshot(&f.esp);
    for dry in [true, false] {
        let error = rerun(&f, |c| {
            c.action = Action::HostFirmwareInstall(source.clone());
            c.dry_run = dry;
        })
        .unwrap_err();
        assert!(error.to_string().contains("image limit"), "{error:#}");
        assert_eq!(snapshot(&f.esp), before);
    }
    let current = f._dir.path().join("firmware-v2.efi");
    let mut image = efi_image(b"firmware-v2");
    image.extend_from_slice(&per_pc_marker());
    fs::write(&current, image).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(current.clone());
    })
    .unwrap();
    rerun(&f, |c| {
        c.action = Action::HostFirmwareInstall(source.clone())
    })
    .unwrap();
    let loaded = Package::read(&f.esp.join(DIRECTORY)).unwrap();
    assert_eq!(loaded.inventory(), (1, 16 * 1024 * 1024 + 1));
    assert!(loaded.check_mount(&fs::read(&current).unwrap()).is_ok());
    assert!(loaded.check_mount(&fs::read(&legacy).unwrap()).is_err());
    let before = snapshot(&f.esp);
    let error = rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(legacy.clone());
    })
    .unwrap_err();
    assert!(
        error
            .to_string()
            .contains("cannot load installed host firmware"),
        "{error:#}"
    );
    assert_eq!(snapshot(&f.esp), before);

    // Uninstall keeps independently managed host firmware. A new setup
    // must check it during planning, even when there is no setup record.
    let firmware_before = snapshot(&f.esp.join(DIRECTORY));
    rerun(&f, |c| c.action = Action::Uninstall).unwrap();
    assert!(!f.esp.join(RECORD).exists());
    assert_eq!(snapshot(&f.esp.join(DIRECTORY)), firmware_before);
    let before = snapshot(&f.esp);
    for candidate in [&legacy, &f.efi] {
        for dry in [true, false] {
            let mut reinstall = command(&f, &f.esp);
            reinstall.efi = Some(candidate.clone());
            reinstall.dry_run = dry;
            let error = setup(&environment(&f.host), reinstall).unwrap_err();
            assert!(
                error
                    .to_string()
                    .contains("cannot load installed host firmware"),
                "{error:#}"
            );
            assert_eq!(snapshot(&f.esp), before);
        }
    }
    let mut reinstall = command(&f, &f.esp);
    reinstall.efi = Some(current.clone());
    reinstall.dry_run = true;
    setup(&environment(&f.host), reinstall).unwrap();
    assert_eq!(snapshot(&f.esp), before);
    let mut reinstall = command(&f, &f.esp);
    reinstall.efi = Some(current);
    setup(&environment(&f.host), reinstall).unwrap();
    assert!(record(&f).complete);
    assert_eq!(snapshot(&f.esp.join(DIRECTORY)), firmware_before);
}

#[test]
fn host_firmware_publication_preserves_guest_bundles() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let source = f._dir.path().join("firmware-package");
    fs::create_dir(&source).unwrap();
    fs::write(source.join("data.bin"), b"abc").unwrap();
    fs::write(source.join("manifest.json"), serde_json::to_vec(&serde_json::json!({
        "schema": 1, "images": [{"name":"Vendor/FW.bin", "path":"data.bin", "length":3,
        "sha256":"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        "target":{"driver":"fixture","hardware":"synthetic","revision":"v1","board":null},
        "provenance":{"source":"synthetic","revision":"v1","member":"fixtures/abc.bin","license":"CC0-1.0"}}]
    })).unwrap()).unwrap();
    let old = snapshot(&f.esp);
    let mut unsupported = command(&f, &f.esp);
    unsupported.action = Action::HostFirmwareInstall(source.clone());
    assert!(
        unsupported
            .dispatch(&environment(&f.host))
            .unwrap_err()
            .to_string()
            .contains("cannot load host firmware")
    );
    assert_eq!(snapshot(&f.esp), old);
    let aware = f._dir.path().join("firmware-aware.efi");
    let mut image = efi_image(b"firmware-aware");
    let mut legacy_marker = stela_loader::host_firmware::MOUNT_CAPABILITY;
    legacy_marker[20] = 1;
    image.extend_from_slice(&legacy_marker);
    fs::write(&aware, image).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(aware.clone());
    })
    .unwrap();
    let before = snapshot(&f.esp);
    let mut dry = command(&f, &f.esp);
    dry.action = Action::HostFirmwareInstall(source.clone());
    dry.dry_run = true;
    let report = dry.dispatch(&environment(&f.host)).unwrap();
    assert!(report.contains("1 verified resources, 3 bytes"));
    assert_eq!(snapshot(&f.esp), before);
    let mut write = command(&f, &f.esp);
    write.action = Action::HostFirmwareInstall(source.clone());
    write.dispatch(&environment(&f.host)).unwrap();
    let published = f.esp.join(stela_loader::host_firmware::MANIFEST_PATH);
    assert!(published.is_file());
    for (path, bytes) in &before {
        assert_eq!(&fs::read(path).unwrap(), bytes);
    }
    let active = fs::read(&published).unwrap();
    let firmware_before = snapshot(&f.esp.join(stela_loader::host_firmware::DIRECTORY));
    let new_efi = f._dir.path().join("firmware-test-update.efi");
    let mut image = efi_image(b"firmware-test-update");
    image.extend_from_slice(&per_pc_marker());
    fs::write(&new_efi, image).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(new_efi.clone());
    })
    .unwrap();
    assert_eq!(
        snapshot(&f.esp.join(stela_loader::host_firmware::DIRECTORY)),
        firmware_before
    );
    let new_os = f._dir.path().join("firmware-test-update.ipsw");
    restore::payload::synthetic_ipsw_of(&new_os, "STELA2").unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateOs;
        c.ipsw = Some(new_os.clone());
    })
    .unwrap();
    assert_eq!(
        snapshot(&f.esp.join(stela_loader::host_firmware::DIRECTORY)),
        firmware_before
    );
    rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert_eq!(
        snapshot(&f.esp.join(stela_loader::host_firmware::DIRECTORY)),
        firmware_before
    );
    let before_downgrade = snapshot(&f.esp);
    assert!(
        rerun(&f, |c| {
            c.action = Action::UpdateStela;
            c.efi = Some(f.efi.clone());
        })
        .unwrap_err()
        .to_string()
        .contains("cannot load installed host firmware")
    );
    assert_eq!(snapshot(&f.esp), before_downgrade);
    fs::write(source.join("data.bin"), b"abd").unwrap();
    let mut corrupt = command(&f, &f.esp);
    corrupt.action = Action::HostFirmwareInstall(source);
    assert!(corrupt.dispatch(&environment(&f.host)).is_err());
    assert_eq!(fs::read(published).unwrap(), active);
}

/// Every `stela` command `text` names in backticks is spelled as this
/// stela takes it, and names the stick only when several hold Stela.
fn assert_hints_name_the_usb(text: &str) {
    let several = SEVERAL.get();
    let mut hints = 0;
    for hint in text.split("`stela ").skip(1) {
        let hint = hint.split('`').next().unwrap();
        hints += 1;
        assert!(
            !hint.starts_with("setup") && !hint.contains("--usb"),
            "`stela {hint}` in:\n{text}"
        );
        let first = hint.split(' ').next().unwrap();
        assert!(
            [
                "create", "update", "repair", "status", "settings", "log", "nvram", "state"
            ]
            .contains(&first),
            "`stela {hint}` in:\n{text}"
        );
        assert_eq!(
            hint.contains("ESP"),
            several && hint != "create",
            "`stela {hint}` in:\n{text}"
        );
    }
    assert!(hints > 0, "no command in:\n{text}");
}

#[test]
fn interrupted_first_setup_status_names_the_command_that_finishes_it() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    interrupt_setup(&f);
    let error = rerun(&f, |c| c.action = Action::Status).unwrap_err();
    let reported = error
        .downcast_ref::<Reported>()
        .expect("status reports its problems");
    // The report goes to standard output; the error is one line.
    assert!(!reported.summary.contains('\n'), "{}", reported.summary);
    assert!(
        reported
            .summary
            .contains("1 problem; the report above names its fix"),
        "{}",
        reported.summary
    );
    let finish = format!("{} finishes it", fix(&f.esp, "repair"));
    assert!(reported.report.contains(&finish), "{}", reported.report);
    assert!(!reported.report.contains("then set up again"));
    assert_hints_name_the_usb(&reported.report);
    let (report, result) = split_report(Err(error));
    assert!(report.contains("was interrupted"), "{report}");
    let summary = result.unwrap_err().to_string();
    assert!(!summary.contains("was interrupted"), "{summary}");
    // Update names the same fix, and repair finishes the setup as it
    // began: its build, model, target and machine state partition.
    let refusal = rerun(&f, |c| c.action = Action::UpdateStela)
        .unwrap_err()
        .to_string();
    assert!(refusal.contains(&finish), "{refusal}");
    rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    let finished = record(&f);
    assert!(finished.complete);
    assert_eq!(finished.installs[0].disk.apple_region, REGION);
    rerun(&f, |c| c.action = Action::Status).unwrap();
}

#[test]
fn every_maintenance_hint_names_the_usb() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    rerun(&f, |c| {
        c.target = Some(REGION2.into());
        c.nor = Some(NOR2.into());
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    // A missing bundle and foreign machine state each name their fix.
    let mut stopped = record(&f);
    let second = stopped.installs[1].ecid;
    stopped.bundles.retain(|b| b.ecid != second);
    write_record(&f.esp, &stopped).unwrap();
    f.host.write(NOR2, 0, &[0xa5; 4096]).unwrap();
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(problems.contains("`stela repair`"), "{problems}");
    assert_hints_name_the_usb(&problems);
    let report = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(report.contains("--initialize-nor"), "{report}");
    assert_hints_name_the_usb(&report);
    // With several sticks that hold Stela, each command names its stick.
    SEVERAL.set(true);
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    SEVERAL.set(false);
    assert!(
        problems.contains(&format!("`stela repair {}`", f.esp.display())),
        "{problems}"
    );
    SEVERAL.set(true);
    assert_hints_name_the_usb(&problems);
    // A stick named on the command line is named as it was typed.
    NAMED.set(Some("/dev/sdb".into()));
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    NAMED.set(None);
    SEVERAL.set(false);
    assert!(problems.contains("`stela repair /dev/sdb`"), "{problems}");
    assert!(
        !problems.contains(&format!("repair {}", f.esp.display())),
        "{problems}"
    );
}

/// Move the partition `from` to `to` on disk `disk`, as a PC that numbers
/// its disks differently after a reboot does.
fn rename_partition(f: &mut Fixture, from: &str, to: &str, disk: &str) {
    let (mut info, path) = f.host.partitions.remove(from).unwrap();
    info.identifier = to.into();
    info.disk = disk.into();
    f.host.partitions.insert(to.into(), (info, path));
    let serial = f.host.serials.get("disk1").unwrap().clone();
    f.host.serials.insert(disk.into(), serial);
    if !f.host.disks.iter().any(|d| d.identifier == disk) {
        let apfs = ("Apple_APFS", None);
        let listed = self::disk(disk, &[("EFI", None), apfs, apfs]);
        f.host.disks.push(listed);
    }
}

#[test]
fn a_renamed_partition_is_found_by_its_guid_and_recorded_by_repair() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    rename_partition(&mut f, REGION, "disk3s2", "disk3");
    rename_partition(&mut f, NOR, "disk3s3", "disk3");
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("was disk1s2; `repair` records the new name"),
        "{status}"
    );
    // NVRAM is still reached through the GUID before repair records it.
    rerun(&f, |c| c.action = Action::NextBoot(NextBoot::Recovery)).unwrap();
    let repaired = rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(
        repaired.contains("install \"macOS\": disk1s2 is now disk3s2"),
        "{repaired}"
    );
    let disk = &record(&f).installs[0].disk;
    assert_eq!(
        (disk.apple_region.as_str(), disk.nor.as_str()),
        ("disk3s2", "disk3s3")
    );
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(!status.contains("records the new name"), "{status}");
}

#[test]
fn repair_with_a_detached_disk_still_repairs_the_usb() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    fs::remove_file(f.esp.join(efi::OPTIONS)).unwrap();
    // The PC's disk is not connected to this computer.
    f.host.partitions.clear();
    let report = rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(
        report.contains("not connected to this computer"),
        "{report}"
    );
    assert!(f.esp.join(efi::OPTIONS).exists());
}

#[test]
fn an_interrupted_disk_add_is_finished_by_the_same_command() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    add_disk(&mut f, true, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let add = |f: &Fixture| {
        rerun(f, |c| {
            c.target = Some(REGION2.into());
            c.nor = Some(NOR2.into());
            c.ipsw = Some(f.ipsw.clone());
        })
    };
    add(&f).unwrap();
    let done = record(&f);
    // The add stopped after recording the install, before its bundle and
    // machine state.
    let mut stopped = done.clone();
    let second = stopped.installs[1].ecid;
    stopped.bundles.retain(|b| b.ecid != second);
    write_record(&f.esp, &stopped).unwrap();
    f.host.write(NOR2, 0, &vec![0; IMAGE_BYTES]).unwrap();
    let finished = add(&f).unwrap();
    assert!(
        finished.contains("finishes adding install \"macOS 2\", which was interrupted"),
        "{finished}"
    );
    let resumed = record(&f);
    assert_eq!(resumed.installs, done.installs);
    assert!(resumed.bundles.iter().any(|b| b.ecid == second));
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("No problems found."), "{status}");
    // A finished install is not added again.
    let again = add(&f).unwrap_err().to_string();
    assert!(again.contains("already belongs"), "{again}");
}

#[test]
fn a_damaged_boot_log_is_not_taken_for_a_machine_that_never_booted() {
    use stela_runtime::pc::boot_log::OFFSET;
    use stela_soc_data::nor::CAPACITY;
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    // A foreign SEP area, and a log header that is not blank but invalid.
    let foreign = vec![0xa5; CAPACITY + stela_formats::machine_state::SEP_BYTES as usize];
    f.host.write(NOR, 0, &foreign).unwrap();
    f.host.write(NOR, OFFSET, &[0x5a; 512]).unwrap();
    let problems = rerun(&f, |c| c.action = Action::Status)
        .unwrap_err()
        .to_string();
    assert!(!problems.contains("never booted"), "{problems}");
    let error = rerun(&f, |c| {
        c.action = Action::Repair;
        c.initialize_nor = true;
    })
    .unwrap_err()
    .to_string();
    assert!(error.contains("may hold its keys"), "{error}");
    assert_eq!(f.host.read(NOR, 0, foreign.len()).unwrap(), foreign);
}

#[test]
fn a_usb_made_without_an_nvidia_gpu_needs_no_firmware_step() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    // The plan of a stela.efi that reads the bundle names the whole bundle,
    // the same with or without a GPU on this computer.
    let bundled = f._dir.path().join("bundled.efi");
    let mut image = efi_image(b"bundled");
    image.extend_from_slice(&stela_loader::host_firmware::MOUNT_CAPABILITY);
    fs::write(&bundled, image).unwrap();
    let mut lines = Vec::new();
    for gpus in [vec![], vec![0x2c02]] {
        f.host.nvidia_gpus.replace(gpus);
        let mut dry = installed_command(&f);
        dry.efi = Some(bundled.clone());
        dry.dry_run = true;
        let plan = setup(&environment(&f.host), dry).unwrap();
        let line = plan
            .lines()
            .find(|line| line.starts_with("  firmware       "))
            .unwrap_or_else(|| panic!("{plan}"))
            .to_string();
        lines.push(line);
    }
    assert_eq!(lines[0], lines[1]);
    assert!(lines[0].contains("86 files, 115.8 MiB"), "{}", lines[0]);
    f.host.nvidia_gpus.borrow_mut().clear();
    let report = setup(&environment(&f.host), installed_command(&f)).unwrap();
    assert!(!report.contains("NVIDIA"), "{report}");
    assert!(!report.contains("host-firmware"), "{report}");
}

#[test]
fn repair_resumes_an_interrupted_setup_with_its_recorded_inputs() {
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    interrupt_setup(&f);
    let found = read_record(&environment(&f.host), &f.esp).unwrap();
    assert!(!found.complete, "an interrupted setup");
    let esp = f.esp.clone();
    // Resuming names the mounted volume, not the whole disk, and the
    // recorded inputs.
    let mut command = command(&f, Path::new("disk5"));
    command.ipsw = None;
    command.install_image = Some(InstallImage::Network);
    command.resume_from(esp, &found);
    assert_eq!(command.usb, f.esp);
    assert!(command.fetch);
    // The full installer it was making goes on.
    assert_eq!(command.install_image, Some(InstallImage::Full));
    assert!(
        found
            .resume_summary()
            .starts_with("continuing the Offline installer: "),
        "{}",
        found.resume_summary()
    );
    assert_eq!(command.build, found.build);
    assert_eq!(command.model.as_deref(), Some(found.model.as_str()));
}

#[test]
fn status_and_repair_name_the_firmware_the_last_boot_asked_for() {
    use stela_loader::host_firmware::{DIRECTORY, MOUNT_CAPABILITY, requirements::LOG_LINE};
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let aware = f._dir.path().join("firmware-aware.efi");
    let mut image = efi_image(b"firmware-aware");
    // A stela.efi before the bundle, which reads exact per-PC files.
    let mut marker = MOUNT_CAPABILITY;
    marker[20] = 3;
    image.extend_from_slice(&marker);
    fs::write(&aware, image).unwrap();
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.efi = Some(aware.clone());
    })
    .unwrap();
    rerun(&f, |c| c.action = Action::Status).unwrap();
    let log = |requests: serde_json::Value| {
        let line = format!(
            "[    4.000] {LOG_LINE}{}\n",
            serde_json::json!({"schema":1,"complete":true,"requirements":requests})
        );
        fs::write(
            f.esp.join(efi::BOOT_LOG),
            crate::boot_log::tests::region(&[&line]),
        )
        .unwrap();
    };
    // A file Stela has no source for is named and fails nothing.
    log(serde_json::json!([{"name":"brcm/synthetic.bin","target":{
        "driver":"synthetic-bluetooth","hardware":"pci:14e4:5fa0","revision":"c0","board":null},
        "minimum_bytes":1}]));
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("asked for brcm/synthetic.bin (synthetic-bluetooth"),
        "{status}"
    );
    let report = rerun(&f, |c| c.action = Action::Repair).unwrap();
    assert!(
        report.contains("asked for brcm/synthetic.bin (synthetic-bluetooth pci:14e4:5fa0 c0), which Stela has no pinned source for"),
        "{report}"
    );
    assert!(!f.esp.join(DIRECTORY).exists());
    // A file setup can add is a problem status names with its fix, and a
    // dry run of repair plans it without a download or a write.
    log(
        serde_json::json!([{"name":"rtl_nic/rtl8168d-1.fw","target":{
        "driver":"realtek","hardware":"10ec:8168","revision":"xid-281","board":null},
        "minimum_bytes":4}]),
    );
    let error = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::Status).unwrap_err()
    );
    assert!(
        error.contains("asked for firmware rtl_nic/rtl8168d-1.fw (realtek 10ec:8168 xid-281)"),
        "{error}"
    );
    assert!(error.contains("`stela repair`"), "{error}");
    let before = snapshot(&f.esp);
    let report = rerun(&f, |c| {
        c.action = Action::Repair;
        c.dry_run = true;
    })
    .unwrap();
    assert!(
        report.contains("firmware       add rtl_nic/rtl8168d-1.fw"),
        "{report}"
    );
    assert!(report.contains("Dry run"), "{report}");
    assert_eq!(snapshot(&f.esp), before);
}

/// `boot prepare --restore` whose bundle holds Apple's answers to every
/// signing request of its install, as the AP ticket's step leaves it.
fn staged_prepare_restore(options: prepare::RamdiskOptions<'_>) -> Result<()> {
    use stela_runtime::restore::tss::staged::{FILE, Kind, Set};
    let output = options.output.to_path_buf();
    fake_prepare_restore(options)?;
    let mut set = Set::default();
    for (kind, body) in Kind::ALL.into_iter().zip([b"a", b"b", b"c"]) {
        set.insert(kind, body, b"answer".to_vec())
            .map_err(|error| anyhow::anyhow!("{error}"))?;
    }
    set.policy_nonce = Some([1; 32]);
    let bytes = set.encode().map_err(|error| anyhow::anyhow!("{error}"))?;
    fs::write(output.join(FILE), bytes)?;
    Ok(())
}

#[test]
fn a_full_installer_with_every_signing_answer_says_the_install_needs_no_network() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let staged = Environment {
        prepare_restore: &staged_prepare_restore,
        ..environment(&f.host)
    };
    let mut dry = command(&f, &f.esp);
    dry.dry_run = true;
    let plan = setup(&staged, dry).unwrap();
    assert!(plan.contains(create::OFFLINE_NOTE), "{plan}");
    // What is not known is said too.
    assert!(plan.contains(create::FIRST_START_NOTE), "{plan}");
    assert!(!plan.contains(create::OFFLINE_SIGNING_NOTE), "{plan}");
    // Without the answers the same USB's install needs the network.
    let mut dry = command(&f, &f.esp);
    dry.dry_run = true;
    let plan = setup(&environment(&f.host), dry).unwrap();
    assert!(!plan.contains(create::OFFLINE_NOTE), "{plan}");
    assert!(plan.contains(create::OFFLINE_SIGNING_NOTE), "{plan}");
    // The written USB's status says the same.
    setup(&staged, command(&f, &f.esp)).unwrap();
    let mut status = command(&f, &f.esp);
    status.ipsw = None;
    status.model = None;
    status.action = Action::Status;
    let report = status.dispatch(&staged).unwrap();
    assert!(report.contains(create::OFFLINE_NOTE), "{report}");
}

#[test]
fn offline_is_made_on_a_usb_of_32_gb_or_more_from_an_image_with_all_of_macos() {
    // Sticks sold as 32 GB: 28.8 GB and 30.0 GB are common capacities.
    for capacity in [0, 28_800_000_000, 30_015_488_000, 31_000_000_000, 64 * GIB] {
        assert_eq!(offline_unavailable(capacity, true), None, "{capacity}");
    }
    assert_eq!(
        offline_unavailable(16_000_000_000, true).as_deref(),
        Some("needs a USB stick of 32 GB or more (this one is 16.0 GB)")
    );
    assert!(offline_unavailable(8_000_000_000, true).is_some());
    let startup_files = offline_unavailable(64 * GIB, false).unwrap();
    assert!(
        startup_files.contains("only its startup files"),
        "{startup_files}"
    );
}

#[test]
fn a_new_setup_makes_the_installer_asked_for_or_else_offline_where_it_fits() {
    let f = fixture(64 * GIB, false);
    let recovery = f._dir.path().join("Recovery.ipsw");
    restore::payload::trim(
        fs::File::open(&f.ipsw).unwrap(),
        restore::payload::SYNTHETIC_MODEL,
        &recovery,
        "Copying boot files",
    )
    .unwrap();
    let choose = |asked, capacity, ipsw: Option<&Path>| {
        create::choose_installer(asked, capacity, ipsw).map_err(|e| e.to_string())
    };
    let (full, network) = (Some(InstallImage::Full), Some(InstallImage::Network));
    // Both on a large USB; Offline by default.
    assert_eq!(choose(None, 64 * GIB, None), Ok(InstallImage::Full));
    assert_eq!(choose(full, 64 * GIB, None), Ok(InstallImage::Full));
    assert_eq!(choose(network, 64 * GIB, None), Ok(InstallImage::Network));
    // Online alone on a small USB, and Offline asked for there is refused.
    assert_eq!(
        choose(None, 16_000_000_000, None),
        Ok(InstallImage::Network)
    );
    assert_eq!(
        choose(network, 16_000_000_000, None),
        Ok(InstallImage::Network)
    );
    assert_eq!(
        choose(full, 16_000_000_000, None),
        Err(
            "Offline needs a USB stick of 32 GB or more (this one is 16.0 GB). Use --installer \
             online, or a larger USB"
                .into()
        )
    );
    // A local IPSW with the startup files alone makes Online only.
    assert_eq!(
        choose(None, 64 * GIB, Some(&recovery)),
        Ok(InstallImage::Network)
    );
    let refused = choose(full, 64 * GIB, Some(&recovery)).unwrap_err();
    assert!(
        refused.starts_with("Offline needs an IPSW file with all of macOS"),
        "{refused}"
    );
    assert_eq!(
        choose(full, 64 * GIB, Some(&f.ipsw)),
        Ok(InstallImage::Full)
    );
}

#[test]
fn installer_flag_offline_on_a_small_usb_is_refused_before_anything_changes() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    for disk in f.host.disks.iter_mut().filter(|d| d.identifier == "disk5") {
        disk.bytes = 16_000_000_000;
    }
    let volume = stela_volume(&f);
    let parse = |kind: &str| {
        let mut request =
            crate::tests::parse(&["create", volume.to_str().unwrap(), "--installer", kind])
                .unwrap();
        assert!(matches!(request.action, Action::Create));
        (request.ipsw, request.efi) = (Some(f.ipsw.clone()), Some(f.efi.clone()));
        request.fetch = false;
        request.model = Some(restore::payload::SYNTHETIC_MODEL.into());
        request.build = restore::payload::SYNTHETIC_BUILD.into();
        request.translation_store_bytes = Some(translations::MIN_PACK_BYTES);
        request
    };
    assert_eq!(parse("offline").install_image, Some(InstallImage::Full));
    assert_eq!(parse("online").install_image, Some(InstallImage::Network));
    let before = (snapshot(&f.esp), snapshot(&volume));
    let error = setup(&environment(&f.host), parse("offline"))
        .unwrap_err()
        .to_string();
    assert!(
        error.starts_with("Offline needs a USB stick of 32 GB or more (this one is 16.0 GB)"),
        "{error}"
    );
    assert_eq!((snapshot(&f.esp), snapshot(&volume)), before);
    let mut online = parse("online");
    online.dry_run = true;
    let report = setup(&environment(&f.host), online).unwrap();
    assert!(report.contains("installer      Online: "), "{report}");
}

#[test]
fn a_usb_named_by_its_mounted_volume_gets_the_installer_its_size_allows() {
    for (bytes, full) in [(16_000_000_000, false), (32_000_000_000, true)] {
        let mut f = fixture(64 * GIB, false);
        add_stela(&mut f);
        for disk in f.host.disks.iter_mut().filter(|d| d.identifier == "disk5") {
            disk.bytes = bytes;
        }
        let volume = stela_volume(&f);
        let mut dry = command(&f, &volume);
        dry.install_image = None;
        dry.dry_run = true;
        let plan = setup(&environment(&f.host), dry).unwrap();
        assert_eq!(
            plan.contains("installer      Offline: "),
            full,
            "{bytes}: {plan}"
        );
        assert_eq!(
            plan.contains("installer      Online: "),
            !full,
            "{bytes}: {plan}"
        );
    }
}

#[test]
fn the_offline_promise_says_the_install_checks_the_answers_first() {
    // Setup cannot run the install's dry run (it needs the install's own
    // nonce and machine), so the plan and status say the check.
    assert!(create::OFFLINE_NOTE.contains("only if they do not fit"));
    for message in [
        create::OFFLINE_NOTE,
        create::OFFLINE_SIGNING_NOTE,
        create::ONLINE_NOTE,
    ] {
        assert!(
            message.starts_with("Offline: ") || message.starts_with("Online: "),
            "{message}"
        );
    }
}

#[test]
fn boot_log_report_goes_to_the_output_file() {
    let dir = tempfile::tempdir().unwrap();
    let output = dir.path().join("boot-log.txt");
    let request =
        crate::tests::parse(&["log", "/Volumes/EFI", "-o", output.to_str().unwrap()]).unwrap();
    assert!(matches!(request.action, Action::BootLog));
    assert_eq!(request.output.as_deref(), Some(output.as_path()));
    save_report(&output, "=== STELA BOOT 1 ===\nline\n").unwrap();
    assert_eq!(
        fs::read_to_string(&output).unwrap(),
        "=== STELA BOOT 1 ===\nline\n"
    );
    assert!(save_report(&dir.path().join("no/such/file"), "").is_err());
}

#[test]
fn boot_log_without_a_usb_reads_every_disk() {
    let parse = |words: &[&str]| {
        crate::tests::parse(
            &std::iter::once("log")
                .chain(words.iter().copied())
                .collect::<Vec<_>>(),
        )
    };
    let request = parse(&["-o", "log.txt"]).unwrap();
    assert!(matches!(request.action, Action::BootLog));
    assert!(request.usb.as_os_str().is_empty());
    assert_eq!(request.output.as_deref(), Some(Path::new("log.txt")));
    // An install is chosen on a setup's stick only.
    assert!(parse(&["--install", "macOS"]).is_err());
    let request = parse(&["E:\\", "--install", "macOS"]).unwrap();
    assert_eq!(request.usb, Path::new("E:\\"));
    assert_eq!(request.install.as_deref(), Some("macOS"));
}

/// Partition `identifier` of its disk, backed by an image file holding
/// `bytes`.
fn raw_partition(host: &mut FakeHost, dir: &Path, identifier: &str, bytes: &[u8]) {
    let path = dir.join(format!("{identifier}.img"));
    fs::write(&path, bytes).unwrap();
    let info = PartitionInfo {
        identifier: identifier.into(),
        disk: identifier[..5].into(),
        guid: String::new(),
        bytes: bytes.len() as u64,
        content: String::new(),
        mount: None,
        container: None,
    };
    host.partitions.insert(identifier.into(), (info, path));
}

#[test]
fn boot_log_scan_reads_the_usb_raw_before_the_system_disk() {
    use crate::boot_log::tests::{fat_volume, region};
    use stela_runtime::pc::boot_log::{OFFSET, PARTITION_BYTES};
    let dir = tempfile::tempdir().unwrap();
    let mounted = dir.path().join("STELA");
    let mut host = FakeHost::new(
        BTreeMap::new(),
        vec![
            disk("disk0", &[("Apple_APFS", None)]),
            disk("disk5", &[("EFI", None), ("Apple_HFS", Some(&mounted))]),
        ],
    );
    let mut state = vec![0u8; PARTITION_BYTES as usize];
    state[OFFSET as usize..].copy_from_slice(&region(&["NOR persistent\n"]));
    raw_partition(&mut host, dir.path(), "disk0s1", &state);
    let usb_log = region(&["CHOICE 3 (Install macOS)\n"]);
    raw_partition(
        &mut host,
        dir.path(),
        "disk5s1",
        &fat_volume(Some(&usb_log)),
    );
    // Its EFI partition is not mounted: the log is read through the disk.
    let logs = crate::boot_log::scan(&host).unwrap();
    let device = |number: u32| {
        if cfg!(windows) {
            format!("PhysicalDrive{number}")
        } else {
            format!("/dev/disk{number}")
        }
    };
    let usb = format!(
        "Disk disk5 (137.4 GB, {}, {})",
        device(5),
        mounted.display()
    );
    assert_eq!(
        logs.len(),
        1,
        "the system disk waits while the USB has a log"
    );
    assert_eq!(
        logs[0].0,
        format!("{usb}: partition disk5s1, EFI/stela/boot.log")
    );
    assert_eq!(logs[0].1.boots, 1);
    // A USB without a log: the system disk is read too.
    raw_partition(&mut host, dir.path(), "disk5s1", &fat_volume(None));
    let logs = crate::boot_log::scan(&host).unwrap();
    assert_eq!(logs.len(), 1);
    assert_eq!(
        logs[0].0,
        format!("Disk disk0 (137.4 GB, {}): partition disk0s1", device(0))
    );
    // No log anywhere names the disks, the ones that could not be read,
    // and what to try.
    fs::write(host.image("disk0s1").unwrap(), vec![0u8; 4096]).unwrap();
    host.partitions.get_mut("disk5s1").unwrap().1 = dir.path().join("gone.img");
    let error = format!("{:#}", crate::boot_log::scan(&host).unwrap_err());
    assert!(
        error.starts_with("no Stela boot log on this computer's disks."),
        "{error}"
    );
    assert!(
        error.contains(&format!("Looked at:\n  {usb}\n  Disk disk0")),
        "{error}"
    );
    assert!(
        error.contains(&format!("Could not read:\n  {usb}: ")),
        "{error}"
    );
    assert!(
        error.ends_with(&format!(
            "Plug in the USB Stela booted from and run this again {} {}.",
            if cfg!(windows) { "from" } else { "with" },
            host::ADMIN
        )),
        "{error}"
    );
}

#[test]
fn setup_boot_log_reads_the_usb_raw_when_its_folder_cannot_be_read() {
    use crate::boot_log::tests::{fat_volume, region};
    let mut f = fixture(64 * GIB, false);
    add_installed_disk(&mut f, &[], &[]);
    setup(&environment(&f.host), installed_command(&f)).unwrap();
    let dir = f._dir.path().to_path_buf();
    let usb_log = region(&["CHOICE 3 (Install macOS)\n"]);
    raw_partition(&mut f.host, &dir, "disk5s1", &fat_volume(Some(&usb_log)));
    // The folder holds no log file the host lets setup read.
    fs::remove_file(f.esp.join(efi::BOOT_LOG)).unwrap();
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(
        text.starts_with("Boot log on the USB (Disk disk5 (137.4 GB, "),
        "{text}"
    );
    assert!(
        text.contains(
            "partition disk5s1, EFI/stela/boot.log, read directly): Stela boot log: 1 boots"
        ),
        "{text}"
    );
    assert!(text.contains("CHOICE 3 (Install macOS)\n"), "{text}");
    // Without the record, the whole disk is read.
    fs::remove_file(f.esp.join(RECORD)).unwrap();
    let text = rerun(&f, |c| c.action = Action::BootLog).unwrap();
    assert!(text.starts_with("The setup on "), "{text}");
    assert!(
        text.contains("so its disk was read directly.\n\n"),
        "{text}"
    );
    assert!(
        text.ends_with("=== STELA BOOT 1 ===\nCHOICE 3 (Install macOS)\n"),
        "{text}"
    );
    // Other actions still need the record.
    assert!(rerun(&f, |c| c.action = Action::Status).is_err());
    // A USB without a log on it names both reasons.
    raw_partition(&mut f.host, &dir, "disk5s1", &fat_volume(None));
    let error = format!(
        "{:#}",
        rerun(&f, |c| c.action = Action::BootLog).unwrap_err()
    );
    assert!(error.contains("holds no Stela setup"), "{error}");
    assert!(error.contains("nor does Disk disk5"), "{error}");
}

#[test]
fn without_usb_the_one_disk_holding_a_setup_is_used_and_two_are_never_guessed() {
    let dir = tempfile::tempdir().unwrap();
    let volume = |name: &str, setup: Option<&str>| {
        let path = dir.path().join(name);
        fs::create_dir_all(&path).unwrap();
        if let Some(inner) = setup {
            fs::create_dir_all(path.join(inner).parent().unwrap()).unwrap();
            if inner == RECORD {
                fs::write(path.join(inner), b"{}").unwrap();
            } else {
                fs::create_dir_all(path.join(inner)).unwrap();
            }
        }
        path
    };
    let system = volume("system", None);
    let esp = volume("esp", Some(RECORD));
    let data = volume("data", Some("stela/boot"));
    let other = volume("other", Some("stela/boot"));
    let host = |disks: Vec<Disk>| FakeHost::new(BTreeMap::new(), disks);
    // None: the error says how to name one.
    let error = choose_setup(
        &host(vec![disk("disk0", &[("Linux", Some(&system))])]),
        "status",
    )
    .unwrap_err()
    .to_string();
    assert!(error.contains("no mounted volume"), "{error}");
    assert!(error.contains("`stela create`"), "{error}");
    // One: its EFI partition, which holds the record, over its data volume.
    let one = host(vec![
        disk("disk0", &[("Linux", Some(&system))]),
        disk(
            "disk5",
            &[("Microsoft Basic Data", Some(&data)), ("EFI", Some(&esp))],
        ),
    ]);
    assert_eq!(choose_setup(&one, "status").unwrap().0, esp);
    // A USB whose EFI partition is not mounted is found by its data volume.
    let data_only = host(vec![disk(
        "disk5",
        &[("EFI", None), ("Microsoft Basic Data", Some(&data))],
    )]);
    assert_eq!(choose_setup(&data_only, "status").unwrap().0, data);
    // Two: both are listed and neither is used.
    let two = host(vec![
        disk("disk5", &[("EFI", Some(&esp))]),
        disk("disk6", &[("Microsoft Basic Data", Some(&other))]),
    ]);
    let error = choose_setup(&two, "settings").unwrap_err().to_string();
    assert!(error.starts_with("2 sticks hold Stela"), "{error}");
    // Each is named by its whole disk, never by a volume path, which may
    // hold the user's name.
    assert!(error.contains("disk5 (Disk disk5)"), "{error}");
    assert!(error.contains("disk6 (Disk disk6)"), "{error}");
    assert!(!error.contains(&esp.display().to_string()), "{error}");
    assert!(
        error.contains(&format!(
            "such as `stela settings {}disk5`",
            host::DISK_PREFIX
        )),
        "{error}"
    );
    assert_eq!(command_words(&Action::UpdateOs), "update");
}

#[test]
fn maintenance_commands_parse_without_usb() {
    for args in [
        &["status"][..],
        &["repair"],
        &["update"],
        &["update", "--macos", "latest"],
        &["settings"],
        &["uninstall"],
        &["nvram", "reset"],
        &["state", "backup", "state.bin"],
    ] {
        let request = crate::tests::parse(args).unwrap_or_else(|e| panic!("{args:?}: {e}"));
        assert!(request.usb.as_os_str().is_empty(), "{args:?}");
        assert!(finds_usb(&request.action), "{args:?}");
    }
    // Create finds a stick to erase, and log reads every disk.
    for args in [&["create"][..], &["log"]] {
        let request = crate::tests::parse(args).unwrap();
        assert!(!finds_usb(&request.action), "{args:?}");
    }
}

#[test]
fn a_stick_named_by_its_whole_disk_is_read_through_its_stela_volume() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    // The whole disk and its device path name the volume that holds the
    // record; a volume names itself.
    assert_eq!(stick_volume(&f.host, Path::new("disk5")).unwrap(), f.esp);
    assert_eq!(
        stick_volume(&f.host, Path::new("/dev/disk5")).unwrap(),
        f.esp
    );
    assert_eq!(stick_volume(&f.host, &f.data).unwrap(), f.data);
    // A disk without Stela says how to make an installer on it.
    let error = stick_volume(&f.host, Path::new("disk0"))
        .unwrap_err()
        .to_string();
    assert!(error.contains("`stela create disk0`"), "{error}");
}

/// A setup whose record says Stela `release` (none before 0.4) made it.
fn made_by(release: Option<&str>) -> Fixture {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let mut made = record(&f);
    assert_eq!(made.release.as_deref(), Some(release::CURRENT));
    made.release = release.map(Into::into);
    record::write_record(&f.esp, &made).unwrap();
    f
}

#[test]
fn a_stick_records_its_stela_version_and_an_update_brings_an_older_one_to_this_one() {
    let f = made_by(None);
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("Stela 0.3 or older; this stela is"),
        "{status}"
    );
    rerun(&f, |c| c.action = Action::UpdateStela).unwrap();
    assert_eq!(record(&f).release.as_deref(), Some(release::CURRENT));
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains(&format!("Stela {}\n", release::CURRENT)),
        "{status}"
    );
}

#[test]
fn an_older_stela_asks_before_it_changes_a_newer_stick_and_a_no_changes_nothing() {
    let f = made_by(Some("99.0.0"));
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(
        status.contains("Stela 99.0.0, newer than this stela"),
        "{status}"
    );
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let unchanged = || {
        assert_eq!(
            (snapshot(&f.esp), snapshot(&stela_volume(&f))),
            before,
            "the stick changed"
        )
    };
    for action in [
        || Action::UpdateStela,
        || Action::Repair,
        || Action::Configure,
    ] {
        let action = || {
            let action = action();
            let configure = matches!(action, Action::Configure);
            (action, configure.then_some(command::Wait::Seconds(3)))
        };
        // Without a terminal: refused, naming --yes.
        let error = rerun(&f, |c| (c.action, c.chooser_timeout) = action())
            .unwrap_err()
            .to_string();
        assert!(
            error.contains("newer than this stela") && error.contains("Add --yes"),
            "{error}"
        );
        unchanged();
        // Asked on a terminal, and declined.
        let no = ui::tests::Scripted::new(&["no"]);
        let mut request = command(&f, &f.esp);
        (request.ipsw, request.model) = (None, None);
        (request.action, request.chooser_timeout) = action();
        let environment = Environment {
            ui: &no,
            ..environment(&f.host)
        };
        let error = request.dispatch(&environment).unwrap_err().to_string();
        assert_eq!(error, "nothing was changed");
        assert!(
            no.transcript()
                .contains("Stela 99.0.0, newer than this stela")
        );
        unchanged();
    }
    // Finishing a macOS switch the newer Stela began asks too.
    let mut switching = record(&f);
    switching.replacing = Some("STELA2".into());
    record::write_record(&f.esp, &switching).unwrap();
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let error = rerun(&f, |c| c.action = Action::Repair)
        .unwrap_err()
        .to_string();
    assert!(
        error.contains("newer than this stela") && error.contains("Add --yes"),
        "{error}"
    );
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    let mut done = record(&f);
    done.replacing = None;
    record::write_record(&f.esp, &done).unwrap();
    // --yes puts this Stela on it.
    rerun(&f, |c| {
        c.action = Action::UpdateStela;
        c.yes = true;
    })
    .unwrap();
    assert_eq!(record(&f).release.as_deref(), Some(release::CURRENT));
}

#[test]
fn repair_finishes_an_interrupted_create_with_its_wifi_and_asks_nothing() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    let passphrase = f._dir.path().join("passphrase");
    fs::write(&passphrase, "correct horse").unwrap();
    let mut online = command(&f, &f.esp);
    online.install_image = Some(InstallImage::Network);
    online.wifi = Some("Home".into());
    online.wifi_passphrase_file = Some(passphrase);
    setup(&environment(&f.host), online).unwrap();
    let mut stopped = record(&f);
    stopped.complete = false;
    write_record(&f.esp, &stopped).unwrap();
    for path in [efi::LOADER, efi::OPTIONS, CHOICES_PATH] {
        fs::remove_file(f.esp.join(path)).unwrap();
    }
    // No terminal and no passphrase file: the key the create wrote serves.
    let report = rerun(&f, |c| {
        c.action = Action::Repair;
        c.ipsw = Some(f.ipsw.clone());
    })
    .unwrap();
    assert!(
        report.contains("continuing the Online installer"),
        "{report}"
    );
    let finished = record(&f);
    assert!(finished.complete);
    assert_eq!(finished.wifi.unwrap().ssid, "Home");
    assert_eq!(
        fs::read_to_string(f.esp.join(wifi::KEY_PATH))
            .unwrap()
            .trim(),
        "correct horse"
    );
    rerun(&f, |c| c.action = Action::Status).unwrap();
}

#[test]
fn repair_asks_before_downloading_more_than_a_gigabyte_and_a_no_changes_nothing() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    let medium = record(&f).active_medium().cloned().unwrap();
    let first = medium
        .directory()
        .join(stela_formats::payload::piece_name(0));
    let piece = fs::read(&first).unwrap();
    fs::write(&first, &piece[1..]).unwrap();
    // Apple's image is 3 GB: most of it is missing from the stick.
    let size = |_: &str, _: &str| -> Result<(u64, u64)> { Ok((1_000_000, 3_000_000_000)) };
    let fetch = |_: &Path, _: &str, _: &str, need: sources::Need| -> Result<sources::Fetched> {
        anyhow::bail!("asked for {need:?}")
    };
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let repair = |ui: &dyn ui::Ui, yes: bool| {
        let mut request = update_request(&f, &f._dir.path().join("cache"));
        request.action = Action::Repair;
        request.yes = yes;
        request.dispatch(&Environment {
            image_size: &size,
            fetch_image: &fetch,
            ui,
            ..environment(&f.host)
        })
    };
    // Without a terminal: refused before anything changes, naming --yes.
    let error = repair(&ui::NoTerminal, false).unwrap_err().to_string();
    assert!(
        error.contains("this downloads") && error.contains("Add --yes"),
        "{error}"
    );
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // Asked on a terminal, saying the size; a no changes nothing.
    let no = ui::tests::Scripted::new(&["no"]);
    let error = repair(&no, false).unwrap_err().to_string();
    assert_eq!(error, "nothing was changed");
    assert!(
        no.transcript().contains("from Apple"),
        "{}",
        no.transcript()
    );
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    // A yes, or --yes, goes on to the download.
    let yes = ui::tests::Scripted::new(&["yes"]);
    let error = repair(&yes, false).unwrap_err().to_string();
    assert!(error.contains("asked for Pieces"), "{error}");
    let error = repair(&ui::NoTerminal, true).unwrap_err().to_string();
    assert!(error.contains("asked for Pieces"), "{error}");
}

#[test]
fn finishing_a_create_asks_before_downloading_more_than_a_gigabyte() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    interrupt_setup(&f);
    // Apple's image is 3 GB: most of it is missing from the stick.
    let size = |_: &str, _: &str| -> Result<(u64, u64)> { Ok((1_000_000, 3_000_000_000)) };
    let fetch = |_: &Path, _: &str, _: &str, need: sources::Need| -> Result<sources::Fetched> {
        anyhow::bail!("asked for {need:?}")
    };
    let before = (snapshot(&f.esp), snapshot(&stela_volume(&f)));
    let finish = |ui: &dyn ui::Ui, yes: bool| {
        let mut request = update_request(&f, &f._dir.path().join("cache"));
        request.action = Action::Repair;
        request.yes = yes;
        request.dispatch(&Environment {
            image_size: &size,
            fetch_image: &fetch,
            ui,
            ..environment(&f.host)
        })
    };
    let error = finish(&ui::NoTerminal, false).unwrap_err().to_string();
    assert!(
        error.contains("this downloads") && error.contains("Add --yes"),
        "{error}"
    );
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    let no = ui::tests::Scripted::new(&["no"]);
    assert_eq!(
        finish(&no, false).unwrap_err().to_string(),
        "nothing was changed"
    );
    assert!(
        no.transcript().contains("the rest of macOS"),
        "{}",
        no.transcript()
    );
    assert_eq!((snapshot(&f.esp), snapshot(&stela_volume(&f))), before);
    let error = finish(&ui::NoTerminal, true).unwrap_err().to_string();
    // --yes goes on past the question to the download.
    assert!(!error.contains("this downloads"), "{error}");
    // Plain stela names the size in its Finish it choice.
    let person = ui::tests::Scripted::new(&["Quit"]);
    let environment = Environment {
        image_size: &size,
        fetch_image: &fetch,
        ui: &person,
        ..environment(&f.host)
    };
    let request = Request {
        efi: Some(f.efi.clone()),
        ..Request::home(command::Global::default())
    };
    home::run(&environment, request).unwrap();
    let shown = person.transcript();
    assert!(shown.contains("downloads the rest (3.0 GB)"), "{shown}");
}

/// Plain `stela` on `f`'s computer, answered by `answers`; what it showed
/// and how it ended. `networks` are the Wi-Fi networks this computer sees.
fn plain_stela(f: &Fixture, answers: &[&str], networks: &[&str]) -> (String, Result<()>, bool) {
    let person = ui::tests::Scripted::new(answers);
    let networks: Vec<String> = networks.iter().map(|n| (*n).to_owned()).collect();
    let list = move || networks.clone();
    let environment = Environment {
        ui: &person,
        wifi_networks: &list,
        ..environment(&f.host)
    };
    let request = Request {
        efi: Some(f.efi.clone()),
        ..Request::home(command::Global::default())
    };
    let result = home::run(&environment, request);
    (person.transcript(), result, person.done())
}

/// Every file on `f`'s stick.
fn stick_files(f: &Fixture) -> (BTreeMap<PathBuf, Vec<u8>>, BTreeMap<PathBuf, Vec<u8>>) {
    (snapshot(&f.esp), snapshot(&f.data))
}

#[test]
fn journey_a_blank_stick_walks_through_the_wizard_and_cancel_changes_nothing() {
    let f = fixture(64 * GIB, false);
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(
        &f,
        &[
            "Online",
            "No Wi-Fi (Ethernet)",
            "Change something",
            "Installer",
            "Offline",
            "Cancel",
        ],
        &[],
    );
    result.unwrap();
    assert!(done, "{shown}");
    // One stick: it is not asked for. Offline is offered first, Wi-Fi only
    // for Online, and the erase is named before the last question.
    assert!(!shown.contains("? Which stick?"), "{shown}");
    for line in [
        "? Installer",
        "● Offline · macOS is copied to the stick now",
        "? Wi-Fi during installation",
        "◇ Review",
        "  Installer  Online · the install downloads macOS",
        "  Installer  Offline · the install needs no internet",
        "? Change what?",
        "● Erase and create",
        "■ Nothing was changed.",
    ] {
        assert!(shown.contains(line), "{line:?} missing:\n{shown}");
    }
    assert!(shown.contains("Everything on"), "{shown}");
    assert_eq!(stick_files(&f), before);
    assert!(f.host.unmounted.borrow().is_empty());
    // Escape at the first question changes nothing either.
    let (shown, result, _) = plain_stela(&f, &["cancel"], &[]);
    result.unwrap();
    assert!(shown.ends_with("■ Nothing was changed."), "{shown}");
    assert_eq!(stick_files(&f), before);
}

#[test]
fn review_and_status_show_the_wifi_country() {
    let f = fixture(64 * GIB, false);
    let person = ui::tests::Scripted::new(&["Online", "Home", "correct horse", "Cancel"]);
    let networks = || vec!["Home".to_owned()];
    let country = || Some("DE".to_owned());
    let environment = Environment {
        ui: &person,
        wifi_networks: &networks,
        wifi_country: &country,
        ..environment(&f.host)
    };
    let request = Request {
        efi: Some(f.efi.clone()),
        ..Request::home(command::Global::default())
    };
    home::run(&environment, request).unwrap();
    let shown = person.transcript();
    assert!(shown.contains("Network    Wi-Fi Home, DE rules"), "{shown}");
    // Status names the network and its country.
    let f = made_by(Some(release::CURRENT));
    let status = rerun(&f, |c| c.action = Action::Status).unwrap();
    assert!(status.contains("Wi-Fi          none"), "{status}");
    let mut joined = record(&f);
    joined.wifi = Some(wifi::Network {
        ssid: "Home".into(),
        country: Some("DE".into()),
    });
    record::write_record(&f.esp, &joined).unwrap();
    let status = rerun(&f, |c| c.action = Action::Status).unwrap_or_else(|e| e.to_string());
    assert!(
        status.contains("Wi-Fi          \"Home\", DE rules"),
        "{status}"
    );
}

#[test]
fn journey_b_a_stick_with_stela_is_not_treated_as_blank() {
    let f = made_by(Some(release::CURRENT));
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(!shown.contains("? Installer"), "{shown}");
    assert!(
        shown.contains(&format!("● Stela {} · Offline · macOS", release::CURRENT)),
        "{shown}"
    );
    assert!(shown.contains("This stick is ready."), "{shown}");
    assert!(!shown.contains("Nothing to do"), "{shown}");
    // Enter takes Quit, the one row marked: no row that writes the stick.
    assert!(shown.contains("● Quit"), "{shown}");
    assert!(
        shown.contains("○ Erase it and make a new installer"),
        "{shown}"
    );
    assert!(shown.ends_with("■ Nothing was changed."), "{shown}");
    assert_eq!(shown.matches("Nothing was changed.").count(), 1, "{shown}");
    assert_eq!(stick_files(&f), before);
    // Erasing it from the menu warns that it holds Stela; Cancel keeps it.
    let (shown, result, done) = plain_stela(
        &f,
        &["Erase it and make a new installer", "Offline", "Cancel"],
        &[],
    );
    result.unwrap();
    assert!(done, "{shown}");
    assert!(
        shown.contains("▲ This stick holds Stela with macOS"),
        "{shown}"
    );
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_c_an_older_stick_offers_the_update_and_quit_changes_nothing() {
    let f = made_by(None);
    let before = stick_files(&f);
    let question = format!("? Update this stick to Stela {}?", release::CURRENT);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(shown.contains(&question), "{shown}");
    assert!(shown.contains("● Stela 0.3 or older · Offline"), "{shown}");
    assert!(shown.contains("● Replaces stela.efi"), "{shown}");
    assert!(shown.ends_with("■ Nothing was changed."), "{shown}");
    assert_eq!(stick_files(&f), before);
    // Update brings it to this Stela.
    let (shown, result, _) = plain_stela(&f, &["Update", "Not now"], &[]);
    result.unwrap();
    assert_eq!(record(&f).release.as_deref(), Some(release::CURRENT));
    assert!(shown.contains("└ Done."), "{shown}");
}

#[test]
fn journey_d_a_newer_stick_is_left_alone() {
    let f = made_by(Some("99.0.0"));
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Show status"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(
        shown.contains("▲ This stick has Stela 99.0.0, newer than this stela"),
        "{shown}"
    );
    // Nothing that changes the stick is offered.
    assert!(!shown.contains("Update"), "{shown}");
    assert!(!shown.contains("Repair"), "{shown}");
    assert!(!shown.contains("Erase"), "{shown}");
    assert_eq!(stick_files(&f), before);
}

#[test]
fn the_menu_saves_the_boot_logs_to_the_file_it_names() {
    let f = made_by(Some(release::CURRENT));
    let usb = crate::boot_log::tests::region(&["CHOICE 3 (Install macOS)\n"]);
    fs::write(f.esp.join(efi::BOOT_LOG), &usb).unwrap();
    let before = stick_files(&f);
    let output = f._dir.path().join("log.txt");
    let person = ui::tests::Scripted::new(&[]);
    let environment = Environment {
        ui: &person,
        ..environment(&f.host)
    };
    let request = Request {
        action: Action::BootLog,
        usb: f.esp.clone(),
        output: Some(output.clone()),
        ..Request::default()
    };
    home::perform(&environment, request).unwrap();
    let saved = fs::read_to_string(&output).unwrap();
    assert!(saved.starts_with("Boot log on the USB ("), "{saved}");
    assert!(saved.contains("CHOICE 3 (Install macOS)\n"), "{saved}");
    assert!(
        person.transcript().starts_with("└ Saved the boot logs to "),
        "{}",
        person.transcript()
    );
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_e_an_interrupted_create_offers_to_finish_it() {
    let mut f = fixture(64 * GIB, false);
    add_stela(&mut f);
    setup(&environment(&f.host), command(&f, &f.esp)).unwrap();
    interrupt_setup(&f);
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    for line in [
        "▲ A create stopped part way: continuing the Offline installer",
        "? Finish the create?",
        "● Finish it · Keeps what is written, checks it, and writes the rest",
        "○ Start over · Erases the stick",
        "■ Nothing was changed.",
    ] {
        assert!(shown.contains(line), "{line:?} missing:\n{shown}");
    }
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_f_an_interrupted_macos_switch_offers_to_finish_it() {
    let f = made_by(Some(release::CURRENT));
    let mut switching = record(&f);
    switching.replacing = Some("STELA2".into());
    record::write_record(&f.esp, &switching).unwrap();
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(
        shown.contains("● Finish the switch to macOS STELA2"),
        "{shown}"
    );
    assert!(
        shown.contains("cannot install macOS until it is"),
        "{shown}"
    );
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_g_a_damaged_stick_offers_repair_and_quit_changes_nothing() {
    let f = made_by(Some(release::CURRENT));
    fs::remove_file(f.esp.join(LOADER_PATH)).unwrap();
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(shown.contains("▲ This stick needs repair:"), "{shown}");
    assert!(shown.contains("? Repair this stick?"), "{shown}");
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_h_without_a_terminal_plain_stela_reports_and_fails_when_something_is_due() {
    let f = made_by(None);
    let before = stick_files(&f);
    let request = Request::home(command::Global::default());
    let error = home::run(&environment(&f.host), request).unwrap_err();
    assert!(error.is::<home::Due>());
    let (text, due) = home::report(&home::sticks(&environment(&f.host)).unwrap());
    assert!(due);
    assert!(text.contains("Stela 0.3 or older · Offline"), "{text}");
    assert!(text.contains("Next: stela update\n"), "{text}");
    assert_eq!(stick_files(&f), before);
    // A healthy stick: nothing is due.
    let f = made_by(Some(release::CURRENT));
    let request = Request::home(command::Global::default());
    home::run(&environment(&f.host), request).unwrap();
    let (text, due) = home::report(&home::sticks(&environment(&f.host)).unwrap());
    assert!(!due, "{text}");
    assert!(text.contains("Nothing to do."), "{text}");
    // A blank stick names the command that makes an installer, and is not
    // due.
    let f = fixture(64 * GIB, false);
    let request = Request::home(command::Global::default());
    home::run(&environment(&f.host), request).unwrap();
    let (text, due) = home::report(&home::sticks(&environment(&f.host)).unwrap());
    assert!(!due, "{text}");
    let command = format!("Next: stela create {}disk5", host::DISK_PREFIX);
    assert!(text.contains(&command), "{text}");
}

#[test]
fn journey_settings_change_the_wifi_and_cancel_changes_nothing() {
    let f = made_by(Some(release::CURRENT));
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(
        &f,
        &[
            "Change Wi-Fi or network settings",
            "Home",
            "correct horse",
            "Automatic",
            "Cancel",
        ],
        &["Home"],
    );
    result.unwrap();
    assert!(done, "{shown}");
    assert!(shown.contains("? Save these settings?"), "{shown}");
    assert!(shown.ends_with("■ Nothing was changed."), "{shown}");
    assert_eq!(stick_files(&f), before);
    let (shown, result, done) = plain_stela(
        &f,
        &[
            "Change Wi-Fi or network settings",
            "Home",
            "correct horse",
            "Automatic",
            "Save",
        ],
        &["Home"],
    );
    result.unwrap();
    assert!(done, "{shown}");
    let saved = record(&f);
    assert_eq!(
        saved.wifi.as_ref().map(|wifi| wifi.ssid.as_str()),
        Some("Home")
    );
}

#[test]
fn settings_without_a_terminal_and_without_a_setting_change_nothing() {
    let f = made_by(Some(release::CURRENT));
    let before = stick_files(&f);
    let error = rerun(&f, |c| c.action = Action::Configure)
        .unwrap_err()
        .to_string();
    assert!(
        error.starts_with("name a setting to change, such as --wifi SSID"),
        "{error}"
    );
    assert!(error.contains("`stela settings` asks"), "{error}");
    assert_eq!(stick_files(&f), before);
}

#[test]
fn journey_several_sticks_are_chosen_from_and_quit_changes_nothing() {
    let mut f = made_by(Some(release::CURRENT));
    let mut second = disk("disk6", &[("EFI", None)]);
    second.removal = host::Removal::Usb;
    f.host.disks.push(second);
    let before = stick_files(&f);
    let (shown, result, done) = plain_stela(&f, &["Quit"], &[]);
    result.unwrap();
    assert!(done, "{shown}");
    assert!(shown.contains("? Which stick?"), "{shown}");
    // The Stela stick comes first.
    let stela = shown.find("Disk disk5").unwrap();
    let other = shown.find("Disk disk6").unwrap();
    assert!(stela < other, "{shown}");
    assert_eq!(stick_files(&f), before);
}
