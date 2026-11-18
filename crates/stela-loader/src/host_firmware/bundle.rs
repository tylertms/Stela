//! The complete host firmware bundle: every pinned recipe's file, whatever
//! computer made the USB. Files are stored once by content (`<digest>.bin`)
//! beside their license notices, under [`DIRECTORY`] on the USB's data
//! volume, and [`MANIFEST`] lists them. A boot reads only the files the PCI
//! functions it observes need ([`select`]), and [`BundleProvider`] gives a
//! driver's exact request the file its recipe pins.
use super::{
    Error, MAX_IMAGE_BYTES, MAX_MANIFEST_BYTES, fail, logical_path,
    recipes::{self, Artifact, Plan, Recipe},
    requirements::{Inventory, Request},
    storage_path, valid_label,
};
use alloc::{boxed::Box, format, string::String, vec::Vec};
use core::cell::RefCell;
use serde::{Deserialize, Serialize};
use stela_driver_core::firmware::{
    self as driver, Catalog, ErrorKind, Image, Provider, Requirement,
};

/// The bundle's directory on its volume.
pub const DIRECTORY: &str = "stela/firmware";
/// The bundle's manifest on its volume.
pub const MANIFEST: &str = "stela/firmware/bundle.json";
/// At most this many files and as many notices.
pub const MAX_OBJECTS: usize = 256;
/// The target a bundle file has in the boot's catalog, under the name
/// [`name`] gives its digest. Only [`BundleProvider`] asks for it.
pub const TARGET: driver::Target<'static> = driver::Target {
    driver: "stela-bundle",
    hardware: "content",
    revision: "sha256",
    board: None,
};

/// Which GPU ports a build runs, and so whose firmware its bundle holds:
/// the AMD port (amdgpu), which a release leaves out with its driver. The
/// boot image enables this crate's `amdgpu-port` exactly where it enables
/// amdgpu (`crates/stela-boot/Cargo.toml`), and `stela` where it is built
/// with `amdgpu-port` (`crates/stela-cli/Cargo.toml`), so a default stick
/// holds the files of the default image's drivers and no others. No Intel
/// GPU firmware is pinned yet; its port joins here with its recipes.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ports {
    pub amd: bool,
}

impl Ports {
    /// This build's ports.
    pub const BUILT: Self = Self {
        amd: cfg!(feature = "amdgpu-port"),
    };
    /// A build without the AMD port, as a release is.
    pub const NONE: Self = Self { amd: false };
    /// A build with every port.
    pub const ALL: Self = Self { amd: true };
}

/// The port a recipe family belongs to, if any.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
enum Port {
    Always,
    Amd,
}

/// Every recipe family, in the order a volume short of room keeps them: the
/// small network and Bluetooth files first, the GPU's large images last.
const ALL: [(&str, &[Recipe], Port); 10] = [
    (
        "Wi-Fi regulatory database",
        recipes::REGULATORY,
        Port::Always,
    ),
    ("Realtek Ethernet", recipes::REALTEK, Port::Always),
    ("Realtek USB Ethernet", recipes::REALTEK_USB, Port::Always),
    ("Realtek Wi-Fi", recipes::REALTEK_WIFI, Port::Always),
    ("MediaTek Wi-Fi", recipes::MEDIATEK_WIFI, Port::Always),
    ("Intel Wi-Fi", recipes::INTEL_WIFI, Port::Always),
    (
        "MediaTek Bluetooth",
        recipes::MEDIATEK_BLUETOOTH,
        Port::Always,
    ),
    ("Intel Bluetooth", recipes::INTEL_RSA, Port::Always),
    ("AMD GPU", recipes::AMDGPU, Port::Amd),
    ("NVIDIA GSP-RM", recipes::GSP, Port::Always),
];

/// The recipe families a build with `ports` bundles, in [`ALL`] order.
pub fn families_for(ports: Ports) -> impl Iterator<Item = (&'static str, &'static [Recipe])> {
    ALL.iter()
        .filter(move |(_, _, port)| match port {
            Port::Always => true,
            Port::Amd => ports.amd,
        })
        .map(|&(name, recipes, _)| (name, recipes))
}

/// The recipe families this build bundles ([`Ports::BUILT`]).
pub fn families() -> impl Iterator<Item = (&'static str, &'static [Recipe])> {
    families_for(Ports::BUILT)
}

/// Every recipe of the catalog this build bundles.
pub fn recipes() -> impl Iterator<Item = &'static Recipe> {
    families().flat_map(|(_, recipes)| recipes.iter())
}

/// Each distinct file of the catalog once, by digest, in [`families`]
/// order with each family's files smallest first.
pub fn objects() -> Vec<&'static Artifact> {
    objects_for(Ports::BUILT)
}

/// [`objects`] of a build with `ports`.
pub fn objects_for(ports: Ports) -> Vec<&'static Artifact> {
    let mut objects: Vec<&'static Artifact> = Vec::new();
    for (_, family) in families_for(ports) {
        let start = objects.len();
        for recipe in family {
            if !objects
                .iter()
                .any(|seen| seen.sha256 == recipe.image.sha256)
            {
                objects.push(&recipe.image);
            }
        }
        objects[start..].sort_by_key(|artifact| (artifact.length, artifact.sha256));
    }
    objects
}

/// Each distinct license notice of the catalog once, by digest.
pub fn notices() -> Vec<&'static Artifact> {
    let mut notices: Vec<&'static Artifact> = Vec::new();
    for recipe in recipes() {
        if !notices
            .iter()
            .any(|seen| seen.sha256 == recipe.license.sha256)
        {
            notices.push(recipe.license);
        }
    }
    notices
}

/// The complete bundle's file count and bytes, notices aside.
pub fn size() -> (usize, usize) {
    let objects = objects();
    (
        objects.len(),
        objects.iter().map(|artifact| artifact.length).sum(),
    )
}

/// The catalog name of the file with `sha256`.
pub fn name(sha256: &str) -> String {
    format!("sha256/{sha256}")
}

/// A file's path in [`DIRECTORY`]: its digest's first half.
pub fn object_path(sha256: &str) -> String {
    format!("{}.bin", &sha256[..32.min(sha256.len())])
}

/// A notice's path in [`DIRECTORY`].
pub fn notice_path(sha256: &str) -> String {
    format!("licenses/{}.txt", &sha256[..32.min(sha256.len())])
}

#[derive(Debug, Deserialize, Serialize)]
#[serde(deny_unknown_fields)]
pub struct Bundle<'a> {
    pub schema: u32,
    #[serde(borrow)]
    pub objects: Vec<Object<'a>>,
    #[serde(borrow)]
    pub notices: Vec<Notice<'a>>,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Object<'a> {
    pub path: &'a str,
    pub sha256: &'a str,
    pub length: usize,
    pub source: &'a str,
    pub revision: &'a str,
    pub member: &'a str,
    /// The path of its notice.
    pub license: &'a str,
}

#[derive(Debug, Deserialize, Serialize, PartialEq, Eq)]
#[serde(deny_unknown_fields)]
pub struct Notice<'a> {
    pub path: &'a str,
    pub sha256: &'a str,
    pub length: usize,
    pub source: &'a str,
    pub revision: &'a str,
    pub member: &'a str,
}

fn digest(sha256: &str) -> bool {
    sha256.len() == 64
        && sha256
            .bytes()
            .all(|b| b.is_ascii_digit() || (b'a'..=b'f').contains(&b))
}

/// The manifest of a bundle of `files`, each with its license notice.
pub fn manifest(files: &[(&Artifact, &Artifact)]) -> Result<Vec<u8>, Error> {
    let mut notices: Vec<&Artifact> = Vec::new();
    for (_, notice) in files {
        if !notices.iter().any(|seen| seen.sha256 == notice.sha256) {
            notices.push(notice);
        }
    }
    let objects: Vec<_> = files
        .iter()
        .map(|(object, notice)| {
            serde_json::json!({"path":object_path(object.sha256),"sha256":object.sha256,
                "length":object.length,"source":object.source,"revision":object.revision,
                "member":object.member,"license":notice_path(notice.sha256)})
        })
        .collect();
    let notices: Vec<_> = notices
        .iter()
        .map(|notice| {
            serde_json::json!({"path":notice_path(notice.sha256),"sha256":notice.sha256,
                "length":notice.length,"source":notice.source,"revision":notice.revision,
                "member":notice.member})
        })
        .collect();
    let bytes = serde_json::to_vec_pretty(
        &serde_json::json!({"schema":1,"objects":objects,"notices":notices}),
    )
    .map_err(|e| Error(format!("host firmware bundle manifest: {e}")))?;
    Bundle::parse(&bytes)?;
    Ok(bytes)
}

/// The complete bundle's files with their notices, in [`objects`] order.
pub fn complete() -> Vec<(&'static Artifact, &'static Artifact)> {
    objects()
        .into_iter()
        .filter_map(|object| {
            let recipe = recipes().find(|recipe| recipe.image.sha256 == object.sha256)?;
            Some((object, recipe.license))
        })
        .collect()
}

impl<'a> Bundle<'a> {
    pub fn parse(bytes: &'a [u8]) -> Result<Self, Error> {
        if bytes.len() > MAX_MANIFEST_BYTES {
            return Err(fail("host firmware bundle manifest exceeds 128 KiB"));
        }
        let bundle: Self = serde_json::from_slice(bytes)
            .map_err(|e| Error(format!("host firmware bundle manifest: {e}")))?;
        if bundle.schema != 1 {
            return Err(fail("unsupported host firmware bundle schema"));
        }
        if bundle.objects.len() > MAX_OBJECTS || bundle.notices.len() > MAX_OBJECTS {
            return Err(fail("host firmware bundle lists too many files"));
        }
        for (index, notice) in bundle.notices.iter().enumerate() {
            if !digest(notice.sha256)
                || notice.path != notice_path(notice.sha256)
                || notice.length == 0
                || notice.length > MAX_IMAGE_BYTES
                || !valid_label(notice.source)
                || !valid_label(notice.revision)
                || !logical_path(notice.member)
                || bundle.notices[..index]
                    .iter()
                    .any(|other| other.sha256 == notice.sha256)
            {
                return Err(fail("invalid host firmware bundle notice"));
            }
        }
        for (index, object) in bundle.objects.iter().enumerate() {
            if !digest(object.sha256)
                || object.path != object_path(object.sha256)
                || !storage_path(object.path)
                || object.length == 0
                || object.length > MAX_IMAGE_BYTES
                || !valid_label(object.source)
                || !valid_label(object.revision)
                || !logical_path(object.member)
                || !bundle
                    .notices
                    .iter()
                    .any(|notice| notice.path == object.license)
                || bundle.objects[..index]
                    .iter()
                    .any(|other| other.sha256 == object.sha256 || other.path == object.path)
            {
                return Err(fail("invalid host firmware bundle file"));
            }
        }
        Ok(bundle)
    }

    /// The file with `sha256`, when the bundle holds it.
    pub fn object(&self, sha256: &str) -> Option<&Object<'a>> {
        self.objects.iter().find(|object| object.sha256 == sha256)
    }

    /// The catalog's files the bundle lacks.
    pub fn missing(&self) -> Vec<&'static Artifact> {
        objects()
            .into_iter()
            .filter(|artifact| self.object(artifact.sha256).is_none())
            .collect()
    }
}

/// The schema 1 manifest a boot hands the engine: `exact`'s images, then
/// each of `selected` under its digest's [`name`] and [`TARGET`], in that
/// order, which the image regions follow.
pub fn handoff_manifest(
    exact: Option<&super::Manifest<'_>>,
    selected: &[&Object<'_>],
) -> Result<Vec<u8>, Error> {
    let mut images = Vec::new();
    for image in exact.iter().flat_map(|manifest| &manifest.images) {
        images.push(serde_json::json!({"name":image.name,"path":image.path,
            "target":{"driver":image.target.driver,"hardware":image.target.hardware,
                "revision":image.target.revision,"board":image.target.board},
            "provenance":{"source":image.provenance.source,"revision":image.provenance.revision,
                "member":image.provenance.member,"license":image.provenance.license},
            "length":image.length,"sha256":image.sha256}));
    }
    for object in selected {
        images.push(serde_json::json!({"name":name(object.sha256),
            "path":format!("bundle/{}", object.path),
            "target":{"driver":TARGET.driver,"hardware":TARGET.hardware,
                "revision":TARGET.revision,"board":null},
            "provenance":{"source":object.source,"revision":object.revision,
                "member":object.member,"license":object.license},
            "length":object.length,"sha256":object.sha256}));
    }
    let bytes = serde_json::to_vec(&serde_json::json!({"schema":1,"images":images}))
        .map_err(|e| Error(format!("host firmware handoff manifest: {e}")))?;
    super::Manifest::parse(&bytes)?;
    Ok(bytes)
}

/// One PCI function a boot observed: where it is, IDs and base class, and
/// a bridge's bus numbers.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Pci {
    pub segment: u16,
    pub bus: u8,
    pub vendor: u16,
    pub device: u16,
    pub class: u8,
    /// A bridge's (type 1 header) secondary and subordinate bus numbers.
    pub bridge: Option<(u8, u8)>,
}

impl Pci {
    /// The function on `bus` of `segment` whose configuration header
    /// starts with `header`: identity from its first 12 bytes and, for a
    /// type 1 header, the bus numbers at 0x19 and 0x1a. `None` when the
    /// header is shorter than a type 1 header's bus numbers.
    pub fn from_header(segment: u16, bus: u8, header: &[u8]) -> Option<Self> {
        let header = header.get(..0x1b)?;
        let word = |offset: usize| u16::from_le_bytes([header[offset], header[offset + 1]]);
        Some(Self {
            segment,
            bus,
            vendor: word(0),
            device: word(2),
            class: header[11],
            bridge: (header[0x0e] & 0x7f == 1).then_some((header[0x19], header[0x1a])),
        })
    }
}

/// The PCI ports among `functions` where a device the firmware left
/// without power can answer later: a bridge with a secondary bus of its
/// own that holds none of `functions`. Stela's engine powers such a port on
/// through ACPI as Linux does before its PCI census
/// (`stela_drivers::probe::d3cold`), which finds what lies below it: a
/// hybrid laptop's discrete GPU in D3cold answers only then.
pub fn empty_ports(functions: &[Pci]) -> impl Iterator<Item = &Pci> {
    functions.iter().filter(|port| {
        // A bridge without a bus number of its own forwards nothing yet.
        port.bridge.is_some_and(|(secondary, subordinate)| {
            secondary > port.bus && subordinate >= secondary
        }) && !functions.iter().any(|function| {
            function.segment == port.segment
                && Some(function.bus) == port.bridge.map(|(secondary, _)| secondary)
        })
    })
}

/// What the firmware's PCI census can say before an install or a start
/// about the NVIDIA GPU macOS needs to start.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum NvidiaNotice {
    /// A display controller of a chip Stela's catalog names answers.
    Driven,
    /// No NVIDIA display controller answers, and no PCI port shows nothing
    /// below it, so none can answer later.
    Absent,
    /// NVIDIA display controllers answer, none of a chip Stela drives, and
    /// no PCI port shows nothing below it.
    NotDriven,
    /// No NVIDIA display controller Stela drives answers yet, and some PCI
    /// ports show nothing below them ([`empty_ports`]): a GPU the firmware
    /// left without power there answers only after Stela's engine powers
    /// the port, which the census cannot foresee, since only ACPI says
    /// whether a port's power is off.
    NotYet,
}

/// The [`NvidiaNotice`] for a PC whose firmware showed `functions`.
pub fn nvidia_notice(functions: &[Pci]) -> NvidiaNotice {
    let mut nvidia = functions
        .iter()
        .filter(|function| function.vendor == 0x10de && function.class == 0x03)
        .peekable();
    let seen = nvidia.peek().is_some();
    if nvidia.any(|function| recipes::nvidia_chip(function.device).is_some()) {
        NvidiaNotice::Driven
    } else if empty_ports(functions).next().is_some() {
        NvidiaNotice::NotYet
    } else if seen {
        NvidiaNotice::NotDriven
    } else {
        NvidiaNotice::Absent
    }
}

/// A GPU driver's files, all of them, for a GPU that may answer only after
/// the firmware's exit: no PCI function the firmware showed names the chip,
/// so every chip's files are read.
#[derive(Debug)]
pub struct Late {
    /// The recipe family, as [`families`] names it.
    pub family: &'static str,
    /// Its files the selection did not already want, each once.
    pub files: Vec<&'static Artifact>,
    /// Why they may be needed.
    pub why: String,
}

impl Late {
    /// The files' bytes.
    pub fn bytes(&self) -> usize {
        self.files.iter().map(|artifact| artifact.length).sum()
    }
}

/// The GPU families a GPU can need late: the family's name in [`ALL`], the
/// PCI vendor of its GPUs and the port it belongs to.
const GPU_FAMILIES: [(&str, u16, Port); 2] = [
    ("AMD GPU", 0x1002, Port::Amd),
    ("NVIDIA GSP-RM", 0x10de, Port::Always),
];

/// The images and bytes the firmware handoff has left for bundle files.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub struct Room {
    pub images: usize,
    pub bytes: usize,
}

/// The files a PC needs and why, and its devices of a supported family
/// whose files no recipe pins.
#[derive(Debug, Default)]
pub struct Selection {
    /// Each file once, with the device that asked first.
    pub wanted: Vec<(&'static Artifact, String)>,
    /// Devices of a supported family that no recipe covers, each once.
    pub unknown: Vec<String>,
    /// The GPU families whose files a GPU that answers only after the
    /// firmware's exit would need, not yet in [`Self::wanted`]:
    /// [`Self::take_late`] moves those the handoff has room for there.
    pub late: Vec<Late>,
}

impl Selection {
    fn add(&mut self, artifact: &'static Artifact, why: &str) {
        if !self
            .wanted
            .iter()
            .any(|(seen, _)| seen.sha256 == artifact.sha256)
        {
            self.wanted.push((artifact, why.into()));
        }
    }

    fn plan(&mut self, inventory: &Inventory, why: &str) {
        if let Ok(plan) = Plan::select(inventory) {
            for selection in plan.selections() {
                self.add(&selection.recipe.image, why);
            }
        }
    }

    fn members(&mut self, recipes: &'static [Recipe], members: &[&str], why: &str) {
        for recipe in recipes {
            if members.is_empty() || members.contains(&recipe.image.member) {
                self.add(&recipe.image, why);
            }
        }
    }

    /// The bytes of the wanted files.
    pub fn bytes(&self) -> usize {
        self.wanted
            .iter()
            .map(|(artifact, _)| artifact.length)
            .sum()
    }

    /// Each GPU family of `ports` whose GPUs none of the observed
    /// `driven` vendors' functions named, as [`Late`] for `why`.
    fn add_late(&mut self, ports: Ports, driven: &[u16], why: &str) {
        for (family, vendor, port) in GPU_FAMILIES {
            if driven.contains(&vendor) || (port == Port::Amd && !ports.amd) {
                continue;
            }
            let Some((_, recipes, _)) = ALL.iter().find(|(name, _, _)| *name == family) else {
                continue;
            };
            let mut files: Vec<&'static Artifact> = Vec::new();
            for recipe in *recipes {
                let image = &recipe.image;
                if !files.iter().any(|seen| seen.sha256 == image.sha256)
                    && !self
                        .wanted
                        .iter()
                        .any(|(seen, _)| seen.sha256 == image.sha256)
                {
                    files.push(image);
                }
            }
            if !files.is_empty() {
                self.late.push(Late {
                    family,
                    files,
                    why: format!("{why}: {family}"),
                });
            }
        }
    }

    /// Moves each [`Late`] family that fits whole in `room`, less what
    /// [`Self::wanted`] already takes, into [`Self::wanted`], smallest
    /// first; a family's files are of use only together. Gives back the
    /// families left out for want of room.
    pub fn take_late(&mut self, room: Room) -> Vec<Late> {
        let mut images = room.images.saturating_sub(self.wanted.len());
        let mut bytes = room.bytes.saturating_sub(self.bytes());
        let mut late = core::mem::take(&mut self.late);
        late.sort_by_key(Late::bytes);
        let mut left = Vec::new();
        for family in late {
            if family.files.len() > images || family.bytes() > bytes {
                left.push(family);
                continue;
            }
            images -= family.files.len();
            bytes -= family.bytes();
            for artifact in &family.files {
                self.add(artifact, &family.why);
            }
        }
        left
    }
}

/// The files a PC with these PCI functions needs. USB Bluetooth controllers
/// and Ethernet adapters are not on PCI before the boot's own USB stack
/// runs, so their files, about 2 MB in all, are always selected.
pub fn select(functions: &[Pci]) -> Selection {
    select_for(functions, Ports::BUILT)
}

/// The USB files every boot reads.
fn usb() -> Selection {
    let mut selection = Selection::default();
    selection.members(recipes::REALTEK_USB, &[], "usb ethernet");
    selection.members(recipes::MEDIATEK_BLUETOOTH, &[], "usb bluetooth");
    selection.members(recipes::INTEL_RSA, &[], "usb bluetooth");
    selection
}

/// The files of a boot without the firmware's PCI census: the USB files
/// and, as [`Late`], every GPU family's of a build with `ports`, since no
/// GPU's chip is known.
pub fn select_without_census(ports: Ports) -> Selection {
    let mut selection = usb();
    selection.add_late(ports, &[], "a GPU the firmware's PCI census could not name");
    selection
}

/// [`select`] in a build with `ports`: a GPU of a port the build leaves out
/// has no driver and so no files. Where the firmware showed a PCI port
/// with nothing below it ([`empty_ports`]) and no GPU of a family whose
/// chip a recipe covers, that family's files are [`Late`]: a GPU the
/// firmware left without power there answers only after Stela's engine
/// powers the port, when no file can be read through the firmware.
pub fn select_for(functions: &[Pci], ports: Ports) -> Selection {
    let mut selection = usb();
    // The vendors of the GPUs whose files are selected.
    let mut driven: Vec<u16> = Vec::new();
    for function in functions {
        let why = format!("pci {:04x}:{:04x}", function.vendor, function.device);
        let mut inventory = Inventory::default();
        if function.vendor == 0x10de && function.class == 0x03 {
            match recipes::nvidia_chip(function.device) {
                Some(chip) if recipes::require_nvidia(&mut inventory, chip).is_ok() => {
                    selection.plan(&inventory, &why);
                    driven.push(function.vendor);
                }
                _ => {
                    if !selection.unknown.contains(&why) {
                        selection.unknown.push(why.clone());
                    }
                }
            }
            continue;
        }
        if function.vendor == 0x1002 && function.class == 0x03 {
            if !ports.amd {
                continue;
            }
            if recipes::require_amdgpu(&mut inventory, function.device).is_ok() {
                selection.plan(&inventory, &why);
                driven.push(function.vendor);
            } else if !selection.unknown.contains(&why) {
                selection.unknown.push(why.clone());
            }
            continue;
        }
        let wifi = match (function.vendor, function.device) {
            (0x10ec, device) if recipes::REALTEK_PCI_DEVICES.contains(&device) => {
                selection.members(recipes::REALTEK, &[], &why);
                false
            }
            (0x10ec, 0xc822 | 0xc82f) => {
                selection.members(
                    recipes::REALTEK_WIFI,
                    &["rtw88/rtw8822c_fw.bin", "rtw88/rtw8822c_wow_fw.bin"],
                    &why,
                );
                true
            }
            (0x10ec, 0xb852 | 0xb85b) => {
                selection.members(recipes::REALTEK_WIFI, &["rtw89/rtw8852b_fw-2.bin"], &why);
                true
            }
            (vendor, device) => {
                recipes::require_wifi(&mut inventory, vendor, device).unwrap_or(false)
            }
        };
        if wifi {
            selection.plan(&inventory, &why);
            selection.members(recipes::REGULATORY, &[], &why);
        }
    }
    let empty = empty_ports(functions).count();
    if empty > 0 {
        selection.add_late(
            ports,
            &driven,
            &format!(
                "a GPU that may answer only after Stela powers one of {empty} PCI ports with \
                 nothing below them"
            ),
        );
    }
    selection
}

/// What became of a file of the USB's bundle before the firmware's exit,
/// for the engine's words about a driver that lacks it.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Held {
    /// The USB has no bundle that a boot could read.
    NoBundle,
    /// The USB's bundle does not hold it.
    Lacks,
    /// Read, and handed to the engine.
    Read,
    /// Reading it failed.
    Failed,
    /// Selected for a GPU that may answer late, and left out for want of
    /// room in the firmware handoff.
    NoRoom,
    /// Not selected: no PCI function the firmware showed needed it.
    Unread,
}

impl Held {
    const STATES: [(Self, u8); 4] = [
        (Self::Read, b'r'),
        (Self::Failed, b'f'),
        (Self::NoRoom, b'b'),
        (Self::Unread, b'n'),
    ];
}

/// The record the engine reads with [`held`]: one line per file of
/// `bundle`, its digest and what `state` says became of it.
pub fn held_record(bundle: &Bundle<'_>, state: impl Fn(&str) -> Held) -> Vec<u8> {
    let mut record = Vec::with_capacity(bundle.objects.len() * 67);
    for object in &bundle.objects {
        let held = state(object.sha256);
        let Some(&(_, code)) = Held::STATES.iter().find(|(state, _)| *state == held) else {
            continue;
        };
        record.extend_from_slice(object.sha256.as_bytes());
        record.extend_from_slice(&[b' ', code, b'\n']);
    }
    record
}

/// What became of the file with `sha256` by `record` ([`held_record`]),
/// `None` when the boot found no bundle.
pub fn held(record: Option<&[u8]>, sha256: &str) -> Held {
    let Some(record) = record else {
        return Held::NoBundle;
    };
    record
        .split(|&byte| byte == b'\n')
        .find_map(|line| {
            let (digest, code) = line.split_at_checked(64)?;
            if digest != sha256.as_bytes() {
                return None;
            }
            let &[b' ', code] = code else {
                return None;
            };
            Held::STATES
                .iter()
                .find(|(_, known)| *known == code)
                .map(|&(state, _)| state)
        })
        .unwrap_or(Held::Lacks)
}

/// Why a driver lacks files after the firmware's exit, from what became of
/// each ([`held`]): the USB lacking any comes first, as `stela repair`
/// fixes it; then a failed read; else every one is on the USB and was not
/// read because its GPU answered only after the firmware's exit.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Absent {
    /// The USB lacks some.
    NotOnUsb,
    /// Reading some failed.
    ReadFailed,
    /// All on the USB, not read: their GPU answered too late, or the
    /// handoff had no room for them.
    AnsweredLate,
}

impl Absent {
    pub fn of(states: impl IntoIterator<Item = Held>) -> Self {
        let mut absent = Self::AnsweredLate;
        for state in states {
            match state {
                Held::NoBundle | Held::Lacks => return Self::NotOnUsb,
                Held::Failed | Held::Read => absent = Self::ReadFailed,
                Held::NoRoom | Held::Unread => {}
            }
        }
        absent
    }
}

/// Gives a driver's exact request the bundle file its recipe pins, when the
/// catalog has no exact binding for it: the file, found by digest, is bound
/// once to the request's own name and target, so the driver sees the image
/// it asked for. Call only under the host drivers' lock, like
/// [`super::requirements::RecordingProvider`].
pub struct BundleProvider {
    inner: &'static dyn Provider,
    pin: fn(&Request) -> Option<&'static Artifact>,
    bound: RefCell<Vec<&'static Catalog>>,
}

fn pinned(request: &Request) -> Option<&'static Artifact> {
    recipes::recipe(request).map(|recipe| &recipe.image)
}

impl BundleProvider {
    pub fn new(inner: &'static dyn Provider) -> Self {
        Self::with_pins(inner, pinned)
    }

    /// With `pin` in place of the recipes: the file a request binds to.
    fn with_pins(
        inner: &'static dyn Provider,
        pin: fn(&Request) -> Option<&'static Artifact>,
    ) -> Self {
        Self {
            inner,
            pin,
            bound: RefCell::new(Vec::new()),
        }
    }
}

impl Provider for BundleProvider {
    fn load(&self, requirement: Requirement<'_>) -> Result<Image<'_>, driver::Error> {
        let exact = self.inner.load(requirement);
        match &exact {
            Err(error) if matches!(error.kind, ErrorKind::Missing | ErrorKind::WrongTarget) => {}
            _ => return exact,
        }
        for catalog in self.bound.borrow().iter() {
            if let Ok(image) = catalog.load(requirement) {
                return Ok(image);
            }
        }
        let Some(pin) = (self.pin)(&Request::from(requirement)) else {
            return exact;
        };
        let name = name(pin.sha256);
        let stored = match self.inner.load(Requirement {
            name: &name,
            target: TARGET,
            minimum_bytes: requirement.minimum_bytes,
        }) {
            Ok(stored) => stored,
            Err(error) if error.kind == ErrorKind::Missing => return exact,
            Err(mut error) => {
                error.name = requirement.name.into();
                return Err(error);
            }
        };
        if pin
            .verify_digest(stored.bytes().len(), stored.sha256())
            .is_err()
        {
            return exact;
        }
        let catalog: &'static Catalog = Box::leak(Box::new(Catalog::rebind(
            stored,
            requirement.name,
            requirement.target,
        )?));
        self.bound.borrow_mut().push(catalog);
        catalog.load(requirement)
    }
}

#[cfg(test)]
mod tests;
