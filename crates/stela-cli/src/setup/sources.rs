//! What a command needs from a macOS restore image, and where it gets it.
//! One place decides, before anything is downloaded or changed, so every
//! command says up front what it reads and downloads.
//!
//! A command needs the image's boot files ([`Need::BootFiles`], the
//! members preparing a bundle reads, `boot::restore_inputs_in`) to prepare boot
//! bundles or name a remote install medium, and the whole image
//! ([`Need::WholeImage`]) only to write a full install medium
//! ([`need`]). [`locate`] tries, in order:
//!
//! 1. `--ipsw`;
//! 2. an image (or, for boot files, a boot archive) of the build already on
//!    this computer: in `--download-dir`, this user's cache, the invoking
//!    user's cache under `sudo` and root's cache when it is readable
//!    ([`image_caches`]);
//! 3. for boot files, the members a setup that streams the image keeps on
//!    its USB (`inputs`), or the USB's own full install medium of the
//!    build, which holds the image byte for byte (`medium::image`);
//! 4. Apple's servers: the boot files alone by byte ranges (about 0.45 GB of
//!    macOS 27.2's 27 GB), or the whole image streamed onto the USB's
//!    pieces (`medium::download`), its size read first.
use super::{Environment, InstallImage, host::bytes_text, medium, record::Record};
use crate::restore;
use anyhow::{Context, Result};
use std::{
    fs,
    path::{Path, PathBuf},
};

/// What a command must read from the restore image.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub(super) enum Need {
    /// The boot files: enough to prepare boot bundles and name the image
    /// on Apple's servers.
    BootFiles,
    /// The whole image, which a full install medium holds, and its boot
    /// files, to prepare boot bundles.
    WholeImage,
    /// The whole image for a full install medium, and no boot bundle: its
    /// boot files are not read.
    Pieces,
}

impl Need {
    /// Whether it writes the whole image as a full install medium.
    pub(super) fn whole(self) -> bool {
        matches!(self, Self::WholeImage | Self::Pieces)
    }
}

/// Whether `record` installs from a medium that streams the image from
/// Apple's servers.
fn wants_remote(record: &Record) -> bool {
    record.install_kind() != InstallImage::Full
}

/// Whether `record`'s medium of `build` stays as it is: present, intact
/// and of the kind the setup installs from.
pub(super) fn medium_ready(record: &Record, build: &str) -> bool {
    record
        .medium_of(build)
        .is_some_and(|m| m.remote == wants_remote(record) && m.problems().is_empty())
}

/// What a command that prepares boot bundles (`bundles`) and puts the
/// install medium of `build` right needs from the image.
pub(super) fn need(record: &Record, build: &str, bundles: bool) -> Option<Need> {
    match medium_ready(record, build) {
        false if !wants_remote(record) => Some(match bundles {
            true => Need::WholeImage,
            false => Need::Pieces,
        }),
        false => Some(Need::BootFiles),
        true => bundles.then_some(Need::BootFiles),
    }
}

/// Where the restore image comes from.
#[derive(Debug)]
pub(super) enum Source {
    /// `--ipsw`.
    Given(PathBuf),
    /// An image or boot archive of the build on this computer.
    Cached(PathBuf),
    /// The boot files a setup keeps on its USB (`inputs`).
    Kept(PathBuf),
    /// The USB's full install medium of the build.
    Medium(medium::Medium),
    /// Apple's servers: the image's boot files, `boot` bytes read by range,
    /// and for a whole image the `pieces` bytes of it the USB lacks, of the
    /// image's `whole`.
    Remote { boot: u64, pieces: u64, whole: u64 },
}

impl Source {
    /// Whether it is a file on this computer already, which a dry run
    /// reads.
    pub(super) fn is_local(&self) -> bool {
        matches!(self, Self::Given(_) | Self::Cached(_) | Self::Kept(_))
    }

    /// The bytes of the whole image when it is written to the USB.
    pub(super) fn image_bytes(&self) -> Result<u64> {
        Ok(match self {
            Self::Given(path) | Self::Cached(path) => fs::metadata(path)?.len(),
            Self::Kept(_) | Self::Medium(_) => 0,
            Self::Remote { whole, .. } => *whole,
        })
    }
}

/// The bytes `source` downloads from Apple: none from a file.
pub(super) fn download_bytes(source: &Source) -> u64 {
    match source {
        Source::Remote { boot, pieces, .. } => boot + pieces,
        _ => 0,
    }
}

/// Refuse, before anything is downloaded, a USB without room for the whole
/// image of `build` from `source` as an install medium beside what it
/// keeps.
pub(super) fn check_room(
    environment: &Environment<'_>,
    record: &Record,
    build: &str,
    source: &Source,
) -> Result<()> {
    medium::check_space(
        environment.host,
        medium::SpaceOptions {
            volume: &record.volume,
            build,
            ipsw_bytes: source.image_bytes()?,
            piece_bytes: environment.piece_bytes,
        },
    )?;
    Ok(())
}

/// What [`locate`] looks for, and where.
pub(super) struct Lookup<'a> {
    pub(super) need: Need,
    pub(super) build: &'a str,
    pub(super) model: &'a str,
    pub(super) ipsw: Option<&'a Path>,
    pub(super) download_dir: Option<&'a Path>,
    /// The setup's install medium of the build.
    pub(super) medium: Option<&'a medium::Medium>,
}

impl Lookup<'_> {
    /// The directories searched for a downloaded image, `--download-dir`
    /// first.
    fn directories(&self, environment: &Environment<'_>) -> Vec<PathBuf> {
        let mut directories: Vec<PathBuf> = self
            .download_dir
            .map(Path::to_path_buf)
            .into_iter()
            .collect();
        for directory in environment.image_caches {
            if !directories.contains(directory) {
                directories.push(directory.clone());
            }
        }
        directories
    }

    /// Where downloads and boot archives read from the USB go.
    fn download_dir(&self, environment: &Environment<'_>) -> PathBuf {
        self.download_dir
            .map(Path::to_path_buf)
            .or_else(|| environment.image_caches.first().cloned())
            .unwrap_or_else(super::download_dir)
    }

    fn what(&self) -> &'static str {
        match self.need {
            Need::BootFiles => "its boot files",
            Need::WholeImage | Need::Pieces => "the whole image",
        }
    }
}

/// Decide where `lookup`'s image comes from, downloading nothing but the
/// image's directory. Fails when nothing holds it and Apple's servers
/// cannot be asked for it.
pub(super) fn locate(environment: &Environment<'_>, lookup: &Lookup<'_>) -> Result<Source> {
    if let Some(ipsw) = lookup.ipsw {
        return Ok(Source::Given(ipsw.to_path_buf()));
    }
    let directories = lookup.directories(environment);
    if let Some(path) = directories
        .iter()
        .find_map(|directory| cached(directory, lookup))
    {
        return Ok(Source::Cached(path));
    }
    if lookup.need == Need::BootFiles
        && let Some(path) = lookup
            .medium
            .and_then(|m| super::inputs::kept(environment, &m.volume, lookup.build, lookup.model))
    {
        return Ok(Source::Kept(path));
    }
    if lookup.need == Need::BootFiles
        && let Some(medium) = lookup
            .medium
            .filter(|m| !m.remote && m.build == lookup.build && m.problems().is_empty())
    {
        return Ok(Source::Medium(medium.clone()));
    }
    let searched = directories
        .iter()
        .map(|d| d.display().to_string())
        .collect::<Vec<_>>()
        .join(", ");
    let medium = match lookup.medium {
        Some(m) if m.remote => "this stick streams it from Apple's servers",
        Some(_) => "this stick's copy of it is damaged",
        None => "this stick holds no copy of it",
    };
    match (environment.image_size)(lookup.build, lookup.model) {
        Ok((boot, whole)) => Ok(Source::Remote {
            boot: match lookup.need {
                Need::Pieces => 0,
                Need::BootFiles | Need::WholeImage => boot,
            },
            // A damaged copy of the build keeps its intact pieces.
            pieces: match lookup.need {
                Need::BootFiles => 0,
                Need::WholeImage | Need::Pieces => match lookup.medium {
                    Some(m) if m.build == lookup.build => medium::missing_bytes(
                        &m.volume,
                        lookup.build,
                        whole,
                        environment.piece_bytes,
                        &m.damaged,
                    ),
                    _ => whole,
                },
            },
            whole,
        }),
        Err(error) => anyhow::bail!(
            "the macOS {build} restore image is not on this computer (looked in {searched}) and \
             {medium}, and Apple's servers could not be asked for {what} ({error:#}). Pass the \
             image with --ipsw FILE, or connect to the network and run this again{only}",
            build = lookup.build,
            what = lookup.what(),
            only = match lookup.need {
                Need::BootFiles => ", which downloads only its boot files, not the whole image",
                Need::WholeImage | Need::Pieces => "",
            },
        ),
    }
}

/// The plan line that names `source`.
pub(super) fn describe(source: &Source, lookup: &Lookup<'_>) -> String {
    let build = lookup.build;
    match source {
        Source::Given(path) => format!("from {}", path.display()),
        Source::Cached(path) => format!("from {}, downloaded earlier", path.display()),
        Source::Kept(path) => format!(
            "from the boot files kept on this stick in {}; nothing is downloaded",
            path.display()
        ),
        Source::Medium(medium) => format!(
            "from this stick's copy of macOS {build} in {}; nothing is downloaded",
            medium.directory().display()
        ),
        Source::Remote { .. } => format!(
            "download {}: no copy of it is on this {}computer (pass --ipsw FILE to use one you \
             have)",
            remote_text(source, build),
            match lookup.need {
                Need::BootFiles => "stick or this ",
                Need::WholeImage | Need::Pieces => "",
            }
        ),
    }
}

/// What a [`Source::Remote`] downloads of `build`'s image, with the sizes.
fn remote_text(source: &Source, build: &str) -> String {
    let Source::Remote {
        boot,
        pieces,
        whole,
    } = *source
    else {
        return String::new();
    };
    let total = bytes_text(boot + pieces);
    match (boot, pieces) {
        // No boot bundle is prepared: the image's pieces alone.
        (0, _) if pieces == whole => {
            format!("{total} from Apple: the whole macOS {build} image straight onto the stick")
        }
        (0, _) => format!(
            "{total} from Apple: the pieces of the macOS {build} image this stick's copy lacks, \
             straight onto the stick; the pieces it holds are checked and kept"
        ),
        (_, 0) => format!(
            "{total} of the macOS {build} image's {} from Apple: its boot files, read by range",
            bytes_text(whole)
        ),
        _ if pieces == whole => format!(
            "{total} from Apple: the macOS {build} image's boot files, read by range, then the \
             whole image, {}, straight onto the stick",
            bytes_text(whole)
        ),
        _ => format!(
            "{total} from Apple: the macOS {build} image's boot files, read by range, then the \
             {} of it this stick's copy lacks, straight onto the stick; the pieces it holds are \
             checked and kept",
            bytes_text(pieces)
        ),
    }
}

/// A restore image file to prepare from: for a whole image downloaded
/// onto the USB, its boot files, with the download that writes the rest
/// (`Environment::download`). A boot archive read from the USB is removed
/// with it.
pub(super) struct Obtained {
    pub(super) path: PathBuf,
    pub(super) download: Option<medium::download::Download>,
    _temporary: Option<tempfile::TempDir>,
}

/// What `Environment::fetch_image` fetched.
pub(super) struct Fetched {
    pub(super) path: PathBuf,
    pub(super) download: Option<medium::download::Download>,
}

/// A temporary directory beside downloads, never the system temp
/// directory, which is often too small.
fn temporary_in(directory: &Path) -> Result<tempfile::TempDir> {
    fs::create_dir_all(directory)
        .with_context(|| format!("cannot create {}", directory.display()))?;
    Ok(tempfile::Builder::new()
        .prefix("stela-")
        .tempdir_in(directory)?)
}

/// The file `source` names, reading or downloading what it takes.
pub(super) fn obtain(
    environment: &Environment<'_>,
    source: &Source,
    lookup: &Lookup<'_>,
) -> Result<Obtained> {
    match source {
        Source::Given(path) | Source::Cached(path) | Source::Kept(path) => Ok(Obtained {
            path: path.clone(),
            download: None,
            _temporary: None,
        }),
        Source::Medium(medium) => {
            let temporary = temporary_in(&lookup.download_dir(environment))?;
            let path = temporary.path().join("boot.ipsw");
            eprintln!(
                "Reading the boot files of macOS {} from this stick",
                lookup.build
            );
            // The members a download reads, and no more.
            (|| -> Result<u64> {
                let mut image = medium::image::open(medium)?;
                let mut archive =
                    zip::ZipArchive::new(&mut image as &mut dyn restore::ImageSource)?;
                let names = (environment.bundle_inputs)(&mut archive, lookup.model)?;
                restore::copy_inputs(
                    &mut archive,
                    &names,
                    &path,
                    "Reading boot files from the stick",
                    None,
                )
            })()
            .with_context(|| format!("reading {}", medium.directory().display()))?;
            Ok(Obtained {
                path,
                download: None,
                _temporary: Some(temporary),
            })
        }
        Source::Remote { .. } => {
            eprintln!(
                "Downloading {}. No copy of it is on this computer; pass --ipsw FILE to use one \
                 you have",
                remote_text(source, lookup.build)
            );
            let directory = lookup.download_dir(environment);
            let (directory, temporary) = match lookup.need {
                // Kept, so a later command finds it.
                Need::BootFiles => (directory, None),
                // Only its boot files land here, until the USB holds them.
                Need::WholeImage | Need::Pieces => {
                    let temporary = temporary_in(&directory)?;
                    (temporary.path().to_path_buf(), Some(temporary))
                }
            };
            let fetched =
                (environment.fetch_image)(&directory, lookup.build, lookup.model, lookup.need)?;
            Ok(Obtained {
                path: fetched.path,
                download: fetched.download,
                _temporary: temporary,
            })
        }
    }
}

/// An image this computer holds that serves `need` for `build` and `model`
/// (`--download-dir` first, then every cache [`image_caches`] names): a new
/// setup takes it as `--ipsw` instead of downloading.
pub(super) fn cached_image(
    environment: &Environment<'_>,
    need: Need,
    build: &str,
    model: &str,
    download_dir: Option<&Path>,
) -> Option<PathBuf> {
    let lookup = Lookup {
        need,
        build,
        model,
        ipsw: None,
        download_dir,
        medium: None,
    };
    lookup
        .directories(environment)
        .iter()
        .find_map(|directory| cached(directory, &lookup))
}

/// A complete image of `lookup`'s build and model in `directory`, or for
/// boot files also a boot archive: the one a command downloaded, or the
/// one a first setup left.
fn cached(directory: &Path, lookup: &Lookup<'_>) -> Option<PathBuf> {
    let (build, model) = (lookup.build, lookup.model);
    let mut candidates = Vec::new();
    if lookup.need == Need::BootFiles {
        candidates.push(directory.join(super::prepare::inputs_name(build, model)));
        // A recovery archive an earlier stela downloaded holds them too.
        candidates.push(directory.join(super::prepare::recovery_name(build, model)));
        // Where a first setup keeps its downloads until it finishes.
        candidates.push(directory.join("setup").join("boot.ipsw"));
    }
    if let Ok(entries) = fs::read_dir(directory) {
        let mut images: Vec<PathBuf> = entries
            .filter_map(|entry| entry.ok())
            .map(|entry| entry.path())
            .filter(|path| {
                path.file_name()
                    .and_then(|name| name.to_str())
                    .is_some_and(|name| {
                        name.starts_with("UniversalMac_")
                            && name.ends_with(&format!("_{build}_Restore.ipsw"))
                    })
            })
            .collect();
        images.sort();
        candidates.extend(images);
    }
    candidates.into_iter().find(|path| {
        path.is_file()
            && restore::payload::identity(path, model).is_ok_and(|(_, found)| found == build)
            && (lookup.need == Need::BootFiles
                || restore::payload::holds_system_image(path).unwrap_or(false))
    })
}

/// The directories downloaded images may be in: this user's cache, and on
/// Unix the invoking user's under `sudo` and root's, so a setup run with
/// `sudo` and an update run without find each other's downloads, and a
/// dry run looks where the real run does.
pub(super) fn image_caches() -> Vec<PathBuf> {
    let own = super::download_dir();
    #[cfg(not(unix))]
    let directories = vec![own];
    #[cfg(unix)]
    let directories = {
        let mut directories = vec![own];
        let homes = std::env::var("SUDO_USER")
            .ok()
            .filter(|user| !user.is_empty() && user != "root")
            .and_then(|user| home_of(&user))
            .into_iter()
            .chain(std::iter::once(PathBuf::from(match std::env::consts::OS {
                "macos" => "/var/root",
                _ => "/root",
            })));
        for home in homes {
            let home = home.into_os_string();
            if let Some(cache) = super::prepare::cache_dir_in(std::env::consts::OS, |name| {
                (name == "HOME").then(|| home.clone())
            }) {
                let images = cache.join("images");
                if !directories.contains(&images) {
                    directories.push(images);
                }
            }
        }
        directories
    };
    directories
}

/// The home directory of the user `name`: `/etc/passwd`'s, or macOS's
/// `/Users/<name>`.
#[cfg(unix)]
fn home_of(name: &str) -> Option<PathBuf> {
    if !name
        .bytes()
        .all(|c| c.is_ascii_alphanumeric() || matches!(c, b'-' | b'_' | b'.'))
        || name.starts_with('.')
    {
        return None;
    }
    let passwd = fs::read_to_string("/etc/passwd").unwrap_or_default();
    passwd
        .lines()
        .filter_map(|line| {
            let fields: Vec<&str> = line.split(':').collect();
            (fields.len() >= 6 && fields[0] == name).then(|| PathBuf::from(fields[5]))
        })
        .find(|home| home.is_absolute())
        .or_else(|| (std::env::consts::OS == "macos").then(|| Path::new("/Users").join(name)))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn caches_start_with_this_users() {
        let caches = image_caches();
        assert_eq!(caches[0], super::super::download_dir());
        let mut unique = caches.clone();
        unique.dedup();
        assert_eq!(unique, caches);
    }

    #[cfg(unix)]
    #[test]
    fn invoking_user_names_are_checked() {
        assert_eq!(home_of("../etc"), None);
        assert_eq!(home_of(".hidden"), None);
        assert_eq!(home_of("a/b"), None);
        if cfg!(target_os = "linux") {
            assert_eq!(home_of("root"), Some(PathBuf::from("/root")));
        }
    }
}
