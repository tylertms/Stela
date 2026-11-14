//! Vendor firmware provisioning for a native-driver requirement inventory.
//! Network/cache policy stays in the CLI; the loader validates pins/packages.
use anyhow::{Context, Result, ensure};
use reqwest::{blocking::Client, header, redirect::Policy};
use std::{
    fs::{self, File},
    io::{Read, Write},
    path::Path,
    time::Duration,
};
use stela_drivers::firmware::{Requirement, Target};
use stela_loader::host_firmware::{
    self, MAX_MANIFEST_BYTES, bundle,
    recipes::{self, Artifact, Plan},
    requirements::{self, Inventory, Request},
};

fn read_cached(path: &Path, artifact: &Artifact) -> Result<Vec<u8>> {
    ensure!(
        !fs::symlink_metadata(path)?.is_symlink(),
        "firmware cache object is a symlink"
    );
    let mut file = File::open(path)?;
    ensure!(
        file.metadata()?.is_file() && file.metadata()?.len() == artifact.length as u64,
        "firmware cache object length differs from its pin"
    );
    let mut bytes = vec![0; artifact.length];
    file.read_exact(&mut bytes)?;
    ensure!(
        file.read(&mut [0])? == 0,
        "firmware cache object grew during read"
    );
    artifact.verify(&bytes)?;
    Ok(bytes)
}

/// The most one file's sources may make it wait in all. A source that asks
/// for a longer pause (a Retry-After of an hour, as GitLab's per-IP limit
/// sends) is left for the next source at once.
const WAIT_BUDGET: Duration = Duration::from_secs(30);
/// Requests to one source for one file that bring nothing new.
const ATTEMPTS: u32 = 3;
/// Requests to one source for one file that go on from where a cut
/// connection left the file: each brought more of it, so they wait only
/// [`RESUME_PAUSE`] and leave the budget alone.
const RESUMES: u32 = 8;
const RESUME_PAUSE: Duration = Duration::from_secs(1);
/// How long a download may go without a byte. There is no limit on the
/// whole transfer: a GPU's 64 MB firmware takes many minutes on a slow
/// network.
const IDLE_TIMEOUT: Duration = Duration::from_secs(60);

/// How one request to a source ended, when it brought no file.
enum Answer {
    /// A rate limit or a source down for now (429, 503), with the pause
    /// its Retry-After asks for.
    Busy(Option<Duration>, String),
    /// A failure the same request may not meet again.
    Transient(String),
    /// The connection ended inside the file; whether it brought bytes the
    /// part kept so far lacked.
    Cut(String, bool),
    /// An answer asking again would not change.
    Final(String),
}

/// The pause a `Retry-After` header asks for, in seconds; an HTTP date is
/// taken as a long pause.
fn retry_after(response: &reqwest::blocking::Response) -> Option<Duration> {
    let value = response.headers().get(header::RETRY_AFTER)?.to_str().ok()?;
    Some(
        value
            .trim()
            .parse::<u64>()
            .map_or(Duration::MAX, Duration::from_secs),
    )
}

/// The first byte, last byte and length a `Content-Range: bytes a-b/n`
/// header names.
fn content_range(response: &reqwest::blocking::Response) -> Option<(u64, u64, u64)> {
    let value = response
        .headers()
        .get(header::CONTENT_RANGE)?
        .to_str()
        .ok()?;
    let (range, total) = value.trim().strip_prefix("bytes ")?.split_once('/')?;
    let (first, last) = range.split_once('-')?;
    Some((first.parse().ok()?, last.parse().ok()?, total.parse().ok()?))
}

/// A broken transfer in words: the HTTP client's error with every cause
/// under it, which says whether the server closed the connection or went
/// quiet.
fn transfer_error(error: &std::io::Error) -> String {
    let timed_out = error.kind() == std::io::ErrorKind::TimedOut
        || error
            .get_ref()
            .and_then(|inner| inner.downcast_ref::<reqwest::Error>())
            .is_some_and(reqwest::Error::is_timeout);
    crate::net::explain_causes(error, timed_out)
}

fn mib(bytes: usize) -> String {
    format!("{:.1}", bytes as f64 / MIB as f64)
}

/// Ask `url` for `artifact`, going on from the `part` of it earlier
/// requests brought with a Range request. On success `part` holds the
/// checked file; after a cut it holds what arrived, for the next request.
fn request(
    client: &Client,
    url: &str,
    artifact: &Artifact,
    part: &mut Vec<u8>,
) -> Result<(), Answer> {
    let had = part.len();
    let mut get = client.get(url).header(header::ACCEPT_ENCODING, "identity");
    if had > 0 {
        get = get.header(header::RANGE, format!("bytes={had}-"));
    }
    let mut response = match get.send() {
        Ok(response) => response,
        Err(error) if crate::net::transient(&error) => {
            return Err(Answer::Transient(crate::net::explain(&error)));
        }
        Err(error) => return Err(Answer::Final(crate::net::explain(&error))),
    };
    let status = response.status();
    let length = artifact.length as u64;
    let start = match status.as_u16() {
        // The whole file: a server without ranges sends it again.
        200 => 0,
        206 if had > 0 => {
            if content_range(&response) != Some((had as u64, length - 1, length)) {
                part.clear();
                return Err(Answer::Transient(
                    "sent a range other than the one asked for".into(),
                ));
            }
            had
        }
        416 if had > 0 => {
            part.clear();
            return Err(Answer::Transient(format!("answered {status} to a resume")));
        }
        429 | 503 => {
            let after = retry_after(&response);
            let asked = match after {
                Some(after) if after != Duration::MAX => {
                    format!(", Retry-After {} s", after.as_secs())
                }
                Some(_) => ", Retry-After a date".into(),
                None => String::new(),
            };
            return Err(Answer::Busy(after, format!("answered {status}{asked}")));
        }
        _ if crate::net::transient_status(status) => {
            return Err(Answer::Transient(format!("answered {status}")));
        }
        _ => return Err(Answer::Final(format!("answered {status}"))),
    };
    part.truncate(start);
    let remaining = length - start as u64;
    if response.content_length().is_some_and(|n| n != remaining) {
        part.clear();
        return Err(Answer::Final("sent a length other than the pin's".into()));
    }
    let cut = response
        .by_ref()
        .take(remaining + 1)
        .read_to_end(part)
        .err();
    if let Some(error) = cut.as_ref().map(transfer_error).or_else(|| {
        (part.len() < artifact.length).then(|| "the server closed the connection".to_owned())
    }) {
        let what = format!(
            "transfer cut at {} of {} MiB: {error}",
            mib(part.len()),
            mib(artifact.length)
        );
        return Err(Answer::Cut(what, part.len() > had));
    }
    if artifact.verify(part).is_err() {
        part.clear();
        let what = "sent bytes whose length or SHA-256 differs from the pin";
        // The ends of a resumed file may not join: fetch it whole once.
        return Err(if start > 0 {
            Answer::Transient(format!("{what} after a resume"))
        } else {
            Answer::Final(what.into())
        });
    }
    Ok(())
}

fn host_of(url: &str) -> &str {
    url.split("://")
        .nth(1)
        .and_then(|rest| rest.split('/').next())
        .unwrap_or(url)
}

fn download(client: &Client, artifact: &Artifact) -> Result<Vec<u8>> {
    download_from(client, artifact, &artifact.urls(), &mut std::thread::sleep)
}

/// Download `artifact` from the first of `urls` that sends its pinned
/// bytes. A connection cut inside the file goes on from where it stopped,
/// up to [`RESUMES`] times. A source that is busy waits only within
/// [`WAIT_BUDGET`]; past it, or after [`ATTEMPTS`] requests that bring
/// nothing, the next source is asked at once. The error names the file and
/// what each source answered.
fn download_from(
    client: &Client,
    artifact: &Artifact,
    urls: &[String],
    sleep: &mut dyn FnMut(Duration),
) -> Result<Vec<u8>> {
    let mut waited = Duration::ZERO;
    let mut answers = Vec::new();
    for url in urls {
        let host = host_of(url);
        let mut part = Vec::with_capacity(artifact.length);
        let mut backoff = Duration::from_secs(1);
        let mut attempt = 1;
        let mut resumes = 0;
        let last = loop {
            let pause = match request(client, url, artifact, &mut part) {
                Ok(()) => return Ok(part),
                Err(Answer::Cut(what, true)) if resumes < RESUMES => {
                    eprintln!(
                        "note: {} from {host}: {what}; going on from there in {} s",
                        artifact.member,
                        RESUME_PAUSE.as_secs()
                    );
                    sleep(RESUME_PAUSE);
                    resumes += 1;
                    continue;
                }
                Err(Answer::Busy(Some(after), what)) => (Some(after), what),
                Err(Answer::Busy(None, what) | Answer::Transient(what) | Answer::Cut(what, _)) => {
                    (Some(backoff), what)
                }
                Err(Answer::Final(what)) => (None, what),
            };
            match pause {
                (Some(pause), what)
                    if attempt < ATTEMPTS
                        && waited.checked_add(pause).is_some_and(|w| w <= WAIT_BUDGET) =>
                {
                    eprintln!(
                        "note: {} from {host}: {what}; asking again in {} s",
                        artifact.member,
                        pause.as_secs().max(1)
                    );
                    sleep(pause);
                    waited += pause;
                    backoff *= 2;
                    attempt += 1;
                }
                (_, what) => break what,
            }
        };
        if urls.len() > 1 {
            eprintln!("note: {} from {host}: {last}", artifact.member);
        }
        answers.push(format!("{host} {last}"));
    }
    anyhow::bail!(
        "cannot download {}: {}",
        artifact.member,
        answers.join("; ")
    )
}

fn cached(client: &Client, directory: &Path, artifact: &Artifact) -> Result<Vec<u8>> {
    let path = directory.join(artifact.sha256);
    match fs::symlink_metadata(&path) {
        Ok(_) => return read_cached(&path, artifact),
        Err(error) if error.kind() == std::io::ErrorKind::NotFound => {}
        Err(error) => return Err(error.into()),
    }
    let bytes = download(client, artifact)?;
    store(directory, artifact, &bytes)
}

/// Keep `bytes`, `artifact`'s checked download, in the cache `directory`.
fn store(directory: &Path, artifact: &Artifact, bytes: &[u8]) -> Result<Vec<u8>> {
    artifact.verify(bytes)?;
    let path = directory.join(artifact.sha256);
    fs::create_dir_all(directory)?;
    ensure!(
        !fs::symlink_metadata(directory)?.is_symlink(),
        "firmware cache directory is a symlink"
    );
    let mut temporary = tempfile::NamedTempFile::new_in(directory)?;
    temporary.write_all(bytes)?;
    temporary.as_file().sync_all()?;
    crate::efi::persist_new(temporary, &path)
        .with_context(|| format!("cannot publish immutable firmware cache {}", path.display()))?;
    host_firmware::sync_directory(directory)?;
    read_cached(&path, artifact)
}

/// Notes about this computer's NVIDIA GPUs, for information only: the stick
/// holds every supported GPU's firmware whatever this computer has.
pub(super) fn host_notes(host: &dyn super::host::Host) -> Result<Vec<String>> {
    let mut notes = Vec::new();
    for device in host.nvidia_gpus()? {
        if recipes::nvidia_chip(device).is_none() {
            notes.push(format!(
                "this computer's NVIDIA GPU (device {device:04x}) is not one Stela supports \
                 ({}); the stick still works on a PC that has one",
                stela_boot_core::gpu_status::SUPPORTED
            ));
        }
    }
    Ok(notes)
}

/// The firmware line of a setup plan.
pub(super) fn bundle_summary() -> String {
    let (count, bytes) = bundle::size();
    format!(
        "every supported device: {count} files, {:.1} MiB (each PC loads only its own)",
        bytes as f64 / MIB as f64
    )
}

/// Download every file of the complete bundle that neither the cache nor
/// the stick's data volume `volume` (when it exists yet) holds, before
/// setup, `repair` or `update` changes anything on the stick: a failed
/// download leaves the stick as it was. Nothing for a stela.efi
/// (`mount_image`) that reads no bundle.
pub(super) fn prefetch_bundle(mount_image: &[u8], volume: Option<&Path>) -> Result<()> {
    if !host_firmware::reads_bundle(mount_image) {
        return Ok(());
    }
    let client = client()?;
    prefetch(
        &bundle::complete(),
        &super::cache_dir().join("firmware"),
        volume
            .map(|volume| volume.join(bundle::DIRECTORY))
            .as_deref(),
        &mut |artifact| download(&client, artifact),
    )
    .context("the firmware bundle could not be downloaded, so the stick was left as it was")?;
    Ok(())
}

/// [`prefetch_bundle`] of `files` into `cache` through `download`; the
/// files `on_stick` already holds by hash are not downloaded. Returns how
/// many files were downloaded.
fn prefetch(
    files: &[(&Artifact, &Artifact)],
    cache: &Path,
    on_stick: Option<&Path>,
    download: &mut dyn FnMut(&Artifact) -> Result<Vec<u8>>,
) -> Result<usize> {
    let mut wanted: Vec<(&Artifact, String)> = Vec::new();
    for (object, notice) in files {
        for (artifact, path) in [
            (*object, bundle::object_path(object.sha256)),
            (*notice, bundle::notice_path(notice.sha256)),
        ] {
            if wanted
                .iter()
                .any(|(seen, _)| seen.sha256 == artifact.sha256)
            {
                continue;
            }
            let cached = holds(&cache.join(artifact.sha256), artifact);
            let kept = on_stick.is_some_and(|directory| holds(&directory.join(&path), artifact));
            if !cached && !kept {
                wanted.push((artifact, path));
            }
        }
    }
    if wanted.is_empty() {
        return Ok(0);
    }
    let bytes: usize = wanted.iter().map(|(artifact, _)| artifact.length).sum();
    eprintln!(
        "Downloading the firmware bundle: {} files, {:.1} MiB",
        wanted.len(),
        bytes as f64 / MIB as f64
    );
    for (index, (artifact, _)) in wanted.iter().enumerate() {
        eprintln!(
            "  {} ({:.1} MiB), file {} of {}",
            artifact.member,
            artifact.length as f64 / MIB as f64,
            index + 1,
            wanted.len()
        );
        let bytes = download(artifact)?;
        store(cache, artifact, &bytes)?;
    }
    Ok(wanted.len())
}

const MIB: u64 = 1024 * 1024;
/// Room a volume short of space keeps free beside the bundle.
const MARGIN: u64 = 16 * MIB;

/// The bundle's files that go on a volume with `free` bytes left (`None`:
/// no limit) beside `present` bytes the bundle already holds there, in
/// [`bundle::FAMILIES`] order, and the files left out.
pub(super) fn bundle_plan(
    free: Option<u64>,
    present: u64,
) -> (
    Vec<(&'static Artifact, &'static Artifact)>,
    Vec<&'static Artifact>,
) {
    let mut room = free.map(|free| (free + present).saturating_sub(MARGIN));
    let mut kept = Vec::new();
    let mut left = Vec::new();
    for (object, notice) in bundle::complete() {
        let length = (object.length + notice.length) as u64;
        match &mut room {
            Some(room) if *room < length => left.push(object),
            Some(room) => {
                *room -= length;
                kept.push((object, notice));
            }
            None => kept.push((object, notice)),
        }
    }
    (kept, left)
}

/// Whether `path` holds `artifact`'s exact bytes.
fn holds(path: &Path, artifact: &Artifact) -> bool {
    fs::symlink_metadata(path).is_ok_and(|m| m.is_file() && m.len() == artifact.length as u64)
        && read_cached(path, artifact).is_ok()
}

/// Bytes of `files` that `directory` already holds.
fn present_bytes(directory: &Path) -> u64 {
    bundle::complete()
        .iter()
        .flat_map(|(object, notice)| {
            [
                (bundle::object_path(object.sha256), object.length),
                (bundle::notice_path(notice.sha256), notice.length),
            ]
        })
        .filter_map(|(path, length)| {
            fs::metadata(directory.join(path))
                .is_ok_and(|m| m.len() == length as u64)
                .then_some(length as u64)
        })
        .sum()
}

/// Write `files` and their notices to the bundle on `volume`, each fetched
/// through `fetch` unless the volume holds it already, then its manifest,
/// and remove files no longer listed.
pub(super) fn publish_bundle(
    volume: &Path,
    files: &[(&Artifact, &Artifact)],
    fetch: &mut dyn FnMut(&Artifact) -> Result<Vec<u8>>,
) -> Result<()> {
    let directory = volume.join(bundle::DIRECTORY);
    fs::create_dir_all(directory.join("licenses"))?;
    let mut kept = vec![directory.join("bundle.json")];
    for (object, notice) in files {
        for (artifact, path) in [
            (*object, bundle::object_path(object.sha256)),
            (*notice, bundle::notice_path(notice.sha256)),
        ] {
            let path = directory.join(path);
            if !kept.contains(&path) && !holds(&path, artifact) {
                let bytes = fetch(artifact)?;
                artifact.verify(&bytes)?;
                let parent = path.parent().context("bundle path has no directory")?;
                let mut temporary = tempfile::NamedTempFile::new_in(parent)?;
                temporary.write_all(&bytes)?;
                temporary.as_file().sync_all()?;
                temporary.persist(&path)?;
            }
            kept.push(path);
        }
    }
    let manifest = bundle::manifest(files)?;
    let path = directory.join("bundle.json");
    if fs::read(&path).ok().as_deref() != Some(manifest.as_slice()) {
        let mut temporary = tempfile::NamedTempFile::new_in(&directory)?;
        temporary.write_all(&manifest)?;
        temporary.as_file().sync_all()?;
        temporary.persist(&path)?;
    }
    for listing in [directory.clone(), directory.join("licenses")] {
        for entry in fs::read_dir(&listing)? {
            let path = entry?.path();
            if path.is_file() && !kept.contains(&path) {
                fs::remove_file(path)?;
            }
        }
    }
    Ok(())
}

/// Put the complete bundle on the stick's data volume `volume`, beside the
/// EFI partition `esp` whose boot program is `mount_image`, downloading
/// what the cache lacks. A volume short of room gets what fits, in
/// [`bundle::FAMILIES`] order, and the notes name what was left out. Once
/// the bundle holds every file of an older per-PC set on the EFI partition,
/// that set is removed. Nothing depends on this computer's devices.
pub(super) fn provision_bundle(
    host: &dyn super::host::Host,
    volume: &Path,
    esp: &Path,
    mount_image: &[u8],
) -> Result<Vec<String>> {
    if !host_firmware::reads_bundle(mount_image) {
        return Ok(Vec::new());
    }
    let directory = volume.join(bundle::DIRECTORY);
    let free = host.volume(volume)?.free_bytes;
    let (files, left) = bundle_plan(Some(free), present_bytes(&directory));
    let bytes: usize = files.iter().map(|(object, _)| object.length).sum();
    eprintln!(
        "Firmware for every supported device: {} files, {:.1} MiB, to {}",
        files.len(),
        bytes as f64 / MIB as f64,
        directory.display()
    );
    // Every download before the first write to the stick.
    prefetch_bundle(mount_image, Some(volume))?;
    let cache = super::cache_dir().join("firmware");
    publish_bundle(volume, &files, &mut |artifact| {
        read_cached(&cache.join(artifact.sha256), artifact)
    })
    .context("the stick did not get the complete firmware bundle")?;
    let mut notes = Vec::new();
    if !left.is_empty() {
        notes.push(format!(
            "{} has no room for {} of the firmware bundle's files, so PCs that need them \
             lack them: {}",
            volume.display(),
            left.len(),
            left.iter()
                .map(|artifact| artifact.member)
                .collect::<Vec<_>>()
                .join(", ")
        ));
    }
    remove_superseded(esp, volume, &files)?;
    Ok(notes)
}

/// Remove the older per-PC firmware on the EFI partition when the bundle on
/// another volume holds every one of its files.
fn remove_superseded(esp: &Path, volume: &Path, files: &[(&Artifact, &Artifact)]) -> Result<()> {
    let old = esp.join(host_firmware::DIRECTORY);
    if esp == volume {
        return Ok(());
    }
    let Some(bytes) = installed_manifest(&old)? else {
        return Ok(());
    };
    let manifest = host_firmware::Manifest::parse(&bytes)?;
    if manifest.images.iter().all(|image| {
        files
            .iter()
            .any(|(object, _)| object.sha256 == image.sha256)
    }) {
        fs::remove_dir_all(&old)?;
    }
    Ok(())
}

/// What `stela status` says of the bundle on `volume`.
pub(super) fn bundle_status(volume: &Path) -> std::result::Result<String, String> {
    let path = volume.join(bundle::MANIFEST);
    let Ok(bytes) = fs::read(&path) else {
        // Files without the manifest, which setup writes last: a bundle
        // whose writing stopped.
        let directory = volume.join(bundle::DIRECTORY);
        let present = bundle::complete()
            .iter()
            .filter(|(object, _)| {
                fs::metadata(directory.join(bundle::object_path(object.sha256)))
                    .is_ok_and(|m| m.len() == object.length as u64)
            })
            .count();
        return Err(if present == 0 {
            "the stick has no firmware bundle".into()
        } else {
            format!(
                "the firmware bundle is incomplete: {present} of {} files are on the stick",
                bundle::size().0
            )
        });
    };
    let parsed = bundle::Bundle::parse(&bytes).map_err(|error| error.0)?;
    let directory = volume.join(bundle::DIRECTORY);
    let damaged = parsed
        .objects
        .iter()
        .filter(|object| {
            fs::metadata(directory.join(object.path))
                .map_or(true, |m| m.len() != object.length as u64)
        })
        .count();
    let missing = parsed.missing();
    let (count, bytes) = bundle::size();
    if missing.is_empty() && damaged == 0 {
        return Ok(format!(
            "complete: {count} files, {:.1} MiB",
            bytes as f64 / MIB as f64
        ));
    }
    Err(format!(
        "the bundle lacks {} of {count} files{}{}",
        missing.len(),
        if missing.is_empty() {
            String::new()
        } else {
            format!(
                " ({})",
                missing
                    .iter()
                    .map(|artifact| artifact.member)
                    .collect::<Vec<_>>()
                    .join(", ")
            )
        },
        if damaged == 0 {
            String::new()
        } else {
            format!(" and {damaged} of its files are damaged")
        }
    ))
}

fn client() -> Result<Client> {
    Ok(Client::builder()
        .user_agent(concat!("stela/", env!("CARGO_PKG_VERSION")))
        .connect_timeout(Duration::from_secs(30))
        // The blocking client applies this to each read, not the transfer.
        .timeout(IDLE_TIMEOUT)
        .redirect(Policy::none())
        .build()?)
}

/// The firmware manifest in `destination`, when it has one.
fn installed_manifest(destination: &Path) -> Result<Option<Vec<u8>>> {
    let previous = destination.join("manifest.json");
    if !previous.try_exists()? {
        return Ok(None);
    }
    let mut bytes = Vec::new();
    File::open(&previous)?
        .take(MAX_MANIFEST_BYTES as u64 + 1)
        .read_to_end(&mut bytes)?;
    Ok(Some(bytes))
}

/// Whether `image` is the exact binding `request` asks for.
fn binds(image: &host_firmware::Image<'_>, request: &Request) -> bool {
    request.name == image.name
        && request.target.driver == image.target.driver
        && request.target.hardware == image.target.hardware
        && request.target.revision == image.target.revision
        && request.target.board.as_deref() == image.target.board
}

/// What the newest firmware inventory in a USB's boot log asks for that the
/// USB's firmware lacks: files with a pinned source, which `repair` and
/// `update stela` add, and files without one, which they name.
#[derive(Debug, Default)]
pub(super) struct Requested {
    pub(super) pinned: Vec<Request>,
    pub(super) unpinned: Vec<Request>,
    /// Why the newest inventory line holds none.
    pub(super) unreadable: Option<String>,
}

/// A missing file as the reports name it: its name and the driver's target.
pub(super) fn describe(request: &Request) -> String {
    let target = &request.target;
    let board = target
        .board
        .as_deref()
        .map(|board| format!(" board {board}"))
        .unwrap_or_default();
    format!(
        "{} ({} {} {}{board})",
        request.name, target.driver, target.hardware, target.revision
    )
}

impl Requested {
    pub(super) fn is_empty(&self) -> bool {
        self.pinned.is_empty() && self.unpinned.is_empty() && self.unreadable.is_none()
    }

    /// The lines `repair` and `update stela` report.
    pub(super) fn lines(&self) -> Vec<String> {
        let mut lines = Vec::new();
        if let Some(why) = &self.unreadable {
            lines.push(format!(
                "the boot log's newest firmware request is unreadable: {why}"
            ));
        }
        for request in &self.pinned {
            lines.push(format!(
                "add {}, which the last boot asked for",
                describe(request)
            ));
        }
        for request in &self.unpinned {
            lines.push(format!(
                "the last boot asked for {}, which Stela has no pinned source for; the device \
                 stays off",
                describe(request)
            ));
        }
        lines
    }

    /// Download the pinned files and publish them with what the stick's
    /// firmware on `esp` already holds.
    pub(super) fn provision(&self, esp: &Path, mount_image: &[u8]) -> Result<()> {
        self.plan(
            esp,
            mount_image,
            &super::cache_dir().join("firmware"),
            false,
        )
        .context("the stick did not get the firmware the last boot asked for")?;
        Ok(())
    }

    /// [`Self::provision`] through `cache`; a dry run only plans.
    fn plan(
        &self,
        esp: &Path,
        mount_image: &[u8],
        cache: &Path,
        dry_run: bool,
    ) -> Result<Option<String>> {
        if self.pinned.is_empty() {
            return Ok(None);
        }
        let destination = esp.join(host_firmware::DIRECTORY);
        let mut inventory = Inventory::default();
        for request in &self.pinned {
            inventory.observe(request.view())?;
        }
        keep_installed(&mut inventory, &destination, true)?;
        publish_plan(&inventory, &destination, cache, dry_run, mount_image).map(Some)
    }
}

/// What the newest inventory in the boot log on `esp` asks for that the
/// USB's firmware lacks, for a boot program `mount_image`. A USB without a
/// log or without such a line, and a boot program that reads no firmware
/// from the stick, need nothing.
pub(super) fn requested(esp: &Path, mount_image: &[u8]) -> Result<Requested> {
    if !host_firmware::supports_mount(mount_image) {
        return Ok(Requested::default());
    }
    let Some(log) = fs::read(esp.join(crate::efi::BOOT_LOG))
        .ok()
        .and_then(|region| stela_runtime::pc::boot_log::read(&region).ok())
    else {
        return Ok(Requested::default());
    };
    let inventory = match requirements::newest(&log.stream) {
        None => return Ok(Requested::default()),
        Some(Ok(inventory)) => inventory,
        Some(Err(error)) => {
            return Ok(Requested {
                unreadable: Some(error.0),
                ..Requested::default()
            });
        }
    };
    let manifest = installed_manifest(&esp.join(host_firmware::DIRECTORY))?;
    let installed = manifest
        .as_deref()
        .map(host_firmware::Manifest::parse)
        .transpose()?;
    let mut requested = Requested::default();
    for request in inventory.requests() {
        if requirements::read_by_boot(&request.view().target)
            || installed
                .as_ref()
                .is_some_and(|manifest| manifest.images.iter().any(|image| binds(image, request)))
        {
            continue;
        }
        if recipes::recipe(request).is_some() {
            // The bundle holds every pinned file: `repair` completes it.
            if !host_firmware::reads_bundle(mount_image) {
                requested.pinned.push(request.clone());
            }
        } else {
            requested.unpinned.push(request.clone());
        }
    }
    Ok(requested)
}

/// Require what the firmware in `destination` holds, when `add` says the
/// inventory adds to it; refuse an inventory that would remove a binding.
fn keep_installed(inventory: &mut Inventory, destination: &Path, add: bool) -> Result<()> {
    if let Some(bytes) = installed_manifest(destination)? {
        let previous = host_firmware::Manifest::parse(&bytes)?;
        if add {
            for image in &previous.images {
                inventory.observe(Requirement {
                    name: image.name,
                    target: Target {
                        driver: image.target.driver,
                        hardware: image.target.hardware,
                        revision: image.target.revision,
                        board: image.target.board,
                    },
                    minimum_bytes: 1,
                })?;
            }
        }
        for image in &previous.images {
            ensure!(
                inventory
                    .requests()
                    .iter()
                    .any(|request| binds(image, request)),
                "observed requirements do not cover installed firmware binding {}; refusing to remove it",
                image.name
            );
        }
    }
    Ok(())
}

/// Plan `inventory`'s pinned files and, unless `dry_run`, download them into
/// `cache` and publish them to `destination`.
fn publish_plan(
    inventory: &Inventory,
    destination: &Path,
    cache: &Path,
    dry_run: bool,
    mount_image: &[u8],
) -> Result<String> {
    let plan = Plan::select(inventory)?;
    host_firmware::MountLimits::from_image(mount_image)
        .context("stela.efi has no supported host firmware mount capability")?
        .check_lengths(
            plan.selections()
                .iter()
                .map(|selection| selection.recipe.image.length),
        )?;
    let mut report = format!(
        "Host firmware: {} exact observed bindings, {} pinned image/license artifacts\n",
        plan.selections().len(),
        plan.artifacts().len()
    );
    for selection in plan.selections() {
        use std::fmt::Write as _;
        writeln!(
            report,
            "  {} {:?}: {} bytes SHA-256 {}",
            selection.request.name,
            selection.request.target,
            selection.recipe.image.length,
            selection.recipe.image.sha256
        )?;
    }
    if dry_run {
        report.push_str("Dry run: no downloads or writes.\n");
        return Ok(report);
    }
    let client = client()?;
    let package = plan.package(|artifact| {
        cached(&client, cache, artifact)
            .map_err(|e| host_firmware::Error(format!("firmware provisioning: {e:#}")))
    })?;
    package.check_mount(mount_image)?;
    package.publish(destination)?;
    report.push_str("Published pinned firmware and accompanying redistribution notices.\n");
    Ok(report)
}

#[cfg(test)]
mod tests {
    use super::*;
    #[test]
    fn the_bundle_plan_holds_every_recipe_whatever_the_computer() {
        let (files, left) = bundle_plan(None, 0);
        assert!(left.is_empty());
        for recipe in bundle::recipes() {
            assert!(
                files
                    .iter()
                    .any(|(object, notice)| object.sha256 == recipe.image.sha256
                        && notice.sha256 == recipe.license.sha256),
                "{}",
                recipe.image.member
            );
        }
        let manifest = bundle::manifest(&files).unwrap();
        let parsed = bundle::Bundle::parse(&manifest).unwrap();
        assert!(parsed.missing().is_empty());
        assert_eq!(parsed.objects.len(), 87);
        assert!(bundle_summary().starts_with("every supported device: 87 files, 115.8 MiB"));
        // A volume short of room keeps the small files and leaves the large
        // GPU images out, by name.
        let (files, left) = bundle_plan(Some(MARGIN + 40 * MIB), 0);
        assert!(
            files
                .iter()
                .any(|(object, _)| object.member == "regulatory.db")
        );
        let left: Vec<_> = left.iter().map(|artifact| artifact.member).collect();
        assert!(
            left.contains(&"nvidia/ga102/gsp/gsp-570.144.bin"),
            "{left:?}"
        );
        assert!(
            left.contains(&"nvidia/tu102/gsp/gsp-570.144.bin"),
            "{left:?}"
        );
        assert!(!left.contains(&"regulatory.db"));
        // Files the volume already holds count as room.
        let (_, left) = bundle_plan(Some(MARGIN), 200 * MIB);
        assert!(left.is_empty());
    }

    /// A synthetic catalog: files whose digests these tests can satisfy.
    fn synthetic() -> Vec<(Artifact, Vec<u8>)> {
        use sha2::{Digest, Sha256};
        let artifact = |member: &'static str, bytes: &[u8]| {
            let sha256: &'static str =
                Box::leak(hex::encode(Sha256::digest(bytes)).into_boxed_str());
            (
                Artifact {
                    source: "https://vendor.invalid",
                    revision: "synthetic",
                    member,
                    length: bytes.len(),
                    sha256,
                },
                bytes.to_vec(),
            )
        };
        vec![
            artifact("vendor/a.bin", b"first file"),
            artifact("vendor/b.bin", b"second file"),
            artifact("LICENSES/LICENCE.vendor", b"notice"),
        ]
    }

    fn snapshot(directory: &Path) -> Vec<(String, Vec<u8>)> {
        let mut files = Vec::new();
        for entry in walk(directory) {
            files.push((
                entry
                    .strip_prefix(directory)
                    .unwrap()
                    .display()
                    .to_string()
                    .replace('\\', "/"),
                fs::read(&entry).unwrap(),
            ));
        }
        files.sort();
        files
    }

    fn walk(directory: &Path) -> Vec<std::path::PathBuf> {
        let mut found = Vec::new();
        for entry in fs::read_dir(directory).unwrap() {
            let path = entry.unwrap().path();
            if path.is_dir() {
                found.extend(walk(&path));
            } else {
                found.push(path);
            }
        }
        found
    }

    #[test]
    fn the_bundle_is_the_same_on_every_computer_and_stores_a_shared_file_once() {
        use super::super::{host::Host, tests::FakeHost};
        let catalog = synthetic();
        let [(a, a_bytes), (b, b_bytes), (notice, notice_bytes)] = &catalog[..] else {
            unreachable!()
        };
        // Two names for the first file's content.
        let alias = Artifact {
            member: "vendor/alias.bin",
            ..*a
        };
        let mut files: Vec<(&Artifact, &Artifact)> = Vec::new();
        for object in [a, &alias, b] {
            if !files.iter().any(|(seen, _)| seen.sha256 == object.sha256) {
                files.push((object, notice));
            }
        }
        let fetches = std::cell::Cell::new(0);
        let mut fetch = |artifact: &Artifact| -> Result<Vec<u8>> {
            fetches.set(fetches.get() + 1);
            Ok([a_bytes, b_bytes, notice_bytes]
                .into_iter()
                .zip([a, b, notice])
                .find(|(_, candidate)| candidate.sha256 == artifact.sha256)
                .unwrap()
                .0
                .clone())
        };
        // A computer with an NVIDIA GPU and Wi-Fi, and one with neither.
        let with = FakeHost::new(Default::default(), Vec::new());
        with.nvidia_gpus.replace(vec![0x2c02]);
        let without = FakeHost::new(Default::default(), Vec::new());
        let mut snapshots = Vec::new();
        for host in [&with as &dyn Host, &without] {
            let dir = tempfile::tempdir().unwrap();
            assert!(host.nvidia_gpus().is_ok());
            publish_bundle(dir.path(), &files, &mut fetch).unwrap();
            snapshots.push(snapshot(dir.path()));
            // A second publication fetches nothing.
            let before = fetches.get();
            publish_bundle(dir.path(), &files, &mut fetch).unwrap();
            assert_eq!(fetches.get(), before);
        }
        assert_eq!(snapshots[0], snapshots[1]);
        // Two files and one notice: the shared content once.
        let names: Vec<_> = snapshots[0].iter().map(|(name, _)| name.as_str()).collect();
        assert_eq!(names.len(), 4, "{names:?}");
        assert!(names.contains(&"stela/firmware/bundle.json"));
        let manifest = &snapshots[0]
            .iter()
            .find(|(name, _)| name == "stela/firmware/bundle.json")
            .unwrap()
            .1;
        let parsed = bundle::Bundle::parse(manifest).unwrap();
        assert_eq!(parsed.objects.len(), 2);
        assert_eq!(parsed.notices.len(), 1);
        // A damaged file is fetched again; a file no longer listed goes.
        let dir = tempfile::tempdir().unwrap();
        publish_bundle(dir.path(), &files, &mut fetch).unwrap();
        let stored = dir
            .path()
            .join(bundle::DIRECTORY)
            .join(bundle::object_path(a.sha256));
        fs::write(&stored, b"first filX").unwrap();
        publish_bundle(dir.path(), &files[..1], &mut fetch).unwrap();
        assert_eq!(fs::read(&stored).unwrap(), *a_bytes);
        assert!(
            !dir.path()
                .join(bundle::DIRECTORY)
                .join(bundle::object_path(b.sha256))
                .exists()
        );
    }

    #[test]
    fn prefetch_downloads_only_what_neither_the_cache_nor_the_stick_holds() {
        let catalog = synthetic();
        let [(a, a_bytes), (b, b_bytes), (notice, notice_bytes)] = &catalog[..] else {
            unreachable!()
        };
        let files: Vec<(&Artifact, &Artifact)> = vec![(a, notice), (b, notice)];
        let dir = tempfile::tempdir().unwrap();
        let cache = dir.path().join("cache");
        let volume = dir.path().join("volume");
        let stick = volume.join(bundle::DIRECTORY);
        // The stick kept the first file from a stopped setup.
        fs::create_dir_all(&stick).unwrap();
        fs::write(stick.join(bundle::object_path(a.sha256)), a_bytes).unwrap();
        let before = snapshot(&volume);
        let asked = std::cell::RefCell::new(Vec::new());
        let fail_on = std::cell::Cell::new(Some(b.sha256));
        let mut download = |artifact: &Artifact| -> Result<Vec<u8>> {
            asked.borrow_mut().push(artifact.member);
            if fail_on.get() == Some(artifact.sha256) {
                anyhow::bail!("cannot download {}: every source refused", artifact.member);
            }
            Ok([a_bytes, b_bytes, notice_bytes]
                .into_iter()
                .zip([a, b, notice])
                .find(|(_, candidate)| candidate.sha256 == artifact.sha256)
                .unwrap()
                .0
                .clone())
        };
        // A failed download: the stick is as it was, the cache keeps what
        // came.
        let error = prefetch(&files, &cache, Some(&stick), &mut download).unwrap_err();
        assert!(error.to_string().contains("vendor/b.bin"), "{error}");
        assert_eq!(snapshot(&volume), before);
        assert_eq!(*asked.borrow(), ["LICENSES/LICENCE.vendor", "vendor/b.bin"]);
        // Run again: only the failed file is asked for, once.
        fail_on.set(None);
        asked.borrow_mut().clear();
        assert_eq!(
            prefetch(&files, &cache, Some(&stick), &mut download).unwrap(),
            1
        );
        assert_eq!(*asked.borrow(), ["vendor/b.bin"]);
        assert_eq!(snapshot(&volume), before);
        // Then nothing; and publishing from the cache alone completes the
        // bundle, keeping the stick's file.
        assert_eq!(
            prefetch(&files, &cache, Some(&stick), &mut download).unwrap(),
            0
        );
        let mut from_cache =
            |artifact: &Artifact| read_cached(&cache.join(artifact.sha256), artifact);
        publish_bundle(&volume, &files, &mut from_cache).unwrap();
        let manifest = fs::read(stick.join("bundle.json")).unwrap();
        assert_eq!(bundle::Bundle::parse(&manifest).unwrap().objects.len(), 2);
        assert!(
            !cache.join(a.sha256).exists(),
            "the stick's file was never downloaded"
        );
    }

    #[test]
    fn status_names_a_partly_written_bundle() {
        let dir = tempfile::tempdir().unwrap();
        let directory = dir.path().join(bundle::DIRECTORY);
        fs::create_dir_all(&directory).unwrap();
        // Three files of a setup whose download stopped, no manifest yet.
        for (object, _) in bundle::complete().iter().take(3) {
            fs::write(
                directory.join(bundle::object_path(object.sha256)),
                vec![0; object.length],
            )
            .unwrap();
        }
        let status = bundle_status(dir.path()).unwrap_err();
        assert_eq!(
            status,
            "the firmware bundle is incomplete: 3 of 87 files are on the stick"
        );
    }

    #[test]
    fn status_names_a_bundle_as_complete_or_what_it_lacks() {
        let dir = tempfile::tempdir().unwrap();
        assert!(
            bundle_status(dir.path())
                .unwrap_err()
                .contains("no firmware bundle")
        );
        // A partial bundle: one real recipe's entry, its file not written.
        let first = bundle::complete()[0];
        let directory = dir.path().join(bundle::DIRECTORY);
        fs::create_dir_all(&directory).unwrap();
        fs::write(
            directory.join("bundle.json"),
            bundle::manifest(&[first]).unwrap(),
        )
        .unwrap();
        let status = bundle_status(dir.path()).unwrap_err();
        assert!(status.contains("lacks 86 of 87 files"), "{status}");
        assert!(
            status.contains("nvidia/ga102/gsp/gsp-570.144.bin"),
            "{status}"
        );
        assert!(status.contains("1 of its files are damaged"), "{status}");
    }

    #[test]
    fn an_older_per_pc_set_goes_once_the_bundle_holds_its_files() {
        let dir = tempfile::tempdir().unwrap();
        let esp = dir.path().join("esp");
        let volume = dir.path().join("volume");
        let old = esp.join(host_firmware::DIRECTORY);
        fs::create_dir_all(&old).unwrap();
        let image = recipes::REGULATORY[0].image;
        let manifest = serde_json::json!({"schema":1,"images":[{
            "name":"regulatory.db", "path":"0.bin",
            "target":{"driver":"wifi-regulatory","hardware":"wireless-regdb","revision":"v20","board":null},
            "provenance":{"source":image.source,"revision":image.revision,"member":image.member,"license":"x"},
            "length":image.length,"sha256":image.sha256}]});
        fs::write(
            old.join("manifest.json"),
            serde_json::to_vec(&manifest).unwrap(),
        )
        .unwrap();
        // A bundle without that file keeps the set.
        let files = bundle::complete();
        let without: Vec<_> = files
            .iter()
            .copied()
            .filter(|(object, _)| object.sha256 != image.sha256)
            .collect();
        remove_superseded(&esp, &volume, &without).unwrap();
        assert!(old.exists());
        // The same volume as the EFI partition keeps it too.
        remove_superseded(&esp, &esp, &files).unwrap();
        assert!(old.exists());
        remove_superseded(&esp, &volume, &files).unwrap();
        assert!(!old.exists());
    }

    #[test]
    fn cache_reuse_rechecks_exact_length_hash_and_symlink_identity() {
        let dir = tempfile::tempdir().unwrap();
        let path = dir.path().join("object");
        let artifact = Artifact {
            source: "https://vendor.invalid",
            revision: "synthetic",
            member: "one.bin",
            length: 3,
            sha256: "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
        };
        fs::write(&path, b"abc").unwrap();
        assert_eq!(read_cached(&path, &artifact).unwrap(), b"abc");
        for bytes in [b"abd".as_slice(), b"ab", b"abcd"] {
            fs::write(&path, bytes).unwrap();
            assert!(read_cached(&path, &artifact).is_err());
        }
        #[cfg(unix)]
        {
            let alias = dir.path().join("alias");
            std::os::unix::fs::symlink(&path, &alias).unwrap();
            assert!(read_cached(&alias, &artifact).is_err());
        }
    }
    #[test]
    fn dry_run_plans_pins_without_cache_or_destination_writes() {
        let dir = tempfile::tempdir().unwrap();
        let inventory = dir.path().join("requests.json");
        fs::write(&inventory, br#"{"schema":1,"complete":true,"requirements":[{"name":"rtl_nic/rtl8168d-1.fw","target":{"driver":"realtek","hardware":"10ec:8168","revision":"xid-281","board":null},"minimum_bytes":4}]}"#).unwrap();
        let cache = dir.path().join("cache");
        let esp = dir.path().join("esp");
        let text = pinned(&inventory)
            .plan(&esp, &host_firmware::MOUNT_CAPABILITY, &cache, true)
            .unwrap()
            .unwrap();
        assert!(text.contains("1492 bytes"));
        assert!(!cache.exists());
        assert!(!esp.exists());
    }

    /// An inventory file's requests as the files a boot asked for.
    fn pinned(inventory: &Path) -> Requested {
        let file = File::open(inventory).unwrap();
        let mut bytes = Vec::new();
        file.take(MAX_MANIFEST_BYTES as u64 + 1)
            .read_to_end(&mut bytes)
            .unwrap();
        Requested {
            pinned: Inventory::parse(&bytes).unwrap().requests().to_vec(),
            ..Requested::default()
        }
    }

    #[test]
    fn transport_rejects_redirect_truncation_growth_and_wrong_digest() {
        use std::{net::TcpListener, thread};
        for (response, valid) in [
            (
                "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nabc",
                true,
            ),
            (
                "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nab",
                false,
            ),
            ("HTTP/1.1 200 OK\r\nConnection: close\r\n\r\nabcd", false),
            (
                "HTTP/1.1 200 OK\r\nContent-Length: 4\r\nConnection: close\r\n\r\nabcd",
                false,
            ),
            (
                "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nabd",
                false,
            ),
            (
                "HTTP/1.1 302 Found\r\nLocation: https://vendor.invalid/alternate\r\nContent-Length: 0\r\nConnection: close\r\n\r\n",
                false,
            ),
        ] {
            let listener = TcpListener::bind("127.0.0.1:0").unwrap();
            let address = listener.local_addr().unwrap();
            let server = thread::spawn(move || {
                let (mut socket, _) = listener.accept().unwrap();
                socket
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let _ = socket.read(&mut [0; 4096]);
                let _ = socket.write_all(response.as_bytes());
            });
            let artifact = Artifact {
                source: Box::leak(format!("http://{address}").into_boxed_str()),
                revision: "synthetic-v1",
                member: "one.bin",
                length: 3,
                sha256: "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
            };
            let client = Client::builder()
                .timeout(Duration::from_secs(2))
                .redirect(Policy::none())
                .build()
                .unwrap();
            assert_eq!(
                download_from(&client, &artifact, &artifact.urls(), &mut |_| {}).is_ok(),
                valid
            );
            server.join().unwrap();
        }
    }

    /// A source on localhost that answers its requests with `responses` in
    /// order, then closes; its URL, and how many requests it served.
    fn source(responses: Vec<&'static str>) -> (String, std::thread::JoinHandle<usize>) {
        use std::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/one.bin", listener.local_addr().unwrap());
        let server = std::thread::spawn(move || {
            let mut served = 0;
            for response in responses {
                let (mut socket, _) = listener.accept().unwrap();
                socket
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let _ = socket.read(&mut [0; 4096]);
                let _ = socket.write_all(response.as_bytes());
                served += 1;
            }
            served
        });
        (url, server)
    }

    const ABC: Artifact = Artifact {
        source: "https://vendor.invalid",
        revision: "synthetic-v1",
        member: "one.bin",
        length: 3,
        sha256: "ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad",
    };
    const OK: &str = "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nabc";
    const HOURLY: &str = "HTTP/1.1 429 Too Many Requests\r\nRetry-After: 2789\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    const SHORT: &str = "HTTP/1.1 429 Too Many Requests\r\nRetry-After: 2\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    const DOWN: &str =
        "HTTP/1.1 503 Service Unavailable\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";
    const GONE: &str = "HTTP/1.1 404 Not Found\r\nContent-Length: 0\r\nConnection: close\r\n\r\n";

    fn client() -> Client {
        Client::builder()
            .timeout(Duration::from_secs(2))
            .redirect(Policy::none())
            .build()
            .unwrap()
    }

    #[test]
    fn an_hour_long_rate_limit_goes_to_the_next_source_at_once() {
        let (first, first_server) = source(vec![HOURLY]);
        let (second, second_server) = source(vec![OK]);
        let mut slept = Vec::new();
        let bytes =
            download_from(&client(), &ABC, &[first, second], &mut |d| slept.push(d)).unwrap();
        assert_eq!(bytes, b"abc");
        assert!(slept.is_empty(), "{slept:?}");
        assert_eq!(first_server.join().unwrap(), 1);
        assert_eq!(second_server.join().unwrap(), 1);
    }

    #[test]
    fn a_short_retry_after_is_waited_out_on_the_same_source() {
        let (first, server) = source(vec![SHORT, OK]);
        let mut slept = Vec::new();
        let bytes = download_from(&client(), &ABC, &[first], &mut |d| slept.push(d)).unwrap();
        assert_eq!(bytes, b"abc");
        assert_eq!(slept, [Duration::from_secs(2)]);
        assert_eq!(server.join().unwrap(), 2);
    }

    #[test]
    fn when_every_source_fails_the_error_names_the_file_and_each_answer() {
        let (first, first_server) = source(vec![DOWN, DOWN, DOWN]);
        let (second, second_server) = source(vec![GONE]);
        let mut slept = Vec::new();
        let error = download_from(&client(), &ABC, &[first, second], &mut |d| slept.push(d))
            .unwrap_err()
            .to_string();
        assert!(
            error.starts_with("cannot download one.bin: 127.0.0.1:"),
            "{error}"
        );
        assert!(
            error.contains("answered 503 Service Unavailable"),
            "{error}"
        );
        assert!(error.contains("answered 404 Not Found"), "{error}");
        // Two backoffs on the busy source, within the budget, then the next.
        assert_eq!(slept, [Duration::from_secs(1), Duration::from_secs(2)]);
        assert_eq!(first_server.join().unwrap(), 3);
        assert_eq!(second_server.join().unwrap(), 1);
        // A wrong file from the first source is never kept; the second's is.
        let (first, _) = source(vec![
            "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nabd",
        ]);
        let (second, _) = source(vec![OK]);
        assert_eq!(
            download_from(&client(), &ABC, &[first, second], &mut |_| {}).unwrap(),
            b"abc"
        );
    }

    /// A source on localhost that answers each request with the pieces of
    /// one of `responses`, `pause` apart, then closes; its URL, and the
    /// head of each request it served.
    fn scripted(
        responses: Vec<Vec<&'static str>>,
        pause: Duration,
    ) -> (String, std::thread::JoinHandle<Vec<String>>) {
        use std::net::TcpListener;
        let listener = TcpListener::bind("127.0.0.1:0").unwrap();
        let url = format!("http://{}/one.bin", listener.local_addr().unwrap());
        let server = std::thread::spawn(move || {
            let mut heads = Vec::new();
            for pieces in responses {
                let (mut socket, _) = listener.accept().unwrap();
                socket
                    .set_read_timeout(Some(Duration::from_secs(2)))
                    .unwrap();
                let mut head = [0; 4096];
                let n = socket.read(&mut head).unwrap_or(0);
                heads.push(String::from_utf8_lossy(&head[..n]).to_ascii_lowercase());
                for (index, piece) in pieces.iter().enumerate() {
                    if index > 0 {
                        std::thread::sleep(pause);
                    }
                    let _ = socket.write_all(piece.as_bytes());
                    let _ = socket.flush();
                }
            }
            heads
        });
        (url, server)
    }

    const CUT: &str = "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nab";
    const REST: &str = "HTTP/1.1 206 Partial Content\r\nContent-Range: bytes 2-2/3\r\nContent-Length: 1\r\nConnection: close\r\n\r\nc";

    #[test]
    fn a_cut_connection_goes_on_from_where_it_stopped() {
        let (url, server) = scripted(vec![vec![CUT], vec![REST]], Duration::ZERO);
        let mut slept = Vec::new();
        let bytes = download_from(&client(), &ABC, &[url], &mut |d| slept.push(d)).unwrap();
        assert_eq!(bytes, b"abc");
        assert_eq!(slept, [RESUME_PAUSE]);
        let heads = server.join().unwrap();
        assert!(!heads[0].contains("range:"), "{heads:?}");
        assert!(heads[1].contains("range: bytes=2-\r\n"), "{heads:?}");
    }

    #[test]
    fn a_server_without_ranges_sends_the_whole_file_again() {
        let (url, server) = scripted(vec![vec![CUT], vec![OK]], Duration::ZERO);
        let bytes = download_from(&client(), &ABC, &[url], &mut |_| {}).unwrap();
        assert_eq!(bytes, b"abc");
        assert!(server.join().unwrap()[1].contains("range: bytes=2-\r\n"));
        // A range other than the one asked for starts the file over.
        let (url, server) = scripted(
            vec![
                vec![CUT],
                vec![
                    "HTTP/1.1 206 Partial Content\r\nContent-Range: bytes 1-2/3\r\nContent-Length: 2\r\nConnection: close\r\n\r\nbc",
                ],
                vec![OK],
            ],
            Duration::ZERO,
        );
        let bytes = download_from(&client(), &ABC, &[url], &mut |_| {}).unwrap();
        assert_eq!(bytes, b"abc");
        assert!(!server.join().unwrap()[2].contains("range:"));
    }

    #[test]
    fn a_wrong_hash_is_refused_and_a_resumed_one_fetched_whole_again() {
        let (url, server) = scripted(
            vec![vec![
                "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\nabd",
            ]],
            Duration::ZERO,
        );
        let error = download_from(&client(), &ABC, &[url], &mut |_| {})
            .unwrap_err()
            .to_string();
        assert!(
            error.starts_with("cannot download one.bin: 127.0.0.1:"),
            "{error}"
        );
        assert!(error.contains("SHA-256 differs from the pin"), "{error}");
        assert_eq!(server.join().unwrap().len(), 1);
        // The resumed end does not join: the next request asks for the
        // whole file.
        let (url, server) = scripted(
            vec![
                vec![CUT],
                vec![
                    "HTTP/1.1 206 Partial Content\r\nContent-Range: bytes 2-2/3\r\nContent-Length: 1\r\nConnection: close\r\n\r\nd",
                ],
                vec![OK],
            ],
            Duration::ZERO,
        );
        let bytes = download_from(&client(), &ABC, &[url], &mut |_| {}).unwrap();
        assert_eq!(bytes, b"abc");
        assert!(!server.join().unwrap()[2].contains("range:"));
    }

    #[test]
    fn a_slow_transfer_outlasts_the_client_timeout_while_bytes_arrive() {
        let client = Client::builder()
            .timeout(Duration::from_millis(500))
            .redirect(Policy::none())
            .build()
            .unwrap();
        let (url, server) = scripted(
            vec![vec![
                "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\na",
                "b",
                "c",
            ]],
            Duration::from_millis(300),
        );
        let bytes = download_from(&client, &ABC, &[url], &mut |_| {}).unwrap();
        assert_eq!(bytes, b"abc");
        assert_eq!(server.join().unwrap().len(), 1);
    }

    #[test]
    fn a_transfer_that_never_progresses_says_why_it_failed() {
        let empty = "HTTP/1.1 200 OK\r\nContent-Length: 3\r\nConnection: close\r\n\r\n";
        let (url, server) = scripted(vec![vec![empty]; 3], Duration::ZERO);
        let mut slept = Vec::new();
        let error = download_from(&client(), &ABC, &[url], &mut |d| slept.push(d))
            .unwrap_err()
            .to_string();
        assert!(
            error.starts_with("cannot download one.bin: 127.0.0.1:"),
            "{error}"
        );
        assert!(
            error.contains("transfer cut at 0.0 of 0.0 MiB: request or response body error: "),
            "{error}"
        );
        assert!(!error.contains('\n'), "{error}");
        assert_eq!(slept, [Duration::from_secs(1), Duration::from_secs(2)]);
        assert_eq!(server.join().unwrap().len(), 3);
    }

    /// A boot log on `esp` whose boots print `lines`.
    fn boot_log(esp: &Path, boots: &[&str]) {
        fs::create_dir_all(esp.join("EFI/stela")).unwrap();
        fs::write(
            esp.join(crate::efi::BOOT_LOG),
            crate::boot_log::tests::region(boots),
        )
        .unwrap();
    }

    /// One inventory line as stela.efi prints it, of these requests.
    fn requests_line(requests: &[serde_json::Value]) -> String {
        format!(
            "[    3.250] {}{}\n",
            requirements::LOG_LINE,
            serde_json::json!({"schema":1,"complete":true,"requirements":requests})
        )
    }

    fn realtek(name: &str, xid: &str) -> serde_json::Value {
        serde_json::json!({"name":name,"target":{"driver":"realtek","hardware":"10ec:8168",
            "revision":xid,"board":null},"minimum_bytes":4})
    }

    #[test]
    fn the_boot_log_s_newest_requests_split_into_what_setup_adds_and_names() {
        let dir = tempfile::tempdir().unwrap();
        let esp = dir.path().join("esp");
        let destination = esp.join(host_firmware::DIRECTORY);
        fs::create_dir_all(&destination).unwrap();
        // The USB holds one Realtek file already.
        let manifest = serde_json::json!({"schema":1,"images":[{
            "name":"rtl_nic/rtl8168e-3.fw", "path":"rtl/one.bin",
            "target":{"driver":"realtek","hardware":"10ec:8168","revision":"xid-2c8","board":null},
            "provenance":{"source":"synthetic-fixture","revision":"v1","member":"one.bin","license":"CC0-1.0"},
            "length":3,"sha256":"ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad"}]});
        fs::write(
            destination.join("manifest.json"),
            serde_json::to_vec(&manifest).unwrap(),
        )
        .unwrap();
        // A stela.efi before the bundle, which reads exact per-PC files.
        let mut image = host_firmware::MOUNT_CAPABILITY;
        image[20] = 3;
        // No log, or a log without requests: nothing.
        assert!(requested(&esp, &image).unwrap().is_empty());
        boot_log(&esp, &["CHOICE 1 (macOS)\n"]);
        assert!(requested(&esp, &image).unwrap().is_empty());
        let unpinned = serde_json::json!({"name":"brcm/synthetic.bin","target":{
            "driver":"synthetic-bluetooth","hardware":"pci:14e4:5fa0","revision":"c0",
            "board":null},"minimum_bytes":1});
        let variable = serde_json::json!({"name":"intel/wifi/uefi/UefiCnvWlanUATS.bin",
            "target":{"driver":"wifi-intel","hardware":"uefi:92daaf2f-c02b-455b-b2ec-f5a3594f4aea",
            "revision":"CnvUefiWlanUATS","board":null},"minimum_bytes":1});
        let older = requests_line(&[realtek("rtl_nic/rtl8168f-1.fw", "xid-480")]);
        let newer = requests_line(&[
            realtek("rtl_nic/rtl8168e-3.fw", "xid-2c8"),
            realtek("rtl_nic/rtl8168d-1.fw", "xid-281"),
            unpinned,
            variable,
        ]);
        boot_log(
            &esp,
            &[&older, &format!("USB added 1-4\n{newer}HOST WARNING x\n")],
        );
        let found = requested(&esp, &image).unwrap();
        let names = |requests: &[Request]| -> Vec<String> {
            requests
                .iter()
                .map(|request| request.name.clone())
                .collect()
        };
        assert_eq!(names(&found.pinned), ["rtl_nic/rtl8168d-1.fw"]);
        assert_eq!(names(&found.unpinned), ["brcm/synthetic.bin"]);
        assert!(found.unreadable.is_none());
        // A stela.efi that reads the bundle needs no file added: the bundle
        // holds every pinned one. The unpinned stay named.
        let bundled = requested(&esp, &host_firmware::MOUNT_CAPABILITY).unwrap();
        assert!(bundled.pinned.is_empty());
        assert_eq!(names(&bundled.unpinned), ["brcm/synthetic.bin"]);
        let lines = found.lines().join("\n");
        assert!(
            lines.contains("add rtl_nic/rtl8168d-1.fw (realtek 10ec:8168 xid-281)"),
            "{lines}"
        );
        assert!(
            lines.contains("brcm/synthetic.bin (synthetic-bluetooth pci:14e4:5fa0 c0), which Stela has no pinned source for"),
            "{lines}"
        );
        // The plan keeps what the stick holds and downloads nothing in a dry run.
        let cache = dir.path().join("cache");
        let plan = found.plan(&esp, &image, &cache, true).unwrap().unwrap();
        assert!(plan.contains("2 exact observed bindings"), "{plan}");
        assert!(plan.contains("rtl_nic/rtl8168e-3.fw"), "{plan}");
        assert!(plan.contains("rtl_nic/rtl8168d-1.fw"), "{plan}");
        assert!(plan.contains("Dry run"), "{plan}");
        assert!(!cache.exists());
        // Unpinned requests alone plan nothing.
        let only = Requested {
            pinned: Vec::new(),
            ..found
        };
        assert!(only.plan(&esp, &image, &cache, true).unwrap().is_none());
        only.provision(&esp, &image).unwrap();
        // A boot program that reads no firmware from the stick asked for none.
        assert!(requested(&esp, b"no marker").unwrap().is_empty());
        // A line cut short is named, not followed.
        boot_log(&esp, &[&newer[..60]]);
        let cut = requested(&esp, &image).unwrap();
        assert!(cut.pinned.is_empty() && cut.unreadable.is_some(), "{cut:?}");
    }

    #[test]
    #[ignore = "oracle: STELA_FIRMWARE_TEST_OUTPUT"]
    fn vendor_provisioning_binds_pins_and_notices() {
        let output = std::path::PathBuf::from(
            std::env::var_os("STELA_FIRMWARE_TEST_OUTPUT")
                .expect("explicit fresh evidence directory"),
        );
        assert!(!output.exists(), "never overwrite historical evidence");
        fs::create_dir_all(&output).unwrap();
        let requests = [
            ("rtl_nic/rtl8168d-1.fw", "xid-281"), ("rtl_nic/rtl8168d-2.fw", "xid-280"),
            ("rtl_nic/rtl8168e-1.fw", "xid-2c1"), ("rtl_nic/rtl8168e-2.fw", "xid-2c0"),
            ("rtl_nic/rtl8168e-3.fw", "xid-2c8"), ("rtl_nic/rtl8168f-1.fw", "xid-480"),
            ("rtl_nic/rtl8168f-2.fw", "xid-481"), ("rtl_nic/rtl8411-1.fw", "xid-488"),
            ("rtl_nic/rtl8168g-2.fw", "xid-4c0"), ("rtl_nic/rtl8168g-3.fw", "xid-509"),
            ("rtl_nic/rtl8411-2.fw", "xid-5c8"), ("rtl_nic/rtl8168h-2.fw", "xid-541"),
            ("rtl_nic/rtl8168fp-3.fw", "xid-54a"), ("rtl_nic/rtl8125a-3.fw", "xid-609"),
            ("rtl_nic/rtl8125b-2.fw", "xid-641"), ("rtl_nic/rtl8125d-1.fw", "xid-688"),
            ("rtl_nic/rtl8125d-2.fw", "xid-689"), ("rtl_nic/rtl8125k-1.fw", "xid-68a"),
            ("rtl_nic/rtl9151a-1.fw", "xid-68b"), ("rtl_nic/rtl8125cp-1.fw", "xid-708"),
            ("rtl_nic/rtl8125bp-2.fw", "xid-681"), ("rtl_nic/rtl8126a-2.fw", "xid-649"),
            ("rtl_nic/rtl8126a-3.fw", "xid-64a"),
        ].map(|(name, revision)| serde_json::json!({"name":name,"target":{"driver":"realtek","hardware":"10ec:8168","revision":revision,"board":null},"minimum_bytes":4}));
        let inventory = output.join("synthetic-observed-requirements.json");
        fs::write(
            &inventory,
            serde_json::to_vec_pretty(
                &serde_json::json!({"schema":1,"complete":true,"requirements":requests}),
            )
            .unwrap(),
        )
        .unwrap();
        let esp = output.join("esp");
        let destination = esp.join(host_firmware::DIRECTORY);
        let cache = output.join("cache");
        let requested = pinned(&inventory);
        let report = requested
            .plan(&esp, &host_firmware::MOUNT_CAPABILITY, &cache, false)
            .unwrap()
            .unwrap();
        fs::write(output.join("provisioning.txt"), report).unwrap();
        let manifest_bytes = fs::read(destination.join("manifest.json")).unwrap();
        let manifest = host_firmware::Manifest::parse(&manifest_bytes).unwrap();
        let mut observed = Inventory::default();
        for request in &requested.pinned {
            observed.observe(request.view()).unwrap();
        }
        assert_eq!(manifest.images.len(), 23);
        for (image, request) in manifest.images.iter().zip(observed.requests()) {
            assert_eq!(image.name, request.name);
            assert_eq!(image.target.revision, request.target.revision);
            assert_eq!(image.target.board, request.target.board.as_deref());
            let bytes = fs::read(destination.join(image.path)).unwrap();
            let recipe = Plan::select(&observed).unwrap();
            recipe
                .selections()
                .iter()
                .find(|selected| selected.request.name == image.name)
                .unwrap()
                .recipe
                .image
                .verify(&bytes)
                .unwrap();
        }
        let objects = destination
            .join(manifest.images[0].path)
            .parent()
            .unwrap()
            .to_path_buf();
        let notice = fs::read(objects.join("license-0.txt")).unwrap();
        assert_eq!(notice.len(), 219);
        assert!(
            String::from_utf8(notice)
                .unwrap()
                .contains("Realtek Semiconductor Corporation")
        );
        assert!(objects.join("provenance.json").is_file());
        assert_eq!(
            host_firmware::Package::read(&destination)
                .unwrap()
                .inventory(),
            (23, 115_404)
        );
        let again = requested
            .plan(&esp, &host_firmware::MOUNT_CAPABILITY, &cache, false)
            .unwrap()
            .unwrap();
        assert!(again.contains("Published pinned firmware"));
        assert_eq!(
            fs::read(destination.join("manifest.json")).unwrap(),
            manifest_bytes
        );
    }
}
